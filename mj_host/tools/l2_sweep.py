#!/usr/bin/env python3
"""Seed-averaged A/B for the duck's LEVEL 2 -- the harness the rung-2 verdicts lacked.

Every §17 table in docs/plans-and-designs/microduck_rung2_regime_design.md was read at seed 2
only (CLAUDE.md §3 rule 3, §3.7: a single seed is a signal, not a finding).  This runs each
level-2 config from scratch over N seeds, in parallel, and reports the §17 metrics over the
CONTROL phase (after the identification babble), paired against the first config by seed.

Usage:
  python3 mj_host/tools/l2_sweep.py CFG.json [CFG2.json ...] [--seeds 6] [--secs 1500]
      [--control-from S] [--jobs J] [--host-args '--wander-bored 8 --wander-turn 90']
      [--arm name:module.param=value[,module.param=value]]* [--logdir DIR]

  --control-from  start of the judged window in seconds; default = the config's
                  motor_epm_intent.babble_ticks / 50 Hz + 100 s (R23 judged 700–1500 s of a
                  1500 s run with babble_ticks 30000)
  --arm           a single-lever arm derived from the FIRST config (mkarm-style, refuses no-ops)

Per arm and seed, from the host's own JSONL (stdout) and summary (stderr):
  walls/min   wall-contact EPISODES per minute (0→1 transitions of `wall`)  -- the §17.2 headline
  contact%    ticks in contact
  tooclose    mean TooClose fraction (tofs[3])
  path        metres travelled
  cells       distinct 0.25 m cells visited
  span        x-range × y-range (m)
  nodes       distinct map winners (the map EPM's live vocabulary)
  mapTLE      mean map TLE;  novel%  fraction of ticks the map called novel
  turns       the host's wander heading changes (stderr), if the wander rule is on
  rescues/min, walker-driven %, and the identified A rows (read-backs: a silent-confound arm
  cannot happen quietly, §3.2 rule 7).
Blind-metric complements (CLAUDE.md §3 rule 4): cells vs walls/min (an orbit scores 0 walls
and 9 cells; a wall-rider scores many cells and hundreds of walls), path vs span.
"""
import argparse, concurrent.futures, json, math, os, re, statistics, subprocess, sys, tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HOST = REPO / "mj_host/build/ogma_mjhost"
CELL_M = 0.25


def _coerce(v):
    try: return json.loads(v)
    except Exception: return v


def make_arm(base_cfg: Path, spec: str, outdir: Path) -> Path:
    """name:module.param=value,... -> a temp config differing from base by exactly those params."""
    name, _, ov = spec.partition(":")
    cfg = json.load(open(base_cfg))
    changes = []
    for kv in ov.split(",") if ov else []:
        k, _, v = kv.partition("=")
        mod, _, param = k.strip().partition(".")
        val = _coerce(v.strip())
        hit = [m for m in cfg["modules"] if m.get("id") == mod]
        if len(hit) != 1:
            sys.exit(f"arm {name}: module id {mod!r} not found once in {base_cfg.name}")
        p = hit[0].setdefault("params", {})
        if p.get(param) == val:
            sys.exit(f"arm {name}: {mod}.{param} already {val!r} -- a TAUTOLOGY, not an arm")
        changes.append(f"{mod}.{param}: {p.get(param)!r} -> {val!r}")
        p[param] = val
    cfg.setdefault("metadata", {})["name"] = f"ARM {name} (base {base_cfg.stem}): " + "; ".join(changes)
    out = outdir / f"_l2arm_{name}.json"
    json.dump(cfg, open(out, "w"), indent=1)
    print(f"[arm] {name}: " + "; ".join(changes), file=sys.stderr)
    return out


def control_from_default(cfg: Path) -> float:
    d = json.load(open(cfg))
    for m in d.get("modules", []):
        bt = m.get("params", {}).get("babble_ticks")
        if bt is not None:
            return float(bt) / 50.0 + 100.0
    return 0.0


def run_one(cfg: Path, seed: int, secs: int, control_from: float, host_args: tuple, logdir: Path | None,
            scene: str = "", noise: float = 0.0, phase_at: float | None = None) -> dict:
    cmd = [str(HOST), "--level2", *([scene] if scene else []), "--graph", str(cfg), "--secs", str(secs), "--seed", str(seed),
           "--noise", str(noise), *host_args]
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=7200, cwd=str(REPO / "mj_host"))
    if logdir is not None:
        # a compact stream: the fields the metrics read (a full level-2 JSONL carries qpos and the
        # 64 ToF zones per tick -- ~75 MB per 1500 s run, which filled a tmpfs quota on first use)
        logdir.mkdir(parents=True, exist_ok=True)
        keep = ("t", "x", "y", "z", "tilt", "drive", "wall", "tofs", "map")
        with open(logdir / f"{cfg.stem}_s{seed}.jsonl", "w") as f:
            for line in p.stdout.splitlines():
                if not line.startswith("{"): continue
                try: row = json.loads(line)
                except ValueError: continue
                f.write(json.dumps({k: row[k] for k in keep if k in row}) + "\n")
        (logdir / f"{cfg.stem}_s{seed}.stderr").write_text(p.stderr)
    err = p.stderr
    out = {"seed": seed, "rc": p.returncode}
    m = re.search(r"level-2 [\d.]+ s — (\d+) rescues, (\d+)% of the run walker-driven", err)
    out["rescues_min"] = int(m.group(1)) * 60.0 / secs if m else float("nan")
    out["driven_pct"] = float(m.group(2)) if m else float("nan")
    m = re.search(r"wander: (\d+) heading changes", err)
    out["turns"] = int(m.group(1)) if m else None
    out["readback"] = " | ".join(l.strip() for l in err.splitlines() if re.match(r"\s+(vx|vy|vyaw)\s+:", l))
    # ---- JSONL over the control phase (and, with --phase-at, the two halves around a perturbation)
    cells, xs, ys, winners = set(), [], [], set()
    path = 0.0; prev = None
    wall_eps = contact = n = 0; prev_wall = 0
    tooclose = tle_sum = 0.0; novel = 0
    ph = {"before": {"cells": set(), "walls": 0, "n": 0, "prev_wall": 0, "nodes": set()},
          "after":  {"cells": set(), "walls": 0, "n": 0, "prev_wall": 0, "nodes": set()}}
    for line in p.stdout.splitlines():
        if not line.startswith("{"): continue
        try: r = json.loads(line)
        except ValueError: continue
        t = float(r.get("t", 0.0))
        if t < control_from: continue
        if phase_at is not None:
            g = ph["before" if t < phase_at else "after"]
            g["n"] += 1; g["cells"].add((math.floor(float(r["x"]) / CELL_M), math.floor(float(r["y"]) / CELL_M)))
            gw = int(r.get("wall", 0)); g["walls"] += (gw and not g["prev_wall"]); g["prev_wall"] = gw
            mp0 = r.get("map") or []
            if len(mp0) >= 3: g["nodes"].add(int(mp0[2]))
        n += 1
        x, y = float(r["x"]), float(r["y"])
        xs.append(x); ys.append(y)
        cells.add((math.floor(x / CELL_M), math.floor(y / CELL_M)))
        if prev is not None: path += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
        w = int(r.get("wall", 0)); contact += w
        if w and not prev_wall: wall_eps += 1
        prev_wall = w
        tofs = r.get("tofs") or [0, 0, 0, 0]; tooclose += float(tofs[3]) if len(tofs) > 3 else 0.0
        mp = r.get("map") or []
        if len(mp) >= 3:
            tle_sum += float(mp[0]); novel += int(mp[1]); winners.add(int(mp[2]))
    minutes = max(1e-9, (secs - control_from) / 60.0)
    if phase_at is not None:
        mb = max(1e-9, (phase_at - control_from) / 60.0); ma = max(1e-9, (secs - phase_at) / 60.0)
        out["cells_before"] = len(ph["before"]["cells"]); out["cells_after"] = len(ph["after"]["cells"])
        out["walls_before"] = ph["before"]["walls"] / mb; out["walls_after"] = ph["after"]["walls"] / ma
        out["nodes_before"] = len(ph["before"]["nodes"]); out["nodes_after"] = len(ph["after"]["nodes"])
    out.update({
        "samples": n, "walls_min": wall_eps / minutes, "contact_pct": 100.0 * contact / max(1, n),
        "tooclose": tooclose / max(1, n), "path_m": path, "cells": len(cells),
        "span": (max(xs) - min(xs)) * (max(ys) - min(ys)) if xs else 0.0,
        "nodes": len(winners), "map_tle": tle_sum / max(1, n), "novel_pct": 100.0 * novel / max(1, n),
    })
    return out


def fmt(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals: return "     -      "
    return f"{statistics.mean(vals):7.2f}±{statistics.stdev(vals):5.2f}" if len(vals) > 1 else f"{vals[0]:7.2f}      "


def paired(a_rows, b_rows, key, better):
    a = {r["seed"]: r for r in a_rows}; b = {r["seed"]: r for r in b_rows}
    d = [b[s][key] - a[s][key] for s in sorted(set(a) & set(b))
         if a[s].get(key) is not None and b[s].get(key) is not None and not math.isnan(a[s][key]) and not math.isnan(b[s][key])]
    if len(d) < 2: return "n<2"
    sd = statistics.stdev(d); t = statistics.mean(d) / (sd / math.sqrt(len(d))) if sd else float("inf")
    return f"Δ {statistics.mean(d):+8.2f}  sd {sd:6.2f}  t {t:+5.2f}  sign {sum(v>0 for v in d)}+/{sum(v<0 for v in d)}−  n={len(d)}  ({better} better)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("configs", nargs="+")
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--secs", type=int, default=1500)
    ap.add_argument("--control-from", type=float, default=None)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 2))
    ap.add_argument("--host-args", default="")
    ap.add_argument("--arm", action="append", default=[])
    ap.add_argument("--logdir", default=None)
    ap.add_argument("--scene", default=str(REPO / "mj_host/models/microduck/scene_arena.xml"),
                    help="the level-2 scene (default: the 2 m arena -- the host's own default is the OPEN floor, where 'zero wall contacts' means no walls)")
    ap.add_argument("--noise", type=float, default=0.05, help="reset noise on the start pose, so seeds vary the start and not only the babble (host --noise)")
    ap.add_argument("--phase-at", type=float, default=None, help="split the control phase at this second (e.g. the --arena-shift time) and report cells / walls / nodes before and after -- the (d) reading")
    args = ap.parse_args()
    if not HOST.exists(): sys.exit(f"host binary missing: {HOST} (build with ./mj_host/run.sh build)")
    logdir = Path(args.logdir) if args.logdir else None
    tmp = Path(tempfile.mkdtemp(prefix="l2sweep_"))
    cfgs = [Path(c).resolve() for c in args.configs]
    cfgs += [make_arm(cfgs[0], spec, tmp) for spec in args.arm]
    ctrl = args.control_from if args.control_from is not None else control_from_default(cfgs[0])
    host_args = tuple(args.host_args.split())
    print(f"level-2 sweep: {len(cfgs)} arms × {args.seeds} seeds × {args.secs} s, control phase from {ctrl:.0f} s, scene {Path(args.scene).name}, reset noise {args.noise}, host args {host_args or '-'}", file=sys.stderr)

    jobs = [(c, s) for c in cfgs for s in range(1, args.seeds + 1)]
    results = {c: [] for c in cfgs}
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futs = {ex.submit(run_one, c, s, args.secs, ctrl, host_args, logdir, args.scene, args.noise, args.phase_at): (c, s) for c, s in jobs}
        for fut in concurrent.futures.as_completed(futs):
            c, s = futs[fut]; r = fut.result(); results[c].append(r)
            print(f"  {c.stem:34s} seed {s}: walls {r['walls_min']:6.1f}/min  cells {r['cells']:3d}  path {r['path_m']:6.1f} m  "
                  f"nodes {r['nodes']:3d}  mapTLE {r['map_tle']:.3f}  rescues {r['rescues_min']:.1f}/min", file=sys.stderr)

    print(f"\n=== LEVEL-2 A/B, {args.seeds} seeds × {args.secs} s, control phase {ctrl:.0f}–{args.secs} s ===")
    keys = [("walls_min", "walls/min"), ("contact_pct", "contact%"), ("tooclose", "tooclose"), ("path_m", "path m"),
            ("cells", "cells"), ("span", "span m²"), ("nodes", "nodes"), ("map_tle", "mapTLE"), ("novel_pct", "novel%"),
            ("turns", "turns"), ("rescues_min", "resc/min"), ("driven_pct", "driven%")]
    print(f"{'arm':34s} " + " ".join(f"{lbl:>13s}" for _, lbl in keys))
    for c in cfgs:
        rows = sorted(results[c], key=lambda r: r["seed"])
        print(f"{c.stem:34s} " + " ".join(f"{fmt([r.get(k) for r in rows]):>13s}" for k, _ in keys))
    if len(cfgs) > 1:
        ref = cfgs[0]
        for c in cfgs[1:]:
            print(f"\n  PAIRED {c.stem} − {ref.stem}:")
            for k, better in (("walls_min", "lower"), ("cells", "higher"), ("path_m", "-"), ("nodes", "higher"), ("map_tle", "-"), ("rescues_min", "lower")):
                print(f"    {k:12s} {paired(results[ref], results[c], k, better)}")
    if args.phase_at is not None:
        print(f"\n  (d) split at {args.phase_at:.0f} s -- before | after:")
        for c in cfgs:
            rows = sorted(results[c], key=lambda r: r["seed"])
            print(f"    {c.stem:34s} cells {fmt([r['cells_before'] for r in rows])} | {fmt([r['cells_after'] for r in rows])}   "
                  f"walls/min {fmt([r['walls_before'] for r in rows])} | {fmt([r['walls_after'] for r in rows])}   "
                  f"nodes {fmt([r['nodes_before'] for r in rows])} | {fmt([r['nodes_after'] for r in rows])}")
    print("\nread-backs (identified A rows, first seed):")
    for c in cfgs:
        rows = sorted(results[c], key=lambda r: r["seed"])
        print(f"  {c.stem:34s} {rows[0]['readback'] if rows else '-'}")


if __name__ == "__main__":
    main()
