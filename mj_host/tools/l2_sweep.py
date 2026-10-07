#!/usr/bin/env python3
"""Seed-averaged A/B for the duck's LEVEL 2 -- the harness the rung-2 verdicts lacked.

Every §17 table in docs/plans-and-designs/microduck_rung2_regime_design.md was read at seed 2
only (CLAUDE.md §3 rule 3, §3.7: a single seed is a signal, not a finding).  This runs each
level-2 config from scratch over N seeds, in parallel, and reports the §17 metrics over the
CONTROL phase (after the identification babble), paired against the first config by seed.

Usage:
  python3 mj_host/tools/l2_sweep.py CFG.json [CFG2.json ...] [--seeds 6] [--secs 1500]
      [--control-from S] [--jobs J] [--host-args '--wander-bored 8 --wander-turn 90']
      [--arm name:module.param=value[,module.param=value]]* [--logdir DIR [--full-logs]]

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
  headW rms   the head loop (--head-graph): RMS of the head gyro over all three axes (rad/s) while upright --
              the head's WORLD motion, which a head locked to the trunk scores worst on (before 2026-09-10
              night it summed x, y only; x is the head's yaw); headG dev = RMS of the head
              gravity's roll and pitch components (the IMU's x axis points down when level; 0 = level).
              nan without a head graph.
  down%       ticks with the trunk past 60 deg of tilt -- on the floor, whether or not a rescue is running
              (a body wedged on a table leg shows here and nowhere else; the seeds' means hide a single 40 % run
              unless you read the per-seed lines)
  span        x-range × y-range (m)
  nodes       distinct map winners (the map EPM's live vocabulary)
  switch/min  winner switches per minute -- how often the map's 'where am I' changes (R37: 130-155 on the duck)
  straight    median over 20 s windows of net displacement / path length (1 = a line, 0 = an orbit) -- the
              complement cells and path are blind to
  mapTLE      mean map TLE;  novel%  fraction of ticks the map called novel
  turns       the host's wander heading changes (stderr), if the wander rule is on
  rescues/min, walker-driven %, and the identified A rows (read-backs: a silent-confound arm
  cannot happen quietly, §3.2 rule 7).
  W0, the ten-minute instrument (playroom plan §12.6): the BEHAVIOUR HISTOGRAM over the control phase --
  walk%   ticks walking under the twist brain      stopW%  ticks in a scheduled stop held by the walker
  stand%  ticks the joint brain owns the legs      resc%   ticks under the rescue scaffold
  and the stop counters from the host's summary: stops, handbacks (the legs given to the joint brain),
  survived (held to the stop's end), handoffs (given back on lean), refused (the gate), stopResc (a fall).
  Its blind metric is variety; the complement is contingency (nothing here fires without an event).
  --host-arm NAME:'ARGS' adds an arm that is the FIRST config with those host args appended (the lever
  lives in the host, not the graph: R39's stops).
Blind-metric complements (CLAUDE.md §3 rule 4): cells vs walls/min (an orbit scores 0 walls
and 9 cells; a wall-rider scores many cells and hundreds of walls), path vs span.
"""
import argparse, concurrent.futures, json, math, os, re, statistics, subprocess, sys, tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HOST = REPO / "mj_host/build/ogma_mjhost"
CELL_M = 0.25
FULL_LOGS = False          # --full-logs


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


class Arm:
    """A config plus the host args that make it an arm; `stem` names it in the tables."""
    def __init__(self, cfg: Path, label: str | None = None, extra: tuple = ()):
        self.cfg, self.extra = cfg, tuple(extra)
        self.stem = label or cfg.stem
    def __hash__(self): return hash((self.cfg, self.stem, self.extra))
    def __eq__(self, o): return (self.cfg, self.stem, self.extra) == (o.cfg, o.stem, o.extra)


def run_one(arm, seed: int, secs: int, control_from: float, host_args: tuple, logdir: Path | None,
            scene: str = "", noise: float = 0.0, phase_at: float | None = None, arena_half: float = 1e9) -> dict:
    cfg = arm.cfg if isinstance(arm, Arm) else Path(arm)
    extra = arm.extra if isinstance(arm, Arm) else ()
    stem = arm.stem if isinstance(arm, Arm) else cfg.stem
    cmd = [str(HOST), "--level2", *([scene] if scene else []), "--graph", str(cfg), "--secs", str(secs), "--seed", str(seed),
           "--noise", str(noise), *host_args, *extra]
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=7200, cwd=str(REPO / "mj_host"))
    if logdir is not None:
        # a compact stream: the fields the metrics read (a full level-2 JSONL carries qpos and the
        # 64 ToF zones per tick -- ~75 MB per 1500 s run, which filled a tmpfs quota on first use)
        logdir.mkdir(parents=True, exist_ok=True)
        keep = ("t", "x", "y", "z", "tilt", "drive", "wall", "obj", "tofs", "map", "hdg", "twist", "hg", "hw", "head", "stop", "satt", "event", "scan")
        with open(logdir / f"{stem}_s{seed}.jsonl", "w") as f:
            if FULL_LOGS:                     # --full-logs: the host's stdout verbatim (cloud records, qpos, ToF zones)
                f.write(p.stdout)
            for line in ([] if FULL_LOGS else p.stdout.splitlines()):
                if not line.startswith("{"): continue
                try: row = json.loads(line)
                except ValueError: continue
                f.write(json.dumps({k: row[k] for k in keep if k in row}) + "\n")
        (logdir / f"{stem}_s{seed}.stderr").write_text(p.stderr)
    err = p.stderr
    out = {"seed": seed, "rc": p.returncode}
    if p.returncode != 0 or not any(l.startswith("{") for l in p.stdout.splitlines()):
        # §3.2 rule 7: a run that did not happen must not print as a row of zeros
        tail = " | ".join(p.stderr.strip().splitlines()[-2:])
        print(f"  !! {stem} seed {seed}: host rc {p.returncode}, no JSONL — {tail}", file=sys.stderr)
    m = re.search(r"level-2 [\d.]+ s — (\d+) rescues, (\d+)% of the run walker-driven", err)
    out["rescues_min"] = int(m.group(1)) * 60.0 / secs if m else float("nan")
    out["driven_pct"] = float(m.group(2)) if m else float("nan")
    m = re.search(r"wander: (\d+) heading changes", err)
    out["turns"] = int(m.group(1)) if m else None
    m = re.search(r"stops: (\d+) started, (\d+) hand-backs, (\d+) refused by the gate, (\d+) survived to the end, "
                  r"(\d+) handed back to the walker, (\d+) rescued", err)
    for k, v in zip(("stops", "handbacks", "refused", "survived", "handoffs", "stop_resc"),
                    (int(x) for x in m.groups()) if m else [None] * 6):
        out[k] = v
    out["survive_pct"] = (100.0 * out["survived"] / out["handbacks"]) if m and out["handbacks"] else None
    m = re.search(r"arrival stops: (\d+) of (\d+) started when the seek loop reached its target", err)
    out["stops_arrive"] = int(m.group(1)) if m else None
    m = re.search(r"stuck stops: (\d+) of (\d+) started when a forward stall", err)
    m2 = re.search(r"; (\d+) escapes", err)
    out["escapes"] = int(m2.group(1)) if m2 else None
    m3 = re.search(r"spins: (\d+) detected .*?; (\d+) \w+ fired", err)
    out["spins"] = int(m3.group(1)) if m3 else None
    out["spin_rolls"] = int(m3.group(2)) if m3 else None
    out["stops_stuck"] = int(m.group(1)) if m else None
    m = re.search(r"look: (\d+) saccades, (\d+) holds extended by novelty, (\d+) of (\d+) stops ended by a quiet round; mean stop ([\d.]+) s", err)
    out["saccades"] = int(m.group(1)) if m else None
    out["novel_holds"] = int(m.group(2)) if m else None
    out["bored_pct"] = (100.0 * int(m.group(3)) / max(1, int(m.group(4)))) if m else None
    out["stop_len"] = float(m.group(5)) if m else None
    m = re.search(r"orient: (\d+) rolls \((\d+) skipped at a surface\), (\d+) changes \((\d+) within 3 s of a roll, (\d+) unprompted\), (\d+) orientations, (\d+) arrivals, (\d+) timeouts, mean reach ([\d.]+) m", err)
    for k, v in zip(("rolls", "rolls_skipped", "changes", "prompted", "unprompted", "orientations", "arrivals", "orient_timeouts", "reach_m"),
                    ([int(x) for x in m.groups()[:8]] + [float(m.group(9))]) if m else [None] * 9):
        out[k] = v
    out["detect_pct"] = (100.0 * out["prompted"] / out["rolls"]) if m and out["rolls"] else None
    m = re.search(r"map baking: inserted (\d+) at stops / (\d+) on walks, baked (\d+) at stops / (\d+) on walks, pruned (\d+) \(of which baked (\d+)\); (\d+) nodes, (\d+) baked at the end", err)
    for k, v in zip(("ins_stop", "ins_walk", "bake_stop", "bake_walk", "pruned", "pruned_baked", "nodes_end", "baked_end"),
                    (int(x) for x in m.groups()) if m else [None] * 8):
        out[k] = v
    m = re.search(r"cloud: (\d+) stops accumulated, mean (\d+) voxels of which (\d+) break the floor", err)
    out["cloud_vox"] = int(m.group(2)) if m else None
    out["cloud_brk"] = int(m.group(3)) if m else None
    out["readback"] = " | ".join(l.strip() for l in err.splitlines() if re.match(r"\s+(vx|vy|vyaw)\s+:|place vector:|\s+play \{", l))
    # ---- JSONL over the control phase (and, with --phase-at, the two halves around a perturbation)
    cells, xs, ys, winners = set(), [], [], set()
    path = 0.0; prev = None
    switches = 0; prev_w = None          # winner switches: how often the map's 'where am I' changes (R37)
    win_pts = []                         # (x, y) per tick for the 20 s straightness windows
    wall_eps = contact = n = 0; prev_wall = 0
    obj_eps = 0; prev_obj = 0
    down = 0      # ticks with the trunk past 60 deg of tilt: on the floor, rescued or not (the table-leg trap, 2026-09-10)
    # the head loop (H2): head gyro RMS (x, y — the world motion of the head, whatever the joints do),
    # head-gravity deviation from vertical, and the trunk's own pitch/roll rate for scale
    hw2 = hg2 = tw2 = 0.0; nh = 0
    # the playroom's manifest names every movable's qpos address: displacement is read from
    # the JSONL's qpos, the same numbers the viewer draws
    layout = []
    man = Path(scene).with_suffix(".manifest.json") if scene else None
    if man and man.exists():
        layout = [(nm, adr) for nm, adr, nn in json.load(open(man)).get("qpos_layout", []) if nn == 7]
    obj_start = {}; obj_end = {}
    tooclose = tle_sum = 0.0; novel = 0; escaped = 0; steer_avoid = steer_play = 0
    steer_seek = seek_held = seek_ends = 0; prev_seek_value = 0.0
    hist = {"walk": 0, "stopW": 0, "stand": 0, "resc": 0}       # W0: the behaviour histogram
    yaw_stop, yaw_walk = [], []                                    # W2: the head-yaw joint (policy index 7) at stops vs walking
    # W5 (the twist brain's yaw channel, design doc §17.17): the instruments that measured it NOT
    # regulating -- the heading error it is held to, how hard it commands, and how often it changes
    # its mind.  Walking ticks only (a stop zeroes the twist; a rescue is not the brain's).
    hdg_err = 0.0; vyaw_abs = 0.0; vx_cmd = 0.0; nyaw = 0; yaw_flips = 0; prev_vyaw = None
    # refFollow (2026-09-12, W5): how many radians the heading REFERENCE moves per radian the
    # body turns, over 0.5 s steps where the reference did not jump.  ~0 = a target in the world
    # (turning closes the error); ~1 = a target that turns with the body, which turning cannot
    # close.  Measured +0.50 on the W3c stack: half of every correction is cancelled.
    ref_win = []; rf_h = []; rf_r = []
    ph = {"before": {"cells": set(), "walls": 0, "n": 0, "prev_wall": 0, "nodes": set()},
          "after":  {"cells": set(), "walls": 0, "n": 0, "prev_wall": 0, "nodes": set()}}
    for line in p.stdout.splitlines():
        if not line.startswith("{"): continue
        try: r = json.loads(line)
        except ValueError: continue
        t = float(r.get("t", 0.0))
        if t < control_from: continue
        if abs(float(r["x"])) > arena_half or abs(float(r["y"])) > arena_half: escaped += 1; continue
        if phase_at is not None:
            g = ph["before" if t < phase_at else "after"]
            g["n"] += 1; g["cells"].add((math.floor(float(r["x"]) / CELL_M), math.floor(float(r["y"]) / CELL_M)))
            gw = int(r.get("wall", 0)); g["walls"] += (gw and not g["prev_wall"]); g["prev_wall"] = gw
            mp0 = r.get("map") or []
            if len(mp0) >= 3: g["nodes"].add(int(mp0[2]))
        n += 1
        drv = r.get("drive", "walk"); sp = int(r.get("stop", 0))
        hist["resc" if drv == "scaffold" else "stand" if drv == "stand" else "stopW" if sp else "walk"] += 1
        q = r.get("q")
        if q and drv != "scaffold": (yaw_stop if sp else yaw_walk).append(float(q[7]))
        if drv == "walk" and not sp:
            tw = r.get("twist"); hd = r.get("hdg")
            if tw and hd and len(hd) >= 2:
                e = float(hd[0]) - float(hd[1])
                e = (e + math.pi) % (2.0 * math.pi) - math.pi       # the error is an ANGLE: wrap it
                hdg_err += abs(e); vyaw_abs += abs(float(tw[2])); vx_cmd += float(tw[0]); nyaw += 1
                if prev_vyaw is not None and prev_vyaw * float(tw[2]) < 0.0: yaw_flips += 1
                prev_vyaw = float(tw[2])
                ref_win.append((float(hd[0]), float(hd[1])))
                if len(ref_win) > 25:
                    h0, r0 = ref_win[-26]; h1, r1 = ref_win[-1]
                    dh = (h1 - h0 + math.pi) % (2*math.pi) - math.pi
                    dr = (r1 - r0 + math.pi) % (2*math.pi) - math.pi
                    if abs(dr) <= 0.5: rf_h.append(dh); rf_r.append(dr)   # a jump is not a follow
        x, y = float(r["x"]), float(r["y"])
        xs.append(x); ys.append(y)
        cells.add((math.floor(x / CELL_M), math.floor(y / CELL_M)))
        if prev is not None: path += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y); win_pts.append((x, y))
        w = int(r.get("wall", 0)); contact += w
        st_ = int(r.get("steer", 0)); steer_avoid += (st_ == 2); steer_play += (st_ == 1); steer_seek += (st_ == 3)
        sk = r.get("seek")
        if sk:
            seek_held += sk[0] > 0.0
            if prev_seek_value > 0.0 and sk[0] == 0.0: seek_ends += 1     # a held target dropped: arrived or forgotten
            prev_seek_value = sk[0]
        if w and not prev_wall: wall_eps += 1
        prev_wall = w
        down += float(r.get("tilt", 0.0)) > 60.0
        if "hw" in r and float(r.get("tilt", 0.0)) < 60.0:          # a head on the floor is not the loop's to keep still
            # the head IMU's x axis points DOWN when the camera is level: roll and pitch are hg[1], hg[2]
            # the head's rate over all three axes (its x axis is yaw: the IMU's x points down when level)
            hw = r["hw"]; hg = r["hg"]; hw2 += hw[0] ** 2 + hw[1] ** 2 + hw[2] ** 2; hg2 += hg[1] ** 2 + hg[2] ** 2; nh += 1
        o = int(r.get("obj", 0))
        if o and not prev_obj: obj_eps += 1
        prev_obj = o
        if layout and "qpos" in r:
            for nm, adr in layout:
                obj_start.setdefault(nm, (r["qpos"][adr], r["qpos"][adr + 1]))
                obj_end[nm] = (r["qpos"][adr], r["qpos"][adr + 1])
        tofs = r.get("tofs") or [0, 0, 0, 0]; tooclose += float(tofs[3]) if len(tofs) > 3 else 0.0
        mp = r.get("map") or []
        if len(mp) >= 3:
            tle_sum += float(mp[0]); novel += int(mp[1]); winners.add(int(mp[2]))
            if prev_w is not None and int(mp[2]) != prev_w and int(mp[2]) >= 0: switches += 1
            if int(mp[2]) >= 0: prev_w = int(mp[2])
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
        "switch_min": switches * 60.0 / max(1e-9, (secs - control_from)), "straight": _straightness(win_pts),
        "escaped": escaped, "avoid_pct": 100.0 * steer_avoid / max(1, n), "play_pct": 100.0 * steer_play / max(1, n),
        "seek_pct": 100.0 * steer_seek / max(1, n), "seek_held_pct": 100.0 * seek_held / max(1, n), "seek_ends": seek_ends,
        "objs_min": obj_eps / minutes, "down_pct": 100.0 * down / max(1, n),
        "head_w_rms": math.sqrt(hw2 / nh) if nh else float("nan"), "head_g_dev": math.sqrt(hg2 / nh) if nh else float("nan"),
        "obj_moved_m": sum(math.hypot(obj_end[k][0] - obj_start[k][0], obj_end[k][1] - obj_start[k][1]) for k in obj_end),
        **{f"{k}_pct": 100.0 * v / max(1, n) for k, v in hist.items()},
        "hdg_err": hdg_err / nyaw if nyaw else float("nan"),
        "ref_follow": _slope(rf_h, rf_r),
        "vyaw_abs": vyaw_abs / nyaw if nyaw else float("nan"),
        "vx_cmd": vx_cmd / nyaw if nyaw else float("nan"),
        "yaw_flips": yaw_flips * 60.0 * 50.0 / nyaw if nyaw else float("nan"),
        "yaw_stop": statistics.pstdev(yaw_stop) if len(yaw_stop) > 1 else float("nan"),
        "yaw_walk": statistics.pstdev(yaw_walk) if len(yaw_walk) > 1 else float("nan"),
    })
    return out


def fmt(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals: return "     -      "
    return f"{statistics.mean(vals):7.2f}±{statistics.stdev(vals):5.2f}" if len(vals) > 1 else f"{vals[0]:7.2f}      "


def _straightness(pts, window_ticks=1000):
    """Median over 20 s windows of net displacement / path length: 1 = a line, 0 = back where it
    started.  The blind-metric complement to cells and path (CLAUDE.md §3 rule 4): an orbit scores
    cells and path and ~0 here."""
    vals = []
    for i in range(0, len(pts) - window_ticks, window_ticks):
        seg = pts[i:i + window_ticks + 1]
        p = sum(math.hypot(seg[k + 1][0] - seg[k][0], seg[k + 1][1] - seg[k][1]) for k in range(len(seg) - 1))
        d = math.hypot(seg[-1][0] - seg[0][0], seg[-1][1] - seg[0][1])
        vals.append(d / p if p > 0 else 0.0)
    return statistics.median(vals) if vals else float("nan")


def _slope(xs, ys):
    """least squares dy/dx; the reference-follow slope (1 = the target turns with the body)."""
    if len(xs) < 10: return float("nan")
    mx = statistics.mean(xs); my = statistics.mean(ys)
    sxx = sum((a - mx) ** 2 for a in xs)
    return sum((a - mx) * (b - my) for a, b in zip(xs, ys)) / sxx if sxx else float("nan")


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
    ap.add_argument("--seed-from", type=int, default=1, help="the first seed (2026-09-29: a confirmation on seeds a signal did not use, e.g. --seeds 12 --seed-from 7)")
    ap.add_argument("--secs", type=int, default=1500)
    ap.add_argument("--control-from", type=float, default=None)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 2))
    ap.add_argument("--host-args", default="")
    ap.add_argument("--arm", action="append", default=[])
    ap.add_argument("--arm-base", default=None, help="the config --arm derives from (default: the first config)")
    ap.add_argument("--host-arm", action="append", default=[], metavar="NAME:ARGS",
                    help="an arm = the first config with these host args appended (for a lever that lives in the host)")
    ap.add_argument("--logdir", default=None)
    ap.add_argument("--full-logs", action="store_true", help="with --logdir: keep the host's stdout verbatim (~110 MB per 1500 s playroom run) instead of the compact stream -- for offline cloud/object analysis")
    ap.add_argument("--scene", default=str(REPO / "mj_host/models/microduck/scene_arena.xml"),
                    help="the level-2 scene (default: the 2 m arena -- the host's own default is the OPEN floor, where 'zero wall contacts' means no walls)")
    ap.add_argument("--noise", type=float, default=0.05, help="reset noise on the start pose, so seeds vary the start and not only the babble (host --noise)")
    ap.add_argument("--phase-at", type=float, default=None, help="split the control phase at this second (e.g. the --arena-shift time) and report cells / walls / nodes before and after -- the (d) reading")
    ap.add_argument("--arena-half", type=float, default=None, help="samples with |x| or |y| beyond this (m) are ESCAPED (the shifted scene leaves a gap) and excluded from cells/span/path; the count is reported (default: the scene manifest's half + 0.05, else 1.05 for the arena)")
    args = ap.parse_args()
    global FULL_LOGS
    FULL_LOGS = bool(args.full_logs)
    if not HOST.exists(): sys.exit(f"host binary missing: {HOST} (build with ./mj_host/run.sh build)")
    logdir = Path(args.logdir) if args.logdir else None
    tmp = Path(tempfile.mkdtemp(prefix="l2sweep_"))
    cfgs = [Path(c).resolve() for c in args.configs]
    arm_base = Path(args.arm_base).resolve() if args.arm_base else cfgs[0]
    cfgs += [make_arm(arm_base, spec, tmp) for spec in args.arm]
    cfgs = [Arm(c) for c in cfgs]
    for spec in args.host_arm:
        name, _, extra = spec.partition(":")
        if not extra.strip(): sys.exit(f"--host-arm {name}: no host args -- a TAUTOLOGY, not an arm")
        cfgs.append(Arm(cfgs[0].cfg, name, tuple(extra.split())))
        print(f"[host-arm] {name}: {cfgs[0].cfg.name} + {extra.strip()}", file=sys.stderr)
    ctrl = args.control_from if args.control_from is not None else control_from_default(cfgs[0].cfg)
    host_args = tuple(args.host_args.split())
    args.scene = str(Path(args.scene).resolve())          # the host runs in mj_host/: a relative scene path would miss
    man = Path(args.scene).with_suffix(".manifest.json")
    if args.arena_half is None:
        args.arena_half = (json.load(open(man)).get("half", 1.0) + 0.05) if man.exists() else 1.05
    if man.exists():
        mj = json.load(open(man))
        print(f"scene manifest: seed {mj.get('seed')}  sha {mj.get('xml_sha256')}  objects {len(mj.get('objects', []))}  half {mj.get('half')} m", file=sys.stderr)
    print(f"level-2 sweep: {len(cfgs)} arms × {args.seeds} seeds × {args.secs} s, control phase from {ctrl:.0f} s, scene {Path(args.scene).name}, reset noise {args.noise}, arena half {args.arena_half}, host args {host_args or '-'}", file=sys.stderr)

    jobs = [(c, s) for c in cfgs for s in range(args.seed_from, args.seed_from + args.seeds)]
    results = {c: [] for c in cfgs}
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futs = {ex.submit(run_one, c, s, args.secs, ctrl, host_args, logdir, args.scene, args.noise, args.phase_at, args.arena_half): (c, s) for c, s in jobs}
        for fut in concurrent.futures.as_completed(futs):
            c, s = futs[fut]; r = fut.result(); results[c].append(r)
            print(f"  {c.stem:34s} seed {s}: walls {r['walls_min']:6.1f}/min  cells {r['cells']:3d}  path {r['path_m']:6.1f} m  "
                  f"nodes {r['nodes']:3d}  mapTLE {r['map_tle']:.3f}  rescues {r['rescues_min']:.1f}/min", file=sys.stderr)

    print(f"\n=== LEVEL-2 A/B, {args.seeds} seeds × {args.secs} s, control phase {ctrl:.0f}–{args.secs} s ===")
    keys = [("walls_min", "walls/min"), ("contact_pct", "contact%"), ("tooclose", "tooclose"), ("path_m", "path m"),
            ("cells", "cells"), ("span", "span m²"), ("straight", "straight"), ("nodes", "nodes"), ("switch_min", "switch/min"), ("map_tle", "mapTLE"), ("novel_pct", "novel%"),
                  ("hdg_err", "hdgErr"), ("ref_follow", "refFollow"), ("vyaw_abs", "|vyaw|"), ("yaw_flips", "yawFlip/min"), ("vx_cmd", "vxCmd"),
            ("turns", "turns"), ("rescues_min", "resc/min"), ("driven_pct", "driven%"), ("escaped", "escaped"), ("avoid_pct", "avoid%"), ("play_pct", "play%"),
            ("seek_pct", "seek%"), ("seek_held_pct", "seekHeld%"), ("seek_ends", "seekEnds"),
            ("objs_min", "objs/min"), ("obj_moved_m", "objMoved m"), ("down_pct", "down%"),
            ("head_w_rms", "headW rms"), ("head_g_dev", "headG dev"),
            ("walk_pct", "walk%"), ("stopW_pct", "stopW%"), ("stand_pct", "stand%"), ("resc_pct", "resc%"),
            ("stops", "stops"), ("stops_arrive", "stopsArrive"), ("stops_stuck", "stopsStuck"), ("escapes", "escapes"), ("spins", "spins"), ("spin_rolls", "spinRolls"), ("handbacks", "handbacks"), ("survived", "survived"), ("survive_pct", "survive%"),
            ("handoffs", "handoffs"), ("refused", "refused"), ("stop_resc", "stopResc"),
            ("yaw_stop", "yawStop sd"), ("yaw_walk", "yawWalk sd"),
            ("saccades", "saccades"), ("novel_holds", "novelHolds"), ("bored_pct", "bored%"), ("stop_len", "stop s"),
            ("cloud_vox", "cloudVox"), ("cloud_brk", "cloudBrk"), ("ins_stop", "insStop"), ("bake_stop", "bakeStop"), ("pruned", "pruned"), ("pruned_baked", "prunedBaked"), ("nodes_end", "nodesEnd"), ("baked_end", "bakedEnd"),
            ("rolls", "rolls"), ("rolls_skipped", "rollSkip"), ("changes", "changes"), ("prompted", "prompted"), ("detect_pct", "detect%"), ("unprompted", "unprompted"), ("orientations", "orients"), ("arrivals", "arrivals"), ("orient_timeouts", "timeouts"), ("reach_m", "reach m")]
    print(f"{'arm':34s} " + " ".join(f"{lbl:>13s}" for _, lbl in keys))
    for c in cfgs:
        rows = sorted(results[c], key=lambda r: r["seed"])
        print(f"{c.stem:34s} " + " ".join(f"{fmt([r.get(k) for r in rows]):>13s}" for k, _ in keys))
    if len(cfgs) > 1:
        ref = cfgs[0]
        for c in cfgs[1:]:
            print(f"\n  PAIRED {c.stem} − {ref.stem}:")
            for k, better in (("walls_min", "lower"), ("cells", "higher"), ("path_m", "-"), ("straight", "higher"), ("nodes", "higher"), ("switch_min", "-"), ("map_tle", "-"),
                                    ("hdg_err", "lower"), ("ref_follow", "lower"), ("vyaw_abs", "lower"), ("yaw_flips", "lower"),
                                    ("rescues_min", "lower"),
                              ("stand_pct", "-"), ("survive_pct", "higher"), ("handoffs", "lower"), ("stop_resc", "lower")):
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
