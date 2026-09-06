#!/usr/bin/env python3
"""cell_liveness.py — the gate a Cell config must pass before any battery counts.

Cell system audit, 2026-09-06.  Twelve EPMs across six maze configs were fed zero vectors
for months, a study config shipped with its vision loop weighted to zero, and play's climb
never engaged in any run — none of it visible in eats.  This runs a config headless for a
short window and evaluates liveness predicates on the diagnostic stream, printing one
PASS / FAIL / n/a line each.  It is CLAUDE.md §3.2 (checks 2, 4, 5, 7) made mechanical.

    python3 godot_host/project/scripts_tools/cell_liveness.py \\
        --config res://addons/ami_ogma/configs/the_cell_arbiter_room_pillars_vision.json \\
        [--duration 60] [--arm key=val,...] [--expect Module.param=value] [--vary-world-seed N]

Predicates
  world      the seed line is present and echoed (which pillar layout, which run seed)
  params     OGMA_DUMP_PARAMS lines: every --expect value landed in the module that reports it
  EPM <id>   nodes > 2, baked >= 1, tle > 0 on some tick, top winner < 95 % of ticks
  arbiter    every loop the config enables wins >= 1 tick; every enabled G term is non-zero somewhere
  play       route_exists on some tick (else climb is structurally impossible); climb fraction reported
  planner    raw planner value > 0 on some tick (a route was ever scored)
  vision     food-in-view on some tick when a vision loop is present
"""
from __future__ import annotations
import argparse, json, os, re, subprocess, sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cell_coverage import _res_to_fs, _coerce, patch_config, gen_world, GODOT_BIN, PROJECT_DIR, TICKS_PER_SEC

SEED_LINE = re.compile(r"TheCell: room=.*\(seed (-?\d+), obs_seed (-?\d+)\)")


def run(res: str, duration_s: int, world_seed: int, obstacle_seed: int | None) -> tuple[str, str, int]:
    env = dict(os.environ)
    env.update(OGMA_SEED=str(world_seed), OGMA_CELL_CONFIG=res, OGMA_TURBO="1", OGMA_FWDLOG="1",
               OGMA_DUMP_PARAMS="1", OGMA_EPISODE_LENGTH=str(duration_s * TICKS_PER_SEC),
               OGMA_QUIT_AFTER_TICKS=str(duration_s * TICKS_PER_SEC + 60))
    if obstacle_seed is not None:
        env["OGMA_OBSTACLE_SEED"] = str(obstacle_seed)
    cmd = ["timeout", "--signal=TERM", str(max(60, 5 * duration_s)), GODOT_BIN, "--path", str(PROJECT_DIR),
           "--headless", "--fixed-fps", "60", "--disable-render-loop", "res://scenes/the_cell.tscn"]
    p = subprocess.run(cmd, env=env, capture_output=True, text=True)
    return p.stdout, p.stderr, p.returncode


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--duration", type=int, default=60)
    ap.add_argument("--arm", default="", help="key=val[,key=val] overrides (cell_coverage syntax: Type.param or bare PlayLoop key)")
    ap.add_argument("--expect", action="append", default=[], help="Module.param=value that must have landed (from the PARAMS dump)")
    ap.add_argument("--world-seed", type=int, default=1000)
    ap.add_argument("--vary-world-seed", type=int, default=None, help="draw a --vary-world world from this seed")
    ap.add_argument("--explore-seed", type=int, default=11)
    ap.add_argument("--metadata", action="append", default=[], help="k=v world metadata override (e.g. obstacle_density=0.06)")
    a = ap.parse_args()

    overrides = {}
    for kv in a.arm.split(",") if a.arm else []:
        k, _, v = kv.partition("="); overrides[k.strip()] = _coerce(v.strip())
    world = gen_world(a.vary_world_seed) if a.vary_world_seed is not None else None
    for kv in a.metadata:
        k, _, v = kv.partition("="); world = {**(world or {}), k.strip(): _coerce(v.strip())}
    res, fs = patch_config(a.config, overrides, a.explore_seed, f"liveness_{os.getpid()}", world=world, planner_seed=a.explore_seed + 7919)
    cfg = json.load(open(fs))
    try:
        out, err, rc = run(res, a.duration, a.world_seed, a.world_seed)
    finally:
        try: fs.unlink()
        except FileNotFoundError: pass

    # ---- config facts that decide which predicates apply ----
    mods = {m["id"]: m for m in cfg.get("modules", [])}
    types = Counter(m["type"] for m in mods.values())
    arb = next((m for m in mods.values() if m["type"] == "EFEArbiter"), None)
    ap_ = (arb or {}).get("params", {})
    loops_enabled = {"klino": True, "planner": True,
                     "play": float(ap_.get("play_weight", 0.0)) > 0 and bool(ap_.get("play_value_topic", "")),
                     "vision": float(ap_.get("vision_weight", 0.0)) > 0 and bool(ap_.get("vision_value_topic", ""))}
    planner_epi = bool(ap_.get("planner_epistemic", True))
    precision_mode = ap_.get("scoring_mode") == "precision"   # round 3: no epistemic G terms by construction

    # ---- stream ----
    seed_line = None
    params_lines = {}
    epm = defaultdict(lambda: {"ticks": 0, "nodes": 0, "baked": 0, "tle_pos": 0, "wid": Counter()})
    win = Counter(); gterms = defaultdict(float); arb_ticks = 0; rp_pos = 0
    play_ticks = climb = rex = 0
    vis_ticks = hfood = 0
    diag_lines = 0
    eats = 0
    for ln in out.splitlines():
        ln = ln.strip()
        if not ln.startswith("{"):
            m = SEED_LINE.search(ln)
            if m: seed_line = (int(m.group(1)), int(m.group(2)))
            continue
        try: r = json.loads(ln)
        except json.JSONDecodeError: continue
        if r.get("event") == "PARAMS":
            params_lines[r["module"]] = r; continue
        if r.get("event") == "HIT":
            eats += 1; continue
        if "pos" not in r: continue
        diag_lines += 1
        for mid, mv in r.get("modules", {}).items():
            if not isinstance(mv, dict): continue
            t = mods.get(mid, {}).get("type", "")
            if t == "EPM" or "nodes" in mv and "wid" in mv:
                e = epm[mid]; e["ticks"] += 1
                e["nodes"] = max(e["nodes"], int(mv.get("nodes", 0))); e["baked"] = max(e["baked"], int(mv.get("baked", 0)))
                e["tle_pos"] += float(mv.get("tle", 0.0)) > 0.0; e["wid"][int(mv.get("wid", -1))] += 1
            if "win" in mv and "vact" in mv:
                arb_ticks += 1; win[int(mv.get("win", -1))] += 1
                for k in ("gpk", "gpp", "gek", "gep", "gepl", "gvi"):
                    gterms[k] = max(gterms[k], abs(float(mv.get(k, 0.0))))
                rp_pos += float(mv.get("rp", 0.0)) > 0.0
            if "wand" in mv and "climb" in mv:
                play_ticks += 1; climb += bool(mv.get("climb")); rex += bool(mv.get("rex"))
            if "hfood" in mv:
                vis_ticks += 1; hfood += bool(mv.get("hfood"))

    # ---- verdicts ----
    fails = warns = 0
    def line(name, ok, detail, weak=False):
        """ok True/False/None -> PASS / FAIL (or WARN when weak) / n/a.  A weak predicate is one
        the world can fail for the brain (no eat yet, food never in view) -- it warns, and only
        a structural predicate fails the gate."""
        nonlocal fails, warns
        if ok is None: tag = "n/a "
        elif ok:       tag = "PASS"
        elif weak:     tag = "WARN"; warns += 1
        else:          tag = "FAIL"; fails += 1
        print(f"  {tag}  {name:<28} {detail}")
    print(f"liveness: {a.config}  {a.duration}s  arm={overrides or '{}'}  diag samples {diag_lines} (one per second)  eats {eats}  rc {rc}")
    # the module graph, read not assumed (2026-09-06: a base whose klino was v1 instead of the study's V2 went unnoticed for a battery)
    print("  graph  " + ", ".join(f"{m['id']}:{m['type']}" for m in cfg.get("modules", [])))
    line("run", diag_lines > 0, f"{diag_lines} diag lines" + ("" if diag_lines else f"; stderr: {err.strip().splitlines()[-2:]}"))
    line("world seed", seed_line is not None, f"seed/obs_seed {seed_line}" if seed_line else "no TheCell seed line found")
    for ex in a.expect:
        mod, _, kv = ex.partition("."); key, _, val = kv.partition("=")
        want = _coerce(val); got = None; src = "-"
        for pl in params_lines.values():
            if pl["module"] == mod or pl["type"] == mod:
                got = pl["params"].get(key); src = pl["source"]; break
        line(f"param {ex}", got is not None and (got == want or (isinstance(got, (int, float)) and isinstance(want, (int, float)) and abs(got - want) < 1e-9)),
             f"landed as {got!r} ({src})")
    for mid, e in sorted(epm.items()):
        top = (e["wid"].most_common(1)[0][1] / e["ticks"]) if e["ticks"] else 1.0
        ok = e["nodes"] > 2 and e["baked"] >= 1 and e["tle_pos"] > 0 and top < 0.95
        # a vision EPM only moves when something is in view; treat its liveness as weak
        weak = "vision" in mid.lower() and hfood == 0
        line(f"EPM {mid}", ok, f"nodes {e['nodes']} baked {e['baked']} tle>0 on {e['tle_pos']}/{e['ticks']} samples, top winner {top:.2f}"
             + ("  (nothing was in view; run longer to judge)" if weak and not ok else ""), weak=weak)
    if arb is not None:
        names = ["klino", "planner", "play", "vision"]
        for i, n in enumerate(names):
            if not loops_enabled[n]:
                line(f"arbiter {n} wins", None, "disabled by config"); continue
            weak = (n == "planner" and eats == 0) or (n == "vision" and hfood == 0)
            line(f"arbiter {n} wins", win[i] > 0, f"{win[i]}/{arb_ticks} samples"
                 + ("  (no eat yet: the planner has nothing to route to)" if n == "planner" and eats == 0 else "")
                 + ("  (food never in view)" if n == "vision" and hfood == 0 else ""), weak=weak)
        line("G klino pragmatic", gterms["gpk"] > 0, f"max |G| {gterms['gpk']:.3f}")
        line("G klino epistemic", None if precision_mode else gterms["gek"] > 0, f"max |G| {gterms['gek']:.3f}" + ("  (precision mode: no epistemic terms)" if precision_mode else ""))
        line("G planner pragmatic", gterms["gpp"] > 0, f"max |G| {gterms['gpp']:.3f}" + ("  (no eat yet)" if eats == 0 else ""), weak=(eats == 0))
        line("G planner epistemic", None if (precision_mode or not planner_epi) else gterms["gep"] > 0, f"max |G| {gterms['gep']:.3f}" + ("" if planner_epi and not precision_mode else "  (off by mode/config)"))
        if loops_enabled["play"]:   line("G play", gterms["gepl"] > 0, f"max |G| {gterms['gepl']:.3f}")
        if loops_enabled["vision"]: line("G vision", gterms["gvi"] > 0, f"max |G| {gterms['gvi']:.3f}" + ("  (food never in view)" if hfood == 0 else ""), weak=(hfood == 0))
        line("planner route scored", rp_pos > 0, f"raw planner value > 0 on {rp_pos}/{arb_ticks} samples" + ("  (no eat yet)" if eats == 0 else ""), weak=(eats == 0))
    if play_ticks:
        line("play route_exists", rex > 0, f"route_exists on {rex}/{play_ticks} samples; climb fraction {climb/play_ticks:.3f}")
    if "VisualHomingNav" in types or vis_ticks:
        line("vision food in view", hfood > 0, f"{hfood}/{vis_ticks} samples  (world geometry, not the brain)", weak=True)
    src = Counter(pl["source"] for pl in params_lines.values())
    print(f"  info  PARAMS dump: {len(params_lines)} modules ({dict(src)})")
    print(f"liveness: {'PASS' if fails == 0 else f'FAIL ({fails})'}{f'  warnings {warns}' if warns else ''}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
