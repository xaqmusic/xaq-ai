#!/usr/bin/env python3
"""Cell PLAY coverage / maze-traversal analyzer + A/B driver.

Measures how well play's WANDER traverses a maze for foraging discovery:

  * coverage(t)  — cumulative distinct world-cells (default 2 m) visited, binned
  * first-far    — first tick the bug crosses into the far region (behind the
                   lbend wall: world z > z_wall)
  * crossings    — south<->north wall crossings over the run (the foraging-
                   traversal proxy: forage a 2-region maze = cross repeatedly)
  * n_nodes / play state (climb/wand/forced-wand/stale)

The play wander RNG is `explore_seed` (a module param), NOT OGMA_SEED, and the
fixed-food env is seed-inert, so A/B power comes from varying explore_seed. This
driver patches a base config into temp configs (one per arm x explore_seed),
runs them headless-turbo, and reports per-arm summaries. Summaries built-in
(feedback_test_summarisation); raw JSONL suppressed.

Usage (single config):
  python3 godot_host/project/scripts_tools/cell_coverage.py \
    --config res://addons/ami_ogma/configs/the_cell_play_only_lbend.json \
    --n-explore 5 --duration 240

Usage (A/B over frontier_bias):
  python3 godot_host/project/scripts_tools/cell_coverage.py \
    --config res://addons/ami_ogma/configs/the_cell_play_only_lbend.json \
    --arm base:frontier_bias=0 --arm mid:frontier_bias=0.5 --arm full:frontier_bias=1.0 \
    --n-explore 5 --duration 240
"""
from __future__ import annotations

import argparse
import concurrent.futures
import copy
import json
import math
import os
import random
import statistics
import subprocess
import sys
from pathlib import Path

# Recovered 2026-09-05 from the ami-ogma origin (branch cell-maze, 2026-07-13) for the
# Kalman-lessons campaign; the report's harness (cell report §"Harness and data").  It lived
# at <repo>/scripts/ there; here it sits beside the picrawler tools, so the project dir is
# derived the way mkarm.py derives it.
PROJECT_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT   = PROJECT_DIR.parent.parent
CONFIG_DIR  = PROJECT_DIR / "addons" / "ami_ogma" / "configs"
GODOT_BIN   = os.environ.get("GODOT4", "godot4")
TICKS_PER_SEC = 60


def _res_to_fs(res_path: str) -> Path:
    if res_path.startswith("res://"):
        return PROJECT_DIR / res_path[len("res://"):]
    return Path(res_path)


def _coerce(v: str):
    if v.lower() in ("true", "false"):        # bool params (e.g. learn_appearance) need real JSON booleans
        return v.lower() == "true"
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


def gen_world(seed: int, room_frac: float = 0.36, min_sep: float = 0.5) -> dict:
    """A per-seed FIXED-within-episode foraging world (doctrine-safe power): 2 food sites drawn
    from the seed, spread ≥min_sep apart, inside [-room_frac, room_frac] (away from the outer
    wall); spawn tied to site A; pillar layout = seed. Fixed for the whole run — the agent can
    still learn it — but each seed samples a different world for statistical power (paired A/B)."""
    rng = random.Random((seed * 2654435761) & 0xffffffff)
    a = b = None
    for _ in range(300):
        a = (rng.uniform(-room_frac, room_frac), rng.uniform(-room_frac, room_frac))
        b = (rng.uniform(-room_frac, room_frac), rng.uniform(-room_frac, room_frac))
        if math.hypot(a[0] - b[0], a[1] - b[1]) >= min_sep:
            break
    else:
        a, b = (-room_frac, 0.0), (room_frac, 0.0)
    return {"food_positions": [[round(a[0], 3), round(a[1], 3)], [round(b[0], 3), round(b[1], 3)]],
            "spawn_pos": [round(a[0], 3), round(a[1], 3)],
            "obstacle_seed": int(seed)}


def patch_config(base_res: str, overrides: dict, explore_seed: int, tag: str,
                 world: dict | None = None) -> tuple[str, Path]:
    """Write a temp config with param overrides + explore_seed (+ optional per-seed world metadata).
    Returns (res_path, fs_path).

    Override keys: a bare name (e.g. "frontier_bias") patches ALL PlayLoop modules; a dotted
    "Type.param" (e.g. "EFEArbiter.vision_weight") patches every module of that type.
    """
    base_fs = _res_to_fs(base_res)
    cfg = json.load(open(base_fs))
    if world:
        md = cfg.setdefault("metadata", {})
        md.update(world)
    # explore_seed always onto PlayLoop (kept for play A/B power)
    for mod in cfg.get("modules", []):
        if mod.get("type") == "PlayLoop":
            mod.setdefault("params", {})["explore_seed"] = int(explore_seed)
    for k, val in overrides.items():
        mtype, _, param = k.partition(".")
        if not param:      # bare key → PlayLoop
            mtype, param = "PlayLoop", k
        for mod in cfg.get("modules", []):
            if mod.get("type") == mtype:
                mod.setdefault("params", {})[param] = val
    name = f"_cov_tmp_{tag}.json"
    fs = CONFIG_DIR / name
    json.dump(cfg, open(fs, "w"), indent=2)
    return f"res://addons/ami_ogma/configs/{name}", fs


def run_one(config_res: str, duration_s: int, cell_m: float, z_wall: float,
            world_seed: int = 42, turbo: bool = True, fwdlog: bool = False) -> dict:
    env = dict(os.environ)
    env["OGMA_SEED"] = str(world_seed)
    env["OGMA_CELL_CONFIG"] = config_res
    if turbo:
        env["OGMA_TURBO"] = "1"
    if fwdlog:
        env["OGMA_FWDLOG"] = "1"   # emit FWDLOG w/ fdist (distance to nearest active food) every 5 ticks
    env["OGMA_EPISODE_LENGTH"] = str(duration_s * TICKS_PER_SEC)
    env["OGMA_QUIT_AFTER_TICKS"] = str(duration_s * TICKS_PER_SEC + 60)

    wall_timeout = max(60, 5 * duration_s) if turbo else max(120, duration_s * 3)
    base = [GODOT_BIN, "--path", str(PROJECT_DIR), "--headless"]
    if turbo:
        base += ["--fixed-fps", "60", "--disable-render-loop"]
    base += ["res://scenes/the_cell.tscn"]
    cmd = ["timeout", "--signal=TERM", str(wall_timeout)] + base
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True)

    cells: set[tuple[int, int]] = set()
    cov_series: list[tuple[int, int]] = []
    first_far: int | None = None
    far_ticks = crossings = total_diag = n_nodes = eats = 0
    prev_far: bool | None = None
    play_ticks = climb_ticks = wand_ticks = fwand_ticks = hfront_ticks = 0
    stale_sum = 0.0
    vh_ticks = hfood_ticks = 0
    vval_max = 0.0
    arb_ticks = vwin_ticks = 0   # arbiter ticks, and ticks the VISION policy won (winner==3)
    win_hist = [0, 0, 0, 0]      # winner histogram: klino / planner / play / vision
    min_fdist = None
    fd_sum = 0.0
    near2 = near1 = fwd_samples = 0   # ticks with food within 2m / 1m (close approaches)
    for ln in proc.stdout.splitlines():
        ln = ln.strip()
        if not ln.startswith("{"):
            continue
        try:
            r = json.loads(ln)
        except json.JSONDecodeError:
            continue
        if r.get("event") == "HIT":
            eats += 1
            continue
        if r.get("event") == "FWDLOG":
            fd = float(r.get("fdist", -1.0))
            if fd >= 0.0:
                fwd_samples += 1
                fd_sum += fd
                if min_fdist is None or fd < min_fdist:
                    min_fdist = fd
                if fd < 2.0:
                    near2 += 1
                if fd < 1.0:
                    near1 += 1
            continue
        if "pos" not in r:
            continue
        total_diag += 1
        t = int(r.get("t", 0))
        x, z = float(r["pos"][0]), float(r["pos"][1])
        cells.add((round(x / cell_m), round(z / cell_m)))
        cov_series.append((t, len(cells)))
        is_far = z > z_wall
        if is_far:
            far_ticks += 1
            if first_far is None:
                first_far = t
        if prev_far is not None and is_far != prev_far:
            crossings += 1
        prev_far = is_far
        for mid, mv in r.get("modules", {}).items():
            if not isinstance(mv, dict):
                continue
            if "wand" in mv:                       # PlayLoop
                n_nodes = max(n_nodes, int(mv.get("nn", 0)))
                play_ticks += 1
                climb_ticks += bool(mv.get("climb"))
                wand_ticks += bool(mv.get("wand"))
                fwand_ticks += bool(mv.get("fwand"))
                hfront_ticks += bool(mv.get("hfront"))
                stale_sum += float(mv.get("stale", 0))
            if "hfood" in mv:                      # VisualHomingNav
                vh_ticks += 1
                hfood_ticks += bool(mv.get("hfood"))
                vval_max = max(vval_max, float(mv.get("val", 0.0)))
            if "win" in mv and "vact" in mv:       # EFEArbiter
                arb_ticks += 1
                w = int(mv.get("win", -1))
                if 0 <= w <= 3:
                    win_hist[w] += 1
                if w == 3:
                    vwin_ticks += 1
        if "hits" in r:
            eats = max(eats, int(r.get("hits", 0)))

    marks = {}
    if cov_series:
        last_t = cov_series[-1][0]
        for frac in (0.25, 0.5, 0.75, 1.0):
            tgt = last_t * frac
            best = 0
            for (t, c) in cov_series:
                if t <= tgt:
                    best = c
                else:
                    break
            marks[f"cov{int(frac*100)}"] = best
    return {
        "final_cov": len(cells), "marks": marks, "first_far": first_far,
        "far_frac": round(far_ticks / total_diag, 3) if total_diag else 0.0,
        "crossings": crossings, "n_nodes": n_nodes, "eats": eats,
        "climb_frac": round(climb_ticks / play_ticks, 3) if play_ticks else 0.0,
        "wand_frac": round(wand_ticks / play_ticks, 3) if play_ticks else 0.0,
        "fwand_frac": round(fwand_ticks / play_ticks, 3) if play_ticks else 0.0,
        "hfront_frac": round(hfront_ticks / play_ticks, 3) if play_ticks else 0.0,
        "mean_stale": round(stale_sum / play_ticks, 1) if play_ticks else 0.0,
        "hfood_frac": round(hfood_ticks / vh_ticks, 3) if vh_ticks else 0.0,
        "vwin_frac": round(vwin_ticks / arb_ticks, 3) if arb_ticks else 0.0,
        "win_hist": [round(x / arb_ticks, 3) for x in win_hist] if arb_ticks else [0, 0, 0, 0],
        "vval_max": round(vval_max, 3),
        "min_fdist": round(min_fdist, 2) if min_fdist is not None else None,
        "mean_fdist": round(fd_sum / fwd_samples, 2) if fwd_samples else None,
        "near2_frac": round(near2 / fwd_samples, 3) if fwd_samples else 0.0,
        "near1_frac": round(near1 / fwd_samples, 3) if fwd_samples else 0.0,
        "diag_lines": total_diag,
        "stderr_tail": proc.stderr.strip().splitlines()[-3:] if proc.returncode else [],
    }


def _mean(v):
    return f"{statistics.mean(v):.1f}" if v else "-"


def summarize(arm: str, rows: list[dict], duration_s: int) -> None:
    ok = [r for r in rows if r["diag_lines"] > 0]
    print(f"\n===== [{arm}] — {len(ok)}/{len(rows)} runs, {duration_s}s =====")
    if not ok:
        print("  NO DIAG LINES — check config/build. stderr tails:")
        for r in rows:
            for l in r["stderr_tail"]:
                print("   ", l)
        return
    cov = lambda k: [r["marks"].get(k) for r in ok if r["marks"].get(k) is not None]
    reached = [r for r in ok if r["first_far"] is not None]
    print(f"  coverage @25/50/75/100%: {_mean(cov('cov25'))} / {_mean(cov('cov50'))} "
          f"/ {_mean(cov('cov75'))} / {_mean(cov('cov100'))}   final {_mean([r['final_cov'] for r in ok])}")
    print(f"  FAR reached: {len(reached)}/{len(ok)}  first-far {_mean([r['first_far'] for r in reached])}  "
          f"per-run {[r['first_far'] for r in ok]}")
    print(f"  crossings (S<->N): {_mean([r['crossings'] for r in ok])}  per-run {[r['crossings'] for r in ok]}")
    print(f"  far dwell frac: {_mean([r['far_frac'] for r in ok])}   eats: {_mean([r['eats'] for r in ok])}  per-run {[r['eats'] for r in ok]}")
    if any(r.get("min_fdist") is not None for r in ok):
        mf = [r["min_fdist"] for r in ok if r.get("min_fdist") is not None]
        mfd = [r["mean_fdist"] for r in ok if r.get("mean_fdist") is not None]
        print(f"  FORAGE approach: MEAN-food-dist {_mean(mfd)}m per-run {mfd}  "
              f"(dense proxy: lower = better approach)  min {_mean(mf)}m")
    if any(r.get("vwin_frac") for r in ok) or any(r.get("hfood_frac") for r in ok):
        print(f"  VISION: won-frac {_mean([r['vwin_frac'] for r in ok])}  food-in-view frac {_mean([r['hfood_frac'] for r in ok])}  "
              f"max vision_value {_mean([r['vval_max'] for r in ok])}")
    hists = [r.get("win_hist") for r in ok if r.get("win_hist")]
    if hists:
        avg = [round(sum(h[i] for h in hists) / len(hists), 3) for i in range(4)]
        print(f"  WINNER FRAC (klino/planner/play/vision): {avg[0]} / {avg[1]} / {avg[2]} / {avg[3]}")
    print(f"  play state: climb {_mean([r['climb_frac'] for r in ok])}  wand {_mean([r['wand_frac'] for r in ok])}  "
          f"fwand {_mean([r['fwand_frac'] for r in ok])}  hfront {_mean([r['hfront_frac'] for r in ok])}  "
          f"stale {_mean([r['mean_stale'] for r in ok])}  nodes {_mean([r['n_nodes'] for r in ok])}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--arm", action="append", default=[],
                    help="name:key=val[,key=val] play-param override arm. Repeatable. "
                         "Default (none) = single arm 'base' with no override.")
    ap.add_argument("--n-explore", type=int, default=3, help="explore_seed variants per arm")
    ap.add_argument("--explore-base", type=int, default=11)
    ap.add_argument("--duration", type=int, default=240)
    ap.add_argument("--cell-m", type=float, default=2.0)
    ap.add_argument("--z-wall", type=float, default=3.6)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--fwdlog", action="store_true",
                    help="enable OGMA_FWDLOG → track closest-approach-to-food (fdist) for close-failure diagnosis")
    ap.add_argument("--vary-world", action="store_true",
                    help="POWERED mode: per seed, draw a FIXED-within-episode world (food positions + pillar "
                         "layout) shared across all arms (paired). Samples the world population for real power.")
    ap.add_argument("--room-frac", type=float, default=0.36, help="--vary-world: food within ±this·room from centre")
    ap.add_argument("--min-sep", type=float, default=0.5, help="--vary-world: min food-site separation (room fraction)")
    args = ap.parse_args()

    arms: list[tuple[str, dict]] = []
    if not args.arm:
        arms = [("base", {})]
    else:
        for spec in args.arm:
            name, _, ov = spec.partition(":")
            d = {}
            for kv in ov.split(",") if ov else []:
                k, _, v = kv.partition("=")
                d[k.strip()] = _coerce(v.strip())
            arms.append((name.strip(), d))

    eseeds = [args.explore_base + i for i in range(args.n_explore)]
    tmp_files: list[Path] = []
    jobs = []   # (arm_name, seed, res_path)
    for name, ov in arms:
        for es in eseeds:
            world = gen_world(es, args.room_frac, args.min_sep) if args.vary_world else None
            res, fs = patch_config(args.config, ov, es, f"{name}_e{es}", world=world)
            tmp_files.append(fs)
            jobs.append((name, es, res))
    if args.vary_world:
        print(f"[--vary-world] {args.n_explore} per-seed FIXED worlds (food+pillars drawn per seed, shared across arms; paired)")

    results: dict[str, list[tuple[int, dict]]] = {name: [] for name, _ in arms}
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
            futs = {ex.submit(run_one, res, args.duration, args.cell_m, args.z_wall,
                              42, True, args.fwdlog): (name, es)
                    for name, es, res in jobs}
            for fut in concurrent.futures.as_completed(futs):
                name, es = futs[fut]
                results[name].append((es, fut.result()))
    finally:
        for f in tmp_files:
            try:
                f.unlink()
            except FileNotFoundError:
                pass

    for name, _ in arms:
        summarize(name, [r for _, r in sorted(results[name])], args.duration)
    # paired A/B stat — arms[0] (the reference, e.g. fused) vs EACH later arm.
    # For a leave-one-out, put fused FIRST; each pair reports Δ = reference − minus
    # (positive eats Δ = the removed loop CONTRIBUTES). Falls back to the plain
    # two-arm A/B when only two arms are given.
    if len(arms) >= 2:
        ref_name, ref_res = arms[0][0], results[arms[0][0]]
        for k in range(1, len(arms)):
            m_name = arms[k][0]
            paired_ab(m_name, results[m_name], ref_name, ref_res)  # B=ref → Δ = ref − minus
    return 0


def paired_ab(nameA, resA, nameB, resB):
    """Paired analysis of two arms sharing per-seed worlds: eats(B)−eats(A) per seed, mean, paired-t."""
    a = {s: r for s, r in resA if r["diag_lines"] > 0}
    b = {s: r for s, r in resB if r["diag_lines"] > 0}
    seeds = sorted(set(a) & set(b))
    if len(seeds) < 2:
        return
    for metric, better in (("eats", "higher"), ("mean_fdist", "lower")):
        diffs = []
        for s in seeds:
            va, vb = a[s].get(metric), b[s].get(metric)
            if va is None or vb is None:
                continue
            diffs.append(vb - va)   # B − A
        if len(diffs) < 2:
            continue
        md = statistics.mean(diffs); sd = statistics.pstdev(diffs) if len(diffs) < 2 else statistics.stdev(diffs)
        se = sd / math.sqrt(len(diffs)) if sd else 0.0
        t = md / se if se else float("inf")
        npos = sum(1 for d in diffs if d > 0); nneg = sum(1 for d in diffs if d < 0)
        print(f"\n  ===== PAIRED  {nameB} − {nameA}  ({metric}, {better}=better for B, n={len(diffs)}) =====")
        print(f"    per-seed Δ: {[round(d,2) for d in diffs]}")
        print(f"    mean Δ = {md:+.2f}   sd = {sd:.2f}   paired-t = {t:+.2f}   sign: {npos}+ / {nneg}−")


if __name__ == "__main__":
    sys.exit(main())
