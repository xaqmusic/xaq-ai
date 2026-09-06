#!/usr/bin/env python3
"""(d) perturbation test — the sharpest active-inference evidence (doctrine §2 bar d).

Drives a mid-episode VISION DROPOUT window on the vision loop and measures the eat rate
across three phases:  pre (vision ON) → lesion (vision BLIND) → post (vision RESTORED).
A load-bearing inference loop shows:  perturbation → DEGRADATION (eats fall when the sense
is removed) → RECOVERY (eats return when it comes back). A no-lesion CONTROL arm isolates
the effect from any over-time drift.

Lesions (--lesion): vision = VisualBearing.lesion_after/until_ticks (the original);
stick = VisualBearing.stick_after_ticks (camera frozen on its last sighting, restored at t2
via stick_until_ticks if the module supports it, else held to the end); scent_noise and
heading_drift = body-side instruments (metadata scent_noise_sigma / heading_drift_sigma with
*_after_ticks / *_until_ticks), added 2026-09-06 by the cell system audit.

2026-09-06 (cell system audit): the world seed used to be fixed at 42 for every run and
only PlayLoop's RNG varied; --vary-world now draws a distinct world (food sites, spawn,
pillar layout) per job, paired across the lesion and control arms, with its own OGMA_SEED /
OGMA_OBSTACLE_SEED, and every seed set is printed.

Usage:
  python3 godot_host/project/scripts_tools/cell_perturbation_d.py \
    --config res://addons/ami_ogma/configs/the_cell_vision_dtest.json \
    --ticks 45000 --n 5 [--vary-world] [--lesion vision|stick|scent_noise|heading_drift]
"""
from __future__ import annotations
import argparse, concurrent.futures, json, os, statistics, subprocess, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cell_coverage import gen_world   # the same per-seed world draw as the coverage harness

# Recovered 2026-09-05 from the ami-ogma origin (branch cell-maze); see cell_coverage.py.
PROJ = Path(__file__).resolve().parents[1]
REPO = PROJ.parent.parent
CFGDIR = PROJ / "addons" / "ami_ogma" / "configs"
GODOT = os.environ.get("GODOT4", "godot4")


def _fs(res): return PROJ / res[len("res://"):] if res.startswith("res://") else Path(res)


def patch(base_res, lesion, t1, t2, eseed, tag, kind="vision", world=None, planner_seed=None):
    cfg = json.load(open(_fs(base_res)))
    if world:
        cfg.setdefault("metadata", {}).update(world)
    md = cfg.setdefault("metadata", {})
    for m in cfg.get("modules", []):
        if m.get("type") == "PlayLoop":
            m.setdefault("params", {})["explore_seed"] = int(eseed)
        if planner_seed is not None and m.get("type") == "PlaceGraphPlanner":
            m.setdefault("params", {})["explore_seed"] = int(planner_seed)
        if kind == "vision" and m.get("type") == "VisualBearing":
            p = m.setdefault("params", {})
            p["lesion_after_ticks"] = int(t1) if lesion else -1
            p["lesion_until_ticks"] = int(t2) if lesion else -1
        if kind == "stick" and m.get("type") == "VisualBearing":
            p = m.setdefault("params", {})
            p["stick_after_ticks"] = int(t1) if lesion else -1
            p["stick_until_ticks"] = int(t2) if lesion else -1
    if kind == "scent_noise":
        md["scent_noise_sigma"] = 0.05 if lesion else 0.0
        md["scent_noise_after_ticks"] = int(t1); md["scent_noise_until_ticks"] = int(t2)
    if kind == "heading_drift":
        md["heading_drift_sigma"] = 0.01 if lesion else 0.0
        md["heading_drift_after_ticks"] = int(t1); md["heading_drift_until_ticks"] = int(t2)
    fs = CFGDIR / f"_dtmp_{tag}.json"
    json.dump(cfg, open(fs, "w"))
    return f"res://addons/ami_ogma/configs/_dtmp_{tag}.json", fs


def run(res, ticks, t1, t2, wseed=42, obs_seed=None, label=""):
    env = dict(os.environ)
    env.update(OGMA_SEED=str(wseed), OGMA_CELL_CONFIG=res, OGMA_TURBO="1", OGMA_FWDLOG="1",
               OGMA_EPISODE_LENGTH=str(ticks), OGMA_QUIT_AFTER_TICKS=str(ticks + 60))
    if obs_seed is not None:
        env["OGMA_OBSTACLE_SEED"] = str(obs_seed)
    print(f"[seed] {label} OGMA_SEED={wseed} OGMA_OBSTACLE_SEED={obs_seed if obs_seed is not None else '(unset -> OGMA_SEED)'}", flush=True)
    cmd = ["timeout", "--signal=TERM", str(max(120, ticks // 60 * 3)),
           GODOT, "--path", str(PROJ), "--headless", "--fixed-fps", "60",
           "--disable-render-loop", "res://scenes/the_cell.tscn"]
    p = subprocess.run(cmd, env=env, capture_output=True, text=True)
    # phase-bin: eats (sparse, honest) + mean food-distance (DENSE, the primary (d) signal) + vision food-in-view
    eats = [0, 0, 0]
    hfood = [0, 0, 0]; diag = [0, 0, 0]
    fd_sum = [0.0, 0.0, 0.0]; fd_n = [0, 0, 0]
    def phase(t): return 0 if t < t1 else (1 if t < t2 else 2)
    for ln in p.stdout.splitlines():
        ln = ln.strip()
        if not ln.startswith("{"):
            continue
        try:
            r = json.loads(ln)
        except json.JSONDecodeError:
            continue
        ev = r.get("event")
        if ev == "HIT":
            eats[phase(int(r.get("t", 0)))] += 1
            continue
        if ev == "FWDLOG":
            fd = float(r.get("fdist", -1.0))
            if fd >= 0.0:
                ph = phase(int(r.get("t", 0))); fd_sum[ph] += fd; fd_n[ph] += 1
            continue
        if "pos" in r:
            ph = phase(int(r.get("t", 0)))
            diag[ph] += 1
            for v in r.get("modules", {}).values():
                if isinstance(v, dict) and "hfood" in v and v.get("hfood"):
                    hfood[ph] += 1
    return {"eats": eats, "hfood_frac": [round(hfood[i] / diag[i], 3) if diag[i] else 0.0 for i in range(3)],
            "mean_fdist": [round(fd_sum[i] / fd_n[i], 2) if fd_n[i] else None for i in range(3)],
            "diag": diag, "stderr_tail": p.stderr.strip().splitlines()[-2:] if p.returncode else []}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--ticks", type=int, default=45000)
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--lesion", choices=["vision", "stick", "scent_noise", "heading_drift"], default="vision")
    ap.add_argument("--vary-world", action="store_true",
                    help="per-job world (food sites, spawn, pillar layout) with its own OGMA_SEED/OGMA_OBSTACLE_SEED, paired across arms")
    ap.add_argument("--world-base", type=int, default=1000)
    ap.add_argument("--legacy-seeding", action="store_true", help="the pre-2026-09-06 behaviour: OGMA_SEED=42 for every run")
    args = ap.parse_args()
    t1, t2 = args.ticks // 3, 2 * args.ticks // 3     # lesion = the middle third
    eseeds = [11 + i for i in range(args.n)]

    tmp = []
    jobs = []   # (arm, res, wseed, obs_seed, label)
    for arm, lesion in (("lesion", True), ("control", False)):
        for i, es in enumerate(eseeds):
            ws = 42 if args.legacy_seeding else args.world_base + i
            world = gen_world(ws) if args.vary_world else None
            if world is not None and args.legacy_seeding:
                world.pop("obstacle_seed", None)
            res, fs = patch(args.config, lesion, t1, t2, es, f"{arm}_e{es}", kind=args.lesion, world=world,
                            planner_seed=None if args.legacy_seeding else es + 7919)
            tmp.append(fs); jobs.append((arm, res, ws, ws, f"arm={arm} es={es}"))
    results = {"lesion": [], "control": []}
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
            futs = {ex.submit(run, res, args.ticks, t1, t2, ws, obs, lab): arm for arm, res, ws, obs, lab in jobs}
            for fut in concurrent.futures.as_completed(futs):
                results[futs[fut]].append(fut.result())
    finally:
        for fs in tmp:
            try: fs.unlink()
            except FileNotFoundError: pass

    print(f"\n===== (d) PERTURBATION [{args.lesion}] — n={args.n}, {args.ticks} ticks, lesion window [{t1},{t2})"
          f"{', varied worlds' if args.vary_world else ', one world'}{' (legacy seeding)' if args.legacy_seeding else ''} =====")
    print(f"    phases (ticks each ~{t1}): PRE (vision on) | LESION (blind) | POST (restored)\n")
    def col(rows, i, key): return [r[key][i] for r in rows if r[key][i] is not None]
    def m(v): return f"{statistics.mean(v):.2f}" if v else "-"
    for arm in ("lesion", "control"):
        rows = [r for r in results[arm] if r["diag"][0] > 0]
        if not rows:
            print(f"  [{arm}] NO DATA"); [print("   ", l) for r in results[arm] for l in r["stderr_tail"]]; continue
        fd = [col(rows, i, "mean_fdist") for i in range(3)]
        hf = [col(rows, i, "hfood_frac") for i in range(3)]
        e  = [col(rows, i, "eats") for i in range(3)]
        print(f"  [{arm:7}] mean food-DIST (m): PRE {m(fd[0])}  |  LESION {m(fd[1])}  |  POST {m(fd[2])}   "
              f"(vision cut → can't approach → DIST RISES)")
        print(f"  {'':10} eats/phase: PRE {m(e[0])} | LES {m(e[1])} | POST {m(e[2])}   "
              f"food-in-view: PRE {m(hf[0])} | LES {m(hf[1])} | POST {m(hf[2])}  (LES≈0 = vision confirmed blind)")
    # verdict — DENSE metric = mean food-distance. Degradation = dist RISES under lesion; recovery = FALLS back.
    L = [r for r in results["lesion"] if r["diag"][0] > 0]
    C = [r for r in results["control"] if r["diag"][0] > 0]
    if L:
        pre, les, post = (statistics.mean(col(L, i, "mean_fdist")) for i in range(3))
        cles = statistics.mean(col(C, 1, "mean_fdist")) if C else None
        deg = les - pre; rec = les - post
        print(f"\n  (d) SIGNATURE (mean food-distance, lesion arm):  degradation Δ(lesion−pre)={deg:+.2f} m  "
              f"recovery Δ(lesion−post)={rec:+.2f} m")
        if cles is not None:
            print(f"      contrast @ lesion phase: lesion {les:.2f} m vs control {cles:.2f} m  "
                  f"(lesion further from food = vision was the approach driver)")
        ok = deg > 0 and rec > 0 and (cles is None or les > cles)
        print(f"      {'PASS' if ok else 'WEAK/NULL'}: perturbation → degradation (can't approach) → recovery"
              f" — the (d) bar for a load-bearing inference loop")
    return 0


if __name__ == "__main__":
    sys.exit(main())
