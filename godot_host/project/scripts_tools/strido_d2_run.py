#!/usr/bin/env python3
"""PART V stage D2 — the displacement experiment under the C2 criterion.

    python3 gainevo_make_arms.py d2        # emit the 18 arm configs first
    python3 strido_d2_run.py [outdir]      # then this (18 runs x 600k ticks, ~6 h)

The tsw2 protocol exactly (arena, difficulty 0.3, 600k ticks, OGMA_SEED 1-6,
arms sigma0 / fix008 / fix020, displaced start written into BOTH the evolver seed
and the MotorEPM params) with the C2 criterion applied by the config generator.
Pre-registration: stride_odometry_and_criterion_repair_plan.md §3-D (D2 block),
committed before any run.  Analyze with gainevo_tsweep.py <outdir>.
"""
import concurrent.futures as cf
import os, pathlib, subprocess, sys, time

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
ARMS = ["sigma0", "fix008", "fix020"]
SEEDS = range(1, 7)
MAX_STEPS = 600000
CONCURRENCY = 6      # 12 oversubscribed instances measured 4x slower per run (D1)


def run_one(arm, seed, outdir, port):
    log = f"{outdir}/{arm}_s{seed}.log"
    cfg = f"d2_{arm}_s{seed}.json"
    env = dict(os.environ,
               OGMA_PICRAWLER_GYM="arena", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(port),
               OGMA_PICRAWLER_GYM_DIFFICULTY="0.3",
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(MAX_STEPS))
    t0 = time.time()
    with open(log, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "20000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"],
                       cwd=PROJ, env=env, stdout=f, stderr=subprocess.STDOUT)
    ok = any(cfg in line for line in open(log, errors="replace"))
    print(f"  {arm} s{seed}: {time.time()-t0:.0f}s config_confirmed={ok}", flush=True)
    return ok


def main(outdir):
    os.makedirs(outdir, exist_ok=True)
    jobs = [(arm, s, outdir, 7700 + k)
            for k, (arm, s) in enumerate((a, s) for a in ARMS for s in SEEDS)]
    print(f"D2: {len(jobs)} runs x {MAX_STEPS} ticks -> {outdir}")
    with cf.ThreadPoolExecutor(max_workers=CONCURRENCY) as ex:
        ok = list(ex.map(lambda j: run_one(*j), jobs))
    print(f"done: {sum(ok)}/{len(ok)} config-confirmed")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         os.path.expanduser(time.strftime("~/xaq_runs/stridoD2_%Y%m%d")))
