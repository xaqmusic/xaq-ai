#!/usr/bin/env python3
"""PART V stage D1 — coupling landscape under the old vs the C1 criterion.

    python3 strido_d1_run.py [outdir]

Pre-registration (predictions + decision rule) is in
`stride_odometry_and_criterion_repair_plan.md` §3-D, committed before any run.
Protocol = the coupling-authority shape: σ=0 observer (j1s4's evolver is frozen),
OGMA_PICRAWLER_SETPARAM_AT steps motor_epm.coupling_gain through six levels,
three scored 12k windows per level, warmup 10000.  Seeds 1-6; odd ascend, even
descend (the analyzers' convention).  Two arms on the SAME seeds:

  old/  j1s4     — the unmodified criterion (control; must replicate the basin)
  c1/   j1s4_c1  — travel_topic=stride_v + flow_min_form=1 + flow_turn_k=4

Bodies are behavior-identical across arms (measured at the C1 commit), so any J
difference is the criterion alone.  Analyze each arm dir with
coupling_authority.py; the D1 verdict applies the pre-registered rule.

⚠ OGMA_SEED is the master seed override (harness-traps memory); this is how the
seeds are varied, on purpose.
"""
import concurrent.futures as cf
import os, pathlib, subprocess, sys, time

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
CFG = "the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__%s.json"
ARMS = [("old", CFG % "j1s4"), ("c1", CFG % "j1s4_c1")]
LEVELS_ASC = [0.0, 0.4, 0.8, 1.2, 1.6, 2.0]
WARMUP, WINDOW, WPL = 10000, 12000, 3
MAX_STEPS = WARMUP + len(LEVELS_ASC) * WPL * WINDOW + 2000   # 228k: slack for the last flush
SEEDS = range(1, 7)
CONCURRENCY = 12


def schedule(asc):
    levels = LEVELS_ASC if asc else list(reversed(LEVELS_ASC))
    ents = [f"1:motor_epm:coupling_gain:{levels[0]}"]
    for i, lv in enumerate(levels[1:], start=1):
        ents.append(f"{WARMUP + i * WPL * WINDOW}:motor_epm:coupling_gain:{lv}")
    return ";".join(ents)


def run_one(arm, cfg, seed, outdir, port):
    asc = seed % 2 == 1
    log = f"{outdir}/{arm}/s{seed}_{'asc' if asc else 'desc'}.log"
    os.makedirs(os.path.dirname(log), exist_ok=True)
    env = dict(os.environ,
               OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(port),
               OGMA_PICRAWLER_GYM_DIFFICULTY="0.3",
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(MAX_STEPS),
               OGMA_PICRAWLER_SETPARAM_AT=schedule(asc))
    t0 = time.time()
    with open(log, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "8000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"],
                       cwd=PROJ, env=env, stdout=f, stderr=subprocess.STDOUT)
    ok = any(cfg in line for line in open(log, errors="replace"))
    print(f"  {arm} s{seed}: {time.time()-t0:.0f}s config_confirmed={ok}", flush=True)
    return ok


def main(outdir):
    jobs = [(arm, cfg, s, outdir, 7500 + k)
            for k, (arm, cfg, s) in enumerate((a, c, s) for a, c in ARMS for s in SEEDS)]
    print(f"D1: {len(jobs)} runs x {MAX_STEPS} ticks -> {outdir}")
    with cf.ThreadPoolExecutor(max_workers=CONCURRENCY) as ex:
        ok = list(ex.map(lambda j: run_one(*j), jobs))
    print(f"done: {sum(ok)}/{len(ok)} config-confirmed")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         os.path.expanduser(time.strftime("~/xaq_runs/stridoD1_%Y%m%d")))
