#!/usr/bin/env python3
"""Perturbation-RECOVERY gate: does the learned coordination come back after the robot
gets stuck, or does it lock in whatever it was doing at the time?

This is the (d) test for any always-on adaptation.  The operator's hard-won lesson:
**permanent crystallization can backfire** — an earlier mechanism learned a destructive
escape movement while stuck and then could not relearn to walk.  So a lever that adapts
`gait_phase` forever must be shown to DRIFT under perturbation and RETURN afterwards.

Structurally there are two kinds of adaptation in MotorEPM and only one is safe to leave on:
  * `coord_adapt_rate`     — leaky first-order tracker toward the body's CURRENT measured
                             phase pattern.  No stored winner ⇒ nothing to lock in.
  * `coord_reward_drive`   — (1+1) hill-climb that KEEPS the best-fitness probe and reverts
                             to it.  A thrash that momentarily scores high fwd_v becomes the
                             incumbent forever.  This is the shape that burned us.

Repeatedly teleports the robot onto the hump crest (OGMA_PICRAWLER_TELEPORT_EVERY) and
reports, per seed: how far gait_phase wandered, whether it came back, and whether forward
progress resumed after each hit.

Usage: recoveravg.py <config.json> [n_seeds] [max_steps] [difficulty] [teleport_every]
"""
import json, math, os, statistics, subprocess, sys, concurrent.futures as cf
import pathlib

# Derived from this script's own location so a fresh clone works anywhere
# (was a hardcoded home directory, which broke every non-author checkout).
PROJ = str(pathlib.Path(__file__).resolve().parents[1])
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_recover")
os.makedirs(SP, exist_ok=True)

def run_one(cfg, seed, max_steps, difficulty, every):
    out = f"{SP}/rec_{os.path.splitext(cfg)[0]}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7600+seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_PICRAWLER_TELEPORT_RAMP_AT="1200",
               OGMA_PICRAWLER_TELEPORT_EVERY=str(every),
               OGMA_PICRAWLER_TELEPORT_XZ="0,2.6",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(max_steps))
    with open(out, "w") as f:
        subprocess.run(["godot4","--headless","--fixed-fps","60","--quit-after","4000000",
                        "--path",".","res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out)

def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def parse(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if '"x":' not in line or not line.startswith('{'): continue
        try: d = json.loads(line)
        except: continue
        if 'x' in d: rows.append(d)
    if len(rows) < 10: return None
    gp = [r['gait_phase'] for r in rows if isinstance(r.get('gait_phase'), list)]
    if not gp: return None
    base = gp[0]
    # Distance of the live offsets from where they STARTED (wrapped, per leg, RMS).
    dist = [math.sqrt(sum(wrap(v - b)**2 for v, b in zip(g, base)) / len(g)) for g in gp]
    n = len(dist)
    peak = max(dist)                       # how far the coordination wandered
    final = statistics.mean(dist[-max(1, n//10):])   # where it settled at the end
    # Recovery = did it come back toward baseline after the excursion?  1.0 = fully
    # returned, 0.0 = stayed at its worst.  Undefined if it never wandered.
    recov = (peak - final) / peak if peak > 1e-6 else float('nan')
    zs = [r['z'] for r in rows]
    # Progress in the LAST third — after every perturbation has already happened.  A
    # locked-in destructive pattern shows up here as a body that no longer advances.
    k = max(2, n // 3)
    late = zs[-k:]
    return dict(gp_peak=peak, gp_final=final, gp_recovered=recov,
                late_progress=late[-1] - late[0], net_z=zs[-1] - zs[0],
                falls=max(r.get('auto_reset_count', 0) for r in rows))

if __name__ == "__main__":
    cfg = sys.argv[1]
    n     = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 12000
    diff  = sys.argv[4] if len(sys.argv) > 4 else "0.3"
    every = int(sys.argv[5]) if len(sys.argv) > 5 else 2400
    with cf.ThreadPoolExecutor(max_workers=min(8, n)) as ex:
        res = list(ex.map(lambda s: run_one(cfg, s, steps, diff, every), range(1, n+1)))
    ok = [r for r in res if r]
    print(f"\nRECOVERY GATE  {cfg}  (n={len(ok)}/{n}, {steps} ticks, teleport every {every})")
    if not ok: print("  no valid runs (is gait_phase in the body diag?)"); sys.exit(1)
    for k in ("gp_peak","gp_final","gp_recovered","late_progress","net_z","falls"):
        vals = [r[k] for r in ok if not (isinstance(r[k], float) and math.isnan(r[k]))]
        if not vals: print(f"  {k:<14} n/a (offsets never moved — adaptation off?)"); continue
        m, s = statistics.mean(vals), statistics.pstdev(vals)
        print(f"  {k:<14} mean={m:+.3f}  std={s:.3f}   "
              f"[{'  '.join(f'{v:+.2f}' for v in vals)}]")
