#!/usr/bin/env python3
"""Seed-averaged HUMP-traversal gate for the picrawler corridor.

Teleports an already-walking robot onto the leading crest of the corridor hump
(OGMA_PICRAWLER_TELEPORT_RAMP_AT + OGMA_PICRAWLER_TELEPORT_XZ=0,2.6) and reports
whether it gets over.  This is the OBSTACLE-REGRESSION GATE: obstacle handling is
currently good, so any flat-speed lever must clear this before it can be promoted.

Reports final_z (post-teleport) + belly clearance + falls.  A clean clear is
final_z well past the hump base (z=4); ~2.6 means it high-centred and stalled.

Usage: humpavg.py <config_basename.json> [n_seeds] [max_steps] [difficulty] [teleport_at]
"""
import json, os, statistics, subprocess, sys, concurrent.futures as cf

PROJ = "/home/xaqmusic/xaq-ai/godot_host/project"
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_seedavg")
os.makedirs(SP, exist_ok=True)
WARMUP_PAD = 600          # settle window after the teleport before scoring posture

def run_one(cfg, seed, max_steps, difficulty, tp_at):
    out = f"{SP}/hump_{os.path.splitext(cfg)[0]}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7500+seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_PICRAWLER_TELEPORT_RAMP_AT=str(tp_at),
               OGMA_PICRAWLER_TELEPORT_XZ="0,2.6",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(max_steps))
    with open(out, "w") as f:
        subprocess.run(["godot4","--headless","--fixed-fps","60","--quit-after","4000000",
                        "--path",".","res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out, tp_at)

def parse(path, tp_at):
    rows = []
    for line in open(path):
        line = line.strip()
        if '"x":' not in line or not line.startswith('{'): continue
        try: d = json.loads(line)
        except: continue
        if 'x' in d: rows.append(d)
    post = [d for d in rows if d.get('t', 0) > tp_at]
    if len(post) < 3: return None
    gc = [d['gc_raw'] for d in post if 'gc_raw' in d and d.get('t',0) > tp_at + WARMUP_PAD]
    zs = [d['z'] for d in post]
    return dict(final_z=zs[-1], max_z=max(zs), gain_z=zs[-1]-zs[0],
                bellyc=statistics.mean(gc) if gc else 0.0,
                bellyc_min=min(gc) if gc else 0.0,
                falls=max(d.get('auto_reset_count', 0) for d in post))

if __name__ == "__main__":
    cfg = sys.argv[1]
    n     = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 9000
    diff  = sys.argv[4] if len(sys.argv) > 4 else "0.3"
    tp_at = int(sys.argv[5]) if len(sys.argv) > 5 else 3000
    seeds = list(range(1, n+1))
    with cf.ThreadPoolExecutor(max_workers=min(8, n)) as ex:
        res = list(ex.map(lambda s: run_one(cfg, s, steps, diff, tp_at), seeds))
    ok = [r for r in res if r]
    print(f"\nHUMP GATE  {cfg}  (n={len(ok)}/{n}, {steps} ticks, diff {diff}, teleport@{tp_at})")
    if not ok: print("  no valid runs"); sys.exit(1)
    for k in ("final_z","max_z","gain_z","bellyc","bellyc_min","falls"):
        vals = [r[k] for r in ok]
        m, s = statistics.mean(vals), statistics.pstdev(vals)
        fmt = ".3f" if k.startswith("belly") else ".2f"
        print(f"  {k:<11} mean={m:+{fmt}}  std={s:{fmt}}   "
              f"[{'  '.join(f'{v:{fmt}}' for v in vals)}]")
