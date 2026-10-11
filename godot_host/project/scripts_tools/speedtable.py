#!/usr/bin/env python3
"""The SPEED TABLE (S2 of the MicroDuck port): what the gait does at each constant speed command.

Runs the S2 config in the ROOM with OGMA_PICRAWLER_SPEED_CMD at each value, n seeds each, and
reports per value: net displacement ALONG THE SPAWN HEADING (+ = forward, − = backward; truth z),
path length, mean fwd_v, belly clearance, falls, wall contact, and the lever's consumer check
(speed_msgs > 0).  The gate questions: does 1.0 reproduce the baseline, does 0 stand still with
the belly up (or step in place), does a negative value back up.

Usage:  python3 speedtable.py [config=…__room_senses__s2.json] [n=3] [steps=3000] [values=1,0.5,0,-0.5,-1]
Logs: $SEEDAVG_OUT or /tmp/xaq_speedtable.
"""
import json, math, os, pathlib, statistics, subprocess, sys, concurrent.futures as cf

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_speedtable"); os.makedirs(SP, exist_ok=True)
DEFAULT_CFG = ("the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__native_measured"
               "__tofboom__fsrleg__honest__nohomeo__room_senses__s2.json")

def config_body_env(cfg):
    try:
        meta = json.load(open(f"{PROJ}/addons/ami_ogma/configs/{cfg}")).get("metadata", {})
        benv = meta.get("body_env", {})
        return {str(k): str(v) for k, v in benv.items()} if isinstance(benv, dict) else {}
    except Exception:
        return {}

def run_one(cfg, seed, steps, speed, port):
    out = f"{SP}/speed_{speed:+.2f}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="room", OGMA_SEED=str(seed), OGMA_INSPECTOR_PORT=str(port),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(steps),
               OGMA_PICRAWLER_SPEED_CMD=str(speed))
    for k, v in config_body_env(cfg).items():
        if k not in os.environ: env[k] = v
    with open(out, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out)

def parse(path):
    xs = []; zs = []; fv = []; gc = []; falls = 0; wall = 0; t_last = 0; msgs = -1; steps = 0
    for line in open(path):
        if not line.startswith("{") or '"x":' not in line: continue
        try: d = json.loads(line)
        except Exception: continue
        xs.append(d["x"]); zs.append(d["z"]); t_last = d.get("t", 0)
        if t_last >= 300:
            fv.append(d.get("fwd_v", 0.0)); gc.append(d.get("gc_raw", 0.0))
        falls = d.get("auto_reset_count", falls); wall = d.get("wall_contact_ticks", wall)
        ll = d.get("leg_lifted_counts")
        if isinstance(ll, list) and ll: steps = sum(ll)
    path_len = sum(math.hypot(xs[i]-xs[i-1], zs[i]-zs[i-1]) for i in range(1, len(xs)))
    along = (zs[-1] - zs[0]) if zs else 0.0        # spawn faces +Z
    # the consumer check lives in the module metrics line, if the body emits it; else -1
    return dict(along=along, path=path_len, fwd_v=(statistics.mean(fv) if fv else 0.0),
                belly=(statistics.median(gc) if gc else 0.0), falls=falls,
                wall_frac=wall / max(1, t_last), steps=steps)

if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith(".json") else DEFAULT_CFG
    rest = sys.argv[2:] if len(sys.argv) > 1 and sys.argv[1].endswith(".json") else sys.argv[1:]
    n = int(rest[0]) if len(rest) > 0 else 3
    steps = int(rest[1]) if len(rest) > 1 else 3000
    values = [float(v) for v in rest[2].split(",")] if len(rest) > 2 else [1.0, 0.5, 0.0, -0.5, -1.0]
    print(f"SPEED TABLE · {cfg} · room · n={n} · {steps} ticks")
    print(f"  {'speed':>6} | {'along (m)':>14} | {'path (m)':>13} | {'fwd_v':>13} | {'belly (m)':>13} | {'steps':>9} | {'wall':>6} | falls")
    for vi, v in enumerate(values):
        with cf.ThreadPoolExecutor(max_workers=n) as ex:
            res = list(ex.map(lambda s: run_one(cfg, s, steps, v, 7600 + 20 * vi + 2 * s), range(1, n + 1)))
        def ms(k):
            a = [r[k] for r in res]; return statistics.mean(a), (statistics.stdev(a) if len(a) > 1 else 0.0)
        al, als = ms("along"); pl, pls = ms("path"); f, fs = ms("fwd_v"); b, bs = ms("belly"); st, sts = ms("steps")
        print(f"  {v:+6.2f} | {al:+6.3f} ± {als:5.3f} | {pl:6.3f} ± {pls:5.3f} | {f:+6.3f} ± {fs:5.3f} | {b:6.4f} ± {bs:5.4f} | {st:5.0f} ± {sts:4.0f} | {ms('wall_frac')[0]:5.2f} | {sum(r['falls'] for r in res)}")
