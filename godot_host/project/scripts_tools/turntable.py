#!/usr/bin/env python3
"""The TURN TABLE (S1/S2 of the MicroDuck port): the gait's open-loop yaw response to a constant
body-relative goal bearing — the picrawler's analogue of the duck's yaw calibration table.

Runs the S2 config in the ARENA (flat, no walls near the spawn) with OGMA_PICRAWLER_GOAL_BEARING_DEG
at each error, n seeds each, and reports per error: the mean yaw rate over the run (from the
unwrapped heading, deg/s; + = toward body +X), the forward speed while turning, the path's
curvature (yaw per metre), falls.  A constant body-relative bearing never closes, so the steady
rate IS the table entry; reach_k for the arbiter's speed gate comes from it.

Usage:  python3 turntable.py [config] [n=3] [steps=2000] [degs=0,15,45,90,180,-15,-45,-90]
Logs: $SEEDAVG_OUT or /tmp/xaq_turntable.
"""
import json, math, os, pathlib, statistics, subprocess, sys, concurrent.futures as cf

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_turntable"); os.makedirs(SP, exist_ok=True)
DEFAULT_CFG = ("the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__native_measured"
               "__tofboom__fsrleg__honest__nohomeo__room_senses__s2.json")

def config_body_env(cfg):
    try:
        meta = json.load(open(f"{PROJ}/addons/ami_ogma/configs/{cfg}")).get("metadata", {})
        benv = meta.get("body_env", {})
        return {str(k): str(v) for k, v in benv.items()} if isinstance(benv, dict) else {}
    except Exception:
        return {}

def run_one(cfg, seed, steps, deg, port):
    out = f"{SP}/turn_{deg:+.0f}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="arena", OGMA_PICRAWLER_GYM_DIFFICULTY="0.3",
               OGMA_SEED=str(seed), OGMA_INSPECTOR_PORT=str(port),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(steps),
               OGMA_PICRAWLER_GOAL_BEARING_DEG=str(deg))
    for k, v in config_body_env(cfg).items():
        if k not in os.environ: env[k] = v
    with open(out, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out)

def parse(path):
    ts = []; yaws = []; xs = []; zs = []; fv = []; falls = 0
    for line in open(path):
        if not line.startswith("{") or '"x":' not in line: continue
        try: d = json.loads(line)
        except Exception: continue
        ts.append(d.get("t", 0)); yaws.append(d.get("heading_yaw", 0.0)); xs.append(d["x"]); zs.append(d["z"])
        if d.get("t", 0) >= 300: fv.append(d.get("fwd_v", 0.0))
        falls = d.get("auto_reset_count", falls)
    # unwrap the yaw, then rate = total turn / time (ticks at 50 Hz)
    un = [yaws[0]] if yaws else [0.0]
    for y in yaws[1:]:
        dy = y - (un[-1] % (2 * math.pi)) if False else y - yaws[len(un) - 1]
        while dy > math.pi: dy -= 2 * math.pi
        while dy < -math.pi: dy += 2 * math.pi
        un.append(un[-1] + dy)
    turn = un[-1] - un[0] if len(un) > 1 else 0.0
    secs = (ts[-1] - ts[0]) / 50.0 if len(ts) > 1 else 1.0
    path_len = sum(math.hypot(xs[i]-xs[i-1], zs[i]-zs[i-1]) for i in range(1, len(xs)))
    return dict(rate_dps=math.degrees(turn) / max(1e-6, secs), fwd_v=(statistics.mean(fv) if fv else 0.0),
                curv=(turn / path_len if path_len > 0.05 else 0.0), path=path_len, falls=falls)

if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith(".json") else DEFAULT_CFG
    rest = sys.argv[2:] if len(sys.argv) > 1 and sys.argv[1].endswith(".json") else sys.argv[1:]
    n = int(rest[0]) if len(rest) > 0 else 3
    steps = int(rest[1]) if len(rest) > 1 else 2000
    degs = [float(v) for v in rest[2].split(",")] if len(rest) > 2 else [0, 15, 45, 90, 180, -15, -45, -90]
    print(f"TURN TABLE · {cfg} · arena · n={n} · {steps} ticks · bearing + = body +X")
    print(f"  {'error':>7} | {'yaw rate (deg/s)':>18} | {'fwd_v':>14} | {'rad/m':>14} | {'path (m)':>13} | falls")
    for di, deg in enumerate(degs):
        with cf.ThreadPoolExecutor(max_workers=n) as ex:
            res = list(ex.map(lambda s: run_one(cfg, s, steps, deg, 7700 + 20 * di + 2 * s), range(1, n + 1)))
        def ms(k):
            a = [r[k] for r in res]; return statistics.mean(a), (statistics.stdev(a) if len(a) > 1 else 0.0)
        r, rs = ms("rate_dps"); f, fs = ms("fwd_v"); c, cs = ms("curv"); p, ps = ms("path")
        print(f"  {deg:+7.0f} | {r:+8.2f} ± {rs:6.2f} | {f:+6.3f} ± {fs:5.3f} | {c:+6.3f} ± {cs:5.3f} | {p:6.3f} ± {ps:5.3f} | {sum(x['falls'] for x in res)}")
