#!/usr/bin/env python3
"""Seed-averaging harness for the ROOM gym (S1 of the MicroDuck port plan, 2026-10-10).

Runs a config across N seeds in the walled room with the forward ultrasonic model ON, and
reports the wall-avoidance metric set beside the gait metrics:

  first_contact  tick of the first wall/box contact (−1 = never)
  wall_frac      fraction of ticks with the chassis or a lower leg against a wall/box
  episodes       contact episodes (rising edges)
  coverage       distinct 0.25 m floor cells visited (the blind-metric complement: a body
                 that spins at the centre scores zero walls — report coverage beside it)
  net_disp / path / straight   as seedavg
  falls          auto_reset_count at the end
  us_valid       fraction of diag rows with an echo
  us_err         mean |us_r − us_true| over valid rows (the lobe's nearest vs the axis)
  confound       rows where the AXIS sees a surface within range but the ping had no echo
                 (a glancing surface reading as open floor — the channel's built-in confound)

Usage:  python3 roomavg.py <config_basename.json> [n_seeds=6] [max_steps=6000] [room_size=3.0] [extra_env=...]
Logs go to $SEEDAVG_OUT or /tmp/xaq_roomavg.  Seeds run concurrently on inspector ports 7500+10*seed.
"""
import json, math, os, pathlib, statistics, subprocess, sys, concurrent.futures as cf

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_roomavg")
os.makedirs(SP, exist_ok=True)

def config_body_env(cfg):
    try:
        meta = json.load(open(f"{PROJ}/addons/ami_ogma/configs/{cfg}")).get("metadata", {})
        benv = meta.get("body_env", {})
        return {str(k): str(v) for k, v in benv.items()} if isinstance(benv, dict) else {}
    except Exception:
        return {}

def run_one(cfg, seed, max_steps, room_size, extra):
    out = f"{SP}/room_{os.path.splitext(cfg)[0]}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="room", OGMA_PICRAWLER_ULTRASONIC="1",
               OGMA_PICRAWLER_ROOM_SIZE=str(room_size), OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7500 + 10 * seed),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(max_steps))
    for k, v in config_body_env(cfg).items():
        if k not in os.environ:
            env[k] = v
    for kv in extra:
        k, _, v = kv.partition("="); env[k] = v
    with open(out, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out)

def parse(path):
    xs = []; zs = []; first = -1; wt = 0; ep = 0; cov = 0; falls = 0; n = 0; t_last = 0
    us_valid = 0; us_err = []; confound = 0; us_rows = 0
    for line in open(path):
        if not line.startswith("{") or '"x":' not in line: continue
        try: d = json.loads(line)
        except Exception: continue
        n += 1; t_last = d.get("t", 0)
        xs.append(d["x"]); zs.append(d["z"])
        wt = d.get("wall_contact_ticks", wt); ep = d.get("wall_contact_episodes", ep)
        cov = d.get("room_cov", cov); falls = d.get("auto_reset_count", falls)
        if first < 0 and wt > 0: first = d.get("t", 0)
        if "us_r" in d:
            us_rows += 1
            if d["us_v"] > 0.5:
                us_valid += 1
                if d["us_true"] >= 0: us_err.append(abs(d["us_r"] - d["us_true"]))
            elif 0 <= d["us_true"] <= 1.5:
                confound += 1
    path_len = sum(math.hypot(xs[i] - xs[i-1], zs[i] - zs[i-1]) for i in range(1, len(xs)))
    net = math.hypot(xs[-1] - xs[0], zs[-1] - zs[0]) if xs else 0.0
    return dict(first_contact=first, wall_frac=wt / max(1, t_last), episodes=ep, coverage=cov,
                net_disp=net, path=path_len, straight=(net / path_len if path_len > 0 else 0.0),
                falls=falls, us_valid=us_valid / max(1, us_rows),
                us_err=(statistics.mean(us_err) if us_err else 0.0), confound=confound)

if __name__ == "__main__":
    cfg = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 6000
    room = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
    extra = sys.argv[5:]
    with cf.ThreadPoolExecutor(max_workers=n) as ex:
        res = list(ex.map(lambda s: run_one(cfg, s, steps, room, extra), range(1, n + 1)))
    keys = ["first_contact", "wall_frac", "episodes", "coverage", "net_disp", "path", "straight",
            "falls", "us_valid", "us_err", "confound"]
    print(f"ROOM {room:.1f} m · {cfg} · n={n} · {steps} ticks" + (f" · {' '.join(extra)}" if extra else ""))
    for k in keys:
        v = [r[k] for r in res]
        m = statistics.mean(v); sd = statistics.stdev(v) if len(v) > 1 else 0.0
        print(f"  {k:14s} {m:9.3f} ± {sd:7.3f}   per seed: " + " ".join(f"{x:.3f}" if isinstance(x, float) else str(x) for x in v))
