#!/usr/bin/env python3
"""Substrate-robustness harness: does the SAME brain produce a walk across every
sensor reference frame and actuation backend we can throw at it?

The operator's thesis (2026-07-26) is that the architecture is robust to what it is
sensing through — observed twice independently: five different foot-height reference
frames all walked, and the same brain walks on both the legacy hinge servos and the
g6dof joints. This turns that observation into a NUMBER that can be defended and
regression-tested, instead of an impression.

Sweeps a config across:
  * SENSOR axis    — feet_topic, i.e. the reference frame the swing gate reads
                     (god's-eye world-Y ... fully hardware-honest IMU-fused)
  * ACTUATION axis — OGMA_PICRAWLER_JOINT_BACKEND ∈ {hinge, g6dof}

and reports, per cell, whether a gait EMERGED against an up-front criterion — never
eyeballed after the fact:

    net_z > 1.0  AND  straight > 0.4  AND  falls == 0

Deliberately a LOW bar: the question is "did a walk emerge at all", not "was it fast".
Quality is what seedavg.py measures; this measures existence. A substrate that only
walks in the cell it was tuned for is not robust, however good that cell looks.

Prints an emergence matrix plus the spread of net_z across cells — a robust substrate
should show a HIGH emergence rate and a MODEST spread. Wide spread with full emergence
is the interesting middle case: robust in kind, sensitive in degree.

Usage: robustavg.py <config.json> [n_seeds] [max_steps] [difficulty]
"""
import json, math, os, statistics, subprocess, sys, concurrent.futures as cf

PROJ = "/home/xaqmusic/xaq-ai/godot_host/project"
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_robust")
os.makedirs(SP, exist_ok=True)

# The sensor axis: every foot-reference frame built during the oracle refactor, ordered
# from least to most physically honest.  `legal` records whether a real robot could
# produce it at all — the god's-eye entry is kept BECAUSE it is the historical baseline
# every earlier result was measured against.
SENSORS = [
    ("world-Y (god's-eye)",      "reality.proprio.feet_y",                  False),
    ("body-frame FK",            "reality.proprio.feet_y_body",             True),
    ("FK + belly ToF",           "reality.proprio.feet_y_ground",           True),
    ("true foot contact",        "reality.proprio.foot_contact",            True),
    ("gravity, achieved FK",     "reality.proprio.feet_y_gravity",          True),
    ("gravity, commanded FK",    "reality.proprio.feet_y_gravity_cmd",      True),
    ("gravity, accel-only IMU",  "reality.proprio.feet_y_gravity_cmd_acc",  True),
    ("gravity, fused IMU",       "reality.proprio.feet_y_gravity_cmd_imu",  True),
]
BACKENDS = ["hinge", "g6dof"]

# Emergence criterion — fixed BEFORE the runs, per CLAUDE.md §3 (decide the metric, then
# measure).  Intentionally generous; this asks whether a walk exists.
EMERGE_NET_Z    = 1.0
EMERGE_STRAIGHT = 0.4


def arm_config(base, topic, tag):
    """Write a single-field variant of `base` into the configs dir; return its filename."""
    cfg_dir = os.path.join(PROJ, "addons/ami_ogma/configs")
    with open(os.path.join(cfg_dir, base)) as f:
        d = json.load(f)
    mods = [m for m in d["modules"] if m.get("type") == "MotorEPM"]
    if len(mods) != 1:
        raise SystemExit(f"expected 1 MotorEPM in {base}, found {len(mods)}")
    mods[0]["params"]["feet_topic"] = topic
    d.setdefault("metadata", {})["name"] = f"ROBUSTNESS {tag}"
    out = f"{os.path.splitext(base)[0]}__rb_{tag}.json"
    with open(os.path.join(cfg_dir, out), "w") as f:
        json.dump(d, f, indent=1)
    return out


def run_one(cfg, seed, backend, max_steps, difficulty, port):
    out = f"{SP}/rb_{os.path.splitext(cfg)[0]}_{backend}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(port),
               OGMA_PICRAWLER_JOINT_BACKEND=backend,
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(max_steps))
    with open(out, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out)


def parse(path):
    xs, zs, ar = [], [], []
    for line in open(path):
        line = line.strip()
        if '"x":' not in line or not line.startswith('{'):
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        if 'x' in d:
            xs.append(d['x']); zs.append(d['z']); ar.append(d.get('auto_reset_count', 0))
    if len(zs) < 5:
        return None
    path_len = sum(math.hypot(xs[i]-xs[i-1], zs[i]-zs[i-1]) for i in range(1, len(xs)))
    net = math.hypot(xs[-1]-xs[0], zs[-1]-zs[0])
    return dict(net_z=zs[-1]-zs[0], straight=(net/path_len if path_len > 1e-6 else 0.0),
                falls=max(ar))


if __name__ == "__main__":
    base  = sys.argv[1]
    n     = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 6000
    diff  = sys.argv[4] if len(sys.argv) > 4 else "0.3"

    cells, port = [], 7700
    for si, (label, topic, legal) in enumerate(SENSORS):
        cfg = arm_config(base, topic, f"s{si}")
        for backend in BACKENDS:
            cells.append((label, legal, backend, cfg, port)); port += 40

    def do_cell(cell):
        label, legal, backend, cfg, p0 = cell
        with cf.ThreadPoolExecutor(max_workers=min(4, n)) as ex:
            res = list(ex.map(lambda s: run_one(cfg, s, backend, steps, diff, p0 + s), range(1, n+1)))
        ok = [r for r in res if r]
        if not ok:
            return (label, legal, backend, None)
        agg = dict(net_z=statistics.mean(r['net_z'] for r in ok),
                   straight=statistics.mean(r['straight'] for r in ok),
                   falls=max(r['falls'] for r in ok),
                   # Emergence is judged PER SEED, then required of all of them — a cell
                   # that works on 2 of 3 seeds is not a cell where the walk reliably emerges.
                   emerged=all(r['net_z'] > EMERGE_NET_Z and r['straight'] > EMERGE_STRAIGHT
                               and r['falls'] == 0 for r in ok),
                   n=len(ok))
        return (label, legal, backend, agg)

    print(f"\nROBUSTNESS MATRIX  {base}")
    print(f"  n={n} seeds/cell · {steps} ticks · diff {diff} · {len(cells)} cells")
    print(f"  emergence criterion (fixed in advance): net_z > {EMERGE_NET_Z} "
          f"AND straight > {EMERGE_STRAIGHT} AND falls == 0, on EVERY seed\n")
    print(f"  {'sensor reference':<26} {'legal':<6} {'backend':<7} "
          f"{'net_z':>7} {'straight':>9} {'falls':>6}  emerged")
    print("  " + "-" * 76)

    results = [do_cell(c) for c in cells]
    emerged_n = total_n = 0
    net_all = []
    for label, legal, backend, agg in results:
        if agg is None:
            print(f"  {label:<26} {'yes' if legal else 'NO':<6} {backend:<7} "
                  f"{'—':>7} {'—':>9} {'—':>6}  NO RUNS")
            total_n += 1
            continue
        total_n += 1
        emerged_n += 1 if agg['emerged'] else 0
        net_all.append(agg['net_z'])
        print(f"  {label:<26} {'yes' if legal else 'NO':<6} {backend:<7} "
              f"{agg['net_z']:>7.2f} {agg['straight']:>9.2f} {agg['falls']:>6.0f}  "
              f"{'YES' if agg['emerged'] else 'no'}")

    print("  " + "-" * 76)
    print(f"  EMERGENCE RATE: {emerged_n}/{total_n} cells "
          f"({100.0*emerged_n/max(1,total_n):.0f}%)")
    if net_all:
        lo, hi = min(net_all), max(net_all)
        print(f"  net_z spread across cells: {lo:.2f} … {hi:.2f} "
              f"({hi/lo:.1f}× ratio, mean {statistics.mean(net_all):.2f})")
        print("  Read together: a high emergence rate with a wide spread means the substrate")
        print("  is robust IN KIND but sensitive IN DEGREE — walks everywhere, walks well only")
        print("  in some cells. That is a different claim from 'sensor agnostic'.")
