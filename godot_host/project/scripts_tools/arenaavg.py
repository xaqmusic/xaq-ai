#!/usr/bin/env python3
"""Seed-averaged harness for the ARENA gym (the open floor the operator watches in).

WHY THIS EXISTS.  Every other tool here hardcodes `OGMA_PICRAWLER_GYM="corridor"`, and the
corridor is built *"inside self-centering 30 deg walls"* (`picrawler_body.gd:2684`).  That
geometry actively re-centers the body, so a yaw excursion -- from a swing leg's reaction
torque, say -- gets corrected by the wall before it shows up in `straight` or `turns`.
Measuring an anti-spin lever there tests it in the one gym where its target failure is
suppressed.  The arena is an open floor with 45 deg outward wedges only at the perimeter,
so heading disturbance is free to express itself and be measured.

Corollary that also matters: `net_z` is meaningless in an open arena (the robot may leave
in any direction).  `net_disp` and `straight` are the honest progress metrics here.

THE MECHANISM METRIC.  `yaw_swing_excess` = mean |yaw rate| while ANY foot is airborne,
minus the all-four-down reference.  That is the operator's UI observation stated as a
number: does swinging a limb spin the chassis?  `yaw_per_leg` attributes it per limb, so
"it's the back legs" is checkable rather than impressionistic.  A lever aimed at swing
dynamics must move THIS, not merely distance -- otherwise a null is unreadable.

Usage:  arenaavg.py <config_basename.json> [n_seeds] [max_steps] [difficulty] [K=V ...]
Logs go to $SEEDAVG_OUT or /tmp/xaq_seedavg.
"""
import json, math, os, statistics, subprocess, sys, concurrent.futures as cf
import pathlib

# Derived from this script's own location so a fresh clone works anywhere
# (was a hardcoded home directory, which broke every non-author checkout).
PROJ = str(pathlib.Path(__file__).resolve().parents[1])
SP = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_seedavg")
os.makedirs(SP, exist_ok=True)

# Same spawn transient skip as seedavg.py: the robot spawns tucked and settles.
WARMUP_TICKS = int(os.environ.get("SEEDAVG_WARMUP", 900))
# Foot is PLANTED when feet_y < this (matches picrawler_body.gd).  Used only for the
# static-support column; the swing metrics below come from the TRUE contact sensor.
STANCE_TH = float(os.environ.get("OGMA_PICRAWLER_STANCE_Y_THRESHOLD", 0.04))
# The arena floor is 20x20 with perimeter wedges; past this radius the run is up against
# the boundary and `falls`/`chassis_y` stop measuring the gait (see seedavg.py's guard).
ARENA_SAFE_R = float(os.environ.get("ARENAAVG_SAFE_R", 8.0))


def run_one(cfg, seed, max_steps, difficulty, extra):
    out = f"{SP}/ar_{os.path.splitext(cfg)[0]}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="arena", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7700 + seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(max_steps))
    for kv in extra:
        k, _, v = kv.partition("=")
        env[k] = v
    with open(out, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out)


def parse(path):
    xs, zs, hy, ar, ts = [], [], [], [], []
    gc, cy, tilt, lat, planted = [], [], [], [], []
    tib, fr = [], []
    lifts = lifts0 = None
    last = None
    for line in open(path):
        line = line.strip()
        if '"x":' not in line or not line.startswith("{"):
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        xs.append(d["x"]); zs.append(d["z"]); hy.append(d.get("heading_yaw", 0.0))
        ar.append(d.get("auto_reset_count", 0)); ts.append(d.get("t", 0))
        if "yaw_swing_excess" in d:
            last = d
        if d.get("t", 0) < WARMUP_TICKS:
            continue
        if "gc_raw" in d: gc.append(d["gc_raw"])
        cy.append(d.get("y", 0.0)); tilt.append(d.get("tilt", 0.0))
        lat.append(abs(d.get("lateral_v", 0.0)))
        fy = d.get("feet_y")
        if isinstance(fy, list) and fy:
            planted.append(sum(1 for v in fy if v < STANCE_TH))
        # POSTURE, from the CAD kinematics (docs/operational/picrawler_geometry.md:
        # L1 coxa 37.26, L2 femur 53.6, L3 tibia 75.5 mm; rest pose = hip2 0 (femur PARALLEL
        # to ground) + knee -80 deg, which puts the tibia 10 deg off vertical).  In that
        # convention the tibia's angle from vertical is hip2 + knee + pi/2.
        #   tib_off  -- how far the shank is from vertical.  The design rest is 10 deg; the
        #               deployed gait measures ~37 deg, i.e. the robot walks sprawled.
        #               !! 2026-08-28: this formula assumes hinge 0 = straight leg.  The
        #               body's hinges read 0 at the CONSTRUCTION pose (tibia already dropped
        #               80 deg), so tib_off UNDER-READS the sprawl by ~30 deg and foot_r
        #               below is chassis-centre -> tibia MIDPOINT, not hip1 -> toe.  Both
        #               are kept unchanged for continuity with the ledger; the honest
        #               posture/reach numbers come from reach_check.py on the same logs.
        #   foot_r   -- horizontal foot radius = the MOMENT ARM the hip torque works through.
        #               Total straight-line leg reach is 166 mm, so a mean near that means
        #               the legs are running straight out at minimum mechanical advantage.
        # Read these WITH `scrub`: sprawled legs push outward, the lateral components largely
        # cancel left-to-right, and that work is thrown away.
        h2, kn = d.get("hip2"), d.get("knee")
        if isinstance(h2, list) and isinstance(kn, list) and len(h2) == len(kn):
            for a, b in zip(h2, kn):
                tib.append(abs(a + b + math.pi / 2))
        fxz = d.get("foot_xz")
        if isinstance(fxz, list):
            for q in fxz:
                if isinstance(q, list) and len(q) == 2: fr.append(math.hypot(q[0], q[1]))
        ll = d.get("leg_lifted_counts")
        if isinstance(ll, list) and ll:
            if lifts0 is None: lifts0 = list(ll)
            lifts = ll
    if len(xs) < 5:
        return None
    # RUN-COMPLETION GUARD (2026-08-10, mirrored from seedavg): a crashed run's log
    # simply stops, and scoring the corpse manufactured the walk-or-shuffle "basin
    # lottery" (ledger §4 ★★★★).  Truncated runs are excluded loudly, never averaged.
    expected = int(os.environ.get("SEEDAVG_EXPECT_TICKS", "0"))
    if expected > 0 and ts and ts[-1] < 0.95 * expected:
        print(f"  !! CRASHED/TRUNCATED RUN: {os.path.basename(path)} ended at t={ts[-1]}"
              f" of {expected} — excluded from averages")
        return dict(_crashed=True, last_t=ts[-1])

    # Unwrapped net rotation.  Kept only as a companion to `straight` -- on its own it is a
    # BLIND metric (a body that swings its heading and nets ~0 scores perfectly).
    acc, prev = hy[0], hy[0]
    for h in hy[1:]:
        dd = h - prev
        while dd > math.pi:  dd -= 2 * math.pi
        while dd < -math.pi: dd += 2 * math.pi
        acc += dd; prev = h
    path_len = sum(math.hypot(xs[i] - xs[i-1], zs[i] - zs[i-1]) for i in range(1, len(xs)))
    net_disp = math.hypot(xs[-1] - xs[0], zs[-1] - zs[0])
    max_r = max(math.hypot(x - xs[0], z - zs[0]) for x, z in zip(xs, zs))
    lp = [a - b for a, b in zip(lifts, lifts0)] if (lifts and lifts0) else (lifts or [])

    r = dict(
        net_disp=net_disp, path_len=path_len,
        straight=net_disp / path_len if path_len > 1e-6 else 0.0,
        turns=(acc - hy[0]) / (2 * math.pi), max_r=max_r,
        falls=max(ar),
        bellyc=statistics.mean(gc) if gc else 0.0,
        bellyc_min=min(gc) if gc else 0.0,
        chassis_y=statistics.mean(cy) if cy else 0.0,
        planted=statistics.mean(planted) if planted else 0.0,
        unstable=(sum(1 for p in planted if p < 3) / len(planted)) if planted else 0.0,
        tilt_sd=statistics.pstdev(tilt) if len(tilt) > 1 else 0.0,
        scrub=statistics.mean(lat) if lat else 0.0,
        steps=sum(lp), step_bal=(min(lp) / max(lp)) if (lp and max(lp) > 0) else 0.0,
        tib_off=statistics.mean(tib) * 180 / math.pi if tib else 0.0,
        foot_r=statistics.mean(fr) * 1000 if fr else 0.0,
    )
    # Mechanism metrics, carried from the last diag line (they are cumulative run means).
    for k in ("yaw_allplant", "yaw_anyswing", "yaw_swing_excess", "swing_tuck_frac",
              "yawd_allplant", "yawd_anyswing", "yawd_swing_excess",
              "sgate", "sgate_spr", "contact_duty", "td_plv", "pos_stance", "pos_swing",
              # P6 (2026-08-10): rhythm + detector + body-rhythm reads, all cumulative
              # run-means carried on the same diag lines (need gait_align_diag=1).
              "step_cv_real", "step_per_real", "stance_bout", "swing_bout",
              "short_bouts", "brt_plv", "brt_err", "brt_period"):
        r[k] = float(last.get(k, 0.0)) if last else 0.0
    # WINDOWED activity (P6): steps + displacement per quarter of the run — the ctrl_lr
    # ladder's activity PEAKS at 14–20k and decays, and an aggregate averages that away
    # (the 2026-08-02 protocol lesson).  Vectors, printed outside the scalar groups.
    if ts and lifts and lifts0:
        t0, t1 = ts[0], ts[-1]
        span = max(1, t1 - t0)
        marks = [t0 + span * q // 4 for q in range(5)]
        w_steps, w_disp = [], []
        # per-line cumulative lifts were only kept as first/last; re-scan cheaply:
        cum = []
        for line in open(path):
            if '"leg_lifted_counts"' not in line: continue
            try: d = json.loads(line.strip())
            except Exception: continue
            ll = d.get("leg_lifted_counts")
            if isinstance(ll, list) and ll:
                cum.append((d.get("t", 0), sum(ll), d.get("x", 0.0), d.get("z", 0.0)))
        for q in range(4):
            seg = [c for c in cum if marks[q] <= c[0] < marks[q + 1]]
            if len(seg) >= 2:
                w_steps.append(seg[-1][1] - seg[0][1])
                w_disp.append(math.hypot(seg[-1][2] - seg[0][2], seg[-1][3] - seg[0][3]))
            else:
                w_steps.append(0); w_disp.append(0.0)
        r["w_steps"] = w_steps
        r["w_disp"]  = [round(v, 2) for v in w_disp]
    r["yaw_per_leg"]  = (last or {}).get("yaw_per_leg", [])
    r["yawd_per_leg"] = (last or {}).get("yawd_per_leg", [])
    return r


def ms(v):
    v = [x for x in v if x == x]
    return (statistics.mean(v), statistics.pstdev(v)) if v else (float("nan"),) * 2


if __name__ == "__main__":
    cfg = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 6000
    diff = sys.argv[4] if len(sys.argv) > 4 else "0.3"
    extra = list(sys.argv[5:])
    seeds = list(range(1, n + 1))
    os.environ["SEEDAVG_EXPECT_TICKS"] = str(steps)   # arm the completion guard
    with cf.ThreadPoolExecutor(max_workers=min(8, n)) as ex:
        res = list(ex.map(lambda s: run_one(cfg, s, steps, diff, extra), seeds))
    crashed = [i + 1 for i, r in enumerate(res) if r and r.get("_crashed")]
    ok = [r for r in res if r and not r.get("_crashed")]
    print(f"\n{cfg}  [ARENA]  (n={len(ok)}/{n} seeds, {steps} ticks, diff {diff})")
    if crashed:
        print(f"  !! {len(crashed)}/{n} RUNS CRASHED (seeds {crashed}) — fix the build "
              f"before comparing")
    if not ok:
        print("  no valid runs"); sys.exit(1)

    GROUPS = (
        ("SWING DYNAMICS (the mechanism) -- yawd_* is the honest one; see the module header",
         ("yawd_swing_excess", "yawd_anyswing", "yawd_allplant",
          "yaw_swing_excess", "swing_tuck_frac")),
        ("PROGRESS", ("net_disp", "path_len", "straight", "turns", "max_r")),
        ("BELLY-UP", ("bellyc", "bellyc_min", "chassis_y")),
        ("STABILITY", ("planted", "unstable", "tilt_sd", "falls", "scrub")),
        ("POSTURE (sprawl) -- design rest: tib_off 10 deg; total leg reach 166 mm",
         ("tib_off", "foot_r")),
        ("RHYTHM", ("steps", "step_bal", "contact_duty", "td_plv", "pos_stance", "pos_swing")),
        # P6: true-contact regularity (micro-lifts filtered) + detector + body-rhythm
        # lock.  All 0.0 unless the arm sets gait_align_diag=1.
        ("STEP-CLOCK", ("step_cv_real", "step_per_real", "stance_bout", "swing_bout",
                        "short_bouts")),
        ("BODY-RHYTHM", ("brt_plv", "brt_err", "brt_period")),
    )
    for label, keys in GROUPS:
        print(f"  -- {label}")
        for k in keys:
            m, s = ms([r[k] for r in ok])
            fmt = ".1f" if k in ("tib_off", "foot_r") else \
                  ".4f" if k.startswith("yaw") or k in ("bellyc", "bellyc_min", "chassis_y",
                                                        "scrub", "tilt_sd") else ".2f"
            per = '  '.join(f"{r[k]:{fmt}}" for r in ok)
            print(f"     {k:<17} mean={m:+{fmt}}  std={s:{fmt}}   [{per}]")
    # Per-limb yaw attribution -- "it's the back legs" made checkable.  Order [FL,FR,RL,RR].
    if ok[0]["yawd_per_leg"]:
        cols = list(zip(*[r["yawd_per_leg"] for r in ok if r["yawd_per_leg"]]))
        print("  -- YAW IMPULSE BY LIMB (mean |d yaw rate| while that limb is airborne)")
        for name, col in zip(("FL", "FR", "RL", "RR"), cols):
            print(f"     {name:<17} mean={statistics.mean(col):+.4f}   [{'  '.join(f'{c:.4f}' for c in col)}]")
    # P6 windowed activity: quarter-run steps + displacement per seed — the honest read
    # of a peak-and-decay trajectory an aggregate would average away.
    if ok[0].get("w_steps"):
        print("  -- WINDOWS (quarter-run [Q1 Q2 Q3 Q4] per seed)")
        for nm, key in (("w_steps", "w_steps"), ("w_disp", "w_disp")):
            rows = "  |  ".join(" ".join(f"{v}" for v in r.get(key, [])) for r in ok)
            print(f"     {nm:<10} {rows}")
    near = [(i + 1, r["max_r"]) for i, r in enumerate(ok) if r["max_r"] > ARENA_SAFE_R]
    if near:
        print(f"\n  !! ARENA-BOUNDARY WARNING: {len(near)}/{len(ok)} seeds passed r={ARENA_SAFE_R}")
        for sd, mr in near:
            print(f"       seed {sd}: max_r={mr:.2f}")
        print("     `falls`/`chassis_y` are not trustworthy for those seeds -- shorten the run.")
