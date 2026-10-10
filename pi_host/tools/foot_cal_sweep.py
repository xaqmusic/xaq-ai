#!/usr/bin/env python3
"""Probe every foot sensor with the robot's own weight.  (Foot-FSR bench step E6.)

The mass series with weights and a kitchen scale is slow, and §5.6.1 records what it
costs: the leg is hand-posed for every foot, so `cos theta` differs between them, the
unpowered pose sags under load, and the dwell is whatever the operator's hands did that
time.  The robot can load its own feet instead -- with its own mass, through its own load
path, in the pose regime it actually operates in -- and it can do it identically on all
four, as many times as anyone likes.

WHAT THIS MEASURES, AND WHAT IT DOES NOT

  It measures COUNTS against a POSE STEP, per foot, sweeping up and back down.
  It does NOT convert to grams.

That is deliberate.  Grams needs the foot's contact point, which means FK, and
`ogma::body::fk_leg` needs per-leg world anchors benchd does not build -- and cpp_core's
own CMakeLists records why a second copy of that maths in Python is the wrong answer:
that is how sim and host drift apart.  It would also inherit a KNOWN-WRONG constant: the
printed foot added ~16.9 mm to L3 and the geometry has not been re-baselined
(picrawler_foot_fsr_mod.md §5.1), so an absolute force computed today is wrong by up to
20 % before anything else goes in.

What needs no geometry at all is the COMPARISON, and the comparison is the open question.
§5.7.7 found the same 175 g reading as 56 g, 105 g, 170 g and 370 g across the four feet,
and could not say whether the stalls in two of them were repeatable nonlinearity (which a
per-foot curve absorbs) or slack (which it cannot).  Equal microsecond steps are equal
ANGLE steps on every leg -- `us_per_rad` is a property of the part, not the channel -- so
stepping each leg through the same offsets from its own mirrored pose puts the four feet
under the same geometry, and an up-and-down sweep answers the repeatability question
directly.  Both without a single length constant.

⚠ THE ROBOT MOVES.  It props one corner up off a belly-down pose and works the knee.

⚠⚠ THE KEEPALIVE IS NOT OPTIONAL.  benchd's deadman (DEADMAN_MS = 1000) commands the
saved RESCUE pose when no client command is fresh, and this HAT cannot limp a servo at
all, so "safe" is a pose and not slack.  calib/sensors.json records what that cost the
servo-scale measurement: a rescue excursion of 100-200 us looks like nothing happening
while it silently replaces the commanded angle.  A keepalive thread runs here throughout.

USAGE
  foot_cal_sweep.py                          # all four feet, default steps
  foot_cal_sweep.py --feet FL,RL
  foot_cal_sweep.py --steps -150,-75,0,75,150 --dwell 2.0
  foot_cal_sweep.py --pose-suffix _down      # front_left_down, rear_right_down, ...

The poses must already exist (pose.save).  Each puts its own foot down and leaves the
opposite side of the chassis flat on the ground as the lever.
"""
import argparse, json, math, os, statistics as st, sys, threading, time
import zmq

ENDPOINT = "tcp://127.0.0.1:5590"
# foot -> (pose stem, ADC channel).  Anatomy, not sim names -- see the port doc's
# leg-naming mirror and the wiring doc §5 table.
FEET = {"FL": ("front_left", 0), "FR": ("front_right", 1),
        "RL": ("rear_left", 2), "RR": ("rear_right", 3)}

_ctx = zmq.Context()
_lock = threading.Lock()
_stop = threading.Event()


def rpc(verb, _allow_err=False, **kw):
    """⚠ RAISES on ok:false unless _allow_err.  rail_separation_test.py's header records
    why: a harness that discards error replies cannot detect that it is doing nothing, and
    one that did invalidated four measurements on a robot that never moved."""
    with _lock:
        s = _ctx.socket(zmq.REQ)
        s.setsockopt(zmq.RCVTIMEO, 20000)
        s.setsockopt(zmq.LINGER, 0)
        s.connect(ENDPOINT)
        s.send_string(json.dumps({"verb": verb, **kw}))
        try:
            r = json.loads(s.recv_string())
        except zmq.Again:
            r = {"ok": False, "error": "TIMEOUT"}
        finally:
            s.close()
    if not r.get("ok") and not _allow_err:
        raise RuntimeError(f"{verb} failed: {r.get('error')}  (sent {kw})")
    return r


def keepalive():
    while not _stop.is_set():
        try:
            rpc("ping")
        except Exception:
            pass                      # a dropped ping is not worth ending the sweep
        _stop.wait(0.2)


def settle(eta_ms, timeout=25.0):
    """⚠ Wait for the move to START before waiting for it to END.  The first version of
    rail_separation_test.py called this straight after pose.set and got True because the
    move had not begun -- "settled" meaning "has not started" is the same lie as a starved
    sampler."""
    t0 = time.time()
    while time.time() - t0 < min(3.0, 1.0 + eta_ms / 1000.0):
        if rpc("status").get("pose_move_active"):
            break
        time.sleep(0.05)
    while time.time() - t0 < timeout:
        if not rpc("status").get("pose_move_active"):
            return True
        time.sleep(0.05)
    return False


def knee_channel(mapping, physical):
    for sv in mapping.get("servos", []):
        if sv.get("physical") == physical and sv.get("joint") == "knee":
            return sv.get("ch"), sv.get("sign", 1)
    raise RuntimeError(f"no knee channel mapped for {physical} -- run cal.map first")


def tilt_deg(st):
    """Body tilt from vertical, out of the IMU's fused gravity estimate.

    ⚠ THIS IS THE INSTRUMENT THAT SAYS WHETHER THE POSE STEP DID ANYTHING.  The first run
    (2026-10-01) produced counts that were rock-steady at each hold and NOT monotone across
    the sweep -- because a belly resting flat is an indeterminate contact and the body
    re-seats between steps.  Counts alone cannot tell that apart from a bad sensor.  Tilt
    can: if it does not climb with the step, the lever is not doing what the sweep assumes
    and the numbers are the harness, not the feet."""
    imu = st.get("imu") or {}
    up = imu.get("up_fused")
    if not up or len(up) != 3:
        return None
    n = math.sqrt(sum(c * c for c in up)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, up[1] / n))))


def sample(ch, n, gap):
    """n live ADC reads of one channel plus the body tilt.  `status` rebuilds the frame, and
    frame() reads the ADC, so every call is a fresh conversion rather than a cached one."""
    vals, tilts = [], []
    for _ in range(n):
        st_ = rpc("status")
        vals.append(st_["adc"][ch])
        t = tilt_deg(st_)
        if t is not None:
            tilts.append(t)
        time.sleep(gap)
    return vals, (st.fmean(tilts) if tilts else float("nan"))


def report(results, steps, feet):
    # ---- hysteresis: the question this sweep exists to answer --------------------
    print("\n=== hysteresis  (up minus down at the same step; slack shows here)\n")
    print(f"    {'foot':<6}" + "".join(f"{s:>+9}" for s in steps[:-1]) + f"{'worst':>10}")
    for foot, rows in results.items():
        up = {s: m for s, d, m, *_ in rows if d == "up"}
        dn = {s: m for s, d, m, *_ in rows if d == "down"}
        row, worst = "", 0.0
        for s in steps[:-1]:
            if s in up and s in dn:
                diff = up[s] - dn[s]
                worst = max(worst, abs(diff))
                row += f"{diff:>+9.0f}"
            else:
                row += f"{'-':>9}"
        print(f"    {foot:<6}{row}{worst:>10.0f}")
    print("\n    A loop that closes -- worst comparable to the sd column above -- is a"
          "\n    repeatable nonlinearity, and a per-foot curve absorbs it."
          "\n    ⚠ A loop that does NOT close is slack in the toe-to-foot linkage, and no"
          "\n    curve can represent it: the reading then depends on how the load was reached.")

    print("\n=== cross-foot, up sweep  (same us step = same angle on every leg)\n")
    print(f"    {'step':>7}" + "".join(f"{f:>9}" for f in feet))
    for s in steps:
        row = ""
        for f in feet:
            m = next((m for s2, d, m, *_ in results[f] if s2 == s and d == "up"), None)
            row += f"{'-':>9}" if m is None else f"{m:>9.0f}"
        print(f"    {s:>+7}{row}")
    print("\n    ⚠ These are counts, not grams.  The columns are comparable to each other"
          "\n    because the geometry is mirrored; none of them is an absolute force.")

    print("\n=== did the lever actually move?  body tilt (deg from vertical), up sweep\n")
    print(f"    {'step':>7}" + "".join(f"{f:>9}" for f in feet))
    for s in steps:
        row = ""
        for f in feet:
            t = next((r[6] for r in results[f]
                      if r[0] == s and r[1] == "up" and len(r) > 6), None)
            row += f"{'-':>9}" if t is None else f"{t:>9.2f}"
        print(f"    {s:>+7}{row}")
    print("\n    ⚠ Tilt must climb monotonically with the step, because that is the lever the"
          "\n    whole method rests on.  If it does not, the body is re-seating between steps"
          "\n    and the counts above are a measurement of the harness, not of the feet.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--feet", default="FL,FR,RL,RR")
    ap.add_argument("--steps", default="-150,-100,-50,0,50,100,150",
                    help="knee offsets in us from the pose, in the leg's own sign convention")
    ap.add_argument("--dwell", type=float, default=2.0, help="seconds held before reading")
    ap.add_argument("--reads", type=int, default=12)
    ap.add_argument("--pose-suffix", default="_down")
    ap.add_argument("--ref", type=int, default=None, metavar="US",
                    help="after every step, return to this knee offset and read it again. "
                         "The spread of those readings is mechanical repeatability with NO "
                         "model in it -- no tilt, no interpolation, no force. Default: the "
                         "middle step. Pass 99999 to disable")
    ap.add_argument("--via", default="toes_up",
                    help="pose to pass through before each <foot>_down; '' to go direct. "
                         "⚠ the toes drag on the way into a _down pose, and on a high-friction "
                         "surface that is both a damage risk and a source of residual")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    feet = [f.strip().upper() for f in a.feet.split(",") if f.strip()]
    steps = [int(x) for x in a.steps.split(",")]
    for f in feet:
        if f not in FEET:
            sys.exit(f"unknown foot {f!r} -- expected any of {', '.join(FEET)}")

    st0 = rpc("status")
    mapping = st0.get("map", {})
    poses = rpc("pose.list")["poses"]
    print(f"benchd up · body={st0.get('body')} · tick_hz={st0.get('tick_hz')} · vbat={st0.get('vbat')}")

    via = (a.via or "").strip() or None
    missing = [FEET[f][0] + a.pose_suffix for f in feet
               if FEET[f][0] + a.pose_suffix not in poses]
    if via and via not in poses:
        missing.append(via)
    if missing:
        sys.exit("missing pose(s): " + ", ".join(missing) +
                 "\n  have: " + ", ".join(sorted(poses)))
    print(f"transit pose: {via or '(none — going direct, toes drag)'}")

    ref = None if a.ref == 99999 else (a.ref if a.ref is not None else steps[len(steps) // 2])
    if ref is not None and ref not in steps:
        sys.exit(f"--ref {ref} is not one of the steps {steps}")
    print(f"reference step: {'(disabled)' if ref is None else format(ref, '+d') + ' us'}")
    ref_summary = {}
    threading.Thread(target=keepalive, daemon=True).start()
    rpc("adc.rate", ms=20)            # the full time series, settle transients included
    results = {}
    try:
        for foot in feet:
            stem, adc_ch = FEET[foot]
            pose_name = stem + a.pose_suffix
            ch, sign = knee_channel(mapping, foot)
            base = rpc("pose.get", name=pose_name)["us"]
            print(f"\n=== {foot}  pose {pose_name}  knee ch{ch} (sign {sign:+d})  ADC A{adc_ch}")
            # ⚠ Unload the toes before the long travel into the next _down pose.  Going
            # straight from one corner-up stance to the next drags a LOADED toe across the
            # surface, which on a high-friction mat strains the glued leg joint and the wire
            # tie -- and leaves the foot somewhere its own sweep did not put it.
            if via:
                rpc("mark", text=f"footcal {foot} transit {via}")
                vr = rpc("pose.set", us=rpc("pose.get", name=via)["us"])
                settle(vr.get("eta_ms", 0))
                time.sleep(0.6)
                br = rpc("pose.set", us=base)      # then down onto this foot, toes unloaded
                settle(br.get("eta_ms", 0))
                time.sleep(0.6)
            print(f"    {'step us':>9}{'dir':>6}{'counts':>9}{'sd':>7}{'min':>7}{'max':>7}{'tilt°':>8}")

            rows = []                 # (step, direction, mean, sd, min, max, tilt)
            refs = []                 # readings taken back at the reference step
            order = [(s, "up") for s in steps] + [(s, "down") for s in reversed(steps[:-1])]
            for step, direction in order:
                us = list(base)
                want = base[ch] + sign * step
                us[ch] = want
                rpc("mark", text=f"footcal {foot} knee{step:+d}us {direction}")
                r = rpc("pose.set", us=us)
                got = r["us"][ch]
                if abs(got - want) > 1:
                    print(f"    ⚠ ch{ch} clamped {want} -> {got} -- step is beyond the envelope")
                if not settle(r.get("eta_ms", 0)):
                    print("    ⚠ move did not settle in time; reading anyway")
                time.sleep(a.dwell)
                v, tilt = sample(adc_ch, a.reads, 0.05)
                m = st.fmean(v)
                sd = st.pstdev(v) if len(v) > 1 else 0.0
                rows.append((step, direction, m, sd, min(v), max(v), tilt))
                print(f"    {step:>+9}{direction:>6}{m:>9.1f}{sd:>7.2f}{min(v):>7}{max(v):>7}{tilt:>8.2f}")
                # ⚠ Return to one fixed pose and read it again.  Everything else in this
                # sweep compares readings taken at DIFFERENT poses, which needs tilt as a
                # stand-in for load and an interpolation to line them up.  This does not:
                # same commanded pose, same foot, minutes apart.  Whatever spread shows up
                # here is mechanical and cannot be argued away.
                if ref is not None and step != ref:
                    us2 = list(base); us2[ch] = base[ch] + sign * ref
                    rpc("mark", text=f"footcal {foot} ref{ref:+d}us after {step:+d} {direction}")
                    r2 = rpc("pose.set", us=us2)
                    settle(r2.get("eta_ms", 0))
                    time.sleep(max(0.8, a.dwell * 0.6))
                    v2, t2 = sample(adc_ch, max(6, a.reads // 2), 0.05)
                    refs.append((step, direction, st.fmean(v2), t2))
            results[foot] = rows
            if refs:
                vals = [m for _, _, m, _ in refs]
                tl = [t for *_, t in refs if t == t]
                print(f"    --- back at {ref:+d} us, {len(vals)} returns: "
                      f"mean {st.fmean(vals):.0f}  spread {max(vals) - min(vals):.0f}  "
                      f"sd {st.pstdev(vals):.0f}" +
                      (f"   tilt sd {st.pstdev(tl):.2f}°" if len(tl) > 1 else ""))
                ref_summary[foot] = (st.fmean(vals), max(vals) - min(vals), st.pstdev(vals),
                                     st.pstdev(tl) if len(tl) > 1 else float("nan"), len(vals))
    finally:
        # ⚠ DO NOT finish with `limp`.  It commands the saved RESCUE pose, whose toes sweep
        # out far enough to catch on the wall of a safety box -- observed 2026-10-01, and the
        # operator had to lift the robot clear several times.  Park on the transit pose
        # instead: toes up, nothing to snag, and it is a pose this sweep already trusts.
        #
        # ⚠ This cannot cover being KILLED.  benchd's deadman commands rescue ~1 s after the
        # keepalive stops, so an interrupted run ends in rescue no matter what is written
        # here.  Let the run finish, or raise the timeout -- do not shorten it.
        rpc("adc.rate", ms=0, _allow_err=True)
        parked = False
        if via:
            try:
                pr = rpc("pose.set", us=rpc("pose.get", name=via)["us"])
                settle(pr.get("eta_ms", 0))
                parked = True
            except Exception as e:
                print(f"⚠ could not park on {via}: {e}")
        _stop.set()
        print(f"\nsampler off, robot parked on {via if parked else 'whatever it last held'}.")

    if ref_summary:
        print("\n=== mechanical repeatability: the SAME commanded pose, revisited\n")
        print(f"    {'foot':<6}{'n':>4}{'mean':>9}{'spread':>9}{'sd':>8}{'tilt sd':>10}")
        for f, (mn, sp, sd, tsd, n) in ref_summary.items():
            print(f"    {f:<6}{n:>4}{mn:>9.0f}{sp:>9.0f}{sd:>8.0f}{tsd:>10.2f}")
        print("\n    ⚠ No model in this table -- same pose, same foot, minutes apart.  Compare"
              "\n    `sd` against the per-hold sd in the tables above: that is the electrical"
              "\n    floor.  Anything well beyond it is the mechanics, and `tilt sd` says how"
              "\n    much of it the body failed to return to.")

    report(results, steps, feet)

    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "log",
                                time.strftime("footcal_%Y%m%d_%H%M%S.json"))
    with open(out, "w") as f:
        json.dump({"steps": steps, "dwell_s": a.dwell, "reads": a.reads,
                   "pose_suffix": a.pose_suffix,
                   "feet": {k: [list(r) for r in v] for k, v in results.items()}}, f, indent=2)
    print(f"\nraw sweep: {os.path.normpath(out)}")
    print("full 50 Hz series, settle transients included, is in the benchd JSONL "
          "(adc_fast records, split on the `footcal` marks)")


if __name__ == "__main__":
    main()
