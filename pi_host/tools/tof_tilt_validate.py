#!/usr/bin/env python3
"""Does the boom tilt correction have the right SIGN and MAGNITUDE?  Measured, not assumed.

⚠ THE TEST IS NOT "belly height stays constant".  Pitching the robot by changing leg lengths
changes the belly height too, so nothing should stay constant and a test built on that would
prove nothing.  What IS predictable from the measured attitude alone is the size of the
correction itself:

    comp - uncomp = (d - H)(up.y - 1) - boom_z*up.z   ~=  -boom_z*up.z

So with boom_z = -0.070, comp_delta should track +0.070*up.z to within the sub-0.1 mm second
term -- independent of how high the belly actually is.  A sign error doubles the artefact
instead of removing it and still looks plausible, so the sign is checked separately.

Tilt is induced by trimming the FRONT knees (ch3 FL, ch9 FR) against the REAR ones (ch0 RL,
ch6 RR) from the `stand` pose, sweeping both directions.  The achieved pitch is READ from the
IMU rather than predicted, so no leg kinematics are needed.

⚠ THE ROBOT PITCHES NOSE-UP AND NOSE-DOWN.  Aborts on excess tilt, bus errors, or current.
"""
import json, math, statistics as st, sys, time, zmq
ENDPOINT = "tcp://127.0.0.1:5590"; _ctx = zmq.Context()
FRONT_KNEES, REAR_KNEES = (3, 9), (0, 6)
STEPS = (0, 40, 80, 120, -40, -80, -120, 0)
PITCH_ABORT_DEG, I_ABORT_A = 22.0, 2.6

def rpc(verb, _allow_err=False, **kw):
    s = _ctx.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 25000); s.setsockopt(zmq.LINGER, 0)
    s.connect(ENDPOINT); s.send_string(json.dumps({"verb": verb, **kw}))
    try: r = json.loads(s.recv_string())
    except zmq.Again: r = {"ok": False, "error": "TIMEOUT"}
    finally: s.close()
    if not r.get("ok") and not _allow_err: raise RuntimeError(f"{verb}: {r.get('error')}")
    return r

def pose_us(n): return rpc("pose.get", name=n)["us"]
def vec(v): return list(v) if isinstance(v,(list,tuple)) else [v.get("x",0),v.get("y",0),v.get("z",0)]
def settle(t=40):
    t0=time.time()
    while time.time()-t0 < t:
        s=rpc("status")
        if not s.get("pose_move_active") and not s.get("pose_queue"): return
        time.sleep(0.1)

stand = pose_us("stand")
lim = rpc("limits.set", confirm=True)
print(f"stand knees: " + "  ".join(f"ch{c}={stand[c]}" for c in FRONT_KNEES+REAR_KNEES))
print(f"pose_slew {lim['pose_slew_us']}  stagger {lim['stagger_ms']} ms\n")
print(f"{'trim':>5} {'pitch°':>7} {'up.z':>8} {'raw':>6} {'uncomp':>8} {'comp':>8} "
      f"{'delta':>8} {'predicted':>10} {'resid':>8}")
rows = []
try:
    rpc("pose.set", us=stand); settle(); time.sleep(1.5)
    for trim in STEPS:
        us = list(stand)
        for c in FRONT_KNEES: us[c] = stand[c] + trim
        for c in REAR_KNEES:  us[c] = stand[c] - trim
        rpc("pose.set", us=us); settle(); time.sleep(1.5)
        pit, upz, raw, unc, cmp_, dlt = [], [], [], [], [], []
        t0 = time.time()
        while time.time() - t0 < 3.0:
            s = rpc("status"); t = s.get("tof") or {}; imu = s.get("imu") or {}; ina = s.get("ina") or {}
            if (ina.get("i_a") or 0) > I_ABORT_A: raise RuntimeError(f"current {ina['i_a']:.2f} A — abort")
            if s["bus_errors"]: raise RuntimeError("I2C bus errors — abort")
            if imu.get("ok"):
                u = vec(imu["up_fused"])
                p = math.degrees(math.atan2(-u[2], u[1]))
                if abs(p) > PITCH_ABORT_DEG: raise RuntimeError(f"pitch {p:.1f}° — abort")
                pit.append(p); upz.append(u[2])
            if t.get("ok") and t.get("valid") and t.get("comp_valid"):
                raw.append(t["raw_mm"]); unc.append(t["m"]); cmp_.append(t["m_comp"]); dlt.append(t["comp_delta"])
            time.sleep(0.1)
        if not (pit and dlt): print(f"{trim:>5}   no valid samples"); continue
        P, Z = st.median(pit), st.median(upz)
        D, R, U, C = st.median(dlt), st.median(raw), st.median(unc), st.median(cmp_)
        pred = -(-0.070) * Z          # -boom_z*up.z with boom_z = -0.070
        print(f"{trim:>5} {P:>7.2f} {Z:>8.4f} {R:>6.1f} {U*1000:>7.2f}m {C*1000:>7.2f}m "
              f"{D*1000:>+7.2f}m {pred*1000:>+9.2f}m {(D-pred)*1000:>+7.2f}m")
        rows.append((trim, P, D, pred))
finally:
    rpc("pose.set", _allow_err=True, us=pose_us("rescue"))

if len(rows) >= 4:
    resid = [abs(d - p) * 1000 for _, _, d, p in rows]
    print(f"\nmagnitude: worst residual vs -boom_z*up.z is {max(resid):.3f} mm "
          f"({'PASS' if max(resid) < 0.5 else 'REVIEW'}; the second-order term is <0.1 mm)")
    tilted = [(P, D) for _, P, D, _ in rows if abs(P) > 1.0]
    if tilted:
        wrong = [(P, D) for P, D in tilted if P * D > 0]
        print(f"sign: {len(tilted)-len(wrong)}/{len(tilted)} tilted points have "
              f"nose-up(+pitch) -> negative correction "
              f"({'PASS' if not wrong else 'FAIL — a sign error DOUBLES the artefact'})")
        span = max(abs(D) for _, D in tilted) * 1000
        print(f"span: the correction reached {span:.1f} mm — against the ~1.5 mm of pose signal "
              f"§9.9 measured, i.e. {span/1.5:.0f}x the thing it defends")
