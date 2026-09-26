#!/usr/bin/env python3
"""The boom's true elevation over the floor, referenced to BELLY-DOWN (the X pose).

§9.2 fitted mount_offset_mm = 64.8 on 2026-09-07 by anchoring at belly-down.  This re-reads
that reference directly: in X the legs go horizontal and the belly sits ON the floor, so the
median raw range IS the boom's standoff, and clearance in that pose must be ~0 by definition.

Also samples attitude, because the boom sits ~7 cm aft of centre (§9.1) and pitch therefore
couples into the reading at roughly 70*sin(theta) mm -- ~12 mm per 10 deg.  Hardware applies NO
tilt compensation today, so the pitch is recorded alongside every range rather than corrected.

⚠ THE ROBOT MOVES: rescue -> X -> rescue, at whatever pose slew the daemon is running.
"""
import json, math, statistics as st, sys, time, zmq
ENDPOINT = "tcp://127.0.0.1:5590"; _ctx = zmq.Context()

def rpc(verb, _allow_err=False, **kw):
    s = _ctx.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 25000); s.setsockopt(zmq.LINGER, 0)
    s.connect(ENDPOINT); s.send_string(json.dumps({"verb": verb, **kw}))
    try: r = json.loads(s.recv_string())
    except zmq.Again: r = {"ok": False, "error": "TIMEOUT"}
    finally: s.close()
    if not r.get("ok") and not _allow_err: raise RuntimeError(f"{verb}: {r.get('error')}")
    return r

def pose_us(n): return rpc("pose.get", name=n)["us"]

def vec(v):
    if isinstance(v, dict): return [v.get("x",0.0), v.get("y",0.0), v.get("z",0.0)]
    return list(v) if isinstance(v, (list, tuple)) else [0.0,0.0,0.0]

def tilt_deg(up):
    x, y, z = vec(up)
    # the shared contract, ogma::body::pitch_roll_from_up
    return (math.degrees(math.atan2(-z, y)), math.degrees(math.atan2(x, y)))

def settle(t=40):
    t0 = time.time()
    while time.time() - t0 < t:
        s = rpc("status")
        if not s.get("pose_move_active") and not s.get("pose_queue"): return
        time.sleep(0.1)

def sample(label, secs):
    raw, m, pit, rol, sig, amb, bad = [], [], [], [], [], [], 0
    t0 = time.time()
    while time.time() - t0 < secs:
        s = rpc("status"); t = s.get("tof") or {}; imu = s.get("imu") or {}
        if t.get("ok"):
            if t.get("valid", True) and t.get("raw_mm"):
                raw.append(t["raw_mm"]); m.append(t.get("m"))
                sig.append(t.get("signal", 0)); amb.append(t.get("ambient", 0))
            else: bad += 1
        if imu.get("ok"):
            p, r = tilt_deg(imu.get("up_fused")); pit.append(p); rol.append(r)
        time.sleep(0.1)
    if not raw:
        print(f"{label:9s}  NO VALID SAMPLES ({bad} invalid)"); return None
    sd = st.stdev(raw) if len(raw) > 1 else 0.0
    print(f"{label:9s} raw {st.median(raw):6.1f} mm  sd {sd:4.2f}  "
          f"min {min(raw):5.1f}  max {max(raw):5.1f}  n={len(raw):3d} bad={bad:2d}   "
          f"clearance {st.median([x for x in m if x is not None]):+.4f} m")
    if pit:
        print(f"{'':9s} pitch {st.median(pit):+6.2f}°  roll {st.median(rol):+6.2f}°   "
              f"signal {st.median(sig):.2f}  ambient {st.median(amb):.2f} Mcps")
    return st.median(raw), sd, (st.median(pit) if pit else None)

lim = rpc("limits.set", confirm=True)     # read back without changing anything
print(f"pose_slew {lim['pose_slew_us']}  stagger {lim['stagger_ms']} ms   "
      f"stored mount_offset 64.8 mm\n")
res = {}
res["rescue0"] = sample("rescue", 5.0)
rpc("pose.set", us=pose_us("X")); settle(); time.sleep(1.5)
res["X"] = sample("X (belly)", 8.0)
rpc("pose.set", us=pose_us("rescue")); settle(); time.sleep(1.5)
res["rescue1"] = sample("rescue", 5.0)

print()
if res["X"]:
    raw_x, sd_x, pitch_x = res["X"]
    print(f"BELLY-DOWN REFERENCE: boom standoff = {raw_x:.1f} mm  (sd {sd_x:.2f})")
    print(f"  stored mount_offset_mm = 64.8  ->  delta {raw_x - 64.8:+.1f} mm")
    print(f"  clearance in X must be ~0 by definition; with the stored offset it reads "
          f"{(raw_x - 64.8)/1000:+.4f} m")
    if pitch_x is not None:
        print(f"  chassis pitch in X: {pitch_x:+.2f}° — at a 70 mm aft boom that is "
              f"{70*math.sin(math.radians(pitch_x)):+.1f} mm of the reading")
if res["rescue0"] and res["rescue1"]:
    a, b = res["rescue0"][0], res["rescue1"][0]
    print(f"\nrescue REPEATABILITY: {a:.1f} vs {b:.1f} mm  ->  {abs(a-b):.1f} mm apart "
          f"({'consistent' if abs(a-b) < 6 else 'NOT consistent — same pose, different reading'})")
