#!/usr/bin/env python3
"""Validate the boom tilt correction at GAIT amplitude — and measure where it fails.

Two modes, because two different physical setups test two different things, and confusing them
is the trap this script exists to prevent (BOM §9.10.3):

  --mode shim    Front (or rear) feet raised on a block, FLOOR LEVEL.
                 Chassis pitches relative to HORIZONTAL ground -> the real gait case.
                 The correction should be EXACT here.  Self-checking: comp_delta is compared
                 against the full prediction from the measured attitude, so no ruler is needed.

  --mode slope   Whole robot on an inclined plane (a box with a smooth bottom, tilted).
                 Chassis ends up PARALLEL to the ground: pitch vs gravity is large, pitch vs
                 GROUND is zero, and the raw reading barely moves.  The correction should be
                 WRONG here, by boom_z*sin(alpha) -- 18 mm at 15 deg, over-reporting when
                 nose-down.  Needs a level reference first (--label level), then the incline.

⚠ WHY THE SLOPE TEST IS NOT A FAILED VALIDATION.  On a slope-aligned chassis the UNCOMPENSATED
reading is the correct one and the compensated one is not; the two arms swap places.  The
discriminator -- pitch relative to the GROUND rather than to gravity -- is not in the channel at
all.  So this mode measures a known limitation on purpose, the way rail.inject induces a rail
event on purpose.

⚠ THE ROBOT IS NOT COMMANDED TO MOVE except to reach `stand` once.  Physical setup is yours.

Examples:
  # gait case, floor level, front feet on a 40 mm block (~9.7 deg)
  python3 tof_ground_truth_check.py --mode shim --end front --shim-mm 40
  # slope case: level reference, then inclined
  python3 tof_ground_truth_check.py --mode slope --label level
  python3 tof_ground_truth_check.py --mode slope --label inclined
"""
import argparse, json, math, os, statistics as st, sys, time, zmq

ENDPOINT = "tcp://127.0.0.1:5590"
STORE    = os.path.expanduser("~/xaq-ai/pi_host/log/tof_truth_check.jsonl")
# ⚠ FITTED ON THE ROBOT, in the `stand` pose, 2026-09-27 -- NOT the sim's 0.233 receipt.
# Two shims under the front feet back the span out independently, from the pitch each one
# produced over the level baseline (-0.94°):  46 mm -> 12.47° -> 208.0 ± 0.9 mm
#                                             61 mm -> 16.12° -> 211.1 ± 0.7 mm
# The sim's foot_xz gives 233 mm, ~11% wider.  Both can be right: that receipt is the sim's
# nominal pose and this is the robot's saved `stand`, which is a different geometry.  A
# robot-side setup check wants the robot's number.
# ⚠ The 1.5% growth between the two shims is ~3σ and real -- most likely the rounded toe's
# contact point rolling as the leg angle changes.  It BOUNDS front-leg compliance at a couple
# of percent rather than leaving it unknown, which is what the two-point sweep was for: had
# the legs been absorbing the lift, the implied span would have grown WITH shim height.
FOOT_SPAN_M = 0.210        # fore-aft toe span in `stand`, fitted (see above)
_ctx = zmq.Context()

def rpc(verb, _allow_err=False, **kw):
    s = _ctx.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 25000); s.setsockopt(zmq.LINGER, 0)
    s.connect(ENDPOINT); s.send_string(json.dumps({"verb": verb, **kw}))
    try: r = json.loads(s.recv_string())
    except zmq.Again: r = {"ok": False, "error": "TIMEOUT"}
    finally: s.close()
    if not r.get("ok") and not _allow_err:
        raise RuntimeError(f"{verb} failed: {r.get('error')}  (sent {kw})")
    return r

def pose_us(n): return rpc("pose.get", name=n)["us"]
def vec(v): return list(v) if isinstance(v,(list,tuple)) else [v.get("x",0),v.get("y",0),v.get("z",0)]

def settle(t=40):
    t0 = time.time()
    while time.time() - t0 < t:
        s = rpc("status")
        if not s.get("pose_move_active") and not s.get("pose_queue"): return
        time.sleep(0.1)

def sample(secs):
    """Medians over `secs`, plus the spread -- §9.1 measured sd 1.51 mm and it is all sensor."""
    raw, unc, cmp_, dlt, pit, rol, upy, upz, bad = [], [], [], [], [], [], [], [], 0
    t0 = time.time()
    while time.time() - t0 < secs:
        s = rpc("status"); t = s.get("tof") or {}; imu = s.get("imu") or {}
        if imu.get("ok"):
            u = vec(imu["up_fused"])
            pit.append(math.degrees(math.atan2(-u[2], u[1])))
            rol.append(math.degrees(math.atan2(u[0], u[1])))
            upy.append(u[1]); upz.append(u[2])
        if t.get("ok") and t.get("valid"):
            if not t.get("comp_valid"): bad += 1; continue
            raw.append(t["raw_mm"]); unc.append(t["m"]); cmp_.append(t["m_comp"]); dlt.append(t["comp_delta"])
        else: bad += 1
        time.sleep(0.1)
    if len(raw) < 15 or not pit:
        sys.exit(f"only {len(raw)} valid samples ({bad} rejected) — not enough for a median")
    m = lambda a: st.median(a)
    return dict(n=len(raw), bad=bad, raw_mm=m(raw), raw_sd=st.stdev(raw) if len(raw)>1 else 0.0,
                uncomp_m=m(unc), comp_m=m(cmp_), comp_delta_m=m(dlt),
                pitch_deg=m(pit), roll_deg=m(rol), up_y=m(upy), up_z=m(upz))

ap = argparse.ArgumentParser()
ap.add_argument("--mode", required=True, choices=("shim", "slope"))
ap.add_argument("--end", choices=("front", "rear"), help="shim mode: which end is raised")
ap.add_argument("--shim-mm", type=float, help="shim mode: block height, mm")
ap.add_argument("--label", help="slope mode: 'level' for the reference, then any name")
ap.add_argument("--secs", type=float, default=6.0)
ap.add_argument("--no-stand", action="store_true", help="skip commanding stand (robot already posed)")
a = ap.parse_args()

s0 = rpc("status")
tof0 = s0.get("tof") or {}
H  = (tof0.get("offset_mm") or 64.8) / 1000.0
BZ = tof0.get("boom_z_m")
if BZ is None: sys.exit("this benchd does not publish boom_z_m — rebuild (§9.10)")
print(f"H (sensor above belly) = {H*1000:.1f} mm   boom_z = {BZ*1000:.1f} mm   "
      f"vbat {s0['vbat']:.2f}")

if not a.no_stand:
    rpc("pose.set", us=pose_us("stand")); settle(); time.sleep(1.5)
r = sample(a.secs)
print(f"\nn={r['n']} (rejected {r['bad']})   raw {r['raw_mm']:.1f} mm (sd {r['raw_sd']:.2f})   "
      f"pitch {r['pitch_deg']:+.2f}°  roll {r['roll_deg']:+.2f}°")
print(f"  uncompensated {r['uncomp_m']*1000:+8.2f} mm     compensated {r['comp_m']*1000:+8.2f} mm"
      f"     delta {r['comp_delta_m']*1000:+7.2f} mm")

rec = dict(t=time.time(), mode=a.mode, end=a.end, shim_mm=a.shim_mm, label=a.label, H_m=H,
           boom_z_m=BZ, **r)

if a.mode == "shim":
    if a.shim_mm:
        exp = math.degrees(math.atan2(a.shim_mm / 1000.0, FOOT_SPAN_M))
        # ⚠ NOSE-UP IS NEGATIVE PITCH HERE, and the first version of this line had it
        # backwards.  `pitch_deg` is atan2(-up.z, up.y); the axis map (BOM §4.1) makes the
        # sim body frame +Z FORWARD, and pitching the nose up puts the world-up vector's
        # forward component POSITIVE (up.z = +sin θ) -- so up.z > 0 and pitch_deg < 0.
        # MEASURED 2026-09-27: front feet on a shim, pitch went -0.94° -> -13.41°.
        # Shimming the FRONT therefore predicts a NEGATIVE pitch.  With the sign inverted
        # this printed a ~24° gap and read exactly like "the legs absorbed the shim".
        sign = -1 if a.end == "front" else +1
        print(f"\n  shim {a.shim_mm:.0f} mm at the {a.end} over a {FOOT_SPAN_M*1000:.0f} mm toe span "
              f"predicts {sign*exp:+.2f}° of pitch; measured {r['pitch_deg']:+.2f}°")
        print(f"  implied toe span = shim/tan(pitch) = "
              f"{a.shim_mm/math.tan(math.radians(abs(r['pitch_deg']))):.0f} mm "
              f"(nominal {FOOT_SPAN_M*1000:.0f}) — ⚠ this ignores the pre-shim baseline pitch, "
              f"so read it with the level run")
        print(f"  ⚠ a large gap here means the legs absorbed the shim rather than the chassis "
              f"pitching — the setup, not the correction.")
    # THE check: the full prediction, both terms.  At gait amplitude the second term is no
    # longer a rounding artefact (~1.8 mm at 15°), so this tests the whole formula.
    lead = -BZ * r["up_z"]
    second = (r["uncomp_m"] - 0.0) * (r["up_y"] - 1.0)   # (d-H)(up.y-1); d-H IS uncomp
    pred = lead + second
    resid = r["comp_delta_m"] - pred
    print(f"\n  predicted delta = leading {lead*1000:+.3f} mm  +  second-order {second*1000:+.3f} mm"
          f"  =  {pred*1000:+.3f} mm")
    print(f"  measured delta  = {r['comp_delta_m']*1000:+.3f} mm     residual "
          f"{resid*1000:+.3f} mm")
    ok_mag  = abs(resid) < 0.30
    ok_sign = (r["pitch_deg"] * r["comp_delta_m"] <= 0) or abs(r["pitch_deg"]) < 1.0
    print(f"  magnitude {'PASS' if ok_mag else 'FAIL'} (<0.30 mm)     "
          f"sign {'PASS' if ok_sign else 'FAIL — a sign error DOUBLES the artefact'}")
    if abs(r["pitch_deg"]) < 8.0:
        print(f"  ⚠ only {abs(r['pitch_deg']):.1f}° of pitch — §9.10 already validated below 4°. "
              f"Use a taller shim: 40 mm ~ 9.7°, 60 mm ~ 14.4°.")
    rec.update(pred_m=pred, resid_m=resid, pass_mag=ok_mag, pass_sign=ok_sign)

else:  # slope
    prev = []
    if os.path.exists(STORE):
        prev = [json.loads(l) for l in open(STORE) if l.strip()]
    ref = next((p for p in reversed(prev) if p.get("mode") == "slope" and p.get("label") == "level"), None)
    if a.label == "level" or ref is None:
        print(f"\n  stored as the LEVEL reference. Now incline the box and re-run with "
              f"--label inclined --no-stand")
        if a.label != "level":
            print(f"  ⚠ no level reference found, so this run was stored as the baseline "
                  f"regardless of its label.")
            rec["label"] = "level"
    else:
        d_unc = r["uncomp_m"] - ref["uncomp_m"]
        d_cmp = r["comp_m"]   - ref["comp_m"]
        # ⚠ PREDICT THE DRIFT AS A DIFFERENCE FROM THE REFERENCE, AND USE BOTH TERMS.  The
        # first version used `boom_z*sin(pitch_now)`, which drops two things: the reference
        # run's OWN pitch (a level box is never level -- ours sat at -1.04°) and the
        # second-order term.  MEASURED 2026-09-27: it predicted +14.63 mm against a measured
        # +12.16 mm and "passed" only because the gate was 4 mm wide.  The full form below
        # predicted +12.196 against +12.161 -- a 0.034 mm residual.  ⚠ The 2.5 mm was the
        # APPROXIMATION, not the physics, and a gate loose enough to absorb your own sloppy
        # prediction cannot fail for the right reason either.  Hence 1.5 mm.
        lead       = -BZ * (r["up_z"] - ref["up_z"])
        second     = r["uncomp_m"] * (r["up_y"] - 1.0) - ref["uncomp_m"] * (ref["up_y"] - 1.0)
        drift_pred = lead + second
        print(f"\n  vs the level reference (pitch {ref['pitch_deg']:+.2f}° -> {r['pitch_deg']:+.2f}°):")
        print(f"    uncompensated moved {d_unc*1000:+7.2f} mm   "
              f"(~0 WITHIN ONE STAND: the ray is perpendicular to the surface either way;\n"
              f"                                 across pose recalls expect up to ~3.4 mm, §9.3)")
        # ⚠ TEST THE CORRECTION'S CHANGE, NOT THE CLEARANCE'S.  By definition
        # m_comp = m + comp_delta, so d_cmp = d_unc + drift_pred: any shift in the underlying
        # stand carries straight through to the compensated reading and is NOT a failure of
        # the correction.  The first version compared d_cmp against drift_pred alone.
        # ⚠ MEASURED 2026-09-27, and this is why it survived a passing run: the level and
        # nose-up runs shared ONE continuous stand, so d_unc was exactly 0.00 and the missing
        # term was invisible.  The nose-down run was a fresh recall (the keepalive had died
        # and the robot re-stood), d_unc came in at -3.00 mm, and the "residual" was -3.06 --
        # the same number wearing a different name.  §9.3 already measured `stand` recalling
        # to 52.1 / 48.7 / 50.8 mm, so 3 mm is pose repeatability, not a result.
        # Comparing d_delta against drift_pred is immune to all of it.
        d_delta   = r["comp_delta_m"] - ref["comp_delta_m"]
        resid     = d_delta - drift_pred
        same_pose = abs(d_unc) < 1.0/1000
        print(f"    compensated   moved {d_cmp*1000:+7.2f} mm   = uncomp drift "
              f"{d_unc*1000:+.2f} + correction drift {d_delta*1000:+.2f}")
        print(f"    THE TEST — correction drift {d_delta*1000:+7.2f} mm   predicted "
              f"{drift_pred*1000:+7.2f} mm  (lead {lead*1000:+.2f}, 2nd-order {second*1000:+.2f})")
        print(f"    residual            {resid*1000:+7.2f} mm")
        if not same_pose:
            print(f"  ⚠ the stand moved {d_unc*1000:+.2f} mm between the reference and this run, so "
                  f"this is a DIFFERENT POSE RECALL,\n     not one stand tilted. §9.3 measured that "
                  f"spread at 3.4 mm. It does not affect the test above.")
        verdict = "CONFIRMED" if abs(resid) < 1.5/1000 else "UNEXPECTED"
        print(f"\n  §9.10.3 prediction {verdict}.")
        if verdict == "CONFIRMED":
            # ⚠ REPORT THE DIRECTION MEASURED, NEVER A REMEMBERED ONE.  The first version
            # hardcoded "nose-DOWN over-reports", inherited from §9.10.3's original text --
            # which was itself inverted.  It therefore printed a claim CONTRADICTING the run
            # it had just made (nose-up, over-reporting).  Three copies of one inverted sign:
            # the doc, its commit message, and here.  Derive it instead and it cannot go stale.
            heading = "NOSE-UP / climbing" if r["pitch_deg"] < 0 else "NOSE-DOWN / descending"
            worse   = "OVER-reports clearance — PHANTOM CLEARANCE, the dangerous sign" \
                      if d_cmp > 0 else "under-reports clearance — conservative"
            print(f"  ⚠ On a slope the UNCOMPENSATED arm is the correct one and the compensated "
                  f"arm is not.")
            print(f"  ⚠ MEASURED HERE: {heading} at {abs(r['pitch_deg']):.1f}° — the compensated "
                  f"arm {worse},\n     by {abs(d_cmp)*1000:.1f} mm. Do not promote m_comp for terrain.")
        rec.update(d_uncomp_m=d_unc, d_comp_m=d_cmp, d_delta_m=d_delta, resid_m=resid,
                   drift_pred_m=drift_pred, drift_lead_m=lead, drift_second_m=second,
                   same_pose=same_pose, verdict=verdict)

os.makedirs(os.path.dirname(STORE), exist_ok=True)
with open(STORE, "a") as f: f.write(json.dumps(rec) + "\n")
print(f"\nappended to {STORE}")
