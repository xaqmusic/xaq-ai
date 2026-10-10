#!/usr/bin/env python3
"""Does a TOUCHDOWN land on the right side of the threshold, every time?  (Bench step E7.)

§5.7.12 read the deployed config and found that nothing in the brain consumes foot-load
MAGNITUDE.  Every consumer is a threshold: GainEvolver's `unloaded` term (weight 1.0) takes
max(foot_load) over the 12 ticks after a touchdown and tests it ONCE against load_thresh
0.05 -- 29.5 g -- then scores a RATIO of touchdown counts.  StrideOdometry's stance gate is
a binary test at ~0.2.  So the question was never "how accurate is the curve".  It is
"does a touchdown get classified correctly, repeatably".

⚠ AND ALL THE REPEATABILITY DATA SO FAR IS AT THE WRONG LOAD.  §5.7.9-11 swept 150-500 g,
where the curve is flat (~2 counts/g).  Both decisions live at 29.5 g and 118 g, where it
is steep (~19 counts/g).  Nothing has ever been measured there.

THE METHOD (operator, 2026-10-01).  From `toes_up`, pivot ONE leg down at hip2 until the
toe taps the ground, then return.  Two things make this better than the knee sweep it
replaces:

  * Pivoting at hip2 keeps the lower leg's angle fixed relative to the body, so the toe
    lands on the same part of the hemisphere each time.  The knee sweep ROTATED the lower
    leg, dragging the toe -- which is the confound §5.7.11 localised.  Removed by
    construction rather than by care.
  * A real touchdown IS a toe tapping down with the leg swinging.  This exercises the
    event the decision is actually made on, not a body tipping onto a foot.

WHAT IT REPORTS.  The peak over each tap, taken from the daemon's own 50 Hz `adc_fast`
record -- 12 ticks is 12 samples at that rate, which is the criterion's own horizon.  The
spread of those peaks across repeats, at a fixed command, IS the answer: no calibration, no
geometry, no force model.

⚠ IT SEARCHES DOWNWARD FROM A LOADED POSE, NOT UPWARD FROM AN UNLOADED ONE, and that is a
correction.  The first version started at `toes_up` and drove hip2 until the toe registered.
It never did: `toes_up` parks FL's hip2 at 500 us -- ITS END STOP -- so the direction that
would lower the toe had zero room, and 700 us the other way (74 degrees) never reached the
ground.  Starting instead from `<foot>_down`, where the toe is known loaded, and searching
for the RELEASE point brackets the light-load region by construction: load passes through
0.05 and 0.2 on the way out.  The unload direction is detected rather than assumed.

⚠ THE ROBOT MOVES.  Travel is bounded by --max-travel AND by the channel's own envelope,
and the search stops the moment the foot releases.  ⚠ The keepalive is
not optional -- see foot_cal_sweep.py's header for what benchd's deadman costs.
"""
import argparse, json, os, statistics as st, subprocess, sys, threading, time
import zmq

ENDPOINT = "tcp://127.0.0.1:5590"
TOUCHDOWN_FLOOR = 20     # counts; the edge the 12-tick horizon is measured from
FEET = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}
_ctx, _lock, _stop = zmq.Context(), threading.Lock(), threading.Event()


def rpc(verb, _allow_err=False, **kw):
    with _lock:
        s = _ctx.socket(zmq.REQ)
        s.setsockopt(zmq.RCVTIMEO, 20000); s.setsockopt(zmq.LINGER, 0)
        s.connect(ENDPOINT); s.send_string(json.dumps({"verb": verb, **kw}))
        try: r = json.loads(s.recv_string())
        except zmq.Again: r = {"ok": False, "error": "TIMEOUT"}
        finally: s.close()
    if not r.get("ok") and not _allow_err:
        raise RuntimeError(f"{verb} failed: {r.get('error')}  (sent {kw})")
    return r


def keepalive():
    while not _stop.is_set():
        try: rpc("ping")
        except Exception: pass
        _stop.wait(0.2)


def settle(eta_ms, timeout=20.0):
    t0 = time.time()
    while time.time() - t0 < min(3.0, 1.0 + eta_ms / 1000.0):
        if rpc("status").get("pose_move_active"): break
        time.sleep(0.04)
    while time.time() - t0 < timeout:
        if not rpc("status").get("pose_move_active"): return True
        time.sleep(0.04)
    return False


def hip2_channel(mapping, physical):
    for sv in mapping.get("servos", []):
        if sv.get("physical") == physical and sv.get("joint") == "hip2":
            return sv.get("ch"), sv.get("sign", 1)
    raise RuntimeError(f"no hip2 channel mapped for {physical} -- run cal.map first")


def knee_state(stt, ch):
    """⚠ Only hip2 is ever commanded here, so a knee that MOVES is being back-driven by the
    load, not driven by us.  Printing target vs current is what tells those apart."""
    if ch is None: return "—"
    sv = next((x for x in stt.get("servos", []) if x.get("ch") == ch), None)
    if not sv: return "—"
    t, c = sv.get("target_us"), sv.get("current_us")
    return f"{c}us" + ("" if t == c else f" (cmd {t}, Δ{c - t:+d})")


def read_adc(ch):
    return rpc("status")["adc"][ch]


def tilt_of(stt):
    import math
    up = (stt.get("imu") or {}).get("up_fused")
    if not up or len(up) != 3: return float("nan")
    n = math.sqrt(sum(c * c for c in up)) or 1.0
    return math.degrees(math.acos(max(-1.0, min(1.0, up[1] / n))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--foot", default="FL")
    ap.add_argument("--base", default=None,
                    help="loaded pose to search down from; default <foot>_down")
    ap.add_argument("--dir", type=int, default=0, choices=(0, 1, -1),
                    help="hip2 step direction that UNLOADS the toe. 0 = detect it")
    ap.add_argument("--probe-step", type=int, default=10, help="us per release-search step")
    ap.add_argument("--max-travel", type=int, default=400,
                    help="us; the search gives up here. ⚠ Also bounded by the channel's own "
                         "envelope, which is the real limit: in `toes_up` a hip2 can already "
                         "sit AT its end stop, leaving travel in one direction only")
    ap.add_argument("--contact-counts", type=int, default=60,
                    help="ADC counts that count as the toe TOUCHING on the way back down")
    ap.add_argument("--release-counts", type=int, default=30,
                    help="ADC counts below which the toe counts as RELEASED")
    ap.add_argument("--over", default="5,10,20,40",
                    help="us back from the release point, toward load, to tap at")
    ap.add_argument("--taps", type=int, default=8, help="repeats per level")
    ap.add_argument("--hold", type=float, default=0.5, help="s held down per tap")
    a = ap.parse_args()

    foot = a.foot.upper()
    if foot not in FEET: sys.exit(f"unknown foot {foot!r}")
    adc_ch = FEET[foot]
    overs = [int(x) for x in a.over.split(",")]

    stt = rpc("status"); mapping = stt.get("map", {})
    poses = rpc("pose.list")["poses"]
    base_name = a.base or {"FL": "front_left_down", "FR": "front_right_down",
                           "RL": "rear_left_down", "RR": "rear_right_down"}[foot]
    if base_name not in poses:
        sys.exit(f"pose {base_name!r} not found; have: {', '.join(sorted(poses))}")
    ch, sign = hip2_channel(mapping, foot)
    knee_ch = next((sv.get("ch") for sv in mapping.get("servos", [])
                    if sv.get("physical") == foot and sv.get("joint") == "knee"), None)
    base = rpc("pose.get", name=base_name)["us"]
    sv = next((x for x in stt.get("servos", []) if x.get("ch") == ch), {})
    lo, hi = sv.get("min_us", 500), sv.get("max_us", 2500)
    print(f"{foot}: hip2 ch{ch} (map sign {sign:+d}) · ADC A{adc_ch} · loaded pose {base_name}")
    print(f"tick_hz {stt.get('tick_hz')} · vbat {stt.get('vbat'):.2f}")
    print(f"ch{ch} parks at {base[ch]} us, envelope {lo}-{hi}")

    threading.Thread(target=keepalive, daemon=True).start()
    rpc("adc.rate", ms=20)
    taps = []
    try:
        def goto(off):
            us = list(base)
            want = base[ch] + off
            us[ch] = max(lo, min(hi, want))
            if us[ch] != want:
                print(f"    ⚠ clamped {want} -> {us[ch]} by the envelope")
            r = rpc("pose.set", us=us)
            settle(r.get("eta_ms", 0))
            return us[ch]

        goto(0); time.sleep(0.8)
        loaded = read_adc(adc_ch)
        print(f"\nloaded, on {base_name}: A{adc_ch} = {loaded} counts")
        if loaded < a.release_counts * 3:
            sys.exit(f"⚠ the foot is not meaningfully loaded in {base_name} "
                     f"({loaded} counts). Re-pose it before measuring repeatability.")

        # ---- which way unloads?  Detect it; do not assume ----------------------
        # ⚠ DETECT THE UNLOADING DIRECTION FROM TILT, NOT FROM COUNTS.  The first version
        # compared ADC counts over a +-20 us probe and picked the direction whose reading was
        # 46 counts lower -- on a channel §5.7.11 measured as irreproducible by 265-805
        # counts.  It chose the direction that LOADS the foot and drove 42 degrees the wrong
        # way.  Body tilt has sd 0.03 degrees and moves by whole degrees over the same probe:
        # unloading lowers the propped corner, so the lower tilt is the way out.
        probe = a.probe_step * 8
        dirs = [a.dir] if a.dir else [+1, -1]
        if not a.dir:
            print(f"probing +-{probe} us; the direction that LOWERS TILT is the one that unloads:")
            trial = {}
            for dd in (+1, -1):
                if not (lo <= base[ch] + dd * probe <= hi):
                    print(f"   {dd:+d}: no envelope room"); continue
                goto(dd * probe); time.sleep(0.5)
                stp = rpc("status")
                trial[dd] = tilt_of(stp)
                print(f"   {dd * probe:>+5} us -> tilt {trial[dd]:6.2f}°   "
                      f"(A{adc_ch} {stp['adc'][adc_ch]:>5} counts)")
                goto(0); time.sleep(0.4)
            trial = {k: v for k, v in trial.items() if v == v}
            if not trial:
                sys.exit("⚠ no usable tilt reading either way — is the IMU up?")
            dirs = [min(trial, key=trial.get)]
            print(f"   -> unloading direction is {dirs[0]:+d} "
                  f"(tilt {trial[dirs[0]]:.2f}° against {max(trial.values()):.2f}°)")
        d = dirs[0]
        room = (hi - base[ch]) if d > 0 else (base[ch] - lo)
        cap = min(a.max_travel, room)

        # ---- A: lift until the belly is flat ----------------------------------
        # Operator's protocol: go to <foot>_down, lift hip2 until the belly is flat, then
        # come back down until the toe touches.  Approaching contact from UNLOADED is the
        # direction a real touchdown happens in, and it gives an unambiguous first-contact
        # edge instead of hunting for a release in a signal that is already loaded.
        print(f"\nA: hip2 {d:+d} until the belly is flat  (step {a.probe_step} us, cap {cap} us)")
        flat = None
        off, prev_tilt = 0, None
        while off + a.probe_step <= cap:
            off += a.probe_step
            goto(d * off); time.sleep(0.3)
            stp = rpc("status")
            v, tl = stp["adc"][adc_ch], tilt_of(stp)
            kn = knee_state(stp, knee_ch)
            print(f"   {d * off:>+5} us -> {v:>5} counts   tilt {tl:6.2f}°   knee {kn}")
            settled = prev_tilt is not None and abs(prev_tilt - tl) < 0.05
            if v < a.release_counts and settled:
                flat = off; break
            prev_tilt = tl
        if flat is None:
            goto(0)
            sys.exit(f"\n⚠ the belly never went flat within {cap} us "
                     f"(need counts < {a.release_counts} and tilt settling). "
                     f"Raise --max-travel.")
        print(f"   belly flat at {d * flat:+d} us")
        stp = rpc("status"); flat_tilt = tilt_of(stp)
        if flat_tilt == flat_tilt and flat_tilt < 3.0:
            print(f"   ⚠ tilt is {flat_tilt:.2f}° at 'flat' — the chassis is down on its belly,"
                  f"\n     so this corner was barely propped and a tap has nothing to push"
                  f"\n     against. Re-pose {base_name} so the corner lifts.")

        # ---- B: back down until the toe touches --------------------------------
        fine = max(2, a.probe_step // 4)
        print(f"\nB: back down in {fine} us steps until the toe contacts "
              f"({a.contact_counts}+ counts)")
        release = None
        o = flat
        while o - fine >= 0:
            o -= fine
            goto(d * o); time.sleep(0.3)
            stp = rpc("status")
            v, tl = stp["adc"][adc_ch], tilt_of(stp)
            print(f"   {d * o:>+5} us -> {v:>5} counts   tilt {tl:6.2f}°   "
                  f"knee {knee_state(stp, knee_ch)}")
            if v >= a.contact_counts:
                release = o; break
        if release is None:
            goto(0)
            sys.exit("\n⚠ came all the way back to the loaded pose without a clean contact "
                     "edge — the toe was never fully released.")
        print(f"\nfirst contact at {d * release:+d} us; "
              f"resting at {d * flat:+d} us between taps")

        rest_off = d * flat                           # belly flat: the toe is clear

        # ---- the taps ----------------------------------------------------------
        for over in overs:
            tap_off = d * (release - over)            # back toward load by `over`
            print(f"\n--- {a.taps} taps at release-{over} us  (ch{ch} offset {tap_off:+d})")
            for i in range(a.taps):
                goto(rest_off); time.sleep(0.35)
                rpc("mark", text=f"tap {foot} over{over:+d} n{i}")
                goto(tap_off)
                time.sleep(a.hold)
                taps.append({"over": over, "n": i, "tilt": tilt_of(rpc("status"))})
                rpc("mark", text=f"tap {foot} over{over:+d} n{i} RELEASE")
            print("    done")
    finally:
        rpc("adc.rate", ms=0, _allow_err=True)
        try:
            r = rpc("pose.set", us=base); settle(r.get("eta_ms", 0))
            print(f"\nparked on {base_name}.")
        except Exception as e:
            print(f"⚠ could not park: {e}")
        _stop.set()

    # ---- peaks, from the daemon's own 50 Hz record -------------------------
    logs = sorted(__import__("glob").glob(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "log", "benchd_*.jsonl")))
    peaks, seg_max, misses = {}, {}, {}
    if logs:
        cur = None; series = {}
        for line in open(logs[-1]):
            try: r = json.loads(line)
            except Exception: continue
            k = r.get("kind")
            if k == "mark":
                t = str(r["data"].get("text", ""))
                cur = t if t.startswith(f"tap {foot} ") and not t.endswith("RELEASE") else None
                if cur: series[cur] = []
            elif k == "adc_fast" and cur is not None:
                series[cur].append(r["data"]["a"][adc_ch])
        # ⚠ THE 12-TICK HORIZON STARTS AT TOUCHDOWN, NOT AT THE COMMAND.  The mark is written
        # before pose.set, and pose.set staggers channels 100 ms apart, so the first 12 samples
        # after a mark can land entirely before the leg has moved.  Taking them cost a whole
        # FR run, which reported 1-3 counts on every tap and read as a foot that never touched
        # -- the samples were there, 12 ticks too early.  Find the first sample above the floor,
        # then take the max of the next 12, which is what GainEvolver does.
        for lbl, vals in series.items():
            if len(vals) < 4: continue
            over = int(lbl.split("over")[1].split()[0])
            seg_max.setdefault(over, []).append(max(vals))
            td = next((i for i, v in enumerate(vals) if v >= TOUCHDOWN_FLOOR), None)
            if td is None:
                misses[over] = misses.get(over, 0) + 1
                continue
            peaks.setdefault(over, []).append(max(vals[td:td + 12]))

    print("\n=== peak counts over the 12 ticks after touchdown — the criterion's own horizon\n")
    print(f"    {'over us':>9}{'n':>4}{'miss':>6}{'mean':>9}{'sd':>8}{'min':>8}{'max':>8}"
          f"{'spread':>9}{'segmax':>9}")
    for over in overs:
        v, sm = peaks.get(over, []), seg_max.get(over, [])
        if not v:
            print(f"    {over:>+9}{0:>4}{misses.get(over, 0):>6}"
                  f"{'— never reached the touchdown floor':>45}"
                  f"{(max(sm) if sm else 0):>9}")
            continue
        print(f"    {over:>+9}{len(v):>4}{misses.get(over, 0):>6}{st.fmean(v):>9.0f}"
              f"{(st.pstdev(v) if len(v) > 1 else 0):>8.1f}{min(v):>8}{max(v):>8}"
              f"{max(v) - min(v):>9}{(max(sm) if sm else 0):>9}")
    tl = [t["tilt"] for t in taps if t["tilt"] == t["tilt"]]
    if len(tl) > 1:
        print(f"\n    body tilt across all taps: sd {st.pstdev(tl):.2f}°  "
              f"(a rising tilt means the corner lifted and the force self-limited)")
    print("\n    ⚠ This is the decision, run many times — not a force curve.  Compare `spread`"
          "\n    against the gap between a touchdown and no touchdown.  A spread small against"
          "\n    that gap means the threshold is safe however rough the calibration is.")


if __name__ == "__main__":
    main()
