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

⚠ THE ROBOT MOVES, and it drives a leg toward the floor.  The contact search is bounded
(--max-travel, default 150 us) and stops the moment the foot registers.  ⚠ The keepalive is
not optional -- see foot_cal_sweep.py's header for what benchd's deadman costs.
"""
import argparse, json, os, statistics as st, subprocess, sys, threading, time
import zmq

ENDPOINT = "tcp://127.0.0.1:5590"
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
    ap.add_argument("--base", default="toes_up", help="pose the leg returns to between taps")
    ap.add_argument("--dir", type=int, default=1, choices=(1, -1),
                    help="sign of the hip2 step that moves the toe DOWN, in the leg's own "
                         "convention. If the search finds nothing, try the other one")
    ap.add_argument("--probe-step", type=int, default=5, help="us per contact-search step")
    ap.add_argument("--max-travel", type=int, default=150, help="us; the search gives up here")
    ap.add_argument("--contact-counts", type=int, default=40,
                    help="ADC counts that count as 'the toe is touching' during the search")
    ap.add_argument("--over", default="0,5,10,20,40",
                    help="us past first contact to tap at")
    ap.add_argument("--taps", type=int, default=8, help="repeats per level")
    ap.add_argument("--hold", type=float, default=0.5, help="s held down per tap")
    a = ap.parse_args()

    foot = a.foot.upper()
    if foot not in FEET: sys.exit(f"unknown foot {foot!r}")
    adc_ch = FEET[foot]
    overs = [int(x) for x in a.over.split(",")]

    stt = rpc("status"); mapping = stt.get("map", {})
    poses = rpc("pose.list")["poses"]
    if a.base not in poses:
        sys.exit(f"pose {a.base!r} not found; have: {', '.join(sorted(poses))}")
    ch, sign = hip2_channel(mapping, foot)
    base = rpc("pose.get", name=a.base)["us"]
    print(f"{foot}: hip2 ch{ch} (map sign {sign:+d}) · ADC A{adc_ch} · base pose {a.base}")
    print(f"tick_hz {stt.get('tick_hz')} · vbat {stt.get('vbat'):.2f}")

    threading.Thread(target=keepalive, daemon=True).start()
    rpc("adc.rate", ms=20)
    taps = []
    try:
        def goto(off, eta_wait=True):
            us = list(base); us[ch] = base[ch] + sign * a.dir * off
            r = rpc("pose.set", us=us)
            if eta_wait: settle(r.get("eta_ms", 0))
            return r

        goto(0); time.sleep(0.6)
        rest = read_adc(adc_ch)
        print(f"\nat rest on {a.base}: A{adc_ch} = {rest} counts")

        # ---- bounded contact search ------------------------------------------
        print(f"contact search: {a.probe_step} us steps, stop at {a.contact_counts} counts, "
              f"give up at {a.max_travel} us")
        contact = None
        off = 0
        while off < a.max_travel:
            off += a.probe_step
            goto(off); time.sleep(0.25)
            v = read_adc(adc_ch)
            print(f"   {off:>4} us -> {v:>5} counts")
            if v >= a.contact_counts:
                contact = off; break
        if contact is None:
            goto(0)
            sys.exit(f"\n⚠ no contact within {a.max_travel} us. The toe never reached the "
                     f"ground, or --dir {a.dir} drives it the wrong way. Try --dir {-a.dir}.")
        print(f"\nfirst contact at {contact:+d} us past {a.base}")
        goto(0); time.sleep(0.5)

        # ---- the taps ---------------------------------------------------------
        for over in overs:
            print(f"\n--- {a.taps} taps at contact{over:+d} us")
            for i in range(a.taps):
                rpc("mark", text=f"tap {foot} over{over:+d} n{i}")
                goto(contact + over)
                time.sleep(a.hold)
                stt2 = rpc("status")
                taps.append({"over": over, "n": i, "tilt": tilt_of(stt2)})
                rpc("mark", text=f"tap {foot} over{over:+d} n{i} RELEASE")
                goto(0)
                time.sleep(0.35)
            print(f"    done")
    finally:
        rpc("adc.rate", ms=0, _allow_err=True)
        try:
            r = rpc("pose.set", us=base); settle(r.get("eta_ms", 0))
            print(f"\nparked on {a.base}.")
        except Exception as e:
            print(f"⚠ could not park: {e}")
        _stop.set()

    # ---- peaks, from the daemon's own 50 Hz record -------------------------
    logs = sorted(__import__("glob").glob(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "log", "benchd_*.jsonl")))
    peaks = {}
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
        for lbl, vals in series.items():
            if len(vals) < 4: continue
            over = int(lbl.split("over")[1].split()[0])
            peaks.setdefault(over, []).append(max(vals[:12]) if len(vals) >= 12 else max(vals))

    print("\n=== peak counts over the 12 ticks after touchdown — the criterion's own horizon\n")
    print(f"    {'over us':>9}{'n':>4}{'mean':>9}{'sd':>8}{'min':>8}{'max':>8}{'spread':>9}")
    for over in overs:
        v = peaks.get(over, [])
        if not v:
            print(f"    {over:>+9}{0:>4}{'— no samples captured':>42}")
            continue
        print(f"    {over:>+9}{len(v):>4}{st.fmean(v):>9.0f}"
              f"{(st.pstdev(v) if len(v) > 1 else 0):>8.1f}{min(v):>8}{max(v):>8}"
              f"{max(v) - min(v):>9}")
    tl = [t["tilt"] for t in taps if t["tilt"] == t["tilt"]]
    if len(tl) > 1:
        print(f"\n    body tilt across all taps: sd {st.pstdev(tl):.2f}°  "
              f"(a rising tilt means the corner lifted and the force self-limited)")
    print("\n    ⚠ This is the decision, run many times — not a force curve.  Compare `spread`"
          "\n    against the gap between a touchdown and no touchdown.  A spread small against"
          "\n    that gap means the threshold is safe however rough the calibration is.")


if __name__ == "__main__":
    main()
