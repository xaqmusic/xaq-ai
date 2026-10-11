#!/usr/bin/env python3
"""scale_probe.py — one servo's torque AND current at the same time: hip2 presses a leg onto a kitchen scale.

ON THE PI, robot on its stand, operator at the scale.  Why (ledger 2026-10-05, "SIM POWER MODEL S1"):
the sim's current model needs amps per newton-metre, and two estimates say the robot's servos act
about twice as strong as the MG90S datasheet's 0.18 N m.  A blocked push measures stall CURRENT;
only a known force on a known arm measures TORQUE.  Here the scale is the stop.  The target never
leaves the servo map, because the scale stops the leg long before the map would.

Sequence (one leg):
  [--pre-pose, e.g. 'toes_up', every servo armed with the legs clear] -> --pose, e.g. 'torque_check'
  (the leg on test straight, its knee a couple of mm above the scale) -> operator TARES the scale
  -> pressing direction: the way hip2 moved from the pre-pose (legs up) into the pose (onto the
     scale), unless --press-sign says otherwise
  -> steps: hip2 target = contact + k * step, held while the operator reads the scale, then back to
     contact to rest.  Current is benchd's 50 Hz INA219 reading (whole HAT, battery side), and the
     baseline is the second before each push
  -> rescue pose.
Torque = scale reading x g x arm.  arm = horizontal distance from the hip2 axis to the knee contact
(53.6 mm, the CAD femur, when the femur is horizontal; measure it and pass --arm-mm).

Safety: each push is held for at most --max-hold s (then it backs off, and the reading can still be
typed), it returns at once above --abort-a, and it rests at contact between pushes.

  python3 pi_host/tools/scale_probe.py --leg FL --pre-pose toes_up --pose torque_check
  python3 pi_host/tools/scale_probe.py --leg FR --dry-run      # checks benchd and the feed, nothing moves
"""
import argparse
import json
import select
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stall_probe import MIRROR, REPO, Events, Recorder, Rpc  # noqa: E402

G = 9.80665


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--leg", required=True, choices=sorted(MIRROR), help="PHYSICAL leg on the scale")
    ap.add_argument("--step", type=int, default=40, help="us per step past contact")
    ap.add_argument("--max-steps", type=int, default=12)
    ap.add_argument("--settle", type=float, default=0.8, help="s after the slew before reading")
    ap.add_argument("--max-hold", type=float, default=6.0, help="s pushing before it backs off")
    ap.add_argument("--rest", type=float, default=2.0, help="s resting at contact between pushes")
    ap.add_argument("--abort-a", type=float, default=2.5, help="return at once above this current")
    ap.add_argument("--arm-mm", type=float, default=53.6)
    ap.add_argument("--pose", default="torque_check", help="the leg on test just above the scale")
    ap.add_argument("--pre-pose", default="toes_up", help="legs clear of everything first ('' = none)")
    ap.add_argument("--press-sign", type=int, choices=(-1, 1), default=None,
                    help="+1/-1 us direction that presses; default: pre-pose -> pose direction")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    bench = Rpc()
    st = bench.call("status")
    if not st.get("ok", True) or st.get("mode") != "bench" or st.get("stopped"):
        sys.exit(f"benchd must be reachable, in bench mode and not STOPPED: {st.get('mode')} stopped={st.get('stopped')}")
    smap = json.load(open(REPO / "pi_host/calib/servo_map.json"))
    sv = next(s for s in smap["servos"] if s.get("sim_leg") == MIRROR[a.leg] and s["joint"] == "hip2")
    ch, lo, hi = sv["ch"], sv["min_us"], sv["max_us"]
    pose = (bench.call("pose.get", name=a.pose) or {}).get("us")
    if not pose:
        sys.exit(f"no saved pose '{a.pose}'")
    pre = (bench.call("pose.get", name=a.pre_pose) or {}).get("us") if a.pre_pose else None
    if a.pre_pose and not pre:
        sys.exit(f"no saved pose '{a.pre_pose}'")
    if a.press_sign is not None:
        sign = a.press_sign
    elif pre and pre[ch] != pose[ch]:
        sign = 1 if pose[ch] > pre[ch] else -1
    else:
        sys.exit("cannot infer the pressing direction: pass --press-sign")
    out = REPO / "pi_host/log" / f"scaleprobe_{time.strftime('%Y%m%d_%H%M%S')}_{a.leg}"
    out.mkdir(parents=True, exist_ok=True)
    ev = Events(out / "events.jsonl")
    rec = Recorder(out / "feed.jsonl")
    stop_ping = threading.Event()

    def pinger():                           # a controlling client: the bench deadman stays quiet
        r = Rpc()
        while not stop_ping.is_set():
            r.call("ping")
            stop_ping.wait(0.2)
    threading.Thread(target=pinger, daemon=True).start()

    samples = []                            # (mono s, i_a) from the feed, for windowed means
    def sampler():
        while not stop_ping.is_set():
            samples.append((time.monotonic(), rec.i_a))
            time.sleep(0.02)
    threading.Thread(target=sampler, daemon=True).start()

    def window(t0, t1):
        v = [i for t, i in samples if t0 <= t <= t1]
        return (sum(v) / len(v), max(v)) if v else (float("nan"), float("nan"))

    def goto(us):
        r = bench.call("servo.set", ch=ch, us=us)
        if not r.get("ok"):
            raise RuntimeError(f"servo.set ch{ch} {us}: {r.get('error')}")

    def wait_slew(d_us):
        time.sleep(abs(d_us) / 40 * 0.02 + 0.1)

    def ask(prompt, timeout=None, on_timeout=None):
        """input() with an optional deadline; on_timeout runs once when it passes, then it keeps waiting."""
        print(prompt, end="", flush=True)
        t_end = None if timeout is None else time.monotonic() + timeout
        fired = False
        while True:
            wait = 0.05 if t_end is None else max(0.0, min(0.05, t_end - time.monotonic()))
            r, _, _ = select.select([sys.stdin], [], [], wait)
            if r:
                return sys.stdin.readline().strip()
            if t_end is not None and not fired and time.monotonic() >= t_end:
                fired = True
                if on_timeout:
                    on_timeout()
            if t_end is not None and rec.i_a > a.abort_a and not fired:
                fired = True
                ev("abort", i_a=rec.i_a)
                print(f"\n    current {rec.i_a:.2f} A > {a.abort_a} — backing off. ", end="", flush=True)
                if on_timeout:
                    on_timeout()

    rows, contact = [], pose[ch]
    print(f"scale_probe: leg {a.leg}, hip2 = ch {ch} (map {lo}-{hi} us, {a.pose} {pose[ch]} us, pressing "
          f"{'+' if sign > 0 else '-'}us), arm {a.arm_mm} mm -> {out}")
    try:
        if a.dry_run:
            time.sleep(1.0)
            print(f"  DRY RUN — nothing moved. feed frames {rec.n}, current {rec.i_a:.3f} A")
            return
        def pose_to(name, us):
            r = bench.call("pose.set", us=us)
            if not r.get("ok"):
                raise RuntimeError(f"pose.set {name}: {r.get('error')}")
            ev("pose_start", pose=name)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 3 and not bench.call("status").get("pose_move_active"):
                time.sleep(0.05)
            while bench.call("status").get("pose_move_active"):
                time.sleep(0.1)
            ev("pose_landed", pose=name)
            print(f"  at '{name}'")
        if pre:
            ask(f"  >> Press Enter to move to '{a.pre_pose}' ")
            pose_to(a.pre_pose, pre)
            ask(f"  >> Press Enter to move to '{a.pose}' ")
        pose_to(a.pose, pose)
        ask(f"  >> Check the {a.leg} knee is just above the scale, TARE it, then press Enter to start "
            f"pushing (hip2 {'+' if sign > 0 else '-'}{a.step} us per step) ")
        ev("direction", sign=sign)
        print(f"  Type the scale reading in GRAMS at each step (q to stop). Each push holds at most "
              f"{a.max_hold:.0f} s.")

        for k in range(1, a.max_steps + 1):
            tgt = contact + sign * k * a.step
            if not lo <= tgt <= hi:
                print(f"  target {tgt} us is outside the map ({lo}-{hi}) — stopping")
                break
            tb = time.monotonic()
            base, _ = window(tb - 1.0, tb)
            goto(tgt)
            ev("push", step=k, to_us=tgt)
            wait_slew(k * a.step)
            time.sleep(a.settle)
            t_read0 = time.monotonic()
            backed = {"t": None}

            def back_off():
                goto(contact)
                backed["t"] = time.monotonic()
                ev("back_off", step=k)
                print(f"\n    held {a.max_hold:.0f} s — backed off; type the reading you saw: ", end="", flush=True)
            g_txt = ask(f"  step {k:2d}  ({sign * k * a.step:+4d} us)  grams? ", a.max_hold, back_off)
            t_read1 = backed["t"] or time.monotonic()
            if backed["t"] is None:
                goto(contact)
            i_mean, i_peak = window(t_read0, t_read1)
            if g_txt.lower().startswith("q"):
                ev("quit", step=k)
                break
            try:
                grams = float(g_txt)
            except ValueError:
                grams = float("nan")
            tau = grams / 1000 * G * a.arm_mm / 1000
            row = {"step": k, "delta_us": sign * k * a.step, "grams": grams, "torque_nm": round(tau, 4),
                   "i_base": round(base, 3), "i_mean": round(i_mean, 3), "i_peak": round(i_peak, 3),
                   "i_servo": round(i_mean - base, 3), "hold_s": round(t_read1 - t_read0, 2)}
            rows.append(row)
            ev("reading", **row)
            print(f"           -> {tau:.3f} N m, current {i_mean:.2f} A (base {base:.2f}, servo +{i_mean - base:.2f}, peak {i_peak:.2f})")
            time.sleep(a.rest)
    except KeyboardInterrupt:
        print("\n  interrupted — returning to contact and ending")
        goto(contact)
        ev("interrupt")
        time.sleep(1.0)
    finally:
        if not a.dry_run:
            # Legs clear first: the rescue pose (here, and the deadman's) is not designed around a
            # scale under one knee.
            try:
                if pre:
                    bench.call("servo.set", ch=ch, us=contact)
                    pose_to(a.pre_pose, pre)
                ask("  >> Move the scale away, then press Enter for the rescue pose ")
            except (KeyboardInterrupt, NameError):
                pass
            r = bench.call("limp")                      # the rescue pose, deadman still fed
            ev("rescue", ok=r.get("ok"))
            t0 = time.monotonic()
            while time.monotonic() - t0 < 15 and (bench.call("status").get("rescue_active")
                                                    or bench.call("status").get("pose_move_active")):
                time.sleep(0.2)
        stop_ping.set()
        rec.stop()
        json.dump({"leg": a.leg, "ch": ch, "contact_us": contact, "arm_mm": a.arm_mm, "step_us": a.step,
                   "rows": rows, "feed_frames": rec.n}, open(out / "meta.json", "w"), indent=1)
        if rows:
            print("\n  step  delta_us  grams  torque N m  servo A (battery side)")
            for r_ in rows:
                print(f"  {r_['step']:4d}  {r_['delta_us']:+8d}  {r_['grams']:5.0f}  {r_['torque_nm']:10.3f}  {r_['i_servo']:+.2f}")
        print(f"scale_probe: {len(rows)} readings, {rec.n} feed frames -> {out}")


if __name__ == "__main__":
    main()
