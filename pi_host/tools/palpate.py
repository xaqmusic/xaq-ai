#!/usr/bin/env python3
"""palpate -- the scripted probe: one front toe lowered onto whatever is under it, station by station.

Session 1 (2026-11-01, ledger) was driven by hand and its error budget was the hand: the approach speed set
how far the pose record lagged the touch, and the path differed tap to tap.  This script makes the probe an
INSTRUMENT -- the same stations, the same knee settings, a slow constant descent, a mark before every tap --
so touches compare across conditions and sessions, as the sim's seed-averaged A/B compares arms.

It speaks benchd's CALIBRATION channel only (PROTOCOL.md): servo.set under the 30/s guard, pose.get/pose.set,
mark, adc.rate, ping.  It cannot start a brain and the daemon refuses it in any brain mode.  The dashboard may
stay open: it shows the voxels the touches make.  Ctrl-C retracts the leg and exits.

THE VERTICAL PROBE (floors, box tops, pillows).  For each leg, hip1 station and knee setting: set the station,
settle, then lower hip2 by `--step` us every `--dt` s (default 6 us / 60 ms ~ 100 us/s; at the 10 Hz telemetry
that stops within ~1 mm) until the foot's counts cross `--thresh` in two consecutive frames; then keep lowering
`--extra` us while recording counts against depth (the COMPLIANCE: a box rises steeply, a pillow slowly), hold,
retract.  No contact within `--travel` us is recorded as "none" and retracted.  A bus-current rise of more than
`--i-guard` A over the pre-touch level ends any push at once.  The direction "down" is read from the planar FK
(reach_check.py's toe_planar), not assumed.

    .venv/bin/python pi_host/tools/palpate.py --cond floor [--host picrawler.local] [--legs FL,FR]
            [--stations -60,-30,0,30,60] [--knees A,B] [--reps 1] [--dry]

Record: the daemon's benchd_*.jsonl (adc_fast + marks + telemetry) is the evidence; read it with
touch_session.py.  This script also writes palpate_<stamp>_<cond>.jsonl beside it on the laptop with each
tap's summary, for a look while the session runs.
"""
from __future__ import annotations
import argparse, json, math, os, sys, time

try:
    import zmq
except ImportError:
    sys.exit("palpate needs pyzmq: run with the repo's venv (.venv/bin/python)")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import touch_session as T   # body params, planar FK, the servo map reader

FSR_CH = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}
KNEE_DEG = {"A": 5.0, "B": 50.0}        # A: femur ~perpendicular to the tibia; B: more bent (session 1's two settings)


class Bench:
    def __init__(self, host: str, timeout_ms: int = 3000):
        self.ctx = zmq.Context()
        self.req = self.ctx.socket(zmq.REQ); self.req.setsockopt(zmq.RCVTIMEO, timeout_ms); self.req.setsockopt(zmq.LINGER, 0)
        self.req.connect(f"tcp://{host}:5590")
        self.sub = self.ctx.socket(zmq.SUB); self.sub.setsockopt(zmq.CONFLATE, 1); self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub.connect(f"tcp://{host}:5591")
        self.last_ping = 0.0
        self.frame = None

    def call(self, **req) -> dict:
        self.req.send_string(json.dumps(req))
        try:
            rep = json.loads(self.req.recv_string())
        except zmq.Again:
            raise RuntimeError(f"no reply from benchd to {req.get('verb')}")
        if not rep.get("ok", False):
            raise RuntimeError(f"{req.get('verb')}: {rep.get('error', rep)}")
        return rep

    def ping(self):
        now = time.monotonic()
        if now - self.last_ping > 0.3:
            self.call(verb="ping"); self.last_ping = now

    def poll(self) -> dict | None:
        """The latest telemetry frame (CONFLATE: at most one is queued)."""
        got = None
        while True:
            try:
                msg = self.sub.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            parts = msg.split(" ", 1)
            try:
                got = json.loads(parts[1] if len(parts) > 1 else parts[0])
            except Exception:
                got = None
        if got is not None: self.frame = got
        return self.frame

    def wait_frame(self, max_s: float = 2.0) -> dict:
        t0 = time.monotonic()
        while time.monotonic() - t0 < max_s:
            f = self.poll()
            if f is not None: return f
            time.sleep(0.02)
        raise RuntimeError("no telemetry from benchd (is it running, is the port open?)")


def counts(frame: dict, ch: int) -> int:
    adc = frame.get("adc") or []
    return int(adc[ch]) if ch < len(adc) else 0


def i_a(frame: dict) -> float:
    return float(((frame or {}).get("ina") or {}).get("i_a", float("nan")))


def current_us(frame: dict) -> dict:
    return {int(s["ch"]): int(s.get("current_us", -1)) for s in (frame.get("servos") or []) if isinstance(s, dict)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="picrawler.local"); ap.add_argument("--cond", required=True, help="condition label for the marks (floor, box5, pillow, wall ...)")
    ap.add_argument("--legs", default="FL,FR"); ap.add_argument("--stations", default="-60,-30,0,30,60", help="hip1 angles, degrees")
    ap.add_argument("--knees", default="A,B"); ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--step", type=int, default=6); ap.add_argument("--dt", type=float, default=0.06)
    ap.add_argument("--extra", type=int, default=60, help="us of further descent after contact (the compliance record)")
    ap.add_argument("--travel", type=int, default=900, help="cap on the descent before giving up (no contact); the FK sets a tighter one per knee setting")
    ap.add_argument("--thresh", type=int, default=200); ap.add_argument("--i-guard", type=float, default=0.6)
    ap.add_argument("--settle", type=float, default=1.5); ap.add_argument("--hold", type=float, default=0.6)
    ap.add_argument("--pose", default="touch_ready"); ap.add_argument("--body", default="measured")
    ap.add_argument("--dry", action="store_true", help="print the plan, command nothing")
    ap.add_argument("--no-recall", action="store_true", help="do not pose.set the start pose first")
    ap.add_argument("--start-at", type=int, default=1, help="resume an interrupted pass at this tap number")
    ap.add_argument("--lift", type=int, default=0, help="start hip2 this many us further UP than the start pose (tall objects: at knee B the pose's own start already sits a 60 mm box's height)")
    a = ap.parse_args()

    b = Bench(a.host)
    p = T.body_params(a.body)
    smap = b.call(verb="cal.load")["map"]
    by_ch, by_pj = T.servo_map(smap)
    lims = {int(s["ch"]): (int(s["min_us"]), int(s["max_us"])) for s in smap["servos"]}
    pose_us = [int(x) for x in b.call(verb="pose.get", name=a.pose)["us"]]
    fr = b.wait_frame()
    if str(fr.get("mode", "?")) != "bench": sys.exit(f"daemon mode is '{fr.get('mode')}', not bench: the brain holds the servos")
    if bool(fr.get("stopped", False)): sys.exit("the robot is STOPPED (SPACE in the dashboard to resume)")
    if str(fr.get("body", "?")) != "measured": sys.exit(f"daemon body is '{fr.get('body')}', calibration needs 'measured'")

    def us_of(ph, joint, deg):
        ch = by_pj[(ph, joint)]; e = by_ch[ch]
        us = int(round(e["origin"] + e["sign"] * math.radians(deg) * T.US_PER_RAD))
        lo, hi = lims[ch]; return ch, max(lo + 10, min(hi - 10, us))

    def toe_y(ph, cur):
        c1, c2, c3 = by_pj[(ph, "hip1")], by_pj[(ph, "hip2")], by_pj[(ph, "knee")]
        return T.toe_body(p, ph, *(T.angle(cur[c], by_ch[c]) for c in (c1, c2, c3)))[1]

    legs = [x.strip().upper() for x in a.legs.split(",") if x.strip()]
    stations = [float(x) for x in a.stations.split(",")]
    knees = [x.strip().upper() for x in a.knees.split(",")]
    plan = []
    for ph in legs:
        for kn in knees:
            for st in stations:
                for rep in range(1, a.reps + 1):
                    plan.append((ph, kn, st, rep))
    # The hip2 start is the start pose's own value (front legs up in touch_ready: the femur near vertical).
    # "Down" is NOT the local slope there -- at the top of the arc the toe moves sideways -- but the direction
    # toward the hip2 pulse at which the planar FK puts the toe LOWEST for that knee setting; the travel is
    # that distance plus a margin.  (Session 1's floor taps sat near that minimum: FL hip2 ~1080 us, knee A.)
    start = {}
    for ph in legs:
        c2 = by_pj[(ph, "hip2")]; lo, hi = lims[c2]
        start[ph] = pose_us[c2]
        for kn in knees:
            cur = dict(enumerate(pose_us)); cur[by_pj[(ph, "knee")]] = us_of(ph, "knee", KNEE_DEG[kn])[1]
            best_us, best_y = None, 1e9
            for us in range(lo, hi + 1, 5):
                cur[c2] = us; y = toe_y(ph, cur)
                if y < best_y: best_y, best_us = y, us
            start[(ph, kn, "dir")] = +1 if best_us > start[ph] else -1
            if a.lift > 0:        # further from the lowest point, within the channel's limits
                start[ph] = max(lo + 10, min(hi - 10, pose_us[c2] - start[(ph, kn, "dir")] * a.lift))
            start[(ph, kn, "travel")] = min(900, abs(best_us - start[ph]) + 150)
            start[(ph, kn, "lowest")] = (best_us, best_y)
    print(f"palpate --cond {a.cond}: {len(plan)} taps; start pose '{a.pose}'; stations {stations} deg; knees {knees}")
    for ph in legs:
        c2 = by_pj[(ph, "hip2")]
        print(f"  {ph}: hip2 P{c2} starts at {start[ph]} us; hip1 stations -> {[us_of(ph, 'hip1', s)[1] for s in stations]} us")
        for kn in knees:
            bu, by_ = start[(ph, kn, "lowest")]
            print(f"      knee {kn} ({KNEE_DEG[kn]:.0f} deg -> {us_of(ph, 'knee', KNEE_DEG[kn])[1]} us): toe lowest at hip2 {bu} us (toe y {by_*1000:.0f} mm below the body), "
                  f"so down is {'+' if start[(ph, kn, 'dir')] > 0 else '-'}us, travel limit {start[(ph, kn, 'travel')]} us")
    if a.dry:
        return

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = f"palpate_{stamp}_{a.cond}.jsonl"
    out = open(out_path, "w")
    moved: dict[int, int] = {}     # ch -> start pose value, for the retract on exit

    def set_us(ch, us):
        rep = b.call(verb="servo.set", ch=ch, us=int(us)); moved.setdefault(ch, pose_us[ch]); return int(rep.get("clamped_us", us))

    def wait(s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            b.ping(); b.poll(); time.sleep(0.03)

    def settled(chs, tol=6, max_s=4.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < max_s:
            f = b.wait_frame(); cu = current_us(f)
            tg = {int(s["ch"]): int(s.get("target_us", -1)) for s in f.get("servos", []) if isinstance(s, dict)}
            if all(abs(cu.get(c, -99) - tg.get(c, 99)) <= tol for c in chs): return True
            b.ping(); time.sleep(0.05)
        return False

    try:
        if not a.no_recall:
            b.call(verb="adc.rate", ms=20)
            cu = current_us(b.wait_frame())
            if any(abs(cu.get(c, 0) - pose_us[c]) > 15 for c in range(12)):
                print(f"  recalling '{a.pose}' (slewed) ...")
                b.call(verb="pose.set", us=pose_us)
                t0 = time.monotonic()
                while time.monotonic() - t0 < 15:
                    f = b.wait_frame()
                    if not bool(f.get("pose_move_active", False)) and time.monotonic() - t0 > 1.0: break
                    b.ping(); time.sleep(0.1)
        else:
            b.call(verb="adc.rate", ms=20)
        b.call(verb="mark", text=f"palpate start cond={a.cond} legs={','.join(legs)} stations={a.stations} knees={','.join(knees)} reps={a.reps} step={a.step}us/{a.dt}s")
        print(f"{'#':>3} {'leg':3} {'knee':4} {'hip1':>5} {'rep':>3} | {'hip2 start':>10} {'contact us':>10} {'depth us':>8} {'onset':>5} {'plat':>5} {'slope c/us':>10} {'toe y mm':>8} {'I pre->max':>12} result")
        for n, (ph, kn, st, rep) in enumerate(plan, 1):
            if n < a.start_at: continue
            fsr = FSR_CH[ph]; c1, u1 = us_of(ph, "hip1", st); c3, u3 = us_of(ph, "knee", KNEE_DEG[kn]); c2 = by_pj[(ph, "hip2")]
            direction = start[(ph, kn, "dir")]; u2 = start[ph]; travel = start[(ph, kn, "travel")]
            # station: raise hip2 to the start first (clear of the floor), then hip1 and the knee
            set_us(c2, u2); wait(0.3); set_us(c1, u1); set_us(c3, u3)
            settled([c1, c2, c3]); wait(a.settle)
            b.call(verb="mark", text=f"palpate tap {n} cond={a.cond} leg={ph} knee={kn} hip1={st:+.0f} rep={rep}")
            f = b.wait_frame(); i_pre = i_a(f); i_max = i_pre
            if counts(f, fsr) >= a.thresh:
                print(f"{n:3d} {ph:3} {kn:4} {st:+5.0f} {rep:3d} | {u2:10d} {'-':>10} {'-':>8} {'-':>5} {'-':>5} {'-':>10} {'-':>8} {'-':>12} ALREADY IN CONTACT at the start — station skipped")
                continue
            # descend
            us = u2; hit = None; hits = 0; result = "none"; trace = []
            while abs(us - u2) < min(a.travel, travel):
                us = set_us(c2, us + direction * a.step)
                t_step = time.monotonic()
                while time.monotonic() - t_step < a.dt:
                    b.ping(); time.sleep(0.01)
                f = b.poll() or f
                c = counts(f, fsr); ia = i_a(f)
                if not math.isnan(ia): i_max = max(i_max, ia)
                if not math.isnan(ia) and not math.isnan(i_pre) and ia - i_pre > a.i_guard:
                    result = "current guard"; break
                if bool(f.get("stopped", False)): raise KeyboardInterrupt
                if hit is None:
                    hits = hits + 1 if c >= a.thresh else 0
                    if hits >= 2:
                        hit = us; trace.append((0, c)); result = "contact"
                else:
                    trace.append((abs(us - hit), c))
                    if abs(us - hit) >= a.extra: break
            if hit is not None:
                wait(a.hold)
                f = b.wait_frame(); plat = max([x[1] for x in trace] + [counts(f, fsr)])
                slope = (trace[-1][1] - trace[0][1]) / max(1, trace[-1][0]) if len(trace) > 1 else float("nan")
                cur = current_us(f); cur[c2] = hit
                ty = toe_y(ph, cur) * 1000
                print(f"{n:3d} {ph:3} {kn:4} {st:+5.0f} {rep:3d} | {u2:10d} {hit:10d} {abs(hit-u2):8d} {trace[0][1]:5d} {plat:5d} {slope:10.1f} {ty:8.1f} {i_pre:5.2f}->{i_max:5.2f} {result}")
            else:
                ty = float("nan"); plat = 0; slope = float("nan")
                print(f"{n:3d} {ph:3} {kn:4} {st:+5.0f} {rep:3d} | {u2:10d} {'-':>10} {abs(us-u2):8d} {'-':>5} {'-':>5} {'-':>10} {'-':>8} {i_pre:5.2f}->{i_max:5.2f} {result}")
            out.write(json.dumps(dict(n=n, cond=a.cond, leg=ph, knee=kn, hip1=st, rep=rep, hip2_start=u2, contact_us=hit, result=result,
                                      trace=trace, plateau=plat, slope_counts_per_us=slope, toe_y_mm=ty, i_pre=i_pre, i_max=i_max,
                                      t_wall=time.time())) + "\n"); out.flush()
            # retract in larger steps
            while abs(us - u2) > 0:
                us = set_us(c2, us - direction * min(20, abs(us - u2))); wait(0.04)
            wait(0.3)
        b.call(verb="mark", text=f"palpate end cond={a.cond}")
        print(f"\ndone: {len(plan)} taps -> {out_path}; the record is the Pi's benchd_*.jsonl (read with touch_session.py)")
    except KeyboardInterrupt:
        print("\ninterrupted: retracting")
    finally:
        try:
            for ph in legs:
                c2 = by_pj[(ph, "hip2")]
                if c2 in moved:
                    f = b.wait_frame(); cu = current_us(f); us = cu.get(c2, start[ph]); u2 = start[ph]
                    d = +1 if u2 > us else -1
                    while abs(us - u2) > 0:
                        us += d * min(20, abs(us - u2)); set_us(c2, us); wait(0.04)
            for ch, u in moved.items():
                if by_ch.get(ch, {}).get("joint") != "hip2": set_us(ch, u); wait(0.05)
        except Exception as e:
            print(f"retract: {e}")
        out.close()


if __name__ == "__main__":
    main()
