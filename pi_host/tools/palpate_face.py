#!/usr/bin/env python3
"""palpate_face -- the HORIZONTAL probe: one front toe advanced forward at a fixed height until it meets a face.

The vertical probe (palpate.py) answered tops; this answers FACES -- walls and the sides of boxes -- which the box
pass met by accident (2026-11-02): a toe meeting a vertical face loads the servo before it loads the sole, so the
bus current rose 0.6-0.8 A while the foot sensor stayed under threshold.  Here that is the instrument's design.

From `toes_up` (the belly resting), per leg and per height above the floor: hip1 is set so the leg's heading
points straight forward; the toe is lowered at `--r-start` mm from hip1 until the sole touches the FLOOR -- that
touch is the height reference for this probe, measured where the probe stands rather than extrapolated from the
calibration (which was fitted at other knee angles and read an inch where 40 mm was asked, 2026-11-02) -- then
hip2 and knee are solved together (a two-joint planar IK on the shared FK) to lift the toe by the requested height
and the toe advances `--step` mm every `--dt` s along the heading, the height held, until the FIRST of: the sole's counts cross
`--thresh`; the bus current rises `--i-guard` A over the pre-touch level; the body leans `--lean-guard` degrees;
the reach is exhausted (`--r-end`).  The record is the daemon's (adc_fast + marks + telemetry) and a local JSONL of
every step (reach, counts, current, lean, belly ToF), so a face's current-per-mm reads beside a top's.

    .venv/bin/python pi_host/tools/palpate_face.py --cond wall [--legs FL,FR] [--heights 20,40] [--hip1 auto]
                                                   [--r-start 90 --r-end 165 --step 1.0 --dt 0.1] [--dry]
"""
from __future__ import annotations
import argparse, json, math, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import touch_session as T
from palpate import Bench, counts, i_a, current_us, FSR_CH

FORWARD_HIP1 = {"FL": -45.0, "FR": +45.0, "RL": -135.0, "RR": +135.0}   # the hip1 angle that turns each PHYSICAL leg's heading to +Z (checked on the bench, 2026-11-02: FL -45 met the wall ahead)


def host_range(host: str, timeout_s: float = 1.0):
    """The forward ultrasonic from ogma_host's control port (plain TCP, one JSON line): (metres, valid) or (nan, False)."""
    import socket
    try:
        with socket.create_connection((host, 7400), timeout=timeout_s) as sk:
            sk.sendall(b'{"verb":"host_sensors"}\n'); buf = b""
            while b"\n" not in buf:
                chunk = sk.recv(4096)
                if not chunk: break
                buf += chunk
        rep = json.loads(buf.split(b"\n")[0].decode())
        r = rep.get("range") or {}
        return float(r.get("m", float("nan"))), bool(r.get("valid", False))
    except Exception:
        return float("nan"), False


SENSOR_AHEAD_MM = 49.0     # the ultrasonic's face ahead of the body origin (ProxMemory sensor_fwd_m)


def lean_deg(fr) -> float:
    u = ((fr or {}).get("imu") or {}).get("up_fused")
    if isinstance(u, list) and len(u) >= 3 and abs(float(u[1])) <= 1.0:
        return math.degrees(math.acos(max(-1.0, min(1.0, float(u[1])))))
    return 0.0


def tof_mm(fr) -> float:
    d = (fr or {}).get("tof") or {}
    return float(d.get("m", float("nan"))) * 1000.0 if d.get("valid") else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="picrawler.local"); ap.add_argument("--cond", required=True)
    ap.add_argument("--legs", default="FL,FR"); ap.add_argument("--heights", default="20,40", help="mm above the calibrated floor")
    ap.add_argument("--hip1", default="auto", help="degrees, or 'auto' = the angle that points the leg's heading forward")
    ap.add_argument("--r-start", type=float, default=120.0); ap.add_argument("--r-end", type=float, default=165.0)
    ap.add_argument("--step", type=float, default=1.0); ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--thresh", type=int, default=200); ap.add_argument("--i-guard", type=float, default=0.4); ap.add_argument("--lean-guard", type=float, default=8.0)
    ap.add_argument("--settle", type=float, default=1.5); ap.add_argument("--hold", type=float, default=0.6)
    ap.add_argument("--pose", default="toes_up"); ap.add_argument("--body", default="measured_fsr")
    ap.add_argument("--calib", default=os.path.join(T.REPO, "pi_host/calib/toe_height_calib.json"))
    ap.add_argument("--floor-mm", type=float, default=None, help="the floor below the body origin (default: the calibration's anchor, else -29)")
    ap.add_argument("--dry", action="store_true"); ap.add_argument("--reps", type=int, default=1)
    a = ap.parse_args()

    b = Bench(a.host); p = T.body_params(a.body)
    calib = T.load_calib(a.calib)
    floor = a.floor_mm
    if floor is None:
        try: floor = float(json.load(open(a.calib)).get("anchor_belly_mm", -29.0))
        except Exception: floor = -29.0
    smap = b.call(verb="cal.load")["map"]; by_ch, by_pj = T.servo_map(smap)
    lims = {int(s["ch"]): (int(s["min_us"]), int(s["max_us"])) for s in smap["servos"]}
    pose_us = [int(x) for x in b.call(verb="pose.get", name=a.pose)["us"]]
    fr = b.wait_frame()
    if str(fr.get("mode", "?")) != "bench": sys.exit(f"daemon mode is '{fr.get('mode')}', not bench")
    if bool(fr.get("stopped", False)): sys.exit("the robot is STOPPED")

    def us_of(ph, joint, rad):
        ch = by_pj[(ph, joint)]; e = by_ch[ch]
        us = int(round(e["origin"] + e["sign"] * rad * T.US_PER_RAD)); lo, hi = lims[ch]
        return ch, max(lo + 10, min(hi - 10, us))

    def within(ph, h, k):
        """the solution's pulses sit inside the channels' limits (an unclamped command)"""
        for joint, rad in (("hip2", h), ("knee", k)):
            ch = by_pj[(ph, joint)]; e = by_ch[ch]; us = e["origin"] + e["sign"] * rad * T.US_PER_RAD; lo, hi = lims[ch]
            if us < lo + 10 or us > hi - 10: return False
        return True

    def ik(ph, a1, r_mm, y_true_mm, h0, k0):
        """hip2, knee (rad) putting the toe at (r, y) in the leg's plane: Gauss-Newton from (h0, k0), then from a grid of
        seeds if that fails; a solution outside the channels' limits is no solution."""
        seeds = [(h0, k0)] + [(hh, kk) for hh in (-1.6, -1.2, -0.8, -0.4, 0.0) for kk in (-0.8, -0.3, 0.3, 0.8, 1.2)]
        for (hs, ks) in seeds:
            sol = ik_from(ph, a1, r_mm, y_true_mm, hs, ks)
            if sol is not None and within(ph, *sol): return sol
        return None

    def ik_from(ph, a1, r_mm, y_true_mm, h0, k0):
        h, k = h0, k0
        for _ in range(40):
            def f(h_, k_):
                r, y = T.toe_planar(p, h_, k_)
                return r * 1000.0 - r_mm, y * 1000.0 - y_true_mm
            e_r, e_y = f(h, k)
            if abs(e_r) < 0.2 and abs(e_y) < 0.2: break
            d = 1e-4
            er_h, ey_h = f(h + d, k); er_k, ey_k = f(h, k + d)
            J = [[(er_h - e_r) / d, (er_k - e_r) / d], [(ey_h - e_y) / d, (ey_k - e_y) / d]]
            det = J[0][0] * J[1][1] - J[0][1] * J[1][0]
            if abs(det) < 1e-9: return None
            dh = (-e_r * J[1][1] + e_y * J[0][1]) / det; dk = (-J[0][0] * e_y + J[1][0] * e_r) / det
            h += max(-0.3, min(0.3, dh)); k += max(-0.3, min(0.3, dk))
        e_r, e_y = f(h, k)
        return (h, k) if abs(e_r) < 1.0 and abs(e_y) < 1.0 else None

    legs = [x.strip().upper() for x in a.legs.split(",") if x.strip()]
    heights = [float(x) for x in a.heights.split(",")]
    plan = [(ph, hgt, rep) for ph in legs for hgt in heights for rep in range(1, a.reps + 1)]
    # the sonar's view of what is ahead: the expected reach of a face straight in front of a front hip
    def expected_reach():
        m, ok = host_range(a.host)
        if not ok or math.isnan(m): return float("nan"), m, ok
        return m * 1000.0 + SENSOR_AHEAD_MM - p["hz"] * 1000.0, m, ok
    r_exp, m_us, ok_us = expected_reach()
    print(f"  sonar: {('%.3f m' % m_us) if ok_us else 'no echo / no ogma_host'}" + (f" -> a face straight ahead would be met at reach ~{r_exp:.0f} mm" if ok_us else ""))
    print(f"palpate_face --cond {a.cond}: {len(plan)} probes; floor at {floor:+.0f} mm; reach {a.r_start:.0f} -> {a.r_end:.0f} mm at {a.step:.1f} mm / {a.dt:.2f} s; "
          f"stop on sole >= {a.thresh}, current +{a.i_guard} A, lean +{a.lean_guard} deg")
    seeds = {}
    for ph in legs:
        a1 = math.radians(float(a.hip1) if a.hip1 != "auto" else FORWARD_HIP1[ph])
        c1, u1 = us_of(ph, "hip1", a1)
        sol0 = ik(ph, a1, a.r_start, floor + heights[0], -0.6, 0.3); sol1 = ik(ph, a1, a.r_end, floor + heights[0], -0.6, 0.3)
        seeds[ph] = a1
        print(f"  {ph}: hip1 {math.degrees(a1):+.0f} deg -> P{c1} {u1} us (heading forward); at {heights[0]:.0f} mm: start (hip2, knee) = "
              + (f"{math.degrees(sol0[0]):+.0f}, {math.degrees(sol0[1]):+.0f} deg" if sol0 else "UNREACHABLE") + "; end = "
              + (f"{math.degrees(sol1[0]):+.0f}, {math.degrees(sol1[1]):+.0f} deg" if sol1 else "UNREACHABLE"))
    if a.dry: return

    stamp = time.strftime("%Y%m%d_%H%M%S"); out = open(f"palpate_face_{stamp}_{a.cond}.jsonl", "w")
    moved = {}
    def set_us(ch, us):
        rep = b.call(verb="servo.set", ch=ch, us=int(us)); moved.setdefault(ch, pose_us[ch]); return int(rep.get("clamped_us", us))
    def wait(s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s: b.ping(); b.poll(); time.sleep(0.03)
    try:
        b.call(verb="adc.rate", ms=20)
        cu = current_us(b.wait_frame())
        if any(abs(cu.get(c, 0) - pose_us[c]) > 15 for c in range(12)):
            print(f"  recalling '{a.pose}' (slewed) ..."); b.call(verb="pose.set", us=pose_us)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 15:
                f = b.wait_frame()
                if not bool(f.get("pose_move_active", False)) and time.monotonic() - t0 > 1.0: break
                b.ping(); time.sleep(0.1)
        b.call(verb="mark", text=f"palpate_face start cond={a.cond} legs={','.join(legs)} heights={a.heights} r={a.r_start:.0f}-{a.r_end:.0f} pose={a.pose}")
        print(f"{'#':>3} {'leg':3} {'h mm':>5} | {'floor fk':>8} {'reach at stop':>13} {'sole':>5} {'I pre->max':>12} {'A/mm':>6} {'lean rise':>9} {'ToF lift':>8} result")
        for n, (ph, hgt, rep) in enumerate(plan, 1):
            fsr = FSR_CH[ph]; a1 = seeds[ph]; y_t = floor + hgt
            c1, u1 = us_of(ph, "hip1", a1); c2 = by_pj[(ph, "hip2")]; c3 = by_pj[(ph, "knee")]
            # lift first (toes_up's own hip2), then the station; then FIND THE FLOOR at the start reach: the toe comes
            # down at r_start until the sole fires, and that FK height is this probe's floor.
            set_us(c2, pose_us[c2]); wait(0.3); set_us(c1, u1); wait(0.6)
            y_probe = floor + 35.0; h, k = -0.6, 0.3; y_floor = None
            sol = ik(ph, a1, a.r_start, y_probe, h, k)
            if sol is None: print(f"{n:3d} {ph:3} {hgt:5.0f} | start unreachable"); continue
            h, k = sol; set_us(c3, us_of(ph, "knee", k)[1]); wait(0.2); set_us(c2, us_of(ph, "hip2", h)[1]); wait(a.settle)
            f = b.wait_frame()
            while y_probe > floor - 25.0:
                y_probe -= 1.0; sol = ik(ph, a1, a.r_start, y_probe, h, k)
                if sol is None: break
                h, k = sol; set_us(c2, us_of(ph, "hip2", h)[1]); set_us(c3, us_of(ph, "knee", k)[1])
                t_step = time.monotonic()
                while time.monotonic() - t_step < a.dt: b.ping(); time.sleep(0.01)
                f = b.poll() or f
                if counts(f, fsr) >= a.thresh: y_floor = y_probe; break
            if y_floor is None:
                print(f"{n:3d} {ph:3} {hgt:5.0f} | no floor found at reach {a.r_start:.0f} mm (FK height down to {y_probe:.0f}) — skipped")
                set_us(c2, pose_us[c2]); wait(0.3); continue
            y_t = y_floor + hgt                       # the requested height above the floor the toe just felt
            for yy in range(1, int(hgt) + 1, 2):      # lift to it in 2 mm steps
                sol = ik(ph, a1, a.r_start, y_floor + yy, h, k)
                if sol is None: break
                h, k = sol; set_us(c2, us_of(ph, "hip2", h)[1]); set_us(c3, us_of(ph, "knee", k)[1]); wait(0.06)
            sol = ik(ph, a1, a.r_start, y_t, h, k)
            if sol is not None: h, k = sol; set_us(c2, us_of(ph, "hip2", h)[1]); set_us(c3, us_of(ph, "knee", k)[1])
            wait(a.settle)
            r_exp, m_us, ok_us = expected_reach()
            r_limit = min(a.r_end, r_exp + 30.0) if ok_us and not math.isnan(r_exp) else a.r_end
            b.call(verb="mark", text=f"palpate_face probe {n} cond={a.cond} leg={ph} height={hgt:.0f} hip1={math.degrees(a1):+.0f} rep={rep} pose={a.pose} floor_fk={y_floor:.0f} sonar_m={m_us if ok_us else 'none'}")
            f = b.wait_frame(); i_pre = i_a(f); lean0 = lean_deg(f); tof0 = tof_mm(f); i_max = i_pre; lean_max = lean0; tof_max = tof0
            if counts(f, fsr) >= a.thresh:
                print(f"{n:3d} {ph:3} {hgt:5.0f} | already in contact at the start — skipped"); continue
            r = a.r_start; steps = []; result = "none"; r_stop = None
            while r < r_limit:
                r += a.step
                sol = ik(ph, a1, r, y_t, h, k)
                if sol is None: result = "unreachable"; break
                h, k = sol
                set_us(c2, us_of(ph, "hip2", h)[1]); set_us(c3, us_of(ph, "knee", k)[1])
                t_step = time.monotonic()
                while time.monotonic() - t_step < a.dt: b.ping(); time.sleep(0.01)
                f = b.poll() or f
                c = counts(f, fsr); ia = i_a(f); ln = lean_deg(f); tf = tof_mm(f)
                if not math.isnan(ia): i_max = max(i_max, ia)
                lean_max = max(lean_max, ln)
                if not math.isnan(tf): tof_max = max(tof_max, tf) if not math.isnan(tof_max) else tf
                steps.append((round(r, 1), c, ia, round(ln, 2), tf))
                if bool(f.get("stopped", False)): raise KeyboardInterrupt
                if c >= a.thresh: result = "sole"; r_stop = r; break
                if not math.isnan(ia) and not math.isnan(i_pre) and ia - i_pre > a.i_guard: result = "current"; r_stop = r; break
                if ln - lean0 > a.lean_guard: result = "lean"; r_stop = r; break
            if r_stop is not None: wait(a.hold)
            # current per mm over the last 10 mm before the stop (the face's compliance), if it stopped
            a_per_mm = float("nan")
            if r_stop is not None and len(steps) >= 4:
                seg = [s_ for s_ in steps if s_[0] >= r_stop - 10 and not math.isnan(s_[2])]
                if len(seg) >= 3: a_per_mm = (seg[-1][2] - seg[0][2]) / max(0.5, seg[-1][0] - seg[0][0])
            tb = T.toe_body(p, ph, a1, h, k)
            print(f"{n:3d} {ph:3} {hgt:5.0f} | {y_floor:8.0f} {(r_stop if r_stop else r):13.1f} {counts(f, fsr):5d} {i_pre:5.2f}->{i_max:5.2f} {a_per_mm:6.3f} {lean_max - lean0:9.2f} "
                  f"{(tof_max - tof0) if not math.isnan(tof_max) else float('nan'):8.1f} {result}"
                  + (f"   (sonar {m_us:.3f} m -> expected reach {r_exp:.0f})" if ok_us else "   (no sonar)"))
            out.write(json.dumps(dict(n=n, cond=a.cond, leg=ph, height_mm=hgt, hip1_deg=math.degrees(a1), r_start=a.r_start, r_stop=r_stop, result=result,
                                      floor_fk_mm=y_floor, toe_body_mm=[tb[0] * 1000, tb[1] * 1000, tb[2] * 1000],
                                      sonar_m=(m_us if ok_us else None), expected_reach_mm=(r_exp if ok_us else None),
                                      i_pre=i_pre, i_max=i_max, a_per_mm=a_per_mm, lean0=lean0, lean_max=lean_max, tof0=tof0, tof_max=tof_max,
                                      steps=steps, t_wall=time.time())) + "\n"); out.flush()
            # retract: back along the heading, then lift
            while r > a.r_start:
                r -= 4.0; sol = ik(ph, a1, max(r, a.r_start), y_t, h, k)
                if sol is None: break
                h, k = sol; set_us(c2, us_of(ph, "hip2", h)[1]); set_us(c3, us_of(ph, "knee", k)[1]); wait(0.08)
            set_us(c2, pose_us[c2]); wait(0.3); set_us(c3, pose_us[c3]); set_us(c1, pose_us[c1]); wait(0.5)
        b.call(verb="mark", text=f"palpate_face end cond={a.cond}")
        print(f"\ndone: {len(plan)} probes -> {out.name}")
    except KeyboardInterrupt:
        print("\ninterrupted: retracting")
    finally:
        try:
            for ph in legs:
                c2 = by_pj[(ph, "hip2")]
                if c2 in moved: set_us(c2, pose_us[c2]); wait(0.3)
            for ch, u in moved.items():
                if by_ch.get(ch, {}).get("joint") != "hip2": set_us(ch, u); wait(0.05)
        except Exception as e:
            print(f"retract: {e}")
        out.close()


if __name__ == "__main__":
    main()
