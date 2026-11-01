#!/usr/bin/env python3
"""touch_session -- read a bench PALPATION session from the daemon's own record (palpation stage 0, 2026-11-01).

The question the first session asks (robot resume, 2026-11-01): does the toe's FSR register a touch,
how repeatably, and where was the toe when it did?  Everything needed is in ONE file, benchd's
`benchd_<stamp>.jsonl`, once `adc.rate ms=20` was on for the session:

    adc_fast    the four FSR channels at 50 Hz, and since 2026-11-01 the twelve servos' current pulses (`pulse`)
                -> the touch ONSETS (counts cross the contact threshold) and the pose AT the onset
    telemetry   10 Hz: servos' current_us, imu.up_fused, ina.i_a, tof.m   -> the pose at onset, the lean, the current
    verb        every servo.set the dashboard sent      -> which channel was moving into the touch
    mark        the operator's fence posts               -> the condition each touch belongs to (none = one segment)
    cal.load    reply: the servo map                     -> channel -> (physical leg, joint, sign, origin)

For each touch: onset time, counts at onset and the plateau over the hold, the hold's length, the moving
channel, the toe's height from the planar FK of hip2 + knee at the daemon's current pulses -- the fast row's own
`pulse` when the daemon records it, else the telemetry frame before the onset, which session 1 showed is worth
5-30 mm of height at tapping speed (the `pose` column says which was used) -- (reach_check.py's
`toe_planar`, validated against the sim's _fk_leg), levelled by the fused up vector and referenced to the
median height of the OTHER planted feet -- the same two references TouchMemory and the dashboard's twin
use -- the bus-current step from before the touch to the hold, and the body's lean change.

    python3 pi_host/tools/touch_session.py <benchd_*.jsonl> [--segments "0-90:floor,90-200:box"] [--body measured]
                                           [--thresh 200] [--off 150] [--hold-s 1.5]

Segments are seconds on the SESSION clock (0 = the first fast ADC sample) and are only needed when no marks
were sent; with marks, each mark opens a segment named by its text.  Channels: A0 FL, A1 FR, A2 RL, A3 RR.
Nothing here is a lever; it is the reader the session's verdict is written from.
"""
from __future__ import annotations
import argparse, json, math, os, statistics, sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PHYS = ["FL", "FR", "RL", "RR"]                       # A0..A3
SIM_OF = {"FL": 1, "FR": 0, "RL": 3, "RR": 2}         # physical -> sim leg index (the mirror)
HEADINGS = [(-0.7071, 0.7071), (0.7071, 0.7071), (-0.7071, -0.7071), (0.7071, -0.7071)]   # sim index: (x, z)
US_PER_RAD = 545.2


def body_params(name):
    d = json.load(open(os.path.join(REPO, "godot_host/project/addons/ami_ogma/body", name + ".json")))
    L, r, c = d["links"], d["rest"], d["chassis"]
    return dict(L1=L["l1"], L2=L["l2"], L3=L["l3"], drop_z=L["coxa_z_drop"], DROP=-r["lower_leg_drop_angle"],
                hx=c["hip_x_span"] * 0.5, hz=c["hip_z_span"] * 0.5)


def toe_planar(p, h, k):
    """Toe (r along the leg's heading, y up) relative to hip1, metres (reach_check.py, validated vs _fk_leg)."""
    r = math.sqrt(max(0.0, p["L1"] ** 2 - p["drop_z"] ** 2)); y = -p["drop_z"]
    r += p["L2"] * math.cos(h); y -= p["L2"] * math.sin(h)
    tau = h + p["DROP"] + k
    r += p["L3"] * math.cos(tau); y -= p["L3"] * math.sin(tau)
    return r, y


def toe_body(p, phys, a_hip1, a_hip2, a_knee):
    """Toe in the body frame (x = body +X, y = up, z = forward), from hip1's corner anchor."""
    si = SIM_OF[phys]
    hx, hz = HEADINGS[si]
    # hip1 anchor at the chassis corner in the heading's quadrant; the heading rotated about +Y by hip1.
    ax, az = math.copysign(p["hx"], hx), math.copysign(p["hz"], hz)
    c, s = math.cos(a_hip1), math.sin(a_hip1)
    dx, dz = hx * c + hz * s, -hx * s + hz * c
    r, y = toe_planar(p, a_hip2, a_knee)
    return (ax + dx * r, y, az + dz * r)


def load(path):
    adc, tele, sets, marks, smap = [], [], [], [], None
    with open(path) as f:
        for line in f:
            try: r = json.loads(line)
            except Exception: continue
            k, t, d = r.get("kind"), r.get("t_mono_ms"), r.get("data") or {}
            if k == "adc_fast" and "a" in d: adc.append((t, d["a"], d.get("pulse")))
            elif k == "telemetry": tele.append((t, d))
            elif k == "mark": marks.append((t, d.get("text", "")))
            elif k == "verb":
                req = d.get("req") or {}
                if req.get("verb") == "servo.set": sets.append((t, int(req.get("ch", -1)), int(req.get("us", -1))))
                elif req.get("verb") == "cal.load" and isinstance(d.get("reply"), dict) and "map" in d["reply"]:
                    smap = d["reply"]["map"]
    return adc, tele, sets, marks, smap


def servo_map(smap):
    """ch -> dict(physical, joint, sign, origin); and (physical, joint) -> ch."""
    by_ch, by_pj = {}, {}
    for s in (smap or {}).get("servos", []):
        e = dict(physical=s["physical"], joint=s["joint"], sign=int(s.get("sign", 1)), origin=int(s.get("origin_us", 1500)))
        by_ch[int(s["ch"])] = e; by_pj[(e["physical"], e["joint"])] = int(s["ch"])
    return by_ch, by_pj


def angle(us, e):
    return e["sign"] * (us - e["origin"]) / US_PER_RAD


def frame_before(tele, t):
    lo, hi = 0, len(tele)
    while lo < hi:
        mid = (lo + hi) // 2
        if tele[mid][0] <= t: lo = mid + 1
        else: hi = mid
    return tele[lo - 1][1] if lo > 0 else None


def up_of(fr):
    u = ((fr or {}).get("imu") or {}).get("up_fused")
    if isinstance(u, list) and len(u) >= 3 and abs(u[1]) > 0.3: return tuple(float(x) for x in u[:3])
    return (0.0, 1.0, 0.0)


def events(adc, ch, thresh, off, min_on=3, min_off=3):
    """Rising edges of channel ch: (t_on, i_on, t_off, i_off)."""
    out, on, cnt_on, cnt_off, i_on = [], False, 0, 0, -1
    for i, row in enumerate(adc):
        t, a = row[0], row[1]
        v = a[ch]
        if not on:
            cnt_on = cnt_on + 1 if v >= thresh else 0
            if cnt_on >= min_on:
                on, i_on, cnt_off = True, i - min_on + 1, 0
        else:
            cnt_off = cnt_off + 1 if v < off else 0
            if cnt_off >= min_off:
                out.append((adc[i_on][0], i_on, adc[i - min_off + 1][0], i - min_off + 1)); on = False; cnt_on = 0
    if on: out.append((adc[i_on][0], i_on, adc[-1][0], len(adc) - 1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log"); ap.add_argument("--segments", default=""); ap.add_argument("--body", default="measured")
    ap.add_argument("--thresh", type=int, default=200); ap.add_argument("--off", type=int, default=150)
    ap.add_argument("--hold-s", type=float, default=1.5); ap.add_argument("--min-gap-s", type=float, default=0.3)
    ap.add_argument("--all-feet", action="store_true", help="keep events on feet a palpate mark did not name (the planted feet's co-events)")
    a = ap.parse_args()
    adc, tele, sets, marks, smap = load(a.log)
    if not adc: sys.exit("no adc_fast rows: was `adc.rate ms=20` on?")
    by_ch, by_pj = servo_map(smap)
    p = body_params(a.body)
    t0 = adc[0][0]
    print(f"session: {len(adc)} fast ADC samples over {(adc[-1][0]-t0)/1000:.1f} s, {len(tele)} telemetry frames, "
          f"{len(sets)} servo.set, {len(marks)} marks, servo map {'from the log' if smap else 'MISSING'}")
    # baseline per channel
    for ch in range(4):
        vals = [x[1][ch] for x in adc]
        frac = sum(1 for v in vals if v >= a.thresh) / len(vals)
        print(f"  A{ch} {PHYS[ch]}: median {statistics.median(vals):5.0f} counts, min {min(vals):4d}, max {max(vals):4d}, "
              f"above {a.thresh}: {100*frac:4.0f} % of the session")
    # segments
    segs = []
    def short(text):
        # palpate's marks: "palpate tap 7 cond=floor leg=FL knee=B hip1=-30 rep=1" -> "floor FL B -30"
        kv = dict(part.split("=", 1) for part in text.split() if "=" in part)
        if "leg" in kv: return " ".join(x for x in (kv.get("cond", ""), kv["leg"], kv.get("knee", ""), kv.get("hip1", "")) if x)
        return text
    if marks:
        for i, (t, text) in enumerate(marks):
            t_end = marks[i + 1][0] if i + 1 < len(marks) else adc[-1][0] + 1
            segs.append((t, t_end, short(text)))
    elif a.segments:
        for part in a.segments.split(","):
            rng, _, label = part.partition(":"); s0, _, s1 = rng.partition("-")
            segs.append((t0 + float(s0) * 1000, t0 + float(s1) * 1000, label or rng))
    if not segs: segs = [(t0, adc[-1][0] + 1, "all")]
    def seg_of(t):
        for s0, s1, lab in segs:
            if s0 <= t < s1: return lab
        return "-"
    # events on every channel
    rows = []
    for ch in range(4):
        for (t_on, i_on, t_off, i_off) in events(adc, ch, a.thresh, a.off):
            if rows and rows[-1]["ch"] == ch and t_on - rows[-1]["t_off"] < a.min_gap_s * 1000:
                continue
            hold = [x[1][ch] for x in adc[i_on:i_off + 1] if x[0] <= t_on + a.hold_s * 1000]
            fr = frame_before(tele, t_on); fr_pre = frame_before(tele, t_on - 600)
            servos = (fr or {}).get("servos") or []
            cur = {int(s["ch"]): int(s.get("current_us", -1)) for s in servos if isinstance(s, dict)}
            pose_src = "tele"
            if len(adc[i_on]) > 2 and isinstance(adc[i_on][2], list) and len(adc[i_on][2]) >= 12:
                cur = {c: int(adc[i_on][2][c]) for c in range(12)}; pose_src = "fast"
            # the channel that moved most in the 2 s before onset (from the dashboard's own commands)
            recent = [(t, c, us) for (t, c, us) in sets if t_on - 2000 <= t <= t_on + 100]
            mv = {}
            for (t, c, us) in recent: mv.setdefault(c, []).append(us)
            moving = max(mv.items(), key=lambda kv: (max(kv[1]) - min(kv[1]))) [0] if mv else -1
            moving_lbl = f"P{moving} {by_ch[moving]['physical']} {by_ch[moving]['joint']}" if moving in by_ch else ("none" if moving < 0 else f"P{moving}")
            # FK of the touching leg at onset, levelled by the fused up vector, relative to the other planted feet
            phys = PHYS[ch]; h_mm = toe_y_mm = plane_mm = float("nan"); up = up_of(fr)
            def toe_h(ph):
                try:
                    c1, c2, c3 = by_pj[(ph, "hip1")], by_pj[(ph, "hip2")], by_pj[(ph, "knee")]
                    tb = toe_body(p, ph, angle(cur[c1], by_ch[c1]), angle(cur[c2], by_ch[c2]), angle(cur[c3], by_ch[c3]))
                    return tb[0] * up[0] + tb[1] * up[1] + tb[2] * up[2], tb[1]
                except KeyError:
                    return float("nan"), float("nan")
            if by_pj and cur:
                hg, ty = toe_h(phys); toe_y_mm = ty * 1000
                others = [toe_h(PHYS[j])[0] for j in range(4) if j != ch and adc[i_on][1][j] >= a.thresh]
                others = [o for o in others if not math.isnan(o)]
                if others:
                    plane = statistics.median(others); plane_mm = plane * 1000; h_mm = (hg - plane) * 1000
                else:
                    h_mm = hg * 1000
            # current and lean
            def i_a(f): return float(((f or {}).get("ina") or {}).get("i_a", float("nan")))
            i_pre = i_a(fr_pre); i_hold = max((i_a(frame_before(tele, t)) for t in range(int(t_on), int(min(t_off, t_on + a.hold_s * 1000)) + 1, 100)), default=float("nan"))
            up_pre, up_hold = up_of(fr_pre), up_of(frame_before(tele, min(t_off, t_on + 700)))
            lean = math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(up_pre, up_hold))))))
            seg = seg_of(t_on)
            if not a.all_feet and seg.split() and len(seg.split()) >= 2 and seg.split()[1] in PHYS and seg.split()[1] != phys:
                continue          # a palpate mark names the driven foot: the planted feet's co-events are the body's rock, not touches
            rows.append(dict(ch=ch, phys=phys, seg=seg, t=(t_on - t0) / 1000, t_off=t_off, dur=(t_off - t_on) / 1000,
                             onset=adc[i_on][1][ch], plateau=max(hold) if hold else adc[i_on][1][ch], moving=moving_lbl,
                             h_mm=h_mm, toe_y_mm=toe_y_mm, plane_mm=plane_mm, i_pre=i_pre, i_hold=i_hold, lean=lean, pose=pose_src))
    rows.sort(key=lambda r: r["t"])
    print(f"\n{len(rows)} touch events (onset >= {a.thresh} counts for 60 ms, release < {a.off}):")
    print(f"{'t(s)':>7} {'foot':4} {"segment":16} {'onset':>5} {'plat':>5} {'hold s':>6} {'h vs plane mm':>13} {'toe y mm':>8} {'I pre->hold A':>14} {'lean deg':>8} {'pose':4}  moving")
    for r in rows:
        print(f"{r['t']:7.1f} {r['phys']:4} {r["seg"][:16]:16} {r['onset']:5d} {r['plateau']:5d} {r['dur']:6.2f} "
              f"{r['h_mm']:13.1f} {r['toe_y_mm']:8.1f} {r['i_pre']:6.2f}->{r['i_hold']:5.2f} {r['lean']:8.2f} {r['pose']:4}  {r['moving']}")
    # per segment / foot summary
    print("\nper segment and foot (n, onset counts mean±sd, plateau mean, h vs plane mm mean±sd, current step A mean):")
    groups = {}
    for r in rows: groups.setdefault((r["seg"], r["phys"]), []).append(r)
    def msd(xs):
        xs = [x for x in xs if not (isinstance(x, float) and math.isnan(x))]
        if not xs: return "   nan"
        return f"{statistics.mean(xs):6.1f}±{(statistics.pstdev(xs) if len(xs) > 1 else 0.0):4.1f}"
    for (seg, ph), rs in sorted(groups.items(), key=lambda kv: min(r["t"] for r in kv[1])):
        print(f"  {seg[:14]:14} {ph:3} n={len(rs):2d}  onset {msd([r['onset'] for r in rs])}  plateau {statistics.mean([r['plateau'] for r in rs]):6.0f}  "
              f"h {msd([r['h_mm'] for r in rs])} mm  I step {msd([r['i_hold'] - r['i_pre'] for r in rs])} A")
    # palpate passes: the same by (condition, leg, knee) across the hip1 stations -- the comparison across conditions
    passes = {}
    for r in rows:
        w = r["seg"].split()
        if len(w) >= 3 and w[1] in PHYS: passes.setdefault((w[0], w[1], w[2]), []).append(r)
    if passes:
        print("\nper condition, leg and knee setting across the hip1 stations (toe y = body-frame height at onset, from the fast rows' pulses):")
        for (cond, ph, kn), rs in sorted(passes.items(), key=lambda kv: min(r["t"] for r in kv[1])):
            print(f"  {cond:8} {ph} knee {kn}: n={len(rs):2d}  onset {msd([r['onset'] for r in rs])}  plateau {statistics.mean([r['plateau'] for r in rs]):6.0f}  "
                  f"toe y {msd([r['toe_y_mm'] for r in rs])} mm  h vs plane {msd([r['h_mm'] for r in rs])} mm  I step {msd([r['i_hold'] - r['i_pre'] for r in rs])} A")


if __name__ == "__main__":
    main()
