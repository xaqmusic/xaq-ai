#!/usr/bin/env python3
"""touch_curve -- the force-against-depth curve of each palpate tap, from the probe's own log.

palpate.py records, after each contact, the sole's counts at every further step of descent (`trace`: [depth_us,
counts]).  This turns that into a curve a box and a pillow can be told apart by: depth in millimetres of commanded
toe travel (the planar FK's slope at the tap's pose, through the servo map), load in grams through the FSR divider
curve BrainInputs uses, and the curve's shape -- where it saturates (a hard surface: the servo gives, the sole
loads within a millimetre) or keeps climbing (a soft one: the toe keeps sinking).

    python3 pi_host/tools/touch_curve.py <palpate_*.jsonl> [more ...] [--body measured_fsr]
"""
from __future__ import annotations
import argparse, json, math, os, statistics, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import touch_session as T

FSR_CURVE = [(60, 0.0), (1536, 30.0), (1920, 50.0), (2118, 100.0), (2362, 175.0), (2603, 300.0), (2925, 500.0)]


def grams(counts: float) -> float:
    if counts <= FSR_CURVE[0][0]: return 0.0
    for (c0, g0), (c1, g1) in zip(FSR_CURVE, FSR_CURVE[1:]):
        if counts <= c1: return g0 + (g1 - g0) * (counts - c0) / (c1 - c0)
    c0, g0 = FSR_CURVE[-2]; c1, g1 = FSR_CURVE[-1]
    return g1 + (g1 - g0) * (counts - c1) / (c1 - c0)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("logs", nargs="+"); ap.add_argument("--body", default="measured_fsr")
    ap.add_argument("--map", default=os.path.join(T.REPO, "pi_host/calib/servo_map.json"))
    ap.add_argument("--depths", default="0,1,2,4,6,8", help="commanded depths (mm) to tabulate the load at")
    a = ap.parse_args()
    p = T.body_params(a.body)
    smap = json.load(open(a.map)); by_ch, by_pj = T.servo_map(smap)
    rows = []
    for f in a.logs:
        rows += [json.loads(l) for l in open(f)]
    depths = [float(x) for x in a.depths.split(",")]
    d_mid = depths[len(depths) // 3]; d_end = depths[-1]
    print(f"{'cond':9} {'leg':3} {'hip1':>5} | {'mm/100us':>8} | load (g) at commanded depth " + " / ".join(f"{d:g}" for d in depths) + f" mm | g/mm 0-{d_mid:g} g/mm {d_mid:g}-{d_end:g}  shape")
    for r in rows:
        if r.get("result") != "contact" or not r.get("trace"): continue
        ph = r["leg"]; c1, c2, c3 = by_pj[(ph, "hip1")], by_pj[(ph, "hip2")], by_pj[(ph, "knee")]
        # the pose at contact: hip1 and knee from the station, hip2 at the contact pulse
        a1 = math.radians(r["hip1"]); a3 = math.radians({"A": 5.0, "B": 50.0}[r["knee"]])
        a2 = T.angle(r["contact_us"], by_ch[c2]); e2 = by_ch[c2]
        y0 = T.toe_body(p, ph, a1, a2, a3)[1]; y1 = T.toe_body(p, ph, a1, T.angle(r["contact_us"] + 100, e2), a3)[1]
        mm_per_100us = -(y1 - y0) * 1000.0            # commanded descent per 100 us of hip2
        if abs(mm_per_100us) < 1e-6: continue
        pts = [(d * mm_per_100us / 100.0, grams(c)) for d, c in r["trace"]]
        def at(mm):
            best = min(pts, key=lambda q: abs(q[0] - mm)); return best[1] if abs(best[0] - mm) <= 1.5 else float("nan")
        g = [at(m) for m in depths]
        def slope(m0, m1):
            seg = [q for q in pts if m0 <= q[0] <= m1]
            if len(seg) < 2: return float("nan")
            x = [q[0] for q in seg]; y = [q[1] for q in seg]; mx, my = statistics.mean(x), statistics.mean(y)
            sxx = sum((v - mx) ** 2 for v in x); return sum((v - mx) * (w - my) for v, w in zip(x, y)) / sxx if sxx > 1e-9 else float("nan")
        s02, s28 = slope(0, d_mid), slope(d_mid, d_end)
        deepest = max(q[0] for q in pts)
        shape = "flat" if not math.isnan(s28) and abs(s28) < 3 else ("climbing" if not math.isnan(s28) and s28 > 0 else ("falling" if not math.isnan(s28) else f"to {deepest:.1f} mm only"))
        print(f"{r['cond']:9} {ph:3} {int(r['hip1']):+5d} | {mm_per_100us:8.2f} | " + " / ".join(f"{v:4.0f}" if not math.isnan(v) else "   -" for v in g) +
              f" | {s02:8.1f} {s28:8.1f}  {shape}")


if __name__ == "__main__":
    main()
