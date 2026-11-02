#!/usr/bin/env python3
"""toe_floor_calib -- the per-leg floor calibration of the commanded toe kinematics, from a palpate floor pass.

The floor pass (palpate.py --cond floor) touches one known surface with each probed leg at several hip1 stations
and knee settings.  The commanded FK should put every one of those toes at the same height -- the floor -- and
it does not: session 2 (2026-11-01) read the left leg 7-12 mm lower than the right, with a knee-setting term
and a slope along hip1.  Those are biases of the commanded kinematics (servo-map origins, link lengths, compliance),
and the floor is the reference that measures them.  This fits them and writes the correction every touch
consumer applies (touch_session.py, the bench dashboard's twin, TouchMemory on the brain):

    y_true = y_FK + a_leg + b_leg * knee_angle + c_leg * hip1_angle          (mm; angles in rad, the map's convention)

ANCHOR.  While the belly rests on the floor (the ToF clearance ~0 in touch_ready) the floor IS the belly plane,
`--belly-mm` below the body origin: the body file's lowest chassis box (-26 mm for `measured_fsr`).  The sag
study's constant (2026-10-24) suggests the physical belly sits ~32 mm below the origin; `--belly-mm -32` is that
reading.  Inter-leg consistency does not depend on the choice; the absolute height of a voxel against the
dashboard's floor plane does, and the body file's plane is what the dashboard draws.

The planted legs (the rear pair in touch_ready) get a constant from their resting height; their knee and hip1
terms wait for a rear-leg floor pass.  Prints the fit, the residuals per tap, and the hip2-origin shift that would
absorb each leg's constant (for the operator: a servo-map change is a lever on the gait and is not made here).

    python3 pi_host/tools/toe_floor_calib.py <benchd_floor_pass.jsonl> [--body measured_fsr] [--belly-mm -26]
                                             [--out pi_host/calib/toe_height_calib.json]
"""
from __future__ import annotations
import argparse, json, math, os, statistics, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import touch_session as T


def lstsq(X, y):
    """Ordinary least squares for a few parameters, no numpy: solve (X^T X) b = X^T y by Gaussian elimination."""
    n, k = len(X), len(X[0])
    A = [[sum(X[r][i] * X[r][j] for r in range(n)) for j in range(k)] for i in range(k)]
    v = [sum(X[r][i] * y[r] for r in range(n)) for i in range(k)]
    for i in range(k):
        piv = max(range(i, k), key=lambda r: abs(A[r][i]))
        A[i], A[piv] = A[piv], A[i]; v[i], v[piv] = v[piv], v[i]
        if abs(A[i][i]) < 1e-12: return None
        for r in range(k):
            if r == i: continue
            f = A[r][i] / A[i][i]
            for c in range(i, k): A[r][c] -= f * A[i][c]
            v[r] -= f * v[i]
    return [v[i] / A[i][i] for i in range(k)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+", help="benchd records holding palpate floor passes (several are pooled)")
    ap.add_argument("--body", default="measured_fsr"); ap.add_argument("--belly-mm", type=float, default=-29.0,
                    help="the floor's height below the body origin while the belly rests on it: -29 is where the rear pair found the floor from toes_up (2026-11-02, belly ToF 0.0 mm); the body file's box says -26, the sag study's constant -32")
    ap.add_argument("--pose", default="toes_up", help="only taps whose mark names this start pose (the belly must rest on the floor); '' = any")
    ap.add_argument("--apply-config", action="append", default=[], help="brain config(s) whose TouchMemory module gets the constants as leg_y_offset_* (sim leg order: fl=physical FR, fr=FL, rl=RR, rr=RL); the knee and hip1 terms stay with the dashboard and the reader")
    ap.add_argument("--out", default=os.path.join(T.REPO, "pi_host/calib/toe_height_calib.json"))
    ap.add_argument("--thresh", type=int, default=200); ap.add_argument("--off", type=int, default=150)
    a = ap.parse_args()
    p = T.body_params(a.body); H = a.belly_mm
    pts = {ph: [] for ph in T.PHYS}; seen = set(); rest = {ph: [] for ph in T.PHYS}; by_ch = by_pj = None
    for log in a.logs:
        adc, tele, sets, marks, smap = T.load(log)
        if smap: by_ch, by_pj = T.servo_map(smap)
        if not by_ch: sys.exit("no servo map in the record (the dashboard's LOAD MAP / connect puts one there)")
        collect(a, p, H, adc, tele, marks, by_ch, by_pj, pts, seen, rest)
    write_fit(a, p, H, by_ch, by_pj, pts, rest)


def collect(a, p, H, adc, tele, marks, by_ch, by_pj, pts, seen, rest):
    def seg(t):
        best = ""
        for tm, text in marks:
            if tm <= t: best = text
        return dict(x.split("=", 1) for x in best.split() if "=" in x)

    def angles(ph, cur):
        return tuple(T.angle(cur[by_pj[(ph, j)]], by_ch[by_pj[(ph, j)]]) for j in ("hip1", "hip2", "knee"))

    def height(ph, cur, up):
        tb = T.toe_body(p, ph, *angles(ph, cur))
        return (tb[0] * up[0] + tb[1] * up[1] + tb[2] * up[2]) * 1000.0

    # probed legs: one point per (leg, knee, hip1) from the first onset after its mark
    for ch in range(4):
        ph = T.PHYS[ch]
        for (t_on, i_on, t_off, i_off) in T.events(adc, ch, a.thresh, a.off):
            kv = seg(t_on)
            if kv.get("cond") != "floor" or kv.get("leg") != ph: continue
            if a.pose and kv.get("pose", "") not in (a.pose, ""): continue       # older marks carry no pose: admitted
            key = (ph, kv.get("knee"), kv.get("hip1"))
            if key in seen: continue
            pulse = adc[i_on][2] if len(adc[i_on]) > 2 else None
            if not pulse: continue
            seen.add(key)
            cur = {c: int(pulse[c]) for c in range(12)}; up = T.up_of(T.frame_before(tele, t_on))
            pts[ph].append((angles(ph, cur), height(ph, cur, up), kv.get("knee"), kv.get("hip1")))
    # planted legs: resting height over the pass (median of the fast rows where the foot is loaded and no probe mark names it)
    global rest_n
    rest_n = rest_n + len(adc[::25]) if "rest_n" in globals() else len(adc[::25])
    for i, row in enumerate(adc[::25]):
        t, cnt = row[0], row[1]; pulse = row[2] if len(row) > 2 else None
        if not pulse: continue
        kv = seg(t)
        if kv.get("cond") != "floor" or (a.pose and kv.get("pose", "") not in (a.pose, "")): continue   # only inside the floor pass
        up = T.up_of(T.frame_before(tele, t)); cur = {c: int(pulse[c]) for c in range(12)}
        for ch in range(4):
            ph = T.PHYS[ch]
            if cnt[ch] >= a.thresh and kv.get("leg") != ph: rest[ph].append(height(ph, cur, up))



def write_fit(a, p, H, by_ch, by_pj, pts, rest):
    src = ", ".join(os.path.basename(l) for l in a.logs)
    calib = {"version": 1, "fitted": time.strftime("%Y-%m-%dT%H:%M:%S"), "source": src, "body": a.body, "pose": a.pose,
             "anchor_belly_mm": H, "model": "y_true_mm = y_fk_mm + a + b*knee_rad + c*hip1_rad", "legs": {}}
    print(f"floor calibration from {src}: body {a.body} (L3 {p['L3']*1000:.1f} mm), pose {a.pose or 'any'}, floor anchored at the belly plane {H:+.0f} mm")
    for ph in T.PHYS:
        P = pts[ph]
        if len(P) >= 4:
            X = [[1.0, th[2], th[0]] for (th, y, kn, h1) in P]; r = [H - y for (th, y, kn, h1) in P]
            b = lstsq(X, r)
            if b is None: continue
            res = [r[i] - (b[0] + b[1] * X[i][1] + b[2] * X[i][2]) for i in range(len(P))]
            raw_sd = statistics.pstdev([y for (_, y, _, _) in P])
            calib["legs"][ph] = {"a_mm": round(b[0], 2), "b_mm_per_rad_knee": round(b[1], 2), "c_mm_per_rad_hip1": round(b[2], 2),
                                 "n": len(P), "residual_sd_mm": round(statistics.pstdev(res), 2), "kind": "probed"}
            print(f"  {ph}: n={len(P)}  a {b[0]:+6.1f} mm  b {b[1]:+6.1f} mm/rad knee  c {b[2]:+6.1f} mm/rad hip1   "
                  f"raw floor sd {raw_sd:4.1f} -> residual sd {statistics.pstdev(res):4.1f} mm")
            for (th, y, kn, h1), e in zip(P, res):
                print(f"      knee {kn} hip1 {h1:>4}: fk {y:6.1f}  corrected {y + b[0] + b[1]*th[2] + b[2]*th[0]:6.1f}  residual {e:+5.1f}")
            # the hip2-origin shift that would absorb the constant: dy/dtheta2 at the mean pose
            th_m = [statistics.mean(th[i] for (th, _, _, _) in P) for i in range(3)]
            y0 = T.toe_body(p, ph, *th_m)[1] * 1000; y1 = T.toe_body(p, ph, th_m[0], th_m[1] + 0.01, th_m[2])[1] * 1000
            dy = (y1 - y0) / 0.01
            if abs(dy) > 1e-6:
                d_rad = -b[0] / dy; e2 = by_ch[by_pj[(ph, "hip2")]]
                print(f"      (a hip2 origin shift of {math.degrees(d_rad):+.1f} deg = {e2['sign'] * d_rad * T.US_PER_RAD:+.0f} us on P{by_pj[(ph, 'hip2')]} would absorb the constant — a servo-map lever, NOT applied)")
        elif len(rest[ph]) >= 20 and len(rest[ph]) >= 0.5 * max(1, rest_n):
            # a planted leg must have been loaded through most of the pass; a toe that touched for a moment is not a reference
            med = statistics.median(rest[ph])
            calib["legs"][ph] = {"a_mm": round(H - med, 2), "b_mm_per_rad_knee": 0.0, "c_mm_per_rad_hip1": 0.0,
                                 "n": len(rest[ph]), "residual_sd_mm": round(statistics.pstdev(rest[ph]), 2), "kind": "planted (constant only)"}
            print(f"  {ph}: planted, resting fk height {med:6.1f} mm (sd {statistics.pstdev(rest[ph]):.1f}) -> a {H - med:+6.1f} mm  (knee and hip1 terms unmeasured)")
        else:
            print(f"  {ph}: no data")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(calib, open(a.out, "w"), indent=1); print(f"-> {a.out}")
    # the brain's TouchMemory: constants only, in the SIM leg order the module indexes its feet by
    sim_of = {"FL": "fr", "FR": "fl", "RL": "rr", "RR": "rl"}
    for cfg_path in a.apply_config:
        raw = open(cfg_path).read(); cfg = json.loads(raw); done = False
        for m in cfg.get("modules", []):
            if m.get("type") != "TouchMemory": continue
            prm = m.setdefault("params", {})
            for ph, e in calib["legs"].items():
                prm["leg_y_offset_" + sim_of[ph]] = round(e["a_mm"] / 1000.0, 4)
            done = True
        if done:
            indent = 1 if raw.startswith("{\n \"") else 2
            open(cfg_path, "w").write(json.dumps(cfg, indent=indent, ensure_ascii=False) + ("\n" if raw.endswith("\n") else ""))
            print(f"-> {os.path.basename(cfg_path)}: TouchMemory leg_y_offset_* = " + ", ".join(f"{sim_of[ph]} {e['a_mm']/1000:+.4f}" for ph, e in calib["legs"].items()))
        else:
            print(f"   {os.path.basename(cfg_path)}: no TouchMemory module, untouched")


if __name__ == "__main__":
    main()
