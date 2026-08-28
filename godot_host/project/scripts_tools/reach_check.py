#!/usr/bin/env python3
"""Leg-extension check on JSON-per-line traces: how much of the leg's reach does the
planted foot actually use?

WHY THIS EXISTS.  The port doc's hardware gate is "`foot_r` inside the real leg's 156.5 mm
reach with margin".  But `foot_r` in arenaavg.py is the horizontal distance from the CHASSIS
CENTRE to the TIBIA MIDPOINT (`foot_xz` = the lower capsule's origin), while reach is
hip1->toe.  Those are different quantities with different origins, so the ledger's
"170 mm vs 166 mm reach" comparison never measured the gate.  This script computes the
gate's own quantity from the achieved hinge angles in the trace:

    ext  = |hip1 -> toe| / (L1 + L2 + L3)        1.0 = leg straight, full extension

plus the postural angles in the body's REAL hinge frame.  Hinge 0 is the CONSTRUCTION
pose (femur horizontal outward, tibia dropped by LOWER_LEG_DROP_ANGLE = -80 deg), NOT a
straight leg -- picrawler_body.gd builds every segment with an identity basis and
`_relative_angle_world_axis` measures rotation from that pose.  Sign conventions, read
off the FK spot table `_report_reach_gates()` prints in every run's log:
    hip2  < 0  raises the femur;  hip2 > 0 lowers it
    knee  > 0  folds the tibia UNDER the femur (the standing side); knee < 0 straightens it
The planar model below reproduces that table to 0.1 mm (cad and measured), so the
numbers here are the body's own FK, not a reading of comments.

  fold      = 80 deg + knee              0 = straight, 180 = tibia parallel to femur
  tib_vert  = 90 - (hip2 + 80 + knee)    tibia angle from vertical, + = toe outward
  (arenaavg's `tib_off = |hip2 + knee + pi/2|` assumes hinge 0 = straight; it under-reads
  the sprawl by ~30 deg.  Kept there for continuity; use tib_vert here.)

Usage:  reach_check.py <log-or-dir> [<log-or-dir> ...]
Each log is self-describing: the body comes from its GEOMETRY RECEIPT line and the link
lengths from addons/ami_ogma/body/<name>.json.  Planted = feet_y < STANCE_TH (0.04, the
body's own threshold).  Warm-up skip matches arenaavg.py (SEEDAVG_WARMUP, 900 ticks).
Prints one block per body: per-seed rows, then mean +- sd across seeds.
"""
import json, math, os, pathlib, re, statistics, sys

PROJ = pathlib.Path(__file__).resolve().parents[1]
STANCE_TH = float(os.environ.get("OGMA_PICRAWLER_STANCE_Y_THRESHOLD", 0.04))
WARMUP = int(os.environ.get("SEEDAVG_WARMUP", 900))
EXT_STRAIGHT = 0.95     # "straight-legged" if the toe is beyond this fraction of reach


def body_params(name):
    d = json.load(open(PROJ / "addons/ami_ogma/body" / f"{name}.json"))
    L = d["links"]; r = d["rest"]
    return dict(L1=L["l1"], L2=L["l2"], L3=L["l3"], drop_z=L["coxa_z_drop"],
                DROP=-r["lower_leg_drop_angle"])


def toe_planar(p, h, k):
    """Toe (r along heading, y up) rel hip1, metres.  Validated against _fk_leg."""
    r = math.sqrt(max(0.0, p["L1"] ** 2 - p["drop_z"] ** 2)); y = -p["drop_z"]
    r += p["L2"] * math.cos(h); y -= p["L2"] * math.sin(h)
    tau = h + p["DROP"] + k
    r += p["L3"] * math.cos(tau); y -= p["L3"] * math.sin(tau)
    return r, y


def analyse(path):
    body = None; p = None; reach = None
    ext_pl, r_pl, fold_pl, tibv_pl, h_pl = [], [], [], [], []
    ext_all = []; foot_r_legacy = []; ys = []
    n_pl = 0; n_frames = 0
    for line in open(path):
        if body is None:
            m = re.search(r"GEOMETRY RECEIPT '(\w+)'", line)
            if m:
                body = m.group(1); p = body_params(body); reach = p["L1"] + p["L2"] + p["L3"]
            continue
        if not line.startswith("{"):
            continue
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if d.get("t", 0) < WARMUP:
            continue
        h2, kn, fy = d.get("hip2"), d.get("knee"), d.get("feet_y")
        if not (isinstance(h2, list) and isinstance(kn, list) and isinstance(fy, list)):
            continue
        n_frames += 1
        ys.append(d.get("y", float("nan")))
        for q in d.get("foot_xz") or []:
            if isinstance(q, list) and len(q) == 2:
                foot_r_legacy.append(math.hypot(q[0], q[1]))
        for i in range(4):
            r, y = toe_planar(p, h2[i], kn[i])
            e = math.hypot(r, y) / reach
            ext_all.append(e)
            if fy[i] < STANCE_TH:
                n_pl += 1
                ext_pl.append(e); r_pl.append(r)
                fold_pl.append(math.degrees(p["DROP"] + kn[i]))
                tibv_pl.append(90.0 - math.degrees(h2[i] + p["DROP"] + kn[i]))
                h_pl.append(math.degrees(h2[i]))
    if body is None or not ext_pl:
        return None
    ext_sorted = sorted(ext_pl)
    return dict(body=body, reach_mm=reach * 1000, frames=n_frames, planted=n_pl,
                ext_mean=statistics.mean(ext_pl),
                ext_p95=ext_sorted[int(0.95 * (len(ext_sorted) - 1))],
                ext_max=ext_sorted[-1],
                straight_frac=sum(1 for e in ext_pl if e > EXT_STRAIGHT) / len(ext_pl),
                toe_r_hip1_mm=statistics.mean(r_pl) * 1000,
                fold_deg=statistics.mean(fold_pl),
                tib_vert_deg=statistics.mean(tibv_pl),
                hip2_deg=statistics.mean(h_pl),
                chassis_y_mm=statistics.mean(ys) * 1000,
                foot_r_legacy_mm=(statistics.mean(foot_r_legacy) * 1000) if foot_r_legacy else float("nan"))


COLS = [("ext_mean", ".3f"), ("ext_p95", ".3f"), ("ext_max", ".3f"), ("straight_frac", ".2f"),
        ("toe_r_hip1_mm", ".1f"), ("fold_deg", ".1f"), ("tib_vert_deg", ".1f"),
        ("hip2_deg", ".1f"), ("chassis_y_mm", ".1f"), ("foot_r_legacy_mm", ".1f")]


def main(argv):
    logs = []
    for a in argv:
        pa = pathlib.Path(a)
        logs += sorted(pa.glob("*.log")) if pa.is_dir() else [pa]
    by_body = {}
    for lg in logs:
        res = analyse(lg)
        if res:
            by_body.setdefault(res["body"], []).append((lg.name, res))
    for body, rows in by_body.items():
        reach = rows[0][1]["reach_mm"]
        print(f"== body {body}  reach {reach:.1f} mm  planted = feet_y < {STANCE_TH}  "
              f"straight = ext > {EXT_STRAIGHT}  (n={len(rows)} logs)")
        print("   " + " ".join(f"{k:>15}" for k, _ in COLS) + "   frames/planted")
        for name, r in rows:
            print("   " + " ".join(f"{r[k]:>15{f}}" for k, f in COLS) + f"   {r['frames']}/{r['planted']}  {name}")
        if len(rows) > 1:
            line = []
            for k, f in COLS:
                v = [r[k] for _, r in rows]
                line.append(f"{statistics.mean(v):{f}}+-{statistics.pstdev(v):{f}}")
            print("   mean+-sd  " + "  ".join(line))
    if not by_body:
        print("no analysable logs (need a GEOMETRY RECEIPT line and hip2/knee/feet_y trace fields)")


if __name__ == "__main__":
    main(sys.argv[1:])
