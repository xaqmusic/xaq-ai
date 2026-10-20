#!/usr/bin/env python3
"""Terrain-adaptation scorecard for corridor runs (per-tick diag logs, OGMA_PICRAWLER_DIAG_INTERVAL=1).

Operator, 2026-10-19: "the original, while noisy, was very good at uneven terrain and adjusting the height
of the chassis accordingly; the new walker looks like a wind-up toy." This turns that observation into
numbers, per corridor zone (z of the chassis, the corridor's own layout in picrawler_body.gd):

  flat       z in [0.3, 1.9]    run-in
  hump_up    z in [2.0, 3.0]    the hump's climb (peak 3.0 m; height scales with difficulty, 66 mm at 0.3)
  hump_down  z in [3.0, 4.0]    its descent
  rumble     z in [4.2, 6.3]    half-buried cylinders every 0.5 m (28 mm exposed at 0.3)

Per zone, mean over seeds that entered it:
  reach      fraction of seeds whose chassis entered the zone
  ticks      ticks spent in the zone (a stall shows as many)
  belly      belly ToF clearance (gc_raw, m): mean and 5th percentile -- does the chassis keep its height?
  pitch      mean sin(pitch) (tilt_p): the body's attitude on the slope
  tilt_sd    attitude variability
  posture    ADAPTATION: mean |change| of each leg's STANCE hip2 and knee (the servo's commanded-lagged angle,
             planted ticks only) from its own flat-zone value, averaged over legs (rad). A body that adapts
             to terrain changes its stance posture where the ground changes; a fixed-shape gait does not.
  stride_cv  coefficient of variation of the touchdown interval (all legs): low = clockwork.
  falls      auto-resets that happened in the zone

Usage: terrain_scorecard.py NAME=<glob of logs> [NAME=<glob> ...]
"""
import glob, json, math, statistics as st, sys

ZONES = [("flat", 0.3, 1.9), ("hump_up", 2.0, 3.0), ("hump_down", 3.0, 4.0), ("rumble", 4.2, 6.3)]


def zone_of(z):
    for name, a, b in ZONES:
        if a <= z < b:
            return name
    return None


def one(path):
    rows = []
    for l in open(path, errors="replace"):
        if l.startswith("{") and '"con"' in l and '"x"' in l:
            try:
                rows.append(json.loads(l))
            except Exception:
                pass
    rows = [r for r in rows if r["t"] > 300]
    out = {}
    per = {name: [] for name, _, _ in ZONES}
    for i, r in enumerate(rows):
        zn = zone_of(r["z"])
        if zn:
            per[zn].append(i)
    flat_post = None
    for name, _, _ in ZONES:
        idx = per[name]
        if not idx:
            continue
        seg = [rows[i] for i in idx]
        o = {"ticks": len(seg)}
        gc = sorted(r.get("gc_raw", float("nan")) for r in seg)
        o["belly"] = st.mean(gc); o["belly_p05"] = gc[max(0, len(gc) // 20)]
        o["pitch"] = st.mean(r.get("cpitch", r.get("tilt_p", float("nan"))) for r in seg)   # true attitude when logged
        o["roll_abs"] = st.mean(abs(r.get("croll", r.get("tilt_r", float("nan")))) for r in seg)
        o["tilt_sd"] = st.pstdev(r.get("tilt", 0.0) for r in seg)
        post = []
        for k in range(4):
            planted = [r for r in seg if r["con"][k] > 0.5]
            if len(planted) < 20:
                post.append((float("nan"), float("nan")))
            else:
                post.append((st.mean(r["hip2"][k] for r in planted), st.mean(r["knee"][k] for r in planted)))
        o["_post"] = post
        if name == "flat":
            flat_post = post
        # touchdown intervals inside the zone
        ints = []
        for k in range(4):
            tds = [rows[i]["t"] for i in idx[1:] if rows[i]["con"][k] > 0.5 and rows[i - 1]["con"][k] <= 0.5]
            ints += [b - a for a, b in zip(tds, tds[1:]) if b - a < 400]
        o["stride_cv"] = (st.pstdev(ints) / st.mean(ints)) if len(ints) > 5 else float("nan")
        ar = [r.get("auto_reset_count", 0) for r in seg]
        o["falls"] = max(ar) - min(ar)
        out[name] = o
    for name in out:
        if flat_post is None:
            out[name]["posture"] = float("nan"); continue
        d = []
        for k in range(4):
            for j in (0, 1):
                a, b = out[name]["_post"][k][j], flat_post[k][j]
                if not (math.isnan(a) or math.isnan(b)):
                    d.append(abs(a - b))
        out[name]["posture"] = st.mean(d) if d else float("nan")
    return out


def main():
    keys = ["ticks", "belly", "belly_p05", "pitch", "roll_abs", "tilt_sd", "posture", "stride_cv", "falls"]
    for spec in sys.argv[1:]:
        name, _, pat = spec.partition("=")
        res = [one(p) for p in sorted(glob.glob(pat))]
        print(f"\n=== {name}  (n={len(res)})")
        print(f"  {'zone':10s} {'reach':>5s} " + " ".join(f"{k:>10s}" for k in keys))
        for zn, _, _ in ZONES:
            got = [r[zn] for r in res if zn in r]
            if not got:
                print(f"  {zn:10s} {0:5.2f}"); continue
            vals = []
            for k in keys:
                v = [g[k] for g in got if not (isinstance(g[k], float) and math.isnan(g[k]))]
                vals.append(f"{st.mean(v):10.4f}" if v else f"{'—':>10s}")
            print(f"  {zn:10s} {len(got) / len(res):5.2f} " + " ".join(vals))


if __name__ == "__main__":
    main()
