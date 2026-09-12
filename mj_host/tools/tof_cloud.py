#!/usr/bin/env python3
"""tof_cloud.py — study 3 of the ToF work (2026-09-12): the point cloud the head babble builds.

The operator's idea: at a stop the duck babbles its gaze and accumulates the returns into a cloud
(nothing baked), then watches the cloud for CHANGE.  Three questions:
  C1  what does a sweep buy over a single frame — solid angle, points, voxels?
  C2  is the body still enough for the cloud to compose (it is in the levelled TRUNK frame, which
      is fixed only while the trunk is)?
  C3  can a change in the cloud detect a mover that a single frame misses?  R44's single-frame
      detector caught a rolling ball 42 % of the time; that is the number to beat.
"""
import json, math, sys
from pathlib import Path
import numpy as np

VOX = 0.04          # 4 cm voxels: a block is 4-5 cm, so one voxel is one block
CELL_DEG = 5.0      # solid-angle cells for coverage


def stops(d):
    """contiguous runs of stop phase > 0, as (start, end) indices into the tick arrays"""
    s = d["stop"] > 0
    edges = np.diff(s.astype(int))
    starts = list(np.where(edges > 0)[0] + 1)
    ends = list(np.where(edges < 0)[0] + 1)
    if s[0]: starts = [0] + starts
    if s[-1]: ends = ends + [len(s)]
    return [(a, b) for a, b in zip(starts, ends) if b - a > 50]


def cloud_in(d, lo, hi, derotate_from=None):
    """Every levelled point cast between tick lo and hi, as (n, 3), z measured from the floor.

    The levelled trunk frame is fixed only while the TRUNK is, and the trunk's heading drifts ~10 deg
    over a stop (measured below) -- 17 cm of smear at a metre, against a 4 cm block.  With
    derotate_from set, each cast is turned back by its own (yaw - yaw_ref) before it joins the cloud.
    The duck has that yaw from its own contact odometry to 0.1 deg (design doc §16.3), so this is a
    correction it can make itself, not an oracle."""
    m = (d["cloud_i"] >= lo) & (d["cloud_i"] < hi)
    ci = d["cloud_i"][m]
    pts = d["cloud"][m]                      # casts x 64 x 3
    z = d["z"][ci]
    p = pts.copy()
    p[:, :, 2] = pts[:, :, 2] + z[:, None]   # z becomes height above the floor
    if derotate_from is not None:
        dy = d["hdg"][ci, 0] - d["hdg"][derotate_from, 0]
        c, s_ = np.cos(-dy), np.sin(-dy)
        x0, y0 = p[:, :, 0].copy(), p[:, :, 1].copy()
        p[:, :, 0] = c[:, None] * x0 - s_[:, None] * y0
        p[:, :, 1] = s_[:, None] * x0 + c[:, None] * y0
    p = p.reshape(-1, 3)
    ok = np.isfinite(p).all(1)
    return p[ok], int(m.sum())


def vox(p):
    return set(map(tuple, np.floor(p / VOX).astype(np.int32)))


def solid_cells(p):
    r = np.linalg.norm(p, axis=1) + 1e-9
    az = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    el = np.degrees(np.arcsin(np.clip(p[:, 2] / r, -1, 1)))
    return set(zip(np.floor(az / CELL_DEG).astype(int), np.floor(el / CELL_DEG).astype(int)))


def c1c2(d, name):
    print(f"\n=== C1/C2  what a gaze sweep buys  [{name}] ===")
    rows = []
    for lo, hi in stops(d):
        p_all, ncast = cloud_in(d, lo, hi)
        p_rot, _ = cloud_in(d, lo, hi, derotate_from=lo)
        if ncast < 20: continue
        # one cast in the middle, for the comparison
        mid = d["cloud_i"][(d["cloud_i"] >= lo) & (d["cloud_i"] < hi)][ncast // 2]
        p_one, _ = cloud_in(d, mid, mid + 1)
        drift = float(np.hypot(d["x"][hi - 1] - d["x"][lo], d["y"][hi - 1] - d["y"][lo]))
        hdrift = float(abs(((d["hdg"][hi - 1, 0] - d["hdg"][lo, 0]) + np.pi) % (2 * np.pi) - np.pi))
        yaw_sd = float(np.std(d["q"][lo:hi, 7]))
        rows.append(dict(secs=(hi - lo) / 50.0, casts=ncast, pts_one=len(p_one), pts_all=len(p_all),
                         cells_one=len(solid_cells(p_one)), cells_all=len(solid_cells(p_all)),
                         vox_one=len(vox(p_one)), vox_all=len(vox(p_all)), vox_rot=len(vox(p_rot)),
                         cells_rot=len(solid_cells(p_rot)),
                         drift=drift, hdrift=math.degrees(hdrift), yaw_sd=yaw_sd))
    if not rows:
        print("  no stops"); return []
    k = lambda f: np.mean([r[f] for r in rows])
    print(f"  {len(rows)} stops, {k('secs'):.0f} s each, {k('casts'):.0f} casts")
    print(f"  points       one cast {k('pts_one'):6.0f}   whole sweep {k('pts_all'):7.0f}   x{k('pts_all')/k('pts_one'):.0f}")
    print(f"  5 deg cells  one cast {k('cells_one'):6.0f}   whole sweep {k('cells_all'):7.0f}   x{k('cells_all')/k('cells_one'):.1f}")
    print(f"  4 cm voxels  one cast {k('vox_one'):6.0f}   whole sweep {k('vox_all'):7.0f}   x{k('vox_all')/k('vox_one'):.1f}")
    print(f"  de-rotated  one cast {k('vox_one'):6.0f}   whole sweep {k('vox_rot'):7.0f}   x{k('vox_rot')/k('vox_one'):.1f}"
          f"   ({100*(k('vox_rot')-k('vox_all'))/k('vox_all'):+.0f} % voxels vs raw: less smear = fewer, sharper)")
    print(f"  body drift over a stop {k('drift')*100:.1f} cm, heading {k('hdrift'):.1f} deg;  head-yaw sd {k('yaw_sd'):.3f} rad")
    print(f"    (the cloud lives in the levelled TRUNK frame, so the drift is its blur)")
    return rows


def c3(d, name, mover="obj_ball0"):
    """change detection: newly-occupied voxels in a 2 s window against a 4 s reference,
    scored against the mover's own motion as ground truth."""
    names = list(d["mover_names"])
    if mover not in names:
        print(f"  {mover} not in {names}"); return None
    mi = names.index(mover)
    pos = d["free"][:, mi, :2]
    speed = np.r_[0.0, np.linalg.norm(np.diff(pos, axis=0), axis=1)] * 50.0
    moving = speed > 0.05
    print(f"\n=== C3  change in the cloud  [{name}] ===")
    print(f"  {mover} moves on {int(moving.sum())} ticks ({100*moving.mean():.1f} %), peak {speed.max():.2f} m/s")
    rolling = (speed > 0.05) & (speed < 5.0)           # the ball actually rolling
    teleport = speed >= 5.0                             # the harness PLACING it: also a real change
    W, REF = int(1.0 * 50), int(4.0 * 50)
    scores, labels, single, rot = [], [], [], []
    for lo, hi in stops(d):
        for t in range(lo + REF + W, hi, W):
            ref, nr = cloud_in(d, t - W - REF, t - W, derotate_from=lo)
            cur, nc = cloud_in(d, t - W, t, derotate_from=lo)
            ref0, _ = cloud_in(d, t - W - REF, t - W)
            cur0, _ = cloud_in(d, t - W, t)
            if nr < 10 or nc < 10: continue
            vr, vc = vox(ref0), vox(cur0)
            new = len(vc - vr) / max(1, len(vc))       # fraction of the window's voxels unseen in the reference
            scores.append(new)
            vr2, vc2 = vox(ref), vox(cur)
            rot.append(len(vc2 - vr2) / max(1, len(vc2)))
            labels.append(bool((rolling | teleport)[t - W:t].any()))
            # the single-frame comparison: the largest per-cast jump in mean range in the window
            m = (d["cloud_i"] >= t - W) & (d["cloud_i"] < t)
            rr = np.nanmean(np.where(d["tofr"][d["cloud_i"][m]] > 0, d["tofr"][d["cloud_i"][m]], np.nan), axis=1)
            single.append(float(np.nanmax(np.abs(np.diff(rr)))) if len(rr) > 2 else 0.0)
    scores = np.array(scores); labels = np.array(labels); single = np.array(single); rot = np.array(rot)
    if labels.sum() < 3:
        print(f"  only {int(labels.sum())} positive windows — not enough"); return None
    print(f"  {len(scores)} windows, {int(labels.sum())} with the mover in motion")
    for nm, sc in (("single frame: max range jump", single), ("cloud: new voxels", scores),
                   ("cloud, de-rotated", rot)):
        pos_s, neg_s = sc[labels], sc[~labels]
        # detection rate at the threshold that gives a 5 % false-positive rate on quiet windows
        thr = np.quantile(neg_s, 0.95)
        det = float((pos_s > thr).mean())
        # and the rank statistic, threshold-free
        auc = float((pos_s[:, None] > neg_s[None, :]).mean() + 0.5 * (pos_s[:, None] == neg_s[None, :]).mean())
        print(f"    {nm:30s} mean {pos_s.mean():.3f} moving vs {neg_s.mean():.3f} quiet;"
              f"  detection at 5 % FP {100*det:4.0f} %;  AUC {auc:.3f}")
    return dict(n=len(scores), pos=int(labels.sum()))


def c4(d, name):
    """The question the single-frame studies could not answer: does a SWEEP put enough points on a
    small object to represent it?  M1 measured 1-2 zones on a block per frame, at the sensor's
    resolution limit.  Counted here in the de-rotated cloud of each stop, against the object's own
    position expressed in that cloud's frame (instrumentation: the duck never reads it)."""
    print(f"\n=== C4  points on an object: one frame vs one sweep  [{name}] ===")
    names = list(d["static_names"]) + list(d["mover_names"])
    kinds = list(d["static_kinds"]) + ["ball" if "ball" in m else "block" for m in d["mover_names"]]
    tops = {"block": 0.10, "ball": 0.12, "chair": 0.55, "table": 0.60, "shelf": 0.95,
            "rug": 0.02, "clock": 0.55, "wall": 0.55}
    agg = {}
    for lo, hi in stops(d):
        p_all, ncast = cloud_in(d, lo, hi, derotate_from=lo)
        if ncast < 20: continue
        x0, y0, yaw0 = d["x"][lo], d["y"][lo], d["hdg"][lo, 0]
        sx = np.concatenate([d["static_xy"][:, 0], d["free"][lo, :, 0]])
        sy = np.concatenate([d["static_xy"][:, 1], d["free"][lo, :, 1]])
        ext = np.concatenate([d["static_ext"], np.full(len(d["mover_names"]), 0.05, np.float32)])
        for j, (nm, kd) in enumerate(zip(names, kinds)):
            if kd == "wall": continue
            dx, dy = sx[j] - x0, sy[j] - y0
            fx = np.cos(-yaw0) * dx - np.sin(-yaw0) * dy
            fy = np.sin(-yaw0) * dx + np.cos(-yaw0) * dy
            dist = math.hypot(fx, fy)
            # in range AND inside the cone the gaze actually sweeps: the sensor's 22.5 deg half-FOV
            # plus the babble's own yaw excursion (sd 0.35 rad, so ~2 sd = 40 deg of reach).
            if dist > 2.0 or abs(math.degrees(math.atan2(fy, fx))) > 60.0: continue
            rad = float(ext[j]) + 0.06
            m = (np.hypot(p_all[:, 0] - fx, p_all[:, 1] - fy) < rad) & (p_all[:, 2] < tops.get(kd, 0.6)) & (p_all[:, 2] > 0.015)
            a = agg.setdefault(kd, dict(stops=0, pts=0, casts=0, hits=0))
            a["stops"] += 1; a["pts"] += int(m.sum()); a["casts"] += ncast
            a["hits"] += 1 if m.sum() >= 3 else 0
    print(f"    {'kind':8s} {'stops in range':>14s} {'points/sweep':>13s} {'points/cast':>12s} {'sweeps with >=3 pts':>20s}")
    for kd, a in sorted(agg.items(), key=lambda kv: -kv[1]["pts"]):
        if a["stops"] == 0: continue
        print(f"    {kd:8s} {a['stops']:>14d} {a['pts']/a['stops']:>13.1f} "
              f"{a['pts']/max(1,a['casts']):>12.3f} {a['hits']}/{a['stops']:<19d}")
    return agg


if __name__ == "__main__":
    for f in sys.argv[1:]:
        d = np.load(f, allow_pickle=True); d = {k: d[k] for k in d.files}
        n = Path(f).stem
        c1c2(d, n)
        c4(d, n)
        c3(d, n)
