#!/usr/bin/env python3
"""Export the ToF studies' figure data as one compact JSON for the report page."""
import json, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from tof_prep import label_fov
from tof_cloud import stops, cloud_in, vox, solid_cells

REPO = Path(__file__).resolve().parents[2]
NPZ = REPO / "mj_host/log/tof_study/npz"
EPM = REPO / "mj_host/log/tof_study/epm"
out = {}

# ---- S1/S2 numbers straight from the study JSONs ----
for tag, f in (("s6", "base_s6b_s12.json"), ("s3", "base_s3b_s12.json")):
    out[f"s12_{tag}"] = json.load(open(EPM / f))

# ---- raw-64 PCA scatter, coloured by what is in view ----
d = np.load(NPZ / "base_s6.npz", allow_pickle=True); d = {k: d[k] for k in d.files}
best, dist, bear, names, kinds = label_fov(d, max_range=2.0)
sel = np.where(d["t"] >= 700)[0]
sel = sel[::6]
raw = d["tofr"][sel].copy(); raw[raw < 0] = 4.0; raw /= 4.0
Xc = raw - raw.mean(0)
U, S, Vt = np.linalg.svd(Xc, full_matrices=False)
P = Xc @ Vt[:3].T
var = (S ** 2) / (len(Xc) - 1); var /= var.sum()
kind_of = {i: k for i, k in enumerate(kinds)}
lab = [kind_of.get(int(b), "none") if b >= 0 else "none" for b in best[sel]]
out["raw_pca"] = dict(var=[round(float(v), 4) for v in var[:8]],
                      pts=[[round(float(a), 3), round(float(b2), 3), l]
                           for a, b2, l in zip(P[:, 0], P[:, 1], lab)])

# ---- the cloud: one cast vs one sweep, for the richest stop ----
best_stop, best_n = None, -1
for lo, hi in stops(d):
    p, nc = cloud_in(d, lo, hi, derotate_from=lo)
    if len(p) > best_n: best_n, best_stop = len(p), (lo, hi)
lo, hi = best_stop
sweep, ncast = cloud_in(d, lo, hi, derotate_from=lo)
ci = d["cloud_i"][(d["cloud_i"] >= lo) & (d["cloud_i"] < hi)]
one, _ = cloud_in(d, ci[len(ci) // 2], ci[len(ci) // 2] + 1)
rng = np.random.default_rng(0)
sub = sweep[rng.choice(len(sweep), min(3000, len(sweep)), replace=False)]
# the objects in that stop's frame, as instrumentation for the reader
x0, y0, yaw0 = float(d["x"][lo]), float(d["y"][lo]), float(d["hdg"][lo, 0])
objs = []
sx = np.concatenate([d["static_xy"][:, 0], d["free"][lo, :, 0]])
sy = np.concatenate([d["static_xy"][:, 1], d["free"][lo, :, 1]])
ext = np.concatenate([d["static_ext"], np.full(len(d["mover_names"]), 0.05, np.float32)])
for j, (nm, kd) in enumerate(zip(names, kinds)):
    dx, dy = float(sx[j]) - x0, float(sy[j]) - y0
    fx = math.cos(-yaw0) * dx - math.sin(-yaw0) * dy
    fy = math.sin(-yaw0) * dx + math.cos(-yaw0) * dy
    if math.hypot(fx, fy) < 2.5 and kd != "wall":
        objs.append([round(fx, 3), round(fy, 3), round(float(ext[j]), 3), kd])
out["cloud"] = dict(secs=round((hi - lo) / 50.0, 1), casts=int(ncast),
                    one=[[round(float(a), 3), round(float(b), 3), round(float(c), 3)] for a, b, c in one],
                    sweep=[[round(float(a), 3), round(float(b), 3), round(float(c), 3)] for a, b, c in sub],
                    n_sweep=len(sweep), objs=objs)

json.dump(out, open(sys.argv[1], "w"))
print(f"figure data -> {sys.argv[1]}  ({Path(sys.argv[1]).stat().st_size/1024:.0f} kB)")
print(f"  raw_pca {len(out['raw_pca']['pts'])} pts;  cloud stop {out['cloud']['secs']} s, "
      f"{out['cloud']['casts']} casts, one cast {len(out['cloud']['one'])} pts, sweep {out['cloud']['n_sweep']} pts")
