#!/usr/bin/env python3
"""tof_prep.py — one pass over a ToF-study JSONL into numpy arrays.

The ToF studies (2026-09-12) need the same stream many times over, so it is parsed once.
Everything world-derived here is INSTRUMENTATION, in the repo's sense: object positions and
labels are used to JUDGE what the sensor and the EPMs can separate, never fed to a brain.

Beam geometry (Tof.cpp): zone i -> row i//8 (0 = TOP), col i%8 (0 = LEFT); FOV 45 deg, zone
centres inset half a zone, so elevation = +19.6875 - 5.625*row deg and azimuth = +19.6875 -
5.625*col deg in the sensor frame (+x forward, +y left, +z up).
"""
import json, math, sys
import numpy as np
from pathlib import Path

ROWS = COLS = 8
HALF_DEG = 45.0 / 2.0 - 45.0 / COLS / 2.0
STEP_DEG = 2.0 * HALF_DEG / (COLS - 1)
EL_DEG = np.array([HALF_DEG - STEP_DEG * r for r in range(ROWS)])     # per row, + = up
AZ_DEG = np.array([HALF_DEG - STEP_DEG * c for c in range(COLS)])     # per col, + = left
DRIVE = {"walk": 0, "stand": 1, "scaffold": 2}


def load(path, manifest):
    man = json.load(open(manifest))
    statics = [(o["name"], o["kind"], o["x"], o["y"], o.get("extent", 0.1))
               for o in man["objects"] if o["cls"] != "movable"]
    movers = [o["name"] for o in man["objects"] if o["cls"] == "movable"]

    t, x, y, z, tilt, drv, stop, wall, obj, mtle = ([] for _ in range(10))
    tofr, tofz, mp, hdg, tw, q = [], [], [], [], [], []
    cloud_i, cloud = [], []                       # cast ticks only
    free = []
    for i, line in enumerate(open(path)):
        d = json.loads(line)
        t.append(d["t"]); x.append(d["x"]); y.append(d["y"]); z.append(d.get("z", 0.0))
        tilt.append(d.get("tilt", 0.0)); drv.append(DRIVE.get(d.get("drive", "walk"), 0))
        stop.append(d.get("stop", 0)); wall.append(d.get("wall", 0)); obj.append(d.get("obj", 0))
        mtle.append(d.get("mtle", float("nan")))
        tofr.append(d["tofr"]); tofz.append([int(c) for c in d["tofz"]])
        mp.append(d.get("map", [0, 0, -1, -1])); hdg.append(d.get("hdg", [0.0, 0.0]))
        tw.append(d.get("twist", [0.0, 0.0, 0.0])); q.append(d.get("q", [0.0] * 14))
        free.append([d.get("free", {}).get(nm, [0, 0, 0]) for nm in movers])
        if "tofp" in d:
            pts = np.full((64, 3), np.nan, dtype=np.float32)
            for zi, px, py, pz in d["tofp"]:
                pts[zi] = (px, py, pz)
            cloud_i.append(i); cloud.append(pts)

    out = dict(
        t=np.array(t, np.float32), x=np.array(x, np.float32), y=np.array(y, np.float32),
        z=np.array(z, np.float32), tilt=np.array(tilt, np.float32), drive=np.array(drv, np.int8),
        stop=np.array(stop, np.int8), wall=np.array(wall, np.int8), obj=np.array(obj, np.int8),
        mtle=np.array(mtle, np.float32), tofr=np.array(tofr, np.float32),
        tofz=np.array(tofz, np.int8), map=np.array(mp, np.float32), hdg=np.array(hdg, np.float32),
        twist=np.array(tw, np.float32), q=np.array(q, np.float32),
        free=np.array(free, np.float32), cloud_i=np.array(cloud_i, np.int32),
        cloud=np.array(cloud, np.float32),
        static_xy=np.array([[s[2], s[3]] for s in statics], np.float32),
        static_ext=np.array([s[4] for s in statics], np.float32),
    )
    out["static_names"] = np.array([s[0] for s in statics])
    out["static_kinds"] = np.array([s[1] for s in statics])
    out["mover_names"] = np.array(movers)
    return out


def label_fov(d, max_range=2.0):
    """For every tick: the nearest object inside the ToF's 45 deg cone, as an index into a
    combined [statics..., movers...] list, plus its distance and bearing.  -1 = nothing in view."""
    yaw = d["hdg"][:, 0]
    n = len(yaw)
    names = list(d["static_names"]) + list(d["mover_names"])
    kinds = list(d["static_kinds"]) + ["ball" if "ball" in m else "block" for m in d["mover_names"]]
    px = np.concatenate([np.repeat(d["static_xy"][:, 0:1], n, 1).T, d["free"][:, :, 0]], 1)
    py = np.concatenate([np.repeat(d["static_xy"][:, 1:2], n, 1).T, d["free"][:, :, 1]], 1)
    ext = np.concatenate([d["static_ext"], np.full(len(d["mover_names"]), 0.05, np.float32)])
    dx = px - d["x"][:, None]; dy = py - d["y"][:, None]
    dist = np.hypot(dx, dy)
    bear = np.arctan2(dy, dx) - yaw[:, None]
    bear = (bear + np.pi) % (2 * np.pi) - np.pi
    # a wall is not a point: its four entries share the room centre, so they are excluded here
    # and walls are labelled geometrically by the room's own extent instead.
    is_wall = np.array([k == "wall" for k in kinds])
    half_fov = math.radians(22.5)
    in_fov = (np.abs(bear) < half_fov + np.arctan2(ext, np.maximum(dist, 0.05))) & (dist < max_range) & ~is_wall
    d2 = np.where(in_fov, dist, np.inf)
    idx = np.argmin(d2, 1)
    best = np.where(np.isfinite(d2[np.arange(n), idx]), idx, -1)
    return best, dist, bear, names, kinds


def point_heights(d):
    """Height above the floor of every returned point, per cast tick (trunk-frame z + trunk z)."""
    return d["cloud"][:, :, 2] + d["z"][d["cloud_i"]][:, None]


if __name__ == "__main__":
    src, man, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    d = load(src, man)
    np.savez_compressed(dst, **d)
    print(f"{Path(src).name}: {len(d['t'])} ticks, {len(d['cloud_i'])} casts -> {dst}")
