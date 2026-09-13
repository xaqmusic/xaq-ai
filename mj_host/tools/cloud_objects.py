#!/usr/bin/env python3
"""cloud_objects — what the duck's sweep clouds say about small objects on the floor, scored offline.

Reads the `cloudv` records ogma::CloudMap files into a host run's JSONL.  The log must be FULL: the host run with
`--cloud` (and `--log-cloud-profile`), or a harness run with `l2_sweep.py --logdir DIR --full-logs`; the harness's
compact stream drops the cloud records, qpos and joint positions.

  cloud_objects.py rules LOG... [--gapk 0.12]
      the STACK RULE's ladder (design doc §17.31): every break-band cluster by what it really was, and the precision
      and recall of each rule for telling a small object from a wall base, a chair leg or furniture
  cloud_objects.py arms NAME=GLOB [NAME=GLOB ...]
      per arm and seed: the head's motion through the stops, cloud size, the objects within 2 m found, balls found,
      and the rule's precision -- the R52 gaze-sweep A/B

The reduction runs in the cloud's own body-anchored frame, which is what a brain could compute: break-band voxels
(mean point height 2-20 cm) grouped into 8-connected columns; each cluster's STACK TOP is the contiguous chain of
voxel heights over its footprint (dilated by one voxel) with a gap of max(10 cm, GAPK x range), because the ToF's
rows are 5.625 deg apart and the vertical spacing of its returns grows with distance.  A cluster that tops out below
16 cm and spans at most 20 cm is a SMALL THING; one that keeps rising is an obstacle.

Scoring only (instrumentation, never an input): the world pose each cloud was anchored on and the free bodies'
qpos at filing time, against scene_playroom.manifest.json, label clusters and objects.  A cluster within 8 cm of a
ball's or block's surface is that object; within 12 cm of a wall, a wall; near a chair, table or shelf, that.
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
import math
import os
import statistics
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
MANIFEST = REPO / "mj_host/models/microduck/scene_playroom.manifest.json"
BREAK_LO, BREAK_HI = 0.02, 0.20
SMALL_TOP, SMALL_EXT = 0.16, 0.20
GAP_MIN = 0.10
NEAR_M = 2.0                                                   # objects this close to a cloud's anchor are "in reach"
BABBLE_FIELD = 0.7 + math.radians(22.5)                        # the babble's yaw clamp + half the sensor's field
HEAD_YAW_Q, HEAD_PITCH_Q = 7, 6                                # joint order: duck-control's JOINT_NAMES minus the mouth


def load_scene(path: Path = MANIFEST):
    m = json.load(open(path))
    lay = {n: i for n, i, _k in m["qpos_layout"]}
    movable = [o for o in m["objects"] if o["cls"] == "movable"]
    furniture = [o for o in m["objects"] if o["kind"] in ("chair", "table", "shelf")]
    return m["half"], lay, movable, furniture


def mean_heights(c: dict) -> tuple[np.ndarray, np.ndarray]:
    V = np.array([v + [float("nan")] * (5 - len(v)) for v in c["vox"]], float)
    h = np.where(np.isfinite(V[:, 4]), V[:, 4] / 1000.0, (V[:, 2] + 0.5) * c["voxel_m"])
    return V, h


def clusters(V: np.ndarray, h: np.ndarray, vm: float, gapk: float) -> list[dict]:
    cols = collections.defaultdict(list)
    for (ix, iy, _iz, hits, _mm), hh in zip(V, h):
        if hh >= BREAK_LO:
            cols[(int(ix), int(iy))].append((hh, hits))
    seeds = {k for k, v in cols.items() if any(BREAK_LO <= hh < BREAK_HI for hh, _ in v)}
    seen, out = set(), []
    for k in seeds:
        if k in seen:
            continue
        stack, comp = [k], []
        seen.add(k)
        while stack:
            a = stack.pop()
            comp.append(a)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    b = (a[0] + dx, a[1] + dy)
                    if b in seeds and b not in seen:
                        seen.add(b)
                        stack.append(b)
        xs = np.array([(x + 0.5) * vm for x, _ in comp])
        ys = np.array([(y + 0.5) * vm for _, y in comp])
        cx, cy = float(xs.mean()), float(ys.mean())
        rng = math.hypot(cx, cy)
        ext = max(xs.max() - xs.min(), ys.max() - ys.min()) + vm
        gap = max(GAP_MIN, gapk * rng)
        dilated = {(x + dx, y + dy) for x, y in comp for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
        lo = min(hh for kk in comp for hh, _ in cols[kk] if BREAK_LO <= hh < BREAK_HI)
        top = lo
        for hh in sorted(hh for kk in dilated if kk in cols for hh, _ in cols[kk]):
            if hh < lo:
                continue
            if hh > top + gap:
                break
            top = max(top, hh)
        out.append(dict(cx=cx, cy=cy, rng=rng, ext=float(ext), top=top, ncols=len(comp),
                        hits=float(sum(ht for kk in comp for _, ht in cols[kk]))))
    return out


def is_small(cl: dict) -> bool:
    return cl["top"] < SMALL_TOP and cl["ext"] <= SMALL_EXT


def to_world(anchor, x: float, y: float) -> tuple[float, float]:
    ax, ay, ayaw = anchor
    return math.cos(ayaw) * x - math.sin(ayaw) * y + ax, math.sin(ayaw) * x + math.cos(ayaw) * y + ay


def objects_at(rec: dict, lay: dict, movable: list) -> list[tuple]:
    q = rec["qpos"]
    return [(o["kind"], q[lay[o["name"]]], q[lay[o["name"]] + 1], o["extent"]) for o in movable]


def label(wx: float, wy: float, objs: list, half: float, furniture: list) -> str:
    d, kind = min((math.hypot(wx - ox, wy - oy) - e, k) for k, ox, oy, e in objs)
    if d < 0.08:
        return kind
    if half - max(abs(wx), abs(wy)) < 0.12:
        return "wall"
    for o in furniture:
        if math.hypot(wx - o["x"], wy - o["y"]) < o["extent"] + 0.12:
            return o["kind"]
    return "other"


def run_seconds(path: str) -> float:
    with open(path, "rb") as fh:
        fh.seek(max(0, os.path.getsize(path) - 200000))
        tail = fh.read().decode(errors="ignore")
    ts = [json.loads(line)["t"] for line in tail.splitlines() if line.startswith('{"t":')]
    return ts[-1] if ts else float("nan")


# ------------------------------------------------------------------------------------------------ rules

def cmd_rules(paths: list[str], gapk: float) -> None:
    half, lay, movable, furniture = load_scene()
    rows, seconds = [], {}
    for p in paths:
        n = 0
        for line in open(p):
            if '"cloudv"' not in line or not line.startswith("{"):
                continue
            rec = json.loads(line)
            c = rec["cloudv"]
            if not c.get("vox"):
                continue
            V, h = mean_heights(c)
            objs = objects_at(rec, lay, movable)
            for cl in clusters(V, h, c["voxel_m"], gapk):
                wx, wy = to_world(c["anchor"], cl["cx"], cl["cy"])
                rows.append(dict(cl, lab=label(wx, wy, objs, half, furniture), run=p, t=rec["t"]))
            n += 1
        seconds[p] = run_seconds(p)
        print(f"{os.path.basename(p)}: {n} clouds, run {seconds[p]:.0f} s")
    print(f"\n{len(rows)} break-band clusters; gap max({GAP_MIN * 100:.0f} cm, {gapk} x range)")
    print(f"{'label':>8} {'n':>4} | stack top p10/p50/p90 cm | footprint p50/p90 cm | range p10/p50/p90 m | hits p50")
    for lab, n in collections.Counter(r["lab"] for r in rows).most_common():
        s = [r for r in rows if r["lab"] == lab]
        pct = lambda k, ps, scale: "/".join(f"{np.percentile([x[k] for x in s], p) * scale:.0f}" for p in ps)
        rng = "/".join(f"{np.percentile([x['rng'] for x in s], p):.2f}" for p in (10, 50, 90))
        print(f"{lab:>8} {n:4d} | {pct('top', (10, 50, 90), 100):>24} | {pct('ext', (50, 90), 100):>19} | {rng:>19} | "
              f"{np.median([x['hits'] for x in s]):.0f}")
    real = lambda r: r["lab"] in ("ball", "block")
    n_real = sum(real(r) for r in rows)
    print(f"\n{'rule':44} flagged  precision  recall")
    ladder = [("break band alone (the object EPM's input)", lambda r: True),
              ("stack top < 12 cm", lambda r: r["top"] < 0.12),
              ("stack top < 16 cm", lambda r: r["top"] < 0.16),
              ("stack top < 20 cm", lambda r: r["top"] < 0.20),
              ("footprint <= 20 cm", lambda r: r["ext"] <= 0.20),
              ("SMALL: top < 16 cm and footprint <= 20 cm", is_small)]
    for name, rule in ladder:
        flagged = [r for r in rows if rule(r)]
        tp = sum(real(r) for r in flagged)
        print(f"{name:44} {len(flagged):7d}  {tp / max(1, len(flagged)):9.2f}  {tp / max(1, n_real):6.2f}")
    flagged = [r for r in rows if is_small(r)]
    print("false positives of SMALL by label:", dict(collections.Counter(r["lab"] for r in flagged if not real(r))))
    stops = {(r["run"], r["t"]) for r in flagged if real(r)}
    print(f"clouds holding a flagged real object: {len(stops)}, {len(stops) / (sum(seconds.values()) / 60):.2f} per minute of run")


# ------------------------------------------------------------------------------------------------ arms

def score_log(path: str, gapk: float) -> dict:
    half, lay, movable, furniture = load_scene()
    res = dict(clouds=0, vox=[], brk=[], flagged=0, flagged_real=0, near=0, found=0, hits=[], ball_near=0,
               ball_found=0, field_near=0, field_found=0, wide_near=0, wide_found=0, stop_ticks=0, moving=0,
               yaw_span=[], open_ticks=0, seconds=0.0, obst=0, obst_small=0, speeds=[])
    prev, yaws = None, []
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        rec = json.loads(line)
        res["seconds"] = rec["t"]
        if rec.get("stop", 0) and "q" in rec:
            # the head's JOINTS, not its command: a babble step changes the command on one tick and the head then
            # travels for ~10.  Moving = faster than 0.05 rad/s on yaw or pitch.
            y, p = rec["q"][HEAD_YAW_Q], rec["q"][HEAD_PITCH_Q]
            res["stop_ticks"] += 1
            res["open_ticks"] += "cld" in rec
            if prev is not None:
                res["speeds"].append(50.0 * math.hypot(y - prev[0], p - prev[1]))
                if abs(y - prev[0]) > 1e-3 or abs(p - prev[1]) > 1e-3:
                    res["moving"] += 1
            prev = (y, p)
            yaws.append(y)
        else:
            if len(yaws) > 50:
                res["yaw_span"].append(float(np.percentile(yaws, 95) - np.percentile(yaws, 5)))
            prev, yaws = None, []
        c = rec.get("cloudv")
        if not c or not c.get("vox"):
            continue
        V, h = mean_heights(c)
        vm = c["voxel_m"]
        ax, ay, ayaw = c["anchor"]
        ca, sa = math.cos(ayaw), math.sin(ayaw)
        res["clouds"] += 1
        res["vox"].append(len(V))
        res["brk"].append(int(((h >= BREAK_LO) & (h < BREAK_HI)).sum()))
        objs = objects_at(rec, lay, movable)
        for cl in clusters(V, h, vm, gapk):
            wx, wy = to_world(c["anchor"], cl["cx"], cl["cy"])
            lab = label(wx, wy, objs, half, furniture)
            if lab in ("wall", "chair", "table", "shelf"):       # obstacles: a good cloud reads them as TALL
                res["obst"] += 1
                res["obst_small"] += is_small(cl)
            if is_small(cl):
                res["flagged"] += 1
                res["flagged_real"] += lab in ("ball", "block")
        off = h >= BREAK_LO
        bx, by = (V[off, 0] + 0.5) * vm, (V[off, 1] + 0.5) * vm
        wx, wy, hits = ca * bx - sa * by + ax, sa * bx + ca * by + ay, V[off, 3]
        for kind, ox, oy, e in objs:
            if math.hypot(ox - ax, oy - ay) > NEAR_M:
                continue
            bearing = abs(math.atan2(-sa * (ox - ax) + ca * (oy - ay), ca * (ox - ax) + sa * (oy - ay)))
            on = np.hypot(wx - ox, wy - oy) < e + 0.04
            found = bool(on.any())
            res["near"] += 1
            res["found"] += found
            if found:
                res["hits"].append(float(hits[on].sum()))
            if kind == "ball":
                res["ball_near"] += 1
                res["ball_found"] += found
            key = "field" if bearing <= BABBLE_FIELD else "wide"
            res[f"{key}_near"] += 1
            res[f"{key}_found"] += found
    return res


def cmd_arms(specs: list[str], gapk: float) -> None:
    ms = lambda xs: (f"{statistics.mean(xs):.2f} ± {statistics.stdev(xs):.2f}" if len(xs) > 1
                     else f"{xs[0]:.2f}" if xs else "-")
    frac = lambda a, b: f"{a}/{b} {100 * a / b:3.0f}%" if b else f"{a}/{b}    -"
    print(f"{'arm':>12} {'seed':>4} | clouds vox/cl brk/cl | precision | objs<2m found | in babble field | outside it |"
          f" balls | obj hits p50 | head moving % | yaw span p50 | cloud open %")
    for spec in specs:
        name, pattern = spec.split("=", 1)
        per = []
        for path in sorted(glob.glob(pattern)):
            r = score_log(path, gapk)
            per.append(r)
            tail = os.path.basename(path).rsplit("_s", 1)[-1].split(".")[0]
            print(f"{name:>12} {tail if tail.isdigit() else '-':>4} | {r['clouds']:6d} {np.mean(r['vox']) if r['vox'] else 0:6.0f} "
                  f"{np.mean(r['brk']) if r['brk'] else 0:6.0f} | {r['flagged_real'] / max(1, r['flagged']):9.2f} | "
                  f"{frac(r['found'], r['near']):>13} | {frac(r['field_found'], r['field_near']):>15} | "
                  f"{frac(r['wide_found'], r['wide_near']):>10} | {frac(r['ball_found'], r['ball_near']):>5} | "
                  f"{np.median(r['hits']) if r['hits'] else 0:12.0f} | {100 * r['moving'] / max(1, r['stop_ticks']):13.0f} | "
                  f"{np.median(r['yaw_span']) if r['yaw_span'] else 0:12.2f} | {100 * r['open_ticks'] / max(1, r['stop_ticks']):11.0f}")
        if not per:
            print(f"{name:>12}: no logs match {pattern}")
            continue
        tot = lambda k: sum(r[k] for r in per)
        print(f"{name:>12} POOL | stop minutes/run {ms([r['stop_ticks'] / 3000 for r in per])}, clouds/run "
              f"{ms([r['clouds'] for r in per])}, voxels/cloud {ms([float(np.mean(r['vox'])) for r in per if r['vox']])}, "
              f"break voxels/cloud {ms([float(np.mean(r['brk'])) for r in per if r['brk']])}")
        print(f"{'':>12}      | objects within {NEAR_M:.0f} m found {tot('found')}/{tot('near')} "
              f"(per seed {ms([r['found'] / r['near'] for r in per if r['near']])}); in the babble's field "
              f"{tot('field_found')}/{tot('field_near')}; outside it {tot('wide_found')}/{tot('wide_near')}; balls "
              f"{tot('ball_found')}/{tot('ball_near')}")
        print(f"{'':>12}      | SMALL clusters {tot('flagged')}, real {tot('flagged_real')} (precision "
              f"{tot('flagged_real') / max(1, tot('flagged')):.2f}); real per minute of run "
              f"{ms([60 * r['flagged_real'] / r['seconds'] for r in per if r['seconds']])}")
        speeds = np.concatenate([np.array(r["speeds"]) for r in per if r["speeds"]] or [np.zeros(1)])
        moving = speeds[speeds > 0.05]
        print(f"{'':>12}      | obstacle clusters (walls, chairs, table, shelf) misread as SMALL: {tot('obst_small')}/{tot('obst')} "
              f"({100 * tot('obst_small') / max(1, tot('obst')):.1f} %); head speed while moving p50/p90 "
              f"{np.percentile(moving, 50) if moving.size else 0:.2f}/{np.percentile(moving, 90) if moving.size else 0:.2f} rad/s")
        print(f"{'':>12}      | head moving {ms([100 * r['moving'] / max(1, r['stop_ticks']) for r in per])} % of stop ticks; "
              f"yaw span p50 {ms([float(np.median(r['yaw_span'])) for r in per if r['yaw_span']])} rad; cloud open "
              f"{ms([100 * r['open_ticks'] / max(1, r['stop_ticks']) for r in per])} % of stop ticks")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("rules", help="the stack rule's ladder over one or more full host logs")
    r.add_argument("logs", nargs="+")
    r.add_argument("--gapk", type=float, default=0.12, help="stack gap per metre of range (0 = a fixed 10 cm gap)")
    a = sub.add_parser("arms", help="per-arm cloud and object scores, NAME=GLOB per arm")
    a.add_argument("arms", nargs="+")
    a.add_argument("--gapk", type=float, default=0.12)
    args = ap.parse_args()
    if args.cmd == "rules":
        cmd_rules(args.logs, args.gapk)
    else:
        cmd_arms(args.arms, args.gapk)


if __name__ == "__main__":
    main()
