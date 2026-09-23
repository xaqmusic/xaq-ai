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
  cloud_objects.py seek LOG...
      the things phase's T2: every SEEK EPISODE (a held target, from the stop that set it to the tick its need returns
      to 0) with what the target really was (the last attended thing of that stop, labelled by the manifest), the body's
      closest approach to the target's true position, whether an object was touched, and how the episode ended
  cloud_objects.py heading LOG...
      the circling (2026-09-17): on walking ticks, by which loop holds the reference (play / seek), the heading error's
      median and the share under 0.3 rad, the REFERENCE's own motion (rad/s; a reference that turns at the body's rate
      is a target inside the turning radius), the yaw command on its rail, and the heading reflex's share
  cloud_objects.py stops LOG...
      the things phase's T4: every STOP by how it started (the timer, or the seek loop's arrival), how far the nearest
      object was when it began, and whether the stop's cloud attended a real object and at what range -- did a reached
      thing get looked at
  cloud_objects.py where LOG...
      the operator's "interesting" scale (2026-09-22): every STOP by WHERE the body is when it starts -- at a small
      thing (within 0.6 m of a movable object's edge), at a wall (within 0.35 m of one), or on open floor -- with
      the seconds of stop spent in each, per seed.  A walk that stands at walls is boring; one that stands at
      things is not.  Reads the full log (x, y, qpos, stop).
  cloud_objects.py spins LOG...
      the operator's "frustration" (2026-09-23): 20 s windows of walking (from 600 s, under 20 % stop) in which the body
      turned more than a full turn while its net displacement stayed under 0.5 m -- the spin near the blocks at
      750-850 s of R72 seed 1.  Per log: the count, and the seconds spent spinning.  The host's --skill-on-spin uses
      the same rule on the body's own odometry.
  cloud_objects.py skills LOG...
      every SKILL fired (kick / peck / push, from the record's `skill` field) against the manifest: the nearest movable
      thing at its start and that thing's true displacement 8 s later; whether the outcome loop observed an answer
      within 36 s; and at the look stop after the unwind, the thing's range and bearing off the nose.  Splits the
      unknown answers into "rolled beyond the match radius", "moved a little", "stayed", and "no thing within 0.5 m"
      (a wall base or a leg the cloud attended as a thing, or a dead-reckoned arrival at a place the thing is not).
  cloud_objects.py things LOG...
      the things phase's T1 (microduck_things_phase.md): the MODULE's stack rule against this file's, cluster by
      cluster on the same filed clouds (faithfulness); what the ATTENDED thing really was, per tick ("thg", labelled
      with the anchor the next "cloudv" carries); and the thing EPM's nodes by object kind ("tepm"): purity, and how
      many poses each real object was attended from under how many winners (the pose-invariance reading O40 could
      not make)

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
WALL_T = 0.025                                                 # playroom_gen.py: a wall's half-thickness (inner face at half - WALL_T)
WALL_REACH = 2.4                                               # m: CloudMap keeps returns to max_range 2.5
WALL_BEARINGS = np.radians(np.arange(-64, 65, 2))              # across the cloud view's +-64 deg
WALL_BEARING_TOL, WALL_RANGE_TOL = math.radians(2.0), 0.12


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
               yaw_span=[], open_ticks=0, seconds=0.0, obst=0, obst_small=0, speeds=[], wall_expected=0,
               wall_seen=0, wall_seen_per_cloud=[])
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
        # WALLS LOCATED: of the bearings across the view (every 2 deg over +-64) whose room wall lies within WALL_REACH of
        # the anchor, the share where the cloud holds an off-floor voxel within 2 deg and 12 cm of that wall's true range.
        # "An idea of where the walls are": furniture standing in front of a wall hides it the same in every arm.
        ob = np.arctan2((V[off, 1] + 0.5) * vm, (V[off, 0] + 0.5) * vm)
        orng = np.hypot((V[off, 0] + 0.5) * vm, (V[off, 1] + 0.5) * vm)
        inner = half - WALL_T
        expected = seen = 0
        for b in WALL_BEARINGS:
            dx, dy = math.cos(ayaw + b), math.sin(ayaw + b)
            tx = (math.copysign(inner, dx) - ax) / dx if abs(dx) > 1e-9 else math.inf
            ty = (math.copysign(inner, dy) - ay) / dy if abs(dy) > 1e-9 else math.inf
            dist = min(tx, ty)
            if not 0.0 < dist <= WALL_REACH:
                continue
            expected += 1
            dbear = np.abs(np.angle(np.exp(1j * (ob - b))))
            seen += bool(np.any((dbear <= WALL_BEARING_TOL) & (np.abs(orng - dist) <= WALL_RANGE_TOL)))
        res["wall_expected"] += expected
        res["wall_seen"] += seen
        if expected:
            res["wall_seen_per_cloud"].append(seen / expected)
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
        print(f"{'':>12}      | walls located: {tot('wall_seen')}/{tot('wall_expected')} bearings within {WALL_REACH} m "
              f"({100 * tot('wall_seen') / max(1, tot('wall_expected')):.0f} %); per seed "
              f"{ms([100 * r['wall_seen'] / r['wall_expected'] for r in per if r['wall_expected']])} %; per cloud p25/p50 "
              + "/".join(f"{100 * np.percentile(sum((r['wall_seen_per_cloud'] for r in per), []), q):.0f}" for q in (25, 50)) + " %")
        print(f"{'':>12}      | head moving {ms([100 * r['moving'] / max(1, r['stop_ticks']) for r in per])} % of stop ticks; "
              f"yaw span p50 {ms([float(np.median(r['yaw_span'])) for r in per if r['yaw_span']])} rad; cloud open "
              f"{ms([100 * r['open_ticks'] / max(1, r['stop_ticks']) for r in per])} % of stop ticks")


# ------------------------------------------------------------------------------------------------ things

def cmd_things(paths: list[str], gapk: float) -> None:
    half, lay, movable, furniture = load_scene()
    faithful = dict(clouds=0, n_mod=0, n_py=0, matched=0, small_agree=0, small_mod=0, small_py=0, top_dev=[], ext_dev=[])
    att_rows = []                       # one per attended tick, labelled
    tick_rows = dict(ticks=0, attended=0, small_seen=0)
    for p in paths:
        pending = []                    # "thg" ticks waiting for their cloud's anchor
        for line in open(p):
            if not line.startswith('{"t"'):
                continue
            if '"thg"' not in line and '"cloudv"' not in line:
                continue
            rec = json.loads(line)
            th = rec.get("thg")
            if th is not None:
                tick_rows["ticks"] += 1
                tick_rows["small_seen"] += th[1] > 0
                if th[2] >= 0:
                    tick_rows["attended"] += 1
                    pending.append((rec["t"], th, rec.get("tepm")))
            c = rec.get("cloudv")
            if not c:
                continue
            objs = objects_at(rec, lay, movable)
            names = [o["name"] for o in movable]
            for t, th, tepm in pending:
                wx, wy = to_world(c["anchor"], th[3], th[4])
                lab = label(wx, wy, objs, half, furniture)
                # which object, by name, for the pose count
                d, which = min((math.hypot(wx - ox, wy - oy) - e, nm) for (k, ox, oy, e), nm in zip(objs, names))
                att_rows.append(dict(run=p, t=t, lab=lab, obj=which if d < 0.08 else None, rng=th[5], ext=th[6], top=th[7],
                                     ncols=th[8], hits=th[9], chain=th[10], winner=tepm[0] if tepm else None,
                                     tle=tepm[1] if tepm else None, anchor=tuple(round(a, 2) for a in c["anchor"])))
            pending = []
            # FAITHFULNESS: the module's clusters of this filed cloud against this file's rule on the same voxels
            mod = rec.get("things")
            if mod is None or not c.get("vox"):
                continue
            V, h = mean_heights(c)
            py = clusters(V, h, c["voxel_m"], gapk)
            faithful["clouds"] += 1
            faithful["n_mod"] += len(mod)
            faithful["n_py"] += len(py)
            faithful["small_mod"] += sum(m[8] for m in mod)
            faithful["small_py"] += sum(is_small(q) for q in py)
            used = set()
            for m in mod:
                best = min(((math.hypot(m[0] - q["cx"], m[1] - q["cy"]), i) for i, q in enumerate(py) if i not in used),
                           default=(9.0, -1))
                if best[0] < 0.03 and best[1] >= 0:
                    used.add(best[1])
                    q = py[best[1]]
                    faithful["matched"] += 1
                    faithful["small_agree"] += bool(m[8]) == is_small(q)
                    faithful["top_dev"].append(abs(m[4] - q["top"]))
                    faithful["ext_dev"].append(abs(m[3] - q["ext"]))
    f = faithful
    print(f"FAITHFULNESS over {f['clouds']} filed clouds: module clusters {f['n_mod']}, offline rule {f['n_py']}, matched by "
          f"centroid (< 3 cm) {f['matched']}; SMALL verdict agrees on {f['small_agree']}/{f['matched']} "
          f"(module small {f['small_mod']}, offline small {f['small_py']}); |top| dev p50/p90 "
          f"{100 * np.percentile(f['top_dev'], 50) if f['top_dev'] else 0:.1f}/{100 * np.percentile(f['top_dev'], 90) if f['top_dev'] else 0:.1f} cm, "
          f"|footprint| dev p90 {100 * np.percentile(f['ext_dev'], 90) if f['ext_dev'] else 0:.1f} cm")
    tr = tick_rows
    print(f"\nATTENTION: {tr['ticks']} thing ticks; a small cluster in view on {tr['small_seen']} ({100 * tr['small_seen'] / max(1, tr['ticks']):.0f} %); "
          f"attended on {tr['attended']}; labelled {len(att_rows)}")
    real = lambda r: r["lab"] in ("ball", "block")
    if att_rows:
        n_real = sum(real(r) for r in att_rows)
        print(f"  the attended thing was a real object on {n_real}/{len(att_rows)} ticks (precision {n_real / len(att_rows):.2f}); by label:",
              dict(collections.Counter(r["lab"] for r in att_rows).most_common()))
        print(f"  {'label':>8} {'n':>5} | range p10/50/90 m | columns p50 | hits/col p50 | top p50 cm | chain p50")
        for lab, n in collections.Counter(r["lab"] for r in att_rows).most_common():
            rs = [r for r in att_rows if r["lab"] == lab]
            rng = "/".join(f"{np.percentile([x['rng'] for x in rs], q):.2f}" for q in (10, 50, 90))
            print(f"  {lab:>8} {n:5d} | {rng:>17} | {np.median([x['ncols'] for x in rs]):11.0f} | "
                  f"{np.median([x['hits'] / max(1, x['ncols']) for x in rs]):12.1f} | {100 * np.median([x['top'] for x in rs]):10.0f} | "
                  f"{np.median([x['chain'] for x in rs]):9.0f}")
        # THE THING EPM: node purity by label, and poses per object under how many winners
        wr = [r for r in att_rows if r["winner"] is not None and r["winner"] >= 0]
        if wr:
            # Node ids are PER RUN (each seed grows its own EPM), so purity is scored per run and averaged;
            # pooling ids across runs mixed unrelated nodes and read 0.56 where the runs themselves read 0.87.
            per = []
            for run in sorted({r["run"] for r in wr}):
                rs = [r for r in wr if r["run"] == run]
                by_node = collections.defaultdict(collections.Counter)
                for r in rs:
                    by_node[r["winner"]][r["lab"]] += 1
                pure = sum(max(c.values()) for c in by_node.values()) / len(rs)
                chance = max(collections.Counter(r["lab"] for r in rs).values()) / len(rs)
                per.append((run, len(by_node), len(rs), pure, chance))
            print(f"\nTHING EPM, per run (nodes seen, attended ticks, majority-label purity, chance = largest label's share):")
            for run, nn, n, pure, chance in per:
                print(f"  {os.path.basename(run):>28}: {nn:3d} nodes {n:5d} ticks  purity {pure:.2f}  chance {chance:.2f}  (+{pure - chance:.2f})")
            print(f"  mean over runs: nodes {statistics.mean(x[1] for x in per):.1f}, purity {statistics.mean(x[3] for x in per):.2f} "
                  f"± {statistics.stdev(x[3] for x in per) if len(per) > 1 else 0:.2f}, chance {statistics.mean(x[4] for x in per):.2f}, "
                  f"purity - chance {statistics.mean(x[3] - x[4] for x in per):+.2f} ± {statistics.stdev(x[3] - x[4] for x in per) if len(per) > 1 else 0:.2f}")
            run0 = per[0][0]
            by_node = collections.defaultdict(collections.Counter)
            for r in wr:
                if r["run"] == run0:
                    by_node[r["winner"]][r["lab"]] += 1
            print(f"  the first run's nodes:")
            for node, cnt in sorted(by_node.items(), key=lambda kv: -sum(kv[1].values()))[:8]:
                print(f"    node {node:3d}: {sum(cnt.values()):5d} ticks  {dict(cnt.most_common(3))}")
            by_obj = collections.defaultdict(lambda: dict(poses=set(), winners=collections.Counter()))
            for r in wr:
                if r["obj"]:
                    by_obj[(r["run"], r["obj"])]["poses"].add(r["anchor"])
                    by_obj[(r["run"], r["obj"])]["winners"][r["winner"]] += 1
            if by_obj:
                shares = collections.defaultdict(list)
                for (run, nm), d in by_obj.items():
                    shares[nm].append(max(d["winners"].values()) / sum(d["winners"].values()))
                print("  per real object, over runs: poses attended from (mean), modal winner's share (mean +- sd) -- pose invariance")
                for nm in sorted(shares):
                    poses = [len(d["poses"]) for (run, n2), d in by_obj.items() if n2 == nm]
                    print(f"    {nm:>10}: {statistics.mean(poses):4.1f} poses, modal share {statistics.mean(shares[nm]):.2f} "
                          f"± {statistics.stdev(shares[nm]) if len(shares[nm]) > 1 else 0:.2f} over {len(shares[nm])} runs")
            tles = [r["tle"] for r in wr if r["tle"] is not None]
            if tles:
                print(f"  TLE at the attended thing p10/50/90 {np.percentile(tles, 10):.3f}/{np.percentile(tles, 50):.3f}/{np.percentile(tles, 90):.3f}")


# ------------------------------------------------------------------------------------------------ seek

def seek_episodes(path: str) -> tuple[list, float]:
    """Seek episodes of one run: from the tick a target is first held to the tick its need returns to 0.  The target
    is the LAST attended thing of the stop that set it (its world position through the next cloudv's anchor)."""
    half, lay, movable, furniture = load_scene()
    eps, cur, pending, last_att = [], None, [], None
    obj0 = objlast = None
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        t, q = r["t"], r["qpos"]
        objs = objects_at(r, lay, movable)
        if t >= 700:
            if obj0 is None:
                obj0 = [(ox, oy) for _, ox, oy, _ in objs]
            objlast = [(ox, oy) for _, ox, oy, _ in objs]
        th = r.get("thg")
        if th is not None and th[2] >= 0:
            pending.append((t, th))
        c = r.get("cloudv")
        if c and pending:
            t0, th0 = pending[-1]
            wx, wy = to_world(c["anchor"], th0[3], th0[4])
            last_att = dict(t=t0, wx=wx, wy=wy, lab=label(wx, wy, objs, half, furniture), rng=th0[5])
            pending = []
        sk = r.get("seek")
        if not sk:
            continue
        held = sk[0] > 0
        if held and cur is None:
            cur = dict(t0=t, target=None, dmin=9.0, touched=0, walls=0, steer3=0, ticks=0, end=None)
        if cur is not None:
            if r.get("stop", 0) == 0 and cur["target"] is None and last_att is not None:
                cur["target"] = last_att
            cur["ticks"] += 1
            cur["steer3"] += r.get("steer") == 3
            cur["touched"] += r.get("obj", 0)
            cur["walls"] += r.get("wall", 0)
            if cur["target"]:
                cur["dmin"] = min(cur["dmin"], math.hypot(q[0] - cur["target"]["wx"], q[1] - cur["target"]["wy"]))
            if not held:
                cur["end"] = "arrive" if sk[1] < 0.3 else "forget"
                cur["t1"] = t
                eps.append(cur)
                cur = None
    moved = sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(obj0, objlast)) if obj0 and objlast else 0.0
    return eps, moved


def cmd_seek(paths: list[str]) -> None:
    tot, dmins, ends, touched = collections.Counter(), collections.defaultdict(list), collections.Counter(), collections.Counter()
    for path in paths:
        eps, moved = seek_episodes(path)
        labs = collections.Counter((e["target"] or {}).get("lab", "none") for e in eps)
        print(f"{os.path.basename(path)}: {len(eps)} episodes, targets {dict(labs)}, objects moved over the control phase {moved:.2f} m")
        for e in eps:
            lab = (e["target"] or {}).get("lab", "none")
            tot[lab] += 1; dmins[lab].append(e["dmin"]); ends[e["end"]] += 1
            touched[lab] += e["touched"] > 0
            print(f"   t {e['t0']:6.1f}-{e.get('t1', 0):6.1f} target {lab:>6} at {(e['target'] or {}).get('rng', 0):.2f} m: closest "
                  f"{e['dmin']:.2f} m, seek won {100 * e['steer3'] / max(1, e['ticks']):3.0f} % of {e['ticks'] / 50:5.1f} s, "
                  f"touched an object on {e['touched']} ticks, wall ticks {e['walls']}, end {e['end']}")
    print(f"\nEPISODES by target: {dict(tot)}; ends: {dict(ends)}")
    for lab in tot:
        print(f"  {lab:>6}: closest approach p50 {statistics.median(dmins[lab]):.2f} m, within 0.4 m on "
              f"{sum(d < 0.4 for d in dmins[lab])}/{len(dmins[lab])}, an object touched during {touched[lab]}")


# ------------------------------------------------------------------------------------------------ heading

def cmd_heading(paths: list[str], control_from: float = 700.0) -> None:
    err = {1: [], 3: []}; refmove = {1: [], 3: []}; rail = {1: [], 3: []}; share = []; straight_w = []
    agree_n = agree_yes = clamp_n = 0     # the body turns the way PLAY asks (2026-09-19): sign of the requested turn vs the heading's change over 1 s
    for p in paths:
        prev = None; xs = []; ys = []; hist = []
        for line in open(p):
            if not line.startswith('{"t"'):
                continue
            r = json.loads(line)
            if r["t"] < control_from or r.get("stop", 0):
                prev = None; hist = []
                if len(xs) > 500:
                    path = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
                    straight_w.append(math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) / max(path, 1e-6))
                xs, ys = [], []
                continue
            xs.append(r["x"]); ys.append(r["y"])
            if "pb" in r:
                hist.append((r.get("steer", 0), math.atan2(r["pb"][0], r["pb"][1]), r["hdg"][0]))
                if len(hist) > 50:
                    st0, ang0, h0 = hist[-51]
                    if st0 == 1 and abs(ang0) > 0.6:
                        agree_n += 1; agree_yes += (-ang0 > 0) == (r["hdg"][0] - h0 > 0)
                    if abs(abs(ang0) - 2.89) < 0.02: clamp_n += 1
            st = r.get("steer", 0)
            if "hr" in r:
                share.append(r["hr"])
            if st not in err:
                prev = None
                continue
            h, ref = r["hdg"]
            err[st].append(abs(math.atan2(math.sin(h - ref), math.cos(h - ref))))
            rail[st].append(abs(r["twist"][2]) > 0.9)
            if prev is not None and prev[0] == st:
                refmove[st].append(abs(ref - prev[1]) * 50.0)
            prev = (st, ref)
    print(f"{len(paths)} runs, walking ticks from {control_from:.0f} s")
    for st, name in ((1, "play"), (3, "seek")):
        e = np.array(err[st]); m = np.array(refmove[st]); rl = np.array(rail[st])
        if not len(e):
            print(f"  {name}: no ticks"); continue
        print(f"  {name:>4}: {len(e):6d} ticks; |heading error| median {np.median(e):.2f} rad, under 0.3 rad on {100 * np.mean(e < 0.3):3.0f} %; "
              f"the reference moves {np.median(m) if len(m) else 0:.2f} rad/s (median), faster than 0.5 rad/s on {100 * np.mean(m > 0.5) if len(m) else 0:3.0f} %; "
              f"|vyaw| > 0.9 on {100 * rl.mean():3.0f} %")
    if share:
        sh = np.array(share)
        print(f"  heading reflex share: mean {sh.mean():.2f}; owns the yaw (1.0) on {100 * np.mean(sh > 0.99):.0f} % of walking ticks")
    if straight_w:
        print(f"  straightness per walk (net / path, walks over 10 s): p25/p50/p75 {'/'.join(f'{np.percentile(straight_w, q):.2f}' for q in (25, 50, 75))} over {len(straight_w)} walks")
    if agree_n:
        print(f"  play asks a turn of more than 0.6 rad on {agree_n} ticks; the body turns THAT way over the next second on {100 * agree_yes / agree_n:.0f} % of them; "
              f"the bearing sits at the loop's committed-turn clamp (0.92 pi) on {clamp_n} ticks")


# ------------------------------------------------------------------------------------------------ stops

def cmd_spins(paths: list[str], win_s: float = 20.0, turns: float = 1.0, net_m: float = 0.5) -> None:
    """Windows of walking in which the body turned more than `turns` full turns without moving `net_m`."""

    def yaw_of(q):
        w, x, y, z = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    for path in paths:
        T, X, Y, H, S = [], [], [], [], []
        for line in open(path):
            if not line.startswith('{"t":'):
                continue
            r = json.loads(line)
            T.append(r["t"]); X.append(r["x"]); Y.append(r["y"]); H.append(yaw_of(r["qpos"][3:7])); S.append(1 if r.get("stop", 0) else 0)
        if len(T) < 3:
            continue
        dt = T[1] - T[0]
        W = max(2, int(win_s / dt))
        Hu = np.unwrap(np.array(H)); X = np.array(X); Y = np.array(Y); S = np.array(S); T = np.array(T)
        n = 0; secs = 0.0; first = None
        for i in range(0, len(T) - W, W // 2):
            if T[i] < 600 or S[i:i + W].mean() > 0.2:
                continue
            if abs(Hu[i + W] - Hu[i]) > turns * 2 * math.pi and math.hypot(X[i + W] - X[i], Y[i + W] - Y[i]) < net_m:
                n += 1; secs += win_s / 2
                if first is None:
                    first = T[i]
        print(f"{Path(path).stem:34s} spin windows {n:3d}  spinning {secs:5.0f} s  first at {first if first is not None else '-'}")


def cmd_skills(paths: list[str]) -> None:
    """Every skill fired against the manifest: did it reach, did the loop see the answer, and where was the thing at the look."""
    half, lay, movable, _furniture = load_scene()

    def yaw_of(q):
        w, x, y, z = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    rows = []
    for path in paths:
        recs = [json.loads(l) for l in open(path) if l.startswith('{"t":')]
        prev, i = "", 0
        while i < len(recs):
            r = recs[i]
            sk = r.get("skill", "")
            if sk and sk != prev:
                x, y = r["x"], r["y"]
                d, kind, ox, oy = min((math.hypot(x - ox, y - oy) - e, k, ox, oy) for k, ox, oy, e in objects_at(r, lay, movable))
                j = i
                while j < len(recs) and recs[j]["t"] < r["t"] + 8:
                    j += 1
                later = [(ox2, oy2) for k2, ox2, oy2, _e in objects_at(recs[min(j, len(recs) - 1)], lay, movable)
                         if k2 == kind and math.hypot(ox2 - ox, oy2 - oy) < 1.5]
                disp = min(math.hypot(ox2 - ox, oy2 - oy) for ox2, oy2 in later) if later else float("nan")
                observed = any(recs[m].get("outc") and recs[m]["outc"][2] for m in range(i, min(len(recs), i + 1800)))
                # the look stop: the first stop after the skill's window (and the stop that hosted it)
                j = i
                while j < len(recs) and recs[j].get("skill", "") == sk:
                    j += 1
                while j < len(recs) and recs[j].get("stop", 0):
                    j += 1
                while j < len(recs) and not recs[j].get("stop", 0) and recs[j]["t"] < r["t"] + 12:
                    j += 1
                rng = brg = float("nan")
                if j < len(recs) and recs[j].get("stop", 0):
                    s = recs[j]
                    h = yaw_of(s["qpos"][3:7])
                    _dd, _kk, tx, ty = min((math.hypot(s["x"] - ox2, s["y"] - oy2) - e2, k2, ox2, oy2)
                                           for k2, ox2, oy2, e2 in objects_at(s, lay, movable) if k2 == kind)
                    rng = math.hypot(tx - s["x"], ty - s["y"])
                    brg = math.degrees((math.atan2(ty - s["y"], tx - s["x"]) - h + math.pi) % (2 * math.pi) - math.pi)
                rows.append(dict(log=Path(path).stem, t=r["t"], skill=sk, kind=kind, d0=d, disp=disp, observed=observed, rng=rng, brg=brg))
            prev = sk
            i += 1
    n = len(rows)
    if not n:
        print("no skills in these logs")
        return
    obs = [r for r in rows if r["observed"]]
    unk = [r for r in rows if not r["observed"]]
    far = [r for r in unk if r["d0"] > 0.5]
    rolled = [r for r in unk if r["d0"] <= 0.5 and r["disp"] > 0.6]
    little = [r for r in unk if r["d0"] <= 0.5 and 0.1 < r["disp"] <= 0.6]
    stayed = [r for r in unk if r["d0"] <= 0.5 and r["disp"] <= 0.1]
    by_skill = collections.Counter(r["skill"] for r in rows)
    print(f"skills {n} ({', '.join(f'{k} {v}' for k, v in sorted(by_skill.items()))}); moved their thing > 5 cm: {sum(1 for r in rows if r['disp'] > 0.05)}"
          f" ({', '.join(f'{k} {sum(1 for r in rows if r[chr(115)+chr(107)+chr(105)+chr(108)+chr(108)] == k and r[chr(100)+chr(105)+chr(115)+chr(112)] > 0.05)}/{v}' for k, v in sorted(by_skill.items()))})")
    print(f"answers observed by the loop {len(obs)}; unknown {len(unk)} = no thing within 0.5 m {len(far)} + rolled beyond 0.6 m {len(rolled)} + moved 0.1-0.6 m {len(little)} + stayed {len(stayed)}")
    for label, v in (("observed", obs), ("unknown", unk)):
        vv = [r for r in v if not math.isnan(r["rng"])]
        if vv:
            print(f"  at the look stop, {label} (n={len(vv)}): thing range median {statistics.median(r['rng'] for r in vv):.2f} m, |bearing| median {statistics.median(abs(r['brg']) for r in vv):.0f} deg, beyond 35 deg {sum(1 for r in vv if abs(r['brg']) > 35)}")
    print("  true displacement 8 s after a skill, median by skill: " + ", ".join(f"{k} {statistics.median(r['disp'] for r in rows if r['skill'] == k):.2f} m" for k in sorted(by_skill)))


def cmd_where(paths: list[str]) -> None:
    """Every stop by where the body stands when it starts: at a thing, at a wall, or on open floor."""
    half, lay, movable, _furniture = load_scene()

    def classify(rec: dict) -> str:
        x, y = rec["x"], rec["y"]
        d_thing = min(math.hypot(x - ox, y - oy) - e for _k, ox, oy, e in objects_at(rec, lay, movable))
        d_wall = half - max(abs(x), abs(y))
        if d_thing < 0.6:
            return "thing"
        if d_wall < 0.35:
            return "wall"
        return "open"

    keys = ("thing", "wall", "open")
    per: list[tuple[str, dict, dict]] = []
    for path in paths:
        cnt = {k: 0 for k in keys}
        secs = {k: 0.0 for k in keys}
        prev, cls, t_prev = 0, None, None
        with open(path) as fh:
            for line in fh:
                if '"stop":' not in line or '"qpos"' not in line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                s = rec.get("stop", 0)
                if s and not prev:
                    cls = classify(rec)
                    cnt[cls] += 1
                if s and cls and t_prev is not None:
                    secs[cls] += rec["t"] - t_prev
                prev, t_prev = s, rec["t"]
        per.append((Path(path).stem, cnt, secs))
    n = len(per) or 1
    tot = {k: sum(c[k] for _s, c, _t in per) / n for k in keys}
    tsec = {k: sum(t[k] for _s, _c, t in per) / n for k in keys}
    print(f"{'log':34s} stops   thing  wall  open | stop-seconds  thing  wall  open")
    for stem, c, t in per:
        print(f"{stem:34s} {sum(c.values()):5d}   {c['thing']:5d} {c['wall']:5d} {c['open']:5d} | {t['thing']:12.0f} {t['wall']:5.0f} {t['open']:5.0f}")
    print(f"{'mean per seed':34s} {sum(tot.values()):5.1f}   {tot['thing']:5.1f} {tot['wall']:5.1f} {tot['open']:5.1f} | {tsec['thing']:12.0f} {tsec['wall']:5.0f} {tsec['open']:5.0f}")


def cmd_stops(paths: list[str]) -> None:
    half, lay, movable, furniture = load_scene()
    rows = []
    for path in paths:
        cur = None; pending = []
        for line in open(path):
            if not line.startswith('{"t"'):
                continue
            r = json.loads(line)
            ev = r.get("event") or ""
            if ev in ("stop:start", "stop:arrive"):
                q = r["qpos"]; objs = objects_at(r, lay, movable)
                dn, kind = min((math.hypot(q[0] - ox, q[1] - oy), k) for k, ox, oy, e in objs)
                cur = dict(run=os.path.basename(path), t=r["t"], how=ev.split(":")[1], near=dn, near_kind=kind, att=[], seek=r.get("seek"))
                pending = []
            th = r.get("thg")
            if cur is not None and th is not None and th[2] >= 0:
                pending.append(th)
            c = r.get("cloudv")
            if cur is not None and c:
                objs = objects_at(r, lay, movable)
                for th in pending:
                    wx, wy = to_world(c["anchor"], th[3], th[4])
                    cur["att"].append((label(wx, wy, objs, half, furniture), th[5]))
                pending = []
                rows.append(cur); cur = None
    by = collections.defaultdict(list)
    for r in rows:
        by[r["how"]].append(r)
    print(f"{len(rows)} stops with a filed cloud over {len(paths)} runs")
    for how, rs in by.items():
        near = [r["near"] for r in rs]
        real_att = [any(l in ("ball", "block") for l, _ in r["att"]) for r in rs]
        real_near = [any(l in ("ball", "block") and rg < 0.8 for l, rg in r["att"]) for r in rs]
        print(f"  started by {how:>6}: {len(rs):3d} stops; nearest object at the start p10/50/90 "
              f"{'/'.join(f'{np.percentile(near, q):.2f}' for q in (10, 50, 90))} m, within 0.5 m on {sum(d < 0.5 for d in near)}; "
              f"the cloud attended a real object during {sum(real_att)} ({sum(real_near)} within 0.8 m); "
              f"nothing attended on {sum(not r['att'] for r in rs)}")
    arr = by.get("arrive", [])
    if arr:
        print("  arrival stops, one per line: nearest object (kind, m) -> what the cloud attended (label, range)")
        for r in arr:
            att = collections.Counter(l for l, _ in r["att"]).most_common(2)
            rmin = {l: min(rg for l2, rg in r["att"] if l2 == l) for l, _ in att}
            print(f"    {r['run']:>24} t {r['t']:7.1f}: {r['near_kind']:>5} {r['near']:.2f} m -> "
                  + (", ".join(f"{l} x{n} (nearest {rmin[l]:.2f} m)" for l, n in att) if att else "nothing"))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("rules", help="the stack rule's ladder over one or more full host logs")
    r.add_argument("logs", nargs="+")
    r.add_argument("--gapk", type=float, default=0.12, help="stack gap per metre of range (0 = a fixed 10 cm gap)")
    a = sub.add_parser("arms", help="per-arm cloud and object scores, NAME=GLOB per arm")
    a.add_argument("arms", nargs="+")
    a.add_argument("--gapk", type=float, default=0.12)
    hd = sub.add_parser("heading", help="the heading error, the reference's own motion and the yaw rail by which loop holds the reference")
    hd.add_argument("logs", nargs="+")
    sp = sub.add_parser("stops", help="every stop by how it started and what its cloud attended (things phase T4)")
    sp.add_argument("logs", nargs="+")
    sn = sub.add_parser("spins", help="20 s windows of walking in which the body turned a full turn without moving half a metre (the frustration)")
    sn.add_argument("logs", nargs="+")
    sl = sub.add_parser("skills", help="every skill fired against the manifest: reach, the answer seen or not, the thing at the look stop")
    sl.add_argument("logs", nargs="+")
    wh = sub.add_parser("where", help="every stop by where the body stands: at a thing, at a wall, or on open floor (the interesting scale)")
    wh.add_argument("logs", nargs="+")
    sk = sub.add_parser("seek", help="every seek episode against the manifest (things phase T2)")
    sk.add_argument("logs", nargs="+")
    th = sub.add_parser("things", help="the module's things against the manifest and the offline rule (things phase T1)")
    th.add_argument("logs", nargs="+")
    th.add_argument("--gapk", type=float, default=0.12)
    args = ap.parse_args()
    if args.cmd == "rules":
        cmd_rules(args.logs, args.gapk)
    elif args.cmd == "things":
        cmd_things(args.logs, args.gapk)
    elif args.cmd == "seek":
        cmd_seek(args.logs)
    elif args.cmd == "stops":
        cmd_stops(args.logs)
    elif args.cmd == "where":
        cmd_where(args.logs)
    elif args.cmd == "skills":
        cmd_skills(args.logs)
    elif args.cmd == "spins":
        cmd_spins(args.logs)
    elif args.cmd == "heading":
        cmd_heading(args.logs)
    else:
        cmd_arms(args.arms, args.gapk)


if __name__ == "__main__":
    main()
