#!/usr/bin/env python3
"""mover_tracks — stage 0 of chasing moving things: does a MOVING cluster separate from the static clusters'
jitter in the walking cloud?  Scored offline, against the truth the host logs for the scorer only.

Reads a host run's FULL JSONL made with `--train SPEED RUN STOP --log-movers WINDOW_S` (the walking cloud on
in the graph: CloudMap.walk_cloud).  On every cast tick with a cloud open the record carries
    "mvc": [[cx, cy, ext, top, ncols, hits, small, fresh, age_s], ...]   the stack rule's clusters through the recency
                             window, in the cloud's frame (the anchor's: x forward, y left); fresh = the share of the
                             cluster's voxels first seen inside the window, age_s = their mean age (logs before the
                             freshness fields carry seven entries and get the speed sections only)
    "mva": [wx, wy, wyaw]                                   the anchor's WORLD pose (truth; the scorer's labels)
    "mvw": 1                                                a walking cloud (0: a stop's)
    "train": [x, y, yaw, vx, vy, moving]                    the train's truth
and the trunk's world x, y and its quaternion in qpos, the movables' poses in qpos (the manifest's layout).

What it does, per log:
  1. puts every cluster in the world frame through the anchor and LABELS it by what it really was: `train` (within
     --label-m of the train's centre), `obj` (a ball or block), else `static` (legs, small furniture parts) or
     `wide` (a static cluster wider than --compact-m: a wall base or a table edge, whose VISIBLE part slides with
     the field of view -- a centroid that moves without the world moving, so not a gate candidate);
  2. TRACKS clusters across consecutive casts of one cloud (same anchor) by nearest neighbour under --gate, in the
     cloud's frame -- the frame in which the static world holds still and a mover moves;
  3. fits each track's velocity by least squares over the last --window seconds (three casts at least), and the
     COMMON-MODE velocity per cast as the median over the compact static tracks (the odometry's own error);
  4. reports the train's visibility (is it a cluster at all, by range and by moving/stopped), the speed
     distributions (static jitter against the train moving and the train stopped), how a threshold set from the
     static jitter's own spread would detect the moving train and how often it would fire on nothing, and
     whether subtracting the common mode tightens both;
  5. reports FRESHNESS: a thing that moves keeps entering voxels the cloud has never held, so its voxels stay fresh
     for as long as it moves, while a static thing's voxels are re-hit and age however much the visible subset
     flickers -- the measure the centroid's velocity turned out not to be (n = 6, 2026-09-27: the moving train's
     centroid speed read 0.09 m/s against 0.2 true, under the compact static clusters' p99 of 0.38).
The go/no-go for stage 1 (the mover gate): a gate on the cloud's own per-voxel ages that catches the moving train
and stays quiet on the static room.

usage:
  mover_tracks.py LOG... [--scene mj_host/models/microduck/scene_playroom_train.xml] [--window 0.5] [--chases]
                         [--gate 0.10] [--label-m 0.22] [--compact-m 0.25] [--svg out.svg] [--svg-fresh out.svg] [--per-seed]
"""
import argparse
import json
import math
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HZ = 50.0
CAST_S = 4.0 / HZ


def quat_yaw(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def pct(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    k = (len(s) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (k - lo)


def lsq_velocity(pts):
    """pts: [(t, x, y)] -> (vx, vy) by least squares; None with fewer than 3 points or under 0.15 s of span."""
    if len(pts) < 3:
        return None
    t0 = pts[0][0]
    ts = [p[0] - t0 for p in pts]
    if ts[-1] - ts[0] < 0.15:
        return None
    mt = sum(ts) / len(ts)
    den = sum((t - mt) ** 2 for t in ts)
    if den <= 0:
        return None
    mx = sum(p[1] for p in pts) / len(pts)
    my = sum(p[2] for p in pts) / len(pts)
    vx = sum((t - mt) * (p[1] - mx) for t, p in zip(ts, pts)) / den
    vy = sum((t - mt) * (p[2] - my) for t, p in zip(ts, pts)) / den
    return vx, vy


class Track:
    __slots__ = ("pts", "label", "last_tick", "id")
    _n = 0

    def __init__(self, label):
        self.pts = []
        self.label = label
        self.last_tick = -1
        Track._n += 1
        self.id = Track._n


def load_manifest(scene):
    man = Path(scene).with_suffix(".manifest.json")
    if not man.exists():
        sys.exit(f"no manifest beside {scene}")
    m = json.load(open(man))
    layout = {name: (adr, n) for name, adr, n in m["qpos_layout"]}
    objs = [name for name in layout if name.startswith("obj_")]
    return m, layout, objs


def score_log(path, layout, objs, window, gate, label_m, compact_m):
    """One log -> a dict of samples and counters."""
    out = dict(
        casts_walk=0, casts_stop=0, clusters=0,
        inview=defaultdict(lambda: [0, 0]),          # (moving, range bin) -> [ticks in view, ticks with a train cluster]
        speeds=defaultdict(list),                    # label key -> |v| estimates
        speeds_cm=defaultdict(list),                 # the same after subtracting the common mode
        train_err=[], train_err_cm=[],               # |v_est - v_true| for the moving train
        static_walk_s=0.0,                           # seconds of walking-cloud casts (for false-alarm rates)
        cm_norm=[],                                  # |common mode| per cast, walking clouds
        rows=[],                                     # (t, label, v_true, v_est, v_est_cm, range, fresh, age, cloud_age, track_age, dir_err, hits, age_w)
        seed=None,
    )
    seg_start = 0.0
    m = re.search(r"_s(\d+)\.jsonl$", path.name)
    out["seed"] = int(m.group(1)) if m else path.stem
    seg_key = None
    tracks = []
    for line in open(path):
        if not line.startswith("{") or '"mvc"' not in line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        tick = r["tick"]
        tsec = tick / HZ
        walking = r.get("mvw", 0) == 1
        ax, ay, ayaw = r["mva"]
        tr = r.get("train")
        if tr is None:
            continue
        tx, ty, _tyaw, tvx, tvy, tmoving = tr
        tmoving = int(tmoving) == 1
        qpos = r["qpos"]
        bx, by = r["x"], r["y"]
        byaw = quat_yaw(qpos[3], qpos[4], qpos[5], qpos[6])
        head_yaw = r["q"][7]                       # the head-yaw joint; home 0
        look = byaw + head_yaw
        obj_xy = [(qpos[layout[o][0]], qpos[layout[o][0] + 1]) for o in objs]

        if walking:
            out["casts_walk"] += 1
            out["static_walk_s"] += CAST_S
        else:
            out["casts_stop"] += 1

        key = (walking, round(ax, 3), round(ay, 3), round(ayaw, 3))
        if key != seg_key:
            seg_key = key
            tracks = []
            seg_start = tsec
        cloud_age = tsec - seg_start

        # the clusters, labelled in the world frame
        ca, sa = math.cos(ayaw), math.sin(ayaw)
        clusters = []
        for c in r["mvc"]:
            cx, cy = c[0], c[1]
            wx = ax + ca * cx - sa * cy
            wy = ay + sa * cx + ca * cy
            d_train = math.hypot(wx - tx, wy - ty)
            if d_train < label_m:
                label = "train"
            elif any(math.hypot(wx - ox, wy - oy) < label_m for ox, oy in obj_xy):
                label = "obj"
            else:
                label = "static"
            rng = math.hypot(wx - bx, wy - by)
            # a static cluster wider than a thing is a wall base or a table edge: its visible part slides with the
            # field of view, so its centroid moves without the world moving.  The gate's candidates are compact.
            if label == "static" and c[2] > compact_m:
                label = "wide"
            clusters.append(dict(cx=cx, cy=cy, label=label, rng=rng, ext=c[2], hits=c[5],
                                 fresh=c[7] if len(c) > 8 else float("nan"), age=c[8] if len(c) > 8 else float("nan"),
                                 age_w=c[9] if len(c) > 9 else float("nan"), vacated=c[10] if len(c) > 10 else float("nan"),
                                 tall=c[11] if len(c) > 11 else float("nan")))
        out["clusters"] += len(clusters)

        # the train's visibility: is it in the ToF's cone at all (within 2 m and 0.6 rad of where the head looks)
        if walking:
            d = math.hypot(tx - bx, ty - by)
            bear = wrap(math.atan2(ty - by, tx - bx) - look)
            if d < 2.0 and abs(bear) < 0.6:
                rb = min(3, int(d / 0.5))
                cell = out["inview"][(tmoving, rb)]
                cell[0] += 1
                if any(c["label"] == "train" for c in clusters):
                    cell[1] += 1

        # association: greedy nearest neighbour under the gate, in the cloud's frame
        alive = [t for t in tracks if tick - t.last_tick <= 24]    # a track survives six missed casts (flicker at range)
        pairs = []
        for i, c in enumerate(clusters):
            for j, t in enumerate(alive):
                px, py = t.pts[-1][1], t.pts[-1][2]
                dd = math.hypot(c["cx"] - px, c["cy"] - py)
                if dd < gate:
                    pairs.append((dd, i, j))
        pairs.sort()
        used_c, used_t = set(), set()
        assign = {}
        for dd, i, j in pairs:
            if i in used_c or j in used_t:
                continue
            used_c.add(i); used_t.add(j); assign[i] = alive[j]
        new_tracks = []
        for i, c in enumerate(clusters):
            t = assign.get(i)
            if t is None:
                t = Track(c["label"])
                new_tracks.append(t)
            t.pts.append((tsec, c["cx"], c["cy"]))
            t.last_tick = tick
            t.label = c["label"]                     # the latest truth label
            c["track"] = t
        tracks = alive + new_tracks

        # velocities over the window, and the common mode over the static tracks
        vels = {}
        for c in clusters:
            t = c["track"]
            pts = [p for p in t.pts if tsec - p[0] <= window + 1e-9]
            v = lsq_velocity(pts)
            if v is not None:
                vels[t.id] = v
        static_v = [vels[c["track"].id] for c in clusters if c["label"] == "static" and c["track"].id in vels]
        cm = None
        if len(static_v) >= 2:
            cm = (statistics.median(v[0] for v in static_v), statistics.median(v[1] for v in static_v))
            if walking:
                out["cm_norm"].append(math.hypot(*cm))
        # the train's true velocity in the cloud's frame
        tvx_c = ca * tvx + sa * tvy
        tvy_c = -sa * tvx + ca * tvy
        for c in clusters:
            t = c["track"]
            track_age = tsec - t.pts[0][0]
            if c["label"] == "train":
                key = "train_moving" if tmoving else "train_stopped"
                vt = math.hypot(tvx, tvy)
            else:
                key = c["label"]
                vt = 0.0
            if not walking:
                key += "@stop"
            if t.id not in vels:
                # no velocity yet (a track under three casts): still a freshness sample
                out["rows"].append((tsec, key, vt, float("nan"), float("nan"), c["rng"], c["fresh"], c["age"], cloud_age, track_age, float("nan"), c["hits"], c["age_w"], c["vacated"], c["tall"]))
                continue
            vx, vy = vels[t.id]
            sp = math.hypot(vx, vy)
            sp_cm = math.hypot(vx - cm[0], vy - cm[1]) if cm else float("nan")
            dir_err = float("nan")
            if c["label"] == "train" and tmoving:
                out["train_err"].append(math.hypot(vx - tvx_c, vy - tvy_c))
                if cm:
                    out["train_err_cm"].append(math.hypot(vx - cm[0] - tvx_c, vy - cm[1] - tvy_c))
                if sp > 1e-6:
                    dir_err = abs(wrap(math.atan2(vy, vx) - math.atan2(tvy_c, tvx_c)))
            out["speeds"][key].append(sp)
            if cm:
                out["speeds_cm"][key].append(sp_cm)
            out["rows"].append((tsec, key, vt, sp, sp_cm, c["rng"], c["fresh"], c["age"], cloud_age, track_age, dir_err, c["hits"], c["age_w"], c["vacated"], c["tall"]))
    return out


def score_chases(path, layout, objs, label_m):
    """THE CHASE (stage 1): every chase episode from the "chase" records -- [chasing, n, tx, ty, vx, vy, mover_seen] with the
    target in the ODOMETRY frame -- labelled by truth through the body's own pose: target_world = body_world +
    R(yaw_true - yaw_odom) (target_odom - body_odom).  Per episode: start, length, what it chased (the label of the target
    at the start and over the episode), the closest true approach to the train while it moved, and how it ended."""
    eps = []
    cur = None
    for line in open(path):
        if not line.startswith("{") or '"chase"' not in line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        ch = r["chase"]
        chasing = int(ch[0]) == 1
        tick = r["tick"]
        tr = r.get("train")
        qpos = r["qpos"]
        bx, by = r["x"], r["y"]
        byaw = quat_yaw(qpos[3], qpos[4], qpos[5], qpos[6])
        ox, oy, oyaw = r["odom"]
        if chasing:
            # the target in the world through the body's pose
            dx, dy = ch[2] - ox, ch[3] - oy
            dth = byaw - oyaw
            wx = bx + math.cos(dth) * dx - math.sin(dth) * dy
            wy = by + math.sin(dth) * dx + math.cos(dth) * dy
            label = "static"
            d_train = float("inf")
            if tr:
                d_train = math.hypot(wx - tr[0], wy - tr[1])
                if d_train < label_m:
                    label = "train_moving" if int(tr[5]) == 1 else "train_stopped"
            if label == "static":
                for o in objs:
                    if math.hypot(wx - qpos[layout[o][0]], wy - qpos[layout[o][0] + 1]) < label_m:
                        label = "obj"; break
            body_to_train = math.hypot(bx - tr[0], by - tr[1]) if tr else float("inf")
            if cur is None:
                cur = dict(start=tick, end=tick, labels=defaultdict(int), first=label, closest=body_to_train,
                           closest_moving=body_to_train if (tr and int(tr[5]) == 1) else float("inf"), walls=0, wall_prev=0)
            cur["end"] = tick
            cur["labels"][label] += 1
            cur["closest"] = min(cur["closest"], body_to_train)
            if tr and int(tr[5]) == 1:
                cur["closest_moving"] = min(cur["closest_moving"], body_to_train)
            w = r.get("wall", 0)
            if w and not cur["wall_prev"]:
                cur["walls"] += 1
            cur["wall_prev"] = w
        elif cur is not None:
            cur["end_event"] = r.get("event", "")
            eps.append(cur); cur = None
    if cur is not None:
        eps.append(cur)
    return eps


def report_chases(all_eps, secs_per_log):
    print("\n7. THE CHASE (stage 1): every chase episode, what it really chased (the truth label of its target), per seed")
    print("   seed  chases   s/chase  train-moving  train-stopped  obj  static   closest to a MOVING train (m, p50)   walls in chases")
    for seed, eps in all_eps:
        if not eps:
            print(f"   {str(seed):5s}      0"); continue
        by = defaultdict(int)
        for e in eps:
            by[max(e["labels"], key=e["labels"].get)] += 1
        lens = [(e["end"] - e["start"]) / HZ for e in eps]
        cm = [e["closest_moving"] for e in eps if e["closest_moving"] < 10]
        print(f"   {str(seed):5s}  {len(eps):6d}   {statistics.median(lens):6.1f}   {by['train_moving']:8d}   {by['train_stopped']:10d}   {by['obj']:4d}   {by['static']:5d}"
              f"   {fmt(pct(cm, .5), 2) if cm else '  -  '} (n={len(cm)})                 {sum(e['walls'] for e in eps)}")
    tot = [e for _, eps in all_eps for e in eps]
    if tot:
        by = defaultdict(int)
        for e in tot:
            by[max(e["labels"], key=e["labels"].get)] += 1
        n = len(tot)
        print(f"   all: {n} chases over {len(all_eps)} logs ({n / max(1, len(all_eps)):.1f} a run); by target: train moving {by['train_moving']}"
              f" ({by['train_moving'] / n:.2f}), stopped {by['train_stopped']}, balls/blocks {by['obj']}, static {by['static']} ({by['static'] / n:.2f});"
              f" median length {statistics.median((e['end'] - e['start']) / HZ for e in tot):.1f} s;"
              f" seconds chasing a run {sum((e['end'] - e['start']) / HZ for e in tot) / max(1, len(all_eps)):.0f}")


def fmt(x, d=3):
    return "nan" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{d}f}"


def report(results, window, per_seed):
    pooled = dict(speeds=defaultdict(list), speeds_cm=defaultdict(list), train_err=[], train_err_cm=[], cm_norm=[],
                  inview=defaultdict(lambda: [0, 0]), casts_walk=0, casts_stop=0, clusters=0, static_walk_s=0.0)
    for r in results:
        for k, v in r["speeds"].items():
            pooled["speeds"][k] += v
        for k, v in r["speeds_cm"].items():
            pooled["speeds_cm"][k] += v
        for k in ("train_err", "train_err_cm", "cm_norm"):
            pooled[k] += r[k]
        for k, v in r["inview"].items():
            pooled["inview"][k][0] += v[0]; pooled["inview"][k][1] += v[1]
        for k in ("casts_walk", "casts_stop", "clusters", "static_walk_s"):
            pooled[k] += r[k]

    print(f"mover_tracks: {len(results)} log(s), window {window:.2f} s")
    print(f"  casts with a cloud open: walking {pooled['casts_walk']}  at stops {pooled['casts_stop']}  "
          f"clusters {pooled['clusters']}  ({pooled['clusters'] / max(1, pooled['casts_walk'] + pooled['casts_stop']):.1f} per cast)")

    # 1. visibility
    print("\n1. THE TRAIN'S VISIBILITY on the walk (the train within 2 m and 0.6 rad of where the head looks): casts in view,")
    print("   and the share with a cluster within the label radius of it -- by the train's state and range")
    print("   state     range      in view   seen   share")
    for moving in (True, False):
        for rb in range(4):
            n, s = pooled["inview"].get((moving, rb), [0, 0])
            if n:
                print(f"   {'moving ' if moving else 'stopped'}   {rb * 0.5:.1f}-{rb * 0.5 + 0.5:.1f} m   {n:7d}  {s:5d}   {s / n:.2f}")

    # 2. speeds
    print("\n2. ESTIMATED SPEED of tracked clusters (m/s), walking clouds -- the static jitter against the train")
    print("   label            n      p50     p90     p99     max   | after the common mode: p50     p90     p99")
    for key in ("static", "wide", "obj", "train_stopped", "train_moving"):
        xs = pooled["speeds"].get(key, [])
        ys = pooled["speeds_cm"].get(key, [])
        if xs:
            print(f"   {key:14s} {len(xs):6d}  {fmt(pct(xs, .5))}  {fmt(pct(xs, .9))}  {fmt(pct(xs, .99))}  {fmt(max(xs))}   |"
                  f"  {fmt(pct(ys, .5)) if ys else '   -  '}  {fmt(pct(ys, .9)) if ys else '   -  '}  {fmt(pct(ys, .99)) if ys else '   -  '}")
    stop_keys = [k for k in pooled["speeds"] if k.endswith("@stop")]
    if stop_keys:
        print("   (at stops, a sweeping gaze:)")
        for key in sorted(stop_keys):
            xs = pooled["speeds"][key]
            print(f"   {key:14s} {len(xs):6d}  {fmt(pct(xs, .5))}  {fmt(pct(xs, .9))}  {fmt(pct(xs, .99))}  {fmt(max(xs))}")
    if pooled["train_err"]:
        print(f"   the moving train's velocity error |v_est - v_true|: median {fmt(pct(pooled['train_err'], .5))} m/s"
              f" (after the common mode {fmt(pct(pooled['train_err_cm'], .5)) if pooled['train_err_cm'] else 'nan'});"
              f" the common mode itself: median {fmt(pct(pooled['cm_norm'], .5))} p90 {fmt(pct(pooled['cm_norm'], .9))} m/s")

    # 3. detection
    st = pooled["speeds"].get("static", [])
    mv = pooled["speeds"].get("train_moving", [])
    ts = pooled["speeds"].get("train_stopped", [])
    if st and mv:
        p99 = pct(st, .99)
        mins = pooled["static_walk_s"] / 60.0
        print(f"\n3. A THRESHOLD FROM THE STATIC JITTER (p99 = {p99:.3f} m/s), walking clouds: recall on the moving train's"
              f" clusters,\n   the train stopped mistaken for a mover, and static clusters above it per minute of walking cloud ({mins:.1f} min)")
        print("   k x p99   thresh   recall(moving)   stopped>thr   static/min")
        for k in (1.0, 1.5, 2.0, 3.0):
            thr = k * p99
            rec = sum(1 for x in mv if x > thr) / len(mv)
            fs = sum(1 for x in ts if x > thr) / len(ts) if ts else float("nan")
            fa = sum(1 for x in st if x > thr) / max(1e-9, mins)
            print(f"   {k:3.1f}       {thr:.3f}     {rec:.2f}            {fmt(fs, 2)}         {fa:.2f}")
        p999 = pct(st, .999)
        print(f"   static p99.9 = {p999:.3f} m/s; moving-train p10 = {pct(mv, .1):.3f}, p50 = {pct(mv, .5):.3f} m/s (truth 0.2)")

    # 5. freshness: the voxels' own ages
    fr_rows = [row for r in results for row in r["rows"]
               if not row[1].endswith("@stop") and not math.isnan(row[6]) and row[8] >= 2 * window and row[9] >= window]
    if fr_rows:
        print(f"\n5. FRESHNESS on the walk -- of a cluster's voxels inside the window, the share the cloud had never held; clusters"
              f"\n   tracked for >= {window:.1f} s in clouds older than {2 * window:.1f} s ({len(fr_rows)} samples)")
        print("   label            n     fresh p10   p50    p90   | age s p10   p25   p50   p90")
        by = defaultdict(list)
        for row in fr_rows:
            by[row[1]].append(row)
        for key in ("static", "wide", "obj", "train_stopped", "train_moving"):
            xs = [row[6] for row in by.get(key, [])]
            ages = [row[7] for row in by.get(key, [])]
            if xs:
                print(f"   {key:14s} {len(xs):6d}     {pct(xs, .1):.2f}    {pct(xs, .5):.2f}   {pct(xs, .9):.2f}   |"
                      f"   {pct(ages, .1):5.2f} {pct(ages, .25):5.2f} {pct(ages, .5):5.2f} {pct(ages, .9):5.2f}")
        mv = by.get("train_moving", [])
        others = [row for k, v in by.items() if k not in ("train_moving", "train_stopped") for row in v]
        stopped = by.get("train_stopped", [])
        mins = sum(r["static_walk_s"] for r in results) / 60.0
        if mv and others:
            print("\n   the age gate at A = 0.3 s by RANGE and by SAMPLING (the cluster's returns in the window): recall on the moving"
                  "\n   train and everything else passing per minute -- where the false alarms live")
            print("   range        n(mov)  recall   others/min  |  hits      n(mov)  recall   others/min")
            hb = ((0, 4), (4, 8), (8, 16), (16, 1e9))
            for rb in range(4):
                mvb = [row for row in mv if rb * 0.5 <= row[5] < rb * 0.5 + 0.5]
                otb = [row for row in others if rb * 0.5 <= row[5] < rb * 0.5 + 0.5]
                lo, hi = hb[rb]
                mvh = [row for row in mv if lo <= row[11] < hi]
                oth = [row for row in others if lo <= row[11] < hi]
                print(f"   {rb * 0.5:.1f}-{rb * 0.5 + 0.5:.1f} m   {len(mvb):6d}   {sum(1 for row in mvb if row[7] < 0.3) / max(1, len(mvb)):.2f}"
                      f"    {sum(1 for row in otb if row[7] < 0.3) / max(1e-9, mins):7.2f}    |  {lo:2.0f}-{min(hi, 99):<3.0f}   {len(mvh):6d}"
                      f"   {sum(1 for row in mvh if row[7] < 0.3) / max(1, len(mvh)):.2f}    {sum(1 for row in oth if row[7] < 0.3) / max(1e-9, mins):7.2f}")
            print("   ...and by WHAT passed, at A = 0.3 s (per minute; within 1.5 m in brackets):",
                  "  ".join(f"{k} {sum(1 for row in v if row[7] < 0.3) / max(1e-9, mins):.1f} ({sum(1 for row in v if row[7] < 0.3 and row[5] < 1.5) / max(1e-9, mins):.1f})"
                            for k, v in by.items() if k not in ("train_moving", "train_stopped")))
            if any(not math.isnan(row[12]) for row in mv):
                print("\n   the HIT-WEIGHTED age (a re-hit voxel counts for its returns): the same gate")
                print("   A (s)     recall(moving)   stopped   others/min   others<1.5m/min")
                for A in (0.3, 0.5, 0.7, 1.0):
                    rec = sum(1 for row in mv if row[12] < A) / len(mv)
                    st = sum(1 for row in stopped if row[12] < A) / len(stopped) if stopped else float("nan")
                    fa = sum(1 for row in others if row[12] < A) / max(1e-9, mins)
                    fa_near = sum(1 for row in others if row[12] < A and row[5] < 1.5) / max(1e-9, mins)
                    print(f"   {A:.1f}         {rec:.2f}           {fmt(st, 2)}      {fa:6.2f}        {fa_near:6.2f}")
            if any(len(row) > 13 and not math.isnan(row[13]) for row in mv):
                print("\n   the TRAIL (vacated voxels within 0.25 m, last 1 s): share of clusters with a trail, and the gate 'hit-weighted age"
                      " < A AND trail >= 1'")
                print("   label            n    trail>=1   trail>=2  |  A (s)  recall(moving)  stopped  others/min  others<1.5m/min")
                for key in ("static", "wide", "obj", "train_stopped", "train_moving"):
                    rs = [row for row in by.get(key, []) if len(row) > 13 and not math.isnan(row[13])]
                    if rs:
                        print(f"   {key:14s} {len(rs):6d}     {sum(1 for row in rs if row[13] >= 1) / len(rs):.2f}      {sum(1 for row in rs if row[13] >= 2) / len(rs):.2f}")
                for A in (0.3, 0.5, 1.0):
                    rec = sum(1 for row in mv if row[12] < A and row[13] >= 1) / len(mv)
                    st = sum(1 for row in stopped if row[12] < A and row[13] >= 1) / len(stopped) if stopped else float("nan")
                    fa = sum(1 for row in others if row[12] < A and row[13] >= 1) / max(1e-9, mins)
                    fa_near = sum(1 for row in others if row[12] < A and row[13] >= 1 and row[5] < 1.5) / max(1e-9, mins)
                    print(f"                                                    |  {A:.1f}      {rec:.2f}         {fmt(st, 2)}     {fa:6.2f}       {fa_near:6.2f}")
            if any(len(row) > 14 and not math.isnan(row[14]) for row in mv):
                print("\n   ISOLATION (tall voxels within 0.25 m): share of clusters with none, and the gate 'hit-weighted age < A AND isolated'")
                print("   label            n    isolated  |  A (s)  recall(moving)  stopped  others/min  others<1.5m/min")
                for key in ("static", "wide", "obj", "train_stopped", "train_moving"):
                    rs = [row for row in by.get(key, []) if len(row) > 14 and not math.isnan(row[14])]
                    if rs:
                        print(f"   {key:14s} {len(rs):6d}     {sum(1 for row in rs if row[14] == 0) / len(rs):.2f}")
                for A in (0.3, 0.5, 1.0):
                    rec = sum(1 for row in mv if row[12] < A and row[14] == 0) / len(mv)
                    st = sum(1 for row in stopped if row[12] < A and row[14] == 0) / len(stopped) if stopped else float("nan")
                    fa = sum(1 for row in others if row[12] < A and row[14] == 0) / max(1e-9, mins)
                    fa_near = sum(1 for row in others if row[12] < A and row[14] == 0 and row[5] < 1.5) / max(1e-9, mins)
                    print(f"                                     |  {A:.1f}      {rec:.2f}         {fmt(st, 2)}     {fa:6.2f}       {fa_near:6.2f}")
            print(f"\n   an AGE GATE, no tracker (the module's own numbers: the cloud older than {2 * window:.1f} s, the cluster's mean"
                  f" voxel age under A): recall on the moving train,\n   the stopped train passing, everything else passing per minute"
                  f" of walking cloud ({mins:.1f} min) -- and of those, the share within 1.5 m")
            print("   A (s)     recall(moving)   stopped   others/min   others<1.5m/min   direction error p50 (rad)")
            for A in (0.3, 0.4, 0.5, 0.7, 1.0):
                rec = sum(1 for row in mv if row[7] < A) / len(mv)
                st = sum(1 for row in stopped if row[7] < A) / len(stopped) if stopped else float("nan")
                fa = sum(1 for row in others if row[7] < A) / max(1e-9, mins)
                fa_near = sum(1 for row in others if row[7] < A and row[5] < 1.5) / max(1e-9, mins)
                de = [row[10] for row in mv if row[7] < A and not math.isnan(row[10])]
                print(f"   {A:.1f}         {rec:.2f}           {fmt(st, 2)}      {fa:6.2f}        {fa_near:6.2f}          {fmt(pct(de, .5), 2) if de else '  -  '}")
        mv = by.get("train_moving", [])
        others = [row for k, v in by.items() if k != "train_moving" for row in v]
        stopped = by.get("train_stopped", [])
        mins = sum(r["static_walk_s"] for r in results) / 60.0
        if mv and others:
            print(f"\n   a FRESHNESS GATE: recall on the moving train, the stopped train passing it, and everything else passing it"
                  f" per minute of walking cloud ({mins:.1f} min)\n   fresh >=   recall(moving)   stopped   others/min   direction error p50 (rad) of the passing movers' velocity")
            for F in (0.5, 0.6, 0.7, 0.8, 0.9):
                rec = sum(1 for row in mv if row[6] >= F) / len(mv)
                st = sum(1 for row in stopped if row[6] >= F) / len(stopped) if stopped else float("nan")
                fa = sum(1 for row in others if row[6] >= F) / max(1e-9, mins)
                de = [row[10] for row in mv if row[6] >= F and not math.isnan(row[10])]
                print(f"   {F:.1f}         {rec:.2f}           {fmt(st, 2)}      {fa:6.2f}       {fmt(pct(de, .5), 2) if de else '  -  '}")
            print("   by range, fresh >= 0.7: moving train recall / others per minute")
            for rb in range(4):
                mvb = [row for row in mv if rb * 0.5 <= row[5] < rb * 0.5 + 0.5]
                otb = [row for row in others if rb * 0.5 <= row[5] < rb * 0.5 + 0.5]
                if mvb:
                    print(f"     {rb * 0.5:.1f}-{rb * 0.5 + 0.5:.1f} m   n={len(mvb):5d}  recall {sum(1 for row in mvb if row[6] >= 0.7) / len(mvb):.2f}"
                          f"   others/min {sum(1 for row in otb if row[6] >= 0.7) / max(1e-9, mins):.2f}")

    # 6. age against the time watched: the scale-free form
    ag_rows = [row for r in results for row in r["rows"]
               if not row[1].endswith("@stop") and not math.isnan(row[7]) and row[9] >= 0.5]
    if ag_rows:
        print("\n6. VOXEL AGE against the TIME WATCHED (the track's age), walking clouds.  A mover's voxels are as old as the"
              "\n   time it takes to cross one voxel (4 cm / 0.2 m/s = 0.2 s), however long it has been watched; a static"
              "\n   thing's voxels are as old as the watching.  The ratio age / watched is the scale-free mover score.")
        print("   watched (s)   label            n    age p50   p90   |  age/watched p50   p90")
        for lo, hi in ((0.5, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 1e9)):
            for key in ("static", "wide", "obj", "train_stopped", "train_moving"):
                rs = [row for row in ag_rows if row[1] == key and lo <= row[9] < hi]
                if len(rs) < 5:
                    continue
                ages = [row[7] for row in rs]
                ratio = [row[7] / row[9] for row in rs]
                print(f"   {lo:3.1f}-{min(hi, 99):<4.1f}     {key:14s} {len(rs):6d}   {pct(ages, .5):.2f}   {pct(ages, .9):.2f}   |"
                      f"      {pct(ratio, .5):.2f}         {pct(ratio, .9):.2f}")
        mv = [row for row in ag_rows if row[1] == "train_moving" and row[9] >= 1.0]
        others = [row for row in ag_rows if row[1] not in ("train_moving", "train_stopped") and row[9] >= 1.0]
        stopped = [row for row in ag_rows if row[1] == "train_stopped" and row[9] >= 1.0]
        mins = sum(r["static_walk_s"] for r in results) / 60.0
        if mv and others:
            print(f"\n   an AGE GATE on clusters watched >= 1 s: age / watched < R -- recall on the moving train, the stopped train"
                  f" passing, everything else passing per minute of walking cloud ({mins:.1f} min)")
            print("   R       recall(moving)   stopped   others/min   direction error p50 (rad)")
            for R in (0.15, 0.25, 0.35, 0.5):
                rec = sum(1 for row in mv if row[7] / row[9] < R) / len(mv)
                st = sum(1 for row in stopped if row[7] / row[9] < R) / len(stopped) if stopped else float("nan")
                fa = sum(1 for row in others if row[7] / row[9] < R) / max(1e-9, mins)
                de = [row[10] for row in mv if row[7] / row[9] < R and not math.isnan(row[10])]
                print(f"   {R:.2f}       {rec:.2f}           {fmt(st, 2)}      {fa:6.2f}       {fmt(pct(de, .5), 2) if de else '  -  '}")
            print("   the moving train's samples watched >= 1 s:", len(mv), " others:", len(others))

    if per_seed:
        print("\n4. PER SEED (walking clouds): casts, train-moving clusters tracked, their p50 speed, static p99, in-view share seen")
        print("   seed   castsW   trainMov  p50mov   staticP99   seenMov  seenStop")
        for r in results:
            mvs = r["speeds"].get("train_moving", [])
            sts = r["speeds"].get("static", [])
            iv_m = [v for k, v in r["inview"].items() if k[0]]
            iv_s = [v for k, v in r["inview"].items() if not k[0]]
            sm = sum(v[1] for v in iv_m) / max(1, sum(v[0] for v in iv_m))
            ss = sum(v[1] for v in iv_s) / max(1, sum(v[0] for v in iv_s))
            print(f"   {str(r['seed']):5s}  {r['casts_walk']:6d}   {len(mvs):7d}   {fmt(pct(mvs, .5)) if mvs else '  -  '}    "
                  f"{fmt(pct(sts, .99)) if sts else '  -  '}      {sm:.2f}     {ss:.2f}")
    return pooled


def write_svg(results, path, window, value=3, ylabel="|v| estimated (m/s)", title="estimated cluster speed on the walk",
              refline=(0.2, "the train's true speed 0.20"), gate=None, log=False):
    """One dot per tracked cluster on a walking cloud, in a strip per label: the static clusters, the wide ones, the
    balls and blocks, the stopped train and the moving train.  value 3 = the estimated speed, 6 = freshness."""
    rows = [row for r in results for row in r["rows"] if not row[1].endswith("@stop") and not math.isnan(row[value])]
    if gate:
        rows = [row for row in rows if row[8] >= 2 * window and row[9] >= window]
    if not rows:
        return
    W, H, L, B = 860, 440, 70, 60
    ymax = 1.0 if value == 6 else 60.0 if log else max(0.5, min(1.5, pct([r[value] for r in rows], .995) * 1.1))
    ymin = 0.04 if log else 0.0
    cols = {"static": "#7a7a7a", "wide": "#b0b0b0", "obj": "#2a9d8f", "train_stopped": "#e9a23b", "train_moving": "#d62828"}
    xs_of = {"static": 0.05, "wide": 0.27, "obj": 0.49, "train_stopped": 0.71, "train_moving": 0.93}
    import random
    rnd = random.Random(1)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" font-family="sans-serif" font-size="12">',
             f'<rect width="{W}" height="{H}" fill="white"/>',
             f'<text x="{W/2}" y="20" text-anchor="middle" font-size="14">{title}, {window:.1f} s window'
             f' -- {len(rows)} tracked clusters</text>']
    pw, ph = W - L - 20, H - B - 40
    def Y(v):
        if log:
            f = (math.log10(max(v, ymin)) - math.log10(ymin)) / (math.log10(ymax) - math.log10(ymin))
            return 40 + ph * (1 - min(1.0, f))
        return 40 + ph * (1 - min(v, ymax) / ymax)
    ticks = (0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50) if log else [ymax * k / 5 for k in range(6)]
    for v in ticks:
        parts.append(f'<line x1="{L}" x2="{W-20}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" stroke="#ddd"/>')
        parts.append(f'<text x="{L-6}" y="{Y(v)+4:.1f}" text-anchor="end">{v:g}</text>')
    if refline:
        parts.append(f'<line x1="{L}" x2="{W-20}" y1="{Y(refline[0]):.1f}" y2="{Y(refline[0]):.1f}" stroke="#d62828" stroke-dasharray="4 3"/>')
        parts.append(f'<text x="{W-24}" y="{Y(refline[0])-4:.1f}" text-anchor="end" fill="#d62828">{refline[1]}</text>')
    counts = defaultdict(int)
    for row in rows:
        key = row[1]
        counts[key] += 1
    for key, xc in xs_of.items():
        parts.append(f'<text x="{L + pw*xc:.1f}" y="{H-B+18}" text-anchor="middle" fill="{cols[key]}">{key} (n={counts[key]})</text>')
    # at most ~1500 dots per strip, so the rare classes (the train) are drawn in full
    per = defaultdict(list)
    for row in rows:
        per[row[1]].append(row)
    drawn = []
    for key, rs in per.items():
        step = max(1, len(rs) // 1500)
        drawn += rs[::step]
    for row in drawn:
        key, sp = row[1], row[value]
        x = L + pw * (xs_of[key] + rnd.uniform(-0.09, 0.09))
        parts.append(f'<circle cx="{x:.1f}" cy="{Y(sp):.1f}" r="1.8" fill="{cols[key]}" fill-opacity="0.45"/>')
    parts.append(f'<text x="18" y="{H/2}" transform="rotate(-90 18 {H/2})" text-anchor="middle">{ylabel}</text>')
    parts.append("</svg>")
    Path(path).write_text("\n".join(parts))
    print(f"svg: {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--scene", default=str(REPO / "mj_host/models/microduck/scene_playroom_train.xml"))
    ap.add_argument("--window", type=float, default=0.5, help="seconds of track the velocity is fitted over")
    ap.add_argument("--gate", type=float, default=0.10, help="association gate between casts (m)")
    ap.add_argument("--compact-m", type=float, default=0.25, help="a static cluster wider than this is 'wide' (a wall base), not a gate candidate")
    ap.add_argument("--label-m", type=float, default=0.22, help="a cluster within this of a body's centre is that body")
    ap.add_argument("--svg", default=None, help="the speed scatter")
    ap.add_argument("--svg-fresh", default=None, help="the freshness scatter")
    ap.add_argument("--svg-age", default=None, help="the voxel-age scatter (log scale)")
    ap.add_argument("--per-seed", action="store_true")
    ap.add_argument("--chases", action="store_true", help="stage 1: score the chase episodes of the 'chase' records only")
    a = ap.parse_args()
    _m, layout, objs = load_manifest(a.scene)
    if a.chases:
        all_eps = []
        for p in a.logs:
            eps = score_chases(Path(p), layout, objs, a.label_m)
            m = re.search(r"_s(\d+)\.jsonl$", Path(p).name)
            all_eps.append((int(m.group(1)) if m else Path(p).stem, eps))
        report_chases(all_eps, 0)
        return
    results = [score_log(Path(p), layout, objs, a.window, a.gate, a.label_m, a.compact_m) for p in a.logs]
    report(results, a.window, a.per_seed)
    if a.svg:
        write_svg(results, a.svg, a.window)
    if a.svg_age:
        write_svg(results, a.svg_age, a.window, value=7, ylabel="mean voxel age of the cluster (s, log)",
                  title="voxel age of tracked clusters on the walk", refline=(0.3, "a gate at 0.3 s"), gate=True, log=True)
    if a.svg_fresh:
        write_svg(results, a.svg_fresh, a.window, value=6, ylabel="fresh (share of the cluster's voxels new to the cloud)",
                  title="freshness of tracked clusters on the walk", refline=(0.7, "a gate at 0.7"), gate=True)


if __name__ == "__main__":
    main()
