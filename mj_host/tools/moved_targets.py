#!/usr/bin/env python3
"""moved_targets — interactions aimed at a thing that had already moved (the chase push, design doc §17.113, 2026-10-08).

The operator: "when the robot sees the moving object, it will walk to the place the object was when it completed the look,
stop there, and begin the stationary object interaction on nothing."  From the host's FULL JSONL and the scene manifest
(CLOUD_OBJECTS_MANIFEST, as cloud_objects.py): every time the seek loop takes a target (its need rising from 0), the target is
put into the world through the odometry's own pose and matched to the nearest movable thing (within 0.2 m of its edge, the
train included); at the next arrival stop and the next skill, that thing is looked up again.  Counted:
  arrival / skill  at its thing      the thing is within 0.15 m of where it was when targeted
                   its thing moved   it is not: the interaction is aimed at an empty place
and, of the moved ones, which kind moved and whether the target was taken at a stop or on the walk.
Usage: CLOUD_OBJECTS_MANIFEST=... moved_targets.py "glob" [...] [--per-run]
Scoring only: the truth (qpos, manifest) labels; nothing here is a brain input.
"""
from __future__ import annotations

import collections
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cloud_objects as co  # noqa: E402


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def world(r, tx, ty):
    ox, oy, oyaw = r["odom"]
    d = yaw_of(r["qpos"][3:7]) - oyaw
    return (r["x"] + math.cos(d) * (tx - ox) - math.sin(d) * (ty - oy),
            r["y"] + math.sin(d) * (tx - ox) + math.cos(d) * (ty - oy))


def run(path, lay, movable):
    out, kinds, where = collections.Counter(), collections.Counter(), collections.Counter()
    seek_prev, tgt, prev_sk = 0.0, None, ""
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        s, ch = r.get("seek"), r.get("chase")
        sv = s[0] if s else 0.0
        ev, sk = r.get("event") or "", r.get("skill", "")
        if sv > 0 and seek_prev == 0 and ch:
            wx, wy = world(r, ch[2], ch[3])
            d, name, ox, oy = min((math.hypot(ox - wx, oy - wy) - e, k, ox, oy) for k, ox, oy, e in co.objects_at(r, lay, movable))
            tgt = dict(kind=name if d < 0.2 else None, ox=ox, oy=oy, stopped=int(r.get("stop", 0)) != 0)
        seek_prev = sv
        is_skill = bool(sk) and sk != prev_sk
        if (ev == "stop:arrive" or is_skill) and tgt and tgt["kind"]:
            now = [(ox, oy) for k, ox, oy, _e in co.objects_at(r, lay, movable) if k == tgt["kind"]]
            moved = min(math.hypot(ox - tgt["ox"], oy - tgt["oy"]) for ox, oy in now) > 0.15
            out[("skill" if is_skill else "arrival", "its thing moved" if moved else "at its thing")] += 1
            if moved:
                kinds[tgt["kind"]] += 1
                where["targeted at a stop" if tgt["stopped"] else "targeted on the walk"] += 1
        prev_sk = sk
    return out, kinds, where


def main():
    args = sys.argv[1:]
    per_run = "--per-run" in args
    args = [a for a in args if a != "--per-run"]
    _half, lay, movable, _f = co.load_scene()
    paths = sorted(p for a in args for p in glob.glob(a) if p.endswith(".jsonl"))
    tot, kinds, where = collections.Counter(), collections.Counter(), collections.Counter()
    for p in paths:
        o, k, w = run(p, lay, movable)
        tot.update(o); kinds.update(k); where.update(w)
        if per_run:
            print(f"  {os.path.basename(p)}: " + ", ".join(f"{a} {b}: {v}" for (a, b), v in sorted(o.items())))
    n = max(1, len(paths))
    for what in ("arrival", "skill"):
        moved, at = tot[(what, "its thing moved")], tot[(what, "at its thing")]
        print(f"{what:8s} at its thing {at / n:5.2f} a run, its thing moved {moved / n:5.2f} a run "
              f"({100 * moved / max(1, moved + at):.0f} %)")
    print("the moved things:", dict(kinds.most_common()), "|", dict(where))


if __name__ == "__main__":
    main()
