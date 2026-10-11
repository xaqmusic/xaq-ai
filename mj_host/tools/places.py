#!/usr/bin/env python3
"""places — where the walk spends its time, and whether it goes back to where it was stuck (design doc §17.116, 2026-10-10).

The operator, after ★ E4: "the next improvement is in the play loop and how we handle places ... so the robot can avoid areas
it has been stuck in or are simply corners without objects to interact with."  From the host's FULL JSONL and the scene
manifest (CLOUD_OBJECTS_MANIFEST, as cloud_objects.py), per run from --from s:
  corner %     of the ticks, the trunk within 0.5 m of two of the room's walls
  open %       the trunk at least 0.5 m from every wall and every piece of furniture's edge
  cells        0.25 m cells the trunk visited
  traps        sits (sit:settle) and stuck stops (stop:stuck), a run -- the escape after either is not counted
  back         of those, within 0.4 m of an earlier trap: any time later / more than 60 s later (a return, not the same
               episode)
  first        traps with no earlier trap within 0.4 m; seek-led: the seek loop steered most of the 5 s before (§17.117)
  skills       skills started a run (the `skill` field rising), the sit not counted
Usage: CLOUD_OBJECTS_MANIFEST=... places.py DIR [DIR ...] [--from 30]
Several dirs: a column each, then the mean difference to the first over the seeds both have, with its 95 % CI.
Scoring only: the truth (qpos, manifest) labels; nothing here is a brain input.
"""
from __future__ import annotations

import collections
import glob
import json
import math
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cloud_objects as co  # noqa: E402

KEYS = ("corner %", "open %", "cells", "traps", "back", "back >60 s", "first", "first, seek-led", "skills")


def run(path, half, furniture, t0):
    n = corner = opn = 0
    cells, traps = set(), []
    back = back60 = skills = first = first_seek = 0
    steer = collections.deque(maxlen=250)   # the steering loop over the last 5 s
    prev_sk = ""
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        if r["t"] < t0:
            continue
        x, y = r["x"], r["y"]
        n += 1
        dw = [half - abs(x), half - abs(y)]
        corner += dw[0] < 0.5 and dw[1] < 0.5
        df = min((math.hypot(x - f["x"], y - f["y"]) - f["extent"] for f in furniture), default=9.0)
        opn += min(dw) >= 0.5 and df >= 0.5
        cells.add((math.floor(x / 0.25), math.floor(y / 0.25)))
        ev = r.get("event") or ""
        steer.append(r.get("steer", 0))
        if ev in ("sit:settle", "stop:stuck"):
            near = [r["t"] - t for tx, ty, t in traps if math.hypot(x - tx, y - ty) < 0.4]
            back += bool(near)
            if not near:
                first += 1
                first_seek += collections.Counter(steer).most_common(1)[0][0] == 3
            back60 += any(g > 60 for g in near)
            traps.append((x, y, r["t"]))
        sk = r.get("skill", "")
        skills += bool(sk) and sk != prev_sk and sk != "sit"
        prev_sk = sk
    if not n:
        return None
    return {"corner %": 100 * corner / n, "open %": 100 * opn / n, "cells": len(cells), "traps": len(traps),
            "back": back, "back >60 s": back60, "first": first, "first, seek-led": first_seek, "skills": skills}


def seed_of(p):
    m = re.search(r"_s(\d+)\.jsonl$", p)
    return int(m.group(1)) if m else p


def main():
    args = sys.argv[1:]
    t0 = 30.0
    if "--from" in args:
        i = args.index("--from"); t0 = float(args[i + 1]); del args[i:i + 2]
    half, _lay, _mov, furniture = co.load_scene()
    arms = []
    for d in args:
        per = {}
        for p in sorted(glob.glob(os.path.join(d, "*.jsonl"))):
            v = run(p, half, furniture, t0)
            if v:
                per[seed_of(p)] = v
        arms.append((os.path.basename(d.rstrip("/")), per))
    print(f"{'':16s}" + "".join(f"{name:>22s}" for name, _ in arms))
    for k in KEYS:
        row = f"{k:16s}"
        for _name, per in arms:
            v = [x[k] for x in per.values()]
            row += f"{statistics.mean(v):13.1f} ± {statistics.stdev(v) if len(v) > 1 else 0:5.1f}  " if v else f"{'-':>22s}"
        print(row)
    base = arms[0][1]
    for name, per in arms[1:]:
        common = sorted(set(base) & set(per))
        if len(common) < 2:
            continue
        print(f"\n{name} - {arms[0][0]} (paired, n {len(common)})")
        for k in KEYS:
            d = [per[s][k] - base[s][k] for s in common]
            m, sd = statistics.mean(d), statistics.stdev(d)
            h = 2.07 * sd / math.sqrt(len(d))
            print(f"  {k:16s} {m:+7.2f}  [{m - h:+.2f}, {m + h:+.2f}]")


if __name__ == "__main__":
    main()
