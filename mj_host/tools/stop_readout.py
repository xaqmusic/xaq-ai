#!/usr/bin/env python3
"""stop_readout — what the stop-and-look costs and what it buys (the stop as a loop, design doc §17.112, 2026-10-08).

The operator: "the look is currently interrupting the chase."  From the host's FULL JSONL (the `stop`, `event`, `chase`,
`train`, `cld` records), per stop:

  trigger      the event that started it: arrive / look (after a skill's back-off) / impeded / start (the timer) /
               stuck / lost / the look (the arbiter's look loop, host --stop-by-look)
  length       seconds from the start to the walk's return
  end          chase (a chase confirmed during the stop ended it -- the host logs these as stop:bored), cap (ran the full
               --stop-secs), fall (rescued), cloud (ended early otherwise: the cloud stopped growing, or the gaze's novelty)
  surprise     the integral of the stop cloud's new_fraction (the share of the voxels touched in the last second that were
               never seen before) over the stop, in seconds -- what the look resolved; and its value in the first second
  train        seconds the train was MOVING within 1.5 m of the trunk while standing; of them seconds chasing

and, for the whole run, the moving train within 1.5 m split by walk and stand with the chase's share of each.

Usage: stop_readout.py "glob" [...] [--cap SECS] [--per-run]
Scoring only: the train's pose is truth; nothing here is a brain input.
"""
from __future__ import annotations

import collections
import glob
import json
import math
import statistics
import sys

DT = 0.02
NEAR = 1.5
STARTS = {"stop:arrive": "arrive", "stop:look": "look", "stop:impeded": "impeded", "stop:start": "timer",
          "stop:stuck": "stuck", "stop:lost": "lost", "stop:bylook": "the look"}


def run(path: str, cap_s: float) -> dict:
    stops, cur = [], None
    near = collections.Counter()
    prev_chase = 0
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        ev = r.get("event") or ""
        ch = r.get("chase")
        tr = r.get("train")
        chasing = int(ch[0]) if ch else 0
        moving_near = bool(tr and tr[5] and math.hypot(tr[0] - r["x"], tr[1] - r["y"]) < NEAR)
        in_stop = int(r.get("stop", 0)) != 0
        if moving_near:
            k = "stand" if in_stop else "walk"
            near[k] += 1
            near[k + "_chase"] += 1 if (chasing or (ch and ch[10])) else 0
        if ev in STARTS and cur is None:
            cur = dict(trigger=STARTS[ev], t0=r["t"], n=0, surprise=0.0, first_s=[], train=0, train_chase=0,
                       end="cloud", chase_edge=False)
        if cur is not None:
            if in_stop or cur["n"] == 0:
                cur["n"] += 1
                cld = r.get("cld")
                if cld:
                    cur["surprise"] += cld[2] * DT
                    if r["t"] - cur["t0"] < 1.0:
                        cur["first_s"].append(cld[2])
                if moving_near:
                    cur["train"] += 1
                    cur["train_chase"] += chasing
                if chasing and not prev_chase:
                    cur["chase_edge"] = True
                if ev == "stop:rescued":
                    cur["end"] = "fall"
                if ev == "stop:escape":
                    cur["escape"] = True
            else:
                if chasing and not prev_chase:
                    cur["chase_edge"] = True          # the stop ended on this tick: --stop-on-chase
                cur["len"] = cur["n"] * DT
                if cur["end"] != "fall":
                    cur["end"] = "chase" if cur["chase_edge"] else ("cap" if cur["len"] >= cap_s - 0.1 else "cloud")
                stops.append(cur)
                cur = None
        prev_chase = chasing
    if cur is not None:
        cur["len"] = cur["n"] * DT
        cur["end"] = "run end"
        stops.append(cur)
    return dict(run=path.rsplit("/", 1)[-1], stops=stops, near=near)


def pct(a, b):
    return 100.0 * a / b if b else float("nan")


def main() -> None:
    args = sys.argv[1:]
    cap_s, per_run = 60.0, False
    if "--cap" in args:
        i = args.index("--cap"); cap_s = float(args[i + 1]); del args[i:i + 2]
    if "--per-run" in args:
        args.remove("--per-run"); per_run = True
    paths = sorted(p for a in args for p in glob.glob(a) if p.endswith(".jsonl"))
    if not paths:
        sys.exit("no logs")
    runs = [run(p, cap_s) for p in paths]
    n = len(runs)
    allst = [s for r in runs for s in r["stops"]]
    print(f"{n} runs, {len(allst)} stops ({len(allst) / n:.1f} a run), standing {sum(s['len'] for s in allst) / n:.0f} s a run")
    by = collections.defaultdict(list)
    for s in allst:
        by[s["trigger"]].append(s)
    print(f"{'trigger':9s} {'/run':>5s} {'len s':>6s} {'surprise s':>10s} {'1st s nf':>8s} {'train s':>8s}   ends")
    for k, v in sorted(by.items(), key=lambda kv: -len(kv[1])):
        ends = collections.Counter(s["end"] for s in v)
        fs = [statistics.mean(s["first_s"]) for s in v if s["first_s"]]
        print(f"{k:9s} {len(v) / n:5.1f} {statistics.mean(s['len'] for s in v):6.1f} "
              f"{statistics.mean(s['surprise'] for s in v):10.2f} {statistics.mean(fs) if fs else float('nan'):8.3f} "
              f"{sum(s['train'] for s in v) * DT / n:8.1f}   "
              + " ".join(f"{e} {c}" for e, c in ends.most_common()))
    near = collections.Counter()
    for r in runs:
        near.update(r["near"])
    print(f"the moving train within {NEAR} m: walk {near['walk'] * DT / n:.1f} s a run (chasing {pct(near['walk_chase'], near['walk']):.1f} %), "
          f"stand {near['stand'] * DT / n:.1f} s a run (chasing {pct(near['stand_chase'], near['stand']):.1f} %)")
    ended_chase = sum(1 for s in allst if s["end"] == "chase")
    print(f"stops ended by a chase: {ended_chase} ({ended_chase / n:.1f} a run)")
    if per_run:
        for r in runs:
            st = r["stops"]
            print(f"  {r['run']}: {len(st)} stops, {sum(s['len'] for s in st):.0f} s standing, train near standing "
                  f"{r['near']['stand'] * DT:.1f} s (chasing {r['near']['stand_chase'] * DT:.1f}), "
                  f"walking {r['near']['walk'] * DT:.1f} s (chasing {r['near']['walk_chase'] * DT:.1f})")


if __name__ == "__main__":
    main()
