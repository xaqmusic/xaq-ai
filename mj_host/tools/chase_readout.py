#!/usr/bin/env python3
"""chase_readout — how well the duck chases the train (ten-minutes phase, the chase push, 2026-10-02).

The operator: "the chase loop, once arbitrated, needs to keep the robot moving; I am seeing the robot stop and look after a
short chase, which breaks off the chasing behaviour."  From the host's FULL JSONL (the `chase`, `train`, `stop`, `drive`
records) and its stderr summary, per run:

  chases          chase episodes started (the record's chase[0] rising), their median length
  at the train    episodes whose target was within 0.4 m of the train's true position at some tick (the rest chased
                  something else: a fragment, a ball)
  chase s         seconds chasing; of them seconds on the MOVING train; the share of those spent CLOSING on it (the
                  trunk-to-train distance falling)
  following s     seconds the train was moving within 1.5 m of the trunk and the walk was not stopped (the measure the
                  operator watches: is the duck going after it)
  closest         the trunk's closest approach to the train over the run; contacts (trunk within 0.15 m of the train's
                  surface)
  lost -> stop    stops started by a lost chase (`stop:lost`): the break-off; and stand seconds within 4 s after a
                  chase ends
  re-acquired     from the stderr summary line ("N re-acquired")
  pursuit s       seconds in the pursuit after a loss (chase[10], the coasting flag, once the phantom lever lands)

Usage: chase_readout.py "glob" [...] [--per-run]
Scoring only: the train's pose is truth; nothing here is a brain input.
"""
from __future__ import annotations

import glob
import json
import math
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ten_minutes as tm  # noqa: E402

TRAIN_HALF = 0.09          # the train's half-length-ish, for the surface distance
DT = 0.02


def run(path: str) -> dict:
    out = dict(run=os.path.basename(path), chases=0, at_train=0, chase_s=0.0, chase_moving_s=0.0, closing_s=0.0,
               follow_s=0.0, closest=9.0, contacts=0, lost_stops=0, stand_after_s=0.0, pursuit_s=0.0, lens=[],
               reacq=0)
    prev_c, prev_d, t0, at, in_contact, after_until = 0, None, 0.0, False, False, -1.0
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        t = r["t"]
        ch = r.get("chase")
        tr = r.get("train")
        if ch is None or tr is None:
            continue
        x, y = r["x"], r["y"]
        d = math.hypot(tr[0] - x, tr[1] - y)
        moving = bool(tr[5])
        c = int(ch[0])
        if r["event"] == "stop:lost":
            out["lost_stops"] += 1
        if c and not prev_c:
            out["chases"] += 1
            t0, at = t, False
        if c:
            out["chase_s"] += DT
            byaw = tm.yaw_of(r["qpos"][3:7])
            ox, oy, oyaw = r["odom"]
            dy_ = byaw - oyaw
            wx = x + math.cos(dy_) * (ch[2] - ox) - math.sin(dy_) * (ch[3] - oy)
            wy = y + math.sin(dy_) * (ch[2] - ox) + math.cos(dy_) * (ch[3] - oy)
            if math.hypot(wx - tr[0], wy - tr[1]) < 0.4:
                at = True
            if moving:
                out["chase_moving_s"] += DT
                if prev_d is not None and d < prev_d:
                    out["closing_s"] += DT
        if prev_c and not c:
            out["lens"].append(t - t0)
            out["at_train"] += at
            after_until = t + 4.0
        if t <= after_until and r["drive"] == "stand":
            out["stand_after_s"] += DT
        if len(ch) > 10 and ch[10]:
            out["pursuit_s"] += DT
        if moving and d < 1.5 and r["drive"] == "walk" and not r.get("stop", 0):
            out["follow_s"] += DT
        surf = d - TRAIN_HALF
        out["closest"] = min(out["closest"], surf)
        if surf < 0.15 and not in_contact:
            out["contacts"] += 1
        in_contact = surf < 0.15
        prev_c, prev_d = c, d
    err = os.path.splitext(path)[0] + ".stderr"
    if os.path.exists(err):
        m = re.search(r"chases: \d+ started \((\d+) re-acquired", open(err).read())
        if m:
            out["reacq"] = int(m.group(1))
    return out


def main():
    args = sys.argv[1:]
    per_run = "--per-run" in args
    args = [a for a in args if a != "--per-run"]
    paths = sorted(p for g in args for p in glob.glob(g))
    runs = [run(p) for p in paths]
    n = len(runs) or 1
    lens = sorted(x for r in runs for x in r["lens"])

    def mean(k):
        return sum(r[k] for r in runs) / n
    print(f"{len(runs)} runs")
    print(f"  chases {mean('chases'):.1f} a run (at the train {sum(r['at_train'] for r in runs)}/{sum(r['chases'] for r in runs)}), "
          f"median length {statistics.median(lens) if lens else 0:.1f} s, p90 {lens[int(.9 * len(lens))] if lens else 0:.1f} s")
    cm = sum(r["chase_moving_s"] for r in runs)
    print(f"  chase {mean('chase_s'):.1f} s a run, on the moving train {mean('chase_moving_s'):.1f} s "
          f"(closing on it {100 * sum(r['closing_s'] for r in runs) / cm if cm else 0:.0f} % of that)")
    print(f"  following the moving train (within 1.5 m, walking) {mean('follow_s'):.1f} s a run")
    print(f"  closest approach median {statistics.median(r['closest'] for r in runs):.2f} m; contacts {mean('contacts'):.1f} a run")
    print(f"  lost -> stop {mean('lost_stops'):.1f} a run; standing within 4 s after a chase {mean('stand_after_s'):.1f} s a run; "
          f"re-acquired {mean('reacq'):.1f} a run; pursuit {mean('pursuit_s'):.1f} s a run")
    if per_run:
        for r in runs:
            print(f"    {r['run']:40s} chases {r['chases']:3d} chase {r['chase_s']:5.1f}s moving {r['chase_moving_s']:5.1f}s "
                  f"follow {r['follow_s']:5.1f}s closest {r['closest']:.2f} contacts {r['contacts']} lost-stops {r['lost_stops']} "
                  f"reacq {r['reacq']} pursuit {r['pursuit_s']:.1f}s")


if __name__ == "__main__":
    main()
