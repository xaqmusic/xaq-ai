#!/usr/bin/env python3
"""neck_burst_readout — does pushing the head forward make the walk faster? (the causal test, design doc §17.113 N1, 2026-10-09)

The operator: "explore a short period where the neck extends the head out farther forward for a short burst of speed when the
robot is already walking."  From the host's FULL JSONL (`nb` 1 during a burst, `fore` the slide, `chase`, `stop`, `twist`,
`qpos`): half-second windows of a chase on the walk (chasing or coasting, no stop at either end), split by whether a burst was
on at the window's start --
  speed     the body's real displacement over the window (m/s)
  vx cmd    the forward command
  fore      the slide's position (its forward limit is 0.66 at --intent-head-translate 0.6)
  pitch     the trunk's pitch (rad, + nose down)
and, per run, falls (`reset:handoff`) within 3 s after a burst's start against the run's falls.  Per-run means are paired
(burst against no burst in the same run): the mean difference and its 95 % CI.
Usage: neck_burst_readout.py "glob" [...]
"""
import glob, json, math, statistics, sys


def pitch_of(q):
    w, x, y, z = q
    return math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))


def main():
    pairs, agg = [], {True: [], False: []}
    falls_after = falls_all = bursts = 0
    for path in sorted(p for a in sys.argv[1:] for p in glob.glob(a)):
        R = [json.loads(l) for l in open(path) if l.startswith('{"t"')]
        per = {True: [], False: []}
        starts = [r["t"] for k, r in enumerate(R) if r.get("nb") and (k == 0 or not R[k - 1].get("nb"))]
        bursts += len(starts)
        for r in R:
            if r.get("event") == "reset:handoff":
                falls_all += 1
                if any(0 <= r["t"] - s <= 3 for s in starts):
                    falls_after += 1
        for i in range(0, len(R) - 25, 25):
            a, b = R[i], R[i + 25]
            ch = a.get("chase") or [0] * 14
            if not (ch[0] or ch[10]) or int(a.get("stop", 0)) or int(b.get("stop", 0)):
                continue
            row = (math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / 0.5, a["twist"][0], a.get("fore", 0.0), pitch_of(a["qpos"][3:7]))
            per[bool(a.get("nb"))].append(row)
            agg[bool(a.get("nb"))].append(row)
        if per[True] and per[False]:
            pairs.append(statistics.mean(x[0] for x in per[True]) - statistics.mean(x[0] for x in per[False]))
    for k, name in ((True, "burst"), (False, "no burst")):
        v = agg[k]
        if v:
            print(f"{name:9s} windows {len(v):5d}: speed {statistics.mean(x[0] for x in v):.3f} m/s, vx cmd {statistics.mean(x[1] for x in v):.2f}, "
                  f"fore {statistics.mean(x[2] for x in v):+.2f}, trunk pitch {statistics.mean(x[3] for x in v):+.3f}")
    if len(pairs) >= 2:
        m, sd = statistics.mean(pairs), statistics.stdev(pairs)
        h = 2.07 * sd / math.sqrt(len(pairs))
        print(f"paired by run (n {len(pairs)}): speed burst - no burst {m:+.3f} m/s, 95 % CI [{m - h:+.3f}, {m + h:+.3f}]")
    print(f"bursts {bursts}; falls within 3 s of a burst's start {falls_after} of {falls_all}")


if __name__ == "__main__":
    main()
