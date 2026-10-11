#!/usr/bin/env python3
"""stop_falls — falls by where they happen and which way (the operator, 2026-10-09: "the robot's head is tilted up when it enters
the look; it usually results in a fall backwards and the robot loses the chase").  From the host's FULL JSONL: every rescue
(`reset:handoff` / `stop:rescued`), its direction from the trunk's pitch (the body quaternion: nose up = backward) over the
1.5 s before, and whether it came within 4 s of a stop's start (and which stop); per run.
Usage: stop_falls.py "glob" [...]
"""
import collections, glob, json, math, sys

STARTS = ("stop:arrive", "stop:look", "stop:impeded", "stop:stuck", "stop:bylook", "stop:start", "stop:lost")


def pitch_of(q):
    w, x, y, z = q
    return math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))


def main():
    c = collections.Counter(); runs = 0
    for path in sorted(p for a in sys.argv[1:] for p in glob.glob(a)):
        runs += 1
        hist = collections.deque(maxlen=75); stop_start, kind = None, ""
        for line in open(path):
            if not line.startswith('{"t"'):
                continue
            r = json.loads(line); ev = r.get("event") or ""
            if ev in STARTS:
                stop_start, kind = r["t"], ev[5:]
            hist.append(r)
            if ev in ("reset:handoff", "stop:rescued"):
                p = max((pitch_of(h["qpos"][3:7]) for h in hist), key=abs)
                d = "forward" if p > 0 else "backward"
                at = "at a stop's start" if stop_start is not None and r["t"] - stop_start < 4 else "elsewhere"
                c[(at, d)] += 1; c["all"] += 1
    n = max(1, runs)
    print(f"{runs} runs: falls {c['all'] / n:.2f} a run; at a stop's start: backward {c[('at a stop' + chr(39) + 's start', 'backward')] / n:.2f}, "
          f"forward {c[('at a stop' + chr(39) + 's start', 'forward')] / n:.2f}; elsewhere: backward {c[('elsewhere', 'backward')] / n:.2f}, "
          f"forward {c[('elsewhere', 'forward')] / n:.2f}")


if __name__ == "__main__":
    main()
