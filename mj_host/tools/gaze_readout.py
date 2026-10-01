#!/usr/bin/env python3
"""Does the head look at what the walk is going to? (2026-10-01, the gaze leading the turn, --seek-gaze)

On walking ticks where the seek loop holds the heading reference (steer 3), the target's bearing relative to the body is
the reference minus the heading ("hdg": [heading, reference], + = left), and relative to the HEAD it is that minus the
head's yaw joint (q[7], + = left, home 0). The ToF sees +-22.5 deg. Reported per glob:
  - seek-steered seconds a run, and the share of them with the target inside the ToF's field: by the body (a still head
    would see it) and by the head (what the head actually saw);
  - the head's yaw on the walk: its median |angle| and the share past 0.3 rad, while seeking and otherwise;
  - the target's bearing error (body) median, the turn it asks for.
Usage: gaze_readout.py "glob" [...]   (walking ticks after 30 s)
"""
import glob, json, math, sys

HALF = math.radians(22.5)


def wrap(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else float('nan')


for g in sys.argv[1:]:
    runs = 0; seek = 0; body_in = 0; head_in = 0; yaw_seek = []; yaw_other = []; err = []
    for path in sorted(glob.glob(g)):
        runs += 1
        for line in open(path):
            if not line.startswith('{'):
                continue
            r = json.loads(line)
            if r['t'] < 30 or r['drive'] != 'walk' or r.get('stop', 0) != 0:
                continue
            yaw = r['q'][7]
            if r.get('steer') == 3 and 'hdg' in r:
                rel = wrap(r['hdg'][1] - r['hdg'][0])
                seek += 1; err.append(abs(rel)); yaw_seek.append(abs(yaw))
                body_in += abs(rel) < HALF
                head_in += abs(wrap(rel - yaw)) < HALF
            else:
                yaw_other.append(abs(yaw))
    if not seek:
        print(f"{g}: no seek-steered walking ticks"); continue
    n = max(1, runs)
    print(f"{g}  ({runs} runs): seek steers {seek * 0.02 / n:.0f} s a run; the target in the ToF's field by the body "
          f"{100 * body_in / seek:.0f} %, by the head {100 * head_in / seek:.0f} %; bearing error median {math.degrees(pct(err, 0.5)):.0f} deg")
    print(f"  head yaw |angle| median: seeking {pct(yaw_seek, 0.5):.2f} rad ({100 * sum(1 for y in yaw_seek if y > 0.3) / len(yaw_seek):.0f} % past 0.3), "
          f"otherwise {pct(yaw_other, 0.5):.2f} rad ({100 * sum(1 for y in yaw_other if y > 0.3) / max(1, len(yaw_other)):.0f} % past 0.3)")
