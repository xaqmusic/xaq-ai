#!/usr/bin/env python3
"""The ToF's registration error under its real timing (--tof-real SPREAD LAG): how far each return is composed from
where it truly was, because the frame's four sub-frames were taken from earlier poses of a moving head.

Reads the host's JSONL "tre" field ([mean, max] of the last cast, m; present only with --tof-real) on cast ticks
(one in four), with the head gyro "hw" (rad/s) as the head's angular speed. Reported per glob:
  - walk and stop: the mean registration error, its p90, and the share of casts over 2 cm (half a block's width);
  - on the walk, the mean error by the head's angular speed (|hw|): what head motion costs the cloud.
Usage: tre_readout.py "glob" [...] [--from S]
"""
import glob, json, math, sys

BINS = [(0.0, 0.3), (0.3, 0.6), (0.6, 1.0), (1.0, 2.0), (2.0, 99.0)]


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else float('nan')


def analyse(pattern, t_from):
    walk, stop = [], []
    by_bin = {b: [] for b in BINS}
    runs = 0
    for path in sorted(glob.glob(pattern)):
        runs += 1
        for line in open(path):
            if not line.startswith('{'):
                continue
            r = json.loads(line)
            if r['t'] < t_from or 'tre' not in r or r['tick'] % 4 != 0:
                continue
            e = r['tre'][0]
            if r['drive'] == 'walk' and r.get('stop', 0) == 0:
                walk.append(e)
                w = math.sqrt(sum(x * x for x in r.get('hw', [0, 0, 0])))
                for b in BINS:
                    if b[0] <= w < b[1]:
                        by_bin[b].append(e)
            elif r['drive'] == 'stand':
                stop.append(e)
    if not walk:
        print(f"{pattern}: no tre records (was --tof-real on?)"); return
    m = lambda xs: sum(xs) / len(xs) if xs else float('nan')
    over = lambda xs: 100.0 * sum(1 for x in xs if x > 0.02) / len(xs) if xs else float('nan')
    print(f"{pattern}  ({runs} runs)")
    print(f"  walk: mean {m(walk) * 100:.2f} cm  p90 {pct(walk, 0.9) * 100:.2f} cm  over 2 cm {over(walk):.0f} %   |   "
          f"stop: mean {m(stop) * 100:.2f} cm  p90 {pct(stop, 0.9) * 100:.2f} cm  over 2 cm {over(stop):.0f} %")
    print("  on the walk by the head's angular speed: " + "  ".join(
        f"{b[0]:.1f}-{b[1] if b[1] < 50 else 'inf'} rad/s: {m(v) * 100:.2f} cm ({100.0 * len(v) / len(walk):.0f} %)" for b, v in by_bin.items() if v))


if __name__ == '__main__':
    args = sys.argv[1:]; t_from = 0.0
    if '--from' in args:
        k = args.index('--from'); t_from = float(args[k + 1]); del args[k:k + 2]
    for g in args:
        analyse(g, t_from)
