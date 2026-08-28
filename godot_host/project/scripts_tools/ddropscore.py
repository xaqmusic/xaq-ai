#!/usr/bin/env python3
"""Lever (b) (d)-test — drop the plan pull mid-run, watch for re-coordination.

Arms (both 16k ticks, matched seeds):
  BASE — planpull, gain 0.1 throughout.
  FLIP — identical, but SETPARAM_AT zeroes plan_gain at tick 10000.

Reports per 2k-tick window (mean ± std over seeds, FLIP vs BASE):
  disp  — net XZ displacement in the window (m)
  steps — total leg-lift events in the window
  plant — mean planted-feet count
  tilt  — tilt sd in the window
  pull  — mean pl_pull (the consumer meter: MUST drop to ~0 after the flip in
          the FLIP arm and stay level in BASE — the §3.2 flip-fired check)

The (d) signature: a transient degradation in the first post-flip window that
RECOVERS by the later windows (re-coordination without the pull).  A flat table
means the pull was not load-bearing at this dose; a persistent drop means the
body had come to DEPEND on it (entrenchment, the arm-seed-2 risk).

Usage: ddropscore.py '/tmp/xaq_dt_flip/sa_*.log' '/tmp/xaq_dt_base/sa_*.log' [flip_tick=10000]
"""
import glob, json, re, statistics, sys

WINDOWS = [(6000, 8000), (8000, 10000), (10000, 12000), (12000, 14000), (14000, 16000)]


def parse(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if '"x":' not in line or not line.startswith('{'):
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if 'x' not in d:
            continue
        ll = d.get('leg_lifted_counts')
        rows.append((d.get('t', 0), d['x'], d['z'],
                     sum(ll) if isinstance(ll, list) else None,
                     sum(1 for v in d.get('feet_y', []) if v < 0.04)
                     if isinstance(d.get('feet_y'), list) else None,
                     d.get('tilt', 0.0), d.get('pl_pull', 0.0)))
    return rows


def window_stats(rows, lo, hi):
    w = [r for r in rows if lo <= r[0] < hi]
    if len(w) < 5:
        return None
    disp = ((w[-1][1] - w[0][1]) ** 2 + (w[-1][2] - w[0][2]) ** 2) ** 0.5
    steps = (w[-1][3] - w[0][3]) if (w[0][3] is not None and w[-1][3] is not None) else float('nan')
    plant = statistics.mean(r[4] for r in w if r[4] is not None)
    tilt = statistics.pstdev([r[5] for r in w])
    pull = statistics.mean(r[6] for r in w)
    return disp, steps, plant, tilt, pull


def arm(pattern):
    out = {}
    for f in sorted(glob.glob(pattern)):
        m = re.search(r"_s(\d+)\.log$", f)
        if not m:
            continue
        rows = parse(f)
        out[m.group(1)] = [window_stats(rows, lo, hi) for lo, hi in WINDOWS]
    return out


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    flip, base = arm(argv[0]), arm(argv[1])
    seeds = sorted(set(flip) & set(base))
    if not seeds:
        print("no matched seeds")
        return 1
    flip_tick = int(argv[2]) if len(argv) > 2 else 10000
    print(f"ddropscore: {len(seeds)} matched seeds, flip at t={flip_tick}")
    names = ["disp", "steps", "plant", "tilt", "pull"]
    for mi, name in enumerate(names):
        print(f"\n  {name}   " + "".join(f"{lo//1000}-{hi//1000}k".rjust(11) for lo, hi in WINDOWS))
        for label, data in (("FLIP", flip), ("BASE", base)):
            row = f"  {label:>5}"
            for wi in range(len(WINDOWS)):
                vals = [data[s][wi][mi] for s in seeds
                        if data[s][wi] is not None and data[s][wi][mi] == data[s][wi][mi]]
                row += (f"{statistics.mean(vals):>7.2f}±{statistics.pstdev(vals):<3.2f}"
                        if vals else "      —    ")
            print(row)
    print("\nCHECKS: FLIP pull must fall to ~0 from the 10-12k window (flip fired);")
    print("BASE pull must stay level.  The (d) signature = FLIP dips vs BASE in")
    print("10-12k then re-converges by 14-16k.  Flat everywhere = pull not load-")
    print("bearing at this dose; persistent FLIP deficit = dependence/entrenchment.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
