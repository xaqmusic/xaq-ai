#!/usr/bin/env python3
"""M1 — score the MASK AUTHOR's run: did the slow loop earn any inhibition?

The author (MotorPlanner author_mode=1) proposes region masks, trials each one
under the TRUE dual-cone counterfactual (final vs unmasked-raw verified per tick
on the same arriving reality), and keeps candidates whose whole-body final/raw
error ratio < author_keep_ratio.  The body log's "plan.au" mirror carries the
cumulative trial state; the LAST mirrored line of each run is that run's verdict.

Reported:
  per seed — trials judged, kept count, best ratio, the kept list
  across seeds — REGION RECURRENCE: kept masks grouped by (joint, depth window)
  with overlapping value intervals.  A region kept in most seeds with ratio < 1
  is the M1 positive; an honest empty kept-set everywhere is the M1 null
  (no region-separable excess mass on this vocabulary — conditioning goes deeper
  before authority).

§3.2 consumer checks printed per seed: trials must advance, masked_out must grow.

Usage: authorscore.py '/tmp/xaq_m1a/m1a_s*.log'
"""
import glob, json, statistics, sys
from collections import defaultdict


def last_au(path):
    au, mk, t = None, 0, 0
    for line in open(path):
        line = line.lstrip()
        if not line.startswith("{") or '"plan"' not in line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        p = d.get("plan")
        if isinstance(p, dict) and "au" in p:
            au, mk, t = p["au"], p.get("mk", 0), d.get("t", 0)
    return au, mk, t


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    files = sorted(glob.glob(argv[0]))
    per = []
    for f in files:
        au, mk, t = last_au(f)
        if au is None:
            print(f"{f}: no plan.au mirror — author not wired or run too short")
            continue
        per.append((f, au, mk, t))
    if not per:
        print("no scorable runs")
        return 1

    print(f"authorscore: {len(per)} runs")
    all_kept = []
    for f, au, mk, t in per:
        kept = au.get("kept", [])
        best = f"{min(k['r'] for k in kept):.3f}" if kept else "—"
        ok = "OK" if (au.get("trials", 0) > 0 and mk > 0) else "CONSUMER-FAIL"
        print(f"  {f.split('/')[-1]}: t={t}  trials={int(au.get('trials', 0))}  "
              f"kept={len(kept)}  best_r={best}  masked_out={int(mk)}  [{ok}]")
        for k in kept:
            print(f"      j{k['j']:>2} [{k['lo']:+.2f},{k['hi']:+.2f}] "
                  f"d[{k['dlo']},{k['dhi']}]  r={k['r']:.3f} rt={k['rt']:.3f} "
                  f"n={k['n']}  {'guided' if k.get('g') else 'random'}")
            all_kept.append((f, k))

    # cross-seed recurrence: same joint + same depth window + overlapping values
    groups = defaultdict(list)
    for f, k in all_kept:
        groups[(k["j"], k["dlo"], k["dhi"])].append((f, k))
    print("\nregion recurrence across seeds (joint, depth window):")
    any_rec = False
    for (j, dlo, dhi), members in sorted(groups.items()):
        seeds = {f for f, _ in members}
        lo = max(k["lo"] for _, k in members)
        hi = min(k["hi"] for _, k in members)
        overlap = hi > lo
        if len(seeds) >= 2:
            any_rec = True
            rs = [k["r"] for _, k in members]
            print(f"  j{j} d[{dlo},{dhi}]: {len(seeds)}/{len(per)} seeds, "
                  f"value-overlap={'yes' if overlap else 'NO'} "
                  f"[{lo:+.2f},{hi:+.2f}], mean r={statistics.mean(rs):.3f}")
    if not any_rec:
        print("  none — no region kept in ≥2 seeds")
    print("\nGATE: a region kept in most seeds with overlapping values and r<0.95 is the")
    print("M1 positive.  Kept-empty everywhere = honest null: no region-separable excess")
    print("mass on this vocabulary — vocabulary conditioning precedes inhibitory authority.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
