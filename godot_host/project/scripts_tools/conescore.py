#!/usr/bin/env python3
"""M0.b — read the MotorPlanner's probability-cone verification scores (PART III).

The planner verifies ITSELF online: every tick it rolls the phase-conditioned cone
from the t0 row (the actual present) and stores the probe-depth rows; when the
future arrives it scores them.  The body log's "plan" mirror carries the CUMULATIVE
per-depth means, so the LAST mirrored line of each run is that run's verdict.

Reported per depth (mean ± std over runs):
  top1  — P(cone argmax == actual token)      [the honest hit rate]
  topk  — P(actual token is IN the beam)      [did the cone even contain reality?]
  mass  — mean probability the cone assigned the actual token  [calibration-ish]
  ent   — mean cone entropy (nats)            [how committed the cone is]
Baselines: marg = argmax-of-marginal hit rate (context-free); persistence is the
M0 number the gate is judged against (self-transition mass 0.72 per-tick).

Usage: conescore.py '/tmp/xaq_m0b/m0b_s*.log'
"""
import glob, json, math, statistics, sys


def last_plan(path, warmup=3000):
    """Final cumulative plan mirror + the PERSISTENCE baseline at each probe depth,
    computed from the same run's per-tick bp_win stream (post-warmup):
    persist[k] = P(s_{t+k} == s_t) — the M0 degenerate attractor the cone must beat."""
    plan, nline, toks = None, 0, []
    for line in open(path):
        line = line.lstrip()
        if not line.startswith("{") or '"plan"' not in line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "plan" in d:
            plan, nline = d["plan"], d.get("t", 0)
        if d.get("t", 0) >= warmup and d.get("bp_win", -1) >= 0:
            toks.append(d["bp_win"])
    persist = {}
    if plan and len(toks) > 200:
        for k in (int(x) for x in plan["d"]):
            n = len(toks) - k
            persist[k] = sum(1 for i in range(n) if toks[i] == toks[i + k]) / max(1, n)
    return plan, nline, persist


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    runs = []
    for f in sorted(glob.glob(argv[0])):
        p, t, per = last_plan(f)
        if p is None:
            print(f"  {f}: NO plan mirror — consumer never fired")
            continue
        runs.append((f, p, t, per))
        print(f"  {f}: last mirror t={t}  n_obs={p['obs']}  mask_mode={p['mm']}  "
              f"masked_out={p['mk']}")
    if not runs:
        print("no scorable runs")
        return 1
    depths = runs[0][1]["d"]
    print(f"\nconescore: {len(runs)} runs   "
          f"marginal-argmax baseline {statistics.mean(r[1]['mg'] for r in runs):.3f}")
    print(f"{'depth':>6}{'n':>8}{'top1':>14}{'persist':>9}{'lift':>6}"
          f"{'topk':>14}{'mass':>14}{'ent':>10}")
    for i, k in enumerate(depths):
        def col(key):
            vs = [r[1][key][i] for r in runs]
            return statistics.mean(vs), (statistics.stdev(vs) if len(vs) > 1 else 0.0)
        n = statistics.mean(r[1]["n"][i] for r in runs)
        t1, t1s = col("t1"); tk, tks = col("tk"); ms, mss = col("ms"); en, _ = col("en")
        pers = [r[3].get(int(k)) for r in runs if r[3].get(int(k)) is not None]
        pm = statistics.mean(pers) if pers else float("nan")
        lift = t1 / pm if pers and pm > 0 else float("nan")
        print(f"{k:>6}{n:>8.0f}{t1:>8.3f} ±{t1s:.3f}{pm:>9.3f}{lift:>6.2f}"
              f"{tk:>8.3f} ±{tks:.3f}{ms:>8.3f} ±{mss:.3f}{en:>10.2f}")
    print("\nM0 reference (UNconditioned chain, same protocol): event-space 3.9x chance"
          "\nat k=1 decaying below baseline by k=5; per-tick self-transition mass 0.72."
          "\nGATE: phase conditioning must hold top1 above the marginal baseline AND above"
          "\npersistence-shaped decay at k>=5, or vocabulary conditioning precedes M1.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
