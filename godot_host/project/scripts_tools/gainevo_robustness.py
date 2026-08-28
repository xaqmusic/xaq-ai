#!/usr/bin/env python3
"""Long-run robustness: does a LIVE GainEvolver damage the gait over time?

    python3 gainevo_robustness.py <live dir> <frozen dir>

The question is not "is the end state good" — a gait that gets good and then rots
would pass that. It is whether quality DEGRADES across the run, and whether any
degradation belongs to the search or to the body. Both arms run the same config
and differ only in mutation_sigma, so the frozen arm is the same body doing the
same thing with no search: it supplies the drift baseline.

Everything is reported per QUARTER of the run. Displacement metrics are omitted
on purpose — outer-wall recentering teleports the body, so net_disp/straight are
meaningless here; gait QUALITY is what carries the answer.
"""
import glob, json, math, statistics as st, sys

QUARTERS = 4
# (log key, label, "lower"|"higher" is better)
METRICS = [("tilt", "|tilt|", "lower"),
           ("fwd_v", "fwd_v", "higher"),
           ("e_avg", "energy", "lower"),
           ("ge_ji", "criterion J", "lower")]


def per_quarter(path):
    """Return (quarters, falls_per_quarter, rear_rate) from one run."""
    rows = []
    for ln in open(path, errors="replace"):
        ln = ln.strip()
        if not ln.startswith("{") or '"tilt"' not in ln:
            continue
        try:
            rows.append(json.loads(ln))
        except json.JSONDecodeError:
            continue
    if len(rows) < 4 * QUARTERS:
        return None
    n = len(rows) // QUARTERS
    out, falls, rear = [], [], []
    for q in range(QUARTERS):
        blk = rows[q * n:(q + 1) * n]
        vals = {}
        for k, _, _ in METRICS:
            xs = [abs(float(r[k])) for r in blk if isinstance(r.get(k), (int, float))
                  and not (k == "ge_ji" and float(r[k]) < 0)]
            vals[k] = st.mean(xs) if xs else float("nan")
        out.append(vals)
        # falls: the body's cumulative counter differenced across the block
        a = [int(r.get("auto_reset_count", 0)) for r in blk if "auto_reset_count" in r]
        falls.append((a[-1] - a[0]) if len(a) >= 2 else 0)
        # rear-landing consumer ACTIVITY RATE (must not collapse as the search runs)
        rl = [int(r.get("rlt", 0)) for r in blk if "rlt" in r]
        rear.append((rl[-1] - rl[0]) if len(rl) >= 2 else 0)
    return out, falls, rear


def arm(d):
    runs = [per_quarter(p) for p in sorted(glob.glob(d + "/*.log"))]
    runs = [r for r in runs if r]
    if not runs:
        return None
    q = []
    for i in range(QUARTERS):
        row = {}
        for k, _, _ in METRICS:
            xs = [r[0][i][k] for r in runs if r[0][i][k] == r[0][i][k]]
            row[k] = st.mean(xs) if xs else float("nan")
        row["falls"] = st.mean([r[1][i] for r in runs])
        row["rear"] = st.mean([r[2][i] for r in runs])
        q.append(row)
    return q, len(runs)


def main(live_dir, froz_dir):
    L, R = arm(live_dir), arm(froz_dir)
    if not L or not R:
        print("missing data"); return
    (lq, ln_), (rq, rn) = L, R
    # State the tick span the quarters actually cover: this script quarters the
    # rows PRESENT, so on a partial run "Q4" is the last quarter of what has been
    # written so far, not of the intended run.  Reading a mid-run Q4 as the end
    # state is an easy and expensive mistake.
    def span(d):
        best = 0
        for p_ in glob.glob(d + "/*.log"):
            c = sum(1 for ln in open(p_, errors="replace")
                    if ln.startswith('{') and '"tilt"' in ln)
            best = max(best, c)
        return best * 60      # diag emits every 60 ticks
    print(f"LIVE n={ln_}   FROZEN n={rn}")
    print(f"quarters cover ~{span(live_dir)} ticks (live) / ~{span(froz_dir)} ticks (frozen)")
    print("⚠ if that is short of the intended run, Q4 is NOT the end state\n")
    for key, label, better in METRICS + [("falls", "falls/quarter", "lower"),
                                         ("rear", "rear-land events", "higher")]:
        print(f"  {label}")
        for tag, qs in (("live  ", lq), ("frozen", rq)):
            print(f"    {tag} " + "  ".join(f"Q{i+1} {qs[i][key]:8.4f}" for i in range(QUARTERS)))
        # degradation = last quarter vs first, for each arm
        dl = lq[-1][key] - lq[0][key]
        dr = rq[-1][key] - rq[0][key]
        worse_l = (dl > 0) if better == "lower" else (dl < 0)
        # the number that matters: did LIVE drift worse than FROZEN?
        rel = dl - dr if better == "lower" else dr - dl
        verdict = ("LIVE degrades MORE than frozen" if rel > 0 and worse_l else
                   "live no worse than frozen")
        print(f"    Q4-Q1  live {dl:+.4f}   frozen {dr:+.4f}   -> {verdict}\n")
    print("READ: the search is safe if LIVE's Q1->Q4 drift is no worse than FROZEN's.")
    print("      Frozen drifting too = the BODY's doing, not the evolver's.")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
