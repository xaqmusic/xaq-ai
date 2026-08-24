#!/usr/bin/env python3
"""target_accept x step-size sweep, judged on measured dJ AND band re-entry.

    python3 gainevo_tsweep.py <tsweep dir>

Every arm starts from the SAME displaced point, with coupling_gain at 0.30 — inside
its measured BAD band (0-0.8) — while amp_target and postural_gain start inside
their good bands. Coupling is the gain whose optimum replicated from two independent
operating points, so it is the one with the most credible gradient to climb.

A pass needs BOTH halves to agree:
  * J FALLS across generations, and
  * coupling RE-ENTERS 1.2-2.0.
J falling alone can mean the search improved something other than the displaced
gain; band re-entry alone can be drift that happened to go the right way. Only the
conjunction is evidence the search followed its own criterion.

Acceptance rate is reported but is NOT a verdict: with target_accept on AUTO the
anneal servos sigma until acceptance equals the noise floor, so on those arms the
rate is a controller setpoint rather than a measurement of the search.
"""
import glob, json, math, os, statistics as st, sys

COUPLING_IDX = 1                  # gain_keys = [amp_target, coupling_gain, postural_gain]
GOOD_LO, GOOD_HI = 1.2, 2.0       # coupling's measured good band
ARMS = ["auto", "tgt020", "tgt070", "fix003", "fix008", "fix020", "fix045"]


def series(p):
    """(gen, J, vec, sigma, accepts) per generation, first sighting of each gen."""
    out, seen = [], set()
    for ln in open(p, errors="replace"):
        if '"ge_gen"' not in ln:
            continue
        try:
            d = json.loads(ln)
        except json.JSONDecodeError:
            continue
        g = int(d.get("ge_gen", -1)); j = float(d.get("ge_ji", -1))
        if g < 0 or g in seen:
            continue
        if j > 0:
            seen.add(g)
            out.append((g, j, d.get("ge_vec"), float(d.get("ge_sig", 0)),
                        int(d.get("ge_acc", 0))))
    return out


def slope(xs, ys):
    mx, my = st.mean(xs), st.mean(ys)
    den = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den if den else 0.0


def main(d):
    print("start: amp 0.385 | coupling 0.30 (BAD band 0-0.8) | postural 1.092")
    print(f"pass = J falls AND coupling re-enters [{GOOD_LO}, {GOOD_HI}]\n")
    hdr = (f"  {'arm':<9}{'n':>3}{'dJ':>9}{'slope/gen':>11}{'t(dJ)':>7}"
           f"{'coupling end':>14}{'re-entered':>12}{'sigma':>8}{'acc rate':>10}")
    print(hdr); print("  " + "-" * (len(hdr) - 2))
    rows = {}
    for arm in ARMS:
        dJs, slopes, cends, sigs, rates, gens = [], [], [], [], [], []
        for p in sorted(glob.glob(f"{d}/{arm}_s*.log")):
            s = series(p)
            if len(s) < 6:
                continue
            xs = [g for g, *_ in s]; ys = [j for _, j, *_ in s]
            k = max(2, len(ys) // 3)
            dJs.append(st.mean(ys[-k:]) - st.mean(ys[:k]))
            slopes.append(slope(xs, ys))
            last_vec = next((v for _, _, v, _, _ in reversed(s) if v), None)
            if last_vec:
                cends.append(float(last_vec[COUPLING_IDX]))
            sigs.append(s[-1][3]); gens.append(s[-1][0])
            rates.append(s[-1][4] / s[-1][0] if s[-1][0] else float("nan"))
        if not dJs:
            print(f"  {arm:<9}  no data"); continue
        m = st.mean(dJs)
        sd = st.stdev(dJs) if len(dJs) > 1 else 0.0
        t = (m / (sd / math.sqrt(len(dJs)))) if sd > 1e-12 else float("nan")
        if sd <= 1e-12 and len(dJs) > 1:
            print(f"  !! {arm}: {len(dJs)} runs are IDENTICAL — replicates did not vary (check OGMA_SEED)")
        reent = sum(1 for c in cends if GOOD_LO <= c <= GOOD_HI)
        cend_txt = ", ".join(f"{c:.2f}" for c in cends)
        print(f"  {arm:<9}{len(dJs):>3}{m:>+9.3f}{st.mean(slopes):>+11.5f}{t:>+7.2f}"
              f"{cend_txt:>14}{reent:>8}/{len(cends):<3}{st.mean(sigs):>8.3f}{st.mean(rates):>10.3f}")
        rows[arm] = (m, t, reent, len(cends), st.mean(sigs))
    print("\n  dJ = mean J of the last third minus the first third; NEGATIVE = improving.")
    print("  |t| > ~2.9 is p<0.05 at n=3, so n=3 can only surface a LOUD effect — which is")
    print("  the intent: a real capability should not need heavy averaging to appear.\n")
    passed = [a for a, (m, t, r, n, _) in rows.items() if m < 0 and t < -2.0 and r > n / 2]
    if passed:
        print(f"  ARMS PASSING BOTH HALVES: {', '.join(passed)}")
    else:
        print("  NO ARM PASSES BOTH HALVES.")
        best = min(rows.items(), key=lambda kv: kv[1][0]) if rows else None
        if best:
            print(f"  Closest on dJ: {best[0]} ({best[1][0]:+.3f}, t={best[1][1]:+.2f}, "
                  f"re-entry {best[1][2]}/{best[1][3]})")
    # does target_accept act only through sigma?  the fix* arms pin sigma, so if the
    # tgt* arms land on the same sigma as a fix* arm AND the same dJ, it is a sigma knob
    print("\n  target_accept as a sigma selector — realized sigma by arm:")
    for a in ("tgt020", "auto", "tgt070"):
        if a in rows:
            print(f"    {a:<8} settled sigma {rows[a][4]:.3f}")
    print("    (compare against the pinned fix* arms above: if the tgt* arms simply land")
    print("     on a fix* sigma and inherit its dJ, target_accept is a step-size knob.)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
