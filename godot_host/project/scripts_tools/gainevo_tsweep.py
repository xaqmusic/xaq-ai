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
# sigma0 is the CONTROL and every searching arm is judged against it — the first
# committed version omitted it from this list, so the analyzer could not produce
# the comparison its own docstring promises.  tgt* arms were dropped after being
# measured as TAUTOLOGY (a sigma selector); fix003/fix045 after being measured.
ARMS = ["sigma0", "auto", "fix008", "fix020",
        "tgt020", "tgt070", "fix003", "fix045"]
CONTROL = "sigma0"


def series(p):
    """(idx, J, vec, sigma, accepts, gen) per SCORED WINDOW — a boundary is the
    ge_wt counter resetting, the same rule gainevo_landscape.py uses.

    Window-based, not generation-based, because the sigma0 CONTROL never
    increments ge_gen (a silent observer has no generations) yet scores every
    window — the first committed version indexed by ge_gen and could not read
    the control arm at all.  It also matches the registered estimator's own
    words: 'last third of SCORED WINDOWS minus the first third'."""
    out, prev, cur = [], None, None
    def emit(d):
        if d and float(d.get("ge_ji", -1)) > 0:
            out.append((len(out), float(d["ge_ji"]), d.get("ge_vec"),
                        float(d.get("ge_sig", 0)), int(d.get("ge_acc", 0)),
                        int(d.get("ge_gen", 0))))
    for ln in open(p, errors="replace"):
        if '"ge_wt"' not in ln:
            continue
        try:
            d = json.loads(ln)
        except json.JSONDecodeError:
            continue
        wt = int(d.get("ge_wt", -1))
        if prev is not None and wt < prev:
            emit(cur)
        prev = wt
        cur = d
    emit(cur)
    return out


def slope(xs, ys):
    mx, my = st.mean(xs), st.mean(ys)
    den = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den if den else 0.0


def main(d):
    print("start: amp 0.385 | coupling 0.30 (BAD band 0-0.8) | postural 1.092")
    print(f"pass = J falls AND coupling re-enters [{GOOD_LO}, {GOOD_HI}]\n")
    hdr = (f"  {'arm':<9}{'n':>3}{'dJ':>9}{'slope/win':>11}{'t(dJ)':>7}"
           f"{'coupling end':>14}{'re-entered':>12}{'sigma':>8}{'acc rate':>10}")
    print(hdr); print("  " + "-" * (len(hdr) - 2))
    rows, arm_dJs = {}, {}
    for arm in ARMS:
        dJs, slopes, cends, sigs, rates, gens = [], [], [], [], [], []
        for p in sorted(glob.glob(f"{d}/{arm}_s*.log")):
            s = series(p)
            if len(s) < 6:
                continue
            xs = [w for w, *_ in s]; ys = [j for _, j, *_ in s]
            k = max(2, len(ys) // 3)
            dJs.append(st.mean(ys[-k:]) - st.mean(ys[:k]))
            slopes.append(slope(xs, ys))
            last_vec = next((v for _, _, v, _, _, _ in reversed(s) if v), None)
            if last_vec:
                cends.append(float(last_vec[COUPLING_IDX]))
            sigs.append(s[-1][3]); gens.append(s[-1][5])
            # acceptance rate is accepts per GENERATION (the sigma0 control has
            # no generations; its rate is undefined, not zero)
            rates.append(s[-1][4] / s[-1][5] if s[-1][5] else float("nan"))
        if not dJs:
            print(f"  {arm:<9}  no data"); continue
        m = st.mean(dJs)
        sd = st.stdev(dJs) if len(dJs) > 1 else 0.0
        t = (m / (sd / math.sqrt(len(dJs)))) if sd > 1e-12 else float("nan")
        if sd <= 1e-12 and len(dJs) > 1:
            print(f"  !! {arm}: {len(dJs)} runs are IDENTICAL — replicates did not vary (check OGMA_SEED)")
        reent = sum(1 for c in cends if GOOD_LO <= c <= GOOD_HI)
        cend_txt = ",".join(f"{c:.2f}" for c in cends)
        print(f"  {arm:<9}{len(dJs):>3}{m:>+9.3f}{st.mean(slopes):>+11.5f}{t:>+7.2f}"
              f"  {cend_txt:<30}{reent:>4}/{len(cends):<3}{st.mean(sigs):>8.3f}{st.mean(rates):>10.3f}")
        rows[arm] = (m, t, reent, len(cends), st.mean(sigs))
        arm_dJs[arm] = dJs
    print("\n  dJ = mean J of the last third minus the first third; NEGATIVE = improving.")

    # ---- the comparison that carries the verdict: each searching arm vs the ----
    # ---- sigma0 control (Welch t).  dJ falling in isolation is unreadable —  ----
    # ---- the control is what separates search from settling.                 ----
    ctrl = arm_dJs.get(CONTROL)
    vs = {}
    if ctrl and len(ctrl) > 1:
        print(f"  vs control ({CONTROL}: dJ {st.mean(ctrl):+.3f} ± {st.stdev(ctrl):.3f}, "
              f"n={len(ctrl)}):")
        for arm, dJs in arm_dJs.items():
            if arm == CONTROL or len(dJs) < 2:
                continue
            va, vc = st.variance(dJs), st.variance(ctrl)
            se = math.sqrt(va / len(dJs) + vc / len(ctrl))
            tw = (st.mean(dJs) - st.mean(ctrl)) / se if se > 1e-12 else float("nan")
            vs[arm] = tw
            print(f"    {arm:<9} dJ-dJ_ctrl {st.mean(dJs)-st.mean(ctrl):+7.3f}   Welch t {tw:+6.2f}")
    else:
        print(f"  ⚠ NO {CONTROL} CONTROL DATA — dJ columns above cannot separate search")
        print("    from settling; do not read them as a verdict on the search.")
    passed = [a for a, (m, t, r, n, _) in rows.items()
              if a != CONTROL and r > n / 2 and vs.get(a, t) < -2.0]
    if passed:
        print(f"\n  ARMS PASSING BOTH HALVES (dJ beats control AND band re-entry): "
              f"{', '.join(passed)}")
    else:
        print("\n  NO ARM PASSES BOTH HALVES.")
        cand = {a: v for a, v in rows.items() if a != CONTROL}
        best = min(cand.items(), key=lambda kv: kv[1][0]) if cand else None
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
