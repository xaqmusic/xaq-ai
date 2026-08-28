#!/usr/bin/env python3
"""Step 0 — does the criterion have AUTHORITY over coupling_gain?

    python3 coupling_authority.py <step0 dir>

The ledger's rule, applied INWARD to the criterion: a consumer-fired check proves
the code runs; an authority check proves it can matter.  The GainEvolver does not
reliably discover coupling_gain, and three explanations demand three different
fixes.  This separates them.

Method: the evolver is held in OBSERVER mode (sigma=0) so it scores every window
without searching or publishing, while SETPARAM_AT steps motor_epm.coupling_gain
through six levels, three scored windows per level.  Ascending on half the seeds
and descending on the other half, so hysteresis shows rather than hides.

Windows are attributed to levels by ORDER (window_index // 3), which is exact
because the schedule is deterministic — no reliance on a tick field.

Reads:
  J moves with coupling            -> criterion CAN see it; the problem is search reach
  J flat but FALLS drop            -> benefit is catastrophe avoidance; go lexicographic
  neither moves                    -> coupling does not belong in the searched vector
"""
import glob, json, math, statistics as st, sys

LEVELS_ASC = [0.0, 0.4, 0.8, 1.2, 1.6, 2.0]
WINDOWS_PER_LEVEL = 3
TERMS = [("ge_ji", "J"), ("ge_tilt", "upright sd"), ("ge_unl", "unloaded"),
         ("ge_flow", "flow"), ("ge_energy", "energy"), ("ge_dwell", "dwell")]


def windows(path):
    """One record per SCORED window, in order.  A window boundary is ge_wt resetting.

    ⚠ ATTRIBUTION FIX (2026-08-25, measured on the stage-D1 corpus): the just-closed
    window's ge_* terms appear on the FIRST diag line AFTER the reset — the evolver
    updates inc_terms when the window closes, and the last line BEFORE the reset still
    carries the PREVIOUS window.  The original last-line-before form lagged every
    window by one, diluting level contrast (ordering conclusions survive; exact
    per-level values shift).  Ledger 2026-08-25 D1 entry, bug ★6."""
    rows = []
    for ln in open(path, errors="replace"):
        ln = ln.strip()
        if not ln.startswith("{") or '"ge_wt"' not in ln:
            continue
        try:
            rows.append(json.loads(ln))
        except json.JSONDecodeError:
            continue
    out = []
    for i in range(1, len(rows)):
        prev, d = rows[i - 1], rows[i]
        if int(d.get("ge_wt", -1)) < int(prev.get("ge_wt", -1)) and d.get("ge_ji", -1) > 0:
            cur = dict(d)
            cur["_falls_cum"] = int(d.get("auto_reset_count", 0))
            out.append(cur)
    # per-window falls from the cumulative counter
    for i, w in enumerate(out):
        w["_falls"] = w["_falls_cum"] - (out[i - 1]["_falls_cum"] if i else 0)
    return out


def main(d):
    per_level = {lv: {k: [] for k, _ in TERMS} for lv in LEVELS_ASC}
    for lv in LEVELS_ASC:
        per_level[lv]["falls"] = []
    seeds = 0
    for p in sorted(glob.glob(d + "/s*_*.log")):
        asc = "_asc" in p
        levels = LEVELS_ASC if asc else list(reversed(LEVELS_ASC))
        ws = windows(p)
        if len(ws) < WINDOWS_PER_LEVEL:
            print(f"  {p.split('/')[-1]}: only {len(ws)} scored windows — EXCLUDE")
            continue
        seeds += 1
        for i, w in enumerate(ws):
            li = i // WINDOWS_PER_LEVEL
            if li >= len(levels):
                break
            lv = levels[li]
            for k, _ in TERMS:
                if w.get(k) is not None:
                    per_level[lv][k].append(float(w[k]))
            per_level[lv]["falls"].append(w.get("_falls", 0))
    print(f"{seeds} seeds, {WINDOWS_PER_LEVEL} windows per level\n")

    hdr = f"  {'coupling':>9}" + "".join(f"{lbl:>13}" for _, lbl in TERMS) + f"{'falls':>8}{'n':>5}"
    print(hdr); print("  " + "-" * (len(hdr) - 2))
    for lv in LEVELS_ASC:
        row = f"  {lv:>9.1f}"
        for k, _ in TERMS:
            v = per_level[lv][k]
            row += f"{st.mean(v):>13.4f}" if v else f"{'—':>13}"
        fl = per_level[lv]["falls"]
        row += f"{st.mean(fl):>8.2f}" if fl else f"{'—':>8}"
        row += f"{len(per_level[lv]['ge_ji']):>5}"
        print(row)

    # correlation of each term against coupling level (the authority number)
    print("\n  AUTHORITY — corr(coupling_gain, term) across all windows:")
    verdict = {}
    for k, lbl in TERMS + [("falls", "falls")]:
        xs, ys = [], []
        for lv in LEVELS_ASC:
            for v in per_level[lv][k]:
                xs.append(lv); ys.append(float(v))
        if len(xs) < 6:
            print(f"    {lbl:<12} insufficient data"); continue
        mx, my = st.mean(xs), st.mean(ys)
        num = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
        den = math.sqrt(sum((a - mx) ** 2 for a in xs) * sum((b - my) ** 2 for b in ys))
        r = num / den if den else 0.0
        verdict[lbl] = r
        strength = "STRONG" if abs(r) >= 0.5 else ("weak" if abs(r) >= 0.25 else "none")
        print(f"    {lbl:<12} r = {r:+.3f}   [{strength}]")

    print("\n  READ:")
    rj = abs(verdict.get("J", 0.0)); rf = abs(verdict.get("falls", 0.0))
    if rj >= 0.25:
        print("    J responds to coupling -> the criterion CAN see it.")
        print("    => the problem is SEARCH REACH (dimensionality / generations / margin).")
    elif rf >= 0.25:
        print("    J is flat but FALLS respond -> coupling's benefit is CATASTROPHE")
        print("    AVOIDANCE, which is exactly the signal that left J for the guard.")
        print("    => go LEXICOGRAPHIC: viability first, then quality.")
    else:
        print("    Neither J nor falls respond to coupling in this regime.")
        print("    => coupling does not belong in the SEARCHED vector: a dimension the")
        print("       criterion cannot sense injects noise into every comparison.")
        print("       Hand-set it, drop to a 7-gain evolver, and report that plainly.")
    print("\n  (hysteresis check: compare asc vs desc seeds in the per-level table above —")
    print("   a large asc/desc gap at the same level means order effects, not authority)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
