#!/usr/bin/env python3
"""PART V stage C2 — the criterion's term budget, re-measured on the D1 corpus.

    python3 strido_c2_budget.py <d1 outdir>   # expects old/ and c1/ arm subdirs

The gate-analyze method, applied to the stage-D1 sigma=0 landscape runs:

  NOISE  = spread of a quantity across CONSECUTIVE SAME-LEVEL windows (coupling is
           stepped every 3 windows, so each level contributes 2 same-config pairs
           per seed — the same estimator shape as gate 2's revert pairs).
  SIGNAL = total spread across all windows minus that noise.
  BUDGET = per-term share of var(J) using the WEIGHTED terms and their covariances
           (variance share, never magnitude — the gate-2 lesson).

Also scores the CANDIDATE slip term (window-mean of the body's published
stride_slip, w=0 today) with the same estimator, so its weight is decided by
measurement — or it ships inert, like dwell did.
"""
import glob, json, math, statistics as st, sys

WPL = 3
# weighted terms of J as configured in j1s4/j1s4_c1 (w_falls=w_distress=w_dwell=0)
TERMS = [("ge_tilt", "tilt_sd", 1.0), ("ge_unl", "unloaded", 1.0),
         ("ge_flow", "flow", 1.0), ("ge_energy", "energy", 4.0)]


def windows(path):
    """One record per scored window.  ⚠ ATTRIBUTION, measured 2026-08-25: a window's
    ge_* terms appear on the FIRST diag line AFTER the ge_wt reset that closes it —
    the last line before the reset still carries the PREVIOUS window (the off-by-one
    that also lived in coupling_authority.py).  stride_slip is averaged over the
    closing window's own lines (between the two resets) and attached to its record."""
    rows = []
    for ln in open(path, errors="replace"):
        ln = ln.strip()
        if not ln.startswith("{") or '"ge_wt"' not in ln:
            continue
        try:
            rows.append(json.loads(ln))
        except json.JSONDecodeError:
            continue
    out, slip_acc = [], []
    for i in range(1, len(rows)):
        prev, d = rows[i - 1], rows[i]
        if "stride_slip" in prev:
            slip_acc.append(float(prev["stride_slip"]))
        if int(d.get("ge_wt", -1)) < int(prev.get("ge_wt", -1)):
            if d.get("ge_ji", -1) > 0:
                rec = dict(d)
                rec["slip_mean"] = st.mean(slip_acc) if slip_acc else float("nan")
                out.append(rec)
            slip_acc = []
    return out


def sig_noise(vals):
    """(signal_sd, noise_sd) from a per-window series with 3 windows per level:
    noise from same-level consecutive pairs, total from everything."""
    diffs = []
    for i in range(len(vals) - 1):
        if (i % WPL) != WPL - 1:               # i and i+1 share a level
            diffs.append(vals[i + 1] - vals[i])
    noise_var = st.variance(diffs) / 2.0 if len(diffs) > 1 else float("nan")
    total_var = st.variance(vals) if len(vals) > 1 else float("nan")
    sig_var = max(0.0, total_var - noise_var) if total_var == total_var else float("nan")
    return math.sqrt(sig_var), math.sqrt(noise_var) if noise_var == noise_var else float("nan")


def main(d):
    for arm in ("old", "c1"):
        rows_all = []
        for p in sorted(glob.glob(f"{d}/{arm}/s*_*.log")):
            ws = windows(p)
            rows_all.append(ws)
        if not rows_all:
            print(f"{arm}: no logs"); continue
        print(f"\n=== arm {arm} ({len(rows_all)} seeds, "
              f"{[len(w) for w in rows_all]} windows) ===")
        # per-term signal/noise + weighted variance share of J
        flat = [w for ws in rows_all for w in ws]
        J = [float(w["ge_ji"]) for w in flat]
        print(f"  J: mean {st.mean(J):.3f}  var {st.variance(J):.5f}")
        wt_series = {}
        for key, lbl, wgt in TERMS:
            v = [wgt * float(w.get(key, 0.0)) for w in flat]
            wt_series[lbl] = v
        var_J = st.variance(J)
        print(f"  {'term':<10}{'w·mean':>9}{'var share':>11}{'signal':>9}{'noise':>9}{'sig/noise':>10}")
        for key, lbl, wgt in TERMS:
            v = wt_series[lbl]
            # variance share = cov(term, J)/var(J) — includes covariance effects
            mv, mj = st.mean(v), st.mean(J)
            cov = sum((a - mv) * (b - mj) for a, b in zip(v, J)) / (len(v) - 1)
            share = cov / var_J if var_J else float("nan")
            # signal/noise measured per seed then pooled (levels differ per seed order)
            sigs, noises = [], []
            for ws in rows_all:
                s, n = sig_noise([wgt * float(w.get(key, 0.0)) for w in ws])
                if s == s: sigs.append(s)
                if n == n: noises.append(n)
            sg, nz = (st.mean(sigs) if sigs else float("nan")), (st.mean(noises) if noises else float("nan"))
            print(f"  {lbl:<10}{st.mean(v):>9.4f}{share:>11.3f}{sg:>9.4f}{nz:>9.4f}"
                  f"{(sg/nz if nz and nz == nz and nz > 0 else float('nan')):>10.2f}")
        # the candidate slip term, measured the same way (unweighted; it has no weight yet)
        sl = [w.get("slip_mean", float("nan")) for w in flat]
        sl = [x for x in sl if x == x]
        if sl:
            sigs, noises = [], []
            for ws in rows_all:
                vals = [w.get("slip_mean", float("nan")) for w in ws]
                if any(v != v for v in vals):
                    continue
                s, n = sig_noise(vals)
                if s == s: sigs.append(s)
                if n == n: noises.append(n)
            sg = st.mean(sigs) if sigs else float("nan")
            nz = st.mean(noises) if noises else float("nan")
            print(f"  {'slip cand.':<10}{st.mean(sl):>9.4f}{'—':>11}{sg:>9.4f}{nz:>9.4f}"
                  f"{(sg/nz if nz and nz == nz and nz > 0 else float('nan')):>10.2f}")
        print("  (share = cov(w·term, J)/var(J); shares sum to ~1 over the weighted terms)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    main(sys.argv[1])
