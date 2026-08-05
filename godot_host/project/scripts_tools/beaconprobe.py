#!/usr/bin/env python3
"""Stage-0 sensing envelope: can the beacon scalar actually be climbed?

Built BEFORE any nav loop, because the loop's failure mode and the sensor's failure mode are
indistinguishable after the fact, and this project has repeatedly spent weeks on a policy
problem that was a conditioning problem (doctrine 6).

WHAT IT ASKS.  Run-and-tumble climbs a scalar by comparing it across a RUN — metres, not
centimetres.  So the question is not "is there a gradient" (there always is) but:

    over one run-length of travel, does the beacon change by more than its own noise?

Two numbers decide it, per range band:
  * d_run   — the mean beacon change produced by RUN_M of closing range
  * sigma   — the sample-to-sample spread of beacon at fixed range (quantisation + pose jitter)
  * SNR = d_run / sigma.  SNR > ~1 means a run is informative; below that the loop is
    integrating noise and no policy can rescue it.

Also reports the QUANTISATION floor: one ray of the grid.  A d_run below one pixel is not a
weak signal, it is no signal — the sensor cannot represent the change at all.

⚠ `tgt_range` in the JSONL is GOD'S-EYE and exists only so this analysis can be done.  It is
diagnostic-only and never reaches the brain.

Usage: beaconprobe.py <log> [<log> ...]   [--run-m 1.0]
"""
import json, math, statistics, sys

RUN_M = 1.0          # metres of travel per run — overridable
BANDS = [(1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 8), (8, 12)]


def load(path):
    rows = []
    for line in open(path):
        if not line.startswith("{") or '"x":' not in line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        if "beacon" in d and "tgt_range" in d:
            rows.append(d)
    return rows


def analyse(path, run_m):
    rows = load(path)
    if len(rows) < 20:
        return None
    wh = rows[0].get("vis_wh", [0, 0])
    npx = max(1, wh[0] * wh[1])
    quantum = 1.0 / npx
    # beacon(r) profile, and the spread within each band
    prof, spread = {}, {}
    for lo, hi in BANDS:
        # ⚠ CONDITION ON VISIBILITY.  Pooling across in-FOV and out-of-FOV mixes two
        # populations and the "noise" is then just the target leaving the 53.5 deg frame —
        # measured sigma > mean at some ranges, which is that artefact, not sensor noise.
        # Run-and-tumble only gets gradient information WHILE the beacon is in view, so the
        # honest question is the SNR conditional on seeing it.
        v = [r["beacon"] for r in rows if lo <= r["tgt_range"] < hi and r["beacon"] > 0.0]
        if len(v) >= 5:
            prof[(lo, hi)] = statistics.mean(v)
            spread[(lo, hi)] = statistics.pstdev(v)
    print(f"\n{path.split('/')[-1]}   grid {wh[0]}x{wh[1]} = {npx} px   "
          f"1 px = {quantum:.5f}   n={len(rows)}   run={run_m} m")
    print(f"  {'range':>8}{'beacon':>10}{'in px':>8}{'sigma':>10}"
          f"{'d_run':>10}{'d_run px':>10}{'SNR':>7}")
    ks = sorted(prof)
    for i, k in enumerate(ks):
        # local slope from the neighbouring band, scaled to one run
        if i + 1 < len(ks):
            k2 = ks[i + 1]
            dr = ((k2[0] + k2[1]) - (k[0] + k[1])) / 2.0
            slope = (prof[k] - prof[k2]) / max(1e-9, dr)      # closing range ⇒ positive
        else:
            slope = 0.0
        d_run = slope * run_m
        snr = d_run / spread[k] if spread[k] > 1e-12 else float("nan")
        flag = ""
        if abs(d_run) < quantum:
            flag = "  << below 1 px — UNREPRESENTABLE"
        elif not math.isnan(snr) and snr < 1.0:
            flag = "  << SNR<1 — a run is not informative"
        print(f"  {k[0]:>3}-{k[1]:<4}{prof[k]:>10.5f}{prof[k]/quantum:>8.1f}"
              f"{spread[k]:>10.5f}{d_run:>10.5f}{d_run/quantum:>10.1f}{snr:>7.2f}{flag}")
    vis = sum(1 for r in rows if r["beacon"] > 0) / len(rows)
    print(f"  beacon visible on {vis*100:.0f}% of samples "
          f"(the rest = target outside the {53.5:.1f}° FOV — direction must be ACTED out)")
    return prof


if __name__ == "__main__":
    argv = sys.argv[1:]
    run_m = RUN_M
    if "--run-m" in argv:
        i = argv.index("--run-m"); run_m = float(argv[i + 1]); del argv[i:i + 2]
    if not argv:
        print(__doc__); sys.exit(2)
    for p in argv:
        analyse(p, run_m)
    print("\nGATE: the bands the arena actually presents must show d_run above 1 px AND SNR>~1."
          "\nFail ⇒ raise vision_res_* or shrink pyramid_max_r BEFORE building the loop.")
