#!/usr/bin/env python3
"""Early-vs-late window comparison WITHIN a single run — the does-it-develop instrument.

Whole-run aggregates average a transient away.  This splits each seed log into windows and
reports per-tick RATES, which turns "does the behaviour improve over the run?" into a number.

Built 2026-08-02 after the operator watched a pure-HK arm morph from convulsions into a crab
walk by ~10k ticks while every seedavg number said "no locomotion" — the campaign had been
running at 6000 ticks, which is the DEPLOYED config's protocol, and was measuring the transient.

It is also the shape the ADAPTATION question needs: a scripted walker shows no early-vs-late
difference; a learning one does.  Pair it with a perturbation (humpavg/recoveravg) to ask
whether the body still changes late in a run.

Usage: windowavg.py <log_dir> <tag_substring> [tag_substring ...]
  Reads the seedavg body logs in <log_dir> (SEEDAVG_OUT), matching *<tag>*.log.

The operator watched convulsions morph into a crab walk by ~10k ticks.  Whole-run
aggregates average that transition away.  This splits each run into windows and reports
per-tick RATES, so "does the behaviour improve over the run?" becomes a number.
"""
import glob, json, math, os, statistics, sys

def windows(path, edges):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("{") or '"x":' not in line:
            continue
        try: d = json.loads(line)
        except Exception: continue
        if "x" in d and "t" in d:
            rows.append(d)
    out = []
    for lo, hi in edges:
        w = [r for r in rows if lo <= r["t"] < hi]
        if len(w) < 3:
            out.append(None); continue
        path_len = sum(math.hypot(w[i]["x"]-w[i-1]["x"], w[i]["z"]-w[i-1]["z"])
                       for i in range(1, len(w)))
        disp = math.hypot(w[-1]["x"]-w[0]["x"], w[-1]["z"]-w[0]["z"])
        dt = w[-1]["t"] - w[0]["t"]
        lifts0 = w[0].get("leg_lifted_counts") or [0,0,0,0]
        lifts1 = w[-1].get("leg_lifted_counts") or [0,0,0,0]
        steps = sum(b-a for a,b in zip(lifts0, lifts1))
        tilts = [r.get("tilt", 0.0) for r in w]
        out.append({
            "disp_rate":  disp/dt*1000,          # metres per 1000 ticks
            "path_rate":  path_len/dt*1000,
            "straight":   disp/path_len if path_len > 1e-9 else 0.0,
            "step_rate":  steps/dt*1000,         # lifts per 1000 ticks
            "tilt_sd":    statistics.pstdev(tilts) if len(tilts) > 1 else 0.0,
            "chassis_y":  statistics.mean(r.get("y", 0.0) for r in w),
        })
    return out

EDGES = [(0, 6000), (7000, 13000), (14000, 20000)]
LBL   = ["EARLY 0-6k", "MID 7-13k", "LATE 14-20k"]

for tag in sys.argv[2:]:
    logs = sorted(glob.glob(os.path.join(sys.argv[1], f"*{tag}*.log")))
    per = [windows(p, EDGES) for p in logs]
    per = [w for w in per if w and all(x is not None for x in w)]
    print(f"\n=== {tag}   (n={len(per)} seeds, rates per 1000 ticks)")
    if not per: print("  no parsable runs"); continue
    keys = ["disp_rate","path_rate","straight","step_rate","tilt_sd","chassis_y"]
    print("  %-11s %s" % ("window", "  ".join(f"{k:>10s}" for k in keys)))
    for i, lbl in enumerate(LBL):
        vals = {k: [w[i][k] for w in per] for k in keys}
        print("  %-11s %s" % (lbl, "  ".join(f"{statistics.mean(vals[k]):10.4f}" for k in keys)))
