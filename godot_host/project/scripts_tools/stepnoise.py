#!/usr/bin/env python3
"""THE NOISY STEP, measured (2026-10-12).  Reads per-tick body logs (OGMA_PICRAWLER_DIAG_INTERVAL=1)
and reports, per arm:

  rev_per_stride   command reversals per gait period (70 ticks) per joint: a reversal is a sign
                   change of the per-tick delta of the commanded target (hip1/hip2/knee arrays)
                   with |delta| above a floor — the servo thrash the ledger names
  flicker_per_min  foot-contact transitions per minute per leg (contact = feet_y < 0.04, the
                   body's own stance test) — the robot's FSR metric, god's-eye in the sim
  clean_strides    strides with a >= 0.25 s swing then a >= 0.25 s stance, as a fraction
  swing_s / stance_s  median bout lengths

Usage: python3 stepnoise.py <label=dir> [label=dir ...]   (each dir holds sa_*.log from arenaavg)
"""
import glob, json, os, statistics, sys

PERIOD = 70; HZ = 50.0; STANCE_TH = 0.04; DELTA_FLOOR = 0.01

def one(path):
    rows = []
    for l in open(path):
        if l.startswith("{") and '"hip1":' in l and '"feet_y":' in l:
            try: rows.append(json.loads(l))
            except Exception: pass
    rows = [r for r in rows if r.get("t", 0) >= 900]
    if len(rows) < 200: return None
    ticks = rows[-1]["t"] - rows[0]["t"]; secs = ticks / HZ
    out = {}
    # reversals per stride per joint
    for j in ("hip1", "hip2", "knee"):
        revs = 0
        for leg in range(4):
            v = [r[j][leg] for r in rows]; prev = 0.0
            for k in range(1, len(v)):
                d = v[k] - v[k-1]
                if abs(d) < DELTA_FLOOR: continue
                if prev != 0.0 and d * prev < 0: revs += 1
                prev = d
        out[f"rev_{j}"] = revs / 4.0 / (ticks / PERIOD)
    # contact flicker and clean strides from feet_y
    flick = []; clean = 0; strides = 0; swings = []; stances = []
    for leg in range(4):
        c = [1 if r["feet_y"][leg] < STANCE_TH else 0 for r in rows]
        trans = sum(1 for k in range(1, len(c)) if c[k] != c[k-1]); flick.append(trans / secs * 60)
        bouts = []; cur = c[0]; L = 1
        for k in range(1, len(c)):
            if c[k] == cur: L += 1
            else: bouts.append((cur, L)); cur = c[k]; L = 1
        bouts.append((cur, L))
        for k in range(1, len(bouts)):
            if bouts[k-1][0] == 0 and bouts[k][0] == 1:
                strides += 1
                if bouts[k-1][1] >= 0.25 * HZ and bouts[k][1] >= 0.25 * HZ: clean += 1
        swings += [L for s, L in bouts if s == 0]; stances += [L for s, L in bouts if s == 1]
    out["flicker_per_min"] = statistics.mean(flick)
    out["clean_frac"] = clean / max(1, strides); out["strides"] = strides
    out["swing_s"] = statistics.median(swings) / HZ if swings else 0.0
    out["stance_s"] = statistics.median(stances) / HZ if stances else 0.0
    return out

if __name__ == "__main__":
    keys = ["rev_hip1", "rev_hip2", "rev_knee", "flicker_per_min", "clean_frac", "strides", "swing_s", "stance_s"]
    print(f"{'arm':14s} | " + " | ".join(f"{k:>15s}" for k in keys))
    for arg in sys.argv[1:]:
        label, d = arg.split("=", 1)
        res = [r for r in (one(p) for p in sorted(glob.glob(os.path.join(d, "sa_*.log")))) if r]
        if not res: print(f"{label:14s} | (no per-tick rows)"); continue
        cells = []
        for k in keys:
            v = [r[k] for r in res]; m = statistics.mean(v); sd = statistics.stdev(v) if len(v) > 1 else 0.0
            cells.append(f"{m:7.3f}±{sd:5.3f}" if k not in ("strides",) else f"{m:9.0f}      ")
        print(f"{label:14s} | " + " | ".join(f"{c:>15s}" for c in cells))
