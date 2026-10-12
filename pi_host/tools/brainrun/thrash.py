#!/usr/bin/env python3
"""THE NOISY STEP on the robot: the brain's RAW action targets (ogma_host --dump-actions, one
{"kind":"actions","tick":t,"u":[12],"us":[12]} row per tick, before benchd's slew) against the
feed's applied pulses, and the FSR contact waveform.

  rev_per_stride_raw   sign changes of the per-tick delta of the raw target pulse per servo,
                       per 70-tick gait period, |delta| >= 20 us
  rev_per_stride_out   the same on benchd's applied `out` pulses (the feed)
  flicker_per_min      FSR contact (>= 200) transitions per minute per leg
  clean_frac           strides with a >= 0.25 s swing then a >= 0.25 s stance

Usage: python3 thrash.py <host.log> [feed.jsonl]
"""
import json, statistics, sys
PERIOD = 70; HZ = 50.0

def reversals(series_list, floor):
    revs = 0; n = 0
    for v in series_list:
        prev = 0
        for k in range(1, len(v)):
            d = v[k] - v[k-1]
            if abs(d) < floor: continue
            if prev != 0 and d * prev < 0: revs += 1
            prev = d
        n += len(v)
    return revs / max(1, len(series_list)) / max(1.0, (n / max(1, len(series_list))) / PERIOD)

host = sys.argv[1]
acts = []
for l in open(host):
    if l.startswith("{") and '"kind":"actions"' in l:
        try: acts.append(json.loads(l))
        except Exception: pass
print(f"raw action rows: {len(acts)}")
if acts:
    raw = [[a["us"][c] for a in acts] for c in range(12)]
    print(f"rev_per_stride_raw (per servo, |d|>=20us): {reversals(raw, 20):.2f}")
    sat = statistics.mean(1 if abs(a["u"][c]) >= 0.99 else 0 for a in acts for c in range(12))
    print(f"raw |u| at the rail (>=0.99): {sat:.2f} of servo-ticks")
if len(sys.argv) > 2:
    rows = [json.loads(l) for l in open(sys.argv[2]) if l.startswith("{")]
    br = [d for d in rows if d.get("mode") != "bench" and not d.get("stopped") and "out" in d and "fsr" in d]
    out = [[d["out"][c] for d in br] for c in range(12)]
    print(f"rev_per_stride_out (benchd's applied pulses): {reversals(out, 20):.2f}")
    secs = (br[-1]["t"] - br[0]["t"]) / 1000.0; flick = []; clean = 0; strides = 0
    for leg in range(4):
        c = [1 if d["fsr"][leg] >= 200 else 0 for d in br]
        flick.append(sum(1 for k in range(1, len(c)) if c[k] != c[k-1]) / secs * 60)
        bouts = []; cur = c[0]; L = 1
        for k in range(1, len(c)):
            if c[k] == cur: L += 1
            else: bouts.append((cur, L)); cur = c[k]; L = 1
        bouts.append((cur, L))
        for k in range(1, len(bouts)):
            if bouts[k-1][0] == 0 and bouts[k][0] == 1:
                strides += 1
                if bouts[k-1][1] >= 0.25 * HZ and bouts[k][1] >= 0.25 * HZ: clean += 1
    print(f"flicker_per_min per leg: {[round(f) for f in flick]}   clean strides: {clean}/{strides} ({100*clean/max(1,strides):.0f} %)")
