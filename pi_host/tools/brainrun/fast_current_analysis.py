#!/usr/bin/env python3
"""fast_current_analysis.py <inacap.csv> <feed.jsonl> <benchd_events.jsonl> — what browns the HAT out.

Inputs share CLOCK_MONOTONIC: benchd's fast INA219 capture (sag mode, ~940 Hz: current AND pack
voltage), its 50 Hz state feed, and its own event records (hat_reset, resume...).  The INA219 is
powered from the HAT's 3V3, so when the HAT browns out the capture itself breaks — a gap or
garbage — which dates the brownout to the millisecond (benchd's own detection lags up to 100 ms).
Reports:
  1. each captured brownout, 10 ms by 10 ms, the half second before it
  2. how often brain gait reaches the same current WITHOUT a brownout (lifted periods excluded)
  3. the commanded context of the biggest fast peaks (channels slewing at the cap, feet loaded)
  4. the "catch-up" after each recovery (operator: servos run full speed and do not brown out)
"""
import json
import sys

import numpy as np

cap_path, feed_path, ev_path = sys.argv[1:4]
rows = np.loadtxt(cap_path, delimiter=",", comments="#", skiprows=2, dtype=np.int64)
t_ms = rows[:, 0] / 1000.0
amps = rows[:, 1] * 10e-6 / 0.01                    # shunt LSB 10 uV over 10 mOhm
volts = (rows[:, 2] >> 3) * 4e-3
# The HAT's 3.3 V rail (captures from 2026-10-05 evening on carry an A4 column, every 4th sample):
# rail ~= 3.3 * INA pack V / (A4 read as if the reference were 3.3 V)
a4 = rows[:, 3] if rows.shape[1] > 3 else np.full(len(rows), -1)
has_a4 = (a4 > 0) & (volts > 5.0)
rail = np.full(len(rows), np.nan)
rail[has_a4] = 3.3 * volts[has_a4] / (a4[has_a4] * 3.3 / 4095 * 3.0)
ev = [json.loads(l) for l in open(ev_path)]
feed = [json.loads(l) for l in open(feed_path)]
ft = np.array([f["t"] for f in feed], float)
fout = np.array([f["out"] for f in feed], float)
fsr = np.array([f["fsr"] for f in feed], float)
tof = np.array([f["tof_m"] if f.get("tof_valid") else np.nan for f in feed])
driven = np.array([f.get("mode") in ("dev", "autonomous") and not f.get("stopped") for f in feed])
dt = np.diff(t_ms)
print(f"capture: {len(t_ms)} samples over {(t_ms[-1] - t_ms[0]) / 1000:.0f} s "
      f"(median dt {np.median(dt) * 1000:.0f} us); current p50/p99/max {np.median(amps):.2f}/"
      f"{np.percentile(amps, 99):.2f}/{amps.max():.2f} A; pack V min {volts[volts > 1].min():.2f}")

# ---- lifted: belly ToF far off the floor (the operator lifting the robot to reset it) ----
lifted = (tof > 0.10)
print(f"feed: {len(feed)} frames; brain-driven {driven.mean() * 100:.0f} %; lifted (belly ToF > 100 mm) "
      f"{lifted.mean() * 100:.1f} % of frames")


def feed_idx(t):
    return int(np.clip(np.searchsorted(ft, t) - 1, 0, len(ft) - 1))


# ---- 1. each captured brownout ----
resets = [e for e in ev if e["kind"] == "hat_reset"]
in_cap = [e for e in resets if t_ms[0] < e["t_mono_ms"] < t_ms[-1]]
print(f"\n1. {len(resets)} HAT resets in the record, {len(in_cap)} inside the capture")
pre_profiles = []
for e in in_cap:
    td = e["t_mono_ms"]
    # the brownout instant: the last sample before the capture breaks (a gap > 5 ms or a
    # voltage reading collapsing to garbage) in the 300 ms before benchd detected it
    w = np.where((t_ms > td - 400) & (t_ms <= td))[0]
    brk = None
    for k in w[1:]:
        if t_ms[k] - t_ms[k - 1] > 5.0 or volts[k] < 4.0 or volts[k] > 9.5:
            brk = k
            break
    t_b = t_ms[brk] if brk is not None else td
    lag = td - t_b
    pre = (t_ms > t_b - 500) & (t_ms < t_b)
    bins = []
    for s in range(-500, 0, 10):
        m = (t_ms >= t_b + s) & (t_ms < t_b + s + 10)
        bins.append((amps[m].max() if m.any() else np.nan, volts[m].min() if m.any() else np.nan))
    a_ = np.array(bins)
    pre_profiles.append(a_)
    i = feed_idx(t_b - 50)
    sl = np.abs(fout[i] - fout[max(0, i - 1)])
    print(f"  #{e['data']['count']} ({e['data']['why'][:28]}): capture breaks {lag:.0f} ms before benchd's detection")
    print(f"     last 500 ms: current max {np.nanmax(a_[:, 0]):.2f} A, time above 3 A "
          f"{np.mean(amps[pre] > 3.0) * 500:.0f} ms, above 4 A {np.mean(amps[pre] > 4.0) * 500:.0f} ms; pack V min "
          f"{np.nanmin(a_[:, 1]):.2f}")
    print("     last 200 ms, 10 ms max A: " + " ".join(f"{x:.1f}" for x in a_[-20:, 0]))
    print("     last 200 ms, 10 ms min V: " + " ".join(f"{x:.2f}" for x in a_[-20:, 1]))
    rw = (t_ms > td - 600) & (t_ms <= td) & np.isfinite(rail)
    if rw.any():
        rb = []
        for s_ in range(-300, 0, 20):
            m = rw & (t_ms >= td + s_) & (t_ms < td + s_ + 20)
            rb.append(np.nanmin(rail[m]) if m.any() else np.nan)
        print(f"     3.3 V rail, last 300 ms before benchd's detection (20 ms min): "
              + " ".join(f"{x:.2f}" for x in rb) + f"   lowest {np.nanmin(rail[rw]):.2f} V")
    print(f"     at the break: feet loaded {(fsr[i] > 200).sum()}, channels at the slew cap {(sl >= 35).sum()}, "
          f"belly {tof[i] * 1000 if np.isfinite(tof[i]) else float('nan'):.0f} mm")

# ---- 2. the same current without a brownout ----
fi = np.clip(np.searchsorted(ft, t_ms) - 1, 0, len(ft) - 1)
ok = driven[fi] & ~lifted[fi]
# exclude 2 s around every reset
for e in resets:
    ok &= ~((t_ms > e["t_mono_ms"] - 1500) & (t_ms < e["t_mono_ms"] + 4000))
secs = ok.sum() * np.median(dt) / 1000
print(f"\n2. brain-driven, on the floor, away from resets: {secs:.0f} s at ~940 Hz")
# 10 ms bins
b0 = t_ms[ok][0]
bi = ((t_ms[ok] - b0) // 10).astype(int)
bmax = np.full(bi.max() + 1, np.nan); np.fmax.at(bmax, bi, amps[ok])
bmin_v = np.full(bi.max() + 1, np.nan); np.fmin.at(bmin_v, bi, np.where(volts[ok] > 1, volts[ok], np.nan))
bm = bmax[np.isfinite(bmax)]
print(f"   10 ms peak current: p50 {np.percentile(bm, 50):.2f}  p90 {np.percentile(bm, 90):.2f}  p99 {np.percentile(bm, 99):.2f}"
      f"  p99.9 {np.percentile(bm, 99.9):.2f}  max {bm.max():.2f} A")
for th in (3.0, 4.0, 5.0):
    above = bmax > th
    starts = np.where(above & ~np.r_[False, above[:-1]])[0]
    lens = []
    for s in starts:
        e_ = s
        while e_ < len(above) and above[e_]:
            e_ += 1
        lens.append((e_ - s) * 10)
    lens = np.array(lens) if lens else np.array([0])
    print(f"   excursions above {th:.0f} A: {len(starts)} ({len(starts) / secs * 60:.1f} per min), duration median "
          f"{np.median(lens):.0f} ms, p95 {np.percentile(lens, 95):.0f}, max {lens.max():.0f} ms")
if np.isfinite(rail[ok]).any():
    rr = rail[ok][np.isfinite(rail[ok])]
    print(f"   3.3 V rail (gait, away from resets): p0.1 {np.percentile(rr, 0.1):.2f}  p1 {np.percentile(rr, 1):.2f}  "
          f"p50 {np.median(rr):.2f} V; below 3.0 V {np.mean(rr < 3.0) * 100:.2f} % of {len(rr)} samples")
vv = bmin_v[np.isfinite(bmin_v)]
print(f"   10 ms min pack V: p1 {np.percentile(vv, 1):.2f}  p0.1 {np.percentile(vv, 0.1):.2f}  min {vv.min():.2f} V")

# ---- 3. the commanded context of the biggest peaks ----
order = np.argsort(bm)[::-1]
print("\n3. the 15 largest 10 ms peaks (not near a reset): commanded context at that moment")
seen = []
shown = 0
valid_bins = np.where(np.isfinite(bmax))[0]
for j in np.argsort(bmax[valid_bins])[::-1]:
    b = valid_bins[j]
    tb = b0 + b * 10
    if any(abs(tb - s) < 300 for s in seen):
        continue
    seen.append(tb)
    i = feed_idx(tb)
    sl = np.abs(fout[i] - fout[max(0, i - 1)])
    rev = np.sum(np.sign(fout[i] - fout[max(0, i - 1)]) * np.sign(fout[max(0, i - 1)] - fout[max(0, i - 2)]) < 0)
    print(f"   {bmax[b]:5.2f} A  feet loaded {(fsr[i] > 200).sum()}  max foot {fsr[i].max():5.0f}  "
          f"channels at the slew cap {(sl >= 35).sum():2d}  reversing {rev:2d}  belly {tof[i] * 1000 if np.isfinite(tof[i]) else float('nan'):4.0f} mm")
    shown += 1
    if shown >= 15:
        break

# ---- 4. the catch-up after each recovery ----
resumes = [e for e in ev if e["kind"] == "resume" and e["data"].get("who") == "hat recovery"]
print(f"\n4. the 3 s after each automatic recovery ({len(resumes)}; operator: 'servos run full speed to catch up')")
for e in resumes:
    tr = e["t_mono_ms"]
    m = (t_ms > tr) & (t_ms < tr + 3000)
    i0, i1 = feed_idx(tr), feed_idx(tr + 3000)
    sl = np.abs(np.diff(fout[i0:i1 + 1], axis=0))
    capped = (sl >= 35).sum(1)
    if m.any():
        print(f"   t={tr / 1000:.0f}s: current mean {amps[m].mean():.2f} A, max {amps[m].max():.2f} A, "
              f"above 3 A {np.mean(amps[m] > 3) * 3000:.0f} ms; channels at the slew cap per frame: mean "
              f"{capped.mean():.1f}, max {capped.max()}; feet loaded {np.mean((fsr[i0:i1] > 200).sum(1)):.1f}")
    else:
        print(f"   t={tr / 1000:.0f}s: outside the capture")
