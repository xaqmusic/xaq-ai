#!/usr/bin/env python3
"""stall_audio_analysis.py <stallprobe dir> — what the stall probe recorded, seen as the brain sees it.

Laptop-side (numpy/scipy). For every 1024-sample window (what ogma_host hands the audio EPM per
tick, AudioCapture.window_samples) labels the condition from the commanded events:
  silence       the probe's quiet segments (servos holding the stand pose, nothing commanded)
  free_motion   a free move or return while the servo slews (|delta| / 40 us per tick)
  free_hold     after a free move has landed, holding, unloaded on the stand
  blocked_push  pushing into the operator's block (servo stalled)
and reports per class: level, spectrum bands, servo current, and whether the brain's OWN audio
features separate them — an exact Python twin of FrozenSTFTEncoder (128 log bands 80-8000 Hz,
direct DFT per band, log1p, divide by norm + 0.01), classified by nearest centroid with whole
EVENTS held out (windows of one move are correlated; holding out single windows would leak).
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import welch

D = Path(sys.argv[1])
WIN = 1024


def stft_twin(x, fs, n_filters=128, f_min=80.0, f_max=8000.0):
    """FrozenSTFTEncoder::encode_mono, vectorised: same centres, same maths, same normaliser."""
    t = np.arange(len(x))
    lo, hi = np.log(max(f_min, 1.0)), np.log(max(f_max, f_min + 1.0))
    fc = np.exp(lo + np.linspace(0.0, 1.0, n_filters) * (hi - lo))
    ph = 2 * np.pi * np.outer(fc / fs, t)
    re = (x * np.cos(ph)).sum(1)
    im = -(x * np.sin(ph)).sum(1)
    feat = np.log1p(np.sqrt(re * re + im * im) / len(x))
    return feat / (np.linalg.norm(feat) + 0.01)


fs, audio = wavfile.read(D / "audio.wav")
audio = audio.astype(np.float64) / 32768.0
ev = [json.loads(l) for l in open(D / "events.jsonl")]
feed = [json.loads(l) for l in open(D / "feed.jsonl")]
meta = json.load(open(D / "meta.json"))
t_audio0 = next(e for e in ev if e["kind"] == "audio_start")["popen_after_ms"]

# ---- alignment: the onset of every free move, in the 4-8 kHz band, against its commanded start ----
# ⚠ MEASURED IN dB OVER A MEDIAN BASELINE.  The first version thresholded broadband RMS in linear
# units against a baseline with transients in it; it detected 1 move of 9 and applied that one
# +85 ms lag to everything, which put the labels of short moves AFTER the moves (a 100 us move
# lasts ~50 ms).  A moving servo whines 4-8 kHz ~40 dB above the room, so the band is the clock.
from scipy.signal import butter, sosfiltfilt
EW = 480                                                   # 10 ms envelope frames
bp = sosfiltfilt(butter(4, [4000, 8000], btype="band", fs=fs, output="sos"), audio)
ne = len(bp) // EW
env_db = 20 * np.log10(np.sqrt((bp[: ne * EW].reshape(ne, EW) ** 2).mean(1)) + 1e-9)
quiet = [e for e in ev if e["kind"] == "silence_start" and e["label"] == "start"][0]
q0 = int((quiet["mono_ms"] - t_audio0) / 1000 * fs / EW)
floor_db = float(np.median(env_db[q0 + 10:q0 + 290]))
lags = []
for e in ev:
    if e["kind"] == "move_start" and e["trial"] in ("free", "free_return"):
        w0 = int((e["mono_ms"] - t_audio0) / 1000 * fs / EW)
        # Only moves that start from QUIET: a move launched while the last one still rings
        # would "onset" at the search edge and drag the median (the first fix did exactly that).
        if np.max(env_db[w0 - 15:w0 - 5]) > floor_db + 10:
            continue
        hit = next((k for k in range(w0 - 5, w0 + 30) if env_db[k] > floor_db + 20), None)
        if hit is not None:
            lags.append((hit - w0) * EW / fs * 1000)
n_free = sum(1 for e in ev if e["kind"] == "move_start" and e["trial"] in ("free", "free_return"))
lag_ms = float(np.median(lags)) if lags else 0.0
nwin = len(audio) // WIN
print(f"{D.name}: {len(audio) / fs:.1f} s audio, {len(feed)} feed frames; leg {meta['leg']}, joints {meta['joints']}")
print(f"alignment: 4-8 kHz onset {lag_ms:+.0f} ms after the commanded start (median of {len(lags)}/{n_free} free moves,"
      f" IQR {np.percentile(lags, 25) if lags else 0:.0f}..{np.percentile(lags, 75) if lags else 0:.0f} ms; room floor {floor_db:.0f} dB)")
t_audio0 -= lag_ms                       # shift so commanded starts land on audible onsets

# ---- label windows ----
label = np.array([""] * nwin, dtype=object)
event_id = np.full(nwin, -1)
joint_of = np.array([""] * nwin, dtype=object)


def win_range(t0_ms, t1_ms):
    a = int(max(0, (t0_ms - t_audio0) / 1000 * fs) // WIN)
    b = int(max(0, (t1_ms - t_audio0) / 1000 * fs) // WIN)
    return range(a, min(b, nwin))


for i, e in enumerate(ev):
    if e["kind"] == "silence_start":
        end = next(x for x in ev[i:] if x["kind"] == "silence_end")
        for k in win_range(e["mono_ms"] + 100, end["mono_ms"] - 50):
            label[k], event_id[k] = "silence", i
    if e["kind"] == "move_start":
        end = next(x for x in ev[i:] if x["kind"] in ("hold_end", "abort"))
        slew_ms = abs(e["delta"]) / 40 * 20 + 20
        if e["trial"] in ("free", "free_return"):
            for k in win_range(e["mono_ms"], e["mono_ms"] + slew_ms):
                label[k], event_id[k], joint_of[k] = "free_motion", i, e["joint"]
            for k in win_range(e["mono_ms"] + slew_ms + 100, end["mono_ms"]):
                label[k], event_id[k], joint_of[k] = "free_hold", i, e["joint"]
        elif e["trial"] == "blocked":
            for k in win_range(e["mono_ms"] + 120, end["mono_ms"]):
                label[k], event_id[k], joint_of[k] = "blocked_push", i, e["joint"]

# ---- current per window, from the 50 Hz feed on the same clock ----
ft = np.array([f["t"] for f in feed], dtype=float)
fi = np.array([f["i_a"] if f["i_a"] is not None else np.nan for f in feed])
wt = t_audio0 + (np.arange(nwin) + 0.5) * WIN / fs * 1000
cur = np.interp(wt, ft, fi)

classes = ["silence", "free_hold", "free_motion", "blocked_push"]
feats = np.array([stft_twin(audio[k * WIN:(k + 1) * WIN], fs) if label[k] else np.zeros(128)
                  for k in range(nwin)])
print(f"\n{'class':13} {'windows':>7} {'events':>6} {'level dBFS':>11} {'current A':>14}  "
      f"band share of energy: <1k  1-4k  4-8k  8-16k  >16k")
for c in classes:
    idx = np.where(label == c)[0]
    if len(idx) == 0:
        continue
    seg = np.concatenate([audio[k * WIN:(k + 1) * WIN] for k in idx])
    f, p = welch(seg, fs, nperseg=2048)
    bands = [(0, 1000), (1000, 4000), (4000, 8000), (8000, 16000), (16000, 24001)]
    e = np.array([p[(f >= a) & (f < b)].sum() for a, b in bands]); e /= e.sum()
    lvl = 20 * np.log10(np.sqrt((seg ** 2).mean()) + 1e-12)
    print(f"{c:13} {len(idx):7d} {len(set(event_id[idx])):6d} {lvl:11.1f} {np.nanmean(cur[idx]):6.3f} / {np.nanmax(cur[idx]):5.3f}  "
          + "  ".join(f"{v:4.2f}" for v in e))

print("\nper joint:  free motion vs blocked push   (level dBFS, mean current A, peak A)")
for j in meta["joints"]:
    row = []
    for c in ("free_motion", "blocked_push"):
        idx = np.where((label == c) & (joint_of == j))[0]
        if len(idx):
            seg = np.concatenate([audio[k * WIN:(k + 1) * WIN] for k in idx])
            row.append(f"{20 * np.log10(np.sqrt((seg ** 2).mean()) + 1e-12):6.1f} dB {np.nanmean(cur[idx]):.3f} A {np.nanmax(cur[idx]):.3f} A")
        else:
            row.append("—")
    print(f"  {j:5}  {row[0]}   |   {row[1]}")

# ---- does the brain's encoder separate the classes?  hold out whole events ----
use = np.array([l in classes for l in label])
X, y, g = feats[use], label[use], event_id[use]
events = sorted(set(g))
rng = np.random.default_rng(0)
conf = defaultdict(lambda: defaultdict(int))
for fold in range(5):
    test_ev = set(rng.permutation(events)[: max(1, len(events) // 5)])
    tr = np.array([x not in test_ev for x in g]); te = ~tr
    cents = {c: X[tr & (y == c)].mean(0) for c in classes if (tr & (y == c)).any()}
    for xv, yv in zip(X[te], y[te]):
        pred = min(cents, key=lambda c: np.linalg.norm(xv - cents[c]))
        conf[yv][pred] += 1
print("\nnearest-centroid on the brain's audio features (FrozenSTFTEncoder twin), whole events held out:")
print(f"  {'true \\ predicted':16}" + "".join(f"{c:>14}" for c in classes) + "   recall")
for c in classes:
    tot = sum(conf[c].values())
    if tot:
        print(f"  {c:16}" + "".join(f"{conf[c][p]:14d}" for p in classes) + f"   {conf[c][c] / tot:5.2f}")
# the one question the stall witness needs answered: motion vs stalled, given the servo is commanded
mv = [l in ("free_motion", "blocked_push") for l in y]
Xm, ym = X[mv], y[mv]
cm, cb = Xm[ym == "free_motion"].mean(0), Xm[ym == "blocked_push"].mean(0)
cos = float(cm @ cb / (np.linalg.norm(cm) * np.linalg.norm(cb)))
print(f"\ncentroid cosine, free_motion vs blocked_push: {cos:.3f}  (1.0 = indistinguishable shapes)")

# ---- the question the stall witness actually asks: GIVEN the servo is commanded to move, ----
# ---- is it moving or stalled?  (a stall sounds like silence; only the intent tells them apart) ----
both = np.where(np.isin(label, ["free_motion", "blocked_push"]))[0]
Xb, yb, gb = feats[both], label[both], event_id[both]
evs = sorted(set(gb)); hits = tot = 0
per = defaultdict(lambda: [0, 0])
for held in evs:                                     # leave one EVENT out
    tr = gb != held
    c_m = Xb[tr & (yb == "free_motion")].mean(0); c_b = Xb[tr & (yb == "blocked_push")].mean(0)
    for xv, yv in zip(Xb[gb == held], yb[gb == held]):
        pred = "free_motion" if np.linalg.norm(xv - c_m) < np.linalg.norm(xv - c_b) else "blocked_push"
        hits += pred == yv; tot += 1; per[yv][0] += pred == yv; per[yv][1] += 1
print(f"\ngiven a commanded move — moving or stalled? (encoder twin, leave-one-event-out, per window):"
      f" accuracy {hits / tot:.2f}; free_motion {per['free_motion'][0]}/{per['free_motion'][1]},"
      f" blocked_push {per['blocked_push'][0]}/{per['blocked_push'][1]}")
# and per EVENT (a 1 s push is ~45 windows; the brain integrates over a push, not one window)
ev_hits = ev_tot = 0
for held in evs:
    tr = gb != held
    c_m = Xb[tr & (yb == "free_motion")].mean(0); c_b = Xb[tr & (yb == "blocked_push")].mean(0)
    xm = Xb[gb == held].mean(0); truth = yb[gb == held][0]
    pred = "free_motion" if np.linalg.norm(xm - c_m) < np.linalg.norm(xm - c_b) else "blocked_push"
    ev_hits += pred == truth; ev_tot += 1
print(f"  per event (window mean): {ev_hits}/{ev_tot} correct")
