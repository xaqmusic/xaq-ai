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

# ---- alignment: the onset of the large free moves against their commanded start ----
nwin = len(audio) // WIN
rms = np.sqrt((audio[: nwin * WIN].reshape(nwin, WIN) ** 2).mean(1))
quiet = [e for e in ev if e["kind"] == "silence_start" and e["label"] == "start"][0]
q0 = int((quiet["mono_ms"] - t_audio0) / 1000 * fs / WIN)
base_mu, base_sd = rms[q0 + 5:q0 + 120].mean(), rms[q0 + 5:q0 + 120].std()
lags = []
for e in ev:
    if e["kind"] == "move_start" and e["trial"] == "free" and abs(e["delta"]) >= 400:
        w0 = int((e["mono_ms"] - t_audio0) / 1000 * fs / WIN)
        for k in range(w0 - 3, w0 + 25):
            if rms[k] > base_mu + 6 * base_sd:
                lags.append((k - w0) * WIN / fs * 1000)
                break
lag_ms = float(np.median(lags)) if lags else 0.0
print(f"{D.name}: {len(audio) / fs:.1f} s audio, {len(feed)} feed frames; leg {meta['leg']}, joints {meta['joints']}")
print(f"alignment: motion onset {lag_ms:+.0f} ms after the commanded start (median of {len(lags)} large moves;"
      f" spread {np.min(lags) if lags else 0:.0f}..{np.max(lags) if lags else 0:.0f} ms) — applied")
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
