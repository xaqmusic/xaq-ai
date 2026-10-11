#!/usr/bin/env python3
"""run_audio_analysis.py <dashrun_dir> [...] — a recorded brain run, as sound against command.

For dash runs recorded with audio (dash_run.py RunController.record): audio.wav (48 kHz mono),
feed.jsonl (benchd's 50 Hz state feed: commanded `us`, sent `out`, `i_a`, `fsr`, `tof_m`, mode,
stopped), events.jsonl (sync taps on CLOCK_MONOTONIC).  Reports:
  alignment      the sync taps' 4-8 kHz onsets against their commands (stall_probe #1: a moving
                 servo whines there ~40 dB above the room, onset ~+20 ms)
  contingency    how well commanded motion predicts the whine, the best lag, which joints are heard
  current        vs feet loaded, at 50 Hz
  stalls         windows where the command moves, the whine is absent and the current is high
  events         HAT resets and long stops (the tilt guard): the second before each
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import butter, sosfiltfilt

EW = 480                     # 10 ms envelope frames
SERVO_RESP_MS = 20.0         # command -> audible onset, stall_probe #1 (IQR 20-30 ms)
JOINT_OF_CH = {}
for s in json.load(open(Path(__file__).resolve().parents[3] / "pi_host/calib/servo_map.json"))["servos"]:
    JOINT_OF_CH[s["ch"]] = s["joint"]


def band_db(x, fs, lo, hi):
    b = sosfiltfilt(butter(4, [lo, hi], btype="band", fs=fs, output="sos"), x)
    n = len(b) // EW
    return 20 * np.log10(np.sqrt((b[: n * EW].reshape(n, EW) ** 2).mean(1)) + 1e-9)


def analyse(D: Path):
    fs, a = wavfile.read(D / "audio.wav")
    a = a.astype(float) / 32768.0
    ev = [json.loads(l) for l in open(D / "events.jsonl")]
    feed = [json.loads(l) for l in open(D / "feed.jsonl")]
    t_req = next(e["mono_ms"] for e in ev if e["kind"] == "audio_start_request")
    hi_db = band_db(a, fs, 4000, 8000)
    # ⚠ THE ROOM FLOOR IS MEASURED BEFORE ANYTHING MOVES (the first second of the recording,
    # before the pose).  A whole-recording median is the GAIT's noise, not the room's: in a
    # brain run the servos move in nearly every frame.
    floor = float(np.median(hi_db[5:95]))

    # ---- alignment: find t0 (mono ms of the first sample) so the taps' onsets land at +20 ms ----
    taps = [e for e in ev if e["kind"] == "sync_tap"]
    best = None
    for t0 in np.arange(t_req, t_req + 400, 2.0):        # arecord starts after the request
        score = 0.0
        for e in taps:
            k = int((e["mono_ms"] + SERVO_RESP_MS - t0) / 1000 * fs / EW)
            if 2 <= k < len(hi_db) - 6:
                score += hi_db[k:k + 4].mean() - hi_db[k - 2:k].mean()   # rise across the onset
        if best is None or score > best[1]:
            best = (t0, score)
    t0 = best[0]
    rises = []
    for e in taps:
        k = int((e["mono_ms"] + SERVO_RESP_MS - t0) / 1000 * fs / EW)
        rises.append(hi_db[k:k + 4].mean() - hi_db[k - 3:k - 1].mean())
    print(f"\n=== {D.name}  ({len(a) / fs:.0f} s audio, {len(feed)} feed frames)")
    print(f"alignment: first sample {t0 - t_req:.0f} ms after the record request; the 6 sync taps rise "
          f"{np.median(rises):.0f} dB (min {min(rises):.0f}) in 4-8 kHz at +{SERVO_RESP_MS:.0f} ms; quiet room {floor:.0f} dB")

    # ---- the feed on the 10 ms grid ----
    ft = np.array([f["t"] for f in feed], float)
    out = np.array([f["out"] for f in feed], float)
    cur = np.array([f["i_a"] if f["i_a"] is not None else np.nan for f in feed])
    fsr = np.array([f["fsr"] for f in feed], float)
    driven = np.array([f.get("mode") in ("dev", "autonomous") and not f.get("stopped") for f in feed])
    nt = len(hi_db)
    tg = t0 + (np.arange(nt) + 0.5) * EW / fs * 1000              # 10 ms frame centres
    idx = np.clip(np.searchsorted(ft, tg) - 1, 0, len(ft) - 2)
    ok_t = (tg >= ft[0]) & (tg <= ft[-1])
    # commanded motion: |delta out| per feed frame, summed per joint type, spread over the frame
    dout = np.abs(np.diff(out, axis=0))
    dout = np.vstack([dout, dout[-1:]])
    motion = {j: dout[:, [c for c in range(12) if JOINT_OF_CH.get(c) == j]].sum(1) for j in ("hip1", "hip2", "knee")}
    m_all = sum(motion.values())
    g_motion = m_all[idx]; g_cur = cur[idx]; g_drv = driven[idx] & ok_t
    g_loaded = (fsr[idx] > 200).sum(1)
    sel = g_drv & np.isfinite(g_cur)
    print(f"brain-driven: {sel.sum() * EW / fs:.0f} s")

    # ---- contingency: commanded motion -> the 4-8 kHz whine, over lags ----
    hp = 10 ** (hi_db / 10)                                         # band power
    best_r, best_lag = -1, 0
    for lag in range(-5, 21):                                        # -50..+200 ms, audio after command
        x = g_motion[sel]
        y = np.roll(hp, -lag)[sel]
        r = np.corrcoef(x, y)[0, 1]
        if r > best_r:
            best_r, best_lag = r, lag
    print(f"contingency: commanded motion vs 4-8 kHz power r = {best_r:+.2f} at audio lag {best_lag * 10:+d} ms")
    ys = np.roll(hp, -best_lag)[sel]
    for j in ("hip1", "hip2", "knee"):
        r = np.corrcoef(motion[j][idx][sel], ys)[0, 1]
        print(f"  {j:5} motion vs whine r = {r:+.2f}   (mean {motion[j][idx][sel].mean():6.1f} us per 20 ms frame)")
    # quiet vs moving: the whine distribution when nothing is commanded vs a lot
    yd = np.roll(hi_db, -best_lag)[sel]
    q = g_motion[sel]
    for lo_, hi_, name in ((0, 1, "no commanded motion"), (1, 120, "some"), (120, 1e9, "a lot (>120 us/frame)")):
        m = (q >= lo_) & (q < hi_)
        if m.any():
            print(f"  whine level, {name:24}: median {np.median(yd[m]):6.1f} dB  (n={m.sum()})")

    # ---- current vs feet loaded, 50 Hz ----
    c50, l50 = cur[driven], (fsr[driven] > 200).sum(1)
    print("current vs feet loaded (FSR > 200), 50 Hz:")
    for n in range(5):
        m = l50 == n
        if m.sum() > 20:
            print(f"  {n} feet: n={m.sum():5d}  mean {np.nanmean(c50[m]):.2f} A  p95 {np.nanpercentile(c50[m], 95):.2f} A")

    # ---- how often is the robot quiet enough for "stalled = silent" to be readable at all? ----
    yq = np.roll(hi_db, -best_lag)[sel]
    print(f"whine above the quiet room during the brain run: median {np.median(yq) - floor:+.0f} dB; "
          f"frames within 12 dB of the room: {np.mean(yq < floor + 12) * 100:.1f} %; "
          f"frames with no commanded motion: {np.mean(g_motion[sel] < 1) * 100:.2f} %")

    # ---- events: HAT resets and long stops, from the feed's stopped flag in a brain mode ----
    brain_mode = np.array([f.get("mode") in ("dev", "autonomous") for f in feed])
    stopped = np.array([bool(f.get("stopped")) for f in feed])
    seg, i = [], 0
    while i < len(feed):
        if brain_mode[i] and stopped[i] and (i == 0 or not stopped[i - 1]):
            j = i
            while j < len(feed) and stopped[j] and brain_mode[j]:
                j += 1
            seg.append((i, j))
            i = j
        i += 1
    for n_seg, (i, j) in enumerate(seg):
        dur = (ft[min(j, len(ft) - 1)] - ft[i]) / 1000
        pre = slice(max(0, i - 50), i)
        never_driven = not driven[:i].any()
        if never_driven:
            kind = "mode-change stop (before resume)"
        elif j >= len(feed) - 1 or not brain_mode[min(j, len(feed) - 1)]:
            kind = "end of run (E)"
        else:
            kind = "HAT reset + recovery" if dur < 10 else "long stop (tilt guard / operator)"
        k = int((ft[i] - t0) / 1000 * fs / EW)
        print(f"  t+{(ft[i] - ft[0]) / 1000:6.1f}s {kind:34} {dur:5.1f} s | 1 s before: current mean "
              f"{np.nanmean(cur[pre]):.2f} max {np.nanmax(cur[pre]):.2f} A, feet loaded {np.mean((fsr[pre] > 200).sum(1)):.1f}, "
              f"motion {m_all[pre].mean():.0f} us/frame, whine {np.median(hi_db[max(0, k - 100):k]):.0f} dB")


for d in sys.argv[1:]:
    analyse(Path(d))
