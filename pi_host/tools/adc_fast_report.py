#!/usr/bin/env python3
"""What is actually on the ADC pins?  (Foot-FSR bench order steps E2 / E2b.)

Reads the `adc_fast` records benchd writes when `adc.rate` is on, and reports, per
segment and per channel: the level, the noise, and -- separately -- the WANDER.

⚠ IT READS THE RECORD.  IT DOES NOT TOUCH THE BUS.  benchd owns I2C; a script that
polls the ADC alongside it contends for the same device.  `rail_separation_test.py`'s
header records what that cost the first time: a sampler that wedged, one sample across
six pose cycles, reported as a clean PASS.  One owner, and everyone else reads the log.

⚠⚠ THE SERVO FRAME ALIASES TO A 20-SECOND WANDER, NOT TO NOISE.
The HAT's servo frame is 49.95 Hz (PROTOCOL.md) and this samples at 50 Hz, so anything
picked up at the frame rate folds to |50 - 49.95| = 0.05 Hz -- a slow drift with a ~20 s
period.  That is the same shape, the same timescale and the same direction as FSR creep,
which is the quantity the graded `unloaded` criterion term (weight 1.0) is built on.  An
unfiltered divider would therefore not look noisy.  It would look like a sensor that
creeps, and it would be believed.

That is why `wander` is reported beside `sigma` instead of being folded into it:

  sigma    sample-to-sample spread of the raw counts
  white    stdev(diff)/sqrt(2) -- the part that behaves like white noise
  wander   stdev of 1 s block means -- the low-frequency part
  wander%  wander as a multiple of what white noise ALONE would give (sigma/sqrt(n_blk)).
           ~1.0x means nothing slow is present.  Several x means something is.

USAGE
  adc_fast_report.py                       # newest log in pi_host/log
  adc_fast_report.py path/to/benchd_*.jsonl
  adc_fast_report.py --csv out.csv         # also dump every sample
  adc_fast_report.py --fft                 # spectral peaks (needs numpy)

Segments are split at `mark` records, so label the arms as you go:
  python3 bench_verb.py mark text="Zsrc=5k cap=none servos=hold"
"""
import glob, json, math, os, statistics as st, sys

VREF, FULL = 3.3, 4095
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "log")
BLOCK_S = 1.0          # the wander window


def load(path):
    """Returns (samples, marks, errors). samples: (t_ms, [c0..c3], read_us)."""
    samples, marks, errors, rates = [], [], [], []
    with open(path) as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue                      # a torn last line during a live tail
            kind, t, d = r.get("kind"), r.get("t_mono_ms"), r.get("data", {})
            if kind == "adc_fast":
                a = d.get("a")
                if isinstance(a, list) and len(a) == 4:
                    samples.append((t, a, d.get("us")))
            elif kind == "mark":
                marks.append((t, d.get("text", "")))
            elif kind == "bus_error" and d.get("where") == "adc_fast":
                errors.append((t, d.get("what", ""), d.get("count")))
            elif kind == "verb" and d.get("req", {}).get("verb") == "adc.rate":
                rates.append((t, d.get("req", {}).get("ms")))
    return samples, marks, errors, rates


def segment(samples, marks):
    """Split samples at marks. Returns [(label, [samples])], earliest first."""
    if not marks:
        return [("(no marks — whole log)", samples)]
    out, bounds = [], [(t, txt) for t, txt in marks]
    first = [s for s in samples if s[0] < bounds[0][0]]
    if first:
        out.append(("(before the first mark)", first))
    for i, (t0, txt) in enumerate(bounds):
        t1 = bounds[i + 1][0] if i + 1 < len(bounds) else float("inf")
        out.append((txt, [s for s in samples if t0 <= s[0] < t1]))
    return out


def channel_stats(ts, counts):
    """ts in ms, counts a list of ints for ONE channel."""
    n = len(counts)
    s = {"n": n, "mean": st.fmean(counts), "min": min(counts), "max": max(counts)}
    s["mean_v"] = s["mean"] * VREF / FULL
    s["pinned"] = "LOW" if s["max"] == 0 else ("HIGH" if s["min"] >= FULL else "")
    if n < 3:
        return s
    s["sigma"] = st.pstdev(counts)
    diffs = [counts[i + 1] - counts[i] for i in range(n - 1)]
    s["white"] = st.pstdev(diffs) / math.sqrt(2.0)
    # wander: stdev of BLOCK_S means
    blocks, cur, t0 = [], [], ts[0]
    for t, c in zip(ts, counts):
        if t - t0 >= BLOCK_S * 1000.0 and cur:
            blocks.append(st.fmean(cur)); cur, t0 = [], t
        cur.append(c)
    if cur:
        blocks.append(st.fmean(cur))
    if len(blocks) >= 3:
        per_block = n / len(blocks)
        s["wander"] = st.pstdev(blocks)
        expected = s["sigma"] / math.sqrt(per_block) if per_block > 0 else 0.0
        # A dead channel has sigma 0, so the ratio is meaningless rather than infinite.
        # The CONSTANT/PINNED flag is what says something is wrong; don't print "inf" too.
        s["wander_x"] = (s["wander"] / expected) if expected > 1e-9 else None
        s["blocks"] = len(blocks)
    return s


def spectrum(ts, counts, top=5):
    try:
        import numpy as np
    except ImportError:
        return None
    n = len(counts)
    if n < 64:
        return None
    dt = (ts[-1] - ts[0]) / 1000.0 / (n - 1)
    x = np.asarray(counts, float); x -= x.mean()
    w = np.hanning(n)
    # ⚠ 4/n, not 2/n.  2/n is the one-sided scaling for an UNwindowed transform; Hanning has
    # a coherent gain of 0.5, so without dividing it back out every amplitude reads half its
    # real size.  Verified against an injected 40 LSB tone, which must come back as 40.
    mag = np.abs(np.fft.rfft(x * w)) * 2.0 / (n * w.mean())
    freq = np.fft.rfftfreq(n, dt)
    # Peaks are only peaks against the floor.  A flat channel has no spectrum, and listing
    # its largest noise bins as "peaks" invites reading structure into nothing.
    floor = float(np.median(mag[1:])) if n > 4 else 0.0
    thresh = max(floor * 4.0, 0.05)
    bin_hz = float(freq[1]) if len(freq) > 1 else 0.0
    out = []
    for i in np.argsort(mag)[::-1]:
        f = float(freq[i])
        if i == 0 or f < 0.02 or float(mag[i]) < thresh:
            continue
        # leakage puts the same tone in neighbouring bins; keep the tallest of each cluster
        if any(abs(f - g) <= 2.5 * bin_hz for g, _ in out):
            continue
        out.append((f, float(mag[i])))
        if len(out) >= top:
            break
    return out, floor


def main():
    args = [a for a in sys.argv[1:]]
    want_fft = "--fft" in args; args = [a for a in args if a != "--fft"]
    csv_path = None
    if "--csv" in args:
        i = args.index("--csv"); csv_path = args[i + 1]; del args[i:i + 2]
    if args:
        path = args[0]
    else:
        cand = sorted(glob.glob(os.path.join(LOG_DIR, "benchd_*.jsonl")))
        if not cand:
            sys.exit(f"no benchd_*.jsonl in {os.path.normpath(LOG_DIR)} — name one explicitly")
        path = cand[-1]

    samples, marks, errors, rates = load(path)
    print(f"log     : {path}")
    if rates:
        print("adc.rate: " + ", ".join(f"ms={ms}" for _, ms in rates))
    # ⚠ Loud, not zero-filled.  A report that prints a clean empty table for a sampler that
    # never ran is the failure this whole bench order exists to avoid.
    if not samples:
        sys.exit("NO `adc_fast` RECORDS IN THIS LOG.\n"
                 "  The sampler was never enabled, or it was enabled on a different run.\n"
                 "  Enable it with:  python3 bench_verb.py adc.rate ms=20")

    if csv_path:
        with open(csv_path, "w") as f:
            f.write("t_mono_ms,a0,a1,a2,a3,read_us\n")
            for t, a, us in samples:
                f.write(f"{t},{a[0]},{a[1]},{a[2]},{a[3]},{'' if us is None else us}\n")
        print(f"csv     : {csv_path}  ({len(samples)} samples)")

    for label, seg in segment(samples, marks):
        print(f"\n=== {label}")
        if len(seg) < 3:
            print(f"    only {len(seg)} samples — skipped")
            continue
        ts = [s[0] for s in seg]
        span = (ts[-1] - ts[0]) / 1000.0
        hz = (len(seg) - 1) / span if span > 0 else 0.0
        gaps = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        nominal = st.median(gaps)
        dropped = sum(1 for g in gaps if g > nominal * 1.5)
        us = [s[2] for s in seg if s[2] is not None]
        print(f"    {len(seg)} samples over {span:.1f} s = {hz:.2f} Hz"
              f"   (median gap {nominal:.0f} ms, {dropped} gap{'' if dropped == 1 else 's'} > 1.5x)")
        if us:
            print(f"    read cost: mean {st.fmean(us):.0f} us, max {max(us)} us  for 4 channels")
        nerr = sum(1 for t, *_ in errors if ts[0] <= t <= ts[-1])
        if nerr:
            print(f"    ⚠ {nerr} adc_fast bus errors inside this window")

        print(f"    {'ch':<4}{'mean':>9}{'mV':>9}{'sigma':>8}{'white':>8}"
              f"{'wander':>8}{'wander x':>10}   {'range':>12}")
        for ch in range(4):
            col = [s[1][ch] for s in seg]
            k = channel_stats(ts, col)
            wx = k.get("wander_x")
            wx_s = "—" if wx is None else f"{wx:.1f}x"
            flag = ""
            if k["pinned"]:
                flag = f"  ⚠ PINNED {k['pinned']}"
            elif k.get("sigma", 0) == 0:
                flag = "  ⚠ CONSTANT — disconnected or not sampled?"
            elif wx is not None and wx >= 3.0:
                flag = "  ⚠ low-frequency content — see the header on the 0.05 Hz beat"
            print(f"    A{ch:<3}{k['mean']:>9.1f}{k['mean_v']*1000:>9.1f}"
                  f"{k.get('sigma', 0):>8.2f}{k.get('white', 0):>8.2f}"
                  f"{k.get('wander', 0):>8.2f}{wx_s:>10}"
                  f"   {k['min']:>5}..{k['max']:<5}{flag}")
        if want_fft:
            for ch in range(4):
                res = spectrum(ts, [s[1][ch] for s in seg])
                if res is None:
                    print("    (--fft needs numpy, or the segment is too short)")
                    break
                pk, floor = res
                if pk:
                    print(f"    A{ch} peaks: " +
                          "  ".join(f"{f:.2f} Hz @ {m:.1f} LSB" for f, m in pk) +
                          f"   (floor {floor:.2f})")
                else:
                    print(f"    A{ch} peaks: none above the floor ({floor:.2f} LSB) "
                          f"— broadband, no line to blame")


if __name__ == "__main__":
    main()
