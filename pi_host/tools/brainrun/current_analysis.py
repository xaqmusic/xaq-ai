#!/usr/bin/env python3
"""current_analysis.py <benchd_*.jsonl ...> — S0 of the power-budget work: what draws the current?

Uses benchd's own 10 Hz telemetry records, frames where a brain had the servos (mode dev or
autonomous, not stopped). Per frame:
  I          servo-branch current (INA219, ~68 ms average)
  moving     channels whose pulse on the line moved >= 60 us in the last frame (>= 30 % of full slew)
  slew       total |delta pulse| across the 12 channels, us per frame
  at_limit   channels whose TARGET sits on its envelope bound — the command pushes past the range
  stalled    at_limit channels that are not moving — pressing, the stall candidates
  fsr        sum of the four foot-ADC counts (load proxy; HAT-reset garbage frames dropped)
Then: correlations with I, the feature means in high- vs low-current frames, and the second
before each recorded hat_reset.
"""
import json
import statistics as st
import sys


def frames(path):
    out, events = [], []
    for line in open(path, errors="replace"):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        k = r.get("kind")
        if k in ("hat_reset", "adc_garbage", "mcu_reset"):
            events.append((r["t_mono_ms"], k))
        if k != "telemetry":
            continue
        d = r["data"]
        if d.get("mode") not in ("dev", "autonomous") or d.get("stopped"):
            continue
        ina = d.get("ina") or {}
        if not ina.get("ok") or "servos" not in d:
            continue
        out.append((r["t_mono_ms"], d))
    return out, events


def features(prev, cur):
    sp, sc = prev["servos"], cur["servos"]
    dl = [abs(sc[c].get("out_us", sc[c]["current_us"]) - sp[c].get("out_us", sp[c]["current_us"])) for c in range(12)]
    moving = sum(1 for x in dl if x >= 60)
    at_lim = [c for c in range(12) if sc[c]["armed"] and sc[c]["target_us"] in (sc[c]["min_us"], sc[c]["max_us"])]
    stalled = sum(1 for c in at_lim if dl[c] < 10)
    adc = cur.get("adc") or []
    fsr = sum(adc[:4]) if len(adc) >= 4 else None
    return {"I": (cur.get("ina") or {}).get("i_a", 0.0), "moving": moving, "slew": sum(dl),
            "at_limit": len(at_lim), "stalled": stalled, "fsr": fsr, "vbat": cur.get("vbat")}


def corr(xs, ys):
    if len(xs) < 3:
        return float("nan")
    mx, my = st.mean(xs), st.mean(ys)
    sx = sum((x - mx) ** 2 for x in xs) ** 0.5
    sy = sum((y - my) ** 2 for y in ys) ** 0.5
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy) if sx and sy else float("nan")


rows, before_reset = [], []
for path in sys.argv[1:]:
    fr, ev = frames(path)
    resets = [t for t, k in ev if k in ("hat_reset", "mcu_reset")]
    for (t0, a), (t1, b) in zip(fr, fr[1:]):
        if t1 - t0 > 250:
            continue                                   # a gap: not consecutive frames
        f = features(a, b)
        if f["vbat"] and f["vbat"] > 9.0:
            continue                                   # post-reset garbage
        f["file"] = path.split("/")[-1]
        rows.append(f)
        if any(0 <= r - t1 <= 1000 for r in resets):
            before_reset.append(f)

if not rows:
    sys.exit("no brain-driven frames found")
I = [r["I"] for r in rows]
print(f"{len(rows)} brain-driven frames from {len(set(r['file'] for r in rows))} records;"
      f" current mean {st.mean(I):.2f} A, p95 {sorted(I)[int(.95 * len(I))]:.2f}, max {max(I):.2f}")
print("\ncorrelation with current:")
for k in ("moving", "slew", "at_limit", "stalled", "fsr"):
    xs = [r[k] for r in rows if r[k] is not None]
    ys = [r["I"] for r in rows if r[k] is not None]
    print(f"  {k:9} r = {corr(xs, ys):+.2f}")
hi = [r for r in rows if r["I"] >= 2.0]
lo = [r for r in rows if r["I"] < 1.0]
print(f"\nfeature means: current >= 2.0 A (n={len(hi)})  vs  < 1.0 A (n={len(lo)})")
for k in ("moving", "slew", "at_limit", "stalled", "fsr"):
    h = [r[k] for r in hi if r[k] is not None]
    l = [r[k] for r in lo if r[k] is not None]
    if h and l:
        print(f"  {k:9} {st.mean(h):8.1f}  vs  {st.mean(l):8.1f}")
if before_reset:
    print(f"\nthe second before a HAT reset (n={len(before_reset)} frames):")
    for k in ("I", "moving", "slew", "at_limit", "stalled"):
        print(f"  {k:9} mean {st.mean(r[k] for r in before_reset):6.2f}  max {max(r[k] for r in before_reset):6.2f}")
