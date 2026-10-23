#!/usr/bin/env python3
"""Self-righting scorer: the robot is dropped UPSIDE DOWN (OGMA_PICRAWLER_TELEPORT_RAMP_AT + _TELEPORT_FLIP=1
+ _TELEPORT_XZ) with the inversion auto-reset off (OGMA_PICRAWLER_AUTO_RESET_INVERSION=0), and this reads whether
it gets itself back on its feet.  Operator, 2026-10-22: the original walker would eventually thrash enough to flip
itself over; the LPG walker's movements seem restricted to walking.

From the drop (the body's "[teleport] ... tick N" line):
  righted    tilt < UP_TILT rad held for HOLD ticks (on its feet again)
  t_right    ticks from the drop to the start of that hold
  distress   mean published distress while still inverted (tilt > 2.0 rad)
  panic      fraction of inverted ticks with the motor's panic engaged (panic_eff > 0.05)
  leg_sd     mean per-joint standard deviation of the commanded-lagged joint angles while inverted (rad):
             how much the legs actually move
Usage: selfright_score.py NAME=<glob> [NAME=<glob> ...]
"""
import glob, json, math, re, statistics as st, sys
from _logio import open_log, log_glob

UP_TILT, HOLD = 0.6, 100


def one(path):
    tp = None
    for l in open_log(path):
        mm = re.search(r"\[teleport\] dropped.* tick (\d+)", l)
        if mm:
            tp = int(mm.group(1)); break
    if tp is None:
        return None
    rows = []
    for l in open_log(path):
        if l.startswith("{") and '"tilt"' in l and '"x"' in l and '"con"' in l:
            try:
                r = json.loads(l)
            except Exception:
                continue
            if r["t"] > tp + 25:
                rows.append(r)
    if not rows:
        return None
    t_right, run = float("nan"), 0
    for r in rows:
        run = run + 1 if r["tilt"] < UP_TILT else 0
        if run >= HOLD:
            t_right = r["t"] - HOLD - tp; break
    inv = [r for r in rows if r["tilt"] > 2.0 and (math.isnan(t_right) or r["t"] < tp + t_right)]
    d = st.mean(r.get("distress", 0.0) for r in inv) if inv else float("nan")
    pn = sum(1 for r in inv if r.get("panic_eff", 0.0) > 0.05) / len(inv) if inv else float("nan")
    sds = []
    if len(inv) > 50:
        for key in ("hip1", "hip2", "knee"):
            for k in range(4):
                sds.append(st.pstdev(r[key][k] for r in inv))
    return {"righted": not math.isnan(t_right), "t_right": t_right, "distress": d, "panic": pn,
            "leg_sd": st.mean(sds) if sds else float("nan"), "inv_ticks": len(inv), "window": rows[-1]["t"] - tp}


def main():
    for spec in sys.argv[1:]:
        name, _, pat = spec.partition("=")
        res = [r for r in (one(p) for p in log_glob(pat)) if r]
        if not res:
            print(f"  {name:30s} no drop found"); continue
        ok = [r for r in res if r["righted"]]
        def m(k):
            v = [r[k] for r in res if not math.isnan(r[k])]
            return st.mean(v) if v else float("nan")
        print(f"  {name:30s} righted {len(ok)}/{len(res)}  t_right median {st.median([r['t_right'] for r in ok]) if ok else float('nan'):6.0f}"
              f"  | while inverted: distress {m('distress'):.2f}  panic {m('panic'):.2f}  leg sd {m('leg_sd'):.3f} rad"
              f"  ({m('inv_ticks'):.0f} ticks)  window {res[0]['window']}")


if __name__ == "__main__":
    main()
