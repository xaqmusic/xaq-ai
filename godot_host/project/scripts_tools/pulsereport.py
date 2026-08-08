#!/usr/bin/env python3
"""Is the body producing CONSISTENT FORWARD PULSES?

THE OPERATOR'S GOAL, stated 2026-08-05 while watching the intent arm: "there are still
periods where the robot makes good strides and pauses ... try some iterations to produce
consistent forward pulses."  Every metric we have been reading (disp/s, metres, stalled%)
is blind to this: a body that lurches then stalls and a body that strides evenly can post
identical displacement.  That blindness is why four mechanisms in a row read as "ties".

WHAT IS MEASURED.  fwd_v is a quasi-rectified sine at stride frequency (operator's read,
confirmed: sd 0.13 on a -0.35..+0.66 range).  A "pulse" is one positive excursion of
forward velocity.  We want those pulses REGULAR, so the headline number is:

    pulse_cv = sd(inter-pulse interval) / mean(inter-pulse interval)

  ~0.0  a metronome
  ~0.5  irregular but rhythmic
  ~1.0  memoryless -- a Poisson process, i.e. NO rhythm at all

⚠ THE PRIOR READING THIS CORRECTS.  `step_cv` was measured at 0.98 on the deployed gait
and read as "the body has no step period".  That was measured on FOOT CONTACTS, which are
polluted by micro-lifts and by legs that never leave the ground.  Forward-velocity pulses
are the thing the operator is actually watching and the thing that matters for travel.

⚠ THE BLIND SPOT IN pulse_cv ITSELF (CLAUDE.md §3 rule 4).  A body that barely moves emits
tiny, very regular jitter and scores a beautiful CV.  So pulse_cv is NEVER read alone:
  pulse_amp    mean peak fwd_v per pulse -- are the pulses actually strides, or twitches?
  duty         fraction of ticks with fwd_v > 0 -- is it advancing or rocking in place?
  gap_p90/p50  the tail of the gap distribution.  THE PAUSE METRIC: a body that strides
               well then stalls has a fat upper tail even when the CV looks fine, because
               a few long pauses hide among many short gaps.  This is the operator's
               "good strides and pauses" made numeric.
  net / straight / fwd_v   ⚠ THE CORRECTION OF 2026-08-06.  This tool originally reported
               PATH LENGTH per second and called it disp/s.  Path length counts BACKWARD
               motion as progress, so a body thrashing in place scores brilliantly.  It
               certified a +21.7% "win" (t=5.60, 6/6 seeds, replicated out-of-sample) that
               was in truth a 57% LOSS of net displacement -- the operator caught it by eye
               from the fwd_v trace being symmetric about zero.  net_disp is now the
               headline, `straight` = net/path exposes thrash directly, and mean fwd_v must
               stay positive.  Never report a path-length number as progress again.

THE DETECTION THRESHOLD IS DERIVED, NOT TUNED (doctrine §5): a pulse must exceed the
run's own mean positive fwd_v, so the bar scales with whatever the body is doing.  A
fixed threshold would silently re-rank arms that differ in speed.

Usage: pulsereport.py <dir> [tag ...]
"""
import glob, json, math, os, statistics as st, sys

WARM = 1000          # ticks: the body is still finding its gait before this


def pulses(fv):
    """Indices of positive fwd_v excursions, thresholded by the run's OWN mean."""
    pos = [v for v in fv if v > 0]
    if len(pos) < 10:
        return [], []
    thr = st.mean(pos)                      # derived from the signal, not chosen
    idx, amps, i, n = [], [], 0, len(fv)
    while i < n:
        if fv[i] > thr:                     # rising edge above the bar
            j = i
            peak = fv[i]
            while j < n and fv[j] > 0:      # ride the excursion down to zero
                peak = max(peak, fv[j]); j += 1
            idx.append(i); amps.append(peak)
            i = j
        else:
            i += 1
    return idx, amps


def run(f):
    rows = [json.loads(l) for l in open(f)
            if l.startswith("{") and '"fwd_v"' in l]
    rows = [x for x in rows if x.get("t", 0) > WARM]
    if len(rows) < 100:
        return None
    fv = [x["fwd_v"] for x in rows]
    ts = [x["t"] for x in rows]
    idx, amps = pulses(fv)
    if len(idx) < 8:
        return None
    gaps = [ts[idx[i]] - ts[idx[i - 1]] for i in range(1, len(idx))]
    gaps.sort()
    d = [math.hypot(rows[i]["x"] - rows[i - 1]["x"], rows[i]["z"] - rows[i - 1]["z"])
         for i in range(1, len(rows))]
    raw = [ts[idx[i]] - ts[idx[i - 1]] for i in range(1, len(idx))]
    net = math.hypot(rows[-1]["x"] - rows[0]["x"], rows[-1]["z"] - rows[0]["z"])
    path = sum(d)
    return dict(
        cv=st.pstdev(raw) / max(1e-9, st.mean(raw)),
        net=net,
        straight=net / max(path, 1e-9),
        fwdv=st.mean(fv),
        amp=st.mean(amps),
        duty=sum(1 for v in fv if v > 0) / len(fv),
        p50=gaps[len(gaps) // 2],
        p90=gaps[int(len(gaps) * 0.9)],
        dps=net / max(1e-9, (ts[-1] - ts[0]) / 60.0),   # NET per second, never path
    )


def arm(d, tag):
    return [r for r in (run(f) for f in sorted(glob.glob(os.path.join(d, f"{tag}_[0-9].log"))))
            if r]


if __name__ == "__main__":
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    tags = sys.argv[2:] or sorted({os.path.basename(f).rsplit("_", 1)[0]
                                   for f in glob.glob(os.path.join(d, "*_[0-9].log"))})
    print(f"\n  {'arm':<28}{'pulse_cv':>9}{'amp':>8}{'duty':>7}"
          f"{'p90/p50':>9}{'net_m':>8}{'straight':>10}{'fwd_v':>9}{'net/s':>8}{'n':>4}")
    base = None
    for tag in tags:
        a = arm(d, tag)
        if not a:
            print(f"  {tag:<28}   (no data)"); continue
        m = {k: st.mean(x[k] for x in a) for k in a[0]}
        if base is None:
            base = m
        tail = m["p90"] / max(1e-9, m["p50"])
        flag = ""
        if m["cv"] < base["cv"] * 0.9 and m["dps"] > base["dps"] * 0.95:
            flag += "  << MORE REGULAR"
        if m["dps"] < base["dps"] * 0.9:
            flag += "  << SLOWER (do-no-harm fail)"
        print(f"  {tag:<28}{m['cv']:>9.3f}{m['amp']:>8.3f}{m['duty']:>7.2f}"
              f"{tail:>9.2f}{m['net']:>8.2f}{m['straight']:>10.3f}"
              f"{m['fwdv']:>+9.4f}{m['dps']:>8.4f}{len(a):>4}{flag}")
    print("\n  pulse_cv: 0 = metronome, ~1.0 = Poisson (no rhythm at all).")
    print("  p90/p50 is the PAUSE tail -- 'good strides then a stall' shows up here first.")
    print("  Never read pulse_cv alone: a barely-moving body emits beautifully regular twitches.")
