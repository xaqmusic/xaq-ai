#!/usr/bin/env python3
"""Why does a leg BRAKE?  Stance-GRF decomposition by stroke direction.

The stance-hip2 lever (ledger §1, PARTIAL) recruited fr as a propulsor and
deepened fl's brake 2.5×, so net forward force FELL while every count-style
metric improved.  This tool asks the mechanism question the verdict left open:
when a leg's stance-phase forward GRF is negative, is the leg

  (a) RECOVERING WHILE PLANTED — hip1 sweeping in the swing direction with the
      foot still on the ground (late liftoff: a TIMING pathology, the
      stroke-to-step thread), or
  (b) DRAGGING — GRF negative even while the hip1 sweeps in the propulsive
      direction (scrub/geometry: the foot slips or the stroke fights the
      body's motion).

Consumes the per-seed attribution traces actsweep.py banks (or any
OGMA_PICRAWLER_TRACE output).  God's-eye instrument; §5.3: diagnostics only.

Usage:
  flbrake.py <trace.jsonl> [trace2.jsonl ...] [warmup=1000] [deadband=0.0002]

Per-leg, aggregated over the given traces (mean ± std across traces):
  duty     — stance fraction of ticks
  g/tick   — net forward GRF impulse per tick (the brake/propulsor read)
  %rec     — stance ticks spent sweeping in the RECOVERY direction
  g_prop   — GRF per tick while sweeping propulsively   (negative = dragging)
  g_rec    — GRF per tick while sweeping in recovery    (the late-liftoff cost)
"""
import json, statistics, sys

LEGS = ["fl", "fr", "rl", "rr"]


def corr(x, y):
    n = len(x)
    if n < 3: return 0.0
    mx, my = sum(x) / n, sum(y) / n
    sx = sum((v - mx) ** 2 for v in x) ** 0.5
    sy = sum((v - my) ** 2 for v in y) ** 0.5
    return sum((x[k] - mx) * (y[k] - my) for k in range(n)) / (sx * sy) if sx * sy else 0.0


def analyze(path, warmup, dead):
    R = []
    for line in open(path):
        try: d = json.loads(line)
        except json.JSONDecodeError: continue
        if d.get("t", 0) >= warmup: R.append(d)
    if len(R) < 500: return None
    V = [r["fwd_v"] for r in R[1:]]
    # Propulsive hip1 sign per leg, DERIVED from the data (leg_attribution.py's
    # method): the sign that makes stance-gated dh1 co-move with fwd_v.
    signs = []
    for i in range(4):
        sw = [(R[k]["h1"][i] - R[k - 1]["h1"][i]) * (1.0 if R[k]["c"][i] else 0.0)
              for k in range(1, len(R))]
        signs.append(1.0 if corr(sw, V) >= 0 else -1.0)
    out = {}
    for i, leg in enumerate(LEGS):
        st = prop_t = rec_t = 0
        g_all = g_prop = g_rec = 0.0
        for k in range(1, len(R)):
            g = R[k]["grf"][i]
            g_all += g
            if not R[k]["c"][i]: continue
            st += 1
            dh1 = signs[i] * (R[k]["h1"][i] - R[k - 1]["h1"][i])
            if   dh1 >  dead: prop_t += 1; g_prop += g
            elif dh1 < -dead: rec_t  += 1; g_rec  += g
        n = len(R) - 1
        # LIFTOFF LAG — "late liftoff" as a number.  For each liftoff (contact 1→0),
        # walk back to the most recent reversal of the (smoothed) stroke from
        # propulsive to recovery; the gap is how long the leg stayed planted after
        # its stroke turned around.  0 = liftoff at reversal (ideal); large = the
        # braking window flbrake measures as g_rec.
        W = 5
        dh = [signs[i] * (R[k]["h1"][i] - R[k - 1]["h1"][i]) for k in range(1, len(R))]
        sm = []
        s = 0.0
        for k in range(len(dh)):
            s += dh[k]
            if k >= W: s -= dh[k - W]
            sm.append(s / min(k + 1, W))
        lags = []
        for k in range(1, len(R) - 1):
            if R[k]["c"][i] and not R[k + 1]["c"][i]:          # liftoff at k+1
                back = k
                while back > 1 and not (sm[back - 1] > 0 >= sm[back]):
                    back -= 1
                    if k - back > 120: back = -1; break         # no reversal this bout
                if back > 0: lags.append(k + 1 - back)
        out[leg] = dict(duty=st / n, g_tick=g_all / n,
                        pct_rec=rec_t / st if st else 0.0,
                        g_prop=g_prop / prop_t if prop_t else 0.0,
                        g_rec=g_rec / rec_t if rec_t else 0.0,
                        lag=statistics.mean(lags) if lags else -1.0,
                        lag_n=len(lags),
                        sign=signs[i])
    return out


def main(argv):
    paths = [a for a in argv if not a.replace(".", "").isdigit()]
    nums  = [a for a in argv if a.replace(".", "").isdigit()]
    warmup = int(nums[0])   if len(nums) > 0 else 1000
    dead   = float(nums[1]) if len(nums) > 1 else 0.0002
    if not paths:
        print(__doc__); return 2
    per = [r for r in (analyze(p, warmup, dead) for p in paths) if r]
    if not per:
        print("no usable traces"); return 1
    print(f"flbrake: {len(per)}/{len(paths)} traces, warmup={warmup}, deadband={dead}")
    print(f"{'leg':<5}{'duty':>8}{'g/tick':>12}{'%rec':>8}{'g_prop':>12}{'g_rec':>12}"
          f"{'lag':>8}   read")
    for leg in LEGS:
        vals = {m: [r[leg][m] for r in per]
                for m in ("duty", "g_tick", "pct_rec", "g_prop", "g_rec")}
        mu = {m: statistics.mean(v) for m, v in vals.items()}
        sd = {m: statistics.pstdev(v) for m, v in vals.items()}
        lg = [r[leg]["lag"] for r in per if r[leg]["lag"] >= 0]
        lag = statistics.mean(lg) if lg else -1.0
        if   mu["g_tick"] >= 0 and mu["g_prop"] >= 0: read = "propulsor"
        elif mu["g_rec"] < mu["g_prop"] and mu["g_prop"] > 0: read = "brake: LATE LIFTOFF (timing)"
        elif mu["g_prop"] < 0: read = "brake: DRAGS while sweeping (scrub/geometry)"
        else: read = "brake: mixed"
        print(f"{leg:<5}{mu['duty']:>8.3f}{mu['g_tick']:>12.5f}{mu['pct_rec']:>8.1%}"
              f"{mu['g_prop']:>12.5f}{mu['g_rec']:>12.5f}{lag:>8.1f}   {read}"
              f"   (±{sd['g_tick']:.5f} g/tick over traces)")
    print("lag = mean ticks from stroke reversal (prop→rec) to liftoff; 0 = ideal timing")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
