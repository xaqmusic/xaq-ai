#!/usr/bin/env python3
"""Swing-inefficiency analysis — the operator's three claims as numbers.

Claim 1 (EXTENSION): during forward swing the knee extends aggressively, moving the
  leg's CoG outward.  → mean Δhip2 and Δknee over swing bouts, split into the swing's
  first/second half, vs their stance-phase counterparts.
Claim 2 (REACTION): the extended swinging leg's hip1 torque yaws the chassis opposite
  (conservation of angular momentum).  → SIGNED yaw rate during a leg's swing,
  correlated with that leg's sweep direction; reported per leg as reaction gain.
  (The existing yaw_swing_excess instrument is MAGNITUDE-only and cannot see this.)
Claim 3 (MISSED PLANT): the knee must whip back down to plant; missing that moment
  loses the power stroke.  → knee angle at touchdown vs mid-stance (elevation deficit),
  and per-stance forward-GRF in the FIRST 5 ticks vs the rest of stance (early-stance
  purchase).  A "missed plant" = touchdown with knee still ≥ threshold above its
  mid-stance level; reported as miss rate + the transport cost of missed vs clean
  stances.

Usage: swingreport.py <tag1_glob> [tag2_glob ...]   e.g.  '/tmp/xaq_swing/ctl_s*.trace.jsonl'
"""
import glob, json, math, statistics, sys

LEGS = ["fl", "fr", "rl", "rr"]
WARMUP = 1500
REFRACT = 4


def corr(x, y):
    n = len(x)
    if n < 3: return 0.0
    mx, my = sum(x) / n, sum(y) / n
    sx = sum((v - mx) ** 2 for v in x) ** 0.5
    sy = sum((v - my) ** 2 for v in y) ** 0.5
    return sum((x[k] - mx) * (y[k] - my) for k in range(n)) / (sx * sy) if sx * sy else 0.0


def analyze(path):
    R = []
    prev_yaw = None
    for line in open(path):
        try: d = json.loads(line)
        except json.JSONDecodeError: continue
        if d.get("t", 0) < WARMUP: continue
        yr = 0.0
        if prev_yaw is not None:
            dy = d["yaw"] - prev_yaw
            while dy > math.pi:  dy -= 2 * math.pi
            while dy < -math.pi: dy += 2 * math.pi
            yr = dy
        prev_yaw = d["yaw"]
        d["_yr"] = yr
        R.append(d)
    if len(R) < 2000: return None
    out = {}
    for i, leg in enumerate(LEGS):
        # bouts from raw contact with refractory
        bouts = []          # (kind, start, end) over indices
        prev_c = 1 if R[0]["c"][i] else 0
        start = 0
        for k in range(1, len(R)):
            c = 1 if R[k]["c"][i] else 0
            if c != prev_c:
                bouts.append(("st" if prev_c else "sw", start, k))
                start = k; prev_c = c
        swings = [(a, b) for kind, a, b in bouts if kind == "sw" and b - a >= REFRACT]
        stances = [(a, b) for kind, a, b in bouts if kind == "st" and b - a >= REFRACT]
        if len(swings) < 30 or len(stances) < 30: continue
        # C1: joint excursions per swing, split halves; stance reference
        dh2_1, dkn_1, dh2_2, dkn_2 = [], [], [], []
        for a, b in swings:
            m = (a + b) // 2
            dh2_1.append(R[m]["h2"][i] - R[a]["h2"][i]); dkn_1.append(R[m]["kn"][i] - R[a]["kn"][i])
            dh2_2.append(R[b - 1]["h2"][i] - R[m]["h2"][i]); dkn_2.append(R[b - 1]["kn"][i] - R[m]["kn"][i])
        # C2: signed reaction — yaw rate during this leg's swing vs its h1 sweep direction
        yr_sw, h1d_sw = [], []
        for a, b in swings:
            for k in range(a + 1, b):
                yr_sw.append(R[k]["_yr"])
                h1d_sw.append(R[k]["h1"][i] - R[k - 1]["h1"][i])
        react_corr = corr(h1d_sw, yr_sw)
        react_gain = (statistics.mean(abs(v) for v in yr_sw) if yr_sw else 0.0)
        # C3: knee elevation at touchdown vs mid-stance + early-stance GRF purchase
        kn_td, kn_mid, early_g, late_g, miss, clean_e, miss_e = [], [], [], [], 0, [], []
        for a, b in stances:
            kn_td.append(R[a]["kn"][i])
            kn_mid.append(R[(a + b) // 2]["kn"][i])
            eg = sum(R[k]["grf"][i] for k in range(a, min(a + 5, b)))
            lg = sum(R[k]["grf"][i] for k in range(min(a + 5, b), b))
            early_g.append(eg); late_g.append(lg)
        el_def = [t - m for t, m in zip(kn_td, kn_mid)]        # + = knee still ELEVATED at td
        thr = statistics.pstdev(kn_mid) if len(kn_mid) > 1 else 0.05
        for d_, eg in zip(el_def, early_g):
            if d_ > thr: miss += 1; miss_e.append(eg)
            else: clean_e.append(eg)
        out[leg] = dict(
            n_sw=len(swings),
            dh2_rise=statistics.mean(dh2_1), dkn_rise=statistics.mean(dkn_1),
            dh2_fall=statistics.mean(dh2_2), dkn_fall=statistics.mean(dkn_2),
            react_corr=react_corr, react_mag=react_gain,
            el_def=statistics.mean(el_def), miss_rate=miss / len(stances),
            early_g_clean=statistics.mean(clean_e) if clean_e else 0.0,
            early_g_miss=statistics.mean(miss_e) if miss_e else 0.0,
            early_g=statistics.mean(early_g), late_g_pt=statistics.mean(late_g))
    return out


def main(argv):
    if not argv: print(__doc__); return 2
    for pat in argv:
        files = sorted(glob.glob(pat))
        per = [r for r in (analyze(f) for f in files) if r]
        print(f"\n=== {pat}  ({len(per)}/{len(files)} traces)")
        if not per: continue
        print(f"{'leg':<4}{'n_sw':>6}{'Δh2↑':>8}{'Δkn↑':>8}{'Δkn↓':>8}{'react_r':>9}"
              f"{'el_def':>8}{'miss%':>7}{'eG_clean':>10}{'eG_miss':>9}")
        for leg in LEGS:
            rs = [r[leg] for r in per if leg in r]
            if not rs: continue
            m = lambda k: statistics.mean(r[k] for r in rs)
            print(f"{leg:<4}{m('n_sw'):>6.0f}{m('dh2_rise'):>8.3f}{m('dkn_rise'):>8.3f}"
                  f"{m('dkn_fall'):>8.3f}{m('react_corr'):>9.3f}{m('el_def'):>8.3f}"
                  f"{m('miss_rate'):>7.1%}{m('early_g_clean'):>10.4f}{m('early_g_miss'):>9.4f}")
    print("\nΔh2↑/Δkn↑ = first-half-of-swing joint excursion (+ = extension/up); Δkn↓ = second half"
          "\nreact_r = corr(this leg's h1 sweep, SIGNED chassis yaw rate) during its swing"
          "\nel_def = knee elevation at touchdown above mid-stance level (+ = landed un-planted)"
          "\nmiss% = touchdowns with el_def > 1σ; eG = forward GRF impulse in first 5 stance ticks")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
