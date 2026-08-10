#!/usr/bin/env python3
"""P1 shadow-phase scoring — which candidate phase deserves the stroke?

The audit found L.phase retrograde 2/3 of ticks; three shadow candidates now run
as zero-authority instruments inside MotorEPMv2 (sh_a per-leg PLL, sh_b shared
CPG+offsets, sh_c delay-compensated filter), exported per tick when
OGMA_PICRAWLER_DIAG_INTERVAL=1, beside the incumbent (ph_l).

Scores per candidate, per leg, aggregated over seeds:
  retro    — fraction of ticks the phase runs backwards (monotone ≈ 0)
  td_plv   — resultant length of the phase at TRUE-contact touchdown events
             (the campaign's honest lock metric: does touchdown land at one phase?)
  prop_R   — resultant length over stance-ticks-sweeping-propulsively (flbrake's
             window): a phase the stroke could ride should CONCENTRATE there
  n_td     — touchdown count (support; small n makes td_plv meaningless)

Usage:
  phasescore.py collect <config.json> [n=6] [steps=12000] [diff=0.3]   # runs seeds
  phasescore.py score                                                   # scores $PHASESCORE_OUT
Output dir: $PHASESCORE_OUT (default /tmp/xaq_phasescore).
"""
import json, math, os, statistics, subprocess, sys, concurrent.futures as cf
import pathlib

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
OUT  = os.environ.get("PHASESCORE_OUT", "/tmp/xaq_phasescore")
LEGS = ["fl", "fr", "rl", "rr"]
CANDS = ["ph_l", "sh_a", "sh_b", "sh_c"]


def run_seed(cfg, seed, steps, diff):
    log   = f"{OUT}/ps_s{seed}.log"
    trace = f"{OUT}/ps_s{seed}.trace.jsonl"
    env = dict(os.environ,
               OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7400 + seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(diff),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous",
               OGMA_PICRAWLER_MAX_STEPS=str(steps),
               OGMA_PICRAWLER_CHASSIS_COLLIDE="1",
               OGMA_PICRAWLER_DIAG_INTERVAL="1",
               OGMA_PICRAWLER_TRACE=trace)
    with open(log, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60",
                        "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"],
                       cwd=PROJ, env=env, stdout=f, stderr=subprocess.STDOUT)
    return log


def corr(x, y):
    n = len(x)
    if n < 3: return 0.0
    mx, my = sum(x) / n, sum(y) / n
    sx = sum((v - mx) ** 2 for v in x) ** 0.5
    sy = sum((v - my) ** 2 for v in y) ** 0.5
    return sum((x[k] - mx) * (y[k] - my) for k in range(n)) / (sx * sy) if sx * sy else 0.0


def resultant(phis):
    if not phis: return 0.0
    c = sum(math.cos(p) for p in phis) / len(phis)
    s = sum(math.sin(p) for p in phis) / len(phis)
    return math.hypot(c, s)


def score_seed(seed, warmup=1000, dead=0.0002):
    log, tracep = f"{OUT}/ps_s{seed}.log", f"{OUT}/ps_s{seed}.trace.jsonl"
    phases = {}                                    # t -> {cand: [4 phases]}
    for line in open(log):
        line = line.strip()
        if '"sh_a"' not in line or not line.startswith('{'): continue
        try: d = json.loads(line)
        except json.JSONDecodeError: continue
        if d.get("t", 0) < warmup: continue
        phases[d["t"]] = {c: d[c] for c in CANDS if c in d}
    R = []
    for line in open(tracep):
        try: d = json.loads(line)
        except json.JSONDecodeError: continue
        if d.get("t", 0) >= warmup: R.append(d)
    if len(R) < 500 or len(phases) < 500: return None
    V = [r["fwd_v"] for r in R[1:]]
    out = {c: {leg: dict(td=[], prop=[], retro=[0, 0]) for leg in LEGS} for c in CANDS}
    for i, leg in enumerate(LEGS):
        sw = [(R[k]["h1"][i] - R[k - 1]["h1"][i]) * (1.0 if R[k]["c"][i] else 0.0)
              for k in range(1, len(R))]
        s = 1.0 if corr(sw, V) >= 0 else -1.0
        prev = {c: None for c in CANDS}
        for k in range(1, len(R)):
            t = R[k]["t"]
            ph = phases.get(t)
            if not ph: continue
            for c in CANDS:
                if c not in ph or len(ph[c]) <= i: continue
                v = ph[c][i]
                if prev[c] is not None:
                    dd = v - prev[c]
                    while dd > math.pi:  dd -= 2 * math.pi
                    while dd < -math.pi: dd += 2 * math.pi
                    out[c][leg]["retro"][0] += 1 if dd < 0 else 0
                    out[c][leg]["retro"][1] += 1
                prev[c] = v
                if R[k]["c"][i] and not R[k - 1]["c"][i]:          # touchdown
                    out[c][leg]["td"].append(v)
                if R[k]["c"][i] and s * (R[k]["h1"][i] - R[k - 1]["h1"][i]) > dead:
                    out[c][leg]["prop"].append(v)                  # propulsive stance tick
    return out


def main(argv):
    os.makedirs(OUT, exist_ok=True)
    if not argv: print(__doc__); return 2
    if argv[0] == "collect":
        cfg   = argv[1]
        n     = int(argv[2])   if len(argv) > 2 else 6
        steps = int(argv[3])   if len(argv) > 3 else 12000
        diff  = float(argv[4]) if len(argv) > 4 else 0.3
        seeds = list(range(1, n + 1))
        with cf.ThreadPoolExecutor(max_workers=min(8, n)) as ex:
            list(ex.map(lambda s: run_seed(cfg, s, steps, diff), seeds))
        print(f"collected {n} seeds into {OUT}")
        return 0
    if argv[0] == "score":
        seeds = sorted(int(p.name[4:-4]) for p in pathlib.Path(OUT).glob("ps_s*.log"))
        per = [r for r in (score_seed(s) for s in seeds) if r]
        if not per: print("no scorable seeds"); return 1
        print(f"phasescore: {len(per)}/{len(seeds)} seeds  "
              f"(td_plv/prop_R = resultant length, 1 = perfectly locked)")
        print(f"{'cand':<7}{'leg':<5}{'retro':>8}{'td_plv':>9}{'n_td':>7}{'prop_R':>9}")
        summary = {}
        for c in CANDS:
            rs, tds, prs = [], [], []
            for leg in LEGS:
                retro = [r[c][leg]["retro"][0] / max(1, r[c][leg]["retro"][1]) for r in per]
                tdp   = [resultant(r[c][leg]["td"])   for r in per if len(r[c][leg]["td"]) >= 20]
                prp   = [resultant(r[c][leg]["prop"]) for r in per if len(r[c][leg]["prop"]) >= 50]
                ntd   = sum(len(r[c][leg]["td"]) for r in per)
                print(f"{c:<7}{leg:<5}{statistics.mean(retro):>8.3f}"
                      f"{(statistics.mean(tdp) if tdp else -1):>9.3f}{ntd:>7}"
                      f"{(statistics.mean(prp) if prp else -1):>9.3f}")
                rs += retro; tds += tdp; prs += prp
            summary[c] = (statistics.mean(rs),
                          statistics.mean(tds) if tds else -1,
                          statistics.mean(prs) if prs else -1)
        print("-" * 45)
        for c, (r, t, p) in summary.items():
            print(f"{c:<7}mean retro={r:.3f}  td_plv={t:.3f}  prop_R={p:.3f}")
        return 0
    print(__doc__); return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
