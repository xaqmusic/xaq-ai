#!/usr/bin/env python3
"""P5 anticipation gate — does body-pose transition surprise LEAD touchdowns?

The P5 predictor's promote-or-kill (locomotion_substrate_repair_plan.md): before any
consumer exists, the body-pose EPM's surprise must ANTICIPATE debounced touchdowns —
rise BEFORE contact — not merely react (the support-EPM failure, 2026-08-07).  Two
instrument EPMs ride the arm: bp (contract 0.7/0.3 tle mix) and bpt (transition-heavy
0.3/0.7).

collect: n seeds in the OPEN ARENA (gait-quality gym), per-tick diag + trace.
score:   per EPM — event-triggered tle curve around touchdowns (lags −20..+20, per-seed
         z-normalized), anticipation index (mean lift in lags −10..−1 over the −20..−15
         baseline), reactive index (lags 0..+10), and predictive LIFT:
         P(touchdown within 10 ticks | tle > q75) / base rate.
         Plus vocabulary health (nodes/baked; collapse = conditioning problem, §0 rule 2).

Usage:
  p5score.py collect <config.json> [n=6] [steps=12000]
  p5score.py score
Output dir: $P5SCORE_OUT (default /tmp/xaq_p5score).
"""
import json, math, os, statistics, subprocess, sys, concurrent.futures as cf
import pathlib

PROJ = str(pathlib.Path(__file__).resolve().parents[1])
OUT  = os.environ.get("P5SCORE_OUT", "/tmp/xaq_p5score")
LEGS = 4
WARMUP = 3000          # generous: vocabulary growth dominates tle before this
REFRACT = 4            # offline touchdown debounce (drop re-edges within 4 ticks)
LAGS = list(range(-20, 21))


def run_seed(cfg, seed, steps):
    log   = f"{OUT}/p5_s{seed}.log"
    trace = f"{OUT}/p5_s{seed}.trace.jsonl"
    env = dict(os.environ,
               OGMA_PICRAWLER_GYM="arena", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7400 + seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY="0.0",
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


def score_seed(seed):
    log, tracep = f"{OUT}/p5_s{seed}.log", f"{OUT}/p5_s{seed}.trace.jsonl"
    tle = {}                                   # t -> {"bp": v, "bpt": v}
    health = {}
    for line in open(log):
        line = line.strip()
        if '"bp_tle"' not in line or not line.startswith('{'): continue
        try: d = json.loads(line)
        except json.JSONDecodeError: continue
        t = d.get("t", 0)
        tle[t] = {"bp": d.get("bp_tle", -1.0), "bpt": d.get("bpt_tle", -1.0)}
        health = {k: d.get(k, -1) for k in ("bp_n", "bp_b", "bpt_n", "bpt_b")}
    tds = []                                   # (t, leg) debounced touchdowns
    prev = [0] * LEGS; last = [-10**9] * LEGS
    maxt = 0
    for line in open(tracep):
        try: d = json.loads(line)
        except json.JSONDecodeError: continue
        t = d.get("t", 0); maxt = max(maxt, t)
        for i in range(LEGS):
            c = 1 if d["c"][i] else 0
            if c and not prev[i] and t - last[i] >= REFRACT:
                if t >= WARMUP: tds.append((t, i))
                last[i] = t
            prev[i] = c
    ok_t = [t for t in tle if t >= WARMUP]
    if len(ok_t) < 2000 or len(tds) < 100: return None
    out = {"n_td": len(tds), "health": health, "run_ticks": maxt}
    for tag in ("bp", "bpt"):
        vals = [tle[t][tag] for t in ok_t if tle[t][tag] >= 0]
        if not vals or len(vals) < 2000: continue
        mu, sd = statistics.mean(vals), statistics.pstdev(vals) or 1.0
        # event-triggered z-curve
        curve = {L: [] for L in LAGS}
        for (t, _leg) in tds:
            for L in LAGS:
                v = tle.get(t + L, {}).get(tag, -1.0)
                if v >= 0: curve[L].append((v - mu) / sd)
        cm = {L: statistics.mean(curve[L]) for L in LAGS if curve[L]}
        base  = statistics.mean(cm[L] for L in range(-20, -14) if L in cm)
        antic = statistics.mean(cm[L] for L in range(-10, 0)  if L in cm) - base
        react = statistics.mean(cm[L] for L in range(0, 11)   if L in cm) - base
        # predictive lift: P(td within 10 | tle>q75) / base rate
        q75 = sorted(vals)[int(0.75 * len(vals))]
        td_ticks = set(t for (t, _) in tds)
        hits = tot = 0; bhits = btot = 0
        for t in ok_t:
            v = tle[t][tag]
            if v < 0: continue
            fired = any((t + k) in td_ticks for k in range(1, 11))
            btot += 1; bhits += 1 if fired else 0
            if v > q75:
                tot += 1; hits += 1 if fired else 0
        lift = (hits / tot) / (bhits / btot) if tot and btot and bhits else 0.0
        out[tag] = dict(antic=antic, react=react, lift=lift, curve=cm)
    return out


def main(argv):
    os.makedirs(OUT, exist_ok=True)
    if not argv: print(__doc__); return 2
    if argv[0] == "collect":
        cfg = argv[1]; n = int(argv[2]) if len(argv) > 2 else 6
        steps = int(argv[3]) if len(argv) > 3 else 12000
        with cf.ThreadPoolExecutor(max_workers=min(8, n)) as ex:
            list(ex.map(lambda s: run_seed(cfg, s, steps), range(1, n + 1)))
        print(f"collected {n} seeds into {OUT}")
        return 0
    if argv[0] == "score":
        seeds = sorted(int(p.name[4:-4]) for p in pathlib.Path(OUT).glob("p5_s*.log"))
        per = []
        for s in seeds:
            r = score_seed(s)
            if r is None: print(f"  seed {s}: unusable (short/truncated)"); continue
            per.append(r)
            print(f"  seed {s}: run_ticks={r['run_ticks']} n_td={r['n_td']} health={r['health']}")
        if not per: print("no scorable seeds"); return 1
        print(f"\np5score: {len(per)} seeds  (antic/react = z-lift vs −20..−15 baseline; "
              f"gate: antic > 0 decisively, not a reactive-spike shadow)")
        print(f"{'epm':<5}{'antic':>9}{'react':>9}{'lift':>7}")
        for tag in ("bp", "bpt"):
            rs = [r[tag] for r in per if tag in r]
            if not rs: print(f"{tag:<5}  (no data)"); continue
            a = statistics.mean(r["antic"] for r in rs)
            b = statistics.mean(r["react"] for r in rs)
            l = statistics.mean(r["lift"]  for r in rs)
            per_seed = "  ".join("%+.3f" % r["antic"] for r in rs)
            print(f"{tag:<5}{a:>9.3f}{b:>9.3f}{l:>7.2f}   (per-seed antic: {per_seed})")
        # mean curve for the eyeball
        for tag in ("bp", "bpt"):
            rs = [r[tag]["curve"] for r in per if tag in r]
            if not rs: continue
            print(f"\n{tag} event-triggered z-curve (lag: mean):")
            row = []
            for L in LAGS:
                vs = [c[L] for c in rs if L in c]
                if vs: row.append(f"{L:+d}:{statistics.mean(vs):+.2f}")
            print("  " + "  ".join(row))
        return 0
    print(__doc__); return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
