#!/usr/bin/env python3
"""M0 — the planner's raw-material gate (PART III).

Builds the empirical winner-transition matrix from the first half of each run's
body-pose token stream, then scores K-STEP ROLLOUT ACCURACY on the second half:
from state s_t, roll the argmax chain k steps, compare to the actual s_{t+k}.
Baseline = PERSISTENCE (predict s_{t+k} = s_t).

GATE: the transition model must beat persistence decisively at k ≥ 3, or
vocabulary conditioning precedes any planner code.

Also reports: vocabulary size in use, occupancy entropy, self-transition mass
(a chain that mostly self-loops IS persistence and cannot plan past it).

Usage: planscore.py '/tmp/xaq_m0/m0_s*.log' [tag=bp_win]
"""
import glob, json, math, statistics, sys
from collections import defaultdict


def analyze(path, tag, warmup=3000):
    seq = []
    for line in open(path):
        line = line.strip()
        if f'"{tag}"' not in line or not line.startswith('{'): continue
        try: d = json.loads(line)
        except json.JSONDecodeError: continue
        if d.get("t", 0) < warmup: continue
        w = d.get(tag, -1)
        if w >= 0: seq.append((d["t"], w))
    if len(seq) < 4000: return None
    # contiguous per-tick stream only (diag interval 1)
    toks = [w for _, w in seq]
    half = len(toks) // 2
    train, test = toks[:half], toks[half:]
    # empirical transition counts
    trans = defaultdict(lambda: defaultdict(int))
    for a, b in zip(train, train[1:]):
        trans[a][b] += 1
    argmax = {a: max(nb, key=nb.get) for a, nb in trans.items()}
    self_mass = sum(nb.get(a, 0) for a, nb in trans.items()) / max(1, len(train) - 1)
    used = len(set(train))
    occ = defaultdict(int)
    for w in train: occ[w] += 1
    n = len(train)
    ent = -sum((c / n) * math.log(c / n) for c in occ.values())
    out = dict(used=used, entropy=ent, self_mass=self_mass, n=len(toks))
    for k in (1, 2, 3, 5, 8, 10):
        hit_m = hit_p = tot = 0
        for i in range(0, len(test) - k):
            s = test[i]
            cur = s
            ok = True
            for _ in range(k):
                if cur not in argmax: ok = False; break
                cur = argmax[cur]
            if not ok: continue
            tot += 1
            hit_m += 1 if cur == test[i + k] else 0
            hit_p += 1 if s == test[i + k] else 0
        out[f"k{k}"] = (hit_m / tot if tot else 0.0, hit_p / tot if tot else 0.0)
    return out


def main(argv):
    if not argv: print(__doc__); return 2
    pat = argv[0]; tag = argv[1] if len(argv) > 1 else "bp_win"
    per = [r for r in (analyze(f, tag) for f in sorted(glob.glob(pat))) if r]
    if not per: print("no scorable runs"); return 1
    print(f"planscore [{tag}]: {len(per)} runs  "
          f"vocab-in-use {statistics.mean(r['used'] for r in per):.0f}  "
          f"entropy {statistics.mean(r['entropy'] for r in per):.2f} nats  "
          f"self-transition mass {statistics.mean(r['self_mass'] for r in per):.2f}")
    print(f"{'k':>3}{'model':>9}{'persist':>9}{'lift':>7}")
    for k in (1, 2, 3, 5, 8, 10):
        m = statistics.mean(r[f"k{k}"][0] for r in per)
        p = statistics.mean(r[f"k{k}"][1] for r in per)
        print(f"{k:>3}{m:>9.3f}{p:>9.3f}{(m/p if p else 0):>7.2f}")
    print("\nGATE: model must beat persistence decisively at k>=3.  High self-transition"
          "\nmass means the chain IS persistence — conditioning work before planner code.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
