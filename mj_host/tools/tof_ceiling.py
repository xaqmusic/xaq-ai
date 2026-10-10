#!/usr/bin/env python3
"""tof_ceiling.py — a conditional mutual information reported WITH its ceiling and a bias floor.

Methodological fix (microduck design doc §17.29, 2026-09-13).  I(winner; object | pose) was once
reported as "a 92 % collapse" with no denominator, which read as a failure of the vocabulary when it
was a property of the room: pose alone fixed 93 % of what was in view.  The quantity any vocabulary
can reach is H(object | pose), so the conditional is reported as a fraction of that, and shuffling
the winner within each pose cell gives the estimator's own bias floor.

Usage:  tof_ceiling.py POOL.npz EPM.jsonl [--from S] [--shuffles N]
World-derived labels are INSTRUMENTATION: they judge the vocabularies and reach no brain.
"""
import argparse, json, sys
from collections import Counter
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from tof_prep import label_fov


def entropy(a):
    c = np.array(list(Counter(a).values()), float); p = c / c.sum()
    return float(-(p * np.log(p)).sum())


def mi(a, b):
    ua, ia = np.unique(a, return_inverse=True); ub, ib = np.unique(b, return_inverse=True)
    if len(ua) < 2 or len(ub) < 2: return 0.0
    M = np.zeros((len(ua), len(ub))); np.add.at(M, (ia, ib), 1.0); M /= len(a)
    pa = M.sum(1, keepdims=True); pb = M.sum(0, keepdims=True); nz = M > 0
    return float((M[nz] * np.log(M[nz] / (pa @ pb)[nz])).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz"); ap.add_argument("epm")
    ap.add_argument("--from", dest="t_from", type=float, default=700.0)
    ap.add_argument("--shuffles", type=int, default=8)
    a = ap.parse_args()
    rng = np.random.default_rng(0)
    d = np.load(a.npz, allow_pickle=True); d = {k: d[k] for k in d.files}
    best, _, _, _, kinds = label_fov(d, max_range=2.0)
    arms, ticks = None, []
    for line in open(a.epm):
        j = json.loads(line)
        if j.get("event") == "header": arms = j["arms"]
        elif j.get("event") == "tick": ticks.append(j)
    idx = np.array([tk["ln"] for tk in ticks])
    keep = np.array([tk["t"] for tk in ticks]) >= a.t_from
    idx, ticks = idx[keep], [t for t, k in zip(ticks, keep) if k]
    kind_of = dict(enumerate(kinds))
    O = np.array([kind_of.get(int(b), "none") if b >= 0 else "none" for b in best[idx]])
    octant = np.floor(((d["hdg"][idx, 0] % (2 * np.pi)) / (2 * np.pi)) * 8).astype(int)
    P = np.floor(d["x"][idx]).astype(int) * 1000 + np.floor(d["y"][idx]).astype(int) * 10 + octant
    cells = [c for c in np.unique(P) if (P == c).sum() >= 30]
    use = np.isin(P, cells)
    HO = entropy(O[use])
    HOP = sum((P[use] == c).mean() * entropy(O[use][P[use] == c]) for c in cells)
    print(f"frames {int(use.sum())}   pose cells {len(cells)}   H(object) {HO:.3f} nats   "
          f"H(object | pose) {HOP:.3f}  (pose alone fixes {100 * (1 - HOP / HO):.0f} %)")
    print(f"  {'view':12s} {'I(W;O)':>8s} {'I(W;O|pose)':>12s} {'of ceiling':>11s} {'shuffle floor':>14s}")

    def cmi(w, o, p, shuffle=False):
        tot = 0.0
        for c in cells:
            m = p == c
            if m.sum() < 30: continue
            ww = w[m].copy()
            if shuffle: rng.shuffle(ww)
            tot += m.mean() * mi(ww, o[m])
        return tot

    for name in arms:
        w = np.array([tk[name]["winner"] if tk[name] else -1 for tk in ticks])
        ok = (w >= 0) & use
        real = cmi(w[ok], O[ok], P[ok])
        floor = float(np.mean([cmi(w[ok], O[ok], P[ok], shuffle=True) for _ in range(a.shuffles)]))
        print(f"  {name:12s} {mi(w[ok], O[ok]):>8.3f} {real:>12.3f} {100 * real / HOP:>10.0f} % {floor:>14.3f}")


if __name__ == "__main__":
    main()
