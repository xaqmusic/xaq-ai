#!/usr/bin/env python3
"""What does a coordination number MEAN on this body?  The reference ladder for `plv_w`.

The campaign has treated inter-leg PLV as a scale whose top is a trot and whose bottom is
chance, and read the deployed gait's ~0.20 as "essentially uncoordinated".  Only two points
on that scale were ever established, and both by SIMULATION rather than by the robot:

    random-phase null   0.090 +- 0.066     (40 sims, independent legs)
    perfect lock        0.984              (identical frequencies)

Everything in between was assumed.  This tool fills in the middle with arms that have a
KNOWN answer, so a reading can be interpreted instead of admired:

  * SCRIPTED TROT / WALK — an open-loop, perfectly periodic gait with imposed per-leg phase
    offsets.  This is the decisive one.  A designed gait is coordinated BY CONSTRUCTION, so
    if it also scores ~0.2 the number is telling us about the BODY or the INSTRUMENT, not
    about our controller, and the entire "nothing here is coordinated" line needs revisiting.
  * ONE REAR LEG ABLATED — the operator's UI observation is that the resulting tripod looks
    FASTER than the intact gait.  If that holds, the fourth leg is interfering.

⚠ WHY PER-PAIR.  The pooled mean cannot answer the lesion question: with one leg dead its
three pairs decay toward 0 and swamp the survivors, so a perfectly coordinated three-legged
gait still reads about half the pooled value.  This scores the surviving legs AMONG
THEMSELVES.  Pair order is (0,1)(0,2)(0,3)(1,2)(1,3)(2,3) over legs [FL, FR, RL, RR].

Usage: plvladder.py <label>=<logglob> [...]  [--ablated N]
"""
import glob, json, math, statistics, sys

PAIRS = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
LEGS = ["FL", "FR", "RL", "RR"]
NULL, LOCKED = 0.090, 0.984
WARMUP = 900


def load(path):
    rows = []
    for line in open(path):
        if not line.startswith("{") or '"x":' not in line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        if d.get("t", 0) >= WARMUP:
            rows.append(d)
    return rows


def arm(label, pattern, ablated):
    seeds = [load(p) for p in sorted(glob.glob(pattern))]
    seeds = [s for s in seeds if len(s) > 20]
    if not seeds:
        return None
    out = {"label": label, "n": len(seeds)}
    pooled, surv, per = [], [], [[] for _ in PAIRS]
    speed, steps, cv, sup = [], [], [], []
    for s in seeds:
        pw = [r["plv_pairs"] for r in s if isinstance(r.get("plv_pairs"), list)
              and len(r["plv_pairs"]) == 6]
        if pw:
            m = [statistics.mean(v[i] for v in pw) for i in range(6)]
            for i in range(6):
                per[i].append(m[i])
            pooled.append(statistics.mean(m))
            # Survivor pairs only: those touching neither ablated leg.
            keep = [i for i, (a, b) in enumerate(PAIRS)
                    if a not in ablated and b not in ablated]
            if keep:
                surv.append(statistics.mean(m[i] for i in keep))
        z = [r["z"] for r in s]
        t = [r["t"] for r in s]
        speed.append((z[-1] - z[0]) / max(1, t[-1] - t[0]) * 60.0)
        ll = [r["leg_lifted_counts"] for r in s if isinstance(r.get("leg_lifted_counts"), list)]
        if ll:
            steps.append(sum(a - b for a, b in zip(ll[-1], ll[0])))
        cvv = [r["step_cv"] for r in s if "step_cv" in r]
        if cvv:
            cv.append(cvv[-1])
        spv = [r["plv_wn"] for r in s if "plv_wn" in r]
        if spv:
            sup.append(statistics.mean(spv))
    ms = lambda v: (statistics.mean(v), statistics.pstdev(v)) if v else (float("nan"),) * 2
    out.update(pooled=ms(pooled), surv=ms(surv), speed=ms(speed), steps=ms(steps),
               cv=ms(cv), sup=ms(sup),
               per=[statistics.mean(p) if p else float("nan") for p in per])
    return out


if __name__ == "__main__":
    argv = sys.argv[1:]
    ablated = set()
    if "--ablated" in argv:
        i = argv.index("--ablated")
        ablated = {int(x) for x in argv[i + 1].split(",")}
        del argv[i:i + 2]          # drop the flag AND its value
    args = [a for a in argv if "=" in a]
    if not args:
        print(__doc__); sys.exit(2)
    arms = [a for a in (arm(s.split("=", 1)[0], s.split("=", 1)[1], ablated) for s in args) if a]
    print(f"\n  REFERENCE LADDER — random-phase null {NULL:.3f} · perfect lock {LOCKED:.3f}")
    print(f"  {'arm':<14}{'plv_w':>14}{'survivors':>14}{'support':>9}"
          f"{'m/s':>9}{'steps':>8}{'step_cv':>9}")
    for a in arms:
        f = lambda k: (f"{a[k][0]:.3f}±{a[k][1]:.3f}" if not math.isnan(a[k][0]) else "  --  ")
        print(f"  {a['label']:<14}{f('pooled'):>14}{f('surv'):>14}"
              f"{a['sup'][0]:>9.3f}{a['speed'][0]:>9.4f}{a['steps'][0]:>8.1f}{a['cv'][0]:>9.3f}")
    print(f"\n  per-pair plv_w  ({'  '.join(f'{LEGS[i]}-{LEGS[j]}' for i, j in PAIRS)})")
    for a in arms:
        print(f"  {a['label']:<14}" + "  ".join(f"{v:>6.3f}" for v in a["per"]))
    print("\n  Read `plv_w` ONLY beside support: a frozen or fallen body holds a constant"
          "\n  relative phase trivially, and low support means the number is unmeasured.")
