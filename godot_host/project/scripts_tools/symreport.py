#!/usr/bin/env python3
"""Can this body LEARN a straight gait, instead of us tuning the symptom?

THE FINDING THIS EXISTS TO ACT ON (2026-08-04).  The picrawler's skid handedness is
**seed-random, not structural**: the right/left hip1 demand ratio across four seeds is
3.07 / 4.35 / 0.69 / 0.97 — seed 3 is LEFT-heavy — and it still flips with every controller
learning rate at zero.  So it is the 0.01*random initialisation of C, locked in by the gait
as a stable attractor, and the heading PD then spends the whole run fighting it.  That
standing effort is what pins one side near its clamp (right legs request 2.10 against a +-1
limit at 77% clip duty, versus 0.70 on the left), and THAT is why steering authority is
direction-dependent.

⚠ WHY THE OBVIOUS FIX IS THE WRONG ONE.  ~35 symmetry-forcing levers have been refuted here,
all the same way — circling — because they matched AMPLITUDE.  The ledger's own conclusion is
"amplitude symmetry != functional symmetry", and the emergent tripod-skid asymmetry is
recorded as LOAD-BEARING for straightness.  So the target here is not a symmetric body.  It is
**zero net yaw** — functional symmetry — reached by learning, with the asymmetry left intact.

The arms:
  base       the deployed stack
  stroke12   stroke_gain 1.65 -> 1.20.  The SYMPTOM treatment: more headroom below the clamp,
             so less of the commanded differential is rectified away.  Reference row.
  trim1/3    heading_trim_rate — THE MISSING INTEGRAL TERM.  The controller is P+D only, so a
             persistent disturbance leaves a nonzero steady-state steer forever.  An
             integrator learns that DC effort and hands it back.  Leak + clamp mandatory.
  pairinit   c_pair_init — give left/right partners the SAME initial control law, so no side
             starts with an advantage.  Acts on the CAUSE (the init) rather than the effect.
  symlow     ctrl_symmetry_gain=0.02 — the refuted parameter-coupling lever, retried WEAK.
             Its refutation predates knowing the handedness is random; included as the
             honest re-test of a re-use context, not as a new idea.
  trim1pair  the two new levers together — only meaningful if either works alone.

READ IT ON THE FULL SET.  `straight` alone is blind: a body that walks a perfect arc scores
well on |turns| if the arc closes, and a body that barely moves scores well on both.  net_z
and falls are the do-no-harm guard, and R/L is the mechanism check — a lever that improves
straightness WITHOUT moving R/L is doing something other than what it claims.

Usage: symreport.py <dir>
"""
import glob, json, math, os, statistics as st, sys

WARM = 1500


def leg_ratio(rows):
    w = [x for x in rows if x["t"] > WARM and "pre_h1_leg" in x]
    if not w:
        return float("nan")
    pr = [st.mean(x["pre_h1_leg"][i] for x in w) for i in range(4)]
    L, R = (pr[0] + pr[2]) / 2, (pr[1] + pr[3]) / 2
    return R / max(L, 1e-6)


def turns(rows):
    hy = [x["heading_yaw"] for x in rows]
    acc, p = 0.0, hy[0]
    for h in hy[1:]:
        d = h - p
        while d > math.pi:  d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        acc += d; p = h
    return acc / (2 * math.pi)


def straight(rows):
    xs = [x["x"] for x in rows]; zs = [x["z"] for x in rows]
    path = sum(math.hypot(xs[i] - xs[i - 1], zs[i] - zs[i - 1]) for i in range(1, len(xs)))
    net = math.hypot(xs[-1] - xs[0], zs[-1] - zs[0])
    return net / path if path > 1e-6 else 0.0


def arm(d, tag):
    out = []
    for f in sorted(glob.glob(os.path.join(d, f"{tag}_*.log"))):
        rows = [json.loads(l) for l in open(f)
                if l.startswith("{") and '"x":' in l]
        if len(rows) < 20:
            continue
        out.append(dict(straight=straight(rows), turns=turns(rows),
                        ratio=leg_ratio(rows), net_z=rows[-1]["z"] - rows[0]["z"],
                        falls=max(x.get("auto_reset_count", 0) for x in rows),
                        trim=rows[-1].get("heading_trim", 0.0)))
    return out


if __name__ == "__main__":
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    ARMS = [("__base", "base (deployed)"), ("__stroke12", "stroke 1.20 (symptom)"),
            ("__trim1", "I term 1e-4"), ("__trim3", "I term 3e-4"),
            ("__pairinit", "paired L/R init"), ("__symlow", "ctrl_symmetry 0.02"),
            ("__trim1pair", "I term + paired")]
    base = arm(d, "__base")
    b_asym = st.mean(abs(x["ratio"] - 1.0) for x in base) if base else float("nan")
    print(f"\n  {'arm':<22}{'straight':>9}{'|turns|':>9}{'R/L':>7}"
          f"{'|R/L-1|':>9}{'net_z':>8}{'falls':>7}{'trim':>8}")
    for tag, lbl in ARMS:
        a = arm(d, tag)
        if not a:
            print(f"  {lbl:<22}   (no data)"); continue
        asym = st.mean(abs(x["ratio"] - 1.0) for x in a)
        flag = ""
        if base:
            if asym < b_asym * 0.7: flag += "  << ASYMMETRY DOWN"
            if st.mean(x["net_z"] for x in a) < st.mean(x["net_z"] for x in base) * 0.85:
                flag += "  << net_z REGRESSION"
        print(f"  {lbl:<22}{st.mean(x['straight'] for x in a):>9.3f}"
              f"{st.mean(abs(x['turns']) for x in a):>9.3f}"
              f"{st.mean(x['ratio'] for x in a):>7.2f}{asym:>9.2f}"
              f"{st.mean(x['net_z'] for x in a):>8.2f}"
              f"{st.mean(x['falls'] for x in a):>7.2f}"
              f"{st.mean(x['trim'] for x in a):>8.3f}{flag}")
        print(f"  {'':22}per-seed R/L " + " ".join(f"{x['ratio']:.2f}" for x in a))
    print("\n  n=4 fixed-seed = a SIGNAL (promote-or-kill), never a finding (CLAUDE.md §3).")
    print("  A lever that moves `straight` without moving R/L is not doing what it claims.")
