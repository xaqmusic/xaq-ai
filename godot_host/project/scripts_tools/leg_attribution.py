#!/usr/bin/env python3
"""Which leg is responsible for a forward-velocity pulse?

Consumes the per-tick trace written by picrawler_body.gd when
OGMA_PICRAWLER_TRACE names a path.  Usage:

    python3 leg_attribution.py <trace.jsonl> [warmup_ticks=1000] [pulse_smooth=30]

WHY KINEMATIC AND NOT TORQUE.  The obvious metric is mechanical power tau*omega,
and it does not work here.  `reality.proprio.joint_torque` is not applied torque
-- it is the PD value used to set the motor's impulse CAP, dominated by its
-Kd*omega damping term (measured corr with joint motion -0.46..-0.56), so it is
mostly a negated velocity copy.  Its honest replacement, `joint_load` (velocity-
tracking deficit), is a good LOAD signal but load*omega is not power either: a
deficit is anti-correlated with omega by construction.  Propulsion is kinematic
-- a STANCE leg sweeping its foot backward drives the body forward -- and that
needs neither torque nor any assumption about delivered force.

⚠ GOD'S-EYE INSTRUMENT.  fwd_v and foot contact are not egocentric.  That is
legal for a diagnostic and NOT legal for control (CLAUDE.md §5.3).  Nothing
computed here may be fed back into the brain.  The egocentric counterpart is
`reality.proprio.joint_load`.
"""
import json, random, sys

LEGS = ['fl', 'fr', 'rl', 'rr']


def corr(x, y):
    n = len(x)
    if n < 3:
        return 0.0
    mx, my = sum(x) / n, sum(y) / n
    sx = sum((v - mx) ** 2 for v in x) ** 0.5
    sy = sum((v - my) ** 2 for v in y) ** 0.5
    return sum((x[k] - mx) * (y[k] - my) for k in range(n)) / (sx * sy) if sx * sy else 0.0


def smooth(v, w):
    out, s = [], 0.0
    for k in range(len(v)):
        s += v[k]
        if k >= w:
            s -= v[k - w]
        out.append(s / min(k + 1, w))
    return out


def derive_signs(R, V):
    """Per-leg hip1 sign, DERIVED from the data rather than assumed.  A correct
    derivation returns a left/right mirror pattern; if it does not, the trace or
    the contact channel is suspect."""
    out = []
    for i in range(4):
        sw = [(R[k]['h1'][i] - R[k - 1]['h1'][i]) * (1.0 if R[k]['c'][i] else 0.0)
              for k in range(1, len(R))]
        out.append(1.0 if corr(sw, V) >= 0 else -1.0)
    return out


def propulsion(R, signs, mode='stance'):
    rows = []
    for k in range(1, len(R)):
        row = []
        for i in range(4):
            c = R[k]['c'][i]
            g = (1.0 if c else 0.0) if mode == 'stance' else \
                ((0.0 if c else 1.0) if mode == 'swing' else 1.0)
            row.append(signs[i] * (R[k]['h1'][i] - R[k - 1]['h1'][i]) * g)
        rows.append(row)
    return rows


def pulses(V, w):
    S = smooth(V, w) if w > 1 else V
    out, start = [], 0
    for k in range(1, len(S)):
        if (S[k - 1] <= 0) != (S[k] <= 0):
            if k - start >= 3:
                out.append((start, k))
            start = k
    return out


def shares(prop, pl, V, shuffle=False, seed=7):
    rows, imps = [], []
    for a, b in pl:
        e = [sum(prop[k][i] for k in range(a, b)) for i in range(4)]
        tot = sum(abs(x) for x in e)
        if tot > 0:
            rows.append([x / tot for x in e])
            imps.append(sum(V[a:b]))
    if shuffle:
        random.Random(seed).shuffle(imps)
    return rows, imps


def main():
    path = sys.argv[1]
    warm = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
    w = int(sys.argv[3]) if len(sys.argv) > 3 else 30
    R = [json.loads(l) for l in open(path)]
    R = [r for r in R if r['t'] > warm]
    V = [r['fwd_v'] for r in R[1:]]
    signs = derive_signs(R, V)
    pl = pulses(V, w)
    print("%d ticks after warmup %d | %d pulses @ smooth=%d" % (len(R), warm, len(pl), w))
    print("derived hip1 signs: %s%s" % (
        dict(zip(LEGS, [int(s) for s in signs])),
        "" if signs[0] * signs[1] < 0 and signs[2] * signs[3] < 0
        else "   <== ⚠ NOT a left/right mirror; distrust this trace"))

    base, imps = shares(propulsion(R, signs, 'stance'), pl, V)
    print("\n%-6s %-13s %s" % ("leg", "mean |share|", "corr(share, pulse impulse)"))
    for i, lg in enumerate(LEGS):
        print("%-6s %-13.3f %+.3f" % (lg, sum(abs(r[i]) for r in base) / max(1, len(base)),
                                      corr([r[i] for r in base], imps)))

    print("\ncontrols — the metric must beat all of these")
    for label, kw in (("stance (metric)", dict(mode='stance')),
                      ("swing (moves, cannot propel)", dict(mode='swing')),
                      ("ungated (pure movement)", dict(mode='all'))):
        rows, im = shares(propulsion(R, signs, kw['mode']), pl, V)
        cs = [corr([r[i] for r in rows], im) for i in range(4)]
        print("  %-30s mean|r|=%.3f   %s" % (label, sum(abs(c) for c in cs) / 4,
                                             " ".join("%+.3f" % c for c in cs)))
    rows, im = shares(propulsion(R, signs, 'stance'), pl, V, shuffle=True)
    cs = [corr([r[i] for r in rows], im) for i in range(4)]
    print("  %-30s mean|r|=%.3f   (noise floor)" % ("shuffled pulses (null)",
                                                    sum(abs(c) for c in cs) / 4))


if __name__ == '__main__':
    main()
