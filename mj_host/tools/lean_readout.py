#!/usr/bin/env python3
"""The head's lean on the walk: does it follow ACCELERATION (a transient, then home at a steady pace) or SPEED (held
as long as the pace is)?  The operator's picture (2026-09-30): lean into an acceleration as an inverted pendulum
does, and settle to the stable head once the pace is constant; forward and backward alike.

Read from the host's JSONL (walking ticks: drive "walk", no stop phase):
  pitch   the head's attitude pitch, MINUS the head IMU's gravity z (-hg[2]): 0 = level, + = nose DOWN (checked
          2026-10-01: the stops' downward gaze at the floor's things reads hg[2] = -0.33 on the still-head walker)
  v       forward speed, m/s (sensed[0] x 0.4), a 0.5 s box average (the stride, 2.2 Hz, averaged out)
  a       forward acceleration, m/s^2, the difference of v across 0.5 s
Reported per glob:
  - the regression pitch ~ v + a (+ const), with each coefficient's share of the explained variance;
  - the mean pitch in phases: CRUISE (|a| < a_cruise for >= 1 s) forward / slow / backward, ACCELERATING,
    DECELERATING (|a| > a_move), BACKING (v < -0.03 m/s, any a), and the stand's (stop phase: the gaze's own);
  - the settle: after a forward acceleration ends with the pace kept 2 s, the mean pitch 0 / 0.5 / 1 / 1.5 / 2 s on.
The target signature: pitch rides on a, not on v; cruise pitch = the level; the settle within ~1 s.
Usage: lean_readout.py "glob" ["glob" ...]  [--from S]  (S = skip the first S s of each run, e.g. a babble)
"""
import glob, json, math, sys

DT = 0.02
BOX = 25            # 0.5 s
A_CRUISE, A_MOVE = 0.06, 0.12   # m/s^2
V_FWD, V_SLOW = 0.08, 0.03       # m/s


def runs(pattern, t_from):
    for path in sorted(glob.glob(pattern)):
        seg = []
        for line in open(path):
            if not line.startswith('{'):
                continue
            r = json.loads(line)
            if r['t'] < t_from or 'hg' not in r:
                continue
            walk = r['drive'] == 'walk' and r.get('stop', 0) == 0
            if not walk:
                if seg:
                    yield seg, None
                    seg = []
                if r['drive'] == 'stand':
                    yield None, -r['hg'][2]
                continue
            seg.append((r['sensed'][0] * 0.4, -r['hg'][2], r['q'][5], r['q'][6]))
        if seg:
            yield seg, None


def lstsq3(rows):
    # pitch = b0 + bv v + ba a, normal equations (3x3)
    n = len(rows)
    S = [[0.0] * 3 for _ in range(3)]; y = [0.0] * 3
    for v, a, p in rows:
        x = (1.0, v, a)
        for i in range(3):
            y[i] += x[i] * p
            for j in range(3):
                S[i][j] += x[i] * x[j]
    # solve by Gaussian elimination
    M = [S[i] + [y[i]] for i in range(3)]
    for c in range(3):
        piv = max(range(c, 3), key=lambda r: abs(M[r][c]))
        M[c], M[piv] = M[piv], M[c]
        for r in range(3):
            if r != c:
                f = M[r][c] / M[c][c]
                for k in range(c, 4):
                    M[r][k] -= f * M[c][k]
    return [M[i][3] / M[i][i] for i in range(3)]


def mean(xs):
    return sum(xs) / len(xs) if xs else float('nan')


def sd(xs):
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs)) if len(xs) > 1 else float('nan')


def analyse(pattern, t_from):
    rows = []; phases = {k: [] for k in ('cruise fwd', 'cruise slow', 'cruise back', 'accel', 'decel', 'backing', 'stand')}
    settle = {k: [] for k in (0.0, 0.5, 1.0, 1.5, 2.0)}
    cq = []; nruns = len(glob.glob(pattern))
    for seg, stand in runs(pattern, t_from):
        if seg is None:
            phases['stand'].append(stand); continue
        n = len(seg)
        if n < 3 * BOX:
            continue
        vs = [s[0] for s in seg]
        # box average of v, then a = dv over the box
        cs = [0.0]
        for x in vs:
            cs.append(cs[-1] + x)
        vb = [None] * n
        for i in range(BOX - 1, n):
            vb[i] = (cs[i + 1] - cs[i + 1 - BOX]) / BOX
        ab = [None] * n
        for i in range(2 * BOX - 1, n):
            ab[i] = (vb[i] - vb[i - BOX]) / (BOX * DT)
        # the pitch box-averaged too (the stride's nod out; the lean is what stays), aligned with v
        ps = [s[1] for s in seg]
        cp = [0.0]
        for x in ps:
            cp.append(cp[-1] + x)
        pb = [None] * n
        for i in range(BOX - 1, n):
            pb[i] = (cp[i + 1] - cp[i + 1 - BOX]) / BOX
        # cruise: |a| < A_CRUISE for the last second
        calm = 0
        for i in range(2 * BOX - 1, n):
            v, a, p = vb[i], ab[i], pb[i]
            rows.append((v, a, p)); cq.append((seg[i][1], seg[i][2], seg[i][3]))
            calm = calm + 1 if abs(a) < A_CRUISE else 0
            if calm >= 50:
                key = 'cruise fwd' if v > V_FWD else ('cruise back' if v < -V_SLOW else ('cruise slow' if abs(v) < V_SLOW else None))
                if key:
                    phases[key].append(p)
            if v < -V_SLOW:
                phases['backing'].append(p)
            if a > A_MOVE:
                phases['accel'].append(p)
            elif a < -A_MOVE:
                phases['decel'].append(p)
        # the settle: a forward acceleration (a > A_MOVE for >= 0.2 s) ends (a < A_CRUISE) with v > V_FWD kept 2 s
        i = 2 * BOX - 1; run_len = 0
        while i < n:
            if ab[i] > A_MOVE:
                run_len += 1
            else:
                if run_len >= 10:
                    # the acceleration has ended: the first tick within 0.5 s at a cruise's acceleration
                    j0 = next((j for j in range(i, min(n, i + 25)) if abs(ab[j]) < A_CRUISE), None)
                    end = (j0 + int(2.0 / DT)) if j0 is not None else n
                    if j0 is not None and end < n and all(vb[j] > V_FWD for j in range(j0, end + 1)):
                        for k in settle:
                            settle[k].append(pb[j0 + int(k / DT)])
                        i = end; run_len = 0; continue
                run_len = 0
            i += 1
    if not rows:
        print(f"{pattern}: no walking ticks"); return
    b0, bv, ba = lstsq3(rows)
    vv = [r[0] for r in rows]; aa = [r[1] for r in rows]; pp = [r[2] for r in rows]
    # sign check: pitch against the two pitch joints (raw ticks)
    def corr(x, y):
        mx, my = mean(x), mean(y); sx, sy = sd(x), sd(y)
        return sum((a - mx) * (b - my) for a, b in zip(x, y)) / (len(x) * sx * sy)
    c_neck = corr([c[0] for c in cq], [c[1] for c in cq]); c_head = corr([c[0] for c in cq], [c[2] for c in cq])
    print(f"{pattern}  ({nruns} runs, {len(rows) * DT / max(1, nruns):.0f} s of walk a run; pitch ~ neck q r={c_neck:+.2f}, head q r={c_head:+.2f})")
    print(f"  pitch = {b0:+.3f} {bv:+.3f}*v {ba:+.3f}*a    (per 0.1 m/s: {0.1 * bv:+.3f} rad; per 0.2 m/s^2: {0.2 * ba:+.3f} rad;"
          f" sd v {sd(vv):.3f} m/s, sd a {sd(aa):.3f} m/s^2, sd pitch {sd(pp):.3f})")
    print(f"  share of the pitch's variance: v {bv * bv * sd(vv) ** 2 / sd(pp) ** 2 * 100:.0f} %, a {ba * ba * sd(aa) ** 2 / sd(pp) ** 2 * 100:.0f} %")
    st = phases['stand']
    print("  mean pitch (rad, + = down):  " + "  ".join(f"{k} {mean(v):+.3f} ({len(v) * DT / max(1, nruns):.0f} s)" for k, v in phases.items()))
    if settle[0.0]:
        print(f"  settle after {len(settle[0.0])} forward accelerations: " + "  ".join(f"{k:.1f}s {mean(v):+.3f}" for k, v in settle.items()))


if __name__ == '__main__':
    args = sys.argv[1:]; t_from = 0.0
    if '--from' in args:
        k = args.index('--from'); t_from = float(args[k + 1]); del args[k:k + 2]
    for g in args:
        analyse(g, t_from)
