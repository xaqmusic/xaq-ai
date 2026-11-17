#!/usr/bin/env python3
"""stepper_after.py -- read the after-touch record (PoseSequencer at_learn > 0; stepper/AfterTouch.hpp; learned stepper plan 9.5).

usage: stepper_after.py <name>=<log.jsonl[.gz]>[,<log>...] [<name>=...]      (a bare path is its own group)

Per group of runs (an arm): how many landed toes bear in the stance that follows, how many are stepped again at once, both
against the gap -- the height the stance gives the toe on the tick of its touch, over the height it touched at -- and against
the body's tilt; the plan against the hand (how many planned landings end away from their plan, which rule moved them,
how often the plan's prediction that the next leg will be liftable fails and why); how well the gap separates them as it is known at the touch and as it was predictable at the go (the
executor's expected touch height); the shadow model's grade; and the run-level complements (net distance, landings, falls,
share of time re-stepping one leg) so a change in bearing is read with what it cost.
"""
import gzip, json, math, sys, collections

NAMES = ['FL', 'FR', 'RL', 'RR']
OUT = ['does not bear', 'same leg lifts next', 'tilt change (deg)', "wave's turn skipped"]
RULE = ['reach clamp', 'landing-sector clamp', 'edge move', 'obstacle reflex', 'lowering retreat', 'IK shortfall at the touch', 'clip']
HIP = [(-0.0383, 0.033), (0.0383, 0.033), (-0.0383, -0.033), (0.0383, -0.033)]   # hip1 mounts (HIP_X_SPAN 76.6 mm, HIP_Z_SPAN 66 mm)


def load(p):
    rows = []
    try:
        with (gzip.open(p, 'rt') if p.endswith('.gz') else open(p)) as f:
            for l in f:
                if l.startswith('{') and '"fwd_v"' in l:
                    try: rows.append(json.loads(l))
                    except Exception: pass
    except EOFError: pass
    return rows


def auc(pos, neg):
    """P(a positive's value > a negative's), ties halved."""
    if not pos or not neg: return float('nan')
    allv = sorted([(v, 1) for v in pos] + [(v, 0) for v in neg]); n = len(allv); ranks = [0.0] * n; i = 0
    while i < n:
        j = i
        while j + 1 < n and allv[j + 1][0] == allv[i][0]: j += 1
        for k in range(i, j + 1): ranks[k] = (i + j) / 2.0 + 1
        i = j + 1
    r = sum(ranks[k] for k in range(n) if allv[k][1] == 1)
    return (r - len(pos) * (len(pos) + 1) / 2.0) / (len(pos) * len(neg))


def read_run(p):
    rows = load(p)
    if len(rows) < 20: return None
    recs = {}; grade = None; counts = None; outs = {}
    for r in rows:
        o = r.get('ps_out')
        if o and o[0] > 0: outs[int(o[0])] = (o, math.degrees(abs(r['tilt'])))
        a = r.get('ps_aft')
        if a and a[0] > 0 and a[21] in (0, 1): recs[int(a[0])] = a
        if 'ps_aftg' in r: grade = r['ps_aftg']; counts = r['ps_aftn']
    eng = [r for r in rows if r.get('ps_eng', 0) >= 0.5]
    # re-stepping: one leg landing three or more times in a row (stepper_tilt.py's reader)
    seq = []
    for i in range(1, len(rows)):
        a, b = rows[i - 1].get('ps_steps'), rows[i].get('ps_steps')
        if not a or not b: continue
        for l in range(4):
            for _ in range(max(0, b[l] - a[l])): seq.append((i, l))
    tap = 0.0; j = 0; longest = 0.0
    while j < len(seq):
        k = j
        while k + 1 < len(seq) and seq[k + 1][1] == seq[j][1]: k += 1
        if k - j + 1 >= 3: d = (seq[k][0] - seq[j][0]) * 0.1; tap += d; longest = max(longest, d)
        j = k + 1
    net = math.hypot(eng[-1]['x'] - eng[0]['x'], eng[-1]['z'] - eng[0]['z']) if eng else 0.0
    tilt = sorted(math.degrees(abs(r['tilt'])) for r in eng)
    return dict(path=p, recs=list(recs.values()), grade=grade, counts=counts, eng_s=len(eng) * 0.1, net=net, tap_s=tap, longest=longest,
                land=rows[-1].get('ps_land', 0), miss=rows[-1].get('ps_miss', 0), falls=rows[-1].get('auto_reset_count', 0),
                tilt_med=tilt[len(tilt) // 2] if tilt else 0.0, over10=sum(1 for t in tilt if t >= 10) / max(1, len(tilt)),
                tilt_off=rows[-1].get('ps_tilt_off', 0), sunk=rows[-1].get('ps_sunk', 0), outs=list(outs.values()), hrej=rows[-1].get('ps_hrej', 0),
                pairs=[(outs[q][0], recs[q]) for q in recs if q in outs],
                drf=rows[-1].get('ps_drf', [0] * 4), drc=rows[-1].get('ps_drc', [0] * 4), drm=rows[-1].get('ps_drm', [0] * 3),
                rln=rows[-1].get('ps_rln', [0] * 7), rlt=rows[-1].get('ps_rlt', [0] * 7), rls=rows[-1].get('ps_rls', [0] * 7))


def table(recs, key, edges, unit):
    for a, b in zip(edges[:-1], edges[1:]):
        s = [r for r in recs if a <= key(r) < b]
        if not s: continue
        nb = [r for r in s if r[22] >= 0]; rs = [r for r in s if r[23] >= 0]
        print('    [%5g, %5g) %s: %4d landings | bearing read on %4d: does not bear %3.0f%% | a next step on %4d: same leg %3.0f%% | mean load share %.2f' % (
            a, b, unit, len(s), len(nb), 100.0 * sum(r[22] for r in nb) / max(1, len(nb)), len(rs), 100.0 * sum(r[23] for r in rs) / max(1, len(rs)),
            sum(r[16] for r in s) / len(s)))


def report(name, runs):
    recs = [a for r in runs for a in r['recs']]
    print('=' * 100); print('%s: %d runs, %.1f min engaged, %d after-touch records' % (name, len(runs), sum(r['eng_s'] for r in runs) / 60.0, len(recs)))
    for r in runs:
        print('  %-22s net %.2f m, landings %4d, missed %2d, falls %d, tilt-outs %d, median tilt %4.1f deg (>10 deg %2.0f%%), re-stepping %5.0f s (%2.0f%%), longest %4.0f s' % (
            r['path'].split('/')[-1], r['net'], r['land'], r['miss'], r['falls'], r['tilt_off'], r['tilt_med'], 100 * r['over10'], r['tap_s'], 100.0 * r['tap_s'] / max(1.0, r['eng_s']), r['longest']))
    n = len(runs); mean = lambda k: sum(r[k] for r in runs) / n; sd = lambda k: (sum((r[k] - mean(k)) ** 2 for r in runs) / max(1, n - 1)) ** 0.5
    print('  mean of runs: net %.2f +- %.2f m, landings %.0f +- %.0f, re-stepping %.0f +- %.0f s, falls %d in all' % (mean('net'), sd('net'), mean('land'), sd('land'), mean('tap_s'), sd('tap_s'), sum(r['falls'] for r in runs)))
    # the plan against the hand (the planner's drift instrument and the executor's relocation tag; always on)
    S = lambda k, i: sum(r[k][i] for r in runs)
    pl, off2 = S('rln', 1), S('rln', 4)
    if pl:
        print('  the plan against the hand: %d planned landings, %d (%.0f%%) end more than 2 cm from where the plan put them; the largest move among those: %s' % (
            pl, off2, 100.0 * off2 / pl, ', '.join('%s %d' % (RULE[i], S('rlt', i)) for i in sorted(range(7), key=lambda i: -S('rlt', i)) if S('rlt', i))))
        for a, b in [(0, 5), (5, 10), (10, 15), (15, 90)]:
            o = [x for r in runs for x, t in r['outs'] if a <= t < b and x[2] == 1]
            if not o: continue
            offs = [math.hypot(x[6] - x[4], x[7] - x[5]) for x in o]
            dr = [math.hypot(x[6] - HIP[int(x[1])][0], x[7] - HIP[int(x[1])][1]) - math.hypot(x[4] - HIP[int(x[1])][0], x[5] - HIP[int(x[1])][1]) for x in o]
            big = [i for i, v in enumerate(offs) if v > 0.02]
            print('    tilt %2d-%2d deg: %4d planned landings, off by > 2 cm %3.0f%%; among those the toe ends a median %+.0f mm from its hip against the plan (toward the hip %.0f%%)' % (
                a, b, len(o), 100.0 * len(big) / len(o), 1000 * sorted(dr[i] for i in big)[len(big) // 2] if big else 0.0, 100.0 * sum(1 for i in big if dr[i] < 0) / max(1, len(big))))
        pred, bad = S('drf', 1), S('drf', 2)
        if pred and bad:
            print('    the plan predicted the next leg liftable %d times and it was not on %d (%.0f%%); swapping one factor from predicted to actual, what breaks it: the landed toe alone %.0f%%, the other toes alone %.0f%%, the centre of mass alone %.0f%%, only together %.0f%%' % (
                pred, bad, 100.0 * bad / pred, *[100.0 * S('drc', i) / bad for i in range(4)]))
            print('    in those cases the landed toe is a mean %.0f mm from its planned place, the other toes %.0f mm, the centre of mass %.0f mm' % tuple(1000.0 * S('drm', i) / bad for i in range(3)))
    if not recs: print('  no after-touch record in these logs (at_learn 0?)'); return
    nb = [r for r in recs if r[22] >= 0]; rs = [r for r in recs if r[23] >= 0]
    print('  the toe just placed: bearing read on %d landings (planted long enough), does not bear on %d (%.0f%%); a next step followed %d, the same leg again on %d (%.0f%%); sunk (no touch) %d' % (
        len(nb), sum(r[22] for r in nb), 100.0 * sum(r[22] for r in nb) / max(1, len(nb)), len(rs), sum(r[23] for r in rs), 100.0 * sum(r[23] for r in rs) / max(1, len(rs)), sum(1 for r in recs if not r[2])))
    for l in range(4):
        s = [r for r in recs if r[1] == l]; a = [r for r in s if r[22] >= 0]; b = [r for r in s if r[23] >= 0]
        print('    %s: %4d landings, does not bear %3.0f%%, same leg next %3.0f%%, median gap %+5.1f mm' % (NAMES[l], len(s), 100.0 * sum(r[22] for r in a) / max(1, len(a)), 100.0 * sum(r[23] for r in b) / max(1, len(b)), 1000 * sorted(r[6] for r in s)[len(s) // 2] if s else 0.0))
    # what followed a landing, by the executor rule that moved it away from the plan (the outcome's moves, paired by seq)
    P = [x for r in runs for x in r['pairs'] if x[0][2] == 1]
    if P:
        mv = lambda o, k: math.hypot(o[12 + 2 * k], o[13 + 2 * k]); med = lambda v: sorted(v)[len(v) // 2] if v else float('nan')
        def rate(S, k): v = [a[k] for o, a in S if a[k] >= 0]; return 100.0 * sum(v) / len(v) if v else float('nan')
        print('  what followed a planned landing, by the rule that moved it from its plan (the memory\'s ground at the landing was refused by the band %d times):' % sum(r['hrej'] for r in runs))
        for label, sel in [('no rule moved it by > 1 cm', lambda o: max(mv(o, k) for k in range(7)) <= 0.01), ('lowering retreat > 1 cm', lambda o: mv(o, 4) > 0.01),
                           ('obstacle reflex > 1 cm', lambda o: mv(o, 3) > 0.01), ('edge move > 1 cm', lambda o: mv(o, 2) > 0.01)]:
            S = [(o, a) for o, a in P if sel(o)]
            if not S: continue
            print('    %-28s %4d landings: same leg next %3.0f%%, wave\'s turn skipped %3.0f%%, touch against the ground expected %+4.0f mm (median), tilt %2.0f deg (median), front legs %3.0f%%' % (
                label, len(S), rate(S, 23), rate(S, 25), 1000 * med([o[9] - o[8] for o, a in S]), med([a[10] for o, a in S]), 100.0 * sum(1 for o, a in S if o[1] < 2) / len(S)))
    print('  by the gap at the touch (the height the stance gives the toe over the height it touched at):')
    table(recs, lambda r: 1000 * r[6], [-1000, -10, 0, 5, 10, 15, 20, 30, 40, 1000], 'mm')
    print('  by the gap as predictable at the go (the stance height over the touch height the executor expected):')
    table([r for r in recs if r[30]], lambda r: 1000 * (r[5] - r[4]), [-1000, -10, 0, 5, 10, 15, 20, 30, 40, 1000], 'mm')
    print('  by the body\'s tilt at the touch:')
    table(recs, lambda r: r[10], [0, 5, 10, 15, 90], 'deg')
    print('  how well one quantity separates the landings (AUC; 0.5 = not at all):')
    for label, key, sel in [('the gap at the touch', lambda r: r[6], lambda r: True), ('the gap predictable at the go', lambda r: r[5] - r[4], lambda r: r[30]),
                            ('the touch below its prediction', lambda r: r[4] - r[3], lambda r: r[30]), ('the tilt at the touch', lambda r: r[10], lambda r: True),
                            ('the shadow model\'s prediction', None, lambda r: True)]:
        a0 = auc([(key(r) if key else r[26]) for r in nb if sel(r) and r[22] > 0.5], [(key(r) if key else r[26]) for r in nb if sel(r) and r[22] < 0.5])
        a1 = auc([(key(r) if key else r[27]) for r in rs if sel(r) and r[23] > 0.5], [(key(r) if key else r[27]) for r in rs if sel(r) and r[23] < 0.5])
        print('    %-32s does not bear %.2f | same leg next %.2f' % (label, a0, a1))
    # the shadow model, graded on these records (prequential: each prediction was made before its record was learned)
    print('  the shadow model (predictions made at the touch, before the record was learned), on these records:')
    for k in range(4):
        s = [r for r in recs if r[22 + k] >= 0 or k == 2]
        if k == 2: s = [r for r in recs if r[21] in (0, 1)]
        if len(s) < 5: continue
        m = sum(r[22 + k] for r in s) / len(s); mse = sum((r[22 + k] - r[26 + k]) ** 2 for r in s) / len(s); var = sum((r[22 + k] - m) ** 2 for r in s) / len(s)
        print('    %-22s n %4d, mean %+.2f, model residual rms %.2f against the spread %.2f (variance explained %+.0f%%)' % (OUT[k], len(s), m, mse ** 0.5, var ** 0.5, 100.0 * (1 - mse / var) if var > 0 else 0.0))
    g = [r['grade'] for r in runs if r['grade']]
    if g:
        print('  in-brain grade at each run\'s end, per outcome [n, mean, model residual ms, base-rate residual ms, NIS], mean over runs:')
        for k in range(4):
            v = [sum(x[5 * k + i] for x in g) / len(g) for i in range(5)]
            print('    %-22s n %5.0f, mean %+.2f, model %.3f, base rate %.3f (skill %+.0f%%), NIS %.2f' % (OUT[k], v[0], v[1], v[2], v[3], 100.0 * (1 - v[2] / v[3]) if v[3] > 0 else 0.0, v[4]))


if __name__ == '__main__':
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    for arg in sys.argv[1:]:
        name, paths = arg.split('=', 1) if '=' in arg and not arg.startswith('/') else (arg.split('/')[-1], arg)
        runs = [r for r in (read_run(p) for p in paths.split(',')) if r]
        if runs: report(name, runs)
