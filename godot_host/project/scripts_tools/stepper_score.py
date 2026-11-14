#!/usr/bin/env python3
"""stepper_score.py — the full metric set of the learned stepper plan's fixed battery (stepper_battery.sh), per run and per arm x bench.

    stepper_score.py <dir> [<dir> ...] [--runs]     (--runs also prints every run; arms are the file-name prefixes)

Distance alone is a blind metric here (a body that drifts or circles scores on it), so it is always read with its complements:
  z        final forward position (m)                       lvl     highest stairs level reached (m; 0 on the corridor)
  net/strt net displacement and straightness (net / path, the path sampled every 300 ticks)   steps   landings (all legs); rear = the two rear legs
  sps      seconds per step while engaged                   miss    missed landings; tmo phase timeouts
  frz      frozen windows (60 rows = 300 ticks with no body motion and no landing while engaged)
  hang     hung-toe reflex firings                          falls   auto resets; tilt the max |tilt| while engaged
  near/fnear  % of engaged rows a rear / front knee is within 0.15 of straight (ps_u[l][2] < 0.15); past = rows past straight
  lag4     % of rear rows near straight when the leg has waited 4+ other steps;  lag4share = how often it waits that long
  liftnear % of rear lifts that start from a near-straight knee;  gap4 = % of rear stance periods with 4+ other steps between
  skm/skl  skipped turns for no margin / no landing (all legs; _r rear);  strand = % of chosen landings that left the next leg
           unliftable (ps_nxl);  drift = % of predicted-liftable next legs found unliftable (ps_drf), land = of those, broken by
           the landed toe alone (ps_drc[0]);  pnone = decisions where the pose search found nothing;  ik = mean IK residual
Benches are grouped L42 (n = 1, the launcher layout), b38, b64, cor. Moved from a session scratchpad (ab_score.py) 2026-11-14."""
import collections, glob, json, math, os, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _logio import open_log


def rows(f):
    o = []
    for l in open_log(f):
        if l.startswith('{') and '"fwd_v"' in l:
            try: o.append(json.loads(l))
            except Exception: pass
    return o


def frozen(rs, win=60):
    n = 0
    for i in range(0, len(rs) - win, 10):
        a, b = rs[i], rs[i + win]
        if a.get('ps_eng', 0) >= 1 and abs(b['z'] - a['z']) < 0.01 and sum(b['ps_steps']) == sum(a['ps_steps']): n += 1
    return n


def step_events(eng):
    """(row index, leg) for every landing, from the per-leg step counters."""
    ev = []
    for k in range(1, len(eng)):
        a, b = eng[k - 1]['ps_steps'], eng[k]['ps_steps']
        for i in range(4):
            if b[i] > a[i]: ev.append((k, i))
    return ev


def score(f):
    rs = rows(f); r = rs[-1]; eng = [x for x in rs if x.get('ps_eng', 0) >= 1 and 'ps_u' in x] or rs
    steps = sum(r['ps_steps'])
    pr = rs[::60] + [r]   # path sampled every 300 ticks (~2.5 steps): at 5 ticks it counts the body's rock at every shift, not circling
    path = sum(math.hypot(b['x'] - a['x'], b['z'] - a['z']) for a, b in zip(pr, pr[1:]))
    net = math.hypot(r['x'] - rs[0]['x'], r['z'] - rs[0]['z'])
    U = lambda x, l: x['ps_u'][l][2] if 'ps_u' in x else 1.0
    near = sum(1 for x in eng for l in (2, 3) if U(x, l) < 0.15) / (2 * len(eng))
    fnear = sum(1 for x in eng for l in (0, 1) if U(x, l) < 0.15) / (2 * len(eng))
    past = sum(1 for x in eng for l in (2, 3) if U(x, l) < 0.0)
    ev = step_events(eng); evk = collections.defaultdict(list)
    for k, i in ev: evk[k].append(i)
    others = [0] * 4; lag = collections.Counter(); lagn = collections.Counter(); per = per4 = 0
    for k, x in enumerate(eng):
        for i in evk.get(k, ()):
            if i in (2, 3): per += 1; per4 += others[i] >= 4
            for j in range(4):
                if j != i: others[j] += 1
            others[i] = 0
        for l in (2, 3):
            b = min(others[l], 4); lagn[b] += 1; lag[b] += U(x, l) < 0.15
    lifts = [a['ps_u'][b['ps_leg']][2] for a, b in zip(eng, eng[1:])
             if b.get('ps_leg') in (2, 3) and b.get('ps_phase') == 2 and not (a.get('ps_phase') == 2 and a.get('ps_leg') == b.get('ps_leg')) and 'ps_u' in a]
    z4 = lambda k: r.get(k) or [0] * 4
    skm, skl, nxl, drf, drc = z4('ps_skm'), z4('ps_skl'), r.get('ps_nxl') or [0, 0], z4('ps_drf'), z4('ps_drc')
    return dict(z=r['z'], lvl=max(x.get('stairs_level', 0) or 0 for x in rs), net=net, strt=net / max(1e-9, path), steps=steps,
                rear=r['ps_steps'][2] + r['ps_steps'][3], sps=len(eng) * 5 / 50 / max(1, steps), miss=r.get('ps_miss', 0), tmo=r.get('ps_tmo', 0),
                frz=frozen(rs), hang=sum(r['ps_hang']) if isinstance(r.get('ps_hang'), list) else 0, falls=r['auto_reset_count'],
                tilt=max(abs(x.get('tilt', 0) or 0) for x in eng), near=100 * near, fnear=100 * fnear, past=past,
                lag4=100 * lag[4] / max(1, lagn[4]), lag4share=100 * lagn[4] / max(1, sum(lagn.values())),
                liftnear=100 * sum(1 for k in lifts if k < 0.15) / max(1, len(lifts)), gap4=100 * per4 / max(1, per),
                skm=sum(skm), skm_r=skm[2] + skm[3], skl=sum(skl), skl_r=skl[2] + skl[3], strand=100 * nxl[1] / max(1, sum(nxl)),
                drift=100 * drf[2] / max(1, drf[1]), land=100 * drc[0] / max(1, drf[2]), pnone=(r.get('ps_plan') or [0, 0, 0])[2],
                ik=st.mean(x.get('ps_ik', 0) or 0 for x in eng))


K = ['z', 'lvl', 'net', 'strt', 'steps', 'rear', 'sps', 'miss', 'tmo', 'frz', 'hang', 'falls', 'tilt', 'near', 'fnear', 'past',
     'lag4', 'lag4share', 'liftnear', 'gap4', 'skm', 'skm_r', 'skl', 'skl_r', 'strand', 'drift', 'land', 'pnone', 'ik']
BENCHES = ('L42', 'b38', 'b64', 'cor')


def fmt(v): return f'{v:8.3f}' if isinstance(v, float) else f'{v:8d}'


def main(argv):
    show = '--runs' in argv; dirs = [a for a in argv if not a.startswith('--')]; res = {}
    for d in dirs:
        for f in sorted(glob.glob(d.rstrip('/') + '/*.jsonl') + glob.glob(d.rstrip('/') + '/*.jsonl.gz')):
            n = os.path.basename(f).split('.jsonl')[0]
            if '_' not in n: continue
            arm, bench = n.split('_', 1); res[(arm, bench)] = score(f)
    if show:
        print(f"{'run':14s} " + ' '.join(f'{k:>8s}' for k in K))
        for (arm, bench), s in sorted(res.items(), key=lambda kv: (kv[0][1], kv[0][0])):
            print(f"{arm + '_' + bench:14s} " + ' '.join(fmt(s[k]) for k in K))
        print()
    arms = sorted({a for a, _ in res})
    for b in BENCHES:
        for arm in arms:
            xs = [s for (a, bb), s in res.items() if a == arm and (bb == b or bb.startswith(b + '_'))]
            if not xs: continue
            sd = lambda k: st.stdev(x[k] for x in xs) if len(xs) > 1 else 0.0
            print(f"{b} {arm} n{len(xs)}  " + '  '.join(f"{k} {st.mean(x[k] for x in xs):.3f}±{sd(k):.3f}" for k in K if k != 'ik'))
        print()


if __name__ == '__main__':
    main(sys.argv[1:])
