#!/usr/bin/env python3
"""stepper_belief.py — the stance belief's gate (learned stepper plan Phase 3), read from a battery's logs (stepper_battery.sh on a config
carrying StanceFeatures -> epm_stance -> belief_stance).

    stepper_belief.py <dir> [bench prefix ...]          default prefixes: L42 b38 b64 cor

Per bench, over the engaged rows that carry a fresh stance sample (sf_pub advanced):
  vocabulary   the stance EPM's nodes and baked nodes at the end and at 50 % / 75 % of the run (bounded = flat at the end), its TLE;
  separation   for each situation the instruments found decisive -- labels derived from the instruments for EVALUATION ONLY, the EPM never
               sees them:
                 room    a rear leg short of room (its knee within 0.15 of straight, or under 1.5 cm of extension slack; ps_room)
                 inner   a front toe at its inner reach (the commanded toe within 2 mm of the planner's reach_min_m, 6 cm, of its hip; ps_tgts)
                 noseup  the body pitched nose-up beyond the front drop's deadband, 0.05 rad (tilt_p)
                 hang    a hung toe (the 25 ticks up to each hung-toe reflex firing; ps_hang)
               the NMI (sqrt form, as tof_studies.py) of the EPM winner and of the belief's MAP node against the label, a node-level F1
               (a node predicts the label when the label holds on most of its rows) and the label's lift in those nodes -- each against
               a chance level from the label circularly shifted (10 shifts of at least 10 % of the run: same rate, same autocorrelation);
  consistency  the belief's NIS (the run mean, sb_nism) and its switches against the EPM winner's (sb_sw); with a second, legacy-head
               BeliefFilter in the config (belief_stance_legacy), the decode heads' own NIS for both (sb_hnis / sbl_hnis) on the same stream."""
import collections, glob, json, math, os, random, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _logio import open_log

# hip1 per leg (x, z), body frame: recovered from the logged neutral places at the first shift after the settle (2026-11-15, p2d corridor:
# (+-0.109, +0.102) front, (+-0.109, -0.106) rear) less the 10 cm neutral radius along 45-degree headings
HIPS = [(-0.0383, 0.0313), (0.0383, 0.0313), (-0.0383, -0.0353), (0.0383, -0.0353)]


def nmi(a, b):
    """normalised mutual information, sqrt form (as mj_host/tools/tof_studies.py), pure Python"""
    n = len(a)
    if n == 0: return 0.0
    ca = collections.Counter(a); cb = collections.Counter(b)
    if len(ca) < 2 or len(cb) < 2: return 0.0
    cab = collections.Counter(zip(a, b))
    I = sum(c / n * math.log(c * n / (ca[x] * cb[y])) for (x, y), c in cab.items())
    Ha = -sum(c / n * math.log(c / n) for c in ca.values()); Hb = -sum(c / n * math.log(c / n) for c in cb.values())
    return I / math.sqrt(Ha * Hb) if Ha > 0 and Hb > 0 else 0.0


def node_f1(nodes, y):
    """a node predicts the label when the label holds on most of its rows: (F1, the label's lift in those nodes)"""
    if sum(y) == 0: return 0.0, 0.0
    tot = collections.Counter(nodes); pos = collections.Counter(k for k, v in zip(nodes, y) if v)
    lab = {k for k in tot if pos[k] / tot[k] > 0.5}
    npred = sum(tot[k] for k in lab); tp = sum(pos[k] for k in lab)
    f1 = 2 * tp / max(1, npred + sum(y))
    lift = (tp / npred) / (sum(y) / len(y)) if npred else 0.0
    return f1, lift


def labels(rows):
    L = {k: [] for k in ('room', 'inner', 'noseup', 'hang')}
    hang_at = set(); prev = None
    for i, r in enumerate(rows):
        h = (r.get('ps_hang') or [0])[0]
        if prev is not None and h > prev: hang_at.update(range(max(0, i - 4), i + 1))
        prev = h
    for i, r in enumerate(rows):
        rm = r.get('ps_room') or [[1, 1, 1, 1, 1]] * 4
        L['room'].append(int(any(rm[l][1] < 0.15 or rm[l][0] < 0.015 for l in (2, 3))))
        tg = r.get('ps_tgts') or []
        inner = 0
        if len(tg) >= 4 and isinstance(tg[0], list):
            for l in (0, 1):
                hx, hz = HIPS[l]
                if math.hypot(tg[l][0] - hx, tg[l][2] - hz) < 0.062: inner = 1
        L['inner'].append(inner)
        L['noseup'].append(int((r.get('tilt_p') or 0.0) > 0.05))
        L['hang'].append(int(i in hang_at))
    return L


def chance(nodes, y, fn, rng, k=10):
    n = len(y); out = []
    for _ in range(k):
        s = rng.randint(max(1, n // 10), max(2, n - n // 10)); out.append(fn(nodes, y[s:] + y[:s]))
    return out


def read(f):
    rows = []; last_pub = -1
    for l in open_log(f):
        if not l.startswith('{') or '"fwd_v"' not in l: continue
        try: r = json.loads(l)
        except Exception: continue
        if r.get('ps_eng', 0) < 1 or r.get('sb_win', -1) < 0: continue
        p = r.get('sf_pub', 0)
        if p <= last_pub: continue
        last_pub = p; rows.append(r)
    return rows


def vocab(f):
    t = []
    for l in open_log(f):
        if l.startswith('{') and '"sb_n"' in l:
            try: r = json.loads(l); t.append((r.get('sb_n', -1), r.get('sb_baked', -1), r.get('sb_tle', 0.0)))
            except Exception: pass
    return t


def main(argv):
    d = argv[0].rstrip('/'); benches = argv[1:] or ['L42', 'b38', 'b64', 'cor']; rng = random.Random(1)
    for b in benches:
        fs = sorted(glob.glob(f'{d}/*_{b}*.jsonl.gz') + glob.glob(f'{d}/*_{b}*.jsonl'))
        if not fs: continue
        res = {k: {'win': [], 'map': [], 'cw': [], 'cm': [], 'f1': [], 'cf1': [], 'lift': [], 'rate': []} for k in ('room', 'inner', 'noseup', 'hang')}
        voc = []; nis = []; sw = []; nmed = []; heads = []
        for f in fs:
            rows = read(f)
            if len(rows) < 50: continue
            win = [r['sb_win'] for r in rows]; mp = [r.get('sb_map', -1) for r in rows]
            L = labels(rows)
            for k, y in L.items():
                if sum(y) < 3: continue
                R = res[k]; R['rate'].append(sum(y) / len(y))
                R['win'].append(nmi(win, y)); R['map'].append(nmi(mp, y))
                R['cw'] += chance(win, y, nmi, rng); R['cm'] += chance(mp, y, nmi, rng)
                f1, lift = node_f1(win, y); R['f1'].append(f1); R['lift'].append(lift)
                R['cf1'] += [c[0] for c in chance(win, y, node_f1, rng)]
            v = vocab(f)
            if v: voc.append((v[len(v) // 2][0], v[3 * len(v) // 4][0], v[-1][0], v[-1][1], v[-1][2]))
            last = rows[-1]; nis.append(last.get('sb_nism', 0.0)); sw.append(tuple(last.get('sb_sw', [0, 0])))
            nmed.append(st.median(r.get('sb_nis', 0.0) for r in rows))
            if 'sb_hnis' in last: heads.append((last['sb_hnis'], last.get('sbl_hnis'), last.get('sbl_nism'), tuple(last.get('sbl_sw', [0, 0]))))
        print(f'== {b} ({len(fs)} runs)')
        if voc: print('    vocabulary nodes at 50 / 75 / 100 %: ' + '  '.join(f'{a}/{c}/{e} (baked {g}, tle {h:.3f})' for a, c, e, g, h in voc))
        for k, R in res.items():
            if not R['win']: print(f'    {k:7s} too rare in these runs'); continue
            m = lambda xs: f'{st.mean(xs):.3f}' + (f'±{st.stdev(xs):.3f}' if len(xs) > 1 else '')
            print(f'    {k:7s} rate {m(R["rate"])}  NMI winner {m(R["win"])} (chance {m(R["cw"])})  MAP {m(R["map"])} (chance {m(R["cm"])})'
                  f'  node F1 {m(R["f1"])} (chance {m(R["cf1"])}) lift {m(R["lift"])}  n={len(R["win"])}')
        if nis: print(f'    belief NIS mean {st.mean(nis):.2f} (runs {", ".join(f"{x:.2f}" for x in nis)}), median {st.mean(nmed):.2f};  switches MAP / winner: '
                      + ', '.join(f'{a}/{w}' for a, w in sw))
        if heads: print('    decode-head NIS own / legacy: ' + ', '.join(f'{h:.2f}/{l:.2f}' for h, l, _, _ in heads if l is not None)
                        + ';  legacy predictive NIS ' + ', '.join(f'{n:.2f}' for _, _, n, _ in heads if n is not None)
                        + ';  legacy MAP switches ' + ', '.join(f'{w[0]}' for _, _, _, w in heads))


if __name__ == '__main__':
    main(sys.argv[1:])
