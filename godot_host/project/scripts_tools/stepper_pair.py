#!/usr/bin/env python3
"""stepper_pair.py — an arm against its control, PAIRED BY SEED, on two chains of the same schedule (learned stepper plan Phase 4's
gate reader, 2026-11-16). stepper_practice.py reads one chain's curve over practice; this reads the difference between two chains
that ran the same worlds from the same seeds (stepper_chain.sh: run k of chain c has world seed 10 c + k in both).

    stepper_pair.py <arm chain dir> <control chain dir>      [JOBS=8]

Per world, and per repetition of it, for every metric of stepper_practice.py plus
  cm/step   net forward progress per landing (prog / step rate): what a layer that may only shorten a step is priced in
the arm's mean, the control's mean, and the paired difference (arm - control): mean ± sd over CHAINS of each chain's mean
difference over its runs, with t over chains (n = chains; a chain is the unit, its runs share a carried model). Then the same
difference per repetition, so a cost that grows or fades with practice is seen. The landing offset's own columns (lo_*) are the
arm's alone when the control carries none.

Read together (blind-metric complements, as in stepper_practice.py): progress with stall and falls; touchdown error with progress
(a body that stopped trying has no touchdown error); step rate with cm/step (shorter steps taken faster can tie on progress)."""
import glob, math, os, re, statistics as st, sys
from multiprocessing import Pool
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stepper_practice import read, COLS
from stepper_learning import broken_after

PCOLS = COLS[:7] + [('cmstep', 'cm/step', '{:.2f}')] + COLS[7:]


def load(d, jobs):
    sched = re.search(r'schedule \[([^\]]*)\]', open(f'{d}/manifest.txt').read()).group(1).split()
    chains = sorted({int(re.search(r'_c(\d+)_r', p).group(1)) for p in glob.glob(f'{d}/state_c*_r*.json')})
    cut = broken_after(d); todo = []
    for c in chains:
        for k, w in enumerate(sched, 1):
            if k > cut.get(c, len(sched)): break
            fs = glob.glob(f'{d}/r{k}/*_c{c}.jsonl.gz')
            if fs: todo.append((c, k, w, fs[0], int(w[1:]) / 1000.0 if re.fullmatch(r'b\d+', w) else 0.0))
    with Pool(jobs) as pool: res = pool.map(read, [(t[3], t[4]) for t in todo])
    out = {}
    for (c, k, w, _, _), m in zip(todo, res):
        if m is None: continue
        m['cmstep'] = m['prog'] / m['spm'] if m.get('spm') else None
        out[(c, k)] = (w, m)
    return sched, out


def mean(xs):
    xs = [x for x in xs if x is not None]
    return st.mean(xs) if xs else None


def fmt(x, f='{:.2f}'):
    return '-' if x is None else f.format(x)


def paired(ds):
    """ds: {chain: [differences over its runs]} -> (mean, sd, t, n) over chains of each chain's mean difference."""
    per = [st.mean(v) for v in ds.values() if v]
    if not per: return None
    m = st.mean(per)
    if len(per) < 2: return m, None, None, len(per)
    sd = st.stdev(per)
    return m, sd, (m / (sd / math.sqrt(len(per))) if sd > 0 else None), len(per)


def line(head, a, b, p, f):
    s = f'     {head:12s} arm {fmt(a, f):>8s}  control {fmt(b, f):>8s}'
    if p is not None:
        m, sd, t, n = p
        s += f'   arm - control {m:+.3f}' + ('' if sd is None else f' ± {sd:.3f}') + ('' if t is None else f'  (t {t:+.1f}, n {n})')
    return s


def main(argv):
    jobs = 8; dirs = []
    for a in argv:
        if a.startswith('JOBS='): jobs = int(a[5:])
        else: dirs.append(a.rstrip('/'))
    if len(dirs) != 2: sys.exit(__doc__)
    sa, A = load(dirs[0], jobs); sb, B = load(dirs[1], jobs)
    if sa != sb: sys.exit(f'schedules differ: [{" ".join(sa)}] against [{" ".join(sb)}]')
    keys = sorted(set(A) & set(B)); lone = sorted(set(A) ^ set(B))
    print(f'== {os.path.basename(dirs[0])} (arm) - {os.path.basename(dirs[1])} (control); schedule {" ".join(sa)}; {len(keys)} paired runs'
          + (f'; UNPAIRED (left out): {lone}' if lone else ''))
    for w in dict.fromkeys(sa):
        ks = [k for k in keys if A[k][0] == w]
        if not ks: continue
        reps = {}                                   # the repetition index of run k within this world
        for i, k in enumerate([k for k, x in enumerate(sa, 1) if x == w], 1): reps[k] = i
        print(f'  -- {w}  ({len(ks)} paired runs, {len({c for c, _ in ks})} chains)')
        for key, head, f in PCOLS:
            a = [A[k][1].get(key) for k in ks]; b = [B[k][1].get(key) for k in ks]
            if all(x is None for x in a) and all(x is None for x in b): continue
            ds = {}
            for k in ks:
                x, y = A[k][1].get(key), B[k][1].get(key)
                if x is not None and y is not None: ds.setdefault(k[0], []).append(x - y)
            print(line(head, mean(a), mean(b), paired(ds), f))
        nrep = max(reps.values())
        if nrep > 1:
            print('     arm - control by repetition (mean over chains ± sd):')
            print('       ' + f'{"":12s}' + ' '.join(f'{"rep " + str(i):>16s}' for i in range(1, nrep + 1)))
            for key, head, f in PCOLS:
                cells = []
                for i in range(1, nrep + 1):
                    d = [A[k][1][key] - B[k][1][key] for k in ks if reps[k[1]] == i and A[k][1].get(key) is not None and B[k][1].get(key) is not None]
                    cells.append('-' if not d else f'{st.mean(d):+.2f}' + ('' if len(d) < 2 else f'±{st.stdev(d):.2f}'))
                if any(c != '-' for c in cells): print('       ' + f'{head:12s}' + ' '.join(f'{c:>16s}' for c in cells))


if __name__ == '__main__':
    main(sys.argv[1:])
