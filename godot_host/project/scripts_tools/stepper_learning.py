#!/usr/bin/env python3
"""stepper_learning.py — the LEARNING RATE of the stance vocabulary over a scheduled chain (stepper_chain.sh with SCHEDULE), not
only where it ends (operator 2026-11-15: "some metric for measuring improvement over time, not just final performance. If we can
see a rise in performance, then we may know longer runs or more practice in easier scenarios is in order").

    stepper_learning.py <chain dir> [<chain dir> ...]

The x-axis is PRACTICE: the minutes of engaged stance in one world class (cor, arena, b38, b64, ...) before the point read. Every
repetition of a world is one point on that world's practice curve.

Per world, per repetition (mean ± sd over the chains):
  min      practice in this world at the repetition's midpoint
  TLE      the stance EPM's error (the dual TLE), and its two parts when logged: QE (does the map fit this stance?) and TS
           (did the map predict where the stance went next?)
  ins/1k   node insertions per 1000 rows (positive steps in the node count) -- novelty still arriving
  +baked   nodes baked during the repetition
  reach    the body's furthest forward position (m; god's-eye, analysis only). The body is driven by the frozen PoseSequencer
           scaffold, so this should NOT rise with practice: it is the check that a falling error is learning, not a changed walk.

Then the rates, each fitted PER CHAIN and reported as mean ± sd and t over the chains:
  slope    the metric's change per DOUBLING of practice (fit against log2 minutes), absolute and as % of the first repetition.
           Negative and significant = still learning: more practice in this world should still pay.
  matched  the same slope with the rows binned by where the body is (forward position z, 5 cm bins; a god's-eye coordinate, used
           by this ANALYSIS only, never by the brain) and only bins visited in every repetition compared, each weighted equally.
           A run that spends longer stuck at a riser has a higher share of hard stances; the matched slope removes that terrain
           mix and leaves the learning. (Cutting runs into equal time slices would not: equal slices average back to the run.)
  last     the last repetition minus the one before (paired over chains): is it still falling at the end, or flat?
  return   for a world met again after a gap (more than one other run since its last repetition; the corridor after the stairs) (the corridor after the stairs): the first repetition after the gap
           minus the last one before it. Positive = interference (forgot); ~0 = retained. Only meaningful when the encoder held
           still across the gap (StanceFeatures.freeze_after) -- otherwise the units changed.
"""
import glob, gzip, json, math, os, re, statistics as st, sys

ROW_S = 5 / 60.0   # one diag row = 5 ticks at 60 Hz
BIN_M, BIN_MIN = 0.05, 20   # the matched slope's position bins (forward z, m) and the rows a bin needs in a repetition


def broken_after(d):
    """per chain, the last run of an unbroken link: a run that did not save its state (or failed to resume, or ended short) cuts
    the chain there, because the next run would start from a fresh brain, not the carried one."""
    cut = {}
    try:
        for l in open(f'{d}/runs.txt'):
            m = re.match(r'c(\d+) r(\d+) ', l)
            if m and not l.rstrip().endswith(' OK'):
                c, k = int(m.group(1)), int(m.group(2)); cut[c] = min(cut.get(c, k - 1), k - 1)
    except FileNotFoundError:
        pass
    return cut


def rows(f):
    out = []; last_pub = -1
    with gzip.open(f, 'rt') as fh:
        for l in fh:
            if not l.startswith('{') or '"sb_win"' not in l: continue
            try: r = json.loads(l)
            except Exception: continue
            if r.get('ps_eng', 0) < 1 or r.get('sb_win', -1) < 0: continue
            p = r.get('sf_pub', 0)
            if p <= last_pub: continue
            last_pub = p; out.append(r)
    return out


def baked(path):
    try:
        p = json.load(open(path)); g = json.loads(p['snapshots']['epm_stance'])['gng']
        g = json.loads(g) if isinstance(g, str) else g
        return sum(n['visits'] >= g['baking_threshold'] for n in g['nodes'])
    except Exception:
        return None


def mean_of(R, k):
    xs = [r[k] for r in R if k in r and r[k] is not None and r[k] >= 0]
    return st.mean(xs) if xs else None


def insertions(R):
    ins = sum(max(0, b['sb_n'] - a['sb_n']) for a, b in zip(R, R[1:]))
    return 1000.0 * ins / max(1, len(R))


def slope(xs, ys):
    pts = [(x, y) for x, y in zip(xs, ys) if y is not None and x > 0]
    if len(pts) < 2: return None
    lx = [math.log2(x) for x, _ in pts]; y = [v for _, v in pts]; mx, my = st.mean(lx), st.mean(y)
    sxx = sum((a - mx) ** 2 for a in lx)
    return None if sxx <= 0 else sum((a - mx) * (b - my) for a, b in zip(lx, y)) / sxx


def ms(xs, fmt='{:.3f}'):
    xs = [x for x in xs if x is not None]
    if not xs: return '-'
    return fmt.format(st.mean(xs)) + ('' if len(xs) < 2 else '±' + fmt.format(st.stdev(xs)))


def tstat(xs):
    xs = [x for x in xs if x is not None]
    if len(xs) < 2 or st.stdev(xs) == 0: return '-'
    return f'{st.mean(xs) / (st.stdev(xs) / math.sqrt(len(xs))):+.1f}'


METRICS = [('TLE', 'sb_tle'), ('QE', 'sb_qe'), ('TS', 'sb_ts')]


def main(dirs):
    for d in dirs:
        d = d.rstrip('/')
        sched = re.search(r'schedule \[([^\]]*)\]', open(f'{d}/manifest.txt').read()).group(1).split()
        chains = sorted({int(re.search(r'_c(\d+)_r', p).group(1)) for p in glob.glob(f'{d}/state_c*_r*.json')})
        worlds = list(dict.fromkeys(sched))
        print(f'== {os.path.basename(d)}  (chains {len(chains)}; schedule {" ".join(sched)})')
        # per chain: per world, the list of repetitions {k, mid_min, metrics, fifths}
        reps = {c: {w: [] for w in worlds} for c in chains}
        cut = broken_after(d)
        for c, k in sorted(cut.items()): print(f'   chain {c} broken after run {k}: its later runs began from a fresh brain and are left out')
        for c in chains:
            done = {w: 0.0 for w in worlds}
            for k, w in enumerate(sched, 1):
                if k > cut.get(c, len(sched)): break
                fs = glob.glob(f'{d}/r{k}/*_c{c}.jsonl.gz')
                if not fs: continue
                R = rows(fs[0])
                if len(R) < 50: continue
                dur = len(R) * ROW_S / 60.0
                rec = dict(k=k, mid=done[w] + dur / 2, ins=insertions(R), reach=max((r['z'] for r in R if 'z' in r), default=None))
                for name, key in METRICS: rec[name] = mean_of(R, key)
                bins = {}
                for r in R:
                    if 'z' in r: bins.setdefault(int(math.floor(r['z'] / BIN_M)), []).append(r)
                rec['bins'] = {b: {name: mean_of(B, key) for name, key in METRICS} for b, B in bins.items() if len(B) >= BIN_MIN}
                b1 = baked(f'{d}/state_c{c}_r{k}.json'); b0 = baked(f'{d}/state_c{c}_r{k - 1}.json') if (k > 1 or os.path.exists(f'{d}/state_c{c}_r0.json')) else 0
                rec['gain'] = None if b1 is None or b0 is None else b1 - b0
                rec['gap'] = bool(reps[c][w]) and k - reps[c][w][-1]['k'] > 2   # a gap: more than one other run since this world's last repetition
                reps[c][w].append(rec); done[w] += dur
        have_parts = any(r['QE'] is not None for c in chains for w in worlds for r in reps[c][w])
        names = [n for n, _ in METRICS if have_parts or n == 'TLE']
        for w in worlds:
            nrep = max(len(reps[c][w]) for c in chains)
            print(f'  -- {w}')
            print('     rep  run   min   ' + '  '.join(f'{n:>13s}' for n in names) + '      ins/1k    +baked   reach m')
            for i in range(nrep):
                R = [reps[c][w][i] for c in chains if len(reps[c][w]) > i]
                print(f'     {i + 1:3d} {R[0]["k"]:4d} {ms([r["mid"] for r in R], "{:.1f}"):>6s}  ' + '  '.join(f'{ms([r[n] for r in R]):>13s}' for n in names)
                      + f'  {ms([r["ins"] for r in R], "{:.1f}"):>10s}  {ms([r["gain"] for r in R], "{:.0f}"):>8s}  {ms([r["reach"] for r in R], "{:.2f}"):>9s}' + ('   (after a gap)' if any(r['gap'] for r in R) else ''))
            # the rates, over the repetitions before any gap (one block of practice)
            for n in names:
                sl, rel, mt, last, nb = [], [], [], [], []
                for c in chains:
                    block = []
                    for r in reps[c][w]:
                        if r['gap']: break
                        block.append(r)
                    if len(block) < 2: continue
                    s = slope([r['mid'] for r in block], [r[n] for r in block]); sl.append(s)
                    if s is not None and block[0][n]: rel.append(100.0 * s / block[0][n])
                    common = set.intersection(*[set(r['bins']) for r in block])
                    per_bin = [slope([r['mid'] for r in block], [r['bins'][b][n] for r in block]) for b in sorted(common)]
                    per_bin = [x for x in per_bin if x is not None]
                    mt.append(st.mean(per_bin) if per_bin else None); nb.append(len(per_bin))
                    if block[-1][n] is not None and block[-2][n] is not None: last.append(block[-1][n] - block[-2][n])
                if not sl: continue
                print(f'     {n:4s} per doubling of practice: slope {ms(sl, "{:+.4f}")} ({ms(rel, "{:+.1f}")} %, t {tstat(sl)})   '
                      f'matched {ms(mt, "{:+.4f}")} (t {tstat(mt)}, {ms(nb, "{:.0f}")} bins)   last rep {ms(last, "{:+.4f}")} (t {tstat(last)})')
            # retention across a gap
            for n in names:
                dl = []
                for c in chains:
                    L = reps[c][w]
                    for i in range(1, len(L)):
                        if L[i]['gap'] and L[i][n] is not None and L[i - 1][n] is not None: dl.append(L[i][n] - L[i - 1][n]); break
                if dl: print(f'     {n:4s} return after the gap minus the last before it: {ms(dl, "{:+.4f}")} (t {tstat(dl)})')


if __name__ == '__main__':
    main(sys.argv[1:])
