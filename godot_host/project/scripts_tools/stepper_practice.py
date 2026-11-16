#!/usr/bin/env python3
"""stepper_practice.py — practice curves of STEP OUTCOME ERROR AND BEHAVIOUR over a scheduled chain (learned stepper plan Phase 4's
instrument, 2026-11-16): stepper_learning.py's sibling, re-pointed from the stance vocabulary's fit to what a controlling learner
is judged on. Read on the gain-0 chains before any arm; an arm is then read against them.

    stepper_practice.py <chain dir> [<chain dir> ...]      [JOBS=8]

The x-axis is practice: minutes of engaged stance in one world class before the repetition's midpoint. Per world and repetition
(mean ± sd over the chains):
  td cm     touchdown error: rms of (touch height - the memory's predicted ground) over the steps that touched
  corr cm   the hand's correction: the summed length of the tagged landing moves (reach, sector, edge, obstacle, retreat, IK,
            clip) per planned step
  hang %    steps on which the hang reflex fired          miss %   steps that ended without a touch
  skip/100  turns the scaffold skipped (no lift margin, no landing, a miss) per 100 steps -- the floor's refusals as it stands
  step/min  landings per minute
  prog      net forward progress, cm per minute (god's-eye z, analysis only)
  stall %   rows with less than 1 cm of progress over the next 2 s
  falls     auto resets in the run
  risers    stairs only: the highest riser any toe stood on, and the highest all four stood on (god's-eye, analysis only)
Then, per metric, the change per DOUBLING of practice over the repetitions before any gap (fit per chain; mean ± sd, % of the
first repetition, t over chains) and the last repetition minus the one before. Blind-metric complements, read together: a lower
touchdown error with lower progress or fewer risers is a body that stopped trying; high step/min with high stall is stepping in
place."""
import glob, json, math, os, re, statistics as st, sys
from multiprocessing import Pool
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _logio import open_log
from stepper_learning import broken_after, slope, ms, tstat

ROW_S = 5 / 60.0
STALL_ROWS, STALL_M = 24, 0.01
COLS = [('td', 'td cm', '{:.2f}'), ('corr', 'corr cm', '{:.2f}'), ('hang', 'hang %', '{:.1f}'), ('miss', 'miss %', '{:.1f}'), ('skip', 'skip/100', '{:.1f}'),
        ('spm', 'step/min', '{:.1f}'), ('prog', 'prog cm/min', '{:.1f}'), ('stall', 'stall %', '{:.0f}'), ('falls', 'falls', '{:.1f}'),
        ('top', 'riser any', '{:.2f}'), ('all', 'riser all', '{:.2f}')]


def pm(xs, nd):
    xs = [x for x in xs if x is not None]
    if not xs: return '-'
    return f'{st.mean(xs):+.{nd}f}' + ('' if len(xs) < 2 else f'±{st.stdev(xs):.{nd}f}')


def read(arg):
    f, riser_m = arg
    z = []; recs = {}; last = None; eng = 0; top = 0.0; allup = 0.0; r0 = None
    for l in open_log(f):
        if not l.startswith('{') or '"fwd_v"' not in l: continue
        try: r = json.loads(l)
        except Exception: continue
        if r0 is None: r0 = r
        last = r
        if 'z' in r: z.append(r['z'])
        if r.get('ps_eng', 0) >= 1: eng += 1
        o = r.get('ps_out')
        if o and o[0] >= 1 and o[0] not in recs: recs[o[0]] = o
        th = r.get('stairs_truth_h')
        if th and riser_m > 0: top = max(top, max(th) / riser_m); allup = max(allup, min(th) / riser_m)
    if last is None or eng < 50 or len(z) < 2 * STALL_ROWS: return None
    R = [o for o in recs.values() if o[2] >= 0]; n = max(1, len(R)); minutes = eng * ROW_S / 60.0
    touched = [o for o in R if o[3] == 1]; plan = [o for o in R if o[2] == 1]
    m = dict(minutes=minutes, steps=len(R))
    m['td'] = 100 * math.sqrt(st.mean((o[9] - o[8]) ** 2 for o in touched)) if touched else None
    m['corr'] = 100 * st.mean(sum(math.hypot(o[12 + 2 * k], o[13 + 2 * k]) for k in range(7)) for o in plan) if plan else None
    m['hang'] = 100.0 * sum(o[11] for o in R) / n; m['miss'] = 100.0 * sum(1 - o[3] for o in R) / n
    m['skip'] = 100.0 * (sum(last.get('ps_skm', [])) + sum(last.get('ps_skl', [])) + sum(last.get('ps_skx', []))) / n
    m['spm'] = len(R) / minutes
    m['prog'] = 100.0 * (z[-1] - z[0]) / (len(z) * ROW_S / 60.0)
    m['stall'] = 100.0 * sum(abs(z[i + STALL_ROWS] - z[i]) < STALL_M for i in range(len(z) - STALL_ROWS)) / (len(z) - STALL_ROWS)
    m['falls'] = float(last.get('auto_reset_count', 0) - r0.get('auto_reset_count', 0))
    m['top'] = top if riser_m > 0 else None; m['all'] = allup if riser_m > 0 else None
    return m


def main(argv):
    jobs = 8; dirs = []
    for a in argv:
        if a.startswith('JOBS='): jobs = int(a[5:])
        else: dirs.append(a.rstrip('/'))
    for d in dirs:
        sched = re.search(r'schedule \[([^\]]*)\]', open(f'{d}/manifest.txt').read()).group(1).split()
        chains = sorted({int(re.search(r'_c(\d+)_r', p).group(1)) for p in glob.glob(f'{d}/state_c*_r*.json')})
        worlds = list(dict.fromkeys(sched)); cut = broken_after(d)
        print(f'== {os.path.basename(d)}  (chains {len(chains)}; schedule {" ".join(sched)})')
        for c, k in sorted(cut.items()): print(f'   chain {c} broken after run {k}: its later runs began from a fresh brain and are left out')
        todo = []
        for c in chains:
            for k, w in enumerate(sched, 1):
                if k > cut.get(c, len(sched)): break
                fs = glob.glob(f'{d}/r{k}/*_c{c}.jsonl.gz')
                if fs: todo.append((c, k, w, fs[0], int(w[1:]) / 1000.0 if re.fullmatch(r'b\d+', w) else 0.0))
        with Pool(jobs) as pool: res = pool.map(read, [(t[3], t[4]) for t in todo])
        reps = {c: {w: [] for w in worlds} for c in chains}; done = {c: {w: 0.0 for w in worlds} for c in chains}
        for (c, k, w, _, _), m in zip(todo, res):
            if m is None: continue
            m['k'] = k; m['mid'] = done[c][w] + m['minutes'] / 2; done[c][w] += m['minutes']
            m['gap'] = bool(reps[c][w]) and k - reps[c][w][-1]['k'] > 2
            reps[c][w].append(m)
        for w in worlds:
            cols = [c for c in COLS if any(r[c[0]] is not None for ch in chains for r in reps[ch][w])]
            nrep = max((len(reps[c][w]) for c in chains), default=0)
            print(f'  -- {w}')
            print('     rep  run    min  ' + ' '.join(f'{h:>12s}' for _, h, _ in cols))
            for i in range(nrep):
                R = [reps[c][w][i] for c in chains if len(reps[c][w]) > i]
                print(f'     {i + 1:3d} {R[0]["k"]:4d} {ms([r["mid"] for r in R], "{:.1f}"):>6s}  ' + ' '.join(f'{ms([r[key] for r in R], fmt):>12s}' for key, _, fmt in cols)
                      + ('   (after a gap)' if any(r['gap'] for r in R) else ''))
            for key, head, _ in cols:
                sl, rel, last = [], [], []
                for c in chains:
                    block = []
                    for r in reps[c][w]:
                        if r['gap']: break
                        block.append(r)
                    if len(block) < 2: continue
                    s = slope([r['mid'] for r in block], [r[key] for r in block]); sl.append(s)
                    if s is not None and block[0][key]: rel.append(100.0 * s / block[0][key])
                    if block[-1][key] is not None and block[-2][key] is not None: last.append(block[-1][key] - block[-2][key])
                if any(x is not None for x in sl):
                    print(f'     {head:12s} per doubling of practice: {pm(sl, 3)} ({pm(rel, 1)} %, t {tstat(sl)})   last rep {pm(last, 3)} (t {tstat(last)})')


if __name__ == '__main__':
    main(sys.argv[1:])
