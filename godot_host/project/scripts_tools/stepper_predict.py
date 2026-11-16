#!/usr/bin/env python3
"""stepper_predict.py — does the stance belief make a STEP'S CONSEQUENCE more predictable? (learned stepper plan §9.4, step 0:
an offline read of chains already run; no authority, no new run.)

    stepper_predict.py <chain dir> [<chain dir> ...]      [KAPPA=2] [PCS=8] [P0=1] [JOBS=8] [SKIP=1]

Every landing of a chain (stepper_chain.sh) is one sample: the step-outcome record (ps_out) joined to the stance the step was
PLANNED in — the stance EPM's winner and the belief's MAP on the first logged row after the previous landing, which is when the
scaffold decided this step. Nothing from the step itself or after it enters a predictor.

Targets (each a consequence the scaffold's planner assumes or ignores):
  off     realised minus planned landing, 2-D (cm; planned steps only). The scaffold assumes 0: "the toe lands where I planned".
  big     that offset exceeding 2 cm (planned steps)
  dh      touchdown height minus the memory's predicted ground (cm; touched steps). The scaffold assumes 0.
  dur     the step's duration (s)
  hang    the step hung (the hang reflex fired)
  miss    the step ended without a touch
  nxbad   the next leg, predicted liftable, was found unliftable at the next decision (the plan-drift counter's increment)
  room    the smallest extension slack over the four legs after the landing (cm)
  pitch   POSITIVE CONTROL: the body's pitch on the planning row (deg). It is an input of the stance vector, so the stance must
          predict it; a low score here means this reader's join is broken, not that the stance is uninformative.

Predictors, all PREQUENTIAL (each sample is predicted from the samples before it in the chain's own time order, then learned
from — what an online learner in the brain would have had):
  zero    the scaffold's assumption (0; for the binaries "does not happen")
  leg     running mean per leg                      -- the reference: no stance, no action
  node    running mean per (leg, stance winner), shrunk to the leg mean by KAPPA pseudo-counts
  map     the same on the belief's MAP node
  node3+  `node` scored only on the steps whose cell had been seen three or more times, against the reference on those same
          steps: what the table is worth where it has data
  shuf    NEGATIVE CONTROL: `node` with the winner labels permuted across the chain's steps (must not beat `leg`)
  proto   per-leg recursive least squares on the winner's prototype (its first PCS principal components over the chain's
          vocabulary): does the stance LATENT carry it, generalising across nodes?
  act     per-leg RLS on the step's own parameters (planned landing x, z; predicted ground)   -- action, no stance
  a+p     per-leg RLS on both

Score: skill = 1 - SSE(model) / SSE(leg) on the scored steps (binaries: the Brier score's SSE), per chain, reported as the mean ±
sd over chains and t. Positive = the predictor beats the per-leg mean. The first SKIP runs of a chain are learned from and not
scored. Per world, with the sparsity that bounds a per-node table: steps, distinct (leg, node) cells, and the share of steps whose
cell had been seen at least once / three times before."""
import collections, glob, json, math, os, re, statistics as st, sys
from multiprocessing import Pool
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _logio import open_log

KEYS = ('t', 'sb_win', 'sb_map', 'ps_out', 'ps_drf', 'ps_ext', 'tilt_p')
TARGETS = ['off', 'big', 'dh', 'dur', 'hang', 'miss', 'nxbad', 'room', 'ext0', 'pitch']
BINARY = {'big', 'hang', 'miss', 'nxbad'}
MODELS = ['zero', 'wleg', 'leg', 'node', 'node3+', 'map', 'shuf', 'proto', 'act', 'a+p']
UNIT = {'off': 'cm', 'dh': 'cm', 'dur': 's', 'room': 'cm', 'ext0': 'cm', 'pitch': 'deg'}


def broken_after(d):
    cut = {}
    try:
        for l in open(f'{d}/runs.txt'):
            m = re.match(r'c(\d+) r(\d+) ', l)
            if m and not l.rstrip().endswith(' OK'):
                c, k = int(m.group(1)), int(m.group(2)); cut[c] = min(cut.get(c, k - 1), k - 1)
    except FileNotFoundError:
        pass
    return cut


def schedule(d):
    w = {}
    for l in open(f'{d}/runs.txt'):
        m = re.match(r'c(\d+) r(\d+) (\S+) ', l)
        if m: w[(int(m.group(1)), int(m.group(2)))] = m.group(3)
    return w


def steps_of(f):
    """one run's landings: the outcome record joined to the planning row (the first row after the previous landing)"""
    rows = []
    for l in open_log(f):
        if not l.startswith('{') or '"sb_win"' not in l: continue
        try: r = json.loads(l)
        except Exception: continue
        rows.append({k: r.get(k) for k in KEYS})
    first = {}
    for i, r in enumerate(rows):
        o = r['ps_out']
        if o and o[0] >= 1 and o[0] not in first: first[int(o[0])] = i
    out = []
    for q in sorted(first):
        if q - 1 not in first: continue          # the planning row is the previous landing's
        i, ip = first[q], first[q - 1]
        o, pr = rows[i]['ps_out'], rows[ip]
        if o[2] < 0: continue                    # untracked
        d0 = rows[i]['ps_drf']
        dprev = rows[max(i - 1, 0)]['ps_drf']
        ext = rows[i]['ps_ext']
        out.append(dict(seq=q, leg=int(o[1]), kind=int(o[2]), touched=int(o[3]), fx=o[4], fz=o[5], lx=o[6], lz=o[7], hp=o[8], ht=o[9],
                        ticks=o[10], hang=int(o[11]), win=int(pr['sb_win']), map=int(pr['sb_map']), pitch=math.degrees(pr['tilt_p'] or 0.0),
                        # the check of THIS step's prediction is made at the decision after it lands: the counter's step on this row
                        nxbad=int(d0[2] > dprev[2]) if d0 and dprev else 0, room=100 * min(ext) if ext else 0.0,
                        ext0=100 * pr['ps_ext'][int(o[1])] if pr['ps_ext'] else 0.0))
    return out


def protos(d, c, runs):
    """node id -> prototype, the latest saved one (analysis only: a node's prototype may have moved a little after the step)"""
    P = {}
    for k in sorted(runs, reverse=True):
        try:
            p = json.load(open(f'{d}/state_c{c}_r{k}.json')); g = json.loads(p['snapshots']['epm_stance'])['gng']
            g = json.loads(g) if isinstance(g, str) else g
        except Exception:
            continue
        for n in g['nodes']:
            if n['id'] not in P: P[n['id']] = np.asarray(n['prototype'], float)
    return P


class RLS:
    def __init__(self, nx, ny, p0=1.0):
        self.W = np.zeros((nx, ny)); self.P = np.eye(nx) * p0
    def predict(self, x): return x @ self.W
    def update(self, x, y):
        Px = self.P @ x; k = Px / (1.0 + x @ Px)
        self.W += np.outer(k, y - x @ self.W); self.P -= np.outer(k, Px)


class Means:
    """running means per key, the cell mean shrunk to its parent's by kappa pseudo-counts"""
    def __init__(self, ny, kappa): self.s = collections.defaultdict(lambda: np.zeros(ny)); self.n = collections.Counter(); self.k = kappa; self.ny = ny
    def mean(self, key): return self.s[key] / self.n[key] if self.n[key] else np.zeros(self.ny)
    def shrunk(self, key, parent): return (self.s[key] + self.k * self.mean(parent)) / (self.n[key] + self.k)
    def update(self, key, y): self.s[key] += y; self.n[key] += 1


def targets(s):
    y = {}
    if s['kind'] == 1:
        off = np.array([100 * (s['lx'] - s['fx']), 100 * (s['lz'] - s['fz'])]); y['off'] = off; y['big'] = np.array([float(np.hypot(*off) > 2.0)])
    if s['touched']: y['dh'] = np.array([100 * (s['ht'] - s['hp'])])
    if s['ticks'] > 0: y['dur'] = np.array([s['ticks'] / 60.0])
    y['hang'] = np.array([float(s['hang'])]); y['miss'] = np.array([float(not s['touched'])]); y['nxbad'] = np.array([float(s['nxbad'])])
    y['room'] = np.array([s['room']]); y['ext0'] = np.array([s['ext0']]); y['pitch'] = np.array([s['pitch']])
    return y


def chain_scores(S, P, kappa, pcs, skip, rng, p0):
    """S: the chain's steps in time order, each with run, world. Returns sse[world][target][model], n[world][target], sparsity."""
    ids = sorted(P); basis = None; mu = None
    if len(ids) > pcs:
        M = np.stack([P[i] for i in ids]); mu = M.mean(0); _, sv, vt = np.linalg.svd(M - mu, full_matrices=False)
        basis = vt[:pcs].T / (sv[:pcs] / math.sqrt(len(ids)))          # whitened components
    def pc(i): return (P[i] - mu) @ basis if basis is not None and i in P else np.zeros(pcs)
    perm = [s['win'] for s in S]; rng.shuffle(perm)
    sse = collections.defaultdict(lambda: collections.defaultdict(lambda: collections.Counter())); cnt = collections.defaultdict(collections.Counter)
    sp = collections.defaultdict(lambda: dict(n=0, seen1=0, seen3=0, cells=set(), nodes=set()))
    cellseen = collections.Counter()
    mdl = {}
    for t in TARGETS:
        ny = 2 if t == 'off' else 1
        mdl[t] = dict(wleg=Means(ny, kappa), leg=Means(ny, kappa), node=Means(ny, kappa), map=Means(ny, kappa), shuf=Means(ny, kappa),
                      proto=[RLS(1 + pcs, ny, p0) for _ in range(4)], act=[RLS(4, ny, p0) for _ in range(4)], ap=[RLS(4 + pcs, ny, p0) for _ in range(4)])
    for j, s in enumerate(S):
        leg, w = s['leg'], s['world']; scored = s['run_idx'] >= skip
        xp = np.concatenate([[1.0], pc(s['win'])]); xa = np.array([1.0, 10 * s['fx'], 10 * s['fz'], 10 * s['hp']]); xap = np.concatenate([xa, xp[1:]])
        cell = (leg, s['win']); seen3 = cellseen[cell] >= 3
        if scored:
            q = sp[w]; q['n'] += 1; q['seen1'] += cellseen[cell] >= 1; q['seen3'] += cellseen[cell] >= 3; q['cells'].add(cell); q['nodes'].add(s['win'])
        cellseen[cell] += 1
        for t, y in targets(s).items():
            m = mdl[t]; L = ('l', leg)
            pred = {'zero': np.zeros_like(y), 'wleg': m['wleg'].mean((w, leg)), 'leg': m['leg'].mean(L), 'node': m['node'].shrunk((leg, s['win']), L) if m['node'].n[L] else m['node'].mean(L),
                    'map': m['map'].shrunk((leg, s['map']), L) if m['map'].n[L] else m['map'].mean(L),
                    'shuf': m['shuf'].shrunk((leg, perm[j]), L) if m['shuf'].n[L] else m['shuf'].mean(L),
                    'proto': m['proto'][leg].predict(xp), 'act': m['act'][leg].predict(xa), 'a+p': m['ap'][leg].predict(xap)}
            if scored:
                cnt[w][t] += 1
                for k, p in pred.items(): sse[w][t][k] += float(np.sum((y - p) ** 2))
                if seen3: sse[w][t]['node3+'] += float(np.sum((y - pred['node']) ** 2)); sse[w][t]['_ref3'] += float(np.sum((y - pred['wleg']) ** 2))
            m['leg'].update(L, y); m['wleg'].update((w, leg), y)
            for name, key in (('node', s['win']), ('map', s['map']), ('shuf', perm[j])):
                m[name].update(L, y); m[name].update((leg, key), y)
            m['proto'][leg].update(xp, y); m['act'][leg].update(xa, y); m['ap'][leg].update(xap, y)
    return sse, cnt, sp


def ms(xs):
    if not xs: return '      -      '
    m = st.mean(xs); s = st.stdev(xs) if len(xs) > 1 else 0.0
    t = m / (s / math.sqrt(len(xs))) if s > 0 else float('nan')
    return f'{m:+.2f}±{s:.2f} t{t:+5.1f}'


def main(argv):
    opt = {'KAPPA': 2.0, 'PCS': 8, 'JOBS': 8, 'SKIP': 1, 'P0': 1.0}
    dirs = []
    for a in argv:
        m = re.match(r'([A-Z0-9]+)=(.*)', a)
        if m: opt[m.group(1)] = float(m.group(2))
        else: dirs.append(a.rstrip('/'))
    for d in dirs:
        sched = schedule(d); cut = broken_after(d)
        files = {}
        for (c, k), w in sched.items():
            if c in cut and k > cut[c]: continue
            fs = glob.glob(f'{d}/r{k}/*_c{c}.jsonl.gz') + glob.glob(f'{d}/r{k}/*_c{c}.jsonl')
            if fs: files[(c, k)] = fs[0]
        keys = sorted(files)
        with Pool(int(opt['JOBS'])) as pool: res = pool.map(steps_of, [files[k] for k in keys])
        chains = collections.defaultdict(list)
        for (c, k), ss in zip(keys, res):
            for s in ss: s['run'] = k; s['world'] = sched[(c, k)]; chains[c].append(s)
        print(f'== {d}: {len(chains)} chains, {sum(len(v) for v in chains.values())} steps joined to a planning row (KAPPA {opt["KAPPA"]:g}, PCS {int(opt["PCS"])}, P0 {opt["P0"]:g}, first {int(opt["SKIP"])} run(s) unscored)')
        skill = collections.defaultdict(lambda: collections.defaultdict(lambda: collections.defaultdict(list)))   # world, target, model -> per chain
        base = collections.defaultdict(lambda: collections.defaultdict(lambda: collections.defaultdict(list)))    # rms of zero / leg, n
        spar = collections.defaultdict(list)
        for c in sorted(chains):
            S = sorted(chains[c], key=lambda s: (s['run'], s['seq'])); runs = sorted({s['run'] for s in S})
            for s in S: s['run_idx'] = runs.index(s['run'])
            sse, cnt, sp = chain_scores(S, protos(d, c, runs), opt['KAPPA'], int(opt['PCS']), int(opt['SKIP']), np.random.default_rng(1000 + c), opt['P0'])
            for w in sse:
                for t in sse[w]:
                    n = cnt[w][t]; ref = sse[w][t]['wleg']
                    base[w][t]['n'].append(n); base[w][t]['zero'].append(math.sqrt(sse[w][t]['zero'] / n)); base[w][t]['wleg'].append(math.sqrt(ref / n))
                    for k in MODELS:
                        r = sse[w][t]['_ref3'] if k == 'node3+' else ref
                        if r > 0: skill[w][t][k].append(1.0 - sse[w][t][k] / r)
            for w, q in sp.items(): spar[w].append((q['n'], len(q['cells']), len(q['nodes']), q['seen1'] / max(1, q['n']), q['seen3'] / max(1, q['n'])))
        for w in sorted(skill):
            q = spar[w]
            print(f'\n-- world {w}: per chain {st.mean(x[0] for x in q):.0f} scored steps over {st.mean(x[2] for x in q):.0f} stance nodes, {st.mean(x[1] for x in q):.0f} (leg, node) cells;'
                  f' cell seen before {100 * st.mean(x[3] for x in q):.0f}%, seen 3+ times {100 * st.mean(x[4] for x in q):.0f}%')
            print(f'   {"target":6} {"n":>5} {"rms zero":>9} {"rms wleg":>8}   ' + '  '.join(f'{k:^19}' for k in MODELS if k not in ('zero', 'wleg')) + '      (skill vs wleg: mean±sd t over chains)')
            for t in TARGETS:
                if t not in skill[w]: continue
                b = base[w][t]; u = UNIT.get(t, 'p')
                print(f'   {t:6} {st.mean(b["n"]):5.0f} {st.mean(b["zero"]):7.2f}{u:>2} {st.mean(b["wleg"]):6.2f}{u:>2}   ' + '  '.join(f'{ms(skill[w][t][k]):^19}' for k in MODELS if k not in ('zero', 'wleg')))


if __name__ == '__main__':
    main(sys.argv[1:])
