#!/usr/bin/env python3
"""stepper_curriculum.py — read a curriculum chain (stepper_chain.sh with SCHEDULE: known worlds first, then the stairs as a novel
world, then back) for the stance vocabulary (learned stepper plan Phase 3, operator 2026-11-15: "build up a performant brain in the
corridor or arena, then test it on the stairs ... see how the new knowledge is gained on the stairs as a novel environment").

    stepper_curriculum.py <chain dir> [<chain dir> ...]

Per run (mean over the chains; the bench from the manifest's schedule):
  nodes / baked at the run's end, baked gained IN the run;
  stance TLE over the run's second half (engaged rows) -- the learning curve, read per bench;
  reuse    the share of the run's engaged rows won by a node already BAKED when the stairs first began (the known-world vocabulary
           at work in the new world), and by a node baked before this run (all prior knowledge);
  heads    decode-head NIS of the stance filters on the same stream (own sb_hnis, legacy sbl_hnis, visits sbv_hnis; 1 = calibrated);
  switches MAP / winner (the belief's coasting) for each filter present.
Then retention: of the nodes baked before the stairs, how many survive to the last run, and the known worlds' TLE after the
stairs against before.  The carried states (state_c<c>_r<k>.json) give the baked sets; the logs give the rest.
"""
import glob, gzip, json, os, re, statistics as st, sys


def gng(path):
    p = json.load(open(path)); g = json.loads(p['snapshots']['epm_stance'])['gng']
    return json.loads(g) if isinstance(g, str) else g


def baked_ids(path):
    g = gng(path); bt = g['baking_threshold']
    return {n['id'] for n in g['nodes'] if n['visits'] >= bt}, {n['id'] for n in g['nodes']}


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


def m(xs, fmt='{:.3f}'):
    xs = [x for x in xs if x is not None]
    if not xs: return '-'
    return fmt.format(st.mean(xs)) + ('' if len(xs) < 2 else '±' + fmt.format(st.stdev(xs)))


def main(dirs):
    for d in dirs:
        d = d.rstrip('/')
        man = open(f'{d}/manifest.txt').read()
        sched = re.search(r'schedule \[([^\]]*)\]', man).group(1).split()
        chains = sorted({int(re.search(r'_c(\d+)_r', p).group(1)) for p in glob.glob(f'{d}/state_c*_r*.json')})
        first_stairs = next(k for k, b in enumerate(sched, 1) if b.startswith('b'))
        print(f'== {d}  ({os.path.basename(d)}; chains {len(chains)}; schedule {" ".join(sched)}; stairs from run {first_stairs})')
        cut = broken_after(d)
        for c, k in sorted(cut.items()): print(f'   chain {c} broken after run {k}: its later runs began from a fresh brain and are left out')
        known = {c: baked_ids(f'{d}/state_c{c}_r{first_stairs - 1}.json')[0] for c in chains}
        print('  run bench   nodes      baked      +baked   TLE(2nd half)   reuse known / prior   head NIS own / legacy / visits     MAP/winner own, legacy, visits')
        for k, b in enumerate(sched, 1):
            N, B, G, T, RK, RP, H, S = [], [], [], [], [], [], {'sb': [], 'sbl': [], 'sbv': []}, {'sb': [], 'sbl': [], 'sbv': []}
            for c in chains:
                st_k = f'{d}/state_c{c}_r{k}.json'
                if k > cut.get(c, len(sched)) or not os.path.exists(st_k): continue
                bk, nk = baked_ids(st_k); prior = baked_ids(f'{d}/state_c{c}_r{k - 1}.json')[0] if (k > 1 or os.path.exists(f'{d}/state_c{c}_r0.json')) else set()
                N.append(len(nk)); B.append(len(bk)); G.append(len(bk - prior))
                fs = glob.glob(f'{d}/r{k}/*_c{c}.jsonl.gz')
                if not fs: continue
                R = rows(fs[0])
                if not R: continue
                h = R[len(R) // 2:]; T.append(st.mean(r.get('sb_tle', 0.0) for r in h))
                RK.append(sum(r['sb_win'] in known[c] for r in R) / len(R)); RP.append(sum(r['sb_win'] in prior for r in R) / len(R))
                last = R[-1]
                for p in H:
                    if f'{p}_hnis' in last: H[p].append(last[f'{p}_hnis'])
                    sw = last.get(f'{p}_sw')
                    if sw: S[p].append(sw[0] / max(1, sw[1]))
            print(f'  {k:3d} {b:6s} {m(N, "{:.0f}"):>10s} {m(B, "{:.0f}"):>10s} {m(G, "{:.0f}"):>8s}  {m(T):>13s}   {m(RK, "{:.2f}"):>9s} / {m(RP, "{:.2f}"):9s}  '
                  f'{m(H["sb"], "{:.2f}")} / {m(H["sbl"], "{:.2f}")} / {m(H["sbv"], "{:.2f}")}     '
                  f'{m(S["sb"], "{:.2f}")}, {m(S["sbl"], "{:.2f}")}, {m(S["sbv"], "{:.2f}")}')
        last_k = len(sched)
        surv = [len(known[c] & baked_ids(f'{d}/state_c{c}_r{last_k}.json')[1]) / max(1, len(known[c])) for c in chains if c not in cut]
        print(f'  retention: of {m([len(known[c]) for c in chains], "{:.0f}")} nodes baked before the stairs, {m(surv, "{:.2f}")} survive to run {last_k}')


if __name__ == '__main__':
    main(sys.argv[1:])
