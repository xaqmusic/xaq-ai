#!/usr/bin/env python3
"""seq_instruments.py — the PoseSequencer's two 2026-11-14 instruments, read from a battery directory (stepper_battery.sh).

    seq_instruments.py <dir> [bench prefix ...]        default prefixes: L42 b38 b64 cor

blocker (ps_bkr, ps_bkd, ps_bksh): for each leg whose turn was skipped for want of a lift margin, which toe and which check refused
  the smallest margin-giving shift (outer reach, inner reach, protection, sector, extension margin, lift floor), or that no shift in
  the grid gives the margin; with the mean cheapest margin shift and how far the grid fell short.
drift (ps_drf, ps_drc, ps_drm, ps_drma): each decision's prediction of the next leg's lift margin checked at the next decision —
  how often a predicted-liftable leg was found unliftable, the single factor that breaks it when swapped from predicted to actual
  (the stepping toe's landing, the other toes, the centre of mass, only all together), and the mean offsets (m) in those cases
  against all checked decisions. This is the plan's residual: the scaffold's analytic baseline that the learned step generator
  must beat (learned_stepper_plan.md Phase 4).
reloc (ps_rln, ps_rls, ps_rlt, ps_rlo, ps_rlr): the landing-relocation tag — each landing's offset from where its step set out,
  credited to the rule that moved it (reach clamp, sector clamp, edge move, obstacle reflex, lowering retreat, the IK's shortfall
  at the touch, clip); the closure residual is 0 when every move is tagged. The first two readers were session-scratchpad scripts until 2026-11-14."""
import collections, glob, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _logio import open_log

LEG = ['FL', 'FR', 'RL', 'RR']
WHY = {1: 'outer reach', 2: 'inner reach', 3: 'protection', 4: 'sector', 5: 'ext margin', 6: 'lift floor', 0: 'FEASIBLE?!'}


def last_row(f):
    r = None
    for l in open_log(f):
        if l.startswith('{') and '"ps_drf"' in l:
            try: r = json.loads(l)
            except Exception: pass
    return r


def files(d, bench):
    fs = sorted(glob.glob(f'{d}/*_{bench}*.jsonl') + glob.glob(f'{d}/*_{bench}*.jsonl.gz'))
    by_arm = collections.defaultdict(list)
    for f in fs: by_arm[os.path.basename(f).split('_', 1)[0]].append(f)
    return by_arm


def blocker(rs):
    tot = collections.Counter(); d = [0.0] * 4; sh = [0.0] * 4; nd = [0] * 4; ng = [0] * 4
    for r in rs:
        for leg in range(4):
            for idx, n in enumerate(r.get('ps_bkr', [[0] * 29] * 4)[leg]):
                if not n: continue
                if idx == 28: tot[(leg, 'grid', 'short')] += n; ng[leg] += n
                else: tot[(leg, LEG[idx // 7], WHY[idx % 7])] += n; nd[leg] += n
            d[leg] += (r.get('ps_bkd') or [0] * 4)[leg]; sh[leg] += (r.get('ps_bksh') or [0] * 4)[leg]
    for leg in range(4):
        items = [(k, v) for k, v in tot.items() if k[0] == leg]; n = sum(v for _, v in items)
        if not n: print(f'    {LEG[leg]} skipped for no margin: 0'); continue
        parts = ', '.join(f'{k[1]} {k[2]} {v} ({100 * v / n:.0f}%)' for k, v in sorted(items, key=lambda kv: -kv[1])[:5])
        print(f'    {LEG[leg]} skipped for no margin {n:4d}: {parts} | cheapest shift {100 * d[leg] / max(1, nd[leg]):.1f} cm'
              f' | grid short {100 * sh[leg] / max(1, ng[leg]):.1f} cm')


def drift(rs):
    f = [0] * 4; c = [0] * 4; m = [0.0] * 3; ma = [0.0] * 3
    for r in rs:
        for i in range(4): f[i] += r['ps_drf'][i]; c[i] += r['ps_drc'][i]
        for i in range(3): m[i] += r['ps_drm'][i]; ma[i] += r['ps_drma'][i]
    print(f'    drift: checked {f[0]}, predicted liftable {f[1]}, found unliftable {f[2]} ({100 * f[2] / max(1, f[1]):.1f}%),'
          f' predicted unliftable {f[3]}')
    print(f'    broken by: landed toe {c[0]}, other toes {c[1]}, centre of mass {c[2]}, only together {c[3]}')
    print(f'    offsets when broken (landed toe / other toes / CoM, cm): {100 * m[0] / max(1, f[2]):.1f} / {100 * m[1] / max(1, f[2]):.1f}'
          f' / {100 * m[2] / max(1, f[2]):.2f}   on all checked: {100 * ma[0] / max(1, f[0]):.1f} / {100 * ma[1] / max(1, f[0]):.1f}'
          f' / {100 * ma[2] / max(1, f[0]):.2f}')


RULES = ['reach clamp', 'sector clamp', 'edge move', 'obstacle', 'retreat', 'ik shortfall', 'clip']


def reloc(rs):
    rs = [r for r in rs if 'ps_rln' in r]
    if not rs: print('    reloc: not in this log (pre-2026-11-14 build)'); return
    n = [sum(r['ps_rln'][i] if i < len(r['ps_rln']) else 0 for r in rs) for i in range(7)]; sm = [sum(r['ps_rls'][i] for r in rs) for i in range(7)]
    top = [sum(r['ps_rlt'][i] for r in rs) for i in range(7)]; off = sum(r['ps_rlo'] for r in rs); res = max(r['ps_rlr'] for r in rs)
    print(f'    reloc: landings {n[0]}: planned {n[1]}, unplanned {n[2]}, leg switched {n[3]}; planned off > 2 cm {n[4]}'
          f' ({100 * n[4] / max(1, n[1]):.1f}%), with a hang {n[5]}, abandoned {n[6]}; mean planned offset {100 * off / max(1, n[1]):.1f} cm; closure {res:.4f} m')
    order = sorted(range(7), key=lambda i: -sm[i])
    print('    moved by (mean cm per planned landing | >2 cm landings led): '
          + ', '.join(f'{RULES[i]} {100 * sm[i] / max(1, n[1]):.2f} | {top[i]}' for i in order if sm[i] > 0 or top[i]))


def main(argv):
    d = argv[0].rstrip('/'); benches = argv[1:] or ['L42', 'b38', 'b64', 'cor']
    for b in benches:
        for arm, fs in sorted(files(d, b).items()):
            rs = [r for r in (last_row(f) for f in fs) if r]
            if not rs: continue
            print(f'== {arm} {b} ({len(rs)} runs)'); drift(rs); reloc(rs); blocker(rs)


if __name__ == '__main__':
    main(sys.argv[1:])
