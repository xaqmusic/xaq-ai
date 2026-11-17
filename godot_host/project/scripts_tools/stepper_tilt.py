#!/usr/bin/env python3
# stepper_tilt.py <log.jsonl[.gz]> ... : one leg landing three or more times in a row ("tapping") against the chassis tilt, and what the
# planner's block instrument (ps_bkr) names inside it. Written 2026-11-17 for the operator's "front right toe tapping in place" past 10 degrees.
import gzip, json, sys, math, collections
NAMES = ['FL', 'FR', 'RL', 'RR']; WHY = ['shift ok, model refuses', 'reach max', 'reach min', 'protect', 'sector', 'extension margin', '?']
HIP = [(-0.0383, 0.033), (0.0383, 0.033), (-0.0383, -0.033), (0.0383, -0.033)]   # hip1 mounts: HIP_X_SPAN 76.6 mm, HIP_Z_SPAN 66 mm (was +-49 mm until 2026-11-17 evening: the radii printed before then read ~10 mm short)
BINS = [(0, 5), (5, 10), (10, 15), (15, 90)]
def load(p):
    rows = []
    try:
        with gzip.open(p, 'rt') as f:
            for l in f:
                if l.startswith('{') and '"fwd_v"' in l:
                    try: rows.append(json.loads(l))
                    except Exception: pass
    except EOFError: pass
    return rows
def bin_of(r):
    td = math.degrees(abs(r['tilt'])); return next(b for b in BINS if b[0] <= td < b[1])
eng = collections.Counter(); tapt = collections.Counter(); blk = {b: collections.Counter() for b in BINS}; tapblk = collections.Counter()
toe_rank = collections.Counter(); toe_r = []; eps = []; legs = collections.Counter(); land = collections.Counter(); adv = collections.defaultdict(float)
for p in sys.argv[1:]:
    rows = load(p)
    if len(rows) < 10: print(p, 'too short'); continue
    seq = []
    for i in range(1, len(rows)):
        a, b = rows[i - 1]['ps_steps'], rows[i]['ps_steps']
        for l in range(4):
            for _ in range(max(0, b[l] - a[l])): seq.append((i, l))
    intap = {}; j = 0; mine = []
    while j < len(seq):
        k = j
        while k + 1 < len(seq) and seq[k + 1][1] == seq[j][1]: k += 1
        if k - j + 1 >= 3:
            i0, i1 = seq[j][0], seq[k][0]
            for i in range(i0, i1 + 1): intap[i] = seq[j][1]
            tl = [math.degrees(abs(r['tilt'])) for r in rows[i0:i1 + 1]]
            mine.append(((i1 - i0) * 0.1, NAMES[seq[j][1]], k - j + 1, sum(tl) / len(tl), max(tl), rows[i0]['t']))
            legs[NAMES[seq[j][1]]] += 1
        j = k + 1
    n_eng = 0; over10 = 0
    for i in range(1, len(rows)):
        r, q = rows[i], rows[i - 1]
        if r.get('ps_eng', 0) < 0.5: continue
        b = bin_of(r); eng[b] += 1; n_eng += 1; over10 += b[0] >= 10
        land[b] += max(0, r['ps_land'] - q['ps_land'])
        if i in intap: tapt[b] += 1
        for leg in range(4):
            for idx in range(29):
                d = r['ps_bkr'][leg][idx] - q['ps_bkr'][leg][idx]
                if d <= 0: continue
                toe = idx // 7
                key = 'no shift on the grid gives the margin' if idx == 28 else '%s of %s' % (WHY[idx % 7], 'the leg itself' if toe == leg else 'another toe')
                blk[b][key] += d
                if i in intap:
                    tapblk[key] += d
                    if idx != 28 and toe != leg and idx % 7 == 2:
                        order = sorted(range(4), key=lambda t: -r['toe_y'][t]); toe_rank[order.index(toe)] += d
                        toe_r.append(math.hypot(r['foot_xz'][toe][0] - HIP[toe][0], r['foot_xz'][toe][1] - HIP[toe][1]))
    mine.sort(reverse=True)
    e0 = next(r for r in rows if r.get('ps_eng', 0) >= 0.5); net = math.hypot(rows[-1]['x'] - e0['x'], rows[-1]['z'] - e0['z'])
    print('%s: %.1f min engaged, tilt > 10 deg %.0f%% of it, net %.2f m, landings %d, falls %d; tapping %d episodes, %.0f s in all; longest: %s' % (
        p.split('/')[-1], n_eng / 600, 100 * over10 / max(1, n_eng), net, rows[-1]['ps_land'], rows[-1].get('auto_reset_count', 0), len(mine), sum(m[0] for m in mine),
        '; '.join('%s x%d %.0f s at %.0f deg (max %.0f), tick %d' % (m[1], m[2], m[0], m[3], m[4], m[5]) for m in mine[:3])))
    eps += mine
print()
for b in BINS:
    if not eng[b]: continue
    tot = sum(blk[b].values())
    print('tilt %2d-%2d deg: %5.1f min engaged, landings/min %4.1f, tapping %4.1f%% of the time, blocked plans/min %4.1f: %s' % (
        b[0], b[1], eng[b] / 600, land[b] / (eng[b] / 600), 100 * tapt[b] / eng[b], tot / (eng[b] / 600), ', '.join('%s %.0f%%' % (k, 100 * v / tot) for k, v in blk[b].most_common(4)) if tot else '-'))
tot = sum(tapblk.values())
if tot: print('inside tapping:', ', '.join('%s %.0f%%' % (k, 100 * v / tot) for k, v in tapblk.most_common(5)))
if toe_r:
    n = sum(toe_rank.values())
    print('the toe at its minimum reach: highest toe %.0f%%, 2nd %.0f%%, 3rd %.0f%%, lowest %.0f%%; its radius from the hip %.0f mm (median)' % (*[100 * toe_rank[k] / n for k in range(4)], 1000 * sorted(toe_r)[len(toe_r) // 2]))
print('tapping leg:', dict(legs), ' episodes >= 20 s:', sum(1 for e in eps if e[0] >= 20), ' >= 60 s:', sum(1 for e in eps if e[0] >= 60))
