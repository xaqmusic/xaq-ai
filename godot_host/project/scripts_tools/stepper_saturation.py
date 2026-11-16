#!/usr/bin/env python3
"""stepper_saturation.py — the stance encoder's SATURATION per feature kind and world (learned stepper plan §9, 2026-11-16).

    stepper_saturation.py <chain dir> [<chain dir> ...]

StanceFeatures conditions each of its 42 values by a running mean and variance and clips z at +-clip. Its instrument (always on)
counts, per feature kind, the samples at the clip and the largest |z| before the clip (sf_sat, sf_zmax over sf_satn samples; per
run, read from each run's last row). A kind at the clip is a kind the scale cannot hold: the stance EPM sees a constant there.

Per world (the bench of each run in runs.txt), over the runs of every chain:
  clip %   the share of that kind's samples at the clip (a leg kind has four values a sample)
  peak z   the largest |z| before the clip, mean over runs and the largest of any run
  |z|      the unclipped stance norm at the run's last row against the clipped one (sf_zr, sf_z) -- a spot value, not a mean"""
import collections, glob, gzip, json, re, statistics as st, sys

KINDS = ['ext', 'k.str', 'k.fold', 'hip1', 'hip2', 'lift', 'load', 'contact', 'ground', 'relief', 'pitch', 'roll']


def last_row(f):
    r = None
    with gzip.open(f, 'rt', errors='replace') as fh:
        for l in fh:
            if l.startswith('{') and '"sf_sat"' in l:
                try: r = json.loads(l)
                except Exception: pass
    return r


def main(argv):
    for d in argv:
        d = d.rstrip('/'); W = collections.defaultdict(list)
        for l in open(f'{d}/runs.txt'):
            m = re.match(r'c(\d+) r(\d+) (\S+) ', l)
            if not m or not l.rstrip().endswith(' OK'): continue
            fs = glob.glob(f'{d}/r{m.group(2)}/*_c{m.group(1)}.jsonl.gz')
            r = last_row(fs[0]) if fs else None
            if r and r.get('sf_satn', 0) > 0: W[m.group(3)].append(r)
        print(f'== {d}')
        if not W: print('   no sf_sat in these logs (built before the instrument)'); continue
        print(f'   {"world":6} {"runs":>4}  ' + ' '.join(f'{k:>8}' for k in KINDS) + '    |z| unclipped / clipped')
        for w in sorted(W):
            R = W[w]
            share = [st.mean(100.0 * r['sf_sat'][k] / (r['sf_satn'] * (4 if k < 10 else 1)) for r in R) for k in range(12)]
            zmean = [st.mean(r['sf_zmax'][k] for r in R) for k in range(12)]; zmax = [max(r['sf_zmax'][k] for r in R) for k in range(12)]
            print(f'   {w:6} {len(R):4}  ' + ' '.join(f'{x:7.1f}%' for x in share) + f'    {st.mean(r.get("sf_zr", 0) for r in R):.1f} / {st.mean(r.get("sf_z", 0) for r in R):.1f}')
            print(f'   {"":6} {"peak":>4}  ' + ' '.join(f'{x:8.1f}' for x in zmean) + '    (mean over runs)')
            print(f'   {"":6} {"":>4}  ' + ' '.join(f'{x:8.1f}' for x in zmax) + '    (largest)')


if __name__ == '__main__':
    main(sys.argv[1:])
