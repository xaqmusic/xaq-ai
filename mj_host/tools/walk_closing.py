#!/usr/bin/env python3
"""walk_closing -- does the walk go where its loops point?  (2026-09-29, the operator's eye on R108 seed 1: the duck
walks past the thing it attends and circles the block it seeks.)

Over the 'seek', 'hdg', 'steer', 'twist' and 'thg' records of full logs:
  * with a target held on the walk: the share of 1 s windows closing on it (range rate < -0.05 m/s), tangential, opening;
    the same split by who holds the heading (play 1 / seek 3);
  * the heading error |ref - heading| while seek steers;
  * the sign agreement of the yaw command with the reference over half-second windows, by error size;
  * attended-on-the-walk episodes >= 0.5 s where the attended thing is 0.3 m nearer than the held target (or none held);
  * arrival near-misses (the range under 0.25 m then out past 0.35 m without an arrival), arrival stops.

  python3 mj_host/tools/walk_closing.py 'mj_host/log/campaign/c15/a1v2_r108_yield_forgets_s*.jsonl' [more globs]
"""
import glob, json, math, sys
from collections import Counter


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def score(paths):
    rad = Counter(); by_steer = Counter(); err_seek = []; sign = {}; A = 0; C = 0; arrivals = 0; held_s = 0.0
    for path in paths:
        hist = []; shist = []; in_a = 0; near = False
        for line in open(path):
            if not line.startswith('{'):
                continue
            r = json.loads(line); sk = r.get('seek'); th = r.get('thg')
            walking = r['drive'] == 'walk' and r.get('stop', 0) == 0
            held = bool(sk) and sk[0] > 0
            if th is not None:
                if walking and th[2] >= 0 and (not held or th[5] < sk[1] - 0.3):
                    in_a += 1
                    if in_a == 6: A += 1
                else:
                    in_a = 0
            if held:
                if sk[1] < 0.25: near = True
                if near and sk[1] > 0.35: C += 1; near = False
            else:
                near = False
            if 'stop:arrive' in r.get('event', ''): arrivals += 1; near = False
            if walking and held:
                held_s += 0.02
                h = r['hdg']; e = wrap(h[1] - h[0])
                hist.append((r['t'], sk[1], r.get('steer'), abs(e)))
                if len(hist) > 50: hist.pop(0)
                if len(hist) == 50 and hist[-1][0] - hist[0][0] < 1.2:
                    v = (hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0])
                    cls = 'closing' if v < -0.05 else ('opening' if v > 0.05 else 'tangential')
                    st = Counter(x[2] for x in hist).most_common(1)[0][0]
                    rad[cls] += 1; by_steer[(st, cls)] += 1
                    if st == 3: err_seek.append(sum(x[3] for x in hist) / 50)
            else:
                hist = []
            if walking and r.get('steer') in (1, 3):
                h = r['hdg']; e = wrap(h[1] - h[0]); shist.append((r['t'], e, r['twist'][2]))
                if len(shist) > 25: shist.pop(0)
                if len(shist) == 25 and shist[-1][0] - shist[0][0] < 0.6:
                    me = sum(x[1] for x in shist) / 25; wz = sum(x[2] for x in shist) / 25; ae = abs(me)
                    k = '0-0.3' if ae < 0.3 else ('0.3-0.8' if ae < 0.8 else ('0.8-1.5' if ae < 1.5 else ('1.5-2.5' if ae < 2.5 else '>2.5')))
                    if abs(wz) > 0.3:
                        d = sign.setdefault(k, [0, 0]); d[0 if wz * me > 0 else 1] += 1
                    shist = []
            else:
                shist = []
    n = len(paths); tot = max(1, sum(rad.values()))
    print(f"{n} logs; a target held on the walk {held_s / n:.0f} s a run")
    print(f"  closing {rad['closing'] / tot * 100:.0f} %  tangential {rad['tangential'] / tot * 100:.0f} %  opening {rad['opening'] / tot * 100:.0f} %  (1 s windows)")
    for st, name in ((1, 'play'), (3, 'seek'), (4, 'escape')):
        m = sum(v for k, v in by_steer.items() if k[0] == st)
        if m:
            print(f"  {name:6s} holds the heading {m / tot * 100:.0f} %: closing {by_steer[(st, 'closing')] / m * 100:.0f} %  tangential {by_steer[(st, 'tangential')] / m * 100:.0f} %  opening {by_steer[(st, 'opening')] / m * 100:.0f} %")
    if err_seek:
        err_seek.sort(); print(f"  heading error while seek steers: median {err_seek[len(err_seek) // 2]:.2f} rad, p90 {err_seek[9 * len(err_seek) // 10]:.2f}")
    for k in ('0-0.3', '0.3-0.8', '0.8-1.5', '1.5-2.5', '>2.5'):
        if k in sign:
            a, b = sign[k]; print(f"  yaw command toward the reference, |err| {k:8s}: {a / (a + b) * 100:.0f} % of {a + b} firm half-seconds")
    print(f"  attended on the walk, nearer than the held target, not taken: {A / n:.1f} episodes a run; arrival near-misses {C / n:.1f} a run; arrival stops {arrivals / n:.1f} a run")


if __name__ == '__main__':
    for g in sys.argv[1:]:
        paths = sorted(glob.glob(g))
        if not paths:
            print(f"no logs match {g}"); continue
        print(f"== {g}"); score(paths)
