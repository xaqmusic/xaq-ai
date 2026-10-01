#!/usr/bin/env python3
"""The walk's turning: what the walker commands when the reference is far off the nose, what the body does, and how
much of a run is spent orbiting a target it does not reach (2026-10-01, the operator: "the very wide turning radius").

From the host's JSONL, walking ticks (no stop phase) after 30 s:
  err   the heading error, reference minus heading ("hdg"), wrapped (rad)
  cmd   the walker's twist command ("twist": vx m/s, vy m/s, vyaw rad/s)
  body  the sensed velocity ("sensed" x (0.4, 0.3, 1.0): vx m/s, vy m/s, wz rad/s), a 0.5 s box
Reported per glob:
  - by |err| band: the commanded vx and |vyaw|, the sensed vx and |wz|, the turning radius |vx| / |wz| (median), and the
    share of ticks with the forward command above 0.3 m/s (still driving forward while the target is off the nose);
  - the body's turn capacity: sensed |wz| when |vyaw| commanded > 0.8, by the commanded vx (in place / slow / fast);
  - ORBITING: seconds a run inside 5 s windows where the seek loop steers, the range left is 0.25-1.0 m and changes by
    under 0.1 m, while the heading error stays over 0.5 rad -- circling a target at a fixed distance.
Usage: turning_readout.py "glob" [...]
"""
import glob, json, math, sys

BOX = 25


def wrap(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def med(xs):
    xs = sorted(xs)
    return xs[len(xs) // 2] if xs else float('nan')


for g in sys.argv[1:]:
    bands = {(0.0, 0.5): [], (0.5, 1.0): [], (1.0, 2.0): [], (2.0, 4.0): []}
    cap = {'in place (vx<0.08)': [], 'slow (0.08-0.25)': [], 'fast (>0.25)': []}
    orbit_s = 0.0; runs = 0
    for path in sorted(glob.glob(g)):
        runs += 1
        seg = []
        def flush():
            global orbit_s
            # orbiting windows over the segment
            n = len(seg); i = 0
            while i + 250 <= n:
                w = seg[i:i + 250]
                if all(s['steer'] == 3 for s in w) and all(s['rng'] is not None and 0.25 <= s['rng'] <= 1.0 for s in w) \
                        and max(s['rng'] for s in w) - min(s['rng'] for s in w) < 0.1 and all(abs(s['err']) > 0.5 for s in w):
                    orbit_s += 5.0; i += 250
                else:
                    i += 25
        for line in open(path):
            if not line.startswith('{'):
                continue
            r = json.loads(line)
            if r['t'] < 30 or r['drive'] != 'walk' or r.get('stop', 0) != 0 or 'hdg' not in r:
                flush(); seg = []; continue
            seg.append({'err': wrap(r['hdg'][1] - r['hdg'][0]), 'cvx': r['twist'][0], 'cyaw': r['twist'][2],
                        'svx': r['sensed'][0] * 0.4, 'swz': r['sensed'][2] * 1.0, 'steer': r.get('steer'),
                        'rng': r['seek'][1] if 'seek' in r and r['seek'][0] > 0 else None})
            if len(seg) >= BOX:
                b = seg[-BOX:]
                vx = sum(s['svx'] for s in b) / BOX; wz = sum(s['swz'] for s in b) / BOX
                e = abs(seg[-1]['err']); s0 = seg[-1]
                for k in bands:
                    if k[0] <= e < k[1]:
                        bands[k].append((s0['cvx'], abs(s0['cyaw']), vx, abs(wz), abs(vx) / max(abs(wz), 1e-3)))
                if abs(s0['cyaw']) > 0.8:
                    key = 'in place (vx<0.08)' if s0['cvx'] < 0.08 else ('slow (0.08-0.25)' if s0['cvx'] < 0.25 else 'fast (>0.25)')
                    cap[key].append(abs(wz))
        flush()
    print(f"{g}  ({runs} runs)")
    for k, v in bands.items():
        if not v: continue
        print(f"  |err| {k[0]:.1f}-{k[1]:.1f} rad ({len(v) * 0.02 / runs:4.0f} s a run): cmd vx {med([x[0] for x in v]):.2f} m/s, |vyaw| {med([x[1] for x in v]):.2f}; "
              f"body vx {med([x[2] for x in v]):.2f}, |wz| {med([x[3] for x in v]):.2f} rad/s; radius {med([x[4] for x in v]):.2f} m; "
              f"forward cmd > 0.3 on {100 * sum(1 for x in v if x[0] > 0.3) / len(v):.0f} %")
    print("  the body's turn at |vyaw| cmd > 0.8: " + "  ".join(f"{k}: |wz| {med(v):.2f} rad/s ({len(v) * 0.02 / runs:.0f} s)" for k, v in cap.items() if v))
    print(f"  ORBITING (5 s at 0.25-1.0 m from the target, the range still, the error > 0.5 rad): {orbit_s / runs:.0f} s a run")
