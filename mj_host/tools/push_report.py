#!/usr/bin/env python3
"""Read the (d) push test out of an `ogma_mjhost --brain ... --push` JSONL.

    python3 mj_host/tools/push_report.py run.jsonl [more.jsonl ...]
    python3 mj_host/tools/push_report.py --timeline run.jsonl

Per run: brain-driven share, falls (harness handoffs), the fraction of ticks under 15°
of tilt, mean per-tick joint motion |dq|/14 in mrad over brain-driven ticks (the
consolidated stance sits near 0.08), pose distance from the STAND keyframe, and the
final consolidation of each MotorEPM.

Per shove, one row:
  t       when it landed (s)
  N       force magnitude
  body°   direction in the body frame (0 = +x forward, 90 = +y), from the trunk quaternion
  peak°   worst tilt inside the judging window (4 s, or until the next shove)
  rec s   time until tilt was back under 5° and stayed there 0.4 s (nan = never)
  resc    YES if the recovery harness handed the body to the scaffold inside the window
  step    YES if the step hand-off fired inside the window (--step-lean)
  c_bef   consolidation of each MotorEPM the tick before the shove
  c_min   its minimum between this shove and the next
  c_ok s  seconds until every MotorEPM was back at >= 0.95 after having dropped
          (0.0 = never dropped; nan = had not recovered by the next shove)
  dq_bef  |dq| mrad/tick over the 5 s before the shove
  dq_aft  |dq| mrad/tick over the last 5 s before the next shove — did stillness return?
  lrn%    share of ticks with learning enabled between this shove and the next

--timeline prints one line per run: every shove, harness fall, consolidation collapse
(below 0.9) and re-earning (back above 0.95), with the second it happened.
"""
import argparse
import json
import math

# The STAND keyframe (DuckBody.cpp kHomePose); pose distance is mean |q - home| / 14.
HOME = [0.0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0.0, 0.0,
        0.0, 0.0873, 0.4579, 0.0049, -0.4530]
HZ = 50.0
RECOVERED_DEG, RECOVER_HOLD_S, WINDOW_S = 5.0, 0.4, 4.0


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith('{'):
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return rows


def yaw_of(qpos):
    qw, qx, qy, qz = qpos[3:7]
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def dq_mrad(rows, a, b):
    s, n = 0.0, 0
    for i in range(max(a, 1), min(b, len(rows))):
        if rows[i]['drive'] != 'brain' or rows[i - 1]['drive'] != 'brain':
            continue
        s += sum(abs(x - y) for x, y in zip(rows[i - 1]['q'], rows[i]['q'])) / 14.0
        n += 1
    return 1000.0 * s / n if n else float('nan')


def pose_dist(rows, a, b):
    s, n = 0.0, 0
    for i in range(a, min(b, len(rows))):
        if rows[i]['drive'] != 'brain':
            continue
        s += sum(abs(x - h) for x, h in zip(rows[i]['q'], HOME)) / 14.0
        n += 1
    return s / n if n else float('nan')


def shove_starts(rows):
    return [i for i in range(1, len(rows)) if any(rows[i]['push']) and not any(rows[i - 1]['push'])]


def report(path, quiet=5.0, cons_ok=0.95):
    rows = load(path)
    n = len(rows)
    if not n:
        print(f"# {path}: no JSONL rows")
        return
    brain = sum(1 for r in rows if r['drive'] == 'brain')
    falls = sum(1 for r in rows if r.get('event') == 'reset:handoff')
    up15 = sum(1 for r in rows if r['tilt'] < 15) / n
    print(f"# {path}")
    steps = sum(1 for r in rows if r.get('event') == 'step:start')
    print(f"ticks {n} ({n / HZ:.0f} s)  brain-driven {100 * brain / n:.1f}%  falls {falls}  steps {steps}  "
          f"up15 {up15:.3f}  |dq| {dq_mrad(rows, 0, n):.3f} mrad/tick  "
          f"pose {pose_dist(rows, 0, n):.3f}  cons_end {rows[-1].get('cons')}")
    starts = shove_starts(rows)
    if not starts:
        return
    W, Q, hold = int(WINDOW_S * HZ), int(quiet * HZ), int(RECOVER_HOLD_S * HZ)
    print(f"{'#':>2} {'t':>7} {'N':>5} {'body°':>6} {'peak°':>6} {'rec s':>6} {'resc':>4} {'step':>4} "
          f"{'c_bef':>11} {'c_min':>11} {'c_ok s':>7} {'dq_bef':>7} {'dq_aft':>7} {'lrn%':>5}")
    for k, i in enumerate(starts):
        nxt = starts[k + 1] if k + 1 < len(starts) else n
        until = min(n, i + W, nxt)
        p = rows[i]['push']
        mag = math.hypot(p[0], p[1])
        yaw = yaw_of(rows[i]['qpos'])
        bx = math.cos(yaw) * p[0] + math.sin(yaw) * p[1]
        by = -math.sin(yaw) * p[0] + math.cos(yaw) * p[1]
        peak = max(r['tilt'] for r in rows[i:until])
        rec = float('nan')
        for j in range(i, until - hold + 1):
            if all(rows[m]['tilt'] < RECOVERED_DEG for m in range(j, j + hold)):
                rec = (j + hold - i) / HZ
                break
        rescued = any(rows[m].get('event') == 'reset:handoff' for m in range(i, until))
        stepped = any(rows[m].get('event') == 'step:start' for m in range(i, until))
        cb = rows[i - 1].get('cons', [])
        seg = rows[i:nxt]
        cmin = [min(r['cons'][c] for r in seg) for c in range(len(cb))] if cb else []
        c_ok = float('nan')
        if cb and any(m < cons_ok for m in cmin):
            dropped = False
            for j, r in enumerate(seg):
                if min(r['cons']) < cons_ok:
                    dropped = True
                elif dropped:
                    c_ok = j / HZ
                    break
        elif cb:
            c_ok = 0.0
        dqb = dq_mrad(rows, max(0, i - Q), i)
        dqa = dq_mrad(rows, max(i, nxt - Q), nxt)
        lrn = 100 * sum(1 for r in seg if r['learning']) / len(seg)
        fmt = lambda v: ' '.join(f"{x:.2f}" for x in v) if v else '-'
        print(f"{k + 1:>2} {rows[i]['t']:>7.1f} {mag:>5.1f} {math.degrees(math.atan2(by, bx)):>6.0f} "
              f"{peak:>6.1f} {rec:>6.2f} {'YES' if rescued else '-':>4} {'YES' if stepped else '-':>4} {fmt(cb):>11} {fmt(cmin):>11} "
              f"{c_ok:>7.1f} {dqb:>7.3f} {dqa:>7.3f} {lrn:>5.0f}")


def timeline(path):
    rows = load(path)
    ev, prev_c, prev_push = [], 1.0, False
    for r in rows:
        pushing = any(r['push'])
        if pushing and not prev_push:
            ev.append((r['t'], 'PUSH'))
        prev_push = pushing
        if r.get('event') == 'reset:handoff':
            ev.append((r['t'], 'fall'))
        c = min(r['cons']) if r.get('cons') else 1.0
        if c < 0.9 <= prev_c:
            ev.append((r['t'], f'c↓{c:.2f}'))
        if c >= 0.95 > prev_c:
            ev.append((r['t'], 'c↑'))
        prev_c = c
    print(path.split('/')[-1], ' '.join(f"{t:.0f}:{e}" for t, e in ev))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+')
    ap.add_argument('--timeline', action='store_true', help='one event line per run instead of the table')
    ap.add_argument('--quiet', type=float, default=5.0, help='seconds of |dq| before/after each shove')
    args = ap.parse_args()
    for f in args.files:
        if args.timeline:
            timeline(f)
        else:
            report(f, args.quiet)
            print()
