#!/usr/bin/env python3
"""stepper_outcomes.py — the step-outcome stream (learned stepper plan Phase 2) read back from a battery's logs.

    stepper_outcomes.py <dir> [bench prefix ...]        default prefixes: L42 b38 b64 cor

Each landing's record (ps_out, the PoseSequencer diag's StepOutcome: [seq, leg, kind, touched, from x, from z, land x, land z,
h_pred, h_touch, ticks, hang, moves x 7 rules (x, z)]) is recovered once by its sequence number. Per bench: steps recovered against
the step counters (a gap means a record was overwritten between two logged rows), the share planned / touched / hung, the
landing's offset from where the step set out and the closure of the rule moves against it, the ground error h_touch - h_pred, the
step duration, and the room at the end (ps_room: per leg ext slack, knee to straight, knee to fold, hip1, hip2)."""
import collections, glob, json, math, os, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _logio import open_log

RULES = ['reach', 'sector', 'edge', 'obstacle', 'retreat', 'ik', 'clip']


def read(f):
    recs = {}; last = None
    for l in open_log(f):
        if not l.startswith('{') or '"fwd_v"' not in l: continue
        try: r = json.loads(l)
        except Exception: continue
        last = r
        o = r.get('ps_out')
        if o and o[0] not in recs: recs[o[0]] = o
    return recs, last


def main(argv):
    d = argv[0].rstrip('/'); benches = argv[1:] or ['L42', 'b38', 'b64', 'cor']
    for b in benches:
        fs = sorted(glob.glob(f'{d}/*_{b}*.jsonl.gz') + glob.glob(f'{d}/*_{b}*.jsonl'))
        if not fs: continue
        R = []; steps = 0; gaps = 0; rooms = []
        for f in fs:
            recs, last = read(f)
            n = sum(last['ps_steps']) if last else 0; steps += n
            seqs = sorted(recs); gaps += (seqs[-1] - len(seqs)) if seqs else 0
            R += list(recs.values())
            if last and 'ps_room' in last: rooms.append(last['ps_room'])
        if not R: print(f'== {b}: no ps_out in these logs'); continue
        plan = [o for o in R if o[2] == 1]
        off = [math.hypot(o[6] - o[4], o[7] - o[5]) for o in plan]
        clo = [math.hypot(o[6] - o[4] - sum(o[12 + 2 * k] for k in range(7)), o[7] - o[5] - sum(o[13 + 2 * k] for k in range(7))) for o in plan]
        herr = [o[9] - o[8] for o in R if o[3] == 1]
        mv = [sum(math.hypot(o[12 + 2 * k], o[13 + 2 * k]) for o in plan) / max(1, len(plan)) for k in range(7)]
        print(f'== {b} ({len(fs)} runs): {len(R)} records for {steps} landings (gaps {gaps}); planned {len(plan)} ({100 * len(plan) / len(R):.0f}%),'
              f' touched {sum(o[3] for o in R)}, hung {sum(o[11] for o in R)}')
        if plan: print(f'    planned offset mean {100 * st.mean(off):.2f} cm, > 2 cm {sum(x > 0.02 for x in off)}; closure max {max(clo):.5f} m;'
                       f' moves (mean cm): ' + ', '.join(f'{RULES[k]} {100 * mv[k]:.2f}' for k in range(7) if mv[k] > 0))
        if herr: print(f'    ground error (touch - predicted): mean {100 * st.mean(herr):+.2f} cm, rms {100 * math.sqrt(st.mean(x * x for x in herr)):.2f} cm;'
                       f' step {st.mean(o[10] for o in R if o[10] > 0) / 50:.2f} s')
        if rooms:
            m = [[st.mean(rr[leg][k] for rr in rooms) for k in range(5)] for leg in range(4)]
            print('    room at the end (ext m / knee-straight / knee-fold / hip1 / hip2): ' + ' | '.join('/'.join(f'{x:.2f}' for x in m[leg]) for leg in range(4)))


if __name__ == '__main__':
    main(sys.argv[1:])
