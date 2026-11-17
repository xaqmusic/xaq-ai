#!/usr/bin/env python3
"""stepper_freeze.py — find the stepper's FREEZES in body logs and say what state the machine and the legs were in (2026-11-17;
operator: "the robot can still freeze ... if the kinematics solver gets the leg into a position where a joint is at
hyperextension").

    stepper_freeze.py <log.jsonl[.gz]> ...      [MIN=250] [V=1]

A freeze is a run of engaged rows (ps_phase != 0) of MIN ticks or more in which no leg completed a step (sum of ps_steps
constant).  Per freeze: its span, where the body was (truth x, z: an instrument), the phase histogram, the stepping leg, and for
every leg over the window
  ku     the commanded knee, action units (0 = straight, < 0 = past straight, -1 = the hyperextension stop)
  stop   a commanded joint at a stop (|u| >= 0.999): h1 / h2 / k- (hyperextended) / k+ (folded)
  ik     the leg's own IK residual: |FK(command) - target| is not logged per leg; the mean over legs (ps_ik) is given
  dk     the REAL knee against the commanded one (rad, the body's joint angle minus the command): a leg that cannot follow
and the counters that moved inside it (timeouts, clips, misses, deadlock ticks, turns passed, skips, multistarts, switches).
Then a summary over all logs: freezes per minute engaged, the share of engaged time frozen, and the share of freezes with a
hyperextended command on the stepping leg / on any leg.
V=1 prints every freeze; V=0 the summary only."""
import gzip, json, sys, collections

PH = ['idle', 'shift', 'lift', 'swing', 'lower', 'settle']
LEG = ['FL', 'FR', 'RL', 'RR']


def rows(path):
    op = gzip.open if path.endswith('.gz') else open
    out = []
    try:
        with op(path, 'rt', errors='replace') as f:
            for ln in f:
                if '"fwd_v"' not in ln: continue
                i = ln.find('{')
                try: out.append(json.loads(ln[i:]))
                except Exception: pass
    except EOFError: pass   # a log still being written (gzip without its trailer): what is there is read
    return out


def stops(u):
    s = []
    if abs(u[0]) >= 0.999: s.append('h1')
    if abs(u[1]) >= 0.999: s.append('h2+' if u[1] > 0 else 'h2-')
    if u[2] <= -0.999: s.append('k-')
    if u[2] >= 0.999: s.append('k+')
    return s


def analyse(path, min_ticks, verbose):
    R = rows(path)
    if not R: return None
    eng = [r for r in R if r.get('ps_phase', 0) != 0]
    dt_row = (R[-1]['t'] - R[0]['t']) / max(1, len(R) - 1)
    # the commanded knee angle against the real one: the median offset over the run (the two are measured about the same axis)
    offs = []
    for r in R:
        ca, kn = r.get('cmd_ang'), r.get('knee')
        if ca and kn and r.get('ps_phase', 0) != 0:
            for i in range(4): offs.append(kn[i] - ca[3 * i + 2])
    offs.sort(); off = offs[len(offs) // 2] if offs else 0.0
    # THE BODY ASLEEP (2026-11-17): every real joint, the chassis height, tilt and place exactly constant for 2 s or more -- the
    # physics engine has put the body to sleep and it ignores its motors (`slp`, where the log carries it, is the engine's own count)
    sig = lambda r: (tuple(r.get('hip1', ())), tuple(r.get('hip2', ())), tuple(r.get('knee', ())), r.get('y'), r.get('tilt'), r.get('x'), r.get('z'))
    asleep = [False] * len(R); k0 = 0
    while k0 < len(R):
        k1 = k0
        while k1 + 1 < len(R) and sig(R[k1 + 1]) == sig(R[k0]): k1 += 1
        if R[k1]['t'] - R[k0]['t'] >= 100:
            for q in range(k0, k1 + 1): asleep[q] = True
        k0 = k1 + 1
    freezes = []; i = 0; n = len(R)
    while i < n:
        if R[i].get('ps_phase', 0) == 0: i += 1; continue
        s0 = sum(R[i].get('ps_steps', [0]))
        j = i
        while j + 1 < n and R[j + 1].get('ps_phase', 0) != 0 and sum(R[j + 1].get('ps_steps', [0])) == s0: j += 1
        if R[j]['t'] - R[i]['t'] >= min_ticks: freezes.append((i, j))
        i = j + 1
    out = []
    for (a, b) in freezes:
        W = R[a:b + 1]
        ph = collections.Counter(PH[min(5, w.get('ps_phase', 0))] for w in W)
        legs = collections.Counter(w.get('ps_leg', -1) for w in W)
        leg = legs.most_common(1)[0][0]
        d = lambda k, idx=None: ((W[-1].get(k, 0) if idx is None else (W[-1].get(k) or [0] * 9)[idx]) - (W[0].get(k, 0) if idx is None else (W[0].get(k) or [0] * 9)[idx]))
        per = []
        for L in range(4):
            ku = [w['ps_u'][L][2] for w in W if w.get('ps_u')]
            st = collections.Counter(s for w in W if w.get('ps_u') for s in stops(w['ps_u'][L]))
            dk = [abs(w['knee'][L] - w['cmd_ang'][3 * L + 2] - off) for w in W if w.get('cmd_ang') and w.get('knee')]
            ext = [w['ps_room'][L][0] for w in W if w.get('ps_room')]
            per.append({'ku_min': min(ku) if ku else 0.0, 'ku_med': sorted(ku)[len(ku) // 2] if ku else 0.0,
                        'hyper': sum(1 for k in ku if k < -0.02) / max(1, len(ku)), 'stop': {k: v / len(W) for k, v in st.items()},
                        'dk': sorted(dk)[len(dk) // 2] if dk else 0.0, 'dk_max': max(dk) if dk else 0.0, 'ext': min(ext) if ext else 0.0})
        zz = sum(1 for q in range(a, b + 1) if asleep[q]) / len(W)
        shift_share = ph.get('shift', 0) / len(W)
        kind = 'BODY ASLEEP' if zz > 0.5 else ('re-plan loop' if shift_share >= 0.8 and d('ps_pass') >= 3 else ('hyperextended' if any(q['hyper'] > 0.5 for q in per) else ('hang' if d('ps_hang', 0) > 0 else 'slow step')))
        rec = {'kind': kind, 'path': path, 't0': W[0]['t'], 't1': W[-1]['t'], 'dur': W[-1]['t'] - W[0]['t'], 'x': W[0].get('x'), 'z': W[0].get('z'),
               'phase': dict(ph), 'leg': leg, 'legs': dict(legs), 'per': per, 'ik': max(w.get('ps_ik', 0) for w in W),
               'tilt': max(w.get('tilt', 0) for w in W),
               'd': {'tmo': d('ps_tmo'), 'clips': d('ps_clips'), 'miss': d('ps_miss'), 'dead': d('ps_dead'), 'pass': d('ps_pass'), 'plans': d('ps_plan', 0),
                     'ms': d('ps_ms', 0), 'sw': d('ps_ms', 1), 'hang': d('ps_hang', 0), 'obst': d('ps_obst'), 'blk': d('ps_blk'), 'sunk': d('ps_sunk'), 'ret': d('ps_ret')},
               'end': 'log end' if b == n - 1 else ('idle' if R[b + 1].get('ps_phase', 0) == 0 else 'step')}
        out.append(rec)
        if verbose:
            print(f"  freeze [{rec['kind']}] t {rec['t0']}-{rec['t1']} ({rec['dur'] * 0.02:.0f} s) at ({rec['x']:.2f}, {rec['z']:.2f}) ends by {rec['end']}; phases {rec['phase']}; leg {LEG[leg] if 0 <= leg < 4 else leg} {rec['legs']}; ik max {rec['ik'] * 1000:.0f} mm; tilt max {rec['tilt']:.2f}")
            print('     moved: ' + ' '.join(f'{k} {v:+d}' for k, v in rec['d'].items() if v))
            for L in range(4):
                p = per[L]
                print(f"     {LEG[L]}{'*' if L == leg else ' '} knee u min {p['ku_min']:+.2f} med {p['ku_med']:+.2f} past-straight {100 * p['hyper']:3.0f} %  ext slack min {1000 * p['ext']:4.0f} mm  real-cmd knee med {p['dk']:.2f} max {p['dk_max']:.2f} rad  stops {({k: round(v, 2) for k, v in p['stop'].items()}) or ''}")
    eng_ticks = len(eng) * dt_row
    # the step intervals (ticks between landings while engaged), and the stalls: 10 s windows in which the body (truth x, z) moved < 1 cm
    gaps = []; last = None
    for k in range(1, n):
        if R[k].get('ps_phase', 0) == 0 or R[k - 1].get('ps_phase', 0) == 0: last = None; continue
        if sum(R[k].get('ps_steps', [0])) != sum(R[k - 1].get('ps_steps', [0])):
            if last is not None: gaps.append(R[k]['t'] - last)
            last = R[k]['t']
    gaps.sort(); w = max(1, int(round(500 / dt_row))); still = 0; tot_w = 0
    for k in range(w, n):
        if any(R[q].get('ps_phase', 0) == 0 for q in (k - w, k)): continue
        tot_w += 1
        if ((R[k]['x'] - R[k - w]['x']) ** 2 + (R[k]['z'] - R[k - w]['z']) ** 2) ** 0.5 < 0.01: still += 1
    leg_rows = sum(1 for r in eng if r.get('ps_u') for u in r['ps_u'] if u[2] < -0.02)
    return {'path': path, 'eng_ticks': eng_ticks, 'freezes': out, 'rows': len(R), 'steps': sum(R[-1].get('ps_steps', [0])), 'resets': R[-1].get('auto_reset_count', 0),
            'tilt_off': R[-1].get('ps_tilt_off', 0), 'off': off, 'x1': R[-1].get('x'), 'z1': R[-1].get('z'), 'x0': R[min(len(R) - 1, 20)].get('x'), 'z0': R[min(len(R) - 1, 20)].get('z'),
            'hyper_rows': sum(1 for r in eng if r.get('ps_u') and min(u[2] for u in r['ps_u']) < -0.02) / max(1, len(eng)),
            'kstop_rows': sum(1 for r in eng if r.get('ps_u') and min(u[2] for u in r['ps_u']) <= -0.999) / max(1, len(eng)),
            'asleep': sum(1 for q in range(len(R)) if asleep[q] and R[q].get('ps_phase', 0) != 0) / max(1, len(eng)), 'asleep_t': next((R[q]['t'] for q in range(len(R)) if asleep[q]), None),
            'steps_asleep': sum(1 for q in range(1, len(R)) if asleep[q] and sum(R[q].get('ps_steps', [0])) != sum(R[q - 1].get('ps_steps', [0]))), 'slp': max((r.get('slp', 0) for r in R), default=0),
            'gap_med': gaps[len(gaps) // 2] if gaps else 0, 'gap_p90': gaps[int(0.9 * len(gaps))] if gaps else 0, 'gap_max': gaps[-1] if gaps else 0, 'gaps': gaps,
            'still': still / max(1, tot_w), 'kg': R[-1].get('ps_kg'), 'miss': R[-1].get('ps_miss', 0), 'tmo': R[-1].get('ps_tmo', 0), 'sunk': R[-1].get('ps_sunk', 0),
            'ms': (R[-1].get('ps_ms') or [0, 0]), 'hang': (R[-1].get('ps_hang') or [0])[0], 'dk_big': sum(1 for r in eng if r.get('cmd_ang') and r.get('knee') and max(abs(r['knee'][i] - r['cmd_ang'][3 * i + 2] - off) for i in range(4)) > 0.3) / max(1, len(eng)),
            'ik': sum(r.get('ps_ik', 0) for r in eng) / max(1, len(eng)), 'tilt_max': max(r.get('tilt', 0) for r in R), 'path': sum(((R[k]['x'] - R[k - 1]['x']) ** 2 + (R[k]['z'] - R[k - 1]['z']) ** 2) ** 0.5 for k in range(1, n) if R[k].get('ps_phase', 0) != 0 and R[k - 1].get('ps_phase', 0) != 0)}


def main(argv):
    min_ticks = 250; verbose = 1; paths = []
    for a in argv:
        if a.startswith('MIN='): min_ticks = int(a[4:])
        elif a.startswith('V='): verbose = int(a[2:])
        else: paths.append(a)
    if not paths: sys.exit(__doc__)
    tot_f = 0; tot_eng = 0.0; tot_frozen = 0.0; hyp_leg = 0; hyp_any = 0; stop_any = 0; steps = 0; kinds = collections.Counter(); n_asleep = 0; asleep_time = 0.0; moved = []
    for p in paths:
        if verbose: print(f'== {p}')
        r = analyse(p, min_ticks, verbose)
        if r is None: print(f'== {p}: no rows'); continue
        fz = r['freezes']; frozen = sum(f['dur'] for f in fz)
        print(f"{p.split('/')[-1]:28s} engaged {r['eng_ticks'] * 0.02 / 60:4.1f} min  steps {r['steps']:4d}  freezes {len(fz):2d}  frozen {100 * frozen / max(1, r['eng_ticks']):3.0f} %  longest {max([f['dur'] for f in fz] or [0]) * 0.02:4.0f} s"
              f"  past-straight rows {100 * r['hyper_rows']:3.0f} %  at the stop {100 * r['kstop_rows']:3.0f} %  resets {r['resets']}  tilt-off {r['tilt_off']}  moved {((r['x1'] - r['x0']) ** 2 + (r['z1'] - r['z0']) ** 2) ** 0.5:.2f} m")
        if r['asleep'] > 0: print(f"{'':28s} BODY ASLEEP from t {r['asleep_t']}: {100 * r['asleep']:.0f} % of engaged time, {r['steps_asleep']} 'steps' counted over a motionless body" + (f" (engine: up to {r['slp']} segments asleep)" if r['slp'] else ''))
        print(f"{'':28s} step interval s: median {r['gap_med'] * 0.02:.1f} p90 {r['gap_p90'] * 0.02:.1f} max {r['gap_max'] * 0.02:.1f}  still (10 s, < 1 cm) {100 * r['still']:3.0f} %  a real knee > 0.3 rad off its command {100 * r['dk_big']:4.1f} % of rows"
              f"  misses {r['miss']} timeouts {r['tmo']} sunk {r['sunk']} hangs {r['hang']} multistarts {r['ms'][0]} switches {r['ms'][1]}  ik {1000 * r['ik']:.1f} mm  tilt max {r['tilt_max']:.2f}  path {r['path']:.2f} m" + (f"  knee guard [retries, unsolved] {r['kg']}" if r['kg'] else ''))
        tot_f += len(fz); tot_eng += r['eng_ticks']; tot_frozen += frozen; steps += r['steps']
        n_asleep += 1 if r['asleep'] > 0 else 0; asleep_time += r['asleep'] * r['eng_ticks']; moved.append(((r['x1'] - r['x0']) ** 2 + (r['z1'] - r['z0']) ** 2) ** 0.5)
        for f in fz:
            kinds[f['kind']] += 1
            L = f['leg']
            if 0 <= L < 4 and f['per'][L]['hyper'] > 0.5: hyp_leg += 1
            if any(q['hyper'] > 0.5 for q in f['per']): hyp_any += 1
            if any(q['stop'] and max(q['stop'].values()) > 0.5 for q in f['per']): stop_any += 1
    print(f"TOTAL: {tot_f} freezes (>= {min_ticks * 0.02:.0f} s without a step) in {tot_eng * 0.02 / 60:.1f} engaged min = {tot_f / max(1e-9, tot_eng * 0.02 / 60):.2f} /min; frozen {100 * tot_frozen / max(1, tot_eng):.0f} % of engaged time; steps {steps}")
    print(f"       by kind: {dict(kinds)}; runs in which the body fell asleep: {n_asleep} of {len(paths)} ({100 * asleep_time / max(1, tot_eng):.0f} % of engaged time); net distance per run: mean {sum(moved) / max(1, len(moved)):.2f} m")
    print(f"       the stepping leg commanded past straight for most of the freeze: {hyp_leg} of {tot_f}; any leg: {hyp_any}; any leg with a joint at a stop for most of it: {stop_any}")


if __name__ == '__main__':
    main(sys.argv[1:])
