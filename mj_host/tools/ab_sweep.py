#!/usr/bin/env python3
"""Seed-averaged A/B for mj_host brain configs — the duck's seedavg.

Usage:
  python3 mj_host/tools/ab_sweep.py CONFIG.json [CONFIG2.json ...] [--seeds N] [--secs S]

Runs each config over N seeds under the A2 recovery harness and reports, per arm:
  rescues/min   — falls (the A2 headline; lower is better)
  brain%        — share of the run the brain drove
  tilt(brain)   — mean tilt over brain-driven ticks      (anti-blind complement)
  upright15     — fraction of brain ticks with tilt < 15° (the G2 "standing" bar;
                  a sub-trigger crouch scores rescues/min but not this)
  mean|u|, TLE up/down, and the modules' own parameter read-backs — so a
  silent-confound arm cannot happen quietly (§3.2 rule 7).

Every number is stderr/stdout the host already emits; this only aggregates.
"""
import argparse
import json
import re
import statistics
import subprocess
import sys

HOST = 'mj_host/build/ogma_mjhost'


def run_one(cfg, seed, secs):
    p = subprocess.run(
        [HOST, '--brain', '--graph', cfg, '--secs', str(secs), '--seed', str(seed)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3600)
    err = p.stderr
    out = {}
    m = re.search(r'(\d+) rescues, (\d+)% of the run driven by the brain', err)
    out['rescues_min'] = int(m.group(1)) * 60.0 / secs
    out['brain_pct'] = float(m.group(2))
    out['mean_u'] = float(re.search(r'mean \|action\| ([\d.]+)', err).group(1))
    m = re.search(r'motor_tle upright ([\d.]+) \| down ([\d.]+)', err)
    out['tle_up'], out['tle_down'] = float(m.group(1)), float(m.group(2))
    out['down_quieter'] = 'DOWN IS THE QUIETER STATE' in err
    out['readback'] = ' ; '.join(
        re.sub(r'\{[^}]*\}', '', l).strip()
        for l in err.splitlines() if 'motor_tle' in l and 'motor_epm' in l)
    tilts = []
    for line in p.stdout.splitlines():
        if not line.startswith('{'):
            continue
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if row.get('drive') == 'brain':
            tilts.append(row['tilt'])
    out['tilt_mean'] = statistics.mean(tilts) if tilts else float('nan')
    out['upright_frac'] = (sum(1 for t in tilts if t < 15.0) / len(tilts)) if tilts else 0.0
    return out


def fmt(vals):
    return f'{statistics.mean(vals):6.2f}±{statistics.stdev(vals):5.2f}' if len(vals) > 1 \
        else f'{vals[0]:6.2f}      '


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('configs', nargs='+')
    ap.add_argument('--seeds', type=int, default=6)
    ap.add_argument('--secs', type=int, default=240)
    args = ap.parse_args()

    results = {}
    for cfg in args.configs:
        rows = []
        for seed in range(1, args.seeds + 1):
            r = run_one(cfg, seed, args.secs)
            rows.append(r)
            print(f'  {cfg.split("/")[-1]:32s} seed {seed}: {r["rescues_min"]:5.1f} resc/min  '
                  f'brain {r["brain_pct"]:3.0f}%  tilt {r["tilt_mean"]:5.1f}  '
                  f'up15 {r["upright_frac"]:.2f}'
                  + ('  !!DOWN-QUIETER' if r['down_quieter'] else ''), file=sys.stderr)
        results[cfg] = rows

    print(f'\n=== A/B, {args.seeds} seeds x {args.secs} s ===')
    print(f'{"arm":32s} {"resc/min":>13s} {"brain%":>13s} {"tilt(brain)":>13s} '
          f'{"upright15":>11s} {"mean|u|":>12s} {"tle_up":>12s}')
    for cfg, rows in results.items():
        print(f'{cfg.split("/")[-1]:32s} '
              f'{fmt([r["rescues_min"] for r in rows]):>13s} '
              f'{fmt([r["brain_pct"] for r in rows]):>13s} '
              f'{fmt([r["tilt_mean"] for r in rows]):>13s} '
              f'{fmt([r["upright_frac"] for r in rows]):>11s} '
              f'{fmt([r["mean_u"] for r in rows]):>12s} '
              f'{fmt([r["tle_up"] for r in rows]):>12s}')
    print('\nread-backs (first seed):')
    for cfg, rows in results.items():
        print(f'  {cfg.split("/")[-1]:32s} {rows[0]["readback"]}')
    dq = [(c.split('/')[-1], i + 1) for c, rows in results.items()
          for i, r in enumerate(rows) if r['down_quieter']]
    print(f'down-quieter flags: {dq if dq else "none"}')


if __name__ == '__main__':
    main()
