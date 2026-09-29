#!/usr/bin/env python3
"""yield_where -- where do the chase's yields fire? (sweep 13, 2026-09-29)
  python3 mj_host/tools/yield_where.py 'mj_host/log/campaign/c13/a1v2_r107_yield_s*.jsonl'
  Every chase start in the 'chase' record; the target in the world through the body's pose;
its distance to the nearest wall, the body, the table/chairs/shelf (tall furniture), the train, the small things."""
import json, math, sys, glob
from collections import Counter
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from mover_tracks import quat_yaw, load_manifest
REPO = Path(__file__).resolve().parents[2]
m, layout, objs = load_manifest(str(REPO / 'mj_host/models/microduck/scene_playroom_train.xml'))
FURN = {'table': (1.257, -1.359, 0.48), 'chair0': (-1.338, -1.551, 0.22), 'chair1': (-1.254, 0.401, 0.22), 'shelf': (1.845, -0.798, 0.42)}
def furn_d(wx, wy):
    return min(max(0.0, math.hypot(wx - x, wy - y) - r) for x, y, r in FURN.values())
rows = []; ends = Counter()
for path in sorted(glob.glob(sys.argv[1])):
    prev = 0; start = None
    for line in open(path):
        if '"chase"' not in line: continue
        r = json.loads(line); ch = r['chase']; chasing = int(ch[0]) == 1
        if chasing and not prev:
            ox, oy, oyaw = r['odom']; q = r['qpos']; byaw = quat_yaw(q[3], q[4], q[5], q[6])
            dx, dy = ch[2] - ox, ch[3] - oy; dth = byaw - oyaw
            wx = r['x'] + math.cos(dth) * dx - math.sin(dth) * dy; wy = r['y'] + math.sin(dth) * dx + math.cos(dth) * dy
            tr = r['train']; d_train = math.hypot(wx - tr[0], wy - tr[1])
            d_obj = min(math.hypot(wx - q[layout[o][0]], wy - q[layout[o][0] + 1]) for o in objs)
            start = dict(t=r['t'], wall=2.0 - max(abs(wx), abs(wy)), body=math.hypot(wx - r['x'], wy - r['y']), furn=furn_d(wx, wy),
                         train=d_train, moving=int(tr[5]), obj=d_obj, coast=int(ch[10]), wx=wx, wy=wy, walking=r['drive'] != 'stand')
        elif not chasing and prev and start is not None:
            start['len'] = round(r['t'] - start['t'], 2); rows.append(start); start = None
        prev = chasing
short = [x for x in rows if x['len'] <= 0.06]
print(f"{len(rows)} chases, {len(short)} ended within a tick (the yields)")
def lab(x):
    if x['train'] < 0.22: return 'train' + ('+' if x['moving'] else '0')
    if x['obj'] < 0.22: return 'obj'
    return 'static'
for name, sel in (('YIELDS', short), ('kept', [x for x in rows if x['len'] > 0.06])):
    if not sel: continue
    print(f"-- {name}: n={len(sel)}  by label {dict(Counter(lab(x) for x in sel))}  walking {sum(x['walking'] for x in sel)}")
    for k in ('wall', 'body', 'furn', 'train', 'obj'):
        v = sorted(x[k] for x in sel); print(f"   d {k:6s} p10 {v[len(v)//10]:.2f} p50 {v[len(v)//2]:.2f} p90 {v[9*len(v)//10]:.2f}")
    near = sum(1 for x in sel if min(x['wall'], x['furn']) <= 0.4); print(f"   within 0.4 m of a wall or furniture: {near}/{len(sel)}")
    for x in sel[:12]:
        print(f"   t {x['t']:6.1f} {lab(x):6s} wall {x['wall']:.2f} furn {x['furn']:.2f} body {x['body']:.2f} len {x['len']:.2f} at ({x['wx']:.2f},{x['wy']:.2f})")
