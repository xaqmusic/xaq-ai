#!/usr/bin/env python3
"""The first minute (2026-10-02, the operator: "a lot of my evaluation is based on the first minute"): how each run
meets ONE object -- by default the green block that stands ~1.4 m in front of the duck when the playroom run starts
(obj_block0) -- from the start to S seconds.

From the host's FULL JSONL and the scene manifest (CLOUD_OBJECTS_MANIFEST, as cloud_objects.py), per run:
  - the closest the trunk came to the object's edge, and when;
  - the first stop that began within 0.35 m of the object's edge (when);
  - every skill (kick / peck / push) that STARTED with the object the nearest movable thing within 0.35 m: the object's
    edge distance and true bearing off the nose at the skill's start, whether the robot touched a movable thing
    during the skill window (the host's contact flag "obj"), and whether the object moved > 5 cm in the next 8 s.
Summary per glob: runs with a skill at the object within S, runs with a TOUCH of it, runs where it moved, the first
skill's median bearing / edge, and the time to the first skill at it.
Usage: CLOUD_OBJECTS_MANIFEST=... first_minute.py "glob" [...] [--secs 60] [--object obj_block0] [--per-run]
"""
import glob, json, math, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cloud_objects as co  # noqa: E402


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def med(xs):
    xs = sorted(xs)
    return xs[len(xs) // 2] if xs else float('nan')


args = sys.argv[1:]
secs, obj, per_run = 60.0, 'obj_block0', False
if '--secs' in args:
    k = args.index('--secs'); secs = float(args[k + 1]); del args[k:k + 2]
if '--object' in args:
    k = args.index('--object'); obj = args[k + 1]; del args[k:k + 2]
if '--per-run' in args:
    args.remove('--per-run'); per_run = True

half, lay, movable, _furniture = co.load_scene()


def obj_pos(r):
    """(x, y, edge) of the named object at this record, and the name of the nearest movable."""
    q = r['qpos']; best = None; target = None
    for o in movable:
        ox, oy, e = q[lay[o['name']]], q[lay[o['name']] + 1], o['extent']
        d = math.hypot(r['x'] - ox, r['y'] - oy) - e
        if best is None or d < best[0]:
            best = (d, o['name'])
        if o['name'] == obj:
            target = (ox, oy, e)
    return target, (best[1] if best else None)


for g in args:
    rows = []
    for path in sorted(glob.glob(g)):
        recs = []
        for l in open(path):
            if not l.startswith('{"t":'):
                continue
            r = json.loads(l)
            if r['t'] > secs + 10:
                break
            recs.append(r)
        closest = (9.0, None); first_stop = None; skills = []; prev = ''
        for i, r in enumerate(recs):
            if r['t'] > secs:
                break
            tgt, nearest = obj_pos(r)
            if tgt is None:
                continue
            ox, oy, e = tgt
            d = math.hypot(r['x'] - ox, r['y'] - oy) - e
            if d < closest[0]:
                closest = (d, r['t'])
            if first_stop is None and r.get('event', '').startswith('stop:') and r['event'] not in ('stop:escape', 'stop:end', 'stop:bored') and d < 0.35:
                first_stop = r['t']
            sk = r.get('skill', '')
            if sk and sk != prev and nearest == obj and d < 0.35:
                brg = math.degrees(wrap(math.atan2(oy - r['y'], ox - r['x']) - yaw_of(r['qpos'][3:7])))
                touched = False; j = i
                while j < len(recs) and recs[j].get('skill', '') == sk:
                    touched = touched or bool(recs[j].get('obj', 0)); j += 1
                k8 = i
                while k8 < len(recs) and recs[k8]['t'] < r['t'] + 8:
                    k8 += 1
                t8, _ = obj_pos(recs[min(k8, len(recs) - 1)])
                moved = t8 is not None and math.hypot(t8[0] - ox, t8[1] - oy) > 0.05
                skills.append({'t': r['t'], 'skill': sk, 'edge': d, 'brg': brg, 'touched': touched, 'moved': moved})
            prev = sk
        rows.append({'path': os.path.basename(path), 'closest': closest, 'stop': first_stop, 'skills': skills})
    n = len(rows)
    with_skill = [r for r in rows if r['skills']]
    touched = [r for r in rows if any(s['touched'] for s in r['skills'])]
    moved = [r for r in rows if any(s['moved'] for s in r['skills'])]
    firsts = [r['skills'][0] for r in with_skill]
    print(f"{g}  ({n} runs, the first {secs:.0f} s, {obj})")
    print(f"  a skill at it: {len(with_skill)}/{n} runs; touched it: {len(touched)}/{n}; moved it: {len(moved)}/{n}; "
          f"stopped at it: {sum(1 for r in rows if r['stop'] is not None)}/{n}; closest edge median {med([r['closest'][0] for r in rows]):.2f} m")
    if firsts:
        print(f"  the first skill at it: at {med([s['t'] for s in firsts]):.0f} s (median); edge {med([s['edge'] for s in firsts]):.2f} m, "
              f"|bearing| {med([abs(s['brg']) for s in firsts]):.0f} deg; touched {sum(s['touched'] for s in firsts)}/{len(firsts)}; "
              f"kinds {', '.join(sorted(set(s['skill'] for s in firsts)))}")
    if per_run:
        for r in rows:
            sk = '; '.join(f"{s['skill']}@{s['t']:.0f}s e{s['edge']:.2f} b{s['brg']:+.0f}{' T' if s['touched'] else ''}{' M' if s['moved'] else ''}" for s in r['skills'])
            print(f"    {r['path']:42s} closest {r['closest'][0]:.2f} m @ {r['closest'][1] or 0:.0f} s; stop {r['stop'] if r['stop'] is not None else '-'}; {sk or 'no skill at it'}")
