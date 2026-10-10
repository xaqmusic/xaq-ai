#!/usr/bin/env python3
"""Where is the thing when a skill fires? (2026-10-01, the operator: the linear yaw approaches well but kicks and pecks
less accurately.)  Pollen's kick and peck fire straight ahead, so a reach depends on the thing being near AND on the
nose at the moment the skill starts.

From the host's FULL JSONL and the scene manifest (CLOUD_OBJECTS_MANIFEST, as cloud_objects.py), at every skill's
first tick: the nearest movable thing's edge distance and its TRUE bearing off the body's nose, and whether that
thing moved more than 5 cm in the next 8 s.  Per glob and skill kind (kick / peck / push): the medians, the share
started with the thing within 0.15 m and 30 deg, and the share that moved their thing.  Plus, at every arrival stop,
the body's yaw rate in the half second before it began (a body still turning when the stop catches it).
Usage: CLOUD_OBJECTS_MANIFEST=mj_host/models/microduck/scene_playroom_train.manifest.json skill_align.py "glob" [...]
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


half, lay, movable, _furniture = co.load_scene()
for g in sys.argv[1:]:
    res = {'kick': [], 'peck': [], 'push': []}; arr = []
    for path in sorted(glob.glob(g)):
        recs = [json.loads(l) for l in open(path) if l.startswith('{"t":')]
        prev = ''
        for i, r in enumerate(recs):
            sk = r.get('skill', '')
            if sk and sk != prev:
                d, kind, ox, oy = min((math.hypot(r['x'] - ox, r['y'] - oy) - e, k, ox, oy) for k, ox, oy, e in co.objects_at(r, lay, movable))
                brg = math.degrees(wrap(math.atan2(oy - r['y'], ox - r['x']) - yaw_of(r['qpos'][3:7])))
                j = i
                while j < len(recs) and recs[j]['t'] < r['t'] + 8:
                    j += 1
                later = [(a, b) for k2, a, b, _ in co.objects_at(recs[min(j, len(recs) - 1)], lay, movable) if k2 == kind and math.hypot(a - ox, b - oy) < 1.5]
                moved = (min(math.hypot(a - ox, b - oy) for a, b in later) > 0.05) if later else False
                key = 'kick' if sk.startswith('kick') else sk
                if key in res:
                    res[key].append((d, abs(brg), moved))
            prev = sk
            if r.get('event') == 'stop:arrive' and i > 25:
                arr.append(abs(wrap(yaw_of(r['qpos'][3:7]) - yaw_of(recs[i - 25]['qpos'][3:7]))) / 0.5)
    print(f"{g}")
    for k, v in res.items():
        if not v:
            continue
        near = [x for x in v if x[0] < 0.15 and x[1] < 30]
        print(f"  {k:5s} n {len(v):3d}: edge {med([x[0] for x in v]):.2f} m, |bearing| {med([x[1] for x in v]):.0f} deg; "
              f"within 0.15 m and 30 deg {100 * len(near) / len(v):.0f} %; moved their thing {sum(1 for x in v if x[2])} ({100 * sum(1 for x in v if x[2]) / len(v):.0f} %)")
    if arr:
        print(f"  arrival stops {len(arr)}: the body's yaw rate in the 0.5 s before, median {med(arr):.2f} rad/s")
