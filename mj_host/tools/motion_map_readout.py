#!/usr/bin/env python3
"""motion_map_readout — does the motion map learn where movers pass? (the stalk's ambush, design doc §17.113 A0, 2026-10-09)

From the host's FULL JSONL (`mvp`: a confirmed mover sighting [x/2, y/2 in odometry, the motion map's winner, its nodes];
`train`, `odom`, `qpos`): each sighting put into the world through the odometry's own pose at that tick, then
  sighting error   its distance to the train's true position at that tick
  nodes            per run, the motion map's nodes (winners with >= 5 sightings): their world centroid, its distance to the
                   train's PATH (the nearest train position over the whole run), and their spread over the run (the
                   odometry's drift shows here: a node's sightings in world coordinates wander as the dead reckoning does)
  recurrence       how many distinct passes (sightings > 5 s apart) each node collected -- a passing PLACE is one the
                   mover comes back through
Usage: motion_map_readout.py "glob" [...]
Scoring only: the truth (qpos, train) labels; nothing here is a brain input.
"""
import collections, glob, json, math, statistics, sys


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def world(r, tx, ty):
    ox, oy, oyaw = r["odom"]
    d = yaw_of(r["qpos"][3:7]) - oyaw
    return (r["x"] + math.cos(d) * (tx - ox) - math.sin(d) * (ty - oy),
            r["y"] + math.sin(d) * (tx - ox) + math.cos(d) * (ty - oy))


def main():
    errs, node_d, node_spread, passes, nnodes, nsight = [], [], [], [], [], []
    for path in sorted(p for a in sys.argv[1:] for p in glob.glob(a)):
        track, per = [], collections.defaultdict(list)
        for line in open(path):
            if not line.startswith('{"t"'):
                continue
            r = json.loads(line)
            tr = r.get("train")
            if tr and int(r["t"] * 50) % 10 == 0:
                track.append((tr[0], tr[1]))
            mv = r.get("mvp")
            if mv and tr:
                wx, wy = world(r, 2.0 * mv[0], 2.0 * mv[1])
                errs.append(math.hypot(wx - tr[0], wy - tr[1]))
                per[int(mv[2])].append((r["t"], wx, wy))
        nsight.append(sum(len(v) for v in per.values()))
        kept = 0
        for w, pts in per.items():
            if len(pts) < 5:
                continue
            kept += 1
            cx, cy = statistics.mean(p[1] for p in pts), statistics.mean(p[2] for p in pts)
            node_d.append(min(math.hypot(cx - a, cy - b) for a, b in track) if track else float("nan"))
            node_spread.append(statistics.mean(math.hypot(p[1] - cx, p[2] - cy) for p in pts))
            n, last = 0, -1e9
            for t, _x, _y in pts:
                if t - last > 5.0:
                    n += 1
                last = t
            passes.append(n)
        nnodes.append(kept)

    def q(xs):
        xs = sorted(xs)
        return [round(xs[int(len(xs) * f)], 2) for f in (0.25, 0.5, 0.75)] if xs else None
    print(f"{len(nsight)} runs: sightings a run {statistics.mean(nsight):.0f}, nodes with >= 5 sightings a run {statistics.mean(nnodes):.1f}")
    print(f"  sighting error to the train (m) p25/50/75 {q(errs)}")
    print(f"  node centroid to the train's path (m) p25/50/75 {q(node_d)}")
    print(f"  node spread (m) p25/50/75 {q(node_spread)}")
    print(f"  passes per node p25/50/75 {q(passes)}; nodes with >= 2 passes {100 * sum(p >= 2 for p in passes) / max(1, len(passes)):.0f} %")


if __name__ == "__main__":
    main()
