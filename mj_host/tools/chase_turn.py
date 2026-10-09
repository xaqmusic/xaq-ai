#!/usr/bin/env python3
"""chase_turn — can the body turn with the mover it chases? (the chase push, design doc §17.113, 2026-10-09)

The operator, watching ★ M2: "the head follows the train but the body can't turn enough during walking to follow the train
around the turns."  From the host's FULL JSONL (`chase`, `twist`, `odom`, `qpos`, `stop`), over half-second windows of a chase
(chasing or coasting, walking, both ends): binned by how far the chase target sits off the nose (the odometry's own frame),
the forward command, the body's real speed, the yaw command, the body's real turn rate over the window (the gait's sway
cancels), the share turning toward the target, and the turning radius (speed / turn rate).
Usage: chase_turn.py "glob" [...]
"""
import collections, glob, json, math, sys


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    bins = collections.defaultdict(lambda: [0, 0.0, 0.0, 0.0, 0.0, 0, 0.0])
    for path in sorted(p for a in sys.argv[1:] for p in glob.glob(a)):
        R = []
        for line in open(path):
            if not line.startswith('{"t"'):
                continue
            r = json.loads(line)
            R.append((yaw_of(r["qpos"][3:7]), r.get("chase") or [0] * 14, r.get("twist"), r.get("odom"), int(r.get("stop", 0)), r["x"], r["y"]))
        for i in range(0, len(R) - 25, 25):
            yaw, ch, tw, od, st, x, y = R[i]
            yaw2, ch2, _tw2, _od2, st2, x2, y2 = R[i + 25]
            if not ((ch[0] or ch[10]) and (ch2[0] or ch2[10]) and tw and od and st == 0 and st2 == 0):
                continue
            ox, oy, oyaw = od
            e = wrap(math.atan2(ch[3] - oy, ch[2] - ox) - oyaw)
            rate = wrap(yaw2 - yaw) / 0.5
            v = math.hypot(x2 - x, y2 - y) / 0.5
            b = bins[min(int(abs(e) / 0.35), 4)]
            b[0] += 1; b[1] += tw[0]; b[2] += v; b[3] += abs(tw[2]); b[4] += abs(rate); b[5] += rate * e > 0
    labels = ["<20 deg", "20-40", "40-60", "60-80", ">80"]
    tot = sum(v[0] for v in bins.values()) or 1
    for k in range(5):
        n, vx, v, cmd, act, right, _ = bins[k]
        if n:
            print(f"target {labels[k]:7s} {100 * n / tot:4.0f} %: vx cmd {vx / n:.2f}, speed {v / n:.2f} m/s, |yaw cmd| {cmd / n:.2f}, "
                  f"|turn| {act / n:.2f} rad/s, toward it {100 * right / n:.0f} %, radius {v / n / max(act / n, 1e-3):.2f} m")


if __name__ == "__main__":
    main()
