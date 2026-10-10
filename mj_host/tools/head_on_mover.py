#!/usr/bin/env python3
"""head_on_mover -- does the head follow the moving train while the duck walks? (design doc §17.114 step 1, 2026-10-09)

The operator: "the head did not follow the moving train even as it was in view."  From the host's FULL JSONL (`q` the joints,
`qpos` the trunk, `train` its true place, `chase`, `stop`, `head` the head brain's command): on the walk (no stop), with the
train moving and within 2 m, per tick --
  bearing    the train's true bearing from the trunk (rad, + left)
  head yaw   the head-yaw joint from home (rad, + left)
  on it      |bearing - head yaw| < 0.39 rad (the cone's half-width, 22.5 deg)
  aim        the head yaw's share of the bearing (head yaw / bearing, where |bearing| > 0.3)
split by whether a chase holds it (chasing or coasting) or not.  Also what the head brain is told to want while chasing: the
chase target's bearing (chase[2..3], odometry frame) against the true one.
Usage: head_on_mover.py "glob" [...]
Scoring only: the truth labels; nothing here is a brain input.
"""
import glob, json, math, statistics, sys

HOME_YAW = 0.0


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    rows = {True: [], False: []}
    tgt_err = []
    for path in sorted(p for a in sys.argv[1:] for p in glob.glob(a)):
        for line in open(path):
            if not line.startswith('{"t"'):
                continue
            r = json.loads(line)
            tr, ch = r.get("train"), r.get("chase")
            if not tr or not tr[5] or int(r.get("stop", 0)) or r.get("drive") != "walk":
                continue
            dx, dy = tr[0] - r["x"], tr[1] - r["y"]
            if math.hypot(dx, dy) > 2.0:
                continue
            yaw = yaw_of(r["qpos"][3:7])
            b = wrap(math.atan2(dy, dx) - yaw)
            hy = r["q"][7] - HOME_YAW
            chasing = bool(ch and (ch[0] or ch[10]))
            rows[chasing].append((b, hy))
            if chasing and ch[0]:
                ox, oy, oyaw = r["odom"]
                tb = wrap(math.atan2(ch[3] - oy, ch[2] - ox) - oyaw)
                tgt_err.append(abs(wrap(tb - b)))
    for k, name in ((True, "chasing"), (False, "not chasing")):
        v = rows[k]
        if not v:
            continue
        on = sum(abs(wrap(b - h)) < 0.39 for b, h in v) / len(v)
        off = [(b, h) for b, h in v if abs(b) > 0.3]
        aim = statistics.median(h / b for b, h in off) if off else float("nan")
        print(f"{name:12s} {len(v) / 50:7.0f} s: |bearing| {statistics.mean(abs(b) for b, _ in v):.2f} rad, |head yaw| "
              f"{statistics.mean(abs(h) for _, h in v):.2f}, on it {100 * on:.0f} %, the head yaw's share of the bearing {aim:+.2f}")
        for lo, hi in ((0, 0.39), (0.39, 0.8), (0.8, 1.2), (1.2, 3.2)):
            s = [(b, h) for b, h in v if lo <= abs(b) < hi]
            if s:
                print(f"    bearing {lo:.2f}-{hi:.2f}: {100 * len(s) / len(v):3.0f} %, head yaw toward it "
                      f"{statistics.mean(h * math.copysign(1, b) for b, h in s):+.2f} rad, on it "
                      f"{100 * sum(abs(wrap(b - h)) < 0.39 for b, h in s) / len(s):.0f} %")
    if tgt_err:
        print(f"chasing: the chase target's bearing against the train's, median error {statistics.median(tgt_err):.2f} rad")


if __name__ == "__main__":
    main()
