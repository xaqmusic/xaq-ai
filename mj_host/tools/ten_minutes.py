#!/usr/bin/env python3
"""ten_minutes — where a run's minutes go, and what starts the boring ones (ten-minutes phase §3, 2026-10-02).

The operator: the goal is ten minutes of interesting behaviour; the run gets boring when the duck gets stuck in corners
and stares at the wall.  Every tick of a FULL host log is sorted into one category (first match wins); every boring
episode longer than --min-ep seconds is traced to what began it.

  down             the recovery scaffold drives
  skill            a kick, peck or push runs
  chase            the chase holds a mover
  stand@thing      standing, a movable thing within 0.6 m of the trunk
  stare@structure  standing, wall or furniture within 0.5 m along the head's view, nothing movable in the ToF's cone
                   within 1.5 m
  stand-open       any other standing
  pinned           walking, in contact with a wall or furniture, or under 3 cm/s over 2 s with structure within 0.3 m
  seek->thing      walking under the seek loop, its target (the brain's believed position, put into the world through
  seek->structure  the odometry's own pose) labelled by what is truly there: a movable thing, a wall or furniture,
  seek->nothing    or neither
  wander           walking, play (or nothing) holds the reference

Boring = stare@structure + pinned + seek->structure + seek->nothing.  Interesting = skill + chase + stand@thing +
seek->thing.  The rest (down, stand-open, wander) is neutral.  The blind metric of a time budget is variety itself (a
slot machine scores varied, playroom plan §12.6); its complement printed here is contingency: the share of skills that
touched a movable thing, and of stops that began at one.

Usage: CLOUD_OBJECTS_MANIFEST=models/microduck/scene_playroom_train.manifest.json \\
       ten_minutes.py "log/campaign/f12/*.jsonl" [--from 30] [--min-ep 3] [--per-run] [--episodes N] [--json OUT]
Scoring only: the truth (qpos, manifest) labels; nothing here is a brain input.
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cloud_objects as co  # noqa: E402

CATS = ("down", "skill", "chase", "stand@thing", "stare@structure", "stand-open",
        "pinned", "seek->thing", "seek->structure", "seek->nothing", "wander")
BORING = ("stare@structure", "pinned", "seek->structure", "seek->nothing")
INTERESTING = ("skill", "chase", "stand@thing", "seek->thing")
HALF_CONE = math.radians(22.5)
DT = 0.02


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Scene:
    def __init__(self):
        self.half, self.lay, self.movable, self.furniture = co.load_scene()
        self.inner = self.half - co.WALL_T

    def things(self, r):
        q = r["qpos"]
        out = [(o["kind"], q[self.lay[o["name"]]], q[self.lay[o["name"]] + 1], o["extent"]) for o in self.movable
               if o["cls"] == "movable"]
        tr = r.get("train")
        if tr is not None:
            out.append(("train", tr[0], tr[1], 0.09))
        return out

    def structure_d(self, x, y):
        """Distance from (x, y) to the nearest wall face or furniture footprint (a disc of its extent)."""
        d = self.inner - max(abs(x), abs(y))
        for o in self.furniture:
            d = min(d, math.hypot(x - o["x"], y - o["y"]) - o["extent"])
        return d

    def label(self, x, y, things):
        dt = min(math.hypot(x - ox, y - oy) - e for _k, ox, oy, e in things)
        if dt < 0.15:
            return "thing"
        if self.structure_d(x, y) < 0.15:
            return "structure"
        return "nothing"

    def facing_structure(self, x, y, yaw, reach=0.5):
        for k in range(1, int(reach / 0.02) + 1):
            if self.structure_d(x + math.cos(yaw) * 0.02 * k, y + math.sin(yaw) * 0.02 * k) < 0.0:
                return True
        return False


def classify_run(path, sc, t_from, min_ep):
    cnt = collections.Counter()
    eps = []                        # boring episodes: (cat, t0, t1, cause)
    hist = collections.deque()      # (t, x, y) for the 2 s speed
    cur = None                      # [cat, t0, cause]
    last_stop_ev, last_stop_t = "", -1e9
    skills = skills_touch = 0
    in_skill = touched = False
    stops = stops_thing = 0
    prev_stop = 0
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        t = r["t"]
        ev = r.get("event") or ""
        if ev.startswith("stop:") and ev not in ("stop:handback", "stop:handoff"):
            last_stop_ev, last_stop_t = ev[5:], t
        x, y = r["x"], r["y"]
        hist.append((t, x, y))
        while hist and t - hist[0][0] > 2.0:
            hist.popleft()
        things = sc.things(r)
        s = r.get("stop", 0)
        if s and not prev_stop and t >= t_from:
            stops += 1
            stops_thing += min(math.hypot(x - ox, y - oy) - e for _k, ox, oy, e in things) < 0.6
        prev_stop = s
        sk = "skill" in r
        if sk and not in_skill:
            touched = False
        if sk:
            touched = touched or bool(r.get("obj", 0))
        if in_skill and not sk and t >= t_from:
            skills += 1
            skills_touch += touched
        in_skill = sk
        if t < t_from:
            continue

        q = r["qpos"]
        byaw = yaw_of(q[3:7])
        vyaw = byaw + r["q"][co.HEAD_YAW_Q]
        dthing = min(math.hypot(x - ox, y - oy) - e for _k, ox, oy, e in things)
        cause = ""
        if r["drive"] == "scaffold":
            cat = "down"
        elif sk:
            cat = "skill"
        elif r.get("chase", [0])[0]:
            cat = "chase"
        elif r["drive"] == "stand" or s:
            if dthing < 0.6:
                cat = "stand@thing"
            else:
                seen = any(math.hypot(ox - x, oy - y) < 1.5 and abs(wrap(math.atan2(oy - y, ox - x) - vyaw)) < HALF_CONE
                           for _k, ox, oy, _e in things)
                cat = "stare@structure" if (not seen and sc.facing_structure(x, y, vyaw)) else "stand-open"
            cause = "stop:" + last_stop_ev
        else:
            spd = (math.hypot(x - hist[0][1], y - hist[0][2]) / max(t - hist[0][0], 1e-6)) if len(hist) > 1 else 1.0
            sd = sc.structure_d(x, y)
            if r.get("wall", 0) or (spd < 0.03 and t - hist[0][0] > 1.9 and sd < 0.3):
                cat = "pinned"
                cause = {3: "seek", 1: "play", 4: "play-held", 2: "avoid"}.get(r.get("steer", 0), "none")
            elif r.get("steer") == 3 and r.get("seek", [0])[0] > 0 and "chase" in r:
                ox, oy, oyaw = r["odom"]
                tx, ty = r["chase"][2], r["chase"][3]
                dy_ = byaw - oyaw
                wx = x + math.cos(dy_) * (tx - ox) - math.sin(dy_) * (ty - oy)
                wy = y + math.sin(dy_) * (tx - ox) + math.cos(dy_) * (ty - oy)
                cat = "seek->" + sc.label(wx, wy, things)
            else:
                cat = "wander"
        cnt[cat] += 1
        # episodes of one boring category, contiguous
        if cur and cur[0] != cat:
            if cur[0] in BORING and t - cur[1] >= min_ep:
                eps.append((cur[0], cur[1], t, cur[2]))
            cur = None
        if cur is None and cat in BORING:
            cur = [cat, t, cause or ""]
    if cur and cur[0] in BORING:
        eps.append((cur[0], cur[1], t, cur[2]))
    return dict(run=os.path.basename(path), secs=sum(cnt.values()) * DT, cnt=cnt, eps=eps,
                skills=skills, skills_touch=skills_touch, stops=stops, stops_thing=stops_thing)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("globs", nargs="+")
    ap.add_argument("--from", dest="t_from", type=float, default=30.0)
    ap.add_argument("--min-ep", type=float, default=3.0)
    ap.add_argument("--per-run", action="store_true")
    ap.add_argument("--episodes", type=int, default=0, help="print the N longest boring episodes")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    paths = sorted(p for g in a.globs for p in glob.glob(g))
    sc = Scene()
    runs = [classify_run(p, sc, a.t_from, a.min_ep) for p in paths]
    n = len(runs)
    if not n:
        sys.exit("no logs")

    def share(rr, cats):
        tot = sum(rr["cnt"].values()) or 1
        return 100.0 * sum(rr["cnt"][c] for c in cats) / tot

    print(f"{n} runs, judged from {a.t_from:.0f} s; seconds per run (mean ± sd) and share")
    for c in CATS:
        v = [rr["cnt"][c] * DT for rr in runs]
        m = sum(v) / n
        sd = (sum((x - m) ** 2 for x in v) / max(n - 1, 1)) ** 0.5
        tag = " B" if c in BORING else (" I" if c in INTERESTING else "")
        print(f"  {c:16s}{tag:2s} {m:7.1f} ± {sd:6.1f} s   {100 * m / (sum(rr["secs"] for rr in runs) / n or 1):5.1f} %")
    b = [share(rr, BORING) for rr in runs]
    i = [share(rr, INTERESTING) for rr in runs]
    print(f"  BORING {sum(b) / n:.1f} %  (per run {min(b):.0f}–{max(b):.0f})   INTERESTING {sum(i) / n:.1f} %  "
          f"(per run {min(i):.0f}–{max(i):.0f})")
    sk = sum(rr["skills"] for rr in runs); skt = sum(rr["skills_touch"] for rr in runs)
    st = sum(rr["stops"] for rr in runs); stt = sum(rr["stops_thing"] for rr in runs)
    print(f"  contingency: skills touching a thing {skt}/{sk}; stops begun at a thing {stt}/{st}")

    # what starts the boring episodes
    by = collections.defaultdict(lambda: [0, 0.0])
    for rr in runs:
        for cat, t0, t1, cause in rr["eps"]:
            k = (cat, cause)
            by[k][0] += 1
            by[k][1] += t1 - t0
    print(f"\nboring episodes ≥ {a.min_ep:.0f} s by category and what began them (count, seconds per run)")
    for (cat, cause), (c, s) in sorted(by.items(), key=lambda kv: -kv[1][1]):
        print(f"  {cat:16s} {cause:14s} {c:5d}  {s / n:7.1f} s")

    if a.per_run:
        print(f"\n{'run':44s} boring  interesting  stare  pinned  seek->str  seek->none  longest boring")
        for rr in runs:
            lo = max((t1 - t0 for _c, t0, t1, _x in rr["eps"]), default=0.0)
            print(f"{rr['run']:44s} {share(rr, BORING):5.1f}  {share(rr, INTERESTING):11.1f}  "
                  f"{rr['cnt']['stare@structure'] * DT:5.0f}  {rr['cnt']['pinned'] * DT:6.0f}  "
                  f"{rr['cnt']['seek->structure'] * DT:9.0f}  {rr['cnt']['seek->nothing'] * DT:10.0f}  {lo:6.1f}")
    if a.episodes:
        allep = sorted(((t1 - t0, rr["run"], cat, t0, cause) for rr in runs for cat, t0, t1, cause in rr["eps"]),
                       reverse=True)[:a.episodes]
        print(f"\nthe {a.episodes} longest boring episodes")
        for d, run, cat, t0, cause in allep:
            print(f"  {d:6.1f} s  {run:44s} t {t0:6.1f}  {cat:16s} {cause}")
    if a.json:
        json.dump([dict(rr, cnt=dict(rr["cnt"])) for rr in runs], open(a.json, "w"))


if __name__ == "__main__":
    main()
