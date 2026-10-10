#!/usr/bin/env python3
"""motion_readout — does the always-on motion sensor see the moving train, and what else does it call motion?
(the chase push, step 1, 2026-10-02).

From full host logs run with --log-motion ("mot" on every ToF cast: evidence points, tracks, the published track, all in
the odometry frame) and the train's truth.  Odometry positions go to the world through the trunk's true pose and the
odometry's own pose at that tick (as the seek target is placed in ten_minutes.py) -- scoring only.

  in view       casts with the MOVING train within 2 m and within the ToF's 45 deg cone (body yaw + head yaw)
  recall        of those casts: evidence within 0.2 m of the train; a published track within 0.25 m; and the cloud's own mover
                detector (the chase record's mover-seen flag on any of the cast's four ticks) for comparison
  latency       per entry of the moving train into view: seconds to the first published track on it (sensor / cloud)
  false tracks  published tracks more than 0.3 m from a moving train, a minute, labelled by what is truly there
                (a thing, structure, nothing) -- and the stopped train separately

Usage: CLOUD_OBJECTS_MANIFEST=... motion_readout.py "glob" [--per-run]
"""
from __future__ import annotations

import collections
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ten_minutes as tm  # noqa: E402

HALF = math.radians(22.5)


def to_world(r, ox_, oy_):
    x, y = r["x"], r["y"]
    byaw = tm.yaw_of(r["qpos"][3:7])
    ox, oy, oyaw = r["odom"]
    d = byaw - oyaw
    return x + math.cos(d) * (ox_ - ox) - math.sin(d) * (oy_ - oy), y + math.sin(d) * (ox_ - ox) + math.cos(d) * (oy_ - oy)


def run(path, sc):
    o = collections.Counter()
    lat_s, lat_c = [], []
    entry = None
    secs = 0.0
    cloud_seen_recent = collections.deque(maxlen=4)
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        t = r["t"]
        secs = t
        ch = r.get("chase")
        cloud_seen_recent.append(bool(ch and ch[6]))
        m = r.get("mot")
        tr = r.get("train")
        if m is None or tr is None:
            continue
        o["casts"] += 1
        x, y = r["x"], r["y"]
        yaw = tm.yaw_of(r["qpos"][3:7])
        view = yaw + r["q"][7]
        d = math.hypot(tr[0] - x, tr[1] - y)
        b = abs(tm.wrap(math.atan2(tr[1] - y, tr[0] - x) - view))
        moving = bool(tr[5])
        in_view = moving and d < 2.0 and b < HALF
        pub = None
        for trk in m["tr"]:
            if trk[0] == m["p"]:
                pub = to_world(r, trk[1], trk[2])
        ev_near = any(math.hypot(wx - tr[0], wy - tr[1]) < 0.2 for wx, wy in (to_world(r, e[0], e[1]) for e in m["e"]))
        pub_near = pub is not None and math.hypot(pub[0] - tr[0], pub[1] - tr[1]) < 0.25
        cloud = any(cloud_seen_recent)
        if in_view:
            o["view"] += 1
            o["ev"] += ev_near
            o["pub"] += pub_near
            o["cloud"] += cloud
            if entry is None:
                entry = dict(t=t, s=None, c=None)
            if pub_near and entry["s"] is None:
                entry["s"] = t - entry["t"]
            if cloud and entry["c"] is None:
                entry["c"] = t - entry["t"]
        elif entry is not None and not in_view:
            lat_s.append(entry["s"])
            lat_c.append(entry["c"])
            entry = None
        if pub is not None and not pub_near:
            near_train = math.hypot(pub[0] - tr[0], pub[1] - tr[1]) < 0.3
            if near_train and not moving:
                o["fa_train_stopped"] += 1
            elif not near_train:
                o["fa_" + sc.label(pub[0], pub[1], sc.things(r))] += 1
    return o, lat_s, lat_c, secs


def main():
    args = sys.argv[1:]
    per_run = "--per-run" in args
    args = [a for a in args if a != "--per-run"]
    sc = tm.Scene()
    tot = collections.Counter()
    ls, lc = [], []
    mins = 0.0
    for p in sorted(q for g in args for q in glob.glob(g)):
        o, a, b, secs = run(p, sc)
        tot.update(o)
        ls += a
        lc += b
        mins += secs / 60.0
        if per_run:
            v = o["view"] or 1
            print(f"  {os.path.basename(p):42s} in view {o['view']:5d}  ev {100 * o['ev'] / v:4.0f}%  track {100 * o['pub'] / v:4.0f}%  "
                  f"cloud {100 * o['cloud'] / v:4.0f}%  false/min {sum(o[k] for k in o if k.startswith('fa_')) / (secs / 60):.1f}")
    v = tot["view"] or 1
    print(f"casts {tot['casts']}; the moving train in view on {tot['view']} ({100 * tot['view'] / max(1, tot['casts']):.0f} %)")
    print(f"  recall in view: evidence near the train {100 * tot['ev'] / v:.0f} %, a published track on it {100 * tot['pub'] / v:.0f} %, "
          f"the cloud's mover detector {100 * tot['cloud'] / v:.0f} %")
    def lat(xs):
        got = sorted(x for x in xs if x is not None)
        return f"{len(got)}/{len(xs)} entries caught, median {got[len(got) // 2]:.2f} s" if got else f"0/{len(xs)}"
    print(f"  latency after the train enters view: the sensor {lat(ls)}; the cloud {lat(lc)}")
    fa = {k[3:]: tot[k] / mins for k in tot if k.startswith("fa_")}
    print(f"  false published tracks a minute: {sum(fa.values()):.1f}  " + "  ".join(f"{k} {v:.1f}" for k, v in sorted(fa.items())))


if __name__ == "__main__":
    main()
