#!/usr/bin/env python3
"""phantom_chase — does the duck believe a thing is moving when it is not, and does it keep the mover's direction?

The operator, 2026-10-11, on ST3 seed 7: "when the robot notices something moving, according to the arrow icon in the UI, it
may suddenly sense the object has changed direction.  The robot may also sense movement when it does not exist."  The arrow
is the seek loop's chase: its target and velocity (the `chase` record).  From the host's FULL JSONL, per chase episode (the
record's chase[0] rising), at its start:
  how         the decision that started it: 5 = a mover candidate's own sightings moved; 7 = a STATIC target's fixes drifted
              (§17.113 M1); other = a re-acquisition
  what        the thing nearest the target's believed position (the manifest's movables and the train, true poses), and
              whether it was really moving (true speed over the 1 s around the start above 0.03 m/s)
  phantom     the chase started on something that was not moving, or on nothing within 0.4 m
  self-turn   the body's own yaw rate (odometry) over the 1 s before the start: is a phantom made by the duck turning
And over the chase's own (non-coasting) ticks:
  flips       the velocity estimate's heading turning more than 90 deg between consecutive updates -- "it suddenly senses
              the object has changed direction"; for a real mover, the median error of the believed heading against the true
  seconds     per run: chasing or coasting a phantom, and a real mover
Usage: phantom_chase.py DIR [DIR ...] [--from 30] [--per-run]
Several dirs: then per-seed means and the mean difference to the first dir over the seeds both have, with its 95 % CI.
Scoring only: the truth (qpos, manifest, train) labels; nothing here is a brain input.
"""
from __future__ import annotations

import collections
import glob
import json
import math
import os
import re
import statistics
import sys

MANIFEST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "microduck",
                        "scene_playroom_train.manifest.json")
NAMES = {"obj_ball0": "red ball", "obj_ball1": "blue ball", "obj_block0": "green block", "obj_block1": "purple block",
         "mov_train0": "train"}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run(path, lay, t0):
    hist = collections.deque(maxlen=101)       # (t, objs, odom yaw) for the second before
    eps, cur = [], None
    prev_ch = None
    pending = []                               # episodes waiting for the second after, to measure the truth speed
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        t = r["t"]
        q, od = r["qpos"], r["odom"]
        objs = {n: (q[lay[n]], q[lay[n] + 1]) for n in NAMES}
        hist.append((t, objs, od[2]))
        for e in list(pending):                # the truth speed over the 1 s around the start
            if t >= e["t"] + 0.5:
                o0 = e["_objs0"]
                e["speed"] = {n: math.hypot(objs[n][0] - o0[n][0], objs[n][1] - o0[n][1]) / (t - e["_t0"]) for n in NAMES}
                e["true_dir"] = {n: math.atan2(objs[n][1] - o0[n][1], objs[n][0] - o0[n][0]) for n in NAMES}
                pending.remove(e)
        ch = r.get("chase")
        if not ch or t < t0:
            prev_ch = ch
            continue
        byaw = math.atan2(2 * (q[3] * q[6] + q[4] * q[5]), 1 - 2 * (q[5] ** 2 + q[6] ** 2))
        dth = byaw - od[2]
        c, s = math.cos(dth), math.sin(dth)

        def world(px, py):
            dx, dy = px - od[0], py - od[1]
            return r["x"] + c * dx - s * dy, r["y"] + s * dx + c * dy

        active = bool(ch[0])
        if active and not (prev_ch and (prev_ch[0] or prev_ch[10])):
            wx, wy = world(ch[2], ch[3])
            d, n = min((math.hypot(wx - o[0], wy - o[1]), n) for n, o in objs.items())
            back = [h for h in hist if h[0] >= t - 0.5]
            fwd_ago = [h for h in hist if h[0] >= t - 1.0]
            turn = max(abs(wrap(fwd_ago[i + 1][2] - fwd_ago[i][2])) / 0.02 for i in range(len(fwd_ago) - 1)) if len(fwd_ago) > 1 else 0.0
            yawrate = abs(wrap(fwd_ago[-1][2] - fwd_ago[0][2])) / max(1e-3, fwd_ago[-1][0] - fwd_ago[0][0])
            cur = {"t": t, "how": int(ch[13]), "near": n, "d": d, "rng": math.hypot(wx - r["x"], wy - r["y"]), "act_s": 0.0, "coast_s": 0.0, "n0": ch[1], "nc": None, "turn_peak": turn, "yawrate": yawrate,
                   "_objs0": back[0][1], "_t0": back[0][0], "dirs": [], "last_v": None, "flips": 0, "updates": 0,
                   "errs": []}
            eps.append(cur)
            pending.append(cur)
        if cur is not None and (active or ch[10]):
            cur["coast_s" if ch[10] else "act_s"] += 0.02
            if ch[10] and cur["nc"] is None:
                cur["nc"] = ch[1] - cur["n0"]
        if active and cur is not None and not ch[10]:
            v = (ch[4], ch[5])
            if v != cur["last_v"] and math.hypot(*v) > 1e-4:
                wvx, wvy = c * v[0] - s * v[1], s * v[0] + c * v[1]
                h = math.atan2(wvy, wvx)
                if cur["dirs"] and abs(wrap(h - cur["dirs"][-1])) > math.pi / 2:
                    cur["flips"] += 1
                cur["dirs"].append(h)
                cur["updates"] += 1
                cur["last_v"] = v
                tr = r.get("train")
                if cur["near"] == "mov_train0" and cur["d"] < 0.4 and tr and tr[5]:
                    cur["errs"].append(abs(wrap(h - math.atan2(tr[4], tr[3]))))
        if not active and not ch[10]:
            cur = None
        prev_ch = ch
    out = []
    for e in eps:
        if "speed" not in e:
            continue
        sp = e["speed"][e["near"]]
        e["moving"] = e["d"] < 0.4 and sp > 0.03
        out.append(e)
    return out


def seed_of(p):
    m = re.search(r"_s(\d+)\.jsonl$", p)
    return int(m.group(1)) if m else p


def per_seed(eps):
    ph = [e for e in eps if not e["moving"]]
    re_ = [e for e in eps if e["moving"]]
    return {"chases": len(eps), "phantoms": len(ph), "phantom %": 100 * len(ph) / len(eps) if eps else 0.0,
            "phantom s": sum(e["act_s"] + e["coast_s"] for e in ph), "phantom coast s": sum(e["coast_s"] for e in ph),
            "real s": sum(e["act_s"] + e["coast_s"] for e in re_),
            "train s": sum(e["act_s"] + e["coast_s"] for e in re_ if e["near"] == "mov_train0"),
            "flips": sum(e["flips"] for e in eps)}


def main():
    args = sys.argv[1:]
    t0 = 30.0
    per_run = "--per-run" in args
    if per_run:
        args.remove("--per-run")
    if "--from" in args:
        i = args.index("--from"); t0 = float(args[i + 1]); del args[i:i + 2]
    m = json.load(open(MANIFEST))
    lay = {n: i for n, i, _k in m["qpos_layout"]}
    print(f"{'arm':8s} {'runs':>4s} {'chases':>7s} {'phantom':>8s} {'%':>5s}  {'by 5':>9s} {'by 7':>9s} {'other':>9s}"
          f"  {'phantom on':>32s}  {'yaw/s ph|real':>14s}  {'flips/ep ph|real':>16s}  {'hdg err real':>12s}")
    arms = []
    for d in args:
        files = sorted(glob.glob(os.path.join(d, "*.jsonl")))
        allep = []
        seeds = {}
        for p in files:
            eps = run(p, lay, t0)
            seeds[seed_of(p)] = per_seed(eps)
            if per_run:
                for e in eps:
                    print(f"  {os.path.basename(p)} t {e['t']:7.2f} how {e['how']} near {NAMES[e['near']]:12s} d {e['d']:.2f} "
                          f"speed {e['speed'][e['near']]:.3f} {'MOVING ' if e['moving'] else 'PHANTOM'} yaw/s {e['yawrate']:.2f} "
                          f"flips {e['flips']}/{e['updates']}")
            allep += eps
        if not allep:
            continue
        ph = [e for e in allep if not e["moving"]]
        re_ = [e for e in allep if e["moving"]]

        def by(code, L):
            if code == "o":
                return sum(e["how"] not in (5, 7) for e in L)
            return sum(e["how"] == code for e in L)
        cnt = collections.Counter(NAMES[e["near"]] if e["d"] < 0.4 else "nothing" for e in ph)
        on = ", ".join(f"{k} {v}" for k, v in cnt.most_common(3))
        med = lambda L: statistics.median(L) if L else float("nan")
        errs = [x for e in re_ for x in e["errs"]]
        print(f"{os.path.basename(d.rstrip('/')):8s} {len(files):4d} {len(allep) / len(files):7.1f} {len(ph) / len(files):8.1f} "
              f"{100 * len(ph) / len(allep):5.0f}  {by(5, ph):3d}/{by(5, allep):<5d} {by(7, ph):3d}/{by(7, allep):<5d} "
              f"{by('o', ph):3d}/{by('o', allep):<5d}  {on:>32s}  {med([e['yawrate'] for e in ph]):5.2f}|{med([e['yawrate'] for e in re_]):<5.2f}"
              f"     {statistics.mean([e['flips'] for e in ph]) if ph else 0:5.2f}|{statistics.mean([e['flips'] for e in re_]) if re_ else 0:<5.2f}"
              f"      {math.degrees(med(errs)):6.0f} deg")
        arms.append((os.path.basename(d.rstrip("/")), seeds))
    if len(arms) < 2:
        return
    keys = list(next(iter(arms[0][1].values())).keys())
    print(f"\n{'per run':16s}" + "".join(f"{n:>16s}" for n, _ in arms))
    for k in keys:
        print(f"{k:16s}" + "".join(f"{statistics.mean(v[k] for v in s.values()):16.1f}" for _, s in arms))
    base = arms[0][1]
    for name, s in arms[1:]:
        common = sorted(set(base) & set(s), key=str)
        if len(common) < 2:
            continue
        print(f"\n{name} - {arms[0][0]} (paired, n {len(common)})")
        for k in keys:
            dd = [s[x][k] - base[x][k] for x in common]
            m_, h = statistics.mean(dd), 2.07 * statistics.stdev(dd) / math.sqrt(len(dd))
            print(f"  {k:16s} {m_:+8.2f}  [{m_ - h:+.2f}, {m_ + h:+.2f}]{'  *' if abs(m_) > h else ''}")


if __name__ == "__main__":
    main()
