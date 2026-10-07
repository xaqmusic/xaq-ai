#!/usr/bin/env python3
"""fx_study — paired, seed-by-seed comparison of lever arms against a base arm on the full metric set (2026-10-04).

Built for the §17.108 fixes (the mirrored escapes, the mirrored stop sweep, the place map's honest bake, the map's pose
range), and general: give it a base logdir and arm logdirs of FULL host logs from l2_sweep (--full-logs), the same seeds.

Per run (control phase from --from s):
  boring / interesting %      ten_minutes.classify_run
  walls/min, cells            wall-contact episodes a minute; 0.25 m cells the trunk visited
  falls                       rescues (event reset:handoff)
  block touched in 1st min    first_minute.py --per-run (a skill at obj_block0 that touched it)
  chase s on the moving train, contacts      chase_readout.run
  skills, skills touching, stops at things   ten_minutes
  escapes; after an escape: net displacement in 10 s, % of those 10 s in wall contact     (F1)
  arrival stops: % of their ticks with the seek target inside the ToF's 45 deg cone (body yaw + head yaw)   (F2)
  outcome loop: skills observed / requested                                               (F2, the kicked-thing look)
  map: nodes and baked at the end; baked on walks vs at stops (stderr "map baking")        (F3, F4)

Statistics: per metric, the mean ± sd of each arm, and for each arm the paired difference to the base over the seeds
both have: mean diff, its 95 % CI (t), and the smallest difference this n could detect at 80 % power (2.8 sd/sqrt(n)).

Usage: CLOUD_OBJECTS_MANIFEST=... fx_study.py BASE_DIR ARM_DIR [ARM_DIR ...] [--from 30] [--json OUT]
Scoring only: the truth (qpos, train, manifest) labels; nothing here is a brain input.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import statistics
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chase_readout as cr  # noqa: E402
import ten_minutes as tm  # noqa: E402

HALF = math.radians(22.5)
T_CRIT = {5: 2.571, 10: 2.228, 15: 2.131, 17: 2.110, 20: 2.086, 23: 2.069, 30: 2.042}


def tcrit(df):
    best = min(T_CRIT, key=lambda k: abs(k - df))
    return T_CRIT[best]


def seed_of(path):
    m = re.search(r"_s(\d+)\.jsonl$", path)
    return int(m.group(1)) if m else None


def to_world(r, ox_, oy_):
    x, y = r["x"], r["y"]
    byaw = tm.yaw_of(r["qpos"][3:7])
    ox, oy, oyaw = r["odom"]
    d = byaw - oyaw
    return x + math.cos(d) * (ox_ - ox) - math.sin(d) * (oy_ - oy), y + math.sin(d) * (ox_ - ox) + math.cos(d) * (oy_ - oy)


def scan_jsonl(path, t_from):
    out = dict(walls=0, cells=set(), falls=0, minutes=0.0, esc=0, esc_disp=[], esc_wall=[], arr_ticks=0, arr_in=0)
    prev_wall = 0
    pending = []            # escapes: [t0, x0, y0, wall_ticks, ticks]
    arr_target = None       # the arrival stop's target (world), while the stop runs
    prev = None
    t_last = 0.0
    for line in open(path):
        if not line.startswith('{"t"'):
            continue
        r = json.loads(line)
        t = r["t"]
        t_last = t
        ev = r.get("event", "")
        x, y = r["x"], r["y"]
        if ev == "reset:handoff":
            out["falls"] += 1
        if t >= t_from:
            w = int(r.get("wall", 0))
            out["walls"] += (w and not prev_wall)
            prev_wall = w
            out["cells"].add((int(math.floor(x / 0.25)), int(math.floor(y / 0.25))))
        # F1: escapes
        if ev in ("stop:escape", "impeded:wall"):
            out["esc"] += 1
            pending.append([t, x, y, 0, 0])
        for p in pending:
            p[3] += int(r.get("wall", 0)); p[4] += 1
        done = [p for p in pending if t - p[0] >= 10.0]
        for p in done:
            out["esc_disp"].append(math.hypot(x - p[1], y - p[2]))
            out["esc_wall"].append(100.0 * p[3] / max(1, p[4]))
        pending = [p for p in pending if t - p[0] < 10.0]
        # F2: the arrival stop's view of its target
        if ev == "stop:arrive" and prev is not None and prev.get("chase"):
            arr_target = to_world(prev, prev["chase"][2], prev["chase"][3])
        elif ev.startswith("stop:") and ev[5:] in ("end", "bored", "orient", "chase", "escape"):
            arr_target = None
        if arr_target is not None and r.get("drive") == "stand":
            yaw = tm.yaw_of(r["qpos"][3:7]) + r["q"][7]
            b = abs(tm.wrap(math.atan2(arr_target[1] - y, arr_target[0] - x) - yaw))
            out["arr_ticks"] += 1
            out["arr_in"] += b < HALF
        prev = r
    out["minutes"] = max(1e-9, (t_last - t_from) / 60.0)
    return out


def scan_stderr(path):
    out = dict(map_nodes=None, map_baked=None, baked_walk=None, baked_stop=None, observed=None, requests=None)
    if not os.path.exists(path):
        return out
    s = open(path).read()
    m = re.search(r"map baking: inserted \d+ at stops / \d+ on walks, baked (\d+) at stops / (\d+) on walks.*?; (\d+) nodes, (\d+) baked at the end", s)
    if m:
        out["baked_stop"], out["baked_walk"], out["map_nodes"], out["map_baked"] = map(int, m.groups())
    m = re.search(r"^  outcome (\{.*\})$", s, re.M)
    if m:
        try:
            d = json.loads(m.group(1))
            out["observed"] = d.get("observed")
            out["requests"] = d.get("requests")
        except json.JSONDecodeError:
            pass
    return out


def first_minute(paths):
    env = dict(os.environ)
    res = subprocess.run([sys.executable, os.path.join(HERE, "first_minute.py")] + paths + ["--per-run"],
                         capture_output=True, text=True, env=env).stdout
    out = {}
    for line in res.splitlines():
        m = re.match(r"\s+(\S+\.jsonl)\s+closest", line)
        if m:
            out[m.group(1)] = 1.0 if re.search(r"@\d+s e[\d.]+ b[+-]?\d+ T", line) else 0.0
    return out


def run_metrics(path, t_from, sc, fm):
    j = scan_jsonl(path, t_from)
    e = scan_stderr(os.path.splitext(path)[0] + ".stderr")
    c = tm.classify_run(path, sc, t_from, 3.0)
    cnt = c["cnt"]
    tot = max(1, sum(cnt.values()))
    boring = sum(cnt[k] for k in tm.BORING) / tot * 100
    interesting = sum(cnt[k] for k in ("skill", "chase", "stand@thing", "seek->thing")) / tot * 100
    ch = cr.run(path)
    m = {
        "boring %": boring,
        "interesting %": interesting,
        "walls/min": j["walls"] / j["minutes"],
        "cells (0.25 m)": float(len(j["cells"])),
        "falls": float(j["falls"]),
        "block touched, 1st min": fm.get(os.path.basename(path), float("nan")),
        "moving train s": ch["chase_moving_s"],
        "train contacts": float(ch["contacts"]),
        "skills": float(c["skills"]),
        "skills touching %": 100.0 * c["skills_touch"] / c["skills"] if c["skills"] else float("nan"),
        "stops at things %": 100.0 * c["stops_thing"] / c["stops"] if c["stops"] else float("nan"),
        "escapes": float(j["esc"]),
        "after escape: disp 10 s (m)": statistics.mean(j["esc_disp"]) if j["esc_disp"] else float("nan"),
        "after escape: wall % of 10 s": statistics.mean(j["esc_wall"]) if j["esc_wall"] else float("nan"),
        "arrival stop: target in view %": 100.0 * j["arr_in"] / j["arr_ticks"] if j["arr_ticks"] else float("nan"),
        "outcomes observed %": (100.0 * e["observed"] / e["requests"]) if e["requests"] else float("nan"),
        "map nodes": float(e["map_nodes"]) if e["map_nodes"] is not None else float("nan"),
        "map baked on walks": float(e["baked_walk"]) if e["baked_walk"] is not None else float("nan"),
        "map baked at stops": float(e["baked_stop"]) if e["baked_stop"] is not None else float("nan"),
    }
    return m


def arm_runs(d, t_from, sc):
    paths = sorted(glob.glob(os.path.join(d, "*_s*.jsonl")), key=lambda p: seed_of(p) or 0)
    fm = first_minute(paths)
    return {seed_of(p): run_metrics(p, t_from, sc, fm) for p in paths}


def fmt(x):
    return "   nan" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:6.2f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--from", dest="t_from", type=float, default=30.0)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    sc = tm.Scene()
    arms = {os.path.basename(d.rstrip("/")): arm_runs(d, a.t_from, sc) for d in a.dirs}
    names = list(arms)
    base = names[0]
    metrics = list(next(iter(arms[base].values())).keys())
    print(f"base {base} (n={len(arms[base])}); arms " + ", ".join(f"{n} (n={len(arms[n])})" for n in names[1:]))
    for k in metrics:
        print(f"\n{k}")
        for n in names:
            v = [r[k] for r in arms[n].values() if not math.isnan(r[k])]
            mean = statistics.mean(v) if v else float("nan")
            sd = statistics.stdev(v) if len(v) > 1 else float("nan")
            line = f"  {n:6s} {fmt(mean)} ± {fmt(sd)}  (n={len(v)})"
            if n != base:
                seeds = [s for s in arms[n] if s in arms[base] and not math.isnan(arms[n][s][k]) and not math.isnan(arms[base][s][k])]
                d = [arms[n][s][k] - arms[base][s][k] for s in seeds]
                if len(d) > 2:
                    md, sdd = statistics.mean(d), statistics.stdev(d)
                    half = tcrit(len(d) - 1) * sdd / math.sqrt(len(d))
                    mde = 2.8 * sdd / math.sqrt(len(d))
                    sig = "*" if abs(md) > half else " "
                    line += f"   paired Δ {md:+7.2f}  95% CI [{md - half:+.2f}, {md + half:+.2f}] {sig}  (n={len(d)}, detectable ±{mde:.2f})"
            print(line)
    if a.json:
        json.dump({n: {str(s): r for s, r in arms[n].items()} for n in names}, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
