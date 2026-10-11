#!/usr/bin/env python3
"""sim_robot_compare.py — one robot brain run against sim runs of the same config, on the same numbers.

    sim_robot_compare.py --robot <dashrun_dir> --sim <label>=<power_log.csv>[,<more>] [--sim ...]

The robot dir holds the dash run's feed.jsonl (benchd's 50 Hz state feed), its inacap_*_sag.csv
(the fast INA219 capture) and benchd_log.jsonl (benchd's event record, for the HAT resets).  Each sim
log is OGMA_PICRAWLER_POWER_LOG output; a trace beside it (tr<prefix>... with the same seed) adds
foot contact.  Numbers, each measured the same way on both sides:

  line speed   joint angle sampled over exact 100 ms spans, mean |change| per second (rad/s), per
               joint type.  Robot: the pulse on the line (benchd out_us, after the servo lag) over
               the measured 545.2 us/rad.  Sim: the joint itself (the power log's relative joint
               rate, integrated over the span).  What the sim HUD and picrawler-dash both show.
  current      battery-side HAT current: mean, 10 ms peaks, excursions above 3 A.  Sim: the
               electrical model's output (fitted to an earlier carpet run's MEAN only).
  rail         3.3 V rail dips per minute (robot: 3.3 x pack V / A4; sim: the model).
  stepping     contact duty and swings >= 4 ticks per minute (robot: FSR > 200; sim: physics contact).

Robot frames count only while brain-driven, on the floor (belly ToF <= 100 mm), and away from HAT
resets (1.5 s before, 4 s after).
"""
import argparse
import glob
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import power_calib as pc  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
US_PER_RAD = json.load(open(REPO / "pi_host/calib/sensors.json"))["servo"]["us_per_rad"]
JOINT_OF_CH = {s["ch"]: s["joint"] for s in json.load(open(REPO / "pi_host/calib/servo_map.json"))["servos"]}
TYPES = ("hip1", "hip2", "knee")


def swings_per_min(loaded, secs, minlen=4):
    n = 0
    for i in range(loaded.shape[1]):
        run = 0
        for v in loaded[:, i]:
            if not v:
                run += 1
            else:
                if run >= minlen:
                    n += 1
                run = 0
    return n / secs * 60


def robot(D: Path):
    feed = [json.loads(l) for l in open(D / "feed.jsonl")]
    ev = [json.loads(l) for l in open(D / "benchd_log.jsonl")]
    resets = [e["t_mono_ms"] for e in ev if e.get("kind") == "hat_reset"]
    ft = np.array([f["t"] for f in feed], float)
    out = np.array([f.get("out") or [0] * 12 for f in feed], float)
    armed = np.array([int(f.get("armed", 0)) for f in feed])
    drv = np.array([f.get("mode") in ("dev", "autonomous") and not f.get("stopped") for f in feed])
    tof = np.array([f["tof_m"] if f.get("tof_valid") else np.nan for f in feed])
    fsr = np.array([f["fsr"] for f in feed], float)
    ok = drv & ~(tof > 0.10)
    for r in resets:
        ok &= ~((ft > r - 1500) & (ft < r + 4000))
    secs = ok.sum() / 50.0
    # line speed: 100 ms = 5 frames at 50 Hz, only spans that are exactly that and fully ok
    spd = {t: [] for t in TYPES}
    for k in range(5, len(feed)):
        if not (ok[k] and ok[k - 5]) or abs(ft[k] - ft[k - 5] - 100) > 15:
            continue
        dt = (ft[k] - ft[k - 5]) / 1000
        for c in range(12):
            if (armed[k] >> c) & 1 and out[k, c] > 0 and out[k - 5, c] > 0:
                spd[JOINT_OF_CH[c]].append(abs(out[k, c] - out[k - 5, c]) / dt / US_PER_RAD)
    cap = sorted(glob.glob(str(D / "inacap_*_sag.csv")))[0]
    rows = np.loadtxt(cap, delimiter=",", comments="#", skiprows=2, dtype=np.int64)
    t_ms = rows[:, 0] / 1000.0
    amps = rows[:, 1] * 10e-6 / 0.01
    volts = (rows[:, 2] >> 3) * 4e-3
    a4 = rows[:, 3]
    fi = np.clip(np.searchsorted(ft, t_ms) - 1, 0, len(ft) - 1)
    okc = ok[fi]
    has = okc & (a4 > 0) & (volts > 5.0)
    rail = 3.3 * volts[has] / (a4[has] * 3.3 / 4095 * 3.0)
    loaded = fsr[ok] > 200
    return dict(secs=secs, resets=len(resets), spd=spd, cur_t=t_ms[okc], cur=amps[okc], rail_t=t_ms[has],
                rail=rail, duty=loaded.mean(), swings=swings_per_min(loaded, secs))


def sim(paths):
    spd = {t: [] for t in TYPES}
    cur, rail, cur_t, rail_t = [], [], [], []
    secs = 0.0
    duty, swings = [], []
    t_off = 0.0
    for p in paths:
        d = np.genfromtxt(p, delimiter=",", names=True)
        m = d["tick"] > 900                        # past spawn + settle, as arenaavg's warm-up
        w = np.stack([d[f"w{k}"] for k in range(12)], 1)[m]
        n = len(w) // 24                           # 100 ms spans at 240 Hz
        dang = np.abs(w[: n * 24].reshape(n, 24, 12).sum(1) / 240.0) / 0.1
        for jt, t in enumerate(TYPES):
            spd[t].extend(dang[:, jt::3].ravel().tolist())
        tt = np.arange(m.sum()) * 1000.0 / 240.0 + t_off
        cur.append(d["i_bat"][m]); cur_t.append(tt)
        rail.append(d["v_rail"][m]); rail_t.append(tt)
        t_off = tt[-1] + 1e6                       # keep runs apart for the excursion counter
        s = m.sum() / 240.0
        secs += s
        trace = os.path.join(os.path.dirname(p), "tr" + os.path.basename(p).replace(".csv", ".jsonl"))
        if os.path.exists(trace):
            R = [json.loads(l) for l in open(trace)]
            C = np.array([r["c"] for r in R if r["t"] > 900]) > 0
            duty.append(C.mean())
            swings.append(swings_per_min(C, len(C) / 50.0))
    return dict(secs=secs, resets=None, spd=spd, cur_t=np.concatenate(cur_t), cur=np.concatenate(cur),
                rail_t=np.concatenate(rail_t), rail=np.concatenate(rail),
                duty=np.mean(duty) if duty else np.nan, swings=np.mean(swings) if swings else np.nan)


def row(name, r):
    pk = pc.bins_10ms(r["cur_t"], r["cur"], np.fmax)
    mins = r["secs"] / 60
    s = {t: np.array(v) for t, v in r["spd"].items()}
    allv = np.concatenate(list(s.values()))
    return [name, f"{r['secs']:.0f}",
            f"{allv.mean():.2f}", f"{np.percentile(allv, 90):.2f}",
            *(f"{s[t].mean():.2f}" for t in TYPES),
            f"{r['cur'].mean():.2f}", f"{np.percentile(pk, 90):.2f}", f"{np.percentile(pk, 99):.2f}",
            f"{pc.excursions(pk, 3.0) / mins:.0f}",
            f"{pc.dips(r['rail_t'], r['rail'], 3.0) / mins:.2f}",
            f"{r['duty']:.2f}", f"{r['swings']:.0f}",
            "" if r["resets"] is None else str(r["resets"])]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True)
    ap.add_argument("--sim", action="append", default=[])
    a = ap.parse_args()
    hdr = ["", "s", "speed mean", "speed p90", "hip1", "hip2", "knee", "I mean", "pk p90", "pk p99",
           ">3A/min", "dips<3V/min", "duty", "swings/min", "resets"]
    rows = [row("robot " + Path(a.robot).name, robot(Path(a.robot)))]
    for spec in a.sim:
        label, _, files = spec.partition("=")
        rows.append(row("sim " + label, sim(files.split(","))))
    wid = [max(len(r[i]) for r in [hdr] + rows) for i in range(len(hdr))]
    for r in [hdr] + rows:
        print("  ".join(c.rjust(wid[i]) if i else c.ljust(wid[i]) for i, c in enumerate(r)))
    print("speed = joint line speed, rad/s (100 ms spans); robot = pulse on the line, sim = the joint itself")
