#!/usr/bin/env python3
"""power_calib.py — the robot's power statistics, as calibration targets for the sim's model.

    power_calib.py robot <dashrun_dir> [...]   robot stats from a fast INA capture + 50 Hz feed
    power_calib.py sim <power_log.csv> [...]   the same stats from the sim's servo power instrument
                                               (OGMA_PICRAWLER_POWER_LOG), through the electrical
                                               model below

Both are reduced to the same numbers, so they can be read side by side:
  current     battery-side HAT current (servos + HAT logic): mean, and 10 ms peaks p50 to p99.9
  excursions  10 ms peaks above 3 / 4 A, per minute
  rail        3.3 V rail dips below 3.0 / 2.8 / 2.5 V, per minute (robot: 3.3 x pack V / A4)
  by feet     mean current by the number of feet loaded

Robot side: brain-driven frames only, robot on the floor (belly ToF <= 100 mm), with 1.5 s before
and 4 s after every HAT reset excluded, as in fast_current_analysis.py §2.

Sim side, the electrical model (power-budget plan S1, 2026-10-05):
  per servo   I = I_IDLE + I_NL * min(1, |w| / W_NL) + I_STALL * min(1, g * |tau| / TAU_STALL)
              A DC motor's current follows its torque, and a real MG90S cannot deliver more than
              its stall torque, so demand above it is read as a stall at I_STALL.
              g is the gear train's share.  MOTORING (tau * w > 0, the servo doing work) g = 1, the
              datasheet case.  HOLDING or being back-driven, gear friction carries part of the load,
              and the motor needs about (2 eta - 1) of the driving current: ~0.3 at a typical
              spur-train eta ~0.65.  g = H_HOLD + (1 - H_HOLD) * smoothstep(tau * w / P_BLEND),
              with tau and w smoothed over TAU_MECH: vibration at the solver rate, through gear
              backlash, does not turn the motor, so it is not work.
              H_HOLD is the one fitted constant of the servo layer.
  servo bus   I5 = sum over 12 servos, smoothed by the motor and bus capacitance (TAU_BUS)
  battery     I_bat = I5 * 5.0 / (ETA * V_PACK) + I_LOGIC, capped at the regulator limit
  rail        once I5 exceeds the regulator limit I5_LIM, the 5 V output folds back:
              V5 = 5.0 * (I5_LIM / I5) ** GAMMA, and the 3.3 V LDO follows it,
              rail = min(3.3, V5 - LDO_DROP)
"""
import json
import sys
from pathlib import Path

import numpy as np

# ---- electrical model parameters (MG90S datasheet class at 5 V; the rest are fit targets) ----
P = dict(
    I_IDLE=0.010,      # A per servo, holding with no load
    I_NL=0.15,         # A per servo, running free at W_NL
    W_NL=10.5,         # rad/s, MG90S no-load speed (0.1 s / 60 deg)
    I_STALL=0.70,      # A per servo at stall
    TAU_STALL=0.18,    # N m, MG90S stall torque (1.8 kg cm)
    H_HOLD=0.30,       # holding / back-driven current share (2 eta - 1); FIT to the robot's means
    P_BLEND=0.02,      # W, mechanical power over which holding blends into motoring
    TAU_MECH=0.05,     # s, the timescale motoring is judged on (servo loop + gear backlash)
    TAU_BUS=0.004,     # s, motor + bus smoothing of the summed current
    ETA=0.85,          # 5 V regulator efficiency
    V_PACK=7.6,        # V, 2S pack under load
    I_LOGIC=0.10,      # A battery-side, HAT MCU + peripherals
    I5_LIM=4.4,        # A at 5 V: the regulator limit (~3.5 A battery-side at 7.6 V, ETA 0.85)
    GAMMA=1.0,         # fold-back steepness above the limit
    LDO_DROP=0.25,     # V
)


def bins_10ms(t_ms, x, fn):
    b = ((t_ms - t_ms[0]) // 10).astype(int)
    out = np.full(b.max() + 1, np.nan)
    fn.at(out, b, x)
    return out[np.isfinite(out)]


def excursions(peaks, th):
    a = peaks > th
    return int(np.sum(a & ~np.r_[False, a[:-1]]))


def dips(t_ms, rail, th, join_ms=30):
    """Episodes with the rail below th, joined when closer than join_ms."""
    n, last = 0, -1e18
    for t in t_ms[rail < th]:
        if t - last >= join_ms:
            n += 1
        last = t
    return n


def report(name, secs, cur_t, cur, rail_t, rail, loaded=None, cur50=None):
    pk = bins_10ms(cur_t, cur, np.fmax)
    mins = secs / 60
    print(f"\n=== {name}: {secs:.0f} s")
    print(f"  current mean {cur.mean():.2f} A | 10 ms peaks p50 {np.percentile(pk, 50):.2f} p90 {np.percentile(pk, 90):.2f} "
          f"p99 {np.percentile(pk, 99):.2f} p99.9 {np.percentile(pk, 99.9):.2f} max {pk.max():.2f} A")
    print(f"  excursions per min: >3 A {excursions(pk, 3.0) / mins:6.1f}   >4 A {excursions(pk, 4.0) / mins:6.1f}")
    if rail is not None and len(rail):
        print(f"  rail dips per min:  <3.0 V {dips(rail_t, rail, 3.0) / mins:5.2f}   <2.8 V {dips(rail_t, rail, 2.8) / mins:5.2f}"
              f"   <2.5 V {dips(rail_t, rail, 2.5) / mins:5.2f}")
    if loaded is not None:
        parts = []
        for n in range(5):
            m = loaded == n
            if m.sum() > 50:
                parts.append(f"{n}: {np.mean(cur50[m]):.2f}")
        print("  mean current by feet loaded: " + "  ".join(parts))


def robot(D: Path):
    cap = next(D.glob("inacap_*_sag.csv"))
    rows = np.loadtxt(cap, delimiter=",", comments="#", skiprows=2, dtype=np.int64)
    t_ms = rows[:, 0] / 1000.0
    amps = rows[:, 1] * 10e-6 / 0.01
    volts = (rows[:, 2] >> 3) * 4e-3
    a4 = rows[:, 3] if rows.shape[1] > 3 else np.full(len(rows), -1)   # pre-rail captures
    feed = [json.loads(l) for l in open(D / "feed.jsonl")]
    evp = next(p for p in D.glob("*events.jsonl"))
    ev = [json.loads(l) for l in open(evp)]
    ft = np.array([f["t"] for f in feed], float)
    tof = np.array([f["tof_m"] if f.get("tof_valid") else np.nan for f in feed])
    driven = np.array([f.get("mode") in ("dev", "autonomous") and not f.get("stopped") for f in feed])
    fsr = np.array([f["fsr"] for f in feed], float)
    fi = np.clip(np.searchsorted(ft, t_ms) - 1, 0, len(ft) - 1)
    ok = driven[fi] & ~(tof[fi] > 0.10)
    ok50 = driven & ~(tof > 0.10)
    for e in ev:
        if e["kind"] == "hat_reset":
            ok &= ~((t_ms > e["t_mono_ms"] - 1500) & (t_ms < e["t_mono_ms"] + 4000))
            ok50 &= ~((ft > e["t_mono_ms"] - 1500) & (ft < e["t_mono_ms"] + 4000))
    secs = ok.sum() * np.median(np.diff(t_ms)) / 1000
    has = ok & (a4 > 0) & (volts > 5.0)
    rail = 3.3 * volts[has] / (a4[has] * 3.3 / 4095 * 3.0)
    i50 = np.array([f["i_a"] if f["i_a"] is not None else np.nan for f in feed])
    m50 = ok50 & np.isfinite(i50)
    resets = sum(e["kind"] == "hat_reset" for e in ev)
    report(f"robot {D.name} ({resets} HAT resets in the record)", secs, t_ms[ok], amps[ok], t_ms[has], rail,
           (fsr[m50] > 200).sum(1), i50[m50])


def ema(x, tc, dt):
    a = dt / (tc + dt)
    y = np.empty_like(x)
    acc = x[0].copy() if np.ndim(x) > 1 else x[0]
    for k in range(len(x)):
        acc = acc + a * (x[k] - acc)
        y[k] = acc
    return y


def electrical(tau, w, dt, p=P):
    """Per-step battery current and rail from per-joint torque (N m) and speed (rad/s)."""
    ts, ws = ema(tau, p["TAU_MECH"], dt), ema(w, p["TAU_MECH"], dt)
    x = np.clip(ts * ws / p["P_BLEND"], 0.0, 1.0)
    g = p["H_HOLD"] + (1.0 - p["H_HOLD"]) * x * x * (3.0 - 2.0 * x)
    i_servo = (p["I_IDLE"] + p["I_NL"] * np.minimum(1.0, np.abs(w) / p["W_NL"])
               + p["I_STALL"] * np.minimum(1.0, g * np.abs(tau) / p["TAU_STALL"]))
    i5_raw = i_servo.sum(1)
    i5 = ema(i5_raw, p["TAU_BUS"], dt)
    i5_del = np.minimum(i5, p["I5_LIM"])
    i_bat = i5_del * 5.0 / (p["ETA"] * p["V_PACK"]) + p["I_LOGIC"]
    v5 = np.where(i5 > p["I5_LIM"], 5.0 * (p["I5_LIM"] / np.maximum(i5, 1e-9)) ** p["GAMMA"], 5.0)
    rail = np.minimum(3.3, v5 - p["LDO_DROP"])
    return i_bat, rail, i5


def sim(csv: Path, hz=240.0, p=P):
    d = np.genfromtxt(csv, delimiter=",", names=True)
    tau = np.stack([d[f"tau{k}"] for k in range(12)], 1)
    w = np.stack([d[f"w{k}"] for k in range(12)], 1)
    load = np.stack([d[f"load{i}"] for i in range(4)], 1)
    m = d["tick"] > 300                      # past the spawn settle and calibration hold
    i_bat, rail, i5 = electrical(tau[m], w[m], 1.0 / hz, p)
    t_ms = np.arange(m.sum()) * 1000.0 / hz
    # feet loaded: the sim's normalised foot load against the FSR's ~0.1-of-weight threshold
    report(f"sim {csv.name}", m.sum() / hz, t_ms, i_bat, t_ms, rail, (load[m] > 0.10).sum(1), i_bat)
    print(f"  servo-bus demand I5: mean {i5.mean():.2f} p99 {np.percentile(i5, 99):.2f} max {i5.max():.2f} A; "
          f"servo torque beyond MG90S stall {np.mean(np.abs(tau[m]) > p['TAU_STALL']) * 100:.1f} % of joint-steps")
    return i_bat, rail, i5


if __name__ == "__main__":
    kind, paths = sys.argv[1], sys.argv[2:]
    for x in paths:
        (robot if kind == "robot" else sim)(Path(x))
