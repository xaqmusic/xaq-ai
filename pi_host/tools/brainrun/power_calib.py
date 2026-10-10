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
  per servo   I = I_IDLE + I_NL * min(1, |w| / W_NL) + min(I_STALL, K * g * |tau|), cut to I_CUT after
              STALL_CUT_S saturated
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
  battery     I_bat = min(I5, I5_LIM) * 5.0 / (ETA * V_pack) + I_LOGIC, V_pack = V_REST - R_PACK * I_bat
  rail        once I5 exceeds the regulator limit I5_LIM, the 5 V output folds back:
              V5 = 5.0 * (I5_LIM / I5) ** GAMMA, and the 3.3 V LDO follows it,
              rail = min(3.3, V5 - LDO_DROP)
"""
import json
import sys
from pathlib import Path

import numpy as np

# ---- electrical model parameters ----
# FIT 2026-10-06 on the asymmetric-cap body (OGMA_PICRAWLER_HONEST_TORQUE_CAP=0.25, _HOLD_CAP=0.6)
# with P-e·h0's full body_env (3.668 rad/s, solid chassis, honest joints/IMU), arena 0.3, 4 seeds x
# 12000, against the robot's carpet run (runrail, 391 s).  Anchors from the scale probe and
# SunFounder's SF006PRO: I_STALL 1.2 A at 5 V, holding ~free (H_HOLD 0), stall cut-off to <= 0.25 A
# after ~3.3 s.  The ONE fitted constant is K: the physical 1.2 A / 0.25 N m = 4.8 A/N m predicts
# ~2.7 A mean against the robot's 1.55, so K = 2.49 is fitted to the mean and the shape is the test:
# p90 2.22 vs 2.76 A, p99 2.81 vs 3.26, > 3 A 8 vs 87 per minute.  Too narrow.
# The sim port (scripts/servo_power_model.gd) mirrors THIS function step for step; change both.
P = dict(
    I_IDLE=0.005,      # A per servo, standby (SF006PRO <= 5 mA)
    I_NL=0.05,         # A per servo running free at W_NL (fit; SF006PRO peak no-load <= 0.35)
    W_NL=6.2,          # rad/s, SF006PRO no-load speed (0.17 s / 60 deg)
    I_STALL=1.2,       # A per servo, driving into a load it cannot move (scale probe; SF006PRO <= 1.2)
    K=2.49,            # A per N m of torque while driving (FIT, see above)
    H_HOLD=0.0,        # holding / back-driven share of K (scale probe: 0.12 N m held at no current)
    P_BLEND=0.02,      # W, mechanical power over which holding blends into driving
    TAU_MECH=0.05,     # s, the timescale driving vs holding is judged on
    TAU_TQ=0.02,       # s, servo torque bandwidth (the solver's step jitter is not a motor current)
    STALL_CUT_S=3.3,   # s saturated before the servo cuts its own drive (scale probe)
    I_CUT=0.25,        # A per servo after the cut (SF006PRO <= 250 mA)
    TAU_BUS=0.004,     # s, motor + bus smoothing of the summed current
    ETA=0.85,          # 5 V regulator efficiency
    V_REST=7.86,       # V, charged pack at zero HAT current (runrail, Pi included upstream)
    R_PACK=0.245,      # ohm, pack + wiring per amp of HAT current (runrail 0.247, runconc 0.239)
    I_LOGIC=0.10,      # A battery-side, HAT MCU + peripherals (robot idle reading)
    I5_LIM=4.05,       # A at 5 V: the regulator limit; puts the battery-side ceiling at the robot's
                       # ~3.5 A with the pack sagged to ~7.0 V
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
    """Per-step battery current, 3.3 V rail and servo-bus demand from per-joint torque (N m) and speed (rad/s)."""
    ts, ws = ema(tau, p["TAU_MECH"], dt), ema(w, p["TAU_MECH"], dt)
    tq = ema(tau, p["TAU_TQ"], dt) if p["TAU_TQ"] > 0 else tau
    x = np.clip(ts * ws / p["P_BLEND"], 0.0, 1.0)
    g = p["H_HOLD"] + (1.0 - p["H_HOLD"]) * x * x * (3.0 - 2.0 * x)
    i_drive = np.minimum(p["I_STALL"], p["K"] * g * np.abs(tq))
    run = np.zeros(tau.shape[1])
    for k in range(len(i_drive)):                       # the servo's own stall cut-off
        run = np.where(i_drive[k] >= 0.9 * p["I_STALL"], run + dt, 0.0)
        i_drive[k] = np.where(run > p["STALL_CUT_S"], np.minimum(i_drive[k], p["I_CUT"]), i_drive[k])
    i_servo = p["I_IDLE"] + p["I_NL"] * np.minimum(1.0, np.abs(w) / p["W_NL"]) + i_drive
    i5 = ema(i_servo.sum(1), p["TAU_BUS"], dt)
    i_bat = np.empty_like(i5)
    v = p["V_REST"]
    for k in range(len(i5)):                            # pack sag, one step behind (as the sim does)
        i_bat[k] = min(i5[k], p["I5_LIM"]) * 5.0 / (p["ETA"] * v) + p["I_LOGIC"]
        v = p["V_REST"] - p["R_PACK"] * i_bat[k]
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
    print(f"  servo-bus demand I5: mean {i5.mean():.2f} p99 {np.percentile(i5, 99):.2f} max {i5.max():.2f} A")
    return i_bat, rail, i5


if __name__ == "__main__":
    kind, paths = sys.argv[1], sys.argv[2:]
    for x in paths:
        (robot if kind == "robot" else sim)(Path(x))
