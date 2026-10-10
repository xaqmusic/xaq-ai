#!/usr/bin/env python3
"""power_parity.py <power_log.csv> — does the sim's electrical model compute what power_calib.py computes?

The sim (scripts/servo_power_model.gd, electrical_step) is a step-for-step port of
power_calib.electrical().  A port is the same NUMBERS only if it is the same MATHS, so this re-runs
the Python model on the log's own torque and speed columns and compares the sim's logged i_bat,
v_pack, v_rail and i5, row by row.  The log rounds tau to 1e-5 and w to 1e-4, so agreement is
judged against that, not to the last bit.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import power_calib as pc  # noqa: E402

d = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
tau = np.stack([d[f"tau{k}"] for k in range(12)], 1)
w = np.stack([d[f"w{k}"] for k in range(12)], 1)
i_bat, rail, i5 = pc.electrical(tau, w, 1.0 / float(sys.argv[2]) if len(sys.argv) > 2 else 1.0 / 240)
v_pack = pc.P["V_REST"] - pc.P["R_PACK"] * i_bat
ok = True
for name, py, gd, tol in (("i_bat", i_bat, d["i_bat"], 2e-3), ("i5", i5, d["i5"], 2e-3),
                          ("v_pack", v_pack, d["v_pack"], 1e-3), ("v_rail", rail, d["v_rail"], 1e-3)):
    err = np.abs(py - gd)
    bad = err > tol
    ok &= not bad.any()
    print(f"{name:7} max |py - sim| {err.max():.2e}  (tol {tol:.0e})  rows over tol {bad.sum()} / {len(err)}"
          f"   sim mean {gd.mean():.3f}  py mean {py.mean():.3f}")
print("PARITY OK" if ok else "PARITY FAILED")
sys.exit(0 if ok else 1)
