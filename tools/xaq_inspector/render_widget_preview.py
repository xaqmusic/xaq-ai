#!/usr/bin/env python3
"""Render the GainEvolver inspector against a synthetic search so the layout can
be LOOKED AT (dataviz step 7) without waiting on a live brain."""
import math, os, random, sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))

from PyQt6.QtWidgets import QApplication
from xaq_inspector.widgets import widget_for, wrap_with_description

KEYS = ["rear_land_gain", "rear_knee_plant", "rear_push_ext", "amp_target",
        "height_homeo_gain", "postural_gain", "coupling_gain", "plan_gain"]
GMIN = [0.0, 0.0, 0.0, 0.15, 0.0, 0.1, 0.0, 0.0]
GMAX = [1.5, 0.6, 1.5, 0.8, 0.1, 1.5, 2.0, 0.2]   # coupling max 2.0 = the shipped bound
SEED = [0.0, 0.2, 0.0, 0.4, 0.0, 0.3, 0.0, 0.0]
WINDOW = 4000
WITH_BOUNDS = "--nobounds" not in sys.argv   # simulate old .so when absent

app = QApplication([])
cls = widget_for("GainEvolver")
print("widget class ->", cls.__name__)
inner = cls("gain_evolver", "GainEvolver")
w = wrap_with_description(inner, "GainEvolver")
w.resize(1400, 1150)   # the real inspector right-pane size
w.show()

rng = random.Random(7)
inc = list(SEED)
cand = list(SEED)
log = ""
acc = rev = pub = 0
J_inc, J_cand = 0.62, -1.0

# Simulate ~22 generations; the vector drifts toward the hand point and J falls.
HAND = [0.5, 0.2, 0.5, 0.4, 0.04, 0.7, 1.55, 0.05]
for gen in range(22):
    for phase in (1, 2):
        if phase == 2:
            cand = [min(GMAX[i], max(GMIN[i],
                    inc[i] + 0.08 * (GMAX[i] - GMIN[i]) * rng.gauss(0, 1)))
                    for i in range(8)]
        for step in range(0, WINDOW + 1, 400):
            snap = {
                "generation": gen, "accepts": acc, "reverts": rev, "publishes": pub,
                "sigma": 0.08 * (0.97 ** gen), "phase": phase, "win_tick": step,
                "eval_window_ticks": WINDOW,
                "J_inc": J_inc, "J_cand": J_cand if phase == 2 else -1.0,
                "falls": max(0.0, 2.0 - gen * 0.12), "tilt_sd": 0.09 * (0.97 ** gen),
                "distress_duty": 0.0,                       # weight 0 => "off", not "dead"
                "unloaded_mean": max(0.02, 0.42 - gen * 0.017),
                "flow_term": max(0.25, 0.90 - gen * 0.028),
                "loaded_min": min(0.95, 0.30 + gen * 0.03),
                "energy": 0.386 - gen * 0.0009, "fall_alarm": 3.2 + gen*0.1,
                "alarm_on": 1 if gen > 16 else 0, "sigma_est": 0.031, "accept_margin": 0.031, "noise_n": 6,
                "gain_keys": KEYS, "incumbent": inc, "candidate": cand,
                "accept_log": log,
                "vec": cand if phase == 2 else inc,
            }
            if WITH_BOUNDS:
                snap |= {"gain_min": GMIN, "gain_max": GMAX, "gain_seed": SEED,
                         "weights": {"falls": 0.0, "tilt_sd": 1.0, "distress_duty": 0.0,
                                     "unloaded_mean": 1.0, "flow_term": 1.0, "energy": 8.0},
                         "sigma_est": 0.031, "accept_margin": 0.031, "noise_n": 6}
            w.update_payload(gen * 10000 + step, snap)
            app.processEvents()
        if phase == 2:
            better = rng.random() < 0.45
            J_cand = J_inc - (0.03 if better else -0.02)
            if better:
                inc = list(cand); acc += 1; log += "A"; J_inc = J_cand
            else:
                rev += 1; log += "R"
            pub += 2

for _ in range(40):
    app.processEvents()
out = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") \
    else "/tmp/gain_widget.png"
w.grab().save(out)
print("wrote", out)
