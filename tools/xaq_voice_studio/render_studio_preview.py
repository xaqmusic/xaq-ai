"""Render the studio to a PNG with no engine, no sim and no sound card.

The point is to iterate on layout, and to prove the window builds, without standing up
the whole stack — and to make "look at it before shipping" cheap enough to actually do.

It drives the real widgets with a synthetic engine reply and a synthetic meter stream, so
what it renders is what a connected studio renders.

    python tools/xaq_voice_studio/render_studio_preview.py /tmp/studio.png
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # puts tools/ on sys.path

from PyQt6.QtWidgets import QApplication                        # noqa: E402

from xaq_voice_studio.studio import StudioWindow                # noqa: E402

CAPS = {
    "engine": "xaq_voice", "version": 1,
    "waveforms": ["sine", "triangle", "saw", "square", "pulse", "noise_white", "noise_pink"],
    "filter_modes": ["bypass", "lowpass", "highpass", "bandpass", "notch", "vowel"],
    "destinations": ["pitch", "amp", "level", "cutoff", "resonance", "pulse_width",
                     "noise_mix", "vowel_morph", "pan", "detune"],
    "norm_modes": ["median_mad", "threshold_ratio", "minmax", "delta", "raw"],
    "triggers": ["rise", "fall", "true", "increase", "decrease"],
    "event_sounds": ["chirp_up", "chirp_down", "two_notes", "blip_down", "click", "none"],
    "vowels": ["A", "E", "I", "O", "U"],
    "scales": ["chromatic", "major_pentatonic", "minor_pentatonic", "major", "minor",
               "whole_tone", "octaves"],
}


def _route(module, key, dest, depth, mode="median_mad", ref="", curve=1.0):
    return {"source": {"module": module, "key": key}, "dest": dest,
            "norm": {"mode": mode, "z_lo": 0.0, "z_hi": 4.0, "ref_key": ref,
                     "gate": 1.4, "full": 2.0, "in_lo": 0.0, "in_hi": 1.0,
                     "smooth_ms": 60.0, "window_s": 10.0},
            "depth": depth, "curve": curve, "invert": False, "enabled": True}


def _voice(vid, base, wave, routes, events=(), fmode=None):
    # A real patch never stores "bypass" as the mode — FilterCfg defaults to lowpass and
    # `enabled` is the switch — so the preview must not invent one either, or the combo
    # would show a mode the patch does not contain.
    return {"id": vid, "module": vid, "enabled": True,
            "osc": {"waveform": wave, "pulse_width": 0.5, "noise_mix": 0.0,
                    "base_hz": base, "level": 1.0, "pan": 0.0, "glide_ms": 30.0,
                    "attack_ms": 20.0, "release_ms": 150.0, "quantize": -1},
            "filter": {"enabled": fmode is not None, "mode": fmode or "lowpass",
                       "cutoff_hz": 1800.0, "q": 2.0, "mix": 1.0,
                       "vowel_a": "A", "vowel_b": "E", "morph": 0.0},
            "routes": routes, "events": list(events)}


PATCH = {
    "version": 1, "engine": "xaq_voice",
    "brain": {"host": "127.0.0.1", "port": 7400, "hz": 30.0},
    "master": {"volume": 0.5, "quantize": True, "scale": "major_pentatonic", "span": 24.0,
               "filter": {"enabled": True, "mode": "vowel", "cutoff_hz": 4000.0, "q": 0.7,
                          "mix": 0.8, "vowel_a": "O", "vowel_b": "E", "morph": 0.3},
               "routes": [_route("voter", "fused_tle", "vowel_morph", 1.0)]},
    "voices": [
        _voice("motor_epm", 130.81, "square",
               [_route("motor_epm", "motor_tle", "pitch", 24.0),
                _route("motor_epm", "motor_tle", "amp", 1.0, "threshold_ratio", "", 0.5),
                _route("motor_epm", "gait_coherence", "cutoff", -18.0, "raw")]),
        _voice("body_pose", 261.63, "saw",
               [_route("body_pose", "last_tle", "pitch", 24.0),
                _route("body_pose", "last_tle", "amp", 1.0, "threshold_ratio",
                       "novelty_threshold_now", 0.5)],
               [{"source": {"module": "body_pose", "key": "baked_now"},
                 "trigger": "true", "sound": "chirp_up", "enabled": True},
                {"source": {"module": "body_pose", "key": "mitosis_count"},
                 "trigger": "increase", "sound": "two_notes", "enabled": True}],
               fmode="bandpass"),
        _voice("gain_evolver", 1046.5, "triangle",
               [_route("gain_evolver", "sigma", "pitch", 24.0)]),
    ],
}

SOURCES = [
    {"module": "motor_epm", "type": "MotorEPMv2", "frames": 300, "keys": [
        {"key": k, "is_bool": False, "value": v, "delta": 0.0, "median": v, "mad": 0.01,
         "min": v * 0.5, "max": v * 1.5, "mean": v, "var": 0.001, "seen": 300}
        for k, v in [("motor_tle", 0.041), ("gait_coherence", 0.43), ("couple_R", 0.31),
                     ("upright", 0.98), ("fwd_v", -0.006), ("boredom", 0.12),
                     ("hunger", 0.44), ("step_lock", 0.25)]]},
    {"module": "body_pose", "type": "EPM", "frames": 300, "keys": [
        {"key": k, "is_bool": b, "value": v, "delta": 0.0, "median": v, "mad": 0.01,
         "min": 0.0, "max": v * 2, "mean": v, "var": 0.002, "seen": 300}
        for k, v, b in [("last_tle", 0.132, False), ("ema_tle", 0.118, False),
                        ("novelty_threshold_now", 0.092, False), ("nodes", 35, False),
                        ("baked", 19, False), ("mitosis_count", 3, False),
                        ("baked_now", 0, True)]]},
    {"module": "gain_evolver", "type": "GainEvolver", "frames": 300, "keys": [
        {"key": k, "is_bool": False, "value": v, "delta": 0.0, "median": v, "mad": 0.02,
         "min": 0.0, "max": v * 2, "mean": v, "var": 0.01, "seen": 300}
        for k, v in [("sigma", 0.18), ("generation", 7), ("J_inc", 0.33), ("J_cand", 0.41),
                     ("accepts", 2), ("reverts", 5)]]},
    {"module": "voter", "type": "LateralVoter", "frames": 300, "keys": [
        {"key": k, "is_bool": False, "value": v, "delta": 0.0, "median": v, "mad": 0.01,
         "min": 0.0, "max": 1.0, "mean": v, "var": 0.001, "seen": 300}
        for k, v in [("fused_tle", 0.10), ("dopamine", 0.2), ("trust.body_pose", 8.4),
                     ("trust.motor_epm", 3.1)]]},
    {"module": "cpg_clock", "type": "CPGOscillator", "frames": 300, "keys": []},
    {"module": "keyframe_gait", "type": "KeyframeGait", "frames": 300, "keys": []},
]


class FakeClient:
    """Stands in for EngineClient.  Every call answers from the tables above."""

    def __init__(self, *a, **k):
        self.patch = PATCH

    def reconnect(self):    pass
    def close(self):        pass
    def hello(self):        return CAPS
    def get_patch(self):    return self.patch
    def get_sources(self):  return SOURCES
    def set_patch(self, p): self.patch = p
    def apply_ops(self, ops): pass
    def auto_patch(self, vary=False): return self.patch
    def save(self, path):   return path
    def load(self, path):   return self.patch
    def call(self, verb, **kw): return {"status": "ok"}


def fake_state(t: float) -> dict:
    """One meter frame, with everything moving so the bars are not all at zero."""
    def wob(i, lo=0.0, hi=1.0):
        return lo + (hi - lo) * (0.5 + 0.5 * math.sin(t * 1.7 + i))
    return {
        "voices": [
            {"id": "motor_epm", "hz": 130.81 * 2 ** (wob(0) * 1.2), "note": "D3",
             "amp": wob(1), "cutoff": 1800 * 2 ** (wob(2) - 0.5), "q": 2.0,
             "routes": [{"norm": wob(0), "out": wob(0) * 24},
                        {"norm": wob(1), "out": wob(1)},
                        {"norm": wob(2), "out": -wob(2) * 18}]},
            {"id": "body_pose", "hz": 392.0, "note": "G4", "amp": wob(3),
             "cutoff": 1800, "q": 2.0,
             "routes": [{"norm": wob(3), "out": wob(3) * 24},
                        {"norm": wob(4), "out": wob(4)}]},
            {"id": "gain_evolver", "hz": 1244.5, "note": "D#6", "amp": wob(5),
             "cutoff": 4000, "q": 0.7, "routes": [{"norm": wob(5), "out": wob(5) * 24}]},
        ],
        "master": {"routes": [{"norm": wob(6), "out": wob(6)}], "cutoff": 4000, "q": 0.7,
                   "vowel_morph": wob(6), "level": 1.0, "peak": wob(7, 0.2, 0.9),
                   "volume": 0.5, "muted": False},
        "sources": [{"module": m["module"], "type": m["type"],
                     "values": {k["key"]: k["value"] * (0.8 + 0.4 * wob(i + j))
                                for j, k in enumerate(m["keys"])}}
                    for i, m in enumerate(SOURCES)],
    }


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/xaq_voice_studio.png"
    app = QApplication(sys.argv[:1])

    import xaq_voice_studio.studio as studio_mod
    studio_mod.EngineClient = FakeClient

    win = StudioWindow("127.0.0.1", 7460)
    win.stream.start = lambda cb: None          # no ZMQ in the preview
    win.resize(1500, 950)
    win.show()

    # Let the deferred connect run, then push a few synthetic meter frames.
    for _ in range(6):
        app.processEvents()
    for i in range(24):
        win._on_state(fake_state(i * 0.25))
        win._apply_state()
        app.processEvents()

    win.grab().save(out)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
