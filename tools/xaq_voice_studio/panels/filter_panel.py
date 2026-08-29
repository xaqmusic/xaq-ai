"""A filter strip — the four ordinary modes and the vowel bank.

The vowel controls only appear in vowel mode and the cutoff/Q controls only outside it,
because they belong to different machines: the ordinary modes are one state-variable
filter with a cutoff, while `vowel` is three formant resonators whose frequencies come
from the vowel pair and the morph between them.  Showing both sets at once invites the
reasonable-but-wrong conclusion that cutoff moves a vowel.
"""
from __future__ import annotations

from typing import Callable

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QWidget

from ._controls import FloatSlider, LabeledCombo, check


class FilterPanel(QGroupBox):
    def __init__(self, title: str, base_path: str, cfg: dict, caps: dict,
                 on_set: Callable[[str, object], None], parent: QWidget | None = None):
        super().__init__(title, parent)
        self.base = base_path
        self.on_set = on_set
        self.caps = caps

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 6)
        lay.setSpacing(4)

        self.enabled = check("enabled", cfg.get("enabled", False),
                             lambda v: on_set(f"{self.base}/enabled", bool(v)))
        lay.addWidget(self.enabled)

        modes = [m for m in caps.get("filter_modes", []) if m != "bypass"]
        self.mode = LabeledCombo("mode", modes or ["lowpass"], cfg.get("mode", "lowpass"),
                                 label_w=52)
        self.mode.currentTextChanged.connect(self._on_mode)
        lay.addWidget(self.mode)

        self.cutoff = FloatSlider("cutoff", 20, 18000, cfg.get("cutoff_hz", 4000.0),
                                  log=True, unit=" Hz", decimals=0, label_w=52)
        self.cutoff.valueChanged.connect(lambda v: on_set(f"{self.base}/cutoff_hz", float(v)))
        lay.addWidget(self.cutoff)

        self.q = FloatSlider("Q", 0.35, 30.0, cfg.get("q", 0.7), log=True, label_w=52)
        self.q.valueChanged.connect(lambda v: on_set(f"{self.base}/q", float(v)))
        lay.addWidget(self.q)

        vowels = caps.get("vowels", ["A", "E", "I", "O", "U"])
        self.vowel_a = LabeledCombo("vowel A", vowels, cfg.get("vowel_a", "A"), label_w=52)
        self.vowel_a.currentTextChanged.connect(lambda v: on_set(f"{self.base}/vowel_a", v))
        lay.addWidget(self.vowel_a)

        self.vowel_b = LabeledCombo("vowel B", vowels, cfg.get("vowel_b", "E"), label_w=52)
        self.vowel_b.currentTextChanged.connect(lambda v: on_set(f"{self.base}/vowel_b", v))
        lay.addWidget(self.vowel_b)

        self.morph = FloatSlider("morph", 0.0, 1.0, cfg.get("morph", 0.0), label_w=52)
        self.morph.setToolTip("A → B.  Route a source here to make the mouth move.")
        self.morph.valueChanged.connect(lambda v: on_set(f"{self.base}/morph", float(v)))
        lay.addWidget(self.morph)

        self.mix = FloatSlider("mix", 0.0, 1.0, cfg.get("mix", 1.0), label_w=52)
        self.mix.valueChanged.connect(lambda v: on_set(f"{self.base}/mix", float(v)))
        lay.addWidget(self.mix)

        self._sync(cfg.get("mode", "lowpass"))

    def set_caps(self, caps: dict) -> None:
        """Refill the lists that come from the engine.

        The panel is built before the engine has said what it supports, so its combos
        start on a one-item fallback.  Without this the combo silently refuses a
        setCurrentText for a mode it does not list, and the panel ends up showing
        'lowpass' while displaying the vowel controls — the visible state and the patch
        disagreeing, which is the worst kind of UI bug because it looks like it worked.
        """
        self.caps = caps
        modes = [m for m in caps.get("filter_modes", []) if m != "bypass"]
        if modes:
            self.mode.set_items(modes, self.mode.value())
        vowels = caps.get("vowels", [])
        if vowels:
            self.vowel_a.set_items(vowels, self.vowel_a.value())
            self.vowel_b.set_items(vowels, self.vowel_b.value())

    def _on_mode(self, mode: str) -> None:
        self.on_set(f"{self.base}/mode", mode)
        self._sync(mode)

    def _sync(self, mode: str) -> None:
        vowel = mode == "vowel"
        for w in (self.vowel_a, self.vowel_b, self.morph):
            w.setVisible(vowel)
        for w in (self.cutoff, self.q):
            w.setVisible(not vowel)

    def sync_from(self, cfg: dict) -> None:
        self.enabled.blockSignals(True)
        self.enabled.setChecked(bool(cfg.get("enabled", False)))
        self.enabled.blockSignals(False)
        mode = cfg.get("mode", "lowpass")
        self.mode.set_value(mode)
        # A combo cannot select a mode it does not list, and a silent refusal here would
        # leave the shown mode disagreeing with the patch.  Say so instead.
        if self.mode.value() != mode:
            self.mode.set_items(sorted({mode, *(self.caps.get("filter_modes") or [])}
                                       - {"bypass"}), mode)
        self.cutoff.set_value(cfg.get("cutoff_hz", 4000.0))
        self.q.set_value(cfg.get("q", 0.7))
        self.vowel_a.set_value(cfg.get("vowel_a", "A"))
        self.vowel_b.set_value(cfg.get("vowel_b", "E"))
        self.morph.set_value(cfg.get("morph", 0.0))
        self.mix.set_value(cfg.get("mix", 1.0))
        self._sync(cfg.get("mode", "lowpass"))
