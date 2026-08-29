"""One row of the mod matrix: source · destination · normalisation · depth · curve.

This is the widget the whole studio exists to offer, so it is worth saying what each
control is for.

* **source** — any signal any module publishes, addressed ``module.key``.
* **dest** — what it moves.  The depth unit changes with it (semitones for pitch and
  cutoff, Q for resonance, 0..1 for the rest), and the label follows, because a depth of
  24 means two octaves in one case and a nonsense value in another.
* **norm** — how the source's own range is mapped onto 0..1 *before* depth applies.  This
  is the control that makes a slider mean the same thing for ``last_tle`` at 0.1 and
  ``nodes`` at 400, and it is the one an operator will reach for when a route "does
  nothing": the source is probably pinned at an end of its range under the current mode.
* **curve** — ``x^curve``.  Below 1 opens up the quiet end, which is usually what a spiky
  error signal needs to be expressive rather than binary.
* the **meter** shows the route's live normalised contribution, so a route that is doing
  nothing is visibly doing nothing rather than merely inaudible.
"""
from __future__ import annotations

from typing import Callable

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout,
                             QWidget)

from ..theme import INK_MUTED, LINE, SURFACE_ALT, SurfaceWidget, dest_color
from ._controls import FloatSlider, LabeledCombo, MeterBar, check, muted

# Sensible full-scale travel per destination, in that destination's own unit.  A depth
# slider whose range is wrong is worse than no slider: the useful part of the travel ends
# up in the first two pixels.
DEPTH_RANGE = {
    "pitch":       (-48.0, 48.0),
    "detune":      (-24.0, 24.0),
    "cutoff":      (-72.0, 72.0),
    "resonance":   (-20.0, 20.0),
    "amp":         (-2.0, 2.0),
    "level":       (-2.0, 2.0),
    "pulse_width": (-1.0, 1.0),
    "noise_mix":   (-1.0, 1.0),
    "vowel_morph": (-1.0, 1.0),
    "pan":         (-1.0, 1.0),
}
DEPTH_UNIT = {
    "pitch": " st", "detune": " st", "cutoff": " st", "resonance": " Q",
}


class RouteRow(SurfaceWidget):
    """Edits one route in place.  Paths are JSON pointers into the patch."""

    removeRequested = pyqtSignal(str)      # the route's base path

    def __init__(self, base_path: str, route: dict, caps: dict,
                 sources: list[str], on_set: Callable[[str, object], None],
                 parent: QWidget | None = None):
        super().__init__(parent, SURFACE_ALT)
        self.base = base_path
        self.on_set = on_set
        self.caps = caps

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 6, 8, 6)
        outer.setSpacing(4)

        src = route.get("source") or {}
        src_label = f"{src.get('module', '')}.{src.get('key', '')}".strip(".")
        dest = route.get("dest", "pitch")
        norm = route.get("norm") or {}

        # -- top line: source, destination, enable, remove --
        top = QHBoxLayout()
        top.setSpacing(6)

        self.chip = QLabel("  ")
        self.chip.setFixedWidth(8)
        self.chip.setStyleSheet(f"background: {dest_color(dest)}; border-radius: 2px;")
        top.addWidget(self.chip)

        self.source = LabeledCombo("", sorted(set(sources) | {src_label}), src_label, label_w=0)
        self.source.currentTextChanged.connect(self._on_source)
        top.addWidget(self.source, 3)

        self.dest = LabeledCombo("→", caps.get("destinations", [dest]), dest, label_w=14)
        self.dest.currentTextChanged.connect(self._on_dest)
        top.addWidget(self.dest, 2)

        self.enabled = check("", route.get("enabled", True),
                             lambda v: self.on_set(f"{self.base}/enabled", bool(v)))
        self.enabled.setToolTip("Route enabled")
        top.addWidget(self.enabled)

        rm = QPushButton("✕")
        rm.setFixedWidth(26)
        rm.setToolTip("Remove this route")
        rm.clicked.connect(lambda: self.removeRequested.emit(self.base))
        top.addWidget(rm)
        outer.addLayout(top)

        # -- second line: normalisation --
        self.norm = LabeledCombo("norm", caps.get("norm_modes", []),
                                 norm.get("mode", "median_mad"), label_w=44)
        self.norm.currentTextChanged.connect(self._on_norm_mode)
        outer.addWidget(self.norm)

        # The knobs that belong to the chosen normalisation, shown only when they apply —
        # a gate slider under a median/MAD route is a control that does nothing, and a
        # control that does nothing is how an hour gets lost.
        self.norm_box = QWidget()
        self.norm_grid = QGridLayout(self.norm_box)
        self.norm_grid.setContentsMargins(0, 0, 0, 0)
        self.norm_grid.setSpacing(3)
        outer.addWidget(self.norm_box)

        self.z_lo  = FloatSlider("z from", -4, 8, norm.get("z_lo", 0.0), label_w=44)
        self.z_hi  = FloatSlider("z to",   -4, 12, norm.get("z_hi", 4.0), label_w=44)
        self.gate  = FloatSlider("gate",   0.5, 6, norm.get("gate", 1.4), unit="×", label_w=44)
        self.full  = FloatSlider("full",   0.5, 12, norm.get("full", 2.0), unit="×", label_w=44)
        self.in_lo = FloatSlider("in lo", -100, 100, norm.get("in_lo", 0.0), label_w=44)
        self.in_hi = FloatSlider("in hi", -100, 1000, norm.get("in_hi", 1.0), label_w=44)
        self.smooth = FloatSlider("smooth", 0, 500, norm.get("smooth_ms", 60.0),
                                  unit=" ms", decimals=0, label_w=44)
        self.ref = LabeledCombo("ref", [""], norm.get("ref_key", ""), label_w=44)
        self.ref.currentTextChanged.connect(
            lambda v: self.on_set(f"{self.base}/norm/ref_key", v))

        for w, key in ((self.z_lo, "z_lo"), (self.z_hi, "z_hi"), (self.gate, "gate"),
                       (self.full, "full"), (self.in_lo, "in_lo"), (self.in_hi, "in_hi"),
                       (self.smooth, "smooth_ms")):
            w.valueChanged.connect(
                lambda v, k=key: self.on_set(f"{self.base}/norm/{k}", float(v)))

        for w in (self.z_lo, self.z_hi, self.gate, self.full, self.in_lo, self.in_hi,
                  self.ref, self.smooth):
            self.norm_grid.addWidget(w, self.norm_grid.rowCount(), 0)

        # -- third line: depth, curve, invert, meter --
        lo, hi = DEPTH_RANGE.get(dest, (-1.0, 1.0))
        self.depth = FloatSlider("depth", lo, hi, route.get("depth", 1.0),
                                 unit=DEPTH_UNIT.get(dest, ""), label_w=44)
        self.depth.valueChanged.connect(
            lambda v: self.on_set(f"{self.base}/depth", float(v)))
        outer.addWidget(self.depth)

        row = QHBoxLayout()
        row.setSpacing(6)
        self.curve = FloatSlider("curve", 0.1, 4.0, route.get("curve", 1.0), label_w=44)
        self.curve.valueChanged.connect(
            lambda v: self.on_set(f"{self.base}/curve", float(v)))
        row.addWidget(self.curve, 3)
        self.invert = check("inv", route.get("invert", False),
                            lambda v: self.on_set(f"{self.base}/invert", bool(v)))
        self.invert.setToolTip("Flip the normalised value: 1 − x")
        row.addWidget(self.invert)
        outer.addLayout(row)

        mrow = QHBoxLayout()
        mrow.setSpacing(6)
        mrow.addWidget(muted("out"))
        self.meter = MeterBar(dest_color(dest))
        mrow.addWidget(self.meter, 1)
        self.out_label = QLabel("0.00")
        self.out_label.setObjectName("value")
        self.out_label.setMinimumWidth(52)
        self.out_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        mrow.addWidget(self.out_label)
        outer.addLayout(mrow)

        self.setStyleSheet(f"QWidget {{ border-radius: 4px; }} "
                           f"RouteRow {{ border: 1px solid {LINE}; }}")
        self._sync_norm_visibility(norm.get("mode", "median_mad"))

    # -- available reference keys, filled in by the voice panel --

    def set_ref_keys(self, keys: list[str], current: str) -> None:
        self.ref.set_items([""] + keys, current)

    def set_sources(self, sources: list[str]) -> None:
        cur = self.source.value()
        self.source.set_items(sorted(set(sources) | {cur}), cur)

    # -- edits --

    def _on_source(self, label: str) -> None:
        module, _, key = label.partition(".")
        self.on_set(f"{self.base}/source/module", module)
        self.on_set(f"{self.base}/source/key", key)

    def _on_dest(self, dest: str) -> None:
        self.on_set(f"{self.base}/dest", dest)
        self.chip.setStyleSheet(f"background: {dest_color(dest)}; border-radius: 2px;")
        self.meter.set_color(dest_color(dest))
        # Re-range the depth slider for the new unit, keeping the value if it still fits.
        lo, hi = DEPTH_RANGE.get(dest, (-1.0, 1.0))
        v = self.depth.value()
        self.depth.lo, self.depth.hi = lo, hi
        self.depth.unit = DEPTH_UNIT.get(dest, "")
        self.depth.set_value(min(max(v, lo), hi))

    def _on_norm_mode(self, mode: str) -> None:
        self.on_set(f"{self.base}/norm/mode", mode)
        self._sync_norm_visibility(mode)

    def _sync_norm_visibility(self, mode: str) -> None:
        show = {
            "median_mad":      (self.z_lo, self.z_hi, self.smooth),
            "delta":           (self.z_lo, self.z_hi, self.smooth),
            "threshold_ratio": (self.ref, self.gate, self.full, self.smooth),
            "minmax":          (self.smooth,),
            "raw":             (self.in_lo, self.in_hi, self.smooth),
        }.get(mode, (self.smooth,))
        for w in (self.z_lo, self.z_hi, self.gate, self.full, self.in_lo, self.in_hi,
                  self.ref, self.smooth):
            w.setVisible(w in show)

    # -- live --

    def update_meter(self, norm_value: float, out_value: float) -> None:
        self.meter.set_value(abs(norm_value))
        self.out_label.setText(f"{out_value:+.2f}")
