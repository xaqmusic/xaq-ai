"""The right pane: the master bus, its mod rack, and the transport buttons.

The master rack is where the output filter stops being a static shape.  Routing a signal
that belongs to no single voice — consensus surprise, urgency, dopamine — into
``vowel_morph`` makes the whole mix speak, and that reads as the brain's mood rather than
as any one module's opinion.
"""
from __future__ import annotations

from typing import Callable

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QGroupBox, QHBoxLayout, QLabel, QPushButton, QScrollArea,
                             QVBoxLayout, QWidget)

from ..theme import GOOD, INK_PRIMARY
from ._controls import (FloatSlider, LabeledCombo, MeterBar, check, heading, hline, muted)
from .filter_panel import FilterPanel
from .route_row import RouteRow


class MasterPanel(QWidget):
    def __init__(self, model, caps: dict, on_dirty: Callable[[], None],
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.model = model
        self.caps = caps
        self.on_dirty = on_dirty
        self._rows: list[RouteRow] = []
        self._built_n = -1
        self._sources: list[str] = []

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 8, 8, 8)
        lay.setSpacing(6)
        lay.addWidget(heading("Master"))

        meter_row = QHBoxLayout()
        meter_row.addWidget(muted("out"))
        self.meter = MeterBar(GOOD)
        meter_row.addWidget(self.meter, 1)
        lay.addLayout(meter_row)

        self.volume = FloatSlider("volume", 0.0, 1.0, 0.5, label_w=60)
        self.volume.valueChanged.connect(lambda v: self._set("/master/volume", float(v)))
        lay.addWidget(self.volume)

        row = QHBoxLayout()
        self.mute = QPushButton("mute")
        self.mute.setCheckable(True)
        row.addWidget(self.mute)
        self.tone = QPushButton("tone")
        self.tone.setCheckable(True)
        self.tone.setChecked(True)
        row.addWidget(self.tone)
        lay.addLayout(row)

        self.quantize = check("quantise pitch", True,
                              lambda v: self._set("/master/quantize", bool(v)))
        lay.addWidget(self.quantize)

        self.scale = LabeledCombo("scale", caps.get("scales", ["major_pentatonic"]),
                                  "major_pentatonic", label_w=60)
        self.scale.currentTextChanged.connect(lambda v: self._set("/master/scale", v))
        lay.addWidget(self.scale)

        self.mod_smooth = FloatSlider("mod glide", 0, 250, 25.0, unit=" ms", decimals=0,
                                      label_w=60)
        self.mod_smooth.setToolTip(
            "How fast cutoff, resonance, vowel morph, width, noise, pan and level chase\n"
            "their targets.  Diag frames land at ~30 Hz, so without this they move in\n"
            "33 ms stair-steps — audible as zipper noise on a sweep and as a lurching\n"
            "vowel.  Pitch and amplitude are not affected; they have glide and\n"
            "attack/release per voice.  0 restores the stair.")
        self.mod_smooth.valueChanged.connect(
            lambda v: self._set("/master/mod_smooth_ms", float(v)))
        lay.addWidget(self.mod_smooth)

        self.filter_panel = FilterPanel("output filter", "/master/filter", {}, caps,
                                        self._set)
        lay.addWidget(self.filter_panel)

        rack_head = QHBoxLayout()
        rack_head.addWidget(heading("Master modulation"))
        rack_head.addStretch(1)
        add = QPushButton("+ route")
        add.clicked.connect(self._add_route)
        rack_head.addWidget(add)
        lay.addLayout(rack_head)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.rack = QWidget()
        self.rack_lay = QVBoxLayout(self.rack)
        self.rack_lay.setContentsMargins(0, 0, 6, 0)
        self.rack_lay.setSpacing(6)
        self.rack_lay.addStretch(1)
        scroll.setWidget(self.rack)
        lay.addWidget(scroll, 1)

        lay.addWidget(hline())
        self.status = muted("not connected")
        self.status.setWordWrap(True)
        lay.addWidget(self.status)

    # ------------------------------------------------------------------ helpers
    def _set(self, path: str, value) -> None:
        self.model.set(path, value)
        self.on_dirty()

    def set_sources(self, sources: list[str]) -> None:
        self._sources = sources
        for r in self._rows:
            r.set_sources(sources)

    # ------------------------------------------------------------------ build
    def rebuild(self) -> None:
        m = self.model.master()
        self.volume.set_value(m.get("volume", 0.5))
        self.quantize.blockSignals(True)
        self.quantize.setChecked(bool(m.get("quantize", True)))
        self.quantize.blockSignals(False)
        self.scale.set_value(m.get("scale", "major_pentatonic"))
        self.mod_smooth.set_value(m.get("mod_smooth_ms", 25.0))
        self.filter_panel.sync_from(m.get("filter") or {})

        routes = m.get("routes") or []
        if len(routes) != self._built_n:
            self._built_n = len(routes)
            for r in self._rows:
                r.setParent(None)
                r.deleteLater()
            self._rows = []
            for i, r in enumerate(routes):
                row = RouteRow(f"/master/routes/{i}", r, self.caps, self._sources,
                               self._set, parent=self.rack)
                row.removeRequested.connect(self._remove_route)
                self.rack_lay.insertWidget(self.rack_lay.count() - 1, row)
                self._rows.append(row)

    def _add_route(self) -> None:
        src = {"module": "", "key": ""}
        if self._sources:
            module, _, key = self._sources[0].partition(".")
            src = {"module": module, "key": key}
        self.model.master().setdefault("routes", []).append({
            "source": src, "dest": "vowel_morph",
            "norm": {"mode": "median_mad", "z_lo": 0.0, "z_hi": 4.0, "ref_key": "",
                     "gate": 1.4, "full": 2.0, "in_lo": 0.0, "in_hi": 1.0,
                     "smooth_ms": 120.0, "window_s": 10.0},
            "depth": 1.0, "curve": 1.0, "invert": False, "enabled": True,
        })
        self._push_whole()

    def _remove_route(self, base: str) -> None:
        try:
            i = int(base.rsplit("/", 1)[1])
        except (ValueError, IndexError):
            return
        routes = self.model.master().get("routes") or []
        if 0 <= i < len(routes):
            routes.pop(i)
            self._push_whole()

    def _push_whole(self) -> None:
        self.model.flush()
        self.model.set_patch(self.model.patch)
        self._built_n = -1
        self.on_dirty()
        self.structural_change()

    def structural_change(self) -> None:  # pragma: no cover - overridden at wiring time
        pass

    # ------------------------------------------------------------------ live
    def update_state(self, state: dict) -> None:
        m = state.get("master") or {}
        self.meter.set_value(m.get("peak", 0.0))
        rs = m.get("routes") or []
        for i, row in enumerate(self._rows):
            if i < len(rs):
                row.update_meter(rs[i].get("norm", 0.0), rs[i].get("out", 0.0))
