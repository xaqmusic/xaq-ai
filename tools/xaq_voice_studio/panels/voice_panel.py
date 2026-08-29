"""The middle pane: pick a voice, shape its oscillator, wire its routes.

Rebuilding every widget on each patch change would be simple and wrong — it would drop
the combo box the operator is mid-drag on.  So the rack is rebuilt only when the ROUTE
COUNT or the selected voice changes; ordinary edits flow through the model and back into
the existing widgets.
"""
from __future__ import annotations

from typing import Callable

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QComboBox, QGroupBox, QHBoxLayout, QLabel, QPushButton,
                             QScrollArea, QVBoxLayout, QWidget)

from ..theme import INK_PRIMARY, SurfaceWidget
from ._controls import FloatSlider, LabeledCombo, check, heading, hline, muted
from .filter_panel import FilterPanel
from .route_row import RouteRow


class VoicePanel(QWidget):
    def __init__(self, model, caps: dict, on_dirty: Callable[[], None],
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.model = model
        self.caps = caps
        self.on_dirty = on_dirty
        self._index = 0
        self._rows: list[RouteRow] = []
        self._built_key: tuple = ()
        self._sources: list[str] = []

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        top = QHBoxLayout()
        top.addWidget(heading("Voice"))
        self.picker = QComboBox()
        self.picker.currentIndexChanged.connect(self._on_pick)
        top.addWidget(self.picker, 1)
        self.enabled = check("on", True, self._on_enabled)
        top.addWidget(self.enabled)
        lay.addLayout(top)

        self.readout = QLabel("—")
        self.readout.setObjectName("value")
        lay.addWidget(self.readout)
        lay.addWidget(hline())

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.body = QWidget()
        self.body_lay = QVBoxLayout(self.body)
        self.body_lay.setContentsMargins(0, 0, 6, 0)
        self.body_lay.setSpacing(8)
        scroll.setWidget(self.body)
        lay.addWidget(scroll, 1)

        self._build_static()

    # ------------------------------------------------------------------ static
    def _build_static(self) -> None:
        self.osc_box = QGroupBox("oscillator")
        ol = QVBoxLayout(self.osc_box)
        ol.setContentsMargins(8, 4, 8, 6)
        ol.setSpacing(4)

        self.waveform = LabeledCombo("wave", self.caps.get("waveforms", ["square"]),
                                     "square", label_w=64)
        self.waveform.currentTextChanged.connect(
            lambda v: self._set("osc/waveform", v))
        ol.addWidget(self.waveform)

        self.base_hz = FloatSlider("base", 30, 4200, 261.63, log=True, unit=" Hz",
                                   decimals=1, label_w=64)
        self.base_hz.valueChanged.connect(lambda v: self._set("osc/base_hz", float(v)))
        ol.addWidget(self.base_hz)

        self.level = FloatSlider("level", 0, 2, 1.0, label_w=64)
        self.level.valueChanged.connect(lambda v: self._set("osc/level", float(v)))
        ol.addWidget(self.level)

        self.pan = FloatSlider("pan", -1, 1, 0.0, label_w=64)
        self.pan.valueChanged.connect(lambda v: self._set("osc/pan", float(v)))
        ol.addWidget(self.pan)

        self.pulse_width = FloatSlider("width", 0.02, 0.98, 0.5, label_w=64)
        self.pulse_width.setToolTip("Pulse duty.  Only audible on the 'pulse' waveform.")
        self.pulse_width.valueChanged.connect(
            lambda v: self._set("osc/pulse_width", float(v)))
        ol.addWidget(self.pulse_width)

        self.noise_mix = FloatSlider("noise", 0, 1, 0.0, label_w=64)
        self.noise_mix.setToolTip("Blends pink noise over ANY waveform.")
        self.noise_mix.valueChanged.connect(lambda v: self._set("osc/noise_mix", float(v)))
        ol.addWidget(self.noise_mix)

        self.glide = FloatSlider("glide", 0, 500, 30, unit=" ms", decimals=0, label_w=64)
        self.glide.valueChanged.connect(lambda v: self._set("osc/glide_ms", float(v)))
        ol.addWidget(self.glide)

        self.attack = FloatSlider("attack", 0, 500, 20, unit=" ms", decimals=0, label_w=64)
        self.attack.valueChanged.connect(lambda v: self._set("osc/attack_ms", float(v)))
        ol.addWidget(self.attack)

        self.release = FloatSlider("release", 0, 2000, 150, unit=" ms", decimals=0,
                                   label_w=64)
        self.release.valueChanged.connect(lambda v: self._set("osc/release_ms", float(v)))
        ol.addWidget(self.release)

        self.quantize = LabeledCombo("quantise", ["follow master", "off", "on"],
                                     "follow master", label_w=64)
        self.quantize.currentTextChanged.connect(self._on_quantize)
        ol.addWidget(self.quantize)
        self.body_lay.addWidget(self.osc_box)

        self.filter_panel = FilterPanel("filter", "", {}, self.caps, self._set_abs)
        self.body_lay.addWidget(self.filter_panel)

        rack_head = QHBoxLayout()
        rack_head.addWidget(heading("Modulation"))
        rack_head.addStretch(1)
        add = QPushButton("+ route")
        add.clicked.connect(self._add_route)
        rack_head.addWidget(add)
        self.body_lay.addLayout(rack_head)

        self.rack = QWidget()
        self.rack_lay = QVBoxLayout(self.rack)
        self.rack_lay.setContentsMargins(0, 0, 0, 0)
        self.rack_lay.setSpacing(6)
        self.body_lay.addWidget(self.rack)

        self.events_label = muted("")
        self.body_lay.addWidget(self.events_label)
        self.body_lay.addStretch(1)

    # ------------------------------------------------------------------ helpers
    def _vpath(self, suffix: str = "") -> str:
        return f"/voices/{self._index}" + (f"/{suffix}" if suffix else "")

    def _set(self, suffix: str, value) -> None:
        self.model.set(self._vpath(suffix), value)
        self.on_dirty()

    def _set_abs(self, path: str, value) -> None:
        self.model.set(path, value)
        self.on_dirty()

    def _voice(self) -> dict:
        voices = self.model.voices()
        return voices[self._index] if 0 <= self._index < len(voices) else {}

    def _on_pick(self, i: int) -> None:
        if i < 0:
            return
        self._index = i
        self.rebuild()

    def _on_enabled(self, v: bool) -> None:
        self._set("enabled", bool(v))

    def _on_quantize(self, text: str) -> None:
        self._set("osc/quantize", {"follow master": -1, "off": 0, "on": 1}.get(text, -1))

    def set_sources(self, sources: list[str]) -> None:
        self._sources = sources
        for r in self._rows:
            r.set_sources(sources)

    # ------------------------------------------------------------------ build
    def rebuild(self) -> None:
        voices = self.model.voices()

        names = [v.get("id") or v.get("module") or f"voice {i}" for i, v in enumerate(voices)]
        if [self.picker.itemText(i) for i in range(self.picker.count())] != names:
            self.picker.blockSignals(True)
            self.picker.clear()
            self.picker.addItems(names)
            self.picker.blockSignals(False)
        if not voices:
            self._index = 0
            self.readout.setText("no voices — connect an engine, or use Auto-assign")
            self._clear_rack()
            self.osc_box.setVisible(False)
            self.filter_panel.setVisible(False)
            return

        self.osc_box.setVisible(True)
        self.filter_panel.setVisible(True)
        self._index = min(self._index, len(voices) - 1)
        self.picker.blockSignals(True)
        self.picker.setCurrentIndex(self._index)
        self.picker.blockSignals(False)

        v = self._voice()
        osc = v.get("osc") or {}
        self.enabled.blockSignals(True)
        self.enabled.setChecked(bool(v.get("enabled", True)))
        self.enabled.blockSignals(False)

        self.waveform.set_value(osc.get("waveform", "square"))
        self.base_hz.set_value(osc.get("base_hz", 261.63))
        self.level.set_value(osc.get("level", 1.0))
        self.pan.set_value(osc.get("pan", 0.0))
        self.pulse_width.set_value(osc.get("pulse_width", 0.5))
        self.noise_mix.set_value(osc.get("noise_mix", 0.0))
        self.glide.set_value(osc.get("glide_ms", 30.0))
        self.attack.set_value(osc.get("attack_ms", 20.0))
        self.release.set_value(osc.get("release_ms", 150.0))
        self.quantize.set_value({-1: "follow master", 0: "off", 1: "on"}
                                .get(int(osc.get("quantize", -1)), "follow master"))

        self.filter_panel.base = self._vpath("filter")
        self.filter_panel.sync_from(v.get("filter") or {})

        routes = v.get("routes") or []
        key = (self._index, len(routes), tuple(r.get("dest") for r in routes))
        if key != self._built_key:
            self._built_key = key
            self._rebuild_rack(routes)

        events = v.get("events") or []
        if events:
            txt = "  ·  ".join(
                f"{(e.get('source') or {}).get('key','?')} {e.get('trigger','')} → {e.get('sound','')}"
                for e in events)
            self.events_label.setText("events:  " + txt)
        else:
            self.events_label.setText("")

    def _clear_rack(self) -> None:
        for r in self._rows:
            r.setParent(None)
            r.deleteLater()
        self._rows = []

    def _rebuild_rack(self, routes: list[dict]) -> None:
        self._clear_rack()
        module = self._voice().get("module", "")
        ref_keys = [s.split(".", 1)[1] for s in self._sources
                    if s.startswith(module + ".")]
        for i, r in enumerate(routes):
            row = RouteRow(self._vpath(f"routes/{i}"), r, self.caps, self._sources,
                           self._set_abs, parent=self.rack)
            row.set_ref_keys(ref_keys, (r.get("norm") or {}).get("ref_key", ""))
            row.removeRequested.connect(self._remove_route)
            self.rack_lay.addWidget(row)
            self._rows.append(row)

    # ------------------------------------------------------------------ edits
    def _add_route(self) -> None:
        v = self._voice()
        if not v:
            return
        module = v.get("module", "")
        src = {"module": module, "key": ""}
        # Default the new route to a source the voice already uses, so it does something
        # the moment it appears rather than sitting inert until a combo box is found.
        for r in v.get("routes") or []:
            s = r.get("source") or {}
            if s.get("key"):
                src = {"module": s.get("module", module), "key": s["key"]}
                break
        v.setdefault("routes", []).append({
            "source": src, "dest": "cutoff",
            "norm": {"mode": "median_mad", "z_lo": 0.0, "z_hi": 4.0, "ref_key": "",
                     "gate": 1.4, "full": 2.0, "in_lo": 0.0, "in_hi": 1.0,
                     "smooth_ms": 60.0, "window_s": 10.0},
            "depth": 12.0, "curve": 1.0, "invert": False, "enabled": True,
        })
        self._push_whole()

    def _remove_route(self, base: str) -> None:
        try:
            i = int(base.rsplit("/", 1)[1])
        except (ValueError, IndexError):
            return
        routes = self._voice().get("routes") or []
        if 0 <= i < len(routes):
            routes.pop(i)
            self._push_whole()

    def _push_whole(self) -> None:
        """Adding or removing a route changes the SHAPE of the patch.

        Pointer ops can only edit fields that already exist — that restriction is what
        stops a typo'd path from silently inventing one — so a structural change goes as
        a whole-patch set instead.
        """
        self.model.flush()
        self.model.set_patch(self.model.patch)
        self._built_key = ()
        self.on_dirty()
        self.structural_change()

    # Replaced by the window, which owns the engine connection.
    def structural_change(self) -> None:  # pragma: no cover - overridden at wiring time
        pass

    # ------------------------------------------------------------------ live
    def update_state(self, state: dict) -> None:
        voices = state.get("voices") or []
        if not (0 <= self._index < len(voices)):
            return
        v = voices[self._index]
        self.readout.setText(
            f"{v.get('hz', 0):7.1f} Hz {v.get('note',''):<4}   amp {v.get('amp', 0):.2f}   "
            f"cut {v.get('cutoff', 0):6.0f} Hz   Q {v.get('q', 0):.1f}")
        rs = v.get("routes") or []
        for i, row in enumerate(self._rows):
            if i < len(rs):
                row.update_meter(rs[i].get("norm", 0.0), rs[i].get("out", 0.0))
