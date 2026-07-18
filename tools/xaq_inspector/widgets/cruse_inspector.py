"""CruseCoordinator dashboard — gait coordination state + bias telemetry.

Built 2026-06-09 after Joseph UI observation: "I can see the data shows
up but it is not graphed" — Cruse falls through to the RawPayloadView
because no widget was registered for its module type.

Four panels:
  * Per-joint bias norms — three time-series plots stacked (hip1, hip2,
    knee), each with 4 traces (FL/FR/RL/RR).  Lets you see *which* leg
    is getting biased *which* direction and *when*.  Move 2 hip1 swing
    diagnostics live here.
  * Planted-state strip — 4-channel binary heatmap (FL/FR/RL/RR), white =
    planted, dark = swing.  Diagnoses whether Cruse's foot_y stance
    detector matches what the body is actually doing.
  * Rule fire rates — r1/r2/r3 fires per sim sec (delta of total fires).
    Reveals "extreme range of motion" caused by rules firing too
    aggressively.
  * Status label — current gain values, body_state, planted summary,
    cumulative rule counts.  Slow-changing context for the streams.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


_LEG_ORDER = ("fl", "fr", "rl", "rr")
_LEG_COLORS = {
    "fl": (120, 220, 120),   # green
    "fr": (220, 100, 100),   # red
    "rl": (240, 220,  90),   # yellow
    "rr": (110, 160, 240),   # blue
}


_BIAS_DECAY = 0.85   # per-tick decay for visible-trail EMA (~6-tick visible time)
_bias_ema: dict[str, float] = {}


def _flatten_cruse_snapshot(snap: dict) -> dict:
    """Transform Cruse's nested snapshot into a flat dict suitable for
    MultiSeriesPlot's dotted-key resolution.

    Cruse rules ONLY fire on swing/stance transitions (Rule 1 = anterior
    in swing → stance bias; Rule 2 = anterior just touched down → swing
    bias; Rule 3 = contralateral in swing → stance bias).  During pure
    static stance (all 4 legs planted, no swing events), total_factor
    is 0 across all Premotors → bias is zero vector → last_bias_norm =
    0.  Joseph 2026-06-09 UI: "I just saw the bias spike briefly when
    the robot fell over and reset.  then it remains at 0" — this is
    expected (not a widget bug).

    To make spikes visible on the time-series, we apply an EMA-decay
    trail to the bias norms: each tick we max(current, prev_ema *
    decay).  A 1-tick bias spike then decays visibly over ~6-10 ticks
    instead of being a single invisible sample.  The instantaneous raw
    norm is also exposed as `<key>_raw` for the status label.
    """
    global _bias_ema
    out: dict = {}
    for pm in snap.get("premotors") or []:
        pmid = str(pm.get("id", ""))
        jk = str(pm.get("joint_kind", ""))
        leg = pmid.replace("premotor_", "").replace(f"_{jk}", "")
        if jk in ("hip1", "hip2", "knee") and leg in _LEG_ORDER:
            key = f"{jk}_{leg}_bias"
            raw = float(pm.get("last_bias_norm", 0.0) or 0.0)
            decayed = max(raw, _bias_ema.get(key, 0.0) * _BIAS_DECAY)
            _bias_ema[key] = decayed
            out[key] = decayed
            out[f"{key}_raw"] = raw
            out[f"{jk}_{leg}_viol"] = float(pm.get("violation_ema",  0.0) or 0.0)
    for leg_state in snap.get("legs") or []:
        leg = str(leg_state.get("name", ""))
        if leg in _LEG_ORDER:
            out[f"planted_{leg}"] = 1.0 if bool(leg_state.get("is_planted", False)) else 0.0
    out["body_state"]         = float(snap.get("body_state_value", 1.0) or 1.0)
    out["gain"]               = float(snap.get("cruse_bias_gain", 0.0) or 0.0)
    out["gain_hip1"]          = float(snap.get("cruse_bias_gain_hip1", 0.0) or 0.0)
    out["gain_hip2"]          = float(snap.get("cruse_bias_gain_hip2", 1.0) or 1.0)
    out["gain_knee"]          = float(snap.get("cruse_bias_gain_knee", 0.0) or 0.0)
    # Move 5 — saturation gate state (off/on + zone + per-Premotor productive scores).
    out["sat_enabled"]        = bool(snap.get("saturation_gate_enabled", False))
    out["sat_zone_min"]       = float(snap.get("saturation_zone_min", 0.0) or 0.0)
    out["sat_zone_max"]       = float(snap.get("saturation_zone_max", 0.9) or 0.9)
    # Count suppressed Premotors per joint kind for the status label.
    out["sat_suppressed_hip1"] = 0
    out["sat_suppressed_hip2"] = 0
    out["sat_suppressed_knee"] = 0
    for pm in snap.get("premotors") or []:
        if bool(pm.get("saturation_suppressed", False)):
            jk = str(pm.get("joint_kind", ""))
            key = f"sat_suppressed_{jk}"
            if key in out:
                out[key] = int(out[key]) + 1
    out["total_rule1_fires"]  = float(snap.get("total_rule1_fires", 0) or 0)
    out["total_rule2_fires"]  = float(snap.get("total_rule2_fires", 0) or 0)
    out["total_rule3_fires"]  = float(snap.get("total_rule3_fires", 0) or 0)
    return out


def _bias_series_for(joint: str) -> list[Series]:
    return [
        Series(f"{joint}_{leg}_bias", f"{leg.upper()}",
               _LEG_COLORS[leg], width=1.6)
        for leg in _LEG_ORDER
    ]


class _PlantedStrip(QWidget):
    """4-row binary heatmap: planted=white, swing=dark."""
    BUFFER = 600

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.addWidget(QLabel("Planted state per leg (FL FR RL RR — top to bottom)"))
        self._view = pg.PlotWidget()
        self._view.setBackground("k")
        self._view.setMouseEnabled(x=False, y=False)
        self._view.hideAxis("left")
        self._view.setLabel("bottom", "tick (recent)")
        self._img = pg.ImageItem()
        self._view.addItem(self._img)
        layout.addWidget(self._view)
        self._buf = np.zeros((4, self.BUFFER), dtype=np.float32)
        self._cmap = pg.ColorMap(
            pos=np.array([0.0, 1.0]),
            color=np.array([[20, 20, 24, 255], [220, 220, 230, 255]], dtype=np.uint8),
        )

    def update_payload(self, snap_flat: dict) -> None:
        self._buf = np.roll(self._buf, -1, axis=1)
        for r, leg in enumerate(_LEG_ORDER):
            self._buf[r, -1] = float(snap_flat.get(f"planted_{leg}", 0.0))
        self._img.setImage(self._buf.T, autoLevels=False, levels=(0.0, 1.0),
                           lut=self._cmap.getLookupTable(0.0, 1.0, 256))


class _RuleRates(QWidget):
    """Three-line plot of r1/r2/r3 fires per sim sec.

    Cruse exposes cumulative totals; we differentiate against tick delta
    to get rate.  Buffer the last BUFFER samples.
    """
    BUFFER = 600

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.addWidget(QLabel("Rule fire rates (r1=stance / r2=swing / r3=contralateral) per sim sec"))
        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left", "fires/sec")
        self._plot.setLabel("bottom", "tick (recent)")
        self._plot.addLegend(offset=(-10, 10))
        layout.addWidget(self._plot)
        self._buf = {1: np.full(self.BUFFER, np.nan),
                     2: np.full(self.BUFFER, np.nan),
                     3: np.full(self.BUFFER, np.nan)}
        self._curves = {
            1: self._plot.plot([], [], pen=pg.mkPen((255, 180, 100), width=1.5), name="r1 stance"),
            2: self._plot.plot([], [], pen=pg.mkPen((100, 220, 255), width=1.5), name="r2 swing"),
            3: self._plot.plot([], [], pen=pg.mkPen((255, 100, 220), width=1.5), name="r3 contra"),
        }
        self._prev_totals: dict[int, float] = {}
        self._prev_tick: Optional[int] = None

    def update_payload(self, tick_id: int, snap_flat: dict) -> None:
        cur = {
            1: float(snap_flat.get("total_rule1_fires", 0.0)),
            2: float(snap_flat.get("total_rule2_fires", 0.0)),
            3: float(snap_flat.get("total_rule3_fires", 0.0)),
        }
        if self._prev_tick is not None and tick_id > self._prev_tick:
            dt_ticks = tick_id - self._prev_tick
            dt_sec = max(1e-6, dt_ticks * 0.02)  # 50 Hz physics
            for k, v in cur.items():
                rate = (v - self._prev_totals.get(k, v)) / dt_sec
                self._buf[k] = np.roll(self._buf[k], -1)
                self._buf[k][-1] = rate
        self._prev_totals = cur
        self._prev_tick = tick_id
        x = np.arange(self.BUFFER)
        for k, curve in self._curves.items():
            y = self._buf[k]
            mask = np.isfinite(y)
            if mask.any():
                curve.setData(x[mask], y[mask])


class _StatusLabel(QWidget):
    """Single QLabel that summarises the slow-changing state — gains,
    body_state, planted summary, cumulative rule counts."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        self._label = QLabel("(awaiting payload)")
        self._label.setStyleSheet("color: #ddd; font-family: monospace; font-size: 11px;")
        layout.addWidget(self._label)

    def update_payload(self, snap_flat: dict) -> None:
        planted_marks = " ".join(
            f"{leg.upper()}{'●' if snap_flat.get(f'planted_{leg}', 0.0) > 0.5 else '○'}"
            for leg in _LEG_ORDER
        )
        # Max raw bias across 4 legs per joint — tells you which joints
        # are *actually* firing right now (the EMA trail in the time-series
        # makes spikes visible but doesn't tell you which spike was fresh).
        def _max_raw(jk: str) -> float:
            return max(
                (float(snap_flat.get(f"{jk}_{leg}_bias_raw", 0.0)) for leg in _LEG_ORDER),
                default=0.0,
            )
        sat_on = bool(snap_flat.get("sat_enabled", False))
        sat_line = (
            f"  sat_gate=ON  zone=[{snap_flat.get('sat_zone_min', 0.0):.2f},{snap_flat.get('sat_zone_max', 0.9):.2f}]  "
            f"suppressed (this tick) hip1={int(snap_flat.get('sat_suppressed_hip1', 0))} "
            f"hip2={int(snap_flat.get('sat_suppressed_hip2', 0))} "
            f"knee={int(snap_flat.get('sat_suppressed_knee', 0))}"
            if sat_on else "  sat_gate=off"
        )
        self._label.setText(
            f"gain={snap_flat.get('gain', 0.0):.2f}  "
            f"hip1={snap_flat.get('gain_hip1', 0.0):.2f}  "
            f"hip2={snap_flat.get('gain_hip2', 1.0):.2f}  "
            f"knee={snap_flat.get('gain_knee', 0.0):.2f}  "
            f"body_state={snap_flat.get('body_state', 1.0):.2f}\n"
            f"planted: {planted_marks}    "
            f"cum r1/r2/r3 = "
            f"{int(snap_flat.get('total_rule1_fires', 0))}/"
            f"{int(snap_flat.get('total_rule2_fires', 0))}/"
            f"{int(snap_flat.get('total_rule3_fires', 0))}    "
            f"max_raw_bias  hip1={_max_raw('hip1'):.3f}  hip2={_max_raw('hip2'):.3f}  knee={_max_raw('knee'):.3f}"
            + sat_line
        )


class CruseInspector(QWidget):
    def __init__(self, module_id: str, module_type: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)

        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._status = _StatusLabel()
        outer.addWidget(self._status)

        # Top row — three bias plots side by side (one per joint kind).
        # Trails are EMA-decayed (~6-tick visible time) so spikes from
        # brief rule-firing events show up; pure raw values would be
        # invisible because Cruse rules fire only on swing transitions
        # and remain silent during static stance.
        self._hip1_plot = MultiSeriesPlot(
            _bias_series_for("hip1"),
            title="hip1 bias norm (Move 2: swing direction) — EMA trail",
            y_label="bias_norm (EMA)",
        )
        self._hip2_plot = MultiSeriesPlot(
            _bias_series_for("hip2"),
            title="hip2 bias norm (lift coordination) — EMA trail",
            y_label="bias_norm (EMA)",
        )
        self._knee_plot = MultiSeriesPlot(
            _bias_series_for("knee"),
            title="knee bias norm (gated off by Move 1) — EMA trail",
            y_label="bias_norm (EMA)",
        )
        bias_row = QSplitter(Qt.Orientation.Horizontal)
        bias_row.addWidget(self._hip1_plot)
        bias_row.addWidget(self._hip2_plot)
        bias_row.addWidget(self._knee_plot)
        bias_row.setSizes([400, 400, 400])

        # Bottom row — planted strip + rule rates.
        self._planted = _PlantedStrip()
        self._rule_rates = _RuleRates()
        bot_row = QSplitter(Qt.Orientation.Horizontal)
        bot_row.addWidget(self._planted)
        bot_row.addWidget(self._rule_rates)
        bot_row.setSizes([520, 520])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(bias_row)
        v.addWidget(bot_row)
        v.setSizes([360, 280])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        flat = _flatten_cruse_snapshot(snapshot)
        self._hip1_plot.update_payload(flat)
        self._hip2_plot.update_payload(flat)
        self._knee_plot.update_payload(flat)
        self._planted.update_payload(flat)
        self._rule_rates.update_payload(tick_id, flat)
        self._status.update_payload(flat)
