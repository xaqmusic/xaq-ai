"""Klinotaxis dashboard — the honest scalar-gradient follower (epistemic foraging).

Klinotaxis can't read a scalar gradient's DIRECTION (it's a hidden external state), so it
WEAVES and lock-in detects: correlate dscalar/dt against the IMU yaw-rate ω → the in-phase
correlation g ∈ [-1,1] IS the lateral gradient → steer the weave centre (base_heading)
toward it.  The output is the weaving heading, fed to the HeadingController.

Debug focus (2026-06-26): the bug pins when the commanded target lands BEHIND it (|bearing
error| > 90°) — the HeadingController's advance is 0 there (correctly: don't thrust toward a
target behind you), so it turns-without-advancing.  This dial shows exactly that geometry.

Panels:
  * Egocentric dial (forward = up = the bug's nose): the TARGET needle (the commanded
    weave heading) coloured green→red by how far off-axis it is; the CLIMB-direction needle
    (base_heading, cyan) and the weave-range arc; the rear half (the advance-0 PIN ZONE) is
    shaded red — when the target needle sits in it, the bug is pinned.
  * Lock-in bar — g ∈ [-1,1]: the recovered lateral-gradient sign+strength (the steering
    signal).  Near 0 = no usable direction (flat / lost lock); |g|→1 = confident.
  * Time-series — g (steering), bearing-error/π, ω (yaw rate), trend (climbing?).
  * Readout — base/heading/bearing-error in degrees, g, lock-in mag, ω, trend, period.
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


class _KlinoDial(QWidget):
    """Egocentric dial: where the commanded target is relative to the bug's nose."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        title = QLabel("Target vs bug (forward = up) · rear shade = advance-0 PIN ZONE")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setXRange(-1.3, 1.3)
        self._plot.setYRange(-1.3, 1.3)
        self._plot.hideAxis("bottom")
        self._plot.hideAxis("left")
        # rear half (|angle| > 90° → ey < 0): the advance-0 pin zone
        th = np.linspace(math.pi, 2 * math.pi, 48)  # lower semicircle
        xs = np.concatenate([[0.0], np.cos(th), [0.0]])
        ys = np.concatenate([[0.0], np.sin(th), [0.0]])
        pin = pg.PlotDataItem(xs, ys, fillLevel=0, brush=pg.mkBrush(120, 30, 30, 70), pen=None)
        self._plot.addItem(pin)
        ring = np.linspace(0, 2 * math.pi, 64)
        self._plot.plot(np.cos(ring), np.sin(ring), pen=pg.mkPen(80, 80, 80, width=1))
        self._plot.plot([0, 0], [0, 1.15], pen=pg.mkPen(60, 90, 60, width=1))  # forward (nose)
        self._weave = self._plot.plot([0, 0], [0, 0], pen=pg.mkPen(90, 90, 140, width=6))  # weave arc
        self._base = self._plot.plot([0, 0], [0, 0], pen=pg.mkPen(80, 200, 230, width=2))   # climb dir
        self._needle = self._plot.plot([0, 0], [0, 0], pen=pg.mkPen(255, 215, 60, width=3)) # target
        self._tip = pg.ScatterPlotItem(size=12, brush=pg.mkBrush(255, 215, 60, 255), pen=None)
        self._plot.addItem(self._tip)
        self._warn = pg.TextItem("", color=(255, 90, 90), anchor=(0.5, 0.5))
        self._warn.setPos(0, -1.18)
        self._plot.addItem(self._warn)
        layout.addWidget(self._plot)

        self._latest = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        s = self._latest
        vx = float(s.get("vx", 0.0) or 0.0)
        vy = float(s.get("vy", 1.0) or 0.0)
        berr = math.atan2(vx, vy)                    # egocentric bearing to the target
        # climb direction (base) relative to the bug
        base = float(s.get("base_heading", 0.0) or 0.0)
        head = float(s.get("heading", 0.0) or 0.0)
        amp = float(s.get("weave_amp", 0.0) or 0.0)
        base_ego = math.atan2(math.sin(base - head), math.cos(base - head))
        # target needle, coloured by off-axis fraction (green facing → red behind)
        f = min(1.0, abs(berr) / math.pi)
        col = (int(40 + 210 * f), int(220 - 170 * f), 50)
        self._needle.setData([0, vx], [0, vy], pen=pg.mkPen(*col, width=3))
        self._tip.setData([vx], [vy], brush=pg.mkBrush(*col, 255))
        # climb-direction needle (cyan)
        self._base.setData([0, math.sin(base_ego)], [0, math.cos(base_ego)])
        # weave-range arc around the climb direction
        ts = np.linspace(base_ego - amp, base_ego + amp, 16)
        self._weave.setData((np.sin(ts) * 1.05).tolist(), (np.cos(ts) * 1.05).tolist())
        if bool(s.get("turning", False)):
            self._warn.setColor((255, 200, 80)); self._warn.setText("TURNING IN PLACE → climb dir")
        elif abs(berr) > math.pi / 2:
            self._warn.setColor((255, 90, 90)); self._warn.setText("target behind, advance=0")
        else:
            self._warn.setText("")


class _LockinBar(QWidget):
    """g ∈ [-1,1]: the recovered lateral-gradient sign + strength (the steering signal)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        title = QLabel("Lock-in  g = corr(dscalar/dt, ω)   ← steer-right · steer-left →")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)
        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setXRange(-1.05, 1.05)
        self._plot.setYRange(-0.6, 0.6)
        self._plot.hideAxis("left")
        self._plot.plot([0, 0], [-0.5, 0.5], pen=pg.mkPen(80, 80, 80, width=1))
        self._bar = pg.BarGraphItem(x=[0], height=[0.6], width=0.0, brush=pg.mkBrush(120, 200, 255, 220))
        self._plot.addItem(self._bar)
        layout.addWidget(self._plot)
        self._latest = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(90)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        g = float(self._latest.get("g", 0.0) or 0.0)
        g = max(-1.0, min(1.0, g))
        col = (120, 200, 255) if g >= 0 else (255, 150, 120)
        self._bar.setOpts(x0=[min(0.0, g)], width=abs(g), height=0.6,
                          brush=pg.mkBrush(*col, 220))


class _Readout(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet("color: #ddd; font-family: Monospace; font-size: 12px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._lbl, 1)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        s = snapshot
        vx = float(s.get("vx", 0.0) or 0.0); vy = float(s.get("vy", 1.0) or 0.0)
        berr = math.degrees(math.atan2(vx, vy))
        rows = [
            ("base°",       math.degrees(float(s.get("base_heading", 0.0) or 0.0))),
            ("heading°",    math.degrees(float(s.get("heading", 0.0) or 0.0))),
            ("bearing_err°", berr),
            ("g (steer)",   float(s.get("g", 0.0) or 0.0)),
            ("lockin_mag",  float(s.get("lockin_mag", 0.0) or 0.0)),
            ("omega",       float(s.get("omega", 0.0) or 0.0)),
            ("trend",       float(s.get("trend", 0.0) or 0.0)),
            ("period",      float(s.get("period", 0.0) or 0.0)),
        ]
        txt = "\n".join(f"{k:>12}: {v:9.3f}" for k, v in rows)
        if bool(s.get("turning", False)):
            txt += "\n\n  ↻ TURNING IN PLACE (climb dir behind)"
        elif abs(berr) > 90.0:
            txt += "\n\n  ⚠ target behind → advance=0"
        self._lbl.setText(txt)


class KlinotaxisInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._dial = _KlinoDial()
        self._bar = _LockinBar()
        self._series = MultiSeriesPlot(
            [
                Series("g",      "g (steer signal)",  (120, 200, 255), width=2.0),
                Series("berr",   "bearing-err / π",   (255, 215, 60), width=1.5),
                Series("omega",  "ω (yaw rate)",      (255, 120, 140), width=1.0),
                Series("trend",  "trend (climbing?)", (120, 255, 140), width=1.5),
            ],
            title="Steering signal + geometry + climb",
            y_label="value",
        )
        self._readout = _Readout()

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._dial)
        top.addWidget(self._bar)
        top.setSizes([460, 440])

        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.addWidget(self._series)
        bot.addWidget(self._readout)
        bot.setSizes([660, 320])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(bot)
        v.setSizes([380, 280])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._dial.update_payload(snapshot)
        self._bar.update_payload(snapshot)
        # derive bearing-err/π for the series
        vx = float(snapshot.get("vx", 0.0) or 0.0); vy = float(snapshot.get("vy", 1.0) or 0.0)
        snap2 = dict(snapshot)
        snap2["berr"] = math.atan2(vx, vy) / math.pi
        self._series.update_payload(snap2)
        self._readout.update_payload(snapshot)
