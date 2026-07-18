"""VisualBearing dashboard — the Cell's visual food-bearing perception.

VisualBearing scans the camera frame for food-coloured pixels and reduces them to
an EGOCENTRIC bearing toward the food (vx=+right, vy=+forward), with a magnitude
(= detection confidence) and green_frac (how much of the view is food = a looming
/ proximity cue).  This is the perception half of the vision-homing loop (loop #4),
the sight analogue of the ScentCompass.

snapshot: {vx, vy, mag, green_frac, lesioned}

Note: food is FOV-gated, so vx/vy/green_frac sit at 0 most of the time and only
fire when food is actually in view — the "FOOD IN VIEW" banner makes that legible
instead of a frozen-looking JSON blob.

Panels:
  * Bearing compass — forward = up; a needle from the origin draws the food bearing
    (vx, vy), length ∝ confidence, colour ∝ green_frac.  A banner reads FOOD IN VIEW
    / searching… / LESIONED.
  * green_frac / mag time-series.
  * Scalar readout.
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


class _BearingCompass(QWidget):
    """Egocentric compass: a needle toward the seen food (forward = up)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = QLabel("searching…")
        self._banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._banner.setStyleSheet("color:#888; font-weight:bold; font-size:12px;")
        layout.addWidget(self._banner)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setXRange(-1.3, 1.3)
        self._plot.setYRange(-1.3, 1.3)
        self._plot.hideAxis("bottom")
        self._plot.hideAxis("left")
        th = np.linspace(0, 2 * math.pi, 64)
        self._plot.plot(np.cos(th), np.sin(th), pen=pg.mkPen(80, 80, 80, width=1))
        self._plot.plot([0, 0], [0, 1.15], pen=pg.mkPen(60, 90, 60, width=1))   # forward tick
        self._needle = self._plot.plot([0, 0], [0, 0], pen=pg.mkPen(120, 120, 120, width=3))
        self._tip = pg.ScatterPlotItem(size=12, pen=None)
        self._plot.addItem(self._tip)
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
        snap = self._latest
        vx = float(snap.get("vx", 0.0) or 0.0)
        vy = float(snap.get("vy", 0.0) or 0.0)
        mag = float(snap.get("mag", math.hypot(vx, vy)) or 0.0)
        green = float(snap.get("green_frac", 0.0) or 0.0)
        lesioned = bool(snap.get("lesioned", False))

        if lesioned:
            self._banner.setText("LESIONED (blind)")
            self._banner.setStyleSheet("color:#e34948; font-weight:bold; font-size:12px;")
        elif mag > 1e-6 or green > 1e-6:
            self._banner.setText("FOOD IN VIEW")
            self._banner.setStyleSheet("color:#1baf7a; font-weight:bold; font-size:12px;")
        else:
            self._banner.setText("searching…")
            self._banner.setStyleSheet("color:#888; font-weight:bold; font-size:12px;")

        # needle direction; length capped at the ring, colour brightens with green_frac
        if mag > 1e-6:
            scale = min(1.1, mag) / mag
            ex, ey = vx * scale, vy * scale
        else:
            ex, ey = 0.0, 0.0
        gf = max(0.0, min(1.0, green * 4.0))   # green_frac is usually small; boost for colour
        col = (int(60 + 40 * gf), int(90 + 165 * gf), int(60 + 20 * gf))
        self._needle.setData([0, ex], [0, ey])
        self._needle.setPen(pg.mkPen(*col, width=3))
        self._tip.setData([ex], [ey], brush=pg.mkBrush(*col, 255))


class _Readout(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet("color:#ddd; font-family:Monospace; font-size:12px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._lbl, 1)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        vx = float(snapshot.get("vx", 0.0) or 0.0)
        vy = float(snapshot.get("vy", 0.0) or 0.0)
        mag = float(snapshot.get("mag", math.hypot(vx, vy)) or 0.0)
        bearing = math.degrees(math.atan2(vx, vy)) if mag > 1e-9 else 0.0
        rows = [
            ("vx (right)", vx),
            ("vy (fwd)",   vy),
            ("|bearing|",  mag),
            ("bearing°",   bearing),
            ("green_frac", float(snapshot.get("green_frac", 0.0) or 0.0)),
            ("lesioned",   1.0 if snapshot.get("lesioned") else 0.0),
        ]
        self._lbl.setText("\n".join(f"{k:>12}: {v:8.4f}" for k, v in rows))


class VisualBearingInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)

        self._compass = _BearingCompass()
        self._series = MultiSeriesPlot(
            [
                Series("green_frac", "green_frac", (27, 175, 122), width=2.0),
                Series("mag",        "confidence", (237, 161, 0),   width=1.5),
            ],
            title="Food in view (green_frac) + detection confidence",
            y_label="value",
        )
        self._readout = _Readout()

        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.addWidget(self._series)
        bot.addWidget(self._readout)
        bot.setSizes([700, 300])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(self._compass)
        v.addWidget(bot)
        v.setSizes([420, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._compass.update_payload(snapshot)
        self._series.update_payload(snapshot)
        self._readout.update_payload(snapshot)
