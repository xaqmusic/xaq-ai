"""ScentCompass dashboard — the Cell's chemical-gradient perception.

ScentCompass reduces the raw 8-nostril scent ring to a 2-D EGOCENTRIC bearing
toward the up-gradient direction (cx=+right, cy=+forward).  Its magnitude is the
gradient STRENGTH (= confidence); the discarded common-mode is proximity ("how
much food", not "which way").  This is the perception half of the chemotaxis
loop, opaque until now.

Three panels:
  * Compass — a top-down egocentric plot.  The 8 nostrils sit on a body-local
    ring (forward = up); each dot's brightness ∝ its concentration, so you can
    SEE which side smells food.  A needle from the origin draws the gradient
    bearing (cx, cy); its length ∝ confidence.
  * Magnitude / proximity time-series — gradient strength + proximity over time
    (does the bug actually climb the gradient?).
  * Readout — cx / cy / |grad| / proximity scalars.
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


class _Compass(QWidget):
    """Top-down egocentric compass: nostril ring + gradient needle."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        title = QLabel("Scent compass (forward = up)")
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
        # unit reference circle
        th = np.linspace(0, 2 * math.pi, 64)
        self._plot.plot(np.cos(th), np.sin(th),
                        pen=pg.mkPen(80, 80, 80, width=1))
        # forward tick (up)
        self._plot.plot([0, 0], [0, 1.15], pen=pg.mkPen(60, 90, 60, width=1))
        # nostril dots (brightness set per tick)
        self._nostrils = pg.ScatterPlotItem(size=14, pen=None)
        self._plot.addItem(self._nostrils)
        # gradient needle (origin → bearing)
        self._needle = self._plot.plot([0, 0], [0, 0],
                                       pen=pg.mkPen(255, 215, 60, width=3))
        self._tip = pg.ScatterPlotItem(size=12,
                                       brush=pg.mkBrush(255, 215, 60, 255),
                                       pen=None)
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
        scent = snap.get("scent") or []
        n = int(snap.get("nostril_count", len(scent)) or 0)
        n = min(n, len(scent))

        # nostril ring: dot i at body-local angle 2π·i/N → display (cosθ, -sinθ)
        # (C++: +X=right, +Z back, forward=-Z ⇒ display y = -sinθ).
        spots = []
        if n > 0:
            vals = [float(scent[i] or 0.0) for i in range(n)]
            vmax = max(vals + [1e-6])
            for i in range(n):
                ang = 2 * math.pi * i / float(n)
                x = math.cos(ang)
                y = -math.sin(ang)
                frac = max(0.0, vals[i] / vmax)
                # cold (dim blue) → hot (bright green) by concentration
                r = int(40 + 60 * frac)
                g = int(60 + 195 * frac)
                b = int(120 * (1.0 - frac))
                spots.append({"pos": (x, y), "size": 10 + 12 * frac,
                              "brush": pg.mkBrush(r, g, b, 255)})
        self._nostrils.setData(spots)

        cx = float(snap.get("cx", 0.0) or 0.0)
        cy = float(snap.get("cy", 0.0) or 0.0)
        mag = float(snap.get("mag", math.hypot(cx, cy)) or 0.0)
        # draw the needle as a UNIT-ish direction (so it stays on the plot),
        # length scaled by confidence but capped at the ring.
        if mag > 1e-6:
            scale = min(1.1, mag) / mag
            ex, ey = cx * scale, cy * scale
        else:
            ex, ey = 0.0, 0.0
        self._needle.setData([0, ex], [0, ey])
        self._tip.setData([ex], [ey])


class _Readout(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet(
            "color: #ddd; font-family: Monospace; font-size: 12px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._lbl, 1)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        cx = float(snapshot.get("cx", 0.0) or 0.0)
        cy = float(snapshot.get("cy", 0.0) or 0.0)
        mag = float(snapshot.get("mag", math.hypot(cx, cy)) or 0.0)
        bearing = math.degrees(math.atan2(cx, cy)) if mag > 1e-9 else 0.0
        rows = [
            ("cx (right)",  cx),
            ("cy (fwd)",    cy),
            ("|grad|",      mag),
            ("bearing°",    bearing),
            ("proximity",   float(snapshot.get("prox", 0.0) or 0.0)),
            ("nostrils",    int(snapshot.get("nostril_count", 0) or 0)),
        ]
        self._lbl.setText("\n".join(f"{k:>12}: {v:8.4f}" for k, v in rows))


class ScentCompassInspector(QWidget):
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

        self._compass = _Compass()
        self._series = MultiSeriesPlot(
            [
                Series("mag",  "|grad|",    (255, 215,  60), width=2.0),
                Series("prox", "proximity", (120, 200, 255), width=1.5),
            ],
            title="Gradient strength + proximity",
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
