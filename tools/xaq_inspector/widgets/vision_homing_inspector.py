"""VisualHomingNav dashboard — loop #4, CLOSE on a SEEN source.

VisualHomingNav consumes the food-bearing EPM (fed by VisualBearing) and steers
toward a seen source.  Its value is a DETECTION/DIRECTION confidence
(green_gate · informativeness), distance-independent — the sight analogue of the
klino run-commit.  Optional target persistence holds an allocentric bearing while
the food is briefly occluded (default off, net-negative — see the report).

snapshot: {have_food, have_target, persisting, tgt_conf, value, cap_vision,
           eat_green, informativeness, epm_tle, node_count, vx, vy}

Food is FOV-gated, so most of these sit at 0 until food is actually seen — the
status banner makes that legible instead of a frozen JSON blob.

Panels:
  * Bearing + status — a needle toward the seen/remembered food, with a banner:
    FOOD SEEN → homing / homing to remembered target / idle.
  * value / informativeness / cap time-series.
  * State readout (all scalars + the food-bearing EPM size).
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


class _HomingCompass(QWidget):
    """Needle toward the seen/remembered food (forward = up) + a status banner."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = QLabel("idle (no food)")
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
        self._plot.plot([0, 0], [0, 1.15], pen=pg.mkPen(60, 90, 60, width=1))
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
        mag = math.hypot(vx, vy)
        have_food = bool(snap.get("have_food", False))
        persisting = bool(snap.get("persisting", False))
        value = float(snap.get("value", 0.0) or 0.0)

        if have_food:
            self._banner.setText("FOOD SEEN → homing")
            self._banner.setStyleSheet("color:#1baf7a; font-weight:bold; font-size:12px;")
            col = (60, 210, 130)
        elif persisting:
            self._banner.setText("homing to remembered target")
            self._banner.setStyleSheet("color:#eda100; font-weight:bold; font-size:12px;")
            col = (237, 161, 0)
        else:
            self._banner.setText("idle (no food)")
            self._banner.setStyleSheet("color:#888; font-weight:bold; font-size:12px;")
            col = (120, 120, 120)

        if mag > 1e-6:
            scale = min(1.1, mag) / mag
            ex, ey = vx * scale, vy * scale
        else:
            ex, ey = 0.0, 0.0
        # brightness ∝ value (confidence of the homing decision)
        v = max(0.0, min(1.0, value))
        c = tuple(int(60 + (ch - 60) * (0.4 + 0.6 * v)) for ch in col)
        self._needle.setData([0, ex], [0, ey])
        self._needle.setPen(pg.mkPen(*c, width=3))
        self._tip.setData([ex], [ey], brush=pg.mkBrush(*c, 255))


class _StateReadout(QWidget):
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

        def flag(k):
            return "yes" if snapshot.get(k) else " no"

        def num(k):
            try:
                return float(snapshot.get(k, 0.0) or 0.0)
            except (TypeError, ValueError):
                return 0.0

        rows = [
            ("have_food",     flag("have_food")),
            ("have_target",   flag("have_target")),
            ("persisting",    flag("persisting")),
            ("value",         f"{num('value'):.4f}"),
            ("informativeness", f"{num('informativeness'):.4f}"),
            ("cap_vision",    f"{num('cap_vision'):.4f}"),
            ("tgt_conf",      f"{num('tgt_conf'):.4f}"),
            ("epm_tle",       f"{num('epm_tle'):.4f}"),
            ("food-EPM nodes", f"{int(num('node_count'))}"),
        ]
        self._lbl.setText("\n".join(f"{k:>16}: {v:>9}" for k, v in rows))


class VisualHomingInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)

        self._compass = _HomingCompass()
        self._readout = _StateReadout()
        self._series = MultiSeriesPlot(
            [
                Series("value",          "value",         (74, 58, 167),  width=2.0),
                Series("informativeness", "informativeness", (27, 175, 122), width=1.5),
                Series("cap_vision",     "cap",           (237, 161, 0),  width=1.2),
            ],
            title="Homing value + informativeness + reach cap",
            y_label="value",
        )

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._compass)
        top.addWidget(self._readout)
        top.setSizes([440, 360])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([420, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._compass.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._series.update_payload(snapshot)
