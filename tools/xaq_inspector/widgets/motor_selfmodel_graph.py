"""Self-model weight graph — the Motor-EPM's "knowledge graph" analog.

The perceptual EPM crystallizes a GNG of prototype nodes; the Motor-EPM has no
node graph — its knowledge is the LINEAR forward self-model  x̂ = A·y + b
(motor command → predicted next sensor).  The honest graph view of a linear map
is a bipartite weight graph: motor channels on the left, sensor channels on the
right, one edge per A[i,j].  Edge colour = sign (green +, red −), width/alpha =
|weight|.  This is "what the body has learned about how each command moves each
sensor" — the same insight the A heatmap holds, but read as connections.

A is rows_A (sensor) × cols_A (motor), stored column-major:  A[i + j*rows_A].
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget


# Cell motor channels (n_legs=1, motor_dim=2): the bidirectional_paddler maps
# motor[0]/[1] to differential (turn) + common-mode (thrust) downstream.  For a
# legged body these are hip1/hip2/knee per leg.  Labels are best-effort by dim.
_MOTOR_LABELS_BY_DIM = {
    2: ["paddle L", "paddle R"],
    3: ["hip1", "hip2", "knee"],
}


class MotorSelfModelGraph(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._title = QLabel("Self-model graph — awaiting model")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.hideAxis("bottom")
        self._plot.hideAxis("left")
        self._plot.setXRange(-0.3, 1.3)
        # edges behind nodes
        self._edges = pg.PlotCurveItem(connect="finite")
        self._plot.addItem(self._edges)
        self._edge_items: list = []          # per-edge coloured segments
        self._motor = pg.ScatterPlotItem(size=18, symbol="s",
                                         brush=pg.mkBrush(120, 200, 255, 230),
                                         pen=pg.mkPen("w", width=0.5))
        self._sensor = pg.ScatterPlotItem(size=14, symbol="o",
                                          brush=pg.mkBrush(180, 180, 180, 200),
                                          pen=pg.mkPen("w", width=0.5))
        self._plot.addItem(self._motor)
        self._plot.addItem(self._sensor)
        self._labels: list = []
        layout.addWidget(self._plot)

        self._latest = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(120)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    def _clear_dynamic(self) -> None:
        for it in self._edge_items:
            self._plot.removeItem(it)
        self._edge_items = []
        for lbl in self._labels:
            self._plot.removeItem(lbl)
        self._labels = []

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        snap = self._latest
        rows = int(snap.get("rows_A", 0) or 0)   # sensor dims
        cols = int(snap.get("cols_A", 0) or 0)   # motor dims
        flat = snap.get("A") or []
        if rows < 1 or cols < 1 or len(flat) < rows * cols:
            self._title.setText("Self-model graph — collecting (model not init)")
            return
        A = np.asarray(flat[: rows * cols], dtype=np.float32).reshape(
            (cols, rows)).T   # → (rows=sensor, cols=motor)

        self._clear_dynamic()

        # vertical layout: motor nodes left column, sensor nodes right column
        def col_ys(k):
            if k == 1:
                return [0.5]
            return list(np.linspace(0.05, 0.95, k))

        my = col_ys(cols)
        sy = col_ys(rows)
        self._motor.setData([0.0] * cols, my)
        self._sensor.setData([1.0] * rows, sy)

        # motor labels
        mlabels = _MOTOR_LABELS_BY_DIM.get(cols, [f"y{j}" for j in range(cols)])
        for j in range(cols):
            t = pg.TextItem(mlabels[j], color=(180, 210, 255), anchor=(1, 0.5))
            t.setPos(-0.04, my[j])
            self._plot.addItem(t)
            self._labels.append(t)

        amax = float(np.abs(A).max()) or 1e-6
        for i in range(rows):       # sensor
            for j in range(cols):   # motor
                w = float(A[i, j])
                mag = abs(w) / amax
                if mag < 0.05:
                    continue
                # green = +, red = −; alpha + width ∝ |weight|
                if w >= 0:
                    col = (90, 220, 120, int(40 + 200 * mag))
                else:
                    col = (230, 90, 90, int(40 + 200 * mag))
                seg = pg.PlotCurveItem(
                    x=[0.0, 1.0], y=[my[j], sy[i]],
                    pen=pg.mkPen(*col, width=0.6 + 3.0 * mag))
                self._plot.addItem(seg)
                self._edge_items.append(seg)

        self._title.setText(
            f"Self-model graph — {cols} motor → {rows} sensor   "
            f"|A|max {amax:.3f}   (green +, red −)")
