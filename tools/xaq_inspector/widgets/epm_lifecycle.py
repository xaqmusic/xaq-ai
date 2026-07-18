"""GNG lifecycle plot — node count + crystallized count over time.

Port of v3 src/native/widgets/gng_lifecycle_widget.py for a single EPM
modality (the dashboard's parent inspector views one EPM at a time so
multi-modality fan-out is not needed).
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QVBoxLayout, QWidget


BUFFER_SIZE = 600


class EpmLifecyclePlot(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._plot = pg.PlotWidget(title="GNG lifecycle")
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left",   "count")
        self._plot.setLabel("bottom", "tick (recent)")
        self._plot.addLegend(offset=(-10, 10))
        layout.addWidget(self._plot)

        self._curve_total = self._plot.plot(
            pen=pg.mkPen((180, 180, 200), width=2), name="nodes")
        self._curve_baked = self._plot.plot(
            pen=pg.mkPen((255, 215, 0),   width=2), name="baked")

        self._buf_total = np.full(BUFFER_SIZE, np.nan)
        self._buf_baked = np.full(BUFFER_SIZE, np.nan)
        self._dirty = False

        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        gng = snapshot.get("gng") if isinstance(snapshot, dict) else None
        if not isinstance(gng, dict):
            return
        nodes = gng.get("nodes") or []
        if not isinstance(nodes, list):
            return
        baked = sum(1 for n in nodes
                    if isinstance(n, dict)
                    and (int(n.get("post_bake_visits", 0)) > 0
                         or bool(n.get("bake_checked", False))))
        self._buf_total = np.roll(self._buf_total, -1)
        self._buf_baked = np.roll(self._buf_baked, -1)
        self._buf_total[-1] = float(len(nodes))
        self._buf_baked[-1] = float(baked)
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        self._curve_total.setData(self._buf_total, connect="finite")
        self._curve_baked.setData(self._buf_baked, connect="finite")
