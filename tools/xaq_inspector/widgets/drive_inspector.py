"""HomeostaticDrive dashboard.

Two panels:
  * Urgency time-series (top).
  * Per-channel state time-series (bottom) — one curve per declared
    channel.  Channel names are discovered from the snapshot's
    `channels` array on the first payload; new channels appearing
    later (post-hot-patch) extend the plot.

The snapshot doesn't carry per-channel error directly (error =
setpoint - current is computed in tick() and published on
drive.errors but not retained in module state).  We plot the channel's
current value, which is the half of the equation HomeostaticDrive
actually owns; the published error gets visible through urgency
which is the max-normalized error magnitude.
"""
from __future__ import annotations

import itertools
from typing import Dict, List

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


# Reuse channel colours across runs by hashing the channel name into a
# fixed palette — keeps the legend stable when channels reorder.
_PALETTE = [
    (255, 120, 120), (120, 220, 255), (255, 215,  90),
    (170, 255, 130), (255, 160, 220), (200, 200, 200),
    (130, 200, 255), (255, 180,  90),
]
def _palette_for(name: str) -> tuple[int, int, int]:
    return _PALETTE[abs(hash(name)) % len(_PALETTE)]


BUFFER_SIZE = 600


class _ChannelsPlot(QWidget):
    """Per-channel `current` value over time, dynamically discovering channel names."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._plot = pg.PlotWidget(title="Per-channel current value")
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left",   "current")
        self._plot.setLabel("bottom", "tick (recent)")
        self._plot.addLegend(offset=(-10, 10))
        layout.addWidget(self._plot)

        self._curves: Dict[str, pg.PlotDataItem] = {}
        self._buffers: Dict[str, np.ndarray] = {}
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def _ensure(self, name: str) -> None:
        if name in self._curves:
            return
        rgb = _palette_for(name)
        self._curves[name] = self._plot.plot(
            pen=pg.mkPen(*rgb, width=1.5), name=name,
        )
        self._buffers[name] = np.full(BUFFER_SIZE, np.nan)

    def update_payload(self, snapshot: dict) -> None:
        channels = snapshot.get("channels") if isinstance(snapshot, dict) else None
        if not isinstance(channels, list):
            return
        # Roll every existing buffer one column.
        for name in self._buffers:
            self._buffers[name] = np.roll(self._buffers[name], -1)
            self._buffers[name][-1] = np.nan
        # Update with whatever channels the snapshot contains this tick.
        for c in channels:
            if not isinstance(c, dict):
                continue
            name = str(c.get("name", ""))
            if not name:
                continue
            self._ensure(name)
            try:
                self._buffers[name][-1] = float(c.get("current", np.nan))
            except (TypeError, ValueError):
                self._buffers[name][-1] = np.nan
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        for name, curve in self._curves.items():
            curve.setData(self._buffers[name], connect="finite")


class DriveInspector(QWidget):
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

        self._urgency = MultiSeriesPlot(
            [Series("urgency", "urgency", (255, 120, 120), width=2.0)],
            title="Drive urgency",
            y_label="urgency [0,1]",
        )
        self._channels = _ChannelsPlot()

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(self._urgency)
        v.addWidget(self._channels)
        v.setSizes([260, 420])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._urgency.update_payload(snapshot)
        self._channels.update_payload(snapshot)
