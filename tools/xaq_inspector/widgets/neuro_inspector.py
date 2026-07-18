"""NeurochemState dashboard.

Three panels:
  * Modulator time-series — DA, serotonin, DA baseline EMA, reward signal.
  * Pending events bar chart — current-tick pending counts (hits, misses,
    bricks, wall_stuck, whisker_bump).
  * Cumulative counters readout — total hits / misses / bricks since boot.

The reward-signal series isn't in the snapshot directly — it's the
derived (dopamine - da_baseline_ema), what NeurochemState publishes on
neuro.state and what every downstream learning module credits against.
We compute it client-side per payload.
"""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGridLayout, QLabel, QSplitter, QVBoxLayout, QWidget,
)
import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer

from ._multi_series import MultiSeriesPlot, Series


PENDING_KEYS = [
    ("pending_hit_count",    "hits"),
    ("pending_miss_count",   "misses"),
    ("pending_brick_count",  "bricks"),
    ("pending_wall_stuck",   "wall_stuck"),
    ("pending_whisker_bump", "whisker_bump"),
]


class _PendingBars(QWidget):
    """Bar chart of the current-tick pending event counts."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Pending events (this tick)")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=False, y=True, alpha=0.2)
        self._plot.setMouseEnabled(x=False, y=False)
        x = list(range(len(PENDING_KEYS)))
        self._bar = pg.BarGraphItem(
            x=x, height=[0] * len(PENDING_KEYS), width=0.7,
            brush=pg.mkBrush(180, 220, 80, 220),
        )
        self._plot.addItem(self._bar)
        ax = self._plot.getAxis("bottom")
        ax.setTicks([list(zip(x, [lbl for _, lbl in PENDING_KEYS]))])
        self._plot.setYRange(0, 1)
        layout.addWidget(self._plot)

        self._latest_heights = [0] * len(PENDING_KEYS)
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        heights = []
        for key, _ in PENDING_KEYS:
            try:
                heights.append(float(snapshot.get(key, 0) or 0))
            except (TypeError, ValueError):
                heights.append(0.0)
        self._latest_heights = heights
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        x = list(range(len(PENDING_KEYS)))
        self._bar.setOpts(x=x, height=self._latest_heights, width=0.7)
        max_h = max(self._latest_heights + [1.0])
        self._plot.setYRange(0, max_h * 1.1)


class _CumulativeReadout(QWidget):
    """Compact text panel of running counters."""

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
        rows = [
            ("total hits",    snapshot.get("total_hits",   0)),
            ("total misses",  snapshot.get("total_misses", 0)),
            ("total bricks",  snapshot.get("total_bricks", 0)),
            ("DA baseline",   snapshot.get("da_baseline_ema", 0.0)),
            ("DA",            snapshot.get("dopamine",     0.0)),
            ("serotonin",     snapshot.get("serotonin",    0.0)),
        ]
        lines = []
        for k, v in rows:
            try:
                if isinstance(v, float) or (isinstance(v, (int,)) and abs(v) < 1):
                    lines.append(f"{k:>14}: {float(v):.4f}")
                else:
                    lines.append(f"{k:>14}: {int(v)}")
            except (TypeError, ValueError):
                lines.append(f"{k:>14}: ?")
        self._lbl.setText("\n".join(lines))


class NeuroInspector(QWidget):
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

        self._modulators = MultiSeriesPlot(
            [
                Series("dopamine",        "DA",        (120, 200, 255), width=2.0),
                Series("serotonin",       "5-HT",      (200, 130, 255), width=1.5),
                Series("da_baseline_ema", "DA bsline", (140, 140, 200),
                       width=1.0, style=Qt.PenStyle.DashLine),
                Series("_reward_signal",  "reward",    (255, 215,  60), width=1.5),
            ],
            title="Neurochem modulators",
            y_label="value",
        )

        self._pending = _PendingBars()
        self._readout = _CumulativeReadout()

        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.addWidget(self._pending)
        bot.addWidget(self._readout)
        bot.setSizes([700, 300])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(self._modulators)
        v.addWidget(bot)
        v.setSizes([460, 220])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        # Compute the published reward signal: DA - DA_baseline_EMA.
        try:
            rs = float(snapshot.get("dopamine", 0.0)) \
                - float(snapshot.get("da_baseline_ema", 0.0))
        except (TypeError, ValueError):
            rs = 0.0
        snap = dict(snapshot)
        snap["_reward_signal"] = rs

        self._modulators.update_payload(snap)
        self._pending.update_payload(snap)
        self._readout.update_payload(snap)
