"""TLE / novelty-threshold time-series with bake & mitosis event markers.

Port of v3 src/native/widgets/visualizer.py PredictorGraph.

Plots three curves:
  * raw per-tick TLE (cyan, bold)        — `last_tle` field
  * EMA-smoothed TLE  (cyan, thin)       — `ema_tle` field
  * novelty threshold (orange, dashed)   — `novelty_threshold_now`
  * raw quant_error  (green, thin)       — `last_quant_error` field
                                           (= v3's "loss" curve)

Backend support shipped alongside this widget; older brain binaries
that don't expose the per-tick fields fall back to EMA-only.

Event markers fire whenever:
  * gng.last_step_baked  changes  → gold line
  * gng.mitosis_count    changes  → magenta line
  * gng.last_death_step  changes  → red line

Vertical lines accumulate as the buffer fills; the oldest are pruned
once the X-coord scrolls off the left edge.
"""
from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QVBoxLayout, QWidget


BUFFER_SIZE = 600    # ~10 s at 60 Hz, ~20 s at 30 Hz


class EpmTlePlot(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._plot = pg.PlotWidget(title="TLE & Novelty Threshold")
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left",   "value")
        self._plot.setLabel("bottom", "tick (recent)")
        self._plot.addLegend(offset=(-10, 10))
        layout.addWidget(self._plot)

        self._raw_curve   = self._plot.plot(
            pen=pg.mkPen("c", width=2), name="TLE (raw)")
        self._ema_curve   = self._plot.plot(
            pen=pg.mkPen((100, 200, 220), width=1, style=Qt.PenStyle.DashLine),
            name="TLE (ema)")
        self._loss_curve  = self._plot.plot(
            pen=pg.mkPen((150, 255, 150), width=1), name="quant_error")
        self._thresh_curve = self._plot.plot(
            pen=pg.mkPen((255, 165, 0), width=1, style=Qt.PenStyle.DashLine),
            name="novelty threshold")

        self._buf_raw    = np.full(BUFFER_SIZE, np.nan)
        self._buf_ema    = np.full(BUFFER_SIZE, np.nan)
        self._buf_loss   = np.full(BUFFER_SIZE, np.nan)
        self._buf_thresh = np.full(BUFFER_SIZE, np.nan)
        # Event markers: (x_pos_in_buffer, line_item).  Lines move left every
        # tick so they track the data point they were planted next to.
        self._events: deque[tuple[float, pg.InfiniteLine]] = deque(maxlen=200)

        # Track last-seen markers so we know when to add a new line.
        self._last_baked_step:  Optional[int] = None
        self._last_mitosis:     Optional[int] = None
        self._last_death_step:  Optional[int] = None

        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(50)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        gng = snapshot.get("gng") or {}
        if not isinstance(gng, dict):
            return

        # Roll buffers + event lines one column to the left.
        self._buf_raw    = np.roll(self._buf_raw, -1)
        self._buf_ema    = np.roll(self._buf_ema, -1)
        self._buf_loss   = np.roll(self._buf_loss, -1)
        self._buf_thresh = np.roll(self._buf_thresh, -1)
        for i in range(len(self._events)):
            x, line = self._events[i]
            new_x = x - 1
            line.setPos(new_x)
            self._events[i] = (new_x, line)
        # Drop event lines that scrolled off the left.
        while self._events and self._events[0][0] < -2:
            _, line = self._events.popleft()
            self._plot.removeItem(line)

        # `last_tle` / `last_quant_error` land via the backend extension
        # shipped with this widget; older binaries omit them, in which
        # case the curves stay NaN and pyqtgraph just doesn't draw them.
        def _f(key: str) -> float:
            try:
                v = snapshot.get(key)
                return float(v) if v is not None else float("nan")
            except (TypeError, ValueError):
                return float("nan")
        self._buf_raw[-1]    = _f("last_tle")
        self._buf_ema[-1]    = _f("ema_tle")
        self._buf_loss[-1]   = _f("last_quant_error")
        self._buf_thresh[-1] = _f("novelty_threshold_now")

        # Detect events.  GNG counters are cumulative-style integers; a
        # change from one tick to the next means an event fired.
        baked_step  = int(gng.get("last_step_baked", 0))
        mitosis_cnt = int(gng.get("mitosis_count",   0))
        death_step  = int(gng.get("last_death_step", 0))
        if self._last_baked_step is not None and baked_step != self._last_baked_step:
            self._add_marker((255, 215, 0))     # gold
        if self._last_mitosis    is not None and mitosis_cnt != self._last_mitosis:
            self._add_marker((255,   0, 255))   # magenta
        if self._last_death_step is not None and death_step != self._last_death_step:
            self._add_marker((220,  60,  60))   # red
        self._last_baked_step = baked_step
        self._last_mitosis    = mitosis_cnt
        self._last_death_step = death_step

        self._dirty = True

    def _add_marker(self, rgb: tuple[int, int, int]) -> None:
        x = float(BUFFER_SIZE - 1)
        line = pg.InfiniteLine(
            pos=x, angle=90,
            pen=pg.mkPen(*rgb, width=1, style=Qt.PenStyle.DashLine),
        )
        self._plot.addItem(line)
        self._events.append((x, line))

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        # connect="finite" leaves NaN gaps unconnected — older binaries
        # that don't supply last_tle/last_quant_error simply render
        # nothing for those rows.
        self._raw_curve.setData(   self._buf_raw,    connect="finite")
        self._ema_curve.setData(   self._buf_ema,    connect="finite")
        self._loss_curve.setData(  self._buf_loss,   connect="finite")
        self._thresh_curve.setData(self._buf_thresh, connect="finite")
