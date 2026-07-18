"""Run-and-Tumble dashboard — the maze klino (E. coli methylation reflex).

The bug can't read the scalar scent gradient's DIRECTION, so it RUNS (keep heading,
thrust) and modulates the per-tick chance of a TUMBLE (random reorient) by the real
bacterial mechanism: a leaky METHYLATION baseline that tracks the recently-experienced
scent.  The methylation level is a PREDICTION of expected scent; the gap between the
live scent and that baseline is the prediction ERROR that drives tumbling:

  * scent rising ABOVE baseline  → positive error → LOW p_tumble → keep running uphill;
  * scent falling BELOW baseline → negative error → HIGH p_tumble → tumble & scramble.

Run length is not a clock and nothing is learned — it EMERGES from the gradient.

Panels:
  * STATE — the current committed action (RUN green / TUMBLE orange).
  * p_tumble GAUGE — the per-tick tumble probability as a bar (0..tumble_max), with the
    `error` readout that drives it.
  * scent vs baseline — a rolling time-series overlaying `smax` (live scent) and
    `baseline` (the methylation tracking it).  The GAP between the two IS the prediction
    error that modulates the tumble rate.
  * Counts — lifetime runs / tumbles / forced (stuck-triggered) tumbles.
"""
from __future__ import annotations

from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QLabel,
    QProgressBar,
    QSplitter,
    QVBoxLayout,
    QWidget,
)


_TUMBLE_MAX = 0.5  # gauge full-scale (matches the module's default tumble_max clamp)


class _StatePanel(QWidget):
    """The big committed-action readout (RUN / TUMBLE)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self._state = QLabel("—")
        self._state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._state.setStyleSheet("color: #888; font-size: 40px; font-weight: bold;")
        layout.addWidget(self._state, 1)

        self._error = QLabel("error: —")
        self._error.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._error.setStyleSheet("color: #ddd; font-size: 16px;")
        layout.addWidget(self._error)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        action = int(snapshot.get("action", 0) or 0)
        if action == 0:
            self._state.setText("RUN")
            self._state.setStyleSheet("color: #50e070; font-size: 40px; font-weight: bold;")
        else:
            self._state.setText("TUMBLE")
            self._state.setStyleSheet("color: #ffa040; font-size: 40px; font-weight: bold;")
        err = float(snapshot.get("error", 0.0) or 0.0)
        # positive error (rising scent) is good → green; negative (falling) → orange
        col = "#80ffa0" if err >= 0.0 else "#ff9060"
        self._error.setText(f"prediction error  (scent − baseline): {err:+.3f}")
        self._error.setStyleSheet(f"color: {col}; font-size: 16px;")


class _TumbleGauge(QWidget):
    """Per-tick tumble PROBABILITY as a bar (0..tumble_max)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        title = QLabel("p(tumble)  per tick")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)

        self._bar = QProgressBar()
        self._bar.setRange(0, 1000)              # 0.000 .. tumble_max mapped to 0..1000
        self._bar.setTextVisible(True)
        self._bar.setFormat("%.3f" % 0.0)
        self._bar.setStyleSheet(
            "QProgressBar { background: #1a1a1a; border: 1px solid #333; height: 26px;"
            " text-align: center; color: #fff; }"
            " QProgressBar::chunk { background: #ffa040; }"
        )
        layout.addWidget(self._bar)

        self._readout = QLabel("—")
        self._readout.setStyleSheet("color: #ccc; font-family: Monospace; font-size: 12px;")
        layout.addWidget(self._readout)
        layout.addStretch(1)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        p = float(snapshot.get("p_tumble", 0.0) or 0.0)
        frac = max(0.0, min(1.0, p / _TUMBLE_MAX if _TUMBLE_MAX > 0 else 0.0))
        self._bar.setValue(int(round(frac * 1000)))
        self._bar.setFormat(f"{p:.3f}")
        base = float(snapshot.get("baseline", 0.0) or 0.0)
        smax = float(snapshot.get("smax", 0.0) or 0.0)
        # capability = self-reported SENSORY precision → the L2 arbiter's g_prag_klino.
        # cap = current-smell / eat_scent (calibrated once it has eaten; else / speak bootstrap).
        cap = float(snapshot.get("cap", 0.0) or 0.0)
        eat_scent = float(snapshot.get("eat_scent", 0.0) or 0.0)
        calibrated = bool(snapshot.get("have_eat_scent", False))
        tag = "eat-cal" if calibrated else "bootstrap"
        self._readout.setText(
            f"scent {smax:7.3f}   baseline {base:7.3f}   p_tumble {p:.3f}\n"
            f"capability {cap:5.3f} → arbiter g_prag_klino   (eat_scent {eat_scent:.3f}, {tag})"
        )


class _Counts(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet("color: #ddd; font-family: Monospace; font-size: 13px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._lbl, 1)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        s = snapshot
        rows = [
            ("runs", int(s.get("runs", 0) or 0)),
            ("tumbles", int(s.get("tumbles", 0) or 0)),
            ("forced", int(s.get("forced", 0) or 0)),
            ("error", float(s.get("error", 0.0) or 0.0)),
            ("vx", float(s.get("vx", 0.0) or 0.0)),
            ("vy", float(s.get("vy", 0.0) or 0.0)),
        ]
        txt = "\n".join(
            f"{k:>9}: {v:8d}" if isinstance(v, int) else f"{k:>9}: {v:8.3f}"
            for k, v in rows
        )
        self._lbl.setText(txt)


class _ScentBaselineSeries(QWidget):
    """Rolling time-series overlaying scent (smax) and its methylation baseline.

    The vertical GAP between the two curves is the prediction error that drives the
    tumble rate: scent above baseline → keep running; scent below → tumble.
    """

    def __init__(self, buffer_size: int = 600, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._plot = pg.PlotWidget(title="scent vs methylation baseline  (gap = prediction error)")
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left", "scent")
        self._plot.setLabel("bottom", "tick (recent)")
        self._plot.addLegend(offset=(-10, 10))
        self._scent = self._plot.plot(pen=pg.mkPen(120, 255, 140, width=1.5), name="scent")
        self._base = self._plot.plot(pen=pg.mkPen(255, 160, 64, width=1.5), name="baseline")
        layout.addWidget(self._plot)

        self._buf_scent = deque([np.nan] * buffer_size, maxlen=buffer_size)
        self._buf_base = deque([np.nan] * buffer_size, maxlen=buffer_size)
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        try:
            self._buf_scent.append(float(snapshot.get("smax", 0.0) or 0.0))
        except (TypeError, ValueError):
            self._buf_scent.append(np.nan)
        try:
            self._buf_base.append(float(snapshot.get("baseline", 0.0) or 0.0))
        except (TypeError, ValueError):
            self._buf_base.append(np.nan)
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        self._scent.setData(np.asarray(self._buf_scent, dtype=float), connect="finite")
        self._base.setData(np.asarray(self._buf_base, dtype=float), connect="finite")


class RunTumbleInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._state = _StatePanel()
        self._gauge = _TumbleGauge()
        self._counts = _Counts()
        self._series = _ScentBaselineSeries()

        # top row: STATE | p(tumble) gauge | counts
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._state)
        top.addWidget(self._gauge)
        top.addWidget(self._counts)
        top.setSizes([300, 360, 240])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([300, 260])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._state.update_payload(snapshot)
        self._gauge.update_payload(snapshot)
        self._counts.update_payload(snapshot)
        self._series.update_payload(snapshot)
