"""MotorEPMv2 dashboard — the gait dashboard plus the self-model and its priors (2026-10-02).

MotorEPMv2 kept MotorEPM's flat diag keys, so the picrawler's gait dashboard still reads it (first tab).  The duck's
v2 brains — the walker and the head — act through a different part of the module: an identified linear SELF-MODEL
x̂ = A·y + b (state rows × motors) and STATE PRIORS, set points on chosen state elements the controller descends
through the model's authority.  The second tab shows that part:

  * A as a heatmap: which motor moves which state element, and which way (the authority the priors act through).
  * The state now (bars), with each prior index's nominal target (yellow tick) and precision (tick width).
  * The motor output now (bars).
  * motor TLE, the prior's error, the output's size and the clip duty over time.

snapshot keys read here: A (column-major, rows_A × cols_A), rows_A, cols_A, gng.last_x (the state), prev_y (the
output), state_prior_idx / state_prior_tgt / state_prior_wts, state_prior_err, state_prior_active, motor_tle,
out_mag, clip_duty, state_prior_gate.
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QSplitter, QTabWidget, QVBoxLayout, QWidget

from ._kv_readout import GREEN, GREY, Banner, dark_diverging
from ._multi_series import MultiSeriesPlot, Series
from .motor_epm_inspector import MotorEpmInspector


class _SelfModelPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("—")
        layout.addWidget(self._banner)

        self._a_plot = pg.PlotWidget(title="self-model A (row = state, col = motor; red +, blue −)")
        self._a_plot.setBackground("k")
        self._a_img = pg.ImageItem()
        self._a_img.setColorMap(dark_diverging())
        self._a_plot.addItem(self._a_img)
        self._a_plot.invertY(True)
        self._a_plot.setLabel("bottom", "motor")
        self._a_plot.setLabel("left", "state element")

        self._x_plot = pg.PlotWidget(title="state now (bars) · prior targets (yellow; thicker = more precise)")
        self._x_plot.setBackground("k")
        self._x_plot.showGrid(y=True, alpha=0.2)
        self._x_bars = pg.BarGraphItem(x=[], height=[], width=0.7, brush=(74, 144, 217))
        self._x_plot.addItem(self._x_bars)
        self._tgt = pg.ScatterPlotItem(symbol="_", size=22, pen=None)
        self._x_plot.addItem(self._tgt)

        self._y_plot = pg.PlotWidget(title="motor output now")
        self._y_plot.setBackground("k")
        self._y_plot.setYRange(-1.1, 1.1)
        self._y_bars = pg.BarGraphItem(x=[], height=[], width=0.6, brush=(237, 161, 0))
        self._y_plot.addItem(self._y_bars)

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._a_plot)
        right = QSplitter(Qt.Orientation.Vertical)
        right.addWidget(self._x_plot)
        right.addWidget(self._y_plot)
        right.setSizes([300, 140])
        top.addWidget(right)
        top.setSizes([380, 620])

        self._series = MultiSeriesPlot(
            [
                Series("motor_tle", "motor TLE", (74, 58, 167), width=2.0),
                Series("state_prior_err", "prior error", (255, 210, 0), width=1.5),
                Series("out_mag", "output size", (237, 161, 0), width=1.2),
                Series("clip_duty", "clip duty", (224, 85, 85), width=1.2),
                Series("state_prior_gate", "pace gate", (27, 175, 122), width=1.2),
            ],
            title="Prediction error, the prior's error, and the output",
            y_label="value",
        )
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([460, 220])
        layout.addWidget(v, 1)

        self._latest = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(120)
        self._timer.timeout.connect(self._flush)
        self._timer.start()

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True
            self._series.update_payload(snapshot)

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        s = self._latest
        if s.get("state_prior_active"):
            self._banner.set(f"state prior ON — error {float(s.get('state_prior_err', 0)):.4f}, "
                             f"applied {int(s.get('state_prior_applied', 0))}", GREEN)
        else:
            self._banner.set("no state prior active", GREY)

        rows, cols = int(s.get("rows_A", 0) or 0), int(s.get("cols_A", 0) or 0)
        A = s.get("A") or []
        if rows > 0 and cols > 0 and len(A) == rows * cols:
            M = np.asarray(A, dtype=float).reshape(cols, rows)        # column-major: A[i + j*rows]
            lim = max(1e-6, float(np.max(np.abs(M))))
            self._a_img.setImage(M, levels=(-lim, lim), autoLevels=False)   # x = motor, y = state row

        x = ((s.get("gng") or {}).get("last_x")) or []
        self._x_bars.setOpts(x=np.arange(len(x)), height=[float(v) for v in x], width=0.7)
        idx = s.get("state_prior_idx") or []
        tgt = s.get("state_prior_tgt") or []
        wts = s.get("state_prior_wts") or []
        px, py, sizes = [], [], []
        for k, (i, t) in enumerate(zip(idx, tgt)):
            i = int(i)
            if i < 0:
                i += len(x)                                            # negative = from the end
            if 0 <= i < len(x):
                px.append(i)
                py.append(float(t))
                w = float(wts[k]) if k < len(wts) else 1.0
                sizes.append(14 + 10 * min(3.0, max(0.0, w)))
        self._tgt.setData(px, py, size=sizes, pen=pg.mkPen(255, 210, 0, width=3))
        y = s.get("prev_y") or []
        self._y_bars.setOpts(x=np.arange(len(y)), height=[float(v) for v in y], width=0.6)


class MotorEpmV2Inspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        self._tabs = QTabWidget()
        self._tabs.setStyleSheet("QTabWidget::pane { background:#1a1c20; border:1px solid #2c313a; }"
                                 "QTabBar::tab { background:#22252c; color:#bbb; padding:4px 12px; }"
                                 "QTabBar::tab:selected { background:#2a4060; color:#fff; }")
        self._gait = MotorEpmInspector(module_id, module_type)
        self._model = _SelfModelPanel()
        self._model.setStyleSheet("background:#1a1c20;")
        self._tabs.addTab(self._model, "self-model && priors")
        self._tabs.addTab(self._gait, "gait dashboard")
        outer.addWidget(self._tabs, 1)
        self._tab_chosen = False

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        if not self._tab_chosen:                    # a legged body (the picrawler) opens on its gait dashboard
            self._tab_chosen = True
            if int(snapshot.get("n_legs", 1) or 1) > 1:
                self._tabs.setCurrentWidget(self._gait)
        self._model.update_payload(snapshot)
        self._gait.update_payload(tick_id, snapshot)
