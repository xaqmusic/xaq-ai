"""MotorEPM dashboard — the homeokinetic sensorimotor self-model.

The Motor-EPM is the ONE working predictive loop: per leg it learns a forward
self-model  x̂(t+1) = A·y(t) + b  (motor → next sensor) and descends the
motor Time-Loop Error  ξ = x − x̂.  For the Cell this is a single 2-flagella
controller; the cognitive critic injects cog_steer (differential / turn) and
cog_thrust (common-mode / forward-reverse) on top of the alive HK swim.

Panels:
  * Drive + health time-series — motor-TLE (self-model surprise), fwd_v
    (controllability), and the cog_steer / cog_thrust the brain is injecting.
    When the brain is steering, these move; motor-TLE is the substrate's pulse.
  * Self-model pane (tabbed, borrowed from the perceptual EPM inspector):
      - PCA scatter — the sensor trajectory (actual x) with the model's 1-step
        prediction x̂ overlaid as the lone "node"; the gap IS the TLE, shown in
        the space the self-model lives in.  A frozen loop collapses to a point.
      - Model graph — the linear forward model A as a motor → sensor weight graph
        (the "knowledge graph" analog for a linear map).
      - A matrix — the raw A heatmap.
  * Readout — boredom / interest / hunger neuromodulators, the food bearing the
    state SHOULD encode (tc_x/tc_y), and the cog message counters (distinguish
    "brain publishes straight-0" from "brain not publishing").
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QTabWidget, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series
from .epm_pca_scatter import EpmPcaScatter
from .gait_raster import GaitRaster
from .motor_selfmodel_graph import MotorSelfModelGraph


class _AMatrix(QWidget):
    """Heatmap of the leg-0 forward self-model A (rows=sensor, cols=motor)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        title = QLabel("Self-model A  (motor → sensor, leg 0)")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)

        self._view = pg.GraphicsLayoutWidget()
        self._view.setBackground("k")
        self._vb = self._view.addViewBox()
        self._vb.setAspectLocked(False)
        self._vb.invertY(True)
        self._img = pg.ImageItem()
        self._vb.addItem(self._img)
        self._cmap = pg.colormap.get("inferno")
        self._img.setColorMap(self._cmap)
        layout.addWidget(self._view)

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

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        snap = self._latest
        rows = int(snap.get("rows_A", 0) or 0)
        cols = int(snap.get("cols_A", 0) or 0)
        flat = snap.get("A") or []
        if rows < 1 or cols < 1 or len(flat) < rows * cols:
            return
        # C++ Eigen is column-major: data is laid out column by column.
        arr = np.asarray(flat[: rows * cols], dtype=np.float32).reshape(
            (cols, rows)).T  # → (rows, cols)
        # symmetric scale around 0 so sign is readable
        amax = float(np.abs(arr).max()) or 1e-6
        # ImageItem expects (cols, rows) with default axisOrder; pass transposed
        self._img.setImage(arr.T, levels=(-amax, amax), autoLevels=False)


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

        def f(key):
            try:
                return float(snapshot.get(key, 0.0) or 0.0)
            except (TypeError, ValueError):
                return 0.0

        def i(key):
            try:
                return int(snapshot.get(key, 0) or 0)
            except (TypeError, ValueError):
                return 0

        rows = [
            ("motor-TLE",  f("motor_tle")),
            # 2026-08-05 — the commit loop.  commit_prec is the EARNED precision: >1 means
            # the body is predicting itself better than its own running average, which
            # shortens the commit window and slows its release.  Exactly 1.000 = lever off.
            # fwd_progress_ema is the INPUT to the whole commit chain: commit_ticks gates
            # on it and intent_err is measured from it.  Read it against intent_err to see
            # whether cprec is REGULATING or just tracking a constant offset.
            ("fwd_prog_ema", f("fwd_progress_ema")),
            ("intent_err",  f("intent_err")),
            ("commit_prec", f("commit_prec")),
            ("commit_boost", f("commit_boost")),
            ("explore_mult", f("explore_mult")),
            ("fwd_v",      f("fwd_v")),
            ("lateral_v",  f("lateral_v")),
            ("loop_gain",  f("loop_gain")),
            ("gait_coher", f("gait_coherence")),
            ("reset_rate", f("reset_rate")),
            ("reset_cnt",  i("reset_count")),
            ("since_rst",  i("ticks_since_reset")),
            ("cog_steer",  f("cog_steer")),
            ("steer msgs", i("cog_steer_msgs")),
            ("cog_thrust", f("cog_thrust")),
            ("thrust msgs", i("cog_thrust_msgs")),
            ("boredom",    f("boredom")),
            ("bored strk", i("boredom_streak")),
            ("interest",   f("interest")),
            ("hunger",     f("hunger")),
            ("tc_x (food R)", f("tc_x")),
            ("tc_y (food F)", f("tc_y")),
            ("n_legs",     i("n_legs")),
        ]
        lines = []
        for k, v in rows:
            if isinstance(v, int):
                lines.append(f"{k:>14}: {v}")
            else:
                lines.append(f"{k:>14}: {v:8.4f}")
        self._lbl.setText("\n".join(lines))


class MotorEpmInspector(QWidget):
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

        self._series = MultiSeriesPlot(
            [
                Series("motor_tle",  "motor-TLE",  (255, 120, 120), width=2.0),
                # The commit loop as a trace: watch commit_prec lead commit_boost, and
                # explore_mult fall as commit ramps.  If commit_prec sits flat at 1.0 the
                # lever is off; if it swings but commit_boost does not follow, the
                # precision is being computed and not consumed.
                Series("fwd_progress_ema", "fwd-prog-ema", (140, 255, 140), width=2.0),
                Series("intent_err",   "intent-err",   (255, 140, 255), width=1.5),
                Series("commit_prec",  "commit-prec",  (120, 220, 255), width=1.5),
                Series("commit_boost", "commit-boost", (255, 210, 120), width=1.5),
                Series("explore_mult", "explore",      (170, 170, 170), width=1.0),
                Series("fwd_v",      "fwd_v",      (120, 255, 140), width=1.5),
                Series("cog_steer",  "cog_steer",  (255, 215,  60), width=1.5),
                Series("cog_thrust", "cog_thrust", (120, 200, 255), width=1.5),
                Series("gait_coherence", "gait-coher", (180, 120, 255), width=2.0),
                Series("reset_rate",     "reset-rate", (255, 160,  60), width=2.0),
            ],
            title="Self-model health + cognitive drive",
            y_label="value",
        )
        # Self-model pane — tabbed, borrowing the perceptual EPM's PCA scatter.
        self._pca   = EpmPcaScatter()
        self._graph = MotorSelfModelGraph()
        self._amatrix = _AMatrix()
        self._raster = GaitRaster()
        self._model_tabs = QTabWidget()
        # Gait first: it is the tab you want open while WATCHING the robot, and the
        # only view that shows whether thrust is landing on the ground.
        self._model_tabs.addTab(self._raster,  "Gait raster")
        self._model_tabs.addTab(self._pca,     "Model PCA")
        self._model_tabs.addTab(self._graph,   "Model graph")
        self._model_tabs.addTab(self._amatrix, "A matrix")
        self._readout = _Readout()

        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.addWidget(self._model_tabs)
        bot.addWidget(self._readout)
        bot.setSizes([620, 380])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(self._series)
        v.addWidget(bot)
        v.setSizes([360, 360])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._series.update_payload(snapshot)
        self._raster.update_payload(snapshot)
        self._pca.update_payload(tick_id, snapshot)
        self._graph.update_payload(snapshot)
        self._amatrix.update_payload(snapshot)
        self._readout.update_payload(snapshot)
