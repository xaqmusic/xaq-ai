"""Premotor dashboard — graded-policy actor-critic state.

Four panels:
  * Intent distribution stream — rolling N_intents × time heatmap built
    from last_distribution.  Lets the user see policy entropy / mode
    collapse / sharpening at a glance.
  * Weighted accel + entropy + DA time-series — the scalar policy
    outputs Premotor publishes (last_accel, last_entropy) plus the
    DA modulator (the temperature gain).
  * Intent histograms — chosen_intent_counts vs bc_intent_counts as
    paired bars, one cluster per intent index.
  * W matrix heatmap — the (N_intents × latent_dim) Hebbian weight
    matrix.  Drives the policy logits; learning shows up as cells
    drifting toward extreme values.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


# Black-centered diverging cmap, same as encoder strip.
def _signed_cmap() -> pg.ColorMap:
    return pg.ColorMap(
        pos=np.array([0.0, 0.45, 0.5, 0.55, 1.0]),
        color=np.array([
            [ 40, 110, 220, 255],
            [ 20,  35,  60, 255],
            [ 10,  10,  14, 255],
            [ 60,  30,  20, 255],
            [220,  80,  60, 255],
        ], dtype=np.uint8),
    )


# ---------------------------------------------------------------------------
# Intent-distribution rolling stream
# ---------------------------------------------------------------------------

class _IntentDistStream(QWidget):
    BUFFER = 300

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Intent distribution stream — awaiting payload")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._view = pg.PlotWidget()
        self._view.setBackground("k")
        self._view.setMouseEnabled(x=False, y=False)
        self._view.setLabel("left",   "intent")
        self._view.setLabel("bottom", "tick (recent →)")
        self._image = pg.ImageItem(axisOrder="row-major")
        self._view.addItem(self._image)
        layout.addWidget(self._view)

        self._buf: Optional[np.ndarray] = None
        self._n_intents = 0
        self._dirty = False
        self._latest_tick = 0
        self._latest_entropy = 0.0
        self._cmap = pg.colormap.get("inferno")
        self._levels = (0.0, 1.0)  # distributions sum to 1; per-bin in [0,1]

        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        dist = snapshot.get("last_distribution") if isinstance(snapshot, dict) else None
        if not isinstance(dist, list) or len(dist) == 0:
            return
        try:
            vec = np.asarray(dist, dtype=float)
        except (TypeError, ValueError):
            return
        if self._buf is None or vec.size != self._n_intents:
            self._n_intents = int(vec.size)
            self._buf = np.full((self._n_intents, self.BUFFER), np.nan)
        self._buf = np.roll(self._buf, -1, axis=1)
        self._buf[:, -1] = vec
        self._latest_tick = tick_id
        try:
            self._latest_entropy = float(snapshot.get("last_entropy", 0.0) or 0.0)
        except (TypeError, ValueError):
            self._latest_entropy = 0.0
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._buf is None:
            return
        self._dirty = False
        img = np.nan_to_num(self._buf, nan=0.0)
        self._image.setImage(img, levels=self._levels, autoLevels=False)
        try:
            self._image.setLookupTable(self._cmap.getLookupTable(0.0, 1.0, 256))
        except Exception:
            pass
        self._title.setText(
            f"Intent distribution stream  —  N {self._n_intents}   "
            f"tick {self._latest_tick}   H {self._latest_entropy:.3f}"
        )


# ---------------------------------------------------------------------------
# Intent histograms
# ---------------------------------------------------------------------------

class _IntentHistograms(QWidget):
    """Paired bars: chosen vs BC counts per intent index."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Intent histograms (cumulative)")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=False, y=True, alpha=0.2)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setLabel("bottom", "intent")
        self._plot.addLegend(offset=(-10, 10))

        self._bar_chosen = pg.BarGraphItem(
            x=[], height=[], width=0.4,
            brush=pg.mkBrush(120, 200, 255, 220),
            name="chosen",
        )
        self._bar_bc = pg.BarGraphItem(
            x=[], height=[], width=0.4,
            brush=pg.mkBrush(255, 215,  90, 220),
            name="BC",
        )
        self._plot.addItem(self._bar_chosen)
        self._plot.addItem(self._bar_bc)
        # Hack legend entries since BarGraphItem doesn't auto-register.
        legend = self._plot.addLegend(offset=(-10, 10))
        layout.addWidget(self._plot)

        self._latest_chosen: list = []
        self._latest_bc: list = []
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(150)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        chosen = snapshot.get("chosen_intent_counts") or []
        bc     = snapshot.get("bc_intent_counts")     or []
        if isinstance(chosen, list):
            self._latest_chosen = [int(x) for x in chosen]
        if isinstance(bc, list):
            self._latest_bc = [int(x) for x in bc]
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        n = max(len(self._latest_chosen), len(self._latest_bc))
        if n == 0:
            return
        chosen = self._latest_chosen + [0] * (n - len(self._latest_chosen))
        bc     = self._latest_bc     + [0] * (n - len(self._latest_bc))
        x_chosen = [i - 0.2 for i in range(n)]
        x_bc     = [i + 0.2 for i in range(n)]
        self._bar_chosen.setOpts(x=x_chosen, height=chosen, width=0.4)
        self._bar_bc.setOpts(    x=x_bc,     height=bc,     width=0.4)
        ax = self._plot.getAxis("bottom")
        ax.setTicks([list(zip(range(n), [str(i) for i in range(n)]))])
        self._plot.setYRange(0, max(chosen + bc + [1]) * 1.1)


# ---------------------------------------------------------------------------
# W weights matrix heatmap
# ---------------------------------------------------------------------------

class _WeightsHeatmap(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("W matrix — awaiting payload")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._view = pg.PlotWidget()
        self._view.setBackground("k")
        self._view.setMouseEnabled(x=False, y=False)
        self._view.setLabel("left",   "intent (row)")
        self._view.setLabel("bottom", "latent dim (col)")
        self._image = pg.ImageItem(axisOrder="row-major")
        self._view.addItem(self._image)
        layout.addWidget(self._view)

        self._cmap = _signed_cmap()
        self._latest = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(150)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._latest = snapshot.get("W")
        self._dirty  = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        W = self._latest
        if not isinstance(W, dict) or "data" not in W:
            return
        try:
            r = int(W.get("rows", 0)); c = int(W.get("cols", 0))
            data = np.asarray(W.get("data") or [], dtype=float).reshape(r, c)
        except (TypeError, ValueError):
            return
        max_abs = max(1e-6, float(np.nanmax(np.abs(data))))
        levels = (-max_abs, max_abs)
        self._image.setImage(data, levels=levels, autoLevels=False)
        try:
            self._image.setLookupTable(self._cmap.getLookupTable(0.0, 1.0, 256))
        except Exception:
            pass
        self._title.setText(
            f"W matrix  —  ({r}×{c})   |max| {max_abs:.3f}"
        )


# ---------------------------------------------------------------------------
# Top-level inspector
# ---------------------------------------------------------------------------

class PremotorInspector(QWidget):
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

        self._dist     = _IntentDistStream()
        self._scalars  = MultiSeriesPlot(
            [
                Series("last_accel",   "accel",   (255, 120, 120), width=1.8),
                Series("last_entropy", "entropy", (120, 220, 255), width=1.5),
                Series("dopamine",     "DA",      (255, 215,  90), width=1.0,
                       style=Qt.PenStyle.DashLine),
                Series("urgency",      "urgency", (200, 100, 255), width=1.0,
                       style=Qt.PenStyle.DashLine),
                Series("last_alpha",   "alpha",   (180, 255, 180), width=1.0,
                       style=Qt.PenStyle.DashLine),
            ],
            title="Policy outputs + modulators",
            y_label="value",
        )
        self._hists  = _IntentHistograms()
        self._w_heat = _WeightsHeatmap()

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._dist)
        top.addWidget(self._scalars)
        top.setSizes([520, 520])

        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.addWidget(self._hists)
        bot.addWidget(self._w_heat)
        bot.setSizes([400, 640])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(bot)
        v.setSizes([400, 380])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._dist.update_payload(tick_id, snapshot)
        self._scalars.update_payload(snapshot)
        self._hists.update_payload(snapshot)
        self._w_heat.update_payload(snapshot)
