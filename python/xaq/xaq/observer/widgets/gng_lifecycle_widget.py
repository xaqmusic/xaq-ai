"""
gng_lifecycle_widget.py — Per-EPM GNG lifecycle time-series panel.

Shows node_count and crystallization_ratio for each modality over time,
plus bake/mitosis event tick marks.
"""
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout

_COLORS = ["#00CFFF", "#FF9500", "#44FF88", "#FF4444", "#CC88FF", "#FFFF44"]


class GNGLifecycleWidget(QWidget):
    """Two stacked plots: GNG node counts and crystallization ratios per EPM."""

    def __init__(self, modalities, window_size=300, parent=None):
        super().__init__(parent)
        self._mods = list(modalities)
        self._win  = window_size
        self._node_hist  = {m: np.zeros(window_size) for m in modalities}
        self._cryst_hist = {m: np.zeros(window_size) for m in modalities}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        lbl_style = {"color": "#AAAAAA", "font-size": "8pt"}

        # Node count plot
        self._node_pw = pg.PlotWidget()
        self._node_pw.setBackground("#121212")
        self._node_pw.setLabel("left", "GNG Nodes", **lbl_style)
        self._node_pw.setLabel("bottom", "Ticks", **lbl_style)
        self._node_pw.showGrid(x=True, y=True, alpha=0.3)
        self._node_pw.addLegend(offset=(-10, 10))
        self._node_curves: dict = {}
        for i, m in enumerate(modalities):
            color = _COLORS[i % len(_COLORS)]
            self._node_curves[m] = self._node_pw.plot(
                pen=pg.mkPen(color=color, width=2), name=m
            )
        layout.addWidget(self._node_pw, stretch=1)

        # Crystallization ratio plot
        self._cryst_pw = pg.PlotWidget()
        self._cryst_pw.setBackground("#121212")
        self._cryst_pw.setYRange(0, 1)
        self._cryst_pw.setLabel("left", "Cryst. Ratio", **lbl_style)
        self._cryst_pw.setLabel("bottom", "Ticks", **lbl_style)
        self._cryst_pw.showGrid(x=True, y=True, alpha=0.3)
        self._cryst_curves: dict = {}
        dash = pg.Qt.QtCore.Qt.PenStyle.DashLine
        for i, m in enumerate(modalities):
            color = _COLORS[i % len(_COLORS)]
            self._cryst_curves[m] = self._cryst_pw.plot(
                pen=pg.mkPen(color=color, width=1.5, style=dash), name=m
            )
        layout.addWidget(self._cryst_pw, stretch=1)

    def update_stats(self, per_mod_stats: dict):
        """
        Call each UI frame with the latest per-modality stats dict.
        per_mod_stats: {modality: stats_dict}
        """
        for m in self._mods:
            s = per_mod_stats.get(m, {})

            arr = self._node_hist[m]
            arr[:-1] = arr[1:]
            arr[-1] = s.get("gng_nodes", s.get("node_count", 0))
            self._node_curves[m].setData(arr)

            arr = self._cryst_hist[m]
            arr[:-1] = arr[1:]
            arr[-1] = s.get("crystallization_ratio", 0.0)
            self._cryst_curves[m].setData(arr)
