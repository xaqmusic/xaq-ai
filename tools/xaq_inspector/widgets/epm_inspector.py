"""Composite EPM inspector — v3-style "deep inspector" for one EPM module.

Layout:

    +------------------+--------------------+
    |                  |  TLE & threshold   |
    |   GNG canvas     |  time-series       |
    |                  |                    |
    +------------------+--------------------+
    |  Encoder strip   |  GNG lifecycle     |
    |  (rolling)       |                    |
    +------------------+--------------------+

All four panels receive the same diag payload and refresh independently
(each owns its own QTimer at 30-50 Hz redraw).  Adding more panels later
is a single-line extension to update_payload.
"""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGridLayout, QLabel, QSplitter, QTabWidget, QVBoxLayout, QWidget,
)

from .epm_canvas        import EpmCanvas
from .epm_tle_plot      import EpmTlePlot
from .epm_encoder_strip import EpmEncoderStrip
from .epm_lifecycle     import EpmLifecyclePlot
from .epm_pca_scatter   import EpmPcaScatter


class EpmInspector(QWidget):
    def __init__(self, module_id: str, module_type: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        self._header = QLabel(f"{module_id}  ({module_type})")
        self._header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(self._header)

        # Top splitter: GNG canvas (left) | TLE plot (right)
        top_split = QSplitter(Qt.Orientation.Horizontal)
        self._gng = EpmCanvas(module_id, module_type)
        self._tle = EpmTlePlot()
        top_split.addWidget(self._gng)
        top_split.addWidget(self._tle)
        top_split.setSizes([520, 520])

        # Bottom splitter: encoder pane (tabbed) | lifecycle (right)
        bot_split = QSplitter(Qt.Orientation.Horizontal)
        # Encoder pane has two tabs: rolling time-series heatmap and
        # PCA scatter — same data, two perspectives.  Tabs keep both
        # available without crowding the 2x2 dashboard.
        self._enc_tabs = QTabWidget()
        self._enc  = EpmEncoderStrip()
        self._pca  = EpmPcaScatter()
        self._enc_tabs.addTab(self._enc, "Stream")
        self._enc_tabs.addTab(self._pca, "PCA scatter")
        self._life = EpmLifecyclePlot()
        bot_split.addWidget(self._enc_tabs)
        bot_split.addWidget(self._life)
        bot_split.setSizes([520, 520])

        # Vertical stacker for the two splitters
        v_split = QSplitter(Qt.Orientation.Vertical)
        v_split.addWidget(top_split)
        v_split.addWidget(bot_split)
        v_split.setSizes([460, 320])
        outer.addWidget(v_split, 1)

    # Single fan-out point: route every payload to every panel.
    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        self._gng.update_payload(tick_id, snapshot)
        self._tle.update_payload(tick_id, snapshot)
        self._enc.update_payload(tick_id, snapshot)
        self._pca.update_payload(tick_id, snapshot)
        self._life.update_payload(tick_id, snapshot)
