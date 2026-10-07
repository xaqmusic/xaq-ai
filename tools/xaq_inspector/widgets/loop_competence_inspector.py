"""LoopCompetence dashboard — is the world moving the way this loop says it will? (2026-10-02)

One LoopCompetence watches one loop (on the duck: seek, play) and the arbiter's gain for it.  Over each window in
which the loop drove continuously it checks the loop's own prediction once — "my objective improves while I act"
(seek: the range closes; play: novelty rises) — and keeps the success rate as an EMA c, with the tally beside it as a
Beta belief (a, b).  While the loop is not driving, c relaxes back toward its prior (0.5: not knowing) and the counts
toward the flat Beta.  It publishes p (c, or the Beta mean plus an optimism term) and sends 1 − p as an error, so the
voter and the arbiter trust a loop by how well its world behaves.

snapshot (diag_snapshot): competence, published, driving, checks, improvements, objective, gain, beta_a, beta_b

Panels:
  * The Beta belief over the loop's success rate, its mean marked; a banner says whether the loop drives.
  * competence / published / gain / objective over time.
  * Readout.
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._kv_readout import GREEN, GREY, Banner, KVReadout, flag, num
from ._multi_series import MultiSeriesPlot, Series

_P = np.linspace(0.001, 0.999, 200)


def _beta_pdf(a: float, b: float) -> np.ndarray:
    a, b = max(a, 1e-3), max(b, 1e-3)
    logb = math.lgamma(a) + math.lgamma(b) - math.lgamma(a + b)
    return np.exp((a - 1) * np.log(_P) + (b - 1) * np.log(1 - _P) - logb)


class _BetaView(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("—")
        layout.addWidget(self._banner)
        self._plot = pg.PlotWidget(title="belief over the loop's success rate (Beta a, b)")
        self._plot.setBackground("k")
        self._plot.setLabel("bottom", "share of checks where the objective improved")
        self._plot.setXRange(0, 1)
        self._curve = self._plot.plot([], [], pen=pg.mkPen(74, 144, 217, width=2), fillLevel=0,
                                      brush=pg.mkBrush(74, 144, 217, 60))
        self._mean = pg.InfiniteLine(angle=90, pen=pg.mkPen(255, 210, 0, width=2))
        self._pub = pg.InfiniteLine(angle=90, pen=pg.mkPen(27, 175, 122, width=1, style=Qt.PenStyle.DashLine))
        self._plot.addItem(self._mean)
        self._plot.addItem(self._pub)
        self._plot.addItem(pg.InfiniteLine(pos=0.5, angle=90, pen=pg.mkPen(80, 80, 80, style=Qt.PenStyle.DotLine)))
        layout.addWidget(self._plot, 1)
        self._latest = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(150)
        self._timer.timeout.connect(self._flush)
        self._timer.start()

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        s = self._latest
        if s.get("driving"):
            self._banner.set(f"DRIVING — gain {num(s, 'gain'):.2f}; the prediction is being checked", GREEN)
        else:
            self._banner.set("not driving — the belief relaxes toward 0.5", GREY)
        a, b = num(s, "beta_a", 1.0), num(s, "beta_b", 1.0)
        self._curve.setData(_P, _beta_pdf(a, b))
        self._mean.setValue(num(s, "competence", 0.5))
        self._pub.setValue(num(s, "published", 0.5))


def _rows(s: dict):
    checks = int(num(s, "checks"))
    imp = int(num(s, "improvements"))
    return [
        ("driving", flag(s, "driving")),
        ("arbiter gain", f"{num(s, 'gain'):.3f}"),
        ("competence c", f"{num(s, 'competence'):.3f}   (yellow line)"),
        ("published", f"{num(s, 'published'):.3f}   (green dashed; the error sent is 1 − this)"),
        ("checks", f"{checks}   improved {imp}" + (f"  ({100 * imp / checks:.0f} %)" if checks else "")),
        ("Beta a · b", f"{num(s, 'beta_a'):.2f} · {num(s, 'beta_b'):.2f}"),
        ("objective", f"{num(s, 'objective'):.4f}"),
    ]


class LoopCompetenceInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)
        self._beta = _BetaView()
        self._readout = KVReadout(_rows)
        self._series = MultiSeriesPlot(
            [
                Series("competence", "competence", (255, 210, 0), width=2.0),
                Series("published", "published", (27, 175, 122), width=1.5),
                Series("gain", "arbiter gain", (74, 144, 217), width=1.2),
                Series("objective", "objective", (200, 120, 200), width=1.2),
            ],
            title="Competence, what is published, and the gain it earns",
            y_label="value",
        )
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._beta)
        top.addWidget(self._readout)
        top.setSizes([600, 380])
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([380, 260])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._beta.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._series.update_payload(snapshot)
