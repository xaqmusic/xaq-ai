"""StalkLoop dashboard — the hunter's ambush: where does the mover pass, and is waiting there worth it? (§17.113, 2026-10-12)

A chase that loses its mover rarely finds it again, and the body is slower than the train; a hunter with a slow body waits
where the prey passes.  StalkLoop files each confirmed mover sighting under the motion map's node for it (a PASSING
PLACE).  A place the mover came back through (min_passes) earns a prediction: how often the mover returns (the median
interval between passes) and P(a wait there sees it), a Beta over its waits.  After a lost chase, with nothing else held,
it sends the seek loop to the best place and waits as long as that place's own return predicts.  A wait that sees the
mover pays; one that outlasts the prediction fails, and failed waits make the hunter bored without a timer.

snapshot (diag_snapshot): state (0 idle, 1 going, 2 waiting), node, need, ambushes, waits_paid, waits_failed,
  places [[node, sightings, passes, interval_s, p, x, y]...]   (odometry frame, metres)

Panels:
  * The passing places from above, in the odometry frame: each a ring sized by its passes and coloured by
    P(a wait sees the mover); the place chosen for the ambush yellow.  A banner says idle, going or waiting.
  * need and state over time.
  * Readout, with each place's sightings, passes, return interval and P.
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._kv_readout import AMBER, GREEN, GREY, Banner, KVReadout, num
from ._multi_series import MultiSeriesPlot, Series

_STATES = {0: "idle", 1: "going to a passing place", 2: "waiting"}


def _places(s: dict) -> list[list]:
    return [p for p in (s.get("places") or []) if isinstance(p, (list, tuple)) and len(p) >= 7]


class _PlaceMap(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("idle")
        layout.addWidget(self._banner)
        self._plot = pg.PlotWidget(title="passing places (odometry frame, metres)")
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.showGrid(x=True, y=True, alpha=0.2)
        self._cmap = pg.ColorMap([0.0, 0.5, 1.0], [(90, 90, 110), (74, 144, 217), (27, 200, 122)])
        self._rings = pg.ScatterPlotItem(symbol="o", pen=None)
        self._chosen = pg.ScatterPlotItem(size=30, symbol="o", brush=None, pen=pg.mkPen(255, 210, 0, width=3))
        self._plot.addItem(self._rings)
        self._plot.addItem(self._chosen)
        layout.addWidget(self._plot, 1)
        self._latest = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(200)
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
        st = int(num(s, "state"))
        colour = {0: GREY, 1: AMBER, 2: GREEN}.get(st, GREY)
        self._banner.set(f"{_STATES.get(st, '?').upper()}" + (f" — place {int(num(s, 'node'))}, need {num(s, 'need'):.2f}"
                                                              if st else f" — {int(num(s, 'ambushes'))} ambushes so far"),
                         colour)
        pl = _places(s)
        if not pl:
            self._rings.setData([], [])
            self._chosen.setData([], [])
            return
        xs = np.array([float(p[5]) for p in pl])
        ys = np.array([float(p[6]) for p in pl])
        sizes = np.array([10 + 4 * min(int(p[2]), 8) for p in pl])
        brushes = [pg.mkBrush(*self._cmap.map(float(p[4]), mode="byte")[:3], 200) for p in pl]
        self._rings.setData(xs, ys, size=sizes, brush=brushes)
        node = int(num(s, "node", -1))
        ch = [i for i, p in enumerate(pl) if int(p[0]) == node]
        if st and ch:
            self._chosen.setData([xs[ch[0]]], [ys[ch[0]]])
        else:
            self._chosen.setData([], [])


def _rows(s: dict):
    rows = [
        ("state", _STATES.get(int(num(s, "state")), "?")),
        ("place chosen", f"{int(num(s, 'node', -1))}"),
        ("need", f"{num(s, 'need'):.3f}   (to the seek loop)"),
        ("ambushes", f"{int(num(s, 'ambushes'))}"),
        ("waits paid · failed", f"{int(num(s, 'waits_paid'))} · {int(num(s, 'waits_failed'))}"),
        ("", ""),
    ]
    for p in _places(s):
        interval = float(p[3])
        rows.append((f"place {int(p[0])}",
                     f"{int(p[1])} seen, {int(p[2])} passes, "
                     + (f"back every {interval:.0f} s" if interval > 0 else "return unknown")
                     + f", P {float(p[4]):.2f}"))
    return rows


class StalkLoopInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)
        self._map = _PlaceMap()
        self._readout = KVReadout(_rows)
        self._series = MultiSeriesPlot(
            [
                Series("need", "need", (255, 210, 0), width=2.0),
                Series("state", "state (0 idle, 1 going, 2 waiting)", (74, 144, 217), width=1.2),
            ],
            title="The ambush's need, and what the hunter is doing",
            y_label="value",
        )
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._map)
        top.addWidget(self._readout)
        top.setSizes([600, 420])
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([420, 220])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._map.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._series.update_payload(snapshot)
