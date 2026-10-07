"""JointSensorimotorBridge dashboard — what the motor brain senses, tick by tick (2026-10-02).

The bridge pairs each motor's last command with its sensed position and the position's change, and appends the
SENSE slots (the host's `reality.proprio.sense` block: on the duck's walker the ToF summary in the body frame, the
contact share, the gaze and range senses, the head's attitude...).  That vector is the whole of what the MotorEPMv2
downstream of it predicts and acts on — if a signal is not here, the motor brain cannot use it (CLAUDE.md §1 step 2).

snapshot (diag_snapshot): joints [{name, pos, act, delta}], sense [...], load_slots, group_size, outputs,
  load_topic, have_proprio, have_load, publishes, proprio_in, action_in

Panels:
  * Per motor: sensed position, last command, and change (×10) as grouped bars.
  * The sense slots now (bars) and their recent history (a rolling heat strip, one row per slot).
  * Readout.
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._kv_readout import KVReadout, dark_diverging, flag, num

_HIST = 300


class _JointBars(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._plot = pg.PlotWidget(title="per motor: position (blue) · command (amber) · change ×10 (green)")
        self._plot.setBackground("k")
        self._plot.setYRange(-1.1, 1.1)
        self._plot.showGrid(y=True, alpha=0.2)
        self._bars = [pg.BarGraphItem(x=[], height=[], width=0.25, brush=c) for c in
                      ((74, 144, 217), (237, 161, 0), (27, 175, 122))]
        for b in self._bars:
            self._plot.addItem(b)
        layout.addWidget(self._plot, 1)
        self._names: list[str] = []
        self._latest = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(75)
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
        joints = self._latest.get("joints") or []
        x = np.arange(len(joints), dtype=float)
        pos = [float(j.get("pos", 0)) for j in joints]
        act = [float(j.get("act", 0)) for j in joints]
        dlt = [10.0 * float(j.get("delta", 0)) for j in joints]
        for b, off, h in zip(self._bars, (-0.27, 0.0, 0.27), (pos, act, dlt)):
            b.setOpts(x=x + off, height=h, width=0.25)
        names = [str(j.get("name", i)) for i, j in enumerate(joints)]
        if names != self._names:
            self._names = names
            self._plot.getAxis("bottom").setTicks([[(i, n) for i, n in enumerate(names)]])


class _SenseView(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._bars_plot = pg.PlotWidget(title="sense slots now")
        self._bars_plot.setBackground("k")
        self._bars_plot.showGrid(y=True, alpha=0.2)
        self._bars = pg.BarGraphItem(x=[], height=[], width=0.7, brush=(200, 120, 200))
        self._bars_plot.addItem(self._bars)
        layout.addWidget(self._bars_plot, 1)
        self._heat_plot = pg.PlotWidget(title="sense slots, recent history (row = slot, newest right)")
        self._heat_plot.setBackground("k")
        self._img = pg.ImageItem()
        self._img.setColorMap(dark_diverging())
        self._heat_plot.addItem(self._img)
        layout.addWidget(self._heat_plot, 1)
        self._hist: np.ndarray | None = None
        self._latest = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._flush)
        self._timer.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        sense = snapshot.get("sense") or []
        if sense:
            v = np.asarray(sense, dtype=float)
            if self._hist is None or self._hist.shape[1] != v.size:
                self._hist = np.full((_HIST, v.size), np.nan)
            self._hist = np.roll(self._hist, -1, axis=0)
            self._hist[-1] = v
        self._latest = snapshot
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        sense = self._latest.get("sense") or []
        self._bars.setOpts(x=np.arange(len(sense)), height=[float(v) for v in sense], width=0.7)
        if self._hist is not None:
            img = np.nan_to_num(self._hist, nan=0.0)
            lim = max(1e-6, float(np.nanmax(np.abs(img))))
            self._img.setImage(img, levels=(-lim, lim), autoLevels=False)


def _rows(s: dict):
    return [
        ("outputs", ", ".join(s.get("outputs") or [])),
        ("group size", f"{int(num(s, 'group_size', 1))}"),
        ("sense slots", f"{int(num(s, 'load_slots'))}   from {s.get('load_topic') or '—'}"),
        ("proprio · load", f"{flag(s, 'have_proprio')} · {flag(s, 'have_load')}"),
        ("publishes", f"{int(num(s, 'publishes'))}"),
        ("inputs", f"proprio {int(num(s, 'proprio_in'))}   actions {int(num(s, 'action_in'))}"),
    ]


class JointBridgeInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)
        self._joints = _JointBars()
        self._sense = _SenseView()
        self._readout = KVReadout(_rows)
        left = QSplitter(Qt.Orientation.Vertical)
        left.addWidget(self._joints)
        left.addWidget(self._readout)
        left.setSizes([360, 200])
        h = QSplitter(Qt.Orientation.Horizontal)
        h.addWidget(left)
        h.addWidget(self._sense)
        h.setSizes([440, 600])
        outer.addWidget(h, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._joints.update_payload(snapshot)
        self._sense.update_payload(snapshot)
        self._readout.update_payload(snapshot)
