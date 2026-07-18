"""ActionDecoder dashboard — the coxswain's decision (the cognitive actor).

The ActionDecoder is the brain's policy.  In coxswain mode it scores depth-H
plans over its learned forward model P(s'|s,a) by the PREFERRED OBSERVATION
(active-inference C prior: scent proximity + green loom), and decodes the best
joint action into a turn (differential) + thrust (common-mode) it publishes to
the MotorEPM.  Opaque until now: this shows what it's reasoning about.

Three panels:
  * Action plot — a 2-D map of the joint action space (x = turn, +right;
    y = thrust, +forward).  The COMMITTED action (held this option) and the
    action the H-step PLAN recommends are both plotted, so you can see what the
    bug intends and whether commit ≠ plan (mid-commit hold).
  * Target / learning time-series — the preferred-observation targets (scent +
    green) and the per-state node value + forward-model surprise (action_tle).
    Homing shows up as these targets rising.
  * Readout — belief node, mode flags, targets, plan/commit decode, model size.
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


class _ActionPlot(QWidget):
    """turn × thrust action space: committed vs planned action."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        title = QLabel("Action  (x = turn →,  y = thrust ↑)")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setXRange(-4.5, 4.5)
        self._plot.setYRange(-4.5, 4.5)
        self._plot.addLine(x=0, pen=pg.mkPen(80, 80, 80))
        self._plot.addLine(y=0, pen=pg.mkPen(80, 80, 80))
        # planned action (hollow, where the plan WANTS to go)
        self._plan = pg.ScatterPlotItem(size=18, symbol="o",
                                        pen=pg.mkPen(120, 200, 255, width=2),
                                        brush=None)
        # committed action (filled, what the body is DOING)
        self._commit = pg.ScatterPlotItem(size=15, symbol="x",
                                          pen=pg.mkPen(255, 215, 60, width=3))
        self._plot.addItem(self._plan)
        self._plot.addItem(self._commit)
        layout.addWidget(self._plot)

        self._latest = None
        self._dirty = False
        self._max = 4.0
        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
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

        def f(k):
            try:
                return float(snap.get(k, 0.0) or 0.0)
            except (TypeError, ValueError):
                return 0.0

        ct, cth = f("commit_turn"), f("commit_thrust")
        pt, pth = f("plan_turn"), f("plan_thrust")
        self._commit.setData([ct], [cth])
        self._plan.setData([pt], [pth])
        # expand range if the action exceeds the default ±4 box
        m = max(4.0, abs(ct), abs(cth), abs(pt), abs(pth)) * 1.1
        if abs(m - self._max) > 0.5:
            self._max = m
            self._plot.setXRange(-m, m)
            self._plot.setYRange(-m, m)


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

        def f(k):
            try:
                return float(snapshot.get(k, 0.0) or 0.0)
            except (TypeError, ValueError):
                return 0.0

        def i(k):
            try:
                return int(snapshot.get(k, 0) or 0)
            except (TypeError, ValueError):
                return 0

        mode = []
        if snapshot.get("efe_select"):
            mode.append("EFE")
        if snapshot.get("joint_action"):
            mode.append("joint")
        ph = i("plan_horizon")
        if ph > 1:
            mode.append(f"plan H={ph}")
        mode_str = "+".join(mode) if mode else "legacy TD"

        pH = f("plan_entropy")
        explore_lbl = ("EXPLORING" if pH > 0.6 else "confident" if pH < 0.25 else "mixed")
        rows = [
            ("mode",        mode_str),
            ("explore/exploit", f"H={pH:.2f} ({explore_lbl})  conf={f('plan_confidence'):.2f}"),
            ("state node",  i("state_node")),
            ("proprio",     i("proprio_node")),
            ("n_actions",   i("n_actions")),
            ("",            ""),
            ("pref scent",  f"{f('pref_obs'):.4f}"),
            ("green loom",  f"{f('green_obs'):.4f}"),
            ("node value",  f"{f('node_value'):.4f}"),
            ("plan value",  f"{f('plan_value'):.4f}"),
            ("",            ""),
            ("commit idx",  i("commit_action_idx")),
            ("commit turn", f"{f('commit_turn'):.3f}"),
            ("commit thr",  f"{f('commit_thrust'):.3f}"),
            ("plan idx",    i("plan_action_idx")),
            ("plan turn",   f"{f('plan_turn'):.3f}"),
            ("plan thr",    f"{f('plan_thrust'):.3f}"),
            ("",            ""),
            ("action_tle",  f"{f('action_tle'):.4f}"),
            ("interest",    f"{f('interest'):.4f}"),
            ("fwd model",   i("fwd_model_size")),
            ("obs states",  i("obs_states_known")),
        ]
        lines = []
        for k, v in rows:
            if k == "":
                lines.append("")
            else:
                lines.append(f"{k:>12}: {v}")
        self._lbl.setText("\n".join(lines))


class ActionPlanInspector(QWidget):
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

        self._action = _ActionPlot()
        self._series = MultiSeriesPlot(
            [
                Series("pref_obs",   "scent",      (120, 255, 140), width=2.0),
                Series("green_obs",  "green",      (140, 220,  80), width=1.5),
                Series("node_value", "node value", (255, 215,  60), width=1.5),
                Series("action_tle", "action-TLE", (255, 120, 120), width=1.0),
            ],
            title="Preferred-obs targets + learning",
            y_label="value",
        )
        self._readout = _Readout()

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._action)
        top.addWidget(self._readout)
        top.setSizes([520, 480])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([380, 300])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._action.update_payload(snapshot)
        self._series.update_payload(snapshot)
        self._readout.update_payload(snapshot)
