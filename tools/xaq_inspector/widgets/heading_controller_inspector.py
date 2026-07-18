"""HeadingController dashboard — the Cell's LEARNED heading-following locomotor.

The HeadingController turns a desired egocentric heading [cx=+right, cy=+forward]
into motor commands, learning TWO things from the body's own dynamics (no oracle):

  1. TURN forward-model  k_body = EMA(|yaw_rate| / |steer|): how fast THIS body
     rotates per unit steer.  It then INVERTS it — steer = turn_fraction*bearing/
     k_body — to command a calibrated turn that nulls the heading error without a
     hand-set P-gain (so it transfers across bodies/friction).

  2. ADVANCE policy (when learn_advance) V[err_bin][thrust]: a UCB-selected thrust
     per heading-error bin, rewarded by FORWARD progress along the commanded
     heading (max(0,vel_fwd)*cos(bearing) - effort*|vel_fwd|), food-INDEPENDENT.
     "brake-turn-charge" EMERGES: forward wins when facing (bin 0), stop/brake wins
     off-axis (let the steer rotate to face, then charge).

Panels:
  * Heading dial — egocentric needle to the desired heading (forward = up); colored
    by the current error bin; dim when nav is off (no confident heading).
  * Advance policy grid — V[err_bin][thrust] heatmap; the argmax per row = the
    learned policy (▶), the live (bin,thrust) cell is ringed.  Watch forward light
    up at bin 0 and brake/stop off-axis as it learns.
  * Time-series — k_body (turn model, converges), thrust, vel_fwd (the reward's
    velocity), |steer|.
  * Readout — bearing°, gains, command, reward, coverage, current bin/act.
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (QGridLayout, QLabel, QSplitter, QVBoxLayout, QWidget)

from ._multi_series import MultiSeriesPlot, Series


def _val_color(v: float, vmax: float) -> str:
    """Diverging heatmap: positive = green, negative = red, ~0 = dark."""
    if vmax < 1e-6:
        return "#1e1e1e"
    frac = max(-1.0, min(1.0, v / vmax))
    if frac >= 0:
        g = int(50 + 180 * frac)
        return f"#{20:02x}{g:02x}{40:02x}"
    f = -frac
    r = int(50 + 180 * f)
    return f"#{r:02x}{30:02x}{40:02x}"


class _HeadingDial(QWidget):
    """Top-down egocentric dial: needle to the desired heading (forward = up)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        title = QLabel("Desired heading (forward = up)")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setXRange(-1.3, 1.3)
        self._plot.setYRange(-1.3, 1.3)
        self._plot.hideAxis("bottom")
        self._plot.hideAxis("left")
        th = np.linspace(0, 2 * math.pi, 64)
        self._plot.plot(np.cos(th), np.sin(th), pen=pg.mkPen(80, 80, 80, width=1))
        self._plot.plot([0, 0], [0, 1.15], pen=pg.mkPen(60, 90, 60, width=1))  # forward tick
        self._needle = self._plot.plot([0, 0], [0, 0], pen=pg.mkPen(255, 215, 60, width=3))
        self._tip = pg.ScatterPlotItem(size=12, brush=pg.mkBrush(255, 215, 60, 255), pen=None)
        self._plot.addItem(self._tip)
        layout.addWidget(self._plot)

        self._latest = None
        self._dirty = False
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
        bearing = float(snap.get("bearing", 0.0) or 0.0)   # theta/pi, [-1,1]
        nav_on = bool(snap.get("nav_on", False))
        theta = bearing * math.pi
        ex, ey = math.sin(theta), math.cos(theta)          # cx=+right, cy=+forward
        if not nav_on:
            self._needle.setData([0, 0], [0, 0]); self._tip.setData([0], [0]); return
        # colour by alignment: facing (|bearing|~0) green → off-axis red
        f = min(1.0, abs(bearing))
        col = (int(40 + 200 * f), int(220 - 160 * f), 50)
        self._needle.setData([0, ex], [0, ey], pen=pg.mkPen(*col, width=3))
        self._tip.setData([ex], [ey], brush=pg.mkBrush(*col, 255))


class _AdvanceGrid(QWidget):
    """V[err_bin][thrust] heatmap — the learned advance policy."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        self._title = QLabel("Learned advance policy  V[heading-error bin][thrust]")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        outer.addWidget(self._title)
        self._grid = QGridLayout()
        self._grid.setSpacing(2)
        outer.addLayout(self._grid, 1)
        hint = QLabel("▶ = learned best (argmax) · ◉ = live cell · green=+ red=−")
        hint.setStyleSheet("color: #888; font-size: 10px;")
        outer.addWidget(hint)

        self._cells: dict[tuple[int, int], QLabel] = {}
        self._n_bins = 0
        self._n_acts = 0
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

    def _rebuild(self, nb: int, na: int) -> None:
        while self._grid.count():
            item = self._grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._cells.clear()
        self._n_bins, self._n_acts = nb, na
        # column headers: thrust levels (reverse … stop … forward)
        def act_label(a: int) -> str:
            if na <= 1: return "fwd"
            if a == 0: return "rev"
            if a == na - 1: return "fwd"
            if na % 2 == 1 and a == na // 2: return "stop"
            return f"t{a}"
        self._grid.addWidget(self._hdr("err\\thr"), 0, 0)
        for a in range(na):
            self._grid.addWidget(self._hdr(act_label(a)), 0, a + 1)
        for b in range(nb):
            lo = int(b * 180 / nb); hi = int((b + 1) * 180 / nb)
            tag = "face" if b == 0 else ("back" if b == nb - 1 else "")
            self._grid.addWidget(self._hdr(f"{lo}-{hi}° {tag}".strip()), b + 1, 0)
            for a in range(na):
                lbl = QLabel("·")
                lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
                lbl.setMinimumSize(58, 30)
                self._cells[(b, a)] = lbl
                self._grid.addWidget(lbl, b + 1, a + 1)

    def _hdr(self, txt: str) -> QLabel:
        l = QLabel(txt)
        l.setStyleSheet("color: #bbb; font-size: 10px; font-family: Monospace;")
        l.setAlignment(Qt.AlignmentFlag.AlignCenter)
        return l

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        snap = self._latest
        if not snap.get("learn_advance"):
            self._title.setText("Advance policy: hand-coded cos gate (learn_advance=false)")
            return
        nb = int(snap.get("n_err_bins", 0) or 0)
        na = int(snap.get("n_thrust_acts", 0) or 0)
        vals = snap.get("adv_value") or []
        if nb <= 0 or na <= 0 or len(vals) < nb * na:
            return
        if nb != self._n_bins or na != self._n_acts:
            self._rebuild(nb, na)
        cur_b = int(snap.get("err_bin", -1)); cur_a = int(snap.get("thrust_act", -1))
        vmax = max(1e-6, max(abs(float(v)) for v in vals))
        for b in range(nb):
            row = [float(vals[b * na + a]) for a in range(na)]
            best_a = int(np.argmax(row))
            for a in range(na):
                v = row[a]
                mark = "▶ " if a == best_a else ""
                ring = (b == cur_b and a == cur_a)
                lbl = self._cells[(b, a)]
                lbl.setText(f"{mark}{v:+.3f}")
                border = "2px solid #ffd23c" if ring else "1px solid #333"
                lbl.setStyleSheet(
                    f"background:{_val_color(v, vmax)}; color:#eee; border:{border};"
                    f" font-family:Monospace; font-size:11px;")


class _Readout(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet("color: #ddd; font-family: Monospace; font-size: 12px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._lbl, 1)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        s = snapshot
        bearing = float(s.get("bearing", 0.0) or 0.0)
        rows = [
            ("bearing°",   bearing * 180.0),
            ("nav_on",     1.0 if s.get("nav_on") else 0.0),
            ("k_body",     float(s.get("k_body", 0.0) or 0.0)),
            ("gain",       float(s.get("gain", 0.0) or 0.0)),
            ("steer",      float(s.get("steer", 0.0) or 0.0)),
            ("thrust",     float(s.get("thrust", 0.0) or 0.0)),
        ]
        if s.get("learn_advance"):
            rows += [
                ("vel_fwd",    float(s.get("vel_fwd", 0.0) or 0.0)),
                ("adv_reward", float(s.get("adv_reward", 0.0) or 0.0)),
                ("adv_spread", float(s.get("adv_spread", 0.0) or 0.0)),
                ("coverage",   float(s.get("adv_cov", 0.0) or 0.0)),
                ("err_bin",    float(s.get("err_bin", -1))),
                ("thrust_act", float(s.get("thrust_act", -1))),
            ]
        self._lbl.setText("\n".join(f"{k:>11}: {v:8.3f}" for k, v in rows))


class HeadingControllerInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._dial = _HeadingDial()
        self._grid = _AdvanceGrid()
        self._series = MultiSeriesPlot(
            [
                Series("k_body",  "k_body (turn model)", (255, 215, 60), width=2.0),
                Series("thrust",  "thrust",              (120, 200, 255), width=1.5),
                Series("vel_fwd", "vel_fwd",             (120, 255, 140), width=1.5),
                Series("steer",   "steer",               (255, 120, 140), width=1.0),
            ],
            title="Turn model + commands + velocity",
            y_label="value",
        )
        self._readout = _Readout()

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._dial)
        top.addWidget(self._grid)
        top.setSizes([360, 540])

        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.addWidget(self._series)
        bot.addWidget(self._readout)
        bot.setSizes([680, 320])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(bot)
        v.setSizes([380, 280])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._dial.update_payload(snapshot)
        self._grid.update_payload(snapshot)
        self._series.update_payload(snapshot)
        self._readout.update_payload(snapshot)
