"""BearingSeekLoop dashboard — the duck's seek loop and its chase (2026-10-02).

The seek loop holds ONE target.  A small thing the cloud attends is fixed in the odometry frame by dead reckoning and
homed to after the cloud closes; the outcome loop's need can RENEW a dropped target; a MOVER (a young, isolated cluster
confirmed by its own prediction) is CHASED at its predicted position, and a lost chase can be kept in MEMORY and
re-acquired.  Its need (value) races play in the arbiter; its honest signal is the range left.  A target ends by
arrival, by forgetting (unseen too long), by the progress forget (walked half a metre without closing), at contact, or
by a yield near tall structure.

snapshot (diag_snapshot): seen, target, value, range, arrivals, forgets, renewals, refixes, cx/cy (the published
  unit bearing, + right / + forward), tx/ty (the target, odometry frame), pose [x, y, yaw], conf, target_src,
  pull, chase {chasing, coasting, have_cand, mover_seen, cand_n, cand_x, cand_y, vx, vy, have_memory, mem_x, mem_y,
  last_miss, last_speed, last_decision, lost_now, yield_live}, counts {...}, progress {walked, best_range}

Panels:
  * Body-frame plan, forward UP: the held target (its needle, coloured by source), the chase candidate with its
    velocity, the remembered mover; a banner names the mode.
  * value / range / pull / confidence over time.
  * Readout: every way a target starts and ends.
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._kv_readout import AMBER, GREEN, GREY, MAGENTA, Banner, KVReadout, flag, num
from ._multi_series import MultiSeriesPlot, Series

_SRC = {0: "—", 1: "a sighting", 2: "the renewal", 3: "a mover that stopped"}
_RMAX = 2.5


def _to_body(px, py, pyaw, x, y):
    """An odometry-frame point in the body frame, on screen: (right, forward)."""
    dx, dy = x - px, y - py
    fwd = math.cos(pyaw) * dx + math.sin(pyaw) * dy
    left = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
    return -left, fwd


class _SeekPlan(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("idle")
        layout.addWidget(self._banner)
        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setXRange(-_RMAX, _RMAX)
        self._plot.setYRange(-1.0, _RMAX)
        self._plot.setLabel("bottom", "right (m)")
        self._plot.setLabel("left", "forward (m)")
        th = np.linspace(0, 2 * math.pi, 90)
        for r in (0.5, 1.0, 1.5, 2.0, 2.5):
            self._plot.plot(r * np.cos(th), r * np.sin(th), pen=pg.mkPen(55, 55, 55, width=1, style=Qt.PenStyle.DotLine))
        self._plot.plot([0, 0], [0, _RMAX], pen=pg.mkPen(60, 90, 60, width=1))
        self._plot.plot([-0.06, 0, 0.06, -0.06], [-0.08, 0.12, -0.08, -0.08], pen=pg.mkPen(120, 220, 120, width=2))
        self._needle = self._plot.plot([0, 0], [0, 0], pen=pg.mkPen(120, 120, 120, width=3))
        self._tip = pg.ScatterPlotItem(size=14, pen=None)
        self._cand = pg.ScatterPlotItem(size=16, symbol="star", pen=None, brush=pg.mkBrush(213, 95, 213, 230))
        self._vel = self._plot.plot([], [], pen=pg.mkPen(213, 95, 213, width=2))
        self._mem = pg.ScatterPlotItem(size=14, symbol="x", pen=pg.mkPen(237, 161, 0, width=2), brush=None)
        for it in (self._tip, self._cand, self._mem):
            self._plot.addItem(it)
        self._text = pg.TextItem(color=(220, 220, 220), anchor=(0, 1))
        self._plot.addItem(self._text)
        layout.addWidget(self._plot, 1)
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
        s = self._latest
        ch = s.get("chase") or {}
        pose = s.get("pose") or [0.0, 0.0, 0.0]
        px, py, pyaw = float(pose[0]), float(pose[1]), float(pose[2])
        have = bool(s.get("target"))
        rng = num(s, "range")
        src = int(num(s, "target_src"))

        if ch.get("chasing"):
            self._banner.set(f"CHASING a mover — {int(ch.get('cand_n', 0))} sightings, "
                             f"{math.hypot(ch.get('vx', 0), ch.get('vy', 0)):.2f} m/s", MAGENTA)
            col = (213, 95, 213)
        elif ch.get("coasting"):
            self._banner.set("COASTING on a lost mover's prediction", AMBER)
            col = (237, 161, 0)
        elif have:
            seen = "in view" if s.get("seen") else "remembered"
            self._banner.set(f"SEEKING {rng:.2f} m — from {_SRC.get(src, src)}, {seen}", GREEN)
            col = (27, 175, 122)
        elif ch.get("have_memory"):
            self._banner.set("no target — a lost mover held in memory", AMBER)
            col = (120, 120, 120)
        else:
            self._banner.set("idle — no target", GREY)
            col = (120, 120, 120)
        if ch.get("yield_live"):
            self._banner.setText(self._banner.text() + "   · yielded near tall structure")

        if have:
            ex, ey = _to_body(px, py, pyaw, float(s.get("tx", 0.0)), float(s.get("ty", 0.0)))
            v = max(0.0, min(1.0, num(s, "value")))
            c = tuple(int(60 + (k - 60) * (0.4 + 0.6 * v)) for k in col)
            self._needle.setData([0, ex], [0, ey])
            self._needle.setPen(pg.mkPen(*c, width=3))
            self._tip.setData([ex], [ey], brush=pg.mkBrush(*c, 255))
            self._text.setText(f"{rng:.2f} m")
            self._text.setPos(ex + 0.05, ey + 0.05)
        else:
            self._needle.setData([0, 0], [0, 0])
            self._tip.setData([], [])
            self._text.setText("")

        if ch.get("have_cand"):
            cx, cy = _to_body(px, py, pyaw, float(ch.get("cand_x", 0)), float(ch.get("cand_y", 0)))
            self._cand.setData([cx], [cy])
            vx, vy = float(ch.get("vx", 0)), float(ch.get("vy", 0))
            # the velocity turned into the body frame, drawn as one second of travel
            fwd = math.cos(pyaw) * vx + math.sin(pyaw) * vy
            left = -math.sin(pyaw) * vx + math.cos(pyaw) * vy
            self._vel.setData([cx, cx - left], [cy, cy + fwd])
        else:
            self._cand.setData([], [])
            self._vel.setData([], [])

        if ch.get("have_memory"):
            mx, my = _to_body(px, py, pyaw, float(ch.get("mem_x", 0)), float(ch.get("mem_y", 0)))
            self._mem.setData([mx], [my])
        else:
            self._mem.setData([], [])


def _rows(s: dict):
    ch = s.get("chase") or {}
    ct = s.get("counts") or {}
    pr = s.get("progress") or {}
    return [
        ("target", flag(s, "target") + f"   from {_SRC.get(int(num(s, 'target_src')), '?')}"),
        ("in view", flag(s, "seen")),
        ("value (need)", f"{num(s, 'value'):.3f}"),
        ("confidence", f"{num(s, 'conf'):.3f}"),
        ("range left", f"{num(s, 'range'):.2f} m"),
        ("progress", f"walked {float(pr.get('walked', 0)):.2f} m, best range {float(pr.get('best_range', -1)):.2f} m"),
        ("pull", f"{num(s, 'pull'):.2f}"),
        ("", ""),
        ("chasing", "yes" if ch.get("chasing") else " no"),
        ("candidate", f"{'yes' if ch.get('have_cand') else ' no'}  sightings {int(ch.get('cand_n', 0))}  "
                      f"v {math.hypot(ch.get('vx', 0), ch.get('vy', 0)):.2f} m/s"),
        ("last sighting", f"miss {float(ch.get('last_miss', 0)):.2f} m  speed {float(ch.get('last_speed', 0)):.2f}  "
                          f"decision {int(ch.get('last_decision', 0))}"),
        ("memory", "yes" if ch.get("have_memory") else " no"),
        ("", ""),
        ("arrivals", f"{int(num(s, 'arrivals'))}"),
        ("forgets", f"{int(num(s, 'forgets'))}  progress {int(ct.get('progress_forgets', 0))}  "
                    f"contact {int(ct.get('contact_forgets', 0))}"),
        ("renewals", f"{int(num(s, 'renewals'))}   refixes {int(num(s, 'refixes'))}   walk takes {int(ct.get('walk_takes', 0))}"),
        ("chases", f"{int(ct.get('chases', 0))}  lost {int(ct.get('lost', 0))}  stopped {int(ct.get('stopped', 0))}  "
                   f"re-acquired {int(ct.get('reacquired', 0))}"),
        ("yields", f"chase {int(ct.get('yielded', 0))} (drops {int(ct.get('yield_drops', 0))})  "
                   f"static {int(ct.get('static_yielded', 0))}"),
        ("candidates lost", f"replaced {int(ct.get('cand_replaced', 0))}  fast {int(ct.get('cand_fast', 0))}  "
                            f"still {int(ct.get('cand_still', 0))}  timeout {int(ct.get('cand_timeout', 0))}"),
    ]


class BearingSeekInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)
        self._plan = _SeekPlan()
        self._readout = KVReadout(_rows)
        self._series = MultiSeriesPlot(
            [
                Series("value", "need", (27, 175, 122), width=2.0),
                Series("range", "range left m", (74, 144, 217), width=1.5),
                Series("conf", "confidence", (150, 150, 150), width=1.2),
                Series("pull", "pull", (237, 161, 0), width=1.2),
                Series("chase.chasing", "chasing", (213, 95, 213), width=1.5),
            ],
            title="Seek need, range left, chase",
            y_label="value",
        )
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._plan)
        top.addWidget(self._readout)
        top.setSizes([560, 460])
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([520, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._plan.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._series.update_payload(snapshot)
