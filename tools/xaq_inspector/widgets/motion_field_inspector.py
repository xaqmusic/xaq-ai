"""MotionField dashboard — the always-on motion sensor: where the world was just seen to be empty (2026-10-04).

MotionField keeps the last memory_s of the ToF's rays, in the odometry frame.  A new off-floor return is MOTION EVIDENCE
when a ray passed within near_m of that point a moment ago and went on past it: something now stands where the world
was just empty.  Evidence points cluster into BLOBS (min_points within cluster_m in one cast); blobs associated cast to
cast become TRACKS with a world velocity, published after persist_casts casts (and, with min_travel_m, once they have
travelled).  The most salient track is published; with cloud_mover_topic set the cloud's mover passes through when no
track of its own is (the union), and with vouch_m only once this module's evidence vouches for it.

snapshot (diag_snapshot): evidence, blobs, tracks, published, casts, published_ticks, passed_through, vouched, refused,
  track_list [{id, x, y, vx, vy, casts, points, salience}...], evidence_xyz [[x, y, z]...], pose [x, y, yaw],
  published_index

Panels:
  * Plan view from the body, forward UP: this cast's evidence points (orange), every track as a ring with its velocity
    (1 s ahead) as a line, the published track yellow; range rings every 0.5 m; the sensor's 45 deg cone.
  * Counts over time: evidence points, blobs, tracks, and whether a track is published.
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

_HALF = math.radians(22.5)


def _to_body(pose, x, y):
    """Odometry frame -> the body's frame on screen (forward up, right to the right)."""
    px, py, yaw = pose
    dx, dy = np.asarray(x) - px, np.asarray(y) - py
    c, s = math.cos(yaw), math.sin(yaw)
    fwd, left = c * dx + s * dy, -s * dx + c * dy
    return -left, fwd


class _MotionPlan(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("no motion")
        layout.addWidget(self._banner)
        self._plot = pg.PlotWidget(title="motion from the body (forward up)")
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setXRange(-2.0, 2.0)
        self._plot.setYRange(-0.5, 2.5)
        for r in (0.5, 1.0, 1.5, 2.0, 2.5):
            a = np.linspace(0, 2 * math.pi, 120)
            self._plot.plot(r * np.cos(a), r * np.sin(a), pen=pg.mkPen(55, 55, 55, width=1, style=Qt.PenStyle.DotLine))
        for sgn in (-1, 1):
            self._plot.plot([0, 2.5 * math.sin(sgn * _HALF)], [0, 2.5 * math.cos(_HALF)],
                            pen=pg.mkPen(90, 90, 120, width=1, style=Qt.PenStyle.DashLine))
        self._plot.plot([-0.06, 0.0, 0.06, -0.06], [-0.08, 0.1, -0.08, -0.08], pen=pg.mkPen(120, 220, 120, width=2))
        self._ev = pg.ScatterPlotItem(size=6, pen=None, brush=pg.mkBrush(235, 140, 40, 220))
        self._tracks = pg.ScatterPlotItem(size=16, symbol="o", brush=None, pen=pg.mkPen(200, 200, 210, width=2))
        self._pub = pg.ScatterPlotItem(size=22, symbol="o", brush=None, pen=pg.mkPen(255, 210, 0, width=3))
        for it in (self._ev, self._tracks, self._pub):
            self._plot.addItem(it)
        self._vel: list[pg.PlotDataItem] = []
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
        pose = s.get("pose") or [0.0, 0.0, 0.0]
        ev = s.get("evidence_xyz") or []
        tl = s.get("track_list") or []
        pi = int(num(s, "published_index", -1))
        if ev:
            ex, ey = _to_body(pose, [p[0] for p in ev], [p[1] for p in ev])
            self._ev.setData(np.atleast_1d(ex), np.atleast_1d(ey))
        else:
            self._ev.setData([], [])
        for item in self._vel:
            self._plot.removeItem(item)
        self._vel = []
        if tl:
            tx, ty = _to_body(pose, [t["x"] for t in tl], [t["y"] for t in tl])
            tx, ty = np.atleast_1d(tx), np.atleast_1d(ty)
            self._tracks.setData(tx, ty)
            hx, hy = _to_body(pose, [t["x"] + t["vx"] for t in tl], [t["y"] + t["vy"] for t in tl])
            for i in range(len(tl)):
                colour = (255, 210, 0) if i == pi else (200, 200, 210)
                self._vel.append(self._plot.plot([tx[i], np.atleast_1d(hx)[i]], [ty[i], np.atleast_1d(hy)[i]],
                                                 pen=pg.mkPen(*colour, width=2)))
        else:
            self._tracks.setData([], [])
        if 0 <= pi < len(tl):
            t = tl[pi]
            px_, py_ = _to_body(pose, [t["x"]], [t["y"]])
            self._pub.setData(np.atleast_1d(px_), np.atleast_1d(py_))
            sp = math.hypot(t["vx"], t["vy"])
            self._banner.set(f"MOTION — track {t['id']}, {sp:.2f} m/s, {t['casts']} casts, salience {t['salience']:.2f}", GREEN)
        else:
            self._pub.setData([], [])
            self._banner.set(f"no published track — {len(tl)} tentative, {len(ev)} evidence points this cast", GREY)


def _rows(s: dict):
    casts = int(num(s, "casts"))
    pub_t = int(num(s, "published_ticks"))
    return [
        ("published now", flag(s, "published")),
        ("evidence · blobs · tracks", f"{int(num(s, 'evidence'))} · {int(num(s, 'blobs'))} · {int(num(s, 'tracks'))}"),
        ("casts processed", f"{casts}"),
        ("ticks with a published track", f"{pub_t}"),
        ("cloud mover passed through", f"{int(num(s, 'passed_through'))}"),
        ("vouched · refused", f"{int(num(s, 'vouched'))} · {int(num(s, 'refused'))}"),
    ]


class MotionFieldInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)
        self._plan = _MotionPlan()
        self._readout = KVReadout(_rows)
        self._series = MultiSeriesPlot(
            [
                Series("evidence", "evidence points", (235, 140, 40), width=1.2),
                Series("blobs", "blobs", (74, 144, 217), width=1.2),
                Series("tracks", "tracks", (200, 200, 210), width=1.5),
                Series("published", "published", (255, 210, 0), width=2.0),
            ],
            title="What the motion sensor sees, cast by cast",
            y_label="count",
        )
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._plan)
        top.addWidget(self._readout)
        top.setSizes([600, 380])
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([420, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._plan.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._series.update_payload(snapshot)
