"""CloudMap dashboard — the duck's ToF cloud, its things, and what it attends (2026-10-02).

CloudMap accumulates the ToF's returns into 4 cm voxels — a stop's cloud while the body stands, a walking cloud
translated by the odometry between stops — and reduces it to clusters by the STACK RULE: a cluster that tops out
under small_top and spans at most small_ext is a small thing, anything that keeps rising is structure.  The nearest
small thing is ATTENDED (its bearing drives the seek loop); a young, isolated cluster is the MOVER candidate (the
chase).  With free_rays on, every ray's free space is recorded per column, and with small_needs_top a cluster is
small only once a ray has passed over its top (the ten-minutes phase S1).

snapshot (diag_snapshot): open, walking, voxels, break, small, things, place, filed, cached, newfrac, revisit,
  attended (range), mover (range), mover_cands, vacated, voxel_m, vox [[ix, iy, iz, hits, mm]...],
  things_rows [[cx, cy, rng, ext, top, ncols, hits, small, seen_above, tall_near, age_w]...], attended_i,
  mover_row, cfg {...}, free_cols [[ix, iy, mm]...], body [x, y, yaw] (the cloud's frame)

Panels:
  * Plan view, forward UP, in the cloud's own frame: voxel columns coloured by height band (orange 2–20 cm = a thing
    could be here, blue = furniture height, pale = above); clusters as rings — SMALL (orange), small by shape but
    its top never seen (red, the S1 refusal), structure (grey); the ATTENDED thing (yellow), the MOVER (magenta);
    the body as a triangle.  Toggle the floor and the free-space ("top seen") layer.
  * Counts over time: small things, all clusters, mover candidates, the share of new voxels.
  * Readout: the cloud's state and the switches it runs with.
"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QSplitter, QVBoxLayout, QWidget

from ._kv_readout import BLUE, GREEN, GREY, Banner, KVReadout, flag, num
from ._multi_series import MultiSeriesPlot, Series

# height bands, as the static voxel viewer draws them (mm)
_FLOOR, _BREAK_HI, _FURN = 20, 200, 450
_C_FLOOR = (70, 70, 70, 120)
_C_BREAK = (235, 140, 40, 220)
_C_FURN = (70, 130, 220, 200)
_C_HIGH = (200, 200, 210, 150)


def _screen(x_fwd, y_left):
    """The cloud frame (x forward, y left) on screen: forward up, right to the right."""
    return -np.asarray(y_left), np.asarray(x_fwd)


class _CloudPlan(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("no cloud open")
        layout.addWidget(self._banner)

        row = QHBoxLayout()
        self._show_floor = QCheckBox("floor")
        self._show_free = QCheckBox("free space (top seen)")
        self._show_free.setToolTip("teal: the highest point a ray passed through each column without returning there "
                                   "(needs CloudMap free_rays and host --tof-free-rays)")
        for cb in (self._show_floor, self._show_free):
            cb.setStyleSheet("color:#bbb;")
            cb.toggled.connect(self._mark_dirty)
            row.addWidget(cb)
        row.addStretch(1)
        self._legend = QLabel(
            '<span style="color:#eb8c28">&#9675; small</span> &nbsp; '
            '<span style="color:#e05555">&#9675; top unseen</span> &nbsp; '
            '<span style="color:#999">&#9675; structure</span> &nbsp; '
            '<span style="color:#ffd200">&#9678; attended</span> &nbsp; '
            '<span style="color:#d55fd5">&#9733; mover</span>')
        row.addWidget(self._legend)
        layout.addLayout(row)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.showGrid(x=True, y=True, alpha=0.15)
        self._plot.setLabel("bottom", "right (m)")
        self._plot.setLabel("left", "forward (m)")
        self._free = pg.ScatterPlotItem(pxMode=False, symbol="s", pen=None)
        self._vox = pg.ScatterPlotItem(pxMode=False, symbol="s", pen=None)
        self._rings: list[pg.PlotDataItem] = []
        self._clusters = pg.ScatterPlotItem(pxMode=False, symbol="o", brush=None)
        self._att = pg.ScatterPlotItem(pxMode=False, symbol="o", brush=None, pen=pg.mkPen(255, 210, 0, width=3))
        self._mover = pg.ScatterPlotItem(size=18, symbol="star", pen=None, brush=pg.mkBrush(213, 95, 213, 230))
        self._body = pg.PlotDataItem(pen=pg.mkPen(120, 220, 120, width=2))
        for it in (self._free, self._vox, self._clusters, self._att, self._mover, self._body):
            self._plot.addItem(it)
        self._label = pg.TextItem(color=(255, 210, 0), anchor=(0, 1))
        self._plot.addItem(self._label)
        layout.addWidget(self._plot, 1)
        self._range_set = False

        self._latest = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(120)
        self._timer.timeout.connect(self._flush)
        self._timer.start()

    def _mark_dirty(self, *_):
        self._dirty = True

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    def _draw_rings(self, bx: float, by: float, rmax: float) -> None:
        for r in self._rings:
            self._plot.removeItem(r)
        self._rings = []
        th = np.linspace(0, 2 * math.pi, 90)
        for r in np.arange(0.5, rmax + 1e-6, 0.5):
            sx, sy = _screen(bx + r * np.cos(th), by + r * np.sin(th))
            item = self._plot.plot(sx, sy, pen=pg.mkPen(60, 60, 60, width=1, style=Qt.PenStyle.DotLine))
            self._rings.append(item)

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        s = self._latest
        vm = num(s, "voxel_m", 0.04)
        cfg = s.get("cfg") or {}
        small_top = float(cfg.get("small_top", 0.16))
        small_ext = float(cfg.get("small_ext", 0.20))
        needs_top = bool(cfg.get("small_needs_top", False))

        if not s.get("open"):
            self._banner.set(f"cloud closed — {int(num(s, 'filed'))} filed, {int(num(s, 'cached'))} places cached", GREY)
        elif s.get("walking"):
            self._banner.set(f"WALKING CLOUD — {int(num(s, 'voxels'))} voxels, {int(num(s, 'small'))} small of "
                             f"{int(num(s, 'things'))} clusters", BLUE)
        else:
            self._banner.set(f"STOP CLOUD (place {int(num(s, 'place', -1))}) — {int(num(s, 'voxels'))} voxels, "
                             f"{int(num(s, 'small'))} small of {int(num(s, 'things'))} clusters", GREEN)

        # voxel columns: the highest mean height per (ix, iy)
        cols: dict[tuple[int, int], int] = {}
        for v in s.get("vox") or []:
            if len(v) < 5:
                continue
            k = (int(v[0]), int(v[1]))
            mm = int(v[4])
            if mm > cols.get(k, -10_000):
                cols[k] = mm
        show_floor = self._show_floor.isChecked()
        xs, ys, brushes = [], [], []
        for (ix, iy), mm in cols.items():
            if mm < _FLOOR:
                if not show_floor:
                    continue
                c = _C_FLOOR
            elif mm < _BREAK_HI:
                c = _C_BREAK
            elif mm < _FURN:
                c = _C_FURN
            else:
                c = _C_HIGH
            xs.append((ix + 0.5) * vm)
            ys.append((iy + 0.5) * vm)
            brushes.append(pg.mkBrush(*c))
        if xs:
            sx, sy = _screen(xs, ys)
            self._vox.setData(sx, sy, size=vm * 0.9, brush=brushes)
        else:
            self._vox.setData([], [])

        if self._show_free.isChecked() and s.get("free_cols"):
            fc = np.asarray(s["free_cols"], dtype=float)
            sx, sy = _screen((fc[:, 0] + 0.5) * vm, (fc[:, 1] + 0.5) * vm)
            a = np.clip(fc[:, 2] / 400.0, 0.05, 1.0)
            self._free.setData(sx, sy, size=vm, brush=[pg.mkBrush(40, 170, 170, int(25 + 90 * x)) for x in a])
        else:
            self._free.setData([], [])

        rows = s.get("things_rows") or []
        cx, cy, sizes, pens = [], [], [], []
        for r in rows:
            if len(r) < 9:
                continue
            x, y, _rng, ext, top, _nc, _h, small, seen = r[:9]
            cx.append(x)
            cy.append(y)
            shape_small = top < small_top and ext <= small_ext
            # a ring the thing's size; structure (often a metre of wall) as a small grey marker at its centroid
            sizes.append(max(ext, 0.06) + 0.04 if (small or shape_small) else 0.08)
            if small:
                pens.append(pg.mkPen(235, 140, 40, width=2))
            elif shape_small and needs_top and seen < top + vm:
                pens.append(pg.mkPen(224, 85, 85, width=2, style=Qt.PenStyle.DashLine))
            else:
                pens.append(pg.mkPen(150, 150, 150, width=1))
        if cx:
            sx, sy = _screen(cx, cy)
            self._clusters.setData(sx, sy, size=sizes, pen=pens)
        else:
            self._clusters.setData([], [])

        ai = int(num(s, "attended_i", -1))
        if 0 <= ai < len(rows):
            r = rows[ai]
            sx, sy = _screen([r[0]], [r[1]])
            self._att.setData(sx, sy, size=[max(r[3], 0.06) + 0.12])
            self._label.setText(f"attended  {r[2]:.2f} m  top {100 * r[4]:.0f} cm")
            self._label.setPos(float(sx[0]) + 0.08, float(sy[0]) + 0.08)
        else:
            self._att.setData([], [])
            self._label.setText("")

        mv = s.get("mover_row")
        if isinstance(mv, list) and len(mv) >= 2:
            sx, sy = _screen([mv[0]], [mv[1]])
            self._mover.setData(sx, sy)
        else:
            self._mover.setData([], [])

        body = s.get("body") or [0.0, 0.0, 0.0]
        bx, by, byaw = float(body[0]), float(body[1]), float(body[2])
        tri = np.array([[0.12, 0.0], [-0.06, 0.05], [-0.06, -0.05], [0.12, 0.0]])
        c, sn = math.cos(byaw), math.sin(byaw)
        px = bx + c * tri[:, 0] - sn * tri[:, 1]
        py = by + sn * tri[:, 0] + c * tri[:, 1]
        sx, sy = _screen(px, py)
        self._body.setData(sx, sy)
        rmax = float(cfg.get("max_range", 2.5))
        self._draw_rings(bx, by, rmax)
        if not self._range_set:
            self._plot.setXRange(-rmax, rmax)
            self._plot.setYRange(-0.5, rmax)
            self._range_set = True


def _rows(s: dict):
    cfg = s.get("cfg") or {}
    rows = s.get("things_rows") or []
    ai = int(num(s, "attended_i", -1))
    att = rows[ai] if 0 <= ai < len(rows) else None
    out = [
        ("open", flag(s, "open")),
        ("walking", flag(s, "walking")),
        ("place", f"{int(num(s, 'place', -1))}"),
        ("voxels", f"{int(num(s, 'voxels'))}  (break band {int(num(s, 'break'))})"),
        ("new share", f"{num(s, 'newfrac'):.3f}"),
        ("revisit change", f"{num(s, 'revisit', -1):.3f}  (overlap {int(num(s, 'overlap'))})"),
        ("clusters", f"{int(num(s, 'things'))}  small {int(num(s, 'small'))}"),
        ("attended", "—" if att is None else
         f"{att[2]:.2f} m  ext {100 * att[3]:.0f} cm  top {100 * att[4]:.0f} cm  seen above {100 * att[8]:.0f} cm"),
        ("mover", f"{num(s, 'mover', -1):.2f} m  (candidates {int(num(s, 'mover_cands'))})"),
        ("filed / cached", f"{int(num(s, 'filed'))} / {int(num(s, 'cached'))}"),
        ("free columns", f"{len(s.get('free_cols') or [])}"),
        ("", ""),
        ("small_top", f"{100 * float(cfg.get('small_top', 0)):.0f} cm   small_ext {100 * float(cfg.get('small_ext', 0)):.0f} cm"),
        ("walk_cloud", "yes" if cfg.get("walk_cloud") else " no"),
        ("free_rays", "yes" if cfg.get("free_rays") else " no"),
        ("small_needs_top", "yes" if cfg.get("small_needs_top") else " no"),
        ("mover_isolated", "yes" if cfg.get("mover_isolated") else " no"),
        ("things_isolated", "yes" if cfg.get("things_isolated") else " no"),
    ]
    return out


class CloudMapInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)

        self._plan = _CloudPlan()
        self._readout = KVReadout(_rows)
        self._series = MultiSeriesPlot(
            [
                Series("small", "small things", (235, 140, 40), width=2.0),
                Series("things", "clusters", (150, 150, 150), width=1.2),
                Series("mover_cands", "mover candidates", (213, 95, 213), width=1.2),
                Series("newfrac", "new share", (74, 144, 217), width=1.2),
                Series("attended", "attended range m", (255, 210, 0), width=1.5),
            ],
            title="The cloud's reduction over time",
            y_label="count / value",
        )
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._plan)
        top.addWidget(self._readout)
        top.setSizes([760, 340])
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([620, 220])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._plan.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._series.update_payload(snapshot)
