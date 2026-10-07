#!/usr/bin/env python3
"""voxel_viewer — a standalone, static viewer for the duck's sweep clouds.

Reads the `cloudv` records ogma::CloudMap files into a host run's JSONL (or a saved CloudMap module
snapshot, a JSON object with a `vox` key) and shows them as voxels in an interactive 3D view: orbit
with the left mouse button, pan with the middle, zoom with the wheel.  Nothing here connects to a
brain; for the live module, the inspector proper is the tool.

What it shows, and what is instrumentation
------------------------------------------
A cloud is what the duck accumulated while standing still at one place: the ToF's returns,
gravity-levelled, de-rotated by its own odometry yaw, voxelised (4 cm by default).  Its voxels arrive
in the cloud's own BODY-ANCHORED frame — the frame the duck actually has, which the BODY view shows.
Each record also carries the WORLD pose the cloud was anchored on.  That pose is instrumentation: it is
the only reason the WORLD view can lay every cloud out in the room beside the furniture, and no brain
reads it.

Colour
------
Height bands (the default) classify each voxel by the MEAN height of the points in it, never by its
centre — the ground layer's centre sits exactly on the 2 cm threshold, which once put the whole floor
in the break band (microduck design doc §17.30).  The bands match the duck viewer's:
    floor      below 2 cm    muted grey         the floor itself (hidden by default)
    break      2-20 cm       orange  #d95926    something standing on the floor
    furniture  20-45 cm      blue    #3987e5    chair and table height
    tall       45 cm and up  pale               wall tops, the shelf, the clock
Hits colours a voxel by how many returns landed in it, on a log scale, because the distribution is
heavy-tailed (median 16, p99 about 2 600 on the R46 runs), along one blue ramp from dim to bright.
The two accents pass the dataviz validator against this background (#0c0e12), all pairs.  Logs written
before 2026-09-13 carry no mean height; their voxels fall back to the centre and say so in the readout.

Usage
-----
    tools/run_voxel_viewer.sh RUN.jsonl [--place ID] [--frame world|body] [--colour bands|hits]
                              [--show-floor] [--min-hits N] [--screenshot OUT.png]

--screenshot renders the 3D view once, prints the readout, and exits.  It needs a display: this GL
stack draws nothing on Qt's offscreen platform (measured 2026-09-13, both point sprites and meshes).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QVector3D
from PyQt6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox, QHBoxLayout, QLabel,
                             QListWidget, QListWidgetItem, QMainWindow, QPushButton, QRadioButton,
                             QSpinBox, QSplitter, QVBoxLayout, QWidget)
import warnings

# A Wayland session hands Qt an OpenGL ES 3.2 context, and pyqtgraph warns about it on every launch.
# It renders correctly on this machine (both the cube mesh and the markers, checked 2026-09-13), so
# the warning is noise here; anything that actually fails to draw still logs its own error.
warnings.filterwarnings("ignore", message="pyqtgraph.opengl is primarily tested against OpenGL Desktop")
import pyqtgraph.opengl as gl  # noqa: E402

BG = "#0c0e12"
PANEL = "#12151b"
INK = "#cbd2dc"
MUTED = "#8a95a8"
BANDS = (                                   # (upper bound m, name, colour)
    (0.02, "floor", "#454c5a"),
    (0.20, "break", "#d95926"),
    (0.45, "furniture", "#3987e5"),
    (math.inf, "tall", "#9fb1c6"),
)
HITS_RAMP = ("#256abf", "#3987e5", "#6da7ec", "#9ec5f4", "#cde2fb")   # few -> many returns
# In the room view the other clouds recede twice over: their colour blends toward the background, and
# their cubes shrink.  The shrink is what makes the selection legible — clouds of one wall share voxel
# cells, and equal cubes in one cell z-fight, so the selected cloud's full-size cubes must enclose the rest.
DIM = 0.35                                  # colour kept by an unselected cloud (0 = background)
SEL_SIZE, OTHER_SIZE = 0.92, 0.55           # cube edge as a fraction of the voxel
MARK_SEL, MARK_OTHER = "#e8edf3", "#566072"


def _rgb(hexcode: str) -> np.ndarray:
    h = hexcode.lstrip("#")
    return np.array([int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)])


# --------------------------------------------------------------------------------------- data

@dataclass
class Cloud:
    place: int
    t: float | None
    voxel_m: float
    anchor: tuple[float, float, float]      # world x, y, yaw at the moment the cloud opened
    revisit: float                          # -1 = no earlier cloud of this place to compare
    revisit_dist: float                     # NaN when the log predates it
    vox: np.ndarray                         # (N, 5): ix, iy, iz, hits, mean height mm (NaN if absent)
    has_world_pose: bool = True

    @property
    def heights(self) -> np.ndarray:
        mm = self.vox[:, 4]
        centre = (self.vox[:, 2] + 0.5) * self.voxel_m
        return np.where(np.isfinite(mm), mm / 1000.0, centre)

    @property
    def has_mean_height(self) -> bool:
        return bool(np.isfinite(self.vox[:, 4]).any())

    def label(self) -> str:
        t = f"t {self.t:5.0f} s" if self.t is not None else "snapshot"
        s = f"place {self.place:>3}  {t}  {len(self.vox):>5} vox"
        if self.revisit >= 0:
            d = f" @ {self.revisit_dist:.2f} m" if math.isfinite(self.revisit_dist) else ""
            s += f"  revisit {self.revisit:.2f}{d}"
        return s


def _vox_array(rows: list) -> np.ndarray:
    out = np.full((len(rows), 5), np.nan)
    for i, r in enumerate(rows):
        out[i, :min(5, len(r))] = r[:5]
    return out


def load_clouds(path: Path) -> list[Cloud]:
    """Every filed cloud in a host JSONL, in the order they were filed; or the one cloud of a saved
    CloudMap snapshot.  Only lines that mention `cloudv` are parsed, so a 100 MB run loads in seconds."""
    if path.suffix == ".json":
        d = json.loads(path.read_text())
        d = d.get("state", d) if isinstance(d, dict) else d
        if not isinstance(d, dict) or "vox" not in d:
            raise ValueError(f"{path} has no `vox` — expected a CloudMap snapshot")
        return [Cloud(place=int(d.get("place", -1)), t=None, voxel_m=float(d.get("voxel_m", 0.04)),
                      anchor=(0.0, 0.0, float(d.get("anchor_yaw", 0.0))),
                      revisit=float(d.get("revisit", -1.0)), revisit_dist=float(d.get("revisit_dist", float("nan"))),
                      vox=_vox_array(d["vox"]), has_world_pose=False)]
    clouds: list[Cloud] = []
    with open(path) as fh:
        for line in fh:
            if '"cloudv"' not in line or not line.startswith("{"):
                continue
            rec = json.loads(line)
            c = rec["cloudv"]
            if not c.get("vox"):
                continue
            ax, ay, ayaw = c.get("anchor", (0.0, 0.0, 0.0))
            clouds.append(Cloud(place=int(c.get("place", -1)), t=rec.get("t"), voxel_m=float(c["voxel_m"]),
                                anchor=(float(ax), float(ay), float(ayaw)),
                                revisit=float(c.get("revisit", -1.0)),
                                revisit_dist=float(c.get("revisit_dist", float("nan"))),
                                vox=_vox_array(c["vox"])))
    return clouds


# --------------------------------------------------------------------------------------- geometry

# one cube: 8 corners, 12 triangles
_CORNERS = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                     [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], float)
_TRIS = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6], [0, 4, 5], [0, 5, 1],
                  [1, 5, 6], [1, 6, 2], [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]])


def centres(cloud: Cloud, world: bool) -> np.ndarray:
    v = cloud.voxel_m
    p = (cloud.vox[:, :3] + 0.5) * v
    if not world:
        return p
    ax, ay, ayaw = cloud.anchor
    ca, sa = math.cos(ayaw), math.sin(ayaw)
    out = p.copy()
    out[:, 0] = ca * p[:, 0] - sa * p[:, 1] + ax
    out[:, 1] = sa * p[:, 0] + ca * p[:, 1] + ay
    return out


def band_colours(h: np.ndarray) -> np.ndarray:
    out = np.empty((len(h), 3))
    lo = -math.inf
    for hi, _name, col in BANDS:
        out[(h >= lo) & (h < hi)] = _rgb(col)
        lo = hi
    return out


def hits_colours(hits: np.ndarray, hits_max: float) -> np.ndarray:
    ramp = np.stack([_rgb(c) for c in HITS_RAMP])
    x = np.log1p(np.maximum(hits, 0)) / max(1e-9, math.log1p(hits_max))
    x = np.clip(x, 0.0, 1.0) * (len(ramp) - 1)
    i = np.minimum(np.floor(x).astype(int), len(ramp) - 2)
    f = (x - i)[:, None]
    return ramp[i] * (1 - f) + ramp[i + 1] * f


def cube_mesh(c: np.ndarray, size: np.ndarray | float, rgb: np.ndarray):
    n = len(c)
    half = np.broadcast_to(np.asarray(size, float) * 0.5, (n,))[:, None, None]
    verts = (c[:, None, :] + _CORNERS[None] * half).reshape(-1, 3)
    faces = (_TRIS[None] + (np.arange(n) * 8)[:, None, None]).reshape(-1, 3)
    rgba = np.concatenate([rgb, np.ones((n, 1))], axis=1)
    return verts, faces, np.repeat(rgba, 12, axis=0)


# --------------------------------------------------------------------------------------- view

class VoxelView(QWidget):
    def __init__(self, clouds: list[Cloud], parent: QWidget | None = None):
        super().__init__(parent)
        self.clouds = clouds
        self.hits_max = float(max(np.nanmax(c.vox[:, 3]) for c in clouds))
        self.any_world = any(c.has_world_pose for c in clouds)

        # ---- the 3D view
        self.gl = gl.GLViewWidget()
        self.gl.setBackgroundColor(BG)
        self.grid = gl.GLGridItem()
        self.grid.setSize(4, 4, 1)
        self.grid.setSpacing(0.5, 0.5, 1)
        if hasattr(self.grid, "setColor"):
            self.grid.setColor((58, 64, 76, 255))
        self.gl.addItem(self.grid)
        self.mesh = gl.GLMeshItem(smooth=False, drawEdges=False, shader="shaded")
        self.gl.addItem(self.mesh)
        self.markers = gl.GLLinePlotItem(mode="lines", width=2, antialias=True)
        self.gl.addItem(self.markers)

        # ---- the panel
        mono = QFont("IBM Plex Mono")
        mono.setStyleHint(QFont.StyleHint.Monospace)
        mono.setPointSizeF(9.5)
        self.list = QListWidget()
        self.list.setFont(mono)
        for c in clouds:
            QListWidgetItem(c.label(), self.list)
        self.show_all = QCheckBox("Every cloud, laid out in the room")
        self.show_all.setEnabled(self.any_world)
        self.frame_world = QRadioButton("World  (placed at the anchor — instrumentation)")
        self.frame_body = QRadioButton("Body  (the cloud's own frame — what the duck has)")
        frames = QButtonGroup(self)
        frames.addButton(self.frame_world)
        frames.addButton(self.frame_body)
        self.frame_world.setEnabled(self.any_world)
        self.colour = QComboBox()
        self.colour.addItems(["Height bands", "Hits (log)"])
        self.show_floor = QCheckBox("Show the floor (below 2 cm)")
        self.min_hits = QSpinBox()
        self.min_hits.setRange(1, 100000)
        self.min_hits.setPrefix("Min hits  ")
        self.reframe = QPushButton("Frame the view")
        self.legend = QLabel()
        self.legend.setTextFormat(Qt.TextFormat.RichText)
        self.stats = QLabel()
        self.stats.setFont(mono)
        self.stats.setWordWrap(True)
        self.stats.setAlignment(Qt.AlignmentFlag.AlignTop)

        panel = QWidget()
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(12, 12, 12, 12)
        pl.setSpacing(8)
        title = QLabel("Sweep clouds")
        title.setStyleSheet(f"font-size: 15px; font-weight: 600; color: {INK};")
        pl.addWidget(title)
        hint = QLabel("Orbit: left drag · pan: middle drag · zoom: wheel")
        hint.setStyleSheet(f"color: {MUTED}; font-size: 11px;")
        pl.addWidget(hint)
        pl.addWidget(self.list, 3)
        pl.addWidget(self.show_all)
        pl.addWidget(self.frame_world)
        pl.addWidget(self.frame_body)
        row = QHBoxLayout()
        row.addWidget(self.colour, 1)
        row.addWidget(self.min_hits, 1)
        pl.addLayout(row)
        pl.addWidget(self.show_floor)
        pl.addWidget(self.reframe)
        pl.addWidget(self.legend)
        pl.addWidget(self.stats, 2)

        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(panel)
        split.addWidget(self.gl)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([420, 900])
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(split)

        # picking another cloud while the whole room is shown keeps the camera where the operator left it
        self.list.currentRowChanged.connect(lambda *_: self.redraw(reframe=not self.show_all.isChecked()))
        for sig in (self.show_all.toggled, self.frame_world.toggled):
            sig.connect(lambda *_: self.redraw(reframe=True))
        for sig in (self.colour.currentIndexChanged, self.show_floor.toggled, self.min_hits.valueChanged):
            sig.connect(lambda *_: self.redraw(reframe=False))
        self.reframe.clicked.connect(lambda: self.redraw(reframe=True))

    # ---- state
    def configure(self, place: int | None, frame: str, colour: str, show_floor: bool, min_hits: int) -> None:
        widgets = (self.list, self.show_all, self.frame_world, self.frame_body, self.colour,
                   self.show_floor, self.min_hits)
        for w in widgets:
            w.blockSignals(True)
        row = len(self.clouds) - 1
        if place is not None:
            matches = [i for i, c in enumerate(self.clouds) if c.place == place]
            if matches:
                row = matches[-1]
        self.list.setCurrentRow(row)
        self.show_all.setChecked(place is None and self.any_world and len(self.clouds) > 1)
        (self.frame_world if frame == "world" and self.any_world else self.frame_body).setChecked(True)
        self.colour.setCurrentIndex(1 if colour == "hits" else 0)
        self.show_floor.setChecked(show_floor)
        self.min_hits.setValue(max(1, min_hits))
        for w in widgets:
            w.blockSignals(False)
        self.redraw(reframe=True)

    def redraw(self, reframe: bool) -> None:
        sel = self.list.currentRow()
        everything = self.show_all.isChecked()
        world = self.frame_world.isChecked() or everything
        self.frame_body.setEnabled(not everything)
        if everything and not self.frame_world.isChecked():
            self.frame_world.blockSignals(True)
            self.frame_world.setChecked(True)
            self.frame_world.blockSignals(False)
        shown = list(range(len(self.clouds))) if everything else ([sel] if sel >= 0 else [])
        hits_mode = self.colour.currentIndex() == 1
        bg = _rgb(BG)
        all_c, all_rgb, all_size, marks, mark_rgb = [], [], [], [], []
        for i in shown:
            c = self.clouds[i]
            other = everything and i != sel
            if world and c.has_world_pose:              # the heading the cloud was anchored on
                ax, ay, ayaw = c.anchor
            else:                                       # body frame: the duck sits at the origin facing +x
                ax, ay, ayaw = 0.0, 0.0, 0.0
            marks += [(ax, ay, 0.01), (ax + 0.25 * math.cos(ayaw), ay + 0.25 * math.sin(ayaw), 0.01)]
            mark_rgb += [_rgb(MARK_OTHER if other else MARK_SEL)] * 2
            h = c.heights
            keep = c.vox[:, 3] >= self.min_hits.value()
            if not self.show_floor.isChecked():
                keep &= h >= BANDS[0][0]
            if not keep.any():
                continue
            rgb = hits_colours(c.vox[keep, 3], self.hits_max) if hits_mode else band_colours(h[keep])
            if other:
                rgb = bg + (rgb - bg) * DIM
            all_c.append(centres(c, world)[keep])
            all_rgb.append(rgb)
            all_size.append(np.full(int(keep.sum()), c.voxel_m * (OTHER_SIZE if other else SEL_SIZE)))
        if all_c:
            verts, faces, cols = cube_mesh(np.concatenate(all_c), np.concatenate(all_size),
                                           np.concatenate(all_rgb))
            self.mesh.setMeshData(vertexes=verts, faces=faces, faceColors=cols)
            self.mesh.setVisible(True)
        else:
            self.mesh.setVisible(False)
        if marks:
            rgba = np.concatenate([np.array(mark_rgb), np.full((len(mark_rgb), 1), 0.95)], axis=1)
            self.markers.setData(pos=np.array(marks), color=rgba)
            self.markers.setVisible(True)
        else:
            self.markers.setVisible(False)
        if world:
            self.grid.resetTransform()
        else:
            self.grid.resetTransform()
            self.grid.translate(1.0, 0.0, 0.0)          # the body frame looks forward along +x
        self._legend(hits_mode)
        self._stats(sel, shown)
        if reframe and all_c:
            # Centre the bounds across the line of sight, so nothing is cropped at the sides, but move
            # the look point along it to the voxels' centre of mass: a sweep is mostly the far walls,
            # and centring the bounds in depth leaves the empty near floor filling half the frame.
            pts = np.concatenate(all_c)
            lo, hi = pts.min(0), pts.max(0)
            look = (lo + hi) / 2
            azimuth = -135 if world else -160
            ahead = -np.array([math.cos(math.radians(azimuth)), math.sin(math.radians(azimuth)), 0.0])
            look += ahead * float(np.dot(pts.mean(0) - look, ahead))
            span = float(np.max(hi - lo))
            self.gl.setCameraPosition(pos=QVector3D(*map(float, look)), distance=max(1.2, span * 1.5),
                                      elevation=32, azimuth=azimuth)

    def _legend(self, hits_mode: bool) -> None:
        if hits_mode:
            sw = "".join(f'<span style="color:{c}">■</span>' for c in HITS_RAMP)
            self.legend.setText(f'<span style="color:{MUTED}">few returns</span> {sw} '
                                f'<span style="color:{MUTED}">many (log scale, max {int(self.hits_max)})</span>')
        else:
            parts = []
            for lo_hi, (hi, name, col) in zip(("< 2 cm", "2–20 cm", "20–45 cm", "≥ 45 cm"), BANDS):
                parts.append(f'<span style="color:{col}">■</span> <span style="color:{INK}">{name}</span> '
                             f'<span style="color:{MUTED}">{lo_hi}</span>')
            self.legend.setText("&nbsp;&nbsp; ".join(parts))

    def _stats(self, sel: int, shown: list[int]) -> None:
        if sel < 0:
            self.stats.setText("")
            return
        c = self.clouds[sel]
        h = c.heights
        counts, lo = [], -math.inf
        for hi, name, _col in BANDS:
            counts.append(f"{name} {int(((h >= lo) & (h < hi)).sum())}")
            lo = hi
        lines = [f"place {c.place}" + (f"  ·  filed at t {c.t:.0f} s" if c.t is not None else "  ·  snapshot"),
                 f"{len(c.vox)} voxels at {c.voxel_m * 100:.0f} cm:  " + ",  ".join(counts),
                 f"hits: median {int(np.nanmedian(c.vox[:, 3]))}, max {int(np.nanmax(c.vox[:, 3]))}"]
        if c.revisit >= 0:
            d = f", anchors {c.revisit_dist:.2f} m apart by odometry" if math.isfinite(c.revisit_dist) else ""
            lines.append(f"revisit: {c.revisit:.2f} of this cloud absent from the last one here{d}")
        else:
            lines.append("revisit: no earlier cloud of this place")
        if c.has_world_pose:
            ax, ay, ayaw = c.anchor
            lines.append(f"anchor (world, instrumentation): x {ax:+.2f}  y {ay:+.2f}  yaw {math.degrees(ayaw):+.0f}°")
        if not c.has_mean_height:
            lines.append("this log predates mean heights — bands use the voxel centre, which counts floor as break")
        if len(shown) > 1:
            lines.append(f"showing {len(shown)} clouds; the selected one at full size and colour, "
                         f"the rest smaller and darker")
        self.stats.setText("\n".join(lines))

    def readout(self) -> str:
        return self.stats.text()


# --------------------------------------------------------------------------------------- app

STYLE = f"""
QWidget {{ background: {PANEL}; color: {INK}; font-size: 12px; }}
QListWidget {{ background: {BG}; border: 1px solid #222a36; }}
QListWidget::item:selected {{ background: #1c2a3d; color: #ffffff; }}
QComboBox, QSpinBox, QPushButton {{ background: {BG}; border: 1px solid #303a49; padding: 4px 6px; }}
QPushButton:hover {{ border-color: #3987e5; }}
QCheckBox::indicator, QRadioButton::indicator {{ width: 13px; height: 13px; }}
"""


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="The duck's sweep clouds in an interactive 3D view.")
    ap.add_argument("path", help="a host run's JSONL (with cloudv records) or a CloudMap snapshot .json")
    ap.add_argument("--place", type=int, default=None, help="open on this place alone (its latest cloud)")
    ap.add_argument("--frame", choices=("world", "body"), default="world")
    ap.add_argument("--colour", choices=("bands", "hits"), default="bands")
    ap.add_argument("--show-floor", action="store_true", help="draw the floor voxels too")
    ap.add_argument("--min-hits", type=int, default=1, help="hide voxels with fewer returns than this")
    ap.add_argument("--screenshot", metavar="OUT.png", help="render the 3D view once, print the readout, exit")
    ap.add_argument("--size", default="1360x860", help="window size WxH")
    a = ap.parse_args(argv)

    path = Path(a.path)
    if not path.exists():
        # a repo-relative path typed from somewhere else in the repo (tools/, say) still resolves
        repo = Path(__file__).resolve().parents[2]
        if not path.is_absolute() and (repo / path).exists():
            path = repo / path
        else:
            sys.exit(f"no such file: {a.path}  (looked in {Path.cwd()} and in the repo root {repo})")
    clouds = load_clouds(path)
    if not clouds:
        sys.exit(f"no clouds in {path} — the run needs a CloudMap in its graph and --cloud on the host")

    app = QApplication(sys.argv[:1])
    app.setStyleSheet(STYLE)
    win = QMainWindow()
    win.setWindowTitle(f"Sweep clouds — {path.name}")
    view = VoxelView(clouds)
    win.setCentralWidget(view)
    w, h = (int(x) for x in a.size.lower().split("x"))
    win.resize(w, h)
    view.configure(a.place, a.frame, a.colour, a.show_floor, a.min_hits)
    win.show()

    if a.screenshot:
        def shoot() -> None:
            img = win.grab().toImage()                  # the whole window: the panel and the 3D view
            ok = img.save(a.screenshot)
            print(f"{'wrote' if ok else 'FAILED to write'} {a.screenshot}  ({img.width()}x{img.height()})")
            print(f"{len(clouds)} clouds loaded from {path.name}")
            print(view.readout())
            app.quit()
        QTimer.singleShot(900, shoot)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
