"""PCA latent scatter — recent encoder outputs projected to 2-D, with the
GNG node prototypes overlaid in the same projection.

Why not the EpmCanvas's prototype[:2] axis-aligned projection? That picks
two arbitrary latent dimensions; PCA picks the two that *capture the
variance the encoder is producing*, so the plot tracks where the encoder
is actually moving in latent space.  The GNG nodes are projected onto
the same components so the trail sits in the same coordinate system as
the learned topology — the eye can immediately tell whether new input
is landing on or near existing nodes.

Refit policy
------------
PCA components are recomputed every REFIT_INTERVAL payloads.  Between
refits, points are projected onto cached components.  Sign-flip
ambiguity (each PC is determined up to ±1) is resolved at refit time
by aligning to the previous components — without this the scatter
flips horizontally / vertically every refit and the trail becomes
unreadable.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget


BUFFER_TICKS    = 300   # latent history length
REFIT_INTERVAL  = 60    # payloads between PCA refits (≈ 2 s at 30 Hz)
MIN_FIT_POINTS  = 10    # skip PCA until the buffer holds this many


class EpmPcaScatter(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Latent PCA scatter — awaiting payload")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.2)
        self._plot.setLabel("bottom", "PC1")
        self._plot.setLabel("left",   "PC2")
        self._plot.setAspectLocked(False)
        layout.addWidget(self._plot)

        # Three layers, drawn back-to-front.
        # 1) Trail: recent encoder outputs, fading by recency.
        self._trail = pg.ScatterPlotItem(
            size=6, pen=pg.mkPen(None), hoverable=False)
        self._plot.addItem(self._trail)
        # 2) GNG nodes: amber circles, larger, in the same projection.
        self._nodes = pg.ScatterPlotItem(
            size=12, pen=pg.mkPen("w", width=0.5), hoverable=False)
        self._plot.addItem(self._nodes)
        # 3) Latest point: prominent cross marker so the eye can find
        #    "where is the encoder right now" instantly.
        self._latest = pg.ScatterPlotItem(
            size=18, symbol="x",
            pen=pg.mkPen((100, 255, 255), width=2),
            brush=pg.mkBrush(100, 255, 255, 220),
            hoverable=False)
        self._plot.addItem(self._latest)

        # State
        self._buf: Optional[np.ndarray] = None  # (T, dim)
        self._dim = 0
        self._fill = 0       # how many slots are populated
        self._head = 0       # next write index (ring)
        self._refit_counter = 0
        self._pcs:  Optional[np.ndarray] = None   # (2, dim)
        self._mean: Optional[np.ndarray] = None   # (dim,)
        self._latest_node_count = 0
        self._latest_tick = 0
        self._dirty = False

        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    # ------------------------------------------------------------------
    # Payload intake
    # ------------------------------------------------------------------

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        gng = snapshot.get("gng") if isinstance(snapshot, dict) else None
        if not isinstance(gng, dict):
            return
        last_x = gng.get("last_x")
        if last_x is None or not isinstance(last_x, list) or len(last_x) == 0:
            return
        try:
            vec = np.asarray(last_x, dtype=float)
        except (TypeError, ValueError):
            return

        if self._buf is None or vec.size != self._dim:
            self._dim = int(vec.size)
            self._buf = np.full((BUFFER_TICKS, self._dim), np.nan)
            self._fill = 0
            self._head = 0
            self._pcs  = None
            self._mean = None

        self._buf[self._head] = vec
        self._head = (self._head + 1) % BUFFER_TICKS
        self._fill = min(self._fill + 1, BUFFER_TICKS)
        self._refit_counter += 1
        self._latest_tick = tick_id

        # Stash latest node prototypes for overlay rendering at flush time.
        nodes = gng.get("nodes") or []
        protos = []
        for n in nodes:
            try:
                p = n.get("prototype") if isinstance(n, dict) else None
                if p and len(p) == self._dim:
                    protos.append(p)
            except (TypeError, ValueError):
                continue
        self._latest_protos = (np.asarray(protos, dtype=float)
                                if protos else np.empty((0, self._dim)))
        self._latest_node_count = len(nodes)

        self._dirty = True

    # ------------------------------------------------------------------
    # Render
    # ------------------------------------------------------------------

    def _refit_pca(self) -> None:
        if self._buf is None or self._fill < MIN_FIT_POINTS:
            return
        # Use only populated rows.  When the ring isn't full yet, those
        # are rows [0..fill).  Once full, all BUFFER_TICKS rows are
        # valid; in that regime the head/tail order doesn't matter for
        # PCA because we're fitting on a set, not a sequence.
        valid = self._buf[: self._fill] if self._fill < BUFFER_TICKS else self._buf
        # Filter out any leftover NaN rows (defensive — shouldn't happen).
        finite = valid[~np.isnan(valid).any(axis=1)]
        if finite.shape[0] < MIN_FIT_POINTS:
            return
        mean = finite.mean(axis=0)
        centered = finite - mean
        # SVD-based PCA: V's first two rows are the top-2 PCs.
        try:
            _, _, vt = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return
        new_pcs = vt[:2]
        # Sign-align with previous PCs so the scatter doesn't flip
        # between refits.
        if self._pcs is not None:
            for i in range(2):
                if new_pcs[i] @ self._pcs[i] < 0:
                    new_pcs[i] = -new_pcs[i]
        self._pcs  = new_pcs
        self._mean = mean

    def _project(self, X: np.ndarray) -> np.ndarray:
        if self._pcs is None or self._mean is None:
            return np.empty((0, 2))
        return (X - self._mean) @ self._pcs.T

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False

        if self._refit_counter >= REFIT_INTERVAL or self._pcs is None:
            self._refit_pca()
            self._refit_counter = 0

        if self._buf is None or self._pcs is None:
            self._title.setText(
                f"Latent PCA scatter — collecting samples "
                f"({self._fill}/{MIN_FIT_POINTS})"
            )
            return

        # Trail in chronological order: oldest → newest, so colour by index.
        if self._fill < BUFFER_TICKS:
            chrono = self._buf[: self._fill]
        else:
            chrono = np.concatenate(
                (self._buf[self._head:], self._buf[: self._head]), axis=0)
        # Drop NaN rows defensively.
        finite_mask = ~np.isnan(chrono).any(axis=1)
        chrono = chrono[finite_mask]
        if chrono.shape[0] == 0:
            return
        proj = self._project(chrono)

        # Recency-fade brushes: oldest point alpha ~30, newest ~230.
        n = proj.shape[0]
        alphas = np.linspace(30, 230, n).astype(int)
        spots = [
            {"pos": (float(proj[i, 0]), float(proj[i, 1])),
             "brush": pg.mkBrush(80, 220, 255, int(alphas[i]))}
            for i in range(n)
        ]
        self._trail.setData(spots=spots)

        # Latest point isolated as a cross marker.
        self._latest.setData(spots=[{
            "pos": (float(proj[-1, 0]), float(proj[-1, 1])),
        }])

        # GNG node prototypes overlay.
        if self._latest_protos.shape[0] > 0:
            node_proj = self._project(self._latest_protos)
            node_spots = [
                {"pos": (float(node_proj[i, 0]), float(node_proj[i, 1])),
                 "brush": pg.mkBrush(255, 200, 80, 200)}
                for i in range(node_proj.shape[0])
            ]
            self._nodes.setData(spots=node_spots)
        else:
            self._nodes.setData(spots=[])

        self._title.setText(
            f"Latent PCA scatter — dim {self._dim}   "
            f"tick {self._latest_tick}   "
            f"trail {n}   nodes {self._latest_node_count}"
        )
