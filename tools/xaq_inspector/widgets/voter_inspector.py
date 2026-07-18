"""LateralVoter dashboard — consensus / fusion / Hebbian view.

Four panels:
  * Hebbian assoc matrix — QTableWidget heatmap of the assoc dict-of-dicts.
    Row/col axes are modality names so the matrix reads at a glance; each
    cell shows the numeric value tinted by magnitude.
  * Per-modality surprise EMA — multi-line, one curve per modality.
  * Per-modality trust weights — multi-line.  Trust weights live on the
    prev_token sub-snapshot, not at the voter root, so we extract them
    via a lightweight key-flattening adapter.
  * Active-modality timeline + fused TLE / dopamine — recent winners
    rendered as a horizontal coloured strip (one cell per recent tick),
    with TLE + DA on a second small plot underneath.

The assoc and surprise/trust dicts have dynamic keys (modalities can
in principle appear/disappear with hot-patches).  The widget discovers
new keys lazily and reuses a stable colour palette per name so the
legend doesn't reshuffle when new modalities arrive.
"""
from __future__ import annotations

from collections import deque
from typing import Deque, Dict, List, Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QBrush, QColor
from PyQt6.QtWidgets import (
    QLabel, QSplitter, QStackedWidget, QVBoxLayout, QWidget, QHBoxLayout,
)

from ._multi_series import MultiSeriesPlot, Series


_PALETTE = [
    (255, 120, 120), (120, 220, 255), (255, 215,  90),
    (170, 255, 130), (255, 160, 220), (200, 200, 200),
    (130, 200, 255), (255, 180,  90), (180, 255, 220),
    (220, 180, 255), (255, 200, 150), (160, 240, 200),
]
def _palette_for(name: str) -> tuple[int, int, int]:
    return _PALETTE[abs(hash(name)) % len(_PALETTE)]


def _signed_cmap() -> pg.ColorMap:
    """Black-centered diverging colormap: blue → near-black → red.

    Same idiom as encoder strip / W-matrix heatmap so all the diverging
    heatmaps in the inspector share one visual language.
    """
    return pg.ColorMap(
        pos=np.array([0.0, 0.45, 0.5, 0.55, 1.0]),
        color=np.array([
            [ 40, 110, 220, 255],
            [ 20,  35,  60, 255],
            [ 10,  10,  14, 255],
            [ 60,  30,  20, 255],
            [220,  80,  60, 255],
        ], dtype=np.uint8),
    )


# ---------------------------------------------------------------------------
# Hebbian matrix heatmap
# ---------------------------------------------------------------------------

class _AssocMatrix(QWidget):
    """Heatmap view of the Hebbian association matrix.

    Implementation note: the previous version used a QTableWidget with
    per-cell text + brush updates and a header set to ResizeToContents.
    For non-trivial matrices that combination is O(N²) work per refresh
    and the layout recalc kicked off by ResizeToContents triggered a
    full re-measure of every section every time — enough to wedge the
    GUI thread once the user enabled Hebbian association on a real
    voter.  Replaced with a pyqtgraph ImageItem (one numpy→texture
    upload per refresh, constant cost regardless of matrix size) and
    custom AxisItem ticks for modality labels.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Hebbian assoc matrix")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        # Stack[0] = placeholder for the empty-matrix case (most configs
        # leave LateralVoter.association_enabled=false, so the snapshot's
        # `assoc` dict stays {} forever — without a placeholder it just
        # looks like the widget broke).
        # Stack[1] = the populated heatmap.
        self._stack = QStackedWidget()
        layout.addWidget(self._stack, 1)

        # Stats line under the heatmap — N, |max|, mean off-diagonal
        # magnitude, symmetry, and the strongest off-diagonal pair.
        # Filled in by _flush whenever the matrix has data.
        self._stats = QLabel("")
        self._stats.setStyleSheet(
            "color: #aaa; font-family: Monospace; font-size: 10px;"
        )
        self._stats.setWordWrap(True)
        layout.addWidget(self._stats)

        self._placeholder = QLabel(
            "waiting for data\n\n"
            "(association_enabled is off in this config,\n"
            "or no co-activations have been observed yet)"
        )
        self._placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._placeholder.setStyleSheet(
            "color: #777; font-size: 13px; background: #0c0e12;"
        )
        self._stack.addWidget(self._placeholder)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("#0c0e12")
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.invertY(True)   # row 0 at top, like a spreadsheet
        self._plot.showGrid(x=False, y=False)
        # Hide axis ticks + labels — long modality names overlap and are
        # illegible at any reasonable widget size.  The ordering of rows
        # / columns is conveyed by the stats line under the heatmap
        # (top off-diagonal pair, etc.) instead.
        for axis_name in ("bottom", "left", "top", "right"):
            self._plot.getAxis(axis_name).setStyle(showValues=False)
            self._plot.getAxis(axis_name).setPen(pg.mkPen(None))
        self._plot.hideAxis("bottom")
        self._plot.hideAxis("left")
        self._image = pg.ImageItem(axisOrder="row-major")
        self._plot.addItem(self._image)
        self._cmap = _signed_cmap()
        self._stack.addWidget(self._plot)
        self._stack.setCurrentWidget(self._placeholder)

        self._keys: List[str] = []
        self._dirty = False
        self._latest: Optional[dict] = None
        self._refresh = QTimer(self)
        self._refresh.setInterval(200)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._latest = snapshot.get("assoc")
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        assoc = self._latest if isinstance(self._latest, dict) else {}

        # Union of row + column keys, alphabetically.
        keys: set[str] = set(assoc.keys())
        for inner in assoc.values():
            if isinstance(inner, dict):
                keys.update(inner.keys())
        sorted_keys = sorted(keys)

        if not sorted_keys:
            self._stack.setCurrentWidget(self._placeholder)
            self._title.setText("Hebbian assoc matrix  —  no data")
            self._stats.setText("")
            return

        self._stack.setCurrentWidget(self._plot)

        n = len(sorted_keys)
        # Build the dense N×N matrix.  Missing cells default to 0.
        mat = np.zeros((n, n), dtype=float)
        for i, r in enumerate(sorted_keys):
            inner = assoc.get(r) or {}
            if not isinstance(inner, dict):
                continue
            for j, c in enumerate(sorted_keys):
                try:
                    v = inner.get(c, 0.0)
                    if v is not None:
                        mat[i, j] = float(v)
                except (TypeError, ValueError):
                    pass

        max_abs = max(1e-6, float(np.nanmax(np.abs(mat))))
        levels = (-max_abs, max_abs)
        self._image.setImage(mat, levels=levels, autoLevels=False)
        try:
            self._image.setLookupTable(self._cmap.getLookupTable(0.0, 1.0, 256))
        except Exception:
            pass

        # Lock the view extent to the matrix only when the size changes;
        # axis labels are off so this is the only ranging needed.
        if sorted_keys != self._keys:
            self._keys = sorted_keys
            self._plot.setRange(xRange=(0, n), yRange=(0, n), padding=0)

        # ------------------------------------------------------------------
        # Stats line.
        #
        # Mean off-diagonal magnitude — a self-association of 1.0 on the
        # diagonal is mostly bookkeeping; what's interesting is how much
        # cross-modality association has formed.
        #
        # Symmetry — Hebbian co-activation is intrinsically symmetric, so
        # asymmetry tracks how far the learning has diverged from its
        # mathematical ideal.  Computed as
        #   1 - ||M - M^T||_F / (2 * ||M||_F)   ∈ [0, 1]
        # 1 = perfectly symmetric, 0 = fully anti-symmetric.
        #
        # Top off-diagonal pair — the most associated cross-modality pair
        # tells the user where the system has actually formed structure,
        # which is more informative than the raw |max|.
        # ------------------------------------------------------------------
        if n > 1:
            mask_off = ~np.eye(n, dtype=bool)
            off = mat[mask_off]
            mean_off = float(np.mean(np.abs(off)))
            # Top off-diagonal pair by absolute value.
            tri_idx = np.argmax(np.abs(off))
            # Reconstruct (i, j) from the flattened off-diagonal index.
            flat_i, flat_j = np.where(mask_off)
            ti, tj = int(flat_i[tri_idx]), int(flat_j[tri_idx])
            top_val = float(mat[ti, tj])
            top_str = f"{sorted_keys[ti]} → {sorted_keys[tj]}: {top_val:+.3f}"

            frob_M = float(np.linalg.norm(mat))
            sym = 1.0
            if frob_M > 1e-9:
                frob_diff = float(np.linalg.norm(mat - mat.T))
                sym = max(0.0, 1.0 - frob_diff / (2.0 * frob_M))
        else:
            mean_off = 0.0
            top_str = "—"
            sym = 1.0

        self._title.setText(
            f"Hebbian assoc matrix  —  {n} modalities, |max| {max_abs:.3f}"
        )
        self._stats.setText(
            f"mean |off-diag|={mean_off:.3f}   "
            f"symmetry={sym:.3f}   "
            f"top: {top_str}"
        )


# ---------------------------------------------------------------------------
# Active modality recency strip
# ---------------------------------------------------------------------------

class _ActiveModalityStrip(QWidget):
    """Horizontal coloured strip showing the last N active modalities.

    One vertical band per recent tick, coloured by which modality won the
    consensus position-encoding priority.  Stable per-modality colour
    palette via name hash so the strip pattern is recognisable across
    sessions.
    """

    HISTORY = 200

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Active modality (recent)")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._view = pg.PlotWidget()
        self._view.setBackground("k")
        self._view.setMouseEnabled(x=False, y=False)
        self._view.hideAxis("left")
        self._view.setLabel("bottom", "tick (recent →)")
        self._image = pg.ImageItem(axisOrder="row-major")
        self._view.addItem(self._image)
        layout.addWidget(self._view)

        self._history: Deque[str] = deque(maxlen=self.HISTORY)
        self._key_to_id: Dict[str, int] = {"": 0}
        self._dirty = False

        self._refresh = QTimer(self)
        self._refresh.setInterval(120)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        prev = snapshot.get("prev_token") or {}
        modality = str(prev.get("active_modality", "")) if isinstance(prev, dict) else ""
        self._history.append(modality)
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or not self._history:
            return
        self._dirty = False
        # Build colour LUT and image.
        rgb_lut = [(20, 20, 20)]  # idx 0 = empty / unknown
        idx_for_name = {"": 0}
        for name in self._history:
            if name not in idx_for_name:
                idx_for_name[name] = len(rgb_lut)
                rgb_lut.append(_palette_for(name))
        # Image is 1 row × HISTORY cols.  Pad on the left if history is short
        # so the strip always sits flush-right.
        n = self.HISTORY
        ids = [0] * (n - len(self._history)) + [idx_for_name[m] for m in self._history]
        img = np.array([ids], dtype=np.int32)
        # Convert ids → RGB via lookup.
        lut_arr = np.array(rgb_lut + [(20, 20, 20)] * (256 - len(rgb_lut)),
                           dtype=np.uint8)
        # Render directly as RGB (skip pyqtgraph colormap pipeline).
        rgb_img = lut_arr[img]
        self._image.setImage(rgb_img, levels=None, autoLevels=False)
        # Title shows the latest modality name.
        latest = self._history[-1] or "—"
        self._title.setText(f"Active modality (recent)  —  now: {latest}")


# ---------------------------------------------------------------------------
# Per-modality multi-line plot — discovers keys from snapshot at runtime
# ---------------------------------------------------------------------------

class _PerModalityPlot(QWidget):
    """Multi-line plot where each curve is one modality from a dict-shaped
    snapshot field (surprise_ema, trust_weights, prediction_counts, ...).
    """

    BUFFER = 600

    def __init__(self, snapshot_path: str, *, title: str, y_label: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._path = snapshot_path  # dotted path resolving to a dict
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._plot = pg.PlotWidget(title=title)
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left",   y_label)
        self._plot.setLabel("bottom", "tick (recent)")
        self._plot.addLegend(offset=(-10, 10))
        layout.addWidget(self._plot)

        self._curves: Dict[str, pg.PlotDataItem] = {}
        self._buffers: Dict[str, np.ndarray] = {}
        self._dirty = False

        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def _resolve(self, snapshot: dict) -> Optional[dict]:
        cur = snapshot
        for part in self._path.split("."):
            if not isinstance(cur, dict):
                return None
            cur = cur.get(part)
            if cur is None:
                return None
        return cur if isinstance(cur, dict) else None

    def _ensure(self, name: str) -> None:
        if name in self._curves:
            return
        rgb = _palette_for(name)
        self._curves[name] = self._plot.plot(
            pen=pg.mkPen(*rgb, width=1.5), name=name,
        )
        self._buffers[name] = np.full(self.BUFFER, np.nan)

    def update_payload(self, snapshot: dict) -> None:
        d = self._resolve(snapshot)
        for name in self._buffers:
            self._buffers[name] = np.roll(self._buffers[name], -1)
            self._buffers[name][-1] = np.nan
        if isinstance(d, dict):
            for name, v in d.items():
                self._ensure(str(name))
                try:
                    self._buffers[str(name)][-1] = float(v)
                except (TypeError, ValueError):
                    self._buffers[str(name)][-1] = np.nan
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        for name, curve in self._curves.items():
            curve.setData(self._buffers[name], connect="finite")


# ---------------------------------------------------------------------------
# Top-level voter inspector
# ---------------------------------------------------------------------------

class VoterInspector(QWidget):
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

        self._matrix    = _AssocMatrix()
        self._surprise  = _PerModalityPlot(
            "surprise_ema",
            title="Per-modality surprise EMA",
            y_label="surprise",
        )
        self._trust     = _PerModalityPlot(
            "prev_token.trust_weights",
            title="Per-modality trust weight",
            y_label="trust [0,1]",
        )
        self._meta      = MultiSeriesPlot(
            [
                Series("prev_token.fused_tle", "fused TLE", (120, 220, 255), width=1.8),
                Series("dopamine",             "DA",        (255, 215,  60), width=1.5,
                       style=Qt.PenStyle.DashLine),
            ],
            title="Fused TLE + DA",
            y_label="value",
        )
        self._strip = _ActiveModalityStrip()

        # Top: matrix (left) | surprise EMA (right)
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._matrix)
        top.addWidget(self._surprise)
        top.setSizes([520, 520])

        # Bottom: trust + meta plot (left, vertically stacked) | active strip (right)
        bot_left = QSplitter(Qt.Orientation.Vertical)
        bot_left.addWidget(self._trust)
        bot_left.addWidget(self._meta)
        bot_left.setSizes([240, 200])

        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.addWidget(bot_left)
        bot.addWidget(self._strip)
        bot.setSizes([700, 320])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(bot)
        v.setSizes([460, 380])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._matrix.update_payload(snapshot)
        self._surprise.update_payload(snapshot)
        self._trust.update_payload(snapshot)
        self._meta.update_payload(snapshot)
        self._strip.update_payload(snapshot)
