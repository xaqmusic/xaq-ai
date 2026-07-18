"""Rolling heatmap of EPM encoder output (gng.last_x).

Each diag payload contributes one column (the latest latent vector,
projection_dim long) to a `dim × time` heatmap that scrolls left every
update.  Yields a quick "is the encoder responding to input?" feel —
flat horizontal stripes mean a stuck encoder, busy vertical patterns
mean live activity.

The latent dimensionality is resolved on first payload; if it ever
changes (e.g. live config edit) the buffer is reallocated.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget


BUFFER_TICKS = 300  # horizontal width of the rolling strip


class EpmEncoderStrip(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Encoder output (gng.last_x)  —  awaiting payload")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._view = pg.PlotWidget()
        self._view.setBackground("k")
        self._view.showGrid(x=False, y=False)
        self._view.setMouseEnabled(x=False, y=False)
        self._view.setLabel("bottom", "tick (recent →)")
        self._view.setLabel("left",   "dim")
        layout.addWidget(self._view)

        # axisOrder='row-major' so img[dim_idx, tick_idx] indexes naturally
        # (cols = time, rows = dims).  Older pyqtgraph builds expose this
        # only via setOpts / direct attribute, not a setAxisOrder method.
        self._image = pg.ImageItem(axisOrder="row-major")
        self._view.addItem(self._image)
        # Diverging colormap so positive / negative latent values are
        # distinct, BUT centred on near-black instead of white.  Stock
        # CET-D1A maps the midpoint to white, which means initialised-
        # but-not-yet-filled cells (NaN→0) and steady-state-near-zero
        # latents render against a bright background that drowns the
        # actual signal — the user can't read the heatmap at all.
        # Custom stops give the same red/blue separation with a black
        # midpoint, matching the plot background.
        self._cmap = pg.ColorMap(
            pos=np.array([0.0, 0.45, 0.5, 0.55, 1.0]),
            color=np.array([
                [ 40, 110, 220, 255],   # deep blue
                [ 20,  35,  60, 255],   # blue → near-black
                [ 10,  10,  14, 255],   # near-black at midpoint
                [ 60,  30,  20, 255],   # near-black → red
                [220,  80,  60, 255],   # deep red
            ], dtype=np.uint8),
        )
        self._levels = (-3.0, 3.0)

        self._buf: Optional[np.ndarray] = None
        self._dim = 0
        self._dirty = False
        self._latest_tick = 0
        self._latest_min = 0.0
        self._latest_max = 0.0

        self._refresh = QTimer(self)
        self._refresh.setInterval(50)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

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
            self._buf = np.full((self._dim, BUFFER_TICKS), np.nan)

        # Roll left, append on the right.
        self._buf = np.roll(self._buf, -1, axis=1)
        self._buf[:, -1] = vec

        self._latest_tick = tick_id
        self._latest_min  = float(np.nanmin(vec))
        self._latest_max  = float(np.nanmax(vec))
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._buf is None:
            return
        self._dirty = False
        # Replace NaNs with 0 for display only — the buffer keeps NaN so
        # min/max calculations elsewhere can ignore them.
        img = np.nan_to_num(self._buf, nan=0.0)
        self._image.setImage(img, levels=self._levels, autoLevels=False)
        # Apply colormap each frame (cheap; the cmap object is reused).
        try:
            self._image.setLookupTable(self._cmap.getLookupTable(0.0, 1.0, 256))
        except Exception:
            pass
        self._title.setText(
            f"Encoder output (gng.last_x)  —  dim {self._dim}   "
            f"tick {self._latest_tick}   "
            f"range [{self._latest_min:+.2f}, {self._latest_max:+.2f}]"
        )
