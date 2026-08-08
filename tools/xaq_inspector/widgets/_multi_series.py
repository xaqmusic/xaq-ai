"""Reusable multi-channel scrolling time-series plot.

Used by every dashboard whose state is naturally a few scalar streams
over time (NeurochemState DA/HT, HomeostaticDrive errors, FaderController
alpha components, Premotor weighted_accel/entropy, ...).  Avoids
re-authoring the ring-buffer + rolling-curve idiom in each dashboard.

Each `Series` describes one curve:

  Series(key="dopamine", label="DA", color=(120, 200, 255))

`key` is a dotted path resolved against the snapshot dict, supporting
nested lookups like "errors.alive_pulse" or "trust_weights.video.flow".

Hosts call update_payload(snapshot) every tick; the buffer rolls left
one column and the new value is appended on the right.  Missing keys
write NaN, which pyqtgraph's connect="finite" leaves unconnected.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QApplication, QVBoxLayout, QWidget, QLabel


@dataclass
class Series:
    key: str                                   # dotted path into snapshot
    label: str
    color: Tuple[int, int, int]                # RGB 0..255
    width: float = 1.5
    style: Optional[Qt.PenStyle] = None        # None → solid line
    y_offset: float = 0.0                      # add to value before plotting


def _resolve(snapshot: dict, key: str):
    """Resolve a dotted key path against a nested dict.

    Returns None on any missing segment or non-dict intermediate.
    """
    cur = snapshot
    for part in key.split("."):
        if not isinstance(cur, dict):
            return None
        cur = cur.get(part)
        if cur is None:
            return None
    return cur


class MultiSeriesPlot(QWidget):
    def __init__(
        self,
        series: Sequence[Series],
        *,
        title: str = "",
        y_label: str = "value",
        buffer_size: int = 600,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._series: list[Series] = list(series)
        self._buffer_size = int(buffer_size)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._plot = pg.PlotWidget(title=title)
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left",   y_label)
        self._plot.setLabel("bottom", "tick (recent)")
        # 2026-08-05 — the legend used to be addLegend(offset=(-10, 10)), i.e. an overlay
        # anchored INSIDE the view box, which sat on top of the traces exactly where the
        # recent samples are (the right-hand edge) and made the newest data — the part being
        # read — the hardest to see.  The operator is drawing correlations BETWEEN series,
        # so occluding one to name the others is the wrong trade.  A flat strip under the
        # plot is always legible, never moves, and costs one text line of vertical space.
        # 2026-08-05 — the strip is now CLICKABLE.  Hiding a trace is not cosmetic here:
        # pyqtgraph's ViewBox skips invisible items when computing childrenBounds, so
        # hiding a large-magnitude series RESCALES the Y axis for the ones left behind.
        # That is the actual need — `intent_err` is spread-normalized and runs O(1) while
        # `fwd_progress_ema` runs O(0.04), so on a shared axis the signal being read is a
        # flat line pinned to the bottom.  Buffers keep filling while hidden, so a trace
        # brought back still has its history.
        #   click        toggle one trace
        #   shift/ctrl   solo it (click the soloed one again to restore)
        #   "all"        show everything
        layout.addWidget(self._plot)
        self._visible: list[bool] = [True] * len(self._series)
        self._legend = QLabel()
        self._legend.setTextFormat(Qt.TextFormat.RichText)
        self._legend.setWordWrap(True)
        self._legend.setContentsMargins(6, 0, 6, 2)
        self._legend.setOpenExternalLinks(False)
        self._legend.setToolTip("click a name to hide/show it · shift-click to solo")
        self._legend.linkActivated.connect(self._on_legend_click)
        layout.addWidget(self._legend)

        self._curves: list[pg.PlotDataItem] = []
        self._buffers: list[np.ndarray] = []
        for s in self._series:
            kw = dict(width=s.width)
            if s.style is not None:
                kw["style"] = s.style
            curve = self._plot.plot(
                pen=pg.mkPen(*s.color, **kw),
                name=s.label,
            )
            self._curves.append(curve)
            self._buffers.append(np.full(self._buffer_size, np.nan))
        self._render_legend()

        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    # ---- legend interaction ------------------------------------------------
    def _render_legend(self) -> None:
        parts = []
        for i, s in enumerate(self._series):
            r, g, b = s.color
            if self._visible[i]:
                style = f"color:rgb({r},{g},{b});text-decoration:none"
                mark = "&#9632;"                    # filled square
            else:
                # Dim toward the background AND hollow the marker, so "off" reads at a
                # glance without having to compare against a neighbouring colour.
                style = (f"color:rgb({r // 3 + 40},{g // 3 + 40},{b // 3 + 40});"
                         "text-decoration:line-through")
                mark = "&#9633;"                    # hollow square
            parts.append(f'<a href="{i}" style="{style}">{mark} {s.label}</a>')
        if not all(self._visible):
            parts.append('<a href="__all__" style="color:rgb(170,170,170)">[all]</a>')
        self._legend.setText("&nbsp; ".join(parts))

    def _apply_visibility(self) -> None:
        for curve, vis in zip(self._curves, self._visible):
            curve.setVisible(vis)
        self._render_legend()
        # Invisible items are excluded from childrenBounds, but the ViewBox only notices
        # on the next autorange pass -- ask for one now so the axis rescales immediately.
        try:
            self._plot.getViewBox().autoRange()
        except Exception:
            pass

    def _on_legend_click(self, href: str) -> None:
        if href == "__all__":
            self._visible = [True] * len(self._series)
            self._apply_visibility()
            return
        try:
            idx = int(href)
        except ValueError:
            return
        if not (0 <= idx < len(self._series)):
            return
        mods = QApplication.keyboardModifiers()
        solo = bool(mods & (Qt.KeyboardModifier.ShiftModifier
                            | Qt.KeyboardModifier.ControlModifier))
        if solo:
            # Already soloed on this one => restore everything, so the same gesture undoes
            # itself and the operator never has to hunt for [all].
            only_this = self._visible[idx] and sum(self._visible) == 1
            self._visible = [only_this or i == idx for i in range(len(self._series))]
        else:
            self._visible[idx] = not self._visible[idx]
        self._apply_visibility()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        for i, s in enumerate(self._series):
            self._buffers[i] = np.roll(self._buffers[i], -1)
            v = _resolve(snapshot, s.key)
            try:
                self._buffers[i][-1] = (
                    float(v) + s.y_offset if v is not None else np.nan
                )
            except (TypeError, ValueError):
                self._buffers[i][-1] = np.nan
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        for curve, buf in zip(self._curves, self._buffers):
            curve.setData(buf, connect="finite")
