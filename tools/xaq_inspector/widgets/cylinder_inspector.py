"""Cylinder (place-code panorama) dashboard.

As the bug spins, the CylinderBuilder bins the first-person colour around the heading circle
into `n_bins` cells, each holding the mean RGB seen while facing that bin.  The finished
`panorama` is a heading-indexed colour signature of WHERE the bug is — view-invariant because
it's keyed by absolute heading, not by what's momentarily in front of it.  This panorama feeds
the place-EPM, and that EPM's prediction error (TLE) is the novelty signal the planner's
explorer chases — high TLE = "I haven't been here / this place is new".

Panels:
  * Panorama strip — a painted horizontal band of `n_bins` cells, cell i filled with
    panorama bin i's mean RGB.  Bin 0 = heading 0 rad on the left, wrapping to 2π on the right.
  * Readouts — built (cylinders finalised, lifetime) and bins_filled (coverage of the last
    sweep, out of n_bins).
  * Caption — reminds that this panorama feeds the place-EPM whose TLE is the explorer's
    novelty drive.
"""
from __future__ import annotations

import math
from typing import Optional

from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget


class _PanoramaStrip(QWidget):
    """Painted horizontal strip of per-heading-bin mean RGB cells.

    Mirrors the painted-canvas idiom of the encoder strip, but renders the
    actual bin colours directly via paintEvent (each cell is a literal
    QColor from the panorama), rather than an imshow heatmap.
    """

    _MARGIN = 6
    _LABEL_H = 16

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._panorama: list[float] = []
        self._n_bins = 0
        self.setMinimumHeight(70)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._n_bins = int(snapshot.get("n_bins", 0) or 0)
        pano = snapshot.get("panorama")
        if isinstance(pano, (list, tuple)):
            try:
                self._panorama = [float(x) for x in pano]
            except (TypeError, ValueError):
                self._panorama = []
        self.update()  # schedule a repaint

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(10, 10, 14))

        n = self._n_bins
        if n <= 0 or len(self._panorama) < n * 3:
            p.setPen(QColor(150, 150, 150))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "awaiting panorama")
            p.end()
            return

        m = self._MARGIN
        strip_top = m
        strip_h = max(1.0, self.height() - 2 * m - self._LABEL_H)
        usable_w = max(1.0, self.width() - 2 * m)
        cell_w = usable_w / n

        for i in range(n):
            r = max(0.0, min(1.0, self._panorama[i * 3 + 0]))
            g = max(0.0, min(1.0, self._panorama[i * 3 + 1]))
            b = max(0.0, min(1.0, self._panorama[i * 3 + 2]))
            color = QColor(int(r * 255), int(g * 255), int(b * 255))
            x0 = m + i * cell_w
            p.fillRect(QRectF(x0, strip_top, cell_w + 1.0, strip_h), color)

        # thin frame around the strip
        p.setPen(QPen(QColor(60, 60, 60), 1))
        p.drawRect(QRectF(m, strip_top, usable_w, strip_h))

        # heading labels under the ends: bin 0 = 0 rad … wraps 2π
        p.setPen(QColor(200, 200, 200))
        label_y = strip_top + strip_h + self._LABEL_H
        p.drawText(QRectF(m, strip_top + strip_h, usable_w * 0.5, self._LABEL_H),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   "0 rad (bin 0)")
        p.drawText(QRectF(m + usable_w * 0.5, strip_top + strip_h, usable_w * 0.5, self._LABEL_H),
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                   f"→ 2π ({n} bins)")
        _ = label_y  # documented baseline; rects handle vertical placement
        p.end()


class CylinderInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        title = QLabel("Place-code panorama  (heading-indexed mean RGB — the place signature)")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        outer.addWidget(title)

        self._strip = _PanoramaStrip()
        outer.addWidget(self._strip, 1)

        self._readout = QLabel("—")
        self._readout.setStyleSheet(
            "color: #ddd; font-family: Monospace; font-size: 13px;"
        )
        outer.addWidget(self._readout)

        caption = QLabel(
            "Feeds the place-EPM → its prediction error (TLE) is the planner's "
            "exploration novelty signal (high TLE = unvisited place)."
        )
        caption.setWordWrap(True)
        caption.setStyleSheet("color: #888; font-size: 11px; font-style: italic;")
        outer.addWidget(caption)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._strip.update_payload(snapshot)
        n_bins = int(snapshot.get("n_bins", 0) or 0)
        built = int(snapshot.get("built", 0) or 0)
        bins_filled = int(snapshot.get("bins_filled", 0) or 0)
        self._readout.setText(
            f"built (cylinders finalised): {built:6d}     "
            f"bins_filled (last sweep): {bins_filled} / {n_bins}"
        )
