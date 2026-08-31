"""Small reusable controls: a float slider that reads back, a sparkline, a meter.

The slider matters more than it looks.  Qt's QSlider is integer-only, and the values
here span a Hz range where a linear scale wastes nine tenths of the travel on the top
octave — so a control that is going to be dragged for an hour gets a log option and a
numeric readout that is always visible.  Tuning by ear needs the number too: "that one"
has to be writable down.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Callable

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt6.QtCore import QPointF
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QHBoxLayout, QLabel, QSizePolicy,
                             QSlider, QWidget)

from ..theme import INK_MUTED, INK_PRIMARY, LINE, SURFACE_ALT, SurfaceWidget, TRACK

_STEPS = 1000


class FloatSlider(QWidget):
    """label · slider · readout, on a linear or logarithmic scale."""

    valueChanged = pyqtSignal(float)

    def __init__(self, label: str, lo: float, hi: float, value: float, *, log: bool = False,
                 unit: str = "", decimals: int = 2, label_w: int = 74,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.lo, self.hi, self.log = float(lo), float(hi), log
        self.unit, self.decimals = unit, decimals
        self._emitting = False

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        self._label = QLabel(label)
        self._label.setObjectName("muted")
        self._label.setMinimumWidth(label_w)
        lay.addWidget(self._label)

        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(0, _STEPS)
        self._slider.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._slider.valueChanged.connect(self._on_slider)
        lay.addWidget(self._slider, 1)

        self._read = QLabel("")
        self._read.setObjectName("value")
        self._read.setMinimumWidth(64)
        self._read.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        lay.addWidget(self._read)

        self.set_value(value)

    # -- scale --

    def _to_pos(self, v: float) -> int:
        v = min(max(v, self.lo), self.hi)
        if self.log:
            lo = math.log(max(self.lo, 1e-6))
            hi = math.log(max(self.hi, 1e-6))
            t = (math.log(max(v, 1e-6)) - lo) / (hi - lo) if hi > lo else 0.0
        else:
            t = (v - self.lo) / (self.hi - self.lo) if self.hi > self.lo else 0.0
        return int(round(t * _STEPS))

    def _from_pos(self, p: int) -> float:
        t = p / _STEPS
        if self.log:
            lo = math.log(max(self.lo, 1e-6))
            hi = math.log(max(self.hi, 1e-6))
            return math.exp(lo + t * (hi - lo))
        return self.lo + t * (self.hi - self.lo)

    # -- value --

    def value(self) -> float:
        return self._from_pos(self._slider.value())

    def set_value(self, v: float) -> None:
        """Set without emitting — for syncing the UI to the engine's own state."""
        self._emitting = True
        self._slider.setValue(self._to_pos(float(v)))
        self._emitting = False
        self._update_readout(float(v))

    def _update_readout(self, v: float) -> None:
        self._read.setText(f"{v:.{self.decimals}f}{self.unit}")

    def _on_slider(self, pos: int) -> None:
        v = self._from_pos(pos)
        self._update_readout(v)
        if not self._emitting:
            self.valueChanged.emit(v)


class LabeledCombo(QWidget):
    currentTextChanged = pyqtSignal(str)

    def __init__(self, label: str, items: list[str], value: str = "", *, label_w: int = 74,
                 parent: QWidget | None = None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lb = QLabel(label)
        lb.setObjectName("muted")
        lb.setMinimumWidth(label_w)
        lay.addWidget(lb)
        self.combo = QComboBox()
        self.combo.addItems(items)
        if value in items:
            self.combo.setCurrentText(value)
        self.combo.currentTextChanged.connect(self.currentTextChanged)
        lay.addWidget(self.combo, 1)

    def set_items(self, items: list[str], value: str = "") -> None:
        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(items)
        if value in items:
            self.combo.setCurrentText(value)
        self.combo.blockSignals(False)

    def set_value(self, v: str) -> None:
        self.combo.blockSignals(True)
        self.combo.setCurrentText(v)
        self.combo.blockSignals(False)

    def value(self) -> str:
        return self.combo.currentText()


class Sparkline(SurfaceWidget):
    """A tiny history plot, autoscaled to what it holds.

    Autoscaled rather than fixed because the whole point is browsing sources whose ranges
    nobody knows yet — a fixed axis would show most of them as a flat line at the bottom
    and hide exactly the structure worth routing.
    """

    def __init__(self, n: int = 96, color: str = INK_PRIMARY, parent: QWidget | None = None):
        super().__init__(parent, SURFACE_ALT)
        self._buf: deque[float] = deque(maxlen=n)
        self._color = QColor(color)
        self.setMinimumSize(60, 18)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def push(self, v: float) -> None:
        try:
            f = float(v)
        except (TypeError, ValueError):
            return
        if math.isfinite(f):
            self._buf.append(f)
            self.update()

    def set_color(self, color: str) -> None:
        self._color = QColor(color)

    def paintEvent(self, _e) -> None:
        if len(self._buf) < 2:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        lo, hi = min(self._buf), max(self._buf)
        if hi - lo < 1e-12:
            lo, hi = lo - 0.5, hi + 0.5
        w, h = self.width(), self.height()
        pad = 2
        pts = QPolygonF()
        n = len(self._buf)
        for i, v in enumerate(self._buf):
            x = pad + (w - 2 * pad) * (i / (n - 1))
            y = h - pad - (h - 2 * pad) * ((v - lo) / (hi - lo))
            pts.append(QPointF(x, y))
        p.setPen(QPen(self._color, 1.2))
        p.drawPolyline(pts)


class MeterBar(SurfaceWidget):
    """A 0..1 bar.  Used for route contribution and the master level."""

    def __init__(self, color: str = INK_PRIMARY, parent: QWidget | None = None):
        super().__init__(parent, TRACK)
        self._v = 0.0
        self._color = QColor(color)
        self.setMinimumSize(40, 6)
        self.setMaximumHeight(8)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set_value(self, v: float) -> None:
        try:
            f = float(v)
        except (TypeError, ValueError):
            return
        f = min(max(f, 0.0), 1.0)
        if abs(f - self._v) > 1e-3:
            self._v = f
            self.update()

    def set_color(self, color: str) -> None:
        self._color = QColor(color)
        self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(TRACK))
        if self._v > 0:
            r = self.rect()
            r.setWidth(int(r.width() * self._v))
            p.fillRect(r, self._color)


def hline() -> QWidget:
    w = QWidget()
    w.setFixedHeight(1)
    w.setStyleSheet(f"background: {LINE};")
    return w


def check(label: str, value: bool, on_toggle: Callable[[bool], None]) -> QCheckBox:
    c = QCheckBox(label)
    c.setChecked(bool(value))
    c.toggled.connect(on_toggle)
    return c


def muted(text: str) -> QLabel:
    lb = QLabel(text)
    lb.setObjectName("muted")
    return lb


def heading(text: str) -> QLabel:
    lb = QLabel(text)
    lb.setObjectName("h2")
    return lb


__all__ = ["FloatSlider", "LabeledCombo", "Sparkline", "MeterBar", "hline", "check",
           "muted", "heading", "INK_MUTED"]
