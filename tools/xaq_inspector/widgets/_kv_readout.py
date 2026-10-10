"""A monospace key/value readout and a status banner, shared by the duck's dashboards.

Every dashboard in this package ends with a readout of its scalars and most open with a coloured banner naming
the module's mode; the duck's modules (2026-10-02) share these two pieces rather than re-authoring them in each file.

  KVReadout(rows_fn)   rows_fn(snapshot) -> [(label, text), ...]; redrawn on a 75 ms timer from the latest snapshot
  Banner()             set(text, colour) -- one bold line, coloured by state
"""
from __future__ import annotations

from typing import Callable, Iterable

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QScrollArea, QVBoxLayout, QWidget

GREY = "#888888"
GREEN = "#1baf7a"
AMBER = "#eda100"
MAGENTA = "#d55fd5"
BLUE = "#4a90d9"
RED = "#e05555"


def dark_diverging():
    """Blue (−) · near-black (0) · red (+): a diverging map that stays dark at zero, for the dark panels."""
    import pyqtgraph as pg
    return pg.ColorMap([0.0, 0.5, 1.0], [(70, 130, 255), (16, 16, 22), (255, 95, 60)])


def num(snapshot: dict, key: str, default: float = 0.0) -> float:
    """A float from the snapshot (dotted path), the default on anything missing or unparseable."""
    cur = snapshot
    for part in key.split("."):
        if not isinstance(cur, dict):
            return default
        cur = cur.get(part)
    try:
        return float(cur) if cur is not None else default
    except (TypeError, ValueError):
        return default


def flag(snapshot: dict, key: str) -> str:
    cur = snapshot
    for part in key.split("."):
        if not isinstance(cur, dict):
            return " no"
        cur = cur.get(part)
    return "yes" if cur else " no"


class Banner(QLabel):
    def __init__(self, text: str = "—", parent: QWidget | None = None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set(text, GREY)

    def set(self, text: str, colour: str) -> None:
        self.setText(text)
        self.setStyleSheet(f"color:{colour}; font-weight:bold; font-size:12px;")


class KVReadout(QWidget):
    def __init__(self, rows_fn: Callable[[dict], Iterable[tuple[str, str]]], parent: QWidget | None = None):
        super().__init__(parent)
        self._rows_fn = rows_fn
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet("color:#ddd; background:#15171c; font-family:Monospace; font-size:12px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self._lbl.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._lbl)
        scroll.setStyleSheet("QScrollArea { border: none; background:#15171c; }")
        scroll.viewport().setStyleSheet("background:#15171c;")
        layout.addWidget(scroll, 1)
        self._latest: dict | None = None
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
        try:
            rows = list(self._rows_fn(self._latest))
        except Exception as e:                     # a malformed snapshot never takes the panel down
            rows = [("error", str(e))]
        w = max((len(k) for k, _ in rows), default=8)
        self._lbl.setText("\n".join(f"{k:>{w}}: {v}" if k else "" for k, v in rows))
