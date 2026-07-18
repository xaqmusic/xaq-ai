"""Generic reflex inspector — used for every reflex / detector module.

The reflex chain is small enough (7 module types, mostly EMAs +
refractory counters + fire-count telemetry) that one auto-discovering
widget is more useful than seven bespoke ones.  Each module's
snapshot_state contributes a different mix of fields; we just walk
the top-level dict and split into:

  * scalar curves      — bool / int / float fields, one curve each
  * cumulative counters — int fields whose name contains 'count' or
                          'total' or 'remaining' (drawn on a separate
                          plot so they don't squash the small-value
                          curves' y-axis)
  * sub-dicts of floats — flattened into '<key>.<sub>' entries on
                          the scalar plot (covers e.g. last_values
                          on the whisker reflexes)
  * RNG strings, version numbers — surfaced in the bottom-left text
                                   readout, not plotted

Auto-discovery happens on the first payload and on any subsequent
payload that introduces a new key.  The series cap is 16; if a module
has more fields than that the dashboard quietly truncates, with a note
in the title.

This intentionally trades v3-fidelity for fast coverage — the reflex
modules are well-served by "fire count over time + current EMAs +
refractory countdown" without needing per-type custom panels.
"""
from __future__ import annotations

from typing import Dict

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget


BUFFER_SIZE = 600
MAX_SERIES  = 16


_PALETTE = [
    (255, 120, 120), (120, 220, 255), (255, 215,  90),
    (170, 255, 130), (255, 160, 220), (200, 200, 200),
    (130, 200, 255), (255, 180,  90), (180, 255, 220),
    (220, 180, 255), (255, 200, 150), (160, 240, 200),
    (200, 100, 255), (255, 100, 100), (100, 255, 200),
    (255, 255, 200),
]
def _palette_for(name: str) -> tuple[int, int, int]:
    return _PALETTE[abs(hash(name)) % len(_PALETTE)]


def _looks_like_counter(name: str) -> bool:
    n = name.lower()
    return ("count" in n or "_total" in n or n.startswith("total") or
            "remaining" in n or n in ("tick_count", "fire_count"))


def _flatten(snapshot: dict, prefix: str = "") -> Dict[str, float]:
    out: Dict[str, float] = {}
    for k, v in snapshot.items():
        if k.startswith("_") or k in ("version", "rng",
                                       "head_on_rng", "pulse_rng"):
            continue
        full = f"{prefix}{k}" if not prefix else f"{prefix}.{k}"
        if isinstance(v, bool):
            out[full] = float(v)
        elif isinstance(v, (int, float)):
            try:
                out[full] = float(v)
            except (TypeError, ValueError):
                pass
        elif isinstance(v, dict):
            sub = _flatten(v, full)
            out.update(sub)
        # lists / strings are ignored — they don't fit a time series here.
    return out


# ---------------------------------------------------------------------------
# Auto-discovering plot — one curve per discovered numeric key
# ---------------------------------------------------------------------------

class _AutoSeriesPlot(QWidget):
    def __init__(self, title: str, *, accept_counter: bool,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._accept_counter = accept_counter
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._plot = pg.PlotWidget(title=title)
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left",   "value")
        self._plot.setLabel("bottom", "tick (recent)")
        self._plot.addLegend(offset=(-10, 10))
        layout.addWidget(self._plot)

        self._curves: Dict[str, pg.PlotDataItem] = {}
        self._buffers: Dict[str, np.ndarray] = {}
        self._truncated = False
        self._dirty = False

        self._refresh = QTimer(self)
        self._refresh.setInterval(100)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def _ensure(self, name: str) -> bool:
        if name in self._curves:
            return True
        if len(self._curves) >= MAX_SERIES:
            self._truncated = True
            return False
        rgb = _palette_for(name)
        self._curves[name] = self._plot.plot(
            pen=pg.mkPen(*rgb, width=1.4), name=name,
        )
        self._buffers[name] = np.full(BUFFER_SIZE, np.nan)
        return True

    def update_payload(self, flat: Dict[str, float]) -> None:
        for name in self._buffers:
            self._buffers[name] = np.roll(self._buffers[name], -1)
            self._buffers[name][-1] = np.nan
        for name, v in flat.items():
            is_counter = _looks_like_counter(name)
            if is_counter != self._accept_counter:
                continue
            if not self._ensure(name):
                continue
            self._buffers[name][-1] = v
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        for name, curve in self._curves.items():
            curve.setData(self._buffers[name], connect="finite")


# ---------------------------------------------------------------------------
# Current-state readout
# ---------------------------------------------------------------------------

class _CurrentValuesReadout(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        self._title = QLabel("current state")
        self._title.setStyleSheet("color: #ddd; font-size: 11px; font-weight: bold;")
        layout.addWidget(self._title)

        self._lbl = QLabel("—")
        self._lbl.setStyleSheet(
            "color: #cdd; font-family: Monospace; font-size: 11px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self._lbl.setWordWrap(False)
        layout.addWidget(self._lbl, 1)

    def update_payload(self, flat: Dict[str, float]) -> None:
        if not flat:
            return
        # Sort by name for stable layout; show all values up to a generous cap.
        rows = sorted(flat.items())[:64]
        max_key_w = max((len(k) for k, _ in rows), default=8)
        lines = []
        for k, v in rows:
            if abs(v) < 1e6 and (v != int(v) or abs(v) < 1):
                lines.append(f"{k.rjust(max_key_w)}: {v:+.4f}")
            else:
                lines.append(f"{k.rjust(max_key_w)}: {int(v):>10d}")
        self._lbl.setText("\n".join(lines))


# ---------------------------------------------------------------------------
# Top-level inspector
# ---------------------------------------------------------------------------

class ReflexInspector(QWidget):
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

        self._scalars  = _AutoSeriesPlot("State scalars (EMAs / last values)",
                                          accept_counter=False)
        self._counters = _AutoSeriesPlot("Counters",
                                          accept_counter=True)
        self._readout  = _CurrentValuesReadout()

        right = QSplitter(Qt.Orientation.Vertical)
        right.addWidget(self._counters)
        right.addWidget(self._readout)
        right.setSizes([300, 300])

        h = QSplitter(Qt.Orientation.Horizontal)
        h.addWidget(self._scalars)
        h.addWidget(right)
        h.setSizes([700, 360])
        outer.addWidget(h, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        flat = _flatten(snapshot)
        self._scalars.update_payload(flat)
        self._counters.update_payload(flat)
        self._readout.update_payload(flat)
