"""PlaceNav dashboard — the planner reframed as a place/region NAVIGATOR.

Same cognitive-map render as the v1 PlaceGraphPlanner (nodes = crystallized PLACES,
directed edges = the ABSOLUTE heading learned travelling A->B, node heat = learned route
value, a red ring = a live FOOD TAG), so the place-graph / bearing-dial / value-series
panels are REUSED verbatim.  The reframe (2026-07-09) changes what the map is FOR:

  * play GROWS the map, klino CLOSES on scent, so PlaceNav NAVIGATES the known map to a
    remembered food-REGION -- a LOOSE remembrance, not a per-node food-value optimizer.
  * the food ring is now a BOUNDED honest TAG (SET 1 on a real eat, on-arrival-without-eat
    COLLAPSE, slow fade) -- never an immortal super-attractor.
  * `plan_value` is HONEST: >0 only for a fresh tag + an executable (non-ceded) route, so
    the L2 arbiter is not fed a "food reachable this way" lie.

The status readout swaps in the region-navigator signals: live food TAGS map-wide, whether
the current committed hop was CEDED (ruled unreachable -- a wall), and the stall counter.
"""
from __future__ import annotations

import math

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

# reuse the v1 planner's render primitives — my diag_snapshot emits the same keys.
from .place_graph_inspector import (
    _BearingDial,
    _DriverMeters,
    _GraphView,
    _StateMeter,
    _ValueSeries,
)


class _NavStatus(QWidget):
    """Region-navigator facts: route, live food TAGS, reachability (ceded / stall)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet("color: #ddd; font-family: Monospace; font-size: 12px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._lbl, 1)

        self._latest = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(120)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        s = self._latest
        cur = int(s.get("cur_node", -1))
        nxt = int(s.get("next_node", -1))
        fx = float(s.get("fx", 0.0) or 0.0)
        fy = float(s.get("fy", 0.0) or 0.0)
        bearing = math.degrees(math.atan2(fx, fy)) if math.hypot(fx, fy) > 1e-9 else 0.0
        ceded = bool(s.get("route_ceded", False))
        hop = f"{cur} -> {nxt}" if nxt >= 0 else f"{cur} -> (none)"
        rows = [
            ("route",        hop + ("  [CEDED]" if ceded else "")),
            ("n_nodes",      int(s.get("n_nodes", 0) or 0)),
            ("food tags",    int(s.get("n_food_tags", 0) or 0)),   # live tags map-wide (region memory)
            ("route_stall",  int(s.get("route_stall", 0) or 0)),   # ticks on the hop w/o a transition
            ("bearing",      f"{bearing:.1f}"),
            ("plan_value",   f"{float(s.get('plan_value', 0.0) or 0.0):.3f}"),   # -> arbiter reach_planner (honest)
            ("plan_novelty", f"{float(s.get('plan_novelty', 0.0) or 0.0):.3f}"),  # coverage need when no food route
        ]
        self._lbl.setText("\n".join(f"{k:>13}: {v}" for k, v in rows))


class PlaceNavInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})  —  place/region navigator")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._graph = _GraphView()
        self._dial = _BearingDial()
        self._state_meter = _StateMeter()
        self._meters = _DriverMeters()
        self._status = _NavStatus()
        self._series = _ValueSeries()

        right = QSplitter(Qt.Orientation.Vertical)
        right.addWidget(self._dial)
        right.addWidget(self._state_meter)
        right.addWidget(self._meters)
        right.addWidget(self._status)
        right.setSizes([200, 120, 200, 160])

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._graph)
        top.addWidget(right)
        top.setSizes([700, 340])

        main = QSplitter(Qt.Orientation.Vertical)
        main.addWidget(top)
        main.addWidget(self._series)
        main.setSizes([360, 240])
        outer.addWidget(main, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._graph.update_payload(snapshot)
        self._dial.update_payload(snapshot)
        self._state_meter.update_payload(snapshot)
        self._meters.update_payload(snapshot)
        self._status.update_payload(snapshot)
        self._series.update_payload(snapshot)
