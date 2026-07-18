"""EPM GNG canvas — port of v3 src/native/widgets/graph_canvas.py.

Focused subset: 2-D scatter of nodes from the EPM's GNG snapshot, edges
between connected nodes, hue mapped to node health (green = healthy,
red = degraded).  v3's PCA-projected coordinate space is replaced with
a direct first-two-prototype-dims projection — adequate for "yes, the
graph is changing" feedback the inspector exists to give.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget


class EpmCanvas(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        self._latest: dict | None = None
        self._tick = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self._header = QLabel(f"{module_id}  ({module_type})")
        self._header.setStyleSheet("color: #ddd; font-weight: bold;")
        layout.addWidget(self._header)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.setAspectLocked(True)
        layout.addWidget(self._plot, 1)

        # Edges drawn first (behind nodes).  PlotCurveItem with NaN-
        # separated segments instead of GraphItem because pyqtgraph
        # 0.13's GraphItem.setData routes through ScatterPlotItem and
        # blows up with `'NoneType' object has no attribute '_id'`
        # whenever brush/symbol resolve to None — even when the GraphItem
        # is being used purely as an edge renderer.  PlotCurveItem with
        # connect="finite" treats NaN as a segment break, so we just
        # interleave (a, b, NaN) for each (a, b) edge.
        self._edges = pg.PlotCurveItem(
            pen=pg.mkPen((140, 140, 140, 140), width=0.8),
            connect="finite",
        )
        self._plot.addItem(self._edges)

        # Glow layer underneath nodes — simulates the v3 cyan halo.
        self._glow = pg.ScatterPlotItem(size=22, pen=pg.mkPen(None))
        self._plot.addItem(self._glow)
        # Main node scatter on top.
        self._scatter = pg.ScatterPlotItem(
            size=10,
            pen=pg.mkPen("w", width=0.5),
        )
        self._plot.addItem(self._scatter)

        # Throttle redraws to ~30 Hz independent of incoming sub Hz.
        self._latest_dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(33)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        self._latest = snapshot
        self._tick = tick_id
        self._latest_dirty = True

    def _flush(self) -> None:
        if self._latest is None or not self._latest_dirty:
            return
        self._latest_dirty = False
        snap = self._latest
        gng = snap.get("gng") if isinstance(snap, dict) else None
        if not isinstance(gng, dict):
            self._header.setText(
                f"{self.module_id}  ({self.module_type})   "
                f"tick {self._tick}   (no gng in snapshot)"
            )
            return

        nodes = gng.get("nodes") or []
        edges = gng.get("edges") or []

        # Project each node prototype to 2-D using its first two dims.
        node_pos: Dict[int, Tuple[float, float]] = {}
        spots: List[dict] = []
        glow_spots: List[dict] = []
        baked_count = 0
        for n in nodes:
            try:
                proto = n.get("prototype") or []
                if len(proto) < 2:
                    continue
                x = float(proto[0])
                y = float(proto[1])
                node_id = int(n.get("id", 0))
                node_pos[node_id] = (x, y)
                health = float(n.get("health", 1.0))
                visits = int(n.get("visits", 0))
                baked = bool(n.get("post_bake_visits", 0) > 0
                             or n.get("bake_checked", False))
                if baked:
                    baked_count += 1
                # Health → hue: green at 1.0, red near 0.
                r = int(np.clip(255 * (1.0 - health), 0, 255))
                g = int(np.clip(255 * health, 0, 255))
                b = 80 if baked else 40
                spots.append({
                    "pos": (x, y),
                    "brush": pg.mkBrush(r, g, b, 220),
                    "size": 12 if baked else 8,
                })
                # Glow: cyan, opacity scaled by recent-visit activity
                # (capped to the most recent visit window in `step`).
                step = int(gng.get("step", 1)) or 1
                last_v = int(n.get("last_visited_step", 0))
                recency = max(0.0, 1.0 - min(1.0, (step - last_v) / 500.0))
                if recency > 0.05:
                    glow_spots.append({
                        "pos": (x, y),
                        "brush": pg.mkBrush(80, 220, 255, int(120 * recency)),
                        "size": 22,
                    })
            except (TypeError, ValueError):
                continue

        self._scatter.setData(spots=spots)
        self._glow.setData(spots=glow_spots)

        # Edge segments — interleave (a, b, NaN) per edge so the
        # PlotCurveItem with connect="finite" renders each pair as an
        # isolated line segment with no continuation.
        if node_pos:
            id_list = list(node_pos.keys())
            id_to_idx = {nid: i for i, nid in enumerate(id_list)}
            pos_array = np.array([node_pos[nid] for nid in id_list], dtype=float)
            xs: List[float] = []
            ys: List[float] = []
            for e in edges:
                pos = e.get("positions") if isinstance(e, dict) else None
                if not pos or len(pos) != 2:
                    continue
                a, b = int(pos[0]), int(pos[1])
                if a not in id_to_idx or b not in id_to_idx:
                    continue
                ai, bi = id_to_idx[a], id_to_idx[b]
                xs.extend([float(pos_array[ai, 0]), float(pos_array[bi, 0]), np.nan])
                ys.extend([float(pos_array[ai, 1]), float(pos_array[bi, 1]), np.nan])
            if xs:
                self._edges.setData(x=xs, y=ys, connect="finite")
            else:
                self._edges.setData(x=[], y=[])

        self._header.setText(
            f"{self.module_id}  ({self.module_type})   "
            f"tick {self._tick}   nodes {len(nodes)}   baked {baked_count}"
        )
