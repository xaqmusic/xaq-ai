"""PlaceGraphPlanner dashboard — the Cell's route-following map planner.

The PlaceGraphPlanner is the cognitive map: as the bug forages it crystallizes
discrete PLACE nodes and learns the ABSOLUTE heading to travel between adjacent
places (directed edges).  Each node carries a `food` estimate (how much it has
eaten there) and a learned `value` (route-quality, propagated back from food).
When `planning`, it picks the next hop toward high value and emits an egocentric
nav bearing (fx=+right, fy=+forward) for the HeadingController to follow; when
not planning it falls through to raw FORAGING (scent chemotaxis).

This widget makes that map+route LEGIBLE:

  * Place graph — nodes as circles, directed edges as arrows.  The 2-D LAYOUT is
    UNFOLDED from the learned edge-headings: a root node sits at the origin, then
    each connected node is placed at parent + (sin θ, cos θ) along the absolute
    edge heading (egocentric convention, +x right / +y forward), spread by BFS.
    So the drawing reflects the real learned geometry — it should take on the
    shape of the maze.  Positions are cached and only relaid out when the
    node/edge set changes, so the graph stays still frame-to-frame.
  * Heat by VALUE (low → high route quality); FOOD nodes get a red ring.  The
    current node has a bright outline; the planned cur→next hop is a thick gold
    arrow, and the greedy argmax-value route is traced a few hops ahead.
  * Bearing dial — the chosen egocentric (fx, fy) nav direction (forward = up).
  * Status readout — PLANNING vs FORAGING headline, cur→next, hunger, progress.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from ._multi_series import MultiSeriesPlot, Series


# value heat ramp: low (cold blue) → mid (teal/green) → high (warm yellow)
def _value_color(frac: float) -> tuple[int, int, int]:
    f = max(0.0, min(1.0, frac))
    if f < 0.5:
        t = f / 0.5                      # blue → green
        r = int(40 + 20 * t)
        g = int(80 + 140 * t)
        b = int(180 - 100 * t)
    else:
        t = (f - 0.5) / 0.5              # green → yellow
        r = int(60 + 195 * t)
        g = int(220 - 10 * t)
        b = int(80 - 60 * t)
    return r, g, b


def _layout_positions(nodes: list[int], edges: list) -> dict[int, tuple[float, float]]:
    """Unfold node positions from the learned edge-headings.

    Pick a root, place it at the origin, then BFS outward placing each child at
    parent + (sin θ, cos θ) where θ is the ABSOLUTE edge heading (egocentric:
    +x right, +y forward).  Disconnected nodes (no incident placed edge) fall
    back to a small spiral so nothing is lost.
    """
    pos: dict[int, tuple[float, float]] = {}
    if not nodes:
        return pos

    # adjacency from directed edges, keeping the heading for each direction.
    adj: dict[int, list[tuple[int, float]]] = {n: [] for n in nodes}
    for e in edges:
        try:
            fr, to, hd = int(e[0]), int(e[1]), float(e[2])
        except (TypeError, ValueError, IndexError):
            continue
        if fr in adj:
            adj[fr].append((to, hd))
        # reverse edge for layout connectivity (place `fr` relative to `to`)
        if to in adj:
            adj[to].append((fr, hd + math.pi))

    node_set = set(nodes)
    remaining = list(nodes)            # preserve order for deterministic roots
    visited: set[int] = set()
    while remaining:
        # next root = first unplaced node (most-connected gives a tidier unfold)
        root = max(
            (n for n in remaining if n not in visited),
            key=lambda n: len(adj.get(n, [])),
            default=None,
        )
        if root is None:
            break
        pos[root] = (0.0, 0.0) if not pos else _spiral_slot(len(pos))
        visited.add(root)
        queue = deque([root])
        while queue:
            cur = queue.popleft()
            cx, cy = pos[cur]
            for (nb, hd) in adj.get(cur, []):
                if nb in visited or nb not in node_set:
                    continue
                pos[nb] = (cx + math.sin(hd), cy + math.cos(hd))
                visited.add(nb)
                queue.append(nb)
        remaining = [n for n in remaining if n not in visited]

    # any node still unplaced (shouldn't happen, but be safe) → spiral
    for n in nodes:
        if n not in pos:
            pos[n] = _spiral_slot(len(pos))
    return pos


def _spiral_slot(i: int) -> tuple[float, float]:
    """Fallback position for disconnected nodes — a loose outward spiral."""
    r = 0.6 + 0.35 * i
    a = 2.399963 * i                    # golden-angle spread
    return r * math.cos(a), r * math.sin(a)


class _GraphView(QWidget):
    """pyqtgraph scatter+arrows render of the place graph."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._title = QLabel("Place graph — (no map yet)")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setMouseEnabled(x=True, y=True)
        self._plot.hideAxis("bottom")
        self._plot.hideAxis("left")

        self._edge_items: list = []        # per-edge arrow + line segments
        self._labels: list = []
        # nodes drawn last (on top of edges)
        self._nodes = pg.ScatterPlotItem(pxMode=True)
        self._plot.addItem(self._nodes)
        layout.addWidget(self._plot)

        # cached layout — only recomputed when the topology changes
        self._pos: dict[int, tuple[float, float]] = {}
        self._topo_key: tuple = ()

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

    def _clear_dynamic(self) -> None:
        for it in self._edge_items:
            self._plot.removeItem(it)
        self._edge_items = []
        for lbl in self._labels:
            self._plot.removeItem(lbl)
        self._labels = []

    def _add_arrow(self, x0, y0, x1, y1, color, width, head=10):
        """Directed edge: a coloured line + an ArrowItem at the destination."""
        seg = pg.PlotCurveItem(x=[x0, x1], y=[y0, y1],
                               pen=pg.mkPen(*color, width=width))
        self._plot.addItem(seg)
        self._edge_items.append(seg)
        ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
        # ArrowItem points along -angle by convention; place at the node, a touch
        # back so the head doesn't bury under the node marker.
        arr = pg.ArrowItem(angle=180 - ang, headLen=head, tipAngle=28,
                           brush=pg.mkBrush(*color), pen=None)
        bx = x1 - 0.16 * math.cos(math.radians(ang))
        by = y1 - 0.16 * math.sin(math.radians(ang))
        arr.setPos(bx, by)
        self._plot.addItem(arr)
        self._edge_items.append(arr)

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        snap = self._latest

        nodes = [int(n) for n in (snap.get("nodes") or [])]
        edges = list(snap.get("edges") or [])
        food = [float(x or 0.0) for x in (snap.get("food") or [])]
        value = [float(x or 0.0) for x in (snap.get("value") or [])]
        cur_node = int(snap.get("cur_node", -1))
        next_node = int(snap.get("next_node", -1))
        planning = bool(snap.get("planning", False))

        if not nodes:
            self._clear_dynamic()
            self._nodes.setData([])
            self._title.setText("Place graph — (no map yet)")
            return

        # index maps parallel to `nodes`
        idx = {n: i for i, n in enumerate(nodes)}
        val_of = {n: (value[i] if i < len(value) else 0.0) for n, i in idx.items()}
        food_of = {n: (food[i] if i < len(food) else 0.0) for n, i in idx.items()}

        # relayout only when topology (node set + edge connectivity) changes
        edge_key = tuple(sorted(
            (int(e[0]), int(e[1])) for e in edges
            if isinstance(e, (list, tuple)) and len(e) >= 2
        ))
        topo_key = (tuple(sorted(nodes)), edge_key)
        # Prefer the planner's path-integration centroids (true learned geometry); fall
        # back to the BFS edge-heading unfold when positions aren't published.
        node_pos = snap.get("node_pos")
        if node_pos and len(node_pos) == len(nodes):
            self._pos = {n: (float(node_pos[i][0]), float(node_pos[i][1]))
                         for i, n in enumerate(nodes)}
            self._topo_key = topo_key
        elif topo_key != self._topo_key or not self._pos:
            self._pos = _layout_positions(nodes, edges)
            self._topo_key = topo_key

        pos = self._pos
        self._clear_dynamic()

        # ---- edges (drawn first, under the nodes) -------------------------
        # planned next-hop + greedy route highlighted; others dim grey.
        route_edges = self._greedy_route(cur_node, next_node, edges, val_of)
        for e in edges:
            try:
                fr, to = int(e[0]), int(e[1])
                cnt = int(e[3]) if len(e) > 3 else 1
            except (TypeError, ValueError, IndexError):
                continue
            if fr not in pos or to not in pos:
                continue
            x0, y0 = pos[fr]; x1, y1 = pos[to]
            if (fr, to) == (cur_node, next_node) and next_node >= 0:
                # the planned next hop — thick bright gold
                self._add_arrow(x0, y0, x1, y1, (255, 210, 60), 3.2, head=13)
            elif (fr, to) in route_edges:
                # greedy route a few hops ahead — amber
                self._add_arrow(x0, y0, x1, y1, (235, 160, 60), 2.0, head=10)
            else:
                a = min(220, 60 + 25 * cnt)
                self._add_arrow(x0, y0, x1, y1, (110, 110, 130, a), 1.2, head=8)

        # ---- nodes --------------------------------------------------------
        vmax = max(val_of.values()) if val_of else 0.0
        vmin = min(val_of.values()) if val_of else 0.0
        vspan = max(1e-6, vmax - vmin)
        feps = 1e-3
        spots = []
        for n in nodes:
            if n not in pos:
                continue
            x, y = pos[n]
            frac = (val_of[n] - vmin) / vspan
            r, g, b = _value_color(frac)
            size = 16 + 14 * frac
            is_food = food_of[n] > feps
            is_cur = (n == cur_node)
            if is_cur:
                pen = pg.mkPen(255, 255, 255, width=3)      # bright outline
                size += 4
            elif is_food:
                pen = pg.mkPen(255, 70, 70, width=2.5)       # red food ring
            else:
                pen = pg.mkPen(40, 40, 40, width=1)
            spots.append({"pos": (x, y), "size": size,
                          "brush": pg.mkBrush(r, g, b, 255), "pen": pen})
            # value label
            lbl = pg.TextItem(f"{n}", color=(20, 20, 20), anchor=(0.5, 0.5))
            lbl.setPos(x, y)
            self._plot.addItem(lbl)
            self._labels.append(lbl)
        self._nodes.setData(spots)

        n_food = sum(1 for n in nodes if food_of[n] > feps)
        state = "PLANNING" if planning else "FORAGING"
        self._title.setText(
            f"Place graph — {len(nodes)} nodes · {len(edges)} edges · "
            f"{n_food} food · {state}   "
            f"(white=current · red ring=food · gold=next hop)")

    @staticmethod
    def _greedy_route(start: int, nxt: int, edges: list,
                      val_of: dict, hops: int = 4) -> set:
        """Trace the greedy argmax-value route a few hops ahead from `nxt`."""
        out: set = set()
        if start < 0 or nxt < 0:
            return out
        # adjacency: from → list of to
        adj: dict[int, list[int]] = {}
        for e in edges:
            try:
                fr, to = int(e[0]), int(e[1])
            except (TypeError, ValueError, IndexError):
                continue
            adj.setdefault(fr, []).append(to)
        cur = nxt
        seen = {start}
        for _ in range(hops):
            cands = [t for t in adj.get(cur, []) if t not in seen]
            if not cands:
                break
            best = max(cands, key=lambda t: val_of.get(t, 0.0))
            out.add((cur, best))
            seen.add(best)
            cur = best
        return out


class _BearingDial(QWidget):
    """Top-down dial for the chosen egocentric nav bearing (fx, fy)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        title = QLabel("Nav bearing (fx,fy) — forward = up")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setXRange(-1.3, 1.3)
        self._plot.setYRange(-1.3, 1.3)
        self._plot.hideAxis("bottom")
        self._plot.hideAxis("left")
        th = np.linspace(0, 2 * math.pi, 64)
        self._plot.plot(np.cos(th), np.sin(th), pen=pg.mkPen(80, 80, 80, width=1))
        self._plot.plot([0, 0], [0, 1.15], pen=pg.mkPen(60, 90, 60, width=1))
        self._needle = self._plot.plot([0, 0], [0, 0],
                                       pen=pg.mkPen(255, 215, 60, width=3))
        self._tip = pg.ScatterPlotItem(size=12,
                                       brush=pg.mkBrush(255, 215, 60, 255), pen=None)
        self._plot.addItem(self._tip)
        layout.addWidget(self._plot)

        self._latest = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(90)
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
        snap = self._latest
        fx = float(snap.get("fx", 0.0) or 0.0)      # +x right
        fy = float(snap.get("fy", 0.0) or 0.0)      # +y forward
        mag = math.hypot(fx, fy)
        planning = bool(snap.get("planning", False))
        if mag > 1e-6:
            scale = min(1.1, mag) / mag
            ex, ey = fx * scale, fy * scale
        else:
            ex, ey = 0.0, 0.0
        col = (255, 210, 60) if planning else (120, 200, 255)
        self._needle.setData([0, ex], [0, ey], pen=pg.mkPen(*col, width=3))
        self._tip.setData([ex], [ey], brush=pg.mkBrush(*col, 255))


class _Status(QWidget):
    """Slim discrete map/route facts (the state headline is the _StateMeter gauge above)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet(
            "color: #ddd; font-family: Monospace; font-size: 12px;")
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
        hop = f"{cur} -> {nxt}" if nxt >= 0 else f"{cur} -> (none)"
        # discrete map/route facts; the live scalars are in the meters + value series below.
        rows = [
            ("route",        hop),
            ("n_nodes",      int(s.get("n_nodes", 0) or 0)),
            ("cur_heading°", f"{math.degrees(float(s.get('cur_heading', 0.0) or 0.0)):.1f}"),
            ("bearing°",     f"{bearing:.1f}"),
            ("plan_value",   f"{float(s.get('plan_value', 0.0) or 0.0):.3f}"),
            # §2.3/§2.2 signals published to the L2 EFE arbiter:
            ("plan_precis",  f"{float(s.get('plan_precision', 0.0) or 0.0):.3f}"),  # belief sharpness (model precision)
            ("plan_novelty", f"{float(s.get('plan_novelty', 0.0) or 0.0):.3f}"),    # frontier uncertainty (epistemic)
        ]
        self._lbl.setText("\n".join(f"{k:>13}: {v:>10}" for k, v in rows))


class _Meter(QWidget):
    """One labelled horizontal bar + numeric readout (a normalised [0,1] driver)."""

    def __init__(self, name: str, col: tuple[int, int, int], parent: QWidget | None = None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(2, 1, 2, 1)
        v.setSpacing(1)
        self._lbl = QLabel(name)
        self._lbl.setStyleSheet("color: #bbb; font-size: 10px;")
        self._bar = QProgressBar()
        self._bar.setRange(0, 1000)
        self._bar.setTextVisible(True)
        self._bar.setFixedHeight(16)
        hexcol = "#%02x%02x%02x" % col
        self._bar.setStyleSheet(
            "QProgressBar { background: #1a1a1a; border: 1px solid #333; text-align: center;"
            " color: #fff; font-size: 10px; }"
            " QProgressBar::chunk { background: %s; }" % hexcol)
        v.addWidget(self._lbl)
        v.addWidget(self._bar)

    def set(self, frac: float, text: str) -> None:
        self._bar.setValue(int(round(max(0.0, min(1.0, frac)) * 1000)))
        self._bar.setFormat(text)


class _StateMeter(QWidget):
    """The plan↔forage decision as a CENTERED gauge: the uphill gradient V[next]−V[cur] IS the
    threshold-free decision signal — push RIGHT (gold) toward PLANNING when there is a strictly
    uphill neighbour to route to, push LEFT (steel) toward WANDERING when the field is flat / at a
    local peak (nothing to climb → explore bootstrap). Vision-homing (food in line of sight) is a
    hard override shown above the gauge."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(2)
        self._state = QLabel("—")
        self._state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._state.setStyleSheet("color:#888; font-size:20px; font-weight:bold;")
        layout.addWidget(self._state)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setMenuEnabled(False)
        self._plot.hideAxis("left")
        self._plot.hideAxis("bottom")
        self._plot.setXRange(-1.15, 1.15)
        self._plot.setYRange(-1.0, 1.0)
        self._plot.setFixedHeight(64)
        # centre line = the decision boundary (V[next] == V[cur]).
        self._plot.plot([0, 0], [-0.7, 0.7], pen=pg.mkPen(200, 200, 200, 120, width=1))
        # the gradient bar grows from the centre toward the winning side.
        self._bar = pg.BarGraphItem(x0=[0], x1=[0], y0=[-0.45], y1=[0.45],
                                    brushes=[pg.mkBrush(120, 160, 255)])
        self._plot.addItem(self._bar)
        lw = pg.TextItem("WANDER ◄", color=(120, 150, 190), anchor=(0, 0.5))
        lw.setPos(-1.12, 0.0)
        rp = pg.TextItem("► PLAN", color=(255, 210, 60), anchor=(1, 0.5))
        rp.setPos(1.12, 0.0)
        self._plot.addItem(lw)
        self._plot.addItem(rp)
        layout.addWidget(self._plot)

    def update_payload(self, s: dict) -> None:
        if not isinstance(s, dict):
            return
        planning = bool(s.get("planning", False))
        wandering = bool(s.get("wandering", False))
        homing = bool(s.get("homing_vision", False))
        v_cur = float(s.get("v_cur", 0.0) or 0.0)
        v_next = float(s.get("v_next", 0.0) or 0.0)
        v_max = float(s.get("v_max", 0.0) or 0.0)
        grad = v_next - v_cur                              # the threshold-free decision signal
        # normalise to [-1,1] by the value scale so the needle is comparable across maps.
        frac = max(-1.0, min(1.0, grad / v_max)) if v_max > 1e-6 else 0.0
        if homing:
            self._state.setText("VISION-HOMING")
            self._state.setStyleSheet("color:#ff9bd2; font-size:20px; font-weight:bold;")
            x1 = frac
        elif planning:
            self._state.setText("PLANNING")
            self._state.setStyleSheet("color:#ffd23c; font-size:20px; font-weight:bold;")
            x1 = max(0.02, frac)                            # gold, right of centre
        else:  # wandering / foraging bootstrap
            self._state.setText("WANDERING")
            self._state.setStyleSheet("color:#78b4ff; font-size:20px; font-weight:bold;")
            x1 = min(-0.02, frac) if frac < 0 else -0.06    # steel, left of centre
        col = (255, 210, 60) if x1 > 0 else (120, 160, 255)
        self._bar.setOpts(x0=[0.0], x1=[float(x1)], brushes=[pg.mkBrush(*col)])


class _DriverMeters(QWidget):
    """The planner's live drivers as normalised bars: what is pulling the bug right now."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(3)
        title = QLabel("drivers")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)
        self._hunger = _Meter("hunger (food drive — weights V's food term)", (240, 150, 60))
        self._boredom = _Meter("boredom — max habituation (explore drive)", (180, 130, 235))
        self._climb = _Meter("value climb  V[cur] / V[max]", (120, 160, 255))
        for m in (self._hunger, self._boredom, self._climb):
            layout.addWidget(m)
        layout.addStretch(1)

    def update_payload(self, s: dict) -> None:
        if not isinstance(s, dict):
            return
        hunger = float(s.get("hunger", 0.0) or 0.0)
        boredom = float(s.get("max_hab", 0.0) or 0.0)
        v_cur = float(s.get("v_cur", 0.0) or 0.0)
        v_max = float(s.get("v_max", 0.0) or 0.0)
        self._hunger.set(hunger, f"{hunger:.2f}")
        self._boredom.set(boredom, f"{boredom:.2f}")
        self._climb.set((v_cur / v_max) if v_max > 1e-6 else 0.0,
                        f"{v_cur:.2f} / {v_max:.2f}")


class _ValueSeries(QWidget):
    """Rolling value dynamics — the planner's 'value race' analogue: the route value it
    publishes to the L2 arbiter (plan_value) plus the value field it is climbing
    (V[cur] → V[next] → V[max])."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._plot = MultiSeriesPlot(
            [
                Series("plan_value", "plan_value → arbiter", (255, 210, 60), width=2.0),
                Series("v_next", "V[next hop]", (120, 160, 255), width=1.5),
                Series("v_cur", "V[current]", (90, 120, 200), width=1.2),
                Series("v_max", "V[map max]", (200, 200, 200), width=1.0,
                       style=Qt.PenStyle.DashLine),
            ],
            title="planner value dynamics  (route value + the value field it climbs)",
            y_label="value",
        )
        layout.addWidget(self._plot)

    def update_payload(self, snapshot: dict) -> None:
        self._plot.update_payload(snapshot)


class PlaceGraphInspector(QWidget):
    def __init__(self, module_id: str, module_type: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})  —  route-following map planner")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._graph = _GraphView()
        self._dial = _BearingDial()
        self._state_meter = _StateMeter()
        self._meters = _DriverMeters()
        self._status = _Status()
        self._series = _ValueSeries()

        # right column: bearing dial, the plan↔forage state gauge, the driver meters, slim status.
        right = QSplitter(Qt.Orientation.Vertical)
        right.addWidget(self._dial)
        right.addWidget(self._state_meter)
        right.addWidget(self._meters)
        right.addWidget(self._status)
        right.setSizes([200, 120, 200, 140])

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._graph)
        top.addWidget(right)
        top.setSizes([700, 340])

        # bottom: the value-dynamics time-series (the planner's 'value race' analogue).
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
