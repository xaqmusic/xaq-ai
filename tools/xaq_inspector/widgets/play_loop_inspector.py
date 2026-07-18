"""PlayLoop dashboard — the Cell's third policy: GROW the shared place-map (epistemic play).

PlayLoop is "PlaceGraphPlanner MINUS traverse": where the planner *descends* the
value field to route to remembered food (pragmatic exploit), PlayLoop *ascends*
NOVELTY toward the frontier to grow the shared cognitive map (epistemic explore).
Both are thin overlays on the ONE node-creator (the cylinder place-EPM), so this
widget deliberately mirrors the PlaceGraphInspector — same map + bearing dial +
state gauge + driver meters + value series — with play's semantics:

  * Play map — the same place graph, but nodes are heat-coloured by V_play (the
    novelty value field it climbs) and the FRONTIER (highest-novelty node, the
    least-modelled place) gets a violet ring (play's analogue of the planner's
    red food ring).  The current node has a bright outline; the cur→next CLIMB
    hop is a thick gold arrow.
  * State gauge — CLIMB (ascend novelty → frontier) ◄►  WANDER (run-and-tumble
    BEYOND the mapped graph, the discovery the planner structurally can't do);
    STALL-WANDER (forced_wander) when the map has stopped growing so the loop is
    pushed past the frontier.  The planner's PLANNING↔WANDERING gauge analogue.
  * Driver meters — novelty (place-EPM TLE = model degradation), boredom (max
    habituation), eat-credit, and play_value (the epistemic value → L2 arbiter).
  * Value series — the 'value race' analogue: play_value → arbiter plus the
    novelty field it climbs (novelty@cur, V_play peak, boredom).
"""
from __future__ import annotations

import math

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
# Reuse the planner widget's map helpers — PlayLoop shares the exact node/edge
# geometry (it is a second overlay on the same place-EPM), so the layout unfold
# and the value heat ramp are identical.
from .place_graph_inspector import _value_color, _layout_positions


class _PlayGraphView(QWidget):
    """pyqtgraph render of the play overlay: nodes heat-coloured by V_play, the
    frontier (max novelty) ringed, the cur→next CLIMB hop as a gold arrow."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._title = QLabel("Play map — (no map yet)")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.setAspectLocked(True)
        self._plot.setMouseEnabled(x=True, y=True)
        self._plot.hideAxis("bottom")
        self._plot.hideAxis("left")

        self._edge_items: list = []
        self._labels: list = []
        self._nodes = pg.ScatterPlotItem(pxMode=True)
        self._plot.addItem(self._nodes)
        layout.addWidget(self._plot)

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
        seg = pg.PlotCurveItem(x=[x0, x1], y=[y0, y1],
                               pen=pg.mkPen(*color, width=width))
        self._plot.addItem(seg)
        self._edge_items.append(seg)
        ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
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
        value = [float(x or 0.0) for x in (snap.get("value") or [])]
        novelty = [float(x or 0.0) for x in (snap.get("novelty") or [])]
        cur_node = int(snap.get("cur_node", -1))
        next_node = int(snap.get("next_node", -1))
        climbing = bool(snap.get("climbing", False))
        wandering = bool(snap.get("wandering", False))

        if not nodes:
            self._clear_dynamic()
            self._nodes.setData([])
            self._title.setText("Play map — (no map yet)")
            return

        idx = {n: i for i, n in enumerate(nodes)}
        val_of = {n: (value[i] if i < len(value) else 0.0) for n, i in idx.items()}
        nov_of = {n: (novelty[i] if i < len(novelty) else 0.0) for n, i in idx.items()}

        # frontier = the least-modelled (max-novelty) node — play's "goal" analogue.
        frontier = max(nodes, key=lambda n: nov_of.get(n, 0.0)) if nov_of else -1

        edge_key = tuple(sorted(
            (int(e[0]), int(e[1])) for e in edges
            if isinstance(e, (list, tuple)) and len(e) >= 2
        ))
        topo_key = (tuple(sorted(nodes)), edge_key)
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

        # Large maps (the odometry grid grows toward ~1k nodes) churn the inspector if we
        # add a TextItem per node + the full edge mesh every 120 ms.  Above this count we
        # SUMMARISE: keep the batched node scatter (a single draw call, so the whole map is
        # still visible) but limit per-node labels + the faint background edges to the
        # salient nodes (current / next / frontier).  The count stays in the title.
        NODE_SUMMARY_MAX = 150
        dense = len(nodes) > NODE_SUMMARY_MAX

        # ---- edges (under the nodes): the cur→next climb hop highlighted -----
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
                self._add_arrow(x0, y0, x1, y1, (255, 210, 60), 3.2, head=13)   # climb hop
            elif (fr, to) in route_edges:
                self._add_arrow(x0, y0, x1, y1, (235, 160, 60), 2.0, head=10)   # greedy climb ahead
            elif not dense:
                a = min(220, 60 + 25 * cnt)
                self._add_arrow(x0, y0, x1, y1, (110, 110, 130, a), 1.2, head=8)
            # dense map: skip the O(E) background mesh — climb + route edges still drawn

        # ---- nodes (heat by V_play; frontier ringed violet, current white) ---
        vmax = max(val_of.values()) if val_of else 0.0
        vmin = min(val_of.values()) if val_of else 0.0
        vspan = max(1e-6, vmax - vmin)
        spots = []
        for n in nodes:
            if n not in pos:
                continue
            x, y = pos[n]
            frac = (val_of[n] - vmin) / vspan
            r, g, b = _value_color(frac)
            size = 16 + 14 * frac
            if n == cur_node:
                pen = pg.mkPen(255, 255, 255, width=3)       # bright outline = current
                size += 4
            elif n == frontier:
                pen = pg.mkPen(210, 130, 255, width=2.5)      # violet ring = frontier (max novelty)
            else:
                pen = pg.mkPen(40, 40, 40, width=1)
            spots.append({"pos": (x, y), "size": size,
                          "brush": pg.mkBrush(r, g, b, 255), "pen": pen})
            if not dense or n in (cur_node, next_node, frontier):
                lbl = pg.TextItem(f"{n}", color=(20, 20, 20), anchor=(0.5, 0.5))
                lbl.setPos(x, y)
                self._plot.addItem(lbl)
                self._labels.append(lbl)
        self._nodes.setData(spots)

        if wandering:
            state = "WANDER"
        elif climbing:
            state = "CLIMB"
        else:
            state = "—"
        summ = " · summarised" if dense else ""
        self._title.setText(
            f"Play map — {len(nodes)} nodes · {len(edges)} edges · {state}{summ}   "
            f"(white=current · violet ring=frontier · gold=climb hop)")

    @staticmethod
    def _greedy_route(start: int, nxt: int, edges: list,
                      val_of: dict, hops: int = 4) -> set:
        """Trace the greedy argmax-value climb a few hops ahead from `nxt`."""
        out: set = set()
        if start < 0 or nxt < 0:
            return out
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


class _PlayBearingDial(QWidget):
    """Top-down dial for the chosen egocentric play bearing (fx, fy) — gold while
    climbing, violet while wandering beyond the frontier."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        title = QLabel("Play bearing (fx,fy) — forward = up")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)

        import numpy as np
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
        fx = float(snap.get("fx", 0.0) or 0.0)
        fy = float(snap.get("fy", 0.0) or 0.0)
        mag = math.hypot(fx, fy)
        wandering = bool(snap.get("wandering", False))
        if mag > 1e-6:
            scale = min(1.1, mag) / mag
            ex, ey = fx * scale, fy * scale
        else:
            ex, ey = 0.0, 0.0
        col = (200, 130, 255) if wandering else (255, 210, 60)   # violet wander / gold climb
        self._needle.setData([0, ex], [0, ey], pen=pg.mkPen(*col, width=3))
        self._tip.setData([ex], [ey], brush=pg.mkBrush(*col, 255))


class _PlayStateMeter(QWidget):
    """The CLIMB↔WANDER decision as a CENTERED gauge — the play analogue of the planner's
    PLANNING↔WANDERING gauge. Push RIGHT (gold) toward CLIMB when ascending the novelty
    field to the frontier; push LEFT (violet) toward WANDER when run-and-tumbling BEYOND the
    mapped graph. STALL-WANDER (forced_wander, magenta) is the map-stopped-growing override."""

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
        self._plot.plot([0, 0], [-0.7, 0.7], pen=pg.mkPen(200, 200, 200, 120, width=1))
        self._bar = pg.BarGraphItem(x0=[0], x1=[0], y0=[-0.45], y1=[0.45],
                                    brushes=[pg.mkBrush(200, 130, 255)])
        self._plot.addItem(self._bar)
        lw = pg.TextItem("WANDER ◄", color=(190, 150, 220), anchor=(0, 0.5))
        lw.setPos(-1.12, 0.0)
        rp = pg.TextItem("► CLIMB", color=(255, 210, 60), anchor=(1, 0.5))
        rp.setPos(1.12, 0.0)
        self._plot.addItem(lw)
        self._plot.addItem(rp)
        layout.addWidget(self._plot)

    def update_payload(self, s: dict) -> None:
        if not isinstance(s, dict):
            return
        climbing = bool(s.get("climbing", False))
        wandering = bool(s.get("wandering", False))
        forced = bool(s.get("forced_wander", False))
        nov_cur = float(s.get("novelty_cur", 0.0) or 0.0)
        v_peak = float(s.get("value_peak", 0.0) or 0.0)
        stale = float(s.get("stale_explore", 0.0) or 0.0)
        # climb magnitude = how high up the novelty field the current node sits;
        # wander magnitude grows with the stall (ticks since the map last grew).
        climb_frac = max(0.0, min(1.0, nov_cur / v_peak)) if v_peak > 1e-6 else 0.0
        stall_frac = max(0.0, min(1.0, stale / 60.0))
        if forced:
            self._state.setText("STALL-WANDER")
            self._state.setStyleSheet("color:#ff6bd2; font-size:20px; font-weight:bold;")
            x1 = -max(0.06, stall_frac)
            col = (255, 107, 210)
        elif wandering:
            self._state.setText("WANDER")
            self._state.setStyleSheet("color:#c882ff; font-size:20px; font-weight:bold;")
            x1 = -max(0.06, stall_frac)
            col = (200, 130, 255)
        elif climbing:
            self._state.setText("CLIMB")
            self._state.setStyleSheet("color:#ffd23c; font-size:20px; font-weight:bold;")
            x1 = max(0.02, climb_frac)
            col = (255, 210, 60)
        else:
            self._state.setText("—")
            self._state.setStyleSheet("color:#888; font-size:20px; font-weight:bold;")
            x1 = 0.0
            col = (120, 120, 130)
        self._bar.setOpts(x0=[0.0], x1=[float(x1)], brushes=[pg.mkBrush(*col)])


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


class _PlayDriverMeters(QWidget):
    """PlayLoop's live drivers: what is pulling the epistemic loop right now."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(3)
        title = QLabel("drivers")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(title)
        self._novelty = _Meter("novelty — place-EPM TLE @ cur (model degradation)", (210, 130, 255))
        self._boredom = _Meter("boredom — max habituation (sweep pressure)", (180, 130, 235))
        self._eatcr = _Meter("eat-credit — EMA(episode → real eat)", (90, 210, 130))
        self._playval = _Meter("play_value → arbiter (frontier value, energy-gated)", (255, 210, 60))
        for m in (self._novelty, self._boredom, self._eatcr, self._playval):
            layout.addWidget(m)
        layout.addStretch(1)

    def update_payload(self, s: dict) -> None:
        if not isinstance(s, dict):
            return
        nov = float(s.get("novelty_cur", 0.0) or 0.0)
        v_peak = float(s.get("value_peak", 0.0) or 0.0)
        boredom = float(s.get("max_hab", 0.0) or 0.0)
        eatcr = float(s.get("eat_credit", 0.0) or 0.0)
        playval = float(s.get("play_value", 0.0) or 0.0)
        # novelty is unbounded; show it as a fraction of its own running peak so the bar reads.
        self._novelty.set((nov / v_peak) if v_peak > 1e-6 else 0.0, f"{nov:.3f}")
        self._boredom.set(boredom, f"{boredom:.2f}")
        self._eatcr.set(eatcr, f"{eatcr:.2f}")
        self._playval.set(playval, f"{playval:.3f}")


class _PlayStatus(QWidget):
    """Slim discrete map/route facts (the state headline is the _PlayStateMeter gauge above)."""

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
        rows = [
            ("route",        hop),
            ("n_nodes",      int(s.get("n_nodes", 0) or 0)),
            ("cur_heading°", f"{math.degrees(float(s.get('cur_heading', 0.0) or 0.0)):.1f}"),
            ("bearing°",     f"{bearing:.1f}"),
            ("play_value",   f"{float(s.get('play_value', 0.0) or 0.0):.3f}"),   # → L2 arbiter
            ("novelty@cur",  f"{float(s.get('novelty_cur', 0.0) or 0.0):.3f}"),  # place-EPM TLE
            ("V_play peak",  f"{float(s.get('value_peak', 0.0) or 0.0):.3f}"),   # play_value normaliser
            ("max_hab",      f"{float(s.get('max_hab', 0.0) or 0.0):.3f}"),      # boredom (sweep pressure)
            ("eat_credit",   f"{float(s.get('eat_credit', 0.0) or 0.0):.3f}"),
            ("stale_ticks",  int(s.get("stale_explore", 0) or 0)),               # ticks since the map grew
        ]
        self._lbl.setText("\n".join(f"{k:>13}: {v:>10}" for k, v in rows))


class _PlayValueSeries(QWidget):
    """Rolling value dynamics — the play 'value race' analogue: the epistemic value it
    publishes to the L2 arbiter (play_value) plus the novelty field it climbs (novelty@cur,
    V_play peak) and the boredom that makes it sweep."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._plot = MultiSeriesPlot(
            [
                Series("play_value", "play_value → arbiter", (255, 210, 60), width=2.0),
                Series("novelty_cur", "novelty @ cur (TLE)", (210, 130, 255), width=1.5),
                Series("value_peak", "V_play peak (normaliser)", (200, 200, 200), width=1.0,
                       style=Qt.PenStyle.DashLine),
                Series("max_hab", "boredom (max hab)", (150, 120, 200), width=1.0),
            ],
            title="play value dynamics  (frontier value + the novelty field it climbs)",
            y_label="value",
        )
        layout.addWidget(self._plot)

    def update_payload(self, snapshot: dict) -> None:
        self._plot.update_payload(snapshot)


class PlayLoopInspector(QWidget):
    def __init__(self, module_id: str, module_type: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})  —  epistemic GROW loop (novelty → frontier)")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._graph = _PlayGraphView()
        self._dial = _PlayBearingDial()
        self._state_meter = _PlayStateMeter()
        self._meters = _PlayDriverMeters()
        self._status = _PlayStatus()
        self._series = _PlayValueSeries()

        right = QSplitter(Qt.Orientation.Vertical)
        right.addWidget(self._dial)
        right.addWidget(self._state_meter)
        right.addWidget(self._meters)
        right.addWidget(self._status)
        right.setSizes([200, 120, 220, 160])

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
