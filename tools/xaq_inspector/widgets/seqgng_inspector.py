"""SequenceGNG dashboard — n-gram clusterer state for the chunk pipeline.

Four panels:
  * Cluster growth time-series — node_count + baked_count over the
    inspector's session.  The headline "is SeqGNG finding stable
    sub-patterns?" signal — flat baked_count = no chunks forming yet.
  * Match scalars — current_motif_id, match_confidence, motif_phase.
    Rolling line plot so the user sees how often the active motif
    switches and how well incoming windows fit existing clusters.
  * Per-motif visit-count distribution — bar plot of GNG-node
    visit counts, baked nodes coloured distinctly.  Answers the
    "are these meaningful clusters or noise crystals?" gut-check:
    concentrated distribution with a few high-visit baked nodes is
    healthy; flat distribution across many low-visit nodes is the
    bake-failing-due-to-volatility pattern we hit in Rung 7.1.
  * Transition matrix heatmap — built from successor_counts.  Reveals
    motif-to-motif dynamics; off-diagonal mass = the body has
    repeatable sequential structure that chunks can capture.

Source data: SequenceGNG::snapshot_state() returns
  {gng: {nodes:[{visit_count, ...}]}, winner_window, current_motif_id,
   match_confidence, motif_phase, just_baked, successor_counts:[[a,b,c]]}
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


# ---------------------------------------------------------------------------
# Per-motif visit-count distribution (bar plot)
# ---------------------------------------------------------------------------

class _VisitCountBars(QWidget):
    """Bar plot of GNG node visit counts.  Baked nodes coloured.

    Sorted descending so the dominant clusters are leftmost.  Y-axis
    auto-scales; if the top bar dwarfs the rest, that IS the signal —
    one motif is dominating.  Flat profile = no concentration = bake
    won't fire.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Per-node visit counts — awaiting payload")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=False, y=True, alpha=0.25)
        self._plot.setMouseEnabled(x=False, y=True)
        self._plot.setLabel("left",   "visits")
        self._plot.setLabel("bottom", "node (visit-rank)")
        self._bar_baked = pg.BarGraphItem(
            x=[], height=[], width=0.8,
            brush=pg.mkBrush(120, 220, 140, 220),
        )
        self._bar_raw = pg.BarGraphItem(
            x=[], height=[], width=0.8,
            brush=pg.mkBrush(110, 130, 200, 220),
        )
        self._plot.addItem(self._bar_raw)
        self._plot.addItem(self._bar_baked)
        layout.addWidget(self._plot)

        self._latest: Optional[list[tuple[int, bool]]] = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(150)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        gng = snapshot.get("gng") if isinstance(snapshot, dict) else None
        if not isinstance(gng, dict):
            return
        nodes = gng.get("nodes")
        if not isinstance(nodes, list):
            return
        # baking_threshold lives at the gng JSON top level (gng.cpp:554).
        baking_threshold = int(gng.get("baking_threshold", 50))
        # Each node has `visits` (NOT visit_count — gng.cpp:580).  is_baked
        # isn't serialised; we infer it from visits >= baking_threshold.
        pairs: list[tuple[int, bool]] = []
        for nd in nodes:
            if not isinstance(nd, dict):
                continue
            vc = int(nd.get("visits", 0) or 0)
            baked = vc >= baking_threshold
            pairs.append((vc, baked))
        # Sort descending by visit count.
        pairs.sort(key=lambda p: p[0], reverse=True)
        self._latest = pairs
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        pairs = self._latest
        n = len(pairs)
        # Build two parallel arrays for baked and raw nodes; missing
        # values are 0 so the BarGraphItems line up at the same x.
        x_baked = [i for i, (_, b) in enumerate(pairs) if b]
        h_baked = [vc for (vc, b) in pairs if b]
        x_raw   = [i for i, (_, b) in enumerate(pairs) if not b]
        h_raw   = [vc for (vc, b) in pairs if not b]
        self._bar_baked.setOpts(x=x_baked, height=h_baked, width=0.8)
        self._bar_raw.setOpts(  x=x_raw,   height=h_raw,   width=0.8)
        baked_n = len(x_baked)
        if pairs:
            top_vc = pairs[0][0]
        else:
            top_vc = 0
        self._title.setText(
            f"Per-node visit counts  —  nodes {n}   baked {baked_n}   "
            f"top {top_vc} visits"
        )


# ---------------------------------------------------------------------------
# Transition matrix heatmap (from successor_counts)
# ---------------------------------------------------------------------------

class _TransitionMatrix(QWidget):
    """Heatmap of motif-to-motif transitions from successor_counts.

    Off-diagonal mass = the body has sequential dynamics that chunks can
    capture.  Pure diagonal (self-loops only) = motifs are sticky and
    not transitioning — also a chunk failure mode.  Sparse/empty matrix
    = no recurring transitions at all.
    """

    MAX_DIM = 64   # cap for display; show top-K motifs by visit count

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Transition matrix — awaiting payload")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._view = pg.PlotWidget()
        self._view.setBackground("k")
        self._view.setMouseEnabled(x=False, y=False)
        self._view.setLabel("left",   "from motif (visit-rank)")
        self._view.setLabel("bottom", "to motif (visit-rank)")
        self._image = pg.ImageItem(axisOrder="row-major")
        self._view.addItem(self._image)
        layout.addWidget(self._view)

        self._cmap = pg.colormap.get("inferno")
        self._latest: Optional[np.ndarray] = None
        self._latest_off_diag = 0
        self._latest_self     = 0
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(200)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        succ = snapshot.get("successor_counts")
        if not isinstance(succ, list):
            return
        # Build a (from, to, count) table.  Then restrict to the top-K
        # motifs by total visits (sum of counts entering+leaving).
        triples: list[tuple[int, int, int]] = []
        for triple in succ:
            try:
                a, b, c = int(triple[0]), int(triple[1]), int(triple[2])
                triples.append((a, b, c))
            except (TypeError, ValueError, IndexError):
                continue
        if not triples:
            return
        # Aggregate per-motif visit count proxy = sum of out-edges.
        out_sum: dict[int, int] = {}
        for a, b, c in triples:
            out_sum[a] = out_sum.get(a, 0) + c
            out_sum[b] = out_sum.get(b, 0)  # ensure b is in dict
        # Rank motifs by visit proxy.
        ranked = sorted(out_sum.keys(), key=lambda m: -out_sum.get(m, 0))[: self.MAX_DIM]
        idx_of = {m: i for i, m in enumerate(ranked)}
        n = len(ranked)
        mat = np.zeros((n, n), dtype=float)
        self_loops = 0
        off_diag   = 0
        for a, b, c in triples:
            if a in idx_of and b in idx_of:
                mat[idx_of[a], idx_of[b]] += c
                if a == b: self_loops += c
                else:      off_diag   += c
        self._latest = mat
        self._latest_off_diag = off_diag
        self._latest_self     = self_loops
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        mat = self._latest
        # Log scale to surface low-mass cells alongside high-mass.
        with np.errstate(divide="ignore"):
            logm = np.log10(mat + 1.0)
        max_v = float(logm.max()) if logm.size else 0.0
        levels = (0.0, max(max_v, 0.1))
        self._image.setImage(logm, levels=levels, autoLevels=False)
        try:
            self._image.setLookupTable(self._cmap.getLookupTable(0.0, 1.0, 256))
        except Exception:
            pass
        total = self._latest_off_diag + self._latest_self
        ratio = (self._latest_self / total) if total else 0.0
        self._title.setText(
            f"Transition matrix  —  motifs {mat.shape[0]}   "
            f"self-loops {self._latest_self}  cross {self._latest_off_diag}  "
            f"(self/total {ratio:.2f})"
        )


# ---------------------------------------------------------------------------
# Winner-window sparkline
# ---------------------------------------------------------------------------

class _WinnerWindow(QWidget):
    """Recent winner_id sequence as a tiny scatter.

    Shows the last N winner_ids fed into the n-gram window.  Diversity
    = signal volatility (the Rung 7.1 failure pattern); repeated runs
    of the same id = a stuck input.  The current motif_id is annotated
    in the title.
    """

    BUFFER = 240   # recent winner_ids to display

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Winner window — awaiting payload")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=False, y=True, alpha=0.2)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setLabel("left",   "winner_id / chosen_intent")
        self._plot.setLabel("bottom", "recent samples →")
        self._scatter = pg.ScatterPlotItem(
            x=[], y=[], size=4, pen=None,
            brush=pg.mkBrush(200, 220, 255, 200),
        )
        self._plot.addItem(self._scatter)
        layout.addWidget(self._plot)

        self._buf: np.ndarray = np.full(self.BUFFER, np.nan)
        self._motif_id = -1
        self._match = 0.0
        self._phase = 0
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(100)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        ww = snapshot.get("winner_window")
        if isinstance(ww, list) and ww:
            try:
                last = float(ww[-1])
                self._buf = np.roll(self._buf, -1)
                self._buf[-1] = last
            except (TypeError, ValueError):
                pass
        else:
            # Fall back to action_window for source_kind="action".
            aw = snapshot.get("action_window")
            if isinstance(aw, list) and aw:
                try:
                    last = float(aw[-1])
                    self._buf = np.roll(self._buf, -1)
                    self._buf[-1] = last
                except (TypeError, ValueError):
                    pass
        self._motif_id = int(snapshot.get("current_motif_id", -1) or -1)
        try:
            self._match = float(snapshot.get("match_confidence", 0.0) or 0.0)
        except (TypeError, ValueError):
            self._match = 0.0
        self._phase = int(snapshot.get("motif_phase", 0) or 0)
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        xs = np.arange(self.BUFFER)
        finite = np.isfinite(self._buf)
        self._scatter.setData(x=xs[finite], y=self._buf[finite])
        self._title.setText(
            f"Winner window  —  motif {self._motif_id}   "
            f"phase {self._phase}   match {self._match:.2f}"
        )


# ---------------------------------------------------------------------------
# Top-level inspector
# ---------------------------------------------------------------------------

class SeqGNGInspector(QWidget):
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

        # Cluster growth: node_count + baked_count over time.  Both come
        # from the underlying GNG state (node count = len(gng.nodes);
        # baked = count of nodes with visit_count >= threshold).  The
        # snapshot exposes them via gng.node_count / gng.baked_count.
        self._growth = MultiSeriesPlot(
            [
                Series("gng.node_count",  "nodes",  (120, 130, 220), width=1.6),
                Series("gng.baked_count", "baked",  ( 60, 220, 140), width=1.8),
            ],
            title="Cluster growth (node_count + baked_count)",
            y_label="count",
        )
        # Match scalars: current_motif_id + match_confidence + phase
        # (low-frequency content; gives a feel for stability).
        self._scalars = MultiSeriesPlot(
            [
                Series("current_motif_id", "motif", (220, 130,  90), width=1.2),
                Series("match_confidence", "match", (120, 220, 255), width=1.5),
                Series("motif_phase",      "phase", (200, 200,  80), width=1.0,
                       style=Qt.PenStyle.DashLine),
            ],
            title="Active motif + match confidence + phase",
            y_label="value",
        )
        self._winner = _WinnerWindow()
        self._visits = _VisitCountBars()
        self._trans  = _TransitionMatrix()

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._growth)
        top.addWidget(self._scalars)
        top.setSizes([520, 520])

        mid = QSplitter(Qt.Orientation.Horizontal)
        mid.addWidget(self._winner)
        mid.addWidget(self._visits)
        mid.setSizes([520, 520])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(mid)
        v.addWidget(self._trans)
        v.setSizes([280, 280, 320])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        # The GNG JSON (gng.cpp:547) doesn't serialise node_count or
        # baked_count as fields — they have to be derived from
        # len(gng.nodes) and a visits>=baking_threshold count.  Inject
        # them into the snapshot dict so MultiSeriesPlot's dotted-key
        # resolver ("gng.node_count" / "gng.baked_count") finds them.
        gng = snapshot.get("gng")
        if isinstance(gng, dict):
            nodes = gng.get("nodes") if isinstance(gng.get("nodes"), list) else []
            threshold = int(gng.get("baking_threshold", 50))
            gng["node_count"]  = len(nodes)
            gng["baked_count"] = sum(
                1 for nd in nodes
                if isinstance(nd, dict)
                and int(nd.get("visits", 0) or 0) >= threshold
            )
        self._growth.update_payload(snapshot)
        self._scalars.update_payload(snapshot)
        self._winner.update_payload(snapshot)
        self._visits.update_payload(snapshot)
        self._trans.update_payload(snapshot)
