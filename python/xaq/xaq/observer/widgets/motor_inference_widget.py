"""
motor_inference_widget.py — Motor inference diagnostic panel.

Three sub-panels:
  1. Position strip  — ball_x, paddle_x, EPM motor target (PC1 of active node)
  2. Trust vote bars — per-modality trust weight, active EPM highlighted
  3. PCA scatter     — GNG topology in 2D latent space, active node as star
"""
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtGui import QPainter, QColor, QFont
from PyQt6.QtCore import Qt

_COLORS = ["#00CFFF", "#FF9500", "#44FF88", "#FF4444", "#CC88FF", "#FFFF44"]


# ---------------------------------------------------------------------------
# Trust bar chart (custom painted)
# ---------------------------------------------------------------------------

class _TrustBarsWidget(QWidget):
    """Horizontal bar chart: one bar per modality showing current trust weight."""

    def __init__(self, modalities, parent=None):
        super().__init__(parent)
        self._mods   = list(modalities)
        self._trust  = {m: 0.0 for m in modalities}
        self._active = modalities[0] if modalities else ""
        row_h = 28
        self.setMinimumHeight(max(56, len(modalities) * row_h))
        self.setMaximumHeight(max(56, len(modalities) * row_h) + 4)

    def update_trust(self, trust_weights: dict, active_mod: str):
        self._trust.update(trust_weights)
        self._active = active_mod
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        n = max(1, len(self._mods))
        row_h = h / n

        for i, m in enumerate(self._mods):
            y    = int(i * row_h) + 1
            rh   = max(1, int(row_h) - 3)
            tw   = max(0.0, min(1.0, self._trust.get(m, 0.0)))
            col  = QColor(_COLORS[i % len(_COLORS)])
            is_a = m == self._active

            # Background track
            p.fillRect(0, y, w, rh, QColor(22, 22, 33))
            # Bar fill
            bar_w = int(w * tw)
            fill = col if is_a else QColor(col.red() // 3, col.green() // 3, col.blue() // 3)
            if bar_w > 0:
                p.fillRect(0, y, bar_w, rh, fill)
            # Label
            p.setPen(QColor(230, 230, 230) if is_a else QColor(120, 120, 120))
            font = QFont("monospace", 9)
            font.setBold(is_a)
            p.setFont(font)
            tag = "▶ " if is_a else "  "
            label = f"{tag}{m}  {tw:.2f}"
            p.drawText(4, y, w - 8, rh, Qt.AlignmentFlag.AlignVCenter, label)


# ---------------------------------------------------------------------------
# Motor inference composite widget
# ---------------------------------------------------------------------------

class MotorInferenceWidget(QWidget):
    """
    Three-panel motor inference diagnostic.

    Parameters
    ----------
    modalities  : list of modality names
    game_w      : canvas pixel width (for normalizing ball_x / paddle_x)
    window_size : number of ticks to keep in rolling history
    """

    def __init__(self, modalities, game_w=320, window_size=200, parent=None):
        super().__init__(parent)
        self._mods   = list(modalities)
        self._game_w = game_w
        self._win    = window_size

        # Rolling history buffers (all normalized to [0, 1])
        self._ball_h   = np.full(window_size, 0.5, dtype=np.float32)
        self._paddle_h = np.full(window_size, 0.5, dtype=np.float32)
        self._motor_h  = np.full(window_size, 0.5, dtype=np.float32)

        # PCA sign stabilisation — maps node_id → last known (pc1, pc2) position.
        # On each update we check whether the majority of shared nodes have flipped
        # sign on a given axis; if so, we negate that column before rendering.
        # This is independent of the adapter's own sign tracking so it also covers
        # flips that arise when the active modality switches.
        self._pca_node_pos: dict = {}  # {node_id: np.ndarray shape (2,)}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        lbl = {"color": "#AAAAAA", "font-size": "8pt"}
        dash = pg.Qt.QtCore.Qt.PenStyle.DashLine

        # ---- 1. Position strip ----
        self._strip = pg.PlotWidget(title="Position Tracking (normalised)")
        self._strip.setBackground("#121212")
        self._strip.setYRange(0, 1)
        self._strip.setLabel("left", "x", **lbl)
        self._strip.showGrid(x=True, y=True, alpha=0.3)
        self._strip.addLegend(offset=(-10, 10))
        self._ball_c   = self._strip.plot(pen=pg.mkPen("#FF4444", width=2),   name="ball_x")
        self._paddle_c = self._strip.plot(pen=pg.mkPen("#44FF88", width=2),   name="paddle_x")
        self._motor_c  = self._strip.plot(pen=pg.mkPen("#FFFF44", width=1.5, style=dash), name="EPM target")
        layout.addWidget(self._strip, stretch=3)

        # ---- 2. Trust bars ----
        self._trust_w = _TrustBarsWidget(modalities)
        layout.addWidget(self._trust_w, stretch=0)

        # ---- 3. PCA scatter ----
        self._scatter_pw = pg.PlotWidget(title="Latent Space — GNG Topology (PCA)")
        self._scatter_pw.setBackground("#121212")
        self._scatter_pw.setLabel("left",   "PC2", **lbl)
        self._scatter_pw.setLabel("bottom", "PC1", **lbl)
        self._scatter_pw.showGrid(x=True, y=True, alpha=0.3)
        self._nodes_sc  = pg.ScatterPlotItem(
            size=7, brush=pg.mkBrush("#00CFFF60"), pen=pg.mkPen(None)
        )
        self._baked_sc  = pg.ScatterPlotItem(
            size=7, brush=pg.mkBrush("#FFD70080"), pen=pg.mkPen(None)
        )
        self._active_sc = pg.ScatterPlotItem(
            size=16, symbol="star",
            brush=pg.mkBrush("#FF4444"), pen=pg.mkPen("#ffffff", width=1)
        )
        self._scatter_pw.addItem(self._nodes_sc)
        self._scatter_pw.addItem(self._baked_sc)
        self._scatter_pw.addItem(self._active_sc)
        layout.addWidget(self._scatter_pw, stretch=4)

    # ------------------------------------------------------------------
    # Update methods (called from V3BrainServerWindow._update_frame)
    # ------------------------------------------------------------------

    def update_game_state(self, ball_x: float, paddle_x: float):
        """Raw pixel coords; normalises internally using game_w."""
        gw = max(1, self._game_w)
        bn = float(ball_x) / gw
        pn = float(paddle_x) / gw
        self._ball_h[:-1]   = self._ball_h[1:]
        self._ball_h[-1]    = bn
        self._paddle_h[:-1] = self._paddle_h[1:]
        self._paddle_h[-1]  = pn
        self._ball_c.setData(self._ball_h)
        self._paddle_c.setData(self._paddle_h)

    def update_trust(self, trust_weights: dict, active_mod: str):
        self._trust_w.update_trust(trust_weights, active_mod)

    def update_viz(self, viz_data: dict, active_node_id: int):
        """
        Refresh PCA scatter and derive the motor target from the active node's PC1.
        viz_data: dict from adapter.get_visualization_data()
        active_node_id: current winner node ID (from stats["active_node"])
        """
        if not viz_data:
            return
        pca_raw = viz_data.get("pca_coords")   # (N, 2) float32
        indices = viz_data.get("indices")       # (N,) int64 node IDs
        nodes_m = viz_data.get("nodes", [])     # list of node metadata dicts

        if pca_raw is None or len(pca_raw) == 0:
            return

        # Work on a copy so we don't mutate the shared viz dict
        pca = np.asarray(pca_raw, dtype=np.float32).copy()

        # Use the PCA coordinates directly from the adapter.
        # Stabilisation (anti-flip/anti-swap) is now handled in the backend.
        pca = np.asarray(pca_raw, dtype=np.float32)

        # Update stored positions for next frame (metrics/target calculation)
        if indices is not None:
            idx_arr = np.asarray(indices)
            self._pca_node_pos = {int(idx_arr[j]): pca[j].copy()
                                  for j in range(len(pca))}

        xs = pca[:, 0].tolist()
        ys = pca[:, 1].tolist()

        # Separate baked vs unbaked for colour distinction
        baked_mask = np.array([n.get("baked", False) for n in nodes_m], dtype=bool)
        if len(baked_mask) == len(xs):
            ubx = [x for x, b in zip(xs, baked_mask) if not b]
            uby = [y for y, b in zip(ys, baked_mask) if not b]
            bx  = [x for x, b in zip(xs, baked_mask) if b]
            by_  = [y for y, b in zip(ys, baked_mask) if b]
        else:
            ubx, uby, bx, by_ = xs, ys, [], []

        self._nodes_sc.setData(x=ubx, y=uby)
        self._baked_sc.setData(x=bx,  y=by_)

        # Active node
        if active_node_id >= 0 and indices is not None:
            idx_arr = np.asarray(indices)
            hits = np.where(idx_arr == active_node_id)[0]
            if len(hits) > 0:
                ai = hits[0]
                ax, ay = float(pca[ai, 0]), float(pca[ai, 1])
                self._active_sc.setData(x=[ax], y=[ay])

                # Motor target: normalise active node's PC1 across all nodes
                xmin, xmax = pca[:, 0].min(), pca[:, 0].max()
                x_range = float(xmax - xmin)
                motor_t = (ax - float(xmin)) / x_range if x_range > 1e-6 else 0.5
                self._motor_h[:-1] = self._motor_h[1:]
                self._motor_h[-1]  = motor_t
                self._motor_c.setData(self._motor_h)
            else:
                self._active_sc.setData(x=[], y=[])
        else:
            self._active_sc.setData(x=[], y=[])
