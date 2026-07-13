import sys
import os
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QGraphicsOpacityEffect, QPushButton
from PyQt6.QtCore import pyqtSignal, Qt, QPropertyAnimation, QEasingCurve, QTimer, QPoint
from PyQt6 import QtGui
import pyqtgraph as pg
import numpy as np

class GraphCanvas(QWidget):
    """
    High-performance 2D Canvas for visualizing the Knowledge Graph.
    Uses PyQtGraph's PlotWidget (standard Qt painting) to avoid GLX/OpenGL drivers issues.
    """
    node_selected = pyqtSignal(str) # Emits Node ID
    nodes_selected = pyqtSignal(list) # Emits list of Node IDs for multi-select
    toggle_suspension = pyqtSignal(bool) # Emits True if user clicked Suspend

    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

        # 2D Plot Widget
        self.view = pg.PlotWidget()
        self.view.setBackground('k') # Black background
        self.view.showGrid(x=True, y=True, alpha=0.3)
        self.view.setAspectLocked(True)
        # Initial range matches v3 PCA coordinate space (±20 from adapter).
        self.view.setRange(xRange=(-30, 30), yRange=(-30, 30), padding=0)
        
        # Edges (Multiple layers for weighted opacity)
        self.edge_layers = []
        opacities = [40, 80, 140, 200, 255]
        widths = [0.5, 0.5, 1.0, 1.0, 1.5]
        for i in range(5):
             layer = pg.GraphItem()
             pen = pg.mkPen((180, 180, 180, opacities[i]), width=widths[i]) # Grey base
             layer.setData(pen=pen)
             self.view.addItem(layer)
             self.edge_layers.append(layer)
        
        # Active Path Highlighting (Top layer)
        self.active_path_item = pg.GraphItem()
        path_pen = pg.mkPen((100, 255, 255, 255), width=2)
        self.active_path_item.setData(pen=path_pen)
        self.view.addItem(self.active_path_item)
        
        # Glow Layer (Underneath nodes)
        self.glow_scatter = pg.ScatterPlotItem(
            size=30, 
            pen=pg.mkPen(None),
            hoverable=False
        )
        self.view.addItem(self.glow_scatter)
        self.glow_nodes = {} # node_id -> {'pos': (x,y), 'opacity': alpha}
        
        # Optimization: Pre-calculate Glow Brushes for all opacities
        self.glow_brushes = [] # Cyan
        self.super_glow_brushes = [] # Purple
        for i in range(256):
            # Cyan Glow
            grad = QtGui.QRadialGradient(0.5, 0.5, 0.5)
            grad.setCoordinateMode(QtGui.QGradient.CoordinateMode.ObjectBoundingMode)
            color = QtGui.QColor(0, 255, 255, i)
            grad.setColorAt(0, color); grad.setColorAt(1, QtGui.QColor(0, 255, 255, 0))
            self.glow_brushes.append(QtGui.QBrush(grad))
            
            # Purple Glow (Supernodes)
            grad_s = QtGui.QRadialGradient(0.5, 0.5, 0.5)
            grad_s.setCoordinateMode(QtGui.QGradient.CoordinateMode.ObjectBoundingMode)
            color_s = QtGui.QColor(168, 85, 247, i) # #A855F7
            grad_s.setColorAt(0, color_s); grad_s.setColorAt(1, QtGui.QColor(168, 85, 247, 0))
            self.super_glow_brushes.append(QtGui.QBrush(grad_s))
            
        self.supernode_ids = set() # Cache for rendering
            
        # UI Styling Cache
        self.brush_cache = {}
        self.active_brush = pg.mkBrush(0, 255, 255, 255)
        self.default_brush = pg.mkBrush(100, 100, 100, 150)
        self.baked_brush = pg.mkBrush(239, 68, 68, 200) # #EF4444 (Red)
        self.selected_brush = pg.mkBrush(244, 114, 182, 255)
        self.multi_selected_brush = pg.mkBrush(236, 72, 153, 200)

        # Scatter Plot Item (Nodes on top)
        self.scatter = pg.ScatterPlotItem(
            size=10, 
            pen=pg.mkPen(None), 
            brush=pg.mkBrush(255, 255, 255, 120),
            hoverable=True,
            pxMode=True, # Ensure consistent sizing
            tip=None # Disable default coordinate tooltip
        )
        self.view.addItem(self.scatter)
        
        self.active_node_id = None # Track active node from Brain
        self.selected_node_id = None # Track manually selected node
        self.selected_node_ids = set() # For multi-select
        
        self.hovered_node_id = None # Track node under cursor (proximity)
        self.hover_ring = pg.ScatterPlotItem(
            size=25, 
            pen=pg.mkPen('y', width=2),
            brush=None,
            pxMode=True,
            hoverable=False
        )
        self.view.addItem(self.hover_ring)
        self.hover_ring.hide()

        self.selection_ring = pg.ScatterPlotItem(
            size=30, # Slightly larger/different than hover
            pen=pg.mkPen((244, 114, 182, 255), width=2),
            brush=None,
            pxMode=True,
            hoverable=False
        )
        self.view.addItem(self.selection_ring)
        self.selection_ring.hide()
        
        # Rubber Band Selection Box
        self.rubber_band = pg.RectROI([0, 0], [0, 0], pen=pg.mkPen((244, 114, 182, 255), width=2, style=Qt.PenStyle.DashLine), movable=False, resizable=False, rotatable=False)
        self.rubber_band.setZValue(1000)
        self.rubber_band.hide()
        self.view.addItem(self.rubber_band)
        
        self.drag_start_pos = None
        
        # Styling Cache (v30)
        self.node_ids = []
        self.node_metadata = {}
        self.cached_brushes = []
        self.cached_sizes = []
        self.node_positions = np.array([])

        # Leveled Suspension Overlay
        self.suspension_overlay = QLabel(self)
        self.suspension_overlay.setText("Graph Updates Suspended.\nUncheck 'Suspend Graph Updates' in Advanced Settings to Resume")
        self.suspension_overlay.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.suspension_overlay.setStyleSheet("""
            QLabel {
                background-color: rgba(0, 0, 0, 230);
                color: #22d3ee;
                font-weight: bold;
                font-size: 20px;
                border: 2px solid #0891b2;
                border-radius: 15px;
                padding: 20px;
            }
        """)
        self.suspension_overlay.setFixedWidth(650)
        self.suspension_overlay.setFixedHeight(120)
        self.suspension_overlay.hide()

        # Manual Suspension Button (Bottom Right)
        self.btn_suspend = QPushButton("Suspend Graph Updates", self)
        self.btn_suspend.setCheckable(True)
        self.btn_suspend.setFixedWidth(200)
        self.btn_suspend.setFixedHeight(40)
        self.btn_suspend.setStyleSheet("""
            QPushButton {
                background-color: rgba(20, 20, 20, 150);
                color: #22d3ee;
                border: 1px solid #0891b2;
                border-radius: 8px;
                font-weight: bold;
                font-size: 13px;
                text-align: center;
            }
            QPushButton:hover {
                background-color: rgba(8, 145, 178, 50);
                border: 1px solid #22d3ee;
            }
            QPushButton:checked {
                background-color: rgba(220, 38, 38, 100);
                color: #fca5a5;
                border: 1px solid #ef4444;
                text: "Resume Graph Updates";
            }
        """)
        self.btn_suspend.toggled.connect(self._on_suspend_clicked)
        self.btn_suspend.hide()
        
        # Interaction
        self.scatter.sigClicked.connect(self.on_point_clicked)
        # Mouse Move for Proximity
        self.view.scene().sigMouseMoved.connect(self.on_mouse_move)
        # Scene Click for Proximity (overrrides Scatter Click usually if we handle it first, 
        # but actually Scatter Click is preferred. We will trigger manually if needed.)
        self.view.scene().sigMouseClicked.connect(self.on_scene_clicked)
        
        # Shift-Drag interception on the ViewBox
        self.view.plotItem.vb.mouseDragEvent = self.custom_mouse_drag_event
        
        self.node_ids = [] # Parallel list to scatter points data
        self.node_positions = np.zeros((0, 2)) # Cache for fast search
        self.cached_edges = [] # Store edge data until positions are ready

        # Active Concept Popup
        self.popup_label = QLabel(self)
        self.popup_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.popup_label.setVisible(False)
        self.popup_effect = QGraphicsOpacityEffect(self.popup_label)
        self.popup_label.setGraphicsEffect(self.popup_effect)
        
        self.popup_anim = QPropertyAnimation(self.popup_effect, b"opacity")
        self.popup_anim.setDuration(1500) # 1.5s fade out
        self.popup_anim.setEasingCurve(QEasingCurve.Type.InQuad)
        self.popup_anim.finished.connect(self.popup_label.hide)
        
        self.layout.addWidget(self.view)
        
    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Center the suspension overlay
        if hasattr(self, 'suspension_overlay'):
            self.suspension_overlay.move(
                (self.width() - self.suspension_overlay.width()) // 2,
                (self.height() - self.suspension_overlay.height()) // 2
            )
        
        # Position manual suspend button (Bottom Right)
        if hasattr(self, 'btn_suspend'):
            margin = 20
            self.btn_suspend.move(
                self.width() - self.btn_suspend.width() - margin,
                self.height() - self.btn_suspend.height() - margin
            )

        # Keep popup centered if visible
        if hasattr(self, 'popup_label') and self.popup_label.isVisible():
            self._center_popup()

    def set_suspended_mode(self, suspended: bool):
        """Show/Hide the suspension overlay and CLEAR the graph UI."""
        if suspended == getattr(self, '_last_suspended_mode', None):
             return
        self._last_suspended_mode = suspended
        
        if suspended:
            self.scatter.hide()
            for layer in self.edge_layers: layer.hide()
            # self.active_path_item.hide() # v15: Always render active edge
            self.glow_scatter.hide()
            
            self.suspension_overlay.show()
            self.suspension_overlay.raise_()
        else:
            self.scatter.show()
            for layer in self.edge_layers: layer.show()
            self.active_path_item.show()
            self.glow_scatter.show()
            
            self.suspension_overlay.hide()

    def _on_suspend_clicked(self, checked):
        if checked:
            self.btn_suspend.setText("Resume Graph Updates")
        else:
            self.btn_suspend.setText("Suspend Graph Updates")
        self.toggle_suspension.emit(checked)

    def _center_popup(self):
        # Top Center with margin
        x = (self.width() - self.popup_label.width()) // 2
        y = 40 # Top margin
        self.popup_label.move(x, y)

    def show_concept_popup(self, text, color_str=None):
        """
        Displays a transient popup for the active concept.
        """
        self.popup_label.setText(text)
        
        # Style
        if not color_str: color_str = "#00FFFF" # Default Cyan
        
        # Calculate appropriate text color (black or white) based on background?
        # For now, assume dark background colors -> white text, or light -> black.
        # Simple fix: Use bold white text with a colored border/glow or semi-transparent background.
        
        # Let's use a style sheet for a pill shape
        css = f"""
            QLabel {{
                background-color: {color_str};
                color: black;
                border-radius: 15px;
                padding: 8px 16px;
                font-weight: bold;
                font-size: 16px;
                border: 2px solid white;
            }}
        """
        self.popup_label.setStyleSheet(css)
        self.popup_label.adjustSize()
        
        self._center_popup()
        
        # Reset Opacity and Start Animation
        self.popup_anim.stop()
        self.popup_effect.setOpacity(1.0)
        self.popup_label.show()
        self.popup_label.raise_()
        
        # Animate Fade Out (Start from 1.0, wait a bit, then fade? 
        # QPropertyAnimation interpolates. To wait, we might need a SequentialAnimation or just a slow fade.)
        # Let's just do a slow fade for now, maybe starting after a brief timer?
        # Simpler: Start value 1.0, End value 0.0. 
        self.popup_anim.setStartValue(1.0)
        self.popup_anim.setEndValue(0.0)
        self.popup_anim.start()

    def set_data(self, node_ids, node_positions, active_node_id=None, edges=None, metadata=None, last_transition=None):
        """
        Legacy wrapper for backward compatibility during refactor.
        Calls the specialized update methods.
        """
        # Always update positions
        self.update_positions(node_ids, node_positions)
        
        # Check if we need to update styling
        # We always pass metadata to update_styling to ensure latest state
        self.update_styling(node_ids, active_node_id, metadata)
        
        if edges is not None:
             self.update_edges(edges, node_positions, last_transition)
        elif hasattr(self, 'cached_edges'):
             # If edges not provided but positions moved, we must re-draw edges at new positions
             self.update_edges(self.cached_edges, node_positions, last_transition)

    def update_positions(self, node_ids, node_positions):
        """
        FAST PATH: Updates only the spatial coordinates of nodes and edges.
        Does NOT update colors, sizes, or topology unless necessary.
        """
        if len(node_ids) != len(self.cached_brushes):
             self.update_styling(node_ids, self.active_node_id)

        self.node_ids = node_ids
        self.node_positions = node_positions
        
        # 1. Update Scatter (Including Styles to prevent reset)
        self.scatter.setData(
            x=node_positions[:, 0], 
            y=node_positions[:, 1],
            brush=self.cached_brushes,
            size=self.cached_sizes,
            data=node_ids
        )
        
        # 2. Update Glow Positions (if any)
        if self.glow_nodes:
             gx, gy = [], []
             gbrushes = []
             id_map = {uid: i for i, uid in enumerate(node_ids)}
             for nid in list(self.glow_nodes.keys()):
                 if nid in id_map:
                     idx = id_map[nid]
                     if idx < len(node_positions):
                         pos = node_positions[idx]
                         self.glow_nodes[nid]["pos"] = pos
                         gx.append(pos[0]); gy.append(pos[1])
                         alpha = max(0, min(255, int(self.glow_nodes[nid]["opacity"])))
                         is_super = nid in self.supernode_ids
                         brush_pool = self.super_glow_brushes if is_super else self.glow_brushes
                         gbrushes.append(brush_pool[alpha])
             if gx:
                 view_rect = self.view.viewRect()
                 base_scale = 1.0 - (view_rect.width() - 400) / 1200.0
                 glow_scale = max(0.5, min(2.0, base_scale))
                 gsizes = [ (90 if nid in self.supernode_ids else 30) * glow_scale for nid in list(self.glow_nodes.keys()) if nid in id_map ]
                 try:
                     self.glow_scatter.setData(x=gx, y=gy, brush=gbrushes, size=gsizes)
                 except Exception as e: print(f"GraphCanvas Error (Glow): {e}")
        # v19/v29: Active Selection Ring + Robust ID Map cleanup
        self._id_map_cache = {uid: i for i, uid in enumerate(node_ids)}
        
        if self.selected_node_id and str(self.selected_node_id) in node_ids:
             idx = self._id_map_cache.get(str(self.selected_node_id))
             if idx is not None:
                  pos = node_positions[idx]
                  self.selection_ring.setData(x=[pos[0]], y=[pos[1]])
        
        # Hover Ring
        if self.hovered_node_id and str(self.hovered_node_id) in node_ids:
             if not hasattr(self, '_id_map_cache'):
                  self._id_map_cache = {uid: i for i, uid in enumerate(node_ids)}
             idx = self._id_map_cache.get(str(self.hovered_node_id))
             if idx is not None:
                  pos = node_positions[idx]
                  self.hover_ring.setData(x=[pos[0]], y=[pos[1]])

    def update_styling(self, node_ids, active_node_id, metadata=None):
        """
        Updates node visual styles (colors, selection, active glow).
        """
        self.node_ids = node_ids
        self.active_node_id = active_node_id
        if metadata is not None:
             self.node_metadata = metadata
             
        # v29: Sync ID map for edges/styling block
        self._id_map_cache = {uid: i for i, uid in enumerate(node_ids)}
        
        # 1. Update Glow Logic (State only)
        sid = str(active_node_id) if active_node_id is not None else None
        
        if sid and sid in node_ids:
             if sid not in self.glow_nodes:
                  self.glow_nodes[sid] = {'pos': (0,0), 'opacity': 30}
        
        # Decay Glows
        to_delete = []
        for nid in self.glow_nodes:
            if nid == sid:
                self.glow_nodes[nid]['opacity'] = min(200, self.glow_nodes[nid]['opacity'] + 80)
            else:
                self.glow_nodes[nid]['opacity'] -= 25 # v15: Faster decay (was 12)
                if self.glow_nodes[nid]['opacity'] <= 0:
                    to_delete.append(nid)
        for nid in to_delete: del self.glow_nodes[nid]

        # 2. Calculate Brushes and Sizes
        brushes = []
        sizes = []
        self.supernode_ids.clear()
        
        for uid in node_ids:
            uid_int = int(uid)
            uid_str = str(uid)
            md = self.node_metadata.get(uid_str, {})
            
            # Robust Supernode/Baked checks
            is_super = uid_int < 0 or md.get('is_super', False)
            is_baked = md.get('baked', False)
            custom_color = md.get('color')
            
            is_selected = self.selected_node_id == uid_str
            is_group_selected = uid_str in self.selected_node_ids
            
            # Size selection (3x for supernodes)
            base_size = 30 if is_super else 10
            sizes.append(base_size)
            
            if is_super: self.supernode_ids.add(uid_str)
            
            # Brush selection (Priority logic)
            if is_selected: brushes.append(self.selected_brush)
            elif is_group_selected: brushes.append(self.multi_selected_brush)
            elif sid and uid_str == sid: brushes.append(self.active_brush)
            elif custom_color and isinstance(custom_color, str) and custom_color.startswith('#'):
                if custom_color not in self.brush_cache:
                    self.brush_cache[custom_color] = pg.mkBrush(QtGui.QColor(custom_color))
                brushes.append(self.brush_cache[custom_color])
            elif is_super:
                # Default purple for supernodes if no custom color
                brushes.append(self.brush_cache.get("#A855F7", pg.mkBrush("#A855F7")))
            elif is_baked: brushes.append(self.baked_brush)
            else: brushes.append(self.default_brush)
            
        self.cached_brushes = brushes
        self.cached_sizes = sizes
        
        # 3. Apply to Scatter (if positions exist)
        if len(self.node_positions) == len(brushes):
             # v30: Simplified scale logic when in pxMode
             node_scale = 1.0 # Standard pixel size
             
             final_sizes = [s * node_scale for s in sizes]
             
             self.scatter.setData(
                 x=self.node_positions[:, 0], 
                 y=self.node_positions[:, 1], 
                 data=node_ids, 
                 brush=brushes, 
                 size=final_sizes
             )

    def update_edges(self, edges, node_positions, last_transition=None, throttled=False):
        """
        Updates edge rendering.
        edges: Adjacency Matrix (N, N)
        throttled: If True, only draw the ACTIVE edge.
        """
        self.cached_edges = edges
        
        # Dimension Check
        if edges is not None and node_positions is not None:
             if len(edges) != len(node_positions):
                  # Mismatch (Sync issue), skip update
                  return

        n_nodes = len(node_positions) if node_positions is not None else 0
        self.last_edge_node_count = n_nodes  # Track for safety in fast-path

        if edges is None or len(edges) == 0:
             for layer in self.edge_layers:
                  layer.setData(pos=node_positions, adj=np.array([], dtype=int))
             self.active_path_item.setData(pos=node_positions, adj=np.array([], dtype=int))
             return

        adj_mat = edges
        max_w = adj_mat.max() if adj_mat.size > 0 else 1.0
        if max_w == 0: max_w = 1.0
        
        # Tier 2 Optimization: Skip background edges if throttled
        if throttled:
             for layer in self.edge_layers:
                  layer.setData(pos=node_positions, adj=np.array([], dtype=int))
             rows, cols = np.array([], dtype=int), np.array([], dtype=int)
             weights = np.array([], dtype=float)
        else:
             rows, cols = np.where(adj_mat > 0.001)
             weights = adj_mat[rows, cols]
        
        # Safe Check for bounds
        valid_mask = (rows < n_nodes) & (cols < n_nodes)
        rows = rows[valid_mask]
        cols = cols[valid_mask]
        weights = weights[valid_mask]
        
        # OPTIMIZATION: Simplify rendering for large graphs
        # v15: Relaxed to 500 nodes (was 250)
        # v23: ONLY prune if throttled=True. Keep full detail for pauses/stops.
        min_tier = 0
        if throttled and n_nodes > 500:
            min_tier = 2 # Skip tiers 0 and 1 (weakest)
        
        norm_w = weights / max_w
        bins = (norm_w * 4.99).astype(int)
        
        for i in range(5):
             if i < min_tier:
                 # Hide weak layers entirely for performance
                 self.edge_layers[i].setData(pos=node_positions, adj=np.array([], dtype=int))
                 continue
                 
             mask = (bins == i)
             if mask.any():
                 layer_adj = np.column_stack((rows[mask], cols[mask]))
                 try:
                     self.edge_layers[i].setData(pos=node_positions, adj=layer_adj)
                 except: pass 
             else:
                 self.edge_layers[i].setData(pos=node_positions, adj=np.array([], dtype=int))
                 
        # v30: Path Highlighting (Lead-in only)
        # Reverting to last_transition for path traversal visualization, 
        # but retaining the robust v29 ID mapping logic.
        adj_path = []
        if last_transition:
            s_id, t_id = str(last_transition.get('source')), str(last_transition.get('target'))
            idx_s = self._id_map_cache.get(s_id)
            idx_t = self._id_map_cache.get(t_id)
            
            if idx_s is not None and idx_t is not None and idx_s < n_nodes and idx_t < n_nodes:
                adj_path = [[idx_s, idx_t]]
        
        if adj_path:
             self.active_path_item.setData(pos=node_positions, adj=np.array(adj_path, dtype=int))
             self.active_path_item.show()
        else:
             self.active_path_item.setData(pos=node_positions, adj=np.array([], dtype=int))

    def update_edges_positions_only(self, node_positions):
        """
        Updates ONLY the positions of existing edges. Assumes topology hasn't changed.
        CRITICAL: If node count changed since last full update, we must SKIP this 
        otherwise GraphItem crashes mapping old indices to new smaller position array.
        """
        if not hasattr(self, 'last_edge_node_count'):
            self.last_edge_node_count = 0
            
        if len(node_positions) != self.last_edge_node_count:
            # Count mismatch (Pruning/Growth happened, but Topology update is Throttled)
            # Skip update allows edges to "lag" (pointing to old positions or just staying put)
            # rather than crashing the app.
            return

        # Ensure we don't crash if buffer mismatch
        for layer in self.edge_layers:
             layer.setData(pos=node_positions)
        self.active_path_item.setData(pos=node_positions)

    def on_point_clicked(self, plot, points, event):
        if len(points) > 0:
            pt = points[0]
            mx, my = pt.pos().x(), pt.pos().y()
            node_id = None
            
            if len(self.node_positions) > 0:
                d_x = np.abs(self.node_positions[:, 0] - mx)
                d_y = np.abs(self.node_positions[:, 1] - my)
                mask = (d_x < 1.0) & (d_y < 1.0) 
                indices = np.where(mask)[0]
                
                if len(indices) > 0:
                    candidates = self.node_positions[indices]
                    d2 = (candidates[:, 0] - mx)**2 + (candidates[:, 1] - my)**2
                    best_local_idx = np.argmin(d2)
                    best_global_idx = indices[best_local_idx]
                    
                    if best_global_idx < len(self.node_ids):
                         node_id = str(self.node_ids[best_global_idx])
            
            if node_id:
                self.select_node(node_id)
                
    def on_scene_clicked(self, event):
        if self.hovered_node_id:
             self.select_node(self.hovered_node_id)
             event.accept()

    def custom_mouse_drag_event(self, ev, axis=None):
        modifiers = QtGui.QGuiApplication.keyboardModifiers()
        is_shift = (modifiers & Qt.KeyboardModifier.ShiftModifier)
        
        if is_shift and ev.button() == Qt.MouseButton.LeftButton:
            ev.accept()
            
            # Map screen pos to scene/view pos
            pos = self.view.plotItem.vb.mapToView(ev.pos())
            
            if ev.isStart():
                self.drag_start_pos = pos
                self.rubber_band.setPos(pos)
                self.rubber_band.setSize([0, 0])
                self.rubber_band.show()
                
            elif ev.isFinish():
                if self.drag_start_pos is not None:
                    # Finalize selection
                    self.rubber_band.hide()
                    p1 = self.drag_start_pos
                    p2 = pos
                    
                    x_min, x_max = min(p1.x(), p2.x()), max(p1.x(), p2.x())
                    y_min, y_max = min(p1.y(), p2.y()), max(p1.y(), p2.y())
                    
                    self.selected_node_ids.clear()
                    
                    if len(self.node_positions) > 0:
                        xs = self.node_positions[:, 0]
                        ys = self.node_positions[:, 1]
                        
                        mask = (xs >= x_min) & (xs <= x_max) & (ys >= y_min) & (ys <= y_max)
                        indices = np.where(mask)[0]
                        
                        for idx in indices:
                            if idx < len(self.node_ids):
                                self.selected_node_ids.add(str(self.node_ids[idx]))
                    
                    if len(self.selected_node_ids) > 0:
                        # Clear single selection ring
                        self.selected_node_id = None
                        self.selection_ring.hide()
                        
                        self.update_styling(self.node_ids, self.active_node_id)
                        self.nodes_selected.emit(list(self.selected_node_ids))
                    else:
                        self.clear_selection()
                        
                    self.drag_start_pos = None
            else:
                # During drag
                if self.drag_start_pos is not None:
                    p1 = self.drag_start_pos
                    p2 = pos
                    x_min, x_max = min(p1.x(), p2.x()), max(p1.x(), p2.x())
                    y_min, y_max = min(p1.y(), p2.y()), max(p1.y(), p2.y())
                    
                    self.rubber_band.setPos([x_min, y_min])
                    self.rubber_band.setSize([x_max - x_min, y_max - y_min])
        else:
            # Pass through to default viewbox panning
            pg.ViewBox.mouseDragEvent(self.view.plotItem.vb, ev, axis)

    def select_node(self, node_id):
        modifiers = QtGui.QGuiApplication.keyboardModifiers()
        is_shift = (modifiers & Qt.KeyboardModifier.ShiftModifier)

        self.selected_node_id = node_id 
        
        if is_shift:
             self.selected_node_ids.add(str(node_id))
        else:
             self.selected_node_ids = {str(node_id)}
        
        if str(node_id) in self.node_ids:
            idx = self.node_ids.index(str(node_id))
            pos = self.node_positions[idx]
            self.selection_ring.setData(x=[pos[0]], y=[pos[1]])
            self.selection_ring.show()
        
        self.node_selected.emit(node_id)

    def clear_selection(self):
        """Clears the current node selection."""
        self.selected_node_id = None
        self.selected_node_ids = set()
        self.selection_ring.hide()
        # Trigger restyle to clear selected brushes
        self.update_styling(self.node_ids, self.active_node_id)

    def on_mouse_move(self, pos):
        if len(self.node_positions) == 0: return

        mouse_point = self.view.plotItem.vb.mapSceneToView(pos)
        mx, my = mouse_point.x(), mouse_point.y()
        
        p = self.view.pixelSize() 
        rx, ry = p.x() * 15, p.y() * 15 
        
        d_x = np.abs(self.node_positions[:, 0] - mx)
        d_y = np.abs(self.node_positions[:, 1] - my)
        
        mask = (d_x < rx) & (d_y < ry)
        indices = np.where(mask)[0]
        
        found_id = None
        if len(indices) > 0:
            candidates = self.node_positions[indices]
            d2 = (candidates[:, 0] - mx)**2 + (candidates[:, 1] - my)**2
            best_idx_in_subset = np.argmin(d2)
            best_Global_idx = indices[best_idx_in_subset]
            
            if best_Global_idx < len(self.node_ids):
                found_id = str(self.node_ids[best_Global_idx])
                found_pos = self.node_positions[best_Global_idx]
                
                self.hover_ring.setData(
                    x=[found_pos[0]], 
                    y=[found_pos[1]],
                    size=25, 
                    pen=pg.mkPen('y', width=3),
                    brush=None
                )
                self.hover_ring.show()
        else:
            self.hover_ring.hide()
            
        self.hovered_node_id = found_id
