import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout
import numpy as np

class MatrixViewWidget(QWidget):
    def __init__(self, max_nodes=50):
        super().__init__()
        self.max_nodes = max_nodes
        self.node_to_idx = {}
        self.idx_to_node = {}
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Use GraphicsLayoutWidget for precise layout control
        self.win = pg.GraphicsLayoutWidget()
        self.win.setBackground('#121212')
        layout.addWidget(self.win)
        
        # Label at the top
        self.label = pg.LabelItem("Association Matrix", color="#AAAAAA", size="10pt")
        self.win.addItem(self.label, row=0, col=0, colspan=2)
        
        # Heatmap ViewBox
        self.view = self.win.addViewBox(row=1, col=0)
        # self.view.setAspectLocked(True) # Remove to allow it to fill window better
        self.view.invertY(True)
        
        # Crosshair State
        self.crosshair_x = 0.0
        self.crosshair_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_idA = None
        self.target_idB = None
        
        # Image item for heatmap (added first to render behind crosshairs)
        self.image = pg.ImageItem()
        self.view.addItem(self.image)
        
        # Crosshair Lines (White, 50% opacity)
        pen = pg.mkPen((255, 255, 255, 128), width=1)
        self.v_line = pg.InfiniteLine(angle=90, movable=False, pen=pen)
        self.h_line = pg.InfiniteLine(angle=0, movable=False, pen=pen)
        self.v_line.setZValue(10) # ensure on top
        self.h_line.setZValue(10)
        self.view.addItem(self.v_line)
        self.view.addItem(self.h_line)
        
        # Color map and Bar
        colormap = pg.colormap.get('viridis')
        self.bar = pg.ColorBarItem(
            values=(0, 1), 
            colorMap=colormap, 
            label="Weight", 
            interactive=False
        )
        self.bar.axis.setTickFont(pg.Qt.QtGui.QFont('Arial', 7))
        self.bar.axis.setPen('#AAAAAA')
        self.bar.axis.setTextPen('#AAAAAA')
        
        # Fix color bar width so it doesn't overwhelm the graph, but numbers remain visible
        self.bar.setFixedWidth(70)
        
        self.win.addItem(self.bar, row=1, col=1)
        self.bar.setImageItem(self.image)
        
        self.matrix = np.zeros((self.max_nodes, self.max_nodes))

    def update_crosshair_target(self, idA, idB):
        """
        Bind the mathematical target node IDs of the most recent consensus.
        The physical rendering matches target_x and target_y inside update_matrix
        because the ephemeral node-to-index mapping resets continuously.
        """
        # Note: pg.ImageItem renders the X axis from the first dimension of the numpy array.
        # Since we pass self.matrix.T, the X axis corresponds to idxB (the max ID target),
        # and the Y axis corresponds to idxA (the min ID target).
        self.target_idA = max(idA, idB) # X-axis target maps to max_id
        self.target_idB = min(idA, idB) # Y-axis target maps to min_id

    def update_matrix(self, association_matrix, nav_speed=0.1, node_count=None):
        """
        Update heatmap from the sparse dictionary.
        """
        if node_count is not None:
            self.label.setText(f"Association Matrix | Active Nodes: {node_count}")
        
        if not association_matrix:
            return
            
        # Collect node connection strengths
        from collections import defaultdict
        node_weights = defaultdict(float)
        for nA, targets in association_matrix.items():
            for nB, weight in targets.items():
                node_weights[nA] += weight
                node_weights[nB] += weight
                
        if not node_weights:
            self.matrix.fill(0)
            self.image.setImage(self.matrix.T, autoLevels=False)
            return

        # Sort by weight to get the top active max_nodes
        top_nodes = sorted(node_weights.items(), key=lambda x: x[1], reverse=True)[:self.max_nodes]
        nodes = sorted([n for n, w in top_nodes])
        
        # Build ephemeral mapping so matrix always tracks the hottest current nodes
        self.node_to_idx = {n: i for i, n in enumerate(nodes)}
        self.idx_to_node = {i: n for i, n in enumerate(nodes)}
        
        # Ensure matrix matches dynamic size if under max_nodes
        curr_size = len(nodes)
        if self.matrix.shape[0] != curr_size:
            self.matrix = np.zeros((curr_size, curr_size))
        else:
            self.matrix.fill(0)
        
        # Fill matrix from sparse dict
        # Initialize max_val infinitesimally small so it mathematically expands to any actual weight encountered
        max_val = 1e-6
        for nodeA, targets in association_matrix.items():
            if nodeA in self.node_to_idx:
                idxA = self.node_to_idx[nodeA]
                for nodeB, weight in targets.items():
                    if nodeB in self.node_to_idx:
                        idxB = self.node_to_idx[nodeB]
                        # Mirror the mathematical upper-triangular connection so it renders entirely symmetrical in UI
                        self.matrix[idxA, idxB] = weight
                        self.matrix[idxB, idxA] = weight
                        if weight > max_val:
                            max_val = weight
        
        # Update image and scale levels so even small changes are visible.
        # Fixed range floor of 1.0 ensures that small decayed weights appear dim.
        viz_max = max(1.0, max_val)
        self.image.setImage(self.matrix.T, autoLevels=False)
        self.image.setLevels([0, viz_max]) 
        self.bar.setLevels(low=0, high=viz_max)
        
        # Auto-scale the view to fit the data
        # Zoom in on the active portion of the matrix for better visibility
        if len(self.node_to_idx) > 0:
             # Show at least a 4x4 grid so it's not too "jumpy" but still zoomed
             grid_size = max(4, len(self.node_to_idx))
             self.view.setRange(xRange=(0, grid_size), yRange=(0, grid_size), padding=0.1)
             
        # Resolve mathematical target mapping
        if self.target_idA in self.node_to_idx and self.target_idB in self.node_to_idx:
            # Map physical pixel center (+0.5) to the node's current matrix grid index
            self.target_x = self.node_to_idx[self.target_idA] + 0.5
            self.target_y = self.node_to_idx[self.target_idB] + 0.5
            
        # Interpolate Crosshairs towards dynamic targets
        self.crosshair_x += (self.target_x - self.crosshair_x) * nav_speed
        self.crosshair_y += (self.target_y - self.crosshair_y) * nav_speed
        self.v_line.setPos(self.crosshair_x)
        self.h_line.setPos(self.crosshair_y)
             
        # Debug: Print if we have significant associations appearing
        if max_val > 0.11: # 0.1 is our base max_val
             pass # print(f"DEBUG: Association Matrix Updated. Max Weight: {max_val:.2f}, Active Nodes: {len(self.node_to_idx)}")
        
        # Update axis labels could be added here for better debug
