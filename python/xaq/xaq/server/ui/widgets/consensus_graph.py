from PyQt6.QtWidgets import QWidget, QVBoxLayout
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

class ConsensusGraphWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('k')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        layout.addWidget(self.plot_widget)
        
        # Items
        self.graph_item = pg.GraphItem()
        self.plot_widget.addItem(self.graph_item)
        
        # Initial State
        self.sources = []
        self.positions = []
        self.adjacencies = []
        
        # Central Hub (Consensus)
        # We will visualize:
        #  - A central node (Consensus)
        #  - Satellite nodes (Sources)
        #  - Edges thickness = Resonance/Trust

    def update_graph(self, consensus_data):
        """
        Update the visualization based on consensus data.
        consensus_data: {
          'consensus': ConsensusToken,
          'candidates': [RealityToken, ...],
          'resonance': float
        }
        """
        if not consensus_data:
            self.graph_item.setData(pos=[], adj=[], pen=[], size=[], symbol=[])
            return

        candidates = consensus_data.get('candidates', [])
        consensus_token = consensus_data.get('consensus')
        
        if not candidates:
             self.graph_item.setData(pos=[], adj=[], pen=[], size=[], symbol=[])
             return

        # Layout: Star topology
        # Center = Consensus (0,0)
        # Satellites = Candidates (Cos/Sin)
        
        import math
        
        n_candidates = len(candidates)
        radius = 5.0
        
        pos = [[0, 0]] # Center
        symbols = ['star']
        
        # Resonance logic for the "Glow"
        res = consensus_token.resonance_score if consensus_token else 0.0
        # Color: White -> Yellow based on resonance
        # White = (255, 255, 255), Yellow = (255, 255, 0)
        center_color = (255, 255, int(255 * (1.0 - res)), 255)
        brushes = [center_color]
        
        # Size: Pulse or just scale based on resonance
        center_size = 30 + (res * 30) # Scale up to 60px
        sizes = [center_size]
        
        adj = []
        pens = []
        
        # Add Candidates
        for i, token in enumerate(candidates):
            angle = (2 * math.pi * i) / n_candidates
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            pos.append([x, y])
            
            # Style based on Dopamine/Serotonin
            dopa = token.dopamine_level
            sero = token.serotonin_level
            
            r = int(255 * (1.0 - sero))
            g = int(255 * dopa)
            b = int(255 * sero)
            
            brushes.append((r, g, b, 255))
            symbols.append('o')
            sizes.append(20)
            
            # Edge to Center
            adj.append([0, i + 1])
            
            # Thickness/Alpha based on contribution
            width = 1.0 + (dopa * 5.0)
            alpha = int(50 + (sero * 200))
            # pens.append(pg.mkPen(color=(r, g, b, alpha), width=width))
            
        # Convert data to numpy arrays
        pos_np = np.array(pos)
        adj_np = np.array(adj, dtype=int)
        
        # PyqtGraph GraphItem fails with array of QPens (AttributeError .dtype)
        # We use a single pen for now to stabilize visualization
        single_pen = pg.mkPen((255, 255, 255, 100), width=2)
        sizes_np = np.array(sizes)
        
        self.graph_item.setData(
            pos=pos_np,
            adj=adj_np,
            pen=single_pen,
            size=sizes_np,
            symbol=symbols,
            brush=brushes,
            pxMode=True
        )

import numpy as np
