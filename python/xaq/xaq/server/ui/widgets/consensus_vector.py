import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
import numpy as np

class ConsensusVectorWidget(QWidget):
    """
    Visualizes the 768-dim consensus embedding as a real-time heatmap/scrolling strip.
    """
    def __init__(self, dim=768):
        super().__init__()
        self.dim = dim
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.lbl_title = QLabel("<b>Consensus Token Embedding</b> (768-dim Vector)")
        layout.addWidget(self.lbl_title)
        
        # We use an ImageItem to show the vector as a scrolling 2D heatmap 
        # (Time on Y, Dimension on X)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#000000')
        self.plot_widget.setLabel('left', 'History (Frames)')
        self.plot_widget.setLabel('bottom', 'Latent Dimension')
        
        self.img_item = pg.ImageItem()
        self.plot_widget.addItem(self.img_item)
        
        # Color Map (Sleek Dark Theme)
        colormap = pg.colormap.get('viridis')
        self.img_item.setLookupTable(colormap.getLookupTable())
        
        # Data Buffer: (Time, Dim)
        self.history_len = 50
        self.data_buffer = np.zeros((self.history_len, self.dim))
        
        layout.addWidget(self.plot_widget)

    def update_vector(self, embedding):
        """Update the heatmap with the latest vector."""
        if embedding is None:
            return
            
        vector = embedding.flatten()
        if vector.shape[0] != self.dim:
            vector = np.resize(vector, self.dim)
            
        # Normalization/Gain for visibility
        v_max = np.max(np.abs(vector))
        if v_max > 1e-6:
            vector = vector / v_max # Auto-normalize to 1.0 for the heatmap
            
        # Shift buffer
        self.data_buffer[:-1] = self.data_buffer[1:]
        self.data_buffer[-1] = vector
        
        # Update image
        self.img_item.setImage(self.data_buffer.T) # Transpose for (Dim, Time) display
        self.img_item.setRect([0, 0, self.dim, self.history_len])
