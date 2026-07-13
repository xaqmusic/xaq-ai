import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame
import numpy as np

class ActiveInferenceVizWidget(QWidget):
    """
    Visualizes the internal dynamics of Active Inference:
    - Real consensus embedding (projected)
    - Hallucinated future path
    - Target goal
    """
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.lbl_title = QLabel("<b>Active Inference Dynamics</b> (Latent Projection)")
        layout.addWidget(self.lbl_title)
        
        # Plotting area
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#121212')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setAspectLocked(True) # Spatial projections should be square
        
        # Labels
        label_style = {'color': '#AAAAAA', 'font-size': '8pt'}
        self.plot_widget.setLabel('left', 'Spatial Proxy (Y)', **label_style)
        self.plot_widget.setLabel('bottom', 'Spatial Proxy (X)', **label_style)
        
        # Current State (Real) - Yellow point
        self.real_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 0, 200))
        self.plot_widget.addItem(self.real_scatter)
        
        # Hallucinated Path - Green dotted line
        self.path_curve = self.plot_widget.plot(pen=pg.mkPen(color='#00FF00', width=1, style=pg.Qt.QtCore.Qt.PenStyle.DashLine))
        self.path_points = pg.ScatterPlotItem(size=5, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 100))
        self.plot_widget.addItem(self.path_points)
        
        # Target Point - Red Crosshair/Circle
        self.target_scatter = pg.ScatterPlotItem(size=15, symbol='+', pen=pg.mkPen('#FF0000', width=2))
        self.plot_widget.addItem(self.target_scatter)
        
        layout.addWidget(self.plot_widget)
        
        # Metrics Frame
        self.metrics_lbl = QLabel("Calibrating Latent Space...")
        layout.addWidget(self.metrics_lbl)

    def update_viz(self, harness, graph_data):
        """
        Update the visualization from the harness state.
        """
        if not harness or not harness.meta_epm:
            return
            
        consensus = graph_data.get('consensus')
        if not consensus:
            return
            
        # 1. Projection Logic
        # If calibrated, use spatial_axis as X. For Y, let's use the next highest variance dim or index 1.
        idx_x = harness.spatial_axis if harness.spatial_axis is not None else 0
        idx_y = (idx_x + 1) % 768 # Simple fallback
        
        z_real = consensus.fused_embedding.flatten()
        pos_real = (z_real[idx_x], z_real[idx_y])
        self.real_scatter.setData(pos=[pos_real])
        
        # 2. Hallucination Path
        # Use the path already computed by on_consensus() — avoids a second GRU rollout per tick
        future_path = getattr(harness, 'last_future_path', None)
        if future_path:
            pts = []
            for z_f in future_path:
                zf_flat = z_f.flatten()
                pts.append((zf_flat[idx_x], zf_flat[idx_y]))
            
            self.path_curve.setData([p[0] for p in pts], [p[1] for p in pts])
            self.path_points.setData(pos=pts)
            
            # Target is the end of the path
            self.target_scatter.setData(pos=[pts[-1]])
        else:
            self.path_curve.clear()
            self.path_points.clear()
            self.target_scatter.clear()

        # 3. Update Metrics Label
        if harness.spatial_axis is not None:
            self.metrics_lbl.setText(
                f"<b>Spatial Axis:</b> {harness.spatial_axis} | "
                f"<b>TLE:</b> {graph_data.get('tle', 0.0):.4f} | "
                f"<b>Action:</b> {getattr(harness, 'last_velocity', 0.0):.2f}"
            )
        else:
            self.metrics_lbl.setText(f"Calibrating... ({len(harness.calibration_buffer)}/{harness.max_calibration_samples})")
