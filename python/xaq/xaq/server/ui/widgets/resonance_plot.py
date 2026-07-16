import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout
import numpy as np
import time

# Color-coded dock title for the parent DockWidget.
# Built as HTML so each metric name matches its line color.
SERIES_LEGEND = [
    ("#FFFF00", "Resonance"),
    ("#00FF00", "Meta-TLE"),
    ("#FF6600", "Reflex%"),
    ("#00CFFF", "Consistency"),
    ("#FF00FF", "Heb.Res"),
]
DOCK_TITLE_HTML = "  ".join(
    f'<span style="color:{c}; font-weight:bold;">{name}</span>'
    for c, name in SERIES_LEGEND
)


class ResonancePlotWidget(QWidget):
    # Expose the HTML title so the parent dock can use it
    dock_title_html = DOCK_TITLE_HTML

    def __init__(self, window_size=200):
        super().__init__()
        self.window_size = window_size
        self.data = np.zeros(self.window_size)
        self.tle_data = np.zeros(self.window_size)
        self.reflex_data = np.zeros(self.window_size)
        self.consistency_data = np.zeros(self.window_size)
        self.heb_resonance_data = np.zeros(self.window_size)
        self.ptr = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#121212')
        # self.plot_widget.setYRange(0, 1) # Removed to allow auto-scaling
        self.plot_widget.enableAutoRange(axis='y', enable=True)

        # Small labels
        label_style = {'color': '#AAAAAA', 'font-size': '8pt'}
        self.plot_widget.setLabel('left', 'Resonance', **label_style)
        self.plot_widget.setLabel('bottom', 'Samples', **label_style)

        # Small ticks
        for axis in ['left', 'bottom']:
            self.plot_widget.getAxis(axis).setTickFont(pg.Qt.QtGui.QFont('Arial', 7))

        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)

        self.curve = self.plot_widget.plot(pen=pg.mkPen(color='#FFFF00', width=2), name="Resonance")
        self.tle_curve = self.plot_widget.plot(pen=pg.mkPen(color='#00FF00', width=1.5), name="Meta-TLE")
        self.reflex_curve = self.plot_widget.plot(pen=pg.mkPen(color='#FF6600', width=1.5), name="Reflex %")
        self.consistency_curve = self.plot_widget.plot(pen=pg.mkPen(color='#00CFFF', width=1.5), name="Consistency")
        self.heb_resonance_curve = self.plot_widget.plot(pen=pg.mkPen(color='#FF00FF', width=1.5), name="Heb. Resonance")

        # Threshold line
        self.threshold_line = pg.InfiniteLine(pos=0.6, angle=0, pen=pg.mkPen(color='#FF0000', width=1, style=pg.Qt.QtCore.Qt.PenStyle.DashLine))
        self.plot_widget.addItem(self.threshold_line)

        layout.addWidget(self.plot_widget)
        
    def update_resonance(self, resonance, threshold=None, tle=None,
                         reflex_inf=None, consistency=None,
                         heb_resonance=None):
        self.data[:-1] = self.data[1:]
        self.data[-1] = resonance
        self.curve.setData(self.data)

        if tle is not None:
            self.tle_data[:-1] = self.tle_data[1:]
            # Use raw TLE value
            self.tle_data[-1] = tle
            self.tle_curve.setData(self.tle_data)

        if reflex_inf is not None:
            self.reflex_data[:-1] = self.reflex_data[1:]
            self.reflex_data[-1] = reflex_inf
            self.reflex_curve.setData(self.reflex_data)

        if consistency is not None:
            self.consistency_data[:-1] = self.consistency_data[1:]
            self.consistency_data[-1] = consistency
            self.consistency_curve.setData(self.consistency_data)

        if heb_resonance is not None:
            self.heb_resonance_data[:-1] = self.heb_resonance_data[1:]
            # Scale to comparable range — resonance_score can be > 1
            self.heb_resonance_data[-1] = heb_resonance / 2.0
            self.heb_resonance_curve.setData(self.heb_resonance_data)

        if threshold is not None:
            self.threshold_line.setValue(threshold)
