from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtGui import QPainter, QColor, QBrush, QPen
from PyQt6.QtCore import Qt, QRectF, QTimer
import time
import random
import collections

class TrackWidget(QWidget):
    def __init__(self, agent_id, parent=None):
        super().__init__(parent)
        self.agent_id = agent_id
        self.setFixedHeight(80) # Fixed height track
        
        # Layout
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Left Panel (Info + Meters)
        self.info_panel = QFrame()
        self.info_panel.setFixedWidth(250) # Expanded slightly to fit horizontal text
        self.info_panel.setStyleSheet("background-color: #2a2a2a; border-right: 1px solid #444;")
        
        info_layout = QVBoxLayout(self.info_panel)
        info_layout.setContentsMargins(5, 5, 5, 5)
        
        # Name
        self.lbl_name = QLabel(agent_id)
        self.lbl_name.setStyleSheet("color: white; font-weight: bold;")
        info_layout.addWidget(self.lbl_name)
        
        # Meters & Layout Container
        meter_layout = QHBoxLayout()
        
        # Dopamine
        self.meter_dopa = QFrame()
        self.meter_dopa.setFixedSize(10, 40)
        self.meter_dopa.setStyleSheet("background-color: #333; border: 1px solid #555;")
        self.fill_dopa = QFrame(self.meter_dopa)
        self.fill_dopa.setStyleSheet("background-color: gold;")
        self.fill_dopa.setGeometry(0, 40, 10, 0) # Start empty
        
        # Serotonin
        self.meter_sero = QFrame()
        self.meter_sero.setFixedSize(10, 40)
        self.meter_sero.setStyleSheet("background-color: #333; border: 1px solid #555;")
        self.fill_sero = QFrame(self.meter_sero)
        self.fill_sero.setStyleSheet("background-color: #0088ff;")
        self.fill_sero.setGeometry(0, 40, 10, 0)
        
        meter_layout.addWidget(self.meter_dopa)
        meter_layout.addWidget(self.meter_sero)
        
        # Active text layout (Horizontal to Meters)
        text_layout = QVBoxLayout()
        self.lbl_node = QLabel("Node: --")
        self.lbl_node.setStyleSheet("color: #aaaaaa; font-size: 10px;")
        text_layout.addWidget(self.lbl_node)
        
        self.lbl_text = QLabel("")
        self.lbl_text.setStyleSheet("color: #00ffff; font-size: 10px;")
        self.lbl_text.setWordWrap(True)
        text_layout.addWidget(self.lbl_text)
        text_layout.addStretch()
        
        meter_layout.addLayout(text_layout)
        meter_layout.addStretch()
        
        info_layout.addLayout(meter_layout)
        info_layout.addStretch()
        
        layout.addWidget(self.info_panel)
        
        # Right Panel (Timeline Canvas)
        self.timeline = TrackCanvas()
        layout.addWidget(self.timeline)

    def update_state(self, token):
        # Update Telemetry Stats
        self.lbl_node.setText(f"Node: {token.active_node_id}")
        self.lbl_text.setText(token.text_label if token.text_label else "")
        
        # Update Meters
        h = 40
        
        d = token.dopamine_level
        dh = int(d * h)
        self.fill_dopa.setGeometry(0, h - dh, 10, dh)
        
        s = token.serotonin_level
        sh = int(s * h)
        self.fill_sero.setGeometry(0, h - sh, 10, sh)
        
        # Push to Timeline
        self.timeline.add_event(token)

class TrackCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("background-color: transparent;")
        self.events = collections.deque()
        self.window_seconds = 10.0
        
        self.token_buffer = []

        # NOTE: No self-owned repaint timer. The parent TimelineWidget
        # drives all repaints via a single consolidated timer.
        
        # Heuristic for colors per node ID
        self.color_map = {}
        self.active_nodes = set()

    def get_color(self, node_id):
        if node_id not in self.color_map:
            # Generate random pastel color
            hue = random.random()
            c = QColor.fromHsvF(hue, 0.7, 0.9)
            self.color_map[node_id] = c
        return self.color_map[node_id]

    def add_event(self, token):
        self.token_buffer.append(token)
        
        if len(self.token_buffer) >= 10:
            nodes = [t.active_node_id for t in self.token_buffer]
            try:
                dominant_id = collections.Counter(nodes).most_common(1)[0][0]
            except Exception:
                dominant_id = -1
                
            start_time = self.token_buffer[0].timestamp
            end_time = self.token_buffer[-1].timestamp + 0.04
            
            c = self.get_color(dominant_id)
            
            # Consolidate continuous identical blocks
            if self.events and self.events[-1]['id'] == dominant_id and (start_time - self.events[-1]['end'] < 0.2):
                self.events[-1]['end'] = end_time
            else:
                self.events.append({
                    'start': start_time,
                    'end': end_time,
                    'id': dominant_id,
                    'color': c
                })
                
            self.token_buffer.clear()
        
        # Prune old
        now = time.time()
        cutoff = now - self.window_seconds
        
        # Prune from front
        while self.events and self.events[0]['end'] < cutoff:
            self.events.popleft()

    def set_active_nodes(self, node_ids):
        """Update the set of nodes that have active associations."""
        self.active_nodes = node_ids

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        w = self.width()
        h = self.height()
        now = time.time()
        
        pixels_per_sec = w / self.window_seconds
        bar_height = h / 2
        y_offset = h / 4
        
        for ev in self.events:
            t_start = ev['start']
            t_end = ev['end']
            node_id = ev['id']
            
            x_start = w - (now - t_start) * pixels_per_sec
            x_end = w - (now - t_end) * pixels_per_sec
            
            # Fix 3: Early skip for off-screen events
            if x_end < 0:
                continue
            if x_start > w:
                continue
            
            is_active = node_id in self.active_nodes or node_id == -1
            width = max(2, x_end - x_start)
            rect = QRectF(x_start, y_offset, width, bar_height)
            
            painter.setPen(Qt.PenStyle.NoPen)
            color = QColor(ev['color'])
            color.setAlpha(255 if is_active else 60)
            painter.setBrush(QBrush(color))
            painter.drawRoundedRect(rect, 4, 4)
            
            if is_active and width > 20:
                painter.setPen(QColor("black"))
                painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, str(node_id))
