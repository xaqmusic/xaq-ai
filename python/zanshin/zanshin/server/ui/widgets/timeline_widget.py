from PyQt6.QtWidgets import QWidget, QVBoxLayout, QScrollArea, QFrame, QSizePolicy
from PyQt6.QtCore import Qt, QTimer, QPoint
from PyQt6.QtGui import QPainter, QPen, QColor
import time
from .track_widget import TrackWidget

class TimelineWidget(QWidget):
    def __init__(self, voter_node):
        super().__init__()
        self.voter = voter_node
        self.tracks = {} # agent_id -> TrackWidget
        
        # Main Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Scroll Area for Tracks
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setStyleSheet("background-color: #121212;")
        
        # Container inside Scroll
        self.container = QWidget()
        self.track_layout = QVBoxLayout(self.container)
        self.track_layout.setContentsMargins(0, 0, 0, 0)
        self.track_layout.setSpacing(1) # Gap between tracks
        self.track_layout.addStretch() # Push tracks up
        
        self.scroll.setWidget(self.container)
        
        layout.addWidget(self.scroll)
        
        # Overlay for Zipper Lines
        self.zipper_overlay = ZipperOverlay(self.container, self.voter)
        self.container.installEventFilter(self.zipper_overlay)
        
        # Fix 1: Single consolidated paint timer for ALL children (30fps)
        # Replaces individual 60Hz timers on each TrackCanvas.
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._repaint_tick)
        self.refresh_timer.start(33)  # ~30fps
    
    def _repaint_tick(self):
        """Lightweight repaint tick — no data recomputation, just redraws."""
        for track in self.tracks.values():
            track.timeline.update()
        self.zipper_overlay.update()

    def update_timeline(self, graph_data):
        if not graph_data: return
        
        candidates = graph_data.get('candidates', [])
        now = time.time()
        
        # Calculate interesting nodes once per update for ghosting and filtering
        interesting_nodes = set(self.voter.association_matrix.keys())
        for sub_dict in self.voter.association_matrix.values():
            interesting_nodes.update(sub_dict.keys())
            
        # Add nodes from recent consensus
        recent_window = 10.0
        recent_consensus = [c for c in self.voter.consensus_history if c.timestamp > now - recent_window]
        for c in recent_consensus:
            if c.contributing_ids:
                interesting_nodes.update(c.contributing_ids)
        
        # 1. Ensure Track exists
        changed = False
        for token in candidates:
            aid = token.source_id
            
            if aid not in self.tracks:
                # Add new track
                count = self.track_layout.count()
                track = TrackWidget(aid)
                self.track_layout.insertWidget(count - 1, track)
                self.tracks[aid] = track
                changed = True
                
            # 2. Update Track State
            self.tracks[aid].update_state(token)
            self.tracks[aid].timeline.set_active_nodes(interesting_nodes)
            
        if changed:
            # Trigger overlay resize/update
            self.zipper_overlay.lower()
            self.zipper_overlay.invalidate_cache()
            
        # 3. Update Overlay (Zipper)
        self.zipper_overlay.process_data_update(graph_data, interesting_nodes)
        self.zipper_overlay.update()

class ZipperOverlay(QWidget):
    def __init__(self, parent, voter):
        super().__init__(parent)
        self.voter = voter
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground) # Transparent
        self.move(0, 0)
        self.zipper_lines = [] 
        self.track_cache = {} # TrackWidget -> {'y1': int, 'y2': int}
        
        self.lookback_window = 0.5
        
    def invalidate_cache(self):
        self.track_cache.clear()

    def eventFilter(self, obj, event):
        if obj == self.parent() and (event.type() == event.Type.Resize or event.type() == event.Type.Move):
            self.resize(obj.size())
            self.move(0, 0)
            self.invalidate_cache()
            self.raise_()
        return False

    def process_data_update(self, consensus_data, interesting_nodes=None):
        """
        Offload correlation logic from paintEvent to data-driven update.
        Pre-calculates lines based on the latest consensus and matrix state.
        """
        start_time = time.perf_counter()
        parent_widget = self.parent()
        tracks = parent_widget.findChildren(TrackWidget)
        if len(tracks) < 2: return

        # Update Track Cache (Only when resized/moved/changed)
        if not self.track_cache:
            for t in tracks:
                canvas = t.timeline
                p1_y = canvas.mapTo(parent_widget, QPoint(0, int(canvas.height() * 0.75))).y()
                p2_y = canvas.mapTo(parent_widget, QPoint(0, int(canvas.height() * 0.25))).y()
                self.track_cache[t] = (p1_y, p2_y)

        now = time.time()
        
        # 1. Connectivity Filtering
        # Only nodes with matrix connections or recent consensus are worth matching.
        if interesting_nodes is None:
            # Fallback if called directly
            interesting_nodes = set(self.voter.association_matrix.keys())
            for sub_dict in self.voter.association_matrix.values():
                interesting_nodes.update(sub_dict.keys())
            
            # Add nodes from recent consensus
            recent_consensus = [c for c in self.voter.consensus_history if c.timestamp > now - self.lookback_window]
            for c in recent_consensus:
                if c.contributing_ids:
                    interesting_nodes.update(c.contributing_ids)
        else:
            # If interesting_nodes is provided, recent_consensus is still needed for resonance
            recent_consensus = [c for c in self.voter.consensus_history if c.timestamp > now - self.lookback_window]
        
        # 2. Fixed Lookback Scaling
        # To simplify display, we only show connections made during the past 10 tokens (~0.5s)
        self.lookback_window = 0.5

        new_zippers = []
        
        # Iterate pairs of tracks
        for i in range(len(tracks)):
            for j in range(i + 1, len(tracks)):
                t1, t2 = tracks[i], tracks[j]
                canvas1, canvas2 = t1.timeline, t2.timeline
                
                # Filter events by connectivity to reduce O(N^2) load
                # Also apply sampling if still too many
                events1 = [ev for ev in list(canvas1.events) if ev['start'] > now - self.lookback_window and (ev['id'] in interesting_nodes or ev['id'] == -1)]
                if len(events1) > 100: events1 = events1[::2]
                
                events2 = [ev for ev in list(canvas2.events) if ev['start'] > now - self.lookback_window and (ev['id'] in interesting_nodes or ev['id'] == -1)]
                if len(events2) > 100: events2 = events2[::2]
                
                if not events1 or not events2: continue

                idx2_start = 0
                for ev1 in events1:
                    start1 = ev1['start']
                    id1 = ev1['id']
                    
                    # Advance idx2_start
                    while idx2_start < len(events2) and events2[idx2_start]['start'] < start1 - 0.5:
                        idx2_start += 1
                        
                    for k in range(idx2_start, len(events2)):
                        ev2 = events2[k]
                        start2 = ev2['start']
                        if start2 > start1 + 0.5: break
                        id2 = ev2['id']
                        
                        # Weight from matrix (Fast lookup)
                        weight = self.voter.association_matrix[min(id1, id2)][max(id1, id2)]
                        
                        # Heuristic: Skip weak uninteresting lines
                        if weight < 0.05:
                            # Still check for resonance (recent consensus)
                            resonance = 0.0
                            for c in recent_consensus:
                                if abs(start1 - c.timestamp) < 0.4:
                                    resonance = c.resonance_score
                                    break
                            if resonance < self.voter.threshold:
                                continue
                        else:
                            resonance = 0.0 
                        
                        new_zippers.append({
                            't1_y': self.track_cache[t1][0],
                            't2_y': self.track_cache[t2][1],
                            'mid_time': (start1 + start2) / 2,
                            'resonance': resonance,
                            'weight': weight
                        })
        
        self.zipper_lines = new_zippers[-200:]  # Fix 2: Cap to 200 most recent lines
        self.last_proc_time = time.perf_counter() - start_time

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.lower()
        
        now = time.time()
        parent_widget = self.parent()
        tracks = parent_widget.findChildren(TrackWidget)
        if not tracks or not self.zipper_lines: return
        
        # Assume all canvases have the same scaling
        canvas = tracks[0].timeline 
        w = canvas.width()
        window_seconds = canvas.window_seconds
        pixels_per_sec = w / window_seconds
        canvas_x_offset = canvas.mapTo(parent_widget, canvas.rect().topLeft()).x()

        for zip_line in self.zipper_lines:
            # Map time to X
            x_local = w - (now - zip_line['mid_time']) * pixels_per_sec
            x_final = canvas_x_offset + x_local
            
            # Clip
            if x_final < canvas_x_offset or x_final > canvas_x_offset + w:
                continue
                
            # Visual coding
            res = zip_line['resonance']
            weight = zip_line['weight']
            
            if res > self.voter.threshold:
                pen = QPen(QColor(255, 255, 0, 255), 3, Qt.PenStyle.SolidLine)
            elif weight > 0.01:
                pen = QPen(QColor(255, 255, 255, 200), 2, Qt.PenStyle.SolidLine)
            else:
                pen = QPen(QColor(0, 255, 255, 100), 1, Qt.PenStyle.DashLine)
            
            painter.setPen(pen)
            painter.drawLine(int(x_final), zip_line['t1_y'], int(x_final), zip_line['t2_y'])
