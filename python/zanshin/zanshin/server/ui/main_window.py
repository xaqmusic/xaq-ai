import asyncio
import sys
import time
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QDockWidget, QMenuBar, QMenu, QDialog, QTextEdit, QProgressBar, QPushButton, QFileDialog
from PyQt6.QtCore import Qt, QTimer, QSettings, QProcess, QThread, pyqtSignal, QObject
from PyQt6.QtGui import QAction
from ..server import BrainSocketServer
from ..lateral_voter import LateralVoterNode

class ConsensusWorker(QObject):
    finished = pyqtSignal()
    consensus_ready = pyqtSignal(dict)

    def __init__(self, voter):
        super().__init__()
        self.voter = voter
        self.running = True

    def run(self):
        while self.running:
            try:
                # 1. Pump the Voter Loop
                consensus = self.voter.process_consensus()
                
                if consensus:
                    # Gather candidate objects for viz
                    # We avoid deep copying for speed, but we should be careful with shared state
                    graph_data = {
                        'consensus': consensus,
                        'candidates': [b[-1] for b in self.voter.input_buffers.values() if b],
                        'resonance': consensus.resonance_score
                    }
                    self.consensus_ready.emit(graph_data)
                
                # Run at ~25Hz (40ms)
                QThread.msleep(40)
            except Exception as e:
                print(f"Error in consensus worker: {e}")
                QThread.msleep(100)

    def stop(self):
        self.running = False
from .widgets.epm_list_widget import EPMListWidget
from .widgets.timeline_widget import TimelineWidget
from .widgets.consensus_graph import ConsensusGraphWidget
from .widgets.resonance_plot import ResonancePlotWidget
from .widgets.matrix_view import MatrixViewWidget
from .widgets.voting_controls import VotingControlsWidget
from .widgets.consensus_log import ConsensusLogWidget
from .widgets.active_inference_widget import ActiveInferenceWidget
from .widgets.active_inference_viz import ActiveInferenceVizWidget
from .widgets.consensus_vector import ConsensusVectorWidget
from src.native.widgets.remote_control_window import RemoteControlWindow
from .settings_window import SettingsWindow
import os
import tempfile
import json

class ExportBrainDialog(QDialog):
    def __init__(self, parent=None, script_path=None, matrix_data=None, config_data=None):
        super().__init__(parent)
        self.setWindowTitle("Exporting Brain Server to C++")
        self.resize(600, 400)
        self.script_path = script_path
        self.matrix_data = matrix_data
        self.config_data = config_data
        
        layout = QVBoxLayout(self)
        
        self.text_edit = QTextEdit(self)
        self.text_edit.setReadOnly(True)
        self.text_edit.setStyleSheet("background-color: #1e1e1e; color: #00FF00; font-family: monospace;")
        layout.addWidget(self.text_edit)
        
        self.progress = QProgressBar(self)
        self.progress.setRange(0, 0)
        layout.addWidget(self.progress)
        
        self.btn_close = QPushButton("Close", self)
        self.btn_close.setEnabled(False)
        self.btn_close.clicked.connect(self.accept)
        layout.addWidget(self.btn_close)
        
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.on_ready_read_stdout)
        self.process.readyReadStandardError.connect(self.on_ready_read_stderr)
        self.process.finished.connect(self.on_finished)
        
    def start_export(self):
        # 1. Save data to a temporary file
        temp_dir = tempfile.gettempdir()
        data_path = os.path.join(temp_dir, "ami_ogma_brain_export.json")
        
        with open(data_path, "w") as f:
            json.dump({
                'matrix': self.matrix_data,
                'config': self.config_data
            }, f)
            
        self.text_edit.append(f"Prepared deployment payload at {data_path}\n")
        self.text_edit.append(f"Triggering C++ Brain Sync hook -> {self.script_path}\n")
        
        import sys
        args = [self.script_path, data_path]
        if hasattr(self, 'target_dir') and self.target_dir:
             args.append(self.target_dir)
             
        self.process.start(sys.executable, args)
        
    def on_ready_read_stdout(self):
        msg = self.process.readAllStandardOutput().data().decode()
        self.text_edit.append(msg.strip())
        
    def on_ready_read_stderr(self):
        msg = self.process.readAllStandardError().data().decode()
        self.text_edit.append(msg.strip())
        
    def on_finished(self, exit_code, exit_status):
        self.progress.setRange(0, 100)
        self.progress.setValue(100)
        self.btn_close.setEnabled(True)
        
        if exit_code == 0:
            self.text_edit.append("\n✅ Brain Server Export Completed Successfully!")
            if hasattr(self, 'target_dir') and self.target_dir:
                 self.text_edit.append(f"Native C++ artifacts are now available in '{self.target_dir}'.")
            else:
                 self.text_edit.append("Native C++ artifacts are now available in the project's 'cpp_core/exported' directory.")
        else:
            self.text_edit.append(f"\n❌ Export Process Failed with exit code {exit_code}.")


class BrainServerWindow(QMainWindow):
    def __init__(self, loop=None):
        super().__init__()
        self.setWindowTitle("Ogma Brain Server (Lateral Voting)")
        self.resize(1200, 800)
        
        # Core Logic
        self.voter = LateralVoterNode()
        self.voter.register_callback(self.on_voter_update)
        
        self.server = BrainSocketServer(self.voter)
        self.loop = loop or asyncio.get_event_loop()
        
        
        # Layout
        self.init_ui()
        self.create_menus()
        
        # Persistence
        self.settings = QSettings("AmiOgma", "BrainServer")
        self.restore_layout_state()
        
        # Start Server (Async)
        self.start_server_task()
        
        # Graph View Window State
        self.graph_window = None
        
        # UI Timer for constant UI refresh (sliding animations, etc)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui_frame)
        self.timer.start(33) # 30Hz UI refresh
        
        # 2. Consensus Background Thread (DISABLED - Now handled by BrainSocketServer unified loop)
        # self.consensus_thread = QThread()
        # self.consensus_worker = ConsensusWorker(self.voter)
        # self.consensus_worker.moveToThread(self.consensus_thread)
        # self.consensus_thread.started.connect(self.consensus_worker.run)
        # self.consensus_worker.consensus_ready.connect(self.on_consensus_received)
        # self.consensus_thread.start()
        
        # UI reads staged consensus data from server at the timer rate (Option A decoupling)
        
    def init_ui(self):
        # Center: Timeline
        self.timeline_widget = TimelineWidget(self.voter)
        self.setCentralWidget(self.timeline_widget)
        
        # Dock: Agent List
        self.dock_agents = QDockWidget("Connected EPMs", self)
        self.dock_agents.setObjectName("DockConnectedEPMs")
        self.list_widget = EPMListWidget()
        self.dock_agents.setWidget(self.list_widget)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.dock_agents)

        # Dock: Voting Controls
        self.dock_voting = QDockWidget("Voting Controls", self)
        self.dock_voting.setObjectName("DockVotingControls")
        self.voting_controls = VotingControlsWidget(self.voter)
        self.dock_voting.setWidget(self.voting_controls)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.dock_voting)

        # Dock: Resonance Plot
        self.dock_resonance = QDockWidget("Resonance History", self)
        self.dock_resonance.setObjectName("DockResonancePlot")
        self.resonance_plot = ResonancePlotWidget()
        self.dock_resonance.setWidget(self.resonance_plot)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.dock_resonance)

        # Dock: Matrix View
        self.dock_matrix = QDockWidget("Association Matrix", self)
        self.dock_matrix.setObjectName("DockMatrixView")
        self.matrix_view = MatrixViewWidget()
        self.dock_matrix.setWidget(self.matrix_view)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dock_matrix)
        
        # Dock: Consensus Log
        self.dock_log = QDockWidget("Consensus Log", self)
        self.dock_log.setObjectName("DockConsensusLog")
        self.consensus_log = ConsensusLogWidget()
        self.dock_log.setWidget(self.consensus_log)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dock_log)

        # Dock: Active Inference
        self.dock_active_inf = QDockWidget("Active Inference", self)
        self.dock_active_inf.setObjectName("DockActiveInference")
        
        self.ai_container = QWidget()
        self.ai_layout = QVBoxLayout(self.ai_container)
        
        self.active_inf_widget = ActiveInferenceWidget(server=self.server, harness=self.server.active_harness)
        self.active_inf_widget.mode_changed.connect(self.on_active_inf_toggle)
        self.ai_layout.addWidget(self.active_inf_widget)
        
        self.ai_viz = ActiveInferenceVizWidget()
        self.ai_layout.addWidget(self.ai_viz)
        
        self.dock_active_inf.setWidget(self.ai_container)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.dock_active_inf)
        
        # Dock: Vector Heatmap
        self.dock_vector = QDockWidget("Latent Vector History", self)
        self.dock_vector.setObjectName("DockVectorHeatmap")
        self.vector_widget = ConsensusVectorWidget()
        self.dock_vector.setWidget(self.vector_widget)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.dock_vector)
        
    def create_menus(self):
        menubar = self.menuBar()
        
        # File Menu
        file_menu = menubar.addMenu("File")
        
        act_quit = QAction("Quit", self)
        act_quit.setShortcut("Ctrl+Q")
        act_quit.triggered.connect(self.close)
        file_menu.addAction(act_quit)
        
        file_menu.addSeparator()

        act_export = QAction("Export to C++ (Native)...", self)
        act_export.triggered.connect(self.export_brain_to_cpp)
        file_menu.addAction(act_export)
        
        # Window Menu
        window_menu = menubar.addMenu("Window")
        
        act_graph = QAction("Show Graph View", self)
        act_graph.triggered.connect(self.show_graph_view)
        window_menu.addAction(act_graph)
        
        window_menu.addSeparator()
        
        act_settings = QAction("Settings...", self)
        act_settings.triggered.connect(self.show_settings)
        window_menu.addAction(act_settings)
        
        window_menu.addSeparator()
        
        # Remote Control
        act_remote = QAction("Remote Control (C++)...", self)
        act_remote.triggered.connect(self.show_remote_control)
        window_menu.addAction(act_remote)

        window_menu.addSeparator()

        # Toggle Docks
        window_menu.addAction(self.dock_agents.toggleViewAction())
        window_menu.addAction(self.dock_voting.toggleViewAction())
        window_menu.addAction(self.dock_resonance.toggleViewAction())
        window_menu.addAction(self.dock_matrix.toggleViewAction())
        window_menu.addAction(self.dock_log.toggleViewAction())
        window_menu.addAction(self.dock_active_inf.toggleViewAction())

        window_menu.addSeparator()
        
        act_reset = QAction("Reset Layout", self)
        act_reset.triggered.connect(self.reset_layout)
        window_menu.addAction(act_reset)

    def on_active_inf_toggle(self, active):
        """Handle manual toggle of active inference from the UI."""
        # We call the server's handler manually (wrapped in task)
        coro = self.server.on_set_active_inference(None, {'active': active})
        asyncio.ensure_future(coro, loop=self.loop)
        
    def show_settings(self):
        dlg = SettingsWindow(self.server, self)
        dlg.exec()
        
    def show_remote_control(self):
        """Open the Remote Control configuration dialog."""
        if not hasattr(self, 'remote_dialog'):
            self.remote_dialog = RemoteControlWindow(self)
        self.remote_dialog.show()
        self.remote_dialog.raise_()
        self.remote_dialog.activateWindow()

    def export_brain_to_cpp(self):
        """Trigger the sync_brain.py script with current live associations."""
        
        exports_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../exports'))
        os.makedirs(exports_dir, exist_ok=True)
        
        dialog = QFileDialog(self, "Select or Type New Brain Config Export Folder", exports_dir)
        dialog.setFileMode(QFileDialog.FileMode.Directory)
        dialog.setAcceptMode(QFileDialog.AcceptMode.AcceptSave)
        dialog.setOption(QFileDialog.Option.DontUseNativeDialog, True)
        dialog.setOption(QFileDialog.Option.ShowDirsOnly, True)
        
        if dialog.exec() == QDialog.DialogCode.Accepted:
            target_dir = dialog.selectedFiles()[0]
        else:
            return  # User cancelled
            
        if target_dir:
            os.makedirs(target_dir, exist_ok=True)
            
        script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../cpp_core/tooling/sync_brain.py'))
        
        # Pull live state from the voter
        matrix = {}
        for nA, conns in self.voter.association_matrix.items():
            matrix[int(nA)] = {int(nB): float(w) for nB, w in conns.items()}
            
        config = {
            "hebbian_rate": self.voter.hebbian_rate,
            "decay_rate": self.voter.decay_rate,
            "threshold": self.voter.threshold,
            "sync_window_ms": self.voter.sync_window_ms
        }
        
        self.export_dlg = ExportBrainDialog(self, script_path, matrix, config)
        self.export_dlg.target_dir = target_dir
        self.export_dlg.show()
        self.export_dlg.start_export()

        
    def restore_layout_state(self):
        if self.settings.contains("geometry"):
            self.restoreGeometry(self.settings.value("geometry"))
        if self.settings.contains("windowState"):
            self.restoreState(self.settings.value("windowState"))
            
    def reset_layout(self):
        # Reset to defaults
        # We just remove the saved state
        self.settings.remove("geometry")
        self.settings.remove("windowState")
        self.dock_agents.setFloating(False)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.dock_agents)
        self.dock_voting.setFloating(False)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.dock_voting)
        self.dock_log.setFloating(False)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dock_log)
        self.resize(1200, 800)
        
    def show_graph_view(self):
        if not self.graph_window:
            self.graph_window = QWidget()
            self.graph_window.setWindowTitle("Consensus Graph View")
            self.graph_window.resize(600, 600)
            layout = QVBoxLayout(self.graph_window)
            self.graph_view_widget = ConsensusGraphWidget()
            layout.addWidget(self.graph_view_widget)
        
        self.graph_window.show()
        self.graph_window.raise_()
        self.graph_window.activateWindow()
        
    def start_server_task(self):
        # Schedule the server start coroutine
        asyncio.ensure_future(self.server.start_server(), loop=self.loop)
        
    def on_voter_update(self, data):
        # Called from Voter (likely in async thread context or main thread depending on execution)
        # We store data for the timer to pick up, or update directly if thread-safe
        self.last_voter_data = data
        
    def on_consensus_received(self, graph_data):
        """
        Called when the background thread has fresh consensus data.
        """
        consensus = graph_data['consensus']
        
        # 2. Update Visualizations
        if consensus:
             self.timeline_widget.update_timeline(graph_data)
             
             if self.graph_window and self.graph_window.isVisible():
                 self.graph_view_widget.update_graph(graph_data)

             # Update Debug Plots
             self.resonance_plot.update_resonance(
                 consensus.resonance_score, 
                 self.voter.threshold,
                 tle=graph_data.get('tle')
             )
             
             # Update Active Inference Viz
             self.ai_viz.update_viz(self.server.active_harness, graph_data)
             
             # Update Vector Heatmap
             if hasattr(self, 'vector_widget'):
                 self.vector_widget.update_vector(consensus.fused_embedding)
             
             # Extract active nodes from paired token list for crosshair
             if len(graph_data['candidates']) >= 2:
                 idA = graph_data['candidates'][0].active_node_id
                 idB = graph_data['candidates'][1].active_node_id
                 if idA != -1 and idB != -1:
                     self.matrix_view.update_crosshair_target(idA, idB)
             
             # Pump Matrix Render Loop
             nav_speed = getattr(self.voter, 'nav_speed', 0.1)
             self.matrix_view.update_matrix(
                 self.voter.association_matrix, 
                 nav_speed, 
                 node_count=graph_data.get('node_count', 0)
             )
             
             # Update Log Metrics
             self.consensus_log.update_log(self.voter, node_count=graph_data.get('node_count', 0))

    def update_ui_frame(self):
        """
        High-frequency frame update for telemetry and simple list stats.
        Also drains staged consensus data from the inference loop (decoupled via _pending_ui_data).
        """
        # Drain latest consensus from the inference loop
        data = self.server._pending_ui_data
        if data is not None:
            self.server._pending_ui_data = None
            self.on_consensus_received(data)

        # Update List
        # Iterate over all sources in voter buffer
        for source_id, buffer in list(self.voter.input_buffers.items()):
            if not buffer: continue
            last_token = buffer[-1]
            
            # Calculate latency using server-side received_time to avoid clock skew
            # Latency = (Time Now - Time Token Arrived at Server)
            latency = (time.time() - last_token.received_time) * 1000 if last_token.received_time > 0 else 0
            
            self.list_widget.update_agent(source_id, {
                'state': 'RUNNING',
                'latency': f"{latency:.1f}ms",
                'dopamine': last_token.dopamine_level,
                'serotonin': last_token.serotonin_level
            })
            
        # Update Active Inference Metrics
        self.active_inf_widget.update_metrics()

    def closeEvent(self, event):
        # Save state
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        
        # Stop Server
        asyncio.ensure_future(self.server.stop_server(), loop=self.loop)
        event.accept()
