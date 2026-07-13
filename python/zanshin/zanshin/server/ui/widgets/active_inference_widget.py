from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame, QSlider, QGridLayout
from PyQt6.QtCore import Qt, pyqtSignal
import time

class ActiveInferenceWidget(QWidget):
    mode_changed = pyqtSignal(bool) # True for Active, False for Passive

    def __init__(self, server=None, harness=None, parent=None):
        super().__init__(parent)
        self.server = server
        self.harness = harness
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 1. Status Section
        status_frame = QFrame()
        status_frame.setFrameShape(QFrame.Shape.StyledPanel)
        status_layout = QVBoxLayout(status_frame)
        
        self.lbl_mode = QLabel("Mode: <b>PASSIVE (Observation)</b>")
        self.lbl_mode.setStyleSheet("color: #FFA500;")
        status_layout.addWidget(self.lbl_mode)
        
        self.lbl_tle = QLabel("Temporal Loss (TLE): 0.000")
        status_layout.addWidget(self.lbl_tle)
        
        self.lbl_stability = QLabel("Stability (Serotonin): 0.0%")
        status_layout.addWidget(self.lbl_stability)
        
        self.lbl_contributors = QLabel("Contributing Agents: 0")
        status_layout.addWidget(self.lbl_contributors)
        
        self.lbl_resonance = QLabel("Consonance Score: 0.000")
        status_layout.addWidget(self.lbl_resonance)
        
        layout.addWidget(status_frame)
        
        # 2. Controls Section
        ctrl_layout = QHBoxLayout()
        
        self.btn_toggle = QPushButton("GO ACTIVE")
        self.btn_toggle.setCheckable(True)
        self.btn_toggle.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")
        self.btn_toggle.clicked.connect(self.on_toggle_clicked)
        ctrl_layout.addWidget(self.btn_toggle)
        
        self.btn_reset = QPushButton("Reset Meta-EPM")
        self.btn_reset.clicked.connect(self.on_reset_clicked)
        ctrl_layout.addWidget(self.btn_reset)
        
        self.btn_flip = QPushButton("Flip Control Direction")
        self.btn_flip.clicked.connect(self.on_flip_clicked)
        ctrl_layout.addWidget(self.btn_flip)
        
        layout.addLayout(ctrl_layout)
        
        # 3. Control Parameters Section
        param_frame = QFrame()
        param_frame.setFrameShape(QFrame.Shape.StyledPanel)
        param_layout = QGridLayout(param_frame)
        
        param_layout.addWidget(QLabel("Action Stiffness (P-Gain):"), 0, 0)
        self.sld_stiffness = QSlider(Qt.Orientation.Horizontal)
        self.sld_stiffness.setRange(1, 200) # 0.01 to 2.0
        self.sld_stiffness.setValue(80) 
        self.sld_stiffness.valueChanged.connect(self.on_stiffness_changed)
        param_layout.addWidget(self.sld_stiffness, 0, 1)
        self.lbl_stiffness = QLabel("0.80")
        param_layout.addWidget(self.lbl_stiffness, 0, 2)
        
        param_layout.addWidget(QLabel("Prediction Horizon:"), 1, 0)
        self.sld_horizon = QSlider(Qt.Orientation.Horizontal)
        self.sld_horizon.setRange(1, 50)
        self.sld_horizon.setValue(10)
        self.sld_horizon.valueChanged.connect(self.on_horizon_changed)
        param_layout.addWidget(self.sld_horizon, 1, 1)
        self.lbl_horizon = QLabel("10")
        param_layout.addWidget(self.lbl_horizon, 1, 2)
        
        param_layout.addWidget(QLabel("Meta-Learning Rate:"), 2, 0)
        self.sld_lr = QSlider(Qt.Orientation.Horizontal)
        self.sld_lr.setRange(1, 100) # 0.001 to 0.1
        self.sld_lr.setValue(10) # Default 0.01
        self.sld_lr.valueChanged.connect(self.on_lr_changed)
        param_layout.addWidget(self.sld_lr, 2, 1)
        self.lbl_lr = QLabel("0.010")
        param_layout.addWidget(self.lbl_lr, 2, 2)
        
        layout.addWidget(QLabel("<b>Dynamic Control Tuning</b>"))
        layout.addWidget(param_frame)

        # 4. Game Connection Status
        self.lbl_game_conn = QLabel("Game Connection: <i>Checking...</i>")
        layout.addWidget(self.lbl_game_conn)
        
        layout.addStretch()

    def update_metrics(self):
        if not self.harness:
            return
            
        # Update Labels
        if hasattr(self.harness, 'is_active'):
            active = self.harness.is_active
            mode_text = "ACTIVE (Control)" if active else "PASSIVE (Observation)"
            color = "#00FF00" if active else "#FFA500"
            self.lbl_mode.setText(f"Mode: <b>{mode_text}</b>")
            self.lbl_mode.setStyleSheet(f"color: {color};")
            self.btn_toggle.setText("GO PASSIVE" if active else "GO ACTIVE")
            self.btn_toggle.setChecked(active)
            self.btn_toggle.setStyleSheet(f"background-color: {'#c62828' if active else '#2e7d32'}; color: white; font-weight: bold;")

        if hasattr(self.harness, 'meta_epm'):
            tle = getattr(self.harness.meta_epm, 'last_tle', 0.0)
            self.lbl_tle.setText(f"Temporal Loss (TLE): {tle:.4f}")
            
            stability = max(0.0, 1.0 - tle) * 100
            self.lbl_stability.setText(f"Stability (Serotonin): {stability:.1f}%")
            
        # Update Consensus Info from Voter (via server)
        if self.server and self.server.voter:
            voter = self.server.voter
            sources = len(voter.active_sources)
            self.lbl_contributors.setText(f"Contributing Agents: {sources}")
            if voter.last_consensus:
                self.lbl_resonance.setText(f"Consonance Score: {voter.last_consensus.resonance_score:.3f}")
        elif hasattr(self.harness, 'stability'):
            # Legacy Harness
            self.lbl_tle.setText("Temporal Loss (TLE): N/A (Legacy)")
            stability = getattr(self.harness, 'stability', 0.0) * 100
            self.lbl_stability.setText(f"Stability (Legacy): {stability:.1f}%")
        else:
            self.lbl_tle.setText("Temporal Loss (TLE): 0.0000")
            self.lbl_stability.setText("Stability: 0.0%")

        # Check if game is among connected agents
        if self.server:
            last_active = self.server.agent_last_active.get('breakout_game')
            if last_active and (time.time() - last_active < 2.0):
                self.lbl_game_conn.setText("Game Connection: <span style='color: #00FF00;'><b>Connected</b></span>")
            else:
                self.lbl_game_conn.setText("Game Connection: <span style='color: #FF0000;'><i>Disconnected</i></span>")

    def on_toggle_clicked(self, checked):
        if self.harness:
            # We emit through the server side if possible, or update harness directly
            self.mode_changed.emit(checked)
            
    def on_stiffness_changed(self, val):
        f_val = val / 100.0
        self.lbl_stiffness.setText(f"{f_val:.2f}")
        if self.harness and hasattr(self.harness, 'set_action_stiffness'):
            self.harness.set_action_stiffness(f_val)

    def on_horizon_changed(self, val):
        self.lbl_horizon.setText(str(val))
        if self.harness and hasattr(self.harness, 'set_prediction_horizon'):
            self.harness.set_prediction_horizon(val)

    def on_lr_changed(self, val):
        f_val = val / 1000.0
        self.lbl_lr.setText(f"{f_val:.3f}")
        if self.harness and hasattr(self.harness, 'set_meta_learning_rate'):
            self.harness.set_meta_learning_rate(f_val)

    def on_reset_clicked(self):
        if self.harness and hasattr(self.harness, 'meta_epm'):
            self.harness.meta_epm.reset()
            # Also clear calibration
            if hasattr(self.harness, 'calibration_buffer'):
                self.harness.calibration_buffer = []
                self.harness.is_calibrated = False
            print("Cognitive State & Calibration Reset.")

    def on_flip_clicked(self):
        if self.harness and hasattr(self.harness, 'flip_spatial_sign'):
            self.harness.flip_spatial_sign()
            print("Spatial Control Direction Flipped.")
