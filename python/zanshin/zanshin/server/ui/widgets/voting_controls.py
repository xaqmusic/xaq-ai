from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QDoubleSpinBox, QGroupBox, QGridLayout, QPushButton, QSpinBox
from PyQt6.QtCore import Qt, QTimer

class VotingControlsWidget(QWidget):
    def __init__(self, voter):
        super().__init__()
        self.voter = voter
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        
        group_box = QGroupBox("Lateral Voter Parameters")
        grid = QGridLayout(group_box)
        grid.setSpacing(10)
        layout.addWidget(group_box)
        
        row = 0
        
        # 1. Hebbian Threshold
        self.thresh_label = QLabel("Hebbian Threshold:")
        self.thresh_spin = QDoubleSpinBox()
        self.thresh_spin.setRange(0.01, 1.0)
        self.thresh_spin.setSingleStep(0.05)
        self.thresh_spin.setValue(self.voter.threshold)
        self.thresh_slider = QSlider(Qt.Orientation.Horizontal)
        self.thresh_slider.setRange(1, 100)
        self.thresh_slider.setValue(int(self.voter.threshold * 100))
        
        self.thresh_spin.valueChanged.connect(self._on_thresh_spin)
        self.thresh_slider.valueChanged.connect(self._on_thresh_slider)
        
        grid.addWidget(self.thresh_label, row, 0)
        grid.addWidget(self.thresh_slider, row, 1)
        grid.addWidget(self.thresh_spin, row, 2)
        row += 1
        
        # 2. Temporal Horizon
        self.horizon_label = QLabel("Temporal Horizon (s):")
        self.horizon_spin = QDoubleSpinBox()
        self.horizon_spin.setRange(0.1, 5.0)
        self.horizon_spin.setSingleStep(0.1)
        self.horizon_spin.setValue(self.voter.agent_timeout)
        self.horizon_slider = QSlider(Qt.Orientation.Horizontal)
        self.horizon_slider.setRange(1, 50)
        self.horizon_slider.setValue(int(self.voter.agent_timeout * 10))
        
        self.horizon_spin.valueChanged.connect(self._on_horizon_spin)
        self.horizon_slider.valueChanged.connect(self._on_horizon_slider)
        
        grid.addWidget(self.horizon_label, row, 0)
        grid.addWidget(self.horizon_slider, row, 1)
        grid.addWidget(self.horizon_spin, row, 2)
        row += 1
        
        # 3. Base Co-occurrence
        self.base_label = QLabel("Base Co-occurrence:")
        self.base_spin = QDoubleSpinBox()
        self.base_spin.setRange(0.0, 1.0)
        self.base_spin.setSingleStep(0.05)
        self.base_spin.setValue(self.voter.temporal_coincidence)
        self.base_slider = QSlider(Qt.Orientation.Horizontal)
        self.base_slider.setRange(0, 100)
        self.base_slider.setValue(int(self.voter.temporal_coincidence * 100))
        
        self.base_spin.valueChanged.connect(self._on_base_spin)
        self.base_slider.valueChanged.connect(self._on_base_slider)
        
        grid.addWidget(self.base_label, row, 0)
        grid.addWidget(self.base_slider, row, 1)
        grid.addWidget(self.base_spin, row, 2)
        row += 1
        
        # 4. Hebbian Learning Rate
        self.rate_label = QLabel("Learning Rate:")
        self.rate_spin = QDoubleSpinBox()
        self.rate_spin.setRange(0.001, 1.0)
        self.rate_spin.setDecimals(3)
        self.rate_spin.setSingleStep(0.01)
        self.rate_spin.setValue(self.voter.hebbian_rate)
        self.rate_slider = QSlider(Qt.Orientation.Horizontal)
        self.rate_slider.setRange(1, 1000)
        self.rate_slider.setValue(int(self.voter.hebbian_rate * 1000))
        
        self.rate_spin.valueChanged.connect(self._on_rate_spin)
        self.rate_slider.valueChanged.connect(self._on_rate_slider)
        
        grid.addWidget(self.rate_label, row, 0)
        grid.addWidget(self.rate_slider, row, 1)
        grid.addWidget(self.rate_spin, row, 2)
        row += 1
        
        # 5. Matrix Decay Rate
        self.decay_label = QLabel("Matrix Decay:")
        self.decay_spin = QDoubleSpinBox()
        self.decay_spin.setRange(0.0, 0.1)
        self.decay_spin.setDecimals(4)
        self.decay_spin.setSingleStep(0.001)
        self.decay_spin.setValue(self.voter.decay_rate)
        self.decay_slider = QSlider(Qt.Orientation.Horizontal)
        self.decay_slider.setRange(0, 1000)
        self.decay_slider.setValue(int(self.voter.decay_rate * 10000))
        
        self.decay_spin.valueChanged.connect(self._on_decay_spin)
        self.decay_slider.valueChanged.connect(self._on_decay_slider)
        
        grid.addWidget(self.decay_label, row, 0)
        grid.addWidget(self.decay_slider, row, 1)
        grid.addWidget(self.decay_spin, row, 2)
        row += 1
        
        # 6. Navigation Speed
        self.nav_label = QLabel("Navigation Speed:")
        self.nav_spin = QDoubleSpinBox()
        self.nav_spin.setRange(0.001, 0.1)
        self.nav_spin.setDecimals(3)
        self.nav_spin.setSingleStep(0.005)
        self.nav_spin.setValue(getattr(self.voter, 'nav_speed', 0.05))
        self.nav_slider = QSlider(Qt.Orientation.Horizontal)
        self.nav_slider.setRange(1, 100)
        self.nav_slider.setValue(int(getattr(self.voter, 'nav_speed', 0.05) * 1000))
        
        self.nav_spin.valueChanged.connect(self._on_nav_spin)
        self.nav_slider.valueChanged.connect(self._on_nav_slider)
        
        grid.addWidget(self.nav_label, row, 0)
        grid.addWidget(self.nav_slider, row, 1)
        grid.addWidget(self.nav_spin, row, 2)
        row += 1

        # 7. Trajectory Weight
        self.traj_label = QLabel("Trajectory Weight:")
        self.traj_spin = QDoubleSpinBox()
        self.traj_spin.setRange(0.0, 1.0)
        self.traj_spin.setSingleStep(0.05)
        self.traj_spin.setValue(getattr(self.voter, 'trajectory_weight', 0.4))
        self.traj_slider = QSlider(Qt.Orientation.Horizontal)
        self.traj_slider.setRange(0, 100)
        self.traj_slider.setValue(int(getattr(self.voter, 'trajectory_weight', 0.4) * 100))
        
        self.traj_spin.valueChanged.connect(self._on_traj_spin)
        self.traj_slider.valueChanged.connect(self._on_traj_slider)
        
        grid.addWidget(self.traj_label, row, 0)
        grid.addWidget(self.traj_slider, row, 1)
        grid.addWidget(self.traj_spin, row, 2)
        row += 1

        # 8. Instant Match Weight
        self.inst_label = QLabel("Instant Match Weight:")
        self.inst_spin = QDoubleSpinBox()
        self.inst_spin.setRange(0.0, 1.0)
        self.inst_spin.setSingleStep(0.05)
        self.inst_spin.setValue(getattr(self.voter, 'instant_match_weight', 0.2))
        self.inst_slider = QSlider(Qt.Orientation.Horizontal)
        self.inst_slider.setRange(0, 100)
        self.inst_slider.setValue(int(getattr(self.voter, 'instant_match_weight', 0.2) * 100))
        
        self.inst_spin.valueChanged.connect(self._on_inst_spin)
        self.inst_slider.valueChanged.connect(self._on_inst_slider)
        
        grid.addWidget(self.inst_label, row, 0)
        grid.addWidget(self.inst_slider, row, 1)
        grid.addWidget(self.inst_spin, row, 2)
        row += 1

        # 9. Node Weight
        self.node_label = QLabel("Node Weight:")
        self.node_spin = QDoubleSpinBox()
        self.node_spin.setRange(0.0, 5.0)
        self.node_spin.setSingleStep(0.1)
        self.node_spin.setValue(getattr(self.voter, 'node_weight', 1.0))
        self.node_slider = QSlider(Qt.Orientation.Horizontal)
        self.node_slider.setRange(0, 500)
        self.node_slider.setValue(int(getattr(self.voter, 'node_weight', 1.0) * 100))
        
        self.node_spin.valueChanged.connect(self._on_node_spin)
        self.node_slider.valueChanged.connect(self._on_node_slider)
        
        grid.addWidget(self.node_label, row, 0)
        grid.addWidget(self.node_slider, row, 1)
        grid.addWidget(self.node_spin, row, 2)
        row += 1

        # 10. Neurotransmitter Weight
        self.neuro_label = QLabel("Neurotransmitter Weight:")
        self.neuro_spin = QDoubleSpinBox()
        self.neuro_spin.setRange(0.0, 5.0)
        self.neuro_spin.setSingleStep(0.1)
        self.neuro_spin.setValue(getattr(self.voter, 'neurotransmitter_weight', 1.0))
        self.neuro_slider = QSlider(Qt.Orientation.Horizontal)
        self.neuro_slider.setRange(0, 500)
        self.neuro_slider.setValue(int(getattr(self.voter, 'neurotransmitter_weight', 1.0) * 100))
        
        self.neuro_spin.valueChanged.connect(self._on_neuro_spin)
        self.neuro_slider.valueChanged.connect(self._on_neuro_slider)
        
        grid.addWidget(self.neuro_label, row, 0)
        grid.addWidget(self.neuro_slider, row, 1)
        grid.addWidget(self.neuro_spin, row, 2)
        row += 1
        
        # 11. Retroactive Lookback
        self.lookback_label = QLabel("Retroactive Lookback:")
        self.lookback_spin = QSpinBox()
        self.lookback_spin.setRange(1, 100)
        self.lookback_spin.setSingleStep(1)
        self.lookback_spin.setValue(getattr(self.voter, 'lookback_window', 25))
        self.lookback_slider = QSlider(Qt.Orientation.Horizontal)
        self.lookback_slider.setRange(1, 100)
        self.lookback_slider.setValue(int(getattr(self.voter, 'lookback_window', 25)))
        
        self.lookback_spin.valueChanged.connect(self._on_lookback_spin)
        self.lookback_slider.valueChanged.connect(self._on_lookback_slider)
        
        grid.addWidget(self.lookback_label, row, 0)
        grid.addWidget(self.lookback_slider, row, 1)
        grid.addWidget(self.lookback_spin, row, 2)
        
        # Add Tooltips
        self.thresh_label.setToolTip("Minimum resonance required to create a concept token and associate nodes.")
        self.horizon_label.setToolTip("Max time difference (seconds) between agents to be considered for consensus.")
        self.base_label.setToolTip("Baseline boost solely for arriving in the same temporal window.")
        self.rate_label.setToolTip("Amount to increase the Hebbian association weight when consensus is reached.")
        self.decay_label.setToolTip("Amount to decrease all association weights per consensus cycle.")
        self.nav_label.setToolTip("Speed at which the crosshairs move towards the latest consensus.")
        self.traj_label.setToolTip("Weight of the trajectory overlap (previous 5 nodes) when determining resonance.")
        self.inst_label.setToolTip("Weight of instantaneous active node matching when determining resonance.")
        self.node_label.setToolTip("Multiplier for raw dopamine level of active nodes (Unbaked=0.0, Baked=0.5, Supernode=1.0).")
        self.neuro_label.setToolTip("Multiplier for raw serotonin level when scaling trust/confidence of the node.")
        self.lookback_label.setToolTip("Number of recent frames to retroactively boost or penalize.")

        layout.addStretch()
        
        # Reset Button Functionality
        self.reset_btn = QPushButton("Reset Consensus History")
        self.reset_btn.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold; border-radius: 4px; padding: 8px;")
        self.reset_btn.clicked.connect(self._on_reset_clicked)
        layout.addWidget(self.reset_btn)

    def _on_reset_clicked(self):
        self.voter.reset_consensus()
        self.reset_btn.setText("Resetting...")
        self.reset_btn.setEnabled(False)
        QTimer.singleShot(1000, self._restore_reset_btn)
        
    def _restore_reset_btn(self):
        self.reset_btn.setText("Reset Consensus History")
        self.reset_btn.setEnabled(True)

    # Handlers (Sync SpinBox & Slider, apply to Voter)
    def _on_thresh_spin(self, val):
        self.thresh_slider.blockSignals(True)
        self.thresh_slider.setValue(int(val * 100))
        self.thresh_slider.blockSignals(False)
        self.voter.threshold = val

    def _on_thresh_slider(self, val):
        fval = val / 100.0
        self.thresh_spin.blockSignals(True)
        self.thresh_spin.setValue(fval)
        self.thresh_spin.blockSignals(False)
        self.voter.threshold = fval

    def _on_horizon_spin(self, val):
        self.horizon_slider.blockSignals(True)
        self.horizon_slider.setValue(int(val * 10))
        self.horizon_slider.blockSignals(False)
        self.voter.agent_timeout = val

    def _on_horizon_slider(self, val):
        fval = val / 10.0
        self.horizon_spin.blockSignals(True)
        self.horizon_spin.setValue(fval)
        self.horizon_spin.blockSignals(False)
        self.voter.agent_timeout = fval

    def _on_base_spin(self, val):
        self.base_slider.blockSignals(True)
        self.base_slider.setValue(int(val * 100))
        self.base_slider.blockSignals(False)
        self.voter.temporal_coincidence = val

    def _on_base_slider(self, val):
        fval = val / 100.0
        self.base_spin.blockSignals(True)
        self.base_spin.setValue(fval)
        self.base_spin.blockSignals(False)
        self.voter.temporal_coincidence = fval

    def _on_rate_spin(self, val):
        self.rate_slider.blockSignals(True)
        self.rate_slider.setValue(int(val * 1000))
        self.rate_slider.blockSignals(False)
        self.voter.hebbian_rate = val

    def _on_rate_slider(self, val):
        fval = val / 1000.0
        self.rate_spin.blockSignals(True)
        self.rate_spin.setValue(fval)
        self.rate_spin.blockSignals(False)
        self.voter.hebbian_rate = fval

    def _on_decay_spin(self, val):
        self.decay_slider.blockSignals(True)
        self.decay_slider.setValue(int(val * 10000))
        self.decay_slider.blockSignals(False)
        self.voter.decay_rate = val

    def _on_decay_slider(self, val):
        fval = val / 10000.0
        self.decay_spin.blockSignals(True)
        self.decay_spin.setValue(fval)
        self.decay_spin.blockSignals(False)
        self.voter.decay_rate = fval

    def _on_nav_spin(self, val):
        self.nav_slider.blockSignals(True)
        self.nav_slider.setValue(int(val * 1000))
        self.nav_slider.blockSignals(False)
        self.voter.nav_speed = val

    def _on_nav_slider(self, val):
        fval = val / 1000.0
        self.nav_spin.blockSignals(True)
        self.nav_spin.setValue(fval)
        self.nav_spin.blockSignals(False)
        self.voter.nav_speed = fval

    def _on_traj_spin(self, val):
        self.traj_slider.blockSignals(True)
        self.traj_slider.setValue(int(val * 100))
        self.traj_slider.blockSignals(False)
        self.voter.trajectory_weight = val

    def _on_traj_slider(self, val):
        fval = val / 100.0
        self.traj_spin.blockSignals(True)
        self.traj_spin.setValue(fval)
        self.traj_spin.blockSignals(False)
        self.voter.trajectory_weight = fval

    def _on_inst_spin(self, val):
        self.inst_slider.blockSignals(True)
        self.inst_slider.setValue(int(val * 100))
        self.inst_slider.blockSignals(False)
        self.voter.instant_match_weight = val

    def _on_inst_slider(self, val):
        fval = val / 100.0
        self.inst_spin.blockSignals(True)
        self.inst_spin.setValue(fval)
        self.inst_spin.blockSignals(False)
        self.voter.instant_match_weight = fval

    def _on_node_spin(self, val):
        self.node_slider.blockSignals(True)
        self.node_slider.setValue(int(val * 100))
        self.node_slider.blockSignals(False)
        self.voter.node_weight = val

    def _on_node_slider(self, val):
        fval = val / 100.0
        self.node_spin.blockSignals(True)
        self.node_spin.setValue(fval)
        self.node_spin.blockSignals(False)
        self.voter.node_weight = fval

    def _on_neuro_spin(self, val):
        self.neuro_slider.blockSignals(True)
        self.neuro_slider.setValue(int(val * 100))
        self.neuro_slider.blockSignals(False)
        self.voter.neurotransmitter_weight = val

    def _on_neuro_slider(self, val):
        fval = val / 100.0
        self.neuro_spin.blockSignals(True)
        self.neuro_spin.setValue(fval)
        self.neuro_spin.blockSignals(False)
        self.voter.neurotransmitter_weight = fval

    def _on_lookback_spin(self, val):
        self.lookback_slider.blockSignals(True)
        self.lookback_slider.setValue(val)
        self.lookback_slider.blockSignals(False)
        self.voter.lookback_window = val

    def _on_lookback_slider(self, val):
        self.lookback_spin.blockSignals(True)
        self.lookback_spin.setValue(val)
        self.lookback_spin.blockSignals(False)
        self.voter.lookback_window = val
