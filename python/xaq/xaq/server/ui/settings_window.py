from PyQt6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QDoubleSpinBox, QPushButton, QFormLayout
from PyQt6.QtCore import Qt

class SettingsWindow(QDialog):
    def __init__(self, server_instance, parent=None):
        super().__init__(parent)
        self.server = server_instance
        self.setWindowTitle("Global Settings")
        self.setMinimumWidth(300)
        
        self.init_ui()
        self.load_current_settings()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        form_layout = QFormLayout()
        
        # Timeout Checkbox
        self.chk_timeout = QCheckBox("Enable Agent Timeout")
        self.chk_timeout.stateChanged.connect(self.on_settings_changed)
        form_layout.addRow(self.chk_timeout)
        
        # Timeout Spinbox
        self.spin_timeout = QDoubleSpinBox()
        self.spin_timeout.setRange(1.0, 60.0)
        self.spin_timeout.setSingleStep(1.0)
        self.spin_timeout.setSuffix(" sec")
        self.spin_timeout.valueChanged.connect(self.on_settings_changed)
        form_layout.addRow("Timeout Duration:", self.spin_timeout)

        # Max Consensus Token Count
        self.spin_max_tokens = QDoubleSpinBox()
        self.spin_max_tokens.setRange(100.0, 100000.0)
        self.spin_max_tokens.setSingleStep(500.0)
        self.spin_max_tokens.setDecimals(0)
        self.spin_max_tokens.valueChanged.connect(self.on_settings_changed)
        form_layout.addRow("Max Consensus Tokens:", self.spin_max_tokens)
        
        layout.addLayout(form_layout)
        layout.addStretch()
        
        # Close Button
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.accept)
        btn_layout.addWidget(btn_close)
        
        layout.addLayout(btn_layout)
        
    def load_current_settings(self):
        self.chk_timeout.blockSignals(True)
        self.spin_timeout.blockSignals(True)
        self.spin_max_tokens.blockSignals(True)
        
        self.chk_timeout.setChecked(self.server.timeout_enabled)
        self.spin_timeout.setValue(self.server.timeout_seconds)
        if hasattr(self.parent(), 'voter'):
            self.spin_max_tokens.setValue(float(getattr(self.parent().voter, 'max_consensus_count', 5000)))
        
        self.chk_timeout.blockSignals(False)
        self.spin_timeout.blockSignals(False)
        self.spin_max_tokens.blockSignals(False)
        
    def on_settings_changed(self):
        self.server.set_timeout_enabled(self.chk_timeout.isChecked())
        self.server.set_timeout_seconds(self.spin_timeout.value())
        if hasattr(self.parent(), 'voter'):
            self.parent().voter.set_max_consensus_count(int(self.spin_max_tokens.value()))
