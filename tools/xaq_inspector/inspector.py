"""Main window: module list on the left, focused inspector on the right.

Architecture:
  - ControlClient owns the TCP request/reply socket (low-rate verbs).
  - DiagSubscriber owns the ZMQ SUB socket (high-rate streaming).
  - One subscription per module currently being inspected; clicking a
    different module unsubscribes the old one and creates a new one.
  - Diag callbacks fire on the SUB thread; we marshal them to the GUI
    thread via a Qt signal so widget updates run on the main thread.

Connection failures are surfaced as status-bar errors but never crash
the app — the inspector is a viewer, not a controller, so it should
degrade gracefully when the brain process isn't around.
"""
from __future__ import annotations

import argparse
import sys
from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QAction, QGuiApplication
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QListWidget, QListWidgetItem, QSplitter,
    QStackedWidget, QStatusBar, QVBoxLayout, QWidget, QLabel,
    QPushButton, QHBoxLayout, QSpinBox, QMessageBox,
)

from .transport import ControlClient, DiagSubscriber, DiagPayload
from .widgets import widget_for, wrap_with_description


# ---------------------------------------------------------------------------
# Bridge: SUB-thread → GUI-thread
# ---------------------------------------------------------------------------
# Qt signals are thread-safe across the moc dispatch boundary; we use this
# as the sole hand-off point between DiagSubscriber's worker thread and
# the inspector window.

class PayloadBridge(QObject):
    payload = pyqtSignal(object)  # DiagPayload


class InspectorWindow(QMainWindow):
    def __init__(self, control_host: str, control_port: int,
                 diag_host: str, diag_port: int):
        super().__init__()
        self.setWindowTitle("xaq inspector")
        # Size against the AVAILABLE screen, not a hardcoded 1280x800.  Qt sizes in
        # LOGICAL pixels, so on a 1920x1080 display at 150% desktop scaling the whole
        # logical desktop is only 1280x720 — the old fixed size asked for exactly the
        # full logical width, which put the window's right edge on the screen boundary
        # where it cannot be grabbed.  The window then resized vertically but not
        # horizontally, which is precisely the reported symptom.
        scr = QGuiApplication.primaryScreen()
        if scr is not None:
            av = scr.availableGeometry()
            w = max(640, min(1280, int(av.width() * 0.88)))
            h = max(480, min(800, int(av.height() * 0.88)))
            self.resize(w, h)
            self.move(av.left() + (av.width() - w) // 2,
                      av.top() + (av.height() - h) // 2)
        else:
            self.resize(1280, 800)
        # Never let content pin the window wide: it must always be shrinkable.
        self.setMinimumSize(560, 400)

        self.control = ControlClient(control_host, control_port)
        self.diag    = DiagSubscriber(diag_host, diag_port)

        # state
        self._modules: list[dict] = []
        self._current_sub_id: Optional[int] = None
        self._current_topic_prefix: Optional[str] = None
        self._current_module_id: Optional[str] = None
        self._current_widget = None  # the live widget in the right pane

        self._bridge = PayloadBridge()
        self._bridge.payload.connect(self._on_payload_main_thread)

        self._build_ui()
        self._wire_diag()
        self._refresh_modules()

    # ----- UI construction -----

    def _build_ui(self) -> None:
        split = QSplitter()
        split.setOrientation(Qt.Orientation.Horizontal)

        # Left: module list + controls
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(6, 6, 6, 6)

        header = QLabel("Modules")
        header.setStyleSheet("color:#fff; font-weight:bold; font-size: 13px;")
        left_layout.addWidget(header)

        self._list = QListWidget()
        self._list.itemActivated.connect(self._on_module_activated)
        self._list.itemClicked.connect(self._on_module_activated)
        left_layout.addWidget(self._list, 1)

        # Subscribe rate selector
        rate_row = QHBoxLayout()
        rate_row.addWidget(QLabel("hz:"))
        self._hz = QSpinBox()
        self._hz.setRange(1, 60)
        self._hz.setValue(30)
        rate_row.addWidget(self._hz)
        rate_row.addStretch(1)
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_modules)
        rate_row.addWidget(refresh_btn)
        left_layout.addLayout(rate_row)

        split.addWidget(left)

        # Right: stacked widget with one card per inspected module
        self._right = QStackedWidget()
        self._placeholder = QLabel(
            "Click a module to subscribe to its live diag stream."
        )
        self._placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._placeholder.setStyleSheet("color:#888; font-size: 14px;")
        self._right.addWidget(self._placeholder)
        split.addWidget(self._right)

        split.setSizes([max(180, self.width() // 5), self.width() - max(180, self.width() // 5)])
        self.setCentralWidget(split)

        self.setStatusBar(QStatusBar())
        self._set_status("idle")

        # Window-level dark styling (per-widget overrides in the cards).
        self.setStyleSheet("""
            QMainWindow { background: #1a1c20; }
            QListWidget { background: #181a1d; color: #ddd; border: 1px solid #2a2d33; }
            QListWidget::item:selected { background: #2a4060; }
            QLabel { color: #ddd; }
            QSpinBox { background: #222; color: #ddd; border: 1px solid #444; }
            QPushButton { background: #2a4060; color: #fff; padding: 4px 12px;
                          border: 1px solid #345; border-radius: 3px; }
            QPushButton:hover { background: #345070; }
        """)

        quit_action = QAction("Quit", self)
        quit_action.setShortcut("Ctrl+Q")
        quit_action.triggered.connect(self.close)
        self.addAction(quit_action)

    def _wire_diag(self) -> None:
        # Bridge from worker thread → GUI signal.
        self.diag.start(lambda p: self._bridge.payload.emit(p))

    # ----- module list / subscription -----

    def _refresh_modules(self) -> None:
        # Force a fresh control-socket connection on every refresh.
        # When Godot relaunches, the prior TCP socket is dead but
        # ControlClient.connect() is idempotent — we'd reuse the stale
        # socket and the next call() would fail.  reconnect() drops
        # any existing socket and opens a new one, so the user can
        # leave the inspector window open across Godot restarts and
        # just hit Refresh.
        try:
            self.control.reconnect()
            resp = self.control.call("list_modules")
        except Exception as e:
            self._set_status(f"control error: {e}")
            return
        if resp.get("status") != "ok":
            self._set_status(f"list_modules: {resp}")
            return
        # Tear down any prior subscription state — sub_id from the
        # previous Godot instance is meaningless to the new one, and
        # the active widget is showing stale data.  User re-clicks a
        # module to subscribe in the new instance.
        prior_module_id = self._current_module_id
        self._current_sub_id = None
        self._current_topic_prefix = None
        self._current_module_id = None
        if self._current_widget is not None:
            self._right.removeWidget(self._current_widget)
            self._current_widget.deleteLater()
            self._current_widget = None
            self._right.setCurrentWidget(self._placeholder)
        self._modules = list(resp.get("modules", []))
        self._list.clear()
        for m in self._modules:
            item = QListWidgetItem(f"{m.get('id')}   ({m.get('type')})")
            item.setData(Qt.ItemDataRole.UserRole, m)
            self._list.addItem(item)
            # Auto-restore selection if the same module id exists in
            # the new instance (typical re-launch case: same config).
            if prior_module_id is not None and m.get("id") == prior_module_id:
                self._list.setCurrentItem(item)
        self._set_status(f"{len(self._modules)} modules (reconnected)")

    def _on_module_activated(self, item: QListWidgetItem) -> None:
        m = item.data(Qt.ItemDataRole.UserRole) or {}
        module_id   = str(m.get("id", ""))
        module_type = str(m.get("type", ""))
        if not module_id:
            return
        self._switch_to(module_id, module_type)

    def _switch_to(self, module_id: str, module_type: str) -> None:
        # Tear down prior subscription.
        if self._current_sub_id is not None:
            try:
                self.control.call("unsubscribe", sub_id=self._current_sub_id)
            except Exception:
                pass
            if self._current_topic_prefix is not None:
                self.diag.remove_prefix(self._current_topic_prefix)
        # Detach prior widget.
        if self._current_widget is not None:
            self._right.removeWidget(self._current_widget)
            self._current_widget.deleteLater()
            self._current_widget = None

        # Subscribe new.
        try:
            resp = self.control.call(
                "module_subscribe_diag",
                id=module_id,
                topic="",
                hz=int(self._hz.value()),
            )
        except Exception as e:
            self._set_status(f"subscribe error: {e}")
            return
        if resp.get("status") != "ok":
            self._set_status(f"subscribe: {resp}")
            return
        sub_id = int(resp.get("sub_id", -1))
        topic_prefix = str(resp.get("topic_prefix", ""))
        self._current_sub_id = sub_id
        self._current_topic_prefix = topic_prefix
        self._current_module_id = module_id
        self.diag.add_prefix(topic_prefix)

        widget_cls = widget_for(module_type)
        inner = widget_cls(module_id, module_type, parent=self)
        # Decorate every module widget with its educational description panel
        # (layman summary + exact formulas). The card forwards update_payload.
        self._current_widget = wrap_with_description(inner, module_type, parent=self)
        self._right.addWidget(self._current_widget)
        self._right.setCurrentWidget(self._current_widget)
        self._set_status(
            f"subscribed sub_id={sub_id}  prefix={topic_prefix}  hz={self._hz.value()}"
        )

    def _on_payload_main_thread(self, p: DiagPayload) -> None:
        if self._current_widget is None:
            return
        if p.module_id != self._current_module_id:
            return
        if hasattr(self._current_widget, "update_payload"):
            self._current_widget.update_payload(p.tick_id, p.snapshot)

    # ----- shutdown -----

    def closeEvent(self, e):
        try:
            if self._current_sub_id is not None:
                self.control.call("unsubscribe", sub_id=self._current_sub_id)
        except Exception:
            pass
        self.diag.stop()
        self.control.close()
        super().closeEvent(e)

    def _set_status(self, msg: str) -> None:
        self.statusBar().showMessage(msg)


def main() -> None:
    p = argparse.ArgumentParser(description="xaq inspector sidecar")
    p.add_argument("--control-host", default="127.0.0.1")
    p.add_argument("--control-port", type=int, default=7400)
    p.add_argument("--diag-host",    default="127.0.0.1")
    p.add_argument("--diag-port",    type=int, default=7401)
    args = p.parse_args()

    app = QApplication(sys.argv)
    win = InspectorWindow(
        control_host=args.control_host, control_port=args.control_port,
        diag_host=args.diag_host,       diag_port=args.diag_port,
    )
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
