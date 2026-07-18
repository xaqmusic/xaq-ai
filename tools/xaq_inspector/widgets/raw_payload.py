"""Raw payload viewer — pretty-printed JSON of the latest snapshot.

Default widget for any module type without a specialised view.  Always
shows *something*, so connecting a sub to a module type the inspector
doesn't yet know how to render still surfaces the data.
"""
from __future__ import annotations

import json

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QPlainTextEdit, QVBoxLayout, QWidget, QLabel


class RawPayloadView(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        self._latest: dict | None = None
        self._tick = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self._header = QLabel(f"{module_id}  ({module_type})")
        self._header.setStyleSheet("color: #ddd; font-weight: bold;")
        layout.addWidget(self._header)

        self._text = QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QFont("Monospace", 9))
        self._text.setStyleSheet("background:#101216; color:#cdd;")
        layout.addWidget(self._text, 1)

        # Throttle the JSON re-render — no point re-pretty-printing 60×/sec
        # when the user can read once-per-200-ms just fine.
        self._refresh = QTimer(self)
        self._refresh.setInterval(200)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    # Called from the GUI thread (via signal) for each diag payload.
    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        self._latest = snapshot
        self._tick = tick_id

    def _flush(self) -> None:
        if self._latest is None:
            return
        try:
            text = json.dumps(self._latest, indent=2, sort_keys=True)
        except Exception as e:
            text = f"<<unserialisable: {e!r}>>"
        self._header.setText(
            f"{self.module_id}  ({self.module_type})   tick {self._tick}"
        )
        # Preserve scroll position roughly — only update if changed to
        # avoid kicking the user's text-cursor selection.
        if self._text.toPlainText() != text:
            self._text.setPlainText(text)
