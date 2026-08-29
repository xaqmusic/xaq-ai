"""The patch, as the studio holds it: a plain dict plus pointer bookkeeping.

Deliberately NOT a typed mirror of the C++ ``Patch``.  The engine owns the schema; a
second definition here would be a second thing to keep in step, and it would go stale in
exactly the way that produces a Save which quietly drops a field.  The studio edits the
dict it was handed and sends JSON-pointer ops back.

Writes are **coalesced**.  A slider drag emits a value on every pixel, and one round trip
per pixel would flood the socket and stutter the UI; instead the newest value per path is
kept and flushed on a timer.  Newest-wins is the right merge for a drag — an intermediate
value has no meaning once the operator has moved past it.
"""
from __future__ import annotations

from typing import Any, Callable

from PyQt6.QtCore import QObject, QTimer, pyqtSignal


def pointer_get(patch: dict, path: str, default: Any = None) -> Any:
    """Read a JSON-pointer path out of the patch dict."""
    node: Any = patch
    for tok in path.strip("/").split("/"):
        if tok == "":
            continue
        tok = tok.replace("~1", "/").replace("~0", "~")
        if isinstance(node, list):
            try:
                node = node[int(tok)]
            except (ValueError, IndexError):
                return default
        elif isinstance(node, dict):
            if tok not in node:
                return default
            node = node[tok]
        else:
            return default
    return node


def pointer_set(patch: dict, path: str, value: Any) -> bool:
    """Write locally so the UI stays consistent before the engine's reply arrives."""
    toks = [t for t in path.strip("/").split("/") if t != ""]
    if not toks:
        return False
    node: Any = patch
    for tok in toks[:-1]:
        tok = tok.replace("~1", "/").replace("~0", "~")
        if isinstance(node, list):
            try:
                node = node[int(tok)]
            except (ValueError, IndexError):
                return False
        elif isinstance(node, dict) and tok in node:
            node = node[tok]
        else:
            return False
    last = toks[-1].replace("~1", "/").replace("~0", "~")
    if isinstance(node, list):
        try:
            node[int(last)] = value
            return True
        except (ValueError, IndexError):
            return False
    if isinstance(node, dict) and last in node:
        node[last] = value
        return True
    return False


class PatchModel(QObject):
    """Holds the patch and batches edits toward the engine."""

    changed = pyqtSignal()          # the patch was replaced wholesale
    edited  = pyqtSignal(str)       # one path was edited locally

    FLUSH_MS = 50

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._patch: dict = {"version": 1, "master": {}, "voices": []}
        self._pending: dict[str, Any] = {}
        self._send: Callable[[list[dict]], None] | None = None
        self._timer = QTimer(self)
        self._timer.setInterval(self.FLUSH_MS)
        self._timer.timeout.connect(self.flush)

    # -- wiring --

    def set_sender(self, send: Callable[[list[dict]], None] | None) -> None:
        self._send = send

    # -- whole patch --

    @property
    def patch(self) -> dict:
        return self._patch

    def set_patch(self, patch: dict) -> None:
        self._patch = patch or {"version": 1, "master": {}, "voices": []}
        self._pending.clear()
        self.changed.emit()

    def voices(self) -> list[dict]:
        v = self._patch.get("voices")
        return v if isinstance(v, list) else []

    def master(self) -> dict:
        m = self._patch.get("master")
        return m if isinstance(m, dict) else {}

    # -- edits --

    def get(self, path: str, default: Any = None) -> Any:
        return pointer_get(self._patch, path, default)

    def set(self, path: str, value: Any) -> None:
        """Edit one field.  Applied locally at once, sent to the engine on the timer."""
        if pointer_get(self._patch, path, object()) == value:
            return
        pointer_set(self._patch, path, value)
        self._pending[path] = value
        self.edited.emit(path)
        if not self._timer.isActive():
            self._timer.start()

    def flush(self) -> None:
        self._timer.stop()
        if not self._pending or self._send is None:
            self._pending.clear()
            return
        ops = [{"path": p, "value": v} for p, v in self._pending.items()]
        self._pending.clear()
        self._send(ops)
