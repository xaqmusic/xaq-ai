"""xaq_voice studio — tune the brain's voice while it is running.

Three panes: the signals the brain publishes, the voice they drive, the master bus.

**The studio talks to the ENGINE, never to the brain.**  The engine is already subscribed
to every module, so a second subscriber would double the sim's per-tick serialisation cost
for nothing — and a leaked subscription costs the sim on every tick for the life of the
process, and they stack across restarts.  Nothing here can leak one.

Threading follows the inspector: the meter stream arrives on a ZMQ thread and crosses to
the GUI thread through a one-signal QObject bridge, and every engine command runs on a
worker so a dead engine cannot freeze the window.  That last part is the inspector's one
known rough edge — it calls its control client inline on the GUI thread despite the
docstring saying not to, and a dead brain hangs it for the full timeout — so it is fixed
here rather than copied.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from PyQt6.QtCore import QObject, Qt, QTimer, QThreadPool, QRunnable, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import (QApplication, QFileDialog, QHBoxLayout, QLabel, QMainWindow,
                             QMessageBox, QPushButton, QSplitter, QStatusBar, QVBoxLayout,
                             QWidget)

from .engine_client import EngineClient, EngineError, StateStream
from .panels.master_panel import MasterPanel
from .panels.sources_panel import SourcesPanel
from .panels.voice_panel import VoicePanel
from .patch_model import PatchModel
from .theme import CRIT, GOOD, INK_MUTED, QSS


class StateBridge(QObject):
    """ZMQ thread → GUI thread.  Never touch a widget from the subscriber thread."""
    state = pyqtSignal(object)


class Job(QRunnable):
    """Run one engine call off the GUI thread."""

    class Signals(QObject):
        done = pyqtSignal(object)
        failed = pyqtSignal(str)

    def __init__(self, fn):
        super().__init__()
        self.fn = fn
        self.signals = Job.Signals()

    @pyqtSlot()
    def run(self) -> None:
        try:
            self.signals.done.emit(self.fn())
        except EngineError as exc:
            self.signals.failed.emit(str(exc))
        except Exception as exc:                       # pragma: no cover - defensive
            self.signals.failed.emit(f"{type(exc).__name__}: {exc}")


class StudioWindow(QMainWindow):
    REDRAW_MS = 100          # meters arrive at 15 Hz; redraw at 10

    def __init__(self, host: str, port: int, patch_path: str | None = None):
        super().__init__()
        self.setWindowTitle("xaq_voice studio")
        self.client = EngineClient(host=host, port=port)
        self.stream = StateStream(host=host, port=port + 1)
        self.pool = QThreadPool.globalInstance()
        self.caps: dict = {}
        self.model = PatchModel(self)
        self.model.set_sender(self._send_ops)
        self._latest_state: dict | None = None
        self._connected = False
        self._patch_path = patch_path
        self._source_signature: tuple = ()

        # Screen-relative, not a fixed 1280: on a 150%-scaled display a hardcoded width
        # put the inspector's title bar off-screen where it could not be grabbed.
        geo = QApplication.primaryScreen().availableGeometry()
        w = min(int(geo.width() * 0.88), 1500)
        h = min(int(geo.height() * 0.88), 950)
        self.resize(w, h)
        self.move(geo.left() + (geo.width() - w) // 2, geo.top() + (geo.height() - h) // 2)
        self.setMinimumSize(720, 480)

        self._build_ui()

        self._bridge = StateBridge()
        self._bridge.state.connect(self._on_state)

        self._redraw = QTimer(self)
        self._redraw.setInterval(self.REDRAW_MS)
        self._redraw.timeout.connect(self._apply_state)
        self._redraw.start()

        QTimer.singleShot(0, self.connect_engine)

    # ------------------------------------------------------------------ ui
    def _build_ui(self) -> None:
        self.setStyleSheet(QSS)

        bar = QWidget()
        bl = QHBoxLayout(bar)
        bl.setContentsMargins(8, 6, 8, 0)
        bl.setSpacing(6)
        self.btn_connect = QPushButton("Reconnect")
        self.btn_connect.clicked.connect(self.connect_engine)
        bl.addWidget(self.btn_connect)
        self.btn_auto = QPushButton("Auto-assign")
        self.btn_auto.setToolTip("Rebuild the patch from the signals the brain is publishing")
        self.btn_auto.clicked.connect(lambda: self.auto_assign(False))
        bl.addWidget(self.btn_auto)
        self.btn_auto_vary = QPushButton("Auto-assign (varied)")
        self.btn_auto_vary.setToolTip("As above, but give each voice a different waveform")
        self.btn_auto_vary.clicked.connect(lambda: self.auto_assign(True))
        bl.addWidget(self.btn_auto_vary)
        bl.addStretch(1)
        self.btn_load = QPushButton("Load…")
        self.btn_load.clicked.connect(self.load_patch)
        bl.addWidget(self.btn_load)
        self.btn_save = QPushButton("Save As…")
        self.btn_save.clicked.connect(self.save_patch)
        bl.addWidget(self.btn_save)

        self.sources = SourcesPanel(self._route_from_source)
        self.voices = VoicePanel(self.model, self.caps, self._mark_dirty)
        self.master = MasterPanel(self.model, self.caps, self._mark_dirty)
        self.voices.structural_change = self._push_whole_patch
        self.master.structural_change = self._push_whole_patch
        self.master.mute.toggled.connect(
            lambda v: self._run(lambda: self.client.call("set_mute", value=bool(v))))
        self.master.tone.toggled.connect(
            lambda v: self._run(lambda: self.client.call("set_tone", value=bool(v))))

        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.sources)
        split.addWidget(self.voices)
        split.addWidget(self.master)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 4)
        split.setStretchFactor(2, 3)

        root = QWidget()
        rl = QVBoxLayout(root)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(4)
        rl.addWidget(bar)
        rl.addWidget(split, 1)
        self.setCentralWidget(root)

        self.setStatusBar(QStatusBar())
        quit_action = QAction("Quit", self)
        quit_action.setShortcut(QKeySequence("Ctrl+Q"))
        quit_action.triggered.connect(self.close)
        self.addAction(quit_action)

    # ------------------------------------------------------------------ jobs
    def _run(self, fn, on_done=None) -> None:
        job = Job(fn)
        if on_done is not None:
            job.signals.done.connect(on_done)
        job.signals.failed.connect(self._on_failed)
        self.pool.start(job)

    def _on_failed(self, msg: str) -> None:
        self._set_status(msg, ok=False)

    def _set_status(self, msg: str, ok: bool = True) -> None:
        self.statusBar().showMessage(msg)
        self.master.status.setText(msg)
        self.master.status.setStyleSheet(f"color: {GOOD if ok else CRIT};")

    # ------------------------------------------------------------------ connect
    def connect_engine(self) -> None:
        self._set_status("connecting…")
        self.client.reconnect()

        def work():
            caps = self.client.hello()
            patch = self.client.get_patch()
            sources = self.client.get_sources()
            return caps, patch, sources

        self._run(work, self._on_connected)

    def _on_connected(self, result) -> None:
        caps, patch, sources = result
        self.caps.clear()
        self.caps.update(caps)
        # The panels were built before the engine said what it supports, so refresh the
        # lists that come from it rather than hardcoding them on the Python side.
        self.voices.caps = self.caps
        self.master.caps = self.caps
        self.master.scale.set_items(self.caps.get("scales", []),
                                    (patch.get("master") or {}).get("scale", ""))
        # BOTH filter panels, not just the master's: a combo cannot select a mode it does
        # not list, so a panel left on its fallback list shows one mode while the patch
        # holds another.
        self.master.filter_panel.set_caps(self.caps)
        self.voices.filter_panel.set_caps(self.caps)
        self.voices.waveform.set_items(self.caps.get("waveforms", []), "square")

        self.model.set_patch(patch)
        self._apply_sources(sources)
        self._connected = True
        self.stream.stop()
        self.stream.start(lambda s: self._bridge.state.emit(s))
        self._set_status(f"connected — {len(self.model.voices())} voice(s)")

        if self._patch_path:
            path, self._patch_path = self._patch_path, None
            self._run(lambda: self.client.load(path), self._on_patch_replaced)

    def _apply_sources(self, modules: list) -> None:
        sig = tuple((m.get("module"), tuple(k.get("key") for k in (m.get("keys") or [])))
                    for m in modules)
        if sig != self._source_signature:
            self._source_signature = sig
            self.sources.rebuild(modules)
        labels = [f"{m.get('module')}.{k.get('key')}"
                  for m in modules for k in (m.get("keys") or [])]
        self.voices.set_sources(labels)
        self.master.set_sources(labels)
        self._rebuild_panels()

    def _rebuild_panels(self) -> None:
        self.voices.rebuild()
        self.master.rebuild()

    # ------------------------------------------------------------------ edits
    def _send_ops(self, ops: list) -> None:
        self._run(lambda: self.client.apply_ops(ops))

    def _mark_dirty(self) -> None:
        pass          # the model flushes on its own timer

    def _push_whole_patch(self) -> None:
        """A structural change (route added or removed) replaces the whole patch."""
        patch = self.model.patch
        self._run(lambda: (self.client.set_patch(patch), self.client.get_patch())[1],
                  self._on_patch_replaced)

    def _on_patch_replaced(self, patch) -> None:
        self.model.set_patch(patch)
        self._rebuild_panels()
        self._set_status("patch updated")

    def _route_from_source(self, path: str) -> None:
        """Double-clicking a signal adds a route from it to the selected voice."""
        voices = self.model.voices()
        if not voices:
            self._set_status("no voice to route into — try Auto-assign", ok=False)
            return
        module, _, key = path.partition(".")
        i = self.voices._index
        voices[i].setdefault("routes", []).append({
            "source": {"module": module, "key": key}, "dest": "cutoff",
            "norm": {"mode": "median_mad", "z_lo": 0.0, "z_hi": 4.0, "ref_key": "",
                     "gate": 1.4, "full": 2.0, "in_lo": 0.0, "in_hi": 1.0,
                     "smooth_ms": 60.0, "window_s": 10.0},
            "depth": 12.0, "curve": 1.0, "invert": False, "enabled": True,
        })
        self._push_whole_patch()
        self._set_status(f"routed {path} → cutoff on {voices[i].get('id', '')}")

    def auto_assign(self, vary: bool) -> None:
        self._run(lambda: self.client.auto_patch(vary), self._on_patch_replaced)

    # ------------------------------------------------------------------ files
    def _default_dir(self) -> str:
        d = Path(__file__).resolve().parents[1] / "xaq_voice" / "patches"
        return str(d if d.is_dir() else Path.cwd())

    def save_patch(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save patch", self._default_dir(),
                                              "Patch (*.json)")
        if not path:
            return
        if not path.endswith(".json"):
            path += ".json"
        self.model.flush()
        # The ENGINE writes the file, from its own serialiser.  The studio never
        # serialises a patch itself, so what is saved is exactly what is sounding.
        self._run(lambda: self.client.save(path),
                  lambda p: self._set_status(f"saved {p}"))

    def load_patch(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Load patch", self._default_dir(),
                                              "Patch (*.json)")
        if path:
            self._run(lambda: self.client.load(path), self._on_patch_replaced)

    # ------------------------------------------------------------------ live
    def _on_state(self, state: dict) -> None:
        self._latest_state = state          # buffer only; redraw is on the timer

    def _apply_state(self) -> None:
        if self._latest_state is None:
            return
        state, self._latest_state = self._latest_state, None
        self.sources.update_values(state.get("sources") or [])
        self.voices.update_state(state)
        self.master.update_state(state)

    # ------------------------------------------------------------------ exit
    def closeEvent(self, e) -> None:
        self.model.flush()
        self._redraw.stop()
        self.stream.stop()
        self.pool.waitForDone(1500)
        self.client.close()
        super().closeEvent(e)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Tune the xaq_voice synth against a running brain.")
    ap.add_argument("--engine-host", default="127.0.0.1")
    ap.add_argument("--engine-port", type=int,
                    default=int(os.environ.get("XAQ_VOICE_PORT", "7460")))
    ap.add_argument("--patch", default=None, help="load this patch once connected")
    args = ap.parse_args(argv)

    app = QApplication(sys.argv)
    win = StudioWindow(args.engine_host, args.engine_port, args.patch)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
