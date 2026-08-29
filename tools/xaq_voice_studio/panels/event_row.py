"""One event: a discrete thing happening in the brain, given a sound.

Events are not modulation. A route maps a *continuous* signal onto a continuous parameter;
an event watches for a **transition** and fires a short gesture. A node earning its place,
a node splitting, a node dying — those are moments, and a moment wants a chirp rather than
a change in pitch.

The trigger matters more than it looks:

* ``true`` is a LEVEL test, not an edge. `baked_now` is already a one-tick pulse, so an
  edge test would need it to go false between two bakes to fire the second one.
* ``increase`` / ``decrease`` watch a counter move. `mitosis_count` only ever climbs, and
  `nodes` falling is a prune — which is the only way to hear one.
* ``rise`` / ``fall`` are the edge tests, for a flag that stays set.
"""
from __future__ import annotations

from typing import Callable

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QHBoxLayout, QPushButton, QWidget

from ..theme import LINE, SURFACE_ALT, SurfaceWidget
from ._controls import LabeledCombo, check


class EventRow(SurfaceWidget):
    removeRequested = pyqtSignal(str)

    def __init__(self, base_path: str, event: dict, caps: dict, sources: list[str],
                 on_set: Callable[[str, object], None], parent: QWidget | None = None):
        super().__init__(parent, SURFACE_ALT)
        self.base = base_path
        self.on_set = on_set

        src = event.get("source") or {}
        src_label = f"{src.get('module', '')}.{src.get('key', '')}".strip(".")

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(6)

        self.enabled = check("", event.get("enabled", True),
                             lambda v: self.on_set(f"{self.base}/enabled", bool(v)))
        self.enabled.setToolTip("Play this event")
        lay.addWidget(self.enabled)

        self.source = LabeledCombo("", sorted(set(sources) | {src_label}), src_label,
                                   label_w=0)
        self.source.currentTextChanged.connect(self._on_source)
        lay.addWidget(self.source, 3)

        self.trigger = LabeledCombo("on", caps.get("triggers", []),
                                    event.get("trigger", "increase"), label_w=16)
        self.trigger.currentTextChanged.connect(
            lambda v: self.on_set(f"{self.base}/trigger", v))
        lay.addWidget(self.trigger, 2)

        self.sound = LabeledCombo("→", caps.get("event_sounds", []),
                                  event.get("sound", "chirp_up"), label_w=14)
        self.sound.currentTextChanged.connect(
            lambda v: self.on_set(f"{self.base}/sound", v))
        lay.addWidget(self.sound, 2)

        rm = QPushButton("✕")
        rm.setFixedWidth(26)
        rm.setToolTip("Remove this event")
        rm.clicked.connect(lambda: self.removeRequested.emit(self.base))
        lay.addWidget(rm)

        self.setStyleSheet(f"EventRow {{ border: 1px solid {LINE}; border-radius: 4px; }}")

    def set_sources(self, sources: list[str]) -> None:
        cur = self.source.value()
        self.source.set_items(sorted(set(sources) | {cur}), cur)

    def set_caps(self, caps: dict) -> None:
        if caps.get("triggers"):
            self.trigger.set_items(caps["triggers"], self.trigger.value())
        if caps.get("event_sounds"):
            self.sound.set_items(caps["event_sounds"], self.sound.value())

    def _on_source(self, label: str) -> None:
        module, _, key = label.partition(".")
        self.on_set(f"{self.base}/source/module", module)
        self.on_set(f"{self.base}/source/key", key)
