"""ThingBelief dashboard — is the thing the duck is walking to still where it believes? (design doc §17.113, 2026-10-12)

The seek loop's target is a PLACE: a sighting fixes it by dead reckoning, and the arrival fires the stop and the outcome
loop's request without anything checking the thing is still there.  ThingBelief holds a belief about the THING: each
tick CloudMap reports a small thing at the target (a confirmation) or free rays through the target's footprint below the
thing's top (the place seen empty, a refutation).  Since the target was set, presence = (1 + confirming) / (2 + confirming
+ refuting); a chased thing is present by being chased.  Passive in the ★ stacks: nothing reads it yet, the host logs it.

snapshot (diag_snapshot): presence, confirming, refuting, since (ticks since the target was set), have, chasing

Panels:
  * A banner: no target, chasing, or the presence belief with its evidence.
  * presence, confirming and refuting ticks over time.
  * Readout.
"""
from __future__ import annotations

from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget
from PyQt6.QtCore import Qt, QTimer

from ._kv_readout import AMBER, GREEN, GREY, RED, Banner, KVReadout, flag, num
from ._multi_series import MultiSeriesPlot, Series


class _State(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("no target")
        layout.addWidget(self._banner)
        self._latest = None
        self._dirty = False
        self._timer = QTimer(self)
        self._timer.setInterval(150)
        self._timer.timeout.connect(self._flush)
        self._timer.start()

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        s = self._latest
        if not s.get("have"):
            self._banner.set("no target — the seek loop holds nothing", GREY)
            return
        if s.get("chasing"):
            self._banner.set("CHASING — a chased thing is present by being chased", GREEN)
            return
        p = num(s, "presence", 1.0)
        colour = GREEN if p >= 0.6 else (AMBER if p >= 0.4 else RED)
        self._banner.set(f"presence {p:.2f} — {int(num(s, 'confirming'))} ticks seen there, "
                         f"{int(num(s, 'refuting'))} seen empty", colour)


def _rows(s: dict):
    since = int(num(s, "since"))
    return [
        ("has a target", flag(s, "have")),
        ("chasing", flag(s, "chasing")),
        ("presence", f"{num(s, 'presence', 1.0):.3f}   (1 + confirming) / (2 + confirming + refuting)"),
        ("confirming ticks", f"{int(num(s, 'confirming'))}"),
        ("refuting ticks", f"{int(num(s, 'refuting'))}"),
        ("target set", f"{since} ticks ago ({since / 50.0:.1f} s)"),
    ]


class ThingBeliefInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)
        self._state = _State()
        outer.addWidget(self._state)
        self._readout = KVReadout(_rows)
        self._presence = MultiSeriesPlot(
            [Series("presence", "presence", (27, 175, 122), width=2.0)],
            title="Is the thing still there?",
            y_label="belief",
        )
        self._evidence = MultiSeriesPlot(
            [
                Series("confirming", "ticks seen there", (74, 144, 217), width=1.5),
                Series("refuting", "ticks seen empty", (224, 85, 85), width=1.5),
            ],
            title="The evidence since the target was set",
            y_label="ticks",
        )
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._presence)
        top.addWidget(self._readout)
        top.setSizes([600, 420])
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._evidence)
        v.setSizes([360, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._state.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._presence.update_payload(snapshot)
        self._evidence.update_payload(snapshot)
