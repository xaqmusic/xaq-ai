"""Run-and-Tumble V2 dashboard — the clean-room KF-ladder taxis (RunTumbleNavV2).

Same E. coli methylation reflex as the v1 RunTumbleNav (RUN + modulate p(tumble) by the
scent-vs-methylation prediction error), so the STATE / p(tumble) gauge / scent-baseline
series panels are REUSED verbatim.  The "obvious update" for the versioned module is the
KF-ladder readout that replaces v1's `speak` bootstrap denom:

  * KF4 — a stationary NOISE FLOOR (`nfloor`) learned from the sensor's own variance, and a
    learned speed scale (`vscale`) so the tumble threshold is derived from the body's own
    dynamics rather than a hand constant.
  * KF2 — efference-matched stuck detection (the same `vscale`).
  * KF6 — a directional belief (`dir_mu` = believed up-gradient heading) with `reorienting`
    when committing to reach it before the next tumble.
  * KF3 — `muted` when another loop / a reflex holds the motor bus and klino coasts.
"""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from .run_tumble_inspector import (
    _ScentBaselineSeries,
    _StatePanel,
    _TumbleGauge,
)


class _V2Counts(QWidget):
    """Lifetime counts + the V2 KF-ladder health readout."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet("color: #ddd; font-family: Monospace; font-size: 12px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._lbl, 1)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        s = snapshot
        muted = bool(s.get("muted", False))
        reorient = bool(s.get("reorienting", False))
        rows = [
            ("runs",     f"{int(s.get('runs', 0) or 0):d}"),
            ("tumbles",  f"{int(s.get('tumbles', 0) or 0):d}"),
            ("forced",   f"{int(s.get('forced', 0) or 0):d}"),
            ("f_in_turn", f"{int(s.get('forced_in_turn', 0) or 0):d}"),
            ("", ""),
            ("KF4 nfloor", f"{float(s.get('nfloor', 0.0) or 0.0):.4f}"),   # stationary noise floor
            ("KF2/4 vscale", f"{float(s.get('vscale', 0.0) or 0.0):.3f}"),  # learned speed scale
            ("run_up/dn", f"{float(s.get('run_len_up', 0.0) or 0.0):.1f} / {float(s.get('run_len_down', 0.0) or 0.0):.1f}"),
            ("turn_frac", f"{float(s.get('turn_frac', 0.0) or 0.0):.3f}"),   # K1 turn burn
            ("KF6 dir_mu°", f"{_deg(s.get('dir_mu', 0.0)):.0f}"),            # believed up-gradient heading
            ("reorienting", "YES" if reorient else "no"),
            ("KF3 muted",  "MUTED" if muted else "live"),                    # another loop has the bus
        ]
        self._lbl.setText("\n".join(f"{k:>13}: {v}" if k else "" for k, v in rows))


def _deg(rad) -> float:
    import math
    try:
        return math.degrees(float(rad or 0.0))
    except (TypeError, ValueError):
        return 0.0


class RunTumbleV2Inspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})  —  KF-ladder taxis")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._state = _StatePanel()
        self._gauge = _TumbleGauge()
        self._counts = _V2Counts()
        self._series = _ScentBaselineSeries()

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._state)
        top.addWidget(self._gauge)
        top.addWidget(self._counts)
        top.setSizes([280, 340, 300])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([300, 260])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._state.update_payload(snapshot)
        self._gauge.update_payload(snapshot)
        self._counts.update_payload(snapshot)
        self._series.update_payload(snapshot)
