"""FaderController dashboard.

Three panels:
  * Alpha trajectory — alpha (smoothed) + alpha_target (raw) + alpha_long_ema
    (slow EMA, the homeostatic envelope).  At a glance: is alpha tracking,
    saturating, or oscillating?
  * Drive components — surprise_scalar, familiarity_scalar, last_boredom.
    These are the inputs alpha_target is computed from.  Lets you see
    which signal is dominating the blend at any moment.
  * Learning state — learned_alpha_setpoint, reward_ema, last_reward_signal.
    Shows the slow-time-scale "is the brain earning its α?" loop.

Also a small text readout: publish_count, learned_warmed_up flag.
"""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


class _StatusReadout(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self._lbl = QLabel("—")
        self._lbl.setStyleSheet(
            "color: #ddd; font-family: Monospace; font-size: 12px;")
        self._lbl.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._lbl, 1)

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        rows = [
            ("alpha",          snapshot.get("alpha",          0.0)),
            ("alpha_target",   snapshot.get("alpha_target",   0.0)),
            ("alpha_long_ema", snapshot.get("alpha_long_ema", 0.0)),
            ("setpoint",       snapshot.get("learned_alpha_setpoint", 0.0)),
            ("reward_ema",     snapshot.get("reward_ema",     0.0)),
            ("warmed_up",      snapshot.get("learned_warmed_up", False)),
            ("publish_count",  snapshot.get("publish_count",  0)),
        ]
        lines = []
        for k, v in rows:
            if isinstance(v, bool):
                lines.append(f"{k:>14}: {'yes' if v else 'no'}")
            elif isinstance(v, float):
                lines.append(f"{k:>14}: {v:+.4f}")
            else:
                try:
                    lines.append(f"{k:>14}: {int(v)}")
                except (TypeError, ValueError):
                    lines.append(f"{k:>14}: ?")
        self._lbl.setText("\n".join(lines))


class FaderInspector(QWidget):
    def __init__(self, module_id: str, module_type: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._alpha = MultiSeriesPlot(
            [
                Series("alpha",          "alpha",      (120, 220, 255), width=2.5),
                Series("alpha_target",   "α target",   (255, 180,  60),
                       width=1.0, style=Qt.PenStyle.DashLine),
                Series("alpha_long_ema", "α long EMA", (200, 200, 200),
                       width=1.0, style=Qt.PenStyle.DashLine),
            ],
            title="α(t) trajectory",
            y_label="alpha [0,1]",
        )

        self._drivers = MultiSeriesPlot(
            [
                Series("surprise_scalar",    "surprise",    (255, 100, 100), width=1.5),
                Series("familiarity_scalar", "familiarity", (100, 255, 200), width=1.5),
                Series("last_boredom",       "boredom",     (255, 200,  80), width=1.5),
            ],
            title="α drivers",
            y_label="value",
        )

        self._learning = MultiSeriesPlot(
            [
                Series("learned_alpha_setpoint", "setpoint",    (180, 220, 255), width=2.0),
                Series("reward_ema",             "reward EMA",  (255, 215,  90), width=1.5),
                Series("last_reward_signal",     "last reward", (200, 100, 100), width=1.0),
            ],
            title="Learning state",
            y_label="value",
        )

        self._readout = _StatusReadout()

        bot = QSplitter(Qt.Orientation.Horizontal)
        bot.addWidget(self._learning)
        bot.addWidget(self._readout)
        bot.setSizes([720, 280])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(self._alpha)
        v.addWidget(self._drivers)
        v.addWidget(bot)
        v.setSizes([300, 240, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._alpha.update_payload(snapshot)
        self._drivers.update_payload(snapshot)
        self._learning.update_payload(snapshot)
        self._readout.update_payload(snapshot)
