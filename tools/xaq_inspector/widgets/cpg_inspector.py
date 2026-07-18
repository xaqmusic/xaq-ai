"""CPGOscillator dashboard — visualises the spinal CPG's competence
gate, amplitude blend, and per-joint bias breakdown.

Three panels:

  * Gate & EMA timeline — competence_gate (red, [0,1]) and the slow
    EMA of NeuroState.reward_signal it derives from (blue, around 0).
    Shows whether sustained standing competence is being accumulated;
    the gate's slow climb is the substrate's self-regulating mechanism
    for switching from cold-start standing-bias to walking rhythm.

  * Amplitude blend timeline — last_walking_amp (green) and
    last_standing_factor (purple) over time.  Visualises the gated
    handoff: at cold start standing_factor dominates and walking_amp
    is at floor; as gate opens they swap.

  * Per-joint bias breakdown — current-tick stacked bars per joint
    showing standing component, walking component, and final blended
    output.  Read at a glance whether the CPG is asymmetric (one leg
    fighting another) or co-contracting cleanly.  Joint labels pulled
    from output_topics so the layout reflects whatever the config wired.

Source data: OgmaBrain::get_module_metrics() exposes per-tick cached
state on the CPGOscillator instance (see CPGOscillator.hpp accessors).
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._multi_series import MultiSeriesPlot, Series


# ---------------------------------------------------------------------------
# Per-joint bias breakdown — grouped bars
# ---------------------------------------------------------------------------

class _BiasBars(QWidget):
    """Per-joint stacked bars: standing bias, walking bias, blended output.

    Read joint order from output_topics so the X axis reflects the
    config (typically leg-grouped: fl_hip1, fl_hip2, fl_knee, …).
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Per-joint bias breakdown — awaiting payload")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=False, y=True, alpha=0.25)
        self._plot.setMouseEnabled(x=False, y=True)
        self._plot.setLabel("left", "accel (clamped to ±1)")
        self._plot.setLabel("bottom", "joint")
        # Three bar layers side-by-side per joint group.
        self._bar_walking = pg.BarGraphItem(
            x=[], height=[], width=0.25,
            brush=pg.mkBrush(140, 200, 120, 200),
            name="walking",
        )
        self._bar_standing = pg.BarGraphItem(
            x=[], height=[], width=0.25,
            brush=pg.mkBrush(200, 140, 220, 200),
            name="standing",
        )
        self._bar_blended = pg.BarGraphItem(
            x=[], height=[], width=0.25,
            brush=pg.mkBrush(240, 220, 100, 230),
            name="blended",
        )
        self._plot.addItem(self._bar_walking)
        self._plot.addItem(self._bar_standing)
        self._plot.addItem(self._bar_blended)
        # Reference line at y=0
        self._zero_line = pg.InfiniteLine(pos=0.0, angle=0,
                                          pen=pg.mkPen(180, 180, 180, 80))
        self._plot.addItem(self._zero_line)
        self._plot.setYRange(-1.05, 1.05)
        layout.addWidget(self._plot)

        self._latest: Optional[dict] = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(150)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._latest = snapshot
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        snap = self._latest

        topics = snap.get("output_topics") or []
        bw = snap.get("last_bias_walking")  or []
        bs = snap.get("last_bias_standing") or []
        bb = snap.get("last_blended")       or []
        n = min(len(topics), len(bw), len(bs), len(bb))
        if n == 0:
            self._title.setText("Per-joint bias breakdown — no joints yet")
            return

        labels = []
        for t in topics[:n]:
            # action.<joint> → <joint>
            s = str(t)
            labels.append(s.split(".", 1)[-1] if "." in s else s)

        xs = np.arange(n, dtype=np.float32)
        self._bar_walking.setOpts(
            x=(xs - 0.25).tolist(), height=[float(v) for v in bw[:n]])
        self._bar_standing.setOpts(
            x=xs.tolist(),           height=[float(v) for v in bs[:n]])
        self._bar_blended.setOpts(
            x=(xs + 0.25).tolist(),  height=[float(v) for v in bb[:n]])

        # X-axis tick labels
        ax = self._plot.getPlotItem().getAxis("bottom")
        ax.setTicks([list(zip(xs.tolist(), labels))])

        gate = float(snap.get("competence_gate", 0.0) or 0.0)
        wamp = float(snap.get("last_walking_amp", 0.0) or 0.0)
        sfac = float(snap.get("last_standing_factor", 0.0) or 0.0)
        self._title.setText(
            f"Per-joint bias breakdown   "
            f"gate={gate:.3f}   walking_amp={wamp:.3f}   standing_factor={sfac:.3f}   "
            f"(green=walking, purple=standing, yellow=blended)"
        )


# ---------------------------------------------------------------------------
# Top-level CPGInspector
# ---------------------------------------------------------------------------

class CPGInspector(QWidget):
    """Three stacked panels: gate timeline, amplitude blend, per-joint bias.

    Inspector contract: __init__(module_id, module_type, parent) and
    update_payload(tick_id, snapshot).  Snapshot comes from
    CPGOscillator::snapshot_state().
    """

    def __init__(self, module_id: str, module_type: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        layout.addWidget(header)

        self._splitter = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(self._splitter, 1)

        # Panel 1 — gate + the two competence signals it derives from.
        # gate (red, primary): the value driving the standing/walking
        # blend.  ema(fused_tle) (cyan): the substrate's slow-EMA
        # prediction error across all modalities; lower = more
        # competent.  ema(reward_signal) (faded blue): old dopamine-
        # based signal, retained for telemetry but NO LONGER drives
        # the gate (it coupled to prop-induced rewards).
        self._gate_plot = MultiSeriesPlot(
            [
                Series(key="competence_gate",
                       label="gate [0..1]",
                       color=(220, 100, 100), width=2.0),
                Series(key="ema_fused_tle",
                       label="ema(fused_tle)",
                       color=(100, 220, 200), width=1.8),
                Series(key="ema_reward_signal",
                       label="ema(reward) (info)",
                       color=(80, 120, 180), width=1.0,
                       style=Qt.PenStyle.DashLine),
            ],
            title="Competence gate timeline",
            y_label="value",
            buffer_size=600,
        )
        self._splitter.addWidget(self._gate_plot)

        # Panel 2 — amplitude blend.  Phase deliberately omitted: it
        # sweeps 0..2π every period_ticks (60 by default) and its range
        # dwarfs the amplitude scalars [0..1], so an auto-Y plot would
        # squash walking_amp + standing_factor flat against the X axis.
        # Per-joint bias bars (panel 3) already show the phase's
        # downstream effect.
        self._amp_plot = MultiSeriesPlot(
            [
                Series(key="last_walking_amp",
                       label="walking_amp",
                       color=(140, 220, 130), width=2.0),
                Series(key="last_standing_factor",
                       label="standing_factor",
                       color=(220, 140, 240), width=2.0),
            ],
            title="Amplitude blend (walking_amp vs standing_factor)",
            y_label="amplitude",
            buffer_size=600,
        )
        self._splitter.addWidget(self._amp_plot)

        # Panel 3 — per-joint bias breakdown
        self._bias_bars = _BiasBars()
        self._splitter.addWidget(self._bias_bars)

        self._splitter.setSizes([200, 200, 260])

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._gate_plot.update_payload(snapshot)
        self._amp_plot.update_payload(snapshot)
        self._bias_bars.update_payload(snapshot)
