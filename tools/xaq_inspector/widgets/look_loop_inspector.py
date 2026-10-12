"""LookLoop dashboard — the stop-and-look as a loop: will standing still here pay? (design doc §17.112, 2026-10-12)

The host used to decide when the duck stops to look (a timer, an arrival) and when the look ends.  LookLoop owns the error
those scaffolds stand in for.  Per context (a context EPM's winner over [the place map's TLE, the seek need, the outcome
need]) it keeps a prediction of what a stop delivers, and hands the arbiter a NEED: the fifth policy, G_look.

Two need modes (need_mode):
  * "surprise" — a stop delivers the cloud's surprise: the integral of new_fraction (the share of voxels touched in the
    last second never seen before) over the stop, a running mean and variance per context.
  * "yield" (the ★ stacks) — a stop PAYS when a seek target appears or a skill is requested during it: a Beta posterior
    per context (paid_a, unpaid_b); during a stop, the probability a payoff is still to come, given none yet.

snapshot (diag_snapshot): need, prediction, in_stop, new_fraction, peak, episodes, last_realized, last_predicted,
  mean_abs_error, cells {"ctx": {n, mean, paid_a, unpaid_b}}, need_mode, paid, brier, in_stop_paid, commit

Panels:
  * Per context: the predicted payoff (yield: the Beta mean a / (a + b)) or the mean surprise a stop resolved
    (surprise), the bar labelled with its stops; a banner says whether a stop is on and whether it has paid.
  * need, prediction, in a stop, and the cloud's surprise over time.
  * Readout.
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QSplitter, QVBoxLayout, QWidget

from ._kv_readout import AMBER, BLUE, GREEN, GREY, Banner, KVReadout, flag, num
from ._multi_series import MultiSeriesPlot, Series


def _cell_items(s: dict) -> list[tuple[str, dict]]:
    cells = s.get("cells") or {}
    return sorted(cells.items(), key=lambda kv: int(kv[0]) if str(kv[0]).lstrip("-").isdigit() else 0)


class _Cells(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("—")
        layout.addWidget(self._banner)
        self._plot = pg.PlotWidget(title="what a stop is predicted to deliver, per context")
        self._plot.setBackground("k")
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.showGrid(x=False, y=True, alpha=0.2)
        self._bars = pg.BarGraphItem(x=[0], height=[0], width=0.6, brushes=[pg.mkBrush(85, 85, 85)])
        self._plot.addItem(self._bars)
        layout.addWidget(self._plot, 1)
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
        yield_mode = s.get("need_mode") == "yield"
        if s.get("in_stop"):
            if yield_mode and s.get("in_stop_paid"):
                self._banner.set("IN A STOP — it has paid (a target appeared or a skill was asked for)", GREEN)
            else:
                self._banner.set(f"IN A STOP — need {num(s, 'need'):.2f}"
                                 + ("  ·  committed" if s.get("commit") else ""), AMBER)
        else:
            self._banner.set(f"walking — the need a stop here would earn: {num(s, 'need'):.2f}", GREY)
        items = _cell_items(s)
        if not items:
            self._bars.setOpts(x=[0], height=[0], brushes=[pg.mkBrush(85, 85, 85)])
            return
        xs, hs, brushes, ticks = [], [], [], []
        for i, (k, c) in enumerate(items):
            c = c or {}
            if yield_mode:
                a, b = float(c.get("paid_a", 1.0)), float(c.get("unpaid_b", 1.0))
                h = a / (a + b) if a + b > 0 else 0.5
            else:
                h = float(c.get("mean", 0.0))
            xs.append(i)
            hs.append(h)
            brushes.append(pg.mkBrush(240, 220, 90, 220) if int(c.get("n", 0)) > 0 else pg.mkBrush(85, 85, 85))
            ticks.append((i, f"context {k} · {int(c.get('n', 0))} stops"))
        self._bars.setOpts(x=xs, height=hs, brushes=brushes)
        self._plot.getAxis("bottom").setTicks([ticks])
        self._plot.setXRange(-0.6, len(xs) - 0.4)
        self._plot.setLabel("left", "P(a stop pays)" if yield_mode else "surprise resolved (s)")
        if yield_mode:
            self._plot.setYRange(0, 1)
        else:
            self._plot.enableAutoRange(axis="y")


def _rows(s: dict):
    yield_mode = s.get("need_mode") == "yield"
    rows = [
        ("need mode", str(s.get("need_mode", "—"))),
        ("in a stop", flag(s, "in_stop") + ("   (paid)" if s.get("in_stop_paid") else "")),
        ("committed", flag(s, "commit")),
        ("need", f"{num(s, 'need'):.3f}   (the arbiter's input)"),
        ("prediction", f"{num(s, 'prediction'):.3f}"),
        ("stops", f"{int(num(s, 'episodes'))}" + (f"   paid {int(num(s, 'paid'))}" if yield_mode else "")),
    ]
    if yield_mode:
        rows.append(("calibration (Brier)", f"{num(s, 'brier', 0.25):.3f}   (0.25 = a coin; lower is better)"))
    else:
        rows += [
            ("cloud surprise now", f"{num(s, 'new_fraction'):.3f}"),
            ("last stop", f"realized {num(s, 'last_realized'):.2f} s, predicted {num(s, 'last_predicted'):.2f} s"),
            ("mean |error|", f"{num(s, 'mean_abs_error'):.3f} s"),
            ("running peak", f"{num(s, 'peak'):.3f} s"),
        ]
    rows.append(("", ""))
    for k, c in _cell_items(s):
        c = c or {}
        if yield_mode:
            a, b = float(c.get("paid_a", 1.0)), float(c.get("unpaid_b", 1.0))
            rows.append((f"context {k}", f"{int(c.get('n', 0))} stops, P(pays) {a / (a + b):.2f}  (a {a:.0f}, b {b:.0f})"))
        else:
            rows.append((f"context {k}", f"{int(c.get('n', 0))} stops, mean {float(c.get('mean', 0.0)):.2f} s"))
    return rows


class LookLoopInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)
        self._cells = _Cells()
        self._readout = KVReadout(_rows)
        self._series = MultiSeriesPlot(
            [
                Series("need", "need", (240, 220, 90), width=2.0),
                Series("prediction", "prediction", (74, 144, 217), width=1.2),
                Series("in_stop", "in a stop", (27, 175, 122), width=1.2),
                Series("new_fraction", "cloud surprise", (224, 85, 85), width=1.0),
            ],
            title="The look's need, and the stops it earns",
            y_label="value",
        )
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._cells)
        top.addWidget(self._readout)
        top.setSizes([600, 420])
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([400, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._cells.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._series.update_payload(snapshot)
