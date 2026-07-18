"""EFE Arbiter dashboard — the Cell's L2 active-inference policy selection.

Individually-competent loops compete for the body:
  * klino   — the NEAR-FOOD CLOSER (E. coli methylation chemotaxis).
  * planner — the FAR-FIELD SEARCHER/router (place-graph value iteration).
  * play    — the epistemic GROWER (task #33), present only when wired + weighted (play_active).

The policy set is DATA-DRIVEN: every panel iterates the `POLICIES` roster (below), rendering the
subset actually racing this tick, so a policy that isn't active simply isn't drawn and a future
policy appears by adding one `_Policy` row — no panel code changes.

The arbiter runs in one of two scoring modes (`scoring_mode`):

  * "efe"  (the precision refactor, doctrine §2.2/§2.3) — each policy is scored as an
    EXPECTED FREE ENERGY  G = pragmatic + epistemic, with the two PRAGMATIC terms in
    SHARED UNITS (hunger × reach-probability) so the klino/planner scale mismatch is gone
    by construction (no cede, no plan_peak, no max-of-three):
        g_prag_klino   = hunger · cap_klino          (reach-prob = eat-calibrated SENSORY precision)
        g_prag_planner = hunger · clamp(plan_value)   (reach-prob = discounted route value γ^hops)
        g_epist_klino  = (1−hunger) · z-spike         (scent rising = "food this way")
        g_epist_planner= (1−hunger) · plan_novelty    (frontier map-uncertainty to resolve)
    `hunger` is the PREFERENCE PRECISION: hungry → pragmatic (exploit) dominates; sated →
    epistemic (explore) takes over. No tuned λ.

  * "value_race" (legacy default) — klino = MAX(z-spike, proximity level), planner = a
    route LEVEL ceded by proximity. The asymmetric-normalisation predecessor.

Winner-take-all with an ADAPTIVE hysteresis margin (= hysteresis_k · running_std of the score
gap). The winner gets a hard MotorBus gain 1.0, the loser 0.0.

Panels:
  * WINNER    — big KLINO/PLANNER readout, the two hard gain faders, the scoring-mode badge.
  * EFE RACE  — in efe, STACKED bars: pragmatic (solid) + epistemic (translucent) for each
                policy, total height = G; in value_race, the v_klino/v_planner bars.
  * PRECISION — cap_klino (SENSORY precision) vs plan_precision (MODEL precision) meters, and
                the pragmatic↔epistemic (exploit↔explore) balance driven by hunger.
  * TIMESERIES— rolling G_klino / G_planner (or v_klino / v_planner) with the ±margin band.
"""
from __future__ import annotations

from collections import deque
from typing import NamedTuple

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QPainter
from PyQt6.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QSplitter,
    QVBoxLayout,
    QWidget,
)


_KLINO_COL = (80, 224, 112)     # green  — the CLOSER (sensory)
_PLANNER_COL = (120, 160, 255)  # blue   — the SEARCHER (model)
_PLAY_COL = (205, 130, 255)     # violet — the GROWER (epistemic map growth, task #33)
_EXPLOIT_COL = (240, 170, 70)   # amber  — pragmatic / exploit (hunger)
_EXPLORE_COL = (90, 200, 220)   # cyan   — epistemic / explore (1−hunger)
_DIM = (85, 85, 85)


def _f(snapshot: dict, key: str, default: float = 0.0) -> float:
    return float(snapshot.get(key, default) or 0.0)


class _Policy(NamedTuple):
    name: str                    # snapshot key stem: g_prag_<name>, g_epist_<name>, G_<name>, v_<name>, gain_<name>
    label: str                   # display name
    color: tuple[int, int, int]
    always: bool                 # True = a base loop, always in the race; False = shown only when present_key is set
    present_key: str             # snapshot bool that gates a conditional policy into the race


# The policy roster, in the C++ arbiter's winner-index order (0=klino, 1=planner, 2=play). Every
# panel below iterates the PRESENT subset (`_present`), so a policy whose data is absent this tick
# is simply not drawn — and a FUTURE policy shows up automatically once it is wired: add ONE _Policy
# row here (its stem matching the arbiter's g_prag_<name>/g_epist_<name>/G_<name>/v_<name>/gain_<name>
# diag keys, plus the bool the arbiter publishes to say it is active) and no panel code changes.
POLICIES: list[_Policy] = [
    _Policy("klino",   "KLINO",   _KLINO_COL,   True,  ""),
    _Policy("planner", "PLANNER", _PLANNER_COL, True,  ""),
    _Policy("play",    "PLAY",    _PLAY_COL,    False, "play_active"),
]


def _present(snapshot: dict) -> list[_Policy]:
    """The policies actually in the race this tick: base loops + any wired-and-active extras."""
    return [p for p in POLICIES if p.always or bool(snapshot.get(p.present_key))]


def _winner_name(snapshot: dict) -> str:
    w = int(snapshot.get("winner", 0) or 0)
    return POLICIES[w].name if 0 <= w < len(POLICIES) else ""


class _WinnerPanel(QWidget):
    """Big winner readout, the two hard gain faders, and the scoring-mode badge."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self._mode = QLabel("mode —")
        self._mode.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._mode.setStyleSheet("color: #999; font-size: 11px; font-weight: bold;"
                                 " letter-spacing: 1px;")
        layout.addWidget(self._mode)

        self._winner = QLabel("—")
        self._winner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._winner.setStyleSheet("color: #888; font-size: 34px; font-weight: bold;")
        layout.addWidget(self._winner, 1)

        # one hard-gain fader per DECLARED policy; hidden until that policy is present in the
        # race (so a future policy's fader appears automatically — see POLICIES).
        faders = QHBoxLayout()
        self._faders: dict[str, dict] = {}
        for p in POLICIES:
            f = self._make_fader(p.name, p.color)
            self._faders[p.name] = f
            faders.addWidget(f["box"])
        layout.addLayout(faders)

    def _make_fader(self, name: str, col: tuple[int, int, int]) -> dict:
        box = QWidget()
        v = QVBoxLayout(box)
        v.setContentsMargins(2, 2, 2, 2)
        lbl = QLabel(f"gain {name}")
        lbl.setStyleSheet("color: #bbb; font-size: 10px;")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        bar = QProgressBar()
        bar.setRange(0, 1000)
        bar.setTextVisible(True)
        bar.setFormat("0.00")
        hexcol = "#%02x%02x%02x" % col
        bar.setStyleSheet(
            "QProgressBar { background: #1a1a1a; border: 1px solid #333; height: 22px;"
            " text-align: center; color: #fff; }"
            " QProgressBar::chunk { background: %s; }" % hexcol
        )
        v.addWidget(lbl)
        v.addWidget(bar)
        return {"box": box, "bar": bar}

    def update_payload(self, snapshot: dict) -> None:
        mode = str(snapshot.get("scoring_mode", "value_race") or "value_race")
        if mode == "efe":
            self._mode.setText("● EFE  (pragmatic + epistemic)")
            self._mode.setStyleSheet("color: #d0d0d0; font-size: 11px; font-weight: bold;"
                                     " letter-spacing: 1px;")
        else:
            self._mode.setText("○ VALUE_RACE  (legacy)")
            self._mode.setStyleSheet("color: #888; font-size: 11px; font-weight: bold;"
                                     " letter-spacing: 1px;")
        wname = _winner_name(snapshot)
        wpol = next((p for p in POLICIES if p.name == wname), None)
        if wpol is not None:
            self._winner.setText(wpol.label)
            self._winner.setStyleSheet(
                "color: #%02x%02x%02x; font-size: 34px; font-weight: bold;" % wpol.color)
        present = {p.name for p in _present(snapshot)}
        for p in POLICIES:
            f = self._faders[p.name]
            show = p.name in present
            f["box"].setVisible(show)
            if show:
                g = _f(snapshot, f"gain_{p.name}")
                f["bar"].setValue(int(round(max(0.0, min(1.0, g)) * 1000)))
                f["bar"].setFormat(f"{g:.2f}")


class _EFERace(QWidget):
    """The policy score race. In efe: STACKED pragmatic (solid) + epistemic (translucent)
    bars per policy, total height = G. In value_race: the v_klino / v_planner bars."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        self._title = QLabel("EFE score race")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        legend = QLabel("■ pragmatic (exploit)   ▨ epistemic (explore)")
        legend.setStyleSheet("color: #999; font-size: 10px;")
        layout.addWidget(legend)

        self._plot = pg.PlotWidget()
        self._plot.setBackground("k")
        self._plot.showGrid(x=False, y=True, alpha=0.2)
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setXRange(-0.6, 1.6)
        # two stacked layers: pragmatic (bottom, solid) + epistemic (top, translucent). Both are
        # (re)sized every tick to the PRESENT policies (see update_payload), so play/any future
        # policy adds a column automatically. Start with a single empty placeholder bar.
        self._prag = pg.BarGraphItem(x=[0], y0=[0], height=[0], width=0.6,
                                     brushes=[pg.mkBrush(*_DIM)])
        self._epist = pg.BarGraphItem(x=[0], y0=[0], height=[0], width=0.6,
                                      brushes=[pg.mkBrush(*_DIM, 110)])
        self._plot.addItem(self._prag)
        self._plot.addItem(self._epist)
        layout.addWidget(self._plot, 1)

        self._readout = QLabel("—")
        self._readout.setStyleSheet("color: #ccc; font-family: Monospace; font-size: 11px;")
        layout.addWidget(self._readout)

    def update_payload(self, snapshot: dict) -> None:
        mode = str(snapshot.get("scoring_mode", "value_race") or "value_race")
        present = _present(snapshot)
        wname = _winner_name(snapshot)

        xs = list(range(len(present)))
        prag_h, epist_h, prag_brush, epist_brush, ticks, lines = [], [], [], [], [], []
        for i, p in enumerate(present):
            bright = (p.name == wname)
            base = p.color if bright else _DIM
            a_solid = 255 if bright else 45
            a_trans = 120 if bright else 45
            ticks.append((i, p.name))
            if mode == "efe":
                # play has no g_prag_* key → 0 (its pragmatic term is ~0); its epistemic term is
                # g_epist_play, which the g_epist_<name> pattern already resolves.
                prag = _f(snapshot, f"g_prag_{p.name}")
                epist = _f(snapshot, f"g_epist_{p.name}")
                G = _f(snapshot, f"G_{p.name}")
                prag_h.append(prag)
                epist_h.append(epist)
                lines.append(f"{p.label:<7} G {G:6.3f} = prag {prag:6.3f} + epist {epist:6.3f}")
            else:
                v = _f(snapshot, f"v_{p.name}")
                prag_h.append(v)
                epist_h.append(0.0)
                lines.append(f"{p.label:<7} v {v:7.3f}  raw {_f(snapshot, 'raw_' + p.name):8.4f}")
            prag_brush.append(pg.mkBrush(*base, a_solid))
            epist_brush.append(pg.mkBrush(*base, a_trans))

        if xs:
            self._prag.setOpts(x=xs, y0=[0.0] * len(xs), height=prag_h, brushes=prag_brush)
            # epistemic stacked ON TOP of pragmatic (y0 = prag height); all-zero in value_race.
            self._epist.setOpts(x=xs, y0=list(prag_h), height=epist_h, brushes=epist_brush)
        self._plot.getAxis("bottom").setTicks([ticks])
        self._plot.setXRange(-0.6, (len(present) - 1 + 0.6) if present else 1.6)
        lines.append(f"margin {_f(snapshot, 'margin'):6.3f}  (adaptive hysteresis)")
        self._readout.setText("\n".join(lines))
        self._title.setText(
            "EFE score race   G = pragmatic + epistemic  (winner bright)" if mode == "efe"
            else "value race   klino: max(spike, level)  ·  planner: route level")


class _BalanceBar(QWidget):
    """Pragmatic↔epistemic (exploit↔explore) balance, driven by hunger. Left segment width
    ∝ hunger (pragmatic weight), right ∝ 1−hunger (epistemic weight)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._hunger = 0.0
        self.setMinimumHeight(30)

    def set_hunger(self, h: float) -> None:
        self._hunger = max(0.0, min(1.0, h))
        self.update()

    def paintEvent(self, _evt) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        split = int(w * self._hunger)
        p.fillRect(0, 0, split, h, QColor(*_EXPLOIT_COL))
        p.fillRect(split, 0, w - split, h, QColor(*_EXPLORE_COL))
        p.setPen(QColor("#000"))
        p.drawText(6, 0, max(0, split - 8), h,
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, "EXPLOIT")
        p.drawText(split + 6, 0, max(0, w - split - 10), h,
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, "EXPLORE")
        p.setPen(QColor("#fff"))
        p.drawText(0, 0, w, h, Qt.AlignmentFlag.AlignCenter,
                   f"hunger {self._hunger:.2f}")
        p.end()


class _PrecisionPanel(QWidget):
    """§2.3 precision readouts: cap_klino (SENSORY) vs plan_precision (MODEL), plus the
    exploit↔explore balance gauge. Near food the OBSERVATION (cap→1) is more precise than the
    BELIEF (route), and the arbitration reflects that ordering automatically."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(8, 6, 8, 6)
        grid.setVerticalSpacing(4)
        grid.setHorizontalSpacing(8)

        title = QLabel("PRECISION  (§2.3 — which is sharper, the observation or the belief?)")
        title.setStyleSheet("color: #ddd; font-size: 11px;")
        grid.addWidget(title, 0, 0, 1, 2)

        self._cap = self._meter("cap_klino — SENSORY (klino smell)", _KLINO_COL)
        self._prec = self._meter("plan_precision — MODEL (food belief)", _PLANNER_COL)
        grid.addWidget(self._cap["lbl"], 1, 0)
        grid.addWidget(self._cap["bar"], 1, 1)
        grid.addWidget(self._prec["lbl"], 2, 0)
        grid.addWidget(self._prec["bar"], 2, 1)

        bal_lbl = QLabel("balance  (hunger = preference precision)")
        bal_lbl.setStyleSheet("color: #bbb; font-size: 10px;")
        grid.addWidget(bal_lbl, 3, 0)
        self._balance = _BalanceBar()
        grid.addWidget(self._balance, 3, 1)
        grid.setColumnStretch(1, 1)

    def _meter(self, text: str, col: tuple[int, int, int]) -> dict:
        lbl = QLabel(text)
        lbl.setStyleSheet("color: #bbb; font-size: 10px;")
        bar = QProgressBar()
        bar.setRange(0, 1000)
        bar.setTextVisible(True)
        bar.setFormat("0.00")
        hexcol = "#%02x%02x%02x" % col
        bar.setStyleSheet(
            "QProgressBar { background: #1a1a1a; border: 1px solid #333; height: 18px;"
            " text-align: center; color: #fff; }"
            " QProgressBar::chunk { background: %s; }" % hexcol
        )
        return {"lbl": lbl, "bar": bar}

    def update_payload(self, snapshot: dict) -> None:
        cap = _f(snapshot, "cap_klino")
        prec = _f(snapshot, "plan_precision")
        self._cap["bar"].setValue(int(round(max(0.0, min(1.0, cap)) * 1000)))
        self._cap["bar"].setFormat(f"{cap:.2f}")
        self._prec["bar"].setValue(int(round(max(0.0, min(1.0, prec)) * 1000)))
        self._prec["bar"].setFormat(f"{prec:.2f}")
        self._balance.set_hunger(_f(snapshot, "hunger"))


class _RaceSeries(QWidget):
    """Rolling policy scores (G_<policy> in efe, v_<policy> in value_race) with the ±margin band
    around the incumbent. One line per DECLARED policy; a policy that isn't racing this tick
    stays NaN (invisible) until it is present, so play / any future policy joins automatically."""

    def __init__(self, buffer_size: int = 600, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._plot = pg.PlotWidget(title="policy-score race  (band = switch margin around incumbent)")
        self._plot.setBackground("k")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("left", "policy score")
        self._plot.setLabel("bottom", "tick (recent)")
        self._plot.getAxis("left").enableAutoSIPrefix(False)   # keep natural units (no ×0.001 prefix)
        self._plot.addLegend(offset=(-10, 10))
        # one line + one rolling buffer per DECLARED policy (present-only policies stay NaN).
        self._lines: dict[str, object] = {}
        self._buf: dict[str, deque] = {}
        for p in POLICIES:
            self._lines[p.name] = self._plot.plot(pen=pg.mkPen(*p.color, width=1.5), name=p.name)
            self._buf[p.name] = deque([np.nan] * buffer_size, maxlen=buffer_size)
        self._band_hi = self._plot.plot(pen=pg.mkPen(220, 220, 220, 60, style=Qt.PenStyle.DashLine))
        self._band_lo = self._plot.plot(pen=pg.mkPen(220, 220, 220, 60, style=Qt.PenStyle.DashLine))
        layout.addWidget(self._plot)

        self._bhi = deque([np.nan] * buffer_size, maxlen=buffer_size)
        self._blo = deque([np.nan] * buffer_size, maxlen=buffer_size)
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(75)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        mode = str(snapshot.get("scoring_mode", "value_race") or "value_race")
        present = {p.name for p in _present(snapshot)}
        wname = _winner_name(snapshot)
        incumbent = 0.0
        for p in POLICIES:
            if p.name in present:
                val = _f(snapshot, f"G_{p.name}" if mode == "efe" else f"v_{p.name}")
            else:
                val = np.nan
            self._buf[p.name].append(val)
            if p.name == wname and not np.isnan(val):
                incumbent = val
        margin = _f(snapshot, "margin")
        self._bhi.append(incumbent + margin)
        self._blo.append(incumbent - margin)
        self._dirty = True

    def _flush(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        for p in POLICIES:
            self._lines[p.name].setData(np.asarray(self._buf[p.name], dtype=float), connect="finite")
        self._band_hi.setData(np.asarray(self._bhi, dtype=float), connect="finite")
        self._band_lo.setData(np.asarray(self._blo, dtype=float), connect="finite")


class EFEArbiterInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})  —  L2 EFE policy selection")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._winner = _WinnerPanel()
        self._race = _EFERace()
        self._precision = _PrecisionPanel()
        self._series = _RaceSeries()

        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._winner)
        top.addWidget(self._race)
        top.setSizes([300, 400])

        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._precision)
        v.addWidget(self._series)
        v.setSizes([260, 120, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._winner.update_payload(snapshot)
        self._race.update_payload(snapshot)
        self._precision.update_payload(snapshot)
        self._series.update_payload(snapshot)
