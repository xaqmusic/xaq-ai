"""GainEvolver dashboard — watching the lifetime (1+1)-ES search its gain vector.

PART IV's instrument.  The charter's requirement is that the operator can WATCH
IT SEARCH, so the layout answers, top to bottom, the four questions that are
actually asked of a running search:

  1. THE RACK (centerpiece) — "where does each gain sit in its allowed range,
     right now?"  One bounded track per gain: the SEED it started from, the
     INCUMBENT (current believed-best), and — only while a candidate window is
     running — the CANDIDATE being probed.  This is the live parameter view.
  2. TRAJECTORY — "where has the vector walked?"  All gains on ONE axis,
     each NORMALIZED to its own [min,max].  Normalizing is not cosmetic: the
     declared ranges differ by 30x (height_homeo 0..0.1 vs coupling 0..3), so
     on a shared raw axis seven of eight traces are a flat line on the floor.
  3. CRITERION — "is J falling?"  J_inc vs J_cand; lower is better.  The
     incumbent trace is the one that must fall; the candidate trace above it
     means the probe was worse and will be reverted.
  4. TERMS — "which term is actually deciding?"  Weighted contributions
     (w*term), not raw values: a big raw term with a small weight decides
     nothing.  A term pinned at zero is called out as DEAD — which is a
     measurement about its SENSOR, not a verdict on the criterion.  (w_distress
     is the live suspect: the body's distress signal is known-contaminated.)

Colour: the dataviz categorical palette, dark steps, validated for this surface
(worst adjacent CVD dE 8.4, normal-vision 19.3, all >= 3:1 contrast).  Hues are
assigned to gains in FIXED ORDER and never cycled — colour follows the gain, so
hiding a trace never repaints the others.  Text always wears ink tokens; the
small colour chip beside a name is what carries identity.
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush, QFontMetrics
from PyQt6.QtWidgets import (
    QLabel, QScrollArea, QSizePolicy, QSplitter, QVBoxLayout, QWidget,
)

from ._multi_series import MultiSeriesPlot, Series

# ---------------------------------------------------------------------------
# Palette (dataviz categorical, dark steps) + ink tokens.
# ---------------------------------------------------------------------------
SERIES_HEX = ["#3987e5", "#d95926", "#199e70", "#c98500",
              "#d55181", "#008300", "#9085e9", "#e66767"]
SERIES_RGB = [tuple(int(h[i:i + 2], 16) for i in (1, 3, 5)) for h in SERIES_HEX]

SURFACE     = "#1a1a19"    # the surface the palette was validated against
TRACK       = "#2a2d33"
INK_PRIMARY = "#e8eaed"
INK_SECOND  = "#cbd2dc"
INK_MUTED   = "#8b929c"
GOOD        = "#2f9e5f"    # status: accepted
CRIT        = "#d1544f"    # status: reverted

# Gains the picrawler's v1 vector declares, and what each one's scope is.
# Front/rear is NOT mirrored L<->R (the mirror warning applies to left/right),
# so "rear pair" is unambiguous and worth stating on the face of the panel.
SCOPE = {
    "rear_land_gain":    "rear pair",
    "rear_knee_plant":   "rear pair",
    "rear_push_ext":     "rear pair",
    "amp_target":        "body",
    "height_homeo_gain": "body",
    "postural_gain":     "body",
    "coupling_gain":     "body",
    "plan_gain":         "body",
}
PHASES = ["WARMUP", "INCUMBENT", "CANDIDATE"]


def _fl(v, default=float("nan")) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


class _SurfaceWidget(QWidget):
    """A widget that actually paints the chart surface.

    Custom-painted panels inherit Qt's default (light) background otherwise, and
    every ink token here is chosen for the DARK surface the palette was
    validated against — on a light one the text renders invisible.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(self.backgroundRole(), QColor(SURFACE))
        self.setPalette(pal)

    def _fill(self, p: QPainter) -> None:
        p.fillRect(self.rect(), QColor(SURFACE))


class _GainRack(_SurfaceWidget):
    """Bounded-range tracks: seed tick, incumbent marker, candidate probe."""

    ROW_H = 22
    LABEL_W = 214          # fits the longest key + its scope without clipping
    VALUE_W = 150          # value AND its range live here, so nothing lands on
                           # the row boundary and collides with the next track

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._keys: list[str] = []
        self._inc: list[float] = []
        self._cand: list[float] = []
        self._seed: list[float] = []
        self._lo: list[float] = []
        self._hi: list[float] = []
        self._phase = 0
        self._exact_bounds = False
        # Observed extremes, the fallback when the module predates the
        # bounds-in-diag change (an older .so): better an honest auto-scale
        # than a track whose ends are invented.
        self._obs_lo: list[float] = []
        self._obs_hi: list[float] = []
        self.setMinimumHeight(self.ROW_H * 8 + 26)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

    def update_payload(self, s: dict) -> None:
        keys = s.get("gain_keys")
        if not isinstance(keys, list) or not keys:
            return
        if keys != self._keys:
            self._keys = [str(k) for k in keys]
            n = len(self._keys)
            self._obs_lo = [float("inf")] * n
            self._obs_hi = [float("-inf")] * n
        n = len(self._keys)
        get = lambda k: (s.get(k) if isinstance(s.get(k), list) else [])
        self._inc = [_fl(v) for v in get("incumbent")][:n]
        self._cand = [_fl(v) for v in get("candidate")][:n]
        self._seed = [_fl(v) for v in get("gain_seed")][:n]
        lo, hi = [_fl(v) for v in get("gain_min")], [_fl(v) for v in get("gain_max")]
        self._exact_bounds = len(lo) == n and len(hi) == n
        for i, v in enumerate(self._inc):
            if i < len(self._obs_lo) and v == v:
                self._obs_lo[i] = min(self._obs_lo[i], v)
                self._obs_hi[i] = max(self._obs_hi[i], v)
        if self._exact_bounds:
            self._lo, self._hi = lo, hi
        else:
            self._lo, self._hi = [], []
        self._phase = int(_fl(s.get("phase"), 0) or 0)
        self.setMinimumHeight(self.ROW_H * max(1, n) + 26)
        self.update()

    def _bounds(self, i: int) -> tuple[float, float]:
        if self._exact_bounds and i < len(self._lo):
            return self._lo[i], self._hi[i]
        lo = self._obs_lo[i] if i < len(self._obs_lo) else 0.0
        hi = self._obs_hi[i] if i < len(self._obs_hi) else 1.0
        if not (lo < hi):                       # degenerate until it moves
            lo, hi = min(lo, 0.0), max(hi, lo + 1e-6)
        pad = (hi - lo) * 0.25
        out_lo = lo - pad
        # Don't let padding imply a gain can go negative when it never has —
        # an invented negative end reads as a real declared bound.
        if lo >= 0.0:
            out_lo = max(0.0, out_lo)
        return out_lo, hi + pad

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        self._fill(p)
        if not self._keys:
            p.setPen(QPen(QColor(INK_MUTED)))
            p.drawText(6, 18, "waiting for the first gain vector…")
            p.end()
            return
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        fm = QFontMetrics(p.font())
        w = self.width()
        x0 = self.LABEL_W
        x1 = max(x0 + 40, w - self.VALUE_W - 8)
        span = x1 - x0

        head = ("exact bounds from the module"
                if self._exact_bounds else
                "AUTO-SCALED (module predates bounds-in-diag; rebuild for exact ranges)")
        p.setPen(QPen(QColor(INK_MUTED)))
        p.drawText(6, 14, f"gain rack — track spans [min .. max] · {head}")

        for i, key in enumerate(self._keys):
            y = 22 + i * self.ROW_H
            cy = y + self.ROW_H // 2
            rgb = SERIES_RGB[i % len(SERIES_RGB)]
            col = QColor(*rgb)

            # identity chip + name + scope (text stays ink, chip carries identity)
            p.setBrush(QBrush(col))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRoundedRect(QRectF(6, cy - 4, 8, 8), 2, 2)
            p.setPen(QPen(QColor(INK_SECOND)))
            p.drawText(20, cy + 4, key)
            scope = SCOPE.get(key, "")
            if scope:
                p.setPen(QPen(QColor(INK_MUTED)))
                p.drawText(20 + fm.horizontalAdvance(key) + 6, cy + 4, scope)

            lo, hi = self._bounds(i)
            rng = (hi - lo) or 1.0
            pos = lambda v: x0 + span * min(1.0, max(0.0, (v - lo) / rng))

            # recessive track
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(QColor(TRACK)))
            p.drawRoundedRect(QRectF(x0, cy - 3, span, 6), 3, 3)

            # seed tick — where the search started
            if i < len(self._seed) and self._seed[i] == self._seed[i]:
                sx = pos(self._seed[i])
                p.setPen(QPen(QColor(INK_MUTED), 1, Qt.PenStyle.DashLine))
                p.drawLine(int(sx), cy - 8, int(sx), cy + 8)

            inc = self._inc[i] if i < len(self._inc) else float("nan")
            cand = self._cand[i] if i < len(self._cand) else float("nan")

            # filled span from seed to incumbent makes the DIRECTION of travel legible
            if i < len(self._seed) and self._seed[i] == self._seed[i] and inc == inc:
                a, b = sorted((pos(self._seed[i]), pos(inc)))
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(QColor(rgb[0], rgb[1], rgb[2], 70)))
                p.drawRoundedRect(QRectF(a, cy - 3, max(1.0, b - a), 6), 3, 3)

            # candidate probe (hollow ring), only while a candidate is being evaluated
            if self._phase == 2 and cand == cand:
                cx = pos(cand)
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.setPen(QPen(QColor(SURFACE), 4))          # surface ring: stays legible on overlap
                p.drawEllipse(QRectF(cx - 5, cy - 5, 10, 10))
                p.setPen(QPen(col, 2))
                p.drawEllipse(QRectF(cx - 5, cy - 5, 10, 10))

            # incumbent (filled) — the current believed-best
            if inc == inc:
                ix = pos(inc)
                p.setPen(QPen(QColor(SURFACE), 2))
                p.setBrush(QBrush(col))
                p.drawEllipse(QRectF(ix - 5.5, cy - 5.5, 11, 11))

            # value + range, both in the value column (ink, never the series colour)
            p.setPen(QPen(QColor(INK_PRIMARY)))
            txt = "—" if inc != inc else f"{inc:.4g}"
            p.drawText(x1 + 8, cy + 4, txt)
            p.setPen(QPen(QColor(INK_MUTED)))
            p.drawText(x1 + 8 + fm.horizontalAdvance(txt) + 8, cy + 4,
                       f"[{lo:.2g} – {hi:.2g}]")
        p.end()


class _TermBars(_SurfaceWidget):
    """Weighted criterion contributions — w*term, the thing that moves J."""

    TERMS = [("falls", "falls (guard)"), ("tilt_sd", "upright sd"),
             ("dwell", "dwell (near-inv)"),
             ("distress_duty", "distress"), ("unloaded_mean", "unloaded"),
             ("flow_term", "flow"), ("energy", "energy (current)")]
    ROW_H = 22

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._vals: dict[str, float] = {}
        self._weights: dict[str, float] = {}
        self._have_w = False
        self.setMinimumHeight(self.ROW_H * len(self.TERMS) + 26)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

    def update_payload(self, s: dict) -> None:
        self._vals = {k: _fl(s.get(k)) for k, _ in self.TERMS}
        w = s.get("weights")
        self._have_w = isinstance(w, dict)
        self._weights = {k: _fl(w.get(k), 1.0) for k, _ in self.TERMS} if self._have_w \
            else {k: 1.0 for k, _ in self.TERMS}
        self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        self._fill(p)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        fm = QFontMetrics(p.font())
        p.setPen(QPen(QColor(INK_MUTED)))
        p.drawText(6, 14, "criterion terms — weighted contribution to J (lower = better)"
                          if self._have_w else
                          "criterion terms — RAW (weights unavailable; rebuild to weight)")
        contrib = {k: (self._vals.get(k, float("nan")) * self._weights.get(k, 1.0))
                   for k, _ in self.TERMS}
        finite = [v for v in contrib.values() if v == v]
        top = max(finite) if finite else 1.0
        top = top if top > 1e-9 else 1.0
        x0, w = 150, self.width()   # fits the longest term label without clipping
        x1 = max(x0 + 30, w - 190)      # room for value + the DEAD callout
        for i, (key, label) in enumerate(self.TERMS):
            y = 22 + i * self.ROW_H
            cy = y + self.ROW_H // 2
            v = contrib.get(key, float("nan"))
            p.setPen(QPen(QColor(INK_SECOND)))
            p.drawText(6, cy + 4, label)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(QColor(TRACK)))
            p.drawRoundedRect(QRectF(x0, cy - 5, x1 - x0, 10), 4, 4)
            if v == v and v > 0:
                frac = min(1.0, v / top)
                p.setBrush(QBrush(QColor(*SERIES_RGB[i % len(SERIES_RGB)])))
                p.drawRoundedRect(QRectF(x0, cy - 5, max(2.0, (x1 - x0) * frac), 10), 4, 4)
            vtxt = "—" if v != v else f"{v:.4g}"
            p.setPen(QPen(QColor(INK_PRIMARY)))
            p.drawText(x1 + 8, cy + 4, vtxt)
            if v == v and abs(v) < 1e-9:
                # Distinguish the two zeros, because they mean opposite things:
                # weight 0 is a DESIGN choice (falls is guard-only; distress is
                # off until its sensor is fixed), whereas a weighted term that
                # never fires is a statement about that SENSOR.
                off = abs(self._weights.get(key, 1.0)) < 1e-12
                p.setPen(QPen(QColor(INK_MUTED if off else CRIT)))
                p.drawText(x1 + 8 + fm.horizontalAdvance(vtxt) + 10, cy + 4,
                           "off (weight 0)" if off else "DEAD — sensor never fired")
        p.end()


class _HistoryStrip(_SurfaceWidget):
    """Accept / revert outcomes, newest on the right.  Letter + colour, never colour alone."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._log = ""
        self.setFixedHeight(30)

    def update_payload(self, s: dict) -> None:
        log = s.get("accept_log")
        self._log = str(log) if isinstance(log, str) else ""
        self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        self._fill(p)
        p.setPen(QPen(QColor(INK_MUTED)))
        p.drawText(6, 12, "generations (oldest → newest)")
        bw, x = 14, 6
        for ch in self._log[-40:]:
            good = ch == "A"
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(QColor(GOOD if good else CRIT)))
            p.drawRoundedRect(QRectF(x, 16, bw - 2, 11), 2, 2)
            p.setPen(QPen(QColor("#0d0f0e")))
            p.drawText(int(x + 3), 25, "A" if good else "R")
            x += bw
        p.end()


class GainEvolverInspector(QWidget):
    def __init__(self, module_id: str, module_type: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(self.backgroundRole(), QColor(SURFACE))
        self.setPalette(pal)

        self._status = QLabel("waiting for the evolver…")
        self._status.setTextFormat(Qt.TextFormat.RichText)
        self._status.setStyleSheet(
            f"color:{INK_SECOND}; font-size:12px; background:{SURFACE}; padding:3px 4px;")
        outer.addWidget(self._status)

        self._rack = _GainRack()
        self._history = _HistoryStrip()
        self._terms = _TermBars()

        # J is a plain flat key pair, so the shared series plot handles it as-is.
        self._j = MultiSeriesPlot(
            [Series("J_inc",  "J incumbent", SERIES_RGB[0], width=2.0),
             Series("J_cand", "J candidate", SERIES_RGB[1], width=1.5,
                    style=Qt.PenStyle.DashLine)],
            title="criterion J — incumbent vs candidate (lower = better)",
            y_label="J",
        )
        self._j.setMinimumHeight(120)
        # J runs O(0.1–1); the SI prefix would relabel that as "500 x0.001".
        self._j._plot.getPlotItem().getAxis("left").enableAutoSIPrefix(False)
        # Built lazily: the gain names arrive with the first payload.
        self._traj: Optional[MultiSeriesPlot] = None
        self._traj_keys: list[str] = []

        split = QSplitter(Qt.Orientation.Vertical)
        top = QWidget(); tl = QVBoxLayout(top)
        tl.setContentsMargins(0, 0, 0, 0); tl.setSpacing(2)
        tl.addWidget(self._rack)
        tl.addWidget(self._history)
        split.addWidget(top)
        self._split = split
        split.addWidget(self._j)
        split.addWidget(self._terms)
        split.setSizes([250, 250, 180])
        # Natural height of the whole stack: the plots deserve ~200px each to be
        # readable, so ask for that and let the scroll area absorb a short pane
        # rather than shrinking every panel to its floor.
        split.setMinimumHeight(770)
        # The rack is the centerpiece; a short pane must SCROLL, never clip it
        # (Qt otherwise shaves the over-minimum child and silently hides gains).
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"background:{SURFACE};")
        scroll.setWidget(split)
        outer.addWidget(scroll, 1)

    # ---- lazy trajectory plot -------------------------------------------------
    def _ensure_traj(self, keys: Sequence[str]) -> None:
        if self._traj is not None and list(keys) == self._traj_keys:
            return
        if self._traj is not None:
            self._traj.setParent(None)
            self._traj.deleteLater()
        self._traj_keys = [str(k) for k in keys]
        series = [Series(f"g{i}", k, SERIES_RGB[i % len(SERIES_RGB)], width=1.6)
                  for i, k in enumerate(self._traj_keys)]
        self._traj = MultiSeriesPlot(
            series,
            title="gain trajectory — each normalized to its own [min, max]",
            y_label="position in range",
        )
        self._traj.setMinimumHeight(120)
        # The axis IS the normalized range, so pin it: left to autorange it
        # rescales per-frame (the traces appear to jump when nothing moved), and
        # pyqtgraph's SI-prefix would relabel a [0,1] axis as "400 x0.001".
        ax = self._traj._plot.getPlotItem()
        ax.setYRange(0.0, 1.0, padding=0.02)
        ax.enableAutoRange(axis="y", enable=False)
        ax.getAxis("left").enableAutoSIPrefix(False)
        self._split.insertWidget(1, self._traj)
        self._split.setSizes([230, 220, 220, 140])
        self._split.setMinimumHeight(830)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        s = snapshot
        self._rack.update_payload(s)
        self._history.update_payload(s)
        self._terms.update_payload(s)
        # J carries -1 as "no score yet" (warmup, or no candidate this window).
        # Plotting the sentinel would draw a square wave down to -1 and squash the
        # real signal into a flat line — drop the key instead so the curve BREAKS.
        jd = {}
        for k in ("J_inc", "J_cand"):
            v = _fl(s.get(k), -1.0)
            if v >= 0.0:
                jd[k] = v
        self._j.update_payload(jd)

        keys = s.get("gain_keys")
        if isinstance(keys, list) and keys:
            self._ensure_traj([str(k) for k in keys])
            # Normalize each gain into its own declared range so one axis is honest.
            vec = s.get("incumbent") if isinstance(s.get("incumbent"), list) else []
            norm: dict[str, float] = {}
            for i in range(len(keys)):
                lo, hi = self._rack._bounds(i)
                v = _fl(vec[i]) if i < len(vec) else float("nan")
                if v == v and hi > lo:
                    norm[f"g{i}"] = (v - lo) / (hi - lo)
            if self._traj is not None:
                self._traj.update_payload(norm)

        gen   = int(_fl(s.get("generation"), 0) or 0)
        acc   = int(_fl(s.get("accepts"), 0) or 0)
        rev   = int(_fl(s.get("reverts"), 0) or 0)
        pub   = int(_fl(s.get("publishes"), 0) or 0)
        sigma = _fl(s.get("sigma"), 0.0)
        ph    = int(_fl(s.get("phase"), 0) or 0)
        wt    = int(_fl(s.get("win_tick"), 0) or 0)
        win   = int(_fl(s.get("eval_window_ticks"), 0) or 0)
        jin, jca = _fl(s.get("J_inc"), -1.0), _fl(s.get("J_cand"), -1.0)
        minld = _fl(s.get("loaded_min"), float("nan"))
        prog = f"{wt}/{win}" if win else str(wt)
        phase_col = {0: INK_MUTED, 1: SERIES_HEX[0], 2: SERIES_HEX[1]}.get(ph, INK_MUTED)
        searching = ("SEARCHING" if sigma > 0 else "OBSERVER (σ=0 — scoring only, nothing published)")
        self._status.setText(
            f"<b>gen {gen}</b> &nbsp; "
            f"<span style='color:{phase_col}'>{PHASES[ph] if 0 <= ph < 3 else '?'}</span> "
            f"<span style='color:{INK_MUTED}'>{prog}</span> &nbsp;·&nbsp; "
            f"σ {sigma:.3f} <span style='color:{INK_MUTED}'>{searching}</span> &nbsp;·&nbsp; "
            f"<span style='color:{GOOD}'>{acc} accepted</span> / "
            f"<span style='color:{CRIT}'>{rev} reverted</span> &nbsp;·&nbsp; "
            f"J_inc {jin:.4f} &nbsp; J_cand {'—' if jca < 0 else f'{jca:.4f}'} &nbsp;·&nbsp; "
            f"per-leg loaded min {minld:.3f} &nbsp;·&nbsp; "
            f"{'<b style=\'color:#d1544f\'>FALL ALARM</b> ' if int(_fl(s.get('alarm_on'),0) or 0) else ''}"
            f"alarm {_fl(s.get('fall_alarm'), 0.0):.2f} &nbsp;·&nbsp; "
            f"margin {_fl(s.get('accept_margin'), 0.0):.4f} "
            f"<span style='color:{INK_MUTED}'>(σ̂ {_fl(s.get('sigma_est'), 0.0):.4f}, "
            f"n={int(_fl(s.get('noise_n'), 0) or 0)})</span> &nbsp;·&nbsp; "
            f"<span style='color:{INK_MUTED}'>{pub} vectors published</span>"
        )
