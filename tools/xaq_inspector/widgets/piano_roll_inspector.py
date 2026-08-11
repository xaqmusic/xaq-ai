"""Motor piano roll — the MotorPlanner's probability cone as a DAW timeline.

THE OPERATOR'S ARCHITECTURE (2026-08-11).  The motor buffer is a piano roll:
the EXECUTED past scrolls left of the playhead and is immutable; the t0 column
is owned by reflexes (the actual present); the FUTURE columns are mutable rows
that planning loops write into — slow loops far out, fast loops near-in.  Loops
suppress reflexes only where their confidence is EARNED, and the earning is
measured: the planner verifies its own cone at probe depths against the future
when it arrives.

WHAT EACH TRACK SHOWS.  One joint per track (12 = 4 legs × hip1/hip2/knee),
normalised angle vs time in ticks relative to NOW:

    left of the playhead    the actual executed pose (solid, immutable)
    red line at 0           the playhead — the reflex-owned present
    blue curve rightward    the cone's expected pose (argmax-free: the full
                            probability-weighted decode through the per-token
                            pose readout)
    blue band               ±1σ of the decode — the FAN.  Its width is the
                            product: watch it blow out at the information
                            horizon, and tighten as the vocabulary improves.
    gold dashed line        THE AUTHORITY HORIZON — the deepest probe where the
                            cone's verified argmax accuracy still beats the
                            persistence baseline (scored under the identical
                            pending protocol) by ≥5%, with n≥200 verdicts.
                            Left of it the future is drawn saturated (authority
                            defensible); right of it, washed out.  At 0 the
                            planner has earned nothing and reflexes own the
                            whole roll — today's honest state.
    faint dotted verticals  the stride ruler — one line per predicted stride
                            (period 2π/ω from the rhythm reference).
    faint horizontal        the joint's slow-EMA rest level (self-calibrating
                            neutral; bars read up/down against it).

DATA PATH.  The planner ships the WHOLE roll (decoded mean/sd × horizon) and a
128-tick past ring in every diag payload — not accumulated client-side, because
DiagPublisher throttles to the subscription hz and an accumulator would alias
(the gait-raster lesson).
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (QComboBox, QHBoxLayout, QLabel, QPushButton,
                             QVBoxLayout, QWidget)

_LEGS = ("FL", "FR", "RL", "RR")
_JOINTS = ("h1", "h2", "kn")
# reality.proprio.joints layout: 4 hip1 + 4 hip2 + 4 knee → joint index = leg + 4*j
_PAST_SHOW = 128          # ticks of past drawn (== planner ring size)

_C_PAST      = (205, 205, 210)
_C_FUT_AUTH  = (130, 200, 255)        # earned-authority segment: bright, wide
_C_FUT_DIM   = (130, 200, 255, 175)   # unearned segment: same hue, clearly visible,
                                      # distinguished by width/solidity not invisibility
_C_FAN       = (120, 190, 255, 70)
_C_PLAYHEAD  = (255, 95, 85)
_C_AUTHORITY = (255, 205, 95)        # GLOBAL (token-chain) authority: dashed
_C_JOINT_BAND = (255, 175, 60, 34)   # per-joint authority BAND fill
_C_JOINT_BAND_EDGE = (255, 175, 60, 140)
_C_NEUTRAL   = (130, 130, 135, 110)
_C_STRIDE    = (255, 255, 255, 26)
_MAX_STRIDE_LINES = 8


class _Track:
    """One joint's timeline: past curve, future mean (two authority segments),
    ±1σ fan, playhead, authority line, neutral line, stride ruler pool."""

    def __init__(self, plot: pg.PlotItem, label: str):
        self.plot = plot
        plot.setLabel("left", label)
        plot.getAxis("left").setStyle(showValues=False, tickLength=0)
        plot.getAxis("left").setWidth(44)
        plot.getAxis("left").enableAutoSIPrefix(False)
        # wheel-zoom / drag-pan on the TIME axis only — the x-link makes every
        # gesture a whole-roll gesture.  y is never mouse-driven: each track's
        # vertical scale auto-fits the VISIBLE time window every flush, so
        # zooming into time magnifies vertical detail uniformly on all tracks
        # (per-axis wheel zoom on the hovered track was the inconsistency).
        plot.setMouseEnabled(x=True, y=False)
        plot.hideButtons()
        plot.setMenuEnabled(False)

        self.past = plot.plot(pen=pg.mkPen(_C_PAST, width=1))
        # fan bounds: invisible pens, filled between
        self.fan_hi = plot.plot(pen=None)
        self.fan_lo = plot.plot(pen=None)
        self.fan = pg.FillBetweenItem(self.fan_hi, self.fan_lo,
                                      brush=pg.mkBrush(_C_FAN))
        plot.addItem(self.fan)
        self.fut_auth = plot.plot(pen=pg.mkPen(_C_FUT_AUTH, width=2.5))
        self.fut_dim = plot.plot(pen=pg.mkPen(_C_FUT_DIM, width=1.5))

        self.playhead = pg.InfiniteLine(pos=0.0, angle=90,
                                        pen=pg.mkPen(_C_PLAYHEAD, width=2))
        plot.addItem(self.playhead)
        self.authority = pg.InfiniteLine(
            pos=0.0, angle=90,
            pen=pg.mkPen(_C_AUTHORITY, width=2, style=Qt.PenStyle.DashLine))
        plot.addItem(self.authority)
        # this JOINT's own verified authority BAND — the marginal can lose at
        # k=1 (jump to token mean vs a barely-moving body) yet win at stride
        # scale, so earned authority is an INTERVAL, not a line from the present
        self.band = pg.LinearRegionItem(
            values=(0, 0), movable=False,
            brush=pg.mkBrush(_C_JOINT_BAND),
            pen=pg.mkPen(_C_JOINT_BAND_EDGE, width=1))
        self.band.setZValue(-10)
        self.band.setVisible(False)
        plot.addItem(self.band)
        self.neutral = pg.InfiniteLine(
            pos=0.0, angle=0,
            pen=pg.mkPen(_C_NEUTRAL, width=1, style=Qt.PenStyle.DotLine))
        plot.addItem(self.neutral)
        self.stride_lines = []
        for _ in range(_MAX_STRIDE_LINES):
            ln = pg.InfiniteLine(pos=0.0, angle=90, pen=pg.mkPen(_C_STRIDE, width=1))
            ln.setVisible(False)
            plot.addItem(ln)
            self.stride_lines.append(ln)
        self.rest_ema: float | None = None   # client-side slow EMA of the actual
        self.ylo: float | None = None        # smoothed adaptive y-range bounds
        self.yhi: float | None = None


class PianoRollInspector(QWidget):
    """Live motor piano roll for the MotorPlanner (see module docstring)."""

    def __init__(self, module_id: str = "motor_planner",
                 module_type: str = "MotorPlanner",
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        header = QHBoxLayout()
        self._title = QLabel("Motor piano roll — awaiting MotorPlanner diag")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        header.addWidget(self._title, 1)
        self._leg_sel = QComboBox()
        self._leg_sel.addItems(["All legs (12 tracks)"] + [f"{l} (3 tracks)" for l in _LEGS])
        self._leg_sel.currentIndexChanged.connect(self._rebuild_tracks)
        header.addWidget(self._leg_sel)
        self._fit_btn = QPushButton("Auto-fit")
        self._fit_btn.setCheckable(True)
        self._fit_btn.setChecked(True)
        self._fit_btn.setToolTip(
            "Checked: the time window follows the data (centred playhead). "
            "Wheel-zoom or drag any track to take manual control of the time "
            "axis — all tracks move together, and each track's vertical scale "
            "always auto-fits the visible window.  Click to snap back.")
        self._fit_btn.toggled.connect(self._on_fit_toggled)
        header.addWidget(self._fit_btn)
        layout.addLayout(header)

        self._view = pg.GraphicsLayoutWidget()
        self._view.setBackground("k")
        layout.addWidget(self._view, 1)

        self._readout = QLabel("")
        self._readout.setStyleSheet(
            "color: #ddd; font-family: Monospace; font-size: 11px;")
        self._readout.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._readout)

        self._tracks: list[tuple[int, _Track]] = []   # (joint_index, track)
        self._latest: dict | None = None
        self._dirty = False
        self._auto_view = True
        self._rebuild_tracks()

        self._refresh = QTimer(self)
        self._refresh.setInterval(150)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    # ------------------------------------------------------------------
    def _joint_rows(self) -> list[tuple[int, str]]:
        """(joint_index, label) rows for the current leg selection, grouped by leg."""
        sel = self._leg_sel.currentIndex()
        legs = range(4) if sel == 0 else [sel - 1]
        rows = []
        for leg in legs:
            for j, jn in enumerate(_JOINTS):
                rows.append((leg + 4 * j, f"{_LEGS[leg]}·{jn}"))
        return rows

    def _rebuild_tracks(self) -> None:
        self._view.clear()
        self._tracks = []
        rows = self._joint_rows()
        first_plot = None
        for i, (jidx, label) in enumerate(rows):
            plot = self._view.addPlot(row=i, col=0)
            if i < len(rows) - 1:
                plot.getAxis("bottom").setStyle(showValues=False, tickLength=0)
            else:
                plot.setLabel("bottom", "ticks relative to now")
            self._tracks.append((jidx, _Track(plot, label)))
            # the time axis is ONE axis: zooming any track zooms them all
            if first_plot is None:
                first_plot = plot
            else:
                plot.setXLink(first_plot)
            plot.vb.sigRangeChangedManually.connect(self._on_manual_zoom)
        self._dirty = True

    def _on_manual_zoom(self, *_a) -> None:
        if self._auto_view:
            self._fit_btn.setChecked(False)   # toggled() flips _auto_view

    def _on_fit_toggled(self, checked: bool) -> None:
        self._auto_view = checked
        if checked:
            self._dirty = True                # snap back on next flush

    # ------------------------------------------------------------------
    def update_payload(self, tick_id: int, snapshot: dict | None = None) -> None:
        # Top-level convention: the inspector (via the description card) calls
        # (tick_id, snapshot).  Accept (snapshot) alone too for offscreen tests.
        if snapshot is None and isinstance(tick_id, dict):
            snapshot = tick_id
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    # ------------------------------------------------------------------
    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        snap = self._latest

        nj = int(snap.get("joints", 12))
        roll_len = int(snap.get("roll_len", 0))
        past_len = int(snap.get("past_len", 0))
        if nj != 12 or past_len == 0:
            self._title.setText("Motor piano roll — no roll data in payload yet")
            return
        horizon = int(snap.get("horizon", 40))
        rm = np.asarray(snap.get("roll_mean", []), dtype=float).reshape(-1, nj) \
            if roll_len else np.zeros((0, nj))
        rs = np.asarray(snap.get("roll_sd", []), dtype=float).reshape(-1, nj) \
            if roll_len else np.zeros((0, nj))
        past = np.asarray(snap.get("past", []), dtype=float).reshape(-1, nj)
        authority = int(snap.get("authority_depth", 0))
        joint_band = snap.get("joint_band", [0, 0] * nj)
        phi = float(snap.get("phi", 0.0))
        omega = float(snap.get("omega", 0.105))
        period = (2.0 * np.pi / omega) if omega > 1e-6 else 0.0

        px = np.arange(-past.shape[0] + 1, 1)          # past ends AT the playhead
        fx = np.arange(1, roll_len + 1)
        # symmetric window, t0 dead centre — the cone's SPREAD is the object of
        # analysis, so the future half gets equal billing (and headroom beyond
        # the current horizon, which will grow).
        half_w = min(_PAST_SHOW, max(64, horizon + 8))
        # stride ruler: phase-zero crossings at n0 + m·period, spanning the view
        stride_xs = []
        if period > 0:
            n0 = ((2.0 * np.pi - phi) % (2.0 * np.pi)) / omega
            m0 = int(np.ceil((-half_w - n0) / period))
            x = n0 + m0 * period
            while x <= half_w and len(stride_xs) < _MAX_STRIDE_LINES:
                stride_xs.append(x)
                x += period

        for jidx, tr in self._tracks:
            tr.past.setData(px, past[:, jidx])
            actual_now = float(past[-1, jidx])
            if roll_len:
                # anchor the cone AT the playhead: it opens from the actual
                # present pose with σ=0 — the immutable point every future
                # fans out of.
                mean = np.concatenate(([actual_now], rm[:, jidx]))
                sd = np.concatenate(([0.0], rs[:, jidx]))
                xs = np.concatenate(([0], fx))
                tr.fan_hi.setData(xs, mean + sd)
                tr.fan_lo.setData(xs, mean - sd)
                # full future washed; THIS JOINT's earned band overdrawn bright
                tr.fut_dim.setData(xs, mean)
                blo = int(joint_band[2 * jidx]) if 2 * jidx + 1 < len(joint_band) else 0
                bhi = int(joint_band[2 * jidx + 1]) if 2 * jidx + 1 < len(joint_band) else 0
                if bhi > 0:
                    bm = (xs >= blo) & (xs <= bhi)
                    tr.fut_auth.setData(xs[bm], mean[bm])
                    tr.band.setRegion((blo, bhi))
                    tr.band.setVisible(True)
                else:
                    tr.fut_auth.setData([], [])
                    tr.band.setVisible(False)
            else:
                for c in (tr.fan_hi, tr.fan_lo, tr.fut_auth, tr.fut_dim):
                    c.setData([], [])
            # ALWAYS drawn: at authority 0 the gold line sits collapsed ON the
            # playhead — "the planner's authority ends at the present" — and
            # visibly detaches rightward as authority is earned.  (Hiding it at
            # 0 made honest-zero indistinguishable from a broken feature.)
            tr.authority.setPos(float(authority))
            tr.rest_ema = actual_now if tr.rest_ema is None else \
                tr.rest_ema + 0.02 * (actual_now - tr.rest_ema)
            tr.neutral.setPos(tr.rest_ema)
            for i, ln in enumerate(tr.stride_lines):
                if i < len(stride_xs):
                    ln.setPos(stride_xs[i])
                    ln.setVisible(True)
                else:
                    ln.setVisible(False)
            if self._auto_view:
                tr.plot.setXRange(-half_w, half_w, padding=0)
                xlo, xhi = -half_w, half_w
            else:
                xlo, xhi = tr.plot.vb.viewRange()[0]
            # y ALWAYS auto-fits the data inside the VISIBLE time window,
            # smoothed so the view breathes instead of jittering; min span
            # guards flat joints.  Zooming time thus magnifies vertical detail
            # uniformly across tracks — y is never mouse-driven.
            vals = [past[(px >= xlo) & (px <= xhi), jidx]]
            if roll_len:
                fm = (fx >= xlo) & (fx <= xhi)
                vals += [rm[fm, jidx] + rs[fm, jidx], rm[fm, jidx] - rs[fm, jidx]]
            allv = np.concatenate([v for v in vals if v.size])
            if allv.size:
                lo, hi = float(allv.min()), float(allv.max())
                pad = max(0.18 * (hi - lo), 0.04)
                lo, hi = lo - pad, hi + pad
                if tr.ylo is None:
                    tr.ylo, tr.yhi = lo, hi
                else:
                    tr.ylo += 0.3 * (lo - tr.ylo)
                    tr.yhi += 0.3 * (hi - tr.yhi)
                tr.plot.setYRange(tr.ylo, tr.yhi, padding=0)

        # readout: the earned-authority story in numbers
        depths = snap.get("probe_depths", [])
        top1 = snap.get("cone_top1", [])
        pers = snap.get("cone_persist", [])
        pairs = "  ".join(
            f"k{int(d)}:{t:.2f}/{p:.2f}"
            for d, t, p in zip(depths, top1, pers))
        self._title.setText(
            f"Motor piano roll — AUTHORITY HORIZON = {authority} ticks"
            + (f" ({authority / period:.2f} strides)" if period > 0 else "")
            + ("   [reflexes own the roll]" if authority == 0 else ""))
        def _band_str(jidx: int) -> str:
            lo, hi = int(joint_band[2 * jidx]), int(joint_band[2 * jidx + 1])
            return f"{lo}-{hi}" if hi > 0 else "·"
        jgrid = "  ".join(
            f"{_LEGS[leg]} " + "/".join(_band_str(leg + 4 * j) for j in range(3))
            for leg in range(4)) if len(joint_band) >= 24 else "—"
        self._readout.setText(
            f"  stride ≈ {period:.0f} ticks   n_obs {int(snap.get('n_obs', 0))}   "
            f"mask_mode {snap.get('mask_mode', 0):.0f} "
            f"(pruned {int(snap.get('masked_out', 0))})\n"
            f"  verified cone-top1/persistence per depth:  {pairs}\n"
            f"  per-joint authority BANDS (h1/h2/kn, ticks):  {jgrid}\n"
            f"  dashed gold = whole-body token authority (from t0); amber band + "
            f"bright segment = depths where that JOINT's decoded marginal beats "
            f"hold-pose by ≥5%.")
