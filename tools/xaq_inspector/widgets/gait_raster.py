"""Footfall raster — the picture that makes the stroke-to-step relation visible.

WHY THIS EXISTS.  The picrawler campaign's sharpest finding was invisible in every
aggregate metric it had: the power stroke rides a 22-24 tick knee-derived clock while
the leg actually steps every 26-30 ticks, so push direction is statistically INDEPENDENT
of whether the foot is on the ground (fraction of STANCE in the stroke's positive half =
0.512, over SWING = 0.513).  Those two clocks beat at ~2.5 s, which is exactly the
operator's report that "occasionally the three planted legs are in a good position and
the fourth steps forward and moves the body, but this synchronization is often lost."

A number could not show that.  A Hildebrand plot can: four rows, filled where the foot
is down, with the stroke's PUSH half shaded behind.  Push happening while a row is empty
is thrust spent in the air, and it is obvious at a glance.

WHAT EACH ROW MEANS

    ███  solid        that foot is on the ground (TRUE physics contact)
    ░░░  empty        that foot is airborne
    shaded backdrop   the stroke's commanded push half for that leg
    thin lower strip  the INCUMBENT foot-height detector's swing state

That last strip is worth its space: the height detector is what `stance_lift` and every
Cruse rule gate on, and it was measured firing about TWICE per real step.  Drawn against
ground truth, its chatter is visible rather than merely documented.

DATA PATH.  MotorEPM keeps a 512-tick ring (`gait_raster_diag=1`) of packed per-tick bits
and ships it whole in `diag_snapshot()`.  It is NOT accumulated here from successive
payloads, because DiagPublisher throttles each subscription to a target Hz (default 30)
against a ~52 tick/s brain — a client-side accumulator would alias at exactly the
touchdown edges this plot exists to show.
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

# Bit layout, mirroring MotorEPM::update_gait_raster().
_BIT_CONTACT = 0      # bits 0-3
_BIT_STROKE = 4       # bits 4-7
_BIT_SWING = 8        # bits 8-11

_LEG_NAMES = ("FL", "FR", "RL", "RR")

# Rows per leg in the rendered image: a thick contact band, a thin detector strip,
# and a gap.  Chosen so the contact band dominates and the detector reads as an
# annotation rather than a competing signal.
_H_CONTACT = 7
_H_DETECT = 2
_H_GAP = 2
_H_LEG = _H_CONTACT + _H_DETECT + _H_GAP

# RGB rows.  Stance is bright; the push backdrop is a dim warm wash so it reads
# *behind* the footfall rather than competing with it.
_C_BG = (18, 18, 20)
_C_STANCE = (235, 238, 245)
_C_STANCE_PUSH = (255, 205, 120)      # planted AND pushing — the productive state
_C_SWING_PUSH = (150, 80, 60)         # airborne AND pushing — thrust thrown away
_C_SWING = (40, 42, 48)
_C_DETECT = (110, 160, 255)


class GaitRaster(QWidget):
    """Live Hildebrand plot of footfall vs the commanded power stroke."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._title = QLabel("Footfall raster — awaiting gait_raster_diag=1")
        self._title.setStyleSheet("color: #ddd; font-size: 11px;")
        layout.addWidget(self._title)

        self._view = pg.GraphicsLayoutWidget()
        self._view.setBackground("k")
        self._vb = self._view.addViewBox()
        self._vb.setAspectLocked(False)
        self._vb.invertY(True)
        self._vb.setMouseEnabled(x=False, y=False)
        self._img = pg.ImageItem(axisOrder="row-major")
        self._vb.addItem(self._img)
        layout.addWidget(self._view, 1)

        self._legend = QLabel(
            "  ███ planted   ▓▓▓ planted+pushing   ▒▒▒ AIRBORNE+pushing (wasted)   "
            "··· foot-height detector")
        self._legend.setStyleSheet("color: #999; font-family: Monospace; font-size: 10px;")
        layout.addWidget(self._legend)

        self._readout = QLabel("")
        self._readout.setStyleSheet(
            "color: #ddd; font-family: Monospace; font-size: 11px;")
        self._readout.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._readout)

        self._latest: dict | None = None
        self._dirty = False
        self._refresh = QTimer(self)
        self._refresh.setInterval(150)
        self._refresh.timeout.connect(self._flush)
        self._refresh.start()

    # ------------------------------------------------------------------
    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._latest = snapshot
            self._dirty = True

    # ------------------------------------------------------------------
    def _flush(self) -> None:
        if not self._dirty or self._latest is None:
            return
        self._dirty = False
        snap = self._latest

        raster = snap.get("gait_raster")
        if not raster:
            self._title.setText(
                "Footfall raster — set gait_raster_diag=1 (and contact_topic) on this MotorEPM")
            return
        n_legs = int(snap.get("gait_raster_legs", 4) or 4)
        n_legs = max(1, min(4, n_legs))
        words = np.asarray(raster, dtype=np.int32)
        n = words.size
        if n < 2:
            return

        img = np.zeros((n_legs * _H_LEG, n, 3), dtype=np.uint8)
        img[:, :] = _C_BG
        for leg in range(n_legs):
            contact = (words >> (_BIT_CONTACT + leg)) & 1
            push = (words >> (_BIT_STROKE + leg)) & 1
            swing_det = (words >> (_BIT_SWING + leg)) & 1

            # Four states, and the two that matter are the mixed ones: planted+pushing
            # is the leg doing its job, airborne+pushing is thrust spent in the air.
            row = np.empty((n, 3), dtype=np.uint8)
            row[:] = _C_SWING
            row[(contact == 1)] = _C_STANCE
            row[(contact == 1) & (push == 1)] = _C_STANCE_PUSH
            row[(contact == 0) & (push == 1)] = _C_SWING_PUSH

            y0 = leg * _H_LEG
            img[y0:y0 + _H_CONTACT, :] = row[None, :, :]
            # Incumbent detector strip: lit where IT thinks the leg is swinging.
            det = np.empty((n, 3), dtype=np.uint8)
            det[:] = _C_BG
            det[swing_det == 1] = _C_DETECT
            img[y0 + _H_CONTACT:y0 + _H_CONTACT + _H_DETECT, :] = det[None, :, :]

        self._img.setImage(img, autoLevels=False)
        self._vb.setRange(xRange=(0, n), yRange=(0, n_legs * _H_LEG), padding=0)

        self._title.setText(
            f"Footfall raster — last {n} ticks   ({'  '.join(_LEG_NAMES[:n_legs])} top→bottom)")
        self._readout.setText(self._summarize(snap, words, n_legs))

    # ------------------------------------------------------------------
    @staticmethod
    def _summarize(snap: dict, words: np.ndarray, n_legs: int) -> str:
        """The numbers that say whether the lock is installed, and whether it helps.

        Deliberately reports BOTH, labelled, because they are not the same question:
        with a touchdown-referenced phase `td_plv` and `pos_stance` become deterministic
        functions of `stroke_phase` and the duty factor — consumer verification, not
        evidence.  `mv_stance`/`mv_swing` are computed on ACHIEVED motion and are the
        honest ones.
        """
        def f(key, default=0.0):
            try:
                return float(snap.get(key, default) or default)
            except (TypeError, ValueError):
                return default

        # Fraction of each leg's AIRBORNE time spent pushing — the waste, straight from
        # the picture rather than from a separate accumulator.
        wasted, productive = [], []
        for leg in range(n_legs):
            contact = (words >> (_BIT_CONTACT + leg)) & 1
            push = (words >> (_BIT_STROKE + leg)) & 1
            air = int((contact == 0).sum())
            gnd = int((contact == 1).sum())
            wasted.append(int(((contact == 0) & (push == 1)).sum()) / air if air else 0.0)
            productive.append(int(((contact == 1) & (push == 1)).sum()) / gnd if gnd else 0.0)

        src = int(f("step_phase_src"))
        src_name = {0: "L.phase (knee, legacy)", 1: "step clock (contact)",
                    2: "step clock (hip1 load)"}.get(src, f"?{src}")
        lock = f("step_lock")
        lines = [
            f"  phase source : {src_name}",
            f"  step_lock    : {lock:5.2f}   (legs with a locked step clock; 0 = "
            f"{'unwired sensor' if src else 'lever off'})",
            f"  step_period  : {f('step_period'):5.1f} ticks     stroke rides: "
            f"{'the step' if src and lock > 0 else 'the knee'}",
            "",
            "  ── installed? (TAUTOLOGICAL once locked — verification only) ──",
            f"  td_plv       : {f('stroke_td_plv'):5.2f}   pos_stance {f('stroke_pos_stance'):.2f}"
            f" / pos_swing {f('stroke_pos_swing'):.2f}",
            "",
            "  ── does it help? (ACHIEVED motion — the honest read) ──",
            f"  mv_stance    : {f('mv_stance'):+7.4f}   (planted foot travel; want NEGATIVE = pushing)",
            f"  mv_swing     : {f('mv_swing'):+7.4f}   (airborne foot travel; want POSITIVE)",
            f"  separation   : {f('mv_swing') - f('mv_stance'):+7.4f}   (0 = the leg scrubs "
            "whatever the command says)",
            "",
            "  push wasted in the air, per leg:  "
            + "  ".join(f"{_LEG_NAMES[i]} {wasted[i]:.0%}" for i in range(n_legs)),
            "  push landing on the ground     :  "
            + "  ".join(f"{_LEG_NAMES[i]} {productive[i]:.0%}" for i in range(n_legs)),
        ]
        return "\n".join(lines)
