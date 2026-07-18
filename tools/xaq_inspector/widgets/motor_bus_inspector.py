"""MotorBus meter bridge — the mixing-console viz for the motor influencer bus.

A classic console meter bridge: one stereo (L/R) level meter per influencer
showing its POST-FADER contribution to the two motor channels, a downward
gain-reduction meter (compressor GR), and a master OUTPUT stereo meter.

Scale is dB-like with the compressor `limit` = 0 dBFS (the ceiling the tanh
saturates at).  Inputs can run HOT — above 0 dB, into the red — which is exactly
what drives the bus into gain reduction; the master output, post-compressor,
sits at or below 0 dB.  Two channels per meter (L/R) keeps the stereo analogy;
a multi-motor body (picrawler) would aggregate its left/right motors into the
two channels upstream, so the same bridge reads as bilateral averages.

Reads MotorBus.diag_snapshot(): names[], contrib_l[], contrib_r[], gains[],
active[], out_l, out_r, gr, limit.
"""
from __future__ import annotations

import math

from PyQt6.QtCore import Qt, QTimer, QRectF
from PyQt6.QtGui import QColor, QLinearGradient, QPainter, QPen
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget


DB_FLOOR = -40.0   # bottom of the meter
DB_CEIL  = 18.0    # top of the meter (headroom above 0 dB for hot inputs)
_EPS     = 1e-5


def _db_frac(val: float, ref: float) -> float:
    """|val| → [0,1] meter fraction on a dB scale with 0 dB at `ref`."""
    a = abs(float(val))
    if a < _EPS or ref < _EPS:
        return 0.0
    db = 20.0 * math.log10(a / ref)
    return max(0.0, min(1.0, (db - DB_FLOOR) / (DB_CEIL - DB_FLOOR)))


def _zero_db_frac() -> float:
    return (0.0 - DB_FLOOR) / (DB_CEIL - DB_FLOOR)   # where the 0 dB line sits


class _MeterBridge(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumHeight(240)
        self.setMinimumWidth(360)
        self._snap: dict | None = None
        self._peaks: dict[str, float] = {}   # key "i:L" -> held frac
        self._refresh = QTimer(self)
        self._refresh.setInterval(50)
        self._refresh.timeout.connect(self.update)
        self._refresh.start()

    def update_payload(self, snapshot: dict) -> None:
        if isinstance(snapshot, dict):
            self._snap = snapshot

    def _peak(self, key: str, frac: float) -> float:
        held = self._peaks.get(key, 0.0) * 0.90   # decay
        held = max(held, frac)
        self._peaks[key] = held
        return held

    def paintEvent(self, _ev) -> None:
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(14, 14, 16))
        snap = self._snap
        if not isinstance(snap, dict) or not snap.get("names"):
            p.setPen(QColor(150, 150, 150))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "awaiting MotorBus…")
            p.end()
            return

        names = snap.get("names") or []
        cl = snap.get("contrib_l") or []
        cr = snap.get("contrib_r") or []
        gains = snap.get("gains") or []
        active = snap.get("active") or []
        # Levels are NORMALIZED (post-fader, ±1 = unity); 0 dB = unity ref.
        ref = float(snap.get("unity", 1.0) or 1.0)
        n = len(names)

        # groups: N influencer stereo meters, 1 GR meter, 1 master OUT stereo.
        groups = n + 2
        W = self.width()
        H = self.height()
        pad = 8
        label_h = 26
        top = pad
        bottom = H - label_h - pad
        bar_area_h = max(10.0, bottom - top)
        gw = (W - 2 * pad) / groups          # group width

        zfrac = _zero_db_frac()

        def draw_stereo(gx: float, vals, keys, label: str, gain_txt: str,
                        is_out: bool, dim: bool):
            # two bars (L,R) within the group slot
            inner = gw * 0.78
            bx0 = gx + (gw - inner) / 2.0
            bw = inner * 0.42
            gap = inner * 0.16
            for ch, (v, key) in enumerate(zip(vals, keys)):
                bxx = bx0 + ch * (bw + gap)
                frac = _db_frac(v, ref)
                # bar background
                p.fillRect(QRectF(bxx, top, bw, bar_area_h), QColor(30, 30, 34))
                # Sign-aware gradient (bottom→top), positioned over the full bar.
                # POSITIVE (forward): green→amber→red (hot = into the limit).
                # NEGATIVE (reverse): teal→blue→violet — the cool palette so the
                # console reads direction at a glance.
                grad = QLinearGradient(0, bottom, 0, top)
                if float(v) >= 0.0:
                    lo  = QColor(60, 200, 90)  if not dim else QColor(45, 110, 65)
                    mid = QColor(210, 200, 70) if not dim else QColor(120, 115, 55)
                    hi  = QColor(230, 70, 60)  if not dim else QColor(130, 55, 50)
                else:
                    lo  = QColor(60, 200, 210) if not dim else QColor(45, 110, 115)
                    mid = QColor(70, 130, 235) if not dim else QColor(50, 80, 130)
                    hi  = QColor(170, 90, 240) if not dim else QColor(95, 55, 130)
                grad.setColorAt(0.0, lo)
                grad.setColorAt(max(0.0, zfrac - 0.12), lo)
                grad.setColorAt(zfrac, mid)
                grad.setColorAt(1.0, hi)
                fh = frac * bar_area_h
                p.fillRect(QRectF(bxx, bottom - fh, bw, fh), grad)
                # peak hold marker
                ph = self._peak(key, frac)
                py = bottom - ph * bar_area_h
                p.setPen(QPen(QColor(235, 235, 245), 1))
                p.drawLine(int(bxx), int(py), int(bxx + bw), int(py))
            # 0 dB reference line across the group
            zy = bottom - zfrac * bar_area_h
            p.setPen(QPen(QColor(120, 120, 130), 1, Qt.PenStyle.DashLine))
            p.drawLine(int(bx0), int(zy), int(bx0 + inner), int(zy))
            # labels
            p.setPen(QColor(210, 210, 220) if not dim else QColor(120, 120, 130))
            p.drawText(QRectF(gx, bottom + 2, gw, 12),
                       Qt.AlignmentFlag.AlignCenter, label)
            if gain_txt:
                p.setPen(QColor(150, 170, 210))
                p.drawText(QRectF(gx, bottom + 13, gw, 12),
                           Qt.AlignmentFlag.AlignCenter, gain_txt)

        # influencer meters
        for i in range(n):
            gx = pad + i * gw
            vl = cl[i] if i < len(cl) else 0.0
            vr = cr[i] if i < len(cr) else 0.0
            g = gains[i] if i < len(gains) else 1.0
            act = bool(active[i]) if i < len(active) else False
            draw_stereo(gx, [vl, vr], [f"{i}:L", f"{i}:R"],
                        str(names[i]), f"×{float(g):.2f}",
                        is_out=False, dim=(not act))

        # gain-reduction meter (downward from top, red)
        gx = pad + n * gw
        gr = float(snap.get("gr", 0.0) or 0.0)
        inner = gw * 0.5
        bx0 = gx + (gw - inner) / 2.0
        p.fillRect(QRectF(bx0, top, inner, bar_area_h), QColor(30, 30, 34))
        grh = gr * bar_area_h
        p.fillRect(QRectF(bx0, top, inner, grh), QColor(230, 90, 70))
        p.setPen(QColor(210, 210, 220))
        p.drawText(QRectF(gx, bottom + 2, gw, 12),
                   Qt.AlignmentFlag.AlignCenter, "GR")
        p.setPen(QColor(230, 140, 120))
        p.drawText(QRectF(gx, bottom + 13, gw, 12),
                   Qt.AlignmentFlag.AlignCenter, f"{gr * 100:.0f}%")

        # master output stereo
        gx = pad + (n + 1) * gw
        draw_stereo(gx, [snap.get("out_l", 0.0), snap.get("out_r", 0.0)],
                    ["out:L", "out:R"], "OUT", "", is_out=True, dim=False)

        p.end()


class MotorBusInspector(QWidget):
    def __init__(self, module_id: str, module_type: str,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})   — meter bridge "
                        "(0 dB = compressor limit; L/R per channel)")
        header.setStyleSheet("color: #ddd; font-weight: bold;")
        outer.addWidget(header)

        self._bridge = _MeterBridge()
        outer.addWidget(self._bridge, 1)

        self._readout = QLabel("—")
        self._readout.setStyleSheet(
            "color: #bbb; font-family: Monospace; font-size: 11px;")
        outer.addWidget(self._readout)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._bridge.update_payload(snapshot)
        names = snapshot.get("names") or []
        gains = snapshot.get("gains") or []
        parts = []
        for i, nm in enumerate(names):
            g = gains[i] if i < len(gains) else 1.0
            parts.append(f"{nm}×{float(g):.2f}")
        self._readout.setText(
            "  ".join(parts)
            + f"   |  GR {float(snapshot.get('gr',0.0))*100:.0f}%"
            + f"   OUT L{float(snapshot.get('out_l',0.0)):+.2f} "
            f"R{float(snapshot.get('out_r',0.0)):+.2f}"
            + f"   limit {float(snapshot.get('limit',4.0)):.1f}")
