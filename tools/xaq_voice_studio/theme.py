"""Palette and the surface base class.

The tokens are the ones already validated for the inspector's newest widget
(``tools/xaq_inspector/widgets/gain_evolver_inspector.py``) against
``tools/xaq_inspector/validate_palette.py`` — OKLab lightness band, chroma floor,
colour-vision-deficient separation, and contrast against the surface.  They are copied
rather than imported so the studio does not drag in a widget module (and pyqtgraph with
it) for eight strings; re-run the validator if any of them changes.

Two rules from that file carry over and are easy to lose:

* hues are assigned in **fixed order and never cycled**, so a colour belongs to a thing
  and hiding one thing never repaints the others;
* **text always wears an ink token**, with a small colour chip carrying identity — colour
  never carries meaning on its own.
"""
from __future__ import annotations

from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtWidgets import QWidget

SERIES_HEX = ["#3987e5", "#d95926", "#199e70", "#c98500",
              "#d55181", "#008300", "#9085e9", "#e66767"]
SERIES_RGB = [tuple(int(h[i:i + 2], 16) for i in (1, 3, 5)) for h in SERIES_HEX]

SURFACE     = "#1a1a19"    # the surface the palette was validated against
SURFACE_ALT = "#212120"
TRACK       = "#2a2d33"
LINE        = "#33363c"
INK_PRIMARY = "#e8eaed"
INK_SECOND  = "#cbd2dc"
INK_MUTED   = "#8b929c"
GOOD        = "#2f9e5f"
CRIT        = "#d1544f"
ACCENT      = "#3987e5"


def series_hex(i: int) -> str:
    return SERIES_HEX[i % len(SERIES_HEX)]


# Colour follows the destination, not a route's position in the rack, so reordering never
# repaints anything and the eye can follow one route as it moves.
#
# Only THREE hues, which is a measured limit rather than a stylistic choice.  The eight
# series colours above are validated for *adjacent* pairs — the right test for traces in a
# chart, where a series is compared with its neighbours.  A route rack shows every chip at
# once, so the right test is every pair, and under `--pairs all` only three of the eight
# stay separable:
#
#   python tools/xaq_inspector/validate_palette.py \
#       "#3987e5,#d95926,#199e70" --mode dark --surface "#1a1a19" --pairs all
#
# Adding a fourth fails on colour-vision deficiency every time — violet against blue is
# 1.9 dE under protanopia and pink against green is 1.6 under deuteranopia, both of which
# are "identical" in practice.  That is the well-known collapse of the red-green axis,
# not a shortcoming of these particular swatches.
#
# So destinations are grouped by WHAT THEY MODULATE, and the chip is a secondary cue: the
# destination's NAME sits beside it in the combo box at all times, because colour must
# never carry meaning on its own.
_PITCH_HUE = SERIES_HEX[0]   # blue   — where the note sits
_LEVEL_HUE = SERIES_HEX[1]   # orange — how loud it is
_SHAPE_HUE = SERIES_HEX[2]   # green  — what it sounds like, and where

DEST_COLOR = {
    "pitch":       _PITCH_HUE,
    "detune":      _PITCH_HUE,
    "amp":         _LEVEL_HUE,
    "level":       _LEVEL_HUE,
    "cutoff":      _SHAPE_HUE,
    "resonance":   _SHAPE_HUE,
    "pulse_width": _SHAPE_HUE,
    "noise_mix":   _SHAPE_HUE,
    "vowel_morph": _SHAPE_HUE,
    "pan":         _SHAPE_HUE,
}


def dest_color(dest: str) -> str:
    return DEST_COLOR.get(dest, INK_MUTED)


QSS = f"""
QWidget           {{ background: {SURFACE}; color: {INK_SECOND}; }}
QMainWindow       {{ background: {SURFACE}; }}
QLabel            {{ color: {INK_SECOND}; }}
QLabel#h1         {{ color: {INK_PRIMARY}; font-weight: bold; font-size: 13px; }}
QLabel#h2         {{ color: {INK_PRIMARY}; font-weight: bold; }}
QLabel#muted      {{ color: {INK_MUTED}; }}
QLabel#value      {{ color: {INK_PRIMARY}; font-family: monospace; }}

QGroupBox         {{ border: 1px solid {LINE}; border-radius: 4px; margin-top: 14px;
                     padding-top: 8px; color: {INK_MUTED}; }}
QGroupBox::title  {{ subcontrol-origin: margin; left: 8px; padding: 0 4px;
                     color: {INK_MUTED}; }}

QTreeWidget, QListWidget {{ background: {SURFACE_ALT}; border: 1px solid {LINE};
                            alternate-background-color: {SURFACE}; }}
QTreeWidget::item:selected, QListWidget::item:selected {{ background: #2a4060;
                                                          color: {INK_PRIMARY}; }}
QHeaderView::section {{ background: {SURFACE}; color: {INK_MUTED};
                        border: 0; border-bottom: 1px solid {LINE}; padding: 3px; }}

QPushButton       {{ background: {TRACK}; color: {INK_PRIMARY}; border: 1px solid {LINE};
                     border-radius: 3px; padding: 4px 10px; }}
QPushButton:hover {{ background: #345070; }}
QPushButton:disabled {{ color: {INK_MUTED}; background: {SURFACE_ALT}; }}
QPushButton:checked  {{ background: #2a4060; border-color: {ACCENT}; }}

QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
    background: {SURFACE_ALT}; color: {INK_PRIMARY}; border: 1px solid {LINE};
    border-radius: 3px; padding: 2px 4px; }}
QComboBox QAbstractItemView {{ background: {SURFACE_ALT}; color: {INK_PRIMARY};
                               selection-background-color: #2a4060; }}

QSlider::groove:horizontal {{ background: {TRACK}; height: 4px; border-radius: 2px; }}
QSlider::handle:horizontal {{ background: {INK_SECOND}; width: 10px; margin: -5px 0;
                              border-radius: 5px; }}
QSlider::sub-page:horizontal {{ background: {ACCENT}; height: 4px; border-radius: 2px; }}

QCheckBox         {{ color: {INK_SECOND}; }}
QScrollArea       {{ border: 0; }}
QSplitter::handle {{ background: {LINE}; }}
QStatusBar        {{ color: {INK_MUTED}; }}
QToolTip          {{ background: {SURFACE_ALT}; color: {INK_PRIMARY};
                     border: 1px solid {LINE}; }}
"""


class SurfaceWidget(QWidget):
    """Base for anything that paints itself.

    Qt's default widget background is light.  A custom-painted panel that does not set
    its own background inherits it, and every ink token then becomes invisible against
    it — which looks like the widget failed to draw rather than like a colour mistake.
    """

    def __init__(self, parent: QWidget | None = None, color: str = SURFACE):
        super().__init__(parent)
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QPalette.ColorRole.Window, QColor(color))
        self.setPalette(pal)
