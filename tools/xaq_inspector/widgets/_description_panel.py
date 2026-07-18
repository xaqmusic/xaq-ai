"""Collapsible educational description panel + card wrapper.

Every module inspector is wrapped in an :class:`InspectorCard`, which stacks
a :class:`DescriptionPanel` above the live widget. The panel shows a friendly
title and a plain-language summary at all times, plus a "Formulas & details"
section that expands on demand (kept collapsed by default so it never crowds
the live visualisation).

The card forwards ``update_payload`` to the wrapped widget, so the inspector
window can keep treating it as an ordinary module widget.
"""
from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QFrame, QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget,
)

from .descriptions import ModuleDoc, doc_for


_PANEL_QSS = """
#descCard { background: #1d2027; border: 1px solid #2c313a;
            border-radius: 4px; }
#descTitle { color: #e6b800; font-weight: bold; font-size: 12px; }
#descSummary { color: #cbd2dc; font-size: 12px; }
#descToggle { background: transparent; color: #7fa8d8; border: none;
              padding: 2px 0; text-align: left; font-size: 11px; }
#descToggle:hover { color: #a9c8ee; }
#descFormulas { color: #c7cdd6; font-size: 12px; }
#descFormulas code { color: #e0c88a;
                     font-family: "DejaVu Sans Mono", Menlo, Consolas, monospace; }
#descFormScroll { background: #16181d; border: 1px solid #262b33;
                  border-radius: 3px; }
"""


class DescriptionPanel(QFrame):
    """Title + always-on layman summary + collapsible formula sheet."""

    def __init__(self, doc: ModuleDoc, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("descCard")
        self.setStyleSheet(_PANEL_QSS)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)

        title = QLabel(doc.title)
        title.setObjectName("descTitle")
        title.setWordWrap(True)
        lay.addWidget(title)

        summary = QLabel(doc.summary)
        summary.setObjectName("descSummary")
        summary.setTextFormat(Qt.TextFormat.RichText)
        summary.setWordWrap(True)
        summary.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        lay.addWidget(summary)

        # '&&' renders a literal '&' (a lone '&' becomes a QPushButton mnemonic).
        self._toggle = QPushButton("▸  Formulas && details")
        self._toggle.setObjectName("descToggle")
        self._toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self._toggle.clicked.connect(self._on_toggle)
        lay.addWidget(self._toggle)

        # Formula sheet lives inside a height-capped scroll area so a long
        # derivation never pushes the live plots off-screen.
        formulas = QLabel(doc.formulas)
        formulas.setObjectName("descFormulas")
        formulas.setTextFormat(Qt.TextFormat.RichText)
        formulas.setWordWrap(True)
        formulas.setAlignment(
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        formulas.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        formulas.setContentsMargins(8, 6, 8, 6)

        self._scroll = QScrollArea()
        self._scroll.setObjectName("descFormScroll")
        self._scroll.setWidgetResizable(True)
        self._scroll.setMaximumHeight(280)
        self._scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self._scroll.setWidget(formulas)
        self._scroll.setVisible(False)
        lay.addWidget(self._scroll)

    def _on_toggle(self) -> None:
        # Use isHidden() (the explicit-hidden flag), not isVisible(): the latter
        # is False whenever an ancestor is unshown, which would wedge the toggle.
        show = self._scroll.isHidden()
        self._scroll.setVisible(show)
        arrow = "▾" if show else "▸"  # ▾ open / ▸ closed
        self._toggle.setText(f"{arrow}  Formulas && details")


class InspectorCard(QWidget):
    """Stacks a DescriptionPanel above a live module widget.

    Exposes ``update_payload`` so the inspector window can drive the wrapped
    widget without knowing it has been decorated.
    """

    def __init__(self, inner: QWidget, module_type: str,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._inner = inner

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(6)
        lay.addWidget(DescriptionPanel(doc_for(module_type)))
        lay.addWidget(inner, 1)

    def update_payload(self, tick_id: int, snapshot) -> None:
        if hasattr(self._inner, "update_payload"):
            self._inner.update_payload(tick_id, snapshot)


def wrap_with_description(inner: QWidget, module_type: str,
                         parent: Optional[QWidget] = None) -> InspectorCard:
    """Decorate a freshly-built module widget with its description panel."""
    return InspectorCard(inner, module_type, parent=parent)
