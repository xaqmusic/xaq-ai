"""The left pane: every signal the brain actually publishes, live.

This is the answer to "what can I even modulate with?", and it is built entirely from
observed frames — the engine reports what arrived, never a compiled-in list, so this pane
is correct for whatever brain config happens to be running.

A module that was subscribed and published nothing is shown greyed rather than omitted,
with the file to fix it in the tooltip.  That distinction is the whole difference between
a known gap and an oscillator that mysteriously never sounds.
"""
from __future__ import annotations

from typing import Callable

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QBrush, QColor, QFont
from PyQt6.QtWidgets import (QHBoxLayout, QHeaderView, QLabel, QPushButton, QTreeWidget,
                             QTreeWidgetItem, QVBoxLayout, QWidget)

from ..theme import INK_MUTED, INK_PRIMARY, INK_SECOND
from ._controls import Sparkline, heading, muted

_ROLE_PATH = Qt.ItemDataRole.UserRole + 1


class SourcesPanel(QWidget):
    def __init__(self, on_add_route: Callable[[str], None], parent: QWidget | None = None):
        super().__init__(parent)
        self.on_add_route = on_add_route
        self._items: dict[str, QTreeWidgetItem] = {}
        self._sparks: dict[str, Sparkline] = {}
        self._modules: dict[str, QTreeWidgetItem] = {}

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 4, 8)
        lay.setSpacing(6)

        head = QHBoxLayout()
        head.addWidget(heading("Sources"))
        head.addStretch(1)
        self.count = muted("")
        head.addWidget(self.count)
        lay.addLayout(head)

        self.tree = QTreeWidget()
        self.tree.setColumnCount(3)
        self.tree.setHeaderLabels(["signal", "value", "history"])
        self.tree.setRootIsDecorated(True)
        self.tree.setAlternatingRowColors(False)
        self.tree.setUniformRowHeights(False)
        hdr = self.tree.header()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)
        hdr.setSectionResizeMode(2, QHeaderView.ResizeMode.Fixed)
        self.tree.setColumnWidth(1, 84)
        self.tree.setColumnWidth(2, 96)
        self.tree.itemDoubleClicked.connect(self._on_double_click)
        lay.addWidget(self.tree, 1)

        self.hint = muted("double-click a signal to route it to the selected voice")
        self.hint.setWordWrap(True)
        lay.addWidget(self.hint)

    # ------------------------------------------------------------------ build
    def rebuild(self, modules: list[dict]) -> None:
        """Called when the source set changes, which is rare — on connect and reload."""
        self.tree.clear()
        self._items.clear()
        self._sparks.clear()
        self._modules.clear()
        n_sources = 0

        mono = QFont("monospace")
        mono.setStyleHint(QFont.StyleHint.Monospace)

        for m in modules:
            mid = m.get("module", "")
            mtype = m.get("type", "")
            keys = m.get("keys") or []
            top = QTreeWidgetItem([f"{mid}    {mtype}", "", ""])
            top.setForeground(0, QBrush(QColor(INK_PRIMARY if keys else INK_MUTED)))
            top.setFirstColumnSpanned(False)
            self.tree.addTopLevelItem(top)
            self._modules[mid] = top

            if not keys:
                top.setText(1, "silent")
                top.setToolTip(
                    0,
                    f"{mid} ({mtype}) publishes nothing on the 'lite' topic.\n"
                    f"Add a diag_lite() override to cpp_core/src/ogma/modules/{mtype}.cpp\n"
                    "to make its signals routable here.")
                continue

            top.setExpanded(True)
            for k in keys:
                key = k.get("key", "")
                path = f"{mid}.{key}"
                item = QTreeWidgetItem([key, "", ""])
                item.setData(0, _ROLE_PATH, path)
                item.setFont(1, mono)
                item.setForeground(0, QBrush(QColor(INK_SECOND)))
                lo, hi = k.get("min", 0.0), k.get("max", 0.0)
                item.setToolTip(
                    0, f"{path}\nseen {k.get('seen', 0)} frames\n"
                       f"range {lo:.4g} … {hi:.4g}\nmedian {k.get('median', 0.0):.4g}  "
                       f"MAD {k.get('mad', 0.0):.4g}"
                       + ("\n(boolean)" if k.get("is_bool") else ""))
                top.addChild(item)
                spark = Sparkline()
                self.tree.setItemWidget(item, 2, spark)
                self._items[path] = item
                self._sparks[path] = spark
                n_sources += 1

        silent = sum(1 for m in modules if not (m.get("keys") or []))
        self.count.setText(f"{n_sources} signals · {len(modules)} modules"
                           + (f" · {silent} silent" if silent else ""))

    # ------------------------------------------------------------------ live
    def update_values(self, sources: list[dict]) -> None:
        for m in sources:
            mid = m.get("module", "")
            for key, val in (m.get("values") or {}).items():
                path = f"{mid}.{key}"
                item = self._items.get(path)
                if item is None:
                    continue
                try:
                    f = float(val)
                except (TypeError, ValueError):
                    continue
                item.setText(1, f"{f:.4g}")
                spark = self._sparks.get(path)
                if spark is not None:
                    spark.push(f)

    # ------------------------------------------------------------------ input
    def selected_path(self) -> str:
        it = self.tree.currentItem()
        return it.data(0, _ROLE_PATH) if it is not None else ""

    def _on_double_click(self, item: QTreeWidgetItem, _col: int) -> None:
        path = item.data(0, _ROLE_PATH)
        if path:
            self.on_add_route(path)
