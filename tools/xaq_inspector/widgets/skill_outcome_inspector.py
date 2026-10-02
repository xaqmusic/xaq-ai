"""SkillOutcomeLoop dashboard — what the duck has learned each intent does to each kind of thing (2026-10-02).

At an arrival the outcome loop asks for an intent by name (kick, peck, push) on the thing it reached — the one whose
answer for this KIND it knows least — and then watches: the thing's next sighting within match_radius of where it
was, inside observe_ticks, is the outcome (how far it moved); unseen is UNKNOWN, not zero.  Each (kind, intent) cell
keeps a running mean and spread of the displacement; a cell with min_samples answers is KNOWN, and a kind whose
intents are all known loses its pull — habituation.  The surprise is an answer against its cell's own spread.

snapshot (diag_snapshot): requests, observed, unknown, pending, armed, misses, node (the kind now attended), need,
  surprise, nodes_known, peck_outcomes, push_outcomes, stats {"node:intent": {n, mean, sd}}, last {node, pred, obs,
  surprise}, min_samples, kicked_node, tx/ty

Panels:
  * The outcome table: one row per kind, one column per intent — n, mean displacement ± spread; green = known,
    amber = tried, dark = never asked.  The attended kind's row is marked.
  * need and surprise over time.
  * Readout: requests, answers seen, unknowns, the last answer.
"""
from __future__ import annotations

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QBrush, QColor
from PyQt6.QtWidgets import (QAbstractItemView, QHeaderView, QLabel, QSplitter, QTableWidget, QTableWidgetItem,
                             QVBoxLayout, QWidget)

from ._kv_readout import AMBER, BLUE, GREEN, GREY, Banner, KVReadout, flag, num
from ._multi_series import MultiSeriesPlot, Series

_INTENTS = ["kick", "peck", "push"]


class _OutcomeTable(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self._banner = Banner("no request yet")
        layout.addWidget(self._banner)
        self._table = QTableWidget(0, len(_INTENTS))
        self._table.setHorizontalHeaderLabels(_INTENTS)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self._table.setStyleSheet("QTableWidget { background:#15171c; color:#ddd; gridline-color:#333; }"
                                  "QHeaderView { background:#15171c; } QHeaderView::section { background:#22252c; color:#bbb; }"
                                  "QTableCornerButton::section { background:#22252c; }")
        layout.addWidget(self._table, 1)
        note = QLabel("cell: answers n · mean displacement ± spread (m).  green = known, amber = tried, dark = never asked")
        note.setStyleSheet("color:#888; font-size:11px;")
        layout.addWidget(note)
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
        need = num(s, "need")
        if s.get("pending"):
            self._banner.set(f"WATCHING for the answer — kind {int(num(s, 'kicked_node', -1))}", BLUE)
        elif s.get("armed"):
            self._banner.set("a request is out — waiting for the skill", AMBER)
        elif need > 0:
            self._banner.set(f"need {need:.2f} — something left to learn at kind {int(num(s, 'node', -1))}", GREEN)
        else:
            self._banner.set("habituated here — nothing left to ask", GREY)

        stats = s.get("stats") or {}
        cells: dict[int, dict[str, dict]] = {}
        for key, st in stats.items():
            node, _, intent = str(key).partition(":")
            try:
                cells.setdefault(int(node), {})[intent] = st
            except ValueError:
                continue
        attended = int(num(s, "node", -1))
        nodes = sorted(set(cells) | ({attended} if attended >= 0 else set()))
        min_n = int(num(s, "min_samples", 2))
        self._table.setRowCount(len(nodes))
        self._table.setVerticalHeaderLabels([f"kind {n}" + ("  ◀" if n == attended else "") for n in nodes])
        for r, node in enumerate(nodes):
            for c, intent in enumerate(_INTENTS):
                st = cells.get(node, {}).get(intent)
                if st is None:
                    item = QTableWidgetItem("—")
                    item.setBackground(QBrush(QColor(30, 32, 38)))
                else:
                    n = int(st.get("n", 0))
                    item = QTableWidgetItem(f"{n} · {float(st.get('mean', 0)):.3f} ± {float(st.get('sd', 0)):.3f}")
                    item.setBackground(QBrush(QColor(25, 90, 60) if n >= min_n else QColor(110, 80, 20)))
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self._table.setItem(r, c, item)


def _rows(s: dict):
    last = s.get("last") or {}
    return [
        ("requests", f"{int(num(s, 'requests'))}"),
        ("answers seen", f"{int(num(s, 'observed'))}   unknown {int(num(s, 'unknown'))}   misses {int(num(s, 'misses'))}"),
        ("pecks · pushes", f"{int(num(s, 'peck_outcomes'))} · {int(num(s, 'push_outcomes'))} answered"),
        ("cells known", f"{int(num(s, 'nodes_known'))}   (min {int(num(s, 'min_samples', 2))} answers)"),
        ("pending", flag(s, "pending")),
        ("armed", flag(s, "armed")),
        ("attended kind", f"{int(num(s, 'node', -1))}"),
        ("need", f"{num(s, 'need'):.3f}"),
        ("", ""),
        ("last answer", f"kind {int(last.get('node', -1))}: predicted {float(last.get('pred', 0)):.3f} m, "
                        f"saw {float(last.get('obs', 0)):.3f} m"),
        ("surprise", f"{float(last.get('surprise', 0)):.3f}"),
    ]


class SkillOutcomeInspector(QWidget):
    def __init__(self, module_id: str, module_type: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.module_id = module_id
        self.module_type = module_type
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        header = QLabel(f"{module_id}  ({module_type})")
        header.setStyleSheet("color:#ddd; font-weight:bold;")
        outer.addWidget(header)
        self._table = _OutcomeTable()
        self._readout = KVReadout(_rows)
        self._series = MultiSeriesPlot(
            [
                Series("need", "need", (27, 175, 122), width=2.0),
                Series("surprise", "surprise", (224, 85, 85), width=1.5),
                Series("pending", "watching", (74, 144, 217), width=1.2),
            ],
            title="The need to learn, and the surprise of each answer",
            y_label="value",
        )
        top = QSplitter(Qt.Orientation.Horizontal)
        top.addWidget(self._table)
        top.addWidget(self._readout)
        top.setSizes([600, 420])
        v = QSplitter(Qt.Orientation.Vertical)
        v.addWidget(top)
        v.addWidget(self._series)
        v.setSizes([420, 240])
        outer.addWidget(v, 1)

    def update_payload(self, tick_id: int, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._table.update_payload(snapshot)
        self._readout.update_payload(snapshot)
        self._series.update_payload(snapshot)
