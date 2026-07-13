"""
v3_observer_window.py — V3BrainServerWindow

Observable control panel for AMI-Ogma v3 multi-EPM Breakout sessions.

Composes v2 brain server widgets with new v3-specific panels.
No BrainSocketServer, no LateralVoterNode, no socket ports.

Data flow:
    GameLoopDriver (QTimer @ fps Hz)
        → adapter.process_chunk()       [adapter._last_stats updated]
        → cv2.imshow(frame)             [game in its own window]
        → window.notify_viz(viz)        [topology every 5 ticks]

    V3BrainServerWindow._update_frame() (QTimer @ 30 Hz)
        → reads adapter._last_stats + per_mod_stats + voter._assoc
        → pushes to all widget update methods
"""

import numpy as np
from PyQt6.QtWidgets import QMainWindow, QDockWidget, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget, QTableWidgetItem, QHeaderView, QPushButton
from PyQt6.QtCore import Qt, QTimer, QSettings
from PyQt6.QtGui import QFont

# v2 widgets reused directly
from zanshin.server.ui.widgets.epm_list_widget  import EPMListWidget
from zanshin.server.ui.widgets.resonance_plot   import ResonancePlotWidget
from zanshin.server.ui.widgets.matrix_view      import MatrixViewWidget
from zanshin.server.ui.widgets.consensus_vector import ConsensusVectorWidget

# v3-specific widgets
from zanshin.observer.widgets.gng_lifecycle_widget   import GNGLifecycleWidget
from zanshin.observer.widgets.motor_inference_widget import MotorInferenceWidget


# ---------------------------------------------------------------------------
# Simple v3 stats table (replaces ConsensusLogWidget which needs v2 voter)
# ---------------------------------------------------------------------------

class _V3StatsWidget(QWidget):
    """Live key/value table showing fused EPM stats."""

    _KEYS = [
        ("tle",                  "Fused TLE"),
        ("threshold",            "Threshold"),
        ("gng_nodes",            "Total Nodes"),
        ("gng_baked",            "Baked Nodes"),
        ("crystallization_ratio","Cryst. Ratio"),
        ("mitosis_count",        "Mitosis Count"),
        ("active_modality",      "Active EPM"),
        ("is_novel",             "Novel"),
        ("is_mature",            "Mature"),
        # Neurochemical
        ("dopamine",             "Dopamine"),
        ("serotonin",            "Serotonin"),
        ("chem_hit_rate",        "Hit Rate"),
        # Action decoder
        ("learned_associations", "Learned Assoc."),
        ("reflex_fraction",      "Reflex Frac."),
        ("explore_sigma",        "Explore σ"),
        ("reflex_ratchet",       "Reflex Ratchet"),
        # Consistency gate (B2)
        ("consistency",          "D/V Consistency"),
        ("tle_divergence",       "TLE Divergence"),
        ("violations",           "Violations"),
        # Hebbian resonance (B4)
        ("resonance_score",      "Heb. Resonance"),
        ("hebbian_entries",      "Heb. Entries"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)

        hdr = QLabel("Fused EPM Stats")
        hdr.setStyleSheet("font-weight: bold; font-size: 13px; color: #00CFFF;")
        layout.addWidget(hdr)

        self._table = QTableWidget(len(self._KEYS), 2)
        self._table.setHorizontalHeaderLabels(["Field", "Value"])
        self._table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self._table.setStyleSheet("background:#121212; color:#cccccc; font-family:monospace; font-size:10px;")
        for r, (_, label) in enumerate(self._KEYS):
            self._table.setItem(r, 0, QTableWidgetItem(label))
            self._table.setItem(r, 1, QTableWidgetItem("—"))
        layout.addWidget(self._table)

    def update_stats(self, stats: dict):
        for r, (key, _) in enumerate(self._KEYS):
            val = stats.get(key, "—")
            if isinstance(val, float):
                txt = f"{val:.4f}"
            else:
                txt = str(val)
            item = self._table.item(r, 1)
            if item:
                item.setText(txt)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class V3BrainServerWindow(QMainWindow):
    """
    Observable control panel for an AMI-Ogma v3 multi-EPM session.

    Parameters
    ----------
    adapter    : MultiEPMAdapter | CppEPMAdapter
    modalities : list[str]
    game       : ObservableBreakout   (for reading ball/paddle state)
    game_size  : (w, h) pixels
    projection_dim : int  (latent vector dimension)
    """

    def __init__(self, adapter, modalities, game=None,
                 game_size=(320, 240), projection_dim=128,
                 chem=None, decoder=None):
        super().__init__()
        self._adapter   = adapter
        self._mods      = list(modalities)
        self._game      = game
        self._game_w    = game_size[0]
        self._proj_dim  = projection_dim
        self._chem      = chem
        self._decoder   = decoder
        self._latest_viz: dict = {}
        self._tick_count = 0
        import time as _time
        self._run_start_ts = _time.monotonic()

        self.setWindowTitle("AMI-Ogma v3 — Observer Panel")
        self.resize(1440, 900)
        # Persistent status-bar elapsed-time indicator. Refreshed every
        # _update_frame tick so the operator can see how long the current
        # session has been running without hunting in logs.
        self._elapsed_label = QLabel("elapsed: 0s")
        self._elapsed_label.setStyleSheet(
            "color: #ddd; font-family: monospace; padding: 0 8px;")
        self.statusBar().addPermanentWidget(self._elapsed_label)

        self._build_ui()
        self._restore_layout()

        self._ui_timer = QTimer(self)
        self._ui_timer.setInterval(67)          # ~15 Hz — halved to give the 20Hz
        self._ui_timer.timeout.connect(self._update_frame)  # game QTimer more headroom
        self._ui_timer.start()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        # ---- Motor Inference — central widget ----
        self._motor = MotorInferenceWidget(
            self._mods, game_w=self._game_w, window_size=300
        )
        self.setCentralWidget(self._motor)

        # ---- EPM Ensemble list (left) ----
        self._epm_list = EPMListWidget()
        self._epm_list.setHorizontalHeaderLabels(
            ["Modality", "State", "TLE", "Trust", "Nodes"]
        )
        self._add_dock("EPM Ensemble", "DockEPMEnsemble",
                       self._epm_list, Qt.DockWidgetArea.LeftDockWidgetArea)

        # ---- GNG Lifecycle (left, below EPM list) ----
        self._lifecycle = GNGLifecycleWidget(self._mods, window_size=300)
        self._add_dock("GNG Lifecycle", "DockLifecycle",
                       self._lifecycle, Qt.DockWidgetArea.LeftDockWidgetArea)

        # ---- Resonance History (bottom) ----
        self._resonance = ResonancePlotWidget(window_size=300)
        dock_res = self._add_dock("Resonance", "DockResonance",
                                  self._resonance, Qt.DockWidgetArea.BottomDockWidgetArea)
        # Color-coded title bar: each metric name matches its line color
        _title_label = QLabel(self._resonance.dock_title_html)
        _title_label.setTextFormat(Qt.TextFormat.RichText)
        _title_label.setContentsMargins(6, 2, 6, 2)
        _title_label.setStyleSheet(
            "background: #1e1e1e; border-bottom: 1px solid #333;"
        )
        dock_res.setTitleBarWidget(_title_label)

        # ---- Association Matrix (right) ----
        self._matrix = MatrixViewWidget(max_nodes=16)
        self._add_dock("Association Matrix", "DockMatrix",
                       self._matrix, Qt.DockWidgetArea.RightDockWidgetArea)

        # ---- Latent Vector heatmap (right, tabbed with matrix) ----
        self._vector = ConsensusVectorWidget(dim=self._proj_dim)
        dock_vec = self._add_dock("Latent Vector", "DockVector",
                                  self._vector, Qt.DockWidgetArea.RightDockWidgetArea)

        # Tab matrix and vector together
        dock_mat = self.findChild(QDockWidget, "DockMatrix")
        if dock_mat and dock_vec:
            self.tabifyDockWidget(dock_mat, dock_vec)
            dock_mat.raise_()

        # ---- v3 Stats (right, bottom) ----
        self._stats_w = _V3StatsWidget()
        self._add_dock("EPM Stats", "DockStats",
                       self._stats_w, Qt.DockWidgetArea.RightDockWidgetArea)

        # ---- Reflex kill-switch toggle ----
        self._reflex_disabled = False
        self._reflex_btn = QPushButton("Disable Reflex")
        self._reflex_btn.setCheckable(True)
        self._reflex_btn.setStyleSheet(
            "QPushButton { background: #333; color: #ccc; padding: 6px 12px; "
            "border: 1px solid #555; border-radius: 4px; font-weight: bold; }"
            "QPushButton:checked { background: #cc3300; color: #fff; border-color: #ff4400; }"
        )
        self._reflex_btn.toggled.connect(self._on_reflex_toggle)
        self._add_dock("Reflex Control", "DockReflexCtrl",
                       self._reflex_btn, Qt.DockWidgetArea.RightDockWidgetArea)

    def _on_reflex_toggle(self, checked: bool):
        self._reflex_disabled = checked
        if self._decoder is not None:
            self._decoder.set_reflex_disabled(checked)
        self._reflex_btn.setText(
            "Exploration DISABLED" if checked else "Disable Exploration"
        )

    def _add_dock(self, title, obj_name, widget, area):
        dock = QDockWidget(title, self)
        dock.setObjectName(obj_name)
        dock.setWidget(widget)
        dock.setAllowedAreas(Qt.DockWidgetArea.AllDockWidgetAreas)
        self.addDockWidget(area, dock)
        return dock

    # ------------------------------------------------------------------
    # Called by GameLoopDriver every 5 ticks
    # ------------------------------------------------------------------

    def notify_viz(self, viz: dict):
        """Store latest topology data; used by _update_frame for PCA scatter."""
        self._latest_viz = viz

    # ------------------------------------------------------------------
    # 30 Hz UI refresh
    # ------------------------------------------------------------------

    def _update_frame(self):
        import time as _time
        _el = int(_time.monotonic() - self._run_start_ts)
        _mm, _ss = divmod(_el, 60)
        _hh, _mm = divmod(_mm, 60)
        if _hh > 0:
            self._elapsed_label.setText(f"elapsed: {_hh:d}h{_mm:02d}m{_ss:02d}s")
        else:
            self._elapsed_label.setText(f"elapsed: {_mm:d}m{_ss:02d}s")

        stats = getattr(self._adapter, "_last_stats", None) or {}
        if not stats:
            return

        # Per-modality stats
        if hasattr(self._adapter, "get_per_modality_stats"):
            per_mod = self._adapter.get_per_modality_stats()
        else:
            per_mod = {self._mods[0]: stats}

        active_mod = stats.get("active_modality", self._mods[0])
        trust      = stats.get("trust_weights",
                               {m: 1.0 / max(1, len(self._mods)) for m in self._mods})
        active_node = stats.get("active_node", -1)

        # ---- EPM list ----
        for m in self._mods:
            ms    = per_mod.get(m, {})
            tle   = ms.get("tle", 0.0)
            tw    = trust.get(m, 0.0)
            nodes = ms.get("gng_nodes", ms.get("node_count", 0))
            novel = ms.get("is_novel", False)
            boot  = ms.get("is_bootstrapping", False)
            state = "NOVEL" if novel else ("BOOTSTRAPPING" if boot else "RUNNING")
            self._epm_list.update_agent(m, {
                "state":     state,
                "latency":   f"{tle:.4f}",
                "dopamine":  tw,
                "serotonin": float(nodes),
            })

        # ---- Resonance — trust entropy, inverted (consensus = high) ----
        tw_vals = np.array([trust.get(m, 0.0) for m in self._mods], dtype=np.float32)
        tw_vals = np.clip(tw_vals, 1e-9, 1.0)
        ent     = float(-np.sum(tw_vals * np.log(tw_vals)))
        max_ent = float(np.log(max(2, len(tw_vals))))
        resonance = 1.0 - (ent / max_ent if max_ent > 0 else 0.0)
        # Get dynamic reflex influence from the action decoder
        reflex_inf = None
        if self._decoder is not None:
            dd = self._decoder.to_dict()
            reflex_inf = dd.get("dynamic_reflex_inf", None)

        # Consistency gate score (B2)
        cg = stats.get("consistency_gate")
        consistency = cg.get("consistency", 1.0) if cg else None

        # Hebbian resonance (B4) — check visual sub-voter first, then main
        heb_res = stats.get("resonance_score")
        vcd = stats.get("visual_consensus_detail")
        # If visual sub-voter has its own resonance, prefer that
        vis_voter = getattr(self._adapter, "_visual_voter", None)
        if vis_voter is not None:
            heb_res = vis_voter._resonance_score

        self._resonance.update_resonance(
            resonance,
            threshold=stats.get("threshold", 0.5),
            tle=stats.get("tle", 0.0),
            reflex_inf=reflex_inf,
            consistency=consistency,
            heb_resonance=heb_res,
        )

        # ---- Association matrix (B4: sparse Hebbian / C: Motor Transitions) ----
        assoc = None
        if self._decoder is not None and len(self._decoder._table) > 0:
            # Display Motor Transitions from the Action Decoder (Preferred Source)
            raw_assoc = self._decoder._table
            # Condense 4-tuples (mod, prev, cur, p) -> 2D (prev, cur) for viz
            condensed = {}
            for (_, prev, cur, _), w in raw_assoc.items():
                if abs(w) < 0.005: continue
                # key is (prev, cur)
                condensed_key = (prev, cur)
                condensed[condensed_key] = max(condensed.get(condensed_key, 0), abs(w))
            
            if condensed:
                # Map to MatrixView format
                unique_ids = sorted(set([k[0] for k in condensed.keys()] + [k[1] for k in condensed.keys()]))
                idx_map = {nid: i for i, nid in enumerate(unique_ids)}
                n = len(unique_ids)
                if n > 0 and n <= self._matrix.max_nodes:
                    mat_d = {}
                    for (prev, cur), w in condensed.items():
                        i, j = idx_map[prev], idx_map[cur]
                        mat_d.setdefault(i, {})[j] = float(w)
                    self._matrix.update_matrix(mat_d, nav_speed=0.12, node_count=n)
        else:
            # Fallback to Voter-based cross-modal graph
            voter = getattr(self._adapter, "_visual_voter", None) or \
                    getattr(self._adapter, "_voter", None)
            if voter is not None and hasattr(voter, "_assoc") and isinstance(voter._assoc, dict):
                assoc = voter._assoc
                if assoc:
                    seen = set()
                    for (ma, na, mb, nb) in assoc:
                        seen.add((ma, na))
                        seen.add((mb, nb))
                    ids = sorted(seen)
                    idx_map = {k: i for i, k in enumerate(ids)}
                    n = len(ids)
                    if n > 0 and n <= self._matrix.max_nodes:
                        mat_d = {}
                        for (ma, na, mb, nb), w in assoc.items():
                            if w < 0.005: continue
                            i, j = idx_map[(ma, na)], idx_map[(mb, nb)]
                            mat_d.setdefault(i, {})[j] = float(w)
                            mat_d.setdefault(j, {})[i] = float(w)
                        self._matrix.update_matrix(mat_d, nav_speed=0.12, node_count=n)

        # ---- GNG lifecycle ----
        self._lifecycle.update_stats(per_mod)

        # ---- Motor inference ----
        self._motor.update_trust(trust, active_mod)

        if self._game is not None:
            try:
                gs = self._game.state()
                bx = float(gs["ball"][0])
                px = float(gs["p1_x"])
                self._motor.update_game_state(bx, px)
            except Exception:
                pass

        if self._latest_viz:
            self._motor.update_viz(self._latest_viz, active_node)

            # Feed fused embedding (if available) or active node embedding
            # to the vector heatmap.  Fused = trust-weighted blend of all
            # EPM latent vectors; falls back to single-EPM GNG prototype.
            fused = stats.get("fused_embedding")
            if fused is None:
                fused = stats.get("latent")
            if fused is not None and hasattr(fused, '__len__') and len(fused) > 0:
                self._vector.update_vector(np.asarray(fused))
            else:
                emb  = self._latest_viz.get("embeddings")
                idxs = self._latest_viz.get("indices")
                if emb is not None and idxs is not None and active_node >= 0:
                    try:
                        idx_arr = np.asarray(idxs)
                        hits = np.where(idx_arr == active_node)[0]
                        if len(hits) > 0:
                            self._vector.update_vector(emb[hits[0]])
                    except Exception:
                        pass

        # ---- Fused stats table (merge chem + decoder + gates) ----
        display_stats = dict(stats)
        if self._chem is not None:
            cd = self._chem.to_dict()
            display_stats["dopamine"]   = cd.get("dopamine",  0.0)
            display_stats["serotonin"]  = cd.get("serotonin", 0.0)
            display_stats["chem_hit_rate"] = cd.get("hit_rate", 0.0)
        if self._decoder is not None:
            dd = self._decoder.to_dict()
            display_stats["learned_associations"] = dd.get("learned_associations", 0)
            display_stats["reflex_fraction"]       = dd.get("reflex_fraction",      0.0)
            display_stats["explore_sigma"]         = dd.get("explore_sigma",        0.0)
            display_stats["reflex_ratchet"]        = dd.get("reflex_ratchet",       False)
        # Consistency gate (B2)
        cg = stats.get("consistency_gate")
        if cg:
            display_stats["consistency"]    = cg.get("consistency", 1.0)
            display_stats["tle_divergence"] = cg.get("tle_divergence", 0.0)
            display_stats["violations"]     = cg.get("total_violations", 0)
        # Hebbian resonance (B4)
        display_stats["resonance_score"]  = stats.get("resonance_score", 0.0)
        display_stats["hebbian_entries"]  = stats.get("hebbian_entries", 0)
        self._stats_w.update_stats(display_stats)

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def _restore_layout(self):
        s = QSettings("AmiOgma", "V3ObserverPanel")
        if s.contains("geometry"):
            self.restoreGeometry(s.value("geometry"))
        if s.contains("windowState"):
            self.restoreState(s.value("windowState"))

    def closeEvent(self, event):
        s = QSettings("AmiOgma", "V3ObserverPanel")
        s.setValue("geometry",    self.saveGeometry())
        s.setValue("windowState", self.saveState())
        event.accept()
