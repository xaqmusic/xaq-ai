"""
BrainV3Adapter — full brain interface shim for the native UI.

Wraps BrainV3 and satisfies every attribute/method that BrainThread
(src/native/workers.py) and AMI_Ogma_Window (src/native/main_window.py)
expect from a brain object.

Designed to be the single point of change for future backends:
    Python v3 EPM  →  BrainV3Adapter(BrainV3(...))
    C++ EPM        →  CppBrainAdapter(CppBridge(...))   # future

get_visualization_data() converts GNG topology to the exact dict format
the physics engine and graph canvas consume:
    {
        'nodes':       [{'id', 'label', 'color', 'count', 'salience',
                          'baked', 'is_super', 'accuracy'}, ...],
        'indices':     np.ndarray (N,) int64  — stable node IDs
        'embeddings':  np.ndarray (N, D)      — prototype vectors
        'edges':       np.ndarray (N, N)      — transition-weighted adjacency
        'pca_coords':  np.ndarray (N, 2)      — PCA 2D layout
        'last_transition': (src_id, tgt_id) or None
    }

process_chunk() returns the stats dict consumed by VisualizerPanel.update_plots()
and AMI_Ogma_Window.on_stats_updated():
    {
        'tle', 'threshold', 'is_novel', 'active_node', 'node_count',
        'is_bootstrapping', 'brain_profile', 'loss', 'nl',
        'quant_error', 'trans_surprise', 'gng_nodes'
    }
"""

import time
import threading
import contextlib
import numpy as np
from typing import Optional, Dict, Any

from .brain_v3 import BrainV3
from .epm import EPMResult


# ---------------------------------------------------------------------------
# Minimal stubs for infrastructure objects the UI reads
# ---------------------------------------------------------------------------

class _EncoderStub:
    """Presents the encoder interface the worker uses to detect sample rate."""
    def __init__(self, sample_rate: int, modality: str):
        self.mel_config = {"sample_rate": sample_rate}
        self.modality   = modality
        # No state_dict — hasattr checks in main_window gate this path


class _SessionLoggerStub:
    """Minimal stub for session_logger accesses in BrainThread."""
    file_path: Optional[str] = None


# ---------------------------------------------------------------------------
# BrainV3Adapter
# ---------------------------------------------------------------------------

class BrainV3Adapter:
    """
    Full brain interface adapter wrapping BrainV3 for the native UI.

    Instantiate once per modality, then pass to AMI_Ogma_Window(brain=adapter).
    """

    def __init__(self, brain: BrainV3):
        self._brain = brain

        # --- Interface attributes expected by BrainThread / AMI_Ogma_Window ---

        # torch.device-like (workers reads this in NativeAudioManager init)
        import torch
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Context manager lock (workers hot-loop uses `with self.brain.processing_lock`)
        self.processing_lock = threading.RLock()

        # Identity
        self.model_type  = brain.modality      # "audio" → STFT path; "retinal" → video path
        self.brain_name  = brain.agent_id
        self.max_history = 5

        # State flags (workers reads these each tick)
        self.is_bootstrapping = False
        self.dream_mode       = False

        # Infrastructure (workers accesses directly)
        self.transmitter    = brain.transmitter
        self.receiver       = brain.receiver
        self.session_logger = _SessionLoggerStub()

        # Encoder stub (workers reads mel_config for sample rate sync)
        self.encoder = _EncoderStub(brain.sample_rate, brain.modality)

        # Neuro-dynamic params (UI sliders read these for display)
        self.adaptation_rate      = brain.epm.alpha
        self.threshold_multiplier = 1.5
        self.distance_metric      = "euclidean"
        self.baked_suppression    = 0.5
        self.node_decay_enabled   = True
        self.threshold            = 0.5   # novelty threshold (homeostatic)

        # Per-node label/color overrides (set via inspector)
        self._node_labels: Dict[int, str]  = {}
        self._node_colors: Dict[int, str]  = {}

        # Viz throttle
        self._last_viz_time    = 0.0
        self._viz_interval     = 1.0 / 30   # 30 Hz max
        self._last_result: Optional[EPMResult] = None
        self._last_audio_chunk: Optional[np.ndarray] = None
        # PCA sign stabilisation — stores the previous eigenvector rows (shape 2×D).
        # Each recalculation, new rows are dot-producted against the previous ones;
        # a negative dot product means the axis flipped, so we negate that row.
        self._prev_pca_Vt: Optional[np.ndarray] = None

        # Stereo mode flag — set by UI channel selector
        self._stereo_mode = False

        # Memory-like proxy (UI reads baking_threshold from memory)
        self.memory = _MemoryProxy(brain.epm.gng)

    # ------------------------------------------------------------------
    # Primary tick method (called by BrainThread every audio/video frame)
    # ------------------------------------------------------------------

    def process_chunk(self, audio_chunk=None, video_chunk=None,
                      external_label=None) -> dict:
        """
        Route input to the correct EPM modality and return a stats dict.

        audio_chunk may be a torch.Tensor (from NativeAudioManager) or np.ndarray.
        video_chunk is an np.ndarray frame.
        """
        import torch

        result: Optional[EPMResult] = None
        chunk_np: Optional[np.ndarray] = None

        if self._brain.modality == "audio":
            if audio_chunk is not None:
                if torch.is_tensor(audio_chunk):
                    chunk_np = audio_chunk.detach().cpu().numpy().squeeze()
                else:
                    chunk_np = np.asarray(audio_chunk, dtype=np.float32).squeeze()

                # Audio from the pipeline is (channels, time) after squeeze on (1, C, T).
                # In stereo mode pass (2, T) through to the encoder's stereo path;
                # in mono mode (default) average across channels first.
                if chunk_np.ndim == 2:
                    if self._stereo_mode and chunk_np.shape[0] == 2:
                        # Keep (2, T) for generic stereo handling — do NOT flatten yet
                        pass
                    else:
                        chunk_np = chunk_np.mean(axis=0)   # (C, T) → (T,)

                chunk_np = chunk_np.astype(np.float32)

                # Skip completely silent frames — zero vector is ambiguous in
                # latent space.  Use 1e-4 (not 1e-6) so low-gain mic streams pass.
                if np.abs(chunk_np).max() < 1e-4:
                    return self._empty_stats()

                result = self._brain.process_audio_chunk(
                    chunk_np, external_label or "")
        else:
            if video_chunk is not None:
                frame_np = np.asarray(video_chunk, dtype=np.float32)
                result = self._brain.process_video_chunk(
                    frame_np, external_label or "")

        if result is None:
            return self._empty_stats()

        self._last_result = result
        self._last_audio_chunk = chunk_np  # None for visual modalities

        # Update homeostatic threshold from EPM's running average
        self.threshold = self._brain.epm._running_avg_tle

        return {
            # Core metrics (VisualizerPanel.update_plots reads these)
            "tle":              result.tle,
            "threshold":        self.threshold * self.threshold_multiplier,
            "loss":             result.quantization_error,   # spatial component
            "nl":               result.moc_mu,              # audio spectral scalar (audio only)

            # Graph tracking (AMI_Ogma_Window.on_stats_updated reads these)
            "active_node":      result.active_node_id,
            "active_label":     self._node_labels.get(result.active_node_id, ""),
            "active_color":     self._node_colors.get(
                                    result.active_node_id,
                                    "#00FF00" if result.is_novel else
                                    ("#FFD700" if result.just_crystallised else "#00FFFF")),
            "node_count":       self._brain.epm.gng.node_count,
            "is_novel":         result.is_novel,
            "is_bootstrapping": False,
            "brain_profile":    {"total": 0},

            # v3-specific extras (shown in status bar / advanced panel)
            "quant_error":      result.quantization_error,
            "trans_surprise":   result.transition_surprise,  # GNG temporal
            "dopamine":         result.dopamine_level,
            "serotonin":        result.serotonin_level,
            "gng_nodes":        self._brain.epm.gng.node_count,
            "gng_baked":        self._brain.epm.gng.baked_count,
            "gng_mean_error":   self._brain.epm.gng.running_mean_error,
            "gng_converged":    (self._brain.epm.gng.running_mean_error
                                 < self._brain.epm.gng.min_insertion_error),

            # Mitosis Gatekeeper signals
            "crystallization_ratio": result.crystallization_ratio,
            "context_novelty":       result.context_novelty,
            "is_mature":             result.is_mature,

            # Visualization helpers (VisualizerPanel.update_plots)
            "encoder_type":     self._brain.modality,
            "encoder_output":   result.encoder_output,       # 128D feature vector
            "encoder_frame":    result.encoder_frame,        # preprocessed frame for 2D viz
            "audio_monitor":    getattr(self, '_last_audio_chunk', None),
        }

    def _empty_stats(self) -> dict:
        return {
            "tle": 0.0, "threshold": self.threshold, "loss": 0.0, "nl": 0.0,
            "active_node": -1, "node_count": self._brain.epm.gng.node_count,
            "is_novel": False, "is_bootstrapping": False,
            "brain_profile": {"total": 0},
            "quant_error": 0.0, "trans_surprise": 0.0,
            "gng_nodes": self._brain.epm.gng.node_count,
            "gng_baked": self._brain.epm.gng.baked_count,
            "crystallization_ratio": self._brain.epm.gng.crystallization_ratio,
            "context_novelty": 0.0, "is_mature": False,
        }

    # ------------------------------------------------------------------
    # Spatial probe (SpatialProbeWidget calls this each frame)
    # ------------------------------------------------------------------

    def get_spatial_probe(self) -> Optional[dict]:
        """Pass-through to the encoder's saliency probe.  Returns None if the
        active modality is not saliency or the encoder has no probe data yet."""
        enc = getattr(self._brain.epm, "encoder", None)
        if enc is None or not hasattr(enc, "get_spatial_probe"):
            return None
        return enc.get_spatial_probe()

    def rebuild_encoder(self, **kwargs) -> bool:
        """Rebuild the current encoder with overridden constructor kwargs
        (e.g. inject_centroid, centroid_gain).  Resets GNG since the latent
        space shifts.  Returns True on success."""
        from .encoders import make_encoder
        try:
            modality = self._brain.modality
            new_enc = make_encoder(
                modality,
                projection_dim=self._brain.epm.projection_dim,
                sample_rate=self._brain.sample_rate,
                **kwargs,
            )
            self._brain.epm.encoder = new_enc
            self._brain.epm.gng.reset()
            return True
        except Exception as e:
            print(f"[BrainV3Adapter] rebuild_encoder failed: {e}")
            return False

    # ------------------------------------------------------------------
    # Visualization data (BrainThread calls this each frame for the graph)
    # ------------------------------------------------------------------

    def get_visualization_data(self) -> Optional[dict]:
        """
        Convert GNG topology → the dict format the physics engine and
        graph canvas consume. Rate-limited to 30 Hz.
        """
        now = time.time()
        if now - self._last_viz_time < self._viz_interval:
            return None
        self._last_viz_time = now

        topology = self._brain.epm.gng.get_topology()
        nodes_list = topology["nodes"]
        if not nodes_list:
            return None

        N = len(nodes_list)
        node_ids   = np.array([n["id"]        for n in nodes_list], dtype=np.int64)
        prototypes = np.array([n["prototype"]  for n in nodes_list], dtype=np.float32)
        visits     = np.array([n["visits"]     for n in nodes_list], dtype=np.int32)
        errors     = np.array([n["error"]       for n in nodes_list], dtype=np.float32)
        ema_errs   = np.array([n["ema_error"]   for n in nodes_list], dtype=np.float32)
        cryst      = np.array([n["crystallised"] for n in nodes_list], dtype=bool)

        # --- PCA 2D layout ---
        pca_coords = np.zeros((N, 2), dtype=np.float32)
        if N >= 2:
            try:
                centered = prototypes - prototypes.mean(axis=0)
                _, _, Vt = np.linalg.svd(centered, full_matrices=False)
                Vt2 = Vt[:2]  # shape (2, D) — the two principal axes

                # Sign stabilisation: PCA eigenvectors are only defined up to sign.
                # Each recalculation numpy may flip one or both axes, causing the
                # entire graph to mirror instantly.  Fix: if the dot product between
                # a new axis row and the previous one is negative, negate that row.
                if (self._prev_pca_Vt is not None
                        and self._prev_pca_Vt.shape == Vt2.shape):
                    for i in range(2):
                        if np.dot(Vt2[i], self._prev_pca_Vt[i]) < 0:
                            Vt2[i] *= -1
                self._prev_pca_Vt = Vt2.copy()

                pca_coords = (centered @ Vt2.T).astype(np.float32)
                mx = np.abs(pca_coords).max()
                if mx > 1e-6:
                    # Scale to ±20 — matches the physics engine's calibration range
                    # (v2 PCA coords ran to ~±30; ±1 made all nodes collapse into a
                    # blob because collision radius 100 >> entire coordinate space 2).
                    pca_coords = pca_coords / mx * 20.0
            except Exception:
                pass

        # --- Edge adjacency (N×N) from GNG edges + transition weights ---
        id_to_idx = {int(nid): i for i, nid in enumerate(node_ids)}
        edges = np.zeros((N, N), dtype=np.float32)

        # GNG structural edges (unweighted)
        for edge in topology["edges"]:
            ai = id_to_idx.get(edge["a"])
            bi = id_to_idx.get(edge["b"])
            if ai is not None and bi is not None:
                edges[ai, bi] = max(edges[ai, bi], 0.3)
                edges[bi, ai] = max(edges[bi, ai], 0.3)

        # Transition-frequency edges (directional, weighted)
        epm = self._brain.epm
        for prev_id, row in epm._transition_counts.items():
            pi = id_to_idx.get(prev_id)
            if pi is None:
                continue
            total = max(epm._node_totals.get(prev_id, 1), 1)
            for curr_id, count in row.items():
                ci = id_to_idx.get(curr_id)
                if ci is not None:
                    edges[pi, ci] = max(edges[pi, ci], count / total)

        # --- Winner node ---
        hist = self._brain.epm.gng.get_history(1)
        winner_id = hist[0] if hist else -1

        # --- Node metadata ---
        nodes_meta = []
        for i, n in enumerate(nodes_list):
            nid = n["id"]
            is_winner = (nid == winner_id)
            default_color = (
                "#00FF00" if is_winner else
                "#FFD700" if n["crystallised"] else
                "#00CFFF"
            )
            # Salience: normalise visit count (higher visits = higher salience)
            max_visits = max(int(visits.max()), 1)
            salience = float(visits[i]) / max_visits

            is_baked = bool(cryst[i])
            # Color priority: user override → winner → baked → unbaked
            default_color = (
                "#00FF00" if is_winner else
                "#FFD700" if is_baked else
                "#00CFFF"
            )
            nodes_meta.append({
                "id":        nid,
                "label":     self._node_labels.get(nid, ""),
                "color":     self._node_colors.get(nid, default_color),
                "count":     int(visits[i]),
                "salience":  salience,
                "baked":     is_baked,
                "is_super":  False,
                "accuracy":  -1.0,
                # v3 extras shown in the inspector
                "error":     float(errors[i]),
                "ema_error": float(ema_errs[i]),
            })

        # --- Last transition (graph_canvas expects {'source': id, 'target': id}) ---
        full_hist = self._brain.epm.gng.get_history(2)
        last_transition = ({"source": full_hist[-2], "target": full_hist[-1]}
                           if len(full_hist) >= 2 else None)

        return {
            "nodes":           nodes_meta,
            "indices":         node_ids,
            "embeddings":      prototypes,
            "edges":           edges,
            "pca_coords":      pca_coords,
            "last_transition": last_transition,
        }

    # ------------------------------------------------------------------
    # Neuro-dynamic controls (UI sliders / settings panel)
    # ------------------------------------------------------------------

    def set_encoder(self, modality: str):
        """
        Switch the EPM to a different sensory modality.

        Swaps the encoder in-place and resets the GNG (new modality → new latent
        space; old node prototypes are meaningless).  Valid v3 modalities:
            audio: 'audio'
            video: 'retinal', 'color', 'optical_flow', 'saliency', 'dorsal', 'ventral'

        Called by main_window.on_encoder_changed() when the user selects a new
        encoder from the combo box.  After returning, workers.py reads
        self.model_type to decide whether to pull frames from the VideoManager
        or samples from the AudioManager.
        """
        _V3_MODALITIES = {
            "audio",
            "retinal", "color", "optical_flow", "saliency", "dorsal", "ventral",
        }
        if modality not in _V3_MODALITIES:
            print(f"[BrainV3Adapter] Modality '{modality}' not supported in v3. "
                  f"Valid: {sorted(_V3_MODALITIES)}")
            return

        from .encoders import make_encoder
        old_modality = self._brain.modality

        # 1. Snapshot user-tuned GNG params before reset wipes them.
        #    EPM.reset() creates a fresh GNG with constructor defaults, so any
        #    slider values set since startup would be lost without this save.
        gng = self._brain.epm.gng
        saved = {
            "min_insertion_error": gng.min_insertion_error,
            "gng_lambda_new":      gng.lambda_new,
            "gng_max_age":         gng.max_age,
            "baking_threshold":    gng.baking_threshold,
            "node_decay_enabled":  gng.stale_prune_enabled,
            "node_decay_seconds":  gng.stale_window_factor / 30.0,  # steps→seconds at ~30fps
            "adaptation_rate":     self._brain.epm.alpha,
            "threshold_multiplier": self.threshold_multiplier,
        }

        # 2. Build replacement encoder
        new_encoder = make_encoder(
            modality,
            projection_dim=self._brain.epm.projection_dim,
            sample_rate=self._brain.sample_rate,
        )

        # 3. Swap encoder and update modality flags everywhere workers reads them
        self._brain.epm.encoder   = new_encoder
        self._brain.epm.modality  = modality
        self._brain.modality      = modality
        self.model_type           = modality          # workers.py routing key
        self.encoder.modality     = modality          # _EncoderStub used by worker rate-sync

        # 4. Full cognitive reset — old node prototypes are in the old latent space
        self._brain.reset()
        self._node_labels.clear()
        self._node_colors.clear()
        self._last_result   = None
        self._prev_pca_Vt   = None

        # 5. Restore user-tuned params to the fresh GNG
        self.update_neuro_dynamics(saved)

        print(f"[BrainV3Adapter] Modality switched: {old_modality} → {modality}")

    def update_neuro_dynamics(self, params: dict):
        if "adaptation_rate" in params:
            self.adaptation_rate = float(params["adaptation_rate"])
            self._brain.epm.alpha = self.adaptation_rate
        if "threshold_multiplier" in params:
            self.threshold_multiplier = float(params["threshold_multiplier"])
        if "baking_threshold" in params:
            self._brain.epm.gng.baking_threshold = int(params["baking_threshold"])
            self.memory.baking_threshold = self._brain.epm.gng.baking_threshold
        if "baked_suppression" in params:
            # baked_suppression is not used in v3 — baked prototypes are fully
            # frozen in gng.step() regardless of this setting.  Store locally
            # only so UI reads back the right value; do not route to epm.beta
            # (which is the TLE transition weight, an unrelated parameter).
            self.baked_suppression = float(params["baked_suppression"])
        if "min_insertion_error" in params:
            self._brain.epm.gng.min_insertion_error = float(params["min_insertion_error"])
        if "gng_lambda_new" in params:
            self._brain.epm.gng.lambda_new = max(1, int(params["gng_lambda_new"]))
        if "gng_max_age" in params:
            self._brain.epm.gng.max_age = max(5, int(params["gng_max_age"]))
        if "node_decay_enabled" in params:
            # "Active Decay" checkbox — unchecking disables stale-prune entirely
            # so nodes survive indefinitely (useful between game episodes).
            self._brain.epm.gng.stale_prune_enabled = bool(params["node_decay_enabled"])
            self.node_decay_enabled = bool(params["node_decay_enabled"])
        if "node_decay_seconds" in params:
            # stale_window_factor is now absolute steps; convert seconds→steps at ~30fps
            steps = max(300.0, float(params["node_decay_seconds"]) * 30.0)
            self._brain.epm.gng.stale_window_factor = steps

    def update_senses(self, params: dict):
        """Handle UI sense updates — only sample_rate is meaningful for v3."""
        if "sample_rate" in params:
            self.encoder.mel_config["sample_rate"] = int(params["sample_rate"])

    def update_prediction_horizon(self, n):  pass
    def update_predictor_lr(self, lr):       pass
    def set_predictor(self, ptype):          pass
    def set_frozen(self, v):                 pass
    def set_stereo_mode(self, v):
        self._stereo_mode = bool(v)
        mode = "stereo" if self._stereo_mode else "mono"
        print(f"[BrainV3Adapter] Audio mode → {mode}")
    def set_dream_mode(self, v):             pass
    def start_bootstrap(self, ticks=0):      pass
    def stop_bootstrap(self):                pass

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save_state(self, folder_path: str):
        import json, os
        os.makedirs(folder_path, exist_ok=True)
        state = {
            "version":   "v3",
            "modality":  self._brain.modality,
            "agent_id":  self._brain.agent_id,
            "gng":       self._brain.epm.gng.to_dict(),
            "params": {
                "alpha":               self._brain.epm.alpha,
                "beta":                self._brain.epm.beta,
                "min_insertion_error": self._brain.epm.gng.min_insertion_error,
                "baking_threshold":    self._brain.epm.gng.baking_threshold,
            },
            # Node annotations survive reload
            "node_labels": {str(k): v for k, v in self._node_labels.items()},
            "node_colors": {str(k): v for k, v in self._node_colors.items()},
        }
        with open(os.path.join(folder_path, "brain_state_v3.json"), "w") as f:
            json.dump(state, f, indent=2)
        print(f"[BrainV3Adapter] Saved state → {folder_path}")

    def load_state(self, folder_path: str) -> bool:
        import json, os
        from .gng import GNG
        path = os.path.join(folder_path, "brain_state_v3.json")
        if not os.path.exists(path):
            print(f"[BrainV3Adapter] No v3 state at {path}")
            return False
        with open(path) as f:
            state = json.load(f)
        self._brain.epm.gng = GNG.from_dict(
            state["gng"], dim=self._brain.epm.projection_dim)
        p = state.get("params", {})
        if "alpha"               in p: self._brain.epm.alpha                          = p["alpha"]
        if "beta"                in p: self._brain.epm.beta                           = p["beta"]
        if "min_insertion_error" in p: self._brain.epm.gng.min_insertion_error        = p["min_insertion_error"]
        if "baking_threshold"    in p: self._brain.epm.gng.baking_threshold           = p["baking_threshold"]
        # Restore annotations (keys stored as str in JSON)
        self._node_labels = {int(k): v for k, v in state.get("node_labels", {}).items()}
        self._node_colors = {int(k): v for k, v in state.get("node_colors", {}).items()}
        # Update memory proxy
        self.memory = _MemoryProxy(self._brain.epm.gng)
        print(f"[BrainV3Adapter] Loaded state ← {folder_path}  ({self._brain.epm.gng.node_count} nodes)")
        return True

    def save_graph_binary(self, path: str):
        """Save GNG prototypes in the C++ binary format (header + float32 rows)."""
        topo = self._brain.epm.gng.get_topology()
        nodes = topo["nodes"]
        if not nodes:
            return
        protos = np.array([n["prototype"] for n in nodes], dtype=np.float32)
        header = np.array([len(nodes), protos.shape[1]], dtype=np.int32)
        with open(path, "wb") as f:
            f.write(header.tobytes())
            f.write(protos.tobytes())
        print(f"[BrainV3Adapter] Saved {len(nodes)} GNG prototypes to {path}")

    def save_encoder_weights(self, path: str):
        print("[BrainV3Adapter] Frozen encoder — no weights to save.")

    def load_encoder_weights(self, path: str):
        print("[BrainV3Adapter] Frozen encoder — no weights to load.")

    def start_recording(self, path: str): pass
    def stop_recording(self):             pass

    def get_node_memories(self, node_id: int) -> list:
        return []

    # ------------------------------------------------------------------
    # Graph editing (inspector operations)
    # ------------------------------------------------------------------

    def update_node_attributes(self, node_id, attrs: dict):
        nid = int(node_id)
        if "label" in attrs:
            lbl = attrs["label"]
            if lbl:
                self._node_labels[nid] = lbl
            else:
                self._node_labels.pop(nid, None)
        if "color" in attrs:
            self._node_colors[nid] = attrs["color"]

    def delete_nodes(self, node_ids: list):
        """Delete nodes from GNG topology and clean up label/color overrides."""
        gng = self._brain.epm.gng
        for raw_id in node_ids:
            nid = int(raw_id)
            self._node_labels.pop(nid, None)
            self._node_colors.pop(nid, None)
            pos = gng._id_to_pos.get(nid)
            if pos is not None and gng._alive[pos]:
                for nb_pos in list(gng._adj.get(pos, set())):
                    ek = frozenset({pos, nb_pos})
                    gng._remove_edge(ek)
                gng._kill_node(pos)

    def prune_unbaked_nodes(self):
        """Remove all GNG nodes below baking_threshold (keeps at least 2)."""
        gng = self._brain.epm.gng
        n = gng.prune_unbaked()
        print(f"[BrainV3Adapter] Pruned {n} unbaked nodes. Remaining: {gng.node_count}")

    def detect_patterns(self):          pass
    def detect_temporal_clusters(self): pass
    def group_by_labels(self):          pass
    def dissolve_all_supernodes(self):  pass

    def reset(self):
        self._brain.reset()
        self._node_labels.clear()
        self._node_colors.clear()
        self._last_result = None
        self._prev_pca_Vt = None  # clear sign reference on full reset


# ---------------------------------------------------------------------------
# Proxy that satisfies `self.brain.memory.baking_threshold` in main_window
# ---------------------------------------------------------------------------

class _MemoryProxy:
    def __init__(self, gng):
        self._gng = gng

    @property
    def baking_threshold(self) -> int:
        return self._gng.baking_threshold

    @baking_threshold.setter
    def baking_threshold(self, v: int):
        self._gng.baking_threshold = int(v)
