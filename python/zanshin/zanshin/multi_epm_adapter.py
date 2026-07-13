"""
multi_epm_adapter.py — MultiEPMAdapter for the Phase 3 ZeroMQ message bus.

Manages N CppEPMAdapter instances (one per modality), routes audio/video to
the correct EPM in parallel threads, and fuses their RealityTokens via
LateralVoterV3.  The active (highest-trust) EPM's visualization data is
exposed to the UI.

Port allocation:
    EPM 0 (modalities[0]): control=base_port,   pub=base_port+1
    EPM 1 (modalities[1]): control=base_port+2, pub=base_port+3
    ...

The shared ZMQ SUB connects to ALL PUB ports.  A background thread drains
the bus and updates _latest_tokens, which process_chunk() reads after
submitting frames to all EPMs.

Usage:
    adapter = MultiEPMAdapter(
        modalities=["retinal", "audio"],
        projection_dim=128,
        base_port=7200,
    )
    stats = adapter.process_chunk(audio_chunk, video_chunk=frame)
    viz   = adapter.get_visualization_data()
"""

import os
import threading
import time
from multiprocessing.shared_memory import SharedMemory
from typing import Dict, List, Optional

import numpy as np

from zanshin.cpp_adapter       import CppEPMAdapter, _TransmitterStub, _ReceiverStub, _SessionLoggerStub
from zanshin.consistency_gate  import ConsistencyGate
from zanshin.lateral_voter_v3  import LateralVoterV3
from zanshin.zmq_bus           import RealityTokenSubscriber

_VIDEO_MODALITIES = {"retinal", "color", "optical_flow", "saliency", "dorsal", "ventral",
                     "visual_consensus"}
_AUDIO_MODALITIES = {"audio"}
_STATE_MODALITIES = {"proprioceptive"}


class MultiEPMAdapter:
    """
    Brain adapter that runs N C++ EPM subprocesses in parallel and fuses
    their output via a ZeroMQ pub/sub bus + LateralVoterV3.
    """

    def __init__(self,
                 modalities:     List[str],
                 projection_dim: int  = 128,
                 base_port:      int  = 7200,
                 binary_path:    Optional[str] = None,
                 audio_mode:  str  = "stft",
                 encoder_res:    int  = 0,
                 inject_centroid: bool = False,
                 centroid_gain:   float = 22.6,
                 proprio_state_dims: int = 6):

        if len(modalities) < 1:
            raise ValueError("MultiEPMAdapter requires at least one modality")

        self._modalities     = list(modalities)
        self._projection_dim = projection_dim
        self._base_port      = base_port

        # Action modality: which EPM drives the action decoder.
        # Position-encoding modalities (ventral, retinal) produce clean
        # position→velocity mappings for the Hebbian table.  Motion-encoding
        # modalities (dorsal, optical_flow) contribute through the voter
        # and consistency gate but should NOT drive action keying — their
        # transitions encode motion-pattern changes, not ball trajectories.
        _POSITION_MODS = ("ventral", "retinal", "color", "saliency")
        self.action_modality = next(
            (m for m in modalities if m in _POSITION_MODS),
            modalities[0]  # fallback: first modality
        )

        # UI-facing identity (mirrors CppEPMAdapter)
        self.model_type           = ",".join(modalities)
        self.brain_name           = f"multi-epm-{'_'.join(modalities)}"
        self.max_history          = 5
        self.is_bootstrapping     = False
        self.dream_mode           = False
        self.processing_lock      = threading.RLock()
        self.transmitter          = _TransmitterStub()
        self.receiver             = _ReceiverStub()
        self.session_logger       = _SessionLoggerStub()
        self.threshold            = 0.5
        self.threshold_multiplier = 1.5
        self.distance_metric      = "euclidean"
        self.baked_suppression    = 0.5
        self.node_decay_enabled   = True

        import torch
        self.device = torch.device("cpu")

        # Per-EPM adapters
        self._adapters: Dict[str, CppEPMAdapter] = {}
        for i, mod in enumerate(modalities):
            port = base_port + i * 2
            self._adapters[mod] = CppEPMAdapter(
                modality=mod,
                projection_dim=projection_dim,
                port=port,
                binary_path=binary_path,
                _start_zmq_sub=False,   # MultiEPMAdapter owns the shared SUB
                audio_mode=audio_mode,
                encoder_res=encoder_res,
                inject_centroid=inject_centroid,
                centroid_gain=centroid_gain,
                proprio_state_dims=proprio_state_dims,
            )

        # Shared ZMQ SUB — subscribes to all EPM PUB sockets
        self._zmq_sub = RealityTokenSubscriber()
        for i, mod in enumerate(modalities):
            pub_port = base_port + i * 2 + 1
            self._zmq_sub.connect(f"tcp://127.0.0.1:{pub_port}", topic=mod)
        self._zmq_sub.start()

        # Tiered voting: when 2+ visual EPMs, insert a Visual Voter that
        # pre-fuses them into a single visual_consensus token before the
        # master voter (B1: Sub-group voters).
        self._visual_mods    = [m for m in modalities if m in _VIDEO_MODALITIES]
        self._nonvisual_mods = [m for m in modalities if m not in _VIDEO_MODALITIES]
        self._tiered = len(self._visual_mods) >= 2

        if self._tiered:
            self._visual_voter = LateralVoterV3(self._visual_mods, projection_dim)
            master_mods = ["visual_consensus"] + self._nonvisual_mods
            self._voter = LateralVoterV3(master_mods, projection_dim)
            print(f"[MultiEPMAdapter] Tiered voting: visual={self._visual_mods} "
                  f"→ visual_consensus → master={master_mods}")
        else:
            self._visual_voter = None
            self._voter = LateralVoterV3(modalities, projection_dim)

        # B2: Dorsal/Ventral consistency gate — active when both are present
        self._has_consistency_gate = ("dorsal" in modalities and
                                      "ventral" in modalities)
        self._consistency_gate = ConsistencyGate() if self._has_consistency_gate else None
        if self._has_consistency_gate:
            print("[MultiEPMAdapter] Consistency gate active (dorsal ↔ ventral)")
        print(f"[MultiEPMAdapter] Action decoder pinned to: {self.action_modality}")

        # Latest fused stats (updated each process_chunk)
        self._last_stats:    dict = {}
        self._last_per_mod_stats: Dict[str, dict] = {}
        self._active_mod:    str  = modalities[0]
        self._winner_history: list = []

        # Proxy for brain.memory used by UI
        from zanshin.cpp_adapter import _MemoryProxy
        self.memory = _MemoryProxy()

        # Shared memory for video frames — one region, all video EPMs read it
        # Allocated lazily on first video frame to avoid sizing issues at init
        self._video_shm: Optional[SharedMemory] = None
        self._video_shm_arr: Optional[np.ndarray] = None
        self._video_shm_dims: Optional[tuple] = None   # (h, w, c)
        self._video_shm_name = f"ami-ogma-vid-{os.getpid()}"

    # ------------------------------------------------------------------
    # Shared memory helpers
    # ------------------------------------------------------------------

    def _ensure_video_shm(self, frame: np.ndarray) -> None:
        """Allocate (or reallocate) the shared video frame buffer."""
        h, w = frame.shape[:2]
        c = frame.shape[2] if frame.ndim == 3 else 1
        dims = (h, w, c)
        if self._video_shm is not None and self._video_shm_dims == dims:
            return  # already correct size

        # Release old region if dimensions changed
        if self._video_shm is not None:
            try:
                self._video_shm.close()
                self._video_shm.unlink()
            except Exception:
                pass
            self._video_shm = None

        sz = h * w * c
        # Unlink any stale shm from a previous crash before creating
        try:
            SharedMemory(name=self._video_shm_name, create=False, size=sz).unlink()
        except Exception:
            pass
        self._video_shm = SharedMemory(name=self._video_shm_name, create=True, size=sz)
        self._video_shm_arr = np.ndarray((h, w, c), dtype=np.uint8,
                                         buffer=self._video_shm.buf)
        self._video_shm_dims = dims
        print(f"[MultiEPMAdapter] shared video buffer: {w}×{h}×{c}  "
              f"shm={self._video_shm_name}")

    # ------------------------------------------------------------------
    # Frame processing — parallel dispatch
    # ------------------------------------------------------------------

    def process_chunk(self,
                      audio_chunk=None,
                      video_chunk=None,
                      external_label=None,
                      body_state=None) -> dict:
        """
        Route audio/video/state to the correct EPMs simultaneously via threads,
        then fuse their ZMQ tokens via the Lateral Voter.

        Video path: frame is written to a single shared-memory region once;
        all video EPMs read from the same buffer (zero duplication).
        Audio path: unchanged (base64 over TCP — chunks are small).
        State path: body state vector sent to proprioceptive EPM.
        """
        # Write video frame to shared memory before dispatching threads
        video_shm_info: Optional[dict] = None
        if video_chunk is not None:
            frame = np.asarray(video_chunk, dtype=np.uint8)
            if frame.ndim == 2:
                frame = frame[:, :, np.newaxis]
            self._ensure_video_shm(frame)
            np.copyto(self._video_shm_arr, frame)
            h, w, c = self._video_shm_dims
            video_shm_info = {
                "shm_key":  self._video_shm_name,
                "height":   h,
                "width":    w,
                "channels": c,
            }

        results: Dict[str, dict] = {}
        lock = threading.Lock()
        threads = []
        mod_times: Dict[str, float] = {}

        def run(mod: str, adapter: CppEPMAdapter):
            t0 = time.perf_counter()
            try:
                if mod in _VIDEO_MODALITIES and video_shm_info is not None:
                    s = adapter._process_video_shm(video_shm_info)
                elif mod in _AUDIO_MODALITIES and audio_chunk is not None:
                    s = adapter.process_chunk(audio_chunk)
                elif mod in _STATE_MODALITIES and body_state is not None:
                    s = adapter.process_state(body_state)
                else:
                    return  # no data for this EPM this tick
                with lock:
                    results[mod] = s
                    mod_times[mod] = time.perf_counter() - t0
            except Exception as e:
                print(f"[MultiEPMAdapter] {mod} error: {e}")

        t_dispatch = time.perf_counter()
        for mod, adapter in self._adapters.items():
            t = threading.Thread(target=run, args=(mod, adapter), daemon=True)
            threads.append(t)
            t.start()

        for t in threads:
            t.join(timeout=0.5)
        t_joined = time.perf_counter()

        # Profiling: accumulate per-modality times; emit once on first 200-tick mark
        # Set profile=True to enable periodic output (every 200 ticks).
        if not hasattr(self, '_profile_tick'):
            self._profile_tick = 0
            self._profile_accum: Dict[str, float] = {}
            self._profile_wall_accum = 0.0
            self._profile_reported = False
        self._profile_tick += 1
        self._profile_wall_accum += (t_joined - t_dispatch)
        for mod, dt in mod_times.items():
            self._profile_accum[mod] = self._profile_accum.get(mod, 0.0) + dt
        _period = 200
        if self._profile_tick % _period == 0 and not self._profile_reported:
            n = _period
            wall_ms = self._profile_wall_accum / n * 1000
            parts = ", ".join(
                f"{m}={self._profile_accum.get(m, 0)/n*1000:.1f}ms"
                for m in self._modalities
            )
            print(f"[profile] parallel_wall={wall_ms:.1f}ms  per-EPM: {parts}")
            self._profile_wall_accum = 0.0
            self._profile_accum = {}
            self._profile_reported = True

        # Also pull any tokens from the ZMQ bus (may include frames processed
        # slightly ahead of our TCP round-trip — catches async bursts)
        zmq_tokens = self._zmq_sub.get_all_latest()
        for mod, tok in zmq_tokens.items():
            if mod not in results:
                results[mod] = self._adapters[mod]._resp_to_stats(tok, None) \
                    if mod in self._adapters else tok

        if not results:
            return self._last_stats or {}

        # ---- Tiered fusion (B1) ----
        if self._tiered:
            # Step 1: fuse visual EPMs → visual_consensus token
            visual_tokens = {m: results[m] for m in self._visual_mods if m in results}
            nonvisual_tokens = {m: results[m] for m in self._nonvisual_mods if m in results}

            if visual_tokens:
                visual_consensus = self._visual_voter.fuse(visual_tokens)
            else:
                visual_consensus = self._visual_voter._empty_stats()

            # B2: Dorsal/Ventral consistency gate — inject TLE penalty on
            # perceptual inconsistency (object permanence violations)
            consistency_penalty = 0.0
            if (self._consistency_gate is not None
                    and "dorsal" in results and "ventral" in results):
                consistency_penalty = self._consistency_gate.update(
                    results["dorsal"], results["ventral"])
                visual_consensus["tle"] = visual_consensus.get("tle", 0.0) + consistency_penalty

            # Step 2: feed visual_consensus + non-visual tokens to master voter
            master_tokens = {"visual_consensus": visual_consensus}
            master_tokens.update(nonvisual_tokens)
            fused = self._voter.fuse(master_tokens)

            # Inject visual sub-voter telemetry
            fused["visual_consensus_detail"] = {
                "trust_weights": visual_consensus.get("trust_weights", {}),
                "trust_ema":     visual_consensus.get("trust_ema", {}),
                "tle":           visual_consensus.get("tle", 0.0),
                "is_novel":      visual_consensus.get("is_novel", False),
            }
            if self._consistency_gate is not None:
                fused["consistency_gate"] = self._consistency_gate.to_dict()

            # Resolve active_modality: if master picked "visual_consensus",
            # drill down to the actual visual EPM that won
            if fused.get("active_modality") == "visual_consensus":
                real_visual_mod = visual_consensus.get("active_modality",
                                                       self._visual_mods[0])
                fused["active_modality"] = real_visual_mod

            # Dispatch boosts from visual sub-voter → individual visual EPMs
            for mod, node_id, amount in self._visual_voter.consume_boosts():
                adapter = self._adapters.get(mod)
                if adapter is not None and hasattr(adapter, 'boost_node'):
                    try:
                        adapter.boost_node(node_id, amount)
                    except Exception:
                        pass
        else:
            fused = self._voter.fuse(results)

        self._last_per_mod_stats = dict(results)
        self._active_mod = fused.get("active_modality", self._modalities[0])

        # Overwrite per-EPM node counts with the true per-modality values
        # (tiered fusion can mask individual visual EPMs behind "visual_consensus").
        fused["gng_nodes_per_epm"] = {
            m: int(results[m].get("gng_nodes", 0)) for m in results
        }

        # Expose per-modality results directly in the fused dict so downstream
        # consumers (ActionDecoder) can read e.g. stats["results"]["proprioceptive"]
        # without a separate get_per_modality_stats() call.
        fused["results"] = dict(results)

        # Inject action-pinned modality and node into fused stats so the
        # action decoder always keys off the position-encoding EPM.
        action_stats = results.get(self.action_modality, {})
        fused["action_modality"] = self.action_modality
        fused["action_node"]     = action_stats.get("active_node", -1)

        # Expose the saliency trajectory (Reality Token semantic history)
        # for trajectory-keyed active inference.
        action_adapter = self._adapters.get(self.action_modality)
        if action_adapter is not None:
            fused["action_trajectory"] = list(action_adapter._winner_history)
        else:
            fused["action_trajectory"] = []

        self._last_stats = fused

        # Dispatch consensus-driven bake boosts from master voter to EPMs
        for mod, node_id, amount in self._voter.consume_boosts():
            if mod == "visual_consensus":
                continue  # visual sub-voter handles individual visual EPM boosts
            adapter = self._adapters.get(mod)
            if adapter is not None and hasattr(adapter, 'boost_node'):
                try:
                    adapter.boost_node(node_id, amount)
                except Exception:
                    pass  # best-effort — don't let boost failures break the loop

        # Update winner history from active EPM
        winner = fused.get("active_node", -1)
        if winner >= 0:
            self._winner_history.append(winner)
            if len(self._winner_history) > 8:
                self._winner_history.pop(0)

        return fused

    def get_per_modality_stats(self) -> Dict[str, dict]:
        """Return the latest per-EPM stats dict (modality → stats)."""
        return dict(self._last_per_mod_stats)

    # ------------------------------------------------------------------
    # Spatial probe — C++ EPMs do not expose encoder internals
    # ------------------------------------------------------------------

    def get_spatial_probe(self) -> Optional[dict]:
        return None

    def rebuild_encoder(self, **kwargs) -> bool:
        return False

    # ------------------------------------------------------------------
    # Visualization — delegate to active EPM
    # ------------------------------------------------------------------

    def get_visualization_data(self) -> Optional[dict]:
        adapter = self._adapters.get(self._active_mod)
        if adapter is None:
            return None
        viz = adapter.get_visualization_data()
        if viz is None:
            return None
        # Inject last_transition from shared history
        h = self._winner_history
        if len(h) >= 2 and h[-1] != h[-2]:
            viz["last_transition"] = {"source": h[-2], "target": h[-1]}
        return viz

    def get_prototype(self, node_id: int, modality: Optional[str] = None) -> Optional[np.ndarray]:
        """Fetch prototype from a specific modality, or the current action modality."""
        target_mod = modality or self.action_modality
        adapter = self._adapters.get(target_mod)
        if adapter and hasattr(adapter, 'get_prototype'):
            return adapter.get_prototype(node_id)
        return None

    # ------------------------------------------------------------------
    # Parameter control — broadcast to all EPMs
    # ------------------------------------------------------------------

    def update_neuro_dynamics(self, params: dict) -> None:
        # Per-modality overrides: if a key in params matches a modality name
        # AND its value is a dict, route that sub-dict to that adapter only
        # (applied AFTER the global pass so per-mod values win).
        mod_overrides = {k: v for k, v in params.items()
                         if k in self._adapters and isinstance(v, dict)}
        global_params = {k: v for k, v in params.items() if k not in mod_overrides}

        for adapter in self._adapters.values():
            adapter.update_neuro_dynamics(global_params)
        for mod, sub in mod_overrides.items():
            self._adapters[mod].update_neuro_dynamics(sub)

        if "baking_threshold" in global_params:
            self.memory.baking_threshold = int(global_params["baking_threshold"])

    def set_serotonin(self, serotonin: float) -> None:
        """Forward serotonin to all voters for Hebbian gating."""
        if self._visual_voter is not None:
            self._visual_voter.set_serotonin(serotonin)
        self._voter.set_serotonin(serotonin)

    def update_senses(self, params: dict) -> None:
        for adapter in self._adapters.values():
            adapter.update_senses(params)

    def update_prediction_horizon(self, *a, **kw): pass
    def set_predictor(self, *a, **kw): pass
    def set_dream_mode(self, *a, **kw): pass
    def set_agent_id(self, *a, **kw): pass
    def set_brain_name(self, *a, **kw): pass

    # ------------------------------------------------------------------
    # Persistence — delegate to active EPM
    # ------------------------------------------------------------------

    def save_state(self, folder_path: str) -> None:
        for adapter in self._adapters.values():
            adapter.save_state(folder_path)

    def load_state(self, folder_path: str) -> None:
        for adapter in self._adapters.values():
            adapter.load_state(folder_path)

    def save_encoder_weights(self, file_path: str) -> None:
        adapter = self._adapters.get(self._active_mod)
        if adapter:
            adapter.save_encoder_weights(file_path)

    # ------------------------------------------------------------------
    # Encoder switching — rebuild the adapter set
    # ------------------------------------------------------------------

    def set_encoder(self, modality: str) -> None:
        """Replace the active visual EPM with a different modality."""
        old_visual = [m for m in self._modalities if m in _VIDEO_MODALITIES]
        if not old_visual:
            return
        old_mod = old_visual[0]
        if old_mod == modality:
            return

        idx = self._modalities.index(old_mod)
        port = self._base_port + idx * 2

        # Stop old adapter
        old_adapter = self._adapters.pop(old_mod)
        old_adapter._stop_subprocess()

        # Rebuild with new modality
        new_adapter = CppEPMAdapter(
            modality=modality,
            projection_dim=self._projection_dim,
            port=port,
            _start_zmq_sub=False,
        )
        self._adapters[modality] = new_adapter
        self._modalities[idx] = modality
        self.model_type = ",".join(self._modalities)

        # Reconnect ZMQ SUB to new PUB port
        pub_port = port + 1
        self._zmq_sub._sock.connect(f"tcp://127.0.0.1:{pub_port}")
        self._zmq_sub._sock.setsockopt_string(__import__("zmq").SUBSCRIBE, modality)

        self._active_mod = modality

        # Update visual voter modality list if tiered
        if self._tiered and old_mod in self._visual_mods:
            vi = self._visual_mods.index(old_mod)
            self._visual_mods[vi] = modality
            self._visual_voter = LateralVoterV3(self._visual_mods,
                                                 self._projection_dim)

        print(f"[MultiEPMAdapter] Encoder switched {old_mod} → {modality}")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _stop_subprocess(self) -> None:
        self._zmq_sub.stop()
        for adapter in self._adapters.values():
            adapter._stop_subprocess()
        # Release shared video frame buffer
        if self._video_shm is not None:
            try:
                self._video_shm.close()
                self._video_shm.unlink()
            except Exception:
                pass
            self._video_shm = None

    def __del__(self):
        try:
            self._stop_subprocess()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Status properties (UI compat)
    # ------------------------------------------------------------------

    @property
    def encoder(self):
        return self._adapters.get(self._active_mod, next(iter(self._adapters.values()))).encoder
