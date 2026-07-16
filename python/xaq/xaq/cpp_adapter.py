"""
CppEPMAdapter — Python front-end for the C++ ami-ogma-v3 EPM binary.

Implements the same interface as BrainV3Adapter so BrainThread and the UI
require no modifications to use the C++ backend.

Architecture:
  - Launches ami-ogma-v3 as a subprocess
  - Communicates via TCP JSON (newline-delimited) to the C++ ControlServer
  - Encodes frames / audio as base64 in the JSON payload
  - Decodes RealityToken JSON responses and returns the same stats dict
    that VisualizerPanel.update_plots() expects

Usage (in main_window.py or app startup):
    from xaq.cpp_adapter import CppEPMAdapter
    brain = CppEPMAdapter(modality="retinal", projection_dim=128)
    # Pass to AMI_Ogma_Window(brain=brain) exactly like BrainV3Adapter
"""

import base64
import json
import socket
import subprocess
import threading
import time
import os
import sys
import numpy as np
from typing import Optional, Dict, Any


# Path to the C++ binary relative to the repo root
_CPP_BINARY = os.path.join(
    os.path.dirname(__file__),
    "..", "..", "cpp_core", "build", "ami-ogma-v3"
)


class _EncoderStub:
    """Minimal encoder stub — BrainThread reads mel_config for sample rate."""
    def __init__(self, sample_rate: int, modality: str):
        self.mel_config = {"sample_rate": sample_rate}
        self.modality   = modality


class _SessionLoggerStub:
    file_path: Optional[str] = None


class _MemoryProxy:
    """Proxy for brain.memory.baking_threshold used by the UI."""
    def __init__(self): self.baking_threshold = 50

    @property
    def baking_threshold(self): return self._bt
    @baking_threshold.setter
    def baking_threshold(self, v): self._bt = int(v)
    # init
    _bt = 50


class _TransmitterStub:
    app_state  = "PAUSED"
    enabled    = False
    connected  = False
    server_url = ""
    def emit_status(self, s): pass
    def get_local_ip(self): return "127.0.0.1"


class _ReceiverStub:
    enabled = False
    def read(self): return None


# ---------------------------------------------------------------------------
# TCP JSON client (newline-delimited)
# ---------------------------------------------------------------------------

class _TCPClient:
    """
    Persistent TCP client — reuses a single socket connection across calls to
    avoid per-call TCP handshake overhead (which was the primary framerate
    bottleneck when sending large base64 frames at high fps).

    Reconnects automatically on any socket error.
    """
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self._host    = host
        self._port    = port
        self._timeout = timeout
        self._lock    = threading.Lock()
        self._sock: "socket.socket | None" = None

    def _connect(self) -> socket.socket:
        s = socket.create_connection((self._host, self._port), timeout=self._timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # disable Nagle
        s.settimeout(self._timeout)
        return s

    def _ensure_connected(self) -> socket.socket:
        if self._sock is None:
            self._sock = self._connect()
        return self._sock

    def _reconnect(self) -> socket.socket:
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        self._sock = self._connect()
        return self._sock

    def call(self, cmd: dict) -> dict:
        with self._lock:
            payload = (json.dumps(cmd) + "\n").encode()
            for attempt in range(2):   # one retry on connection error
                try:
                    s = self._ensure_connected()
                    s.sendall(payload)
                    buf = b""
                    while True:
                        chunk = s.recv(65536)
                        if not chunk:
                            raise ConnectionResetError("server closed connection")
                        buf += chunk
                        if buf.endswith(b"\n"):
                            break
                    return json.loads(buf.decode())
                except Exception as e:
                    if attempt == 0:
                        # Connection may have gone stale — reconnect and retry
                        try:
                            self._reconnect()
                        except Exception:
                            self._sock = None
                    else:
                        return {"status": "error", "message": str(e)}
            return {"status": "error", "message": "failed after retry"}

    def close(self):
        with self._lock:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None


# ---------------------------------------------------------------------------
# CppEPMAdapter
# ---------------------------------------------------------------------------

_VIDEO_MODALITIES = {
    "retinal", "color", "optical_flow", "saliency", "dorsal", "ventral"
}


class CppEPMAdapter:
    """
    Drop-in replacement for BrainV3Adapter that delegates all computation
    to the C++ ami-ogma-v3 binary.

    Implements the full brain interface expected by BrainThread + AMI_Ogma_Window.
    """

    def __init__(self, modality: str = "retinal",
                 projection_dim: int = 128,
                 port: int = 7200,
                 binary_path: Optional[str] = None,
                 _start_zmq_sub: bool = True,
                 audio_mode: str = "stft",
                 encoder_res: int = 0,
                 proprio_state_dims: int = 6,
                 inject_centroid: bool = False,
                 centroid_gain: float = 22.6):
        # _start_zmq_sub is reserved for MultiEPMAdapter which manages its own
        # shared ZMQ SUB; CppEPMAdapter does not own a ZMQ SUB socket.
        # audio_mode: "stft" (generic spectral front-end)
        # inject_centroid: append 2D saliency center-of-mass before JL projection
        # (saliency modality only; other modalities ignore the flag).

        self._modality      = modality
        self._projection_dim = projection_dim
        self._port          = port
        self._binary_path   = os.path.abspath(binary_path or _CPP_BINARY)
        self._audio_mode = audio_mode.lower()
        self._encoder_res   = encoder_res
        self._proprio_dims  = proprio_state_dims
        self._inject_centroid = bool(inject_centroid)
        self._centroid_gain   = float(centroid_gain)

        # UI-facing attributes (mirrors BrainV3Adapter)
        self.model_type           = modality
        self.brain_name           = f"cpp-epm-{modality}"
        self.max_history          = 5
        self.is_bootstrapping     = False
        self.dream_mode           = False
        self.processing_lock      = threading.RLock()
        self.transmitter          = _TransmitterStub()
        self.receiver             = _ReceiverStub()
        self.session_logger       = _SessionLoggerStub()
        self.encoder              = _EncoderStub(48000, modality)
        self.memory               = _MemoryProxy()
        self.adaptation_rate      = 0.05
        self.threshold_multiplier = 1.5
        self.distance_metric      = "euclidean"
        self.baked_suppression    = 0.5
        self.node_decay_enabled   = True
        self.threshold            = 0.5

        import torch
        self.device = torch.device("cpu")

        self._node_labels: Dict[int, str] = {}
        self._node_colors: Dict[int, str] = {}
        self._last_viz_time  = 0.0
        self._viz_interval   = 1.0 / 30
        self._prev_pca_Vt: Optional[np.ndarray] = None
        self._viz_scale:     float = 20.0
        self._last_audio_chunk: Optional[np.ndarray] = None
        self._stereo_mode    = False

        # Last token cache (for empty_stats fallback)
        self._last_node_count = 0
        self._last_baked_count = 0

        # Winner history — last 8 node IDs for temporal trail visualisation
        self._winner_history: list = []

        # Param cache — replayed into the subprocess after any restart
        self._param_cache: dict = {}
        self._last_stats: dict = {}
        self._prototype_cache: Dict[int, np.ndarray] = {}

        # Launch subprocess and connect
        self._proc: Optional[subprocess.Popen] = None
        self._tcp: Optional[_TCPClient] = None
        self._start_subprocess()

    # ------------------------------------------------------------------
    # Subprocess lifecycle
    # ------------------------------------------------------------------

    def _start_subprocess(self):
        if not os.path.exists(self._binary_path):
            raise FileNotFoundError(
                f"C++ binary not found: {self._binary_path}\n"
                f"Run:  cd cpp_core/build && cmake .. && make ami-ogma-v3"
            )

        cmd = [
            self._binary_path,
            self._modality,
            str(self._projection_dim),
            str(self._port),
        ]
        if self._encoder_res > 0:
            cmd.extend(["--encoder-res", str(self._encoder_res)])
        if self._modality == "proprioceptive":
            cmd.extend(["--proprio-dims", str(self._proprio_dims)])
        if self._inject_centroid and self._modality == "saliency":
            cmd.append("--inject-centroid")
            cmd.extend(["--centroid-gain", f"{self._centroid_gain:.4f}"])
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        print(f"[CppEPMAdapter] Launched: {' '.join(cmd)}")

        # Wait for "ready" line on stdout (max 10s)
        deadline = time.time() + 10.0
        while time.time() < deadline:
            line = self._proc.stdout.readline()
            if line:
                print(f"[cpp] {line.rstrip()}")
            if "ready" in line.lower():
                break
            if self._proc.poll() is not None:
                raise RuntimeError(
                    f"C++ binary exited unexpectedly (code={self._proc.returncode})")
        else:
            raise TimeoutError("C++ binary did not report ready within 10s")

        # Start a thread to drain stdout so the subprocess never blocks
        def drain():
            for line in self._proc.stdout:
                if line.strip():
                    print(f"[cpp] {line.rstrip()}")
        threading.Thread(target=drain, daemon=True).start()

        self._tcp = _TCPClient("127.0.0.1", self._port)

    def _stop_subprocess(self):
        if self._tcp:
            try:
                self._tcp.call({"command": "shutdown"})
            except Exception:
                pass
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None
        self._tcp  = None

    def __del__(self):
        self._stop_subprocess()

    # ------------------------------------------------------------------
    # Primary tick methods
    # ------------------------------------------------------------------

    def process_chunk(self, audio_chunk=None, video_chunk=None,
                      external_label=None) -> dict:
        if self._modality == "audio":
            return self._process_audio(audio_chunk)
        else:
            return self._process_video(video_chunk)

    def _process_video(self, frame) -> dict:
        if frame is None:
            return self._empty_stats()

        frame_np = np.asarray(frame, dtype=np.uint8)
        if frame_np.ndim == 2:
            frame_np = frame_np[:, :, np.newaxis]

        h, w, c = frame_np.shape
        b64 = base64.b64encode(frame_np.tobytes()).decode()

        resp = self._tcp.call({
            "command":  "process_video_frame",
            "pixels":   b64,
            "height":   h,
            "width":    w,
            "channels": c,
        })

        if resp.get("status") != "ok":
            return self._empty_stats()

        s = self._resp_to_stats(resp, audio_monitor=None)
        self._last_stats = s
        return s

    def _process_video_shm(self, shm_info: dict) -> dict:
        """Process a video frame from a shared-memory region (zero-copy path).

        shm_info must contain:
          shm_key  — POSIX shm name (without leading '/'; C++ adds it)
          height, width, channels
        """
        resp = self._tcp.call({
            "command":  "process_video_frame_shm",
            "shm_key":  shm_info["shm_key"],
            "height":   shm_info["height"],
            "width":    shm_info["width"],
            "channels": shm_info["channels"],
        })

        if resp.get("status") != "ok":
            return self._empty_stats()

        s = self._resp_to_stats(resp, audio_monitor=None)
        self._last_stats = s
        return s

    def _process_audio(self, audio_chunk) -> dict:
        if audio_chunk is None:
            return self._empty_stats()

        import torch
        if torch.is_tensor(audio_chunk):
            chunk_np = audio_chunk.detach().cpu().numpy().squeeze()
        else:
            chunk_np = np.asarray(audio_chunk, dtype=np.float32).squeeze()

        if chunk_np.ndim == 2:
            if self._stereo_mode and chunk_np.shape[0] == 2:
                pass  # (2, T) — stereo
            else:
                chunk_np = chunk_np.mean(axis=0)

        chunk_np = chunk_np.astype(np.float32)
        if np.abs(chunk_np).max() < 1e-4:
            return self._empty_stats()

        self._last_audio_chunk = chunk_np

        if chunk_np.ndim == 2:
            # stereo: (2, T) → interleaved (T*2,) float32
            channels  = 2
            n_samples = chunk_np.shape[1]
            flat = chunk_np.T.flatten()   # (T, 2) → (T*2,)
        else:
            channels  = 1
            n_samples = len(chunk_np)
            flat = chunk_np

        b64 = base64.b64encode(flat.tobytes()).decode()
        resp = self._tcp.call({
            "command":  "process_audio_chunk",
            "samples":  b64,
            "n_samples": n_samples,
            "channels": channels,
        })

        if resp.get("status") != "ok":
            return self._empty_stats()

        s = self._resp_to_stats(resp, audio_monitor=chunk_np)
        self._last_stats = s
        return s

    def _resp_to_stats(self, resp: dict, audio_monitor) -> dict:
        node_count = resp.get("node_count", 0)
        baked      = resp.get("baked_count", 0)
        self._last_node_count  = node_count
        self._last_baked_count = baked

        winner_id     = resp.get("winner_id", -1)
        is_novel      = resp.get("is_novel", False)
        just_baked    = resp.get("just_baked", False)
        just_mitosis  = resp.get("just_mitosis", False)
        mitosis_count = resp.get("mitosis_count", 0)
        tle           = resp.get("tle", 0.0)
        threshold     = resp.get("threshold", self.threshold)
        qe            = resp.get("quant_error", 0.0)
        ts            = resp.get("transition_surp", 0.0)
        cratio        = resp.get("crystallization_ratio", 0.0)
        mean_err      = resp.get("running_mean_error", 0.0)
        just_pruned   = resp.get("just_pruned", False)
        pruned_ids    = resp.get("pruned_ids", [])
        
        self.threshold = threshold  # keep Python-side in sync for UI reads

        # Decode latent vector from base64 float32
        latent = None
        latent_b64 = resp.get("latent_b64", "")
        latent_dim = resp.get("latent_dim", 0)
        if latent_b64 and latent_dim > 0:
            import base64
            raw = base64.b64decode(latent_b64)
            latent = np.frombuffer(raw, dtype=np.float32).copy()

        # Track winner history for temporal trail
        if winner_id >= 0:
            with self.processing_lock:
                self._winner_history.append(winner_id)
                if len(self._winner_history) > 8:
                    self._winner_history.pop(0)

        color = ("#00FF00" if is_novel else
                 "#FFD700" if just_baked else "#00CFFF")

        return {
            "tle":              tle,
            "threshold":        threshold,
            "loss":             qe,
            "nl":               0.0,   # spectral scalar not exposed from C++ yet
            "active_node":      winner_id,
            "active_label":     self._node_labels.get(winner_id, ""),
            "active_color":     self._node_colors.get(winner_id, color),
            "node_count":       node_count,
            "is_novel":         is_novel,
            "just_baked":       just_baked,
            "just_mitosis":     just_mitosis,
            "mitosis_count":    mitosis_count,
            "is_bootstrapping": False,
            "brain_profile":    {"total": 0},
            "quant_error":      qe,
            "trans_surprise":   ts,
            "dopamine":         0.0,
            "serotonin":        0.0,
            "gng_nodes":        node_count,
            "gng_baked":        baked,
            "gng_mean_error":   mean_err,
            "just_pruned":      just_pruned,
            "pruned_ids":       pruned_ids,
            "gng_converged":    mean_err < 0.01,
            "crystallization_ratio": cratio,
            "context_novelty":  0.0,
            "is_mature":        cratio > 0.8,
            "encoder_type":     self._modality,
            "encoder_output":   None,
            "encoder_frame":    None,
            "audio_monitor":    audio_monitor,
            "latent":           latent,
        }

    def _empty_stats(self) -> dict:
        return {
            "tle": 0.0, "threshold": self.threshold, "loss": 0.0, "nl": 0.0,
            "active_node": -1, "node_count": self._last_node_count,
            "is_novel": False, "is_bootstrapping": False,
            "brain_profile": {"total": 0},
            "quant_error": 0.0, "trans_surprise": 0.0,
            "gng_nodes": self._last_node_count,
            "gng_baked": self._last_baked_count,
            "crystallization_ratio": 0.0,
            "context_novelty": 0.0, 
            "is_mature": False,
            "just_pruned": False,
            "pruned_ids": [],
        }

    # ------------------------------------------------------------------
    # Spatial probe — C++ backend does not expose encoder internals
    # ------------------------------------------------------------------

    def get_spatial_probe(self) -> Optional[dict]:
        return None

    def rebuild_encoder(self, **kwargs) -> bool:
        # C++ encoder params are set at subprocess launch time.  Requires
        # full restart; not supported live.
        return False

    # ------------------------------------------------------------------
    # Visualization data (matches BrainV3Adapter.get_visualization_data)
    # ------------------------------------------------------------------

    def get_visualization_data(self) -> Optional[dict]:
        now = time.time()
        if now - self._last_viz_time < self._viz_interval:
            return None
        self._last_viz_time = now

        resp = self._tcp.call({"command": "get_topology"})
        if resp.get("status") != "ok":
            return None

        nodes_list = resp.get("nodes", [])
        edges_list = resp.get("edges", [])
        if not nodes_list:
            return None

        N = len(nodes_list)
        node_ids   = np.array([n["id"]          for n in nodes_list], dtype=np.int64)
        prototypes = np.array([n["prototype"]    for n in nodes_list], dtype=np.float32)

        # Update local prototype cache for the Predictor
        with self.processing_lock:
            for ni, proto in zip(node_ids, prototypes):
                self._prototype_cache[int(ni)] = proto

        visits     = np.array([n["visits"]       for n in nodes_list], dtype=np.int32)
        errors     = np.array([n.get("error", 0.0)     for n in nodes_list], dtype=np.float32)
        ema_errs   = np.array([n.get("ema_error", 0.0) for n in nodes_list], dtype=np.float32)
        cryst      = np.array([n["visits"] >= resp.get("baking_threshold", 50)
                               for n in nodes_list], dtype=bool)

        # PCA 2D layout
        pca_coords = np.zeros((N, 2), dtype=np.float32)
        if N >= 2:
            try:
                centered = prototypes - prototypes.mean(axis=0)
                _, _, Vt = np.linalg.svd(centered, full_matrices=False)
                Vt2 = Vt[:2].copy()  # Current top 2 components (2, D)

                if self._prev_pca_Vt is not None and self._prev_pca_Vt.shape == Vt2.shape:
                    # Robust Axis Alignment: maximize correlation with previous frame
                    # This handles both sign-flips (180deg) AND axis-swaps (90deg)
                    C = Vt2 @ self._prev_pca_Vt.T  # 2x2 correlation matrix
                    
                    # Option A: Identity-like (0->0, 1->1)
                    score_a = abs(C[0, 0]) + abs(C[1, 1])
                    # Option B: Swap-like (0->1, 1->0)
                    score_b = abs(C[0, 1]) + abs(C[1, 0])
                    
                    if score_b > score_a:
                        # Axes swapped (PC1 and PC2 changed places)
                        Vt2 = Vt2[[1, 0], :]
                        C = Vt2 @ self._prev_pca_Vt.T
                    
                    # Fix signs
                    if C[0, 0] < 0: Vt2[0] *= -1
                    if C[1, 1] < 0: Vt2[1] *= -1
                
                self._prev_pca_Vt = Vt2.copy()
                pca_coords = (centered @ Vt2.T).astype(np.float32)

                # Leaky Normalization (Anti-Pumping)
                # We use a rolling maximum magnitude to scale the coordinates
                # so that the graph doesn't instantly shrink when one node moves out.
                mx = float(np.abs(pca_coords).max())
                self._viz_scale = 0.92 * self._viz_scale + 0.08 * max(5.0, mx)
                pca_coords = (pca_coords / self._viz_scale) * 20.0
                
            except Exception:
                pass

        # Edge adjacency
        id_to_idx = {int(nid): i for i, nid in enumerate(node_ids)}
        edges_mat = np.zeros((N, N), dtype=np.float32)
        for edge in edges_list:
            pos_a, pos_b = edge.get("positions", [None, None])
            # In the C++ serialisation, positions are node IDs
            ai = id_to_idx.get(pos_a)
            bi = id_to_idx.get(pos_b)
            if ai is not None and bi is not None:
                edges_mat[ai, bi] = max(edges_mat[ai, bi], 0.3)
                edges_mat[bi, ai] = max(edges_mat[bi, ai], 0.3)

        # Node metadata
        max_visits = max(int(visits.max()), 1)
        nodes_meta = []
        for i, n in enumerate(nodes_list):
            nid = n["id"]
            is_baked = bool(cryst[i])
            color = self._node_colors.get(
                nid,
                "#FFD700" if is_baked else "#00CFFF"
            )
            nodes_meta.append({
                "id":        nid,
                "label":     self._node_labels.get(nid, ""),
                "color":     color,
                "count":     int(visits[i]),
                "salience":  float(visits[i]) / max_visits,
                "baked":     is_baked,
                "is_super":  False,
                "accuracy":  -1.0,
                "error":     float(errors[i]),
                "ema_error": float(ema_errs[i]),
            })

        # Build last_transition from the two most recent distinct winner IDs
        last_transition = None
        h = self._winner_history
        if len(h) >= 2 and h[-1] != h[-2]:
            last_transition = {"source": h[-2], "target": h[-1]}

        return {
            "nodes":           nodes_meta,
            "indices":         node_ids,
            "embeddings":      prototypes,
            "edges":           edges_mat,
            "pca_coords":      pca_coords,
            "last_transition": last_transition,
        }

    def get_prototype(self, node_id: int) -> Optional[np.ndarray]:
        """Return the latent prototype for a given node ID from the local cache."""
        with self.processing_lock:
            return self._prototype_cache.get(int(node_id))

    # ------------------------------------------------------------------
    # Parameter controls (UI sliders)
    # ------------------------------------------------------------------

    def update_neuro_dynamics(self, params: dict):
        # Update local cache first so restarts replay everything
        self._param_cache.update(params)

        mapping = {
            "min_insertion_error":       "min_insertion_error",
            "epsilon_b":                 "epsilon_b",
            "epsilon_n":                 "epsilon_n",
            "gng_lambda_new":            "lambda_new",
            "gng_max_age":               "max_age",
            "baking_threshold":          "baking_threshold",
            "node_decay_enabled":        "stale_prune_enabled",
            "threshold_multiplier":      "threshold_multiplier",
            "mitosis_error_threshold":   "mitosis_error_threshold",
            "mitosis_check_interval":    "mitosis_check_interval",
            "mitosis_split_distance":    "mitosis_split_distance",
            "health_boost":              "health_boost",
            "health_base_decay":         "health_base_decay",
            "health_resilience_k":       "health_resilience_k",
            "health_death_threshold":    "health_death_threshold",
            "max_deaths_per_tick":       "max_deaths_per_tick",
            "health_death_min_nodes":    "health_death_min_nodes",
            "death_cooldown_steps":      "death_cooldown_steps",
            "near_baked_fraction":       "near_baked_fraction",
            # Legacy compat: old metabolic param → new health model
            "metabolic_prunes_per_tick": "max_metabolic_prunes_per_tick",
        }
        for key, cpp_key in mapping.items():
            if key in params:
                value = params[key]
                if key == "node_decay_enabled":
                    value = 1.0 if bool(value) else 0.0
                self._tcp.call({"command": "set_param",
                                "param":   cpp_key,
                                "value":   float(value)})

        if "node_decay_seconds" in params:
            # stale_window_factor is now absolute steps; convert seconds→steps at ~30fps
            steps = max(300.0, float(params["node_decay_seconds"]) * 30.0)
            self._tcp.call({"command": "set_param",
                            "param":   "stale_window_factor",
                            "value":   steps})

        if "threshold_multiplier" in params:
            self.threshold_multiplier = float(params["threshold_multiplier"])

        if "baking_threshold" in params:
            self.memory.baking_threshold = int(params["baking_threshold"])

    def process_state(self, state_vector: list) -> dict:
        """Process a proprioceptive body state vector.

        state_vector: list of floats [position, velocity, acceleration, efference_error, ...]
        """
        resp = self._tcp.call({
            "command": "process_state",
            "state":   [float(v) for v in state_vector],
        })

        if resp.get("status") != "ok":
            return self._empty_stats()

        s = self._resp_to_stats(resp, audio_monitor=None)
        self._last_stats = s
        return s

    def boost_node(self, node_id: int, amount: int = 1):
        """Boost a GNG node's visit count toward baking threshold.

        Used by consensus crystallization: when downstream consensus stabilizes,
        the contributing upstream nodes get accelerated baking.
        """
        self._tcp.call({"command": "boost_node",
                        "node_id": int(node_id),
                        "amount":  int(amount)})

    def set_encoder(self, modality: str):
        """Switch modality — restarts the C++ subprocess and replays cached params."""
        self._stop_subprocess()
        self._modality        = modality
        self.model_type       = modality
        self.encoder.modality = modality
        self._node_labels.clear()
        self._node_colors.clear()
        self._prev_pca_Vt  = None
        self._last_node_count = 0
        self._last_baked_count = 0
        self._start_subprocess()
        # Re-apply all UI params so the fresh subprocess inherits current settings
        if self._param_cache:
            self.update_neuro_dynamics(self._param_cache)
        print(f"[CppEPMAdapter] Modality switched → {modality}")

    def update_senses(self, params: dict):
        if "sample_rate" in params:
            self.encoder.mel_config["sample_rate"] = int(params["sample_rate"])

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save_state(self, folder_path: str):
        os.makedirs(folder_path, exist_ok=True)
        path = os.path.join(folder_path, "gng_state_cpp.json")
        self._tcp.call({"command": "save", "path": path})
        print(f"[CppEPMAdapter] Saved GNG → {path}")

    def load_state(self, folder_path: str) -> bool:
        path = os.path.join(folder_path, "gng_state_cpp.json")
        if not os.path.exists(path):
            return False
        resp = self._tcp.call({"command": "load", "path": path})
        ok = resp.get("status") == "ok"
        if ok:
            print(f"[CppEPMAdapter] Loaded GNG ← {path}  ({resp.get('node_count', '?')} nodes)")
        return ok

    def save_graph_binary(self, path: str):
        self._tcp.call({"command": "save", "path": path})

    def save_encoder_weights(self, path: str):
        print("[CppEPMAdapter] Frozen encoder — no weights to save.")

    def load_encoder_weights(self, path: str):
        print("[CppEPMAdapter] Frozen encoder — no weights to load.")

    # ------------------------------------------------------------------
    # Graph editing
    # ------------------------------------------------------------------

    def update_node_attributes(self, node_id, attrs: dict):
        nid = int(node_id)
        if "label" in attrs:
            lbl = attrs["label"]
            if lbl: self._node_labels[nid] = lbl
            else:   self._node_labels.pop(nid, None)
        if "color" in attrs:
            self._node_colors[nid] = attrs["color"]

    def delete_nodes(self, node_ids: list):
        print("[CppEPMAdapter] delete_nodes not yet implemented for C++ backend")

    def prune_unbaked_nodes(self):
        print("[CppEPMAdapter] prune_unbaked not yet implemented for C++ backend")

    def reset(self):
        self._tcp.call({"command": "reset"})
        self._node_labels.clear()
        self._node_colors.clear()
        self._prev_pca_Vt    = None
        self._last_node_count = 0

    # ------------------------------------------------------------------
    # Stubs for unused v2 methods
    # ------------------------------------------------------------------

    def get_per_modality_stats(self) -> dict:
        """Return {modality: latest_stats} — compatible with MultiEPMAdapter interface."""
        return {self._modality: self._last_stats}

    def update_prediction_horizon(self, n): pass
    def update_predictor_lr(self, lr):      pass
    def set_predictor(self, ptype):         pass
    def set_frozen(self, v):                pass
    def set_stereo_mode(self, v):
        self._stereo_mode = bool(v)
    def set_dream_mode(self, v):            pass
    def start_bootstrap(self, ticks=0):     pass
    def stop_bootstrap(self):               pass
    def start_recording(self, path):        pass
    def stop_recording(self):               pass
    def detect_patterns(self):              pass
    def detect_temporal_clusters(self):     pass
    def group_by_labels(self):              pass
    def dissolve_all_supernodes(self):      pass
    def get_node_memories(self, node_id):   return []
