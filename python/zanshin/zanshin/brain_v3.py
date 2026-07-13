"""
BrainV3 — simplified orchestrator for a single v3 EPM node.

This replaces RealTimeAudioBrain (legacy_v2) for v3 modalities.
It deliberately omits:
    - PyTorch / ONNX (no models to train or export)
    - Bootstrapping / PCA calibration
    - Encoder switching at runtime

What it keeps (by importing from legacy_v2):
    - NetworkTransmitter — same socket protocol, same UI visualizations
    - NetworkReceiver    — remote control commands from brain server
    - SessionLogger      — session event recording
    - AudioManager       — audio capture (audio modality)

The brain server (ogma_brain_server) connects via socket and receives
RealityTokens in the same format as v2 — existing visualizations
(consensus graph, resonance plot, matrix view, TLE oscilloscope) all work
without any changes to the server side.
"""

import threading
import time
import numpy as np
from typing import Optional, Dict, Any

from .epm import EPM_V3, EPMResult

# Infrastructure re-used from legacy_v2 (import by reference, not copy)
from zanshin_core.bus.transmitter import NetworkTransmitter
from zanshin_core.bus.receiver import NetworkReceiver
from zanshin_core.logging.session_logger import NullSessionLogger as SessionLogger


class BrainV3:
    """
    Single-modality v3 EPM orchestrator.

    Lifecycle:
        brain = BrainV3(modality='audio', agent_id='breakout_audio')
        brain.start()
        # ... feed chunks via brain.process_audio_chunk() or brain.process_video_chunk()
        brain.stop()
    """

    def __init__(self,
                 modality: str = "audio",
                 agent_id: str = "",
                 projection_dim: int = 128,
                 sample_rate: int = 48000,
                 alpha: float = 0.6,
                 beta: float = 0.4,
                 server_url: str = "http://localhost:5000",
                 gng_kwargs: Optional[Dict[str, Any]] = None):

        self.modality = modality
        self.agent_id = agent_id or f"v3_{modality}"
        self.sample_rate = sample_rate

        # Core EPM
        self.epm = EPM_V3(
            modality=modality,
            agent_id=self.agent_id,
            projection_dim=projection_dim,
            sample_rate=sample_rate,
            alpha=alpha,
            beta=beta,
            gng_kwargs=gng_kwargs,
        )

        # Infrastructure
        self.transmitter = NetworkTransmitter(
            server_url=server_url,
            agent_id=self.agent_id,
            modality=modality.upper(),
        )
        self.receiver = NetworkReceiver()
        self.session_logger = SessionLogger()

        # State
        self._running = False
        self._lock = threading.RLock()
        self._tick_count = 0
        self._last_result: Optional[EPMResult] = None
        self._result_callbacks = []

        # Remote-control params (settable by brain server)
        self.epm.alpha = alpha
        self.epm.beta  = beta

        # Graph emission throttle (don't flood the socket)
        self._last_graph_emit = 0.0
        self._graph_emit_interval = 2.0   # seconds

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Start transmitter and receiver."""
        self.transmitter.start()
        self.receiver.start()
        self._running = True

    def stop(self):
        """Graceful shutdown."""
        self._running = False
        self.transmitter.stop()
        self.receiver.stop()

    # ------------------------------------------------------------------
    # Processing
    # ------------------------------------------------------------------

    def process_audio_chunk(self, audio_chunk: np.ndarray,
                            external_label: str = "") -> Optional[EPMResult]:
        """
        Process one audio chunk (audio modality).

        Args:
            audio_chunk: numpy array of float32 audio samples (mono or stereo)
            external_label: optional text label for the current state

        Returns:
            EPMResult if processing succeeded, None otherwise.
        """
        if self.modality != "audio":
            return None
        return self._process(audio_chunk, external_label)

    def process_video_chunk(self, video_frame: np.ndarray,
                            external_label: str = "") -> Optional[EPMResult]:
        """
        Process one video frame (visual modalities).

        Args:
            video_frame: numpy array, shape (H, W), (H, W, C), or (C, H, W)
            external_label: optional text label

        Returns:
            EPMResult if processing succeeded, None otherwise.
        """
        if self.modality == "audio":
            return None
        return self._process(video_frame, external_label)

    def _process(self, raw_input: np.ndarray,
                 external_label: str = "") -> Optional[EPMResult]:
        """Core processing loop step — called for all modalities."""
        with self._lock:
            try:
                result = self.epm.process(raw_input)
                self._tick_count += 1
                self._last_result = result

                # Emit RealityToken to brain server
                self._emit_reality(result, external_label)

                # Periodically emit GNG topology for graph visualization
                now = time.time()
                if now - self._last_graph_emit > self._graph_emit_interval:
                    self._emit_graph()
                    self._last_graph_emit = now

                # Notify local callbacks (e.g., CLI stats printer)
                for cb in self._result_callbacks:
                    try:
                        cb(result)
                    except Exception:
                        pass

                return result

            except Exception as e:
                import logging
                logging.getLogger("BrainV3").error(
                    f"[{self.agent_id}] process error: {e}", exc_info=True)
                return None

    def _emit_reality(self, result: EPMResult, label: str = ""):
        """
        Send RealityToken to the brain server in the v2-compatible format.
        All existing visualizations (resonance plot, matrix view, consensus log)
        consume this format unchanged.
        """
        trajectory = [{"id": nid, "dt": result.tle}
                      for nid in result.history_trace]

        neurotransmitters = {
            "dopamine":      result.dopamine_level,
            "serotonin":     result.serotonin_level,
            "norepinephrine": 0.0,
            "acetylcholine":  0.0,
        }

        self.transmitter.emit_reality(
            current_id=result.active_node_id,
            trajectory=trajectory,
            neurotransmitters=neurotransmitters,
            current_dt=result.tle,
            text_label=label or "",
        )

    def _emit_graph(self):
        """
        Send GNG topology to brain server for graph visualization.
        Maps GNG nodes → the transmitter's node format.
        """
        topology = self.epm.gng.get_topology()
        nodes_data = []
        for n in topology["nodes"]:
            nodes_data.append({
                "id": n["id"],
                "label": f"n{n['id']}",
                "visits": n["visits"],
                "crystallised": n["crystallised"],
                # prototype sent as embedding for centroid viz
                "embedding": n["prototype"][:16],  # truncate to avoid socket bloat
            })

        if hasattr(self.transmitter, 'emit_graph') and nodes_data:
            try:
                self.transmitter.emit_graph(nodes_data)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Callbacks and remote control
    # ------------------------------------------------------------------

    def add_result_callback(self, fn):
        """Register a callback(EPMResult) called after each tick."""
        self._result_callbacks.append(fn)

    def set_alpha_beta(self, alpha: float, beta: float):
        """Adjust TLE weighting from remote control or CLI."""
        with self._lock:
            self.epm.alpha = alpha
            self.epm.beta  = beta

    def reset(self):
        """Full cognitive reset."""
        with self._lock:
            self.epm.reset()
            self._tick_count = 0
            self._last_result = None

    def get_stats(self) -> Dict[str, Any]:
        stats = self.epm.get_stats()
        stats["tick_count"] = self._tick_count
        return stats
