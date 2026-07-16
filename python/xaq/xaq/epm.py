"""
EPM_V3 — Episodic Predictive Memory, v3 tabula rasa implementation.

Pipeline per tick:
    raw input
        → FrozenEncoder.encode()        → 128D latent vector
        → GNG.step()                    → (winner_id, quantization_error)
        → _transition_surprise()        → temporal TLE component
        → dual TLE                      = α * quant_error + β * trans_surprise
        → _update_transitions()
        → EPMResult (drop-in for RealityToken)

No ONNX. No PyTorch. No pre-training. Pure numpy.

The EPMResult carries all fields needed to call transmitter.emit_reality()
and feed the existing brain server visualizations unchanged.
"""

import numpy as np
import math
import time
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

from .encoders import make_encoder
from .gng import GNG


# ---------------------------------------------------------------------------
# Result type (maps to RealityToken fields expected by the brain server)
# ---------------------------------------------------------------------------

@dataclass
class EPMResult:
    """Output of EPM_V3.process() — compatible with brain server token format."""
    # Core identity
    source_id: str
    timestamp: float

    # GNG output
    active_node_id: int
    history_trace: List[int]           # last N node IDs (oldest first)

    # TLE breakdown
    quantization_error: float          # GNG spatial surprise
    transition_surprise: float         # temporal/causal surprise
    tle: float                         # combined TLE

    # Brain-server "neurochemistry" fields (mapped from TLE / baking status)
    dopamine_level: float              # 0=fresh 0.5=crystallised 1.0=supernode
    serotonin_level: float             # inverse TLE (stability)
    confidence: float

    # Optional: current prototype vector (for centroid visualization)
    prototype: Optional[np.ndarray] = None

    # Encoder output vector (for cochleagram / feature strip visualization)
    encoder_output: Optional[np.ndarray] = None

    # Preprocessed frame as seen by the encoder (for 2D visualizer).
    # Still-image modalities: (H, W, C) float32 [0-1].
    # Optical-flow modalities: (H, W, 3) HSV→RGB visualization float32 [0-1].
    encoder_frame: Optional[np.ndarray] = None

    # audio spectral scalar (audio modality only)
    # Starts at mu_base (e.g. -0.5); goes more negative under loud input.
    moc_mu: float = 0.0

    # Maturity / Mitosis Gatekeeper signals
    crystallization_ratio: float = 0.0   # fraction of baked nodes (0→1)
    context_novelty: float = 0.0         # min dist from x to any baked prototype
    is_mature: bool = False              # EPM has crystallised its context

    # Flags
    is_novel: bool = False
    just_crystallised: bool = False


# ---------------------------------------------------------------------------
# EPM_V3
# ---------------------------------------------------------------------------

class EPM_V3:
    """
    A single v3 EPM node for one sensory modality.

    Parameters
    ----------
    modality : str
        e.g. 'audio', 'retinal', 'color', 'optical_flow', 'saliency'
    agent_id : str
        Unique identifier sent in RealityTokens (e.g. 'breakout_audio').
    projection_dim : int
        Latent space dimensionality (encoder output == GNG input).
    sample_rate : int
        Audio sample rate (audio modality only).
    alpha : float
        Weight for quantization_error in combined TLE.
    beta : float
        Weight for transition_surprise in combined TLE.
    gng_kwargs : dict
        Extra keyword arguments forwarded to GNG constructor.
    """

    def __init__(self,
                 modality: str,
                 agent_id: str = "",
                 projection_dim: int = 128,
                 sample_rate: int = 48000,
                 alpha: float = 0.6,
                 beta: float = 0.4,
                 gng_kwargs: Optional[Dict[str, Any]] = None):

        self.modality    = modality
        self.agent_id    = agent_id or f"v3_{modality}"
        self.alpha       = alpha
        self.beta        = beta
        self.projection_dim = projection_dim

        # Components
        self.encoder = make_encoder(modality, projection_dim=projection_dim,
                                    sample_rate=sample_rate)
        self.gng = GNG(dim=projection_dim, **(gng_kwargs or {}))

        # Transition table: sparse counts
        # transition_counts[prev_id][curr_id] = count
        self._transition_counts: Dict[int, Dict[int, int]] = {}
        self._node_totals: Dict[int, int] = {}

        # State
        self._prev_node_id: Optional[int] = None
        self._ticks: int = 0

        # Running average TLE (for normalisation and homeostasis)
        self._running_avg_tle: float = 0.5
        self._ema_alpha: float = 0.05

        # Maturity tracking — for Mitosis Gatekeeper
        # Steps where the winner was a baked node (crystallised hits)
        self._crystallised_hits: int = 0
        # EMA of context_novelty — how far recent inputs are from baked prototypes
        self._novelty_ema: float = 1.0
        self._novelty_ema_alpha: float = 0.1
        # Maturity threshold: EPM is "mature" when crystallization_ratio exceeds
        # this AND sustained_low_tle has held for enough steps.
        self.maturity_threshold: float = 0.75  # 75% of nodes crystallised

        # Thread safety
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Primary interface
    # ------------------------------------------------------------------

    def process(self, raw_input: np.ndarray) -> EPMResult:
        """
        Run one tick.

        Args:
            raw_input: numpy array. Shape depends on modality:
                audio       → 1-D audio samples (float32)
                retinal     → (H, W) or (H, W, 1) grayscale
                color       → (H, W, 3) RGB
                optical_flow→ (H, W, 2) flow vectors
                saliency    → (H, W) or (H, W, 1) saliency map

        Returns:
            EPMResult with all fields populated.
        """
        with self._lock:
            self._ticks += 1
            t = time.time()

            # 1. Encode
            embedding = self.encoder.encode(raw_input)

            # 2. GNG step
            winner_id, quant_error = self.gng.step(embedding)

            # 3. Transition surprise
            trans_surprise = self._transition_surprise(self._prev_node_id, winner_id)

            # 4. Dual TLE
            tle = self.alpha * quant_error + self.beta * trans_surprise

            # 5. Update transition table
            self._update_transitions(self._prev_node_id, winner_id)
            self._prev_node_id = winner_id

            # 6. Running average TLE (homeostasis)
            self._running_avg_tle = (
                (1.0 - self._ema_alpha) * self._running_avg_tle
                + self._ema_alpha * tle
            )
            self._running_avg_tle = max(self._running_avg_tle, 1e-4)

            # 7. Neurochemistry mapping
            serotonin = 1.0 / (1.0 + tle / max(self._running_avg_tle, 1e-4))
            serotonin = float(np.clip(serotonin, 0.0, 1.0))

            visit_count = self.gng.get_visit_count(winner_id)
            bt = self.gng.baking_threshold
            just_crystallised = (visit_count == bt)   # crossed threshold this tick
            crystallised = (visit_count >= bt)

            # 7b. Maturity signals for Mitosis Gatekeeper
            cryst_ratio  = self.gng.crystallization_ratio
            ctx_novelty  = self.gng.context_novelty(embedding)
            self._novelty_ema = (
                (1.0 - self._novelty_ema_alpha) * self._novelty_ema
                + self._novelty_ema_alpha * ctx_novelty
            )
            # EPM is "mature" when most nodes are baked AND recent inputs
            # consistently land near crystallised prototypes.
            is_mature = (
                cryst_ratio >= self.maturity_threshold
                and self.gng.baked_count >= 3         # need meaningful topology
            )

            if crystallised:
                dopamine = 1.0
            elif visit_count > bt // 2:
                dopamine = 0.5
            else:
                dopamine = max(0.0, float(visit_count) / max(bt // 2, 1)) * 0.5

            is_novel = (visit_count <= 1)   # first visit = novel

            # 8. History trace
            history = self.gng.get_history(5)

            # 9. Prototype for centroid visualisation
            prototype = self.gng.get_prototype(winner_id)

            return EPMResult(
                source_id=self.agent_id,
                timestamp=t,
                active_node_id=winner_id,
                history_trace=history,
                quantization_error=quant_error,
                transition_surprise=trans_surprise,
                tle=tle,
                dopamine_level=dopamine,
                serotonin_level=serotonin,
                confidence=serotonin,
                prototype=prototype,
                encoder_output=embedding,
                encoder_frame=getattr(self.encoder, 'last_encoded_frame', None),
                moc_mu=getattr(self.encoder, 'last_mu', 0.0),
                crystallization_ratio=cryst_ratio,
                context_novelty=float(ctx_novelty) if ctx_novelty != float('inf') else 1.0,
                is_mature=is_mature,
                is_novel=is_novel,
                just_crystallised=just_crystallised,
            )

    def get_stats(self) -> Dict[str, Any]:
        """Summary stats for logging / CLI display."""
        with self._lock:
            return {
                "modality": self.modality,
                "agent_id": self.agent_id,
                "ticks": self._ticks,
                "gng_nodes": self.gng.node_count,
                "running_avg_tle": self._running_avg_tle,
                "transition_table_size": sum(len(v) for v in self._transition_counts.values()),
            }

    def reset(self):
        """Full cognitive reset — wipes GNG and transition table."""
        with self._lock:
            self.gng = GNG(dim=self.projection_dim)
            self._transition_counts.clear()
            self._node_totals.clear()
            self._prev_node_id = None
            self._ticks = 0
            self._running_avg_tle = 0.5

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _transition_surprise(self, prev_id: Optional[int], curr_id: int) -> float:
        """
        -log P(curr_id | prev_id) from the transition frequency table.

        If prev_id is None (first tick) or the transition has never been seen,
        returns log(max(node_count, 1)) — the maximum-entropy baseline.
        """
        if prev_id is None:
            n = max(self.gng.node_count, 1)
            return math.log(n)

        total = self._node_totals.get(prev_id, 0)
        if total == 0:
            n = max(self.gng.node_count, 1)
            return math.log(n)

        count = self._transition_counts.get(prev_id, {}).get(curr_id, 0)
        # Laplace smoothing: add 1 to avoid log(0)
        p = (count + 1.0) / (total + self.gng.node_count)
        return -math.log(p)

    def _update_transitions(self, prev_id: Optional[int], curr_id: int):
        if prev_id is None:
            return
        if prev_id not in self._transition_counts:
            self._transition_counts[prev_id] = {}
        row = self._transition_counts[prev_id]
        row[curr_id] = row.get(curr_id, 0) + 1
        self._node_totals[prev_id] = self._node_totals.get(prev_id, 0) + 1
