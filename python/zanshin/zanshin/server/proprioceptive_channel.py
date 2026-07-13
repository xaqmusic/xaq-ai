import time
import numpy as np
import logging
from .adapters.base import GameAdapter

logger = logging.getLogger("ProprioceptiveChannel")


class ProprioceptiveChannel:
    """
    Level-2 Proprioceptive Channel.

    Bridges the GameAdapter to the cognitive architecture:
      - Maintains a body_belief vector (current body state, normalized [-1,1])
      - Applies efference copy: updates body_belief the moment an action is
        emitted, before sensor confirmation arrives (prevents oscillation)
      - Emits Reality Tokens into the LateralVoterNode so the Meta-EPM GRU
        learns joint world+body dynamics

    This is a Python-native component living inside the brain server process.
    It is the precursor to a full C++ ProprioceptiveEPM (Phase 3 roadmap).

    See: docs/proprioception_and_embodiment_roadmap.md
    """

    SOURCE_ID = "proprioceptive"

    # Number of discrete position buckets.  10 buckets give ~64px resolution
    # on a 640px screen — coarse enough to bake recurring positions, fine
    # enough to distinguish left / centre / right meaningfully.
    N_NODES = 10

    def __init__(self, adapter: GameAdapter, voter, embedding_dim: int = 768,
                 emit_to_voter: bool = True, seed: int = 0xBEEF):
        self.adapter = adapter
        self.voter = voter
        self.embedding_dim = embedding_dim
        self.emit_to_voter = emit_to_voter

        # Fixed random projection: body_dim → embedding_dim
        # Column-normalised Gaussian — spreads low-dim body state into the
        # shared latent space used by all EPMs.
        rng = np.random.default_rng(seed)
        P = rng.standard_normal((embedding_dim, adapter.body_dim))
        col_norms = np.linalg.norm(P, axis=0, keepdims=True)
        self._projection = (P / np.maximum(col_norms, 1e-8)).astype(np.float64)

        # State
        self.body_belief = np.zeros(adapter.body_dim, dtype=np.float32)
        self.is_ready: bool = False
        self._last_action: dict = {}

        # Discrete node tracking — trajectory history for lateral voter binding
        self._active_node: int = 0
        self._history: list = []   # rolling window of recent node IDs

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def update(self, raw_state: dict):
        """
        Called on every raw_game_state event from the environment.
        Updates body_belief from the sensor reading, then optionally emits
        a Reality Token to the lateral voter.
        """
        encoded = self.adapter.encode_body_state(raw_state)
        if encoded is None:
            return  # This packet carries no body information

        # Trust the sensor reading directly.
        encoded = self.adapter.encode_body_state(raw_state)
        if encoded is not None:
            self.body_belief = encoded
        self.is_ready = True

        # Always update discrete node tracking (used by action decoder + voter)
        node_id = self._quantize(self.body_belief)
        self._active_node = node_id
        self._history.append(node_id)
        if len(self._history) > 5:
            self._history.pop(0)

        if self.emit_to_voter:
            self._emit_token()

    def on_action_sent(self, action: dict):
        """
        Efference copy: update body_belief the instant an action is emitted,
        without waiting for the next sensor packet.  This prevents the decoder
        from re-issuing the same command on the next tick because it hasn't
        yet seen the paddle move.
        """
        if not self.is_ready:
            return
        predicted = self.adapter.predict_next_body_state(
            self.body_belief, action, dt=0.033
        )
        self.body_belief = predicted
        self._last_action = action

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _quantize(self, body_state: np.ndarray) -> int:
        """
        Map the primary body axis (position) to a discrete node ID 0..N_NODES-1.

        Uses the first body state dimension (paddle_x, normalized [-1,1]).
        10 nodes give ~64px resolution on a 640px screen.  These nodes represent
        recurring body configurations and enable trajectory binding in the voter,
        analogous to place cells in the hippocampus.
        """
        px = float(np.float64(body_state[0]))  # cast to f64 to avoid f32 boundary artifacts
        idx = int((px + 1.0) * 0.5 * self.N_NODES)
        return max(0, min(self.N_NODES - 1, idx))

    def _emit_token(self):
        """Emit Reality Token using already-computed node and history."""
        # Project body state into shared embedding space
        emb = np.tanh(self._projection @ self.body_belief.astype(np.float64))
        norm = np.linalg.norm(emb)
        if norm > 1e-8:
            emb /= norm

        token_data = {
            "header": {"agent_id": self.SOURCE_ID},
            "payload": {
                "current_id": self._active_node,
                "trajectory": [{"id": n} for n in self._history],
                "neurotransmitters": {
                    # Depressurized authority — body state must not dominate
                    # visual/audio EPMs that carry the spatial world information.
                    # Lower values allow optical_flow/retinal spatial signals to shine.
                    "dopamine":  0.0,
                    "serotonin": 0.1,
                },
                "embedding": emb.tolist(),
            },
        }
        try:
            self.voter.ingest_token(token_data)
        except Exception as e:
            logger.debug(f"ProprioceptiveChannel: voter ingest error: {e}")
