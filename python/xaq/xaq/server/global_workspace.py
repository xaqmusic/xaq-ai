import time
import numpy as np
import logging
import asyncio
import torch
from collections import deque
from xaq.meta_epm import MetaEPM
from xaq.server.benchmarks.workspace_interface import WorkspaceHarness

logger = logging.getLogger("GlobalWorkspace")

class GlobalWorkspaceHarness(WorkspaceHarness):
    """
    The True Active Inference Global Workspace.
    Integrates multi-modal consensus into a Meta-EPM and drives behaviour via
    Homeokinetic Error Minimization.

    Action decoding uses the cerebellum model:
        body_error = world_target_norm - current_body_norm
        velocity   = body_error * stiffness * gain

    world_target_norm  — PCA projection of GRU-predicted future state, scaled
                         to [-1, 1] via calibration statistics.
    current_body_norm  — paddle x from ProprioceptiveChannel.body_belief[0],
                         already in [-1, 1].

    A ProprioceptiveChannel can be attached via set_proprioceptive_channel().
    Without it the decoder falls back to the old world-absolute approach.
    """

    def __init__(self, voter, server, loop=None):
        super().__init__(voter, server)
        self.loop = loop or asyncio.get_event_loop()

        # 1. Meta-EPM (Layer 4)
        self.meta_epm = MetaEPM(device='cpu')
        self.is_active = False

        # 2. Voter Lifecycle API
        self.prediction_horizon = 1    # 1-step prediction: 8-step hallucination regressed
                                       # to a fixed attractor (x≈370-420); 1-step stays
                                       # close to current state and tracks ball movement
        self.stiffness = 1.0

        # 3. Homeokinetic state
        self.z_target = None
        self.last_pred = None

        # 4. Action Decoder Parameters
        self.action_stiffness = 0.6
        # Gain on normalised body_error → velocity.
        # With normalised coords, 120 is the sweet spot to avoid overshooting oscillations.
        self._action_gain = 120.0

        # 5. Calibration / Spatial Mapping
        self.calibration_buffer = []
        self.spatial_axis = None        # dominant PCA dim index (for UI)
        self.spatial_projection = None  # 768-dim PCA unit vector
        self.calibration_mean = None    # consensus mean — centres projection
        self.spatial_scale = 1.0        # std of (Z @ v) — normalises projection to [-1,1]
        self.spatial_sign = 1.0
        self.max_calibration_samples = 300
        self.is_calibrated = False

        # 6. Proprioceptive Channel (set by server after construction)
        self.proprioceptive_channel = None

        # 7. Cached future path (shared with viz widget)
        self.last_future_path = None

        # 8. Reinforcement State
        self.transition_trace = deque(maxlen=5)
        self.last_node_idx = None
        self.ema_velocity = 0.0
        self.ema_alpha = 0.6

    # ------------------------------------------------------------------
    # Wiring
    # ------------------------------------------------------------------

    def set_proprioceptive_channel(self, channel):
        """Attach a ProprioceptiveChannel.  Called by the server after init."""
        self.proprioceptive_channel = channel
        logger.info("GlobalWorkspace: ProprioceptiveChannel attached.")

    # ------------------------------------------------------------------
    # Active Inference Loop
    # ------------------------------------------------------------------

    def on_consensus(self, consensus, candidates):
        """Triggered by the Lateral Voter on each multi-modal consensus."""
        fused_z = consensus.fused_embedding

        contributors = [
            (c.source_id, c.active_node_id)
            for c in candidates
            if c.active_node_id != -1
        ]

        pred_current, tle = self.meta_epm.process_consensus(
            fused_z,
            contributors=contributors or None,
            association_matrix=self.voter.association_matrix if contributors else None,
        )

        # Transition tracking for reinforcement trace
        current_node_idx, _ = (
            self.meta_epm.memory.find_nearest_concept_warped(
                torch.from_numpy(fused_z).float().to(self.meta_epm.device),
                contributors, self.voter.association_matrix
            ) if contributors else
            self.meta_epm.memory.find_nearest_concept(
                torch.from_numpy(fused_z).float().to(self.meta_epm.device)
            )
        )
        if self.last_node_idx is not None and current_node_idx is not None:
            self.transition_trace.append((self.last_node_idx, current_node_idx))
        self.last_node_idx = current_node_idx

        if pred_current is None:
            return

        # Forward Projection — hallucinate future global state
        future_path = self.meta_epm.predict_future(horizon=self.prediction_horizon)
        self.last_future_path = future_path
        if not future_path:
            return

        z_target_proj = future_path[-1]

        # Body-relative action decoding (cerebellum model)
        velocity = self.decode_action(z_target_proj, fused_z)
        self.last_velocity = velocity

        if self.server and self.is_active:
            coro = self.server.broadcast_action(player=1, velocity=velocity)
            self.loop.call_soon_threadsafe(lambda: asyncio.create_task(coro))

            # Efference copy: update body belief before next sensor arrives
            if self.proprioceptive_channel is not None:
                self.proprioceptive_channel.on_action_sent({"velocity": velocity})

        logger.info(
            f"Active Inference: TLE={tle:.4f} | vel={velocity:.2f} "
            f"| sero={self.meta_epm.serotonin:.2f} | stiff={self.action_stiffness}"
        )

    # ------------------------------------------------------------------
    # Action Decoder — Cerebellum Model
    # ------------------------------------------------------------------

    def decode_action(self, z_target, z_real):
        """
        Cerebellum model: action = f(world_estimate - body_reference)

        Uses z_real (current consensus) as the world estimate.
        GRU-based prediction (z_target) was discarded: it converges to a fixed
        attractor at the mean training-distribution ball position regardless of
        actual ball state, because the GRU was trained on passive observations
        where the ball visited x≈368-416 most frequently.

        world_x_norm: PCA projection of z_real, normalised to [-1, 1].
        body_x_norm:  paddle x from ProprioceptiveChannel, [-1, 1].
        body_error = world_x_norm - body_x_norm → 0 when paddle tracks correctly.
        """
        # Reactive world estimate: use current consensus (z_real) directly.
        # GRU-based z_target converges to a mean-position attractor (x≈368-416) regardless
        # of actual ball state — a structural bias from passive training distribution.
        # Using z_real gives a PCA projection of the *current* multi-modal consensus,
        # which correlates with actual ball position and avoids the GRU attractor loop.
        z_r = np.array(z_real).flatten().astype(np.float64)
        z_use = z_r

        mean = self.calibration_mean if self.calibration_mean is not None else np.zeros_like(z_use)
        z_centered = z_use - mean

        # Project onto calibrated spatial axis
        if self.spatial_projection is not None:
            raw_proj = float(np.dot(z_centered, self.spatial_projection)) * self.spatial_sign
        elif self.spatial_axis is not None:
            raw_proj = float(z_centered[self.spatial_axis]) * self.spatial_sign
        else:
            raw_proj = float(z_centered[0])

        # Normalise to [-1, 1] using calibration statistics so the projection
        # is on the same scale as the body state from the adapter (also [-1, 1]).
        world_target_norm = raw_proj / self.spatial_scale

        # Body reference: paddle x from proprioceptive channel
        if self.proprioceptive_channel is not None and self.proprioceptive_channel.is_ready:
            body_x_norm = float(self.proprioceptive_channel.body_belief[0])
        else:
            body_x_norm = 0.0  # assume centre until channel is ready

        # Body-relative error: how far does the paddle need to move?
        body_error = world_target_norm - body_x_norm

        effective_stiffness = self.action_stiffness * (1.0 + self.meta_epm.dopamine)
        velocity = body_error * effective_stiffness * self._action_gain
        velocity = max(-15.0, min(15.0, velocity))

        self.ema_velocity = self.ema_alpha * velocity + (1.0 - self.ema_alpha) * self.ema_velocity
        return float(self.ema_velocity)

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def perform_calibration(self):
        """PCA Calibration: extract top principal component as spatial projection.

        Power-iteration SVD on the calibration window.  Also computes
        spatial_scale = 2 * std(Z @ v), which maps raw projections to the
        same [-1, 1] range as the normalised body state from the adapter.
        """
        logger.info("Workspace: Performing PCA Calibration (power iteration)...")
        Z = np.array(self.calibration_buffer, dtype=np.float64)   # (N, 768)
        self.calibration_mean = Z.mean(axis=0)
        Z -= self.calibration_mean

        # Power iteration for top-1 singular vector
        rng = np.random.default_rng(0xC0FFEE)
        v = rng.standard_normal(Z.shape[1])
        v /= np.linalg.norm(v)
        for _ in range(30):
            v = Z.T @ (Z @ v)
            norm = np.linalg.norm(v)
            if norm < 1e-12:
                break
            v /= norm

        self.spatial_projection = v
        self.spatial_axis = int(np.argmax(np.abs(v)))
        self.spatial_sign = 1.0

        # Compute scale: 2-sigma range of projection values ≈ [-1, 1] for typical inputs
        proj_values = Z @ v
        self.spatial_scale = max(float(np.std(proj_values)) * 2.0, 1e-6)

        self.is_calibrated = True

        var_pc1    = float(np.var(proj_values))
        var_total  = float(np.mean(np.var(Z, axis=0)))
        logger.info(
            f"Workspace: PCA Calibration complete — "
            f"PC1 variance {var_pc1:.5f}  (mean-dim variance {var_total:.5f}  "
            f"ratio {var_pc1/max(var_total,1e-12):.1f}x)  "
            f"dominant dim {self.spatial_axis}  "
            f"spatial_scale {self.spatial_scale:.4f}"
        )
        self.calibration_buffer = []

    # ------------------------------------------------------------------
    # Raw State (Game Telemetry)
    # ------------------------------------------------------------------

    def on_raw_state(self, data):
        """Handle raw telemetry: reinforcement, calibration, and proprioception."""

        # 1. Update proprioceptive channel (body state tracking)
        if self.proprioceptive_channel is not None:
            self.proprioceptive_channel.update(data)

        # 2. Reinforcement Events
        hit  = data.get('hit', False) or data.get('event') == 'self_paddle_hit'
        miss = data.get('miss', False) or data.get('event') == 'paddle_miss'

        if hit:
            logger.info("Workspace: REWARD (Paddle Hit)! Upgrading pathways.")
            self.meta_epm.dopamine = 1.0
            self.meta_epm.serotonin = min(1.0, self.meta_epm.serotonin + 0.2)
            for src, tgt in self.transition_trace:
                self.meta_epm.memory.reinforce_transition(src, tgt, factor=1.5)
            if self.voter and self.is_active:
                self.voter.retroactive_boost_pathways(factor=0.2)

        if miss:
            logger.warning("Workspace: PUNISH (Paddle Miss)! Downgrading pathways.")
            self.meta_epm.dopamine = 0.0
            self.meta_epm.serotonin = max(0.0, self.meta_epm.serotonin - 0.5)
            for src, tgt in self.transition_trace:
                self.meta_epm.memory.penalize_transition(src, tgt, factor=0.5)
            if self.voter and self.is_active:
                last_resonance = (self.voter.last_consensus.resonance_score
                                  if self.voter.last_consensus else 0.0)
                if last_resonance > self.voter.threshold:
                    self.voter.retroactive_penalize_pathways(factor=0.2)
                else:
                    logger.debug(
                        f"Workspace: Miss ignored for Hebbian penalty "
                        f"(resonance {last_resonance:.3f} ≤ threshold {self.voter.threshold:.3f})"
                    )
            self.meta_epm.reset_state()
            self.transition_trace.clear()

        # Decay dopamine slowly back to baseline
        self.meta_epm.dopamine *= 0.98

        # 3. Calibration (Passive mode only)
        if (not self.is_active and not self.is_calibrated
                and len(self.calibration_buffer) < self.max_calibration_samples):
            consensus = self.voter.last_consensus
            if consensus and consensus.fused_embedding is not None:
                self.calibration_buffer.append(consensus.fused_embedding.copy())
                if len(self.calibration_buffer) == self.max_calibration_samples:
                    self.perform_calibration()

    # ------------------------------------------------------------------
    # Utility / UI
    # ------------------------------------------------------------------

    def evaluate_stability(self, raw_state=None):
        return max(0.0, 1.0 - self.meta_epm.last_tle)

    def set_action_stiffness(self, val):
        self.action_stiffness = val
        logger.info(f"Harness: Action Stiffness updated to {val}")

    def set_prediction_horizon(self, val):
        self.prediction_horizon = int(val)
        logger.info(f"Harness: Prediction Horizon updated to {val}")

    def set_meta_learning_rate(self, val):
        self.meta_epm.set_learning_rate(val)
        logger.info(f"Harness: Meta-Learning Rate updated to {val}")

    def flip_spatial_sign(self):
        self.spatial_sign *= -1.0
        logger.info(f"Workspace: Spatial Sign Flipped. New Sign: {self.spatial_sign}")
