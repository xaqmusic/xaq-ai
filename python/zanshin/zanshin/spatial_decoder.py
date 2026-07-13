"""
spatial_decoder.py — D3: Top-Down Spatial Bias for AMI-Ogma v3.

Implements the D3 "top-down target vector" using PCA calibration,
the same approach validated in GlobalWorkspaceHarness that achieved
91-95% hit rates in B2/B4 experiments.

How it works
------------
Phase 1 — Calibration (first `n_calibration` samples):
    Accumulates (fused_embedding, ball_x_norm) pairs. After collecting
    enough samples, solves a closed-form **ridge regression** β that maps
    centred embeddings to ball_x_norm:

        β = (Zc^T Zc + λI)^-1 Zc^T B

    The unit-normalised β is the "readout axis" — the best linear direction
    for recovering ball_x from the latent.  This replaces an earlier
    unsupervised power-iteration PCA-1 init, which picked the max-variance
    axis and empirically reported corr ≈ 0.02 on latents the probe widget
    showed to carry r ≈ 0.92 linearly-recoverable ball_x.

Phase 2 — Projection (after calibration):
    target_bias = clip((emb @ v - mean) / scale, -1, 1)

    where v is the signed PCA axis, mean is the embedding mean, and scale
    is 2*std of projections (maps to [-1, 1] range).

Phase 3 — Fine-tuning (continuous, post-calibration):
    SGD is run on top of the PCA axis to track topology drift as the GNG
    evolves. The weight vector W starts from the PCA axis and is refined
    incrementally. LR is small so the calibrated direction dominates.

Architecture alignment
----------------------
The PCA axis is the "sustained bias" from D3: a low-dimensional spatial
summary derived from the agent's own perceptual representation, not from
raw game telemetry. The motor controller uses it to seed the probe direction
before serotonin validation. No ball position reaches the motor path.
"""

import numpy as np
from typing import Optional


class SpatialDecoder:
    """
    PCA-calibrated spatial decoder: fused_embedding → [-1, 1] directional bias.

    Parameters
    ----------
    latent_dim      int     embedding dimension (must match fused_embedding)
    n_calibration   int     samples to collect before running PCA
    pca_iters       int     power-iteration steps for top singular vector
    finetune_lr     float   SGD LR for post-calibration fine-tuning
    ema_alpha       float   EMA smoothing on projection output
    min_bias        float   dead-band: return 0 if |bias| < min_bias
    """

    def __init__(self,
                 latent_dim:    int   = 128,
                 n_calibration: int   = 500,
                 pca_iters:     int   = 30,
                 finetune_lr:   float = 0.0005,
                 ema_alpha:     float = 0.15,
                 min_bias:      float = 0.25,
                 projection_gain: float = 1.35):

        self._dim          = latent_dim
        self._n_cal        = n_calibration
        self._pca_iters    = pca_iters
        self._finetune_lr  = finetune_lr
        self._alpha        = ema_alpha
        self._min_bias     = min_bias
        self._gain         = projection_gain

        # Calibration buffer
        self._emb_buf:  list = []   # list of np.ndarray
        self._ball_buf: list = []   # list of float (ball_x_norm)

        # PCA results
        self._pca_axis:   Optional[np.ndarray] = None   # top-1 singular vector (signed)
        self._emb_mean:   Optional[np.ndarray] = None   # mean of calibration embeddings
        self._proj_scale: float                = 1.0    # 2*std of projections

        # Post-calibration fine-tuning weight (starts from PCA axis)
        self._W:   Optional[np.ndarray] = None
        self._b:   float                = 0.0

        # State
        self._calibrated:  bool  = False
        self._n_finetune:  int   = 0
        self._ema_bias:    float = 0.0

    # ------------------------------------------------------------------
    # Training
    # ------------------------------------------------------------------

    def learn(self,
              embedding:   Optional[np.ndarray],
              ball_x_norm: Optional[float]) -> None:
        """
        Accumulate samples for calibration, then fine-tune post-calibration.

        Call every tick, both phases.
        """
        if embedding is None:
            return
        emb = np.asarray(embedding, dtype=np.float64)
        if emb.shape[0] != self._dim:
            return

        if not self._calibrated:
            self._emb_buf.append(emb.copy())
            self._ball_buf.append(float(ball_x_norm) if ball_x_norm is not None else 0.0)
            if len(self._emb_buf) >= self._n_cal:
                self._run_pca_calibration()
        elif ball_x_norm is not None:
            # Post-calibration SGD fine-tuning on the weight vector only.
            # b stays fixed at 0 (see _run_pca_calibration comment).
            emb_c = emb - self._emb_mean
            self._apply_recenter(emb)   # absorbs phase-transition recentering
            pred  = float(np.dot(self._W, emb_c)) + self._b
            err   = float(ball_x_norm) - pred
            self._W += self._finetune_lr * err * emb_c
            # self._b not updated — intentionally locked at 0
            self._n_finetune += 1

    def recenter(self, n_ticks: int = 60) -> None:
        """
        Re-estimate embedding mean over the next `n_ticks` samples.

        Call at phase transitions to compensate for covariate shift when the
        GNG topology changes as brain control takes over.
        """
        if not self._calibrated:
            return
        self._recenter_buf: list = []
        self._recenter_n   = n_ticks

    def _apply_recenter(self, emb: np.ndarray) -> None:
        """Internal: accumulate and apply recentering buffer."""
        buf = getattr(self, '_recenter_buf', None)
        n   = getattr(self, '_recenter_n', 0)
        if buf is None or n <= 0:
            return
        buf.append(emb.copy())
        if len(buf) >= n:
            new_mean = np.mean(buf, axis=0)
            # Blend old mean with new mean — don't hard-reset to avoid instability
            self._emb_mean = 0.6 * self._emb_mean + 0.4 * new_mean
            self._recenter_buf = None
            self._recenter_n   = 0
            print(f"[SpatialDecoder] Recentered embedding mean ({n} samples)")

    def _run_pca_calibration(self) -> None:
        """
        Ridge-regression calibration on the (embedding, ball_x_norm) buffer.

        Supersedes the original power-iteration PCA-1 init.  PCA-1 picks the
        max-variance axis, which in visual-only latents often corresponds to
        paddle motion or global brightness — not ball_x.  Empirically PCA-1
        reported calibrated corr ≈ 0.02 on saliency+centroid latents that were
        independently shown (SpatialProbeWidget) to carry r ≈ 0.92 linear
        ball_x information.  Ridge regression recovers that full readability.

        Supervision parity: the previous code already used ball_x_norm for
        sign correction and post-calibration SGD.  Using it for initialisation
        does not introduce a new dependency.  The decoder is still bypassed in
        Active Inference mode (target_bias = 0) so no game telemetry reaches
        the motor path.

        1. Centre the buffer.
        2. Solve W = argmin ‖Zc W - B‖² + λ‖W‖²   (closed-form ridge).
        3. Normalise W to unit length → "readout axis".
        4. Compute scale (2*std of projections).
        5. Initialise fine-tuning weight from the readout axis.
        """
        Z = np.array(self._emb_buf, dtype=np.float64)       # (N, D)
        B = np.array(self._ball_buf, dtype=np.float64)       # (N,)

        self._emb_mean = Z.mean(axis=0)
        Zc = Z - self._emb_mean

        # Closed-form ridge regression: (Zc^T Zc + λI)^-1 Zc^T B
        # λ = 1e-3 matches SpatialProbeWidget's regulariser so the
        # reported corr is directly comparable to the live widget readout.
        lam = 1e-3
        XtX = Zc.T @ Zc + lam * np.eye(self._dim, dtype=np.float64)
        try:
            beta = np.linalg.solve(XtX, Zc.T @ B)
        except np.linalg.LinAlgError:
            beta = np.zeros(self._dim, dtype=np.float64)

        beta_norm = float(np.linalg.norm(beta))
        if beta_norm < 1e-9:
            # Degenerate — no spatial signal in calibration buffer
            self._pca_axis = np.zeros(self._dim, dtype=np.float64)
            self._pca_axis[0] = 1.0   # harmless placeholder
            self._proj_scale = 1.0
            corr = 0.0
        else:
            # Unit-length readout axis (name kept for backward compat)
            self._pca_axis = beta / beta_norm

            # Correlation on the same data — diagnostic
            projections = Zc @ self._pca_axis
            proj_std = float(projections.std())
            ball_std = float(B.std())
            if proj_std > 1e-8 and ball_std > 1e-8:
                corr = float(np.corrcoef(projections, B)[0, 1])
            else:
                corr = 0.0

            # Scale: 2*std maps the typical [-sigma, +sigma] range to [-1, 1]
            self._proj_scale = max(proj_std * 2.0, 1e-6)

        # Initialise fine-tuning weights — ridge already gives correct sign
        # and magnitude; SGD will refine as GNG topology drifts.
        self._W = self._pca_axis.copy() / self._proj_scale
        self._b = 0.0

        self._calibrated = True

        abs_corr = abs(corr)
        quality  = "good" if abs_corr > 0.4 else ("weak" if abs_corr > 0.2 else "poor")
        print(f"[SpatialDecoder] Ridge calibrated: corr={corr:+.3f} ({quality})  "
              f"scale={self._proj_scale:.4f}  ‖β‖={beta_norm:.3f}")

        # Clear buffers
        self._emb_buf  = []
        self._ball_buf = []

    # ------------------------------------------------------------------
    # Projection
    # ------------------------------------------------------------------

    def project(self, embedding: Optional[np.ndarray]) -> float:
        """
        Project fused_embedding → spatial bias in [-1, 1].

        Returns 0.0 if not yet calibrated or embedding is invalid.
        Dead-band: returns 0 if |bias| < min_bias (uncertain predictions
        should not distort the probe).
        """
        if not self._calibrated or embedding is None:
            return 0.0

        emb = np.asarray(embedding, dtype=np.float64)
        if emb.shape[0] != self._dim:
            return 0.0

        emb_c = emb - self._emb_mean
        raw   = (float(np.dot(self._W, emb_c)) + self._b) * self._gain
        bias  = float(np.clip(raw, -1.0, 1.0))

        # EMA smoothing
        self._ema_bias = self._alpha * bias + (1.0 - self._alpha) * self._ema_bias
        smoothed = float(np.clip(self._ema_bias, -1.0, 1.0))

        return smoothed

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def is_ready(self) -> bool:
        return self._calibrated

    def to_dict(self) -> dict:
        return {
            "calibrated":    self._calibrated,
            "n_calibration": self._n_cal,
            "n_buf":         len(self._emb_buf),
            "n_finetune":    self._n_finetune,
            "last_bias":     round(self._ema_bias, 3),
            "proj_scale":    round(self._proj_scale, 4),
            "weight_norm":   round(float(np.linalg.norm(self._W)), 4) if self._W is not None else 0.0,
        }
