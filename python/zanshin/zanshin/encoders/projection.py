"""
Frozen Geometric Encoders — v3 tabula rasa encoders.

All encoders are fixed at construction time (no gradient, no training, no ONNX).
They project raw sensory input into a stable 128-dimensional latent space using
mathematically grounded transforms.

Visual modalities (retinal, color, optical_flow, saliency):
    Johnson-Lindenstrauss random projection — a fixed Gaussian matrix drawn once
    from a seeded RNG. The seed is derived from the modality name so each modality
    gets its own distinct projection.

Proprioception:
    A fixed random projection of the body-state vector (see ProprioceptiveEncoder).

The generic audio encoder lives in `audio.py` (FrozenSTFTEncoder).
"""

import numpy as np
from typing import Optional


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _modality_seed(modality: str) -> int:
    """Deterministic seed from modality name for reproducible projections."""
    h = 0
    for ch in modality.encode():
        h = (h * 31 + ch) & 0xFFFFFFFF
    return h


# ---------------------------------------------------------------------------
# Visual encoder: frozen random projection
# ---------------------------------------------------------------------------

# Canonical spatial resolution per modality (H, W) before flattening.
# Small enough to keep projection matrix RAM-friendly.
_SPATIAL_RES = {
    "retinal":      (32, 32),   # grayscale
    "color":        (24, 24),   # RGB
    "optical_flow": (20, 20),   # 2-channel (dx, dy)
    "saliency":     (32, 32),   # single-channel
    "dorsal":       (32, 32),   # grayscale
    "ventral":      (32, 32),   # grayscale
}

_SPATIAL_CHANNELS = {
    "retinal":      1,
    "color":        3,
    "optical_flow": 2,
    "saliency":     1,
    "dorsal":       1,
    "ventral":      1,
}


class FrozenProjectionEncoder:
    """
    Johnson-Lindenstrauss random projection for visual modalities.

    Steps per tick:
        1. Resize raw frame to canonical (H, W) resolution.
        2. Flatten to 1-D and L2-normalise.
        3. Multiply by fixed random projection matrix R: shape (input_dim, projection_dim).
        4. L2-normalise the result.

    R is drawn once from N(0, 1/projection_dim) with a seed derived from the
    modality name — identical across runs, never updated.
    """

    def __init__(self, modality: str, projection_dim: int = 128,
                 inject_centroid: bool = False, centroid_gain: float = 22.6):
        self.modality = modality
        self.projection_dim = projection_dim

        h, w = _SPATIAL_RES.get(modality, (32, 32))
        c = _SPATIAL_CHANNELS.get(modality, 1)
        self.target_h = h
        self.target_w = w
        self.channels = c
        self.input_dim = h * w * c

        # Centroid injection: saliency only.  Other modalities ignore the flag.
        # Adds 2 rows to R — the 2D center-of-mass of the saliency map feeds in
        # before L2 normalisation and JL projection.  Output dim unchanged.
        self.inject_centroid = bool(inject_centroid) and modality == "saliency"
        self.centroid_gain   = float(centroid_gain)
        effective_input_dim  = self.input_dim + (2 if self.inject_centroid else 0)

        seed = _modality_seed(modality)
        rng = np.random.default_rng(seed)
        # JL projection: scale by 1/sqrt(projection_dim) preserves distances in expectation
        self.R = rng.standard_normal((effective_input_dim, projection_dim)).astype(np.float32)
        self.R /= np.sqrt(projection_dim)

        # Stateful optical flow (optical_flow / dorsal modalities only).
        # Stores the previous grayscale frame at target resolution (uint8) so
        # Farneback flow can be computed on each call.  None until first frame.
        # Stateful previous frame — used by optical_flow/dorsal (Farneback)
        # and saliency (motion-weighted temporal difference).
        self._prev_gray: Optional[np.ndarray] = None

        # Last preprocessed frame fed into the projection — stored for the
        # 2D visualizer so the user can verify what the encoder actually sees.
        # Shape: (target_h, target_w, channels) float32 [0–1] for still-image
        # modalities; HSV-coloured flow image (target_h, target_w, 3) for flow;
        # grayscale heat-map (H, W, 1) for saliency.
        self.last_encoded_frame: Optional[np.ndarray] = None

        # Last 2D saliency centroid (pre-gain [-1,1] × centroid_gain), set by
        # the saliency branch when inject_centroid is enabled.  None otherwise.
        self._last_centroid: Optional[np.ndarray] = None
        # Probe cache: latest saliency map (H, W, float in [0, 1]) and latent
        # (projection_dim,), populated by the saliency branch of encode().
        # Always populated regardless of inject_centroid — the centroid field
        # is None when injection is off.
        self._last_saliency_map: Optional[np.ndarray] = None
        self._last_latent:       Optional[np.ndarray] = None

    def encode(self, raw_frame: np.ndarray) -> np.ndarray:
        """
        Args:
            raw_frame: numpy array, any of:
                - (H, W)        grayscale
                - (H, W, C)     RGB / multi-channel
                - (C, H, W)     channel-first

        Returns:
            np.ndarray shape (projection_dim,), float32, L2-normalised
        """
        import cv2

        frame = raw_frame.astype(np.float32)

        # Normalise to [0, 1] if uint8
        if frame.max() > 1.5:
            frame = frame / 255.0

        # Ensure (H, W, C)
        if frame.ndim == 2:
            frame = frame[:, :, np.newaxis]
        elif frame.ndim == 3 and frame.shape[0] in (1, 2, 3) and frame.shape[0] < frame.shape[1]:
            # Likely (C, H, W) — transpose
            frame = frame.transpose(1, 2, 0)

        _FLOW_MODALITIES = ("optical_flow", "dorsal")

        if self.modality in _FLOW_MODALITIES:
            # --- Stateful Farneback optical flow at FULL input resolution ---
            #
            # Critical: flow MUST be computed at the full input resolution
            # (typically 128×128 from VideoManager).  If we downscale first to
            # 20×20, a 5px ball moving 3px/frame becomes a 0.47px displacement —
            # sub-pixel, invisible to Farneback.  At 128×128 it's a clear 3px
            # displacement across multiple pixels, well within the algorithm's
            # detection range.
            #
            # After computing the full-res (H_in, W_in, 2) flow field we
            # downsample it to (target_h, target_w, 2) for the JL projection.
            # cv2.COLOR_RGB2GRAY (not BGR2GRAY) because VideoManager outputs RGB.

            frame_u8 = (frame * 255).clip(0, 255).astype(np.uint8)
            if frame_u8.shape[2] >= 3:
                gray_full = cv2.cvtColor(frame_u8, cv2.COLOR_RGB2GRAY)
            else:
                gray_full = frame_u8[:, :, 0]
            # gray_full: (H_in, W_in) at original input resolution (e.g. 128×128)

            if self._prev_gray is not None and self._prev_gray.shape == gray_full.shape:
                flow_full = cv2.calcOpticalFlowFarneback(
                    self._prev_gray, gray_full, None,
                    pyr_scale=0.5, levels=3, winsize=15,
                    iterations=3, poly_n=5, poly_sigma=1.2,
                    flags=0,
                )  # (H_in, W_in, 2) float32 — pixels/frame at input scale

                # Downsample flow to GNG input resolution
                flow_small = cv2.resize(
                    flow_full, (self.target_w, self.target_h),
                    interpolation=cv2.INTER_AREA,
                )  # (target_h, target_w, 2)

                # Normalise: clip at ±16 input-pixels/frame and scale to [-1, 1].
                # A Breakout ball moves ~3-8 px/frame at 128px width; 16px is a
                # safe upper bound that keeps the signal dense without saturation.
                max_disp = 16.0
                flow_small = np.clip(flow_small, -max_disp, max_disp) / max_disp

                # Full-res HSV visualisation — shows actual motion field
                mag_full, ang_full = cv2.cartToPolar(flow_full[..., 0], flow_full[..., 1])
                hsv_viz = np.zeros((*gray_full.shape, 3), dtype=np.uint8)
                hsv_viz[..., 0] = (ang_full * 180 / np.pi / 2).astype(np.uint8)
                hsv_viz[..., 1] = 255
                # Scale magnitude: 16px → full brightness, <1px → dim
                hsv_viz[..., 2] = np.clip(mag_full / max_disp * 255, 0, 255).astype(np.uint8)
                rgb_viz = cv2.cvtColor(hsv_viz, cv2.COLOR_HSV2RGB).astype(np.float32) / 255.0
                self.last_encoded_frame = rgb_viz  # (H_in, W_in, 3) full-res viz
            else:
                flow_small = np.zeros((self.target_h, self.target_w, 2), dtype=np.float32)
                self.last_encoded_frame = np.zeros((*gray_full.shape, 3), dtype=np.float32)

            self._prev_gray = gray_full
            frame_resized = flow_small  # (target_h, target_w, 2)

        elif self.modality == "saliency":
            # --- Motion-weighted spatial saliency (stateful) ---
            #
            # Motivation: raw pixel projection (same as retinal) adds no new
            # information.  Saliency should answer "what deserves attention?" —
            # the answer in a dynamic scene is always the MOVING high-contrast
            # region (the ball in Breakout, the paddle when it moves).
            #
            # Formula per pixel:
            #   motion  = |current_gray - prev_gray|          (temporal difference)
            #   edges   = |Laplacian(current_gray)|           (spatial contrast)
            #   salient = motion × (1 + 0.5 × edges_norm)    (motion-weighted)
            #
            # At full input resolution → log-compressed → downsampled → project.
            # GNG nodes represent "attention events": ball or paddle crossing a
            # region, brick destruction flash — precisely the game-critical moments.
            #
            # Complementary roles:
            #   Retinal  → where everything IS (full scene, always)
            #   Dorsal   → what DIRECTION things are moving
            #   Saliency → what deserves attention RIGHT NOW (moving edges)

            frame_u8 = (frame * 255).clip(0, 255).astype(np.uint8)
            if frame_u8.shape[2] >= 3:
                gray_full = cv2.cvtColor(frame_u8, cv2.COLOR_RGB2GRAY)
            else:
                gray_full = frame_u8[:, :, 0]

            gray_f = gray_full.astype(np.float32) / 255.0  # [0, 1]

            # Spatial contrast: absolute Laplacian (edge strength per pixel)
            lap = cv2.Laplacian(gray_full, cv2.CV_32F)
            edges = np.abs(lap) / 255.0
            edges_norm = edges / (edges.max() + 1e-6)  # [0, 1]

            if self._prev_gray is not None and self._prev_gray.shape == gray_full.shape:
                prev_f = self._prev_gray.astype(np.float32) / 255.0
                motion = np.abs(gray_f - prev_f)                        # [0, 1]
                # Weight motion by local edge strength: amplifies moving edges
                salient = motion * (1.0 + 0.5 * edges_norm)             # [0, 1.5]
                # Log-compress to prevent ball spikes drowning out weaker signals
                salient = np.log1p(salient * 9.0) / np.log1p(9.0)      # [0, 1]
            else:
                # First frame: no previous → spatial edges only (no motion)
                salient = edges_norm * 0.3

            self._prev_gray = gray_full

            # Downsample to target resolution for GNG
            sal_small = cv2.resize(salient, (self.target_w, self.target_h),
                                   interpolation=cv2.INTER_AREA)

            frame_resized = sal_small[:, :, np.newaxis]  # (H, W, 1)

            # Probe cache (always on) — SpatialProbeWidget reads this per tick.
            self._last_saliency_map = sal_small.astype(np.float32, copy=True)

            # Cache 2D center-of-mass for optional injection below.  Encodes
            # "where is the moving high-contrast region" as a spatial anchor —
            # the axis the raw JL projection destroys.
            #
            # When saliency mass ≈ 0 (duplicate frame read, ball off-screen,
            # truly static scene) the centroid is undefined.  Holding the
            # previous centroid instead of collapsing to origin prevents a
            # spurious "null-state" GNG node from being baked at (0, 0) when
            # the EPM tick rate exceeds video fps.
            if self.inject_centroid:
                mass = float(sal_small.sum())
                if mass > 1e-9 and self.target_w > 1 and self.target_h > 1:
                    y_idx, x_idx = np.mgrid[0:self.target_h, 0:self.target_w]
                    cx = float((x_idx * sal_small).sum() / mass)
                    cy = float((y_idx * sal_small).sum() / mass)
                    x_n = 2.0 * cx / (self.target_w - 1) - 1.0
                    y_n = 2.0 * cy / (self.target_h - 1) - 1.0
                    self._last_centroid = np.array(
                        [self.centroid_gain * x_n, self.centroid_gain * y_n],
                        dtype=np.float32,
                    )
                # else: keep self._last_centroid from previous tick unchanged
                # (initial None carries through to the flatten branch, which
                # skips concat — identical to pre-injection behavior).
            else:
                self._last_centroid = None

            # Visualisation: green-tinted heat-map at full resolution
            viz = np.zeros((*gray_full.shape, 3), dtype=np.float32)
            viz[..., 1] = salient      # green channel = saliency magnitude
            viz[..., 2] = gray_f * 0.2  # faint blue = scene context
            self.last_encoded_frame = viz

        else:
            # --- Still-image modalities (retinal, color, ventral, …) ---

            # Channel mismatch: resolve to target channel count before resize
            if frame.shape[2] != self.channels:
                if self.channels == 1:
                    frame = np.mean(frame, axis=2, keepdims=True)
                elif self.channels == 3 and frame.shape[2] == 1:
                    frame = np.repeat(frame, 3, axis=2)
                elif frame.shape[2] > self.channels:
                    frame = frame[:, :, :self.channels]
                else:
                    pad = np.zeros((*frame.shape[:2], self.channels - frame.shape[2]),
                                   dtype=frame.dtype)
                    frame = np.concatenate([frame, pad], axis=2)

            # Resize
            if frame.shape[0] != self.target_h or frame.shape[1] != self.target_w:
                frame_resized = cv2.resize(frame, (self.target_w, self.target_h),
                                           interpolation=cv2.INTER_AREA)
                if frame_resized.ndim == 2:
                    frame_resized = frame_resized[:, :, np.newaxis]
            else:
                frame_resized = frame

            self.last_encoded_frame = frame_resized.copy()

        # Flatten and append centroid channels BEFORE L2-normalise.
        # Pre-JL injection keeps output dim at projection_dim (unchanged).
        flat = frame_resized.flatten()
        if self.inject_centroid and self._last_centroid is not None:
            flat = np.concatenate([flat, self._last_centroid])

        norm = np.linalg.norm(flat)
        if norm > 1e-6:
            flat = flat / norm

        # Project
        projected = flat @ self.R          # shape (projection_dim,)
        p_norm = np.linalg.norm(projected)
        if p_norm > 1e-6:
            projected = projected / p_norm

        projected = projected.astype(np.float32)
        if self.modality == "saliency":
            self._last_latent = projected.copy()
        return projected

    @property
    def output_dim(self) -> int:
        return self.projection_dim

    def get_spatial_probe(self) -> Optional[dict]:
        """Return the most recent saliency-centroid probe payload, or None if
        this encoder has no spatial data (non-saliency modalities, or saliency
        before its first encode)."""
        if self.modality != "saliency" or self._last_saliency_map is None:
            return None
        return {
            "saliency_map":     self._last_saliency_map,
            "centroid":         self._last_centroid,   # may be None
            "latent":           self._last_latent,
            "inject_centroid":  self.inject_centroid,
            "centroid_gain":    self.centroid_gain,
            "target_h":         self.target_h,
            "target_w":         self.target_w,
        }

    def __repr__(self):
        return (f"FrozenProjectionEncoder(modality={self.modality!r}, "
                f"input_dim={self.input_dim}, projection_dim={self.projection_dim})")


# ---------------------------------------------------------------------------
# (Audio encoder moved to audio.py — FrozenSTFTEncoder)
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Proprioceptive encoder: frozen random projection over body-state vector
# ---------------------------------------------------------------------------

# Body state vector layout (4 dimensions):
#   [0] pos_norm       — paddle x position, normalised to [-1, 1]
#   [1] vel_norm       — paddle velocity, normalised to [-1, 1]
#   [2] accel_norm     — velocity change this tick, normalised to [-1, 1]
#   [3] efference_error — commanded vs actual velocity diff, normalised to [-1, 1]
_PROPRIO_INPUT_DIM = 4


class ProprioceptiveEncoder:
    """
    Frozen Johnson-Lindenstrauss projection for proprioceptive body state.

    Maps a 4-dimensional body state vector (paddle position, velocity,
    acceleration, efference error) to a stable 128D latent representation
    using the same seeded JL pattern as the visual encoders.

    The projection matrix is fixed at construction from a seed derived
    from the string "proprioceptive" — identical across runs, never updated.
    This is the body-model counterpart to FrozenProjectionEncoder — it gives
    the GNG a consistent coordinate system for motor-state learning without
    any hyperparameter tuning.

    Input contract:
        state_vector — array-like of length >= 4, floats already in [-1, 1]:
            [pos_norm, vel_norm, accel_norm, efference_error]
            Extra dimensions are ignored; missing dimensions are zero-padded.
    """

    def __init__(self, projection_dim: int = 128,
                 input_dim: int = _PROPRIO_INPUT_DIM):
        self.modality      = "proprioceptive"
        self.projection_dim = projection_dim
        self.input_dim      = input_dim

        seed = _modality_seed("proprioceptive")
        rng  = np.random.default_rng(seed)
        # JL projection: scale preserves pairwise distances in expectation
        self.R = rng.standard_normal(
            (input_dim, projection_dim)).astype(np.float32)
        self.R /= np.sqrt(projection_dim)

        # Last encoded state for diagnostics
        self.last_encoded_state: Optional[np.ndarray] = None

    def encode(self, state_vector) -> np.ndarray:
        """
        Args:
            state_vector: array-like of floats, at least 4 elements.

        Returns:
            np.ndarray shape (projection_dim,), float32, L2-normalised.
        """
        sv = np.asarray(state_vector, dtype=np.float32).flatten()

        # Pad or trim to expected input_dim
        if len(sv) < self.input_dim:
            sv = np.pad(sv, (0, self.input_dim - len(sv)))
        elif len(sv) > self.input_dim:
            sv = sv[: self.input_dim]

        self.last_encoded_state = sv.copy()

        # Project
        projected = sv @ self.R          # shape (projection_dim,)
        p_norm = np.linalg.norm(projected)
        if p_norm > 1e-6:
            projected = projected / p_norm

        return projected.astype(np.float32)

    @property
    def output_dim(self) -> int:
        return self.projection_dim

    def __repr__(self):
        return (f"ProprioceptiveEncoder(input_dim={self.input_dim}, "
                f"projection_dim={self.projection_dim})")


