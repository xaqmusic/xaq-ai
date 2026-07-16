import numpy as np
from typing import Optional
from .base import GameAdapter


class BreakoutAdapter(GameAdapter):
    """
    GameAdapter for the Ogma Break-Pong benchmark.

    Body: Player 1 paddle (bottom).
    Body state: [paddle_x_norm, paddle_vx_norm]
      - paddle_x_norm: paddle centre x, normalised to [-1, 1] over screen width
      - paddle_vx_norm: estimated paddle velocity, normalised to [-1, 1]

    The game deliberately withholds ball position from the per-tick telemetry
    ('integrity_check: strict').  Ball position is the exclusive domain of the
    visual/audio EPMs — this adapter only touches body (paddle) state.
    """

    MAX_VELOCITY = 15.0   # max velocity command sent to the game
    VX_SCALE    = 20.0    # multiplier for per-tick delta when estimating velocity

    # Structural Forcing Function (calibration-only motor seed)
    FORCING_DELTA_SCALE = 200.0   # divisor for (ball_x - paddle_x) → [-1, 1]
    FORCING_EMA_ALPHA   = 0.15    # EMA mixing weight for new sample
    FORCING_GATE        = 0.10    # |EMA| must exceed this to take effect

    def __init__(self, screen_w: int = 640):
        self.screen_w = screen_w
        self._prev_px_norm: float = 0.0
        self._has_prev: bool = False
        self._bilateral_ema: float = 0.0

    @property
    def body_schema(self):
        return ["paddle_x", "paddle_vx"]

    # ------------------------------------------------------------------
    def encode_body_state(self, raw_state: dict) -> np.ndarray:
        px = raw_state.get("p1_x")
        if px is None:
            return None  # caller skips if no body info in this packet

        px_norm = float(px) / self.screen_w * 2.0 - 1.0  # [-1, 1]

        if self._has_prev:
            pvx_norm = np.clip(
                (px_norm - self._prev_px_norm) * self.VX_SCALE, -1.0, 1.0
            )
        else:
            pvx_norm = 0.0
            self._has_prev = True

        self._prev_px_norm = px_norm
        return np.array([px_norm, pvx_norm], dtype=np.float32)

    # ------------------------------------------------------------------
    def decode_action(self, body_error: np.ndarray) -> dict:
        """
        body_error[0]: normalised position error (desired_x - current_x).
        Scale to raw velocity command; caller applies stiffness gain first.
        """
        velocity = float(body_error[0]) * self.MAX_VELOCITY
        velocity = float(np.clip(velocity, -self.MAX_VELOCITY, self.MAX_VELOCITY))
        return {"player": 1, "velocity": velocity}

    # ------------------------------------------------------------------
    def predict_next_body_state(
        self, body_state: np.ndarray, action: dict, dt: float = 0.033
    ) -> np.ndarray:
        v = action.get("velocity", 0.0)
        v_norm = np.clip(v / self.MAX_VELOCITY, -1.0, 1.0)
        # Simple kinematic step: x_next = x + v * dt * empirical_scale
        # The scale factor (30) converts normalised velocity to normalised
        # position change per dt — calibrated to the game's pixel rate.
        px_next = np.clip(body_state[0] + v_norm * dt * 30.0, -1.0, 1.0)
        return np.array([px_next, v_norm], dtype=np.float32)

    # ------------------------------------------------------------------
    def parse_events(self, raw_state: dict) -> dict:
        event = raw_state.get("event", "")
        hit  = (event == "self_paddle_hit")
        miss = (event == "paddle_miss")
        return {
            "hit":     hit,
            "miss":    miss,
            "reward":  1.0 if hit  else 0.0,
            "penalty": 1.0 if miss else 0.0,
        }

    # ------------------------------------------------------------------
    def extract_world_target(self, raw_state: dict) -> Optional[dict]:
        """
        Pong-specific calibration & motor-seed signals derived from raw game
        state. Two outputs:

        - calibration_target: ball_x normalised to [-1, 1]. Fed to the
          SpatialDecoder during its supervised PCA-axis warm-up.
        - forcing_bias: EMA-smoothed (ball_x - paddle_x) ∈ [-1, 1], gated by
          |EMA| > FORCING_GATE. Used as a fallback motor seed during the
          SpatialDecoder warm-up window only — the runner reads it when the
          decoder is not yet calibrated. This is the "Structural Forcing
          Function" described in docs/v3_architecture_status.md §5.
        - forcing_active: True when forcing_bias is non-zero (gate passed).

        Returns None if the raw state lacks ball/paddle telemetry (e.g. when
        telemetry is disabled or the packet is incomplete).
        """
        ball = raw_state.get("ball")
        paddle_x = raw_state.get("p1_x")
        if ball is None or paddle_x is None:
            return None
        ball_x = ball[0]
        if ball_x == 0.0 and ball[1] == 0.0:
            # Telemetry-disabled mode broadcasts (0,0) as a sentinel; do not
            # treat zeros as a real ball position.
            return None

        ball_x_norm = (float(ball_x) / self.screen_w) * 2.0 - 1.0

        delta_x = float(ball_x) - float(paddle_x)
        norm_delta = float(np.clip(delta_x / self.FORCING_DELTA_SCALE, -1.0, 1.0))
        self._bilateral_ema = (
            self.FORCING_EMA_ALPHA * norm_delta
            + (1.0 - self.FORCING_EMA_ALPHA) * self._bilateral_ema
        )
        forcing_active = abs(self._bilateral_ema) > self.FORCING_GATE
        forcing_bias = float(self._bilateral_ema) if forcing_active else 0.0

        return {
            "calibration_target": float(ball_x_norm),
            "forcing_bias":       forcing_bias,
            "forcing_active":     forcing_active,
        }
