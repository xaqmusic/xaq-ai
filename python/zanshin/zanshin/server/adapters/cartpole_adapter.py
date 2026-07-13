"""
cartpole_adapter.py — GameAdapter binding for the CartPole benchmark.

Body schema is richer than Gymnasium's raw 4D state: it adds derived signals
(angular + lateral accel, efference-copy error) plus proximity-to-failure
"whiskers" on both the tilt and the track-edge limits. These whiskers give
the neurochemical aversive channel a continuous pre-fall signal, which is the
CartPole analog of the paddle's tactile whiskers in the Pong harness.

The adapter is expected to cache per-tick history (prev theta_dot, prev cart_vx,
prev commanded force) needed to compute accel and efference diffs.
"""
import math
from collections import deque
from typing import Optional, List
import numpy as np

from .base import GameAdapter


class CartPoleAdapter(GameAdapter):
    """
    Body: cart-on-track + pole.

    Body state (10D, all clipped to [-1, 1]):
      0  theta_norm        theta / THETA_FAIL              (tilt)
      1  theta_dot_norm    theta_dot / ANG_VEL_SCALE       (tilt rate)
      2  theta_ddot_norm   (theta_dot - prev_theta_dot) / ANG_ACC_SCALE
      3  cart_x_norm       cart_x / X_FAIL                 (lateral pos)
      4  cart_vx_norm      cart_vx / VEL_SCALE             (lateral vel)
      5  cart_ax_norm      (cart_vx - prev_cart_vx) / ACC_SCALE
      6  efference_err     commanded_force_norm - observed_cart_accel_norm
      7  tilt_whisker_L    max(0, -theta/THETA_FAIL - TILT_WHISKER_ONSET) × 2
      8  tilt_whisker_R    max(0,  theta/THETA_FAIL - TILT_WHISKER_ONSET) × 2
      9  edge_whisker      max(0, |cart_x|/X_FAIL - EDGE_WHISKER_ONSET)  × 2
    """

    THETA_FAIL    = 12 * 2 * math.pi / 360   # match _CartPoleSim
    X_FAIL        = 2.4
    VEL_SCALE     = 4.0
    ANG_VEL_SCALE = 4.0
    ANG_ACC_SCALE = 8.0
    ACC_SCALE     = 8.0
    FORCE_INPUT   = 4.0    # ActionDecoder emits accel ~[-4, +4]

    # Whisker onset fractions — signal starts firing when state crosses
    # this fraction of the failure boundary. Tuned for CartPole's narrow
    # operating range: 0.5 = half-way to fail.
    TILT_WHISKER_ONSET = 0.5
    EDGE_WHISKER_ONSET = 0.5

    def __init__(self):
        self._prev_theta_dot: float = 0.0
        self._prev_cart_vx:   float = 0.0
        self._last_force_norm: float = 0.0

        # Ecological-agency self-tuning: the cart-pole body is its own agent
        # and its whisker onsets are tuned by its own memory of where danger
        # lives — the fractions of the failure boundary that preceded past
        # falls. Seeded with the prior fixed values until 10 falls observed.
        self._tilt_onset: float = self.TILT_WHISKER_ONSET
        self._edge_onset: float = self.EDGE_WHISKER_ONSET
        self._theta_fall_history: deque = deque(maxlen=50)
        self._edge_fall_history:  deque = deque(maxlen=50)
        self._was_fallen: bool = False

    # ------------------------------------------------------------------
    @property
    def body_schema(self) -> List[str]:
        return [
            "theta", "theta_dot", "theta_ddot",
            "cart_x", "cart_vx", "cart_ax",
            "efference_err",
            "tilt_whisker_L", "tilt_whisker_R",
            "edge_whisker",
        ]

    @property
    def proprio_input_dim(self) -> int:
        return 10

    @property
    def disable_wall_reflex(self) -> bool:
        return True    # cart has its own track-edge failure logic

    # ------------------------------------------------------------------
    def encode_body_state(self, raw_state: dict) -> Optional[np.ndarray]:
        """Canonical 4D state (for consumers that speak the GameAdapter
        contract but don't want the enriched vector). Not used by the v3
        runner — it calls runner_proprio() instead."""
        theta = raw_state.get("theta")
        if theta is None:
            return None
        return np.array([
            float(np.clip(theta / self.THETA_FAIL, -1.0, 1.0)),
            float(np.clip(raw_state.get("theta_dot", 0.0) / self.ANG_VEL_SCALE, -1.0, 1.0)),
            float(np.clip(raw_state.get("cart_x",    0.0) / self.X_FAIL,        -1.0, 1.0)),
            float(np.clip(raw_state.get("cart_vx",   0.0) / self.VEL_SCALE,     -1.0, 1.0)),
        ], dtype=np.float32)

    # ------------------------------------------------------------------
    def decode_action(self, body_error: np.ndarray) -> dict:
        signal = float(body_error[0]) if len(body_error) > 0 else 0.0
        force = float(np.clip(signal, -1.0, 1.0)) * self.FORCE_INPUT
        return {"force": force}

    # ------------------------------------------------------------------
    def predict_next_body_state(
        self, body_state: np.ndarray, action: dict, dt: float = 0.02
    ) -> np.ndarray:
        """Crude efference-copy forward model (4D canonical state). Not used
        by the v3 runner (which carries its own predictor)."""
        theta_n, theta_dot_n, cart_x_n, cart_vx_n = body_state[:4]
        force = action.get("force", 0.0) / self.FORCE_INPUT
        next_cart_vx = float(np.clip(cart_vx_n + 0.05 * force, -1.0, 1.0))
        next_cart_x  = float(np.clip(cart_x_n  + dt * cart_vx_n, -1.0, 1.0))
        next_theta     = float(np.clip(theta_n + dt * theta_dot_n, -1.0, 1.0))
        next_theta_dot = theta_dot_n
        return np.array([next_theta, next_theta_dot, next_cart_x, next_cart_vx], dtype=np.float32)

    # ------------------------------------------------------------------
    def parse_events(self, raw_state: dict) -> dict:
        return {
            "hit":     bool(raw_state.get("hit",  False)),
            "miss":    bool(raw_state.get("miss", False)),
            "reward":  1.0 if raw_state.get("hit",  False) else 0.0,
            "penalty": 1.0 if raw_state.get("miss", False) else 0.0,
        }

    # ------------------------------------------------------------------
    def extract_world_target(self, raw_state: dict) -> Optional[dict]:
        """No oracle / forcing function — tabula rasa."""
        return None

    # ------------------------------------------------------------------
    def runner_proprio(self, raw_state: dict, last_commanded: float) -> Optional[list]:
        """10D body-state vector for the v3 runner. See class docstring for
        channel layout. last_commanded is the force the runner emitted on
        the *previous* tick, used to compute efference-copy error against
        the now-observed cart acceleration."""
        theta = raw_state.get("theta")
        if theta is None:
            return None

        theta_dot = raw_state.get("theta_dot", 0.0)
        cart_x    = raw_state.get("cart_x",    0.0)
        cart_vx   = raw_state.get("cart_vx",   0.0)

        # Derived accels from cached history
        theta_ddot = (theta_dot - self._prev_theta_dot) / 1.0   # per-tick delta
        cart_ax    = (cart_vx   - self._prev_cart_vx)   / 1.0
        self._prev_theta_dot = float(theta_dot)
        self._prev_cart_vx   = float(cart_vx)

        # Efference copy: commanded force direction vs observed lateral accel
        cmd_norm = float(np.clip(self._last_force_norm / self.FORCE_INPUT, -1.0, 1.0))
        cart_ax_norm_for_eff = float(np.clip(cart_ax / self.ACC_SCALE, -1.0, 1.0))
        efference_err = float(np.clip(cmd_norm - cart_ax_norm_for_eff, -1.0, 1.0))
        self._last_force_norm = float(last_commanded)

        # Whiskers: proximity-to-failure signals, asymmetric on tilt.
        theta_frac = theta / self.THETA_FAIL
        edge_frac  = abs(cart_x) / self.X_FAIL

        # Self-tune onsets from the body's own fall distribution — quantile
        # of the |state|/failure fraction observed on the tick before a fall.
        fallen_now = bool(raw_state.get("fallen", False))
        if fallen_now and not self._was_fallen:
            self._theta_fall_history.append(abs(theta_frac))
            self._edge_fall_history.append(edge_frac)
            if len(self._theta_fall_history) >= 10:
                self._tilt_onset = float(np.clip(
                    np.quantile(self._theta_fall_history, 0.30), 0.30, 0.70))
            if len(self._edge_fall_history) >= 10:
                self._edge_onset = float(np.clip(
                    np.quantile(self._edge_fall_history, 0.30), 0.30, 0.70))
        self._was_fallen = fallen_now

        tilt_L = max(0.0, -theta_frac - self._tilt_onset) * 2.0
        tilt_R = max(0.0,  theta_frac - self._tilt_onset) * 2.0
        edge_w = max(0.0,  edge_frac  - self._edge_onset) * 2.0

        return [
            float(np.clip(theta              / self.THETA_FAIL,    -1.0, 1.0)),
            float(np.clip(theta_dot          / self.ANG_VEL_SCALE, -1.0, 1.0)),
            float(np.clip(theta_ddot         / self.ANG_ACC_SCALE, -1.0, 1.0)),
            float(np.clip(cart_x             / self.X_FAIL,        -1.0, 1.0)),
            float(np.clip(cart_vx            / self.VEL_SCALE,     -1.0, 1.0)),
            float(np.clip(cart_ax            / self.ACC_SCALE,     -1.0, 1.0)),
            efference_err,
            float(np.clip(tilt_L, 0.0, 1.0)),
            float(np.clip(tilt_R, 0.0, 1.0)),
            float(np.clip(edge_w, 0.0, 1.0)),
        ]
