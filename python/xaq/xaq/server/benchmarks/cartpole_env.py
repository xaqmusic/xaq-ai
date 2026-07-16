"""
cartpole_env.py — Pure-numpy CartPole benchmark for AMI-Ogma v3.

Implements the classic Sutton-Barto / Gymnasium CartPole-v1 physics in pure
numpy + cv2, mirroring the HeadlessBreakout surface so the existing
`run_e2e_v3.py` runner can drive it polymorphically:

    tick() -> rendered RGB frame
    state() -> dict telemetry
    event_deltas() -> {hit, miss, brick, wall_stuck, whisker_bump}
    apply_force(accel) -> queue motor command for next tick
    set_brain_mode(), set_telemetry_disabled(), etc.

CartPole's "hit" event fires once per sustained upright streak crossing
UPRIGHT_THRESHOLD_TICKS (positive reinforcement for the neurochemical system).
"miss" fires on each pole-fall (auto-respawn keeps the loop alive without the
runner needing to know about episodes).
"""
import math
from collections import deque
import numpy as np
import cv2


class _CartPoleSim:
    """Pure-numpy cart-pole physics + cv2 rendering. Standard Gymnasium
    CartPole-v1 dynamics (Euler integration, identical constants)."""

    GRAVITY      = 9.8
    MASS_CART    = 1.0
    MASS_POLE    = 0.1
    POLE_LEN     = 0.5      # actually half-length to pole's CoM
    FORCE_MAG    = 10.0
    TAU          = 0.02     # seconds between physics steps (50 Hz)
    THETA_FAIL   = 12 * 2 * math.pi / 360   # ~0.2094 rad (12°)
    X_FAIL       = 2.4

    def __init__(self, width: int = 320, height: int = 240, seed: int = 0):
        self.width  = int(width)
        self.height = int(height)
        self._rng = np.random.default_rng(seed)
        self.episode_count = 0
        self.reset()

    def reset(self) -> None:
        self.x         = float(self._rng.uniform(-0.05, 0.05))
        self.x_dot     = float(self._rng.uniform(-0.05, 0.05))
        self.theta     = float(self._rng.uniform(-0.05, 0.05))
        self.theta_dot = float(self._rng.uniform(-0.05, 0.05))
        self.fallen    = False
        self.steps     = 0
        self.upright_streak = 0

    def respawn(self) -> None:
        """Reset state but increment the cumulative episode counter."""
        self.episode_count += 1
        self.x         = float(self._rng.uniform(-0.05, 0.05))
        self.x_dot     = float(self._rng.uniform(-0.05, 0.05))
        self.theta     = float(self._rng.uniform(-0.05, 0.05))
        self.theta_dot = float(self._rng.uniform(-0.05, 0.05))
        self.fallen    = False
        self.upright_streak = 0

    def step(self, force_norm: float) -> None:
        """force_norm: continuous control signal in [-1, 1]; scaled to ±FORCE_MAG."""
        force = float(np.clip(force_norm, -1.0, 1.0)) * self.FORCE_MAG

        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        total_mass     = self.MASS_CART + self.MASS_POLE
        pole_mass_len  = self.MASS_POLE * self.POLE_LEN

        temp = (force + pole_mass_len * self.theta_dot ** 2 * sin_t) / total_mass
        theta_acc = (self.GRAVITY * sin_t - cos_t * temp) / (
            self.POLE_LEN * (4.0/3.0 - self.MASS_POLE * cos_t ** 2 / total_mass)
        )
        x_acc = temp - pole_mass_len * theta_acc * cos_t / total_mass

        self.x         += self.TAU * self.x_dot
        self.x_dot     += self.TAU * x_acc
        self.theta     += self.TAU * self.theta_dot
        self.theta_dot += self.TAU * theta_acc
        self.steps     += 1

        if abs(self.x) > self.X_FAIL or abs(self.theta) > self.THETA_FAIL:
            self.fallen = True
            self.upright_streak = 0
        else:
            self.upright_streak += 1

    def render(self) -> np.ndarray:
        """Render scene as BGR uint8 frame (cv2-compatible). Same shape as
        the OgmaBreakout output so downstream encoders see identical input
        format."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        track_y = int(self.height * 0.70)
        cv2.line(frame, (0, track_y), (self.width, track_y), (60, 60, 60), 1)

        cart_w = max(8, int(self.width * 0.10))
        cart_h = max(4, int(self.height * 0.05))
        cart_cx = int(self.width * 0.5 + (self.x / self.X_FAIL) * (self.width * 0.5))
        cv2.rectangle(
            frame,
            (cart_cx - cart_w // 2, track_y - cart_h),
            (cart_cx + cart_w // 2, track_y),
            (200, 200, 200),
            -1,
        )

        pole_render_len = int(self.height * 0.30)
        tip_x = cart_cx + int(pole_render_len * math.sin(self.theta))
        tip_y = (track_y - cart_h) - int(pole_render_len * math.cos(self.theta))
        cv2.line(
            frame,
            (cart_cx, track_y - cart_h),
            (tip_x, tip_y),
            (180, 90, 255),
            3,
        )
        return frame


class CartPoleEnv:
    """HeadlessBreakout-compatible wrapper around _CartPoleSim. Drop-in
    replacement for the runner's `game` object.

    Reward-cadence is self-tuning: the cart-pole body is an agent whose
    "hit" pulse rate adapts to its own recent streak ecology. Early training
    streaks are short, so the threshold stays small and positive reinforcement
    fires often; as the agent improves, the threshold grows so pulses remain
    ~3 per typical successful streak. No central scheduler sets this — only
    the body's memory of its own history."""

    FORCE_INPUT_SCALE       = 4.0   # ActionDecoder emits force in [-4, +4]
    _THRESHOLD_MIN          = 5
    _THRESHOLD_MAX          = 200
    _STREAK_HISTORY_LEN     = 20
    _PULSES_PER_STREAK      = 3     # Target: ~3 positive pulses per successful streak

    def __init__(self, width: int = 320, height: int = 240, seed: int = 0):
        self._game = _CartPoleSim(width=width, height=height, seed=seed)
        self._last_force_norm: float = 0.0
        self._telemetry_disabled: bool = False
        self._prev_episode: int = 0
        # Streak ecology: seed with [1] so mean is defined before any falls.
        self._streak_history: deque = deque([1], maxlen=self._STREAK_HISTORY_LEN)
        self._threshold: int = self._THRESHOLD_MIN
        self._prev_upright_streak: int = 0

    # ---- Runner-facing interface (HeadlessBreakout shape) -----------------
    def tick(self) -> np.ndarray:
        self._game.step(self._last_force_norm)
        # Record the just-ended streak length before respawn wipes it,
        # then re-tune the body's pulse threshold from its own history.
        if self._game.fallen:
            if self._prev_upright_streak > 0:
                self._streak_history.append(self._prev_upright_streak)
                mean_streak = sum(self._streak_history) / len(self._streak_history)
                target = int(mean_streak // self._PULSES_PER_STREAK)
                self._threshold = max(self._THRESHOLD_MIN,
                                      min(self._THRESHOLD_MAX, target))
            self._game.respawn()
        self._prev_upright_streak = self._game.upright_streak
        return self._game.render()

    def state(self) -> dict:
        g = self._game
        if self._telemetry_disabled:
            return {
                "cart_x":         0.0,
                "cart_vx":        0.0,
                "theta":          0.0,
                "theta_dot":      0.0,
                "fallen":         False,
                "episode":        0,
                "upright_streak": 0,
            }
        return {
            "cart_x":         g.x,
            "cart_vx":        g.x_dot,
            "theta":          g.theta,
            "theta_dot":      g.theta_dot,
            "fallen":         g.fallen,
            "episode":        g.episode_count,
            "upright_streak": g.upright_streak,
            "pulse_threshold": self._threshold,
        }

    def event_deltas(self) -> dict:
        """hit = sustained-upright pulse fires every UPRIGHT_THRESHOLD_TICKS
        physics steps while the pole stays up (so balance duration produces
        a proportional stream of reinforcement, not a single one-shot event).
        miss = pole fell (episode incremented)."""
        g = self._game
        miss = g.episode_count > self._prev_episode
        hit  = (
            g.upright_streak > 0
            and g.upright_streak % self._threshold == 0
        )
        self._prev_episode = g.episode_count
        return {
            "hit":          hit,
            "miss":         miss,
            "brick":        False,
            "wall_stuck":   False,
            "whisker_bump": False,
        }

    def set_brain_mode(self) -> None:
        # CartPole is always brain-controlled; no auto-play counterpart.
        pass

    def set_velocity(self, v: float) -> None:
        # Compatibility: treat raw velocity command as a normalised force.
        self._last_force_norm = float(np.clip(v / 10.0, -1.0, 1.0))

    def apply_force(self, accel: float) -> None:
        self._last_force_norm = float(np.clip(accel / self.FORCE_INPUT_SCALE, -1.0, 1.0))

    def set_telemetry_disabled(self, disabled: bool) -> None:
        self._telemetry_disabled = bool(disabled)

    # ---- Breakout-only modes; no-op shims for runner compatibility --------
    def set_rally_mode(self) -> None: pass
    def set_observer_mode(self) -> None: pass
    def set_mode(self, mode: str) -> None: pass
