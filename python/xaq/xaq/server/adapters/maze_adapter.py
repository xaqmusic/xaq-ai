"""
maze_adapter.py — GameAdapter binding for the 5×5 maze benchmark.

Body is a 2D rodent with 8-compass whiskers. The cognitive stack (EFE,
NodePathBlame, ActionTransitionModel) remains 1D-action; the adapter maps
the scalar force signal into a turn command (see maze_env._MazeSim.step).

The proprio schema intentionally exposes both position (pos_x, pos_y) and
heading (sin, cos): position gives kinesthetic "where my body is" — the
same self-modeling sense CartPole uses for cart_x — while heading lets
the GNG disambiguate "facing a wall at (2,1)" from "facing the corridor
at (2,1)," which matter enormously for place-cell formation.
"""
import math
from typing import List, Optional
import numpy as np

from .base import GameAdapter


class MazeAdapter(GameAdapter):
    """
    Body: 2D disc with heading + 8-ray whiskers.

    Body state (12D, all clipped to [-1, 1]):
      0  pos_x_norm        maze_x / maze_w × 2 − 1
      1  pos_y_norm        maze_y / maze_h × 2 − 1
      2  heading_sin       sin(theta)
      3  heading_cos       cos(theta)
      4..11  wall_whisker_{N,NE,E,SE,S,SW,W,NW}   ∈ [0, 1], 1 = touching
    """

    FORCE_INPUT   = 4.0    # ActionDecoder scalar scale (±4)

    def __init__(self):
        pass

    # ------------------------------------------------------------------
    @property
    def body_schema(self) -> List[str]:
        return [
            "pos_x", "pos_y",
            "heading_sin", "heading_cos",
            # 8 wall-proximity whiskers — 1 = touching, 0 = clear. Aversive
            # (drains serotonin when any ray is near a wall).
            "wall_whisker_N",  "wall_whisker_NE",
            "wall_whisker_E",  "wall_whisker_SE",
            "wall_whisker_S",  "wall_whisker_SW",
            "wall_whisker_W",  "wall_whisker_NW",
            # 8 directional scent whiskers — olfactory gradient to goal,
            # occluded by walls. Appetitive (pulses dopamine on rising scent).
            "scent_whisker_N",  "scent_whisker_NE",
            "scent_whisker_E",  "scent_whisker_SE",
            "scent_whisker_S",  "scent_whisker_SW",
            "scent_whisker_W",  "scent_whisker_NW",
            "hunger",          # interoception: 1.0 - energy; drives serotonin aversion
            "pheromone",       # trail self-scent: recently-visited places smell stale
        ]

    @property
    def proprio_input_dim(self) -> int:
        return 22

    @property
    def disable_wall_reflex(self) -> bool:
        return True   # maze body is not a 1D paddle

    # ------------------------------------------------------------------
    def encode_body_state(self, raw_state: dict) -> Optional[np.ndarray]:
        vec = self.runner_proprio(raw_state, 0.0)
        if vec is None:
            return None
        return np.array(vec, dtype=np.float32)

    # ------------------------------------------------------------------
    def decode_action(self, body_error: np.ndarray) -> dict:
        signal = float(body_error[0]) if len(body_error) > 0 else 0.0
        force = float(np.clip(signal, -1.0, 1.0)) * self.FORCE_INPUT
        return {"force": force}

    # ------------------------------------------------------------------
    def predict_next_body_state(
        self, body_state: np.ndarray, action: dict, dt: float = 0.033
    ) -> np.ndarray:
        """Crude 1-step forward model for efference copy. The maze physics
        is non-linear (heading + collisions); this is only a smooth estimate
        that assumes no collision this tick — good enough for oscillation
        damping in the motor loop."""
        if body_state is None or len(body_state) < 4:
            return body_state
        out = np.array(body_state, dtype=np.float32, copy=True)
        # Update heading from force (matches _MazeSim.TURN_RATE scale).
        f = float(np.clip(action.get("force", 0.0) / self.FORCE_INPUT, -1.0, 1.0))
        # reconstruct heading from sin/cos
        theta = math.atan2(float(out[2]), float(out[3]))
        theta += f * 0.18
        out[2] = float(math.sin(theta))
        out[3] = float(math.cos(theta))
        # Nudge position forward along heading in normalized space (very small)
        speed = 0.01 * (1.0 - abs(f))
        out[0] = float(np.clip(out[0] + speed * math.cos(theta), -1.0, 1.0))
        out[1] = float(np.clip(out[1] + speed * math.sin(theta), -1.0, 1.0))
        return out

    # ------------------------------------------------------------------
    def parse_events(self, raw_state: dict) -> dict:
        return {
            "hit":     bool(raw_state.get("hit",  False)),
            "miss":    False,
            "reward":  1.0 if raw_state.get("hit",  False) else 0.0,
            "penalty": 0.0,
        }

    def extract_world_target(self, raw_state: dict) -> Optional[dict]:
        """Tabula rasa — no goal oracle / compass leaked."""
        return None

    # ------------------------------------------------------------------
    def runner_proprio(self, raw_state: dict, last_commanded: float) -> Optional[list]:
        """21D body-state vector: pos(2) + heading(2) + 8 wall + 8 scent + hunger."""
        x = raw_state.get("maze_x")
        y = raw_state.get("maze_y")
        if x is None or y is None:
            return None
        w = float(raw_state.get("maze_w", 640))
        h = float(raw_state.get("maze_h", 480))
        theta = float(raw_state.get("heading", 0.0))
        whiskers = raw_state.get("whiskers") or [0.0] * 8
        if len(whiskers) < 8:
            whiskers = list(whiskers) + [0.0] * (8 - len(whiskers))
        scent = raw_state.get("scent_whiskers") or [0.0] * 8
        if len(scent) < 8:
            scent = list(scent) + [0.0] * (8 - len(scent))

        pos_x_norm = float(np.clip((float(x) / w) * 2.0 - 1.0, -1.0, 1.0))
        pos_y_norm = float(np.clip((float(y) / h) * 2.0 - 1.0, -1.0, 1.0))
        # Hunger = 1.0 - energy, clipped. Fed to NeurochemicalState.on_hunger
        # by the runner as serotonin-drain aversion; the GNG also sees it as
        # part of the proprio vector so valence can condition on body state.
        energy = float(raw_state.get("energy", 1.0))
        hunger = float(np.clip(1.0 - energy, 0.0, 1.0))
        # Pheromone ∈ [0, ~1+]; clipped here so the proprio vector stays in
        # the [-1, 1] band the GNG encoder expects.
        pheromone = float(np.clip(raw_state.get("pheromone", 0.0), 0.0, 1.0))

        return [
            pos_x_norm,
            pos_y_norm,
            float(np.clip(math.sin(theta), -1.0, 1.0)),
            float(np.clip(math.cos(theta), -1.0, 1.0)),
            *[float(np.clip(w_, 0.0, 1.0)) for w_ in whiskers[:8]],
            *[float(np.clip(s_, 0.0, 1.0)) for s_ in scent[:8]],
            hunger,
            pheromone,
        ]
