from abc import ABC, abstractmethod
import numpy as np
from typing import List, Optional


class GameAdapter(ABC):
    """
    Abstraction layer connecting AMI-Ogma's cognitive architecture to a
    specific embodied environment (game, simulation, or robot).

    Separates the WHAT (body state, action) from the HOW (environment specifics).
    All body state vectors are normalized to [-1, 1] before entering the pipeline.

    To support a new environment, subclass this and pass the instance to
    ProprioceptiveChannel and GlobalWorkspaceHarness.  The cognitive architecture
    never changes — only the adapter does.

    See: docs/proprioception_and_embodiment_roadmap.md
    """

    @property
    @abstractmethod
    def body_schema(self) -> List[str]:
        """
        Ordered list of named body state dimensions, all normalized to [-1, 1].
        Examples:
          Breakout:   ["paddle_x", "paddle_vx"]
          Platformer: ["pos_x", "pos_y", "vel_x", "vel_y", "on_ground", "facing"]
          Robot arm:  ["j0_angle", "j0_vel", "j1_angle", "j1_vel", "ee_x", "ee_y", "ee_z"]
        """

    @property
    def body_dim(self) -> int:
        return len(self.body_schema)

    @abstractmethod
    def encode_body_state(self, raw_state: dict) -> np.ndarray:
        """
        Extract and normalize the body/self state from raw environment telemetry.

        MUST NOT include world-object positions (ball, enemies, targets, etc.).
        Those are the exclusive domain of the sensory EPMs.  Only encode what
        the agent would know about its own body without external perception.

        Returns: float32 array of shape (body_dim,), values in [-1, 1].
        Returns None if the raw_state does not contain body information.
        """

    @abstractmethod
    def decode_action(self, body_error: np.ndarray) -> dict:
        """
        Map a body-relative error vector to environment-specific action commands.

        body_error = (desired_body_state - current_body_state), normalized [-1, 1].
        Positive error[0] means the body should move in the positive direction.

        Returns: dict matching the game/robot's action API.
        """

    @abstractmethod
    def predict_next_body_state(
        self, body_state: np.ndarray, action: dict, dt: float = 0.033
    ) -> np.ndarray:
        """
        Efference copy: given current body state and an outgoing action command,
        predict the body state on the NEXT tick WITHOUT waiting for sensor
        confirmation.  Used to prevent oscillation in the motor loop.

        Returns: float32 array of shape (body_dim,), values in [-1, 1].
        """

    @abstractmethod
    def parse_events(self, raw_state: dict) -> dict:
        """
        Extract reward/penalty signals from raw environment state.

        Returns: {
            "hit":     bool,   # positive reinforcement event
            "miss":    bool,   # negative reinforcement event
            "reward":  float,  # continuous reward [0, 1]
            "penalty": float,  # continuous penalty [0, 1]
        }
        """

    def extract_world_target(self, raw_state: dict) -> Optional[np.ndarray]:
        """
        Optional privileged world-target override for calibration / oracle modes.
        Default: None — world targets come solely from EPM prediction.
        """
        return None

    def runner_proprio(self, raw_state: dict, last_commanded: float) -> Optional[list]:
        """
        Optional per-game proprioceptive body-state vector for the v3 runner.

        Returning None tells the runner to use its built-in fallback (currently
        the Pong paddle-physics path inline in run_e2e_v3.py). Override in
        adapters whose telemetry isn't shaped like Breakout's (CartPole,
        maze rover, etc.). The adapter is expected to cache any per-tick
        history (prev pos/vel) needed for diff computations.

        Returns a 1-D list of floats, all in [-1, 1], that will be fed into
        ProprioceptiveEncoder. Length should match the encoder's input_dim
        (default 4); extra elements are trimmed by the encoder.
        """
        return None

    # ------------------------------------------------------------------
    # Tuning hints — adapters override these to shape the encoder/GNG for
    # their environment. The runner reads these at construction time and
    # passes them through to the C++ EPM and the GNG.
    # ------------------------------------------------------------------
    @property
    def proprio_input_dim(self) -> int:
        """Number of dimensions in the proprio body-state vector. Default 6
        matches Breakout. CartPole / maze rovers override to expose richer
        body senses."""
        return 6

    @property
    def disable_wall_reflex(self) -> bool:
        """Pong's ActionDecoder has a spinal-level wall escape that fires
        whenever paddle_x is near the left/right edge. For non-paddle
        environments (CartPole, maze) the paddle_x kwarg is either stubbed
        to 0 or means something different, so this reflex misfires. Adapters
        whose body is not a 1D paddle override this to True."""
        return False
