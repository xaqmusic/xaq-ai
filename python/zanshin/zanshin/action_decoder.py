"""
action_decoder.py — Active Inference motor decoder for AMI-Ogma v3.

Architecture
------------
Three-layer motor policy using Expected Free Energy (EFE) minimization:

Layer 1 — EFE Policy (primary motor drive)
    Selects actions by minimizing Expected Free Energy:
      EFE(action) = pragmatic_value + w * epistemic_value
    Pragmatic: expected valence of predicted next state (NodeValenceMap).
    Epistemic: entropy of the transition distribution (curiosity/novelty).
    Naturally implements the Playful Machine: boredom → explore, surprise → focus.

Layer 2 — Hebbian Transition Table (fast-path habit)
    Key: (modality, prev_node, cur_node, proprio_node) → velocity bias.
    Acts as a "habit cache" that can override deliberative EFE when
    confidence is high. Analogous to dual-process theory: System 1 (habit)
    can preempt System 2 (deliberation) for well-practiced responses.

Layer 3 — Perturbation Probe (forced exploration)
    Short random taps when EFE has insufficient data. Ensures continuous
    motor engagement to populate the action-conditioned transition model.

The EFE policy requires:
  - NodeValenceMap:        emotional coloring of GNG states (from dopamine)
  - ActionTransitionModel: P(next_state | current_state, action)
Both are updated externally and passed via set_active_inference_components().

Critically, ball position is NEVER consulted. The decoder discovers tracking
purely by noticing that certain motor trajectories produce positive dopamine
(hits) and avoiding states that produce negative dopamine (misses/wall-stuck).

Usage:
    decoder = ActionDecoder(projection_dim=128, game_width=640)
    decoder.set_active_inference_components(valence_map, transition_model, efe_policy)
    accel = decoder.act(fused_embedding=emb, serotonin=0.6, dopamine=0.2, ...)
    game.apply_force(accel)
"""

import numpy as np
from collections import deque
from typing import Optional, Dict, Tuple

from zanshin_core.rng import derive_rng

# ---------------------------------------------------------------------------
# Probe state machine
# ---------------------------------------------------------------------------

_PROBE_PHASE_POS = "pos"     # currently applying +δ
_PROBE_PHASE_NEG = "neg"     # currently applying -δ
_PROBE_PHASE_HOLD = "hold"   # using winner direction, waiting for next cycle


class ActionDecoder:
    """
    Homeokinetic Active Inference decoder.

    Parameters
    ----------
    projection_dim  int     embedding dimension (unused — kept for API compat)
    game_width      int     canvas width in pixels (unused — kept for API compat)
    accel           float   base acceleration magnitude per tick
    hebbian_lr      float   learning rate for Hebbian table updates
    probe_period    int     ticks between perturbation probe cycles
    probe_ticks     int     ticks to hold each probe direction (+δ / -δ)
    stag_trigger    int     idle ticks that force-restart the probe cycle
    """

    def __init__(self,
                 projection_dim:  int   = 128,
                 game_width:      int   = 640,
                 accel:           float = 4.0,
                 hebbian_lr:      float = 0.03,
                 probe_period:    int   = 20,
                 probe_ticks:     int   = 6,
                 stag_trigger:    int   = 20,
                 min_bias:        float = 0.35,
                 master_seed:     int   = 0,
                 # Legacy / compat args — silently accepted, not used
                 **kwargs):

        self._dim         = projection_dim
        self._width       = game_width
        self._accel       = accel
        self._lr          = hebbian_lr
        self._probe_period = probe_period
        self._probe_ticks  = probe_ticks
        self._stag_trigger = stag_trigger
        # Local RNG isolates decoder stochasticity from every other component's
        # stream. Changes elsewhere (e.g. an extra random call in the voter)
        # do not shift probe direction sequences.
        self._rng = derive_rng(master_seed, "decoder.probe")

        # ------------------------------------------------------------------
        # Hebbian Transition Table
        # Key: (modality, prev_node, cur_node, proprio_node) → velocity bias
        # ------------------------------------------------------------------
        self._table: Dict[Tuple[str, int, int, int], float] = {}
        self._coarse_table: Dict[Tuple[str, int, int], float] = {}  # (mod, node, proprio) → velocity
        self._prev_winner: Dict[str, int] = {}
        self._trace = deque(maxlen=12)   # eligibility trace: (key, cmd_v, credit)
        self._max_v = 12.0               # table value clamp
        self._reflex_disabled = False    # Babbling toggle
        self._wall_reflex_disabled = False    # Pong-specific edge-escape
        self._reinforced_keys: set = set()  # keys with recent dopamine reinforcement

        # ------------------------------------------------------------------
        # Perturbation probe state machine
        # ------------------------------------------------------------------
        self._probe_phase     = _PROBE_PHASE_HOLD
        self._probe_countdown = probe_period      # ticks until next probe
        self._probe_step      = 0                 # ticks into current phase
        self._probe_dir       = +1.0              # +1 or -1
        self._probe_sero_pos: list = []           # serotonin samples during +δ
        self._probe_sero_neg: list = []           # serotonin samples during -δ
        self._probe_winner    = 0.0               # chosen direction after compare
        self._hold_dir        = 0.0               # persisted between probes

        # ------------------------------------------------------------------
        # Stagnation detection
        # ------------------------------------------------------------------
        self._last_paddle_x   = None
        self._idle_ticks      = 0
        self._last_force      = 0.0     # previous tick's motor command (for transition model)

        # ------------------------------------------------------------------
        # Extinction learning: sustained aversion erodes habits
        # ------------------------------------------------------------------
        self._aversion_ticks  = 0       # consecutive ticks with serotonin below gate
        self._serotonin_gate  = 0.35    # threshold for "sustained pain"
        self._extinction_trigger = 10   # ticks of pain before extinction kicks in

        # ------------------------------------------------------------------
        # Active Inference components (set via set_active_inference_components)
        # ------------------------------------------------------------------
        self._valence_map      = None   # NodeValenceMap
        self._transition_model = None   # ActionTransitionModel
        self._efe_policy       = None   # EFEPolicy
        self._path_blame       = None   # NodePathBlame

        # ------------------------------------------------------------------
        # Telemetry
        # ------------------------------------------------------------------
        self.total_actions      = 0
        self.predicted_accuracy = 0.0
        self._tle_baseline      = 0.5
        self._tle_alpha         = 0.01
        self._probe_count       = 0
        self._indeterminate_run = 0   # consecutive ties → force flip
        self._last_serotonin    = 0.5
        self._last_target_bias  = 0.0
        self._last_active_key   = None   # last valid Hebbian key seen
        self._explore_sigma     = 0.0    # EMA of exploration intensity
        # Motor-share EMAs — what's actually driving the body.
        self._habit_sigma       = 0.0    # EMA of |blend * table_accel|
        self._efe_sigma         = 0.0    # EMA of |(1-blend) * efe_drive|
        self._noise_sigma       = 0.0    # EMA of |(1-blend) * exploration_v|
        # Reward-drought tracker: ticks since last positive blame event.
        # Drives an extra "drought-boredom" term that overrides the TLE
        # boredom when sparse-reward search has stalled. Reactive games
        # never stall (events fire densely) so the drought stays near 0
        # and contributes nothing. Maze, with rare eats, ramps drought
        # over hundreds of ticks → forces exploration when the value-
        # iteration policy gets stuck in a dead loop.
        self._ticks_since_positive_event = 0
        self._drought_boredom_sigma = 0.0   # EMA for telemetry

        # --- Phase 4b intervention flags (opt-in, default off) ---
        # When set, drought-driven noise amplification scales down with
        # V-map maturity: noise_boost = 1.5 * (1 - 0.7 * v_maturity).
        # Rationale: a strong V(s) gradient means the policy has something
        # informed to follow, so cranking random exploration fights it.
        self._v_maturity_noise_boost = False
        # When set, probe direction choice at each tap start uses a drought-
        # scaled flip probability: flip_prob = 0.7 - 0.5 * drought_ema.
        # Under sustained drought, the agent commits to a direction longer.
        self._direction_persistence = False
        # When set, a probe tap ends as soon as the active visual node
        # changes (agent moved to a new cell), with a hard cap at 60 ticks.
        self._state_change_tap_gate = False
        self._probe_tap_start_node  = -1   # node ID at tap start

    # ------------------------------------------------------------------
    # Active Inference wiring
    # ------------------------------------------------------------------

    def set_active_inference_components(self, valence_map, transition_model,
                                        efe_policy, path_blame=None):
        """Wire in the EFE components. Called once during setup."""
        self._valence_map      = valence_map
        self._transition_model = transition_model
        self._efe_policy       = efe_policy
        self._path_blame       = path_blame
        components = "NodeValenceMap + ActionTransitionModel + EFEPolicy"
        if path_blame:
            components += " + NodePathBlame"
        print(f"  [ActionDecoder] Active Inference components wired: {components}")

    @property
    def has_efe(self) -> bool:
        return self._efe_policy is not None

    # ------------------------------------------------------------------
    # Primary interface
    # ------------------------------------------------------------------

    def act(self,
            fused_embedding: Optional[np.ndarray] = None,
            ball_x:     float = 0.0,    # accepted, not used
            paddle_x:   float = 0.0,
            dopamine:   float = 0.2,
            serotonin:  float = 0.5,
            tle:        float = 0.0,
            paddle_width: float = 100.0,
            target_bias:  float = 0.0,  # D3: spatial prior from SpatialDecoder
            active_modality: str  = "retinal",
            active_node:     int  = -1,
            proprio_node:    int  = -1,
            visual_trajectory: list = None,
            **kwargs) -> float:
        """
        Compute paddle acceleration for this tick.

        target_bias: [-1, 1] directional prior from SpatialDecoder (D3).
            Positive = probe right first, negative = probe left first.
            Never steers directly — only seeds the probe's opening direction.
        """
        self.total_actions += 1
        self._last_serotonin = serotonin
        self._last_target_bias = target_bias

        # ------------------------------------------------------------------
        # 1. Stagnation detection
        # ------------------------------------------------------------------
        if self._last_paddle_x is not None:
            if abs(paddle_x - self._last_paddle_x) < 1.0:
                self._idle_ticks += 1
            else:
                self._idle_ticks = 0
        self._last_paddle_x = paddle_x

        # Stagnation-triggered probe acceleration: if stalled and table is
        # cold, force the probe cycle to restart immediately (not toward the
        # ball — toward whichever direction improves serotonin).
        if (self._idle_ticks >= self._stag_trigger and
                self._probe_phase == _PROBE_PHASE_HOLD and
                self._probe_countdown > self._probe_ticks):
            self._probe_countdown = self._probe_ticks  # skip to probe now
            self._idle_ticks = 0

        # ------------------------------------------------------------------
        # 1b. Visual trajectory (Reality Token semantic history)
        #     Convert raw winner_history into a fixed-length trajectory tuple.
        #     The trajectory encodes the ball's direction of travel through
        #     the GNG topology — the key predictive signal for where the ball
        #     will be when it reaches the paddle.
        # ------------------------------------------------------------------
        TRAJ_LEN = 2  # last 2 distinct visual nodes (direction of travel)
        if visual_trajectory is None:
            visual_trajectory = []
        # Deduplicate consecutive repeats (same node held for multiple ticks)
        deduped = []
        for nid in visual_trajectory:
            if not deduped or nid != deduped[-1]:
                deduped.append(nid)
        # Take last TRAJ_LEN entries, pad with -1 if shorter
        if len(deduped) >= TRAJ_LEN:
            traj_tuple = tuple(deduped[-TRAJ_LEN:])
        else:
            traj_tuple = tuple([-1] * (TRAJ_LEN - len(deduped)) + deduped)
        self._current_trajectory = traj_tuple

        # ------------------------------------------------------------------
        # 2. Hebbian learning: Bio-Mimetic Neurochemical Plasticity
        #    Valence (V)  = Dopamine deviation from baseline (0.2).
        #    Arousal (η)  = Inverse Serotonin (high pain/noise = high turnover).
        #    Update Rule  = current + (lr * V * η * credit)
        # ------------------------------------------------------------------
        valence = dopamine - 0.2
        if self._trace and abs(valence) > 0.01:
            # Arousal acts as a pruning catalyst: low serotonin (pain/instability)
            # makes existing associations much more volatile.
            # Range: ~0.9 (stable) to 10.0 (high pain/aversive state).
            arousal = 1.0 / (serotonin + 0.1)
            
            # Learning rate scales with valence (hits reinforced more than punishment,
            # but not overwhelmingly — 12x was letting one lucky hit near a wall
            # create habits that hundreds of wall-stuck ticks couldn't erode).
            dynamic_lr = self._lr * 3.0 if valence > 0.0 else self._lr
            
            decay = 1.0
            for key, prev_v, credit in reversed(self._trace):
                current = self._table.get(key, 0.0)

                # Bi-directional update:
                # Positive valence reinforces; negative valence erodes.
                # Low serotonin amplifies both directions but dominates in the negative.
                update = dynamic_lr * valence * arousal * decay * credit

                self._table[key] = float(np.clip(
                    current + update,
                    -self._max_v, self._max_v
                ))
                # Also update coarse table: (mod, cur_node, proprio)
                # This accumulates denser data by dropping prev_node.
                coarse_key = (key[0], key[2], key[3])  # (mod, cur_node, proprio)
                coarse_current = self._coarse_table.get(coarse_key, 0.0)
                self._coarse_table[coarse_key] = float(np.clip(
                    coarse_current + update,
                    -self._max_v, self._max_v
                ))

                # Protect recently-reinforced entries from metabolic pruning.
                if abs(update) > 0.01:
                    self._reinforced_keys.add(key)

                decay *= 0.8  # Eligibility trace decay

        # ------------------------------------------------------------------
        # 3. Read Hebbian table suggestion
        # ------------------------------------------------------------------
        table_v = 0.0
        actual_confidence = 0.0
        key = None

        if active_node != -1:
            prev_id = self._prev_winner.get(active_modality, active_node)
            key = (active_modality, prev_id, active_node, proprio_node)
            self._prev_winner[active_modality] = active_node
            self._last_active_key = key   # update to latest valid key

            if key in self._table and not getattr(self, '_hebbian_disabled', False):
                table_v = self._table[key]
                actual_confidence = min(1.0, abs(table_v) / self._max_v)

            # Coarse Hebbian: (modality, cur_node, proprio_node) — denser data.
            # This captures "what should I do at this visual+paddle state?"
            # without requiring the exact previous node to match.
            coarse_key = (active_modality, active_node, proprio_node)
            coarse_v = self._coarse_table.get(coarse_key, 0.0)
            if abs(coarse_v) > abs(table_v):
                # Coarse table has stronger signal — use it
                table_v = coarse_v
                actual_confidence = min(1.0, abs(coarse_v) / self._max_v)
        # else: active_node == -1 (novel tick) — reuse _last_active_key for probe write

        # ------------------------------------------------------------------
        # 3b. Record node visit to path blame system
        #     The path is the causal chain: when a hit/miss occurs, the
        #     blame system walks this path backward with exponential decay
        #     to credit/blame only the nodes and edges that led to the event.
        # ------------------------------------------------------------------
        if self._path_blame is not None and active_node >= 0:
            self._path_blame.record(
                visual_node  = active_node,
                proprio_node = proprio_node,
                hebbian_key  = key if key else self._last_active_key,
                action_force = self._last_force,
            )

        # ------------------------------------------------------------------
        # 4. The Playful Machine (Homeokinetic Boredom Drive)
        # ------------------------------------------------------------------
        # Following Der & Martius: If prediction is perfect (low TLE), the agent 
        # gets bored and actively INCREASES motor noise to perturb its environment
        # and discover new causal relationships to the topological graph. If it is
        # highly surprised (high TLE), it suppresses noise to focus on the event.
        boredom = max(0.0, 1.0 - (tle * 2.0))

        # Drought-boredom: when the agent has gone a long time without a
        # positive event, override TLE-suppressed boredom so it explores
        # rather than committing harder to a stuck policy. Threshold scales
        # to 2× the path-blame's own learned mean inter-event interval —
        # automatic per-env tuning, no hand-picked seconds. Reactive games
        # never trip it; maze ramps it after a minute of empty search.
        drought_boredom = 0.0
        if self._path_blame is not None:
            _intervals = getattr(self._path_blame, "_event_intervals", None)
            if _intervals and len(_intervals) > 0:
                _mean_iv = sum(_intervals) / len(_intervals)
                _drought_thresh = max(200.0, 2.0 * _mean_iv)
                drought_boredom = float(np.clip(
                    self._ticks_since_positive_event / _drought_thresh,
                    0.0, 1.0,
                ))
        boredom = max(boredom, drought_boredom)
        self._drought_boredom_sigma = (
            0.95 * self._drought_boredom_sigma + 0.05 * drought_boredom
        )
        self._ticks_since_positive_event += 1

        probe_v = self._run_probe(serotonin, active_node=active_node)
        # Drought-noise amplification. Default 1.5× (Up to 2x noisy when bored).
        # When --v-maturity-noise-boost is set, attenuate by V-map maturity:
        # a strong V(s) gradient is the signal the policy should be following,
        # so random exploration steps on it.
        noise_scale = 1.5
        if self._v_maturity_noise_boost and self._efe_policy is not None:
            v_mat = getattr(self._efe_policy, "_last_v_maturity", 0.0)
            noise_scale = 1.5 * (1.0 - 0.7 * float(v_mat))
        playful_noise = probe_v * (0.5 + noise_scale * boredom)
        
        # EMA Telemetry: Ensure logs show average exploration intensity
        # rather than just single-tick snapshots.
        current_noise_mag = abs(playful_noise)
        self._explore_sigma = 0.95 * self._explore_sigma + 0.05 * current_noise_mag
        
        exploration_v = playful_noise
        
        if getattr(self, '_reflex_disabled', False):
            # Even if explicitly disabled, inject a tiny 15% environmental noise
            # baseline to ensure the paddle never safely rests in a trivial mathematical 
            # dead-center attractor (the "lazy robot" trap).
            exploration_v = playful_noise * 0.15

        # ------------------------------------------------------------------
        # 5. Compose final action via EFE + habit + probe
        # ------------------------------------------------------------------
        table_accel = float(np.clip(table_v, -self._accel, self._accel))

        # --- Active Inference: EFE policy as primary motor drive ---
        # The EFE policy considers both pragmatic value (move toward
        # high-valence states) and epistemic value (explore uncertain
        # transitions). It replaces the old spatial decoder innate drive.
        efe_drive = 0.0
        if self._efe_policy is not None and active_node >= 0:
            # Update valence map with current neurochemical state
            if self._valence_map is not None:
                self._valence_map.update(
                    active_node, proprio_node, dopamine, serotonin,
                    trajectory=self._current_trajectory)
                # Asymmetric decay every tick: negative fades fast (noisy miss signal),
                # positive fades slow (rare hit signal worth preserving).
                self._valence_map.decay_all()

            # Record transition in action-conditioned model
            if self._transition_model is not None:
                self._transition_model.record(
                    active_node, proprio_node, self._last_force,
                    trajectory=self._current_trajectory)

            # Query EFE policy for action
            efe_drive = self._efe_policy.select_action(
                visual_node  = active_node,
                proprio_node = proprio_node,
                valence_map  = self._valence_map,
                transition_model = self._transition_model,
                tle          = tle,
                serotonin    = serotonin,
                trajectory   = self._current_trajectory,
            )

        # --- Three-way composition ---
        # The motor command blends three signals:
        #   1. Habit (Hebbian table) — fast, automatic, high when confident
        #   2. EFE policy — deliberative, model-based, the primary drive
        #   3. Probe — forced random exploration, fallback when both are weak
        #
        # As the transition model fills in, the EFE policy subsumes the
        # probe's role (it has its own epistemic exploration via entropy).
        # The probe should fade as EFE gains data — otherwise random noise
        # fights the EFE's informed decisions.
        blend = actual_confidence  # 0 = EFE+probe, 1 = habit

        # How much data does the EFE transition model have for this state?
        # Only attenuate probe when the model has DENSE data (≥10 observations)
        # for this state — sparse data still needs probe-driven exploration.
        efe_density = 0
        if (self._transition_model is not None and active_node >= 0):
            efe_density = self._transition_model.data_density(
                active_node, proprio_node, trajectory=self._current_trajectory)

        if efe_density >= 10:
            # EFE has enough data to navigate — reduce probe interference
            exploration_v *= 0.4

        # Stagnation override: if idle too long, override everything
        if self._idle_ticks > 20:
            blend *= 0.1
            if abs(exploration_v) < 0.01 and abs(efe_drive) < 0.01:
                exploration_v = (self._rng.random() * 2.0 - 1.0) * (self._accel * 0.30)
            elif abs(exploration_v) > 0.01:
                exploration_v *= 3.0

        final_accel = (blend * table_accel +
                       (1.0 - blend) * (efe_drive + exploration_v))
        final_accel = float(np.clip(final_accel, -self._accel, self._accel))

        # Motor-share telemetry — track how much each channel actually
        # contributed to the command *after* the blend mix. Sum ≈ |final_accel|.
        _h_contrib = abs(blend * table_accel)
        _e_contrib = abs((1.0 - blend) * efe_drive)
        _n_contrib = abs((1.0 - blend) * exploration_v)
        self._habit_sigma = 0.95 * self._habit_sigma + 0.05 * _h_contrib
        self._efe_sigma   = 0.95 * self._efe_sigma   + 0.05 * _e_contrib
        self._noise_sigma = 0.95 * self._noise_sigma + 0.05 * _n_contrib

        # ------------------------------------------------------------------
        # 5b. Proprioceptive wall reflex (spinal-level escape)
        #     Sub-cortical pain withdrawal — no GNG, no EFE, no learning.
        #     The agent's body knows it's at a boundary through proprio.
        #     After 5 ticks at a wall, inject a fixed AWAY force that
        #     overrides other signals. Generalizes to any embodied agent:
        #     "if touching a boundary, push away."
        # ------------------------------------------------------------------
        wall_margin = 15.0  # pixels from edge to count as "at wall"
        at_left_wall  = paddle_x < wall_margin
        at_right_wall = (paddle_x + paddle_width) > (self._width - wall_margin)

        if self._wall_reflex_disabled:
            at_left_wall = at_right_wall = False

        if at_left_wall or at_right_wall:
            self._wall_ticks = getattr(self, '_wall_ticks', 0) + 1
            if self._wall_ticks > 5:
                # Reflex strength ramps: gentle at first, forceful if stuck
                reflex_strength = min(1.0, (self._wall_ticks - 5) / 15.0)
                escape_dir = self._accel if at_left_wall else -self._accel
                # Override: reflex blends in proportionally to how stuck we are
                final_accel = (1.0 - reflex_strength) * final_accel + reflex_strength * escape_dir
        else:
            self._wall_ticks = 0

        # ------------------------------------------------------------------
        # 6. Eligibility trace update
        # ------------------------------------------------------------------
        if key:
            credit = max(0.15, actual_confidence)
            self._trace.append((key, final_accel, credit))

        # ------------------------------------------------------------------
        # 7. Reinforcement key decay
        # ------------------------------------------------------------------
        # Clear reinforced keys periodically so that only recently-reinforced
        # entries get protection. Old entries should eventually be prunable.
        if self.total_actions % 500 == 0 and self._reinforced_keys:
            self._reinforced_keys.clear()

        # ------------------------------------------------------------------
        # 8. Telemetry
        # ------------------------------------------------------------------
        self._last_force = final_accel
        self._tle_baseline = (
            (1 - self._tle_alpha) * self._tle_baseline + self._tle_alpha * tle
        )
        self.predicted_accuracy = max(0.0, 1.0 - self._tle_baseline)

        return float(final_accel)

    # ------------------------------------------------------------------
    # Probe state machine
    # ------------------------------------------------------------------

    def _run_probe(self, serotonin: float, active_node: int = -1) -> float:
        """
        True stochastic motor babbler (short random taps).
        Mimics human-like explorative key presses: silent intervals punctuated by 1-2 frame taps.
        """
        if self._probe_phase == _PROBE_PHASE_HOLD:
            self._probe_countdown -= 1
            if self._probe_countdown <= 0:
                self._probe_phase = _PROBE_PHASE_POS  # Re-purposed to mean "tapping"
                self._probe_step = 0
                self._probe_tap_start_node = int(active_node)

                # Innate Spatial Prior (target_bias) skews the chance of tapping toward
                # the ball by up to +/- 30%, but guarantees it remains fundamentally random.
                bias = getattr(self, '_last_target_bias', 0.0)
                prob_right = 0.5 + (bias * 0.3)

                # --direction-persistence: under drought, the agent commits
                # to a direction across taps; `flip_prob = 0.7 - 0.5 * drought`
                # controls how often a fresh random direction overrides the
                # previous one. With drought≈0 we flip 70% of taps (near-
                # baseline randomness); with drought≈1 we flip only 20%.
                if self._direction_persistence:
                    drought = float(np.clip(self._drought_boredom_sigma, 0.0, 1.0))
                    flip_prob = 0.7 - 0.5 * drought
                    if self._rng.random() < flip_prob:
                        self._probe_dir = 1.0 if self._rng.random() < prob_right else -1.0
                    # else: carry over previous self._probe_dir
                else:
                    self._probe_dir = 1.0 if self._rng.random() < prob_right else -1.0

                # Arousal scaling: low serotonin (pain) → shorter waits → more urgent.
                # Range: 0.25x (pain) to 1.0x (happy).
                arousal = 0.25 + 0.75 * serotonin

                # Short idle intervals: 3-15 frames (~100ms to 500ms at 30fps).
                # Active inference requires continuous motor engagement to
                # populate the action-conditioned transition model.
                self._next_probe_wait = max(2, int(self._rng.integers(3, 16) * arousal))
                self._probe_count += 1

            return 0.0  # True idle during HOLD phase

        elif self._probe_phase == _PROBE_PHASE_POS:
            self._probe_step += 1

            # Autotune tap duration from path_blame's inter-event cadence.
            # Reactive envs (Pong/CartPole, path_length≈30-60) keep taps at
            # 1-2 frames — brief perturbation, rapid feedback. Sparse-reward
            # envs (maze, path_length≥200) extend taps so each random
            # commitment actually traverses between cells before re-deciding.
            # Reuses path_length which already self-tunes from the same
            # signal (inter-event interval).
            tap_max = 2
            if self._path_blame is not None:
                pl = int(getattr(self._path_blame, "path_length", 60))
                tap_max = int(max(2, min(12, pl / 40.0)))
            if tap_max <= 2:
                tap_duration = 1 if self._rng.random() < 0.8 else 2
            else:
                tap_duration = int(self._rng.integers(1, tap_max + 1))

            # --state-change-tap-gate: end the tap when the active visual
            # node changes from the node at tap start (agent moved cell),
            # or at a 60-tick hard cap to prevent stuck state forever.
            # Bypasses the autotuned tap_duration entirely when set.
            if self._state_change_tap_gate:
                state_changed = (active_node >= 0 and
                                 self._probe_tap_start_node >= 0 and
                                 int(active_node) != self._probe_tap_start_node)
                if state_changed or self._probe_step >= 60:
                    self._probe_phase = _PROBE_PHASE_HOLD
                    self._probe_countdown = getattr(self, '_next_probe_wait', 20)
            elif self._probe_step >= tap_duration:
                # Tap finished. Go back to sleep.
                self._probe_phase = _PROBE_PHASE_HOLD
                self._probe_countdown = getattr(self, '_next_probe_wait', 20)

            return float(self._probe_dir * self._accel)

        return 0.0

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def learned_count(self) -> int:
        return len([v for v in self._table.values() if abs(v) > 0.1])

    def action_distribution(self) -> dict:
        return {}   # no longer tracking per-action counts (probe dominates early)

    def set_reflex_disabled(self, disabled: bool):
        """Toggle exploration/babbling probe."""
        self._reflex_disabled = disabled
        print(f"  [ActionDecoder] {'Exploration' if not disabled else 'Inference-ONLY'} mode")

    def set_wall_reflex_disabled(self, disabled: bool):
        """Toggle the Pong-specific spinal-level wall escape. Must be True
        for non-paddle environments (CartPole, maze) or the reflex misfires
        on any paddle_x kwarg that doesn't represent a 1D horizontal body."""
        self._wall_reflex_disabled = disabled

    def set_hebbian_disabled(self, disabled: bool):
        """Toggle Hebbian weighting (rely on prediction)."""
        self._hebbian_disabled = disabled
        print(f"  [ActionDecoder] {'Hebbian' if not disabled else 'Prediction-ONLY'} mode")

    def fire_blame_event(self, signal: float):
        """
        Fire temporal credit/blame assignment along the node path.

        Called by the game loop when a neurochemical event occurs:
          signal > 0  →  credit (hit, brick)
          signal < 0  →  blame  (miss, sustained wall-stuck)

        Walks the path backward with exponential decay, updating valence
        map and Hebbian table for each node in the causal chain.
        """
        if self._path_blame is None:
            return {}
        # ANY blame event resets the drought counter — both hits AND misses
        # mean the agent is engaging with the world. Drought ramps only
        # when nothing happens, signalling a stuck/spinning policy.
        self._ticks_since_positive_event = 0
        return self._path_blame.assign(
            signal         = signal,
            valence_map    = self._valence_map,
            hebbian_table  = self._table,
            transition_model = self._transition_model,
        )

    def adaptive_metabolic_rate(self, gng_node_count: int) -> int:
        """
        Compute the metabolic pruning rate from the system's own state.

        No hard-coded neurochemical thresholds — the rate emerges from
        the relationship between GNG size, model utilization, and whether
        the agent is actively learning.

        Three signals drive the decision:
          1. Utilization = (nodes with transition data) / (total GNG nodes)
          2. Learning momentum = is the transition model still growing?
          3. Network size relative to what the model actually uses

        Returns int: max prunes per tick (0-2).
        """
        if gng_node_count < 1:
            return 0

        # Count nodes the transition model has data for
        utilized = 0
        trans_total = 0
        if self._transition_model is not None:
            seen_nodes = set()
            for key in self._transition_model._transitions:
                traj_or_single, p, a = key
                # Count all visual nodes referenced in trajectory keys
                for nid in traj_or_single:
                    if nid >= 0:
                        seen_nodes.add(nid)
            utilized = len(seen_nodes)
            trans_total = self._transition_model.total_transitions

        utilization = utilized / gng_node_count

        # Transition density: how many observations per utilized node?
        # High density = rich model, nodes are well-characterized.
        # Low density = sparse data spread thin across too many nodes.
        density = trans_total / max(1, utilized)

        # Small network protection: relative to model needs.
        min_viable = max(30, int(utilized * 1.5))
        if gng_node_count <= min_viable:
            return 0

        # Density-aware control:
        # Dense model (>15 obs/node) and high utilization → protect
        # Sparse model (<5 obs/node) with low utilization → bloated, prune
        # The thresholds are relative ratios, not absolute numbers.
        if utilization > 0.5 and density > 10:
            return 0   # lean, well-characterized — protect
        elif utilization > 0.4:
            return 0   # reasonably lean — protect
        elif utilization > 0.25:
            return 1   # moderate bloat — gentle trim
        else:
            return 2   # heavy bloat — prune harder

    def to_dict(self) -> dict:
        table_size = len(self._table)
        d = {
            "learned_associations": self.learned_count(),
            "fwd_error":            round(self._tle_baseline, 4),
            "fwd_confidence":       round(
                len([v for v in self._table.values() if abs(v) > 0.1]) / max(1, table_size),
                3
            ),
            "predicted_accuracy":   round(self.predicted_accuracy, 3),
            "explore_sigma":        round(self._explore_sigma, 3),
            # Motor-share split — fraction of motor drive from each channel.
            # brain_pct = EFE (deliberative), habit_pct = Hebbian table,
            # random_pct = playful/probe noise. Sum ≈ 1.0.
            **(lambda _t: {
                "brain_pct":  round(self._efe_sigma   / _t, 3) if _t > 1e-6 else 0.0,
                "habit_pct":  round(self._habit_sigma / _t, 3) if _t > 1e-6 else 0.0,
                "random_pct": round(self._noise_sigma / _t, 3) if _t > 1e-6 else 0.0,
            })(self._efe_sigma + self._habit_sigma + self._noise_sigma),
            "probe_count":          self._probe_count,
            "probe_hold_dir":       round(self._hold_dir, 3),
            "probe_phase":          self._probe_phase,
            "target_bias":          round(self._last_target_bias, 3),
            "stagnation_ticks":     self._idle_ticks,
            "last_serotonin":       round(self._last_serotonin, 3),
            "table_size":           table_size,
        }
        # Active Inference telemetry
        if self._efe_policy is not None:
            d["efe"] = self._efe_policy.stats()
        if self._valence_map is not None:
            d["valence"] = self._valence_map.stats()
        if self._transition_model is not None:
            d["transitions"] = self._transition_model.stats()
        if self._path_blame is not None:
            d["path_blame"] = self._path_blame.stats()
        return d

    def purge_nodes(self, modality: str, pruned_ids: list):
        """
        Metabolic Cleanup (D1): Delete associations involving pruned GNG nodes,
        but PROTECT entries that received recent dopamine reinforcement. These
        carry learned action→outcome information that is costly to re-discover.
        """
        if not pruned_ids:
            return

        to_del = []
        protected = 0
        p_set = set(pruned_ids)

        for key in self._table.keys():
            k_mod, k_prev, k_cur, k_proprio = key

            matched = False
            if k_mod == modality and (k_prev in p_set or k_cur in p_set):
                matched = True
            elif modality == "proprioceptive" and k_proprio in p_set:
                matched = True

            if matched:
                if key in self._reinforced_keys:
                    protected += 1  # spare this entry
                else:
                    to_del.append(key)

        for k in to_del:
            del self._table[k]

        # Purge coarse table entries too
        coarse_del = []
        for ck in self._coarse_table:
            c_mod, c_node, c_proprio = ck
            if c_mod == modality and c_node in p_set:
                coarse_del.append(ck)
            elif modality == "proprioceptive" and c_proprio in p_set:
                coarse_del.append(ck)
        for ck in coarse_del:
            del self._coarse_table[ck]

        # Clean cache
        if modality in self._prev_winner:
            if self._prev_winner[modality] in p_set:
                del self._prev_winner[modality]

        if self._last_active_key and self._last_active_key in to_del:
            self._last_active_key = None

        # Also purge stale EFE model entries referencing dead nodes
        is_proprio = (modality == "proprioceptive")
        if self._valence_map is not None:
            self._valence_map.purge_nodes(p_set, is_proprio)
        if self._transition_model is not None:
            self._transition_model.purge_nodes(p_set, is_proprio)

        if to_del or protected:
            print(f"[ActionDecoder] Purged {len(to_del)} Hebbian entries in {modality}"
                  f"{f' (protected {protected} reinforced)' if protected else ''}")

    def save_table(self, path: str) -> None:
        """Persist Hebbian table to disk for cross-session accumulation."""
        import pickle, pathlib
        pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "table":        dict(self._table),
            "tle_baseline": self._tle_baseline,
            "hold_dir":     self._hold_dir,
            "probe_count":  self._probe_count,
        }
        with open(path, "wb") as f:
            pickle.dump(payload, f)
        print(f"[ActionDecoder] Saved {len(self._table)} table entries → {path}")

    def load_table(self, path: str) -> None:
        """Restore a previously saved Hebbian table to warm-start this session."""
        import pickle, pathlib
        p = pathlib.Path(path)
        if not p.exists():
            print(f"[ActionDecoder] No checkpoint at {path} — starting fresh")
            return
        with open(path, "rb") as f:
            payload = pickle.load(f)
        self._table       = payload.get("table", {})
        self._tle_baseline = payload.get("tle_baseline", self._tle_baseline)
        self._hold_dir    = payload.get("hold_dir", 0.0)
        self._probe_count = payload.get("probe_count", 0)
        print(f"[ActionDecoder] Loaded {len(self._table)} table entries ← {path}")
