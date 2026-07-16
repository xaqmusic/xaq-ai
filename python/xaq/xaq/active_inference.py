"""
active_inference.py — Expected Free Energy (EFE) components for AMI-Ogma v3.

Implements the three pillars of active inference for embodied action selection:

1. NodeValenceMap     — Emotional coloring of GNG topology states via
                        neurochemical experience (dopamine → valence).

2. ActionTransitionModel — Action-conditioned generative model:
                        P(next_state | current_state, action_bin).
                        The agent's learned model of how its actions
                        affect the world.

3. EFEPolicy          — Expected Free Energy minimization:
                        pragmatic value (move toward high-valence states)
                        + epistemic value (explore uncertain transitions).

Together these replace the old blend of SpatialDecoder (trained on ball_x)
+ Hebbian velocity lookup with a principled active inference loop that
requires zero game telemetry.

References:
    - Friston et al., "Active Inference and Learning" (2016)
    - Der & Martius, "The Playful Machine" (2012)
    - AMI-Ogma docs/the_playful_machine_principles.md
"""

import math
import numpy as np
from collections import defaultdict, deque
from typing import Dict, Tuple, Optional

from xaq_core.rng import derive_rng

# ---------------------------------------------------------------------------
# Action binning
# ---------------------------------------------------------------------------
ACTION_LEFT  = -1
ACTION_IDLE  =  0
ACTION_RIGHT = +1
ACTION_BINS  = (ACTION_LEFT, ACTION_IDLE, ACTION_RIGHT)

def bin_action(force: float, threshold: float = 0.5) -> int:
    """Map continuous force to discrete action bin."""
    if force < -threshold:
        return ACTION_LEFT
    elif force > threshold:
        return ACTION_RIGHT
    return ACTION_IDLE

def action_to_force(action_bin: int, magnitude: float = 4.0) -> float:
    """Convert discrete action bin back to continuous force."""
    return action_bin * magnitude


# =========================================================================
# 1. Node Valence Map
# =========================================================================

class NodeValenceMap:
    """
    Emotional coloring of the GNG topology.

    Every (visual_node, proprio_node) pair accumulates a valence score
    from neurochemical signals. This creates a "preference landscape"
    over the agent's state space — entirely learned from experience,
    no game telemetry required.

    Valence is the dopamine deviation from baseline, smoothed via EMA.
    Positive valence = rewarding state (hits, bricks).
    Negative valence = aversive state (misses, wall-sticking).

    Principle: In active inference, prior preferences determine what the
    agent considers "surprising." A low-valence state is unexpected (the
    agent prefers to be in high-valence states), driving it to act to
    escape. A high-valence state confirms expectations, reinforcing the
    actions that led there.
    """

    def __init__(self,
                 alpha:     float = 0.05,
                 decay_pos: float = 0.99999,
                 decay_neg: float = 0.9999,
                 baseline:  float = 0.2,
                 max_size:  int   = 2000):
        self._alpha    = alpha      # EMA learning rate for valence updates
        self._decay_pos = decay_pos # per-call decay for positive valence (slow — hits are rare signal)
        self._decay_neg = decay_neg # per-call decay for negative valence (fast — misses are noisy)
        self._baseline = baseline   # dopamine baseline (neutral valence = 0)
        self._max_size = max_size   # cap to prevent unbounded growth

        # Core map: (visual_node, proprio_node) → valence ∈ [-1, 1]
        self._map: Dict[Tuple[int, int], float] = {}

        # Proprio-only valence: collapses visual dimension.
        # Captures "which paddle positions tend to lead to good outcomes"
        # — more stable than joint map since proprio nodes change slowly.
        self._proprio_map: Dict[int, float] = {}
        self._proprio_visits: Dict[int, int] = defaultdict(int)

        # Trajectory valence: (trajectory_tuple, proprio_node) → valence.
        # Encodes "when the ball was traveling along this path AND paddle
        # was at this position, was the outcome good?" This is the key
        # predictive signal — same visual node can be good or bad depending
        # on the ball's direction of travel.
        self._trajectory_map: Dict[Tuple[tuple, int], float] = {}
        self._trajectory_visits: Dict[Tuple[tuple, int], int] = defaultdict(int)

        # Visit counts for confidence weighting
        self._visits: Dict[Tuple[int, int], int] = defaultdict(int)

        # Ecological-agency self-tuning: the valence map is its own agent,
        # and its decay rates become functions of the event density it lives
        # in. Rolling 60-tick-per-second-assumed window of (hit_flag, miss_flag)
        # drives event-frequency-scaled half-lives:
        #   +5 positive events per positive half-life
        #   −2 negative events per negative half-life (asymmetry preserved)
        # A fast-event env (Pong) decays fast; a slow-event env (CartPole)
        # decays slow. The same absolute decay numbers no longer need to
        # be hand-picked per env.
        self._event_window_ticks: int = 1800   # ~60s @ 30fps; retuned in on_tick
        self._event_hits:   deque = deque(maxlen=self._event_window_ticks)
        self._event_misses: deque = deque(maxlen=self._event_window_ticks)
        self._adaptive_decay_enabled: bool = True

    def update(self, visual_node: int, proprio_node: int,
               dopamine: float, serotonin: float = 0.65,
               trajectory: Optional[tuple] = None):
        """
        Update valence for current joint state and trajectory.

        Uses a combined signal: dopamine deviation (reward/punishment)
        weighted by inverse serotonin (instability amplifier). Low
        serotonin (pain/aversion) amplifies the signal, making negative
        events register more strongly in the valence map.

        GATED: Only updates when signal exceeds threshold, preventing
        near-baseline ticks from diluting meaningful events.
        """
        if visual_node < 0:
            return

        key = (visual_node, proprio_node)
        self._visits[key] += 1

        # Combined signal: dopamine deviation × instability amplifier
        # Low serotonin (pain) amplifies the signal by up to 3x
        da_signal = dopamine - self._baseline
        instability = 1.0 / (serotonin + 0.3)  # range ~0.77 (stable) to ~3.3 (pain)
        signal = da_signal * instability

        # Gate: only meaningful deviations (catches ~5-10 ticks after events)
        if abs(signal) < 0.03:
            return

        current = self._map.get(key, 0.0)
        # Adaptive alpha: stronger anchors update more slowly (resist
        # erosion). Empty states learn fast (full alpha); a saturated +1.0
        # food anchor decays 20× slower under neutral baseline drift.
        # Mathematically the same EMA, just with rate scaled by node
        # certainty — biological analog of long-term memory consolidation.
        # Without this, a path-blame anchor at +1.0 decays to ~0.36 after
        # 20 revisits, defeating value-iteration propagation in sparse-
        # reward envs. Reactive games are unaffected: their valences
        # rarely sit near saturation, so adaptive_alpha ≈ self._alpha.
        adaptive_alpha = self._alpha * (1.0 - 0.95 * abs(current))
        self._map[key] = current + adaptive_alpha * (signal - current)

        # Also update proprio-only map (position preference)
        self._proprio_visits[proprio_node] += 1
        p_current = self._proprio_map.get(proprio_node, 0.0)
        p_alpha = self._alpha * (1.0 - 0.95 * abs(p_current))
        self._proprio_map[proprio_node] = p_current + p_alpha * (signal - p_current)

        # Also update trajectory valence map (directional prediction)
        if trajectory is not None and len(trajectory) > 0 and trajectory[0] >= 0:
            traj_key = (trajectory, proprio_node)
            self._trajectory_visits[traj_key] += 1
            t_current = self._trajectory_map.get(traj_key, 0.0)
            t_alpha = self._alpha * (1.0 - 0.95 * abs(t_current))
            self._trajectory_map[traj_key] = t_current + t_alpha * (signal - t_current)

        # Cap size: evict lowest-visit entries if over limit
        if len(self._map) > self._max_size:
            # Find the least-visited entry (least informative)
            min_key = min(self._visits, key=self._visits.get)
            if min_key != key:  # don't evict what we just updated
                del self._map[min_key]
                del self._visits[min_key]

        # Cap trajectory map too (can grow large with unique trajectories)
        if len(self._trajectory_map) > self._max_size:
            min_traj_key = min(self._trajectory_visits, key=self._trajectory_visits.get)
            traj_key_now = (trajectory, proprio_node) if trajectory is not None else None
            if min_traj_key != traj_key_now:
                self._trajectory_map.pop(min_traj_key, None)
                self._trajectory_visits.pop(min_traj_key, None)

    def get(self, visual_node: int, proprio_node: int,
            trajectory: Optional[tuple] = None) -> float:
        """
        Get valence for a state with trajectory-aware lookup.

        Lookup cascade (most specific → most general):
          1. Trajectory + proprio (directional + positional — best predictor)
          2. Visual + proprio (positional only — current node)
          3. Proprio-only (position preference — most stable fallback)
        """
        # 1. Trajectory-keyed: "ball traveling A→B→C with paddle at P"
        if trajectory is not None and len(trajectory) > 0 and trajectory[0] >= 0:
            traj_val = self._trajectory_map.get((trajectory, proprio_node), None)
            if traj_val is not None:
                return traj_val

        # 2. Joint (visual, proprio)
        val = self._map.get((visual_node, proprio_node), None)
        if val is not None:
            return val

        # 3. Fallback: proprio-only valence (position preference)
        return self._proprio_map.get(proprio_node, 0.0) * 0.5  # damped

    def get_proprio(self, proprio_node: int) -> float:
        """Get position-only valence for pragmatic decision-making."""
        return self._proprio_map.get(proprio_node, 0.0)

    # ------------------------------------------------------------------
    # Graph value iteration (Bellman backup over GNG topology)
    # ------------------------------------------------------------------
    # The raw valence map stores R(s) — direct reward signal tied to the
    # states the agent actually experienced a pulse in. A maze agent gets
    # reward at one cell in the graph; everywhere else R=0 (or negative
    # from wall bumps). Without propagation, only that one cell "tastes
    # good" — no gradient exists for the EFE to climb.
    #
    # Value iteration treats the GNG + transition model as an MDP:
    #   V(s) = R(s) + γ · max_a Σ_s' P(s'|s,a) · V(s')
    # Seeding V=R and iterating K sweeps gives every node a value equal to
    # the discounted return of its best learned policy. One eat now paints
    # a gradient across the entire reachable subgraph.
    #
    # State = (visual_node, proprio_node). Transitions use the single-node
    # keyed distribution (dense; trajectory-keyed stays for raw lookup).
    # ------------------------------------------------------------------

    def __post_init_value_iteration__(self):
        """Lazy field init — called from compute_value_map if not set yet."""
        if not hasattr(self, "_value_map"):
            self._value_map: Dict[Tuple[int, int], float] = {}
            self._vi_last_tick: int = -10**9
            self._vi_gamma: float = 0.95
            self._vi_sweeps: int = 10
            self._vi_period: int = 200   # refresh every ~10s at 20Hz
            self._vi_clip:   float = 5.0
            self._vi_enabled: bool = True
            self._vi_compute_count: int = 0

    def configure_value_iteration(self, *, gamma: float = None,
                                   sweeps: int = None, period: int = None,
                                   enabled: bool = None):
        """Tune VI from the runner if needed. None = leave unchanged."""
        self.__post_init_value_iteration__()
        if gamma   is not None: self._vi_gamma   = float(gamma)
        if sweeps  is not None: self._vi_sweeps  = int(sweeps)
        if period  is not None: self._vi_period  = int(period)
        if enabled is not None: self._vi_enabled = bool(enabled)

    def compute_value_map(self, transition_model, tick: int = 0,
                          force: bool = False) -> Dict[Tuple[int, int], float]:
        """Refresh V(s) via Bellman sweeps. Throttled to once per
        `_vi_period` ticks unless `force=True`. Returns current V map."""
        self.__post_init_value_iteration__()
        if not self._vi_enabled:
            return self._value_map
        if not force and (tick - self._vi_last_tick) < self._vi_period:
            return self._value_map
        self._vi_last_tick = tick

        # Build cached P(s'|s,a) from single-keyed transitions — dense path.
        # Each transition carries a confidence weight = min(1, count/sat),
        # where `sat` is the saturation threshold (autotuned). Singleton
        # transitions (count=1) would otherwise propagate P=1.0 as certainty
        # — confidence weighting damps their Bellman contribution until more
        # observations accumulate. Autotune: `sat` grows with graph maturity
        # so cold-start trusts every obs (sat≈1) while a well-explored graph
        # demands multiple obs before full trust.
        _tot = max(1, transition_model.total_transitions)
        conf_sat = float(max(1.0, min(5.0, math.sqrt(_tot / 500.0))))
        trans_cache: Dict[Tuple[int, int, int],
                          Tuple[Dict[Tuple[int, int], float], float]] = {}
        states: set = set(self._map.keys())
        for key, out_dist in transition_model._transitions.items():
            traj_or_single, p, a = key
            if len(traj_or_single) != 1:
                continue
            v = traj_or_single[0]
            states.add((v, p))
            for nxt in out_dist:
                states.add(nxt)
            total = sum(out_dist.values())
            if total > 0:
                confidence = min(1.0, total / conf_sat)
                trans_cache[(v, p, a)] = (
                    {nxt: c / total for nxt, c in out_dist.items()},
                    confidence,
                )

        if len(states) < 10:
            # Too little structure to propagate meaningfully.
            self._value_map = {s: self._map.get(s, 0.0) for s in states}
            return self._value_map

        # Initialize V = R (raw valence) for all seen states; unseen = 0.
        V: Dict[Tuple[int, int], float] = {
            s: float(self._map.get(s, 0.0)) for s in states
        }

        gamma = self._vi_gamma
        clip  = self._vi_clip
        for _sweep in range(self._vi_sweeps):
            V_new: Dict[Tuple[int, int], float] = {}
            for s in states:
                v, p = s
                best_q = None
                for a in ACTION_BINS:
                    cached = trans_cache.get((v, p, a))
                    if not cached:
                        continue
                    dist, confidence = cached
                    q = 0.0
                    for nxt, prob in dist.items():
                        q += prob * V.get(nxt, 0.0)
                    q *= confidence
                    if best_q is None or q > best_q:
                        best_q = q
                R_s = self._map.get(s, 0.0)
                if best_q is None:
                    V_new[s] = R_s
                else:
                    v_new = R_s + gamma * best_q
                    # Clip to bound asymptotic growth from positive cycles.
                    if   v_new >  clip: v_new =  clip
                    elif v_new < -clip: v_new = -clip
                    V_new[s] = v_new
            V = V_new

        self._value_map = V
        self._vi_compute_count += 1
        return V

    def get_value(self, visual_node: int, proprio_node: int,
                  trajectory: Optional[tuple] = None) -> float:
        """Lookup cascade that prefers propagated V(s) over raw R(s):
           1. Trajectory-keyed (directional refinement, unchanged)
           2. V(joint)  — Bellman-propagated joint valence (NEW)
           3. Raw joint valence
           4. Proprio-only fallback (damped)"""
        self.__post_init_value_iteration__()
        if trajectory is not None and len(trajectory) > 0 and trajectory[0] >= 0:
            tv = self._trajectory_map.get((trajectory, proprio_node), None)
            if tv is not None:
                return tv
        vv = self._value_map.get((visual_node, proprio_node), None)
        if vv is not None:
            return vv
        rv = self._map.get((visual_node, proprio_node), None)
        if rv is not None:
            return rv
        return self._proprio_map.get(proprio_node, 0.0) * 0.5

    def value_stats(self) -> dict:
        """Telemetry for value-iteration diagnostics."""
        self.__post_init_value_iteration__()
        if not self._value_map:
            return {"vi_states": 0, "vi_pos": 0, "vi_neg": 0,
                    "vi_max": 0.0, "vi_min": 0.0, "vi_computes": 0}
        vals = list(self._value_map.values())
        pos  = sum(1 for x in vals if x >  0.01)
        neg  = sum(1 for x in vals if x < -0.01)
        return {
            "vi_states":   len(vals),
            "vi_pos":      pos,
            "vi_neg":      neg,
            "vi_max":      round(max(vals), 3),
            "vi_min":      round(min(vals), 3),
            "vi_computes": self._vi_compute_count,
        }

    def confidence(self, visual_node: int, proprio_node: int) -> float:
        """How many times have we visited this state? Normalized to [0, 1]."""
        visits = self._visits.get((visual_node, proprio_node), 0)
        return min(1.0, visits / 50.0)  # saturates after 50 visits

    def on_tick(self, hit: bool, miss: bool, fps: int = 30) -> None:
        """Record per-tick event flags; retune decay rates from the 60s rate.
        Called from the runner once per tick. The map adapts its own decay
        to the env's event ecology — no external scheduler."""
        self._event_window_ticks = max(60, 60 * int(fps))
        if len(self._event_hits) != self._event_window_ticks:
            # Resize windows if fps changed
            self._event_hits   = deque(self._event_hits,   maxlen=self._event_window_ticks)
            self._event_misses = deque(self._event_misses, maxlen=self._event_window_ticks)
        self._event_hits.append(1.0 if hit else 0.0)
        self._event_misses.append(1.0 if miss else 0.0)
        if not self._adaptive_decay_enabled:
            return
        n = len(self._event_hits)
        if n < fps * 5:   # warmup: need ≥5s of data
            return
        pos_rate = sum(self._event_hits)   / n    # events / tick
        neg_rate = sum(self._event_misses) / n
        # Half-life in ticks = N_events_until_half / rate. Floor rate to
        # avoid div-zero when the env is silent.
        hl_pos = 5.0 / max(pos_rate, 1e-5)
        hl_neg = 2.0 / max(neg_rate, 1e-5)
        # Clamp half-lives to sane bounds (2s … 2h at fps).
        hl_pos = float(np.clip(hl_pos, 2 * fps, 7200 * fps))
        hl_neg = float(np.clip(hl_neg, 2 * fps, 7200 * fps))
        self._decay_pos = math.exp(-math.log(2.0) / hl_pos)
        self._decay_neg = math.exp(-math.log(2.0) / hl_neg)

    def decay_rates(self) -> Tuple[float, float]:
        return (self._decay_pos, self._decay_neg)

    def decay_all(self):
        """
        Apply asymmetric global decay toward neutral.

        Negative valence (from misses) decays faster than positive (from hits).
        Rationale: misses are ~3x more frequent than hits, so negative signal
        is noisy and should fade quickly. Hits are rare and informative — their
        positive valence should persist longer to guide exploitation.

        Called every tick. Half-lives at 30fps:
          Positive: 0.99999 → ~38 min half-life (persistent — hits are rare signal)
          Negative: 0.9999  → ~3.9 min half-life (fades within session — misses are noise)
        """
        for key in self._map:
            v = self._map[key]
            if v >= 0:
                self._map[key] = v * self._decay_pos
            else:
                self._map[key] = v * self._decay_neg
        # Proprio-only map: same asymmetric decay
        for key in self._proprio_map:
            v = self._proprio_map[key]
            if v >= 0:
                self._proprio_map[key] = v * self._decay_pos
            else:
                self._proprio_map[key] = v * self._decay_neg
        # Trajectory map: same asymmetric decay
        for key in self._trajectory_map:
            v = self._trajectory_map[key]
            if v >= 0:
                self._trajectory_map[key] = v * self._decay_pos
            else:
                self._trajectory_map[key] = v * self._decay_neg

    def purge_nodes(self, pruned_ids: set, is_proprio: bool = False):
        """Remove valence entries referencing pruned GNG nodes."""
        to_del = []
        for (v_node, p_node) in self._map:
            if is_proprio and p_node in pruned_ids:
                to_del.append((v_node, p_node))
            elif not is_proprio and v_node in pruned_ids:
                to_del.append((v_node, p_node))
        for k in to_del:
            del self._map[k]
            self._visits.pop(k, None)

    @property
    def size(self) -> int:
        return len(self._map)

    def stats(self) -> dict:
        if not self._map:
            base = {"size": 0, "mean_valence": 0.0, "max_valence": 0.0,
                    "min_valence": 0.0, "positive_states": 0, "negative_states": 0,
                    "trajectory_states": 0}
        else:
            vals = list(self._map.values())
            base = {
                "size":              len(vals),
                "mean_valence":      round(float(np.mean(vals)), 4),
                "max_valence":       round(float(max(vals)), 4),
                "min_valence":       round(float(min(vals)), 4),
                "positive_states":   sum(1 for v in vals if v > 0.01),
                "negative_states":   sum(1 for v in vals if v < -0.01),
                "trajectory_states": len(self._trajectory_map),
                "decay_pos":         round(float(self._decay_pos), 6),
                "decay_neg":         round(float(self._decay_neg), 6),
            }
        # Merge VI telemetry (Bellman-propagated V-map) alongside raw R-map.
        base.update(self.value_stats())
        return base


# =========================================================================
# 2. Action-Conditioned Transition Model
# =========================================================================

class ActionTransitionModel:
    """
    The agent's generative model: P(next_state | current_state, action).

    This is the core of active inference — the agent can mentally simulate
    "what would happen if I moved left?" vs "what would happen if I moved
    right?" and choose the action with the best predicted outcome.

    State is the joint (visual_node, proprio_node). Actions are discretized
    into LEFT / IDLE / RIGHT bins.

    Principle: Active inference requires a generative model that can be
    "run forward" to predict consequences. Without this, the agent can
    only react to what already happened (the old Hebbian approach). With
    it, the agent can plan — even if just one step ahead.

    The model also provides EPISTEMIC VALUE: transitions with high entropy
    (many possible outcomes) are uncertain and worth exploring. This is
    how curiosity emerges from the math rather than being hard-coded.
    """

    def __init__(self, max_entries: int = 5000):
        self._max_entries = max_entries

        # Trajectory-keyed transitions:
        #   (trajectory_tuple, proprio, action_bin) → {(next_visual, next_proprio): count}
        # When no trajectory is available, falls back to single-node keying:
        #   ((visual,), proprio, action_bin) → ...
        self._transitions: Dict[tuple, Dict[Tuple[int, int], int]] = \
            defaultdict(lambda: defaultdict(int))

        # 1-tick delay buffer: store current state for next-tick recording
        self._prev_visual:  int = -1
        self._prev_proprio: int = -1
        self._prev_action:  int = ACTION_IDLE
        self._prev_trajectory: Optional[tuple] = None

        self.total_transitions = 0

        # Top-1 predictor accuracy, EMA'd. On each record(), we check whether
        # the model's top-1 prediction for the previous (state, action) matches
        # what actually happened. Seeded at 0.5 (uninformative) so early
        # horizon-gating defaults to ~mid until evidence arrives.
        self._pred_accuracy_ema: float = 0.5
        self._pred_accuracy_alpha: float = 0.02   # ~50-tick half-life
        self._pred_accuracy_samples: int = 0

    @staticmethod
    def _make_key(trajectory: Optional[tuple], visual: int,
                  proprio: int, action_bin: int) -> tuple:
        """Build a transition key from trajectory (preferred) or single node."""
        if trajectory is not None and len(trajectory) > 0 and trajectory[0] >= 0:
            return (trajectory, proprio, action_bin)
        return ((visual,), proprio, action_bin)

    def record(self, visual_node: int, proprio_node: int, action_force: float,
               trajectory: Optional[tuple] = None):
        """
        Record a transition. Called each tick with the CURRENT state.
        The transition is from (prev_state, prev_action) → current_state.

        When trajectory is provided, the key encodes the sequence of recent
        visual nodes — the ball's direction of travel through the GNG
        topology. This gives the model predictive power about WHERE the ball
        is heading, not just where it currently is.
        """
        if self._prev_visual < 0 or visual_node < 0:
            self._prev_visual  = visual_node
            self._prev_proprio = proprio_node
            self._prev_action  = bin_action(action_force)
            self._prev_trajectory = trajectory
            return

        # Record transition under BOTH trajectory key AND single-node key.
        # The single-node key accumulates density fast (always fires).
        # The trajectory key provides directional refinement when it has data.
        to_state = (visual_node, proprio_node)

        # Predictor top-1 accuracy: before updating counts, ask "what would I
        # have predicted for the previous (state, action)?" and compare to the
        # observed `to_state`. EMA tracks compounding-rollout trustworthiness.
        prev_dist = self.predict(self._prev_visual, self._prev_proprio,
                                 self._prev_action,
                                 trajectory=self._prev_trajectory)
        if prev_dist:
            top1 = max(prev_dist.items(), key=lambda kv: kv[1])[0]
            hit = 1.0 if top1 == to_state else 0.0
            a = self._pred_accuracy_alpha
            self._pred_accuracy_ema = (1.0 - a) * self._pred_accuracy_ema + a * hit
            self._pred_accuracy_samples += 1

        # Always record single-node (guaranteed density)
        single_key = ((self._prev_visual,), self._prev_proprio, self._prev_action)
        self._transitions[single_key][to_state] += 1
        self.total_transitions += 1

        # Also record trajectory-keyed if we have a valid trajectory
        if (self._prev_trajectory is not None and
                len(self._prev_trajectory) > 1 and self._prev_trajectory[0] >= 0):
            traj_key = (self._prev_trajectory, self._prev_proprio, self._prev_action)
            self._transitions[traj_key][to_state] += 1

        # Update delay buffer for next tick
        self._prev_visual  = visual_node
        self._prev_proprio = proprio_node
        self._prev_action  = bin_action(action_force)
        self._prev_trajectory = trajectory

        # Size management: if too many unique from_keys, prune least-used
        if len(self._transitions) > self._max_entries:
            min_key = min(self._transitions,
                          key=lambda k: sum(self._transitions[k].values()))
            del self._transitions[min_key]

    def predict(self, visual: int, proprio: int, action_bin: int,
                trajectory: Optional[tuple] = None
                ) -> Dict[Tuple[int, int], float]:
        """
        Predict next-state distribution: P(next | current, action).
        Returns {(next_visual, next_proprio): probability}.

        Blends trajectory-keyed prediction (directional, sparse) with
        single-node prediction (positional, dense). When trajectory data
        exists, it gets higher weight (more specific = more informative).
        When only single-node data exists, uses that alone.
        """
        single_key = ((visual,), proprio, action_bin)
        single_counts = self._transitions.get(single_key)

        traj_counts = None
        if trajectory is not None and len(trajectory) > 1 and trajectory[0] >= 0:
            traj_key = (trajectory, proprio, action_bin)
            traj_counts = self._transitions.get(traj_key)

        if not single_counts and not traj_counts:
            return {}

        # If only one source has data, use it directly
        if traj_counts and not single_counts:
            total = sum(traj_counts.values())
            return {s: c / total for s, c in traj_counts.items()}
        if single_counts and not traj_counts:
            total = sum(single_counts.values())
            return {s: c / total for s, c in single_counts.items()}

        # Both exist: blend. Trajectory gets higher weight when it has
        # enough observations (more specific = more trustworthy).
        traj_total = sum(traj_counts.values())
        single_total = sum(single_counts.values())

        # Trajectory confidence: saturates at 10 observations
        traj_confidence = min(1.0, traj_total / 10.0)
        # Blend weight: trajectory gets up to 70% weight when confident
        traj_weight = 0.7 * traj_confidence
        single_weight = 1.0 - traj_weight

        blended: Dict[Tuple[int, int], float] = defaultdict(float)
        for s, c in traj_counts.items():
            blended[s] += traj_weight * (c / traj_total)
        for s, c in single_counts.items():
            blended[s] += single_weight * (c / single_total)

        return dict(blended)

    def rollout(self, visual: int, proprio: int, action_bin: int,
                n_steps: int,
                trajectory: Optional[tuple] = None) -> list:
        """Unroll the 1-step transition model N ticks into the future under
        a constant action. Uses argmax at each step (picks the most likely
        next state and feeds it back in). Returns a list of
        (visual, proprio, cumul_prob) triples — one per simulated tick,
        in chronological order. Empty list if the model has no data.

        This is the "where will I be if I commit to this action?" question
        the EFE policy asks when its horizon grows. Argmax is cheap and the
        compounded prediction error naturally discounts deep rollouts into
        unreliable territory — consumers should still apply a γ^k discount
        to the valence of each step, which is the shape the risk-guard
        relies on."""
        if n_steps <= 0:
            return []
        cur_v, cur_p = int(visual), int(proprio)
        cur_traj = trajectory
        cumul = 1.0
        out: list = []
        for k in range(int(n_steps)):
            dist = self.predict(cur_v, cur_p, action_bin, trajectory=cur_traj)
            if not dist:
                break
            (nxt_v, nxt_p), p = max(dist.items(), key=lambda kv: kv[1])
            cumul *= float(p)
            out.append((nxt_v, nxt_p, cumul))
            cur_v, cur_p = nxt_v, nxt_p
            # Trajectory is observation-only and can't be extended through
            # a rollout (we don't know the future visual sequence), so fall
            # back to single-node keying past step 0.
            cur_traj = None
        return out

    def entropy(self, visual: int, proprio: int, action_bin: int,
                trajectory: Optional[tuple] = None) -> float:
        """
        Shannon entropy of transition distribution.

        High entropy = many possible outcomes = high uncertainty = high
        epistemic value (worth exploring to reduce uncertainty).
        Low entropy = predictable outcome = low epistemic value.

        This is the formal measure of "curiosity" in active inference.
        """
        dist = self.predict(visual, proprio, action_bin, trajectory=trajectory)
        if not dist:
            # No data → uncertain, but not overwhelmingly so.
            # Must be comparable in scale to pragmatic values (~0.01-0.1)
            # so that the EFE balance works. Too high (e.g., 2.0) and the
            # policy becomes pure random exploration regardless of valence.
            return 0.5  # moderate epistemic prior for unexplored transitions

        h = 0.0
        for p in dist.values():
            if p > 0:
                h -= p * np.log2(p + 1e-10)
        return float(h)

    def has_data(self, visual: int, proprio: int,
                 trajectory: Optional[tuple] = None) -> bool:
        """Does the model have any transition data for this state?"""
        return any(
            self._make_key(trajectory, visual, proprio, a) in self._transitions
            for a in ACTION_BINS
        )

    def data_density(self, visual: int, proprio: int,
                     trajectory: Optional[tuple] = None) -> int:
        """Total observations across all actions for this state."""
        total = 0
        for a in ACTION_BINS:
            key = self._make_key(trajectory, visual, proprio, a)
            counts = self._transitions.get(key)
            if counts:
                total += sum(counts.values())
            # Also count fallback single-node entries
            if trajectory is not None and len(trajectory) > 1:
                fallback = ((visual,), proprio, a)
                fb_counts = self._transitions.get(fallback)
                if fb_counts:
                    total += sum(fb_counts.values())
        return total

    def purge_nodes(self, pruned_ids: set, is_proprio: bool = False):
        """Remove transition entries referencing pruned GNG nodes.

        Keys are now (trajectory_or_singleton, proprio, action_bin) where
        trajectory_or_singleton is a tuple of node IDs. We check each node
        in the trajectory tuple against pruned_ids.
        """
        to_del = []
        for key in self._transitions:
            traj_or_single, p, a = key
            if is_proprio and p in pruned_ids:
                to_del.append(key)
            elif not is_proprio:
                # Check if any node in the trajectory is pruned
                if any(nid in pruned_ids for nid in traj_or_single):
                    to_del.append(key)
        for k in to_del:
            del self._transitions[k]
        # Also clean destination states inside remaining entries
        for key, dests in list(self._transitions.items()):
            dead = [d for d in dests
                    if (is_proprio and d[1] in pruned_ids)
                    or (not is_proprio and d[0] in pruned_ids)]
            for d in dead:
                del dests[d]
            if not dests:
                del self._transitions[key]
        # Reset delay buffer if it references a pruned node
        if not is_proprio and self._prev_visual in pruned_ids:
            self._prev_visual = -1
        if is_proprio and self._prev_proprio in pruned_ids:
            self._prev_proprio = -1

    def stats(self) -> dict:
        n_entries = len(self._transitions)
        if n_entries == 0:
            return {"entries": 0, "total_transitions": 0, "mean_entropy": 0.0,
                    "trajectory_keyed": 0, "single_keyed": 0}

        entropies = []
        traj_keyed = 0
        single_keyed = 0
        # Snapshot the items to avoid concurrent-mutation from the decoder
        # thread updating transitions while the UI reads stats().
        for key, counts in list(self._transitions.items()):
            traj_or_single, p, a = key
            if len(traj_or_single) > 1:
                traj_keyed += 1
            else:
                single_keyed += 1
            total = sum(counts.values())
            h = 0.0
            for c in counts.values():
                prob = c / total
                if prob > 0:
                    h -= prob * np.log2(prob + 1e-10)
            entropies.append(h)

        return {
            "entries":            n_entries,
            "total_transitions":  self.total_transitions,
            "mean_entropy":       round(float(np.mean(entropies)), 4),
            "trajectory_keyed":   traj_keyed,
            "single_keyed":       single_keyed,
            "predictor_accuracy": round(self._pred_accuracy_ema, 4),
            "predictor_samples":  self._pred_accuracy_samples,
        }


# =========================================================================
# 3. Expected Free Energy (EFE) Policy
# =========================================================================

class EFEPolicy:
    """
    Action selection by minimizing Expected Free Energy.

    For each possible action (LEFT, IDLE, RIGHT), computes:
      EFE(a) = pragmatic_value(a) + w_epistemic * epistemic_value(a)

    Pragmatic value: expected valence of predicted next state.
      "Will this action lead me somewhere good?"

    Epistemic value: entropy of the transition distribution.
      "Will this action teach me something new?"

    The balance between pragmatic and epistemic is controlled by
    exploration_weight, which is derived from the agent's neurochemical
    state:
      - Low TLE (bored) → high exploration_weight → seek novelty
      - High TLE (surprised) → low exploration_weight → exploit/focus
      - Low serotonin (pain) → moderate exploration → escape current state

    This is the formal implementation of The Playful Machine principles:
    boredom drives exploration, surprise drives exploitation, and the
    agent naturally oscillates between them based on its experience.

    Softmax temperature controls action stochasticity: high temperature
    makes the policy more random (early learning), low temperature makes
    it more deterministic (late, confident).
    """

    def __init__(self,
                 base_exploration: float = 1.0,
                 pragmatic_gain:   float = 10.0,
                 temperature:      float = 1.0,
                 force_magnitude:   float = 4.0,
                 horizon_max:      int   = 10,
                 gamma:            float = 0.9,
                 master_seed:      int   = 0):
        self._base_exploration = base_exploration
        self._pragmatic_gain   = pragmatic_gain  # amplify valence signal to match epistemic scale
        self._temperature      = temperature
        self._force_magnitude  = force_magnitude
        self._rng              = derive_rng(master_seed, "efe.policy")

        # Adaptive rollout horizon — self-tuned from the NodePathBlame's
        # inter-event cadence (same ecological signal that drives blame
        # depth). CartPole/Pong land at 1 (reactive); maze ramps toward
        # horizon_max as credit horizon deepens. Seeded at 1 — reactive
        # until an event cadence is measured, at which point the policy
        # starts asking "where am I if I commit to this action for N ticks?"
        self._horizon       = 1
        self._horizon_max   = int(horizon_max)
        self._gamma         = float(gamma)
        self._path_blame_ref = None

        # Telemetry: track per-action EFE components
        self._last_efe = {a: 0.0 for a in ACTION_BINS}
        self._last_pragmatic = {a: 0.0 for a in ACTION_BINS}
        self._last_epistemic = {a: 0.0 for a in ACTION_BINS}
        self._last_action_probs = {a: 1/3 for a in ACTION_BINS}
        # EMA of pragmatic L-R differentiation (how directional is the signal?)
        self._pragmatic_diff_ema = 0.0
        # Count how often transition model has trajectory-keyed predictions
        self._traj_predict_hits = 0
        self._traj_predict_total = 0
        # Tick counter — monotone across select_action calls. Drives the
        # value-iteration throttle (refresh once per `_vi_period` calls).
        self._efe_call_count = 0

        # Latest V-map maturity (V_max / clip_cap). Published each
        # select_action() so the ActionDecoder can modulate drought-driven
        # exploration when the gradient is strong.
        self._last_v_maturity = 0.0

    def bind_path_blame(self, path_blame) -> None:
        """Wire the NodePathBlame whose inter-event cadence drives the
        rollout horizon. Called once during setup."""
        self._path_blame_ref = path_blame

    def _sync_horizon(self, transition_model: Optional["ActionTransitionModel"] = None) -> None:
        """Re-derive self._horizon from the bound NodePathBlame.path_length,
        then cap by the transition model's top-1 predictor accuracy.

        Rationale: rolling out N steps on a 3%-accurate predictor compounds
        hallucination. The cadence-derived horizon sets the *ceiling* (how
        far blame reaches), but the *effective* horizon is only as deep as
        the model is trustworthy:

            effective = min(cadence_horizon, ceil(horizon_max * pred_acc))

        Seeded-predictor regime (samples < 100) pegs the cap at the cadence
        horizon — we don't punish a fresh model for having no track record.
        Once it has data, low accuracy clamps horizon to 1 (reactive), high
        accuracy unlocks full lookahead."""
        pb = self._path_blame_ref
        if pb is None:
            return
        pl = int(getattr(pb, "path_length", self._horizon))
        cadence_h = max(1, min(self._horizon_max, pl // 50))

        # Quality gate: only applies once the predictor has enough samples
        # to have a meaningful accuracy estimate.
        if (transition_model is not None and
                getattr(transition_model, "_pred_accuracy_samples", 0) >= 100):
            acc = float(getattr(transition_model, "_pred_accuracy_ema", 0.5))
            quality_cap = max(1, int(math.ceil(self._horizon_max * acc)))
            new_h = min(cadence_h, quality_cap)
        else:
            new_h = cadence_h
        self._horizon = new_h

    def select_action(self,
                      visual_node:   int,
                      proprio_node:  int,
                      valence_map:   NodeValenceMap,
                      transition_model: ActionTransitionModel,
                      tle:           float = 0.0,
                      serotonin:     float = 0.5,
                      trajectory:    Optional[tuple] = None,
                      ) -> float:
        """
        Select motor force via EFE minimization.

        When trajectory is provided, the transition model uses the sequence
        of recent visual nodes (ball's path through GNG topology) to predict
        FUTURE states — not just current position. This is the key insight:
        the same visual_node can lead to very different outcomes depending on
        the trajectory that led there (ball moving left vs right).

        Returns continuous force in [-force_magnitude, +force_magnitude].
        """
        if visual_node < 0:
            # No valid visual state — pure random exploration
            return (self._rng.random() * 2.0 - 1.0) * self._force_magnitude * 0.5

        # Pull the current self-tuned horizon from the bound NodePathBlame.
        # Cheap: just reads an int. Done once per action, not per bin.
        # Pass the transition model so pred_acc can clamp the horizon.
        self._sync_horizon(transition_model)

        # Refresh V(s) via Bellman sweeps over the GNG topology.
        # Throttled internally — runs once per ~10s of ticks. One positive
        # reward (an eat) propagates discounted value across every reachable
        # node, giving the pragmatic term a spatial gradient to climb even
        # when the raw valence map has only a single hot state.
        self._efe_call_count += 1
        valence_map.compute_value_map(transition_model,
                                      tick=self._efe_call_count)

        # Exploration weight: high when bored (low TLE), low when surprised.
        # This is the Playful Machine inversion: predictability → exploration.
        boredom = max(0.0, 1.0 - tle * 2.0)
        # Pain (low serotonin) also drives exploration — escape the bad state
        pain_drive = max(0.0, 0.5 - serotonin)
        exploration_weight = self._base_exploration * (0.3 + 0.7 * boredom + pain_drive)

        # Adaptive pragmatic gain: scales with valence map maturity.
        # Signal is V_max relative to VI's own clip cap. Only positive
        # propagation counts — the exploit signal should reflect reward-
        # seeking gradient strength, not avoidance alone. A weak V(s)=[-1,+1]
        # landscape distinguishes itself from a strong V(s)=[-1,+5] one,
        # which range-based maturity couldn't do. Autotunes against the
        # bound the system itself sets; no hardcoded threshold.
        vm = getattr(valence_map, "_value_map", None) or valence_map._map
        if vm:
            v_max_m = max(vm.values())
            clip_cap = float(getattr(valence_map, "_vi_clip", 5.0))
            maturity = min(1.0, max(0.0, v_max_m) / max(1e-6, clip_cap))
        else:
            maturity = 0.0
        adaptive_gain = self._pragmatic_gain * (0.3 + 1.7 * maturity)
        self._last_v_maturity = float(maturity)

        efe_scores = {}

        for action_bin in ACTION_BINS:
            # --- Pragmatic value ---
            # Two-tier scoring:
            #   Tier 1: Trajectory-proprio pragmatic — "given the ball's
            #           direction, which paddle position has best outcomes?"
            #           This is the KEY differentiator between L/I/R.
            #   Tier 2: Full predicted state valence (when transition model
            #           has data). This adds refinement but won't differentiate
            #           actions much since visual predictions are action-independent.
            predicted = transition_model.predict(
                visual_node, proprio_node, action_bin, trajectory=trajectory)
            self._traj_predict_total += 1

            # Tier 1: Proprio-marginalized scoring from predicted transitions.
            # Marginalize out visual: Σ_v P(v, p|a) * V_traj(trajectory, p)
            # This scores "what proprio position will this action put me in,
            # and does that position align with the ball's trajectory?"
            if predicted:
                self._traj_predict_hits += 1
                # Marginalize to proprio: accumulate P(proprio_p) across visual
                proprio_dist: Dict[int, float] = defaultdict(float)
                for (nv, np_node), prob in predicted.items():
                    proprio_dist[np_node] += prob

                # Score each predicted proprio position against trajectory valence
                traj_pragmatic = sum(
                    prob * valence_map.get(visual_node, pp, trajectory=trajectory)
                    for pp, prob in proprio_dist.items()
                )
                # Also include full joint valence for refinement (lower weight).
                # Uses get_value() — prefers Bellman-propagated V over raw R so
                # the pragmatic score reflects long-range reachability of reward,
                # not just which 1-step neighbors happen to be hot.
                full_pragmatic = sum(
                    prob * valence_map.get_value(nv, np_node, trajectory=trajectory)
                    for (nv, np_node), prob in predicted.items()
                )
                pragmatic = adaptive_gain * (0.7 * traj_pragmatic + 0.3 * full_pragmatic)

                # Multi-step rollout pragmatic bonus (Adaptation 2).
                # When horizon > 1, unroll the model under this action and
                # γ-discount the valence of each predicted future state. The
                # existing 1-step score above stays as the dominant term; this
                # only adds "and where does this action keep leading?" signal,
                # which is what sparse-reward envs like the maze need. Weight
                # scales with cumulative transition probability so unreliable
                # deep rollouts fade on their own.
                if self._horizon > 1:
                    future = transition_model.rollout(
                        visual_node, proprio_node, action_bin,
                        n_steps=self._horizon, trajectory=trajectory)
                    rollout_bonus = 0.0
                    for k, (fv, fp, cum_prob) in enumerate(future, start=1):
                        gamma_k = self._gamma ** k
                        rollout_bonus += gamma_k * cum_prob * valence_map.get_value(
                            fv, fp, trajectory=trajectory)
                    # Blend the rollout bonus as a soft additive signal: the
                    # 1-step score stays authoritative, multi-step only breaks
                    # ties when immediate valence is flat.
                    pragmatic += adaptive_gain * 0.5 * rollout_bonus
            else:
                # No transition data — fall back to proprio-only valence
                pragmatic = adaptive_gain * 0.3 * valence_map.get_proprio(proprio_node)

            # --- Epistemic value: information gain from this action ---
            epistemic = transition_model.entropy(
                visual_node, proprio_node, action_bin, trajectory=trajectory)

            efe = pragmatic + exploration_weight * epistemic
            efe_scores[action_bin] = efe

            # Telemetry
            self._last_pragmatic[action_bin] = pragmatic
            self._last_epistemic[action_bin] = epistemic
            self._last_efe[action_bin] = efe

        # Track pragmatic differentiation: how different are L vs R?
        prag_diff = abs(self._last_pragmatic.get(ACTION_LEFT, 0) -
                        self._last_pragmatic.get(ACTION_RIGHT, 0))
        self._pragmatic_diff_ema = 0.99 * self._pragmatic_diff_ema + 0.01 * prag_diff

        # Softmax action selection (stochastic to maintain exploration)
        actions = list(efe_scores.keys())
        scores = np.array([efe_scores[a] for a in actions])

        # Temperature-scaled softmax
        # Avoid overflow: subtract max before exp
        scores_scaled = scores / max(0.1, self._temperature)
        scores_scaled -= scores_scaled.max()
        exp_scores = np.exp(scores_scaled)
        probs = exp_scores / (exp_scores.sum() + 1e-10)

        # Store probabilities for telemetry
        for i, a in enumerate(actions):
            self._last_action_probs[a] = float(probs[i])

        # Sample action
        chosen_idx = self._rng.choice(len(actions), p=probs)
        chosen_action = actions[chosen_idx]

        # Convert to continuous force with slight noise for motor diversity
        force = action_to_force(chosen_action, self._force_magnitude)
        # Add small continuous noise (±10%) for smoother trajectories
        force += self._rng.standard_normal() * self._force_magnitude * 0.1

        return float(np.clip(force, -self._force_magnitude, self._force_magnitude))

    def stats(self) -> dict:
        return {
            "efe_left":      round(self._last_efe.get(ACTION_LEFT, 0), 4),
            "efe_idle":      round(self._last_efe.get(ACTION_IDLE, 0), 4),
            "efe_right":     round(self._last_efe.get(ACTION_RIGHT, 0), 4),
            "pragmatic_left":  round(self._last_pragmatic.get(ACTION_LEFT, 0), 4),
            "pragmatic_right": round(self._last_pragmatic.get(ACTION_RIGHT, 0), 4),
            "epistemic_left":  round(self._last_epistemic.get(ACTION_LEFT, 0), 4),
            "epistemic_right": round(self._last_epistemic.get(ACTION_RIGHT, 0), 4),
            "prob_left":     round(self._last_action_probs.get(ACTION_LEFT, 0), 3),
            "prob_idle":     round(self._last_action_probs.get(ACTION_IDLE, 0), 3),
            "prob_right":    round(self._last_action_probs.get(ACTION_RIGHT, 0), 3),
            "pragmatic_diff": round(self._pragmatic_diff_ema, 6),
            "traj_predict_pct": round(100.0 * self._traj_predict_hits / max(1, self._traj_predict_total), 1),
            "horizon":        int(self._horizon),
            "horizon_max":    int(self._horizon_max),
        }


# =========================================================================
# 4. Node Path Blame — Temporal Credit Assignment
# =========================================================================

class NodePathBlame:
    """
    Tracks the agent's node traversal path and applies temporal credit
    assignment across the full active inference stack when neurochemical
    events occur.

    On a hit: credit the path that led here — strengthen valence, reinforce
    Hebbian entries.
    On a miss/wall-stuck: blame the path — downgrade valence, erode Hebbian
    entries, flag nodes for increased prune priority.

    The blame/credit curve is exponential decay from the current node
    backward through the path:

        signal(k) = magnitude * lambda_decay^k

    where k=0 is the active node (full signal) and k increases going back
    in time. This concentrates attribution on recent states while giving
    diminishing responsibility to earlier ones.

    Biological analogy: dopamine retroactively tags recently-active
    synapses via eligibility traces. Synapses that fired just before
    reward get the strongest reinforcement; those that fired long before
    get a fading echo. Same mechanism, but applied to GNG nodes and
    the entire active inference model rather than just synaptic weights.

    Parameters
    ----------
    path_length     int     max nodes to remember (trace depth)
    lambda_decay    float   per-step decay factor for credit/blame curve
                            0.8 = moderate depth, 0.5 = shallow/concentrated
    credit_scale    float   magnitude multiplier for positive events (hits)
    blame_scale     float   magnitude multiplier for negative events (misses)
    """

    def __init__(self,
                 path_length:   int   = 20,
                 lambda_decay:  float = 0.75,
                 credit_scale:  float = 1.0,
                 blame_scale:   float = 1.5,
                 adaptive:      bool  = True,
                 length_min:    int   = 20,
                 length_max:    int   = 500):
        self._path_length  = path_length
        self._lambda_decay = lambda_decay
        self._credit_scale = credit_scale
        self._blame_scale  = blame_scale

        # Path buffer: list of (visual_node, proprio_node, hebbian_key, action_bin)
        # Most recent entry is at the END (index -1).
        self._path: list = []

        # Ecological-agency self-tuning: the blame horizon is a local loop
        # whose depth should match the event cadence it operates in. A rolling
        # window of ticks-between-events feeds the re-computation on each fire.
        # Sparse events (maze goal-reach) → deep horizon; dense events
        # (CartPole per-streak pulses) → shallow horizon. Seeded with [30] so
        # mean is defined before any event fires.
        self._adaptive           = bool(adaptive)
        self._length_min         = int(length_min)
        self._length_max         = int(length_max)
        self._event_intervals    = deque([30], maxlen=20)
        self._ticks_since_event  = 0

        # Telemetry
        self.total_credit_events = 0
        self.total_blame_events  = 0

    def record(self, visual_node: int, proprio_node: int,
               hebbian_key: Optional[tuple] = None,
               action_force: float = 0.0):
        """
        Record the current node visit. Called every tick.
        """
        if visual_node < 0:
            return

        entry = (visual_node, proprio_node, hebbian_key, bin_action(action_force))
        self._path.append(entry)
        if len(self._path) > self._path_length:
            self._path.pop(0)
        self._ticks_since_event += 1

    def assign(self,
               signal: float,
               valence_map:      Optional['NodeValenceMap'] = None,
               hebbian_table:    Optional[dict] = None,
               transition_model: Optional['ActionTransitionModel'] = None,
               ) -> dict:
        """
        Walk the path backward and apply credit (signal>0) or blame (signal<0)
        with exponential decay.

        Parameters
        ----------
        signal          float   positive = credit (hit), negative = blame (miss)
        valence_map     NodeValenceMap to update (or None to skip)
        hebbian_table   dict of Hebbian entries to update (or None to skip)
        transition_model ActionTransitionModel (reserved for future count decay)

        Returns
        -------
        dict with telemetry: nodes_affected, max_signal, path_depth
        """
        if not self._path or abs(signal) < 0.001:
            return {"nodes_affected": 0}

        # Adaptive horizon: record the inter-event interval and re-derive
        # path_length from the rolling mean. 1.5× mean covers ~85th-percentile
        # under λ=0.75 (further back contributes <5% of credit).
        if self._adaptive:
            self._event_intervals.append(max(1, self._ticks_since_event))
            self._ticks_since_event = 0
            mean_interval = sum(self._event_intervals) / len(self._event_intervals)
            new_len = int(mean_interval * 1.5)
            self._path_length = max(self._length_min, min(self._length_max, new_len))
            # Trim buffer if horizon shrank below its current length.
            while len(self._path) > self._path_length:
                self._path.pop(0)

        is_credit = signal > 0
        scale = self._credit_scale if is_credit else self._blame_scale
        magnitude = abs(signal) * scale

        if is_credit:
            self.total_credit_events += 1
        else:
            self.total_blame_events += 1

        nodes_affected = 0
        blamed_nodes = set()   # unique (visual, proprio) pairs touched

        # Walk backward: index -1 = most recent = k=0 = strongest signal
        for k, entry in enumerate(reversed(self._path)):
            v_node, p_node, h_key, a_bin = entry

            # Decay curve: lambda^k
            weight = magnitude * (self._lambda_decay ** k)
            if weight < 0.001:
                break  # negligible — stop walking

            signed_weight = weight if is_credit else -weight

            # --- 1. Valence map: shift emotional coloring of this state ---
            if valence_map is not None:
                current_v = valence_map._map.get((v_node, p_node), 0.0)
                valence_map._map[(v_node, p_node)] = float(np.clip(
                    current_v + signed_weight * 0.1,  # scaled down: valence is EMA'd
                    -1.0, 1.0,
                ))

            # --- 2. Hebbian table: reinforce or erode the entry ---
            if hebbian_table is not None and h_key is not None and h_key in hebbian_table:
                current_h = hebbian_table[h_key]
                hebbian_table[h_key] = float(np.clip(
                    current_h + signed_weight * 0.5,  # stronger effect on habits
                    -12.0, 12.0,
                ))

            blamed_nodes.add((v_node, p_node))
            nodes_affected += 1

        return {
            "nodes_affected": nodes_affected,
            "max_signal":     round(magnitude, 4),
            "path_depth":     len(self._path),
            "is_credit":      is_credit,
            "blamed_nodes":   blamed_nodes,
        }

    def get_path_nodes(self) -> list:
        """Return the current path as a list of (visual, proprio) tuples."""
        return [(v, p) for v, p, _, _ in self._path]

    @property
    def path_length(self) -> int:
        return int(self._path_length)

    def stats(self) -> dict:
        mean_int = (sum(self._event_intervals) / len(self._event_intervals)
                    if self._event_intervals else 0.0)
        return {
            "path_depth":     len(self._path),
            "max_depth":      self._path_length,
            "lambda_decay":   self._lambda_decay,
            "credit_events":  self.total_credit_events,
            "blame_events":   self.total_blame_events,
            "mean_interval":  round(float(mean_int), 1),
            "adaptive":       self._adaptive,
        }
