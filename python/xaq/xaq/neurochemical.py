"""
neurochemical.py — Dopamine / Serotonin state for AMI-Ogma v3.

This is the evolutionary substrate: hard-coded structural valuation signals
that bootstrap learning without external reward engineering.

  dopamine  — reward / approach signal.  Fires on hits and brick clears.
              Decays toward baseline between events.
              High dopamine → faster GNG learning (epsilon_b up) around the
              states that caused the reward.

  serotonin — stability / aversion signal.  Drops on misses.
              Low serotonin → suppressed crystallization of aversive states
              (min_insertion_error up — don't bake "miss" patterns).

Both signals are clamped [0, 1] and decay exponentially each tick.
The decay rates are tuned for 20fps game ticks (τ ≈ 5-10 ticks per event).

Usage:
    chem = NeurochemicalState()
    chem.tick(hit=True, miss=False, brick=False)
    scale = chem.epsilon_b_scale()   # → [0.3, 2.5]
    reward = chem.reward_signal()    # → [-1, 1], centred at 0
"""

from typing import Optional
import numpy as np


class NeurochemicalState:
    """
    Minimal 2-chemical state machine.

    Parameters
    ----------
    da_decay        float  per-tick dopamine decay factor   (0→instant, 1→no decay)
    ht_decay        float  per-tick serotonin decay factor
    da_baseline     float  resting dopamine level
    ht_baseline     float  resting serotonin level
    da_hit          float  dopamine pulse on paddle hit
    da_brick        float  dopamine pulse on brick destroyed
    ht_miss_drop    float  serotonin drop on miss
    """

    def __init__(self,
                 da_decay:     float = 0.88,
                 ht_decay:     float = 0.93,
                 da_baseline:  float = 0.20,
                 ht_baseline:  float = 0.65,
                 da_hit:       float = 0.45,
                 da_brick:     float = 0.65,
                 da_miss_drop: float = 0.25,
                 ht_miss_drop: float = 0.30):
        self._da_decay    = da_decay
        self._ht_decay    = ht_decay
        self._da_base     = da_baseline
        self._ht_base     = ht_baseline
        self._da_hit      = da_hit
        self._da_brick    = da_brick
        self._da_miss     = da_miss_drop
        self._ht_miss     = ht_miss_drop

        self.dopamine  = da_baseline
        self.serotonin = ht_baseline

        # Motor efficacy: tonic dopamine modulation based on whether
        # the agent's actions are affecting its sensory stream.
        # Homeokinetic principle: a living system should prefer states
        # where its motor commands have consequences.
        self._prev_sensory_hash = 0.0   # hash of previous sensory state
        self._efficacy_ema      = 0.0   # EMA of motor efficacy

        # Telemetry
        self.total_hits   = 0
        self.total_misses = 0
        self.total_bricks = 0

        # Intrinsic-motivation layer (ecological-agency pivot):
        #   dopamine   ← TLE reduction (the agent got better at prediction)
        #   serotonin  ← whisker-level integral (proximity to failure boundary)
        # Hit/miss events remain for evaluation-only counters; they no longer
        # drive the learning signal. See on_tle / on_whisker below.
        self._prev_tle:            Optional[float] = None
        self._intrinsic_da_gain:   float = 0.05   # pulse per unit TLE drop
        self._whisker_ht_rate:     float = 0.02   # serotonin drain / tick / unit whisker
        self._hunger_ht_rate:      float = 0.01   # serotonin drain / tick / unit hunger
        self._scent_da_rate:       float = 0.25   # dopamine pulse / unit scent delta (rise only)
        self._prev_scent:          Optional[float] = None
        self._travel_da_rate:      float = 0.02   # dopamine pulse / tick / unit (speed × openness)
        self._pheromone_ht_rate:   float = 0.005  # serotonin drain / tick / unit (lvl-thresh)
        self._pheromone_threshold: float = 0.3    # below this, pheromone is ignored
        # Feature flags — enabled by default under the pivot. Flip to False
        # to reinstate event-coupled dopamine / serotonin for ablation.
        self._event_coupled_da:    bool  = False
        self._event_coupled_ht:    bool  = False

    # ------------------------------------------------------------------
    # Primary update — call once per game tick
    # ------------------------------------------------------------------

    def tick(self, hit: bool = False, miss: bool = False,
             brick: bool = False, whisker_bump: bool = False,
             wall_stuck: bool = False, tle: float = 0.0) -> None:
        """
        Decay both signals toward baseline, then inject event pulses.
        Call AFTER reading game state deltas for the current tick.

        The optional *tle* (Time-Loop Error from the current tick) amplifies
        the neurochemical pulse: a hit during high perceptual surprise is more
        informative than one during steady tracking, and likewise for misses.
        Amplification factor: (1 + tle), so TLE=0 → normal pulse, TLE=1 → 2x.
        """
        # Exponential decay toward baseline
        self.dopamine  = self._da_base + (self.dopamine  - self._da_base)  * self._da_decay
        self.serotonin = self._ht_base + (self.serotonin - self._ht_base)  * self._ht_decay

        amp = 1.0 + tle   # TLE-amplified pulse (A3)

        # Event counters always update (evaluation-only scoring). Whether the
        # events also drive dopamine/serotonin is gated by the pivot flags —
        # by default the intrinsic layer (on_tle / on_whisker) is authoritative
        # and hit/miss are purely telemetry.
        if hit:
            self.total_hits += 1
            if self._event_coupled_da:
                self.dopamine = min(1.0, self.dopamine + self._da_hit * amp)
        if brick:
            self.total_bricks += 1
            if self._event_coupled_da:
                self.dopamine = min(1.0, self.dopamine + self._da_brick * amp)
        if miss:
            self.total_misses += 1
            if self._event_coupled_da:
                self.dopamine  = max(0.0, self.dopamine  - self._da_miss * amp)
            if self._event_coupled_ht:
                self.serotonin = max(0.0, self.serotonin - self._ht_miss * amp)
            
        # Tactile Curiosities — DISABLED (ablation).
        # Was: +0.15 dopamine per tick of whisker contact. Level-triggered
        # (fires every tick paddle is near a wall, not just on entry), which
        # pinned dopamine at 1.0 during wall contact and imprinted strong
        # positive valence on wall-stuck (v,p) pairs. Confirmed via diag
        # audit: paddle_id≈±0.95 → proprio_valence≈+0.7 vs center≈+0.05.
        # Removing to isolate the true reward signal.
        # if whisker_bump:
        #     self.dopamine = min(1.0, self.dopamine + 0.15 * amp)
            
        # Somatic Aversion (Pain)
        if wall_stuck:
            # Prolonged wall grinding drains both motivation and stability
            # Increased penalty to break stagnation (Refinement Plan)
            self.dopamine = max(0.0, self.dopamine - 0.35 * amp)
            self.serotonin = max(0.0, self.serotonin - 0.15 * amp)

    def on_tle(self, tle: float) -> None:
        """Intrinsic dopamine from TLE reduction (Step 4 pivot).

        Reward is "I just got better at predicting my own dynamics." The agent
        is its own teacher. Dopamine pulses on any drop in TLE between
        consecutive ticks; no change in TLE = no pulse; increase = no pulse
        (no negative pulse either — that's serotonin's job via whiskers)."""
        if self._prev_tle is not None:
            delta = self._prev_tle - float(tle)   # positive when prediction improved
            if delta > 0:
                self.dopamine = min(1.0, self.dopamine + self._intrinsic_da_gain * delta)
        self._prev_tle = float(tle)

    def on_whisker(self, level: float) -> None:
        """Intrinsic serotonin drain from whisker integral (Step 5 pivot).

        Aversion fires *before* a fall / collision when the body's proximity
        sensors (whiskers) trip. Credit assignment tightens because the
        aversive signal lines up with the action that drove the approach,
        not the consequence two seconds later."""
        lvl = float(np.clip(level, 0.0, 1.0))
        if lvl > 0.0:
            self.serotonin = max(0.0, self.serotonin - self._whisker_ht_rate * lvl)

    def on_hunger(self, level: float) -> None:
        """Intrinsic serotonin drain from interoceptive hunger (Phase 2).

        Continuous aversion tied to depleting body energy. Unlike whisker
        aversion which fires on instantaneous proximity, hunger drains
        serotonin every tick the agent is below full energy — the longer
        the search, the stronger the aversion to the current (state, place)
        gets imprinted. Same neurotransmitter as whiskers because both are
        "something is wrong" signals. Separate rate so tuning can stay
        distinct."""
        lvl = float(np.clip(level, 0.0, 1.0))
        if lvl > 0.0:
            self.serotonin = max(0.0, self.serotonin - self._hunger_ht_rate * lvl)

    def on_pheromone(self, level: float) -> None:
        """Aversive serotonin drain from re-entering recently-visited ground
        (Phase 2.5). Threshold-gated: below _pheromone_threshold the signal
        is fresh-passing noise and fires nothing; above, it scales linearly
        with excess staleness so only genuine revisits trigger aversion. This
        keeps the pragmatic R-map gradient intact without flooding
        pain_drive → exploration_weight, which previously broke navigation
        in tests (run 3: prag_diff 0.28 but time-to-eat worsened 7× from the
        jitter)."""
        lvl = float(np.clip(level, 0.0, 1.0))
        excess = lvl - self._pheromone_threshold
        if excess > 0.0:
            self.serotonin = max(
                0.0,
                self.serotonin - self._pheromone_ht_rate * excess,
            )

    def on_travel(self, level: float) -> None:
        """Appetitive dopamine from clean locomotion (Phase 2.5).

        level = actual_speed × openness, where openness = 1 - max(whiskers).
        A stationary agent (speed=0) or one in tight corners (openness≈0)
        gets nothing; fast travel through an open corridor gets a strong
        positive pulse. The rodent analog of tonic dopamine during open-field
        corridor-running — locomotion itself is rewarding, independent of
        what's at the end of the corridor. Biases the agent toward navigable
        space without encoding any goal representation."""
        lvl = float(np.clip(level, 0.0, 1.0))
        if lvl > 0.0:
            self.dopamine = min(1.0, self.dopamine + self._travel_da_rate * lvl)

    def on_scent(self, max_level: float) -> None:
        """Appetitive dopamine from rising olfactory gradient.

        The body's scent whiskers report distance-to-food through open
        corridors (wall-occluded). Dopamine pulses on the *delta* — getting
        closer rewards the action that drove the approach, just like TLE
        reduction rewards prediction improvements. Falling scent fires
        nothing (no negative pulse; losing the trail isn't punishment —
        just the absence of reward)."""
        lvl = float(np.clip(max_level, 0.0, 1.0))
        if self._prev_scent is not None:
            delta = lvl - self._prev_scent
            if delta > 0:
                self.dopamine = min(1.0, self.dopamine + self._scent_da_rate * delta)
        self._prev_scent = lvl

    def update_motor_efficacy(self, sensory_delta: float, motor_magnitude: float):
        """
        Tonic dopamine modulation from sensory-motor efficacy.

        Homeokinetic principle (Der & Martius): an embodied agent should
        prefer states where its actions have consequences on its sensory
        stream. Movement that changes perception = alive. Movement that
        doesn't = stuck. Stillness = dead.

        This is NOT a game-specific reward — it's a structural prior about
        embodiment that transfers to any agent in any environment.

        Parameters
        ----------
        sensory_delta   float   magnitude of sensory state change this tick
                                (e.g., change in GNG winner node, embedding distance)
        motor_magnitude float   magnitude of motor command this tick (abs(force))
        """
        # Efficacy = sensory change per unit motor effort
        # If moving but nothing changes → low efficacy (stuck at wall)
        # If moving and world changes → high efficacy (engaged)
        # If idle and nothing changes → neutral (not penalized for resting)
        if motor_magnitude > 0.5:  # only measure when actually trying to move
            efficacy = min(1.0, sensory_delta / (motor_magnitude + 0.1))
        else:
            efficacy = 0.5  # neutral when idle — don't penalize resting

        # EMA smoothing: slow update to capture trends, not noise
        self._efficacy_ema = 0.95 * self._efficacy_ema + 0.05 * efficacy

        # Tonic modulation: small continuous dopamine adjustment
        # High efficacy → slight boost (up to +0.02/tick)
        # Low efficacy → slight drain (down to -0.03/tick, asymmetric
        # because stagnation should be more aversive than movement is rewarding)
        tonic = (self._efficacy_ema - 0.3) * 0.05  # centered at 0.3 efficacy
        tonic = max(-0.03, min(0.02, tonic))

        self.dopamine = max(0.0, min(1.0, self.dopamine + tonic))

    # ------------------------------------------------------------------
    # Derived signals for downstream consumers
    # ------------------------------------------------------------------

    def reward_signal(self) -> float:
        """
        Signed reward for Hebbian update, centred at 0.
        Range approximately [-0.65, +0.80].
        """
        return self.dopamine - self._da_base

    def epsilon_b_scale(self) -> float:
        """
        Multiplicative scale for GNG winner learning rate (epsilon_b).
        High dopamine → learn faster around reward-proximal states.
        Range [0.3, 2.5].
        """
        return 0.3 + 2.2 * self.dopamine

    def min_insertion_error_scale(self) -> float:
        """
        Multiplicative scale for GNG min_insertion_error.
        Low serotonin (aversive state) → raise insertion floor so the system
        does not crystallize miss-proximal patterns.
        Range [0.5, 1.8].
        """
        return 1.8 - 1.3 * self.serotonin

    def mitosis_threshold_scale(self) -> float:
        """
        Multiplicative scale for GNG mitosis_error_threshold.
        High serotonin (stable) → raise threshold → suppress mitosis.
        Low serotonin (after miss) → lower threshold → allow restructuring.
        Range [0.6, 1.8].
        """
        return 0.6 + 1.2 * self.serotonin

    def hit_rate(self) -> float:
        total = self.total_hits + self.total_misses
        return self.total_hits / total if total > 0 else 0.0

    def to_dict(self) -> dict:
        return {
            "dopamine":         round(self.dopamine, 4),
            "serotonin":        round(self.serotonin, 4),
            "epsilon_b_scale":  round(self.epsilon_b_scale(), 3),
            "mie_scale":        round(self.min_insertion_error_scale(), 3),
            "hit_rate":         round(self.hit_rate(), 4),
            "hits":             self.total_hits,
            "misses":           self.total_misses,
            "bricks":           self.total_bricks,
        }
