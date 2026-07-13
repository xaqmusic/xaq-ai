"""
consistency_gate.py — Dorsal/Ventral Consistency Gate (B2).

Compares motion predictions (dorsal EPM) with identity predictions (ventral EPM)
to detect object permanence violations.

Three disagreement signals:

  1. **Novelty mismatch** — one stream fires is_novel while the other doesn't.
     Ventral-only novelty = "ghost" (identity appeared without motion).
     Dorsal-only novelty  = "phantom motion" (motion with no identity change).

  2. **TLE divergence** — large difference in per-tick prediction error between
     the two streams.  If dorsal TLE spikes while ventral is calm, an object
     moved unexpectedly.  If ventral TLE spikes while dorsal is calm, identity
     changed without corresponding motion (teleportation / occlusion).

  3. **Winner stability divergence** — one stream's GNG winner is changing while
     the other is stable.  Tracked via a short EMA of winner-change rate.

The consistency score is in [0, 1]:  1 = perfectly consistent, 0 = maximum
disagreement.  The gate outputs an *inconsistency penalty* that the visual
voter can add to the fused TLE to flag perceptual anomalies.

Usage:
    gate = ConsistencyGate()
    penalty = gate.update(dorsal_stats, ventral_stats)
    # penalty ≥ 0; add to visual consensus TLE or use as a flag
"""

import numpy as np


class ConsistencyGate:
    """
    Dorsal/Ventral consistency checker.

    Parameters
    ----------
    tle_divergence_weight : float
        How much TLE divergence contributes to inconsistency.
    novelty_mismatch_weight : float
        How much novelty disagreement contributes.
    stability_window : int
        EMA window for winner-change rate tracking.
    penalty_scale : float
        Maximum TLE penalty emitted on full inconsistency.
    """

    def __init__(self,
                 tle_divergence_weight:    float = 0.4,
                 novelty_mismatch_weight:  float = 0.3,
                 stability_divergence_weight: float = 0.3,
                 penalty_scale:            float = 0.5,
                 ema_alpha:                float = 0.15):
        self._w_tle   = tle_divergence_weight
        self._w_novel = novelty_mismatch_weight
        self._w_stab  = stability_divergence_weight
        self._penalty_scale = penalty_scale
        self._alpha = ema_alpha

        # State
        self._dorsal_prev_winner:  int   = -1
        self._ventral_prev_winner: int   = -1
        self._dorsal_change_ema:   float = 0.0
        self._ventral_change_ema:  float = 0.0

        # Telemetry
        self.consistency:       float = 1.0
        self.tle_divergence:    float = 0.0
        self.novelty_mismatch:  float = 0.0
        self.stability_divergence: float = 0.0
        self.total_violations:  int   = 0

    def update(self, dorsal_stats: dict, ventral_stats: dict) -> float:
        """
        Compare dorsal and ventral tokens for this tick.

        Returns
        -------
        float
            Inconsistency penalty (≥ 0).  Zero when streams agree;
            up to ``penalty_scale`` on maximum disagreement.
        """
        d_tle = dorsal_stats.get("tle", 0.0)
        v_tle = ventral_stats.get("tle", 0.0)
        d_novel = dorsal_stats.get("is_novel", False)
        v_novel = ventral_stats.get("is_novel", False)
        d_winner = dorsal_stats.get("active_node", -1)
        v_winner = ventral_stats.get("active_node", -1)

        # --- Signal 1: TLE divergence ---
        # Normalise: both TLEs are in [0, ~2]; divergence in [0, 1]
        tle_div = abs(d_tle - v_tle) / (max(d_tle, v_tle) + 0.05)
        self.tle_divergence = tle_div

        # --- Signal 2: Novelty mismatch ---
        novel_mm = 1.0 if (d_novel != v_novel) else 0.0
        self.novelty_mismatch = novel_mm

        # --- Signal 3: Winner stability divergence ---
        d_changed = 1.0 if (d_winner != self._dorsal_prev_winner
                            and self._dorsal_prev_winner >= 0) else 0.0
        v_changed = 1.0 if (v_winner != self._ventral_prev_winner
                            and self._ventral_prev_winner >= 0) else 0.0

        self._dorsal_change_ema = ((1 - self._alpha) * self._dorsal_change_ema
                                   + self._alpha * d_changed)
        self._ventral_change_ema = ((1 - self._alpha) * self._ventral_change_ema
                                    + self._alpha * v_changed)

        # Divergence: one stream changing much faster than the other
        stab_div = abs(self._dorsal_change_ema - self._ventral_change_ema)
        self.stability_divergence = stab_div

        self._dorsal_prev_winner = d_winner
        self._ventral_prev_winner = v_winner

        # --- Combined inconsistency score ---
        inconsistency = (self._w_tle   * tle_div
                         + self._w_novel * novel_mm
                         + self._w_stab  * stab_div)
        inconsistency = min(1.0, inconsistency)

        self.consistency = 1.0 - inconsistency

        if inconsistency > 0.5:
            self.total_violations += 1

        return inconsistency * self._penalty_scale

    def to_dict(self) -> dict:
        return {
            "consistency":          round(self.consistency, 4),
            "tle_divergence":       round(self.tle_divergence, 4),
            "novelty_mismatch":     round(self.novelty_mismatch, 4),
            "stability_divergence": round(self.stability_divergence, 4),
            "total_violations":     self.total_violations,
        }
