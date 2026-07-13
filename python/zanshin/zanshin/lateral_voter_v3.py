"""
lateral_voter_v3.py — Simplified Lateral Voter for the v3 multi-EPM bus.

Receives RealityToken dicts from N C++ EPM nodes and fuses them into a single
consensus stats dict that the UI can consume.

Algorithm
---------
1. Trust weight per EPM:  w_i = 1 / (tle_i + ε),  L1-normalised *within each
   modality group* (video / audio / other).  Each group receives equal total
   weight (e.g. 50/50 when both video and audio EPMs are present).  This
   prevents a single fast-converging audio EPM from starving video EPMs of
   their share in the trust display.

2. Fused TLE:  weighted average of per-EPM TLEs.

3. Novelty:  trust-weighted — the summed trust of novel-firing EPMs must exceed
   0.35 of total trust mass.

4. Active EPM:  select the modality group with the highest *total* group
   activity (sum of trust_i × (tle_i + ε)), then pick the highest-activity
   member within that group.  This ensures video EPMs are preferred as active
   when they collectively have more signal than a single audio EPM.

5. Pass-through:  remaining UI fields are taken from the active EPM so the
   visualiser, threshold display, and histogram all show meaningful data.

Hebbian association matrix update (stub — full implementation in Phase 5):
   A += η × outer(embed_i, embed_j)  for all (i, j) EPM pairs
This will enable cross-modal resonance detection.  For now, association is
tracked but not yet used to modulate trust.
"""

from collections import defaultdict

import numpy as np
from typing import Dict, Optional, Tuple


_EPS = 0.05   # prevents division by zero and keeps TLE=0 EPMs from dominating

# Modality groups are derived from the encoder registry so that any encoder —
# built-in (generic "audio") or contributed by a plugin (e.g. the private
# AMI-Awen "cochlear") — self-declares its group without this module naming it.
# The extra members below are group aliases that are not themselves encoders
# (e.g. a fused "visual_consensus" channel).
def _mods_in_group(group: str, extra: frozenset = frozenset()) -> frozenset:
    try:
        from zanshin.encoders.registry import groups as _reg_groups
        return frozenset(_reg_groups().get(group, set())) | extra
    except Exception:
        return extra

_VIDEO_MODS = _mods_in_group("video", frozenset({"visual_consensus"}))
_AUDIO_MODS = _mods_in_group("audio")


def _group_proportional_trust(mods: list, inv_tle: np.ndarray) -> np.ndarray:
    """
    Compute trust weights with modality-group balancing.

    Within each group (video / audio / other) trust is proportional to inv_tle.
    Each group receives equal total weight regardless of how many members it has
    or how low any individual EPM's TLE is.

    This prevents a fast-converging audio EPM (low TLE) from monopolising trust
    and starving video EPMs of their share in the bar graph display.
    """
    n = len(mods)
    video_idxs = [i for i, m in enumerate(mods) if m in _VIDEO_MODS]
    audio_idxs = [i for i, m in enumerate(mods) if m in _AUDIO_MODS]
    other_idxs = [i for i in range(n)
                  if i not in set(video_idxs) and i not in set(audio_idxs)]

    groups = [g for g in [video_idxs, audio_idxs, other_idxs] if g]
    n_groups = len(groups)

    trust = np.zeros(n, dtype=np.float32)
    group_weight = 1.0 / n_groups
    for group_idxs in groups:
        g_inv = inv_tle[group_idxs]
        # Proportional (L1-normalised) within the group — no exp() amplification
        g_trust = g_inv / g_inv.sum()
        for local_i, global_i in enumerate(group_idxs):
            trust[global_i] = g_trust[local_i] * group_weight

    return trust


class LateralVoterV3:
    """
    Phase 3 Lateral Voter — trust-weighted fusion of N EPM RealityTokens.

    Parameters
    ----------
    modalities : list[str]
        Ordered list of EPM modality names.
    projection_dim : int
        Latent vector dimension (must be the same for all EPMs).
    hebbian_lr : float
        Learning rate for the Hebbian association matrix (future use).
    """

    def __init__(self,
                 modalities: list,
                 projection_dim: int = 128,
                 hebbian_lr: float = 0.05,
                 hebbian_decay: float = 0.999,
                 hebbian_max_entries: int = 5000,
                 resonance_trust_boost: float = 0.15,
                 consensus_stability_window: int = 10,
                 consensus_boost_amount: int = 3):
        self.modalities     = list(modalities)
        self.projection_dim = projection_dim
        self.hebbian_lr     = hebbian_lr
        self._hebbian_decay = hebbian_decay
        self._hebbian_max   = hebbian_max_entries
        self._resonance_boost = resonance_trust_boost

        # B4: Sparse Hebbian association matrix — cross-modal node-level links.
        # Key: (mod_a, node_a, mod_b, node_b) with mod_a < mod_b lexically.
        # Value: association strength in [0, 1].
        self._assoc: Dict[Tuple[str, int, str, int], float] = defaultdict(float)
        self._decay_counter = 0
        self._resonance_score = 0.0  # latest tick resonance

        # EMA of trust weights for smooth display
        self._trust_ema: Optional[np.ndarray] = None
        self._ema_alpha = 0.1

        # Consensus stability tracking — top-down bake acceleration
        self._stability_window = consensus_stability_window
        self._boost_amount     = consensus_boost_amount
        self._prev_winners: Dict[str, list] = {}  # modality → recent winner_ids
        self._tle_ema      = 0.0
        self._tle_var_ema  = 0.0
        self._tle_alpha    = 0.1
        self._consensus_stable = False
        self._pending_boosts: list = []  # [(modality, node_id, amount), ...]

        # Neurochemical gate: serotonin below this threshold suppresses
        # Hebbian learning. When the agent is in pain (wall-stuck, miss),
        # co-activations should NOT be reinforced — they represent a bad
        # state, not a meaningful cross-modal association.
        self._serotonin = 0.65  # default baseline (healthy)
        self._serotonin_gate = 0.35  # below this = suppress learning

    def set_serotonin(self, serotonin: float):
        """Update current serotonin level for Hebbian gating."""
        self._serotonin = serotonin

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def fuse(self, tokens: Dict[str, dict]) -> dict:
        """
        Fuse RealityToken stats from all active EPMs.

        Parameters
        ----------
        tokens : dict[modality → stats_dict]
            Only modalities present in this dict are fused; others are skipped.

        Returns
        -------
        Fused stats dict compatible with the UI's on_stats_updated handler.
        """
        if not tokens:
            return self._empty_stats()

        mods    = [m for m in self.modalities if m in tokens]
        stats   = [tokens[m] for m in mods]

        tles    = np.array([s.get("tle", 0.0) for s in stats], dtype=np.float32)
        inv_tle = 1.0 / (tles + _EPS)
        trust   = _group_proportional_trust(mods, inv_tle)

        # B4: Hebbian resonance modulates trust — EPMs whose current winners
        # have strong cross-modal associations get a trust boost.
        # Only apply after the matrix has enough entries to be meaningful
        # (warmup: need at least 200 entries before resonance can modulate trust).
        resonance = self._compute_resonance(mods, stats)
        if resonance.sum() > 0 and len(self._assoc) >= 200:
            # Normalise resonance to [0, 1] range and apply as multiplicative boost
            res_norm = resonance / (resonance.max() + 1e-6)
            trust = trust * (1.0 + self._resonance_boost * res_norm)
            # Re-normalise trust to sum to 1
            trust = trust / (trust.sum() + 1e-8)

        # Update EMA trust
        if self._trust_ema is None or len(self._trust_ema) != len(trust):
            self._trust_ema = trust.copy()
        else:
            self._trust_ema = (1 - self._ema_alpha) * self._trust_ema + self._ema_alpha * trust

        fused_tle   = float(np.dot(trust, tles))

        # --- Fused embedding: trust-weighted blend of per-EPM latent vectors ---
        # This gives downstream consumers (action decoder) a continuous
        # representation that encodes cross-modal state, not just discrete node IDs.
        fused_embedding = None
        latents = []
        latent_weights = []
        for i, s in enumerate(stats):
            lat = s.get("latent")
            if lat is not None and hasattr(lat, '__len__') and len(lat) > 0:
                latents.append(np.asarray(lat, dtype=np.float32))
                latent_weights.append(float(trust[i]))
        if latents:
            # Pad shorter vectors to the max dim (e.g. proprio=128, retinal=128,
            # but future modalities might differ)
            max_dim = max(v.shape[0] for v in latents)
            padded = []
            for v in latents:
                if v.shape[0] < max_dim:
                    v = np.pad(v, (0, max_dim - v.shape[0]))
                padded.append(v)
            w = np.array(latent_weights, dtype=np.float32)
            w /= w.sum() + 1e-8
            fused_embedding = sum(wi * vi for wi, vi in zip(w, padded))
            # L2 normalize so the decoder sees unit-sphere directions, not magnitudes
            norm = np.linalg.norm(fused_embedding)
            if norm > 1e-6:
                fused_embedding = fused_embedding / norm

        # Trust-weighted novelty: only count novelty proportional to each EPM's trust.
        # OR across all EPMs inflates the rate when low-trust EPMs (e.g. audio on
        # ambient noise) fire is_novel every tick.
        novel_trust = float(sum(trust[i] for i, s in enumerate(stats)
                                if s.get("is_novel", False)))
        is_novel    = novel_trust > 0.35   # majority of trust-mass must agree
        threshold   = max((s.get("threshold", 0.0) for s in stats), default=0.0)

        # Active EPM: pick the group with the highest *total* activity, then
        # choose the most active member within that group.
        # Using group-summed activity (rather than individual argmax) prevents
        # a single low-TLE audio EPM from monopolising active_modality when a
        # larger video group collectively has more informational signal.
        activity = trust * (tles + _EPS)

        grp_vid   = [(i, activity[i]) for i, m in enumerate(mods) if m in _VIDEO_MODS]
        grp_aud   = [(i, activity[i]) for i, m in enumerate(mods) if m in _AUDIO_MODS]
        grp_other = [(i, activity[i]) for i, m in enumerate(mods)
                     if m not in _VIDEO_MODS and m not in _AUDIO_MODS]
        groups    = [g for g in [grp_vid, grp_aud, grp_other] if g]

        best_group = max(groups, key=lambda g: sum(a for _, a in g))
        active_idx  = max(best_group, key=lambda x: x[1])[0]
        active_mod  = mods[active_idx]
        active_stat = stats[active_idx]

        # Hebbian update (cross-modal, symmetric)
        self._update_hebbian(mods, stats)

        # Consensus stability → top-down bake acceleration
        self._pending_boosts.clear()
        self._assess_consensus_stability(mods, stats, fused_tle)

        return {
            # Fused signals
            "fused_embedding":   fused_embedding,
            "tle":               fused_tle,
            "threshold":         threshold,
            "is_novel":          is_novel,
            "node_count":        sum(s.get("node_count", 0)   for s in stats),
            "gng_nodes":         sum(s.get("gng_nodes", 0)    for s in stats),
            "gng_nodes_per_epm": {m: int(s.get("gng_nodes", 0)) for m, s in zip(mods, stats)},
            "gng_baked":         sum(s.get("gng_baked", 0)    for s in stats),
            "gng_mean_error":    float(np.dot(trust, [s.get("gng_mean_error", 0.0) for s in stats])),
            "crystallization_ratio": float(np.mean([s.get("crystallization_ratio", 0.0) for s in stats])),
            "is_mature":         all(s.get("is_mature", False) for s in stats),

            # Lateral voter metadata
            "active_modality":   active_mod,
            "trust_weights":     {m: float(w) for m, w in zip(mods, trust)},
            "trust_ema":         {m: float(w) for m, w in zip(mods, self._trust_ema)},

            # Hebbian resonance
            "resonance_score":   round(self._resonance_score, 4),
            "hebbian_entries":   len(self._assoc),

            # Consensus stability
            "consensus_stable":  self._consensus_stable,
            "consensus_boosts":  list(self._pending_boosts),

            # Pass-through from active EPM (for visualiser, dopamine, etc.)
            **{k: v for k, v in active_stat.items()
               if k not in ("tle", "threshold", "is_novel", "node_count",
                            "gng_nodes", "gng_baked", "gng_mean_error",
                            "crystallization_ratio", "is_mature")},
        }

    def get_trust_weights(self) -> Dict[str, float]:
        """Current EMA trust weights, keyed by modality."""
        if self._trust_ema is None:
            return {m: 1.0 / len(self.modalities) for m in self.modalities}
        return {m: float(w) for m, w in zip(self.modalities, self._trust_ema)}

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _assess_consensus_stability(self, mods: list, stats: list,
                                       fused_tle: float) -> None:
        """
        Track whether downstream consensus is stable.  When it is, emit
        boost signals for upstream EPM nodes that are consistently winning.

        Stability is defined as:
        1. Fused TLE variance (EMA) is low — the global prediction is steady.
        2. Per-EPM winners are consistent — the same node keeps winning.

        When both hold, the voter emits (modality, node_id, boost_amount) tuples
        that the orchestrator should forward to each EPM's boost_node() method.
        """
        a = self._tle_alpha

        # Update fused TLE EMA and variance EMA
        self._tle_ema     = (1 - a) * self._tle_ema + a * fused_tle
        tle_dev           = (fused_tle - self._tle_ema) ** 2
        self._tle_var_ema = (1 - a) * self._tle_var_ema + a * tle_dev

        # Track per-EPM winner consistency (sliding window)
        for m, s in zip(mods, stats):
            wid = s.get("active_node", -1)
            hist = self._prev_winners.setdefault(m, [])
            hist.append(wid)
            if len(hist) > self._stability_window:
                hist.pop(0)

        # Check stability conditions
        tle_stable = self._tle_var_ema < 0.05  # low variance in fused TLE

        # Per-EPM: winner must be the same for >60% of recent window
        stable_mods = []
        for m in mods:
            hist = self._prev_winners.get(m, [])
            if len(hist) < self._stability_window:
                continue
            # Most common winner in the window
            from collections import Counter
            counts = Counter(hist)
            most_common_id, most_common_n = counts.most_common(1)[0]
            if most_common_id >= 0 and most_common_n >= 0.6 * self._stability_window:
                stable_mods.append((m, most_common_id))

        self._consensus_stable = tle_stable and len(stable_mods) > 0

        # Emit boost signals for stable winners that haven't baked yet
        if self._consensus_stable:
            for m, node_id in stable_mods:
                # Find the stats for this modality to check if already baked
                idx = mods.index(m)
                s = stats[idx]
                cryst = s.get("crystallization_ratio", 0.0)
                # Only boost if the EPM isn't already fully crystallized
                if cryst < 0.8:
                    self._pending_boosts.append((m, node_id, self._boost_amount))

    def consume_boosts(self) -> list:
        """Return and clear pending boost signals. Each is (modality, node_id, amount)."""
        boosts = list(self._pending_boosts)
        self._pending_boosts.clear()
        return boosts

    @staticmethod
    def _assoc_key(mod_a: str, node_a: int,
                   mod_b: str, node_b: int) -> Tuple[str, int, str, int]:
        """Canonical key with mod_a < mod_b lexically (symmetric)."""
        if mod_a > mod_b or (mod_a == mod_b and node_a > node_b):
            mod_a, node_a, mod_b, node_b = mod_b, node_b, mod_a, node_a
        return (mod_a, node_a, mod_b, node_b)

    def _update_hebbian(self, mods: list, stats: list) -> None:
        """
        Sparse Hebbian update: strengthen links between co-active and
        temporally proximate GNG winners across modalities.

        GATED by serotonin: when the agent is in an aversive state
        (wall-stuck, post-miss), serotonin drops below the gate threshold
        and association learning is suppressed. This prevents the system
        from reinforcing co-activations that represent stagnation or
        failure — only "healthy" perceptual states build associations.

        Two signals drive association:
          1. **Simultaneous co-activation** — same tick, full learning rate.
          2. **Temporal co-occurrence** — current winner of EPM A paired with
             recent winner history of EPM B (and vice versa).  Learning rate
             decays exponentially with temporal distance: lr × 0.5^lag.
        """
        # Collect current winners (only valid node IDs)
        winners = {}
        for m, s in zip(mods, stats):
            nid = s.get("active_node", -1)
            if nid >= 0:
                winners[m] = nid

        # Serotonin gate: scale learning rate by serotonin level.
        # Below gate threshold, learning is heavily suppressed (not zero,
        # to allow slow recovery). Above threshold, full learning rate.
        # This breaks the wall-stuck → high resonance positive feedback loop.
        sero_scale = min(1.0, max(0.05, (self._serotonin - self._serotonin_gate + 0.1) / 0.3))
        lr = self.hebbian_lr * sero_scale

        # 1. Simultaneous co-activation (full LR)
        winner_mods = list(winners.keys())
        for i in range(len(winner_mods)):
            for j in range(i + 1, len(winner_mods)):
                m_a, m_b = winner_mods[i], winner_mods[j]
                key = self._assoc_key(m_a, winners[m_a], m_b, winners[m_b])
                self._assoc[key] = min(1.0, self._assoc[key] + lr)

        # 2. Temporal co-occurrence — current winner × recent history of other EPMs
        for m_cur, n_cur in winners.items():
            for m_other in winners:
                if m_other == m_cur:
                    continue
                hist = self._prev_winners.get(m_other, [])
                # Walk history backwards; skip the most recent (that's the
                # simultaneous pair already handled above)
                for lag, n_hist in enumerate(reversed(hist[:-1]), start=1):
                    if n_hist < 0:
                        continue
                    temporal_lr = lr * (0.5 ** lag)  # exponential decay with lag
                    if temporal_lr < 0.0005:
                        break  # negligible contribution
                    key = self._assoc_key(m_cur, n_cur, m_other, n_hist)
                    self._assoc[key] = min(1.0, self._assoc[key] + temporal_lr)

        # Periodic decay + pruning (every 10 ticks to save CPU)
        self._decay_counter += 1
        if self._decay_counter >= 10:
            self._decay_counter = 0
            decay = self._hebbian_decay ** 10  # compound 10 ticks of decay
            to_delete = []
            for key, val in self._assoc.items():
                val *= decay
                if val < 0.001:
                    to_delete.append(key)
                else:
                    self._assoc[key] = val
            for key in to_delete:
                del self._assoc[key]

            # Hard cap: if too many entries, prune weakest
            if len(self._assoc) > self._hebbian_max:
                sorted_keys = sorted(self._assoc, key=self._assoc.get)
                n_prune = len(self._assoc) - self._hebbian_max
                for key in sorted_keys[:n_prune]:
                    del self._assoc[key]

    def _compute_resonance(self, mods: list, stats: list) -> np.ndarray:
        """
        Compute per-EPM resonance scores from the Hebbian association matrix.

        For each EPM, sum the association strengths between its current winner
        and every other EPM's current winner.  Returns an array of shape (N,)
        where N = len(mods).  Values are in [0, ∞) — higher means the EPM's
        current state is well-associated with other modalities.
        """
        winners = {}
        for m, s in zip(mods, stats):
            nid = s.get("active_node", -1)
            if nid >= 0:
                winners[m] = nid

        scores = np.zeros(len(mods), dtype=np.float32)
        for i, m_i in enumerate(mods):
            if m_i not in winners:
                continue
            n_i = winners[m_i]
            for j, m_j in enumerate(mods):
                if i == j or m_j not in winners:
                    continue
                n_j = winners[m_j]
                key = self._assoc_key(m_i, n_i, m_j, n_j)
                scores[i] += self._assoc.get(key, 0.0)

        self._resonance_score = float(scores.sum())
        return scores

    @staticmethod
    def _empty_stats() -> dict:
        return {
            "fused_embedding": None,
            "tle": 0.0, "threshold": 0.0, "is_novel": False,
            "node_count": 0, "gng_nodes": 0, "gng_baked": 0,
            "gng_mean_error": 0.0, "crystallization_ratio": 0.0,
            "is_mature": False, "active_modality": "none",
            "trust_weights": {}, "trust_ema": {},
            "loss": 0.0, "nl": 0.0, "dopamine": 0.0, "serotonin": 0.0,
            "context_novelty": 0.0, "active_node": -1,
            "brain_profile": {"total": 0}, "is_bootstrapping": False,
        }
