#pragma once

// =============================================================================
// LateralVoter.hpp  --  Module 3 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/LateralVoter.md
// v3 reference:    src/ami_ogma_v3/lateral_voter_v3.py
//
// Aggregates per-tick RealityToken payloads from every EPM at one level into
// a single fused ConsensusToken.  Three jobs:
//
//   1. Trust-weighted fusion of latents.  Trust = 1/(tle + ε), with optional
//      modality-group balancing so a high-cardinality group cannot drown out
//      a low-cardinality one.
//   2. Active-modality selection — `<group>.<member>` of the highest-trust
//      group, with `priority_group` (default "proprio") winning ties.
//   3. Hebbian association_matrix across modalities.  Stub-implemented in
//      Phase 1; the schema is wired so Phase 3 can promote without amending
//      the contract.
//
// Modality-group parsing comes from the topic name itself.  For an
// `input_pattern` of "reality.", the topic "reality.video.retinal" yields
// group "video" and member "retinal".  This is the load-bearing claim of
// the hierarchical namespace decision.

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class LateralVoter : public Module {
public:
    LateralVoter();
    ~LateralVoter() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // Read-only accessors for white-box tests.
    int    level()                    const { return level_; }
    size_t num_pending_inputs()       const { return pending_.size(); }

    // B1 observability: Hebbian association_matrix density + magnitude.
    // Both are 0 when association_enabled_=false (matrix never updated).
    // Used by OgmaBrain::get_module_metrics() to expose per-tick state to
    // the JSONL audit trail so A/B runs can verify the mechanism activated.
    size_t assoc_matrix_nnz() const {
        size_t n = 0;
        for (auto const& kv : assoc_) n += kv.second.size();
        return n;
    }
    float  assoc_matrix_sum() const {
        float s = 0.0f;
        for (auto const& kv : assoc_)
            for (auto const& inner : kv.second) s += std::fabs(inner.second);
        return s;
    }

private:
    void   handle_input(std::string_view topic, MessagePtr payload);
    void   handle_neuro(std::string_view topic, MessagePtr payload);

    void   publish_placeholder(uint64_t tick_id);

    // Parse "<group>.<member>" from a topic, given input_pattern_.
    // Returns ("", "") if the topic doesn't begin with input_pattern_.
    std::pair<std::string, std::string>
    parse_group_member(std::string const& topic) const;

    // 2026-06-20 — crystallized informativeness ∈ [0,1] from a channel's
    // baked_count: 0 = ≤1 baked concept (degenerate), 1 = ≥info_baked_ref baked.
    // The flicker-immune signal that escapes the trap fooling both TLE and QE.
    float channel_informativeness(RealityToken const& tok) const;

    // Configuration
    int          level_                    = 0;
    std::string  input_pattern_            = "reality.";
    // Phase-6.0.c: optional prefix(es) to ignore among matches of
    // input_pattern_.  Lets a top voter subscribe to "reality." but
    // skip raw EPM outputs that are already feeding a mid-voter (e.g.
    // exclude "reality.kinematic_in.") while still seeing the mid-
    // voter's published-as-reality fused output.
    //
    // Phase 6.6.M — accepts either a single string (legacy) or a list
    // of strings (multi-exclude, needed when more than one mid-voter
    // is feeding fused outputs back into the top voter; e.g. bilateral
    // whisker split publishes two reality.whisker_{left,right}.*
    // prefixes that must both be hidden from voter_0).  Stored as a
    // vector internally; a string param is wrapped to a single-item
    // vector at on_setup time.  Empty (default) = no exclusion.
    std::vector<std::string> input_excludes_;
    std::string  output_topic_             = "consensus.0";
    float        trust_epsilon_            = 0.05f;
    // 2026-06-20 — informativeness gate (escape the low-TLE trap).  0 = legacy
    // 1/(tle+ε) trust (bit-identical).  >0 = precision(1/(quant_error+ε)) ×
    // (info_floor + (1−info_floor)·winner_entropy)^gain.
    float        informativeness_gain_     = 0.0f;
    float        info_floor_               = 0.0f;   // additive floor on info; 0 = a dead channel gets EXACTLY 0 trust
    float        info_baked_ref_           = 3.0f;   // baked_count at which informativeness saturates to 1
    // Kalman-lessons Stage 2 (docs/plans-and-designs/epm_kalman_lessons_plan.md) —
    // ACTIVITY term, the observability proxy.  Doctrine §2.3: on a motor system
    // prediction error is not a proxy for competence, activity is; a channel that
    // has stopped moving is trivially predictable and earns the MOST trust from
    // 1/(err+ε) (measured: a dead sensor kept 0.66 of the trust on the bench, and
    // an occluded camera 0.80 on the Cell).  Per channel: an EMA of the latent's
    // tick-to-tick displacement, normalised by its own decaying peak so it is
    // dimensionless and self-calibrating (in [0,1]); raw trust is multiplied by
    // activity^activity_gain.  A republished (sub-rate) token has zero displacement,
    // so a stale channel decays the same way.  0 (default) = byte-identical.
    // Kalman-lessons Stage 2, lever 2 — WHICH error and WHAT power.  Kalman weights
    // by inverse VARIANCE; 1/(err+ε) compresses a 9:1 precision ratio to ~2:1
    // (bench S5: trust on the clean sensor 0.67 vs the optimal 0.90).
    //   trust_source: "default" = each path's own source (legacy: tle; informativeness
    //                 path: quant_error), "tle", "quant_error", or "expected" = the
    //                 winner node's running RMS residual (RealityToken::expected_error).
    //   trust_power:  precision = 1 / (err + ε)^power; 1 (default) = legacy, byte-
    //                 identical; 2 = inverse variance; −1 = the wrong-sign control.
    std::string  trust_source_             = "default";
    float        trust_power_              = 1.0f;
    float        activity_gain_            = 0.0f;
    float        activity_alpha_           = 0.1f;    // EMA rate of the displacement
    float        activity_peak_decay_      = 0.999f;  // per-tick decay of the running peak
    float        activity_floor_           = 1e-3f;   // lowest activity factor (keeps the wrong-sign arm finite)
    bool         group_balance_            = true;
    float        softmax_temperature_      = 1.0f;
    std::string  priority_group_           = "proprio";
    bool         association_enabled_      = false;
    float        association_decay_        = 0.9999f;
    int64_t      association_max_size_     = 10000;
    float        novelty_threshold_        = 0.35f;
    uint64_t     master_seed_              = 0;
    // Phase-6.0.c: when non-empty, after publishing the ConsensusToken to
    // output_topic_ the voter ALSO synthesises a RealityToken and publishes
    // it to this topic so a higher-level voter can consume the fused output
    // through its standard reality.* subscription.  Enables hierarchical
    // voting without a new module class.  Empty (default) = disabled.
    std::string  publish_as_reality_topic_ = "";
    // Phase-6.0.c: when false, suppress the ConsensusToken publish to
    // consensus.<level> entirely.  Mid-voters use this to avoid polluting
    // the consensus.* namespace that downstream modules (NeurochemState,
    // ActionDecoder, etc.) consume — only their publish_as_reality output
    // matters to the parent voter.  Default true preserves existing
    // single-voter behaviour.
    bool         publish_consensus_         = true;

    // Working state
    struct Pending {
        std::string                              topic;
        std::shared_ptr<const RealityToken>      token;
    };

    // Map keyed by topic so duplicate publishes within a tick keep only the
    // latest delivery.  Cleared at the end of every tick().
    std::map<std::string, Pending> pending_;

    // Latest neuro.state for trust-temperature modulation.
    float dopamine_ = 0.0f;

    // The ConsensusToken from the prior tick (republished when no inputs
    // arrive — Invariant 7 of the contract).
    std::shared_ptr<ConsensusToken> prev_token_;

    // Hebbian association_matrix (stub; only updated when
    // `association_enabled_ == true`).
    std::unordered_map<std::string, std::unordered_map<std::string, float>> assoc_;

    // Phase 6.6.E — predicted_pathway surprise modulation.
    //
    // For each modality topic we hold the previous tick's published
    // `predicted_pathway[0]` (i.e. the EPM's prediction of *this* tick's
    // winner).  When this tick's RealityToken arrives we compute a
    // surprise scalar in [0, 1], EMA-smooth it per modality, and use it
    // to attenuate the inverse-TLE raw trust at the existing fusion site.
    //
    //   surprise_kind_ == "binary"    — 0 if predicted_id == observed_id else 1.
    //                                  Cheap; no embedding lookup; same as
    //                                  the original 6.6.E behaviour.
    //   surprise_kind_ == "embedding" — Phase 6.6.H (this commit): cosine-
    //                                  distance surprise.
    //                                    cos = <E_pred, E_obs> / (|E_pred|·|E_obs|)
    //                                    s   = clamp((1 - cos) / 2, 0, 1)
    //                                  Falls back to binary 1.0 when the
    //                                  predicted node's embedding isn't in
    //                                  the local cache (never seen / pruned).
    //                                  Smoother gradient than binary so the
    //                                  FaderController's α responds to
    //                                  *how wrong* the prediction was, not
    //                                  just *whether* it was wrong.
    //
    // surprise_gain_ = 0 ⇒ behavior is bit-identical to pre-6.6.E voter
    // regardless of surprise_kind_.
    float       surprise_gain_      = 0.0f;
    float       surprise_alpha_     = 0.1f;
    float       surprise_floor_     = 0.05f;   // raw trust scale floor (avoid zeroing)
    std::string surprise_kind_      = "binary";
    // Phase 6.6.J — Bayesian shrinkage on the surfaced surprise.  With
    // few prediction samples per modality the raw EMA is unreliable;
    // shrink toward a uniform 0.5 prior so a cold-start EPM doesn't
    // immediately publish "perfect predictor" (surprise=0 → α=1).
    //   surfaced = (1 - c) * 0.5 + c * raw_ema
    //   c        = n / (n + n_prior)
    //   n_prior  = 1 / surprise_alpha   (the EMA's own effective window)
    // The internal trust modulator continues to use the raw EMA so its
    // sensitivity isn't blunted; only the ConsensusToken.surprise_ema
    // value MotorFader/FaderController consume goes through shrinkage.
    // Default true honours the "no hand tuning" directive — α doesn't
    // need a per-env tweak to avoid early brain dominance.
    bool        surprise_calibrate_ = true;
    std::unordered_map<std::string, int>     last_predicted_next_;
    std::unordered_map<std::string, float>   surprise_ema_;
    // Stage 2 activity state (serialised only when activity_gain_ != 0).
    std::unordered_map<std::string, Eigen::VectorXf> act_prev_latent_;
    std::unordered_map<std::string, float>           act_ema_;
    std::unordered_map<std::string, float>           act_peak_;
public:
    /// Current activity factor of a channel, in [0,1]; 0 if unseen.  White-box tests + diag.
    float activity(std::string const& topic) const {
        auto e = act_ema_.find(topic); auto pk = act_peak_.find(topic);
        if (e == act_ema_.end() || pk == act_peak_.end() || pk->second <= 1e-12f) return 0.0f;
        return std::min(1.0f, e->second / pk->second);
    }
private:
    std::unordered_map<std::string, int64_t> prediction_counts_;
    // Per-modality embedding cache (topic → node_id → prototype).  The
    // voter already sees every input modality's RealityToken via its
    // reality.* subscription, so the cache is populated incrementally
    // from `winner_prototype` on each delivery.  Eviction on
    // `pruned_ids` keeps it consistent with GNG mutations.  Used only
    // when surprise_kind_ == "embedding".
    std::unordered_map<std::string,
        std::unordered_map<int, Eigen::VectorXf>> embedding_cache_;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).

public:
    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_lite()      const override;
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
