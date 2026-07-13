#pragma once

// =============================================================================
// NeurochemState.hpp  --  Module 1 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/NeurochemState.md
// v3 reference:    src/ami_ogma_v3/neurochemical.py
//
// The bath's broadcast hormone-and-neurotransmitter layer.  Integrates
// per-tick TLE samples, environment events, and proprioceptive scalar streams
// into a small bundle of dimensionless signals that every other module reads
// from `neuro.state`.
//
// Math is faithful to v3.  Decay constants, intrinsic-motivation rates, and
// scaling-factor formulas all match the v3 frozen baseline.

#include <cstdint>
#include <string>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class NeurochemState : public Module {
public:
    NeurochemState();
    ~NeurochemState() override;

    // Module interface
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // Read-only accessors for white-box tests.
    float dopamine()  const { return dopamine_; }
    float serotonin() const { return serotonin_; }
    int   total_hits()   const { return total_hits_; }
    int   total_misses() const { return total_misses_; }
    int   total_bricks() const { return total_bricks_; }

private:
    // Subscription handlers
    void handle_reality(std::string_view topic, MessagePtr payload);
    void handle_consensus(std::string_view topic, MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);
    void handle_proprio(std::string_view topic, MessagePtr payload);

    // Bookkeeping helpers
    void   reset_pending();
    float  scale_epsilon_b()        const;
    float  scale_min_insertion()    const;
    float  scale_mitosis()          const;
    float  scale_novelty()          const;

    // Parameters (defaults match v3 frozen baseline).
    double da_baseline_       = 0.20;
    double ht_baseline_       = 0.65;
    double da_decay_          = 0.88;
    double ht_decay_          = 0.93;

    // Phase 6.5.3.10 — adaptive (EMA-based) dopamine baseline.
    // When > 0, reward_signal = dopamine - da_baseline_ema, where
    // da_baseline_ema tracks dopamine on a slow time-scale.  This makes
    // the substrate's reward signal self-zero-centering: at steady-
    // state event rates the EMA catches up and reward_signal returns
    // toward zero — the brain only learns from DEVIATIONS from its
    // own recent dopamine experience.  Mirrors Schultz et al's reward
    // prediction error model.  When 0, behaves identically to the
    // fixed-baseline implementation (backwards compatible).
    double da_baseline_ema_alpha_ = 0.0;     // 0 = static baseline; ~0.001 = slow adaptation

    // Intrinsic-motivation gains (v3 defaults).
    double intrinsic_da_gain_   = 0.05;     // per unit TLE drop
    double scent_da_rate_       = 0.25;     // per unit positive scent delta
    double green_da_rate_       = 0.0;      // per unit positive Δgreen_fraction (anticipatory "hope" — food looming). 0=off
    double travel_da_rate_      = 0.02;     // per unit (speed × openness)

    double whisker_ht_rate_     = 0.02;     // serotonin drain per unit whisker
    double hunger_ht_rate_      = 0.01;     // serotonin drain per unit hunger
    double pheromone_ht_rate_   = 0.005;    // serotonin drain per unit excess pheromone
    double pheromone_threshold_ = 0.30;     // below this, pheromone is ignored

    double wall_stuck_da_drain_ = 0.35;
    double wall_stuck_ht_drain_ = 0.15;

    // Event-coupled gains (off by default per v3 — events are telemetry only
    // unless explicitly enabled for ablation runs).
    bool   event_coupled_da_     = false;
    bool   event_coupled_ht_     = false;
    double da_hit_gain_          = 0.45;
    double da_brick_gain_        = 0.65;
    double da_miss_drop_         = 0.25;
    double ht_miss_drop_         = 0.30;
    // v5.3 Phase C — handtuned vision+scent reward scaffold.  When an
    // EventConjunction fires events.scent_aligned_with_green (or any other
    // event named the configured `aligned_event_name`), inject a phasic DA
    // pulse of magnitude `da_aligned_gain`.  Larger than da_hit_gain by
    // default since it's an EXPLICIT teaching signal — chemotaxis is what
    // we want the substrate to learn, and the scaffold says "this is what
    // success looks like before you've actually got the food."
    double      da_aligned_gain_     = 0.0;     // 0 = scaffold off (default)
    std::string aligned_event_name_  = "scent_aligned_with_green";

    // Phase 7.2-EPM — list of trailing-dot prefixes (e.g.
    // "reality.joint_fl.", "reality.leg.") whose RealityToken
    // deliveries are dropped from TLE aggregation.  Mirrors
    // LateralVoter's input_exclude pattern.  Used to keep derived /
    // observer-only EPM pathways from contaminating the global
    // dopamine signal.  Empty = no exclusion (legacy behaviour).
    std::vector<std::string> input_exclude_;

    // Master seed forwarded to derive_rng() (no stochastic ops yet but
    // declared for forward-compat per docs/primitives/_rng.md).
    uint64_t master_seed_ = 0;

    // Working state.
    float    dopamine_           = 0.20f;   // matches da_baseline_ at construction
    float    serotonin_          = 0.65f;   // matches ht_baseline_
    float    da_baseline_ema_    = 0.20f;   // Phase 6.5.3.10 — slow EMA of dopamine

    bool     prev_tle_set_       = false;
    float    prev_tle_           = 0.0f;
    bool     prev_scent_set_     = false;
    float    prev_scent_         = 0.0f;
    bool     prev_green_set_     = false;
    float    prev_green_         = 0.0f;

    // Pending per-tick signals.  Filled by handlers; consumed and reset in tick().
    int      pending_hit_count_      = 0;
    int      pending_aligned_count_  = 0;     // v5.3 Phase C — scent_aligned_with_green firings this tick
    int      pending_miss_count_     = 0;
    int      pending_brick_count_    = 0;
    int      pending_wall_stuck_     = 0;
    int      pending_whisker_bump_   = 0;
    float    pending_tle_sum_        = 0.0f;
    int      pending_tle_count_      = 0;
    float    pending_consensus_tle_  = 0.0f;
    bool     pending_consensus_seen_ = false;
    float    pending_whisker_max_    = 0.0f;
    float    pending_hunger_max_     = 0.0f;
    float    pending_pheromone_max_  = 0.0f;
    float    pending_scent_max_      = 0.0f;
    float    pending_green_max_      = 0.0f;
    float    pending_travel_max_     = 0.0f;
    // Phase 6.5.3.F — terminal-event flush flag.  Set by handle_event when
    // events.solved or events.failed fires; consumed at end of next tick to
    // reset dopamine + EMA so the queued hit/miss pulse is realized once
    // (crediting the terminal-action via td_update) but does not leak into
    // the new episode's TD updates via the dopamine decay tail.
    bool     pending_terminal_       = false;

    // Telemetry counters (cumulative).
    int total_hits_   = 0;
    int total_misses_ = 0;
    int total_bricks_ = 0;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).

public:
    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
