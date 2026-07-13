#pragma once

// =============================================================================
// ⚠️  FULLY DEPRECATED — ARCHIVED FOR HISTORICAL PURPOSES (2026-07-03)
// =============================================================================
// Retired. Do NOT wire into new configs; do NOT extend. Kept only as a
// historical reference; it still appears in the legacy motor-repertoire brains
// (the_cell_derived.json + the picrawler drive-chunk configs).
//
// Audited against the brain-building doctrine and retired because:
//   • §1   it PREDICTS NOTHING — no generative/forward model, no TLE; it is a
//          statistical anomaly detector on urgency + a random-accel RNG.
//   • §2.1 its escape accel is RANDOM, undirected motor noise — NOT movement in
//          service of inference (real epistemic foraging grades the run by a
//          consequence; this grades nothing and steers nowhere).
//   • §6   a thicket of hand-set magnitude/threshold constants (episode length,
//          jitter, cooldown, entropy fractions) — the no-tuning anti-pattern.
//   • Its inputs (DriveErrors.urgency, MotorChunks, ActionOut.chunk_remaining,
//          Premotor PolicyToken.entropy) are produced ONLY by the retired
//          MotorRepertoire / ActionDecoder / Premotor stack — signals the cell
//          EFE-arbiter substrate does not even emit.
//
// Superseded by the cell PlayLoop — the energy-gated epistemic explore / grow-
// the-map loop arbitrated by the L2 EFEArbiter (curiosity is instrumental: play
// most when FULL, so the map is built before hunger forces a return). Its two
// salvageable IDEAS — a self-normalised "engagement fell below my own recent
// norm" stall gate, and crediting exploration success off events.eat — are
// carried into PlayLoop by design, not by reusing this code.
// =============================================================================
//
// =============================================================================
// HomeokineticExploration.hpp  --  Module 10 of 10 (Phase 5 dead-zone primitive)
// =============================================================================
//
// Contract:        docs/primitives/HomeokineticExploration.md
// v3 reference:    none.  New primitive added after Phase 5 falsification
//                  (see docs/v4_cell_competence.md).
//
// Failure-detector for the drive/chunk loop.  Maintains a rolling window of
// urgency samples; when the window simultaneously satisfies
//
//   1. urgency rising              (least-squares slope > urgency_rise_eps)
//   2. drive-delta flat            (population variance < drive_flat_eps)
//   3. no chunk applicable         (best motor.chunks outcome <= chunk_match_eps)
//
// the module arms an exploration episode for `episode_ticks` ticks.  During
// the episode the directive carries `active=true` and a deterministic
// per-episode random `accel` sample drawn from a PRNG namespaced
// "kinesis.<id>.episode".  After the episode, a `cooldown_ticks` interval
// must elapse before the next arm is permitted.
//
// Replaces the v3 / Phase-1 Probe machine inside ActionDecoder.  Probe gated
// on a satiety proxy and overrode chunks unconditionally; this primitive
// gates on architectural failure and cannot arm while a chunk is playing.

#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class HomeokineticExploration : public Module {
public:
    HomeokineticExploration();
    ~HomeokineticExploration() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // White-box accessors (used by tests + Godot host metrics).
    bool     active()              const { return ticks_remaining_ > 0; }
    uint64_t episodes_armed()      const { return episodes_armed_; }
    uint64_t current_episode_id()  const { return current_episode_id_; }
    int      window_fill()         const { return int(urgency_window_.size()); }
    bool     gate_chunk_block()    const { return chunk_blocks_; }
    float    long_change_ema()     const { return long_change_ema_; }
    uint64_t sample_count()        const { return sample_count_; }
    int      urgency_buffer_fill() const { return int(urgency_long_buffer_.size()); }
    int      ratio_buffer_fill()   const { return int(ratio_long_buffer_.size()); }
    float    success_rate()        const { return success_rate_ema_; }
    int      saturation_streak()   const { return saturation_streak_; }
    uint64_t entropy_collapse_fires() const { return entropy_collapse_fires_; }
    int      entropy_tracked_count()  const { return int(entropy_state_.size()); }
    int      effective_episode_ticks() const;
    float    effective_accel_jitter()  const;

    // Hits accumulated during the currently-active episode.  Resets on
    // episode end → success_rate_ema_ takes a 1.0 sample if non-zero.
    int      episode_hits()        const { return episode_hits_; }

private:
    void handle_drive(std::string_view topic, MessagePtr payload);
    void handle_chunks(std::string_view topic, MessagePtr payload);
    void handle_action(std::string_view topic, MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);
    void handle_policy_intent(std::string_view topic, MessagePtr payload);

    bool gate_holds() const;        // all three conditions
    void publish_directive(uint64_t tick_id);

    // Configuration
    //
    // Time-scale parameters — physical horizons, not behavioral thresholds:
    int     window_ticks_           = 120;       // short-window length (~2 s)
    int     long_window_ticks_      = 480;       // long-history window (~8 s)
    float   change_ema_alpha_       = 0.002f;    // long-EMA rate of |Δurgency|
    //
    // Anomaly factor — gate fires when current change-rate is below
    // `anomaly_factor` × median of recent change-rate history.  Default
    // 0.5 = "half of typical," a universal anomaly convention.  Robust to
    // outliers (median, not mean) and self-disengaging in sustained low-
    // change regimes (median follows the new normal, gate can't fire).
    float   anomaly_factor_         = 0.5f;
    //
    // Chunk-applicable threshold (intrinsically bounded).
    float   chunk_match_eps_        = 0.01f;
    //
    // Episode shape — Phase 3 outcome-feedback adapts these:
    int     episode_ticks_          = 60;
    int     cooldown_ticks_         = 30;
    float   accel_jitter_           = 1.0f;
    uint64_t master_seed_           = 0;
    // Phase 6.7 — saturation-streak trigger.  Complements the ratio-based
    // anomaly gate (which structurally cannot fire when long_change_ema
    // decays to 0 — exactly the state observed in the user's stuck-pose
    // snapshot where urgency was pinned at 1.000 for 24+ min).  When
    // urgency has been above saturation_clamp_ for >= saturation_streak_
    // _threshold_ consecutive samples, the gate fires regardless of the
    // ratio history.  Both 0 = disabled (default → legacy behaviour).
    float    saturation_clamp_      = 0.0f;
    int      saturation_streak_threshold_ = 0;
    int      saturation_streak_    = 0;

    // Phase 6.7+ — policy-entropy collapse trigger.  HomeokineticExploration's
    // urgency-based gate is structurally blind to policy lock-ins where
    // reward firing rate stays steady but Premotor entropy collapses
    // (verified empirically on picrawler: hk_long_change_ema stayed
    // constant at 0.008 across a 1-hour run while pre_H_fl_pitch crashed
    // from 1.609 → 0.054).  This trigger taps PolicyToken.entropy directly:
    // when any subscribed Premotor's smoothed entropy drops below
    // entropy_collapse_fraction_ × its peak (with peak > entropy_min_peak_
    // to avoid early-training false positives), the gate fires.
    //
    // entropy_collapse_fraction_=0 disables (default → legacy behaviour).
    // Tracked state below: per-producer peak + EMA, no static threshold,
    // satisfies `feedback-no-tuning` (peak adapts to each Premotor's own
    // historical maximum).
    float    entropy_collapse_fraction_   = 0.0f;
    float    entropy_min_peak_            = 1.0f;
    float    entropy_ema_alpha_           = 0.05f;
    int      entropy_cooldown_ticks_      = 1500;
    struct EntropyState {
        float ema  = 0.0f;
        float peak = 0.0f;
        uint64_t cooldown_until = 0;
        bool   initialised = false;
        // Phase 6.7++ dynamic escalation.  Each consecutive fire on the
        // same Premotor (entropy still collapsed when next eligible)
        // bumps consecutive_fires; the next fire's episode_ticks is
        // base_episode_ticks * (1 << consecutive_fires) up to the
        // entropy_max_episode_ticks_ ceiling.  Resets to 0 when entropy
        // has recovered above entropy_recover_fraction × peak by the
        // recovery_check tick.
        int      consecutive_fires = 0;
        uint64_t recovery_check_at = 0;
    };
    mutable std::unordered_map<std::string, EntropyState> entropy_state_;
    mutable uint64_t entropy_collapse_fires_ = 0;
    // Pid that triggered the most recent entropy-collapse fire.  Set by
    // gate_holds(); consumed by tick() to size the episode and by
    // recovery evaluation when the episode ends.  Empty when no
    // entropy-driven episode is active.
    mutable std::string entropy_last_trigger_pid_;
    // Dynamic-escalation params.  Off by default (entropy_max_episode_
    // ticks_ <= base episode_ticks_ disables escalation).
    int      entropy_recovery_window_ticks_ = 60;
    float    entropy_recover_fraction_      = 0.7f;
    int      entropy_max_episode_ticks_     = 60;
    // Last tick we received a PolicyToken update for any producer — used
    // by the gate to decide whether the entropy map is fresh enough to
    // evaluate (avoids firing during early-tick periods before any
    // PolicyToken arrived).
    uint64_t entropy_last_seen_tick_ = uint64_t(-1);

    // Working state
    std::deque<float>  urgency_window_;
    bool               chunk_blocks_         = false;   // true if best chunk's outcome > chunk_match_eps
    int                action_chunk_left_    = 0;       // observed from action.out
    int                ticks_remaining_      = 0;       // > 0 → episode active
    int                cooldown_remaining_   = 0;
    uint64_t           current_episode_id_   = 0;
    float              held_accel_           = 0.0f;
    uint64_t           episodes_armed_       = 0;

    // Adaptive-baseline state — tracks |Δurgency| against its own running EMA
    // so the gate adapts to whatever scale this run's urgency happens to take
    // (saturated, mid, low).  TLE-analog: detect anomalously-low recent
    // change against historical change.
    bool     drive_seen_              = false;
    float    prev_urgency_            = 0.0f;
    float    long_change_ema_         = 0.0f;
    //
    // Outcome-feedback state (Phase 3): track Δurgency between episode arm
    // and episode end; EMA the success rate (1 = urgency dropped, 0 = no
    // improvement).  Effective episode_ticks and accel_jitter scale with
    // (1 - success_rate): when episodes consistently fail, escalate by up
    // to 2× — when they succeed, return to baseline.  No fixed gains; the
    // "rate of escalation" is the empirical success rate itself.
    float    arm_urgency_             = 0.0f;
    float    success_rate_ema_        = 0.5f;     // initialise mid-range
    // Phase 6.1+: counted from events.hit during the active episode and
    // used as the success signal at episode end (replaces the urgency-drop
    // proxy noted as "too rare" in the prior comment).  events.hit is the
    // same outcome signal MotorRepertoire's chunk crystallisation uses.
    int      episode_hits_            = 0;
    //
    // Long-history sliding-window buffers of urgency and of the per-tick
    // short/long change ratio.  Gate fires only when current samples are
    // in the statistical tail of these distributions (95th percentile of
    // urgency, 5th percentile of ratio at default anomaly_percentile=5).
    // Sliding-window percentiles (vs EMA mean+std) are robust to step
    // changes in the input that would inflate EMA variance estimates.
    std::deque<float>  urgency_long_buffer_;
    std::deque<float>  ratio_long_buffer_;
    uint64_t           sample_count_  = 0;

    std::mt19937_64    episode_rng_;

    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).

public:
    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const&) override;
};

} // namespace ogma
