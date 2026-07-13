#pragma once

// =============================================================================
// FaderController.hpp  --  Phase 6.6.G α-computation primitive
// =============================================================================
//
// Splits the α-computation half out of Phase 6.6.F's MotorFader so that
// multiple MotorFader channels (left + right in the bilateral Cell, or
// any N-actuator body) share one coherent α.  The controller owns all
// alpha_source / alpha_fixed / alpha_smoothing / alpha_min / alpha_max /
// surprise_aggregation params and publishes a FaderState on
// `alpha_topic` (default `motor.fader.alpha`).  Each MotorFader
// subscribes to that topic and reads `alpha` to blend its own
// (brain, reflex) ActionOut pair.
//
// Backwards compatibility: existing single-channel configs that did NOT
// declare a FaderController still work because MotorFader retains a
// fallback to its own `alpha_fixed` default when no FaderState ever
// arrives — implemented in Phase 6.6.G step 2.

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class FaderController : public Module {
public:
    FaderController();
    ~FaderController() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // White-box accessors for tests.
    float alpha()              const { return alpha_; }
    float alpha_target()       const { return alpha_target_; }
    float boredom_term()       const { return last_boredom_; }
    float alpha_long_ema()     const { return alpha_long_ema_; }
    float surprise_scalar()    const { return surprise_scalar_; }
    int   action_msgs_received() const { return action_msgs_received_; }
    int   policy_msgs_received() const { return policy_msgs_received_; }
    float last_premotor_entropy() const { return last_premotor_entropy_; }
    int   last_premotor_n_intents() const { return last_premotor_n_intents_; }
    int   chunk_active_ticks()   const { return chunk_active_ticks_; }
    int   current_chunk_id_diag() const { return current_chunk_id_; }
    float familiarity_scalar() const { return familiarity_scalar_; }
    float learned_setpoint()   const { return learned_alpha_setpoint_; }
    float reward_ema()         const { return reward_ema_; }
    int   publish_count()      const { return publish_count_; }

private:
    void handle_consensus(MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);
    void handle_policy(MessagePtr payload);   // v5.4.M premotor_certainty

    // Compute α_target from current cached state.  Returns alpha_min when
    // alpha_source == "surprise" but no consensus has arrived yet
    // (cold start = reflex-dominated, the safe default).
    float compute_alpha_target() const;

    // Aggregate per-modality surprise EMA into a single scalar.
    float aggregate_surprise(
        std::unordered_map<std::string, float> const& m) const;

    // Configuration
    std::string consensus_topic_  = "consensus.0";
    std::string alpha_topic_      = topics::kMotorFaderAlpha;
    // Reserved for the deferred urgency-driven panic channel — wired as
    // a subscription target in 6.6.G but not yet consulted in
    // compute_alpha_target.  Empty disables the subscription entirely.
    std::string urgency_topic_    = "";
    std::string alpha_source_     = "surprise";   // "fixed" | "surprise" | "chunk_gated"
    float       alpha_fixed_      = 0.0f;
    // v5.4 Phase B — chunk-gated α.  When alpha_source="chunk_gated", read
    // ActionOut.chunk_id on action_topic_; α flips between alpha_chunk_active
    // (when a chunk is replaying) and alpha_chunk_idle (otherwise).  Hand-
    // tuned diagnostic to test "suppress reflex when brain has high-
    // certainty plan" — the dynamic equivalent would be α =
    // sigmoid(entry_match_certainty), but knob-twiddling first gives us
    // direction for the adaptive design.
    std::string action_topic_         = "action.out";
    float       alpha_chunk_active_   = 1.0f;
    float       alpha_chunk_idle_     = 0.5f;
    int         current_chunk_id_     = -1;        // updated by handle_action
    // v5.4 Phase F (Proposal B) — continuous α = sigmoid(chunk_quality).
    // When alpha_source="chunk_quality_sigmoid", FaderController reads
    // ActionOut.chunk_quality each tick and computes:
    //   α = α_min + (α_max − α_min) × σ(k × (quality − midpoint))
    // where σ is the logistic sigmoid.  When chunk_quality = 0
    // (no chunk active), α tapers to α_min (1 − σ(k × midpoint))-ish;
    // when quality = 1, α tapers to α_max.  Smoother handoff than
    // the binary chunk_gated mode.
    float       current_chunk_quality_ = 0.0f;
    float       alpha_quality_k_       = 8.0f;
    float       alpha_quality_midpoint_= 0.55f;
    float       alpha_quality_min_     = 0.5f;
    float       alpha_quality_max_     = 0.95f;
    // v5.4.M — premotor_certainty α source.  α modulated by Premotor's
    // instantaneous softmax entropy (high entropy = uncertain policy =
    // let reflex drive; low entropy = committed policy = brain takes
    // authority).
    float       alpha_certainty_min_   = 0.30f;
    float       alpha_certainty_max_   = 0.95f;
    float       last_premotor_entropy_ = 0.0f;
    int         last_premotor_n_intents_ = 0;
    int         policy_msgs_received_  = 0;   // v5.4.M debug
    int         action_msgs_received_ = 0;
    int         chunk_active_ticks_   = 0;
    float       alpha_smoothing_  = 0.05f;
    float       alpha_min_        = 0.0f;
    float       alpha_max_        = 1.0f;
    std::string surprise_aggregation_ = "mean";   // "mean" | "max"
    // Phase 6.6.K — pathway-familiarity coupling to α.  When the EPMs
    // keep predicting states the agent has visited many times AND the
    // predictions stay accurate (low surprise), the agent is in a rut:
    // surprise alone can't tell us we're stuck because we ARE
    // accurately predicting that we're stuck.  Couple in a per-(modality,
    // node_id) state-visit-EMA computed from ConsensusToken's
    // winner_ids_by_modality + predicted_pathways.  When familiarity is
    // high, push α down so reflexes can break the loop.
    //
    //   effective = max(surprise_scalar, familiarity_scalar)
    //   target    = clamp(1 - effective, alpha_min, alpha_max)
    //
    // No new tunable knob — both signals are already auto-calibrated:
    // surprise via 6.6.J Bayesian shrinkage in the voter; familiarity
    // via accumulated visit counts.  state_visit_alpha defaults to the
    // same 0.05 used elsewhere; pathway_alpha_coupling=1.0 means full
    // coupling, 0 disables.
    float       state_visit_alpha_     = 0.05f;
    float       pathway_alpha_coupling_ = 0.0f;   // default off; bilateral configs opt in
    std::unordered_map<std::string, std::unordered_map<int, float>> state_visit_ema_;

    // Phase 6.6.N — learned α setpoint via reward feedback.
    //
    // The 6.6.K familiarity coupling drags α toward 0 over long runs
    // because every state eventually gets revisited (visit_ema → 1 →
    // familiarity → 1 → α_target → 0). User observed this as "reflex
    // dominates after many minutes." The fix is to replace the
    // hardcoded "α = 1 - signal" with a SETPOINT learned from
    // reward feedback: the brain earns its α by demonstrating that
    // high-α periods correlate with hits.
    //
    // Per-tick update (when learned_alpha_lr_ > 0):
    //   reward_ema += reward_alpha * (signed_reward(events) - reward_ema)
    //   setpoint   += learned_alpha_lr * (alpha_ - setpoint) * reward_ema
    //   α_target   = clamp(setpoint * (1 - surprise_scalar),
    //                      alpha_min, alpha_max)
    //
    // The surprise term stays as a fast tactical override (drops α on
    // bad predictions) but no longer dominates the slow-time signal.
    // Familiarity coupling is bypassed when learned mode is on — the
    // correlation already captures whatever familiarity should mean
    // for this env's reward landscape.
    //
    // No hand-tuning: the setpoint converges to whatever mix the
    // env actually rewards.  default 0 = legacy 6.6.K behaviour
    // preserved.
    float       learned_alpha_lr_           = 0.0f;
    float       learned_alpha_setpoint_init_ = 0.5f;
    float       learned_alpha_setpoint_     = 0.5f;   // mutable runtime state
    float       reward_alpha_               = 0.05f;
    float       alpha_long_alpha_           = 0.005f; // slow EMA of α (~200 ticks)
    float       reward_weight_hit_          = 1.0f;
    float       reward_weight_miss_         = -0.5f;
    float       reward_weight_wall_stuck_   = -0.5f;
    float       reward_ema_                 = 0.0f;
    float       alpha_long_ema_             = 0.5f;   // initialised at setpoint
    bool        learned_warmed_up_          = false;
    float       last_reward_signal_         = 0.0f;

    // Phase 6.6.Q — boredom-α floor.  Adds boredom_gain * (1 - alpha_long_ema)
    // to compute_alpha_target's surprise-driven target.  Independent of the
    // 6.6.K familiarity coupling and the 6.6.N learned setpoint — this is
    // the homeokinetic "stability is itself surprising" channel.
    float       boredom_gain_   = 0.0f;
    float       last_boredom_   = 0.0f;

    // Working state
    float alpha_              = 0.0f;
    float alpha_target_       = 0.0f;
    float surprise_scalar_    = 0.0f;
    float familiarity_scalar_ = 0.0f;
    int   publish_count_      = 0;

    // Latest consensus we ever saw (last-value-style).  Consensus may
    // arrive less often than every tick; we still want α to reflect the
    // most recent surprise even on consensus-silent ticks.
    std::shared_ptr<const ConsensusToken> last_consensus_;
};

} // namespace ogma
