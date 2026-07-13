#pragma once

// =============================================================================
// SynergyTimer.hpp  --  Closed-loop adaptive timer (Phase 7.9)
// =============================================================================
//
// Addresses the open-loop CPG failure mode (Phase 7.x): an oscillator
// running independently of body state can be in the wrong phase relative
// to an emerging gait and cancel it.  SynergyTimer derives phase FROM
// body touchdown events, so it can't mismatch the body's actual rhythm.
//
// Architecture:
//   reality.proprio.feet_y  ──▶ touchdown detection (adaptive hysteresis)
//                                ▼
//                          per-leg state:
//                              last_touchdown_tick
//                              inter_touchdown_ema (period)
//                              predicted_touchdown_tick
//                              predicted_touchdown_error_ema
//                              rhythm_confidence
//                              phase, phase_bin
//                                ▼
//   events.hit / events.miss ──▶ Hebbian update of
//                                   B[premotor, phase_bin, intent]
//   policy.<premotor> ────────▶ (observe chosen intent per Premotor)
//                                ▼
//                          Output (per tick, per Premotor):
//                              rhythm.bias.<premotor_id>
//                              = rhythm_bias_gain × confidence[leg]
//                                × B[premotor, phase_bin[leg], :]
//
// Output composes additively with W (REINFORCE), epistemic_gain, and
// value_head_gain inside Premotor's softmax — no override.
//
// Safety:
//   * confidence → 0 during standing or chaotic stumbling → bias → 0
//   * reward-gated update only — non-reward-correlated rhythms don't
//     strengthen B
//   * decay_per_tick prevents runaway
//   * rhythm_bias_gain = 0 (default) → byte-identical legacy behaviour

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class SynergyTimer : public Module {
public:
    SynergyTimer();
    ~SynergyTimer() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json snapshot_state() const override;

    // Inspector / diag accessors.
    int                       n_legs()          const { return int(leg_state_.size()); }
    int                       n_premotors()     const { return int(premotor_state_.size()); }
    int                       n_bins()          const { return n_bins_; }
    int                       n_intents()       const { return n_intents_; }
    float                     rhythm_bias_gain() const { return rhythm_bias_gain_; }
    std::vector<float>        rhythm_confidence_all() const;
    std::vector<float>        period_ema_all() const;
    std::vector<int>          phase_bin_all() const;
    std::vector<int>          touchdown_count_all() const;

private:
    struct LegState {
        std::string name;                 // "fl"|"fr"|"rl"|"rr"
        bool        is_planted             = false;
        // Adaptive thresholds.
        float       feet_y_low_ema         = 0.0f;
        float       feet_y_high_ema        = 0.0f;
        bool        thresholds_initialised = false;
        // Predictive state.
        int64_t     last_touchdown_tick    = -1;
        float       inter_touchdown_ema    = 0.0f;
        float       predicted_error_ema    = 0.0f;
        int64_t     predicted_touchdown_tick = -1;
        float       rhythm_confidence      = 0.0f;
        float       phase                  = 0.0f;
        int         phase_bin              = 0;
        int         touchdown_count        = 0;
    };

    struct PremotorState {
        std::string  id;
        int          leg_idx     = -1;        // index into leg_state_
        std::string  policy_topic;
        // B[bin][intent] — synergy table for this premotor.
        // Stored as (n_bins × n_intents) matrix.
        Eigen::MatrixXf B;
        int          last_chosen_intent = -1;
        Eigen::VectorXf last_bias_published;   // n_intents
    };

    void handle_feet_y(MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);
    void handle_policy(int premotor_idx, MessagePtr payload);

    void update_leg(int leg_idx, float foot_y, uint64_t tick_id);
    void on_touchdown(int leg_idx, uint64_t tick_id);
    void apply_reward(float reward_signed, uint64_t tick_id);
    void publish_biases(uint64_t tick_id);

    // ---- Config ----
    std::string feet_y_topic_       = "reality.proprio.feet_y";
    int         n_bins_             = 8;
    int         n_intents_          = 5;
    float       rhythm_bias_gain_   = 0.0f;
    float       min_confidence_     = 0.2f;
    float       alpha_period_       = 0.05f;
    float       alpha_err_          = 0.05f;
    float       learning_rate_      = 0.01f;
    float       decay_per_tick_     = 1e-5f;
    // 2026-06-10 E1 — self-supervised rhythm bias.  When > 0, B(bin, chosen) is
    // reinforced every tick by self_supervised_rate × rhythm_confidence WITHOUT a
    // reward gate.  Amplifies the body's OWN phase-correlated intent structure
    // (the weak emergent shuffle, foot_autocorr ~0.4) into more coherent stepping
    // — phase-binned, so it strengthens phase-DEPENDENT alternation rather than
    // collapsing to one intent.  Bounded by decay_per_tick + bias_cap.  0 = off.
    float       self_supervised_rate_ = 0.0f;
    float       bias_cap_             = 2.0f;   // clamp on |B| cells
    float       hysteresis_low_frac_  = 0.25f;
    float       hysteresis_high_frac_ = 0.50f;
    float       hysteresis_ema_alpha_ = 0.01f;
    int         max_period_ticks_   = 180;     // 3 sec at 60Hz — soft cap
    int         min_period_ticks_   = 10;
    bool        publish_when_silent_ = true;   // true = publish zero bias to keep Premotor's
                                                //         subscription warm; false = no publish

    // ---- State ----
    std::vector<LegState>      leg_state_;
    std::vector<PremotorState> premotor_state_;
    uint64_t                   current_tick_ = 0;
};

} // namespace ogma
