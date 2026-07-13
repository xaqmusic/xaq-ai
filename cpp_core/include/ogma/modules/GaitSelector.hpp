#pragma once

// =============================================================================
// GaitSelector.hpp  --  Phase 8 A2: temporal gait-option selector
// =============================================================================
//
// The action-vocabulary fix for the A1 falsification.  A1 (static postures
// selected per tick) could never converge to a coherent gait: the reward
// landscape over instantaneous postures is flat (many keep the body upright),
// so per-tick credit has no gradient to organize postures into a periodic
// sequence — the policy parks at high entropy and the body random-walks.
//
// GaitSelector makes the temporal SEQUENCE the unit of selection and credit.
// It owns a small library of gait primitives (each a posture-index sequence
// over the whole-body Premotor's bilateral_table).  Each tick it streams the
// active gait's current posture index on intent.override (the whole-body
// Premotor with intent_channel=-1 reads IntentToken.index and emits that
// posture's 12 channels).  When a gait completes, GaitSelector credits it via
// REINFORCE with the return accrued over its execution window:
//
//     W.row(g) += lr * advantage * (1{g=selected} - dist[g]) * latent_at_sel
//
// (advantage = (return - running_mean) / running_std over advantage_window).
// This is the temporal credit A1 lacked: a gait that produces sustained
// outward progress over its window earns more reward AS A UNIT and is
// preferred, instead of incoherent per-tick posture jitter.
//
// force_gait_id >= 0  : open-loop mode — always dispatch that gait, no
//                       learning.  Used for the A2.0 feasibility check
//                       ("can the body walk with SOME temporal gait?").
//
// Mirrors Premotor's graded-softmax + MC-REINFORCE math (consensus latent,
// DA-modulated temperature, advantage-normalized score-function update).

#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class GaitSelector : public Module {
public:
    GaitSelector();
    ~GaitSelector() override;

    std::string_view       type_name()     const override;
    std::vector<TopicSpec> input_topics()  const override;
    std::vector<TopicSpec> output_topics() const override;
    ParamSchema            params_schema() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    // White-box accessors (diag / tests).
    int   active_gait()      const { return active_gait_id_; }
    int   num_gaits()        const { return int(gaits_.size()); }
    int   total_selections() const { return total_selections_; }
    float last_entropy()     const { return last_entropy_; }

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const&) override;
    nlohmann::json diag_snapshot() const override;

private:
    void handle_consensus(std::string_view topic, MessagePtr payload);
    void handle_event(std::string_view topic, MessagePtr payload);
    void handle_neuro(std::string_view topic, MessagePtr payload);

    int  select_gait();              // softmax sample (or force / argmax)
    void credit_completed_gait();    // REINFORCE on the just-finished gait

    // Configuration
    int          level_                  = 0;
    std::string  intent_override_topic_  = std::string(topics::kIntentOverride);
    std::vector<std::vector<int>> gaits_;        // each = posture-index sequence
    float        lr_                     = 0.05f;
    float        temperature_base_       = 1.0f;
    float        temperature_da_gain_    = 0.5f;
    bool         sample_action_          = true;
    int          force_gait_id_          = -1;    // >=0 = open-loop, no learning
    bool         advantage_normalization_ = true;
    int          advantage_window_       = 100;
    uint64_t     master_seed_            = 0;

    // Policy weights (late-init on first consensus token with known latent_dim).
    bool            weights_initialised_ = false;
    int             latent_dim_          = 0;
    Eigen::MatrixXf W_;                          // K x latent_dim
    Eigen::VectorXf b_;                          // K

    // Working state
    Eigen::VectorXf last_latent_;
    float           dopamine_            = 0.0f;
    int             active_gait_id_      = -1;
    int             gait_pos_            = 0;
    // Snapshot of the selection point, used to credit the gait on completion.
    Eigen::VectorXf sel_latent_;
    Eigen::VectorXf sel_dist_;
    int             sel_gait_            = -1;
    float           accrued_reward_      = 0.0f;
    std::deque<float> recent_returns_;

    // Diagnostics
    std::vector<int> gait_select_counts_;
    int              total_selections_   = 0;
    float            last_entropy_       = 0.0f;
    float            last_advantage_     = 0.0f;
    int              gaits_completed_    = 0;

    std::mt19937_64 rng_;
};

} // namespace ogma
