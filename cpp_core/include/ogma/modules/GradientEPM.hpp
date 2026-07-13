#pragma once

// =============================================================================
// GradientEPM.hpp  --  meta-EPM scalar-gradient follower (generic follow / flee)
// =============================================================================
//
// The action module for the SCENT-GRADIENT strange-loop (operator design, 2026-06-26).
// Generic: follows (or flees) ANY scalar gradient. It is a META-EPM — an EPM on top of
// the loop's CONSENSUS (scent-EPM ⊕ compass-EPM ⊕ CPG-EPM fused by a LateralVoter), made
// action-conditioned: it predicts the FUTURE Δscalar consequence of a candidate heading.
//
//   node  = cluster over  [ proj(consensus) ⊕ scalar-trend ⊕ sin(action) ⊕ cos(action) ]
//   each node learns  pred_Δscalar  = EMA(realized Δscalar that followed)   (its forward model)
//   a node BAKES when its Δscalar prediction is consistent (low TLE + enough visits)
//   OUTPUT heading = argmax over candidate headings of predicted future Δscalar   (follow)
//
// Two inputs, because scalar scent is special (1-D → no coarse-graining needed):
//   - the CONSENSUS (coarse fused context: orientation [compass], phase [CPG], coarse scent)
//   - the DIRECT scalar (kept as a fine EMA trend AND the prediction target).
//
// This is the directed upgrade of the run-and-tumble value table: the 1-scalar trend → a
// rich EPM-clustered consensus state, and run/tumble → a continuous heading chosen by a
// small forward-model sweep (the loop-level "choose a pathway into the future", §2). It
// stays honest: predicts with the agent's OWN Δscalar (§4), EPM-clustered (§5), and feeds
// the FULL advance-on HeadingController (§5 — never a stripped loop). Default-off / opt-in.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nlohmann/json_fwd.hpp>

namespace ogma {

class GradientEPM : public Module {
public:
    GradientEPM() = default;
    ~GradientEPM() override = default;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void                   on_param_change(std::string_view key, ParamValue const& value) override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;

    nlohmann::json diag_snapshot() const override;

    // accessors (tests / telemetry)
    int   node_count()   const { return int(nodes_.size()); }
    int   baked_count()  const;
    float last_pred()    const { return last_pred_; }       // predicted Δscalar of the chosen heading
    float last_dscalar() const { return last_dscalar_; }    // last realized Δscalar
    float chosen_heading() const { return committed_heading_; }

private:
    struct Node {
        Eigen::VectorXf proto;   // [proj_consensus (P) ⊕ trend ⊕ sin_a ⊕ cos_a]
        float pred = 0.0f;       // EMA predicted Δscalar
        float tle  = 0.0f;       // EMA |realized − pred| (consistency)
        int   visits = 0;
        bool  baked = false;
    };

    void handle_consensus(MessagePtr payload);
    void handle_scalar(MessagePtr payload);
    void handle_heading(MessagePtr payload);
    void handle_hit(MessagePtr payload);

    Eigen::VectorXf project(Eigen::VectorXf const& consensus);          // 128→P random projection (unit)
    Eigen::VectorXf feature(Eigen::VectorXf const& proj_c, float trend, float a) const;
    int   nearest(Eigen::VectorXf const& x, bool baked_only) const;    // -1 if none
    float select_heading();                                            // forward-model sweep → argmax (absolute)
    void credit_and_adapt();                                           // close the committed cycle

    std::string consensus_topic_ = "consensus.scent_loop";  // ConsensusToken.fused_embedding
    std::string scalar_topic_    = "reality.proprio.scent_max";
    std::string heading_topic_   = "reality.proprio.heading";
    std::string hit_topic_       = "events.hit";
    std::string output_topic_    = "percept.gradient_heading";

    int   proj_dim_        = 16;     // consensus random-projection target
    int   n_headings_      = 12;     // candidate absolute headings swept at select
    int   cycle_ticks_     = 30;     // commit window (measure Δscalar over this)
    float value_lr_        = 0.15f;  // pred_Δscalar EMA rate
    float tle_lr_          = 0.1f;   // consistency EMA rate
    bool  reward_norm_     = true;   // whiten Δscalar by running scale
    float reward_scale_lr_ = 0.02f;
    float r_hit_           = 1.0f;
    float insertion_dist_  = 0.6f;   // GNG: new node if nearest farther than this
    int   max_nodes_       = 64;
    float adapt_lr_        = 0.05f;  // GNG winner move toward feature
    int   bake_visits_     = 8;      // min visits to bake
    float bake_tle_        = 0.5f;   // max TLE (whitened units) to bake
    float temperature_     = 0.3f;   // softmax over candidate Δscalar (<=0 = argmax)
    float epistemic_gain_  = 0.4f;   // novelty bonus: prefer under-modeled headings
    float trend_ema_       = 0.2f;   // scalar-trend EMA rate
    int   mode_            = 1;      // +1 follow (climb), −1 flee
    float w_head_          = 1.0f;   // heading weight in the feature distance
    uint64_t master_seed_  = 7;

    // latest inputs
    Eigen::VectorXf proj_c_;         // projected consensus (P)
    bool  have_consensus_ = false;
    float scalar_ = 0.0f, trend_ = 0.0f, prev_scalar_ = 0.0f, heading_ = 0.0f;
    bool  have_scalar_ = false, have_prev_ = false;
    bool  hit_in_cycle_ = false;

    Eigen::MatrixXf proj_R_;         // P×128 random projection
    int   consensus_dim_ = 0;        // learned from the first token
    std::vector<Node> nodes_;
    float reward_scale_ = 0.0f;

    // committed cycle
    int   ticks_left_ = 0;
    bool  have_commit_ = false;
    Eigen::VectorXf commit_feat_;    // feature of the committed (state ⊕ action)
    float commit_scalar_ = 0.0f;     // scalar at commit (for realized Δ)
    float committed_heading_ = 0.0f; // absolute chosen heading
    float last_pred_ = 0.0f, last_dscalar_ = 0.0f;

    std::mt19937 rng_{7};
};

}  // namespace ogma
