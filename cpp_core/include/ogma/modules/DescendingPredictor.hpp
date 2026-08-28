#pragma once

// =============================================================================
// DescendingPredictor.hpp  --  Module 6 of 9 in the Phase 1 dependency chain
// =============================================================================
//
// Contract:        docs/primitives/DescendingPredictor.md
// v3 reference:    none (new primitive — closes the top-down loop v3 lacked).
//
// Per-target AR(1) linear predictor:
//
//     predicted_latent_target = W_target · consensus_embedding + b_target
//
// One W and b per declared target.  Online supervised update each tick:
//
//     1. Read reality.<target>(t-1) via Feedback   (the EPM's actual residual)
//     2. error = reality(t-1).latent - cached_prediction
//     3. SGD step: W += lr · outer(error, cached_consensus); b += lr · error
//     4. Read consensus(t) (Direct, current tick)
//     5. Forward pass → prediction.<target>(t)
//     6. Cache (consensus(t), prediction(t)) for the next tick's supervisory pair
//
// EPM subscribes to prediction.<target> via Feedback so it sees the
// previous-tick prediction (subtracted before its GNG step).

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class DescendingPredictor : public Module {
public:
    DescendingPredictor();
    ~DescendingPredictor() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    void           restore_state(nlohmann::json const& s) override;

    // Read-only accessors for white-box tests.
    int    target_count() const { return int(targets_.size()); }
    float  confidence(std::string const& target) const;
    Eigen::MatrixXf const* weights(std::string const& target) const;

    // Per-target diagnostics for the audit — by index so the host can
    // iterate without knowing topic strings.  Surface err_ema, norm_ema
    // and weight shape so a JSONL diag can verify the predictor is
    // actually training (not just initialised) per modality.
    std::string const& target_label(size_t i) const { return targets_[i].label; }
    std::string const& target_topic(size_t i) const { return targets_[i].topic; }
    float              target_err_ema(size_t i)  const { return targets_[i].err_ema; }
    float              target_norm_ema(size_t i) const { return targets_[i].norm_ema; }
    int                target_W_rows(size_t i)   const { return int(targets_[i].W.rows()); }
    int                target_W_cols(size_t i)   const { return int(targets_[i].W.cols()); }
    bool               target_cached_valid(size_t i) const { return targets_[i].cached_valid; }

private:
    struct Target {
        std::string         topic;          // full reality.<group>.<modality> name
        std::string         out_topic;      // prediction.<group>.<modality>
        std::string         label;          // <group>.<modality>
        Eigen::MatrixXf     W;              // (target_dim × source_dim)
        Eigen::VectorXf     b;              // (target_dim)
        Eigen::VectorXf     cached_prediction;
        bool                cached_valid    = false;
        // Confidence tracking — rolling mean of |error| / |target|.
        float               err_ema         = 1.0f;
        float               norm_ema        = 1.0f;
    };

    void  handle_consensus(std::string_view topic, MessagePtr payload);
    void  handle_reality(std::string_view topic, MessagePtr payload, size_t target_idx);

    // Configuration
    std::string  consensus_topic_   = "consensus.0";
    std::string  update_method_     = "sgd";
    float        learning_rate_     = 0.01f;
    float        rls_forget_        = 0.99f;
    float        init_noise_scale_  = 0.01f;
    int64_t      freeze_after_ticks_= 0;
    int64_t      confidence_window_ = 100;
    uint64_t     master_seed_       = 0;
    bool         target_is_residual_ = false;   // target publishes residual → integrate it

    int          source_dim_        = 0;
    bool         dim_known_         = false;

    std::vector<Target>                          targets_;
    std::unordered_map<std::string, size_t>      target_idx_by_topic_;

    // Latest consensus payload + cached vector at last forward pass.
    Eigen::VectorXf                              latest_consensus_;
    bool                                         consensus_seen_   = false;
    Eigen::VectorXf                              cached_consensus_;
    bool                                         cached_consensus_valid_ = false;

    uint64_t                                     ticks_run_        = 0;
    // sub_ids_ lives on Module base (Phase 6.6.A teardown fix).
};

} // namespace ogma
