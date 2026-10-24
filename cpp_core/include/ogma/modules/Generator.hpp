// =============================================================================
// Generator.hpp  --  the EPM's decoder (design note generator_and_belief_filter.md §3.1, stage G1)
// =============================================================================
//
// An EPM recognises: observation -> winner node.  Nothing ran the other way.  The Generator attaches to one EPM's
// RealityToken stream and, for each of its nodes k, keeps the running mean mu_k and diagonal variance var_k of a
// TARGET (any ProprioToken streams, concatenated, in their own units) observed `horizon` ticks after k won.  It then
// publishes, for the current winner, the predicted target at t + horizon -- the measurement model H of a Bayes filter,
// earned from the stream.
//
// It is graded by its own error, always: each prediction is scored when its target arrives, against two baselines
// (the target's global running mean, and persistence: the target `horizon` ticks earlier), and with the normalised
// innovation e^2 / var (the filter-consistency statistic; ~1 per dimension when the stated variance is honest).
//
// Update schedule: the Kalman gain for a constant, 1/(n+1), clamped to [gain_floor, gain_cap] (Kalman-lessons Stage 1:
// the uncapped head chases a moving input; the floor is a small process noise so a node can follow drift).
// A node the Generator has not seen starts from the target's global statistics; a pruned node is dropped.
//
// It owns no partition of its own (CLAUDE.md §0 rule 1): it reads the EPM's.  It is an INSTRUMENT: publishing its
// prediction changes nothing unless some consumer is wired to it, and no config wires one.
// =============================================================================
#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class Generator : public Module {
public:
    Generator();
    ~Generator() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    // ---- accessors for tests ----
    int    nodes_tracked() const { return int(nodes_.size()); }
    bool   has_node(int k) const { return nodes_.count(k) > 0; }
    Eigen::VectorXf node_mean(int k) const;
    Eigen::VectorXf node_var(int k) const;
    double mse()               const { return mse_n_ > 0 ? mse_sum_ / mse_n_ : 0.0; }
    double mse_mean_baseline() const { return mse_n_ > 0 ? base_mean_sum_ / mse_n_ : 0.0; }
    double mse_persistence()   const { return mse_n_ > 0 ? base_pers_sum_ / mse_n_ : 0.0; }
    double nis()               const { return mse_n_ > 0 ? nis_sum_ / mse_n_ : 0.0; }
    int64_t scored()           const { return int64_t(mse_n_); }

private:
    struct NodeStat { Eigen::VectorXf mu, var; double n = 0.0; double err_ema = 0.0; };
    struct Pending  { int winner; Eigen::VectorXf mu, var, pers; };

    void handle_reality(MessagePtr p);
    void handle_target(size_t idx, MessagePtr p);
    NodeStat& node(int k);

    // config
    std::string reality_topic_ = "reality.bodypose.pose";
    std::vector<std::string> target_topics_;
    std::string output_topic_;
    int    horizon_     = 0;
    double gain_cap_    = 0.2;
    double gain_floor_  = 0.002;
    double var_floor_   = 1e-6;
    double var_rel_floor_ = 0.01;  // a node's variance never below this fraction of the target's global variance
    int    warmup_      = 0;      // updates before a node's predictions are scored
    int    report_every_ = 0;     // > 0: print a GEN_DIAG json line every N ticks (headless instrument)
    int    trace_every_ = 0;      // > 0: print a GEN_TRACE line (winner, prediction) every N ticks

    // state
    int dim_ = 0;
    std::vector<Eigen::VectorXf> target_parts_;
    std::vector<bool> part_fresh_;
    int winner_ = -1;
    std::unordered_map<int, NodeStat> nodes_;
    Eigen::VectorXf g_mu_, g_var_;      // global running statistics of the target (the prior; the mean baseline)
    double g_n_ = 0.0;
    std::deque<Pending> pending_;       // predictions waiting `horizon` ticks for their target
    std::deque<Eigen::VectorXf> past_targets_;   // for the persistence baseline
    // scores (running sums over scored predictions)
    double mse_sum_ = 0.0, base_mean_sum_ = 0.0, base_pers_sum_ = 0.0, nis_sum_ = 0.0, mse_n_ = 0.0;
    Eigen::VectorXd mse_dim_, nis_dim_, base_mean_dim_, base_pers_dim_;   // per target dimension
    int64_t ticks_ = 0, predicted_ = 0;
};

} // namespace ogma
