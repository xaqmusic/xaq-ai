// =============================================================================
// BeliefFilter.hpp  --  predict-then-correct over an EPM's nodes (design note generator_and_belief_filter.md §3.2, G2)
// =============================================================================
//
// The EPM names ONE winner a tick: the nearest prototype, blind to where the stream was a tick ago.  A Bayes filter
// instead carries a belief b over states, predicts it forward with the dynamics, and corrects it with the evidence.
// This module does that over the EPM's own nodes, with every part learned from the stream:
//
//   dynamics   T(i -> j)  counted from consecutive EPM winners, Dirichlet-smoothed (alpha); the smoothing is also the
//              diffusion that spreads the belief when nothing is observed (Q);
//   evidence   each node's diagonal Gaussian over the observation (mean, variance), learned from the EPM's labels
//              (the Generator's decode head, horizon 0) -- the measurement model H;
//   predict    b-(j) = sum_i b(i) T(i, j);
//   correct    b(j) proportional to b-(j) * N(obs; mu_j, var_j), only on a fresh reading (none: the belief spreads).
//
// Outputs (ProprioToken on output_topic): [filtered estimate E_b[mu] (dim), MAP node probability, belief entropy,
// predictive surprise -log p(obs | b-) per dimension].  Diag: MAP node, agreement with the EPM's winner, node count.
// An INSTRUMENT: nothing consumes it.  It owns no partition (CLAUDE.md §0 rule 1).
// =============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class BeliefFilter : public Module {
public:
    BeliefFilter();
    ~BeliefFilter() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    // ---- accessors for tests and the bench ----
    Eigen::VectorXf estimate() const { return est_; }        // E_b[mu], the filtered observation
    int     map_node()  const { return map_; }
    double  map_prob()  const { return map_p_; }
    double  entropy()   const { return entropy_; }
    double  surprise()  const { return surprise_; }
    double  belief_of(int node) const;
    int     nodes()     const { return int(ids_.size()); }

private:
    void handle_reality(MessagePtr p);
    void handle_obs(MessagePtr p);
    int  index_of(int node);                // adds a node if new
    void drop_node(int node);

    // config
    std::string reality_topic_ = "reality.bodypose.pose";
    std::string obs_topic_     = "reality.proprio.joints";
    std::string output_topic_;
    double alpha_       = 0.1;    // Dirichlet pseudo-count per transition (diffusion)
    double gain_cap_    = 0.2;
    double gain_floor_  = 0.002;
    double var_rel_floor_ = 0.01;
    int    min_count_   = 5;      // a node joins the correction once its decode head has this many samples

    // state: nodes in index order
    std::vector<int> ids_;
    std::unordered_map<int, int> idx_;
    std::vector<Eigen::VectorXf> mu_, var_;
    std::vector<double> n_;
    std::vector<std::vector<double>> C_;     // transition counts C[i][j]
    std::vector<double> b_;                  // belief
    int last_winner_ = -1, winner_ = -1;
    Eigen::VectorXf obs_, g_mu_, g_var_;
    double g_n_ = 0.0;
    bool obs_fresh_ = false, winner_fresh_ = false;
    int dim_ = 0;
    // outputs
    Eigen::VectorXf est_;
    int map_ = -1;
    double map_p_ = 0.0, entropy_ = 0.0, surprise_ = 0.0;
    int64_t ticks_ = 0, corrected_ = 0, agree_ = 0, compared_ = 0;
};

} // namespace ogma
