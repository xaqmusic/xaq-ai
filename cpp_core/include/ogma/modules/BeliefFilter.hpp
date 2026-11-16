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
// THE CONSISTENCY CHECK (learned stepper Phase 3, 2026-11-15; diag only, the outputs unchanged): the normalised innovation
// squared of each fresh observation against the PREDICTED belief's moment-matched mixture -- m = sum b-(j) mu_j,
// S = sum b-(j) (var_j + mu_j^2) - m^2 -- averaged over the dimensions with variance: its expected value is 1 for a filter
// whose stated spread is honest (>1 overconfident, <1 underconfident).  nis (this tick), nis_ema, nis_mean; and how often
// the MAP node and the EPM's winner change (a belief that coasts through flicker switches less than the raw winner).  head_nis:
// the same statistic for the EPM winner's own decode head alone, taken before the head learns from the reading -- the heads'
// calibration, free of the transition model (the predictive NIS above averages 1 only over all ticks, jumps included).
// snapshot_state / restore_state carry the learned model (transitions, decode heads, belief) across runs.
// THE HEADS' OWN STATISTICS (head_update "own", learned stepper Phase 3, operator 2026-11-15; "legacy" = the default, byte-identical):
// a new node's decode head starts from its first sample, not the global mean and variance, and keeps the exact running mean and
// variance of its own samples (gain 1/n down to gain_floor, no gain_cap), read with the small-sample correction n/(n-1).  The
// legacy head started at the global variance and moved at most gain_cap per sample, so a node seen a few dozen times kept a spread
// far wider than its own -- NIS 0.3-0.6 on the stepper's stance vocabulary.
// THE HEADS' VISITS (head_update "visits", 2026-11-15): on a tick-rate stream a run of wins is one observation, and a head
// built from one ~2 s dwell holds ~100 near-identical samples -- "own" heads read NIS ~2.5 on the stairs for that reason.  Here
// a head's n counts VISITS (entries of the winner): each visit is folded in once, at its end, as its mean (the "own" recursion
// over visit means, so between-visit spread), and its spread about that mean is pooled into a within-visit variance.  The
// head's variance is the two summed -- the predictive spread of a tick in a new visit, the between part as the Student-t
// predictive of n estimated visits, var (n + 1)/(n - 3).  min_count then counts visits.
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
    nlohmann::json snapshot_state() const override;
    void restore_state(nlohmann::json const& s) override;
    double  nis()       const { return nis_; }
    double  nis_mean()  const { return nis_n_ > 0 ? nis_sum_ / double(nis_n_) : 0.0; }
    double  head_nis()  const { return head_nis_; }
    double  head_nis_mean() const { return head_nis_n_ > 0 ? head_nis_sum_ / double(head_nis_n_) : 0.0; }

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
    bool   head_own_    = false;  // head_update "own" (or "visits"): a head's statistics are its own samples' (see the header)
    bool   head_visits_ = false;  // head_update "visits": its n counts visits, not ticks (see the header)
    // head_update "visits": per node the pooled within-visit sum of squares and ticks; the open visit's node id and sums
    std::vector<Eigen::VectorXf> wss_; std::vector<double> wT_;
    int vis_j_ = -1; double vis_len_ = 0.0; Eigen::VectorXd vis_sum_, vis_ss_;
    void close_visit();
    Eigen::VectorXf head_var(int j) const;   // a node's decode-head variance as the likelihood reads it

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
    double nis_ = 0.0, nis_ema_ = 0.0, nis_sum_ = 0.0; int64_t nis_n_ = 0;   // the consistency check (diag only)
    double head_nis_ = 0.0, head_nis_sum_ = 0.0; int64_t head_nis_n_ = 0;      // the winner's decode head alone (diag only)
    int prev_map_ = -1, prev_winner_ = -1; int64_t map_switches_ = 0, winner_switches_ = 0;
};

} // namespace ogma
