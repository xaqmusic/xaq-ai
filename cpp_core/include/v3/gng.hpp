#pragma once

/**
 * GNG — Growing Neural Gas (v3 C++ implementation)
 *
 * Ports src/ami_ogma_v3/gng.py to C++.
 *
 * Reference: Fritzke (1995) "A Growing Neural Gas Network Learns Topologies"
 *
 * Key features matching the Python implementation:
 *   - Bootstrap from first 2 real inputs
 *   - Stability-dampened winner/neighbour learning rates
 *   - Two-gate baking: frequency (visits) + consistency (ema_error)
 *   - Demotion on consistency failure
 *   - Baked nodes are frozen (not moved, not pruned)
 *   - Convergence guard: insertion suppressed when ema_error < min_insertion_error
 *   - Stale-prune: remove non-baked nodes not visited in stale_window steps
 *   - Baked-q insertion: when highest-error node is baked, insert at last input
 *   - Serialisation: to_json() / from_json() matching schema v2
 */

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdint>
#include <limits>
#include <optional>

namespace ami_ogma {
namespace v3 {

struct GNGNode {
    Eigen::VectorXf prototype;
    double  error             = 0.0;
    double  ema_error         = 0.0;
    int     visits            = 0;
    int     last_visited_step = 0;
    bool    bake_checked      = false;  // one-shot: consistency gate has fired
    // Post-bake tracking for Mitosis Gatekeeper
    int     post_bake_visits  = 0;
    double  post_bake_error   = 0.0;

    // --- Biological health model ---
    // Health grows with activity and decays non-linearly: mature nodes
    // (high health) decay much slower than young nodes (low health).
    // Replaces the old utility-based mass culling with per-node lifecycle.
    //
    // Biological analogy: synaptic consolidation. Young synapses are
    // volatile (LTD-prone). Heavily-used synapses develop LTP and become
    // increasingly resistant to decay. Death is gradual, individual, smooth.
    float   health            = 1.0f;   // accumulated activity strength
};

class GNG {
public:
    // ---------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------

    struct Config {
        int   dim                 = 128;
        int   max_nodes           = 2000;
        float epsilon_b           = 0.05f;
        float epsilon_n           = 0.003f;
        int   max_age             = 88;
        int   lambda_new          = 25;
        float alpha               = 0.5f;
        float beta                = 0.0005f;
        int   baking_threshold    = 100;
        float min_insertion_error = 0.02f;
        bool  stale_prune_enabled = true;
        float stale_window_factor = 12000.0f; // absolute steps (~400s at 30fps)
        // Mitosis Gatekeeper
        bool  mitosis_enabled         = true;
        float mitosis_error_threshold = 0.30f;
        int   mitosis_check_interval  = 50;
        float mitosis_split_distance  = 0.10f;
        // Biological health model (replaces metabolic mass-culling)
        float health_boost             = 0.5f;    // health gained per visit (activity-dependent potentiation)
        float health_base_decay        = 0.995f;  // decay rate at health=0 (young, volatile)
        float health_resilience_k      = 0.08f;   // how quickly health → resilience (higher = faster consolidation)
        float health_death_threshold   = 0.01f;   // health below this → node dies
        int   max_deaths_per_tick      = 1;        // at most 1 death per tick (smooth gradient)
        // Pressure gate (2) — skip death sweep when the population is small.
        // Below this count the GNG is still establishing coverage; culling
        // "acquired then forgotten" nodes there is the main cause of 30→4 collapse.
        int   health_death_min_nodes   = 16;
        // Cooldown (5) — minimum steps between two deaths. Prevents per-tick
        // snowballing when several nodes drop below threshold at once.
        int   death_cooldown_steps     = 25;      // matches default lambda_new
        // Smooth protection (3) — fraction of baking_threshold above which a
        // node is "near-baked": half-speed health decay + immune to
        // isolated/stale pruning. Catches concepts that are clearly forming
        // but haven't crossed the binary bake gate yet.
        float near_baked_fraction      = 0.6f;
    };

    explicit GNG(const Config& cfg);

    // ---------------------------------------------------------------------------
    // Public API (matches Python GNG interface)
    // ---------------------------------------------------------------------------

    /**
     * Process one input vector. Updates topology and returns winner.
     *
     * @param x  Input vector of size cfg.dim
     * @return   {winner_node_id, quantization_error}
     *           Returns {0, 0.0} during the 2-sample bootstrap window.
     */
    std::pair<int, float> step(const Eigen::VectorXf& x);

    // Node access
    int   node_count()    const;
    int   baked_count()   const;
    float crystallization_ratio() const;
    float context_novelty(const Eigen::VectorXf& x) const;

    // Retrieve prototype by stable node ID (returns empty optional if not found)
    std::optional<Eigen::VectorXf> get_prototype(int node_id) const;

    int  get_visit_count(int node_id) const;
    bool is_crystallised(int node_id) const;

    /// True if the winner from the last step() just crossed the baking gate.
    bool last_step_baked() const { return last_step_baked_; }

    // Consensus-driven bake acceleration: boost a node's visit count
    // toward the baking threshold without moving its prototype.
    // Returns true if the node exists and was boosted.
    bool boost_visits(int node_id, int amount);

    // Running mean squared quantization error (EMA, α=0.05) — for diagnostics
    float running_mean_error() const { return running_mean_error_; }

    // Prune all non-baked nodes manually (e.g. on explicit reset)
    int prune_unbaked();

    // ---------------------------------------------------------------------------
    // Runtime-adjustable parameters (wired to UI sliders)
    // ---------------------------------------------------------------------------

    void set_stale_prune_enabled(bool enabled) { cfg_.stale_prune_enabled = enabled; }
    void set_stale_window_factor(float factor) { cfg_.stale_window_factor = factor; }
    void set_min_insertion_error(float e)      { cfg_.min_insertion_error = e; }
    void set_epsilon_b(float e)                { cfg_.epsilon_b = e; }
    void set_epsilon_n(float e)                { cfg_.epsilon_n = e; }
    void set_lambda_new(int l)                 { cfg_.lambda_new = l; }
    void set_max_age(int a)                    { cfg_.max_age = a; }
    void set_baking_threshold(int t)           { cfg_.baking_threshold = t; }
    void set_mitosis_enabled(bool e)           { cfg_.mitosis_enabled = e; }
    void set_mitosis_error_threshold(float t)  { cfg_.mitosis_error_threshold = t; }
    void set_mitosis_check_interval(int n)     { cfg_.mitosis_check_interval = n; }
    void set_mitosis_split_distance(float d)   { cfg_.mitosis_split_distance = d; }

    int  mitosis_count() const { return mitosis_count_; }

    const std::vector<int>& last_pruned_ids() const { return last_pruned_ids_; }

    // Returns true if a split occurred
    bool maybe_mitosis(int winner_id, const Eigen::VectorXf& x);

    // Read-only accessors for current config
    const Config& config() const { return cfg_; }
    Config& config_ref() { return cfg_; }
    int step_count() const { return step_; }

    // Lightweight visit-count snapshot — for diag streams that don't
    // need the full to_json() payload (prototypes + edges + history).
    // Iterates nodes_ in unspecified order; consumers that need ranking
    // sort the returned vector themselves.
    std::vector<int> visit_counts() const {
        std::vector<int> out;
        out.reserve(nodes_.size());
        for (auto const& [_, n] : nodes_) out.push_back(n.visits);
        return out;
    }

    // ---------------------------------------------------------------------------
    // Serialisation — schema 2, matching Python to_dict() / from_dict()
    // ---------------------------------------------------------------------------

    nlohmann::json to_json() const;
    static GNG from_json(const nlohmann::json& j);

private:
    Config cfg_;

    // Node storage: stable node ID → node data
    std::unordered_map<int, GNGNode> nodes_;
    int next_id_ = 0;

    // Edge storage: packed edge key (min_id << 32 | max_id) → age
    std::unordered_map<uint64_t, int> edges_;
    // Adjacency: node_id → set of neighbour node_ids
    std::unordered_map<int, std::unordered_set<int>> adj_;

    // Step counter
    int step_ = 0;

    // Mitosis counter
    int mitosis_count_ = 0;

    // Set by step() when the winner just crossed the baking gate
    bool last_step_baked_ = false;

    // Metabolic Cleanup (D1)
    std::vector<int> last_pruned_ids_;

    // Step of the most recent death (for cooldown gate). -inf-equivalent at start.
    int last_death_step_ = -1000000;

    // Bootstrap state
    bool bootstrapped_ = false;
    std::vector<Eigen::VectorXf> bootstrap_buf_;

    // Last input (used for baked-q insertion placement)
    std::optional<Eigen::VectorXf> last_x_;

    // Running mean squared quantization error
    float running_mean_error_ = 1.0f;

    // Recent winner history (last 32 node IDs)
    std::vector<int> history_;

    // ---------------------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------------------

    int  add_node(const Eigen::VectorXf& prototype);
    void kill_node(int id);

    uint64_t edge_key(int a, int b) const {
        int lo = std::min(a, b), hi = std::max(a, b);
        return (static_cast<uint64_t>(lo) << 32) | static_cast<uint32_t>(hi);
    }

    void add_edge(int a, int b);
    void remove_edge(int a, int b);

    // Find the two nearest nodes; returns {s1_id, s2_id, d1, d2}
    struct NearestResult { int s1_id, s2_id; float d1, d2; };
    NearestResult find_two_nearest(const Eigen::VectorXf& x) const;

    void remove_isolated();
    void prune_stale_unbaked();
    void insert_node();
};

} // namespace v3
} // namespace ami_ogma
