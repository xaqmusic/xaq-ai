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
#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>

namespace ami_ogma {
namespace v3 {

// Winner-update gain schedule (Kalman-lessons Stage 1,
// docs/plans-and-designs/epm_kalman_lessons_plan.md).
//   Linear — the legacy anneal eps_b * (1 - 0.9 * visits / N), frozen at bake.
//   Kalman — per-node scalar Kalman gain; see Config below.
enum class GainKind { Linear, Kalman };

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

    // Prior-variance ratio for the per-node Kalman gain, in units of the
    // node's own observation noise (so the schedule is dimensionless).  Read
    // only when Config::gain_kind == GainKind::Kalman, and serialised only
    // then, so a Linear-mode snapshot is byte-identical to the pre-feature form.
    float   p                 = 1.0f;
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

        // ---------------------------------------------------------------------
        // Ecological self-tuning of the insertion gate
        // ---------------------------------------------------------------------
        //
        // `min_insertion_error` is an ABSOLUTE error threshold, so it only means
        // anything relative to the typical quantisation error of the signal in
        // front of it.  Held fixed, on a stream whose typical error sits above
        // it, insertion never stops being justified and the GNG grows until it
        // hits max_nodes -- measured on the picrawler motor layer 2026-08-06:
        // nodes pinned at the cap, and RAISING the cap 120 -> 400 made baking
        // WORSE in absolute terms (19-27 baked -> 6-11), i.e. the cap was the
        // only thing forcing revisits.  Flat ema_tle for 12000 ticks confirmed
        // it never converged.
        //
        // The fix is not a better constant.  The v3 Python reference
        // (python/xaq/xaq/gng.py:105-114, 676-685) had the GNG pick its own
        // floor from the 30th percentile of its recent squared-TLE distribution
        // -- "the split between stable/don't-grow and surprising/grow tracks
        // whatever input density the environment actually presents."  Only the
        // frozen debug branch of that survived the port to C++, while EPM.md
        // went on documenting the auto-tuning as present.  This restores it.
        //
        // The quantile is a RANK -- dimensionless, invariant to the signal's
        // units -- which is what makes this adaptive rather than one more
        // constant tuned to a signal's scale (doctrine rule 5).
        //
        // Effective gate = max(min_insertion_error, quantile) * neuro_scale.
        // The configured value is therefore the FLOOR, which is what the EPM
        // contract always claimed it was.  ⚠ This DIVERGES from the Python
        // reference, which replaced the threshold outright; a pure replacement
        // lets the gate collapse toward zero in a low-error regime and growth
        // runs away again.
        //
        // false (default) = exactly the pre-2026-08-06 fixed-threshold path.
        bool  insertion_autotune          = false;
        float insertion_autotune_quantile = 0.30f;

        // ---------------------------------------------------------------------
        // Per-node Kalman gain (Kalman-lessons Stage 1)
        // ---------------------------------------------------------------------
        //
        // The winner update w += g (x - w) has the form of the Kalman filter
        // for a constant, but the legacy schedule g_n = eps_b (1 - 0.9 n/N)
        // is not its gain: measured on the bench (2026-09-05) a baked
        // prototype keeps 24 % of its weight on the point it was born at and
        // carries 2x the MSE of the mean of the same samples.  The Kalman gain
        // for a constant is 1/(n+1).
        //
        // In Kalman mode each node carries p, its prior variance as a ratio
        // of its own observation noise.  Per win:
        //     p += kalman_q;  K = min(kalman_gain_cap, p / (p + 1));
        //     w += K (x - w);  p *= (1 - K).
        // With kalman_p0 = 1 and kalman_q = 0 that is exactly 1/(n+1) (the
        // seed counts as one sample) and baked nodes stay frozen.  With
        // kalman_q > 0 every node — baked included — settles at the random-
        // walk steady-state gain (q + sqrt(q^2 + 4q))/2 and tracks slow drift
        // instead of waiting for mitosis.  eps_b, the visit/health damping and
        // the neurochemical eps_b scale are not consulted in this mode; the
        // neighbour pull eps_n is unchanged.
        //
        // Linear (default) leaves step() byte-identical.
        GainKind gain_kind       = GainKind::Linear;
        float    kalman_p0       = 1.0f;
        float    kalman_q        = 0.0f;
        float    kalman_gain_cap = 1.0f;

        bool  stale_prune_enabled = true;
        float stale_window_factor = 12000.0f; // absolute steps (~400s at 30fps)
        // 2026-09-01 (guarded; default false = byte-identical everywhere): exempt
        // BAKED nodes from the health-death sweep.  The health system replaced
        // binary baked-immunity with a smooth gradient — and thereby silently
        // removed the "baked = permanent" contract the header still promises:
        // during a long perturbation (fall, inversion, rescue) an unvisited baked
        // node's health decays to the death threshold in minutes, the operator's
        // observed prune-then-relearn cascade.  Measured on the microduck regime
        // EPM: 25 of 41 node ids dead within 50 minutes, the standing regime's
        // identity churning 1→16→29 — which orphans any consumer keyed by
        // winner_id.  A REGIME vocabulary is a set of permanent facts about the
        // body; earned nodes should not be forgotten for the crime of a long
        // absence.
        bool  health_death_spares_baked = false;
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

    /**
     * Drop the entire topology and return to the pre-bootstrap state, as if
     * step() had never been called — but WITHOUT recycling node IDs.
     *
     * The only supported caller is an owner that has just changed the meaning
     * of its input space underneath the GNG (see EPM's `dim_autocal_ticks`:
     * the commissioning window earns a vocabulary in provisional units, then
     * rescales, at which point every prototype is expressed in the old units
     * and is garbage).  Keeping such a vocabulary would leave the map moving
     * under a topology that BAKES, which is the one thing baking assumes
     * cannot happen.
     *
     * `next_id_` deliberately survives, so IDs issued after the reset can
     * never collide with IDs a downstream consumer saw before it (EPM.md
     * Invariant 4: "once issued, an ID is never reused even after pruning").
     * A consumer holding a stale winner_id therefore sees an ID that no
     * longer resolves — which is honest — rather than one silently rebound
     * to an unrelated region of a different space.
     */
    void reset_topology();

    // ---------------------------------------------------------------------------
    // Runtime-adjustable parameters (wired to UI sliders)
    // ---------------------------------------------------------------------------

    void set_stale_prune_enabled(bool enabled) { cfg_.stale_prune_enabled = enabled; }
    void set_health_death_spares_baked(bool v) { cfg_.health_death_spares_baked = v; }
    void set_stale_window_factor(float factor) { cfg_.stale_window_factor = factor; }
    void set_min_insertion_error(float e)      { cfg_.min_insertion_error = e; }

    // Neurochemical modulation of the insertion gate.
    //
    // ⚠ ONLY consulted when insertion_autotune is on.  In the legacy path the
    // owner folds the scale into set_min_insertion_error() itself, and that
    // must keep working byte-identically -- so with autotune off this value is
    // ignored entirely rather than applied twice.  When autotune IS on the
    // owner passes the UNSCALED floor above and the scale here, so the scale
    // multiplies the auto-tuned gate: dopamine then widens or narrows growth
    // relative to the body's CURRENT typical surprise instead of relative to a
    // fixed constant.
    void set_neuro_min_insertion_scale(float s) { neuro_min_insertion_scale_ = s; }

    /// The threshold actually applied this step (see Config::insertion_autotune).
    float effective_min_insertion_error() const {
        if (!cfg_.insertion_autotune) return cfg_.min_insertion_error;
        return std::max(cfg_.min_insertion_error, autotune_value_)
             * neuro_min_insertion_scale_;
    }

    /// Diagnostic: the raw quantile before the floor and scale are applied.
    /// Negative until the warmup window has filled.
    float autotune_value() const { return autotune_value_; }
    void set_epsilon_b(float e)                { cfg_.epsilon_b = e; }
    void set_epsilon_n(float e)                { cfg_.epsilon_n = e; }
    void set_kalman_q(float q)                 { cfg_.kalman_q = q; }
    void set_kalman_gain_cap(float c)          { cfg_.kalman_gain_cap = c; }
    GainKind gain_kind() const                 { return cfg_.gain_kind; }
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

    // Insertion-gate self-tuning state.  History holds recent SQUARED
    // quantisation distances, matching the units of GNGNode::ema_error (an EMA
    // of d1_sq, gng.cpp:119) -- the quantity the gate is compared against.
    // Sizes are sample COUNTS, not signal-scale constants, and match the v3
    // Python reference (deque maxlen 1000, warmup 100).
    static constexpr size_t kAutotuneHistoryMax = 1000;
    static constexpr size_t kAutotuneWarmup     = 100;
    std::deque<double> autotune_hist_;
    float              autotune_value_            = -1.0f;   // < 0 = not yet warm
    float              neuro_min_insertion_scale_ = 1.0f;
    void               update_autotune_threshold();

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
