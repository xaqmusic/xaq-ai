/**
 * GNG — Growing Neural Gas implementation (v3 C++ port of gng.py)
 */

#include "v3/gng.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <iostream>

namespace ami_ogma {
namespace v3 {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

GNG::GNG(const Config& cfg) : cfg_(cfg) {}

// ---------------------------------------------------------------------------
// Node management
// ---------------------------------------------------------------------------

int GNG::add_node(const Eigen::VectorXf& prototype) {
    int id = next_id_++;
    GNGNode& n = nodes_[id];
    n.prototype         = prototype;
    n.error             = 0.0;
    n.ema_error         = 0.0;
    n.visits            = 0;
    n.last_visited_step = step_;
    n.health            = 1.0f;  // born young — must earn resilience through visits
    adj_[id];           // ensure adjacency entry exists (empty set)
    return id;
}

void GNG::kill_node(int id) {
    // Remove all edges
    auto it = adj_.find(id);
    if (it != adj_.end()) {
        for (int nb : it->second) {
            edges_.erase(edge_key(id, nb));
            auto nb_it = adj_.find(nb);
            if (nb_it != adj_.end())
                nb_it->second.erase(id);
        }
        adj_.erase(it);
    }
    nodes_.erase(id);
}

void GNG::add_edge(int a, int b) {
    if (a == b) return;
    edges_[edge_key(a, b)] = 0;
    adj_[a].insert(b);
    adj_[b].insert(a);
}

void GNG::remove_edge(int a, int b) {
    edges_.erase(edge_key(a, b));
    auto it_a = adj_.find(a);
    if (it_a != adj_.end()) it_a->second.erase(b);
    auto it_b = adj_.find(b);
    if (it_b != adj_.end()) it_b->second.erase(a);
}

// ---------------------------------------------------------------------------
// Find two nearest nodes — O(N) scan
// ---------------------------------------------------------------------------

GNG::NearestResult GNG::find_two_nearest(const Eigen::VectorXf& x) const {
    NearestResult r{-1, -1,
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity()};

    for (const auto& [id, node] : nodes_) {
        float d = (node.prototype - x).norm();
        if (d < r.d1) {
            r.d2 = r.d1; r.s2_id = r.s1_id;
            r.d1 = d;    r.s1_id = id;
        } else if (d < r.d2) {
            r.d2 = d;    r.s2_id = id;
        }
    }
    return r;
}

// ---------------------------------------------------------------------------
// step() — one GNG update tick
// ---------------------------------------------------------------------------

std::pair<int, float> GNG::step(const Eigen::VectorXf& x) {
    // Bootstrap: collect first 2 real inputs before running normal GNG
    if (!bootstrapped_) {
        bootstrap_buf_.push_back(x);
        if (static_cast<int>(bootstrap_buf_.size()) >= 2) {
            add_node(bootstrap_buf_[0]);
            add_node(bootstrap_buf_[1]);
            bootstrapped_ = true;
            bootstrap_buf_.clear();
        }
        return {0, 0.0f};
    }

    last_pruned_ids_.clear();
    ++step_;
    last_x_ = x;

    // 1. Find winner (s1) and runner-up (s2)
    auto [s1_id, s2_id, d1, d2] = find_two_nearest(x);
    if (s1_id < 0) return {0, 0.0f};  // no nodes (shouldn't happen post-bootstrap)

    GNGNode& s1 = nodes_.at(s1_id);

    // 2. Accumulate error at winner; update short-term EMA
    double d1_sq = static_cast<double>(d1) * d1;
    s1.error     += d1_sq;
    s1.ema_error  = 0.9 * s1.ema_error + 0.1 * d1_sq;

    // Insertion-gate self-tuning: remember this step's squared quantisation
    // error so the gate can be set from the signal's OWN error distribution.
    // Accumulated only when enabled, so the off-path allocates nothing.
    if (cfg_.insertion_autotune) {
        autotune_hist_.push_back(d1_sq);
        if (autotune_hist_.size() > kAutotuneHistoryMax) autotune_hist_.pop_front();
    }

    // 3. Move winner toward input — health-dampened plasticity.
    //    High-health nodes move less (consolidated). Fully consolidated
    //    (visits >= baking_threshold AND low error) are frozen.
    if (s1.visits < cfg_.baking_threshold) {
        // Stability from both visits AND health: health adds smooth damping
        float visit_stability = static_cast<float>(s1.visits) / cfg_.baking_threshold;
        float health_damping = std::min(1.0f, s1.health * 0.02f); // health=50 → 100% damped
        float stability = std::max(visit_stability, health_damping);
        float eff_eb    = cfg_.epsilon_b * (1.0f - 0.9f * stability);
        s1.prototype   += eff_eb * (x - s1.prototype);
    }

    // 4. Move neighbours toward input (skip consolidated neighbours)
    if (adj_.count(s1_id)) {
        for (int nb_id : adj_.at(s1_id)) {
            auto it = nodes_.find(nb_id);
            if (it == nodes_.end()) continue;
            GNGNode& nb = it->second;
            if (nb.visits >= cfg_.baking_threshold) continue;
            float nb_visit_stab = static_cast<float>(nb.visits) / cfg_.baking_threshold;
            float nb_health_damp = std::min(1.0f, nb.health * 0.02f);
            float nb_stab = std::max(nb_visit_stab, nb_health_damp);
            float eff_en  = cfg_.epsilon_n * (1.0f - 0.9f * nb_stab);
            nb.prototype += eff_en * (x - nb.prototype);
        }
    }

    // 5. Age s1's edges; remove old ones.
    //    Edges connected to high-health, consolidated, or near-baked nodes
    //    are exempt — they form the stable skeleton of crystallized memory.
    {
        const int near_baked_visits =
            static_cast<int>(cfg_.baking_threshold * cfg_.near_baked_fraction);
        auto is_protected = [&](int id) {
            auto it = nodes_.find(id);
            if (it == nodes_.end()) return false;
            const auto& n = it->second;
            return n.visits >= cfg_.baking_threshold
                || n.visits >= near_baked_visits
                || n.health > 20.0f;
        };
        std::vector<std::pair<int,int>> to_remove;
        bool s1_consolidated = is_protected(s1_id);
        if (adj_.count(s1_id)) {
            for (int nb_id : adj_.at(s1_id)) {
                uint64_t ek = edge_key(s1_id, nb_id);
                auto it = edges_.find(ek);
                if (it == edges_.end()) continue;
                // Skip age-out if either endpoint is consolidated / near-baked
                if (s1_consolidated || is_protected(nb_id)) continue;
                ++(it->second);
                if (it->second > cfg_.max_age)
                    to_remove.emplace_back(s1_id, nb_id);
            }
        }
        for (auto [a, b] : to_remove)
            remove_edge(a, b);
    }

    // 6. Refresh / create s1–s2 edge
    if (s2_id >= 0 && s1_id != s2_id)
        add_edge(s1_id, s2_id);

    // 7. Remove isolated non-baked nodes (keep ≥ 2)
    remove_isolated();

    // 8. Periodic: insertion + stale prune
    if (step_ % cfg_.lambda_new == 0) {
        if (node_count() < cfg_.max_nodes)
            insert_node();
        prune_stale_unbaked();
    }

    // 9. Biological health decay — per-node, non-linear.
    //    Mature nodes (high health) decay much slower than young nodes.
    //    decay_rate = base_decay ^ (1 / (1 + health * resilience_k))
    //    At health=0:  decay ≈ base_decay (fast, volatile)
    //    At health=50: decay ≈ base_decay^0.2 (very slow, consolidated)
    //
    //    This replaces the binary baked/unbaked utility system with a
    //    smooth gradient that mirrors synaptic consolidation.
    {
        int deaths = 0;
        float worst_health = 1e9f;
        int worst_id = -1;
        const int near_baked_visits =
            static_cast<int>(cfg_.baking_threshold * cfg_.near_baked_fraction);

        for (auto& [id, node] : nodes_) {
            node.error *= (1.0 - cfg_.beta);

            // Non-linear decay: exponent shrinks as health grows
            float exponent = 1.0f / (1.0f + node.health * cfg_.health_resilience_k);
            float decay = std::pow(cfg_.health_base_decay, exponent);
            // (3) Near-baked nodes decay at half speed — partially consolidated
            // concepts get time to finish forming rather than being culled as
            // "acquired then forgotten".
            if (node.visits >= near_baked_visits && node.visits < cfg_.baking_threshold) {
                decay = 0.5f * (1.0f + decay);  // halve the distance from 1.0
            }
            node.health *= decay;

            // Track the weakest node for potential death (near-baked nodes
            // remain eligible, but near-baked decay is halved above, so they
            // rarely reach the threshold).  With the spares-baked guard on,
            // BAKED nodes are exempt — see the config note: earned regimes are
            // permanent facts, not casualties of a long absence.
            if (cfg_.health_death_spares_baked
                && node.visits >= cfg_.baking_threshold) continue;
            if (node.health < worst_health) {
                worst_health = node.health;
                worst_id = id;
            }
        }

        // 10. Per-node death: only the weakest, one at a time.
        //     No mass culling — smooth, gradual attrition.
        //     Gated by (2) population floor and (5) cooldown so a burst of
        //     low-health nodes can't snowball into a collapse.
        bool above_floor = node_count() > cfg_.health_death_min_nodes;
        bool cooldown_ok = (step_ - last_death_step_) >= cfg_.death_cooldown_steps;
        if (worst_id >= 0 && worst_health < cfg_.health_death_threshold
            && node_count() > 2 && deaths < cfg_.max_deaths_per_tick
            && above_floor && cooldown_ok) {
            last_pruned_ids_.push_back(worst_id);
            kill_node(worst_id);
            deaths++;
            last_death_step_ = step_;
        }
    }

    // 11. Running mean squared QE (EMA α=0.05)
    running_mean_error_ = 0.95f * running_mean_error_ + 0.05f * static_cast<float>(d1_sq);

    // 12. Update winner stats: visits, last-visited, health boost
    s1.visits++;
    s1.last_visited_step = step_;
    s1.health = std::min(100.0f, s1.health + cfg_.health_boost); // Activity-dependent potentiation

    history_.push_back(s1_id);
    if (static_cast<int>(history_.size()) > 32)
        history_.erase(history_.begin());

    // 12. Consistency gate (fires once when node first reaches baking_threshold)
    last_step_baked_ = false;
    if (!s1.bake_checked && s1.visits >= cfg_.baking_threshold) {
        s1.bake_checked = true;
        if (s1.ema_error >= effective_min_insertion_error()) {
            // Demotion: concept not tight enough
            s1.bake_checked = false;  // allow re-check after demotion
            s1.visits     = std::max(0, cfg_.baking_threshold - 3);
            s1.ema_error *= 0.5;
            s1.error     *= 0.5;
        } else {
            last_step_baked_ = true;
        }
    }

    // 13. Post-bake tracking for Mitosis Gatekeeper
    if (s1.visits > cfg_.baking_threshold) {
        s1.post_bake_visits++;
        s1.post_bake_error += static_cast<double>(d1_sq);
    }

    return {s1_id, d1};
}

// ---------------------------------------------------------------------------
// Remove isolated non-baked nodes (keep ≥ 2)
// ---------------------------------------------------------------------------

void GNG::remove_isolated() {
    if (node_count() <= 2) return;

    const int near_baked_visits =
        static_cast<int>(cfg_.baking_threshold * cfg_.near_baked_fraction);
    std::vector<int> to_kill;
    for (const auto& [id, node] : nodes_) {
        // Consolidated, near-baked, or high-health nodes survive isolation
        if (node.visits >= cfg_.baking_threshold) continue;
        if (node.visits >= near_baked_visits) continue;
        if (node.health > 5.0f) continue;
        auto it = adj_.find(id);
        if (it == adj_.end() || it->second.empty())
            to_kill.push_back(id);
    }
    for (int id : to_kill) {
        if (node_count() <= 2) break;
        kill_node(id);
    }
}

// ---------------------------------------------------------------------------
// Stale-prune: remove non-baked nodes not visited recently
// ---------------------------------------------------------------------------

void GNG::prune_stale_unbaked() {
    if (!cfg_.stale_prune_enabled || node_count() <= 2) return;

    // stale_window is an absolute step count — independent of baking_threshold
    int stale_window = static_cast<int>(cfg_.stale_window_factor);
    int cutoff = step_ - stale_window;

    const int near_baked_visits =
        static_cast<int>(cfg_.baking_threshold * cfg_.near_baked_fraction);
    std::vector<int> to_kill;
    for (const auto& [id, node] : nodes_) {
        // Baked, near-baked, or high-health nodes are immune to stale pruning
        if (node.visits >= cfg_.baking_threshold) continue;
        if (node.visits >= near_baked_visits) continue;
        if (node.health > 5.0f) continue;
        if (node.last_visited_step < cutoff)
            to_kill.push_back(id);
    }
    for (int id : to_kill) {
        if (node_count() <= 2) break;
        last_pruned_ids_.push_back(id);
        kill_node(id);
    }
}

// ---------------------------------------------------------------------------
// Node insertion (Fritzke + baked-q extension)
// ---------------------------------------------------------------------------

// Recompute the self-tuned insertion floor from the recent squared-TLE
// distribution.  Linear-interpolated quantile, matching numpy's default so the
// C++ tracks the v3 Python reference rather than merely resembling it.
void GNG::update_autotune_threshold() {
    if (!cfg_.insertion_autotune) return;
    if (autotune_hist_.size() < kAutotuneWarmup) return;   // stays < 0 until warm
    std::vector<double> sorted(autotune_hist_.begin(), autotune_hist_.end());
    std::sort(sorted.begin(), sorted.end());
    const double q   = std::clamp(double(cfg_.insertion_autotune_quantile), 0.0, 1.0);
    const double pos = q * double(sorted.size() - 1);
    const size_t lo  = static_cast<size_t>(pos);
    const size_t hi  = std::min(lo + 1, sorted.size() - 1);
    const double frac = pos - double(lo);
    autotune_value_ = static_cast<float>(sorted[lo] * (1.0 - frac) + sorted[hi] * frac);
}

void GNG::insert_node() {
    if (static_cast<int>(nodes_.size()) < 2) return;

    // Refresh the gate at the same cadence as the insertion check itself
    // (every lambda_new steps), which is where the reference put it.
    update_autotune_threshold();

    // q = node with maximum accumulated error
    int q_id = -1;
    double max_err = -1.0;
    for (const auto& [id, node] : nodes_) {
        if (node.error > max_err) {
            max_err = node.error;
            q_id    = id;
        }
    }
    if (q_id < 0) return;

    // Convergence guard: skip if ema_error is below threshold
    if (nodes_.at(q_id).ema_error < effective_min_insertion_error()) return;

    // f = neighbour of q with maximum accumulated error
    int f_id = -1;
    double f_max_err = -1.0;
    if (adj_.count(q_id)) {
        for (int nb_id : adj_.at(q_id)) {
            auto it = nodes_.find(nb_id);
            if (it == nodes_.end()) continue;
            if (it->second.error > f_max_err) {
                f_max_err = it->second.error;
                f_id      = nb_id;
            }
        }
    }
    if (f_id < 0) return;

    GNGNode& q = nodes_.at(q_id);
    GNGNode& f = nodes_.at(f_id);

    bool q_is_baked = (q.visits >= cfg_.baking_threshold);

    // Determine insertion position
    Eigen::VectorXf new_proto;
    if (q_is_baked && last_x_.has_value()) {
        new_proto = last_x_.value();  // insert at current input
    } else {
        new_proto = 0.5f * (q.prototype + f.prototype);  // midpoint
    }

    int r_id = add_node(new_proto);

    // Connect new node and redistribute errors
    if (q_is_baked) {
        add_edge(r_id, q_id);
        if (f_id != q_id)
            add_edge(r_id, f_id);
    } else {
        remove_edge(q_id, f_id);
        add_edge(q_id, r_id);
        add_edge(r_id, f_id);
    }

    // Need to re-fetch references since nodes_ may have reallocated
    nodes_.at(q_id).error *= cfg_.alpha;
    nodes_.at(f_id).error *= cfg_.alpha;
    nodes_.at(r_id).error  = nodes_.at(q_id).error;
}

// ---------------------------------------------------------------------------
// Public accessors
// ---------------------------------------------------------------------------

int GNG::node_count() const {
    return static_cast<int>(nodes_.size());
}

int GNG::baked_count() const {
    int count = 0;
    for (const auto& [id, node] : nodes_)
        if (node.visits >= cfg_.baking_threshold) ++count;
    return count;
}

float GNG::crystallization_ratio() const {
    int n = node_count();
    if (n == 0) return 0.0f;
    return static_cast<float>(baked_count()) / n;
}

float GNG::context_novelty(const Eigen::VectorXf& x) const {
    float min_d = std::numeric_limits<float>::infinity();
    for (const auto& [id, node] : nodes_) {
        if (node.visits < cfg_.baking_threshold) continue;
        float d = (node.prototype - x).norm();
        if (d < min_d) min_d = d;
    }
    return min_d;
}

std::optional<Eigen::VectorXf> GNG::get_prototype(int node_id) const {
    auto it = nodes_.find(node_id);
    if (it == nodes_.end()) return std::nullopt;
    return it->second.prototype;
}

int GNG::get_visit_count(int node_id) const {
    auto it = nodes_.find(node_id);
    return (it != nodes_.end()) ? it->second.visits : 0;
}

bool GNG::is_crystallised(int node_id) const {
    return get_visit_count(node_id) >= cfg_.baking_threshold;
}

bool GNG::boost_visits(int node_id, int amount) {
    auto it = nodes_.find(node_id);
    if (it == nodes_.end()) return false;
    auto& node = it->second;
    // Only boost unbaked nodes — already-baked nodes don't need acceleration
    if (node.visits >= cfg_.baking_threshold) return false;
    node.visits = std::min(node.visits + amount, cfg_.baking_threshold);
    return true;
}

int GNG::prune_unbaked() {
    if (node_count() <= 2) return 0;
    std::vector<int> to_kill;
    for (const auto& [id, node] : nodes_)
        if (node.visits < cfg_.baking_threshold)
            to_kill.push_back(id);
    int pruned = 0;
    for (int id : to_kill) {
        if (node_count() <= 2) break;
        kill_node(id);
        ++pruned;
    }
    return pruned;
}

// ---------------------------------------------------------------------------
// Topology reset — for an owner that has rescaled the input space
// ---------------------------------------------------------------------------

void GNG::reset_topology() {
    nodes_.clear();
    edges_.clear();
    adj_.clear();
    step_             = 0;
    mitosis_count_    = 0;
    last_step_baked_  = false;
    last_pruned_ids_.clear();
    last_death_step_  = -1000000;
    bootstrapped_     = false;
    bootstrap_buf_.clear();
    last_x_.reset();
    running_mean_error_ = 1.0f;
    // next_id_ is NOT reset — see the header contract.
}

// ---------------------------------------------------------------------------
// Mitosis Gatekeeper — split a saturated baked node into two daughters
// ---------------------------------------------------------------------------

bool GNG::maybe_mitosis(int winner_id, const Eigen::VectorXf& x) {
    if (!cfg_.mitosis_enabled) return false;
    if (node_count() >= cfg_.max_nodes - 1) return false;

    auto it = nodes_.find(winner_id);
    if (it == nodes_.end()) return false;
    GNGNode& q = it->second;

    // Only consider baked nodes that have enough post-bake data
    if (q.visits <= cfg_.baking_threshold) return false;
    if (q.post_bake_visits < cfg_.mitosis_check_interval) return false;

    // Check post-bake mean error against threshold
    double mean_pb_error = q.post_bake_error / q.post_bake_visits;
    if (mean_pb_error < cfg_.mitosis_error_threshold) {
        // Not saturated — reset window and continue
        q.post_bake_visits = 0;
        q.post_bake_error  = 0.0;
        return false;
    }

    // --- Split ---
    // Direction: from prototype toward current input (temporal gradient)
    Eigen::VectorXf dir = x - q.prototype;
    float norm = dir.norm();
    if (norm < 1e-6f) {
        // Degenerate: use random perturbation
        dir = Eigen::VectorXf::Random(q.prototype.size());
        norm = dir.norm();
    }
    dir /= norm;

    // Scale offset as fraction of prototype norm (or fixed small value)
    float proto_norm = q.prototype.norm();
    float offset = cfg_.mitosis_split_distance * (proto_norm > 1e-6f ? proto_norm : 1.0f);

    Eigen::VectorXf proto_a = q.prototype + offset * dir;
    Eigen::VectorXf proto_b = q.prototype - offset * dir;

    // Collect parent's neighbours before killing it
    std::vector<int> neighbours;
    if (adj_.count(winner_id))
        neighbours.assign(adj_.at(winner_id).begin(), adj_.at(winner_id).end());

    kill_node(winner_id);

    int da = add_node(proto_a);
    int db = add_node(proto_b);

    // Connect daughters to each other and to all former neighbours
    add_edge(da, db);
    for (int nb : neighbours) {
        if (nodes_.count(nb)) {
            add_edge(da, nb);
            add_edge(db, nb);
        }
    }

    ++mitosis_count_;
    return true;
}

// ---------------------------------------------------------------------------
// Serialisation — schema 2 (mirrors Python to_dict / from_dict)
// ---------------------------------------------------------------------------

nlohmann::json GNG::to_json() const {
    nlohmann::json j;
    // Schema 3 (Phase 6.5.4): added per-node bake_checked + health, and
    // module-level running_mean_error_, last_step_baked_, last_death_step_,
    // history_ — required for OgmaInstance::clone() byte-equivalence.
    j["schema"]              = 3;
    j["dim"]                 = cfg_.dim;
    j["baking_threshold"]    = cfg_.baking_threshold;
    j["min_insertion_error"] = cfg_.min_insertion_error;
    j["lambda_new"]          = cfg_.lambda_new;
    j["max_age"]             = cfg_.max_age;
    j["stale_prune_enabled"] = cfg_.stale_prune_enabled;
    j["stale_window_factor"] = cfg_.stale_window_factor;
    j["step"]                = step_;
    j["next_id"]             = next_id_;
    j["mitosis_count"]       = mitosis_count_;
    j["running_mean_error"]  = running_mean_error_;
    // Insertion-gate self-tuning state.  Emitted ONLY when enabled, so a GNG
    // with autotune off serialises byte-identically to the pre-feature form.
    // The history MUST round-trip: a restored GNG that re-warms from an empty
    // history has no gate for its first kAutotuneWarmup steps and grows
    // unbounded in exactly that window -- same class of bug as EPM Invariant 11.
    if (cfg_.insertion_autotune) {
        j["insertion_autotune"]          = cfg_.insertion_autotune;
        j["insertion_autotune_quantile"] = cfg_.insertion_autotune_quantile;
        j["autotune_value"]              = autotune_value_;
        j["autotune_hist"]               = autotune_hist_;
    }
    j["last_step_baked"]     = last_step_baked_;
    j["last_death_step"]     = last_death_step_;
    j["history"]             = history_;
    if (last_x_) {
        std::vector<float> lx(last_x_->data(), last_x_->data() + last_x_->size());
        j["last_x"] = lx;
    } else {
        j["last_x"] = nullptr;
    }

    nlohmann::json nodes_arr = nlohmann::json::array();
    for (const auto& [id, node] : nodes_) {
        nlohmann::json nj;
        nj["id"]               = id;
        nj["error"]            = node.error;
        nj["ema_error"]        = node.ema_error;
        nj["visits"]           = node.visits;
        nj["last_visited_step"]= node.last_visited_step;
        nj["bake_checked"]     = node.bake_checked;
        nj["post_bake_visits"] = node.post_bake_visits;
        nj["post_bake_error"]  = node.post_bake_error;
        nj["health"]           = node.health;
        std::vector<float> proto(node.prototype.data(),
                                  node.prototype.data() + node.prototype.size());
        nj["prototype"] = proto;
        nodes_arr.push_back(nj);
    }
    j["nodes"] = nodes_arr;

    nlohmann::json edges_arr = nlohmann::json::array();
    for (const auto& [ek, age] : edges_) {
        int lo = static_cast<int>(ek >> 32);
        int hi = static_cast<int>(ek & 0xFFFFFFFFu);
        nlohmann::json ej;
        ej["positions"] = {lo, hi};
        ej["age"]       = age;
        edges_arr.push_back(ej);
    }
    j["edges"] = edges_arr;

    return j;
}

GNG GNG::from_json(const nlohmann::json& j) {
    GNG::Config cfg;
    cfg.dim                 = j.value("dim",                 128);
    cfg.baking_threshold    = j.value("baking_threshold",    100);
    cfg.min_insertion_error = j.value("min_insertion_error", 0.02f);
    cfg.insertion_autotune          = j.value("insertion_autotune", false);
    cfg.insertion_autotune_quantile = j.value("insertion_autotune_quantile", 0.30f);
    cfg.lambda_new          = j.value("lambda_new",          25);
    cfg.max_age             = j.value("max_age",             88);
    cfg.stale_prune_enabled = j.value("stale_prune_enabled", true);
    cfg.stale_window_factor = j.value("stale_window_factor", 3.0f);

    GNG gng(cfg);
    gng.step_          = j.value("step",          0);
    gng.next_id_       = j.value("next_id",       0);
    gng.mitosis_count_ = j.value("mitosis_count", 0);
    gng.autotune_value_ = j.value("autotune_value", -1.0f);
    if (j.contains("autotune_hist") && j["autotune_hist"].is_array()) {
        auto h = j["autotune_hist"].get<std::vector<double>>();
        gng.autotune_hist_.assign(h.begin(), h.end());
    }
    gng.bootstrapped_  = true;
    // Schema-3 additions (Phase 6.5.4); fall back to defaults when reading
    // older snapshots so existing on-disk artifacts still load.
    gng.running_mean_error_ = j.value("running_mean_error", 1.0f);
    gng.last_step_baked_    = j.value("last_step_baked",    false);
    gng.last_death_step_    = j.value("last_death_step",    -1000000);
    if (j.contains("history") && j["history"].is_array()) {
        gng.history_ = j["history"].get<std::vector<int>>();
    }
    if (j.contains("last_x") && !j["last_x"].is_null()) {
        auto lx_vec = j["last_x"].get<std::vector<float>>();
        gng.last_x_ = Eigen::Map<const Eigen::VectorXf>(lx_vec.data(), lx_vec.size());
    }

    for (const auto& nj : j.at("nodes")) {
        int id = nj.at("id").get<int>();
        GNGNode& node = gng.nodes_[id];
        auto proto_vec = nj.at("prototype").get<std::vector<float>>();
        node.prototype = Eigen::Map<const Eigen::VectorXf>(proto_vec.data(),
                                                            proto_vec.size());
        node.error             = nj.value("error",             0.0);
        node.ema_error         = nj.value("ema_error",         0.0);
        node.visits            = nj.value("visits",            0);
        node.last_visited_step = nj.value("last_visited_step", 0);
        node.bake_checked      = nj.value("bake_checked",      false);
        node.post_bake_visits  = nj.value("post_bake_visits",  0);
        node.post_bake_error   = nj.value("post_bake_error",   0.0);
        node.health            = nj.value("health",            1.0f);
        gng.adj_[id];   // ensure adjacency entry
    }

    for (const auto& ej : j.at("edges")) {
        auto positions = ej.at("positions").get<std::vector<int>>();
        int age = ej.value("age", 0);
        if (positions.size() == 2) {
            int a = positions[0], b = positions[1];
            gng.edges_[gng.edge_key(a, b)] = age;
            gng.adj_[a].insert(b);
            gng.adj_[b].insert(a);
        }
    }

    return gng;
}

} // namespace v3
} // namespace ami_ogma
