#include "ogma/modules/GNGRollout.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

#include "ogma/Rng.hpp"

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("GNGRollout param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("GNGRollout param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("GNGRollout param '" + key + "' must be bool");
}

} // namespace

GNGRollout::GNGRollout()  = default;
GNGRollout::~GNGRollout() = default;

std::string_view GNGRollout::type_name() const { return "GNGRollout"; }

std::vector<TopicSpec> GNGRollout::input_topics() const {
    return {
        TopicSpec{topics::kRolloutQuery, std::type_index(typeid(RolloutQuery)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kRealityPrefix, std::type_index(typeid(RealityToken)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kSequenceMotifPrefix, std::type_index(typeid(SequenceMotif)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kDriveErrors, std::type_index(typeid(DriveErrors)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> GNGRollout::output_topics() const {
    return { TopicSpec{topics::kRolloutResult, std::type_index(typeid(RolloutResult))} };
}

ParamSchema GNGRollout::params_schema() const {
    return {
        {"K_default",                       ParamMutability::HotMutable, "Default trajectory count", ParamValue{int64_t{32}}},
        {"M_default",                       ParamMutability::HotMutable, "Default forward horizon",  ParamValue{int64_t{5}}},
        {"transition_smoothing",            ParamMutability::HotMutable, "Laplace smoothing on edge probs", ParamValue{0.01}},
        {"motif_teleport_enabled",          ParamMutability::HotMutable, "Allow motif-driven teleport",     ParamValue{false}},
        {"motif_teleport_min_confidence",   ParamMutability::HotMutable, "Min match_confidence to teleport", ParamValue{0.5}},
        {"value_window_ticks",              ParamMutability::HotMutable, "Window for terminal-value calibration", ParamValue{int64_t{100}}},
        {"max_concurrent_queries",          ParamMutability::HotMutable, "Cap per-tick queries served", ParamValue{int64_t{4}}},
        {"master_seed",                     ParamMutability::ConstructionOnly, "RNG namespace seed", ParamValue{int64_t{0}}},
    };
}

void GNGRollout::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("GNGRollout requires a non-null Bus");

    apply_param(params, "K_default",                     [&](auto const& v){ K_default_                     = std::max(1, int(get_int(v, "K_default"))); });
    apply_param(params, "M_default",                     [&](auto const& v){ M_default_                     = std::max(1, int(get_int(v, "M_default"))); });
    apply_param(params, "transition_smoothing",          [&](auto const& v){ transition_smoothing_          = float(get_double(v, "transition_smoothing")); });
    apply_param(params, "motif_teleport_enabled",        [&](auto const& v){ motif_teleport_enabled_        = get_bool(v, "motif_teleport_enabled"); });
    apply_param(params, "motif_teleport_min_confidence", [&](auto const& v){ motif_teleport_min_confidence_ = float(get_double(v, "motif_teleport_min_confidence")); });
    apply_param(params, "value_window_ticks",            [&](auto const& v){ value_window_ticks_            = get_int(v, "value_window_ticks"); });
    apply_param(params, "max_concurrent_queries",        [&](auto const& v){ max_concurrent_queries_        = get_int(v, "max_concurrent_queries"); });
    apply_param(params, "master_seed",                   [&](auto const& v){ master_seed_                   = uint64_t(get_int(v, "master_seed")); });

    rng_ = derive_rng(master_seed_,
        std::string("rollout.") + (id_.empty() ? std::string("rollout") : id_) + ".trajectories");

    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(topics::kRealityPrefix, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_reality(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kRolloutQuery, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_query(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kSequenceMotifPrefix, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_sequence(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kDriveErrors, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_drive(t, p); }));
}

void GNGRollout::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "K_default")                       K_default_                     = std::max(1, int(get_int(value, k)));
    else if (k == "M_default")                       M_default_                     = std::max(1, int(get_int(value, k)));
    else if (k == "transition_smoothing")            transition_smoothing_          = float(get_double(value, k));
    else if (k == "motif_teleport_enabled")          motif_teleport_enabled_        = get_bool(value, k);
    else if (k == "motif_teleport_min_confidence")   motif_teleport_min_confidence_ = float(get_double(value, k));
    else if (k == "value_window_ticks")              value_window_ticks_            = get_int(value, k);
    else if (k == "max_concurrent_queries")          max_concurrent_queries_        = get_int(value, k);
    else if (k == "master_seed")
        throw std::invalid_argument("GNGRollout.master_seed is ConstructionOnly");
    else
        throw std::invalid_argument("GNGRollout: unknown param '" + k + "'");
}

void GNGRollout::handle_reality(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt || rt->winner_id < 0) return;

    std::string source = std::string(topic);
    auto& src = per_source_[source];
    if (src.last_winner >= 0) ++src.trans[src.last_winner][rt->winner_id];
    src.last_winner = rt->winner_id;
}

void GNGRollout::handle_sequence(std::string_view /*topic*/, MessagePtr /*payload*/) {
    // Phase 1.8: motif teleport is opt-in and disabled by default.  Hook
    // present so Phase 3 wires it without re-touching the contract.
}

void GNGRollout::handle_drive(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    if (auto d = std::dynamic_pointer_cast<const DriveErrors>(payload))
        latest_drive_urgency_ = d->urgency;
}

std::vector<int> GNGRollout::sample_trajectory(PerSource const& src, int start,
                                                int steps, std::mt19937_64& rng) {
    std::vector<int> traj;
    traj.reserve(steps);
    int cur = start;
    for (int s = 0; s < steps; ++s) {
        auto it = src.trans.find(cur);
        if (it == src.trans.end() || it->second.empty()) {
            // No outgoing transitions known — stay put.
            traj.push_back(cur);
            continue;
        }
        // Build a smoothed CDF over neighbour counts.
        std::vector<int>   nexts;
        std::vector<float> weights;
        nexts.reserve(it->second.size());
        weights.reserve(it->second.size());
        float total = 0.0f;
        for (auto const& [n, c] : it->second) {
            float w = float(c) + transition_smoothing_;
            nexts.push_back(n);
            weights.push_back(w);
            total += w;
        }
        std::uniform_real_distribution<float> dist(0.0f, total);
        float u = dist(rng);
        int pick = nexts.back();
        float acc = 0.0f;
        for (size_t i = 0; i < nexts.size(); ++i) {
            acc += weights[i];
            if (u <= acc) { pick = nexts[i]; break; }
        }
        traj.push_back(pick);
        cur = pick;
    }
    return traj;
}

float GNGRollout::compute_entropy(std::vector<std::vector<int>> const& trajectories) const {
    if (trajectories.empty()) return 0.0f;
    std::unordered_map<int, int> term_counts;
    for (auto const& t : trajectories)
        if (!t.empty()) ++term_counts[t.back()];
    if (term_counts.empty()) return 0.0f;
    float K = float(trajectories.size());
    float H = 0.0f;
    for (auto const& [_, c] : term_counts) {
        float p = float(c) / K;
        if (p > 0.0f) H -= p * std::log(p);
    }
    return H;
}

void GNGRollout::handle_query(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto q = std::dynamic_pointer_cast<const RolloutQuery>(payload);
    if (!q) return;

    auto resp = std::make_shared<RolloutResult>();
    resp->tick_id     = q->tick_id;
    resp->producer_id = id_.empty() ? std::string("rollout") : id_;
    resp->request_id  = q->request_id;

    // Per-tick concurrency cap.
    if (queries_this_tick_ >= max_concurrent_queries_) {
        bus_->publish(topics::kRolloutResult, resp);
        return;
    }
    ++queries_this_tick_;

    // Look up cached source.  Source key is the full topic name; the
    // contract uses the short modality form ("video.retinal"), so we
    // accept both.
    PerSource const* src = nullptr;
    auto it_full  = per_source_.find(std::string("reality.") + q->source_modality);
    auto it_short = per_source_.find(q->source_modality);
    if (it_full  != per_source_.end()) src = &it_full->second;
    if (!src && it_short != per_source_.end()) src = &it_short->second;
    if (!src) {
        bus_->publish(topics::kRolloutResult, resp);
        return;
    }

    int K = std::min(K_default_, std::max(1, q->K_samples));
    int M = std::min(M_default_, std::max(1, q->M_steps));

    if (q->winner_id < 0 || src->trans.find(q->winner_id) == src->trans.end()) {
        bus_->publish(topics::kRolloutResult, resp);
        return;
    }

    resp->trajectories.reserve(K);
    resp->terminal_values.reserve(K);
    for (int k = 0; k < K; ++k) {
        auto traj = sample_trajectory(*src, q->winner_id, M, rng_);
        // Heuristic terminal value: drive-error urgency penalises
        // far-from-setpoint terminals.  Negative values mean "this
        // trajectory leaves the body further from setpoint".
        float val = -latest_drive_urgency_;
        resp->terminal_values.push_back(val);
        resp->trajectories.push_back(std::move(traj));
    }
    resp->entropy = compute_entropy(resp->trajectories);

    bus_->publish(topics::kRolloutResult, resp);
}

void GNGRollout::tick(uint64_t /*tick_id*/) {
    queries_this_tick_ = 0;
}

size_t GNGRollout::known_nodes(std::string const& source) const {
    auto it = per_source_.find(source);
    if (it == per_source_.end()) return 0;
    return it->second.trans.size();
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------

nlohmann::json GNGRollout::snapshot_state() const {
    nlohmann::json sources = nlohmann::json::object();
    for (auto const& [name, src] : per_source_) {
        nlohmann::json trans = nlohmann::json::array();
        for (auto const& [from, to_map] : src.trans) {
            for (auto const& [to, count] : to_map) {
                trans.push_back({from, to, count});
            }
        }
        sources[name] = nlohmann::json{
            {"last_winner", src.last_winner},
            {"trans",       trans},
        };
    }
    std::ostringstream oss; oss << rng_;
    return nlohmann::json{
        {"version",              1},
        {"per_source",           sources},
        {"latest_drive_urgency", latest_drive_urgency_},
        {"queries_this_tick",    queries_this_tick_},
        {"rng",                  oss.str()},
    };
}

void GNGRollout::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("GNGRollout::restore_state: unknown version " +
                                 std::to_string(version));
    }
    per_source_.clear();
    if (s.contains("per_source") && s["per_source"].is_object()) {
        for (auto it = s["per_source"].begin(); it != s["per_source"].end(); ++it) {
            PerSource src;
            src.last_winner = it.value().value("last_winner", -1);
            if (it.value().contains("trans") && it.value()["trans"].is_array()) {
                for (auto const& triple : it.value()["trans"]) {
                    int from  = triple[0].get<int>();
                    int to    = triple[1].get<int>();
                    int count = triple[2].get<int>();
                    src.trans[from][to] = count;
                }
            }
            per_source_[it.key()] = std::move(src);
        }
    }
    latest_drive_urgency_ = s.value("latest_drive_urgency", latest_drive_urgency_);
    queries_this_tick_    = s.value("queries_this_tick",    queries_this_tick_);
    std::string rng_s = s.value("rng", std::string{});
    if (!rng_s.empty()) {
        std::istringstream iss(rng_s);
        iss >> rng_;
    }
}

} // namespace ogma
