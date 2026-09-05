#include "ogma/modules/SequenceGNG.hpp"

#include <algorithm>
#include <cmath>
#include <random>
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
    throw std::invalid_argument("SequenceGNG param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("SequenceGNG param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("SequenceGNG param '" + key + "' must be bool");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("SequenceGNG param '" + key + "' must be string");
}

SequenceGNG::SourceKind parse_kind(std::string const& s) {
    if (s == "winner") return SequenceGNG::SourceKind::Winner;
    if (s == "action") return SequenceGNG::SourceKind::Action;
    if (s == "intent") return SequenceGNG::SourceKind::Intent;   // v5.3 Phase B
    throw std::invalid_argument("SequenceGNG: unknown source_kind '" + s + "'");
}

std::string default_output_topic(std::string const& source) {
    return std::string("sequence.motif.") + source;
}

} // namespace

SequenceGNG::SequenceGNG()  = default;
SequenceGNG::~SequenceGNG() = default;

std::string_view SequenceGNG::type_name() const { return "SequenceGNG"; }

std::vector<TopicSpec> SequenceGNG::input_topics() const {
    std::vector<TopicSpec> out;
    if (source_kind_ == SourceKind::Action) {
        out.push_back(TopicSpec{source_topic_, std::type_index(typeid(ActionOut)),
                                  SubscriptionKind::Direct, /*required=*/true});
    } else if (source_kind_ == SourceKind::Intent) {
        // v5.3 Phase B — Intent source.  Primary producer is Premotor
        // publishing PolicyToken to policy.intent (with chosen_intent field).
        // Also accepts IntentToken (e.g., if a future module publishes
        // intent indices directly).  We declare PolicyToken here; the
        // handler dynamic_casts to whichever arrives.
        out.push_back(TopicSpec{source_topic_, std::type_index(typeid(PolicyToken)),
                                  SubscriptionKind::Direct, /*required=*/true});
    } else {
        // Winner source: RealityToken or ConsensusToken.  Declare RealityToken
        // here; the handler dynamic_casts to whichever arrives.
        out.push_back(TopicSpec{source_topic_, std::type_index(typeid(RealityToken)),
                                  SubscriptionKind::Direct, /*required=*/true});
    }
    out.push_back(TopicSpec{topics::kNeuroState,
                              std::type_index(typeid(NeuroState)),
                              SubscriptionKind::Direct, /*required=*/false});
    return out;
}

std::vector<TopicSpec> SequenceGNG::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(SequenceMotif))} };
}

ParamSchema SequenceGNG::params_schema() const {
    return {
        {"source_topic",                ParamMutability::ConstructionOnly, "Subscribed input stream", std::nullopt},
        {"source_kind",                 ParamMutability::ConstructionOnly, "winner | action", ParamValue{std::string("winner")}},
        {"window_size",                 ParamMutability::ConstructionOnly, "n-gram length", ParamValue{int64_t{5}}},
        {"projection_dim",              ParamMutability::ConstructionOnly, "GNG input dim", ParamValue{int64_t{128}}},
        {"prototype_per_winner_dim",    ParamMutability::ConstructionOnly, "Per-winner sub-encoding size", ParamValue{int64_t{32}}},
        {"motif_branching_threshold",   ParamMutability::HotMutable,       "Successor-entropy threshold for mitosis", ParamValue{0.4}},
        {"baking_threshold",            ParamMutability::HotMutable,       "GNG baking visit count", ParamValue{int64_t{50}}},
        {"health_death_spares_baked",   ParamMutability::HotMutable,
         "Exempt BAKED nodes from the GNG health-death sweep — see the EPM's param of the "
         "same name for the mechanism and measurements (the operator's prune-then-relearn "
         "cascade). false = legacy, byte-identical.", ParamValue{false}},
        {"min_insertion_error",         ParamMutability::HotMutable,       "GNG min_insertion_error", ParamValue{0.02}},
        {"epsilon_b",                   ParamMutability::HotMutable,       "GNG winner LR", ParamValue{0.05}},
        {"epsilon_n",                   ParamMutability::HotMutable,       "GNG neighbour LR", ParamValue{0.003}},
        {"max_age",                     ParamMutability::HotMutable,       "GNG edge max_age", ParamValue{int64_t{88}}},
        {"max_nodes",                   ParamMutability::HotMutable,       "GNG max_nodes", ParamValue{int64_t{2000}}},
        {"mitosis_enabled",             ParamMutability::HotMutable,       "GNG mitosis on/off", ParamValue{true}},
        {"mitosis_error_threshold",     ParamMutability::HotMutable,       "GNG mitosis trigger", ParamValue{0.30}},
        {"output_topic",                ParamMutability::ConstructionOnly, "Override output topic (default sequence.motif.<source>)", ParamValue{std::string("")}},
        {"context_topic",               ParamMutability::ConstructionOnly, "Optional SequenceMotif topic; gates GNG updates on context-motif stability over window_size ticks", ParamValue{std::string("")}},
        {"event_mode",                  ParamMutability::ConstructionOnly, "Winner source only: push the n-gram window on winner CHANGE (dwell-collapsed events) and step the GNG only when the window changed. Default false = byte-identical legacy per-tick windows.", ParamValue{false}},
        {"master_seed",                 ParamMutability::ConstructionOnly, "RNG namespace seed", ParamValue{int64_t{0}}},
    };
}

void SequenceGNG::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("SequenceGNG requires a non-null Bus");

    auto find_required = [&](std::string const& key) -> ParamValue const& {
        auto it = params.find(key);
        if (it == params.end())
            throw std::invalid_argument("SequenceGNG: required param '" + key + "' missing");
        return it->second;
    };

    source_topic_ = get_string(find_required("source_topic"), "source_topic");
    apply_param(params, "source_kind",                 [&](auto const& v){ source_kind_              = parse_kind(get_string(v, "source_kind")); });
    apply_param(params, "window_size",                 [&](auto const& v){ window_size_              = std::max(2, int(get_int(v, "window_size"))); });
    apply_param(params, "projection_dim",              [&](auto const& v){ projection_dim_           = int(get_int(v, "projection_dim")); });
    apply_param(params, "prototype_per_winner_dim",    [&](auto const& v){ prototype_per_winner_dim_ = std::max(1, int(get_int(v, "prototype_per_winner_dim"))); });
    apply_param(params, "motif_branching_threshold",   [&](auto const& v){ motif_branching_threshold_ = float(get_double(v, "motif_branching_threshold")); });
    apply_param(params, "master_seed",                 [&](auto const& v){ master_seed_              = uint64_t(get_int(v, "master_seed")); });

    apply_param(params, "output_topic", [&](auto const& v){
        auto s = get_string(v, "output_topic");
        if (!s.empty()) output_topic_ = s;
    });
    if (output_topic_.empty()) {
        // Default: sequence.motif.<source-without-prefix>.
        output_topic_ = default_output_topic(source_topic_);
    }
    apply_param(params, "context_topic", [&](auto const& v){
        context_topic_ = get_string(v, "context_topic");
    });
    apply_param(params, "event_mode", [&](auto const& v){
        event_mode_ = get_bool(v, "event_mode");
    });

    // GNG configuration.
    ami_ogma::v3::GNG::Config gng_cfg;
    gng_cfg.dim                 = projection_dim_;
    apply_param(params, "baking_threshold",        [&](auto const& v){ gng_cfg.baking_threshold        = int(get_int(v, "baking_threshold")); });
    apply_param(params, "health_death_spares_baked", [&](auto const& v){ gng_cfg.health_death_spares_baked = get_bool(v, "health_death_spares_baked"); });
    apply_param(params, "min_insertion_error",     [&](auto const& v){ gng_cfg.min_insertion_error     = float(get_double(v, "min_insertion_error")); });
    apply_param(params, "epsilon_b",               [&](auto const& v){ gng_cfg.epsilon_b               = float(get_double(v, "epsilon_b")); });
    apply_param(params, "epsilon_n",               [&](auto const& v){ gng_cfg.epsilon_n               = float(get_double(v, "epsilon_n")); });
    apply_param(params, "max_age",                 [&](auto const& v){ gng_cfg.max_age                 = int(get_int(v, "max_age")); });
    apply_param(params, "max_nodes",               [&](auto const& v){ gng_cfg.max_nodes               = int(get_int(v, "max_nodes")); });
    apply_param(params, "mitosis_enabled",         [&](auto const& v){ gng_cfg.mitosis_enabled         = get_bool(v, "mitosis_enabled"); });
    apply_param(params, "mitosis_error_threshold", [&](auto const& v){ gng_cfg.mitosis_error_threshold = float(get_double(v, "mitosis_error_threshold")); });
    gng_ = std::make_unique<ami_ogma::v3::GNG>(gng_cfg);

    // Build the windowed JL projection.  Input dim depends on source_kind.
    int input_dim;
    if (source_kind_ == SourceKind::Action) input_dim = window_size_;
    else                                    input_dim = window_size_ * prototype_per_winner_dim_;
    windowed_jl_ = ami_ogma::v3::FrozenJLEncoder::make_state_encoder(
        std::string("seqgng_") + (id_.empty() ? std::string("default") : id_),
        projection_dim_,
        input_dim);

    // Subscribe.
    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(source_topic_, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_source(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kNeuroState, SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_neuro(t, p); }));
    if (!context_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(context_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*t*/, MessagePtr p){
                if (auto m = std::dynamic_pointer_cast<const SequenceMotif>(p)) {
                    last_context_motif_id_ = m->motif_id;
                }
            }));
    }
}

void SequenceGNG::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "motif_branching_threshold") motif_branching_threshold_ = float(get_double(value, k));
    else if (k == "baking_threshold")          gng_->set_baking_threshold(int(get_int(value, k)));
    else if (k == "health_death_spares_baked") gng_->set_health_death_spares_baked(get_bool(value, k));
    else if (k == "min_insertion_error")       gng_->set_min_insertion_error(float(get_double(value, k)));
    else if (k == "epsilon_b")                 gng_->set_epsilon_b(float(get_double(value, k)));
    else if (k == "epsilon_n")                 gng_->set_epsilon_n(float(get_double(value, k)));
    else if (k == "max_age")                   gng_->set_max_age(int(get_int(value, k)));
    else if (k == "mitosis_enabled")           gng_->set_mitosis_enabled(get_bool(value, k));
    else if (k == "mitosis_error_threshold")   gng_->set_mitosis_error_threshold(float(get_double(value, k)));
    else if (k == "source_topic" || k == "source_kind" || k == "window_size"
          || k == "projection_dim" || k == "prototype_per_winner_dim"
          || k == "output_topic" || k == "master_seed" || k == "max_nodes")
        throw std::invalid_argument("SequenceGNG param '" + k + "' is ConstructionOnly");
    else
        throw std::invalid_argument("SequenceGNG: unknown param '" + k + "'");
}

void SequenceGNG::handle_source(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    if (source_kind_ == SourceKind::Action) {
        auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
        if (!a) return;
        action_window_.push_back(a->accel);
        while (int(action_window_.size()) > window_size_) action_window_.pop_front();
        return;
    }

    // v5.3 Phase B — Intent source: each tick brings one IntentToken.index;
    // we route it through the same per-winner hashing path as the Winner
    // kind by buffering into winner_window_, so all the existing motif /
    // successor / mitosis logic applies unchanged.
    if (source_kind_ == SourceKind::Intent) {
        // Accept either PolicyToken (chosen_intent field) or IntentToken
        // (index field).  Premotor publishes PolicyToken to policy.intent
        // every tick — that's the primary source.
        int wid = -1;
        if (auto pt = std::dynamic_pointer_cast<const PolicyToken>(payload)) {
            wid = pt->chosen_intent;
        } else if (auto it = std::dynamic_pointer_cast<const IntentToken>(payload)) {
            wid = it->index;
        }
        if (wid < 0) return;
        if (last_winner_id_ >= 0)
            track_successor(current_motif_id_, wid);
        last_winner_id_ = wid;
        winner_window_.push_back(wid);
        while (int(winner_window_.size()) > window_size_) winner_window_.pop_front();
        return;
    }

    // Winner source: prefer RealityToken; fall back to ConsensusToken.
    int wid = -1;
    if (auto rt = std::dynamic_pointer_cast<const RealityToken>(payload))
        wid = rt->winner_id;
    else if (auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload))
        wid = ct->active_winner_id;
    if (wid < 0) return;     // skip bootstrap placeholder tokens
    if (event_mode_ && wid == last_winner_id_) return;   // dwell: not an event

    if (last_winner_id_ >= 0)
        track_successor(current_motif_id_, wid);         // event-successors in event mode
    last_winner_id_ = wid;

    winner_window_.push_back(wid);
    while (int(winner_window_.size()) > window_size_) winner_window_.pop_front();
    window_dirty_ = true;
    ++n_events_;
}

void SequenceGNG::handle_neuro(std::string_view /*topic*/, MessagePtr /*payload*/) {
    // Neuro modulation is a Phase 3 stretch; placeholder for symmetry with EPM.
}

Eigen::VectorXf SequenceGNG::hash_winner(int winner_id) const {
    // Deterministic int → vector-of-float mapping, identical across runs.
    Eigen::VectorXf v(prototype_per_winner_dim_);
    std::mt19937_64 rng(uint64_t(winner_id) * 0x9E3779B97F4A7C15ull
                        + master_seed_      * 0xBF58476D1CE4E5B9ull);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (int i = 0; i < prototype_per_winner_dim_; ++i) v[i] = dist(rng);
    return v;
}

bool SequenceGNG::encode_window(Eigen::VectorXf& out) const {
    if (source_kind_ == SourceKind::Action) {
        if (int(action_window_.size()) < window_size_) return false;
        Eigen::VectorXf concat(window_size_);
        for (int i = 0; i < window_size_; ++i) concat[i] = action_window_[i];
        out = windowed_jl_->encode_state(concat.data(), int(concat.size()));
        return out.size() == projection_dim_;
    }

    if (int(winner_window_.size()) < window_size_) return false;
    Eigen::VectorXf concat(window_size_ * prototype_per_winner_dim_);
    for (int i = 0; i < window_size_; ++i) {
        Eigen::VectorXf hashed = hash_winner(winner_window_[i]);
        concat.segment(i * prototype_per_winner_dim_, prototype_per_winner_dim_) = hashed;
    }
    out = windowed_jl_->encode_state(concat.data(), int(concat.size()));
    return out.size() == projection_dim_;
}

void SequenceGNG::track_successor(int active_motif, int next_winner) {
    if (active_motif < 0) return;
    ++successor_counts_[active_motif][next_winner];
}

void SequenceGNG::tick(uint64_t tick_id) {
    auto out = std::make_shared<SequenceMotif>();
    out->tick_id      = tick_id;
    out->producer_id  = id_.empty() ? std::string("seqgng") : id_;
    out->motif_id     = -1;
    out->phase        = 0;
    out->motif_length = window_size_;

    // Phase 6.5.3.2 — context-stability gate.  If a context_topic was
    // configured, check that the context's motif_id has been the same
    // for the past `window_size_` ticks before allowing GNG updates.
    // Tracked via context_history_ (deque of last N context motif ids).
    // Outside the gate we publish an empty SequenceMotif (motif_id=-1)
    // and skip both the GNG update and successor tracking — downstream
    // (MotorRepertoire) treats this as "no motif this tick," which is
    // the correct semantics during context transitions where action-
    // pattern recurrence isn't a functional regularity.
    bool gate_open = true;
    if (!context_topic_.empty()) {
        context_history_.push_back(last_context_motif_id_);
        while (int(context_history_.size()) > window_size_)
            context_history_.pop_front();
        gate_open = (int(context_history_.size()) == window_size_)
                 && std::all_of(context_history_.begin(), context_history_.end(),
                                [first = context_history_.front()](int v){ return v == first; })
                 && (last_context_motif_id_ >= 0);
    }
    if (!gate_open) {
        bus_->publish(output_topic_, out);
        return;
    }

    // Event mode: during dwell the window is unchanged — re-stepping the GNG
    // on the identical encoding would inflate visit counts with dwell time.
    // Publish the standing motif state and return.
    if (event_mode_ && !window_dirty_) {
        if (current_motif_id_ >= 0) {
            out->motif_id         = current_motif_id_;
            out->phase            = motif_phase_;
            out->match_confidence = match_confidence_;
            out->is_baked         = gng_ && gng_->is_crystallised(current_motif_id_);
            auto it = successor_counts_.find(current_motif_id_);
            if (it != successor_counts_.end() && !it->second.empty()) {
                int best_id = -1, best_count = -1;
                for (auto const& [next, count] : it->second)
                    if (count > best_count) { best_count = count; best_id = next; }
                out->predicted_next_id = best_id;
            }
        }
        bus_->publish(output_topic_, out);
        return;
    }

    Eigen::VectorXf encoded;
    if (encode_window(encoded)) {
        auto [winner_id, qe] = gng_->step(encoded);
        window_dirty_ = false;
        just_baked_     = gng_->last_step_baked();
        if (winner_id >= 0 && gng_->node_count() >= 2) {
            current_motif_id_ = winner_id;
            // Phase: how far along the window we are within the motif's
            // "cycle"; with hash-based encoding each winner advance moves
            // phase by 1.  Approximate via tick_id modulo motif_length.
            motif_phase_      = int(tick_id % uint64_t(window_size_));
            // Match confidence: 1 - normalised quant_error.
            float thresh = std::max(1e-6f, gng_->config().min_insertion_error * 5.0f);
            match_confidence_ = std::max(0.0f, std::min(1.0f, 1.0f - qe / thresh));

            out->motif_id          = winner_id;
            out->phase             = motif_phase_;
            out->match_confidence  = match_confidence_;
            out->just_baked        = just_baked_;
            out->is_baked          = gng_->is_crystallised(winner_id);

            // Predicted next: argmax of successor counts for this motif.
            auto it = successor_counts_.find(winner_id);
            if (it != successor_counts_.end() && !it->second.empty()) {
                int    best_id    = -1;
                int    best_count = -1;
                for (auto const& [next, count] : it->second) {
                    if (count > best_count) { best_count = count; best_id = next; }
                }
                out->predicted_next_id = best_id;
            }
        }
    }
    bus_->publish(output_topic_, out);
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------

// The high-rate payload (xaq_voice).  Which motif the sequence layer thinks it is in,
// how far through it, and how well it matches — the scalars only, never the GNG.  Without
// this the module publishes `{}` while still matching xaq_voice's type filter, which
// opens a permanently silent oscillator with no diagnostic.
nlohmann::json SequenceGNG::diag_lite() const {
    return {
        {"current_motif_id", current_motif_id_},
        {"motif_phase",      motif_phase_},
        {"match_confidence", match_confidence_},
        {"just_baked",       just_baked_},
        {"n_events",         n_events_},
    };
}

nlohmann::json SequenceGNG::diag_snapshot() const {
    // Lightweight payload tailored to tools/xaq_inspector/widgets/
    // seqgng_inspector.py.  The full snapshot_state() ships every
    // GNG node's prototype (cfg.dim floats — typically 128), all
    // edges (O(N^2) worst-case), and the full successor_counts map.
    // After hours of training that payload exceeds 1 MB per tick;
    // multiplied by the 30 Hz diag rate it saturates the ZMQ pub
    // socket and spikes both Godot CPU and the inspector CPU.
    //
    // The inspector widgets only consume:
    //   gng.nodes[].visits       (visit-count bars)
    //   gng.baking_threshold     (baked/raw colouring)
    //   winner_window[-1]        (winner sparkline; widget only reads
    //                             the LAST element each tick)
    //   action_window[-1]        (action variant of the same)
    //   current_motif_id
    //   match_confidence
    //   motif_phase
    //   successor_counts (capped) (transition matrix; widget already
    //                             trims to MAX_DIM=64 motifs by mass)
    //
    // No prototypes / edges / history / context shipped over diag.
    // Shape stays compatible with the existing widget so no Python
    // change is required.
    nlohmann::json nodes = nlohmann::json::array();
    int baking_threshold = 100;
    if (gng_) {
        baking_threshold = gng_->config().baking_threshold;
        for (int v : gng_->visit_counts()) {
            nodes.push_back({{"visits", v}});
        }
    }
    nlohmann::json gng_json = {
        {"nodes",            std::move(nodes)},
        {"baking_threshold", baking_threshold},
    };

    // Successor counts: cap at top-K entries by count.  The transition-
    // matrix widget already trims to its own MAX_DIM (64) by mass, but
    // we don't want to ship a huge list to it either.  K = 2048 lets a
    // 45-motif dense matrix through fully (45^2 = 2025) while bounding
    // long-tail bloat.
    constexpr size_t kSuccessorCap = 2048;
    std::vector<std::tuple<int,int,int>> triples;
    triples.reserve(kSuccessorCap);
    for (auto const& [from, to_map] : successor_counts_) {
        for (auto const& [to, count] : to_map) {
            triples.emplace_back(from, to, count);
        }
    }
    if (triples.size() > kSuccessorCap) {
        // Partial sort: keep top K by count, drop the rest.  Order
        // within kept K doesn't matter — widget aggregates by ranking.
        std::nth_element(
            triples.begin(), triples.begin() + kSuccessorCap, triples.end(),
            [](auto const& a, auto const& b){
                return std::get<2>(a) > std::get<2>(b);
            });
        triples.resize(kSuccessorCap);
    }
    nlohmann::json succ = nlohmann::json::array();
    for (auto const& [from, to, count] : triples) {
        succ.push_back({from, to, count});
    }

    // Winner/action window: only the LAST element is read by the widget
    // (see _WinnerWindow.update_payload — it does buf[-1] then rolls).
    nlohmann::json wwin = nlohmann::json::array();
    if (!winner_window_.empty()) wwin.push_back(winner_window_.back());
    nlohmann::json awin = nlohmann::json::array();
    if (!action_window_.empty()) awin.push_back(action_window_.back());

    return nlohmann::json{
        {"version",                1},
        {"gng",                    std::move(gng_json)},
        {"winner_window",          std::move(wwin)},
        {"action_window",          std::move(awin)},
        {"current_motif_id",       current_motif_id_},
        {"motif_phase",            motif_phase_},
        {"match_confidence",       match_confidence_},
        {"just_baked",             just_baked_},
        {"event_mode",             event_mode_},
        {"n_events",               n_events_},
        {"successor_counts",       std::move(succ)},
    };
}

nlohmann::json SequenceGNG::snapshot_state() const {
    nlohmann::json wwin = nlohmann::json::array();
    for (auto v : winner_window_) wwin.push_back(v);
    nlohmann::json awin = nlohmann::json::array();
    for (auto v : action_window_) awin.push_back(v);
    nlohmann::json chist = nlohmann::json::array();
    for (auto v : context_history_) chist.push_back(v);
    nlohmann::json succ = nlohmann::json::array();
    for (auto const& [from, to_map] : successor_counts_) {
        for (auto const& [to, count] : to_map) {
            succ.push_back({from, to, count});
        }
    }
    nlohmann::json gng_json = nullptr;
    if (gng_) gng_json = gng_->to_json();
    return nlohmann::json{
        {"version",                1},
        {"gng",                    gng_json},
        {"winner_window",          wwin},
        {"action_window",          awin},
        {"last_winner_id",         last_winner_id_},
        {"current_motif_id",       current_motif_id_},
        {"motif_phase",            motif_phase_},
        {"motif_length",           motif_length_},
        {"match_confidence",       match_confidence_},
        {"just_baked",             just_baked_},
        {"context_history",        chist},
        {"last_context_motif_id",  last_context_motif_id_},
        {"successor_counts",       succ},
    };
}

void SequenceGNG::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("SequenceGNG::restore_state: unknown version " +
                                 std::to_string(version));
    }
    if (gng_ && s.contains("gng") && !s["gng"].is_null()) {
        *gng_ = ami_ogma::v3::GNG::from_json(s["gng"]);
    }
    winner_window_.clear();
    if (s.contains("winner_window") && s["winner_window"].is_array())
        for (auto const& v : s["winner_window"]) winner_window_.push_back(v.get<int>());
    action_window_.clear();
    if (s.contains("action_window") && s["action_window"].is_array())
        for (auto const& v : s["action_window"]) action_window_.push_back(v.get<float>());
    last_winner_id_        = s.value("last_winner_id",        last_winner_id_);
    current_motif_id_      = s.value("current_motif_id",      current_motif_id_);
    motif_phase_           = s.value("motif_phase",           motif_phase_);
    motif_length_          = s.value("motif_length",          motif_length_);
    match_confidence_      = s.value("match_confidence",      match_confidence_);
    just_baked_            = s.value("just_baked",            just_baked_);
    context_history_.clear();
    if (s.contains("context_history") && s["context_history"].is_array())
        for (auto const& v : s["context_history"]) context_history_.push_back(v.get<int>());
    last_context_motif_id_ = s.value("last_context_motif_id", last_context_motif_id_);
    successor_counts_.clear();
    if (s.contains("successor_counts") && s["successor_counts"].is_array()) {
        for (auto const& triple : s["successor_counts"]) {
            int from  = triple[0].get<int>();
            int to    = triple[1].get<int>();
            int count = triple[2].get<int>();
            successor_counts_[from][to] = count;
        }
    }
}

} // namespace ogma
