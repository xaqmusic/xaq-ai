#include "ogma/modules/NeurochemState.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename T>
T clamp01(T v) { return std::clamp(v, T{0}, T{1}); }

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("NeurochemState param '" + key + "' must be numeric");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("NeurochemState param '" + key + "' must be bool");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("NeurochemState param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("NeurochemState param '" + key + "' must be string");
}

} // namespace

NeurochemState::NeurochemState()  = default;
NeurochemState::~NeurochemState() = default;

std::string_view NeurochemState::type_name() const { return "NeurochemState"; }

std::vector<TopicSpec> NeurochemState::input_topics() const {
    return {
        // Per-EPM TLE via Feedback (cycle-break against EPM publishing
        // reality.<modality>).  Also catches reality.proprio.<sensor> deliveries
        // because they share the prefix; the handler filters by payload type.
        TopicSpec{"reality.",        std::type_index(typeid(RealityToken)),  SubscriptionKind::Feedback, /*required=*/true},
        TopicSpec{topics::kConsensusPrefix, std::type_index(typeid(ConsensusToken)), SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kEventsPrefix,    std::type_index(typeid(EnvEvent)),       SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{topics::kRealityProprioPrefix, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> NeurochemState::output_topics() const {
    return { TopicSpec{topics::kNeuroState, std::type_index(typeid(NeuroState))} };
}

ParamSchema NeurochemState::params_schema() const {
    return {
        {"da_baseline",          ParamMutability::HotMutable,        "Resting dopamine level",      ParamValue{0.20}, ParamValue{0.0}, ParamValue{1.0}},
        {"da_baseline_ema_alpha",ParamMutability::HotMutable,        "EMA rate for adaptive dopamine baseline (0=disabled; ~0.001 typical for slow adaptation)", ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"ht_baseline",          ParamMutability::HotMutable,        "Resting serotonin level",     ParamValue{0.65}, ParamValue{0.0}, ParamValue{1.0}},
        {"da_decay",             ParamMutability::HotMutable,        "Per-tick dopamine decay",     ParamValue{0.88}, ParamValue{0.0}, ParamValue{1.0}},
        {"ht_decay",             ParamMutability::HotMutable,        "Per-tick serotonin decay",    ParamValue{0.93}, ParamValue{0.0}, ParamValue{1.0}},
        {"intrinsic_da_gain",    ParamMutability::HotMutable,        "Dopamine per unit TLE drop",  ParamValue{0.05}},
        {"scent_da_rate",        ParamMutability::HotMutable,        "Dopamine per unit scent delta", ParamValue{0.25}},
        {"green_da_rate",        ParamMutability::HotMutable,        "Dopamine per unit positive Δgreen_fraction — anticipatory 'hope' as food (green) LOOMS larger in view. Potential-based (rising-only) → Goodhart-safe. 0=off.", ParamValue{0.0}},
        {"travel_da_rate",       ParamMutability::HotMutable,        "Dopamine per unit travel",    ParamValue{0.02}},
        {"whisker_ht_rate",      ParamMutability::HotMutable,        "Serotonin drain per unit whisker", ParamValue{0.02}},
        {"hunger_ht_rate",       ParamMutability::HotMutable,        "Serotonin drain per unit hunger",  ParamValue{0.01}},
        {"pheromone_ht_rate",    ParamMutability::HotMutable,        "Serotonin drain per unit pheromone", ParamValue{0.005}},
        {"pheromone_threshold",  ParamMutability::HotMutable,        "Pheromone drain threshold",   ParamValue{0.30}},
        {"wall_stuck_da_drain",  ParamMutability::HotMutable,        "Dopamine drain on wall_stuck", ParamValue{0.35}},
        {"wall_stuck_ht_drain",  ParamMutability::HotMutable,        "Serotonin drain on wall_stuck", ParamValue{0.15}},
        {"event_coupled_da",     ParamMutability::HotMutable,        "If true, hits/misses/bricks pulse dopamine", ParamValue{false}},
        {"event_coupled_ht",     ParamMutability::HotMutable,        "If true, misses pulse serotonin", ParamValue{false}},
        {"da_hit_gain",          ParamMutability::HotMutable,        "Dopamine pulse on hit (gated)", ParamValue{0.45}},
        {"da_brick_gain",        ParamMutability::HotMutable,        "Dopamine pulse on brick (gated)", ParamValue{0.65}},
        {"da_aligned_gain",      ParamMutability::HotMutable,        "v5.3 Phase C — DA pulse when an EventConjunction fires events.<aligned_event_name>.  0=scaffold off (default).  Larger than da_hit_gain because this is an explicit teaching signal.", ParamValue{0.0}},
        {"aligned_event_name",   ParamMutability::ConstructionOnly,  "v5.3 Phase C — name of the conjunction event whose firing triggers da_aligned_gain (default scent_aligned_with_green).", ParamValue{std::string("scent_aligned_with_green")}},
        {"da_miss_drop",         ParamMutability::HotMutable,        "Dopamine drop on miss (gated)", ParamValue{0.25}},
        {"ht_miss_drop",         ParamMutability::HotMutable,        "Serotonin drop on miss (gated)", ParamValue{0.30}},
        {"master_seed",          ParamMutability::ConstructionOnly,  "RNG namespace seed (forward compat)", ParamValue{int64_t{0}}},
        {"input_exclude",        ParamMutability::ConstructionOnly,  "Phase 7.2-EPM: trailing-dot reality.* prefixes whose RealityTokens are dropped from TLE aggregation (e.g. [\"reality.joint_fl.\", \"reality.leg.\"]).  Accepts a single string or string array.  Empty = legacy behaviour (no exclusion).", ParamValue{std::string("")}},
    };
}

void NeurochemState::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("NeurochemState requires a non-null Bus");

    // Apply provided params; missing keys keep defaults.
    apply_param(params, "da_baseline",         [&](auto const& v){ da_baseline_       = get_double(v, "da_baseline"); });
    apply_param(params, "da_baseline_ema_alpha", [&](auto const& v){ da_baseline_ema_alpha_ = get_double(v, "da_baseline_ema_alpha"); });
    apply_param(params, "ht_baseline",         [&](auto const& v){ ht_baseline_       = get_double(v, "ht_baseline"); });
    apply_param(params, "da_decay",            [&](auto const& v){ da_decay_          = get_double(v, "da_decay"); });
    apply_param(params, "ht_decay",            [&](auto const& v){ ht_decay_          = get_double(v, "ht_decay"); });
    apply_param(params, "intrinsic_da_gain",   [&](auto const& v){ intrinsic_da_gain_ = get_double(v, "intrinsic_da_gain"); });
    apply_param(params, "scent_da_rate",       [&](auto const& v){ scent_da_rate_     = get_double(v, "scent_da_rate"); });
    apply_param(params, "green_da_rate",       [&](auto const& v){ green_da_rate_     = get_double(v, "green_da_rate"); });
    apply_param(params, "travel_da_rate",      [&](auto const& v){ travel_da_rate_    = get_double(v, "travel_da_rate"); });
    apply_param(params, "whisker_ht_rate",     [&](auto const& v){ whisker_ht_rate_   = get_double(v, "whisker_ht_rate"); });
    apply_param(params, "hunger_ht_rate",      [&](auto const& v){ hunger_ht_rate_    = get_double(v, "hunger_ht_rate"); });
    apply_param(params, "pheromone_ht_rate",   [&](auto const& v){ pheromone_ht_rate_ = get_double(v, "pheromone_ht_rate"); });
    apply_param(params, "pheromone_threshold", [&](auto const& v){ pheromone_threshold_ = get_double(v, "pheromone_threshold"); });
    apply_param(params, "wall_stuck_da_drain", [&](auto const& v){ wall_stuck_da_drain_ = get_double(v, "wall_stuck_da_drain"); });
    apply_param(params, "wall_stuck_ht_drain", [&](auto const& v){ wall_stuck_ht_drain_ = get_double(v, "wall_stuck_ht_drain"); });
    apply_param(params, "event_coupled_da",    [&](auto const& v){ event_coupled_da_  = get_bool(v, "event_coupled_da"); });
    apply_param(params, "event_coupled_ht",    [&](auto const& v){ event_coupled_ht_  = get_bool(v, "event_coupled_ht"); });
    apply_param(params, "da_hit_gain",         [&](auto const& v){ da_hit_gain_       = get_double(v, "da_hit_gain"); });
    apply_param(params, "da_aligned_gain",     [&](auto const& v){ da_aligned_gain_   = get_double(v, "da_aligned_gain"); });
    apply_param(params, "aligned_event_name",  [&](auto const& v){ aligned_event_name_= get_string(v, "aligned_event_name"); });
    apply_param(params, "da_brick_gain",       [&](auto const& v){ da_brick_gain_     = get_double(v, "da_brick_gain"); });
    apply_param(params, "da_miss_drop",        [&](auto const& v){ da_miss_drop_      = get_double(v, "da_miss_drop"); });
    apply_param(params, "ht_miss_drop",        [&](auto const& v){ ht_miss_drop_      = get_double(v, "ht_miss_drop"); });
    apply_param(params, "master_seed",         [&](auto const& v){ master_seed_       = uint64_t(get_int(v, "master_seed")); });

    // Phase 7.2-EPM input_exclude — accepts string OR string array, like
    // LateralVoter (Phase 6.6.M).  Stored internally as a vector.
    auto it_excl = params.find("input_exclude");
    if (it_excl != params.end()) {
        input_exclude_.clear();
        if (auto v = std::get_if<std::vector<std::string>>(&it_excl->second)) {
            for (auto const& s : *v) if (!s.empty()) input_exclude_.push_back(s);
        } else if (auto s = std::get_if<std::string>(&it_excl->second)) {
            if (!s->empty()) input_exclude_.push_back(*s);
        }
    }

    // Initialize working state to baselines.
    dopamine_         = float(da_baseline_);
    da_baseline_ema_  = float(da_baseline_);
    serotonin_ = float(ht_baseline_);

    // Bind handlers.  Capture `this` — handlers run synchronously from Bus::publish
    // (Direct) or Bus::begin_tick (Feedback), always on the same thread.
    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe("reality.",        SubscriptionKind::Feedback,
        [this](auto t, auto p){ this->handle_reality(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kConsensusPrefix,        SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_consensus(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix,           SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_event(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kRealityProprioPrefix,   SubscriptionKind::Direct,
        [this](auto t, auto p){ this->handle_proprio(t, p); }));
}

void NeurochemState::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "da_baseline")         da_baseline_         = get_double(value, k);
    else if (k == "da_baseline_ema_alpha") da_baseline_ema_alpha_ = get_double(value, k);
    else if (k == "ht_baseline")         ht_baseline_         = get_double(value, k);
    else if (k == "da_decay")            da_decay_            = get_double(value, k);
    else if (k == "ht_decay")            ht_decay_            = get_double(value, k);
    else if (k == "intrinsic_da_gain")   intrinsic_da_gain_   = get_double(value, k);
    else if (k == "scent_da_rate")       scent_da_rate_       = get_double(value, k);
    else if (k == "green_da_rate")       green_da_rate_       = get_double(value, k);
    else if (k == "travel_da_rate")      travel_da_rate_      = get_double(value, k);
    else if (k == "whisker_ht_rate")     whisker_ht_rate_     = get_double(value, k);
    else if (k == "hunger_ht_rate")      hunger_ht_rate_      = get_double(value, k);
    else if (k == "pheromone_ht_rate")   pheromone_ht_rate_   = get_double(value, k);
    else if (k == "pheromone_threshold") pheromone_threshold_ = get_double(value, k);
    else if (k == "wall_stuck_da_drain") wall_stuck_da_drain_ = get_double(value, k);
    else if (k == "wall_stuck_ht_drain") wall_stuck_ht_drain_ = get_double(value, k);
    else if (k == "event_coupled_da")    event_coupled_da_    = get_bool(value, k);
    else if (k == "event_coupled_ht")    event_coupled_ht_    = get_bool(value, k);
    else if (k == "da_hit_gain")         da_hit_gain_         = get_double(value, k);
    else if (k == "da_aligned_gain")     da_aligned_gain_     = get_double(value, k);
    else if (k == "da_brick_gain")       da_brick_gain_       = get_double(value, k);
    else if (k == "da_miss_drop")        da_miss_drop_        = get_double(value, k);
    else if (k == "ht_miss_drop")        ht_miss_drop_        = get_double(value, k);
    else if (k == "master_seed")
        throw std::invalid_argument("NeurochemState.master_seed is ConstructionOnly");
    else if (k == "aligned_event_name" || k == "input_exclude")
        throw std::invalid_argument("NeurochemState." + k + " is ConstructionOnly");
    else
        throw std::invalid_argument("NeurochemState: unknown param '" + k + "'");
}

void NeurochemState::handle_reality(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt) return;   // proprio tokens land here via the prefix; ignore them
    // Phase 7.2-EPM: drop deliveries from excluded prefixes so derived
    // EPM pathways (e.g. per-joint sensorimotor EPMs feeding sub-voters)
    // don't contaminate global dopamine TLE aggregation.
    for (auto const& excl : input_exclude_) {
        if (topic.size() >= excl.size() &&
            topic.compare(0, excl.size(), excl) == 0) return;
    }
    pending_tle_sum_   += rt->tle;
    pending_tle_count_ += 1;
}

void NeurochemState::handle_consensus(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (!ct) return;
    pending_consensus_tle_  = ct->fused_tle;
    pending_consensus_seen_ = true;
}

void NeurochemState::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ev = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!ev) return;
    auto const& name = ev->name.empty() ? std::string(topic.substr(std::min(topic.size(),
                                                       std::string_view("events.").size()))) : ev->name;
    if      (name == "hit")          ++pending_hit_count_;
    else if (name == "miss")         ++pending_miss_count_;
    else if (name == "brick")        ++pending_brick_count_;
    else if (name == "wall_stuck")   ++pending_wall_stuck_;
    else if (name == "whisker_bump") ++pending_whisker_bump_;
    else if (name == "solved" || name == "failed") pending_terminal_ = true;
    else if (name == aligned_event_name_)  ++pending_aligned_count_;   // v5.3 Phase C
    // Unknown events are ignored silently.
}

void NeurochemState::handle_proprio(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() == 0) return;

    // Sensor name comes from the topic suffix or the explicit field.
    std::string sensor = pt->sensor;
    if (sensor.empty()) {
        constexpr std::string_view kPrefix = "reality.proprio.";
        if (topic.size() > kPrefix.size())
            sensor = std::string(topic.substr(kPrefix.size()));
    }

    float raw = float(pt->values[0]);
    float v = clamp01(raw);
    if      (sensor == "whisker")   pending_whisker_max_   = std::max(pending_whisker_max_, v);
    else if (sensor == "hunger")    pending_hunger_max_    = std::max(pending_hunger_max_, v);
    else if (sensor == "pheromone") pending_pheromone_max_ = std::max(pending_pheromone_max_, v);
    // Scent is a CONCENTRATION (sum of diffusion envelopes), not a [0,1] sensor —
    // with the room-filling field it sits ~1-4, so clamp01 would saturate it and
    // kill the Δscent "warmer/cooler" gradient.  Use the raw value for the delta.
    else if (sensor == "scent")     pending_scent_max_     = std::max(pending_scent_max_, raw);
    else if (sensor == "green_fraction") pending_green_max_ = std::max(pending_green_max_, v);
    else if (sensor == "travel")    pending_travel_max_    = std::max(pending_travel_max_, v);
    // Unknown sensors are ignored.
}

void NeurochemState::tick(uint64_t tick_id) {
    // 1. Exponential decay toward baselines.
    dopamine_  = float(da_baseline_) + (dopamine_  - float(da_baseline_))  * float(da_decay_);
    serotonin_ = float(ht_baseline_) + (serotonin_ - float(ht_baseline_)) * float(ht_decay_);

    // 2. Aggregate TLE from any reality.* Feedback deliveries (averaged) plus
    //    consensus.0 if present.  Then apply intrinsic-dopamine on TLE drop.
    float new_tle = 0.0f;
    int   tle_terms = 0;
    if (pending_tle_count_ > 0) {
        new_tle    += pending_tle_sum_ / float(pending_tle_count_);
        tle_terms  += 1;
    }
    if (pending_consensus_seen_) {
        new_tle    += pending_consensus_tle_;
        tle_terms  += 1;
    }
    if (tle_terms > 0) {
        new_tle /= float(tle_terms);
        if (prev_tle_set_) {
            float delta = prev_tle_ - new_tle;          // positive if prediction improved
            if (delta > 0.0f) {
                dopamine_ = clamp01(dopamine_ + float(intrinsic_da_gain_) * delta);
            }
        }
        prev_tle_     = new_tle;
        prev_tle_set_ = true;
    }

    // 3. Event-coupled (off by default) hit/miss/brick handling.
    if (pending_hit_count_ > 0) {
        total_hits_ += pending_hit_count_;
        if (event_coupled_da_)
            dopamine_ = clamp01(dopamine_ + float(da_hit_gain_) * pending_hit_count_);
    }
    // v5.3 Phase C — handtuned reward scaffold.  Always applied (not gated
    // by event_coupled_da_) because da_aligned_gain_=0 default keeps it
    // off; setting da_aligned_gain_>0 in config explicitly opts in.
    if (pending_aligned_count_ > 0 && da_aligned_gain_ > 0.0) {
        dopamine_ = clamp01(dopamine_ + float(da_aligned_gain_) * pending_aligned_count_);
    }
    if (pending_brick_count_ > 0) {
        total_bricks_ += pending_brick_count_;
        if (event_coupled_da_)
            dopamine_ = clamp01(dopamine_ + float(da_brick_gain_) * pending_brick_count_);
    }
    if (pending_miss_count_ > 0) {
        total_misses_ += pending_miss_count_;
        if (event_coupled_da_)
            dopamine_ = clamp01(dopamine_ - float(da_miss_drop_) * pending_miss_count_);
        if (event_coupled_ht_)
            serotonin_ = clamp01(serotonin_ - float(ht_miss_drop_) * pending_miss_count_);
    }

    // 4. Wall-stuck somatic aversion (always on; this is structural pain).
    if (pending_wall_stuck_ > 0) {
        dopamine_  = clamp01(dopamine_  - float(wall_stuck_da_drain_) * pending_wall_stuck_);
        serotonin_ = clamp01(serotonin_ - float(wall_stuck_ht_drain_) * pending_wall_stuck_);
    }

    // 5. Intrinsic-motivation channels.
    if (pending_whisker_max_ > 0.0f)
        serotonin_ = clamp01(serotonin_ - float(whisker_ht_rate_) * pending_whisker_max_);
    if (pending_hunger_max_ > 0.0f)
        serotonin_ = clamp01(serotonin_ - float(hunger_ht_rate_)  * pending_hunger_max_);
    if (pending_pheromone_max_ > float(pheromone_threshold_)) {
        float excess = pending_pheromone_max_ - float(pheromone_threshold_);
        serotonin_ = clamp01(serotonin_ - float(pheromone_ht_rate_) * excess);
    }
    if (pending_scent_max_ > 0.0f) {
        if (prev_scent_set_) {
            float delta = pending_scent_max_ - prev_scent_;
            if (delta > 0.0f)
                dopamine_ = clamp01(dopamine_ + float(scent_da_rate_) * delta);
        }
        prev_scent_     = pending_scent_max_;
        prev_scent_set_ = true;
    }
    // Anticipatory "hope" — dopamine on RISING green_fraction (food looming
    // larger in view = on the right path).  Potential-based (rising-only) so a
    // bug that just stares at green earns nothing; only CLOSING earns.  No
    // green in view → no rise → dopamine decays toward baseline (the aversive
    // "call to action" when combined with hunger/urgency).  Mirrors scent_da;
    // green is a cleaner long-range cue (looms monotonically; scent flattens).
    if (green_da_rate_ > 0.0) {
        if (prev_green_set_) {
            float delta = pending_green_max_ - prev_green_;
            if (delta > 0.0f)
                dopamine_ = clamp01(dopamine_ + float(green_da_rate_) * delta);
        }
        prev_green_     = pending_green_max_;
        prev_green_set_ = true;
    }
    if (pending_travel_max_ > 0.0f)
        dopamine_ = clamp01(dopamine_ + float(travel_da_rate_) * pending_travel_max_);

    // 6. NaN guard — replace any accidental NaN with the baseline (Failure
    //    Modes row in the contract: "Internal NaN... Reset that variable to
    //    its baseline, log error, continue").  We don't log here yet; Phase
    //    1+ will add a host-supplied log sink.
    if (std::isnan(dopamine_))  dopamine_  = float(da_baseline_);
    if (std::isnan(serotonin_)) serotonin_ = float(ht_baseline_);

    // Phase 6.5.3.10 — adaptive baseline.  When da_baseline_ema_alpha_ > 0,
    // EMA tracks dopamine on a slow time-scale; reward_signal becomes
    // (dopamine − EMA) — a self-zero-centering signal that mirrors the
    // Schultz reward-prediction-error model.  At α=0 (default) the EMA
    // stays at the static baseline so behaviour is byte-identical to
    // the prior fixed-baseline implementation.
    if (da_baseline_ema_alpha_ > 0.0) {
        float a = float(da_baseline_ema_alpha_);
        da_baseline_ema_ = (1.0f - a) * da_baseline_ema_ + a * dopamine_;
    } else {
        da_baseline_ema_ = float(da_baseline_);
    }

    // 7. Build and publish NeuroState.
    auto out = std::make_shared<NeuroState>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("neurochem") : id_;
    out->dopamine                  = dopamine_;
    out->serotonin                 = serotonin_;
    out->reward_signal             = dopamine_ - da_baseline_ema_;
    out->epsilon_b_scale           = scale_epsilon_b();
    out->min_insertion_error_scale = scale_min_insertion();
    out->mitosis_threshold_scale   = scale_mitosis();
    out->novelty_threshold_scale   = scale_novelty();
    bus_->publish(topics::kNeuroState, out);

    // Phase 6.5.3.F — terminal-event flush.  When events.solved or .failed
    // fired since the last tick, the queued hit/miss has now been processed
    // and the resulting reward_signal published — meaning ActionDecoder's
    // next td_update will credit the terminal-action with this tick's full
    // reward_signal (the goal-causing or failure-causing action gets
    // attributed correctly).  Reset dopamine + EMA to baselines so the
    // decay tail of the terminal pulse doesn't leak into the new episode's
    // TD updates via subsequent ticks' reward_signal != 0.  Diagnosed in
    // §17 (MountainCar hot-streak collapse: seed 44 eps 38–55 rewards
    // monotonically improved -992 → -290, then immediately collapsed
    // because the goal-hit dopamine decay tail credited random reset-state
    // actions over ~10-20 subsequent ticks of the new episode).
    if (pending_terminal_) {
        dopamine_         = float(da_baseline_);
        da_baseline_ema_  = float(da_baseline_);
        pending_terminal_ = false;
    }

    reset_pending();
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4 — instance cloning)
// ---------------------------------------------------------------------------
//
// Captures all working state.  Static parameters (da_baseline_, da_decay_,
// da_hit_gain_, etc.) are NOT included — those come from GraphConfig and
// are restored when the clone is constructed from the same config.

nlohmann::json NeurochemState::snapshot_state() const {
    return nlohmann::json{
        {"version", 1},
        // Working state.
        {"dopamine",                dopamine_},
        {"serotonin",               serotonin_},
        {"da_baseline_ema",         da_baseline_ema_},
        {"prev_tle_set",            prev_tle_set_},
        {"prev_tle",                prev_tle_},
        {"prev_scent_set",          prev_scent_set_},
        {"prev_scent",              prev_scent_},
        // Pending per-tick signals.
        {"pending_hit_count",       pending_hit_count_},
        {"pending_miss_count",      pending_miss_count_},
        {"pending_brick_count",     pending_brick_count_},
        {"pending_wall_stuck",      pending_wall_stuck_},
        {"pending_whisker_bump",    pending_whisker_bump_},
        {"pending_tle_sum",         pending_tle_sum_},
        {"pending_tle_count",       pending_tle_count_},
        {"pending_consensus_tle",   pending_consensus_tle_},
        {"pending_consensus_seen",  pending_consensus_seen_},
        {"pending_whisker_max",     pending_whisker_max_},
        {"pending_hunger_max",      pending_hunger_max_},
        {"pending_pheromone_max",   pending_pheromone_max_},
        {"pending_scent_max",       pending_scent_max_},
        {"pending_travel_max",      pending_travel_max_},
        {"pending_terminal",        pending_terminal_},
        // Telemetry counters.
        {"total_hits",              total_hits_},
        {"total_misses",            total_misses_},
        {"total_bricks",            total_bricks_},
    };
}

void NeurochemState::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("NeurochemState::restore_state: unknown snapshot version " +
                                 std::to_string(version));
    }
    dopamine_                = s.value("dopamine",                dopamine_);
    serotonin_               = s.value("serotonin",               serotonin_);
    da_baseline_ema_         = s.value("da_baseline_ema",         da_baseline_ema_);
    prev_tle_set_            = s.value("prev_tle_set",            prev_tle_set_);
    prev_tle_                = s.value("prev_tle",                prev_tle_);
    prev_scent_set_          = s.value("prev_scent_set",          prev_scent_set_);
    prev_scent_              = s.value("prev_scent",              prev_scent_);
    pending_hit_count_       = s.value("pending_hit_count",       pending_hit_count_);
    pending_miss_count_      = s.value("pending_miss_count",      pending_miss_count_);
    pending_brick_count_     = s.value("pending_brick_count",     pending_brick_count_);
    pending_wall_stuck_      = s.value("pending_wall_stuck",      pending_wall_stuck_);
    pending_whisker_bump_    = s.value("pending_whisker_bump",    pending_whisker_bump_);
    pending_tle_sum_         = s.value("pending_tle_sum",         pending_tle_sum_);
    pending_tle_count_       = s.value("pending_tle_count",       pending_tle_count_);
    pending_consensus_tle_   = s.value("pending_consensus_tle",   pending_consensus_tle_);
    pending_consensus_seen_  = s.value("pending_consensus_seen",  pending_consensus_seen_);
    pending_whisker_max_     = s.value("pending_whisker_max",     pending_whisker_max_);
    pending_hunger_max_      = s.value("pending_hunger_max",      pending_hunger_max_);
    pending_pheromone_max_   = s.value("pending_pheromone_max",   pending_pheromone_max_);
    pending_scent_max_       = s.value("pending_scent_max",       pending_scent_max_);
    pending_travel_max_      = s.value("pending_travel_max",      pending_travel_max_);
    pending_terminal_        = s.value("pending_terminal",        pending_terminal_);
    total_hits_              = s.value("total_hits",              total_hits_);
    total_misses_            = s.value("total_misses",            total_misses_);
    total_bricks_            = s.value("total_bricks",            total_bricks_);
}

void NeurochemState::reset_pending() {
    pending_hit_count_      = 0;
    pending_aligned_count_  = 0;   // v5.3 Phase C
    pending_miss_count_     = 0;
    pending_brick_count_    = 0;
    pending_wall_stuck_     = 0;
    pending_whisker_bump_   = 0;
    pending_tle_sum_        = 0.0f;
    pending_tle_count_      = 0;
    pending_consensus_tle_  = 0.0f;
    pending_consensus_seen_ = false;
    pending_whisker_max_    = 0.0f;
    pending_hunger_max_     = 0.0f;
    pending_pheromone_max_  = 0.0f;
    pending_scent_max_      = 0.0f;
    pending_green_max_      = 0.0f;
    pending_travel_max_     = 0.0f;
}

float NeurochemState::scale_epsilon_b() const {
    // v3 formula: 0.3 + 2.2 * dopamine, range [0.3, 2.5].
    return 0.3f + 2.2f * dopamine_;
}

float NeurochemState::scale_min_insertion() const {
    // v3 formula: 1.8 - 1.3 * serotonin, range [0.5, 1.8].
    return 1.8f - 1.3f * serotonin_;
}

float NeurochemState::scale_mitosis() const {
    // v3 formula: 0.6 + 1.2 * serotonin, range [0.6, 1.8].
    return 0.6f + 1.2f * serotonin_;
}

float NeurochemState::scale_novelty() const {
    // Documented contract range [0.5, 1.5]; v3 didn't expose this; we derive
    // it from dopamine here (high dopamine → looser novelty threshold to
    // promote exploitation around reward-proximal states).
    return 0.5f + 1.0f * dopamine_;
}

} // namespace ogma
