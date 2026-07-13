#include "ogma/modules/WhiskerAversionReflex.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

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
    throw std::invalid_argument("WhiskerAversionReflex: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("WhiskerAversionReflex: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("WhiskerAversionReflex: param '" + key + "' must be a string");
}

} // namespace

WhiskerAversionReflex::WhiskerAversionReflex()  = default;
WhiskerAversionReflex::~WhiskerAversionReflex() = default;

std::string_view WhiskerAversionReflex::type_name() const { return "WhiskerAversionReflex"; }

std::vector<TopicSpec> WhiskerAversionReflex::input_topics() const {
    std::vector<TopicSpec> ts;
    ts.push_back(TopicSpec{whisker_topic_prefix_, std::type_index(typeid(ProprioToken)),
                            SubscriptionKind::Direct, /*required=*/false});
    if (!suppression_topic_.empty())
        ts.push_back(TopicSpec{suppression_topic_, std::type_index(typeid(ReflexGate)),
                                SubscriptionKind::Direct, /*required=*/false});
    return ts;
}

std::vector<TopicSpec> WhiskerAversionReflex::output_topics() const {
    return {
        TopicSpec{"events.miss",       std::type_index(typeid(EnvEvent))},
        TopicSpec{"events.wall_stuck", std::type_index(typeid(EnvEvent))},
    };
}

ParamSchema WhiskerAversionReflex::params_schema() const {
    return {
        {"whisker_topic_prefix", ParamMutability::ConstructionOnly,
            "Prefix-pattern subscription for whisker proprio topics",
            ParamValue{std::string("reality.proprio.whisker_")}},
        {"threshold", ParamMutability::HotMutable,
            "Max-contact value above which events.miss fires (0..1)",
            ParamValue{0.30}},
        {"wall_stuck_threshold", ParamMutability::HotMutable,
            "Max-contact value above which events.wall_stuck fires (0..1, no refractory)",
            ParamValue{0.55}},
        {"refractory_ticks", ParamMutability::HotMutable,
            "Ticks to suppress further events.miss after one fires",
            ParamValue{int64_t{30}}},
        {"suppression_topic", ParamMutability::ConstructionOnly,
            "Optional ReflexGate input — when set, miss-event intensity is "
            "scaled by (1 - gate.value) while gate.active is true.  Empty "
            "string = no suppression.  Replaces the body-side scent-gate.",
            ParamValue{std::string("")}},
    };
}

ParamMap WhiskerAversionReflex::current_params() const {
    ParamMap m;
    m["whisker_topic_prefix"] = ParamValue{whisker_topic_prefix_};
    m["threshold"]            = ParamValue{double(threshold_)};
    m["wall_stuck_threshold"] = ParamValue{double(wall_stuck_threshold_)};
    m["refractory_ticks"]     = ParamValue{int64_t(refractory_ticks_)};
    m["suppression_topic"]    = ParamValue{suppression_topic_};
    return m;
}

void WhiskerAversionReflex::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("WhiskerAversionReflex requires a non-null Bus");

    apply_param(params, "whisker_topic_prefix",
        [&](auto const& v){ whisker_topic_prefix_ = get_string(v, "whisker_topic_prefix"); });
    apply_param(params, "threshold",
        [&](auto const& v){ threshold_ = float(get_double(v, "threshold")); });
    apply_param(params, "wall_stuck_threshold",
        [&](auto const& v){ wall_stuck_threshold_ = float(get_double(v, "wall_stuck_threshold")); });
    apply_param(params, "refractory_ticks",
        [&](auto const& v){ refractory_ticks_ = int(get_int(v, "refractory_ticks")); });
    apply_param(params, "suppression_topic",
        [&](auto const& v){ suppression_topic_ = get_string(v, "suppression_topic"); });

    if (!suppression_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(suppression_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_gate(p); }));
    }

    // The Bus only treats trailing-dot patterns as prefixes; our default
    // "reality.proprio.whisker_" doesn't qualify (ends in underscore).
    // Subscribe to the parent "reality.proprio." prefix and filter inside
    // handle_whisker by the configured whisker_topic_prefix_.  This keeps
    // the param free-form (any custom topic prefix the host wants to use).
    std::string parent_prefix = whisker_topic_prefix_;
    auto last_dot = parent_prefix.rfind('.');
    if (last_dot != std::string::npos) {
        parent_prefix = parent_prefix.substr(0, last_dot + 1);
    } else {
        parent_prefix.push_back('.');
    }
    sub_ids_.push_back(bus_->subscribe(parent_prefix, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_whisker(t, p); }));
}

void WhiskerAversionReflex::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "threshold")                threshold_            = float(get_double(value, k));
    else if (k == "wall_stuck_threshold") wall_stuck_threshold_ = float(get_double(value, k));
    else if (k == "refractory_ticks")     refractory_ticks_     = int(get_int(value, k));
    else throw std::invalid_argument("WhiskerAversionReflex: unknown/non-mutable param '" + k + "'");
}

void WhiskerAversionReflex::handle_gate(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto g = std::dynamic_pointer_cast<const ReflexGate>(payload);
    if (!g) return;
    last_gate_value_  = g->value;
    last_gate_active_ = g->active;
}

void WhiskerAversionReflex::handle_whisker(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    // Filter to only topics matching the configured whisker prefix; the bus
    // delivered us the broader "reality.proprio." subscription (the bus only
    // honors trailing-dot prefixes, see on_setup).
    if (topic.size() < whisker_topic_prefix_.size()) return;
    if (topic.compare(0, whisker_topic_prefix_.size(), whisker_topic_prefix_) != 0) return;

    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() == 0) return;
    last_values_[std::string(topic)] = pt->values[0];
}

void WhiskerAversionReflex::tick(uint64_t tick_id) {
    float max_w = 0.0f;
    for (auto const& [_, v] : last_values_) {
        if (v > max_w) max_w = v;
    }
    last_max_w_ = max_w;

    // wall_stuck has no refractory — fires every tick max contact stays high.
    if (max_w > wall_stuck_threshold_) {
        auto e = std::make_shared<EnvEvent>();
        e->tick_id   = tick_id;
        e->name      = "wall_stuck";
        e->intensity = max_w;
        bus_->publish("events.wall_stuck", e);
        ++wall_stuck_count_;
    }

    // Match the body-side `if (_miss_refractory > 0): _miss_refractory -= 1
    // elif max_w > threshold: fire` semantics — a refractory tick consumes
    // the eligibility slot, so refractory_ticks=N means "fire, then skip the
    // next N ticks" (next eligible fire is at fire_tick + N + 1).
    if (refractory_remaining_ > 0) {
        --refractory_remaining_;
    } else if (max_w > threshold_) {
        // Scale miss-event intensity by (1 - gate.value) when an active
        // ReflexGate suppression input is present.  Triggering still uses
        // raw max_w against threshold so the gate softens but doesn't
        // suppress the trigger entirely (matches body's pre-port semantics).
        float gate = (last_gate_active_) ? last_gate_value_ : 0.0f;
        auto e = std::make_shared<EnvEvent>();
        e->tick_id   = tick_id;
        e->name      = "miss";
        e->intensity = max_w * (1.0f - gate);
        bus_->publish("events.miss", e);
        ++miss_count_;
        refractory_remaining_ = refractory_ticks_;
    }
}

nlohmann::json WhiskerAversionReflex::snapshot_state() const {
    nlohmann::json values = nlohmann::json::object();
    for (auto const& [k, v] : last_values_) values[k] = v;
    return nlohmann::json{
        {"version",              1},
        {"last_values",          values},
        {"refractory_remaining", refractory_remaining_},
        {"miss_count",           miss_count_},
        {"wall_stuck_count",     wall_stuck_count_},
        {"last_max_w",           last_max_w_},
        {"last_gate_value",      last_gate_value_},
        {"last_gate_active",     last_gate_active_},
    };
}

void WhiskerAversionReflex::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("WhiskerAversionReflex::restore_state: unknown version " +
                                 std::to_string(version));
    }
    last_values_.clear();
    if (s.contains("last_values") && s["last_values"].is_object())
        for (auto it = s["last_values"].begin(); it != s["last_values"].end(); ++it)
            last_values_[it.key()] = it.value().get<float>();
    refractory_remaining_ = s.value("refractory_remaining", refractory_remaining_);
    miss_count_           = s.value("miss_count",           miss_count_);
    wall_stuck_count_     = s.value("wall_stuck_count",     wall_stuck_count_);
    last_max_w_           = s.value("last_max_w",           last_max_w_);
    last_gate_value_      = s.value("last_gate_value",      last_gate_value_);
    last_gate_active_     = s.value("last_gate_active",     last_gate_active_);
}

} // namespace ogma
