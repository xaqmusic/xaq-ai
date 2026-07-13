#include "ogma/modules/ActionGate.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <variant>

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
    throw std::invalid_argument("ActionGate param '" + key + "' must be numeric");
}

} // namespace

ActionGate::ActionGate()  = default;
ActionGate::~ActionGate() = default;

std::string_view ActionGate::type_name() const { return "ActionGate"; }

std::vector<TopicSpec> ActionGate::input_topics() const {
    return {
        TopicSpec{topics::kPolicyIntent,         std::type_index(typeid(PolicyToken)),
                  SubscriptionKind::Direct, /*required=*/true},
        TopicSpec{topics::kExplorationDirective, std::type_index(typeid(ExplorationDirective)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> ActionGate::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(ActionOut))},
    };
}

ParamSchema ActionGate::params_schema() const {
    return {
        {"accel_min", ParamMutability::HotMutable, "Output clamp minimum",  ParamValue{-4.0}},
        {"accel_max", ParamMutability::HotMutable, "Output clamp maximum",  ParamValue{4.0}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Topic on which ActionOut is published. Use action.left / action.right "
            "for one half of a bilateral motor pair; default action.out for legacy "
            "single-channel bodies.",
            ParamValue{std::string("action.out")}},
    };
}

ParamMap ActionGate::current_params() const {
    ParamMap m;
    m["accel_min"]    = ParamValue{double(accel_min_)};
    m["accel_max"]    = ParamValue{double(accel_max_)};
    m["output_topic"] = ParamValue{output_topic_};
    return m;
}

void ActionGate::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ActionGate requires a non-null Bus");

    apply_param(params, "accel_min", [&](auto const& v){ accel_min_ = float(get_double(v, "accel_min")); });
    apply_param(params, "accel_max", [&](auto const& v){ accel_max_ = float(get_double(v, "accel_max")); });
    apply_param(params, "output_topic", [&](auto const& v){
        if (auto p = std::get_if<std::string>(&v)) output_topic_ = *p;
        else throw std::invalid_argument("ActionGate: 'output_topic' must be a string");
    });
    if (output_topic_.empty())
        throw std::invalid_argument("ActionGate: 'output_topic' must not be empty");

    sub_ids_.push_back(bus_->subscribe(topics::kPolicyIntent, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_policy(t, p); }));
    sub_ids_.push_back(bus_->subscribe(topics::kExplorationDirective, SubscriptionKind::Direct,
        [this](std::string_view t, MessagePtr p){ handle_exploration(t, p); }));
}

void ActionGate::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "accel_min")      accel_min_ = float(get_double(value, k));
    else if (k == "accel_max") accel_max_ = float(get_double(value, k));
    else throw std::invalid_argument("ActionGate: unknown param '" + k + "'");
}

void ActionGate::handle_policy(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    last_policy_ = std::dynamic_pointer_cast<const PolicyToken>(payload);
}

void ActionGate::handle_exploration(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    last_explore_ = std::dynamic_pointer_cast<const ExplorationDirective>(payload);
}

void ActionGate::tick(uint64_t tick_id) {
    auto act = std::make_shared<ActionOut>();
    act->tick_id     = tick_id;
    act->producer_id = id_.empty() ? std::string("action_gate") : id_;
    act->probe       = false;
    act->action_tle  = 0.0f;

    // Hard priority: explore overrides policy when active.
    if (last_explore_ && last_explore_->active) {
        act->accel  = std::clamp(last_explore_->accel, accel_min_, accel_max_);
        act->source = "explore";
        ++explore_count_;
    } else if (last_policy_) {
        act->accel  = std::clamp(last_policy_->weighted_accel, accel_min_, accel_max_);
        act->source = "premotor";
        ++policy_count_;
    } else {
        // Bootstrap — neither stream has produced yet.
        act->accel  = 0.0f;
        act->source = "";
    }

    last_accel_  = act->accel;
    last_source_ = act->source;
    bus_->publish(output_topic_, act);
}

// last_policy_ / last_explore_ are handler-driven shared_ptrs refreshed by
// upstream publishes before each tick; nulling them on restore is safe
// because the next tick's handler dispatch repopulates them before tick()
// reads them.  Snapshot only the scalar telemetry needed for inspector
// continuity.

nlohmann::json ActionGate::snapshot_state() const {
    return nlohmann::json{
        {"version",        1},
        {"policy_count",   policy_count_},
        {"explore_count",  explore_count_},
        {"last_accel",     last_accel_},
        {"last_source",    last_source_},
    };
}

void ActionGate::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("ActionGate::restore_state: unknown version " +
                                 std::to_string(version));
    }
    policy_count_  = s.value("policy_count",  policy_count_);
    explore_count_ = s.value("explore_count", explore_count_);
    last_accel_    = s.value("last_accel",    last_accel_);
    last_source_   = s.value("last_source",   last_source_);
}

} // namespace ogma
