#include "ogma/modules/EventConjunction.hpp"

#include <algorithm>
#include <numeric>
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
    throw std::invalid_argument("EventConjunction: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("EventConjunction: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("EventConjunction: param '" + key + "' must be a string");
}

std::vector<std::string> get_string_list(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    if (auto p = std::get_if<std::string>(&v)) return {*p};   // single-string convenience
    throw std::invalid_argument("EventConjunction: param '" + key + "' must be a string array");
}

} // namespace

EventConjunction::EventConjunction()  = default;
EventConjunction::~EventConjunction() = default;

std::string_view EventConjunction::type_name() const { return "EventConjunction"; }

std::vector<TopicSpec> EventConjunction::input_topics() const {
    std::vector<TopicSpec> ts;
    for (auto const& t : input_event_topics_) {
        ts.push_back(TopicSpec{t, std::type_index(typeid(EnvEvent)),
                                SubscriptionKind::Direct, /*required=*/false});
    }
    if (!motion_floor_topic_.empty()) {
        ts.push_back(TopicSpec{motion_floor_topic_, std::type_index(typeid(ProprioToken)),
                                SubscriptionKind::Direct, /*required=*/false});
    }
    return ts;
}

std::vector<TopicSpec> EventConjunction::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(EnvEvent))},
    };
}

ParamSchema EventConjunction::params_schema() const {
    return {
        {"input_event_topics", ParamMutability::ConstructionOnly,
            "Array of event topic names (e.g., [\"events.scent_hit\", \"events.green_visible\"]) — fires output when ALL have fired within window_ticks.",
            std::nullopt},
        {"output_event_name", ParamMutability::ConstructionOnly,
            "Event suffix; published as events.<this> when conjunction holds",
            ParamValue{std::string("conjunction")}},
        {"window_ticks", ParamMutability::HotMutable,
            "Conjunction window: each input must have fired within the last N ticks (default 30 = ~500ms at 60Hz)",
            ParamValue{int64_t{30}}},
        {"refractory_ticks", ParamMutability::HotMutable,
            "Suppress further conjunction events for N ticks after one fires (0=off, default 30)",
            ParamValue{int64_t{30}}},
        {"motion_floor_topic", ParamMutability::ConstructionOnly,
            "Optional gating proprio topic — fire only if its scalar > motion_floor_min",
            ParamValue{std::string("")}},
        {"motion_floor_index", ParamMutability::ConstructionOnly,
            "Index into motion-floor proprio values vector",
            ParamValue{int64_t{0}}},
        {"motion_floor_min", ParamMutability::HotMutable,
            "Minimum motion-floor scalar required to fire",
            ParamValue{0.0}},
    };
}

ParamMap EventConjunction::current_params() const {
    ParamMap m;
    m["input_event_topics"]  = ParamValue{input_event_topics_};
    m["output_event_name"]   = ParamValue{output_event_name_};
    m["window_ticks"]        = ParamValue{int64_t(window_ticks_)};
    m["refractory_ticks"]    = ParamValue{int64_t(refractory_ticks_)};
    m["motion_floor_topic"]  = ParamValue{motion_floor_topic_};
    m["motion_floor_index"]  = ParamValue{int64_t(motion_floor_index_)};
    m["motion_floor_min"]    = ParamValue{double(motion_floor_min_)};
    return m;
}

void EventConjunction::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("EventConjunction requires a non-null Bus");

    auto it = params.find("input_event_topics");
    if (it == params.end())
        throw std::invalid_argument("EventConjunction: required param 'input_event_topics' missing");
    input_event_topics_ = get_string_list(it->second, "input_event_topics");
    if (input_event_topics_.size() < 2)
        throw std::invalid_argument("EventConjunction: input_event_topics needs at least 2 entries");

    apply_param(params, "output_event_name",
        [&](auto const& v){ output_event_name_ = get_string(v, "output_event_name"); });
    apply_param(params, "window_ticks",
        [&](auto const& v){ window_ticks_ = std::max(1, int(get_int(v, "window_ticks"))); });
    apply_param(params, "refractory_ticks",
        [&](auto const& v){ refractory_ticks_ = std::max(0, int(get_int(v, "refractory_ticks"))); });
    apply_param(params, "motion_floor_topic",
        [&](auto const& v){ motion_floor_topic_ = get_string(v, "motion_floor_topic"); });
    apply_param(params, "motion_floor_index",
        [&](auto const& v){ motion_floor_index_ = int(get_int(v, "motion_floor_index")); });
    apply_param(params, "motion_floor_min",
        [&](auto const& v){ motion_floor_min_ = float(get_double(v, "motion_floor_min")); });

    if (output_event_name_.empty())
        throw std::invalid_argument("EventConjunction: output_event_name must not be empty");
    output_topic_ = std::string("events.") + output_event_name_;

    last_fire_ticks_.assign(input_event_topics_.size(), int64_t(-1));
    last_intensities_.assign(input_event_topics_.size(), 0.0f);

    for (std::size_t i = 0; i < input_event_topics_.size(); ++i) {
        std::string topic = input_event_topics_[i];
        std::size_t idx   = i;
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct,
            [this, idx](std::string_view, MessagePtr p){ handle_event(idx, p); }));
    }
    if (!motion_floor_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(motion_floor_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_motion(p); }));
    }
}

void EventConjunction::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "window_ticks")           window_ticks_     = std::max(1, int(get_int(value, k)));
    else if (k == "refractory_ticks")  refractory_ticks_ = std::max(0, int(get_int(value, k)));
    else if (k == "motion_floor_min")  motion_floor_min_ = float(get_double(value, k));
    else throw std::invalid_argument("EventConjunction: unknown/non-mutable param '" + k + "'");
}

void EventConjunction::handle_event(std::size_t input_idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!e) return;
    if (input_idx >= last_fire_ticks_.size()) return;
    last_fire_ticks_[input_idx]  = int64_t(e->tick_id);
    last_intensities_[input_idx] = e->intensity;
    ++inputs_seen_total_;
}

void EventConjunction::handle_motion(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() <= motion_floor_index_) return;
    motion_val_  = pt->values[motion_floor_index_];
    motion_seen_ = true;
}

void EventConjunction::tick(uint64_t tick_id) {
    if (refractory_remaining_ > 0) --refractory_remaining_;
    if (refractory_remaining_ > 0) return;

    // All inputs must have fired within window_ticks of NOW.
    int64_t now = int64_t(tick_id);
    for (auto t : last_fire_ticks_) {
        if (t < 0) return;                          // never fired
        if (now - t > window_ticks_) return;        // expired
    }
    if (!motion_floor_topic_.empty()) {
        if (!motion_seen_) return;
        if (motion_val_ < motion_floor_min_) return;
    }

    auto e = std::make_shared<EnvEvent>();
    e->tick_id     = tick_id;
    e->producer_id = id_.empty() ? std::string("event_conjunction") : id_;
    e->name        = output_event_name_;
    // Intensity = mean of input intensities (each input fired with its own
    // strength signal; conjunction intensity is the average).
    double sum = 0.0;
    for (auto v : last_intensities_) sum += double(v);
    e->intensity   = float(sum / double(last_intensities_.size()));
    bus_->publish(output_topic_, e);
    ++fire_count_;
    refractory_remaining_ = refractory_ticks_;
}

nlohmann::json EventConjunction::snapshot_state() const {
    return nlohmann::json{
        {"version",              1},
        {"last_fire_ticks",      last_fire_ticks_},
        {"last_intensities",     last_intensities_},
        {"refractory_remaining", refractory_remaining_},
        {"fire_count",           fire_count_},
        {"inputs_seen_total",    inputs_seen_total_},
        {"motion_val",           motion_val_},
        {"motion_seen",          motion_seen_},
    };
}

void EventConjunction::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("EventConjunction::restore_state: unknown version "
                                 + std::to_string(version));
    }
    if (s.contains("last_fire_ticks"))
        last_fire_ticks_ = s["last_fire_ticks"].get<std::vector<int64_t>>();
    if (s.contains("last_intensities"))
        last_intensities_ = s["last_intensities"].get<std::vector<float>>();
    refractory_remaining_ = s.value("refractory_remaining", refractory_remaining_);
    fire_count_           = s.value("fire_count",           fire_count_);
    inputs_seen_total_    = s.value("inputs_seen_total",    inputs_seen_total_);
    motion_val_           = s.value("motion_val",           motion_val_);
    motion_seen_          = s.value("motion_seen",          motion_seen_);
}

} // namespace ogma
