#include "ogma/modules/DualEMADetector.hpp"

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
    throw std::invalid_argument("DualEMADetector: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("DualEMADetector: param '" + key + "' must be integer");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("DualEMADetector: param '" + key + "' must be bool");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("DualEMADetector: param '" + key + "' must be a string");
}

} // namespace

DualEMADetector::DualEMADetector()  = default;
DualEMADetector::~DualEMADetector() = default;

std::string_view DualEMADetector::type_name() const { return "DualEMADetector"; }

std::vector<TopicSpec> DualEMADetector::input_topics() const {
    std::vector<TopicSpec> ts;
    ts.push_back(TopicSpec{input_topic_, std::type_index(typeid(ProprioToken)),
                            SubscriptionKind::Direct, /*required=*/false});
    if (!motion_floor_topic_.empty())
        ts.push_back(TopicSpec{motion_floor_topic_, std::type_index(typeid(ProprioToken)),
                                SubscriptionKind::Direct, /*required=*/false});
    return ts;
}

std::vector<TopicSpec> DualEMADetector::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(EnvEvent))},
    };
}

ParamSchema DualEMADetector::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly,
            "Proprio topic to monitor (scalar; use input_index to pick a dim)",
            ParamValue{std::string("reality.proprio.scent_max")}},
        {"input_index", ParamMutability::ConstructionOnly,
            "Index into the proprio values vector",
            ParamValue{int64_t{0}}},
        {"output_event_name", ParamMutability::ConstructionOnly,
            "Event suffix; published as events.<this>",
            ParamValue{std::string("hit")}},
        {"alpha_short", ParamMutability::HotMutable,
            "Short-EMA alpha (~10-tick window at 0.1)",
            ParamValue{0.1}},
        {"alpha_long", ParamMutability::HotMutable,
            "Long-EMA alpha (~17s window at 0.001)",
            ParamValue{0.001}},
        {"ratio_threshold", ParamMutability::HotMutable,
            "Fire when short_ema > long_ema * this",
            ParamValue{1.5}},
        {"refractory_ticks", ParamMutability::HotMutable,
            "Suppress further events for N ticks after one fires (0=off)",
            ParamValue{int64_t{0}}},
        {"require_long_pos", ParamMutability::HotMutable,
            "Wait until long_ema>0 before firing (avoids cold-start spurious fires)",
            ParamValue{true}},
        {"motion_floor_topic", ParamMutability::ConstructionOnly,
            "Optional gating topic — fire only if its scalar > motion_floor_min",
            ParamValue{std::string("")}},
        {"motion_floor_index", ParamMutability::ConstructionOnly,
            "Index into the motion-floor proprio values vector",
            ParamValue{int64_t{0}}},
        {"motion_floor_min", ParamMutability::HotMutable,
            "Minimum motion-floor value required to fire",
            ParamValue{0.0}},
    };
}

ParamMap DualEMADetector::current_params() const {
    ParamMap m;
    m["input_topic"]         = ParamValue{input_topic_};
    m["input_index"]         = ParamValue{int64_t(input_index_)};
    m["output_event_name"]   = ParamValue{output_event_name_};
    m["alpha_short"]         = ParamValue{double(alpha_short_)};
    m["alpha_long"]          = ParamValue{double(alpha_long_)};
    m["ratio_threshold"]     = ParamValue{double(ratio_threshold_)};
    m["refractory_ticks"]    = ParamValue{int64_t(refractory_ticks_)};
    m["require_long_pos"]    = ParamValue{require_long_pos_};
    m["motion_floor_topic"]  = ParamValue{motion_floor_topic_};
    m["motion_floor_index"]  = ParamValue{int64_t(motion_floor_index_)};
    m["motion_floor_min"]    = ParamValue{double(motion_floor_min_)};
    return m;
}

void DualEMADetector::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("DualEMADetector requires a non-null Bus");

    apply_param(params, "input_topic",
        [&](auto const& v){ input_topic_ = get_string(v, "input_topic"); });
    apply_param(params, "input_index",
        [&](auto const& v){ input_index_ = int(get_int(v, "input_index")); });
    apply_param(params, "output_event_name",
        [&](auto const& v){ output_event_name_ = get_string(v, "output_event_name"); });
    apply_param(params, "alpha_short",
        [&](auto const& v){ alpha_short_ = float(get_double(v, "alpha_short")); });
    apply_param(params, "alpha_long",
        [&](auto const& v){ alpha_long_ = float(get_double(v, "alpha_long")); });
    apply_param(params, "ratio_threshold",
        [&](auto const& v){ ratio_threshold_ = float(get_double(v, "ratio_threshold")); });
    apply_param(params, "refractory_ticks",
        [&](auto const& v){ refractory_ticks_ = int(get_int(v, "refractory_ticks")); });
    apply_param(params, "require_long_pos",
        [&](auto const& v){ require_long_pos_ = get_bool(v, "require_long_pos"); });
    apply_param(params, "motion_floor_topic",
        [&](auto const& v){ motion_floor_topic_ = get_string(v, "motion_floor_topic"); });
    apply_param(params, "motion_floor_index",
        [&](auto const& v){ motion_floor_index_ = int(get_int(v, "motion_floor_index")); });
    apply_param(params, "motion_floor_min",
        [&](auto const& v){ motion_floor_min_ = float(get_double(v, "motion_floor_min")); });

    if (output_event_name_.empty())
        throw std::invalid_argument("DualEMADetector: output_event_name must not be empty");
    output_topic_ = std::string("events.") + output_event_name_;

    sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_input(p); }));
    if (!motion_floor_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(motion_floor_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_motion(p); }));
    }
}

void DualEMADetector::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "alpha_short")           alpha_short_      = float(get_double(value, k));
    else if (k == "alpha_long")        alpha_long_       = float(get_double(value, k));
    else if (k == "ratio_threshold")   ratio_threshold_  = float(get_double(value, k));
    else if (k == "refractory_ticks")  refractory_ticks_ = int(get_int(value, k));
    else if (k == "require_long_pos")  require_long_pos_ = get_bool(value, k);
    else if (k == "motion_floor_min")  motion_floor_min_ = float(get_double(value, k));
    else throw std::invalid_argument("DualEMADetector: unknown/non-mutable param '" + k + "'");
}

void DualEMADetector::handle_input(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() <= input_index_) return;
    float x = pt->values[input_index_];
    if (!short_ema_initialized_) {
        short_ema_ = x;
        long_ema_  = x;
        short_ema_initialized_ = true;
        return;
    }
    short_ema_ = (1.0f - alpha_short_) * short_ema_ + alpha_short_ * x;
    long_ema_  = (1.0f - alpha_long_)  * long_ema_  + alpha_long_  * x;
}

void DualEMADetector::handle_motion(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() <= motion_floor_index_) return;
    motion_val_  = pt->values[motion_floor_index_];
    motion_seen_ = true;
}

void DualEMADetector::tick(uint64_t tick_id) {
    if (refractory_remaining_ > 0) --refractory_remaining_;

    if (!short_ema_initialized_) return;
    if (require_long_pos_ && long_ema_ <= 0.0f) return;
    if (refractory_remaining_ > 0) return;
    if (!motion_floor_topic_.empty()) {
        if (!motion_seen_) return;
        if (motion_val_ < motion_floor_min_) return;
    }
    if (short_ema_ <= long_ema_ * ratio_threshold_) return;

    auto e = std::make_shared<EnvEvent>();
    e->tick_id   = tick_id;
    e->name      = output_event_name_;
    e->intensity = short_ema_;
    bus_->publish(output_topic_, e);
    ++fire_count_;
    refractory_remaining_ = refractory_ticks_;
}

// ---------------------------------------------------------------------------
// Snapshot / restore (UI-dev W3.2 Tier A)
// ---------------------------------------------------------------------------

nlohmann::json DualEMADetector::snapshot_state() const {
    return nlohmann::json{
        {"version",               1},
        {"short_ema_initialized", short_ema_initialized_},
        {"short_ema",             short_ema_},
        {"long_ema",              long_ema_},
        {"motion_val",            motion_val_},
        {"motion_seen",           motion_seen_},
        {"refractory_remaining",  refractory_remaining_},
        {"fire_count",            fire_count_},
    };
}

void DualEMADetector::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("DualEMADetector::restore_state: unknown version " +
                                 std::to_string(version));
    }
    short_ema_initialized_ = s.value("short_ema_initialized", short_ema_initialized_);
    short_ema_             = s.value("short_ema",             short_ema_);
    long_ema_              = s.value("long_ema",              long_ema_);
    motion_val_            = s.value("motion_val",            motion_val_);
    motion_seen_           = s.value("motion_seen",           motion_seen_);
    refractory_remaining_  = s.value("refractory_remaining",  refractory_remaining_);
    fire_count_            = s.value("fire_count",            fire_count_);
}

} // namespace ogma
