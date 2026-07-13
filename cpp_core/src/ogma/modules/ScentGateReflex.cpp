#include "ogma/modules/ScentGateReflex.hpp"

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
    throw std::invalid_argument("ScentGateReflex: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("ScentGateReflex: param '" + key + "' must be integer");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("ScentGateReflex: param '" + key + "' must be bool");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ScentGateReflex: param '" + key + "' must be a string");
}

} // namespace

ScentGateReflex::ScentGateReflex()  = default;
ScentGateReflex::~ScentGateReflex() = default;

std::string_view ScentGateReflex::type_name() const { return "ScentGateReflex"; }

std::vector<TopicSpec> ScentGateReflex::input_topics() const {
    return {
        TopicSpec{input_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> ScentGateReflex::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(ReflexGate))},
    };
}

ParamSchema ScentGateReflex::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly,
            "Proprio topic carrying the scalar to track",
            ParamValue{std::string("reality.proprio.scent_max")}},
        {"input_index", ParamMutability::ConstructionOnly,
            "Index into the proprio values vector",
            ParamValue{int64_t{0}}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Topic on which ReflexGate tokens are published",
            ParamValue{std::string("reflex.gate.scent_aversion")}},
        {"alpha_short", ParamMutability::HotMutable,
            "Short-EMA alpha (~10-tick window at 0.1)",
            ParamValue{0.1}},
        {"alpha_long", ParamMutability::HotMutable,
            "Long-EMA alpha (~17s window at 0.001)",
            ParamValue{0.001}},
        {"cap", ParamMutability::HotMutable,
            "Maximum suppression value (0..1)",
            ParamValue{0.5}},
        {"enabled", ParamMutability::HotMutable,
            "Runtime kill switch; when false the gate is permanently 0/inactive",
            ParamValue{true}},
        {"long_pos_min", ParamMutability::HotMutable,
            "Long-EMA must exceed this before the gate goes active (warmup floor)",
            ParamValue{0.001}},
    };
}

ParamMap ScentGateReflex::current_params() const {
    ParamMap m;
    m["input_topic"]  = ParamValue{input_topic_};
    m["input_index"]  = ParamValue{int64_t(input_index_)};
    m["output_topic"] = ParamValue{output_topic_};
    m["alpha_short"]  = ParamValue{double(alpha_short_)};
    m["alpha_long"]   = ParamValue{double(alpha_long_)};
    m["cap"]          = ParamValue{double(cap_)};
    m["enabled"]      = ParamValue{enabled_};
    m["long_pos_min"] = ParamValue{double(long_pos_min_)};
    return m;
}

void ScentGateReflex::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ScentGateReflex requires a non-null Bus");

    apply_param(params, "input_topic",
        [&](auto const& v){ input_topic_ = get_string(v, "input_topic"); });
    apply_param(params, "input_index",
        [&](auto const& v){ input_index_ = int(get_int(v, "input_index")); });
    apply_param(params, "output_topic",
        [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "alpha_short",
        [&](auto const& v){ alpha_short_ = float(get_double(v, "alpha_short")); });
    apply_param(params, "alpha_long",
        [&](auto const& v){ alpha_long_ = float(get_double(v, "alpha_long")); });
    apply_param(params, "cap",
        [&](auto const& v){ cap_ = float(get_double(v, "cap")); });
    apply_param(params, "enabled",
        [&](auto const& v){ enabled_ = get_bool(v, "enabled"); });
    apply_param(params, "long_pos_min",
        [&](auto const& v){ long_pos_min_ = float(get_double(v, "long_pos_min")); });

    sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_input(p); }));
}

void ScentGateReflex::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "alpha_short")        alpha_short_ = float(get_double(value, k));
    else if (k == "alpha_long")    alpha_long_  = float(get_double(value, k));
    else if (k == "cap")           cap_         = float(get_double(value, k));
    else if (k == "enabled")       enabled_     = get_bool(value, k);
    else if (k == "long_pos_min")  long_pos_min_= float(get_double(value, k));
    else throw std::invalid_argument("ScentGateReflex: unknown/non-mutable param '" + k + "'");
}

void ScentGateReflex::handle_input(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() <= input_index_) return;
    float x = pt->values[input_index_];
    if (!ema_initialized_) {
        short_ema_       = x;
        long_ema_        = x;
        ema_initialized_ = true;
        return;
    }
    short_ema_ = (1.0f - alpha_short_) * short_ema_ + alpha_short_ * x;
    long_ema_  = (1.0f - alpha_long_)  * long_ema_  + alpha_long_  * x;
}

void ScentGateReflex::tick(uint64_t tick_id) {
    auto out = std::make_shared<ReflexGate>();
    out->tick_id = tick_id;

    if (!enabled_ || !ema_initialized_ || long_ema_ <= long_pos_min_) {
        out->value  = 0.0f;
        out->active = false;
        last_value_ = 0.0f;
    } else {
        float ratio = (short_ema_ - long_ema_) / long_ema_;
        out->value  = std::clamp(ratio, 0.0f, cap_);
        out->active = true;
        last_value_ = out->value;
    }
    bus_->publish(output_topic_, out);
}

nlohmann::json ScentGateReflex::snapshot_state() const {
    return nlohmann::json{
        {"version",          1},
        {"ema_initialized",  ema_initialized_},
        {"short_ema",        short_ema_},
        {"long_ema",         long_ema_},
        {"last_value",       last_value_},
    };
}

void ScentGateReflex::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("ScentGateReflex::restore_state: unknown version " +
                                 std::to_string(version));
    }
    ema_initialized_ = s.value("ema_initialized", ema_initialized_);
    short_ema_       = s.value("short_ema",       short_ema_);
    long_ema_        = s.value("long_ema",        long_ema_);
    last_value_      = s.value("last_value",      last_value_);
}

} // namespace ogma
