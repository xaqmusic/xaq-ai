#include "ogma/modules/AdaptiveThresholdTracker.hpp"

#include <cmath>
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
    throw std::invalid_argument("AdaptiveThresholdTracker: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("AdaptiveThresholdTracker: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("AdaptiveThresholdTracker: param '" + key + "' must be a string");
}

} // namespace

AdaptiveThresholdTracker::AdaptiveThresholdTracker()  = default;
AdaptiveThresholdTracker::~AdaptiveThresholdTracker() = default;

std::string_view AdaptiveThresholdTracker::type_name() const { return "AdaptiveThresholdTracker"; }

std::vector<TopicSpec> AdaptiveThresholdTracker::input_topics() const {
    return {
        TopicSpec{input_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> AdaptiveThresholdTracker::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(AdaptiveThreshold))},
    };
}

ParamSchema AdaptiveThresholdTracker::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly,
            "Proprio topic providing the scalar to track",
            ParamValue{std::string("reality.proprio.scent_max")}},
        {"input_index", ParamMutability::ConstructionOnly,
            "Index into the proprio values vector",
            ParamValue{int64_t{0}}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Topic on which AdaptiveThreshold tokens are published",
            ParamValue{std::string("metrics.adaptive_threshold.scent_max")}},
        {"alpha", ParamMutability::HotMutable,
            "EMA alpha for both mean and variance trackers (smaller = slower)",
            ParamValue{0.001}},
        {"n_stddev", ParamMutability::HotMutable,
            "Threshold = mean + n_stddev × stddev",
            ParamValue{2.0}},
        {"warmup_ticks", ParamMutability::HotMutable,
            "Samples required before `warm` flag flips true",
            ParamValue{int64_t{1000}}},
        {"min_stddev", ParamMutability::HotMutable,
            "Floor on stddev to avoid div-by-zero / NaN",
            ParamValue{1e-6}},
    };
}

ParamMap AdaptiveThresholdTracker::current_params() const {
    ParamMap m;
    m["input_topic"]    = ParamValue{input_topic_};
    m["input_index"]    = ParamValue{int64_t(input_index_)};
    m["output_topic"]   = ParamValue{output_topic_};
    m["alpha"]          = ParamValue{double(alpha_)};
    m["n_stddev"]       = ParamValue{double(n_stddev_)};
    m["warmup_ticks"]   = ParamValue{int64_t(warmup_ticks_)};
    m["min_stddev"]     = ParamValue{double(min_stddev_)};
    return m;
}

void AdaptiveThresholdTracker::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("AdaptiveThresholdTracker requires a non-null Bus");

    apply_param(params, "input_topic",
        [&](auto const& v){ input_topic_ = get_string(v, "input_topic"); });
    apply_param(params, "input_index",
        [&](auto const& v){ input_index_ = int(get_int(v, "input_index")); });
    apply_param(params, "output_topic",
        [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "alpha",
        [&](auto const& v){ alpha_ = float(get_double(v, "alpha")); });
    apply_param(params, "n_stddev",
        [&](auto const& v){ n_stddev_ = float(get_double(v, "n_stddev")); });
    apply_param(params, "warmup_ticks",
        [&](auto const& v){ warmup_ticks_ = int(get_int(v, "warmup_ticks")); });
    apply_param(params, "min_stddev",
        [&](auto const& v){ min_stddev_ = float(get_double(v, "min_stddev")); });

    sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_input(p); }));
}

void AdaptiveThresholdTracker::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "alpha")             alpha_         = float(get_double(value, k));
    else if (k == "n_stddev")     n_stddev_      = float(get_double(value, k));
    else if (k == "warmup_ticks") warmup_ticks_  = int(get_int(value, k));
    else if (k == "min_stddev")   min_stddev_    = float(get_double(value, k));
    else throw std::invalid_argument("AdaptiveThresholdTracker: unknown/non-mutable param '" + k + "'");
}

void AdaptiveThresholdTracker::handle_input(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() <= input_index_) return;
    float x  = pt->values[input_index_];
    float dx = x - mean_ema_;
    mean_ema_ = (1.0f - alpha_) * mean_ema_ + alpha_ * x;
    var_ema_  = (1.0f - alpha_) * var_ema_  + alpha_ * dx * dx;
    ++samples_seen_;
}

float AdaptiveThresholdTracker::stddev() const {
    float s = std::sqrt(std::max(0.0f, var_ema_));
    return std::max(s, min_stddev_);
}

float AdaptiveThresholdTracker::threshold() const {
    return mean_ema_ + n_stddev_ * stddev();
}

void AdaptiveThresholdTracker::tick(uint64_t tick_id) {
    auto t = std::make_shared<AdaptiveThreshold>();
    t->tick_id   = tick_id;
    t->mean      = mean_ema_;
    t->stddev    = stddev();
    t->threshold = threshold();
    t->warm      = warm();
    bus_->publish(output_topic_, t);
}

nlohmann::json AdaptiveThresholdTracker::snapshot_state() const {
    return nlohmann::json{
        {"version",       1},
        {"mean_ema",      mean_ema_},
        {"var_ema",       var_ema_},
        {"samples_seen",  samples_seen_},
    };
}

void AdaptiveThresholdTracker::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("AdaptiveThresholdTracker::restore_state: unknown version " +
                                 std::to_string(version));
    }
    mean_ema_     = s.value("mean_ema",     mean_ema_);
    var_ema_      = s.value("var_ema",      var_ema_);
    samples_seen_ = s.value("samples_seen", samples_seen_);
}

} // namespace ogma
