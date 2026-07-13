#include "ogma/modules/StaleConfidenceDecay.hpp"

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
    throw std::invalid_argument("StaleConfidenceDecay: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("StaleConfidenceDecay: param '" + key + "' must be integer");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("StaleConfidenceDecay: param '" + key + "' must be bool");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("StaleConfidenceDecay: param '" + key + "' must be a string");
}

} // namespace

StaleConfidenceDecay::StaleConfidenceDecay()  = default;
StaleConfidenceDecay::~StaleConfidenceDecay() = default;

std::string_view StaleConfidenceDecay::type_name() const { return "StaleConfidenceDecay"; }

std::vector<TopicSpec> StaleConfidenceDecay::input_topics() const {
    return {
        TopicSpec{input_topic_, std::type_index(typeid(RealityToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> StaleConfidenceDecay::output_topics() const {
    return {
        TopicSpec{output_topic_, std::type_index(typeid(ReflexGate))},
    };
}

ParamSchema StaleConfidenceDecay::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly,
            "RealityToken topic to watch (typically one EPM's "
            "reality.<group>.<modality> output)",
            ParamValue{std::string("reality.kinematic.imu")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Topic on which the staleness ReflexGate is published",
            ParamValue{std::string("cognition.boredom")}},
        {"idle_threshold_ticks", ParamMutability::HotMutable,
            "Same-winner streak length before staleness begins ramping. "
            "Smaller = more eager to declare boredom; larger = patient.",
            ParamValue{int64_t(60)}},
        {"decay_window_ticks", ParamMutability::HotMutable,
            "Number of ticks past the threshold over which staleness "
            "ramps linearly from 0 to 1 (clamped at 1 after that).",
            ParamValue{int64_t(240)}},
        {"publish_when_zero", ParamMutability::HotMutable,
            "When true, publish a ReflexGate{value=0, active=false} every "
            "tick the winner just changed.  Default false = silent until "
            "boredom actually rises (saves bus traffic).",
            ParamValue{false}},
    };
}

ParamMap StaleConfidenceDecay::current_params() const {
    ParamMap m;
    m["input_topic"]          = ParamValue{input_topic_};
    m["output_topic"]         = ParamValue{output_topic_};
    m["idle_threshold_ticks"] = ParamValue{int64_t(idle_threshold_ticks_)};
    m["decay_window_ticks"]   = ParamValue{int64_t(decay_window_ticks_)};
    m["publish_when_zero"]    = ParamValue{publish_when_zero_};
    return m;
}

void StaleConfidenceDecay::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("StaleConfidenceDecay requires a non-null Bus");

    apply_param(params, "input_topic",          [&](auto const& v){ input_topic_  = get_string(v, "input_topic"); });
    apply_param(params, "output_topic",         [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "idle_threshold_ticks", [&](auto const& v){ idle_threshold_ticks_ = int(get_int(v, "idle_threshold_ticks")); });
    apply_param(params, "decay_window_ticks",   [&](auto const& v){ decay_window_ticks_   = int(get_int(v, "decay_window_ticks")); });
    apply_param(params, "publish_when_zero",    [&](auto const& v){ publish_when_zero_    = get_bool(v, "publish_when_zero"); });

    if (idle_threshold_ticks_ < 0)
        throw std::invalid_argument("StaleConfidenceDecay: idle_threshold_ticks must be >= 0");
    if (decay_window_ticks_ <= 0)
        throw std::invalid_argument("StaleConfidenceDecay: decay_window_ticks must be > 0");

    sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_input(p); }));
}

void StaleConfidenceDecay::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "idle_threshold_ticks") {
        int v = int(get_int(value, k));
        if (v < 0) throw std::invalid_argument("StaleConfidenceDecay: idle_threshold_ticks must be >= 0");
        idle_threshold_ticks_ = v;
    } else if (k == "decay_window_ticks") {
        int v = int(get_int(value, k));
        if (v <= 0) throw std::invalid_argument("StaleConfidenceDecay: decay_window_ticks must be > 0");
        decay_window_ticks_ = v;
    } else if (k == "publish_when_zero") {
        publish_when_zero_ = get_bool(value, k);
    } else if (k == "input_topic" || k == "output_topic") {
        throw std::invalid_argument("StaleConfidenceDecay: param '" + k + "' is ConstructionOnly");
    } else {
        throw std::invalid_argument("StaleConfidenceDecay: unknown param '" + k + "'");
    }
}

void StaleConfidenceDecay::handle_input(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt) return;
    pending_winner_id_ = rt->winner_id;
    pending_input_seen_ = true;
    ++inputs_received_;
}

void StaleConfidenceDecay::tick(uint64_t tick_id) {
    bool changed = false;
    if (pending_input_seen_) {
        if (pending_winner_id_ != current_winner_id_) {
            current_winner_id_ = pending_winner_id_;
            streak_ticks_      = 0;
            staleness_         = 0.0f;
            ++reset_count_;
            changed = true;
        } else {
            ++streak_ticks_;
        }
        pending_input_seen_ = false;
    } else {
        // No input this tick — preserve last streak/staleness; don't tick streak
        // because there's nothing to confirm.
    }

    // Compute staleness from the streak.  Below the threshold staleness is 0;
    // past the threshold it ramps linearly to 1 over decay_window_ticks.
    int over = streak_ticks_ - idle_threshold_ticks_;
    if (over <= 0 || decay_window_ticks_ <= 0) {
        staleness_ = 0.0f;
    } else if (over >= decay_window_ticks_) {
        staleness_ = 1.0f;
    } else {
        staleness_ = float(over) / float(decay_window_ticks_);
    }

    if (staleness_ > 0.0f || publish_when_zero_) {
        auto gate = std::make_shared<ReflexGate>();
        gate->tick_id     = tick_id;
        gate->producer_id = id_.empty() ? std::string("stale_decay") : id_;
        gate->value       = staleness_;
        gate->active      = staleness_ > 0.0f;
        bus_->publish(output_topic_, gate);
    }
    (void)changed;
}

nlohmann::json StaleConfidenceDecay::snapshot_state() const {
    return nlohmann::json{
        {"version",            1},
        {"current_winner_id",  current_winner_id_},
        {"streak_ticks",       streak_ticks_},
        {"staleness",          staleness_},
        {"reset_count",        reset_count_},
        {"inputs_received",    inputs_received_},
    };
}

void StaleConfidenceDecay::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("StaleConfidenceDecay::restore_state: unknown version " +
                                 std::to_string(version));
    }
    current_winner_id_ = s.value("current_winner_id", current_winner_id_);
    streak_ticks_      = s.value("streak_ticks",      streak_ticks_);
    staleness_         = s.value("staleness",         staleness_);
    reset_count_       = s.value("reset_count",       reset_count_);
    inputs_received_   = s.value("inputs_received",   inputs_received_);
}

} // namespace ogma
