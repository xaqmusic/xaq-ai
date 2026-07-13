#include "ogma/modules/ChunkOutcomeGate.hpp"

#include <algorithm>
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
    throw std::invalid_argument("ChunkOutcomeGate: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("ChunkOutcomeGate: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ChunkOutcomeGate: param '" + key + "' must be string");
}

} // namespace

ChunkOutcomeGate::ChunkOutcomeGate()  = default;
ChunkOutcomeGate::~ChunkOutcomeGate() = default;

std::string_view ChunkOutcomeGate::type_name() const { return "ChunkOutcomeGate"; }

std::vector<TopicSpec> ChunkOutcomeGate::input_topics() const {
    return {
        TopicSpec{outcome_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{action_topic_, std::type_index(typeid(ActionOut)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> ChunkOutcomeGate::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(EnvEvent))} };
}

ParamSchema ChunkOutcomeGate::params_schema() const {
    return {
        {"outcome_topic",         ParamMutability::ConstructionOnly,
            "Proprio topic to monitor (e.g., reality.proprio.scent_max)",
            ParamValue{std::string("reality.proprio.scent_max")}},
        {"outcome_index",         ParamMutability::ConstructionOnly,
            "Index into the proprio values vector",
            ParamValue{int64_t{0}}},
        {"action_topic",          ParamMutability::ConstructionOnly,
            "ActionOut topic to read chunk_id from (default action.out)",
            ParamValue{std::string("action.out")}},
        {"output_event_name",     ParamMutability::ConstructionOnly,
            "Event suffix; published as events.<this>.  Default chunk_abort so ActionDecoder's existing chunk_abort handler terminates the chunk.",
            ParamValue{std::string("chunk_abort")}},
        {"target_sign",           ParamMutability::HotMutable,
            "'rising' (fire if outcome hasn't risen by improvement_threshold) or 'falling' (mirror)",
            ParamValue{std::string("rising")}},
        {"min_check_ticks",       ParamMutability::HotMutable,
            "Wait N ticks of chunk replay before checking outcome (default 30 = ~500ms at 60Hz)",
            ParamValue{int64_t{30}}},
        {"improvement_threshold", ParamMutability::HotMutable,
            "Minimum signal change required (in target direction) within min_check_ticks; 0 = any non-improvement triggers abort",
            ParamValue{0.0}},
        {"refractory_ticks",      ParamMutability::HotMutable,
            "Suppress further aborts for N ticks (default 60)",
            ParamValue{int64_t{60}}},
    };
}

ParamMap ChunkOutcomeGate::current_params() const {
    ParamMap m;
    m["outcome_topic"]         = ParamValue{outcome_topic_};
    m["outcome_index"]         = ParamValue{int64_t(outcome_index_)};
    m["action_topic"]          = ParamValue{action_topic_};
    m["output_event_name"]     = ParamValue{output_event_name_};
    m["target_sign"]           = ParamValue{target_sign_};
    m["min_check_ticks"]       = ParamValue{int64_t(min_check_ticks_)};
    m["improvement_threshold"] = ParamValue{double(improvement_threshold_)};
    m["refractory_ticks"]      = ParamValue{int64_t(refractory_ticks_)};
    return m;
}

void ChunkOutcomeGate::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ChunkOutcomeGate requires a non-null Bus");

    apply_param(params, "outcome_topic",
        [&](auto const& v){ outcome_topic_ = get_string(v, "outcome_topic"); });
    apply_param(params, "outcome_index",
        [&](auto const& v){ outcome_index_ = int(get_int(v, "outcome_index")); });
    apply_param(params, "action_topic",
        [&](auto const& v){ action_topic_ = get_string(v, "action_topic"); });
    apply_param(params, "output_event_name",
        [&](auto const& v){ output_event_name_ = get_string(v, "output_event_name"); });
    apply_param(params, "target_sign",
        [&](auto const& v){ target_sign_ = get_string(v, "target_sign"); });
    apply_param(params, "min_check_ticks",
        [&](auto const& v){ min_check_ticks_ = std::max(1, int(get_int(v, "min_check_ticks"))); });
    apply_param(params, "improvement_threshold",
        [&](auto const& v){ improvement_threshold_ = float(get_double(v, "improvement_threshold")); });
    apply_param(params, "refractory_ticks",
        [&](auto const& v){ refractory_ticks_ = std::max(0, int(get_int(v, "refractory_ticks"))); });

    if (target_sign_ != "rising" && target_sign_ != "falling")
        throw std::invalid_argument("ChunkOutcomeGate: target_sign must be 'rising' or 'falling'");
    if (output_event_name_.empty())
        throw std::invalid_argument("ChunkOutcomeGate: output_event_name must not be empty");
    output_topic_ = std::string("events.") + output_event_name_;

    sub_ids_.push_back(bus_->subscribe(outcome_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_outcome(p); }));
    sub_ids_.push_back(bus_->subscribe(action_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_action(p); }));
}

void ChunkOutcomeGate::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "target_sign")           target_sign_           = get_string(value, k);
    else if (k == "min_check_ticks")       min_check_ticks_       = std::max(1, int(get_int(value, k)));
    else if (k == "improvement_threshold") improvement_threshold_ = float(get_double(value, k));
    else if (k == "refractory_ticks")      refractory_ticks_      = std::max(0, int(get_int(value, k)));
    else throw std::invalid_argument("ChunkOutcomeGate: unknown/non-mutable param '" + k + "'");
}

void ChunkOutcomeGate::handle_outcome(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() <= outcome_index_) return;
    current_signal_ = pt->values[outcome_index_];
    signal_seen_    = true;
}

void ChunkOutcomeGate::handle_action(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    ++action_msgs_seen_;
    int new_chunk_id = a->chunk_id;

    if (new_chunk_id != active_chunk_id_) {
        // Chunk transition: either started a new one or finished an active one.
        // -1 = no active chunk (sentinel from ActionOut); anything else = active
        // chunk id (positive for organic, negative for seeded — both valid).
        active_chunk_id_ = new_chunk_id;
        ticks_in_chunk_  = 0;
        if (new_chunk_id != -1 && signal_seen_) {
            signal_at_start_ = current_signal_;
        }
    } else if (active_chunk_id_ != -1) {
        ++ticks_in_chunk_;
    }
}

void ChunkOutcomeGate::tick(uint64_t tick_id) {
    if (refractory_remaining_ > 0) --refractory_remaining_;
    if (refractory_remaining_ > 0) return;
    if (active_chunk_id_ == -1) return;   // no chunk active (seeded chunks have negative ids — not -1)
    if (!signal_seen_) return;
    if (ticks_in_chunk_ < min_check_ticks_) return;

    float delta = current_signal_ - signal_at_start_;
    bool  unhelpful = (target_sign_ == "rising")
        ? (delta <= improvement_threshold_)
        : (-delta <= improvement_threshold_);
    if (!unhelpful) return;

    auto e = std::make_shared<EnvEvent>();
    e->tick_id     = tick_id;
    e->producer_id = id_.empty() ? std::string("chunk_outcome_gate") : id_;
    e->name        = output_event_name_;
    e->intensity   = std::abs(delta);
    bus_->publish(output_topic_, e);
    ++aborts_total_;
    last_abort_tick_      = int64_t(tick_id);
    refractory_remaining_ = refractory_ticks_;
    // Reset chunk-tracking so we don't re-fire on the next tick before
    // ActionDecoder receives the event (synchronous bus, but be safe).
    active_chunk_id_ = -1;   // sentinel = no active chunk
    ticks_in_chunk_  = 0;
}

nlohmann::json ChunkOutcomeGate::snapshot_state() const {
    return nlohmann::json{
        {"version",              1},
        {"active_chunk_id",      active_chunk_id_},
        {"ticks_in_chunk",       ticks_in_chunk_},
        {"signal_at_start",      signal_at_start_},
        {"current_signal",       current_signal_},
        {"signal_seen",          signal_seen_},
        {"refractory_remaining", refractory_remaining_},
        {"aborts_total",         aborts_total_},
        {"last_abort_tick",      last_abort_tick_},
    };
}

void ChunkOutcomeGate::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1)
        throw std::runtime_error("ChunkOutcomeGate::restore_state: unknown version "
                                 + std::to_string(version));
    active_chunk_id_      = s.value("active_chunk_id",      active_chunk_id_);
    ticks_in_chunk_       = s.value("ticks_in_chunk",       ticks_in_chunk_);
    signal_at_start_      = s.value("signal_at_start",      signal_at_start_);
    current_signal_       = s.value("current_signal",       current_signal_);
    signal_seen_          = s.value("signal_seen",          signal_seen_);
    refractory_remaining_ = s.value("refractory_remaining", refractory_remaining_);
    aborts_total_         = s.value("aborts_total",         aborts_total_);
    last_abort_tick_      = s.value("last_abort_tick",      last_abort_tick_);
}

} // namespace ogma
