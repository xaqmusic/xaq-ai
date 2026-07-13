#include "ogma/modules/ChunkAbortGate.hpp"

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
    throw std::invalid_argument("ChunkAbortGate: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("ChunkAbortGate: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ChunkAbortGate: param '" + key + "' must be string");
}

} // namespace

ChunkAbortGate::ChunkAbortGate()  = default;
ChunkAbortGate::~ChunkAbortGate() = default;

std::string_view ChunkAbortGate::type_name() const { return "ChunkAbortGate"; }

std::vector<TopicSpec> ChunkAbortGate::input_topics() const {
    return {
        TopicSpec{consensus_topic_, std::type_index(typeid(ConsensusToken)),
                  SubscriptionKind::Direct, /*required=*/true},
    };
}

std::vector<TopicSpec> ChunkAbortGate::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(EnvEvent))} };
}

ParamSchema ChunkAbortGate::params_schema() const {
    return {
        {"consensus_topic",       ParamMutability::ConstructionOnly,
            "ConsensusToken topic to monitor for surprise_ema (default consensus.0)",
            ParamValue{std::string("consensus.0")}},
        {"output_event_name",     ParamMutability::ConstructionOnly,
            "Event suffix; published as events.<this>",
            ParamValue{std::string("chunk_abort")}},
        {"baseline_alpha",        ParamMutability::HotMutable,
            "EMA rate for baseline mean+var of mean per-modality surprise (default 0.005 = ~200-tick window)",
            ParamValue{0.005}},
        {"k_sigma",               ParamMutability::HotMutable,
            "Surprise spike threshold = baseline_mean + k * sqrt(baseline_var).  0 = disabled (never fires).  Default 2.0.",
            ParamValue{2.0}},
        {"min_consecutive_ticks", ParamMutability::HotMutable,
            "Number of consecutive above-threshold ticks required before firing (default 5)",
            ParamValue{int64_t{5}}},
        {"refractory_ticks",      ParamMutability::HotMutable,
            "Suppress further aborts for N ticks after one fires (default 60)",
            ParamValue{int64_t{60}}},
    };
}

ParamMap ChunkAbortGate::current_params() const {
    ParamMap m;
    m["consensus_topic"]       = ParamValue{consensus_topic_};
    m["output_event_name"]     = ParamValue{output_event_name_};
    m["baseline_alpha"]        = ParamValue{double(baseline_alpha_)};
    m["k_sigma"]               = ParamValue{double(k_sigma_)};
    m["min_consecutive_ticks"] = ParamValue{int64_t(min_consecutive_ticks_)};
    m["refractory_ticks"]      = ParamValue{int64_t(refractory_ticks_)};
    return m;
}

void ChunkAbortGate::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ChunkAbortGate requires a non-null Bus");

    apply_param(params, "consensus_topic",
        [&](auto const& v){ consensus_topic_ = get_string(v, "consensus_topic"); });
    apply_param(params, "output_event_name",
        [&](auto const& v){ output_event_name_ = get_string(v, "output_event_name"); });
    apply_param(params, "baseline_alpha",
        [&](auto const& v){ baseline_alpha_ = float(get_double(v, "baseline_alpha")); });
    apply_param(params, "k_sigma",
        [&](auto const& v){ k_sigma_ = float(get_double(v, "k_sigma")); });
    apply_param(params, "min_consecutive_ticks",
        [&](auto const& v){ min_consecutive_ticks_ = std::max(1, int(get_int(v, "min_consecutive_ticks"))); });
    apply_param(params, "refractory_ticks",
        [&](auto const& v){ refractory_ticks_ = std::max(0, int(get_int(v, "refractory_ticks"))); });

    if (output_event_name_.empty())
        throw std::invalid_argument("ChunkAbortGate: output_event_name must not be empty");
    output_topic_ = std::string("events.") + output_event_name_;

    sub_ids_.push_back(bus_->subscribe(consensus_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_consensus(p); }));
}

void ChunkAbortGate::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "baseline_alpha")         baseline_alpha_        = float(get_double(value, k));
    else if (k == "k_sigma")                k_sigma_               = float(get_double(value, k));
    else if (k == "min_consecutive_ticks")  min_consecutive_ticks_ = std::max(1, int(get_int(value, k)));
    else if (k == "refractory_ticks")       refractory_ticks_      = std::max(0, int(get_int(value, k)));
    else throw std::invalid_argument("ChunkAbortGate: unknown/non-mutable param '" + k + "'");
}

void ChunkAbortGate::handle_consensus(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (!ct) return;

    // Aggregate per-modality surprise into a single scalar (mean across
    // modalities present in the surprise_ema map).  Empty map → 0 (no
    // surprise reporting yet; baseline doesn't update this tick).
    if (ct->surprise_ema.empty()) return;
    double sum = 0.0;
    int    n   = 0;
    for (auto const& [k, v] : ct->surprise_ema) { (void)k; sum += double(v); ++n; }
    last_surprise_ = float(sum / std::max(1, n));

    // EMA mean + variance (Welford-flavored on EMA).
    if (!baseline_init_) {
        baseline_mean_ = last_surprise_;
        baseline_var_  = 0.0f;
        baseline_init_ = true;
        return;
    }
    float delta = last_surprise_ - baseline_mean_;
    baseline_mean_ += baseline_alpha_ * delta;
    // Variance EMA tracks squared deviation from running mean.
    baseline_var_ += baseline_alpha_ * (delta * delta - baseline_var_);
}

void ChunkAbortGate::tick(uint64_t tick_id) {
    if (refractory_remaining_ > 0) --refractory_remaining_;
    if (k_sigma_ <= 0.0f || !baseline_init_ || refractory_remaining_ > 0) {
        above_thresh_streak_ = 0;
        return;
    }
    float thresh = baseline_mean_ + k_sigma_ * std::sqrt(std::max(0.0f, baseline_var_));
    if (last_surprise_ > thresh) {
        ++above_thresh_streak_;
    } else {
        above_thresh_streak_ = 0;
        return;
    }
    if (above_thresh_streak_ >= min_consecutive_ticks_) {
        auto e = std::make_shared<EnvEvent>();
        e->tick_id     = tick_id;
        e->producer_id = id_.empty() ? std::string("chunk_abort_gate") : id_;
        e->name        = output_event_name_;
        e->intensity   = last_surprise_;
        bus_->publish(output_topic_, e);
        ++aborts_total_;
        last_abort_tick_      = int64_t(tick_id);
        above_thresh_streak_  = 0;
        refractory_remaining_ = refractory_ticks_;
    }
}

nlohmann::json ChunkAbortGate::snapshot_state() const {
    return nlohmann::json{
        {"version",              1},
        {"baseline_init",        baseline_init_},
        {"baseline_mean",        baseline_mean_},
        {"baseline_var",         baseline_var_},
        {"last_surprise",        last_surprise_},
        {"above_thresh_streak",  above_thresh_streak_},
        {"refractory_remaining", refractory_remaining_},
        {"aborts_total",         aborts_total_},
        {"last_abort_tick",      last_abort_tick_},
    };
}

void ChunkAbortGate::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("ChunkAbortGate::restore_state: unknown version "
                                 + std::to_string(version));
    }
    baseline_init_       = s.value("baseline_init",       baseline_init_);
    baseline_mean_       = s.value("baseline_mean",       baseline_mean_);
    baseline_var_        = s.value("baseline_var",        baseline_var_);
    last_surprise_       = s.value("last_surprise",       last_surprise_);
    above_thresh_streak_ = s.value("above_thresh_streak", above_thresh_streak_);
    refractory_remaining_= s.value("refractory_remaining",refractory_remaining_);
    aborts_total_        = s.value("aborts_total",        aborts_total_);
    last_abort_tick_     = s.value("last_abort_tick",     last_abort_tick_);
}

} // namespace ogma
