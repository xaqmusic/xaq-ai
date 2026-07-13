// =============================================================================
// MotivationGate.cpp  --  homeostatic gate on foraging (forage because hungry)
// =============================================================================
#include "ogma/modules/MotivationGate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
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
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("MotivationGate param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("MotivationGate param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("MotivationGate param '" + key + "' must be string");
}
}

MotivationGate::MotivationGate()  = default;
MotivationGate::~MotivationGate() = default;

std::string_view MotivationGate::type_name() const { return "MotivationGate"; }

std::vector<TopicSpec> MotivationGate::input_topics() const {
    return { TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken))},
             TopicSpec{energy_topic_,  std::type_index(typeid(ProprioToken))} };
}

std::vector<TopicSpec> MotivationGate::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema MotivationGate::params_schema() const {
    return {
        {"heading_topic", ParamMutability::ConstructionOnly,
            "Food-bearing ProprioToken [cx,cy,(prox)] (e.g. percept.scent_compass).",
            ParamValue{std::string("percept.scent_compass")}},
        {"energy_topic", ParamMutability::ConstructionOnly,
            "Body energy ProprioToken (scalar [0,1]).", ParamValue{std::string("reality.proprio.energy")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Gated desired-heading (→ HeadingController.input_topic).",
            ParamValue{std::string("percept.motivated_heading")}},
        {"cx_index", ParamMutability::ConstructionOnly, "Index of +right component.", ParamValue{int64_t{0}}},
        {"cy_index", ParamMutability::ConstructionOnly, "Index of +forward component.", ParamValue{int64_t{1}}},
        {"sated_energy", ParamMutability::HotMutable,
            "Homeostatic setpoint: gain=0 at/above this energy (sated→idle), ramps to 1 at empty.",
            ParamValue{0.8}},
        {"freeze_gain", ParamMutability::HotMutable,
            "ABLATION: ≥0 overrides the hunger gain with a constant (1=always pursue, no "
            "need-modulation). <0 = use the homeostatic gain (default).", ParamValue{-1.0}},
    };
}

ParamMap MotivationGate::current_params() const {
    ParamMap m;
    m["heading_topic"] = ParamValue{heading_topic_};
    m["energy_topic"]  = ParamValue{energy_topic_};
    m["output_topic"]  = ParamValue{output_topic_};
    m["cx_index"]      = ParamValue{int64_t(cx_index_)};
    m["cy_index"]      = ParamValue{int64_t(cy_index_)};
    m["sated_energy"]  = ParamValue{double(sated_energy_)};
    m["freeze_gain"]   = ParamValue{double(freeze_gain_)};
    return m;
}

void MotivationGate::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("MotivationGate requires a non-null Bus");

    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v, "heading_topic"); });
    apply_param(params, "energy_topic",  [&](auto const& v){ energy_topic_  = get_string(v, "energy_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v, "output_topic"); });
    apply_param(params, "cx_index",      [&](auto const& v){ cx_index_ = int(get_int(v, "cx_index")); });
    apply_param(params, "cy_index",      [&](auto const& v){ cy_index_ = int(get_int(v, "cy_index")); });
    apply_param(params, "sated_energy",  [&](auto const& v){ sated_energy_ = float(get_double(v, "sated_energy")); });
    apply_param(params, "freeze_gain",   [&](auto const& v){ freeze_gain_  = float(get_double(v, "freeze_gain")); });

    sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
        [this](std::string_view /*topic*/, MessagePtr p){ handle_heading(p); }));
    if (!energy_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(energy_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_energy(p); }));
}

void MotivationGate::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "sated_energy") sated_energy_ = float(get_double(value, k));
    else if (k == "freeze_gain")  freeze_gain_  = float(get_double(value, k));
    else throw std::invalid_argument("MotivationGate: param '" + k + "' is construction-only / unknown");
}

void MotivationGate::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    n_in_ = int(pt->values.size());
    cx_ = (cx_index_ < n_in_) ? float(pt->values[cx_index_]) : 0.0f;
    cy_ = (cy_index_ < n_in_) ? float(pt->values[cy_index_]) : 0.0f;
    have_prox_ = (n_in_ > 2);
    if (have_prox_) prox_ = float(pt->values[2]);
}

void MotivationGate::handle_energy(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) latest_energy_ = float(pt->values[0]);
}

void MotivationGate::tick(uint64_t tick_id) {
    // Homeostatic gain ∝ hunger: 0 at/above setpoint (sated), ramps to 1 at empty.
    float g;
    if (freeze_gain_ >= 0.0f) {
        g = freeze_gain_;                                   // ablation: constant pursuit
    } else {
        float denom = std::max(1e-6f, sated_energy_);
        g = std::clamp((sated_energy_ - latest_energy_) / denom, 0.0f, 1.0f);
    }
    last_gain_ = g;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id;
    out->producer_id = id_.empty() ? std::string("motivation_gate") : id_;
    int n = std::max(2, n_in_);
    out->values = Eigen::VectorXf::Zero(n);
    out->values[cx_index_] = cx_ * g;       // scale MAGNITUDE (direction preserved)
    out->values[cy_index_] = cy_ * g;
    if (have_prox_ && n > 2) out->values[2] = prox_;   // passthrough (unscaled)
    bus_->publish(output_topic_, out);
}

nlohmann::json MotivationGate::snapshot_state() const {
    return nlohmann::json{{"version", 1}};
}

nlohmann::json MotivationGate::diag_snapshot() const {
    return nlohmann::json{{"gain", last_gain_}, {"energy", latest_energy_}};
}

void MotivationGate::restore_state(nlohmann::json const& /*s*/) {}

} // namespace ogma
