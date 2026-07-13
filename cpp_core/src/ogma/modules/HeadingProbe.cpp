// =============================================================================
// HeadingProbe.cpp  --  random-heading isolation source for the action layer
// =============================================================================
#include "ogma/modules/HeadingProbe.hpp"

#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
constexpr float kPi    = 3.14159265358979323846f;
constexpr float kInvPi = 1.0f / kPi;
constexpr float kTwoPi = 2.0f * kPi;

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("HeadingProbe param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("HeadingProbe param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("HeadingProbe param '" + key + "' must be string");
}
// wrap to (-pi, pi]
float wrap_pi(float a) {
    while (a >  kPi) a -= kTwoPi;
    while (a < -kPi) a += kTwoPi;
    return a;
}
}

HeadingProbe::HeadingProbe()  = default;
HeadingProbe::~HeadingProbe() = default;

std::string_view HeadingProbe::type_name() const { return "HeadingProbe"; }

std::vector<TopicSpec> HeadingProbe::input_topics() const {
    return { TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken))} };
}

std::vector<TopicSpec> HeadingProbe::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema HeadingProbe::params_schema() const {
    return {
        {"output_topic", ParamMutability::ConstructionOnly,
            "Egocentric desired-heading ProprioToken [cx=+right, cy=+forward] emitted to "
            "the HeadingController (same shape as percept.scent_compass).",
            ParamValue{std::string("percept.scent_compass")}},
        {"heading_topic", ParamMutability::ConstructionOnly,
            "Body heading ProprioToken (rad) used to convert the world target → egocentric.",
            ParamValue{std::string("reality.proprio.heading")}},
        {"hold_ticks", ParamMutability::HotMutable,
            "Ticks a random target is held before re-randomizing (long enough to align+charge).",
            ParamValue{int64_t{300}}},
        {"sign", ParamMutability::HotMutable,
            "Egocentric convention multiplier (+1/-1). Flip if the body SPINS instead of "
            "converging (the heading-sign loop is then positive feedback).", ParamValue{1.0}},
        {"master_seed", ParamMutability::ConstructionOnly,
            "Seed for the random target sequence (reproducible).", ParamValue{int64_t{1234}}},
    };
}

ParamMap HeadingProbe::current_params() const {
    ParamMap m;
    m["output_topic"]  = ParamValue{output_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["hold_ticks"]    = ParamValue{int64_t(hold_ticks_)};
    m["sign"]          = ParamValue{double(sign_)};
    m["master_seed"]   = ParamValue{int64_t(master_seed_)};
    return m;
}

void HeadingProbe::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("HeadingProbe requires a non-null Bus");

    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v, "output_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v, "heading_topic"); });
    apply_param(params, "hold_ticks",    [&](auto const& v){ hold_ticks_    = int(get_int(v, "hold_ticks")); });
    apply_param(params, "sign",          [&](auto const& v){ sign_          = float(get_double(v, "sign")); });
    apply_param(params, "master_seed",   [&](auto const& v){ master_seed_   = uint64_t(get_int(v, "master_seed")); });

    rng_.seed(master_seed_);

    if (!heading_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_heading(p); }));
}

void HeadingProbe::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "hold_ticks") hold_ticks_ = int(get_int(value, k));
    else if (k == "sign")       sign_       = float(get_double(value, k));
    else throw std::invalid_argument("HeadingProbe: param '" + k + "' is construction-only / unknown");
}

void HeadingProbe::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) heading_ = float(pt->values[0]);
}

void HeadingProbe::tick(uint64_t tick_id) {
    std::uniform_real_distribution<float> u(0.0f, kTwoPi);
    if (!started_ || ticks_held_ <= 0) {
        target_     = u(rng_);
        ticks_held_ = std::max(1, hold_ticks_);
        started_    = true;
    }
    // Egocentric bearing to the world target, scaled to [-1,1] (matches ScentCompass /
    // the HeadingController convention: bearing = atan2(cx,cy)*1/pi).
    float beta = sign_ * wrap_pi(target_ - heading_);   // radians
    float cx   = std::sin(beta);
    float cy   = std::cos(beta);
    last_bearing_ = beta * kInvPi;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id;
    out->producer_id = id_.empty() ? std::string("heading_probe") : id_;
    out->values = Eigen::VectorXf(2);
    out->values[0] = cx;   // +right
    out->values[1] = cy;   // +forward
    bus_->publish(output_topic_, out);

    --ticks_held_;
}

nlohmann::json HeadingProbe::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"target", target_}, {"ticks_held", ticks_held_}};
}

nlohmann::json HeadingProbe::diag_snapshot() const {
    return nlohmann::json{
        {"target_deg", target_ * 180.0f * kInvPi},
        {"bearing", last_bearing_},
        {"ticks_held", ticks_held_},
    };
}

void HeadingProbe::restore_state(nlohmann::json const& s) {
    if (s.contains("target"))     target_     = s.value("target", target_);
    if (s.contains("ticks_held")) ticks_held_ = s.value("ticks_held", ticks_held_);
    started_ = true;
}

} // namespace ogma
