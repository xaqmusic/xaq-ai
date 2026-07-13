#include "ogma/modules/ChemotaxisAI.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

constexpr float kInvPi = 0.318309886183790671538f;   // 1/pi

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("ChemotaxisAI: param '" + key + "' must be numeric");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ChemotaxisAI: param '" + key + "' must be a string");
}

} // namespace

ChemotaxisAI::ChemotaxisAI()  = default;
ChemotaxisAI::~ChemotaxisAI() = default;

std::string_view ChemotaxisAI::type_name() const { return "ChemotaxisAI"; }

std::vector<TopicSpec> ChemotaxisAI::input_topics() const {
    return {
        TopicSpec{nav_topic_, std::type_index(typeid(ProprioToken)),
                  SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> ChemotaxisAI::output_topics() const {
    return {
        TopicSpec{output_topic_left_,  std::type_index(typeid(ActionOut))},
        TopicSpec{output_topic_right_, std::type_index(typeid(ActionOut))},
    };
}

ParamSchema ChemotaxisAI::params_schema() const {
    return {
        {"nav_topic", ParamMutability::ConstructionOnly,
            "2-D egocentric goal-bearing ProprioToken from the ScentCompass "
            "perception module (percept.scent_compass: RAW gradient vector toward "
            "the up-gradient direction, [x=+right, y=+forward]; magnitude = "
            "gradient strength). The cell analog of the picrawler's target_compass.",
            ParamValue{std::string("percept.scent_compass")}},
        {"nav_gain", ParamMutability::HotMutable,
            "PERCEPTION→STEERING: steer-toward-gradient strength. Reads the "
            "egocentric scent bearing and steers to drive its lateral component "
            "to zero (gradient dead-ahead) — the active-inference closure. "
            "0 = off (open loop); sign tunable (>0 steers toward the gradient).",
            ParamValue{0.8}},
        {"base_thrust", ParamMutability::HotMutable,
            "Forward thrust fraction (rate units, ×4 → the body's accel∈[-4,4] "
            "per-flagellum frame). 1.0 = full forward drive when the gradient is "
            "dead-ahead; eased off when the source is off-axis so the cell pivots.",
            ParamValue{1.0}},
        {"min_signal", ParamMutability::HotMutable,
            "Raw |gradient| below this → no confident bearing → the cell drives "
            "straight (punching through a nutrient at its saturated peak, or "
            "exploring when far) instead of chasing a noisy bearing and "
            "spin-orbiting. The morphology's edge-of-signal gate.",
            ParamValue{0.06}},
        {"output_topic_left", ParamMutability::ConstructionOnly,
            "Left-flagellum ActionOut topic (default action.left).",
            ParamValue{std::string(topics::kActionLeft)}},
        {"output_topic_right", ParamMutability::ConstructionOnly,
            "Right-flagellum ActionOut topic (default action.right).",
            ParamValue{std::string(topics::kActionRight)}},
    };
}

ParamMap ChemotaxisAI::current_params() const {
    ParamMap m;
    m["nav_topic"]          = ParamValue{nav_topic_};
    m["nav_gain"]           = ParamValue{double(nav_gain_)};
    m["base_thrust"]        = ParamValue{double(base_thrust_)};
    m["min_signal"]         = ParamValue{double(min_signal_)};
    m["output_topic_left"]  = ParamValue{output_topic_left_};
    m["output_topic_right"] = ParamValue{output_topic_right_};
    return m;
}

void ChemotaxisAI::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ChemotaxisAI requires a non-null Bus");

    apply_param(params, "nav_topic",
        [&](auto const& v){ nav_topic_ = get_string(v, "nav_topic"); });
    apply_param(params, "nav_gain",
        [&](auto const& v){ nav_gain_ = float(get_double(v, "nav_gain")); });
    apply_param(params, "base_thrust",
        [&](auto const& v){ base_thrust_ = float(get_double(v, "base_thrust")); });
    apply_param(params, "min_signal",
        [&](auto const& v){ min_signal_ = float(get_double(v, "min_signal")); });
    apply_param(params, "output_topic_left",
        [&](auto const& v){ output_topic_left_  = get_string(v, "output_topic_left"); });
    apply_param(params, "output_topic_right",
        [&](auto const& v){ output_topic_right_ = get_string(v, "output_topic_right"); });

    if (!nav_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(nav_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_compass(p); }));
    }
}

void ChemotaxisAI::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "nav_gain")         nav_gain_    = float(get_double(value, k));
    else if (k == "base_thrust") base_thrust_ = float(get_double(value, k));
    else if (k == "min_signal")  min_signal_  = float(get_double(value, k));
    else throw std::invalid_argument("ChemotaxisAI: unknown/non-mutable param '" + k + "'");
}

void ChemotaxisAI::handle_compass(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 2) return;
    compass_x_ = float(pt->values[0]);   // lateral, + = gradient to the right
    compass_y_ = float(pt->values[1]);   // forward, + = gradient ahead
}

void ChemotaxisAI::tick(uint64_t tick_id) {
    // The compass arrives RAW: its direction is the up-gradient bearing, its
    // magnitude is the gradient strength (the cell's confidence in that
    // direction).  Steer only when the gradient is confident; otherwise drive
    // straight — which punches THROUGH a nutrient near its saturated peak (where
    // the gradient flattens) and explores when far, instead of chasing a noisy
    // bearing and spin-orbiting.
    float cx = compass_x_;
    float cy = compass_y_;
    float raw_mag = std::sqrt(cx * cx + cy * cy);
    bool nav_on = (raw_mag > min_signal_) && (nav_gain_ != 0.0f);

    float fwd_frac;
    float steer;
    if (nav_on) {
        // Confident gradient → the picrawler active-inference law: steer to null
        // the lateral bearing, and ease off forward when the source is off-axis
        // so the cell can pivot toward it.
        float ux = cx / raw_mag;
        float uy = cy / raw_mag;
        float bearing = std::atan2(ux, uy) * kInvPi;            // [-1, 1]
        steer    = nav_gain_ * bearing;                         // rate units
        fwd_frac = base_thrust_ * std::clamp(0.25f + 0.75f * uy, 0.0f, 1.0f);
    } else {
        // Weak/flat gradient (near a nutrient's saturated peak, or far away) →
        // drive straight: punch THROUGH the food, or explore.  This gate is the
        // morphology accommodation — the differential_paddler only translates
        // when both flagella spike, so chasing a noisy near-peak bearing would
        // pivot it in place and orbit the food forever (never satisfying the
        // forward-motion hit condition).
        steer    = 0.0f;
        fwd_frac = base_thrust_;
    }

    // Skid-steer differential → per-flagellum thrust in the body's accel∈[-4,4]
    // frame (the body clamps al/4, ar/4 into the [0,1] spike-rate range; a
    // negative side simply yields a zero rate → pure rotation on that side).
    float al = std::clamp((fwd_frac + steer) * 4.0f, -4.0f, 4.0f);
    float ar = std::clamp((fwd_frac - steer) * 4.0f, -4.0f, 4.0f);

    auto act_l = std::make_shared<ActionOut>();
    act_l->tick_id     = tick_id;
    act_l->producer_id = id_.empty() ? std::string("chemotaxis_ai") : id_;
    act_l->accel       = al;
    act_l->source      = "chemotaxis_ai";
    act_l->probe       = false;
    act_l->action_tle  = 0.0f;
    bus_->publish(output_topic_left_, act_l);

    auto act_r = std::make_shared<ActionOut>();
    act_r->tick_id     = tick_id;
    act_r->producer_id = act_l->producer_id;
    act_r->accel       = ar;
    act_r->source      = "chemotaxis_ai";
    act_r->probe       = false;
    act_r->action_tle  = 0.0f;
    bus_->publish(output_topic_right_, act_r);

    last_nav_on_ = nav_on;
    last_al_     = al;
    last_ar_     = ar;
    ++tick_count_;
}

nlohmann::json ChemotaxisAI::snapshot_state() const {
    // Stateless controller (the only "state" is the last compass, refreshed
    // every tick by the bus) — record the diagnostics for parity with siblings.
    return nlohmann::json{
        {"version",     1},
        {"tick_count",  tick_count_},
        {"compass_x",   compass_x_},
        {"compass_y",   compass_y_},
    };
}

void ChemotaxisAI::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("ChemotaxisAI::restore_state: unknown version " +
                                 std::to_string(version));
    }
    tick_count_ = s.value("tick_count", tick_count_);
    compass_x_  = s.value("compass_x",  compass_x_);
    compass_y_  = s.value("compass_y",  compass_y_);
}

} // namespace ogma
