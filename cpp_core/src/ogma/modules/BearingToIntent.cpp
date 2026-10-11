// =============================================================================
// BearingToIntent.cpp  --  see the header
// =============================================================================
#include "ogma/modules/BearingToIntent.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("BearingToIntent: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("BearingToIntent: param '" + k + "' must be a string");
}
double wrap_pi(double a) {
    const double tp = 2.0 * 3.14159265358979323846;
    a = std::fmod(a + 3.14159265358979323846, tp);
    if (a < 0.0) a += tp;
    return a - 3.14159265358979323846;
}
} // namespace

BearingToIntent::BearingToIntent()  = default;
BearingToIntent::~BearingToIntent() = default;

std::string_view BearingToIntent::type_name() const { return "BearingToIntent"; }

std::vector<TopicSpec> BearingToIntent::input_topics() const {
    return { TopicSpec{bearing_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{odom_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}
std::vector<TopicSpec> BearingToIntent::output_topics() const {
    return { TopicSpec{goal_topic_,  std::type_index(typeid(ProprioToken))},
             TopicSpec{speed_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema BearingToIntent::params_schema() const {
    return {
        {"bearing_topic", ParamMutability::ConstructionOnly, "ProprioToken [cx, cy, value]: the loop's egocentric bearing, cx = body +X, cy = forward; zero = silent.", ParamValue{std::string("percept.avoid_bearing")}},
        {"odom_topic",    ParamMutability::ConstructionOnly, "ProprioToken [x, y, yaw]: the body's dead-reckoned pose; yaw is the heading the reference is kept in.", ParamValue{std::string("reality.proprio.odom")}},
        {"goal_topic",    ParamMutability::ConstructionOnly, "Output: [vx, vy] for MotorEPMv2 goal_bearing_topic.", ParamValue{std::string("reality.proprio.goal_bearing")}},
        {"speed_topic",   ParamMutability::ConstructionOnly, "Output: [s] for MotorEPMv2 speed_topic.", ParamValue{std::string("reality.proprio.speed_cmd")}},
        {"floor",          ParamMutability::HotMutable, "|bearing| below this = the loop is silent; the goal then points at the held reference.", ParamValue{0.05}},
        {"speed_max",      ParamMutability::HotMutable, "The speed with the goal dead ahead.", ParamValue{1.0}},
        {"speed_pow",      ParamMutability::HotMutable, "s = speed_max * sgn(cos e) * |cos e|^speed_pow. 0 = no gating.", ParamValue{0.0}},
        {"speed_min",      ParamMutability::HotMutable, "Floor on s while the goal is ahead (|e| < 90 deg).", ParamValue{0.0}},
        {"reverse_behind", ParamMutability::HotMutable, "cos e < 0: true = back up at the gated magnitude (the duck backs while it turns); false = hold speed_min.", ParamValue{false}},
        {"ref_tau_s",      ParamMutability::HotMutable, "> 0: while the loop is silent the reference leaks toward the current heading with this time constant (s). 0 = a hard hold.", ParamValue{0.0}},
        {"hz",             ParamMutability::ConstructionOnly, "Tick rate, for ref_tau_s.", ParamValue{50.0}},
    };
}

ParamMap BearingToIntent::current_params() const {
    ParamMap m;
    m["bearing_topic"] = ParamValue{bearing_topic_}; m["odom_topic"] = ParamValue{odom_topic_};
    m["goal_topic"] = ParamValue{goal_topic_}; m["speed_topic"] = ParamValue{speed_topic_};
    m["floor"] = ParamValue{floor_}; m["speed_max"] = ParamValue{speed_max_}; m["speed_pow"] = ParamValue{speed_pow_};
    m["speed_min"] = ParamValue{speed_min_}; m["reverse_behind"] = ParamValue{reverse_behind_};
    m["ref_tau_s"] = ParamValue{ref_tau_s_}; m["hz"] = ParamValue{hz_};
    return m;
}

void BearingToIntent::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("BearingToIntent requires a non-null Bus");
    apply_param(params, "bearing_topic", [&](auto const& v){ bearing_topic_ = get_string(v, "bearing_topic"); });
    apply_param(params, "odom_topic",    [&](auto const& v){ odom_topic_    = get_string(v, "odom_topic"); });
    apply_param(params, "goal_topic",    [&](auto const& v){ goal_topic_    = get_string(v, "goal_topic"); });
    apply_param(params, "speed_topic",   [&](auto const& v){ speed_topic_   = get_string(v, "speed_topic"); });
    apply_param(params, "floor",      [&](auto const& v){ floor_     = get_double(v, "floor"); });
    apply_param(params, "speed_max",  [&](auto const& v){ speed_max_ = get_double(v, "speed_max"); });
    apply_param(params, "speed_pow",  [&](auto const& v){ speed_pow_ = get_double(v, "speed_pow"); });
    apply_param(params, "speed_min",  [&](auto const& v){ speed_min_ = get_double(v, "speed_min"); });
    apply_param(params, "reverse_behind", [&](auto const& v){ if (auto b = std::get_if<bool>(&v)) reverse_behind_ = *b; });
    apply_param(params, "ref_tau_s",  [&](auto const& v){ ref_tau_s_ = get_double(v, "ref_tau_s"); });
    apply_param(params, "hz",         [&](auto const& v){ hz_        = get_double(v, "hz"); });
    sub_ids_.push_back(bus_->subscribe(bearing_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_bearing(p); }));
    sub_ids_.push_back(bus_->subscribe(odom_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_odom(p); }));
}

void BearingToIntent::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "floor")      floor_     = get_double(value, k);
    else if (k == "speed_max")  speed_max_ = get_double(value, k);
    else if (k == "speed_pow")  speed_pow_ = get_double(value, k);
    else if (k == "speed_min")  speed_min_ = get_double(value, k);
    else if (k == "ref_tau_s")  ref_tau_s_ = get_double(value, k);
    else if (k == "reverse_behind") { if (auto b = std::get_if<bool>(&value)) reverse_behind_ = *b; }
    else throw std::invalid_argument("BearingToIntent: param '" + k + "' is construction-only / unknown");
}

void BearingToIntent::handle_bearing(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 2) return;
    cx_ = double(pt->values[0]); cy_ = double(pt->values[1]);
    val_ = pt->values.size() > 2 ? double(pt->values[2]) : std::sqrt(cx_ * cx_ + cy_ * cy_);
    have_bearing_ = true; ++msgs_;
}

void BearingToIntent::handle_odom(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 3) return;
    yaw_ = double(pt->values[2]); have_odom_ = true;
}

void BearingToIntent::tick(uint64_t tick_id) {
    if (!have_odom_) return;                       // no heading, no reference, nothing said
    if (!ref_set_) { ref_yaw_ = yaw_; ref_set_ = true; }
    const double mag = std::sqrt(cx_ * cx_ + cy_ * cy_);
    double e;                                      // the bearing error we publish, rad, + = toward body +X
    if (have_bearing_ && mag >= floor_) {
        active_ = true;
        e = std::atan2(cx_, cy_);
        ref_yaw_ = yaw_ + e;                       // the loop sets the reference
    } else {
        active_ = false;
        if (ref_tau_s_ > 0.0) ref_yaw_ += (yaw_ - ref_yaw_) * std::min(1.0, 1.0 / (ref_tau_s_ * hz_));
        e = wrap_pi(ref_yaw_ - yaw_);              // the hold: the bearing TO the reference
    }
    vx_ = float(std::sin(e)); vy_ = float(std::cos(e)); err_ = float(e);
    // Speed sheds with the turn.
    const double ce = std::cos(e);
    double s;
    if (speed_pow_ <= 0.0) s = speed_max_;
    else {
        const double g = std::pow(std::fabs(ce), speed_pow_);
        if (ce >= 0.0) s = std::max(speed_min_, speed_max_ * g);
        else           s = reverse_behind_ ? -speed_max_ * g : speed_min_;
    }
    speed_ = float(std::clamp(s, -1.0, 1.0));

    auto g = std::make_shared<ProprioToken>();
    g->tick_id = tick_id; g->producer_id = id_.empty() ? std::string("intent") : id_; g->sensor = "goal_bearing";
    g->values = Eigen::VectorXf(2); g->values[0] = vx_; g->values[1] = vy_;
    bus_->publish(goal_topic_, g);
    auto sp = std::make_shared<ProprioToken>();
    sp->tick_id = tick_id; sp->producer_id = g->producer_id; sp->sensor = "speed_cmd";
    sp->values = Eigen::VectorXf::Constant(1, speed_);
    bus_->publish(speed_topic_, sp);
}

nlohmann::json BearingToIntent::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"ref_yaw", ref_yaw_}, {"ref_set", ref_set_}, {"msgs", msgs_}};
}
void BearingToIntent::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    ref_yaw_ = s.value("ref_yaw", 0.0); ref_set_ = s.value("ref_set", false); msgs_ = s.value("msgs", int64_t{0});
}
nlohmann::json BearingToIntent::diag_snapshot() const {
    return nlohmann::json{{"active", active_}, {"err", err_}, {"vx", vx_}, {"vy", vy_}, {"speed", speed_},
                          {"ref_yaw", ref_yaw_}, {"yaw", yaw_}, {"value", val_}, {"msgs", msgs_}};
}

} // namespace ogma
