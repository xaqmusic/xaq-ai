// =============================================================================
// StuckEscape.cpp  --  see the header
// =============================================================================
#include "ogma/modules/StuckEscape.hpp"

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
    throw std::invalid_argument("StuckEscape: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("StuckEscape: param '" + k + "' must be a string");
}
constexpr double kDeg = 3.14159265358979323846 / 180.0;
double wrap_pi(double a) {
    const double tp = 2.0 * 3.14159265358979323846;
    a = std::fmod(a + 3.14159265358979323846, tp);
    if (a < 0.0) a += tp;
    return a - 3.14159265358979323846;
}
} // namespace

StuckEscape::StuckEscape()  = default;
StuckEscape::~StuckEscape() = default;

std::string_view StuckEscape::type_name() const { return "StuckEscape"; }

std::vector<TopicSpec> StuckEscape::input_topics() const {
    return { TopicSpec{stuck_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{view_topic_,  std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{odom_topic_,  std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}
std::vector<TopicSpec> StuckEscape::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema StuckEscape::params_schema() const {
    return {
        {"stuck_topic",  ParamMutability::ConstructionOnly, "ProprioToken [score, ...]: StuckSense's output.", ParamValue{std::string("reality.proprio.stuck")}},
        {"view_topic",   ParamMutability::ConstructionOnly, "ProprioToken [8]: ProxMemory's view, sector 0 = the -X side, 1 = nothing.", ParamValue{std::string("reality.proprio.prox_view")}},
        {"odom_topic",   ParamMutability::ConstructionOnly, "ProprioToken [x, y, yaw]: the heading the escape turn is held in.", ParamValue{std::string("reality.proprio.odom")}},
        {"output_topic", ParamMutability::ConstructionOnly, "Output: [cx, cy, value, speed] for BearingToIntent's override_topic; value 0 = released.", ParamValue{std::string("percept.escape")}},
        {"on",            ParamMutability::HotMutable, "Stuck score at or above this, held for confirm_ticks, triggers an escape.", ParamValue{0.7}},
        {"confirm_ticks", ParamMutability::HotMutable, "A leaky count: +1 per tick at or above `on`, -1 below, floored at 0; an escape triggers when it reaches this.", ParamValue{int64_t{50}}},
        {"back_ticks",    ParamMutability::HotMutable, "Ticks of backing.", ParamValue{int64_t{75}}},
        {"back_speed",    ParamMutability::HotMutable, "Backing speed (published negative).", ParamValue{0.5}},
        {"turn_ticks",    ParamMutability::HotMutable, "Ticks the escape heading is held.", ParamValue{int64_t{100}}},
        {"turn_speed",    ParamMutability::HotMutable, "Forward speed while turning to the escape heading.", ParamValue{0.5}},
        {"refract_ticks", ParamMutability::HotMutable, "Ticks after release during which no new escape can trigger.", ParamValue{int64_t{150}}},
        {"fallback_turn_deg", ParamMutability::HotMutable, "When the view has nothing to say (all sectors free or all equally blocked), turn this much, alternating sides per episode.", ParamValue{90.0}},
        {"view_free",     ParamMutability::HotMutable, "A view sector at or above this is 'nothing remembered'.", ParamValue{0.9}},
    };
}

ParamMap StuckEscape::current_params() const {
    ParamMap m;
    m["stuck_topic"] = ParamValue{stuck_topic_}; m["view_topic"] = ParamValue{view_topic_};
    m["odom_topic"] = ParamValue{odom_topic_}; m["output_topic"] = ParamValue{output_topic_};
    m["on"] = ParamValue{on_}; m["confirm_ticks"] = ParamValue{int64_t(confirm_ticks_)};
    m["back_ticks"] = ParamValue{int64_t(back_ticks_)}; m["back_speed"] = ParamValue{back_speed_};
    m["turn_ticks"] = ParamValue{int64_t(turn_ticks_)}; m["turn_speed"] = ParamValue{turn_speed_};
    m["refract_ticks"] = ParamValue{int64_t(refract_ticks_)}; m["fallback_turn_deg"] = ParamValue{fallback_turn_deg_};
    m["view_free"] = ParamValue{view_free_};
    return m;
}

void StuckEscape::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("StuckEscape requires a non-null Bus");
    apply_param(params, "stuck_topic",  [&](auto const& v){ stuck_topic_  = get_string(v, "stuck_topic"); });
    apply_param(params, "view_topic",   [&](auto const& v){ view_topic_   = get_string(v, "view_topic"); });
    apply_param(params, "odom_topic",   [&](auto const& v){ odom_topic_   = get_string(v, "odom_topic"); });
    apply_param(params, "output_topic", [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "on",            [&](auto const& v){ on_            = get_double(v, "on"); });
    apply_param(params, "confirm_ticks", [&](auto const& v){ confirm_ticks_ = int(get_double(v, "confirm_ticks")); });
    apply_param(params, "back_ticks",    [&](auto const& v){ back_ticks_    = int(get_double(v, "back_ticks")); });
    apply_param(params, "back_speed",    [&](auto const& v){ back_speed_    = get_double(v, "back_speed"); });
    apply_param(params, "turn_ticks",    [&](auto const& v){ turn_ticks_    = int(get_double(v, "turn_ticks")); });
    apply_param(params, "turn_speed",    [&](auto const& v){ turn_speed_    = get_double(v, "turn_speed"); });
    apply_param(params, "refract_ticks", [&](auto const& v){ refract_ticks_ = int(get_double(v, "refract_ticks")); });
    apply_param(params, "fallback_turn_deg", [&](auto const& v){ fallback_turn_deg_ = get_double(v, "fallback_turn_deg"); });
    apply_param(params, "view_free",     [&](auto const& v){ view_free_     = get_double(v, "view_free"); });
    sub_ids_.push_back(bus_->subscribe(stuck_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle_stuck(p); }));
    sub_ids_.push_back(bus_->subscribe(view_topic_,  SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle_view(p); }));
    sub_ids_.push_back(bus_->subscribe(odom_topic_,  SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle_odom(p); }));
}

void StuckEscape::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "on")            on_            = get_double(value, k);
    else if (k == "confirm_ticks") confirm_ticks_ = int(get_double(value, k));
    else if (k == "back_ticks")    back_ticks_    = int(get_double(value, k));
    else if (k == "back_speed")    back_speed_    = get_double(value, k);
    else if (k == "turn_ticks")    turn_ticks_    = int(get_double(value, k));
    else if (k == "turn_speed")    turn_speed_    = get_double(value, k);
    else if (k == "refract_ticks") refract_ticks_ = int(get_double(value, k));
    else if (k == "fallback_turn_deg") fallback_turn_deg_ = get_double(value, k);
    else if (k == "view_free")     view_free_     = get_double(value, k);
    else throw std::invalid_argument("StuckEscape: param '" + k + "' is construction-only / unknown");
}

void StuckEscape::handle_stuck(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 1) return;
    score_ = double(pt->values[0]);
}
void StuckEscape::handle_view(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 8) return;
    for (int k = 0; k < 8; ++k) view_[k] = float(pt->values[k]);
    have_view_ = true;
}
void StuckEscape::handle_odom(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 3) return;
    yaw_ = double(pt->values[2]); have_odom_ = true;
}

void StuckEscape::tick(uint64_t tick_id) {
    switch (state_) {
    case State::Idle:
        // A leaky count, not a consecutive run: the stall score FLICKERS on a body sliding
        // along a wall it cannot see (measured, S3b seed 2), and a run that resets on every
        // dip never confirms.  Up one while above `on`, down one while below, floored at 0.
        above_ = std::max(0, above_ + ((score_ >= on_) ? 1 : -1));
        if (above_ >= confirm_ticks_) {
            // Choose the escape heading NOW, from the view: the freest sector, ties and an empty
            // view -> the fallback turn, alternating sides.  Sector k spans [-60 + 15k, -45 + 15k).
            int best = -1; float bestv = -1.0f; bool any_info = false;
            for (int k = 0; k < 8; ++k) {
                if (view_[k] < view_free_) any_info = true;
                if (view_[k] > bestv + 1e-6f) { bestv = view_[k]; best = k; }
            }
            if (have_view_ && any_info && best >= 0 && bestv >= view_free_) {
                turn_rad_ = (-60.0 + 15.0 * best + 7.5) * kDeg; chosen_sector_ = best;
            } else {
                turn_rad_ = fallback_sign_ * fallback_turn_deg_ * kDeg; fallback_sign_ = -fallback_sign_; chosen_sector_ = -1;
            }
            target_yaw_ = yaw_ + turn_rad_;
            state_ = State::Back; counter_ = 0; above_ = 0; ++episodes_;
        }
        break;
    case State::Back:
        if (++counter_ >= back_ticks_) { state_ = State::Turn; counter_ = 0; }
        break;
    case State::Turn:
        if (++counter_ >= turn_ticks_) { state_ = State::Refract; counter_ = 0; }
        break;
    case State::Refract:
        if (++counter_ >= refract_ticks_) { state_ = State::Idle; counter_ = 0; above_ = 0; }
        break;
    }
    switch (state_) {
    case State::Back:  out_cx_ = 0.0f; out_cy_ = 1.0f; out_val_ = 1.0f; out_speed_ = float(-back_speed_); break;
    case State::Turn: {
        const double e = have_odom_ ? wrap_pi(target_yaw_ - yaw_) : turn_rad_;
        out_cx_ = float(std::sin(e)); out_cy_ = float(std::cos(e)); out_val_ = 1.0f; out_speed_ = float(turn_speed_);
        break; }
    default: out_cx_ = 0.0f; out_cy_ = 0.0f; out_val_ = 0.0f; out_speed_ = 0.0f; break;
    }
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("escape") : id_; out->sensor = "escape";
    out->values = Eigen::VectorXf(4);
    out->values[0] = out_cx_; out->values[1] = out_cy_; out->values[2] = out_val_; out->values[3] = out_speed_;
    bus_->publish(output_topic_, out);
}

nlohmann::json StuckEscape::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"episodes", episodes_}, {"fallback_sign", fallback_sign_}};
}
void StuckEscape::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    episodes_ = s.value("episodes", int64_t{0}); fallback_sign_ = s.value("fallback_sign", 1);
}
nlohmann::json StuckEscape::diag_snapshot() const {
    return nlohmann::json{{"state", int(state_)}, {"score", score_}, {"episodes", episodes_}, {"counter", counter_},
                          {"turn_deg", turn_rad_ / kDeg}, {"sector", chosen_sector_}, {"speed", out_speed_}, {"value", out_val_}};
}

} // namespace ogma
