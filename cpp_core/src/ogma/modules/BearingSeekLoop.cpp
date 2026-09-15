// =============================================================================
// BearingSeekLoop.cpp  --  see the header
// =============================================================================
#include "ogma/modules/BearingSeekLoop.hpp"

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
    throw std::invalid_argument("BearingSeekLoop: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("BearingSeekLoop: param '" + k + "' must be a string");
}
} // namespace

BearingSeekLoop::BearingSeekLoop()  = default;
BearingSeekLoop::~BearingSeekLoop() = default;

std::string_view BearingSeekLoop::type_name() const { return "BearingSeekLoop"; }

std::vector<TopicSpec> BearingSeekLoop::input_topics() const {
    return { TopicSpec{bearing_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{pose_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}
std::vector<TopicSpec> BearingSeekLoop::output_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))},
                              TopicSpec{value_topic_,  std::type_index(typeid(ProprioToken))} };
    if (!range_topic_.empty()) v.push_back(TopicSpec{range_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema BearingSeekLoop::params_schema() const {
    return {
        {"bearing_topic", ParamMutability::ConstructionOnly,
            "ProprioToken [vx=+right, vy=+forward, proximity] to the attended thing (CloudMap thing_bearing_topic); proximity 0 = nothing seen.",
            ParamValue{std::string("percept.thing_bearing")}},
        {"pose_topic", ParamMutability::ConstructionOnly,
            "ProprioToken [x, y, yaw]: the body's own dead-reckoned pose, the frame the remembered position lives in.",
            ParamValue{std::string("reality.proprio.odom")}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Bearing to the thing (live, or remembered and re-aimed) [cx=+right, cy=+forward, 0]; zero when none.",
            ParamValue{std::string("percept.seek_bearing")}},
        {"value_topic", ParamMutability::ConstructionOnly,
            "The loop's need in [0,1]: confidence in the held target (1 seen, decaying remembered, 0 none) -- the arbiter's preference.",
            ParamValue{std::string("reality.cognitive.seek_value")}},
        {"range_topic", ParamMutability::ConstructionOnly,
            "The loop's honest signal for LoopCompetence (sign -1): the remaining range to the target, metres; the last value when none. Empty = not published.",
            ParamValue{std::string("reality.cognitive.seek_range")}},
        {"proximity_range", ParamMutability::HotMutable,
            "The range the bearing's proximity is scaled by: proximity = 1 - range / this (CloudMap's things_range, else its max_range).",
            ParamValue{2.5}, ParamValue{0.1}, ParamValue{10.0}},
        {"min_conf", ParamMutability::HotMutable,
            "Proximity above which the thing counts as in view this tick.", ParamValue{0.02}},
        {"arrive_m", ParamMutability::HotMutable,
            "Remaining range (metres, by dead reckoning) at which the target is reached and dropped.", ParamValue{0.25}},
        {"forget_ticks", ParamMutability::HotMutable,
            "While the thing is unseen the confidence decays by 1/this per tick (about this many ticks of memory).", ParamValue{3000.0}},
        {"floor", ParamMutability::HotMutable,
            "Confidence below which the remembered target is dropped.", ParamValue{0.05}},
    };
}

ParamMap BearingSeekLoop::current_params() const {
    ParamMap m;
    m["bearing_topic"] = ParamValue{bearing_topic_}; m["pose_topic"] = ParamValue{pose_topic_};
    m["output_topic"] = ParamValue{output_topic_}; m["value_topic"] = ParamValue{value_topic_};
    m["range_topic"] = ParamValue{range_topic_};
    m["proximity_range"] = ParamValue{proximity_range_}; m["min_conf"] = ParamValue{double(min_conf_)};
    m["arrive_m"] = ParamValue{arrive_m_}; m["forget_ticks"] = ParamValue{forget_ticks_}; m["floor"] = ParamValue{double(floor_)};
    return m;
}

void BearingSeekLoop::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("BearingSeekLoop requires a non-null Bus");
    apply_param(params, "bearing_topic",   [&](auto const& v){ bearing_topic_ = get_string(v,"bearing_topic"); });
    apply_param(params, "pose_topic",      [&](auto const& v){ pose_topic_    = get_string(v,"pose_topic"); });
    apply_param(params, "output_topic",    [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "value_topic",     [&](auto const& v){ value_topic_   = get_string(v,"value_topic"); });
    apply_param(params, "range_topic",     [&](auto const& v){ range_topic_   = get_string(v,"range_topic"); });
    apply_param(params, "proximity_range", [&](auto const& v){ proximity_range_ = get_double(v,"proximity_range"); });
    apply_param(params, "min_conf",        [&](auto const& v){ min_conf_      = float(get_double(v,"min_conf")); });
    apply_param(params, "arrive_m",        [&](auto const& v){ arrive_m_      = get_double(v,"arrive_m"); });
    apply_param(params, "forget_ticks",    [&](auto const& v){ forget_ticks_  = std::max(1.0, get_double(v,"forget_ticks")); });
    apply_param(params, "floor",           [&](auto const& v){ floor_         = float(get_double(v,"floor")); });
}

void BearingSeekLoop::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "proximity_range") proximity_range_ = get_double(value, k);
    else if (k == "min_conf")        min_conf_ = float(get_double(value, k));
    else if (k == "arrive_m")        arrive_m_ = get_double(value, k);
    else if (k == "forget_ticks")    forget_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "floor")           floor_ = float(get_double(value, k));
    else throw std::invalid_argument("BearingSeekLoop: param '" + k + "' is construction-only / unknown");
}

void BearingSeekLoop::tick(uint64_t tick_id) {
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(pose_topic_)))
        if (pt->values.size() >= 3) { px_ = pt->values[0]; py_ = pt->values[1]; pyaw_ = pt->values[2]; have_pose_ = true; }
    float vx = 0.0f, vy = 0.0f, prox = 0.0f;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(bearing_topic_))) {
        if (pt->values.size() > 0) vx   = float(pt->values[0]);
        if (pt->values.size() > 1) vy   = float(pt->values[1]);
        if (pt->values.size() > 2) prox = float(pt->values[2]);
    }
    seen_ = prox > min_conf_ && (vx * vx + vy * vy) > 1e-6f;
    const double c = std::cos(pyaw_), s = std::sin(pyaw_);
    if (seen_ && have_pose_) {
        // fix the thing's position: the body's pose plus the bearing (body frame: +x forward, +y left)
        // at the range the proximity encodes
        const double n = std::sqrt(double(vx) * vx + double(vy) * vy);
        const double fwd = vy / n, left = -vx / n;                       // cx = +right → left = -cx
        const double range = std::max(0.0, 1.0 - double(prox)) * proximity_range_;
        const double bx = fwd * range, by = left * range;                // body frame
        tx_ = px_ + c * bx - s * by;
        ty_ = py_ + s * bx + c * by;
        have_target_ = true;
        conf_ = 1.0f;
        range_left_ = range;
        cx_ = vx / float(n); cy_ = vy / float(n);
    } else if (have_target_) {
        // home to the remembered position: re-aim from the body's current pose
        const double dx = tx_ - px_, dy = ty_ - py_;
        range_left_ = std::hypot(dx, dy);
        const double bx =  c * dx + s * dy;                               // world → body
        const double by = -s * dx + c * dy;
        if (range_left_ < arrive_m_) {
            have_target_ = false; conf_ = 0.0f; ++arrivals_;              // reached: the belief is fulfilled
            cx_ = 0.0f; cy_ = 0.0f;
        } else {
            conf_ *= float(1.0 - 1.0 / forget_ticks_);
            if (conf_ < floor_) { have_target_ = false; conf_ = 0.0f; ++forgets_; cx_ = 0.0f; cy_ = 0.0f; }
            else { cx_ = float(-by / range_left_); cy_ = float(bx / range_left_); }
        }
    } else {
        cx_ = 0.0f; cy_ = 0.0f; conf_ = 0.0f;
    }
    value_ = have_target_ ? conf_ : 0.0f;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("seek") : id_; out->sensor = "seek_bearing";
    out->values = Eigen::VectorXf::Zero(3);
    out->values[0] = cx_; out->values[1] = cy_; out->values[2] = value_;
    bus_->publish(output_topic_, out);
    auto v = std::make_shared<ProprioToken>();
    v->tick_id = tick_id; v->producer_id = out->producer_id; v->sensor = "seek_value";
    v->values = Eigen::VectorXf::Constant(1, value_);
    bus_->publish(value_topic_, v);
    if (!range_topic_.empty()) {
        auto r = std::make_shared<ProprioToken>();
        r->tick_id = tick_id; r->producer_id = out->producer_id; r->sensor = "seek_range";
        r->values = Eigen::VectorXf::Constant(1, float(range_left_));
        bus_->publish(range_topic_, r);
    }
}

nlohmann::json BearingSeekLoop::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"have_target", have_target_}, {"tx", tx_}, {"ty", ty_}, {"conf", conf_},
                          {"arrivals", arrivals_}, {"forgets", forgets_}};
}
void BearingSeekLoop::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    have_target_ = s.value("have_target", false); tx_ = s.value("tx", 0.0); ty_ = s.value("ty", 0.0);
    conf_ = s.value("conf", 0.0f); arrivals_ = s.value("arrivals", 0); forgets_ = s.value("forgets", 0);
}
nlohmann::json BearingSeekLoop::diag_lite() const {
    return nlohmann::json{{"seen", seen_}, {"target", have_target_}, {"value", value_}, {"range", range_left_},
                          {"arrivals", arrivals_}, {"forgets", forgets_}};
}
nlohmann::json BearingSeekLoop::diag_snapshot() const {
    nlohmann::json j = diag_lite();
    j["cx"] = cx_; j["cy"] = cy_; j["tx"] = tx_; j["ty"] = ty_;
    return j;
}

} // namespace ogma
