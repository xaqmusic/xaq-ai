// =============================================================================
// ProxMemory.cpp  --  see the header
// =============================================================================
#include "ogma/modules/ProxMemory.hpp"

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
    throw std::invalid_argument("ProxMemory: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ProxMemory: param '" + k + "' must be a string");
}
constexpr double kDeg = 3.14159265358979323846 / 180.0;
} // namespace

ProxMemory::ProxMemory()  = default;
ProxMemory::~ProxMemory() = default;

std::string_view ProxMemory::type_name() const { return "ProxMemory"; }

std::vector<TopicSpec> ProxMemory::input_topics() const {
    return { TopicSpec{range_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
             TopicSpec{odom_topic_,  std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
}
std::vector<TopicSpec> ProxMemory::output_topics() const {
    return { TopicSpec{prox_topic_, std::type_index(typeid(ProprioToken))},
             TopicSpec{view_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema ProxMemory::params_schema() const {
    return {
        {"range_topic", ParamMutability::ConstructionOnly, "ProprioToken [distance_m, valid]: the forward range beam (the robot's sense.range).", ParamValue{std::string("sense.range")}},
        {"odom_topic",  ParamMutability::ConstructionOnly, "ProprioToken [x, y, yaw]: the body's dead-reckoned pose (DeadReckon's frame).", ParamValue{std::string("reality.proprio.odom")}},
        {"prox_topic",  ParamMutability::ConstructionOnly, "Output: [minus_x, ahead, plus_x, too_close] in [0,1], 1 = touching. Slot 0 = the body's -X side, slot 2 = +X.", ParamValue{std::string("reality.proprio.prox")}},
        {"view_topic",  ParamMutability::ConstructionOnly, "Output: [8] nearest range per 15 deg sector across +/-60 deg over view_range_m, 1 = nothing. Sector 0 = the -X side.", ParamValue{std::string("reality.proprio.prox_view")}},
        {"range_m",       ParamMutability::HotMutable, "Proximity = 1 - r / range_m.", ParamValue{1.0}},
        {"view_range_m",  ParamMutability::HotMutable, "View = r / view_range_m.", ParamValue{1.5}},
        {"sensor_fwd_m",  ParamMutability::HotMutable, "The beam's origin ahead of the odom origin (the chassis front face, m).", ParamValue{0.049}},
        {"too_close_m",   ParamMutability::HotMutable, "The live ping below this reads as contact: too_close = 1 - d / too_close_m.", ParamValue{0.15}},
        {"memory_ticks",  ParamMutability::HotMutable, "A stored point lives this many ticks (the body's own yaw wander is the sweep).", ParamValue{int64_t{150}}},
        {"behind_m",      ParamMutability::HotMutable, "A point this far behind the sensor is dropped.", ParamValue{0.10}},
        {"ahead_half_deg", ParamMutability::HotMutable, "|azimuth| within this is the ahead slot (the duck's 5.625 deg).", ParamValue{5.625}},
        {"side_max_deg",   ParamMutability::HotMutable, "|azimuth| beyond this leaves the side slots (still in the view).", ParamValue{60.0}},
    };
}

ParamMap ProxMemory::current_params() const {
    ParamMap m;
    m["range_topic"] = ParamValue{range_topic_}; m["odom_topic"] = ParamValue{odom_topic_};
    m["prox_topic"] = ParamValue{prox_topic_};   m["view_topic"] = ParamValue{view_topic_};
    m["range_m"] = ParamValue{range_m_}; m["view_range_m"] = ParamValue{view_range_m_};
    m["sensor_fwd_m"] = ParamValue{sensor_fwd_m_}; m["too_close_m"] = ParamValue{too_close_m_};
    m["memory_ticks"] = ParamValue{int64_t(memory_ticks_)}; m["behind_m"] = ParamValue{behind_m_};
    m["ahead_half_deg"] = ParamValue{ahead_half_deg_}; m["side_max_deg"] = ParamValue{side_max_deg_};
    return m;
}

void ProxMemory::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ProxMemory requires a non-null Bus");
    apply_param(params, "range_topic", [&](auto const& v){ range_topic_ = get_string(v, "range_topic"); });
    apply_param(params, "odom_topic",  [&](auto const& v){ odom_topic_  = get_string(v, "odom_topic"); });
    apply_param(params, "prox_topic",  [&](auto const& v){ prox_topic_  = get_string(v, "prox_topic"); });
    apply_param(params, "view_topic",  [&](auto const& v){ view_topic_  = get_string(v, "view_topic"); });
    apply_param(params, "range_m",      [&](auto const& v){ range_m_      = get_double(v, "range_m"); });
    apply_param(params, "view_range_m", [&](auto const& v){ view_range_m_ = get_double(v, "view_range_m"); });
    apply_param(params, "sensor_fwd_m", [&](auto const& v){ sensor_fwd_m_ = get_double(v, "sensor_fwd_m"); });
    apply_param(params, "too_close_m",  [&](auto const& v){ too_close_m_  = get_double(v, "too_close_m"); });
    apply_param(params, "memory_ticks", [&](auto const& v){ memory_ticks_ = int(get_double(v, "memory_ticks")); });
    apply_param(params, "behind_m",     [&](auto const& v){ behind_m_     = get_double(v, "behind_m"); });
    apply_param(params, "ahead_half_deg", [&](auto const& v){ ahead_half_deg_ = get_double(v, "ahead_half_deg"); });
    apply_param(params, "side_max_deg",   [&](auto const& v){ side_max_deg_   = get_double(v, "side_max_deg"); });
    sub_ids_.push_back(bus_->subscribe(range_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_range(p); }));
    sub_ids_.push_back(bus_->subscribe(odom_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_odom(p); }));
}

void ProxMemory::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "range_m")        range_m_        = get_double(value, k);
    else if (k == "view_range_m")   view_range_m_   = get_double(value, k);
    else if (k == "sensor_fwd_m")   sensor_fwd_m_   = get_double(value, k);
    else if (k == "too_close_m")    too_close_m_    = get_double(value, k);
    else if (k == "memory_ticks")   memory_ticks_   = int(get_double(value, k));
    else if (k == "behind_m")       behind_m_       = get_double(value, k);
    else if (k == "ahead_half_deg") ahead_half_deg_ = get_double(value, k);
    else if (k == "side_max_deg")   side_max_deg_   = get_double(value, k);
    else throw std::invalid_argument("ProxMemory: param '" + k + "' is construction-only / unknown");
}

void ProxMemory::handle_range(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 2) return;
    const double d = double(pt->values[0]);
    const bool valid = pt->values[1] > 0.5f;
    // The same held reading republished every tick is one ping: a new point only when the
    // value or validity changes (the sim and the robot both hold the last ping between pings).
    if (d != last_d_ || valid != last_valid_) { new_ping_ = true; ++pings_; }
    last_d_ = d; last_valid_ = valid;
}

void ProxMemory::handle_odom(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 3) return;
    ox_ = double(pt->values[0]); oy_ = double(pt->values[1]); oyaw_ = double(pt->values[2]);
    have_odom_ = true;
}

void ProxMemory::tick(uint64_t tick_id) {
    last_tick_ = tick_id;
    // Odom-frame conventions (DeadReckon): heading h = (cos yaw, sin yaw); body +X = (-sin yaw, cos yaw).
    const double c = std::cos(oyaw_), s = std::sin(oyaw_);
    // 1. A new valid ping becomes a point at the beam's origin + d along the heading.
    if (new_ping_ && last_valid_ && have_odom_) {
        const double along = sensor_fwd_m_ + last_d_;
        pts_.push_back(Pt{ox_ + along * c, oy_ + along * s, tick_id});
    }
    new_ping_ = false;
    // 2. Age out.
    while (!pts_.empty() && tick_id - pts_.front().t > uint64_t(std::max(1, memory_ticks_))) pts_.pop_front();
    // 3. Re-express every point in the current body frame, relative to the beam's origin,
    //    bin by azimuth (az = atan2(body +X component, forward component)), nearest per bin.
    float best[3] = {1e9f, 1e9f, 1e9f};
    float view_best[8]; for (float& v : view_best) v = 1e9f;
    const double sx = ox_ + sensor_fwd_m_ * c, sy = oy_ + sensor_fwd_m_ * s;   // the beam's origin, odom frame
    for (auto it = pts_.begin(); it != pts_.end();) {
        const double dx = it->x - sx, dy = it->y - sy;
        const double fwd = dx * c + dy * s;             // along the heading
        const double lat = -dx * s + dy * c;            // along body +X
        if (fwd < -behind_m_) { it = pts_.erase(it); continue; }
        const double r  = std::sqrt(fwd * fwd + lat * lat);
        const double az = std::atan2(lat, fwd) / kDeg;  // degrees, + = toward body +X
        if (std::fabs(az) <= ahead_half_deg_)       best[1] = std::min(best[1], float(r));
        else if (az > 0 && az <= side_max_deg_)     best[2] = std::min(best[2], float(r));
        else if (az < 0 && -az <= side_max_deg_)    best[0] = std::min(best[0], float(r));
        if (std::fabs(az) < 60.0) {
            const int sec = std::clamp(int((az + 60.0) / 15.0), 0, 7);
            view_best[sec] = std::min(view_best[sec], float(r));
        }
        ++it;
    }
    for (int i = 0; i < 3; ++i)
        slots_[i] = best[i] < 1e8f ? float(std::clamp(1.0 - double(best[i]) / range_m_, 0.0, 1.0)) : 0.0f;
    // 4. too_close from the LIVE ping only: contact is now, not remembered.
    slots_[3] = (last_valid_ && last_d_ < too_close_m_)
              ? float(std::clamp(1.0 - last_d_ / too_close_m_, 0.0, 1.0)) : 0.0f;
    for (int k = 0; k < 8; ++k)
        view_[k] = view_best[k] < 1e8f ? float(std::clamp(double(view_best[k]) / view_range_m_, 0.0, 1.0)) : 1.0f;

    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("prox") : id_; out->sensor = "prox";
    out->values = Eigen::VectorXf(4);
    for (int i = 0; i < 4; ++i) out->values[i] = slots_[i];
    bus_->publish(prox_topic_, out);
    auto vw = std::make_shared<ProprioToken>();
    vw->tick_id = tick_id; vw->producer_id = out->producer_id; vw->sensor = "prox_view";
    vw->values = Eigen::VectorXf(8);
    for (int k = 0; k < 8; ++k) vw->values[k] = view_[k];
    bus_->publish(view_topic_, vw);
}

nlohmann::json ProxMemory::snapshot_state() const {
    nlohmann::json pts = nlohmann::json::array();
    for (auto const& p : pts_) pts.push_back({p.x, p.y, p.t});
    return nlohmann::json{{"version", 1}, {"points", pts}, {"pings", pings_},
                          {"last_d", last_d_}, {"last_valid", last_valid_}};
}
void ProxMemory::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    pts_.clear();
    for (auto const& p : s.value("points", nlohmann::json::array()))
        if (p.is_array() && p.size() == 3) pts_.push_back(Pt{p[0].get<double>(), p[1].get<double>(), p[2].get<uint64_t>()});
    pings_ = s.value("pings", int64_t{0});
}
nlohmann::json ProxMemory::diag_snapshot() const {
    return nlohmann::json{{"minus_x", slots_[0]}, {"ahead", slots_[1]}, {"plus_x", slots_[2]}, {"too_close", slots_[3]},
                          {"points", int(pts_.size())}, {"pings", pings_}, {"last_d", last_d_}, {"last_valid", last_valid_}};
}

} // namespace ogma
