// =============================================================================
// TouchMemory.cpp  --  see the header
// =============================================================================
#include "ogma/modules/TouchMemory.hpp"

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
    throw std::invalid_argument("TouchMemory: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("TouchMemory: param '" + k + "' must be a string");
}
} // namespace

TouchMemory::TouchMemory()  = default;
TouchMemory::~TouchMemory() = default;

std::string_view TouchMemory::type_name() const { return "TouchMemory"; }

std::vector<TopicSpec> TouchMemory::input_topics() const {
    std::vector<TopicSpec> v = {
        TopicSpec{contact_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{feet_xz_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{feet_y_topic_,  std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{odom_topic_,    std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
    if (!load_topic_.empty())
        v.push_back(TopicSpec{load_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> TouchMemory::output_topics() const { return {}; }

ParamSchema TouchMemory::params_schema() const {
    return {
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4]: per-leg contact 0/1 (sim physics flag; robot FSR >= 200 counts). A touch is the RISING edge.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"feet_xz_topic", ParamMutability::ConstructionOnly, "ProprioToken [8]: per leg [x, z] of the toe in the chassis rest frame at the commanded pose (x = body +X, z = forward).", ParamValue{std::string("reality.proprio.feet_xz")}},
        {"feet_y_topic",  ParamMutability::ConstructionOnly, "ProprioToken [4]: per leg toe height along fused gravity, body origin 0 (planted ~ -0.09 m). Use reality.proprio.toe_y (the toe TIP, calibrated on the robot); the default feet_y_gravity_cmd_imu is the gait's parity foot (shank centre minus half a shank), which sits below the toe by up to 43 mm depending on the pose.", ParamValue{std::string("reality.proprio.feet_y_gravity_cmd_imu")}},
        {"odom_topic",    ParamMutability::ConstructionOnly, "ProprioToken [x, y, yaw]: the body's dead-reckoned pose (DeadReckon's frame).", ParamValue{std::string("reality.proprio.odom")}},
        {"load_topic",    ParamMutability::ConstructionOnly, "Optional ProprioToken [4]: per leg load as a fraction of body weight, kept per voxel as load_max. \"\" = none.", ParamValue{std::string("")}},
        {"voxel_m",          ParamMutability::HotMutable, "Voxel edge (m). The kinematic error is ~3 cm, so 2 cm voxels make agreement visible without hiding it.", ParamValue{0.02}},
        {"odom_translation", ParamMutability::HotMutable, "How much of the odometry's TRANSLATION carries the stored touches (ProxMemory's knob): 1 = the literal odometry, 0 = rotation only.", ParamValue{1.0}},
        {"max_voxels",       ParamMutability::HotMutable, "The memory's size; the least recently touched voxel goes first.", ParamValue{int64_t{4000}}},
        {"diag_max",         ParamMutability::HotMutable, "diag_snapshot lists at most this many voxels, nearest to the body first (0 = all).", ParamValue{int64_t{400}}},
        {"plane_tau_ticks",  ParamMutability::HotMutable, "EMA time constant (ticks) of the support plane: the median height of the toes that were already planted.", ParamValue{50.0}},
        {"leg_y_offset_fl",  ParamMutability::HotMutable, "Floor calibration (m) added to leg fl's toe height (sim order; physical FR on the robot). toe_floor_calib.py's a_mm / 1000. 0 = none.", ParamValue{0.0}},
        {"leg_y_offset_fr",  ParamMutability::HotMutable, "Floor calibration (m) added to leg fr's toe height (physical FL on the robot).", ParamValue{0.0}},
        {"leg_y_offset_rl",  ParamMutability::HotMutable, "Floor calibration (m) added to leg rl's toe height (physical RR on the robot).", ParamValue{0.0}},
        {"leg_y_offset_rr",  ParamMutability::HotMutable, "Floor calibration (m) added to leg rr's toe height (physical RL on the robot).", ParamValue{0.0}},
    };
}

ParamMap TouchMemory::current_params() const {
    ParamMap m;
    m["contact_topic"] = ParamValue{contact_topic_}; m["feet_xz_topic"] = ParamValue{feet_xz_topic_};
    m["feet_y_topic"] = ParamValue{feet_y_topic_};   m["odom_topic"] = ParamValue{odom_topic_};
    m["load_topic"] = ParamValue{load_topic_};
    m["voxel_m"] = ParamValue{voxel_m_}; m["odom_translation"] = ParamValue{odom_translation_};
    m["max_voxels"] = ParamValue{int64_t(max_voxels_)}; m["diag_max"] = ParamValue{int64_t(diag_max_)};
    m["plane_tau_ticks"] = ParamValue{plane_tau_ticks_};
    m["leg_y_offset_fl"] = ParamValue{leg_y_offset_[0]}; m["leg_y_offset_fr"] = ParamValue{leg_y_offset_[1]};
    m["leg_y_offset_rl"] = ParamValue{leg_y_offset_[2]}; m["leg_y_offset_rr"] = ParamValue{leg_y_offset_[3]};
    return m;
}

void TouchMemory::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("TouchMemory requires a non-null Bus");
    apply_param(params, "contact_topic", [&](auto const& v){ contact_topic_ = get_string(v, "contact_topic"); });
    apply_param(params, "feet_xz_topic", [&](auto const& v){ feet_xz_topic_ = get_string(v, "feet_xz_topic"); });
    apply_param(params, "feet_y_topic",  [&](auto const& v){ feet_y_topic_  = get_string(v, "feet_y_topic"); });
    apply_param(params, "odom_topic",    [&](auto const& v){ odom_topic_    = get_string(v, "odom_topic"); });
    apply_param(params, "load_topic",    [&](auto const& v){ load_topic_    = get_string(v, "load_topic"); });
    for (auto const& k : {"voxel_m", "odom_translation", "max_voxels", "diag_max", "plane_tau_ticks",
                          "leg_y_offset_fl", "leg_y_offset_fr", "leg_y_offset_rl", "leg_y_offset_rr"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    sub_ids_.push_back(bus_->subscribe(contact_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle_contact(p); }));
    sub_ids_.push_back(bus_->subscribe(feet_xz_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle_feet_xz(p); }));
    sub_ids_.push_back(bus_->subscribe(feet_y_topic_,  SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle_feet_y(p); }));
    sub_ids_.push_back(bus_->subscribe(odom_topic_,    SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle_odom(p); }));
    if (!load_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(load_topic_, SubscriptionKind::Direct, [this](std::string_view, MessagePtr p){ handle_load(p); }));
}

void TouchMemory::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "voxel_m")          voxel_m_ = std::max(0.002, get_double(value, k));
    else if (k == "odom_translation") odom_translation_ = get_double(value, k);
    else if (k == "max_voxels")       max_voxels_ = std::max(1, int(get_double(value, k)));
    else if (k == "diag_max")         diag_max_ = std::max(0, int(get_double(value, k)));
    else if (k == "plane_tau_ticks")  plane_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "leg_y_offset_fl")  leg_y_offset_[0] = get_double(value, k);
    else if (k == "leg_y_offset_fr")  leg_y_offset_[1] = get_double(value, k);
    else if (k == "leg_y_offset_rl")  leg_y_offset_[2] = get_double(value, k);
    else if (k == "leg_y_offset_rr")  leg_y_offset_[3] = get_double(value, k);
    else throw std::invalid_argument("TouchMemory: param '" + k + "' is construction-only / unknown");
}

void TouchMemory::handle_contact(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 4) return;
    for (int i = 0; i < 4; ++i) contact_[i] = pt->values[i];
    have_contact_ = true;
}
void TouchMemory::handle_feet_xz(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 8) return;
    for (int i = 0; i < 8; ++i) feet_xz_[i] = pt->values[i];
    have_xz_ = true;
}
void TouchMemory::handle_feet_y(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 4) return;
    for (int i = 0; i < 4; ++i) feet_y_[i] = pt->values[i] + float(leg_y_offset_[i]);   // the floor calibration
    have_y_ = true;
}
void TouchMemory::handle_odom(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 3) return;
    ox_ = double(pt->values[0]); oy_ = double(pt->values[1]); oyaw_ = double(pt->values[2]);
    have_odom_ = true;
}
void TouchMemory::handle_load(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 4) return;
    for (int i = 0; i < 4; ++i) load_[i] = pt->values[i];
    have_load_ = true;
}

// Odom-frame conventions (DeadReckon): heading = (cos yaw, sin yaw); body +X = (-sin yaw, cos yaw).
// With no odometry at all the body frame IS the frame (a bench twin that never walks).
void TouchMemory::odom_pose(double& px, double& py, double& c, double& s) const {
    if (!have_odom_) { px = 0.0; py = 0.0; c = 1.0; s = 0.0; return; }
    c = std::cos(oyaw_); s = std::sin(oyaw_);
    px = (odom_translation_ == 1.0) ? ox_ : ox_ * odom_translation_;
    py = (odom_translation_ == 1.0) ? oy_ : oy_ * odom_translation_;
}

void TouchMemory::add_touch(int leg, double x, double y, double h, float load, uint64_t t) {
    const Key k{ int64_t(std::floor(x / voxel_m_)), int64_t(std::floor(y / voxel_m_)), int64_t(std::floor(h / voxel_m_)) };
    auto it = vox_.find(k);
    if (it == vox_.end()) {
        evict_if_full();
        it = vox_.emplace(k, Voxel{}).first;
        it->second.first_t = t;
    }
    Voxel& v = it->second;
    v.sx += x; v.sy += y; v.sh += h; ++v.hits; v.last_t = t;
    v.foot_mask |= (1 << leg);
    v.load_max = std::max(v.load_max, load);
    ++touches_;
}

void TouchMemory::evict_if_full() {
    while (int(vox_.size()) >= max_voxels_) {
        auto oldest = vox_.begin();
        for (auto it = vox_.begin(); it != vox_.end(); ++it)
            if (it->second.last_t < oldest->second.last_t) oldest = it;
        vox_.erase(oldest);
    }
}

void TouchMemory::tick(uint64_t tick_id) {
    last_tick_ = tick_id;
    if (!have_contact_) return;
    // Which feet LAND this tick: the rising edge of their contact flag.
    bool rising[4];
    for (int i = 0; i < 4; ++i) rising[i] = contact_[i] > 0.5f && prev_contact_[i] <= 0.5f;
    // The support plane from the feet that were already planted (never the landing foot).
    if (have_y_) {
        float planted[4]; int n = 0;
        for (int i = 0; i < 4; ++i)
            if (contact_[i] > 0.5f && !rising[i]) planted[n++] = feet_y_[i];
        if (n > 0) {
            std::sort(planted, planted + n);
            const double med = (n % 2) ? planted[n / 2] : 0.5 * (planted[n / 2 - 1] + planted[n / 2]);
            if (!have_plane_) { plane_y_ = med; have_plane_ = true; }
            else plane_y_ += (med - plane_y_) / plane_tau_ticks_;
        }
    }
    if (have_xz_ && have_y_) {
        double px, py, c, s; odom_pose(px, py, c, s);
        for (int i = 0; i < 4; ++i) {
            if (!rising[i]) continue;
            const double lat = double(feet_xz_[2 * i]), fwd = double(feet_xz_[2 * i + 1]);
            const double h = have_plane_ ? double(feet_y_[i]) - plane_y_ : 0.0;
            if (!have_plane_) { plane_y_ = feet_y_[i]; have_plane_ = true; }   // the first touch defines the floor
            const double X = px + fwd * c - lat * s;
            const double Y = py + fwd * s + lat * c;
            add_touch(i, X, Y, h, have_load_ ? load_[i] : 0.0f, tick_id);
        }
    }
    for (int i = 0; i < 4; ++i) prev_contact_[i] = contact_[i];
}

std::vector<TouchMemory::BodyVoxel> TouchMemory::body_voxels(int n) const {
    double px, py, c, s; odom_pose(px, py, c, s);
    std::vector<BodyVoxel> out; out.reserve(vox_.size());
    for (auto const& [k, v] : vox_) {
        const double x = v.sx / v.hits, y = v.sy / v.hits, h = v.sh / v.hits;
        const double dx = x - px, dy = y - py;
        out.push_back(BodyVoxel{ dx * c + dy * s, -dx * s + dy * c, h, v.hits, v.foot_mask,
                                 last_tick_ >= v.last_t ? last_tick_ - v.last_t : 0, v.load_max });
    }
    std::sort(out.begin(), out.end(), [](BodyVoxel const& a, BodyVoxel const& b) {
        return a.fwd * a.fwd + a.lat * a.lat + a.h * a.h < b.fwd * b.fwd + b.lat * b.lat + b.h * b.h; });
    if (n > 0 && int(out.size()) > n) out.resize(size_t(n));
    return out;
}

nlohmann::json TouchMemory::snapshot_state() const {
    nlohmann::json vs = nlohmann::json::array();
    for (auto const& [k, v] : vox_)
        vs.push_back({k.ix, k.iy, k.iz, v.sx, v.sy, v.sh, v.hits, v.first_t, v.last_t, v.foot_mask, v.load_max});
    return nlohmann::json{{"version", 1}, {"voxels", vs}, {"touches", touches_},
                          {"plane_y", plane_y_}, {"have_plane", have_plane_}, {"voxel_m", voxel_m_}};
}
void TouchMemory::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    vox_.clear();
    for (auto const& e : s.value("voxels", nlohmann::json::array())) {
        if (!e.is_array() || e.size() != 11) continue;
        Key k{e[0].get<int64_t>(), e[1].get<int64_t>(), e[2].get<int64_t>()};
        Voxel v; v.sx = e[3]; v.sy = e[4]; v.sh = e[5]; v.hits = e[6]; v.first_t = e[7]; v.last_t = e[8];
        v.foot_mask = e[9]; v.load_max = e[10];
        vox_[k] = v;
    }
    touches_ = s.value("touches", int64_t{0});
    plane_y_ = s.value("plane_y", 0.0); have_plane_ = s.value("have_plane", false);
}
nlohmann::json TouchMemory::diag_snapshot() const {
    nlohmann::json vs = nlohmann::json::array();
    for (auto const& b : body_voxels(diag_max_))
        vs.push_back({std::round(b.fwd * 1e4) / 1e4, std::round(b.lat * 1e4) / 1e4, std::round(b.h * 1e4) / 1e4,
                      b.hits, b.foot_mask, b.age, std::round(double(b.load_max) * 1e3) / 1e3});
    return nlohmann::json{{"voxels", vs}, {"count", int(vox_.size())}, {"touches", touches_},
                          {"plane_y", plane_y_}, {"have_plane", have_plane_}, {"voxel_m", voxel_m_},
                          {"odom", {ox_, oy_, oyaw_}}, {"have_odom", have_odom_}, {"odom_translation", odom_translation_},
                          // Input health, so a dashboard can say WHICH stream is missing rather than show an empty cloud.
                          {"have_contact", have_contact_}, {"have_xz", have_xz_}, {"have_y", have_y_}, {"have_load", have_load_},
                          {"leg_y_offset", {leg_y_offset_[0], leg_y_offset_[1], leg_y_offset_[2], leg_y_offset_[3]}},
                          {"ticks", last_tick_}};
}

} // namespace ogma
