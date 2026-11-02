// =============================================================================
// EncounterFeatures.cpp  --  see the header
// =============================================================================
#include "ogma/modules/EncounterFeatures.hpp"
#include <variant>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

namespace ogma {
namespace {
template <typename Fn> void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key); if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("EncounterFeatures: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("EncounterFeatures: param '" + k + "' must be a string");
}
float lean_of(float upright) { return float(std::acos(std::max(-1.0f, std::min(1.0f, upright)))); }
} // namespace

EncounterFeatures::EncounterFeatures()  = default;
EncounterFeatures::~EncounterFeatures() = default;
std::string_view EncounterFeatures::type_name() const { return "EncounterFeatures"; }

std::vector<TopicSpec> EncounterFeatures::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : {contact_topic_, feet_xz_topic_, feet_y_topic_, load_topic_, current_topic_, belly_topic_, upright_topic_, range_topic_})
        if (!t.empty()) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> EncounterFeatures::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema EncounterFeatures::params_schema() const {
    return {
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4] per-leg contact 0/1; an event is the rising edge.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"feet_xz_topic", ParamMutability::ConstructionOnly, "ProprioToken [8] per leg [x = body +X, z = forward] of the toe at the commanded pose.", ParamValue{std::string("reality.proprio.feet_xz")}},
        {"feet_y_topic",  ParamMutability::ConstructionOnly, "ProprioToken [4] toe height along gravity, body origin 0. Use reality.proprio.toe_y (the toe TIP, calibrated on the robot); the default is the gait's parity foot, up to 43 mm below the toe.", ParamValue{std::string("reality.proprio.feet_y_gravity_cmd_imu")}},
        {"load_topic",    ParamMutability::ConstructionOnly, "ProprioToken [4] foot load as a fraction of body weight (\"\" = none).", ParamValue{std::string("reality.proprio.foot_load")}},
        {"current_topic", ParamMutability::ConstructionOnly, "ProprioToken [1] bus current (A) (\"\" = none).", ParamValue{std::string("sense.servo_current")}},
        {"belly_topic",   ParamMutability::ConstructionOnly, "ProprioToken [1] belly clearance (m) (\"\" = none).", ParamValue{std::string("reality.proprio.belly_m")}},
        {"upright_topic", ParamMutability::ConstructionOnly, "ProprioToken [1] the body's up.y (\"\" = none).", ParamValue{std::string("reality.proprio.upright")}},
        {"range_topic",   ParamMutability::ConstructionOnly, "ProprioToken [m, valid] the forward range (\"\" = none).", ParamValue{std::string("sense.range")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "The feature vector, one token per touch at its window's end.", ParamValue{std::string("sense.touch_encounter")}},
        {"pre_ticks",     ParamMutability::HotMutable, "Pre-buffer length (ticks): the approach direction and the current rise before onset are read over it.", ParamValue{int64_t{15}}},
        {"post_ticks",    ParamMutability::HotMutable, "Window after onset (ticks): the body's response is read over it; the token is published when it closes.", ParamValue{int64_t{25}}},
        {"plane_tau_ticks", ParamMutability::HotMutable, "EMA time constant of the support plane (the median height of the toes already planted).", ParamValue{50.0}},
        {"belly_plane_m",   ParamMutability::HotMutable, "The belly plane below the body origin (m): with a valid belly ToF the floor is belly_plane_m - clearance, and toe heights are taken from THAT floor (the rear pair measured -0.029 on the robot, 2026-11-02); the planted feet's plane is the fallback. 0 = never use the ToF.", ParamValue{-0.029}},
        {"leg_y_offset_fl", ParamMutability::HotMutable, "Floor calibration (m) added to leg fl's toe height (TouchMemory's).", ParamValue{0.0}},
        {"leg_y_offset_fr", ParamMutability::HotMutable, "Floor calibration (m), leg fr.", ParamValue{0.0}},
        {"leg_y_offset_rl", ParamMutability::HotMutable, "Floor calibration (m), leg rl.", ParamValue{0.0}},
        {"leg_y_offset_rr", ParamMutability::HotMutable, "Floor calibration (m), leg rr.", ParamValue{0.0}},
        {"position_relative", ParamMutability::HotMutable, "Publish the toe's position RELATIVE to the leg's running mean of its touch positions (EMA over position_tau_touches touches; the first touch per leg reads 0), so the vocabulary is about the object, not the leg (conditioning, 2026-11-02). Off = the body-frame position.", ParamValue{false}},
        {"position_tau_touches", ParamMutability::HotMutable, "Touches per leg over which the position mean adapts.", ParamValue{20.0}},
        {"recent_max", ParamMutability::HotMutable, "How many recent events the diag keeps as rows [tick, leg, features...].", ParamValue{int64_t{32}}},
    };
}

ParamMap EncounterFeatures::current_params() const {
    ParamMap m;
    m["contact_topic"] = ParamValue{contact_topic_}; m["feet_xz_topic"] = ParamValue{feet_xz_topic_}; m["feet_y_topic"] = ParamValue{feet_y_topic_};
    m["load_topic"] = ParamValue{load_topic_}; m["current_topic"] = ParamValue{current_topic_}; m["belly_topic"] = ParamValue{belly_topic_};
    m["upright_topic"] = ParamValue{upright_topic_}; m["range_topic"] = ParamValue{range_topic_}; m["output_topic"] = ParamValue{output_topic_};
    m["pre_ticks"] = ParamValue{int64_t(pre_ticks_)}; m["post_ticks"] = ParamValue{int64_t(post_ticks_)}; m["plane_tau_ticks"] = ParamValue{plane_tau_ticks_};
    m["belly_plane_m"] = ParamValue{belly_plane_m_};
    m["position_relative"] = ParamValue{position_relative_}; m["position_tau_touches"] = ParamValue{position_tau_touches_}; m["recent_max"] = ParamValue{int64_t(recent_max_)};
    m["leg_y_offset_fl"] = ParamValue{leg_y_offset_[0]}; m["leg_y_offset_fr"] = ParamValue{leg_y_offset_[1]};
    m["leg_y_offset_rl"] = ParamValue{leg_y_offset_[2]}; m["leg_y_offset_rr"] = ParamValue{leg_y_offset_[3]};
    return m;
}

void EncounterFeatures::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("EncounterFeatures requires a non-null Bus");
    const std::pair<const char*, std::string*> topics[] = {
        {"contact_topic", &contact_topic_}, {"feet_xz_topic", &feet_xz_topic_}, {"feet_y_topic", &feet_y_topic_},
        {"load_topic", &load_topic_}, {"current_topic", &current_topic_}, {"belly_topic", &belly_topic_}, {"upright_topic", &upright_topic_},
        {"range_topic", &range_topic_}, {"output_topic", &output_topic_}};
    for (auto const& pr : topics)
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    for (auto const& k : {"pre_ticks", "post_ticks", "plane_tau_ticks", "belly_plane_m", "leg_y_offset_fl", "leg_y_offset_fr", "leg_y_offset_rl", "leg_y_offset_rr",
                          "position_relative", "position_tau_touches", "recent_max"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    auto sub = [&](std::string const& topic, std::string which) {
        if (topic.empty()) return;
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct, [this, which](std::string_view, MessagePtr p){ handle(which, p); }));
    };
    sub(contact_topic_, "contact"); sub(feet_xz_topic_, "xz"); sub(feet_y_topic_, "y"); sub(load_topic_, "load");
    sub(current_topic_, "cur"); sub(belly_topic_, "belly"); sub(upright_topic_, "up"); sub(range_topic_, "range");
}

void EncounterFeatures::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "pre_ticks")        pre_ticks_ = std::max(2, int(get_double(value, k)));
    else if (k == "post_ticks")       post_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "plane_tau_ticks")  plane_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "belly_plane_m")    belly_plane_m_ = get_double(value, k);
    else if (k == "position_relative") position_relative_ = std::holds_alternative<bool>(value) ? std::get<bool>(value) : get_double(value, k) != 0.0;
    else if (k == "position_tau_touches") position_tau_touches_ = std::max(1.0, get_double(value, k));
    else if (k == "recent_max")       recent_max_ = std::max(0, int(get_double(value, k)));
    else if (k == "leg_y_offset_fl")  leg_y_offset_[0] = get_double(value, k);
    else if (k == "leg_y_offset_fr")  leg_y_offset_[1] = get_double(value, k);
    else if (k == "leg_y_offset_rl")  leg_y_offset_[2] = get_double(value, k);
    else if (k == "leg_y_offset_rr")  leg_y_offset_[3] = get_double(value, k);
    else throw std::invalid_argument("EncounterFeatures: param '" + k + "' is construction-only / unknown");
}

void EncounterFeatures::handle(std::string const& which, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    auto const& v = pt->values;
    if (which == "contact" && v.size() >= 4)      { for (int i = 0; i < 4; ++i) contact_[i] = v[i]; have_contact_ = true; }
    else if (which == "xz" && v.size() >= 8)      { for (int i = 0; i < 8; ++i) feet_xz_[i] = v[i]; have_xz_ = true; }
    else if (which == "y" && v.size() >= 4)       { for (int i = 0; i < 4; ++i) feet_y_[i] = v[i] + float(leg_y_offset_[i]); have_y_ = true; }
    else if (which == "load" && v.size() >= 4)    { for (int i = 0; i < 4; ++i) load_[i] = v[i]; }
    else if (which == "cur" && v.size() >= 1)     { cur_ = v[0]; have_cur_ = true; }
    else if (which == "belly" && v.size() >= 1)   { belly_ = v[0]; have_belly_ = true; }
    else if (which == "up" && v.size() >= 1)      { upright_ = v[0]; have_up_ = true; }
    else if (which == "range" && v.size() >= 2)   { range_ = v[0]; range_valid_ = v[1] > 0.5f; }
}

void EncounterFeatures::tick(uint64_t tick_id) {
    last_tick_ = tick_id;
    if (!have_contact_) return;
    // the support plane, from the feet that were already planted (TouchMemory's rule)
    bool rising[4];
    for (int i = 0; i < 4; ++i) rising[i] = contact_[i] > 0.5f && prev_contact_[i] <= 0.5f;
    if (have_y_) {
        float planted[4]; int n = 0;
        for (int i = 0; i < 4; ++i) if (contact_[i] > 0.5f && !rising[i]) planted[n++] = feet_y_[i];
        if (n > 0) {
            std::sort(planted, planted + n);
            const double med = (n % 2) ? planted[n / 2] : 0.5 * (planted[n / 2 - 1] + planted[n / 2]);
            if (!have_plane_) { plane_y_ = med; have_plane_ = true; } else plane_y_ += (med - plane_y_) / plane_tau_ticks_;
        }
    }
    const float lean_now = lean_of(upright_);
    // open windows on rising edges -- once the pre-buffer is full: the feet the body was found standing on are a
    // starting state, not encounters, and the approach direction needs the buffer
    const bool armed = int(pre_.size()) >= pre_ticks_;
    for (int i = 0; i < 4; ++i) {
        if (!armed || !rising[i] || !have_xz_ || !have_y_) continue;
        Window& w = win_[i];
        w = Window{}; w.open = true; w.t_on = tick_id;
        // the floor: from the belly ToF when it reads (the belly plane minus the clearance), else the planted feet's plane
        const bool tof_floor = belly_plane_m_ != 0.0 && have_belly_;
        const float floor_y = tof_floor ? float(belly_plane_m_) - belly_ : float(plane_y_);
        w.h = (tof_floor || have_plane_) ? feet_y_[i] - floor_y : 0.0f;
        w.lat = feet_xz_[2 * i]; w.fwd = feet_xz_[2 * i + 1];
        // the position relative to this leg's own mean touch position (the mean before this touch; first touch = 0)
        if (pos_n_[i] == 0) { pos_mean_[i] = {w.fwd, w.lat}; }
        w.fwd_rel = float(w.fwd - pos_mean_[i][0]); w.lat_rel = float(w.lat - pos_mean_[i][1]);
        { const double a = 1.0 / std::min(double(pos_n_[i] + 1), position_tau_touches_);
          pos_mean_[i][0] += a * (w.fwd - pos_mean_[i][0]); pos_mean_[i][1] += a * (w.lat - pos_mean_[i][1]); ++pos_n_[i]; }
        w.i0 = cur_; w.belly0 = belly_; w.lean0 = lean_now; w.range = range_valid_ ? range_ : -1.0f;
        w.i_max = cur_; w.belly_max = belly_; w.lean_max = lean_now; w.load_peak = load_[i];
        if (!pre_.empty()) {
            float i_min = cur_; float dy = 0, dxz = 0;
            for (auto const& s : pre_) i_min = std::min(i_min, s.cur);
            auto const& first = pre_.front();
            dy = std::fabs(feet_y_[i] - first.y[i]);
            dxz = std::hypot(feet_xz_[2 * i] - first.xz[2 * i], feet_xz_[2 * i + 1] - first.xz[2 * i + 1]);
            w.i_pre = cur_ - i_min;
            w.vert = (dy + dxz) > 1e-6f ? dy / (dy + dxz) : 0.0f;
        }
    }
    // advance open windows; publish when they close
    for (int i = 0; i < 4; ++i) {
        Window& w = win_[i];
        if (!w.open) continue;
        w.i_max = std::max(w.i_max, cur_); w.belly_max = std::max(w.belly_max, belly_); w.lean_max = std::max(w.lean_max, lean_now);
        w.load_peak = std::max(w.load_peak, load_[i]);
        if (++w.ticks >= post_ticks_) { publish(i, w, tick_id); w.open = false; }
    }
    // the pre-buffer
    pre_.push_back(Sample{cur_, belly_, lean_now, feet_xz_, feet_y_});
    while (int(pre_.size()) > pre_ticks_) pre_.pop_front();
    for (int i = 0; i < 4; ++i) prev_contact_[i] = contact_[i];
}

void EncounterFeatures::publish(int leg, Window const& w, uint64_t tick_id) {
    last_ = {w.h, position_relative_ ? w.fwd_rel : w.fwd, position_relative_ ? w.lat_rel : w.lat, w.load_peak, w.i_pre, w.i_max - w.i0,
             w.belly_max - w.belly0, w.lean_max - w.lean0, w.range, w.vert};
    last_leg_ = leg; ++events_;
    if (recent_max_ > 0) {
        std::array<float, kDim + 2> row{}; row[0] = float(tick_id); row[1] = float(leg);
        for (int k = 0; k < kDim; ++k) row[size_t(k + 2)] = last_[size_t(k)];
        recent_.push_back(row); while (int(recent_.size()) > recent_max_) recent_.pop_front();
    }
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id_.empty() ? std::string("encounter") : id_; out->sensor = "touch_encounter";
    out->values = Eigen::VectorXf(kDim);
    for (int k = 0; k < kDim; ++k) out->values[k] = last_[size_t(k)];
    bus_->publish(output_topic_, out);
}

nlohmann::json EncounterFeatures::diag_snapshot() const {
    nlohmann::json f = nlohmann::json::array();
    for (float x : last_) f.push_back(x);
    int open = 0; for (auto const& w : win_) open += w.open ? 1 : 0;
    nlohmann::json recent = nlohmann::json::array();
    for (auto const& r : recent_) { nlohmann::json row = nlohmann::json::array(); for (float x : r) row.push_back(x); recent.push_back(row); }
    return nlohmann::json{{"events", events_}, {"last", f}, {"last_leg", last_leg_}, {"last_tick", last_tick_}, {"open_windows", open}, {"plane_y", plane_y_},
                          {"recent", recent}, {"position_relative", position_relative_},
                          {"have", {{"contact", have_contact_}, {"xz", have_xz_}, {"y", have_y_}, {"current", have_cur_}, {"belly", have_belly_}, {"upright", have_up_}}},
                          {"names", {"h_plane_m", "fwd_m", "lat_m", "load_peak", "i_rise_pre_a", "i_rise_post_a", "belly_rise_m", "lean_rise_rad", "range_m", "vert_frac"}}};
}

} // namespace ogma
