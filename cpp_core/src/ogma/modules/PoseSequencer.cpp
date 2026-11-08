#include "ogma/modules/PoseSequencer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

namespace ogma {
namespace {
template <typename Fn> void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key); if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("PoseSequencer: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PoseSequencer: param '" + k + "' must be a string");
}
using body::Vec3f;
Vec3f sub(const Vec3f& a, const Vec3f& b) { return Vec3f(a.x - b.x, a.y - b.y, a.z - b.z); }
Vec3f add(const Vec3f& a, const Vec3f& b) { return Vec3f(a.x + b.x, a.y + b.y, a.z + b.z); }
double len(const Vec3f& a) { return std::sqrt(double(a.x) * a.x + double(a.y) * a.y + double(a.z) * a.z); }
const char* phase_name(PoseSequencer::Phase p) {
    switch (p) { case PoseSequencer::Phase::Idle: return "idle"; case PoseSequencer::Phase::Shift: return "shift"; case PoseSequencer::Phase::Lift: return "lift";
                 case PoseSequencer::Phase::Swing: return "swing"; case PoseSequencer::Phase::Lower: return "lower"; }
    return "?";
}
} // namespace

PoseSequencer::PoseSequencer()  = default;
PoseSequencer::~PoseSequencer() = default;
std::string_view PoseSequencer::type_name() const { return "PoseSequencer"; }

std::vector<TopicSpec> PoseSequencer::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : {model_topic_, support_topic_, patch_topic_, feet_xz_topic_, feet_y_topic_, up_topic_, contact_topic_, load_topic_})
        if (!t.empty()) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    for (auto const& t : gait_action_topics_) if (!t.empty()) v.push_back(TopicSpec{t, std::type_index(typeid(ActionOut)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> PoseSequencer::output_topics() const { return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} }; }

ParamSchema PoseSequencer::params_schema() const {
    return {
        {"model_topic",   ParamMutability::ConstructionOnly, "ProprioToken [116]: the body's leg model (reality.proprio.leg_model) -- the shared FK's anchors and the action-to-angle map.", ParamValue{std::string("reality.proprio.leg_model")}},
        {"support_topic", ParamMutability::ConstructionOnly, "ProprioToken: SupportModel's support.state (the per-leg lift margins decide when a leg may leave).", ParamValue{std::string("support.state")}},
        {"patch_topic",   ParamMutability::ConstructionOnly, "ProprioToken: TerrainMemory's patch (the ground under a toe and at its landing point).", ParamValue{std::string("terrain.patch")}},
        {"feet_xz_topic", ParamMutability::ConstructionOnly, "ProprioToken [x, z] x 4: the body's own toes (the FK check).", ParamValue{std::string("reality.proprio.feet_xz")}},
        {"feet_y_topic",  ParamMutability::ConstructionOnly, "ProprioToken [4]: the toes' gravity heights (the FK check).", ParamValue{std::string("reality.proprio.toe_y")}},
        {"up_topic",      ParamMutability::ConstructionOnly, "ProprioToken [3]: the body's gravity-up in the body frame.", ParamValue{std::string("reality.proprio.up_body")}},
        {"contact_topic", ParamMutability::ConstructionOnly, "ProprioToken [4]: the contact flags -- the sensor of a toe having landed.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"load_topic",    ParamMutability::ConstructionOnly, "ProprioToken [4]: the foot loads.", ParamValue{std::string("reality.proprio.foot_load")}},
        {"gait_action_topics", ParamMutability::ConstructionOnly, "The gait's 12 action topics (leg-major hip1, hip2, knee): the pose the sequencer starts from when it engages.", ParamValue{std::vector<std::string>{}}},
        {"output_topic",  ParamMutability::ConstructionOnly, "ProprioToken [engage, u x 12] for MotorEPMv2's pose_topic.", ParamValue{std::string("pose.targets")}},
        {"belly_topic",   ParamMutability::ConstructionOnly, "ProprioToken [m]: the belly clearance; the sequencer engages only with the body standing (>= engage_min_belly_m). \"\" = no check.", ParamValue{std::string("reality.proprio.belly_m")}},
        {"engage_min_belly_m", ParamMutability::HotMutable, "The belly clearance a standing body shows (m).", ParamValue{0.045}},
        {"engage_min_tick", ParamMutability::HotMutable, "No engage before this tick (the gait's warm-up).", ParamValue{int64_t{200}}},
        {"engage_min_contacts", ParamMutability::HotMutable, "Toes in contact needed at the moment of engaging.", ParamValue{int64_t{3}}},
        {"phase_max_ticks", ParamMutability::HotMutable, "A LIFT or SWING not arrived by then proceeds anyway (counted).", ParamValue{int64_t{150}}},
        {"stand_h_m",     ParamMutability::HotMutable, "The body's height above the planted toes' mean ground (m): planted toes are held there in the body frame.", ParamValue{0.085}},
        {"elev_max_m",    ParamMutability::HotMutable, "The most a planted toe may be held above the others' mean (a riser higher than this pitches the body).", ParamValue{0.08}},
        {"reach_max_m",   ParamMutability::HotMutable, "A toe target is kept within this horizontal distance of the leg's hip (the full leg reaches ~0.143 at the standing height).", ParamValue{0.14}},
        {"bear_tau_ticks", ParamMutability::HotMutable, "The leaky estimate of each toe's bearing from the contact flag (ticks); the lift-off transient on the other legs does not reach the sequence.", ParamValue{8.0}},
        {"bear_on",       ParamMutability::HotMutable, "The estimate above which a toe bears ...", ParamValue{0.6}},
        {"bear_off",      ParamMutability::HotMutable, "... and below which it no longer does (hysteresis).", ParamValue{0.3}},
        {"touch_ticks",   ParamMutability::HotMutable, "A landing needs the raw flag on this many ticks in a row.", ParamValue{int64_t{3}}},
        {"touch_load",    ParamMutability::HotMutable, "... and, with a load topic, at least this fraction of the body's weight (0 = the flag alone; 0.05 missed landings whose toe had touched but not yet taken weight).", ParamValue{0.0}},
        {"creep_v_mps",   ParamMutability::HotMutable, "A planted toe without contact creeps down at this rate until it bears.", ParamValue{0.02}},
        {"creep_max_m",   ParamMutability::HotMutable, "... by at most this (m).", ParamValue{0.02}},
        {"creep_relax_v_mps", ParamMutability::HotMutable, "... and relaxes back at this rate while it bears.", ParamValue{0.005}},
        {"elev_deadband_m", ParamMutability::HotMutable, "A toe's ground elevation against the others (from the terrain patch) smaller than this is noise (kept 0); the memory's heights scatter by 2-3 cm on flat ground.", ParamValue{0.03}},
        {"elev_tau_ticks", ParamMutability::HotMutable, "EMA of each toe's ground elevation from the patch.", ParamValue{100.0}},
        {"air_ticks_min", ParamMutability::HotMutable, "A planted toe counts as airborne (placed first) after this many ticks without contact -- long enough for the creep to have tried.", ParamValue{int64_t{100}}},
        {"advance_m",     ParamMutability::HotMutable, "Each SHIFT carries the body forward by this (m) before the leg may lift (a quarter stride).", ParamValue{0.015}},
        {"centre_gain",   ParamMutability::HotMutable, "During SHIFT the body also moves this fraction of its offset from the four toes' centroid per tick's bound.", ParamValue{0.5}},
        {"margin_extra_m", ParamMutability::HotMutable, "The shift aims for margin_min_m plus this (m).", ParamValue{0.005}},
        {"neutral_r_m",   ParamMutability::HotMutable, "The stance: each toe's neutral place is this far from its hip along the leg's heading (m); the legs step toward it from wherever the gait left them.", ParamValue{0.10}},
        {"sector_half_rad", ParamMutability::HotMutable, "A toe target stays within this angle of the leg's neutral heading about its hip (rad); 0.6 keeps a toe outboard of the body.", ParamValue{0.6}},
        {"reach_min_m",   ParamMutability::HotMutable, "... and no closer to the hip than this (m).", ParamValue{0.07}},
        {"retreat_m",     ParamMutability::HotMutable, "A lowering toe out of reach moves this far toward its hip and tries again.", ParamValue{0.01}},
        {"unreach_tol_m", ParamMutability::HotMutable, "A toe the IK leaves this far from its target is out of reach: a lowering toe is taken as landed where it is.", ParamValue{0.015}},
        {"engage",        ParamMutability::HotMutable, "1 = the sequencer owns the legs (ramped in over engage_ramp_ticks); 0 = the rhythm (the default; byte-identical).", ParamValue{0.0}},
        {"engage_ramp_ticks", ParamMutability::HotMutable, "Ticks over which the blend ramps in or out.", ParamValue{int64_t{50}}},
        {"shift_v_mps",   ParamMutability::HotMutable, "The body's advance over the planted toes during SHIFT (m/s).", ParamValue{0.03}},
        {"shift_max_m",   ParamMutability::HotMutable, "The most one SHIFT may advance before a leg is lifted anyway (the model's safest).", ParamValue{0.05}},
        {"margin_min_m",  ParamMutability::HotMutable, "The lift margin (m) the chosen leg needs before it may lift.", ParamValue{0.005}},
        {"lift_clear_m",  ParamMutability::HotMutable, "Clearance above the higher of the ground under the toe and at its landing point.", ParamValue{0.03}},
        {"lift_v_mps",    ParamMutability::HotMutable, "The toe's rise rate.", ParamValue{0.10}},
        {"swing_v_mps",   ParamMutability::HotMutable, "The toe's forward rate in the air.", ParamValue{0.12}},
        {"lower_v_mps",   ParamMutability::HotMutable, "The toe's descent rate.", ParamValue{0.06}},
        {"stride_m",      ParamMutability::HotMutable, "A toe lands this far ahead of the place it stood at engage... per step: the landing is neutral + stride/2.", ParamValue{0.06}},
        {"sink_max_m",    ParamMutability::HotMutable, "A lowering toe this far below the predicted ground without contact is taken as landed (the memory's heights scatter by 2 cm on the flat).", ParamValue{0.03}},
        {"tilt_disengage", ParamMutability::HotMutable, "Below this vertical component of the body's up the legs go back to the rhythm.", ParamValue{0.75}},
        {"dt_s",          ParamMutability::HotMutable, "The tick (s).", ParamValue{0.02}},
        {"ik_iters",      ParamMutability::HotMutable, "Damped least-squares iterations per tick per leg.", ParamValue{int64_t{4}}},
        {"ik_damping_m",  ParamMutability::HotMutable, "The IK's damping (m).", ParamValue{0.01}},
        {"ik_step_max_rad", ParamMutability::HotMutable, "The most a joint moves per tick (rad).", ParamValue{0.12}},
        {"ik_tol_m",      ParamMutability::HotMutable, "A toe within this of its target has arrived.", ParamValue{0.004}},
    };
}

ParamMap PoseSequencer::current_params() const {
    return {{"model_topic", ParamValue{model_topic_}}, {"support_topic", ParamValue{support_topic_}}, {"patch_topic", ParamValue{patch_topic_}},
            {"feet_xz_topic", ParamValue{feet_xz_topic_}}, {"feet_y_topic", ParamValue{feet_y_topic_}}, {"up_topic", ParamValue{up_topic_}},
            {"contact_topic", ParamValue{contact_topic_}}, {"load_topic", ParamValue{load_topic_}}, {"gait_action_topics", ParamValue{gait_action_topics_}}, {"output_topic", ParamValue{output_topic_}},
            {"engage", ParamValue{engage_}}, {"engage_ramp_ticks", ParamValue{int64_t(engage_ramp_ticks_)}}, {"belly_topic", ParamValue{belly_topic_}},
            {"stand_h_m", ParamValue{stand_h_m_}}, {"sector_half_rad", ParamValue{sector_half_rad_}}, {"neutral_r_m", ParamValue{neutral_r_m_}}, {"centre_gain", ParamValue{centre_gain_}}, {"advance_m", ParamValue{advance_m_}}, {"margin_extra_m", ParamValue{margin_extra_m_}}, {"creep_v_mps", ParamValue{creep_v_}}, {"bear_tau_ticks", ParamValue{bear_tau_ticks_}}, {"bear_on", ParamValue{bear_on_}}, {"bear_off", ParamValue{bear_off_}}, {"touch_ticks", ParamValue{int64_t(touch_ticks_)}}, {"touch_load", ParamValue{touch_load_}}, {"creep_max_m", ParamValue{creep_max_m_}}, {"creep_relax_v_mps", ParamValue{creep_relax_v_}}, {"elev_deadband_m", ParamValue{elev_deadband_m_}}, {"elev_tau_ticks", ParamValue{elev_tau_ticks_}}, {"air_ticks_min", ParamValue{int64_t(air_ticks_min_)}}, {"reach_min_m", ParamValue{reach_min_m_}}, {"retreat_m", ParamValue{retreat_m_}}, {"elev_max_m", ParamValue{elev_max_m_}}, {"reach_max_m", ParamValue{reach_max_m_}}, {"unreach_tol_m", ParamValue{unreach_tol_m_}},
            {"engage_min_belly_m", ParamValue{engage_min_belly_m_}}, {"engage_min_tick", ParamValue{int64_t(engage_min_tick_)}}, {"engage_min_contacts", ParamValue{int64_t(engage_min_contacts_)}}, {"phase_max_ticks", ParamValue{int64_t(phase_max_ticks_)}}, {"shift_v_mps", ParamValue{shift_v_}}, {"shift_max_m", ParamValue{shift_max_m_}},
            {"margin_min_m", ParamValue{margin_min_m_}}, {"lift_clear_m", ParamValue{lift_clear_m_}}, {"lift_v_mps", ParamValue{lift_v_}}, {"swing_v_mps", ParamValue{swing_v_}},
            {"lower_v_mps", ParamValue{lower_v_}}, {"stride_m", ParamValue{stride_m_}}, {"sink_max_m", ParamValue{sink_max_m_}}, {"tilt_disengage", ParamValue{tilt_disengage_}},
            {"dt_s", ParamValue{dt_}}, {"ik_iters", ParamValue{int64_t(ik_iters_)}}, {"ik_damping_m", ParamValue{ik_damping_m_}}, {"ik_step_max_rad", ParamValue{ik_step_max_rad_}}, {"ik_tol_m", ParamValue{ik_tol_m_}}};
}

void PoseSequencer::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("PoseSequencer requires a non-null Bus");
    for (auto pr : std::initializer_list<std::pair<const char*, std::string*>>{
            {"model_topic", &model_topic_}, {"support_topic", &support_topic_}, {"patch_topic", &patch_topic_}, {"feet_xz_topic", &feet_xz_topic_},
            {"feet_y_topic", &feet_y_topic_}, {"up_topic", &up_topic_}, {"contact_topic", &contact_topic_}, {"load_topic", &load_topic_}, {"output_topic", &output_topic_}, {"belly_topic", &belly_topic_}})
        apply_param(params, pr.first, [&](auto const& v){ *pr.second = get_string(v, pr.first); });
    apply_param(params, "gait_action_topics", [&](auto const& v){ if (auto p = std::get_if<std::vector<std::string>>(&v)) gait_action_topics_ = *p; });
    for (auto const& k : {"engage", "engage_ramp_ticks", "stand_h_m", "sector_half_rad", "neutral_r_m", "reach_min_m", "retreat_m", "centre_gain", "advance_m", "margin_extra_m", "bear_tau_ticks", "bear_on", "bear_off", "touch_ticks", "touch_load", "creep_v_mps", "creep_max_m", "creep_relax_v_mps", "elev_deadband_m", "elev_tau_ticks", "air_ticks_min", "elev_max_m", "reach_max_m", "unreach_tol_m", "engage_min_belly_m", "engage_min_tick", "engage_min_contacts", "phase_max_ticks", "shift_v_mps", "shift_max_m", "margin_min_m", "lift_clear_m", "lift_v_mps", "swing_v_mps", "lower_v_mps",
                          "stride_m", "sink_max_m", "tilt_disengage", "dt_s", "ik_iters", "ik_damping_m", "ik_step_max_rad", "ik_tol_m"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    auto sub = [&](std::string const& topic, std::string which, int idx) {
        if (topic.empty()) return;
        sub_ids_.push_back(bus_->subscribe(topic, SubscriptionKind::Direct, [this, which, idx](std::string_view, MessagePtr p){ handle(which, idx, p); }));
    };
    sub(model_topic_, "model", 0); sub(support_topic_, "support", 0); sub(patch_topic_, "patch", 0); sub(feet_xz_topic_, "xz", 0); sub(feet_y_topic_, "y", 0);
    sub(up_topic_, "up", 0); sub(contact_topic_, "contact", 0); sub(load_topic_, "load", 0); sub(belly_topic_, "belly", 0);
    for (int i = 0; i < int(gait_action_topics_.size()) && i < 12; ++i) sub(gait_action_topics_[size_t(i)], "gait", i);
}

void PoseSequencer::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "engage")           engage_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "engage_ramp_ticks") engage_ramp_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "stand_h_m")        stand_h_m_ = std::max(0.02, get_double(value, k));
    else if (k == "sector_half_rad")  sector_half_rad_ = std::clamp(get_double(value, k), 0.1, 3.0);
    else if (k == "neutral_r_m")      neutral_r_m_ = std::max(0.03, get_double(value, k));
    else if (k == "centre_gain")      centre_gain_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "advance_m")        advance_m_ = std::max(0.0, get_double(value, k));
    else if (k == "bear_tau_ticks")   bear_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "bear_on")          bear_on_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "bear_off")         bear_off_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "touch_ticks")      touch_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "touch_load")       touch_load_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_v_mps")      creep_v_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_max_m")      creep_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "creep_relax_v_mps") creep_relax_v_ = std::max(0.0, get_double(value, k));
    else if (k == "elev_deadband_m")  elev_deadband_m_ = std::max(0.0, get_double(value, k));
    else if (k == "elev_tau_ticks")   elev_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "air_ticks_min")    air_ticks_min_ = std::max(1, int(get_double(value, k)));
    else if (k == "margin_extra_m")   margin_extra_m_ = std::max(0.0, get_double(value, k));
    else if (k == "reach_min_m")      reach_min_m_ = std::max(0.0, get_double(value, k));
    else if (k == "retreat_m")        retreat_m_ = std::max(1e-3, get_double(value, k));
    else if (k == "elev_max_m")       elev_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "reach_max_m")      reach_max_m_ = std::max(0.03, get_double(value, k));
    else if (k == "unreach_tol_m")    unreach_tol_m_ = std::max(1e-3, get_double(value, k));
    else if (k == "engage_min_belly_m") engage_min_belly_m_ = get_double(value, k);
    else if (k == "engage_min_tick")  engage_min_tick_ = std::max(0, int(get_double(value, k)));
    else if (k == "engage_min_contacts") engage_min_contacts_ = std::clamp(int(get_double(value, k)), 0, 4);
    else if (k == "phase_max_ticks")  phase_max_ticks_ = std::max(1, int(get_double(value, k)));
    else if (k == "shift_v_mps")      shift_v_ = std::max(0.0, get_double(value, k));
    else if (k == "shift_max_m")      shift_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "margin_min_m")     margin_min_m_ = get_double(value, k);
    else if (k == "lift_clear_m")     lift_clear_m_ = std::max(0.0, get_double(value, k));
    else if (k == "lift_v_mps")       lift_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "swing_v_mps")      swing_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "lower_v_mps")      lower_v_ = std::max(1e-3, get_double(value, k));
    else if (k == "stride_m")         stride_m_ = std::max(0.0, get_double(value, k));
    else if (k == "sink_max_m")       sink_max_m_ = std::max(0.0, get_double(value, k));
    else if (k == "tilt_disengage")   tilt_disengage_ = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "dt_s")             dt_ = std::max(1e-3, get_double(value, k));
    else if (k == "ik_iters")         ik_iters_ = std::clamp(int(get_double(value, k)), 1, 32);
    else if (k == "ik_damping_m")     ik_damping_m_ = std::max(1e-4, get_double(value, k));
    else if (k == "ik_step_max_rad")  ik_step_max_rad_ = std::max(1e-3, get_double(value, k));
    else if (k == "ik_tol_m")         ik_tol_m_ = std::max(1e-4, get_double(value, k));
}

void PoseSequencer::handle(std::string const& which, int idx, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    if (which == "gait") {
        auto a = std::dynamic_pointer_cast<const ActionOut>(p);
        if (!a || idx < 0 || idx >= 12) return;
        gait_u_[size_t(idx)] = a->accel; have_gait_ = true; return;
    }
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    auto const& v = pt->values;
    if (which == "model" && v.size() >= kModelFloats) {
        auto vec = [&](int k) { return Vec3f(v[k], v[k + 1], v[k + 2]); };
        for (int i = 0; i < 4; ++i) {
            const int b = 27 * i;
            auto& a = anchors_[size_t(i)];
            a.hip1_world = vec(b); a.hip2_world = vec(b + 3); a.knee_world = vec(b + 6);
            a.coxa_rest_origin = vec(b + 9); a.upper_rest_origin = vec(b + 12); a.lower_rest_origin = vec(b + 15);
            a.hip2_axis = vec(b + 18); a.knee_axis = vec(b + 21); toe_off_[size_t(i)] = vec(b + 24);
        }
        const int m = 108;
        hip1_rest_ = v[m]; hip1_range_ = v[m + 1]; hip2_rest_ = v[m + 2]; hip2_range_ = v[m + 3];
        knee_rest_ = v[m + 4]; knee_fold_ = v[m + 5]; knee_hyper_ = v[m + 6]; knee_widening_ = v[m + 7] > 0.5f;
        have_model_ = true;
    }
    else if (which == "support" && v.size() >= 26) { for (int i = 0; i < 26; ++i) support_[size_t(i)] = v[i]; have_support_ = true; }
    else if (which == "patch" && v.size() >= 4) {
        const int N = int(v[0]); if (N < 2 || v.size() < 4 + 2 * N * N) return;
        N_ = N; cell_ = std::max(1e-3, double(v[1])); patch_.assign(v.data(), v.data() + 4 + 2 * N * N); have_patch_ = true;
    }
    else if (which == "xz" && v.size() >= 8) { for (int i = 0; i < 8; ++i) xz_[size_t(i)] = v[i]; have_xz_ = true; }
    else if (which == "y" && v.size() >= 4) { for (int i = 0; i < 4; ++i) y_[size_t(i)] = v[i]; have_y_ = true; }
    else if (which == "contact" && v.size() >= 4) { for (int i = 0; i < 4; ++i) contact_[size_t(i)] = v[i]; have_contact_ = true; }
    else if (which == "load" && v.size() >= 4) { for (int i = 0; i < 4; ++i) load_[size_t(i)] = v[i]; have_load_ = true; }
    else if (which == "belly" && v.size() >= 1) { belly_ = v[0]; have_belly_ = true; }
    else if (which == "up" && v.size() >= 3) { const double n = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]); if (n > 0.5) { up_ = {v[0] / n, v[1] / n, v[2] / n}; have_up_ = true; } }
}

// ---- the kinematics ----
Vec3f PoseSequencer::toe_of(int leg, const std::array<double, 3>& t) const {
    const auto pose = body::fk_leg(anchors_[size_t(leg)], t[0], t[1], t[2], 0.0);
    return pose.lower.xform(toe_off_[size_t(leg)]);
}
std::array<double, 3> PoseSequencer::angles_of_u(int leg, const std::array<double, 3>& u) const {
    (void)leg;
    const double kr = knee_widening_ ? (u[2] >= 0.0 ? knee_fold_ : knee_hyper_) : hip2_range_;
    return {u[0] * hip1_range_ + hip1_rest_, u[1] * hip2_range_ + hip2_rest_, u[2] * kr + knee_rest_};
}
std::array<double, 3> PoseSequencer::u_of_angles(int leg, const std::array<double, 3>& t) const {
    (void)leg;
    const double dk = t[2] - knee_rest_;
    const double kr = knee_widening_ ? (dk >= 0.0 ? knee_fold_ : knee_hyper_) : hip2_range_;
    return {std::clamp((t[0] - hip1_rest_) / hip1_range_, -1.0, 1.0), std::clamp((t[1] - hip2_rest_) / hip2_range_, -1.0, 1.0), std::clamp(dk / kr, -1.0, 1.0)};
}
// one damped least-squares step toward `target`: dt = J^T (J J^T + lambda^2 I)^-1 e, J by central differences, the step bounded
bool PoseSequencer::ik_step(int leg, const Vec3f& target, std::array<double, 3>& t) const {
    const Vec3f cur = toe_of(leg, t);
    const Vec3f e = sub(target, cur);
    const double en = len(e);
    if (en < ik_tol_m_) return true;
    double J[3][3]; const double h = 1e-3;
    for (int j = 0; j < 3; ++j) {
        auto tp = t, tm = t; tp[size_t(j)] += h; tm[size_t(j)] -= h;
        const Vec3f d = sub(toe_of(leg, tp), toe_of(leg, tm));
        J[0][j] = d.x / (2 * h); J[1][j] = d.y / (2 * h); J[2][j] = d.z / (2 * h);
    }
    // A = J J^T + lambda^2 I (3x3), solve A y = e, dt = J^T y
    const double l2 = ik_damping_m_ * ik_damping_m_;
    double A[9];
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) { double s = 0; for (int k = 0; k < 3; ++k) s += J[r][k] * J[c][k]; A[r * 3 + c] = s + (r == c ? l2 : 0.0); }
    const double det = A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) + A[2] * (A[3] * A[7] - A[4] * A[6]);
    if (std::fabs(det) < 1e-18) return false;
    const double id = 1.0 / det;
    const double Ai[9] = { (A[4] * A[8] - A[5] * A[7]) * id, -(A[1] * A[8] - A[2] * A[7]) * id,  (A[1] * A[5] - A[2] * A[4]) * id,
                          -(A[3] * A[8] - A[5] * A[6]) * id,  (A[0] * A[8] - A[2] * A[6]) * id, -(A[0] * A[5] - A[2] * A[3]) * id,
                           (A[3] * A[7] - A[4] * A[6]) * id, -(A[0] * A[7] - A[1] * A[6]) * id,  (A[0] * A[4] - A[1] * A[3]) * id };
    const double ev[3] = {e.x, e.y, e.z};
    double y[3]; for (int r = 0; r < 3; ++r) y[r] = Ai[r * 3] * ev[0] + Ai[r * 3 + 1] * ev[1] + Ai[r * 3 + 2] * ev[2];
    double dt[3]; double mx = 0.0;
    for (int j = 0; j < 3; ++j) { dt[j] = J[0][j] * y[0] + J[1][j] * y[1] + J[2][j] * y[2]; mx = std::max(mx, std::fabs(dt[j])); }
    const double sc = mx > ik_step_max_rad_ ? ik_step_max_rad_ / mx : 1.0;
    for (int j = 0; j < 3; ++j) t[size_t(j)] += dt[j] * sc;
    // joint limits, in action units
    const auto u = u_of_angles(leg, t); t = angles_of_u(leg, u);
    return false;
}

bool PoseSequencer::patch_h(double fwd, double lat, double& h) const {
    h = 0.0;
    if (!have_patch_) return false;
    const int ci = int(std::floor(fwd / cell_)) + N_ / 2, cj = int(std::floor(lat / cell_)) + N_ / 2;
    double sw = 0.0, swh = 0.0;
    for (int di = -1; di <= 1; ++di) for (int dj = -1; dj <= 1; ++dj) {
        const int i = ci + di, j = cj + dj;
        if (i < 0 || j < 0 || i >= N_ || j >= N_) continue;
        const float hv = patch_[size_t(4 + i * N_ + j)]; const double w = patch_[size_t(4 + N_ * N_ + i * N_ + j)];
        if (hv <= -98.0f || w <= 0.0) continue;
        sw += w; swh += w * hv;
    }
    if (sw <= 0.0) return false;
    h = swh / sw; return true;
}

// ---- the sequence ----
// the signed distance of (cx, cz) inside the triangle p (+ inside) and the inward unit normal of the edge nearest it
double PoseSequencer::tri_margin(const std::array<std::array<double, 2>, 3>& p, double cx, double cz, double& nx, double& nz) {
    // orient counter-clockwise in the (x, z) plane
    std::array<std::array<double, 2>, 3> q = p;
    const double area = (q[1][0] - q[0][0]) * (q[2][1] - q[0][1]) - (q[1][1] - q[0][1]) * (q[2][0] - q[0][0]);
    if (area < 0) std::swap(q[1], q[2]);
    double best = 1e9; nx = 0; nz = 0;
    for (int i = 0; i < 3; ++i) {
        const auto& a = q[size_t(i)]; const auto& b = q[size_t((i + 1) % 3)];
        const double ex = b[0] - a[0], ez = b[1] - a[1], L = std::hypot(ex, ez); if (L < 1e-9) continue;
        const double d = (ex * (cz - a[1]) - ez * (cx - a[0])) / L;   // + on the interior side of a CCW edge
        if (d < best) { best = d; nx = -ez / L; nz = ex / L; }         // the inward normal of that edge
    }
    return best;
}

// the leg's workspace: a sector about its neutral heading (hip1 -> hip2) from its hip, radius in [reach_min, reach_max]
bool PoseSequencer::sector_clamp(int leg, Vec3f& p) const {
    const Vec3f hip = hip1_of(leg); const Vec3f h2 = anchors_[size_t(leg)].hip2_world;
    const double hx = h2.x - hip.x, hz = h2.z - hip.z, hn = std::hypot(hx, hz);
    if (hn < 1e-6) return false;
    const double ux = hx / hn, uz = hz / hn;                      // the neutral heading
    double dx = p.x - hip.x, dz = p.z - hip.z; const double r = std::hypot(dx, dz);
    if (r < 1e-9) { p.x = float(hip.x + ux * reach_min_m_); p.z = float(hip.z + uz * reach_min_m_); return true; }
    const double ang = std::atan2(dx * uz - dz * ux, dx * ux + dz * uz);   // signed angle from the heading
    const double a = std::clamp(ang, -sector_half_rad_, sector_half_rad_), rr = std::clamp(r, reach_min_m_, reach_max_m_);
    const bool clamped = std::fabs(a - ang) > 1e-9 || std::fabs(rr - r) > 1e-9;
    if (!clamped) return false;
    const double c = std::cos(a), sn = std::sin(a);               // the heading rotated by a, in the x-z plane
    const double vx = ux * c + uz * sn, vz = uz * c - ux * sn;
    const double nx_ = hip.x + vx * rr, nz_ = hip.z + vz * rr;
    const bool moved = std::hypot(nx_ - p.x, nz_ - p.z) > 1e-4;    // blocked means the clamp took more than a tenth of a millimetre back
    p.x = float(nx_); p.z = float(nz_);
    return moved;
}

void PoseSequencer::engage_now(uint64_t tick_id) {
    for (int i = 0; i < 4; ++i) {
        std::array<double, 3> u{0.0, 0.0, 0.0};
        if (have_gait_) u = {gait_u_[size_t(3 * i)], gait_u_[size_t(3 * i + 1)], gait_u_[size_t(3 * i + 2)]};
        angles_[size_t(i)] = angles_of_u(i, u);
        const Vec3f toe = toe_of(i, angles_[size_t(i)]);
        // the stance: this toe's place in the plane, kept within reach of its hip, at the standing height
        Vec3f t(toe.x, float(-stand_h_m_), toe.z);
        if (sector_clamp(i, t)) {   // a toe at the edge of its reach gets a centimetre of room, so the first shift is not blocked by it
            const Vec3f hip = hip1_of(i); const double dx = hip.x - t.x, dz = hip.z - t.z, dn = std::max(1e-6, std::hypot(dx, dz));
            t.x += float(dx / dn * 0.01); t.z += float(dz / dn * 0.01);
        }
        toe_tgt_[size_t(i)] = t;
        // the neutral place: neutral_r along the leg's heading from its hip, at the standing height
        const Vec3f hip = hip1_of(i), h2 = anchors_[size_t(i)].hip2_world;
        const double hx = h2.x - hip.x, hz = h2.z - hip.z, hn = std::max(1e-6, std::hypot(hx, hz));
        neutral_[size_t(i)] = Vec3f(float(hip.x + hx / hn * neutral_r_m_), float(-stand_h_m_), float(hip.z + hz / hn * neutral_r_m_));
        elev_[size_t(i)] = 0.0; ground_h_[size_t(i)] = -stand_h_m_; creep_[size_t(i)] = 0.0; air_ticks_[size_t(i)] = 0;
    }
    shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ticks_ = 0; leg_ = choose_leg(); phase_ = Phase::Shift; engaged_at_ = int64_t(tick_id);
}

// the planted toes' heights: stand_h below the body plus each toe's relative elevation, recentred on their mean
void PoseSequencer::hold_stance() {
    // each toe's ground elevation from the memory: the patch under its place against the mean under the others' places
    if (have_patch_) {
        double hs[4]; bool ok[4]; int n_ok = 0; double sum = 0.0;
        for (int i = 0; i < 4; ++i) { ok[i] = patch_h(toe_tgt_[size_t(i)].z, toe_tgt_[size_t(i)].x, hs[i]); if (ok[i]) { sum += hs[i]; ++n_ok; } }
        if (n_ok >= 2) {
            const double a = 1.0 / elev_tau_ticks_;
            for (int i = 0; i < 4; ++i) {
                if (!ok[i]) continue;
                const double others = (sum - hs[i]) / std::max(1, n_ok - 1);
                double e = hs[i] - others; if (std::fabs(e) < elev_deadband_m_) e = 0.0;
                elev_[size_t(i)] += a * (std::clamp(e, -0.1, 0.15) - elev_[size_t(i)]);
            }
        }
    }
    double mean = 0.0; int n = 0;
    for (int i = 0; i < 4; ++i) if (i != leg_ || phase_ == Phase::Shift) { mean += elev_[size_t(i)]; ++n; }
    if (n > 0) mean /= n;
    for (int i = 0; i < 4; ++i) {
        if (i == leg_ && phase_ != Phase::Shift) { creep_[size_t(i)] = 0.0; air_ticks_[size_t(i)] = 0; continue; }   // the stepping toe is the machine's
        // the load-seeking creep: down while the flag is off, back while it bears
        if (have_contact_) {
            if (!bearing(i)) { creep_[size_t(i)] = std::min(creep_max_m_, creep_[size_t(i)] + creep_v_ * dt_); ++air_ticks_[size_t(i)]; }
            else { creep_[size_t(i)] = std::max(0.0, creep_[size_t(i)] - creep_relax_v_ * dt_); air_ticks_[size_t(i)] = 0; }
        }
        toe_tgt_[size_t(i)].y = float(-stand_h_m_ + std::clamp(elev_[size_t(i)] - mean, -elev_max_m_, elev_max_m_) - creep_[size_t(i)]);
    }
}

// the leg whose toe is furthest behind its neutral place steps next (the crawl's order emerges); among equals, the model's
// safest
int PoseSequencer::choose_leg() const {
    if (have_contact_) for (int i = 0; i < 4; ++i) if (!bearing(i) && air_ticks_[size_t(i)] >= air_ticks_min_) return i;   // a toe the creep could not bring to ground is placed first
    // the toe furthest from its neutral place steps next -- behind it (the body has advanced), or tucked in or out
    // (the body has swayed); the crawl's order emerges from the geometry, and the model's margin breaks ties
    int best = -1; double best_score = -1e9;
    for (int i = 0; i < 4; ++i) {
        const double dx = double(neutral_[size_t(i)].x - toe_tgt_[size_t(i)].x), dz = double(neutral_[size_t(i)].z - toe_tgt_[size_t(i)].z);
        (void)dx;
        const double away = std::max(0.0, dz);   // behind its place (a sway moves every toe sideways alike; a toe AHEAD of its place is fine where it is)
        const double margin = have_support_ ? double(support_[size_t(10 + 5 * i)]) : 0.0;
        const double age = double(int64_t(tick_now_) - last_step_tick_[size_t(i)]) * 1e-6;   // among equals, the leg that stepped longest ago
        const double score = away + 0.2 * margin + age;
        score_[size_t(i)] = score;
        if (score > best_score) { best_score = score; best = i; }
    }
    return best;
}

void PoseSequencer::step_machine() {
    ++phase_ticks_;
    const double adv = shift_v_ * dt_;
    switch (phase_) {
    case Phase::Idle: break;
    case Phase::Shift: {
        // the body's move: into the triangle the other toes make, by the margin still missing, and toward the toes' centroid
        const double cx = have_support_ ? double(support_[3]) : 0.0, cz = have_support_ ? double(support_[2]) : 0.0;   // the model's centre of mass (lat = x, fwd = z)
        std::array<std::array<double, 2>, 3> tri{}; int k = 0; double mx = 0, mz = 0;
        for (int i = 0; i < 4; ++i) { mx += toe_tgt_[size_t(i)].x * 0.25; mz += toe_tgt_[size_t(i)].z * 0.25; if (i != leg_ && k < 3) { tri[size_t(k)] = {double(toe_tgt_[size_t(i)].x), double(toe_tgt_[size_t(i)].z)}; ++k; } }
        double nx = 0, nz = 0; const double m_local = tri_margin(tri, cx, cz, nx, nz);
        const double model_margin = have_support_ ? double(support_[size_t(10 + 5 * leg_)]) : 1.0;
        const int bearing = have_support_ ? int(support_[1] + 0.5f) : 4;
        // the margin still missing, by the local geometry and -- when all four toes bear and the model disagrees -- by the model
        double need = std::max(0.0, margin_min_m_ + margin_extra_m_ - m_local);
        if (bearing >= 4) need = std::max(need, margin_min_m_ + margin_extra_m_ - model_margin);
        const double fwd_left = std::max(0.0, advance_m_ - fwd_acc_);
        const double cg = centre_off_ ? 0.0 : centre_gain_;   // the centroid's pull yields when it has cancelled the margin's (stagnation)
        double dx = need * nx + cg * (mx - cx), dz = need * nz + cg * (mz - cz) + std::min(adv, fwd_left);
        {   // the advance yields to the margin: if the move would leave the weight short of the support that will remain, drop it
            double dn0 = std::hypot(dx, dz); double sx = dx, sz = dz; if (dn0 > adv) { sx *= adv / dn0; sz *= adv / dn0; }
            std::array<std::array<double, 2>, 3> tri2 = tri; for (auto& q : tri2) { q[0] -= sx; q[1] -= sz; }
            double n2x, n2z; const double m_after = tri_margin(tri2, cx, cz, n2x, n2z);
            if (fwd_left > 0.0 && m_after < m_local && m_after < margin_min_m_ + margin_extra_m_) { dz -= std::min(adv, fwd_left); fwd_blocked_ = true; }
        }
        const double dn = std::hypot(dx, dz);
        if (dn > adv) { dx *= adv / dn; dz *= adv / dn; }
        shift_dx_ = dx; shift_dz_ = dz; fwd_acc_ += std::max(0.0, dz);
        // a shift that has stopped moving (the margin's pull cancels the advance) has no advance to give this leg
        if (std::hypot(dx, dz) < 0.1 * adv) { if (++still_ticks_ >= 10) fwd_blocked_ = true; } else still_ticks_ = 0;
        bool blocked = false; blocked_leg_ = -1;
        for (int i = 0; i < 4; ++i) { toe_tgt_[size_t(i)].x -= float(dx); toe_tgt_[size_t(i)].z -= float(dz); if (sector_clamp(i, toe_tgt_[size_t(i)])) { blocked = true; if (blocked_leg_ < 0) blocked_leg_ = i; } }   // the toes move the other way, as far as they reach
        shift_acc_ += std::hypot(dx, dz); ++shift_ticks_;
        if (blocked) ++shift_blocked_;
        // may the leg go?  By the geometry the shift is steering on; the model vetoes only when all four toes bear (with
        // fewer, a flickering flag makes every other leg's margin the floor, and the shift would run to its bound)
        const bool safe_now = m_local >= margin_min_m_ && (bearing < 4 || model_margin >= margin_min_m_);
        bool go = safe_now && (fwd_acc_ >= advance_m_ || fwd_blocked_ || blocked);
        if (go && fwd_acc_ < advance_m_) ++steps_no_advance_;
        // a shift standing still without the leg being safe: first let the margin's pull act alone, then treat as spent
        bool spent = blocked || shift_acc_ >= shift_max_m_;
        if (!go && !safe_now && still_ticks_ >= 10) { if (!centre_off_) { centre_off_ = true; still_ticks_ = 0; } else spent = true; }
        if (!go && spent) {
            // the body can move no further for this leg: take the toe that blocks it if ITS margin allows, else the leg the
            // model rates safest if that one's does; a leg without a margin is never lifted -- better to stand than to fall
            auto margin_of_leg = [&](int i) { return have_support_ ? double(support_[size_t(10 + 5 * i)]) : 1.0; };
            int cand = (blocked && blocked_leg_ >= 0) ? blocked_leg_ : -1;
            if (cand < 0 || margin_of_leg(cand) < margin_min_m_) {
                // the safest leg among those that NEED a step (behind their place): a toe already ahead has nothing to gain
                double bm = -1e9; cand = -1;
                for (int i = 0; i < 4; ++i) { const bool needs = neutral_[size_t(i)].z - toe_tgt_[size_t(i)].z > 0.0; if (needs && margin_of_leg(i) > bm) { bm = margin_of_leg(i); cand = i; } }
                if (cand < 0) for (int i = 0; i < 4; ++i) if (margin_of_leg(i) > bm) { bm = margin_of_leg(i); cand = i; }
                if (cand >= 0 && bm < margin_min_m_) cand = -1;
            }
            if (cand >= 0) { if (cand != leg_) leg_ = cand; ++shift_exhausted_; go = true; }
            else ++deadlock_ticks_;
        }
        if (go) {
            const Vec3f cur = toe_tgt_[size_t(leg_)];
            // the landing point: the neutral place plus half a stride ahead, kept within reach of the hip
            land_ = Vec3f(neutral_[size_t(leg_)].x, cur.y, float(neutral_[size_t(leg_)].z + stride_m_ * 0.5));
            {   // a centimetre inside the reach, so the swing can finish and the toe can still lower
                const Vec3f hip = hip1_of(leg_); const double dx = land_.x - hip.x, dz = land_.z - hip.z, r = std::hypot(dx, dz), rm = reach_max_m_ - 0.01;
                if (r > rm) { land_.x = float(hip.x + dx * rm / r); land_.z = float(hip.z + dz * rm / r); }
                sector_clamp(leg_, land_);
            }
            // the ground: under the toe now and at the landing, from the memory where it has it, else the stance's own height
            const double stance_h = gravity_h(Vec3f(cur.x, float(-stand_h_m_), cur.z));
            double h_here = stance_h, h_land = stance_h, hp;
            if (patch_h(cur.z, cur.x, hp)) h_here = hp;             // the patch's frame: fwd = z, lat = x, as TerrainMemory stores the toes
            if (patch_h(land_.z, land_.x, hp)) h_land = hp;
            land_h_ = h_land; lift_h_ = std::max(h_here, h_land) + lift_clear_m_;
            phase_ = Phase::Lift; phase_ticks_ = 0;
        }
        break; }
    case Phase::Lift: {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        const Vec3f top = at_gravity_h(tg.x, tg.z, lift_h_);
        const double dy = double(top.y - tg.y), step = lift_v_ * dt_;
        tg.y += float(std::fabs(dy) <= step ? dy : (dy > 0 ? step : -step));
        if (std::fabs(dy) <= step && len(sub(toe_of(leg_, angles_[size_t(leg_)]), tg)) < 2.0 * ik_tol_m_) { phase_ = Phase::Swing; phase_ticks_ = 0; }
        else if (phase_ticks_ > phase_max_ticks_) { ++phase_timeouts_; phase_ = Phase::Swing; phase_ticks_ = 0; }
        break; }
    case Phase::Swing: {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        const Vec3f goal = at_gravity_h(land_.x, land_.z, lift_h_);
        const Vec3f d = sub(goal, tg); const double dn = len(d), step = swing_v_ * dt_;
        if (dn <= step) tg = goal; else tg = add(tg, Vec3f(float(d.x / dn * step), float(d.y / dn * step), float(d.z / dn * step)));
        if (dn <= step && len(sub(toe_of(leg_, angles_[size_t(leg_)]), tg)) < 2.0 * ik_tol_m_) { phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; }
        else if (phase_ticks_ > phase_max_ticks_) { ++phase_timeouts_; tg = goal; phase_ = Phase::Lower; phase_ticks_ = 0; touch_run_ = 0; }
        break; }
    case Phase::Lower: {
        Vec3f& tg = toe_tgt_[size_t(leg_)];
        const Vec3f cmd = toe_of(leg_, angles_[size_t(leg_)]);
        const bool unreachable = len(sub(cmd, tg)) > unreach_tol_m_;
        if (!unreachable) tg.y -= float(lower_v_ * dt_);           // keep lowering only while the leg can follow
        else if (phase_ticks_ % 5 == 0) {                            // out of reach: the landing retreats toward the hip and the toe tries again
            const Vec3f hip = hip1_of(leg_); const double dx = hip.x - tg.x, dz = hip.z - tg.z, dn = std::hypot(dx, dz);
            if (dn > reach_min_m_ + retreat_m_) { tg.x += float(dx / dn * retreat_m_); tg.z += float(dz / dn * retreat_m_); ++retreats_; }
        }
        const double h_cmd = gravity_h(cmd);
        const bool raw_touch = have_contact_ && contact_[size_t(leg_)] > 0.5f && (!have_load_ || load_[size_t(leg_)] >= float(touch_load_));
        touch_run_ = raw_touch ? touch_run_ + 1 : 0;
        const bool touched = touch_run_ >= touch_ticks_ && phase_ticks_ > 3;
        const bool sunk = h_cmd < land_h_ - sink_max_m_ || phase_ticks_ > 4 * phase_max_ticks_;
        if (touched || sunk) {
            if (sunk) ++lower_sunk_;
            // the toe's height where it stopped, against the toes already down: its relative elevation; and the memory's
            // prediction of the ground here against where the toe met it -- the careful stepper's error
            const double h_toe = (touched && have_y_) ? double(y_[size_t(leg_)]) : h_cmd;
            ground_h_[size_t(leg_)] = h_toe;
            if (touched) { land_err_last_ = h_toe - land_h_; land_err_ms_ += 0.1 * (land_err_last_ * land_err_last_ - land_err_ms_); ++landings_; } else ++missed_;
            tg = cmd;   // the stance takes it from here
            ++steps_[size_t(leg_)]; last_step_tick_[size_t(leg_)] = int64_t(tick_now_);
            cycles_ = *std::min_element(steps_.begin(), steps_.end());   // full crawl cycles: every leg has stepped this many times
            leg_ = choose_leg(); shift_acc_ = 0.0; fwd_acc_ = 0.0; fwd_blocked_ = false; still_ticks_ = 0; centre_off_ = false; phase_ = Phase::Shift; phase_ticks_ = 0;
        }
        break; }
    }
}

void PoseSequencer::publish(uint64_t tick_id) {
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); out->sensor = "pose";
    out->values = Eigen::VectorXf::Zero(13);
    out->values[0] = engage_level_;
    for (int i = 0; i < 4; ++i) { const auto u = u_of_angles(i, angles_[size_t(i)]); for (int j = 0; j < 3; ++j) out->values[1 + 3 * i + j] = float(u[size_t(j)]); }
    bus_->publish(output_topic_, out);
}

void PoseSequencer::tick(uint64_t tick_id) {
    tick_now_ = tick_id;
    if (have_contact_) {   // the leaky bearing estimate, with hysteresis
        const double a = 1.0 / bear_tau_ticks_;
        for (int i = 0; i < 4; ++i) {
            bear_[size_t(i)] += a * ((contact_[size_t(i)] > 0.5f ? 1.0 : 0.0) - bear_[size_t(i)]);
            if (bear_[size_t(i)] > bear_on_) bearing_[size_t(i)] = true; else if (bear_[size_t(i)] < bear_off_) bearing_[size_t(i)] = false;
        }
    }
    // the FK check against the body's own toes (the model is the body's; this is the wiring's check), from the gait's command
    if (have_model_ && have_gait_ && have_xz_ && have_y_ && phase_ == Phase::Idle) {
        double e = 0.0;
        for (int i = 0; i < 4; ++i) {
            const Vec3f toe = toe_of(i, angles_of_u(i, {gait_u_[size_t(3 * i)], gait_u_[size_t(3 * i + 1)], gait_u_[size_t(3 * i + 2)]}));
            e += std::hypot(double(toe.x) - xz_[size_t(2 * i)], double(toe.z) - xz_[size_t(2 * i + 1)]) / 4.0;
        }
        fk_err_ema_ += 0.02 * (e - fk_err_ema_);
    }
    const bool upright = !have_up_ || up_[1] >= tilt_disengage_;
    const bool want = engage_ > 0.0 && have_model_ && upright;
    if (!upright && phase_ != Phase::Idle) ++disengaged_tilt_;
    if (want && phase_ == Phase::Idle) {   // from a standing body only
        int n_contact = 0; if (have_contact_) for (int i = 0; i < 4; ++i) n_contact += bearing(i);
        const bool standing = int64_t(tick_id) >= engage_min_tick_ && (!have_contact_ || n_contact >= engage_min_contacts_)
                              && (belly_topic_.empty() || (have_belly_ && belly_ >= float(engage_min_belly_m_)));
        if (standing) engage_now(tick_id);
    }
    if (!want && phase_ != Phase::Idle) { phase_ = Phase::Idle; leg_ = -1; }
    const float ramp = 1.0f / float(engage_ramp_ticks_);
    engage_level_ = std::clamp(engage_level_ + (phase_ != Phase::Idle ? ramp : -ramp), 0.0f, 1.0f);
    if (phase_ != Phase::Idle) {
        ++ticks_engaged_;
        step_machine();
        hold_stance();
        double err = 0.0;
        for (int i = 0; i < 4; ++i) {
            bool done = false;
            for (int k = 0; k < ik_iters_ && !done; ++k) done = ik_step(i, toe_tgt_[size_t(i)], angles_[size_t(i)]);
            err += len(sub(toe_of(i, angles_[size_t(i)]), toe_tgt_[size_t(i)])) / 4.0;
        }
        ik_err_ema_ += 0.05 * (err - ik_err_ema_);
    }
    if (engage_level_ > 0.0f || phase_ != Phase::Idle || engage_ > 0.0) publish(tick_id);
    else if (!output_topic_.empty()) {   // engage 0: the socket sees 0 and the gait is untouched
        auto out = std::make_shared<ProprioToken>();
        out->tick_id = tick_id; out->producer_id = id().empty() ? std::string("pose_sequencer") : std::string(id()); out->sensor = "pose";
        out->values = Eigen::VectorXf::Zero(13); bus_->publish(output_topic_, out);
    }
}

nlohmann::json PoseSequencer::diag_snapshot() const {
    nlohmann::json tg = nlohmann::json::array(), st = nlohmann::json::array(), gh = nlohmann::json::array(), nt = nlohmann::json::array(), sc = nlohmann::json::array();
    for (int i = 0; i < 4; ++i) { tg.push_back({toe_tgt_[size_t(i)].x, toe_tgt_[size_t(i)].y, toe_tgt_[size_t(i)].z}); st.push_back(steps_[size_t(i)]); gh.push_back(ground_h_[size_t(i)]);
                                  nt.push_back({neutral_[size_t(i)].x, neutral_[size_t(i)].z}); sc.push_back(score_[size_t(i)]); }
    return nlohmann::json{{"phase", phase_name(phase_)}, {"phase_id", int(phase_)}, {"leg", leg_}, {"engage", engage_level_}, {"model", have_model_}, {"support", have_support_},
                          {"patch", have_patch_}, {"steps", st}, {"cycles", cycles_}, {"shift_exhausted", shift_exhausted_}, {"shift_ticks", shift_ticks_},
                          {"disengaged_tilt", disengaged_tilt_}, {"lower_sunk", lower_sunk_}, {"phase_timeouts", phase_timeouts_}, {"engaged_at", engaged_at_},
                          {"landings", landings_}, {"missed", missed_}, {"shift_blocked", shift_blocked_}, {"retreats", retreats_}, {"shift_d", {shift_dx_, shift_dz_}}, {"steps_no_advance", steps_no_advance_}, {"deadlock_ticks", deadlock_ticks_}, {"creep", {creep_[0], creep_[1], creep_[2], creep_[3]}}, {"bear", {bear_[0], bear_[1], bear_[2], bear_[3]}}, {"land_err", land_err_last_}, {"land_rms", std::sqrt(land_err_ms_)}, {"elev", {elev_[0], elev_[1], elev_[2], elev_[3]}}, {"ik_err", ik_err_ema_}, {"fk_err", fk_err_ema_}, {"ticks_engaged", ticks_engaged_},
                          {"toe_tgt", tg}, {"ground_h", gh}, {"neutral", nt}, {"score", sc}, {"air_ticks", {air_ticks_[0], air_ticks_[1], air_ticks_[2], air_ticks_[3]}}, {"lift_h", lift_h_}, {"land_h", land_h_}, {"shift_acc", shift_acc_}, {"phase_ticks", phase_ticks_}};
}

} // namespace ogma
