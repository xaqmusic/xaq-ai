// =============================================================================
// LoadPhaseGait.cpp  --  see the header
// =============================================================================
#include "ogma/modules/LoadPhaseGait.hpp"

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
    throw std::invalid_argument("LoadPhaseGait: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("LoadPhaseGait: param '" + k + "' must be a string");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("LoadPhaseGait: param '" + k + "' must be a string array");
}
std::vector<double> get_double_vec(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("LoadPhaseGait: param '" + k + "' must be a numeric array");
}
float wrap(float a) {
    const float two_pi = 6.28318530717958647692f;
    a = std::fmod(a, two_pi);
    if (a < 0.0f) a += two_pi;
    return a;
}
} // namespace

LoadPhaseGait::LoadPhaseGait()  = default;
LoadPhaseGait::~LoadPhaseGait() = default;

std::string_view LoadPhaseGait::type_name() const { return "LoadPhaseGait"; }

std::vector<TopicSpec> LoadPhaseGait::input_topics() const {
    std::vector<TopicSpec> v = {
        TopicSpec{contact_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false} };
    if (!load_topic_.empty())
        v.push_back(TopicSpec{load_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!track_topic_.empty())
        v.push_back(TopicSpec{track_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!support_topic_.empty())
        v.push_back(TopicSpec{support_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!belly_topic_.empty())
        v.push_back(TopicSpec{belly_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}
std::vector<TopicSpec> LoadPhaseGait::output_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : output_topics_) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken))});
    return v;
}

ParamSchema LoadPhaseGait::params_schema() const {
    return {
        {"contact_topic", ParamMutability::ConstructionOnly,
         "ProprioToken [c_0..c_{n-1}], 1 = the foot touches (the physics flag in the sim; the FSR threshold on the robot).",
         ParamValue{std::string("reality.proprio.foot_contact")}},
        {"load_topic", ParamMutability::ConstructionOnly,
         "Optional ProprioToken [f_0..f_{n-1}], the per-foot force (reality.proprio.foot_load). When set, each leg's load "
         "SHARE is f_i / sum_j f_j; when empty, the share is c_i / (number of feet touching). Empty = contact only.",
         ParamValue{std::string("")}},
        {"output_topics", ParamMutability::ConstructionOnly,
         "One ProprioToken topic per leg: [cos phi, sin phi, swing (1/0), load share].",
         ParamValue{std::vector<std::string>{"rhythm.leg.fl", "rhythm.leg.fr", "rhythm.leg.rl", "rhythm.leg.rr"}}},
        {"n_legs", ParamMutability::ConstructionOnly, "Number of legs (= output_topics length).", ParamValue{int64_t{4}}},
        {"period_ticks", ParamMutability::HotMutable,
         "The free-running period (ticks): omega = 2pi / period. 70 = the body's measured stride.", ParamValue{70.0}},
        {"sigma_ratio", ParamMutability::HotMutable,
         "sigma / omega: how hard the leg's own load holds its phase in stance. dphi = omega (1 - sigma_ratio * N * cos phi). "
         "With shares N <= 1, a leg carrying more than 1/sigma_ratio of the body's load STOPS at the end of its push and "
         "does not lift until the others take the load. 0 = free clock, no load coupling: the open-loop sequencer the "
         "record rejected, kept as the structure control.", ParamValue{2.0}},
        {"load_floor", ParamMutability::HotMutable,
         "A share below this reads as 0 (sensor noise on an unloaded foot).", ParamValue{0.02}},
        {"seed", ParamMutability::ConstructionOnly, "Seed for the legs' random starting phases.", ParamValue{int64_t{1234}}},
        {"load_mode", ParamMutability::HotMutable,
         "0 = SHARES: N_i = f_i / sum_j f_j (the deployed form; the sum is 1 whatever the body weighs, so the belly's "
         "load and the stride's dynamic peaks are invisible). 1 = ABSOLUTE, Owaki's form: N_i = the load as published "
         "(body weights in the sim; with load_norm_tau, the foot's load over its own stance reference; with no "
         "load_topic, the contact flag 0/1 -- the 'phasic receptor' control). In mode 1, sigma_ratio = sigma * load_unit "
         "/ omega: Owaki & Ishiguro 2017's sigma = 6 rad/(N s) on a 2 kg body is 117.6 rad/s per body weight, i.e. "
         "sigma_ratio 19.6 at their walk (omega 6), 13 at trot (9), 4.5 at gallop (26).", ParamValue{int64_t{0}}},
        {"load_norm_tau", ParamMutability::HotMutable,
         "PER-FOOT SELF-NORMALISATION of the load before the shares: each foot's load is divided by its own running "
         "reference (EMA over tau ticks of its load while touching). The robot's FSRs differ 3x in sensitivity under one "
         "curve (RL 0.5 share, RR 0.05 measured 2026-10-13), which held one leg and freed another. Try 300-1000. 0 = raw.",
         ParamValue{0.0}},
        {"track_topic", ParamMutability::ConstructionOnly,
         "Optional ProprioToken [ratio]: how far the joints lag their commands, as a fraction of the stroke (MotorEPMv2 "
         "track_topic). Feeds the adaptive period (7.30). Empty = no subscription.", ParamValue{std::string("")}},
        {"period_adapt_rate", ParamMutability::HotMutable,
         "ADAPTIVE PERIOD: the period (ticks) moves by rate * (lag ratio - period_adapt_ref) * period per tick, within "
         "period_adapt_min..max, starting from period_ticks: slower strides when the servos cannot complete their strokes, "
         "faster when they keep up. Setting period_ticks live re-seeds it. 0 = off, byte-identical (7.30).", ParamValue{0.0}},
        {"period_adapt_ref", ParamMutability::HotMutable,
         "The lag ratio the adaptive period holds (the fraction of the stroke the joints may trail the command by) (7.30).",
         ParamValue{0.35}},
        {"period_adapt_min", ParamMutability::HotMutable, "Lower bound of the adaptive period (ticks) (7.30).", ParamValue{30.0}},
        {"period_adapt_max", ParamMutability::HotMutable, "Upper bound of the adaptive period (ticks) (7.30).", ParamValue{90.0}},
        {"support_topic", ParamMutability::ConstructionOnly,
         "Optional ProprioToken [x_0, z_0, .. x_{n-1}, z_{n-1}]: each toe's horizontal position in the chassis frame (m), from "
         "the COMMANDED forward kinematics (reality.proprio.feet_xz). Feeds the support-gated lift-off. Empty = none.",
         ParamValue{std::string("")}},
        {"belly_topic", ParamMutability::ConstructionOnly,
         "Optional ProprioToken [m]: belly clearance (reality.proprio.belly_m). The support gate learns each leg's allowance "
         "from the belly dips its lift-offs cause. Empty = none.", ParamValue{std::string("")}},
        {"support_gate", ParamMutability::HotMutable,
         "SUPPORT-GATED LIFT-OFF: 1 = a leg about to enter swing is held at the end of its stance while the CoG lies more than "
         "its learned allowance outside the feet that stay down (support polygon, or the line between two feet), for at most "
         "support_hold_max ticks. Needs support_topic and the contacts; learns with belly_topic. 0 = off, byte-identical.",
         ParamValue{0.0}},
        {"support_margin_init", ParamMutability::HotMutable,
         "Each leg's starting allowance (m): how far outside the remaining support the CoG may be at lift-off.", ParamValue{0.03}},
        {"support_margin_min", ParamMutability::HotMutable,
         "Lower bound of the allowance (m). Above 0: two diagonal feet are a line the CoG is never exactly on.", ParamValue{0.005}},
        {"support_margin_max", ParamMutability::HotMutable, "Upper bound of the allowance (m).", ParamValue{0.08}},
        {"support_learn_rate", ParamMutability::HotMutable,
         "Allowance step per lift-off (m): theta += rate * (support_dip_target - dip). 0 = fixed allowance.", ParamValue{0.002}},
        {"support_dip_target", ParamMutability::HotMutable,
         "The fraction of a leg's lift-offs allowed to dip the belly; the allowance settles where they do.", ParamValue{0.1}},
        {"support_dip_m", ParamMutability::HotMutable,
         "A dip: the belly falls this far (m) below its value at lift-off within support_dip_window ticks.", ParamValue{0.012}},
        {"support_dip_window", ParamMutability::HotMutable, "Ticks a lift-off is watched for a dip.", ParamValue{int64_t{20}}},
        {"support_hold_max", ParamMutability::HotMutable,
         "Longest a leg is held at lift-off (ticks); then it lifts regardless (and the result still teaches).",
         ParamValue{int64_t{25}}},
        {"support_cog", ParamMutability::HotMutable,
         "The CoG's horizontal position in the chassis frame [x, z] (m); [0, 0] = the chassis centre.",
         ParamValue{std::vector<double>{0.0, 0.0}}},
        {"init_phase", ParamMutability::ConstructionOnly,
         "Optional explicit starting phases (radians, one per leg). Empty = seeded random, as on Owaki's robot.",
         ParamValue{std::vector<double>{}}},
    };
}

ParamMap LoadPhaseGait::current_params() const {
    ParamMap m;
    m["contact_topic"] = ParamValue{contact_topic_}; m["load_topic"] = ParamValue{load_topic_};
    m["output_topics"] = ParamValue{output_topics_}; m["n_legs"] = ParamValue{int64_t(n_legs_)};
    m["period_ticks"] = ParamValue{period_ticks_}; m["sigma_ratio"] = ParamValue{sigma_ratio_};
    m["track_topic"] = ParamValue{track_topic_}; m["period_adapt_rate"] = ParamValue{period_adapt_rate_};
    m["period_adapt_ref"] = ParamValue{period_adapt_ref_}; m["period_adapt_min"] = ParamValue{period_adapt_min_};
    m["period_adapt_max"] = ParamValue{period_adapt_max_};
    m["load_floor"] = ParamValue{load_floor_}; m["seed"] = ParamValue{seed_}; m["load_norm_tau"] = ParamValue{load_norm_tau_};
    m["load_mode"] = ParamValue{int64_t(load_mode_)};
    m["init_phase"] = ParamValue{init_phase_};
    m["support_topic"] = ParamValue{support_topic_}; m["belly_topic"] = ParamValue{belly_topic_};
    m["support_gate"] = ParamValue{support_gate_}; m["support_margin_init"] = ParamValue{support_margin_init_};
    m["support_margin_min"] = ParamValue{support_margin_min_}; m["support_margin_max"] = ParamValue{support_margin_max_};
    m["support_learn_rate"] = ParamValue{support_learn_rate_}; m["support_dip_target"] = ParamValue{support_dip_target_};
    m["support_dip_m"] = ParamValue{support_dip_m_}; m["support_dip_window"] = ParamValue{int64_t(support_dip_window_)};
    m["support_hold_max"] = ParamValue{int64_t(support_hold_max_)}; m["support_cog"] = ParamValue{support_cog_};
    return m;
}

void LoadPhaseGait::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("LoadPhaseGait requires a non-null Bus");
    apply_param(params, "contact_topic", [&](auto const& v){ contact_topic_ = get_string(v, "contact_topic"); });
    apply_param(params, "load_topic",    [&](auto const& v){ load_topic_    = get_string(v, "load_topic"); });
    apply_param(params, "output_topics", [&](auto const& v){ output_topics_ = get_string_vec(v, "output_topics"); });
    apply_param(params, "n_legs",        [&](auto const& v){ n_legs_        = std::max(1, int(get_double(v, "n_legs"))); });
    apply_param(params, "period_ticks",  [&](auto const& v){ period_ticks_  = std::max(2.0, get_double(v, "period_ticks")); });
    apply_param(params, "track_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) track_topic_ = *p; });
    apply_param(params, "period_adapt_rate", [&](auto const& v){ period_adapt_rate_ = get_double(v, "period_adapt_rate"); });
    apply_param(params, "period_adapt_ref",  [&](auto const& v){ period_adapt_ref_  = get_double(v, "period_adapt_ref"); });
    apply_param(params, "period_adapt_min",  [&](auto const& v){ period_adapt_min_  = get_double(v, "period_adapt_min"); });
    apply_param(params, "period_adapt_max",  [&](auto const& v){ period_adapt_max_  = get_double(v, "period_adapt_max"); });
    apply_param(params, "sigma_ratio",   [&](auto const& v){ sigma_ratio_   = get_double(v, "sigma_ratio"); });
    apply_param(params, "load_floor",    [&](auto const& v){ load_floor_    = get_double(v, "load_floor"); });
    apply_param(params, "load_mode",     [&](auto const& v){ load_mode_     = int(get_double(v, "load_mode")); });
    apply_param(params, "load_norm_tau", [&](auto const& v){ load_norm_tau_ = get_double(v, "load_norm_tau"); });
    // Read as an integer: the per-run seeds OgmaBrain derives are 64-bit, and through a
    // double they rounded to ONE value, so every OGMA_SEED started the legs from the same
    // phases (found 2026-10-18; the record's LPG seed-averages share one set of start phases).
    apply_param(params, "seed",          [&](auto const& v){
        if (auto p = std::get_if<int64_t>(&v)) seed_ = *p; else seed_ = int64_t(get_double(v, "seed")); });
    apply_param(params, "init_phase",    [&](auto const& v){ init_phase_    = get_double_vec(v, "init_phase"); });
    apply_param(params, "support_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) support_topic_ = *p; });
    apply_param(params, "belly_topic",   [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) belly_topic_ = *p; });
    for (const char* k : {"support_gate", "support_margin_init", "support_margin_min", "support_margin_max",
                          "support_learn_rate", "support_dip_target", "support_dip_m", "support_dip_window",
                          "support_hold_max", "support_cog"})
        apply_param(params, k, [&](auto const& v){ on_param_change(k, v); });
    if (int(output_topics_.size()) != n_legs_)
        throw std::invalid_argument("LoadPhaseGait: output_topics length must equal n_legs");
    if (!init_phase_.empty() && int(init_phase_.size()) != n_legs_)
        throw std::invalid_argument("LoadPhaseGait: init_phase length must equal n_legs");

    const size_t n = size_t(n_legs_);
    phase_.assign(n, 0.0f); share_.assign(n, 0.0f); omega_eff_.assign(n, float(kTwoPi / period_ticks_));
    contact_.assign(n, 1.0f); load_.assign(n, 0.0f); prev_contact_.assign(n, 1);
    td_c_.assign(n, 0.0f); td_s_.assign(n, 0.0f); lo_c_.assign(n, 0.0f); lo_s_.assign(n, 0.0f);
    td_n_.assign(n, 0); lo_n_.assign(n, 0);
    load_ref_.assign(n, 0.0f);
    toe_x_.assign(n, 0.0f); toe_z_.assign(n, 0.0f); theta_.assign(n, float(support_margin_init_));
    hold_n_.assign(n, 0); watch_left_.assign(n, 0); watch_ref_.assign(n, 0.0f); watch_min_.assign(n, 0.0f);
    const size_t np = n * (n - 1) / 2;
    pair_c_.assign(np, 0.0f); pair_s_.assign(np, 0.0f);
    if (!init_phase_.empty()) {
        for (size_t i = 0; i < n; ++i) phase_[i] = wrap(float(init_phase_[i]));
    } else {
        std::mt19937 rng{uint32_t(seed_)};
        std::uniform_real_distribution<float> u(0.0f, kTwoPi);
        for (size_t i = 0; i < n; ++i) phase_[i] = u(rng);
    }

    sub_ids_.push_back(bus_->subscribe(contact_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_contact(p); }));
    if (!load_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(load_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_load(p); }));
    if (!track_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(track_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_track(p); }));
    if (!support_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(support_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_support(p); }));
    if (!belly_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(belly_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_belly(p); }));
}

void LoadPhaseGait::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "period_ticks") { period_ticks_ = std::max(2.0, get_double(value, k)); period_eff_ = period_ticks_; }
    else if (k == "period_adapt_rate") period_adapt_rate_ = get_double(value, k);
    else if (k == "period_adapt_ref")  period_adapt_ref_  = get_double(value, k);
    else if (k == "period_adapt_min")  period_adapt_min_  = get_double(value, k);
    else if (k == "period_adapt_max")  period_adapt_max_  = get_double(value, k);
    else if (k == "sigma_ratio")  sigma_ratio_  = get_double(value, k);
    else if (k == "load_floor")   load_floor_   = get_double(value, k);
    else if (k == "load_mode")    load_mode_    = int(get_double(value, k));
    else if (k == "load_norm_tau") load_norm_tau_ = get_double(value, k);
    else if (k == "support_gate")        support_gate_        = get_double(value, k);
    else if (k == "support_margin_init") {
        support_margin_init_ = get_double(value, k);
        for (auto& t : theta_) t = float(support_margin_init_);           // a new start for every leg
    }
    else if (k == "support_margin_min")  support_margin_min_  = get_double(value, k);
    else if (k == "support_margin_max")  support_margin_max_  = get_double(value, k);
    else if (k == "support_learn_rate")  support_learn_rate_  = get_double(value, k);
    else if (k == "support_dip_target")  support_dip_target_  = get_double(value, k);
    else if (k == "support_dip_m")       support_dip_m_       = get_double(value, k);
    else if (k == "support_dip_window")  support_dip_window_  = std::max(1, int(get_double(value, k)));
    else if (k == "support_hold_max")    support_hold_max_    = std::max(0, int(get_double(value, k)));
    else if (k == "support_cog") {
        auto c = get_double_vec(value, k);
        if (c.size() != 2) throw std::invalid_argument("LoadPhaseGait: support_cog must be [x, z]");
        support_cog_ = c;
    }
    else throw std::invalid_argument("LoadPhaseGait: param '" + k + "' is construction-only / unknown");
}

void LoadPhaseGait::handle_contact(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    const int k = std::min<int>(int(pt->values.size()), n_legs_);
    for (int i = 0; i < k; ++i) contact_[size_t(i)] = pt->values[i];
    have_contact_ = true;
}
void LoadPhaseGait::handle_track(MessagePtr p) {
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 1) return;
    track_ratio_ = pt->values[0];
    have_track_ = true;
}

void LoadPhaseGait::handle_support(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || int(pt->values.size()) < 2 * n_legs_) return;
    for (int i = 0; i < n_legs_; ++i) { toe_x_[size_t(i)] = pt->values[2 * i]; toe_z_[size_t(i)] = pt->values[2 * i + 1]; }
    have_support_ = true;
}
void LoadPhaseGait::handle_belly(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt || pt->values.size() < 1) return;
    belly_ = pt->values[0];
    have_belly_ = true;
}

// Signed distance (m) from the CoG to the support left if leg `lifting` leaves the ground: + inside the polygon of the
// other touching feet, - outside it.  Two feet: minus the distance to the line through them.  One or none: -0.2.
float LoadPhaseGait::support_margin(size_t lifting) const {
    const float cx = float(support_cog_[0]), cz = float(support_cog_[1]);
    std::vector<std::pair<float, float>> pts;
    for (size_t k = 0; k < size_t(n_legs_); ++k)
        if (k != lifting && contact_[k] > 0.5f) pts.emplace_back(toe_x_[k], toe_z_[k]);
    if (pts.size() <= 1) return -0.2f;
    if (pts.size() == 2) {
        const float dx = pts[1].first - pts[0].first, dz = pts[1].second - pts[0].second;
        const float L = std::hypot(dx, dz);
        if (L < 1e-6f) return -0.2f;
        return -std::fabs((cx - pts[0].first) * (-dz) + (cz - pts[0].second) * dx) / L;
    }
    float mx = 0.0f, mz = 0.0f;
    for (auto& q : pts) { mx += q.first; mz += q.second; }
    mx /= float(pts.size()); mz /= float(pts.size());
    std::sort(pts.begin(), pts.end(), [&](auto const& a, auto const& b) {
        return std::atan2(a.second - mz, a.first - mx) < std::atan2(b.second - mz, b.first - mx); });
    float m = 1e9f;
    for (size_t k = 0; k < pts.size(); ++k) {
        const auto& a = pts[k]; const auto& b = pts[(k + 1) % pts.size()];
        const float dx = b.first - a.first, dz = b.second - a.second, L = std::hypot(dx, dz);
        if (L < 1e-6f) continue;
        m = std::min(m, ((cx - a.first) * (-dz) + (cz - a.second) * dx) / L);   // inward normal, counter-clockwise
    }
    return m;
}

void LoadPhaseGait::handle_load(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (!pt) return;
    const int k = std::min<int>(int(pt->values.size()), n_legs_);
    for (int i = 0; i < k; ++i) load_[size_t(i)] = std::max(0.0f, pt->values[i]);
    have_load_ = true;
}

void LoadPhaseGait::tick(uint64_t tick_id) {
    const size_t n = size_t(n_legs_);
    // ADAPTIVE PERIOD (period_adapt_rate; Tegotae plan 7.30).  The right period depends on how fast the servos can
    // move (the operator settled 50 ticks at 4.22 rad/s by hand; 30 at 3.12 rad/s thrashed).  The error is the
    // joints trailing their commands by more than period_adapt_ref of the stroke: then the stride is slower.
    // Relative change, so the step is the same fraction at any period.  0 = off, byte-identical.
    if (period_adapt_rate_ > 0.0) {
        if (period_eff_ <= 0.0) period_eff_ = period_ticks_;
        if (have_track_)
            period_eff_ *= 1.0 + period_adapt_rate_ * (track_ratio_ - period_adapt_ref_);
        period_eff_ = std::clamp(period_eff_, period_adapt_min_, period_adapt_max_);
    }
    const float omega = kTwoPi / float(period_now());
    // ---- load shares: f_i / sum f, or c_i / n_touching ----
    float total = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        float v = have_load_ ? load_[i] : (contact_[i] > 0.5f ? 1.0f : 0.0f);
        if (have_load_ && load_norm_tau_ > 0.0) {
            // the foot's own reference: its load while touching, seeded on the first touch
            if (contact_[i] > 0.5f && v > 0.0f) {
                if (load_ref_[i] <= 0.0f) load_ref_[i] = v;
                else load_ref_[i] += (1.0f / float(load_norm_tau_)) * (v - load_ref_[i]);
            }
            v = (load_ref_[i] > 1e-6f) ? v / load_ref_[i] : 0.0f;
        }
        share_[i] = v; total += v;
    }
    for (size_t i = 0; i < n; ++i) {
        if (load_mode_ != 1) share_[i] = total > 1e-6f ? share_[i] / total : 0.0f;   // mode 1: absolute
        if (share_[i] < float(load_floor_)) share_[i] = 0.0f;
    }
    // ---- the rule, and the instruments it feeds ----
    int retro = 0, sw_n = 0, agree = 0;
    const bool gate = support_gate_ > 0.0 && have_support_ && have_contact_;
    for (size_t i = 0; i < n; ++i) {
        const float d = omega * (1.0f - float(sigma_ratio_) * share_[i] * std::cos(phase_[i]));
        const float before = phase_[i];
        phase_[i] = wrap(phase_[i] + d);
        if (gate && std::sin(before) < 0.0f && std::sin(phase_[i]) > 0.0f && d > 0.0f) {   // about to lift off
            const float mg = support_margin(i);
            sg_margin_ema_ += 0.05f * (mg - sg_margin_ema_);
            if (mg < -theta_[i] && hold_n_[i] < support_hold_max_) {
                phase_[i] = kTwoPi - 1e-3f;                           // held at the end of its stance
                if (hold_n_[i] == 0) ++sg_holds_;
                ++hold_n_[i]; ++sg_hold_ticks_;
            } else {
                if (hold_n_[i] >= support_hold_max_ && mg < -theta_[i]) ++sg_released_;
                hold_n_[i] = 0; ++sg_liftoffs_;
                if (have_belly_) { watch_left_[i] = support_dip_window_; watch_ref_[i] = belly_; watch_min_[i] = belly_; }
            }
        }
        if (watch_left_[i] > 0) {                                     // was that lift-off a dip?
            watch_min_[i] = std::min(watch_min_[i], belly_);
            if (--watch_left_[i] == 0) {
                const bool dip = watch_ref_[i] - watch_min_[i] >= float(support_dip_m_);
                if (dip) ++sg_dips_;
                ++sg_learned_;
                theta_[i] = std::clamp(theta_[i] + float(support_learn_rate_ * (support_dip_target_ - (dip ? 1.0 : 0.0))),
                                       float(support_margin_min_), float(support_margin_max_));
            }
        }
        omega_eff_[i] += kEmaAlpha * (d - omega_eff_[i]);
        if (d < 0.0f) ++retro;
        const bool sw = std::sin(phase_[i]) > 0.0f;
        if (sw) ++sw_n;
        const bool touching = contact_[i] > 0.5f;
        if (sw != touching) ++agree;                      // swing while not touching, stance while touching
        if (have_contact_) {
            const uint8_t c = touching ? 1 : 0;
            // The event EMAs are SEEDED on the first event (an EMA from zero reads
            // 1 - (1 - a)^k, which is the instrument warming up, not the lock).
            if (c && !prev_contact_[i]) {                 // touchdown: where was the phase?
                const float a = td_n_[i] ? kPlvAlpha : 1.0f;
                td_c_[i] += a * (std::cos(phase_[i]) - td_c_[i]);
                td_s_[i] += a * (std::sin(phase_[i]) - td_s_[i]);
                ++td_n_[i]; ++touchdowns_;
            } else if (!c && prev_contact_[i]) {          // liftoff
                const float a = lo_n_[i] ? kPlvAlpha : 1.0f;
                lo_c_[i] += a * (std::cos(phase_[i]) - lo_c_[i]);
                lo_s_[i] += a * (std::sin(phase_[i]) - lo_s_[i]);
                ++lo_n_[i];
            }
            prev_contact_[i] = c;
        }
    }
    size_t pi = 0;
    for (size_t i = 0; i < n; ++i)
        for (size_t j = i + 1; j < n; ++j, ++pi) {
            const float dp = phase_[i] - phase_[j];
            pair_c_[pi] += kPairAlpha * (std::cos(dp) - pair_c_[pi]);
            pair_s_[pi] += kPairAlpha * (std::sin(dp) - pair_s_[pi]);
        }
    retro_ema_ += kEmaAlpha * (float(retro) / float(n) - retro_ema_);
    swing_ema_ += kEmaAlpha * (float(sw_n) / float(n) - swing_ema_);
    agree_ema_ += kEmaAlpha * (float(agree) / float(n) - agree_ema_);
    ++ticks_;
    // ---- publish ----
    for (size_t i = 0; i < n; ++i) {
        auto out = std::make_shared<ProprioToken>();
        out->tick_id = tick_id;
        out->producer_id = id_.empty() ? std::string("load_phase") : id_;
        out->sensor = "legphase";
        out->values = Eigen::VectorXf(4);
        out->values[0] = std::cos(phase_[i]);
        out->values[1] = std::sin(phase_[i]);
        out->values[2] = std::sin(phase_[i]) > 0.0f ? 1.0f : 0.0f;
        out->values[3] = share_[i];
        bus_->publish(output_topics_[i], out);
    }
}

nlohmann::json LoadPhaseGait::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"phase", phase_}, {"ticks", ticks_}};
}
void LoadPhaseGait::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    auto ph = s.value("phase", std::vector<float>{});
    if (ph.size() == phase_.size()) phase_ = ph;
    ticks_ = s.value("ticks", int64_t{0});
}
nlohmann::json LoadPhaseGait::diag_snapshot() const {
    const size_t n = size_t(n_legs_);
    const float omega = kTwoPi / float(period_now());
    float td = 0.0f, lo = 0.0f, weff = 0.0f, nmean = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        td += std::hypot(td_c_[i], td_s_[i]); lo += std::hypot(lo_c_[i], lo_s_[i]);
        weff += omega_eff_[i]; nmean += share_[i];
    }
    float il = 0.0f;
    for (size_t p = 0; p < pair_c_.size(); ++p) il += std::hypot(pair_c_[p], pair_s_[p]);
    const float np = float(std::max<size_t>(1, pair_c_.size()));
    return nlohmann::json{
        {"phase", phase_}, {"share", share_},
        {"td_plv", td / float(n)},            // phase at touchdown: constant across steps -> 1
        {"lo_plv", lo / float(n)},            // phase at liftoff
        {"interleg_plv", il / np},            // pairs holding a constant relative phase -> 1
        {"retro", retro_ema_},                // fraction of leg-ticks the phase ran backwards
        {"swing_frac", swing_ema_},           // fraction of leg-ticks the phase says swing
        {"agree", agree_ema_},                // phase's swing/stance vs the contact flag
        {"omega_rel", weff / float(n) / omega},  // mean effective rate / free rate
        {"load_mean", nmean / float(n)},        {"load_ref", load_ref_},
        {"touchdowns", touchdowns_}, {"ticks", ticks_}, {"period_eff", period_now()}, {"track_ratio", track_ratio_},
        {"have_load", have_load_}, {"have_contact", have_contact_},
        {"sg_theta", theta_}, {"sg_holds", sg_holds_}, {"sg_hold_ticks", sg_hold_ticks_}, {"sg_released", sg_released_},
        {"sg_liftoffs", sg_liftoffs_}, {"sg_dips", sg_dips_}, {"sg_learned", sg_learned_}, {"sg_margin", sg_margin_ema_},
        {"have_support", have_support_}};
}

} // namespace ogma
