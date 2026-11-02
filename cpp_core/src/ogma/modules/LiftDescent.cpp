#include "ogma/modules/LiftDescent.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("LiftDescent param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("LiftDescent param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    if (auto p = std::get_if<double>(&v))  return *p != 0.0;
    throw std::invalid_argument("LiftDescent param '" + key + "' must be a bool");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("LiftDescent param '" + key + "' must be a string");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("LiftDescent param '" + key + "' must be a list of strings");
}

} // namespace

LiftDescent::LiftDescent()  = default;
LiftDescent::~LiftDescent() = default;

std::string_view LiftDescent::type_name() const { return "LiftDescent"; }

std::vector<TopicSpec> LiftDescent::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& t : phase_topics_) v.push_back(TopicSpec{t, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    v.push_back(TopicSpec{contact_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!stall_topic_.empty())    v.push_back(TopicSpec{stall_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!tilt_topic_.empty())     v.push_back(TopicSpec{tilt_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!distress_topic_.empty()) v.push_back(TopicSpec{distress_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    if (!situation_topic_.empty()) v.push_back(TopicSpec{situation_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> LiftDescent::output_topics() const {
    return {TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))}};
}

ParamSchema LiftDescent::params_schema() const {
    return {
        {"phase_topics", ParamMutability::ConstructionOnly,
         "The gait's per-leg rhythm topics in leg order fl, fr, rl, rr: [cos phi, sin phi, swing (1/0), load share] (LoadPhaseGait).",
         ParamValue{std::vector<std::string>{"rhythm.leg.fl", "rhythm.leg.fr", "rhythm.leg.rl", "rhythm.leg.rr"}}},
        {"contact_topic", ParamMutability::ConstructionOnly, "Per-leg foot contact [4], 1 = touching.", ParamValue{std::string("reality.proprio.foot_contact")}},
        {"stall_topic", ParamMutability::ConstructionOnly, "StuckSense output; value 0 = the stall score (the progress error). Empty = not used.", ParamValue{std::string("reality.proprio.stuck")}},
        {"tilt_topic", ParamMutability::ConstructionOnly, "[sin pitch, cos pitch, sin roll, cos roll]: the complement term's source. Empty = not used.", ParamValue{std::string("reality.proprio.tilt")}},
        {"distress_topic", ParamMutability::ConstructionOnly, "Optional; a trial with distress above distress_max is discarded.", ParamValue{std::string("")}},
        {"output_topic", ParamMutability::ConstructionOnly, "Published [front multiplier, rear multiplier] (MotorEPMv2 lift_handle_topic).", ParamValue{std::string("handle.lift")}},
        {"enabled", ParamMutability::HotMutable, "false = publish nothing (MotorEPMv2 then runs byte-identically).", ParamValue{false}},
        {"u_init_front", ParamMutability::ConstructionOnly, "Starting front multiplier.", ParamValue{1.0}},
        {"u_init_rear", ParamMutability::ConstructionOnly, "Starting rear multiplier.", ParamValue{1.0}},
        {"u_lo", ParamMutability::HotMutable, "Known-safe lower bound on both multipliers.", ParamValue{0.5}},
        {"u_hi", ParamMutability::HotMutable, "Known-safe upper bound on both multipliers.", ParamValue{3.0}},
        {"hold_ticks", ParamMutability::HotMutable, "Ticks each trial setting is held.", ParamValue{int64_t{400}}},
        {"settle_ticks", ParamMutability::HotMutable, "Ticks at the start of a trial that are not measured.", ParamValue{int64_t{100}}},
        {"delta0", ParamMutability::HotMutable, "Initial dither half-width.", ParamValue{0.25}},
        {"delta_min", ParamMutability::HotMutable, "Smallest dither half-width.", ParamValue{0.05}},
        {"delta_anneal", ParamMutability::HotMutable, "delta = max(delta_min, delta0 / sqrt(1 + valid trials / this)).", ParamValue{20.0}},
        {"step_max", ParamMutability::HotMutable, "Largest move of the operating point per step.", ParamValue{0.05}},
        {"forget", ParamMutability::HotMutable, "RLS forgetting factor per trial (the world drifts as the terrain changes).", ParamValue{0.97}},
        {"distress_max", ParamMutability::HotMutable, "Distress above this in a trial discards it.", ParamValue{0.5}},
        {"min_swing", ParamMutability::HotMutable, "Measured swing ticks each pair needs for the trial to count.", ParamValue{int64_t{40}}},
        {"report_every", ParamMutability::HotMutable, "> 0: print {\"event\":\"LIFT_DIAG\", ...} every N ticks. 0 = off.", ParamValue{int64_t{0}}},
        {"situation_topic", ParamMutability::ConstructionOnly, "G4b (2026-11-02): a RealityToken stream whose winner_id names the SITUATION (the encounter EPM: reality.touch.encounter). Each situation keeps its own operating point and slope. \"\" = one global point (G4a, byte-identical).", ParamValue{std::string("")}},
        {"situation_tau_ticks", ParamMutability::HotMutable, "Occupancy EMA of the winners (ticks): a per-footfall vocabulary held into a context of a stride or more.", ParamValue{60.0}},
        {"situation_min_occ", ParamMutability::HotMutable, "The leading occupancy a situation needs before the body switches to its operating point (confident and stable).", ParamValue{0.5}},
        {"situation_min_frac", ParamMutability::HotMutable, "A trial counts for the situation that held most of its measured ticks, and only if that is at least this fraction (the situation changed mid-way otherwise: discarded).", ParamValue{0.6}},
        {"situation_max", ParamMutability::HotMutable, "Distinct situations kept; further winners use the global point.", ParamValue{int64_t{16}}},
    };
}

ParamMap LiftDescent::current_params() const {
    return {
        {"phase_topics", ParamValue{phase_topics_}}, {"contact_topic", ParamValue{contact_topic_}},
        {"stall_topic", ParamValue{stall_topic_}}, {"tilt_topic", ParamValue{tilt_topic_}},
        {"distress_topic", ParamValue{distress_topic_}}, {"output_topic", ParamValue{output_topic_}},
        {"enabled", ParamValue{enabled_}}, {"u_init_front", ParamValue{u_init_front_}}, {"u_init_rear", ParamValue{u_init_rear_}},
        {"u_lo", ParamValue{u_lo_}}, {"u_hi", ParamValue{u_hi_}}, {"hold_ticks", ParamValue{int64_t(hold_ticks_)}},
        {"settle_ticks", ParamValue{int64_t(settle_ticks_)}}, {"delta0", ParamValue{delta0_}}, {"delta_min", ParamValue{delta_min_}},
        {"delta_anneal", ParamValue{delta_anneal_}}, {"step_max", ParamValue{step_max_}}, {"forget", ParamValue{forget_}},
        {"distress_max", ParamValue{distress_max_}}, {"min_swing", ParamValue{int64_t(min_swing_)}},
        {"report_every", ParamValue{int64_t(report_every_)}},
        {"situation_topic", ParamValue{situation_topic_}}, {"situation_tau_ticks", ParamValue{situation_tau_ticks_}},
        {"situation_min_occ", ParamValue{situation_min_occ_}}, {"situation_min_frac", ParamValue{situation_min_frac_}},
        {"situation_max", ParamValue{int64_t(situation_max_)}},
    };
}

void LiftDescent::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("LiftDescent requires a non-null Bus");
    phase_topics_ = {"rhythm.leg.fl", "rhythm.leg.fr", "rhythm.leg.rl", "rhythm.leg.rr"};
    apply_param(params, "phase_topics",   [&](auto const& v){ phase_topics_   = get_string_vec(v, "phase_topics"); });
    apply_param(params, "contact_topic",  [&](auto const& v){ contact_topic_  = get_string(v, "contact_topic"); });
    apply_param(params, "stall_topic",    [&](auto const& v){ stall_topic_    = get_string(v, "stall_topic"); });
    apply_param(params, "tilt_topic",     [&](auto const& v){ tilt_topic_     = get_string(v, "tilt_topic"); });
    apply_param(params, "distress_topic", [&](auto const& v){ distress_topic_ = get_string(v, "distress_topic"); });
    apply_param(params, "output_topic",   [&](auto const& v){ output_topic_   = get_string(v, "output_topic"); });
    apply_param(params, "situation_topic", [&](auto const& v){ situation_topic_ = get_string(v, "situation_topic"); });
    apply_param(params, "u_init_front",   [&](auto const& v){ u_init_front_   = get_double(v, "u_init_front"); });
    apply_param(params, "u_init_rear",    [&](auto const& v){ u_init_rear_    = get_double(v, "u_init_rear"); });
    for (auto const& kv : params) {
        const std::string& k = kv.first;
        if (k == "enabled" || k == "u_lo" || k == "u_hi" || k == "hold_ticks" || k == "settle_ticks" || k == "delta0"
            || k == "delta_min" || k == "delta_anneal" || k == "step_max" || k == "forget" || k == "distress_max"
            || k == "min_swing" || k == "report_every" || k == "situation_tau_ticks" || k == "situation_min_occ"
            || k == "situation_min_frac" || k == "situation_max")
            on_param_change(k, kv.second);
    }
    if (phase_topics_.size() != 4) throw std::invalid_argument("LiftDescent: phase_topics must name 4 legs (fl, fr, rl, rr)");
    sits_[-1].u = {std::clamp(u_init_front_, u_lo_, u_hi_), std::clamp(u_init_rear_, u_lo_, u_hi_)};
    u_trial_ = sits_[-1].u;
    sit_now_ = -1;
    for (size_t i = 0; i < 4; ++i)
        sub_ids_.push_back(bus_->subscribe(phase_topics_[i], SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){ handle_phase(i, p); }));
    sub_ids_.push_back(bus_->subscribe(contact_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_contact(p); }));
    if (!stall_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(stall_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_stall(p); }));
    if (!tilt_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(tilt_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_tilt(p); }));
    if (!distress_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(distress_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_distress(p); }));
    if (!situation_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(situation_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_situation(p); }));
    // A teleport / respawn mid-trial makes the trial meaningless: discard it.
    sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix, SubscriptionKind::Direct,
        [this](std::string_view topic, MessagePtr){ if (topic == "events.reset") trial_bad_ = true; }));
}

void LiftDescent::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "enabled")      enabled_      = get_bool(value, k);
    else if (k == "u_lo")         u_lo_         = get_double(value, k);
    else if (k == "u_hi")         u_hi_         = get_double(value, k);
    else if (k == "hold_ticks")   hold_ticks_   = std::max(2, int(get_int(value, k)));
    else if (k == "settle_ticks") settle_ticks_ = std::max(0, int(get_int(value, k)));
    else if (k == "delta0")       delta0_       = std::max(0.0, get_double(value, k));
    else if (k == "delta_min")    delta_min_    = std::max(0.0, get_double(value, k));
    else if (k == "delta_anneal") delta_anneal_ = std::max(1e-6, get_double(value, k));
    else if (k == "step_max")     step_max_     = std::max(0.0, get_double(value, k));
    else if (k == "forget")       forget_       = std::clamp(get_double(value, k), 0.5, 1.0);
    else if (k == "distress_max") distress_max_ = get_double(value, k);
    else if (k == "min_swing")    min_swing_    = std::max(1, int(get_int(value, k)));
    else if (k == "report_every") report_every_ = std::max(0, int(get_int(value, k)));
    else if (k == "situation_tau_ticks") situation_tau_ticks_ = std::max(1.0, get_double(value, k));
    else if (k == "situation_min_occ")   situation_min_occ_   = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "situation_min_frac")  situation_min_frac_  = std::clamp(get_double(value, k), 0.0, 1.0);
    else if (k == "situation_max")       situation_max_       = std::max(1, int(get_int(value, k)));
    else throw std::invalid_argument("LiftDescent param '" + k + "' is ConstructionOnly or unknown");
}

void LiftDescent::handle_phase(size_t leg, MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (pt && pt->values.size() >= 3) swing_[leg] = pt->values[2];
}
void LiftDescent::handle_contact(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (pt && pt->values.size() >= 4) for (int i = 0; i < 4; ++i) contact_[size_t(i)] = pt->values[i];
}
void LiftDescent::handle_stall(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (pt && pt->values.size() >= 1) stall_ = pt->values[0];
}
void LiftDescent::handle_tilt(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (pt && pt->values.size() >= 3) { sin_pitch_ = pt->values[0]; sin_roll_ = pt->values[2]; have_tilt_ = true; }
}
void LiftDescent::handle_distress(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(p);
    if (pt && pt->values.size() >= 1) distress_ = pt->values[0];
}

void LiftDescent::handle_situation(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(p);
    if (rt && rt->winner_id >= 0) winner_ = rt->winner_id;
}

LiftDescent::Situation& LiftDescent::sit(int key) {
    auto it = sits_.find(key);
    if (it != sits_.end()) return it->second;
    // a new situation starts from the operating point the body is at (its neighbour in time)
    Situation s = sits_.count(sit_now_) ? sits_.at(sit_now_) : sits_.at(-1);
    s.theta = Eigen::Vector3d::Zero(); s.P = Eigen::Matrix3d::Identity() * 100.0; s.res_var = 1.0; s.res_n = 0.0;
    s.trials_valid = 0; s.steps = 0;
    return sits_.emplace(key, s).first->second;
}
LiftDescent::Situation const& LiftDescent::sit_const(int key) const {
    auto it = sits_.find(key);
    return it != sits_.end() ? it->second : sits_.at(-1);
}

double LiftDescent::delta(Situation const& s) const {
    return std::max(delta_min_, delta0_ / std::sqrt(1.0 + double(s.trials_valid) / delta_anneal_));
}

void LiftDescent::start_trial() {
    trial_delta_ = delta(sit(sit_now_));
    trial_bad_ = false;
    sw_n_[0] = sw_n_[1] = sw_touch_[0] = sw_touch_[1] = 0;
    stall_sum_ = tilt_n_ = p_sum_ = p_sq_ = r_sum_ = r_sq_ = 0;
    trial_sit_ticks_.clear();
}

void LiftDescent::end_trial() {
    const double measured = double(hold_ticks_ - settle_ticks_);
    // the situation the trial belongs to: the one that held most of its measured ticks
    int dom = -1; int dom_ticks = -1;
    for (auto const& [k, n] : trial_sit_ticks_) if (n > dom_ticks) { dom = k; dom_ticks = n; }
    const double frac = measured > 0 && dom_ticks > 0 ? double(dom_ticks) / measured : 0.0;
    const bool valid = !trial_bad_ && sw_n_[0] >= min_swing_ && sw_n_[1] >= min_swing_ && measured > 0
                       && frac >= situation_min_frac_;
    if (!valid) { ++trials_discarded_; return; }
    Situation& S = sit(dom);
    last_trial_sit_ = dom;
    // the dither as it was applied to THIS situation's point (clamped at the bound as published)
    std::array<double, 2> du{0.0, 0.0};
    du[size_t(axis_)] = std::clamp(S.u[size_t(axis_)] + double(sign_) * trial_delta_, u_lo_, u_hi_) - S.u[size_t(axis_)];
    std::array<double, 4> o{};
    o[0] = sw_touch_[0] / sw_n_[0];
    o[1] = sw_touch_[1] / sw_n_[1];
    o[2] = stall_sum_ / measured;
    if (tilt_n_ > 1) {
        const double vp = std::max(0.0, p_sq_ / tilt_n_ - std::pow(p_sum_ / tilt_n_, 2));
        const double vr = std::max(0.0, r_sq_ / tilt_n_ - std::pow(r_sum_ / tilt_n_, 2));
        o[3] = std::sqrt(vp + vr);
    }
    last_o_ = o;
    ++trials_valid_; ++S.trials_valid;
    // Running spread of each outcome over trials: the normalisation (adapted, not tuned).
    o_n_ += 1.0;
    double obj = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        const double d = o[i] - o_mean_[i];
        o_mean_[i] += d / o_n_;
        o_m2_[i]   += d * (o[i] - o_mean_[i]);
        const double sd = o_n_ > 1.0 ? std::sqrt(o_m2_[i] / (o_n_ - 1.0)) : 0.0;
        obj += o[i] / std::max(sd, 1e-3 + 0.1 * std::fabs(o_mean_[i]));
    }
    last_obj_ = obj;
    if (o_n_ < 4.0) return;            // the spreads need a few trials before the objective means anything
    // RLS: objective ≈ θ0 + θ1 du_front + θ2 du_rear, du relative to the situation's operating point.
    const Eigen::Vector3d x(1.0, du[0], du[1]);
    const double lam = forget_;
    const Eigen::Vector3d Px = S.P * x;
    const double denom = lam + x.dot(Px);
    const Eigen::Vector3d k = Px / denom;
    const double e = obj - S.theta.dot(x);
    S.theta += k * e;
    S.P = (S.P - k * Px.transpose()) / lam;
    S.res_n += 1.0;
    S.res_var = S.res_n < 2.0 ? e * e : 0.9 * S.res_var + 0.1 * e * e;
    // Step only after the −δ half of an axis's pair, and only on a significant slope.
    if (sign_ < 0) {
        const int a = axis_;
        const double g  = S.theta[a + 1];
        const double se = std::sqrt(std::max(0.0, S.P(a + 1, a + 1)) * std::max(S.res_var, 1e-9));
        if (std::fabs(g) > 2.0 * se) {
            const double step = -std::copysign(std::min(step_max_, 0.5 * delta(S)), g);
            const double before = S.u[size_t(a)];
            S.u[size_t(a)] = std::clamp(S.u[size_t(a)] + step, u_lo_, u_hi_);
            S.theta[0] += g * (S.u[size_t(a)] - before);   // re-centre the intercept on the new operating point
            if (S.u[size_t(a)] != before) { ++steps_; ++S.steps; }
        }
    }
}

void LiftDescent::tick(uint64_t) {
    ++ticks_;
    if (report_every_ > 0 && ticks_ % report_every_ == 0) {
        auto d = diag_snapshot();
        d["event"] = "LIFT_DIAG"; d["id"] = std::string(id()); d["tick"] = ticks_;
        std::cout << d.dump() << std::endl;
    }
    if (!enabled_) return;
    // the situation: the leading winner by occupancy, once it leads clearly; held otherwise
    if (!situation_topic_.empty() && winner_ >= 0) {
        const double a = 1.0 / situation_tau_ticks_;
        for (auto& [k, o] : occ_) o *= (1.0 - a);
        occ_[winner_] += a;
        int best = -1; double best_o = -1.0;
        for (auto const& [k, o] : occ_) if (o > best_o) { best = k; best_o = o; }
        if (best >= 0 && best_o >= situation_min_occ_ && best != sit_now_) {
            if (sits_.count(best) || int(sits_.size()) - 1 < situation_max_) { sit(best); sit_now_ = best; }
            else sit_now_ = -1;
        }
    }
    if (trial_tick_ == 0) start_trial();
    // the handle: the current situation's operating point with the trial's dither on one axis
    {
        Situation const& S = sit(sit_now_);
        u_trial_ = S.u;
        u_trial_[size_t(axis_)] = std::clamp(S.u[size_t(axis_)] + double(sign_) * trial_delta_, u_lo_, u_hi_);
    }
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = ticks_; out->producer_id = std::string(id()); out->sensor = "lift_handle";
    out->values.resize(2);
    out->values[0] = float(u_trial_[0]); out->values[1] = float(u_trial_[1]);
    bus_->publish(output_topic_, out);
    ++trial_tick_;
    if (trial_tick_ > settle_ticks_) {
        ++trial_sit_ticks_[sit_now_];
        for (size_t i = 0; i < 4; ++i) {
            if (swing_[i] > 0.5f) {
                const int pair = i < 2 ? 0 : 1;
                sw_n_[pair] += 1.0;
                if (contact_[i] > 0.5f) sw_touch_[pair] += 1.0;
            }
        }
        stall_sum_ += stall_;
        if (have_tilt_) { p_sum_ += sin_pitch_; p_sq_ += double(sin_pitch_) * sin_pitch_; r_sum_ += sin_roll_; r_sq_ += double(sin_roll_) * sin_roll_; tilt_n_ += 1.0; }
        if (!distress_topic_.empty() && distress_ > distress_max_) trial_bad_ = true;
    }
    if (trial_tick_ >= hold_ticks_) {
        end_trial();
        trial_tick_ = 0;
        // dither order: front +, front −, rear +, rear −
        if (sign_ > 0) sign_ = -1; else { sign_ = 1; axis_ = 1 - axis_; }
    }
}

nlohmann::json LiftDescent::diag_snapshot() const {
    Situation const& S = sit_const(sit_now_);
    const double se_f = std::sqrt(std::max(0.0, S.P(1, 1)) * std::max(S.res_var, 1e-9));
    const double se_r = std::sqrt(std::max(0.0, S.P(2, 2)) * std::max(S.res_var, 1e-9));
    nlohmann::json sits = nlohmann::json::object();
    for (auto const& [k, s] : sits_) sits[std::to_string(k)] = {s.u[0], s.u[1], s.trials_valid, s.steps};
    return nlohmann::json{
        {"enabled", enabled_}, {"u_front", S.u[0]}, {"u_rear", S.u[1]},
        {"trial_front", u_trial_[0]}, {"trial_rear", u_trial_[1]}, {"delta", delta(S)},
        {"trials_valid", trials_valid_}, {"trials_discarded", trials_discarded_}, {"steps", steps_},
        {"g_front", S.theta[1]}, {"g_rear", S.theta[2]}, {"se_front", se_f}, {"se_rear", se_r},
        {"e_front", last_o_[0]}, {"e_rear", last_o_[1]}, {"e_stall", last_o_[2]}, {"e_tilt", last_o_[3]},
        {"objective", last_obj_},
        {"situation", sit_now_}, {"winner", winner_}, {"n_situations", int(sits_.size()) - 1}, {"last_trial_situation", last_trial_sit_},
        {"situations", sits},
    };
}

} // namespace ogma
