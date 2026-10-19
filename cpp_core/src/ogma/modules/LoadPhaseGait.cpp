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
    m["load_floor"] = ParamValue{load_floor_}; m["seed"] = ParamValue{seed_}; m["load_norm_tau"] = ParamValue{load_norm_tau_};
    m["load_mode"] = ParamValue{int64_t(load_mode_)};
    m["init_phase"] = ParamValue{init_phase_};
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
}

void LoadPhaseGait::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    if      (k == "period_ticks") period_ticks_ = std::max(2.0, get_double(value, k));
    else if (k == "sigma_ratio")  sigma_ratio_  = get_double(value, k);
    else if (k == "load_floor")   load_floor_   = get_double(value, k);
    else if (k == "load_mode")    load_mode_    = int(get_double(value, k));
    else if (k == "load_norm_tau") load_norm_tau_ = get_double(value, k);
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
    const float omega = kTwoPi / float(period_ticks_);
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
    for (size_t i = 0; i < n; ++i) {
        const float d = omega * (1.0f - float(sigma_ratio_) * share_[i] * std::cos(phase_[i]));
        phase_[i] = wrap(phase_[i] + d);
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
    const float omega = kTwoPi / float(period_ticks_);
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
        {"touchdowns", touchdowns_}, {"ticks", ticks_},
        {"have_load", have_load_}, {"have_contact", have_contact_}};
}

} // namespace ogma
