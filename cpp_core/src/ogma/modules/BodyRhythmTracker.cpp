#include "ogma/modules/BodyRhythmTracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("BodyRhythmTracker param '" + key + "' must be a string array");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("BodyRhythmTracker param '" + key + "' must be string");
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("BodyRhythmTracker param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("BodyRhythmTracker param '" + key + "' must be integer");
}
std::vector<double> get_double_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("BodyRhythmTracker param '" + key + "' must be a numeric array");
}
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
} // namespace

BodyRhythmTracker::BodyRhythmTracker()  = default;
BodyRhythmTracker::~BodyRhythmTracker() = default;

std::string_view BodyRhythmTracker::type_name() const { return "BodyRhythmTracker"; }

std::vector<TopicSpec> BodyRhythmTracker::input_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(proprio_topics_.size());
    for (auto const& t : proprio_topics_)
        v.emplace_back(t, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    return v;
}

std::vector<TopicSpec> BodyRhythmTracker::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false} };
}

ParamSchema BodyRhythmTracker::params_schema() const {
    return {
        {"proprio_topics", ParamMutability::ConstructionOnly,
         "Per-leg ProprioToken input topics ([pos,act,delta]×motor_dim).",
         std::nullopt, std::nullopt, std::nullopt},
        {"output_topic", ParamMutability::ConstructionOnly,
         "Afferent-reference output: ProprioToken [cos φ_body, sin φ_body, ω_body] the CPG entrains to.",
         std::nullopt, std::nullopt, std::nullopt},
        {"leg_signs", ParamMutability::ConstructionOnly,
         "Per-leg signs of the collective coordinate (trot diagonal, e.g. [1,-1,-1,1]). Length = #legs.",
         std::nullopt, std::nullopt, std::nullopt},
        {"motor_dim", ParamMutability::ConstructionOnly, "joints per leg",
         ParamValue{int64_t(3)}, ParamValue{int64_t(1)}, ParamValue{int64_t(12)}},
        {"swing_joint", ParamMutability::HotMutable,
         "Joint index whose position carries the gait rhythm (the PLL locks to this coordinate).",
         ParamValue{int64_t(0)}, ParamValue{int64_t(0)}, ParamValue{int64_t(11)}},
        {"mean_alpha", ParamMutability::HotMutable, "DC-removal EMA rate for the collective coordinate.",
         ParamValue{0.005}, ParamValue{0.0}, ParamValue{1.0}},
        {"amp_alpha", ParamMutability::HotMutable, "Amplitude EMA rate (diag + crossing hysteresis).",
         ParamValue{0.02}, ParamValue{0.0}, ParamValue{1.0}},
        {"period_alpha", ParamMutability::HotMutable, "EMA rate for the measured up-crossing period.",
         ParamValue{0.10}, ParamValue{0.0}, ParamValue{1.0}},
        {"omega_lp", ParamMutability::HotMutable,
         "Per-tick low-pass of ω toward 2π/period — smooths frequency drift (no waveform breaks).",
         ParamValue{0.05}, ParamValue{0.0}, ParamValue{1.0}},
        {"phase_lock", ParamMutability::HotMutable,
         "Proportional pull of φ toward the reference (φ=0) at each up-crossing (feedback phase lock).",
         ParamValue{0.10}, ParamValue{0.0}, ParamValue{1.0}},
        {"init_period", ParamMutability::ConstructionOnly, "Initial period in ticks before a crossing is measured (ω₀ = 2π/init_period).",
         ParamValue{60.0}, ParamValue{2.0}, ParamValue{1000.0}},
        {"period_min", ParamMutability::HotMutable, "Sanity floor on the reported period (ticks).",
         ParamValue{16.0}, ParamValue{2.0}, ParamValue{1000.0}},
        {"period_max", ParamMutability::HotMutable, "Sanity ceiling on the reported period (ticks).",
         ParamValue{400.0}, ParamValue{2.0}, ParamValue{10000.0}},
    };
}

ParamMap BodyRhythmTracker::current_params() const {
    ParamMap m;
    m["proprio_topics"] = std::vector<std::string>(proprio_topics_);
    m["output_topic"]   = output_topic_;
    m["leg_signs"]      = std::vector<double>(leg_signs_.begin(), leg_signs_.end());
    m["motor_dim"]      = int64_t(motor_dim_);
    m["swing_joint"]    = int64_t(swing_joint_);
    m["mean_alpha"]     = mean_alpha_;
    m["amp_alpha"]      = amp_alpha_;
    m["period_alpha"]   = period_alpha_;
    m["omega_lp"]       = omega_lp_;
    m["phase_lock"]     = phase_lock_;
    m["period_min"]     = period_min_;
    m["period_max"]     = period_max_;
    return m;
}

void BodyRhythmTracker::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("BodyRhythmTracker requires a non-null Bus");

    double init_period = 60.0;
    apply_param(params, "motor_dim",   [&](auto const& v){ motor_dim_   = int(get_int(v, "motor_dim")); });
    apply_param(params, "swing_joint", [&](auto const& v){ swing_joint_ = int(get_int(v, "swing_joint")); });
    apply_param(params, "mean_alpha",  [&](auto const& v){ mean_alpha_  = get_double(v, "mean_alpha"); });
    apply_param(params, "amp_alpha",   [&](auto const& v){ amp_alpha_   = get_double(v, "amp_alpha"); });
    apply_param(params, "period_alpha",[&](auto const& v){ period_alpha_= get_double(v, "period_alpha"); });
    apply_param(params, "omega_lp",    [&](auto const& v){ omega_lp_    = get_double(v, "omega_lp"); });
    apply_param(params, "phase_lock",  [&](auto const& v){ phase_lock_  = get_double(v, "phase_lock"); });
    apply_param(params, "init_period", [&](auto const& v){ init_period  = get_double(v, "init_period"); });
    apply_param(params, "period_min",  [&](auto const& v){ period_min_  = get_double(v, "period_min"); });
    apply_param(params, "period_max",  [&](auto const& v){ period_max_  = get_double(v, "period_max"); });
    apply_param(params, "output_topic",[&](auto const& v){ output_topic_= get_string(v, "output_topic"); });
    apply_param(params, "proprio_topics", [&](auto const& v){ proprio_topics_ = get_string_vec(v, "proprio_topics"); });
    apply_param(params, "leg_signs", [&](auto const& v){
        auto d = get_double_vec(v, "leg_signs");
        leg_signs_.assign(d.begin(), d.end());
    });

    if (proprio_topics_.empty())
        throw std::invalid_argument("BodyRhythmTracker: proprio_topics must be non-empty");
    if (swing_joint_ < 0 || swing_joint_ >= motor_dim_)
        throw std::invalid_argument("BodyRhythmTracker: swing_joint out of [0,motor_dim)");

    int nl = int(proprio_topics_.size());
    if (leg_signs_.empty()) {
        // Default: trot diagonal (+,-,-,+) for the canonical [fl,fr,rl,rr] order; +1 for anything longer.
        leg_signs_.assign(nl, 1.0f);
        const float diag[4] = {1.0f, -1.0f, -1.0f, 1.0f};
        for (int i = 0; i < nl && i < 4; ++i) leg_signs_[i] = diag[i];
    }
    if (int(leg_signs_.size()) != nl)
        throw std::invalid_argument("BodyRhythmTracker: leg_signs length must equal #proprio_topics");

    pos_.assign(nl, Eigen::VectorXf());
    seen_.assign(nl, 0);
    mean_ema_.assign(motor_dim_, 0.0f);
    amp_ema_.assign(motor_dim_, 0.0f);
    f_last_.assign(motor_dim_, 0.0f);
    below_.assign(motor_dim_, 1);
    ticks_since_up_.assign(motor_dim_, 0);
    period_zc_ema_.assign(motor_dim_, 0.0f);
    int nlm = nl * motor_dim_;
    raw_mean_.assign(nlm, 0.0f);
    raw_amp_.assign(nlm, 0.0f);
    raw_period_.assign(nlm, 0.0f);
    raw_below_.assign(nlm, 1);
    raw_tsu_.assign(nlm, 0);

    omega_ = kTwoPi_ / float(std::max(2.0, init_period));
    phi_body_ = 0.0f;
    crossings_seen_ = 0;

    sub_ids_.clear();
    for (int leg = 0; leg < nl; ++leg)
        sub_ids_.push_back(bus_->subscribe(proprio_topics_[leg], SubscriptionKind::Direct,
            [this, leg](std::string_view, MessagePtr p){ handle_proprio(leg, p); }));
}

void BodyRhythmTracker::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "swing_joint") swing_joint_ = std::clamp(int(get_int(value, k)), 0, motor_dim_ - 1);
    else if (k == "mean_alpha")  mean_alpha_  = get_double(value, k);
    else if (k == "amp_alpha")   amp_alpha_   = get_double(value, k);
    else if (k == "period_alpha")period_alpha_= get_double(value, k);
    else if (k == "omega_lp")    omega_lp_    = get_double(value, k);
    else if (k == "phase_lock")  phase_lock_  = get_double(value, k);
    else if (k == "period_min")  period_min_  = get_double(value, k);
    else if (k == "period_max")  period_max_  = get_double(value, k);
    else throw std::invalid_argument("BodyRhythmTracker: param '" + k + "' is not HotMutable");
}

void BodyRhythmTracker::handle_proprio(int leg, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p || leg < 0 || leg >= int(pos_.size())) return;
    if (p->values.size() >= 3 * motor_dim_) {
        if (pos_[leg].size() != motor_dim_) pos_[leg] = Eigen::VectorXf::Zero(motor_dim_);
        for (int j = 0; j < motor_dim_; ++j) pos_[leg][j] = p->values[3 * j];  // position at index 3j
        seen_[leg] = 1;
    }
}

void BodyRhythmTracker::tick(uint64_t tick_id) {
    int nl = int(proprio_topics_.size());
    for (int leg = 0; leg < nl; ++leg) if (!seen_[leg]) return;   // wait for a full frame

    // --- Per-joint collective coordinate F_j = Σ_leg sign[leg]·pos[leg][j], mean-removed ---
    // (all joints tracked so the measurement pass can pick the cleanest swing carrier)
    const float ma = float(mean_alpha_), aa = float(amp_alpha_), pa = float(period_alpha_);
    bool swing_up = false;                              // did the SWING joint up-cross this tick?
    for (int j = 0; j < motor_dim_; ++j) {
        float raw = 0.0f;
        for (int leg = 0; leg < nl; ++leg) raw += leg_signs_[leg] * pos_[leg][j];
        mean_ema_[j] += ma * (raw - mean_ema_[j]);
        float f = raw - mean_ema_[j];
        amp_ema_[j] = (1.0f - aa) * amp_ema_[j] + aa * std::fabs(f);

        // Up-crossing period with amplitude-proportional hysteresis (noise-robust).
        float hys = 0.2f * amp_ema_[j];
        ++ticks_since_up_[j];
        if (below_[j] && f > hys) {
            if (period_zc_ema_[j] <= 0.0f) period_zc_ema_[j] = float(ticks_since_up_[j]);
            else period_zc_ema_[j] = (1.0f - pa) * period_zc_ema_[j] + pa * float(ticks_since_up_[j]);
            ticks_since_up_[j] = 0;
            below_[j] = 0;
            if (j == swing_joint_) { swing_up = true; ++crossings_seen_; }
        } else if (!below_[j] && f < -hys) {
            below_[j] = 1;
        }
        f_last_[j] = f;
    }
    f_swing_ = f_last_[swing_joint_];

    // --- Raw per-(leg,joint) zero-crossing period + amplitude (diagnostic; NOT the diagonal
    // coordinate) — shows whether a single leg's joints share one frequency (intra-leg coherence)
    // and whether the same joint agrees across legs (inter-leg coherence).
    for (int leg = 0; leg < nl; ++leg) {
        for (int j = 0; j < motor_dim_; ++j) {
            int idx = leg * motor_dim_ + j;
            float raw = pos_[leg][j];
            raw_mean_[idx] += ma * (raw - raw_mean_[idx]);
            float f = raw - raw_mean_[idx];
            raw_amp_[idx] = (1.0f - aa) * raw_amp_[idx] + aa * std::fabs(f);
            float hys = 0.2f * raw_amp_[idx];
            ++raw_tsu_[idx];
            if (raw_below_[idx] && f > hys) {
                if (raw_period_[idx] <= 0.0f) raw_period_[idx] = float(raw_tsu_[idx]);
                else raw_period_[idx] = (1.0f - pa) * raw_period_[idx] + pa * float(raw_tsu_[idx]);
                raw_tsu_[idx] = 0; raw_below_[idx] = 0;
            } else if (!raw_below_[idx] && f < -hys) {
                raw_below_[idx] = 1;
            }
        }
    }

    // --- Measurement-seeded PLL locked to the swing coordinate ---
    // FEED-FORWARD frequency: the up-crossing interval is an unbiased period measurement; ω is
    // low-passed toward 2π/period so it drifts smoothly.  FEEDBACK phase: φ integrates ω and is
    // softly pulled to the reference (φ=0) at each up-crossing → locks phase without a jump.
    float w_hi = kTwoPi_ / float(std::max(2.0, period_min_));   // sanity rails (NOT the aliasing
    float w_lo = kTwoPi_ / float(std::max(2.0, period_max_));   // clamp — that's the CPG's job)
    float P = period_zc_ema_[swing_joint_];
    if (crossings_seen_ >= 2 && P > 1.0f) {
        float omega_target = std::clamp(kTwoPi_ / P, w_lo, w_hi);
        omega_ += float(omega_lp_) * (omega_target - omega_);
    }
    omega_ = std::clamp(omega_, w_lo, w_hi);

    phi_body_ += omega_;                                        // integrate (smooth by construction)
    if (swing_up) {                                            // proportional phase-lock to φ_ref = 0
        float err = (phi_body_ > kTwoPi_ * 0.5f) ? (kTwoPi_ - phi_body_) : (-phi_body_);
        // Lock instrument, PRE-pull (post-pull would measure the corrector, not the lock):
        // where in [0,2π) does the crossing actually land, and how big is the residual.
        lock_cos_sum_ += std::cos(phi_body_);
        lock_sin_sum_ += std::sin(phi_body_);
        ++lock_n_;
        lock_err_ema_ = (lock_err_ema_ < 0.0f) ? std::fabs(err)
                        : 0.95f * lock_err_ema_ + 0.05f * std::fabs(err);
        phi_body_ += float(phase_lock_) * err;
    }
    phi_body_ = std::fmod(phi_body_, kTwoPi_);
    if (phi_body_ < 0.0f) phi_body_ += kTwoPi_;

    // --- Publish the afferent reference the CPG entrains to ---
    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("body_rhythm_tracker") : id_;
    out->sensor      = "body.gait";
    out->values      = Eigen::VectorXf(3);
    out->values[0]   = std::cos(phi_body_);
    out->values[1]   = std::sin(phi_body_);
    out->values[2]   = omega_;              // rad/tick — the frequency the CPG locks toward
    bus_->publish(output_topic_, out);
}

// ---- diag (live viz / measurement) ----
nlohmann::json BodyRhythmTracker::diag_snapshot() const {
    nlohmann::json j;
    j["phi_body"]    = phi_body_;
    j["omega"]       = omega_;
    j["period_est"]  = omega_ > 1e-6f ? kTwoPi_ / omega_ : 0.0f;   // the frequency the CPG locks toward
    j["locked"]      = crossings_seen_ >= 2;
    j["crossings"]   = crossings_seen_;
    // Lock QUALITY (2026-08-09) — "locked" above is a warm-up counter, not a measure.
    j["lock_plv"]     = lock_n_ ? std::sqrt(lock_cos_sum_ * lock_cos_sum_
                                          + lock_sin_sum_ * lock_sin_sum_) / double(lock_n_) : 0.0;
    j["lock_err_ema"] = lock_err_ema_;
    j["lock_n"]       = lock_n_;
    j["f_swing"]     = f_swing_;
    j["swing_joint"] = swing_joint_;
    // Per-joint amplitude + up-crossing period (the raw measurement / pick the cleanest carrier).
    nlohmann::json amp = nlohmann::json::array(), pzc = nlohmann::json::array();
    for (int jn = 0; jn < motor_dim_; ++jn) { amp.push_back(amp_ema_[jn]); pzc.push_back(period_zc_ema_[jn]); }
    j["amp_per_joint"]       = amp;
    j["period_zc_per_joint"] = pzc;
    // Raw per-(leg,joint) period + amplitude: [leg][joint] (joint order hip1,hip2,knee).
    int nl = int(proprio_topics_.size());
    nlohmann::json rp = nlohmann::json::array(), ra = nlohmann::json::array();
    for (int leg = 0; leg < nl; ++leg) {
        nlohmann::json lp = nlohmann::json::array(), la = nlohmann::json::array();
        for (int jt = 0; jt < motor_dim_; ++jt) {
            int idx = leg * motor_dim_ + jt;
            lp.push_back(idx < int(raw_period_.size()) ? raw_period_[idx] : 0.0f);
            la.push_back(idx < int(raw_amp_.size())    ? raw_amp_[idx]    : 0.0f);
        }
        rp.push_back(lp); ra.push_back(la);
    }
    j["raw_period"] = rp;
    j["raw_amp"]    = ra;
    return j;
}

// ---- snapshot / restore (persist the PLL + DC/measurement state) ----
nlohmann::json BodyRhythmTracker::snapshot_state() const {
    // NOTE FOR ANYONE ADDING AN INSTRUMENT: headless runs read
    // brain.get_module_snapshot() → THIS json — diag_snapshot() feeds only the live
    // inspector.  Until 2026-08-09 this module had no "module" dict, so every BRT
    // diagnostic read 0.0/absent in every headless run (the documented
    // instrument-invisibility trap, third occurrence).  Export in BOTH places.
    nlohmann::json mod;
    mod["phi_body"]     = phi_body_;
    mod["omega"]        = omega_;
    mod["period_est"]   = omega_ > 1e-6f ? kTwoPi_ / omega_ : 0.0f;
    mod["crossings"]    = crossings_seen_;
    mod["lock_plv"]     = lock_n_ ? std::sqrt(lock_cos_sum_ * lock_cos_sum_
                                            + lock_sin_sum_ * lock_sin_sum_)
                                        / double(lock_n_) : 0.0;
    mod["lock_err_ema"] = lock_err_ema_;
    mod["lock_n"]       = lock_n_;
    {
        nlohmann::json pzc = nlohmann::json::array();
        for (int jn = 0; jn < motor_dim_; ++jn) pzc.push_back(period_zc_ema_[jn]);
        mod["period_zc_per_joint"] = pzc;
    }
    return nlohmann::json{
        {"version", 1},
        {"omega", omega_}, {"phi_body", phi_body_}, {"crossings", crossings_seen_},
        {"mean_ema",  std::vector<float>(mean_ema_.begin(),  mean_ema_.end())},
        {"amp_ema",   std::vector<float>(amp_ema_.begin(),   amp_ema_.end())},
        {"period_zc", std::vector<float>(period_zc_ema_.begin(), period_zc_ema_.end())},
        {"module", mod},
    };
}

void BodyRhythmTracker::restore_state(nlohmann::json const& s) {
    omega_          = s.value("omega", omega_);
    phi_body_       = s.value("phi_body", phi_body_);
    crossings_seen_ = s.value("crossings", crossings_seen_);
    auto load = [&](const char* key, std::vector<float>& dst) {
        if (!s.contains(key)) return;
        auto v = s[key].get<std::vector<float>>();
        for (int j = 0; j < int(dst.size()) && j < int(v.size()); ++j) dst[j] = v[j];
    };
    load("mean_ema",  mean_ema_);
    load("amp_ema",   amp_ema_);
    load("period_zc", period_zc_ema_);
}

} // namespace ogma
