#include "ogma/modules/Klinotaxis.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

namespace ogma {

namespace {
constexpr float kPi  = 3.14159265358979323846f;
constexpr float k2Pi = 6.28318530717958647692f;
inline float wrap_pi(float a) { return std::atan2(std::sin(a), std::cos(a)); }

template <class Fn>
void apply_param(ParamMap const& params, char const* key, Fn fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("Klinotaxis: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("Klinotaxis: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("Klinotaxis: param '" + k + "' must be a string");
}
bool get_bool(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("Klinotaxis: param '" + k + "' must be bool");
}
}  // namespace

std::string_view Klinotaxis::type_name() const { return "Klinotaxis"; }

std::vector<TopicSpec> Klinotaxis::input_topics() const {
    return {
        TopicSpec{scalar_topic_,  std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{imu_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> Klinotaxis::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(ProprioToken))} };
}

ParamSchema Klinotaxis::params_schema() const {
    return {
        {"scalar_topic",  ParamMutability::ConstructionOnly, "SCALAR gradient to follow.", ParamValue{std::string("reality.proprio.scent_max")}},
        {"heading_topic", ParamMutability::ConstructionOnly, "Egomotion heading.", ParamValue{std::string("reality.proprio.heading")}},
        {"imu_topic",     ParamMutability::ConstructionOnly, "Reafferent yaw rate ω (demod reference).", ParamValue{std::string("reality.proprio.ang_vel")}},
        {"authority_topic", ParamMutability::ConstructionOnly,
            "klino's MotorBus authority ∈[0,1] (e.g. motor.bus.authority.klino). Below authority_floor a "
            "reflex is driving → base_heading re-anchors to the ACTUAL heading (accept the escape, don't "
            "fight back). Empty (default) = off.", ParamValue{std::string("")}},
        {"authority_floor", ParamMutability::HotMutable,
            "Authority below this = a reflex has the bus → re-anchor base_heading to the current heading.", ParamValue{0.5}},
        {"vel_topic",     ParamMutability::ConstructionOnly, "Egocentric velocity [v_right, v_forward] = the ACTUAL line of travel.", ParamValue{std::string("reality.proprio.vel_ego")}},
        {"align_gate",    ParamMutability::HotMutable,
            "HEADING-vs-TRAVEL gate (>0=on): the reafference Δscent·ω assumes v ‖ heading, which wall-slides "
            "violate (heading into the wall, motion along it) → spurious gradient. Weight the reafferent "
            "base_heading update by forward-alignment = clamp(v_forward/|v|,0,1): full when moving along the "
            "heading, →0 when sliding/wedged, so klino only LEARNS the gradient from motion it actually made. "
            "false (default) = ungated.", ParamValue{false}},
        {"output_topic",  ParamMutability::ConstructionOnly, "Weaving heading [vx,vy] → HeadingController.", ParamValue{std::string("percept.klino_heading")}},
        {"period_ticks",  ParamMutability::HotMutable, "Initial weave period (~1 Hz = 60 @ 60fps).", ParamValue{60.0}},
        {"weave_amp",     ParamMutability::HotMutable, "Lateral weave excursion (rad) — the far-field sensing wiggle.", ParamValue{0.5}},
        {"weave_adapt",   ParamMutability::HotMutable,
            "PROXIMITY-ADAPTIVE weave (the fine close): shrink the sensing excursion as the bug nears the "
            "source — weave_amp·(1−cap), cap = scalar / running-peak (the field's OWN max, self-calibrated, "
            "no magic threshold). Wide probe far (find the gradient), fades to a COMMIT on the source so the "
            "±weave stops swinging the mouth across the food. false (default) = fixed weave.", ParamValue{false}},
        {"peak_decay",    ParamMutability::HotMutable,
            "Slow decay of the running scalar peak (the proximity denominator); ~1/decay-tick memory of the "
            "strongest source. A memory timescale, not a behavioural knob.", ParamValue{0.0005}},
        {"short_ema_a",   ParamMutability::HotMutable, "Trend fast-EMA rate.", ParamValue{0.3}},
        {"long_ema_a",    ParamMutability::HotMutable, "Trend slow-EMA rate (short−long = derivative).", ParamValue{0.05}},
        {"lockin_lr",     ParamMutability::HotMutable, "Lock-in low-pass rate (~1/few periods).", ParamValue{0.02}},
        {"steer_gain",    ParamMutability::HotMutable, "Base-heading steer per unit correlation.", ParamValue{0.05}},
        {"per_tick_gain", ParamMutability::HotMutable,
            "SIMPLE per-tick reafference (>0): steer base_heading by the RAW ddt·ω each tick (ddt "
            "scale-normalised → field-independent) instead of the lock-in Pearson correlation. Rewards "
            "improvement per-tick and lets base_heading INTEGRATE (no low-pass, no adaptive period, no "
            "noise-averaging cycle). 0 (default) = legacy lock-in.", ParamValue{0.0}},
        {"turn_commit",   ParamMutability::HotMutable, "|climb dir−heading| beyond this → turn in place (no weave).", ParamValue{1.5708}},
        {"mode",          ParamMutability::HotMutable, "+1 follow / -1 flee.", ParamValue{int64_t{1}}},
        {"shuffle_omega",  ParamMutability::HotMutable, "ABLATION (c): replace ω with noise → break the lock-in.", ParamValue{false}},
        {"lesion_at",     ParamMutability::HotMutable, "(d) drop ω (=0) from this tick; -1 = off.", ParamValue{int64_t{-1}}},
        {"lesion_for",    ParamMutability::HotMutable, "(d) ticks to keep ω dropped, then restore.", ParamValue{int64_t{0}}},
        {"adapt_period",  ParamMutability::HotMutable, "SNR-adaptive weave period (hill-climb |g|).", ParamValue{true}},
        {"period_min",    ParamMutability::HotMutable, "Fastest weave allowed (body limit).", ParamValue{50.0}},
        {"period_max",    ParamMutability::HotMutable, "Slowest weave allowed.", ParamValue{240.0}},
        {"adapt_interval",ParamMutability::HotMutable, "Ticks between period hill-climb steps.", ParamValue{int64_t{300}}},
        {"adapt_step",    ParamMutability::HotMutable, "Period change per hill-climb step.", ParamValue{8.0}},
    };
}

ParamMap Klinotaxis::current_params() const {
    ParamMap m;
    m["scalar_topic"] = ParamValue{scalar_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["imu_topic"] = ParamValue{imu_topic_};
    m["authority_topic"] = ParamValue{authority_topic_};
    m["authority_floor"] = ParamValue{double(authority_floor_)};
    m["vel_topic"] = ParamValue{vel_topic_};
    m["align_gate"] = ParamValue{align_gate_};
    m["output_topic"] = ParamValue{output_topic_};
    m["period_ticks"] = ParamValue{double(period_ticks_)};
    m["weave_amp"] = ParamValue{double(weave_amp_)};
    m["weave_adapt"] = ParamValue{weave_adapt_};
    m["peak_decay"] = ParamValue{double(peak_decay_)};
    m["short_ema_a"] = ParamValue{double(short_ema_a_)};
    m["long_ema_a"] = ParamValue{double(long_ema_a_)};
    m["lockin_lr"] = ParamValue{double(lockin_lr_)};
    m["steer_gain"] = ParamValue{double(steer_gain_)};
    m["per_tick_gain"] = ParamValue{double(per_tick_gain_)};
    m["turn_commit"] = ParamValue{double(turn_commit_)};
    m["mode"] = ParamValue{int64_t(mode_)};
    m["shuffle_omega"] = ParamValue{shuffle_omega_};
    m["lesion_at"] = ParamValue{int64_t(lesion_at_)};
    m["lesion_for"] = ParamValue{int64_t(lesion_for_)};
    m["adapt_period"] = ParamValue{adapt_period_};
    m["period_min"] = ParamValue{double(period_min_)};
    m["period_max"] = ParamValue{double(period_max_)};
    m["adapt_interval"] = ParamValue{int64_t(adapt_interval_)};
    m["adapt_step"] = ParamValue{double(adapt_step_)};
    return m;
}

void Klinotaxis::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "period_ticks")   { period_ticks_ = float(get_double(value, k)); period_ = period_ticks_; }
    else if (k == "weave_amp")      weave_amp_     = float(get_double(value, k));
    else if (k == "weave_adapt")    weave_adapt_   = get_bool(value, k);
    else if (k == "peak_decay")     peak_decay_    = float(get_double(value, k));
    else if (k == "short_ema_a")    short_ema_a_   = float(get_double(value, k));
    else if (k == "long_ema_a")     long_ema_a_    = float(get_double(value, k));
    else if (k == "lockin_lr")      lockin_lr_     = float(get_double(value, k));
    else if (k == "steer_gain")     steer_gain_    = float(get_double(value, k));
    else if (k == "per_tick_gain")  per_tick_gain_ = float(get_double(value, k));
    else if (k == "turn_commit")    turn_commit_   = float(get_double(value, k));
    else if (k == "mode")           mode_          = int(get_int(value, k)) >= 0 ? 1 : -1;
    else if (k == "shuffle_omega")  shuffle_omega_ = get_bool(value, k);
    else if (k == "lesion_at")      lesion_at_     = int(get_int(value, k));
    else if (k == "lesion_for")     lesion_for_    = int(get_int(value, k));
    else if (k == "adapt_period")   adapt_period_  = get_bool(value, k);
    else if (k == "period_min")     period_min_    = float(get_double(value, k));
    else if (k == "period_max")     period_max_    = float(get_double(value, k));
    else if (k == "adapt_interval") adapt_interval_= std::max(1, int(get_int(value, k)));
    else if (k == "adapt_step")     adapt_step_    = float(get_double(value, k));
    else if (k == "authority_floor") authority_floor_ = float(get_double(value, k));
    else if (k == "align_gate")     align_gate_    = get_bool(value, k);
    else throw std::invalid_argument("Klinotaxis: param '" + k + "' is construction-only / unknown");
}

void Klinotaxis::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "scalar_topic",  [&](auto const& v){ scalar_topic_  = get_string(v,"scalar_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v,"heading_topic"); });
    apply_param(params, "imu_topic",     [&](auto const& v){ imu_topic_     = get_string(v,"imu_topic"); });
    apply_param(params, "authority_topic", [&](auto const& v){ authority_topic_ = get_string(v,"authority_topic"); });
    apply_param(params, "authority_floor", [&](auto const& v){ authority_floor_ = float(get_double(v,"authority_floor")); });
    apply_param(params, "vel_topic",     [&](auto const& v){ vel_topic_     = get_string(v,"vel_topic"); });
    apply_param(params, "align_gate",    [&](auto const& v){ align_gate_    = get_bool(v,"align_gate"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "period_ticks",  [&](auto const& v){ period_ticks_  = float(get_double(v,"period_ticks")); });
    apply_param(params, "weave_amp",     [&](auto const& v){ weave_amp_     = float(get_double(v,"weave_amp")); });
    apply_param(params, "weave_adapt",   [&](auto const& v){ weave_adapt_   = get_bool(v,"weave_adapt"); });
    apply_param(params, "peak_decay",    [&](auto const& v){ peak_decay_    = float(get_double(v,"peak_decay")); });
    apply_param(params, "short_ema_a",   [&](auto const& v){ short_ema_a_   = float(get_double(v,"short_ema_a")); });
    apply_param(params, "long_ema_a",    [&](auto const& v){ long_ema_a_    = float(get_double(v,"long_ema_a")); });
    apply_param(params, "lockin_lr",     [&](auto const& v){ lockin_lr_     = float(get_double(v,"lockin_lr")); });
    apply_param(params, "steer_gain",    [&](auto const& v){ steer_gain_    = float(get_double(v,"steer_gain")); });
    apply_param(params, "per_tick_gain", [&](auto const& v){ per_tick_gain_ = float(get_double(v,"per_tick_gain")); });
    apply_param(params, "turn_commit",   [&](auto const& v){ turn_commit_   = float(get_double(v,"turn_commit")); });
    apply_param(params, "mode",          [&](auto const& v){ mode_          = int(get_int(v,"mode")) >= 0 ? 1 : -1; });
    apply_param(params, "shuffle_omega",[&](auto const& v){ shuffle_omega_ = get_bool(v,"shuffle_omega"); });
    apply_param(params, "lesion_at",     [&](auto const& v){ lesion_at_     = int(get_int(v,"lesion_at")); });
    apply_param(params, "lesion_for",    [&](auto const& v){ lesion_for_    = int(get_int(v,"lesion_for")); });
    apply_param(params, "adapt_period",  [&](auto const& v){ adapt_period_  = get_bool(v,"adapt_period"); });
    apply_param(params, "period_min",    [&](auto const& v){ period_min_    = float(get_double(v,"period_min")); });
    apply_param(params, "period_max",    [&](auto const& v){ period_max_    = float(get_double(v,"period_max")); });
    apply_param(params, "adapt_interval",[&](auto const& v){ adapt_interval_= std::max(1, int(get_int(v,"adapt_interval"))); });
    apply_param(params, "adapt_step",    [&](auto const& v){ adapt_step_    = float(get_double(v,"adapt_step")); });

    period_ = std::clamp(period_ticks_, period_min_, period_max_);

    if (!scalar_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(scalar_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_scalar(p); }));
    if (!heading_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_heading(p); }));
    if (!imu_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(imu_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_imu(p); }));
    if (!authority_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(authority_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_authority(p); }));
    if (!vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_vel(p); }));
}

void Klinotaxis::handle_scalar(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { scalar_ = float(pt->values[0]); have_scalar_ = true; }
}
void Klinotaxis::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { heading_ = float(pt->values[0]); have_heading_ = true; }
}
void Klinotaxis::handle_imu(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) omega_ = float(pt->values[0]);   // yaw rate
}
void Klinotaxis::handle_authority(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { authority_ = float(pt->values[0]); have_authority_ = true; }
}
void Klinotaxis::handle_vel(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() >= 2) { v_right_ = float(pt->values[0]); v_forward_ = float(pt->values[1]); }  // egocentric [right, forward]
}

void Klinotaxis::tick(uint64_t tick_id) {
    if (!inited_) {                       // anchor the weave centre to the current heading
        base_heading_ = heading_;
        short_ema_ = long_ema_ = scalar_;
        inited_ = true;
    }

    // DEFENSIBILITY CONTROLS (§2 bar c/d) — corrupt the reafference to prove the lock-in
    // is load-bearing: (c) shuffle ω → its correlation with dscalar/dt → 0; (d) drop ω for
    // a window → the inference goes blind, then recovers when ω is restored.
    if (shuffle_omega_) { std::uniform_real_distribution<float> u(-1.0f, 1.0f); omega_ = u(rng_); }
    if (lesion_at_ >= 0 && int64_t(tick_id) >= lesion_at_ &&
        int64_t(tick_id) < int64_t(lesion_at_) + lesion_for_) omega_ = 0.0f;

    // advance the self-generated weave phase (the ACTIVE probe that makes the body turn)
    phase_ = wrap_pi(phase_ + k2Pi / std::max(1.0f, period_));

    // TREND: dual-timescale band-pass (telemetry: am I climbing overall)
    short_ema_ += short_ema_a_ * (scalar_ - short_ema_);
    long_ema_  += long_ema_a_  * (scalar_ - long_ema_);

    // SELF-CALIBRATED PROXIMITY (for the adaptive weave): normalise the scalar between the bug's OWN
    // searching FLOOR (running min) and source PEAK (running max), both slow-tracking. cap→0 at the
    // floor (far → FULL weave to sense) and →1 at the peak (on the source → NO weave, commit). This
    // keeps the sensing probe full-amplitude while SEARCHING, shrinking only as it actually closes.
    if (!have_scalar_peak_) { scalar_peak_ = scalar_min_ = scalar_; have_scalar_peak_ = true; }
    scalar_peak_ = std::max(scalar_, scalar_peak_ * (1.0f - peak_decay_));
    if (scalar_ < scalar_min_) scalar_min_ = scalar_;                       // new low → snap down
    else scalar_min_ += peak_decay_ * (scalar_ - scalar_min_);              // else slowly leak up
    cap_ = std::clamp((scalar_ - scalar_min_) / (scalar_peak_ - scalar_min_ + 1e-6f), 0.0f, 1.0f);

    // DIRECTION via REAFFERENCE: dscalar/dt = grad_lateral · ω instantaneously, so the
    // whitened correlation ⟨dscalar/dt · ω⟩ recovers the lateral gradient. ω is the IMU's
    // ACTUAL yaw rate (what the body did) — no commanded-phase reference, no lag, no offset.
    float ddt = have_prev_ ? (scalar_ - prev_scalar_) : 0.0f;
    prev_scalar_ = scalar_; have_prev_ = true;
    float steer_signal;
    if (per_tick_gain_ > 0.0f) {
        // PER-TICK REAFFERENCE (simple): reward improvement each tick — steer toward whichever yaw
        // raised the scent. ddt·ω recovers the lateral-gradient sign; ddt is scale-normalised
        // (field-independent gain) but NOT lock-in low-passed → base_heading INTEGRATES it, so the
        // behaviour is its own low-pass (no cycle, no Pearson, no adaptive period, no noise-averaging).
        ddt_scale_ += 0.02f * (std::fabs(ddt) - ddt_scale_);
        g_ = std::clamp((ddt / (ddt_scale_ + 1e-6f)) * omega_, -4.0f, 4.0f);   // reafferent product (telemetry)
        steer_signal = per_tick_gain_ * g_;
    } else {
        cov_     += lockin_lr_ * (ddt * omega_    - cov_);
        var_ddt_ += lockin_lr_ * (ddt * ddt       - var_ddt_);
        var_om_  += lockin_lr_ * (omega_ * omega_ - var_om_);
        g_ = cov_ / (std::sqrt(var_ddt_ * var_om_) + 1e-9f);   // Pearson corr ∈ [-1,1] (bounded by C-S)
        steer_signal = steer_gain_ * g_;
    }

    // ---- AUTHORITY-GATED RE-ANCHOR (operator fix for the reflex fight) -------
    // When a reflex (stuck-escape) has the MotorBus — klino's authority drops — it is re-pointing the
    // bug to get out of a corner. ACCEPT that: snap base_heading to the CURRENT heading so klino
    // re-homes from wherever the reflex leaves it, instead of yanking it back to the stale pre-spin
    // heading (the fight the operator saw). While klino is driving (authority high) the STABLE
    // base_heading is kept — the weave stays a clean probe (strong reafferent sensing) and the homing
    // stays decisive. Inert when the authority topic isn't wired (have_authority_ false).
    if (have_authority_ && authority_ < authority_floor_) {
        base_heading_ = heading_;
        turning_ = false;
    }

    // TURN-COMMIT: if the climb direction is far behind, turn in place to face it. On entry LATCH
    // a rotation direction (sign of the error) and hold it — directly-behind is a ±π dither point.
    float base_ego = wrap_pi(base_heading_ - heading_);
    if (!turning_ && std::fabs(base_ego) > turn_commit_) {
        turning_  = true;
        turn_dir_ = (base_ego >= 0.0f) ? 1.0f : -1.0f;          // latch at entry
    } else if (turning_ && std::fabs(base_ego) < turn_commit_ * 0.4f) {
        turning_ = false;                                       // now facing it → resume (hysteresis)
    }

    // HEADING-vs-TRAVEL alignment gate: the reafference Δscent = ∇scent·v assumes v ‖ heading. When the
    // bug SLIDES along a wall (heading INTO it, motion ALONG it) that assumption breaks — ddt·ω then
    // encodes a gradient the bug never actually sampled along its facing. align = forward component of the
    // ACTUAL travel = clamp(v_forward/|v|,0,1): 1 when moving along the heading, →0 when sliding/wedged.
    // Weight the reafferent base_heading LEARNING by it so klino only steers from motion it truly made.
    // Ungated (align=1) unless align_gate_ is on; also →0 when stationary (no translation ⇒ no valid sample).
    if (align_gate_) {
        float speed = std::hypot(v_forward_, v_right_);
        align_ = (speed > 1e-4f) ? std::clamp(v_forward_ / speed, 0.0f, 1.0f) : 0.0f;
    } else {
        align_ = 1.0f;
    }

    // STEER the maintained base_heading toward (follow) / away from (flee) the lateral gradient —
    // but NOT while turning in place (else the base runs away from the bug as it rotates). The steer is
    // scaled by align_ so wall-slide samples (where heading ≠ travel) don't corrupt the learned climb dir.
    if (!turning_)
        base_heading_ = wrap_pi(base_heading_ + float(mode_) * steer_signal * align_);

    // SNR-adaptive weave period (lock-in mode only; per-tick has no cycle to tune).
    if (adapt_period_ && per_tick_gain_ <= 0.0f) {
        g_accum_ += lockin_mag(); ++g_count_;
        if (++adapt_timer_ >= adapt_interval_) {
            float mean_g = (g_count_ > 0) ? g_accum_ / float(g_count_) : 0.0f;
            if (prev_mean_g_ >= 0.0f && mean_g < prev_mean_g_) period_dir_ = -period_dir_;  // worse → reverse
            period_ = std::clamp(period_ + period_dir_ * adapt_step_, period_min_, period_max_);
            prev_mean_g_ = mean_g; g_accum_ = 0.0f; g_count_ = 0; adapt_timer_ = 0;
        }
    }

    // OUTPUT
    float delta;
    if (turning_) {
        // committed turn-in-place: command toward the climb dir on the LATCHED side, angle capped
        // under π so it's never the ambiguous directly-behind point.
        delta = turn_dir_ * std::min(std::fabs(base_ego), 0.92f * kPi);
    } else {
        // weaving heading = steered centre + lateral oscillation (the sensing probe)
        // PROXIMITY-ADAPTIVE weave: fade the sensing wiggle to a COMMIT as the bug reaches the source
        // (cap→1 ⇒ weave→0), so the ±weave stops swinging the mouth across the food. Fixed if !weave_adapt_.
        float weave_amp_eff = weave_adapt_ ? weave_amp_ * (1.0f - cap_) : weave_amp_;
        delta = wrap_pi((base_heading_ + weave_amp_eff * std::sin(phase_)) - heading_);
    }
    out_vx_ = std::sin(delta);
    out_vy_ = std::cos(delta);

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("klino") : id_;
    out->sensor      = "klino_heading";
    out->values.resize(2);
    out->values[0] = out_vx_;
    out->values[1] = out_vy_;
    bus_->publish(output_topic_, out);
}

nlohmann::json Klinotaxis::diag_snapshot() const {
    return nlohmann::json{
        {"base_heading", base_heading_},
        {"heading", heading_},          // bug's egomotion heading (for the bearing-error geometry)
        {"base_ego", wrap_pi(base_heading_ - heading_)},
        {"turning", turning_},          // turn-in-place commit active
        {"weave_amp", weave_amp_},
        {"cap", cap_},                                                        // self-calibrated proximity ∈[0,1]
        {"weave_eff", weave_adapt_ ? weave_amp_ * (1.0f - cap_) : weave_amp_}, // the applied (shrunk) weave
        {"align", align_},                                                    // heading-vs-travel alignment gate ∈[0,1]
        {"trend", short_ema_ - long_ema_},
        {"g", g_}, {"omega", omega_},
        {"lockin_mag", lockin_mag()},
        {"period", period_},
        {"phase", phase_},
        {"vx", out_vx_}, {"vy", out_vy_},
    };
}

}  // namespace ogma
