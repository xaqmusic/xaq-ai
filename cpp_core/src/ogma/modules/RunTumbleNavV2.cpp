#include "ogma/modules/RunTumbleNavV2.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace ogma {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float wrap_pi(float a) { return std::atan2(std::sin(a), std::cos(a)); }
// Circle-lerp from angle a (w=0) to angle b (w=1) via the two unit vectors, re-angled.
inline float circ_interp(float a, float b, float w) {
    float x = (1.0f - w) * std::cos(a) + w * std::cos(b);
    float y = (1.0f - w) * std::sin(a) + w * std::sin(b);
    if (x == 0.0f && y == 0.0f) return a;
    return std::atan2(y, x);
}
template <class Fn>
void apply_param(ParamMap const& params, char const* key, Fn fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("RunTumbleNavV2: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("RunTumbleNavV2: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("RunTumbleNavV2: param '" + k + "' must be a string");
}
RunTumbleNavV2::Ablation parse_ablation(std::string const& s) {
    if (s.empty() || s == "none") return RunTumbleNavV2::Ablation::None;
    if (s == "shuffle")           return RunTumbleNavV2::Ablation::Shuffle;
    if (s == "kinesis")           return RunTumbleNavV2::Ablation::Kinesis;
    if (s == "wrong_sign")        return RunTumbleNavV2::Ablation::WrongSign;
    if (s == "shuffle_dir")       return RunTumbleNavV2::Ablation::ShuffleDir;
    throw std::invalid_argument("RunTumbleNavV2: unknown ablation '" + s + "'");
}
const char* ablation_name(RunTumbleNavV2::Ablation a) {
    switch (a) {
        case RunTumbleNavV2::Ablation::None:       return "none";
        case RunTumbleNavV2::Ablation::Shuffle:    return "shuffle";
        case RunTumbleNavV2::Ablation::Kinesis:    return "kinesis";
        case RunTumbleNavV2::Ablation::WrongSign:  return "wrong_sign";
        case RunTumbleNavV2::Ablation::ShuffleDir: return "shuffle_dir";
    }
    return "none";
}
}  // namespace

std::string_view RunTumbleNavV2::type_name() const { return "RunTumbleNavV2"; }

std::vector<TopicSpec> RunTumbleNavV2::input_topics() const {
    return {
        TopicSpec{scent_topic_,   std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{vel_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{eat_topic_,     std::type_index(typeid(EnvEvent)),     SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> RunTumbleNavV2::output_topics() const {
    return {
        TopicSpec{output_topic_,     std::type_index(typeid(ProprioToken))},
        TopicSpec{confidence_topic_, std::type_index(typeid(ProprioToken))},
    };
}

ParamSchema RunTumbleNavV2::params_schema() const {
    return {
        {"scent_topic",   ParamMutability::ConstructionOnly, "SCALAR scent concentration (no ring).", ParamValue{std::string("reality.proprio.scent_max")}},
        {"heading_topic", ParamMutability::ConstructionOnly, "Egomotion heading (run-direction frame).", ParamValue{std::string("reality.proprio.heading")}},
        {"vel_topic",     ParamMutability::ConstructionOnly, "Egomotion velocity ([vx,vy]; +fwd = index 1).", ParamValue{std::string("reality.proprio.vel_ego")}},
        {"eat_topic",     ParamMutability::ConstructionOnly, "GROUND-TRUTH eat event -> calibrates eat_scent_ (confidence ONLY; policy reward-free).", ParamValue{std::string("events.eat")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "Chosen heading [vx,vy] -> HeadingController.", ParamValue{std::string("percept.klino_heading")}},
        {"confidence_topic", ParamMutability::ConstructionOnly, "SCALAR self-reported capability in [0,1] -> EFEArbiter.", ParamValue{std::string("percept.klino_confidence")}},
        {"authority_topic", ParamMutability::ConstructionOnly, "KF3: MotorBus authority for the klino channel (scalar in [0,1]); while muted klino coasts + suppresses tumble/belief. Empty = no read.", ParamValue{std::string("")}},
        {"baseline_alpha", ParamMutability::HotMutable, "Methylation adaptation rate (EMA of scent = the prediction baseline).", ParamValue{0.05}},
        {"scale_alpha",   ParamMutability::HotMutable, "EMA rate of the running |error| normaliser.", ParamValue{0.02}},
        {"noise_floor_alpha", ParamMutability::HotMutable, "KF4: EMA rate of the stationary-noise floor (|error| learned while the body is ~stationary, where the true gradient contribution is 0). error_scale is floored by it so flat/saddle fields don't amplify noise into phantom gradient.", ParamValue{0.01}},
        {"tumble_base",   ParamMutability::HotMutable, "Per-tick tumble probability at flat gradient (mean run ~1/this).", ParamValue{0.1}},
        {"tumble_gain",   ParamMutability::HotMutable, "How strongly the normalised temporal gradient modulates tumble probability.", ParamValue{0.1}},
        {"tumble_min",    ParamMutability::HotMutable, "Tumble-probability clamp (low end).", ParamValue{0.0}},
        {"tumble_max",    ParamMutability::HotMutable, "Tumble-probability clamp (high end).", ParamValue{0.5}},
        {"tumble_range",  ParamMutability::HotMutable, "Max reorient per tumble (rad); +/-pi/2 keeps the reorient forward.", ParamValue{1.5708}},
        {"stuck_frac",    ParamMutability::HotMutable, "KF2 efference-matched stuck: while EXECUTING a run, blocked = |vel| < stuck_frac * vel_scale (achieved << the body's own capable speed). Retires the static m/s threshold.", ParamValue{0.2}},
        {"stuck_ticks",   ParamMutability::HotMutable, "Sustained blocked ticks (while executing) -> forced tumble.", ParamValue{int64_t{10}}},
        {"peak_decay",    ParamMutability::HotMutable, "Slow decay of the scent-magnitude / vel-scale memories (halflife ~1400 ticks).", ParamValue{0.0005}},
        {"eat_scent_alpha", ParamMutability::HotMutable, "Per-EAT EMA rate of eat_scent_ (the capability denominator; confidence only).", ParamValue{0.2}},
        {"dir_lr",        ParamMutability::HotMutable, "KF6 EMA rate of the directional belief (short timescale; also the decay-on-loss base rate).", ParamValue{0.1}},
        {"authority_floor", ParamMutability::HotMutable, "KF3: authority below this = muted.", ParamValue{0.5}},
        {"ablation",      ParamMutability::HotMutable, "Validation control. 'none'=full taxis; 'shuffle'=gradient-blind random-walk FLOOR; 'kinesis'=directional belief off (R=0); 'wrong_sign'=belief update sign flipped (must regress); 'shuffle_dir'=randomise run_dir in the belief update (must regress to kinesis).", ParamValue{std::string("none")}},
        {"master_seed",   ParamMutability::ConstructionOnly, "RNG seed.", ParamValue{int64_t{11}}},
    };
}

ParamMap RunTumbleNavV2::current_params() const {
    ParamMap m;
    m["scent_topic"] = ParamValue{scent_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["vel_topic"] = ParamValue{vel_topic_};
    m["eat_topic"] = ParamValue{eat_topic_};
    m["output_topic"] = ParamValue{output_topic_};
    m["confidence_topic"] = ParamValue{confidence_topic_};
    m["authority_topic"] = ParamValue{authority_topic_};
    m["baseline_alpha"] = ParamValue{double(baseline_alpha_)};
    m["scale_alpha"] = ParamValue{double(scale_alpha_)};
    m["noise_floor_alpha"] = ParamValue{double(noise_floor_alpha_)};
    m["tumble_base"] = ParamValue{double(tumble_base_)};
    m["tumble_gain"] = ParamValue{double(tumble_gain_)};
    m["tumble_min"] = ParamValue{double(tumble_min_)};
    m["tumble_max"] = ParamValue{double(tumble_max_)};
    m["tumble_range"] = ParamValue{double(tumble_range_)};
    m["stuck_frac"] = ParamValue{double(stuck_frac_)};
    m["stuck_ticks"] = ParamValue{int64_t(stuck_ticks_)};
    m["peak_decay"] = ParamValue{double(peak_decay_)};
    m["eat_scent_alpha"] = ParamValue{double(eat_scent_alpha_)};
    m["dir_lr"] = ParamValue{double(dir_lr_)};
    m["authority_floor"] = ParamValue{double(authority_floor_)};
    m["ablation"] = ParamValue{std::string(ablation_name(ablation_))};
    m["master_seed"] = ParamValue{int64_t(master_seed_)};
    return m;
}

void RunTumbleNavV2::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "baseline_alpha") baseline_alpha_ = float(get_double(value, k));
    else if (k == "scale_alpha")    scale_alpha_    = float(get_double(value, k));
    else if (k == "noise_floor_alpha") noise_floor_alpha_ = float(get_double(value, k));
    else if (k == "tumble_base")    tumble_base_    = float(get_double(value, k));
    else if (k == "tumble_gain")    tumble_gain_    = float(get_double(value, k));
    else if (k == "tumble_min")     tumble_min_     = float(get_double(value, k));
    else if (k == "tumble_max")     tumble_max_     = float(get_double(value, k));
    else if (k == "tumble_range")   tumble_range_   = float(get_double(value, k));
    else if (k == "stuck_frac")     stuck_frac_     = float(get_double(value, k));
    else if (k == "stuck_ticks")    stuck_ticks_    = std::max(1, int(get_int(value, k)));
    else if (k == "peak_decay")     peak_decay_     = float(get_double(value, k));
    else if (k == "eat_scent_alpha") eat_scent_alpha_ = float(get_double(value, k));
    else if (k == "dir_lr")         dir_lr_         = float(get_double(value, k));
    else if (k == "authority_floor") authority_floor_ = float(get_double(value, k));
    else if (k == "ablation")       ablation_       = parse_ablation(get_string(value, k));
    else throw std::invalid_argument("RunTumbleNavV2: param '" + k + "' is construction-only / unknown");
}

void RunTumbleNavV2::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "scent_topic",   [&](auto const& v){ scent_topic_   = get_string(v,"scent_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v,"heading_topic"); });
    apply_param(params, "vel_topic",     [&](auto const& v){ vel_topic_     = get_string(v,"vel_topic"); });
    apply_param(params, "eat_topic",     [&](auto const& v){ eat_topic_     = get_string(v,"eat_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "confidence_topic", [&](auto const& v){ confidence_topic_ = get_string(v,"confidence_topic"); });
    apply_param(params, "authority_topic", [&](auto const& v){ authority_topic_ = get_string(v,"authority_topic"); });
    apply_param(params, "baseline_alpha",[&](auto const& v){ baseline_alpha_= float(get_double(v,"baseline_alpha")); });
    apply_param(params, "scale_alpha",   [&](auto const& v){ scale_alpha_   = float(get_double(v,"scale_alpha")); });
    apply_param(params, "noise_floor_alpha",[&](auto const& v){ noise_floor_alpha_ = float(get_double(v,"noise_floor_alpha")); });
    apply_param(params, "tumble_base",   [&](auto const& v){ tumble_base_   = float(get_double(v,"tumble_base")); });
    apply_param(params, "tumble_gain",   [&](auto const& v){ tumble_gain_   = float(get_double(v,"tumble_gain")); });
    apply_param(params, "tumble_min",    [&](auto const& v){ tumble_min_    = float(get_double(v,"tumble_min")); });
    apply_param(params, "tumble_max",    [&](auto const& v){ tumble_max_    = float(get_double(v,"tumble_max")); });
    apply_param(params, "tumble_range",  [&](auto const& v){ tumble_range_  = float(get_double(v,"tumble_range")); });
    apply_param(params, "stuck_frac",    [&](auto const& v){ stuck_frac_    = float(get_double(v,"stuck_frac")); });
    apply_param(params, "stuck_ticks",   [&](auto const& v){ stuck_ticks_   = std::max(1, int(get_int(v,"stuck_ticks"))); });
    apply_param(params, "peak_decay",    [&](auto const& v){ peak_decay_    = float(get_double(v,"peak_decay")); });
    apply_param(params, "eat_scent_alpha",[&](auto const& v){ eat_scent_alpha_ = float(get_double(v,"eat_scent_alpha")); });
    apply_param(params, "dir_lr",        [&](auto const& v){ dir_lr_        = float(get_double(v,"dir_lr")); });
    apply_param(params, "authority_floor", [&](auto const& v){ authority_floor_ = float(get_double(v,"authority_floor")); });
    apply_param(params, "ablation",      [&](auto const& v){ ablation_      = parse_ablation(get_string(v,"ablation")); });
    apply_param(params, "master_seed",   [&](auto const& v){ master_seed_   = uint64_t(get_int(v,"master_seed")); });

    rng_.seed(master_seed_);

    if (!scent_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(scent_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_scent(p); }));
    if (!heading_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_heading(p); }));
    if (!vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_vel(p); }));
    if (!eat_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(eat_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_eat(p); }));
    if (!authority_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(authority_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_authority(p); }));
}

void RunTumbleNavV2::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { smax_ = float(pt->values[0]); have_scent_ = true; }
}
void RunTumbleNavV2::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) heading_ = float(pt->values[0]);
}
void RunTumbleNavV2::handle_vel(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() >= 2) vel_fwd_ = float(pt->values[1]);
}
void RunTumbleNavV2::handle_authority(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { authority_ = float(pt->values[0]); have_authority_ = true; }
}
void RunTumbleNavV2::handle_eat(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ev = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!ev) return;
    eat_scent_sample_ = smax_;   // food may move on the eat -> sample the at-eat scent now
    eat_pending_ = true;
}

void RunTumbleNavV2::tick(uint64_t tick_id) {
    if (!have_run_dir_) { run_dir_abs_ = heading_; have_run_dir_ = true; baseline_ = smax_; run_start_scent_ = smax_; }

    const bool grad_on   = (ablation_ != Ablation::Shuffle);                       // Shuffle = gradient-blind
    const bool belief_on = (ablation_ == Ablation::None || ablation_ == Ablation::WrongSign || ablation_ == Ablation::ShuffleDir);

    // --- learned speed scale (KF2/KF4): running peak of the body's own forward speed ---
    vel_scale_ = std::max(std::fabs(vel_fwd_), vel_scale_ * (1.0f - peak_decay_));
    const bool stationary = (vel_scale_ > 1e-4f) && (std::fabs(vel_fwd_) < 0.15f * vel_scale_);

    // --- methylation prediction/error + KF4 noise floor ---
    float error = smax_ - baseline_;
    baseline_    += baseline_alpha_ * (smax_ - baseline_);
    error_scale_ += scale_alpha_ * (std::fabs(error) - error_scale_);
    if (stationary) noise_floor_ += noise_floor_alpha_ * (std::fabs(error) - noise_floor_);  // KF4: noise measured with no motion-gradient
    float eff_scale = std::max(error_scale_, noise_floor_);
    float error_n = error / (eff_scale + 1e-6f);
    last_error_ = error_n;

    // KF6 belief COLLAPSE-ON-LOSS: falling scent (error_n<0) bleeds the accumulator so a stale
    // heading re-infers within the run (perturbation recovery). Uses the module's own error.
    if (belief_on && error_n < 0.0f) {
        float keep = 1.0f - dir_lr_ * std::min(-error_n, 1.0f);
        belief_x_ *= keep; belief_y_ *= keep;
        float bmag = std::sqrt(belief_x_ * belief_x_ + belief_y_ * belief_y_);
        last_R_  = std::clamp(bmag / (belief_absw_ + 1e-6f), 0.0f, 1.0f);
        last_mu_ = std::atan2(belief_y_, belief_x_);
    }

    // tumble probability (NO orthokinesis crank): low while scent rises, high while it falls.
    float p_tumble = std::clamp(tumble_base_ - tumble_gain_ * error_n, tumble_min_, tumble_max_);
    last_p_tumble_ = p_tumble;

    // --- capability (arbiter-facing; policy stays reward-free) ---
    if (eat_pending_) {
        float s = eat_scent_sample_;
        if (!have_eat_scent_) { eat_scent_ = s; have_eat_scent_ = true; }
        else                  eat_scent_ += eat_scent_alpha_ * (s - eat_scent_);
        eat_pending_ = false;
    }
    scent_peak_ = std::max(smax_, scent_peak_ * (1.0f - peak_decay_));
    float cap_denom = have_eat_scent_ ? eat_scent_ : 1.0f;   // pre-eat: field pins each source to 1.0
    capability_ = std::clamp(smax_ / (cap_denom + 1e-4f), 0.0f, 1.0f);

    // --- KF1 run integrity + KF3 authority ---
    constexpr float kTurnExit = 0.6283f;   // ~36deg = "aligned enough to be running"
    bool muted = have_authority_ && (authority_ < authority_floor_);
    if (muted) { run_dir_abs_ = heading_; reorienting_ = false; run_ticks_ = 0; run_start_scent_ = smax_; }
    float pre_delta = wrap_pi(run_dir_abs_ - heading_);
    bool  in_turn   = std::fabs(pre_delta) > kTurnExit;
    if (reorienting_ && !in_turn) reorienting_ = false;    // the run is now underway
    bool  executing = !reorienting_;
    turn_frac_ += 0.02f * ((in_turn ? 1.0f : 0.0f) - turn_frac_);

    // --- KF2 efference-matched stuck: blocked = intending forward but achieving << capable speed ---
    bool forced = false;
    if (muted) {
        stuck_counter_ = 0;
    } else if (executing) {
        bool blocked = (vel_scale_ > 1e-4f) && (std::fabs(vel_fwd_) < stuck_frac_ * vel_scale_);
        if (blocked) ++stuck_counter_; else stuck_counter_ = 0;
    }   // reorienting: hold the counter (low speed is expected while turning)
    if (stuck_counter_ >= stuck_ticks_) { forced = true; stuck_counter_ = 0; ++forced_tumbles_; if (in_turn) ++forced_in_turn_; }

    // --- decide whether to tumble ---
    bool do_tumble;
    if (muted)          do_tumble = false;   // coasting: no decision, no belief credit
    else if (!executing) do_tumble = false;  // KF1: commit to reaching run_dir first
    else {
        std::uniform_real_distribution<float> u01(0.0f, 1.0f);
        do_tumble = grad_on ? (forced || u01(rng_) < p_tumble)
                            : (u01(rng_) < tumble_base_);   // Shuffle: gradient-blind random walk
    }

    if (do_tumble) {
        float run_delta = smax_ - run_start_scent_;
        float len = float(run_ticks_);
        if      (run_delta > 0.0f) run_len_up_   += 0.05f * (len - run_len_up_);
        else if (run_delta < 0.0f) run_len_down_ += 0.05f * (len - run_len_down_);

        // KF6: infer the up-gradient direction from THIS run's outcome (before choosing anew).
        if (belief_on) {
            float outcome_n = std::clamp(run_delta / (eff_scale + 1e-6f), -4.0f, 4.0f);
            if (ablation_ == Ablation::WrongSign) outcome_n = -outcome_n;   // ablation: bias toward BAD dirs
            float dir_for_update = run_dir_abs_;
            if (ablation_ == Ablation::ShuffleDir) {                        // ablation: break dir<->outcome
                std::uniform_real_distribution<float> ur(-kPi, kPi);
                dir_for_update = ur(rng_);
            }
            float ux = std::cos(dir_for_update), uy = std::sin(dir_for_update);
            belief_x_    += dir_lr_ * (outcome_n * ux - belief_x_);
            belief_y_    += dir_lr_ * (outcome_n * uy - belief_y_);
            belief_absw_ += dir_lr_ * (std::fabs(outcome_n) - belief_absw_);
            float bmag = std::sqrt(belief_x_ * belief_x_ + belief_y_ * belief_y_);
            last_R_  = std::clamp(bmag / (belief_absw_ + 1e-6f), 0.0f, 1.0f);
            last_mu_ = std::atan2(belief_y_, belief_x_);
        }

        // choose the new run direction: uniform off the current heading, centre biased toward mu and
        // cone tightened by the belief precision R (R=0 -> full uniform cone = kinesis floor).
        std::uniform_real_distribution<float> ut(-tumble_range_, tumble_range_);
        float u = ut(rng_);
        float centre = heading_;
        if (belief_on && last_R_ > 0.0f) {
            centre = circ_interp(heading_, last_mu_, last_R_);
            u *= (1.0f - 0.5f * last_R_);
        }
        run_dir_abs_ = wrap_pi(centre + u);
        ++tumble_count_; committed_action_ = 1;
        run_ticks_ = 0; run_start_scent_ = smax_;
        reorienting_ = true;   // KF1: commit to reaching the new direction
    } else {
        ++run_count_; committed_action_ = 0;
        ++run_ticks_;
    }

    // egocentric bearing to the committed run direction + turn-commit latch (anti-antipode-dither)
    float delta = wrap_pi(run_dir_abs_ - heading_);
    if (!turning_ && std::fabs(delta) > 1.5708f)     { turning_ = true; turn_dir_ = (delta >= 0.0f) ? 1.0f : -1.0f; }
    else if (turning_ && std::fabs(delta) < 0.6283f) { turning_ = false; }
    if (turning_) delta = turn_dir_ * std::min(std::fabs(delta), 0.92f * kPi);
    out_vx_ = std::sin(delta);
    out_vy_ = std::cos(delta);

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("runtumble2") : id_;
    out->sensor      = "runtumble_heading";
    out->values.resize(2);
    out->values[0] = out_vx_;
    out->values[1] = out_vy_;
    bus_->publish(output_topic_, out);

    if (!confidence_topic_.empty()) {
        auto cap = std::make_shared<ProprioToken>();
        cap->tick_id     = tick_id;
        cap->producer_id = id_.empty() ? std::string("runtumble2") : id_;
        cap->sensor      = "klino_confidence";
        cap->values.resize(1);
        cap->values[0]   = capability_;
        bus_->publish(confidence_topic_, cap);
    }
}

nlohmann::json RunTumbleNavV2::diag_snapshot() const {
    return nlohmann::json{
        {"baseline", baseline_},
        {"error", last_error_},
        {"p_tumble", last_p_tumble_},
        {"action", committed_action_},
        {"runs", run_count_}, {"tumbles", tumble_count_}, {"forced", forced_tumbles_},
        {"vx", out_vx_}, {"vy", out_vy_},
        {"smax", smax_},
        {"cap", capability_},
        {"eat_scent", eat_scent_},
        {"have_eat_scent", have_eat_scent_},
        {"nfloor", noise_floor_},        // KF4 stationary-noise floor
        {"vscale", vel_scale_},          // KF2/KF4 learned speed scale
        {"run_len_up", run_len_up_},
        {"run_len_down", run_len_down_},
        {"turn_frac", turn_frac_},
        {"forced_in_turn", forced_in_turn_},
        {"reorienting", reorienting_},
        {"dir_R", last_R_},              // KF6 belief precision (kappa proxy); 0 = kinesis floor
        {"dir_mu", last_mu_},
        {"authority", authority_},
        {"muted", muted()},
        {"ablation", ablation_name(ablation_)},
    };
}

}  // namespace ogma
