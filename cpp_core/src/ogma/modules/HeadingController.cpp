// =============================================================================
// HeadingController.cpp  --  learned heading-following controller
// =============================================================================
#include "ogma/modules/HeadingController.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

#include <variant>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
constexpr float kPi    = 3.14159265358979323846f;
constexpr float kInvPi = 1.0f / kPi;

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("HeadingController param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("HeadingController param '" + key + "' must be integer");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("HeadingController param '" + key + "' must be string");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("HeadingController param '" + key + "' must be bool");
}
}

HeadingController::HeadingController()  = default;
HeadingController::~HeadingController() = default;

std::string_view HeadingController::type_name() const { return "HeadingController"; }

std::vector<TopicSpec> HeadingController::input_topics() const {
    std::vector<TopicSpec> v{ TopicSpec{input_topic_, std::type_index(typeid(ProprioToken))} };
    if (!speed_gate_topic_.empty())
        v.push_back(TopicSpec{speed_gate_topic_, std::type_index(typeid(ProprioToken))});
    return v;
}

std::vector<TopicSpec> HeadingController::output_topics() const {
    return { TopicSpec{steer_topic_,  std::type_index(typeid(ActionOut))},
             TopicSpec{thrust_topic_, std::type_index(typeid(ActionOut))} };
}

ParamSchema HeadingController::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly,
            "Egocentric desired-heading ProprioToken [cx=+right, cy=+forward] "
            "(e.g. percept.scent_compass = up-gradient bearing).",
            ParamValue{std::string("percept.scent_compass")}},
        {"cx_index", ParamMutability::ConstructionOnly,
            "Index of the +right (lateral) component in the input token.", ParamValue{int64_t{0}}},
        {"cy_index", ParamMutability::ConstructionOnly,
            "Index of the +forward component in the input token.", ParamValue{int64_t{1}}},
        {"steer_output_topic",  ParamMutability::ConstructionOnly,
            "ActionOut turn topic (→ MotorBus steer channel).", ParamValue{std::string("cog.steer")}},
        {"thrust_output_topic", ParamMutability::ConstructionOnly,
            "ActionOut advance topic (→ MotorBus thrust channel).", ParamValue{std::string("cog.thrust")}},
        {"ang_vel_topic", ParamMutability::ConstructionOnly,
            "Body yaw-rate ProprioToken (rad/tick) — the clean steer-driven rotation "
            "signal the controller learns its turn response k_body=|ω|/|steer| from.",
            ParamValue{std::string("reality.proprio.ang_vel")}},
        {"gain_init", ParamMutability::ConstructionOnly,
            "Fallback turn gain before k_body is learned (NOT a behavioral knob).", ParamValue{0.5}},
        {"gain_lr", ParamMutability::HotMutable,
            "EMA rate for learning k_body (the body turn-response model).", ParamValue{0.05}},
        {"turn_fraction", ParamMutability::HotMutable,
            "Fraction of the heading error to null per tick (control stability margin).", ParamValue{0.6}},
        {"gain_min", ParamMutability::HotMutable, "Turn-gain floor (safety rail).", ParamValue{0.05}},
        {"gain_max", ParamMutability::HotMutable, "Turn-gain ceiling (safety rail).", ParamValue{4.0}},
        {"fixed_gain", ParamMutability::HotMutable,
            "FIXED turn gain (>0) — bypasses the online k_body estimate entirely. The body's turn "
            "response is a CONSTANT; the k_body=|omega|/|steer| estimate is contaminated by rotation "
            "the controller didn't cause (tumbles, reflex spins) and inflates → g pins at gain_min → "
            "steer collapses → wide arcs / no turn authority. Set fixed_gain to the true constant "
            "(~2-3) to restore steer authority so the differential can reach the tight-arc / pivot "
            "regime. 0 (default) = use the learned k_body inversion (behaviour unchanged).", ParamValue{0.0}},
        {"turn_commit", ParamMutability::HotMutable,
            "COMMITTED-turn saturation (>0). The proportional steer=g*bearing stays weak at small "
            "bearings → tiny differential → wide arcs; a self-directed forager rarely commands a hard "
            "turn. With turn_commit>0 the steer SATURATES: steer = max_steer*tanh(turn_commit*bearing), "
            "so a MODERATE bearing error already drives steer toward max_steer → the differential "
            "reaches the one-paddle TIGHT-ARC regime. Eases to 0 at alignment (tanh(0)=0, no chatter). "
            "~2-3 = decisive turns; higher = sharper (risk overshoot). 0 (default) = proportional (unchanged).",
            ParamValue{0.0}},
        {"speed_gate_topic", ParamMutability::ConstructionOnly,
            "ORTHOKINESIS: a scalar MAGNITUDE ∈[0,1] (e.g. klino's self-calibrated cap = scent/eat_scent) "
            "that SLOWS the advance as it rises → the bug crawls in its own eating range and dwells on "
            "food instead of barreling through (the change-only loop derives a HEADING, never HOW-CLOSE; "
            "this adds the location channel). thrust *= floor + (1−floor)·(1−gate). Empty (default) = off.",
            ParamValue{std::string("")}},
        {"speed_gate_floor", ParamMutability::HotMutable,
            "Minimum crawl-speed fraction at gate=1 (so it doesn't freeze on the food and can still creep "
            "the mouth on). The ONE constant of the speed gate — a min locomotor speed. Live slider.",
            ParamValue{0.15}},
        {"max_steer",  ParamMutability::HotMutable, "cog.steer output range.",  ParamValue{4.0}},
        {"max_thrust", ParamMutability::HotMutable, "cog.thrust output range.", ParamValue{4.0}},
        {"min_signal", ParamMutability::HotMutable,
            "|gradient| below this → no confident heading → no advance.", ParamValue{0.1}},
        {"align_angle_deg", ParamMutability::HotMutable,
            "Advance at full thrust only when facing within this angle; brake (reverse) "
            "when more off-axis → turn in place to face, then charge (anti-orbit). "
            "(Used only when learn_advance=false — the hand-designed gate.)", ParamValue{30.0}},
        {"forward_only", ParamMutability::HotMutable,
            "true → never REVERSE to navigate: off-axis → STOP (steer rotates in place), "
            "facing → forward. Fixes reversing-into-walls (the maze backing-up). Applies to "
            "both the hand gate (clamp ≥0) and the learned advance (thrust ∈ [0,max]).",
            ParamValue{false}},
        {"reverse_brake_only", ParamMutability::HotMutable,
            "Keep reverse but only as a BRAKE: a reverse command applies while the body has "
            "forward momentum (decelerate to stop), then clamps to 0 (turn in place) — never "
            "a several-second backward drive (the post-eat wall-backing). Needs vel_topic. "
            "Preferred over forward_only. Default false.", ParamValue{false}},
        {"authority_topic", ParamMutability::ConstructionOnly,
            "MotorBus per-channel authority (ProprioToken scalar ∈[0,1]). When a reflex "
            "subsumes this controller on the bus, the cog's authority drops → the advance "
            "learning rate is scaled by it (reflex-driven motion isn't miscredited). Empty = "
            "authority 1 = full learning (byte-identical).", ParamValue{std::string("")}},
        {"learn_advance", ParamMutability::ConstructionOnly,
            "true → LEARN the thrust policy (UCB over thrust levels, reward = forward "
            "progress along the commanded heading) instead of the hand-designed cos gate. "
            "brake-turn-charge emerges. Default false = hand gate (byte-identical).", ParamValue{false}},
        {"vel_topic", ParamMutability::ConstructionOnly,
            "Egocentric afferent velocity ProprioToken [v_right, v_forward] (move_speed-norm) "
            "— the food-independent motion signal the advance reward is built from.",
            ParamValue{std::string("reality.proprio.vel_ego")}},
        {"n_err_bins", ParamMutability::ConstructionOnly,
            "Heading-error |bearing| bins (advance value-table rows).", ParamValue{int64_t{4}}},
        {"n_thrust_acts", ParamMutability::ConstructionOnly,
            "Thrust levels spanning [-max_thrust,+max_thrust] (advance value-table cols; "
            "3 = reverse/stop/forward).", ParamValue{int64_t{3}}},
        {"advance_lr", ParamMutability::HotMutable,
            "EMA rate for the learned advance value table.", ParamValue{0.1}},
        {"ucb_c", ParamMutability::HotMutable,
            "UCB exploration weight for advance action selection (self-annealing — the "
            "adaptive mechanism, not a tuned epsilon).", ParamValue{0.4}},
        {"effort_cost", ParamMutability::HotMutable,
            "Energy prior: cost on |vel_fwd| in the advance reward. Breaks the stop≡reverse "
            "tie off-axis toward STOP (brake = stop, not back away). Metabolic, not behavioral.",
            ParamValue{0.15}},
        {"advance_homeokinetic", ParamMutability::HotMutable,
            "true: advance reward = max(0,vel_fwd) − effort·|vel_fwd| — INTRINSIC controllable-"
            "forward-motion, decoupled from food, so a bad heading can't punish the table into "
            "collapse. false (legacy): ×cos(bearing), food-coupled (collapses when off-axis).",
            ParamValue{false}},
    };
}

ParamMap HeadingController::current_params() const {
    ParamMap m;
    m["input_topic"]         = ParamValue{input_topic_};
    m["cx_index"]            = ParamValue{int64_t(cx_index_)};
    m["cy_index"]            = ParamValue{int64_t(cy_index_)};
    m["steer_output_topic"]  = ParamValue{steer_topic_};
    m["thrust_output_topic"] = ParamValue{thrust_topic_};
    m["ang_vel_topic"]       = ParamValue{ang_vel_topic_};
    m["gain_init"]           = ParamValue{double(gain_init_)};
    m["gain_lr"]             = ParamValue{double(gain_lr_)};
    m["turn_fraction"]       = ParamValue{double(turn_fraction_)};
    m["gain_min"]            = ParamValue{double(gain_min_)};
    m["gain_max"]            = ParamValue{double(gain_max_)};
    m["fixed_gain"]          = ParamValue{double(fixed_gain_)};
    m["turn_commit"]         = ParamValue{double(turn_commit_)};
    m["speed_gate_topic"]    = ParamValue{speed_gate_topic_};
    m["speed_gate_floor"]    = ParamValue{double(speed_gate_floor_)};
    m["max_steer"]           = ParamValue{double(max_steer_)};
    m["max_thrust"]          = ParamValue{double(max_thrust_)};
    m["min_signal"]          = ParamValue{double(min_signal_)};
    m["align_angle_deg"]     = ParamValue{double(align_angle_deg_)};
    m["forward_only"]        = ParamValue{forward_only_};
    m["reverse_brake_only"]  = ParamValue{reverse_brake_only_};
    m["authority_topic"]     = ParamValue{authority_topic_};
    m["learn_advance"]       = ParamValue{learn_advance_};
    m["vel_topic"]           = ParamValue{vel_topic_};
    m["n_err_bins"]          = ParamValue{int64_t(n_err_bins_)};
    m["n_thrust_acts"]       = ParamValue{int64_t(n_thrust_acts_)};
    m["advance_lr"]          = ParamValue{double(advance_lr_)};
    m["ucb_c"]               = ParamValue{double(ucb_c_)};
    m["effort_cost"]         = ParamValue{double(effort_cost_)};
    m["advance_homeokinetic"] = ParamValue{advance_homeokinetic_};
    return m;
}

void HeadingController::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("HeadingController requires a non-null Bus");

    apply_param(params, "input_topic",   [&](auto const& v){ input_topic_ = get_string(v, "input_topic"); });
    apply_param(params, "cx_index",      [&](auto const& v){ cx_index_ = int(get_int(v, "cx_index")); });
    apply_param(params, "cy_index",      [&](auto const& v){ cy_index_ = int(get_int(v, "cy_index")); });
    apply_param(params, "steer_output_topic",  [&](auto const& v){ steer_topic_  = get_string(v, "steer_output_topic"); });
    apply_param(params, "thrust_output_topic", [&](auto const& v){ thrust_topic_ = get_string(v, "thrust_output_topic"); });
    apply_param(params, "ang_vel_topic",  [&](auto const& v){ ang_vel_topic_ = get_string(v, "ang_vel_topic"); });
    apply_param(params, "gain_init",  [&](auto const& v){ gain_init_= float(get_double(v, "gain_init")); });
    apply_param(params, "gain_lr",    [&](auto const& v){ gain_lr_  = float(get_double(v, "gain_lr")); });
    apply_param(params, "turn_fraction", [&](auto const& v){ turn_fraction_ = float(get_double(v, "turn_fraction")); });
    apply_param(params, "gain_min",   [&](auto const& v){ gain_min_ = float(get_double(v, "gain_min")); });
    apply_param(params, "gain_max",   [&](auto const& v){ gain_max_ = float(get_double(v, "gain_max")); });
    apply_param(params, "fixed_gain", [&](auto const& v){ fixed_gain_ = float(get_double(v, "fixed_gain")); });
    apply_param(params, "turn_commit", [&](auto const& v){ turn_commit_ = float(get_double(v, "turn_commit")); });
    apply_param(params, "speed_gate_topic", [&](auto const& v){ speed_gate_topic_ = get_string(v, "speed_gate_topic"); });
    apply_param(params, "speed_gate_floor", [&](auto const& v){ speed_gate_floor_ = float(get_double(v, "speed_gate_floor")); });
    apply_param(params, "max_steer",  [&](auto const& v){ max_steer_  = float(get_double(v, "max_steer")); });
    apply_param(params, "max_thrust", [&](auto const& v){ max_thrust_ = float(get_double(v, "max_thrust")); });
    apply_param(params, "min_signal", [&](auto const& v){ min_signal_ = float(get_double(v, "min_signal")); });
    apply_param(params, "align_angle_deg", [&](auto const& v){
        align_angle_deg_ = float(get_double(v, "align_angle_deg"));
        align_cos_ = std::cos(align_angle_deg_ * kPi / 180.0f); });
    apply_param(params, "forward_only", [&](auto const& v){ forward_only_ = get_bool(v, "forward_only"); });
    apply_param(params, "reverse_brake_only", [&](auto const& v){ reverse_brake_only_ = get_bool(v, "reverse_brake_only"); });
    apply_param(params, "authority_topic", [&](auto const& v){ authority_topic_ = get_string(v, "authority_topic"); });
    apply_param(params, "learn_advance", [&](auto const& v){ learn_advance_ = get_bool(v, "learn_advance"); });
    apply_param(params, "vel_topic",     [&](auto const& v){ vel_topic_     = get_string(v, "vel_topic"); });
    apply_param(params, "n_err_bins",    [&](auto const& v){ n_err_bins_    = std::max(1, int(get_int(v, "n_err_bins"))); });
    apply_param(params, "n_thrust_acts", [&](auto const& v){ n_thrust_acts_ = std::max(2, int(get_int(v, "n_thrust_acts"))); });
    apply_param(params, "advance_lr",    [&](auto const& v){ advance_lr_    = float(get_double(v, "advance_lr")); });
    apply_param(params, "ucb_c",         [&](auto const& v){ ucb_c_         = float(get_double(v, "ucb_c")); });
    apply_param(params, "effort_cost",   [&](auto const& v){ effort_cost_   = float(get_double(v, "effort_cost")); });
    apply_param(params, "advance_homeokinetic", [&](auto const& v){ advance_homeokinetic_ = get_bool(v, "advance_homeokinetic"); });

    if (learn_advance_) {
        adv_value_.assign(size_t(n_err_bins_ * n_thrust_acts_), 0.0f);
        adv_visits_.assign(size_t(n_err_bins_ * n_thrust_acts_), 0);
        adv_bin_visits_.assign(size_t(n_err_bins_), 0);
    }

    sub_ids_.push_back(bus_->subscribe(input_topic_, SubscriptionKind::Direct,
        [this](std::string_view /*topic*/, MessagePtr p){ handle_compass(p); }));
    if (!ang_vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(ang_vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_ang_vel(p); }));
    if ((learn_advance_ || reverse_brake_only_) && !vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_vel_ego(p); }));
    if (!authority_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(authority_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_authority(p); }));
    if (!speed_gate_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(speed_gate_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_speed_gate(p); }));
}

void HeadingController::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "gain_lr")       gain_lr_       = float(get_double(value, k));
    else if (k == "turn_fraction") turn_fraction_ = float(get_double(value, k));
    else if (k == "gain_min")      gain_min_      = float(get_double(value, k));
    else if (k == "gain_max")      gain_max_      = float(get_double(value, k));
    else if (k == "fixed_gain")    fixed_gain_    = float(get_double(value, k));
    else if (k == "turn_commit")   turn_commit_   = float(get_double(value, k));
    else if (k == "speed_gate_floor") speed_gate_floor_ = float(get_double(value, k));
    else if (k == "max_steer")     max_steer_     = float(get_double(value, k));
    else if (k == "max_thrust")    max_thrust_    = float(get_double(value, k));
    else if (k == "min_signal")    min_signal_    = float(get_double(value, k));
    else if (k == "align_angle_deg") { align_angle_deg_ = float(get_double(value, k));
                                       align_cos_ = std::cos(align_angle_deg_ * kPi / 180.0f); }
    else if (k == "forward_only")  forward_only_  = get_bool(value, k);
    else if (k == "reverse_brake_only") reverse_brake_only_ = get_bool(value, k);
    else if (k == "advance_lr")    advance_lr_    = float(get_double(value, k));
    else if (k == "ucb_c")         ucb_c_         = float(get_double(value, k));
    else if (k == "effort_cost")   effort_cost_   = float(get_double(value, k));
    else if (k == "advance_homeokinetic") advance_homeokinetic_ = get_bool(value, k);
    else throw std::invalid_argument("HeadingController: param '" + k +
                                     "' is construction-only / unknown");
}

void HeadingController::handle_compass(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int n = int(pt->values.size());
    compass_x_ = (cx_index_ < n) ? float(pt->values[cx_index_]) : 0.0f;
    compass_y_ = (cy_index_ < n) ? float(pt->values[cy_index_]) : 0.0f;
}

void HeadingController::handle_ang_vel(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) latest_ang_vel_ = float(pt->values[0]);
}

void HeadingController::handle_vel_ego(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    if (pt->values.size() > 0) vel_right_ = float(pt->values[0]);
    if (pt->values.size() > 1) vel_fwd_   = float(pt->values[1]);
}

void HeadingController::handle_authority(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0)
        latest_authority_ = std::clamp(float(pt->values[0]), 0.0f, 1.0f);
}

void HeadingController::handle_speed_gate(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { speed_gate_ = float(pt->values[0]); have_speed_gate_ = true; }
}

void HeadingController::tick(uint64_t tick_id) {
    float cx = compass_x_, cy = compass_y_;
    float mag = std::sqrt(cx * cx + cy * cy);
    bool  nav_on = (mag > min_signal_);

    // Egocentric bearing to the desired heading, scale-invariant in [-1,1]
    // (0 = facing it, +1 = directly behind on the right, etc.). atan2(x,y) so
    // forward (cy) is the zero axis and +right (cx) is positive.
    float bearing = nav_on ? std::atan2(cx, cy) * kInvPi : 0.0f;

    // --- LEARN the body's turn response (a forward model) -------------------
    // Pair the steer we commanded last tick with the yaw rate ω it produced
    // (read this tick).  k_body = EMA(|ω| / |steer|) = turn-rate per unit steer —
    // a clean, purely STEER-DRIVEN signal (translation never rotates the body), so
    // it converges to the body's true turn gain instead of railing.  Only update
    // when the prior steer was significant (signal-to-noise).
    if (have_prev_ && std::fabs(prev_steer_) > 0.1f) {
        float k = std::fabs(latest_ang_vel_) / std::fabs(prev_steer_);
        if (k_body_ <= 0.0f) k_body_ = k;                       // first sample
        else                 k_body_ += gain_lr_ * (k - k_body_);
    }

    // --- command: turn toward the heading, advance when aligned -------------
    // Invert the learned body model: to null a fraction turn_fraction_ of the
    // heading error this tick, steer = turn_fraction_ * bearing_angle / k_body.
    // Before k_body is learned, fall back to gain_init_.  g clamped for safety.
    // fixed_gain>0 bypasses the CONTAMINATED online k_body estimate (tumbles/reflex spins inflate
    // it → g pins at the floor → steer collapses → wide arcs). The body turn-response is constant.
    float g = (fixed_gain_ > 0.0f)
                  ? fixed_gain_
                  : ((k_body_ > 1e-4f) ? std::clamp(turn_fraction_ / k_body_, gain_min_, gain_max_)
                                       : gain_init_);
    last_gain_ = g;
    // turn_commit>0: SATURATING (committed) turn so a moderate bearing already drives steer toward
    // max_steer → the differential reaches the one-paddle tight-arc regime (the proportional g*bearing
    // stays weak at small bearings → wide arcs). tanh eases to 0 at alignment (no chatter).
    float steer;
    if (!nav_on)                    steer = 0.0f;
    else if (turn_commit_ > 0.0f)   steer = max_steer_ * std::tanh(turn_commit_ * bearing);
    else                            steer = std::clamp(g * bearing, -max_steer_, max_steer_);
    float thrust = 0.0f;
    if (learn_advance_) {
        // --- LEARNED advance: UCB over thrust levels, reward = forward progress ---
        // along the commanded heading (food-independent).  brake-turn-charge emerges:
        // off-axis → forward scores ≤0 (max(0,vel_fwd)*cos(bearing) with cos≤0), so
        // reverse/stop win → the body brakes & the learned-gain steer rotates it to
        // face, then forward scores + → it charges.
        if (nav_on) {
            float cosb = std::cos(bearing * kPi);                   // +1 face, 0 @90°, -1 behind
            auto AIDX = [&](int b, int a){ return b * n_thrust_acts_ + a; };
            // (1) credit the PREVIOUS action with the velocity it produced this tick.
            //     reward = forward-progress along the commanded heading − energy cost.
            //     The effort term breaks the stop≡reverse tie off-axis toward STOP
            //     (cos≤0 ⇒ forward<0, reverse=−effort·|v|, stop=0 wins) and makes the
            //     brake = stop instead of backing away; when facing, the forward-
            //     progress term dominates.
            if (have_prev_adv_) {
                // HOMEOKINETIC: intrinsic controllable-forward-motion (food-decoupled) so a
                // bad heading can't punish the advance into collapse.  Legacy: food-coupled.
                float reward = advance_homeokinetic_
                    ? (std::max(0.0f, vel_fwd_) - effort_cost_ * std::fabs(vel_fwd_))
                    : (std::max(0.0f, vel_fwd_) * prev_cmd_cosb_ - effort_cost_ * std::fabs(vel_fwd_));
                float& v = adv_value_[size_t(AIDX(prev_err_bin_, prev_thrust_act_))];
                // authority-gate: scale the learning rate by the cog's bus authority so
                // reflex-driven (subsumed) motion isn't miscredited to the advance policy.
                v += (advance_lr_ * latest_authority_) * (reward - v);
                last_adv_reward_ = reward;
            }
            // (2) select this tick's thrust by UCB on the current heading-error bin.
            int err_bin = std::min(n_err_bins_ - 1,
                                   int(std::floor(std::fabs(bearing) * float(n_err_bins_))));
            int   N        = adv_bin_visits_[size_t(err_bin)];
            int   best_a   = 0;
            float best_scr = -1e30f;
            for (int a = 0; a < n_thrust_acts_; ++a) {
                int   na  = adv_visits_[size_t(AIDX(err_bin, a))];
                float ucb = ucb_c_ * std::sqrt(std::log(float(N) + 1.0f) / float(na + 1));
                float scr = adv_value_[size_t(AIDX(err_bin, a))] + ucb;
                if (scr > best_scr) { best_scr = scr; best_a = a; }
            }
            // map action → thrust level.  forward_only: [0,max] (stop … forward, no
            // reverse); else [-max,+max] (reverse … forward).
            float frac = (n_thrust_acts_ > 1) ? float(best_a) / float(n_thrust_acts_ - 1) : 1.0f;
            thrust = forward_only_ ? (max_thrust_ * frac)
                                   : (-max_thrust_ + 2.0f * max_thrust_ * frac);
            // record for next-tick credit + bump visits
            prev_err_bin_   = err_bin; prev_thrust_act_ = best_a; prev_cmd_cosb_ = cosb;
            have_prev_adv_  = true;
            adv_visits_[size_t(AIDX(err_bin, best_a))] += 1;
            adv_bin_visits_[size_t(err_bin)]           += 1;
            // telemetry: spread of the current err-bin row + coverage
            float lo = 1e30f, hi = -1e30f; int seen = 0, cells = n_err_bins_ * n_thrust_acts_;
            for (int a = 0; a < n_thrust_acts_; ++a) {
                float vv = adv_value_[size_t(AIDX(err_bin, a))];
                lo = std::min(lo, vv); hi = std::max(hi, vv);
            }
            for (int c = 0; c < cells; ++c) if (adv_visits_[size_t(c)] > 0) ++seen;
            last_err_bin_ = err_bin; last_thrust_act_ = best_a;
            last_adv_spread_ = (hi > lo) ? (hi - lo) : 0.0f;
            last_adv_cov_    = cells > 0 ? float(seen) / float(cells) : 0.0f;
        } else {
            have_prev_adv_ = false;   // don't credit across a nav gap
        }
    } else if (nav_on) {
        // --- hand-designed cos gate (default; byte-identical for existing configs) -
        // advance ONLY when nearly facing; BRAKE (reverse) when off-axis → turn IN
        // PLACE to face, then charge (no arc/orbit).  gate = (cos(bearing)-align_cos)/
        // (1-align_cos): 1 when facing, 0 at align_angle, NEGATIVE beyond.
        float c    = std::cos(bearing * kPi);                       // +1 face, 0 @90°, -1 behind
        float gate = (c - align_cos_) / std::max(1e-3f, 1.0f - align_cos_);
        // forward_only: off-axis → STOP (not reverse) → never backs into a wall.
        float lo = forward_only_ ? 0.0f : -1.0f;
        thrust = max_thrust_ * std::clamp(gate, lo, 1.0f);
    }

    // --- reverse = BRAKE only: a reverse command brakes forward momentum, but once the
    // body has stopped it holds (turn in place) instead of driving backward for several
    // seconds (the long post-eat reverse that backs into walls).  Keeps reverse in the
    // vocabulary (braking) without the backward journey.
    if (reverse_brake_only_ && thrust < 0.0f && vel_fwd_ <= 0.0f)
        thrust = 0.0f;

    // ORTHOKINESIS: slow the advance as the scent MAGNITUDE (klino's self-calibrated cap) rises →
    // crawl in the eating range so the mouth can land + DWELL on food (the change-only loop gives a
    // heading, not how-close). thrust *= floor + (1−floor)·(1−gate): gate=0 far → full, gate=1 → floor.
    // Also the recovery fix: after a near-miss the CHANGE signal habituates but the MAGNITUDE stays
    // high → speed stays low → the bug lingers near food instead of driving away (emergent ratchet).
    last_speed_scale_ = 1.0f;
    if (have_speed_gate_) {
        last_speed_scale_ = speed_gate_floor_ + (1.0f - speed_gate_floor_) * (1.0f - std::clamp(speed_gate_, 0.0f, 1.0f));
        thrust *= last_speed_scale_;
    }

    // Wedge recovery is no longer in-controller: the reflex layer (WhiskerSteerReflex +
    // StuckEscapeReflex) handles it via MotorBus subsumption, and the cog's learning is
    // authority-gated so reflex-driven motion isn't miscredited.

    auto st = std::make_shared<ActionOut>();
    st->tick_id = tick_id; st->producer_id = id_.empty() ? std::string("heading_controller") : id_;
    st->accel = steer; st->source = "heading_controller"; st->probe = false;
    bus_->publish(steer_topic_, st);

    auto th = std::make_shared<ActionOut>();
    th->tick_id = tick_id; th->producer_id = st->producer_id;
    th->accel = thrust; th->source = "heading_controller"; th->probe = false;
    bus_->publish(thrust_topic_, th);

    prev_bearing_ = bearing; prev_steer_ = steer; have_prev_ = nav_on;
    last_bearing_ = bearing; last_steer_ = steer; last_thrust_ = thrust; last_nav_on_ = nav_on;
    ++tick_count_;
}

nlohmann::json HeadingController::snapshot_state() const {
    nlohmann::json j{{"version", 1}, {"k_body", k_body_}};
    if (learn_advance_) {
        j["adv_value"]      = adv_value_;
        j["adv_visits"]     = adv_visits_;
        j["adv_bin_visits"] = adv_bin_visits_;
    }
    return j;
}

nlohmann::json HeadingController::diag_snapshot() const {
    nlohmann::json j{
        {"bearing", last_bearing_}, {"gain", last_gain_}, {"k_body", k_body_},
        {"steer", last_steer_}, {"thrust", last_thrust_}, {"nav_on", last_nav_on_},
    };
    if (learn_advance_) {
        j["learn_advance"] = true;
        j["err_bin"]       = last_err_bin_;
        j["thrust_act"]    = last_thrust_act_;
        j["adv_reward"]    = last_adv_reward_;
        j["adv_spread"]    = last_adv_spread_;
        j["adv_cov"]       = last_adv_cov_;
        // full learned advance policy table (for the xaq_inspector grid)
        j["n_err_bins"]    = n_err_bins_;
        j["n_thrust_acts"] = n_thrust_acts_;
        j["max_thrust"]    = max_thrust_;
        j["vel_fwd"]       = vel_fwd_;
        j["adv_value"]     = adv_value_;    // flat [n_err_bins * n_thrust_acts] EMA(reward)
        j["adv_visits"]    = adv_visits_;   // flat visit counts (UCB / confidence)
    }
    return j;
}

void HeadingController::restore_state(nlohmann::json const& s) {
    if (s.contains("k_body")) k_body_ = s.value("k_body", k_body_);
    if (learn_advance_) {
        if (s.contains("adv_value")      && s["adv_value"].size()      == adv_value_.size())
            adv_value_      = s["adv_value"].get<std::vector<float>>();
        if (s.contains("adv_visits")     && s["adv_visits"].size()     == adv_visits_.size())
            adv_visits_     = s["adv_visits"].get<std::vector<int>>();
        if (s.contains("adv_bin_visits") && s["adv_bin_visits"].size() == adv_bin_visits_.size())
            adv_bin_visits_ = s["adv_bin_visits"].get<std::vector<int>>();
    }
}

} // namespace ogma
