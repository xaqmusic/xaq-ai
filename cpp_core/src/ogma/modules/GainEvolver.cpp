#include "ogma/modules/GainEvolver.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

#include "ogma/Bus.hpp"

namespace ogma {

namespace {
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);   // set_param sends ints as int64
    throw std::invalid_argument("GainEvolver param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("GainEvolver param '" + key + "' must be integer");
}
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
// upright must climb back above this before another fall can be counted (one
// count per excursion; the body's auto-reset teleports upright to ~1, which
// clears the latch — that is the intended behavior: reset ends the excursion).
constexpr float kFallRecover = 0.5f;
constexpr int   kAcceptLogMax = 24;
} // namespace

void GainEvolver::WindowStats::reset(int n_legs) {
    falls = 0; up_sum = up_sq = dwell_sum = torque_sum = 0.0; meas_n = distress_hits = 0; flow_q_sum = 0.0;
    flow_mag_sum = flow_pred_sum = flow_turn_sum = 0.0;
    td.assign(size_t(n_legs), 0); unloaded.assign(size_t(n_legs), 0);
}

GainEvolver::GainEvolver()  = default;
GainEvolver::~GainEvolver() = default;

std::string_view GainEvolver::type_name() const { return "GainEvolver"; }

std::vector<TopicSpec> GainEvolver::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const* t : {&upright_topic_, &distress_topic_, &foot_load_topic_,
                          &foot_contact_topic_, &imu_topic_, &gyro_topic_, &torque_topic_})
        if (!t->empty())
            v.emplace_back(*t, std::type_index(typeid(ProprioToken)),
                           SubscriptionKind::Direct, /*required=*/false);
    return v;
}

std::vector<TopicSpec> GainEvolver::output_topics() const {
    if (gain_topic_.empty()) return {};
    return {{gain_topic_, std::type_index(typeid(GainVector)),
             SubscriptionKind::Direct, /*required=*/false}};
}

ParamSchema GainEvolver::params_schema() const {
    return {
        // ---- the declared vector (parallel arrays; anything not listed here
        // stays a hand knob) --------------------------------------------------
        {"gain_keys", ParamMutability::ConstructionOnly,
         "Consumer param keys the evolver owns (parallel with gain_seed/min/max/sigma_scale). The consumer verifies each landing by read-back — a typo'd key shows up in its gains_rejected counter, never silently.",
         std::nullopt, std::nullopt, std::nullopt},
        {"gain_seed", ParamMutability::ConstructionOnly,
         "Initial incumbent vector — the operating point the search starts from (PART IV v1: the operator's baked rear-sequence point).",
         std::nullopt, std::nullopt, std::nullopt},
        {"gain_min", ParamMutability::ConstructionOnly, "Per-gain lower bounds (mutation clamps).",
         std::nullopt, std::nullopt, std::nullopt},
        {"gain_max", ParamMutability::ConstructionOnly, "Per-gain upper bounds (mutation clamps).",
         std::nullopt, std::nullopt, std::nullopt},
        {"gain_sigma_scale", ParamMutability::ConstructionOnly,
         "Per-gain mutation scale multiplier (default 1 each). Step σ_k = mutation_sigma · sigma_scale[k] · (max[k]−min[k]).",
         std::nullopt, std::nullopt, std::nullopt},
        {"gain_topic", ParamMutability::ConstructionOnly,
         "GainVector output topic; the consumer subscribes via ITS gain_topic param.",
         std::nullopt, std::nullopt, std::nullopt},
        // ---- inputs ----------------------------------------------------------
        {"upright_topic", ParamMutability::ConstructionOnly,
         "1-D basis.y.y (accelerometer gravity vector — egocentric-legal). Falls = debounced <thresh excursions; variance = the tilt-variance term.",
         std::nullopt, std::nullopt, std::nullopt},
        {"distress_topic", ParamMutability::ConstructionOnly,
         "1-D distress [0,1]. KNOWN v1 caveat: the body's stall half is world-frame (audit) and under-fires (unit bug) — consumed as-is; fixing the sensor is a separate lever.",
         std::nullopt, std::nullopt, std::nullopt},
        {"foot_load_topic", ParamMutability::ConstructionOnly,
         "Per-leg normal ground-reaction impulse EMA (FSR/servo-current analog — egocentric-legal). First consumer ever (2026-08-17).",
         std::nullopt, std::nullopt, std::nullopt},
        {"foot_contact_topic", ParamMutability::ConstructionOnly,
         "Per-leg touch flags — touchdown EDGES only (sim-only sensor; the hardware form is a load rising-edge, recorded in the plan).",
         std::nullopt, std::nullopt, std::nullopt},
        {"imu_topic", ParamMutability::ConstructionOnly,
         "Body imu token [sin yaw, cos yaw, fwd_v, ang_v]; values[2] feeds the flow-quality term (the one sanctioned speed-flavored input, a soft oracle by the audit — the charter's knowing exception) UNLESS travel_topic is set.",
         std::nullopt, std::nullopt, std::nullopt},
        {"travel_topic", ParamMutability::ConstructionOnly,
         "LEGAL travel input (PART V stage C1, ledger 2026-08-25). \"\" (default) = flow reads imu_topic values[2] — byte-identical to before this param existed. Set to reality.proprio.stride_v to feed the flow term the stance-FK ⊕ IMU estimate: values[1] (v_forward) of a [v_right, v_forward] token (or values[0] of a scalar). When set, imu_topic no longer writes the flow input; travel_rx_n in the diag is the consumer half of the two-sided meter.",
         std::nullopt, std::nullopt, std::nullopt},
        // ---- timing ----------------------------------------------------------
        {"n_legs", ParamMutability::ConstructionOnly, "legs (foot_load/contact dimensionality)",
         ParamValue{int64_t(4)}, ParamValue{int64_t(1)}, ParamValue{int64_t(8)}},
        {"seed", ParamMutability::ConstructionOnly,
         "RNG seed (named 'seed' so the OGMA_SEED master override rewrites it — dedicated stream, never the consumer's).",
         ParamValue{int64_t(0)}, std::nullopt, std::nullopt},
        {"warmup_ticks", ParamMutability::ConstructionOnly,
         "Ticks before the first incumbent window (body stand-up + EMA settling).",
         ParamValue{int64_t(10000)}, ParamValue{int64_t(0)}, ParamValue{int64_t(1000000)}},
        {"eval_window_ticks", ParamMutability::ConstructionOnly,
         "Ticks per evaluation window; continuous terms measure the BACK HALF only. Gate 2 MEASURED 4000 to be far too short (noise sd 1.4 vs signal 0.3); with the post-gate-2 criterion ~11k suffices, so the default is 12000.",
         ParamValue{int64_t(12000)}, ParamValue{int64_t(200)}, ParamValue{int64_t(1000000)}},
        {"republish_every", ParamMutability::ConstructionOnly,
         "If >0, re-send the active vector every N ticks inside a window (robustness against a late-joining consumer). 0 = boundary publishes only.",
         ParamValue{int64_t(0)}, ParamValue{int64_t(0)}, ParamValue{int64_t(100000)}},
        // ---- the search (HotMutable) -----------------------------------------
        {"mutation_sigma", ParamMutability::HotMutable,
         "THE LEVER. 0 = SILENT OBSERVER: evaluator scores windows for the instruments but publishes NOTHING, draws no RNG, mutates nothing — byte-identical behavior. >0 = live (1+1)-ES; the value seeds the self-annealing σ, and writing it overrides the anneal and clears its history.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{0.5}},
        // ---- criterion weights (HotMutable for between-run tuning; FIXED
        // during a judged run — the stationary evaluator) ----------------------
        {"w_falls", ParamMutability::HotMutable,
         "weight on debounced fall excursions. DEFAULT 0 — falls is guard-only (G1). Gate 2 measured it at 80.9% of J's variance: a rare discrete count cannot be a per-window gradient at any affordable window (134k ticks needed). Guard the catastrophe; the criterion optimizes its precursor (w_tilt_sd). >0 re-enables it and re-imports that variance.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{100.0}},
        {"w_tilt_sd", ParamMutability::HotMutable,
         "weight on sd(upright) over the back half — the CONTINUOUS precursor of a fall, sampled every tick. sd not variance: squaring let one tumble dominate quadratically (gate 2).",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{1000.0}},
        {"w_distress", ParamMutability::HotMutable,
         "weight on distress duty. DEFAULT 0: the body's distress signal is known-contaminated (world-frame stall half + a 50-vs-240Hz normalisation bug that makes it under-fire). Re-enable when that sensor is repaired — the param is kept precisely so the fix has somewhere to land.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{100.0}},
        {"w_unloaded", ParamMutability::HotMutable, "weight: mean per-leg unloaded-touchdown rate (ghost touches — femur-lifted landings that earn no load).",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{100.0}},
        {"w_flow", ParamMutability::HotMutable, "weight: (1 − flow_quality), flow = magnitude × predictability of fwd flow.",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{100.0}},
        {"torque_topic", ParamMutability::ConstructionOnly,
         "Per-servo load (reality.proprio.joint_torque): 12 floats normalized to MAX_SERVO_TORQUE. Servo current sensing — the audit's 'REAL LOAD SIGNAL', and on hardware this is battery current.",
         std::nullopt, std::nullopt, std::nullopt},
        {"w_energy", ParamMutability::HotMutable,
         "weight on ENERGY = mean |joint torque| over the measured region. Measured on the gate-2b windows: signal/noise 4.74 vs sd(upright) 0.61 and dwell 0.57 — ~8x better discrimination than anything else in this criterion, largely independent of it (corr -0.125), and agreeing with the flow term (+0.123) rather than fighting it. Default 4.0 — the weight every landscape/basin/tsweep verdict was measured at (an 8.0 default drifted from the configs' 4.0; aligned toward the VALIDATED value 2026-08-24). TWO standing caveats: the freeze trap is not disproven (flow is the counterweight; the per-leg minima guard blocks killing a leg to save current), and the sim signal is measured to be damping-dominated — mostly a joint-speed penalty, not electrical load (corr(tau,dtheta) -0.46..-0.56; see the joint_torque registration string). On hardware this term becomes real current via an INA219.",
         ParamValue{4.0}, ParamValue{0.0}, ParamValue{1000.0}},
        {"fall_alarm_tau", ParamMutability::HotMutable,
         "Decay time constant (ticks) of the leaky fall accumulator. Long enough that several falls must land inside it to trip the alarm, so an adventurous body that falls sporadically never does.",
         ParamValue{50000.0}, ParamValue{100.0}, ParamValue{10000000.0}},
        {"fall_alarm_on", ParamMutability::HotMutable,
         "Alarm level above which the viability guard demands a strict REDUCTION in falls instead of merely no regression. 0 = DISABLED (ships inert). The alarm can only ever TIGHTEN acceptance, never loosen it, so it cannot manufacture a false win.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1000.0}},
        {"w_dwell", ParamMutability::HotMutable,
         "weight on NEAR-INVERSION DWELL = mean(max(0, dwell_thresh - upright)) over the measured region. SHIPS AT 0: measured against the gate-2b logs it is WORSE than sd(upright) (window noise/mean 5.5-5.9 vs 1.93) because instability is bursty and autocorrelated — a rare-event signal in continuous clothing. Built and instrumented at full per-tick resolution so its weight can be decided on real data rather than a 60-tick-sampled proxy.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{100.0}},
        {"dwell_thresh", ParamMutability::HotMutable,
         "upright below this counts as leaning dangerously (0.9 ~ 26 deg off vertical). Fires on ~3.4% of ticks at 0.9, ~1.1% at 0.7 — higher threshold samples more often and is better conditioned.",
         ParamValue{0.9}, ParamValue{-1.0}, ParamValue{1.0}},
        {"settle_ticks", ParamMutability::ConstructionOnly,
         "Ticks of settling at the head of each window, excluded from measurement. 0 = the legacy back-half rule. Measured: |tilt| shows no settling trend and fwd_v settles by ~2000-2500 ticks, so back-half-of-12000 was discarding 4000 usable ticks per window.",
         ParamValue{int64_t(0)}, ParamValue{int64_t(0)}, ParamValue{int64_t(100000)}},
        {"accept_k", ParamMutability::HotMutable,
         "Accept only when J_cand < J_inc − accept_k·sigma_hat, where sigma_hat is the criterion's OWN measured per-window noise (estimated from revert pairs). 0 = legacy bare inequality, which on a stochastic criterion accepts ~50% by coin flip and drives the 1/5th anneal to the sigma ceiling (gate 2, 3/4 seeds). LOWERED 1.0 -> 0.25 on 2026-08-20: the step-0 authority check measured dJ/d(coupling) = -0.182/unit, so a single mutation at sigma 0.08 moves J by 0.044 against a margin of 0.244 — the margin was rejecting REAL gradients as noise. Margin and step must be matched: |dJ/dg|*sigma*range >= accept_k*sigma_hat, i.e. sigma >= 0.447*accept_k for the measured slope.",
         ParamValue{0.25}, ParamValue{0.0}, ParamValue{10.0}},
        {"noise_min_n", ParamMutability::HotMutable,
         "Revert-pair samples required before the acceptance margin is trusted; below it the margin is 0 so the search is not frozen at boot.",
         ParamValue{int64_t(3)}, ParamValue{int64_t(1)}, ParamValue{int64_t(100)}},
        {"noise_alpha", ParamMutability::HotMutable, "EMA rate for the squared revert-pair delta that estimates sigma_hat.",
         ParamValue{0.25}, ParamValue{0.001}, ParamValue{1.0}},
        // ---- guards ----------------------------------------------------------
        {"viability_falls_tol", ParamMutability::HotMutable,
         "G1: candidate falls may exceed incumbent falls by at most this (0 = strict no-regression).",
         ParamValue{int64_t(0)}, ParamValue{int64_t(0)}, ParamValue{int64_t(10)}},
        {"viability_load_tol", ParamMutability::HotMutable,
         "G2: candidate per-leg loaded-contact MINIMUM may fall below incumbent's by at most this (never a group mean — the stance-capture lesson).",
         ParamValue{0.05}, ParamValue{0.0}, ParamValue{1.0}},
        // ---- detectors -------------------------------------------------------
        {"upright_fall_thresh", ParamMutability::HotMutable,
         "upright below this = falling candidate; a fall counts after fall_debounce_ticks consecutive ticks below, once per excursion.",
         ParamValue{0.0}, ParamValue{-1.0}, ParamValue{1.0}},
        {"fall_debounce_ticks", ParamMutability::HotMutable,
         "Consecutive sub-threshold ticks before a fall counts. MUST stay under the body's 30-tick inversion dwell (auto-reset teleports upright back up).",
         ParamValue{int64_t(25)}, ParamValue{int64_t(1)}, ParamValue{int64_t(29)}},
        {"distress_thresh", ParamMutability::HotMutable, "distress duty threshold.",
         ParamValue{0.05}, ParamValue{0.0}, ParamValue{1.0}},
        {"load_thresh", ParamMutability::HotMutable,
         "A touchdown whose max foot_load stays below this through the horizon is UNLOADED (a ghost touch).",
         ParamValue{0.05}, ParamValue{0.0}, ParamValue{2.0}},
        {"touchdown_horizon_ticks", ParamMutability::HotMutable,
         "Post-touchdown ticks over which max foot_load decides loaded vs unloaded.",
         ParamValue{int64_t(12)}, ParamValue{int64_t(1)}, ParamValue{int64_t(200)}},
        {"min_touchdowns", ParamMutability::HotMutable,
         "A leg needs at least this many back-half touchdowns to be scored; below it the leg counts as FULLY UNLOADED (a non-stepping leg must never look clean).",
         ParamValue{int64_t(3)}, ParamValue{int64_t(1)}, ParamValue{int64_t(100)}},
        // ---- flow form (constants copied from MotorEPMv2's fwd-flow homeostat) --
        {"flow_alpha", ParamMutability::HotMutable, "flow/vol EMA alpha (~50-tick window at 0.02).",
         ParamValue{0.02}, ParamValue{0.0001}, ParamValue{1.0}},
        {"flow_vol_k", ParamMutability::HotMutable, "volatility penalty strength in flow quality.",
         ParamValue{4.0}, ParamValue{0.0}, ParamValue{100.0}},
        {"flow_vel_norm", ParamMutability::HotMutable,
         "fwd_v magnitude reference (m/s). Its MEANING depends on flow_min_form: at 0 it is a hard saturation ceiling (magnitude = clamp(v,0,norm)/norm); at 1 it is the HALF-quality speed (magnitude = v/(v+norm)), which has no ceiling to hide behind.",
         ParamValue{0.05}, ParamValue{0.001}, ParamValue{10.0}},
        {"gyro_topic", ParamMutability::ConstructionOnly,
         "Body-frame 3-axis gyro [roll rate, YAW RATE ABOUT OWN UP, pitch rate], each normalized by pi. Index 1 feeds the anti-circling factor. Egocentric and physically realisable (a MEMS gyro is bolted to the chassis), unlike imu[3] which is the WORLD vertical component and diverges from it exactly when the body tilts. '' = the factor has no input and is inert.",
         std::nullopt, std::nullopt, std::nullopt},
        {"flow_turn_k", ParamMutability::HotMutable,
         "ANTI-CIRCLING strength in flow quality. 0 = OFF (byte-identical), which is the default. Why it exists: fwd_v is BODY-FRAME forward speed, so a body carving a tight circle reads as flowing beautifully — the ledger's own recorded blind metric (high fwd_v + low net displacement = fast circling), and the flow term had no complement for it. The factor is 1/(1+k*|turn bias|) using the gyro's yaw rate about the body's own up axis, so it closes the hole with a LAWFUL signal rather than a god's-eye heading. 4.0 matches flow_vol_k's precedent.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{100.0}},
        {"flow_turn_alpha", ParamMutability::HotMutable,
         "EMA alpha for the turn bias — deliberately ~5x SLOWER than flow_alpha (~250 ticks / 5 s vs ~50). This timescale IS the design: the EMA is SIGNED, so a body correcting its heading left-then-right averages toward zero and is not penalized, while a body going round in circles holds a persistent bias and is. Heading regulation is one of the three behaviours this project reads as real capability, so penalizing turning per se would punish the thing we most want to see; only SUSTAINED one-way turn is circling.",
         ParamValue{0.004}, ParamValue{0.00001}, ParamValue{1.0}},
        {"flow_min_form", ParamMutability::HotMutable,
         "How the flow term's two factors combine. 0 = legacy PRODUCT magnitude*predictability (the default, byte-identical). 1 = MINIMUM of the two, with an unsaturating magnitude. Why 1 exists: flow was included as the counterweight that stops the search minimizing everything by standing still, and the 2026-08-23 landscape sweeps measured it failing to do that job. Predictability rises as a gait quietens, a product lets that pay for lost magnitude, and flow ended up voting WITH tilt/energy/unloaded for a slower body — mean |fwd_v| was LOWEST (0.0495) at the amplitude the criterion scored best. A minimum cannot be traded: quality is capped by whichever factor is worse, so predictability earned by not moving buys nothing. This is the same fix, for the same reason, as the per-leg loaded-contact guard using per-leg MINIMA instead of a mean (a mean let one leg die while the others paid for it).",
         ParamValue{int64_t(0)}, ParamValue{int64_t(0)}, ParamValue{int64_t(1)}},
        // ---- anneal ----------------------------------------------------------
        {"anneal_window", ParamMutability::HotMutable, "generations of accept history the 1/5th rule reads.",
         ParamValue{int64_t(10)}, ParamValue{int64_t(1)}, ParamValue{int64_t(1000)}},
        {"target_accept", ParamMutability::HotMutable,
         "Acceptance rate above which sigma grows. NEGATIVE = AUTO (the default), deriving it as the NOISE FLOOR Phi(-accept_k/sqrt(2)) = 0.5*erfc(accept_k/2) — the rate a search reaches while learning nothing. A fixed 0.2 sat BELOW that floor for every accept_k used (43% at k=0.25, 24% at k=1.0), so the 1/5th rule read chance-level acceptance as success and pinned sigma at its ceiling in gates 2 and 2e alike. Auto means sigma grows only when acceptance actually BEATS chance.",
         ParamValue{-1.0}, ParamValue{-1.0}, ParamValue{1.0}},
        {"anneal_up", ParamMutability::HotMutable, "σ multiplier when accepting too often.",
         ParamValue{1.5}, ParamValue{1.0}, ParamValue{10.0}},
        {"anneal_down", ParamMutability::HotMutable, "σ multiplier when accepting too rarely.",
         ParamValue{0.85}, ParamValue{0.01}, ParamValue{1.0}},
        {"sigma_min", ParamMutability::HotMutable,
         "Anneal FLOOR — the search never fully stops ('settle AND remain adaptable'), and must never anneal below the resolution its own criterion can measure. RAISED 0.01 -> 0.08 on 2026-08-20: at sigma 0.015 a coupling step moved J by 0.008 against a 0.244 margin (30x short), so shrinking sigma made acceptance IMPOSSIBLE and fewer accepts shrank sigma further — a self-reinforcing freeze that is indistinguishable from convergence in the logs. The floor now sits at the starting sigma so that collapse cannot happen.",
         ParamValue{0.08}, ParamValue{0.0001}, ParamValue{1.0}},
        {"sigma_max", ParamMutability::HotMutable, "anneal ceiling.",
         ParamValue{0.5}, ParamValue{0.001}, ParamValue{2.0}},
        // ---- operator handoff ------------------------------------------------
        {"incumbent_override", ParamMutability::HotMutable,
         "ADOPT: replace the incumbent with this vector (clamped to bounds; length must match gain_keys). The panel writes the operator's hand-tuned point here before σ-resume so the evolver searches from THEIR point instead of stomping it.",
         std::nullopt, std::nullopt, std::nullopt},
    };
}

ParamMap GainEvolver::current_params() const {
    ParamMap m;
    m["gain_keys"]        = gain_keys_;
    m["gain_seed"]        = gain_seed_;
    m["gain_min"]         = gain_min_;
    m["gain_max"]         = gain_max_;
    m["gain_sigma_scale"] = gain_sigma_scale_;
    m["gain_topic"]       = gain_topic_;
    m["upright_topic"]      = upright_topic_;
    m["distress_topic"]     = distress_topic_;
    m["foot_load_topic"]    = foot_load_topic_;
    m["foot_contact_topic"] = foot_contact_topic_;
    m["imu_topic"]          = imu_topic_;
    m["travel_topic"]       = travel_topic_;
    m["gyro_topic"]         = gyro_topic_;
    m["n_legs"]            = int64_t(n_legs_);
    m["seed"]              = seed_;
    m["warmup_ticks"]      = warmup_ticks_;
    m["eval_window_ticks"] = eval_window_ticks_;
    m["republish_every"]   = republish_every_;
    m["mutation_sigma"]    = sigma_;          // LIVE annealed σ (panel sync shows it)
    m["w_falls"]    = w_falls_;
    m["w_tilt_sd"]  = w_tilt_sd_;
    m["w_distress"] = w_distress_;
    m["w_unloaded"] = w_unloaded_;
    m["w_flow"]     = w_flow_;
    m["w_dwell"]    = w_dwell_;
    m["w_energy"]   = w_energy_;
    m["torque_topic"] = torque_topic_;
    m["fall_alarm_tau"] = fall_alarm_tau_;
    m["fall_alarm_on"] = fall_alarm_on_;
    m["dwell_thresh"] = dwell_thresh_;
    m["settle_ticks"] = settle_ticks_;
    m["accept_k"]    = accept_k_;
    m["noise_min_n"] = noise_min_n_;
    m["noise_alpha"] = noise_alpha_;
    m["viability_falls_tol"] = viability_falls_tol_;
    m["viability_load_tol"]  = viability_load_tol_;
    m["upright_fall_thresh"] = upright_fall_thresh_;
    m["fall_debounce_ticks"] = fall_debounce_ticks_;
    m["distress_thresh"]     = distress_thresh_;
    m["load_thresh"]         = load_thresh_;
    m["touchdown_horizon_ticks"] = touchdown_horizon_ticks_;
    m["min_touchdowns"]      = min_touchdowns_;
    m["flow_alpha"]    = flow_alpha_;
    m["flow_vol_k"]    = flow_vol_k_;
    m["flow_vel_norm"] = flow_vel_norm_;
    m["flow_min_form"] = flow_min_form_;
    m["flow_turn_k"] = flow_turn_k_;
    m["flow_turn_alpha"] = flow_turn_alpha_;
    m["gyro_topic"] = gyro_topic_;
    m["anneal_window"] = anneal_window_;
    m["target_accept"] = target_accept_;
    m["target_eff"]    = (target_accept_ >= 0.0) ? target_accept_ : noise_floor_rate();
    m["anneal_up"]     = anneal_up_;
    m["anneal_down"]   = anneal_down_;
    m["sigma_min"]     = sigma_min_;
    m["sigma_max"]     = sigma_max_;
    m["incumbent_override"] = incumbent_;     // reads back as the live incumbent
    return m;
}

void GainEvolver::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("GainEvolver requires a non-null Bus");

    auto get_dvec = [](ParamValue const& v, char const* key) {
        if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
        throw std::invalid_argument(std::string("GainEvolver: ") + key + " must be a number array");
    };
    apply_param(params, "gain_keys", [&](auto const& v){
        if (auto p = std::get_if<std::vector<std::string>>(&v)) gain_keys_ = *p;
        else throw std::invalid_argument("GainEvolver: gain_keys must be a string array"); });
    apply_param(params, "gain_seed",        [&](auto const& v){ gain_seed_        = get_dvec(v, "gain_seed"); });
    apply_param(params, "gain_min",         [&](auto const& v){ gain_min_         = get_dvec(v, "gain_min"); });
    apply_param(params, "gain_max",         [&](auto const& v){ gain_max_         = get_dvec(v, "gain_max"); });
    apply_param(params, "gain_sigma_scale", [&](auto const& v){ gain_sigma_scale_ = get_dvec(v, "gain_sigma_scale"); });
    for (auto const& [key, member] : std::initializer_list<std::pair<char const*, std::string*>>{
             {"gain_topic", &gain_topic_}, {"upright_topic", &upright_topic_},
             {"distress_topic", &distress_topic_}, {"foot_load_topic", &foot_load_topic_},
             {"foot_contact_topic", &foot_contact_topic_}, {"imu_topic", &imu_topic_},
             {"travel_topic", &travel_topic_}, {"gyro_topic", &gyro_topic_},
             {"torque_topic", &torque_topic_}})
        apply_param(params, key, [&](auto const& v){
            if (auto p = std::get_if<std::string>(&v)) *member = *p; });
    apply_param(params, "n_legs",            [&](auto const& v){ n_legs_            = int(get_int(v, "n_legs")); });
    apply_param(params, "seed",              [&](auto const& v){ seed_              = get_int(v, "seed"); });
    apply_param(params, "warmup_ticks",      [&](auto const& v){ warmup_ticks_      = get_int(v, "warmup_ticks"); });
    apply_param(params, "eval_window_ticks", [&](auto const& v){ eval_window_ticks_ = get_int(v, "eval_window_ticks"); });
    apply_param(params, "republish_every",   [&](auto const& v){ republish_every_   = get_int(v, "republish_every"); });
    apply_param(params, "settle_ticks",      [&](auto const& v){ settle_ticks_      = get_int(v, "settle_ticks"); });
    for (auto const& [key, member] : std::initializer_list<std::pair<char const*, double*>>{
             {"w_falls", &w_falls_}, {"w_tilt_sd", &w_tilt_sd_}, {"w_distress", &w_distress_},
             {"w_unloaded", &w_unloaded_}, {"w_flow", &w_flow_},
             {"w_dwell", &w_dwell_}, {"dwell_thresh", &dwell_thresh_},
             {"w_energy", &w_energy_}, {"fall_alarm_tau", &fall_alarm_tau_},
             {"fall_alarm_on", &fall_alarm_on_},
             {"accept_k", &accept_k_}, {"noise_alpha", &noise_alpha_},
             {"viability_load_tol", &viability_load_tol_},
             {"upright_fall_thresh", &upright_fall_thresh_},
             {"distress_thresh", &distress_thresh_}, {"load_thresh", &load_thresh_},
             {"flow_alpha", &flow_alpha_}, {"flow_vol_k", &flow_vol_k_},
             {"flow_vel_norm", &flow_vel_norm_}, {"flow_turn_k", &flow_turn_k_},
             {"flow_turn_alpha", &flow_turn_alpha_}, {"target_accept", &target_accept_},
             {"anneal_up", &anneal_up_}, {"anneal_down", &anneal_down_},
             {"sigma_min", &sigma_min_}, {"sigma_max", &sigma_max_}})
        apply_param(params, key, [&](auto const& v){ *member = get_double(v, key); });
    for (auto const& [key, member] : std::initializer_list<std::pair<char const*, int64_t*>>{
             {"viability_falls_tol", &viability_falls_tol_},
             {"fall_debounce_ticks", &fall_debounce_ticks_},
             {"touchdown_horizon_ticks", &touchdown_horizon_ticks_},
             {"min_touchdowns", &min_touchdowns_}, {"anneal_window", &anneal_window_},
             // noise_min_n was MISSING here until 2026-08-24 — a config's value was
             // silently ignored at construction (only a hot-mutate applied it) and
             // the default 3 always won.  Every shipped config happened to set 3,
             // so no run was affected; the de-vacuized margin test caught it.
             {"noise_min_n", &noise_min_n_},
             {"flow_min_form", &flow_min_form_}})
        apply_param(params, key, [&](auto const& v){ *member = get_int(v, key); });
    apply_param(params, "mutation_sigma", [&](auto const& v){ sigma_ = get_double(v, "mutation_sigma"); });

    // ---- declared-vector validation (a malformed declaration must refuse to
    // construct, never search a garbage space) --------------------------------
    size_t n = gain_keys_.size();
    if (gain_sigma_scale_.empty()) gain_sigma_scale_.assign(n, 1.0);
    if (gain_seed_.size() != n || gain_min_.size() != n || gain_max_.size() != n ||
        gain_sigma_scale_.size() != n)
        throw std::invalid_argument(
            "GainEvolver: gain_keys/gain_seed/gain_min/gain_max/gain_sigma_scale lengths must match");
    for (size_t k = 0; k < n; ++k) {
        if (gain_min_[k] > gain_max_[k])
            throw std::invalid_argument("GainEvolver: gain_min > gain_max at '" + gain_keys_[k] + "'");
        if (gain_seed_[k] < gain_min_[k] || gain_seed_[k] > gain_max_[k])
            throw std::invalid_argument("GainEvolver: gain_seed out of bounds at '" + gain_keys_[k] + "'");
    }
    if (sigma_ < 0.0) throw std::invalid_argument("GainEvolver: mutation_sigma must be >= 0");
    // w_tilt_var became w_tilt_sd when the term changed from variance to sd
    // (gate 2).  An unknown key in a ParamMap is silently ignored, so a stale
    // config would quietly get the DEFAULT weight on a DIFFERENT quantity —
    // exactly the silent-drift trap this repo keeps re-learning.  Fail loudly.
    if (params.find("w_tilt_var") != params.end())
        throw std::invalid_argument(
            "GainEvolver: 'w_tilt_var' was replaced by 'w_tilt_sd' (the term is now "
            "sd(upright), not variance) — update the config rather than inheriting a default");

    incumbent_ = gain_seed_;
    candidate_ = gain_seed_;
    rng_.seed(uint64_t(seed_));
    foot_load_.assign(size_t(n_legs_), 0.0f);
    foot_contact_.assign(size_t(n_legs_), 0.0f);
    contact_prev_.assign(size_t(n_legs_), 0.0f);
    td_horizon_.assign(size_t(n_legs_), 0);
    td_maxload_.assign(size_t(n_legs_), 0.0f);
    cur_.reset(n_legs_);
    inc_stats_.reset(n_legs_);

    sub_ids_.clear();
    struct Sub { std::string const* topic; void (GainEvolver::*fn)(MessagePtr); };
    for (auto const& s : {Sub{&upright_topic_,      &GainEvolver::handle_upright},
                          Sub{&distress_topic_,     &GainEvolver::handle_distress},
                          Sub{&foot_load_topic_,    &GainEvolver::handle_foot_load},
                          Sub{&foot_contact_topic_, &GainEvolver::handle_foot_contact},
                          Sub{&imu_topic_,          &GainEvolver::handle_imu},
                          Sub{&travel_topic_,       &GainEvolver::handle_travel},
                          Sub{&gyro_topic_,         &GainEvolver::handle_gyro},
                          Sub{&torque_topic_,       &GainEvolver::handle_torque}})
        if (!s.topic->empty())
            sub_ids_.push_back(bus_->subscribe(*s.topic, SubscriptionKind::Direct,
                [this, fn = s.fn](std::string_view, MessagePtr p){ (this->*fn)(p); }));
}

void GainEvolver::on_param_change(std::string_view key, ParamValue const& value) {
    if (key == "mutation_sigma") {
        double v = get_double(value, "mutation_sigma");
        if (v < 0.0) throw std::invalid_argument("GainEvolver: mutation_sigma must be >= 0");
        bool was_off = (sigma_ == 0.0);
        sigma_ = v;
        accept_hist_.clear();               // operator override resets the anneal
        if (v > 0.0 && phase_ != Phase::Warmup) {
            // (Re)start clean: a fresh incumbent window, publish next tick so the
            // consumer is guaranteed on OUR incumbent (not a stale hand-tune or an
            // unevaluated candidate).  Publishing from here would land mid-patch,
            // outside a tick — the flag defers it to tick().
            phase_ = Phase::Incumbent;
            win_tick_ = 0;
            cur_.reset(n_legs_);
            need_publish_ = true;
        } else if (v == 0.0 && !was_off && phase_ == Phase::Candidate) {
            // Pausing mid-candidate: the body must not be LEFT on an unevaluated
            // candidate — one closing incumbent publish, then silent observer.
            phase_ = Phase::Incumbent;
            win_tick_ = 0;
            cur_.reset(n_legs_);
            need_publish_ = true;
        }
        return;
    }
    if (key == "incumbent_override") {
        auto p = std::get_if<std::vector<double>>(&value);
        if (!p || p->size() != gain_keys_.size())
            throw std::invalid_argument("GainEvolver: incumbent_override length must equal gain_keys");
        for (size_t k = 0; k < p->size(); ++k)
            incumbent_[k] = std::clamp((*p)[k], gain_min_[k], gain_max_[k]);
        ++overrides_;
        if (phase_ != Phase::Warmup) {      // adopt = restart from the new point
            phase_ = Phase::Incumbent;
            win_tick_ = 0;
            cur_.reset(n_legs_);
            if (sigma_ > 0.0) need_publish_ = true;
        }
        return;
    }
    struct DKey { char const* k; double* m; };
    for (auto const& d : {DKey{"w_falls", &w_falls_}, DKey{"w_tilt_sd", &w_tilt_sd_},
                          DKey{"w_distress", &w_distress_}, DKey{"w_unloaded", &w_unloaded_},
                          DKey{"w_flow", &w_flow_}, DKey{"w_dwell", &w_dwell_},
                          DKey{"dwell_thresh", &dwell_thresh_}, DKey{"w_energy", &w_energy_},
                          DKey{"fall_alarm_tau", &fall_alarm_tau_},
                          DKey{"fall_alarm_on", &fall_alarm_on_}, DKey{"accept_k", &accept_k_},
                          DKey{"noise_alpha", &noise_alpha_},
                          DKey{"viability_load_tol", &viability_load_tol_},
                          DKey{"upright_fall_thresh", &upright_fall_thresh_},
                          DKey{"distress_thresh", &distress_thresh_}, DKey{"load_thresh", &load_thresh_},
                          DKey{"flow_alpha", &flow_alpha_}, DKey{"flow_vol_k", &flow_vol_k_},
                          DKey{"flow_vel_norm", &flow_vel_norm_},
                          DKey{"flow_turn_k", &flow_turn_k_},
                          DKey{"flow_turn_alpha", &flow_turn_alpha_},
                          DKey{"target_accept", &target_accept_},
                          DKey{"anneal_up", &anneal_up_}, DKey{"anneal_down", &anneal_down_},
                          DKey{"sigma_min", &sigma_min_}, DKey{"sigma_max", &sigma_max_}})
        if (key == d.k) { *d.m = get_double(value, d.k); return; }
    struct IKey { char const* k; int64_t* m; };
    for (auto const& d : {IKey{"viability_falls_tol", &viability_falls_tol_},
                          IKey{"fall_debounce_ticks", &fall_debounce_ticks_},
                          IKey{"touchdown_horizon_ticks", &touchdown_horizon_ticks_},
                          IKey{"min_touchdowns", &min_touchdowns_}, IKey{"noise_min_n", &noise_min_n_},
                          IKey{"anneal_window", &anneal_window_},
                          IKey{"flow_min_form", &flow_min_form_}})
        if (key == d.k) { *d.m = get_int(value, d.k); return; }
    throw std::invalid_argument("GainEvolver: param '" + std::string(key) + "' is not HotMutable");
}

// ---- handlers ---------------------------------------------------------------

void GainEvolver::handle_upright(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    upright_ = pt->values[0];
}

void GainEvolver::handle_distress(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    distress_ = pt->values[0];
}

void GainEvolver::handle_foot_load(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int k = std::min<int>(int(pt->values.size()), n_legs_);
    for (int i = 0; i < k; ++i) foot_load_[size_t(i)] = pt->values[i];
}

void GainEvolver::handle_foot_contact(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int k = std::min<int>(int(pt->values.size()), n_legs_);
    for (int i = 0; i < k; ++i) foot_contact_[size_t(i)] = pt->values[i];
}

void GainEvolver::handle_torque(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    // Mean |torque| across the servos: what the battery pays.  A servo draws
    // current to HOLD a pose as well as to move one, so magnitude (not mechanical
    // power) is the honest electrical proxy — and it is what hardware reports.
    double acc = 0.0;
    for (int i = 0; i < pt->values.size(); ++i) acc += std::fabs(double(pt->values[i]));
    torque_mag_ = acc / double(pt->values.size());
}

void GainEvolver::handle_gyro(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 2) return;
    yaw_rate_ = pt->values[1];       // [roll, YAW ABOUT OWN UP, pitch], /pi normalized
}

void GainEvolver::handle_imu(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 3) return;
    if (travel_topic_.empty())
        fwd_v_ = pt->values[2];             // [sin yaw, cos yaw, fwd_v, ang_v]
}

void GainEvolver::handle_travel(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    // stride_v is [v_right, v_forward] (vel_ego-shaped); a scalar topic also works.
    fwd_v_ = pt->values.size() >= 2 ? pt->values[1] : pt->values[0];
    ++travel_rx_n_;                         // consumer half of the two-sided meter
}

// ---- scoring ----------------------------------------------------------------

double GainEvolver::per_leg_loaded_min(WindowStats const& w) const {
    double lo = 1.0;
    for (int l = 0; l < n_legs_; ++l) {
        double loaded = 0.0;                // a non-stepping leg scores ZERO loaded
        if (w.td[size_t(l)] >= int(min_touchdowns_))
            loaded = 1.0 - double(w.unloaded[size_t(l)]) / double(w.td[size_t(l)]);
        lo = std::min(lo, loaded);
    }
    return lo;
}

GainEvolver::Terms GainEvolver::score(WindowStats const& w) const {
    Terms t;
    t.falls = double(w.falls);
    if (w.meas_n > 0) {
        double mean = w.up_sum / double(w.meas_n);
        // sd, NOT variance: squaring let a single tumble dominate quadratically
        // and made the term a rare-event count in disguise (gate 2).
        t.tilt_sd       = std::sqrt(std::max(0.0, w.up_sq / double(w.meas_n) - mean * mean));
        t.distress_duty = double(w.distress_hits) / double(w.meas_n);
        t.dwell         = w.dwell_sum / double(w.meas_n);
        t.energy        = w.torque_sum / double(w.meas_n);
        t.flow_term     = 1.0 - w.flow_q_sum / double(w.meas_n);
        t.flow_mag      = w.flow_mag_sum  / double(w.meas_n);
        t.flow_pred     = w.flow_pred_sum / double(w.meas_n);
        t.flow_turn     = w.flow_turn_sum / double(w.meas_n);
    } else {
        t.tilt_sd = 0.0; t.distress_duty = 0.0; t.flow_term = 1.0; t.dwell = 0.0; t.energy = 0.0;
        t.flow_mag = 0.0; t.flow_pred = 0.0; t.flow_turn = 0.0;
    }
    double unl_sum = 0.0;
    for (int l = 0; l < n_legs_; ++l) {
        double unl = 1.0;                   // a non-stepping leg is FULLY unloaded
        if (w.td[size_t(l)] >= int(min_touchdowns_))
            unl = double(w.unloaded[size_t(l)]) / double(w.td[size_t(l)]);
        unl_sum += unl;
    }
    t.unloaded_mean = n_legs_ > 0 ? unl_sum / double(n_legs_) : 0.0;
    t.loaded_min    = per_leg_loaded_min(w);
    t.J = w_falls_ * t.falls + w_tilt_sd_ * t.tilt_sd + w_distress_ * t.distress_duty
        + w_unloaded_ * t.unloaded_mean + w_flow_ * t.flow_term + w_dwell_ * t.dwell
        + w_energy_ * t.energy;
    t.valid = true;
    return t;
}

// Pure per-window noise, estimated from the system's own dynamics: two
// consecutive incumbent windows separated by a REVERT scored the SAME vector,
// so their difference is measurement noise alone.  var(difference) = 2*var(window).
double GainEvolver::sigma_est() const {
    if (noise_n_ < noise_min_n_ || noise_ema_ <= 0.0) return 0.0;
    return std::sqrt(noise_ema_ * 0.5);
}

bool GainEvolver::viability_ok(WindowStats const& cand, WindowStats const& inc) const {
    // G1, with the operator's alarm: normally "no regression", but once the leaky
    // fall accumulator says the body is falling PERSISTENTLY, a candidate must
    // strictly REDUCE falls to be accepted at all.  tol can only shrink here —
    // the alarm never widens the gate, so it cannot invent an acceptance.
    int64_t tol = viability_falls_tol_;
    if (fall_alarm_on_ > 0.0 && fall_alarm_ > fall_alarm_on_) tol = -1;
    if (cand.falls > inc.falls + int(tol)) return false;                           // G1
    if (per_leg_loaded_min(cand) < per_leg_loaded_min(inc) - viability_load_tol_)  // G2
        return false;
    return true;
}

// ---- the loop ---------------------------------------------------------------

void GainEvolver::publish_vector(bool candidate) {
    if (gain_topic_.empty() || gain_keys_.empty()) return;
    auto out = std::make_shared<GainVector>();
    out->producer_id  = id_.empty() ? std::string("gain_evolver") : id_;
    out->keys         = gain_keys_;
    out->values       = candidate ? candidate_ : incumbent_;
    out->generation   = generation_;
    out->is_candidate = candidate;
    bus_->publish(gain_topic_, out);
    ++publishes_;
}

void GainEvolver::mutate_candidate() {
    // Fresh unit-normal per generation, scaled after: σ never parameterizes the
    // distribution, so the σ=0 process-abort (the 2026-08-09 coord-search burn)
    // is unreachable by construction.  This path only runs at sigma_ > 0.
    std::normal_distribution<double> nz(0.0, 1.0);
    for (size_t k = 0; k < gain_keys_.size(); ++k)
        candidate_[k] = std::clamp(
            incumbent_[k] + sigma_ * gain_sigma_scale_[k] * (gain_max_[k] - gain_min_[k]) * nz(rng_),
            gain_min_[k], gain_max_[k]);
}

// The acceptance rate a search achieves while learning NOTHING.  If a candidate
// differs from the incumbent by noise alone, the difference of two independent
// window scores has sd sqrt(2)*sigma_hat, so the chance of clearing a margin of
// accept_k*sigma_hat is Phi(-accept_k/sqrt(2)) = 0.5*erfc(accept_k/2).
//
// This is the number the 1/5th rule must be compared against.  A FIXED target
// (0.2) sat BELOW this floor for every accept_k the campaign used — 43% at
// k=0.25, 24% at k=1.0 — so the rule read a chance-level accept rate as success
// and inflated sigma to its ceiling.  Deriving the target instead means sigma
// grows only when acceptance genuinely BEATS chance, which is the property the
// rule was always meant to have (doctrine §5: adapt the constant from the
// system's own dynamics, never hand-tune it to a scale).
double GainEvolver::noise_floor_rate() const {
    return 0.5 * std::erfc(accept_k_ * 0.5);
}

void GainEvolver::anneal(bool accepted) {
    accept_hist_.push_back(uint8_t(accepted));
    while (int64_t(accept_hist_.size()) > anneal_window_) accept_hist_.pop_front();
    if (int64_t(accept_hist_.size()) < anneal_window_) return;
    double rate = 0.0;
    for (uint8_t a : accept_hist_) rate += a;
    rate /= double(accept_hist_.size());
    double target = (target_accept_ >= 0.0) ? target_accept_ : noise_floor_rate();
    sigma_ = std::clamp(sigma_ * (rate > target ? anneal_up_ : anneal_down_),
                        sigma_min_, sigma_max_);
}

void GainEvolver::start_window(Phase p) {
    phase_ = p;
    win_tick_ = 0;
    cur_.reset(n_legs_);
    if (sigma_ > 0.0) publish_vector(p == Phase::Candidate);
}

void GainEvolver::tick(uint64_t /*tick_id*/) {
    if (gain_keys_.empty()) return;

    if (need_publish_) {                    // deferred from on_param_change
        need_publish_ = false;
        if (sigma_ > 0.0) publish_vector(false);
    }

    // ---- every-tick sensing (EMAs + edges run through warmup too, so the
    // first window starts with settled state) ---------------------------------
    float fa = float(flow_alpha_);
    flow_ema_     += fa * (fwd_v_ - flow_ema_);
    flow_vol_ema_ += fa * (std::fabs(fwd_v_ - flow_ema_) - flow_vol_ema_);
    // TURN BIAS.  Signed and slow on purpose (see flow_turn_alpha): left-then-right
    // heading correction averages toward zero, a circle holds a bias.
    turn_ema_ += float(flow_turn_alpha_) * (yaw_rate_ - turn_ema_);

    // Leaky fall alarm: decays every tick regardless of phase, so it measures a
    // SUSTAINED rate rather than one window's luck.  Deliberately outside the
    // measured region — it is not a criterion term, it is a viability mode.
    if (fall_alarm_tau_ > 0.0) fall_alarm_ *= (1.0 - 1.0 / fall_alarm_tau_);
    if (!fall_latched_) {
        if (upright_ < float(upright_fall_thresh_)) {
            if (++fall_below_run_ >= fall_debounce_ticks_) {
                ++cur_.falls;               // one count per excursion
                fall_alarm_ += 1.0;
                fall_latched_ = true;
            }
        } else fall_below_run_ = 0;
    } else if (upright_ > kFallRecover) {
        fall_latched_ = false;
        fall_below_run_ = 0;
    }

    int64_t settle = settle_ticks_ > 0 ? settle_ticks_ : eval_window_ticks_ / 2;
    if (settle >= eval_window_ticks_) settle = eval_window_ticks_ / 2;   // never measure nothing
    bool back_half = (phase_ != Phase::Warmup) && (win_tick_ >= settle);
    for (int l = 0; l < n_legs_; ++l) {
        size_t i = size_t(l);
        bool was = contact_prev_[i] > 0.5f, now = foot_contact_[i] > 0.5f;
        contact_prev_[i] = foot_contact_[i];
        if (!was && now && back_half && td_horizon_[i] == 0) {
            td_horizon_[i] = touchdown_horizon_ticks_;   // arm the post-touchdown watch
            td_maxload_[i] = foot_load_[i];
        }
        if (td_horizon_[i] > 0) {
            td_maxload_[i] = std::max(td_maxload_[i], foot_load_[i]);
            if (--td_horizon_[i] == 0) {                 // verdict
                ++cur_.td[i];
                if (td_maxload_[i] < float(load_thresh_)) ++cur_.unloaded[i];
            }
        }
    }

    if (phase_ == Phase::Warmup) {
        if (++win_tick_ >= warmup_ticks_) start_window(Phase::Incumbent);
        return;
    }

    if (back_half) {
        cur_.up_sum += upright_;
        cur_.up_sq  += double(upright_) * double(upright_);
        ++cur_.meas_n;
        if (distress_ > float(distress_thresh_)) ++cur_.distress_hits;
        if (double(upright_) < dwell_thresh_)
            cur_.dwell_sum += dwell_thresh_ - double(upright_);
        cur_.torque_sum += torque_mag_;
        // FLOW QUALITY = how much steady travel the body is actually producing.
        // Two factors: MAGNITUDE (is it going anywhere) and PREDICTABILITY (is the
        // going steady).  How they combine decides whether this term can do the job
        // it was included for — being the counterweight that stops the search
        // minimizing every other term by standing still.
        double pred = 1.0 / (1.0 + flow_vol_k_ * double(flow_vol_ema_));
        // ANTI-CIRCLING.  fwd_v is body-frame forward speed, so a tight circle reads
        // as flowing beautifully -- travelling and going nowhere are indistinguishable
        // to the magnitude factor.  The complement is a LAWFUL one: the gyro's yaw rate
        // about the body's own up axis, EMA'd slowly and signed, so only a sustained
        // one-way bias counts.  k = 0 gives exactly 1.0 and changes nothing.
        double turn     = 1.0 / (1.0 + flow_turn_k_ * std::fabs(double(turn_ema_)));
        double mag, fq;
        if (flow_min_form_ > 0) {
            // Form 1.  MAGNITUDE CANNOT BE TRADED AWAY.  A quiet body is a very
            // predictable one, so under the product form predictability rose as the
            // gait quietened and paid for the travel that was lost — measured on the
            // 2026-08-23 landscape sweeps, where flow scored BEST at the amplitude
            // whose mean |fwd_v| was LOWEST (0.0495), voting with tilt/energy/
            // unloaded for a slower body instead of opposing them.  A minimum caps
            // quality by whichever factor is worse, so stillness-bought
            // predictability buys nothing.  Same fix and same reason as the per-leg
            // loaded-contact guard taking per-leg MINIMA rather than a mean, after a
            // mean let one leg die while the others paid for it.
            // Magnitude is v/(v+ref) rather than a clamped ratio: monotone in speed
            // with no ceiling to sit against, so more travel always scores better
            // and the factor never drops out of the comparison.
            double v = std::max(0.0, double(flow_ema_));
            mag = v / (v + flow_vel_norm_);
            fq  = std::min(std::min(mag, pred), turn);
        } else {
            mag = std::clamp(double(flow_ema_), 0.0, flow_vel_norm_) / flow_vel_norm_;
            fq  = mag * pred * turn;
        }
        cur_.flow_q_sum    += fq;
        cur_.flow_mag_sum  += mag;    // kept apart so a window can report WHICH
        cur_.flow_pred_sum += pred;   // factor limited it, not just their result
        cur_.flow_turn_sum += turn;
    }

    if (sigma_ > 0.0 && republish_every_ > 0 && win_tick_ > 0 &&
        win_tick_ % republish_every_ == 0)
        publish_vector(phase_ == Phase::Candidate);

    if (++win_tick_ < eval_window_ticks_) return;

    // ---- window boundary ----------------------------------------------------
    if (sigma_ == 0.0) {
        // SILENT OBSERVER: score for the instruments, publish nothing, draw nothing.
        inc_stats_ = cur_;
        inc_terms_ = score(cur_);
        start_window(Phase::Incumbent);
        return;
    }
    if (phase_ == Phase::Incumbent) {
        inc_stats_ = cur_;
        inc_terms_ = score(cur_);
        // Noise sample: only valid when the PREVIOUS generation reverted, i.e.
        // this window and the last one scored the same vector.
        if (prev_inc_pair_ && prev_inc_J_ >= 0.0) {
            double d = inc_terms_.J - prev_inc_J_;
            noise_ema_ = (noise_n_ == 0) ? d * d
                                         : noise_ema_ + noise_alpha_ * (d * d - noise_ema_);
            ++noise_n_;
        }
        prev_inc_J_ = inc_terms_.J;
        prev_inc_pair_ = false;
        mutate_candidate();
        start_window(Phase::Candidate);
        return;
    }
    // Candidate window complete — the contemporaneous compare.
    cand_terms_ = score(cur_);
    // The improvement must clear the criterion's own measured noise, or a coin
    // flip becomes an "accept" and the 1/5th anneal reads that as success.
    accept_margin_ = accept_k_ * sigma_est();
    bool ok = viability_ok(cur_, inc_stats_)
              && cand_terms_.J < inc_terms_.J - accept_margin_;
    if (ok) { incumbent_ = candidate_; ++accepts_; }
    else    { ++reverts_; }
    prev_inc_pair_ = !ok;          // a revert leaves the vector unchanged
    accept_log_.push_back(ok ? 'A' : 'R');
    if (accept_log_.size() > kAcceptLogMax) accept_log_.erase(0, 1);
    ++generation_;
    anneal(ok);
    start_window(Phase::Incumbent);         // revert = the old incumbent re-lands
}

// ---- persistence / diagnostics ----------------------------------------------

nlohmann::json GainEvolver::snapshot_state() const {
    nlohmann::json mod;
    { std::ostringstream os; os << rng_; mod["rng"] = os.str(); }
    mod["incumbent"]  = incumbent_;
    mod["candidate"]  = candidate_;
    mod["phase"]      = int(phase_);
    mod["win_tick"]   = win_tick_;
    mod["generation"] = generation_;
    mod["accepts"]    = accepts_;
    mod["reverts"]    = reverts_;
    mod["publishes"]  = publishes_;
    mod["overrides"]  = overrides_;
    mod["sigma"]      = sigma_;
    mod["accept_hist"] = std::vector<int>(accept_hist_.begin(), accept_hist_.end());
    mod["accept_log"]  = accept_log_;
    mod["need_publish"] = need_publish_;
    mod["noise_ema"] = noise_ema_;
    mod["noise_n"] = noise_n_;
    mod["prev_inc_J"] = prev_inc_J_;
    mod["prev_inc_pair"] = prev_inc_pair_;
    mod["accept_margin"] = accept_margin_;
    mod["fall_alarm"] = fall_alarm_;
    mod["torque_mag"] = torque_mag_;
    auto stats = [](WindowStats const& w) {
        return nlohmann::json{{"falls", w.falls}, {"up_sum", w.up_sum}, {"up_sq", w.up_sq},
                              {"meas_n", w.meas_n}, {"distress_hits", w.distress_hits},
                              {"flow_q_sum", w.flow_q_sum}, {"flow_mag_sum", w.flow_mag_sum},
                              {"flow_pred_sum", w.flow_pred_sum},
                              {"flow_turn_sum", w.flow_turn_sum},
                              {"dwell_sum", w.dwell_sum}, {"torque_sum", w.torque_sum}, {"td", w.td}, {"unloaded", w.unloaded}};
    };
    mod["cur"]       = stats(cur_);
    mod["inc_stats"] = stats(inc_stats_);
    auto terms = [](Terms const& t) {
        return nlohmann::json{{"falls", t.falls}, {"tilt_sd", t.tilt_sd}, {"dwell", t.dwell}, {"energy", t.energy},
                              {"distress_duty", t.distress_duty}, {"unloaded_mean", t.unloaded_mean},
                              {"flow_term", t.flow_term}, {"flow_mag", t.flow_mag},
                              {"flow_pred", t.flow_pred}, {"flow_turn", t.flow_turn},
                              {"loaded_min", t.loaded_min},
                              {"J", t.J}, {"valid", t.valid}};
    };
    mod["inc_terms"]  = terms(inc_terms_);
    mod["cand_terms"] = terms(cand_terms_);
    mod["upright"]  = upright_;
    mod["distress"] = distress_;
    mod["fwd_v"]    = fwd_v_;
    mod["foot_load"]    = foot_load_;
    mod["foot_contact"] = foot_contact_;
    mod["fall_below_run"] = fall_below_run_;
    mod["fall_latched"]   = fall_latched_;
    mod["contact_prev"] = contact_prev_;
    mod["td_horizon"]   = td_horizon_;
    mod["td_maxload"]   = td_maxload_;
    mod["flow_ema"]     = flow_ema_;
    mod["flow_vol_ema"] = flow_vol_ema_;
    mod["turn_ema"]     = turn_ema_;
    return nlohmann::json{{"version", 1}, {"module", mod}};
}

void GainEvolver::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || !s.contains("module")) return;
    int version = s.value("version", 0);
    if (version != 1)
        throw std::runtime_error("GainEvolver::restore_state: unknown version " + std::to_string(version));
    auto const& mod = s.at("module");
    if (mod.contains("rng")) { std::istringstream is(mod["rng"].get<std::string>()); is >> rng_; }
    if (mod.contains("incumbent")) incumbent_ = mod["incumbent"].get<std::vector<double>>();
    if (mod.contains("candidate")) candidate_ = mod["candidate"].get<std::vector<double>>();
    phase_      = Phase(mod.value("phase", int(phase_)));
    win_tick_   = mod.value("win_tick", win_tick_);
    generation_ = mod.value("generation", generation_);
    accepts_    = mod.value("accepts", accepts_);
    reverts_    = mod.value("reverts", reverts_);
    publishes_  = mod.value("publishes", publishes_);
    overrides_  = mod.value("overrides", overrides_);
    sigma_      = mod.value("sigma", sigma_);
    if (mod.contains("accept_hist")) {
        accept_hist_.clear();
        for (int a : mod["accept_hist"].get<std::vector<int>>()) accept_hist_.push_back(uint8_t(a));
    }
    accept_log_   = mod.value("accept_log", accept_log_);
    need_publish_ = mod.value("need_publish", need_publish_);
    noise_ema_ = mod.value("noise_ema", noise_ema_);
    noise_n_ = mod.value("noise_n", noise_n_);
    prev_inc_J_ = mod.value("prev_inc_J", prev_inc_J_);
    prev_inc_pair_ = mod.value("prev_inc_pair", prev_inc_pair_);
    accept_margin_ = mod.value("accept_margin", accept_margin_);
    fall_alarm_ = mod.value("fall_alarm", fall_alarm_);
    torque_mag_ = mod.value("torque_mag", torque_mag_);
    auto stats = [this](nlohmann::json const& j, WindowStats& w) {
        w.reset(n_legs_);
        w.falls = j.value("falls", 0);
        w.up_sum = j.value("up_sum", 0.0); w.up_sq = j.value("up_sq", 0.0);
        w.meas_n = j.value("meas_n", int64_t(0)); w.distress_hits = j.value("distress_hits", int64_t(0));
        w.flow_q_sum = j.value("flow_q_sum", 0.0);
        w.flow_mag_sum = j.value("flow_mag_sum", 0.0);
        w.flow_pred_sum = j.value("flow_pred_sum", 0.0);
        w.flow_turn_sum = j.value("flow_turn_sum", 0.0); w.dwell_sum = j.value("dwell_sum", 0.0); w.torque_sum = j.value("torque_sum", 0.0);
        if (j.contains("td"))       w.td       = j["td"].get<std::vector<int>>();
        if (j.contains("unloaded")) w.unloaded = j["unloaded"].get<std::vector<int>>();
    };
    if (mod.contains("cur"))       stats(mod["cur"], cur_);
    if (mod.contains("inc_stats")) stats(mod["inc_stats"], inc_stats_);
    auto terms = [](nlohmann::json const& j, Terms& t) {
        t.falls = j.value("falls", 0.0); t.tilt_sd = j.value("tilt_sd", 0.0);
        t.dwell = j.value("dwell", 0.0); t.energy = j.value("energy", 0.0);
        t.distress_duty = j.value("distress_duty", 0.0); t.unloaded_mean = j.value("unloaded_mean", 0.0);
        t.flow_term = j.value("flow_term", 0.0); t.loaded_min = j.value("loaded_min", 0.0);
        t.flow_mag = j.value("flow_mag", 0.0); t.flow_pred = j.value("flow_pred", 0.0);
        t.flow_turn = j.value("flow_turn", 0.0);
        t.J = j.value("J", 0.0); t.valid = j.value("valid", false);
    };
    if (mod.contains("inc_terms"))  terms(mod["inc_terms"], inc_terms_);
    if (mod.contains("cand_terms")) terms(mod["cand_terms"], cand_terms_);
    upright_  = mod.value("upright", upright_);
    distress_ = mod.value("distress", distress_);
    fwd_v_    = mod.value("fwd_v", fwd_v_);
    if (mod.contains("foot_load"))    foot_load_    = mod["foot_load"].get<std::vector<float>>();
    if (mod.contains("foot_contact")) foot_contact_ = mod["foot_contact"].get<std::vector<float>>();
    fall_below_run_ = mod.value("fall_below_run", fall_below_run_);
    fall_latched_   = mod.value("fall_latched", fall_latched_);
    if (mod.contains("contact_prev")) contact_prev_ = mod["contact_prev"].get<std::vector<float>>();
    if (mod.contains("td_horizon"))   td_horizon_   = mod["td_horizon"].get<std::vector<int64_t>>();
    if (mod.contains("td_maxload"))   td_maxload_   = mod["td_maxload"].get<std::vector<float>>();
    flow_ema_     = mod.value("flow_ema", flow_ema_);
    flow_vol_ema_ = mod.value("flow_vol_ema", flow_vol_ema_);
    turn_ema_     = mod.value("turn_ema", turn_ema_);
}

nlohmann::json GainEvolver::metrics() const {
    nlohmann::json m;
    m["generation"] = generation_;
    m["accepts"]    = accepts_;
    m["reverts"]    = reverts_;
    m["publishes"]  = publishes_;
    m["sigma"]      = sigma_;
    m["phase"]      = int(phase_);
    m["win_tick"]   = win_tick_;
    m["J_inc"]      = inc_terms_.valid ? inc_terms_.J : -1.0;
    m["J_cand"]     = cand_terms_.valid ? cand_terms_.J : -1.0;
    m["falls"]         = inc_terms_.falls;
    m["tilt_sd"]       = inc_terms_.tilt_sd;
    m["dwell"]         = inc_terms_.dwell;
    m["energy"]        = inc_terms_.energy;
    m["fall_alarm"]    = fall_alarm_;
    m["alarm_on"]      = (fall_alarm_on_ > 0.0 && fall_alarm_ > fall_alarm_on_) ? 1 : 0;
    m["distress_duty"] = inc_terms_.distress_duty;
    m["unloaded_mean"] = inc_terms_.unloaded_mean;
    m["flow_term"]     = inc_terms_.flow_term;
    m["flow_mag"]      = inc_terms_.flow_mag;    // < flow_pred => travel is the limit
    m["flow_pred"]     = inc_terms_.flow_pred;   // < flow_mag  => steadiness is the limit
    m["flow_turn"]     = inc_terms_.flow_turn;   // < the others => circling is the limit
    m["flow_min_form"] = double(flow_min_form_);
    m["loaded_min"]    = inc_terms_.loaded_min;
    m["sigma_est"]     = sigma_est();
    m["target_eff"]    = (target_accept_ >= 0.0) ? target_accept_ : noise_floor_rate();
    m["accept_margin"] = accept_margin_;
    m["noise_n"]       = noise_n_;
    m["travel_rx_n"]   = travel_rx_n_;   // consumer half of the travel_topic two-sided meter
    m["fwd_v_in"]      = fwd_v_;         // the flow input actually being consumed this tick
    m["vec"] = (phase_ == Phase::Candidate) ? candidate_ : incumbent_;
    return m;
}

nlohmann::json GainEvolver::diag_snapshot() const {
    nlohmann::json j = metrics();
    j["gain_keys"]  = gain_keys_;
    j["incumbent"]  = incumbent_;
    j["candidate"]  = candidate_;
    j["accept_log"] = accept_log_;
    j["overrides"]  = overrides_;
    j["eval_window_ticks"] = eval_window_ticks_;
    // The declared BOUNDS and the seed point: a viewer cannot draw "where does
    // this gain sit in its range" without them, and each gain's range differs by
    // more than an order of magnitude (height_homeo 0..0.1 vs coupling 0..3), so
    // a shared raw axis would pin seven of eight traces flat to the bottom.
    j["gain_min"]   = gain_min_;
    j["gain_max"]   = gain_max_;
    j["gain_seed"]  = gain_seed_;
    // The criterion WEIGHTS, so a viewer can show each term's actual
    // CONTRIBUTION to J (w*term) rather than its raw value — a large raw term
    // with a small weight decides nothing, and that distinction is exactly what
    // "is this term dead?" asks.
    j["weights"] = nlohmann::json{{"falls", w_falls_}, {"tilt_sd", w_tilt_sd_}, {"dwell", w_dwell_}, {"energy", w_energy_},
                                  {"distress_duty", w_distress_},
                                  {"unloaded_mean", w_unloaded_}, {"flow_term", w_flow_}};
    nlohmann::json ct;
    ct["falls"] = cand_terms_.falls; ct["tilt_sd"] = cand_terms_.tilt_sd;
    ct["distress_duty"] = cand_terms_.distress_duty; ct["unloaded_mean"] = cand_terms_.unloaded_mean;
    ct["flow_term"] = cand_terms_.flow_term; ct["loaded_min"] = cand_terms_.loaded_min;
    ct["flow_mag"] = cand_terms_.flow_mag; ct["flow_pred"] = cand_terms_.flow_pred;
    ct["flow_turn"] = cand_terms_.flow_turn;
    j["cand_terms"] = std::move(ct);
    return j;
}

} // namespace ogma
