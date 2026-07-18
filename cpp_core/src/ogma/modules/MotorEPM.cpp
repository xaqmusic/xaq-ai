#include "ogma/modules/MotorEPM.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
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
    throw std::invalid_argument("MotorEPM param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("MotorEPM param '" + key + "' must be integer");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("MotorEPM param '" + key + "' must be string array");
}
std::vector<double> get_double_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("MotorEPM param '" + key + "' must be numeric array");
}

} // namespace

MotorEPM::MotorEPM()  = default;
MotorEPM::~MotorEPM() = default;

std::string_view MotorEPM::type_name() const { return "MotorEPM"; }

std::vector<TopicSpec> MotorEPM::input_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(proprio_topics_.size());
    for (auto const& t : proprio_topics_)
        v.emplace_back(t, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!tilt_topic_.empty())
        v.emplace_back(tilt_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!imu_topic_.empty())
        v.emplace_back(imu_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!nav_topic_.empty())
        v.emplace_back(nav_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!cog_steer_topic_.empty())
        v.emplace_back(cog_steer_topic_, std::type_index(typeid(ActionOut)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!cog_thrust_topic_.empty())
        v.emplace_back(cog_thrust_topic_, std::type_index(typeid(ActionOut)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!height_topic_.empty())
        v.emplace_back(height_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!distress_topic_.empty())
        v.emplace_back(distress_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!lateral_topic_.empty())
        v.emplace_back(lateral_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!feet_topic_.empty())
        v.emplace_back(feet_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    return v;
}

std::vector<TopicSpec> MotorEPM::output_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(action_topics_.size());
    for (auto const& t : action_topics_)
        v.emplace_back(t, std::type_index(typeid(ActionOut)),
                       SubscriptionKind::Direct, /*required=*/true);
    return v;
}

ParamSchema MotorEPM::params_schema() const {
    return {
        {"proprio_topics", ParamMutability::ConstructionOnly,
         "ProprioToken input topics, one per leg (length = n_legs). Each carries the leg-state vector x.",
         std::nullopt, std::nullopt, std::nullopt},
        {"action_topics", ParamMutability::ConstructionOnly,
         "ActionOut output topics, length n_legs*motor_dim, ordered [leg0_j0, leg0_j1, leg0_j2, leg1_j0, ...]. Drop-in for the body's action.<leg>_<joint> channels.",
         std::nullopt, std::nullopt, std::nullopt},
        {"n_legs", ParamMutability::ConstructionOnly, "number of legs",
         ParamValue{int64_t(4)}, ParamValue{int64_t(1)}, ParamValue{int64_t(8)}},
        {"motor_dim", ParamMutability::ConstructionOnly, "motors per leg",
         ParamValue{int64_t(3)}, ParamValue{int64_t(1)}, ParamValue{int64_t(6)}},
        {"model_lr", ParamMutability::HotMutable, "forward-model learning rate η_M",
         ParamValue{0.02}, ParamValue{0.0}, ParamValue{1.0}},
        {"ctrl_lr", ParamMutability::HotMutable, "controller (homeokinetic) learning rate η_K",
         ParamValue{0.01}, ParamValue{0.0}, ParamValue{1.0}},
        {"bias_lr", ParamMutability::HotMutable, "controller bias learning rate η_h",
         ParamValue{0.005}, ParamValue{0.0}, ParamValue{1.0}},
        {"reg_eps", ParamMutability::HotMutable, "ε in (LLᵀ+εI)⁻¹ sensitivity-metric regularizer",
         ParamValue{0.01}, ParamValue{1e-6}, ParamValue{10.0}},
        {"max_dctrl", ParamMutability::HotMutable, "per-tick clamp on ‖ΔC‖_F (ignition stability guard)",
         ParamValue{0.05}, ParamValue{0.0}, ParamValue{10.0}},
        {"init_scale", ParamMutability::ConstructionOnly, "init magnitude of C (small → starts near standing)",
         ParamValue{0.01}, ParamValue{0.0}, ParamValue{1.0}},
        {"seed", ParamMutability::ConstructionOnly, "base RNG seed; per-leg seed = base ^ leg",
         ParamValue{int64_t(1234)}, std::nullopt, std::nullopt},
        {"babble_ticks", ParamMutability::ConstructionOnly,
         "motor-babble warmup: for this many proprio frames the model learns from small random commands while the controller stays idle, so HK starts from a model that predicts the body (no startup convulsion).",
         ParamValue{int64_t(200)}, ParamValue{int64_t(0)}, ParamValue{int64_t(100000)}},
        {"babble_scale", ParamMutability::HotMutable, "amplitude of warmup babble commands",
         ParamValue{0.3}, ParamValue{0.0}, ParamValue{1.0}},
        {"sat_lr", ParamMutability::HotMutable,
         "anti-saturation rate: pushes the operating point z=Cx+h out of the tanh rails (surrogate for the dropped ∂G term). Keeps the controller in its sensitive band so g'>0 and HK can't freeze.",
         ParamValue{0.02}, ParamValue{0.0}, ParamValue{1.0}},
        {"postural_gain", ParamMutability::HotMutable,
         "postural reflex strength (spinal-tone analog): weak PD pull of the hip2+knee commands toward the standing REST pose (proprio pos=0). Gives HK a stable upright fixed point to bifurcate from. 0 = off. Reward-free.",
         ParamValue{0.3}, ParamValue{0.0}, ParamValue{2.0}},
        {"explore_noise", ParamMutability::HotMutable,
         "persistent Gaussian motor-noise σ added every tick. Keeps the prediction error ξ nonzero at fixed points so HK does not freeze; the sensitivity-seeking controller amplifies it into oscillation (the homeokinetic exploration drive).",
         ParamValue{0.05}, ParamValue{0.0}, ParamValue{1.0}},
        {"knee_tuck_target", ParamMutability::HotMutable,
         "override the postural knee-rest target (proprio pos) to drive the statically-stable SPIDER stance (knees tucked, chassis suspended below). +0.7..+0.9 = strong tuck. -99 = use the captured spawn pose.",
         ParamValue{-99.0}, ParamValue{-99.0}, ParamValue{1.0}},
        {"motor_gain", ParamMutability::HotMutable,
         "output amplitude multiplier on the HK command (tanh output, before postural+noise). 1 = raw HK; >1 = stronger/larger leg swings (the legs look weak even though servos are strong).",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{5.0}},
        {"coupling_gain", ParamMutability::HotMutable,
         "Rung 3 inter-leg Kuramoto coupling strength. Couples the four legs' own emergent phases toward the gait_phase offsets (entrains the twitching legs to the active one, phase-locks all four). 0 = off (one-leg-spins regime); raise to watch the legs synchronize.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
        {"gait_phase", ParamMutability::HotMutable,
         "per-leg target phase offsets (rad), length n_legs, order [FL,FR,RL,RR]. Default trot [0,π,π,0] (diagonals in phase). The imposed coordination topology; rhythm/frequency emerge.",
         std::nullopt, std::nullopt, std::nullopt},
        {"coord_adapt_rate", ParamMutability::HotMutable,
         "ADAPTIVE COORDINATION: rate the gait_phase offsets crystallise toward the body's OWN emergent per-leg phase pattern (reward-free Hebbian self-organisation — stop imposing the trot, let the coordination self-organise). 0 = fixed offsets. ~0.001 ≈ 16 s.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{0.02}},
        {"coord_explore", ParamMutability::HotMutable,
         "persistent phase-offset exploration noise (rad/tick) so the coordination never fully freezes — the homeokinetic 'keep probing' for continuous gait improvement. 0 = none.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{0.1}},
        {"coord_reward_drive", ParamMutability::HotMutable,
         "AGENCY-REWARD search: (1+1) hill-climb probe scale (rad) on the phase offsets, KEEPING probes that raise controllability (forward thrust fwd_v — coordinated propulsion, Goodhart-robust on flat ground). The directional drive for continuous improvement. 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"coord_probe_ticks", ParamMutability::HotMutable,
         "window (ticks) per agency-reward probe; fitness measured over its back half (after the gait settles to the new offsets). ~240 = 4 s.",
         ParamValue{240}, ParamValue{60}, ParamValue{1200}},
        {"coord_stab_penalty", ParamMutability::HotMutable,
         "AGENCY-REWARD edge-of-chaos guard: subtracts coord_stab_penalty * |tilt| from each probe's fitness, so probes that go faster by going wobbly are rejected. Keeps the climb on the alive (predictable) side of the controllability frontier. 0 = pure thrust fitness.",
         ParamValue{0.3}, ParamValue{0.0}, ParamValue{2.0}},
        {"coord_lat_penalty", ParamMutability::HotMutable,
         "AGENCY-REWARD anti-crab term: subtracts coord_lat_penalty * |lateral velocity| from each probe's fitness, so the search rejects phase patterns that thrust forward by fishtailing sideways. Straightens the gait (cancels the rear-leg-antiphase crab/wobble the forward-only fitness left unpenalised) and is expected to RAISE transport efficiency. 0 = off (crab allowed).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
        {"coord_intent_nav", ParamMutability::HotMutable,
         "SYMMETRIC CONTROLLABILITY reward. 0 = the agency search rewards FORWARD velocity (legacy). >0 = it rewards velocity toward the INTENDED direction (the target_compass bearing) — i.e. progress toward the goal, whether that needs going forward OR turning. Generalises the forward-only agency reward (forward = intent straight ahead); a substrate motivated to ACHIEVE its intent, symmetric for forward and turns. Falls back to forward when no target is present.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"cruse_gain", ParamMutability::HotMutable,
         "CRUSE/Walknet inter-leg coordination CORRECTOR on hip2 (foot lift). Continuous bias: Rule 1 (anterior leg in swing → hold this leg in stance, +hip2 down), Rule 2 (anterior just-planted → release this leg's swing, −hip2 up), Rule 3 (contralateral in swing → hold stance). Catches per-leg co-swing / support-loss the MotorEPM rhythm alone leaves; needs the rhythm it cannot itself generate. 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
        {"cruse_rule3_weight", ParamMutability::HotMutable,
         "Rule 3 (contralateral load tolerance) weight relative to Rule 1 (anterior). 0.5 = contralateral hold is half-strength vs the anterior coupling.",
         ParamValue{0.5}, ParamValue{0.0}, ParamValue{2.0}},
        {"cruse_rule2_window", ParamMutability::HotMutable,
         "ticks after the anterior leg's touchdown during which Rule 2 actively releases this leg's swing (the constructive lift). ~15 = 0.25 s @ 60 Hz.",
         ParamValue{15}, ParamValue{1}, ParamValue{120}},
        {"cruse_rule5_gain", ParamMutability::HotMutable,
         "CRUSE Rule 5 (load distribution): a leg in stance presses its foot down (hip2+knee) ∝ the number of OTHER legs currently in swing — redistributing the swinging legs' weight onto the planted ones. More normal force → more friction → less foot scrub (the stance feet were sliding ~3-4× the body's progress). Shifts CoG onto the support. 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"feet_topic", ParamMutability::ConstructionOnly,
         "4-D per-leg foot-height ProprioToken topic for Cruse stance/swing detection. The body publishes reality.proprio.feet_y every tick.",
         std::nullopt, std::nullopt, std::nullopt},
        {"stroke_gain", ParamMutability::HotMutable,
         "hip1 (fore-aft) propulsion drive amplitude, phase-locked to each leg's step phase. Aligns the stroke DIRECTION so thrust sums to translation, not the tangential spin HK settles into. 0 = HK-only hip1.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
        {"stroke_phase", ParamMutability::HotMutable,
         "phase offset (rad) between the knee/step phase and the hip1 fore-aft drive — tunes when the leg pushes back relative to when it lifts.",
         ParamValue{0.0}, ParamValue{-3.15}, ParamValue{3.15}},
        {"steer", ParamMutability::HotMutable,
         "left/right hip1 drive differential — the steering lever. 0 = straight; ± = turn (the clockwise spin is steer-like). Adds side_sign·steer to each leg's stroke.",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"stroke_signs", ParamMutability::HotMutable,
         "per-leg hip1 stroke direction, length n_legs [FL,FR,RL,RR]. Parallel-caudal pattern → forward; all-same → tangential spin. Default [1,-1,1,-1] (forward guess, confirm by eye).",
         std::nullopt, std::nullopt, std::nullopt},
        {"balance_gain", ParamMutability::HotMutable,
         "active-balance (vestibular) reflex strength. Pushes low-side legs' hip2 to level the body from the chassis tilt direction — equalizes leg load → equal basin depth → straighter walk + fewer stalls. 0 = off; sign tunable (flip if it destabilizes).",
         ParamValue{0.0}, ParamValue{-3.0}, ParamValue{3.0}},
        {"tilt_topic", ParamMutability::ConstructionOnly,
         "4-D tilt ProprioToken topic [sin(pitch),cos(pitch),sin(roll),cos(roll)] for active balance. Requires the body's publish_tilt enabled.",
         std::nullopt, std::nullopt, std::nullopt},
        {"amp_homeo_gain", ParamMutability::HotMutable,
         "per-leg amplitude-homeostat integral rate. Drives every leg's oscillation amplitude toward amp_target — kills the basin asymmetry (the wander) AND the path-dependence (converges regardless of starting basin / slider history). 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{0.05}},
        {"amp_target", ParamMutability::HotMutable,
         "target per-leg oscillation amplitude (phase-vector magnitude) the homeostat regulates toward. ~0.4 ≈ the deep-basin amplitude.",
         ParamValue{0.4}, ParamValue{0.0}, ParamValue{2.0}},
        {"amp_seek_rate", ParamMutability::HotMutable,
         "CoT-SEEKING amplitude search: (1+1) hill-climb probe scale on amp_target, KEEPING probes that raise fwd_v / oscillation-amplitude (speed per effort = inverse cost of transport). Lowers gait amplitude until forward thrust starts to fall → the efficient amplitude. Attacks the dominant motor-effort term the phase search can't. Needs amp_homeo on. 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{0.2}},
        {"amp_seek_ticks", ParamMutability::HotMutable,
         "window (ticks) per CoT amplitude probe; fitness measured over its back half (after the homeostat settles to the new amp_target). ~900 = 15 s.",
         ParamValue{900}, ParamValue{120}, ParamValue{3600}},
        {"heading_gain", ParamMutability::HotMutable,
         "heading-rate regulator (go-straight reflex). Feeds the body's signed yaw rate into steer to counter unwanted turning — kills the residual wander after amplitudes are equalized. steer becomes a turn-rate command (0 = hold straight). 0 = off; sign tunable.",
         ParamValue{0.0}, ParamValue{-3.0}, ParamValue{3.0}},
        {"imu_topic", ParamMutability::ConstructionOnly,
         "4-D IMU ProprioToken topic [sin yaw, cos yaw, fwd_v, ang_v]; index 3 = signed yaw rate for the heading regulator.",
         std::nullopt, std::nullopt, std::nullopt},
        {"nav_gain", ParamMutability::HotMutable,
         "PERCEPTION→STEERING: steer-toward-target strength. Reads the egocentric target bearing (target_compass) and steers to drive its lateral component to zero (target dead-ahead) — the active-inference closure. 0 = off; sign tunable.",
         ParamValue{0.0}, ParamValue{-4.0}, ParamValue{4.0}},
        {"nav_topic", ParamMutability::ConstructionOnly,
         "2-D egocentric target-bearing ProprioToken (target_compass: body-frame unit vector to the active target).",
         std::nullopt, std::nullopt, std::nullopt},
        {"cog_steer_gain", ParamMutability::HotMutable,
         "COGNITIVE→STEERING (cell, n_legs=1/motor_dim=2): strength of a scalar steer (e.g. ActionDecoder's learned left/straight/right) biasing the two-flagella differential. The slow cognitive critic directs the alive swimmer. 0 = off; sign tunable.",
         ParamValue{0.0}, ParamValue{-4.0}, ParamValue{4.0}},
        {"cog_steer_topic", ParamMutability::ConstructionOnly,
         "Scalar ActionOut topic for the cognitive steer (its accel, normalized by /4, biases the flagella differential). Empty = off.",
         std::nullopt, std::nullopt, std::nullopt},
        {"cog_thrust_gain", ParamMutability::HotMutable,
         "COGNITIVE→THRUST (cell, n_legs=1/motor_dim=2): strength of a learned scalar driving COMMON-mode (forward/reverse/pause) on the two flagella, so the cognitive actor can move toward higher scent (the action that changes proximity). Mirrors cog_steer (differential). 0 = off; sign tunable.",
         ParamValue{0.0}, ParamValue{-4.0}, ParamValue{4.0}},
        {"cog_thrust_topic", ParamMutability::ConstructionOnly,
         "Scalar ActionOut topic for the cognitive thrust (its accel, normalized by /4, biases the flagella common-mode). Empty = off.",
         std::nullopt, std::nullopt, std::nullopt},
        {"boredom_noise_gain", ParamMutability::HotMutable,
         "BOREDOM ESCAPE (cell, n_legs=1/motor_dim=2): σ of undirected DIFFERENTIAL (heading) noise injected ∝ boredom (DistressDrive's cognition.boredom). A frozen sensorimotor loop (pinned at a wall) is BORING → sample new headings until something changes; cog steer also fades by (1-boredom). Self-terminating. 0 = off (picrawler/Stage-1 untouched). NOT a turn-away reflex.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{4.0}},
        {"boredom_topic", ParamMutability::ConstructionOnly,
         "ReflexGate topic carrying the 0..1 boredom signal (DistressDrive). Empty = off.",
         std::nullopt, std::nullopt, std::nullopt},
        {"interest_topic", ParamMutability::ConstructionOnly,
         "ReflexGate topic carrying the 0..1 curiosity interest (DistressDrive). Steers the boredom escape: RUN forward when interest high (open/scent-rich ahead), TUMBLE (turn) when low. Empty = pure undirected tumble.",
         std::nullopt, std::nullopt, std::nullopt},
        {"hunger_topic", ParamMutability::ConstructionOnly,
         "ProprioToken topic (reality.proprio.hunger = 1-energy).  When set, the food-ward nav steer is SCALED by hunger (FORAGE when hungry, ignore food when sated) and hunger compounds the boredom escape into desperation.  Empty = nav not hunger-gated.",
         std::nullopt, std::nullopt, std::nullopt},
        {"boredom_escalation_rate", ParamMutability::HotMutable,
         "Escape amplitude growth per tick of sustained boredom (do-more-until-bifurcation): amplitude *= (1 + rate*streak), capped.  Resets when boredom drops (escaped / getting warmer).  0 = no escalation (level-only).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"height_homeo_gain", ParamMutability::HotMutable,
         "chassis-height homeostat (stand-higher reflex) integral rate. The G6DOF springs let the body SAG; the postural reflex defends a joint-angle pose, not a height. This drives a tuck-deepening knee bias toward a SELF-DISCOVERED setpoint (height_k × tallest height reached) so the body stands as tall as it can. 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{0.1}},
        {"height_k", ParamMutability::HotMutable,
         "fraction of the self-discovered max chassis height the homeostat defends (the body finds its own ceiling; this sets how close to it to hold). ~0.9.",
         ParamValue{0.9}, ParamValue{0.0}, ParamValue{1.0}},
        {"height_topic", ParamMutability::ConstructionOnly,
         "1-D chassis-height ProprioToken topic (normalised [0,1] = chassis_y/target_height). The body publishes reality.proprio.chassis_y_norm every tick unconditionally.",
         std::nullopt, std::nullopt, std::nullopt},
        {"panic_on", ParamMutability::HotMutable,
         "PANIC pathway: distress level that ENGAGES panic (hysteresis high). Above this the gait is overridden by decoupled flailing to escape a wedge. Reward-free.",
         ParamValue{0.5}, ParamValue{0.0}, ParamValue{1.0}},
        {"panic_off", ParamMutability::HotMutable,
         "distress level that DISENGAGES panic (hysteresis low). Below this the gait resumes. Keep < panic_on to avoid chatter.",
         ParamValue{0.25}, ParamValue{0.0}, ParamValue{1.0}},
        {"panic_strength", ParamMutability::HotMutable,
         "overall panic effect scale (0 = panic disabled; 1 = full). Scales the decouple + noise + motor-boost together.",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"panic_noise", ParamMutability::HotMutable,
         "explore_noise ADDED at full panic (high-amplitude flailing to dislodge).",
         ParamValue{0.4}, ParamValue{0.0}, ParamValue{1.0}},
        {"panic_motor_mult", ParamMutability::HotMutable,
         "motor_gain multiplier at full panic (stronger escape thrust).",
         ParamValue{1.8}, ParamValue{1.0}, ParamValue{4.0}},
        {"panic_push_amp", ParamMutability::HotMutable,
         "PUSH REFLEX amplitude: a coherent low-freq pump on hip2+knee that drives the legs through full range to lever off an obstacle (per-tick noise alone is low-passed to jitter). ≥1 saturates the joints. 0 = off.",
         ParamValue{1.2}, ParamValue{0.0}, ParamValue{3.0}},
        {"panic_push_hz", ParamMutability::HotMutable,
         "PUSH REFLEX frequency (Hz). Low (~0.8) so the pump survives the servo/spring low-pass and produces big excursions, not jitter.",
         ParamValue{0.8}, ParamValue{0.1}, ParamValue{3.0}},
        {"distress_topic", ParamMutability::ConstructionOnly,
         "1-D distress (wedge severity [0,1]) ProprioToken topic. The body publishes reality.proprio.distress every tick.",
         std::nullopt, std::nullopt, std::nullopt},
        {"lateral_topic", ParamMutability::ConstructionOnly,
         "1-D signed lateral (sideways-slip) velocity ProprioToken topic, fed to the anti-crab coord_lat_penalty. The body publishes reality.proprio.lateral_v every tick.",
         std::nullopt, std::nullopt, std::nullopt},
    };
}

ParamMap MotorEPM::current_params() const {
    ParamMap m;
    std::vector<std::string> pt(proprio_topics_), at(action_topics_);
    m["proprio_topics"] = pt;
    m["action_topics"]  = at;
    m["n_legs"]     = int64_t(n_legs_);
    m["motor_dim"]  = int64_t(motor_dim_);
    m["model_lr"]   = model_lr_;
    m["ctrl_lr"]    = ctrl_lr_;
    m["bias_lr"]    = bias_lr_;
    m["reg_eps"]    = reg_eps_;
    m["max_dctrl"]  = max_dctrl_;
    m["init_scale"]   = init_scale_;
    m["seed"]         = base_seed_;
    m["babble_ticks"]  = babble_ticks_;
    m["babble_scale"]  = babble_scale_;
    m["sat_lr"]        = sat_lr_;
    m["postural_gain"]    = postural_gain_;
    m["explore_noise"]    = explore_noise_;
    m["knee_tuck_target"] = knee_tuck_target_;
    m["motor_gain"]       = motor_gain_;
    m["coupling_gain"]    = coupling_gain_;
    m["gait_phase"]       = gait_phase_;
    m["coord_adapt_rate"] = coord_adapt_rate_;
    m["coord_explore"]    = coord_explore_;
    m["coord_reward_drive"] = coord_reward_drive_;
    m["coord_probe_ticks"]  = coord_probe_ticks_;
    m["coord_stab_penalty"] = coord_stab_penalty_;
    m["coord_lat_penalty"]  = coord_lat_penalty_;
    m["coord_intent_nav"]   = coord_intent_nav_;
    m["cruse_gain"]         = cruse_gain_;
    m["cruse_rule3_weight"] = cruse_rule3_weight_;
    m["cruse_rule2_window"] = cruse_rule2_window_;
    m["cruse_rule5_gain"]   = cruse_rule5_gain_;
    m["feet_topic"]         = feet_topic_;
    m["stroke_gain"]      = stroke_gain_;
    m["stroke_phase"]     = stroke_phase_;
    m["steer"]            = steer_;
    m["stroke_signs"]     = stroke_signs_;
    m["balance_gain"]     = balance_gain_;
    m["tilt_topic"]       = tilt_topic_;
    m["amp_homeo_gain"]   = amp_homeo_gain_;
    m["amp_target"]       = amp_target_;
    m["amp_seek_rate"]    = amp_seek_rate_;
    m["amp_seek_ticks"]   = amp_seek_ticks_;
    m["heading_gain"]     = heading_gain_;
    m["imu_topic"]        = imu_topic_;
    m["nav_gain"]         = nav_gain_;
    m["nav_topic"]        = nav_topic_;
    m["cog_steer_gain"]   = cog_steer_gain_;
    m["cog_steer_topic"]  = cog_steer_topic_;
    m["boredom_noise_gain"] = boredom_noise_gain_;
    m["boredom_topic"]      = boredom_topic_;
    m["interest_topic"]     = interest_topic_;
    m["hunger_topic"]       = hunger_topic_;
    m["boredom_escalation_rate"] = boredom_escalation_rate_;
    m["height_homeo_gain"] = height_homeo_gain_;
    m["height_k"]          = height_k_;
    m["height_topic"]      = height_topic_;
    m["panic_on"]          = panic_on_;
    m["panic_off"]         = panic_off_;
    m["panic_strength"]    = panic_strength_;
    m["panic_noise"]       = panic_noise_;
    m["panic_motor_mult"]  = panic_motor_mult_;
    m["panic_push_amp"]    = panic_push_amp_;
    m["panic_push_hz"]     = panic_push_hz_;
    m["distress_topic"]    = distress_topic_;
    m["lateral_topic"]     = lateral_topic_;
    return m;
}

void MotorEPM::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("MotorEPM::on_setup: null bus");

    apply_param(params, "n_legs",     [&](auto const& v){ n_legs_     = int(get_int(v, "n_legs")); });
    apply_param(params, "motor_dim",  [&](auto const& v){ motor_dim_  = int(get_int(v, "motor_dim")); });
    apply_param(params, "model_lr",   [&](auto const& v){ model_lr_   = get_double(v, "model_lr"); });
    apply_param(params, "ctrl_lr",    [&](auto const& v){ ctrl_lr_    = get_double(v, "ctrl_lr"); });
    apply_param(params, "bias_lr",    [&](auto const& v){ bias_lr_    = get_double(v, "bias_lr"); });
    apply_param(params, "reg_eps",    [&](auto const& v){ reg_eps_    = get_double(v, "reg_eps"); });
    apply_param(params, "max_dctrl",  [&](auto const& v){ max_dctrl_  = get_double(v, "max_dctrl"); });
    apply_param(params, "init_scale", [&](auto const& v){ init_scale_ = get_double(v, "init_scale"); });
    apply_param(params, "seed",       [&](auto const& v){ base_seed_  = get_int(v, "seed"); });
    apply_param(params, "babble_ticks", [&](auto const& v){ babble_ticks_ = get_int(v, "babble_ticks"); });
    apply_param(params, "babble_scale", [&](auto const& v){ babble_scale_ = get_double(v, "babble_scale"); });
    apply_param(params, "sat_lr",       [&](auto const& v){ sat_lr_       = get_double(v, "sat_lr"); });
    apply_param(params, "postural_gain", [&](auto const& v){ postural_gain_ = get_double(v, "postural_gain"); });
    apply_param(params, "explore_noise", [&](auto const& v){ explore_noise_ = get_double(v, "explore_noise"); });
    apply_param(params, "knee_tuck_target", [&](auto const& v){ knee_tuck_target_ = get_double(v, "knee_tuck_target"); });
    apply_param(params, "motor_gain", [&](auto const& v){ motor_gain_ = get_double(v, "motor_gain"); });
    apply_param(params, "coupling_gain", [&](auto const& v){ coupling_gain_ = get_double(v, "coupling_gain"); });
    apply_param(params, "gait_phase", [&](auto const& v){ gait_phase_ = get_double_vec(v, "gait_phase"); });
    apply_param(params, "coord_adapt_rate", [&](auto const& v){ coord_adapt_rate_ = get_double(v, "coord_adapt_rate"); });
    apply_param(params, "coord_explore", [&](auto const& v){ coord_explore_ = get_double(v, "coord_explore"); });
    apply_param(params, "coord_reward_drive", [&](auto const& v){ coord_reward_drive_ = get_double(v, "coord_reward_drive"); });
    apply_param(params, "coord_probe_ticks", [&](auto const& v){ coord_probe_ticks_ = get_int(v, "coord_probe_ticks"); });
    apply_param(params, "coord_stab_penalty", [&](auto const& v){ coord_stab_penalty_ = get_double(v, "coord_stab_penalty"); });
    apply_param(params, "coord_lat_penalty", [&](auto const& v){ coord_lat_penalty_ = get_double(v, "coord_lat_penalty"); });
    apply_param(params, "coord_intent_nav", [&](auto const& v){ coord_intent_nav_ = get_double(v, "coord_intent_nav"); });
    apply_param(params, "cruse_gain", [&](auto const& v){ cruse_gain_ = get_double(v, "cruse_gain"); });
    apply_param(params, "cruse_rule3_weight", [&](auto const& v){ cruse_rule3_weight_ = get_double(v, "cruse_rule3_weight"); });
    apply_param(params, "cruse_rule2_window", [&](auto const& v){ cruse_rule2_window_ = get_int(v, "cruse_rule2_window"); });
    apply_param(params, "cruse_rule5_gain", [&](auto const& v){ cruse_rule5_gain_ = get_double(v, "cruse_rule5_gain"); });
    apply_param(params, "feet_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) feet_topic_ = *p; });
    apply_param(params, "stroke_gain", [&](auto const& v){ stroke_gain_ = get_double(v, "stroke_gain"); });
    apply_param(params, "stroke_phase", [&](auto const& v){ stroke_phase_ = get_double(v, "stroke_phase"); });
    apply_param(params, "steer", [&](auto const& v){ steer_ = get_double(v, "steer"); });
    apply_param(params, "stroke_signs", [&](auto const& v){ stroke_signs_ = get_double_vec(v, "stroke_signs"); });
    apply_param(params, "balance_gain", [&](auto const& v){ balance_gain_ = get_double(v, "balance_gain"); });
    apply_param(params, "tilt_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) tilt_topic_ = *p; });
    apply_param(params, "amp_homeo_gain", [&](auto const& v){ amp_homeo_gain_ = get_double(v, "amp_homeo_gain"); });
    apply_param(params, "amp_target", [&](auto const& v){ amp_target_ = get_double(v, "amp_target"); });
    apply_param(params, "amp_seek_rate", [&](auto const& v){ amp_seek_rate_ = get_double(v, "amp_seek_rate"); });
    apply_param(params, "amp_seek_ticks", [&](auto const& v){ amp_seek_ticks_ = get_int(v, "amp_seek_ticks"); });
    apply_param(params, "heading_gain", [&](auto const& v){ heading_gain_ = get_double(v, "heading_gain"); });
    apply_param(params, "imu_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) imu_topic_ = *p; });
    apply_param(params, "nav_gain", [&](auto const& v){ nav_gain_ = get_double(v, "nav_gain"); });
    apply_param(params, "nav_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) nav_topic_ = *p; });
    apply_param(params, "cog_steer_gain", [&](auto const& v){ cog_steer_gain_ = get_double(v, "cog_steer_gain"); });
    apply_param(params, "cog_steer_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) cog_steer_topic_ = *p; });
    apply_param(params, "cog_thrust_gain", [&](auto const& v){ cog_thrust_gain_ = get_double(v, "cog_thrust_gain"); });
    apply_param(params, "cog_thrust_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) cog_thrust_topic_ = *p; });
    apply_param(params, "boredom_noise_gain", [&](auto const& v){ boredom_noise_gain_ = get_double(v, "boredom_noise_gain"); });
    apply_param(params, "boredom_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) boredom_topic_ = *p; });
    apply_param(params, "interest_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) interest_topic_ = *p; });
    apply_param(params, "hunger_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) hunger_topic_ = *p; });
    apply_param(params, "boredom_escalation_rate", [&](auto const& v){ boredom_escalation_rate_ = get_double(v, "boredom_escalation_rate"); });
    apply_param(params, "height_homeo_gain", [&](auto const& v){ height_homeo_gain_ = get_double(v, "height_homeo_gain"); });
    apply_param(params, "height_k", [&](auto const& v){ height_k_ = get_double(v, "height_k"); });
    apply_param(params, "height_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) height_topic_ = *p; });
    apply_param(params, "panic_on", [&](auto const& v){ panic_on_ = get_double(v, "panic_on"); });
    apply_param(params, "panic_off", [&](auto const& v){ panic_off_ = get_double(v, "panic_off"); });
    apply_param(params, "panic_strength", [&](auto const& v){ panic_strength_ = get_double(v, "panic_strength"); });
    apply_param(params, "panic_noise", [&](auto const& v){ panic_noise_ = get_double(v, "panic_noise"); });
    apply_param(params, "panic_motor_mult", [&](auto const& v){ panic_motor_mult_ = get_double(v, "panic_motor_mult"); });
    apply_param(params, "panic_push_amp", [&](auto const& v){ panic_push_amp_ = get_double(v, "panic_push_amp"); });
    apply_param(params, "panic_push_hz", [&](auto const& v){ panic_push_hz_ = get_double(v, "panic_push_hz"); });
    apply_param(params, "distress_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) distress_topic_ = *p; });
    apply_param(params, "lateral_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) lateral_topic_ = *p; });
    apply_param(params, "proprio_topics", [&](auto const& v){ proprio_topics_ = get_string_vec(v, "proprio_topics"); });
    apply_param(params, "action_topics",  [&](auto const& v){ action_topics_  = get_string_vec(v, "action_topics"); });

    if (int(proprio_topics_.size()) != n_legs_)
        throw std::invalid_argument("MotorEPM: proprio_topics length must equal n_legs");
    if (int(action_topics_.size()) != n_legs_ * motor_dim_)
        throw std::invalid_argument("MotorEPM: action_topics length must equal n_legs*motor_dim");
    if (int(gait_phase_.size()) != n_legs_)
        gait_phase_.assign(n_legs_, 0.0);   // fall back to in-phase if mis-sized
    if (int(stroke_signs_.size()) != n_legs_)
        stroke_signs_.assign(n_legs_, 1.0);

    legs_.assign(n_legs_, Leg{});

    // Cruse stance/swing state + leg topology.  Picrawler (n=4, order FL,FR,RL,RR):
    // anatomical anterior = [none, none, FL, FR]; contralateral = FL↔FR, RL↔RR.
    foot_y_.assign(n_legs_, 0.0f);
    foot_y_ema_.clear();
    in_swing_.assign(n_legs_, 0);
    ticks_since_plant_.assign(n_legs_, 1000);
    cruse_anterior_.assign(n_legs_, -1);
    cruse_contra_.assign(n_legs_, -1);
    if (n_legs_ == 4) {
        cruse_anterior_ = {-1, -1, 0, 1};
        cruse_contra_   = { 1,  0, 3, 2};
    }

    for (int leg = 0; leg < n_legs_; ++leg) {
        sub_ids_.push_back(bus_->subscribe(
            proprio_topics_[leg], SubscriptionKind::Direct,
            [this, leg](std::string_view /*topic*/, MessagePtr p){ handle_proprio(leg, p); }));
    }
    if (!tilt_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            tilt_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_tilt(p); }));
    }
    if (!imu_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            imu_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_imu(p); }));
    }
    if (!nav_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            nav_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_nav(p); }));
    }
    if (!cog_steer_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            cog_steer_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_cog_steer(p); }));
    }
    if (!cog_thrust_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            cog_thrust_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_cog_thrust(p); }));
    }
    if (!boredom_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            boredom_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_boredom(p); }));
    }
    if (!interest_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            interest_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_interest(p); }));
    }
    if (!hunger_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            hunger_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_hunger(p); }));
    }
    if (!height_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            height_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_height(p); }));
    }
    if (!distress_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            distress_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_distress(p); }));
    }
    if (!lateral_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            lateral_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_lateral(p); }));
    }
    if (!feet_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            feet_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_feet(p); }));
    }
    // Gate 0 (L-1a) — prefix-subscribe the body's disruption events (events.miss
    // on a fall, events.reset on a teleport/respawn) for reset-masking.
    sub_ids_.push_back(bus_->subscribe(
        topics::kEventsPrefix, SubscriptionKind::Direct,
        [this](std::string_view topic, MessagePtr p){ handle_event(topic, p); }));
}

void MotorEPM::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto e = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!e) return;
    std::string name(topic.substr(std::string(topics::kEventsPrefix).size()));
    // A catastrophic fall (miss) or a teleport/respawn (reset) both break the
    // upright bout → reset-mask: restart ticks-since-reset and count the disruption.
    if (name == "miss" || name == "reset") {
        ++reset_count_;
        ticks_since_reset_   = 0;
        reset_hit_this_tick_ = true;
    }
}

void MotorEPM::handle_tilt(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 4) return;
    tilt_pitch_ = pt->values[0];   // sin(pitch) — signed fore-aft tilt
    tilt_roll_  = pt->values[2];   // sin(roll)  — signed left-right tilt
}

void MotorEPM::handle_imu(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 4) return;
    fwd_v_    = pt->values[2];     // forward velocity (controllability/thrust signal)
    yaw_rate_ = pt->values[3];     // signed yaw rate (ang_v / π, clamped)
}

void MotorEPM::handle_nav(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 2) return;
    tc_x_ = pt->values[0];
    tc_y_ = pt->values[1];
}

void MotorEPM::handle_cog_steer(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    cog_steer_ = std::clamp(a->accel / 4.0f, -1.0f, 1.0f);   // normalize accel∈[-4,4] → [-1,1]
    ++cog_steer_msgs_;
}

void MotorEPM::handle_cog_thrust(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    cog_thrust_ = std::clamp(a->accel / 4.0f, -1.0f, 1.0f);   // normalize accel∈[-4,4] → [-1,1]
    ++cog_thrust_msgs_;
}

void MotorEPM::handle_boredom(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto g = std::dynamic_pointer_cast<const ReflexGate>(payload);
    if (!g) return;
    boredom_ = g->active ? std::clamp(g->value, 0.0f, 1.0f) : 0.0f;
    // Boredom-DURATION integrator (do-more-until-something-happens): count how long
    // the loop has stayed frozen.  Reset the instant boredom relaxes (escaped / the
    // world started changing) so it self-terminates — not a runaway.  Tracked here
    // (not in the gated escape block) so it resets reliably even when boredom→0.
    if (boredom_ > 0.5f) ++boredom_streak_;
    else                 boredom_streak_ = 0;
}

void MotorEPM::handle_interest(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto g = std::dynamic_pointer_cast<const ReflexGate>(payload);
    if (!g) return;
    interest_ = g->active ? std::clamp(g->value, 0.0f, 1.0f) : 0.0f;
}

void MotorEPM::handle_hunger(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    hunger_ = std::clamp(pt->values[0], 0.0f, 1.0f);   // 1-energy: 0 sated → 1 starving
}

void MotorEPM::handle_height(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    chassis_h_ = pt->values[0];
    // Spike-robust smoothing, then track the tallest height ever reached as the
    // self-discovered ceiling the homeostat defends a fraction (height_k) of.
    if (!chassis_h_seen_) { chassis_h_ema_ = chassis_h_; chassis_h_seen_ = true; }
    else chassis_h_ema_ = (1.0f - kHeightEmaAlpha) * chassis_h_ema_
                        + kHeightEmaAlpha * chassis_h_;
    if (chassis_h_ema_ > chassis_h_max_) chassis_h_max_ = chassis_h_ema_;
}

void MotorEPM::handle_distress(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    distress_ = pt->values[0];
}

void MotorEPM::handle_lateral(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    lateral_v_ = pt->values[0];   // signed sideways-slip velocity (+ = body-right)
}

void MotorEPM::handle_feet(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int k = std::min<int>(int(pt->values.size()), n_legs_);
    if (int(foot_y_.size()) != n_legs_) foot_y_.assign(n_legs_, 0.0f);
    for (int i = 0; i < k; ++i) foot_y_[i] = float(pt->values[i]);
}

// Per-tick Cruse stance/swing bookkeeping: a leg is in SWING when its foot is above
// its own self-calibrating height EMA; a touchdown (swing→stance) resets its
// since-plant counter (used by Rule 2's release window).
void MotorEPM::update_cruse_state() {
    if (int(foot_y_.size()) != n_legs_) return;
    if (int(foot_y_ema_.size()) != n_legs_) {
        foot_y_ema_ = foot_y_;
        in_swing_.assign(n_legs_, 0);
        ticks_since_plant_.assign(n_legs_, 1000);
    }
    for (int i = 0; i < n_legs_; ++i) {
        foot_y_ema_[i] = (1.0f - kFootYEmaAlpha) * foot_y_ema_[i] + kFootYEmaAlpha * foot_y_[i];
        bool sw = foot_y_[i] > foot_y_ema_[i];
        if (in_swing_[i] && !sw) ticks_since_plant_[i] = 0;   // touchdown
        else ticks_since_plant_[i] += 1;
        in_swing_[i] = sw ? 1 : 0;
    }
}

void MotorEPM::on_param_change(std::string_view key, ParamValue const& value) {
    if (key == "model_lr")  model_lr_  = get_double(value, "model_lr");
    else if (key == "ctrl_lr")   ctrl_lr_   = get_double(value, "ctrl_lr");
    else if (key == "bias_lr")   bias_lr_   = get_double(value, "bias_lr");
    else if (key == "reg_eps")   reg_eps_   = get_double(value, "reg_eps");
    else if (key == "max_dctrl") max_dctrl_ = get_double(value, "max_dctrl");
    else if (key == "babble_scale") babble_scale_ = get_double(value, "babble_scale");
    else if (key == "sat_lr")       sat_lr_       = get_double(value, "sat_lr");
    else if (key == "postural_gain") postural_gain_ = get_double(value, "postural_gain");
    else if (key == "explore_noise") explore_noise_ = get_double(value, "explore_noise");
    else if (key == "knee_tuck_target") knee_tuck_target_ = get_double(value, "knee_tuck_target");
    else if (key == "motor_gain") motor_gain_ = get_double(value, "motor_gain");
    else if (key == "coupling_gain") coupling_gain_ = get_double(value, "coupling_gain");
    else if (key == "coord_adapt_rate") coord_adapt_rate_ = get_double(value, "coord_adapt_rate");
    else if (key == "coord_explore") coord_explore_ = get_double(value, "coord_explore");
    else if (key == "coord_reward_drive") coord_reward_drive_ = get_double(value, "coord_reward_drive");
    else if (key == "coord_probe_ticks") coord_probe_ticks_ = get_int(value, "coord_probe_ticks");
    else if (key == "coord_stab_penalty") coord_stab_penalty_ = get_double(value, "coord_stab_penalty");
    else if (key == "coord_lat_penalty") coord_lat_penalty_ = get_double(value, "coord_lat_penalty");
    else if (key == "coord_intent_nav") coord_intent_nav_ = get_double(value, "coord_intent_nav");
    else if (key == "cruse_gain") cruse_gain_ = get_double(value, "cruse_gain");
    else if (key == "cruse_rule3_weight") cruse_rule3_weight_ = get_double(value, "cruse_rule3_weight");
    else if (key == "cruse_rule2_window") cruse_rule2_window_ = get_int(value, "cruse_rule2_window");
    else if (key == "cruse_rule5_gain") cruse_rule5_gain_ = get_double(value, "cruse_rule5_gain");
    else if (key == "gait_phase") {
        auto gp = get_double_vec(value, "gait_phase");
        if (int(gp.size()) == n_legs_) gait_phase_ = gp;
    }
    else if (key == "stroke_gain") stroke_gain_ = get_double(value, "stroke_gain");
    else if (key == "stroke_phase") stroke_phase_ = get_double(value, "stroke_phase");
    else if (key == "steer") steer_ = get_double(value, "steer");
    else if (key == "stroke_signs") {
        auto ss = get_double_vec(value, "stroke_signs");
        if (int(ss.size()) == n_legs_) stroke_signs_ = ss;
    }
    else if (key == "balance_gain") balance_gain_ = get_double(value, "balance_gain");
    else if (key == "amp_homeo_gain") amp_homeo_gain_ = get_double(value, "amp_homeo_gain");
    else if (key == "amp_target") amp_target_ = get_double(value, "amp_target");
    else if (key == "amp_seek_rate") amp_seek_rate_ = get_double(value, "amp_seek_rate");
    else if (key == "amp_seek_ticks") amp_seek_ticks_ = get_int(value, "amp_seek_ticks");
    else if (key == "heading_gain") heading_gain_ = get_double(value, "heading_gain");
    else if (key == "nav_gain") nav_gain_ = get_double(value, "nav_gain");
    else if (key == "cog_steer_gain") cog_steer_gain_ = get_double(value, "cog_steer_gain");
    else if (key == "cog_thrust_gain") cog_thrust_gain_ = get_double(value, "cog_thrust_gain");
    else if (key == "boredom_noise_gain") boredom_noise_gain_ = get_double(value, "boredom_noise_gain");
    else if (key == "boredom_escalation_rate") boredom_escalation_rate_ = get_double(value, "boredom_escalation_rate");
    else if (key == "height_homeo_gain") height_homeo_gain_ = get_double(value, "height_homeo_gain");
    else if (key == "height_k") height_k_ = get_double(value, "height_k");
    else if (key == "panic_on") panic_on_ = get_double(value, "panic_on");
    else if (key == "panic_off") panic_off_ = get_double(value, "panic_off");
    else if (key == "panic_strength") panic_strength_ = get_double(value, "panic_strength");
    else if (key == "panic_noise") panic_noise_ = get_double(value, "panic_noise");
    else if (key == "panic_motor_mult") panic_motor_mult_ = get_double(value, "panic_motor_mult");
    else if (key == "panic_push_amp") panic_push_amp_ = get_double(value, "panic_push_amp");
    else if (key == "panic_push_hz") panic_push_hz_ = get_double(value, "panic_push_hz");
}

void MotorEPM::ensure_leg_init(int leg, int n) {
    Leg& L = legs_[leg];
    if (L.initialized) return;
    int m = motor_dim_;
    std::mt19937 rng(static_cast<uint32_t>(base_seed_ ^ (0x9E3779B9u + uint32_t(leg))));
    std::normal_distribution<float> nd(0.0f, 1.0f);
    L.n = n;
    L.A = Eigen::MatrixXf::Zero(n, m);
    L.b = Eigen::VectorXf::Zero(n);
    L.C = Eigen::MatrixXf::Zero(m, n);
    L.h = Eigen::VectorXf::Zero(m);
    // A: small random motor→sensor (model learns the real coupling quickly).
    // C: small random sensor→motor so the initial command y≈0 → body holds the
    //    export-default (standing) pose; HK then destabilizes it.
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            L.A(i, j) = float(init_scale_) * nd(rng);
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j)
            L.C(i, j) = float(init_scale_) * nd(rng);
    L.x      = Eigen::VectorXf::Zero(n);
    L.prev_x = Eigen::VectorXf::Zero(n);
    L.prev_y = Eigen::VectorXf::Zero(m);
    L.have_prev  = false;
    L.steps_seen = 0;
    L.babble_rng.seed(static_cast<uint32_t>(base_seed_ ^ (0x2545F491u + uint32_t(leg))));
    L.initialized = true;
}

void MotorEPM::handle_proprio(int leg, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() == 0) return;
    int n = int(pt->values.size());
    ensure_leg_init(leg, n);
    Leg& L = legs_[leg];
    if (L.n != n) return;            // dimensionality must be stable
    L.x = pt->values;
    // Capture the spawn pose (first frame = body standing) as the postural rest
    // target — pos=0 is NOT standing (knee rests near pos −0.27).  State layout
    // is [pos,act,delta] per joint → joint j position at index 3j.
    if (!L.rest_captured && n >= 3 * motor_dim_) {
        L.rest_pos = Eigen::VectorXf::Zero(motor_dim_);
        for (int j = 0; j < motor_dim_; ++j) L.rest_pos[j] = L.x[3 * j];
        // Spider stance: override the knee (last joint) rest with the tuck target
        // so postural tone drives the body into the statically-stable spider pose.
        if (knee_tuck_target_ > -90.0)
            L.rest_pos[motor_dim_ - 1] = float(knee_tuck_target_);
        L.rest_captured = true;
    }
    L.fresh = true;
}

void MotorEPM::tick(uint64_t tick_id) {
    int m = motor_dim_;

    // ---- Gate 0 reset-masking bookkeeping (reward-free instrumentation) ----
    // Slow EMA of the per-tick disruption indicator (set by handle_event when a
    // miss/reset arrived this tick): rises while the body keeps falling/respawning,
    // decays toward 0 as the upright prior holds the bout.  ticks_since_reset_
    // counts up between disruptions so consumers can mask the post-reset transient.
    {
        const float hit = reset_hit_this_tick_ ? 1.0f : 0.0f;
        if (!reset_rate_init_) { reset_rate_ema_ = hit; reset_rate_init_ = true; }
        else reset_rate_ema_ += kResetRateAlpha * (hit - reset_rate_ema_);
        if (!reset_hit_this_tick_) ++ticks_since_reset_;   // stays 0 on the reset tick itself
        reset_hit_this_tick_ = false;
    }

    // ---- Chassis-height homeostat (module-level, body-wide signal) ----
    // Slow integral drives a tuck-deepening knee bias toward height_k × the
    // tallest chassis height the body has discovered.  The body finds its own
    // ceiling (no hand-set height); the bias fights the G6DOF spring sag that the
    // joint-angle postural reflex cannot see.  Applied per-leg below the loop.
    if (height_homeo_gain_ > 0.0 && chassis_h_max_ > 1e-4f) {
        float tgt = float(height_k_) * chassis_h_max_;
        height_bias_ += float(height_homeo_gain_) * (tgt - chassis_h_ema_);
        height_bias_ = std::clamp(height_bias_, kHeightBiasMin, kHeightBiasMax);
    }

    // ---- Panic pathway (Stage 2): smooth hysteresis switch from distress ----
    // Above panic_on the gait is overridden by decoupled flailing (escape a wedge);
    // below panic_off it resumes.  panic_ ramps smoothly so the transition isn't a
    // jerk.  pe ∈ [0,1] is the effective override applied to the gait knobs below.
    if (panic_strength_ > 0.0) {
        if (!panic_latched_ && distress_ > float(panic_on_))  panic_latched_ = true;
        else if (panic_latched_ && distress_ < float(panic_off_)) panic_latched_ = false;
    } else {
        panic_latched_ = false;
    }
    panic_ += kPanicRampAlpha * ((panic_latched_ ? 1.0f : 0.0f) - panic_);
    panic_ = std::clamp(panic_, 0.0f, 1.0f);
    const float pe = panic_ * float(panic_strength_);   // effective panic override
    // Push-reflex oscillator — advance a LOW-frequency phase (survives the servo /
    // spring low-pass that turns per-tick noise into mere jitter).
    if (pe > 0.001f) {
        panic_phase_ += 2.0f * float(M_PI) * float(panic_push_hz_) / 60.0f;   // ~60 Hz tick
        if (panic_phase_ > 2.0f * float(M_PI)) panic_phase_ -= 2.0f * float(M_PI);
    }

    // ---- PRE-PASS: per-leg oscillator phase (needed by ALL legs before the
    // coupling can be computed for any one).  Phase from the knee proprio:
    // φ = atan2(knee_Δ · scale, knee_pos − knee_ema), advancing through the cycle.
    if (coupling_gain_ > 0.0 || stroke_gain_ > 0.0 || steer_ != 0.0
        || amp_homeo_gain_ > 0.0 || amp_seek_rate_ > 0.0 || heading_gain_ != 0.0 || nav_gain_ != 0.0) {
        float amp_sum = 0.0f; int amp_n = 0;
        for (int leg = 0; leg < n_legs_; ++leg) {
            Leg& L = legs_[leg];
            if (!L.initialized || L.n < 3 * m) continue;
            float kp = L.x[3 * (m - 1)];          // knee position
            float kd = L.x[3 * (m - 1) + 2];      // knee delta (velocity proxy)
            L.knee_ema = (1.0f - kKneeEmaAlpha) * L.knee_ema + kKneeEmaAlpha * kp;
            float vx = kp - L.knee_ema, vy = kd * kPhaseVelScale;
            L.phase = std::atan2(vy, vx);
            // amplitude homeostat: phase-vector magnitude = oscillation amplitude.
            // Slow integral regulator drives amp_gain so amp_ema → amp_target.
            if (amp_homeo_gain_ > 0.0 || amp_seek_rate_ > 0.0) {
                float amp = std::sqrt(vx * vx + vy * vy);
                amp_sum += amp; ++amp_n;          // for the CoT amplitude search
                if (amp_homeo_gain_ > 0.0) {
                    L.amp_ema = (1.0f - kAmpEmaAlpha) * L.amp_ema + kAmpEmaAlpha * amp;
                    L.amp_gain += float(amp_homeo_gain_) * (float(amp_target_) - L.amp_ema);
                    L.amp_gain = std::clamp(L.amp_gain, kAmpGainMin, kAmpGainMax);
                }
            }
        }
        if (amp_n > 0) cur_amp_ = amp_sum / float(amp_n);
    }

    // ---- Adaptive coordination: crystallise the gait_phase offsets toward the
    // body's OWN emergent phase pattern (relative to leg 0) + a persistent probe.
    // Stops imposing the trot the body fights; the coordination self-organises and
    // never fully freezes.  Reward-free.  Slow (rate ≪ gait frequency) so the
    // Kuramoto coupling still enforces the (lagging) targets as a restoring force.
    if (coord_adapt_rate_ > 0.0 && int(gait_phase_.size()) == n_legs_
        && n_legs_ > 1 && legs_[0].initialized) {
        const float ref = legs_[0].phase;
        std::normal_distribution<float> nz(0.0f, 1.0f);
        for (int i = 1; i < n_legs_; ++i) {
            if (!legs_[i].initialized) continue;
            float emergent = legs_[i].phase - ref;            // measured offset rel. leg 0
            float d = emergent - float(gait_phase_[i]);
            while (d >  float(M_PI)) d -= 2.0f * float(M_PI);  // shortest-path wrap
            while (d < -float(M_PI)) d += 2.0f * float(M_PI);
            float p = float(gait_phase_[i]) + float(coord_adapt_rate_) * d
                    + float(coord_explore_) * nz(coord_rng_);
            while (p >  float(M_PI)) p -= 2.0f * float(M_PI);
            while (p < -float(M_PI)) p += 2.0f * float(M_PI);
            gait_phase_[i] = p;
        }
        gait_phase_[0] = 0.0;   // leg 0 is the phase reference
    }

    // ---- Agency-reward coordination search (intent→action gradient): (1+1)
    // hill-climb on the phase offsets, keeping probes that raise CONTROLLABILITY
    // (forward thrust fwd_v — coordinated propulsion on flat ground, Goodhart-
    // robust: a frozen body scores 0, a drift can't happen).  Fitness over the
    // back half of each window (gait settles to the new offsets first).  A slow
    // decay on the best lets it track drift + escape local optima.
    if (coord_reward_drive_ > 0.0 && int(gait_phase_.size()) == n_legs_ && n_legs_ > 1) {
        if (!coord_best_init_) {                 // seed incumbent from the current offsets
            coord_best_phase_ = gait_phase_; coord_best_init_ = true;
        }
        coord_probe_counter_++;
        if (coord_probe_counter_ > coord_probe_ticks_ / 2) {   // measure back half
            // controllability (forward thrust) minus an edge-of-chaos penalty on
            // tilt magnitude — a probe that goes faster by going wobbly is rejected,
            // keeping the climb on the alive (predictable) side of the frontier —
            // and minus an anti-crab penalty on |lateral velocity| so the search
            // rejects phase patterns that thrust forward by fishtailing sideways
            // (straighter gait = higher transport efficiency).
            float wob = std::sqrt(tilt_pitch_ * tilt_pitch_ + tilt_roll_ * tilt_roll_);
            // Reward signal: forward velocity (legacy) OR — when coord_intent_nav is on
            // and a target bearing exists — velocity TOWARD the intended direction
            // (target_compass · body velocity), so going forward and turning-toward-goal
            // are rewarded symmetrically.  body velocity = (lateral_v, fwd_v); intent =
            // (tc_x, tc_y) is already a body-frame unit vector toward the target.
            float reward_v = fwd_v_;
            if (coord_intent_nav_ > 0.0) {
                float tc_mag = std::sqrt(tc_x_ * tc_x_ + tc_y_ * tc_y_);
                if (tc_mag > 0.1f)                                   // target present
                    reward_v = lateral_v_ * tc_x_ + fwd_v_ * tc_y_;  // velocity toward intent
            }
            coord_fit_accum_ += reward_v - float(coord_stab_penalty_) * wob
                                         - float(coord_lat_penalty_) * std::fabs(lateral_v_);
            coord_fit_count_++;
        }
        if (coord_probe_counter_ >= coord_probe_ticks_) {
            float fit = (coord_fit_count_ > 0) ? coord_fit_accum_ / float(coord_fit_count_) : 0.0f;
            if (fit > coord_best_fitness_) {     // probe won → adopt it
                coord_best_phase_ = gait_phase_; coord_best_fitness_ = fit;
            } else {                              // probe lost → revert
                gait_phase_ = coord_best_phase_;
            }
            coord_best_fitness_ *= 0.99f;        // slow forget (track drift / escape optima)
            std::normal_distribution<float> pz(0.0f, float(coord_reward_drive_));
            for (int i = 1; i < n_legs_; ++i) {   // propose a new probe around the incumbent
                float p = float(coord_best_phase_[i]) + pz(coord_rng_);
                while (p >  float(M_PI)) p -= 2.0f * float(M_PI);
                while (p < -float(M_PI)) p += 2.0f * float(M_PI);
                gait_phase_[i] = p;
            }
            gait_phase_[0] = 0.0;
            coord_probe_counter_ = 0; coord_fit_accum_ = 0.0f; coord_fit_count_ = 0;
        }
    }

    // ---- CoT-seeking amplitude search: (1+1) hill-climb on amp_target maximising
    // fwd_v / oscillation-amplitude (speed per effort = inverse cost of transport).
    // Lowers the gait amplitude until forward thrust starts to fall → the efficient
    // amplitude.  Reward-free; attacks the DOMINANT motor-effort term (leg-cycling
    // amplitude) the phase search structurally cannot.  Needs amp_homeo on (amp_target
    // is what the homeostat regulates toward).
    if (amp_seek_rate_ > 0.0) {
        if (!amp_seek_init_) { amp_seek_best_target_ = amp_target_; amp_seek_init_ = true; }
        amp_seek_counter_++;
        if (amp_seek_counter_ > amp_seek_ticks_ / 2) {   // back half (homeostat settled to new amp_target)
            amp_seek_fwd_accum_ += fwd_v_;
            amp_seek_amp_accum_ += cur_amp_;
            amp_seek_count_++;
        }
        if (amp_seek_counter_ >= amp_seek_ticks_) {
            float mf = (amp_seek_count_ > 0) ? amp_seek_fwd_accum_ / float(amp_seek_count_) : 0.0f;
            float ma = (amp_seek_count_ > 0) ? amp_seek_amp_accum_ / float(amp_seek_count_) : 1e-3f;
            float fit = mf / (ma + 1e-3f);       // speed per amplitude = inverse cost of transport
            if (fit > amp_seek_best_fitness_) {  // probe more efficient → adopt
                amp_seek_best_target_ = amp_target_; amp_seek_best_fitness_ = fit;
            } else {                              // probe less efficient → revert
                amp_target_ = amp_seek_best_target_;
            }
            amp_seek_best_fitness_ *= 0.99f;     // slow forget (track drift / escape optima)
            std::normal_distribution<float> az(0.0f, float(amp_seek_rate_));
            double p = amp_seek_best_target_ + double(az(coord_rng_));
            amp_target_ = std::clamp(p, double(kAmpSeekMin), double(kAmpSeekMax));
            amp_seek_counter_ = 0; amp_seek_fwd_accum_ = 0.0f; amp_seek_amp_accum_ = 0.0f; amp_seek_count_ = 0;
        }
    }

    // Cruse stance/swing bookkeeping (once per tick, before the per-leg output).
    if (cruse_gain_ > 0.0 || cruse_rule5_gain_ > 0.0) update_cruse_state();

    for (int leg = 0; leg < n_legs_; ++leg) {
        Leg& L = legs_[leg];
        if (!L.initialized || !L.fresh) continue;
        L.fresh = false;
        L.steps_seen += 1;
        int n = L.n;
        bool warmup = (L.steps_seen <= babble_ticks_);

        // ---- Learn from the previous command's outcome (motor TLE) ----
        // The MODEL always learns (also during babble warmup, where it learns the
        // body's response to small random commands).  The CONTROLLER (HK + anti-
        // saturation) only learns after warmup, once the model can predict.
        if (L.have_prev) {
            Eigen::VectorXf x_hat = L.A * L.prev_y + L.b;          // forward-model prediction
            Eigen::VectorXf xi    = L.x - x_hat;                   // motor TLE ξ
            // (1) model descent: A += η_M ξ yᵀ ; b += η_M ξ
            L.A.noalias() += float(model_lr_) * xi * L.prev_y.transpose();
            L.b.noalias() += float(model_lr_) * xi;
            L.tle_ema = (1.0f - kTeleEmaAlpha) * L.tle_ema + kTeleEmaAlpha * xi.norm();

            if (!warmup) {
                Eigen::VectorXf z = L.C * L.prev_x + L.h;          // operating point
                Eigen::MatrixXf G = Eigen::MatrixXf::Zero(m, m);
                float sat = 0.0f;
                for (int i = 0; i < m; ++i) {
                    float t = std::tanh(z[i]);
                    G(i, i) = 1.0f - t * t;                        // g'(z)
                    sat += t * t;                                  // saturation indicator
                }
                // (2) controller homeokinetic update — descend E = ξᵀ(LLᵀ+εI)⁻¹ξ,
                //     L = A·G·C.  Holding G and ξ fixed in the metric gradient
                //     (Der–Martius approximation): ∂E/∂C = −2(AG)ᵀ q qᵀL,
                //     q = (LLᵀ+εI)⁻¹ ξ → descent C += 2η_K (AG)ᵀ q (qᵀL).
                Eigen::MatrixXf AG = L.A * G;                      // n x m
                Eigen::MatrixXf Lp = AG * L.C;                     // loop Jacobian
                Eigen::MatrixXf P  = (Lp * Lp.transpose()
                                      + float(reg_eps_) * Eigen::MatrixXf::Identity(n, n)).inverse();
                Eigen::VectorXf q  = P * xi;
                Eigen::MatrixXf dC = 2.0f * float(ctrl_lr_)
                                     * (AG.transpose() * q) * (q.transpose() * Lp);
                float dC_norm = dC.norm();
                if (max_dctrl_ > 0.0 && dC_norm > float(max_dctrl_))
                    dC *= float(max_dctrl_) / dC_norm;             // ignition clamp
                L.C.noalias() += dC;
                Eigen::VectorXf mu = G * (L.A.transpose() * q);    // bias toward less surprise
                L.h.noalias() += float(bias_lr_) * mu;

                // (3) anti-saturation — surrogate for the dropped ∂G term.  Penalty
                //     ∝ zᵢ·tanh²(zᵢ): inert in the linear band, strong at the rails,
                //     so the controller can commit but cannot run to g'=0 and freeze.
                if (sat_lr_ > 0.0) {
                    for (int i = 0; i < m; ++i) {
                        float ti = std::tanh(z[i]);
                        float gsat = z[i] * ti * ti;               // ∝ saturation push-back
                        L.C.row(i).noalias() -= float(sat_lr_) * gsat * L.prev_x.transpose();
                        L.h[i] -= float(sat_lr_) * gsat;
                    }
                }
                L.gain_ema = (1.0f - kTeleEmaAlpha) * L.gain_ema + kTeleEmaAlpha * Lp.norm();
                L.sat_ema  = (1.0f - kTeleEmaAlpha) * L.sat_ema  + kTeleEmaAlpha * (sat / float(m));
            }
        }

        // ---- Emit the new command ----
        Eigen::VectorXf y(m);
        if (warmup) {
            // motor babble: small random commands so the model learns A,B,b before
            // the controller rides the loop.
            std::uniform_real_distribution<float> ud(-float(babble_scale_), float(babble_scale_));
            for (int j = 0; j < m; ++j) y[j] = std::clamp(ud(L.babble_rng), -1.0f, 1.0f);
        } else {
            // per-leg homeostat gain scales the HK oscillation toward amp_target
            float ag = (amp_homeo_gain_ > 0.0) ? L.amp_gain : 1.0f;
            // Panic boosts motor_gain for stronger escape thrust.
            float mg = float(motor_gain_) * (1.0f + pe * (float(panic_motor_mult_) - 1.0f));
            y = L.C * L.x + L.h;
            for (int j = 0; j < m; ++j) y[j] = mg * ag * std::tanh(y[j]);
        }
        // Postural reflex (spinal-tone analog): pull hip2+knee toward the standing
        // REST pose (proprio pos=0).  State layout is [pos,act,delta] per joint, so
        // joint j's position is x[3j].  Applied to motors j>=1 (skip hip1=yaw, which
        // the body sign-flips per leg).  Composes under HK, then clamp once.
        // hip1 included: the body's HIP1_SPLAY_OUT_SIGN is uniform (+1,+1,+1,+1),
        // so the command→pos sign is consistent across legs — no per-leg flip.
        // Holding all three joints near the spawn stance keeps the legs inside
        // their mechanical-advantage envelope (the knees were driving to max
        // hyperextension and splaying the body flat).
        if (postural_gain_ > 0.0 && n >= 3 * m && L.rest_captured) {
            for (int j = 0; j < m; ++j)
                y[j] -= float(postural_gain_) * (L.x[3 * j] - L.rest_pos[j]);
        }
        // Chassis-height homeostat output: drive the femur-LIFT joint (hip2, index
        // 1) — NOT the knee.  The knee carries the stepping rhythm (HK + coupling);
        // a DC bias there clamps the swing and kills the gait (the body fought the
        // dead spider pose).  hip2 angles the upper leg down → with feet planted the
        // hip (chassis) rises — height set on a joint SEPARATE from the rhythm.
        // Sign empirically confirmed (positive height_bias raises chassis_y).
        // Post-warmup so motor babble can explore upward and discover the ceiling.
        if (!warmup && height_homeo_gain_ > 0.0 && m >= 2)
            y[1] += kHeightLiftSign * height_bias_;
        // Cruse/Walknet inter-leg coordination — v2 SEQUENCED LIFT.  Drives hip2 AND
        // knee (they share foot-height authority; either alone is too weak — measured
        // corr(foot_y,joint)≈0.27) to actually plant/clear the foot, not just DC-bias.
        // +cmd = plant firmly (foot down), −cmd = lift CLEAR (foot up): a leg held by a
        // neighbour plants to keep the support polygon; an unheld leg in its own swing
        // lifts clear so its hip1 forward stroke is a free swing, NOT a loaded pull
        // (the operator's swing-becomes-pull → yaw observation).
        if (!warmup && cruse_gain_ > 0.0 && m >= 2 && int(in_swing_.size()) == n_legs_) {
            int ant = cruse_anterior_[leg], con = cruse_contra_[leg];
            float hold = 0.0f;                                                // Rule 1 + Rule 3
            if (ant >= 0 && in_swing_[ant]) hold += 1.0f;                     // Rule 1: anterior swinging
            if (con >= 0 && in_swing_[con]) hold += float(cruse_rule3_weight_);// Rule 3: contralateral load
            bool released = (ant >= 0 && !in_swing_[ant]                      // Rule 2: anterior just planted
                             && ticks_since_plant_[ant] < cruse_rule2_window_);
            float cmd;
            if (hold > 0.0f)                     cmd =  hold;   // hold stance (neighbours win)
            else if (in_swing_[leg] || released) cmd = -1.0f;   // free to swing → lift clear
            else                                 cmd =  0.0f;   // unconstrained stance
            y[1]     += float(cruse_gain_) * cmd;    // hip2: +foot down (plant), −foot up (lift)
            y[m - 1] += -float(cruse_gain_) * cmd;   // knee reinforces the same foot-height move
        }
        // Rule 5 (load distribution) — independent of the v2 lift so it can be tested
        // alone.  A STANCE leg presses its foot down ∝ how many OTHER legs are swinging
        // (taking up their shed weight) → more normal force → more friction → less scrub.
        if (!warmup && cruse_rule5_gain_ > 0.0 && m >= 2 && int(in_swing_.size()) == n_legs_
            && !in_swing_[leg]) {
            int n_sw = 0;
            for (int j = 0; j < n_legs_; ++j) if (j != leg && in_swing_[j]) ++n_sw;
            float load = float(cruse_rule5_gain_) * float(n_sw);
            y[1]     += load;    // +hip2 = press foot down (load for grip)
            y[m - 1] += -load;   // knee extends down with it
        }
        // Persistent exploration noise (post-warmup): keeps ξ alive at fixed
        // points so HK amplifies it into oscillation instead of freezing.
        // Panic adds exploration noise on top of the persistent drive (flailing).
        float noise_sigma = float(explore_noise_) + pe * float(panic_noise_);
        if (!warmup && noise_sigma > 0.0f) {
            std::normal_distribution<float> nz(0.0f, noise_sigma);
            for (int j = 0; j < m; ++j) y[j] += nz(L.babble_rng);
        }
        // Rung 3 inter-leg coupling (post-warmup): Kuramoto bias injected into the
        // knee (the propulsive joint; HK's within-leg coordination carries the rest).
        // c_i = K · mean_{j≠i} sin( (φ_j − φ_i) − (P_j − P_i) ) pulls each leg's
        // phase toward the gait offset relative to every other leg — entrains the
        // twitchers and phase-locks all four.
        if (!warmup && coupling_gain_ > 0.0 && n_legs_ > 1 && int(gait_phase_.size()) == n_legs_) {
            float c = 0.0f;
            for (int j = 0; j < n_legs_; ++j) {
                if (j == leg || !legs_[j].initialized) continue;
                float dphi = (legs_[j].phase - L.phase)
                           - (float(gait_phase_[j]) - float(gait_phase_[leg]));
                c += std::sin(dphi);
            }
            // Panic decouples the legs (break out of the stuck phase-lock).
            y[m - 1] += (1.0f - pe) * float(coupling_gain_) * c / float(n_legs_ - 1);
        }
        // Directional propulsion drive on hip1 (joint 0), phase-locked to the
        // leg's step phase.  stroke_signs sets the per-leg push direction
        // (parallel → forward, tangential → spin); steer is a left/right
        // skid-steer differential (FL,RL = left = +1; FR,RR = right = −1).
        if (!warmup && (stroke_gain_ > 0.0 || steer_ != 0.0 || heading_gain_ != 0.0 || nav_gain_ != 0.0) && m >= 1) {
            float side = (leg == 0 || leg == 2) ? 1.0f : -1.0f;   // left vs right
            float sgn  = (int(stroke_signs_.size()) == n_legs_) ? float(stroke_signs_[leg]) : 1.0f;
            // Steering = manual command + perception (steer toward target,
            // minimizing the bearing) − heading-rate damping (smooth, no spin).
            // The active-inference loop: act to drive the perceived target bearing
            // to zero, with yaw-rate damping for a clean turn.  Steer on the full
            // bearing ANGLE atan2(tc_x,tc_y) (normalized [−1,1]; 0 = dead-ahead,
            // ±1 = directly behind), NOT tc_x alone: tc_x→0 has TWO solutions
            // (target ahead AND behind), and the robot was locking onto BEHIND then
            // walking away (tc_y≈−0.95).  The angle makes "behind" maximally
            // unstable → the body turns around to face the target.
            constexpr float kInvPi = 0.318309886f;
            bool nav_on = (std::fabs(tc_x_) + std::fabs(tc_y_) > 0.05f) && nav_gain_ != 0.0;
            float bearing = nav_on ? std::atan2(tc_x_, tc_y_) * kInvPi : 0.0f;
            // Heading-rate damper is the "go-straight" reflex for UNguided walking;
            // when a target is active it fights the bearing steering and turns the
            // approach into an orbit (heading_gain=−2 stalled the bearing at ~50°).
            // Gate it off during nav so the bearing controller has full authority.
            float head_term = nav_on ? 0.0f : float(heading_gain_) * yaw_rate_;
            float steer_eff = float(steer_) + float(nav_gain_) * bearing - head_term;
            // Facing-gate on the FORWARD thrust (only when a target is active):
            // walk toward what you face.  fwd ∝ tc_y (forward bearing component):
            // target ahead (tc_y→1) = full thrust; target to the side/behind = turn
            // in place, little advance.  Without this the body orbits — it overshoots
            // at full speed, the target slides to its flank, and it circles instead
            // of contacting.  Gating advance to alignment tightens the spiral into a
            // hit.  A small floor keeps it from freezing (no new stall).  steer_eff
            // is NOT gated — turning stays authoritative when off-target.
            float fwd = nav_on ? std::clamp(0.25f + 0.75f * tc_y_, 0.0f, 1.0f) : 1.0f;
            // Skid-steer: the steer differential modulates each leg's stroke
            // MAGNITUDE per side (inside sgn), so left/right thrust DIFFERS → the
            // body yaws.  (Was `sgn*stroke + side*steer`: since stroke_signs == the
            // side pattern, that degenerated to sgn*(stroke+steer) — a COMMON-MODE
            // thrust scale that can't turn and STALLS when steer≈−stroke, pinning
            // the bearing at the cancellation point.  Folding side*steer into the
            // magnitude restores a real differential that never zeroes both sides.)
            // Panic kills the directional drive (it was futilely pushing into the
            // obstacle) — let the boosted, noisy, decoupled HK output flail instead.
            float amp  = sgn * (float(stroke_gain_) * fwd + side * steer_eff);
            y[0] += (1.0f - pe) * amp * std::sin(L.phase + float(stroke_phase_));
        }
        // Cell differential nav-steer (n_legs=1, motor_dim=2): a perceived goal
        // bearing biases the two flagella differentially → the alive homeokinetic
        // swimmer turns toward the goal.  Reward-free active-inference modulation
        // from the slow-perception path; steer toward bearing=0 (dead-ahead).
        // The leg-stroke steering above is inert for the cell (stroke_gain=0); this
        // is the clean 2-output channel.  bearing>0 (goal to the right) → left
        // flagellum beats more (al>ar → CW/right turn).  Sign tunable via nav_gain.
        if (!warmup && nav_gain_ != 0.0 && n_legs_ == 1 && m == 2) {
            constexpr float kInvPiCell = 0.318309886f;
            // The cell compass arrives RAW: magnitude = gradient strength
            // (confidence).  Steer hard when the gradient is strong (near food)
            // and not at all when weak (far / flat) — so the alive swimmer drifts
            // and explores by default and only HOMES when food is genuinely
            // sensed.  conf = clamp(|grad|·k) gives the gentle-far / strong-near
            // profile the angle-only steer lacked.
            float mag = std::sqrt(tc_x_ * tc_x_ + tc_y_ * tc_y_);
            // FORAGE drive: scale the food-ward pull by HUNGER (1-energy).  A sated
            // bug ignores food and keeps exploring (forage→0 = no pull, no linger);
            // a starving bug homes hard (forage→1).  Without this the bug steered
            // toward food it didn't need and the critic never learned the
            // energy→approach contingency.  Empty hunger_topic = always forage
            // (legacy / chemotaxis-port behaviour).
            // Fade the WHOLE forage drive by (1-boredom): when the bug is pinned
            // and bored, the scent pull was NOT yielding (only cog_steer faded),
            // so forage kept driving it INTO the corner/wall toward food it can't
            // reach — a directed ATTRACTOR that fights the escape (operator saw the
            // bug "very interested + bored" pinned at a corner).  Releasing it lets
            // the boredom escape (tumble + desperation dash) turn away and leave.
            // 2026-06-17 — BOREDOM→NAV-OFF TRAP FIX.  Previously the steer was
            // gated by forage = hunger × (1-boredom).  Under reflex_modular (the
            // brain-routed cognitive regime) the MotorEPM scent steer is the ONLY
            // homing, so once the bug stuck → boredom→1 → steer→0 → it could never
            // pull off the wall (self-locking; reproducible headless via
            // OGMA_REFLEX_MODULAR=1, both paddlers, 0 hits — scent bearing present
            // the whole time but its motor effect zeroed by boredom).  Fix: gate
            // the directional STEER on HUNGER ONLY (a starving bug always homes,
            // even when bored/stuck), and keep the (1-boredom) factor on the
            // LINGER only (a bored/sated bug shouldn't loiter, but a starving one
            // must still steer toward food).
            float hunger_drive = (hunger_topic_.empty() ? 1.0f : hunger_);
            float forage = hunger_drive * (1.0f - boredom_);   // linger gate only
            if (mag > 0.02f && hunger_drive > 0.0f) {
                float bearing = std::atan2(tc_x_, tc_y_) * kInvPiCell;   // [-1,1]
                float conf = std::min(mag * 4.0f, 1.0f);
                // Slow the common-mode beat near food (high conf) so the fast
                // swimmer LINGERS and spirals in instead of overshooting; gated by
                // forage so a fed OR stuck bug doesn't loiter.
                float slow = 1.0f - 0.7f * conf * forage;
                y[0] *= slow;
                y[1] *= slow;
                // STEER on HUNGER only → starving bug always homes off the wall.
                float st = float(nav_gain_) * bearing * conf * hunger_drive;
                y[0] += st;
                y[1] -= st;
            }
        }
        // Cognitive steer (cell, n_legs=1/motor_dim=2): a learned scalar steer
        // (ActionDecoder's left/straight/right, normalized to [-1,1]) biases the
        // two flagella differentially — the slow critic directs the alive swimmer.
        // Reward-free in the sense of homeostatic active inference; gated to the
        // cell so the picrawler / Stage-1 path is untouched.
        if (!warmup && cog_steer_gain_ != 0.0 && n_legs_ == 1 && m == 2) {
            // Fade the cognitive steer when bored (frozen sensorimotor loop):
            // hand control to the playful substrate so the wall-driving command
            // can't keep pinning the bug.  Playful Machine arbitration (low
            // surprise → babble dominates; high surprise → brain acts).
            float cst = float(cog_steer_gain_) * cog_steer_ * (1.0f - boredom_);
            y[0] += cst;
            y[1] -= cst;
        }
        // Cognitive THRUST (cell, n_legs=1/motor_dim=2): a second learned scalar
        // drives COMMON-mode (forward/reverse/pause) on both flagella, so the actor
        // can move toward higher scent — the action that actually changes proximity
        // (its preferred observation).  Mirrors the steer's boredom fade.
        if (!warmup && cog_thrust_gain_ != 0.0 && n_legs_ == 1 && m == 2) {
            float ct = float(cog_thrust_gain_) * cog_thrust_ * (1.0f - boredom_);
            y[0] += ct;
            y[1] += ct;
        }
        // Boredom escape (cell, n_legs=1/motor_dim=2): when DistressDrive reports
        // a frozen/parked loop, RUN-AND-TUMBLE toward perceptual interest (③).
        // A held random turn is resampled each window so the heading SWEEPS;
        // interest gates it: high interest (open / scent-rich ahead) → RUN
        // (forward common-mode) into it; low interest (boring wall) → TUMBLE
        // (differential turn) to reorient.  So the bug tumbles through boring
        // headings and runs the instant it faces something interesting →
        // drifts toward open/scent-rich space.  NOT a wall-geometry reflex
        // (random sign, curiosity-driven); only keeps the bug alive + exploring
        // so the cognitive critic can learn.  interest_ = 0 (no topic) → pure
        // undirected tumble (legacy).  Playful Machine #1/#2.
        if (!warmup && boredom_noise_gain_ > 0.0 && boredom_ > 0.0f && n_legs_ == 1 && m == 2) {
            if (boredom_esc_ticks_ <= 0) {
                std::normal_distribution<float> nzb(0.0f, 1.0f);
                boredom_esc_held_  = nzb(L.babble_rng);
                boredom_esc_ticks_ = boredom_hold_ticks_;
            }
            --boredom_esc_ticks_;
            // Klinokinesis: RUN while interest is RISING (this heading is getting
            // more open/scent-rich → climb the gradient toward the interior),
            // TUMBLE when it's not (boring / facing a wall → reorient).  Keying
            // on the gradient (not the level) stops the bug running along a
            // "clear enough" wall and pulls it toward the MOST interesting
            // direction.  Mirrors the scent chemotaxis run-and-tumble.
            if (!interest_ema_init_) { interest_ema_ = interest_; interest_ema_init_ = true; }
            float rising = std::clamp((interest_ - interest_ema_) * 4.0f, 0.0f, 1.0f);
            interest_ema_ = 0.95f * interest_ema_ + 0.05f * interest_;   // ~20-tick baseline
            float drive = float(boredom_noise_gain_) * boredom_;
            // DESPERATION: the longer the loop stays frozen the harder the bug
            // tries (escalation ∝ boredom_streak), and HUNGER compounds it (a
            // starving bug escalates faster — a call to action).  Capped (~5×) so
            // it's a rising struggle, not a divergence; resets when freed.
            float escalation = std::min(1.0f + float(boredom_escalation_rate_) * float(boredom_streak_), 5.0f);
            float hunger_mult = hunger_topic_.empty() ? 1.0f : (1.0f + hunger_);
            drive *= escalation * hunger_mult;
            float turn  = drive * (1.0f - rising) * boredom_esc_held_;   // tumble (differential)
            // RUN forward (common-mode) when interest is rising (climb toward it).
            // BUT a bug wedged in a flat/boring corner has rising≈0 → it would only
            // TUMBLE (rotate in place) forever and never translate OUT (operator
            // saw exactly this: "oscillates back and forth, no forward/backward").
            // So the longer it stays stuck, force an escalating forward DASH floor
            // regardless of interest — a desperate bug powers out of the wedge, not
            // just spins.  Reaches a full forward run after ~1 s pinned (streak 60).
            float desp_floor = std::clamp(0.016f * float(boredom_streak_), 0.0f, 1.0f);
            float run   = drive * std::max(rising, desp_floor);          // run forward (common-mode)
            y[0] += run + turn;
            y[1] += run - turn;
        }
        // Active balance (vestibular reflex) on hip2 (the lift/pitch joint).  Push
        // the low-side legs to level the body: front/rear differential from pitch,
        // left/right differential from roll.  front=FL,FR(0,1); left=FL,RL(0,2).
        if (balance_gain_ != 0.0 && m >= 2) {
            float front = (leg == 0 || leg == 1) ? 1.0f : -1.0f;
            float left  = (leg == 0 || leg == 2) ? 1.0f : -1.0f;
            y[1] += float(balance_gain_) * (front * tilt_pitch_ + left * tilt_roll_);
        }
        // Panic PUSH reflex: a directed DOWNWARD thrust (against gravity).  A
        // SYMMETRIC oscillation just waved the legs sideways around the splayed
        // wedge pose (operator: "lands legs splayed, not legs down; push must be
        // against gravity").  Use a RECTIFIED pump (always ≥0) that biases hip2 to
        // push-down (+, the direction the height homeostat proved raises the
        // chassis) and the knee to EXTEND (−) toward the ground — so the legs drive
        // DOWN and lever the body up.  Per-leg STAGGERED → a churning struggle.
        if (pe > 0.001f && panic_push_amp_ > 0.0 && m >= 2) {
            float off  = float(leg) * (2.0f * float(M_PI) / float(n_legs_));
            float pump = 0.5f + 0.5f * std::sin(panic_phase_ + off);   // [0,1] — always toward LIFT
            float drive = pe * float(panic_push_amp_) * pump;
            // Both joints driven toward the CHASSIS-RAISING direction (the height
            // homeostat proved hip2+ lifts; the spider tuck knee+ suspends the body
            // higher).  knee− (extend) was UN-tucking and fighting the hip2 lift →
            // no lift (chassis_y barely moved).  Same sign = a coherent anti-gravity
            // push.  Per-leg staggered → asymmetric rock that dislodges.
            y[1]     += drive;    // hip2 + = push feet down / lift chassis
            y[m - 1] += drive;    // knee + = tuck (raises the chassis in spider stance)
        }
        for (int j = 0; j < m; ++j) y[j] = std::clamp(y[j], -1.0f, 1.0f);
        L.outmag_ema = (1.0f - kTeleEmaAlpha) * L.outmag_ema + kTeleEmaAlpha * float(y.norm());

        for (int j = 0; j < m; ++j) {
            auto out = std::make_shared<ActionOut>();
            out->tick_id     = tick_id;
            out->producer_id = id_.empty() ? std::string("motor_epm") : id_;
            out->accel       = y[j];                               // body reads accel as u∈[−1,1]
            out->source      = "motor_epm";
            bus_->publish(action_topics_[leg * m + j], out);
        }

        L.prev_x = L.x;
        L.prev_y = y;
        L.have_prev = true;
    }
}

int   MotorEPM::legs_initialized() const {
    int c = 0; for (auto const& L : legs_) if (L.initialized) ++c; return c;
}
float MotorEPM::gait_coherence() const {
    // Kuramoto order parameter on the gait-offset-corrected phases:
    // R = |mean_j e^{i(φ_j − P_j)}|.  1 = all legs locked to the gait pattern.
    float sx = 0.0f, sy = 0.0f; int c = 0;
    for (int j = 0; j < n_legs_; ++j) {
        if (!legs_[j].initialized) continue;
        float a = legs_[j].phase - (int(gait_phase_.size()) == n_legs_ ? float(gait_phase_[j]) : 0.0f);
        sx += std::cos(a); sy += std::sin(a); ++c;
    }
    if (c == 0) return 0.0f;
    return std::sqrt(sx * sx + sy * sy) / float(c);
}
float MotorEPM::tle_ema_mean() const {
    if (legs_.empty()) return 0.0f; float s = 0; int c = 0;
    for (auto const& L : legs_) if (L.initialized) { s += L.tle_ema; ++c; }
    return c ? s / float(c) : 0.0f;
}
float MotorEPM::gain_ema_mean() const {
    if (legs_.empty()) return 0.0f; float s = 0; int c = 0;
    for (auto const& L : legs_) if (L.initialized) { s += L.gain_ema; ++c; }
    return c ? s / float(c) : 0.0f;
}
float MotorEPM::outmag_ema_mean() const {
    if (legs_.empty()) return 0.0f; float s = 0; int c = 0;
    for (auto const& L : legs_) if (L.initialized) { s += L.outmag_ema; ++c; }
    return c ? s / float(c) : 0.0f;
}

// ---- snapshot / restore (determinism contract) ----
nlohmann::json MotorEPM::snapshot_state() const {
    nlohmann::json legs = nlohmann::json::array();
    for (auto const& L : legs_) {
        nlohmann::json lj;
        lj["initialized"] = L.initialized;
        lj["n"]           = L.n;
        lj["have_prev"]   = L.have_prev;
        lj["steps_seen"]  = L.steps_seen;
        lj["tle_ema"]     = L.tle_ema;
        lj["gain_ema"]    = L.gain_ema;
        lj["outmag_ema"]  = L.outmag_ema;
        lj["sat_ema"]     = L.sat_ema;
        lj["knee_ema"]    = L.knee_ema;
        lj["amp_ema"]     = L.amp_ema;
        lj["amp_gain"]    = L.amp_gain;
        lj["rest_captured"] = L.rest_captured;
        if (L.rest_captured)
            lj["rest_pos"] = std::vector<float>(L.rest_pos.data(), L.rest_pos.data() + L.rest_pos.size());
        { std::ostringstream os; os << L.babble_rng; lj["babble_rng"] = os.str(); }
        auto flat = [](Eigen::MatrixXf const& M){
            std::vector<float> v(M.data(), M.data() + M.size()); return v; };
        lj["A"] = flat(L.A); lj["b"] = flat(L.b);
        lj["C"] = flat(L.C); lj["h"] = flat(L.h);
        lj["prev_x"] = flat(L.prev_x); lj["prev_y"] = flat(L.prev_y);
        lj["rows_A"] = int(L.A.rows()); lj["cols_A"] = int(L.A.cols());
        legs.push_back(std::move(lj));
    }
    // --- module-level adaptive state (v2): carries across ticks but is NOT a config
    // knob and was previously dropped → a running agent did not reproduce after
    // restore (agency-fitness search progress, self-discovered height ceiling, and
    // panic phase were all silently reset).  Sensor caches (fwd_v_, tilt, tc_*, …)
    // are intentionally NOT serialized — like L.x they are repopulated by the next
    // bus message.  Cruse stance/swing EMAs are out of scope here (cruse_gain=0 in
    // the canonical tiers); revisit if a Cruse tier needs restore determinism.
    nlohmann::json mod;
    // coordination / agency-fitness (1+1) phase search
    { std::ostringstream os; os << coord_rng_; mod["coord_rng"] = os.str(); }
    mod["gait_phase"]          = gait_phase_;          // mutated by the search (live probe)
    mod["coord_best_phase"]    = coord_best_phase_;
    mod["coord_best_fitness"]  = coord_best_fitness_;
    mod["coord_best_init"]     = coord_best_init_;
    mod["coord_probe_counter"] = coord_probe_counter_;
    mod["coord_fit_accum"]     = coord_fit_accum_;
    mod["coord_fit_count"]     = coord_fit_count_;
    // CoT / amplitude (1+1) search
    mod["amp_target"]            = amp_target_;         // mutated by the search
    mod["amp_seek_best_target"]  = amp_seek_best_target_;
    mod["amp_seek_best_fitness"] = amp_seek_best_fitness_;
    mod["amp_seek_init"]         = amp_seek_init_;
    mod["amp_seek_counter"]      = amp_seek_counter_;
    mod["amp_seek_fwd_accum"]    = amp_seek_fwd_accum_;
    mod["amp_seek_amp_accum"]    = amp_seek_amp_accum_;
    mod["amp_seek_count"]        = amp_seek_count_;
    // chassis-height homeostat (self-discovered ceiling + integral output)
    mod["chassis_h_ema"]  = chassis_h_ema_;
    mod["chassis_h_max"]  = chassis_h_max_;
    mod["height_bias"]    = height_bias_;
    mod["chassis_h_seen"] = chassis_h_seen_;
    // panic pathway (push-oscillator phase + smoothed level + hysteresis latch)
    mod["panic_phase"]   = panic_phase_;
    mod["panic"]         = panic_;
    mod["panic_latched"] = panic_latched_;
    // boredom / curiosity escape (held turn + streak + interest baseline)
    mod["boredom_esc_held"]  = boredom_esc_held_;
    mod["boredom_esc_ticks"] = boredom_esc_ticks_;
    mod["boredom_streak"]    = boredom_streak_;
    mod["interest_ema"]      = interest_ema_;
    mod["interest_ema_init"] = interest_ema_init_;
    // Gate 0 reset-masking counters (reset_hit_this_tick_ is intra-tick → omitted)
    mod["reset_count"]       = reset_count_;
    mod["ticks_since_reset"] = ticks_since_reset_;
    mod["reset_rate_ema"]    = reset_rate_ema_;
    mod["reset_rate_init"]   = reset_rate_init_;
    return nlohmann::json{{"version", 2}, {"legs", legs}, {"module", mod}};
}

// Live viz (xaq_inspector MotorEPM widget): the homeokinetic self-model's health
// (motor-TLE = forward-model surprise, the one working predictive loop), the
// cognitive drive channels the brain injects (cog_steer differential, cog_thrust
// common-mode), the body's resulting forward velocity, the curiosity/hunger
// neuromodulators, and the leg-0 forward self-model A (motor→sensor) so the widget
// can draw a heatmap of what the body has learned to predict about its own motion.
nlohmann::json MotorEPM::diag_snapshot() const {
    nlohmann::json j;
    j["n_legs"]      = n_legs_;
    j["motor_dim"]   = motor_dim_;
    j["motor_tle"]   = tle_ema_mean();          // mean forward-model prediction error (self-model health)
    j["loop_gain"]   = gain_ema_mean();
    // Gate 0 (L-1a) reset-masked gait instruments:
    j["gait_coherence"]    = gait_coherence();  // Kuramoto phase-lock ∈[0,1]; rises as legs lock to the gait
    j["reset_count"]       = reset_count_;       // cumulative fall/teleport disruptions
    j["ticks_since_reset"] = ticks_since_reset_; // for reset-masking the coherence/TLE trend
    j["reset_rate"]        = reset_rate_ema_;    // Gate 0 signal: FALLS as the upright prior holds
    j["out_mag"]     = outmag_ema_mean();
    j["cog_steer"]   = cog_steer_;              // brain → differential (turn)
    j["cog_steer_msgs"] = cog_steer_msgs_;
    j["cog_thrust"]  = cog_thrust_;             // brain → common-mode (fwd/reverse)
    j["cog_thrust_msgs"] = cog_thrust_msgs_;
    j["fwd_v"]       = fwd_v_;                   // chassis forward velocity (controllability)
    j["lateral_v"]   = lateral_v_;
    j["boredom"]     = boredom_;                 // sensorimotor predictability (Playful Machine)
    j["boredom_streak"] = boredom_streak_;
    j["interest"]    = interest_;                // curiosity drive
    j["hunger"]      = hunger_;                  // 0 sated → 1 starving
    j["tc_x"]        = tc_x_;                    // food bearing the state SHOULD encode (lateral)
    j["tc_y"]        = tc_y_;                    // food bearing forward
    // Per-leg motor-TLE so a multi-leg body (picrawler) shows asymmetry; for the
    // cell this is a single flagella controller.
    nlohmann::json leg_tle = nlohmann::json::array();
    for (auto const& L : legs_) leg_tle.push_back(L.tle_ema);
    j["leg_tle"] = leg_tle;
    // Leg-0 forward self-model A (n×m, motor→sensor) for the heatmap, plus the last
    // motor command it conditioned on.  Empty until the leg lazily inits.
    if (!legs_.empty() && legs_[0].initialized) {
        Leg const& L = legs_[0];
        j["rows_A"] = int(L.A.rows());
        j["cols_A"] = int(L.A.cols());
        j["A"]      = std::vector<float>(L.A.data(), L.A.data() + L.A.size());
        j["prev_y"] = std::vector<float>(L.prev_y.data(), L.prev_y.data() + L.prev_y.size());
        // GNG-shaped payload so the EPM's PCA-scatter widget renders the self-model
        // geometrically: the trail = actual sensor observations x (the body's real
        // proprioceptive trajectory), and the one overlaid "node" = the model's
        // 1-step PREDICTION x̂ = A·prev_y + b for the current sensor.  The gap
        // between the latest-x cross and the prediction node IS the motor-TLE,
        // shown in the space the self-model actually lives in.  A frozen loop
        // collapses the trail to a point; a lively limit cycle traces a loop.
        nlohmann::json gng;
        if (L.x.size() > 0) {
            gng["last_x"] = std::vector<float>(L.x.data(), L.x.data() + L.x.size());
            if (L.A.cols() == L.prev_y.size() && L.A.rows() == L.b.size()) {
                Eigen::VectorXf xhat = L.A * L.prev_y + L.b;   // self-model prediction
                nlohmann::json node;
                node["id"]        = 0;
                node["prototype"] = std::vector<float>(xhat.data(), xhat.data() + xhat.size());
                node["health"]    = 1.0f / (1.0f + L.tle_ema);  // green = accurate model
                node["visits"]    = 1;
                gng["nodes"] = nlohmann::json::array({node});
            } else {
                gng["nodes"] = nlohmann::json::array();
            }
            gng["edges"] = nlohmann::json::array();
            gng["step"]  = int(L.steps_seen);
            j["gng"] = std::move(gng);
        }
    } else {
        j["rows_A"] = 0;
        j["cols_A"] = 0;
        j["A"]      = std::vector<float>{};
        j["prev_y"] = std::vector<float>{};
    }
    return j;
}

void MotorEPM::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1 && version != 2)
        throw std::runtime_error("MotorEPM::restore_state: unknown version " + std::to_string(version));
    auto const& legs = s.at("legs");
    legs_.assign(n_legs_, Leg{});
    int m = motor_dim_;
    for (int leg = 0; leg < n_legs_ && leg < int(legs.size()); ++leg) {
        auto const& lj = legs[leg];
        Leg& L = legs_[leg];
        L.initialized = lj.value("initialized", false);
        if (!L.initialized) continue;
        L.n          = lj.value("n", 0);
        L.have_prev  = lj.value("have_prev", false);
        L.steps_seen = lj.value("steps_seen", int64_t(0));
        L.tle_ema    = lj.value("tle_ema", 0.0f);
        L.gain_ema   = lj.value("gain_ema", 1.0f);
        L.outmag_ema = lj.value("outmag_ema", 0.0f);
        L.sat_ema    = lj.value("sat_ema", 0.0f);
        L.knee_ema   = lj.value("knee_ema", 0.0f);
        L.amp_ema    = lj.value("amp_ema", 0.0f);
        L.amp_gain   = lj.value("amp_gain", 1.0f);
        if (lj.contains("babble_rng")) {
            std::istringstream is(lj["babble_rng"].get<std::string>()); is >> L.babble_rng;
        }
        L.rest_captured = lj.value("rest_captured", false);
        if (L.rest_captured && lj.contains("rest_pos")) {
            auto rp = lj["rest_pos"].get<std::vector<float>>();
            L.rest_pos = Eigen::VectorXf(rp.size());
            std::copy(rp.begin(), rp.end(), L.rest_pos.data());
        }
        int n = L.n;
        auto vecf = [](nlohmann::json const& j){ return j.get<std::vector<float>>(); };
        auto toM  = [&](std::vector<float> const& v, int r, int c){
            Eigen::MatrixXf M(r, c); std::copy(v.begin(), v.end(), M.data()); return M; };
        L.A = toM(vecf(lj.at("A")), n, m);
        L.C = toM(vecf(lj.at("C")), m, n);
        L.b = toM(vecf(lj.at("b")), n, 1);
        L.h = toM(vecf(lj.at("h")), m, 1);
        L.prev_x = toM(vecf(lj.at("prev_x")), n, 1);
        L.prev_y = toM(vecf(lj.at("prev_y")), m, 1);
        L.x = Eigen::VectorXf::Zero(n);
        L.fresh = false;
    }
    // --- module-level adaptive state (v2+); absent in v1 snapshots → keep the
    // values applied by on_setup (old behavior: fresh agency/homeostat/panic state).
    if (s.contains("module")) {
        auto const& mod = s.at("module");
        if (mod.contains("coord_rng")) { std::istringstream is(mod["coord_rng"].get<std::string>()); is >> coord_rng_; }
        if (mod.contains("gait_phase"))       gait_phase_       = mod["gait_phase"].get<std::vector<double>>();
        if (mod.contains("coord_best_phase")) coord_best_phase_ = mod["coord_best_phase"].get<std::vector<double>>();
        coord_best_fitness_  = mod.value("coord_best_fitness",  coord_best_fitness_);
        coord_best_init_     = mod.value("coord_best_init",     coord_best_init_);
        coord_probe_counter_ = mod.value("coord_probe_counter", coord_probe_counter_);
        coord_fit_accum_     = mod.value("coord_fit_accum",     coord_fit_accum_);
        coord_fit_count_     = mod.value("coord_fit_count",     coord_fit_count_);
        amp_target_            = mod.value("amp_target",            amp_target_);
        amp_seek_best_target_  = mod.value("amp_seek_best_target",  amp_seek_best_target_);
        amp_seek_best_fitness_ = mod.value("amp_seek_best_fitness", amp_seek_best_fitness_);
        amp_seek_init_         = mod.value("amp_seek_init",         amp_seek_init_);
        amp_seek_counter_      = mod.value("amp_seek_counter",      amp_seek_counter_);
        amp_seek_fwd_accum_    = mod.value("amp_seek_fwd_accum",    amp_seek_fwd_accum_);
        amp_seek_amp_accum_    = mod.value("amp_seek_amp_accum",    amp_seek_amp_accum_);
        amp_seek_count_        = mod.value("amp_seek_count",        amp_seek_count_);
        chassis_h_ema_  = mod.value("chassis_h_ema",  chassis_h_ema_);
        chassis_h_max_  = mod.value("chassis_h_max",  chassis_h_max_);
        height_bias_    = mod.value("height_bias",    height_bias_);
        chassis_h_seen_ = mod.value("chassis_h_seen", chassis_h_seen_);
        panic_phase_   = mod.value("panic_phase",   panic_phase_);
        panic_         = mod.value("panic",         panic_);
        panic_latched_ = mod.value("panic_latched", panic_latched_);
        boredom_esc_held_  = mod.value("boredom_esc_held",  boredom_esc_held_);
        boredom_esc_ticks_ = mod.value("boredom_esc_ticks", boredom_esc_ticks_);
        boredom_streak_    = mod.value("boredom_streak",    boredom_streak_);
        interest_ema_      = mod.value("interest_ema",      interest_ema_);
        interest_ema_init_ = mod.value("interest_ema_init", interest_ema_init_);
        reset_count_       = mod.value("reset_count",       reset_count_);
        ticks_since_reset_ = mod.value("ticks_since_reset", ticks_since_reset_);
        reset_rate_ema_    = mod.value("reset_rate_ema",    reset_rate_ema_);
        reset_rate_init_   = mod.value("reset_rate_init",   reset_rate_init_);
    }
}

} // namespace ogma
