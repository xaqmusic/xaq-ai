// STAGE 0: mechanical copy of MotorEPM.cpp — see MotorEPMv2.hpp for the invariant.
#include "ogma/modules/MotorEPMv2.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

// Oscillation amplitude a leg must exceed before its phase is admitted to the inter-leg
// PLV.  amp_ema is the phase-vector magnitude, i.e. "is this leg actually moving?".
// Without this floor a FROZEN body scores PLV -> 1 (constant phases => constant phase
// difference), which is the same degeneracy that made gait_coherence unusable.
static constexpr float kPlvAmpFloor = 0.02f;

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("MotorEPMv2 param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("MotorEPMv2 param '" + key + "' must be integer");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("MotorEPMv2 param '" + key + "' must be string array");
}
std::vector<double> get_double_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("MotorEPMv2 param '" + key + "' must be numeric array");
}

} // namespace

MotorEPMv2::MotorEPMv2()  = default;
MotorEPMv2::~MotorEPMv2() = default;

std::string_view MotorEPMv2::type_name() const { return "MotorEPMv2"; }

std::vector<TopicSpec> MotorEPMv2::input_topics() const {
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
    if (!upright_topic_.empty())
        v.emplace_back(upright_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!rhythm_topic_.empty())
        v.emplace_back(rhythm_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!contact_topic_.empty())
        v.emplace_back(contact_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!torque_topic_.empty())
        v.emplace_back(torque_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    if (!feet_topic_.empty())
        v.emplace_back(feet_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    return v;
}

std::vector<TopicSpec> MotorEPMv2::output_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(action_topics_.size());
    for (auto const& t : action_topics_)
        v.emplace_back(t, std::type_index(typeid(ActionOut)),
                       SubscriptionKind::Direct, /*required=*/true);
    return v;
}

ParamSchema MotorEPMv2::params_schema() const {
    return {
        {"proprio_topics", ParamMutability::ConstructionOnly,
         "ProprioToken input topics, one per leg (length = n_legs). Each carries the leg-state vector x.",
         std::nullopt, std::nullopt, std::nullopt},
        {"action_topics", ParamMutability::ConstructionOnly,
         "ActionOut output topics, length n_legs*motor_dim, ordered [leg0_j0, leg0_j1, leg0_j2, leg1_j0, ...]. Drop-in for the body's action.<leg>_<joint> channels.",
         std::nullopt, std::nullopt, std::nullopt},
        {"objective_topics", ParamMutability::ConstructionOnly,
         "Optional per-leg PredictionToken topics carrying a SOFT posture target (predicted_latent = motor_dim target joint positions; confidence = weight w). The controller descends toward it (objective-change, not additive — §1.1/§2.4). Empty = socket off (byte-identical HK).",
         std::nullopt, std::nullopt, std::nullopt},
        {"velocity_objective_topics", ParamMutability::ConstructionOnly,
         "Optional per-leg PredictionToken topics carrying a phase-indexed SOFT VELOCITY target (predicted_latent = motor_dim target joint velocities = the propulsive trajectory; confidence = w). Needs cpg_embed + cpg_phase_topic: a second learned feed-forward Cvel is trained to reduce the velocity error (v*−ẋ) at the command phase → the body keeps moving THROUGH the pose (propulsion), where the posture objective only holds it AT the pose. Empty = Cvel stays 0 = byte-identical.",
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
        {"cmd_squash", ParamMutability::HotMutable,
         "ACTUATOR HONESTY (import I3).  0 = today's HARD CLAMP of the assembled command to "
         "[-1,1]; 1 = a smooth tanh squash instead.  Measured motivation: hip1 is clipped 56% "
         "of post-warmup leg-ticks with a mean request of 1.40, so the nonlinearity the BODY "
         "applies is a hard discontinuity while the HK loop-Jacobian G=diag(1-tanh^2) assumes a "
         "smooth one -- L overstates the loop gain wherever the command is railed.  A squash "
         "compresses instead of truncating, preserving the stroke's SHAPE (and therefore its "
         "phase information) through the bound.  0 = byte-identical.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"dep_gain", ParamMutability::HotMutable,
         "DEP (Der & Martius 2015 — reconstructed from the published principle; POSTDATES our "
         "2012 sources, so this is our interpretation, not their code).  Replaces the HK update of "
         "C with the correlation of MOTOR derivatives against the SENSOR derivatives they caused, "
         "row-normalised so dep_gain IS the per-motor loop gain (comparable to c_init).  Amplifies "
         "what the body is already doing instead of exploring away from it — the attractor-forming "
         "property HK lacks.  Costs the forward model: the motor loop stops being predictive.  "
         "0 = off, byte-identical.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{4.0}},
        {"dep_alpha", ParamMutability::HotMutable,
         "EMA rate of DEP's derivative correlation.  Small = long memory of what worked (slower, "
         "more stable modes); large = tracks the last few ticks.",
         ParamValue{0.05}, ParamValue{0.001}, ParamValue{1.0}},
        {"sense", ParamMutability::HotMutable,
         "IMPORT I2: weight on the CONFINING half of the homeokinetic objective — the real ∂G "
         "term (PM's `epsrel`), which sat_lr was a hand-set surrogate for.  It is what stops the "
         "loop collapsing into the degenerate move-nothing minimum (measured decay at 40k: step "
         "rate 12.0 → 5.2 → 2.95).  PM: hexapod 1.5, zoo Sox generator 4.  0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{8.0}},
        {"ctrl_damping", ParamMutability::HotMutable,
         "IMPORT I2's MANDATORY COMPANION: L2 decay on C and h.  sat_lr is the ONLY brake on the "
         "bias integrator h, so retiring it in favour of `sense` without a bound reproduces its "
         "windup (pre-clamp command 14.3 and climbing, 3 of 4 seeds taking zero steps).  PM "
         "supplies this as a separate `damping` param (dog 0.0001, humanoid 0.0001-0.0003) that "
         "we have never had.  0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{0.01}},
        {"whole_body_c", ParamMutability::ConstructionOnly,
         "IMPORT I7: 0 = four independent per-leg controllers (historical); 1 = ONE controller "
         "spanning every joint of every leg, so inter-leg coordination becomes a learnable entry "
         "of C instead of something that can only arrive mechanically through the body.  The "
         "empirical case: HK discovered the hip2+knee lift synergy unaided (joints that SHARE a "
         "C) while inter-leg coherence falls as ctrl_lr rises (joints that do not).  This is also "
         "the structural difference from the Playful Machine, whose dog/hexapod/humanoid all run "
         "ONE Sox across every joint.  0 = byte-identical.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"c_init", ParamMutability::ConstructionOnly,
         "SELF-EXCITING controller init (Playful Machine's Sox cInit).  Adds c_init to each "
         "motor's OWN joint-position feedback weight C(j,3j), so the sensorimotor loop starts "
         "at the edge of instability and the HK gradient SHAPES an existing oscillation instead "
         "of having to create one from a dead fixed point.  Sox uses C=cInit·I with cInit "
         "0.7-1.2 at UNITY output gain; here the effective loop gain is motor_gain·c_init, so "
         "the PM-equivalent value is cInit/motor_gain.  0 = off (legacy small-random init, "
         "byte-identical).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{4.0}},
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
        {"postural_gain_joints", ParamMutability::HotMutable,
         "per-joint [hip1,hip2,knee] RELATIVE profile (MULTIPLIER) on postural_gain — effective_gain[j] = postural_gain * profile[j]. LOOSEN one joint (e.g. profile [1,0.3,1] → hip2 at 30% tone) so the LEARNED controller (C/Cphi) can move it on top of the reflex, while hip1/knee stay at full tone for stance. postural_gain remains an honest GLOBAL knob (scales every joint). Empty/mis-sized = 1.0 for all (pure scalar). See diag postural_eff for the resulting per-joint gains.",
         std::nullopt, std::nullopt, std::nullopt},
        {"explore_noise", ParamMutability::HotMutable,
         "persistent Gaussian motor-noise σ added every tick. Keeps the prediction error ξ nonzero at fixed points so HK does not freeze; the sensitivity-seeking controller amplifies it into oscillation (the homeokinetic exploration drive).",
         ParamValue{0.05}, ParamValue{0.0}, ParamValue{1.0}},
        {"knee_tuck_target", ParamMutability::HotMutable,
         "override the postural knee-rest target (proprio pos) to drive the statically-stable SPIDER stance (knees tucked, chassis suspended below). +0.7..+0.9 = strong tuck. -99 = use the captured spawn pose.",
         ParamValue{-99.0}, ParamValue{-99.0}, ParamValue{1.0}},
        {"hip2_tuck_target", ParamMutability::HotMutable,
         "override the postural hip2(femur)-rest target to CROUCH the femur so hip2 has leverage to lift/load (default rests at the extended spawn pose → no leverage). Sign/magnitude tune per body. -99 = off (spawn pose).",
         ParamValue{-99.0}, ParamValue{-99.0}, ParamValue{2.0}},
        {"motor_gain", ParamMutability::HotMutable,
         "output amplitude multiplier on the HK command (tanh output, before postural+noise). 1 = raw HK; >1 = stronger/larger leg swings (the legs look weak even though servos are strong).",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{5.0}},
        {"coupling_gain", ParamMutability::HotMutable,
         "Rung 3 inter-leg Kuramoto coupling strength. Couples the four legs' own emergent phases toward the gait_phase offsets (entrains the twitching legs to the active one, phase-locks all four). 0 = off (one-leg-spins regime); raise to watch the legs synchronize.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
        {"intent_topic", ParamMutability::ConstructionOnly,
         "MOTOR INTENT from the higher loop: ProprioToken [v_forward*, yaw_rate*]. Commit confidence becomes 'am I achieving what I was asked to do' instead of 'how well do I predict myself'. Empty = commit_prec_gain is INERT, deliberately: confidence has no meaning without a goal, and the residual it used to fall back on was MEASURED to anti-correlate with moving well (corr(motor_tle, displacement) = +0.129 -- a moving body is LESS predictable, so a residual-based confidence penalises exactly the behaviour we want).",
         ParamValue{std::string("")}},
        {"explore_floor", ParamMutability::HotMutable,
         "Lower bound on explore_mult, so commit ATTENUATES search instead of abolishing it. explore_mult currently reaches exactly 0.000 under full commit, which is the frozen-but-confident state the operator reads as indecision. 'Play never abstains' -- fix exploration's output, never its right to win. 0 = legacy. Try 0.15-0.3.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"phase_sym_smooth", ParamMutability::HotMutable,
         "SYMMETRIC phase filter -- the repair for phase_vel_smooth's failure. That lever low-passed only the y-arm of atan2(vel, pos), which shrinks and phase-shifts one component, DISTORTS the ellipse, and warps the phase non-uniformly around the cycle. Because L.phase times the power stroke, the stroke then fires at the wrong point and pushes backward as often as forward: net displacement fell 57% (4.51 m -> 1.91 m, straight 0.356 -> 0.121) even while PATH length rose 21.7%. Filtering BOTH arms with the same kernel rotates the phase vector RIGIDLY -- same noise rejection, but the only phase effect is a CONSTANT offset, which stroke_phase already absorbs. 0 = off, byte-identical. ⚠ JUDGE ON net_disp AND straight, never on path length. Try 2-4.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{32.0}},
        {"phase_vel_smooth", ParamMutability::HotMutable,
         "PHASE-REFERENCE REPAIR. Low-passes the velocity arm of the atan2 that produces L.phase -- the quantity the Kuramoto coupling drives toward gait_phase offsets. MEASURED DEFECT: phase_retro = 0.666, i.e. L.phase runs BACKWARDS two ticks in three, so the coordination layer has been coupling to jitter rather than to an oscillator. The cause is structural: the y-arm is a RAW per-tick joint delta (a high-pass filter), whose noise exceeds the (pos-mean) x-arm near the zero crossings of a ~50-tick cycle. Value is an averaging length: 0 = raw delta, byte-identical; 4 = ~4-tick average. JUDGE IT ON phase_retro FIRST -- that is a property of the signal and needs no behavioural claim. Try 2-8.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{32.0}},
        {"fwd_resonance_gain", ParamMutability::HotMutable,
         "fwd_v RESONANCE FEEDBACK. An adaptive-frequency Hopf oscillator is entrained BY forward velocity -- it LEARNS the frequency the body already propels itself at (fwd_v is the one signal we trust; foot contacts measured as a poor proxy for a step, and the knee-derived L.phase is what the coupling currently rides). This gain then couples the leg oscillators TO that measured rhythm, closing the loop: motion -> fwd_v oscillation -> resonator locks -> legs entrain -> the stroke lands where propulsion actually happens -> more motion. Nothing is injected: the reference is the body's own velocity, so it reinforces the rhythm the body FOUND. 0 = off, byte-identical. Watch couple_R, res_freq, res_lock and phase_retro in the graph. Try 0.2-1.0.",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"intent_rhythm_gain", ParamMutability::HotMutable,
         "STRIDE-PROFILE PREDICTION. Adds 'did this stride look like my strides?' to the intent error. A constant v* is a target a legged body CANNOT hold -- it advances in pulses, so a level-seeking error oscillates forever and commit chases it; measured pulse_cv 0.583 with a p90/p50 gap tail of 2.10, IDENTICAL across every commit arm, which is why they all tie. The body learns its own forward-velocity waveform indexed by gait phase (16 bins, LEARNED from what it does -- no rhythm is injected, doctrine §7) and the error becomes this stride's deviation from it. Descending that means 'make this stride like my strides' = consistent forward pulses, with the waveform's SHAPE still chosen by the body. 0 = off, byte-identical. Try 0.5-2.0.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{4.0}},
        {"lookahead_gain", ParamMutability::HotMutable,
         "RUNG 1 -- ACT ON THE PREDICTED STATE. lambda in x_eff = (1-lambda)*x + lambda*x_hat, fed to the controller instead of the raw sensor. The forward model x_hat = A*y + b is already learned every tick but is consumed ONLY to form the residual: today the loop predicts to LEARN and never to ACT, so the controller always acts on stale state while every loop carries delay (sensing, filtering, actuation). This compensates all of it with a quantity already computed. It also subsumes the phase-filter failures -- L.phase times the power stroke, so lag fires the stroke late and the leg drives backward through part of the stride; the fix is to act ahead, not to filter better. 0 = off, byte-identical. NEGATIVE = the wrong-sign control, which MUST regress. Try 0.2-0.6.",
         ParamValue{0.0}, ParamValue{-1.0}, ParamValue{1.0}},
        {"lookahead_mode", ParamMutability::HotMutable,
         "How x_hat resolves the circularity that x_hat_{t+1} = A*y_t + b needs y_t, the very thing being computed. 0 (default) = FIXED POINT: one Jacobi iteration -- y0 from x, x_hat from y0, y1 from x_hat. A true one-step lookahead, one extra matmul per leg. 1 = PREV-ACTION: x_hat = A*prev_y + b, assuming action continuity. Mode 1 is the CHEAP CONTROL: if it matches mode 0 then the effect is not lookahead, it is any perturbation of the controller input.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"lookahead_null", ParamMutability::HotMutable,
         "CONTROL ARM: drop the dynamics term so x_hat := b instead of A*y + b. The prediction keeps its magnitude and its role in the equation but loses all knowledge of how the action moves the body. Separates 'acting on a PREDICTION' from 'shrinking x toward a constant', which would otherwise be a confound indistinguishable in the aggregate metrics. 0 = full model, 1 = null.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"intent_yaw_gain", ParamMutability::HotMutable,
         "Weight on the YAW term of the intent error. 1.0 = the original (v*, w*) 2-vector. 0.0 = PROGRESS OVER GROUND ONLY. Operator, 2026-08-05: \"chassis yaw should not be part of this function since the chassis oscillates constantly while stepping; if heading needs adjusting that must come from some other mechanism that affects the bilateral stride symmetry, not the direction the chassis happens to be facing. The progress over ground relative to the CoG is all that matters.\" That is a separation-of-concerns argument and the other mechanism ALREADY EXISTS: goal_bearing_topic -> the heading PD -> steer_eff -> the bilateral stroke-amplitude differential. A quadruped yaws every stride BY CONSTRUCTION, so penalising instantaneous chassis yaw asks the body to stop doing the thing that moves it -- gait mechanics leaking into a goal-achievement scalar.",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{4.0}},
        {"commit_prec_gain", ParamMutability::HotMutable,
         "COMMIT AS EARNED PRECISION. Scales the commit window/rise/decay by how well the body is predicting ITSELF right now, measured as the forward-model residual's shortfall against its own running mean (scale-free: nothing is tuned to tle's magnitude). Predicting better than usual => shorter qualifying window, faster ramp, slower release; worse => the reverse. 0 = the fixed schedule, byte-identical. WHY: commit is a precision, and three hand-picked crossover points have now been measured (180/240/90 = 14% stalled, off = 20%, inverted = 22%) -- the constant should be set by the mechanism that ought to set it. It can also engage INSIDE a 1-2 s burst, which a fixed 180-tick window cannot. Try 1.0-3.0.",
         ParamValue{0.0}, ParamValue{-4.0}, ParamValue{4.0}},
        {"commit_window_ticks", ParamMutability::HotMutable,
         "Ticks of sustained forward progress before progress->commit engages at all. Default 180 (3 s). ⚠ MEASURED MISMATCH: bursts last 1-2 s, so at 180 commit arrives AFTER the burst it was meant to protect. Lower it to engage inside a burst.",
         ParamValue{180.0}, ParamValue{1.0}, ParamValue{1200.0}},
        {"commit_rise_ticks", ParamMutability::HotMutable,
         "Ticks to ramp commit to full once engaged. Default 240 (~4 s).",
         ParamValue{240.0}, ParamValue{1.0}, ParamValue{2400.0}},
        {"commit_decay_ticks", ParamMutability::HotMutable,
         "Ticks to release commit once progress falls. Default 90 (~1.5 s) -- tuned for 'release quickly, re-explore', which is right for escaping a stuck state and BACKWARDS for holding a found rhythm. RAISE it to stop abandoning a gait on the first faltering step.",
         ParamValue{90.0}, ParamValue{1.0}, ParamValue{2400.0}},
        {"heading_trim_rate", ParamMutability::HotMutable,
         "THE MISSING INTEGRAL TERM on heading. The controller is P+D only, so a persistent yaw disturbance leaves a NONZERO steady-state steer and one side's stroke sits permanently near its clamp (measured: right legs request 2.10 vs a +-1 limit, 77% clip duty, against 0.70 on the left) -- which is why steering authority is direction-dependent. This learns that DC effort and hands it back. Targets FUNCTIONAL symmetry (zero net yaw), NOT amplitude symmetry, which is where the ~35-lever symmetry family died. 0 = off, byte-identical. Try 1e-4.",
         ParamValue{0.0}, ParamValue{-0.01}, ParamValue{0.01}},
        {"heading_trim_leak", ParamMutability::HotMutable,
         "Per-tick leak on heading_trim (fraction). MANDATORY >0: unbounded integrator windup is this codebase's characteristic failure shape (three documented cases). tau = 1/leak ticks.",
         ParamValue{0.001}, ParamValue{0.0}, ParamValue{0.1}},
        {"c_pair_init", ParamMutability::ConstructionOnly,
         "Give left/right partner legs (0&1, 2&3) the SAME initial control law instead of independent random ones. The skid handedness is seed-random (R/L hip1 demand 3.07/4.35/0.69/0.97 across 4 seeds, and it still flips with all learning off), so no side should start with an advantage. stroke_signs still mirrors the output; fore/aft partners stay different. 0 = legacy per-leg random init. ⚠ Removes half the documented inter-leg symmetry breaker -- measure, do not assume.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"goal_bearing_topic", ParamMutability::ConstructionOnly,
         "L1 NAV SETPOINT. ProprioToken [vx,vy], an EGOCENTRIC unit vector (vy = forward) naming where the nav layer wants to go; atan2(vx,vy) becomes the heading PD's bearing error. Empty (default) = the PD holds the spawn bearing exactly as before = BYTE-IDENTICAL. Deliberately NOT nav_topic: nav_on gates the whole heading PD off (P and D) plus the forward facing gate, which discards the variance collapse the PD exists for.",
         ParamValue{std::string("")}},
        {"couple_prec_gain", ParamMutability::HotMutable,
         "PRECISION-WEIGHT the Kuramoto neighbour average by each neighbour's OWN prediction error: w_j = (amp_j/(tle_j+eps))^k, L1-normalised. 0 = the legacy uniform mean over the other legs (byte-identical). 1 = the LateralVoter's plain 1/(tle+eps) trust, one layer down: a leg pulls toward its neighbours in proportion to how well each of THEM predicts itself, so a flailing leg stops dragging the other three. NEGATIVE = the wrong-sign control arm (trust the leg that predicts itself WORST). k is a sharpness exponent, not a scale: the normalised weights are invariant to a common factor on the precisions, so nothing here is tuned to the residual's magnitude.",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"phase_joint", ParamMutability::ConstructionOnly,
         "Proprio joint index (0=hip1, 1=hip2, 2=knee) whose motion derives the per-leg oscillator phase L.phase (used by the Kuramoto coupling AND the stroke). -1 = knee (m-1, legacy). Set to 0 (hip1, the fore-aft locomotor joint) to lock coordination to the actual stride rhythm rather than the knee's faster flexion.",
         ParamValue{int64_t(-1)}, std::nullopt, std::nullopt},
        {"rhythm_gains", ParamMutability::HotMutable,
         "Per-joint [hip1,hip2,knee] amplitude of a coherent rhythmic drive sin(L.phase+offset) that locks ALL joints to ONE leg phase/frequency (the intra-leg-coherence fix — the keyframe can only crystallize when a leg's joints share one frequency). Empty/all-0 = off (legacy: only hip1 rhythmic, joints drift to own frequencies).",
         std::nullopt, std::nullopt, std::nullopt},
        {"rhythm_offsets", ParamMutability::HotMutable,
         "Per-joint [hip1,hip2,knee] phase offset (rad) for the rhythmic drive.",
         std::nullopt, std::nullopt, std::nullopt},
        {"cpg_phase_topic", ParamMutability::ConstructionOnly,
         "Optional global CPG phase topic (rhythm.cpg.body) to drive the rhythm from — a clean, entrained phase (leg_phase = cpg_phase + gait_phase[leg]) so the whole body locks to ONE frequency. Empty = use the proprio-derived L.phase.",
         std::nullopt, std::nullopt, std::nullopt},
        {"cpg_embed", ParamMutability::HotMutable,
         "CPG-as-embedding: controller learns a phase-dependent feed-forward y=tanh(C·x + Cphi·[cosφ,sinφ] + h). Cphi is trained to reduce the KEYFRAME error (x*−x) at the command phase (not HK surprise) → a phase-indexed push toward the learned posture. Needs cpg_phase_topic + the objective socket. Cphi starts 0 (byte-identical until learned). false = off.",
         ParamValue{false}, std::nullopt, std::nullopt},
        {"embed_lr", ParamMutability::HotMutable, "Cphi learning rate on the keyframe error (phase-indexed feed-forward).",
         ParamValue{0.02}, ParamValue{0.0}, ParamValue{1.0}},
        {"embed_decay", ParamMutability::HotMutable, "L2 decay bounding the learned Cphi feed-forward.",
         ParamValue{0.001}, ParamValue{0.0}, ParamValue{1.0}},
        {"ctrl_symmetry_gain", ParamMutability::HotMutable,
         "Per-leg controller symmetry coupling: per-tick pull of each leg's learned controller (C,h,Cphi) toward its group's cross-leg average, so the four identical legs converge to ONE control law instead of one specializing into a skid (the RR asymmetry). Needs symmetry_group_of. A learning-time prior (shapes what legs LEARN), not a command. 0 = off (byte-identical).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"symmetry_group_of", ParamMutability::ConstructionOnly,
         "Per-leg group id (length n_legs) for ctrl_symmetry_gain; legs sharing an id are coupled. Group by SAME stroke-sign to stay sign-safe, e.g. [0,1,0,1] for stroke_signs [1,-1,1,-1] (couples same-side legs). Empty = off.",
         std::nullopt, std::nullopt, std::nullopt},
        {"rhythm_fade_start", ParamMutability::HotMutable,
         "Gate 2: tick to begin linearly fading the rhythm scaffold to 0 (coherent-scaffold wean: the learned keyframe takes over). -1 = disabled.",
         ParamValue{int64_t(-1)}, std::nullopt, std::nullopt},
        {"rhythm_fade_end", ParamMutability::HotMutable,
         "Gate 2: tick at which the rhythm scaffold reaches 0 (must be > rhythm_fade_start).",
         ParamValue{int64_t(-1)}, std::nullopt, std::nullopt},
        {"coupling_fade_start", ParamMutability::HotMutable,
         "Gate 2: tick to begin linearly fading the imposed coupling to 0 (crystallize-then-wean so the learned map takes over). -1 = disabled.",
         ParamValue{int64_t(-1)}, std::nullopt, std::nullopt},
        {"coupling_fade_end", ParamMutability::HotMutable,
         "Gate 2: tick at which coupling reaches 0 (must be > coupling_fade_start).",
         ParamValue{int64_t(-1)}, std::nullopt, std::nullopt},
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
        {"stuck_explore_gain", ParamMutability::HotMutable,
         "STUCK→EXPLORE desire (active-inference-native propulsion): when forward velocity (fwd_v EMA, compliant) stays below threshold for a sustained window (~5 s), AMPLIFY the exploration channels — explore_noise and the coord phase-search σ — so the gait DISCOVERS a push (curiosity, no external goal). Self-terminating: decays the instant forward progress resumes. Composes with the bearing-hold (which holds heading straight while it searches → directed exploration down the corridor). = max amplification at full stall; 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{5.0}},
        {"progress_commit_gain", ParamMutability::HotMutable,
         "progress→COMMIT desire (lever C, the inverse twin of stuck→explore): when forward progress (fwd_v EMA) stays HIGH for a sustained window (~3 s), ramp a boost that (1) DAMPS the exploration channels (coord phase-search σ + explore_noise) so the gait stops re-searching a found push, and (2) ADDS stroke thrust so it drives into the committed direction. Kills the exploratory dither once a push is found; self-correcting (decays the instant progress falls, re-opening exploration). Mutually exclusive with stuck→explore by construction. Egocentric. = max exploration-damp + thrust-add at full commit; 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{3.0}},
        {"forward_flow_gain", ParamMutability::HotMutable,
         "forward-FLOW homeostat (lever D, homeokinesis applied to locomotion): amplify stroke thrust ∝ the QUALITY of forward flow = magnitude · predictability (strong AND steady fwd_v). A predictability weight 1/(1+k·volatility) rewards smooth forward flow, not raw speed — the homeokinetic heart (predictable sensorimotor flow is intrinsically sought). Continuous (no threshold), self-easing when flow turns erratic. Egocentric. = max stroke amplification at ideal flow; 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
        {"stance_lift_gain", ParamMutability::HotMutable,
         "STANCE-LIFT (belly-up while walking, chassis protection): a KNEE bias applied ONLY to legs currently in STANCE (planted — Cruse foot-height detector) to hold the chassis high off the ground it can push against. Traction-preserving (pushes off planted feet; unlike the hip2 height_homeo lift which rotates the feet off the slope) and rhythm-safe (swing legs get NO bias, so the stepping cycle is untouched — the 'DC knee bias kills the gait' warning only applies to an UNGATED bias). Held constant (not faded) so the belly rides high during fast flat traversal. Requires feet_topic wired. Sign set empirically (which knee dir raises the chassis on a planted foot). 0 = off.",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"stance_release_frac", ParamMutability::HotMutable,
         "STROKE-DIRECTION-AWARE STANCE RELEASE.  The stance biases (stance_lift + its hip2 fraction) press a planted leg down through its ENTIRE stance -- including the measured 7-9 ticks after that leg's own COMMANDED stroke has reversed (flbrake.py 2026-08-09: liftoff lags the commanded reversal body-wide with only 2-3 ticks of servo slew, and the pressed window costs -0.004..-0.015 g/tick of braking shear per leg; posture knobs redistribute WHICH leg pays, never the toll).  From the tick a planted leg's own commanded hip1 delta flips sign (recovery onset), multiply its stance biases by (1 - this) until it leaves stance; reset at the next touchdown.  Fully egocentric (the brain's own command stream), no propulsive-sign convention needed -- whichever way the stroke was going, continuing to press after it turns around is what delays the lift.  Candidate for the shuffle attractor (a stance bias pressing all four planted feet through recovery is a stall-shaped loop).  0 = off, byte-identical.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"swing_hyst_frac", ParamMutability::HotMutable,
         "SWING-DETECTOR DEADBAND, in units of the foot's own running mean-absolute-deviation from its height EMA. The bare `foot_y > foot_y_ema` test has NO deadband, so it splits ~50/50 by construction — it reports gait PHASE, not ground contact — and it closes a positive feedback loop with any consumer that moves the foot (stance_lift, Cruse): bias lifts the foot above its EMA → declared swing → bias removed → foot drops → declared stance → bias returns. That relaxation oscillator runs at the EMA's ~50-tick timescale and competes with the body's own ~70-tick stride, so its cost scales with the consumer's gain. The band stays RELATIVE (foot_y is world-Y; an absolute threshold would call a planted foot on raised terrain permanently swinging — the blindness that retired chassis_y_norm) and self-scales to the gait's own amplitude rather than being tuned. ~1.0 ≈ one mean deviation of hold. 0 = legacy no-deadband detector (byte-identical).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{3.0}},
        {"homeo_leak_upright_only", ParamMutability::HotMutable,
         "POSTURE-GATE the homeostat leak: forget fast while upright, STOP forgetting while uprightness is below this, so the integrators can accumulate and escalate out of a bad posture. This is the INVERSE of homeo_upright_gate (which froze the integrators while inverted and removed the 40x escalation that rights the robot) and a cleaner reading of the same operator finding than homeo_leak_progress_gate, which cannot distinguish CLIMBING from STUCK — gating on forward progress switches the leak off on the hump where the robot legitimately slows, readmitting the height windup (measured hump 6.09 -> 4.84). Requires upright_topic. ~0.5 = stop forgetting past ~60 deg of tilt. 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"homeo_leak_progress_gate", ParamMutability::HotMutable,
         "PROGRESS-GATE the homeostat leak: scale it by forward progress, so the integrators forget fast while the body is getting somewhere and stop forgetting entirely while stalled. A CONSTANT leak is wrong for the same reason a freeze is — the wind-up is not only damage, it is the escalation that escapes a bad posture. Found by hand and replicated: with leak=5 the robot could NOT get off a 30-degree wall; leak=0 let it accumulate until it escaped, then leak=5 restored the walk quickly. This automates that two-position law on height_rest_frac (1 at rest, 0 cruising), which the module already maintains. Forgetting is a luxury of success. 0 = constant leak (byte-identical).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"homeo_leak_cycles", ParamMutability::HotMutable,
         "HOMEOSTAT LEAK, in STRIDE CYCLES: the time constant over which height_bias and amp_gain forget toward neutral (0 lift / unity gain). Makes their memory finite by construction, which is the fix for the inversion-forgetting failure — that was not excess plasticity but ASYMMETRIC plasticity: both wound fast in one direction and could not return (amp_gain only unwinds when amp_ema EXCEEDS target; height_bias is multiplied by height_rest_frac which -> 0 while moving, so walking froze it). A leak needs no regime classifier, no uprightness signal and no snapshot, and unlike a freeze it never blocks the wind itself — so the 40x amplitude escalation that rights an inverted robot is preserved, it just does not persist. Rate is derived from the body's own measured omega (rhythm_topic), so it tracks the actual gait frequency instead of being a tuned tick count. SMALL values are the useful regime: a persistent error balances the leak at x ~ k*err/lambda, so ~2-3 cycles bounds an excursion AND forgets it in a couple of seconds, whereas ~10 cycles barely bounds it at all. 0 = off (byte-identical).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{60.0}},
        {"rhythm_topic", ParamMutability::ConstructionOnly,
         "Body-rhythm topic (rhythm.body.gait from BodyRhythmTracker: [cos, sin, omega]). Only omega is read, to express homeo_leak_cycles in stride cycles. The CPG's own phase token carries no omega, hence a separate input. Empty = fall back to a default stride period.",
         ParamValue{std::string("")}, std::nullopt, std::nullopt},
        {"homeo_upright_gate", ParamMutability::HotMutable,
         "UPRIGHT GATE for the homeostat integrators (height_bias, amp_gain): freeze them whenever uprightness (cos_pitch*cos_roll, ~basis.y.y from the accelerometer) falls below this. Both integrate toward setpoints that are MEANINGLESS when the body is not upright — the belly rangefinder is not looking at the ground it stands on, and amp_target is unreachable — so an inverted episode rails both and they never return. Measured 2026-07-26: after a self-righting from inversion, forward progress stayed NEGATIVE for 7200+ ticks with height_bias pinned at its clamp and amp_gain stuck 30-40x high, while the LEARNED structures (HK self-model, phase search) recovered fine. So this forgetting is an integrator-windup bug, not a learned-weights problem. ~0.5 = freeze past ~60 deg of tilt. 0 = off (byte-identical).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"height_unwind_free", ParamMutability::HotMutable,
         "ASYMMETRIC WINDUP FADE for height_bias. The windup fix multiplies the height integration by height_rest_frac (which -> 0 while moving forward), so a bias cannot wind up while walking — but it equally cannot UNWIND while walking, so a value railed during a disruption is latched for as long as the robot keeps trying to walk. Non-zero fades ONLY the winding direction, preserving the incline fix while letting a railed bias recover. 0 = legacy symmetric fade (byte-identical).",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"coord_fitness_mode", ParamMutability::HotMutable,
         "Which fitness the (1+1) coordination search (coord_reward_drive) ranks probes by. 0 = LEGACY forward-velocity — a TASK REWARD, which §5.1 forbids and which is what gives the ratchet something destructive to lock in (a thrash that momentarily scores high fwd_v while escaping becomes the incumbent, and every normal probe afterwards is reverted back to it). 1 = REWARD-FREE: coherence · activity / (1 + tle) — phase-lock quality × oscillation amplitude ÷ forward-model error, containing no position/distance/velocity term. All three factors are needed: coherence alone is maximal on a FROZEN body (φ=atan2(0,0)=0 for every leg → Kuramoto R=1), 1/(1+tle) alone also favours freezing (a still body is trivially predictable), and the activity factor is the homeokinetic normalisation that kills both. 0 = byte-identical legacy.",
         ParamValue{int64_t(0)}, ParamValue{int64_t(0)}, ParamValue{int64_t(1)}},
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
         "CRUSE/Walknet inter-leg coordination CORRECTOR on hip2 (foot lift). Continuous bias: Rule 1 (anterior leg in swing → hold this leg in stance, +hip2 down), Rule 2 (anterior just-planted → release this leg's swing, −hip2 up), Rule 3 (contralateral in swing → hold stance). Catches per-leg co-swing / support-loss the MotorEPM rhythm alone leaves; needs the rhythm it cannot itself generate. 0 = off.  NEGATIVE allowed for the sign-flip audit (inverts plant↔lift).",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"cruse_rule3_weight", ParamMutability::HotMutable,
         "Rule 3 (contralateral load tolerance) weight relative to Rule 1 (anterior). 0.5 = contralateral hold is half-strength vs the anterior coupling. ⚠️ INERT UNLESS cruse_gain != 0 — this is a sub-weight INSIDE the cruse_gain block, not an independent lever, so its non-zero default DISPLAYS AS ENABLED in the panel while doing nothing. Check cruse_gain before crediting or blaming this knob (see the ledger's `postural_gain_joints` silent-no-op case: a knob that cannot act still looks causal).",
         ParamValue{0.5}, ParamValue{0.0}, ParamValue{2.0}},
        {"cruse_rule2_window", ParamMutability::HotMutable,
         "ticks after the anterior leg's touchdown during which Rule 2 actively releases this leg's swing (the constructive lift). ~15 = 0.25 s @ 60 Hz. ⚠️ INERT UNLESS cruse_gain != 0 (sub-parameter of the cruse_gain block, same trap as cruse_rule3_weight).",
         ParamValue{15}, ParamValue{1}, ParamValue{120}},
        {"cruse_rule5_gain", ParamMutability::HotMutable,
         "CRUSE Rule 5 (load distribution): a leg in stance presses its foot down (hip2+knee) ∝ the number of OTHER legs currently in swing — redistributing the swinging legs' weight onto the planted ones. More normal force → more friction → less foot scrub (the stance feet were sliding ~3-4× the body's progress). Shifts CoG onto the support. 0 = off.  NEGATIVE allowed for the sign-flip audit.",
         ParamValue{0.0}, ParamValue{-1.0}, ParamValue{1.0}},
        {"upright_topic", ParamMutability::ConstructionOnly,
         "Uprightness topic (reality.proprio.upright = chassis basis.y.y): +1 upright, 0 on its side, -1 inverted. Required for homeo_upright_gate to do anything. Do NOT rely on tilt_topic for this — the body's publish_tilt defaults FALSE, so tilt never arrives headless and the gate becomes silent dead code. Empty = no subscription.",
         ParamValue{std::string("")}, std::nullopt, std::nullopt},
        {"contact_topic", ParamMutability::ConstructionOnly,
         "TRUE per-leg ground-contact topic (reality.proprio.foot_contact) — the physics touch flag, and the sensor a REAL picrawler has (a foot switch). When set, a leg is in SWING iff it is not touching: a MEASUREMENT, with none of the chatter or latching the height proxy suffers. When empty, MotorEPM falls back to INFERRING contact from foot height vs that foot's own moving average, which measured 40.3 % 'swinging' while the feet were genuinely down 99.3 % of the time — and every consumer of the swing state (stance_lift, all Cruse rules) gated on that error. This topic was already being published every tick and was simply never wired. Empty = legacy inference (byte-identical).",
         ParamValue{std::string("")}, std::nullopt, std::nullopt},
        {"feet_topic", ParamMutability::ConstructionOnly,
         "4-D per-leg foot-height ProprioToken topic for Cruse stance/swing detection. The body publishes reality.proprio.feet_y every tick.",
         std::nullopt, std::nullopt, std::nullopt},
        {"stroke_gain", ParamMutability::HotMutable,
         "hip1 (fore-aft) propulsion drive amplitude, phase-locked to each leg's step phase. Aligns the stroke DIRECTION so thrust sums to translation, not the tangential spin HK settles into. 0 = HK-only hip1.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
        {"stroke_phase", ParamMutability::HotMutable,
         "phase offset (rad) between the knee/step phase and the hip1 fore-aft drive — tunes when the leg pushes back relative to when it lifts.",
         ParamValue{0.0}, ParamValue{-3.15}, ParamValue{3.15}},
        {"stroke_phase_src", ParamMutability::HotMutable,
         "STROKE-TO-STEP LOCK. Which phase the power stroke rides. 0 (default) = the legacy per-leg oscillator phase L.phase, derived from phase_joint (the KNEE) -- BYTE-IDENTICAL. 1 = a touchdown-referenced STEP CLOCK from true foot contact: phi = 2pi*(ticks since touchdown)/EMA(inter-touchdown interval), so phi=0 IS touchdown and stroke_phase finally selects where in the STEP the push lands. 2 = the same clock with touchdown detected from hip1 |joint_torque| crossing its own mean (buildable on the real robot via servo current sensing, where a foot switch is not; but measured tq_agree is only 0.540 against a 0.5 chance line, so expect a much noisier clock than source 1). WHY: Phase 0 measured the stroke riding a 22-24 tick knee clock while the leg steps every 26-30, beating at ~2.5 s, with the fraction of STANCE in the stroke's positive half at 0.512 against 0.513 over SWING -- push direction statistically INDEPENDENT of whether the foot is down. This is NOT phase_joint=0 again: that refutation was a self-excited oscillator (the stroke drove the same hip1 an atan2 read its phase from), and its premise measured POSITIVE (step_bal 0.30->0.41-0.58). Here the stroke reaches the reference only through the world. Requires contact_topic (src 1) or torque_topic (src 2); with neither wired the clock never locks and the stroke silently keeps using L.phase. NOTE FOR READING THE RESULT: with phi referenced to touchdown, td_plv -> ~1.0 and pos_stance becomes a deterministic function of stroke_phase and duty -- they become CONSUMER VERIFICATION, not evidence. Judge behaviourally and on mv_stance/mv_swing.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
        {"step_phase_debounce", ParamMutability::HotMutable,
         "Ticks of consistent contact required before a touchdown is accepted by the step clock (stroke_phase_src > 0). Contact is a physics touch flag and flickers on impact; an un-debounced edge would reset the phase twice per step and reintroduce exactly the chatter that makes the incumbent foot-height detector unusable (it fires every 12-15 ticks against a 26-30 tick step).",
         ParamValue{2.0}, ParamValue{1.0}, ParamValue{20.0}},
        {"step_period_alpha", ParamMutability::HotMutable,
         "EMA rate on the measured inter-touchdown interval (the step clock's period estimate). Higher = tracks a changing gait faster, noisier phase.",
         ParamValue{0.2}, ParamValue{0.01}, ParamValue{1.0}},
        {"step_period_min", ParamMutability::HotMutable,
         "Sanity floor (ticks) on an accepted inter-touchdown interval before it enters the period EMA, so one anomalous stride cannot drag the estimate. The measured step is 26-30 ticks.",
         ParamValue{8.0}, ParamValue{2.0}, ParamValue{1000.0}},
        {"step_period_max", ParamMutability::HotMutable,
         "Sanity ceiling (ticks) on an accepted inter-touchdown interval. See step_period_min.",
         ParamValue{200.0}, ParamValue{2.0}, ParamValue{10000.0}},
        {"step_phase_lock", ParamMutability::HotMutable,
         "How hard a touchdown pulls the step clock toward phi=0 (stroke_phase_src > 0). 0.10 (default) = BodyRhythmTracker's proportional phase-lock: phi integrates omega every tick and is SOFTLY pulled at each touchdown, so the driven waveform never jumps. 1.0 = a hard snap to phi=0. THE HARD SNAP WAS MEASURED AND IT COLLAPSES THE GAIT (corridor, n=4: net_z 4.58 -> -0.16, tilt_sd 0.065 -> 0.34, the body inverts repeatedly and convulses in place), for two compounding reasons: (1) the stroke can CAUSE touchdowns, so resetting phase ON touchdown is positive feedback -- a push bounces the foot, the bounce re-triggers the reset, and the period estimate runs to its rail; (2) sin(phi+stroke_phase) is a CONTINUOUS motor command, so snapping phi steps the command every time a foot lands off-schedule, which is precisely what an unlocked gait does. The generalizable rule, and the reason the first build got it wrong: a phase that DRIVES a continuous command needs a soft pull (BodyRhythmTracker), while a phase that is only READ to index a discrete bin can take a reset (SynergyTimer). 1.0 is kept reachable so that refutation stays reproducible.",
         ParamValue{0.10}, ParamValue{0.0}, ParamValue{1.0}},
        {"gait_raster_diag", ParamMutability::HotMutable,
         "DIAGNOSTIC ONLY (>0 = on, 0 = off and the whole block is skipped). Maintains a 512-tick ring of per-leg footfall bits (true contact, the stroke's commanded sign, and the incumbent foot-height detector's swing state) which the live inspector renders as a Hildebrand plot. Makes the stroke-vs-step relation -- the thing that is invisible in aggregate metrics and obvious in a picture -- observable while the robot walks, and shows the incumbent detector's documented ~2x-per-step chatter next to ground truth. Shipped as a ring rather than accumulated by the UI because DiagPublisher throttles each subscription to ~30 Hz against a ~52 tick/s brain, which would alias at exactly the touchdown edges. Feeds no command and draws no randomness.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"steer", ParamMutability::HotMutable,
         "left/right hip1 drive differential — the steering lever. 0 = straight; ± = turn (the clockwise spin is steer-like). Adds side_sign·steer to each leg's stroke.",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"stroke_signs", ParamMutability::HotMutable,
         "per-leg hip1 stroke direction, length n_legs [FL,FR,RL,RR]. Parallel-caudal pattern → forward; all-same → tangential spin. Default [1,-1,1,-1] (forward guess, confirm by eye).",
         std::nullopt, std::nullopt, std::nullopt},
        {"propulsion_balance_gain", ParamMutability::HotMutable,
         "per-leg propulsive-credit homeostat. Each leg's FUNCTIONAL fore-aft contribution (hip1 motion phase-aligned with the power stroke — a 'dragging' planted-but-static leg scores ~0) is tracked; a below-group-mean leg gets a self-limiting boost in its stroke direction so it pulls its weight and L/R propulsion equalizes (straighter). Functional (phase-aligned), NOT amplitude/RMS — distinct from the refuted symmetry levers. 0 = off.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{2.0}},
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
        {"heading_hold_gain", ParamMutability::HotMutable,
         "HEADING-HOLD desire (robust go-straight): damps the smoothed body yaw rate with a per-side hip1 differential IN-PHASE with the fore-aft power stroke, so it composes with propulsion-balance / the emergent gait instead of fighting it (unlike heading_gain, which modulated steer magnitude and circled embed). A 'hold the bearing' prior. 0 = off; sign tunable.",
         ParamValue{0.0}, ParamValue{-3.0}, ParamValue{3.0}},
        {"heading_bearing_hold_gain", ParamMutability::HotMutable,
         "HEADING-HOLD-TO-SPAWN (P term): steers to drive the dead-reckoned bearing error (heading_bearing_ = integrated yaw rel. to spawn, Markov-compliant, reset on respawn) back to 0, ROUTED THROUGH the authoritative skid-steer channel (folds into stroke magnitude → real L/R thrust differential that turns the body), not the weak additive nudge it was before. Pairs with heading_hold_gain (D = yaw-rate damping ~0.3) for a clean non-oscillating hold. Gated off during nav. POSITIVE = go-straight (straightness climbs with P through ~5+); NEGATIVE = catastrophic positive-feedback spin. 0 = off.",
         ParamValue{0.0}, ParamValue{-12.0}, ParamValue{12.0}},
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
        {"support_select_gain", ParamMutability::HotMutable,
         "HOMEOKINETIC SUPPORT SELECTOR.  Modulates EXPLORATION by how responsive the body's current support state is: value = responsiveness/(motor_tle+eps), where responsiveness is the egocentric |dx|/|du| (sensor change per unit commanded change) accumulated per COUNT OF PLANTED FEET.  Below-average value (four feet down, unresponsive, going nowhere) raises exploration; above-average commits.  MEASURED: responsiveness 0.470 at 2 planted vs 0.367 at 4 (+28%) with |du| flat, so this prefers 2-leg support WITHOUT being told forward progress is good -- the distinction between homeokinesis and reward shaping on progress (§5.1).  The TLE divisor is mandatory: 1-planted is equally responsive but is falling.  Keyed on sum(contact) rather than the support EPM because the integer measurably explains MORE of the responsiveness variance than the 150-node vocabulary does.  Requires contact_topic (with contact_instrument_only=1, so the stance gate is untouched).  Acts only on explore_mult -- no joint commanded, no coordination imposed.  0 = off, byte-identical.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{4.0}},
        {"stance_lift_hip2", ParamMutability::HotMutable,
         "COMPLETE THE STANCE LIFT.  Fraction of `stance_lift_gain` also applied to hip2, SAME sign, on PLANTED legs only.  stance_lift is currently knee-only on the reasoning 'no hip2 -> no foot-lift traction loss' -- but that covers hip2 MINUS (foot up); on a planted foot hip2 PLUS presses the foot down and raises the chassis (Rule 5: '+hip2 = press foot down'), and the panic pathway drives hip2+ and knee+ together for precisely this, recording that opposite signs cancel the lift.  MEASURED 2026-08-07: hip2 and the knee agree on sign only 50.8% of ticks, and `height_lift_knee` -- the same idea on the HEIGHT path -- was NULL because that path is faded to zero while cruising.  This one is stance-gated, so it is live exactly when the body is walking, and it can never hoist a swing leg.  0 = off, byte-identical.",
         ParamValue{0.0}},
        {"height_lift_knee", ParamMutability::HotMutable,
         "COMPLETE THE LIFT.  Fraction of the height homeostat's hip2 lift also applied to the KNEE, same sign.  MEASURED 2026-08-07: hip2 and the knee agree on sign only 50.8% of ticks (chance), and the panic pathway's own comment records that opposite signs mean the knee UN-TUCKS and fights the hip2 lift so the chassis does not rise -- 'same sign = a coherent anti-gravity push'.  Panic already drives both joints for that reason; the height homeostat drives hip2 alone, a one-joint version of a two-joint action.  This completes it.  Not an imposed coordination topology: it adds no new coupling between joints, it extends an existing anti-gravity command to the joint the codebase already measured as necessary.  0 = off, byte-identical.",
         ParamValue{0.0}},
        {"height_ground_gain", ParamMutability::HotMutable,
         "BELLY-GROUNDING SETPOINT ADAPTATION.  `height_k` is a hand-set fraction of the discovered max clearance, and measurement shows it sits BELOW where the body actually rides (tgt 0.30 vs chassis_h_ema 0.39-0.44), so the height homeostat integrates NEGATIVE and commands hip2 DOWN while the belly is simultaneously grounding (p1 clearance 4mm; 58-64% of the first 200 ticks under 10mm).  When > 0 the setpoint fraction RISES while the belly is grounded and decays back toward height_k when it is not, so the target is discovered from the body's own contact experience instead of asserted.  Acts mainly at rest and during stand-up, where fwd_progress is low and height_rest_frac ~ 1; the measured incline fade is left untouched.  0 = off, byte-identical.",
         ParamValue{0.0}},
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
        {"torque_topic", ParamMutability::ConstructionOnly,
         "Per-servo LOAD ProprioToken topic (reality.proprio.joint_torque): 12 floats, layout hip1[0..3], hip2[0..3], knee[0..3] — joint-major, leg order FL,FR,RL,RR, normalized to MAX_SERVO_TORQUE and one tick delayed. On hardware this is servo current sensing, so it is a legal egocentric observation. Published by the body every tick since 2026-06-01 and, until now, consumed by nothing. Empty = off (byte-identical).",
         std::nullopt, std::nullopt, std::nullopt},
        {"contact_instrument_only", ParamMutability::HotMutable,
         "When >0, a subscribed contact_topic is read as an INSTRUMENT only and does NOT drive the stance/swing gate — the incumbent foot-height detector keeps the control path. Needed because wiring true contact as the swing gate is separately REFUTED (net_z 3.76 -> 2.37: that consumer wanted gait PHASE, not contact), yet the alignment diagnostic needs ground truth. 0 = legacy behaviour.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"stroke_load_gain", ParamMutability::HotMutable,
         "PURCHASE GATE on the power stroke: scale each leg's propulsion by its share of measured hip1 load, so a leg pushes in proportion to the ground it actually has. Requires torque_topic. 0 = off (gate identically 1, byte-identical). The stroke is the one major bias in this stack that is ungated, and Phase 0 measured the cost: the fraction of STANCE spent in the stroke's positive half is 0.512 and over SWING 0.513, i.e. push direction is statistically INDEPENDENT of ground contact and half the power stroke is spent in the air. hip1 is the load signal because Phase 0 measured it to be (stance/swing torque ratio 1.368 hip1, 1.124 hip2, 1.011 knee): hip2 and the knee hold a near-static posture in both phases, while hip1's torque IS the ground reaction to the sweep. The gate is normalized to a mean of 1 across legs, so it REDISTRIBUTES thrust toward the legs with purchase instead of merely attenuating it. It is applied to the propulsion term only, never to the steering term (heading_bearing_hold rides the same channel). Values are an AMPLIFICATION of the ~15% raw load contrast, so useful settings are >1.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{8.0}},
        {"tibia_plumb_gain", ParamMutability::HotMutable,
         "TIBIA-PLUMB REFLEX: hip2 rotates to null the shank's deviation from VERTICAL, so the knee's gait drive TRANSLATES the foot instead of arcing it about the knee axis. 0 = off (byte-identical). hip2 and the knee are a planar 2-link arm; with hip2 pinned at its horizontal rest the knee alone must set both the foot's height AND its fore-aft position, which forces a circular foot path and a large shank sweep. MEASURED (arena, n=3, 1032 leg-frames): hip2 sits at -3.6 +/- 4.4 deg and never leaves neutral, the tibia swings to 37.5 +/- 15.3 deg off vertical (design rest 10, extremes 101), and the feet plant at a 170 mm radius against a 166 mm total leg reach -- straight-legged, maximum moment arm, and scrub 0.100 vs fwd_v 0.050 (sliding sideways twice as fast as it advances). This is the rewrite rule applied to that: implement the ERROR (shank off plumb), not a hip2 trajectory. Distinct from the refuted 'learned hip2', which merely LOOSENED hip2's spring and hoped the HK controller would discover the coordination -- an unconstrained joint is a wobble dimension, not an IK solver; this gives hip2 an objective. Specifies nothing about timing, so the rhythm stays emergent.",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"tibia_plumb_scale", ParamMutability::HotMutable,
         "Kinematic constant, NOT a tuned gain: converts hip2's proprio units to radians. = HIP2_LIMIT = 1.40 for the picrawler (hip2 proprio is published as angle / HIP2_LIMIT).",
         ParamValue{1.40}, ParamValue{0.0}, ParamValue{10.0}},
        {"tibia_plumb_offset", ParamMutability::HotMutable,
         "Kinematic constant, NOT a tuned gain: where 'plumb' sits in the joint encoding. = KNEE_REST + pi/2 = -1.6 + 1.5708 = -0.0292 rad for the picrawler (knee proprio is published as angle - KNEE_REST). Validated against the CAD rest pose, which reads 10 deg off vertical.",
         ParamValue{-0.0292}, ParamValue{-3.2}, ParamValue{3.2}},
        {"swing_tuck_hip2", ParamMutability::HotMutable,
         "SWING TUCK (hip2 half): bias on the femur of legs that are OFF THE GROUND, folding the limb inboard so sweeping it forward stops dumping yaw into the chassis. The MIRROR of stance_lift (which biases the knee of PLANTED legs). Requires contact_topic; 0 = off (byte-identical). This is `hip2_tuck_target` with the gate it was missing: that parameter is an UNGATED postural rest-override applied to every leg all the time and is refuted (\"didn't crouch + destabilized\"), and doctrine §5 says to ask what state should have gated a failed bias before calling the idea dead. hip2's known liability is that it rotates feet OFF THE GROUND and loses traction -- during swing the foot is already off the ground, so the state gate turns that failure mode into the mechanism. Sign is left to the sweep, as stance_lift's was.",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"swing_tuck_knee", ParamMutability::HotMutable,
         "SWING TUCK (knee half): bias on the shank of legs that are OFF THE GROUND, folding it under so the foot rides closer to the body through the swing. Pairs with swing_tuck_hip2. Requires contact_topic; 0 = off (byte-identical). Deliberately gated on TRUE contact rather than the foot-height detector stance_lift uses -- that detector was measured firing ~2x per real step, and a tuck on a chattering gate would retract the limb MID-STANCE, i.e. lift a loaded foot, which is the traction-loss failure that made the height homeostat wreck climbing.",
         ParamValue{0.0}, ParamValue{-2.0}, ParamValue{2.0}},
        {"gait_align_diag", ParamMutability::HotMutable,
         "DIAGNOSTIC ONLY (>0 = on, 0 = off and the whole block is skipped, so the build stays byte-identical). Measures whether the propulsive stroke is phase-locked to ground contact at all. The stroke rides L.phase (derived from the KNEE by default) while the stance gate rides the FOOT-HEIGHT cycle; legphase_agree already reads ~0.5 between them. Publishes a phase-locking value accumulated at each touchdown (stroke_td_plv: ~0 = touchdown lands at a uniformly random stroke phase, i.e. the two clocks are unlocked and half the power stroke is spent in the air), the signed continuous alignment, the stance/swing split of the stroke waveform, cycle periods for hip1/knee/foot, and whether joint_torque separates stance from swing. Feeds no command.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
    };
}

ParamMap MotorEPMv2::current_params() const {
    ParamMap m;
    std::vector<std::string> pt(proprio_topics_), at(action_topics_);
    m["proprio_topics"] = pt;
    m["action_topics"]  = at;
    m["objective_topics"] = std::vector<std::string>(objective_topics_);
    m["velocity_objective_topics"] = std::vector<std::string>(velocity_objective_topics_);
    m["n_legs"]     = int64_t(n_legs_);
    m["motor_dim"]  = int64_t(motor_dim_);
    m["model_lr"]   = model_lr_;
    m["ctrl_lr"]    = ctrl_lr_;
    m["bias_lr"]    = bias_lr_;
    m["reg_eps"]    = reg_eps_;
    m["max_dctrl"]  = max_dctrl_;
    m["init_scale"]   = init_scale_;
    m["c_init"]       = c_init_;
    m["whole_body_c"] = whole_body_c_;
    m["sense"]        = sense_;
    m["dep_gain"]     = dep_gain_;
    m["dep_alpha"]    = dep_alpha_;
    m["ctrl_damping"] = ctrl_damping_;
    m["cmd_squash"]   = cmd_squash_;
    m["seed"]         = base_seed_;
    m["babble_ticks"]  = babble_ticks_;
    m["babble_scale"]  = babble_scale_;
    m["sat_lr"]        = sat_lr_;
    m["postural_gain"]    = postural_gain_;
    m["postural_gain_joints"] = postural_gain_joints_;
    m["explore_noise"]    = explore_noise_;
    m["knee_tuck_target"] = knee_tuck_target_;
    m["hip2_tuck_target"] = hip2_tuck_target_;
    m["motor_gain"]       = motor_gain_;
    m["coupling_gain"]    = coupling_gain_;
    m["intent_topic"] = ParamValue{intent_topic_};
    m["explore_floor"] = explore_floor_;
    m["commit_prec_gain"] = commit_prec_gain_;
    m["intent_yaw_gain"] = intent_yaw_gain_;
    m["lookahead_gain"] = lookahead_gain_;
    m["lookahead_mode"] = lookahead_mode_;
    m["lookahead_null"] = lookahead_null_;
    m["intent_rhythm_gain"] = intent_rhythm_gain_;
    m["fwd_resonance_gain"] = fwd_resonance_gain_;
    m["phase_vel_smooth"] = phase_vel_smooth_;
    m["phase_sym_smooth"] = phase_sym_smooth_;
    m["commit_window_ticks"] = commit_window_ticks_;
    m["commit_rise_ticks"] = commit_rise_ticks_;
    m["commit_decay_ticks"] = commit_decay_ticks_;
    m["heading_trim_rate"] = heading_trim_rate_;
    m["heading_trim_leak"] = heading_trim_leak_;
    m["c_pair_init"] = c_pair_init_;
    m["goal_bearing_topic"] = ParamValue{goal_bearing_topic_};
    m["couple_prec_gain"] = couple_prec_gain_;
    m["phase_joint"]      = int64_t(phase_joint_);
    m["rhythm_gains"]     = rhythm_gains_;
    m["rhythm_offsets"]   = rhythm_offsets_;
    m["cpg_phase_topic"]  = cpg_phase_topic_;
    m["cpg_embed"]        = cpg_embed_;
    m["embed_lr"]         = embed_lr_;
    m["embed_decay"]      = embed_decay_;
    m["ctrl_symmetry_gain"] = ctrl_symmetry_gain_;
    { std::vector<double> sg(symmetry_group_of_.begin(), symmetry_group_of_.end()); m["symmetry_group_of"] = sg; }
    m["rhythm_fade_start"]= rhythm_fade_start_;
    m["rhythm_fade_end"]  = rhythm_fade_end_;
    m["coupling_fade_start"] = coupling_fade_start_;
    m["coupling_fade_end"]   = coupling_fade_end_;
    m["gait_phase"]       = gait_phase_;
    m["coord_adapt_rate"] = coord_adapt_rate_;
    m["coord_explore"]    = coord_explore_;
    m["coord_reward_drive"] = coord_reward_drive_;
    m["stuck_explore_gain"] = stuck_explore_gain_;
    m["progress_commit_gain"] = progress_commit_gain_;
    m["forward_flow_gain"]  = forward_flow_gain_;
    m["stance_lift_gain"]   = stance_lift_gain_;
    m["stance_release_frac"] = stance_release_frac_;
    m["swing_hyst_frac"]    = swing_hyst_frac_;
    m["coord_fitness_mode"] = int64_t(coord_fitness_mode_);
    m["homeo_upright_gate"] = homeo_upright_gate_;
    m["homeo_leak_cycles"] = homeo_leak_cycles_;
    m["homeo_leak_progress_gate"] = homeo_leak_progress_gate_;
    m["homeo_leak_upright_only"]  = homeo_leak_upright_only_;
    m["rhythm_topic"]      = rhythm_topic_;
    m["height_unwind_free"] = height_unwind_free_;
    m["coord_probe_ticks"]  = coord_probe_ticks_;
    m["coord_stab_penalty"] = coord_stab_penalty_;
    m["coord_lat_penalty"]  = coord_lat_penalty_;
    m["coord_intent_nav"]   = coord_intent_nav_;
    m["cruse_gain"]         = cruse_gain_;
    m["cruse_rule3_weight"] = cruse_rule3_weight_;
    m["cruse_rule2_window"] = cruse_rule2_window_;
    m["cruse_rule5_gain"]   = cruse_rule5_gain_;
    m["feet_topic"]         = feet_topic_;
    m["contact_topic"]      = contact_topic_;
    m["torque_topic"]       = torque_topic_;
    m["contact_instrument_only"] = contact_instrument_only_;
    m["gait_align_diag"]    = gait_align_diag_;
    m["stroke_load_gain"]   = stroke_load_gain_;
    m["tibia_plumb_gain"]   = tibia_plumb_gain_;
    m["tibia_plumb_scale"]  = tibia_plumb_scale_;
    m["tibia_plumb_offset"] = tibia_plumb_offset_;
    m["swing_tuck_hip2"]    = swing_tuck_hip2_;
    m["swing_tuck_knee"]    = swing_tuck_knee_;
    m["upright_topic"]      = upright_topic_;
    m["stroke_gain"]      = stroke_gain_;
    m["stroke_phase"]     = stroke_phase_;
    m["stroke_phase_src"]    = stroke_phase_src_;
    m["step_phase_debounce"] = step_phase_debounce_;
    m["step_period_alpha"]   = step_period_alpha_;
    m["step_period_min"]     = step_period_min_;
    m["step_period_max"]     = step_period_max_;
    m["step_phase_lock"]     = step_phase_lock_;
    m["gait_raster_diag"]    = gait_raster_diag_;
    m["steer"]            = steer_;
    m["stroke_signs"]     = stroke_signs_;
    m["propulsion_balance_gain"] = propulsion_balance_gain_;
    m["balance_gain"]     = balance_gain_;
    m["tilt_topic"]       = tilt_topic_;
    m["amp_homeo_gain"]   = amp_homeo_gain_;
    m["amp_target"]       = amp_target_;
    m["amp_seek_rate"]    = amp_seek_rate_;
    m["amp_seek_ticks"]   = amp_seek_ticks_;
    m["heading_gain"]     = heading_gain_;
    m["heading_hold_gain"] = heading_hold_gain_;
    m["heading_bearing_hold_gain"] = heading_bearing_hold_gain_;
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

void MotorEPMv2::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("MotorEPMv2::on_setup: null bus");

    apply_param(params, "n_legs",     [&](auto const& v){ n_legs_     = int(get_int(v, "n_legs")); });
    apply_param(params, "motor_dim",  [&](auto const& v){ motor_dim_  = int(get_int(v, "motor_dim")); });
    apply_param(params, "model_lr",   [&](auto const& v){ model_lr_   = get_double(v, "model_lr"); });
    apply_param(params, "ctrl_lr",    [&](auto const& v){ ctrl_lr_    = get_double(v, "ctrl_lr"); });
    apply_param(params, "bias_lr",    [&](auto const& v){ bias_lr_    = get_double(v, "bias_lr"); });
    apply_param(params, "reg_eps",    [&](auto const& v){ reg_eps_    = get_double(v, "reg_eps"); });
    apply_param(params, "max_dctrl",  [&](auto const& v){ max_dctrl_  = get_double(v, "max_dctrl"); });
    apply_param(params, "init_scale", [&](auto const& v){ init_scale_ = get_double(v, "init_scale"); });
    apply_param(params, "c_init",     [&](auto const& v){ c_init_     = get_double(v, "c_init"); });
    apply_param(params, "whole_body_c", [&](auto const& v){ whole_body_c_ = get_double(v, "whole_body_c"); });
    apply_param(params, "sense",        [&](auto const& v){ sense_        = get_double(v, "sense"); });
    apply_param(params, "dep_gain",     [&](auto const& v){ dep_gain_     = get_double(v, "dep_gain"); });
    apply_param(params, "dep_alpha",    [&](auto const& v){ dep_alpha_    = get_double(v, "dep_alpha"); });
    apply_param(params, "ctrl_damping", [&](auto const& v){ ctrl_damping_ = get_double(v, "ctrl_damping"); });
    apply_param(params, "cmd_squash", [&](auto const& v){ cmd_squash_ = get_double(v, "cmd_squash"); });
    apply_param(params, "seed",       [&](auto const& v){ base_seed_  = get_int(v, "seed"); });
    apply_param(params, "babble_ticks", [&](auto const& v){ babble_ticks_ = get_int(v, "babble_ticks"); });
    apply_param(params, "babble_scale", [&](auto const& v){ babble_scale_ = get_double(v, "babble_scale"); });
    apply_param(params, "sat_lr",       [&](auto const& v){ sat_lr_       = get_double(v, "sat_lr"); });
    apply_param(params, "postural_gain", [&](auto const& v){ postural_gain_ = get_double(v, "postural_gain"); });
    apply_param(params, "postural_gain_joints", [&](auto const& v){ postural_gain_joints_ = get_double_vec(v, "postural_gain_joints"); });
    apply_param(params, "explore_noise", [&](auto const& v){ explore_noise_ = get_double(v, "explore_noise"); });
    apply_param(params, "knee_tuck_target", [&](auto const& v){ knee_tuck_target_ = get_double(v, "knee_tuck_target"); });
    apply_param(params, "hip2_tuck_target", [&](auto const& v){ hip2_tuck_target_ = get_double(v, "hip2_tuck_target"); });
    apply_param(params, "motor_gain", [&](auto const& v){ motor_gain_ = get_double(v, "motor_gain"); });
    apply_param(params, "coupling_gain", [&](auto const& v){ coupling_gain_ = get_double(v, "coupling_gain"); });
    apply_param(params, "intent_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) intent_topic_ = *p; });
    apply_param(params, "explore_floor", [&](auto const& v){ explore_floor_ = get_double(v, "explore_floor"); });
    apply_param(params, "commit_prec_gain", [&](auto const& v){ commit_prec_gain_ = get_double(v, "commit_prec_gain"); });
    apply_param(params, "intent_yaw_gain", [&](auto const& v){ intent_yaw_gain_ = get_double(v, "intent_yaw_gain"); });
    apply_param(params, "lookahead_gain", [&](auto const& v){ lookahead_gain_ = get_double(v, "lookahead_gain"); });
    apply_param(params, "lookahead_mode", [&](auto const& v){ lookahead_mode_ = get_double(v, "lookahead_mode"); });
    apply_param(params, "lookahead_null", [&](auto const& v){ lookahead_null_ = get_double(v, "lookahead_null"); });
    apply_param(params, "intent_rhythm_gain", [&](auto const& v){ intent_rhythm_gain_ = get_double(v, "intent_rhythm_gain"); });
    apply_param(params, "fwd_resonance_gain", [&](auto const& v){ fwd_resonance_gain_ = get_double(v, "fwd_resonance_gain"); });
    apply_param(params, "phase_vel_smooth", [&](auto const& v){ phase_vel_smooth_ = get_double(v, "phase_vel_smooth"); });
    apply_param(params, "phase_sym_smooth", [&](auto const& v){ phase_sym_smooth_ = get_double(v, "phase_sym_smooth"); });
    apply_param(params, "commit_window_ticks", [&](auto const& v){ commit_window_ticks_ = get_double(v, "commit_window_ticks"); });
    apply_param(params, "commit_rise_ticks", [&](auto const& v){ commit_rise_ticks_ = get_double(v, "commit_rise_ticks"); });
    apply_param(params, "commit_decay_ticks", [&](auto const& v){ commit_decay_ticks_ = get_double(v, "commit_decay_ticks"); });
    apply_param(params, "heading_trim_rate", [&](auto const& v){ heading_trim_rate_ = get_double(v, "heading_trim_rate"); });
    apply_param(params, "heading_trim_leak", [&](auto const& v){ heading_trim_leak_ = get_double(v, "heading_trim_leak"); });
    apply_param(params, "c_pair_init", [&](auto const& v){ c_pair_init_ = get_double(v, "c_pair_init"); });
    apply_param(params, "goal_bearing_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) goal_bearing_topic_ = *p; });
    apply_param(params, "couple_prec_gain", [&](auto const& v){ couple_prec_gain_ = get_double(v, "couple_prec_gain"); });
    apply_param(params, "phase_joint",   [&](auto const& v){ phase_joint_   = int(get_int(v, "phase_joint")); });
    apply_param(params, "rhythm_gains",   [&](auto const& v){ rhythm_gains_   = get_double_vec(v, "rhythm_gains"); });
    apply_param(params, "rhythm_offsets", [&](auto const& v){ rhythm_offsets_ = get_double_vec(v, "rhythm_offsets"); });
    apply_param(params, "cpg_phase_topic",[&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) cpg_phase_topic_ = *p; });
    apply_param(params, "cpg_embed", [&](auto const& v){
        if (auto p = std::get_if<bool>(&v)) cpg_embed_ = *p;
        else if (auto p = std::get_if<int64_t>(&v)) cpg_embed_ = (*p != 0);
    });
    apply_param(params, "embed_lr",    [&](auto const& v){ embed_lr_    = get_double(v, "embed_lr"); });
    apply_param(params, "embed_decay", [&](auto const& v){ embed_decay_ = get_double(v, "embed_decay"); });
    apply_param(params, "ctrl_symmetry_gain", [&](auto const& v){ ctrl_symmetry_gain_ = get_double(v, "ctrl_symmetry_gain"); });
    apply_param(params, "symmetry_group_of", [&](auto const& v){
        auto d = get_double_vec(v, "symmetry_group_of"); symmetry_group_of_.clear();
        for (double x : d) symmetry_group_of_.push_back(int(std::lround(x)));
    });
    apply_param(params, "rhythm_fade_start", [&](auto const& v){ rhythm_fade_start_ = get_int(v, "rhythm_fade_start"); });
    apply_param(params, "rhythm_fade_end",   [&](auto const& v){ rhythm_fade_end_   = get_int(v, "rhythm_fade_end"); });
    apply_param(params, "coupling_fade_start", [&](auto const& v){ coupling_fade_start_ = get_int(v, "coupling_fade_start"); });
    apply_param(params, "coupling_fade_end",   [&](auto const& v){ coupling_fade_end_   = get_int(v, "coupling_fade_end"); });
    coupling_eff_ = float(coupling_gain_);
    apply_param(params, "gait_phase", [&](auto const& v){ gait_phase_ = get_double_vec(v, "gait_phase"); });
    apply_param(params, "coord_adapt_rate", [&](auto const& v){ coord_adapt_rate_ = get_double(v, "coord_adapt_rate"); });
    apply_param(params, "coord_explore", [&](auto const& v){ coord_explore_ = get_double(v, "coord_explore"); });
    apply_param(params, "coord_reward_drive", [&](auto const& v){ coord_reward_drive_ = get_double(v, "coord_reward_drive"); });
    apply_param(params, "stuck_explore_gain", [&](auto const& v){ stuck_explore_gain_ = get_double(v, "stuck_explore_gain"); });
    apply_param(params, "progress_commit_gain", [&](auto const& v){ progress_commit_gain_ = get_double(v, "progress_commit_gain"); });
    apply_param(params, "forward_flow_gain", [&](auto const& v){ forward_flow_gain_ = get_double(v, "forward_flow_gain"); });
    apply_param(params, "stance_lift_gain", [&](auto const& v){ stance_lift_gain_ = get_double(v, "stance_lift_gain"); });
    apply_param(params, "stance_release_frac", [&](auto const& v){ stance_release_frac_ = get_double(v, "stance_release_frac"); });
    apply_param(params, "swing_hyst_frac", [&](auto const& v){ swing_hyst_frac_ = get_double(v, "swing_hyst_frac"); });
    apply_param(params, "homeo_leak_upright_only", [&](auto const& v){ homeo_leak_upright_only_ = get_double(v, "homeo_leak_upright_only"); });
    apply_param(params, "homeo_leak_progress_gate", [&](auto const& v){ homeo_leak_progress_gate_ = get_double(v, "homeo_leak_progress_gate"); });
    apply_param(params, "homeo_leak_cycles", [&](auto const& v){ homeo_leak_cycles_ = get_double(v, "homeo_leak_cycles"); });
    apply_param(params, "rhythm_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) rhythm_topic_ = *p; });
    apply_param(params, "homeo_upright_gate", [&](auto const& v){ homeo_upright_gate_ = get_double(v, "homeo_upright_gate"); });
    apply_param(params, "height_unwind_free", [&](auto const& v){ height_unwind_free_ = get_double(v, "height_unwind_free"); });
    apply_param(params, "coord_fitness_mode", [&](auto const& v){ coord_fitness_mode_ = int(get_int(v, "coord_fitness_mode")); });
    apply_param(params, "coord_probe_ticks", [&](auto const& v){ coord_probe_ticks_ = get_int(v, "coord_probe_ticks"); });
    apply_param(params, "coord_stab_penalty", [&](auto const& v){ coord_stab_penalty_ = get_double(v, "coord_stab_penalty"); });
    apply_param(params, "coord_lat_penalty", [&](auto const& v){ coord_lat_penalty_ = get_double(v, "coord_lat_penalty"); });
    apply_param(params, "coord_intent_nav", [&](auto const& v){ coord_intent_nav_ = get_double(v, "coord_intent_nav"); });
    apply_param(params, "cruse_gain", [&](auto const& v){ cruse_gain_ = get_double(v, "cruse_gain"); });
    apply_param(params, "cruse_rule3_weight", [&](auto const& v){ cruse_rule3_weight_ = get_double(v, "cruse_rule3_weight"); });
    apply_param(params, "cruse_rule2_window", [&](auto const& v){ cruse_rule2_window_ = get_int(v, "cruse_rule2_window"); });
    apply_param(params, "cruse_rule5_gain", [&](auto const& v){ cruse_rule5_gain_ = get_double(v, "cruse_rule5_gain"); });
    apply_param(params, "feet_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) feet_topic_ = *p; });
    apply_param(params, "upright_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) upright_topic_ = *p; });
    apply_param(params, "contact_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) contact_topic_ = *p; });
    apply_param(params, "torque_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) torque_topic_ = *p; });
    apply_param(params, "contact_instrument_only", [&](auto const& v){ contact_instrument_only_ = get_double(v, "contact_instrument_only"); });
    apply_param(params, "gait_align_diag", [&](auto const& v){ gait_align_diag_ = get_double(v, "gait_align_diag"); });
    apply_param(params, "stroke_load_gain", [&](auto const& v){ stroke_load_gain_ = get_double(v, "stroke_load_gain"); });
    apply_param(params, "tibia_plumb_gain", [&](auto const& v){ tibia_plumb_gain_ = get_double(v, "tibia_plumb_gain"); });
    apply_param(params, "tibia_plumb_scale", [&](auto const& v){ tibia_plumb_scale_ = get_double(v, "tibia_plumb_scale"); });
    apply_param(params, "tibia_plumb_offset", [&](auto const& v){ tibia_plumb_offset_ = get_double(v, "tibia_plumb_offset"); });
    apply_param(params, "swing_tuck_hip2", [&](auto const& v){ swing_tuck_hip2_ = get_double(v, "swing_tuck_hip2"); });
    apply_param(params, "swing_tuck_knee", [&](auto const& v){ swing_tuck_knee_ = get_double(v, "swing_tuck_knee"); });
    apply_param(params, "stroke_gain", [&](auto const& v){ stroke_gain_ = get_double(v, "stroke_gain"); });
    apply_param(params, "stroke_phase", [&](auto const& v){ stroke_phase_ = get_double(v, "stroke_phase"); });
    apply_param(params, "stroke_phase_src",    [&](auto const& v){ stroke_phase_src_    = get_double(v, "stroke_phase_src"); });
    apply_param(params, "step_phase_debounce", [&](auto const& v){ step_phase_debounce_ = get_double(v, "step_phase_debounce"); });
    apply_param(params, "step_period_alpha",   [&](auto const& v){ step_period_alpha_   = get_double(v, "step_period_alpha"); });
    apply_param(params, "step_period_min",     [&](auto const& v){ step_period_min_     = get_double(v, "step_period_min"); });
    apply_param(params, "step_period_max",     [&](auto const& v){ step_period_max_     = get_double(v, "step_period_max"); });
    apply_param(params, "step_phase_lock",    [&](auto const& v){ step_phase_lock_     = get_double(v, "step_phase_lock"); });
    apply_param(params, "gait_raster_diag",    [&](auto const& v){ gait_raster_diag_    = get_double(v, "gait_raster_diag"); });
    apply_param(params, "steer", [&](auto const& v){ steer_ = get_double(v, "steer"); });
    apply_param(params, "stroke_signs", [&](auto const& v){ stroke_signs_ = get_double_vec(v, "stroke_signs"); });
    apply_param(params, "propulsion_balance_gain", [&](auto const& v){ propulsion_balance_gain_ = get_double(v, "propulsion_balance_gain"); });
    apply_param(params, "balance_gain", [&](auto const& v){ balance_gain_ = get_double(v, "balance_gain"); });
    apply_param(params, "tilt_topic", [&](auto const& v){ if (auto p = std::get_if<std::string>(&v)) tilt_topic_ = *p; });
    apply_param(params, "amp_homeo_gain", [&](auto const& v){ amp_homeo_gain_ = get_double(v, "amp_homeo_gain"); });
    apply_param(params, "amp_target", [&](auto const& v){ amp_target_ = get_double(v, "amp_target"); });
    apply_param(params, "amp_seek_rate", [&](auto const& v){ amp_seek_rate_ = get_double(v, "amp_seek_rate"); });
    apply_param(params, "amp_seek_ticks", [&](auto const& v){ amp_seek_ticks_ = get_int(v, "amp_seek_ticks"); });
    apply_param(params, "heading_gain", [&](auto const& v){ heading_gain_ = get_double(v, "heading_gain"); });
    apply_param(params, "heading_hold_gain", [&](auto const& v){ heading_hold_gain_ = get_double(v, "heading_hold_gain"); });
    apply_param(params, "heading_bearing_hold_gain", [&](auto const& v){ heading_bearing_hold_gain_ = get_double(v, "heading_bearing_hold_gain"); });
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
    apply_param(params, "height_ground_gain", [&](auto const& v){ height_ground_gain_ = get_double(v, "height_ground_gain"); });
    apply_param(params, "height_lift_knee", [&](auto const& v){ height_lift_knee_ = get_double(v, "height_lift_knee"); });
    apply_param(params, "stance_lift_hip2", [&](auto const& v){ stance_lift_hip2_ = get_double(v, "stance_lift_hip2"); });
    apply_param(params, "support_select_gain", [&](auto const& v){ support_select_gain_ = get_double(v, "support_select_gain"); });
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
    apply_param(params, "objective_topics", [&](auto const& v){ objective_topics_ = get_string_vec(v, "objective_topics"); });
    apply_param(params, "velocity_objective_topics", [&](auto const& v){ velocity_objective_topics_ = get_string_vec(v, "velocity_objective_topics"); });

    if (int(proprio_topics_.size()) != n_legs_)
        throw std::invalid_argument("MotorEPM: proprio_topics length must equal n_legs");
    if (int(action_topics_.size()) != n_legs_ * motor_dim_)
        throw std::invalid_argument("MotorEPM: action_topics length must equal n_legs*motor_dim");
    if (!objective_topics_.empty() && int(objective_topics_.size()) != n_legs_)
        throw std::invalid_argument("MotorEPM: objective_topics length must equal n_legs (or be empty)");
    if (!velocity_objective_topics_.empty() && int(velocity_objective_topics_.size()) != n_legs_)
        throw std::invalid_argument("MotorEPM: velocity_objective_topics length must equal n_legs (or be empty)");
    if (int(gait_phase_.size()) != n_legs_)
        gait_phase_.assign(n_legs_, 0.0);   // fall back to in-phase if mis-sized
    if (int(stroke_signs_.size()) != n_legs_)
        stroke_signs_.assign(n_legs_, 1.0);

    legs_.assign(n_legs_, Leg{});
    obj_target_.assign(n_legs_, Eigen::VectorXf());
    obj_weight_.assign(n_legs_, 0.0f);
    sat_clip_leg_.assign(n_legs_, 0.0);
    sat_pre_leg_.assign(n_legs_, 0.0);
    obj_seen_.assign(n_legs_, 0);
    obj_vel_target_.assign(n_legs_, Eigen::VectorXf());
    obj_vel_weight_.assign(n_legs_, 0.0f);
    obj_vel_seen_.assign(n_legs_, 0);

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
    if (!upright_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            upright_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_upright(p); }));
    }
    if (!rhythm_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            rhythm_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_rhythm(p); }));
    }
    if (!contact_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            contact_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_contact(p); }));
    }
    if (!torque_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(
            torque_topic_, SubscriptionKind::Direct,
            [this](std::string_view /*topic*/, MessagePtr p){ handle_torque(p); }));
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
    // Objective socket (L-1b, §1.1) — optional per-leg soft posture targets.  Feedback:
    // a top-down objective that may lag a tick (like EPM's descending prediction); an
    // empty list means no subscription → byte-identical HK.
    for (int leg = 0; leg < int(objective_topics_.size()) && leg < n_legs_; ++leg) {
        if (objective_topics_[leg].empty()) continue;
        sub_ids_.push_back(bus_->subscribe(
            objective_topics_[leg], SubscriptionKind::Feedback,
            [this, leg](std::string_view /*topic*/, MessagePtr p){ handle_objective(leg, p); }));
    }
    // Velocity objective socket (L-1b, the propulsive push) — optional per-leg soft velocity
    // targets, same Feedback semantics.  Empty list = no subscription → Cvel never trains.
    for (int leg = 0; leg < int(velocity_objective_topics_.size()) && leg < n_legs_; ++leg) {
        if (velocity_objective_topics_[leg].empty()) continue;
        sub_ids_.push_back(bus_->subscribe(
            velocity_objective_topics_[leg], SubscriptionKind::Feedback,
            [this, leg](std::string_view /*topic*/, MessagePtr p){ handle_objective_vel(leg, p); }));
    }
    // Coherent-scaffold phase (L-1b): an optional global CPG phase (rhythm.cpg.body,
    // ProprioToken [cos φ, sin φ]) to drive the per-joint rhythm from — a CLEAN entrained phase,
    // so all joints lock to ONE frequency (intra-leg coherence) vs the noisy proprio L.phase.
    if (!intent_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(intent_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_intent(p); }));
    }
        if (!goal_bearing_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(goal_bearing_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_goal_bearing(p); }));
    }
    if (!cpg_phase_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(
            cpg_phase_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_cpg_phase(p); }));
}

// The body's own measured gait rhythm (BodyRhythmTracker: [cos, sin, omega rad/tick]).
// Only omega is used here, to express the homeostat LEAK as a number of STRIDE CYCLES
// rather than a tuned tick constant — the leak rate then tracks whatever frequency the body
// actually settles at (CLAUDE.md §5.5: adapt it from the system's own dynamics).  Note the
// CPG's own token is resize(2) and carries no omega, which is why this is a separate input.
void MotorEPMv2::handle_rhythm(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p || p->values.size() < 3) return;
    body_omega_ = p->values[2];
}

void MotorEPMv2::handle_cpg_phase(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p || p->values.size() < 2) return;
    cpg_phase_ = std::atan2(p->values[1], p->values[0]);
    cpg_seen_  = true;
}

void MotorEPMv2::handle_event(std::string_view topic, MessagePtr payload) {
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
        heading_bearing_ = 0.0f;   // respawn = new bearing origin (dead-reckoning restart)
        // respawn = fresh stall clock (don't count the post-reset settle as "stuck")
        stuck_ticks_ = 0; stuck_boost_ = 0.0f; fwd_progress_ema_ = 0.0f;
        // respawn = fresh commit clock + cold flow EMAs (levers C/D)
        commit_ticks_ = 0; commit_boost_ = 0.0f;
        flow_ema_ = 0.0f; flow_vol_ema_ = 0.0f;
        // respawn = the step clock must RE-ANCHOR.  A teleport is a discontinuity in the
        // body's contact history: without this the leg keeps `step_locked` with
        // `last_td_tick` pointing at a touchdown from BEFORE the respawn, so the stroke
        // drives off a stale phase all through the post-reset settle.  Exactly the shape
        // the ledger already records once ("MotorEPM's leg-phase/EMA survived
        // fall+respawn -> any trend across a reset was fake").  Dropping to unlocked
        // returns the stroke to L.phase until two real touchdowns are seen again, which
        // is the same safe fallback a cold start takes.
        for (auto& L : legs_) {
            L.last_td_tick = -1; L.td_count = 0; L.td_run = 0; L.td_cand_tick = -1;
            L.step_locked  = false; L.td_contact = true;
            L.step_phase   = 0.0f;  L.step_omega = 0.0f;
            // step_per_ema is DELIBERATELY kept: the body's stride period is a property
            // of the morphology and gait, not of the episode, so re-measuring it from
            // scratch after every stumble would throw away good information.
        }
    }
}

void MotorEPMv2::handle_objective(int leg, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const PredictionToken>(payload);
    if (!pt || leg < 0 || leg >= n_legs_) return;
    obj_target_[leg] = pt->predicted_latent;                    // motor_dim target positions
    obj_weight_[leg] = std::clamp(pt->confidence, 0.0f, 1.0f);  // w ∈ [0,1]
    obj_seen_[leg]   = 1;
}

void MotorEPMv2::handle_objective_vel(int leg, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const PredictionToken>(payload);
    if (!pt || leg < 0 || leg >= n_legs_) return;
    obj_vel_target_[leg] = pt->predicted_latent;                    // motor_dim target velocities
    obj_vel_weight_[leg] = std::clamp(pt->confidence, 0.0f, 1.0f);  // w ∈ [0,1]
    obj_vel_seen_[leg]   = 1;
}

// Uprightness straight from the body's dedicated `upright` topic (basis.y.y): +1 upright,
// 0 on its side, −1 inverted.  Accelerometer-derivable, so legal.
//
// This exists because deriving it from `tilt` DOES NOT WORK headless: the body's
// publish_tilt @export defaults FALSE, so the tilt topic never arrives and upright_ sat at
// its 1.0 init — the homeostat gate was silently DEAD CODE, verified by amp_gain winding
// identically with the gate on and off.  The `upright` topic is published unconditionally.
void MotorEPMv2::handle_upright(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    upright_ = pt->values[0];
    have_upright_ = true;
}

void MotorEPMv2::handle_tilt(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 4) return;
    tilt_pitch_ = pt->values[0];   // sin(pitch) — signed fore-aft tilt
    tilt_roll_  = pt->values[2];   // sin(roll)  — signed left-right tilt
    // The cos components were RECEIVED and DISCARDED, which left this module unable to
    // distinguish upright from inverted at all — sin(180°) == sin(0°) == 0.  Their product
    // is ~basis.y.y: +1 upright, 0 on its side, −1 on its back.  Accelerometer-derived, so
    // legal.  Needed by the homeostat upright gate below.
    if (!have_upright_) upright_ = pt->values[1] * pt->values[3];  // fallback only
}

void MotorEPMv2::handle_imu(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 4) return;
    fwd_v_    = pt->values[2];     // forward velocity (controllability/thrust signal)
    yaw_rate_ = pt->values[3];     // signed yaw rate (ang_v / π, clamped)
    yaw_rate_ema_ = (1.0f - kYawRateEmaAlpha) * yaw_rate_ema_ + kYawRateEmaAlpha * yaw_rate_;  // heading-hold
    // Dead-reckoned bearing (bearing-hold): integrate our own yaw rate → heading relative
    // to spawn.  Compliant (a real gyro does this); clamped so a wild spin can't explode
    // the correction term.  Zeroed on respawn in handle_event (new spawn = new origin).
    heading_bearing_ = std::clamp(heading_bearing_ + yaw_rate_ * kBearingIntegDt,
                                  -kBearingClamp, kBearingClamp);
}

// L1 nav setpoint.  An EGOCENTRIC unit vector [vx, vy] (vy = forward) from the nav layer.
// Motor intent from the higher loop: [v_forward*, yaw_rate*].
void MotorEPMv2::handle_intent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 2) return;
    intent_v_ = pt->values[0];
    intent_w_ = pt->values[1];
    intent_seen_ = true;
    ++intent_msgs_;
}

void MotorEPMv2::handle_goal_bearing(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 2) return;
    gb_x_ = pt->values[0];
    gb_y_ = pt->values[1];
    gb_seen_ = true;
    ++gb_msgs_;
}

void MotorEPMv2::handle_nav(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 2) return;
    tc_x_ = pt->values[0];
    tc_y_ = pt->values[1];
}

void MotorEPMv2::handle_cog_steer(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    cog_steer_ = std::clamp(a->accel / 4.0f, -1.0f, 1.0f);   // normalize accel∈[-4,4] → [-1,1]
    ++cog_steer_msgs_;
}

void MotorEPMv2::handle_cog_thrust(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    cog_thrust_ = std::clamp(a->accel / 4.0f, -1.0f, 1.0f);   // normalize accel∈[-4,4] → [-1,1]
    ++cog_thrust_msgs_;
}

void MotorEPMv2::handle_boredom(MessagePtr payload) {
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

void MotorEPMv2::handle_interest(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto g = std::dynamic_pointer_cast<const ReflexGate>(payload);
    if (!g) return;
    interest_ = g->active ? std::clamp(g->value, 0.0f, 1.0f) : 0.0f;
}

void MotorEPMv2::handle_hunger(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    hunger_ = std::clamp(pt->values[0], 0.0f, 1.0f);   // 1-energy: 0 sated → 1 starving
}

void MotorEPMv2::handle_height(MessagePtr payload) {
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

void MotorEPMv2::handle_distress(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    distress_ = pt->values[0];
}

void MotorEPMv2::handle_lateral(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 1) return;
    lateral_v_ = pt->values[0];   // signed sideways-slip velocity (+ = body-right)
}

void MotorEPMv2::handle_feet(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int k = std::min<int>(int(pt->values.size()), n_legs_);
    if (int(foot_y_.size()) != n_legs_) foot_y_.assign(n_legs_, 0.0f);
    for (int i = 0; i < k; ++i) foot_y_[i] = float(pt->values[i]);
}

// TRUE per-leg ground contact (`reality.proprio.foot_contact`) — the physics touch flag,
// which is also the sensor a REAL picrawler has (a foot switch).  It was already being
// published every tick and simply was not wired here: MotorEPM instead INFERRED contact
// from foot HEIGHT relative to that foot's own moving average, which measured 40.3 %
// "swinging" while the feet were genuinely down 99.3 % of the time.  Every consumer of
// in_swing_ (stance_lift, all the Cruse rules) was gating on that error.
void MotorEPMv2::handle_contact(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    int k = std::min<int>(int(pt->values.size()), n_legs_);
    if (int(foot_contact_.size()) != n_legs_) foot_contact_.assign(n_legs_, 1.0f);
    for (int i = 0; i < k; ++i) foot_contact_[i] = float(pt->values[i]);
    have_contact_ = true;
}

// Per-servo LOAD (`reality.proprio.joint_torque`).  12 floats, joint-major:
// hip1[FL,FR,RL,RR], hip2[FL,FR,RL,RR], knee[FL,FR,RL,RR], each normalized to
// MAX_SERVO_TORQUE and one tick delayed (the body's motor block runs after perception).
// This is the load observation the Cruse/Walknet rules never had — on hardware it is
// servo current sensing, so it is legal under the Markov-blanket rule.
void MotorEPMv2::handle_torque(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    const int need = n_legs_ * motor_dim_;
    if (int(joint_torque_.size()) != need) joint_torque_.assign(need, 0.0f);
    int k = std::min<int>(int(pt->values.size()), need);
    for (int i = 0; i < k; ++i) joint_torque_[i] = float(pt->values[i]);
    have_torque_ = true;
}

// Total |load| on one leg, from the joint-major torque vector.  Sums all three servos:
// hip2 and knee carry the vertical load, hip1 the fore-aft reaction, and a leg bearing
// weight loads all of them relative to one waving in the air.
float MotorEPMv2::leg_load(int leg) const {
    if (!have_torque_ || int(joint_torque_.size()) != n_legs_ * motor_dim_) return 0.0f;
    float s = 0.0f;
    for (int j = 0; j < motor_dim_; ++j) s += std::fabs(joint_torque_[j * n_legs_ + leg]);
    return s;
}

// ---------------------------------------------------------------------------
// STROKE-TO-STEP LOCK — the per-leg touchdown-referenced step clock.
//
//   phi = 2*pi * (tick - last_touchdown) / EMA(inter-touchdown interval),  wrapped
//
// phi = 0 AT touchdown, so `stroke_phase` selects where in the STEP the push lands
// rather than where in the knee's own faster flexion cycle it lands.  Ported from
// SynergyTimer.cpp:305-313, which has run this clock for a long time; that module is not
// in the picrawler graph, so the algorithm moves and the module does not.
//
// Three properties the header argues for, made explicit here:
//   * NOT SELF-EXCITED.  The stroke reaches this reference only through the world (the
//     foot leaves the ground and comes back).  `phase_joint=0` failed because the stroke
//     drove the same joint an atan2 read its phase from — an algebraic loop closed inside
//     one tick.  This one is closed by physics, which is the loop a walker actually has.
//   * IT FREE-RUNS, IT DOES NOT STALL.  Between touchdowns phi extrapolates on the
//     measured period, so a leg mid-swing has a well-defined phase.  A leg that stops
//     stepping keeps advancing rather than freezing at whatever phase it died on —
//     freezing would silently convert the stroke into a DC bias, which is the refuted
//     "blind knee bias kills the gait" shape.
//   * IT FAILS BACK, NOT OPEN.  Before two touchdowns are seen (`step_locked`) the stroke
//     keeps using the legacy L.phase, so warmup and any non-stepping leg are unchanged.
//
// DEBOUNCE: a touchdown counts only after `step_phase_debounce` consecutive ticks of the
// new contact state.  Contact is a physics touch flag and can flicker on impact; a
// bounce would otherwise reset phi twice per step and reintroduce exactly the chatter
// that makes the incumbent foot-height detector unusable (12-15 ticks against a 26-30
// tick step).  The interval is then clamped to [step_period_min, step_period_max] before
// it enters the EMA, so one anomalous stride cannot drag the period estimate.
// ---------------------------------------------------------------------------
void MotorEPMv2::update_step_phase(uint64_t tick_id) {
    if (int(legs_.size()) != n_legs_) return;
    const int m = motor_dim_;
    // Source 2 needs a per-leg load mean to threshold against; source 1 needs nothing.
    const bool src_load = (stroke_phase_src_ >= 1.5);
    if (src_load && int(step_load_ema_.size()) != n_legs_) step_load_ema_.assign(n_legs_, 0.0f);
    const bool have_con = have_contact_ && int(foot_contact_.size()) == n_legs_;
    const bool have_tq  = have_torque_  && int(joint_torque_.size()) == n_legs_ * m;
    const int  debounce = std::max(1, int(step_phase_debounce_));

    int   locked_n = 0;
    float per_sum  = 0.0f; int per_n = 0;
    for (int i = 0; i < n_legs_; ++i) {
        Leg& L = legs_[i];
        if (!L.initialized) continue;

        // ---- is this foot down, by the selected source?
        bool td_now = false;                 // an accepted touchdown landed THIS tick
        bool con_now;
        if (src_load) {
            // Touchdown = hip1 |torque| rising through its own slow mean.  hip1 is the
            // joint the ledger's measurement picked (stance/swing ratio 1.368 vs 1.124
            // hip2 / 1.011 knee) — its torque is the ground reaction to the sweep itself.
            // NOTE the measured ceiling on this source: tq_agree is 0.540 against a 0.5
            // chance line, so this detector is RIGHT ABOUT HALF THE TIME.  It exists
            // because it is buildable on the real robot (servo current sensing) where a
            // foot switch is not; it is not expected to match source 1.
            if (!have_tq) continue;
            const float ld = std::fabs(joint_torque_[0 * n_legs_ + i]);   // hip1, joint-major
            con_now = (ld > step_load_ema_[i]);
            step_load_ema_[i] = (1.0f - kStrokeLoadAlpha) * step_load_ema_[i]
                              + kStrokeLoadAlpha * ld;
        } else {
            if (!have_con) continue;
            con_now = (foot_contact_[i] > 0.5f);
        }

        // ---- REFRACTORY debounce: trigger on the RAW rising edge, then ignore further
        // edges for `debounce` ticks.
        //
        // The first version confirmed an edge by requiring N consecutive differing ticks
        // before accepting it.  That is a delay-AND-MERGE filter, and it biased the period
        // estimate badly: contacts shorter than N were swallowed entirely, so the accepted
        // touchdowns were a SUBSET of the real ones and the measured interval read long.
        // Measured: `step_period` 23.6 against a raw contact period of 19.1 — a 24 %
        // frequency error, which no phase pull can close (the loop-gain sweep confirmed it:
        // td_plv stayed at 0.04-0.10 from gain 0.1 to 0.7 while the true phase error sat
        // flat at ~1.55 rad).
        //
        // A refractory window fixes both faults at once: the ANCHOR is the raw edge, so
        // touchdown timing is unbiased, and re-triggers within the window are dropped, so
        // contact bounce still cannot reset the phase twice per step.  This is the standard
        // shape for debouncing a real switch, and it is the shape a foot switch on the
        // physical robot would need.
        // CANDIDATE -> CONFIRM -> BACK-DATE.  Three properties are needed at once and the
        // first two versions each had only one:
        //   * reject a BRIEF spurious touch (a foot brushing something mid-swing must not
        //     count as a footfall) -- the confirm-N-ticks version did this;
        //   * do not BIAS the touchdown time, since that time is the phase anchor -- the
        //     refractory version did this;
        //   * never MERGE two real footfalls into one, which is what silently corrupted the
        //     frequency estimate: confirm-N flipped its state only after N consecutive
        //     ticks, so a contact shorter than N was swallowed whole AND the next rising
        //     edge was no longer seen as rising. Accepted touchdowns became a subset of the
        //     real ones and the measured period read long -- 23.6 ticks against a raw
        //     contact period of 19.1, a 24 % frequency error that no phase pull can close
        //     (measured: td_plv stuck at 0.04-0.10 across loop gains 0.1..0.7).
        //
        // The fix keeps `td_contact` tracking RAW contact, so every real rising edge starts
        // a candidate; the candidate is only ACCEPTED after `debounce` ticks of sustained
        // contact, and is then timed at the edge that started it.  Brief touches are
        // dropped, timing is exact, and nothing is ever merged.
        const bool rising = (con_now && !L.td_contact);
        L.td_contact = con_now;
        if (rising && L.td_cand_tick < 0) {              // start a candidate footfall
            L.td_cand_tick = int64_t(tick_id);
            L.td_run = 0;
        } else if (L.td_cand_tick >= 0 && !con_now) {    // contact broke before confirming
            L.td_cand_tick = -1;                         // -> it was a brush, not a footfall
            L.td_run = 0;
        }
        {
            if (L.td_cand_tick >= 0 && con_now && ++L.td_run >= debounce) {
                const int64_t td_tick = L.td_cand_tick;  // BACK-DATE to the raw edge
                L.td_cand_tick = -1;
                L.td_run = 0;
                if (L.last_td_tick >= 0) {
                    float d = float(td_tick - L.last_td_tick);
                    d = std::clamp(d, float(step_period_min_), float(step_period_max_));
                    L.step_per_ema = (L.step_per_ema <= 0.0f)
                                   ? d
                                   : (1.0f - float(step_period_alpha_)) * L.step_per_ema
                                     + float(step_period_alpha_) * d;
                    if (L.td_count < 1000000) ++L.td_count;
                }
                L.last_td_tick = td_tick;
                if (L.td_count < 1) L.td_count = 1;      // first touchdown: no interval yet
                td_now = true;
            }
        }

        // ---- advance.  Locked once an interval has been measured at least twice, i.e.
        // the period estimate is a real measurement rather than one sample.
        const bool was_locked = L.step_locked;
        L.step_locked = (L.td_count >= 2 && L.step_per_ema > 0.0f && L.last_td_tick >= 0);
        if (was_locked != L.step_locked) ++step_lock_flips_;
        if (L.step_locked) {
            // A PHASE-LOCKED LOOP, NOT A RESET — and the difference is the whole lever.
            //
            // The first build of this snapped `step_phase = 0` at every touchdown.  It
            // collapsed the gait: the body convulsed in place, inverted repeatedly, and
            // netted -0.16 m (against a 4.58 m baseline).  TWO mechanisms, both traceable
            // to the snap:
            //   * POSITIVE FEEDBACK.  The stroke can CAUSE touchdowns, and a touchdown
            //     reset the phase to the point of maximum push -- so a push bounced the
            //     foot, the bounce re-triggered the reset, and the period EMA ran down to
            //     its rail.  Closing the loop "through the world" is only safe when the
            //     stroke cannot trigger the phase-setting EVENT; here it could.
            //   * A COMMAND DISCONTINUITY.  `sin(phi + stroke_phase)` is a continuous
            //     motor command, so snapping phi steps the command every time a foot
            //     lands off-schedule -- and off-schedule is exactly what an unlocked gait
            //     does.  Irregular footfall then injected impulses, which made footfall
            //     more irregular.
            //
            // BodyRhythmTracker already solved this, and its header says so: "PHASE is a
            // pure integrator phi += omega, SOFTLY PULLED to the reference at each
            // up-crossing.  Feed-forward frequency + feedback phase lock."  That module's
            // clock DRIVES the CPG; SynergyTimer's clock (which this was ported from, and
            // which does snap) only INDEXES a discrete phase bin, where a snap is
            // harmless.  The generalizable rule: a phase that DRIVES a continuous command
            // needs a soft pull; a phase that is merely READ can take a reset.
            //
            // step_phase_lock = 1.0 reproduces the hard snap exactly, so the refuted
            // form stays reachable and its measurement reproducible.
            float omega_target = kTwoPi / L.step_per_ema;
            const float w_hi = kTwoPi / std::max(2.0f, float(step_period_min_));
            const float w_lo = kTwoPi / std::max(2.0f, float(step_period_max_));
            omega_target = std::clamp(omega_target, w_lo, w_hi);
            L.step_omega = (L.step_omega <= 0.0f)
                         ? omega_target
                         : L.step_omega + kStepOmegaLp * (omega_target - L.step_omega);
            L.step_omega = std::clamp(L.step_omega, w_lo, w_hi);
            L.step_phase += L.step_omega;                     // integrate: smooth by construction
            if (td_now) {                                     // proportional pull toward phi_ref = 0
                const float err = (L.step_phase > kTwoPi * 0.5f)
                                ? (kTwoPi - L.step_phase) : (-L.step_phase);
                L.step_phase += float(step_phase_lock_) * err;
            }
            L.step_phase = std::fmod(L.step_phase, kTwoPi);
            if (L.step_phase < 0.0f) L.step_phase += kTwoPi;
            ++locked_n;
            per_sum += L.step_per_ema; ++per_n;
            // Phase error at touchdown, averaged: how far off phi=0 the foot is landing.
            // This is the honest lock-quality read, and unlike td_plv it does NOT become
            // tautological, because the pull is partial -- a perfectly entrained clock
            // drives it to 0, a fighting one does not.
            if (td_now) {
                const float e = (L.step_phase > kTwoPi * 0.5f)
                              ? (kTwoPi - L.step_phase) : L.step_phase;
                step_td_err_acc_ += double(std::fabs(e)); ++step_td_err_n_;
            }
        }
    }
    step_lock_frac_   = n_legs_ ? float(locked_n) / float(n_legs_) : 0.0f;
    step_period_mean_ = per_n   ? per_sum / float(per_n)           : 0.0f;
}

// Per-tick footfall raster for the live inspector.  Pure observation — see the header for
// why this is a ring shipped whole rather than accumulated client-side (DiagPublisher
// throttles to ~30 Hz against a ~52 tick/s brain, which would alias at exactly the
// touchdown edges the picture exists to show).
void MotorEPMv2::update_gait_raster() {
    if (int(raster_.size()) != kRasterLen) {
        raster_.assign(kRasterLen, 0);
        raster_head_ = 0; raster_n_ = 0;
    }
    const int nl = std::min(n_legs_, 4);
    uint16_t w = 0;
    for (int i = 0; i < nl; ++i) {
        Leg const& L = legs_[i];
        if (have_contact_ && int(foot_contact_.size()) == n_legs_ && foot_contact_[i] > 0.5f)
            w |= uint16_t(1u << i);
        // The stroke's sign AS COMMANDED, built exactly as the stroke site builds it —
        // including which phase reference is actually live, so the picture tracks the
        // lever instead of a fixed assumption about it.
        if (L.initialized) {
            const float ph  = (stroke_phase_src_ > 0.0 && L.step_locked) ? L.step_phase : L.phase;
            const float sgn = (int(stroke_signs_.size()) == n_legs_) ? float(stroke_signs_[i]) : 1.0f;
            if (sgn * std::sin(ph + float(stroke_phase_)) > 0.0f)
                w |= uint16_t(1u << (4 + i));
        }
        if (int(in_swing_.size()) == n_legs_ && in_swing_[i])
            w |= uint16_t(1u << (8 + i));
    }
    raster_[raster_head_] = w;
    raster_head_ = (raster_head_ + 1) % kRasterLen;
    ++raster_n_;
}

// Per-leg PURCHASE gate for the power stroke.  See the header for why hip1 is the signal
// (measured: stance/swing load ratio 1.368 hip1 / 1.124 hip2 / 1.011 knee) and why the
// gate is mean-normalized rather than a plain attenuation.
//
//   share_i = load_i / mean_j(load_j)            ≈ 1 when the legs are evenly loaded
//   gate_i  = clamp(1 + g·(share_i − 1), …)      g = 0 ⇒ gate ≡ 1 ⇒ byte-identical
//
// g is an AMPLIFICATION of the load contrast, not a blend, because the raw contrast is
// only ~15 %: at g = 1 the gate would swing ±7 % and the lever could not be evaluated.
void MotorEPMv2::update_stroke_load_gate() {
    if (int(stroke_gate_.size()) != n_legs_) stroke_gate_.assign(n_legs_, 1.0f);
    if (int(stroke_load_ema_.size()) != n_legs_) stroke_load_ema_.assign(n_legs_, 0.0f);
    if (stroke_load_gain_ == 0.0 || !have_torque_
        || int(joint_torque_.size()) != n_legs_ * motor_dim_ || n_legs_ < 1) {
        std::fill(stroke_gate_.begin(), stroke_gate_.end(), 1.0f);
        stroke_gate_mean_ = 1.0f; stroke_gate_spread_ = 0.0f;
        return;
    }
    // hip1 is joint 0, and the torque vector is joint-major: τ[j * n_legs + leg].
    float sum = 0.0f;
    for (int i = 0; i < n_legs_; ++i) {
        const float ld = std::fabs(joint_torque_[0 * n_legs_ + i]);
        stroke_load_ema_[i] = (1.0f - kStrokeLoadAlpha) * stroke_load_ema_[i]
                            + kStrokeLoadAlpha * ld;
        sum += stroke_load_ema_[i];
    }
    const float mean = sum / float(n_legs_);
    if (!(mean > 1e-6f)) {              // no load anywhere (airborne / not yet settled)
        std::fill(stroke_gate_.begin(), stroke_gate_.end(), 1.0f);
        stroke_gate_mean_ = 1.0f; stroke_gate_spread_ = 0.0f;
        return;
    }
    float gsum = 0.0f, gmin = kStrokeGateMax, gmax = kStrokeGateMin;
    for (int i = 0; i < n_legs_; ++i) {
        const float share = stroke_load_ema_[i] / mean;
        const float g = std::clamp(1.0f + float(stroke_load_gain_) * (share - 1.0f),
                                   kStrokeGateMin, kStrokeGateMax);
        stroke_gate_[i] = g;
        gsum += g; gmin = std::min(gmin, g); gmax = std::max(gmax, g);
    }
    stroke_gate_mean_   = gsum / float(n_legs_);
    stroke_gate_spread_ = gmax - gmin;
}

// ---------------------------------------------------------------------------
// PHASE-0 GAIT-ALIGNMENT DIAGNOSTIC — measurement only, no command is touched.
//
// The hypothesis under test: the gait runs on TWO uncorrelated per-leg clocks.
//   * thrust  — the power stroke on hip1, `y[0] += amp·sin(L.phase + stroke_phase)`,
//               where L.phase is derived from the KNEE (`phase_joint` defaults to −1)
//   * support — the stance/swing gate, `foot_y > foot_y_ema`, the FOOT-HEIGHT cycle
// If those are unlocked, a leg pushes backward without regard to whether its foot is
// on the ground: half the power stroke spent in the air, half the return swing spent
// scrubbing.  That would explain the operator's "it is always stumbling" AND the
// ledger's standing unknown that flat speed is pinned across every timing lever tried
// — because every one of those levers adjusted phase BETWEEN legs while the relation
// between thrust and support WITHIN a leg stayed random.
//
// The headline is the phase-locking value at touchdown.  Accumulate e^{iθ} at each
// contact onset, θ = the stroke waveform's phase.  Uniformly-distributed touchdown
// phase ⇒ the vectors cancel ⇒ PLV → 0 ⇒ the clocks are unlocked.  A locked gait gives
// PLV → 1, and the mean angle then says whether `stroke_phase` is merely mis-offset.
// (Running sums rather than EMAs: this is a one-shot measurement and the run mean is
// what we want to read.  They are serialized so a restored clone reports the same.)
// ---------------------------------------------------------------------------
void MotorEPMv2::update_gait_align_diag(uint64_t tick_id) {
    if (int(legs_.size()) != n_legs_ || int(in_swing_.size()) != n_legs_) return;
    const int m = motor_dim_;
    auto ensure = [&](auto& v, auto fill){ if (int(v.size()) != n_legs_) v.assign(n_legs_, fill); };
    ensure(ga_tq_ema_,       0.0f);
    ensure(ga_hip1_ema_,     0.0f);
    ensure(ga_hip1_above_,   char(0));
    ensure(ga_knee_above_,   char(0));
    ensure(ga_prev_contact_, char(1));
    ensure(ga_prev_swing_,   char(0));
    ensure(ga_hip1_last_,    int64_t(-1));
    ensure(ga_knee_last_,    int64_t(-1));
    ensure(ga_foot_last_,    int64_t(-1));
    ensure(ga_con_last_,     int64_t(-1));
    ensure(ga_hip1_per_,     0.0f);
    ensure(ga_knee_per_,     0.0f);
    ensure(ga_foot_per_,     0.0f);
    ensure(ga_con_per_,      0.0f);
    ensure(ga_con_iv_sum_,   0.0);
    ensure(ga_con_iv_sq_,    0.0);
    ensure(ga_con_iv_n_,     int64_t(0));
    ensure(ga_bout_run_,     int32_t(0));
    ensure(ga_bout_state_,   char(1));
    ensure(ga_rs_iv_sum_,    0.0);
    ensure(ga_rs_iv_sq_,     0.0);
    ensure(ga_rs_iv_n_,      int64_t(0));
    ensure(ga_rs_last_,      int64_t(-1));

    const bool truth = have_contact_ && int(foot_contact_.size()) == n_legs_;
    // Yaw disturbance attributed to swinging: is the body being spun BY its own swing legs?
    // Split |yaw rate| by support state so the operator's "the extended rear leg sweeps
    // forward and spins the chassis" is a number, not only a UI impression.
    if (truth) {
        if (int(ga_yaw_leg_.size()) != n_legs_) {
            ga_yaw_leg_.assign(n_legs_, 0.0); ga_yaw_leg_n_.assign(n_legs_, 0);
        }
        if (int(ga_yawd_leg_.size()) != n_legs_) {
            ga_yawd_leg_.assign(n_legs_, 0.0); ga_yawd_leg_n_.assign(n_legs_, 0);
        }
        const double ay = std::fabs(double(yaw_rate_));
        // |Δyaw rate| — the impulse a limb's reaction torque actually produces.  The plain
        // rate is swamped by the steering controller (see the header note).
        const double ad = ga_yaw_prev_init_ ? std::fabs(double(yaw_rate_ - ga_prev_yaw_rate_)) : 0.0;
        const bool have_d = ga_yaw_prev_init_;
        ga_prev_yaw_rate_ = yaw_rate_; ga_yaw_prev_init_ = true;
        int n_air = 0;
        for (int i = 0; i < n_legs_; ++i) {
            if (foot_contact_[i] <= 0.5f) {
                ++n_air;
                ga_yaw_leg_[i] += ay; ++ga_yaw_leg_n_[i];
                if (have_d) { ga_yawd_leg_[i] += ad; ++ga_yawd_leg_n_[i]; }
            }
        }
        if (n_air == 0) {
            ga_yaw_allplant_ += ay; ++ga_yaw_allplant_n_;
            if (have_d) { ga_yawd_allplant_ += ad; ++ga_yawd_allplant_n_; }
        } else {
            ga_yaw_anyswing_ += ay; ++ga_yaw_anyswing_n_;
            if (have_d) { ga_yawd_anyswing_ += ad; ++ga_yawd_anyswing_n_; }
        }
    }

    for (int i = 0; i < n_legs_; ++i) {
        Leg const& L = legs_[i];
        if (!L.initialized || L.n < 3 * m) continue;

        // θ and the SIGNED hip1 contribution, exactly as the stroke site builds it:
        // stroke_signs folds the per-leg push direction in, so `s` is comparable across
        // legs.  `s > 0` is one half of the stroke; which half is propulsive is what the
        // stance/swing split below reports, rather than something assumed here.
        // The phase the stroke ACTUALLY rides this tick — must follow `stroke_phase_src`,
        // exactly as the stroke site selects it.  2026-07-27: this originally hard-coded
        // `L.phase`, so on a step-clock arm the whole alignment diagnostic reported the
        // LEGACY phase's alignment and `td_plv` read 0.29 where the mechanism guaranteed
        // ~1.0.  A verification instrument that does not follow the parameter it is
        // verifying is worse than none: it reads like evidence.  (CLAUDE.md §3.2 rule 5,
        // one level up — not "did the consumer fire" but "is the instrument watching it".)
        const float ph_live = (stroke_phase_src_ > 0.0 && L.step_locked) ? L.step_phase : L.phase;
        const float th  = ph_live + float(stroke_phase_);
        const float sgn = (int(stroke_signs_.size()) == n_legs_) ? float(stroke_signs_[i]) : 1.0f;
        const float s   = sgn * std::sin(th);

        // ---- vs the INCUMBENT detector (always available: it is what stance_lift gates on)
        const bool sw_now = (in_swing_[i] != 0);
        if (ga_prev_swing_[i] && !sw_now) {                 // detector's touchdown
            ga_sd_cos_ += std::cos(th); ga_sd_sin_ += std::sin(th); ++ga_sd_n_;
            if (ga_foot_last_[i] >= 0) {                    // foot-cycle period, free
                const float d = float(int64_t(tick_id) - ga_foot_last_[i]);
                if (d > 2.0f && d < 600.0f)
                    ga_foot_per_[i] = (ga_foot_per_[i] <= 0.0f) ? d
                                    : (1.0f - kGaPerAlpha) * ga_foot_per_[i] + kGaPerAlpha * d;
            }
            ga_foot_last_[i] = int64_t(tick_id);
        }
        ga_prev_swing_[i] = sw_now ? 1 : 0;

        // ---- vs TRUE contact (only when the sensor is wired as an instrument)
        if (truth) {
            const bool con = foot_contact_[i] > 0.5f;
            // Bout bookkeeping: close the run whenever contact flips, and record how long
            // it lasted.  Short bouts on BOTH sides are the signature of chatter.
            if ((ga_bout_state_[i] != 0) != con) {
                const int32_t len = ga_bout_run_[i];
                if (len > 0) {
                    if (ga_bout_state_[i]) { ga_st_bout_sum_ += len; ++ga_st_bout_n_;
                                             if (len < kShortBoutTicks) ++ga_st_bout_short_; }
                    else                   { ga_sw_bout_sum_ += len; ++ga_sw_bout_n_;
                                             if (len < kShortBoutTicks) ++ga_sw_bout_short_; }
                }
                // A REAL STEP is a touchdown whose preceding SWING lasted long enough to
                // be a stride rather than a micro-lift.  Measured on the healthy baseline:
                // ~80 % of lifts last 1-3 ticks, so pooling them with real swings is what
                // drives the raw interval CV to ~1.0.
                if (con && !ga_bout_state_[i] && len >= kRealSwingTicks) {
                    if (ga_rs_last_[i] >= 0) {
                        const double d = double(int64_t(tick_id) - ga_rs_last_[i]);
                        if (d > 2.0 && d < 600.0) {
                            ga_rs_iv_sum_[i] += d; ga_rs_iv_sq_[i] += d * d; ++ga_rs_iv_n_[i];
                        }
                    }
                    ga_rs_last_[i] = int64_t(tick_id);
                }
                ga_bout_state_[i] = con ? 1 : 0;
                ga_bout_run_[i]   = 0;
            }
            ++ga_bout_run_[i];
            ga_contact_acc_ += con ? 1.0 : 0.0; ++ga_contact_n_;
            ga_align_acc_   += double(s) * (con ? 1.0 : -1.0); ++ga_align_n_;
            if (con) { ga_stance_pos_ += (s > 0.0f) ? 1.0 : 0.0; ++ga_stance_n_; }
            else     { ga_swing_pos_  += (s > 0.0f) ? 1.0 : 0.0; ++ga_swing_n_;  }
            // THE HONEST VERSION OF THE SAME SPLIT.  `s` above is the COMMANDED waveform,
            // and once the stroke rides a touchdown-referenced clock its stance/swing
            // split is a deterministic function of stroke_phase and the duty factor —
            // i.e. a tautology, and useless as evidence for the lock (CLAUDE.md §3.2
            // rule 1).  This one uses the leg's ACHIEVED fore-aft motion (sgn·Δhip1),
            // which no amount of re-referencing the command can fake: a working lock
            // means the foot really does travel backward-relative-to-body while planted
            // and forward while airborne.  Accumulated for EVERY instrumented arm,
            // including controls with the lock off — a diag computed inside a lever's own
            // block reads 0 on exactly the arm you need to compare against.
            if (L.n > 2) {
                const double mv = double(sgn) * double(L.x[2]);   // hip1 delta, stroke-signed
                if (con) { ga_mv_stance_ += mv; ++ga_mv_stance_n_; }
                else     { ga_mv_swing_  += mv; ++ga_mv_swing_n_;  }
            }
            if (!ga_prev_contact_[i] && con) {              // TRUE touchdown → the PLV
                ga_td_cos_ += std::cos(th); ga_td_sin_ += std::sin(th); ++ga_td_n_;
                if (ga_con_last_[i] >= 0) {                 // the REAL step period
                    const float d = float(int64_t(tick_id) - ga_con_last_[i]);
                    if (d > 2.0f && d < 600.0f) {
                        ga_con_per_[i] = (ga_con_per_[i] <= 0.0f) ? d
                                       : (1.0f - kGaPerAlpha) * ga_con_per_[i] + kGaPerAlpha * d;
                        // ...and the RAW interval moments, for the regularity CV.
                        ga_con_iv_sum_[i] += double(d);
                        ga_con_iv_sq_[i]  += double(d) * double(d);
                        ++ga_con_iv_n_[i];
                    }
                }
                ga_con_last_[i] = int64_t(tick_id);
            }
            // Does LOAD separate stance from swing?  Threshold-free means first (the
            // honest comparison), then the same self-referential above-its-own-mean test
            // the foot-height detector uses, so the number is directly comparable to
            // legphase_agree.  0.5 = chance = a load lever has nothing to gate on.
            if (have_torque_) {
                const float ld = leg_load(i);
                if (int(ga_tq_j_stance_.size()) != m) { ga_tq_j_stance_.assign(m, 0.0); ga_tq_j_swing_.assign(m, 0.0); }
                if (int(joint_torque_.size()) == n_legs_ * m) {
                    for (int j = 0; j < m; ++j) {
                        const double t = std::fabs(joint_torque_[j * n_legs_ + i]);
                        if (con) ga_tq_j_stance_[j] += t; else ga_tq_j_swing_[j] += t;
                    }
                    if (con) ++ga_tq_j_stance_n_; else ++ga_tq_j_swing_n_;
                }
                if (con) { ga_tq_stance_ += ld; ++ga_tq_stance_n_; }
                else     { ga_tq_swing_  += ld; ++ga_tq_swing_n_;  }
                if (ga_tq_ema_[i] > 0.0f || ld > 0.0f) {
                    ga_tq_agree_ += ((ld > ga_tq_ema_[i]) == con) ? 1.0 : 0.0;
                    ++ga_tq_agree_n_;
                }
                ga_tq_ema_[i] = (1.0f - kGaEmaAlpha) * ga_tq_ema_[i] + kGaEmaAlpha * ld;
                // ...and the same test on hip1 ALONE, which is what a load-derived step
                // clock (stroke_phase_src=2) would actually threshold.  The summed
                // three-servo `leg_load` measures 0.540 agreement — barely off the 0.5
                // chance line — but the per-joint stance/swing ratios say hip1 separates
                // better than the sum (1.368 vs 1.148), so the sum may be diluting the
                // one joint that carries the signal.  Measuring this costs nothing and
                // scopes source 2 BEFORE it is built rather than after a failed A/B.
                if (int(ga_tq_h1_ema_.size()) != n_legs_) ga_tq_h1_ema_.assign(n_legs_, 0.0f);
                if (int(joint_torque_.size()) == n_legs_ * m) {
                    const float h1 = std::fabs(joint_torque_[0 * n_legs_ + i]);
                    if (ga_tq_h1_ema_[i] > 0.0f || h1 > 0.0f) {
                        ga_tq_h1_agree_ += ((h1 > ga_tq_h1_ema_[i]) == con) ? 1.0 : 0.0;
                        ++ga_tq_h1_agree_n_;
                    }
                    ga_tq_h1_ema_[i] = (1.0f - kGaEmaAlpha) * ga_tq_h1_ema_[i] + kGaEmaAlpha * h1;
                }
            }
            ga_prev_contact_[i] = con ? 1 : 0;
        }

        // Shank angle from vertical, for EVERY instrumented arm (not just the ones running
        // the plumb reflex) -- otherwise the control reports 0 and there is nothing to
        // compare the lever against.  theta = HIP2_LIMIT*x[hip2] + x[knee] + (KNEE_REST+pi/2)
        // in the picrawler's proprio encoding; validated against the CAD rest pose (10 deg)
        // and cross-checked controller-side vs the raw joint angles (51.5 vs 52.5 deg).
        if (m >= 3) {
            ga_tib_acc_ += std::fabs(float(tibia_plumb_scale_) * L.x[3] + L.x[6]
                                     + float(tibia_plumb_offset_));
            ++ga_tib_n_;
        }

        // ---- cycle periods.  The knee comes free from cos(L.phase) sign flips (that is
        // the same up-crossing the phase is built from); hip1 needs its own slow mean.
        const bool knee_above = (std::cos(L.phase) > 0.0f);
        if (knee_above && !ga_knee_above_[i]) {
            if (ga_knee_last_[i] >= 0) {
                const float d = float(int64_t(tick_id) - ga_knee_last_[i]);
                if (d > 2.0f && d < 600.0f)
                    ga_knee_per_[i] = (ga_knee_per_[i] <= 0.0f) ? d
                                    : (1.0f - kGaPerAlpha) * ga_knee_per_[i] + kGaPerAlpha * d;
            }
            ga_knee_last_[i] = int64_t(tick_id);
        }
        ga_knee_above_[i] = knee_above ? 1 : 0;

        const float h1 = L.x[0];
        ga_hip1_ema_[i] = (1.0f - kGaEmaAlpha) * ga_hip1_ema_[i] + kGaEmaAlpha * h1;
        const bool hip1_above = (h1 > ga_hip1_ema_[i]);
        if (hip1_above && !ga_hip1_above_[i]) {
            if (ga_hip1_last_[i] >= 0) {
                const float d = float(int64_t(tick_id) - ga_hip1_last_[i]);
                if (d > 2.0f && d < 600.0f)
                    ga_hip1_per_[i] = (ga_hip1_per_[i] <= 0.0f) ? d
                                    : (1.0f - kGaPerAlpha) * ga_hip1_per_[i] + kGaPerAlpha * d;
            }
            ga_hip1_last_[i] = int64_t(tick_id);
        }
        ga_hip1_above_[i] = hip1_above ? 1 : 0;
    }
}

// Per-tick Cruse stance/swing bookkeeping: a leg is in SWING when its foot is above
// its own self-calibrating height EMA; a touchdown (swing→stance) resets its
// since-plant counter (used by Rule 2's release window).
void MotorEPMv2::update_cruse_state() {
    if (int(foot_y_.size()) != n_legs_) return;
    if (int(foot_y_ema_.size()) != n_legs_) {
        foot_y_ema_ = foot_y_;
        in_swing_.assign(n_legs_, 0);
        ticks_since_plant_.assign(n_legs_, 1000);
    }
    // TRUE-CONTACT path: when the real foot-contact sensor is wired, a leg is in swing
    // iff it is not touching anything.  No EMA, no deadband, no self-reference — the
    // whole class of chatter/latching failure the height proxy had simply does not
    // arise, because this is a measurement rather than an inference.
    // `contact_instrument_only` splits the subscription from the gate: the diagnostic
    // needs ground truth on the bus while the control path keeps the incumbent detector
    // (wiring contact as the gate is separately refuted — that consumer wanted phase).
    if (have_contact_ && contact_instrument_only_ <= 0.0 && int(foot_contact_.size()) == n_legs_) {
        if (int(in_swing_.size()) != n_legs_) {
            in_swing_.assign(n_legs_, 0);
            ticks_since_plant_.assign(n_legs_, 1000);
        }
        int n_sw = 0;
        for (int i = 0; i < n_legs_; ++i) {
            bool sw = foot_contact_[i] < 0.5f;            // not touching → swinging
            if (in_swing_[i] && !sw) ticks_since_plant_[i] = 0;   // touchdown
            else ticks_since_plant_[i] += 1;
            in_swing_[i] = sw ? 1 : 0;
            if (sw) ++n_sw;
        }
        swing_frac_ema_ = (1.0f - kSwingFracAlpha) * swing_frac_ema_
                        + kSwingFracAlpha * (float(n_sw) / float(std::max(1, n_legs_)));
        cruse_bias_mean_ = cruse_bias_n_ ? (cruse_bias_acc_ / float(cruse_bias_n_)) : 0.0f;
        cruse_bias_acc_ = 0.0f; cruse_bias_n_ = 0;
        return;
    }
    if (int(foot_y_mad_.size()) != n_legs_) foot_y_mad_.assign(n_legs_, 0.0f);
    int n_sw_now = 0;
    for (int i = 0; i < n_legs_; ++i) {
        foot_y_ema_[i] = (1.0f - kFootYEmaAlpha) * foot_y_ema_[i] + kFootYEmaAlpha * foot_y_[i];
        const float dev = foot_y_[i] - foot_y_ema_[i];
        foot_y_mad_[i] = (1.0f - kFootYMadAlpha) * foot_y_mad_[i]
                       + kFootYMadAlpha * std::fabs(dev);
        // Deadband (see the header note).  The frac == 0 path is the LEGACY
        // comparison verbatim, not an equivalent-looking rewrite: with a band the
        // detector holds its state inside the band, and at dev == 0 exactly (which
        // happens on the first tick, since the EMA is seeded from foot_y) holding
        // differs from the legacy `>` returning false.  Branch explicitly so the
        // gain-0 guard is byte-identical by construction, not by argument.
        const float band = float(swing_hyst_frac_) * foot_y_mad_[i];
        bool sw;
        if (band <= 0.0f) {
            sw = foot_y_[i] > foot_y_ema_[i];        // legacy: no deadband
        } else {
            sw = in_swing_[i] != 0;                 // hold state inside the band
            if (dev >  band) sw = true;             // clearly above → swinging
            else if (dev < -band) sw = false;       // clearly below → planted
        }
        if (in_swing_[i] && !sw) ticks_since_plant_[i] = 0;   // touchdown
        else ticks_since_plant_[i] += 1;
        in_swing_[i] = sw ? 1 : 0;
        if (sw) ++n_sw_now;
    }
    // Diagnostic: what fraction of legs this detector calls "swinging".  Without it
    // the gate stance_lift / Cruse rely on is invisible, so a lever built on it
    // cannot be verified to have fired (CLAUDE.md §3.2 rule 5).
    swing_frac_ema_ = (1.0f - kSwingFracAlpha) * swing_frac_ema_
                    + kSwingFracAlpha * (float(n_sw_now) / float(std::max(1, n_legs_)));
    // Does a LEGAL phase reference reproduce this detector's output?  Compares the sign of
    // sin(cpg_phase + gait_phase[leg]) — the body-entrained rhythm, which a real robot has —
    // against in_swing_ from the god's-eye foot-height signal.  Pure diagnostic; decides
    // whether the oracle can be swapped for a phase gate before that gate is built.
    if (cpg_seen_ && int(gait_phase_.size()) == n_legs_ && int(in_swing_.size()) == n_legs_) {
        int agree = 0;
        for (int i = 0; i < n_legs_; ++i) {
            const bool phase_says_swing = std::sin(cpg_phase_ + float(gait_phase_[i])) > 0.0f;
            if (phase_says_swing == (in_swing_[i] != 0)) ++agree;
        }
        phase_agree_ema_ = (1.0f - kPhaseAgreeAlpha) * phase_agree_ema_
                         + kPhaseAgreeAlpha * (float(agree) / float(std::max(1, n_legs_)));
    }
    // Second, sharper comparison: the GLOBAL body phase measured ~0.50 (chance), but the
    // oracle detector is a PER-LEG oscillation threshold, so the matching legal candidate is
    // each leg's OWN phase, L.phase = atan2(joint velocity, joint pos − mean) — derived from
    // joint encoders, hence legal.  The structural analogue of `foot_y > foot_y_ema` on that
    // signal is cos(L.phase) > 0 (position above its own mean).  If THIS agrees, the oracle
    // has a drop-in legal replacement that needs no new sensor at all.
    if (int(in_swing_.size()) == n_legs_) {
        int agree2 = 0, n = 0;
        for (int i = 0; i < n_legs_; ++i) {
            if (!legs_[i].initialized) continue;
            const bool leg_phase_says_swing = std::cos(legs_[i].phase) > 0.0f;
            if (leg_phase_says_swing == (in_swing_[i] != 0)) ++agree2;
            ++n;
        }
        if (n > 0)
            legphase_agree_ema_ = (1.0f - kPhaseAgreeAlpha) * legphase_agree_ema_
                                + kPhaseAgreeAlpha * (float(agree2) / float(n));
    }
    // Fold the PREVIOUS tick's Cruse-contribution accumulator (the leg loop runs after
    // this function, so one tick of lag — irrelevant for a diagnostic).  Publishing the
    // mean rather than the sum keeps it comparable across leg counts; it decays to
    // exactly 0 within a tick of cruse_gain being zeroed, which is the whole point.
    cruse_bias_mean_ = cruse_bias_n_ ? (cruse_bias_acc_ / float(cruse_bias_n_)) : 0.0f;
    cruse_bias_acc_ = 0.0f;
    cruse_bias_n_   = 0;
}

void MotorEPMv2::on_param_change(std::string_view key, ParamValue const& value) {
    if (key == "model_lr")  model_lr_  = get_double(value, "model_lr");
    else if (key == "ctrl_lr")   ctrl_lr_   = get_double(value, "ctrl_lr");
    else if (key == "bias_lr")   bias_lr_   = get_double(value, "bias_lr");
    else if (key == "cmd_squash") cmd_squash_ = get_double(value, "cmd_squash");
    else if (key == "sense")        sense_        = get_double(value, "sense");
    else if (key == "dep_gain")     dep_gain_     = get_double(value, "dep_gain");
    else if (key == "dep_alpha")    dep_alpha_    = get_double(value, "dep_alpha");
    else if (key == "ctrl_damping") ctrl_damping_ = get_double(value, "ctrl_damping");
    else if (key == "reg_eps")   reg_eps_   = get_double(value, "reg_eps");
    else if (key == "max_dctrl") max_dctrl_ = get_double(value, "max_dctrl");
    else if (key == "babble_scale") babble_scale_ = get_double(value, "babble_scale");
    else if (key == "sat_lr")       sat_lr_       = get_double(value, "sat_lr");
    else if (key == "postural_gain") postural_gain_ = get_double(value, "postural_gain");
    else if (key == "postural_gain_joints") postural_gain_joints_ = get_double_vec(value, "postural_gain_joints");
    else if (key == "explore_noise") explore_noise_ = get_double(value, "explore_noise");
    else if (key == "knee_tuck_target") knee_tuck_target_ = get_double(value, "knee_tuck_target");
    else if (key == "hip2_tuck_target") hip2_tuck_target_ = get_double(value, "hip2_tuck_target");
    else if (key == "motor_gain") motor_gain_ = get_double(value, "motor_gain");
    else if (key == "coupling_gain") coupling_gain_ = get_double(value, "coupling_gain");
    else if (key == "couple_prec_gain") couple_prec_gain_ = get_double(value, "couple_prec_gain");
    else if (key == "coupling_fade_start") coupling_fade_start_ = get_int(value, "coupling_fade_start");
    else if (key == "coupling_fade_end")   coupling_fade_end_   = get_int(value, "coupling_fade_end");
    else if (key == "rhythm_gains")   rhythm_gains_   = get_double_vec(value, "rhythm_gains");
    else if (key == "rhythm_offsets") rhythm_offsets_ = get_double_vec(value, "rhythm_offsets");
    else if (key == "rhythm_fade_start") rhythm_fade_start_ = get_int(value, "rhythm_fade_start");
    else if (key == "rhythm_fade_end")   rhythm_fade_end_   = get_int(value, "rhythm_fade_end");
    else if (key == "cpg_embed") {
        if (auto p = std::get_if<bool>(&value)) cpg_embed_ = *p;
        else if (auto p = std::get_if<int64_t>(&value)) cpg_embed_ = (*p != 0);
    }
    else if (key == "embed_lr")    embed_lr_    = get_double(value, "embed_lr");
    else if (key == "embed_decay") embed_decay_ = get_double(value, "embed_decay");
    else if (key == "ctrl_symmetry_gain") ctrl_symmetry_gain_ = get_double(value, "ctrl_symmetry_gain");
    else if (key == "coord_adapt_rate") coord_adapt_rate_ = get_double(value, "coord_adapt_rate");
    else if (key == "coord_explore") coord_explore_ = get_double(value, "coord_explore");
    else if (key == "coord_reward_drive") coord_reward_drive_ = get_double(value, "coord_reward_drive");
    else if (key == "stuck_explore_gain") stuck_explore_gain_ = get_double(value, "stuck_explore_gain");
    else if (key == "progress_commit_gain") progress_commit_gain_ = get_double(value, "progress_commit_gain");
    else if (key == "forward_flow_gain") forward_flow_gain_ = get_double(value, "forward_flow_gain");
    else if (key == "stance_lift_gain") stance_lift_gain_ = get_double(value, "stance_lift_gain");
    else if (key == "stance_release_frac") stance_release_frac_ = get_double(value, "stance_release_frac");
    else if (key == "swing_hyst_frac") swing_hyst_frac_ = get_double(value, "swing_hyst_frac");
    else if (key == "contact_instrument_only") contact_instrument_only_ = get_double(value, "contact_instrument_only");
    else if (key == "gait_align_diag") gait_align_diag_ = get_double(value, "gait_align_diag");
    else if (key == "stroke_load_gain") stroke_load_gain_ = get_double(value, "stroke_load_gain");
    else if (key == "tibia_plumb_gain") tibia_plumb_gain_ = get_double(value, "tibia_plumb_gain");
    else if (key == "tibia_plumb_scale") tibia_plumb_scale_ = get_double(value, "tibia_plumb_scale");
    else if (key == "tibia_plumb_offset") tibia_plumb_offset_ = get_double(value, "tibia_plumb_offset");
    else if (key == "swing_tuck_hip2") swing_tuck_hip2_ = get_double(value, "swing_tuck_hip2");
    else if (key == "swing_tuck_knee") swing_tuck_knee_ = get_double(value, "swing_tuck_knee");
    else if (key == "homeo_leak_upright_only") homeo_leak_upright_only_ = get_double(value, "homeo_leak_upright_only");
    else if (key == "homeo_leak_progress_gate") homeo_leak_progress_gate_ = get_double(value, "homeo_leak_progress_gate");
    else if (key == "homeo_leak_cycles") homeo_leak_cycles_ = get_double(value, "homeo_leak_cycles");
    else if (key == "homeo_upright_gate") homeo_upright_gate_ = get_double(value, "homeo_upright_gate");
    else if (key == "height_unwind_free") height_unwind_free_ = get_double(value, "height_unwind_free");
    else if (key == "coord_fitness_mode") coord_fitness_mode_ = int(get_int(value, "coord_fitness_mode"));
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
    else if (key == "stroke_phase_src")    stroke_phase_src_    = get_double(value, "stroke_phase_src");
    else if (key == "step_phase_debounce") step_phase_debounce_ = get_double(value, "step_phase_debounce");
    else if (key == "step_period_alpha")   step_period_alpha_   = get_double(value, "step_period_alpha");
    else if (key == "step_period_min")     step_period_min_     = get_double(value, "step_period_min");
    else if (key == "step_period_max")     step_period_max_     = get_double(value, "step_period_max");
    else if (key == "step_phase_lock")    step_phase_lock_     = get_double(value, "step_phase_lock");
    else if (key == "gait_raster_diag")    gait_raster_diag_    = get_double(value, "gait_raster_diag");
    else if (key == "steer") steer_ = get_double(value, "steer");
    else if (key == "stroke_signs") {
        auto ss = get_double_vec(value, "stroke_signs");
        if (int(ss.size()) == n_legs_) stroke_signs_ = ss;
    }
    else if (key == "balance_gain") balance_gain_ = get_double(value, "balance_gain");
    else if (key == "propulsion_balance_gain") propulsion_balance_gain_ = get_double(value, "propulsion_balance_gain");
    else if (key == "amp_homeo_gain") amp_homeo_gain_ = get_double(value, "amp_homeo_gain");
    else if (key == "amp_target") amp_target_ = get_double(value, "amp_target");
    else if (key == "amp_seek_rate") amp_seek_rate_ = get_double(value, "amp_seek_rate");
    else if (key == "amp_seek_ticks") amp_seek_ticks_ = get_int(value, "amp_seek_ticks");
    else if (key == "heading_gain") heading_gain_ = get_double(value, "heading_gain");
    else if (key == "heading_hold_gain") heading_hold_gain_ = get_double(value, "heading_hold_gain");
    else if (key == "heading_bearing_hold_gain") heading_bearing_hold_gain_ = get_double(value, "heading_bearing_hold_gain");
    else if (key == "nav_gain") nav_gain_ = get_double(value, "nav_gain");
    else if (key == "cog_steer_gain") cog_steer_gain_ = get_double(value, "cog_steer_gain");
    else if (key == "cog_thrust_gain") cog_thrust_gain_ = get_double(value, "cog_thrust_gain");
    else if (key == "boredom_noise_gain") boredom_noise_gain_ = get_double(value, "boredom_noise_gain");
    else if (key == "boredom_escalation_rate") boredom_escalation_rate_ = get_double(value, "boredom_escalation_rate");
    else if (key == "height_homeo_gain") height_homeo_gain_ = get_double(value, "height_homeo_gain");
    else if (key == "height_k") height_k_ = get_double(value, "height_k");
    else if (key == "height_ground_gain") height_ground_gain_ = get_double(value, "height_ground_gain");
    else if (key == "height_lift_knee") height_lift_knee_ = get_double(value, "height_lift_knee");
    else if (key == "stance_lift_hip2") stance_lift_hip2_ = get_double(value, "stance_lift_hip2");
    else if (key == "support_select_gain") support_select_gain_ = get_double(value, "support_select_gain");
    else if (key == "panic_on") panic_on_ = get_double(value, "panic_on");
    else if (key == "panic_off") panic_off_ = get_double(value, "panic_off");
    else if (key == "panic_strength") panic_strength_ = get_double(value, "panic_strength");
    else if (key == "panic_noise") panic_noise_ = get_double(value, "panic_noise");
    else if (key == "panic_motor_mult") panic_motor_mult_ = get_double(value, "panic_motor_mult");
    else if (key == "panic_push_amp") panic_push_amp_ = get_double(value, "panic_push_amp");
    else if (key == "panic_push_hz") panic_push_hz_ = get_double(value, "panic_push_hz");
}

// =============================================================================
// IMPORT I7 — whole-body controller.  One (A, C, b, h) spanning every joint of every
// leg, so inter-leg coordination is a LEARNABLE entry of C rather than something that
// can only arrive mechanically through the body.  The maths is identical to the per-leg
// homeokinetic update; only the matrices are wider.
// =============================================================================
void MotorEPMv2::wb_init(int n_per_leg) {
    const int N = n_legs_ * n_per_leg, M = n_legs_ * motor_dim_;
    std::mt19937 rng(static_cast<uint32_t>(base_seed_ ^ 0x7B1D0057u));
    std::normal_distribution<float> nd(0.0f, 1.0f);
    Aw_ = Eigen::MatrixXf::Zero(N, M);
    Cw_ = Eigen::MatrixXf::Zero(M, N);
    bw_ = Eigen::VectorXf::Zero(N);
    hw_ = Eigen::VectorXf::Zero(M);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j) Aw_(i, j) = float(init_scale_) * nd(rng);
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < N; ++j) Cw_(i, j) = float(init_scale_) * nd(rng);
    // c_init keeps its meaning: each motor's positive feedback from its OWN joint's
    // position, now on the block diagonal.  The CROSS-LEG terms start at the same small
    // random values the per-leg build used, so they are grown by the HK gradient rather
    // than declared — which is the whole point of moving them into C.
    if (c_init_ > 0.0)
        for (int leg = 0; leg < n_legs_; ++leg)
            for (int j = 0; j < motor_dim_ && 3 * j < n_per_leg; ++j)
                Cw_(leg * motor_dim_ + j, leg * n_per_leg + 3 * j) += float(c_init_);
    Xw_      = Eigen::VectorXf::Zero(N);
    prevXw_  = Eigen::VectorXf::Zero(N);
    prevYw_  = Eigen::VectorXf::Zero(M);
    Zw_      = Eigen::VectorXf::Zero(M);
    Cdepw_   = Eigen::MatrixXf::Zero(M, N);
    prevPrevYw_ = Eigen::VectorXf::Zero(M);
    wb_have_prev_ = false;
    wb_steps_ = 0;
    wb_ready_ = true;
}

void MotorEPMv2::wb_learn_and_control() {
    const int m = motor_dim_, M = n_legs_ * m;
    const int n_per_leg = legs_[0].n, N = n_legs_ * n_per_leg;
    for (int leg = 0; leg < n_legs_; ++leg)                       // gather
        Xw_.segment(leg * n_per_leg, n_per_leg) = legs_[leg].x;
    wb_steps_ += 1;
    const bool warmup = (wb_steps_ <= babble_ticks_);

    if (wb_have_prev_) {
        Eigen::VectorXf x_hat = Aw_ * prevYw_ + bw_;
        Eigen::VectorXf xi    = Xw_ - x_hat;                      // whole-body motor TLE
        Aw_.noalias() += float(model_lr_) * xi * prevYw_.transpose();
        bw_.noalias() += float(model_lr_) * xi;
        wb_tle_ema_ = (1.0f - kTeleEmaAlpha) * wb_tle_ema_ + kTeleEmaAlpha * xi.norm();
        if (!warmup) {
            Eigen::VectorXf z = Cw_ * prevXw_ + hw_;
            Eigen::MatrixXf G = Eigen::MatrixXf::Zero(M, M);
            for (int i = 0; i < M; ++i) { float t = std::tanh(z[i]); G(i, i) = 1.0f - t * t; }
            Eigen::MatrixXf AG = Aw_ * G;                         // N x M
            Eigen::MatrixXf Lp = AG * Cw_;                        // N x N loop Jacobian
            Eigen::MatrixXf P  = (Lp * Lp.transpose()
                                  + float(reg_eps_) * Eigen::MatrixXf::Identity(N, N)).inverse();
            // DEP, whole-body: the rule can now write CROSS-LEG terms directly, because
            // moving one leg mechanically changes its neighbours' sensors and that shows up
            // in the derivative correlation.  This is the combination the coordination
            // evidence points at — per-leg DEP still has no cross-leg entries to write.
            if (dep_gain_ > 0.0) {
                const Eigen::VectorXf dxw = Xw_ - prevXw_;
                const Eigen::VectorXf dyw = prevYw_ - prevPrevYw_;
                const float a_dep = float(dep_alpha_);
                Cdepw_ = (1.0f - a_dep) * Cdepw_ + a_dep * (dyw * dxw.transpose());
                for (int i = 0; i < M; ++i) {
                    const float rn = Cdepw_.row(i).norm();
                    if (rn > 1e-7f) Cw_.row(i) = Cdepw_.row(i) * (float(dep_gain_) / rn);
                }
            }
            Eigen::VectorXf q  = P * xi;
            Eigen::MatrixXf dC = 2.0f * float(ctrl_lr_) * (AG.transpose() * q) * (q.transpose() * Lp);
            float dn = dC.norm();
            // Scale the ignition clamp with sqrt(n_legs): the same PER-ENTRY step size on a
            // matrix with n_legs× the entries has a Frobenius norm that grows, so reusing the
            // per-leg clamp here would silently throttle learning by ~2× and look like
            // "whole-body C learns slower" when it is only clamped harder.
            const float clamp_w = float(max_dctrl_) * std::sqrt(float(n_legs_));
            if (max_dctrl_ > 0.0 && dn > clamp_w) dC *= clamp_w / dn;
            if (dep_gain_ <= 0.0) Cw_.noalias() += dC;   // DEP owns Cw when on
            if (sense_ > 0.0) {                                   // I2, whole-body form
                Eigen::MatrixXf CqqA = Cw_ * (q * q.transpose()) * Aw_;   // M x M
                Eigen::VectorXf eps(M), yt(M);
                for (int i = 0; i < M; ++i) {
                    eps[i] = CqqA(i, i) * G(i, i) * 2.0f * float(sense_);
                    yt[i]  = std::tanh(z[i]);
                }
                Cw_.noalias() -= float(ctrl_lr_) * (eps.cwiseProduct(yt)) * prevXw_.transpose();
            }
            hw_.noalias() += float(bias_lr_) * (G * (Aw_.transpose() * q));
            if (sat_lr_ > 0.0)                                    // the h bound — see the sat_lr entry
                for (int i = 0; i < M; ++i) {
                    float ti = std::tanh(z[i]), gs = z[i] * ti * ti;
                    Cw_.row(i).noalias() -= float(sat_lr_) * gs * prevXw_.transpose();
                    hw_[i] -= float(sat_lr_) * gs;
                }
            if (ctrl_damping_ > 0.0) { Cw_ *= (1.0f - float(ctrl_damping_));
                                       hw_ *= (1.0f - float(ctrl_damping_)); }
        }
    }
    Zw_     = Cw_ * Xw_ + hw_;        // pre-tanh; the per-leg loop slices and finishes it
    prevPrevYw_ = prevYw_;
    prevXw_ = Xw_;
    wb_have_prev_ = true;
}

void MotorEPMv2::ensure_leg_init(int leg, int n) {
    Leg& L = legs_[leg];
    if (L.initialized) return;
    int m = motor_dim_;
    // ---- 2026-08-04 · c_pair_init: SHARE the init seed between left/right partners ------
    // The handedness of this body's skid is SEED-RANDOM, not structural: across 4 seeds the
    // right/left hip1 demand ratio measured 3.07, 4.35, 0.69, 0.97 -- seed 3 is LEFT-heavy --
    // and it still flips with all controller learning off, so it is the 0.01*random C init
    // locked in by the gait as a stable attractor.  This gives the L/R partners the SAME
    // initial control law (legs 0&1, 2&3); stroke_signs still mirrors their OUTPUT, and the
    // fore/aft partners stay different.
    // ⚠ The class header warns that the per-leg random init is the "inter-leg symmetry
    // breaker" that avoids v6-premotor-bilateral-mirror-collapse.  This removes HALF of it
    // (L/R) and keeps the fore/aft difference -- an empirical question, which is why it is
    // default-off and measured rather than argued.
    const uint32_t init_leg = (c_pair_init_ > 0.0) ? uint32_t(leg & ~1) : uint32_t(leg);
    std::mt19937 rng(static_cast<uint32_t>(base_seed_ ^ (0x9E3779B9u + init_leg)));
    std::normal_distribution<float> nd(0.0f, 1.0f);
    L.n = n;
    L.A = Eigen::MatrixXf::Zero(n, m);
    L.b = Eigen::VectorXf::Zero(n);
    L.C = Eigen::MatrixXf::Zero(m, n);
    L.Cdep = Eigen::MatrixXf::Zero(m, n);
    L.prev_prev_y = Eigen::VectorXf::Zero(m);
    L.Cphi = Eigen::MatrixXf::Zero(m, 2);   // phase-conditioning starts at 0 (byte-identical until learned)
    L.Cvel = Eigen::MatrixXf::Zero(m, 2);   // velocity feed-forward starts at 0 (byte-identical until learned)
    L.prev_phi_ctx.setZero();
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
    // Import I1 (Playful Machine, Sox cInit): start the loop SELF-EXCITING.  Each motor
    // gets a positive feedback weight from its OWN joint's position (state index 3j), so
    // the body twitches from tick 1 and the homeokinetic gradient SHAPES that oscillation
    // rather than having to manufacture one from a dead fixed point (which is what
    // babble_ticks / sat_lr / explore_noise were added to do).  ADDED to, not replacing,
    // the per-leg random init — that randomness is the inter-leg symmetry breaker this
    // module depends on (see the class header on bilateral-mirror collapse).
    // c_init = 0 leaves this byte-identical to the legacy init.
    if (c_init_ > 0.0)
        for (int j = 0; j < m && 3 * j < n; ++j)
            L.C(j, 3 * j) += float(c_init_);
    L.x      = Eigen::VectorXf::Zero(n);
    L.prev_x = Eigen::VectorXf::Zero(n);
    L.prev_y = Eigen::VectorXf::Zero(m);
    L.have_prev  = false;
    L.steps_seen = 0;
    L.babble_rng.seed(static_cast<uint32_t>(base_seed_ ^ (0x2545F491u + uint32_t(leg))));
    L.initialized = true;
}

void MotorEPMv2::handle_proprio(int leg, MessagePtr payload) {
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
        // Crouch the femur too: hip2 (joint 1) rests at the extended spawn pose by default, giving
        // it no leverage to lift/load.  A hip2 tuck flexes it into a crouch so the swing can work.
        if (hip2_tuck_target_ > -90.0 && motor_dim_ >= 2)
            L.rest_pos[1] = float(hip2_tuck_target_);
        L.rest_captured = true;
    }
    L.fresh = true;
}

void MotorEPMv2::tick(uint64_t tick_id) {
    int m = motor_dim_;

    // ---- Gate 2 coupling wean: deterministic tick-scheduled linear fade of the imposed
    // Kuramoto coupling → the learned keyframe map takes over the coordination.  Function of
    // tick_id only, so clone-determinism holds.  Disabled (fade_start<0) ⇒ coupling stays flat.
    coupling_eff_ = float(coupling_gain_);
    if (coupling_fade_start_ >= 0 && coupling_fade_end_ > coupling_fade_start_) {
        float f = 1.0f;
        if (int64_t(tick_id) >= coupling_fade_end_)        f = 0.0f;
        else if (int64_t(tick_id) > coupling_fade_start_)
            f = 1.0f - float(int64_t(tick_id) - coupling_fade_start_)
                     / float(coupling_fade_end_ - coupling_fade_start_);
        coupling_eff_ = float(coupling_gain_) * f;
    }
    // Gate 2 (coherent-scaffold wean): fade the CPG-phase rhythm drive so the learned keyframe
    // objective takes over.  Function of tick_id only → clone-deterministic.
    rhythm_scale_ = 1.0f;
    if (rhythm_fade_start_ >= 0 && rhythm_fade_end_ > rhythm_fade_start_) {
        if (int64_t(tick_id) >= rhythm_fade_end_)        rhythm_scale_ = 0.0f;
        else if (int64_t(tick_id) > rhythm_fade_start_)
            rhythm_scale_ = 1.0f - float(int64_t(tick_id) - rhythm_fade_start_)
                                 / float(rhythm_fade_end_ - rhythm_fade_start_);
    }

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
    // Height defense is a STANDING reflex: at rest it fights the G6DOF sag to stand
    // tall (Gate 0).  But while walking/climbing a lift bias hoists the legs off the
    // terrain and loses traction — on an incline the belly must ride LOW to climb
    // (measured 2026-07-23: homeo OFF clears the hump z→4.1; homeo ON winds up and
    // stalls at ~2.6, lifting the legs off the slope).  So fade the reflex out with
    // forward progress: rest_frac 1 at rest → 0 while cruising.  This also anti-winds-up
    // (no integration while moving), so a wound-up bias can't accumulate on the ramp.
    height_rest_frac_ = std::clamp(1.0f - fwd_progress_ema_ / kHeightMoveSuppVel, 0.0f, 1.0f);
    // ---- HOMEOSTAT LEAK (2026-07-26) -------------------------------------------------
    // The inversion-forgetting failure was NOT excess plasticity — it was ASYMMETRIC
    // plasticity.  height_bias and amp_gain both wound fast in one direction and could not
    // come back: amp_gain only unwinds when amp_ema EXCEEDS its target (which after an
    // excursion it may never do), and height_bias is multiplied by height_rest_frac, which
    // goes to 0 while moving — so walking literally froze it.  Both are rectifying.
    //
    // A leak makes their memory FINITE BY CONSTRUCTION: x += k*err - lambda*x.  No regime
    // classifier, no uprightness signal, no snapshot, no supervisor deciding which state is
    // worth keeping — just a forgetting rate.  For low-level motor state that is the right
    // property: learn fast, forget fast.  (An earlier upright-GATE attempt froze these
    // instead, which also froze the 40x amplitude escalation that is what rights the robot
    // — protecting the walk by lesioning the escape.  A leak has no such failure mode: the
    // wind is never blocked, it just does not persist.)
    //
    // The rate is expressed in STRIDE CYCLES off the body's own measured omega, so it
    // tracks whatever frequency the body settles at instead of being a tuned tick count.
    float homeo_leak = 0.0f;
    leak_amp_ = 0.0f; leak_h_ = 0.0f;
    if (homeo_leak_cycles_ > 0.0) {
        const float period = (std::fabs(body_omega_) > 1e-4f)
                           ? (2.0f * float(M_PI) / std::fabs(body_omega_))
                           : kLeakFallbackPeriod;   // no rhythm token yet — measured stride
        homeo_leak = std::clamp(1.0f / (float(homeo_leak_cycles_) * period), 0.0f, 0.5f);
        // WHEN TO STOP FORGETTING (2026-07-26, operator-discovered, two observations).
        // A constant leak is wrong: the integrator wind-up is not only damage, it is the
        // ESCALATION that gets the body out of trouble.  Two separate cases were found by
        // hand, and they need DIFFERENT treatment per integrator:
        //
        //  (1) INVERTED on a 30-degree wall.  With leak=5 the robot could not escape at all
        //      (measured 0/4 vs 2/4 with the leak off); setting leak=0 let effort accumulate
        //      until it got out, then leak=5 restored the walk quickly.  Both integrators
        //      should accumulate here — the posture itself is invalid.
        //  (2) UPRIGHT but BLOCKED at a slanted wall.  The robot "no longer learns how to
        //      climb — it stays in the same gait it was using on the flat."  Here a posture
        //      gate alone does nothing, because the body IS upright.  Effort must escalate.
        //      BUT height_bias must NOT accumulate on a slope: that is precisely the refuted
        //      windup where hip2 lifts the legs off the incline (measured: progress-gating
        //      BOTH integrators cost hump traversal 6.09 -> 4.84).
        //
        // Hence two effective rates.  amp_gain (effort) stops forgetting whenever the agent
        // is FAILING — bad posture or no progress.  height_bias (posture bias) stops
        // forgetting only when the posture is invalid, so a stall on a hill cannot readmit
        // the windup.  Forgetting is a luxury of success; escalation is the response to
        // failure; and which state may escalate depends on what that state means.
        leak_amp_ = homeo_leak;   // effort escalation
        leak_h_   = homeo_leak;   // posture bias
        const bool bad_posture = (homeo_leak_upright_only_ > 0.0
                                  && upright_ < float(homeo_leak_upright_only_));
        if (bad_posture) { leak_amp_ = 0.0f; leak_h_ = 0.0f; }
        // Stall suppresses ONLY the effort leak (see case 2).
        if (homeo_leak_progress_gate_ > 0.0) leak_amp_ *= (1.0f - height_rest_frac_);
    }
    homeo_leak_eff_ = leak_amp_;   // diag: the EFFORT forgetting rate actually applied

    if (height_homeo_gain_ > 0.0 && chassis_h_max_ > 1e-4f) {
        // Setpoint fraction: fixed by config, or discovered from belly grounding.
        if (height_k_eff_ < 0.0f) height_k_eff_ = float(height_k_);
        if (height_ground_gain_ > 0.0) {
            const bool grounded = (chassis_h_ < kHeightGroundThresh);
            // Rise fast on contact, decay slowly back toward the configured floor.  The
            // asymmetry is the point: grounding is evidence the target is too low, while
            // NOT grounding is only weak evidence it is too high.
            if (grounded) height_k_eff_ += float(height_ground_gain_) * (kHeightKMax - height_k_eff_);
            else          height_k_eff_ -= float(height_ground_gain_) * 0.05f * (height_k_eff_ - float(height_k_));
            height_k_eff_ = std::clamp(height_k_eff_, float(height_k_), kHeightKMax);
        } else {
            height_k_eff_ = float(height_k_);
        }
        float tgt = height_k_eff_ * chassis_h_max_;
        float dh = float(height_homeo_gain_) * (tgt - chassis_h_ema_);
        // (a) UPRIGHT GATE.  While the body is not upright the height setpoint is
        // meaningless — the belly rangefinder is not looking at the ground the robot
        // stands on — so integrating here only poisons a reflex that will be needed
        // later.  MEASURED (2026-07-26 inversion repro): an inverted episode rails
        // height_bias to a clamp and it NEVER returns, and forward progress after
        // self-righting stays negative for 7200+ ticks.
        if (homeo_upright_gate_ > 0.0 && upright_ < float(homeo_upright_gate_)) dh = 0.0f;
        // (b) ASYMMETRIC WINDUP FADE.  height_rest_frac was applied to the whole
        // integration: that stopped windup while moving (the incline fix, kept) but ALSO
        // made a railed bias unable to UNWIND while walking — so the more the robot tried
        // to walk, the longer it stayed broken.  Fade only the winding direction.
        if (height_unwind_free_ > 0.0) {
            const bool winding = (dh * height_bias_) > 0.0f
                              || std::fabs(height_bias_) < 1e-6f;   // from neutral = winding
            height_bias_ += winding ? dh * height_rest_frac_ : dh;
        } else {
            height_bias_ += dh * height_rest_frac_;                 // legacy
        }
        height_bias_ -= leak_h_ * height_bias_;       // forget toward neutral lift
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
    // 2026-08-03 — UNGATED.  This pre-pass used to run only when some CONSUMER of the
    // phase was active (coupling / stroke / steer / amp-homeostat / heading / nav).  On the
    // pure_hk tier every one of those is 0, so L.phase never updated, stayed at 0, and
    // gait_coherence() returned EXACTLY 0.000 — because with gait_phase=[0,pi,pi,0] the four
    // unit vectors cancel precisely.  That reads as "the legs are perfectly uncoordinated"
    // when in fact nothing was measured: removing the scaffolds silently removed the
    // instrument for the operator's central question ("do the legs work together?").
    // The pre-pass only writes L.phase / L.knee_ema (the amplitude homeostat inside stays
    // separately gated), so running it always is behaviourally inert — VERIFIED BY
    // MEASUREMENT on both the deployed and pure_hk arms, not by argument.
    {
        float amp_sum = 0.0f; int amp_n = 0;
        const bool homeo_gated = (homeo_upright_gate_ > 0.0
                                  && upright_ < float(homeo_upright_gate_));
        for (int leg = 0; leg < n_legs_; ++leg) {
            Leg& L = legs_[leg];
            if (!L.initialized || L.n < 3 * m) continue;
            int pj = (phase_joint_ >= 0 && phase_joint_ < m) ? phase_joint_ : (m - 1);
            float kp = L.x[3 * pj];               // phase-source joint position (default knee = m-1)
            float kd = L.x[3 * pj + 2];           // phase-source joint delta (velocity proxy)
            L.knee_ema = (1.0f - kKneeEmaAlpha) * L.knee_ema + kKneeEmaAlpha * kp;
            // Low-pass the velocity arm.  alpha is the smoothing WEIGHT on the new sample,
            // so phase_vel_smooth = 0 keeps the raw delta exactly (byte-identical guard) and
            // larger values average over more ticks.  Applied to the delta only.
            if (phase_vel_smooth_ > 0.0 && leg < 8) {
                const float a = 1.0f / (1.0f + float(phase_vel_smooth_));
                phase_vel_ema_[leg] = (1.0f - a) * phase_vel_ema_[leg] + a * kd;
                kd = phase_vel_ema_[leg];
            }
            float vx = kp - L.knee_ema, vy = kd * kPhaseVelScale;
            // Symmetric variant: the SAME kernel on both arms => rigid rotation of the
            // phase vector, so monotonicity improves without warping where in the cycle
            // the stroke lands.  See the header note on why the y-arm-only version failed.
            if (phase_sym_smooth_ > 0.0 && leg < 8) {
                const float a = 1.0f / (1.0f + float(phase_sym_smooth_));
                if (!phase_sym_init_[leg]) {
                    phase_pos_ema_[leg] = vx; phase_vel_ema_[leg] = vy;
                    phase_sym_init_[leg] = true;
                }
                phase_pos_ema_[leg] = (1.0f - a) * phase_pos_ema_[leg] + a * vx;
                phase_vel_ema_[leg] = (1.0f - a) * phase_vel_ema_[leg] + a * vy;
                vx = phase_pos_ema_[leg]; vy = phase_vel_ema_[leg];
            }
            const float phase_new = std::atan2(vy, vx);
            // COUPLING HEALTH (operator asked for coupling as a live metric).  A real
            // oscillator advances monotonically; measure how much of the time this one
            // runs BACKWARDS.  A high retrograde fraction means L.phase is jitter rather
            // than rhythm, and the Kuramoto term is chasing noise -- which is the
            // operator's "micro footfalls tripping the coupling too rapidly", located at
            // the knee-derived phase rather than at the contacts.
            {
                float d = phase_new - L.phase_prev;
                while (d >  float(M_PI)) d -= 2.0f * float(M_PI);
                while (d < -float(M_PI)) d += 2.0f * float(M_PI);
                phase_freq_diag_  = (1.0f - kCommitPrecAlpha) * phase_freq_diag_
                                  + kCommitPrecAlpha * d;
                phase_retro_diag_ = (1.0f - kCommitPrecAlpha) * phase_retro_diag_
                                  + kCommitPrecAlpha * (d < 0.0f ? 1.0f : 0.0f);
                L.phase_prev = phase_new;
            }
            L.phase = phase_new;
            // amplitude homeostat: phase-vector magnitude = oscillation amplitude.
            // Slow integral regulator drives amp_gain so amp_ema → amp_target.
            //
            // ⚠ amp_ema itself is UNGATED as of 2026-08-03.  It used to update only when
            // amp_homeo_gain>0, i.e. only when a CONSUMER wanted it — so on the pure_hk
            // tier it stayed 0 forever.  That silently broke the inter-leg PLV, whose
            // frozen-body gate is "is this leg actually oscillating?" and reads amp_ema:
            // every arm reported plv support 0 and a PLV of 0.000, which looks exactly
            // like "no coordination" and is in fact "no measurement".  Third instance of
            // an instrument gated on the thing being ablated; the estimate is now always
            // computed and only its CONSUMERS stay gated.
            {
                float amp = std::sqrt(vx * vx + vy * vy);
                L.amp_ema = (1.0f - kAmpEmaAlpha) * L.amp_ema + kAmpEmaAlpha * amp;
            }
            if (amp_homeo_gain_ > 0.0 || amp_seek_rate_ > 0.0) {
                float amp = std::sqrt(vx * vx + vy * vy);
                amp_sum += amp; ++amp_n;          // for the CoT amplitude search
                if (amp_homeo_gain_ > 0.0) {
                    // UPRIGHT GATE.  While not upright the body cannot reach amp_target at
                    // all, so this integrator winds to its rail and STAYS there — measured
                    // 0.10 (its floor) -> 2.9..4.3 across an inverted episode, never
                    // returning, which is the "movements are exaggerated" symptom.  Freeze
                    // the integrator; the measurement EMA above keeps tracking so it is
                    // current the moment the body is upright again.
                    if (!homeo_gated) {
                        L.amp_gain += float(amp_homeo_gain_) * (float(amp_target_) - L.amp_ema);
                    }
                    // Forget toward kAmpGainMin, i.e. MINIMUM AUTHORITY.  Unity (1.0) was
                    // tried first as the "semantically neutral" value for a gain and it cost
                    // real performance (net_z 4.75 -> 3.6, straight 0.74 -> 0.56-0.61,
                    // tilt_sd up, and falls at a 2-cycle rate): normal walking pins this
                    // integrator at its floor, so a pull toward unity fights the regulator
                    // every tick.  Semantically neutral is not behaviourally neutral — the
                    // right target is the least-intervention end of the range, where the leak
                    // AGREES with the homeostat in normal operation and only bites after an
                    // excursion.  Applied even when gated so a wound value always decays.
                    L.amp_gain -= leak_amp_ * (L.amp_gain - kAmpGainMin);
                    L.amp_gain = std::clamp(L.amp_gain, kAmpGainMin, kAmpGainMax);
                }
            }
        }
        if (amp_n > 0) cur_amp_ = amp_sum / float(amp_n);
    }

    // ---- STUCK→EXPLORE desire: detect a sustained forward-progress stall (compliant —
    // fwd_v EMA, no god's-eye position) and ramp a boost that amplifies the exploration
    // channels below (explore_noise + the coord phase-search σ) so the gait discovers a
    // push.  Self-terminating: decays the instant fwd progress resumes.  0 = off (no boost
    // → both channels use their base scale → byte-identical).
    // progress→COMMIT (lever C) shares this fwd_progress_ema_ stall detector (inverted).
    // The height homeostat also reads it (to fade its lift out while moving).
    if (stuck_explore_gain_ > 0.0 || progress_commit_gain_ > 0.0 || height_homeo_gain_ > 0.0) {
        fwd_progress_ema_ = (1.0f - kFwdProgressAlpha) * fwd_progress_ema_
                          + kFwdProgressAlpha * fwd_v_;
    }
    // ── fwd_v RESONANCE.  An adaptive-frequency Hopf oscillator entrained BY forward
    // velocity: it does not impose a rhythm, it LEARNS the frequency at which the body
    // already propels itself.  That matters because fwd_v is the one signal the operator
    // trusts -- foot contacts were measured to be a poor proxy for a step, and the
    // knee-derived L.phase is what the coupling currently rides.
    //   Input is centred (fwd_v minus its own slow mean) and divided by its own running
    // spread, so the entrainment strength is dimensionless and nothing is tuned to fwd_v's
    // magnitude.  Frequency adapts on a ~2000-tick timescale, far slower than a stride, so
    // it tracks the body's rhythm rather than individual strides.
    {
        const float fin = fwd_v_ - fwd_progress_ema_;
        if (res_in_spread_ <= 0.0f) res_in_spread_ = std::fabs(fin);
        res_in_spread_ = (1.0f - kCommitPrecAlpha) * res_in_spread_
                       + kCommitPrecAlpha * std::fabs(fin);
        const float F = fin / (res_in_spread_ + 1e-6f);
        // Seed the frequency from the legs' OWN measured phase rate rather than a chosen
        // constant, so the oscillator starts in the right basin (doctrine §5).
        if (res_w_ <= 0.0f && phase_freq_diag_ > 1e-4f) res_w_ = phase_freq_diag_;
        if (res_w_ > 0.0f) {
            const float r = std::sqrt(res_x_ * res_x_ + res_y_ * res_y_);
            const float dx = kResGamma * (1.0f - r * r) * res_x_ - res_w_ * res_y_ + kResEps * F;
            const float dy = kResGamma * (1.0f - r * r) * res_y_ + res_w_ * res_x_;
            res_x_ += dx; res_y_ += dy;
            // Righetti/Ijspeert frequency adaptation: the input pulls omega toward the
            // driving frequency.  This is the "find the resonance" step.
            res_w_ = std::clamp(res_w_ - kResWAlpha * F * res_y_ / std::max(r, 1e-3f),
                                0.005f, 1.2f);
            res_amp_ema_ = (1.0f - kCommitPrecAlpha) * res_amp_ema_ + kCommitPrecAlpha * r;
        }
    }
    if (stuck_explore_gain_ > 0.0) {
        if (fwd_progress_ema_ < kStuckVelThresh) {
            if (stuck_ticks_ < kStuckWindowTicks * 20) ++stuck_ticks_;   // cap the counter
        } else {
            stuck_ticks_ = 0;
        }
        if (stuck_ticks_ >= kStuckWindowTicks)
            stuck_boost_ = std::min(1.0f, stuck_boost_ + kStuckBoostRise);
        else
            stuck_boost_ = std::max(0.0f, stuck_boost_ - kStuckBoostDecay);
    } else {
        stuck_boost_ = 0.0f;   // ensure inert when the lever is off
    }

    // ---- progress→COMMIT (lever C): the INVERSE of the stall detector.  Sustained HIGH
    // forward progress ramps commit_boost_ up; it damps exploration + adds thrust below.
    // 0 = off → commit_boost_ pinned 0 → byte-identical.
    if (progress_commit_gain_ > 0.0) {
        // ---- 2026-08-05 · COMMIT AS EARNED PRECISION, not a schedule -------------------
        // Commit decides "how much do I trust my current motion vs keep searching", which
        // is a PRECISION, and doctrine §2.3 says precision is a CONTROLLED variable --
        // "a designer picking the crossover point is the anti-pattern".  Three crossover
        // points have now been picked by hand and MEASURED: the original 180/240/90
        // (14% stalled), commit off (20%), and an inverted fast-engage/slow-release
        // (22%).  The hand-tuned original won, which is exactly the situation where the
        // constant should be replaced by the mechanism that ought to set it.
        //
        // The agent's own forward-model residual is that mechanism: commit is EARNED when
        // the body is predicting itself better than it usually does, and released when it
        // is not.  Crucially this can fire INSIDE a 1-2 s burst if the burst is genuinely
        // predictable -- which no fixed 180-tick (3 s) window can ever do, and the
        // mismatch between those timescales is the operator's whole complaint.
        //
        // SCALE-FREE by construction (doctrine §6, "whiten by its own running magnitude"):
        // z is the residual's shortfall against ITS OWN running mean, so nothing here is
        // tuned to tle's absolute size.  cp>1 = predicting better than usual.
        float cp = 1.0f;
        if (commit_prec_gain_ != 0.0) {
            // ⚠ 2026-08-05 · THE ACTIVITY TERM IS NOT OPTIONAL — this was built without it
            // and the operator caught the consequence by eye: commit_prec RISING read as
            // "tentative/unsure" and FALLING as "stepping", the exact inverse of the intent.
            //
            // Because a residual alone measures PREDICTABILITY, and a body that has stopped
            // is trivially predictable.  So: flat ground -> most predictable -> commit
            // hardest -> explore_mult -> 0 -> the noise sustaining the oscillation dies ->
            // it stops; and stopped it stays predictable, so it stays stopped.  A stuck leg
            // or a chassis collision makes the body MORE predictable, not less, which is
            // why the cycle starts exactly there.  Obstacles are unpredictable, so commit
            // stays low and the gait works -- "falls apart on flat ground when the dynamics
            // should be most predictable" is not a paradox, it is the mechanism.
            //
            // This is the freeze trap, and the doctrine entry against it was written THIS
            // MORNING from the leg-lesion data: "on a motor system, prediction error is not
            // a proxy for competence -- ACTIVITY is.  Every 1/(tle+eps) weighting needs an
            // activity term, and the burden is on the proposal to name its own."
            // Ours is amp_ema_mean(): oscillation amplitude, which goes to zero when the
            // thing stops.  Same guard as couple_prec_gain's amp/(tle+eps).
            // CONFIDENCE IS GOAL-RELATIVE.  Without a declared intent it has no meaning,
            // so the lever stays inert rather than falling back on a residual we have
            // measured to ANTI-correlate with moving well.  That is deliberate: a
            // commit_prec_gain with no intent_topic is a no-op, and says so in its diag.
            if (intent_seen_) {
                const float ev = fwd_progress_ema_ - intent_v_;
                const float ew = yaw_rate_ema_     - intent_w_;
                // ⚠ MEASURED DEFECT, 2026-08-05.  The first build summed these RAW and the
                // result was 98% yaw variance: |ew| ran 6.8x |ev| (0.151 vs 0.022) because
                // the forward term is bounded by v* (~0.06) while yaw_rate_ema is unbounded
                // (observed to 0.58).  corr(intent_err, fwd_progress_ema) was -0.002 -- the
                // "am I achieving my intent" scalar carried NO information about forward
                // progress.  And since skid-steer locomotion IS yaw, the error rose exactly
                // when the body walked, so commit shortened precisely when it should hold:
                // stalled ticks 14% -> 32%.  Normalising the SUM (below) is scale-free in
                // aggregate and does nothing about the relative weight of two terms in
                // different units -- that conflation was the bug.
                // Fix per doctrine §5 / CLAUDE.md §0 rule 2: centre out the common mode and
                // normalize each term by ITS OWN running spread before combining, so the
                // mix is set by the body's dynamics rather than by a designer's constant.
                if (ev_spread_ema_ <= 0.0f) ev_spread_ema_ = std::fabs(ev);
                if (ew_spread_ema_ <= 0.0f) ew_spread_ema_ = std::fabs(ew);
                ev_spread_ema_ = (1.0f - kCommitPrecAlpha) * ev_spread_ema_
                               + kCommitPrecAlpha * std::fabs(ev);
                ew_spread_ema_ = (1.0f - kCommitPrecAlpha) * ew_spread_ema_
                               + kCommitPrecAlpha * std::fabs(ew);
                const float zw = float(intent_yaw_gain_) * ew / (ew_spread_ema_ + 1e-6f);
                // ── 2026-08-05, operator UI read: "the error peaks much higher than the
                // amount of velocity lost, and the peaks happen while the robot is actually
                // moving forward."  Both are real, and both are the LIKELIHOOD being wrong
                // rather than the filter needing tuning.  Two corrections, no new constants.
                //
                // (1) THE GOAL IS A HALF-SPACE, NOT A POINT.  A symmetric loss about v*
                //     penalises exceeding the intent exactly as much as falling short, so a
                //     good stride overshoots and the error RISES -- precisely the peaks the
                //     operator saw during forward motion.  A higher loop asking for forward
                //     progress means AT LEAST v*.  Only the shortfall is error; going faster
                //     than asked is not a failure to be explained.  This is the shape of the
                //     preference, not a tuned knob.
                const float zv = std::min(0.0f, ev) / (ev_spread_ema_ + 1e-6f);
                // (2) HEAVY-TAILED, NOT GAUSSIAN.  A quadratic loss IS a Gaussian assumption:
                //     it says a large deviation is near-impossible, so when one happens the
                //     model treats it as overwhelming evidence of incompetence.  With ev
                //     reaching -0.061 against a 0.022 spread, one backward lurch gives
                //     |zv| ~ 2.8 and cp = exp(2.5*z) rails at the 0.2 clamp -- the operator's
                //     "a backwards swing shouldn't hugely penalise the forward".  The robust
                //     answer is a Cauchy/Lorentzian likelihood, whose negative log-likelihood
                //     is log1p(z^2): quadratic near zero (small errors still inform) and
                //     LOGARITHMIC in the tail (a lurch reads as "probably a glitch").  It
                //     also fixes the dynamic range the graph shows -- sqrt() of two terms
                //     each normalized to mean |z| = 1 floors near 1.4 and spikes to 3.7,
                //     compressing the useful region into the bottom third of the axis.
                //     Scale still comes entirely from the running spreads; nothing added.
                // (3) DID THIS STRIDE LOOK LIKE MY STRIDES?  The body's own forward-velocity
                //     waveform, indexed by gait phase and learned online.  Nothing about the
                //     waveform is specified here -- only that it should REPEAT.
                float zr = 0.0f;
                if (intent_rhythm_gain_ > 0.0 && !legs_.empty()) {
                    auto const& L0 = legs_[0];
                    const float ph = (stroke_phase_src_ > 0.0 && L0.step_locked)
                                   ? L0.step_phase : L0.phase;
                    float u = std::fmod(ph, 2.0f * float(M_PI));
                    if (u < 0.0f) u += 2.0f * float(M_PI);
                    const int b = std::min(kFwdProfileBins - 1,
                        int(u / (2.0f * float(M_PI)) * kFwdProfileBins));
                    const float dev = fwd_v_ - fwd_profile_[b];
                    // Learn the profile with a per-bin count-annealed rate (doctrine §5:
                    // count-annealed, not a chosen constant) so early ticks move it fast and
                    // a settled profile stops chasing single strides.
                    const uint32_t n = ++fwd_profile_n_[b];
                    fwd_profile_[b] += dev / float(n < 400u ? n : 400u);
                    if (rhythm_spread_ema_ <= 0.0f) rhythm_spread_ema_ = std::fabs(dev);
                    rhythm_spread_ema_ = (1.0f - kCommitPrecAlpha) * rhythm_spread_ema_
                                       + kCommitPrecAlpha * std::fabs(dev);
                    zr = float(intent_rhythm_gain_) * dev / (rhythm_spread_ema_ + 1e-6f);
                    rhythm_dev_diag_ = std::fabs(dev);
                }
                const float e  = std::log1p(zv * zv + zw * zw + zr * zr);
                if (err_run_ema_ <= 0.0f) err_run_ema_ = e;    // seed, no cold-start spike
                err_run_ema_ = (1.0f - kCommitPrecAlpha) * err_run_ema_ + kCommitPrecAlpha * e;
                // z > 0 = achieving the ASSIGNED task better than its own running average.
                // NOTE z is bounded ABOVE at exactly 1: e >= 0 and err_run_ema_ > 0, so
                // (err_run - e)/err_run <= 1, with equality only at a perfect e == 0.  It is
                // unbounded below.  That asymmetry is real and is why the exponent matters.
                const float z = (err_run_ema_ - e) / (err_run_ema_ + 1e-6f);
                // ── DERIVED, NOT TUNED (doctrine §5; the operator's "this seems like a lot
                // of tuning").  A hand-picked exponent of 2.5 reached the 5.0 clamp at
                // z = ln(5)/2.5 = 0.64, so the top 36% of z's range collapsed onto a single
                // value -- measured as 22% of ticks pinned at cp = 5.0.  Saturation is not a
                // stronger signal, it is a DESTROYED one: inside the rail the mechanism can
                // no longer tell "just achieving" from "achieving spectacularly".
                //   The exponent is not free.  z's own upper bound of 1 must map exactly onto
                // the output's upper bound, which fixes k = ln(kCommitPrecHi).  The clamps
                // being reciprocal then makes the map symmetric in log space: z = +1 -> Hi,
                // z = 0 -> 1 (byte-identical), z = -1 -> Lo.  Nothing left to choose.
                //   commit_prec_gain_ survives ONLY as the A/B lever: 0 = off (cp == 1, the
                // gain-0 guard), 1 = the derived map spanning the full range without ever
                // saturating.  ⚠ Its units therefore CHANGED -- it now multiplies the
                // derived exponent rather than being the exponent.  Pre-existing cprecNN
                // sweep configs carry the old meaning and their numbers do not transfer.
                const float k = float(commit_prec_gain_) * std::log(kCommitPrecHi);
                cp = std::clamp(std::exp(k * z), kCommitPrecLo, kCommitPrecHi);
            }
        }
        commit_prec_diag_ = cp;
        // Predicting well shortens the qualifying window, speeds the ramp and slows the
        // release; predicting badly does the reverse.  cp == 1 => byte-identical.
        const int   eff_window = std::max(1, int(commit_window_ticks_ / cp));
        const float eff_rise   = (1.0f / float(commit_rise_ticks_))  * cp;
        const float eff_decay  = (1.0f / float(commit_decay_ticks_)) / cp;
        if (fwd_progress_ema_ > kCommitVelThresh) {
            if (commit_ticks_ < eff_window * 20) ++commit_ticks_;
        } else {
            commit_ticks_ = 0;
        }
        if (commit_ticks_ >= eff_window)
            commit_boost_ = std::min(1.0f, commit_boost_ + eff_rise);
        else
            commit_boost_ = std::max(0.0f, commit_boost_ - eff_decay);
    } else {
        commit_boost_ = 0.0f;
    }

    // ---- forward-FLOW homeostat (lever D): track flow magnitude + volatility so the
    // stroke can be amplified ∝ magnitude·predictability below.  0 = off (still cheap to
    // keep the EMAs cold; guard so it stays byte-identical when disabled).
    if (forward_flow_gain_ > 0.0) {
        float dev = std::abs(fwd_v_ - flow_ema_);
        flow_ema_     = (1.0f - kFlowEmaAlpha) * flow_ema_     + kFlowEmaAlpha * fwd_v_;
        flow_vol_ema_ = (1.0f - kFlowEmaAlpha) * flow_vol_ema_ + kFlowEmaAlpha * dev;
    } else {
        flow_ema_ = 0.0f; flow_vol_ema_ = 0.0f;
    }

    // ---- Shared C/D factors (computed once per tick, applied at the exploration + stroke
    // sites below).  All 1.0 / 0.0 when both levers are off → byte-identical.
    //   commit (C): explore_mult damps the phase-search σ + noise; stroke gets a thrust add.
    //   flow (D):   flow_quality = magnitude·predictability; amplifies stroke ∝ flow.
    const float commit_amt   = float(progress_commit_gain_) * commit_boost_;      // 0..3
    // ---- 2026-08-04 · THE MISSING INTEGRAL TERM ---------------------------------------
    // The heading controller is P+D only -- a grep for an I term returns nothing.  With a
    // persistent yaw disturbance (which this body HAS: a seed-random skid handedness) a PD
    // settles at a NONZERO steady-state steer, so one side's stroke sits permanently closer
    // to its clamp.  Measured: right legs request 2.10 against a +-1 limit and clip 77% of
    // leg-ticks while the left request 0.70 -- so a turn needing MORE right thrust does
    // nothing and steering authority becomes direction-dependent.  An integrator LEARNS that
    // DC effort and hands it back, freeing the P term for transients.  It targets FUNCTIONAL
    // symmetry (zero net yaw), not amplitude symmetry -- which is what the ledger says any
    // symmetry retry must do, the ~35-lever symmetry family having died on amplitude matching.
    // ⚠ LEAK AND CLAMP ARE MANDATORY: unbounded integrator windup is this codebase's
    // characteristic failure shape (three documented cases).
    if (heading_trim_rate_ != 0.0) {
        const float berr = gb_seen_ ? std::atan2(gb_x_, gb_y_) * 0.318309886f
                                    : -heading_bearing_;
        heading_trim_ += float(heading_trim_rate_) * berr;
        heading_trim_ *= (1.0f - float(heading_trim_leak_));
        heading_trim_  = std::clamp(heading_trim_, -kHeadTrimMax, kHeadTrimMax);
    }
    // ⚠ FLOOR, not zero: "play never abstains" -- fix exploration's OUTPUT, never its right
    // to win.  explore_mult reaching exactly 0.000 is what makes the body frozen-but-
    // confident, and it is the state the operator reads as indecision.  0 = legacy.
    float explore_mult = std::max(float(explore_floor_), 1.0f - commit_amt);                 // C: damp exploration

    // ---- HOMEOKINETIC SUPPORT SELECTOR (support_select_gain) --------------------------
    // Value the CURRENT support state by how much the body's own action moves its own
    // sensors, divided by how badly it currently predicts itself.  Then explore MORE when
    // that value is below what this body typically achieves.  See the header for why the
    // criterion is responsiveness and not forward progress, and why the divisor matters.
    support_mult_diag_ = 1.0f;
    // Instantaneous responsiveness: sensor change per unit COMMANDED change, summed
    // over every leg and joint.  Both terms egocentric; the ratio is scale-free.
    // Computed UNCONDITIONALLY (2026-08-09): value = responsiveness/(motor_tle+ε) is the
    // criterion the actuator search scores against (ledger ★ open problem), so every arm
    // must log it — not only arms running the selector.  Selector behaviour is unchanged:
    // explore_mult modulation stays behind support_select_gain below.
    bool resp_fresh = false;   // EMA below must only ingest a THIS-tick ratio, never a stale diag
    {
        double dx = 0.0, du = 0.0;
        for (auto const& L : legs_) {
            if (!L.initialized || L.x.size() != L.prev_x.size()) continue;
            for (int k = 0; k < L.x.size(); ++k) dx += std::fabs(L.x[k] - L.prev_x[k]);
            if (L.prev_y.size() == L.prev_prev_y.size())
                for (int k = 0; k < L.prev_y.size(); ++k)
                    du += std::fabs(L.prev_y[k] - L.prev_prev_y[k]);
        }
        if (du > 1e-6) { support_resp_diag_ = float(dx / du); resp_fresh = true; }
    }
    if (support_select_gain_ > 0.0 && have_contact_ && int(foot_contact_.size()) == n_legs_) {
        int planted = 0;
        for (int i = 0; i < n_legs_; ++i) if (foot_contact_[i] >= 0.5f) ++planted;
        planted = std::clamp(planted, 0, kSupportBins - 1);
        if (resp_fresh) {
            const float inst = support_resp_diag_;
            float& e = resp_ema_[planted];
            e = (resp_seen_[planted] == 0) ? inst : (1.0f - kRespAlpha) * e + kRespAlpha * inst;
            ++resp_seen_[planted];
        }
        support_bin_diag_ = planted;
        // Value, and a SCALE-FREE comparison against what this body typically achieves --
        // never a hand-set target (doctrine rule 5).  Only VISITED bins count, so an
        // unentered support state cannot drag the mean.
        const float tle = tle_ema_mean();
        const float val = resp_ema_[planted] / (tle + 1e-3f);
        double sum = 0.0; int nb = 0;
        for (int k = 0; k < kSupportBins; ++k)
            if (resp_seen_[k] > kRespWarmup) { sum += resp_ema_[k] / (tle + 1e-3f); ++nb; }
        if (nb >= 2 && resp_seen_[planted] > kRespWarmup) {
            const float mean = float(sum / nb);
            const float ratio = (mean > 1e-6f) ? val / mean : 1.0f;
            // ratio < 1 => this support state is LESS responsive than typical => explore more.
            float mult = 1.0f + float(support_select_gain_) * (1.0f - ratio);
            mult = std::clamp(mult, kSupportMultMin, kSupportMultMax);
            support_mult_diag_  = mult;
            support_value_diag_ = ratio;
            // ⚠ FLOOR PRESERVED: "play never abstains".  Scaling can raise or lower the
            // probe but never removes exploration's right to win.
            explore_mult = std::max(float(explore_floor_), explore_mult * mult);
        }
    }
    explore_mult_diag_ = explore_mult;   // diag: is the coordination probe already damped to 0?
    float flow_quality = 0.0f;
    if (forward_flow_gain_ > 0.0) {
        flow_quality = std::clamp(flow_ema_, 0.0f, kFlowVelNorm) / kFlowVelNorm;  // magnitude 0..1
        flow_quality /= (1.0f + kFlowVolK * flow_vol_ema_);                       // × predictability
    }
    const float lever_stroke_mult = (1.0f + kCommitStrokeFrac * commit_amt)       // C: thrust add
                                   * (1.0f + float(forward_flow_gain_) * flow_quality); // D: flow amp
    flow_quality_diag_ = flow_quality;   // for the diag block

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
            if (coord_fitness_mode_ == 1) {
                // ---- REWARD-FREE fitness (2026-07-25).  Mode 0 below ranks probes by
                // FORWARD VELOCITY — a task reward, which §5.1 forbids and which gives the
                // ratchet something destructive to lock in (a thrash that scores high while
                // escaping becomes the incumbent).  This mode ranks the SAME probes by a
                // purely intrinsic quantity: how well the body's own sensorimotor loop works
                // under those offsets.  No position, distance, or velocity term appears.
                //
                //   fitness = coherence · activity / (1 + self-model error)
                //
                // Every factor is load-bearing, because the two obvious signals are BOTH
                // degenerate on their own and degenerate in the SAME direction:
                //   * coherence alone → a FROZEN body wins.  φ = atan2(0,0) = 0 for every
                //     leg, so the Kuramoto order parameter reads 1.0 on a corpse.
                //   * 1/(1+tle) alone → a frozen body wins again: a still body is the
                //     least surprising thing a forward model can predict (this is exactly
                //     why Cphi was deliberately NOT trained on HK surprise — see the header).
                //   * activity (mean per-leg oscillation amplitude) is the homeokinetic
                //     normalisation that kills both: it → 0 when the body stops, so no
                //     frozen solution can win however predictable or "coherent" it looks.
                // A thrash loses too: high activity but scattered phases (low coherence) and
                // high prediction error. What wins is COHERENT, WELL-MODELLED MOTION.
                //
                // Precision form 1/(tle+ε) is the same weighting the LateralVoter fuses on
                // (§0) — trust earned from predictive accuracy.
                const float coh = gait_coherence();          // ∈[0,1] phase-lock quality
                const float act = amp_ema_mean();            // oscillation amplitude
                const float tle = tle_ema_mean();            // forward-model error
                // Wobble stays penalised: tilt is a VIABILITY/homeostatic term, not a task
                // reward, and it keeps the search on the alive side of the frontier.  The
                // lateral-velocity penalty is dropped here — "don't crab" is a task
                // preference, not an intrinsic one.
                coord_fit_accum_ += coh * act / (1.0f + tle)
                                  - float(coord_stab_penalty_) * wob;
            } else {
                float reward_v = fwd_v_;
                if (coord_intent_nav_ > 0.0) {
                    float tc_mag = std::sqrt(tc_x_ * tc_x_ + tc_y_ * tc_y_);
                    if (tc_mag > 0.1f)                                   // target present
                        reward_v = lateral_v_ * tc_x_ + fwd_v_ * tc_y_;  // velocity toward intent
                }
                coord_fit_accum_ += reward_v - float(coord_stab_penalty_) * wob
                                             - float(coord_lat_penalty_) * std::fabs(lateral_v_);
            }
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
            // stuck→explore: enlarge the probe step when forward progress has stalled, so
            // the search jumps OUT of a slow local optimum toward a propulsive coordination.
            float coord_sigma = float(coord_reward_drive_) * (1.0f + float(stuck_explore_gain_) * stuck_boost_)
                              * explore_mult;   // progress→commit (C) damps the phase-search σ
            std::normal_distribution<float> pz(0.0f, coord_sigma);
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
    if (cruse_gain_ != 0.0 || cruse_rule5_gain_ != 0.0 || stance_lift_gain_ != 0.0) update_cruse_state();
    // Stroke-to-step lock: advance each leg's touchdown-referenced step clock.  Must run
    // BEFORE the per-leg output loop (the stroke reads L.step_phase) and AFTER the contact
    // /torque handlers have landed this tick's sensor values.  Skipped entirely at the
    // default 0, so L.step_phase stays at its init and nothing downstream can read it.
    if (stroke_phase_src_ > 0.0) update_step_phase(tick_id);
    // Measurement only, and skipped entirely at the default 0 — see the function header.
    if (gait_align_diag_ > 0.0) update_gait_align_diag(tick_id);
    // Pure observation for the live inspector; skipped entirely at the default 0.
    if (gait_raster_diag_ > 0.0) update_gait_raster();
    // Purchase gate for the power stroke (needs the cross-leg mean, so once per tick,
    // before the per-leg loop).  Self-guards to gate ≡ 1 when the gain is 0.
    update_stroke_load_gate();
    // "did the consumer fire?" as a number — a gate has shipped here as silent dead code
    // before (CLAUDE.md §3.2 rule 5).  0 with the gains non-zero ⇒ contact_topic is unwired.
    swing_tuck_frac_ = swing_tuck_ticks_ ? float(double(swing_tuck_hits_) / double(swing_tuck_ticks_)) : 0.0f;

    // Propulsive-credit homeostat: group-mean credit (from last tick's per-leg
    // updates) so a below-mean "dragging" leg can be boosted this tick.
    if (propulsion_balance_gain_ > 0.0) {
        float s = 0.0f; int c = 0;
        for (int j = 0; j < n_legs_; ++j)
            if (legs_[j].initialized) { s += legs_[j].prop_credit; ++c; }
        prop_credit_mean_ = c ? s / float(c) : 0.0f;
    }

    // IMPORT I7: one controller across every joint, computed BEFORE the per-leg loop so
    // each leg can slice its own commands out of it.  Requires every leg fresh this tick
    // (they share one bridge publish, so they are) — otherwise the concatenated state
    // would mix ticks.  Everything downstream (postural, stroke, coupling, noise, clamp)
    // is untouched: only the source of the pre-tanh operating point changes.
    bool wb_on = false;
    if (whole_body_c_ > 0.0) {
        bool all_fresh = true;
        for (int leg = 0; leg < n_legs_; ++leg)
            if (!legs_[leg].initialized || !legs_[leg].fresh
                || legs_[leg].n != legs_[0].n) { all_fresh = false; break; }
        if (all_fresh) {
            if (!wb_ready_) wb_init(legs_[0].n);
            wb_learn_and_control();
            wb_on = true;
        }
    }

    for (int leg = 0; leg < n_legs_; ++leg) {
        Leg& L = legs_[leg];
        if (!L.initialized || !L.fresh) continue;
        L.fresh = false;
        L.steps_seen += 1;
        int n = L.n;
        bool warmup = (wb_on ? (wb_steps_ <= babble_ticks_)
                             : (L.steps_seen <= babble_ticks_));

        // ---- Learn from the previous command's outcome (motor TLE) ----
        // The MODEL always learns (also during babble warmup, where it learns the
        // body's response to small random commands).  The CONTROLLER (HK + anti-
        // saturation) only learns after warmup, once the model can predict.
        if (L.have_prev && !wb_on) {
            Eigen::VectorXf x_hat = L.A * L.prev_y + L.b;          // forward-model prediction
            Eigen::VectorXf xi    = L.x - x_hat;                   // motor TLE ξ
            // (1) model descent: A += η_M ξ yᵀ ; b += η_M ξ
            L.A.noalias() += float(model_lr_) * xi * L.prev_y.transpose();
            L.b.noalias() += float(model_lr_) * xi;
            L.tle_ema = (1.0f - kTeleEmaAlpha) * L.tle_ema + kTeleEmaAlpha * xi.norm();

            if (!warmup) {
                const bool embed = cpg_embed_ && cpg_seen_;
                Eigen::VectorXf z = L.C * L.prev_x + L.h;          // operating point
                if (embed) {
                    z.noalias() += L.Cphi * L.prev_phi_ctx;        // posture phase-conditioned bias at command time
                    z.noalias() += L.Cvel * L.prev_phi_ctx;        // velocity feed-forward bias (0 until the socket trains it)
                }
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
                // Objective retarget (L-1b socket, §1.1/§2.4): if a fresh soft posture
                // target is present for this leg, the CONTROLLER descends a blended error
                // ξ̃ = (1−w)·ξ + w·(x − x*) on the joint-POSITION components (index 3j) —
                // it LEARNS to reach x*, an objective-change, not an additive output bias.
                // The MODEL (A,b) above kept the raw ξ (self-model stays honest).  w=0 or
                // no objective → ξ̃ = ξ → byte-identical HK.
                Eigen::VectorXf xi_tilde = xi;
                if (obj_seen_[leg] && obj_weight_[leg] > 0.0f
                    && obj_target_[leg].size() == m && n >= 3 * m) {
                    float w = obj_weight_[leg];
                    for (int j = 0; j < m; ++j) {
                        int idx = 3 * j;                                  // joint j position
                        float goal_err = L.x[idx] - obj_target_[leg][j]; // x − x*
                        xi_tilde[idx] = (1.0f - w) * xi[idx] + w * goal_err;
                    }
                }
                // ---- DEP: C from the correlation of MOTOR and SENSOR derivatives -----
                // Causal pairing: the command CHANGE we made last tick (Δprev_y) and the
                // sensor CHANGE it produced (Δx).  Row-normalised so dep_gain is the
                // per-motor loop gain — without normalisation the correlation grows
                // without bound and the loop diverges, which is the first thing that goes
                // wrong with a naive Hebbian rule on a closed sensorimotor loop.
                if (dep_gain_ > 0.0) {
                    const Eigen::VectorXf dx = L.x - L.prev_x;
                    const Eigen::VectorXf dy = L.prev_y - L.prev_prev_y;
                    const float a_dep = float(dep_alpha_);
                    L.Cdep = (1.0f - a_dep) * L.Cdep + a_dep * (dy * dx.transpose());
                    for (int i = 0; i < m; ++i) {
                        const float rn = L.Cdep.row(i).norm();
                        if (rn > 1e-7f) L.C.row(i) = L.Cdep.row(i) * (float(dep_gain_) / rn);
                    }
                }
                Eigen::VectorXf q  = P * xi_tilde;
                Eigen::MatrixXf dC = 2.0f * float(ctrl_lr_)
                                     * (AG.transpose() * q) * (q.transpose() * Lp);
                // I2 — the CONFINING term, ported from sos_avggrad.cpp's `epsrel`.
                // Dimensions follow PM exactly with q qᵀ standing in for their averaged Q:
                // C(m×n)·(q qᵀ)(n×n)·A(n×m) → m×m, take the diagonal, weight by g' and
                // 2·sense, then subtract (epsrel ⊙ y)·xᵀ.  This is the term the anti-
                // saturation hack was approximating.
                if (sense_ > 0.0) {
                    Eigen::MatrixXf CqqA = L.C * (q * q.transpose()) * L.A;   // m x m
                    Eigen::VectorXf eps(m), yt(m);
                    for (int i = 0; i < m; ++i) {
                        eps[i] = CqqA(i, i) * G(i, i) * 2.0f * float(sense_);
                        yt[i]  = std::tanh(z[i]);
                    }
                    dC.noalias() -= float(ctrl_lr_) * (eps.cwiseProduct(yt)) * L.prev_x.transpose();
                }
                float dC_norm = dC.norm();
                if (max_dctrl_ > 0.0 && dC_norm > float(max_dctrl_))
                    dC *= float(max_dctrl_) / dC_norm;             // ignition clamp
                if (dep_gain_ <= 0.0) L.C.noalias() += dC;         // DEP owns C when on
                Eigen::VectorXf mu = G * (L.A.transpose() * q);    // bias toward less surprise
                L.h.noalias() += float(bias_lr_) * mu;
                // Phase-conditioned feed-forward: train Cphi to REDUCE the keyframe error (x* − x)
                // at the command phase — NOT HK surprise (which damps motion).  Self-limiting: as
                // the bias moves the body toward x*, the error shrinks and learning stops.  Small
                // L2 decay bounds it.  This is the "alternate learning signal".
                if (embed && obj_seen_[leg] && obj_weight_[leg] > 0.0f
                    && obj_target_[leg].size() == m && n >= 3 * m) {
                    for (int j = 0; j < m; ++j) {
                        float e = obj_target_[leg][j] - L.x[3 * j];   // x* − x toward the keyframe posture
                        L.Cphi.row(j).noalias() += float(embed_lr_) * e * L.prev_phi_ctx.transpose();
                    }
                    if (embed_decay_ > 0.0) L.Cphi *= (1.0f - float(embed_decay_));
                }
                // Velocity feed-forward (the propulsive push): train Cvel to reduce the VELOCITY
                // error (v* − ẋ) at the command phase, where ẋ = the joint delta (state index 3j+2).
                // Same self-limiting form + L2 bound as Cphi, but the target is the phase-indexed
                // velocity, not the pose — so it keeps driving even when the posture error is 0
                // (moving THROUGH the pose = propulsion).  Zero + inert unless the velocity socket
                // is wired (byte-identical HK otherwise).
                if (embed && obj_vel_seen_[leg] && obj_vel_weight_[leg] > 0.0f
                    && obj_vel_target_[leg].size() == m && n >= 3 * m) {
                    for (int j = 0; j < m; ++j) {
                        float ev = obj_vel_target_[leg][j] - L.x[3 * j + 2];  // v* − ẋ (delta component)
                        L.Cvel.row(j).noalias() += float(embed_lr_) * ev * L.prev_phi_ctx.transpose();
                    }
                    if (embed_decay_ > 0.0) L.Cvel *= (1.0f - float(embed_decay_));
                }

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
                // I2 companion: the explicit bound on C and h that lets sat_lr be retired
                // without reproducing its windup.  PM's `damping`.
                if (ctrl_damping_ > 0.0) {
                    L.C *= (1.0f - float(ctrl_damping_));
                    L.h *= (1.0f - float(ctrl_damping_));
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
            // ── RUNG 1: ACT ON THE PREDICTED STATE ──────────────────────────────
            // The forward model x̂ = A·y + b is already learned online, but today it is
            // consumed ONLY to form the residual: the loop predicts to LEARN and never to
            // ACT (docs/plans-and-designs/motor_layer_is_reactive.md).  The controller
            // reads x_t, so it always acts on stale state -- and every loop has delay
            // (sensing, filtering, actuation).  Feeding it x̂ instead compensates ALL of
            // that with a quantity we already compute each tick.  It also subsumes the
            // phase-filter failures: L.phase times the power stroke, so lag there fires
            // the stroke late; the general fix is to act ahead, not to filter better.
            //
            // ⚠ THE CIRCULARITY.  x̂_{t+1} = A·y_t + b needs y_t, which is what we are
            // computing.  Two resolutions, both shipped so they can be compared:
            //   mode 0 (default) FIXED POINT -- one Jacobi iteration: y⁰ from x, then
            //     x̂ from y⁰, then y¹ from x̂.  A true one-step lookahead, one extra matmul.
            //   mode 1 PREV-ACTION -- x̂ = A·prev_y + b.  Assumes action continuity; the
            //     CHEAP CONTROL.  If it matches mode 0, the effect is not lookahead.
            // lookahead_null drops the A·y term (x̂ := b), testing whether the benefit is
            // the model's DYNAMICS or merely shrinking x toward a constant.
            // gain 0 => x_eff == x exactly => byte-identical to MotorEPM.
            y = wb_on ? Eigen::VectorXf(Zw_.segment(leg * m, m))   // I7: this leg's slice
                      : Eigen::VectorXf(L.C * L.x + L.h);
            if (!wb_on && lookahead_gain_ != 0.0 && L.A.size() > 0 && L.have_prev) {
                const float lam = float(lookahead_gain_);
                Eigen::VectorXf y0 = y;                       // pre-tanh operating point
                for (int j = 0; j < m; ++j) y0[j] = std::tanh(y0[j]);
                const Eigen::VectorXf& yref =
                    (lookahead_mode_ >= 1.0) ? L.prev_y : y0;  // mode 1 = the cheap control
                Eigen::VectorXf xhat = (lookahead_null_ > 0.0)
                    ? Eigen::VectorXf(L.b)                     // null: no dynamics term
                    : Eigen::VectorXf(L.A * yref + L.b);
                if (xhat.size() == L.x.size()) {
                    const Eigen::VectorXf x_eff = (1.0f - lam) * L.x + lam * xhat;
                    y = L.C * x_eff + L.h;
                    la_dev_ema_ = (1.0f - kTeleEmaAlpha) * la_dev_ema_
                                + kTeleEmaAlpha * (x_eff - L.x).norm();
                }
            }
            if (cpg_embed_ && cpg_seen_) {
                Eigen::Vector2f ctx(std::cos(cpg_phase_), std::sin(cpg_phase_));
                y.noalias() += L.Cphi * ctx;   // posture feed-forward (pose)
                y.noalias() += L.Cvel * ctx;   // velocity feed-forward (propulsive push; 0 until trained)
            }
            for (int j = 0; j < m; ++j) y[j] = mg * ag * std::tanh(y[j]);
            // Phase-0 saturation instrument: HK's own contribution, BEFORE any of the
            // additive terms below and before the ±1 clamp at the end of this block.
            if (int(sat_hk_abs_.size()) != m) {
                sat_hk_abs_.assign(m, 0.0);   sat_pre_abs_.assign(m, 0.0);
                sat_pre_max_.assign(m, 0.0);  sat_clip_hits_.assign(m, 0.0);
                sat_n_ = 0.0;
            }
            for (int j = 0; j < m; ++j) sat_hk_abs_[j] += std::fabs(double(y[j]));
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
            // Per-joint spring: the array is a RELATIVE profile (multiplier) on the
            // global tone scalar postural_gain — so postural_gain stays an honest
            // global knob (turning it up firms EVERY joint) and the array only shapes
            // the profile.  e.g. profile [1,0.3,1] LOOSENS hip2 to 30% of the global
            // tone so the learned C/Cphi stroke rides ON TOP of the reflex (the
            // height-homeostat push-up stays full strength below), while hip1/knee
            // stay at full tone for stance.  Empty/mis-sized array = 1.0 (uniform).
            const bool pjg = int(postural_gain_joints_.size()) == m;
            for (int j = 0; j < m; ++j) {
                float g = float(postural_gain_) * (pjg ? float(postural_gain_joints_[j]) : 1.0f);
                if (g > 0.0f) y[j] -= g * (L.x[3 * j] - L.rest_pos[j]);
            }
        }
        // Chassis-height homeostat output: drive the femur-LIFT joint (hip2, index
        // 1) — NOT the knee.  The knee carries the stepping rhythm (HK + coupling);
        // a DC bias there clamps the swing and kills the gait (the body fought the
        // dead spider pose).  hip2 angles the upper leg down → with feet planted the
        // hip (chassis) rises — height set on a joint SEPARATE from the rhythm.
        // Sign empirically confirmed (positive height_bias raises chassis_y).
        // Post-warmup so motor babble can explore upward and discover the ceiling.
        // ⚠ THE COMMENT ABOVE RECORDS A PRIOR REFUTATION of driving the knee from
        // here: "a DC bias there clamps the swing and kills the gait (the body fought
        // the dead spider pose)".  That verdict stands FOR A DC BIAS.  height_lift_knee
        // is not one: `lift` is multiplied by height_rest_frac_, which is ~0 whenever
        // the body is cruising (it fully fades at fwd_progress 0.025 and the body runs
        // ~0.049), so this acts at REST and during STAND-UP and is absent from the gait
        // it was previously found to clamp.  Same distinction that made the stance-gated
        // knee tuck work where a blind DC knee bias killed the gait (CLAUDE.md §1).
        // The swing metrics are therefore the ones that must be read on any A/B of it.
        if (!warmup && height_homeo_gain_ > 0.0 && m >= 2) {
            const float lift = kHeightLiftSign * height_bias_ * height_rest_frac_;
            y[1] += lift;                                    // fade lift out while moving
            // Same sign to the knee — see height_lift_knee.  knee+ tucks, which in
            // spider stance suspends the body higher; knee- would un-tuck and cancel.
            if (height_lift_knee_ > 0.0 && m >= 3)
                y[m - 1] += float(height_lift_knee_) * lift;
        }
        // Cruse/Walknet inter-leg coordination — v2 SEQUENCED LIFT.  Drives hip2 AND
        // knee (they share foot-height authority; either alone is too weak — measured
        // corr(foot_y,joint)≈0.27) to actually plant/clear the foot, not just DC-bias.
        // +cmd = plant firmly (foot down), −cmd = lift CLEAR (foot up): a leg held by a
        // neighbour plants to keep the support polygon; an unheld leg in its own swing
        // lifts clear so its hip1 forward stroke is a free swing, NOT a loaded pull
        // (the operator's swing-becomes-pull → yaw observation).
        if (!warmup && cruse_gain_ != 0.0 && m >= 2 && int(in_swing_.size()) == n_legs_) {
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
            cruse_bias_acc_ += std::fabs(float(cruse_gain_) * cmd);
            ++cruse_bias_n_;
        }
        // Rule 5 (load distribution) — independent of the v2 lift so it can be tested
        // alone.  A STANCE leg presses its foot down ∝ how many OTHER legs are swinging
        // (taking up their shed weight) → more normal force → more friction → less scrub.
        if (!warmup && cruse_rule5_gain_ != 0.0 && m >= 2 && int(in_swing_.size()) == n_legs_
            && !in_swing_[leg]) {
            int n_sw = 0;
            for (int j = 0; j < n_legs_; ++j) if (j != leg && in_swing_[j]) ++n_sw;
            float load = float(cruse_rule5_gain_) * float(n_sw);
            y[1]     += load;    // +hip2 = press foot down (load for grip)
            y[m - 1] += -load;   // knee extends down with it
        }
        // STANCE-LIFT (belly-up while walking): a constant KNEE bias on PLANTED legs
        // only — raise the chassis off the feet it can push against without touching the
        // swing legs' rhythm.  Knee-only (no hip2 → no foot-lift traction loss).  Sign
        // set empirically (which knee dir raises the chassis on a planted foot).
        if (!warmup && stance_lift_gain_ != 0.0 && m >= 2 && int(in_swing_.size()) == n_legs_
            && !in_swing_[leg]) {
            // STROKE-DIRECTION-AWARE RELEASE (stance_release_frac; see the param text).
            // Recovery onset = the leg's OWN commanded hip1 delta flips sign mid-stance;
            // from then until liftoff the press is faded, because continuing to press a
            // foot whose stroke has turned around only buys braking shear (measured
            // −0.004…−0.015 g/tick) and a delayed lift.  The deadband is a numerical
            // guard against zero-chatter, not a tuned threshold: commanded per-tick
            // deltas run ~0.1 here (max_dctrl caps at 0.05 per joint pre-gain).
            float press = 1.0f;
            if (stance_release_frac_ > 0.0 && leg < int(sr_released_.size())
                && L.prev_y.size() > 0 && L.prev_prev_y.size() > 0) {
                if (!sr_was_stance_[leg]) {          // touchdown: start bout pressed
                    sr_released_[leg] = false;
                    sr_prev_dh1_[leg] = 0.0f;
                }
                sr_was_stance_[leg] = true;
                const float dh1 = L.prev_y[0] - L.prev_prev_y[0];
                constexpr float kSrDead = 2e-4f;
                if (std::fabs(dh1) > kSrDead) {
                    if (sr_prev_dh1_[leg] != 0.0f && dh1 * sr_prev_dh1_[leg] < 0.0f)
                        sr_released_[leg] = true;    // commanded reversal → recovery
                    sr_prev_dh1_[leg] = dh1;
                }
                ++sr_stance_ticks_;
                if (sr_released_[leg]) {
                    press = 1.0f - float(stance_release_frac_);
                    ++sr_release_ticks_;
                }
            }
            y[m - 1] += press * float(stance_lift_gain_);
            // Same sign to hip2 — see stance_lift_hip2.  On a PLANTED foot hip2+ presses
            // down and levers the chassis up; it is the other half of the same raise.
            if (stance_lift_hip2_ != 0.0)
                y[1] += press * float(stance_lift_hip2_) * float(stance_lift_gain_);
        } else if (stance_release_frac_ > 0.0 && leg < int(sr_was_stance_.size())
                   && int(in_swing_.size()) == n_legs_ && in_swing_[leg]) {
            sr_was_stance_[leg] = false;             // swing: arm the next bout's reset
        }
        // TIBIA-PLUMB — hip2 nulls the shank's deviation from vertical, so the knee's
        // gait drive TRANSLATES the foot instead of arcing it.  See the header.  Applied to
        // every leg, stance and swing alike: it is a postural constraint, not a phase gate.
        if (!warmup && tibia_plumb_gain_ != 0.0 && m >= 3 && L.n >= 3 * m) {
            const float th = float(tibia_plumb_scale_) * L.x[3] + L.x[6]
                           + float(tibia_plumb_offset_);
            y[1] -= float(tibia_plumb_gain_) * th;
        }
        // SWING TUCK — the mirror of the block above.  stance_lift biases the KNEE on
        // PLANTED legs; this folds hip2 + knee on LIFTED ones so the limb's mass comes
        // inboard and sweeping it forward stops dumping yaw into the chassis.  See the
        // header for why the gate is TRUE CONTACT and not the foot-height detector.
        // Both gains 0 = byte-identical; unwired contact_topic = inert (and observable).
        if (!warmup && (swing_tuck_hip2_ != 0.0 || swing_tuck_knee_ != 0.0) && m >= 3
            && have_contact_ && int(foot_contact_.size()) == n_legs_) {
            ++swing_tuck_ticks_;
            if (foot_contact_[leg] <= 0.5f) {          // this foot is off the ground
                y[1]     += float(swing_tuck_hip2_);   // fold the femur up
                y[m - 1] += float(swing_tuck_knee_);   // fold the shank under
                ++swing_tuck_hits_;
            }
        }
        // Persistent exploration noise (post-warmup): keeps ξ alive at fixed
        // points so HK amplifies it into oscillation instead of freezing.
        // Panic adds exploration noise on top of the persistent drive (flailing).
        // stuck→explore: add undirected motor noise ∝ the stall boost so the whole gait
        // shakes loose (alongside the coord phase-search enlargement above).
        float noise_sigma = float(explore_noise_) * (1.0f + float(stuck_explore_gain_) * stuck_boost_) * explore_mult
                          + pe * float(panic_noise_);   // C damps explore_noise (not panic)
        if (!warmup && noise_sigma > 0.0f) {
            std::normal_distribution<float> nz(0.0f, noise_sigma);
            for (int j = 0; j < m; ++j) y[j] += nz(L.babble_rng);
        }
        // INTER-LEG PLV accumulation — one sample per tick per leg pair, over the whole run.
    // Placed after the phase pre-pass so every leg's phase is current.  Report-only.
    // ⚠ GATED ON ACTUAL OSCILLATION.  PLV asks whether a leg PAIR holds a constant relative
    // phase — and a FROZEN body satisfies that trivially: motionless legs have constant
    // phases, so their difference is constant and PLV -> 1.  Measured 2026-08-03: the
    // sense=1.5 arm takes 7 steps in 40k ticks and reports plv 0.72, against 0.05 for a
    // vigorously stepping control.  That is the frozen-body degeneracy, the SAME failure
    // that made gait_coherence useless, arriving by a different route.  So a pair is only
    // sampled while BOTH its legs are genuinely oscillating: the phase vector's magnitude
    // (the oscillation amplitude) must clear a floor.  plv_n_ then also reports how much
    // support the number has — a PLV backed by few samples is not a measurement.
    if (n_legs_ > 1) {
        bool ok = true;
        for (int i = 0; i < n_legs_; ++i) if (!legs_[i].initialized) { ok = false; break; }
        if (ok) {
            bool any = false;
            for (int i = 0; i < n_legs_; ++i)
                for (int j = i + 1; j < n_legs_ && j < 4; ++j) {
                    const int k = i * 4 + j;
                    const bool admit = !(legs_[i].amp_ema < kPlvAmpFloor
                                      || legs_[j].amp_ema < kPlvAmpFloor);
                    // WINDOWED phasor: decay EVERY tick, add only when admitted.  A pair
                    // that stops oscillating decays to 0 rather than freezing at a stale
                    // value -- see the header note on the frozen-body degeneracy.
                    plv_win_cos_[k] *= (1.0 - kPlvWinAlpha);
                    plv_win_sin_[k] *= (1.0 - kPlvWinAlpha);
                    plv_win_sup_[k] *= (1.0 - kPlvWinAlpha);
                    if (!admit) continue;               // one of them is not moving
                    const float d = legs_[i].phase - legs_[j].phase;
                    plv_win_cos_[k] += kPlvWinAlpha * std::cos(d);
                    plv_win_sin_[k] += kPlvWinAlpha * std::sin(d);
                    plv_win_sup_[k] += kPlvWinAlpha;
                    plv_cos_[k] += std::cos(d);
                    plv_sin_[k] += std::sin(d);
                    plv_pair_n_[k] += 1;
                    any = true;
                }
            if (any) ++plv_n_;
        }
    }

    // Rung 3 inter-leg coupling (post-warmup): Kuramoto bias injected into the
        // knee (the propulsive joint; HK's within-leg coordination carries the rest).
        // c_i = K · mean_{j≠i} sin( (φ_j − φ_i) − (P_j − P_i) ) pulls each leg's
        // phase toward the gait offset relative to every other leg — entrains the
        // twitchers and phase-locks all four.
        if (leg == 0 && n_legs_ > 1 && int(gait_phase_.size()) == n_legs_) {
            // Kuramoto order parameter over gait-offset-corrected phases: 1 = phase-locked,
            // 0 = incoherent.  Report-only.
            float sc = 0.0f, ss = 0.0f; int nn = 0;
            for (int j = 0; j < n_legs_; ++j) {
                if (!legs_[j].initialized) continue;
                const float a = legs_[j].phase - float(gait_phase_[j]);
                sc += std::cos(a); ss += std::sin(a); ++nn;
            }
            couple_R_diag_ = nn ? std::sqrt(sc * sc + ss * ss) / float(nn) : 0.0f;
        }
        if (!warmup && coupling_eff_ > 0.0f && n_legs_ > 1 && int(gait_phase_.size()) == n_legs_) {
            // couple_prec_gain (k) turns the UNIFORM neighbour mean into a PRECISION-WEIGHTED
            // one — see the header note.  k must be tested against `!= 0.0`, never `> 0.0`:
            // the negative arm is this lever's wrong-sign control, and a `> 0.0` guard has
            // already silently eaten a negative gain three times in this file (ledger 4).
            const bool prec_on = (couple_prec_gain_ != 0.0);
            float c = 0.0f, wsum = 0.0f, wmin = 3.4e38f, wmax = 0.0f; int wn = 0;
            for (int j = 0; j < n_legs_; ++j) {
                if (j == leg || !legs_[j].initialized) continue;
                float dphi = (legs_[j].phase - L.phase)
                           - (float(gait_phase_[j]) - float(gait_phase_[leg]));
                float w = 1.0f;
                if (prec_on) {
                    // Neighbour j's OWN precision: how well it predicts itself, scaled by
                    // whether it is actually moving.  legs_[j].tle_ema for j > leg carries
                    // last tick's value (the leg loop is sequential) — the same one-tick
                    // skew legs_[j].phase already has, and negligible at tau ~ 50 ticks.
                    const float prec = legs_[j].amp_ema
                                     / (legs_[j].tle_ema + kCouplePrecEps);
                    w = std::pow(std::max(1e-6f, prec), float(couple_prec_gain_));
                    w = std::clamp(w, kCoupleWMin, kCoupleWMax);
                }
                c += w * std::sin(dphi);
                wsum += w; wmin = std::min(wmin, w); wmax = std::max(wmax, w); ++wn;
            }
            if (prec_on && wn > 0) {   // report-only consumer check (CLAUDE.md 3.2 rule 5)
                const float wmu = wsum / float(wn);
                if (wmu > 1e-12f) { cw_spr_acc_ += double(wmax - wmin) / double(wmu); ++cw_n_; }
            }
            // GAIN-0 GUARD, exact: at k = 0 every w is 1, so c is the legacy sum of sines and
            // the divisor is the legacy (n_legs_ - 1) — not wsum, which would differ from it
            // during the transient where a leg is not yet initialised.
            const float denom = prec_on ? std::max(1e-6f, wsum) : float(n_legs_ - 1);
            // Panic decouples the legs (break out of the stuck phase-lock).
            y[m - 1] += (1.0f - pe) * coupling_eff_ * c / denom;
            // ── RESONANCE FEEDBACK (fwd_resonance_gain).  Couple the leg oscillator to the
            // frequency the BODY ACTUALLY PROPELS AT, closing the loop the operator asked
            // for: motion -> fwd_v oscillation -> the resonator locks -> the legs entrain to
            // it -> the stroke lands where propulsion actually happens -> more motion.  The
            // reference is measured from the body's own forward velocity, not injected, so
            // this reinforces whatever rhythm the body has FOUND rather than dictating one.
            // 0 = off, byte-identical.
            if (fwd_resonance_gain_ != 0.0 && res_w_ > 0.0f && res_amp_ema_ > 0.1f) {
                const float res_phase = std::atan2(res_y_, res_x_);
                float dres = (res_phase + float(gait_phase_[leg])) - L.phase;
                while (dres >  float(M_PI)) dres -= 2.0f * float(M_PI);
                while (dres < -float(M_PI)) dres += 2.0f * float(M_PI);
                y[m - 1] += (1.0f - pe) * float(fwd_resonance_gain_) * std::sin(dres);
                if (leg == 0) {   // lock quality, report-only
                    res_lock_cos_ = (1.0f - kCommitPrecAlpha) * res_lock_cos_
                                  + kCommitPrecAlpha * std::cos(dres);
                    res_lock_sin_ = (1.0f - kCommitPrecAlpha) * res_lock_sin_
                                  + kCommitPrecAlpha * std::sin(dres);
                }
            }
        }
        // Directional propulsion drive on hip1 (joint 0), phase-locked to the
        // leg's step phase.  stroke_signs sets the per-leg push direction
        // (parallel → forward, tangential → spin); steer is a left/right
        // skid-steer differential (FL,RL = left = +1; FR,RR = right = −1).
        if (!warmup && (stroke_gain_ > 0.0 || steer_ != 0.0 || heading_gain_ != 0.0 || nav_gain_ != 0.0
                        || heading_bearing_hold_gain_ != 0.0 || heading_hold_gain_ != 0.0) && m >= 1) {
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
            // Heading-HOLD-to-spawn (PD go-straight when UNguided): steer to drive the
            // dead-reckoned bearing error (P = heading_bearing_, integrated yaw rel. spawn)
            // to zero, with yaw-rate damping (D = heading_hold_gain_·yaw_rate_ema_) for a
            // clean, non-oscillating hold.  Routed through THIS authoritative skid-steer
            // channel (folds into the stroke magnitude → real L/R thrust differential that
            // TURNS the body) — not the weak additive hip1 nudge it was before.  Gated off
            // during nav (a target owns steering then).  Both gains 0 = byte-identical off.
            // ---- 2026-08-04 · THE L1 NAV SETPOINT ----------------------------------
            // WAS: `gain_P * (-heading_bearing_)` — the setpoint was implicitly 0, i.e.
            // "hold the bearing you spawned on".  The nav layer publishes an EGOCENTRIC
            // unit vector, so the angle to it IS the bearing error and simply replaces
            // that term.  The D term is untouched, so the yaw damping that produced the
            // variance collapse still applies to the NEW setpoint — the PD becomes the
            // nav layer's ACTUATOR rather than its competitor.
            // No publisher (or no token yet) => bearing_err == -heading_bearing_
            // => BYTE-IDENTICAL.
            constexpr float kInvPiL = 0.318309886f;
            const float bearing_err = gb_seen_ ? std::atan2(gb_x_, gb_y_) * kInvPiL
                                               : -heading_bearing_;
            float hold_steer = nav_on ? 0.0f
                : float(heading_bearing_hold_gain_) * bearing_err
                + float(heading_hold_gain_)         * (-yaw_rate_ema_)
                + heading_trim_;                    // the learned DC effort (I term)
            float steer_eff = float(steer_) + float(nav_gain_) * bearing - head_term + hold_steer;
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
            // lever_stroke_mult folds in progress→commit thrust (C) + forward-flow amp (D);
            // applied to the propulsion term ONLY (steer stays independent).  =1 when both off.
            // PURCHASE GATE (stroke_load_gain).  Multiplies the PROPULSION term only —
            // `side * steer_eff` carries heading_bearing_hold, a promoted controller on
            // this same skid-steer channel, and gating a heading controller by leg load
            // would corrupt it.  gate ≡ 1 when the lever is off ⇒ byte-identical.
            const float load_gate = (int(stroke_gate_.size()) == n_legs_) ? stroke_gate_[leg] : 1.0f;
            float amp  = sgn * (float(stroke_gain_) * lever_stroke_mult * fwd * load_gate
                                + side * steer_eff);
            // STROKE-TO-STEP LOCK.  The ONLY consumer of L.step_phase: coupling, the
            // amplitude homeostat and prop-credit all keep the legacy L.phase, so this is
            // one lever on one consumer (`phase_joint` moved all six at once, which is
            // part of why its collapse taught us so little).  An EXPLICIT branch, not a
            // blend — at stroke_phase_src == 0 this selects L.phase and the expression is
            // the legacy one character for character, so the gain-0 guard holds by
            // construction rather than by floating-point luck.
            const float stroke_ph = (stroke_phase_src_ > 0.0 && L.step_locked)
                                  ? L.step_phase : L.phase;
            y[0] += (1.0f - pe) * amp * std::sin(stroke_ph + float(stroke_phase_));
        }
        // --- Per-leg propulsive-credit homeostat (functional L/R propulsion balance).
        // Credit = the hip1 motion component phase-aligned with the fore-aft power
        // stroke (a static/"dragging" leg scores ~0; an in-phase stroking leg scores
        // high — the FUNCTIONAL contribution, not raw amplitude).  A below-group-mean
        // leg gets a self-limiting boost in its stroke direction so it pulls its
        // weight and L/R propulsion equalizes; the boost fades to 0 as the deficit
        // closes.  Distinct from the refuted amplitude/velocity symmetry levers.
        // 0 = off (byte-identical).
        if (!warmup && propulsion_balance_gain_ > 0.0 && m >= 1 && n >= 1) {
            float sgn2 = (int(stroke_signs_.size()) == n_legs_) ? float(stroke_signs_[leg]) : 1.0f;
            float sref = std::sin(L.phase + float(stroke_phase_));   // power-stroke waveform
            L.hip1_dc = (1.0f - kPropCreditAlpha) * L.hip1_dc + kPropCreditAlpha * L.x[0];
            float instant = (L.x[0] - L.hip1_dc) * sgn2 * sref;      // in-phase propulsive stroke
            L.prop_credit = (1.0f - kPropCreditAlpha) * L.prop_credit + kPropCreditAlpha * instant;
            float deficit = prop_credit_mean_ - L.prop_credit;       // >0 = this leg lags the group
            if (deficit > 0.0f)
                y[0] += float(propulsion_balance_gain_) * deficit * sgn2 * sref;
        }
        // (heading-hold + bearing-hold now steer through the authoritative skid-steer
        // channel above — steer_eff — instead of a separate weak hip1 nudge here.)
        // --- Coherent per-joint rhythmic drive: lock hip2/knee to the SAME leg phase as the
        // stroke (per-joint amplitude + offset) so the whole leg is ONE oscillator at ONE
        // frequency → the keyframe map can crystallize (the intra-leg-coherence fix). Default
        // gains 0 = off (legacy: only hip1 is rhythmically driven, joints drift to own freqs).
        if (!warmup && int(rhythm_gains_.size()) == m) {
            // Drive from the CLEAN global CPG phase (+ per-leg gait_phase) when available, so all
            // legs/joints lock to one frequency; else fall back to the proprio-derived L.phase.
            float base = (cpg_seen_ && !cpg_phase_topic_.empty())
                       ? (cpg_phase_ + (int(gait_phase_.size()) == n_legs_ ? float(gait_phase_[leg]) : 0.0f))
                       : L.phase;
            for (int j = 0; j < m; ++j) {
                if (rhythm_gains_[j] == 0.0) continue;
                float off = (int(rhythm_offsets_.size()) == m) ? float(rhythm_offsets_[j]) : 0.0f;
                // The fore-aft joint (0) carries the propulsion sign per leg (stroke_signs) so
                // opposite-side legs push the body the SAME way in world space → straight walking,
                // not sideways drift. Lift joints (hip2/knee) stay unsigned (symmetric).
                float lsgn = (j == 0 && int(stroke_signs_.size()) == n_legs_) ? float(stroke_signs_[leg]) : 1.0f;
                y[j] += rhythm_scale_ * (1.0f - pe) * lsgn * float(rhythm_gains_[j]) * std::sin(base + off);
            }
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
        // Intra-leg coordination instrument: do hip2 (index 1) and the knee (index m-1)
        // push the SAME way?  Read on the assembled command, pre-clamp, post-warmup.
        if (!warmup && m >= 3) {
            const float a2 = y[1], b2 = y[m - 1];
            if (std::fabs(a2) > 1e-3f && std::fabs(b2) > 1e-3f) {
                if ((a2 > 0.0f) == (b2 > 0.0f)) ++hk_agree_;
                ++hk_agree_n_;
            }
        }
        // Phase-0 saturation instrument: the command as ASSEMBLED (HK + every additive
        // term), read immediately before the one and only clamp.  clip_duty is the
        // fraction of leg-ticks the body never saw the requested command.
        // Per-leg hip1 split — see the header: the pooled figure averages the two sides
        // together and therefore cannot see a rectified differential.
        if (!warmup && int(sat_clip_leg_.size()) == n_legs_ && m >= 1) {
            const double a0 = std::fabs(double(y[0]));
            sat_pre_leg_[leg] += a0;
            if (a0 > 1.0) sat_clip_leg_[leg] += 1.0;
        }
        if (!warmup && int(sat_clip_hits_.size()) == m) {
            for (int j = 0; j < m; ++j) {
                const double a = std::fabs(double(y[j]));
                sat_pre_abs_[j] += a;
                if (a > sat_pre_max_[j]) sat_pre_max_[j] = a;
                if (a > 1.0) sat_clip_hits_[j] += 1.0;
            }
            sat_n_ += 1.0;
        }
        // Import I3 (actuator honesty).  cmd_squash=0 keeps the historical hard clamp.
        if (cmd_squash_ > 0.0)
            for (int j = 0; j < m; ++j) y[j] = std::tanh(y[j]);
        else
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

        if (wb_on) prevYw_.segment(leg * m, m) = y;   // I7: the model learns from what the
                                                     // body EXECUTED, not from the HK branch
                                                     // alone — same contract as L.prev_y.
        L.prev_x = L.x;
        L.prev_phi_ctx = Eigen::Vector2f(std::cos(cpg_phase_), std::sin(cpg_phase_));  // phase at command time
        L.prev_prev_y = L.prev_y;
        L.prev_y = y;
        L.have_prev = true;
    }

    // ---- Per-leg controller symmetry coupling (anti-asymmetry root fix) ----
    // Softly pull each leg's learned controller (C, h, Cphi) toward the average over the legs
    // in its group, so the four identical legs converge to ONE control law rather than one leg
    // specializing into a skid (the RR asymmetry).  Applied AFTER all legs updated this tick.
    // Group by SAME stroke-sign (sign-safe — no fore-aft mirror conflict).  gain 0 / no groups
    // = off = byte-identical.  A learning-time prior, not a command: it shapes what the legs
    // LEARN, HK exploration + balance still ride on top.
    if (ctrl_symmetry_gain_ > 0.0 && int(symmetry_group_of_.size()) == n_legs_) {
        const float g = float(ctrl_symmetry_gain_);
        int maxg = 0; for (int gid : symmetry_group_of_) maxg = std::max(maxg, gid);
        for (int grp = 0; grp <= maxg; ++grp) {
            // collect initialized legs in this group with matching dims
            int n = -1, cnt = 0;
            for (int leg = 0; leg < n_legs_; ++leg)
                if (symmetry_group_of_[leg] == grp && legs_[leg].initialized) {
                    if (n < 0) n = legs_[leg].n;
                    if (legs_[leg].n == n) ++cnt;
                }
            if (cnt < 2 || n < 0) continue;                 // need >=2 same-dim legs to average
            Eigen::MatrixXf Cbar = Eigen::MatrixXf::Zero(motor_dim_, n);
            Eigen::VectorXf hbar = Eigen::VectorXf::Zero(motor_dim_);
            Eigen::MatrixXf Pbar = Eigen::MatrixXf::Zero(motor_dim_, 2);
            for (int leg = 0; leg < n_legs_; ++leg)
                if (symmetry_group_of_[leg] == grp && legs_[leg].initialized && legs_[leg].n == n) {
                    Cbar += legs_[leg].C; hbar += legs_[leg].h; Pbar += legs_[leg].Cphi;
                }
            Cbar /= float(cnt); hbar /= float(cnt); Pbar /= float(cnt);
            for (int leg = 0; leg < n_legs_; ++leg)
                if (symmetry_group_of_[leg] == grp && legs_[leg].initialized && legs_[leg].n == n) {
                    legs_[leg].C.noalias()    += g * (Cbar - legs_[leg].C);
                    legs_[leg].h.noalias()    += g * (hbar - legs_[leg].h);
                    legs_[leg].Cphi.noalias() += g * (Pbar - legs_[leg].Cphi);
                }
        }
    }
}

int   MotorEPMv2::legs_initialized() const {
    int c = 0; for (auto const& L : legs_) if (L.initialized) ++c; return c;
}
float MotorEPMv2::interleg_plv() const {
    // Mean over leg PAIRS of |mean_t e^{i(phi_i - phi_j)}|.  Chance level for independent
    // legs is ~sqrt(pi)/2/sqrt(N) -> 0 as the run lengthens, so unlike gait_coherence this
    // has a null that DOES go to zero and a reading above ~0.3 is real phase locking.
    double acc = 0.0; int c = 0;
    for (int i = 0; i < n_legs_; ++i)
        for (int j = i + 1; j < n_legs_ && j < 4; ++j) {
            const int k = i * 4 + j;
            if (plv_pair_n_[k] < 200) continue;        // too little oscillation to judge
            acc += std::sqrt(plv_cos_[k] * plv_cos_[k] + plv_sin_[k] * plv_sin_[k])
                   / double(plv_pair_n_[k]);
            ++c;
        }
    return c ? float(acc / c) : 0.0f;
}

float MotorEPMv2::interleg_plv_win() const {
    // Trailing-window twin of interleg_plv().  Mean over pairs of |z_ij|, where z_ij is the
    // EMA phasor above (tau ~ 500 ticks).  Unlike the whole-run form this CAN express a
    // before/after, which is what any perturbation / (d) test needs.  Read beside
    // interleg_plv_win_support(): |z| <= support by construction, so a low reading with a
    // low support means "not moving", not "not coordinated".
    double acc = 0.0; int c = 0;
    for (int i = 0; i < n_legs_; ++i)
        for (int j = i + 1; j < n_legs_ && j < 4; ++j) {
            const int k = i * 4 + j;
            acc += std::sqrt(plv_win_cos_[k] * plv_win_cos_[k]
                           + plv_win_sin_[k] * plv_win_sin_[k]);
            ++c;
        }
    return c ? float(acc / c) : 0.0f;
}
float MotorEPMv2::interleg_plv_win_support() const {
    // Fraction of the trailing window in which a pair was genuinely oscillating, averaged
    // over pairs.  0 = nothing moved, so plv_win carries no information.
    double acc = 0.0; int c = 0;
    for (int i = 0; i < n_legs_; ++i)
        for (int j = i + 1; j < n_legs_ && j < 4; ++j) { acc += plv_win_sup_[i * 4 + j]; ++c; }
    return c ? float(acc / c) : 0.0f;
}

float MotorEPMv2::gait_coherence() const {
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
float MotorEPMv2::tle_ema_mean() const {
    // I7: when the whole-body controller owns the model, the per-leg L.tle_ema is never
    // updated and would report a flat 0.0000 — exactly the "exactly-round null" this
    // ledger warns about, and self-inflicted on 2026-08-03.  Report the whole-body TLE.
    if (whole_body_c_ > 0.0 && wb_ready_) return wb_tle_ema_;
    if (legs_.empty()) return 0.0f; float s = 0; int c = 0;
    for (auto const& L : legs_) if (L.initialized) { s += L.tle_ema; ++c; }
    return c ? s / float(c) : 0.0f;
}
float MotorEPMv2::amp_gain_mean_val() const {
    if (legs_.empty()) return 0.0f; float s = 0; int c = 0;
    for (auto const& L : legs_) if (L.initialized) { s += L.amp_gain; ++c; }
    return c ? s / float(c) : 0.0f;
}
float MotorEPMv2::amp_ema_mean() const {
    if (legs_.empty()) return 0.0f; float s = 0; int c = 0;
    for (auto const& L : legs_) if (L.initialized) { s += L.amp_ema; ++c; }
    return c ? s / float(c) : 0.0f;
}
float MotorEPMv2::gain_ema_mean() const {
    if (legs_.empty()) return 0.0f; float s = 0; int c = 0;
    for (auto const& L : legs_) if (L.initialized) { s += L.gain_ema; ++c; }
    return c ? s / float(c) : 0.0f;
}
float MotorEPMv2::outmag_ema_mean() const {
    if (legs_.empty()) return 0.0f; float s = 0; int c = 0;
    for (auto const& L : legs_) if (L.initialized) { s += L.outmag_ema; ++c; }
    return c ? s / float(c) : 0.0f;
}

// ---- snapshot / restore (determinism contract) ----
nlohmann::json MotorEPMv2::snapshot_state() const {
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
        lj["hip1_dc"]     = L.hip1_dc;
        lj["prop_credit"] = L.prop_credit;
        // Stroke-to-step lock.  Serialized because the step clock IS control state when
        // stroke_phase_src > 0 — a restored clone that dropped it would re-enter the
        // pre-lock fallback and silently walk on L.phase for a couple of strides.
        lj["step_phase"]   = L.step_phase;
        lj["step_per_ema"] = L.step_per_ema;
        lj["step_omega"]   = L.step_omega;
        lj["last_td_tick"] = L.last_td_tick;
        lj["td_count"]     = L.td_count;
        lj["td_run"]       = L.td_run;
        lj["td_cand_tick"] = L.td_cand_tick;
        lj["td_contact"]   = L.td_contact;
        lj["step_locked"]  = L.step_locked;
        lj["rest_captured"] = L.rest_captured;
        if (L.rest_captured)
            lj["rest_pos"] = std::vector<float>(L.rest_pos.data(), L.rest_pos.data() + L.rest_pos.size());
        { std::ostringstream os; os << L.babble_rng; lj["babble_rng"] = os.str(); }
        auto flat = [](Eigen::MatrixXf const& M){
            std::vector<float> v(M.data(), M.data() + M.size()); return v; };
        lj["A"] = flat(L.A); lj["b"] = flat(L.b);
        lj["C"] = flat(L.C); lj["h"] = flat(L.h);
        lj["Cphi"] = flat(L.Cphi);
        lj["Cvel"] = flat(L.Cvel);
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
    mod["height_k_eff"]   = height_k_eff_;
    mod["chassis_h_seen"] = chassis_h_seen_;
    // swing-detector observability (the gate stance_lift / Cruse ride on).  Mirrors
    // height_bias: serialized so the BODY can surface it in its stdout diag JSON.
    mod["swing_frac"]     = swing_frac_ema_;
    mod["cruse_bias"]     = cruse_bias_mean_;
    mod["phase_agree"]    = phase_agree_ema_;
    // Ratchet observability for the forgetting diagnosis (2026-07-26).  These three are
    // the state that does NOT recover after an inverted episode, so they must be readable
    // from the body's diag stream, not just the inspector:
    //   coord_best_fitness — the stored winner of the (1+1) phase search (decays 0.99 per
    //                        probe window, tau ~400 s; NOT cleared on a disruption)
    //   amp_gain_mean      — the amplitude homeostat's integrator, which winds to its rail
    //                        while the body is tucked and barely moving
    //   motor_tle          — self-model error, to see when the HK model has re-adapted
    mod["coord_best_fitness"] = coord_best_fitness_;
    mod["amp_gain_mean"]      = amp_gain_mean_val();
    mod["upright"]            = upright_;
    mod["motor_tle"]          = tle_ema_mean();
    mod["legphase_agree"] = legphase_agree_ema_;
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
    mod["heading_bearing"]   = heading_bearing_;   // integrator (yaw_rate_ema_ is transient, this is not)
    mod["fwd_progress_ema"]  = fwd_progress_ema_;   // stuck→explore stall detector + boost state
    mod["stuck_ticks"]       = stuck_ticks_;
    mod["stuck_boost"]       = stuck_boost_;
    mod["commit_ticks"]      = commit_ticks_;       // progress→commit (C) state
    mod["commit_boost"]      = commit_boost_;
    mod["flow_ema"]          = flow_ema_;           // forward-flow (D) state
    mod["flow_vol_ema"]      = flow_vol_ema_;
    // Phase-0 gait-alignment accumulators.  Serialized for two reasons: the BODY reads
    // this snapshot to put them in its stdout diag JSON (same route as h_ema/h_bias),
    // and a restored clone must report the same measurement as its original.
    mod["ga_td_cos"] = ga_td_cos_;  mod["ga_td_sin"] = ga_td_sin_;  mod["ga_td_n"] = ga_td_n_;
    mod["ga_sd_cos"] = ga_sd_cos_;  mod["ga_sd_sin"] = ga_sd_sin_;  mod["ga_sd_n"] = ga_sd_n_;
    mod["ga_align_acc"]   = ga_align_acc_;   mod["ga_align_n"]   = ga_align_n_;
    mod["ga_stance_pos"]  = ga_stance_pos_;  mod["ga_stance_n"]  = ga_stance_n_;
    mod["ga_swing_pos"]   = ga_swing_pos_;   mod["ga_swing_n"]   = ga_swing_n_;
    mod["ga_contact_acc"] = ga_contact_acc_; mod["ga_contact_n"] = ga_contact_n_;
    mod["ga_tq_stance"]   = ga_tq_stance_;   mod["ga_tq_stance_n"] = ga_tq_stance_n_;
    mod["ga_tq_swing"]    = ga_tq_swing_;    mod["ga_tq_swing_n"]  = ga_tq_swing_n_;
    mod["ga_tq_agree"]    = ga_tq_agree_;    mod["ga_tq_agree_n"]  = ga_tq_agree_n_;
    mod["ga_tq_ema"]      = ga_tq_ema_;
    mod["ga_tq_h1_agree"] = ga_tq_h1_agree_; mod["ga_tq_h1_agree_n"] = ga_tq_h1_agree_n_;
    mod["ga_tq_h1_ema"]   = ga_tq_h1_ema_;
    mod["ga_mv_stance"]   = ga_mv_stance_;   mod["ga_mv_stance_n"] = ga_mv_stance_n_;
    mod["ga_mv_swing"]    = ga_mv_swing_;    mod["ga_mv_swing_n"]  = ga_mv_swing_n_;
    mod["step_td_err_acc"] = step_td_err_acc_; mod["step_td_err_n"] = step_td_err_n_;
    mod["step_lock_flips_state"] = step_lock_flips_;
    mod["ga_tq_j_stance"] = ga_tq_j_stance_;  mod["ga_tq_j_stance_n"] = ga_tq_j_stance_n_;
    mod["ga_tq_j_swing"]  = ga_tq_j_swing_;   mod["ga_tq_j_swing_n"]  = ga_tq_j_swing_n_;
    {   // derived per-joint separation, for the body's stdout diag
        std::vector<double> sep(ga_tq_j_stance_.size(), 0.0);
        for (size_t k = 0; k < sep.size(); ++k) {
            const double a = ga_tq_j_stance_n_ ? ga_tq_j_stance_[k] / double(ga_tq_j_stance_n_) : 0.0;
            const double b = ga_tq_j_swing_n_  ? ga_tq_j_swing_[k]  / double(ga_tq_j_swing_n_)  : 0.0;
            sep[k] = a / (b + 1e-9);
        }
        mod["torque_sep_joint"] = sep;
    }
    mod["ga_hip1_ema"]    = ga_hip1_ema_;
    mod["ga_hip1_per"]    = ga_hip1_per_;    mod["ga_knee_per"] = ga_knee_per_;
    mod["ga_foot_per"]    = ga_foot_per_;     mod["ga_con_per"] = ga_con_per_;
    mod["ga_hip1_last"]   = ga_hip1_last_;   mod["ga_knee_last"] = ga_knee_last_;
    mod["ga_foot_last"]   = ga_foot_last_;    mod["ga_con_last"] = ga_con_last_;
    mod["ga_hip1_above"]  = ga_hip1_above_;  mod["ga_knee_above"] = ga_knee_above_;
    mod["ga_prev_contact"] = ga_prev_contact_; mod["ga_prev_swing"] = ga_prev_swing_;
    // Derived readouts the body surfaces directly (so the collector needs no maths).
    mod["stroke_td_plv"] = ga_td_n_ ? std::sqrt(ga_td_cos_ * ga_td_cos_ + ga_td_sin_ * ga_td_sin_)
                                      / double(ga_td_n_) : 0.0;
    mod["stroke_sd_plv"] = ga_sd_n_ ? std::sqrt(ga_sd_cos_ * ga_sd_cos_ + ga_sd_sin_ * ga_sd_sin_)
                                      / double(ga_sd_n_) : 0.0;
    mod["stroke_pos_stance"] = ga_stance_n_ ? ga_stance_pos_ / double(ga_stance_n_) : 0.0;
    mod["stroke_pos_swing"]  = ga_swing_n_  ? ga_swing_pos_  / double(ga_swing_n_)  : 0.0;
    mod["contact_duty"]      = ga_contact_n_ ? ga_contact_acc_ / double(ga_contact_n_) : 0.0;
    mod["torque_agree"]      = ga_tq_agree_n_ ? ga_tq_agree_ / double(ga_tq_agree_n_) : 0.0;
    // NOTE FOR ANYONE ADDING AN INSTRUMENT: the body's stdout diag reads
    // `brain.get_module_metrics()`, which is fed from THIS `mod` dict — NOT from
    // diag_snapshot().  A field added only to diag_snapshot() reaches the live inspector
    // but reads as 0.0 in every headless run and every scripts_tools/ harness, which looks
    // exactly like "the mechanism never fired".  Add new instruments in BOTH places.
    mod["torque_agree_hip1"] = ga_tq_h1_agree_n_ ? ga_tq_h1_agree_ / double(ga_tq_h1_agree_n_) : 0.0;
    mod["mv_stance"]         = ga_mv_stance_n_ ? ga_mv_stance_ / double(ga_mv_stance_n_) : 0.0;
    mod["mv_swing"]          = ga_mv_swing_n_  ? ga_mv_swing_  / double(ga_mv_swing_n_)  : 0.0;
    mod["step_lock"]         = step_lock_frac_;
    mod["step_period"]       = step_period_mean_;
    mod["step_td_err"]       = step_td_err_n_ ? step_td_err_acc_ / double(step_td_err_n_) : 0.0;
    // How often a leg's stroke phase reference SWAPS between the step clock and the
    // L.phase fallback.  A lever that locks intermittently is worse than one that never
    // locks: every swap is a discontinuity in the driven command, which is the failure the
    // soft PLL exists to prevent — so this number decides whether a collapse is "the clock
    // is wrong" or "the clock keeps being taken away".
    mod["step_lock_flips"]   = step_lock_flips_;
    {   // footfall regularity (see diag_snapshot) — the harness reads THIS dict, not diag.
        double pooled = 0.0; int pn = 0;
        for (int i = 0; i < n_legs_ && i < int(ga_con_iv_n_.size()); ++i) {
            if (ga_con_iv_n_[i] < 3) continue;
            const double n = double(ga_con_iv_n_[i]);
            const double mu = ga_con_iv_sum_[i] / n;
            const double var = std::max(0.0, ga_con_iv_sq_[i] / n - mu * mu);
            if (mu > 1e-9) { pooled += std::sqrt(var) / mu; ++pn; }
        }
        mod["step_cv"] = pn ? pooled / double(pn) : 0.0;
    {   // real-step (micro-lift filtered) interval CV — see kRealSwingTicks
        double pooled = 0.0; int pn = 0; double permean = 0.0;
        for (int i = 0; i < n_legs_ && i < int(ga_rs_iv_n_.size()); ++i) {
            if (ga_rs_iv_n_[i] < 3) continue;
            const double n = double(ga_rs_iv_n_[i]);
            const double mu = ga_rs_iv_sum_[i] / n;
            const double var = std::max(0.0, ga_rs_iv_sq_[i] / n - mu * mu);
            if (mu > 1e-9) { pooled += std::sqrt(var) / mu; permean += mu; ++pn; }
        }
        mod["step_cv_real"]  = pn ? pooled / double(pn) : 0.0;
        mod["step_per_real"] = pn ? permean / double(pn) : 0.0;
    }
        mod["stance_bout"] = ga_st_bout_n_ ? ga_st_bout_sum_ / double(ga_st_bout_n_) : 0.0;
        mod["swing_bout"]  = ga_sw_bout_n_ ? ga_sw_bout_sum_ / double(ga_sw_bout_n_) : 0.0;
        mod["short_bout_frac"] = (ga_st_bout_n_ + ga_sw_bout_n_)
            ? double(ga_st_bout_short_ + ga_sw_bout_short_)
              / double(ga_st_bout_n_ + ga_sw_bout_n_) : 0.0;
    }
    mod["torque_stance"]     = ga_tq_stance_n_ ? ga_tq_stance_ / double(ga_tq_stance_n_) : 0.0;
    mod["torque_swing"]      = ga_tq_swing_n_  ? ga_tq_swing_  / double(ga_tq_swing_n_)  : 0.0;
    mod["explore_mult"]      = explore_mult_diag_;
    mod["support_bin"]       = support_bin_diag_;
    mod["support_value"]     = support_value_diag_;
    mod["support_mult"]      = support_mult_diag_;
    mod["support_resp"]      = support_resp_diag_;
    // Consumer-fired check for stance_release_frac: fraction of stance-lift ticks where
    // the release was live.  0.0 with the lever on = the reversal detector never fired.
    mod["sr_duty"] = sr_stance_ticks_ ? double(sr_release_ticks_) / double(sr_stance_ticks_) : 0.0;
    mod["ga_yaw_leg"] = ga_yaw_leg_;  mod["ga_yaw_leg_n"] = ga_yaw_leg_n_;
    mod["ga_yaw_allplant"] = ga_yaw_allplant_; mod["ga_yaw_allplant_n"] = ga_yaw_allplant_n_;
    mod["ga_yaw_anyswing"] = ga_yaw_anyswing_; mod["ga_yaw_anyswing_n"] = ga_yaw_anyswing_n_;
    // ★ INTER-LEG COORDINATION — mirrored from diag_snapshot() 2026-08-03.  It lived in
    // diag ONLY, so it has NEVER been visible to a seedavg arm: the whole campaign has
    // measured distance/steps/balance and never once measured whether the legs are
    // PHASE-LOCKED TO EACH OTHER, which is the operator's actual complaint ("each leg has
    // its own directive").  |mean_j e^{i(phi_j - P_j)}| in [0,1]; 1 = locked to the gait.
    mod["gait_coherence"] = gait_coherence();   // ⚠ INSTANTANEOUS — see interleg_plv
    mod["interleg_plv"]   = interleg_plv();     // the honest coordination read
    mod["plv_support"]    = double(plv_n_);     // ticks with genuine oscillation behind it
    // Trailing-window twin (2026-08-04).  The whole-run form above cannot express a
    // before/after, so it cannot score a perturbation; this one can.  ALWAYS read beside
    // plv_win_support.
    mod["interleg_plv_win"]  = interleg_plv_win();
    mod["plv_win_support"]   = interleg_plv_win_support();
    // PER-PAIR windowed PLV, order (0,1)(0,2)(0,3)(1,2)(1,3)(2,3).  The pooled mean above
    // cannot answer the question a LESION raises: when one leg is dead its three pairs are
    // dragged toward 0 and swamp whatever the SURVIVORS are doing, so a three-legged gait
    // that is perfectly coordinated still reads ~half the pooled value.  Emitted per pair so
    // any subset can be scored — specifically "the surviving legs, among themselves".
    {
        std::vector<double> pw, ps;
        for (int i = 0; i < n_legs_; ++i)
            for (int j = i + 1; j < n_legs_ && j < 4; ++j) {
                const int k = i * 4 + j;
                pw.push_back(std::sqrt(plv_win_cos_[k] * plv_win_cos_[k]
                                     + plv_win_sin_[k] * plv_win_sin_[k]));
                ps.push_back(plv_win_sup_[k]);
            }
        mod["plv_win_pairs"] = pw;
        mod["plv_win_pair_sup"] = ps;
    }
    // ---- 2026-08-04 · PER-LEG PREDICTION ERROR (the inferential-gain direction).
    // tle_ema_mean() has always been reported as ONE body-level number, so the question the
    // precision-weighting direction turns on — do the four legs differ in how well they
    // predict themselves? — has never been answerable.  Emitted per leg, uncollapsed, so
    // Python can window it around a perturbation.  amp_leg is its mandatory companion: a
    // frozen leg is trivially predictable (tle -> 0) and would otherwise read as the most
    // trustworthy leg in the body.  ⚠ An exactly-0.0000 entry means a limb that never moved
    // or was never initialised, NOT a perfect model.
    {
        std::vector<double> tl, al;
        tl.reserve(legs_.size()); al.reserve(legs_.size());
        for (auto const& L : legs_) {
            tl.push_back(L.initialized ? double(L.tle_ema) : -1.0);   // −1 = not initialised
            al.push_back(L.initialized ? double(L.amp_ema) : -1.0);
        }
        mod["tle_leg"] = tl;
        mod["amp_leg"] = al;
    }
    // Panic override.  `(1 - pe)` multiplies the coupling, stroke and rhythm terms, so the
    // deployed stack ALREADY carries a crude surprise-modulated gain — driven by a
    // hand-specified distress scalar rather than by the agent's own prediction error.  It
    // was never instrumented, so it is an unmeasured confound in every gain A/B.
    // ⚠ Instantaneous: read its DUTY over the run, not one sample (the gait_coherence trap).
    mod["panic_eff"] = double(panic_) * panic_strength_;
    // CONSUMER CHECK for couple_prec_gain (CLAUDE.md §3.2 rule 5, and the
    // postural_gain_joints shape: a knob that silently does nothing).  The weights are
    // mean-normalised, so couple_w_mean MUST read 1.00 and couple_w_spr = (max−min)/mean is
    // the read that matters: it must scale monotonically with |k|.  A FLAT spread across a
    // gain sweep means the lever never fired ⇒ a measurement outcome, not a verdict.
    // CONSUMER CHECK for the nav setpoint: goal_bearing_msgs == 0 with a topic configured
    // means the publisher never fired and the PD is silently still holding the spawn bearing.
    mod["fwd_progress_ema"] = double(fwd_progress_ema_);
    mod["couple_R"]    = double(couple_R_diag_);
    mod["phase_retro"] = double(phase_retro_diag_);
    mod["phase_freq"]  = double(phase_freq_diag_);
    mod["res_period"]  = res_w_ > 1e-4f ? (2.0 * M_PI / double(res_w_)) : -1.0;
    mod["res_amp"]     = double(res_amp_ema_);
    mod["res_lock"]    = double(std::sqrt(res_lock_cos_ * res_lock_cos_ + res_lock_sin_ * res_lock_sin_));
    mod["intent_err"] = intent_seen_ ? double(intent_err_norm()) : -1.0;
    mod["commit_prec"] = double(commit_prec_diag_);   // consumer check: 1.0 = lever off
    mod["intent_msgs"] = double(intent_msgs_);   // 0 with intent_topic set = publisher never fired
    mod["commit_boost"] = double(commit_boost_);
    mod["heading_trim"] = double(heading_trim_);   // consumer check for the I term
    mod["goal_bearing_msgs"] = double(gb_msgs_);
    mod["goal_bearing_err"]  = gb_seen_ ? double(std::atan2(gb_x_, gb_y_) * 0.318309886f) : 0.0;
    mod["couple_w_spr"]  = cw_n_ ? cw_spr_acc_ / double(cw_n_) : 0.0;
    mod["couple_w_mean"] = cw_n_ ? 1.0 : 0.0;   // 0 = never evaluated (k=0, or coupling off)
    // ★ INTRA-LEG COORDINATION (2026-08-03).  The operator observed, for the first time,
    // hip2 and knee working TOGETHER to lift the chassis — where the hand-built reflexes
    // only ever drove one or the other.  Those two joints share foot-height authority
    // (measured corr(foot_y, joint) ~ 0.27 each; the Cruse v2 rule had to be TOLD to drive
    // both because either alone is too weak).  This is the sign-agreement of their COMMANDS,
    // pooled over legs: 1.0 = always pushing the same way, 0.5 = chance, 0 = always opposed.
    mod["hip2_knee_agree"] = hk_agree_n_ ? double(hk_agree_) / double(hk_agree_n_) : 0.0;
    // L-1b OBJECTIVE SOCKET — mirrored from diag_snapshot() 2026-08-02.  It existed in
    // diag ONLY, which is the documented trap: the body's stdout reads
    // get_module_metrics() fed from THIS dict, so the socket read 0.0 in every headless
    // run and "the objective is driving" was never verifiable from a seedavg arm.
    // This is the §3.2 rule-5 consumer check for any objective-vs-additive experiment.
    {
        bool on = false; float wsum = 0.0f; int oc = 0;
        for (int i = 0; i < n_legs_ && i < int(obj_seen_.size()); ++i)
            if (obj_seen_[i] && obj_weight_[i] > 0.0f) { on = true; wsum += obj_weight_[i]; ++oc; }
        mod["obj_active"] = on;
        mod["obj_weight"] = oc ? wsum / float(oc) : 0.0f;
        mod["obj_legs"]   = oc;
    }
    mod["swing_tuck_frac"] = swing_tuck_frac_;
    mod["tibia_off_mean"]  = ga_tib_n_ ? (ga_tib_acc_ / double(ga_tib_n_)) : 0.0;
    // ---- 2026-08-02 Phase-0 instruments (report-only; see
    //      docs/reports/playful_machine_source_analysis.md §4) --------------------
    // (F1) SATURATION.  How much of the assembled command the body never sees
    // (`clip_duty`, per motor index, pooled over legs), and how much of what is sent
    // originates in the HK branch rather than the additive scaffolds (`hk_share`).
    {
        const int mdim = motor_dim_;
        std::vector<double> duty(mdim, 0.0), pmag(mdim, 0.0), pmax(mdim, 0.0), hkm(mdim, 0.0);
        double duty_mean = 0.0, hk_sum = 0.0, pre_sum = 0.0;
        if (sat_n_ > 0.0 && int(sat_clip_hits_.size()) == mdim) {
            for (int j = 0; j < mdim; ++j) {
                duty[j] = sat_clip_hits_[j] / sat_n_;
                pmag[j] = sat_pre_abs_[j]   / sat_n_;
                pmax[j] = sat_pre_max_[j];
                hkm[j]  = sat_hk_abs_[j]    / sat_n_;
                duty_mean += duty[j];
                hk_sum    += sat_hk_abs_[j];
                pre_sum   += sat_pre_abs_[j];
            }
            duty_mean /= double(mdim);
        }
        mod["clip_duty_j"] = duty;  mod["clip_duty"] = duty_mean;
        {   // per-leg hip1: LEFT legs are 0,2 and RIGHT are 1,3 (the `side` pattern), so a
            // rectified turn shows as one side's clip duty pinned while the other's falls.
            // ⚠ sat_n_ counts LEG-ticks (it is incremented inside the per-leg loop), while
            // these accumulate once per leg — so the per-leg divisor is sat_n_/n_legs_.
            // Using sat_n_ directly made every figure exactly 4x too small; caught by
            // cross-checking the mean against the pooled clip_h1/pre_h1.
            std::vector<double> cl(n_legs_, 0.0), pl(n_legs_, 0.0);
            const double per_leg_n = (n_legs_ > 0) ? sat_n_ / double(n_legs_) : 0.0;
            for (int i = 0; i < n_legs_ && per_leg_n > 0.0; ++i) {
                cl[i] = sat_clip_leg_[i] / per_leg_n;
                pl[i] = sat_pre_leg_[i]  / per_leg_n;
            }
            mod["clip_h1_leg"] = cl;
            mod["pre_h1_leg"]  = pl;
        }
        mod["pre_mag_j"]   = pmag;  mod["pre_max_j"] = pmax;
        mod["hk_mag_j"]    = hkm;   mod["hk_share"]  = pre_sum > 1e-9 ? hk_sum / pre_sum : 0.0;
    }
    // (F2) ECHO CHANNEL.  The per-joint state is [pos, last_action, delta], so state
    // index 3j+1 IS the command just issued — a channel the self-model can predict
    // exactly and the controller can drive perfectly, i.e. one on which the
    // homeokinetic objective can be satisfied without moving the body.  `echo_a_gain`
    // is the mean self-model gain on that channel (→1 means latched); `c_mass_*` is
    // where the controller's weight actually sits.
    {
        double a_echo = 0.0; int a_n = 0;
        double cm_pos = 0.0, cm_act = 0.0, cm_del = 0.0;
        for (int i = 0; i < n_legs_ && i < int(legs_.size()); ++i) {
            Leg const& LL = legs_[i];
            if (!LL.initialized || LL.n < 3 * motor_dim_) continue;
            for (int j = 0; j < motor_dim_; ++j) {
                a_echo += double(LL.A(3 * j + 1, j)); ++a_n;
                for (int r = 0; r < motor_dim_; ++r) {
                    cm_pos += std::fabs(double(LL.C(r, 3 * j    )));
                    cm_act += std::fabs(double(LL.C(r, 3 * j + 1)));
                    cm_del += std::fabs(double(LL.C(r, 3 * j + 2)));
                }
            }
        }
        const double cm = cm_pos + cm_act + cm_del;
        mod["echo_a_gain"] = a_n ? a_echo / double(a_n) : 0.0;
        mod["c_mass_pos"]  = cm > 1e-12 ? cm_pos / cm : 0.0;
        mod["c_mass_act"]  = cm > 1e-12 ? cm_act / cm : 0.0;
        mod["c_mass_del"]  = cm > 1e-12 ? cm_del / cm : 0.0;
    }
    mod["ga_tib_acc"] = ga_tib_acc_;  mod["ga_tib_n"] = ga_tib_n_;
    {
        const double ap = ga_yaw_allplant_n_ ? ga_yaw_allplant_ / double(ga_yaw_allplant_n_) : 0.0;
        const double sw = ga_yaw_anyswing_n_ ? ga_yaw_anyswing_ / double(ga_yaw_anyswing_n_) : 0.0;
        mod["yaw_allplant"] = ap; mod["yaw_anyswing"] = sw; mod["yaw_swing_excess"] = sw - ap;
        std::vector<double> per(ga_yaw_leg_.size(), 0.0);
        for (size_t k = 0; k < per.size(); ++k)
            per[k] = ga_yaw_leg_n_[k] ? ga_yaw_leg_[k] / double(ga_yaw_leg_n_[k]) : 0.0;
        mod["yaw_per_leg"] = per;
        const double apd = ga_yawd_allplant_n_ ? ga_yawd_allplant_ / double(ga_yawd_allplant_n_) : 0.0;
        const double swd = ga_yawd_anyswing_n_ ? ga_yawd_anyswing_ / double(ga_yawd_anyswing_n_) : 0.0;
        mod["yawd_allplant"] = apd; mod["yawd_anyswing"] = swd;
        mod["yawd_swing_excess"] = swd - apd;
        std::vector<double> perd(ga_yawd_leg_.size(), 0.0);
        for (size_t k = 0; k < perd.size(); ++k)
            perd[k] = ga_yawd_leg_n_[k] ? ga_yawd_leg_[k] / double(ga_yawd_leg_n_[k]) : 0.0;
        mod["yawd_per_leg"] = perd;
    }
    mod["ga_yawd_leg"] = ga_yawd_leg_;  mod["ga_yawd_leg_n"] = ga_yawd_leg_n_;
    mod["ga_yawd_allplant"] = ga_yawd_allplant_; mod["ga_yawd_allplant_n"] = ga_yawd_allplant_n_;
    mod["ga_yawd_anyswing"] = ga_yawd_anyswing_; mod["ga_yawd_anyswing_n"] = ga_yawd_anyswing_n_;
    mod["stroke_gate_mean"]   = stroke_gate_mean_;
    mod["stroke_gate_spread"] = stroke_gate_spread_;
    mod["stroke_load_ema"]    = stroke_load_ema_;
    return nlohmann::json{{"version", 2}, {"legs", legs}, {"module", mod}};
}

// Live viz (xaq_inspector MotorEPM widget): the homeokinetic self-model's health
// (motor-TLE = forward-model surprise, the one working predictive loop), the
// cognitive drive channels the brain injects (cog_steer differential, cog_thrust
// common-mode), the body's resulting forward velocity, the curiosity/hunger
// neuromodulators, and the leg-0 forward self-model A (motor→sensor) so the widget
// can draw a heatmap of what the body has learned to predict about its own motion.
nlohmann::json MotorEPMv2::diag_snapshot() const {
    nlohmann::json j;
    j["n_legs"]      = n_legs_;
    j["motor_dim"]   = motor_dim_;
    j["motor_tle"]   = tle_ema_mean();          // mean forward-model prediction error (self-model health)
    j["loop_gain"]   = gain_ema_mean();
    // Gate 0 (L-1a) reset-masked gait instruments:
    // Phase-0 instruments — mirrored from snapshot_state()'s `mod` dict so the live
    // inspector and the headless harness read the SAME numbers (§4 of the PM analysis).
    {
        const int mdim = motor_dim_;
        double duty_mean = 0.0, hk_sum = 0.0, pre_sum = 0.0;
        std::vector<double> duty(mdim, 0.0);
        if (sat_n_ > 0.0 && int(sat_clip_hits_.size()) == mdim) {
            for (int k = 0; k < mdim; ++k) {
                duty[k]    = sat_clip_hits_[k] / sat_n_;
                duty_mean += duty[k];
                hk_sum    += sat_hk_abs_[k];
                pre_sum   += sat_pre_abs_[k];
            }
            duty_mean /= double(mdim);
        }
        j["clip_duty_j"] = duty;
        j["clip_duty"]   = duty_mean;
        j["hk_share"]    = pre_sum > 1e-9 ? hk_sum / pre_sum : 0.0;
    }
    j["gait_coherence"]    = gait_coherence();  // Kuramoto phase-lock ∈[0,1]; rises as legs lock to the gait
    j["coupling_eff"]      = coupling_eff_;      // Gate 2: live (faded) coupling strength
    j["rhythm_scale"]      = rhythm_scale_;      // Gate 2: live (faded) rhythm-scaffold scale
    j["reset_count"]       = reset_count_;       // cumulative fall/teleport disruptions
    j["ticks_since_reset"] = ticks_since_reset_; // for reset-masking the coherence/TLE trend
    j["reset_rate"]        = reset_rate_ema_;    // Gate 0 signal: FALLS as the upright prior holds
    // L-1b objective socket (§1.1) — is an external posture objective driving the controller?
    {
        bool on = false; float wsum = 0.0f; int oc = 0;
        for (int i = 0; i < n_legs_ && i < int(obj_seen_.size()); ++i)
            if (obj_seen_[i] && obj_weight_[i] > 0.0f) { on = true; wsum += obj_weight_[i]; ++oc; }
        j["obj_active"] = on;
        j["obj_weight"] = oc ? wsum / float(oc) : 0.0f;
    }
    // L-1b velocity objective — is the phase-indexed velocity target (propulsive push) driving?
    {
        bool on = false; float wsum = 0.0f; int oc = 0;
        for (int i = 0; i < n_legs_ && i < int(obj_vel_seen_.size()); ++i)
            if (obj_vel_seen_[i] && obj_vel_weight_[i] > 0.0f) { on = true; wsum += obj_vel_weight_[i]; ++oc; }
        j["obj_vel_active"] = on;
        j["obj_vel_weight"] = oc ? wsum / float(oc) : 0.0f;
    }
    // Propulsive-credit homeostat: per-leg functional forward contribution + the
    // group mean, so the L/R propulsion imbalance (the drag → spin) is observable.
    j["prop_balance_active"] = (propulsion_balance_gain_ > 0.0);
    j["prop_credit_mean"]    = prop_credit_mean_;
    j["heading_hold_active"] = (heading_hold_gain_ != 0.0);
    j["yaw_rate_ema"]        = yaw_rate_ema_;   // the heading-hold's error signal
    j["bearing_hold_active"] = (heading_bearing_hold_gain_ != 0.0);
    j["heading_bearing"]     = heading_bearing_; // the bearing-hold's error signal (rel. spawn, π-units)
    j["stuck_explore_active"] = (stuck_explore_gain_ != 0.0);
    j["fwd_progress_ema"]    = fwd_progress_ema_; // stall detector (below kStuckVelThresh ≈ stuck)
    j["stuck_boost"]         = stuck_boost_;      // current exploration amplification (0..1)
    j["commit_active"]       = (progress_commit_gain_ != 0.0);  // lever C
    j["commit_boost"]        = commit_boost_;     // 0..1 (ramps when flowing → damps explore + adds thrust)
    j["flow_active"]         = (forward_flow_gain_ != 0.0);     // lever D
    j["stance_lift_active"]  = (stance_lift_gain_ != 0.0);      // knee stance-lift (belly-up)
    // Fraction of legs the swing detector calls "swinging".  ~0.5 with no deadband
    // means it is reporting phase, not contact — compare against the body's own
    // absolute planted test before trusting any lever gated on it.
    j["swing_frac"]          = swing_frac_ema_;
    // Mean |MotorEPM's own Cruse contribution|.  EXACTLY 0 ⇒ this module's Cruse block
    // never executed, so any Cruse-looking motion is coming from somewhere else
    // (CruseCoordinator's independent rule3_weight/cruse_bias_gain, or the emergent
    // gait).  Resolves the two-Rule-3 ambiguity by measurement, not inference.
    j["cruse_bias"]          = cruse_bias_mean_;
    // Agreement between a LEGAL body-rhythm phase gate and the god's-eye swing detector.
    // ~0.5 = the phase says nothing about it; ~1.0 = the detector IS a phase gate and can
    // be replaced by one. Decides the oracle refactor BEFORE the replacement is built.
    j["phase_agree"]         = phase_agree_ema_;      // global body phase vs the oracle
    j["legphase_agree"]      = legphase_agree_ema_;   // per-leg joint phase vs the oracle
    // ---- Phase-0 gait-alignment diagnostic (all 0/NaN-free when gait_align_diag = 0).
    // stroke_td_plv is the headline: the phase-locking value of the stroke waveform at
    // TRUE touchdown.  ~0 ⇒ the foot lands at a uniformly random point in the power
    // stroke, i.e. thrust and support are separate, unlocked clocks.
    {
        auto plv = [](double c, double s, int64_t n){
            return n > 0 ? std::sqrt(c * c + s * s) / double(n) : 0.0; };
        auto ang = [](double c, double s, int64_t n){
            return n > 0 ? std::atan2(s, c) : 0.0; };
        j["gait_align_active"]  = (gait_align_diag_ > 0.0);
        j["stroke_td_plv"]      = plv(ga_td_cos_, ga_td_sin_, ga_td_n_);
        j["stroke_td_phase"]    = ang(ga_td_cos_, ga_td_sin_, ga_td_n_);
        j["stroke_td_n"]        = ga_td_n_;
        j["stroke_sd_plv"]      = plv(ga_sd_cos_, ga_sd_sin_, ga_sd_n_);   // vs the incumbent detector
        j["stroke_sd_phase"]    = ang(ga_sd_cos_, ga_sd_sin_, ga_sd_n_);
        j["stroke_sd_n"]        = ga_sd_n_;
        // Signed continuous alignment, normalized so ±1 = a perfectly locked stroke
        // (E[|sin|] = 2/π for a uniform phase, which is the scale a locked signal reaches).
        constexpr double kTwoOverPi = 0.6366197723675814;
        j["stroke_align"]       = ga_align_n_ ? (ga_align_acc_ / double(ga_align_n_)) / kTwoOverPi : 0.0;
        // The most readable form: what fraction of STANCE time is spent in the positive
        // half of the stroke waveform, vs the same over SWING.  Both ≈0.5 ⇒ no relation.
        j["stroke_pos_stance"]  = ga_stance_n_ ? ga_stance_pos_ / double(ga_stance_n_) : 0.0;
        j["stroke_pos_swing"]   = ga_swing_n_  ? ga_swing_pos_  / double(ga_swing_n_)  : 0.0;
        j["contact_duty"]       = ga_contact_n_ ? ga_contact_acc_ / double(ga_contact_n_) : 0.0;
        // Load separation.  torque_sep is threshold-free (a ratio of means); torque_agree
        // is the same self-referential test the foot-height detector uses, so it can be
        // read directly against legphase_agree.  Both at chance ⇒ no load lever is possible.
        j["torque_active"]      = have_torque_;
        j["torque_stance"]      = ga_tq_stance_n_ ? ga_tq_stance_ / double(ga_tq_stance_n_) : 0.0;
        j["torque_swing"]       = ga_tq_swing_n_  ? ga_tq_swing_  / double(ga_tq_swing_n_)  : 0.0;
        j["torque_sep"]         = (ga_tq_swing_n_ && ga_tq_stance_n_)
                                ? (ga_tq_stance_ / double(ga_tq_stance_n_))
                                  / ((ga_tq_swing_ / double(ga_tq_swing_n_)) + 1e-6) : 0.0;
        j["torque_agree"]       = ga_tq_agree_n_ ? ga_tq_agree_ / double(ga_tq_agree_n_) : 0.0;
        // The same test on hip1 ALONE — the signal a load-derived step clock
        // (stroke_phase_src=2) would actually threshold.  Scopes that source before it is
        // built: at chance, source 2 cannot work no matter how the rest is tuned.
        j["torque_agree_hip1"]  = ga_tq_h1_agree_n_ ? ga_tq_h1_agree_ / double(ga_tq_h1_agree_n_) : 0.0;
        // THE NON-TAUTOLOGICAL MECHANISM INSTRUMENT for the stroke-to-step lock: the same
        // stance/swing split on the ACHIEVED fore-aft motion rather than the commanded
        // waveform.  A working lock means mv_stance is negative (the planted foot travels
        // backward relative to the body = it is pushing) while mv_swing is positive (the
        // airborne foot is being carried forward), and their SEPARATION is the number to
        // read.  Both ≈ 0, or equal, means the leg is scrubbing whatever the command says.
        j["mv_stance"]          = ga_mv_stance_n_ ? ga_mv_stance_ / double(ga_mv_stance_n_) : 0.0;
        j["mv_swing"]           = ga_mv_swing_n_  ? ga_mv_swing_  / double(ga_mv_swing_n_)  : 0.0;
        // Per-joint separation: which servo actually reports being loaded?  hip1 does
        // fore-aft work in both phases, so a whole-leg sum can dilute a clean hip2/knee
        // signal.  This picks the input for a load-gated stroke by measurement.
        {
            std::vector<double> sep(ga_tq_j_stance_.size(), 0.0);
            for (size_t k = 0; k < sep.size(); ++k) {
                const double a = ga_tq_j_stance_n_ ? ga_tq_j_stance_[k] / double(ga_tq_j_stance_n_) : 0.0;
                const double b = ga_tq_j_swing_n_  ? ga_tq_j_swing_[k]  / double(ga_tq_j_swing_n_)  : 0.0;
                sep[k] = a / (b + 1e-9);
            }
            j["torque_sep_joint"] = sep;    // [hip1, hip2, knee]; 1.0 = that servo tells you nothing
        }
        // Cycle periods.  hip1 is the stride; knee is what the stroke's phase is read from;
        // foot is what the stance gate rides.  Three different numbers ⇒ three clocks.
        auto mean_of = [](std::vector<float> const& v){
            double s = 0.0; int n = 0;
            for (float x : v) if (x > 0.0f) { s += x; ++n; }
            return n ? s / double(n) : 0.0; };
        j["period_hip1"]        = mean_of(ga_hip1_per_);
        j["period_knee"]        = mean_of(ga_knee_per_);
        j["period_foot"]        = mean_of(ga_foot_per_);   // the INCUMBENT detector's cycle
        j["period_contact"]     = mean_of(ga_con_per_);    // the REAL step period (touch flag)
        j["period_hip1_legs"]   = ga_hip1_per_;
        j["period_knee_legs"]   = ga_knee_per_;
        j["period_foot_legs"]   = ga_foot_per_;
        j["period_contact_legs"] = ga_con_per_;
        // Footfall REGULARITY: cycle-to-cycle coefficient of variation of the true
        // inter-touchdown interval, per leg and pooled.  This is the prerequisite for ANY
        // touchdown-referenced phase: a PLL cannot lock to a rhythm whose period wanders.
        {
            std::vector<double> cv(n_legs_, 0.0);
            double pooled_cv = 0.0; int pooled_n = 0;
            for (int i = 0; i < n_legs_ && i < int(ga_con_iv_n_.size()); ++i) {
                if (ga_con_iv_n_[i] < 3) continue;
                const double n  = double(ga_con_iv_n_[i]);
                const double mu = ga_con_iv_sum_[i] / n;
                const double var = std::max(0.0, ga_con_iv_sq_[i] / n - mu * mu);
                cv[i] = (mu > 1e-9) ? std::sqrt(var) / mu : 0.0;
                pooled_cv += cv[i]; ++pooled_n;
            }
            j["stance_bout"]   = ga_st_bout_n_ ? ga_st_bout_sum_ / double(ga_st_bout_n_) : 0.0;
            j["swing_bout"]    = ga_sw_bout_n_ ? ga_sw_bout_sum_ / double(ga_sw_bout_n_) : 0.0;
            j["short_bout_frac"] = (ga_st_bout_n_ + ga_sw_bout_n_)
                ? double(ga_st_bout_short_ + ga_sw_bout_short_)
                  / double(ga_st_bout_n_ + ga_sw_bout_n_) : 0.0;
            j["step_cv_legs"] = cv;
            j["step_cv"]      = pooled_n ? pooled_cv / double(pooled_n) : 0.0;
        }
    }
    // STROKE-TO-STEP LOCK observability (CLAUDE.md §3.2 rule 5).  `step_lock` is the
    // consumer check: 0 with stroke_phase_src > 0 means the clock never locked, which on
    // this stack means contact_topic (or torque_topic) is unwired and the stroke is
    // silently still riding L.phase — i.e. the arm you think you ran did not load.
    // 2026-08-04 — mirrored from snapshot_state()'s `mod` dict so the live inspector sees
    // the same coordination + per-leg-error read the headless JSONL does.
    j["interleg_plv_win"] = interleg_plv_win();
    j["plv_win_support"]  = interleg_plv_win_support();
    j["panic_eff"]        = double(panic_) * panic_strength_;
    j["goal_bearing_msgs"] = double(gb_msgs_);
    j["goal_bearing_err"]   = gb_seen_ ? double(std::atan2(gb_x_, gb_y_) * 0.318309886f) : 0.0;
    j["couple_w_spr"]     = cw_n_ ? cw_spr_acc_ / double(cw_n_) : 0.0;
    j["couple_w_mean"]    = cw_n_ ? 1.0 : 0.0;
    {
        std::vector<double> tl, al;
        tl.reserve(legs_.size()); al.reserve(legs_.size());
        for (auto const& L : legs_) {
            tl.push_back(L.initialized ? double(L.tle_ema) : -1.0);
            al.push_back(L.initialized ? double(L.amp_ema) : -1.0);
        }
        j["tle_leg"] = tl;
        j["amp_leg"] = al;
    }
    j["step_phase_src"]  = stroke_phase_src_;
    j["step_lock"]       = step_lock_frac_;      // frac of legs with a locked step clock
    j["step_period"]     = step_period_mean_;    // mean measured step period, ticks (~26-30)
    // Mean |phase error at touchdown|, radians.  0 = the clock predicts footfall exactly;
    // ~pi/2 = it is fighting the body.  The NON-tautological lock-quality read (the pull
    // is partial, so this is earned rather than imposed) — unlike td_plv, which a
    // touchdown-referenced phase satisfies by construction.
    j["step_td_err"]     = step_td_err_n_ ? step_td_err_acc_ / double(step_td_err_n_) : 0.0;
    j["step_lock_flips"] = step_lock_flips_;
    {   // per-leg phase, for the UI and for spotting one leg that never locks
        std::vector<double> sp; sp.reserve(legs_.size());
        for (auto const& L : legs_) sp.push_back(L.step_locked ? double(L.step_phase) : -1.0);
        j["step_phase_legs"] = sp;               // −1 = that leg is still on L.phase
    }
    // Footfall raster for the live inspector — shipped whole (see update_gait_raster).
    // Unrolled OLDEST-FIRST so the widget can draw it left-to-right without knowing the
    // ring's head, and truncated to what has actually been written.
    if (gait_raster_diag_ > 0.0 && int(raster_.size()) == kRasterLen) {
        const int n = int(std::min<int64_t>(raster_n_, kRasterLen));
        std::vector<int> out; out.reserve(n);
        for (int k = 0; k < n; ++k)
            out.push_back(int(raster_[(raster_head_ - n + k + 2 * kRasterLen) % kRasterLen]));
        j["gait_raster"]      = out;    // bits 0-3 contact, 4-7 stroke sign, 8-11 in_swing
        j["gait_raster_legs"] = std::min(n_legs_, 4);
    }
    // Is the coordination probe already annealed by progress→commit?  If this sits at 0
    // on flat ground then a precision gate on the same σ would be a TAUTOLOGY, and
    // CLAUDE.md §3.2 rule 1 says find that out before building it.
    // Purchase gate observability (CLAUDE.md §3.2 rule 5 — a gate that never fired has
    // already shipped here once as silent dead code).  mean 1.0 with spread EXACTLY 0
    // means the gate never ran: either the gain is 0 or torque_topic is unwired.
    j["stroke_load_active"]  = (stroke_load_gain_ != 0.0 && have_torque_);
    j["stroke_gate_mean"]    = stroke_gate_mean_;
    j["stroke_gate_spread"]  = stroke_gate_spread_;
    j["stroke_gate"]         = stroke_gate_;
    // Swing tuck: fired-or-not as a number, and the yaw disturbance it targets.
    // yaw_swing_excess is the headline — mean |yaw rate| while ANY foot is airborne minus
    // the all-four-down reference.  If folding the limb works, this falls.
    j["tibia_plumb_active"]  = (tibia_plumb_gain_ != 0.0);
    j["tibia_off_mean"]      = ga_tib_n_ ? float(ga_tib_acc_ / double(ga_tib_n_)) : 0.0f;   // mean |shank off vertical|, radians
    j["swing_tuck_active"]   = ((swing_tuck_hip2_ != 0.0 || swing_tuck_knee_ != 0.0) && have_contact_);
    j["swing_tuck_frac"]     = swing_tuck_frac_;
    {
        const double ap = ga_yaw_allplant_n_ ? ga_yaw_allplant_ / double(ga_yaw_allplant_n_) : 0.0;
        const double sw = ga_yaw_anyswing_n_ ? ga_yaw_anyswing_ / double(ga_yaw_anyswing_n_) : 0.0;
        j["yaw_allplant"]      = ap;
        j["yaw_anyswing"]      = sw;
        j["yaw_swing_excess"]  = sw - ap;
        std::vector<double> per(ga_yaw_leg_.size(), 0.0);
        for (size_t k = 0; k < per.size(); ++k)
            per[k] = ga_yaw_leg_n_[k] ? ga_yaw_leg_[k] / double(ga_yaw_leg_n_[k]) : 0.0;
        j["yaw_per_leg"]       = per;   // [FL,FR,RL,RR] — which limb spins the body most
        // The impulse split — this is the one that can see a swing reaction torque.
        const double apd = ga_yawd_allplant_n_ ? ga_yawd_allplant_ / double(ga_yawd_allplant_n_) : 0.0;
        const double swd = ga_yawd_anyswing_n_ ? ga_yawd_anyswing_ / double(ga_yawd_anyswing_n_) : 0.0;
        j["yawd_allplant"]     = apd;
        j["yawd_anyswing"]     = swd;
        j["yawd_swing_excess"] = swd - apd;
        std::vector<double> perd(ga_yawd_leg_.size(), 0.0);
        for (size_t k = 0; k < perd.size(); ++k)
            perd[k] = ga_yawd_leg_n_[k] ? ga_yawd_leg_[k] / double(ga_yawd_leg_n_[k]) : 0.0;
        j["yawd_per_leg"]      = perd;
    }
    // 2026-08-05 — the commit loop, streamed so the inspector can PLOT it.  The operator
    // is tuning commit_prec_gain by eye and needs to see the three quantities that make up
    // the loop, not just its behavioural residue: what the body's own prediction quality is
    // doing (commit_prec), how far commit has ramped (commit_boost), and the exploration
    // noise it gates (explore_mult).  commit_prec == 1.0 exactly means the lever is OFF.
    // 2026-08-05 — the commit chain END TO END, because the operator is reading these
    // against each other and fwd_progress_ema was the one link that was INVISIBLE despite
    // being the INPUT to all of it: commit_ticks gates on it, and intent error is measured
    // from it.  A chain you can only see the output of cannot be diagnosed.
    j["fwd_progress_ema"]    = fwd_progress_ema_;
    // Report the SPREAD-NORMALIZED error the controller actually descends, not the raw
    // one -- reading a different quantity than the mechanism uses is how the yaw-dominance
    // defect stayed invisible.  Raw terms are broken out so the mix stays auditable.
    j["intent_err"]          = intent_seen_ ? intent_err_norm() : -1.0f;
    j["intent_err_fwd"]      = intent_seen_ ? std::fabs(fwd_progress_ema_ - intent_v_) : -1.0f;
    j["intent_err_yaw"]      = intent_seen_ ? std::fabs(yaw_rate_ema_ - intent_w_)     : -1.0f;
    j["intent_err_scale"]    = err_run_ema_;
    j["couple_R"]            = couple_R_diag_;      // 1 = legs phase-locked, 0 = incoherent
    j["phase_freq"]          = phase_freq_diag_;    // rad/tick advance of L.phase
    j["phase_retro"]         = phase_retro_diag_;   // fraction of ticks running BACKWARDS
    j["res_freq"]            = res_w_;              // learned fwd_v frequency, rad/tick
    j["res_period"]          = res_w_ > 1e-4f ? (2.0 * M_PI / double(res_w_)) : -1.0;
    j["res_amp"]             = res_amp_ema_;
    j["res_lock"]            = std::sqrt(res_lock_cos_ * res_lock_cos_
                                       + res_lock_sin_ * res_lock_sin_);
    j["lookahead_dev"]       = la_dev_ema_;   // ||x_eff - x||: consumer check
    j["rhythm_dev"]          = rhythm_dev_diag_;
    j["rhythm_spread"]       = rhythm_spread_ema_;
    j["commit_prec"]         = commit_prec_diag_;
    j["commit_boost"]        = commit_boost_;
    j["explore_mult"]        = explore_mult_diag_;
    j["support_bin"]         = support_bin_diag_;
    j["support_value"]       = support_value_diag_;
    j["support_mult"]        = support_mult_diag_;
    // Raw |dx|/|du| — the numerator of the actuator-search criterion value =
    // responsiveness/(motor_tle+ε).  In the BODY log (not only the inspector diag) so a
    // seedavg arm can score itself on the criterion; an unparsed metric is invisible.
    j["support_resp"]        = support_resp_diag_;
    j["sr_duty"] = sr_stance_ticks_ ? double(sr_release_ticks_) / double(sr_stance_ticks_) : 0.0;
    j["gait_phase"]          = gait_phase_;        // has the imposed trot [0,π,π,0] drifted?
    j["coord_best_phase"]    = coord_best_phase_;  // the stored winner it reverts to
    // The coordination search, made observable: which fitness is ranking probes, and the
    // incumbent's score.  Without these, "is the phase search locked onto something" is
    // unanswerable — and this search stores a winner, so it CAN lock in.
    j["coord_fitness_mode"]  = coord_fitness_mode_;
    j["coord_best_fitness"]  = coord_best_fitness_;
    j["coord_activity"]      = amp_ema_mean();   // the anti-freeze factor of mode 1
    j["upright"]             = upright_;         // ~basis.y.y; drives the homeostat gate
    j["homeo_leak_eff"]      = homeo_leak_eff_;  // forgetting rate actually applied
    j["homeo_gated"]         = (homeo_upright_gate_ > 0.0 && upright_ < float(homeo_upright_gate_));
    j["flow_quality"]        = flow_quality_diag_; // magnitude·predictability (drives the flow stroke amp)
    {
        std::vector<float> pc(n_legs_, 0.0f);
        for (int i = 0; i < n_legs_ && i < int(legs_.size()); ++i) pc[i] = legs_[i].prop_credit;
        j["prop_credit"] = pc;
    }
    // Effective per-joint postural gains actually applied this tick (postural_gain ×
    // profile).  Makes the array-vs-scalar interaction observable so a knob-turn can
    // never silently be a no-op.
    {
        const bool pjg = int(postural_gain_joints_.size()) == motor_dim_;
        std::vector<float> eff(motor_dim_);
        for (int jj = 0; jj < motor_dim_; ++jj)
            eff[jj] = float(postural_gain_) * (pjg ? float(postural_gain_joints_[jj]) : 1.0f);
        j["postural_eff"] = eff;
    }
    j["out_mag"]     = outmag_ema_mean();
    j["cog_steer"]   = cog_steer_;              // brain → differential (turn)
    j["cog_steer_msgs"] = cog_steer_msgs_;
    j["cog_thrust"]  = cog_thrust_;             // brain → common-mode (fwd/reverse)
    j["cog_thrust_msgs"] = cog_thrust_msgs_;
    j["fwd_v"]       = fwd_v_;                   // chassis forward velocity (controllability)
    j["chassis_h"]     = chassis_h_;             // chassis height norm (1=target, ~0=on the ground/collision)
    j["chassis_h_ema"] = chassis_h_ema_;         // smoothed chassis height
    j["chassis_h_max"] = chassis_h_max_;         // self-discovered height ceiling
    j["height_bias"]   = height_bias_;           // integrated lift bias (hip2)
    j["height_k_eff"]  = height_k_eff_;          // adapted setpoint fraction (belly-grounding)
    j["height_rest_frac"] = height_rest_frac_;   // height-defense fade: 1 at rest → 0 while moving fwd
    { float s = 0.0f; int c = 0; for (auto const& L : legs_) if (L.initialized) { s += L.Cphi.norm(); ++c; }
      j["embed_norm"] = c ? s / float(c) : 0.0f; }   // mean ‖Cphi‖ — how much POSTURE phase-conditioning HK has learned
    { float s = 0.0f; int c = 0; for (auto const& L : legs_) if (L.initialized) { s += L.Cvel.norm(); ++c; }
      j["vel_embed_norm"] = c ? s / float(c) : 0.0f; }   // mean ‖Cvel‖ — how much VELOCITY feed-forward (propulsive pump) HK has learned
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

void MotorEPMv2::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1 && version != 2)
        throw std::runtime_error("MotorEPMv2::restore_state: unknown version " + std::to_string(version));
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
        L.hip1_dc    = lj.value("hip1_dc", 0.0f);
        L.prop_credit = lj.value("prop_credit", 0.0f);
        // Stroke-to-step lock (absent in older snapshots → the Leg{} defaults, i.e. the
        // clock re-locks from the next two touchdowns and the stroke uses L.phase until
        // then, which is the same safe fallback a cold start takes).
        L.step_phase   = lj.value("step_phase",   0.0f);
        L.step_per_ema = lj.value("step_per_ema", 0.0f);
        L.step_omega   = lj.value("step_omega",   0.0f);
        L.last_td_tick = lj.value("last_td_tick", int64_t(-1));
        L.td_count     = lj.value("td_count",     int32_t(0));
        L.td_run       = lj.value("td_run",       int32_t(0));
        L.td_cand_tick = lj.value("td_cand_tick", int64_t(-1));
        L.td_contact   = lj.value("td_contact",   true);
        L.step_locked  = lj.value("step_locked",  false);
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
        L.Cvel = Eigen::MatrixXf::Zero(m, 2);   // ensure valid (m,2) dims even for snapshots without a velocity map
        if (lj.contains("Cphi")) L.Cphi = toM(vecf(lj.at("Cphi")), m, 2);   // legacy snapshots → keep zero-init
        if (lj.contains("Cvel")) L.Cvel = toM(vecf(lj.at("Cvel")), m, 2);   // legacy snapshots → keep zero-init
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
        swing_frac_ema_ = mod.value("swing_frac",     swing_frac_ema_);
        cruse_bias_mean_ = mod.value("cruse_bias",    cruse_bias_mean_);
        phase_agree_ema_ = mod.value("phase_agree",  phase_agree_ema_);
        legphase_agree_ema_ = mod.value("legphase_agree", legphase_agree_ema_);
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
        heading_bearing_   = mod.value("heading_bearing",   heading_bearing_);
        fwd_progress_ema_  = mod.value("fwd_progress_ema",   fwd_progress_ema_);
        stuck_ticks_       = mod.value("stuck_ticks",        stuck_ticks_);
        stuck_boost_       = mod.value("stuck_boost",        stuck_boost_);
        commit_ticks_      = mod.value("commit_ticks",       commit_ticks_);
        commit_boost_      = mod.value("commit_boost",       commit_boost_);
        flow_ema_          = mod.value("flow_ema",           flow_ema_);
        flow_vol_ema_      = mod.value("flow_vol_ema",       flow_vol_ema_);
        // Phase-0 gait-alignment accumulators (diagnostic; restored so a clone reports
        // the same measurement rather than restarting its averages mid-run).
        ga_td_cos_    = mod.value("ga_td_cos",    ga_td_cos_);
        ga_td_sin_    = mod.value("ga_td_sin",    ga_td_sin_);
        ga_td_n_      = mod.value("ga_td_n",      ga_td_n_);
        ga_sd_cos_    = mod.value("ga_sd_cos",    ga_sd_cos_);
        ga_sd_sin_    = mod.value("ga_sd_sin",    ga_sd_sin_);
        ga_sd_n_      = mod.value("ga_sd_n",      ga_sd_n_);
        ga_align_acc_ = mod.value("ga_align_acc", ga_align_acc_);
        ga_align_n_   = mod.value("ga_align_n",   ga_align_n_);
        ga_stance_pos_= mod.value("ga_stance_pos",ga_stance_pos_);
        ga_stance_n_  = mod.value("ga_stance_n",  ga_stance_n_);
        ga_swing_pos_ = mod.value("ga_swing_pos", ga_swing_pos_);
        ga_swing_n_   = mod.value("ga_swing_n",   ga_swing_n_);
        ga_contact_acc_ = mod.value("ga_contact_acc", ga_contact_acc_);
        ga_contact_n_   = mod.value("ga_contact_n",   ga_contact_n_);
        ga_tq_stance_   = mod.value("ga_tq_stance",   ga_tq_stance_);
        ga_tq_stance_n_ = mod.value("ga_tq_stance_n", ga_tq_stance_n_);
        ga_tq_swing_    = mod.value("ga_tq_swing",    ga_tq_swing_);
        ga_tq_swing_n_  = mod.value("ga_tq_swing_n",  ga_tq_swing_n_);
        ga_tq_agree_    = mod.value("ga_tq_agree",    ga_tq_agree_);
        ga_tq_agree_n_  = mod.value("ga_tq_agree_n",  ga_tq_agree_n_);
        ga_tq_h1_agree_   = mod.value("ga_tq_h1_agree",   ga_tq_h1_agree_);
        ga_tq_h1_agree_n_ = mod.value("ga_tq_h1_agree_n", ga_tq_h1_agree_n_);
        if (mod.contains("ga_tq_h1_ema")) ga_tq_h1_ema_ = mod["ga_tq_h1_ema"].get<std::vector<float>>();
        ga_mv_stance_   = mod.value("ga_mv_stance",   ga_mv_stance_);
        ga_mv_stance_n_ = mod.value("ga_mv_stance_n", ga_mv_stance_n_);
        ga_mv_swing_    = mod.value("ga_mv_swing",    ga_mv_swing_);
        ga_mv_swing_n_  = mod.value("ga_mv_swing_n",  ga_mv_swing_n_);
        step_td_err_acc_ = mod.value("step_td_err_acc", step_td_err_acc_);
        step_td_err_n_   = mod.value("step_td_err_n",   step_td_err_n_);
        step_lock_flips_ = mod.value("step_lock_flips_state", step_lock_flips_);
        if (mod.contains("stroke_load_ema")) stroke_load_ema_ = mod["stroke_load_ema"].get<std::vector<float>>();
        if (mod.contains("ga_yaw_leg"))   ga_yaw_leg_   = mod["ga_yaw_leg"].get<std::vector<double>>();
        if (mod.contains("ga_yaw_leg_n")) ga_yaw_leg_n_ = mod["ga_yaw_leg_n"].get<std::vector<int64_t>>();
        ga_yaw_allplant_   = mod.value("ga_yaw_allplant",   ga_yaw_allplant_);
        ga_yaw_allplant_n_ = mod.value("ga_yaw_allplant_n", ga_yaw_allplant_n_);
        ga_yaw_anyswing_   = mod.value("ga_yaw_anyswing",   ga_yaw_anyswing_);
        ga_yaw_anyswing_n_ = mod.value("ga_yaw_anyswing_n", ga_yaw_anyswing_n_);
        if (mod.contains("ga_yawd_leg"))   ga_yawd_leg_   = mod["ga_yawd_leg"].get<std::vector<double>>();
        if (mod.contains("ga_yawd_leg_n")) ga_yawd_leg_n_ = mod["ga_yawd_leg_n"].get<std::vector<int64_t>>();
        ga_yawd_allplant_   = mod.value("ga_yawd_allplant",   ga_yawd_allplant_);
        ga_yawd_allplant_n_ = mod.value("ga_yawd_allplant_n", ga_yawd_allplant_n_);
        ga_yawd_anyswing_   = mod.value("ga_yawd_anyswing",   ga_yawd_anyswing_);
        ga_yawd_anyswing_n_ = mod.value("ga_yawd_anyswing_n", ga_yawd_anyswing_n_);
        ga_tib_acc_ = mod.value("ga_tib_acc", ga_tib_acc_);
        ga_tib_n_   = mod.value("ga_tib_n",   ga_tib_n_);
        if (mod.contains("ga_tq_ema"))      ga_tq_ema_      = mod["ga_tq_ema"].get<std::vector<float>>();
        if (mod.contains("ga_tq_j_stance")) ga_tq_j_stance_ = mod["ga_tq_j_stance"].get<std::vector<double>>();
        if (mod.contains("ga_tq_j_swing"))  ga_tq_j_swing_  = mod["ga_tq_j_swing"].get<std::vector<double>>();
        ga_tq_j_stance_n_ = mod.value("ga_tq_j_stance_n", ga_tq_j_stance_n_);
        ga_tq_j_swing_n_  = mod.value("ga_tq_j_swing_n",  ga_tq_j_swing_n_);
        if (mod.contains("ga_hip1_ema"))    ga_hip1_ema_    = mod["ga_hip1_ema"].get<std::vector<float>>();
        if (mod.contains("ga_hip1_per"))    ga_hip1_per_    = mod["ga_hip1_per"].get<std::vector<float>>();
        if (mod.contains("ga_knee_per"))    ga_knee_per_    = mod["ga_knee_per"].get<std::vector<float>>();
        if (mod.contains("ga_foot_per"))    ga_foot_per_    = mod["ga_foot_per"].get<std::vector<float>>();
        if (mod.contains("ga_con_per"))     ga_con_per_     = mod["ga_con_per"].get<std::vector<float>>();
        if (mod.contains("ga_hip1_last"))   ga_hip1_last_   = mod["ga_hip1_last"].get<std::vector<int64_t>>();
        if (mod.contains("ga_knee_last"))   ga_knee_last_   = mod["ga_knee_last"].get<std::vector<int64_t>>();
        if (mod.contains("ga_foot_last"))   ga_foot_last_   = mod["ga_foot_last"].get<std::vector<int64_t>>();
        if (mod.contains("ga_con_last"))    ga_con_last_    = mod["ga_con_last"].get<std::vector<int64_t>>();
        if (mod.contains("ga_hip1_above"))  ga_hip1_above_  = mod["ga_hip1_above"].get<std::vector<char>>();
        if (mod.contains("ga_knee_above"))  ga_knee_above_  = mod["ga_knee_above"].get<std::vector<char>>();
        if (mod.contains("ga_prev_contact"))ga_prev_contact_= mod["ga_prev_contact"].get<std::vector<char>>();
        if (mod.contains("ga_prev_swing"))  ga_prev_swing_  = mod["ga_prev_swing"].get<std::vector<char>>();
    }
}

} // namespace ogma
