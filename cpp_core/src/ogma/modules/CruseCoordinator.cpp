#include "ogma/modules/CruseCoordinator.hpp"

#include <algorithm>
#include <cmath>
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

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("CruseCoordinator param '" + key + "' must be string");
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("CruseCoordinator param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("CruseCoordinator param '" + key + "' must be integer");
}
bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("CruseCoordinator param '" + key + "' must be boolean");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("CruseCoordinator param '" + key + "' must be string array");
}

} // namespace

CruseCoordinator::CruseCoordinator()  = default;
CruseCoordinator::~CruseCoordinator() = default;

std::string_view CruseCoordinator::type_name() const { return "CruseCoordinator"; }

std::vector<TopicSpec> CruseCoordinator::input_topics() const {
    std::vector<TopicSpec> v;
    v.emplace_back(feet_y_topic_, std::type_index(typeid(ProprioToken)),
                   SubscriptionKind::Direct, /*required=*/true);
    if (!body_state_topic_.empty()) {
        v.emplace_back(body_state_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    }
    for (auto const& pm : premotor_state_) {
        v.emplace_back(pm.policy_topic, std::type_index(typeid(PolicyToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    }
    return v;
}

std::vector<TopicSpec> CruseCoordinator::output_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(premotor_state_.size() + phase_output_topics_.size());
    for (auto const& pm : premotor_state_) {
        v.emplace_back(std::string(topics::kRhythmBiasPrefix) + pm.id,
                       std::type_index(typeid(RhythmBiasToken)));
    }
    // Phase 7.17 closed-loop rhythm output (one ProprioToken per leg).
    if (publish_phase_signals_) {
        for (auto const& t : phase_output_topics_) {
            v.emplace_back(t, std::type_index(typeid(ProprioToken)));
        }
    }
    // 2026-05-29 — per-leg discrete bucket (swing=0 / stance=1) for per-bucket
    // Premotor policy specialization.  Drives the gait-bucket bet.
    if (publish_bucket_signals_) {
        for (auto const& t : bucket_output_topics_) {
            v.emplace_back(t, std::type_index(typeid(ProprioToken)));
        }
    }
    return v;
}

ParamSchema CruseCoordinator::params_schema() const {
    return {
        {"feet_y_topic",        ParamMutability::ConstructionOnly,
            "ProprioToken topic carrying per-leg foot-Y (4 floats: FL, FR, RL, RR).",
            ParamValue{std::string("reality.proprio.feet_y")}},
        {"leg_names",           ParamMutability::ConstructionOnly,
            "Leg names in order matching feet_y vector indices.  Default ['fl','fr','rl','rr'].",
            std::nullopt},
        {"anatomical_anterior", ParamMutability::ConstructionOnly,
            "Array of N strings (one per leg, in leg_names order) naming each leg's anatomical anterior, or '' if none.  Picrawler default: ['','','fl','fr'] = front legs have no anterior, RL's anterior is FL, RR's anterior is FR.",
            std::nullopt},
        {"contralateral",       ParamMutability::ConstructionOnly,
            "Array of N strings (one per leg) naming each leg's contralateral neighbor for Rule 3, or '' if none.  Picrawler default: ['fr','fl','rr','rl'] = FL↔FR and RL↔RR.",
            std::nullopt},
        {"premotor_ids",        ParamMutability::ConstructionOnly,
            "Array of Premotor ids that CruseCoordinator drives.  Same length as the other premotor_* arrays.",
            std::nullopt},
        {"premotor_leg_assignment", ParamMutability::ConstructionOnly,
            "Per-Premotor leg name (one of leg_names).",
            std::nullopt},
        {"premotor_joint_kind", ParamMutability::ConstructionOnly,
            "Per-Premotor joint kind: 'hip2' or 'knee' get bias when their leg's anterior is in swing.  Any other value (e.g. 'hip1', 'other') means this Premotor is published a zero bias.",
            std::nullopt},
        {"premotor_policy_topics", ParamMutability::ConstructionOnly,
            "Per-Premotor policy_output_topic (where CruseCoordinator subscribes to read intent_accels).",
            std::nullopt},
        {"premotor_stance_sign", ParamMutability::ConstructionOnly,
            "Optional per-Premotor stance sign (parallel array to premotor_ids).  +1 (default if unspecified) = legacy behavior: Cruse pushes the Premotor toward POSITIVE accel for stance.  -1 = Cruse pushes toward NEGATIVE accel for stance — needed when a leg's body-geometry convention is sign-inverted (mirrored joint axes).  Without per-leg sign, Cruse's bias is uniform across all 12 Premotors and pushes half the legs in the wrong direction (e.g. the diagonal-split attractor where one diagonal pushes down and the other reaches skyward).  Picrawler diagonal pattern (matches HIP1_SPLAY_OUT_SIGN [+1,-1,-1,+1]): [+1] for FL/RR hip2+knee, [-1] for FR/RL hip2+knee; hip1 Premotors are ignored by Cruse anyway (joint_kind filter) so their sign is don't-care.",
            std::nullopt},
        {"n_intents",           ParamMutability::ConstructionOnly,
            "Number of intents per Premotor.  Must match Premotor's n_intents.  Default 5.",
            ParamValue{int64_t{5}}},
        {"cruse_bias_gain",     ParamMutability::HotMutable,
            "Base scalar on published bias vector.  0 = mechanism off (default, byte-identical legacy).  Recommended 0.5-1.5; bias is FURTHER scaled by per-Premotor violation_ema so the rule fades when brain is naturally compliant.",
            ParamValue{0.0}},
        {"cruse_bias_gain_knee", ParamMutability::HotMutable,
            "Per-joint-kind gain multiplier applied to knee Premotors on top of cruse_bias_gain.  Default 0.0 (knee Premotors receive ZERO Cruse bias — they learn their own knee timing from foot-clearance + body-height signals).  Set to 1.0 to restore legacy 'Cruse pushes knee toward fold during stance' behavior.  Joseph 2026-06-08: V2 trace showed knee_bias 0.4-0.5 fold pressure driving a claw pose; knee is the wrong joint for Cruse's stance/swing coordination layer.  hip2 is unaffected — keeps full cruse_bias_gain.",
            ParamValue{0.0}},
        {"cruse_bias_gain_hip1", ParamMutability::HotMutable,
            "Move 2 (2026-06-08) — per-joint-kind gain for HIP1 (yaw / swing direction) Premotors on top of cruse_bias_gain.  Default 0.0 = legacy (hip1 unbiased; the eligible filter previously excluded hip1 entirely, leaving the brain with no coordination signal for fore-aft leg swing → no gait emergence — Joseph: 'this lack of hip1 is likely a main reason we have not seen a gait emerge; there is no swing of the leg possible').  Set to 1.0 in walking stages: during stance, bias pushes hip1 toward posterior (body forward through traction); during swing (Rule 2 release fires), bias inverts toward anterior (leg lifts forward).  Per-leg direction via premotor_stance_sign — picrawler empirical [+1,-1,+1,-1] for FL/FR/RL/RR hip1 because positive hip1 u rotates left and right legs in opposite world-frame directions.  Keep at 0.0 during standing-only stages to avoid 'swing bias' leaking into posture choice.",
            ParamValue{0.0}},
        {"cruse_bias_gain_hip2", ParamMutability::HotMutable,
            "Move 4 (2026-06-09) — per-joint-kind gain for HIP2 (lift / vertical axis) Premotors on top of cruse_bias_gain.  Default 1.0 = legacy (hip2 used the implicit cruse_bias_gain alone, no per-joint scaling).  Set > 1.0 to amplify hip2's down-during-stance + up-during-swing biases without scaling hip1 and knee.  Joseph 2026-06-09 UI observation after hip1 sign fix: 'stepping is happening; if there was a bit of hip2 bias downward (positive) during the hip1 swings it would help plant the thrusting leg.  also a bit more hip2 upward (negative) would help with swing forward.'  Completes the per-joint gain trio knee/hip1/hip2 introduced by Moves 1, 2, 4.",
            ParamValue{1.0}},
        {"rhythm_inject_gain",   ParamMutability::HotMutable,
            "E2 (2026-06-10) — rhythm INJECTION.  Adds a continuous phase-driven term rhythm_inject_gain*cos(phi_free) to total_factor, where phi_free is the per-leg entrained free-running oscillator.  Injects a periodic stance-press(phi=0)/swing-lift(phi=pi) rhythm the legs lock onto and that entrains to footfalls (closed loop) — bootstraps a rhythm that doesn't yet exist, unlike the transition-triggered rules.  Gentle logit bias (brain can override for balance).  0 = off (legacy).  Recommended 0.3-1.0.",
            ParamValue{0.0}},
        {"joints_topic",         ParamMutability::ConstructionOnly,
            "Move 5 — ProprioToken topic carrying 12-channel joint angle vector ([hip1_fl,hip1_fr,hip1_rl,hip1_rr, hip2_fl,hip2_fr,hip2_rl,hip2_rr, knee_fl,knee_fr,knee_rl,knee_rr]).  Each channel normalised to [-1,+1].  Cruse reads current joint position from this topic to compute productive_score for the saturation gate.  Default 'reality.proprio.joints' matches picrawler body convention.",
            ParamValue{std::string("reality.proprio.joints")}},
        {"premotor_joint_position_indices", ParamMutability::ConstructionOnly,
            "Move 5 — per-Premotor index into the joints_topic vector (parallel array to premotor_ids).  E.g. for the W1 config's interleaved Premotor order [FL_hip1, FR_hip1, RL_hip1, RR_hip1, FL_hip2, FL_knee, FR_hip2, FR_knee, RL_hip2, RL_knee, RR_hip2, RR_knee] the indices into the joint-kind-grouped joints vector are [0,1,2,3, 4,8, 5,9, 6,10, 7,11].  Optional — if omitted, saturation gate is skipped (legacy unconditional bias).",
            std::nullopt},
        {"saturation_gate_enabled", ParamMutability::HotMutable,
            "Move 5 — when true, gate Cruse bias by the current joint position.  Computes productive_score = current_u × bias_direction_sign; if productive_score is OUTSIDE [saturation_zone_min, saturation_zone_max], the bias is zeroed for that Premotor this tick.  Two failure modes addressed: (1) rudder — leg planted on wrong side of perpendicular sweeps through perpendicular when stance bias pushes posterior, dragging body sideways instead of forward; (2) saturation — leg already near target extreme, pushing further is wasted authority that may overshoot.  Default false = legacy unconditional bias (no behavior change).  Joseph 2026-06-09: 'plant and push has the most forward power when the leg is perpendicular, not parallel, with the desired direction of motion.'",
            ParamValue{false}},
        {"saturation_zone_min",  ParamMutability::HotMutable,
            "Move 5 — minimum productive_score (= current_u × bias_direction_sign) for bias to apply.  Default 0.0 = leg must be at or past perpendicular in bias direction.  Below 0 means leg is on the wrong side (rudder case).  Set negative (e.g. -0.2) to allow some slack before the gate kicks in.",
            ParamValue{0.0}},
        {"saturation_zone_max",  ParamMutability::HotMutable,
            "Move 5 — maximum productive_score for bias to apply.  Default 0.9 = leg must not be > 90% of the way to the bias-direction extreme (saturation cutoff).  Above this, leg is already 'at target' and further bias is wasted/may overshoot.  Set 1.0 to disable the saturation cutoff (gate only checks the wrong-side floor).",
            ParamValue{0.9}},
        {"violation_ema_alpha", ParamMutability::HotMutable,
            "EMA rate for per-Premotor violation tracking.  Default 0.05 (~20-tick window).",
            ParamValue{0.05}},
        {"max_violation_ema",   ParamMutability::HotMutable,
            "Cap on violation_ema — sets the ceiling of effective bias (bias = base_gain * min(ema, cap)).  Default 1.0.",
            ParamValue{1.0}},
        {"hysteresis_low_frac", ParamMutability::HotMutable,
            "Plant threshold fraction (foot_y_low_ema + frac * (high - low)).  Default 0.25.",
            ParamValue{0.25}},
        {"hysteresis_high_frac",ParamMutability::HotMutable,
            "Lift threshold fraction.  Default 0.50.",
            ParamValue{0.50}},
        {"hysteresis_ema_alpha",ParamMutability::HotMutable,
            "Adaptive-threshold EMA rate.  Default 0.01.",
            ParamValue{0.01}},
        {"publish_when_silent", ParamMutability::HotMutable,
            "If true (default), publish zero-bias RhythmBiasToken even when no rule is active, so Premotor subscription stays warm.",
            ParamValue{true}},
        {"adaptive_magnitude",  ParamMutability::HotMutable,
            "false (default, Phase 7.13 — Hector-faithful constant gain): rules fire at fixed `cruse_bias_gain` whenever active.  true (Phase 7.11 v1 mode): bias scales with `violation_ema` — fades when brain is naturally compliant.",
            ParamValue{false}},
        {"enable_rule_1",       ParamMutability::HotMutable,
            "Enable Rule 1 (no-swing-overlap): anterior in swing → STANCE bias.  Default true.",
            ParamValue{true}},
        {"enable_rule_2",       ParamMutability::HotMutable,
            "Enable Rule 2 (release): anterior just-touchdown → SWING bias (constructive lift).  Default true.",
            ParamValue{true}},
        {"enable_rule_3",       ParamMutability::HotMutable,
            "Enable Rule 3 (contralateral load tolerance): contralateral in swing → strengthen STANCE bias.  Default true.",
            ParamValue{true}},
        {"rule2_window_ticks",  ParamMutability::HotMutable,
            "Bootstrap-only fallback window (in ticks) for Rule 2.  Used when the just-planted leg has < rule6_min_samples observed swings (per-leg).  After bootstrap, the actual Rule 2 window = rule2_window_fraction × swing_duration_ema (rhythm-coupled to body's own observed timing).  Default 15.",
            ParamValue{int64_t{15}}},
        {"rule2_window_fraction", ParamMutability::HotMutable,
            "Rule 2 window as a fraction of the just-planted leg's own swing_duration_ema (after bootstrap).  E.g. 0.25 = window is 25% of the leg's typical swing time.  Makes Rule 2 timing scale with the body's natural rhythm rather than impose a fixed tick clock.  Default 0.25.",
            ParamValue{0.25}},
        {"rule2_min_swing_ticks", ParamMutability::HotMutable,
            "Minimum ticks the leg must have been in swing before its touchdown counts as a Rule 2 trigger.  Default 0 = legacy (any touchdown qualifies, including chassis-dip glitches where foot_y wobbled briefly across stance_y_threshold).  Recommended 5-10 (~100-200ms at 50Hz) — filters out spurious touchdowns from a body that's just dipping/recovering rather than actually swinging.",
            ParamValue{int64_t{0}}},
        {"rule1_violation_boost", ParamMutability::HotMutable,
            "Extra multiplier applied to Rule 1 contribution when Rule 1 is active AND currently violated (this leg in swing while anterior also in swing).  Default 1.5.  Set to 1.0 to disable boost.",
            ParamValue{1.5}},
        {"rule3_weight",        ParamMutability::HotMutable,
            "Rule 3 stance-bias weight relative to Rule 1 (Rule 1 weight is implicitly 1.0).  Default 0.5.",
            ParamValue{0.5}},
        {"coupling_mode",       ParamMutability::ConstructionOnly,
            "Topology of inter-leg coupling.  'chain' (default, legacy): directional cascade via anatomical_anterior.  'symmetric_dwell' (Phase 7.13 v3): any leg can initiate; Rule 1 fires on any coupled_neighbor in swing; Rule 2 fires only on the SUCCESSOR (longest stance-dwell among coupled_neighbors of just-planted legs).",
            ParamValue{std::string("chain")}},
        {"coupled_neighbors",   ParamMutability::ConstructionOnly,
            "Array of N comma-separated strings (one per leg) listing coupled-neighbor leg names for symmetric_dwell mode.  Picrawler default (non-diagonal, supports trot): ['fr,rl', 'fl,rr', 'rr,fl', 'rl,fr'] = FL↔FR + FL↔RL, FR↔RR, RL↔RR (diagonal pairs FL/RR and FR/RL NOT coupled, so they can swing together in trot).  Ignored in chain mode.",
            std::nullopt},
        {"body_state_topic",    ParamMutability::ConstructionOnly,
            "Optional ProprioToken topic carrying chassis_y_norm in element 0 (single-float vector).  When set, CruseCoordinator multiplies its effective bias magnitude by this value, so the coordinator goes silent when the body is fallen.  Picrawler value: 'reality.proprio.chassis_y_norm'.  Default '' = body-state gate disabled (legacy behavior).",
            ParamValue{std::string("")}},
        {"body_state_min_threshold", ParamMutability::HotMutable,
            "Hard floor on chassis_y_norm for body-state gate.  When chassis_y_norm < threshold, magnitude collapses to 0 instead of scaling proportionally.  Prevents pathological rules from firing at all when body is genuinely fallen.  Default 0.3 (body has to be at least 30% of target_height for ANY Cruse firing).",
            ParamValue{0.3}},
        {"warmup_ticks",        ParamMutability::HotMutable,
            "Amplitude warmup window in ticks.  When > 0, cruse_bias_gain is multiplied by linear ramp factor (0 → 1) over the first N ticks after setup, giving Premotors a quiet window to learn standing via REINFORCE before Cruse rules add directional bias.  Recommended ~3600 (60 s @60Hz) for tabula rasa, or longer (10800 = 3 min) if body needs more time.  0 = off (legacy: rules fully active from tick 1).",
            ParamValue{int64_t{0}}},
        {"enable_rule_6",       ParamMutability::HotMutable,
            "Rule 6 (step duration memory).  Per-leg EMA of swing & stance durations.  When current swing > rule6_max_swing_ratio × swing_ema → bias toward STANCE.  When current stance > rule6_max_stance_ratio × stance_ema → bias toward SWING.  Breaks single-leg attractors (e.g. FL-waving while body stands).  Default false → byte-identical to v3 when off.",
            ParamValue{false}},
        {"rule6_ema_alpha",     ParamMutability::HotMutable,
            "EMA rate for swing/stance duration tracking.  Default 0.2 (~5-sample window for fast adaptation).",
            ParamValue{0.2}},
        {"rule6_max_swing_ratio",  ParamMutability::HotMutable,
            "Bias toward STANCE when current swing duration exceeds this × swing_duration_ema.  Default 2.0.",
            ParamValue{2.0}},
        {"rule6_max_stance_ratio", ParamMutability::HotMutable,
            "Bias toward SWING when current stance duration exceeds this × stance_duration_ema.  Default 1.5.",
            ParamValue{1.5}},
        {"rule6_max_swing_abs",    ParamMutability::HotMutable,
            "Absolute fallback cap (ticks) on swing duration while per-leg EMA is bootstrapping (< rule6_min_samples).  Default 1800 (30s @60Hz) — generous to support slow gaits like a chameleon (~5s swing) or walking stick.  Only fires on literally stuck states.  Lower this only if you want to enforce a faster tempo regime.",
            ParamValue{int64_t{1800}}},
        {"rule6_max_stance_abs",   ParamMutability::HotMutable,
            "Absolute fallback cap (ticks) on stance duration while EMA is bootstrapping.  Default 3600 (60s @60Hz) — generous to support very slow gaits (walking stick stance ~30s).  Only fires on literally stuck states.",
            ParamValue{int64_t{3600}}},
        {"rule6_min_samples",      ParamMutability::HotMutable,
            "# of completed phases before per-leg EMA is trusted over absolute cap.  Default 3.",
            ParamValue{int64_t{3}}},
        {"publish_phase_signals",  ParamMutability::HotMutable,
            "Phase 7.17 — when true, publish a 2-D [cos(phi), sin(phi)] ProprioToken per leg on phase_output_topics.  Closed-loop rhythm output: phi computed from actual touchdown/liftoff events + per-leg EMA durations.  Drop-in replacement for CPGOscillator's open-loop perceptual output.  Default false.",
            ParamValue{false}},
        {"phase_output_topics",    ParamMutability::ConstructionOnly,
            "Per-leg output topic names for the closed-loop rhythm phase signals.  Must equal leg_names length when publish_phase_signals=true.  Typical: ['rhythm.cpg.fl', 'rhythm.cpg.fr', 'rhythm.cpg.rl', 'rhythm.cpg.rr'] to drop-in replace CPGOscillator's perceptual output.",
            std::nullopt},
        {"publish_bucket_signals", ParamMutability::HotMutable,
            "2026-05-29 gait-bucket bet — when true, publish a single-float ProprioToken per leg (0.0=swing / 1.0=stance) on bucket_output_topics each tick.  Lets Premotors condition policy per-bucket via a learned bucket_bias term.  Default false.",
            ParamValue{false}},
        {"bucket_output_topics",   ParamMutability::ConstructionOnly,
            "Per-leg output topic names for the per-leg discrete bucket signals.  Must equal leg_names length when publish_bucket_signals=true.  Typical: ['bucket.fl', 'bucket.fr', 'bucket.rl', 'bucket.rr'].",
            std::nullopt},
        {"phase_mode",             ParamMutability::HotMutable,
            "Phase 7.19/7.20 — phase computation mode.  'state_event' (legacy): phi computed from time since last touchdown/liftoff; saturates when body is stuck.  'entrained_free': Kuramoto-lite free-running oscillator with entrainment pulses on transitions; smooth but loses body-derived period variance.  'smooth_fallback' (recommended): state_event as primary phase (preserves natural period variation cycle-to-cycle), free-running as backup that smoothly fades in only when no transitions for phase_blend_window ticks.  Best of both — variance during active gait, continuity during stalls.",
            ParamValue{std::string("state_event")}},
        {"phase_blend_window",     ParamMutability::HotMutable,
            "Phase 7.20 — for smooth_fallback mode.  Ticks of no transition (across any leg's touchdown/liftoff) before the published phase fully fades from state_event to free.  Below this, mix favours state_event (variance preserved).  At this, fully on free (continuity).  Default 60 (1s @60Hz).",
            ParamValue{60.0}},
        {"entrainment_strength",   ParamMutability::HotMutable,
            "Coupling strength K for entrained_free mode.  On each transition event, phi snaps toward the target phase (stance=0, swing=π) by K × angle_error.  0 = no coupling (pure free oscillator), 1.0 = full snap (no smoothing).  Default 0.20.",
            ParamValue{0.20}},
        {"default_omega",          ParamMutability::HotMutable,
            "Initial angular velocity (rad/tick) for entrained_free mode before any cycle is observed.  Default 0.0349 = 2π/180 = 1 cycle per 3 seconds @60Hz.  Once swing_duration_ema + stance_duration_ema have N samples, omega adapts to 2π/(swing_ema + stance_ema).",
            ParamValue{0.0349}},
    };
}

ParamMap CruseCoordinator::current_params() const {
    ParamMap p;
    p["feet_y_topic"]            = ParamValue{feet_y_topic_};
    p["n_intents"]               = ParamValue{int64_t(n_intents_)};
    p["cruse_bias_gain"]         = ParamValue{double(cruse_bias_gain_)};
    p["cruse_bias_gain_knee"]    = ParamValue{double(cruse_bias_gain_knee_)};
    p["cruse_bias_gain_hip1"]    = ParamValue{double(cruse_bias_gain_hip1_)};
    p["cruse_bias_gain_hip2"]    = ParamValue{double(cruse_bias_gain_hip2_)};
    p["rhythm_inject_gain"]      = ParamValue{double(rhythm_inject_gain_)};
    p["joints_topic"]            = ParamValue{joints_topic_};
    p["saturation_gate_enabled"] = ParamValue{saturation_gate_enabled_};
    p["saturation_zone_min"]     = ParamValue{double(saturation_zone_min_)};
    p["saturation_zone_max"]     = ParamValue{double(saturation_zone_max_)};
    p["violation_ema_alpha"]     = ParamValue{double(violation_ema_alpha_)};
    p["max_violation_ema"]       = ParamValue{double(max_violation_ema_)};
    p["hysteresis_low_frac"]     = ParamValue{double(hysteresis_low_frac_)};
    p["hysteresis_high_frac"]    = ParamValue{double(hysteresis_high_frac_)};
    p["hysteresis_ema_alpha"]    = ParamValue{double(hysteresis_ema_alpha_)};
    p["publish_when_silent"]     = ParamValue{publish_when_silent_};
    p["adaptive_magnitude"]      = ParamValue{adaptive_magnitude_};
    p["enable_rule_1"]           = ParamValue{enable_rule_1_};
    p["enable_rule_2"]           = ParamValue{enable_rule_2_};
    p["enable_rule_3"]           = ParamValue{enable_rule_3_};
    p["rule2_window_ticks"]      = ParamValue{int64_t(rule2_window_ticks_)};
    p["rule2_window_fraction"]   = ParamValue{double(rule2_window_fraction_)};
    p["rule2_min_swing_ticks"]   = ParamValue{int64_t(rule2_min_swing_ticks_)};
    p["rule1_violation_boost"]   = ParamValue{double(rule1_violation_boost_)};
    p["rule3_weight"]            = ParamValue{double(rule3_weight_)};
    p["coupling_mode"]           = ParamValue{coupling_mode_};
    p["body_state_topic"]        = ParamValue{body_state_topic_};
    p["body_state_min_threshold"]= ParamValue{double(body_state_min_threshold_)};
    p["warmup_ticks"]            = ParamValue{int64_t(warmup_ticks_)};
    p["enable_rule_6"]           = ParamValue{enable_rule_6_};
    p["rule6_ema_alpha"]         = ParamValue{double(rule6_ema_alpha_)};
    p["rule6_max_swing_ratio"]   = ParamValue{double(rule6_max_swing_ratio_)};
    p["rule6_max_stance_ratio"]  = ParamValue{double(rule6_max_stance_ratio_)};
    p["rule6_max_swing_abs"]     = ParamValue{int64_t(rule6_max_swing_abs_)};
    p["rule6_max_stance_abs"]    = ParamValue{int64_t(rule6_max_stance_abs_)};
    p["rule6_min_samples"]       = ParamValue{int64_t(rule6_min_samples_)};
    return p;
}

void CruseCoordinator::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("CruseCoordinator requires a non-null Bus");

    apply_param(params, "feet_y_topic",         [&](auto const& v){ feet_y_topic_         = get_string(v, "feet_y_topic"); });
    apply_param(params, "n_intents",            [&](auto const& v){ n_intents_            = std::max(2, int(get_int(v, "n_intents"))); });
    apply_param(params, "cruse_bias_gain",      [&](auto const& v){ cruse_bias_gain_      = std::max(0.0f, float(get_double(v, "cruse_bias_gain"))); });
    apply_param(params, "cruse_bias_gain_knee", [&](auto const& v){ cruse_bias_gain_knee_ = std::max(0.0f, float(get_double(v, "cruse_bias_gain_knee"))); });
    apply_param(params, "cruse_bias_gain_hip1", [&](auto const& v){ cruse_bias_gain_hip1_ = std::max(0.0f, float(get_double(v, "cruse_bias_gain_hip1"))); });
    apply_param(params, "cruse_bias_gain_hip2", [&](auto const& v){ cruse_bias_gain_hip2_ = std::max(0.0f, float(get_double(v, "cruse_bias_gain_hip2"))); });
    apply_param(params, "rhythm_inject_gain", [&](auto const& v){ rhythm_inject_gain_ = std::max(0.0f, float(get_double(v, "rhythm_inject_gain"))); });
    apply_param(params, "joints_topic",            [&](auto const& v){ joints_topic_            = get_string(v, "joints_topic"); });
    apply_param(params, "saturation_gate_enabled", [&](auto const& v){ saturation_gate_enabled_ = get_bool(v, "saturation_gate_enabled"); });
    apply_param(params, "saturation_zone_min",     [&](auto const& v){ saturation_zone_min_     = float(get_double(v, "saturation_zone_min")); });
    apply_param(params, "saturation_zone_max",     [&](auto const& v){ saturation_zone_max_     = float(get_double(v, "saturation_zone_max")); });
    apply_param(params, "violation_ema_alpha",  [&](auto const& v){ violation_ema_alpha_  = std::clamp(float(get_double(v, "violation_ema_alpha")), 0.0f, 1.0f); });
    apply_param(params, "max_violation_ema",    [&](auto const& v){ max_violation_ema_    = std::max(0.0f, float(get_double(v, "max_violation_ema"))); });
    apply_param(params, "hysteresis_low_frac",  [&](auto const& v){ hysteresis_low_frac_  = std::clamp(float(get_double(v, "hysteresis_low_frac")), 0.0f, 1.0f); });
    apply_param(params, "hysteresis_high_frac", [&](auto const& v){ hysteresis_high_frac_ = std::clamp(float(get_double(v, "hysteresis_high_frac")), 0.0f, 1.0f); });
    apply_param(params, "hysteresis_ema_alpha", [&](auto const& v){ hysteresis_ema_alpha_ = std::clamp(float(get_double(v, "hysteresis_ema_alpha")), 0.0f, 1.0f); });
    apply_param(params, "publish_when_silent",  [&](auto const& v){ publish_when_silent_  = get_bool(v, "publish_when_silent"); });
    apply_param(params, "adaptive_magnitude",   [&](auto const& v){ adaptive_magnitude_   = get_bool(v, "adaptive_magnitude"); });
    apply_param(params, "enable_rule_1",        [&](auto const& v){ enable_rule_1_        = get_bool(v, "enable_rule_1"); });
    apply_param(params, "enable_rule_2",        [&](auto const& v){ enable_rule_2_        = get_bool(v, "enable_rule_2"); });
    apply_param(params, "enable_rule_3",        [&](auto const& v){ enable_rule_3_        = get_bool(v, "enable_rule_3"); });
    apply_param(params, "rule2_window_ticks",   [&](auto const& v){ rule2_window_ticks_   = std::max(1, int(get_int(v, "rule2_window_ticks"))); });
    apply_param(params, "rule2_window_fraction",[&](auto const& v){ rule2_window_fraction_= std::max(0.01f, float(get_double(v, "rule2_window_fraction"))); });
    apply_param(params, "rule2_min_swing_ticks",[&](auto const& v){ rule2_min_swing_ticks_= std::max(0, int(get_int(v, "rule2_min_swing_ticks"))); });
    apply_param(params, "rule1_violation_boost",[&](auto const& v){ rule1_violation_boost_= std::max(0.0f, float(get_double(v, "rule1_violation_boost"))); });
    apply_param(params, "rule3_weight",         [&](auto const& v){ rule3_weight_         = std::max(0.0f, float(get_double(v, "rule3_weight"))); });
    apply_param(params, "coupling_mode",        [&](auto const& v){ coupling_mode_        = get_string(v, "coupling_mode"); });
    apply_param(params, "body_state_topic",     [&](auto const& v){ body_state_topic_     = get_string(v, "body_state_topic"); });
    apply_param(params, "body_state_min_threshold",[&](auto const& v){ body_state_min_threshold_ = std::clamp(float(get_double(v, "body_state_min_threshold")), 0.0f, 1.0f); });
    apply_param(params, "warmup_ticks",         [&](auto const& v){ warmup_ticks_         = std::max(0, int(get_int(v, "warmup_ticks"))); });
    apply_param(params, "enable_rule_6",        [&](auto const& v){ enable_rule_6_        = get_bool(v, "enable_rule_6"); });
    apply_param(params, "rule6_ema_alpha",      [&](auto const& v){ rule6_ema_alpha_      = std::clamp(float(get_double(v, "rule6_ema_alpha")), 0.0f, 1.0f); });
    apply_param(params, "rule6_max_swing_ratio",[&](auto const& v){ rule6_max_swing_ratio_= std::max(0.0f, float(get_double(v, "rule6_max_swing_ratio"))); });
    apply_param(params, "rule6_max_stance_ratio",[&](auto const& v){ rule6_max_stance_ratio_= std::max(0.0f, float(get_double(v, "rule6_max_stance_ratio"))); });
    apply_param(params, "rule6_max_swing_abs",  [&](auto const& v){ rule6_max_swing_abs_  = std::max(1, int(get_int(v, "rule6_max_swing_abs"))); });
    apply_param(params, "rule6_max_stance_abs", [&](auto const& v){ rule6_max_stance_abs_ = std::max(1, int(get_int(v, "rule6_max_stance_abs"))); });
    apply_param(params, "rule6_min_samples",    [&](auto const& v){ rule6_min_samples_    = std::max(1, int(get_int(v, "rule6_min_samples"))); });
    apply_param(params, "publish_phase_signals",[&](auto const& v){ publish_phase_signals_= get_bool(v, "publish_phase_signals"); });
    auto pot_it = params.find("phase_output_topics");
    if (pot_it != params.end()) {
        phase_output_topics_ = get_string_vec(pot_it->second, "phase_output_topics");
    }
    apply_param(params, "publish_bucket_signals",[&](auto const& v){ publish_bucket_signals_= get_bool(v, "publish_bucket_signals"); });
    auto bot_it = params.find("bucket_output_topics");
    if (bot_it != params.end()) {
        bucket_output_topics_ = get_string_vec(bot_it->second, "bucket_output_topics");
    }
    apply_param(params, "phase_mode",           [&](auto const& v){ phase_mode_           = get_string(v, "phase_mode"); });
    apply_param(params, "entrainment_strength", [&](auto const& v){ entrainment_strength_ = std::clamp(float(get_double(v, "entrainment_strength")), 0.0f, 1.0f); });
    apply_param(params, "default_omega",        [&](auto const& v){ default_omega_        = std::max(0.0f, float(get_double(v, "default_omega"))); });
    apply_param(params, "phase_blend_window",   [&](auto const& v){ phase_blend_window_   = std::max(1.0f, float(get_double(v, "phase_blend_window"))); });

    // ---- Leg state ----
    std::vector<std::string> leg_names = {"fl", "fr", "rl", "rr"};
    auto ln_it = params.find("leg_names");
    if (ln_it != params.end()) {
        leg_names = get_string_vec(ln_it->second, "leg_names");
    }
    leg_state_.clear();
    leg_state_.resize(leg_names.size());
    for (size_t i = 0; i < leg_names.size(); ++i) {
        leg_state_[i].name = leg_names[i];
    }

    // ---- Anatomical anterior wiring ----
    std::vector<std::string> anterior_names(leg_names.size(), "");
    auto aa_it = params.find("anatomical_anterior");
    if (aa_it != params.end()) {
        anterior_names = get_string_vec(aa_it->second, "anatomical_anterior");
        if (anterior_names.size() != leg_names.size()) {
            throw std::invalid_argument("CruseCoordinator: anatomical_anterior length must match leg_names length");
        }
    } else {
        // Picrawler default: front legs have no anterior, rear legs follow ipsilateral front.
        if (leg_names.size() == 4
            && leg_names[0] == "fl" && leg_names[1] == "fr"
            && leg_names[2] == "rl" && leg_names[3] == "rr") {
            anterior_names = {"", "", "fl", "fr"};
        }
    }
    for (size_t i = 0; i < leg_state_.size(); ++i) {
        if (anterior_names[i].empty()) {
            leg_state_[i].anterior_idx = -1;
        } else {
            int idx = -1;
            for (int j = 0; j < int(leg_state_.size()); ++j) {
                if (leg_state_[j].name == anterior_names[i]) { idx = j; break; }
            }
            if (idx < 0)
                throw std::invalid_argument("CruseCoordinator: anatomical_anterior[" + std::to_string(i)
                                             + "] = '" + anterior_names[i] + "' not in leg_names");
            leg_state_[i].anterior_idx = idx;
        }
    }

    // ---- Coupled-neighbor wiring (Phase 7.13 v3 symmetric_dwell) ----
    // Parse optional `coupled_neighbors` param: array of comma-separated strings.
    // Default for picrawler (non-diagonal, supports trot): each leg coupled to its
    // lateral pair + its ipsilateral fore/aft neighbor; diagonal pair NOT coupled.
    //   FL: {FR, RL}    FR: {FL, RR}    RL: {RR, FL}    RR: {RL, FR}
    std::vector<std::string> cn_csv(leg_names.size(), "");
    auto cn_it = params.find("coupled_neighbors");
    if (cn_it != params.end()) {
        cn_csv = get_string_vec(cn_it->second, "coupled_neighbors");
        if (cn_csv.size() != leg_names.size()) {
            throw std::invalid_argument("CruseCoordinator: coupled_neighbors length must match leg_names");
        }
    } else if (leg_names.size() == 4
        && leg_names[0] == "fl" && leg_names[1] == "fr"
        && leg_names[2] == "rl" && leg_names[3] == "rr") {
        cn_csv = {"fr,rl", "fl,rr", "rr,fl", "rl,fr"};
    }
    auto split_csv = [](std::string const& s) -> std::vector<std::string> {
        std::vector<std::string> out;
        std::string cur;
        for (char c : s) {
            if (c == ',' || c == ' ') {
                if (!cur.empty()) { out.push_back(cur); cur.clear(); }
            } else { cur.push_back(c); }
        }
        if (!cur.empty()) out.push_back(cur);
        return out;
    };
    for (size_t i = 0; i < leg_state_.size(); ++i) {
        leg_state_[i].coupled_neighbors.clear();
        for (auto const& nm : split_csv(cn_csv[i])) {
            int idx = -1;
            for (int j = 0; j < int(leg_state_.size()); ++j) {
                if (leg_state_[j].name == nm) { idx = j; break; }
            }
            if (idx < 0)
                throw std::invalid_argument("CruseCoordinator: coupled_neighbors[" + std::to_string(i)
                                             + "] = '" + nm + "' not in leg_names");
            leg_state_[i].coupled_neighbors.push_back(idx);
        }
    }

    // ---- Contralateral wiring (Phase 7.13 Rule 3) ----
    std::vector<std::string> contralateral_names(leg_names.size(), "");
    auto cl_it = params.find("contralateral");
    if (cl_it != params.end()) {
        contralateral_names = get_string_vec(cl_it->second, "contralateral");
        if (contralateral_names.size() != leg_names.size()) {
            throw std::invalid_argument("CruseCoordinator: contralateral length must match leg_names length");
        }
    } else {
        // Picrawler default: FL↔FR, RL↔RR.
        if (leg_names.size() == 4
            && leg_names[0] == "fl" && leg_names[1] == "fr"
            && leg_names[2] == "rl" && leg_names[3] == "rr") {
            contralateral_names = {"fr", "fl", "rr", "rl"};
        }
    }
    for (size_t i = 0; i < leg_state_.size(); ++i) {
        if (contralateral_names[i].empty()) {
            leg_state_[i].contralateral_idx = -1;
        } else {
            int idx = -1;
            for (int j = 0; j < int(leg_state_.size()); ++j) {
                if (leg_state_[j].name == contralateral_names[i]) { idx = j; break; }
            }
            if (idx < 0)
                throw std::invalid_argument("CruseCoordinator: contralateral[" + std::to_string(i)
                                             + "] = '" + contralateral_names[i] + "' not in leg_names");
            leg_state_[i].contralateral_idx = idx;
        }
    }

    // ---- Premotor state ----
    auto need = [&](char const* k) -> ParamValue const& {
        auto it = params.find(k);
        if (it == params.end())
            throw std::invalid_argument(std::string("CruseCoordinator: required param '") + k + "' missing");
        return it->second;
    };
    auto premotor_ids       = get_string_vec(need("premotor_ids"),            "premotor_ids");
    auto premotor_legs      = get_string_vec(need("premotor_leg_assignment"), "premotor_leg_assignment");
    auto premotor_joints    = get_string_vec(need("premotor_joint_kind"),     "premotor_joint_kind");
    auto premotor_topics    = get_string_vec(need("premotor_policy_topics"),  "premotor_policy_topics");
    // Optional per-Premotor stance_sign array (parallel to premotor_ids).
    // Default = all 1.0 (legacy behavior — Cruse pushes all legs toward +accel
    // for stance).  Set to -1.0 for Premotors whose body-geometry convention
    // is "negative accel = stance direction" (e.g. legs with mirrored joint
    // axes).  Without per-leg sign, Cruse's bias pushes half the legs in the
    // wrong direction → diagonal-split attractors observed in V9c UI.
    std::vector<float> premotor_stance_signs(premotor_ids.size(), 1.0f);
    auto pss_it = params.find("premotor_stance_sign");
    if (pss_it != params.end()) {
        auto const* pv = std::get_if<std::vector<double>>(&pss_it->second);
        if (!pv) {
            throw std::invalid_argument(
                "CruseCoordinator: premotor_stance_sign must be a double array");
        }
        if (pv->size() != premotor_ids.size()) {
            throw std::invalid_argument(
                "CruseCoordinator: premotor_stance_sign length ("
                + std::to_string(pv->size())
                + ") must equal premotor_ids length ("
                + std::to_string(premotor_ids.size()) + ")");
        }
        premotor_stance_signs.clear();
        premotor_stance_signs.reserve(pv->size());
        for (double v : *pv) premotor_stance_signs.push_back(float(v));
    }
    if (premotor_ids.size() != premotor_legs.size()
        || premotor_ids.size() != premotor_joints.size()
        || premotor_ids.size() != premotor_topics.size()) {
        throw std::invalid_argument("CruseCoordinator: premotor_ids, premotor_leg_assignment, premotor_joint_kind, premotor_policy_topics must all have same length");
    }

    // 2026-06-09 Move 5 — optional per-Premotor joint-position index (parallel
    // to premotor_ids) into the joints_topic 12-channel vector.  Stored as
    // vector<double> in ParamValue (matches the existing premotor_stance_sign
    // pattern at line ~486 — ParamValue variant doesn't carry vector<int>).
    // When absent, saturation gate is skipped (joint_position_index stays -1).
    std::vector<int> premotor_joint_position_indices;
    auto jpi_it = params.find("premotor_joint_position_indices");
    if (jpi_it != params.end()) {
        auto const* iv = std::get_if<std::vector<double>>(&jpi_it->second);
        if (!iv) {
            throw std::invalid_argument(
                "CruseCoordinator: premotor_joint_position_indices must be a numeric array");
        }
        if (iv->size() != premotor_ids.size()) {
            throw std::invalid_argument(
                "CruseCoordinator: premotor_joint_position_indices length ("
                + std::to_string(iv->size())
                + ") must equal premotor_ids length ("
                + std::to_string(premotor_ids.size()) + ")");
        }
        premotor_joint_position_indices.reserve(iv->size());
        for (double v : *iv) premotor_joint_position_indices.push_back(int(v));
    }

    premotor_state_.clear();
    premotor_state_.resize(premotor_ids.size());
    for (size_t i = 0; i < premotor_ids.size(); ++i) {
        auto& pm = premotor_state_[i];
        pm.id           = premotor_ids[i];
        pm.policy_topic = premotor_topics[i];
        pm.joint_kind   = premotor_joints[i];
        pm.stance_sign  = (premotor_stance_signs[i] >= 0.0f) ? +1.0f : -1.0f;
        pm.joint_position_index = premotor_joint_position_indices.empty()
                                  ? -1
                                  : premotor_joint_position_indices[i];
        int leg_idx = -1;
        for (int j = 0; j < int(leg_state_.size()); ++j) {
            if (leg_state_[j].name == premotor_legs[i]) { leg_idx = j; break; }
        }
        if (leg_idx < 0)
            throw std::invalid_argument("CruseCoordinator: premotor_leg_assignment[" + std::to_string(i)
                                         + "] = '" + premotor_legs[i] + "' not in leg_names");
        pm.leg_idx = leg_idx;
        pm.last_bias_published = Eigen::VectorXf::Zero(n_intents_);
    }

    // ---- Subscriptions ----
    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(feet_y_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_feet_y(p); }));
    if (!body_state_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(body_state_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ this->handle_body_state(p); }));
    }
    // 2026-06-09 Move 5 — subscribe to joints topic so the saturation gate
    // can read current joint positions.  Always subscribe; gate is opted-in
    // by saturation_gate_enabled (and skipped per-Premotor when index = -1).
    if (!joints_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(joints_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ this->handle_joints(p); }));
    }
    for (int i = 0; i < int(premotor_state_.size()); ++i) {
        int idx = i;
        sub_ids_.push_back(bus_->subscribe(premotor_state_[i].policy_topic,
            SubscriptionKind::Direct,
            [this, idx](std::string_view, MessagePtr p){ this->handle_policy(idx, p); }));
    }
}

void CruseCoordinator::handle_feet_y(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p) return;
    if (int(p->values.size()) != int(leg_state_.size())) return;
    for (int i = 0; i < int(leg_state_.size()); ++i) {
        update_leg(i, float(p->values[i]));
    }
}

// 2026-06-09 Move 5 — store current joint positions for saturation gate.
// Body publishes a 12-channel vector normalised to [-1,+1] (hip1 × 4, then
// hip2 × 4, then knee × 4).  We mirror the entire vector; per-Premotor lookups
// use joint_position_index directly into this buffer.
void CruseCoordinator::handle_joints(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p) return;
    if (p->values.size() <= 0) return;
    current_joint_positions_ = p->values;
}

void CruseCoordinator::handle_body_state(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p) return;
    if (p->values.size() < 1) return;
    body_state_value_ = std::clamp(float(p->values[0]), 0.0f, 1.0f);
    body_state_seen_  = true;
}

void CruseCoordinator::handle_policy(int premotor_idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pol = std::dynamic_pointer_cast<const PolicyToken>(payload);
    if (!pol) return;
    if (premotor_idx < 0 || premotor_idx >= int(premotor_state_.size())) return;
    auto& pm = premotor_state_[premotor_idx];
    if (int(pol->intent_accels.size()) == n_intents_) {
        pm.intent_accels = Eigen::Map<const Eigen::VectorXf>(
            pol->intent_accels.data(), n_intents_);
        pm.intent_accels_known = true;
    }
}

void CruseCoordinator::update_leg(int leg_idx, float foot_y) {
    auto& L = leg_state_[leg_idx];
    if (!L.thresholds_initialised) {
        L.feet_y_low_ema  = foot_y;
        L.feet_y_high_ema = foot_y;
        L.thresholds_initialised = true;
    } else {
        if (L.is_planted) {
            L.feet_y_low_ema = (1.0f - hysteresis_ema_alpha_) * L.feet_y_low_ema
                              + hysteresis_ema_alpha_ * foot_y;
        } else {
            L.feet_y_high_ema = (1.0f - hysteresis_ema_alpha_) * L.feet_y_high_ema
                               + hysteresis_ema_alpha_ * foot_y;
        }
    }
    float span = std::max(0.005f, L.feet_y_high_ema - L.feet_y_low_ema);
    float plant_threshold = L.feet_y_low_ema + hysteresis_low_frac_  * span;
    float lift_threshold  = L.feet_y_low_ema + hysteresis_high_frac_ * span;
    bool prev_planted = L.is_planted;
    if (!L.is_planted && foot_y <= plant_threshold) {
        L.is_planted = true;
    } else if (L.is_planted && foot_y >= lift_threshold) {
        L.is_planted = false;
    }
    // Phase 7.19 — entrained free-running phase advance.  Runs EVERY tick so the
    // rhythm signal is continuous; transitions below apply coupling pulses.
    if (!L.phi_free_initialised) {
        L.omega = default_omega_;
        L.phi_free_initialised = true;
    }
    L.phi_free += L.omega;
    static constexpr float kTwoPi = 6.28318530717958647692f;
    while (L.phi_free >= kTwoPi) L.phi_free -= kTwoPi;
    while (L.phi_free < 0.0f)    L.phi_free += kTwoPi;

    // Track touchdown / liftoff transitions for Rule 2 release window + stance_start_tick
    // for symmetric_dwell successor selection.  On each completed phase, update Rule 6
    // duration EMAs.
    if (L.is_planted != prev_planted) {
        if (L.is_planted) {
            // Swing → stance.  Update swing_duration_ema with completed swing duration.
            // FIRST sample REPLACES the EMA outright (rather than weighted update from a
            // hardcoded initial value of 60).  This lets the EMA reflect the body's actual
            // rhythm after just one cycle, supporting slow gaits (chameleon ~5s swing,
            // walking stick longer) without imposing a fast initial bias.  Subsequent
            // samples use weighted EMA as usual.
            if (L.last_liftoff_tick >= 0) {
                int64_t swing_dur = int64_t(current_tick_) - L.last_liftoff_tick;
                if (swing_dur > 0 && swing_dur < int64_t(1e6)) {
                    if (L.n_swing_samples == 0) {
                        L.swing_duration_ema = float(swing_dur);
                    } else {
                        L.swing_duration_ema = (1.0f - rule6_ema_alpha_) * L.swing_duration_ema
                                              + rule6_ema_alpha_ * float(swing_dur);
                    }
                    ++L.n_swing_samples;
                }
            }
            L.last_touchdown_tick = int64_t(current_tick_);
            L.stance_start_tick   = int64_t(current_tick_);
            // Phase 7.19 entrainment — touchdown pulls phi_free toward 0 (stance start).
            {
                float diff = 0.0f - L.phi_free;
                while (diff >  3.14159265358979323846f) diff -= kTwoPi;
                while (diff < -3.14159265358979323846f) diff += kTwoPi;
                L.phi_free += entrainment_strength_ * diff;
                while (L.phi_free >= kTwoPi) L.phi_free -= kTwoPi;
                while (L.phi_free < 0.0f)    L.phi_free += kTwoPi;
            }
            // Adapt omega from observed full-cycle duration (swing_ema + stance_ema).
            float cycle = L.swing_duration_ema + L.stance_duration_ema;
            if (cycle > 1.0f && L.n_swing_samples >= 1 && L.n_stance_samples >= 1) {
                L.omega = kTwoPi / cycle;
            }
        } else {
            // Stance → swing.  Same first-sample-replacement pattern for stance_duration_ema.
            if (L.stance_start_tick >= 0) {
                int64_t stance_dur = int64_t(current_tick_) - L.stance_start_tick;
                if (stance_dur > 0 && stance_dur < int64_t(1e6)) {
                    if (L.n_stance_samples == 0) {
                        L.stance_duration_ema = float(stance_dur);
                    } else {
                        L.stance_duration_ema = (1.0f - rule6_ema_alpha_) * L.stance_duration_ema
                                               + rule6_ema_alpha_ * float(stance_dur);
                    }
                    ++L.n_stance_samples;
                }
            }
            L.last_liftoff_tick = int64_t(current_tick_);
            // Phase 7.19 entrainment — liftoff pulls phi_free toward π (swing start).
            {
                float diff = 3.14159265358979323846f - L.phi_free;
                while (diff >  3.14159265358979323846f) diff -= kTwoPi;
                while (diff < -3.14159265358979323846f) diff += kTwoPi;
                L.phi_free += entrainment_strength_ * diff;
                while (L.phi_free >= kTwoPi) L.phi_free -= kTwoPi;
                while (L.phi_free < 0.0f)    L.phi_free += kTwoPi;
            }
            float cycle = L.swing_duration_ema + L.stance_duration_ema;
            if (cycle > 1.0f && L.n_swing_samples >= 1 && L.n_stance_samples >= 1) {
                L.omega = kTwoPi / cycle;
            }
        }
    } else if (L.is_planted && L.stance_start_tick < 0) {
        // First-time init: leg started planted at tabula rasa, no transition fired.
        L.stance_start_tick = int64_t(current_tick_);
    }
}

void CruseCoordinator::compute_rule2_successors(uint64_t tick_id) {
    // symmetric_dwell mode pre-pass: for every leg that just-planted within the Rule 2
    // window, pick its SUCCESSOR = coupled_neighbor with the longest current stance
    // dwell (i.e., the leg most "due" to swing).  Multiple just-planted legs can flag
    // the same successor; that's fine (idempotent flag).
    for (auto& L : leg_state_) L.is_rule2_successor = false;
    for (auto const& X : leg_state_) {
        if (!X.is_planted) continue;
        if (X.last_touchdown_tick < 0) continue;
        // NEW: require the leg to have been genuinely in swing before this touchdown.
        // Filters chassis-dip glitch touchdowns where foot_y wobbled across stance_y_threshold
        // without the leg ever lifting.  Without this, Rule 2 fires on every body-recovery
        // touchdown, producing the diagonal U attractor (one diag presses, other reaches sky).
        if (rule2_min_swing_ticks_ > 0) {
            if (X.last_liftoff_tick < 0) continue;
            int64_t swing_dur = X.last_touchdown_tick - X.last_liftoff_tick;
            if (swing_dur < int64_t(rule2_min_swing_ticks_)) continue;
        }
        // Phase 7.13 v5 — Rule 2 window is rhythm-coupled: scales with the just-planted leg's
        // OWN swing_duration_ema once that EMA has been observed enough times.  Before bootstrap
        // (per-leg n_swing_samples < rule6_min_samples), fall back to the absolute rule2_window_ticks.
        int64_t effective_window = (X.n_swing_samples < rule6_min_samples_)
            ? int64_t(rule2_window_ticks_)
            : int64_t(std::max(1.0f, rule2_window_fraction_ * X.swing_duration_ema));
        if ((int64_t(tick_id) - X.last_touchdown_tick) > effective_window) continue;
        int successor_idx = -1;
        int64_t max_dwell = -1;
        for (int n : X.coupled_neighbors) {
            if (n < 0 || n >= int(leg_state_.size())) continue;
            auto const& N = leg_state_[n];
            if (!N.is_planted) continue;          // can't switch a swinging leg
            if (N.stance_start_tick < 0) continue;
            int64_t dwell = int64_t(tick_id) - N.stance_start_tick;
            if (dwell > max_dwell) {
                max_dwell = dwell;
                successor_idx = n;
            }
        }
        if (successor_idx >= 0) leg_state_[successor_idx].is_rule2_successor = true;
    }
}

void CruseCoordinator::publish_biases(uint64_t tick_id) {
    for (auto& pm : premotor_state_) {
        Eigen::VectorXf bias = Eigen::VectorXf::Zero(n_intents_);
        // 2026-06-08 Move 2 — hip1 (yaw/swing) joined the eligible set.  hip1
        // bias is independently gain-gated by cruse_bias_gain_hip1_ (default
        // 0.0 = legacy no-bias behavior); curriculum opts in by setting the
        // gain >0 during walking stages.  Per-leg direction via stance_sign.
        bool eligible = (pm.joint_kind == "hip1" || pm.joint_kind == "hip2" || pm.joint_kind == "knee");

        // Per-rule contribution accumulator.
        // Positive factor = bias toward stance (extend direction).
        // Negative factor = bias toward swing (flex direction).
        float rule1_factor = 0.0f;
        float rule2_factor = 0.0f;
        float rule3_factor = 0.0f;
        float rule6_factor = 0.0f;
        bool  rule1_active = false;
        bool  rule2_active = false;
        bool  rule3_active = false;
        bool  rule6_active = false;
        bool  rule1_violated_now = false;

        if (eligible && pm.leg_idx >= 0 && pm.leg_idx < int(leg_state_.size())) {
            auto const& L = leg_state_[pm.leg_idx];

            // ---- Rule 1: depends on coupling_mode ----
            //   chain:           ipsilateral anterior in swing → STANCE bias.
            //   symmetric_dwell: ANY coupled_neighbor in swing → STANCE bias.
            if (enable_rule_1_) {
                bool any_neighbor_swinging = false;
                if (coupling_mode_ == "symmetric_dwell") {
                    for (int n : L.coupled_neighbors) {
                        if (n < 0 || n >= int(leg_state_.size())) continue;
                        if (!leg_state_[n].is_planted) {
                            any_neighbor_swinging = true;
                            break;
                        }
                    }
                } else if (L.anterior_idx >= 0 && L.anterior_idx < int(leg_state_.size())) {
                    any_neighbor_swinging = !leg_state_[L.anterior_idx].is_planted;
                }
                if (any_neighbor_swinging) {
                    rule1_active = true;
                    rule1_violated_now = !L.is_planted;
                    rule1_factor = +1.0f;
                    if (rule1_violated_now) rule1_factor *= rule1_violation_boost_;
                    if (rule1_violated_now) ++total_rule1_violations_;
                    else                    ++total_rule1_compliant_;
                    ++total_rule1_fires_;
                }
            }

            // ---- Rule 2: depends on coupling_mode ----
            //   chain:           ipsilateral anterior just-touched-down → SWING bias.
            //   symmetric_dwell: this leg is the SUCCESSOR (longest-stance coupled_neighbor
            //                    of any leg that just-planted within window) → SWING bias.
            if (enable_rule_2_ && L.is_planted) {
                bool rule2_should_fire = false;
                if (coupling_mode_ == "symmetric_dwell") {
                    rule2_should_fire = L.is_rule2_successor;
                } else if (L.anterior_idx >= 0 && L.anterior_idx < int(leg_state_.size())) {
                    auto const& A = leg_state_[L.anterior_idx];
                    // Same rhythm-coupled window as symmetric_dwell, but indexed by anterior's
                    // observed swing rhythm.
                    int64_t effective_window = (A.n_swing_samples < rule6_min_samples_)
                        ? int64_t(rule2_window_ticks_)
                        : int64_t(std::max(1.0f, rule2_window_fraction_ * A.swing_duration_ema));
                    rule2_should_fire = (A.is_planted
                        && A.last_touchdown_tick >= 0
                        && (int64_t(tick_id) - A.last_touchdown_tick) <= effective_window);
                }
                if (rule2_should_fire) {
                    rule2_active = true;
                    rule2_factor = -1.0f;   // toward swing
                    ++total_rule2_fires_;
                }
            }

            // ---- Rule 6: step-duration memory (self-bias) ----
            // Per-leg.  If a leg has been swinging much longer than its typical swing → bias to stance.
            // If a leg has been planted much longer than its typical stance → bias to swing.
            // Breaks single-leg attractors (e.g. FL waves indefinitely under v3 symmetric topology).
            if (enable_rule_6_) {
                if (!L.is_planted && L.last_liftoff_tick >= 0) {
                    int64_t cur_swing = int64_t(tick_id) - L.last_liftoff_tick;
                    float threshold = (L.n_swing_samples < rule6_min_samples_)
                                     ? float(rule6_max_swing_abs_)
                                     : rule6_max_swing_ratio_ * L.swing_duration_ema;
                    if (float(cur_swing) > threshold) {
                        rule6_active = true;
                        rule6_factor = +1.0f;  // toward stance
                        ++total_rule6_fires_;
                    }
                } else if (L.is_planted && L.stance_start_tick >= 0) {
                    int64_t cur_stance = int64_t(tick_id) - L.stance_start_tick;
                    float threshold = (L.n_stance_samples < rule6_min_samples_)
                                     ? float(rule6_max_stance_abs_)
                                     : rule6_max_stance_ratio_ * L.stance_duration_ema;
                    if (float(cur_stance) > threshold) {
                        rule6_active = true;
                        rule6_factor = -1.0f;  // toward swing
                        ++total_rule6_fires_;
                    }
                }
            }

            // ---- Rule 3: contralateral in SWING → strengthen own STANCE ----
            if (enable_rule_3_
                && L.contralateral_idx >= 0 && L.contralateral_idx < int(leg_state_.size())) {
                auto const& C = leg_state_[L.contralateral_idx];
                if (!C.is_planted) {
                    rule3_active = true;
                    rule3_factor = +rule3_weight_;
                    ++total_rule3_fires_;
                }
            }

            // ---- Violation EMA tracking (Rule 1 only, for both modes) ----
            if (rule1_active) {
                float target = rule1_violated_now ? 1.0f : 0.0f;
                pm.violation_ema = (1.0f - violation_ema_alpha_) * pm.violation_ema
                                  + violation_ema_alpha_ * target;
            } else {
                pm.violation_ema = (1.0f - violation_ema_alpha_) * pm.violation_ema;
            }
            pm.violation_ema = std::clamp(pm.violation_ema, 0.0f, max_violation_ema_);

            // ---- E2 rhythm injection: continuous phase-driven term ----
            // cos(phi_free): +1 at phi=0 (stance-press), -1 at phi=pi (swing-lift).
            // Always-on (when gain>0) so it can BOOTSTRAP a rhythm; phi_free
            // free-runs at default_omega without footfalls and entrains to them
            // when they occur (closed loop).
            float rhythm_factor = 0.0f;
            if (rhythm_inject_gain_ > 0.0f) {
                rhythm_factor = rhythm_inject_gain_ * std::cos(L.phi_free);
            }

            // ---- Compose bias ----
            float total_factor = rule1_factor + rule2_factor + rule3_factor + rule6_factor + rhythm_factor;
            // 2026-06-08 — per-joint-kind gain.  Knee defaults to 0.0 so knee
            // Premotors get a zero bias regardless of cruse_bias_gain_.  hip2
            // implicitly uses gain 1.0 (no per-joint multiplier).  Joseph's
            // call after V2 trace exposed knee_bias 0.4-0.5 = claw pose.
            float joint_gain = 1.0f;
            if      (pm.joint_kind == "knee") joint_gain = cruse_bias_gain_knee_;
            else if (pm.joint_kind == "hip1") joint_gain = cruse_bias_gain_hip1_;
            else if (pm.joint_kind == "hip2") joint_gain = cruse_bias_gain_hip2_;
            // (Move 4 default = 1.0 = legacy behavior; >1.0 amplifies
            //  hip2's down-during-stance + up-during-swing biases.)
            float effective_base_gain = cruse_bias_gain_ * joint_gain;
            if (effective_base_gain > 0.0f
                && std::abs(total_factor) > 1e-6f
                && pm.intent_accels_known
                && int(pm.intent_accels.size()) == n_intents_) {
                float magnitude = effective_base_gain;
                if (adaptive_magnitude_) {
                    // Phase 7.11 v1 mode — bias scales with violation_ema.
                    magnitude *= pm.violation_ema;
                }
                // Phase 7.13 v4.1 warmup ramp — quiet window for Premotors to learn standing
                // via REINFORCE before Cruse adds directional bias.
                if (warmup_ticks_ > 0 && int64_t(tick_id) < int64_t(warmup_ticks_)) {
                    float ramp = float(tick_id) / float(warmup_ticks_);
                    magnitude *= std::clamp(ramp, 0.0f, 1.0f);
                }
                // Phase 7.13 v4.2 body-state gate — coordinator goes silent when body is fallen.
                // Addresses defects exposed by v4.1 belly-stuck glitch:
                //   1. rules firing on a fallen body (no meaningful gait coordination possible)
                //   2. EMA contamination from pathological short stance/swing phases
                //   3. softmax saturation from constant stance bias when no resistance
                if (body_state_seen_) {
                    if (body_state_value_ < body_state_min_threshold_) {
                        magnitude = 0.0f;
                    } else {
                        magnitude *= body_state_value_;
                    }
                }
                // Direction encoding: bias[i] = magnitude * total_factor * stance_sign * (accel_i - mean) / max_abs.
                // Positive factor × positive (accel - mean) × +1 stance_sign → bias toward +max accel (stance for sign=+1).
                // Positive factor × positive (accel - mean) × -1 stance_sign → bias toward -max accel (stance for sign=-1).
                // Per-Premotor stance_sign lets Cruse address legs whose
                // body-geometry convention is "negative accel = stance" without
                // pushing them in the wrong direction (the diagonal-split
                // attractor observed in V9c UI).
                float mean = pm.intent_accels.mean();
                float max_abs = pm.intent_accels.cwiseAbs().maxCoeff();
                if (max_abs > 1e-6f) {
                    for (int i = 0; i < n_intents_; ++i) {
                        bias(i) = magnitude * total_factor * pm.stance_sign
                                * (pm.intent_accels(i) - mean) / max_abs;
                    }
                }
                // 2026-06-09 Move 5 — saturation gate.  Read current joint
                // position; compute productive_score = current_u ×
                // bias_direction_sign.  Suppress bias when outside
                // [zone_min, zone_max].  Joseph's rudder case: leg planted
                // on wrong side of perpendicular (productive_score < 0)
                // sweeps through perpendicular under stance bias, dragging
                // body sideways.  Saturation case: productive_score > 0.9
                // already at target, further bias overshoots.
                pm.last_productive_score = 0.0f;
                pm.saturation_suppressed = false;
                if (saturation_gate_enabled_
                    && pm.joint_position_index >= 0
                    && pm.joint_position_index < int(current_joint_positions_.size())) {
                    float current_u = current_joint_positions_[pm.joint_position_index];
                    float bias_dir_sign = (total_factor >= 0.0f ? +1.0f : -1.0f)
                                        * pm.stance_sign;
                    float productive_score = current_u * bias_dir_sign;
                    pm.last_productive_score = productive_score;
                    if (productive_score < saturation_zone_min_
                        || productive_score > saturation_zone_max_) {
                        bias.setZero();
                        pm.saturation_suppressed = true;
                    }
                }
            }
        }

        pm.last_bias_published = bias;
        if (!publish_when_silent_ && bias.norm() < 1e-9f) continue;
        auto msg = std::make_shared<RhythmBiasToken>();
        msg->tick_id     = tick_id;
        msg->producer_id = id_.empty() ? std::string("cruse_coordinator") : id_;
        msg->bias        = bias;
        msg->confidence  = pm.violation_ema;
        // phase_bin: packed flags bit 0=rule1, 1=rule2, 2=rule3, 3=rule6 — for telemetry.
        int phase_flags = 0;
        if (rule1_active) phase_flags |= 1;
        if (rule2_active) phase_flags |= 2;
        if (rule3_active) phase_flags |= 4;
        if (rule6_active) phase_flags |= 8;
        msg->phase_bin   = phase_flags;
        bus_->publish(std::string(topics::kRhythmBiasPrefix) + pm.id, msg);
    }
}

void CruseCoordinator::tick(uint64_t tick_id) {
    current_tick_ = tick_id;
    if (coupling_mode_ == "symmetric_dwell") {
        compute_rule2_successors(tick_id);
    }
    publish_biases(tick_id);
    if (publish_phase_signals_) {
        emit_phase_signals(tick_id);
    }
    if (publish_bucket_signals_) {
        emit_bucket_signals(tick_id);
    }
}

void CruseCoordinator::emit_phase_signals(uint64_t tick_id) {
    // Phase 7.17 — closed-loop rhythm output.  Per leg, compute current phase
    // in [0, 2π] from observed touchdown/liftoff events and EMA durations:
    //   stance phase [0, π]: progress through current planted period
    //   swing  phase [π, 2π]: progress through current airborne period
    // Use EMA duration once available (n_samples >= rule6_min_samples), else
    // fall back to half of absolute cap as a reasonable mid-range estimate.
    // Publishes [cos(phi), sin(phi)] ProprioToken — drop-in replacement for
    // the CPGOscillator perceptual output that rhythm EPMs consume.
    int N = int(leg_state_.size());
    if (int(phase_output_topics_.size()) < N) return;
    static constexpr float kPi = 3.14159265358979323846f;
    for (int i = 0; i < N; ++i) {
        auto const& L = leg_state_[i];

        // ---- Compute state_event phase (variance-preserving body-driven) ----
        float phi_state = 0.0f;
        if (L.is_planted) {
            float dur = (L.n_stance_samples >= rule6_min_samples_)
                      ? L.stance_duration_ema
                      : float(rule6_max_stance_abs_) * 0.5f;
            if (L.last_touchdown_tick >= 0 && dur > 1.0f) {
                float t_in_stance = float(int64_t(tick_id) - L.last_touchdown_tick);
                phi_state = kPi * std::clamp(t_in_stance / dur, 0.0f, 1.0f);
            }
        } else {
            float dur = (L.n_swing_samples >= rule6_min_samples_)
                      ? L.swing_duration_ema
                      : float(rule6_max_swing_abs_) * 0.5f;
            if (L.last_liftoff_tick >= 0 && dur > 1.0f) {
                float t_in_swing = float(int64_t(tick_id) - L.last_liftoff_tick);
                phi_state = kPi + kPi * std::clamp(t_in_swing / dur, 0.0f, 1.0f);
            } else {
                phi_state = kPi;
            }
        }

        // ---- Compute published signal based on mode ----
        float cos_out = 0.0f, sin_out = 0.0f;
        if (phase_mode_ == "entrained_free") {
            cos_out = std::cos(L.phi_free);
            sin_out = std::sin(L.phi_free);
        } else if (phase_mode_ == "smooth_fallback") {
            // Phase 7.20 — state_event is the primary phase (preserves natural
            // period variation cycle-to-cycle).  phi_free smoothly fades in only
            // when this leg has had no transition for phase_blend_window ticks
            // (e.g. body stalled).  When transitions resume, mix snaps back to
            // state_event.  Best of both: v3-like variance + no catastrophic
            // saturation.
            int64_t last_event = std::max(L.last_touchdown_tick, L.last_liftoff_tick);
            float ticks_since = (last_event >= 0)
                ? float(int64_t(tick_id) - last_event)
                : float(phase_blend_window_);  // before first event: fully on free
            float mix = std::clamp(ticks_since / phase_blend_window_, 0.0f, 1.0f);
            float cos_s = std::cos(phi_state), sin_s = std::sin(phi_state);
            float cos_f = std::cos(L.phi_free), sin_f = std::sin(L.phi_free);
            cos_out = (1.0f - mix) * cos_s + mix * cos_f;
            sin_out = (1.0f - mix) * sin_s + mix * sin_f;
        } else {
            // state_event (legacy)
            cos_out = std::cos(phi_state);
            sin_out = std::sin(phi_state);
        }

        auto pt = std::make_shared<ProprioToken>();
        pt->tick_id     = tick_id;
        pt->producer_id = id_.empty() ? std::string("cruse_coordinator") : id_;
        pt->sensor      = "cruse_phase";
        pt->values.resize(2);
        pt->values(0) = cos_out;
        pt->values(1) = sin_out;
        bus_->publish(phase_output_topics_[i], pt);
    }
}

void CruseCoordinator::emit_bucket_signals(uint64_t tick_id) {
    // 2026-05-29 gait-bucket bet — publish each leg's discrete bucket (0=swing,
    // 1=stance) as a single-float ProprioToken every tick.  The state machinery
    // (is_planted, driven by touchdown/liftoff + Cruse rules) already exists in
    // leg_state_; this just exposes it so Premotors can condition policy
    // per-bucket via a learned bucket_bias term.
    int N = int(leg_state_.size());
    if (int(bucket_output_topics_.size()) < N) return;
    for (int i = 0; i < N; ++i) {
        auto pt = std::make_shared<ProprioToken>();
        pt->tick_id     = tick_id;
        pt->producer_id = id_.empty() ? std::string("cruse_coordinator") : id_;
        pt->sensor      = "cruse_bucket";
        pt->values.resize(1);
        pt->values(0) = leg_state_[i].is_planted ? 1.0f : 0.0f;
        bus_->publish(bucket_output_topics_[i], pt);
    }
}

void CruseCoordinator::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "cruse_bias_gain")        cruse_bias_gain_        = std::max(0.0f, float(get_double(value, k)));
    else if (k == "cruse_bias_gain_knee")   cruse_bias_gain_knee_   = std::max(0.0f, float(get_double(value, k)));
    else if (k == "cruse_bias_gain_hip1")   cruse_bias_gain_hip1_   = std::max(0.0f, float(get_double(value, k)));
    else if (k == "cruse_bias_gain_hip2")   cruse_bias_gain_hip2_   = std::max(0.0f, float(get_double(value, k)));
    else if (k == "rhythm_inject_gain")     rhythm_inject_gain_     = std::max(0.0f, float(get_double(value, k)));
    else if (k == "saturation_gate_enabled") saturation_gate_enabled_ = get_bool(value, k);
    else if (k == "saturation_zone_min")    saturation_zone_min_    = float(get_double(value, k));
    else if (k == "saturation_zone_max")    saturation_zone_max_    = float(get_double(value, k));
    else if (k == "violation_ema_alpha")    violation_ema_alpha_    = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "max_violation_ema")      max_violation_ema_      = std::max(0.0f, float(get_double(value, k)));
    else if (k == "hysteresis_low_frac")    hysteresis_low_frac_    = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "hysteresis_high_frac")   hysteresis_high_frac_   = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "hysteresis_ema_alpha")   hysteresis_ema_alpha_   = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "publish_when_silent")    publish_when_silent_    = get_bool(value, k);
    else if (k == "adaptive_magnitude")     adaptive_magnitude_     = get_bool(value, k);
    else if (k == "enable_rule_1")          enable_rule_1_          = get_bool(value, k);
    else if (k == "enable_rule_2")          enable_rule_2_          = get_bool(value, k);
    else if (k == "enable_rule_3")          enable_rule_3_          = get_bool(value, k);
    else if (k == "rule2_window_ticks")     rule2_window_ticks_     = std::max(1, int(get_int(value, k)));
    else if (k == "rule2_window_fraction")  rule2_window_fraction_  = std::max(0.01f, float(get_double(value, k)));
    else if (k == "rule1_violation_boost")  rule1_violation_boost_  = std::max(0.0f, float(get_double(value, k)));
    else if (k == "rule3_weight")           rule3_weight_           = std::max(0.0f, float(get_double(value, k)));
    else if (k == "body_state_min_threshold") body_state_min_threshold_ = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "warmup_ticks")           warmup_ticks_           = std::max(0, int(get_int(value, k)));
    else if (k == "enable_rule_6")          enable_rule_6_          = get_bool(value, k);
    else if (k == "rule6_ema_alpha")        rule6_ema_alpha_        = std::clamp(float(get_double(value, k)), 0.0f, 1.0f);
    else if (k == "rule6_max_swing_ratio")  rule6_max_swing_ratio_  = std::max(0.0f, float(get_double(value, k)));
    else if (k == "rule6_max_stance_ratio") rule6_max_stance_ratio_ = std::max(0.0f, float(get_double(value, k)));
    else if (k == "rule6_max_swing_abs")    rule6_max_swing_abs_    = std::max(1, int(get_int(value, k)));
    else if (k == "rule6_max_stance_abs")   rule6_max_stance_abs_   = std::max(1, int(get_int(value, k)));
    else if (k == "rule6_min_samples")      rule6_min_samples_      = std::max(1, int(get_int(value, k)));
    else
        throw std::invalid_argument("CruseCoordinator: param '" + k + "' is ConstructionOnly");
}

std::vector<float> CruseCoordinator::violation_ema_all() const {
    std::vector<float> out;
    out.reserve(premotor_state_.size());
    for (auto const& pm : premotor_state_) out.push_back(pm.violation_ema);
    return out;
}
std::vector<bool> CruseCoordinator::is_planted_all() const {
    std::vector<bool> out;
    out.reserve(leg_state_.size());
    for (auto const& L : leg_state_) out.push_back(L.is_planted);
    return out;
}

nlohmann::json CruseCoordinator::snapshot_state() const {
    auto leg_arr = nlohmann::json::array();
    for (auto const& L : leg_state_) {
        leg_arr.push_back({
            {"name",                 L.name},
            {"anterior_idx",         L.anterior_idx},
            {"contralateral_idx",    L.contralateral_idx},
            {"coupled_neighbors",    L.coupled_neighbors},
            {"is_planted",           L.is_planted},
            {"foot_y_low_ema",       double(L.feet_y_low_ema)},
            {"foot_y_high_ema",      double(L.feet_y_high_ema)},
            {"last_touchdown_tick",  L.last_touchdown_tick},
            {"last_liftoff_tick",    L.last_liftoff_tick},
            {"stance_start_tick",    L.stance_start_tick},
            {"is_rule2_successor",   L.is_rule2_successor},
            {"swing_duration_ema",   double(L.swing_duration_ema)},
            {"stance_duration_ema",  double(L.stance_duration_ema)},
            {"n_swing_samples",      L.n_swing_samples},
            {"n_stance_samples",     L.n_stance_samples},
        });
    }
    auto pm_arr = nlohmann::json::array();
    for (auto const& pm : premotor_state_) {
        pm_arr.push_back({
            {"id",                       pm.id},
            {"leg_idx",                  pm.leg_idx},
            {"joint_kind",               pm.joint_kind},
            {"stance_sign",              double(pm.stance_sign)},
            {"intent_accels_known",      pm.intent_accels_known},
            {"violation_ema",            double(pm.violation_ema)},
            {"last_bias_norm",           double(pm.last_bias_published.norm())},
            // Move 5 — saturation gate telemetry
            {"joint_position_index",     pm.joint_position_index},
            {"last_productive_score",    double(pm.last_productive_score)},
            {"saturation_suppressed",    pm.saturation_suppressed},
        });
    }
    return nlohmann::json{
        {"version",                  3},
        {"coupling_mode",            coupling_mode_},
        {"body_state_topic",         body_state_topic_},
        {"body_state_value",         double(body_state_value_)},
        {"body_state_seen",          body_state_seen_},
        {"body_state_min_threshold", double(body_state_min_threshold_)},
        {"warmup_ticks",             warmup_ticks_},
        {"warmup_ramp_factor",       (warmup_ticks_ > 0 && int64_t(current_tick_) < int64_t(warmup_ticks_))
                                       ? double(current_tick_) / double(warmup_ticks_) : 1.0},
        {"cruse_bias_gain",          double(cruse_bias_gain_)},
        {"cruse_bias_gain_knee",     double(cruse_bias_gain_knee_)},
        {"cruse_bias_gain_hip1",     double(cruse_bias_gain_hip1_)},
        {"cruse_bias_gain_hip2",     double(cruse_bias_gain_hip2_)},
        {"joints_topic",             joints_topic_},
        {"saturation_gate_enabled",  saturation_gate_enabled_},
        {"saturation_zone_min",      double(saturation_zone_min_)},
        {"saturation_zone_max",      double(saturation_zone_max_)},
        {"adaptive_magnitude",       adaptive_magnitude_},
        {"enable_rule_1",            enable_rule_1_},
        {"enable_rule_2",            enable_rule_2_},
        {"enable_rule_3",            enable_rule_3_},
        {"rule2_window_ticks",       rule2_window_ticks_},
        {"rule1_violation_boost",    double(rule1_violation_boost_)},
        {"rule3_weight",             double(rule3_weight_)},
        {"total_rule1_violations",   total_rule1_violations_},
        {"total_rule1_compliant",    total_rule1_compliant_},
        {"total_rule1_fires",        total_rule1_fires_},
        {"total_rule2_fires",        total_rule2_fires_},
        {"total_rule3_fires",        total_rule3_fires_},
        {"total_rule6_fires",        total_rule6_fires_},
        {"enable_rule_6",            enable_rule_6_},
        {"rule6_max_swing_ratio",    double(rule6_max_swing_ratio_)},
        {"rule6_max_stance_ratio",   double(rule6_max_stance_ratio_)},
        {"legs",                     leg_arr},
        {"premotors",                pm_arr},
    };
}

} // namespace ogma
