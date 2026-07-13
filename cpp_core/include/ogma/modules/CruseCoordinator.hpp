#pragma once

// =============================================================================
// CruseCoordinator.hpp  --  Structural-prior inter-leg coordinator
//                          (Phase 7.11 v1 = Rule 1 only / adaptive;
//                           Phase 7.13 v2 = Rules 1+2+3 / constant gain by default)
// =============================================================================
//
// Hard-coded inter-leg coordination law derived from Holk Cruse's stick-insect
// Walknet rules (referenced from Hector hexapod control).  Addresses the
// substrate ceiling diagnosed in Phase 7.5-7.10: REINFORCE on per-servo accel
// intents cannot LEARN inter-leg coordination from scratch within 25-60 min,
// because the action space (5^12 per-tick joint actions) lacks the structural
// prior that real spinal cords inherit from evolution.
//
// Three rules implemented as pre-softmax logit biases:
//
//   Rule 1 (no-swing-overlap, preventative):
//       When this leg's anatomical anterior is in SWING, this leg should
//       remain in STANCE.  Bias direction: +1 (toward stance).
//
//   Rule 2 (release / constructive lift):
//       When this leg's anatomical anterior JUST transitioned to STANCE
//       (within `rule2_window_ticks` of its last touchdown) AND this leg is
//       currently in STANCE, this leg should INITIATE SWING.  Bias direction:
//       -1 (toward swing).  This is the constructive counterpart to Rule 1 —
//       without it the brain only receives stay-in-stance pushes.
//
//   Rule 3 (contralateral load tolerance):
//       When the contralateral neighbour (FL↔FR, RL↔RR) is in SWING, this leg
//       should commit harder to STANCE.  Bias direction: +0.5 (half weight).
//
// Rules 1 and 2 are temporally disjoint by construction (Rule 1 fires only
// while anterior is in swing, Rule 2 only after anterior has just planted),
// so they do not conflict.  Rule 3 composes additively with Rules 1 and 2.
//
// Authority model:
//   `adaptive_magnitude=true`  (Phase 7.11 v1 mode, retained for repro)
//       Bias scales with violation_ema — fades when brain is naturally
//       compliant.  Minimum-intervention design.  Falsified at n=5 × 25 min
//       in Phase 7.11.
//   `adaptive_magnitude=false` (Phase 7.13 default — Hector-faithful)
//       Bias fires at CONSTANT gain whenever its rule activation conditions
//       hold.  Rules are invariants — they don't fade.  Closer to the
//       Walknet formulation that Hector demonstrably walks under.
//
// Additionally, `rule1_violation_boost` (default 1.5) multiplies Rule 1's
// contribution while it is BOTH active AND violated — sharper authority on
// the rule that prevents tip-overs.
//
// Composes additively with W (REINFORCE), epistemic_gain, value_head_gain,
// rhythm_bias (SynergyTimer) — all in logit space pre-softmax.
//
// Affects only the joints whose accel direction encodes stance vs swing
// (hip2 + knee in picrawler; hip1 is yaw control, no stance/swing axis).
// Hip1 Premotors get zero bias.
//
// Composes additively with W (REINFORCE), epistemic_gain, value_head_gain,
// rhythm_bias (SynergyTimer) — all in logit space pre-softmax.
//
// Affects only the joints whose accel direction encodes stance vs swing
// (hip2 + knee in picrawler; hip1 is yaw control, no stance/swing axis).
// Hip1 Premotors get zero bias.
//
// Anatomical wiring is per-instance config:
//   premotor_leg_assignment   — which leg each Premotor belongs to
//   premotor_joint_kind       — "hip2" | "knee" | other (other = no bias)
//   anatomical_anterior       — map of leg → its anterior neighbor (or "" if none)
//   contralateral             — map of leg → its contralateral neighbor (or "" if none)

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json_fwd.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class CruseCoordinator : public Module {
public:
    CruseCoordinator();
    ~CruseCoordinator() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json snapshot_state() const override;

    int                       n_legs()       const { return int(leg_state_.size()); }
    int                       n_premotors()  const { return int(premotor_state_.size()); }
    float                     cruse_bias_gain() const { return cruse_bias_gain_; }
    std::vector<float>        violation_ema_all() const;
    std::vector<bool>         is_planted_all() const;
    int                       total_rule1_fires() const { return total_rule1_fires_; }
    int                       total_rule2_fires() const { return total_rule2_fires_; }
    int                       total_rule3_fires() const { return total_rule3_fires_; }

private:
    struct LegState {
        std::string name;
        int         anterior_idx          = -1;       // legacy chain topology: -1 = no anterior
        int         contralateral_idx     = -1;       // -1 = no contralateral
        std::vector<int> coupled_neighbors;            // Phase 7.13 v3 symmetric topology
        bool        is_planted            = false;
        bool        thresholds_initialised = false;
        float       feet_y_low_ema        = 0.0f;
        float       feet_y_high_ema       = 0.0f;
        int64_t     last_touchdown_tick   = -1;       // tick when is_planted false→true
        int64_t     last_liftoff_tick     = -1;       // tick when is_planted true→false
        int64_t     stance_start_tick     = -1;       // tick when current stance phase began
                                                       //   (== last_touchdown_tick when planted)
        bool        is_rule2_successor    = false;    // computed per-tick in symmetric_dwell mode
        // Phase 7.13 v4 — Rule 6 (step duration memory).
        float       swing_duration_ema    = 60.0f;    // EMA of swing durations (ticks).  Default ~1 sec @60Hz.
        float       stance_duration_ema   = 120.0f;   // EMA of stance durations.  Default ~2 sec.
        int         n_swing_samples       = 0;        // # of completed swings used for bootstrap gate
        int         n_stance_samples      = 0;        // # of completed stances used for bootstrap gate
        // Phase 7.19 — Kuramoto-lite entrained free-running rhythm.
        // phi_free advances every tick at omega rad/tick.  Body events (touchdown,
        // liftoff) nudge phi_free toward target phases (stance=0, swing=π) with
        // coupling strength K.  Omega adapts to (swing_ema + stance_ema) so the
        // oscillator's intrinsic frequency tracks the body's observed cadence.
        // Signal NEVER STOPS — when body falters, phi keeps advancing at last
        // learned omega; rhythm EPMs see continuous pattern, don't lose their
        // baked nodes.
        float       phi_free              = 0.0f;     // current free-running phase ∈ [0, 2π)
        float       omega                 = 0.0349f;  // rad/tick; default = 2π/180 (3s cycle @60Hz)
        bool        phi_free_initialised  = false;
    };

    struct PremotorState {
        std::string  id;
        int          leg_idx           = -1;
        // 2026-06-09 Move 5 — index into the 12-channel reality.proprio.joints
        // vector for this Premotor's joint angle.  Used by the saturation gate
        // to read the current joint position when computing productive_score.
        // -1 = unset (gate has no data → falls through to legacy unconditional
        // bias).  Populated from premotor_joint_position_indices config array.
        int          joint_position_index = -1;
        // Last published bias's productive_score = current_u × bias_direction_sign.
        // Exposed in snapshot for inspector visualization.
        float        last_productive_score = 0.0f;
        // Whether the saturation gate suppressed bias this tick (for inspector).
        bool         saturation_suppressed = false;
        std::string  joint_kind;        // "hip2"|"knee"|other → only first two get bias
        std::string  policy_topic;
        Eigen::VectorXf intent_accels;
        bool         intent_accels_known = false;
        float        violation_ema       = 0.0f;
        Eigen::VectorXf last_bias_published;
        // Per-Premotor stance sign — multiplies the bias direction.  +1 (legacy
        // default) means "positive accel = stance direction" (the brain's V7
        // chosen-intent showed front knees committed to +max for extend).  -1
        // FLIPS the bias for legs whose body-geometry convention is "negative
        // accel = stance direction" (V7 showed rear knees committed to -max
        // for extend).  Without per-leg sign, Cruse's uniform bias pushes
        // half the legs in the wrong direction, producing asymmetric attractors
        // (one diagonal pushes down, the other reaches skyward).
        float        stance_sign        = 1.0f;
    };

    void handle_feet_y(MessagePtr p);
    void handle_body_state(MessagePtr p);
    void handle_joints(MessagePtr p);   // Move 5 — updates current_joint_positions_
    void handle_policy(int premotor_idx, MessagePtr p);
    void emit_phase_signals(uint64_t tick_id);       // Phase 7.17 closed-loop rhythm output
    void emit_bucket_signals(uint64_t tick_id);      // 2026-05-29 per-leg bucket (swing/stance) for per-bucket Premotor specialization
    void update_leg(int leg_idx, float foot_y);
    void compute_rule2_successors(uint64_t tick_id);   // symmetric_dwell mode pre-pass
    void publish_biases(uint64_t tick_id);

    // ---- Config ----
    std::string feet_y_topic_       = "reality.proprio.feet_y";
    int         n_intents_          = 5;
    float       cruse_bias_gain_    = 0.0f;       // 0 = off (default, legacy)
    // 2026-06-08 — per-joint-kind gain on top of cruse_bias_gain_.  Default 0.0
    // for knee → knee Premotors receive NO Cruse bias (was driving the claw
    // pose: V2 trace showed knee_bias 0.4-0.5 fold pressure during stance).
    // hip2 stays at the legacy full gain.  Set knee gain > 0 to opt back into
    // legacy "Cruse pushes knee toward fold during stance" behavior.
    // Joseph 2026-06-08: "concentrate the Cruse rules on hip1 and hip2 for
    // lift and swing.  let the knees learn on their own how to manage the gait."
    float       cruse_bias_gain_knee_ = 0.0f;     // 0 = no knee bias (new default)
    // 2026-06-08 Move 2 — per-joint-kind gain for HIP1 (yaw / swing direction).
    // Default 0.0 = legacy behavior (hip1 unbiased; eligible filter previously
    // excluded hip1).  Set > 0 to enable Cruse bias on hip1: during stance,
    // push leg toward POSTERIOR (push body forward through traction);
    // during swing (Rule 2 release), bias inverts toward ANTERIOR (lift leg
    // forward).  Direction is per-leg via premotor_stance_sign (picrawler
    // empirical: [+1,-1,+1,-1] for FL/FR/RL/RR hip1 — mirrored left/right
    // because positive hip1 u rotates left and right legs in opposite
    // world-frame directions).  Joseph 2026-06-08: "this lack of hip1 is
    // likely a main reason we have not seen a gait emerge; there is no swing
    // of the leg possible."
    float       cruse_bias_gain_hip1_ = 0.0f;     // 0 = no hip1 bias (legacy default)
    // 2026-06-09 Move 4 — per-joint-kind gain for HIP2 (lift / vertical axis).
    // Default 1.0 = legacy behavior (no per-joint multiplier; hip2 used
    // the implicit cruse_bias_gain alone).  Set > 1.0 to amplify the
    // down-during-stance + up-during-swing biases without scaling hip1
    // and knee.  Joseph 2026-06-09 UI observation after hip1 sign fix:
    // "stepping is happening; if there was a bit of hip2 bias downward
    // (positive) during the hip1 swings it would help plant the thrusting
    // leg.  also a bit more hip2 upward (negative) would help with swing
    // forward."  Completes the per-joint gain trio (knee/hip1/hip2).
    float       cruse_bias_gain_hip2_ = 1.0f;     // 1.0 = legacy (no per-joint scaling)
    // 2026-06-10 E2 — rhythm INJECTION.  Adds a CONTINUOUS phase-driven term to
    // total_factor: rhythm_factor = rhythm_inject_gain × cos(phi_free), where
    // phi_free is the per-leg entrained free-running oscillator (advances at
    // default_omega when no footfalls, entrains to touchdown/liftoff when they
    // occur).  cos(phi)= +1 at phi=0 (stance-press), −1 at phi=π (swing-lift), so
    // it injects a periodic lift/plant rhythm the legs lock onto: the free clock
    // SEEDS stepping → stepping produces footfalls → phi_free entrains → coherent
    // gait (closed loop).  Unlike the rule factors (transition-triggered, often
    // total_factor=0 → silent), this is always-on when gain>0, so it can BOOTSTRAP
    // a rhythm that doesn't exist yet — the gap E1's amplifier couldn't cross.
    // Flows through the same magnitude × stance_sign × intent_accels mapping (so
    // it's a gentle logit bias the brain can override for balance, not a motor
    // override).  0 = off (legacy).  Recommended 0.3–1.0.
    float       rhythm_inject_gain_   = 0.0f;
    // 2026-06-09 Move 5 — position-aware bias gate (per Joseph: "Cruse should
    // not apply bias if the joint is already at or past the expected target"
    // + the rudder case where a leg planted parallel-to-forward sweeps through
    // perpendicular when Cruse pushes posterior).  Reads current joint position
    // from reality.proprio.joints.  Computes productive_score = current_u ×
    // bias_direction_sign; suppresses bias when productive_score is outside
    // [zone_min, zone_max].  Default disabled (legacy behavior).
    std::string joints_topic_           = "reality.proprio.joints";
    bool        saturation_gate_enabled_ = false;   // false = legacy unconditional bias
    float       saturation_zone_min_     = 0.0f;    // min productive_score to apply bias
    float       saturation_zone_max_     = 0.9f;    // max productive_score (cuts saturation)
    Eigen::VectorXf current_joint_positions_;        // size = N joints (typically 12), updated via subscription
    float       violation_ema_alpha_= 0.05f;      // EMA rate for violation tracking (telemetry + adaptive mode)
    float       max_violation_ema_  = 1.0f;       // cap on violation_ema (effective bias ceiling under adaptive mode)
    float       hysteresis_low_frac_  = 0.25f;
    float       hysteresis_high_frac_ = 0.50f;
    float       hysteresis_ema_alpha_ = 0.01f;
    bool        publish_when_silent_  = true;     // keep Premotor subscriptions warm
    // Phase 7.13 — multi-rule + authority knobs.
    bool        adaptive_magnitude_   = false;    // false = Hector-faithful constant gain (NEW default);
                                                   // true  = Phase 7.11 v1 violation_ema scaling
    bool        enable_rule_1_        = true;
    bool        enable_rule_2_        = true;
    bool        enable_rule_3_        = true;
    int         rule2_window_ticks_   = 15;       // anterior just-touchdown window for Rule 2 release
                                                   //   (used as bootstrap fallback when swing_duration_ema
                                                   //    has < rule6_min_samples observed swings on the
                                                   //    just-planted leg.  After bootstrap, the window is
                                                   //    rule2_window_fraction × swing_duration_ema —
                                                   //    rhythm-coupled to the body's own observed swing time)
    float       rule2_window_fraction_ = 0.25f;   // Rule 2 window = fraction × swing_duration_ema
    // Minimum genuine-swing duration before counting a touchdown as a Rule 2-relevant
    // stance-start. Filters out chassis-dip glitch touchdowns where foot_y wobbled
    // briefly across stance_y_threshold without the leg ever lifting. Default 0 =
    // legacy (any touchdown qualifies). Recommended 5-10 ticks (~100-200ms).
    int         rule2_min_swing_ticks_ = 0;
    float       rule1_violation_boost_ = 1.5f;    // extra multiplier when Rule 1 active AND violated
    float       rule3_weight_         = 0.5f;     // contralateral-stance bias weight relative to Rule 1
    // Phase 7.13 v3 — coupling topology.
    //   "chain"           = legacy directional cascade via anatomical_anterior (front-to-back or back-to-front)
    //   "symmetric_dwell" = any leg can initiate.  Rule 1 fires on ANY coupled_neighbor in swing.
    //                       Rule 2 fires only on the SUCCESSOR — leg with longest stance dwell
    //                       among coupled_neighbors of any just-planted leg.  Bio-realistic.
    std::string coupling_mode_        = "chain";
    // Phase 7.13 v4 — Rule 6 (step duration memory / self-bias).
    // Phase 7.13 v4.2 — body-state gate.  Multiplies effective bias magnitude by chassis_y_norm
    // so the coordinator goes silent when the body is fallen (no gait coordination is meaningful
    // on a belly-down robot).  Addresses three coupled defects exposed by the v4.1 belly-stuck
    // glitch: (1) rules firing during pathological states, (2) EMA contamination from those
    // states, (3) softmax saturation from constant stance bias.
    // When body_state_topic is set and a message arrives, body_state_value_ tracks the latest
    // chassis_y_norm.  Effective magnitude = cruse_bias_gain × warmup_ramp × body_state_value_.
    // body_state_min_threshold: hard floor — below this, magnitude = 0 (don't fire at all).
    std::string body_state_topic_     = "";     // empty = disabled (legacy behavior)
    float       body_state_value_     = 1.0f;   // last-seen chassis_y_norm; default 1.0 if no topic
    bool        body_state_seen_      = false;  // becomes true on first message
    float       body_state_min_threshold_ = 0.3f;  // below this fraction, magnitude collapses to 0

    // Phase 7.13 v4.1 — amplitude warmup.  Gives Premotors a quiet window to learn what
    // STANCE means from REINFORCE reward before Cruse rules add directional bias.
    // Without this, rules push toward stance from tick 1 while the policy is still random →
    // brain learns "stance = what Cruse pushed me to" rather than "stance = what earned reward."
    int         warmup_ticks_         = 0;       // 0 = off (default — Cruse fully active from tick 1).
                                                  //   N > 0: linear ramp factor (0 → 1) over first N ticks
                                                  //   applied multiplicatively to cruse_bias_gain.
    bool        enable_rule_6_        = false;   // default off → byte-identical to v3 when set false.
    float       rule6_ema_alpha_      = 0.2f;    // EMA rate for swing/stance duration tracking
    float       rule6_max_swing_ratio_  = 2.0f;  // current_swing > ratio × swing_ema → bias to stance
    float       rule6_max_stance_ratio_ = 1.5f;  // current_stance > ratio × stance_ema → bias to swing
    int         rule6_max_swing_abs_  = 1800;    // 30s @60Hz — generous, slow-gait-friendly
    int         rule6_max_stance_abs_ = 3600;    // 60s @60Hz — accommodates very slow gaits
    int         rule6_min_samples_    = 3;       // # of completed phases before EMA is trusted
    // Phase 7.17 — closed-loop rhythm output.  When enabled, CruseCoordinator
    // publishes a 2-D [cos(phi), sin(phi)] ProprioToken per leg, where phi is
    // computed from actual touchdown/liftoff events + EMA durations.  Drop-in
    // replacement for CPGOscillator's open-loop perceptual output.  Rhythm
    // EPMs subscribe to the same topics; only the publisher changes.
    bool                       publish_phase_signals_ = false;
    std::vector<std::string>   phase_output_topics_;    // per-leg, same length as leg_state_

    // 2026-05-29 — per-leg discrete bucket (swing=0 / stance=1) published as a
    // single-float ProprioToken so Premotors can condition policy per bucket.
    // Drives the gait-bucket bet (per-bucket Premotor bias term).
    bool                       publish_bucket_signals_ = false;
    std::vector<std::string>   bucket_output_topics_;   // per-leg, same length as leg_state_
    // Phase 7.19 — entrained mode controls.
    std::string                phase_mode_           = "state_event";  // "state_event" | "entrained_free" | "smooth_fallback"
    float                      entrainment_strength_ = 0.20f;          // K — fraction of phase-error to snap on each transition
    float                      default_omega_        = 0.0349f;        // rad/tick; default = 2π/180 (3s cycle @60Hz)
    float                      phase_blend_window_   = 60.0f;          // ticks of no-transition before smooth_fallback fully switches to phi_free

    // ---- State ----
    std::vector<LegState>      leg_state_;
    std::vector<PremotorState> premotor_state_;
    uint64_t                   current_tick_ = 0;
    // For telemetry: total active-rule firings + total compliant ticks (Rule 1).
    int                        total_rule1_violations_ = 0;
    int                        total_rule1_compliant_  = 0;
    int                        total_rule1_fires_      = 0;
    int                        total_rule2_fires_      = 0;
    int                        total_rule3_fires_      = 0;
    int                        total_rule6_fires_      = 0;
};

} // namespace ogma
