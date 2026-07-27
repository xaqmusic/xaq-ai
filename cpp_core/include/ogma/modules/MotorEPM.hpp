#pragma once

// =============================================================================
// MotorEPM.hpp  --  Homeokinetic sensorimotor controller (the Motor-EPM)
// =============================================================================
//
// The motor mirror of the perceptual EPM, closed around the action loop.
// Per leg it holds:
//   * a forward SELF-MODEL  M:  x̂(t+1) = A·y(t) + b        (motor → next sensor)
//   * a CONTROLLER          K:  y(t)   = tanh(C·x(t) + h)   (sensor → motor)
// and learns BOTH online from the motor Time-Loop Error
//   ξ(t+1) = x(t+1) − x̂(t+1).
//
// The controller descends the sensitivity metric E = ξᵀ (L Lᵀ + εI)⁻¹ ξ with
// L = A·G·C the loop Jacobian (G = diag(g'(Cx+h))).  Because E carries the
// INVERSE loop gain, descending it MAXIMIZES the body's responsiveness to its
// own actions while keeping the model accurate.  The standing fixed point has
// near-zero sensitivity → E is large there → the rule cannot rest at standing.
// Destabilization is intrinsic; the HK CORE (self-model + controller TLE descent)
// has NO reward term.  CAVEAT: this module also offers an OPTIONAL internal
// agency-fitness search (coord_reward_drive, inert at 0 — see :158).  That search
// IS an objective optimizer over forward proprioceptive thrust; it is intrinsic,
// not an external/environment reward, but the module is NOT "reward-free" in the
// strict sense whenever coord_reward_drive > 0.
//
// This realizes the design in docs/plans-and-designs/motor_epm_homeokinetic_plan.md.
// DEVIATION FROM THE PLAN MANIFEST: the plan listed ForwardSelfModel and
// HKController as two modules; they are fused here because the HK update needs
// A (model) and C (controller) within ONE tick to form L — splitting across the
// bus would inject a tick of latency and force cross-module matrix sharing.
//
// Symmetry breaking: each leg's (A,b,C,h) is initialized from its OWN RNG seed
// (base_seed ^ leg), so the legs do NOT collapse to the bilateral mirror that
// the shared-consensus Premotors did (v6-premotor-bilateral-mirror-collapse).
// Inter-leg coupling (signed, antiphase-capable) is added in Rung 3 as a sibling
// LateralMotorCoupling; this module is the uncoupled Rung 1/2 substrate.
//
// Module lifecycle authoring contract: docs/plans-and-designs/primitives/_module_lifecycle.md.

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class MotorEPM : public Module {
public:
    MotorEPM();
    ~MotorEPM() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;   // live viz: motor-TLE + self-model + cog drive
    void           restore_state(nlohmann::json const& s) override;

    // Telemetry (read by OgmaBrain::get_module_metrics / diag).
    int    n_legs()            const { return n_legs_; }
    float  motor_tle_mean()    const { return tle_ema_mean(); }
    float  loop_gain_mean()    const { return gain_ema_mean(); }
    float  out_mag_mean()      const { return outmag_ema_mean(); }
    // 2026-06-16 diag — the ACTUAL cognitive steer the bug receives (ground truth;
    // the ActionDecoder's own telemetry reads the wrong topic).  cog_steer_msgs
    // counts handle_cog_steer fires → distinguishes "critic publishes straight-0"
    // (msgs>0, steer~0) from "critic not publishing" (msgs==0).
    float  cog_steer_diag()    const { return cog_steer_; }
    int    cog_steer_msgs()    const { return cog_steer_msgs_; }
    float  cog_thrust_diag()   const { return cog_thrust_; }
    int    cog_thrust_msgs()   const { return cog_thrust_msgs_; }
    float  boredom_diag()      const { return boredom_; }
    float  interest_diag()     const { return interest_; }
    float  hunger_diag()       const { return hunger_; }            // 0 sated → 1 starving (forage gate)
    int    boredom_streak_diag() const { return boredom_streak_; }  // sustained-boredom ticks (desperation)
    // Food bearing the critic's state SHOULD encode (egocentric scent_compass,
    // from nav_topic): tc_x = lateral (right+), tc_y = forward. Diagnostic for
    // critic state-discriminability (does consensus winner track this?).
    float  tc_x_diag()         const { return tc_x_; }
    float  tc_y_diag()         const { return tc_y_; }
    float  gait_coherence()    const;     // |mean_j e^{i(φ_j − P_j)}| ∈ [0,1]; 1 = locked to the gait
    int    legs_initialized()  const;

private:
    void handle_proprio(int leg, MessagePtr payload);
    void handle_cpg_phase(MessagePtr payload);
    void handle_tilt(MessagePtr payload);
    void handle_imu(MessagePtr payload);
    void handle_nav(MessagePtr payload);
    void handle_height(MessagePtr payload);
    void handle_distress(MessagePtr payload);
    void handle_lateral(MessagePtr payload);
    void ensure_leg_init(int leg, int state_dim);      // lazy alloc once n known
    float tle_ema_mean()    const;
    float amp_ema_mean()    const;   // mean per-leg oscillation amplitude (activity)
    float amp_gain_mean_val() const; // mean per-leg amplitude-homeostat integrator
    float gain_ema_mean()   const;
    float outmag_ema_mean() const;

    // ---- Configuration (params) ----
    std::vector<std::string> proprio_topics_;   // one ProprioToken topic per leg
    std::vector<std::string> action_topics_;    // n_legs * motor_dim ActionOut topics
    std::vector<std::string> objective_topics_; // optional per-leg PredictionToken posture-target topics; empty = socket OFF (§1.1)
    int     n_legs_      = 4;
    int     motor_dim_   = 3;                    // hip1, hip2, knee
    double  model_lr_    = 0.02;                 // η_M  forward-model rate
    double  ctrl_lr_     = 0.01;                 // η_K  controller (HK) rate
    double  bias_lr_     = 0.005;                // η_h  controller bias rate
    double  reg_eps_     = 0.01;                 // ε in (L Lᵀ + εI)⁻¹
    double  max_dctrl_   = 0.05;                 // per-tick clamp on ‖ΔC‖_F (ignition guard)
    double  init_scale_  = 0.01;                 // init magnitude of C (starts near standing)
    int64_t base_seed_   = 1234;                 // per-leg seed = base ^ leg
    // 2026-06-12 — anti-freeze additions (the bare metric-gradient update
    // saturated tanh in ~2 s then froze: g'→0 kills every term of ΔC).
    int64_t babble_ticks_ = 200;                 // motor-babble warmup: model learns, controller idle
    double  babble_scale_ = 0.3;                 // amplitude of babble commands during warmup
    double  sat_lr_       = 0.02;                 // anti-saturation rate (surrogate for dropped ∂G term)
    // 2026-06-12 — postural reflex (spinal-tone analog).  Pure HK has no "up":
    // the controller drifted the body into a folded collapse.  A weak PD pull
    // toward the standing REST pose (proprio pos=0 for every joint) gives HK a
    // stable upright fixed point to bifurcate FROM.  Applied to hip2+knee only
    // (the height-setting joints); hip1=yaw is per-leg sign-flipped by the body
    // so it is left to HK.  Reward-free — a fixed feedback law, not a reward.
    double  postural_gain_ = 0.3;                 // restoring strength toward REST (0 = off)
    std::vector<double> postural_gain_joints_;    // per-joint [hip1,hip2,knee] postural strength; empty = scalar for all
    // 2026-06-12 — persistent exploration noise.  HK learns from the prediction
    // error ξ; at ANY static fixed point the model learns to predict it, ξ→0,
    // and the controller update vanishes (froze at the postural equilibrium).
    // Der–Martius inject persistent motor noise so ξ never zeroes and the
    // sensitivity-seeking controller AMPLIFIES it into oscillation.  This is
    // the "playful" exploration drive, not a tuning knob.
    double  explore_noise_ = 0.05;                // per-tick Gaussian motor noise σ (always on)
    // 2026-06-12 — spider-stance target (operator insight).  Standing tall is an
    // inverted-pendulum equilibrium that needs active balance we don't have; the
    // SPIDER stance (knees tucked up, chassis suspended below, CoG low + wide
    // base) is STATICALLY stable, so postural tone alone holds it.  Overrides the
    // captured spawn knee-rest with a tuck target (pos→+1 = full 170° tuck) so
    // the postural reflex drives the body into the spider pose.  −99 = use spawn.
    double  knee_tuck_target_ = -99.0;            // postural knee-rest pos override (spider tuck)
    double  hip2_tuck_target_ = -99.0;            // postural hip2(femur)-rest override (crouch for leverage); -99 = off
    double  motor_gain_ = 1.0;                     // output amplitude on the HK command (legs look weak → raise)
    // 2026-06-12 — Rung 3: signed inter-leg coupling.  HK reliably lights ONE leg
    // into a propulsive limit cycle (init lottery) while the others twitch; one
    // motor → the body spins, not walks.  A Kuramoto network couples the four
    // legs' OWN emergent phases toward a gait offset pattern (the inherited
    // coordination topology), entraining the twitchers and phase-locking all four
    // for symmetric thrust.  Rhythm emerges; only the offsets are imposed.
    double  coupling_gain_ = 0.0;                  // Kuramoto coupling strength (0 = off; live transition knob)
    int     phase_joint_   = -1;                   // proprio joint sourcing the per-leg oscillator phase (-1 = knee = m-1)
    std::vector<double> rhythm_gains_;             // per-joint coherent rhythmic drive amplitude (empty/0 = off)
    std::vector<double> rhythm_offsets_;           // per-joint phase offset for the rhythmic drive
    std::string         cpg_phase_topic_;          // optional global CPG phase to drive the rhythm from (clean, entrained)
    float               cpg_phase_ = 0.0f;
    bool                cpg_seen_  = false;
    // CPG-as-embedding (controller-only conditioning): the HK controller learns a phase-dependent
    // bias  y = tanh(C·x + Cphi·[cosφ,sinφ] + h)  → phase MODULATES the learned control law (context),
    // it never commands a joint.  HK exploration + balance reflexes stay live.  Default off = byte-identical.
    // Cphi is trained NOT on HK surprise (which damps motion) but to reduce the KEYFRAME error
    // (x* − x) at the command phase — a phase-indexed feed-forward toward the learned posture,
    // self-limiting as the error shrinks.
    bool                cpg_embed_   = false;
    double              embed_lr_    = 0.02;   // Cphi learning rate on the keyframe error
    double              embed_decay_ = 0.001;  // L2 decay bounding the learned feed-forward
    // Per-leg CONTROLLER symmetry coupling (anti-asymmetry root fix): softly pull each leg's
    // learned controller (C, h, Cphi) toward its group's cross-leg average each tick, so the
    // four identical legs converge to ONE control law instead of one leg specializing (the
    // RR-skid).  Group by SAME stroke-sign (sign-safe: no fore-aft mirror conflict).
    double              ctrl_symmetry_gain_ = 0.0;  // per-tick pull toward group-average controller (0 = off)
    std::vector<int>    symmetry_group_of_;         // per-leg group id (length n_legs); empty = off
    // Gate 2 (coherent-scaffold wean): tick-scheduled fade of the rhythm drive so the LEARNED
    // keyframe takes over the gait. -1 = disabled.
    int64_t             rhythm_fade_start_ = -1;
    int64_t             rhythm_fade_end_   = -1;
    float               rhythm_scale_      = 1.0f;
    // Gate 2 (Kuramoto-contrast): deterministic tick-scheduled fade of the imposed coupling so the
    // LEARNED keyframe map takes over (crystallize-then-wean). -1 = disabled (coupling stays flat).
    int64_t coupling_fade_start_ = -1;             // begin linear fade at this tick
    int64_t coupling_fade_end_   = -1;             // coupling = 0 from this tick on
    float   coupling_eff_        = 0.0f;           // effective (faded) coupling used this tick (diag)
    std::vector<double> gait_phase_ = {0.0, 3.14159265, 3.14159265, 0.0};  // per-leg target phase (trot: diagonals in-phase)
    // 2026-06-14 — ADAPTIVE COORDINATION (prototype).  gait_phase is normally a
    // FIXED imposed trot, but the body fights it (settles to antiphase diagonals).
    // When coord_adapt_rate>0 the offsets slowly CRYSTALLISE toward the body's own
    // emergent per-leg phase pattern (Hebbian self-organisation, reward-free) plus
    // a persistent exploration (coord_explore) so the coordination never fully
    // freezes — the homeokinetic "keep probing" meant to give continuous gait
    // improvement.  Leg 0 is the phase reference (offset pinned at 0).
    // Which fitness the (1+1) coordination search ranks probes by.  0 = legacy fwd_v
    // (a TASK REWARD — see §5.1); 1 = reward-free coherence·activity/(1+tle).  Default
    // 0 keeps every existing config byte-identical.
    // Uprightness (cos_pitch*cos_roll ~ basis.y.y): +1 upright, 0 on its side, -1 on its
    // back.  Gates the homeostat integrators — see homeo_upright_gate.
    float   upright_ = 1.0f;
    double  homeo_upright_gate_ = 0.0;   // 0 = off (byte-identical)
    double  homeo_leak_cycles_  = 0.0;   // homeostat forgetting time constant, in strides
    double  homeo_leak_progress_gate_ = 0.0;  // 1 = only forget while making progress
    double  homeo_leak_upright_only_  = 0.0;  // stop forgetting below this uprightness
    float   homeo_leak_eff_ = 0.0f;      // diag: effort leak rate after the gates
    float   leak_amp_ = 0.0f;            // amp_gain (effort) forgetting rate this tick
    float   leak_h_   = 0.0f;            // height_bias (posture) forgetting rate this tick
    std::string rhythm_topic_;           // rhythm.body.gait — supplies omega for the leak
    float   body_omega_ = 0.0f;          // body gait rate (rad/tick) from that topic
    void    handle_rhythm(MessagePtr payload);
    // Stride period used only until the rhythm token arrives; the measured value is ~70.
    static constexpr float kLeakFallbackPeriod = 70.0f;
    double  height_unwind_free_ = 0.0;   // 0 = legacy symmetric windup fade
    int     coord_fitness_mode_ = 0;
    double  coord_adapt_rate_ = 0.0;   // crystallisation rate toward the emergent pattern (0 = fixed)
    double  coord_explore_    = 0.0;   // persistent phase-offset exploration noise (rad/tick)
    std::mt19937 coord_rng_{0xC007Du}; // coordination exploration stream
    // 2026-06-14 — AGENCY-REWARD coordination search (intent→action gradient).  A
    // (1+1) hill-climb on the phase offsets: probe → measure CONTROLLABILITY (the
    // forward proprioceptive thrust fwd_v — for a flat-ground legged body this
    // requires COORDINATED propulsion, not driftable/freezable, so it's Goodhart-
    // robust) → keep probes that improve it.  The directional drive for continuous
    // gait improvement, inside the homeokinetic frame.  coord_reward_drive = probe
    // scale (0 = off); the offsets climb toward the most-controllable coordination.
    double  coord_reward_drive_ = 0.0;          // probe noise scale (rad); 0 = off
    int64_t coord_probe_ticks_  = 240;          // window per probe (4 s @ 60 Hz)
    double  coord_stab_penalty_ = 0.3;          // tilt penalty in the fitness (edge-of-chaos guard)
    double  coord_lat_penalty_  = 0.0;          // |lateral velocity| penalty in the fitness (anti-crab)
    double  coord_intent_nav_   = 0.0;          // SYMMETRIC CONTROLLABILITY reward: 0 = reward forward
                                                // velocity (legacy); >0 = reward velocity toward the intended
                                                // direction (target_compass) — forward AND turn symmetric.
    float   fwd_v_ = 0.0f;                       // chassis forward velocity (imu idx 2)
    float   lateral_v_ = 0.0f;                   // chassis signed lateral velocity (sideways slip)
    std::vector<double> coord_best_phase_;       // incumbent (best-so-far) offsets
    float   coord_best_fitness_  = 0.0f;
    bool    coord_best_init_     = false;
    int64_t coord_probe_counter_ = 0;
    float   coord_fit_accum_     = 0.0f;
    int64_t coord_fit_count_     = 0;
    // 2026-07-23 — STUCK→EXPLORE desire (active-inference-native propulsion): when the
    // body makes ~no forward progress (fwd_v EMA below threshold) for a sustained window,
    // AMPLIFY the exploration channels (explore_noise + the coord phase-search σ) so the
    // gait DISCOVERS a push — curiosity down the corridor, no external goal/reward.  The
    // bearing-hold keeps the heading straight while it searches → directed exploration.
    // Self-terminating: the boost decays the instant forward progress resumes.  0 = off.
    double  stuck_explore_gain_  = 0.0;         // max exploration amplification at full stall (0 = off)
    float   fwd_progress_ema_    = 0.0f;        // slow EMA of forward velocity (the stall detector, compliant)
    int     stuck_ticks_         = 0;           // consecutive ticks below the progress threshold
    float   stuck_boost_         = 0.0f;        // 0..1 exploration boost (ramps when stuck, decays when moving)
    // --- progress→COMMIT (lever C): the INVERSE twin of stuck→explore.  When forward
    // progress is HIGH and sustained (fwd_progress_ema_ above a commit threshold), ramp a
    // boost that (1) DAMPS the exploration channels (stop re-searching a found push) and
    // (2) ADDS stroke thrust (drive into the committed direction).  Mutually exclusive with
    // stuck_boost_ by construction (one needs low progress, the other high).  Egocentric,
    // self-correcting (decays when progress falls).  0 = off → commit_boost_ pinned 0.
    double  progress_commit_gain_ = 0.0;        // max exploration damp + thrust add at full commit (0 = off)
    int     commit_ticks_         = 0;          // consecutive ticks above the commit threshold
    float   commit_boost_         = 0.0f;       // 0..1 commit boost (ramps when flowing, decays when stalled)
    // --- forward-FLOW homeostat (lever D): homeokinesis applied to locomotion.  Amplify
    // stroke thrust ∝ the quality of forward flow, where quality = magnitude · PREDICTABILITY
    // (strong AND steady fwd_v).  flow_vol_ema_ is the volatility (mean-abs-deviation of
    // fwd_v); the 1/(1+k·vol) factor rewards predictable flow, not raw speed — the
    // homeokinetic heart.  Continuous (no threshold), egocentric.  0 = off.
    // --- STANCE-LIFT (belly-up while walking): a knee bias on PLANTED (stance) legs only
    // — raises the chassis off the feet it can push against (traction preserved, unlike
    // hip2 lift which rotates the feet off the ground) while swing legs cycle freely (no
    // DC clamp on the rhythm).  Held constant (NOT faded) so the belly rides high during
    // fast flat walking to protect the chassis.  0 = off.  Sign set empirically.
    double  stance_lift_gain_    = 0.0;
    double  forward_flow_gain_   = 0.0;         // max stroke amplification at ideal (strong+steady) flow (0 = off)
    float   flow_ema_            = 0.0f;        // EMA of fwd_v (forward-flow magnitude)
    float   flow_vol_ema_        = 0.0f;        // EMA of |fwd_v − flow_ema_| (flow volatility = un-predictability)
    float   flow_quality_diag_   = 0.0f;        // transient: last-tick flow_quality (diag only, not serialized)
    float   height_rest_frac_    = 1.0f;        // transient: height-defense fade (1 at rest → 0 while moving fwd)
    // 2026-06-14 — CoT-seeking AMPLITUDE search.  Twin of the phase search but on
    // amp_target: a (1+1) hill-climb maximising fwd_v / oscillation-amplitude (speed
    // per effort = inverse cost of transport).  Lowers gait amplitude until forward
    // thrust starts to fall → the efficient amplitude.  Attacks the DOMINANT motor-
    // effort term (leg-cycling amplitude) the phase search structurally cannot.
    double  amp_seek_rate_  = 0.0;              // probe scale on amp_target (0 = off)
    int64_t amp_seek_ticks_ = 900;             // window per amplitude probe (~15 s @ 60 Hz)
    double  amp_seek_best_target_  = 0.0;       // incumbent amp_target
    float   amp_seek_best_fitness_ = 0.0f;      // best speed-per-amplitude so far
    bool    amp_seek_init_      = false;
    int64_t amp_seek_counter_   = 0;
    float   amp_seek_fwd_accum_ = 0.0f;
    float   amp_seek_amp_accum_ = 0.0f;
    int64_t amp_seek_count_     = 0;
    float   cur_amp_ = 0.0f;                    // mean oscillation amplitude across legs (this tick)
    // 2026-06-14 — CRUSE/Walknet inter-leg coordination as a CORRECTOR (not a
    // generator) layered on the MotorEPM rhythm.  Continuous hip2 bias (+hip2 = foot
    // down/stance, −hip2 = foot up/swing): Rule 1 (anterior in swing → hold stance),
    // Rule 2 (anterior just-planted → release swing), Rule 3 (contralateral in swing →
    // hold stance).  Stance/swing per leg from foot height vs a self-calibrating EMA.
    // Catches the per-leg co-swing / support-loss failures the rhythm alone leaves.
    double  cruse_gain_         = 0.0;          // 0 = off (opt-in); hip2 bias magnitude
    double  cruse_rule3_weight_ = 0.5;          // contralateral (Rule 3) weight vs anterior
    int64_t cruse_rule2_window_ = 15;           // ticks after anterior touchdown that Rule 2 releases swing
    double  cruse_rule5_gain_   = 0.0;          // Rule 5: load distribution — stance legs press down (load
                                                // their foot for grip) ∝ swinging-neighbour count.  0 = off.
    std::string feet_topic_     = "reality.proprio.feet_y";  // 4-D per-leg foot height
    std::vector<float>   foot_y_;               // latest per-leg foot height
    std::vector<float>   foot_y_ema_;           // self-calibrating stance/swing threshold
    std::vector<char>    in_swing_;             // foot above its own EMA = lifted (swing)
    std::vector<int64_t> ticks_since_plant_;    // ticks since each leg's last touchdown
    std::vector<int>     cruse_anterior_;       // per-leg anatomical anterior idx (−1 = none)
    std::vector<int>     cruse_contra_;         // per-leg contralateral idx (−1 = none)
    static constexpr float kFootYEmaAlpha = 0.02f;  // ~50-tick foot-height mean
    // 2026-07-25 — DEADBAND on the swing detector.  The bare `foot_y > foot_y_ema`
    // test has no deadband, so it splits ~50/50 by construction (a signal is above
    // its own moving average about half the time) — it is a PHASE test, not a
    // contact test.  Worse, any consumer that moves the foot (stance_lift, Cruse)
    // closes a positive feedback loop through it: bias raises the foot → foot goes
    // above its EMA → declared swing → bias removed → foot drops → declared stance →
    // bias returns.  That is a relaxation oscillator running at the EMA's ~50-tick
    // timescale, competing with the body's own ~70-tick stride.
    //
    // The band must stay RELATIVE to the EMA: foot_y is world-Y, so an absolute
    // threshold is a god's-eye quantity that would read a planted foot on raised
    // terrain as permanently swinging (the same blindness that retired
    // chassis_y_norm).  And it must not be a tuned constant — it is scaled by the
    // foot's OWN running mean-absolute-deviation, so it tracks whatever amplitude
    // the gait happens to have (CLAUDE.md §5.5: adapt it, don't tune it).
    // 2026-07-25 — "did MotorEPM's Cruse block actually contribute?"  There are TWO
    // Rule-3 parameters in this codebase: MotorEPM's `cruse_rule3_weight` (a sub-weight
    // INSIDE the cruse_gain block — inert when cruse_gain == 0) and CruseCoordinator's
    // own `rule3_weight` (a separate module, gated by `cruse_bias_gain` which defaults
    // to 1.0, i.e. ON).  The MOTOR-EPM panel writes the FORMER.  Zeroing MotorEPM's
    // cruse_gain therefore says nothing about the latter, and the two are easy to
    // confuse by eye.  This accumulator makes MotorEPM's own contribution observable so
    // the question is answered by a number instead of an inference.
    float    cruse_bias_acc_ = 0.0f;            // Σ|cruse contribution| since last diag
    uint64_t cruse_bias_n_   = 0;               // samples in that sum
    float    cruse_bias_mean_ = 0.0f;           // published mean (0 ⇒ block never ran)
    // 2026-07-25 — can a LEGAL phase reference reproduce what the god's-eye swing detector
    // outputs?  `feet_y` is absolute world-Y (see the oracle design note); the detector
    // built on it behaves like a phase gate rather than a contact gate.  If the body's own
    // rhythm phase (already subscribed as cpg_phase_, entrained to the body by
    // BodyRhythmTracker) agrees with that detector, the oracle can be replaced by a signal
    // a real robot has.  Measured BEFORE building the replacement.
    //   ≈0.5 → the phase carries no information about the detector's output
    //   ≈1.0 → the detector IS a phase gate and the phase can stand in for it
    float    phase_agree_ema_    = 0.5f;   // vs the GLOBAL body/CPG phase
    float    legphase_agree_ema_ = 0.5f;   // vs each leg's OWN joint-derived phase
    static constexpr float kPhaseAgreeAlpha = 0.002f;   // ~500-tick mean (several strides)
    std::vector<float>   foot_y_mad_;           // running mean |foot_y − ema|, per leg
    double  swing_hyst_frac_ = 0.0;             // deadband in units of MAD (0 = legacy)
    float   swing_frac_ema_  = 0.0f;            // diag: smoothed fraction of legs in swing
    static constexpr float kFootYMadAlpha  = 0.02f;  // MAD tracks on the EMA's timescale
    static constexpr float kSwingFracAlpha = 0.01f;  // slow, for a readable diagnostic
    void handle_feet(MessagePtr payload);
    // TRUE ground contact per leg (`reality.proprio.foot_contact`) — already published by
    // the body every tick, and the sensor a real picrawler actually has.  Empty topic =
    // fall back to the legacy foot-height inference (byte-identical).
    std::string          contact_topic_;
    std::vector<float>   foot_contact_;
    bool                 have_contact_ = false;
    void handle_contact(MessagePtr payload);
    // 2026-07-26 — let `contact_topic` be subscribed WITHOUT letting it drive in_swing_.
    // Wiring true contact as the swing gate is separately REFUTED (net_z 3.76→2.37: the
    // consumer wanted gait PHASE, not contact), so the alignment diagnostic below must be
    // able to read ground truth while the control path keeps the incumbent detector.
    // 0 (default) = legacy: contact_topic, when set, drives the gate as before.
    double               contact_instrument_only_ = 0.0;
    // Per-servo LOAD (`reality.proprio.joint_torque`, 12 floats, layout hip1[0..3],
    // hip2[0..3], knee[0..3] — joint-major, leg order FL,FR,RL,RR).  Published every tick
    // by the body since 2026-06-01 and never consumed by anything.  On hardware this is
    // servo current sensing, so it is a legal egocentric observation.  Empty = off.
    std::string          torque_topic_;
    std::vector<float>   joint_torque_;
    bool                 have_torque_ = false;
    void handle_torque(MessagePtr payload);
    float leg_load(int leg) const;          // Σ|τ| over that leg's servos (0 if no signal)
    std::string          upright_topic_;   // reality.proprio.upright (basis.y.y)
    bool                 have_upright_ = false;
    void handle_upright(MessagePtr payload);
    void update_cruse_state();

    // ------------------------------------------------------------------
    // PHASE-0 GAIT-ALIGNMENT DIAGNOSTIC (2026-07-26).  DIAGNOSTIC ONLY — nothing in this
    // block feeds a command, and `gait_align_diag_ == 0` (the default) skips it entirely.
    //
    // The question it answers: the propulsive stroke rides `L.phase`, derived from the
    // KNEE (`phase_joint` default −1), while the stance gate rides the FOOT-HEIGHT cycle.
    // Those are two different clocks, and the module's own `legphase_agree_ema_` already
    // reads ~0.5 (chance) between them.  If they are genuinely unlocked then each leg
    // pushes backward without regard to whether its foot is on the ground — roughly half
    // the power stroke spent in the air and half the return swing spent scrubbing — which
    // would explain BOTH the operator's "it is always stumbling" and the ledger's standing
    // unknown that flat speed is pinned across every timing lever ever tried.
    //
    // The headline is a phase-locking value (PLV): accumulate e^{iθ} at each TOUCHDOWN,
    // where θ is the stroke waveform's phase.  PLV≈0 ⇒ touchdown happens at a uniformly
    // random stroke phase (unlocked clocks).  PLV≈1 ⇒ locked, and the accompanying mean
    // angle says whether `stroke_phase` is merely offset.
    // ------------------------------------------------------------------
    double  gait_align_diag_ = 0.0;             // 0 = off (whole block skipped, zero cost)
    void update_gait_align_diag(uint64_t tick_id);
    double  ga_td_cos_ = 0.0, ga_td_sin_ = 0.0; // PLV accumulator vs TRUE contact touchdown
    int64_t ga_td_n_   = 0;
    double  ga_sd_cos_ = 0.0, ga_sd_sin_ = 0.0; // PLV vs the INCUMBENT detector's plant
    int64_t ga_sd_n_   = 0;
    double  ga_align_acc_ = 0.0;                // E[ sgn·sin θ · (contact ? +1 : −1) ]
    int64_t ga_align_n_   = 0;
    double  ga_stance_pos_ = 0.0;               // frac of STANCE ticks with sgn·sin θ > 0
    int64_t ga_stance_n_   = 0;
    double  ga_swing_pos_  = 0.0;               // same over SWING ticks (the complement)
    int64_t ga_swing_n_    = 0;
    double  ga_contact_acc_ = 0.0;              // true stance duty, for reference
    int64_t ga_contact_n_   = 0;
    // Does joint_torque separate stance from swing?  Prerequisite for a load-gated stroke:
    // if these means do not part, a load lever dies here for the cost of one run.
    double  ga_tq_stance_ = 0.0; int64_t ga_tq_stance_n_ = 0;
    double  ga_tq_swing_  = 0.0; int64_t ga_tq_swing_n_  = 0;
    double  ga_tq_agree_  = 0.0; int64_t ga_tq_agree_n_  = 0;   // 0.5 = chance
    std::vector<float>   ga_tq_ema_;            // per-leg running load mean (the threshold)
    // Same test on hip1 ALONE — what a load-derived step clock would actually threshold.
    // The summed version measures 0.540 (near chance) while the per-joint ratios say hip1
    // separates better than the sum (1.368 vs 1.148), so the sum may be diluting it.
    double  ga_tq_h1_agree_ = 0.0; int64_t ga_tq_h1_agree_n_ = 0;
    std::vector<float>   ga_tq_h1_ema_;
    // Per-JOINT stance/swing load, because summing all three servos may be diluting the
    // contrast: hip1 does fore-aft work in BOTH phases, while hip2 and the knee are the
    // ones actually holding the body up.  Whichever joint separates best is the input a
    // load-gated stroke should read.
    std::vector<double>  ga_tq_j_stance_, ga_tq_j_swing_;
    int64_t              ga_tq_j_stance_n_ = 0, ga_tq_j_swing_n_ = 0;
    // Cycle periods, in ticks, from up-crossings of each signal against its own mean.
    // knee comes free from cos(L.phase) sign flips; foot comes free from in_swing_ 0→1.
    std::vector<float>   ga_hip1_ema_;
    std::vector<char>    ga_hip1_above_, ga_knee_above_, ga_prev_contact_, ga_prev_swing_;
    std::vector<int64_t> ga_hip1_last_, ga_knee_last_, ga_foot_last_, ga_con_last_;
    // ga_foot_per_ is the INCUMBENT DETECTOR's cycle; ga_con_per_ is the real step period
    // from the physics touch flag.  They must be reported separately — the detector is a
    // self-referential threshold that any foot-moving bias rings, so a fast ga_foot_per_
    // next to a slow ga_con_per_ is chatter, not stepping.
    std::vector<float>   ga_hip1_per_, ga_knee_per_, ga_foot_per_, ga_con_per_;
    // Yaw disturbance attributed to swinging.  The operator's UI observation (2026-07-27):
    // the rear legs sweep forward with hip2 and the knee held near-horizontal, so the limb's
    // yaw inertia about the body axis is near maximal and the reaction torque spins the
    // chassis, which the heading controller then has to fight.  These accumulate |yaw rate|
    // separately for "all four feet down" vs "this leg is off the ground", so the claim is
    // testable on its own mechanism instead of only on downstream distance.
    std::vector<double>  ga_yaw_leg_;           // Σ|yaw| while leg i is airborne
    std::vector<int64_t> ga_yaw_leg_n_;
    double  ga_yaw_allplant_ = 0.0; int64_t ga_yaw_allplant_n_ = 0;
    double  ga_yaw_anyswing_ = 0.0; int64_t ga_yaw_anyswing_n_ = 0;
    // ...and the same split on |Δyaw rate|, which is the metric that can actually SEE the
    // mechanism.  Measured 2026-07-27: mean |yaw rate| is dominated by INTENTIONAL steering
    // — heading_bearing_hold yaws the body by differencing stroke magnitude per side, which
    // only bites through PLANTED feet — so the aggregate reads *lower* during swing and the
    // reaction torque is invisible under it.  A limb's reaction torque is an IMPULSE: it
    // changes the yaw rate rather than sustaining one.  Differencing separates the two.
    std::vector<double>  ga_yawd_leg_;
    std::vector<int64_t> ga_yawd_leg_n_;
    double  ga_yawd_allplant_ = 0.0; int64_t ga_yawd_allplant_n_ = 0;
    double  ga_yawd_anyswing_ = 0.0; int64_t ga_yawd_anyswing_n_ = 0;
    float   ga_prev_yaw_rate_ = 0.0f; bool ga_yaw_prev_init_ = false;
    float   explore_mult_diag_ = 1.0f;          // progress→commit damping actually applied
    static constexpr float kGaEmaAlpha  = 0.02f;   // ~50-tick mean, matches kFootYEmaAlpha
    static constexpr float kGaPerAlpha  = 0.10f;   // period smoothing (~10 cycles)

    // ------------------------------------------------------------------
    // LOAD-GATED POWER STROKE (2026-07-26).  The stroke is the one major bias in this
    // stack that is UNGATED, and Phase 0 measured what that costs: the fraction of STANCE
    // spent in the stroke's positive half is 0.512, and over SWING it is 0.513 — the push
    // direction is statistically independent of whether the foot is on the ground.  Half
    // the power stroke is spent in the air.
    //
    // CLAUDE.md §1 step 4: gate the bias by the state that makes it valid — the gate is
    // the design, the magnitude is only tuning.  Thrust is valid against PURCHASE, so the
    // gate is per-leg load, and Phase 0 also picked the signal by measurement rather than
    // by assumption: the stance/swing load ratio is 1.368 on **hip1**, 1.124 on hip2 and
    // 1.011 on the knee.  hip2 and the knee hold a near-static posture in both phases and
    // say almost nothing; hip1's torque is the ground reaction to the sweep itself.  That
    // is the physically right quantity — it is measured on the very joint the stroke acts
    // on, and it answers "is this leg meeting ground?"
    //
    // The gate is normalized to a MEAN OF 1 across legs, so it REDISTRIBUTES thrust toward
    // the legs that have purchase rather than merely attenuating it.  That matters: flat
    // speed has been pinned across every timing lever tried, and a gate that could only
    // subtract would read as "slower" and teach us nothing.
    //
    // What this does NOT do: it scales the propulsion term only, never the steering term.
    // heading_bearing_hold is a promoted lever riding the same skid-steer channel, and
    // gating a heading controller by load would corrupt it.
    // ------------------------------------------------------------------
    double  stroke_load_gain_ = 0.0;            // 0 = off, gate ≡ 1, byte-identical
    std::vector<float> stroke_load_ema_;        // per-leg |τ_hip1|, lightly smoothed
    std::vector<float> stroke_gate_;            // diag: the gate actually applied
    float   stroke_gate_mean_   = 1.0f;         // diag: 1.0 with 0 spread ⇒ never fired
    float   stroke_gate_spread_ = 0.0f;
    void update_stroke_load_gate();
    // ~5-tick smoothing: rejects servo noise without blurring the ~26-tick step cycle.
    static constexpr float kStrokeLoadAlpha = 0.2f;
    static constexpr float kStrokeGateMin   = 0.0f;   // saturation guards, not the mechanism
    static constexpr float kStrokeGateMax   = 2.0f;

    // ------------------------------------------------------------------
    // STROKE-TO-STEP LOCK (2026-07-27).  The stroke has never had an observation of its
    // own step.
    //
    // Phase 0 measured three per-leg clocks that nothing forced to agree: the stroke rides
    // `L.phase` off the KNEE (22-24 ticks, `phase_joint` defaults -1), the real step is
    // 26-30 ticks (`foot_contact`), and the incumbent foot-height detector chatters at
    // 12-15.  Stroke and step beat at ~2-2.5 s -- the operator's "occasionally it works
    // very well, then synchronization is lost", as a number -- and the fraction of STANCE
    // spent in the stroke's positive half is 0.512 against 0.513 over SWING.  Push
    // direction is statistically INDEPENDENT of whether the foot is on the ground.
    //
    // The fix, per the ledger's re-use context for `phase_joint=0`: derive the phase the
    // stroke rides from a signal the stroke does NOT drive.  A touchdown-referenced step
    // clock,
    //
    //     phi_step = 2*pi * (ticks since touchdown) / EMA(inter-touchdown interval)
    //
    // puts phi = 0 AT touchdown, so `stroke_phase` finally selects where in the step the
    // push lands.  This is the same clock SynergyTimer.cpp:305-313 already runs (that
    // module is not in the picrawler graph, so the algorithm is ported, not the module).
    //
    // WHY THIS IS NOT `phase_joint=0` AGAIN.  That refutation was a WIRING failure: the
    // stroke drove the very hip1 it read `atan2(velocity, deviation)` from, closing an
    // algebraic self-excited oscillator, and locomotion collapsed at all four offsets of a
    // full-circle sweep.  Here the stroke influences touchdown only THROUGH THE WORLD --
    // the foot leaves the ground and comes back -- which is the loop a walking animal
    // actually has.  The premise of that refutation measured POSITIVE (step_bal 0.30 ->
    // 0.41-0.58), so only its wiring was ever in question.
    //
    // ISOLATION.  `L.phase` feeds six consumers (Kuramoto coupling, the stroke, the
    // amplitude homeostat, prop-credit, `rhythm_gains`, the alignment diagnostics), and
    // `phase_joint` moved ALL of them at once -- part of why its collapse taught us so
    // little.  `L.step_phase` is a SEPARATE field consumed at the stroke site only.  One
    // lever, one consumer.
    //
    // WARNING FOR WHOEVER READS THE RESULT.  With phi referenced to touchdown, `td_plv`
    // -> ~1.0 and `pos_stance` becomes a deterministic function of `stroke_phase` and the
    // duty factor.  They are then CONSUMER VERIFICATION (CLAUDE.md 3.2 rule 5), not
    // evidence -- judge this lever behaviourally, and on `mv_stance`/`mv_swing` below,
    // which are computed on the ACHIEVED hip1 motion rather than the commanded waveform.
    // ------------------------------------------------------------------
    //   0 = legacy L.phase (byte-identical) | 1 = contact touchdown | 2 = hip1-load touchdown
    double  stroke_phase_src_    = 0.0;
    double  step_phase_debounce_ = 2.0;    // ticks of consistent contact before a touchdown counts
    double  step_period_alpha_   = 0.2;    // EMA rate on the inter-touchdown interval
    double  step_period_min_     = 8.0;    // sanity rails (the measured step is 26-30)
    double  step_period_max_     = 200.0;
    // Proportional pull toward phi=0 at touchdown.  BodyRhythmTracker's value (0.10),
    // and for its reason: a phase that DRIVES a continuous command must not jump.  1.0 =
    // the hard snap that was measured to collapse the gait -- kept reachable so that
    // refutation stays reproducible rather than becoming folklore.
    double  step_phase_lock_     = 0.10;
    void    update_step_phase(uint64_t tick_id);
    static constexpr float kTwoPi = 6.28318530717958647692f;
    // Frequency low-pass, matching BodyRhythmTracker's omega_lp default: omega drifts
    // SMOOTHLY toward the measured period so the driven waveform never breaks.
    static constexpr float kStepOmegaLp = 0.05f;
    float   step_lock_frac_ = 0.0f;        // diag: fraction of legs with a locked step clock
    // Mean |phase error at touchdown|, radians.  The lock-quality read that does NOT go
    // tautological: the pull is partial, so an entrained clock drives this toward 0 while
    // a clock fighting the body's real rhythm sits high.
    double  step_td_err_acc_ = 0.0; int64_t step_td_err_n_ = 0;
    // Count of lock<->fallback transitions across all legs.  Intermittent locking swaps the
    // stroke's phase reference mid-gait, and each swap is exactly the driven-command
    // discontinuity the soft PLL exists to prevent.
    int64_t step_lock_flips_ = 0;
    float   step_period_mean_ = 0.0f;      // diag: mean measured step period, ticks
    // Per-leg load EMA for src=2 (touchdown = |tau_hip1| crossing up through its own mean).
    // Separate from stroke_load_ema_ so the purchase gate and the phase source can never
    // be entangled by a shared filter state.
    std::vector<float> step_load_ema_;

    // The honest, NON-tautological mechanism instrument for the lock: the same stance/swing
    // split, computed on the leg's ACHIEVED fore-aft motion (sgn * delta hip1, i.e. L.x[2])
    // instead of on the commanded sin(theta).  A working lock means the foot really does
    // travel backward-relative-to-body while planted and forward while airborne, which no
    // amount of re-referencing the command can fake.  Accumulated in the diagnostic block
    // so an instrumented CONTROL arm reports it too -- a diag accumulated inside a lever's
    // own block reads 0 on the control, which is exactly the arm you need to compare with.
    double  ga_mv_stance_ = 0.0; int64_t ga_mv_stance_n_ = 0;
    double  ga_mv_swing_  = 0.0; int64_t ga_mv_swing_n_  = 0;

    // ------------------------------------------------------------------
    // FOOTFALL RASTER (2026-07-27) -- pure observation, for the live inspector.
    //
    // The operator can see gait quality long before any aggregate metric moves, and the
    // stroke-to-step relation is the specific thing that is invisible in numbers and
    // obvious in a picture.  This is a ring of per-tick bits the UI renders as a
    // Hildebrand plot: contact per leg, the stroke's sign per leg, and the incumbent
    // foot-height detector per leg (which makes its documented ~2x-per-step chatter
    // visible for the first time).
    //
    // WHY A RING RATHER THAN CLIENT-SIDE ACCUMULATION: DiagPublisher throttles each
    // subscription to a target Hz (default 30) against a ~52 tick/s brain, so a widget
    // accumulating what it receives would alias exactly at the touchdown edges that matter.
    // The ring is sampled every tick and shipped whole, so the picture is exact regardless
    // of the stream rate.
    //
    // Feeds no command, draws no randomness: byte-identity is structural.  Still gated
    // (default 0 = block skipped) and still verified by measurement, per CLAUDE.md.
    // ------------------------------------------------------------------
    double  gait_raster_diag_ = 0.0;                  // 0 = off (block skipped)
    static constexpr int kRasterLen = 512;            // ~10 s at 52 tick/s
    std::vector<uint16_t> raster_;                    // bits 0-3 contact, 4-7 stroke sign, 8-11 in_swing
    int     raster_head_ = 0;                         // next write index (ring)
    int64_t raster_n_    = 0;                         // total ticks written (< kRasterLen = partial)
    void    update_gait_raster();

    // ------------------------------------------------------------------
    // SWING TUCK (2026-07-27) — the MIRROR of stance_lift, and the gate `hip2_tuck_target`
    // was missing.  That parameter is an UNGATED postural rest-override applied to every
    // leg all the time, and it is in the refuted table ("didn't crouch + destabilized").
    // Doctrine §5: *when a bias fails, ask what state should have gated it before concluding
    // the idea is dead.*  The state is SWING.  It is the same shape that made stance_lift
    // work: a blind knee bias kills the gait, the same bias gated to planted legs only gave
    // belly-up and ~20 % faster walking.
    //
    //   stance_lift : KNEE bias on PLANTED legs  (push the body up off the feet)
    //   swing_tuck  : hip2 + KNEE bias on LIFTED legs (fold the limb inboard)
    //
    // The error it minimizes: sweeping an extended limb forward transfers angular momentum
    // into the chassis.  Retracting the limb cuts its yaw inertia about the body axis, so
    // the disturbance is prevented rather than corrected after the fact by heading-hold.
    //
    // Note the inversion that makes hip2 usable here at all: hip2's known liability is that
    // it rotates feet OFF THE GROUND and loses traction (this is why the height-homeostat
    // lift wrecked climbing).  During swing the foot is already off the ground, so there is
    // no traction to lose — the state gate turns hip2's failure mode into its use.
    //
    // GATED ON TRUE CONTACT, deliberately, not on the foot-height detector that stance_lift
    // uses: that detector was measured firing ~2x per real step (12-15 ticks vs a 26-30 tick
    // step), and a swing tuck on a chattering gate would retract the limb MID-STANCE, i.e.
    // lift a loaded foot — exactly the traction-loss failure above.  Wiring true contact as
    // a *swing-phase* gate is refuted (§2), but for a consumer that wanted gait PHASE; this
    // one wants "is the foot off the ground", and the ledger's re-use context names the case:
    // "for a consumer that truly needs contact (load distribution, step-over foot placement)".
    // Both signs are left to the sweep rather than assumed, as stance_lift's was.
    // ------------------------------------------------------------------
    // ------------------------------------------------------------------
    // TIBIA-PLUMB REFLEX (2026-07-27) — the operator's inverse-kinematics framing, written
    // as an ERROR rather than a trajectory.
    //
    // hip2 and the knee are a planar 2-link arm (femur L2 = 53.6 mm, tibia L3 = 75.5 mm).
    // With hip2 pinned at its horizontal rest, the KNEE ALONE must produce both the foot's
    // height and its fore-aft position — so the foot is forced along a circular arc about
    // the knee axis, and the shank has to sweep through a large angle to translate the foot
    // at all.  Measured over a whole run (arena, n=3, 1032 leg-frames): hip2 sits at
    // −3.6° ± 4.4 and never leaves neutral, while the tibia swings to 37.5° ± 15.3 off
    // vertical (the design rest pose is 10°, extremes reach 101°) and the feet plant at a
    // 170 mm radius against a 166 mm total leg reach — straight-legged, maximum moment arm.
    //
    // Rewrite rule (§1): implement the error the behaviour minimizes.  The error is "the
    // shank is off plumb"; the action is hip2.  Whatever the knee does to drive the gait,
    // hip2 rotates to re-plumb the shank — which IS the 2-link coordination that translates
    // the foot at constant height instead of arcing it.  Nothing about timing is specified,
    // so the gait's rhythm and inter-leg pattern stay emergent (§5.7).
    //
    // Why this is not the refuted "learned hip2": that lever LOOSENED hip2's postural spring
    // and hoped HK would find the coordination.  It did not, and got less stable — an
    // unconstrained joint is a wobble dimension, not an IK solver.  Here hip2 is given an
    // OBJECTIVE (a measured error to null), which is the form the doctrine prefers.
    //
    //   θ_tibia_from_vertical = hip2_scale·x[hip2] + x[knee] + offset      [radians]
    // for the picrawler's encoding: hip2 proprio = angle / HIP2_LIMIT(1.40), knee proprio =
    // angle − KNEE_REST(−1.6), so offset = KNEE_REST + π/2 = −0.0292.  Both are KINEMATIC
    // constants of the body, not tuned gains — only `tibia_plumb_gain` is swept.
    // ------------------------------------------------------------------
    double  tibia_plumb_gain_  = 0.0;           // 0 = off (byte-identical)
    double  tibia_plumb_scale_ = 1.40;          // = HIP2_LIMIT (body kinematic constant)
    double  tibia_plumb_offset_ = -0.0292;      // = KNEE_REST + pi/2 (body kinematic constant)
    // Mean |θ_tibia| in radians, accumulated in the DIAGNOSTIC block so every instrumented
    // arm reports it — including ones with the plumb reflex off, which is the comparison.
    double  ga_tib_acc_ = 0.0; int64_t ga_tib_n_ = 0;
    double  swing_tuck_hip2_ = 0.0;             // 0 = off (byte-identical)
    double  swing_tuck_knee_ = 0.0;
    float   swing_tuck_frac_ = 0.0f;            // diag: frac of leg-ticks the bias applied
    int64_t swing_tuck_hits_ = 0, swing_tuck_ticks_ = 0;
    // 2026-06-12 — directional propulsion drive on hip1 (the fore-aft joint).
    // The knee coupling locks step TIMING but the hip1 stroke DIRECTION stays
    // HK-driven and pointed tangentially → the four thrusts sum to a torque
    // (spin-in-place).  A drive phase-locked to each leg's step phase aligns the
    // strokes: stroke_signs sets the per-leg push direction (parallel = forward,
    // tangential = turn), steer adds a left/right differential for steering.
    double  stroke_gain_  = 0.0;                   // hip1 fore-aft drive amplitude (0 = HK-only hip1)
    double  stroke_phase_ = 0.0;                   // phase offset knee→hip1 (push timing vs lift)
    double  steer_        = 0.0;                    // left/right differential (turn rate; 0 = straight)
    std::vector<double> stroke_signs_ = {1.0, -1.0, 1.0, -1.0};  // per-leg hip1 stroke direction (forward guess)
    // 2026-07-22 — per-leg propulsive-credit homeostat (functional L/R propulsion
    // balance).  Each leg's FUNCTIONAL contribution to the fore-aft power stroke —
    // the hip1 motion component phase-aligned with the stroke waveform — is tracked
    // as a credit; a leg below the group mean ("dragging": planted but static) gets
    // a self-limiting boost in its stroke direction so it pulls its weight and L/R
    // propulsion equalizes (straighter travel).  The credit is FUNCTIONAL (phase-
    // aligned), NOT amplitude/RMS — the measure the refuted symmetry levers missed.
    // 0 = off (byte-identical).  Boost fades to 0 as the credit deficit closes.
    double  propulsion_balance_gain_ = 0.0;
    float   prop_credit_mean_ = 0.0f;   // group-mean credit (recomputed each tick; telemetry)
    // 2026-06-12 — active balance (vestibular reflex).  The FIRST perception input
    // to the controller, and the right one for the fast loop (vestibulospinal is
    // spinal-level, not cortical).  Reads chassis tilt direction and pushes the
    // low-side legs' hip2 (lift) to level the body — equalizing leg load → equal
    // basin depth → straighter, fewer stalls.  Fixed feedback law, no learning.
    double  balance_gain_ = 0.0;                   // vestibular balance strength (0 = off; sign tunable)
    std::string tilt_topic_ = "reality.proprio.tilt";  // 4-D [sin(pitch),cos(pitch),sin(roll),cos(roll)]
    float   tilt_pitch_ = 0.0f;                    // latest sin(pitch) (signed fore-aft tilt)
    float   tilt_roll_  = 0.0f;                    // latest sin(roll)  (signed left-right tilt)
    // 2026-06-12 — per-leg amplitude homeostat.  HK is BISTABLE (deep large-cycle
    // basin vs shallow twitch basin); which a leg lands in is set by init/history,
    // so behavior is path-dependent (slider order matters) AND asymmetric (one leg
    // stalls → wander).  A slow integral regulator drives each leg's oscillation
    // amplitude toward a shared target by adapting a per-leg output gain — kills
    // the asymmetry AND the path-dependence (converges to target regardless of
    // starting basin).  Fixed-law homeostasis, not learning the gait.
    double  amp_homeo_gain_ = 0.0;                 // integral rate (0 = off)
    double  amp_target_     = 0.4;                 // target oscillation amplitude (phase-vector magnitude)
    // 2026-06-12 — heading-rate regulator ("go straight" reflex).  Amplitudes are
    // equalized but the body still TURNS (no heading feedback), so residual stroke
    // asymmetry accumulates into yaw drift = the remaining wander.  Reads the
    // body's signed yaw rate (IMU) and feeds it into steer to counter unwanted
    // turning: steer becomes a turn-RATE command (0 = hold heading).  Second
    // vestibular channel (yaw), like balance was (tilt).
    double  heading_gain_ = 0.0;                    // yaw-rate feedback strength (0 = off; sign tunable)
    std::string imu_topic_ = "reality.proprio.imu"; // 4-D [sin yaw, cos yaw, fwd_v, ang_v]
    float   yaw_rate_ = 0.0f;                       // latest signed yaw rate (IMU index 3)
    // 2026-07-23 — HEADING-HOLD desire (the robust go-straight lever): damp the body
    // yaw rate with a per-side hip1 differential IN-PHASE with the power stroke, so it
    // composes with propulsion-balance / the emergent gait instead of fighting it (unlike
    // the old heading_gain, which fed steer-magnitude and circled embed).  A "hold the
    // bearing" prior — not a reflex kick.  0 = off; sign tunable.
    double  heading_hold_gain_ = 0.0;
    float   yaw_rate_ema_ = 0.0f;                   // smoothed yaw rate for the heading-hold
    // 2026-07-23 — BEARING-HOLD (the real go-straight lever).  heading_hold_gain_ damps
    // the yaw RATE, which resists spinning but goes to zero once the body has drifted and
    // stopped rotating → it never corrects the accumulated bearing (proven: turn-std flat
    // across a gain sweep).  This term instead integrates the body's OWN yaw rate into a
    // dead-reckoned bearing (heading_bearing_, relative to spawn — Markov-compliant, a real
    // gyro does exactly this) and drives it back to 0.  An error that GROWS with drift and
    // keeps pushing until corrected — integral authority the rate term lacks.  0 = off.
    double  heading_bearing_hold_gain_ = 0.0;
    float   heading_bearing_ = 0.0f;                // integrated yaw rate = bearing rel. to spawn (π-units)
    // 2026-06-13 — PERCEPTION → STEERING (the active-inference closure).  The robot
    // perceives the egocentric bearing to a target (target_compass) and ACTS to
    // minimize it (steer until the target is dead-ahead) — Friston's loop closed
    // through the body.  Perception decides WHERE; homeokinesis decides HOW.  This
    // is also the absolute-heading reference the yaw-rate regulator lacked.
    double  nav_gain_ = 0.0;                         // steer-toward-target strength (0 = off)
    std::string nav_topic_ = "reality.proprio.target_compass";  // 2-D egocentric unit vec to target
    float   tc_x_ = 0.0f;                            // target_compass lateral component (steer error)
    float   tc_y_ = 0.0f;                            // target_compass forward component
    // 2026-06-15 — cognitive steer: a scalar ActionOut (e.g. ActionDecoder's
    // learned left/straight/right) biases the cell's 2-flagella differential
    // (n_legs=1, motor_dim=2).  The slow cognitive critic directs the alive
    // homeokinetic swimmer.  0 gain / empty topic = off (picrawler untouched).
    double  cog_steer_gain_  = 0.0;
    std::string cog_steer_topic_ = "";
    float   cog_steer_ = 0.0f;                        // latest cognitive steer (accel, normalized)
    int     cog_steer_msgs_ = 0;                      // # cog.steer messages received (diag)
    void handle_cog_steer(MessagePtr payload);
    // 2026-06-18 — cognitive THRUST channel (cell, n_legs=1/motor_dim=2): a second
    // learned scalar driving COMMON-mode (forward/reverse/pause), so the actor can
    // move toward higher scent — the action that actually changes its preferred
    // observation (proximity).  Mirrors cog_steer (differential).  Empty = off.
    double  cog_thrust_gain_  = 0.0;
    std::string cog_thrust_topic_ = "";
    float   cog_thrust_ = 0.0f;
    int     cog_thrust_msgs_ = 0;
    void handle_cog_thrust(MessagePtr payload);
    // 2026-06-15 Phase 6.9.A — "boredom of being stuck" escape (Playful Machine
    // #1/#2).  A frozen sensorimotor loop (pinned at a wall) is perfectly
    // predictable; DistressDrive reports it on cognition.boredom (0..1).  Here,
    // boredom ESCALATES undirected DIFFERENTIAL (heading) noise so the
    // differential_paddler samples new headings until it breaks free, and FADES
    // the cog steer (which was driving into the wall) so the playful substrate
    // takes over.  Self-terminating: free → perception flows → boredom → 0.
    // Cell-only (n_legs==1, motor_dim==2); 0 gain / empty topic = off
    // (picrawler / Stage-1 bit-identical).  NOT a turn-away reflex (random dir).
    std::string boredom_topic_     = "";
    double  boredom_noise_gain_    = 0.0;   // amplitude of the differential escape turn at boredom=1
    float   boredom_               = 0.0f;  // latest boredom level [0,1]
    // Held escape turn: a random rotation direction sustained for a window, so the
    // heading SWEEPS (and the forward beat peels the bug off the wall) instead of
    // diffusing under per-tick white noise.  Resampled each window → still
    // undirected (random sign), just temporally correlated.  ~0.5 s window.
    float   boredom_esc_held_      = 0.0f;
    int     boredom_esc_ticks_     = 0;
    int     boredom_hold_ticks_    = 30;    // reorientation timescale (matches body stuck-pulse)
    void handle_boredom(MessagePtr payload);
    // ③ curiosity direction: interest (DistressDrive cognition.interest) steers the
    // escape — RUN forward when interest is high (open/scent-rich ahead), TUMBLE
    // (held random turn) when low (boring).  Empty topic / interest 0 = pure
    // tumble (the prior undirected escape).
    std::string interest_topic_    = "";
    float   interest_              = 0.0f;
    float   interest_ema_          = 0.0f;   // short baseline → RISING-gradient klinokinesis
    bool    interest_ema_init_     = false;
    void handle_interest(MessagePtr payload);
    // 2026-06-16 — hunger-modulated foraging.  hunger (1-energy) scales the
    // food-ward nav steer (FORAGE when hungry: the bug had no food-attraction →
    // drifted past food), and compounds with boredom-DURATION to escalate the
    // escape ("do more until something happens" + hunger → desperation).
    std::string hunger_topic_      = "";    // reality.proprio.hunger (1-energy); empty = nav not hunger-gated
    float   hunger_                = 0.0f;
    double  boredom_escalation_rate_ = 0.0; // escape amplitude growth per bored tick (0 = no escalation)
    int     boredom_streak_        = 0;     // ticks boredom has stayed high (resets when it drops = escaped)
    void handle_hunger(MessagePtr payload);
    // 2026-06-13 — chassis-height homeostat (stand-higher reflex).  The G6DOF
    // springs pull joints to neutral and the freeplay deadband lets the body SAG;
    // the postural reflex defends a joint-ANGLE pose, not a HEIGHT, so under spring
    // sag the chassis settles low.  This closes the loop on actual chassis height:
    // a slow integral drives a tuck-deepening bias into the knee (the spider-lift
    // joint — more tuck suspends the chassis higher) toward a SELF-DISCOVERED
    // setpoint = height_k · (tallest height the body has reached).  No hand-set
    // height; the body discovers how tall it can stand and defends it.  Fixed-law
    // homeostasis, reward-free, twin of the amplitude homeostat.
    double  height_homeo_gain_ = 0.0;                // integral rate (0 = off)
    double  height_k_          = 0.9;                // fraction of discovered max to defend
    std::string height_topic_  = "reality.proprio.chassis_y_norm";  // 1-D chassis height (norm [0,1])
    float   chassis_h_     = 0.0f;                    // latest chassis height
    float   chassis_h_ema_ = 0.0f;                    // smoothed height (spike-robust)
    float   chassis_h_max_ = 0.0f;                    // running max of the EMA (the discovered ceiling)
    float   height_bias_   = 0.0f;                    // integral output → knee tuck-deepen command
    bool    chassis_h_seen_ = false;                  // EMA seeded
    // 2026-06-13 — PANIC PATHWAY (Stage 2).  Subscribe to the body's distress
    // signal (reality.proprio.distress = wedge severity) and, above threshold,
    // neuromodulate the EXISTING knobs to break free: decouple the legs
    // (coupling→0), kill the futile forward stroke, ramp explore_noise + motor_gain
    // → high-amplitude decoupled FLAILING.  Arousal-scaled with hysteresis.
    // Reward-free — this is amplified HomeokineticExploration triggered by
    // predictive-model degradation (the body can't move what it commands).  The
    // switch is a smooth subsumption: panic_ multiplies into the gait knobs, so
    // normal walking is untouched at distress=0 and fully overridden at panic=1.
    double  panic_on_         = 0.5;    // distress to ENGAGE (hysteresis high)
    double  panic_off_        = 0.25;   // distress to DISENGAGE (hysteresis low)
    double  panic_strength_   = 1.0;    // overall effect scale (0 = panic OFF)
    double  panic_noise_      = 0.4;    // explore_noise ADDED at full panic (symmetry-break only)
    double  panic_motor_mult_ = 1.8;    // motor_gain multiplier at full panic
    // 2026-06-13 — PUSH REFLEX.  Per-tick noise is low-passed by the servo/G6DOF
    // dynamics → only jitter (operator obs: hip1 moves, knees/hip2 don't cover
    // range, body stays wedged).  A coherent LOW-frequency, FULL-amplitude pump on
    // hip2+knee (per-leg staggered) passes through the dynamics → big leg
    // excursions that lever the body off the obstacle.  This is the real escape
    // force; noise just breaks symmetry.  Engages with panic (×pe).
    double  panic_push_amp_ = 1.2;      // pump amplitude (≥1 drives joints to full range)
    double  panic_push_hz_  = 0.8;      // pump frequency (low → survives the servo low-pass)
    float   panic_phase_    = 0.0f;     // push-oscillator phase (advances each tick)
    std::string distress_topic_ = "reality.proprio.distress";  // 1-D wedge severity [0,1]
    std::string lateral_topic_  = "reality.proprio.lateral_v";  // 1-D signed sideways-slip velocity
    float   distress_      = 0.0f;                    // latest distress (wedge severity)
    float   panic_         = 0.0f;                    // smoothed panic level [0,1] (telemetry)
    bool    panic_latched_ = false;                   // hysteresis state

    // ---- Gate 0 reset-masking instrumentation (L-1a) -------------------------
    // The body's leg-phase + EMA continuity survives the fall+respawn cycle, so a
    // coherence/TLE trend measured across a reset is fake (the "reset artifact").
    // Subscribe to the body's disruption events — events.miss (catastrophic fall)
    // and events.reset (teleport/respawn) — and expose an honest, reset-masked
    // view: a cumulative count, ticks-since-last (for masking the transient), and
    // a slow rate EMA that FALLS as the upright prior stops the body falling.
    // Reward-free instrumentation — it does not touch the control law.
    void handle_event(std::string_view topic, MessagePtr payload);
    uint64_t reset_count_         = 0;      // cumulative miss/reset disruptions
    uint64_t ticks_since_reset_   = 0;      // ticks since the last disruption (0 on the reset tick)
    float    reset_rate_ema_      = 0.0f;   // slow EMA of the per-tick disruption indicator → falls when stable
    bool     reset_rate_init_     = false;  // EMA seeded
    bool     reset_hit_this_tick_ = false;  // set by handle_event, consumed in tick() (intra-tick, NOT serialized)

    // ---- Objective socket (L-1b, §1.1) — a soft posture target fed by an external
    // generator (PosturalPrior now; KeyframeGait / nav / manipulation later, arbited).
    // The controller descends toward it (surprise-to-descend, §2.4) — NOT additive; w=0
    // or no objective → byte-identical HK.  Transient input (repopulated each message,
    // not serialized).
    void handle_objective(int leg, MessagePtr payload);
    std::vector<Eigen::VectorXf> obj_target_;   // per-leg target joint positions (motor_dim)
    std::vector<float>           obj_weight_;    // per-leg weight w = PredictionToken.confidence ∈ [0,1]
    std::vector<char>            obj_seen_;      // per-leg: an objective has arrived (char, not vector<bool>)

    // ---- L-1b velocity objective (§the propulsive push) ----
    // A phase-indexed VELOCITY target on objective.velocity.<leg> (KeyframeGait's vel map).
    // A second learned feed-forward Cvel is trained to reduce the velocity error (v*−ẋ) at the
    // command phase → the body keeps moving THROUGH the pose (propulsion), where the posture
    // objective (above) only holds it AT the pose.  Empty socket = Cvel stays 0 = byte-identical.
    void handle_objective_vel(int leg, MessagePtr payload);
    std::vector<std::string>     velocity_objective_topics_; // optional per-leg PredictionToken velocity-target topics; empty = OFF
    std::vector<Eigen::VectorXf> obj_vel_target_;// per-leg target joint velocities (motor_dim)
    std::vector<float>           obj_vel_weight_; // per-leg weight w ∈ [0,1]
    std::vector<char>            obj_vel_seen_;   // per-leg: a velocity objective has arrived

    // ---- Per-leg working state (sized n_legs_) ----
    struct Leg {
        bool                initialized = false;
        int                 n           = 0;     // state dim
        Eigen::MatrixXf     A;                    // n x m  (motor → sensor)
        Eigen::VectorXf     b;                    // n
        Eigen::MatrixXf     C;                    // m x n  (sensor → motor)
        Eigen::MatrixXf     Cphi;                 // m x 2  learned phase-conditioning (posture feed-forward)
        Eigen::MatrixXf     Cvel;                 // m x 2  learned phase-conditioning (velocity feed-forward / propulsive pump)
        Eigen::Vector2f     prev_phi_ctx{0.0f, 0.0f}; // [cos φ, sin φ] at command time (for the Cphi/Cvel update)
        Eigen::VectorXf     h;                    // m
        Eigen::VectorXf     x;                    // latest sensor (n)
        Eigen::VectorXf     prev_x;               // sensor at command time (n)
        Eigen::VectorXf     prev_y;               // last motor command (m)
        Eigen::VectorXf     rest_pos;             // standing pose captured at spawn (m pos targets)
        bool                rest_captured = false;
        bool                have_prev   = false;
        bool                fresh       = false;  // new proprio arrived this tick
        int64_t             steps_seen  = 0;      // proprio frames processed (warmup counter)
        std::mt19937        babble_rng;           // per-leg babble stream
        float               tle_ema     = 0.0f;
        float               gain_ema    = 1.0f;
        float               outmag_ema  = 0.0f;
        float               sat_ema     = 0.0f;   // mean tanh² of operating point (saturation telemetry)
        float               knee_ema    = 0.0f;   // slow mean of knee pos (phase reference)
        float               phase       = 0.0f;   // estimated oscillator phase (rad)
        float               amp_ema     = 0.0f;   // slow estimate of oscillation amplitude
        float               amp_gain    = 1.0f;   // homeostat output gain (regulated toward amp_target)
        float               hip1_dc     = 0.0f;   // slow DC mean of hip1 pos (propulsive-credit baseline)
        float               prop_credit = 0.0f;   // EMA of phase-aligned propulsive stroke (fwd contribution)
        // ---- stroke-to-step lock (stroke_phase_src > 0).  A phase the stroke does not
        // drive: phi = 0 AT touchdown, advancing on the leg's own measured step period.
        float               step_phase   = 0.0f;  // radians, [0, 2pi)
        float               step_omega   = 0.0f;  // rad/tick, low-passed toward 2pi/step_per_ema
        float               step_per_ema = 0.0f;  // EMA of the inter-touchdown interval, ticks
        int64_t             last_td_tick = -1;    // tick of the most recent accepted touchdown
        int32_t             td_count     = 0;     // accepted touchdowns (>= 2 ⇒ locked)
        int32_t             td_run       = 0;     // consecutive ticks of the current contact state
        bool                td_contact   = true;  // debounced contact state
        bool                step_locked  = false; // false ⇒ the stroke falls back to L.phase
    };
    std::vector<Leg> legs_;

    static constexpr float kTeleEmaAlpha   = 0.02f;
    static constexpr float kKneeEmaAlpha   = 0.01f;   // slow mean for the phase reference
    static constexpr float kPhaseVelScale  = 15.0f;   // balances knee Δ vs (pos−mean) in atan2
    static constexpr float kAmpEmaAlpha    = 0.01f;   // slow amplitude estimate for the homeostat
    static constexpr float kPropCreditAlpha = 0.01f;  // ~100-tick propulsive-credit EMA (functional balance)
    static constexpr float kAmpGainMin     = 0.1f;
    static constexpr float kAmpGainMax     = 5.0f;
    static constexpr float kAmpSeekMin     = 0.15f;   // amp_target floor for the CoT search (avoid motion collapse)
    static constexpr float kAmpSeekMax     = 0.60f;   // amp_target ceiling for the CoT search
    static constexpr float kHeightEmaAlpha = 0.01f;   // slow height estimate (spike-robust)
    static constexpr float kYawRateEmaAlpha = 0.15f;  // ~7-tick yaw-rate smoothing for the heading-hold
    static constexpr float kBearingIntegDt  = 1.0f/60.0f; // integrate normalized yaw rate → bearing (1.0 ≈ π rad = half turn)
    static constexpr float kBearingClamp    = 4.0f;   // saturate the bearing integral (≈2 turns) so the term can't explode
    static constexpr float kFwdProgressAlpha = 0.01f; // slow forward-velocity EMA (~100-tick window) for stall detection
    static constexpr float kStuckVelThresh  = 0.012f; // fwd_v EMA below this ≈ not progressing (embed walks well at ~0.04)
    static constexpr int   kStuckWindowTicks = 300;   // sustained stall (5 s) before the exploration boost engages
    static constexpr float kStuckBoostRise  = 1.0f/600.0f; // ramp boost to full over ~10 s of sustained stall
    static constexpr float kStuckBoostDecay = 1.0f/120.0f; // decay boost over ~2 s once moving (faster off = self-terminating)
    // progress→COMMIT (lever C): the flow threshold is well above the stall threshold so
    // commit and stuck are disjoint (a dead-band between them = neither fires).
    static constexpr float kCommitVelThresh = 0.030f; // fwd_v EMA above this ≈ genuinely moving (embed cruises ~0.04)
    static constexpr int   kCommitWindowTicks = 180;  // sustained flow (3 s) before commit engages (faster than stuck: seize the push)
    static constexpr float kCommitBoostRise = 1.0f/240.0f; // ramp commit to full over ~4 s of sustained progress
    static constexpr float kCommitBoostDecay = 1.0f/90.0f;  // decay over ~1.5 s once progress falls (release quickly, re-explore)
    // forward-FLOW homeostat (lever D): predictability-weighted flow amplifier.
    static constexpr float kFlowEmaAlpha    = 0.02f;  // ~50-tick EMA of fwd_v (flow magnitude + volatility base)
    static constexpr float kFlowVelNorm     = 0.05f;  // fwd_v that counts as "full" forward flow (embed cruises ~0.04)
    static constexpr float kFlowVolK        = 4.0f;   // volatility penalty weight in 1/(1+k·vol): mild so the flow MAGNITUDE leads (raw fwd_v oscillation inflates vol; magnitude is the cleaner discriminator)
    static constexpr float kCommitStrokeFrac = 0.30f; // stroke thrust added per unit commit_amt (gain·boost) — +30% at gain 1, full commit
    static constexpr float kHeightBiasMin  = -0.5f;   // allow slight relax below neutral lift
    static constexpr float kHeightBiasMax  =  1.5f;   // cap lift authority
    static constexpr float kHeightLiftSign = +1.0f;   // hip2 command dir that RAISES chassis (flip if inverted)
    static constexpr float kHeightMoveSuppVel = 0.025f; // fwd_progress_ema at which the height defense fully fades (height is a STANDING reflex; a lift bias loses traction while walking/climbing — belly must ride low on an incline)
    static constexpr float kPanicRampAlpha = 0.04f;   // smoothing of panic_ toward its hysteresis target
    static constexpr float kResetRateAlpha = 1.0f / 600.0f;  // Gate 0: ~600-tick (~10s@60Hz) smoothing of the disruption-rate EMA (a measurement constant, not a behavioral knob)
};

} // namespace ogma
