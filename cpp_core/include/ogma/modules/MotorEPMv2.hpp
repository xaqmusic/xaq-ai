#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  MotorEPMv2 — the PROACTIVE motor layer.
//
//  STAGE 0 (2026-08-06): this file is a MECHANICAL COPY of MotorEPM.  Only the
//  class name and the registered type name differ.  Nothing else may change in
//  this commit.
//
//  WHY A SEPARATE MODULE (operator's call).  MotorEPM is the benchmark for
//  every arm we run, and it carries ~60 gain-0-guarded levers, each of which
//  must be byte-identical at 0 against every combination of the other 59.
//  Editing the control in order to test alternatives to the control is a
//  conflict of interest, and this session produced the concrete failure mode:
//  changing commit_prec_gain's units mid-session DESTROYED the arm that had
//  produced its own +5.7 %, which then could not be re-measured.
//
//  THE INVARIANT THAT MAKES THE A/B TRUSTWORTHY:
//      v2 with every new feature at 0 must be BYTE-IDENTICAL to MotorEPM.
//  Not close — identical.  Verified by scripts_tools/moduledif.py, which was
//  shipped BEFORE any v2 feature existed.  Until that passes, no v2 number
//  means anything, and any divergence is a v2 BUG, never a finding.
//
//  v2 therefore STARTS AS A COPY, never a rewrite: refactoring and behaviour
//  change in one commit is how a substrate becomes unfalsifiable.
//
//  The plan: docs/plans-and-designs/motor_epm_v2_plan.md
//  The evidence: docs/plans-and-designs/motor_layer_is_reactive.md
// ─────────────────────────────────────────────────────────────────────────────

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

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class MotorEPMv2 : public Module {
public:
    MotorEPMv2();
    ~MotorEPMv2() override;

    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;

    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;
    nlohmann::json diag_lite() const override;   // live viz: motor-TLE + self-model + cog drive
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
    void handle_intent(MessagePtr payload);
    void handle_goal_bearing(MessagePtr payload);
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
    // 2026-08-02 (import I1 from the Playful Machine analysis): self-exciting
    // controller init.  0 = off → legacy small-random init, byte-identical.
    double  c_init_      = 0.0;                  // added to C(j,3j), the own-joint position feedback
    // Import I3: 0 = hard clamp of the assembled command (historical), 1 = tanh squash.
    double  cmd_squash_  = 0.0;
    int64_t base_seed_   = 1234;                 // per-leg seed = base ^ leg
    // 2026-06-12 — anti-freeze additions (the bare metric-gradient update
    // saturated tanh in ~2 s then froze: g'→0 kills every term of ΔC).
    int64_t babble_ticks_ = 200;                 // motor-babble warmup: model learns, controller idle
    double  babble_scale_ = 0.3;                 // amplitude of babble commands during warmup
    double  sat_lr_       = 0.02;                 // anti-saturation rate (surrogate for dropped ∂G term)
    // ---- 2026-08-03 · IMPORT I2: the REAL ∂G term, and the bound it requires ---------
    // Our HK update holds G fixed in the metric gradient (the Der-Martius approximation)
    // and patches the omission with sat_lr, a hand-set constant standing in for a
    // derivative.  PM's sos_avggrad.cpp carries the term in closed form:
    //     epsrel = diag(C·Q·A) ⊙ g' · 2 · sense ;  C_update -= (epsrel ⊙ y)·xᵀ
    // It is the CONFINING half of the homeokinetic objective — what stops the loop
    // collapsing into the degenerate "predict nothing, move nothing" minimum, which is
    // the decay measured at 40k (step rate 12.0 → 5.2 → 2.95).
    // sense = 0 → off, byte-identical.  PM: hexapod 1.5, zoo's Sox generator 4.
    double  sense_        = 0.0;
    // ⚠ MANDATORY COMPANION.  sat_lr is not only an anti-saturation surrogate — it is the
    // ONLY brake on the bias integrator h (h += bias_lr·μ is ungated).  Measured: sat_lr=0
    // alone drives the pre-clamp command to 14.3 and still climbing, three of four seeds
    // taking ZERO steps.  So retiring sat_lr in favour of sense REQUIRES an explicit
    // bound, which PM supplies as a separate `damping` parameter (dog 0.0001, humanoid
    // 0.0001-0.0003) that we have never had.  L2 decay on C and h.  0 = off.
    double  ctrl_damping_ = 0.0;
    // ---- 2026-08-03 · DEP: Differential Extrinsic Plasticity -----------------------
    // ⚠ PROVENANCE: Der & Martius 2015 (PNAS).  This POSTDATES the 2012 sources we hold,
    // so the rule below is reconstructed from the published principle, NOT read from the
    // download.  Treat the implementation as our interpretation, not as their code.
    //
    // Why it is the candidate: every variation of the homeokinetic mechanism we have
    // tested returns the random-phase null for inter-leg coordination (deployed stack,
    // pure HK, whole-body C, belly crawl — all ~0.45 against a null of 0.450).  That is
    // an argument about the MECHANISM, not the tuning.  DEP inverts the premise: instead
    // of maximising sensitivity against an internal model, it AMPLIFIES WHATEVER THE BODY
    // IS ALREADY DOING, by correlating motor derivatives with the sensor derivatives they
    // caused.  Behaviour accumulates into body-resonant modes instead of being explored
    // away from — which is the "settle into an attractor" property HK structurally lacks.
    //
    // It also fits two measurements: HK already puts 44% of |C| mass on the VELOCITY
    // columns (DEP makes derivatives the whole rule), and moving a leg mechanically
    // changes its neighbours' sensors, so on a whole-body C the rule can write cross-leg
    // terms directly rather than waiting for slow mechanical coupling.
    //
    // COST, stated up front: DEP carries no forward model, so the motor loop stops being
    // predictive.  That sits awkwardly with an EPM architecture and is a real tension,
    // not a detail.
    //
    // dep_gain = 0 → off, byte-identical.  Above 0 it REPLACES the HK update of C and
    // becomes the per-motor loop gain (row-normalised), so it is comparable to c_init.
    double  dep_gain_ = 0.0;
    double  dep_alpha_ = 0.05;   // EMA rate of the derivative correlation
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
    // ---- 2026-08-04 · INFERENTIAL COUPLING.  The Kuramoto term above averages the other
    // three legs UNIFORMLY (`c / (n_legs_-1)`).  That divisor is itself a hand-set precision:
    // every leg is trusted exactly equally, forever, which is what makes an otherwise
    // legitimate innate reflex a SCRIPT rather than inference (doctrine 2.3 -- precision is
    // a CONTROLLED variable, and "a designer picking the crossover point is the anti-pattern").
    // Replacing it with a precision-weighted mean over w_j = (amp_j/(tle_j+eps))^k is the
    // LateralVoter's own idiom applied one layer down, and it is exact at k=0.
    //
    // amp_ema in the NUMERATOR is not decoration.  The voter's documented trap
    // (LateralVoter.cpp:80) is that a flat channel is trivially predictable, so tle -> 0 wins
    // maximum trust; the coord-fitness entry hit the same wall and the ledger records that
    // "the activity term is the homeokinetic normalisation that kills both".  A dead leg
    // must get w = 0 here, not w = infinity.
    // ---- 2026-08-04 · L1 NAV SETPOINT ------------------------------------------------
    // The heading PD's P term is `gain * (-heading_bearing_)`: the setpoint is implicitly
    // ZERO, i.e. "hold the bearing you spawned on".  That PD is the best-measured lever in
    // this project (straight 0.05->0.53, net_z variance 0.92->0.07), so a nav layer should
    // STEER it, not replace it.
    //
    // ⚠ WHY NOT `nav_topic`: `nav_on` gates the ENTIRE heading PD off (both P and D) AND the
    // forward facing gate.  Re-entering through nav_gain would throw away the variance
    // collapse that makes the PD worth having, which is exactly what the oracle path does.
    //
    // The producer publishes an EGOCENTRIC unit vector [vx, vy] (vy = forward), so the angle
    // atan2(vx, vy) IS the bearing error and drops straight into the P term.  Unset topic, or
    // no token yet, => falls back to -heading_bearing_ => byte-identical.
    // 2026-08-04 · learned DC heading effort (the missing I term) + paired L/R init.
    double  heading_trim_rate_ = 0.0;              // 0 = off, byte-identical
    double  heading_trim_leak_ = 0.001;            // MANDATORY: windup is this file's failure shape
    double  c_pair_init_       = 0.0;              // 0 = per-leg random init (legacy)
    float   heading_trim_      = 0.0f;
    static constexpr float kHeadTrimMax = 0.5f;    // hard bound on the learned trim
    std::string goal_bearing_topic_;
    float   gb_x_ = 0.0f, gb_y_ = 0.0f;
    bool    gb_seen_ = false;
    int64_t gb_msgs_ = 0;                          // consumer check: did it ever arrive?
    double  couple_prec_gain_ = 0.0;               // 0 = uniform mean (legacy, byte-identical)
    static constexpr float kCouplePrecEps = 1e-4f; // divide guard only; tle_ema runs ~0.24-0.28
    // Numerical guard on the weight RATIO, not a shaping constant: at |k| <= 2 over the
    // measured precision band the clamp never binds, it only catches a frozen/degenerate leg
    // in the negative (wrong-sign control) arm, where pow(->0, -k) would otherwise blow up.
    static constexpr float kCoupleWMin = 1e-3f, kCoupleWMax = 1e3f;
    // ---- 2026-08-04 · PER-LEG hip1 SATURATION (the skid-steer rectification test) ------
    // steer_eff enters as `sgn*(stroke + side*steer)` with side = +1 left / -1 right, so the
    // COMMAND is a symmetric differential.  But the clamp is per-leg and independent
    // (`clamp(y[j],-1,1)`), and hip1 already sits at ~56 % clip duty from the stroke alone.
    // So the OUTER side (stroke + steer) is pushed further into the rail and its speed-up is
    // discarded, while the INNER side (stroke - steer) moves OFF the rail and its slow-down
    // lands in full.  The differential is RECTIFIED — half the commanded turn authority is
    // thrown away, and net thrust drops, which arcs the body instead of pivoting it.
    // Pooled clip_duty cannot see this because it averages the two sides together.
    std::vector<double> sat_clip_leg_, sat_pre_leg_;   // per leg, hip1 only
    double  cw_spr_acc_ = 0.0;                     // consumer check: mean (max-min)/mean weight
    int64_t cw_n_       = 0;
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
    // FOOTFALL REGULARITY — the prerequisite nobody measured before trying to lock a stroke
    // to the step.  Sum / sum-of-squares of TRUE inter-touchdown intervals per leg, so the
    // coefficient of variation (sd/mean) is reportable.  A PLL cannot lock to a rhythm whose
    // period wanders: if CV is large on the HEALTHY gait then no touchdown-referenced clock
    // can ever be well-locked, and step regularity is the thing to fix first.
    std::vector<double>  ga_con_iv_sum_, ga_con_iv_sq_;
    std::vector<int64_t> ga_con_iv_n_;
    // CHATTER vs APERIODICITY — the two readings of a high step_cv, told apart by BOUT
    // duration.  Chatter = many very short stance/swing bouts (a foot flickering while
    // essentially planted); genuine irregularity = normal-length bouts arriving at
    // irregular times.  Identical in step_cv, opposite in what to do next.
    std::vector<int32_t> ga_bout_run_;          // length of the current bout, ticks
    std::vector<char>    ga_bout_state_;        // contact state of the current bout
    double  ga_st_bout_sum_ = 0.0; int64_t ga_st_bout_n_ = 0, ga_st_bout_short_ = 0;
    double  ga_sw_bout_sum_ = 0.0; int64_t ga_sw_bout_n_ = 0, ga_sw_bout_short_ = 0;
    static constexpr int kShortBoutTicks = 4;   // < this = too brief to be a real phase
    // ...and the same interval moments counting ONLY touchdowns preceded by a genuine
    // swing (>= kRealSwingTicks).  If the raw CV is ~1.0 because micro-lifts are being
    // pooled with real steps, this filtered CV is where a hidden rhythm would show up.
    std::vector<double>  ga_rs_iv_sum_, ga_rs_iv_sq_;
    std::vector<int64_t> ga_rs_iv_n_;
    std::vector<int64_t> ga_rs_last_;
    static constexpr int kRealSwingTicks = 4;
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
    // ---- 2026-08-02 Phase-0 SATURATION instrument (pure diagnostic) -------------
    // The HK branch emits motor_gain·tanh(z) (motor_gain=3.0 deployed) and ~7 further
    // additive terms are summed onto it, while the ONLY output clamp is ±1 applied at
    // the very end.  So the actuator nonlinearity the BODY applies is a hard clip, not
    // the tanh the HK loop-Jacobian G=diag(1−tanh²) assumes.  These counters measure
    // how much of the command the body never sees, and HK's share of what is sent.
    // Accumulated post-warmup only, per motor index (0=hip1,1=hip2,2=knee), pooled
    // over legs.  Report-only: nothing here feeds a control path.
    std::vector<double> sat_clip_hits_;   // ticks with |y_pre-clamp| > 1
    std::vector<double> sat_pre_abs_;     // Σ |y_pre-clamp|
    std::vector<double> sat_pre_max_;     // max |y_pre-clamp|
    std::vector<double> sat_hk_abs_;      // Σ |HK branch output| (mg·ag·tanh(z))
    double              sat_n_ = 0.0;     // leg-tick samples behind the sums
    // Intra-leg coordination: hip2 vs knee command sign agreement, pooled over legs.
    int64_t             hk_agree_ = 0, hk_agree_n_ = 0;
    // ---- 2026-08-03 · IMPORT I7: WHOLE-BODY CONTROLLER -----------------------------
    // The empirical case (ledger 2026-08-03): HK coordinates joints that SHARE a C — it
    // discovered the hip2+knee lift synergy unaided, which the hand-built Cruse rule had
    // to be explicitly told to produce.  The one coordination it cannot do is inter-leg,
    // and that is exactly the coordination four independent per-leg blocks CANNOT
    // REPRESENT: C has no cross-leg terms, so legs can only couple mechanically, through
    // the body — which is slow, and which faster learning outruns (coherence 0.484 at
    // ctrl_lr 0.01 falls to 0.408 at 0.10).  Same mechanism, wider matrix.
    // This is also the structural difference from the Playful Machine: their dog, hexapod
    // and humanoid all run ONE Sox across every joint.
    // 0 = per-leg blocks (historical, byte-identical).  1 = one controller, all legs.
    double              whole_body_c_ = 0.0;
    Eigen::MatrixXf     Aw_, Cw_;              // N x M  and  M x N   (N = n_legs*n, M = n_legs*m)
    Eigen::VectorXf     bw_, hw_;              // N, M
    Eigen::VectorXf     Xw_, prevXw_, prevYw_; // concatenated state / command
    Eigen::VectorXf     Zw_;                   // this tick's pre-tanh operating point (sliced per leg)
    Eigen::MatrixXf     Cdepw_;                // M x N  whole-body DEP correlation
    Eigen::VectorXf     prevPrevYw_;           // Δy needs two steps of command history
    bool                wb_ready_ = false, wb_have_prev_ = false;
    int64_t             wb_steps_ = 0;
    float               wb_tle_ema_ = 0.0f;
    // ---- 2026-08-03 · INTER-LEG PLV, replacing gait_coherence as the coordination read.
    // gait_coherence() is the Kuramoto order parameter of the four phases AT AN INSTANT.
    // Sampled once it is nearly meaningless: for four INDEPENDENT phases its distribution
    // has mean 0.450 and sd 0.219, so single-instant readings scatter across [0,1] and look
    // bimodal.  That is exactly what a 12-seed sweep produced, and it was misread as
    // "some seeds phase-lock" -- the operator caught it by eye (locked and unlocked seeds
    // look identical).  Time-averaging it gives 0.453 +- 0.035 on every seed: the random
    // null.  It is also maximal on a FROZEN body, which the ledger already warned about.
    //
    // PLV measures the right thing: whether each leg PAIR holds a CONSTANT relative phase
    // over time, |mean_t e^{i(phi_i - phi_j)}|, which is 1 for a trot (relative phase pi)
    // and -> 0 for independent legs regardless of any instant's alignment.
    double              plv_cos_[16] = {0}, plv_sin_[16] = {0};   // per ordered pair i<j
    int64_t             plv_pair_n_[16] = {0};                    // samples admitted per pair
    int64_t             plv_n_ = 0;
    float               interleg_plv() const;
    // ---- 2026-08-04 · WINDOWED PLV.  interleg_plv() above accumulates over the WHOLE run,
    // so it cannot express a before/after: a perturbation at tick 2500 is diluted by 6000
    // ticks of history and the recovery it is supposed to measure is invisible.  This is the
    // trailing-window form: an EMA of each pair's phasor, DECAYED EVERY TICK and added to
    // only while both legs clear kPlvAmpFloor.  Decaying unconditionally is the point — a
    // pair that stops oscillating decays toward 0 ("no recent evidence of locking") instead
    // of freezing at a stale high value, which is the frozen-body degeneracy that killed
    // gait_coherence arriving by a third route.  plv_win_sup_ is the same EMA over the
    // admission indicator, so |z| <= sup and `plv_win / plv_win_n` reads as "locking GIVEN
    // oscillation".  ALWAYS read plv_win beside plv_win_n (CLAUDE.md, the standing rule).
    static constexpr double kPlvWinAlpha = 1.0 / 500.0;           // tau ~ 500 ticks ~ 10 s
    double              plv_win_cos_[16] = {0}, plv_win_sin_[16] = {0};
    double              plv_win_sup_[16] = {0};                   // EMA of "pair admitted"
    float               interleg_plv_win() const;
    float               interleg_plv_win_support() const;
    void wb_init(int n_per_leg);
    void wb_learn_and_control();
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
    float   height_k_eff_  = -1.0f;   // adapted setpoint fraction; <0 = uninitialised
    double  height_ground_gain_ = 0.0; // 0 = off, byte-identical
    // 2026-08-07 — COMPLETE THE LIFT: drive the KNEE with the height bias too.
    //
    // MEASURED: hip2 and the knee agree on sign only 50.8% +- 1.1% of ticks — a coin
    // flip, i.e. they are independent, and the panic pathway's own comment records why
    // that matters: "knee- (extend) was UN-tucking and fighting the hip2 lift -> no
    // lift (chassis_y barely moved).  Same sign = a coherent anti-gravity push."  So
    // half of every lift attempt is self-cancelling.
    //
    // The PANIC pathway already drives BOTH (y[1] += drive; y[m-1] += drive) for
    // exactly this reason.  The height homeostat drives hip2 ALONE — a one-joint
    // version of an action the codebase already established needs two.  This is not a
    // new coordination topology being imposed (prohibition 7); it is completing an
    // existing anti-gravity action to match the pathway that was measured to work.
    //
    // Fraction of the hip2 lift applied to the knee.  0 = off, byte-identical.
    double  height_lift_knee_ = 0.0;
    // 2026-08-07 — COMPLETE THE STANCE LIFT.  stance_lift biases the KNEE on planted
    // legs and explicitly not hip2 ("no hip2 -> no foot-lift traction loss").  But that
    // reasoning covers hip2 MINUS (foot up); on a PLANTED foot hip2 PLUS presses the
    // foot down and raises the chassis -- Rule 5 says so ("+hip2 = press foot down") and
    // the panic pathway drives hip2+ and knee+ TOGETHER for exactly this, recording that
    // opposite signs cancel the lift.  So stance_lift is the one-joint version of a
    // two-joint chassis raise, sitting in the one carrier that IS live during
    // locomotion (height_lift is faded to zero while cruising; that is why extending
    // THAT path was a NULL).
    //
    // Fraction of stance_lift_gain also applied to hip2, SAME sign.  Planted legs only,
    // so no swing leg is ever hoisted off the terrain -- the measured objection to a
    // whole-body lift does not apply.  0 = off, byte-identical.
    double  stance_lift_hip2_ = 0.0;
    // 2026-08-09 — STROKE-DIRECTION-AWARE STANCE RELEASE.
    //
    // MEASURED (flbrake.py on full actuator-sweep traces): every leg's COMMANDED
    // stroke reverses 7–9 ticks before liftoff (servo slew only 2–3), and the leg
    // pays −0.004…−0.015 g/tick of braking shear for the whole pressed window;
    // posture knobs redistribute which leg pays (fl worst, shear ratio −0.48) but
    // never remove the toll.  So: from the tick a planted leg's own commanded hip1
    // delta flips sign (recovery onset), multiply its stance biases by
    // (1 − stance_release_frac) until it leaves stance.  Fully egocentric (the
    // brain's own command stream), no propulsive-sign convention needed.
    // 0 = off, byte-identical.
    double  stance_release_frac_ = 0.0;
    std::array<bool,  8> sr_released_ {};   // per leg: recovery detected this stance bout
    std::array<bool,  8> sr_was_stance_ {}; // per leg: previous tick's stance state
    std::array<float, 8> sr_prev_dh1_ {};   // per leg: last commanded hip1 delta past deadband
    long   sr_stance_ticks_  = 0;           // diag: consumer-fired check
    long   sr_release_ticks_ = 0;
    // 2026-08-07 — HOMEOKINETIC SUPPORT SELECTOR.
    //
    // MEASURED (n=4, egocentric |dx|/|du| over joint vs commanded deltas):
    //     planted   1      2      3      4
    //     resp      0.472  0.470  0.432  0.367      (+28% at 2 vs 4)
    // with |du| FLAT across states -- the SAME command produces more sensory change
    // with fewer feet down.  So preferring responsive support states prefers 2-leg
    // support WITHOUT EVER BEING TOLD FORWARD PROGRESS IS GOOD.  That distinction is
    // the whole point: "prefer states with better forward impulse" is reward shaping
    // on progress (§5.1, and what coord_reward_drive turned out to be); "prefer states
    // where my actions have the most effect on my own sensors" is homeokinesis.
    //
    // ⚠ THE DIVISOR IS LOAD-BEARING, NOT DECORATION.  1-planted is as responsive as
    // 2-planted (0.472 vs 0.470) but moves less -- one foot is maximally sensitive and
    // maximally UNPREDICTABLE, i.e. falling.  Dividing by the body's own forward-model
    // error is what makes this homeokinesis rather than thrash-seeking.
    //
    // ⚠ AND IT IS KEYED ON sum(contact), NOT ON THE SUPPORT EPM.  Measured: per-node
    // responsiveness spreads 7% across 171 vocabulary nodes vs 11% across the four
    // planted-counts, and only ~5% of node spread survives removing the count effect.
    // The integer explains MORE than the 150-node vocabulary, so the vocabulary is
    // reserved for what the count cannot do (which state follows which).
    //
    // Acts ONLY on explore_mult -- commands no joint, imposes no coordination topology,
    // steers machinery that already exists.  0 = off, byte-identical.
    double  support_select_gain_ = 0.0;
    static constexpr int   kSupportBins   = 5;      // 0..4 feet planted
    static constexpr float kRespAlpha     = 0.01f;  // responsiveness EMA rate
    static constexpr int   kRespWarmup    = 200;    // samples before a bin votes
    static constexpr float kSupportMultMin = 0.5f;  // never silences the probe
    static constexpr float kSupportMultMax = 3.0f;
    float   resp_ema_[kSupportBins]  = {0,0,0,0,0};
    int     resp_seen_[kSupportBins] = {0,0,0,0,0};
    float   support_value_diag_ = 1.0f;
    float   support_resp_diag_  = 0.0f;
    float   support_mult_diag_  = 1.0f;
    int     support_bin_diag_   = -1;
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

    // ---- PART IV gain socket (GainEvolver → evolved-gain vector) -------------
    // A GainVector's (key,value) pairs are stashed by the bus handler and applied
    // at the TOP of the next tick() through the existing on_param_change dispatch,
    // then READ BACK from current_params(): the dispatch chain has no terminal
    // else (unknown keys are silently ignored), so the read-back is the only
    // honest applied-counter (§3.2 — a gate has shipped as silent dead code here
    // before).  gain_topic_ == "" ⇒ no subscription ⇒ byte-identical.
    // applied_gains_ (last landed value per key) exists for restore_state replay:
    // evolved gains live in param members the instance snapshot does NOT
    // round-trip, so a restored clone would silently revert to config gains.
    void handle_gain_vector(MessagePtr payload);
    void apply_pending_gains();
    std::string gain_topic_;                                     // "" = socket off
    std::vector<std::pair<std::string, double>> pending_gains_;  // stashed by handler
    mutable std::mutex pending_gains_mu_;                        // parallel-level proofing
    std::map<std::string, double> applied_gains_;                // last landed values
    int64_t gains_applied_  = 0;                                 // read-back verified
    int64_t gains_rejected_ = 0;                                 // ignored/mismatched keys

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

    // ---- PART III lever (b): the PLANNER's band-gated posture prediction ----
    // A second posture-objective socket (objective.plan.<leg>) carrying the
    // MotorPlanner's BASE-roll decode at its plan depth: predicted_latent =
    // [m targets | m per-joint weights] (weights 0/1 = the earned authority-band
    // gate, planner-side).  Fused with the keyframe objective PER JOINT,
    // precision-weighted (the LateralVoter pattern): w_eff = wk + plan_gain·wp,
    // x*_eff = (wk·xk + plan_gain·wp·xp)/w_eff — so an ungated joint (wp=0)
    // leaves the keyframe pull EXACTLY as it was (never weaken a working loop),
    // and plan_gain=0 is byte-identical.  Empty socket = OFF.
    void handle_plan(int leg, MessagePtr payload);
    std::vector<std::string>     plan_topics_;   // optional per-leg plan-objective topics; empty = OFF
    double                       plan_gain_ = 0.0;
    // OPERATOR SCAFFOLD ([G]/[R] bench class — a lesion-as-test, never an
    // operating mode): crossfade the FINAL assembled command (reflexes +
    // scaffolds, everything) toward a pure position-servo on the planner's
    // published targets.  fade 0 = byte-identical; fade 1 = the stride as the
    // planner imagines it, embodied.  Ungated joints' targets arrive as the
    // current pose (planner-side), so they HOLD rather than flail.
    double                       plan_fade_ = 0.0;
    double                       plan_puppet_gain_ = 2.0;   // servo P on (x*−x)
    std::vector<Eigen::VectorXf> plan_target_;   // per-leg target joint positions (motor_dim)
    std::vector<Eigen::VectorXf> plan_w_;        // per-leg PER-JOINT weights ∈ [0,1] (the band gate)
    std::vector<char>            plan_seen_;
    float plan_pull_ema_ = 0.0f;                 // consumer-fired telemetry: mean w·|x−x*| (EMA)
    float plan_w_mean_   = 0.0f;                 // mean effective plan weight this tick

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
        Eigen::MatrixXf     Bx;                   // n x n  state-transition term (empty unless state_model_lr > 0)
        Eigen::VectorXf     ytrace;               // servo-filtered command (empty unless model_trace > 0)
        Eigen::VectorXf     pulse_x0;             // state at pulse start (babble_isolate bookkeeping)
        int                 pulse_motor = -1;     // which motor the current pulse drives
        float               pulse_sign  = 0.0f;
        Eigen::VectorXf     pulse_dplus;          // completed + window's Δx/hold (awaiting its − twin)
        int                 pulse_dplus_motor = -1;
        Eigen::MatrixXf     Cp;                   // the prior's OWN controller (empty unless state_prior_split)
        Eigen::VectorXf     hr;                   // reach bias (consolidate_reach mode 3): written only in
                                                  // consolidated quiet (rate ∝ c), applied as c·hr — ramps in
                                                  // with consolidation, fades the moment a fall collapses c
        // R1 regime banks (empty unless regime_topic set): per-regime self-models.
        // L.A/Bx/b remain the ACTIVE working copy; banks swap in/out on regime
        // change, so every model path reads/writes exactly as before.
        struct ModelBank { Eigen::MatrixXf A, Bx; Eigen::VectorXf b; float tle_ema = 0.0f;
                           float sp_err = -1.0f;   // per-regime prior-error EMA (−1 = unseen)
                           int64_t samples = 0; };
        std::vector<ModelBank> banks;
        int                 active_bank = -1;
        float               calm_state = 1.0f;    // the annealing ratchet (slow attack, fast release)
        float               calm_peak  = 0.1f;    // decaying peak-hold of the prior error (the reference)
        float               last_mult  = 1.0f;    // the calm multiplier the assembly last applied
        Eigen::VectorXf     b;                    // n
        Eigen::MatrixXf     C;                    // m x n  (sensor → motor)
        Eigen::MatrixXf     Cphi;                 // m x 2  learned phase-conditioning (posture feed-forward)
        Eigen::MatrixXf     Cvel;                 // m x 2  learned phase-conditioning (velocity feed-forward / propulsive pump)
        Eigen::Vector2f     prev_phi_ctx{0.0f, 0.0f}; // [cos φ, sin φ] at command time (for the Cphi/Cvel update)
        Eigen::VectorXf     h;                    // m
        Eigen::VectorXf     x;                    // latest sensor (n)
        Eigen::VectorXf     prev_x;               // sensor at command time (n)
        Eigen::VectorXf     prev_y;               // last motor command (m)
        Eigen::VectorXf     prev_prev_y;          // the one before it — DEP needs Δy
        Eigen::MatrixXf     Cdep;                 // m x n  accumulated Δy·Δxᵀ correlation
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
        float               phase_prev  = 0.0f;   // previous tick, for the advance/retrograde metric
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
        int32_t             td_run       = 0;     // ticks of sustained contact on the candidate
        int64_t             td_cand_tick = -1;    // raw rising edge awaiting confirmation (-1 = none)
        bool                td_contact   = true;  // debounced contact state
        bool                step_locked  = false; // false ⇒ the stroke falls back to L.phase
    };
    std::vector<Leg> legs_;

    static constexpr float kTeleEmaAlpha   = 0.02f;
    static constexpr float kKneeEmaAlpha   = 0.01f;   // slow mean for the phase reference
    static constexpr float kPhaseVelScale  = 15.0f;   // balances knee Δ vs (pos−mean) in atan2
    // ── PHASE-REFERENCE REPAIR (2026-08-05).  MEASURED: phase_retro = 0.666 -- L.phase runs
    // BACKWARDS two ticks in three, so the quantity the Kuramoto coupling drives toward
    // gait_phase offsets is jitter, not an oscillator.  The cause is structural: the atan2's
    // y-arm is x[3*pj+2], a RAW per-tick joint delta, i.e. a high-pass filter.  Near the
    // oscillation's zero crossings its noise exceeds the (pos - mean) x-arm, and the phase
    // vector rattles instead of rotating.  A per-tick difference can never be a clean
    // velocity estimate for a ~50-tick cycle.
    //   phase_vel_smooth low-passes the velocity arm ONLY (the x-arm keeps its own slow-mean
    // reference), which is the minimum change that can restore monotonic rotation.  0 = off,
    // byte-identical.  Judge it on phase_retro directly -- that is a property of the signal,
    // measurable without any behavioural claim.
    float  phase_vel_ema_[8] = {0,0,0,0,0,0,0,0};
    double phase_vel_smooth_ = 0.0;
    // ── SYMMETRIC phase filter.  phase_vel_smooth was RETRACTED (net displacement -57%):
    // filtering only the y-arm of atan2(vel, pos) shrinks and phase-shifts one component,
    // DISTORTING the ellipse so the phase warps non-uniformly around the cycle -- and since
    // L.phase times the power stroke, the stroke then fires at the wrong point and pushes
    // backward as often as forward (the operator's "alternating current" fwd_v).
    //   Filtering BOTH arms with the same kernel rotates the vector RIGIDLY: identical
    // noise rejection, but the only phase effect is a CONSTANT offset, which stroke_phase
    // already exists to absorb.  0 = off, byte-identical.
    float  phase_pos_ema_[8] = {0,0,0,0,0,0,0,0};
    bool   phase_sym_init_[8] = {false,false,false,false,false,false,false,false};
    double phase_sym_smooth_ = 0.0;
    // ── P1 SHADOW PHASES (2026-08-09, substrate-repair campaign) — ZERO AUTHORITY.
    // Three candidate replacements for the retrograde L.phase, computed every tick and
    // only ever exported to diagnostics; no control path reads them.  The winner (scored
    // offline against flbrake's propulsive-window ground truth) gets a consumer switch
    // in a LATER lever, never here.
    //   A: per-leg PLL, BodyRhythmTracker's proven form — integrator + frequency state
    //      from hysteresis up-crossings of the leg's own knee coordinate + soft pull at
    //      each crossing.  No high-pass arm, no group delay ("no filter of any shape can
    //      fix a signal whose value must be timely" — the retraction above).
    //   B: shared reference + per-leg offsets = cpg_phase_ + gait_phase_[i].  Carries a
    //      measured defect to explain first: BRT is UNLOCKED even in walker seeds
    //      (brt_plv ~0.10, campaign log P0).
    //   C: delay-compensated symmetric filter — both-arm EMA advanced by ω·τ, the
    //      retraction's own recorded re-use context; τ = (1−a)/a of the kernel, ω from
    //      candidate A's crossing measurement (the honest per-leg frequency).
    static constexpr float kShAmpAlpha   = 0.02f;   // BRT amp_alpha
    static constexpr float kShPeriodAlpha= 0.2f;    // BRT period EMA weight
    static constexpr float kShOmegaLp    = 0.05f;   // BRT omega_lp
    static constexpr float kShPhaseLock  = 0.10f;   // BRT phase_lock (soft pull)
    static constexpr float kShHysFrac    = 0.2f;    // hysteresis = frac · amp_ema
    static constexpr float kShFilterA    = 1.0f/3;  // C's kernel weight → τ = 2 ticks
    float   shA_amp_[8]    = {0,0,0,0,0,0,0,0};
    float   shA_period_[8] = {0,0,0,0,0,0,0,0};
    float   shA_omega_[8]  = {0,0,0,0,0,0,0,0};     // 0 = uninitialized (seeded on first period)
    float   shA_phi_[8]    = {0,0,0,0,0,0,0,0};
    int     shA_tsu_[8]    = {0,0,0,0,0,0,0,0};
    uint8_t shA_below_[8]  = {1,1,1,1,1,1,1,1};
    float   shB_phi_[8]    = {0,0,0,0,0,0,0,0};
    float   shC_pos_[8]    = {0,0,0,0,0,0,0,0};
    float   shC_vel_[8]    = {0,0,0,0,0,0,0,0};
    uint8_t shC_init_[8]   = {0,0,0,0,0,0,0,0};
    float   shC_phi_[8]    = {0,0,0,0,0,0,0,0};
    // Retro-fraction EMAs per candidate (pooled over legs; same α as phase_retro_diag_).
    float   shA_retro_ = 0.0f, shB_retro_ = 0.0f, shC_retro_ = 0.0f;
    float   shA_prev_[8] = {0,0,0,0,0,0,0,0};
    float   shB_prev_[8] = {0,0,0,0,0,0,0,0};
    float   shC_prev_[8] = {0,0,0,0,0,0,0,0};
    uint8_t sh_prev_init_[8] = {0,0,0,0,0,0,0,0};
    // ── P4 arm 2 (2026-08-09): TOUCHDOWN-CONSISTENCY PHASE OFFSET (phase_td_pull).
    // Arm 1 (per-leg stroke lock) moved step_cv for the first time in campaign history
    // (0.97 → 0.82) and killed transport the same way it did 2026-07-27 — independent
    // thrusts cancel.  This form keeps every consumer (stroke, Kuramoto, amp, fitness)
    // on ONE per-leg phase rotated by a slow offset, nudged at each ACCEPTED touchdown
    // toward the leg's own RUNNING touchdown phase — self-consistency, no imposed
    // target, inter-leg coherence preserved.  0 = off, byte-identical.
    double  phase_td_pull_ = 0.0;
    // ── P4 arm 3 (2026-08-10): SELECT rhythm, don't force it (coord_td_weight).
    // Arms 1–2 proved the phase must stay a raw state observation; the sanctioned knob
    // for WHERE legs sit in the cycle is gait_phase, and the (1+1) search already owns
    // it.  This adds a touchdown-consistency term to the mode-1 fitness — the
    // per-window resultant of L.phase at raw contact onsets — so the search DISCOVERS
    // offsets under which touchdowns land at a repeatable phase.  No phase is touched;
    // selection does the entraining.  0 = off, byte-identical.
    double  coord_td_weight_ = 0.0;
    double  ctd_cos_[8] = {0,0,0,0,0,0,0,0};
    double  ctd_sin_[8] = {0,0,0,0,0,0,0,0};
    int     ctd_n_[8]   = {0,0,0,0,0,0,0,0};
    uint8_t ctd_prev_con_[8] = {0,0,0,0,0,0,0,0};
    float   coord_td_R_diag_ = -1.0f;   // last window's touchdown-consistency, −1 = none yet
    // ── P4 arm 3b (2026-08-10): interval-CV selection (coord_cv_weight).  Arm 3's miss
    // was diagnostic — phase-consistency ≠ time-regularity.  This penalizes the fitness
    // by the mean |interval − running period| / period over the window's DEBOUNCED
    // touchdowns (the v3 machinery's accepted intervals), selecting literally the
    // operator's step-regularity number.  0 = off, byte-identical.
    double  coord_cv_weight_ = 0.0;
    double  ccv_dev_sum_ = 0.0;
    int     ccv_dev_n_   = 0;
    float   coord_cv_diag_ = -1.0f;     // last window's mean interval deviation
    // ── SWING DESCENT (2026-08-10, operator-diagnosed): swing_tuck_hip2 is a CONSTANT
    // lift across the whole swing, so it fights the descent — rear feet land unsettled
    // (miss% 5→9, el_def −0.17→−0.06, rl early stance net-braking) and the stroke
    // sweeps back before full plant.  This splits the swing by phase, SELF-SCALED to
    // each leg's own running swing duration (dimensionless fraction, no tick constant):
    // first half = the lift/fold biases as configured; past kDescentFrac of the leg's
    // typical swing, hip2 flips to +swing_descend_gain (press DOWN, Rule-5 sign) so the
    // foot plants before the stroke reverses.  Knee keeps its fold.  0 = byte-identical.
    double  swing_descend_gain_ = 0.0;
    double  swing_descend_knee_ = 0.0;   // the KNEE half (operator 2026-08-14):
                                         // during descent the shank flips from
                                         // fold to EXTEND toward the ground
    double  swing_overdue_knee_ = 0.0;   // the ERROR-FORM: reach for ground only
                                         // when the swing outlives its own average
    // REAR LANDING SEQUENCE (operator 2026-08-14: "hip2 and knee must DROP
    // first, THEN hip1 sweeps back with a bit of knee extension").  Two
    // separately-gated rear-pair levers (legs 2,3 = cfg rl/rr = anatomical
    // RR/RL — front/rear is NOT mirrored by the naming flip):
    double  rear_land_gain_  = 0.0;      // descent: hip2 press + knee SERVO to
                                         //   the plant angle (closed-loop — the
                                         //   open-loop extension was the
                                         //   measured regression)
    double  rear_knee_plant_ = 0.2;      // the plant angle (slightly flexed for
                                         //   push traction)
    double  rear_push_ext_   = 0.0;      // stance: knee extension while the
                                         //   planted leg's hip1 actually sweeps
    float   dh1_ema_[8] = {0,0,0,0,0,0,0,0};   // per-leg |Δhip1| running scale
    long    rear_land_ticks_ = 0;        // consumer-fired checks
    long    rear_push_ticks_ = 0;
    static constexpr float kDescentFrac = 0.5f;
    int     swd_age_[8] = {0,0,0,0,0,0,0,0};       // ticks in current swing
    float   swd_dur_[8] = {0,0,0,0,0,0,0,0};       // running mean swing duration (EMA)
    uint8_t swd_air_[8] = {0,0,0,0,0,0,0,0};       // previous airborne state
    uint8_t swd_assisted_[8] = {0,0,0,0,0,0,0,0};  // overdue reach fired THIS swing —
                                                   // its duration must NOT train the
                                                   // expectation (the ratchet fix:
                                                   // the reference chases only
                                                   // UNASSISTED swings)
    long    swd_press_ticks_ = 0;                   // consumer-fired check
    long    swd_overdue_ticks_ = 0;                 // touchdown-seeking fired check

    float   td_off_[8]     = {0,0,0,0,0,0,0,0};   // rotation applied to L.phase
    float   td_ref_cos_[8] = {0,0,0,0,0,0,0,0};   // circular EMA of touchdown phase
    float   td_ref_sin_[8] = {0,0,0,0,0,0,0,0};
    long    td_pull_events_ = 0;                   // consumer-fired check
    static constexpr float kTdRefAlpha = 0.05f;
    static constexpr float kTdRefMinR  = 0.10f;    // no pull until the ref means something
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
    // ---- 2026-08-05 · commit TIMING is now tunable, and it is mismatched -------------
    // Operator, watching at 2x with commit ON vs OFF: with commit the robot "at least tries
    // to move forward during the pauses, with much better steps once it moves"; without it
    // the robot "is simply shaking -- it doesn't look like it's taking any steps at all".
    // So commit's job is not preventing stalls (removing it made stalling WORSE, 14%->20%)
    // -- it is SUPPRESSING UNDIRECTED NOISE once directed motion exists.
    //
    // But the timescales do not match the behaviour.  Measured bursts last 1-2 s, while
    // commit needs 180 ticks (3 s) of sustained progress before it engages AT ALL and then
    // ~4 s to ramp -- so it arrives after the burst it was meant to protect is over -- and
    // releases in 1.5 s, abandoning on the first faltering step.  The asymmetry is
    // backwards for sustaining a gait: it was tuned to "release quickly, re-explore",
    // which is right for escaping a stuck state and wrong for holding a found rhythm.
    // Exposed so the fix can be MEASURED rather than argued.  Defaults are the historical
    // constants, so unset = byte-identical.
    // ---- 2026-08-05 · INTENT-RELATIVE CONFIDENCE (operator's reframe) ----------------
    // Everything that went wrong with commit_prec came from measuring an ABSOLUTE property
    // ("how well do I predict myself") when the useful quantity is "am I achieving what I
    // am currently trying to do".  Operator: "confidence and prediction is more predicated
    // on what we want the body to be doing in this moment... if we have forward velocity
    // then all of the other metrics should be suppressed a little so the same model can be
    // continued without interruption. Give us a lever for the higher order loops."
    //
    // Two failures dissolve at once:
    //   * THE FREEZE TRAP.  A stopped body is highly predictable, which is why commit_prec
    //     ROSE when it stalled.  A stopped body is FAILING at "move forward", so its
    //     goal-relative error is high exactly when it is stuck.  No activity-term patch is
    //     needed; the objective supplies it structurally.
    //   * THE SIGN INVERSION.  Moving fast is intrinsically less predictable (measured:
    //     corr(motor_tle, displacement) = +0.129), so a residual-based confidence penalises
    //     the behaviour we want.  Error-against-intent does not.
    //
    // This is plan §1.1's motor objective ("the arbiter's currency is a MOTOR OBJECTIVE,
    // not a heading") getting its first real consumer.
    std::string intent_topic_;                     // ProprioToken [v_forward*, yaw_rate*]
    float  intent_v_ = 0.0f, intent_w_ = 0.0f;
    bool   intent_seen_ = false;
    int64_t intent_msgs_ = 0;                      // consumer check
    float  err_run_ema_ = 0.0f;                    // running scale of the intent error
    // The exact error the commit-precision loop descends, so diagnostics can never drift
    // from the mechanism.  Uses the CURRENT spreads (read-only; the EMAs advance in step()).
    float intent_err_norm() const {
        // Must mirror step() exactly: one-sided forward term (overshooting the intent is
        // not an error) and a Cauchy log1p tail (a lurch must not dominate).  See the long
        // note at the use site for why each is a likelihood choice, not a tuning knob.
        const float zv = std::min(0.0f, fwd_progress_ema_ - intent_v_) / (ev_spread_ema_ + 1e-6f);
        const float zw = float(intent_yaw_gain_) * (yaw_rate_ema_ - intent_w_) / (ew_spread_ema_ + 1e-6f);
        return std::log1p(zv * zv + zw * zw);
    }
    // ── RUNG 1: act on the predicted state.  See the use site in the .cpp.
    double lookahead_gain_ = 0.0;    // 0 = off, byte-identical; <0 = wrong-sign control
    double lookahead_mode_ = 0.0;    // 0 = fixed point (true lookahead), 1 = prev-action
    double lookahead_null_ = 0.0;    // 1 = drop A*y (control: is it the DYNAMICS?)
    float  la_dev_ema_     = 0.0f;   // ||x_eff - x||, the consumer check
    // ── STATE-SPACE PRIOR (2026-08-31, the microduck lever).  A soft prior on
    // ARBITRARY state indices (e.g. the bridge's appended load slot: predicted
    // lean = 0).  TWO coupled halves, both gated by state_prior_gain (see the use
    // sites and test_state_prior for why the keyframe socket's ξ̃-replacement is
    // NOT the mechanism — the HK dC update is sign-blind and amplifies the error):
    //   1. ξ̃[idx] *= (1−w) — the sensitivity rule may REST on the prior-owned dim;
    //   2. C/h descend the prior's own error through the LEARNED model A(idx,·).
    std::vector<double> state_prior_indices_;   // state indices; NEGATIVE = from the end (−1 = last)
    std::vector<double> state_prior_targets_;   // target values x*, parallel to indices
    double state_prior_gain_    = 0.0;          // weight w ∈ [0,1]; 0 = off, byte-identical
    double state_prior_lr_     = 0.1;           // fraction of the GN-normalised correction per tick (C half)
    double state_prior_h_lr_   = -1.0;          // h half's rate; -1 = follow state_prior_lr, 0 = C-only
    double state_model_lr_     = 0.0;           // Bx learning rate; 0 = no state term, byte-identical
    double model_trace_        = 0.0;           // EMA rate β of the command trace; 0 = raw prev_y, byte-identical
    double reset_breaks_pairing_ = 0.0;         // 1 = reset invalidates model pairing; 0 = bug-compatible
    double babble_isolate_       = 0.0;         // 1 = one-motor held-pulse babble; 0 = legacy white
    int    babble_hold_          = 6;           // held-pulse ticks for babble_isolate
    float  state_prior_err_ema_ = 0.0f;         // telemetry: mean |x[idx] − x*| (EMA; DECAYS when off)
    float  state_prior_err_long_ = 0.0f;        // slow EMA of the same (the calm reference scale)
    double state_prior_calm_    = 0.0;          // exploration-precision annealing strength; 0 = off
    std::vector<double> state_prior_calm_indices_;  // the key's own indices (empty = all prior indices)
    double state_prior_calm_fixed_ = 0.0;       // >0 = pin the multiplier (designed gate, tuned magnitude)
    double state_prior_split_  = 0.0;           // 1 = prior writes its OWN matrix Cp; HK keeps C
    double state_prior_damping_ = 0.0;          // L2 brake on Cp ALONE (the split's whole point)
    // R1: the regime socket
    std::string regime_topic_;                  // RealityToken source; empty = banks off, byte-identical
    double babble_owns_a_ = 0.0;                // 1 = the babble's paired-difference estimator owns A forever
    double state_prior_calm_mode_ = 0.0;        // R2: 0 = continuous key (legacy), 1 = regime-keyed
    double consolidate_gain_ = 0.0;             // earned consolidation strength; 0 = off
    float  consolidate_c_    = 0.0f;            // the consolidation state ∈ [0,1]
    double consolidate_n_    = 0.0;             // gate reads only the FIRST N prior indices; 0 = all (legacy)
    double consolidate_spares_prior_ = 0.0;     // >0: the prior's own descent survives consolidation
    double consolidate_reach_ = 0.0;            // >0: indices ≥ consolidate_n engage at lw·c, with h (reach terms)
    double consolidate_reach_lr_ = 0.0;         // mode 5's hr write rate (µ-rate; 0 = hr never writes)
    float  reach_lw_last_ = 0.0f;               // telemetry: the reach terms' current effective lw
    float  state_prior_gate_ema_ = 0.0f;        // satisfaction EMA over the gate subset (consolidate_n > 0)
    float  sp_err_peak_      = 0.0f;            // decaying peak of the prior error (the reference)
    int    regime_banks_   = 6;                 // bank slots (last slot = shared overflow)
    int    regime_winner_  = -1;                // latest winner_id from the token
    std::vector<int> bank_of_winner_;           // winner_id -> bank slot, first-seen order
    int64_t bank_switches_ = 0;                 // §3.2 consumer counter
    int    state_prior_applied_ = 0;
    float  calm_mult_ = 1.0f;                   // last applied annealing multiplier (diag)            // indices that actually resolved last controller tick —
                                                // disambiguates "satisfied (err→0)" from "never in range"
    double intent_yaw_gain_ = 1.0;                  // 0 = progress-over-ground only
    // ── STRIDE-PROFILE PREDICTION (intent_rhythm_gain).  A constant v* is a target a
    // legged body physically CANNOT hold: it advances in pulses, so a level-seeking error
    // oscillates forever and commit chases it.  Measured: pulse_cv 0.583 with a p90/p50
    // gap tail of 2.10 -- rhythm exists but is irregular, and it is identical across every
    // commit arm, which is why they all read as ties.
    //   Instead of smoothing (which adds lag, and lag is what makes a delayed-feedback loop
    // oscillate in the first place), give the body a PREDICTION TO FULFIL (doctrine §7).
    // fwd_profile_ is the body's own average forward-velocity waveform indexed by gait
    // phase -- LEARNED from what it actually does, never imposed.  The error is this
    // stride's deviation from it, so descending that error means "make this stride like my
    // strides" = consistent pulses, with the waveform's SHAPE still chosen by the body.
    static constexpr int kFwdProfileBins = 16;
    std::array<float, kFwdProfileBins> fwd_profile_{};
    std::array<uint32_t, kFwdProfileBins> fwd_profile_n_{};
    float  rhythm_spread_ema_ = 0.0f;
    float  rhythm_dev_diag_   = 0.0f;
    double intent_rhythm_gain_ = 0.0;               // 0 = off, byte-identical
    // ── fwd_v RESONANCE (operator, 2026-08-05: "find the fwd_v resonance and encourage it
    // as a positive feedback loop ... fwd_v is the only reliable metric so far").
    // An adaptive-frequency Hopf oscillator (Righetti/Ijspeert) entrained BY forward
    // velocity.  It does not impose a rhythm -- it LEARNS the frequency the body already
    // propels itself at, which is the one quantity we trust.  Its input is normalised by
    // its own running spread, so nothing here is tuned to fwd_v's magnitude (doctrine §5).
    static constexpr float kResGamma = 0.02f;    // amplitude relaxation toward the unit circle
    static constexpr float kResEps   = 0.02f;    // entrainment strength (dimensionless input)
    static constexpr float kResWAlpha = 0.0005f; // frequency adaptation (slow: ~2000 ticks)
    float  res_x_ = 1.0f, res_y_ = 0.0f;
    float  res_w_ = 0.0f;                        // rad/tick, seeded from the legs' own rate
    float  res_amp_ema_ = 0.0f;
    float  res_in_spread_ = 0.0f;
    float  res_lock_cos_ = 0.0f, res_lock_sin_ = 0.0f;   // PLV(resonator, leg phase)
    double fwd_resonance_gain_ = 0.0;            // 0 = off, byte-identical

    // ── COUPLING AS A LIVE METRIC (operator asked to "tool up ... coupling as a metric in
    // the running graph").  R is the Kuramoto order parameter over the gait-offset-corrected
    // leg phases: 1 = locked, 0 = incoherent.  phase_retro is the fraction of ticks the
    // phase runs BACKWARDS -- a real oscillator advances monotonically, so a high value
    // means L.phase is jitter, not rhythm, and the coupling is chasing noise.
    float  couple_R_diag_ = 0.0f;
    float  phase_freq_diag_ = 0.0f;              // mean rad/tick advance
    float  phase_retro_diag_ = 0.0f;
    float  ev_spread_ema_ = 0.0f;                  // running |forward| error scale (see .cpp)
    float  ew_spread_ema_ = 0.0f;                  // running |yaw| error scale -- 6.8x ev raw
    // ⚠ play never abstains: explore_mult currently reaches EXACTLY 0.000, which is what
    // produces the frozen-but-confident state.  Suppression must attenuate, never abolish.
    double explore_floor_ = 0.0;                   // 0 = legacy (can reach zero)
    double commit_prec_gain_ = 0.0;                // 0 = fixed schedule (byte-identical)
    float  q_run_ema_ = 0.0f;                      // running scale of amp/(tle+eps), the COMPETENCE signal
    float  tle_run_ema_ = 0.0f;                    // (legacy: residual-only scale)
    float  commit_prec_diag_ = 1.0f;
    static constexpr float kCommitPrecAlpha = 0.002f;   // ~500-tick window on that scale
    // The commit-precision output bounds.  They are RECIPROCAL on purpose: cp and 1/cp must
    // be equally reachable, or "confident" and "unsure" are not symmetric modulations of the
    // same window.  Everything below is derived from kCommitPrecHi alone.
    static constexpr float kCommitPrecHi   = 5.0f;
    static constexpr float kCommitPrecLo   = 1.0f / kCommitPrecHi;
    double commit_window_ticks_ = 180.0;
    double commit_rise_ticks_   = 240.0;
    double commit_decay_ticks_  = 90.0;
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
    // 2026-08-06 — BELLY-GROUNDING SETPOINT ADAPTATION.
    //
    // Measured this session: the belly IS on the ground — p1 clearance 4 mm, and
    // 58-64% of the first 200 ticks under 10 mm — while the height homeostat is
    // pushing hip2 DOWN.  Both facts are consistent: tgt = height_k * chassis_h_max
    // = 0.3 * 0.999 = 0.30 while chassis_h_ema runs 0.39-0.44, so the body sits ABOVE
    // its own setpoint and height_bias integrates NEGATIVE (-0.30..-0.47 measured).
    // Nothing is asking hip2 to lift; the one mechanism that could is asking it to
    // lower.  hip2 is consequently 4.5x under-driven (u_hip2 ~+-0.20 of full scale
    // against u_hip1's ~+-0.9) and the KNEE carries the support from a permanently
    // flexed, low-leverage posture.
    //
    // The rewrite rule says: do not script a lift, give it the ERROR the behaviour
    // minimises.  That error already exists and is egocentric — the belly rangefinder
    // reports grounding directly — it simply drives nothing.  So the setpoint fraction
    // ADAPTS: it rises while the belly is grounding and decays back toward the
    // configured height_k when it is not.  A target that is demonstrably too low
    // (the body's own sensor says it is touching) stops being a hand-set constant and
    // becomes something the body discovers.
    //
    // Deliberately NOT touching height_rest_frac: that fade is measured (2026-07-23,
    // homeo ON stalls on the hump at 2.6 vs 4.1 OFF) and removing it is refuted.  This
    // acts where the problem actually is — at rest and during the stand-up phase, where
    // fwd_progress is low, rest_frac ~ 1 and the path is already live.
    static constexpr float kHeightGroundThresh = 0.05f;  // clearance below this = grounded
    static constexpr float kHeightKMax         = 0.95f;  // ceiling on the adapted fraction
    static constexpr float kHeightMoveSuppVel = 0.025f; // fwd_progress_ema at which the height defense fully fades (height is a STANDING reflex; a lift bias loses traction while walking/climbing — belly must ride low on an incline)
    static constexpr float kPanicRampAlpha = 0.04f;   // smoothing of panic_ toward its hysteresis target
    static constexpr float kResetRateAlpha = 1.0f / 600.0f;  // Gate 0: ~600-tick (~10s@60Hz) smoothing of the disruption-rate EMA (a measurement constant, not a behavioral knob)
};

} // namespace ogma
