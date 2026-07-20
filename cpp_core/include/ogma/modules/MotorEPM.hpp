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
    void handle_feet(MessagePtr payload);
    void update_cruse_state();
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

    // ---- Per-leg working state (sized n_legs_) ----
    struct Leg {
        bool                initialized = false;
        int                 n           = 0;     // state dim
        Eigen::MatrixXf     A;                    // n x m  (motor → sensor)
        Eigen::VectorXf     b;                    // n
        Eigen::MatrixXf     C;                    // m x n  (sensor → motor)
        Eigen::MatrixXf     Cphi;                 // m x 2  learned phase-conditioning of the controller
        Eigen::Vector2f     prev_phi_ctx{0.0f, 0.0f}; // [cos φ, sin φ] at command time (for the Cphi update)
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
    };
    std::vector<Leg> legs_;

    static constexpr float kTeleEmaAlpha   = 0.02f;
    static constexpr float kKneeEmaAlpha   = 0.01f;   // slow mean for the phase reference
    static constexpr float kPhaseVelScale  = 15.0f;   // balances knee Δ vs (pos−mean) in atan2
    static constexpr float kAmpEmaAlpha    = 0.01f;   // slow amplitude estimate for the homeostat
    static constexpr float kAmpGainMin     = 0.1f;
    static constexpr float kAmpGainMax     = 5.0f;
    static constexpr float kAmpSeekMin     = 0.15f;   // amp_target floor for the CoT search (avoid motion collapse)
    static constexpr float kAmpSeekMax     = 0.60f;   // amp_target ceiling for the CoT search
    static constexpr float kHeightEmaAlpha = 0.01f;   // slow height estimate (spike-robust)
    static constexpr float kHeightBiasMin  = -0.5f;   // allow slight relax below neutral lift
    static constexpr float kHeightBiasMax  =  1.5f;   // cap lift authority
    static constexpr float kHeightLiftSign = +1.0f;   // hip2 command dir that RAISES chassis (flip if inverted)
    static constexpr float kPanicRampAlpha = 0.04f;   // smoothing of panic_ toward its hysteresis target
    static constexpr float kResetRateAlpha = 1.0f / 600.0f;  // Gate 0: ~600-tick (~10s@60Hz) smoothing of the disruption-rate EMA (a measurement constant, not a behavioral knob)
};

} // namespace ogma
