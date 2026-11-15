#pragma once
// StepExecutor -- the stepper's HAND: it executes a step intent (the body's shift over the planted toes, the lift, the swing, the
// lowering until the FSR says ground), holds the stance, solves the legs' IK, and integrates the sequencer's odometry.  It never
// chooses a step: at each decision it asks intent_hook_ (the scaffold planner today; the step arbiter from Phase 5) and applies the
// StepIntent it gets; a stuck leg's relief step is asked of relief_hook_.  Its members are public: the planner reads the stance
// (const) and the module's diag reads everything; the formal interface is the intent in and (Phase 2d) the step outcome out.
// Split out of PoseSequencer byte-identical (learned stepper plan Phase 2, 2026-11-15): every function body is the
// monolith's, moved, with its members qualified by owner; the golden traces (scripts_tools/stepper_battery.sh +
// stepper_ident.py) are the gate.  The member comments are the monolith's, kept with the member they document.
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include "ogma/body/LegSolver.hpp"
#include "ogma/modules/stepper/StepperCommon.hpp"
#include "ogma/modules/stepper/SupportFloor.hpp"

namespace ogma::stepper {

class StepExecutor {
public:
    StepExecutor(const StepperInputs& in, const SupportFloor& floor, LoadBook& book, body::LegSolver& kin, uint32_t& rng)
        : in_(in), floor_(floor), book_(book), kin_(kin), rng_(rng) {}
    StepExecutor(const StepExecutor&) = delete; StepExecutor& operator=(const StepExecutor&) = delete;

    void sense(uint64_t tick_id);        // the tick, and each toe's leaky bearing estimate (the FSR filtered)
    void engage_now(uint64_t tick_id);   // the stance from the gait's pose; Settle next
    void step_machine();                 // one tick of the phase machine
    void hold_stance();                  // the planted toes' heights
    void solve_ik();                     // the commanded toes to joint angles (and the IK error's EMA)
    void decide_next(uint64_t tick_id);  // the plan's state cleared, the tag's abandoned step counted, then the next intent applied
    void apply(const StepIntent& in);
    bool bearing(int i) const { return bearing_[size_t(i)]; }

    // the seams: who chooses the next step, who finds a relief step, and whether a fixed order exists (a spent shift passes the turn)
    std::function<StepIntent()> intent_hook_;
    std::function<bool(int stuck, double cx, double cz, int bearing, int& leg_out, Vec3f& land_out, double& margin_out)> relief_hook_;
    const std::vector<double>* order_ref_ = nullptr;

    const StepperInputs& in_; const SupportFloor& floor_; LoadBook& book_; body::LegSolver& kin_; uint32_t& rng_;   // shared: the module owns them

    int phase_max_ticks_ = 60;   // a LIFT or SWING that has not arrived by then goes on (an unreachable target must not stop the walk; 150 looked like a freeze at a wall)
    // THE OBSTACLE REFLEX and THE RESET (operator, 2026-11-08: "when the robot collides with one of the walls it freezes;
    // the leg that collides seems to get stuck; resetting the robot leaves the markers out of sync"): a swinging toe whose
    // contact flag comes back on after it left the ground has met something -- the swing ends and the toe is put down
    // where it is; and the body's reset event (events.reset / events.miss: a teleport, the inversion auto-reset, a fall)
    // drops the sequencer to idle at once, so it re-engages from the standing conditions with fresh targets.
    bool left_ground_ = false;
    int obstacle_run_ = 0;
    int64_t obstacle_hits_ = 0;
    double heading_gain_ = 0.6; double yaw_rate_max_ = 0.3; double heading_hold_ = 1.0;
    // the odometry's realisation: the body makes shift_realised of the travel its planted toes are commanded (measured 0.82
    // over four traces: the toes slip in the shift; the surge when a leg lifts and the rock back during its swing cancel to
    // a few centimetres).  A measured constant for now; TerrainMemory's registration by re-observation is the adaptive path.
    double shift_realised_ = 0.82;
    std::string heading_ref_ = "zero";   // "zero": the dead-reckoning frame's own heading (the body's facing at its anchor); "engage": the heading at engage (the trot's last stride left the body yawed 0.26 rad there, and the hold kept it -- 2026-11-08)
    double px_ = 0.0; double py_ = 0.0; double yaw_ref_ = 0.0; double yaw_err_ = 0.0;
    // THE STANCE: every planted toe is held at stand_h_m below the body origin in the BODY frame (equal legs: a level body on
    // flat ground), levelled by level_gain of the IMU's tilt (the low side's toes deeper), plus its own relative
    // elevation (measured at its touchdown against the toes already down, recentred so the body sits stand_h_m above
    // their mean) -- a level body at a constant height on flat ground, and on a riser the raised toe keeps the body level
    // as far as elev_max_m allows.  A landing point is kept within reach_max_m of the leg's hip1 (horizontally).
    double stand_h_m_ = 0.085; double level_gain_ = 0.5; double level_max_m_ = 0.02; double elev_max_m_ = 0.0; double unreach_tol_m_ = 0.015;   // reach_max_m: kin_.p
    // THE WORKSPACE (the second flat run, 2026-11-08: planted toes slid back without bound until the joints hit their
    // limits, and a landing outside the leg's reach left that toe hanging): every toe target is kept in a sector about
    // the leg's neutral heading from its hip -- within +/- sector_half_rad and a radius in [reach_min_m, reach_max_m].  A
    // planted toe that reaches the sector's edge during SHIFT blocks the shift (the body cannot advance further over its
    // feet without a step); a lowering toe out of reach retreats toward its hip by retreat_m before trying again.
    double retreat_m_ = 0.01;   // sector_half_rad, reach_min_m: kin_.p
    double land_sector_rad_ = 0.5;
    // THE LIFT'S GROUND (operator, 2026-11-08: "the rear legs never lift"; the trace: their lift targets 10 cm above the
    // stance, out of reach, from a patch behind the body that only touches feed and whose frame drifts under the crawl):
    // the ground under the toe NOW is where the toe stands (its own gravity height, truth), and the memory's height at
    // the landing point is believed only within [-land_band_lo_m, +land_band_hi_m] of the stance; outside it, the stance.
    double land_band_lo_m_ = 0.02; double land_band_hi_m_ = 0.09;
    int64_t land_h_rejected_ = 0;   // a LANDING point stays within this angle of the heading (the standing sector may be wider: a planted toe at 0.9 rad is fine, a swing to 0.9 rad times out)
    double neutral_r_m_ = 0.10;
    // SETTLE (2026-11-08, the engage trace: the stance inherited the trot's mid-swing toe, one leg swept 50 deg under
    // the tail, and the first lift from there rolled the body): on engage the four toes slide on the floor to their
    // neutral places at settle_v before the first step.
    double settle_v_ = 0.05;
    int64_t settle_ticks_ = 0;
    double settle_ = 1.0; double land_clamp_ = 1.0;
    int obstacle_ticks_ = 5;   // the prune's off switches (2026-11-14): the settle, the go-time landing clamps, the obstacle reflex
    int64_t turns_passed_ = 0;   // the sequencer's own stance: each toe's neutral place is this far from its hip along the leg's heading (the gait stands at 12-13 cm, at the edge of the reach, and left the shift no room -- 2026-11-08)
    int64_t shift_blocked_ = 0; int64_t retreats_ = 0;
    int blocked_leg_ = -1;
    // THE SHIFT'S DIRECTION (the third flat run, 2026-11-08: a shift straight back hit a workspace edge at once and the
    // body never advanced): the body moves toward the INTERIOR of the triangle the other three toes make (the inward
    // normal of the edge nearest the weight, by the margin still missing) and toward the four toes' centroid
    // (centre_gain of the offset), so the weight is carried over the support that will remain and the body follows the
    // feet it has placed ahead.  The move is bounded by shift_v per tick; the planted toes move the opposite way.
    double centre_gain_ = 0.5;
    // THE ADVANCE: each SHIFT also carries the body forward by advance_m (a quarter stride) before the leg may go, so the
    // crawl progresses by construction and not only by the centroid's pull (the fifth flat run, 2026-11-08: a shift that
    // ends the moment the margin is met moves the body millimetres).
    double advance_m_ = 0.015; double fwd_acc_ = 0.0;
    bool fwd_blocked_ = false;
    int64_t steps_no_advance_ = 0;
    int still_ticks_ = 0;
    bool centre_off_ = false;   // a shift that no longer moves (the margin's pull cancels the advance) has no advance to give   // the advance yields to the margin: a front leg's lift needs the weight BEHIND the diagonal, so the body advances with the rear legs' steps
    // THE STANCE SEEKS LOAD (the fourth flat run, 2026-11-08: a toe held a few millimetres above its ground read airborne
    // and was re-stepped forever -- a four-legged stance always has one short leg): a planted toe whose contact flag is
    // off creeps down at creep_v until it bears (at most creep_max_m), and relaxes back while it bears; a measured
    // elevation smaller than elev_deadband_m is noise, not a step; and "airborne" for the leg choice means the flag has
    // been off for air_ticks_min ticks despite the creep.
    // THE FSR, FILTERED (operator, 2026-11-08: "during the first step the weight is lifted from other legs briefly and
    // that resets the sequence"): a leaky estimate of each toe's bearing from the raw flag (bear_tau_ticks), read with
    // hysteresis (on above bear_on, off below bear_off) for everything but the landing; the landing itself needs the raw
    // flag on for touch_ticks in a row and, when a load is published, at least touch_load of the body's weight.
    double bear_tau_ticks_ = 8.0; double bear_on_ = 0.6; double bear_off_ = 0.3; double touch_load_ = 0.0;
    int touch_ticks_ = 3;
    std::array<double, 4> bear_{};
    std::array<bool, 4> bearing_{};
    int touch_run_ = 0;
    double creep_v_ = 0.02; double creep_max_m_ = 0.02; double creep_relax_v_ = 0.005; double elev_deadband_m_ = 0.03;
    int air_ticks_min_ = 100;
    // the elevation of each toe's ground comes from the TERRAIN MEMORY's patch (the ground under the toe against the mean
    // under the others), smoothed; the touch grades the memory (land_err), it does not set the stance (a touch that stops
    // early held one toe 2-3 cm high on flat ground and rolled the body, 2026-11-08)
    double elev_tau_ticks_ = 100.0;
    int64_t deadlock_ticks_ = 0;   // ticks spent in SHIFT with nothing to do and no leg safe to lift
    std::array<double, 4> creep_{};
    std::array<int, 4> air_ticks_{};
    std::array<int64_t, 4> last_step_tick_{};
    uint64_t tick_now_ = 0;
    double shift_dx_ = 0.0; double shift_dz_ = 0.0;   // the last tick's body move (m), for the diag
    std::array<double, 4> elev_{};   // each toe's relative elevation (m)
    double land_err_last_ = 0.0; double land_err_ms_ = 0.0;
    int64_t landings_ = 0; int64_t missed_ = 0;
    double shift_v_ = 0.03; double shift_max_m_ = 0.05; double lift_clear_m_ = 0.03; double lift_v_ = 0.10; double swing_v_ = 0.12; double lower_v_ = 0.06;
    double stride_m_ = 0.06; double sink_max_m_ = 0.06; double dt_ = 0.02;
    // THE PREDICTED GROUND USED AS A PRIOR (lever 1, 2026-11-08; both default off = byte-identical): the lowering descends at
    // lower_fast_v to a standoff above the memory's height at the landing (standoff = lower_standoff_m + lower_standoff_k x the
    // memory's sigma there), then at lower_v until the FSR; and a landing point whose remembered neighbourhood has a relief of
    // land_edge_m or more (a riser's edge) is moved along the heading, a cell at a time within reach, to flat remembered ground.
    double lower_fast_v_ = 0.0; double lower_standoff_m_ = 0.01; double lower_standoff_k_ = 2.0; double land_edge_m_ = 0.0; double land_edge_max_m_ = 0.04;
    int land_edge_cells_ = 3;
    double land_sigma_ = 0.0;
    int64_t fast_ticks_ = 0; int64_t slow_ticks_ = 0; int64_t edge_moves_ = 0; int64_t edge_stuck_ = 0;
    // THE CREEP ON LOAD (2026-11-08; default 0 = the contact flag, byte-identical): a planted toe whose FSR share is below
    // creep_load_min creeps down (the body stood on one diagonal with the fourth toe touching but unloaded, its flag flickering,
    // and the creep relaxed on each flicker: the diagonal's leg could never lift and the unloaded toe was re-stepped 42 times);
    // it relaxes only while its share is above creep_load_relax.
    double creep_load_min_ = 0.0; double creep_load_relax_ = 0.2;
    int64_t creep_load_ticks_ = 0;
    // THE ADVANCE YIELDS TO A STRETCHED LEG (2026-11-08; 0 = off, byte-identical): the body does not advance while a planted toe is
    // within advance_reach_m of its reach limit (seed 11: the body walked away from the rear-right toe until it stood at the
    // reach limit, where it blocked every shift that would have made its own lift safe -- 84 steps of the front-left leg in place).
    double advance_reach_m_ = 0.0;
    int64_t advance_yields_ = 0;
    double relief_ = 0.0;
    int relief_for_ = -1;
    bool relief_pending_ = false;
    body::Vec3f relief_land_{};
    int64_t relief_steps_ = 0; int64_t relief_none_ = 0;
    std::array<bool, 4> blocked_toe_{};
    // LEVER 2b (2026-11-09): the stance by each toe's LANDED height -- elev_source "landed" holds a planted toe at the body-frame
    // height its command had when the FSR said ground (recentred zero-mean, so the body stands stand_h above the toes' mean
    // ground and parallel to their plane), where "patch" reads the memory under the toe (elev_max_m 0 = neither, the default).
    // At a 50 mm riser the body-frame stance pitched the body 19 deg nose-up and drove a front leg to both its stops.
    std::string elev_source_ = "patch";
    std::array<double, 4> elev_landed_{};
    int64_t lower_fold_ticks_ = 0;
    // THE CROUCH (lever 2b, 2026-11-09; crouch_max_m 0 = off, byte-identical): a lowering toe at its downward reach without
    // ground brings the BODY down to it -- the planted legs fold by crouch_v while the deficit lasts, up to crouch_max_m, and
    // relax back at crouch_relax_v while every toe bears (a front leg on a nose-up body missed the same landing 19 times at
    // the first riser, its target 15 cm under its hip; the error was the toe's reach, the actuator is the body's height).
    // PRUNE-P1 (learned stepper Phase 1, 2026-11-14; operator: keep until the cleanup pass, then delete and check byte identity): the re-seed is redundant -- the
    // multistart IK covers it (ledger 2026-11-14 "Learned stepper Phase 1"); off in every config. Its removal takes with it
    // miss_reacquire's walk (it sets reseeding_, which only this walk reads) and the params ik_reseed_ticks / ik_reseed_rate_rad.
    // THE RE-SEED (2026-11-09; ik_reseed_ticks 0 = off, byte-identical): a leg whose IK sits at a joint stop with its toe far from
    // its target for this many ticks is in the wrong branch (the femur folded UP for a target 15 cm down: the damped step is
    // pinned at the stop and cannot cross to the extended solution; a front leg missed the same landing 19 times, its real
    // joints wedged against the chassis) -- its angles are walked back to the neutral standing pose, from which the IK re-solves.
    // THE HUNG TOE (operator, 2026-11-09: "the right rear leg gets hung on the step: the robot shifts its weight forward so the
    // hung leg presses horizontally into the face instead of lifting clear; the lift forward drags the robot backward; the
    // robot could shift its weight back and free the leg; this state would be known if a swing forward moves the chassis
    // backward"). Two egocentric signs: the toe's contact flag never clears for hang_ticks after it is asked 2 cm up (it is
    // caught), and the IMU's forward acceleration integrated over the lift and swing says the body went back by drag_back_m.
    // The freeing move: the toe is set down, the body shifts back by free_back_m, then the lift is free_extra_m higher and
    // its top free_retract_m toward the hip (twice that on a second hang of the same leg). hang_ticks 0 = off.
    int hang_ticks_ = 0;
    double drag_back_m_ = 0.0; double free_back_m_ = 0.02; double free_extra_m_ = 0.02; double free_retract_m_ = 0.01;
    // the retract along the ground before the re-lift (the body shift back of the first form dragged the robot backward and it fell 8 times in 8 runs)
    int hang_run_ = 0; int free_pending_ = 0;
    double free_retract_left_ = 0.0;
    std::array<int, 4> hang_count_{};
    int64_t hangs_ = 0; int64_t hangs_by_contact_ = 0; int64_t hangs_by_drag_ = 0;
    double swing_v_fwd_ = 0.0; double swing_d_fwd_ = 0.0; double a_lin_z_ema_ = 0.0;   // the forward surge integrated within a lift+swing
    double ik_multistart_ = 0.0;   // ik_switch_m: kin_.p
    // LEVER B (2026-11-12; free_form 2 = the retract along the ground, the shipped form; 3 = the operator's reading): the caught
    // toe is UNLOADED by the ordinary margin shift asked for free_margin_m more than a step needs (for the rear-right that is
    // forward-left, the triangle's inward normal), the lift completes to its full height, free_extra_m higher, before any forward
    // motion, with its top free_out_m OUTWARD from the body (an arc that clears the step's edge), and a second hang of the same
    // leg adds a random perturbation of free_noise_m to the arc's height and side -- exploration as the escape from a repeated
    // identical failure, the controller otherwise deterministic. LEVER C (face_gap_m 0 = off): a planned landing with a rise of
    // land_edge_m within face_gap_m AHEAD of it is penalised by face_penalty_m -- a rear toe put down just short of a face is
    // the hang's cause.
    int free_form_ = 2;
    double free_margin_m_ = 0.015; double free_out_m_ = 0.02; double free_noise_m_ = 0.01; double free_out_dx_ = 0.0; double free_out_dy_ = 0.0;
    bool unload_shift_only_ = false;
    // THE LANDING-RELOCATION TAG (2026-11-14, learned stepper plan Phase 1; an instrument, deciding nothing): from the moment a
    // step goes, every move of the landing the stepping toe is headed for is credited to the rule that made it -- the reach clamp,
    // the landing sector's clamp, the edge move, the obstacle reflex, the lowering's retreat, the IK's shortfall at the touch, the
    // clip -- as a vector per rule; at the touch the landing's offset from where the step set out (the plan's landing for a
    // planned step) is the sum of those vectors (reloc_resid_, the closure, is 0 when every move is tagged).  reloc_n_ = [landings
    // tracked, planned, unplanned (no plan), unplanned (the leg switched after the plan), planned and off by > 2 cm, planned with a
    // hang, abandoned (a new decision before the touch: a hang's re-shift passed the turn)]; reloc_sum_[r] = summed |vector| of rule r over planned landings (m); reloc_top_[r] = planned landings off by > 2 cm
    // whose largest move was rule r; reloc_off_ = summed offset over planned landings (m).
    enum Reloc { RelReach = 0, RelSector, RelEdge, RelObstacle, RelRetreat, RelIk, RelClip, RelN };
    bool reloc_active_ = false; bool reloc_planned_ = false; bool reloc_switched_ = false; bool reloc_hang_ = false;
    int reloc_plan_leg_ = -1;
    body::Vec3f reloc_from_{};
    std::array<std::array<double, 2>, RelN> reloc_vec_{};
    std::array<int64_t, 7> reloc_n_{};
    std::array<double, RelN> reloc_sum_{};
    std::array<int64_t, RelN> reloc_top_{};
    double reloc_off_ = 0.0; double reloc_resid_ = 0.0;
    int clip_ticks_ = 0; int clip_run_ = 0;
    int64_t clips_ = 0;
    double miss_reacquire_ = 0.0;
    int miss_ticks_ = 0; int lower_retreats_ = 0;
    std::array<int, 4> miss_count_{};
    // a NULL STEP -- a landing within null_step_m of the toe's lift-off point (a clipped arc that went nowhere, touched down and
    // counted as a landing: one leg "landed" 141 times in place at 0.8 s each) -- counts as a miss; a leg with two or more misses
    // in a row is left out of the search for a wave so the body moves under it (unless no other leg has a step)
    double null_step_m_ = 0.01;
    body::Vec3f lift_from_{};
    int64_t null_steps_ = 0;
    double front_drop_gain_ = 0.0; double front_drop_max_m_ = 0.04; double front_drop_deadband_ = 0.05; double front_drop_ = 0.0; double depth_max_m_ = 0.0;
    int64_t depth_caps_ = 0;
    // the drop follows its target through a first-order filter: at 0.2 m/rad unfiltered the loop through the body's pitch
    // (a toe height over a 20 cm wheelbase) bobbed the planted front legs 2-6x (operator, 2026-11-13)
    double front_drop_tau_ticks_ = 1.0;
    // PRUNE-P1 (learned stepper Phase 1, 2026-11-14; operator: keep until the cleanup pass, then delete and check byte identity): the re-seed's state, below.
    int ik_reseed_ticks_ = 0;
    double ik_reseed_rate_ = 0.04;
    std::array<int, 4> pinned_ticks_{};
    int64_t reseeds_ = 0;
    std::array<bool, 4> reseeding_{};   // the walk back at ik_reseed_rate per tick (the step cap, 6 rad/s, flipped the body once when a planted leg snapped to neutral)
    double crouch_ = 0.0; double crouch_max_m_ = 0.0; double crouch_v_ = 0.03; double crouch_relax_v_ = 0.005;
    int64_t crouch_ticks_ = 0;
    bool plan_active_ = false; bool plan_pending_ = false;
    body::Vec3f plan_land_{};
    double plan_sx_ = 0.0; double plan_sz_ = 0.0; double plan_done_x_ = 0.0; double plan_done_z_ = 0.0; double plan_score_ = 0.0;
    int64_t plan_reached_ = 0;
    // state
    Phase phase_ = Phase::Idle;
    int leg_ = -1;
    std::array<std::array<double, 3>, 4> angles_{};   // the commanded joint angles
    std::array<body::Vec3f, 4> toe_tgt_{}; std::array<body::Vec3f, 4> neutral_{};   // targets and the neutral toe places (body frame)
    std::array<double, 4> ground_h_{};   // the gravity height of the ground under each planted toe (at its touchdown)
    double shift_acc_ = 0.0; double lift_h_ = 0.0; double land_h_ = 0.0; double lift_dy_ = 0.0;
    body::Vec3f land_{};
    int phase_ticks_ = 0;
    // instruments
    std::array<int, 4> steps_{};
    int64_t cycles_ = 0; int64_t shift_exhausted_ = 0; int64_t shift_ticks_ = 0; int64_t lower_sunk_ = 0; int64_t phase_timeouts_ = 0; int64_t engaged_at_ = -1;
    double ik_err_ema_ = 0.0;

    void reloc_add(int r, const Vec3f& before, const Vec3f& after) {
        if (!reloc_active_) return;
        reloc_vec_[size_t(r)][0] += double(after.x) - double(before.x); reloc_vec_[size_t(r)][1] += double(after.z) - double(before.z);
    }
    void reloc_finish(const Vec3f& landed);

    // THE STEP OUTCOME (Phase 2, 2026-11-15; an instrument, deciding nothing): each step's record, written at its touch
    StepOutcome outcome_; int64_t outcome_seq_ = 0; uint64_t go_tick_ = 0;
    void record_outcome(bool tracked, bool touched, double h_touch, const Vec3f& landed);
};

} // namespace ogma::stepper
