#pragma once
// PoseSequencer -- the careful stepper's SEQUENCER (2026-11-08): a pose-level motor controller beside the rhythmic gait,
// which owns the four legs while it is engaged and walks a statically supported CRAWL chosen one pose at a time from the
// support model and the terrain memory.
//
// WHY A SEQUENCER.  The lift gate (LoadPhaseGait margin_topic, ledger 2026-11-08) showed the trot cannot relax into a
// crawl: a leg refused its lift sits at the end of its stroke, nothing moves the body, the margins never change.  What
// the gate had no handle on is the body's TRANSLATION over the planted toes -- and that, plus one leg's lift at a time,
// is what this module commands.  It is the operator's "search over a sequence of full-body poses starting with the
// current pose": at each step the next pose is the one that reduces the support model's error (a leg may leave only when
// the others carry the weight) and places the free toe where the terrain memory says the ground is.
//
// THE LOOP (engaged):
//   SHIFT  all four toes planted; their body-frame targets slide backward at shift_v (the body advances over them) until
//          the chosen leg's lift margin (support.state) reaches margin_min_m -- or shift_max_m is used up, when the leg
//          the model rates safest is taken instead (counted: shift_exhausted);
//   LIFT   that toe rises to lift_clear_m above the higher of the ground under it and the ground at its landing point
//          (the terrain patch; a riser ahead lifts it higher);
//   SWING  it moves forward to its landing point (its neutral position plus half a stride) at swing_v, height held;
//   LOWER  it descends at lower_v until its contact flag reports ground (the FSR is the sensor of bearing) or it is
//          sink_max_m below the ground the memory predicted; then it is planted and the next leg is chosen -- the one
//          whose toe is furthest behind its neutral place, which yields the crawl's order on its own.
// Toe targets become joint angles by damped-least-squares inverse kinematics on the SAME forward kinematics the body
// publishes its toes with (LegKinematics.hpp, the anchors from reality.proprio.leg_model), and angles become the
// body's action units through the published action-to-angle map; the command reaches the servos through MotorEPMv2's
// pose_topic, blended by `engage`, so the gait's learner sees what was executed and the hand-back is a ramp.
//
// SAFETY.  The body tilting past tilt_disengage (up_body's vertical component) hands the legs back to the rhythm.
// engage 0 (the default) publishes engage 0: byte-identical.  Nothing is learned here yet; the module is judged on the
// graded flight by risers climbed, flips, the support margin and the FSR shares.
#include <array>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"
#include "ogma/body/LegKinematics.hpp"

namespace ogma {

class PoseSequencer : public Module {
public:
    PoseSequencer();
    ~PoseSequencer() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    double odom_x() const { return px_; } double odom_y() const { return py_; } double yaw_err() const { return yaw_err_; }
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    enum class Phase { Idle = 0, Shift, Lift, Swing, Lower, Settle };
    static constexpr int kModelFloats = 4 * 27 + 8;
    // ---- the kinematics, exposed for tests ----
    body::Vec3f toe_of(int leg, const std::array<double, 3>& t) const;          // body-frame toe for joint angles
    std::array<double, 3> angles_of_u(int leg, const std::array<double, 3>& u) const;
    std::array<double, 3> u_of_angles(int leg, const std::array<double, 3>& t) const;
    bool ik_step(int leg, const body::Vec3f& target, std::array<double, 3>& t) const;   // one damped step; true when within tol
    // THE MULTI-START SOLVE (lever A, 2026-11-12; ik_multistart 0 = off, byte-identical): damped least squares follows the local
    // gradient and from some poses the nearest solution is the wrong BRANCH -- the femur folded up for a target below, or the
    // knee hyperextended where it should bend (the operator's observation) -- and it then sits at a joint stop with a residual.
    // When the single-seed solve is left further than 2 x ik_tol from its target, the solve is restarted from a set of seeds that
    // cover the branches (the neutral standing pose, femur down / up, knee folded / extended) with the true forward kinematics,
    // and the best within the joint ranges is taken if it beats the current by ik_switch_m: "the leg tries the other signs".
    bool ik_solve(int leg, const body::Vec3f& target, std::array<double, 3>& t, bool allow_multistart);
    bool plan_step(int& leg_out, body::Vec3f& land_out, double& sx_out, double& sz_out, double& score_out);   // the pose search (lever 2a); public for the tests
    Phase phase() const { return phase_; }
    int   active_leg() const { return leg_; }
    float engage_level() const { return engage_level_; }
    int   steps(int leg) const { return steps_[size_t(leg)]; }
    bool  model_ready() const { return have_model_; }

private:
    void handle(std::string const& which, int idx, MessagePtr p);
    bool patch_h(double fwd, double lat, double& h) const;      // remembered ground height (gravity frame) under a body-frame point
    bool patch_h(double fwd, double lat, double& h, double& sigma) const;   // ... with its uncertainty (m) from the cells' precisions
    bool patch_edge(double fwd, double lat, double& relief) const;          // the height relief (max - min) of the remembered cells around a point: a riser's edge
    double gravity_h(const body::Vec3f& p) const { return p.x * up_[0] + p.y * up_[1] + p.z * up_[2]; }
    body::Vec3f at_gravity_h(double x, double z, double h) const { return body::Vec3f(float(x), float((h - x * up_[0] - z * up_[2]) / std::max(0.2, up_[1])), float(z)); }
    void engage_now(uint64_t tick_id);
    int  choose_leg();
    void step_machine();
    void publish(uint64_t tick_id);

    // config
    std::string model_topic_ = "reality.proprio.leg_model", support_topic_ = "support.state", patch_topic_ = "terrain.patch",
                feet_xz_topic_ = "reality.proprio.feet_xz", feet_y_topic_ = "reality.proprio.toe_y", up_topic_ = "reality.proprio.up_body", imu_topic_ = "reality.proprio.imu6",
                contact_topic_ = "reality.proprio.foot_contact", load_topic_ = "reality.proprio.foot_load", output_topic_ = "pose.targets";
    std::vector<std::string> gait_action_topics_;
    double engage_ = 0.0; int engage_ramp_ticks_ = 50;
    // ENGAGE ONLY FROM A STANDING BODY (the first flat run, 2026-11-08: engaged at tick 1 the sequencer froze the gait's
    // lying start pose, legs straight out, and the swing target was out of reach for the whole run): a minimum tick,
    // three toes in contact, and the belly clear of the ground when a belly topic is given.
    std::string belly_topic_ = "reality.proprio.belly_m"; double engage_min_belly_m_ = 0.045; int engage_min_tick_ = 200, engage_min_contacts_ = 3;
    int phase_max_ticks_ = 60;    // a LIFT or SWING that has not arrived by then goes on (an unreachable target must not stop the walk; 150 looked like a freeze at a wall)
    // THE OBSTACLE REFLEX and THE RESET (operator, 2026-11-08: "when the robot collides with one of the walls it freezes;
    // the leg that collides seems to get stuck; resetting the robot leaves the markers out of sync"): a swinging toe whose
    // contact flag comes back on after it left the ground has met something -- the swing ends and the toe is put down
    // where it is; and the body's reset event (events.reset / events.miss: a teleport, the inversion auto-reset, a fall)
    // drops the sequencer to idle at once, so it re-engages from the standing conditions with fresh targets.
    bool left_ground_ = false; int obstacle_run_ = 0; int64_t obstacle_hits_ = 0, resets_seen_ = 0;
    // THE HEADING HOLD and THE SEQUENCER'S ODOMETRY (operator, 2026-11-08).  The crawl drifted into walls because it had no
    // heading: two of four traces yawed 0.1-0.15 rad over 0.45 m and drifted 7-8 cm sideways with it.  During SHIFT the
    // planted toes are rotated about the body's vertical by heading_gain x the yaw error (the gyro-integrated heading from
    // the body's odometry against its value at engage), bounded by yaw_rate_max, so the body turns back.  And the terrain
    // memory's frame rode a STRIDE odometry the crawl does not drive: the sequencer knows the body's motion exactly (it
    // commands it over the planted toes), so it integrates its own pose -- forward and lateral in the dead-reckoning
    // frame (x forward at the anchor, y along the body's +X), yaw passed through from the body's -- and publishes it on
    // odom_out_topic, passing the body's odometry through while disengaged so the frame is continuous.
    std::string odom_in_topic_ = "reality.proprio.odom", odom_out_topic_ = "reality.proprio.odom_seq";
    double heading_gain_ = 0.6, yaw_rate_max_ = 0.3, heading_hold_ = 1.0;
    // the odometry's realisation: the body makes shift_realised of the travel its planted toes are commanded (measured 0.82
    // over four traces: the toes slip in the shift; the surge when a leg lifts and the rock back during its swing cancel to
    // a few centimetres).  A measured constant for now; TerrainMemory's registration by re-observation is the adaptive path.
    double shift_realised_ = 0.82;
    std::string heading_ref_ = "zero";   // "zero": the dead-reckoning frame's own heading (the body's facing at its anchor); "engage": the heading at engage (the trot's last stride left the body yawed 0.26 rad there, and the hold kept it -- 2026-11-08)
    std::array<double, 3> odom_in_{0.0, 0.0, 0.0}; bool have_odom_in_ = false;
    double px_ = 0.0, py_ = 0.0, yaw_ref_ = 0.0, yaw_err_ = 0.0; bool odom_anchored_ = false;
    // THE STANCE: every planted toe is held at stand_h_m below the body origin in the BODY frame (equal legs: a level body on
    // flat ground), levelled by level_gain of the IMU's tilt (the low side's toes deeper), plus its own relative
    // elevation (measured at its touchdown against the toes already down, recentred so the body sits stand_h_m above
    // their mean) -- a level body at a constant height on flat ground, and on a riser the raised toe keeps the body level
    // as far as elev_max_m allows.  A landing point is kept within reach_max_m of the leg's hip1 (horizontally).
    double stand_h_m_ = 0.085, level_gain_ = 0.5, level_max_m_ = 0.02, elev_max_m_ = 0.0, reach_max_m_ = 0.14, unreach_tol_m_ = 0.015;
    // THE WORKSPACE (the second flat run, 2026-11-08: planted toes slid back without bound until the joints hit their
    // limits, and a landing outside the leg's reach left that toe hanging): every toe target is kept in a sector about
    // the leg's neutral heading from its hip -- within +/- sector_half_rad and a radius in [reach_min_m, reach_max_m].  A
    // planted toe that reaches the sector's edge during SHIFT blocks the shift (the body cannot advance further over its
    // feet without a step); a lowering toe out of reach retreats toward its hip by retreat_m before trying again.
    double sector_half_rad_ = 0.9, reach_min_m_ = 0.06, retreat_m_ = 0.01;
    double land_sector_rad_ = 0.5;
    // THE LIFT'S GROUND (operator, 2026-11-08: "the rear legs never lift"; the trace: their lift targets 10 cm above the
    // stance, out of reach, from a patch behind the body that only touches feed and whose frame drifts under the crawl):
    // the ground under the toe NOW is where the toe stands (its own gravity height, truth), and the memory's height at
    // the landing point is believed only within [-land_band_lo_m, +land_band_hi_m] of the stance; outside it, the stance.
    double land_band_lo_m_ = 0.02, land_band_hi_m_ = 0.09; int64_t land_h_rejected_ = 0;   // a LANDING point stays within this angle of the heading (the standing sector may be wider: a planted toe at 0.9 rad is fine, a swing to 0.9 rad times out)
    double neutral_r_m_ = 0.10;
    // SETTLE (2026-11-08, the engage trace: the stance inherited the trot's mid-swing toe, one leg swept 50 deg under
    // the tail, and the first lift from there rolled the body): on engage the four toes slide on the floor to their
    // neutral places at settle_v before the first step.
    double settle_v_ = 0.05; int64_t settle_ticks_ = 0;
    // THE ORDER (2026-11-08, after six batteries of the greedy choice starving the rear legs 5:1): a FIXED crawl order,
    // rear-left, front-left, rear-right, front-right (a wave gait: each side's rear then its front), with the model as
    // the gate -- the body sways / advances until the leg whose turn it is has its margin, and if the shift is spent
    // without one the turn passes (counted).  A toe the creep could not bring to ground is still placed first.
    // order empty = the greedy choice (the first form).
    std::vector<double> order_ = {2, 0, 3, 1}; int order_pos_ = -1; int64_t turns_passed_ = 0;   // the sequencer's own stance: each toe's neutral place is this far from its hip along the leg's heading (the gait stands at 12-13 cm, at the edge of the reach, and left the shift no room -- 2026-11-08)
    int64_t shift_blocked_ = 0, retreats_ = 0; int blocked_leg_ = -1;
    bool sector_clamp(int leg, body::Vec3f& p) const;   // true when clamped
    // THE RELIEF STEP (2026-11-08; relief 0 = off, byte-identical): the leg whose turn it is stands at its reach limit and
    // blocks the very shift that would make its lift safe (seed 11: the rear-right toe put down short by a brush, the body
    // advancing past it until every shift front-left was clamped by it; 84 steps of the front-left leg in place) -- another
    // leg, safe to lift now, steps along its heading to the place that gives the stuck leg its margin WITHOUT a shift (the
    // rear mate stepping back rotates the support diagonal); the stuck leg's turn is retried when it lands.
    // THE POSE SEARCH (lever 2a, 2026-11-09; plan 0 = off, byte-identical): at each decision the leg, its landing and the body's
    // shift are chosen TOGETHER one step ahead -- the shift must keep every planted toe within reach and give the leg its
    // margin; the landing must leave the NEXT leg of the wave liftable by some feasible shift (the stall of 2026-11-08 was a
    // toe at its reach limit clamping every shift that would have freed it); among the feasible, the most forward shift and
    // the landing that leaves the next leg the largest margin, with the wave's order kept where it can be.
    double best_margin_over_shifts(int leg, const std::array<body::Vec3f, 4>& toes, double cx, double cz, double& sx, double& sz, bool forward_first) const;
    bool shift_feasible(int lifting, const std::array<body::Vec3f, 4>& toes, double sx, double sz) const;
    void decide_next(uint64_t tick_id);
    bool plan_relief(int stuck, double cx, double cz, int bearing, int& leg_out, body::Vec3f& land_out, double& margin_out) const;
    // THE SHIFT'S DIRECTION (the third flat run, 2026-11-08: a shift straight back hit a workspace edge at once and the
    // body never advanced): the body moves toward the INTERIOR of the triangle the other three toes make (the inward
    // normal of the edge nearest the weight, by the margin still missing) and toward the four toes' centroid
    // (centre_gain of the offset), so the weight is carried over the support that will remain and the body follows the
    // feet it has placed ahead.  The move is bounded by shift_v per tick; the planted toes move the opposite way.
    double centre_gain_ = 0.5, margin_extra_m_ = 0.005;
    // THE ADVANCE: each SHIFT also carries the body forward by advance_m (a quarter stride) before the leg may go, so the
    // crawl progresses by construction and not only by the centroid's pull (the fifth flat run, 2026-11-08: a shift that
    // ends the moment the margin is met moves the body millimetres).
    double advance_m_ = 0.015, fwd_acc_ = 0.0; bool fwd_blocked_ = false; int64_t steps_no_advance_ = 0; int still_ticks_ = 0; bool centre_off_ = false;   // a shift that no longer moves (the margin's pull cancels the advance) has no advance to give   // the advance yields to the margin: a front leg's lift needs the weight BEHIND the diagonal, so the body advances with the rear legs' steps
    // THE STANCE SEEKS LOAD (the fourth flat run, 2026-11-08: a toe held a few millimetres above its ground read airborne
    // and was re-stepped forever -- a four-legged stance always has one short leg): a planted toe whose contact flag is
    // off creeps down at creep_v until it bears (at most creep_max_m), and relaxes back while it bears; a measured
    // elevation smaller than elev_deadband_m is noise, not a step; and "airborne" for the leg choice means the flag has
    // been off for air_ticks_min ticks despite the creep.
    // THE FSR, FILTERED (operator, 2026-11-08: "during the first step the weight is lifted from other legs briefly and
    // that resets the sequence"): a leaky estimate of each toe's bearing from the raw flag (bear_tau_ticks), read with
    // hysteresis (on above bear_on, off below bear_off) for everything but the landing; the landing itself needs the raw
    // flag on for touch_ticks in a row and, when a load is published, at least touch_load of the body's weight.
    double bear_tau_ticks_ = 8.0, bear_on_ = 0.6, bear_off_ = 0.3, touch_load_ = 0.0; int touch_ticks_ = 3;
    std::array<double, 4> bear_{}; std::array<bool, 4> bearing_{}; int touch_run_ = 0;
    bool bearing(int i) const { return bearing_[size_t(i)]; }
    double creep_v_ = 0.02, creep_max_m_ = 0.02, creep_relax_v_ = 0.005, elev_deadband_m_ = 0.03; int air_ticks_min_ = 100;
    // the elevation of each toe's ground comes from the TERRAIN MEMORY's patch (the ground under the toe against the mean
    // under the others), smoothed; the touch grades the memory (land_err), it does not set the stance (a touch that stops
    // early held one toe 2-3 cm high on flat ground and rolled the body, 2026-11-08)
    double elev_tau_ticks_ = 100.0; int64_t deadlock_ticks_ = 0;   // ticks spent in SHIFT with nothing to do and no leg safe to lift
    std::array<double, 4> creep_{}; std::array<int, 4> air_ticks_{}; mutable std::array<double, 4> score_{}; std::array<int64_t, 4> last_step_tick_{}; uint64_t tick_now_ = 0;
    double shift_dx_ = 0.0, shift_dz_ = 0.0;   // the last tick's body move (m), for the diag
    static double tri_margin(const std::array<std::array<double, 2>, 3>& p, double cx, double cz, double& nx, double& nz);
    std::array<double, 4> elev_{};   // each toe's relative elevation (m)
    double land_err_last_ = 0.0, land_err_ms_ = 0.0; int64_t landings_ = 0, missed_ = 0;
    float belly_ = 0.0f; bool have_belly_ = false;
    double shift_v_ = 0.03, shift_max_m_ = 0.05, margin_min_m_ = 0.005, lift_clear_m_ = 0.03, lift_v_ = 0.10, swing_v_ = 0.12, lower_v_ = 0.06;
    double stride_m_ = 0.06, sink_max_m_ = 0.06, tilt_disengage_ = 0.75, dt_ = 0.02;
    // THE PREDICTED GROUND USED AS A PRIOR (lever 1, 2026-11-08; both default off = byte-identical): the lowering descends at
    // lower_fast_v to a standoff above the memory's height at the landing (standoff = lower_standoff_m + lower_standoff_k x the
    // memory's sigma there), then at lower_v until the FSR; and a landing point whose remembered neighbourhood has a relief of
    // land_edge_m or more (a riser's edge) is moved along the heading, a cell at a time within reach, to flat remembered ground.
    double lower_fast_v_ = 0.0, lower_standoff_m_ = 0.01, lower_standoff_k_ = 2.0, land_edge_m_ = 0.0, land_edge_max_m_ = 0.04; int land_edge_cells_ = 3;
    double land_sigma_ = 0.0; int64_t fast_ticks_ = 0, slow_ticks_ = 0, edge_moves_ = 0, edge_stuck_ = 0;
    // THE CREEP ON LOAD (2026-11-08; default 0 = the contact flag, byte-identical): a planted toe whose FSR share is below
    // creep_load_min creeps down (the body stood on one diagonal with the fourth toe touching but unloaded, its flag flickering,
    // and the creep relaxed on each flicker: the diagonal's leg could never lift and the unloaded toe was re-stepped 42 times);
    // it relaxes only while its share is above creep_load_relax.
    double creep_load_min_ = 0.0, creep_load_relax_ = 0.2; int64_t creep_load_ticks_ = 0;
    // THE ADVANCE YIELDS TO A STRETCHED LEG (2026-11-08; 0 = off, byte-identical): the body does not advance while a planted toe is
    // within advance_reach_m of its reach limit (seed 11: the body walked away from the rear-right toe until it stood at the
    // reach limit, where it blocked every shift that would have made its own lift safe -- 84 steps of the front-left leg in place).
    double advance_reach_m_ = 0.0; int64_t advance_yields_ = 0;
    double relief_ = 0.0, relief_step_m_ = 0.01; int relief_cells_ = 4; int relief_for_ = -1; bool relief_pending_ = false; body::Vec3f relief_land_{}; int64_t relief_steps_ = 0, relief_none_ = 0;
    std::array<bool, 4> blocked_toe_{};
    // LEVER 2b (2026-11-09): the stance by each toe's LANDED height -- elev_source "landed" holds a planted toe at the body-frame
    // height its command had when the FSR said ground (recentred zero-mean, so the body stands stand_h above the toes' mean
    // ground and parallel to their plane), where "patch" reads the memory under the toe (elev_max_m 0 = neither, the default).
    // At a 50 mm riser the body-frame stance pitched the body 19 deg nose-up and drove a front leg to both its stops.
    std::string elev_source_ = "patch"; std::array<double, 4> elev_landed_{}; int64_t lower_fold_ticks_ = 0;
    // THE CROUCH (lever 2b, 2026-11-09; crouch_max_m 0 = off, byte-identical): a lowering toe at its downward reach without
    // ground brings the BODY down to it -- the planted legs fold by crouch_v while the deficit lasts, up to crouch_max_m, and
    // relax back at crouch_relax_v while every toe bears (a front leg on a nose-up body missed the same landing 19 times at
    // the first riser, its target 15 cm under its hip; the error was the toe's reach, the actuator is the body's height).
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
    int hang_ticks_ = 0; double drag_back_m_ = 0.0, free_back_m_ = 0.02, free_extra_m_ = 0.02, free_retract_m_ = 0.01;
    // the retract along the ground before the re-lift (the body shift back of the first form dragged the robot backward and it fell 8 times in 8 runs)
    int hang_run_ = 0, free_pending_ = 0; double free_retract_left_ = 0.0; std::array<int, 4> hang_count_{}; int64_t hangs_ = 0, hangs_by_contact_ = 0, hangs_by_drag_ = 0;
    std::array<float, 6> imu_{}; bool have_imu_ = false; double swing_v_fwd_ = 0.0, swing_d_fwd_ = 0.0, a_lin_z_ema_ = 0.0;   // the forward surge integrated within a lift+swing
    double ik_multistart_ = 0.0, ik_switch_m_ = 0.005;
    // LEVER B (2026-11-12; free_form 2 = the retract along the ground, the shipped form; 3 = the operator's reading): the caught
    // toe is UNLOADED by the ordinary margin shift asked for free_margin_m more than a step needs (for the rear-right that is
    // forward-left, the triangle's inward normal), the lift completes to its full height, free_extra_m higher, before any forward
    // motion, with its top free_out_m OUTWARD from the body (an arc that clears the step's edge), and a second hang of the same
    // leg adds a random perturbation of free_noise_m to the arc's height and side -- exploration as the escape from a repeated
    // identical failure, the controller otherwise deterministic. LEVER C (face_gap_m 0 = off): a planned landing with a rise of
    // land_edge_m within face_gap_m AHEAD of it is penalised by face_penalty_m -- a rear toe put down just short of a face is
    // the hang's cause.
    int free_form_ = 2; double free_margin_m_ = 0.015, free_out_m_ = 0.02, free_noise_m_ = 0.01, free_out_dx_ = 0.0, free_out_dy_ = 0.0; uint32_t rng_ = 12345u;
    double face_gap_m_ = 0.0, face_penalty_m_ = 0.02; int64_t face_penalties_ = 0;
    // LEVER 3 (operator, 2026-11-12: "on the high steps the chassis tilting backwards moves the centre of gravity back and the rear
    // legs cannot be lifted; the front planted legs should lower the chassis in the front"): the FRONT DROP -- the front toes are
    // held front_drop_gain x the nose-up pitch beyond front_drop_deadband shallower (not zero-mean: the front of the chassis
    // comes down), bounded by front_drop_max_m; and the DEPTH CAP -- no toe is asked more than depth_max_m below the body (the
    // freezes at the 38 and 64 mm risers had a rear toe asked 13-14 cm down on a nose-up body: the leg straightened to its pole,
    // the real knee hyperextended against its stop and the servos could not bring it back; the brain has no joint feedback).
    // CLIP AND RE-ACQUIRE (operator, 2026-11-12: "why can't we assume the commanded joint angles are the real joint angles and
    // clip (a partial miss of the target) or attempt re-acquisition of the target?"): clip_ticks -- a lift or swing whose commanded
    // toe has been more than unreach_tol_m from its target for this many ticks takes the part of the arc it reached and moves on
    // (the body froze while a leg waited out its timeouts: only SHIFT moves the body); miss_reacquire -- a missed landing walks the
    // leg to its neutral pose (the re-seed's walk), a lowering gives up after miss_ticks or six retreats instead of 4.8 s, and the
    // leg's next attempt is DIFFERENT: its landing grid perturbed by up to 2 cm, its lift 1 cm lower per miss, the leg penalised
    // in the search by 1 cm of margin per miss so the others step first; the count clears at a landing that touched.
    // THE UNLOADED TOE (2026-11-13; unloaded_shift_m 0 = off, byte-identical): a planted toe that touches but has carried less
    // than unloaded_share of the weight for unloaded_ticks (the body resting on the other diagonal, the front toes tucked at
    // their inner reach: one leg "landed" 134 times in place at 0.8 s each, the only leg the search found feasible) -- before
    // any leg lifts, the body shifts unloaded_shift_m toward that toe (a planned shift, then a fresh decision).
    double unloaded_shift_m_ = 0.0, unloaded_share_ = 0.08; int unloaded_ticks_ = 50; std::array<int, 4> unloaded_run_{}; std::array<int, 4> unload_cool_{};
    bool unload_shift_only_ = false; int64_t unload_shifts_ = 0; std::array<double, 4> land_max_load_{}; int last_landed_ = -1; std::array<int, 4> unloaded_landings_{}; int unloaded_landings_min_ = 2; int last_plan_skips_ = 0, unloaded_skips_min_ = 2;   // the time rule fires only when the last search had to skip this many legs (the unloaded toe blocking the others; a toe lightly loaded on a free crawl is left alone)   // consecutive landings of a leg that never bore weight; the rule fires at the second (one is common and benign)   // the most weight a toe has carried since its landing; the leg that just landed
    // THE LANDING RADIUS (2026-11-13; land_r_min_m 0 = off, byte-identical): a planned landing is kept at least this far from its
    // hip -- the search rewards advance and not base width, and a stance collapsed inward (both right toes at the inner edge of
    // their reach) left every body shift infeasible and one leg landing 134 times at that edge while the body stood.
    double land_r_min_m_ = 0.0; int64_t land_r_rejects_ = 0;
    int clip_ticks_ = 0, clip_run_ = 0; int64_t clips_ = 0; double miss_reacquire_ = 0.0; int miss_ticks_ = 0, lower_retreats_ = 0; std::array<int, 4> miss_count_{};
    // a NULL STEP -- a landing within null_step_m of the toe's lift-off point (a clipped arc that went nowhere, touched down and
    // counted as a landing: one leg "landed" 141 times in place at 0.8 s each) -- counts as a miss; a leg with two or more misses
    // in a row is left out of the search for a wave so the body moves under it (unless no other leg has a step)
    double null_step_m_ = 0.01; body::Vec3f lift_from_{}; int64_t null_steps_ = 0, miss_skips_ = 0;
    double front_drop_gain_ = 0.0, front_drop_max_m_ = 0.04, front_drop_deadband_ = 0.05, front_drop_ = 0.0, depth_max_m_ = 0.0; int64_t depth_caps_ = 0;
    // the drop follows its target through a first-order filter: at 0.2 m/rad unfiltered the loop through the body's pitch
    // (a toe height over a 20 cm wheelbase) bobbed the planted front legs 2-6x (operator, 2026-11-13)
    double front_drop_tau_ticks_ = 1.0; int64_t multistarts_ = 0, branch_switches_ = 0;
    int ik_reseed_ticks_ = 0; double ik_reseed_rate_ = 0.04; std::array<int, 4> pinned_ticks_{}; int64_t reseeds_ = 0; std::array<bool, 4> reseeding_{};   // the walk back at ik_reseed_rate per tick (the step cap, 6 rad/s, flipped the body once when a planted leg snapped to neutral)
    double crouch_ = 0.0, crouch_max_m_ = 0.0, crouch_v_ = 0.03, crouch_relax_v_ = 0.005; int64_t crouch_ticks_ = 0;
    double plan_ = 0.0, plan_shift_m_ = 0.03, plan_shift_fwd_m_ = 0.04, plan_land_m_ = 0.04, plan_cell_m_ = 0.01, plan_adv_w_ = 0.3;
    bool plan_active_ = false, plan_pending_ = false; body::Vec3f plan_land_{}; double plan_sx_ = 0.0, plan_sz_ = 0.0, plan_done_x_ = 0.0, plan_done_z_ = 0.0, plan_score_ = 0.0;
    int64_t plans_ = 0, plan_skips_ = 0, plan_none_ = 0, plan_reached_ = 0;
    int    ik_iters_ = 4; double ik_damping_m_ = 0.01, ik_step_max_rad_ = 0.12, ik_tol_m_ = 0.004;

    // the leg model
    std::array<body::LegAnchors, 4> anchors_{}; std::array<body::Vec3f, 4> toe_off_{};
    double hip1_rest_ = 0, hip1_range_ = 1.4, hip2_rest_ = 0, hip2_range_ = 1.4, knee_rest_ = -1.6, knee_fold_ = 3.2, knee_hyper_ = 0.85; bool knee_widening_ = true;
    bool have_model_ = false;
    // inputs
    std::array<float, 26> support_{}; bool have_support_ = false;
    std::vector<float> patch_; int N_ = 0; double cell_ = 0.03; bool have_patch_ = false;
    std::array<float, 8> xz_{}; std::array<float, 4> y_{}, contact_{}, load_{}; std::array<double, 3> up_{0.0, 1.0, 0.0};
    bool have_xz_ = false, have_y_ = false, have_contact_ = false, have_load_ = false, have_up_ = false;
    std::array<float, 12> gait_u_{}; bool have_gait_ = false;

    // state
    Phase phase_ = Phase::Idle; int leg_ = -1; float engage_level_ = 0.0f;
    std::array<std::array<double, 3>, 4> angles_{};            // the commanded joint angles
    std::array<body::Vec3f, 4> toe_tgt_{}, neutral_{};          // targets and the neutral toe places (body frame)
    std::array<double, 4> ground_h_{};                           // the gravity height of the ground under each planted toe (at its touchdown)
    body::Vec3f hip1_of(int leg) const { return anchors_[size_t(leg)].hip1_world; }
    void hold_stance();
    double shift_acc_ = 0.0, lift_h_ = 0.0, land_h_ = 0.0, lift_dy_ = 0.0; body::Vec3f land_{};
    int phase_ticks_ = 0;
    // instruments
    std::array<int, 4> steps_{}; int64_t cycles_ = 0, shift_exhausted_ = 0, shift_ticks_ = 0, disengaged_tilt_ = 0, lower_sunk_ = 0, phase_timeouts_ = 0, engaged_at_ = -1;
    double ik_err_ema_ = 0.0, fk_err_ema_ = 0.0; int64_t ticks_engaged_ = 0;
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
