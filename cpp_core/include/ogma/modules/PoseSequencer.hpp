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
    Phase phase() const { return phase_; }
    int   active_leg() const { return leg_; }
    float engage_level() const { return engage_level_; }
    int   steps(int leg) const { return steps_[size_t(leg)]; }
    bool  model_ready() const { return have_model_; }

private:
    void handle(std::string const& which, int idx, MessagePtr p);
    bool patch_h(double fwd, double lat, double& h) const;      // remembered ground height (gravity frame) under a body-frame point
    double gravity_h(const body::Vec3f& p) const { return p.x * up_[0] + p.y * up_[1] + p.z * up_[2]; }
    body::Vec3f at_gravity_h(double x, double z, double h) const { return body::Vec3f(float(x), float((h - x * up_[0] - z * up_[2]) / std::max(0.2, up_[1])), float(z)); }
    void engage_now(uint64_t tick_id);
    int  choose_leg();
    void step_machine();
    void publish(uint64_t tick_id);

    // config
    std::string model_topic_ = "reality.proprio.leg_model", support_topic_ = "support.state", patch_topic_ = "terrain.patch",
                feet_xz_topic_ = "reality.proprio.feet_xz", feet_y_topic_ = "reality.proprio.toe_y", up_topic_ = "reality.proprio.up_body",
                contact_topic_ = "reality.proprio.foot_contact", load_topic_ = "reality.proprio.foot_load", output_topic_ = "pose.targets";
    std::vector<std::string> gait_action_topics_;
    double engage_ = 0.0; int engage_ramp_ticks_ = 50;
    // ENGAGE ONLY FROM A STANDING BODY (the first flat run, 2026-11-08: engaged at tick 1 the sequencer froze the gait's
    // lying start pose, legs straight out, and the swing target was out of reach for the whole run): a minimum tick,
    // three toes in contact, and the belly clear of the ground when a belly topic is given.
    std::string belly_topic_ = "reality.proprio.belly_m"; double engage_min_belly_m_ = 0.045; int engage_min_tick_ = 200, engage_min_contacts_ = 3;
    int phase_max_ticks_ = 150;   // a LIFT or SWING that has not arrived by then goes on (an unreachable target must not stop the walk)
    // THE STANCE: every planted toe is held at stand_h_m below the body origin in the BODY frame (equal legs: a level body on
    // flat ground), levelled by level_gain of the IMU's tilt (the low side's toes deeper), plus its own relative
    // elevation (measured at its touchdown against the toes already down, recentred so the body sits stand_h_m above
    // their mean) -- a level body at a constant height on flat ground, and on a riser the raised toe keeps the body level
    // as far as elev_max_m allows.  A landing point is kept within reach_max_m of the leg's hip1 (horizontally).
    double stand_h_m_ = 0.085, level_gain_ = 0.5, elev_max_m_ = 0.0, reach_max_m_ = 0.14, unreach_tol_m_ = 0.015;
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
    double land_band_lo_m_ = 0.04, land_band_hi_m_ = 0.09; int64_t land_h_rejected_ = 0;   // a LANDING point stays within this angle of the heading (the standing sector may be wider: a planted toe at 0.9 rad is fine, a swing to 0.9 rad times out)
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
