#pragma once
// FROZEN 2026-11-14 (operator): this module is a SCAFFOLD to be de-scaffolded onto a learned stepper; no new levers.
// Plan of record: docs/plans-and-designs/learned_stepper_plan.md (phases, gates, stop-and-ask points). Its remaining jobs
// are experience generator, fallback by precision and baseline; changes here are prunes (Phase 1, operator-approved) and
// golden-trace-identical splits (Phase 2), checked with scripts_tools/stepper_battery.sh + stepper_ident.py.
//
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
// THE SPLIT (learned stepper plan Phase 2, 2026-11-15): this module is now the shell -- its topics, params and diag -- over
// the components in modules/stepper/: StepExecutor (the phases, the stance, the IK), ScaffoldPlanner (the pose search, emitting
// StepIntents), SupportFloor (the lift veto, the tilt disengage, the standing check), and body::LegSolver (the leg model).
// Byte-identical to the monolith on the golden traces; the comments below describe the behaviour, wherever its code now lives.
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
#include "ogma/body/LegSolver.hpp"
#include "ogma/modules/stepper/AfterTouch.hpp"
#include "ogma/modules/stepper/LandingOffset.hpp"
#include "ogma/modules/stepper/ScaffoldPlanner.hpp"
#include "ogma/modules/stepper/StepExecutor.hpp"
#include "ogma/modules/stepper/StepperCommon.hpp"
#include "ogma/modules/stepper/SupportFloor.hpp"

namespace ogma {

class PoseSequencer : public Module {
public:
    PoseSequencer();
    ~PoseSequencer() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    double odom_x() const { return executor_.px_; } double odom_y() const { return executor_.py_; } double yaw_err() const { return executor_.yaw_err_; }
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;
    nlohmann::json snapshot_state() const override;   // the learned layers' models only (the landing offset, the after-touch record), each only when it is on (the scaffold itself carries nothing)
    void restore_state(nlohmann::json const&) override;

    using Phase = stepper::Phase;
    static constexpr int kModelFloats = body::LegSolver::kModelFloats;
    // ---- the components and the kinematics, exposed for tests ----
    const body::LegSolver& kinematics() const { return kin_; }
    const stepper::StepExecutor& executor() const { return executor_; }
    const stepper::ScaffoldPlanner& planner() const { return planner_; }
    const stepper::LandingOffset& offset() const { return offset_; }
    const stepper::AfterTouch& after() const { return after_; }
    body::Vec3f toe_of(int leg, const std::array<double, 3>& t) const { return kin_.toe_of(leg, t); }   // body-frame toe for joint angles
    std::array<double, 3> angles_of_u(int leg, const std::array<double, 3>& u) const { return kin_.angles_of_u(leg, u); }
    std::array<double, 3> u_of_angles(int leg, const std::array<double, 3>& t) const { return kin_.u_of_angles(leg, t); }
    bool ik_step(int leg, const body::Vec3f& target, std::array<double, 3>& t) const { return kin_.ik_step(leg, target, t); }   // one damped step; true when within tol
    bool ik_solve(int leg, const body::Vec3f& target, std::array<double, 3>& t, bool allow_multistart) { return kin_.ik_solve(leg, target, t, allow_multistart); }
    bool plan_step(int& leg_out, body::Vec3f& land_out, double& sx_out, double& sz_out, double& score_out) { return planner_.plan_step(leg_out, land_out, sx_out, sz_out, score_out); }   // the pose search; public for the tests
    double ext_slack(int leg, const body::Vec3f& p) const { return kin_.ext_slack(leg, p); }
    double ext_slack_fk(int leg) const { return kin_.ext_slack_fk(leg, executor_.angles_[size_t(leg)]); }
    bool shift_feasible(int lifting, const std::array<body::Vec3f, 4>& toes, double sx, double sz) const { return planner_.shift_feasible(lifting, toes, sx, sz); }
    int shift_refusal(int lifting, const std::array<body::Vec3f, 4>& toes, double sx, double sz, int& toe) const { return planner_.shift_refusal(lifting, toes, sx, sz, toe); }
    Phase phase() const { return executor_.phase_; }
    int   active_leg() const { return executor_.leg_; }
    float engage_level() const { return engage_level_; }
    int   steps(int leg) const { return executor_.steps_[size_t(leg)]; }
    bool  model_ready() const { return kin_.ready(); }

private:
    void handle(std::string const& which, int idx, MessagePtr p);
    void publish(uint64_t tick_id);

    // config
    std::string model_topic_ = "reality.proprio.leg_model"; std::string support_topic_ = "support.state"; std::string patch_topic_ = "terrain.patch"; std::string feet_xz_topic_ = "reality.proprio.feet_xz"; std::string feet_y_topic_ = "reality.proprio.toe_y"; std::string up_topic_ = "reality.proprio.up_body"; std::string imu_topic_ = "reality.proprio.imu6"; std::string contact_topic_ = "reality.proprio.foot_contact"; std::string load_topic_ = "reality.proprio.foot_load"; std::string output_topic_ = "pose.targets";
    std::vector<std::string> gait_action_topics_;
    double engage_ = 0.0;
    int engage_ramp_ticks_ = 50;
    // ENGAGE ONLY FROM A STANDING BODY (the first flat run, 2026-11-08: engaged at tick 1 the sequencer froze the gait's
    // lying start pose, legs straight out, and the swing target was out of reach for the whole run): a minimum tick,
    // three toes in contact, and the belly clear of the ground when a belly topic is given.
    std::string belly_topic_ = "reality.proprio.belly_m";
    int64_t resets_seen_ = 0;
    // THE HEADING HOLD and THE SEQUENCER'S ODOMETRY (operator, 2026-11-08).  The crawl drifted into walls because it had no
    // heading: two of four traces yawed 0.1-0.15 rad over 0.45 m and drifted 7-8 cm sideways with it.  During SHIFT the
    // planted toes are rotated about the body's vertical by heading_gain x the yaw error (the gyro-integrated heading from
    // the body's odometry against its value at engage), bounded by yaw_rate_max, so the body turns back.  And the terrain
    // memory's frame rode a STRIDE odometry the crawl does not drive: the sequencer knows the body's motion exactly (it
    // commands it over the planted toes), so it integrates its own pose -- forward and lateral in the dead-reckoning
    // frame (x forward at the anchor, y along the body's +X), yaw passed through from the body's -- and publishes it on
    // odom_out_topic, passing the body's odometry through while disengaged so the frame is continuous.
    std::string odom_in_topic_ = "reality.proprio.odom"; std::string odom_out_topic_ = "reality.proprio.odom_seq";
    bool odom_anchored_ = false;
    float engage_level_ = 0.0f;
    int64_t disengaged_tilt_ = 0;
    double fk_err_ema_ = 0.0;
    int64_t ticks_engaged_ = 0;
    Bus* bus_ = nullptr;
    std::vector<uint64_t> sub_ids_;

    // the shared state and the components (declared in construction order: the components hold references to the first five)
    stepper::StepperInputs in_; stepper::LoadBook book_; stepper::SupportFloor floor_; body::LegSolver kin_;
    uint32_t rng_ = 12345u;   // the stepper's one random stream (the hung toe's noise and the re-acquire's jitter draw from it in turn)
    stepper::StepExecutor executor_;
    // the step-outcome stream (Phase 2): "" = not published (the default); the diag carries the latest record either way
    std::string outcome_topic_; int64_t outcome_published_ = 0;
    stepper::ScaffoldPlanner planner_;
    // THE LANDING OFFSET (learned stepper plan Phase 4, 2026-11-16; lo_gain 0 and lo_learn 0 = off, byte-identical): the first
    // learned authority over the step, sitting between the planner's intent and the executor -- see stepper/LandingOffset.hpp
    stepper::LandingOffset offset_;
    // THE AFTER-TOUCH RECORD (learned stepper plan 9.5, 2026-11-17; at_learn 0 = off, byte-identical): each step's record carried
    // past first contact -- does the landed toe bear, does the same leg lift next, the tilt -- and a model of it in shadow; it
    // reads the components and moves nothing -- see stepper/AfterTouch.hpp
    stepper::AfterTouch after_;
};

} // namespace ogma
