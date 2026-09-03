#pragma once

// =============================================================================
// OgmaBrainAdapter — an OgmaInstance driving the duck
// =============================================================================
//
// Phase A1.  The seam is `BrainLike`, the same one StubBrain filled while the
// recovery harness was built, so the brain drops into a rig that already works.
//
// Per tick, in this order, which is the order the Godot host uses and the one the
// Scheduler's freshness comparison assumes:
//
//     publish sensors  ->  instance.tick()  ->  read action.*  ->  step physics
//
// THE SENSOR CONDITIONING IS THE PART THAT DECIDES WHETHER THIS WORKS.
// Doctrine §0 rule 2: a signal handed to a discretizer on the wrong scale gets
// collapsed and the diagnosis looks like a modelling failure. Joint positions go
// out CENTRED ON THE HOME POSE and SCALED BY THE COMMAND AMPLITUDE, so:
//
//   * the resting pose is the origin rather than an arbitrary offset,
//   * sensor and action live in the same units, which is what makes the loop
//     Jacobian L = A·G·C well-conditioned,
//   * and the [-1, 1] the JointSensorimotorBridge clamps to means something.
//
// FREEZING IS DONE THROUGH PARAMS, NOT BY SKIPPING TICKS.  While the scaffold
// drives, the brain must still SEE — the fall is the prediction error and it is
// the most informative thing that happens all run — but it must not LEARN, or it
// is fitting the scaffold's policy (§5.6). MotorEPM's four learning rates are
// HotMutable, so `set_learning(false)` zeroes them and restores them after. No
// module is edited, which §Coordination requires of this branch.

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "DuckBody.hpp"
#include "Recovery.hpp"

namespace ogma {
class OgmaInstance;
}

namespace mjhost {

class OgmaBrainAdapter : public BrainLike {
public:
    struct Config {
        std::string graph_path;
        uint64_t seed = 0;
        // Joint command amplitude, radians. The brain's [-1, 1] output spans
        // home ± this, and it is also the scale the observation is divided by.
        double amplitude = 0.35;
        // The brain's command/observation ORIGIN.  Empty = kHomePose (the STAND
        // keyframe).  The harness fills this with the SCAFFOLD'S OWN measured
        // equilibrium (2026-08-31): the keyframe is up to 0.10 rad away from where
        // alpha_stand actually balances (neck −0.10, head +0.09, r_hip_pitch
        // −0.06), so u = 0 at the keyframe is a pose the body topples from in
        // ~0.1 s, and every handback began with a step-change lurch toward it.
        // A scaffold-derived origin is a calibration, named as such — the same
        // category as reading joint ranges from the model instead of transcribing
        // them.
        std::vector<double> home;
        // Calibrated head-CoM offset at the scaffold's stand (trunk frame x/y, m):
        // the sense channel publishes deviations FROM this, so the standing pose
        // is the origin of the head-CoM observation too.
        std::vector<double> head_com0;
    };

    OgmaBrainAdapter(const DuckBody& body, Config config);

    // The origin actually in use (calibrated or keyframe), for the record line.
    const std::array<double, kNumPolicyJoints>& home() const { return home_; }
    ~OgmaBrainAdapter() override;

    std::array<double, kNumPolicyJoints> act(const DuckBody& body) override;
    void on_reset() override;
    void set_learning(bool on) override;
    void set_regime_learning(bool on);   // the learnable-regime gate (see note above)

private:
    void apply_freeze_state();

public:
    const char* name() const override { return "ogma"; }

    // Diagnostics, for the run summary.
    uint64_t ticks() const { return tick_id_; }
    double   mean_abs_action() const;
    double last_cmd_mag() const override { return last_cmd_mag_; }
    int    regime_id()  const override { return regime_id_; }
    double regime_tle() const override { return regime_tle_; }
    std::vector<std::string> module_ids() const;
    // Each MotorEPM's earned-consolidation state c ∈ [0,1], in graph order. The
    // per-tick read-back the (d) push test is judged on (see run_with_brain).
    std::vector<double> consolidation() const;
    // Each MotorEPM's attitude-prior instant error (the gate subset's |e| mean), in graph
    // order: the brain's own saturation signal for the step hand-off.
    std::vector<double> attitude_error() const;

    // robotd's deployed joint-target low-pass (head 0.5, legs 0.7). Off = raw
    // commands (legacy, byte-identical).
    void set_servo_filter(bool on) { servo_filter_ = on; }

    // Brain checkpointing (2026-09-01, the tall-standing snapshot): the full
    // OgmaInstance state — every module's working state incl. earned
    // consolidation, plus the bus's last-value cache — as one JSON blob.
    nlohmann::json brain_state() const;
    void           restore_brain_state(nlohmann::json const& s);

    // ★ The risk this experiment must be able to see.
    //
    // The ledger records that on the picrawler "inverted-on-flat is a LOW-SURPRISE
    // attractor -- motor_tle falls 0.24 -> 0.08". Once a forward model learns
    // falling well, lying down becomes PREDICTABLE, and a rule that descends a
    // sensitivity metric may come to prefer it. Adding gravity to the loop could
    // therefore produce a very good faller.
    //
    // So TLE is bucketed by posture from the first run rather than after somebody
    // wonders. If tle_down < tle_up, the duck is learning to fall.
    double tle_upright() const;
    double tle_down() const;
    void   sample_tle(bool upright);

    // Per-module diag_lite, as "<id> {json}" lines. The §3.2 question every run
    // has to answer before any behaviour is discussed: DID THE CONSUMER FIRE?
    // A motor TLE pinned at zero means the self-model is predicting perfectly,
    // which on a body that is falling over means it is not being fed.
    std::vector<std::string> diagnostics() const;

private:
    void publish_sensors(const DuckBody& body);

    Config c_;
    std::array<double, kNumPolicyJoints> home_{};
    std::unique_ptr<ogma::OgmaInstance> instance_;
    bool servo_filter_ = false;                 // robotd's target low-pass; off = legacy raw
    bool filt_init_ = false;
    std::array<double, kNumPolicyJoints> filt_{};
    uint64_t tick_id_ = 0;
    std::array<std::string, kNumPolicyJoints> action_topics_;
    std::array<double, kNumPolicyJoints> last_u_{};
    std::vector<std::pair<double, double>> range_;    // per policy joint, from the model
    // Learning rates parked while the scaffold drives, keyed "<module id>:<param>".
    std::map<std::string, double> frozen_rates_;
    bool learning_ = true;        // the scaffold axis (BrainLike::set_learning)
    bool regime_ok_ = true;       // the regime axis
    bool frozen_now_ = false;     // what is actually applied (either axis)
    bool announced_freeze_ = false;
    double tle_up_sum_ = 0.0, tle_down_sum_ = 0.0;
    uint64_t tle_up_n_ = 0, tle_down_n_ = 0;
    double action_abs_sum_ = 0.0;
    double last_cmd_mag_ = -1.0;
    int    regime_id_  = -1;
    double regime_tle_ = -1.0;
    uint64_t action_samples_ = 0;
};

}  // namespace mjhost
