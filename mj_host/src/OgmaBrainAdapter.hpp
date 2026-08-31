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
    };

    OgmaBrainAdapter(const DuckBody& body, Config config);
    ~OgmaBrainAdapter() override;

    std::array<double, kNumPolicyJoints> act(const DuckBody& body) override;
    void on_reset() override;
    void set_learning(bool on) override;
    const char* name() const override { return "ogma"; }

    // Diagnostics, for the run summary.
    uint64_t ticks() const { return tick_id_; }
    double   mean_abs_action() const;
    std::vector<std::string> module_ids() const;

    // Per-module diag_lite, as "<id> {json}" lines. The §3.2 question every run
    // has to answer before any behaviour is discussed: DID THE CONSUMER FIRE?
    // A motor TLE pinned at zero means the self-model is predicting perfectly,
    // which on a body that is falling over means it is not being fed.
    std::vector<std::string> diagnostics() const;

private:
    void publish_sensors(const DuckBody& body);

    Config c_;
    std::unique_ptr<ogma::OgmaInstance> instance_;
    uint64_t tick_id_ = 0;
    std::array<std::string, kNumPolicyJoints> action_topics_;
    std::array<double, kNumPolicyJoints> last_u_{};
    std::vector<std::pair<double, double>> range_;    // per policy joint, from the model
    // Learning rates parked while the scaffold drives, keyed "<module id>:<param>".
    std::map<std::string, double> frozen_rates_;
    bool learning_ = true;
    bool announced_freeze_ = false;
    double action_abs_sum_ = 0.0;
    uint64_t action_samples_ = 0;
};

}  // namespace mjhost
