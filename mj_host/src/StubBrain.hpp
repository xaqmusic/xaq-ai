#pragma once

// =============================================================================
// StubBrain — a brain that fails, so the harness can be tested before A1
// =============================================================================
//
// The recovery harness needs a controller that puts the body on the floor. That is
// a much easier thing to write than one that keeps it up, and it means phase A2 can
// be built and validated before phase A1 exists.
//
// It is deliberately not a controller at all: home pose plus a slow random walk, at
// an amplitude chosen so the robot falls over every few seconds. Nothing here
// learns and nothing here should ever be mistaken for a baseline — a run against
// this stub says the HARNESS works, and says nothing whatever about the substrate.
//
// What it does carry is the seam. `on_reset` and `set_learning` are the calls the
// real brain will need on both edges of a rescue, and having the stub honour them
// means the contract is exercised from the first day rather than designed on the
// day it is needed.

#include <array>
#include <random>

#include "DuckBody.hpp"
#include "Recovery.hpp"

namespace mjhost {

class StubBrain : public BrainLike {
public:
    StubBrain(double amplitude, double drift, uint64_t seed)
        : amplitude_(amplitude), drift_(drift), rng_(seed) {}

    std::array<double, kNumPolicyJoints> act(const DuckBody&) override {
        std::normal_distribution<double> step(0.0, drift_);
        std::array<double, kNumPolicyJoints> out{};
        for (int i = 0; i < kNumPolicyJoints; ++i) {
            // A random WALK rather than white noise: uncorrelated jitter at 50 Hz
            // is filtered out by the actuator and the robot just stands there,
            // which would make the harness look like it worked by never firing.
            walk_[i] = std::max(-1.0, std::min(1.0, walk_[i] + step(rng_)));
            out[i] = kHomePose[i] + amplitude_ * walk_[i];
        }
        return out;
    }

    // Both edges of a rescue. The walk is re-centred so the brain does not resume
    // mid-lunge on a body the scaffold has just carefully stood up.
    void on_reset() override { walk_.fill(0.0); }

    void set_learning(bool on) override { learning_ = on; }
    bool learning() const { return learning_; }

    const char* name() const override { return "stub"; }

private:
    double amplitude_;
    double drift_;
    std::mt19937_64 rng_;
    std::array<double, kNumPolicyJoints> walk_{};
    bool learning_ = true;
};

}  // namespace mjhost
