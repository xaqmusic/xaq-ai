#pragma once

// =============================================================================
// DuckBody — the Microduck, as the host sees it
// =============================================================================
//
// The analogue of picrawler_body.gd, in C++ and much smaller.  It owns the MuJoCo
// model and data, resolves every index it needs BY NAME at construction, steps the
// physics, and answers questions about the body's state.
//
// It holds no Bus and no OgmaInstance.  From S2 onward the host publishes from
// these accessors; the body itself never learns anything and never decides anything.
//
// Two rules this file exists to enforce:
//
//   * NOTHING IS INDEXED BY A TRANSCRIBED CONSTANT.  Every joint, actuator, sensor
//     and body is looked up by its name once, in the constructor, and a lookup that
//     fails is a construction failure with the name in the message.  The picrawler's
//     leg-naming mirror is what happens when this rule is relaxed: a wrong index
//     produces a robot that moves plausibly and incorrectly.
//   * THE SUBSTEP COUNT IS DERIVED, NOT ASSUMED.  The brain tick is a whole number
//     of physics steps or the body refuses to run (gate G3).

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace mjhost {

// The 14 joints every alpha policy drives, in ctrl order — `JOINT_NAMES` from
// duck-ipc-proto with the mouth (index 9) removed.  Both the observation and the
// action are in this order.
inline constexpr int kNumPolicyJoints = 14;
extern const char* const kPolicyJoints[kNumPolicyJoints];

// HOME_FRAME / DEFAULT_POSITION, radians, in policy-joint order.  Must match the
// training env exactly: the policy observes joint position RELATIVE to this, so a
// discrepancy is a constant offset on fourteen observation slots.
extern const std::array<double, kNumPolicyJoints> kHomePose;

// The rate the brain runs at, and the rate `robotd`'s control loop runs at.
inline constexpr double kBrainHz = 50.0;

class DuckBody {
public:
    // Throws std::runtime_error with MuJoCo's own message on a load failure, and
    // with the offending name on a lookup failure.
    explicit DuckBody(const std::string& scene_path);
    ~DuckBody();

    DuckBody(const DuckBody&) = delete;
    DuckBody& operator=(const DuckBody&) = delete;

    // ---------------------------------------------------------------------
    // Lifecycle
    // ---------------------------------------------------------------------

    // Reset to a named keyframe. `joint_noise` perturbs every joint by a normal
    // deviate of that width (radians), leaving the free joint alone — the settle
    // test wants a robot in a slightly wrong pose, not one dropped from a height.
    void reset(const std::string& keyframe, double joint_noise = 0.0, uint64_t seed = 0);

    // Advance one brain tick: `substeps()` physics steps holding `ctrl`.
    void step(const std::array<double, kNumPolicyJoints>& ctrl);

    int    substeps() const { return substeps_; }
    double timestep() const { return m_->opt.timestep; }
    double time()     const { return d_->time; }

    // ---------------------------------------------------------------------
    // State — everything egocentric, and everything by name
    // ---------------------------------------------------------------------

    std::array<double, kNumPolicyJoints> joint_positions()  const;
    std::array<double, kNumPolicyJoints> joint_velocities() const;

    // Projected gravity in the IMU site frame, unit vector. Upright is about
    // [0, 0, -1]. This is the legal replacement for an absolute height or attitude:
    // it is what the real robot's IMU board reports and what `robot.state` carries.
    std::array<double, 3> gravity() const;

    // Trunk-frame angular velocity, rad/s, from the model's own gyro sensor.
    std::array<double, 3> gyro() const;

    // Tilt of the trunk's up-axis away from world up, degrees.
    //
    // A DIAGNOSTIC, not an observation: it reads the world frame, so no brain may
    // ever subscribe to it. It exists because "did the robot stay up" cannot be
    // answered from height alone — a settle test that records only z reports a
    // fallen robot as resting comfortably.
    double tilt_deg() const;

    // Trunk position in the world frame. Same rule: instrumentation only.
    std::array<double, 3> trunk_position() const;

    const mjModel* model() const { return m_; }
    const mjData*  data()  const { return d_; }

private:
    mjModel* m_ = nullptr;
    mjData*  d_ = nullptr;
    int      substeps_ = 0;

    // Resolved once, in the constructor, by name.
    std::array<int, kNumPolicyJoints> qpos_adr_{};   // into d_->qpos
    std::array<int, kNumPolicyJoints> qvel_adr_{};   // into d_->qvel
    std::array<int, kNumPolicyJoints> actuator_{};   // into d_->ctrl
    int trunk_body_  = -1;
    int quat_adr_    = -1;   // into d_->sensordata, framequat on the imu site
    int gyro_adr_    = -1;   // into d_->sensordata
};

}  // namespace mjhost
