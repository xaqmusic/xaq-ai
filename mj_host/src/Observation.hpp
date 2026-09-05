#pragma once

// =============================================================================
// Observation — the 61-D vector the alpha policies consume
// =============================================================================
//
// This is the highest-risk file in the host, for the reason duck-control/src/obs.rs
// states about its own copy: it is a flat array whose every index must match what
// the policy was trained against, and a wrong offset does not fail loudly.  It
// produces a plausible-looking robot that falls over, and the symptom reads as a
// tuning or timing problem rather than an indexing one.
//
//   index   width  contents
//   0..3        3  gyro, trunk frame, rad/s
//   3..6        3  projected gravity, trunk frame, unit vector
//   6..20      14  joint position minus home pose
//   20..34     14  joint velocity
//   34..48     14  previous action (raw policy output, before scaling)
//   48..61     13  command (below)
//
//   48..51      3  vx, vy, vyaw
//   51..55      4  neck_pitch, head_pitch, head_yaw, head_roll
//   55..57      2  body x, y   — always zero, unbound in training
//   57          1  body z
//   58          1  body roll
//   59          1  body pitch
//   60          1  body yaw    — always zero, unbound in training
//
// Two things about the command block are easy to get wrong and were taken from
// obs.rs rather than guessed:
//
//   1. Body x, y and yaw are hardcoded zero. They are unbound in the training
//      environment, so an all-zero body command is the NOMINAL encoding rather
//      than a placeholder for something better.
//   2. The body block is ordered z, roll, pitch — not z, pitch, roll.
//
// ** THE COMMAND BLOCK IS AN OUTPUT, NEVER AN INPUT. ** In Track A nothing fills
// it but zeros.  In Track B a brain emits it and the body walks.  Nothing in this
// project ever hands it to a brain as a sensory channel: it is a set-point, and a
// set-point is not an observation (port plan, §"The 13-slot command block").

#include <array>

#include "DuckBody.hpp"

namespace mjhost {

inline constexpr int kObsLen     = 61;
inline constexpr int kActionLen  = 14;
inline constexpr int kCommandLen = 13;

static_assert(3 + 3 + 3 * kNumPolicyJoints + kCommandLen == kObsLen,
              "the block widths must sum to the width the ONNX graph declares");
static_assert(kActionLen == kNumPolicyJoints, "one action per policy joint");

// What a client is asking the robot to do, in the form the policy consumes.
// Zero is "stand still, head level, nominal stance".
struct Command {
    std::array<double, 3> twist{0.0, 0.0, 0.0};   // vx, vy, vyaw
    std::array<double, 4> head{0.0, 0.0, 0.0, 0.0};
    double body_z = 0.0, body_roll = 0.0, body_pitch = 0.0;
};

// Assemble the observation. `last_action` is the previous RAW policy output —
// before action scaling — because that is what the policy was trained observing.
std::array<float, kObsLen> build_observation(const DuckBody& body,
                                             const std::array<float, kActionLen>& last_action,
                                             const Command& command);

}  // namespace mjhost
