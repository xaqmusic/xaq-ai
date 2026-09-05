#include "Observation.hpp"

#include <stdexcept>

namespace mjhost {

std::array<float, kObsLen> build_observation(const DuckBody& body,
                                             const std::array<float, kActionLen>& last_action,
                                             const Command& command) {
    std::array<float, kObsLen> obs{};
    int k = 0;
    const auto put = [&obs, &k](double v) { obs[k++] = float(v); };

    for (double g : body.gyro())    put(g);
    for (double g : body.gravity()) put(g);

    const auto q = body.joint_positions();
    for (int i = 0; i < kNumPolicyJoints; ++i) put(q[i] - kHomePose[i]);

    for (double v : body.joint_velocities()) put(v);

    // Already float and already the right width: the previous action is the
    // policy's own output fed back, not a conversion of anything.
    for (float a : last_action) obs[k++] = a;

    // Reads in the same order as the table in the header, which is the point: this
    // block has no second source of truth, so it should be checkable by eye.
    put(command.twist[0]);
    put(command.twist[1]);
    put(command.twist[2]);
    put(command.head[0]);
    put(command.head[1]);
    put(command.head[2]);
    put(command.head[3]);
    put(0.0);                   // body x — unbound in training
    put(0.0);                   // body y — unbound
    put(command.body_z);
    put(command.body_roll);
    put(command.body_pitch);
    put(0.0);                   // body yaw — unbound

    // Cannot fire: the widths are constants the static_assert in the header pins.
    // Kept because a silent short fill would leave the tail at zero, and a zero in
    // the observation is a joint sitting exactly at its home pose.
    if (k != kObsLen) throw std::logic_error("observation block widths do not sum to kObsLen");
    return obs;
}

}  // namespace mjhost
