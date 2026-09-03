#pragma once
// Where the robot is, estimated from its own legs and IMU.
//
// A port of Pollen's `odometry` crate (itself from the prototype runtime and Rhoban's
// humanoid model_service), kept line-for-line where it matters so the simulated
// estimate is the one the real robot publishes as `odom`.  One point of one sole is
// the contact with the ground; anchor it to the world (flat ground: its Z is 0, its
// X/Y wherever it was when it became the anchor), orient the trunk by the IMU, and
// the trunk's position follows by forward kinematics.  When another sole corner
// drops below the anchor — a step — the anchor moves there, at that corner's current
// world X/Y, so the estimate never jumps.  Heading is the IMU's yaw: no
// magnetometer, so the frame is "wherever the robot was looking at boot".
//
// Egocentric by construction: joint angles and the body's own geometry (FK), and the
// IMU.  Nothing in here reads the simulator's world pose.  The world pose the host
// logs alongside is instrumentation for the reader, and the gate for this port is
// how far the two drift apart over a walk.
#include <array>
#include <optional>
#include <tuple>

namespace mjhost {

struct SitePose {
    std::array<double, 3> pos{};    // in the trunk (IMU) frame
    std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};   // w, x, y, z
};

class Odometry {
public:
    // feet[0] = left_foot site, feet[1] = right_foot site, both in the trunk frame;
    // imu_quat_wxyz = the IMU's orientation (trunk frame → world).
    void update(const std::array<SitePose, 2>& feet, const std::array<double, 4>& imu_quat_wxyz);

    std::array<double, 3> position() const { return position_; }
    double yaw() const { return yaw_; }
    int anchor_foot() const { return anchor_foot_; }
    std::array<double, 2> anchor_xy() const { return anchor_xy_; }

private:
    // Sole half-extents along the foot-site frame X (front/back) and Y (left/right),
    // and the switch rule — the crate's own constants.
    static constexpr double kSoleHalfLen = 0.0270, kSoleHalfWidth = 0.0206;
    static constexpr double kSwitchMargin = -0.010;
    static constexpr int kSwitchConfirmTicks = 2;

    void reproject(const std::array<double, 4>& rot, const std::array<SitePose, 2>& feet);
    std::optional<std::tuple<int, std::array<double, 3>, std::array<double, 2>>>
    lowest_corner(const std::array<double, 4>& rot, const std::array<SitePose, 2>& feet) const;

    int anchor_foot_ = 0;
    std::array<double, 3> anchor_local_{};
    std::array<double, 2> anchor_xy_{};
    std::array<double, 3> position_{};
    double yaw_ = 0.0;
    std::optional<std::tuple<int, std::array<double, 3>, std::array<double, 2>>> pending_;
    int pending_ticks_ = 0;
    bool needs_init_ = true;
};

}  // namespace mjhost
