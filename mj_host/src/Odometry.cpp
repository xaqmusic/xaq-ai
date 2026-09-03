#include "Odometry.hpp"

#include <cmath>

namespace mjhost {
namespace {

// q ⊗ v ⊗ q⁻¹, the crate's expanded form.
std::array<double, 3> rotate(const std::array<double, 4>& q, const std::array<double, 3>& v) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double tx = 2.0 * (y * v[2] - z * v[1]);
    const double ty = 2.0 * (z * v[0] - x * v[2]);
    const double tz = 2.0 * (x * v[1] - y * v[0]);
    return {v[0] + w * tx + y * tz - z * ty,
            v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx};
}

std::array<double, 3> transform_point(const SitePose& p, const std::array<double, 3>& v) {
    const auto r = rotate(p.quat, v);
    return {p.pos[0] + r[0], p.pos[1] + r[1], p.pos[2] + r[2]};
}

double quat_yaw(const std::array<double, 4>& q) {
    const double siny = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    const double cosy = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
    return std::atan2(siny, cosy);
}

std::array<double, 4> normalized(std::array<double, 4> q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0.0) for (auto& c : q) c /= n;
    return q;
}

}  // namespace

void Odometry::update(const std::array<SitePose, 2>& feet, const std::array<double, 4>& imu_quat_wxyz) {
    const auto rot = normalized(imu_quat_wxyz);
    if (needs_init_) {
        const auto anchor_world = rotate(rot, feet[anchor_foot_].pos);
        anchor_xy_ = {anchor_world[0], anchor_world[1]};
        needs_init_ = false;
    }
    reproject(rot, feet);
    const auto cand = lowest_corner(rot, feet);
    if (!cand) {
        pending_.reset();
        pending_ticks_ = 0;
    } else {
        if (pending_ && std::get<0>(*pending_) == std::get<0>(*cand)) {
            ++pending_ticks_;
        } else {
            pending_ = cand;
            pending_ticks_ = 1;
        }
        if (pending_ticks_ >= kSwitchConfirmTicks) {
            const auto [foot, local, world_xy] = *pending_;
            pending_.reset();
            anchor_foot_ = foot;
            anchor_local_ = local;
            anchor_xy_ = world_xy;
            reproject(rot, feet);
            pending_ticks_ = 0;
        }
    }
    yaw_ = quat_yaw(rot);
}

void Odometry::reproject(const std::array<double, 4>& rot, const std::array<SitePose, 2>& feet) {
    const auto contact_in_trunk = transform_point(feet[anchor_foot_], anchor_local_);
    const auto contact = rotate(rot, contact_in_trunk);
    position_ = {anchor_xy_[0] - contact[0], anchor_xy_[1] - contact[1], -contact[2]};
}

std::optional<std::tuple<int, std::array<double, 3>, std::array<double, 2>>>
Odometry::lowest_corner(const std::array<double, 4>& rot, const std::array<SitePose, 2>& feet) const {
    static const std::array<std::array<double, 3>, 4> kCorners = {{
        {kSoleHalfLen, kSoleHalfWidth, 0.0}, {kSoleHalfLen, -kSoleHalfWidth, 0.0},
        {-kSoleHalfLen, kSoleHalfWidth, 0.0}, {-kSoleHalfLen, -kSoleHalfWidth, 0.0}}};
    double lowest = -kSwitchMargin;
    std::optional<std::tuple<int, std::array<double, 3>, std::array<double, 2>>> best;
    for (int foot = 0; foot < 2; ++foot) {
        for (const auto& corner : kCorners) {
            const auto in_world = rotate(rot, transform_point(feet[foot], corner));
            const std::array<double, 3> world = {position_[0] + in_world[0], position_[1] + in_world[1],
                                                 position_[2] + in_world[2]};
            if (world[2] < lowest) {
                lowest = world[2];
                best = std::make_tuple(foot, corner, std::array<double, 2>{world[0], world[1]});
            }
        }
    }
    return best;
}

}  // namespace mjhost
