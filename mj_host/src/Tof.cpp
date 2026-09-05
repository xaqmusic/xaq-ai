#include "Tof.hpp"

#include <algorithm>
#include <cmath>

#include <mujoco/mujoco.h>

#include "DuckBody.hpp"

namespace mjhost {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::array<double, 3> rot(const double* R, const std::array<double, 3>& v) {
    return {R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
            R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
}

std::array<double, 3> qrot(const std::array<double, 4>& q, const std::array<double, 3>& v) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double tx = 2.0 * (y * v[2] - z * v[1]), ty = 2.0 * (z * v[0] - x * v[2]), tz = 2.0 * (x * v[1] - y * v[0]);
    return {v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx};
}

// The rotation that levels the trunk: takes the measured gravity to (0, 0, -1).
std::array<double, 4> level_from_gravity(const std::array<double, 3>& g) {
    const double n = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    if (n < 1e-9) return {1.0, 0.0, 0.0, 0.0};
    const std::array<double, 3> a = {g[0] / n, g[1] / n, g[2] / n}, b = {0.0, 0.0, -1.0};
    const double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    std::array<double, 3> c = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    const double s = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    if (s < 1e-9) return d > 0 ? std::array<double, 4>{1.0, 0.0, 0.0, 0.0} : std::array<double, 4>{0.0, 1.0, 0.0, 0.0};
    const double ang = std::atan2(s, d), h = std::sin(ang / 2.0);
    return {std::cos(ang / 2.0), h * c[0] / s, h * c[1] / s, h * c[2] / s};
}

}  // namespace

Tof::Tof() {
    // Zone centres evenly spread over the FOV with a half-zone inset at the edges; row 0
    // is the top, column 0 the left, as the crate lays them out.  The MJCF tof site's
    // frame IS the crate's sensor frame — at the standing keyframe its axes coincide
    // with the world's: +x the optical axis (forward), +y left, +z up — measured with
    // mj_ray, not assumed: a ray along site +x from the site meets a wall placed 0.5 m
    // ahead at 0.41 m, a ray along −z meets the floor at the site's height.
    const double half = (kFovDeg / 2.0 - kFovDeg / kCols / 2.0) * kPi / 180.0;
    const double step = 2.0 * half / (kCols - 1);
    for (int i = 0; i < kZones; ++i) {
        const double el = half - (i / kCols) * step, az = half - (i % kCols) * step;
        const double f = std::cos(el) * std::cos(az), l = std::cos(el) * std::sin(az), u = std::sin(el);
        beams_[i] = {f, l, u};
    }
}

void Tof::sense(const DuckBody& body, double trunk_height_m) {
    const mjModel* m = body.model();
    const mjData* d = body.data();
    std::array<double, 3> spos;
    std::array<double, 9> smat;
    body.site_world("tof", spos, smat);
    std::array<double, 3> sensor_pos;
    std::array<double, 4> sensor_quat;
    body.site_pose_trunk("tof", sensor_pos, sensor_quat);
    const auto g = body.gravity();
    const auto level = level_from_gravity(g);
    const auto sensor_level = qrot(level, sensor_pos);
    const double above_floor = sensor_level[2] + trunk_height_m;
    const double floor_threshold = above_floor * kFloorSafety;

    static const mjtByte kWorldOnly[mjNGROUP] = {1, 0, 0, 0, 0, 0};
    for (int i = 0; i < kZones; ++i) {
        TofZone& z = zones_[i];
        z = TofZone{};
        const auto dir_world = rot(smat.data(), beams_[i]);
        int geomid = -1;
        const mjtNum r = mj_ray(m, d, spos.data(), dir_world.data(), kWorldOnly, 1, -1, &geomid, nullptr);
        if (r < 0.0 || r > kMaxRangeM) continue;             // Empty
        z.range = r;
        const auto dir = qrot(sensor_quat, beams_[i]);        // the beam in the trunk frame
        const auto dir_level = qrot(level, dir);
        const double downward = -dir_level[2];
        z.point = {sensor_pos[0] + r * dir[0], sensor_pos[1] + r * dir[1], sensor_pos[2] + r * dir[2]};
        if (above_floor > 0.0 && downward > 0.0 && r * downward >= floor_threshold) {
            z.cls = TofZone::Floor;
            continue;
        }
        const double horizontal = r * std::sqrt(dir_level[0] * dir_level[0] + dir_level[1] * dir_level[1]);
        if (horizontal < kMinRangeM) {
            z.cls = TofZone::TooClose;
            continue;
        }
        z.cls = TofZone::Hit;
        z.horizontal = horizontal;
    }
}

std::array<double, Tof::kCols> Tof::column_hit() const {
    std::array<double, kCols> out;
    out.fill(kMaxRangeM);
    for (int i = 0; i < kZones; ++i)
        if (zones_[i].cls == TofZone::Hit) out[i % kCols] = std::min(out[i % kCols], zones_[i].horizontal);
    return out;
}

int Tof::too_close() const {
    int n = 0;
    for (const auto& z : zones_) n += (z.cls == TofZone::TooClose);
    return n;
}

std::array<float, 4> Tof::summary() const {
    const auto col = column_hit();
    const auto prox = [&](int a, int b) {
        double r = kMaxRangeM;
        for (int c = a; c <= b; ++c) r = std::min(r, col[c]);
        return float(std::clamp(1.0 - r / 1.0, 0.0, 1.0));
    };
    return {prox(0, 2), prox(3, 4), prox(5, 7), float(too_close()) / float(kZones)};
}

}  // namespace mjhost
