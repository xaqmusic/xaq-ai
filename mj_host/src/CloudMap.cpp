#include "CloudMap.hpp"

#include <algorithm>
#include <cmath>

namespace mjhost {
namespace {
constexpr double kPi = 3.14159265358979323846;
}

void CloudMap::open(double anchor_yaw, uint64_t tick) {
    vox_.clear();
    anchor_yaw_ = anchor_yaw;
    opened_tick_ = tick;
    points_ = 0;
    break_vox_ = 0;
    open_ = true;
}

void CloudMap::close() {
    open_ = false;
    vox_.clear();
    points_ = 0;
    break_vox_ = 0;
}

void CloudMap::add(const std::array<TofZone, Tof::kZones>& zones, double yaw, double trunk_z,
                   uint64_t tick) {
    if (!open_ || int(vox_.size()) >= p_.max_voxels) return;
    // The de-rotation: turn this cast back onto the heading the cloud was anchored on.
    double d = yaw - anchor_yaw_;
    while (d > kPi) d -= 2.0 * kPi;
    while (d < -kPi) d += 2.0 * kPi;
    const double c = std::cos(-d), s = std::sin(-d);
    for (const auto& z : zones) {
        if (z.cls != TofZone::Hit && z.cls != TofZone::Floor) continue;
        const double px = z.point_level[0], py = z.point_level[1];
        const double x = c * px - s * py;
        const double y = s * px + c * py;
        const double hz = z.point_level[2] + trunk_z;          // height above the floor
        if (std::hypot(x, y) > p_.max_range) continue;
        ++points_;
        const int ix = int(std::floor(x / p_.voxel_m));
        const int iy = int(std::floor(y / p_.voxel_m));
        const int iz = int(std::floor(hz / p_.voxel_m));
        auto& v = vox_[key(ix, iy, iz)];
        if (v.hits == 0) {
            v.first = tick;
            if (hz >= p_.break_lo && hz < p_.break_hi) ++break_vox_;
        }
        ++v.hits;
        v.last = tick;
    }
}

double CloudMap::new_fraction(uint64_t tick, int window_ticks) const {
    if (!open_ || window_ticks <= 0) return 0.0;
    const uint64_t lo = tick > uint64_t(window_ticks) ? tick - uint64_t(window_ticks) : 0;
    int touched = 0, fresh = 0;
    for (const auto& [k, v] : vox_) {
        (void)k;
        if (v.last < lo) continue;
        ++touched;
        if (v.first >= lo) ++fresh;
    }
    return touched ? double(fresh) / double(touched) : 0.0;
}

std::array<float, CloudMap::kProfile> CloudMap::break_profile() const {
    std::array<float, kProfile> out{};
    if (!open_ || vox_.empty()) return out;
    // per sector: nearest break range, that break's height, its vertical extent, its mass
    std::array<double, kSectors> near{}, nh{}, zlo{}, zhi{};
    std::array<int, kSectors> mass{};
    near.fill(p_.max_range);
    zlo.fill(1e9);
    zhi.fill(-1e9);
    int total_break = 0, sectors_hit = 0;
    double hsum = 0.0;
    const double span = 2.0 * p_.half_fov;                     // degrees the sectors divide
    for (const auto& [k, v] : vox_) {
        (void)v;
        // unpack the key back to voxel indices, then to metres at the voxel centre
        const int64_t kz = (k & ((int64_t(1) << 21) - 1));
        const int64_t ky = ((k >> 21) & ((int64_t(1) << 21) - 1));
        const int64_t kx = ((k >> 42) & ((int64_t(1) << 21) - 1));
        const double x = (double(kx - 524288) + 0.5) * p_.voxel_m;
        const double y = (double(ky - 524288) + 0.5) * p_.voxel_m;
        const double h = (double(kz - 524288) + 0.5) * p_.voxel_m;
        if (h < p_.break_lo || h >= p_.break_hi) continue;      // the floor, or furniture-tall
        const double r = std::hypot(x, y);
        if (r < 1e-6 || r > p_.max_range) continue;
        const double az = std::atan2(y, x) * 180.0 / kPi;       // + = left
        if (std::fabs(az) > p_.half_fov) continue;
        const int sec = std::clamp(int((az + p_.half_fov) / span * kSectors), 0, kSectors - 1);
        ++total_break;
        ++mass[size_t(sec)];
        hsum += h;
        zlo[size_t(sec)] = std::min(zlo[size_t(sec)], h);
        zhi[size_t(sec)] = std::max(zhi[size_t(sec)], h);
        if (r < near[size_t(sec)]) { near[size_t(sec)] = r; nh[size_t(sec)] = h; }
    }
    for (int s = 0; s < kSectors; ++s) {
        const bool hit = mass[size_t(s)] > 0;
        if (hit) ++sectors_hit;
        out[size_t(s)]                = float(hit ? near[size_t(s)] / p_.max_range : 1.0);
        out[size_t(kSectors + s)]     = float(hit ? std::clamp(nh[size_t(s)] / p_.break_hi, 0.0, 1.0) : 0.0);
        out[size_t(2 * kSectors + s)] = float(hit ? std::clamp((zhi[size_t(s)] - zlo[size_t(s)]) / p_.break_hi, 0.0, 1.0) : 0.0);
        out[size_t(3 * kSectors + s)] = float(std::clamp(double(mass[size_t(s)]) / 60.0, 0.0, 1.0));
    }
    out[size_t(4 * kSectors + 0)] = float(std::clamp(double(total_break) / 400.0, 0.0, 1.0));
    out[size_t(4 * kSectors + 1)] = float(double(sectors_hit) / double(kSectors));
    out[size_t(4 * kSectors + 2)] = float(total_break ? std::clamp(hsum / total_break / p_.break_hi, 0.0, 1.0) : 0.0);
    out[size_t(4 * kSectors + 3)] = float(std::clamp(double(vox_.size()) / 4000.0, 0.0, 1.0));
    return out;
}

}  // namespace mjhost
