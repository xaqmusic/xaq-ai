#pragma once
// The forward 8x8 time-of-flight field's geometry, shared by its readers (Tof8Range, TerrainMemory; 2026-11-07):
// the 64 beams (row 0 the top, column 0 the left as the robot sees it; x right, y up, z forward in the sensor frame),
// the mount's downward pitch, and the floor as a plane fitted to the bottom rows' returns -- so no IMU enters and the
// robot's reading is the sim's.  Header-only, no state.
#include <array>
#include <cmath>

namespace ogma::body::tof8 {

constexpr int kRows = 8, kCols = 8, kZones = 64;
constexpr double kFovDeg = 45.0;

struct Vec3 { double x, y, z; };
struct Plane { double a = 0, b = 0, c = 0; bool ok = false; int n = 0;   // y = a + b z + c x
               double y_at(double x, double z) const { return a + b * z + c * x; } };

inline Vec3 beam(int zone) {
    const int r = zone / kCols, c = zone % kCols;
    const double pitch = kFovDeg / kCols, d2r = 3.14159265358979323846 / 180.0;
    const double az = (double(c) - 3.5) * pitch * d2r;   // column 0 negative (the left)
    const double el = (3.5 - double(r)) * pitch * d2r;   // row 0 up
    return {std::cos(el) * std::sin(az), std::sin(el), std::cos(el) * std::cos(az)};
}
inline Vec3 beam_pitched(int zone, double pitch_deg) {
    const Vec3 b = beam(zone);
    const double p = pitch_deg * 3.14159265358979323846 / 180.0;
    return {b.x, b.y * std::cos(p) - b.z * std::sin(p), b.y * std::sin(p) + b.z * std::cos(p)};
}
// The returns as points in the frame the beams are given in (the sensor at the origin): r <= 0 or beyond max = invalid.
inline void points(const float* ranges, double pitch_deg, double max_range_m, std::array<Vec3, kZones>& P, std::array<bool, kZones>& ok) {
    for (int z = 0; z < kZones; ++z) {
        const double r = ranges[z];
        ok[size_t(z)] = r > 0.0 && r <= max_range_m;
        if (!ok[size_t(z)]) { P[size_t(z)] = {0, 0, 0}; continue; }
        const Vec3 d = beam_pitched(z, pitch_deg);
        P[size_t(z)] = {r * d.x, r * d.y, r * d.z};
    }
}
// Least squares y = a + b z + c x over the flagged points; n < 4 or a singular system = not ok.
inline Plane fit_plane(const std::array<Vec3, kZones>& P, const std::array<bool, kZones>& use) {
    Plane pl; double S = 0, Sx = 0, Sz = 0, Sxx = 0, Szz = 0, Sxz = 0, Sy = 0, Sxy = 0, Szy = 0; int n = 0;
    for (int z = 0; z < kZones; ++z) {
        if (!use[size_t(z)]) continue;
        const double x = P[size_t(z)].x, y = P[size_t(z)].y, w = P[size_t(z)].z;
        S += 1; Sx += x; Sz += w; Sxx += x * x; Szz += w * w; Sxz += x * w; Sy += y; Sxy += x * y; Szy += w * y; ++n;
    }
    pl.n = n; if (n < 4) return pl;
    const double M[3][3] = {{S, Sz, Sx}, {Sz, Szz, Sxz}, {Sx, Sxz, Sxx}}; const double R[3] = {Sy, Szy, Sxy};
    auto det3 = [](const double m[3][3]) { return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]); };
    const double det = det3(M); if (std::fabs(det) < 1e-12) return pl;
    auto solve = [&](int col) { double N[3][3]; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) N[i][j] = (j == col) ? R[i] : M[i][j]; return det3(N) / det; };
    pl.a = solve(0); pl.b = solve(1); pl.c = solve(2); pl.ok = true; return pl;
}
// The floor: fitted to the bottom `fit_rows` rows, then once more without the points further than `outlier_m` from
// the first fit; believable only when its slopes are gentle and it lies below the sensor.
inline Plane fit_floor(const std::array<Vec3, kZones>& P, const std::array<bool, kZones>& ok, int fit_rows, double outlier_m, double slope_max) {
    std::array<bool, kZones> use{};
    for (int z = 0; z < kZones; ++z) use[size_t(z)] = ok[size_t(z)] && (z / kCols) >= kRows - fit_rows;
    Plane pl = fit_plane(P, use);
    if (pl.ok) {
        for (int z = 0; z < kZones; ++z) if (use[size_t(z)] && std::fabs(P[size_t(z)].y - pl.y_at(P[size_t(z)].x, P[size_t(z)].z)) > outlier_m) use[size_t(z)] = false;
        const Plane p2 = fit_plane(P, use); if (p2.ok) pl = p2;
    }
    pl.ok = pl.ok && std::fabs(pl.b) <= slope_max && std::fabs(pl.c) <= slope_max && pl.a < 0.0;
    return pl;
}

} // namespace ogma::body::tof8
