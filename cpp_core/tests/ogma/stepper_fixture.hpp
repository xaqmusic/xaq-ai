#pragma once
// Shared fixtures for the stepper's tests (PoseSequencer and its Phase 2 components): a published leg model built as the body
// builds it, a flat terrain patch, a support state.  Moved from test_pose_sequencer.cpp 2026-11-15, unchanged.
#include <cmath>
#include <memory>
#include <vector>
#include "ogma/Topics.hpp"
#include "ogma/body/LegKinematics.hpp"
namespace stepper_test {
using ogma::body::Vec3f;
inline std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
inline Vec3f rot(const Vec3f& v, const Vec3f& k, double a) {   // Rodrigues
    const double c = std::cos(a), s = std::sin(a), kd = k.x * v.x + k.y * v.y + k.z * v.z;
    const Vec3f kxv(k.y * v.z - k.z * v.y, k.z * v.x - k.x * v.z, k.x * v.y - k.y * v.x);
    return Vec3f(float(v.x * c + kxv.x * s + k.x * kd * (1 - c)), float(v.y * c + kxv.y * s + k.y * kd * (1 - c)), float(v.z * c + kxv.z * s + k.z * kd * (1 - c)));
}
// the body's _build_leg, in the rest frame (the chassis origin at 0): headings at 45 deg, L1 / L2 / L3, the 7 mm coxa drop,
// the lower leg dropped by 1.396 rad about the lateral, picking the lower of the two
inline std::vector<float> leg_model() {
    const double L1 = 0.03726, L2 = 0.0536, L3 = 0.0755, drop = 0.007, hx = 0.0766 * 0.5, hz = 0.066 * 0.5;
    const double hd[4][2] = {{-0.7071, 0.7071}, {0.7071, 0.7071}, {-0.7071, -0.7071}, {0.7071, -0.7071}};
    std::vector<float> v;
    for (int i = 0; i < 4; ++i) {
        const Vec3f heading(float(hd[i][0]), 0.0f, float(hd[i][1]));
        const Vec3f lateral(-heading.z, 0.0f, heading.x);   // UP x heading = (uy*hz - 0, 0, 0 - uy*hx) = (hz, 0, -hx)... Godot's cross(UP, h) = (1*h.z - 0*0, 0*h.x - 0*h.z, 0*0 - 1*h.x) = (h.z, 0, -h.x)
        const Vec3f lat(heading.z, 0.0f, -heading.x);
        (void)lateral;
        const Vec3f hip1(float((hd[i][0] > 0 ? 1 : -1) * hx), 0.0f, float((hd[i][1] > 0 ? 1 : -1) * hz));
        const double ch = std::sqrt(L1 * L1 - drop * drop);
        Vec3f cdir(float(heading.x * ch), float(-drop), float(heading.z * ch)); const double cn = std::sqrt(double(cdir.x) * cdir.x + double(cdir.y) * cdir.y + double(cdir.z) * cdir.z);
        cdir = Vec3f(float(cdir.x / cn), float(cdir.y / cn), float(cdir.z / cn));
        const Vec3f hip2(float(hip1.x + cdir.x * L1), float(hip1.y + cdir.y * L1), float(hip1.z + cdir.z * L1));
        const Vec3f knee(float(hip2.x + heading.x * L2), hip2.y, float(hip2.z + heading.z * L2));
        const Vec3f down = rot(heading, lat, -1.396), up = rot(heading, lat, 1.396);
        const Vec3f ldir = down.y < up.y ? down : up;
        const Vec3f coxa_o(0.5f * (hip1.x + hip2.x), 0.5f * (hip1.y + hip2.y), 0.5f * (hip1.z + hip2.z));
        const Vec3f upper_o(float(hip2.x + heading.x * L2 * 0.5), hip2.y, float(hip2.z + heading.z * L2 * 0.5));
        const Vec3f lower_o(float(knee.x + ldir.x * L3 * 0.5), float(knee.y + ldir.y * L3 * 0.5), float(knee.z + ldir.z * L3 * 0.5));
        const Vec3f toe_off(float(ldir.x * L3 * 0.5), float(ldir.y * L3 * 0.5), float(ldir.z * L3 * 0.5));
        for (auto const& p : {hip1, hip2, knee, coxa_o, upper_o, lower_o, lat, lat, toe_off}) { v.push_back(p.x); v.push_back(p.y); v.push_back(p.z); }
    }
    for (float x : {0.0f, 1.4f, 0.0f, 1.4f, -1.6f, 3.2f, 0.85f, 1.0f}) v.push_back(x);
    return v;
}
inline std::vector<float> flat_patch(float h) {
    const int N = 16; std::vector<float> v(4 + 2 * N * N, h); v[0] = N; v[1] = 0.03f; v[2] = 0; v[3] = 0;
    for (int k = 0; k < N * N; ++k) v[size_t(4 + N * N + k)] = 40000.0f; return v;
}
inline std::vector<float> support(float m) { std::vector<float> v(26, 0.0f); for (int i = 0; i < 4; ++i) { v[size_t(7 + 5 * i)] = 1; v[size_t(8 + 5 * i)] = 1; v[size_t(9 + 5 * i)] = 0.25f; v[size_t(10 + 5 * i)] = m; } return v; }
} // namespace stepper_test
