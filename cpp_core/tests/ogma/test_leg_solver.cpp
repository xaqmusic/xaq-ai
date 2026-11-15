// body::LegSolver -- the stepper's leg model, split out of PoseSequencer (learned stepper plan Phase 2, 2026-11-15).
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/body/LegSolver.hpp"
#include "stepper_fixture.hpp"
namespace {
using ogma::body::LegSolver; using ogma::body::Vec3f;
using namespace stepper_test;
double dist(const Vec3f& a, const Vec3f& b) { return std::sqrt(double(a.x - b.x) * (a.x - b.x) + double(a.y - b.y) * (a.y - b.y) + double(a.z - b.z) * (a.z - b.z)); }
LegSolver loaded() { LegSolver k; const auto v = leg_model(); EXPECT_TRUE(k.set_model(v.data(), v.size())); return k; }
}

TEST(LegSolver, AShortModelIsRefused_AndTheExtensionSlackIsNeutralUntilAModelArrives) {
    LegSolver k; std::vector<float> v(LegSolver::kModelFloats - 1, 0.0f);
    EXPECT_FALSE(k.set_model(v.data(), v.size())); EXPECT_FALSE(k.ready()); EXPECT_FALSE(k.ext_ready());
    EXPECT_EQ(k.ext_slack(0, Vec3f(0.1f, -0.08f, 0.1f)), 1.0);
    const auto m = leg_model(); EXPECT_TRUE(k.set_model(m.data(), m.size())); EXPECT_TRUE(k.ready()); EXPECT_TRUE(k.ext_ready());
}

TEST(LegSolver, TheInverseConvergesOnTheForward_AndTheActionUnitsRoundTrip) {
    LegSolver k = loaded();
    for (int leg = 0; leg < 4; ++leg) {
        const auto good = k.angles_of_u(leg, {0.2, -0.3, 0.4});
        const Vec3f target = k.toe_of(leg, good);
        auto a = k.angles_of_u(leg, {0.0, 0.0, 0.3}); bool done = false;
        for (int i = 0; i < 20 && !done; ++i) done = k.ik_solve(leg, target, a, false);
        EXPECT_TRUE(done) << "leg " << leg; EXPECT_LT(dist(k.toe_of(leg, a), target), k.p.ik_tol_m);
        const auto u = k.u_of_angles(leg, good), back = k.angles_of_u(leg, u);
        for (int j = 0; j < 3; ++j) EXPECT_NEAR(back[size_t(j)], good[size_t(j)], 1e-12);
    }
}

TEST(LegSolver, TheSectorClampKeepsAToeInReach_AndANarrowerHalfAngleClampsMore) {
    LegSolver k = loaded();
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3f hip = k.hip1(leg), h2 = k.hip2(leg);
        const double hx = h2.x - hip.x, hz = h2.z - hip.z, hn = std::hypot(hx, hz), ux = hx / hn, uz = hz / hn;
        Vec3f in(float(hip.x + ux * 0.10), -0.08f, float(hip.z + uz * 0.10)); const Vec3f in0 = in;
        EXPECT_FALSE(k.sector_clamp(leg, in)); EXPECT_EQ(in.x, in0.x); EXPECT_EQ(in.z, in0.z);
        Vec3f far(float(hip.x + ux * 0.30), -0.08f, float(hip.z + uz * 0.30));
        EXPECT_TRUE(k.sector_clamp(leg, far)); EXPECT_NEAR(std::hypot(far.x - hip.x, far.z - hip.z), k.p.reach_max_m, 1e-6);
        // 0.7 rad off the heading: inside the shift's sector (0.9), outside a landing sector of 0.5
        const double c = std::cos(0.7), s = std::sin(0.7), vx = ux * c + uz * s, vz = uz * c - ux * s;
        Vec3f side(float(hip.x + vx * 0.10), -0.08f, float(hip.z + vz * 0.10)), side2 = side;
        EXPECT_FALSE(k.sector_clamp(leg, side)); EXPECT_TRUE(k.sector_clamp(leg, side2, 0.5));
    }
}

TEST(LegSolver, TheExtensionSlackIsZeroAtFullExtension_AndThePlaneFormAgreesWithTheTrueFk) {
    LegSolver k = loaded();
    for (int leg = 0; leg < 4; ++leg) {
        double lo = 1e9;   // over the knee's range at the rest hips, the smallest slack is the straight leg's: zero
        for (int j = 0; j <= 200; ++j) { const double u = -1.0 + 2.0 * j / 200.0; lo = std::min(lo, k.ext_slack_fk(leg, k.angles_of_u(leg, {0.0, 0.0, u}))); }
        EXPECT_NEAR(lo, 0.0, 2e-4) << "leg " << leg;
        const auto t = k.angles_of_u(leg, {0.3, -0.2, 0.5});   // a yawed, bent leg: the plane approximation within a few mm of the FK
        EXPECT_NEAR(k.ext_slack(leg, k.toe_of(leg, t)), k.ext_slack_fk(leg, t), 0.005) << "leg " << leg;
    }
}

TEST(LegSolver, TheRoomReadsEachJointsDistanceToItsStops) {
    LegSolver k = loaded();
    const auto r0 = k.room(0, k.angles_of_u(0, {0.0, 0.0, 0.0}));
    EXPECT_NEAR(r0.hip1, 1.0, 1e-12); EXPECT_NEAR(r0.hip2, 1.0, 1e-12); EXPECT_NEAR(r0.knee_straight, 0.0, 1e-12); EXPECT_NEAR(r0.knee_fold, 1.0, 1e-12);
    const auto r1 = k.room(0, k.angles_of_u(0, {-0.75, 0.5, 0.6}));
    EXPECT_NEAR(r1.hip1, 0.25, 1e-9); EXPECT_NEAR(r1.hip2, 0.5, 1e-9); EXPECT_NEAR(r1.knee_straight, 0.6, 1e-9); EXPECT_NEAR(r1.knee_fold, 0.4, 1e-9);
    EXPECT_NEAR(r1.ext_m, k.ext_slack_fk(0, k.angles_of_u(0, {-0.75, 0.5, 0.6})), 0.0);
}
