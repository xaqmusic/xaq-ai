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

// THE KNEE GUARD (2026-11-17).  The logged fault (terrain arena, run t4, the rear-right leg at tick 6400): a planted leg at
// full stretch whose target comes 2 cm toward the hip and 1.4 cm up over 40 ticks.  Without the guard the solver leaves the
// singularity on the far side and runs the knee to its hyperextension stop with the toe "on target"; with it the knee bends.
TEST(LegSolver, FromAStraightLegAskedToShorten_TheFreeSolveHyperextends_AndTheGuardedOneBends) {
    for (int guard = 0; guard < 2; ++guard) {
        LegSolver k = loaded(); k.p.knee_guard = guard;
        ASSERT_TRUE(k.knee_ready());
        auto a = k.angles_of_u(3, {0.12, 0.57, 0.06}); const Vec3f s0 = k.toe_of(3, a);
        EXPECT_NEAR(k.knee_bend(a), 0.0, 0.03);   // the start is the straight leg
        double worst_bend = 1e9, res = 0.0;
        for (int i = 0; i <= 60; ++i) {
            const double f = std::min(1.0, i / 40.0); const Vec3f t(float(s0.x - 0.003 * f), float(s0.y + 0.014 * f), float(s0.z + 0.021 * f));
            k.ik_solve(3, t, a, true); worst_bend = std::min(worst_bend, k.knee_bend(a)); res = dist(k.toe_of(3, a), t);
        }
        if (guard == 0) { EXPECT_LT(k.knee_bend(a), -0.8) << "the fault this guard is for no longer reproduces"; EXPECT_EQ(k.knee_retries, 0); }
        else { EXPECT_GE(worst_bend, k.p.knee_floor_rad - 1e-9); EXPECT_GT(k.knee_bend(a), 0.8); EXPECT_LT(res, k.p.ik_tol_m); EXPECT_GE(k.knee_retries, 1); EXPECT_EQ(k.knee_unsolved, 0); }
    }
}

// a leg that is already past straight when the guard comes on (a live switch) is mirrored onto the bent side and reaches the same toe
TEST(LegSolver, TheGuardTurnsAHyperextendedLegBackOntoItsTarget) {
    for (int leg = 0; leg < 4; ++leg) {
        LegSolver k = loaded();
        auto a = k.angles_of_u(leg, {0.1, 0.8, -0.6}); const Vec3f target = k.toe_of(leg, a);
        ASSERT_LT(k.knee_bend(a), -0.3);
        k.p.knee_guard = 1.0; bool done = false;
        for (int i = 0; i < 10 && !done; ++i) { done = k.ik_solve(leg, target, a, true); EXPECT_GE(k.knee_bend(a), k.p.knee_floor_rad - 1e-9) << "leg " << leg << " solve " << i; }
        EXPECT_LT(dist(k.toe_of(leg, a), target), 2.0 * k.p.ik_tol_m) << "leg " << leg;
    }
}

// over smooth random target paths through the whole workspace (reachable and not): the guarded solve never ends past straight,
// leaves no more reachable targets unreached than the free one, and never moves a joint by a branch flip
TEST(LegSolver, OverRandomTargetPaths_TheGuardNeverEndsPastStraight_AndCostsNoReach) {
    long past[2] = {0, 0}, far[2] = {0, 0}; double jump[2] = {0.0, 0.0};
    for (int guard = 0; guard < 2; ++guard) {
        LegSolver k = loaded(); k.p.knee_guard = guard; uint32_t rng = 7u;
        auto U = [&] { rng = rng * 1664525u + 1013904223u; return double(rng >> 8) / 8388608.0 - 1.0; };
        for (int trial = 0; trial < 12; ++trial) for (int leg = 0; leg < 4; ++leg) {
            auto b = k.angles_of_u(leg, {0.0, 0.3, 0.2}); Vec3f t = k.toe_of(leg, b); double vx = 0, vy = 0, vz = 0;
            for (int i = 0; i < 400; ++i) {
                vx += 0.0002 * U() - 0.02 * vx; vy += 0.0002 * U() - 0.02 * vy; vz += 0.0002 * U() - 0.02 * vz;
                Vec3f n(float(t.x + vx), float(std::clamp(double(t.y) + vy, -0.14, -0.02)), float(t.z + vz)); k.sector_clamp(leg, n); t = n;
                const auto before = b; k.ik_solve(leg, t, b, true);
                if (k.knee_bend(b) < 0.0) ++past[guard];
                for (int j = 0; j < 3; ++j) jump[guard] = std::max(jump[guard], std::fabs(b[size_t(j)] - before[size_t(j)]));
                if (k.ext_slack(leg, t) > 0.004 && dist(k.toe_of(leg, b), t) > 0.008) ++far[guard];
            }
        }
    }
    EXPECT_GT(past[0], 100) << "the free solve no longer hyperextends on these paths: the test exercises nothing";
    EXPECT_EQ(past[1], 0);
    EXPECT_LE(far[1], far[0]);
    EXPECT_LT(jump[1], 0.6);   // 4 damped steps of 0.12 rad, or the guard's nudge and its re-solve: never a flip across the leg
}
