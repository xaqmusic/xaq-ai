// Tests for ogma/hw/LegModel.hpp -- the leg model the stepper's kinematics are built from, packed on the robot from the
// calibration the senses and the actuation already use.  The checks run the stepper's own solver (ogma::body::LegSolver)
// on the packed model against the host's FK and action map, and against the toe positions the sim's FK exported.
#include "ogma/hw/LegModel.hpp"
#include "ogma/body/LegSolver.hpp"
#include <gtest/gtest.h>

using namespace ogma::hw::brain;
using ogma::body::LegSolver;
using ogma::body::Vec3f;

namespace {
const std::string kSrc = PI_HOST_SOURCE_DIR;
const std::string kBody = kSrc + "/calib/body_measured_fsr.json";
LegModel real_model() { return leg_model(BodyCalib::load(kBody), ActionMap::load(kBody)); }
}

TEST(LegModel, TheRealCalibrationPacksTheLayoutTheSolverReads) {
    const BodyCalib b = BodyCalib::load(kBody);
    const ActionMap m = ActionMap::load(kBody);
    const LegModel lm = leg_model(b, m);
    ASSERT_TRUE(lm.ok) << lm.why;
    EXPECT_EQ(int(lm.v.size()), LegSolver::kModelFloats);
    for (int l = 0; l < 4; ++l) {
        const Vec3f h1 = b.rest_inv(b.anchors[size_t(l)].hip1_world);
        EXPECT_FLOAT_EQ(lm.v[size_t(27 * l)], h1.x);
        EXPECT_FLOAT_EQ(lm.v[size_t(27 * l + 2)], h1.z);
        EXPECT_FLOAT_EQ(lm.v[size_t(27 * l + 24)], b.toe_off[size_t(l)].x);
        EXPECT_FLOAT_EQ(lm.v[size_t(27 * l + 26)], b.toe_off[size_t(l)].z);
    }
    EXPECT_FLOAT_EQ(lm.v[109], float(m.hip1_range));
    EXPECT_FLOAT_EQ(lm.v[112], float(m.knee_rest));
    EXPECT_FLOAT_EQ(lm.v[113], float(m.knee_fold));
    EXPECT_FLOAT_EQ(lm.v[114], float(m.knee_hyperext));
    EXPECT_FLOAT_EQ(lm.v[115], 1.0f);
}

TEST(LegModel, TheSteppersSolverOnThisModelIsTheHostsFkAndActionMap) {
    const BodyCalib b = BodyCalib::load(kBody);
    const ActionMap m = ActionMap::load(kBody);
    const LegModel lm = leg_model(b, m);
    ASSERT_TRUE(lm.ok) << lm.why;
    LegSolver s;
    ASSERT_TRUE(s.set_model(lm.v.data(), lm.v.size()));
    ASSERT_TRUE(s.ready());
    int checked = 0;
    for (int l = 0; l < 4; ++l)
        for (double u1 : {-1.0, -0.4, 0.0, 0.7, 1.0})
            for (double u2 : {-1.0, -0.3, 0.0, 0.5, 1.0})
                for (double u3 : {-1.0, -0.5, 0.0, 0.25, 1.0}) {
                    const auto ts = s.angles_of_u(l, {u1, u2, u3});          // the stepper's map
                    const auto th = leg_targets_from_u(l, u1, u2, u3, m);    // the actuation's map
                    for (int j = 0; j < 3; ++j) EXPECT_NEAR(ts[size_t(j)], th[size_t(j)], 1e-6) << "leg " << l << " joint " << j;
                    const Vec3f a = s.toe_of(l, th);                         // the stepper's toe
                    const Vec3f h = b.toe_body(l, th[0], th[1], th[2]);      // the toe behind feet_xz and toe_y
                    EXPECT_NEAR(a.x, h.x, 1e-6); EXPECT_NEAR(a.y, h.y, 1e-6); EXPECT_NEAR(a.z, h.z, 1e-6);
                    ++checked;
                }
    EXPECT_EQ(checked, 500);
}

TEST(LegModel, TheSolversToeMatchesTheSimsOwnFkExport) {
    const LegModel lm = real_model();
    ASSERT_TRUE(lm.ok) << lm.why;
    LegSolver s;
    ASSERT_TRUE(s.set_model(lm.v.data(), lm.v.size()));
    const auto j = nlohmann::json::parse(std::ifstream(kBody));
    int checked = 0;
    for (int i = 0; i < 4; ++i)
        for (const auto& c : j["legs"][size_t(i)]["fk_check"]) {
            const auto& t = c["t"];
            const Vec3f got = s.toe_of(i, {t[0].get<double>(), t[1].get<double>(), t[2].get<double>()});
            EXPECT_NEAR(got.x, c["toe"][0].get<double>(), 1e-6) << "leg " << i;
            EXPECT_NEAR(got.y, c["toe"][1].get<double>(), 1e-6) << "leg " << i;
            EXPECT_NEAR(got.z, c["toe"][2].get<double>(), 1e-6) << "leg " << i;
            ++checked;
        }
    EXPECT_EQ(checked, 8);   // a missing export must fail, not pass vacuously
}

TEST(LegModel, TheSolversIkReachesATargetTheHostsFkConfirms) {
    const BodyCalib b = BodyCalib::load(kBody);
    const LegModel lm = leg_model(b, ActionMap::load(kBody));
    ASSERT_TRUE(lm.ok) << lm.why;
    LegSolver s;
    ASSERT_TRUE(s.set_model(lm.v.data(), lm.v.size()));
    for (int l = 0; l < 4; ++l) {
        std::array<double, 3> t = s.angles_of_u(l, {0.0, 0.0, 0.3});
        Vec3f target = s.toe_of(l, t);
        target.y += 0.015f; target.z += 0.02f;                               // a step's worth: up 15 mm, forward 20 mm
        bool ok = false;
        for (int k = 0; k < 40 && !ok; ++k) ok = s.ik_solve(l, target, t, true);
        EXPECT_TRUE(ok) << "leg " << l;
        const Vec3f h = b.toe_body(l, t[0], t[1], t[2]);
        EXPECT_NEAR(h.x, target.x, 0.005) << "leg " << l;
        EXPECT_NEAR(h.y, target.y, 0.005) << "leg " << l;
        EXPECT_NEAR(h.z, target.z, 0.005) << "leg " << l;
    }
}

TEST(LegModel, AHipSignOrAMissingCalibrationIsRefused) {
    const BodyCalib b = BodyCalib::load(kBody);
    ActionMap m = ActionMap::load(kBody);
    m.splay_out_sign[1] = -1.0;
    const LegModel flipped = leg_model(b, m);
    EXPECT_FALSE(flipped.ok);
    EXPECT_NE(flipped.why.find("splay_out_sign"), std::string::npos);
    const LegModel nobody = leg_model(BodyCalib{}, ActionMap::load(kBody));
    EXPECT_FALSE(nobody.ok);
    ActionMap narrow = ActionMap::load(kBody);
    narrow.knee_widening = false; narrow.knee_symmetric = narrow.hip2_range + 0.2;
    EXPECT_FALSE(leg_model(b, narrow).ok);
}
