// Tests for ogma/hw/BrainInputs.hpp — the robot side of the P-e brain input contract.
// They read the real calibration files, so a re-calibration or re-export that breaks the
// contract fails here rather than on the robot.
#include "ogma/hw/BrainInputs.hpp"
#include <gtest/gtest.h>

using namespace ogma::hw::brain;
using ogma::body::Vec3f;

namespace {
const std::string kSrc = PI_HOST_SOURCE_DIR;
ServoMapping real_map()  { return ServoMapping::load(kSrc + "/calib/servo_map.json"); }
BodyCalib    real_body() { return BodyCalib::load(kSrc + "/calib/body_measured_fsr.json"); }
}

TEST(BrainServoMap, TheRealMapFillsEveryLegJointSlotExactlyOnce) {
    const ServoMapping m = real_map();
    EXPECT_TRUE(m.complete) << m.why;
}

TEST(BrainServoMap, SimLegNamesAreTheMirroredPhysicalLegs) {
    // servo_map.json: ch 9 is the PHYSICAL FR knee, which is the SIM fl knee.
    const ServoMapping m = real_map();
    ASSERT_TRUE(m.complete) << m.why;
    EXPECT_EQ(m.by_lj[0 * 3 + 2].ch, 9);   // sim fl knee
    EXPECT_EQ(m.by_lj[1 * 3 + 2].ch, 3);   // sim fr knee  = physical FL
    EXPECT_EQ(m.by_lj[3 * 3 + 0].ch, 2);   // sim rr hip1  = physical RL
}

TEST(BrainServoMap, OriginIsZeroAndOneRadianIsTheMeasuredPulseScale) {
    const ServoMapping m = real_map();
    ASSERT_TRUE(m.complete) << m.why;
    std::array<int, 12> us{};
    for (int k = 0; k < 12; ++k) us[size_t(m.by_lj[size_t(k)].ch)] = int(m.by_lj[size_t(k)].origin_us);
    auto a = hinge_angles_from_us(us, m, 545.2);
    for (double v : a) EXPECT_NEAR(v, 0.0, 1e-12);
    // sim fl knee is sign -1 (ch 9): +545 us past origin is about -1 rad.
    us[9] = int(m.by_lj[2].origin_us) + 545;
    a = hinge_angles_from_us(us, m, 545.2);
    EXPECT_NEAR(a[2], -545.0 / 545.2, 1e-12);
}

TEST(BrainBodyCalib, TheExportLoadsAndIsTheFsrLegBody) {
    const BodyCalib b = real_body();
    ASSERT_TRUE(b.ok) << b.why;
    EXPECT_EQ(b.geometry, "measured_fsr");
    EXPECT_NEAR(b.l3, 0.087, 1e-9);
    EXPECT_NEAR(b.knee_rest, -1.6, 1e-9);
    EXPECT_NEAR(b.total_mass_kg, 0.598, 1e-3);
}

TEST(BrainBodyCalib, SharedFkAtZeroReproducesTheSimsOwnZeroPoseFoot) {
    // The cross-check that matters: our ogma::body::fk_leg, fed the EXPORTED anchors, must
    // land on the lower-leg centre the sim's own _fk_leg computed at zero angles.  It tests
    // the anchors, the shared FK and rest_inv together, against a number the sim wrote.
    const BodyCalib b = real_body();
    ASSERT_TRUE(b.ok) << b.why;
    for (int i = 0; i < 4; ++i) {
        const ogma::body::LegPose p = ogma::body::fk_leg(b.anchors[size_t(i)], 0.0, 0.0, 0.0, 0.0);
        const Vec3f got = b.rest_inv(p.lower.origin);
        EXPECT_NEAR(got.x, b.foot_b_zero[size_t(i)].x, 1e-6) << "leg " << i;
        EXPECT_NEAR(got.y, b.foot_b_zero[size_t(i)].y, 1e-6) << "leg " << i;
        EXPECT_NEAR(got.z, b.foot_b_zero[size_t(i)].z, 1e-6) << "leg " << i;
    }
}

TEST(BrainBodyCalib, ToeAtNonZeroPosesMatchesTheSimsFk) {
    // Zero angles apply no rotation, so the test above only proves the anchor positions
    // load.  These poses exercise the joint axes and the rotation chain: our toe_body()
    // against toe positions the sim's own _fk_leg wrote (export_body_calib.gd fk_check).
    const BodyCalib b = real_body();
    ASSERT_TRUE(b.ok) << b.why;
    std::ifstream f(kSrc + "/calib/body_measured_fsr.json");
    const auto j = nlohmann::json::parse(f);
    int checked = 0;
    for (int i = 0; i < 4; ++i)
        for (const auto& c : j["legs"][size_t(i)]["fk_check"]) {
            const auto& t = c["t"];
            const Vec3f got = b.toe_body(i, t[0].get<double>(), t[1].get<double>(), t[2].get<double>());
            EXPECT_NEAR(got.x, c["toe"][0].get<double>(), 1e-6) << "leg " << i;
            EXPECT_NEAR(got.y, c["toe"][1].get<double>(), 1e-6) << "leg " << i;
            EXPECT_NEAR(got.z, c["toe"][2].get<double>(), 1e-6) << "leg " << i;
            ++checked;
        }
    EXPECT_EQ(checked, 8);   // a missing export must fail, not pass vacuously
}

TEST(BrainBodyCalib, ZeroPoseFootHeightAtLevelIsTheClosedForm) {
    // feet_y = foot_b·up − L3/2.  At level (up = +Y) that is −0.04984 − 0.0435 for every
    // leg — the constant behind feet_y_gravity_cmd_imu in P-e.
    const BodyCalib b = real_body();
    ASSERT_TRUE(b.ok) << b.why;
    const auto fy = feet_y_gravity_zero_pose(b, Vec3f(0.0f, 1.0f, 0.0f));
    for (float v : fy) EXPECT_NEAR(v, -0.04984 - 0.0435, 2e-5);
    // Pitching nose-down (up gains +z in the body frame) raises the front legs' reading
    // and lowers the rear's — it is an attitude detector, by construction.
    const auto fp = feet_y_gravity_zero_pose(b, Vec3f(0.0f, 0.9950f, 0.0998f));
    EXPECT_GT(fp[0], fp[2]);
    EXPECT_GT(fp[1], fp[3]);
}

TEST(BrainJoints, NormalisationMatchesTheSimAndIsJointMajor) {
    const BodyCalib b = real_body();
    ASSERT_TRUE(b.ok) << b.why;
    std::array<double, 12> a{};
    a[1 * 3 + 0] = 0.7;          // fr hip1
    a[2 * 3 + 1] = -1.4;         // rl hip2
    a[3 * 3 + 2] = -1.6 + 0.5;   // rr knee
    a[0 * 3 + 2] = -1.6 + 3.0;   // fl knee, saturates
    const auto j = joints_topic(a, b);
    EXPECT_FLOAT_EQ(j[0 * 4 + 1], 0.5f);
    EXPECT_FLOAT_EQ(j[1 * 4 + 2], -1.0f);
    EXPECT_FLOAT_EQ(j[2 * 4 + 3], 0.5f);
    EXPECT_FLOAT_EQ(j[2 * 4 + 0], 1.0f);
}

TEST(BrainFsr, CurveHitsTheTableAndTheGainEvolverLineSitsBelow30g) {
    FsrModel f;
    EXPECT_NEAR(f.grams(1536), 30.0, 1e-9);
    EXPECT_NEAR(f.grams(2362), 175.0, 1e-9);
    EXPECT_EQ(f.grams(40), 0.0);
    // GainEvolver's unloaded test is foot_load >= 0.05 = 29.9 g of a 598 g body.
    EXPECT_GE(f.load(1536), 0.05f);
    EXPECT_LT(f.load(1400), 0.05f);
}

TEST(BrainFsr, ContactIsAboveInAirNoiseAndBelowALightTouch) {
    FsrModel f;
    EXPECT_EQ(f.contact(60), 0.0f);     // in-air ceiling, wiring doc §5.7
    EXPECT_EQ(f.contact(1000), 1.0f);   // a light finger touch, §5.7.6
}

TEST(BrainFsr, ChannelsAreRemappedIntoTheMirroredSimLegOrder) {
    FsrModel f;
    // A0 = physical FL is loaded, the rest in the air.  Physical FL is SIM fr (index 1).
    const auto c = f.contacts_sim({2000, 0, 0, 0});
    EXPECT_EQ(c[0], 0.0f);
    EXPECT_EQ(c[1], 1.0f);
    EXPECT_EQ(c[2], 0.0f);
    EXPECT_EQ(c[3], 0.0f);
    const auto c2 = f.contacts_sim({0, 0, 2000, 0});   // A2 = physical RL = sim rr
    EXPECT_EQ(c2[3], 1.0f);
}
