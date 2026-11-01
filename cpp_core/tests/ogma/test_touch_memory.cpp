// TouchMemory: the cloud of touches from the toes' commanded FK, carried by odometry.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/TouchMemory.hpp"
namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::TouchMemory m;
    Fix(ogma::ParamMap p = {}) { m.set_id("touch"); m.on_setup(&bus, p); }
    // One tick: contact flags, toe [x,z] x 4 (chassis frame), toe heights, odom.
    void run(uint64_t t, std::vector<float> contact, std::vector<float> xz, std::vector<float> y,
             double ox = 0, double oy = 0, double yaw = 0) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.odom", pv({float(ox), float(oy), float(yaw)}));
        bus.publish("reality.proprio.foot_contact", pv(contact));
        bus.publish("reality.proprio.feet_xz", pv(xz));
        bus.publish("reality.proprio.feet_y_gravity_cmd_imu", pv(y));
        m.tick(t); bus.end_tick();
    }
};
// A stance: four toes 0.1 m out at the corners, all 0.09 m below the body.
const std::vector<float> kXZ = { 0.1f, 0.1f,  -0.1f, 0.1f,  0.1f, -0.1f,  -0.1f, -0.1f };   // [x, z] per leg (x = body +X, z = fwd)
const std::vector<float> kY  = { -0.09f, -0.09f, -0.09f, -0.09f };
// The first stance (three feet down, leg 3 in the air) is three touches of its own: the body
// standing up paints the floor under its feet.  Tests read the leg-3 voxel by its foot mask.
ogma::TouchMemory::BodyVoxel leg3(ogma::TouchMemory const& m) {
    for (auto const& b : m.body_voxels()) if (b.foot_mask & (1 << 3)) return b;
    return ogma::TouchMemory::BodyVoxel{1e9, 1e9, 1e9, 0, 0, 0, 0.0f};
}
}

TEST(TouchMemory, NoInputsNoVoxelsNoCrash) {
    Fix f; f.bus.begin_tick(0); f.m.tick(0); f.bus.end_tick();
    EXPECT_EQ(f.m.voxels(), 0); EXPECT_EQ(f.m.touches(), 0);
    auto d = f.m.diag_snapshot(); EXPECT_EQ(d["count"], 0);
}

TEST(TouchMemory, AFootfallIsAVoxelAtTheToeOnTheSupportPlane) {
    Fix f;
    f.run(0, {1, 1, 1, 0}, kXZ, kY);          // three planted, leg 3 in the air: the plane forms
    EXPECT_EQ(f.m.voxels(), 3) << "the first stance paints three footprints";
    f.run(1, {1, 1, 1, 1}, kXZ, kY);          // leg 3 lands
    ASSERT_EQ(f.m.voxels(), 4); EXPECT_EQ(f.m.touches(), 4);
    auto b = leg3(f.m);
    EXPECT_NEAR(b.fwd, -0.1, 1e-6); EXPECT_NEAR(b.lat, -0.1, 1e-6);
    EXPECT_NEAR(b.h, 0.0, 1e-6) << "a floor touch is at the plane";
    EXPECT_EQ(b.foot_mask, 1 << 3); EXPECT_EQ(b.hits, 1);
    EXPECT_NEAR(f.m.plane_y(), -0.09, 1e-6);
}

TEST(TouchMemory, AHeldContactIsOneTouch) {
    Fix f;
    f.run(0, {1, 1, 1, 0}, kXZ, kY);
    for (uint64_t t = 1; t < 20; ++t) f.run(t, {1, 1, 1, 1}, kXZ, kY);
    EXPECT_EQ(f.m.touches(), 4); EXPECT_EQ(f.m.voxels(), 4);
}

TEST(TouchMemory, AToeLandingAboveThePlaneReadsItsHeightAndRepeatsStackHits) {
    Fix f;
    f.run(0, {1, 1, 1, 0}, kXZ, kY);
    std::vector<float> y_box = kY; y_box[3] = -0.04f;       // leg 3 finds a surface 5 cm above the floor
    for (int k = 0; k < 3; ++k) {
        f.run(uint64_t(2 * k + 1), {1, 1, 1, 1}, kXZ, y_box);
        f.run(uint64_t(2 * k + 2), {1, 1, 1, 0}, kXZ, kY);
    }
    ASSERT_EQ(f.m.voxels(), 4) << "the same spot three times is one voxel";
    auto b = leg3(f.m);
    EXPECT_EQ(b.hits, 3); EXPECT_EQ(f.m.touches(), 6);
    EXPECT_NEAR(b.h, 0.05, 1e-6);
    EXPECT_NEAR(f.m.plane_y(), -0.09, 1e-6) << "the landing foot never moves the plane";
}

TEST(TouchMemory, AStoredTouchStaysInTheWorldWhenTheBodyWalksOn) {
    Fix f;
    f.run(0, {1, 1, 1, 0}, kXZ, kY);
    f.run(1, {1, 1, 1, 1}, kXZ, kY);          // touch at body (fwd -0.1, lat -0.1), odom at origin
    // The body walks 0.3 m forward (odom x): the touch is now 0.4 m behind.
    f.run(2, {1, 1, 1, 1}, kXZ, kY, 0.3, 0.0, 0.0);
    auto b = leg3(f.m);
    EXPECT_NEAR(b.fwd, -0.4, 1e-6); EXPECT_NEAR(b.lat, -0.1, 1e-6);
    // The body yaws +90 deg (toward body +X) at the origin: the odom point (-0.1, -0.1) seen with
    // heading (0, 1) and body +X (-1, 0) is fwd = -0.1, lat = +0.1.
    f.run(3, {1, 1, 1, 1}, kXZ, kY, 0.0, 0.0, M_PI / 2);
    b = leg3(f.m);
    EXPECT_NEAR(b.fwd, -0.1, 1e-6); EXPECT_NEAR(b.lat, 0.1, 1e-6);
}

TEST(TouchMemory, OdomTranslationZeroKeepsTouchesWithTheBody) {
    Fix f({{"odom_translation", ogma::ParamValue{0.0}}});
    f.run(0, {1, 1, 1, 0}, kXZ, kY);
    f.run(1, {1, 1, 1, 1}, kXZ, kY);
    f.run(2, {1, 1, 1, 1}, kXZ, kY, 0.3, 0.0, 0.0);
    EXPECT_NEAR(leg3(f.m).fwd, -0.1, 1e-6) << "rotation-only carriage: the body's travel does not move it";
}

TEST(TouchMemory, DiagListsNearestFirstAndIsBounded) {
    Fix f({{"diag_max", ogma::ParamValue{int64_t{2}}}});
    f.run(0, {1, 1, 1, 0}, kXZ, kY);
    // three footfalls of leg 3 at increasing distance behind the body (odom x grows)
    for (int k = 0; k < 3; ++k) {
        f.run(uint64_t(2 * k + 1), {1, 1, 1, 1}, kXZ, kY, 0.2 * k, 0.0, 0.0);
        f.run(uint64_t(2 * k + 2), {1, 1, 1, 0}, kXZ, kY, 0.2 * k, 0.0, 0.0);
    }
    // Leg 3's second footfall (odom x 0.2) lands where leg 1's first stance footprint is: one voxel, two feet.
    EXPECT_EQ(f.m.voxels(), 5);
    int shared = 0; for (auto const& b : f.m.body_voxels()) if (b.foot_mask == ((1 << 1) | (1 << 3))) ++shared;
    EXPECT_EQ(shared, 1) << "two feet agreeing on one spot is one voxel with both bits";
    auto d = f.m.diag_snapshot();
    ASSERT_EQ(d["voxels"].size(), 2u); EXPECT_EQ(d["count"], 5);
    const double d0 = std::fabs(d["voxels"][0][0].get<double>()), d1 = std::fabs(d["voxels"][1][0].get<double>());
    EXPECT_LE(d0, d1) << "nearest first";
}

TEST(TouchMemory, SnapshotRoundTrips) {
    Fix f;
    f.run(0, {1, 1, 1, 0}, kXZ, kY);
    f.run(1, {1, 1, 1, 1}, kXZ, kY);
    auto s = f.m.snapshot_state();
    Fix g; g.m.restore_state(s);
    EXPECT_EQ(g.m.voxels(), 4); EXPECT_EQ(g.m.touches(), 4); EXPECT_NEAR(g.m.plane_y(), -0.09, 1e-6);
}

TEST(TouchMemory, MaxVoxelsEvictsTheLeastRecentlyTouched) {
    Fix f({{"max_voxels", ogma::ParamValue{int64_t{2}}}});
    f.run(0, {1, 1, 1, 0}, kXZ, kY);
    for (int k = 0; k < 3; ++k) {
        f.run(uint64_t(2 * k + 1), {1, 1, 1, 1}, kXZ, kY, 0.2 * k, 0.0, 0.0);
        f.run(uint64_t(2 * k + 2), {1, 1, 1, 0}, kXZ, kY, 0.2 * k, 0.0, 0.0);
    }
    EXPECT_EQ(f.m.voxels(), 2);
    for (auto const& b : f.m.body_voxels()) EXPECT_GT(b.fwd, -0.55) << "the first (now farthest) touch was evicted";
}
