// =============================================================================
// test_motion_field.cpp
//   ogma::MotionField -- the always-on motion loop's sensor (the ten-minutes phase, the chase push, 2026-10-02).
//   1. AStaticWallSampledByMovingRaysIsNotMotion -- every ray ends ON the wall, so no return lies where a ray went on.
//   2. AThingStandingWhereTheRaysJustPassedIsMotion -- level rays reach a far wall through empty space; a cube then
//      stands in their path and moves: its returns are evidence, a track is published after two casts, with the cube's
//      position and its velocity's direction.
//   3. NothingIsPublishedWithoutAnOutputTopic -- passive: computed, not published.
// =============================================================================
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <memory>
#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/MotionField.hpp"

namespace {
using Pt = std::array<double, 3>;
constexpr int kZ = ogma::MotionField::kZones;
constexpr int kCast = 5 + 3 * kZ + 3 + 3 * kZ;

struct Rig {
    ogma::InProcessBus bus; ogma::MotionField m; uint64_t t = 0;
    explicit Rig(ogma::ParamMap p = {}) { m.set_id("motion"); m.on_setup(&bus, p); }
    // one sense tick (and three repeats, as the host's token repeats between ToF updates): the body at the origin
    void cast(std::vector<Pt> const& pts, Pt origin = {0.0, 0.0, 0.10}) {
        auto tok = std::make_shared<ogma::ProprioToken>();
        tok->values = Eigen::VectorXf::Constant(kCast, std::numeric_limits<float>::quiet_NaN());
        tok->values[0] = 1.0f; tok->values[1] = 0.0f; tok->values[2] = 0.12f; tok->values[3] = 0.0f; tok->values[4] = 0.0f;
        for (size_t i = 0; i < pts.size() && i < size_t(kZ); ++i)
            for (int k = 0; k < 3; ++k) tok->values[int(5 + 3 * i + k)] = float(pts[i][size_t(k)]);
        for (int k = 0; k < 3; ++k) tok->values[5 + 3 * kZ + k] = float(origin[size_t(k)]);
        for (int rep = 0; rep < 4; ++rep) {
            // the host's repeats carry this tick's trunk height in z: the cast is the same cast
            auto rt = std::make_shared<ogma::ProprioToken>(*tok);
            for (size_t i = 0; i < pts.size() && i < size_t(kZ); ++i) rt->values[int(5 + 3 * i + 2)] += 0.001f * float(rep);
            bus.begin_tick(t); bus.publish("reality.proprio.tof_points", rt); m.tick(t);
            if (rep == 0) EXPECT_TRUE(m.cast_now()); else EXPECT_FALSE(m.cast_now()) << "a repeat is not a new cast";
            bus.end_tick(); ++t;
        }
    }
};
// a wall at x = 2.0, level returns at the sensor's height (z 0.10), across y in [-0.3, 0.3], offset by `phase` per cast
std::vector<Pt> wall(double phase) {
    std::vector<Pt> w;
    for (int j = 0; j < 30; ++j) w.push_back({2.0, -0.30 + 0.02 * j + phase, 0.10});
    return w;
}
}  // namespace

TEST(MotionField, AStaticWallSampledByMovingRaysIsNotMotion) {
    Rig r;
    for (int k = 0; k < 40; ++k) r.cast(wall(0.003 * (k % 7)));
    EXPECT_EQ(r.m.evidence_points(), 0);
    EXPECT_TRUE(r.m.tracks().empty());
    EXPECT_EQ(r.m.published(), -1);
}

TEST(MotionField, AThingStandingWhereTheRaysJustPassedIsMotion) {
    ogma::ParamMap p; p["output_topic"] = std::string("percept.motion");
    Rig r(p);
    for (int k = 0; k < 10; ++k) r.cast(wall(0.003 * (k % 7)));     // the rays reach the wall through x = 1
    double y0 = -0.10;
    for (int k = 0; k < 6; ++k) {                                    // a cube at x = 1.0, moving +y 2 cm a cast
        std::vector<Pt> pts;
        for (int j = 0; j < 4; ++j) pts.push_back({1.0, y0 + 0.02 * j, 0.10});        // its face, on the rays' path
        for (auto q : wall(0.0)) if (q[1] / 2.0 < y0 - 0.01 || q[1] / 2.0 > y0 + 0.07) pts.push_back(q);   // the wall beside it
        r.cast(pts);
        y0 += 0.02;
    }
    EXPECT_GE(r.m.evidence_points(), 2) << "the cube's face stands where the rays went on to the wall";
    ASSERT_GE(r.m.published(), 0) << "a track after two casts";
    const auto& tr = r.m.tracks()[size_t(r.m.published())];
    EXPECT_NEAR(tr.x, 1.0, 0.05);
    EXPECT_NEAR(tr.y, y0 - 0.02 + 0.03, 0.06);
    EXPECT_GT(tr.vy, 0.05) << "moving +y";
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("percept.motion"));
    ASSERT_NE(tok, nullptr);
    EXPECT_GT(tok->values[2], 0.4f) << "proximity of a thing 1 m away";
    EXPECT_GT(tok->values[1], 0.9f) << "ahead";
}

TEST(MotionField, NothingIsPublishedWithoutAnOutputTopic) {
    Rig r;
    for (int k = 0; k < 10; ++k) r.cast(wall(0.003 * (k % 7)));
    std::vector<Pt> pts;
    for (int j = 0; j < 4; ++j) pts.push_back({1.0, -0.1 + 0.02 * j, 0.10});
    r.cast(pts);
    for (auto& q : pts) q[1] += 0.01;                               // the next cast: the cube moved a centimetre
    r.cast(pts);
    EXPECT_EQ(r.bus.last_value("percept.motion"), nullptr);
}

// Step 3: with cloud_mover_topic, a tick with no published track of its own passes the cloud's sighting through.
TEST(MotionField, WithoutItsOwnTrackTheCloudsMoverPassesThrough) {
    ogma::ParamMap p; p["output_topic"] = std::string("percept.motion"); p["cloud_mover_topic"] = std::string("percept.mover_bearing");
    Rig r(p);
    r.bus.begin_tick(r.t);
    auto cm = std::make_shared<ogma::ProprioToken>();
    cm->tick_id = r.t; cm->values = Eigen::VectorXf(6); cm->values << -0.3f, 0.95f, 0.5f, 10.0f, 300.0f, 1234.0f;
    r.bus.publish("percept.mover_bearing", cm);
    r.m.tick(r.t); r.bus.end_tick(); ++r.t;
    auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("percept.motion"));
    ASSERT_NE(out, nullptr);
    EXPECT_FLOAT_EQ(out->values[0], -0.3f);
    EXPECT_FLOAT_EQ(out->values[2], 0.5f);
    EXPECT_FLOAT_EQ(out->values[5], 1234.0f) << "the cloud's sighting stamp, so the chase counts it once";
}

// M7, the vouch: the cloud's sighting passes only where this module saw motion evidence lately.  A sighting ahead at 1 m in a
// static scene is refused; the same sighting once a cube has stood where the rays just passed is passed through.
TEST(MotionField, TheCloudsSightingPassesOnlyWhereMotionWasSeen) {
    ogma::ParamMap p; p["output_topic"] = std::string("percept.motion"); p["cloud_mover_topic"] = std::string("percept.mover_bearing");
    p["vouch_m"] = 0.3; p["own_tracks"] = false;
    Rig r(p);
    const auto sighting = [&]() {
        r.bus.begin_tick(r.t);
        auto cm = std::make_shared<ogma::ProprioToken>();
        cm->tick_id = r.t; cm->values = Eigen::VectorXf(6); cm->values << 0.0f, 1.0f, 0.6f, 10.0f, 300.0f, float(r.t);   // 1 m ahead
        r.bus.publish("percept.mover_bearing", cm);
        r.m.tick(r.t); r.bus.end_tick(); ++r.t;
        auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("percept.motion"));
        return out ? out->values[2] : -1.0f;
    };
    for (int k = 0; k < 10; ++k) r.cast(wall(0.003 * (k % 7)));
    EXPECT_FLOAT_EQ(sighting(), 0.0f) << "no motion seen there: refused";
    std::vector<Pt> pts;
    for (int j = 0; j < 4; ++j) pts.push_back({1.0, -0.04 + 0.02 * j, 0.10});
    r.cast(pts);
    EXPECT_FLOAT_EQ(sighting(), 0.6f) << "the cube stood where the rays had passed: vouched";
}
