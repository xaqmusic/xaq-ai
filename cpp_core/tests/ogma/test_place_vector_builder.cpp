// PlaceVectorBuilder (Cell round 2, lever A5): the place vector = panorama ; pose × repeat.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/PlaceVectorBuilder.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
}

TEST(PlaceVectorBuilder, LayoutAndPoseRepeatAreDerived) {
    ogma::InProcessBus bus; ogma::PlaceVectorBuilder m; m.set_id("pv");
    m.on_setup(&bus, {{"n_pano", int64_t{24}}, {"odo_scale", 240.0}});
    EXPECT_EQ(m.pose_repeat(), 6);
    EXPECT_EQ(m.out_dims(), 48);
    std::vector<float> pano(24, 0.25f); pano[3] = 0.9f;
    bus.begin_tick(0);
    bus.publish("percept.cylinder", pv(pano));
    bus.publish("reality.proprio.heading", pv({0.0f}));
    bus.publish("reality.proprio.vel_ego", pv({0.0f, 1.0f}));   // forward one unit
    m.tick(0); bus.end_tick();
    auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("percept.place_vec"));
    ASSERT_TRUE(out); ASSERT_EQ(out->values.size(), 48);
    EXPECT_FLOAT_EQ(out->values[3], 0.9f);
    // forward at heading 0 moves -y (body convention): y = -1 → -1/240
    EXPECT_NEAR(out->values[24], 0.0f, 1e-6f);
    EXPECT_NEAR(out->values[25], -1.0f / 240.0f, 1e-6f);
    EXPECT_NEAR(out->values[26], 1.0f, 1e-6f);   // cos 0
    EXPECT_NEAR(out->values[27], 0.0f, 1e-6f);   // sin 0
    for (int r = 1; r < 6; ++r) EXPECT_FLOAT_EQ(out->values[24 + 4 * r + 1], out->values[25]);
}

TEST(PlaceVectorBuilder, OdometryIntegratesOncePerSampleAndClamps) {
    ogma::InProcessBus bus; ogma::PlaceVectorBuilder m; m.set_id("pv");
    m.on_setup(&bus, {{"n_pano", int64_t{4}}, {"odo_scale", 10.0}});
    uint64_t t = 0;
    for (int i = 0; i < 30; ++i) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.heading", pv({float(M_PI) / 2.0f}));   // facing +x? forward = (-sin h, -cos h) = (-1, 0)
        bus.publish("reality.proprio.vel_ego", pv({0.0f, 1.0f}));
        m.tick(t++); bus.end_tick();
    }
    EXPECT_NEAR(m.odo_x(), -30.0, 1e-4);
    EXPECT_NEAR(m.odo_y(), 0.0, 1e-4);
    EXPECT_FLOAT_EQ(m.last_vector()[4], -1.0f) << "x/odo_scale = -3 clamps to -1";
    // no new velocity sample → no further integration
    bus.begin_tick(t); m.tick(t++); bus.end_tick();
    EXPECT_NEAR(m.odo_x(), -30.0, 1e-4);
}

TEST(PlaceVectorBuilder, SnapshotRoundTrip) {
    ogma::InProcessBus bus; ogma::PlaceVectorBuilder m; m.set_id("pv");
    m.on_setup(&bus, {{"n_pano", int64_t{4}}});
    bus.begin_tick(0); bus.publish("percept.cylinder", pv({0.1f, 0.2f, 0.3f, 0.4f}));
    bus.publish("reality.proprio.vel_ego", pv({0.5f, 0.5f})); m.tick(0); bus.end_tick();
    auto snap = m.snapshot_state();
    ogma::InProcessBus bus2; ogma::PlaceVectorBuilder m2; m2.set_id("pv"); m2.on_setup(&bus2, {{"n_pano", int64_t{4}}});
    m2.restore_state(snap);
    EXPECT_DOUBLE_EQ(m2.odo_x(), m.odo_x()); EXPECT_DOUBLE_EQ(m2.odo_y(), m.odo_y());
    EXPECT_TRUE(m2.have_pano());
}
