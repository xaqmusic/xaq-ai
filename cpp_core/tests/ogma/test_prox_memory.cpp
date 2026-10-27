// ProxMemory: the duck's proximity slots from a single beam carried by odometry.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ProxMemory.hpp"
namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
struct Fix {
    ogma::InProcessBus bus; ogma::ProxMemory m;
    Fix(ogma::ParamMap p = {}) { m.set_id("prox"); m.on_setup(&bus, p); }
    void run(uint64_t t, float d, bool valid, double x, double y, double yaw) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.odom", pv({float(x), float(y), float(yaw)}));
        bus.publish("sense.range", pv({d, valid ? 1.0f : 0.0f}));
        m.tick(t); bus.end_tick();
    }
    std::vector<float> prox() const {
        auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("reality.proprio.prox"));
        std::vector<float> v; if (tok) for (int i = 0; i < tok->values.size(); ++i) v.push_back(tok->values[i]); return v;
    }
};
}
TEST(ProxMemory, NothingValidMeansEmptySlots) {
    Fix f; f.run(0, 1.5f, false, 0, 0, 0);
    auto p = f.prox(); ASSERT_EQ(p.size(), 4u);
    for (float v : p) EXPECT_FLOAT_EQ(v, 0.0f);
    EXPECT_EQ(f.m.points(), 0);
}
TEST(ProxMemory, AWallAheadFillsTheAheadSlotWithOneMinusRangeOverRangeM) {
    Fix f; f.run(0, 0.5f, true, 0, 0, 0);
    auto p = f.prox();
    EXPECT_NEAR(p[1], 0.5f, 1e-5f) << "ahead = 1 - 0.5/1.0";
    EXPECT_FLOAT_EQ(p[0], 0.0f); EXPECT_FLOAT_EQ(p[2], 0.0f); EXPECT_FLOAT_EQ(p[3], 0.0f);
}
TEST(ProxMemory, AHeldReadingIsOnePointAndAYawTurnMovesItToTheSide) {
    Fix f;
    // Three ticks of the same held reading: one point, not three.
    for (uint64_t t = 0; t < 3; ++t) f.run(t, 0.4f, true, 0, 0, 0);
    EXPECT_EQ(f.m.points(), 1);
    // Now the body yaws +30 deg toward body +X (positive yaw): the remembered point, which was
    // straight ahead, is now on the body's -X side (azimuth negative) -> slot 0.
    f.run(3, 1.5f, false, 0, 0, 30.0 * M_PI / 180.0);
    auto p = f.prox();
    EXPECT_GT(p[0], 0.4f) << "the point moved to the -X side";
    EXPECT_FLOAT_EQ(p[1], 0.0f) << "nothing is ahead any more";
    EXPECT_FLOAT_EQ(p[2], 0.0f);
    // A yaw of -30 deg puts it on the +X side instead.
    Fix g; for (uint64_t t = 0; t < 2; ++t) g.run(t, 0.4f, true, 0, 0, 0);
    g.run(2, 1.5f, false, 0, 0, -30.0 * M_PI / 180.0);
    auto q = g.prox(); EXPECT_GT(q[2], 0.4f); EXPECT_FLOAT_EQ(q[0], 0.0f);
}
TEST(ProxMemory, WalkingPastAPointDropsItBehind) {
    Fix f; f.run(0, 0.3f, true, 0, 0, 0);
    // Walk 0.6 m forward along the heading: the point is now 0.25 m BEHIND the beam's origin.
    f.run(1, 1.5f, false, 0.6, 0, 0);
    EXPECT_EQ(f.m.points(), 0) << "a point behind the sensor is dropped";
    auto p = f.prox(); EXPECT_FLOAT_EQ(p[1], 0.0f);
}
TEST(ProxMemory, MemoryAgesOut) {
    Fix f({{"memory_ticks", int64_t{5}}});
    f.run(0, 0.3f, true, 0, 0, 0);
    for (uint64_t t = 1; t <= 10; ++t) f.run(t, 1.5f, false, 0, 0, 0);
    EXPECT_EQ(f.m.points(), 0);
}
TEST(ProxMemory, TooCloseIsTheLivePingOnly) {
    Fix f; f.run(0, 0.05f, true, 0, 0, 0);
    auto p = f.prox(); EXPECT_NEAR(p[3], 1.0f - 0.05f / 0.15f, 1e-5f);
    f.run(1, 1.5f, false, 0, 0, 0);                  // the ping is gone: contact is not remembered
    auto q = f.prox(); EXPECT_FLOAT_EQ(q[3], 0.0f); EXPECT_GT(q[1], 0.9f) << "the point itself is remembered";
}

// ---- corroboration (support_n, too_close_pings) ----
TEST(ProxMemory, ALoneFalseCloseHitIsIgnoredWhenThreePointsMustAgree) {
    // A wall at ~0.6 m pinged repeatedly (slightly different values, as a real sensor reads), then ONE false 0.077 m
    // hit (a toe in the beam), then the wall again.  With support_n 1 the false hit is the ahead obstacle for the whole
    // memory; with support_n 3 the ahead slot keeps reading the wall.
    for (int64_t k : {int64_t{1}, int64_t{3}}) {
        Fix f({{"support_n", k}});
        uint64_t t = 0;
        for (int i = 0; i < 10; ++i) f.run(t++, 0.60f + 0.001f * float(i % 3), true, 0, 0, 0);
        f.run(t++, 0.077f, true, 0, 0, 0);
        for (int i = 0; i < 5; ++i) f.run(t++, 0.60f + 0.001f * float(i % 3), true, 0, 0, 0);
        const float ahead = f.prox()[1];
        if (k == 1) EXPECT_GT(ahead, 0.9f) << "nearest point: the false hit at 0.077 m dominates";
        else        EXPECT_NEAR(ahead, 0.4f, 0.01f) << "3rd nearest: the wall at 0.6 m";
    }
}
TEST(ProxMemory, ARealObstacleIsBelievedOnceEnoughPingsLandOnIt) {
    Fix f({{"support_n", int64_t{3}}});
    f.run(0, 0.30f, true, 0, 0, 0);
    EXPECT_FLOAT_EQ(f.prox()[1], 0.0f) << "one ping: not yet believed";
    f.run(1, 0.301f, true, 0, 0, 0);
    EXPECT_FLOAT_EQ(f.prox()[1], 0.0f) << "two pings: not yet";
    f.run(2, 0.302f, true, 0, 0, 0);
    EXPECT_NEAR(f.prox()[1], 0.698f, 0.01f) << "three pings on the same spot: believed";
}
TEST(ProxMemory, TooCloseWaitsForConsecutiveClosePingsAndOneFarPingResetsIt) {
    Fix f({{"too_close_pings", int64_t{2}}});
    f.run(0, 0.08f, true, 0, 0, 0);
    EXPECT_FLOAT_EQ(f.prox()[3], 0.0f) << "one close ping is not contact";
    f.run(1, 0.081f, true, 0, 0, 0);
    EXPECT_GT(f.prox()[3], 0.4f) << "two in a row are";
    f.run(2, 0.60f, true, 0, 0, 0);
    f.run(3, 0.08f, true, 0, 0, 0);
    EXPECT_FLOAT_EQ(f.prox()[3], 0.0f) << "a far ping between resets the streak";
}
