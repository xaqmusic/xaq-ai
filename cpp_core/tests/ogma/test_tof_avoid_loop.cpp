#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/TofAvoidLoop.hpp"
namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) { auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p; }
struct Fix { ogma::InProcessBus bus; ogma::TofAvoidLoop m; Fix() { m.set_id("avoid"); m.on_setup(&bus, {}); }
    void run(uint64_t t, std::vector<float> tof) { bus.begin_tick(t); bus.publish("reality.proprio.tof", pv(tof)); m.tick(t); bus.end_tick(); } };
}
TEST(TofAvoidLoop, SilentWhenNothingIsNear) {
    Fix f; f.run(0, {0.0f, 0.02f, 0.0f, 0.0f});
    EXPECT_FLOAT_EQ(f.m.last_value(), 0.0f); EXPECT_FLOAT_EQ(f.m.last_cx(), 0.0f); EXPECT_FLOAT_EQ(f.m.last_cy(), 0.0f);
}
TEST(TofAvoidLoop, BearsAwayFromTheNearerSideWithTheNearestProximityAsValue) {
    Fix f; f.run(0, {0.8f, 0.1f, 0.0f, 0.0f});          // wall on the left → bear right
    EXPECT_GT(f.m.last_cx(), 0.0f); EXPECT_GT(f.m.last_cy(), 0.0f); EXPECT_NEAR(f.m.last_value(), 0.8f, 1e-6f);
    f.run(1, {0.0f, 0.1f, 0.6f, 0.0f});                  // wall on the right → bear left
    EXPECT_LT(f.m.last_cx(), 0.0f); EXPECT_NEAR(f.m.last_value(), 0.6f, 1e-6f);
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(f.bus.last_value("percept.avoid_bearing"));
    ASSERT_TRUE(tok); EXPECT_EQ(tok->values.size(), 3);
    auto val = std::dynamic_pointer_cast<const ogma::ProprioToken>(f.bus.last_value("reality.cognitive.avoid_value"));
    ASSERT_TRUE(val); EXPECT_NEAR(val->values[0], 0.6f, 1e-6f);
}
TEST(TofAvoidLoop, BlockedAheadTurnsInPlaceTowardTheFreerSide) {
    Fix f; f.run(0, {0.3f, 0.95f, 0.5f, 0.5f});          // blocked ahead, right nearer → turn left, little forward
    EXPECT_LT(f.m.last_cx(), 0.0f); EXPECT_LT(f.m.last_cy(), 0.4f * std::fabs(f.m.last_cx())) << "mostly a turn, little forward"; EXPECT_NEAR(f.m.last_value(), 0.95f, 1e-6f);
}
