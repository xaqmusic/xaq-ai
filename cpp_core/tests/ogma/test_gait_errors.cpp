// GaitErrors: the gait's own errors per tick (G4b's context input).
#include <gtest/gtest.h>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/GaitErrors.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> gv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)];
    p->producer_id = "host"; return p;
}
}  // namespace

TEST(GaitErrors, SwingContactIsSwingAndTouchingAndStallPassesThrough) {
    ogma::InProcessBus bus; ogma::GaitErrors g; g.set_id("ge"); g.on_setup(&bus, {});
    bus.begin_tick(1);
    const char* legs[4] = {"rhythm.leg.fl", "rhythm.leg.fr", "rhythm.leg.rl", "rhythm.leg.rr"};
    const float swing[4] = {1, 1, 0, 0};
    for (int i = 0; i < 4; ++i) bus.publish(legs[i], gv({1, 0, swing[i], 0.25f}));
    bus.publish("reality.proprio.foot_contact", gv({1, 0, 1, 0}));
    bus.publish("reality.proprio.stuck", gv({0.4f, 0, 0, 0, 0}));
    g.tick(1);
    auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("reality.proprio.gait_errors"));
    ASSERT_TRUE(out);
    // fl: swing + touching = error; fr: swing, in the air = fine; rl: stance + touching = fine; rr: stance, air = fine.
    EXPECT_FLOAT_EQ(out->values[0], 1.0f); EXPECT_FLOAT_EQ(out->values[1], 0.0f);
    EXPECT_FLOAT_EQ(out->values[2], 0.0f); EXPECT_FLOAT_EQ(out->values[3], 0.0f);
    EXPECT_FLOAT_EQ(out->values[4], 0.4f);
    bus.end_tick();
}

TEST(GaitErrors, PublishesNothingBeforeItsInputsArrive) {
    ogma::InProcessBus bus; ogma::GaitErrors g; g.set_id("ge"); g.on_setup(&bus, {});
    bus.begin_tick(1); g.tick(1);
    EXPECT_EQ(bus.last_value("reality.proprio.gait_errors"), nullptr);
    bus.end_tick();
}
