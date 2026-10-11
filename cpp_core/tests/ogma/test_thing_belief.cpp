// ThingBelief (the duck, design doc §17.113): the presence of the thing the seek loop is sent to -- confirmations and the
// place seen through, counted since the target was set.
#include <gtest/gtest.h>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ThingBelief.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::initializer_list<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(long(v.size())); int i = 0; for (float x : v) p->values[i++] = x; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::ThingBelief m; uint64_t t = 0;
    Fix() { m.set_id("thing_belief");
            m.on_setup(&bus, {{"target_topic", std::string("seek.target")}, {"presence_topic", std::string("cloud.presence")},
                              {"belief_topic", std::string("thing.belief")}}); }
    void tick(float have, float set_tick, float chasing, float conf, float refute, float in_cloud = 1.0f) {
        bus.begin_tick(t);
        bus.publish("seek.target", pv({have, 1.0f, 0.0f, set_tick, chasing}));
        bus.publish("cloud.presence", pv({conf, refute, in_cloud, 0.1f}));
        m.tick(t++); bus.end_tick();
    }
};
}

TEST(ThingBelief, ThePlaceSeenThroughLowersPresenceAndASightingRaisesIt) {
    Fix f;
    f.tick(1, 10, 0, 0, 0);
    EXPECT_DOUBLE_EQ(f.m.presence(), 0.5) << "a target with no evidence yet";
    for (int i = 0; i < 8; ++i) f.tick(1, 10, 0, 0, 3);
    EXPECT_NEAR(f.m.presence(), 1.0 / 10.0, 1e-9) << "eight ticks of the place seen empty";
    for (int i = 0; i < 8; ++i) f.tick(1, 10, 0, 1, 0);
    EXPECT_NEAR(f.m.presence(), 9.0 / 18.0, 1e-9);
    f.tick(1, 10, 0, 0, 2, 0.0f);
    EXPECT_EQ(f.m.refuting(), 8) << "evidence outside the cloud does not count";
}

TEST(ThingBelief, ANewTargetForgetsAndAChasedThingIsPresent) {
    Fix f;
    for (int i = 0; i < 5; ++i) f.tick(1, 10, 0, 0, 1);
    EXPECT_LT(f.m.presence(), 0.2);
    f.tick(1, 20, 0, 0, 0);
    EXPECT_DOUBLE_EQ(f.m.presence(), 0.5) << "a new target, a new thing";
    f.tick(1, 20, 1, 0, 1);
    EXPECT_DOUBLE_EQ(f.m.presence(), 1.0) << "chased: present by being chased";
    f.tick(1, 20, 0, 0, 0);
    EXPECT_DOUBLE_EQ(f.m.presence(), 0.5) << "the chase ended: the stopped thing is a new place";
}
