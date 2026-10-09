// StalkLoop (the duck, design doc §17.113 A1): the hunter's ambush -- learn where movers pass, wait there after a lost chase,
// as long as the place's own return predicts; a wait that sees nothing makes the place less worth waiting at.
#include <gtest/gtest.h>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/StalkLoop.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::initializer_list<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(long(v.size())); int i = 0; for (float x : v) p->values[i++] = x; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::StalkLoop m; uint64_t t = 0;
    float px = 0.0f, py = 0.0f, have = 0.0f, chasing = 0.0f, src = 0.0f, mind = 0.0f;
    Fix() { m.set_id("stalk");
            m.on_setup(&bus, {{"mover_pos_topic", std::string("mp")}, {"map_topic", std::string("mm")}, {"target_topic", std::string("tt")},
                              {"mind_topic", std::string("mind")}, {"pose_topic", std::string("pose")}, {"ambush_topic", std::string("amb")},
                              {"pass_gap_ticks", int64_t{100}}}); }
    void tick(bool sight = false, float sx = 0.0f, float sy = 0.0f, int node = 0) {
        bus.begin_tick(t);
        bus.publish("pose", pv({px, py, 0.0f}));
        bus.publish("tt", pv({have, 0.0f, 0.0f, 0.0f, chasing, src}));
        bus.publish("mind", pv({mind}));
        if (sight) {
            auto p = pv({sx / 2.0f, sy / 2.0f}); p->tick_id = t; bus.publish("mp", p);
            auto r = std::make_shared<ogma::RealityToken>(); r->winner_id = node; bus.publish("mm", r);
        }
        m.tick(t++); bus.end_tick();
    }
    std::shared_ptr<const ogma::ProprioToken> amb() const { return std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("amb")); }
};
}

TEST(StalkLoop, LearnsAPassingPlaceAndWaitsThereAfterALostChase) {
    Fix f;
    // the mover passes node 3 at (1, 0) three times, 400 ticks apart, chased each time
    for (int pass = 0; pass < 3; ++pass) {
        f.chasing = 1.0f; f.mind = 1.0f;
        for (int i = 0; i < 10; ++i) f.tick(true, 1.0f, 0.0f, 3);
        f.chasing = 0.0f;
        for (int i = 0; i < 390; ++i) f.tick();
    }
    EXPECT_EQ(f.m.passes(3), 3);
    f.mind = 0.0f; f.tick();
    auto a = f.amb(); ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(a->values[0], 1.0f) << "the mover out of mind, nothing held: go and wait where it passes";
    EXPECT_NEAR(a->values[1], 1.0f, 1e-4f); EXPECT_NEAR(a->values[3], 0.5f, 1e-6f) << "an untried place: P = 0.5";
    EXPECT_EQ(f.m.state(), 1);
    // the body arrives (the seek holds the ambush, source 4) and waits
    f.have = 1.0f; f.src = 4.0f; f.px = 1.0f; f.tick();
    EXPECT_EQ(f.m.state(), 2);
    for (int i = 0; i < 405; ++i) f.tick();          // longer than the place's own return (400): nothing came
    EXPECT_EQ(f.m.waits_failed(), 1);
    EXPECT_EQ(f.m.state(), 0);
    f.have = 0.0f; f.tick();
    EXPECT_EQ(f.m.state(), 0) << "the place just failed is not chosen again at once (the only one)";
}

TEST(StalkLoop, AWaitThatSeesTheMoverPays) {
    Fix f;
    for (int pass = 0; pass < 2; ++pass) { f.chasing = 1.0f; for (int i = 0; i < 10; ++i) f.tick(true, 1.0f, 0.0f, 3); f.chasing = 0.0f; for (int i = 0; i < 290; ++i) f.tick(); }
    f.tick(); ASSERT_EQ(f.m.state(), 1);
    f.have = 1.0f; f.src = 4.0f; f.px = 1.0f; f.tick(); ASSERT_EQ(f.m.state(), 2);
    for (int i = 0; i < 50; ++i) f.tick();
    f.chasing = 1.0f; f.src = 1.0f; f.tick();          // the mover comes by: a chase
    EXPECT_EQ(f.m.waits_paid(), 1); EXPECT_EQ(f.m.state(), 0);
}
