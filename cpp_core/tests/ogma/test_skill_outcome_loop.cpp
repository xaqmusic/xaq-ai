// =============================================================================
// test_skill_outcome_loop.cpp
//   ogma::SkillOutcomeLoop — learning what a kick does (the duck's things phase, O54).
//   1. AnUnknownThingIsKickedAtArrival — the thing seen, the seek need falls to 0 within reach: one request,
//      on the side the thing lies on.
//   2. TheAnswerIsTheThingSeenAgainWithinTheRadius — a later sighting 0.3 m from the fixed position is the
//      outcome; the node's statistics take it; the outcome token carries node, prediction, observation.
//   3. NotSeenIsNotZero — no sighting within observe_ticks: the outcome is unknown and nothing is learned.
//   4. AKnownThingIsLeftAlone — after min_samples outcomes with no spread, the same node's arrival asks for
//      nothing: habituation.
//   6. TheNeedIsTheUnknownIntentsShare — with two intents the need reads 1 before any answer, 0 while an
//      outcome is in flight, 1/2 once the kick's answer is known, 0 once both are: what renews the seek.
// =============================================================================
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/SkillOutcomeLoop.hpp"

namespace {
struct Rig {
    ogma::InProcessBus bus; ogma::SkillOutcomeLoop m; uint64_t t = 0;
    explicit Rig(ogma::ParamMap p = {}) { m.set_id("outcome"); m.on_setup(&bus, p); }
    // one tick: the body at (x, y, yaw), the thing bearing [vx, vy, prox] (prox 0 = unseen), node, the seek need and range
    void step(double x, double y, double yaw, float vx, float vy, float prox, int node, float need, float range) {
        bus.begin_tick(t);
        auto pose = std::make_shared<ogma::ProprioToken>(); pose->values = Eigen::VectorXf(3); pose->values << float(x), float(y), float(yaw);
        bus.publish("reality.proprio.odom", pose);
        auto b = std::make_shared<ogma::ProprioToken>(); b->values = Eigen::VectorXf(3); b->values << vx, vy, prox;
        bus.publish("percept.thing_bearing", b);
        auto rt = std::make_shared<ogma::RealityToken>(); rt->winner_id = node; bus.publish("reality.cognitive.thing", rt);
        auto sv = std::make_shared<ogma::ProprioToken>(); sv->values = Eigen::VectorXf::Constant(1, need); bus.publish("reality.cognitive.seek_value", sv);
        auto sr = std::make_shared<ogma::ProprioToken>(); sr->values = Eigen::VectorXf::Constant(1, range); bus.publish("reality.cognitive.seek_range", sr);
        m.tick(t); bus.end_tick(); ++t;
    }
    bool requested() {
        auto sk = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("intent.skill"));
        return sk && sk->values[1] > 0.5f;
    }
    int request_id() { return int(std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("intent.skill"))->values[0]); }
    // see the thing dead ahead at 1 m for a while, then arrive (need 1 -> 0 at range 0.2)
    void see_then_arrive(int node, float side_vx = 0.0f) {
        for (int i = 0; i < 8; ++i) step(0, 0, 0, side_vx, 1.0f, 0.6f, node, 1.0f, 1.0f);   // prox 0.6 = 1 m of 2.5
        step(0.8, 0, 0, 0, 0, 0, node, 1.0f, 0.2f);
        step(0.8, 0, 0, 0, 0, 0, node, 0.0f, 0.2f);                                          // the arrival tick
    }
};
}  // namespace

TEST(SkillOutcomeLoop, AnUnknownThingIsKickedAtArrival) {
    Rig r;
    r.see_then_arrive(3, -0.3f);                    // the thing a little to the LEFT (vx = +right)
    EXPECT_TRUE(r.requested());
    EXPECT_EQ(r.request_id(), 0) << "left kick for a thing on the left";
    EXPECT_EQ(r.m.requests(), 1);
    r.step(0.8, 0, 0, 0, 0, 0, 3, 0.0f, 0.2f);
    EXPECT_FALSE(r.requested()) << "a request is one tick";
}

TEST(SkillOutcomeLoop, TheAnswerIsTheThingSeenAgainWithinTheRadius) {
    Rig r;
    r.see_then_arrive(3);
    ASSERT_EQ(r.m.requests(), 1);
    for (int i = 0; i < 40; ++i) r.step(0.8, 0, 0, 0, 0, 0, 3, 0.0f, 0.2f);          // the kick's window passes, nothing seen
    // the thing is seen again, 0.3 m further along +x: from the body at (0.8, 0) it is 0.5 m ahead (prox 0.8)
    for (int i = 0; i < 8; ++i) r.step(0.8, 0, 0, 0.0f, 1.0f, 0.8f, 3, 0.0f, 0.2f);
    EXPECT_EQ(r.m.observed(), 1);
    auto st = r.m.stats().at(ogma::SkillOutcomeLoop::key_of(3, 0));
    EXPECT_EQ(st.n, 1);
    EXPECT_NEAR(st.mean, 0.3, 0.05) << "the displacement from the fixed position";
    auto oc = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("reality.cognitive.outcome"));
    // the outcome token carried [node, pred, obs, surprise, n] on the tick it was observed; check the loop's record instead
    EXPECT_NEAR(r.m.last_surprise(), 0.3 / 0.02, 3.0) << "a first outcome against a zero prediction and no spread is a large surprise";
    (void)oc;
}

TEST(SkillOutcomeLoop, NotSeenIsNotZero) {
    ogma::ParamMap p; p["observe_ticks"] = int64_t{100};
    Rig r(p);
    r.see_then_arrive(5);
    ASSERT_EQ(r.m.requests(), 1);
    for (int i = 0; i < 120; ++i) r.step(0.8, 0, 0, 0, 0, 0, 5, 0.0f, 0.2f);
    EXPECT_EQ(r.m.unknown(), 1);
    EXPECT_EQ(r.m.observed(), 0);
    EXPECT_EQ(r.m.stats().count(ogma::SkillOutcomeLoop::key_of(5, 0)) ? r.m.stats().at(ogma::SkillOutcomeLoop::key_of(5, 0)).n : 0, 0) << "nothing learned from an unseen outcome";
}

TEST(SkillOutcomeLoop, AKnownThingIsLeftAlone) {
    ogma::ParamMap p; p["min_samples"] = int64_t{2}; p["explore_gain"] = 1.0;
    Rig r(p);
    for (int k = 0; k < 2; ++k) {                    // two kicks with the same answer: the thing does not move
        r.see_then_arrive(7);
        ASSERT_EQ(r.m.requests(), k + 1);
        for (int i = 0; i < 40; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
        for (int i = 0; i < 8; ++i) r.step(0.8, 0, 0, 0.0f, 1.0f, 0.92f, 7, 0.0f, 0.2f);   // seen again where it was (0.2 m ahead)
        ASSERT_EQ(r.m.observed(), k + 1);
        for (int i = 0; i < 10; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
    }
    r.see_then_arrive(7);
    EXPECT_EQ(r.m.requests(), 2) << "a node known with no spread is not kicked again";
}

// Two intents: the loop asks for the one whose answer for this thing it knows least.  After a kick's outcome
// is recorded the next arrival at the same node asks for the PECK (id 3); after the peck's, the kick again.
TEST(SkillOutcomeLoop, TheLeastKnownIntentIsAsked) {
    ogma::ParamMap p; p["peck_id"] = int64_t{3}; p["min_samples"] = int64_t{3};
    Rig r(p);
    std::vector<int> asked;
    for (int k = 0; k < 3; ++k) {
        r.see_then_arrive(7);
        ASSERT_EQ(r.m.requests(), k + 1);
        asked.push_back(r.request_id());
        for (int i = 0; i < 40; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
        for (int i = 0; i < 8; ++i) r.step(0.8, 0, 0, 0.0f, 1.0f, 0.9f, 7, 0.0f, 0.2f);   // seen again: an outcome
        ASSERT_EQ(r.m.observed(), k + 1);
        for (int i = 0; i < 10; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
    }
    EXPECT_NE(asked[0], 3) << "the first asks for a kick (the last intent starts as the peck)";
    EXPECT_EQ(asked[1], 3) << "the kick answered once, the peck never: the peck";
    EXPECT_NE(asked[2], 3) << "both answered once with the same spread: the one not tried last, the kick";
}

TEST(SkillOutcomeLoop, TheNeedIsTheUnknownIntentsShare) {
    ogma::ParamMap p; p["peck_id"] = int64_t{3}; p["min_samples"] = int64_t{1}; p["explore_gain"] = 0.0;
    p["need_topic"] = std::string("reality.cognitive.outcome_need");
    Rig r(p);
    auto need = [&]() { return std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("reality.cognitive.outcome_need"))->values[0]; };
    r.step(0, 0, 0, 0, 0, 0, 7, 0.0f, 9.0f);
    EXPECT_FLOAT_EQ(need(), 0.0f) << "nothing seen yet";
    for (int i = 0; i < 8; ++i) r.step(0, 0, 0, 0.0f, 1.0f, 0.6f, 7, 1.0f, 1.0f);
    EXPECT_FLOAT_EQ(need(), 1.0f) << "a thing seen, both intents unknown";
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("reality.cognitive.outcome_need"));
    EXPECT_NEAR(tok->values[1], 1.0f, 0.05f) << "the thing's fixed position travels with the need";
    r.step(0.8, 0, 0, 0, 0, 0, 7, 1.0f, 0.2f);
    r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);                                          // the arrival: a kick asked
    ASSERT_EQ(r.m.requests(), 1);
    EXPECT_FLOAT_EQ(need(), 0.0f) << "an outcome in flight: the loop looks, it does not ask";
    for (int i = 0; i < 40; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
    for (int i = 0; i < 8; ++i) r.step(0.8, 0, 0, 0.0f, 1.0f, 0.9f, 7, 0.0f, 0.2f);   // seen again: the kick's answer
    ASSERT_EQ(r.m.observed(), 1);
    EXPECT_FLOAT_EQ(need(), 0.5f) << "the kick known, the peck not";
    for (int i = 0; i < 10; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
    r.see_then_arrive(7);
    ASSERT_EQ(r.m.requests(), 2);
    EXPECT_EQ(r.request_id(), 3) << "the peck";
    for (int i = 0; i < 40; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
    for (int i = 0; i < 8; ++i) r.step(0.8, 0, 0, 0.0f, 1.0f, 0.9f, 7, 0.0f, 0.2f);
    ASSERT_EQ(r.m.observed(), 2);
    EXPECT_FLOAT_EQ(need(), 0.0f) << "both answered: nothing left to ask this thing";
}

// Three intents: the least-known rule cycles the vocabulary at one thing -- kick, then peck, then push
// (fewest outcomes first, ties broken by the one after the last), and the need counts all three.
TEST(SkillOutcomeLoop, ThreeIntentsAreCycledAtAThing) {
    ogma::ParamMap p; p["peck_id"] = int64_t{3}; p["push_id"] = int64_t{4}; p["min_samples"] = int64_t{2}; p["explore_gain"] = 0.0;
    p["need_topic"] = std::string("reality.cognitive.outcome_need");
    Rig r(p);
    std::vector<int> asked;
    for (int k = 0; k < 6; ++k) {
        r.see_then_arrive(7);
        ASSERT_EQ(r.m.requests(), k + 1);
        asked.push_back(r.request_id());
        for (int i = 0; i < 40; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
        for (int i = 0; i < 8; ++i) r.step(0.8, 0, 0, 0.0f, 1.0f, 0.9f, 7, 0.0f, 0.2f);   // seen again: an outcome
        ASSERT_EQ(r.m.observed(), k + 1);
        for (int i = 0; i < 10; ++i) r.step(0.8, 0, 0, 0, 0, 0, 7, 0.0f, 0.2f);
    }
    EXPECT_NE(asked[0], 3); EXPECT_NE(asked[0], 4);
    EXPECT_EQ(asked[1], 3) << "the kick answered once: the peck";
    EXPECT_EQ(asked[2], 4) << "the peck answered once: the push";
    EXPECT_NE(asked[3], 3); EXPECT_NE(asked[3], 4);
    auto need = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("reality.cognitive.outcome_need"))->values[0];
    EXPECT_FLOAT_EQ(need, 0.0f) << "each of the three known twice: nothing left to ask";
    r.see_then_arrive(7);
    EXPECT_EQ(r.m.requests(), 6) << "a thing whose three answers are known is left alone";
}
