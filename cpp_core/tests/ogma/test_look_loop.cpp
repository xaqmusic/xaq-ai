// LookLoop (the duck, design doc §17.112): the stop-and-look as a loop -- it predicts, per context, the surprise a stop
// resolves (the integral of the stop cloud's new_fraction), learns it from each stop, and hands the arbiter a need.
#include <gtest/gtest.h>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/LookLoop.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::initializer_list<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(long(v.size())); int i = 0; for (float x : v) p->values[i++] = x; return p;
}
std::shared_ptr<ogma::RealityToken> ctx(int w) { auto r = std::make_shared<ogma::RealityToken>(); r->winner_id = w; return r; }

struct Fix {
    ogma::InProcessBus bus; ogma::LookLoop m; uint64_t t = 0; int context = -1; float gain = -1.0f;
    explicit Fix(ogma::ParamMap extra = {}) {
        ogma::ParamMap p{{"state_topic", std::string("reality.proprio.cloud_state")}, {"value_topic", std::string("reality.cognitive.look")},
                         {"driving_topic", std::string("look.driving")}};
        for (auto& [k, v] : extra) p[k] = v;
        m.set_id("look"); m.on_setup(&bus, p);
    }
    // one tick of the cloud: open / walking / new_fraction
    void tick(bool open, bool walking, float nf) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.cloud_state", pv({open ? 1.0f : 0.0f, walking ? 1.0f : 0.0f, 100.0f, nf}));
        if (context >= 0) bus.publish("reality.cognitive.look_context", ctx(context));
        if (gain >= 0.0f) bus.publish("arbiter.gain.look", pv({gain}));
        m.tick(t++); bus.end_tick();
    }
    // a stop of n ticks whose new_fraction decays from nf0 by the factor k a tick, then a walk
    void stop(int n, float nf0, float k = 0.97f) { float nf = nf0; for (int i = 0; i < n; ++i) { tick(true, false, nf); nf *= k; } tick(true, true, 0.5f); }
    std::shared_ptr<const ogma::ProprioToken> out(const char* topic = "reality.cognitive.look") const {
        return std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value(topic));
    }
};
}

TEST(LookLoop, UntriedIsFullNeedAndAStopTeachesItsSurprise) {
    Fix f;
    f.tick(true, true, 0.6f);
    EXPECT_DOUBLE_EQ(f.m.need(), 1.0) << "nothing learned yet: an untried look is worth trying";
    f.stop(200, 0.8f);
    EXPECT_EQ(f.m.episodes(), 1u);
    // the integral of 0.8 * 0.97^i over 200 ticks at 50 Hz
    double expect = 0.0, nf = 0.8; for (int i = 0; i < 200; ++i) { expect += nf / 50.0; nf *= 0.97; }
    EXPECT_NEAR(f.m.last_realized(), expect, 1e-4);
    EXPECT_NEAR(f.m.peak(), expect, 1e-4);
    auto o = f.out(); ASSERT_TRUE(o); ASSERT_EQ(o->values.size(), 5);
    EXPECT_FLOAT_EQ(o->values[2], 0.0f);                   // walking again
}

TEST(LookLoop, InAStopTheNeedIsTheSurpriseOverItsOwnPeak) {
    Fix f;
    f.tick(true, false, 0.4f); f.tick(true, false, 0.8f);
    EXPECT_TRUE(f.m.in_stop());
    EXPECT_NEAR(f.m.need(), 1.0, 1e-6);
    f.tick(true, false, 0.2f);
    EXPECT_NEAR(f.m.need(), 0.25, 1e-6) << "the cloud resolving: the need falls with the surprise";
    auto d = f.out("look.driving"); ASSERT_TRUE(d); EXPECT_FLOAT_EQ(d->values[0], 1.0f);
}

TEST(LookLoop, AContextThatPaysLittleIsPredictedLow) {
    Fix f({{"context_topic", std::string("reality.cognitive.look_context")}});
    for (int k = 0; k < 6; ++k) {
        f.context = 0; f.stop(200, 0.9f, 0.99f);           // context 0: a rich look
        f.context = 1; f.stop(60, 0.2f, 0.9f);             // context 1: a quick, poor one
    }
    EXPECT_EQ(f.m.cell_n(0), 6); EXPECT_EQ(f.m.cell_n(1), 6);
    f.context = 0; f.tick(true, true, 0.5f); const double rich = f.m.prediction();
    f.context = 1; f.tick(true, true, 0.5f); const double poor = f.m.prediction();
    f.context = 7; f.tick(true, true, 0.5f); const double untried = f.m.prediction();
    EXPECT_GT(rich, 0.8);
    EXPECT_LT(poor, 0.2);
    EXPECT_DOUBLE_EQ(untried, 1.0) << "an untried context reads the peak";
}

TEST(LookLoop, AShortStillnessIsNotAStop) {
    Fix f;
    for (int i = 0; i < 10; ++i) f.tick(true, false, 0.5f);
    f.tick(true, true, 0.5f);
    EXPECT_EQ(f.m.episodes(), 0u);
}

TEST(LookLoop, ActiveOnlyCountsTheLooksOwnStops) {
    Fix f({{"active_topic", std::string("arbiter.gain.look")}});
    f.gain = 0.0f; f.stop(100, 0.8f);
    EXPECT_EQ(f.m.episodes(), 0u) << "a stop the look did not win is not the look's";
    f.gain = 1.0f; f.stop(100, 0.8f);
    EXPECT_EQ(f.m.episodes(), 1u);
}

TEST(LookLoop, StateRoundTrips) {
    Fix f({{"context_topic", std::string("reality.cognitive.look_context")}});
    f.context = 3; f.stop(120, 0.7f);
    Fix g({{"context_topic", std::string("reality.cognitive.look_context")}});
    g.m.restore_state(f.m.snapshot_state());
    EXPECT_EQ(g.m.cell_n(3), 1); EXPECT_DOUBLE_EQ(g.m.peak(), f.m.peak()); EXPECT_EQ(g.m.episodes(), 1u);
}

// §17.112 lever 2: the pragmatic look -- a stop PAYS when a seek target appears or a skill is requested during it.
namespace {
struct YFix {
    ogma::InProcessBus bus; ogma::LookLoop m; uint64_t t = 0; float seek = 0.0f; bool skill_now = false;
    YFix() {
        m.set_id("look");
        m.on_setup(&bus, {{"state_topic", std::string("reality.proprio.cloud_state")}, {"value_topic", std::string("reality.cognitive.look")},
                          {"need_mode", std::string("yield")}, {"seek_value_topic", std::string("reality.cognitive.seek_value")},
                          {"skill_topic", std::string("intent.skill")}, {"trust_topic", std::string("reality.loop.look")}, {"optimism", 0.0}});
    }
    void tick(bool still) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.cloud_state", pv({1.0f, still ? 0.0f : 1.0f, 100.0f, 0.5f}));
        bus.publish("reality.cognitive.seek_value", pv({seek}));
        if (skill_now) { auto s = pv({3.0f, 1.0f}); s->tick_id = t; bus.publish("intent.skill", s); skill_now = false; }
        m.tick(t++); bus.end_tick();
    }
    // a stop of n ticks that pays at tick k (k < 0: never); the seek need drops back to 0 on the walk after
    void stop(int n, int k) { for (int i = 0; i < n; ++i) { if (i == k) seek = 0.8f; tick(true); } tick(false); seek = 0.0f; tick(false); }
};
}

TEST(LookLoop, YieldLearnsHowOftenAStopPays) {
    YFix f;
    f.tick(false);
    EXPECT_NEAR(f.m.need(), 0.5, 1e-9) << "untried: the flat prior's mean (optimism 0)";
    for (int i = 0; i < 6; ++i) f.stop(100, 50);     // six stops that paid
    for (int i = 0; i < 2; ++i) f.stop(100, -1);     // two that did not
    EXPECT_NEAR(f.m.need(), 7.0 / 10.0, 1e-9) << "Beta(1+6, 1+2)";
    auto tr = std::dynamic_pointer_cast<const ogma::RealityToken>(f.bus.last_value("reality.loop.look"));
    ASSERT_TRUE(tr); EXPECT_LT(tr->expected_error, 0.3f); EXPECT_GT(tr->expected_error, 0.0f);
}

TEST(LookLoop, YieldInAStopTheNeedIsWhatIsStillToCome) {
    YFix f;
    for (int i = 0; i < 8; ++i) f.stop(200, 50);     // every stop pays, one second in
    f.tick(true);
    const double early = f.m.need();
    for (int i = 0; i < 40; ++i) f.tick(true);       // 0.8 s: before any payoff was ever this late, still hopeful
    EXPECT_NEAR(f.m.need(), early, 1e-9);
    for (int i = 0; i < 20; ++i) f.tick(true);       // past the second every payoff came by: nothing still to come
    EXPECT_LT(f.m.need(), 0.01) << "no payoff has ever come this late: the look is spent";
    f.seek = 0.8f; f.tick(true);
    EXPECT_DOUBLE_EQ(f.m.need(), 0.0) << "it paid: the look is done";
}

TEST(LookLoop, YieldASkillRequestIsAPayoff) {
    YFix f;
    for (int i = 0; i < 30; ++i) f.tick(true);
    f.skill_now = true; f.tick(true);
    EXPECT_DOUBLE_EQ(f.m.need(), 0.0);
    f.tick(false);
    EXPECT_DOUBLE_EQ(f.m.last_realized(), 1.0);
}

TEST(LookLoop, YieldASkillDeferredToTheStopIsItsPayoff) {
    // the host defers a skill requested on the walk to the next stop's hand-back: that stop pays at t = 0
    for (bool counts : {false, true}) {
        ogma::InProcessBus bus; ogma::LookLoop m; m.set_id("look");
        m.on_setup(&bus, {{"state_topic", std::string("reality.proprio.cloud_state")}, {"value_topic", std::string("reality.cognitive.look")},
                          {"need_mode", std::string("yield")}, {"skill_topic", std::string("intent.skill")}, {"pending_skill_pays", counts}});
        uint64_t t = 0;
        auto tick = [&](bool still, bool skill) {
            bus.begin_tick(t);
            bus.publish("reality.proprio.cloud_state", pv({1.0f, still ? 0.0f : 1.0f, 100.0f, 0.5f}));
            if (skill) { auto s = pv({3.0f, 1.0f}); s->tick_id = t; bus.publish("intent.skill", s); }
            m.tick(t++); bus.end_tick();
        };
        tick(false, true);                               // requested on the walk, at the arrival
        for (int i = 0; i < 60; ++i) tick(true, false);  // the stop
        tick(false, false);
        EXPECT_DOUBLE_EQ(m.last_realized(), counts ? 1.0 : 0.0);
    }
}

TEST(LookLoop, YieldSeesASkillRequestedOnThePreviousTick) {
    // the outcome loop runs after the look in the duck's graph: its request reaches the look a tick old, once
    ogma::InProcessBus bus; ogma::LookLoop m; m.set_id("look");
    m.on_setup(&bus, {{"state_topic", std::string("reality.proprio.cloud_state")}, {"value_topic", std::string("reality.cognitive.look")},
                      {"need_mode", std::string("yield")}, {"skill_topic", std::string("intent.skill")}});
    for (uint64_t t = 0; t < 40; ++t) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.cloud_state", pv({1.0f, 0.0f, 100.0f, 0.5f}));
        if (t == 30) { auto s = pv({3.0f, 1.0f}); s->tick_id = 29; bus.publish("intent.skill", s); }
        m.tick(t); bus.end_tick();
    }
    EXPECT_DOUBLE_EQ(m.need(), 0.0) << "the stop paid";
}
