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
