// =============================================================================
// test_bearing_fusion.cpp
//   BearingFusion — trust-weighted vision+scent bearing blend (decode path A).
//
//   1. BlendByTrust — fused = (w_s·b_s + w_v·b_v)/(w_s+w_v) (renorm off).
//   2. VisionDroppedRidesScent / ScentDroppedRidesVision — a [0,0] bearing
//      (mag < floor) is gated to weight 0; fused == the survivor.
//   3. AbsentTrustKeyTreatedZero — missing trust key → weight 0.
//   4. BothUntrustedEmitsSurvivor — den==0 → higher-magnitude raw bearing.
//   5. ProxPassthrough — 3-D bearings → trust-weighted values[2].
//   6. TrustKeyedByFullTopic — reads "reality.nav.vision", not "vision".
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/BearingFusion.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::ProprioToken> make_bearing(std::vector<float> v, uint64_t tick = 0) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id = tick;
    p->sensor  = "bearing";
    p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[i];
    return p;
}

std::shared_ptr<ogma::ConsensusToken> make_consensus(
        std::unordered_map<std::string, float> trust, uint64_t tick = 0) {
    auto c = std::make_shared<ogma::ConsensusToken>();
    c->tick_id       = tick;
    c->trust_weights = std::move(trust);
    return c;
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::BearingFusion bf;
    explicit Fixture(ogma::ParamMap const& p = {}) {
        bf.set_id("bearing_fusion");
        bf.on_setup(&bus, p);
    }
    void run_tick(uint64_t t,
                  std::shared_ptr<ogma::ConsensusToken> cons,
                  std::shared_ptr<ogma::ProprioToken> scent,
                  std::shared_ptr<ogma::ProprioToken> vision) {
        bus.begin_tick(t);
        if (cons)   bus.publish("consensus.0", cons);
        if (scent)  bus.publish("percept.scent_compass", scent);
        if (vision) bus.publish("percept.visual_bearing", vision);
        bf.tick(t);
        bus.end_tick();
    }
    std::shared_ptr<const ogma::ProprioToken> out() const {
        return std::dynamic_pointer_cast<const ogma::ProprioToken>(
            bus.last_value("percept.fused_bearing"));
    }
};

}  // namespace

TEST(BearingFusion, BlendByTrust) {
    ogma::ParamMap p{{"renormalize", false}};   // assert the raw blend formula
    Fixture f(p);
    f.run_tick(0,
        make_consensus({{"reality.nav.scent", 0.8f}, {"reality.nav.vision", 0.2f}}),
        make_bearing({1.0f, 0.0f}),    // scent: unit right
        make_bearing({0.0f, 1.0f}));   // vision: unit forward
    // den=1.0 → fx=0.8·1+0.2·0=0.8, fy=0.8·0+0.2·1=0.2
    EXPECT_NEAR(f.bf.last_fx(), 0.8f, 1e-5f);
    EXPECT_NEAR(f.bf.last_fy(), 0.2f, 1e-5f);
    EXPECT_NEAR(f.bf.last_w_scent(),  0.8f, 1e-5f);
    EXPECT_NEAR(f.bf.last_w_vision(), 0.2f, 1e-5f);
}

TEST(BearingFusion, VisionDroppedRidesScent) {
    Fixture f;   // renorm default true
    f.run_tick(0,
        make_consensus({{"reality.nav.scent", 0.5f}, {"reality.nav.vision", 0.9f}}),
        make_bearing({0.6f, 0.8f}),    // scent confident (mag 1)
        make_bearing({0.0f, 0.0f}));   // vision dropped (mag 0 < floor)
    EXPECT_FLOAT_EQ(f.bf.last_w_vision(), 0.0f) << "occluded vision gated despite trust 0.9";
    EXPECT_NEAR(f.bf.last_fx(), 0.6f, 1e-4f);
    EXPECT_NEAR(f.bf.last_fy(), 0.8f, 1e-4f);
}

TEST(BearingFusion, ScentDroppedRidesVision) {
    Fixture f;
    f.run_tick(0,
        make_consensus({{"reality.nav.scent", 0.9f}, {"reality.nav.vision", 0.4f}}),
        make_bearing({0.0f, 0.0f}),    // scent dropped
        make_bearing({0.8f, 0.6f}));   // vision confident
    EXPECT_FLOAT_EQ(f.bf.last_w_scent(), 0.0f);
    EXPECT_NEAR(f.bf.last_fx(), 0.8f, 1e-4f);
    EXPECT_NEAR(f.bf.last_fy(), 0.6f, 1e-4f);
}

TEST(BearingFusion, AbsentTrustKeyTreatedZero) {
    Fixture f;
    // Only scent has a trust entry; vision key is absent → weight 0 even though
    // vision is confident.
    f.run_tick(0,
        make_consensus({{"reality.nav.scent", 0.7f}}),
        make_bearing({1.0f, 0.0f}),
        make_bearing({0.0f, 1.0f}));
    EXPECT_FLOAT_EQ(f.bf.last_w_vision(), 0.0f);
    EXPECT_GT(f.bf.last_w_scent(), 0.0f);
    EXPECT_NEAR(f.bf.last_fx(), 1.0f, 1e-4f) << "fused == scent (vision untrusted)";
}

TEST(BearingFusion, BothUntrustedEmitsSurvivor) {
    Fixture f;
    // Empty trust map → both weights 0 → den==0 → fall back to higher-|mag| raw.
    f.run_tick(0,
        make_consensus({}),
        make_bearing({1.0f, 0.0f}),    // mag 1
        make_bearing({0.0f, 0.0f}));   // mag 0
    EXPECT_NEAR(f.bf.last_fx(), 1.0f, 1e-4f);
    EXPECT_NEAR(f.bf.last_fy(), 0.0f, 1e-4f);
}

TEST(BearingFusion, ProxPassthrough) {
    Fixture f;
    f.run_tick(0,
        make_consensus({{"reality.nav.scent", 1.0f}, {"reality.nav.vision", 0.0f}}),
        make_bearing({0.6f, 0.8f, 0.5f}),   // scent with prox 0.5
        make_bearing({0.0f, 0.0f, 0.0f}));  // vision dropped
    auto o = f.out();
    ASSERT_NE(o, nullptr);
    ASSERT_EQ(o->values.size(), 3);
    EXPECT_NEAR(o->values[2], 0.5f, 1e-4f) << "trust-weighted prox carried through";
}

TEST(BearingFusion, TrustKeyedByFullTopic) {
    Fixture f;
    // A decoy short key "vision" that, if (wrongly) read, would zero vision's weight.
    // The module must read the FULL topic "reality.nav.vision" = 0.9.
    f.run_tick(0,
        make_consensus({{"reality.nav.vision", 0.9f}, {"vision", 0.0f}}),
        make_bearing({1.0f, 0.0f}),    // scent confident but no trust entry
        make_bearing({0.0f, 1.0f}));   // vision confident
    EXPECT_NEAR(f.bf.last_w_vision(), 0.9f, 1e-5f)
        << "read reality.nav.vision (0.9), not the 'vision' decoy (0.0)";
    EXPECT_FLOAT_EQ(f.bf.last_w_scent(), 0.0f);
    EXPECT_NEAR(f.bf.last_fy(), 1.0f, 1e-4f) << "fused rides vision";
}
