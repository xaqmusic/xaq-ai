// VisualHomingNav (Cell L2 loop #4) — CLOSE on a SEEN food source. The visual twin of klino:
// consumes VisualBearing's egocentric food bearing + a vision-food EPM (tle/node_count), publishes
// an eat-calibrated, informativeness-gated pragmatic value → the arbiter, and the bearing → HC.
#include <gtest/gtest.h>
#include "ogma/modules/VisualHomingNav.hpp"
#include "ogma/InProcessBus.hpp"

#include <cmath>
#include <memory>

namespace {

std::shared_ptr<ogma::RealityToken> vepm(int nodes, float tle) {
    auto r = std::make_shared<ogma::RealityToken>();
    r->winner_id = nodes > 0 ? 0 : -1; r->node_count = nodes; r->tle = tle; return r;
}
std::shared_ptr<ogma::ProprioToken> bearing(float vx, float vy, float prox) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->values.resize(3); p->values[0] = vx; p->values[1] = vy; p->values[2] = prox; return p;
}
std::shared_ptr<ogma::ProprioToken> p1(float v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p;
}
std::shared_ptr<ogma::EnvEvent> eat() {
    auto e = std::make_shared<ogma::EnvEvent>(); e->name = "eat"; return e;
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::VisualHomingNav vh;
    Fixture(ogma::ParamMap extra = {}) {
        vh.set_id("vision");
        vh.on_setup(&bus, extra);
    }
    void run(uint64_t t, int nodes, float tle, float vx, float vy, float prox,
             bool do_eat = false, float heading = 0.0f) {
        bus.begin_tick(t);
        bus.publish("reality.cognitive.vision", vepm(nodes, tle));
        bus.publish("percept.visual_bearing", bearing(vx, vy, prox));
        bus.publish("reality.proprio.heading", p1(heading));
        if (do_eat) bus.publish("events.eat", eat());
        vh.tick(t);
        bus.end_tick();
    }
};

}  // namespace

// OCCLUSION: no food in view (proximity below min_conf) → value 0, bearing [0,0], cede.
TEST(VisualHomingNav, OccludedYieldsZeroValueAndZeroBearing) {
    Fixture f;
    f.run(0, /*nodes=*/5, /*tle=*/0.1f, /*vx=*/0.4f, /*vy=*/0.9f, /*prox=*/0.0f);  // occluded (prox 0)
    EXPECT_FALSE(f.vh.have_food());
    EXPECT_NEAR(f.vh.value(), 0.0f, 1e-6f);
    EXPECT_NEAR(f.vh.last_vx(), 0.0f, 1e-6f);
    EXPECT_NEAR(f.vh.last_vy(), 0.0f, 1e-6f);
}

// The value is ALWAYS a valid pragmatic confidence ∈[0,1].
TEST(VisualHomingNav, ValueInUnitRange) {
    Fixture f;
    for (uint64_t t = 0; t < 50; ++t)
        f.run(t, 5, 0.2f, 0.1f, 1.0f, 0.5f, /*eat=*/(t % 10 == 0));
    EXPECT_GE(f.vh.value(), 0.0f);
    EXPECT_LE(f.vh.value(), 1.0f);
    EXPECT_TRUE(f.vh.have_food());
}

// EAT-CALIBRATION: fold green_frac at the eat into eat_green_; cap_vision → 1 when proximity reaches
// the eat scale, < 1 when the food looks smaller (farther) than it does at the eat.
TEST(VisualHomingNav, EatCalibratesReachConfidence) {
    Fixture f;
    // teach: food is visible at prox 0.5 at the moment of the eat → eat_green_ ≈ 0.5.
    f.run(0, 5, 0.1f, 0.0f, 1.0f, 0.5f, /*eat=*/true);
    EXPECT_TRUE(f.vh.eat_green() > 0.3f && f.vh.eat_green() <= 0.5f) << "learned the eat-scale green";
    // now food at the SAME apparent size → within reach → cap ≈ 1.
    f.run(1, 5, 0.1f, 0.0f, 1.0f, 0.5f);
    EXPECT_GT(f.vh.cap_vision(), 0.9f);
    // food much smaller (farther) → cap well below 1.
    f.run(2, 5, 0.1f, 0.0f, 1.0f, 0.15f);
    EXPECT_LT(f.vh.cap_vision(), 0.5f);
}

// INFORMATIVENESS: the EPM's node_count gates the value — a degenerate occluded-only EPM (1 node)
// yields low informativeness even with food in view; a developed food structure (>=node_ref) → ~1.
TEST(VisualHomingNav, InformativenessGatesValue) {
    Fixture degenerate({{"node_ref", 4.0}});
    degenerate.run(0, /*nodes=*/1, 0.1f, 0.0f, 1.0f, 0.5f, /*eat=*/true);
    degenerate.run(1, /*nodes=*/1, 0.1f, 0.0f, 1.0f, 0.5f);
    float v_degen = degenerate.vh.value();

    Fixture developed({{"node_ref", 4.0}});
    developed.run(0, /*nodes=*/5, 0.1f, 0.0f, 1.0f, 0.5f, /*eat=*/true);
    developed.run(1, /*nodes=*/5, 0.1f, 0.0f, 1.0f, 0.5f);
    float v_dev = developed.vh.value();

    EXPECT_LT(v_degen, v_dev) << "an undeveloped (degenerate) vision-EPM is trusted less";
    EXPECT_NEAR(developed.vh.informativeness(), 1.0f, 1e-3f);
    EXPECT_LT(degenerate.vh.informativeness(), 0.5f);
}

// The output bearing passes the visual food bearing through when food is visible (→ HeadingController).
TEST(VisualHomingNav, BearingPassesThroughWhenFoodVisible) {
    Fixture f;
    f.run(0, 5, 0.1f, /*vx=*/0.6f, /*vy=*/0.8f, /*prox=*/0.4f);
    EXPECT_TRUE(f.vh.have_food());
    EXPECT_NEAR(f.vh.last_vx(), 0.6f, 1e-4f);
    EXPECT_NEAR(f.vh.last_vy(), 0.8f, 1e-4f);
}

// The vision-food EPM's TLE is surfaced (the §1 predictive error / telemetry).
TEST(VisualHomingNav, SurfacesEpmTle) {
    Fixture f;
    f.run(0, 5, /*tle=*/0.37f, 0.1f, 1.0f, 0.5f);
    EXPECT_NEAR(f.vh.epm_tle(), 0.37f, 1e-4f);
    EXPECT_EQ(f.vh.node_count(), 5);
}

// CONSUMER FIRES: value_topic set → publishes vision_value; default (empty) → no publish (§8 default-off).
TEST(VisualHomingNav, PublishesValueWhenTopicSetElseSilent) {
    Fixture on({{"value_topic", std::string("reality.cognitive.vision_value")}});
    on.run(0, 5, 0.1f, 0.0f, 1.0f, 0.5f, /*eat=*/true);
    on.run(1, 5, 0.1f, 0.0f, 1.0f, 0.5f);
    auto vv = std::dynamic_pointer_cast<const ogma::ProprioToken>(
        on.bus.last_value("reality.cognitive.vision_value"));
    ASSERT_NE(vv, nullptr) << "vision_value published when the topic is set";
    ASSERT_EQ(vv->values.size(), 1u);
    EXPECT_GE(float(vv->values[0]), 0.0f);
    EXPECT_LE(float(vv->values[0]), 1.0f);

    Fixture off;  // no value_topic
    off.run(0, 5, 0.1f, 0.0f, 1.0f, 0.5f);
    EXPECT_EQ(off.bus.last_value("reality.cognitive.vision_value"), nullptr)
        << "default-off: no vision_value publish without the topic param";
}

constexpr float PI = 3.14159265f;

// VISUAL TARGET PERSISTENCE (object permanence): food is FOV-gated, so a reactive value flickers to 0 the
// moment food is occluded (a pillar). Persistence remembers the food's bearing and KEEPS homing through
// the occlusion, sustaining the arbiter value, then decays if the food is never re-confirmed.
TEST(VisualHomingNav, PersistsTargetThroughOcclusion) {
    Fixture f({{"persist_decay", 0.02}, {"persist_floor", 0.05}});
    f.run(0, 5, 0.1f, /*vx=*/0.0f, /*vy=*/1.0f, /*prox=*/0.5f);   // food AHEAD, heading 0
    EXPECT_TRUE(f.vh.have_food());
    EXPECT_TRUE(f.vh.have_target());
    EXPECT_GT(f.vh.value(), 0.9f);
    f.run(1, 5, 0.1f, 0.0f, 0.0f, 0.0f);                          // OCCLUDED (no food in view)
    EXPECT_FALSE(f.vh.have_food());
    EXPECT_TRUE(f.vh.persisting()) << "occluded but still homing to the remembered food";
    EXPECT_GT(f.vh.value(), 0.0f) << "value SUSTAINED through occlusion (not flickered to 0)";
    EXPECT_NEAR(f.vh.last_vy(), 1.0f, 0.05f) << "still points to the remembered (forward) bearing";
    for (uint64_t t = 2; t < 400; ++t) f.run(t, 5, 0.1f, 0.0f, 0.0f, 0.0f);  // never re-confirmed
    EXPECT_FALSE(f.vh.have_target()) << "an unconfirmed target eventually decays below floor → cede";
    EXPECT_NEAR(f.vh.value(), 0.0f, 1e-6f);
}

// The remembered target is ALLOCENTRIC (world-fixed): when the bug rotates while the food is occluded,
// the output bearing reprojects so it keeps pointing at the same world location (not the stale ego bearing).
TEST(VisualHomingNav, ReprojectsRememberedTargetUnderRotation) {
    Fixture f({{"persist_decay", 0.001}});
    f.run(0, 5, 0.1f, /*vx=*/0.0f, /*vy=*/1.0f, /*prox=*/0.5f, /*eat=*/false, /*heading=*/0.0f);  // food AHEAD @ heading 0
    EXPECT_TRUE(f.vh.have_target());
    f.run(1, 5, 0.1f, 0.0f, 0.0f, 0.0f, false, /*heading=*/PI / 2);   // occluded + bug rotated +90°
    EXPECT_TRUE(f.vh.persisting());
    EXPECT_NEAR(f.vh.last_vx(), -1.0f, 0.1f) << "world-fixed food reprojects to the side after a 90° turn";
    EXPECT_NEAR(f.vh.last_vy(),  0.0f, 0.1f);
}

// Reaching food (an eat) FULFILS the belief → the target is dropped (food_alternate moves it), even though
// food is still in view on the eat tick — vision cedes so the bug turns to find the next food.
TEST(VisualHomingNav, EatDropsTheRememberedTarget) {
    Fixture f({{"persist_decay", 0.001}});
    f.run(0, 5, 0.1f, 0.0f, 1.0f, 0.5f);
    EXPECT_TRUE(f.vh.have_target());
    f.run(1, 5, 0.1f, 0.0f, 1.0f, 0.5f, /*eat=*/true);
    EXPECT_FALSE(f.vh.have_target()) << "eat fulfils + drops the belief (not re-set by the still-visible frame)";
    EXPECT_NEAR(f.vh.value(), 0.0f, 1e-6f);
}

// Default-off: persist_decay=0 → no persistence (prior per-tick reactive value; occluded → value 0).
TEST(VisualHomingNav, PersistenceOffWhenDecayZero) {
    Fixture f({{"persist_decay", 0.0}});
    f.run(0, 5, 0.1f, 0.0f, 1.0f, 0.5f);
    EXPECT_TRUE(f.vh.have_food());
    f.run(1, 5, 0.1f, 0.0f, 0.0f, 0.0f);   // occluded
    EXPECT_FALSE(f.vh.persisting());
    EXPECT_NEAR(f.vh.value(), 0.0f, 1e-6f) << "no persistence → value flickers to 0 when occluded";
}
