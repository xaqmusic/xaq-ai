// PlaceGraphPlanner (Pathway D) — route-following edges, food-memory, value
// propagation, scent-vs-plan arbitration.
#include <gtest/gtest.h>
#include "ogma/modules/PlaceGraphPlanner.hpp"
#include "ogma/InProcessBus.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace {

std::shared_ptr<ogma::RealityToken> place(int wid, float tle = 0.0f) {
    auto r = std::make_shared<ogma::RealityToken>(); r->winner_id = wid; r->tle = tle; return r;
}
std::shared_ptr<ogma::ProprioToken> p1(float v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p;
}
std::shared_ptr<ogma::ProprioToken> p2(float a, float b) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(2); p->values[0] = a; p->values[1] = b; return p;
}
std::shared_ptr<ogma::EnvEvent> hit() {
    auto e = std::make_shared<ogma::EnvEvent>(); e->name = "eat"; return e;   // ground-truth eat (events.eat)
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::PlaceGraphPlanner plan;
    Fixture(ogma::ParamMap extra = {}) {
        plan.set_id("planner");
        plan.on_setup(&bus, extra);
    }
    // node<0 → don't publish place this tick. sb_* <-99 → don't publish scent bearing.
    void run(uint64_t t, int node, float heading, float scent, float hunger,
             bool do_hit = false, float sbx = -99, float sby = -99, float tle = 0.0f) {
        bus.begin_tick(t);
        if (node >= 0) bus.publish("reality.cognitive.place", place(node, tle));
        bus.publish("reality.proprio.heading", p1(heading));
        bus.publish("reality.proprio.scent_max", p1(scent));
        bus.publish("reality.proprio.hunger", p1(hunger));
        if (sbx > -90) bus.publish("percept.scent_compass", p2(sbx, sby));
        if (do_hit)    bus.publish("events.eat", hit());
        plan.tick(t);
        bus.end_tick();
    }
};

constexpr float PI = 3.14159265f;

}  // namespace

TEST(PlaceGraphPlanner, LearnsEdgeHeading) {
    Fixture f;
    f.run(0, /*node=*/0, /*heading=*/0.0f, 0.1f, 0.9f);   // arrive node 0
    f.run(1, /*node=*/1, /*heading=*/0.0f, 0.1f, 0.9f);   // 0→1 travelling heading 0
    f.run(2, /*node=*/2, /*heading=*/PI / 2, 0.1f, 0.9f); // 1→2 travelling heading +pi/2
    EXPECT_NEAR(f.plan.edge_heading(0, 1), 0.0f, 1e-3f);
    EXPECT_NEAR(f.plan.edge_heading(1, 2), PI / 2, 1e-3f);
    EXPECT_EQ(f.plan.edge_count(0, 1), 1);
}

TEST(PlaceGraphPlanner, FoodMemoryOnHit) {
    Fixture f;
    f.run(0, 0, 0.0f, 0.1f, 0.9f, /*hit=*/true);
    EXPECT_GT(f.plan.food_value(0), 0.0f);
    EXPECT_EQ(f.plan.food_value(1), 0.0f);
}

TEST(PlaceGraphPlanner, ValuePropagatesTowardFood) {
    Fixture f;
    f.run(0, 0, 0.0f, 0.1f, 0.9f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f);
    f.run(2, 2, PI / 2, 0.1f, 0.9f, /*hit=*/true);   // food at node 2
    // value gradient: V[2] > V[1] > V[0]
    EXPECT_GT(f.plan.value(2), f.plan.value(1));
    EXPECT_GT(f.plan.value(1), f.plan.value(0));
    EXPECT_GT(f.plan.value(0), 0.0f);
}

TEST(PlaceGraphPlanner, PlansToRememberedFoodWhenStalledAndHungry) {
    Fixture f;
    f.run(0, 0, 0.0f, 0.1f, 0.9f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f);
    f.run(2, 2, PI / 2, 0.1f, 0.9f, /*hit=*/true);
    // bug back at node 0, hungry, scent FLAT (stalled) → should PLAN toward node 1
    f.run(3, 0, 0.0f, 0.1f, 0.9f);
    f.run(4, 0, 0.0f, 0.1f, 0.9f);
    EXPECT_TRUE(f.plan.planning());
    EXPECT_EQ(f.plan.next_node(), 1) << "best next hop from 0 is toward food";
    // travelling heading 0, target edge_heading 0 → forward bearing
    EXPECT_NEAR(f.plan.last_fy(), 1.0f, 0.05f);
    EXPECT_NEAR(f.plan.last_fx(), 0.0f, 0.05f);
}

TEST(PlaceGraphPlanner, PlansProactivelyIgnoringScent) {
    // PROACTIVE design (was ForagesWhenScentRising): the planner no longer defers to a rising
    // scent — klino owns the scent loop. With remembered food it plans toward it regardless of
    // the scent trend (explore/exploit lives in V[n], not a scent-stall gate).
    Fixture f;
    f.run(0, 0, 0.0f, 0.1f, 0.9f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f);
    f.run(2, 2, PI / 2, 0.1f, 0.9f, /*hit=*/true);
    for (uint64_t t = 3; t < 12; ++t)
        f.run(t, 0, 0.0f, /*scent=*/0.1f + 0.05f * float(t), 0.9f, false, 0.7f, 0.7f);
    EXPECT_TRUE(f.plan.planning()) << "proactive: plans toward food regardless of scent";
    EXPECT_EQ(f.plan.next_node(), 1) << "routes toward remembered food";
    EXPECT_NEAR(f.plan.last_fy(), 1.0f, 0.05f) << "plan bearing toward node 1, not scent passthrough";
    EXPECT_NEAR(f.plan.last_fx(), 0.0f, 0.05f);
}

TEST(PlaceGraphPlanner, VisionHomingOverridesEverything) {
    Fixture f({{"vision_topic", std::string("percept.visual_bearing")}, {"vision_floor", 0.05}});
    // vision sees food in line of sight → home straight on it, over scent/plan/wander.
    f.bus.begin_tick(0);
    f.bus.publish("reality.cognitive.place", place(0));
    f.bus.publish("reality.proprio.heading", p1(0.0f));
    f.bus.publish("reality.proprio.scent_max", p1(0.1f));
    f.bus.publish("reality.proprio.hunger", p1(0.9f));
    f.bus.publish("percept.visual_bearing", p2(0.6f, 0.8f));
    f.plan.tick(0); f.bus.end_tick();
    EXPECT_TRUE(f.plan.homing_vision());
    EXPECT_FALSE(f.plan.planning());
    EXPECT_NEAR(f.plan.last_fx(), 0.6f, 1e-3f);
    EXPECT_NEAR(f.plan.last_fy(), 0.8f, 1e-3f);
}

TEST(PlaceGraphPlanner, WandersWhenLostNoForageNoPlan) {
    Fixture f;
    // flat scent (stalled) + no food anywhere (no route) + no scent bearing published →
    // neither forage nor plan → WANDER forward instead of freezing on [0,0].
    for (uint64_t t = 0; t < 4; ++t) f.run(t, 0, 0.0f, 0.1f, 0.9f);   // no hit, no scent bearing
    EXPECT_TRUE(f.plan.wandering());
    EXPECT_FALSE(f.plan.planning());
    EXPECT_NEAR(f.plan.last_fy(), 1.0f, 1e-3f) << "drives forward";
    EXPECT_NEAR(f.plan.last_fx(), 0.0f, 1e-3f);
}

TEST(PlaceGraphPlanner, SatedExploresByRoutingOnUnifiedField) {
    // UNIFIED ALWAYS-ROUTE field (HK habituation era): hunger gates the food term, so when sated
    // the pragmatic pull collapses, but the epistemic (TLE) + residual value still forms a gradient
    // → the planner EXPLORES by ROUTING on that gradient (it does NOT fall back to the blind
    // run-and-tumble; that branch is now only the empty/flat-map bootstrap). The de-scaffold:
    // explore and exploit share one value field. (Was ExploresWhenSated, which asserted run-tumble.)
    Fixture f;
    f.run(0, 0, 0.0f, 0.1f, 0.9f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f);
    f.run(2, 2, PI / 2, 0.1f, 0.9f, /*hit=*/true);   // food known at node 2
    f.run(3, 0, 0.0f, 0.1f, /*hunger=*/0.0f);        // sated
    EXPECT_TRUE(f.plan.planning()) << "always-route: a value gradient exists → route on it (explore)";
    EXPECT_FALSE(f.plan.wandering()) << "routes on the unified field, not the run-tumble bootstrap";
    EXPECT_GE(f.plan.next_node(), 0) << "has a routed next hop";
}

// PURE SEARCHER: the planner NEVER republishes the scent bearing — even when a scent
// bearing IS present on percept.scent_compass, the planner's output is the plan/explore
// bearing (klino owns the scent loop). The scent-vs-plan gate is neutralized for the L2
// arbiter era. Here: no food, hungry, with a strong scent bearing present → still wanders
// (explore), output is the run bearing, NOT the scent bearing [0.7, 0.7].
TEST(PlaceGraphPlanner, PureSearcherNeverRepublishesScent) {
    Fixture f;
    for (uint64_t t = 0; t < 6; ++t)
        f.run(t, 0, 0.0f, /*scent=*/0.1f, /*hunger=*/0.9f, false, /*sbx=*/0.7f, /*sby=*/0.7f);
    EXPECT_FALSE(f.plan.planning());
    EXPECT_TRUE(f.plan.wandering()) << "no food → explore (run-and-tumble), regardless of scent bearing";
    // output is the forward run bearing, NOT the scent passthrough (0.7, 0.7)
    EXPECT_NEAR(f.plan.last_fy(), 1.0f, 0.1f) << "run drives forward, not the scent bearing";
    EXPECT_GT(std::fabs(f.plan.last_fy()) - std::fabs(f.plan.last_fx()), 0.0f)
        << "forward-dominant run, not the diagonal scent bearing";
}

// L2 arbiter input: the planner publishes the FOOD-ROUTE value on plan_value_topic =
// value(next_node) ONLY when food_known && route_exists (a committed route to remembered
// food), else 0 (merely exploring). This SUSTAINED LEVEL — not a z-score — is what stops a
// blind klino from falsely interrupting a steady route (the arbiter normalises it by its peak).
TEST(PlaceGraphPlanner, PublishesFoodRouteValueWhenRoutingToFood) {
    Fixture f({{"plan_value_topic", std::string("reality.cognitive.plan_value")}});
    f.run(0, 0, 0.0f, 0.1f, 0.9f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f);
    f.run(2, 2, PI / 2, 0.1f, 0.9f, /*hit=*/true);   // food at node 2 → value gradient
    // back at node 0, hungry, stalled → plans toward node 1; food known + route exists.
    f.run(3, 0, 0.0f, 0.1f, 0.9f);
    f.run(4, 0, 0.0f, 0.1f, 0.9f);
    ASSERT_TRUE(f.plan.planning());
    auto pv = std::dynamic_pointer_cast<const ogma::ProprioToken>(
        f.bus.last_value("reality.cognitive.plan_value"));
    ASSERT_NE(pv, nullptr) << "plan_value must be published when the topic is set";
    ASSERT_EQ(pv->values.size(), 1u);
    int nxt = f.plan.next_node();
    EXPECT_GE(nxt, 0);
    EXPECT_NEAR(float(pv->values[0]), f.plan.value(nxt), 1e-4f)
        << "published plan_value == value(next_node) (the food-route value) when routing to food";
    EXPECT_NEAR(float(pv->values[0]), f.plan.last_plan_value(), 1e-4f);
    EXPECT_GT(float(pv->values[0]), 0.0f) << "a real food route has positive value";
}

// plan_value is 0 while merely EXPLORING (no remembered food route) — even when the planner
// routes on the epistemic frontier. The arbiter must read v_planner=0 here so klino is free to
// forage; a positive plan_value is reserved for a committed route to REMEMBERED FOOD.
TEST(PlaceGraphPlanner, PlanValueIsZeroWhileExploringNoFood) {
    Fixture f({{"plan_value_topic", std::string("reality.cognitive.plan_value")}});
    // build an epistemic gradient (high-TLE frontier) with NO food anywhere.
    f.run(0, 0, 0.0f, 0.1f, 0.9f, false, -99, -99, /*tle=*/0.0f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f, false, -99, -99, /*tle=*/0.0f);
    f.run(2, 2, PI / 2, 0.1f, 0.9f, false, -99, -99, /*tle=*/1.0f);  // unmodelled frontier, NO food
    f.run(3, 0, 0.0f, 0.1f, 0.9f);                                    // back at 0, hungry, NO food
    EXPECT_TRUE(f.plan.planning()) << "still routes on the epistemic frontier";
    auto pv = std::dynamic_pointer_cast<const ogma::ProprioToken>(
        f.bus.last_value("reality.cognitive.plan_value"));
    ASSERT_NE(pv, nullptr);
    EXPECT_FLOAT_EQ(float(pv->values[0]), 0.0f)
        << "exploring (no remembered food) → plan_value=0 so the arbiter leaves klino to forage";
    EXPECT_FLOAT_EQ(f.plan.last_plan_value(), 0.0f);
}

// plan_value defaults OFF: with no plan_value_topic configured, nothing is published.
TEST(PlaceGraphPlanner, PlanValueOffByDefault) {
    Fixture f;   // no plan_value_topic
    f.run(0, 0, 0.0f, 0.1f, 0.9f, /*hit=*/true);
    EXPECT_EQ(f.bus.last_value("reality.cognitive.plan_value"), nullptr)
        << "default-off: no plan_value publish without the topic param";
}

TEST(PlaceGraphPlanner, PlanPrecisionSharpForOneCacheLowForTwo) {
    // §2.3 MODEL PRECISION = sharpness of the food belief (published to the explicit-EFE arbiter).
    // ONE known cache ⇒ the belief is a point mass ⇒ precision 1. TWO ~equal caches ⇒ high
    // belief-entropy ⇒ precision → 0. This is the CONTROLLED precision the arbiter compares
    // against klino's SENSORY precision (cap) instead of a static 1/(tle+ε), and the signal that
    // dips-then-recovers when the food relocates (the Stage-4 re-inference demo).
    {
        Fixture f;
        f.run(0, 0, 0.0f, 0.1f, 0.9f, /*hit=*/true);   // eat at node 0 only
        f.run(1, 0, 0.0f, 0.1f, 0.9f);                 // dwell, no more eats
        EXPECT_GT(f.plan.last_plan_precision(), 0.99f)
            << "a single known cache = point-mass belief = sharp precision (got "
            << f.plan.last_plan_precision() << ")";
    }
    {
        Fixture f;
        f.run(0, 0, 0.0f, 0.1f, 0.9f, /*hit=*/true);   // eat at node 0
        f.run(1, 1, 0.0f, 0.1f, 0.9f);
        f.run(2, 2, PI / 2, 0.1f, 0.9f, /*hit=*/true); // eat at node 2 — a SECOND, competing cache
        f.run(3, 2, 0.0f, 0.1f, 0.9f);
        EXPECT_LT(f.plan.last_plan_precision(), 0.5f)
            << "two ~equal caches = split belief = low precision (got "
            << f.plan.last_plan_precision() << ")";
    }
}

TEST(PlaceGraphPlanner, NoFoodRoutesToEpistemicFrontier) {
    // UNIFIED ALWAYS-ROUTE field: with no food but a high-TLE (unmodelled) node, the epistemic
    // term builds a value gradient the planner ROUTES on — toward the least-modelled frontier
    // (node 2), which is directed exploration, strictly better than a blind run-and-tumble.
    // The value field carries the epistemic TLE (value(2) > value(0)) and that is now a real
    // route target, not just a gate. (Was ExploresWhenNoFood, which asserted run-tumble.)
    Fixture f;
    f.run(0, 0, 0.0f, 0.1f, 0.9f, false, -99, -99, /*tle=*/0.0f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f, false, -99, -99, /*tle=*/0.0f);
    f.run(2, 2, PI / 2, 0.1f, 0.9f, false, -99, -99, /*tle=*/1.0f);  // node 2 unmodelled, NO food
    f.run(3, 0, 0.0f, 0.1f, 0.9f);                                    // back at 0, hungry, NO food
    EXPECT_TRUE(f.plan.planning()) << "no food but epistemic gradient → route to the frontier";
    EXPECT_FALSE(f.plan.wandering()) << "routes on the unified field, not the run-tumble bootstrap";
    EXPECT_GE(f.plan.next_node(), 0) << "has a routed next hop toward the frontier";
    EXPECT_GT(f.plan.value(2), f.plan.value(0)) << "epistemic TLE still in the value field";
}

TEST(PlaceGraphPlanner, HabituationDeOscillatesBetweenTwoFoodCaches) {
    // HK habituation ("recent = boring") is the de-oscillation drive: with TWO food caches the bare
    // value field ping-pongs between them; habituation breaks the tie by suppressing the LOCAL
    // (food) value of the recently-camped cache, so the OTHER cache becomes the higher-value
    // target and the planner sweeps toward it instead of locking. (The propagated γ·best term is
    // NOT suppressed → a habituated node remains a valid through-waypoint, per design.)
    Fixture f;
    // Line 0-1-2-3, food at endpoints 1 and 3 — two competing value peaks.
    f.run(0, 0, 0.0f, 0.1f, 0.5f);
    f.run(1, 1, 0.0f, 0.1f, 0.5f, /*hit=*/true);   // food cache A at node 1
    f.run(2, 2, 0.0f, 0.1f, 0.5f);
    f.run(3, 3, 0.0f, 0.1f, 0.5f, /*hit=*/true);   // food cache B at node 3
    f.run(4, 2, 0.0f, 0.1f, 0.5f);                 // standing at the midpoint
    float v1_before = f.plan.value(1), v3_before = f.plan.value(3);
    EXPECT_NEAR(v1_before, v3_before, 0.05f * std::max(v1_before, v3_before))
        << "fresh: the two caches are near-tied (the ping-pong condition)";
    // Camp on cache A (node 1) for a long dwell → habituation saturates there.
    for (uint64_t t = 5; t < 400; ++t) f.run(t, 1, 0.0f, 0.1f, 0.5f);
    EXPECT_GT(f.plan.hab_cur(), 0.5f) << "long dwell saturates the current node's habituation";
    EXPECT_GT(f.plan.max_hab(), 0.5f);
    EXPECT_GT(f.plan.n_nodes_hab(), 0);
    // Back at the midpoint: the camped cache A is now suppressed BELOW the un-camped cache B,
    // so routing prefers B (recent = boring → sweep to the other region).
    f.run(400, 2, 0.0f, 0.1f, 0.5f);
    EXPECT_LT(f.plan.value(1), f.plan.value(3))
        << "habituation pushes the recently-camped cache below the un-visited one → de-oscillation";
    EXPECT_LT(f.plan.value(1), v1_before)
        << "the camped cache's value dropped from its fresh (pre-dwell) value";
}

// (patrol) COVERAGE PATROLLER (2026-07-07): with patrol_mode the planner drops food memory and routes
// toward the LEAST-RECENTLY-VISITED known node. It always ROUTES to a reachable adjacent node — never
// drops into the run-tumble wander (play's job) and never fixates on a stale target — and plan_value
// reports the frontier coverage need ∈[0,1]. This is the "planner never gets stuck" property: a route
// target is always an adjacent node connected by a traversed edge, hence reachable by construction.
TEST(PlaceGraphPlanner, PatrolModeRoutesLeastRecentNeverWanders) {
    Fixture f({{"patrol_mode", true},
               {"plan_value_topic", std::string("reality.cognitive.plan_value")}});
    // build a 3-node line map 0-1-2 (walk 0→1→2→1→0 so both edges are learned both directions)
    f.run(0, 0, 0.0f, 0.1f, 0.9f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f);   // 0→1
    f.run(2, 2, 0.0f, 0.1f, 0.9f);   // 1→2  (node 2 visited EARLIEST)
    f.run(3, 1, 0.0f, 0.1f, 0.9f);   // 2→1
    f.run(4, 0, 0.0f, 0.1f, 0.9f);   // 1→0
    f.run(5, 1, 0.0f, 0.1f, 0.9f);   // back at 1 (freshly re-visited → high hab)
    // HOLD at node 1: a healthy patroller keeps ROUTING to a less-recent neighbour, never wanders.
    int route_ticks = 0, wander_ticks = 0; bool saw_neighbor = false;
    for (int i = 0; i < 40; ++i) {
        f.run(6 + i, 1, 0.0f, 0.1f, 0.9f);
        if (f.plan.planning())  ++route_ticks;
        if (f.plan.wandering()) ++wander_ticks;
        int nn = f.plan.next_node();
        if (nn == 0 || nn == 2) saw_neighbor = true;
    }
    EXPECT_EQ(wander_ticks, 0) << "patrol NEVER drops into the run-tumble wander (that is play's job)";
    EXPECT_GT(route_ticks, 35) << "patrol keeps ROUTING to a neighbour every tick (never fixates/stuck)";
    EXPECT_TRUE(saw_neighbor)  << "patrol routes toward an adjacent known node (0 or 2)";
    EXPECT_GT(f.plan.last_plan_value(), 0.0f) << "plan_value = frontier coverage need > 0 (ground to cover)";
    EXPECT_LE(f.plan.last_plan_value(), 1.0f) << "coverage need is a probability in [0,1]";
    EXPECT_TRUE(f.plan.patrol_mode());
}

// (patrol) A relocating source does NOT strand the patroller: unlike the food-memory planner (which
// welds to a stale cache and dithers), patrol_mode ignores food entirely, so an eat that isn't
// repeated never becomes a phantom route — the planner keeps sweeping known ground.
TEST(PlaceGraphPlanner, PatrolModeIgnoresStaleFoodMemory) {
    Fixture f({{"patrol_mode", true},
               {"plan_value_topic", std::string("reality.cognitive.plan_value")}});
    f.run(0, 0, 0.0f, 0.1f, 0.9f);
    f.run(1, 1, 0.0f, 0.1f, 0.9f, /*hit=*/true);   // ate at node 1 once (source then relocates away)
    f.run(2, 2, 0.0f, 0.1f, 0.9f);
    f.run(3, 1, 0.0f, 0.1f, 0.9f);
    // camp at node 0 (away from the old food node 1) — the patroller must NOT be pulled back to node 1
    // as a food route; it routes on COVERAGE (least-recent), and keeps moving.
    int wander_ticks = 0;
    for (int i = 0; i < 60; ++i) { f.run(4 + i, 0, 0.0f, 0.1f, 0.9f); if (f.plan.wandering()) ++wander_ticks; }
    EXPECT_EQ(wander_ticks, 0) << "no stale-food dither: patrol keeps routing on coverage, never stuck";
    EXPECT_GT(f.plan.last_plan_value(), 0.0f) << "there is still known ground to cover";
}

// (route-execution) The planner NEVER dithers on a blocked route: when a committed hop never completes
// (the bug can't traverse it — e.g. it points through a wall) for longer than stall_factor × the bug's
// OWN typical hop time, the planner CEDES (plan_value→0) so klino/play move the bug off the stuck spot.
TEST(PlaceGraphPlanner, StallFactorCedesBlockedRoute) {
    Fixture f({{"stall_factor", 3.0}, {"plan_value_topic", std::string("reality.cognitive.plan_value")}});
    int t = 0;
    auto dwell = [&](int node, int ticks, bool eat = false) {
        for (int i = 0; i < ticks; ++i) f.run(t++, node, 0.0f, 0.1f, 0.9f, /*hit=*/(eat && i == 0));
    };
    // Build a BIDIRECTIONAL map 0↔1↔2 (both edge directions) with food at node 2, and MULTI-TICK hops so
    // the hop-duration EMA trains to ~4 ticks.
    dwell(0, 1); dwell(1, 4); dwell(2, 4, /*eat=*/true);   // edges 0→1, 1→2; food at node 2
    dwell(1, 4); dwell(0, 4);                              // edges 2→1, 1→0 (now bidirectional)
    dwell(1, 4); dwell(2, 4); dwell(1, 4);                 // more hops train the hop EMA (post-food)
    // Now HOLD at node 0, hungry, committed to route toward food node 2 (0→1→2) — the hop never completes.
    bool ceded = false; float pv_ceded = 1.0f;
    for (int i = 0; i < 40; ++i) {
        f.run(t++, 0, 0.0f, 0.1f, 0.95f);
        if (f.plan.route_ceded()) { ceded = true; pv_ceded = f.plan.last_plan_value(); }
    }
    EXPECT_TRUE(ceded) << "a hop that never completes is CEDED after the bug's own derived timeout";
    EXPECT_FLOAT_EQ(pv_ceded, 0.0f) << "ceding drops plan_value to 0 (hands authority to klino/play)";
    EXPECT_GT(f.plan.route_stall(), 0) << "the stall clock ran while the hop was blocked";
}
