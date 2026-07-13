// PlaceNav — the planner reframed as a place/region NAVIGATOR (not a food-value optimizer).
// Shared place map (edge headings) + a LOOSE honest food tag (bounded; SET on eat; collapse on
// arrival-without-eat) + value-iteration route to a tagged place + reachability edge-cost.
#include <gtest/gtest.h>
#include "ogma/modules/PlaceNav.hpp"
#include "ogma/InProcessBus.hpp"

#include <cmath>
#include <memory>

namespace {
std::shared_ptr<ogma::RealityToken> place(int winner, float tle = 0.0f) {
    auto p = std::make_shared<ogma::RealityToken>(); p->winner_id = winner; p->tle = tle; return p;
}
std::shared_ptr<ogma::ProprioToken> p1(float v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::PlaceNav nav;
    Fix(ogma::ParamMap p = {}) { nav.set_id("pn"); nav.on_setup(&bus, p); }
    // one tick at `node` (winner_id; pi_cell_size defaults 0 so node == winner_id), facing `heading`.
    void step(uint64_t t, int node, float heading = 0.0f, float hunger = 0.0f,
              bool eat = false, float tle = 0.0f) {
        bus.begin_tick(t);
        bus.publish("reality.cognitive.place", place(node, tle));
        bus.publish("reality.proprio.heading", p1(heading));
        bus.publish("reality.proprio.hunger", p1(hunger));
        auto v = std::make_shared<ogma::ProprioToken>(); v->values.resize(2);
        v->values[0] = 0.0f; v->values[1] = 0.0f;   // odometry irrelevant when pi_cell_size=0
        bus.publish("reality.proprio.vel_ego", v);
        if (eat) { auto ev = std::make_shared<ogma::EnvEvent>(); ev->name = "eat"; bus.publish("events.eat", ev); }
        nav.tick(t); bus.end_tick();
    }
    void dwell(uint64_t& t, int node, int ticks, float hunger = 0.0f) {
        for (int i = 0; i < ticks; ++i) step(t++, node, 0.0f, hunger);
    }
};
}  // namespace

// (1) Route to a food-tagged place: learn a path 0->1->2, tag food at 2, then from 0 (hungry) the
// value field routes toward the tag (next hop = 1, the uphill neighbour) and plan_value > 0.
TEST(PlaceNav, RoutesToTaggedPlace) {
    Fix f({{"hab_rise", 0.0}});   // isolate routing from habituation suppression
    uint64_t t = 0;
    f.step(t++, 0);                 // start at node 0
    f.step(t++, 1);                 // 0 -> 1 (learn edge 0->1)
    f.step(t++, 2, 0.0f, 0.0f, /*eat=*/true);   // 1 -> 2, eat -> tag node 2
    EXPECT_GT(f.nav.food_tag(2), 0.5f) << "an eat SETS the current place's food tag";
    for (int i = 0; i < 5; ++i) f.step(t++, 0, 0.0f, /*hunger=*/0.9f);   // back at 0, hungry
    EXPECT_TRUE(f.nav.planning()) << "a hungry bug with a known reachable tag plans a route";
    EXPECT_EQ(f.nav.next_node(), 1) << "the uphill neighbour toward the tag is the next hop";
    EXPECT_GT(f.nav.last_plan_value(), 0.0f) << "plan_value reports the reachable food route";
}

// (2) The tag is BOUNDED (SET on eat, not accumulated) — no immortal super-attractors.
TEST(PlaceNav, FoodTagIsBoundedNotAdditive) {
    Fix f({{"hab_rise", 0.0}});
    uint64_t t = 0;
    f.step(t++, 5);
    for (int i = 0; i < 4; ++i) f.step(t++, 5, 0.0f, 0.0f, /*eat=*/true);   // eat 4x at the same node
    EXPECT_LE(f.nav.food_tag(5), 1.01f) << "repeated eats must not push the tag above 1 (SET, not +=)";
    EXPECT_GT(f.nav.food_tag(5), 0.9f);
}

// (3) R1 honest disconfirm: return HUNGRY to a tagged place, don't eat, and COMMIT to leaving
// -> the belief collapses (a hungry forage of the region failed). Camping / full departures don't.
TEST(PlaceNav, HungryFailedForageCollapsesTag) {
    Fix f({{"hab_rise", 0.0}, {"arrival_window", int64_t{10}}, {"arrival_forget", 0.1}});
    uint64_t t = 0;
    f.step(t++, 0);
    f.step(t++, 7, 0.0f, 0.0f, /*eat=*/true);   // tag node 7 (ate here)
    float tagged = f.nav.food_tag(7);
    EXPECT_GT(tagged, 0.5f);
    // leave, then RETURN hungry with no eat -> opens the forage window on node 7...
    f.step(t++, 0, 0.0f, /*hunger=*/0.9f);
    f.dwell(t, 0, 4, 0.9f);
    f.step(t++, 7, 0.0f, /*hunger=*/0.9f);      // return-visit (transition in), no eat
    // ...then COMMIT to leaving while hungry (away > arrival_window) -> disconfirm.
    f.step(t++, 0, 0.0f, 0.9f);
    f.dwell(t, 0, 15, 0.9f);
    EXPECT_LT(f.nav.food_tag(7), tagged * 0.5f)
        << "a hungry forage of the region that failed + committed departure collapses the belief (food_tag="
        << f.nav.food_tag(7) << ")";
}

// (3b) camping a PRODUCTIVE region (periodic eats) keeps the tag alive across the forage cycle —
// a slow closer weaving in and out must not lose the memory it is actively eating from.
TEST(PlaceNav, ProductiveCampingKeepsTag) {
    Fix f({{"hab_rise", 0.0}, {"arrival_window", int64_t{10}}, {"arrival_forget", 0.1}});
    uint64_t t = 0;
    f.step(t++, 0);
    f.step(t++, 7, 0.0f, 0.9f, /*eat=*/true);   // tag + eat at 7 while hungry
    // weave in/out of 7 but keep eating periodically (a slow close) -> tag must survive.
    for (int cyc = 0; cyc < 3; ++cyc) {
        f.step(t++, 8, 0.0f, 0.9f);             // brief excursion to a neighbour
        f.dwell(t, 8, 6, 0.9f);
        f.step(t++, 7, 0.0f, 0.9f, /*eat=*/true);   // back to 7 and eat (re-arm)
    }
    EXPECT_GT(f.nav.food_tag(7), 0.5f) << "periodic eats must keep a productive region's tag alive";
}

// (4) ablation=no_forget: the stale tag PERSISTS (the R1 baseline that should regress on relocation).
TEST(PlaceNav, NoForgetAblationKeepsStaleTag) {
    Fix f({{"hab_rise", 0.0}, {"arrival_window", int64_t{10}}, {"ablation", std::string("no_forget")}});
    uint64_t t = 0;
    f.step(t++, 0);
    f.step(t++, 7, 0.0f, 0.0f, /*eat=*/true);
    f.step(t++, 0, 0.0f, 0.9f);
    f.dwell(t, 0, 4, 0.9f);
    f.step(t++, 7, 0.0f, 0.9f);
    f.step(t++, 0, 0.0f, 0.9f);
    f.dwell(t, 0, 15, 0.9f);
    EXPECT_GT(f.nav.food_tag(7), 0.5f) << "no_forget must keep the stale tag (the deadlock baseline)";
}

// (5) Honest plan_value: no tag anywhere -> no food route -> plan_value == 0 (the arbiter isn't lied to).
TEST(PlaceNav, PlanValueZeroWithoutTag) {
    Fix f;
    uint64_t t = 0;
    f.step(t++, 0);
    for (int i = 0; i < 10; ++i) f.step(t++, (i % 3), 0.0f, /*hunger=*/0.9f);   // wander, never eat
    EXPECT_FLOAT_EQ(f.nav.last_plan_value(), 0.0f) << "no fresh tag -> plan_value must be 0";
    EXPECT_FALSE(f.nav.planning());
}

// (6) Reachability: a committed hop that never transitions (a wall) accrues block_cost and cedes.
TEST(PlaceNav, StalledHopAccruesBlockCostAndCedes) {
    Fix f({{"hab_rise", 0.0}, {"stall_factor", 2.0}});
    uint64_t t = 0;
    // learn a path 0->1->2 with a couple of quick hops (small hop_ema), tag node 2.
    f.step(t++, 0); f.step(t++, 1); f.step(t++, 2, 0.0f, 0.0f, /*eat=*/true);
    f.step(t++, 1); f.step(t++, 0);                 // more transitions -> small hop_ema
    // now sit at node 0 hungry: it commits a hop toward the tag but NEVER transitions (wall).
    bool ceded_seen = false;
    for (int i = 0; i < 120; ++i) {
        f.step(t++, 0, 0.0f, 0.9f);
        if (f.nav.route_ceded()) ceded_seen = true;
    }
    EXPECT_TRUE(ceded_seen) << "a hop that never completes must eventually CEDE (route blocked)";
    EXPECT_GT(f.nav.block_cost(0, 1), 0.0f) << "the blocked hop must accrue a reachability cost";
}

// (7) Output is a unit bearing published on the nav topic.
TEST(PlaceNav, PublishesBearing) {
    float fx = 2.0f;
    Fix f;
    f.bus.subscribe("percept.nav_bearing", ogma::SubscriptionKind::Direct,
        [&](std::string_view, ogma::MessagePtr m){
            auto pt = std::dynamic_pointer_cast<const ogma::ProprioToken>(m);
            if (pt && pt->values.size() >= 2) fx = float(pt->values[0]);
        });
    uint64_t t = 0;
    for (int i = 0; i < 5; ++i) f.step(t++, i, 0.0f, 0.2f);
    EXPECT_GE(fx, -1.01f); EXPECT_LE(fx, 1.01f) << "bearing fx must be a unit-vector component";
}
