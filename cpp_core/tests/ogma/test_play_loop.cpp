// PlayLoop (Cell task #33) — the third policy: GROW the shared place-map by ascending
// novelty→frontier (run-and-tumble beyond the mapped graph), NOT routing to food.
// "PlaceGraphPlanner minus traverse": same map overlay, novelty value-field, no food.
#include <gtest/gtest.h>
#include "ogma/modules/PlayLoop.hpp"
#include "ogma/InProcessBus.hpp"

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
std::shared_ptr<ogma::EnvEvent> eat() {
    auto e = std::make_shared<ogma::EnvEvent>(); e->name = "eat"; return e;   // ground-truth events.eat
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::PlayLoop play;
    Fixture(ogma::ParamMap extra = {}) {
        play.set_id("play");
        play.on_setup(&bus, extra);
    }
    // node<0 → don't publish place this tick.
    void run(uint64_t t, int node, float heading, bool do_eat = false, float tle = 0.0f) {
        bus.begin_tick(t);
        if (node >= 0) bus.publish("reality.cognitive.place", place(node, tle));
        bus.publish("reality.proprio.heading", p1(heading));
        if (do_eat) bus.publish("events.eat", eat());
        play.tick(t);
        bus.end_tick();
    }
};

constexpr float PI = 3.14159265f;

}  // namespace

// The shared map: PlayLoop learns the SAME transition edges the planner does (both are overlays
// on the place-EPM winner_id), so ground play discovers is visible to the planner it feeds.
TEST(PlayLoop, LearnsEdgeHeadingLikeThePlanner) {
    Fixture f;
    f.run(0, 0, 0.0f);
    f.run(1, 1, 0.0f);            // 0→1 heading 0
    f.run(2, 2, PI / 2);         // 1→2 heading +pi/2
    EXPECT_EQ(f.play.n_nodes(), 3);
}

// GROW directive: with a high-novelty (unmodelled) frontier node, V_play forms a gradient toward
// it and play CLIMBS — the epistemic mirror of the planner descending V→food.
TEST(PlayLoop, ClimbsTheNoveltyGradientTowardTheFrontier) {
    Fixture f;
    f.run(0, 0, 0.0f, false, /*tle=*/0.0f);   // node 0 — modelled (low novelty)
    f.run(1, 1, 0.0f, false, /*tle=*/0.0f);   // node 1 — modelled
    f.run(2, 2, PI / 2, false, /*tle=*/1.0f); // node 2 — HIGH novelty (the frontier)
    f.run(3, 0, 0.0f, false, /*tle=*/0.0f);   // back at node 0
    EXPECT_TRUE(f.play.climbing());
    EXPECT_FALSE(f.play.wandering());
    EXPECT_EQ(f.play.next_node(), 1) << "route toward the frontier (via the only out-edge)";
    EXPECT_GT(f.play.value(2), f.play.value(1)) << "novelty value propagates back from the frontier";
    EXPECT_GT(f.play.value(1), f.play.value(0));
}

// WANDER: a flat map (no novelty differential, no out-edges) → the run-and-tumble branch that
// LEAVES the mapped graph into unmapped ground (the one thing the planner structurally can't do).
TEST(PlayLoop, WandersBeyondTheFrontierWhenTheFieldIsFlat) {
    Fixture f;
    for (uint64_t t = 0; t < 5; ++t) f.run(t, 0, 0.0f, false, 0.0f);
    EXPECT_TRUE(f.play.wandering());
    EXPECT_FALSE(f.play.climbing());
    EXPECT_NEAR(f.play.last_fy(), 1.0f, 0.05f) << "run drives forward (no crawl)";
    EXPECT_NEAR(f.play.last_fx(), 0.0f, 0.05f);
}

// Epistemic value → the arbiter: play_value = V_play(frontier) normalised ∈[0,1]. Positive while a
// novel frontier is reachable. UNLIKE the planner's plan_value it is NOT zeroed while a route
// exists — play has no food route to exploit, so its value is always its map-growth potential.
TEST(PlayLoop, PublishesPlayValueNotZeroedWhileClimbing) {
    Fixture f({{"play_value_topic", std::string("reality.cognitive.play_value")}});
    f.run(0, 0, 0.0f, false, 0.0f);
    f.run(1, 1, 0.0f, false, 0.0f);
    f.run(2, 2, PI / 2, false, 1.0f);   // novel frontier at node 2
    f.run(3, 0, 0.0f, false, 0.0f);     // climbing toward it from node 0
    ASSERT_TRUE(f.play.climbing());
    auto pv = std::dynamic_pointer_cast<const ogma::ProprioToken>(
        f.bus.last_value("reality.cognitive.play_value"));
    ASSERT_NE(pv, nullptr) << "play_value published when the topic is set";
    ASSERT_EQ(pv->values.size(), 1u);
    EXPECT_GE(float(pv->values[0]), 0.0f);
    EXPECT_LE(float(pv->values[0]), 1.0f) << "normalised ∈[0,1]";
    EXPECT_GT(float(pv->values[0]), 0.0f) << "climbing toward a novel frontier → positive epistemic value (NOT zeroed)";
    EXPECT_NEAR(float(pv->values[0]), f.play.last_play_value(), 1e-4f);
}

// play_value goes to ~0 when the map is fully explored (all novelty decayed) — nothing to grow,
// so play yields. Here: revisit a single node many times with zero novelty → V_play collapses.
TEST(PlayLoop, PlayValueDecaysAsTheMapIsExhausted) {
    Fixture f({{"play_value_topic", std::string("reality.cognitive.play_value")}});
    f.run(0, 0, 0.0f, false, /*tle=*/1.0f);       // one burst of novelty
    for (uint64_t t = 1; t < 200; ++t) f.run(t, 0, 0.0f, false, /*tle=*/0.0f);  // learned → novelty EMAs to 0
    EXPECT_LT(f.play.last_play_value(), 0.2f)
        << "explored/modelled map → low epistemic value (play yields to exploit)";
}

// Default-off: no play_value_topic ⇒ nothing published (existing configs byte-identical).
TEST(PlayLoop, PlayValueOffByDefault) {
    Fixture f;   // no play_value_topic
    f.run(0, 0, 0.0f, false, 1.0f);
    EXPECT_EQ(f.bus.last_value("reality.cognitive.play_value"), nullptr)
        << "default-off: no play_value publish without the topic param";
}

// EAT-CREDIT (the honest success signal salvaged from the retired HomeokineticExploration, used
// this time): an EMA of "a real events.eat happened", crediting exploration that leads to food.
TEST(PlayLoop, EatCreditTracksRealEats) {
    Fixture f;
    for (uint64_t t = 0; t < 60; ++t) f.run(t, 0, 0.0f, /*eat=*/true, 0.0f);
    EXPECT_GT(f.play.eat_credit(), 0.1f) << "sustained eats raise the credit EMA";

    Fixture g;
    for (uint64_t t = 0; t < 60; ++t) g.run(t, 0, 0.0f, /*eat=*/false, 0.0f);
    EXPECT_NEAR(g.play.eat_credit(), 0.0f, 1e-3f) << "no eats → credit stays ~0";
}

// HK habituation ("recent = boring"): dwelling saturates the current node's habituation, which
// suppresses its LOCAL novelty term in V_play so play sweeps to fresher ground instead of camping.
TEST(PlayLoop, HabituationSuppressesTheCampedNodesValue) {
    Fixture f;
    f.run(0, 0, 0.0f, false, /*tle=*/1.0f);       // novel node 0
    float v0_fresh = f.play.value(0);
    for (uint64_t t = 1; t < 300; ++t) f.run(t, 0, 0.0f, false, /*tle=*/1.0f);  // camp (novelty held high)
    EXPECT_GT(f.play.hab_cur(), 0.8f) << "long dwell saturates habituation";
    EXPECT_GT(f.play.max_hab(), 0.8f);
    EXPECT_LT(f.play.value(0), v0_fresh)
        << "habituation suppresses the camped node's novelty-value → sweep away, don't camp";
}

// STRESS: a rich cyclic graph with revisits, transitions, tle spikes, eats, over many ticks —
// exercises run_value_iteration + climbing + geo_bearing + sub-goal commitment on a large map
// (repro harness for the headless crash — must not segfault or corrupt).
TEST(PlayLoop, StressManyNodesCyclesAndRevisits) {
    // DEFAULT place-winner path (pi_cell_size=0, like the_cell_arbiter_play): LONG single-node dwell
    // (as the place-EPM sits on one node before baking the next) THEN transitions — the map grows
    // from 1 node to many, the climb↔wander switch first activates. Repro harness for the headless
    // silent crash at ~tick 110 (2nd place node baked).
    Fixture f({{"play_value_topic", std::string("reality.cognitive.play_value")}});
    for (uint64_t t = 0; t < 1200; ++t) {
        // node 0 for the first 100 ticks (single-node dwell), then alternate 0/1/2/... every ~40 ticks
        int node = (t < 100) ? 0 : int(((t - 100) / 40) % 6);
        float heading = float((t % 8)) * 0.7853982f;      // rotate heading each tick
        bool ate = (t % 137 == 0);                         // occasional eat
        float tle = (node == 3) ? 1.0f : 0.05f;            // node 3 is the persistent frontier
        // also publish a velocity so path-integration accumulates (geo_bearing exercised)
        f.bus.begin_tick(t);
        f.bus.publish("reality.cognitive.place", place(node, tle));
        f.bus.publish("reality.proprio.heading", p1(heading));
        f.bus.publish("reality.proprio.vel_ego", p2(0.2f, 1.0f));
        if (ate) f.bus.publish("events.eat", eat());
        f.play.tick(t);
        f.bus.end_tick();
    }
    EXPECT_GT(f.play.n_nodes(), 5) << "built a multi-node map";
    EXPECT_GE(f.play.last_play_value(), 0.0f);
    EXPECT_LE(f.play.last_play_value(), 1.0f);
    SUCCEED() << "no crash over 1200 ticks on a rich cyclic graph";
}

// WANDER-BEYOND fix: when the map stops growing (climbing freshly-baked nodes is treadmilling),
// wander_stall_ticks forces the run-tumble WANDER to push PAST the frontier into unmapped ground.
TEST(PlayLoop, StallForcesWanderBeyondTheFrontier) {
    Fixture f({{"wander_stall_ticks", (int64_t)20}});
    f.run(0, 0, 0.0f, false, 0.0f);
    f.run(1, 1, 0.0f, false, 0.0f);
    f.run(2, 2, PI / 2, false, 1.0f);   // node 2 = novel frontier (a climbable gradient)
    f.run(3, 0, 0.0f);
    EXPECT_TRUE(f.play.climbing()) << "initially climbs toward the novel frontier";
    EXPECT_FALSE(f.play.forced_wander());
    for (uint64_t t = 4; t < 30; ++t) f.run(t, 0, 0.0f);   // dwell — no new nodes bake → map stalls
    EXPECT_GT(f.play.stale_explore(), 20) << "the map-stall clock rises while re-treading";
    EXPECT_TRUE(f.play.forced_wander()) << "stall → force wander even though an uphill neighbour exists";
    EXPECT_TRUE(f.play.wandering());
    EXPECT_FALSE(f.play.climbing()) << "the climb is overridden by the stall-wander";
}

// FRONTIER-DIRECTED WANDER: the memoryless run-and-tumble is diffusive (in a maze it orbits the
// spawn region). With frontier_bias>0 the wander steers AWAY from the habituation-weighted centroid
// of visited places — toward unexplored ground. Setup: dwell at spawn (hab saturates → it dominates
// the centroid), travel out along +odo_x (new low-hab cells), then FACE BACK toward spawn with a flat
// field. Legacy would drive forward (backtrack into the explored core); the frontier bias reverses it.
TEST(PlayLoop, FrontierBiasSteersAwayFromTheVisitedCentroid) {
    auto build = [](double bias) {
        auto bus  = std::make_shared<ogma::InProcessBus>();
        auto play = std::make_shared<ogma::PlayLoop>();
        play->set_id("play");
        play->on_setup(bus.get(), ogma::ParamMap{
            {"frontier_bias", bias},
            {"pi_cell_size", 20.0},          // nodes ARE odometry cells → travel makes distinct cells
            {"explore_cycle", (int64_t)5},   // tumble quickly
            {"explore_tumble_range", 0.0},   // no random spread → deterministic direction check
        });
        auto step = [bus, play](uint64_t t, float heading, float vfwd) {
            bus->begin_tick(t);
            bus->publish("reality.cognitive.place", place(0, 0.0f));     // flat novelty → wander, not climb
            bus->publish("reality.proprio.heading", p1(heading));
            bus->publish("reality.proprio.vel_ego", p2(0.0f, vfwd));     // vlat=0, vfwd
            play->tick(t);
            bus->end_tick();
        };
        uint64_t t = 0;
        for (; t < 100; ++t) step(t, 0.0f, 0.0f);            // dwell at spawn → hab saturates the spawn cell
        for (; t < 122; ++t) step(t, -PI / 2, 6.0f);         // travel +odo_x (heading -pi/2, fwd)
        for (; t < 145; ++t) step(t,  PI / 2, 0.0f);         // FACE BACK toward spawn (flat field → wander; frontier should pull outward)
        return play;
    };

    auto off = build(0.0);   // legacy: memoryless run-and-tumble
    EXPECT_TRUE(off->wandering());
    EXPECT_FALSE(off->have_frontier()) << "frontier_bias=0 → no frontier steering (Δ=0 legacy)";
    EXPECT_GT(off->last_fy(), 0.5f) << "legacy drives FORWARD along the current (backtrack) heading";

    auto on = build(1.0);    // frontier-directed
    EXPECT_TRUE(on->wandering());
    EXPECT_TRUE(on->have_frontier()) << "an established habituated core → frontier bearing is defined";
    EXPECT_NEAR(on->frontier_bearing(), -PI / 2, 0.5f) << "outward = toward +odo_x (away from spawn)";
    EXPECT_LT(on->last_fy(), 0.0f)
        << "frontier bias reverses the about-to-backtrack wander toward open ground";
}

// Default off: frontier_bias=0 → the wander is the prior memoryless run-and-tumble (have_frontier never
// engages), so existing configs are byte-identical (Δ=0). Pairs with the flat-field wander test above.
TEST(PlayLoop, FrontierBiasOffByDefault) {
    Fixture f;   // frontier_bias defaults 0
    for (uint64_t t = 0; t < 20; ++t) f.run(t, 0, 0.0f, false, 0.0f);
    EXPECT_TRUE(f.play.wandering());
    EXPECT_FALSE(f.play.have_frontier()) << "off by default → no frontier steering regardless of state";
    EXPECT_NEAR(f.play.last_fy(), 1.0f, 0.05f) << "legacy forward run preserved";
}

// Default off: wander_stall_ticks=0 → the stall-wander never fires (climb-only, prior behaviour).
TEST(PlayLoop, StallWanderOffByDefault) {
    Fixture f;  // wander_stall_ticks defaults 0
    f.run(0, 0, 0.0f, false, 0.0f);
    f.run(1, 1, 0.0f, false, 0.0f);
    f.run(2, 2, PI / 2, false, 1.0f);
    for (uint64_t t = 3; t < 60; ++t) f.run(t, 0, 0.0f);   // long dwell
    EXPECT_FALSE(f.play.forced_wander()) << "off by default → never forces wander regardless of stall";
    EXPECT_TRUE(f.play.climbing()) << "keeps climbing the uphill neighbour (prior behaviour)";
}
