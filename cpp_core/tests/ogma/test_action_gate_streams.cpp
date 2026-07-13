// =============================================================================
// test_action_gate_streams.cpp
//   Behavioral tests for the Phase 6.5.26 ActionGate module.
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ActionGate.hpp"
#include "ogma/Topics.hpp"

namespace {

ogma::ParamMap default_params() {
    return {
        {"accel_min", -4.0},
        {"accel_max",  4.0},
    };
}

struct GateFixture {
    ogma::InProcessBus bus;
    ogma::ActionGate   gate;

    explicit GateFixture(ogma::ParamMap const& p = default_params()) {
        gate.set_id("action_gate");
        gate.on_setup(&bus, p);
    }

    std::shared_ptr<const ogma::ActionOut> last_action() const {
        return std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus.last_value(ogma::topics::kActionOut));
    }

    template <typename F>
    void run_tick(uint64_t t, F&& staged) {
        bus.begin_tick(t);
        staged();
        gate.tick(t);
        bus.end_tick();
    }

    void publish_policy(uint64_t t, float weighted_accel) {
        auto pol = std::make_shared<ogma::PolicyToken>();
        pol->tick_id        = t;
        pol->producer_id    = "premotor";
        pol->weighted_accel = weighted_accel;
        pol->intent_distribution = Eigen::VectorXf::Constant(5, 0.2f);
        pol->intent_accels  = {-4.0f, -2.0f, 0.0f, 2.0f, 4.0f};
        pol->intent_names   = {"hard_left","slow_left","neutral","slow_right","hard_right"};
        bus.publish(ogma::topics::kPolicyIntent, pol);
    }

    void publish_explore(uint64_t t, bool active, float accel) {
        auto e = std::make_shared<ogma::ExplorationDirective>();
        e->tick_id         = t;
        e->active          = active;
        e->ticks_remaining = active ? 30 : 0;
        e->accel           = accel;
        bus.publish(ogma::topics::kExplorationDirective, e);
    }
};

} // namespace

// 1. Bootstrap: no inputs received → emit accel=0, source="".
TEST(ActionGateStreams, BootstrapEmitsZero) {
    GateFixture f;
    f.run_tick(0, [](){});
    auto act = f.last_action();
    ASSERT_NE(act, nullptr);
    EXPECT_FLOAT_EQ(act->accel, 0.0f);
    EXPECT_EQ(act->source, "");
}

// 2. Policy alone: gate forwards weighted_accel with source="premotor".
TEST(ActionGateStreams, PolicyAloneForwarded) {
    GateFixture f;
    f.run_tick(0, [&](){ f.publish_policy(0, 2.5f); });
    auto act = f.last_action();
    ASSERT_NE(act, nullptr);
    EXPECT_FLOAT_EQ(act->accel, 2.5f);
    EXPECT_EQ(act->source, "premotor");
    EXPECT_EQ(f.gate.policy_count(),  1);
    EXPECT_EQ(f.gate.explore_count(), 0);
}

// 3. Explore overrides policy when active.
TEST(ActionGateStreams, ExploreOverridesPolicy) {
    GateFixture f;
    f.run_tick(0, [&](){
        f.publish_policy(0, 2.5f);
        f.publish_explore(0, /*active=*/true, /*accel=*/-3.0f);
    });
    auto act = f.last_action();
    ASSERT_NE(act, nullptr);
    EXPECT_FLOAT_EQ(act->accel, -3.0f);
    EXPECT_EQ(act->source, "explore");
    EXPECT_EQ(f.gate.policy_count(),  0);
    EXPECT_EQ(f.gate.explore_count(), 1);
}

// 4. Inactive explore directive does NOT override policy.
TEST(ActionGateStreams, InactiveExploreFallsThrough) {
    GateFixture f;
    f.run_tick(0, [&](){
        f.publish_policy(0, 1.5f);
        f.publish_explore(0, /*active=*/false, /*accel=*/-3.0f);
    });
    auto act = f.last_action();
    ASSERT_NE(act, nullptr);
    EXPECT_FLOAT_EQ(act->accel, 1.5f);
    EXPECT_EQ(act->source, "premotor");
}

// 5. Output is clamped to [accel_min, accel_max].
TEST(ActionGateStreams, AccelIsClamped) {
    GateFixture f;
    f.run_tick(0, [&](){ f.publish_policy(0, /*weighted_accel=*/12.0f); });
    auto act = f.last_action();
    ASSERT_NE(act, nullptr);
    EXPECT_FLOAT_EQ(act->accel, 4.0f) << "Should clamp to accel_max";
}

// 7. Phase 6.6.D.6 — bilateral output: ActionGate publishes to a configured
//    output_topic so a single config can instantiate one gate per side
//    (action.left, action.right) for differential motor bodies.
TEST(ActionGateStreams, BilateralOutputTopicPublishesToCustomTopic) {
    auto p = default_params();
    p["output_topic"] = std::string("action.left");
    GateFixture f(p);
    f.run_tick(0, [&](){ f.publish_policy(0, 1.5f); });

    auto on_left = std::dynamic_pointer_cast<const ogma::ActionOut>(
        f.bus.last_value("action.left"));
    ASSERT_NE(on_left, nullptr);
    EXPECT_FLOAT_EQ(on_left->accel, 1.5f);

    // The legacy action.out channel must be silent for a left-only gate.
    auto on_default = f.bus.last_value(ogma::topics::kActionOut);
    EXPECT_EQ(on_default, nullptr);
}

// 8. Two gates with different output_topic don't cross-publish; each owns
//    its channel.  Used by bilateral configs (one gate per side).
TEST(ActionGateStreams, TwoBilateralGatesIndependentChannels) {
    ogma::InProcessBus bus;
    ogma::ActionGate left;
    ogma::ActionGate right;
    auto p_l = default_params();
    p_l["output_topic"] = std::string("action.left");
    auto p_r = default_params();
    p_r["output_topic"] = std::string("action.right");
    left.set_id("gate_left");
    right.set_id("gate_right");
    left.on_setup(&bus, p_l);
    right.on_setup(&bus, p_r);

    auto pol = std::make_shared<ogma::PolicyToken>();
    pol->tick_id        = 0;
    pol->weighted_accel = 2.0f;
    pol->intent_distribution = Eigen::VectorXf::Constant(5, 0.2f);
    pol->intent_accels  = {-4.0f, -2.0f, 0.0f, 2.0f, 4.0f};
    pol->intent_names   = {"a", "b", "c", "d", "e"};

    bus.begin_tick(0);
    bus.publish(ogma::topics::kPolicyIntent, pol);
    left.tick(0);
    right.tick(0);
    bus.end_tick();

    auto on_left = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value("action.left"));
    auto on_right = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus.last_value("action.right"));
    ASSERT_NE(on_left,  nullptr);
    ASSERT_NE(on_right, nullptr);
    EXPECT_FLOAT_EQ(on_left->accel,  2.0f);
    EXPECT_FLOAT_EQ(on_right->accel, 2.0f);
    EXPECT_EQ(bus.last_value(ogma::topics::kActionOut), nullptr);
}

// 6. Stale explore directive: once .active becomes false again, source flips
//    back to "premotor" without state leak.
TEST(ActionGateStreams, ExploreReleaseRestoresPolicy) {
    GateFixture f;
    // tick 0: explore active
    f.run_tick(0, [&](){
        f.publish_policy(0, 1.0f);
        f.publish_explore(0, true, -2.0f);
    });
    EXPECT_EQ(f.last_action()->source, "explore");
    // tick 1: explore goes inactive, policy still publishing
    f.run_tick(1, [&](){
        f.publish_policy(1, 1.0f);
        f.publish_explore(1, false, 0.0f);
    });
    auto act = f.last_action();
    ASSERT_NE(act, nullptr);
    EXPECT_FLOAT_EQ(act->accel, 1.0f);
    EXPECT_EQ(act->source, "premotor");
}
