// =============================================================================
// test_manual_routing.cpp  --  per-primitive aux-send filter contract
// =============================================================================
//
// Manual routing puts every module's input gate into default-deny so only
// producer ids reachable through an explicit edge to the receiver get
// admitted into the receiver's handler.  This test pins the contract:
//
//   1. Auto mode (default)        — both counters tick.
//   2. set_auto_subscribe(false)  — neither counter ticks.
//   3. ConnectOp src→counter_b    — only counter_b ticks.
//   4. DisconnectOp src→counter_b — neither ticks.
//   5. set_auto_subscribe(true)   — both tick again.
//
// The publisher is a plain `bus->publish` call from the test thread with
// `Message::producer_id="host"` — same convention OgmaBrain uses for
// its bridged sensor / event publishes — so the manual-mode allowlist
// keying is exercised end-to-end.

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>
#include <typeindex>

#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"
#include "ogma/Module.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

namespace {

// Tiny test module that records every payload reaching its handler
// (after the gate passes).  Subscribes to a configurable input_topic
// and to events.* via Direct dispatch.
class CounterModule : public ogma::Module {
public:
    int counter = 0;

    std::string_view type_name() const override { return "CounterModule"; }

    std::vector<ogma::TopicSpec> input_topics() const override {
        return {
            ogma::TopicSpec{"test.signal",
                            std::type_index(typeid(ogma::EnvEvent)),
                            ogma::SubscriptionKind::Direct},
        };
    }
    std::vector<ogma::TopicSpec> output_topics() const override { return {}; }
    ogma::ParamSchema             params_schema() const override { return {}; }

    void on_setup(ogma::Bus* bus, ogma::ParamMap const&) override {
        bus_ = bus;
        sub_ids_.push_back(bus_->subscribe(
            "test.signal", ogma::SubscriptionKind::Direct,
            [this](std::string_view t, ogma::MessagePtr p){ handle(t, p); }));
    }
    void tick(uint64_t) override {}

private:
    void handle(std::string_view, ogma::MessagePtr p) {
        if (!input_allowed(p->producer_id)) return;
        ++counter;
    }
};

void register_counter_once() {
    static bool done = false;
    if (done) return;
    ogma::ModuleRegistry::instance().register_type(
        "CounterModule", []() -> ogma::ModulePtr {
            return std::make_unique<CounterModule>();
        });
    done = true;
}

ogma::GraphConfig two_counters_config(bool auto_subscribe) {
    register_counter_once();
    ogma::GraphConfig g;
    g.version = 1;
    g.runtime.auto_subscribe = auto_subscribe;
    {
        ogma::ModuleSpec m;
        m.id = "counter_a"; m.type = "CounterModule";
        g.modules.push_back(m);
    }
    {
        ogma::ModuleSpec m;
        m.id = "counter_b"; m.type = "CounterModule";
        g.modules.push_back(m);
    }
    // Boot edges intentionally empty.  The test mutates routing through
    // hot patches so each transition is observable.
    return g;
}

CounterModule* counter(ogma::OgmaInstance& inst, char const* id) {
    return dynamic_cast<CounterModule*>(inst.module(id));
}

void publish_one(ogma::Bus* bus, uint64_t tick) {
    auto e = std::make_shared<ogma::EnvEvent>();
    e->tick_id     = tick;
    e->producer_id = "host";
    e->name        = "ping";
    e->intensity   = 1.0f;
    bus->publish("test.signal", e);
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. Auto mode is the default and back-compat: both counters tick.
// ---------------------------------------------------------------------------
TEST(ManualRouting, AutoModeDeliversToAllSubscribers) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        two_counters_config(/*auto_subscribe=*/true),
        std::make_unique<ogma::InProcessBus>());
    auto* a = counter(*inst, "counter_a");
    auto* b = counter(*inst, "counter_b");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    for (uint64_t t = 0; t < 5; ++t) {
        publish_one(inst->bus(), t);
        inst->tick();
    }
    EXPECT_EQ(a->counter, 5);
    EXPECT_EQ(b->counter, 5);
}

// ---------------------------------------------------------------------------
// 2-5. Manual mode + Connect + Disconnect + toggle-back.
// ---------------------------------------------------------------------------
TEST(ManualRouting, ManualModeGatesByExplicitEdgeOnly) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        two_counters_config(/*auto_subscribe=*/false),
        std::make_unique<ogma::InProcessBus>());
    auto* a = counter(*inst, "counter_a");
    auto* b = counter(*inst, "counter_b");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);

    // 2. Manual mode, no edges yet → no delivery.
    for (uint64_t t = 0; t < 3; ++t) {
        publish_one(inst->bus(), t);
        inst->tick();
    }
    EXPECT_EQ(a->counter, 0) << "manual mode should reject delivery without explicit edge";
    EXPECT_EQ(b->counter, 0);

    // 3. ConnectOp host:test.signal → counter_b.  Now b receives, a doesn't.
    //
    // Ordering wart: enqueue_hot_patch queues; the patch applies at the
    // next tick's process_pending_patches step.  Bus publishes are
    // synchronous and fire handlers immediately.  So we tick ONCE after
    // enqueue (with no publish) so the patch lands before measuring.
    {
        ogma::GraphPatchBatch batch;
        ogma::ConnectOp c;
        c.edge.from  = "host:test.signal";
        c.edge.to    = "counter_b";
        c.edge.topic = "test.signal";
        batch.ops.push_back(c);
        inst->scheduler()->enqueue_hot_patch(std::move(batch));
    }
    inst->tick();   // apply the connect
    int a_before = a->counter, b_before = b->counter;
    for (uint64_t t = 3; t < 6; ++t) {
        publish_one(inst->bus(), t);
        inst->tick();
    }
    EXPECT_EQ(a->counter, a_before)
        << "counter_a has no edge — should still be gated";
    EXPECT_GT(b->counter, b_before)
        << "counter_b gained an edge from host — should now receive";
    int b_after_connect = b->counter;

    // 4. DisconnectOp.  b stops receiving again.
    {
        ogma::GraphPatchBatch batch;
        ogma::DisconnectOp d;
        d.from  = "host:test.signal";
        d.to    = "counter_b";
        d.topic = "test.signal";
        batch.ops.push_back(d);
        inst->scheduler()->enqueue_hot_patch(std::move(batch));
    }
    inst->tick();   // apply the disconnect
    for (uint64_t t = 6; t < 9; ++t) {
        publish_one(inst->bus(), t);
        inst->tick();
    }
    EXPECT_EQ(a->counter, a_before)
        << "counter_a still has no edge — still gated";
    EXPECT_EQ(b->counter, b_after_connect)
        << "counter_b lost its edge — should be gated again";

    // 5. Toggle back to auto mode.  Both deliveries resume.
    inst->scheduler()->set_auto_subscribe(true);
    int a_pre_auto = a->counter, b_pre_auto = b->counter;
    for (uint64_t t = 9; t < 12; ++t) {
        publish_one(inst->bus(), t);
        inst->tick();
    }
    EXPECT_GT(a->counter, a_pre_auto);
    EXPECT_GT(b->counter, b_pre_auto);
}

// ---------------------------------------------------------------------------
// 6. set_auto_subscribe(false) AT RUNTIME (not via boot config) repopulates
//    each module's allowlist from current edges_.  Toggling between modes
//    is non-destructive: counts resume after each transition.
// ---------------------------------------------------------------------------
TEST(ManualRouting, RuntimeToggleIsNonDestructive) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        two_counters_config(/*auto_subscribe=*/true),
        std::make_unique<ogma::InProcessBus>());
    auto* a = counter(*inst, "counter_a");
    auto* b = counter(*inst, "counter_b");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);

    publish_one(inst->bus(), 0); inst->tick();
    EXPECT_EQ(a->counter, 1);
    EXPECT_EQ(b->counter, 1);

    inst->scheduler()->set_auto_subscribe(false);
    publish_one(inst->bus(), 1); inst->tick();
    EXPECT_EQ(a->counter, 1) << "manual → no edges → no delivery";
    EXPECT_EQ(b->counter, 1);

    inst->scheduler()->set_auto_subscribe(true);
    publish_one(inst->bus(), 2); inst->tick();
    EXPECT_EQ(a->counter, 2) << "back to auto → resumes";
    EXPECT_EQ(b->counter, 2);
}
