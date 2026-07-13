// =============================================================================
// test_inprocess_bus.cpp  --  Unit tests for the default Bus implementation
// =============================================================================
//
// Exercises every contract documented in Bus.hpp + InProcessBus.hpp:
//   - publish() synchronously dispatches to matching Direct subscribers
//   - subscribe() with prefix patterns (trailing dot) and exact patterns
//   - unsubscribe() stops further dispatch
//   - last_value() exposes the most-recent payload, nullptr before first publish
//   - Feedback subscriptions deliver previous-tick values during begin_tick()
//   - Direct dispatch within a tick is registration-order deterministic

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"

namespace {

std::shared_ptr<ogma::NeuroState> make_neuro(uint64_t tick, float dopamine) {
    auto m = std::make_shared<ogma::NeuroState>();
    m->tick_id     = tick;
    m->producer_id = "neuro";
    m->dopamine    = dopamine;
    return m;
}

} // namespace

TEST(InProcessBus, LastValueIsNullptrBeforePublish) {
    ogma::InProcessBus bus;
    EXPECT_EQ(bus.last_value("neuro.state"), nullptr);
}

TEST(InProcessBus, LastValueReturnsLatestPublish) {
    ogma::InProcessBus bus;
    bus.publish("neuro.state", make_neuro(0, 0.30f));
    bus.publish("neuro.state", make_neuro(0, 0.50f));

    auto got = bus.last_value("neuro.state");
    ASSERT_NE(got, nullptr);
    auto neuro = std::dynamic_pointer_cast<const ogma::NeuroState>(got);
    ASSERT_NE(neuro, nullptr);
    EXPECT_FLOAT_EQ(neuro->dopamine, 0.50f);
}

TEST(InProcessBus, DirectSubscriptionFiresOnMatchingPublish) {
    ogma::InProcessBus bus;
    int call_count = 0;
    float last_dopamine = 0.0f;

    bus.subscribe("neuro.state", ogma::SubscriptionKind::Direct,
        [&](std::string_view topic, ogma::MessagePtr payload) {
            EXPECT_EQ(topic, "neuro.state");
            ++call_count;
            auto n = std::dynamic_pointer_cast<const ogma::NeuroState>(payload);
            ASSERT_NE(n, nullptr);
            last_dopamine = n->dopamine;
        });

    bus.publish("neuro.state", make_neuro(0, 0.42f));

    EXPECT_EQ(call_count, 1);
    EXPECT_FLOAT_EQ(last_dopamine, 0.42f);
}

TEST(InProcessBus, ExactPatternDoesNotMatchPrefix) {
    ogma::InProcessBus bus;
    int hits = 0;

    bus.subscribe("reality.video", ogma::SubscriptionKind::Direct,
        [&](std::string_view, ogma::MessagePtr) { ++hits; });

    bus.publish("reality.video.retinal", make_neuro(0, 0.1f));
    EXPECT_EQ(hits, 0);

    bus.publish("reality.video", make_neuro(0, 0.1f));
    EXPECT_EQ(hits, 1);
}

TEST(InProcessBus, PrefixPatternMatchesAnyDescendant) {
    ogma::InProcessBus bus;
    std::vector<std::string> seen;

    bus.subscribe("reality.", ogma::SubscriptionKind::Direct,
        [&](std::string_view topic, ogma::MessagePtr) {
            seen.emplace_back(topic);
        });

    bus.publish("reality.video.retinal",  make_neuro(0, 0));
    bus.publish("reality.audio.stft", make_neuro(0, 0));
    bus.publish("reality.proprio.imu",    make_neuro(0, 0));
    bus.publish("consensus.0",            make_neuro(0, 0));   // no match

    ASSERT_EQ(seen.size(), 3u);
    EXPECT_EQ(seen[0], "reality.video.retinal");
    EXPECT_EQ(seen[1], "reality.audio.stft");
    EXPECT_EQ(seen[2], "reality.proprio.imu");
}

TEST(InProcessBus, UnsubscribeStopsDispatch) {
    ogma::InProcessBus bus;
    int count = 0;

    auto id = bus.subscribe("neuro.state", ogma::SubscriptionKind::Direct,
        [&](std::string_view, ogma::MessagePtr) { ++count; });

    bus.publish("neuro.state", make_neuro(0, 0));
    EXPECT_EQ(count, 1);

    bus.unsubscribe(id);
    bus.publish("neuro.state", make_neuro(0, 0));
    EXPECT_EQ(count, 1);
}

TEST(InProcessBus, MultipleSubscribersDispatchInRegistrationOrder) {
    ogma::InProcessBus bus;
    std::vector<int> order;

    bus.subscribe("neuro.state", ogma::SubscriptionKind::Direct,
        [&](std::string_view, ogma::MessagePtr) { order.push_back(1); });
    bus.subscribe("neuro.state", ogma::SubscriptionKind::Direct,
        [&](std::string_view, ogma::MessagePtr) { order.push_back(2); });
    bus.subscribe("neuro.state", ogma::SubscriptionKind::Direct,
        [&](std::string_view, ogma::MessagePtr) { order.push_back(3); });

    bus.publish("neuro.state", make_neuro(0, 0));
    ASSERT_EQ(order.size(), 3u);
    EXPECT_EQ(order[0], 1);
    EXPECT_EQ(order[1], 2);
    EXPECT_EQ(order[2], 3);
}

TEST(InProcessBus, FeedbackDoesNotFireOnPublish) {
    ogma::InProcessBus bus;
    int direct_hits   = 0;
    int feedback_hits = 0;

    bus.subscribe("neuro.state", ogma::SubscriptionKind::Direct,
        [&](std::string_view, ogma::MessagePtr) { ++direct_hits; });
    bus.subscribe("neuro.state", ogma::SubscriptionKind::Feedback,
        [&](std::string_view, ogma::MessagePtr) { ++feedback_hits; });

    bus.publish("neuro.state", make_neuro(0, 0));
    EXPECT_EQ(direct_hits,   1);
    EXPECT_EQ(feedback_hits, 0);   // does NOT fire on publish
}

TEST(InProcessBus, FeedbackFiresOnNextBeginTick) {
    ogma::InProcessBus bus;
    std::vector<float> seen_dopamine;

    bus.subscribe("neuro.state", ogma::SubscriptionKind::Feedback,
        [&](std::string_view, ogma::MessagePtr p) {
            auto n = std::dynamic_pointer_cast<const ogma::NeuroState>(p);
            seen_dopamine.push_back(n ? n->dopamine : -1.f);
        });

    // Tick 0: producer publishes.  Feedback subscribers see nothing yet.
    bus.begin_tick(0);
    EXPECT_TRUE(seen_dopamine.empty());
    bus.publish("neuro.state", make_neuro(0, 0.10f));
    bus.end_tick();
    EXPECT_TRUE(seen_dopamine.empty());

    // Tick 1: at begin_tick, the feedback handler fires with tick-0 value.
    bus.begin_tick(1);
    ASSERT_EQ(seen_dopamine.size(), 1u);
    EXPECT_FLOAT_EQ(seen_dopamine[0], 0.10f);
    bus.publish("neuro.state", make_neuro(1, 0.20f));
    bus.end_tick();

    // Tick 2: feedback handler fires with tick-1 value.
    bus.begin_tick(2);
    ASSERT_EQ(seen_dopamine.size(), 2u);
    EXPECT_FLOAT_EQ(seen_dopamine[1], 0.20f);
    bus.end_tick();
}

TEST(InProcessBus, FeedbackPrefixPatternFiresPerMatchingTopic) {
    ogma::InProcessBus bus;
    std::vector<std::string> seen;

    bus.subscribe("reality.", ogma::SubscriptionKind::Feedback,
        [&](std::string_view topic, ogma::MessagePtr) {
            seen.emplace_back(topic);
        });

    // Tick 0: publish three matching topics.
    bus.begin_tick(0);
    bus.publish("reality.video.retinal",  make_neuro(0, 0));
    bus.publish("reality.audio.stft", make_neuro(0, 0));
    bus.publish("reality.proprio.imu",    make_neuro(0, 0));
    bus.publish("consensus.0",            make_neuro(0, 0));   // does not match prefix
    bus.end_tick();
    EXPECT_TRUE(seen.empty());

    // Tick 1: feedback handler fires once per matching topic.
    bus.begin_tick(1);
    EXPECT_EQ(seen.size(), 3u);
    bus.end_tick();
}

TEST(InProcessBus, SubscribedTopicsReportsRegisteredPatterns) {
    ogma::InProcessBus bus;
    bus.subscribe("neuro.state",       ogma::SubscriptionKind::Direct,   [](auto, auto) {});
    bus.subscribe("reality.",          ogma::SubscriptionKind::Direct,   [](auto, auto) {});
    bus.subscribe("reality.",          ogma::SubscriptionKind::Feedback, [](auto, auto) {});  // dedup

    auto topics = bus.subscribed_topics();
    ASSERT_EQ(topics.size(), 2u);
    EXPECT_EQ(topics[0], "neuro.state");
    EXPECT_EQ(topics[1], "reality.");
}
