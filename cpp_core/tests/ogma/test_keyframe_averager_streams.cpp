// =============================================================================
// test_keyframe_averager_streams.cpp
//   Phase v5.2 — KeyframeAverager behavioral / stream tests.
//
// Three tests cover the contract:
//   1. Rolling mean is mathematically correct (action_out scalar input).
//   2. Window trim happens correctly when more frames arrive than window_size.
//   3. proprio_token vector input averages element-wise.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/KeyframeAverager.hpp"

namespace {

ogma::ParamMap action_out_params(int window = 4) {
    return {
        {"input_topic",   std::string("action.out")},
        {"output_topic",  std::string("reality.proprio.motor_avg")},
        {"payload_kind",  std::string("action_out")},
        {"window_size",   int64_t{window}},
        {"sensor_label",  std::string("motor_avg_test")},
    };
}

ogma::ParamMap proprio_params(int window = 3) {
    return {
        {"input_topic",   std::string("reality.proprio.test")},
        {"output_topic",  std::string("reality.proprio.test_avg")},
        {"payload_kind",  std::string("proprio_token")},
        {"window_size",   int64_t{window}},
        {"sensor_label",  std::string("proprio_avg_test")},
    };
}

std::shared_ptr<ogma::ActionOut> make_action(float accel, uint64_t tick) {
    auto a = std::make_shared<ogma::ActionOut>();
    a->tick_id = tick;
    a->accel   = accel;
    a->source  = "test";
    return a;
}

std::shared_ptr<ogma::ProprioToken> make_proprio(std::vector<float> values, uint64_t tick) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id = tick;
    p->sensor  = "test";
    p->values.resize(values.size());
    for (size_t i = 0; i < values.size(); ++i) p->values[i] = values[i];
    return p;
}

struct Fixture {
    ogma::InProcessBus     bus;
    ogma::KeyframeAverager kfa;
    explicit Fixture(ogma::ParamMap const& p) {
        kfa.set_id("kfa_test");
        kfa.on_setup(&bus, p);
    }
    void run_tick(uint64_t t, ogma::MessagePtr in_msg, std::string const& topic) {
        bus.begin_tick(t);
        if (in_msg) bus.publish(topic, in_msg);
        kfa.tick(t);
        bus.end_tick();
    }
    std::shared_ptr<const ogma::ProprioToken> last_output(std::string const& topic) const {
        return std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value(topic));
    }
};

} // namespace

// =============================================================================
// 1. Rolling mean — action_out scalar input.  Window 4, send accels 1, 2, 3, 4
//    over 4 ticks → rolling mean = 1.0, 1.5, 2.0, 2.5.
// =============================================================================

TEST(KeyframeAveragerStreams, ActionOutRollingMean) {
    Fixture f(action_out_params(/*window*/4));

    f.run_tick(0, make_action(1.0f, 0), "action.out");
    auto t0 = f.last_output("reality.proprio.motor_avg");
    ASSERT_NE(t0, nullptr);
    ASSERT_EQ(t0->values.size(), 1);
    EXPECT_NEAR(t0->values[0], 1.0f, 1e-5f);

    f.run_tick(1, make_action(2.0f, 1), "action.out");
    auto t1 = f.last_output("reality.proprio.motor_avg");
    EXPECT_NEAR(t1->values[0], 1.5f, 1e-5f);

    f.run_tick(2, make_action(3.0f, 2), "action.out");
    auto t2 = f.last_output("reality.proprio.motor_avg");
    EXPECT_NEAR(t2->values[0], 2.0f, 1e-5f);

    f.run_tick(3, make_action(4.0f, 3), "action.out");
    auto t3 = f.last_output("reality.proprio.motor_avg");
    EXPECT_NEAR(t3->values[0], 2.5f, 1e-5f);

    EXPECT_EQ(f.kfa.window_fill(), 4);
    EXPECT_EQ(f.kfa.payload_dim(), 1);
    EXPECT_EQ(f.kfa.total_inputs_seen(), 4);
    EXPECT_EQ(f.kfa.total_publishes(), 4);
    EXPECT_EQ(t3->sensor, "motor_avg_test");
    EXPECT_EQ(t3->producer_id, "kfa_test");
}

// =============================================================================
// 2. Window trim — send 6 frames into a window=4.  Should drop the oldest 2;
//    final mean = mean of last 4 inputs.
// =============================================================================

TEST(KeyframeAveragerStreams, WindowTrimsOldestFrames) {
    Fixture f(action_out_params(/*window*/4));

    // Inputs: 10, 20, 30, 40, 50, 60.  After window trim, buffer = [30,40,50,60].
    // Mean = 45.
    float vals[6] = {10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f};
    for (int t = 0; t < 6; ++t) {
        f.run_tick(uint64_t(t), make_action(vals[t], uint64_t(t)), "action.out");
    }
    auto last = f.last_output("reality.proprio.motor_avg");
    ASSERT_NE(last, nullptr);
    EXPECT_NEAR(last->values[0], 45.0f, 1e-5f);
    EXPECT_EQ(f.kfa.window_fill(), 4);
    EXPECT_EQ(f.kfa.total_inputs_seen(), 6);
}

// =============================================================================
// 3. proprio_token vector input — 3-element vectors averaged element-wise
//    over window=3.  Inputs (1,2,3), (4,5,6), (7,8,9) → mean (4,5,6).
// =============================================================================

TEST(KeyframeAveragerStreams, ProprioTokenElementwiseMean) {
    Fixture f(proprio_params(/*window*/3));

    f.run_tick(0, make_proprio({1.0f, 2.0f, 3.0f}, 0), "reality.proprio.test");
    f.run_tick(1, make_proprio({4.0f, 5.0f, 6.0f}, 1), "reality.proprio.test");
    f.run_tick(2, make_proprio({7.0f, 8.0f, 9.0f}, 2), "reality.proprio.test");

    auto t = f.last_output("reality.proprio.test_avg");
    ASSERT_NE(t, nullptr);
    ASSERT_EQ(t->values.size(), 3);
    EXPECT_NEAR(t->values[0], 4.0f, 1e-5f);
    EXPECT_NEAR(t->values[1], 5.0f, 1e-5f);
    EXPECT_NEAR(t->values[2], 6.0f, 1e-5f);
    EXPECT_EQ(f.kfa.payload_dim(), 3);
}
