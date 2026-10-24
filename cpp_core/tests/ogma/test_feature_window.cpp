// FeatureWindow: several topics summarised (mean, sd) over a sliding window as one vector.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/FeatureWindow.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)];
    return p;
}
}  // namespace

TEST(FeatureWindow, MeansAndSdsOfSelectedChannelsInOrder) {
    ogma::InProcessBus bus; ogma::FeatureWindow f; f.set_id("fw");
    f.on_setup(&bus, {{"sources", std::vector<std::string>{"a", "b:1"}}, {"window", int64_t{4}}, {"publish_every", int64_t{4}}});
    for (uint64_t t = 1; t <= 4; ++t) {
        bus.begin_tick(t);
        bus.publish("a", pv({float(t), 2.0f}));                 // a0 = 1..4, a1 = 2
        bus.publish("b", pv({9.0f, (t % 2) ? 0.0f : 1.0f}));     // b1 alternates 0/1
        f.tick(t); bus.end_tick();
    }
    auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("reality.proprio.gait_window"));
    ASSERT_TRUE(out);
    ASSERT_EQ(out->values.size(), 6);   // a: 2 means + 2 sds; b:1: 1 mean + 1 sd
    EXPECT_NEAR(out->values[0], 2.5f, 1e-5f);                 // mean a0
    EXPECT_NEAR(out->values[1], 2.0f, 1e-5f);                 // mean a1
    EXPECT_NEAR(out->values[2], std::sqrt(1.25f), 1e-4f);     // sd a0
    EXPECT_NEAR(out->values[3], 0.0f, 1e-5f);                 // sd a1
    EXPECT_NEAR(out->values[4], 0.5f, 1e-5f);                 // mean b1
    EXPECT_NEAR(out->values[5], 0.5f, 1e-5f);                 // sd b1
}

TEST(FeatureWindow, NothingUntilEverySourceIsSeenAndTheWindowIsFull) {
    ogma::InProcessBus bus; ogma::FeatureWindow f; f.set_id("fw");
    f.on_setup(&bus, {{"sources", std::vector<std::string>{"a", "b"}}, {"window", int64_t{3}}, {"publish_every", int64_t{1}}});
    bus.begin_tick(1); bus.publish("a", pv({1.0f})); f.tick(1); bus.end_tick();
    EXPECT_FALSE(bus.last_value("reality.proprio.gait_window"));
    for (uint64_t t = 2; t <= 3; ++t) { bus.begin_tick(t); bus.publish("a", pv({1.0f})); bus.publish("b", pv({1.0f})); f.tick(t); bus.end_tick(); }
    EXPECT_FALSE(bus.last_value("reality.proprio.gait_window")) << "two full ticks, window of three";
    bus.begin_tick(4); bus.publish("a", pv({1.0f})); bus.publish("b", pv({1.0f})); f.tick(4); bus.end_tick();
    EXPECT_TRUE(bus.last_value("reality.proprio.gait_window"));
}
