// StanceFeatures: the careful stepper's stance as one conditioned vector (learned stepper plan Phase 3, 2026-11-15).
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/StanceFeatures.hpp"
#include "stepper_fixture.hpp"
namespace {
using ogma::StanceFeatures; using namespace stepper_test;
struct Fix {
    ogma::InProcessBus bus; StanceFeatures m; std::vector<float> feat, obs; int n_feat = 0;
    explicit Fix(ogma::ParamMap p = {}) {
        m.set_id("stance_features"); m.on_setup(&bus, p);
        bus.subscribe("reality.proprio.stance_features", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); feat.assign(t->values.data(), t->values.data() + t->values.size()); ++n_feat; });
        bus.subscribe("reality.proprio.stance_obs", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); obs.assign(t->values.data(), t->values.data() + t->values.size()); });
    }
    // one tick: the leg model, a pose (engage, then u per leg: knee from `knee`, the stepping leg's knee raised by `lift`), loads, contacts, up
    void run(uint64_t t, float engage, float knee = 0.3f, int lifted = -1, std::vector<float> contact = {1, 1, 1, 1}, std::vector<float> up = {0, 1, 0}) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.leg_model", pv(leg_model()));
        std::vector<float> pose{engage}; for (int l = 0; l < 4; ++l) { pose.push_back(0.0f); pose.push_back(l == lifted ? -0.4f : 0.0f); pose.push_back(knee); }
        bus.publish("pose.targets", pv(pose)); bus.publish("reality.proprio.foot_load", pv({0.25f, 0.25f, 0.25f, 0.25f}));
        bus.publish("reality.proprio.foot_contact", pv(contact)); bus.publish("reality.proprio.up_body", pv(up)); bus.publish("terrain.patch", pv(flat_patch(-0.09f)));
        m.tick(t); bus.end_tick();
    }
};
}

TEST(StanceFeatures, NothingWhileDisengaged_ThenAfterTheWarmupBothTopics) {
    Fix f;
    for (uint64_t t = 0; t < 300; ++t) f.run(t, 0.0f);
    EXPECT_EQ(f.n_feat, 0); EXPECT_EQ(f.m.samples(), 0) << "the stance is the stepper's only while it is engaged";
    for (uint64_t t = 300; t < 600; ++t) f.run(t, 1.0f, 0.3f + 0.1f * float(t % 7) / 7.0f);
    EXPECT_EQ(f.m.samples(), 300); EXPECT_EQ(f.n_feat, 100) << "the first 200 samples are the warm-up";
    ASSERT_EQ(int(f.feat.size()), StanceFeatures::kRaw + 1); ASSERT_EQ(int(f.obs.size()), StanceFeatures::kRaw);
    EXPECT_FLOAT_EQ(f.feat.back(), float(std::sqrt(42.0))) << "the constant component keeps the magnitude through the L2 normalisation";
    for (int i = 0; i < StanceFeatures::kRaw; ++i) { EXPECT_LE(std::fabs(f.obs[size_t(i)]), 4.0f); EXPECT_FLOAT_EQ(f.obs[size_t(i)], f.feat[size_t(i)]); }
}

TEST(StanceFeatures, TheRawStanceReadsEachLegsRoomLiftAndTheBodysPitch) {
    Fix f; f.run(0, 1.0f, 0.3f, /*lifted=*/2, {1, 1, 0, 1}, {0.0f, 0.98f, 0.2f});
    Eigen::VectorXd x; ASSERT_TRUE(f.m.raw(x));
    EXPECT_NEAR(x[0 * 10 + 1], 0.3, 1e-6) << "leg 0's knee to straight is its knee command";
    EXPECT_NEAR(x[0 * 10 + 2], 0.7, 1e-6) << "and to fold the rest of its range";
    EXPECT_GT(x[2 * 10 + 5], 0.01) << "the rear-left toe raised by its hip2: it stands out above the others";
    EXPECT_EQ(x[2 * 10 + 7], 0.0); EXPECT_EQ(x[0 * 10 + 7], 1.0);
    EXPECT_NEAR(x[0 * 10 + 6], 0.25, 1e-6);
    EXPECT_NEAR(x[40], std::atan2(0.2, 0.98), 1e-5) << "pitch, nose-up positive"; EXPECT_NEAR(x[41], 0.0, 1e-9);
    EXPECT_NEAR(x[8], 0.0, 1e-6) << "flat remembered ground: no toe above the others";
}

TEST(StanceFeatures, TheFlickerTestFlipsFlagsWhereTheyAreFeaturised) {
    Fix off; for (uint64_t t = 0; t < 400; ++t) off.run(t, 1.0f);
    EXPECT_EQ(off.m.diag_snapshot()["flips"].get<int64_t>(), 0);
    Fix on({{"flicker_p", ogma::ParamValue{0.25}}}); for (uint64_t t = 0; t < 400; ++t) on.run(t, 1.0f);
    const auto flips = on.m.diag_snapshot()["flips"].get<int64_t>();
    EXPECT_GT(flips, 300); EXPECT_LT(flips, 500) << "a quarter of 1600 flag-ticks";
    EXPECT_GT(on.m.var()[7], 0.1) << "the contact feature now varies";
    EXPECT_LT(off.m.var()[7], 1e-9) << "without the test it is constant";
}

TEST(StanceFeatures, ASnapshotRestoresTheStatistics) {
    Fix a; for (uint64_t t = 0; t < 500; ++t) a.run(t, 1.0f, 0.2f + 0.3f * float(t % 11) / 11.0f);
    Fix b; b.m.restore_state(a.m.snapshot_state());
    EXPECT_EQ(b.m.samples(), a.m.samples());
    for (int i = 0; i < StanceFeatures::kRaw; ++i) { EXPECT_DOUBLE_EQ(b.m.mean()[i], a.m.mean()[i]); EXPECT_DOUBLE_EQ(b.m.var()[i], a.m.var()[i]); }
    a.run(500, 1.0f, 0.4f); b.run(500, 1.0f, 0.4f);
    ASSERT_EQ(a.feat.size(), b.feat.size()); for (size_t i = 0; i < a.feat.size(); ++i) EXPECT_FLOAT_EQ(a.feat[i], b.feat[i]);
}

TEST(StanceFeatures, WithNoFloorTheStatisticsSettleAndHoldWhereAFloorKeepsChasingTheRegime) {
    // two regimes in alternating blocks of 400 samples (the carried runs' rotating benches): knee 0.2 or 0.6
    auto drift = [](int64_t n_max) {
        Fix f({{"n_max", ogma::ParamValue{n_max}}}); std::vector<double> at_block_end;
        for (uint64_t t = 0; t < 8000; ++t) { f.run(t, 1.0f, (t / 400) % 2 ? 0.6f : 0.2f); if (t % 400 == 399) at_block_end.push_back(f.m.mean()[1]); }
        return std::fabs(at_block_end.back() - at_block_end[at_block_end.size() - 2]);   // the mean's move over the last block
    };
    const double floored = drift(500), settled = drift(0);
    EXPECT_GT(floored, 0.1) << "a 500-sample memory follows each block";
    EXPECT_LT(settled, 0.015) << "1 / n over 8 000 samples moves by about 400 / 8000 of the regimes' gap";
    EXPECT_LT(settled, floored / 5.0);
}
