// Generator: an EPM node's prediction of a target, graded by its own error (design note generator_and_belief_filter.md).
#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <nlohmann/json.hpp>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/Generator.hpp"

namespace {
std::shared_ptr<ogma::RealityToken> rt(int winner, std::vector<int> pruned = {}) {
    auto t = std::make_shared<ogma::RealityToken>(); t->winner_id = winner; t->pruned_ids = std::move(pruned); return t;
}
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)];
    return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::Generator g;
    explicit Fix(ogma::ParamMap p = {}) {
        p["reality_topic"] = std::string("reality.test.pose");
        if (!p.count("target_topics")) p["target_topics"] = std::vector<std::string>{"t.a", "t.b"};
        g.set_id("gen"); g.on_setup(&bus, p);
    }
    void step(uint64_t t, int winner, std::vector<float> a, std::vector<float> b, std::vector<int> pruned = {}) {
        bus.begin_tick(t);
        bus.publish("reality.test.pose", rt(winner, std::move(pruned)));
        bus.publish("t.a", pv(std::move(a)));
        bus.publish("t.b", pv(std::move(b)));
        g.tick(t); bus.end_tick();
    }
};
}  // namespace

TEST(Generator, EachNodeLearnsItsOwnTargetAndBeatsTheGlobalMean) {
    // Two situations with different targets; the nodes must decode them, the consistency statistic must be ~1
    // (scored after each node's first 20 updates, when its variance estimate means something).
    Fix f({{"warmup", int64_t{20}}});
    std::mt19937 rng(7);
    std::normal_distribution<float> n(0.0f, 0.1f);
    for (uint64_t t = 0; t < 4000; ++t) {
        const int k = (t / 20) % 2;                          // dwell 20 ticks in each
        const float mu_a = k ? 1.0f : -1.0f, mu_b = k ? 0.3f : 0.7f;
        f.step(t, k, {mu_a + n(rng)}, {mu_b + n(rng)});
    }
    EXPECT_NEAR(f.g.node_mean(0)[0], -1.0f, 0.05f); EXPECT_NEAR(f.g.node_mean(1)[0], 1.0f, 0.05f);
    EXPECT_NEAR(f.g.node_mean(0)[1],  0.7f, 0.05f); EXPECT_NEAR(f.g.node_mean(1)[1], 0.3f, 0.05f);
    EXPECT_NEAR(std::sqrt(f.g.node_var(0)[0]), 0.1f, 0.03f) << "the node's spread is the noise, not the switching";
    EXPECT_LT(f.g.mse(), 0.25 * f.g.mse_mean_baseline()) << "the nodes know far more than the average";
    EXPECT_NEAR(f.g.nis(), 1.0, 0.3) << "an honest stated variance gives a normalised innovation of ~1";
}

TEST(Generator, AHorizonPredictsTheTargetThatFollows) {
    // The target equals the winner from 5 ticks earlier: horizon 5 decodes it exactly, horizon 0 cannot.
    Fix ahead({{"horizon", int64_t{5}}, {"target_topics", std::vector<std::string>{"t.a"}}});
    Fix now({{"horizon", int64_t{0}}, {"target_topics", std::vector<std::string>{"t.a"}}});
    std::mt19937 rng(3);
    std::uniform_int_distribution<int> pick(0, 3);
    std::vector<int> w;
    for (uint64_t t = 0; t < 3000; ++t) {
        w.push_back(pick(rng));
        const float target = t >= 5 ? float(w[t - 5]) : 0.0f;
        for (Fix* f : {&ahead, &now}) {
            f->bus.begin_tick(t);
            f->bus.publish("reality.test.pose", rt(w.back()));
            f->bus.publish("t.a", pv({target}));
            f->g.tick(t); f->bus.end_tick();
        }
    }
    EXPECT_LT(ahead.g.mse(), 0.05 * ahead.g.mse_mean_baseline()) << "horizon 5 knows the target exactly";
    EXPECT_GT(now.g.mse(), 0.8 * now.g.mse_mean_baseline()) << "horizon 0 cannot see 5 ticks back";
}

TEST(Generator, ANewNodeStartsFromTheGlobalPriorAndAPrunedNodeIsDropped) {
    Fix f;
    for (uint64_t t = 0; t < 200; ++t) f.step(t, 0, {2.0f}, {4.0f});
    f.step(200, 9, {2.0f}, {4.0f});
    ASSERT_TRUE(f.g.has_node(9));
    EXPECT_NEAR(f.g.node_mean(9)[0], 2.0f, 0.1f) << "a new node predicts what the target usually is";
    f.step(201, 0, {2.0f}, {4.0f}, {9});
    EXPECT_FALSE(f.g.has_node(9));
}

TEST(Generator, NoFreshTargetMeansNoUpdateAndNoScore) {
    Fix f;
    f.step(0, 0, {1.0f}, {1.0f});
    const auto s0 = f.g.scored();
    f.bus.begin_tick(1); f.bus.publish("reality.test.pose", rt(0)); f.g.tick(1); f.bus.end_tick();
    EXPECT_EQ(f.g.scored(), s0) << "a tick with no new target reading is neither learned from nor scored";
}

TEST(Generator, ItPublishesThePredictionAndItsSpread) {
    Fix f;
    for (uint64_t t = 0; t < 100; ++t) f.step(t, 1, {0.5f}, {-0.5f});
    auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(f.bus.last_value("gen.gen"));
    ASSERT_TRUE(out);
    ASSERT_EQ(out->values.size(), 4);
    EXPECT_NEAR(out->values[0], 0.5f, 0.02f); EXPECT_NEAR(out->values[1], -0.5f, 0.02f);
    EXPECT_GE(out->values[2], 0.0f);
    auto d = f.g.diag_snapshot();
    EXPECT_EQ(d["dim"].get<int>(), 2);
    EXPECT_TRUE(d.contains("skill_vs_mean") && d.contains("nis"));
}
