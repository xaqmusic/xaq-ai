// LiftDescent (G4a): the swing lift chosen by local descent on the gait's own errors.
#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/LiftDescent.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)];
    p->producer_id = "host";
    return p;
}
// A synthetic gait: legs fl+rr and fr+rl swing alternately (period 40 ticks).  A swinging foot touches with probability
// p(u) for its pair's current lift multiplier — the world the descent must find its way down.
struct World {
    ogma::InProcessBus bus; ogma::LiftDescent ld; std::mt19937 rng{11};
    std::function<double(double)> p_front, p_rear;
    explicit World(ogma::ParamMap p) {
        p["tilt_topic"] = std::string(""); p["stall_topic"] = std::string("s.stall");
        ld.set_id("lift"); ld.on_setup(&bus, p);
    }
    double uf = 1.0, ur = 1.0;   // the handle as last published
    void step(uint64_t t, bool reset = false) {
        bus.begin_tick(t);
        const bool a = (t / 20) % 2 == 0;
        const bool sw[4] = {a, !a, !a, a};
        std::uniform_real_distribution<double> U(0.0, 1.0);
        std::vector<float> con(4);
        for (int i = 0; i < 4; ++i) {
            bus.publish(i == 0 ? "rhythm.leg.fl" : i == 1 ? "rhythm.leg.fr" : i == 2 ? "rhythm.leg.rl" : "rhythm.leg.rr",
                        pv({1.0f, 0.0f, sw[i] ? 1.0f : 0.0f, 0.25f}));
            const double pt = i < 2 ? p_front(uf) : p_rear(ur);
            con[size_t(i)] = sw[i] ? (U(rng) < pt ? 1.0f : 0.0f) : 1.0f;
        }
        bus.publish("reality.proprio.foot_contact", pv(con));
        bus.publish("s.stall", pv({0.0f}));
        if (reset) { auto e = std::make_shared<ogma::EnvEvent>(); e->producer_id = "host"; e->name = "reset"; bus.publish("events.reset", e); }
        ld.tick(t);
        if (auto h = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("handle.lift"))) { uf = h->values[0]; ur = h->values[1]; }
        bus.end_tick();
    }
};
ogma::ParamMap fast(bool enabled = true) {
    return {{"enabled", enabled}, {"hold_ticks", int64_t{80}}, {"settle_ticks", int64_t{10}}, {"min_swing", int64_t{20}},
            {"delta0", 0.25}, {"delta_min", 0.1}, {"step_max", 0.05}};
}
}  // namespace

TEST(LiftDescent, FindsAnInteriorOptimumOnEachAxis) {
    // Swing contact is least at a front lift of 1.6 and a rear lift of 2.2: the descent must settle near both.
    World w(fast());
    w.p_front = [](double u){ return std::clamp(0.05 + 0.5 * (u - 1.6) * (u - 1.6), 0.0, 1.0); };
    w.p_rear  = [](double u){ return std::clamp(0.05 + 0.5 * (u - 2.2) * (u - 2.2), 0.0, 1.0); };
    for (uint64_t t = 1; t <= 60000; ++t) w.step(t);
    EXPECT_NEAR(w.ld.u(0), 1.6, 0.25);
    EXPECT_NEAR(w.ld.u(1), 2.2, 0.25);
    EXPECT_GT(w.ld.steps(), 10);
}

TEST(LiftDescent, StaysInsideItsBoundsOnAMonotoneWorld) {
    // More lift always helps here, gently (a 0.02 change in contact rate per 0.1 of lift): the descent climbs to the
    // bound and never past it.  Longer trials, as on the body (400 ticks): a shallow slope needs the samples.
    auto p = fast(); p["hold_ticks"] = int64_t{240};
    World w(p);
    w.p_front = [](double u){ return std::clamp(0.6 - 0.2 * u, 0.0, 1.0); };
    w.p_rear  = [](double u){ return std::clamp(0.6 - 0.2 * u, 0.0, 1.0); };
    for (uint64_t t = 1; t <= 150000; ++t) w.step(t);
    EXPECT_LE(w.ld.u(0), 3.0 + 1e-9); EXPECT_GT(w.ld.u(0), 2.5);
    EXPECT_LE(w.ld.u(1), 3.0 + 1e-9); EXPECT_GT(w.ld.u(1), 2.5);
}

TEST(LiftDescent, DisabledPublishesNothing) {
    World w(fast(false));
    w.p_front = w.p_rear = [](double){ return 0.3; };
    for (uint64_t t = 1; t <= 500; ++t) w.step(t);
    EXPECT_EQ(w.bus.last_value("handle.lift"), nullptr);
    EXPECT_EQ(w.ld.trials_valid(), 0);
}

TEST(LiftDescent, AResetDiscardsTheTrial) {
    World w(fast());
    w.p_front = w.p_rear = [](double){ return 0.3; };
    for (uint64_t t = 1; t <= 80; ++t) w.step(t, t == 40);   // one trial, reset in the middle
    EXPECT_EQ(w.ld.trials_discarded(), 1);
    EXPECT_EQ(w.ld.trials_valid(), 0);
}

TEST(LiftDescent, PerSituationPointsFollowTheirOwnOptima_AndNoTopicIsOnePoint) {
    // G4b: the world alternates between two situations every 3000 ticks, announced by a RealityToken winner (0 / 1).
    // Situation 0 wants a low front lift (1.0), situation 1 a high one (2.4).  Each point must move toward its own.
    auto p = fast(); p["situation_topic"] = std::string("reality.touch.encounter"); p["situation_tau_ticks"] = 30.0;
    World w(p); int sit = 0;
    w.p_front = [&](double u){ const double opt = sit == 0 ? 1.0 : 2.4; return std::clamp(0.05 + 0.5 * (u - opt) * (u - opt), 0.0, 1.0); };
    w.p_rear  = [](double u){ return std::clamp(0.05 + 0.5 * (u - 1.5) * (u - 1.5), 0.0, 1.0); };
    for (uint64_t t = 1; t <= 150000; ++t) {
        sit = int((t / 3000) % 2);
        auto rt = std::make_shared<ogma::RealityToken>(); rt->producer_id = "epm"; rt->winner_id = sit;
        w.bus.publish("reality.touch.encounter", rt);
        w.step(t);
    }
    ASSERT_EQ(w.ld.n_situations(), 2);
    EXPECT_LT(w.ld.u_of(0, 0), w.ld.u_of(1, 0) - 0.4) << "situation 0 low, situation 1 high";
    EXPECT_NEAR(w.ld.u_of(0, 0), 1.0, 0.4);
    EXPECT_NEAR(w.ld.u_of(1, 0), 2.4, 0.5);
    // without a situation topic the module keeps one point and the token stream is ignored
    World g(fast()); g.p_front = w.p_front; g.p_rear = w.p_rear;
    for (uint64_t t = 1; t <= 2000; ++t) {
        auto rt = std::make_shared<ogma::RealityToken>(); rt->producer_id = "epm"; rt->winner_id = int(t % 2);
        g.bus.publish("reality.touch.encounter", rt); g.step(t);
    }
    EXPECT_EQ(g.ld.n_situations(), 0); EXPECT_EQ(g.ld.situation(), -1);
}
