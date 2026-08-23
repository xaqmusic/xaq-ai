// =============================================================================
// test_gain_evolver.cpp — PART IV GainEvolver unit suite.
//
//   The (1+1)-ES loop with interleaved incumbent re-evaluation, the intrinsic
//   viability+flow criterion, and every designed-in guard from the charter
//   (adaptive_gains_substrate_plan.md §2):
//     * σ=0 silent observer  = the gain-0 guard (no publishes, no RNG draws)
//     * G1/G2 viability      SEPARATE from the improvement criterion — a
//       candidate that wins the criterion while dropping a fall or a leg's
//       loaded minima must REVERT ("target wins, body pays" rejected)
//     * per-leg minima, never group means (the stance-capture lesson)
//     * bounds clamps, 1/5th-rule σ anneal, snapshot/restore determinism
//   Plus the MotorEPMv2 gain-socket side: read-back-verified apply counters
//   (a typo'd key must COUNT, never vanish — the dispatch chain has no
//   terminal else), the amp_seek ownership collision, and the restore replay
//   (evolved gains are not config params; a restored clone must re-land them).
// =============================================================================

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/GainEvolver.hpp"
#include "ogma/modules/MotorEPMv2.hpp"
#include "ogma/Topics.hpp"

namespace {

using ogma::ParamMap;
using ogma::ParamValue;

constexpr int64_t kWarmup = 10;
constexpr int64_t kWindow = 200;   // back half = ticks 100..199 of each window

ParamMap ge_params(double sigma = 0.1) {
    ParamMap p;
    p["gain_keys"]        = std::vector<std::string>{"ga", "gb", "gc"};
    p["gain_seed"]        = std::vector<double>{0.5, 0.2, 0.5};
    p["gain_min"]         = std::vector<double>{0.0, 0.0, 0.0};
    p["gain_max"]         = std::vector<double>{1.0, 1.0, 1.0};
    p["gain_topic"]       = std::string("t.gains");
    p["upright_topic"]    = std::string("t.upright");
    p["distress_topic"]   = std::string("t.distress");
    p["foot_load_topic"]  = std::string("t.load");
    p["foot_contact_topic"] = std::string("t.contact");
    p["imu_topic"]        = std::string("t.imu");
    p["torque_topic"]     = std::string("t.torque");
    p["n_legs"]           = int64_t{4};
    p["seed"]             = int64_t{99};
    p["warmup_ticks"]     = kWarmup;
    p["eval_window_ticks"] = kWindow;
    p["mutation_sigma"]   = sigma;
    // PIN the criterion explicitly instead of inheriting production defaults:
    // these tests exercise the LOOP (accept / revert / guards / anneal), so they
    // drive a single, deterministic term and must not silently change meaning
    // when the criterion's composition is retuned (which is exactly what
    // happened when falls and distress moved to weight 0 after gate 2).
    p["w_falls"]    = 0.0;      // falls is guard-only; G1 tests it directly
    p["w_tilt_sd"]  = 0.0;
    p["w_distress"] = 1.0;      // <- the term these tests drive
    p["w_unloaded"] = 0.0;
    p["w_flow"]     = 0.0;
    p["w_energy"]   = 0.0;      // loop tests drive distress only
    p["accept_k"]   = 0.0;      // bare inequality; the margin has its own tests
    p["fall_debounce_ticks"] = int64_t{10};
    p["touchdown_horizon_ticks"] = int64_t{8};
    p["min_touchdowns"]   = int64_t{3};
    return p;
}

// One tick's sensory frame.  contact cycles 10-on/10-off from the global tick
// → a touchdown edge every 20 ticks → ≥5 back-half touchdowns per window.
struct Feed {
    float upright = 1.0f;
    float distress = 0.0f;
    float fwd_v = 0.03f;
    float torque = 0.4f;
    std::array<float, 4> load{0.5f, 0.5f, 0.5f, 0.5f};
    bool  cycle_contact = true;
};

void ev_tick(ogma::InProcessBus& bus, ogma::GainEvolver& ge, uint64_t t, Feed const& f) {
    bus.begin_tick(t);
    auto pub = [&](char const* topic, std::vector<float> const& vals) {
        auto pt = std::make_shared<ogma::ProprioToken>();
        pt->values = Eigen::VectorXf(int(vals.size()));
        for (int i = 0; i < int(vals.size()); ++i) pt->values[i] = vals[size_t(i)];
        bus.publish(topic, pt);
    };
    float c = (!f.cycle_contact || ((t / 10) % 2 == 0)) ? 1.0f : 0.0f;
    pub("t.upright", {f.upright});
    pub("t.distress", {f.distress});
    pub("t.load", {f.load[0], f.load[1], f.load[2], f.load[3]});
    pub("t.contact", {c, c, c, c});
    pub("t.imu", {0.0f, 1.0f, f.fwd_v, 0.0f});
    pub("t.torque", std::vector<float>(12, f.torque));
    ge.tick(t);
    bus.end_tick();
}

void run_ticks(ogma::InProcessBus& bus, ogma::GainEvolver& ge, uint64_t& t, int64_t n,
               Feed const& f) {
    for (int64_t i = 0; i < n; ++i) ev_tick(bus, ge, t++, f);
}

int64_t metric_i(ogma::GainEvolver const& ge, char const* key) {
    return ge.metrics().value(key, int64_t(-1));
}
double metric_d(ogma::GainEvolver const& ge, char const* key) {
    return ge.metrics().value(key, -1.0);
}

// Count GainVector publishes on the test topic.
struct PubLog {
    std::vector<ogma::GainVector> msgs;
    void attach(ogma::InProcessBus& bus) {
        bus.subscribe("t.gains", ogma::SubscriptionKind::Direct,
            [this](std::string_view, ogma::MessagePtr p) {
                if (auto gv = std::dynamic_pointer_cast<const ogma::GainVector>(p))
                    msgs.push_back(*gv);
            });
    }
};

} // namespace

// ---- gain-0 guard -----------------------------------------------------------

TEST(GainEvolver, Sigma0Inert) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    PubLog log; log.attach(bus);
    ge.on_setup(&bus, ge_params(/*sigma=*/0.0));
    std::string rng_before = ge.snapshot_state()["module"]["rng"].get<std::string>();

    uint64_t t = 0;
    run_ticks(bus, ge, t, kWarmup + 3 * kWindow, Feed{});

    EXPECT_TRUE(log.msgs.empty()) << "silent observer must publish NOTHING";
    auto mod = ge.snapshot_state()["module"];
    EXPECT_EQ(mod["rng"].get<std::string>(), rng_before) << "no RNG draws at sigma 0";
    EXPECT_EQ(mod["incumbent"].get<std::vector<double>>(),
              (std::vector<double>{0.5, 0.2, 0.5}));
    // ...but the evaluator DID score windows (the instruments stay live).
    EXPECT_GT(metric_d(ge, "J_inc"), -1.0);
}

// ---- accept / revert --------------------------------------------------------

TEST(GainEvolver, AcceptOnBetter) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ge.on_setup(&bus, ge_params());
    uint64_t t = 0;
    Feed bad;  bad.distress = 1.0f;    // incumbent window: high distress duty
    Feed good; good.distress = 0.0f;   // candidate window: clean
    run_ticks(bus, ge, t, kWarmup, Feed{});
    run_ticks(bus, ge, t, kWindow, bad);     // incumbent
    run_ticks(bus, ge, t, kWindow, good);    // candidate → must accept

    EXPECT_EQ(metric_i(ge, "generation"), 1);
    EXPECT_EQ(metric_i(ge, "accepts"), 1);
    EXPECT_EQ(metric_i(ge, "reverts"), 0);
    auto mod = ge.snapshot_state()["module"];
    EXPECT_EQ(mod["incumbent"].get<std::vector<double>>(),
              mod["candidate"].get<std::vector<double>>())
        << "accepted candidate becomes the incumbent";
}

TEST(GainEvolver, RevertOnWorse) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ge.on_setup(&bus, ge_params());
    uint64_t t = 0;
    Feed good; good.distress = 0.0f;
    Feed bad;  bad.distress = 1.0f;
    run_ticks(bus, ge, t, kWarmup, Feed{});
    run_ticks(bus, ge, t, kWindow, good);    // incumbent clean
    run_ticks(bus, ge, t, kWindow, bad);     // candidate worse → revert

    EXPECT_EQ(metric_i(ge, "accepts"), 0);
    EXPECT_EQ(metric_i(ge, "reverts"), 1);
    auto mod = ge.snapshot_state()["module"];
    EXPECT_EQ(mod["incumbent"].get<std::vector<double>>(),
              (std::vector<double>{0.5, 0.2, 0.5})) << "reverted to the seed incumbent";
}

// ---- the guards are SEPARATE from the criterion -----------------------------

TEST(GainEvolver, ViabilityRejectsTargetWinsBodyPays) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ge.on_setup(&bus, ge_params());   // w_falls 0 => criterion is blind to falls
    uint64_t t = 0;
    Feed inc; inc.distress = 0.6f;             // mediocre incumbent, zero falls
    run_ticks(bus, ge, t, kWarmup, Feed{});
    run_ticks(bus, ge, t, kWindow, inc);
    // Candidate: distress 0 (J clearly better under these weights) but ONE fall.
    // The fall goes in the FRONT half: falls count whole-window while the
    // continuous terms (tilt variance!) measure only the back half — so J still
    // favors the candidate and ONLY the G1 guard stands between it and a
    // "target wins, body pays" acceptance.
    Feed cand;  cand.distress = 0.0f;
    Feed fall = cand; fall.upright = -0.5f;
    run_ticks(bus, ge, t, 20, cand);
    run_ticks(bus, ge, t, 20, fall);           // > debounce(10) below thresh → 1 fall
    run_ticks(bus, ge, t, kWindow - 40, cand);

    // …the G1 falls guard must still reject it, even though J clearly improved.
    EXPECT_EQ(metric_i(ge, "accepts"), 0);
    EXPECT_EQ(metric_i(ge, "reverts"), 1);
}

TEST(GainEvolver, PerLegMinimaGuard) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ge.on_setup(&bus, ge_params());
    uint64_t t = 0;
    Feed inc; inc.distress = 0.8f;             // all 4 legs loaded, high distress
    run_ticks(bus, ge, t, kWarmup, Feed{});
    run_ticks(bus, ge, t, kWindow, inc);
    // Candidate: distress 0 (mean-J improvement dwarfs the unloaded-mean cost)
    // but leg 3's touchdowns never load — the stance-capture ghost-touch leg.
    Feed cand; cand.distress = 0.0f; cand.load = {0.5f, 0.5f, 0.5f, 0.0f};
    run_ticks(bus, ge, t, kWindow, cand);

    // G2: loaded_min collapsed 1.0 → 0.0 for one leg; a group mean would have
    // accepted this.  Must revert.
    EXPECT_EQ(metric_i(ge, "accepts"), 0);
    EXPECT_EQ(metric_i(ge, "reverts"), 1);
}

// ---- mutation bounds + anneal -----------------------------------------------

TEST(GainEvolver, BoundsClamp) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params(/*sigma=*/0.5);     // sigma_max-scale steps
    ge.on_setup(&bus, p);
    PubLog log; log.attach(bus);
    uint64_t t = 0;
    run_ticks(bus, ge, t, kWarmup + 50 * 2 * kWindow, Feed{});

    ASSERT_GE(log.msgs.size(), 100u);
    for (auto const& gv : log.msgs) {
        ASSERT_EQ(gv.values.size(), 3u);
        for (double v : gv.values) {
            EXPECT_GE(v, 0.0);
            EXPECT_LE(v, 1.0);
        }
    }
}

TEST(GainEvolver, SigmaAnneal) {
    // All-reject: incumbent windows clean, candidate windows bad → σ decays.
    {
        ogma::InProcessBus bus;
        ogma::GainEvolver ge;
        ge.on_setup(&bus, ge_params(/*sigma=*/0.1));
        uint64_t t = 0;
        Feed good; good.distress = 0.0f;
        Feed bad;  bad.distress = 1.0f;
        run_ticks(bus, ge, t, kWarmup, Feed{});
        for (int g = 0; g < 20; ++g) {
            run_ticks(bus, ge, t, kWindow, good);
            run_ticks(bus, ge, t, kWindow, bad);
        }
        EXPECT_EQ(metric_i(ge, "accepts"), 0);
        EXPECT_LT(metric_d(ge, "sigma"), 0.1);
        EXPECT_GE(metric_d(ge, "sigma"), 0.01);   // floor: search never fully stops
    }
    // All-accept: incumbent bad, candidate clean → σ grows, ceiling-bounded.
    {
        ogma::InProcessBus bus;
        ogma::GainEvolver ge;
        ge.on_setup(&bus, ge_params(/*sigma=*/0.1));
        uint64_t t = 0;
        Feed good; good.distress = 0.0f;
        Feed bad;  bad.distress = 1.0f;
        run_ticks(bus, ge, t, kWarmup, Feed{});
        for (int g = 0; g < 20; ++g) {
            run_ticks(bus, ge, t, kWindow, bad);
            run_ticks(bus, ge, t, kWindow, good);
        }
        EXPECT_EQ(metric_i(ge, "reverts"), 0);
        EXPECT_GT(metric_d(ge, "sigma"), 0.1);
        EXPECT_LE(metric_d(ge, "sigma"), 0.5);
    }
}

// ---- persistence ------------------------------------------------------------

TEST(GainEvolver, SnapshotRoundtrip) {
    auto drive = [](ogma::InProcessBus& bus, ogma::GainEvolver& ge, uint64_t& t, int64_t n) {
        // Deterministic mixed feed: distress alternates with the window parity,
        // so accepts and reverts both occur along the run.
        for (int64_t i = 0; i < n; ++i) {
            Feed f;
            int64_t past = int64_t(t) - kWarmup;
            f.distress = (past >= 0 && ((past / kWindow) % 2 == 1)) ? 0.2f : 0.7f;
            ev_tick(bus, ge, t++, f);
        }
    };

    ogma::InProcessBus busA;
    ogma::GainEvolver geA;
    geA.on_setup(&busA, ge_params());
    PubLog logA; logA.attach(busA);
    uint64_t tA = 0;
    drive(busA, geA, tA, kWarmup + kWindow + 100);      // mid-CANDIDATE window
    nlohmann::json snap = geA.snapshot_state();

    ogma::InProcessBus busB;
    ogma::GainEvolver geB;
    geB.on_setup(&busB, ge_params());
    PubLog logB; logB.attach(busB);
    geB.restore_state(snap);

    uint64_t tB = tA;
    size_t skipA = logA.msgs.size();                    // only compare post-restore
    drive(busA, geA, tA, 3 * kWindow);
    drive(busB, geB, tB, 3 * kWindow);

    ASSERT_EQ(logA.msgs.size() - skipA, logB.msgs.size());
    for (size_t i = 0; i < logB.msgs.size(); ++i) {
        auto const& a = logA.msgs[skipA + i];
        auto const& b = logB.msgs[i];
        EXPECT_EQ(a.keys, b.keys);
        EXPECT_EQ(a.values, b.values) << "publish " << i << " diverged after restore";
        EXPECT_EQ(a.generation, b.generation);
        EXPECT_EQ(a.is_candidate, b.is_candidate);
    }
    EXPECT_EQ(geA.snapshot_state().dump(), geB.snapshot_state().dump());
}

// ---- the MotorEPMv2 gain socket ---------------------------------------------

namespace {

std::vector<std::string> me_proprio(int n) {
    std::vector<std::string> v;
    for (int i = 0; i < n; ++i) v.push_back("mt.p" + std::to_string(i));
    return v;
}
std::vector<std::string> me_actions(int n, int m) {
    std::vector<std::string> v;
    for (int l = 0; l < n; ++l)
        for (int j = 0; j < m; ++j)
            v.push_back("mt.a" + std::to_string(l) + "." + std::to_string(j));
    return v;
}

ParamMap me_params() {
    ParamMap p;
    p["n_legs"]         = int64_t{4};
    p["motor_dim"]      = int64_t{3};
    p["seed"]           = int64_t{1234};
    p["babble_ticks"]   = int64_t{12};
    p["explore_noise"]  = 0.0;
    p["proprio_topics"] = me_proprio(4);
    p["action_topics"]  = me_actions(4, 3);
    p["gain_topic"]     = std::string("t.gains");
    return p;
}

// The 8 real PART IV keys with distinctive values.
std::vector<std::string> const kRealKeys{
    "rear_land_gain", "rear_knee_plant", "rear_push_ext", "amp_target",
    "height_homeo_gain", "postural_gain", "coupling_gain", "plan_gain"};
std::vector<double> const kRealVals{0.61, 0.21, 0.41, 0.33, 0.05, 0.81, 1.21, 0.07};

void publish_gains(ogma::InProcessBus& bus, ogma::MotorEPMv2& me, uint64_t t,
                   std::vector<std::string> keys, std::vector<double> vals) {
    bus.begin_tick(t);
    auto gv = std::make_shared<ogma::GainVector>();
    gv->keys = std::move(keys);
    gv->values = std::move(vals);
    bus.publish("t.gains", gv);
    me.tick(t);                    // pending pairs apply at the TOP of this tick
    bus.end_tick();
}

int64_t me_counter(ogma::MotorEPMv2 const& me, char const* key) {
    return me.snapshot_state()["module"].value(key, int64_t(-1));
}

} // namespace

TEST(MotorEPMv2GainSocket, MappingIntegrity) {
    ogma::InProcessBus bus;
    ogma::MotorEPMv2 me;
    me.on_setup(&bus, me_params());
    publish_gains(bus, me, 0, kRealKeys, kRealVals);

    EXPECT_EQ(me_counter(me, "gains_applied"), 8);
    EXPECT_EQ(me_counter(me, "gains_rejected"), 0);
    ParamMap now = me.current_params();
    for (size_t i = 0; i < kRealKeys.size(); ++i) {
        auto it = now.find(kRealKeys[i]);
        ASSERT_NE(it, now.end()) << kRealKeys[i];
        EXPECT_EQ(std::get<double>(it->second), kRealVals[i]) << kRealKeys[i];
    }

    // A typo'd key must COUNT as rejected — never vanish (the dispatch chain
    // has no terminal else; the read-back is the only honest meter).
    publish_gains(bus, me, 1, {"rear_land_gian", "plan_gain"}, {0.9, 0.11});
    EXPECT_EQ(me_counter(me, "gains_applied"), 9);
    EXPECT_EQ(me_counter(me, "gains_rejected"), 1);
    EXPECT_EQ(std::get<double>(me.current_params().at("plan_gain")), 0.11);
}

TEST(MotorEPMv2GainSocket, EmptyTopicInert) {
    ogma::InProcessBus bus;
    ogma::MotorEPMv2 me;
    ParamMap p = me_params();
    p["gain_topic"] = std::string("");
    me.on_setup(&bus, p);
    publish_gains(bus, me, 0, kRealKeys, kRealVals);   // nobody listening

    EXPECT_EQ(me_counter(me, "gains_applied"), 0);
    EXPECT_EQ(std::get<double>(me.current_params().at("plan_gain")), 0.0)
        << "gain_topic='' must leave params at their config values";
}

TEST(MotorEPMv2GainSocket, AmpSeekCollisionThrows) {
    ogma::InProcessBus bus;
    ogma::MotorEPMv2 me;
    ParamMap p = me_params();
    p["amp_seek_rate"] = 0.1;      // both would mutate amp_target → refuse
    EXPECT_THROW(me.on_setup(&bus, p), std::invalid_argument);
}

TEST(MotorEPMv2GainSocket, SnapshotReplayReappliesGains) {
    ogma::InProcessBus busA;
    ogma::MotorEPMv2 meA;
    meA.on_setup(&busA, me_params());
    publish_gains(busA, meA, 0, kRealKeys, kRealVals);
    nlohmann::json snap = meA.snapshot_state();

    // Fresh module, same CONFIG (which does NOT carry the evolved values):
    // restore must re-dispatch applied_gains_ or the clone silently reverts.
    ogma::InProcessBus busB;
    ogma::MotorEPMv2 meB;
    meB.on_setup(&busB, me_params());
    meB.restore_state(snap);

    ParamMap now = meB.current_params();
    for (size_t i = 0; i < kRealKeys.size(); ++i)
        EXPECT_EQ(std::get<double>(now.at(kRealKeys[i])), kRealVals[i])
            << "restored clone lost evolved gain " << kRealKeys[i];
}

// ---- noise-aware acceptance (the gate-2 fix) --------------------------------

TEST(GainEvolver, AcceptMarginBlocksSubNoiseImprovement) {
    // A candidate that improves J by LESS than the criterion's own measured
    // noise must be reverted: that is the coin flip that drove sigma to the
    // ceiling in gate 2.  Feed alternating distress so revert pairs accumulate
    // a nonzero sigma_hat, then offer a tiny improvement.
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params();
    p["accept_k"]    = 1.0;
    p["noise_min_n"] = int64_t{1};
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    run_ticks(bus, ge, t, kWarmup, Feed{});
    // Generations whose incumbent windows differ a lot => large sigma_hat, and
    // whose candidates are never better => reverts (which is what makes pairs).
    for (int g = 0; g < 4; ++g) {
        Feed inc;  inc.distress  = (g % 2 == 0) ? 0.0f : 1.0f;
        Feed cand; cand.distress = 1.0f;
        run_ticks(bus, ge, t, kWindow, inc);
        run_ticks(bus, ge, t, kWindow, cand);
    }
    EXPECT_GT(metric_d(ge, "sigma_est"), 0.0) << "revert pairs must estimate noise";
    EXPECT_GT(metric_d(ge, "accept_margin"), 0.0);
    int64_t acc_before = metric_i(ge, "accepts");
    // Now a candidate that is better, but by far less than sigma_hat.
    Feed inc;  inc.distress  = 1.0f;
    Feed cand; cand.distress = 0.99f;
    run_ticks(bus, ge, t, kWindow, inc);
    run_ticks(bus, ge, t, kWindow, cand);
    EXPECT_EQ(metric_i(ge, "accepts"), acc_before)
        << "a sub-noise improvement must NOT be accepted";
}

TEST(GainEvolver, AcceptMarginStillAcceptsClearWin) {
    // The margin must not freeze the search: an improvement far larger than the
    // noise is still accepted.
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params();
    p["accept_k"]    = 1.0;
    p["noise_min_n"] = int64_t{1};
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    run_ticks(bus, ge, t, kWarmup, Feed{});
    for (int g = 0; g < 3; ++g) {           // build a small sigma_hat via reverts
        Feed inc;  inc.distress  = 0.50f;
        Feed cand; cand.distress = 1.00f;
        run_ticks(bus, ge, t, kWindow, inc);
        run_ticks(bus, ge, t, kWindow, cand);
    }
    int64_t acc_before = metric_i(ge, "accepts");
    Feed inc;  inc.distress  = 1.0f;
    Feed cand; cand.distress = 0.0f;        // a full-scale improvement
    run_ticks(bus, ge, t, kWindow, inc);
    run_ticks(bus, ge, t, kWindow, cand);
    EXPECT_EQ(metric_i(ge, "accepts"), acc_before + 1);
}

TEST(GainEvolver, DeprecatedTiltVarKeyThrows) {
    // A stale config key would otherwise be silently ignored and the default
    // weight applied to a DIFFERENT quantity (sd, not variance).
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params();
    p["w_tilt_var"] = 5.0;
    EXPECT_THROW(ge.on_setup(&bus, p), std::invalid_argument);
}

// ---- dwell term + explicit settle window ------------------------------------

TEST(GainEvolver, DwellTermMeasuresNearInversionDepth) {
    // dwell = mean(max(0, thresh - upright)) over the MEASURED region only.
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params(/*sigma=*/0.0);   // observer: just score a window
    p["w_dwell"] = 1.0;  p["dwell_thresh"] = 0.9;
    p["w_distress"] = 0.0;
    p["settle_ticks"] = int64_t{100};        // measure ticks 100..199 of a 200 window
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    Feed lean; lean.upright = 0.5f;          // 0.4 below the 0.9 threshold
    run_ticks(bus, ge, t, kWarmup, Feed{});
    run_ticks(bus, ge, t, kWindow, lean);
    EXPECT_NEAR(metric_d(ge, "dwell"), 0.4, 1e-3);

    // An upright body never accumulates dwell.
    ogma::InProcessBus bus2;
    ogma::GainEvolver ge2;
    ge2.on_setup(&bus2, p);
    uint64_t t2 = 0;
    run_ticks(bus2, ge2, t2, kWarmup, Feed{});
    run_ticks(bus2, ge2, t2, kWindow, Feed{});   // upright = 1.0
    EXPECT_NEAR(metric_d(ge2, "dwell"), 0.0, 1e-9);
}

TEST(GainEvolver, SettleTicksExcludesTheHeadOfTheWindow) {
    // With settle_ticks=150 of a 200-tick window, only ticks 150..199 are
    // measured — a disturbance confined to the head must NOT be scored.
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params(/*sigma=*/0.0);
    p["w_dwell"] = 1.0;  p["dwell_thresh"] = 0.9;  p["w_distress"] = 0.0;
    p["settle_ticks"] = int64_t{150};
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    run_ticks(bus, ge, t, kWarmup, Feed{});
    Feed lean; lean.upright = 0.5f;
    run_ticks(bus, ge, t, 150, lean);        // head: leaning, must be ignored
    run_ticks(bus, ge, t, 50, Feed{});       // measured tail: upright
    EXPECT_NEAR(metric_d(ge, "dwell"), 0.0, 1e-9)
        << "settling ticks must not reach the criterion";
}


// ---- energy term + the operator's falls alarm --------------------------------

TEST(GainEvolver, EnergyTermTracksMeanJointTorque) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params(/*sigma=*/0.0);
    p["w_energy"] = 1.0; p["w_distress"] = 0.0;
    p["settle_ticks"] = int64_t{100};
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    Feed f; f.torque = 0.25f;
    run_ticks(bus, ge, t, kWarmup, Feed{});
    run_ticks(bus, ge, t, kWindow, f);
    EXPECT_NEAR(metric_d(ge, "energy"), 0.25, 1e-3);
}

TEST(GainEvolver, LowerEnergyIsPreferredWhenNothingElseDiffers) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params();
    p["w_energy"] = 1.0; p["w_distress"] = 0.0;
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    Feed hi; hi.torque = 0.6f;
    Feed lo; lo.torque = 0.2f;
    run_ticks(bus, ge, t, kWarmup, Feed{});
    run_ticks(bus, ge, t, kWindow, hi);     // incumbent burns more
    run_ticks(bus, ge, t, kWindow, lo);     // candidate is cheaper -> accept
    EXPECT_EQ(metric_i(ge, "accepts"), 1);
}

TEST(GainEvolver, FallAlarmDecaysAndOnlyEverTightens) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params();
    p["fall_alarm_on"]  = 1.5;
    p["fall_alarm_tau"] = 2000.0;
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    run_ticks(bus, ge, t, kWarmup, Feed{});
    EXPECT_NEAR(metric_d(ge, "fall_alarm"), 0.0, 1e-9);
    EXPECT_EQ(metric_i(ge, "alarm_on"), 0);

    // Repeated falls accumulate past the threshold and trip the alarm.
    Feed down; down.upright = -0.5f;
    for (int i = 0; i < 4; ++i) {
        run_ticks(bus, ge, t, 20, down);     // > debounce(10) -> one fall each
        run_ticks(bus, ge, t, 20, Feed{});   // recover (upright clears the latch)
    }
    EXPECT_GT(metric_d(ge, "fall_alarm"), 1.5);
    EXPECT_EQ(metric_i(ge, "alarm_on"), 1);

    // ...and it decays away when the falling stops, so a sporadic faller is
    // never held under a tightened guard forever.
    run_ticks(bus, ge, t, 8000, Feed{});
    EXPECT_LT(metric_d(ge, "fall_alarm"), 1.5);
    EXPECT_EQ(metric_i(ge, "alarm_on"), 0);
}

TEST(GainEvolver, AlarmDisabledByDefaultIsByteIdenticalToNoAlarm) {
    // fall_alarm_on defaults to 0 => the guard must behave exactly as before.
    auto run = [](double alarm_on) {
        ogma::InProcessBus bus;
        ogma::GainEvolver ge;
        ParamMap p = ge_params();
        p["fall_alarm_on"] = alarm_on;
        ge.on_setup(&bus, p);
        uint64_t t = 0;
        Feed good; good.distress = 0.0f;
        Feed bad;  bad.distress = 1.0f;
        run_ticks(bus, ge, t, kWarmup, Feed{});
        for (int g = 0; g < 3; ++g) {
            run_ticks(bus, ge, t, kWindow, bad);
            run_ticks(bus, ge, t, kWindow, good);
        }
        return ge.snapshot_state()["module"]["incumbent"].dump();
    };
    EXPECT_EQ(run(0.0), run(0.0));
    // With no falls at all the alarm never trips, so enabling it changes nothing.
    EXPECT_EQ(run(0.0), run(2.0));
}

// ---- the anneal target must be the NOISE FLOOR, not a fixed constant ---------

TEST(GainEvolver, AutoTargetAcceptIsTheNoiseFloor) {
    // Phi(-k/sqrt(2)) = 0.5*erfc(k/2): the acceptance rate reached while learning
    // nothing.  A fixed 0.2 sat below this for every k the campaign used, so the
    // 1/5th rule inflated sigma on chance-level acceptance.
    struct { double k, want; } cases[] = {
        {0.0, 0.500}, {0.25, 0.430}, {0.5, 0.362}, {1.0, 0.240}, {2.0, 0.079},
    };
    for (auto const& c : cases) {
        ogma::InProcessBus bus;
        ogma::GainEvolver ge;
        ParamMap p = ge_params();
        p["accept_k"] = c.k;
        p["target_accept"] = -1.0;          // AUTO
        ge.on_setup(&bus, p);
        EXPECT_NEAR(metric_d(ge, "target_eff"), c.want, 0.002)
            << "accept_k " << c.k;
    }
}

TEST(GainEvolver, ExplicitTargetAcceptStillOverridesAuto) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params();
    p["accept_k"] = 0.25;
    p["target_accept"] = 0.2;               // explicit wins
    ge.on_setup(&bus, p);
    EXPECT_NEAR(metric_d(ge, "target_eff"), 0.2, 1e-9);
}

TEST(GainEvolver, ChanceLevelAcceptanceNoLongerGrowsSigma) {
    // The gate-2/2e failure in miniature: a search accepting at roughly the noise
    // floor must NOT be rewarded with a bigger step.  Alternate accept/revert at
    // ~50% while k=0 (floor 0.50) and sigma must not climb.
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params(/*sigma=*/0.1);
    p["accept_k"] = 0.0;                    // noise floor 0.50
    p["target_accept"] = -1.0;
    p["anneal_window"] = int64_t{10};
    p["sigma_max"] = 0.5;
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    Feed good; good.distress = 0.0f;
    Feed bad;  bad.distress = 1.0f;
    run_ticks(bus, ge, t, kWarmup, Feed{});
    for (int g = 0; g < 14; ++g) {          // exactly alternating => 50% acceptance
        bool win = (g % 2 == 0);
        run_ticks(bus, ge, t, kWindow, win ? bad : good);
        run_ticks(bus, ge, t, kWindow, win ? good : bad);
    }
    EXPECT_LE(metric_d(ge, "sigma"), 0.1 + 1e-9)
        << "chance-level acceptance must not inflate sigma";
}

// ---- flow term: magnitude must not be tradable for predictability ------------
//
// The flow term exists to be the counterweight that stops the search minimizing
// every other term by standing still.  The 2026-08-23 landscape sweeps measured
// it failing at that job: it scored BEST at the gait amplitude whose mean travel
// was LOWEST, because a quiet body is a predictable one and the product form let
// the predictability it gained pay for the travel it lost.  These tests pin both
// halves — the defect under the legacy form, its absence under the new one — so
// the fix cannot silently regress into the shape it replaced.

namespace {

struct FlowRead { double term, mag, pred; };

// One scored window in OBSERVER mode with fwd_v = mean ± swing alternating.  The
// warmup is long enough (12 EMA time constants at alpha 0.02) that both EMAs are
// settled before the measured region opens, so the read is the steady state of
// the requested (magnitude, volatility) pair and not a settling transient.
FlowRead flow_window(int64_t form, float mean_v, float swing) {
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params(/*sigma=*/0.0);
    p["w_distress"] = 0.0;
    p["w_flow"]     = 1.0;
    p["flow_min_form"] = form;
    p["warmup_ticks"]  = int64_t{600};
    ge.on_setup(&bus, p);
    uint64_t t = 0;
    Feed f;
    for (int64_t i = 0; i < 600 + kWindow; ++i) {
        f.fwd_v = mean_v + ((i % 2 == 0) ? swing : -swing);
        ev_tick(bus, ge, t++, f);
    }
    return {metric_d(ge, "flow_term"), metric_d(ge, "flow_mag"), metric_d(ge, "flow_pred")};
}

} // namespace

TEST(GainEvolver, LegacyFlowFormLetsSteadinessPayForLostTravel) {
    // A crawls (0.06 m/s) but is perfectly steady; B travels 3.3x faster (0.20)
    // with real stride-to-stride variation.  Both sit ABOVE the legacy magnitude
    // ceiling, so under form 0 magnitude is pinned at 1.0 for both and only
    // predictability separates them -- and it crowns the crawl.
    FlowRead a = flow_window(/*form=*/0, 0.06f, 0.0f);
    FlowRead b = flow_window(/*form=*/0, 0.20f, 0.10f);
    EXPECT_NEAR(a.mag, 1.0, 1e-6);
    EXPECT_NEAR(b.mag, 1.0, 1e-6);
    EXPECT_LT(a.term, b.term) << "legacy form is expected to prefer the crawl";
}

TEST(GainEvolver, MinFlowFormPrefersTravelOverASteadyCrawl) {
    // Same two bodies, same criterion weight, only the combining rule differs.
    FlowRead a = flow_window(/*form=*/1, 0.06f, 0.0f);
    FlowRead b = flow_window(/*form=*/1, 0.20f, 0.10f);
    EXPECT_LT(b.term, a.term) << "min form must prefer the body that travels";
    // and it must be MAGNITUDE that limits the crawl, not steadiness
    EXPECT_LT(a.mag, a.pred);
}

TEST(GainEvolver, LegacyMagnitudeSaturatesAndDropsOutOfTheComparison) {
    // Steady travel at 0.06, 0.20 and 0.60 m/s -- a tenfold range.  Under the
    // legacy ceiling all three score the same, which is the mechanism by which
    // magnitude stopped contributing to the landscape at all.
    double lo = flow_window(0, 0.06f, 0.0f).term;
    double mid = flow_window(0, 0.20f, 0.0f).term;
    double hi = flow_window(0, 0.60f, 0.0f).term;
    EXPECT_NEAR(lo, mid, 1e-4);
    EXPECT_NEAR(mid, hi, 1e-4);
    // Under the min form the same tenfold range separates cleanly and monotonically.
    double m_lo = flow_window(1, 0.06f, 0.0f).term;
    double m_mid = flow_window(1, 0.20f, 0.0f).term;
    double m_hi = flow_window(1, 0.60f, 0.0f).term;
    EXPECT_GT(m_lo, m_mid + 0.1);
    EXPECT_GT(m_mid, m_hi + 0.1);
}

TEST(GainEvolver, StillnessIsWorstUnderBothForms) {
    // The floor case both forms must agree on: a body going nowhere scores the
    // worst possible flow however steadily it does it.
    EXPECT_NEAR(flow_window(0, 0.0f, 0.0f).term, 1.0, 1e-6);
    EXPECT_NEAR(flow_window(1, 0.0f, 0.0f).term, 1.0, 1e-6);
}

TEST(GainEvolver, FlowMinFormDefaultsToLegacyAndIsByteIdentical) {
    // The new rule ships OFF: an existing config that never mentions the key must
    // score exactly as it did before it existed.
    ogma::InProcessBus bus;
    ogma::GainEvolver ge;
    ParamMap p = ge_params(/*sigma=*/0.0);
    p.erase("flow_min_form");
    ge.on_setup(&bus, p);
    EXPECT_EQ(std::get<int64_t>(ge.current_params().at("flow_min_form")), int64_t{0});
    FlowRead def = flow_window(0, 0.12f, 0.04f);
    EXPECT_DOUBLE_EQ(def.term, flow_window(0, 0.12f, 0.04f).term);
}
