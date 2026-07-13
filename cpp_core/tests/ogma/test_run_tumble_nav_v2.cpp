// RunTumbleNavV2 — clean-room scalar chemotaxis, kinesis promoted to a reactive taxis.
// Always-on: methylation error + KF4 noise floor, KF1 run integrity, KF2 efference-matched
// stuck, KF6 directional belief. No orthokinesis. A single `ablation` enum drives the
// validation controls (shuffle = random-walk floor; kinesis / wrong_sign / shuffle_dir).
//
// NOTE: run_commit is always-on, so the body must TURN toward the committed run_dir for a run
// to execute (in deployment the HeadingController does this). The harness models it: run_follow()
// and the 2D approach sim feed heading = nav.run_dir(), i.e. an idealised body that faces its
// commit — isolating the tumble POLICY under test.
#include <gtest/gtest.h>
#include "ogma/modules/RunTumbleNavV2.hpp"
#include "ogma/InProcessBus.hpp"

#include <cmath>
#include <memory>

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float wrap_pi(float a) { return std::atan2(std::sin(a), std::cos(a)); }
std::shared_ptr<ogma::ProprioToken> p1(float v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::RunTumbleNavV2 nav;
    Fix(ogma::ParamMap p = {}) { nav.set_id("rt2"); nav.on_setup(&bus, p); }
    // body faces a GIVEN heading (used only where we deliberately test reorientation).
    void run_vel(uint64_t t, float smax, float vfwd, float heading) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent_max", p1(smax));
        bus.publish("reality.proprio.heading", p1(heading));
        auto v = std::make_shared<ogma::ProprioToken>(); v->values.resize(2);
        v->values[0] = 0.0f; v->values[1] = vfwd;
        bus.publish("reality.proprio.vel_ego", v);
        nav.tick(t); bus.end_tick();
    }
    // idealised body: instantly faces the committed run_dir (heading = run_dir) → executing.
    void run_follow(uint64_t t, float smax, float vfwd) { run_vel(t, smax, vfwd, nav.run_dir()); }
    void run_eat(uint64_t t, float smax) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent_max", p1(smax));
        bus.publish("reality.proprio.heading", p1(nav.run_dir()));
        auto ev = std::make_shared<ogma::EnvEvent>(); ev->name = "eat";
        bus.publish("events.eat", ev);
        nav.tick(t); bus.end_tick();
    }
};

// 2D approach: source at the origin, bug starts on the -x side and re-spawns there on arrival.
// The good absolute run direction is ~0 (toward +x). A working taxis infers mu→0, R>0 and reaches
// the source repeatedly. Position clamped to the arena. Returns the arrival count.
int approach_sim(Fix& f, int steps) {
    float px = -12.0f, py = 0.0f; int arrivals = 0; uint64_t t = 0;
    for (int i = 0; i < steps; ++i) {
        float rd = f.nav.run_dir();
        px += 0.4f * std::cos(rd); py += 0.4f * std::sin(rd);
        px = std::clamp(px, -15.0f, 15.0f); py = std::clamp(py, -15.0f, 15.0f);
        float dist = std::sqrt(px*px + py*py);
        if (dist < 1.5f) { ++arrivals; px = -12.0f; py = 0.0f; dist = std::sqrt(px*px + py*py); }
        float s = std::max(0.1f, 20.0f - dist);
        f.run_vel(t++, s, /*vfwd=*/3.0f, /*heading=*/rd);
    }
    return arrivals;
}
}  // namespace

// (1) Rising scent keeps running (methylation prediction error suppresses tumbling).
TEST(RunTumbleNavV2, RisingScentKeepsRunning) {
    Fix f({{"master_seed", int64_t{7}}});
    uint64_t t = 0; float s = 0.5f;
    for (int i = 0; i < 150; ++i) { s += 0.1f; f.run_follow(t++, s, 3.0f); }
    EXPECT_GT(f.nav.last_error(), 0.0f) << "rising scent → positive prediction error";
    int run0 = f.nav.run_count(), tum0 = f.nav.tumble_count();
    for (int i = 0; i < 200; ++i) { s += 0.1f; f.run_follow(t++, s, 3.0f); }
    int runs = f.nav.run_count() - run0, tumbles = f.nav.tumble_count() - tum0;
    double frac = double(tumbles) / double(runs + tumbles);
    EXPECT_LT(frac, 0.2) << "a steady climb must run far more than it tumbles (frac=" << frac << ")";
}

// (2) Falling scent raises tumbling.
TEST(RunTumbleNavV2, FallingScentRaisesTumble) {
    Fix f({{"master_seed", int64_t{7}}});
    uint64_t t = 0; float s = 20.0f;
    for (int i = 0; i < 200; ++i) { s -= 0.1f; f.run_follow(t++, s, 3.0f); }
    EXPECT_LT(f.nav.last_error(), 0.0f) << "falling scent → negative prediction error";
    EXPECT_GT(f.nav.tumble_count(), 0) << "falling scent must produce tumbles";
}

// (3) ablation="shuffle" is the gradient-blind random-walk floor: same tumble fraction on a
// rising vs a falling ramp, whereas the gradient-aware live loop tumbles much less while climbing.
TEST(RunTumbleNavV2, ShuffleIsRandomWalk) {
    auto tumble_frac = [](const char* ablation, bool rising) {
        Fix f({{"ablation", std::string(ablation)}, {"master_seed", int64_t{3}}, {"tumble_base", 0.1}});
        uint64_t t = 0; float s = rising ? 1.0f : 80.0f;
        for (int i = 0; i < 500; ++i) { s += rising ? 0.1f : -0.1f; f.run_follow(t++, s, 3.0f); }
        int total = f.nav.run_count() + f.nav.tumble_count();
        return double(f.nav.tumble_count()) / double(total);
    };
    double shuf_rise = tumble_frac("shuffle", true);
    double shuf_fall = tumble_frac("shuffle", false);
    EXPECT_NEAR(shuf_rise, shuf_fall, 0.05) << "shuffle is gradient-blind (rise=" << shuf_rise << " fall=" << shuf_fall << ")";
    double live_rise = tumble_frac("kinesis", true);
    double live_fall = tumble_frac("kinesis", false);
    EXPECT_LT(live_rise, live_fall - 0.1) << "gradient-aware tumbles less while climbing (rise=" << live_rise << " fall=" << live_fall << ")";
}

// (4) KF4 noise floor: warm the speed scale with motion, then a flat-but-noisy field sampled while
// stationary — the floor rises and clamps the normaliser so p_tumble stays near the flat base.
TEST(RunTumbleNavV2, NoiseFloorClampsFlatFieldResponse) {
    Fix f({{"master_seed", int64_t{5}}, {"noise_floor_alpha", 0.05}});
    uint64_t t = 0; float s = 3.0f;
    for (int i = 0; i < 60; ++i) { s += 0.05f; f.run_follow(t++, s, 3.0f); }   // establish vel_scale
    for (int i = 0; i < 400; ++i) {
        float wob = 0.05f * std::sin(0.7f * float(i));
        f.run_vel(t++, 3.0f + wob, /*vfwd=*/0.0f, /*heading=*/f.nav.run_dir());  // stationary, flat+noise
    }
    EXPECT_GT(f.nav.noise_floor(), 0.0f) << "stationary noise must be learned into the floor";
    EXPECT_LT(f.nav.last_p_tumble(), 0.3f) << "flat noisy field must not peg p_tumble (p=" << f.nav.last_p_tumble() << ")";
}

// (5) KF1 run integrity: no forced tumbles fire mid-reorientation (the K2 livelock signature).
TEST(RunTumbleNavV2, NoMidTurnForcedTumbles) {
    Fix f({{"master_seed", int64_t{7}}, {"stuck_ticks", int64_t{6}}});
    uint64_t t = 0; float s = 1.0f;
    for (int i = 0; i < 20; ++i) { s += 0.1f; f.run_follow(t++, s, 3.0f); }
    for (int i = 0; i < 200; ++i) f.run_vel(t++, 2.0f, /*vfwd=*/0.0f, /*heading=*/f.nav.run_dir());
    EXPECT_EQ(f.nav.forced_in_turn(), 0) << "the stuck counter is held while reorienting → no mid-turn forced tumbles";
}

// (6) KF6 directional belief: the taxis INFERS the up-gradient direction from its own run
// outcomes — precision R rises above the kinesis floor and mu points toward the source (~0).
// (Whether the belief improves FORAGING is the config-battery's job — an open-field unit sim is
// too easy for the taxis edge to show; here we verify the inference mechanism is correct.)
TEST(RunTumbleNavV2, DirectionalBeliefInfersGoodDirection) {
    Fix f({{"master_seed", int64_t{11}}, {"dir_lr", 0.2}});
    approach_sim(f, 4000);
    EXPECT_GT(f.nav.dir_consistency(), 0.02f) << "consistent evidence must raise the belief precision R above 0 (R=" << f.nav.dir_consistency() << ")";
    EXPECT_LT(std::fabs(wrap_pi(f.nav.dir_mu())), 1.5708f) << "the believed heading mu must point toward the +x source hemisphere (mu=" << f.nav.dir_mu() << ")";
}

// (7) ablation="kinesis" clamps the belief off (R stays exactly 0 = the tabula-rasa floor).
TEST(RunTumbleNavV2, KinesisAblationZeroesBelief) {
    Fix kin({{"master_seed", int64_t{11}}, {"dir_lr", 0.2}, {"ablation", std::string("kinesis")}});
    approach_sim(kin, 4000);
    EXPECT_EQ(kin.nav.dir_consistency(), 0.0f) << "kinesis ablation must never accumulate a directional belief";
}

// (8) ablation="wrong_sign" must REGRESS: crediting the WRONG direction cannot home as well.
TEST(RunTumbleNavV2, WrongSignRegresses) {
    Fix live({{"master_seed", int64_t{11}}, {"dir_lr", 0.2}});
    Fix bad ({{"master_seed", int64_t{11}}, {"dir_lr", 0.2}, {"ablation", std::string("wrong_sign")}});
    int a_live = approach_sim(live, 4000);
    int a_bad  = approach_sim(bad,  4000);
    EXPECT_GT(a_live, a_bad) << "wrong-sign credit must home worse than the live taxis (live=" << a_live << " bad=" << a_bad << ")";
}

// (9) Capability: eat-calibrated confidence reads ~1 in the eating range, ~0 when blind.
TEST(RunTumbleNavV2, CapabilityCalibratesToEatScent) {
    Fix f({{"master_seed", int64_t{7}}});
    uint64_t t = 0;
    for (int i = 0; i < 20; ++i) f.run_follow(t++, 0.9f, 3.0f);
    f.run_eat(t++, 0.5f);
    EXPECT_TRUE(f.nav.have_eat_scent());
    f.run_follow(t++, 0.5f, 3.0f);
    EXPECT_GT(f.nav.capability(), 0.9f) << "in its own eating range capability → ~1 (cap=" << f.nav.capability() << ")";
    f.run_follow(t++, 0.0f, 3.0f);
    EXPECT_LT(f.nav.capability(), 0.1f) << "blind → capability ~0";
}

// (10) Capability is published on the confidence topic each tick.
TEST(RunTumbleNavV2, PublishesCapability) {
    float cap = -1.0f;
    Fix f;
    f.bus.subscribe("percept.klino_confidence", ogma::SubscriptionKind::Direct,
        [&](std::string_view, ogma::MessagePtr m){
            auto pt = std::dynamic_pointer_cast<const ogma::ProprioToken>(m);
            if (pt && pt->values.size() > 0) cap = float(pt->values[0]);
        });
    uint64_t t = 0; float s = 1.0f;
    for (int i = 0; i < 30; ++i) { s += 0.1f; f.run_follow(t++, s, 3.0f); }
    EXPECT_GE(cap, 0.0f) << "capability must be published on the confidence topic";
}
