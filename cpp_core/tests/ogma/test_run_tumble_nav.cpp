// RunTumbleNav — honest temporal chemotaxis via the E. coli METHYLATION reflex.
// A leaky methylation baseline (EMA of scent) is a prediction of expected scent; the
// per-tick tumble PROBABILITY is low while scent rises above it (positive prediction
// error → keep running) and rises as scent falls below it (stalling → tumble). No clock,
// no learning. shuffle = the FAIR null: gradient-blind tumbling at the flat tumble_base
// rate (same mean rate, no gradient modulation), isolating gradient-awareness.
#include <gtest/gtest.h>
#include "ogma/modules/RunTumbleNav.hpp"
#include "ogma/InProcessBus.hpp"

#include <memory>

namespace {
std::shared_ptr<ogma::ProprioToken> p1(float v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::RunTumbleNav nav;
    Fix(ogma::ParamMap p = {}) { nav.set_id("rt"); nav.on_setup(&bus, p); }
    void run(uint64_t t, float smax, float heading = 0.0f) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent_max", p1(smax));
        bus.publish("reality.proprio.heading", p1(heading));
        nav.tick(t); bus.end_tick();
    }
    void run_vel(uint64_t t, float smax, float vfwd, float heading = 0.0f) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent_max", p1(smax));
        bus.publish("reality.proprio.heading", p1(heading));
        auto v = std::make_shared<ogma::ProprioToken>(); v->values.resize(2);
        v->values[0] = 0.0f; v->values[1] = vfwd;
        bus.publish("reality.proprio.vel_ego", v);
        nav.tick(t); bus.end_tick();
    }
    // variant that also fires a GROUND-TRUTH eat event this tick (EnvEvent on events.eat) → calibrates eat_scent_.
    void run_eat(uint64_t t, float smax, float heading = 0.0f) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent_max", p1(smax));
        bus.publish("reality.proprio.heading", p1(heading));
        auto ev = std::make_shared<ogma::EnvEvent>(); ev->name = "eat";
        bus.publish("events.eat", ev);
        nav.tick(t); bus.end_tick();
    }
};
}  // namespace

// (1) Rising scent: scent stays above the methylation baseline → positive prediction
// error → p_tumble is SUPPRESSED below the flat-gradient base → mostly RUNNING.  (As in
// real E. coli, a *sustained constant* slope partly adapts away via the normaliser, so we
// claim the honest, robust fact — the gradient clearly suppresses tumbling vs the shuffle
// ~50% control — not literal zero.)
TEST(RunTumbleNav, RisingScentKeepsRunning) {
    Fix f({{"master_seed", int64_t{7}}});
    uint64_t t = 0; float s = 0.5f;
    for (int i = 0; i < 150; ++i) { s += 0.1f; f.run(t++, s); }  // warm-up: settle the normaliser
    EXPECT_GT(f.nav.last_error(), 0.0f) << "rising scent → positive prediction error";
    EXPECT_LT(f.nav.last_p_tumble(), 0.1f)
        << "a settled climb should hold p_tumble below the flat-gradient base (run up-gradient)";
    int run0 = f.nav.run_count(), tum0 = f.nav.tumble_count();
    for (int i = 0; i < 200; ++i) { s += 0.1f; f.run(t++, s); }  // measurement window
    int runs = f.nav.run_count() - run0, tumbles = f.nav.tumble_count() - tum0;
    double frac = double(tumbles) / double(runs + tumbles);
    EXPECT_LT(frac, 0.2)
        << "a steady climb must run far more than it tumbles, well below the shuffle ~0.5 "
           "(runs=" << runs << " tumbles=" << tumbles << " frac=" << frac << ")";
}

// (2) Falling scent: scent drops below the baseline → negative prediction error →
// p_tumble rises toward tumble_max and tumbles actually happen.
TEST(RunTumbleNav, FallingScentRaisesTumble) {
    Fix f({{"master_seed", int64_t{7}}});
    uint64_t t = 0; float s = 20.0f;
    for (int i = 0; i < 200; ++i) { s -= 0.1f; f.run(t++, s); }  // scent monotonically falling
    EXPECT_LT(f.nav.last_error(), 0.0f) << "falling scent → negative prediction error";
    EXPECT_GT(f.nav.last_p_tumble(), 0.15f)
        << "falling scent should push p_tumble well above the flat-gradient base (~0.1)";
    EXPECT_GT(f.nav.tumble_count(), 0) << "falling scent must produce tumbles";
}

// (3) shuffle = the FAIR null. Gradient-BLIND tumbling at the flat tumble_base rate: same
// MEAN rate as the methylation, but the gradient does not modulate it. So the shuffle
// tumble fraction is ~tumble_base AND essentially identical on a rising vs a falling ramp,
// whereas the methylation tumbles much LESS on a rising ramp than on a falling ramp. The
// only variable between the two arms is gradient-awareness.
TEST(RunTumbleNav, ShuffleIsRandomWalk) {
    auto tumble_frac = [](bool shuffle, bool rising) {
        Fix f({{"shuffle", shuffle}, {"master_seed", int64_t{3}}, {"tumble_base", 0.1}});
        uint64_t t = 0; float s = rising ? 1.0f : 80.0f;
        for (int i = 0; i < 400; ++i) { s += rising ? 0.1f : -0.1f; f.run(t++, s); }
        int total = f.nav.run_count() + f.nav.tumble_count();
        return double(f.nav.tumble_count()) / double(total);
    };

    // SHUFFLE (fair null): tumble fraction ~tumble_base on BOTH ramps, gradient-blind.
    double shuf_rise = tumble_frac(/*shuffle=*/true,  /*rising=*/true);
    double shuf_fall = tumble_frac(/*shuffle=*/true,  /*rising=*/false);
    EXPECT_NEAR(shuf_rise, 0.1, 0.05)
        << "shuffle tumble fraction should be ~tumble_base (0.1) on a rising ramp (rise=" << shuf_rise << ")";
    EXPECT_NEAR(shuf_fall, 0.1, 0.05)
        << "shuffle tumble fraction should be ~tumble_base (0.1) on a falling ramp (fall=" << shuf_fall << ")";
    EXPECT_NEAR(shuf_rise, shuf_fall, 0.05)
        << "shuffle is gradient-blind → essentially the same fraction on a rising vs falling ramp "
           "(rise=" << shuf_rise << " fall=" << shuf_fall << ")";

    // METHYLATION (gradient-aware): tumbles much LESS on a rising ramp than on a falling
    // ramp — the gradient suppresses tumbling while climbing and lifts it while descending.
    // (On a *sustained constant* slope the normaliser adapts the rising case back toward the
    // flat base ~0.1, so the robust asymmetry to assert is rising < falling, not rising < base.)
    double meth_rise = tumble_frac(/*shuffle=*/false, /*rising=*/true);
    double meth_fall = tumble_frac(/*shuffle=*/false, /*rising=*/false);
    EXPECT_LT(meth_rise, meth_fall - 0.1)
        << "methylation must tumble much LESS on a rising ramp than on a falling ramp "
           "(rise=" << meth_rise << " fall=" << meth_fall << ")";
}

// (4) Stuck: commanded a run but forward velocity stays ~0 (blocked) → after stuck_ticks
// a tumble is FORCED (the honest egomotion action-consequence).
TEST(RunTumbleNav, TumblesWhenStuck) {
    Fix f({{"stuck_ticks", int64_t{8}}, {"stuck_vel_thresh", 0.5}, {"master_seed", int64_t{7}}});
    uint64_t t = 0; float s = 1.0f;
    // first run a few ticks with good forward velocity (rising scent → low p_tumble)
    for (int i = 0; i < 5; ++i) { s += 0.2f; f.run_vel(t++, s, /*vfwd=*/3.0f); }
    int before = f.nav.forced_tumbles();
    // now blocked: flat scent + zero forward velocity for > stuck_ticks
    for (int i = 0; i < 20; ++i) f.run_vel(t++, s, /*vfwd=*/0.0f);
    EXPECT_GT(f.nav.forced_tumbles(), before) << "a sustained-blocked run must force a tumble";
}

// (6) SELF-REPORTED CAPABILITY (the (i) capability the arbiter consumes). klino is a
// chemotaxer: capability = current-smell / typical-food-smell ∈[0,1], EMERGENT from its own
// running scent scale (NOT a fixed `if scent<eps` gate). →~1 when smelling at its peak,
// →0 when blind (numerator is 0). The peak DECAYS slowly (a wilderness stretch can't erase
// the remembered food-scent scale) — yet capability still reads 0 in the wilderness because
// the NUMERATOR (smax) is 0, not because the peak forgot the food.
TEST(RunTumbleNav, CapabilityHighAtPeakZeroWhenBlind) {
    // fast peak_decay would let a long blind stretch shrink the peak; the default 0.0005 keeps
    // the scale for ~1400 ticks. Use the default to prove the honest behavior.
    Fix f({{"master_seed", int64_t{7}}});
    uint64_t t = 0;
    // smell food at a steady high concentration (maze field is source-normalised to 1.0) → capability
    // is the fraction-of-source-strength ≈ the scent itself pre-eat (cap = smax/1.0).
    for (int i = 0; i < 200; ++i) f.run(t++, /*smax=*/0.95f);
    EXPECT_GT(f.nav.capability(), 0.9f)
        << "smelling near the source strength → capability ~1 (cap=" << f.nav.capability()
        << " peak=" << f.nav.scent_peak() << ")";
    float peak_at_food = f.nav.scent_peak();
    EXPECT_NEAR(peak_at_food, 0.95f, 0.1f) << "scent_peak (telemetry) remembers the typical food magnitude";

    // now BLIND: scent goes to ~0 for a long stretch. cap = smax/1.0 collapses to 0 IMMEDIATELY.
    for (int i = 0; i < 300; ++i) f.run(t++, /*smax=*/0.0f);
    EXPECT_LT(f.nav.capability(), 0.01f)
        << "a BLIND klino (scent≈0) self-reports ~0 capability (cap=" << f.nav.capability() << ")";
    EXPECT_GT(f.nav.scent_peak(), 0.45f)
        << "the scent-magnitude memory (telemetry) decays SLOWLY — a wilderness stretch doesn't erase "
           "the remembered food scale (peak=" << f.nav.scent_peak() << ", was " << peak_at_food << ")";

    // and it RECOVERS to ~1 the instant it smells food again at the source scale.
    for (int i = 0; i < 50; ++i) f.run(t++, /*smax=*/0.95f);
    EXPECT_GT(f.nav.capability(), 0.9f)
        << "smelling food again near the source strength → capability recovers to ~1 (cap="
        << f.nav.capability() << ")";

    // capability is always a valid probability.
    EXPECT_GE(f.nav.capability(), 0.0f);
    EXPECT_LE(f.nav.capability(), 1.0f);
}

// (6b) EAT-CALIBRATED confidence: klino learns the scent at which it ACTUALLY eats and pins that
// scale to capability=1. Before the first eat, capability bootstraps off the running scent peak; a
// bug that has SMELLED a higher scent than it EATS at therefore under-reports at its true eat-scent.
// Once it eats, eat_scent_ anchors the denominator to the real consummatory scent, so capability
// reads ~1 the instant it is back in its own eating range — this is the fix that lets the L2 arbiter
// lift klino's value to the planner's scale and close the food. The eat calibrates ONLY confidence:
// the run/tumble policy is reward-free, so it is unaffected.
TEST(RunTumbleNav, EatCalibratesConfidenceToEatScent) {
    Fix f({{"master_seed", int64_t{7}}});
    uint64_t t = 0;
    // The maze field is source-normalised to 1.0, and the bug's nose samples a SUB-1 scent at its
    // closest approach (e.g. ~0.5 in the eating range). PRE-EAT, capability is the raw fraction-of-source
    // (smax/1.0), so at eat-scale 0.5 it UNDER-reports (~0.5) — the bug does not yet know 0.5 is its
    // eating scale. Warm-up smell then settle at the realistic eat-scent.
    for (int i = 0; i < 200; ++i) f.run(t++, /*smax=*/0.9f);
    for (int i = 0; i < 50;  ++i) f.run(t++, /*smax=*/0.5f);
    ASSERT_FALSE(f.nav.have_eat_scent()) << "no eat yet → pre-eat source-scale (1.0) denominator";
    EXPECT_NEAR(f.nav.capability(), 0.5f, 0.08f)
        << "pre-eat capability at scent 0.5 is the raw fraction-of-source ~0.5 (under-reports the eat-scale; cap="
        << f.nav.capability() << ")";

    // The bug EATS at scent 0.5 → eat_scent_ is seeded to 0.5 on the first hit.
    f.run_eat(t++, /*smax=*/0.5f);
    EXPECT_TRUE(f.nav.have_eat_scent());
    EXPECT_NEAR(f.nav.eat_scent(), 0.5f, 1e-3f) << "first eat seeds eat_scent_ to the scent at the hit";

    // POST-EAT: capability at the SAME scent 0.5 now reads ~1 — calibrated to the real eat-scale, not
    // the source scale. This is the boost the arbiter needs to own the close.
    for (int i = 0; i < 5; ++i) f.run(t++, /*smax=*/0.5f);
    EXPECT_GT(f.nav.capability(), 0.95f)
        << "post-eat capability at the eat-scent is ~1 (was ~0.5 pre-eat): cap=" << f.nav.capability();

    // and a BLIND klino (scent≈0) still self-reports ~0 — the calibration BOOSTS near food, it does
    // not floor the report; the numerator (smax) is 0 far from food regardless of the eat-scale.
    for (int i = 0; i < 50; ++i) f.run(t++, /*smax=*/0.0f);
    EXPECT_LT(f.nav.capability(), 0.01f)
        << "blind klino still reports ~0 capability post-calibration (cap=" << f.nav.capability() << ")";
}

// (7) Capability is PUBLISHED on the confidence topic every tick (the arbiter reads it).
TEST(RunTumbleNav, PublishesCapabilityOnConfidenceTopic) {
    Fix f({{"master_seed", int64_t{7}}});
    for (uint64_t t = 0; t < 100; ++t) f.run(t, /*smax=*/3.0f);
    auto c = std::dynamic_pointer_cast<const ogma::ProprioToken>(
        f.bus.last_value("percept.klino_confidence"));
    ASSERT_NE(c, nullptr) << "klino must publish its capability on percept.klino_confidence";
    ASSERT_EQ(c->values.size(), 1u);
    EXPECT_FLOAT_EQ(float(c->values[0]), f.nav.capability())
        << "the published confidence equals the self-reported capability";
}

// (KF1) RUN INTEGRITY. The per-tick tumble draw re-tumbles during the multi-tick turn
// transient after a tumble (thrust≈0 while reorienting → low velocity → the stuck check
// force-tumbles MID-TURN = the K2 livelock, and most runs die before they start = K1).
// run_commit gates the tumble draw + stuck counter on EXECUTING a run (facing run_dir), so a
// forced tumble can NEVER fire while still reorienting. Baseline (off) shows the pathology.
TEST(RunTumbleNav, RunCommitNoMidTurnForcedTumbles) {
    auto forced_in_turn = [](bool run_commit) {
        Fix f({{"stuck_ticks", int64_t{4}}, {"stuck_vel_thresh", 0.5},
               {"master_seed", int64_t{7}}, {"run_commit", run_commit}});
        uint64_t t = 0; float s = 20.0f;
        // falling scent (frequent tumbles → frequent large reorientations) with ZERO forward
        // velocity (turning, not travelling) — the exact regime that force-tumbles mid-turn.
        for (int i = 0; i < 400; ++i) { s -= 0.05f; f.run_vel(t++, s, /*vfwd=*/0.0f); }
        return f.nav.forced_in_turn();
    };
    EXPECT_GT(forced_in_turn(/*run_commit=*/false), 0)
        << "baseline: the reactive stuck check force-tumbles WHILE reorienting (the K2 livelock)";
    EXPECT_EQ(forced_in_turn(/*run_commit=*/true), 0)
        << "run_commit: a forced tumble never fires mid-turn (the stuck counter holds while reorienting)";
}

// A minimal closed-loop BODY sim for the taxis tests: the body turns toward the committed
// run_dir (as the HeadingController does) and travels along its heading; scent RISES while the
// heading points near a hidden target absolute direction and FALLS elsewhere (a spatial gradient
// discoverable only by acting). Returns (belief precision R, mean |run_dir − target| over the
// last fifth = tumble concentration).
namespace {
inline float wrap_pi_t(float a){ return std::atan2(std::sin(a), std::cos(a)); }
std::pair<double,double> taxis_loop(bool dir_belief, float target, int steps, uint64_t seed = 7) {
    Fix f({{"master_seed", int64_t(seed)}, {"run_commit", true}, {"dir_belief", dir_belief},
           {"dir_lr", 0.2}, {"tumble_base", 0.1}});
    uint64_t t = 0; float s = 5.0f; float heading = 0.0f; float run_dir = 0.0f;
    double sum_absdev = 0.0; int n = 0;
    for (int i = 0; i < steps; ++i) {
        bool good = std::fabs(wrap_pi_t(heading - target)) < 0.5f;   // travelling up-gradient this tick?
        s += good ? 0.03f : -0.03f; s = std::clamp(s, 0.1f, 10.0f);
        f.run_vel(t++, s, /*vfwd=*/ good ? 3.0f : 0.5f, heading);
        auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(
            f.bus.last_value("percept.runtumble_heading"));
        float cmd = std::atan2(o->values[0], o->values[1]);   // commanded turn (may be turn-commit-latched)
        heading = wrap_pi_t(heading + 0.3f * cmd);             // body turns toward the command (as the HC does)
        run_dir = f.nav.run_dir();                             // committed absolute dir (accessor, latch-independent)
        if (i >= steps - steps / 5) { sum_absdev += std::fabs(wrap_pi_t(run_dir - target)); ++n; }
    }
    return {double(f.nav.dir_consistency()), sum_absdev / double(n)};
}
}  // namespace

// (KF6) DIRECTIONAL BELIEF — kinesis → reactive taxis. The belief must INFER the good direction
// from its own run outcomes and concentrate tumbles there. dir_belief OFF is the kinesis ablation.
TEST(RunTumbleNav, DirBeliefInfersAndConcentratesOnGoodDirection) {
    auto on  = taxis_loop(/*dir_belief=*/true,  /*target=*/1.0f, 5000);
    auto off = taxis_loop(/*dir_belief=*/false, /*target=*/1.0f, 5000);
    EXPECT_GT(on.first, 0.3)
        << "belief precision (mean resultant length) rises when a direction consistently pays off (R="
        << on.first << ")";
    // (margin 0.10, not tighter: dir_decay_on_loss trades a little concentration for fast
    // re-inference — the belief bleeds on the ~2/3 of tumble-cone draws that fall off-target.)
    EXPECT_LT(on.second, off.second - 0.10)
        << "dir_belief concentrates runs markedly nearer the good direction than kinesis "
           "(dev_on=" << on.second << " dev_off=" << off.second << ")";
}

// (KF6b) The belief RE-INFERS after a perturbation (bar d): once the good direction FLIPS, the
// adaptive precision collapses and the belief mean rotates toward the new direction — what keeps
// a taxis from locking a stale heading at a maze corner / source relocation. Uses a SMOOTH
// (cosine) local gradient like the real screened-Poisson field (always a direction to descend),
// not a binary far-flip (which is an artificially hard search with no local cue).
TEST(RunTumbleNav, DirBeliefReinfersAfterPerturbation) {
    Fix f({{"master_seed", int64_t{7}}, {"run_commit", true}, {"dir_belief", true},
           {"dir_lr", 0.2}, {"tumble_base", 0.1}});
    uint64_t t = 0; float s = 5.0f; float heading = 0.0f; float run_dir = 0.0f;
    auto step = [&](float target){
        float align = std::cos(wrap_pi_t(heading - target));   // +1 up-gradient … −1 down-gradient
        s = std::clamp(s + 0.03f * align, 0.1f, 10.0f);
        f.run_vel(t++, s, /*vfwd=*/ 1.5f + 1.5f * align, heading);
        auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(
            f.bus.last_value("percept.runtumble_heading"));
        float cmd = std::atan2(o->values[0], o->values[1]);
        heading = wrap_pi_t(heading + 0.3f * cmd);
        run_dir = f.nav.run_dir();
    };
    for (int i = 0; i < 6000; ++i) step(1.0f);       // learn: good direction ≈ +1.0
    float mu_before = f.nav.dir_mu();
    EXPECT_LT(std::fabs(wrap_pi_t(mu_before - 1.0f)), 0.7f)
        << "belief mean converged near the first good direction (mu=" << mu_before << ")";
    for (int i = 0; i < 8000; ++i) step(-1.5f);      // PERTURB: good direction flips to −1.5
    float mu_after = f.nav.dir_mu();
    EXPECT_LT(std::fabs(wrap_pi_t(mu_after - (-1.5f))),
              std::fabs(wrap_pi_t(mu_after - mu_before)))
        << "belief re-inferred: mean is now closer to the NEW good direction than the OLD one "
           "(mu_after=" << mu_after << " new=-1.5 old=" << mu_before << ")";
    EXPECT_LT(std::fabs(wrap_pi_t(mu_after - (-1.5f))), 1.0f)
        << "belief mean rotated to within ~1 rad of the new direction (mu_after=" << mu_after << ")";
}

// (KF-ortho) ORTHOKINESIS ON THE TUMBLE RATE. After eat-calibration, cap = scent/eat_scent is a
// TRUE proximity signal (→1 in the eating range, low far away). tumble_level_gain then SHORTENS runs
// near food (fine sampling at the close) and SCALES with proximity (inert far), so the long approach
// runs are preserved. (Note: cap is only true proximity AFTER the first eat — before it, cap
// bootstraps off the running scent peak and reads ~1 at any constant scent. The bug eats early, so
// this holds in practice; the test eat-calibrates first.)
namespace {
// eat-calibrate at scent 5.0, then measure the tumble fraction at a held scent `smax`.
double ortho_tumble_frac(double level_gain, float smax) {
    Fix f({{"tumble_level_gain", level_gain}, {"master_seed", int64_t{5}}, {"tumble_base", 0.1}});
    uint64_t t = 0;
    for (int i = 0; i < 60; ++i) f.run(t++, 5.0f);      // warm the scale
    f.run_eat(t++, 5.0f);                                // EAT at 5.0 → eat_scent_=5.0 (cap now absolute)
    for (int i = 0; i < 120; ++i) f.run(t++, smax);      // settle at the test scent
    int r0 = f.nav.run_count(), tu0 = f.nav.tumble_count();
    for (int i = 0; i < 400; ++i) f.run(t++, smax);      // measurement window
    int runs = f.nav.run_count() - r0, tum = f.nav.tumble_count() - tu0;
    return double(tum) / double(runs + tum);
}
}  // namespace
TEST(RunTumbleNav, LevelGainShortensRunsNearFood) {
    double near_off = ortho_tumble_frac(0.0, 5.0f);   // in the eating range (cap→1), no level gain
    double near_on  = ortho_tumble_frac(3.0, 5.0f);   // in the eating range, level gain → shorter runs
    double far_on   = ortho_tumble_frac(3.0, 0.5f);   // far (cap≈0.1), level gain nearly inert
    EXPECT_GT(near_on, near_off + 0.15)
        << "level_gain at high scent SHORTENS runs (higher tumble fraction) "
           "(near_on=" << near_on << " near_off=" << near_off << ")";
    EXPECT_GT(near_on, far_on + 0.15)
        << "the shortening SCALES with proximity — much stronger near food than far "
           "(near_on=" << near_on << " far_on=" << far_on << ")";
}

// (5) Output is a unit-ish egocentric bearing toward the committed run direction.
TEST(RunTumbleNav, ForwardWhenAligned) {
    Fix f({{"master_seed", int64_t{7}}});
    f.run(0, 1.0f, /*heading=*/0.0f);  // run_dir initialises to heading → aligned, low p_tumble
    auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(f.bus.last_value("percept.runtumble_heading"));
    ASSERT_NE(o, nullptr);
    ASSERT_EQ(o->values.size(), 2u);
    // [vx, vy] = [sin(delta), cos(delta)] is a unit vector
    float mag = std::sqrt(o->values[0] * o->values[0] + o->values[1] * o->values[1]);
    EXPECT_NEAR(mag, 1.0f, 1e-3f) << "output bearing must be a unit vector";
    if (f.nav.last_action() == 0) {  // a RUN keeps the initial (aligned) direction → forward
        EXPECT_NEAR(o->values[0], 0.0f, 1e-3f);  // vx ~ 0
        EXPECT_NEAR(o->values[1], 1.0f, 1e-3f);  // vy ~ 1 (forward)
    }
}
