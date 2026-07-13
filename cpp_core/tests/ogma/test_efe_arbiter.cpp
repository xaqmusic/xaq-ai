// EFEArbiter — Cell L2 active-inference policy selection over the two competent nav loops.
// ASYMMETRIC normalisation: klino (raw = hunger×scent) is a Z-SCORE (SD above its OWN running
// baseline = "excitement"; smelling food is an EVENT), while planner (raw = food-route value)
// is a SUSTAINED LEVEL = raw_planner / plan_peak ∈ [0,1] (a plan is valid for as long as it
// routes to food — a steady-state property; the level NEVER goes negative). Then winner-take-all
// with an ADAPTIVE hysteresis margin = hysteresis_k · running_std(v_klino − v_planner). Output is
// a hard gain 1.0 (winner) / 0.0 (loser) on arbiter.gain.klino / arbiter.gain.planner.
// force_policy ships the ablation controls.
//
// THE FIX (route-hold): the OLD z-score on the planner FALSELY interrupted a steady route —
// a sustained-high raw_planner pulled its running mean up so v_planner decayed toward 0 and
// dipped NEGATIVE mid-route; a blind klino (z≈0) then read "higher" and broke the route. As a
// LEVEL, v_planner stays ~1 the whole route and never goes negative, so a blind klino cannot
// overtake it — only a real scent z-spike (klino smelling food) can.
#include <gtest/gtest.h>
#include "ogma/modules/EFEArbiter.hpp"
#include "ogma/modules/MotorBus.hpp"
#include "ogma/InProcessBus.hpp"

#include <cmath>
#include <memory>

namespace {
std::shared_ptr<ogma::ProprioToken> p1(float v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p;
}
float gain_on(ogma::InProcessBus& bus, char const* topic) {
    auto pt = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value(topic));
    return (pt && pt->values.size() > 0) ? float(pt->values[0]) : -1.0f;
}

struct Fix {
    ogma::InProcessBus bus;
    ogma::EFEArbiter arb;
    Fix(ogma::ParamMap p = {}) { arb.set_id("arbiter"); arb.on_setup(&bus, p); }
    void run(uint64_t t, float hunger, float scent, float plan_value) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.hunger",        p1(hunger));
        bus.publish("reality.proprio.scent_max",     p1(scent));
        bus.publish("reality.cognitive.plan_value",  p1(plan_value));
        arb.tick(t);
        bus.end_tick();
    }
    // (i) variant: also publish klino's self-reported capability on the confidence topic.
    void run_cap(uint64_t t, float hunger, float scent, float plan_value, float cap) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.hunger",        p1(hunger));
        bus.publish("reality.proprio.scent_max",     p1(scent));
        bus.publish("reality.cognitive.plan_value",  p1(plan_value));
        bus.publish("percept.klino_confidence",      p1(cap));
        arb.tick(t);
        bus.end_tick();
    }
};
}  // namespace

// (1) klino gets EXCITED and wins when scent RISES above its baseline: the near-food
// CLOSER should take over the instant it smells food (raw_klino spikes above its own
// running mean → z-score spikes positive) while the planner sits flat at its baseline.
TEST(EFEArbiter, KlinoWinsWhenScentRisesAboveBaseline) {
    Fix f;
    uint64_t t = 0;
    // establish a LOW-scent baseline for both channels (planner incumbent-ish, klino quiet).
    for (int i = 0; i < 120; ++i) f.run(t++, /*hunger=*/0.9f, /*scent=*/0.05f, /*plan_value=*/0.10f);
    // SMELL FOOD: scent rises sharply above klino's baseline; planner value holds flat.
    int klino_wins = 0;
    for (int i = 0; i < 60; ++i) {
        f.run(t++, 0.9f, 5.0f, 0.10f);
        if (f.arb.winner() == 0) ++klino_wins;
    }
    EXPECT_GT(f.arb.v_klino(), f.arb.v_planner())
        << "klino z-score must spike above the planner's when scent rises above its baseline "
        << "(v_klino=" << f.arb.v_klino() << " v_planner=" << f.arb.v_planner() << ")";
    EXPECT_GT(klino_wins, 30) << "klino should win the majority of the scent-spike window";
    EXPECT_EQ(f.arb.winner(), 0) << "klino (0) drives when it smells food";
    EXPECT_FLOAT_EQ(f.arb.gain_klino(), 1.0f);
    EXPECT_FLOAT_EQ(f.arb.gain_planner(), 0.0f);
}

// (2) planner wins when a FOOD ROUTE is present while scent is flat/dead: the far-field router
// drives once it has a committed route to remembered food. v_planner is a LEVEL → it sits at ~1
// while the route persists. (klino is quiet throughout so the gap is clean — no inflated margin.)
TEST(EFEArbiter, PlannerWinsOnFoodRouteWhileScentDead) {
    Fix f;
    uint64_t t = 0;
    // warm: a food route is present (pv held), scent dead → klino's startup z-transient settles to 0
    // and the planner takes the channel. (klino is the default incumbent, so it owns the first few
    // ticks while its z-baseline settles — the route then cleanly wins, by design.)
    for (int i = 0; i < 200; ++i) f.run(t++, 0.9f, 0.05f, 1.5f);
    // steady state: planner OWNS the food-route window every tick while scent stays dead.
    int planner_wins = 0;
    for (int i = 0; i < 200; ++i) {
        f.run(t++, 0.9f, 0.05f, 1.5f);
        if (f.arb.winner() == 1) ++planner_wins;
    }
    EXPECT_GT(f.arb.v_planner(), 0.9f)
        << "v_planner is a LEVEL → ~1 while routing to food (v_planner=" << f.arb.v_planner() << ")";
    EXPECT_GT(f.arb.v_planner(), f.arb.v_klino())
        << "the routing planner's level beats a dead klino's z-score";
    EXPECT_EQ(planner_wins, 200) << "planner owns the steady-state food-route window while scent is dead";
    EXPECT_EQ(f.arb.winner(), 1) << "planner (1) drives when a food route is present and scent is dead";
    EXPECT_FLOAT_EQ(f.arb.gain_planner(), 1.0f);
    EXPECT_FLOAT_EQ(f.arb.gain_klino(), 0.0f);
}

// (2b) THE FIX (route-hold): a SUSTAINED food route is NOT falsely interrupted by a BLIND klino.
// The planner owns a long route (raw_planner held high, scent≈0 → klino z≈0); v_planner must stay
// ~1 (never decay/go negative as the running mean would have under the OLD z-score) and the planner
// must HOLD the win the WHOLE route. A real scent z-spike (klino smelling food) must still take over
// (clean hand-off to close). This is the operator-observed scenario the fix targets.
TEST(EFEArbiter, SustainedRouteNotInterruptedByBlindKlinoButYieldsToSmell) {
    Fix f;
    uint64_t t = 0;
    // planner owns a food route cleanly (klino blind throughout → clean gap, planner is incumbent).
    for (int i = 0; i < 200; ++i) f.run(t++, 0.9f, 0.02f, 1.2f);
    ASSERT_EQ(f.arb.winner(), 1) << "planner owns the route";

    // LONG sustained route with a BLIND klino (scent≈0). Under the OLD z-score this is exactly when
    // raw_planner's mean catches up, v_planner dips negative, and the blind klino (z≈0) crossed it
    // and broke the route. With the LEVEL it must NEVER happen — the planner holds EVERY tick.
    int planner_holds = 0; float min_vp = 1e9f;
    for (int i = 0; i < 3000; ++i) {
        f.run(t++, 0.9f, 0.02f, 1.2f);           // route held high, klino blind
        if (f.arb.winner() == 1) ++planner_holds;
        min_vp = std::min(min_vp, f.arb.v_planner());
    }
    EXPECT_EQ(planner_holds, 3000)
        << "the planner must HOLD the route every tick a blind klino is present (held "
        << planner_holds << "/3000)";
    EXPECT_GT(min_vp, 0.0f)
        << "v_planner must NEVER go negative over a sustained route (min=" << min_vp << ")";
    EXPECT_GT(f.arb.v_planner(), 0.5f)
        << "v_planner stays a healthy LEVEL through the whole route (not decayed): "
        << f.arb.v_planner();

    // NOW klino SMELLS food (a real scent z-spike): it must take over to close (clean hand-off).
    int klino_takes = 0;
    for (int i = 0; i < 60; ++i) {
        f.run(t++, 0.9f, 8.0f, 1.2f);            // route still high, but klino smells food
        if (f.arb.winner() == 0) ++klino_takes;
    }
    EXPECT_GT(f.arb.v_klino(), f.arb.v_planner())
        << "a real scent z-spike exceeds the route level (hand-off to close): v_klino="
        << f.arb.v_klino() << " v_planner=" << f.arb.v_planner();
    EXPECT_EQ(f.arb.winner(), 0) << "klino takes over to close when it genuinely smells food";
    EXPECT_GT(klino_takes, 40) << "the hand-off happens once scent rises";
}

// PLANNER CEDES to strong direct scent at the REALISTIC field scale (2026-06-30 operator obs).
// klino's level is ABSOLUTE (hunger·scent, capped by the real field max ≈0.7) while v_planner is
// self-normalised to ≈1 — so a SATURATED route out-scaled klino even at max scent and the two
// oscillated near food (bug turned away before the eat). The precision-weighting cede
// (v_planner ×= 1−clamp(hunger·scent)) must let a strongly-smelt hungry bug suppress the map and
// HOLD the close, using a realistic scent (0.75), not the >1 extreme the hand-off test above uses.
TEST(EFEArbiter, PlannerCedesToStrongDirectScentAtFieldScale) {
    Fix f;
    uint64_t t = 0;
    // saturate a food route while klino is blind → planner owns it, v_planner ≈ 1, no cede.
    for (int i = 0; i < 200; ++i) f.run(t++, 0.9f, 0.02f, 5.0f);
    ASSERT_EQ(f.arb.winner(), 1);
    ASSERT_GT(f.arb.v_planner(), 0.9f) << "blind klino → v_planner saturated, un-ceded";
    // bug now RIGHT AT food: strong (but realistic ≤1) scent + hungry, route still published high.
    int klino_holds = 0;
    for (int i = 0; i < 60; ++i) {
        f.run(t++, 1.0f, 0.75f, 5.0f);
        if (f.arb.winner() == 0) ++klino_holds;
    }
    EXPECT_GT(klino_holds, 55) << "strong realistic scent CEDES the planner → klino holds the close ("
                              << klino_holds << "/60)";
    EXPECT_LT(f.arb.v_planner(), 0.4f) << "v_planner suppressed ~×(1−0.75) by the cede (v_planner="
                                      << f.arb.v_planner() << ")";
}

// (3) Hysteresis suppresses CHATTER on a noisy near-tie, but a CLEAR lead still flips. The
// margin = hysteresis_k · running_std(gap) adapts to the gap's own scale, so a noisy regime
// that crosses the tie stays committed (k=1) instead of flipping (k=0) — commitment is a
// property of the dynamics, not a magic dwell-count. Here klino's z jitters around the
// planner's LEVEL (a noisy near-tie), then klino smells food DECISIVELY (z-spike clears it).
namespace {
// Deterministic small noise in [-0.5,0.5] (reproducible, no <random> needed).
float det_noise(int i) { unsigned x = (unsigned)(i * 2654435761u); return ((x >> 9) & 1023) / 1023.0f - 0.5f; }
// Drive a NOISY near-tie: klino's absolute-proximity level (≈hunger·scent) and the planner's route
// level both jitter — ANTI-CORRELATED, so the gap (v_klino − v_planner) swings back and forth across
// zero and the winner chatters without hysteresis. Then a DECISIVE klino lead (strong steady scent →
// klino's level ≈1 far above a weak planner). For a given hysteresis_k, returns {flips during the
// noisy tie, klino-win fraction over the decisive smell window}.
struct ChatterResult { int flips; float decisive_klino_frac; };
ChatterResult chatter_arm(float k, float amp) {
    Fix f({{"hysteresis_k", double(k)}});
    uint64_t t = 0;
    // warm: establish a planner peak so v_planner is a fraction of it (not pinned to 1).
    for (int i = 0; i < 40; ++i) f.run(t++, 0.9f, 0.4f, 0.8f);
    int flips = 0, prev = f.arb.winner();
    for (int i = 0; i < 400; ++i) {
        float n = det_noise((i + 1) * 7);
        float scent = 0.55f + amp * n;            // klino level up when n>0
        float pv    = 0.55f - amp * n;            // planner level up when n<0 (anti-correlated → crossings)
        f.run(t++, 0.9f, scent, pv);
        if (f.arb.winner() != prev) { ++flips; prev = f.arb.winner(); }
    }
    // a DECISIVE klino lead must take the channel even WITH hysteresis: strong steady scent drives
    // klino's absolute level to ~1, far above the weak planner → klino owns the close.
    int klino_wins = 0;
    for (int i = 0; i < 400; ++i) {
        f.run(t++, 0.9f, 1.2f, 0.15f);
        if (f.arb.winner() == 0) ++klino_wins;
    }
    return {flips, klino_wins / 400.0f};
}
}  // namespace

TEST(EFEArbiter, HysteresisSuppressesChatterButYieldsToClearLead) {
    ChatterResult with_hyst = chatter_arm(/*k=*/1.0f, /*amp=*/0.5f);
    ChatterResult no_hyst   = chatter_arm(/*k=*/0.0f, /*amp=*/0.5f);
    EXPECT_LT(with_hyst.flips, no_hyst.flips)
        << "hysteresis must reduce chatter on a noisy near-tie (k=1 flips=" << with_hyst.flips
        << " vs k=0 flips=" << no_hyst.flips << ")";
    // with hysteresis the gap must cross the ADAPTIVE margin (not just zero) to flip → far fewer
    // flips than the bare zero-crossing count (no_hyst). ~5x fewer here over a 400-tick jitter.
    EXPECT_LT(with_hyst.flips * 3, no_hyst.flips)
        << "hysteresis cuts chatter several-fold (k=1 flips=" << with_hyst.flips
        << " vs k=0 flips=" << no_hyst.flips << ")";
    EXPECT_GT(no_hyst.flips, 100)
        << "without hysteresis the winner chatters on every noisy zero-crossing";
    // both still yield to a decisive lead — hysteresis is anti-chatter, not a lock. With
    // hysteresis klino OWNS the decisive smell window; without it klino still leads but the
    // unsuppressed chatter dilutes its share.
    EXPECT_GT(with_hyst.decisive_klino_frac, 0.9f)
        << "a decisive smell is taken even with hysteresis (klino frac="
        << with_hyst.decisive_klino_frac << ")";
    EXPECT_GT(no_hyst.decisive_klino_frac, 0.5f)
        << "klino still leads the decisive smell window without hysteresis";
}

// (4) klino's z-score is SCALE-FREE and FLAT-SIGNAL-STABLE, and the planner's LEVEL is bounded:
// a tiny klino that RISES above its own baseline beats a flat planner regardless of raw units;
// a FLAT klino signal does NOT blow up its z-score (std_eps floors the denominator); and the
// planner level is ALWAYS in [0,1] by construction (the clamp), never exploding or negative.
TEST(EFEArbiter, KlinoZScoreScaleFreePlannerLevelBounded) {
    // (a) a TINY-scale klino that rises sharply above its own baseline outscores the flat planner
    // LEVEL (=1) — the z-score is scale-free, so the raw magnitude (here a tiny 0.002→0.2 jump) is
    // irrelevant; only the rise relative to klino's own baseline matters, and a sharp rise spikes
    // the z-score well above the planner's bounded level.
    {
        Fix f;
        uint64_t t = 0;
        // klino sits at a tiny baseline; planner flat at a much LARGER raw level (route held → level 1).
        for (int i = 0; i < 200; ++i) f.run(t++, /*hunger=*/0.9f, /*scent=*/0.002f, /*plan_value=*/5.0f);
        // klino's tiny raw jumps sharply above ITS baseline (0.002 → 0.2) — a real excitement spike.
        f.run(t++, 0.9f, 0.2f, 5.0f);
        EXPECT_GT(f.arb.v_klino(), f.arb.v_planner())
            << "a tiny klino rising above its baseline beats the flat planner level (scale-free): "
            << "v_klino=" << f.arb.v_klino() << " v_planner=" << f.arb.v_planner();
    }
    // (b) a perfectly FLAT klino signal must not produce an exploding z-score (std_eps floor),
    // and the planner LEVEL is bounded [0,1] by construction.
    {
        Fix f;
        uint64_t t = 0;
        for (int i = 0; i < 500; ++i) f.run(t++, 0.9f, 1.0f, 1.0f);   // dead-flat both channels
        EXPECT_LT(std::fabs(f.arb.v_klino()), 5.0f)
            << "a flat klino signal must not blow up its z-score (v_klino=" << f.arb.v_klino() << ")";
        EXPECT_GE(f.arb.v_planner(), 0.0f) << "the planner level is never negative";
        EXPECT_LE(f.arb.v_planner(), 1.0f) << "the planner level is bounded at 1.0 (clamp)";
    }
}

// (5) The published gains are ALWAYS a valid 0/1 partition: exactly one channel at 1.0,
// the other at 0.0, every tick — basic channel muting (winner-take-all).
TEST(EFEArbiter, GainsAreValid01Partition) {
    Fix f;
    uint64_t t = 0;
    // sweep through regimes so both winners occur
    for (int i = 0; i < 400; ++i) {
        float scent = (i / 100) % 2 == 0 ? 5.0f : 0.001f;
        float pv    = (i / 100) % 2 == 0 ? 0.01f : 2.0f;
        f.run(t++, 0.8f, scent, pv);
        float gk = f.arb.gain_klino(), gp = f.arb.gain_planner();
        ASSERT_TRUE((gk == 1.0f && gp == 0.0f) || (gk == 0.0f && gp == 1.0f))
            << "gains must be a hard 0/1 partition (gk=" << gk << " gp=" << gp << ")";
        ASSERT_FLOAT_EQ(gk + gp, 1.0f) << "exactly one winner";
        // and the published bus topics match the accessors
        EXPECT_FLOAT_EQ(gain_on(f.bus, "arbiter.gain.klino"), gk);
        EXPECT_FLOAT_EQ(gain_on(f.bus, "arbiter.gain.planner"), gp);
    }
}

// (6) force_policy=klino → always klino regardless of values.
TEST(EFEArbiter, ForcePolicyKlino) {
    Fix f({{"force_policy", std::string("klino")}});
    uint64_t t = 0;
    for (int i = 0; i < 100; ++i) f.run(t++, 0.1f, 0.001f, /*plan_value=*/10.0f);  // planner SHOULD dominate
    EXPECT_EQ(f.arb.winner(), 0) << "force_policy=klino must pin the winner to klino";
    EXPECT_FLOAT_EQ(f.arb.gain_klino(), 1.0f);
    EXPECT_FLOAT_EQ(f.arb.gain_planner(), 0.0f);
}

// (7) force_policy=planner → always planner regardless of values.
TEST(EFEArbiter, ForcePolicyPlanner) {
    Fix f({{"force_policy", std::string("planner")}});
    uint64_t t = 0;
    for (int i = 0; i < 100; ++i) f.run(t++, 0.9f, 10.0f, /*plan_value=*/0.0f);  // klino SHOULD dominate
    EXPECT_EQ(f.arb.winner(), 1) << "force_policy=planner must pin the winner to planner";
    EXPECT_FLOAT_EQ(f.arb.gain_planner(), 1.0f);
    EXPECT_FLOAT_EQ(f.arb.gain_klino(), 0.0f);
}

// (8) force_policy=shuffle → a random winner each tick (the FAIR null). Both winners
// occur over a run; the partition is still valid. Reproducible per master_seed.
TEST(EFEArbiter, ForcePolicyShuffleIsRandomButValid) {
    Fix f({{"force_policy", std::string("shuffle")}, {"master_seed", int64_t{7}}});
    uint64_t t = 0;
    int klino_wins = 0, planner_wins = 0;
    for (int i = 0; i < 400; ++i) {
        f.run(t++, 0.5f, 1.0f, 0.5f);
        if (f.arb.winner() == 0) ++klino_wins; else ++planner_wins;
        float gk = f.arb.gain_klino(), gp = f.arb.gain_planner();
        ASSERT_FLOAT_EQ(gk + gp, 1.0f) << "shuffle still emits a valid 0/1 partition";
    }
    EXPECT_GT(klino_wins, 50)   << "shuffle should pick klino sometimes";
    EXPECT_GT(planner_wins, 50) << "shuffle should pick planner sometimes";

    // determinism: same seed + same tick stream → identical winner sequence
    Fix g({{"force_policy", std::string("shuffle")}, {"master_seed", int64_t{7}}});
    uint64_t tg = 0; bool same = true;
    Fix f2({{"force_policy", std::string("shuffle")}, {"master_seed", int64_t{7}}});
    uint64_t t2 = 0;
    for (int i = 0; i < 100; ++i) {
        g.run(tg++, 0.5f, 1.0f, 0.5f);
        f2.run(t2++, 0.5f, 1.0f, 0.5f);
        if (g.arb.winner() != f2.arb.winner()) same = false;
    }
    EXPECT_TRUE(same) << "shuffle must be reproducible for a fixed master_seed (tick-varied RNG)";
}

// (9) (i) CAPABILITY is a BOOST, not a SILENCE gate. The old multiplicative gate (v_klino=cap×z-score)
// zeroed a BLIND klino and collapsed foraging (eats 31->5 @100%, 20->0 @50%, n=5 seeds) — klino's blind
// run-and-tumble IS the forager and the planner cannot close. cap now enters as a MAX'd proximity level
// (clamp(hunger·cap)), so it can only RAISE v_klino near food, never lower it. Regression guard: a BLIND
// report (cap=0) on a real scent SPIKE must leave the z-score excitement fully intact (the forager lives).
TEST(EFEArbiter, CapabilityBoostDoesNotSilenceBlindForager) {
    Fix f;
    uint64_t t = 0;
    for (int i = 0; i < 120; ++i) f.run_cap(t++, 0.9f, 0.05f, 0.10f, 1.0f);   // settle low baselines
    f.run_cap(t++, 0.9f, 5.0f, 0.10f, /*cap=*/0.0f);   // scent spikes; klino reports cap=0 (blind self-report)
    EXPECT_FLOAT_EQ(f.arb.cap_klino(), 0.0f);          // reported (and used as a boost, here inert at cap=0)
    EXPECT_GT(f.arb.v_klino(), 0.5f)                   // NOT silenced — the z-spike excitement stands
        << "cap=0 must not zero v_klino (the boost never silences the forager): v_klino=" << f.arb.v_klino();
}

// (9b) EAT-CALIBRATED CLOSE: at the REAL eat scent (~0.55, below the 0.73 closest-approach cap of the
// source-normalised field), the UNCALIBRATED klino level (~0.5) LOSES to the ceded planner (~0.5) — the
// operator-observed oscillation where the planner re-wins and turns the bug away before the eat. klino's
// EAT-CALIBRATED confidence (cap→1 in its own eating range) lifts its level to ≈hunger, decisively
// clearing the ceded planner → klino OWNS the close. Two arms isolate the calibration as the cause.
TEST(EFEArbiter, EatCalibratedLevelWinsTheCloseAtRealEatScent) {
    // arm A — NO calibration (cap unwired): klino's bare 0.73-capped level barely loses at scent 0.55.
    {
        Fix f;
        uint64_t t = 0;
        for (int i = 0; i < 200; ++i) f.run(t++, 0.9f, 0.02f, 5.0f);      // planner owns a saturated route, klino blind
        ASSERT_EQ(f.arb.winner(), 1);
        for (int i = 0; i < 250; ++i) f.run(t++, 0.9f, 0.55f, 5.0f);      // sit at food at the real eat scent (z-spike settles)
        // Once the approach z-spike settles, klino's LEVEL alone (~0.5) cannot SUSTAIN a lead over the
        // ceded planner (~0.5) — the value race favors the planner, so the lead is fragile and the two
        // oscillate near food (the winner here is only held by hysteresis from the spike, not by value).
        EXPECT_LT(f.arb.v_klino(), f.arb.v_planner())
            << "uncalibrated: klino's ~0.5 level loses to the ceded planner at the real eat-scent "
            << "(v_klino=" << f.arb.v_klino() << " v_planner=" << f.arb.v_planner() << ")";
    }
    // arm B — WITH eat-calibrated confidence (cap≈1 in its eating range): klino's level is lifted and WINS.
    {
        Fix f;
        uint64_t t = 0;
        for (int i = 0; i < 200; ++i) f.run_cap(t++, 0.9f, 0.02f, 5.0f, /*cap=*/0.0f);   // blind: cap 0, planner owns route
        ASSERT_EQ(f.arb.winner(), 1);
        int klino_holds = 0;
        for (int i = 0; i < 250; ++i) {
            f.run_cap(t++, 0.9f, 0.55f, 5.0f, /*cap=*/1.0f);   // at food: klino is in its own eating range → cap≈1
            if (f.arb.winner() == 0) ++klino_holds;
        }
        EXPECT_GT(f.arb.v_klino(), f.arb.v_planner())
            << "calibrated: klino's level lifted to ≈hunger clears the ceded planner "
            << "(v_klino=" << f.arb.v_klino() << " v_planner=" << f.arb.v_planner() << ")";
        EXPECT_EQ(f.arb.winner(), 0) << "calibrated → klino wins the close (oscillation fixed)";
        EXPECT_GT(klino_holds, 200) << "and HOLDS it for the whole close (" << klino_holds << "/250)";
    }
}

// (10) Default cap (topic ABSENT) is 1.0 — behavior unchanged when not wired (the gate is
// opt-in). The plain run() never publishes the confidence topic, so cap stays 1.0.
TEST(EFEArbiter, CapabilityDefaultsToOneWhenTopicAbsent) {
    Fix f;
    uint64_t t = 0;
    for (int i = 0; i < 100; ++i) f.run(t++, 0.9f, 1.0f, 0.5f);  // never publishes confidence
    EXPECT_FLOAT_EQ(f.arb.cap_klino(), 1.0f)
        << "no confidence topic wired → cap_klino defaults to 1.0 (gate is a no-op, behavior unchanged)";
}

// =============================================================================
// EXPLICIT-EFE SCORING (scoring_mode="efe") — the precision refactor (doctrine §2.2/§2.3).
// Each policy scored G = pragmatic (hunger·reach-prob, SHARED UNITS) + epistemic
// ((1−hunger)·uncertainty-reduction). Shared units remove the klino/planner scale mismatch
// BY CONSTRUCTION: near food cap→1 ⇒ g_prag_klino→hunger, which exceeds the planner's
// DISCOUNTED route value hunger·γ^hops to a remembered (non-adjacent) cache → klino owns
// the close with NO cede, NO plan_peak self-norm, NO max-of-three.
// =============================================================================

// (E1) Stage 0 wiring: efe mode runs, emits a valid 0/1 partition every tick, exposes a
// self-consistent four-term decomposition (G = g_prag + g_epist), and PRESERVES THE BLIND
// FORAGER — a fully blind klino with no route leaves klino (the incumbent forager) driving.
// Silencing the blind forager is the historical trap (eats 31→5 @100%, 20→0 @50%).
TEST(EFEArbiter, EfeModeWiresDecomposesAndKeepsBlindForager) {
    Fix f({{"scoring_mode", std::string("efe")}});
    EXPECT_EQ(f.arb.scoring_mode(), "efe");
    uint64_t t = 0;
    for (int i = 0; i < 200; ++i) {
        f.run(t++, /*hunger=*/0.9f, /*scent=*/0.02f, /*plan_value=*/0.0f);   // blind, no route
        float gk = f.arb.gain_klino(), gp = f.arb.gain_planner();
        ASSERT_FLOAT_EQ(gk + gp, 1.0f) << "efe emits a valid 0/1 partition every tick";
        ASSERT_NEAR(f.arb.G_klino(),   f.arb.g_prag_klino()   + f.arb.g_epist_klino(),   1e-6f);
        ASSERT_NEAR(f.arb.G_planner(), f.arb.g_prag_planner() + f.arb.g_epist_planner(), 1e-6f);
    }
    EXPECT_EQ(f.arb.winner(), 0)
        << "blind klino + no route → the forager (klino, the incumbent) keeps the channel";
    EXPECT_LT(f.arb.G_planner(), 0.1f) << "the planner has nothing to route toward (G_planner≈0)";
}

// (E2) THE CLOSE, shared units (Stage 1): near food klino's eat-calibrated capability (cap→1)
// makes g_prag_klino→hunger, exceeding the planner's DISCOUNTED route value (hunger·0.6<hunger)
// to a REMEMBERED cache → klino owns the close DECISIVELY, with NO cede in the efe path. This
// is the operator-observed oscillation, fixed at its root (scale incommensurability).
TEST(EFEArbiter, EfeKlinoOwnsCloseBySharedUnitsNoCede) {
    Fix f({{"scoring_mode", std::string("efe")}});
    uint64_t t = 0;
    // planner routes to a DISTANT remembered cache (discounted value 0.6<1) while klino is blind.
    for (int i = 0; i < 200; ++i) f.run_cap(t++, /*hunger=*/0.9f, /*scent=*/0.02f, /*plan_value=*/0.6f, /*cap=*/0.0f);
    ASSERT_EQ(f.arb.winner(), 1) << "blind klino + a routed planner → planner drives (G_planner≈hunger·0.6)";
    // bug now AT food: klino in its own eating range (cap≈1), realistic sub-1 scent, route still published.
    int klino_holds = 0;
    for (int i = 0; i < 250; ++i) {
        f.run_cap(t++, 0.9f, 0.55f, 0.6f, /*cap=*/1.0f);
        if (f.arb.winner() == 0) ++klino_holds;
    }
    EXPECT_GT(f.arb.g_prag_klino(), f.arb.g_prag_planner())
        << "shared units: g_prag_klino=hunger·1 (" << f.arb.g_prag_klino() << ") > g_prag_planner=hunger·0.6 ("
        << f.arb.g_prag_planner() << ")";
    EXPECT_GT(f.arb.G_klino(), f.arb.G_planner())
        << "klino owns the close on pragmatic value alone (G_klino=" << f.arb.G_klino()
        << " G_planner=" << f.arb.G_planner() << ")";
    EXPECT_EQ(f.arb.winner(), 0) << "klino wins the close — no cede, no peak, no max-of-three";
    EXPECT_GT(klino_holds, 200) << "and HOLDS it for the whole close (" << klino_holds << "/250)";
}

// (E3) ROUTE-HOLD without the cede or plan_peak (Stage 2 commensurability): a SUSTAINED
// discounted route (plan_value=0.7<1, blind klino) keeps G_planner=hunger·0.7 a STABLE
// POSITIVE level that NEVER dips — the false-interruption (negative dip) the value-race
// plan_peak/cede existed to prevent is impossible in efe BY CONSTRUCTION (G_planner is a
// direct product ≥0). A real hungry approach (cap→1) then hands off to klino cleanly.
TEST(EFEArbiter, EfeRouteHoldIsStableThenYieldsToApproach) {
    Fix f({{"scoring_mode", std::string("efe")}});
    uint64_t t = 0;
    for (int i = 0; i < 200; ++i) f.run_cap(t++, /*hunger=*/0.9f, /*scent=*/0.02f, /*plan_value=*/0.7f, /*cap=*/0.0f);
    ASSERT_EQ(f.arb.winner(), 1) << "blind klino + a discounted route → planner drives";
    int planner_holds = 0; float min_gp = 1e9f, min_gap = 1e9f;
    for (int i = 0; i < 3000; ++i) {
        f.run_cap(t++, 0.9f, 0.02f, 0.7f, /*cap=*/0.0f);
        if (f.arb.winner() == 1) ++planner_holds;
        min_gp  = std::min(min_gp,  f.arb.G_planner());
        min_gap = std::min(min_gap, f.arb.G_planner() - f.arb.G_klino());
    }
    EXPECT_EQ(planner_holds, 3000) << "the route holds EVERY tick over a long sustained route (no false interruption)";
    EXPECT_GT(min_gp, 0.5f)  << "G_planner is a stable positive LEVEL (min=" << min_gp << "), never decays/dips";
    EXPECT_GT(min_gap, 0.0f) << "the route-hold MARGIN (G_planner−G_klino) stays strictly positive (min=" << min_gap << ")";
    // a real hungry approach in klino's eating range clears the discounted route → clean hand-off.
    int klino_takes = 0;
    for (int i = 0; i < 60; ++i) {
        f.run_cap(t++, 0.9f, 0.6f, 0.7f, /*cap=*/1.0f);
        if (f.arb.winner() == 0) ++klino_takes;
    }
    EXPECT_GT(f.arb.G_klino(), f.arb.G_planner())
        << "g_prag_klino=hunger·1 clears the discounted hunger·0.7 route (G_klino=" << f.arb.G_klino()
        << " G_planner=" << f.arb.G_planner() << ")";
    EXPECT_EQ(f.arb.winner(), 0) << "klino takes the close — no cede, no plan_peak";
    EXPECT_GT(klino_takes, 40) << "hand-off happens once klino is in its eating range";
}

// (E4) Stage 3 — the planner EPISTEMIC term: when the planner reports frontier novelty and the
// bug is SATED (hunger low → pragmatic terms small), g_epist_planner=(1−hunger)·plan_novelty lets
// the planner win to EXPLORE new ground. Ablating it (planner_epistemic=false) zeroes the term.
namespace {
void run_nov(Fix& f, uint64_t t, float hunger, float scent, float pv, float nov) {
    f.bus.begin_tick(t);
    f.bus.publish("reality.proprio.hunger",         p1(hunger));
    f.bus.publish("reality.proprio.scent_max",      p1(scent));
    f.bus.publish("reality.cognitive.plan_value",   p1(pv));
    f.bus.publish("reality.cognitive.plan_novelty", p1(nov));
    f.arb.tick(t);
    f.bus.end_tick();
}
}  // namespace
TEST(EFEArbiter, EfePlannerEpistemicDrivesSatedExploration) {
    // sated bug (hunger 0.05), no scent, no food route, planner reports HIGH frontier novelty.
    {
        Fix f({{"scoring_mode", std::string("efe")}});
        uint64_t t = 0;
        for (int i = 0; i < 50; ++i) run_nov(f, t++, /*hunger=*/0.05f, /*scent=*/0.01f, /*pv=*/0.0f, /*nov=*/0.9f);
        EXPECT_GT(f.arb.g_epist_planner(), 0.5f)
            << "sated → the planner's epistemic term engages (g_epist_planner=" << f.arb.g_epist_planner() << ")";
        EXPECT_GT(f.arb.G_planner(), f.arb.G_klino())
            << "the planner wins to explore the frontier when sated and klino is blind";
        EXPECT_EQ(f.arb.winner(), 1);
    }
    // ablation: planner_epistemic=false zeroes the term (the Stage-3 coverage A/B control).
    {
        Fix f({{"scoring_mode", std::string("efe")}, {"planner_epistemic", false}});
        uint64_t t = 0;
        for (int i = 0; i < 50; ++i) run_nov(f, t++, 0.05f, 0.01f, 0.0f, 0.9f);
        EXPECT_FLOAT_EQ(f.arb.g_epist_planner(), 0.0f) << "ablated → no planner epistemic term";
    }
    // R1 reach-gated epistemic: a hungry bug is NOT blanket-down-weighted — the gate is 1−max_reach.
    // (a) hungry + food REACHABLE → gate≈0 → epistemic low → the term does not hijack feeding (exploit).
    {
        Fix f({{"scoring_mode", std::string("efe")}});
        uint64_t t = 0;
        for (int i = 0; i < 50; ++i) run_nov(f, t++, /*hunger=*/0.95f, /*scent=*/0.9f, /*pv=*/0.0f, /*nov=*/0.9f);
        EXPECT_LT(f.arb.g_epist_planner(), 0.15f)
            << "hungry + reachable → epistemic gated down, exploit dominates (g_epist_planner="
            << f.arb.g_epist_planner() << ")";
    }
    // (b) hungry + BLIND (no scent, no route) = MAX uncertainty → epistemic STAYS HIGH so play/planner
    //     explore to FIND food (the R1 deadlock fix; the legacy 1−hunger gate silenced exactly this).
    {
        Fix f({{"scoring_mode", std::string("efe")}});
        uint64_t t = 0;
        for (int i = 0; i < 50; ++i) run_nov(f, t++, /*hunger=*/0.95f, /*scent=*/0.01f, /*pv=*/0.0f, /*nov=*/0.9f);
        EXPECT_GT(f.arb.g_epist_planner(), 0.5f)
            << "hungry + blind → epistemic stays high to find food, deadlock broken (g_epist_planner="
            << f.arb.g_epist_planner() << ")";
    }
    // legacy ablation: epistemic_reach_gated=false restores the OLD blanket (1−hunger) down-weight
    // even when blind (reproduces the deadlock — the baseline for the R1 A/B).
    {
        Fix f({{"scoring_mode", std::string("efe")}, {"epistemic_reach_gated", false}});
        uint64_t t = 0;
        for (int i = 0; i < 50; ++i) run_nov(f, t++, 0.95f, 0.01f, 0.0f, 0.9f);
        EXPECT_LT(f.arb.g_epist_planner(), 0.1f)
            << "legacy gate: hungry → epistemic down-weighted regardless of reach (the deadlock)";
    }
}

// =============================================================================
// MotorBus arbiter-gain coupling — the CONSUMER of the arbiter gains (doctrine §8).
// gain_mod_prefix wires each channel to arbiter.gain.<name>; muting a channel (gain 0)
// must zero BOTH its mix contribution AND its authority share (so its learner pauses).
// =============================================================================
namespace {
std::shared_ptr<ogma::ActionOut> act(float a, uint64_t t = 0) {
    auto o = std::make_shared<ogma::ActionOut>(); o->accel = a; o->tick_id = t; return o;
}
std::shared_ptr<ogma::ProprioToken> p1t(float v, uint64_t t) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; p->tick_id = t; return p;
}
// Two-influencer steer_thrust bus with the arbiter-gain prefix + authority publish.
ogma::ParamMap two_chan_bus_params() {
    ogma::ParamMap p;
    p["influencer_names"] = std::vector<std::string>{"klino", "planner"};
    p["kinds"]            = std::vector<std::string>{"steer_thrust", "steer_thrust"};
    p["topics_a"]         = std::vector<std::string>{"cogk.steer", "cog.steer"};
    p["topics_b"]         = std::vector<std::string>{"cogk.thrust", "cog.thrust"};
    p["gains"]            = std::vector<double>{1.0, 1.0};
    p["input_scale"]      = std::vector<double>{4.0, 4.0};
    p["authority_prefix"] = std::string("motor.bus.authority.");
    p["gain_mod_prefix"]  = std::string("arbiter.gain.");
    return p;
}
}  // namespace

TEST(MotorBusArbiterGain, MutedChannelZeroesContributionAndAuthority) {
    ogma::InProcessBus bus;
    ogma::MotorBus mb;
    mb.set_id("motor_bus");
    mb.on_setup(&bus, two_chan_bus_params());

    // both channels drive equally, klino MUTED (arbiter gain 0), planner PASSED (gain 1).
    // NOTE: MotorBus freshness-windows its inputs by tick_id → every payload must be stamped.
    for (uint64_t t = 0; t < 200; ++t) {
        bus.begin_tick(t);
        bus.publish("cogk.steer",  act(0.0f, t)); bus.publish("cogk.thrust", act(3.0f, t));  // klino thrust fwd
        bus.publish("cog.steer",   act(0.0f, t)); bus.publish("cog.thrust",  act(3.0f, t));  // planner thrust fwd
        bus.publish("arbiter.gain.klino",   p1t(0.0f, t));   // MUTE klino
        bus.publish("arbiter.gain.planner", p1t(1.0f, t));   // PASS planner
        mb.tick(t);
        bus.end_tick();
    }
    // klino index 0, planner index 1
    EXPECT_NEAR(mb.eff_gain(0), 0.0f, 1e-6f) << "muted channel effective gain → 0";
    EXPECT_GT(mb.eff_gain(1), 0.5f)          << "passed channel keeps its gain";
    EXPECT_NEAR(mb.contrib_l(0), 0.0f, 1e-6f) << "muted channel contributes nothing to L";
    EXPECT_NEAR(mb.contrib_r(0), 0.0f, 1e-6f) << "muted channel contributes nothing to R";
    EXPECT_GT(std::fabs(mb.contrib_r(1)), 0.1f) << "passed channel still drives the bus";
    EXPECT_NEAR(mb.authority(0), 0.0f, 1e-3f)
        << "muted channel authority → ~0 (its advance learning pauses)";
    EXPECT_GT(mb.authority(1), 0.9f) << "passed channel takes ~all the authority";
    // and the authority is published for the learner to read
    auto a0 = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("motor.bus.authority.klino"));
    ASSERT_NE(a0, nullptr);
    EXPECT_NEAR(float(a0->values[0]), 0.0f, 1e-3f);
}

TEST(MotorBusArbiterGain, NoArbiterGainIsBitIdenticalPass) {
    // Empty gain_mod_prefix → no arbiter coupling; effective gain == base gain (default-off).
    ogma::InProcessBus bus;
    ogma::MotorBus mb;
    ogma::ParamMap p = two_chan_bus_params();
    p["gain_mod_prefix"] = std::string("");   // disable
    mb.set_id("motor_bus");
    mb.on_setup(&bus, p);
    for (uint64_t t = 0; t < 10; ++t) {
        bus.begin_tick(t);
        bus.publish("cogk.steer", act(0.0f, t)); bus.publish("cogk.thrust", act(3.0f, t));
        bus.publish("cog.steer",  act(0.0f, t)); bus.publish("cog.thrust",  act(3.0f, t));
        // even if an arbiter gain is published, it is NOT subscribed → ignored.
        bus.publish("arbiter.gain.klino", p1t(0.0f, t));
        mb.tick(t);
        bus.end_tick();
    }
    EXPECT_NEAR(mb.eff_gain(0), 1.0f, 1e-6f) << "no prefix → base gain passes through unchanged";
    EXPECT_NEAR(mb.eff_gain(1), 1.0f, 1e-6f);
}

// =============================================================================
// task #33 — the PLAY policy (epistemic GROW), energy-surplus weighted.
// =============================================================================
namespace {
ogma::ParamMap efe_play_params() {
    return {
        {"scoring_mode",      std::string("efe")},
        {"planner_epistemic", false},                                  // the epistemic term moves to play
        {"play_value_topic",  std::string("reality.cognitive.play_value")},
        {"play_gain_topic",   std::string("arbiter.gain.play")},
        {"play_weight",       1.0},
    };
}
void run_play(ogma::InProcessBus& bus, ogma::EFEArbiter& arb, uint64_t t,
              float hunger, float scent, float plan_value, float play_value, float cap = -1.0f) {
    bus.begin_tick(t);
    bus.publish("reality.proprio.hunger",       p1(hunger));
    bus.publish("reality.proprio.scent_max",    p1(scent));
    bus.publish("reality.cognitive.plan_value", p1(plan_value));
    bus.publish("reality.cognitive.play_value", p1(play_value));
    if (cap >= 0.0f) bus.publish("percept.klino_confidence", p1(cap));
    arb.tick(t);
    bus.end_tick();
}
}  // namespace

// Default-off: with no play params the arbiter is a byte-identical 2-policy race — play never
// participates and no play_gain is published.
TEST(EFEArbiter, PlayInertByDefault) {
    Fix f;
    for (uint64_t t = 0; t < 5; ++t) f.run(t, 0.1f, 0.1f, 0.0f);
    EXPECT_FALSE(f.arb.play_active());
    EXPECT_FLOAT_EQ(f.arb.gain_play(), 0.0f);
    EXPECT_NE(f.arb.winner(), 2) << "play never wins when absent";
    EXPECT_EQ(f.bus.last_value("arbiter.gain.play"), nullptr)
        << "no play_gain publish without the topic (byte-identical output surface)";
}

// Wired but zero-weight = INERT: play is plumbed (subscribed, gain published for the MotorBus
// consumer) but never wins, so the klino/planner race is unchanged. This is the Stage-1 posture.
TEST(EFEArbiter, PlayWiredZeroWeightIsInertButPublishesZeroGain) {
    auto p = efe_play_params(); p["play_weight"] = 0.0;
    Fix f(p);
    for (uint64_t t = 0; t < 5; ++t) run_play(f.bus, f.arb, t, /*hunger=*/0.1f, 0.05f, 0.0f, /*play_value=*/0.9f);
    EXPECT_FALSE(f.arb.play_active()) << "weight 0 → inert";
    EXPECT_NE(f.arb.winner(), 2) << "zero-weight play never wins even with a high play_value";
    EXPECT_FLOAT_EQ(gain_on(f.bus, "arbiter.gain.play"), 0.0f)
        << "play_gain published but muted → the MotorBus consumer fires";
}

// FULL + novel frontier → play (GROW) wins. hunger low ⇒ energy surplus high ⇒ G_play dominates
// the weak pragmatic terms. This is the core posture: curiosity is instrumental, so explore when
// you can afford it.
TEST(EFEArbiter, PlayWinsWhenFullAndNovel) {
    Fix f(efe_play_params());
    EXPECT_TRUE(f.arb.play_active());
    // FULL (hunger 0.1), no food scent, no route, but a novel frontier (play_value 0.9).
    for (uint64_t t = 0; t < 8; ++t) run_play(f.bus, f.arb, t, /*hunger=*/0.1f, /*scent=*/0.0f, /*plan_value=*/0.0f, /*play_value=*/0.9f);
    EXPECT_EQ(f.arb.winner(), 2) << "full + novel → play GROWS the map";
    EXPECT_FLOAT_EQ(gain_on(f.bus, "arbiter.gain.play"), 1.0f);
    EXPECT_FLOAT_EQ(gain_on(f.bus, "arbiter.gain.klino"), 0.0f);
    EXPECT_GT(f.arb.G_play(), f.arb.G_klino());
}

// HUNGRY → the same novel frontier LOSES: play is down-weighted by (1−hunger) so a smelling/at-food
// klino wins. Same play_value=0.9 as the win case — it is the ENERGY gate, not the novelty, that
// flips the decision (the corrected play-when-full posture).
TEST(EFEArbiter, PlayLosesToKlinoWhenHungry) {
    Fix f(efe_play_params());
    // HUNGRY (0.95), klino in its own eating range (cap 0.9) → strong pragmatic; frontier still novel.
    for (uint64_t t = 0; t < 8; ++t) run_play(f.bus, f.arb, t, /*hunger=*/0.95f, /*scent=*/0.8f, /*plan_value=*/0.0f, /*play_value=*/0.9f, /*cap=*/0.9f);
    EXPECT_EQ(f.arb.winner(), 0) << "hungry → pragmatic klino beats play despite the novel frontier";
    EXPECT_LT(f.arb.G_play(), f.arb.G_klino()) << "energy surplus (1−hunger) suppresses play when hungry";
}

// WRONG-SIGN ablation (bar c): play_hunger_weight flips the gate to hunger, so play explores when
// HUNGRY — the pathology that proves the correct sign is play-when-FULL. Same hungry+novel scenario
// as PlayLosesToKlinoWhenHungry, but the flipped gate makes play WIN.
TEST(EFEArbiter, PlayHungerWeightAblationFlipsTheGate) {
    auto p = efe_play_params(); p["play_hunger_weight"] = true;
    Fix f(p);
    for (uint64_t t = 0; t < 8; ++t) run_play(f.bus, f.arb, t, /*hunger=*/0.9f, /*scent=*/0.0f, /*plan_value=*/0.0f, /*play_value=*/0.9f);
    EXPECT_EQ(f.arb.winner(), 2) << "wrong-sign gate → play explores when HUNGRY (the pathology)";
    EXPECT_GT(f.arb.G_play(), f.arb.G_klino());
}

// =============================================================================
// VISION policy (loop #4) — CLOSE on a SEEN source, the 4th arbiter policy.
// =============================================================================
namespace {
ogma::ParamMap efe_vision_params() {
    return {
        {"scoring_mode",       std::string("efe")},
        {"planner_epistemic",  false},
        {"play_value_topic",   std::string("reality.cognitive.play_value")},
        {"play_gain_topic",    std::string("arbiter.gain.play")},
        {"play_weight",        1.0},
        {"vision_value_topic", std::string("reality.cognitive.vision_value")},
        {"vision_gain_topic",  std::string("arbiter.gain.vision")},
        {"vision_weight",      1.0},
    };
}
void run_vision(ogma::InProcessBus& bus, ogma::EFEArbiter& arb, uint64_t t,
                float hunger, float scent, float plan_value, float play_value, float vision_value) {
    bus.begin_tick(t);
    bus.publish("reality.proprio.hunger",         p1(hunger));
    bus.publish("reality.proprio.scent_max",      p1(scent));
    bus.publish("reality.cognitive.plan_value",   p1(plan_value));
    bus.publish("reality.cognitive.play_value",   p1(play_value));
    bus.publish("reality.cognitive.vision_value", p1(vision_value));
    arb.tick(t);
    bus.end_tick();
}
}  // namespace

// Default-off: no vision params ⇒ byte-identical arbiter — vision never participates, no gain published.
TEST(EFEArbiter, VisionInertByDefault) {
    Fix f(efe_play_params());   // 3-policy (play on, vision absent)
    for (uint64_t t = 0; t < 5; ++t) run_play(f.bus, f.arb, t, 0.5f, 0.1f, 0.0f, 0.5f);
    EXPECT_FALSE(f.arb.vision_active());
    EXPECT_FLOAT_EQ(f.arb.gain_vision(), 0.0f);
    EXPECT_NE(f.arb.winner(), 3) << "vision never wins when absent";
    EXPECT_EQ(f.bus.last_value("arbiter.gain.vision"), nullptr)
        << "no vision_gain publish without the topic (byte-identical output surface)";
}

// Wired but zero-weight = INERT (the Stage-1 posture): plumbed + gain published for the MotorBus
// consumer, but never wins → the 3-policy race is unchanged.
TEST(EFEArbiter, VisionWiredZeroWeightIsInertButPublishesZeroGain) {
    auto p = efe_vision_params(); p["vision_weight"] = 0.0;
    Fix f(p);
    for (uint64_t t = 0; t < 5; ++t) run_vision(f.bus, f.arb, t, 0.1f, 0.05f, 0.0f, 0.9f, /*vision=*/0.9f);
    EXPECT_FALSE(f.arb.vision_active());
    EXPECT_NE(f.arb.winner(), 3);
    auto g = std::dynamic_pointer_cast<const ogma::ProprioToken>(f.bus.last_value("arbiter.gain.vision"));
    ASSERT_NE(g, nullptr) << "gain published (consumer fires) even when inert";
    EXPECT_FLOAT_EQ(float(g->values[0]), 0.0f);
}

// The pragmatic close: HUNGRY + food clearly SEEN (high vision_value) while scent is DEAD and no route
// → vision wins the race (the scent-blind regime the loop exists for).
TEST(EFEArbiter, VisionWinsWhenHungryAndFoodSeenWhileScentDead) {
    Fix f(efe_vision_params());
    for (uint64_t t = 0; t < 8; ++t) run_vision(f.bus, f.arb, t, /*hunger=*/0.95f, /*scent=*/0.0f,
                                                /*plan_value=*/0.0f, /*play_value=*/0.2f, /*vision=*/0.9f);
    EXPECT_EQ(f.arb.winner(), 3) << "hungry + food seen + scent blind → vision closes";
    EXPECT_FLOAT_EQ(f.arb.gain_vision(), 1.0f);
    EXPECT_GT(f.arb.G_vision(), f.arb.G_klino());
    EXPECT_GT(f.arb.G_vision(), f.arb.G_play());
}

// Occluded (vision_value 0) → vision cedes even when hungry; it cannot win on no signal.
TEST(EFEArbiter, VisionCedesWhenOccluded) {
    Fix f(efe_vision_params());
    // hungry, no scent, no route, no food in view (vision 0), but a novel frontier → play should carry.
    for (uint64_t t = 0; t < 8; ++t) run_vision(f.bus, f.arb, t, 0.2f, 0.0f, 0.0f, /*play=*/0.9f, /*vision=*/0.0f);
    EXPECT_NE(f.arb.winner(), 3) << "no food in view → vision never wins";
    EXPECT_FLOAT_EQ(f.arb.G_vision(), 0.0f);
}

// Ablation hook: force_policy=vision drives the vision channel regardless of scores.
TEST(EFEArbiter, ForcePolicyVision) {
    auto p = efe_vision_params(); p["force_policy"] = std::string("vision");
    Fix f(p);
    for (uint64_t t = 0; t < 4; ++t) run_vision(f.bus, f.arb, t, 0.1f, 0.9f, 0.9f, 0.9f, /*vision=*/0.0f);
    EXPECT_EQ(f.arb.winner(), 3);
    EXPECT_FLOAT_EQ(f.arb.gain_vision(), 1.0f);
}




