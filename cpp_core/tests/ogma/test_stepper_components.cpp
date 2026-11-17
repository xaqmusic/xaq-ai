// The stepper's Phase 2 components standing alone (learned stepper plan, 2026-11-15): SupportFloor, StepperInputs and the support
// triangle, StepExecutor and ScaffoldPlanner wired by hand -- no bus, no module.  The module's own behaviour on the golden traces is
// the split's gate; these pin each component's contract.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/modules/stepper/ScaffoldPlanner.hpp"
#include "ogma/modules/stepper/StepExecutor.hpp"
#include "ogma/modules/stepper/StepperCommon.hpp"
#include "ogma/modules/stepper/SupportFloor.hpp"
#include "stepper_fixture.hpp"
namespace {
using namespace ogma::stepper; using ogma::body::LegSolver;

// the components wired as PoseSequencer wires them, on the fixture's leg model, a flat remembered floor and a support state
struct Rig {
    StepperInputs in; LoadBook book; SupportFloor floor; LegSolver kin; uint32_t rng = 12345u;
    StepExecutor ex{in, floor, book, kin, rng};
    ScaffoldPlanner pl{in, floor, book, kin, rng, ex};
    int intents = 0;
    Rig() {
        const auto m = stepper_test::leg_model(); kin.set_model(m.data(), m.size());
        const auto pt = stepper_test::flat_patch(-0.09f); in.patch_.assign(pt.begin(), pt.end()); in.N_ = int(pt[0]); in.cell_ = pt[1]; in.have_patch_ = true;
        const auto s = stepper_test::support(0.03f); for (int i = 0; i < 26; ++i) in.support_[size_t(i)] = s[size_t(i)]; in.have_support_ = true;
        for (int i = 0; i < 4; ++i) in.contact_[size_t(i)] = 1.0f; in.have_contact_ = true;
        ex.intent_hook_ = [this] { ++intents; return pl.decide(); };
        ex.relief_hook_ = [this](int st, double cx, double cz, int b, int& l, Vec3f& L, double& mm) { return pl.plan_relief(st, cx, cz, b, l, L, mm); };
        ex.order_ref_ = &pl.order_;
        ex.engage_now(300);
        for (int i = 0; i < 4; ++i) { ex.toe_tgt_[size_t(i)] = ex.neutral_[size_t(i)]; ex.bear_[size_t(i)] = 1.0; ex.bearing_[size_t(i)] = true; }
        for (int k = 0; k < 20; ++k) ex.solve_ik();   // the commanded angles onto the stance, as the module's tick keeps them
    }
};
}

TEST(SupportFloor, ALegLiftsOnlyWithItsMargin_TheModelVetoingOnlyWhenAllFourToesBear) {
    SupportFloor f;   // margin_min 5 mm
    EXPECT_TRUE(f.lift_safe(0.006, 0.006, 4)); EXPECT_FALSE(f.lift_safe(0.004, 0.02, 4));
    EXPECT_FALSE(f.lift_safe(0.006, 0.001, 4)) << "four bearing: the model's margin vetoes";
    EXPECT_TRUE(f.lift_safe(0.006, 0.001, 3)) << "three bearing: a flickering flag makes the model's margin the floor; the geometry decides";
    StepperInputs in; EXPECT_TRUE(f.upright(in)) << "no up vector yet: upright";
    in.have_up_ = true; in.up_ = {0.0, 0.8, 0.6}; EXPECT_TRUE(f.upright(in)); in.up_ = {0.0, 0.7, 0.71}; EXPECT_FALSE(f.upright(in));
    in.have_contact_ = true; in.have_belly_ = true; in.belly_ = 0.05f;
    EXPECT_TRUE(f.standing(200, in, 3, true)); EXPECT_FALSE(f.standing(199, in, 3, true)); EXPECT_FALSE(f.standing(200, in, 2, true));
    in.belly_ = 0.03f; EXPECT_FALSE(f.standing(200, in, 4, true)); EXPECT_TRUE(f.standing(200, in, 4, false)) << "no belly topic: no belly check";
}

TEST(StepperInputs, ThePatchReadsTheRememberedGroundAndItsRelief_AndTheTriangleSignsItsInside) {
    StepperInputs in; double h = 1.0, sg = 1.0, rel = 1.0;
    EXPECT_FALSE(in.patch_h(0.0, 0.0, h)) << "no patch yet";
    const auto pt = stepper_test::flat_patch(-0.09f); in.patch_.assign(pt.begin(), pt.end()); in.N_ = int(pt[0]); in.cell_ = pt[1]; in.have_patch_ = true;
    ASSERT_TRUE(in.patch_h(0.05, -0.02, h, sg)); EXPECT_NEAR(h, -0.09, 1e-6); EXPECT_NEAR(sg, std::sqrt(1.0 / (9 * 40000.0)), 1e-9);
    ASSERT_TRUE(in.patch_edge(0.05, -0.02, rel)); EXPECT_NEAR(rel, 0.0, 1e-9);
    in.patch_[size_t(4 + 9 * 16 + 9)] = -0.03f;   // one cell 6 cm higher: a riser's edge next to it
    ASSERT_TRUE(in.patch_edge(0.0, 0.0, rel)); EXPECT_NEAR(rel, 0.06, 1e-6);
    double nx, nz; const std::array<std::array<double, 2>, 3> tri{{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}};
    EXPECT_GT(tri_margin(tri, 0.2, 0.2, nx, nz), 0.0); EXPECT_LT(tri_margin(tri, 1.0, 1.0, nx, nz), 0.0);
    EXPECT_NEAR(tri_margin(tri, 0.25, 0.1, nx, nz), 0.1, 1e-12); EXPECT_NEAR(nx, 0.0, 1e-12); EXPECT_NEAR(nz, 1.0, 1e-12) << "the nearest edge's inward normal";
}

TEST(ScaffoldPlanner, ThePoseSearchPlansTheWavesNextLeg_AFeasibleShiftAndALandingInReach) {
    Rig r; r.pl.plan_ = 1.0;
    const StepIntent in = r.pl.decide();
    ASSERT_EQ(in.kind, StepIntent::Plan); EXPECT_EQ(in.leg, 2) << "the wave starts with the rear-left";
    EXPECT_TRUE(r.pl.shift_feasible(in.leg, r.ex.toe_tgt_, in.sx, in.sz));
    const Vec3f hip = r.kin.hip1(in.leg); const double rr = std::hypot(in.land.x - hip.x, in.land.z - hip.z);
    EXPECT_GE(rr, r.kin.p.reach_min_m); EXPECT_LE(rr, r.kin.p.reach_max_m - 0.01 + 1e-6);
    EXPECT_EQ(r.pl.plans_, 1); EXPECT_EQ(r.pl.order_pos_, 0);
}

TEST(ScaffoldPlanner, WithoutTheSearchTheOrderChooses_AndAnUnloadedLegIsShiftedOverFirst) {
    Rig r;   // plan 0: the fixed order RL, FL, RR, FR
    for (int want : {2, 0, 3, 1, 2}) { const auto in = r.pl.decide(); EXPECT_EQ(in.kind, StepIntent::Choice); EXPECT_EQ(in.leg, want); }
    Rig u; u.book.unloaded_shift_m_ = 0.01; u.in.have_load_ = true;
    u.book.last_landed_ = 1; u.book.unloaded_landings_[1] = 1; u.book.land_max_load_[1] = 0.0;   // the front-right just landed, its second landing that never bore weight
    const auto in = u.pl.decide();
    ASSERT_EQ(in.kind, StepIntent::Unload); EXPECT_EQ(in.leg, 1);
    EXPECT_NEAR(std::hypot(in.sx, in.sz), 0.01, 1e-12) << "the body shifts unloaded_shift_m";
    EXPECT_GT(in.sx * u.ex.toe_tgt_[1].x, 0.0); EXPECT_GT(in.sz * u.ex.toe_tgt_[1].z, 0.0) << "toward the unloaded toe";
    EXPECT_EQ(u.book.last_landed_, -1); EXPECT_EQ(u.book.unload_cool_[1], 4 * u.book.unloaded_ticks_); EXPECT_EQ(u.pl.unload_shifts_, 1);
}

TEST(StepExecutor, ADecisionAsksTheHookOnce_ClearsThePlanAndAppliesTheIntent) {
    Rig r; r.pl.plan_ = 1.0;
    EXPECT_EQ(r.ex.phase_, Phase::Settle) << "engage starts with the settle";
    r.ex.plan_active_ = true; r.ex.plan_done_x_ = 0.3; r.ex.reloc_active_ = true;
    r.ex.decide_next(r.ex.tick_now_);
    EXPECT_EQ(r.intents, 1);
    EXPECT_EQ(r.ex.reloc_n_[6], 1) << "a step that never landed is counted abandoned";
    EXPECT_EQ(r.ex.leg_, 2); EXPECT_TRUE(r.ex.plan_active_); EXPECT_TRUE(r.ex.plan_pending_); EXPECT_EQ(r.ex.reloc_plan_leg_, 2);
    EXPECT_EQ(r.ex.plan_done_x_, 0.0) << "the walked part of the last plan is cleared";
    StepIntent c; c.kind = StepIntent::Choice; c.leg = 3; r.ex.apply(c);
    EXPECT_EQ(r.ex.leg_, 3) << "a choice sets the leg and leaves the plan's state to the decision's clearing";
}

TEST(StepExecutor, SettledItCrawls_EveryLegLandsAndTheRelocationTagCloses) {
    Rig r; r.pl.plan_ = 1.0;
    for (int t = 301; t < 3300; ++t) {
        r.ex.sense(uint64_t(t));
        // the floor at the stance's depth: a toe below the stance height touches, one above it is in the air
        for (int i = 0; i < 4; ++i) {
            const bool stepping = i == r.ex.leg_ && (r.ex.phase_ == Phase::Lift || r.ex.phase_ == Phase::Swing || r.ex.phase_ == Phase::Lower);
            r.in.contact_[size_t(i)] = (!stepping || r.ex.toe_tgt_[size_t(i)].y <= float(-r.ex.stand_h_m_ - 0.002)) ? 1.0f : 0.0f;
        }
        r.ex.step_machine(); r.ex.hold_stance(); r.ex.solve_ik();
    }
    for (int i = 0; i < 4; ++i) EXPECT_GE(r.ex.steps_[size_t(i)], 2) << "leg " << i;
    EXPECT_GT(r.ex.reloc_n_[0], 4); EXPECT_LT(r.ex.reloc_resid_, 1e-9) << "every move of a landing was tagged";
    EXPECT_LT(r.ex.ik_err_ema_, 0.015) << "the toes track their targets (moving targets lag by the step cap)";
    // the step outcome: one record per landing, the last a planned step whose corrections sum to its offset from the plan
    int total = 0; for (int i = 0; i < 4; ++i) total += r.ex.steps_[size_t(i)];
    EXPECT_EQ(r.ex.outcome_seq_, total);
    const StepOutcome& o = r.ex.outcome_;
    EXPECT_EQ(o.seq, total); EXPECT_EQ(o.kind, 1); EXPECT_TRUE(o.touched); EXPECT_GT(o.ticks, 10); EXPECT_GE(o.leg, 0);
    double mx = 0.0, mz = 0.0; for (auto const& m : o.moves) { mx += m[0]; mz += m[1]; }
    EXPECT_NEAR(double(o.land.x) - o.from.x, mx, 1e-6); EXPECT_NEAR(double(o.land.z) - o.from.z, mz, 1e-6);
    const auto f = o.floats(); ASSERT_EQ(int(f.size()), StepOutcome::kFloats); EXPECT_EQ(f[0], float(total)); EXPECT_EQ(f[6], o.land.x);
}

// THE PLAN PREDICTS ITS GATE (2026-11-17).  The executor lets a planned leg go only if SupportModel's margin agrees; the search
// rated the leg from the commanded toes alone, and a zero-shift plan the model refuses came back unchanged 68 times in 30 s.
TEST(ScaffoldPlanner, WithPlanModelMarginTheSearchAllowsForTheModelsShortfall_AndOffItIsUntouched) {
    auto margin_after = [](Rig& r, const StepIntent& in) {   // the geometric lift margin of the planned leg after the planned shift
        std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
        for (int i = 0; i < 4; ++i) if (i != in.leg && kk < 3) { t3[size_t(kk)] = {double(r.ex.toe_tgt_[size_t(i)].x) - in.sx, double(r.ex.toe_tgt_[size_t(i)].z) - in.sz}; ++kk; }
        double a, b; return tri_margin(t3, double(r.in.support_[3]), double(r.in.support_[2]), a, b);
    };
    auto geo_now = [](Rig& r, int leg) {
        std::array<std::array<double, 2>, 3> t3{}; int kk = 0;
        for (int i = 0; i < 4; ++i) if (i != leg && kk < 3) { t3[size_t(kk)] = {double(r.ex.toe_tgt_[size_t(i)].x), double(r.ex.toe_tgt_[size_t(i)].z)}; ++kk; }
        double a, b; return tri_margin(t3, double(r.in.support_[3]), double(r.in.support_[2]), a, b);
    };
    const double need = 0.005 + 0.005;
    auto rig = [&](double lever, double shortfall) {
        auto r = std::make_unique<Rig>(); r->pl.plan_ = 1.0; r->pl.plan_model_margin_ = lever;
        r->in.support_[1] = 4.0f;   // all four toes bear: the model's veto is live
        for (int i = 0; i < 4; ++i) r->in.support_[size_t(10 + 5 * i)] = float(geo_now(*r, i) - shortfall);   // the model's margin sits `shortfall` below the geometric one
        return r;
    };
    // off: the model's margin is not read by the search at all
    auto a = rig(0.0, 0.0), b = rig(0.0, 0.022);
    const StepIntent ia = a->pl.decide(), ib = b->pl.decide();
    EXPECT_EQ(ia.leg, ib.leg); EXPECT_EQ(ia.sx, ib.sx); EXPECT_EQ(ia.sz, ib.sz); EXPECT_EQ(ia.land.x, ib.land.x); EXPECT_EQ(ia.land.z, ib.land.z); EXPECT_EQ(b->pl.model_bias_n_, 0);
    ASSERT_LT(margin_after(*b, ib) - 0.022, need) << "the off plan would pass the model anyway: the case tests nothing";
    // on, the model agreeing: the same plan as off
    auto c = rig(1.0, 0.0); const StepIntent ic = c->pl.decide();
    EXPECT_EQ(ic.leg, ia.leg); EXPECT_EQ(ic.sx, ia.sx); EXPECT_EQ(ic.sz, ia.sz); EXPECT_EQ(c->pl.model_bias_n_, 0);
    // on, the model 22 mm short: the plan the gate would refuse is not issued -- whatever is planned leaves its leg the margin
    // by the MODEL's count (the geometric margin less the shortfall)
    auto d = rig(1.0, 0.022); const StepIntent id = d->pl.decide();
    EXPECT_GE(d->pl.model_bias_n_, 1);
    EXPECT_FALSE(id.kind == StepIntent::Plan && id.leg == ib.leg && id.sx == ib.sx && id.sz == ib.sz);
    if (id.kind == StepIntent::Plan) { EXPECT_GE(margin_after(*d, id) - 0.022, need - 1e-9); }
    // on, the model hopelessly short for every leg: no leg is planned on a margin the gate will refuse
    auto e = rig(1.0, 0.2); const StepIntent ie = e->pl.decide();
    EXPECT_NE(ie.kind, StepIntent::Plan); EXPECT_GE(e->pl.plan_none_, 1);
}
