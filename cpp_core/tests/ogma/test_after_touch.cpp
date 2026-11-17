// The after-touch record (learned stepper plan 9.5, 2026-11-17): the model standing alone, then the record on the stepper's
// components wired by hand -- it reads and moves nothing, every landed step gets a record closed at the next one, a toe the
// stance leaves in the air is read as not bearing, and the module carries it only when it is on.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/modules/PoseSequencer.hpp"
#include "ogma/modules/stepper/AfterTouch.hpp"
#include "ogma/modules/stepper/ScaffoldPlanner.hpp"
#include "ogma/modules/stepper/StepExecutor.hpp"
#include "ogma/modules/stepper/StepperCommon.hpp"
#include "ogma/modules/stepper/SupportFloor.hpp"
#include "stepper_fixture.hpp"
namespace {
using namespace ogma::stepper; using ogma::body::LegSolver;

struct Rig {
    StepperInputs in; LoadBook book; SupportFloor floor; LegSolver kin; uint32_t rng = 12345u;
    StepExecutor ex{in, floor, book, kin, rng};
    ScaffoldPlanner pl{in, floor, book, kin, rng, ex};
    AfterTouch at; int air_leg = -1;   // air_leg: a toe whose ground lies below the standing plane -- it touches as it lowers and loses the ground when the stance takes it
    Rig() {
        const auto m = stepper_test::leg_model(); kin.set_model(m.data(), m.size());
        const auto pt = stepper_test::flat_patch(-0.09f); in.patch_.assign(pt.begin(), pt.end()); in.N_ = int(pt[0]); in.cell_ = pt[1]; in.have_patch_ = true;
        const auto s = stepper_test::support(0.03f); for (int i = 0; i < 26; ++i) in.support_[size_t(i)] = s[size_t(i)]; in.have_support_ = true;
        for (int i = 0; i < 4; ++i) { in.contact_[size_t(i)] = 1.0f; in.load_[size_t(i)] = 0.25f; } in.have_contact_ = true; in.have_load_ = true; in.have_up_ = true;
        ex.intent_hook_ = [this] { return pl.decide(); };
        ex.relief_hook_ = [this](int st, double cx, double cz, int b, int& l, Vec3f& L, double& mm) { return pl.plan_relief(st, cx, cz, b, l, L, mm); };
        ex.order_ref_ = &pl.order_;
        ex.engage_now(300);
        for (int i = 0; i < 4; ++i) { ex.toe_tgt_[size_t(i)] = ex.neutral_[size_t(i)]; ex.bear_[size_t(i)] = 1.0; ex.bearing_[size_t(i)] = true; }
        for (int k = 0; k < 20; ++k) ex.solve_ik();
        pl.plan_ = 1.0;
    }
    void crawl(int ticks) {
        for (int t = 301; t < 301 + ticks; ++t) {
            ex.sense(uint64_t(t));
            for (int i = 0; i < 4; ++i) {
                const bool stepping = i == ex.leg_ && (ex.phase_ == Phase::Lift || ex.phase_ == Phase::Swing || ex.phase_ == Phase::Lower);
                const double depth = i == air_leg ? 0.012 : 0.002;   // the ground under this toe, below the standing plane
                const bool on = ex.toe_tgt_[size_t(i)].y <= float(-ex.stand_h_m_ - depth);
                in.contact_[size_t(i)] = (stepping || i == air_leg) ? (on ? 1.0f : 0.0f) : 1.0f;
                in.load_[size_t(i)] = in.contact_[size_t(i)] > 0.5f ? 0.25f : 0.0f;
            }
            ex.step_machine(); ex.hold_stance(); ex.solve_ik();
            if (at.on()) at.observe(ex, pl, in, book);
        }
    }
};

TEST(AfterTouch, TheModelLearnsThatAToeHeldAboveItsTouchDoesNotBear_GradedAgainstTheBaseRate) {
    AfterTouch at; uint32_t rng = 7u; auto unit = [&] { rng = rng * 1664525u + 1013904223u; return double(rng >> 8) / 16777216.0; };
    for (int i = 0; i < 400; ++i) {
        const double gap = -0.01 + 0.05 * unit();                            // the stance's hold against the touch, -1 .. +4 cm
        const bool air = gap > 0.012 + 0.004 * (unit() - 0.5);
        const auto phi = AfterTouch::features(gap, 8.0 * unit(), 0.1 * unit());
        AfterTouch::Out y{air ? 1.0 : 0.0, air && unit() < 0.7 ? 1.0 : 0.0, unit() - 0.5, 0.0};
        at.update(i % 4, phi, y, {true, true, true, i % 3 == 0});
    }
    EXPECT_GT(at.skill(0), 0.5) << "the gap carries the bearing: the model beats each leg's own base rate";
    EXPECT_GT(at.skill(1), 0.2);
    EXPECT_LT(std::abs(at.skill(2)), 0.1) << "an outcome the features do not carry earns no skill";
    EXPECT_LT(at.n_[3], at.n_[0]) << "an outcome a record did not give is not learned from";
    EXPECT_GT(at.nis(0), 0.3); EXPECT_LT(at.nis(0), 3.0);
    const auto lo = at.predict(0, AfterTouch::features(0.0, 4.0, 0.05)), hi = at.predict(0, AfterTouch::features(0.03, 4.0, 0.05));
    EXPECT_LT(lo[0], 0.3); EXPECT_GT(hi[0], 0.7);
    AfterTouch b; b.restore(at.snapshot());
    EXPECT_EQ(b.snapshot().dump(), at.snapshot().dump());
    EXPECT_DOUBLE_EQ(b.predict(2, AfterTouch::features(0.02, 3.0, 0.0))[0], at.predict(2, AfterTouch::features(0.02, 3.0, 0.0))[0]);
}

TEST(AfterTouch, ItReadsAndMovesNothing_AndEveryLandedStepGetsARecordClosedAtTheNext) {
    Rig a, b; b.at.learn_ = 1.0;
    a.crawl(3000); b.crawl(3000);
    EXPECT_EQ(a.ex.outcome_seq_, b.ex.outcome_seq_);
    for (int i = 0; i < 4; ++i) { EXPECT_EQ(a.ex.toe_tgt_[size_t(i)].x, b.ex.toe_tgt_[size_t(i)].x); EXPECT_EQ(a.ex.toe_tgt_[size_t(i)].y, b.ex.toe_tgt_[size_t(i)].y); EXPECT_EQ(a.ex.toe_tgt_[size_t(i)].z, b.ex.toe_tgt_[size_t(i)].z); EXPECT_EQ(a.ex.steps_[size_t(i)], b.ex.steps_[size_t(i)]); }
    EXPECT_EQ(a.rng, b.rng);
    const auto& c = b.at.count_;
    EXPECT_GT(b.ex.outcome_seq_, 6);
    EXPECT_EQ(c[0], b.ex.outcome_seq_) << "a record per outcome";
    EXPECT_GE(c[1] + c[2] + 1, c[0]) << "each is closed by the next (the last may still be open)";
    EXPECT_EQ(c[2], 0);
    EXPECT_GT(c[4], 4) << "the toes stand long enough to be read";
    EXPECT_EQ(c[5], 0) << "on flat ground every landed toe bears";
    EXPECT_EQ(c[7], 0) << "and the wave goes on: no leg steps twice running";
    const auto f = b.at.last_.floats(); ASSERT_EQ(int(f.size()), AfterTouch::Record::kFloats);
    EXPECT_NEAR(f[6], 0.002, 0.004) << "flat ground: the stance holds the toe where it touched, to the lowering's last step";
    EXPECT_EQ(f[30], 1.0) << "the go was seen: the predicted touch height is the executor's";
    EXPECT_NEAR(f[4], f[3], 0.02);
}

TEST(AfterTouch, AToeTheStanceLiftsOffItsGroundIsReadAsNotBearing) {
    Rig r; r.at.learn_ = 1.0; r.air_leg = 0; r.ex.creep_max_m_ = 0.0;   // no load-seeking creep: the stance alone decides the height
    r.crawl(4000);
    ASSERT_GE(r.ex.steps_[0], 2);
    EXPECT_GT(r.at.count_[5], 0) << "the toe touched 12 mm below the plane and was lifted back to it";
    EXPECT_GE(r.at.count_[5] + 2, r.ex.steps_[0]) << "every landing of that leg, the last two perhaps still open or too short";
    EXPECT_GT(r.at.legs_[0].mean[0], 0.8); EXPECT_LT(r.at.legs_[1].mean[0], 0.2);
}

TEST(AfterTouch, TheModuleCarriesItOnlyWhenItIsOn_DiagSnapshotAndParams) {
    ogma::PoseSequencer m;
    EXPECT_FALSE(m.diag_snapshot().contains("at_rec")) << "off: the diag is the scaffold's";
    EXPECT_TRUE(m.snapshot_state().empty()) << "off: nothing is carried";
    EXPECT_EQ(std::get<double>(m.current_params().at("at_learn")), 0.0);
    m.on_param_change("at_learn", ogma::ParamValue{1.0});
    EXPECT_TRUE(m.after().on());
    const auto d = m.diag_snapshot(); ASSERT_TRUE(d.contains("at_rec"));
    EXPECT_EQ(int(d["at_rec"].size()), AfterTouch::Record::kFloats); EXPECT_EQ(d["at_n"].size(), 10u); EXPECT_EQ(d["at_grade"].size(), 20u);
    for (auto const& [k, v] : d.items()) if (k.rfind("at_", 0) == 0) EXPECT_FALSE(v.is_object()) << k << ": the host exports scalars and arrays only";
    EXPECT_FALSE(d.contains("lo_dec")) << "the landing offset is its own switch";
    const auto snap = m.snapshot_state(); ASSERT_TRUE(snap.contains("at")); EXPECT_FALSE(snap.contains("lo"));
    ogma::PoseSequencer n; n.on_param_change("at_learn", ogma::ParamValue{1.0}); n.restore_state(snap);
    EXPECT_EQ(n.snapshot_state().dump(), snap.dump());
    ogma::PoseSequencer off; off.restore_state(snap); EXPECT_TRUE(off.snapshot_state().empty()) << "a carried model is ignored while the record is off";
}
}
