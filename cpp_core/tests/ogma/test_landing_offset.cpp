// The landing offset (learned stepper plan Phase 4, 2026-11-16): the model and its expected-free-energy choice standing alone, then
// the seam on the stepper's components wired by hand -- in shadow the body is the scaffold's, with authority the landings shift
// only where the floor admits and every landed step is paired with the offset that was chosen for it.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/modules/PoseSequencer.hpp"
#include "ogma/modules/stepper/LandingOffset.hpp"
#include "ogma/modules/stepper/ScaffoldPlanner.hpp"
#include "ogma/modules/stepper/StepExecutor.hpp"
#include "ogma/modules/stepper/StepperCommon.hpp"
#include "ogma/modules/stepper/SupportFloor.hpp"
#include "stepper_fixture.hpp"
namespace {
using namespace ogma::stepper; using ogma::body::LegSolver;

std::vector<LandingOffset::Candidate> grid(double max_m, double cell_m, double rise_fwd_m) {
    // offsets 0, +1, -1, ... cells; the terrain: every candidate ahead of the plan sits rise_fwd_m higher (a riser's lip)
    std::vector<LandingOffset::Candidate> c; const int n = int(std::floor(max_m / cell_m + 1e-9));
    for (int i = 0; i <= 2 * n; ++i) {
        const int cell = i == 0 ? 0 : ((i % 2) ? (i + 1) / 2 : -(i / 2));
        LandingOffset::Candidate k; k.delta = cell * cell_m; k.phi = LandingOffset::features(k.delta / max_m, cell > 0 ? rise_fwd_m : 0.0, cell > 0 ? rise_fwd_m : 0.0, 0.002, true);
        c.push_back(k);
    }
    return c;
}

struct Rig {
    StepperInputs in; LoadBook book; SupportFloor floor; LegSolver kin; uint32_t rng = 12345u;
    StepExecutor ex{in, floor, book, kin, rng};
    ScaffoldPlanner pl{in, floor, book, kin, rng, ex};
    LandingOffset lo; bool use_lo = true;
    Rig() {
        const auto m = stepper_test::leg_model(); kin.set_model(m.data(), m.size());
        const auto pt = stepper_test::flat_patch(-0.09f); in.patch_.assign(pt.begin(), pt.end()); in.N_ = int(pt[0]); in.cell_ = pt[1]; in.have_patch_ = true;
        const auto s = stepper_test::support(0.03f); for (int i = 0; i < 26; ++i) in.support_[size_t(i)] = s[size_t(i)]; in.have_support_ = true;
        for (int i = 0; i < 4; ++i) in.contact_[size_t(i)] = 1.0f; in.have_contact_ = true;
        ex.intent_hook_ = [this] { const StepIntent i = pl.decide(); return use_lo && lo.on() ? lo.decide(i, in, floor, pl, ex) : i; };
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
                in.contact_[size_t(i)] = (!stepping || ex.toe_tgt_[size_t(i)].y <= float(-ex.stand_h_m_ - 0.002)) ? 1.0f : 0.0f;
            }
            ex.step_machine(); ex.hold_stance(); ex.solve_ik();
            if (use_lo && lo.on()) lo.absorb(ex);
        }
    }
};
}

TEST(LandingOffset, UngradedItExplores_InDistressItDoesNot_AndAVetoedCandidateIsNeverChosen) {
    LandingOffset lo; auto c = grid(0.03, 0.01, 0.0);
    const auto first = lo.choose(0, c, false);
    EXPECT_NEAR(first.delta, 0.03, 1e-12) << "nothing learned: the most informative candidate, forward first on a tie";
    EXPECT_GT(first.epi, 0.0); EXPECT_EQ(first.risk, 0.0);
    const auto calm = lo.choose(0, c, true);
    EXPECT_EQ(calm.delta, 0.0) << "in distress the epistemic term is dropped; with nothing learned the tie goes to the scaffold's landing";
    for (auto& k : c) if (k.delta > 0.0) k.admitted = false;
    const auto vetoed = lo.choose(0, c, false);
    EXPECT_NEAR(vetoed.delta, -0.03, 1e-12) << "the forward candidates vetoed: the most informative of the rest";
}

TEST(LandingOffset, ItLearnsWhichLandingsHangAndStopsChoosingThem_GradedByItsOwnError) {
    // the world: a landing on the lip ahead (any forward offset: 4 cm of rise there) hangs and lands 1 cm off its predicted ground;
    // the plan's landing and the ones behind it are clean but for a little noise
    LandingOffset lo; uint32_t rng = 7u; auto noise = [&] { rng = rng * 1664525u + 1013904223u; return double(rng >> 8) / 16777216.0 - 0.5; };
    int early_fwd = 0, late_fwd = 0, early_n = 0, late_n = 0; std::array<int, 3> sign{};
    for (int step = 0; step < 400; ++step) {
        const int leg = step % 4; const auto c = grid(0.03, 0.01, 0.04);
        const auto ch = lo.choose(leg, c, false);
        const bool fwd = ch.delta > 1e-9;
        ++sign[size_t(ch.delta > 1e-9 ? 2 : (ch.delta < -1e-9 ? 0 : 1))];
        if (step < 40) { ++early_n; early_fwd += fwd; } else if (step >= 300) { ++late_n; late_fwd += fwd; }
        LandingOffset::Out y; y << (fwd ? 1.0 : 0.0) + 0.2 * noise(), 0.1 * std::abs(noise()), fwd ? 1.0 : 0.0, 0.0;
        lo.update(leg, c[size_t(ch.index)].phi, y);
    }
    EXPECT_GT(early_fwd, 0) << "it tried the lip while it knew nothing about it";
    EXPECT_GT(sign[0], 0); EXPECT_GT(sign[2], 0);
    EXPECT_EQ(late_fwd, 0) << "and stopped once the model predicted the hang (early " << early_fwd << " of " << early_n << ", late " << late_fwd << " of " << late_n << ")";
    {   // the model predicts the hang where it happens and not where it does not
        const auto g = grid(0.03, 0.01, 0.04); double fwd_hang = 0.0, here_hang = 0.0;
        for (int leg = 0; leg < 4; ++leg) { fwd_hang += (lo.legs_[size_t(leg)].W * g[1].phi)[2] / 4.0; here_hang += (lo.legs_[size_t(leg)].W * g[0].phi)[2] / 4.0; }
        EXPECT_GT(fwd_hang, 0.5); EXPECT_LT(std::abs(here_hang), 0.2);
    }
    EXPECT_GT(lo.s2_[2], 0.0); EXPECT_LT(lo.s2_[2], 0.1) << "it paid for the lesson with a few hangs, not many";
    for (int k = 0; k < 2; ++k) { EXPECT_GT(lo.nis(k), 0.3); EXPECT_LT(lo.nis(k), 3.0) << "outcome " << k << ": its errors are about the size it expects"; }
    EXPECT_GT(lo.nis(2), 0.0);
    EXPECT_EQ(lo.nis(3), 0.0) << "no miss ever happened: that outcome has no grade yet";
    // the model round-trips: a restored copy makes the same choice with the same scores
    LandingOffset b; b.restore(lo.snapshot());
    const auto c = grid(0.03, 0.01, 0.04); const auto x = lo.choose(1, c, false), y = b.choose(1, c, false);
    EXPECT_EQ(x.index, y.index); EXPECT_DOUBLE_EQ(x.risk, y.risk); EXPECT_DOUBLE_EQ(x.epi, y.epi);
}

TEST(LandingOffset, InShadowTheBodyIsTheScaffolds_AndTheModelStillLearnsFromItsSteps) {
    Rig a, b; a.use_lo = false; b.lo.learn_ = 1.0;   // b: learning, no authority
    a.crawl(3000); b.crawl(3000);
    EXPECT_EQ(a.ex.outcome_seq_, b.ex.outcome_seq_);
    for (int i = 0; i < 4; ++i) { EXPECT_EQ(a.ex.toe_tgt_[size_t(i)].x, b.ex.toe_tgt_[size_t(i)].x); EXPECT_EQ(a.ex.toe_tgt_[size_t(i)].z, b.ex.toe_tgt_[size_t(i)].z); EXPECT_EQ(a.ex.steps_[size_t(i)], b.ex.steps_[size_t(i)]); }
    EXPECT_EQ(a.pl.ext_rejects_, b.pl.ext_rejects_); EXPECT_EQ(a.rng, b.rng);
    EXPECT_EQ(b.lo.shifted_, 0); EXPECT_EQ(b.lo.vetoed_, 0);
    EXPECT_GT(b.lo.updates_, 4); EXPECT_GE(b.lo.decisions_, b.lo.updates_);
    EXPECT_EQ(b.lo.hist_[6], b.lo.decisions_) << "every choice was the scaffold's landing";
}

TEST(LandingOffset, WithAuthorityItShiftsLandingsTheFloorAdmits_AndPairsEveryLandedStepWithItsOffset) {
    Rig r; r.lo.learn_ = 1.0; r.lo.gain_ = 1.0;
    r.crawl(6000);
    for (int i = 0; i < 4; ++i) EXPECT_GE(r.ex.steps_[size_t(i)], 2) << "the crawl still walks every leg, leg " << i;
    EXPECT_GT(r.lo.shifted_, 0) << "ungraded, it explores";
    EXPECT_GT(r.lo.updates_, 8);
    EXPECT_LT(r.ex.reloc_resid_, 1e-9) << "the relocation tag still closes on the shifted landings";
    int total = 0; for (int i = 0; i < 4; ++i) total += r.ex.steps_[size_t(i)];
    EXPECT_GE(r.lo.updates_ + r.lo.unpaired_ + 4, total) << "landed steps are learned from or counted";
    int64_t h = 0; for (auto v : r.lo.hist_) h += v; EXPECT_EQ(h, r.lo.decisions_);
    EXPECT_EQ(r.lo.veto_by_[0] + r.lo.veto_by_[1] + r.lo.veto_by_[2] + r.lo.veto_by_[3], r.lo.vetoed_) << "every refusal is attributed to a check and a side";
    EXPECT_EQ(r.lo.hist_[0] + r.lo.hist_[1] + r.lo.hist_[2] + r.lo.hist_[10] + r.lo.hist_[11] + r.lo.hist_[12], 0) << "no offset beyond gain x max";
    // half the authority: the offsets on offer stop at one cell (1.5 cm of range on a 1 cm grid)
    Rig q; q.lo.learn_ = 1.0; q.lo.gain_ = 0.5; q.crawl(3000);
    EXPECT_EQ(q.lo.hist_[3] + q.lo.hist_[4] + q.lo.hist_[8] + q.lo.hist_[9], 0);
    EXPECT_GT(q.lo.hist_[5] + q.lo.hist_[7], 0);
}

TEST(LandingOffset, TheModuleCarriesItOnlyWhenItIsOn_DiagSnapshotAndParams) {
    ogma::PoseSequencer m;
    EXPECT_FALSE(m.diag_snapshot().contains("lo_dec")) << "off: the diag is the scaffold's";
    EXPECT_TRUE(m.snapshot_state().empty()) << "off: nothing is carried";
    const auto p0 = m.current_params(); EXPECT_EQ(std::get<double>(p0.at("lo_gain")), 0.0); EXPECT_EQ(std::get<double>(p0.at("lo_learn")), 0.0);
    m.on_param_change("lo_learn", ogma::ParamValue{1.0}); m.on_param_change("lo_gain", ogma::ParamValue{0.5});
    EXPECT_TRUE(m.offset().on()); EXPECT_DOUBLE_EQ(m.offset().gain_, 0.5);
    const auto d = m.diag_snapshot(); ASSERT_TRUE(d.contains("lo_dec")); EXPECT_EQ(d["lo_dec"].size(), 12u); EXPECT_EQ(d["lo_upd"].size(), 7u); EXPECT_EQ(d["lo_hist"].size(), 13u);
    for (auto const& [k, v] : d.items()) if (k.rfind("lo_", 0) == 0) EXPECT_FALSE(v.is_object()) << k << ": the host exports scalars and arrays only";
    const auto snap = m.snapshot_state(); ASSERT_TRUE(snap.contains("lo"));
    ogma::PoseSequencer n; n.on_param_change("lo_learn", ogma::ParamValue{1.0}); n.restore_state(snap);
    EXPECT_EQ(n.snapshot_state().dump(), snap.dump());
    ogma::PoseSequencer off; off.restore_state(snap); EXPECT_TRUE(off.snapshot_state().empty()) << "a carried model is ignored while the layer is off";
}
