#pragma once
// LandingOffset -- the first learned authority over the step (learned stepper plan Phase 4, 2026-11-16; operator: "with a truly
// learning brain in control, the robot should be able to build the i/o mappings that work and build upon them").
//
// THE ONE VALUE OWNED.  The forward/back offset delta of the scaffold's PLANNED landing (StepIntent::Plan only; Choice and Unload
// pass through), along the body's forward axis, over candidates gain x max_m either side on cell_m.  The scaffold keeps the leg
// order and the body's shift.  MEASURED 2026-11-16 (ledger "the first battery with authority"): that does NOT keep progress safe --
// the floor admits backward offsets only (the scaffold already lands at its forward limit), a shorter landing is spent as
// progress (corridor -19 %), and no outcome below prices it.  REGRESSION in that context; the layer stays at gain 0.
//
// THE ERROR IT MINIMISES (CLAUDE.md section 1).  Each step's realised consequence against the stepper's own outcome priors: the
// toe meets the ground the memory predicted (touchdown error 0), the hand does not have to correct the landing (correction 0),
// no hung toe, no missed landing.  The model predicts that outcome vector for a candidate landing from what varies across the
// candidates -- delta, delta^2 and the remembered terrain AT the candidate (its rise over the ground under the toe now, the
// relief around it, the memory's uncertainty there, whether the memory knows it) -- by a recursive least squares per leg whose
// targets are the realised outcomes of the offset it chose, paired at the touch.  The scaffold's choice is never a target (no
// teacher, section 5.6).  Terms that do not vary across the candidates (the stance) cancel in the choice; the stance context
// enters as its own lever, as interactions with delta.
//
// THE CHOICE: EXPECTED FREE ENERGY.  delta = argmin over the candidates the floor admits of
//     G = sum_k yhat_k^2 / (2 s_k^2)  -  (K / 2) ln(1 + phi' P phi)
// risk: the predicted divergence from the priors, each outcome scaled by its own running mean square (never a hand-set scale,
// section 5.5); epistemic value: the information a step at that candidate would give the model (its posterior's leverage there).
// Ungraded, the epistemic term leads and the choice explores; as the posterior tightens, the risk leads.  Ties go to the smaller
// offset.  In distress (the last step hung or missed, the search skipped a leg, the body tilted half-way to the disengage) the
// epistemic term is dropped: no exploration.  Nothing here is a reward: the priors are the stepper's own predictions of itself.
//
// THE FLOOR.  The scaffold's landing is always a candidate.  A shifted landing is offered only if it passes the pose search's own
// landing checks (reach, sector, the landing-radius floor, reachable at the stance's depth) and leaves the next leg no less
// liftable than the scaffold's landing did (ScaffoldPlanner::landing_ok and its margin search); the rest are counted as vetoed.
//
// GAIN-0-GUARDED.  gain 0 and learn 0 (the defaults): never consulted, nothing computed, no diag keys -- byte-identical.
// learn > 0 with gain 0: the model learns in shadow from the scaffold's own landings (delta 0) and reports; the body is the
// scaffold's.  gain is read at every decision (a handle a higher loop may drive).
#include <array>
#include <cstdint>
#include <vector>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "ogma/modules/stepper/StepperCommon.hpp"

namespace ogma::stepper {

class ScaffoldPlanner; class StepExecutor; struct SupportFloor;

class LandingOffset {
public:
    static constexpr int kF = 7;   // [1, d, d^2, rise cm, relief cm, sigma cm, unknown]   (d = delta / max_m)
    static constexpr int kY = 4;   // [touchdown error cm, the hand's correction cm, hang, miss]
    using Feat = Eigen::Matrix<double, kF, 1>; using Out = Eigen::Matrix<double, kY, 1>;
    struct Candidate { double delta = 0.0; Feat phi = Feat::Zero(); bool admitted = true; };
    struct Choice { int index = 0; double delta = 0.0, risk = 0.0, epi = 0.0, u = 0.0; Out yhat = Out::Zero(); };

    LandingOffset() { reset(); }
    bool on() const { return gain_ > 0.0 || learn_ > 0.0; }
    void reset();

    // ---- the model, standing alone (no stepper needed: the unit tests drive it directly) ----
    static Feat features(double d_unit, double rise_m, double relief_m, double sigma_m, bool known);
    static Out outcome_vector(const StepOutcome& o);
    Choice choose(int leg, const std::vector<Candidate>& cands, bool distress) const;
    void update(int leg, const Feat& phi, const Out& y);

    // ---- the seam: between the planner's intent and the executor (PoseSequencer's intent hook) ----
    StepIntent decide(StepIntent in, const StepperInputs& inp, const SupportFloor& floor, ScaffoldPlanner& planner, const StepExecutor& ex);
    void absorb(const StepExecutor& ex);   // an outcome recorded since the last look is paired with its decision and learned

    nlohmann::json diag() const;
    nlohmann::json snapshot() const;
    void restore(const nlohmann::json& s);

    // config (HotMutable through PoseSequencer's lo_* params)
    double gain_ = 0.0, learn_ = 0.0, max_m_ = 0.03, cell_m_ = 0.01, p0_ = 1.0;

    // the model: per leg a shared posterior over the features and one weight row per outcome
    struct Leg { Eigen::Matrix<double, kF, kF> P; Eigen::Matrix<double, kY, kF> W; int64_t n = 0; };
    std::array<Leg, 4> legs_;
    Out s2_ = Out::Zero();    // each outcome's running mean square about its prior (0): the risk's scale
    Out r2_ = Out::Zero();    // the prequential residual's running mean square (predicted before the update)
    // the grade (NIS) per outcome: the summed squared residual over the summed variance the model expected of it -- v2 x (1 + u),
    // v2 the running mean of its earlier residuals each divided by its own (1 + u), i.e. the noise the model has shown once the
    // posterior's uncertainty is taken out -- ~1 when its errors are the size it expects.  A ratio of sums, not a mean of ratios: the
    // first residuals are judged against a spread estimated from one or two samples, and a mean of ratios never recovers from
    // them (the first shadow read, 2026-11-16: 164 on the correction).  An outcome that has never erred has no grade yet.
    Out v2_ = Out::Zero(), nis_num_ = Out::Zero(), nis_den_ = Out::Zero();
    double nis(int k) const { return nis_den_[k] > 0.0 ? nis_num_[k] / nis_den_[k] : 0.0; }
    int64_t n_out_ = 0;

    // the decision awaiting its outcome
    struct Pending { bool valid = false; int leg = -1; double delta = 0.0; Feat phi = Feat::Zero(); Out yhat = Out::Zero(); double u = 0.0; int64_t seq = 0; } pending_;
    int64_t seen_seq_ = 0;          // the executor's outcome sequence already looked at
    bool last_bad_ = false;         // the last outcome hung or missed (distress)
    // instruments
    int64_t decisions_ = 0, shifted_ = 0, vetoed_ = 0, distress_ = 0, updates_ = 0, abandoned_ = 0, unpaired_ = 0;
    // the floor's refusals by check and side: [landing checks, next-leg margin] x [forward, back] -- the scaffold's search already
    // rewards advance, so its landing tends to sit at the forward edge of what the floor admits (the first smoke run, 2026-11-16:
    // no forward offset was ever on offer)
    std::array<int64_t, 4> veto_by_{};
    std::array<int64_t, 13> hist_{};   // the chosen offset, in cells, -6 .. +6
    Choice last_{}; int last_leg_ = -1, last_admitted_ = 0; bool last_distress_ = false;
    Out last_err_ = Out::Zero(); double last_upd_delta_ = 0.0; int last_upd_leg_ = -1; int64_t last_upd_seq_ = 0;
};

} // namespace ogma::stepper
