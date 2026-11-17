#pragma once
// AfterTouch -- the step's record carried PAST first contact, and a model of it in shadow (learned stepper plan section 9.5,
// 2026-11-17; operator: "go straight to the shadow build. I don't know what levers will bear fruit so lets investigate").
//
// WHY.  The stepper's record of a step (StepOutcome) ends at the touch.  The stall the operator saw on rough ground -- the body
// standing on three legs, the fourth toe touching, counted landed, carrying 1-3 % of the weight and stepped again for minutes --
// is a failure of a prediction the stepper never made: that the toe it has just placed will bear.  Read off the body logs of six
// 10-minute terrain runs before this was written: 29 % of touched landings do not bear in the stance that follows, and one
// quantity separates them (AUC 0.98) -- how far ABOVE its touch the stance then holds the toe.  The stance holds every planted
// toe at the standing plane plus the levelling (elev_max_m 0), whatever height the FSR said ground at.
//
// THE RECORD (an instrument, deciding nothing).  Opened at each step's outcome, closed at the next one (or after cap_ticks, or
// dropped at a disengage):
//   gap      the height the stance gives the toe on the tick of its touch, minus the height the command had at the touch (m, body
//            frame; + = the toe is held above the ground it met).  y_pred is the touch height the executor expected at the go
//            (the terrain memory's ground at the landing), so gap can also be read as it was predictable before the step.
//   bearing  over the ticks the toe is planted until the record closes: the ticks its raw contact flag is on, the ticks its load
//            share is at least the book's unloaded_share, its mean share, the mean height the stance held it at.
//   the next step  the leg that goes next (the same leg = a re-step) and whether the search that chose it skipped the wave's turn.
//   tilt     the body's tilt at the touch and at the close (deg).
//
// THE MODEL (in shadow).  At the touch, from what is known then -- [1, gap, the gap's positive part, the tilt, the toe's load
// share] -- a recursive least squares per leg and per outcome predicts [the toe will not bear, the same leg will lift next, the
// tilt's change, the wave's turn will be skipped].  Targets are the realised records only; an outcome a record cannot give (a
// toe lifted again before min_ticks of stance has no bearing to read) is left out of that outcome's update, which is why each
// outcome keeps its own posterior.  Graded prequentially: its squared residual against the running-mean predictor's (b2: what
// the leg's own base rate would have predicted) and its NIS.  "Bears" is a threshold outcome (the robot's FSRs are trusted as
// thresholds, not magnitudes).  Nothing here is a reward and nothing is a teacher: the priors are the stepper's own.
//
// GAIN-0-GUARDED.  at_learn 0 (the default): never called, no diag keys, nothing carried -- byte-identical.  at_learn > 0: it
// reads the executor, the planner and the inputs (all const) and moves nothing; the body is the scaffold's.
#include <array>
#include <cstdint>
#include <vector>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "ogma/modules/stepper/StepperCommon.hpp"

namespace ogma::stepper {

class ScaffoldPlanner; class StepExecutor;

class AfterTouch {
public:
    static constexpr int kF = 5;   // [1, gap cm, the gap's positive part cm, tilt at the touch (deg / 10), the toe's load share at the touch]
    static constexpr int kY = 4;   // [the toe does not bear, the same leg lifts next, the tilt's change (deg), the wave's turn is skipped]
    using Feat = Eigen::Matrix<double, kF, 1>; using Out = std::array<double, kY>; using Has = std::array<bool, kY>;
    enum Close { ByLanding = 0, ByCap = 1, ByDisengage = 2 };

    struct Record {
        bool open = false, went = false, touched = false, skip = false;
        int64_t seq = 0; int leg = -1, go_leg = -1, reason = -1;
        double y_touch = 0.0, y_pred = 0.0, y_hold0 = 0.0, gap = 0.0, h_touch = 0.0, h_pred = 0.0, load0 = 0.0, tilt0 = 0.0, tilt1 = 0.0, creep = 0.0;
        bool have_pred = false;
        int ticks = 0, planted = 0, contact = 0, loaded = 0; double load_sum = 0.0, hold_sum = 0.0;
        Feat phi = Feat::Zero(); Out y{}, yhat{}; Has has{};
        static constexpr int kFloats = 32;
        // [0 seq, 1 leg, 2 touched, 3 y_touch, 4 y_pred (or y_touch when the go was not seen), 5 y_hold0, 6 gap, 7 h_touch, 8 h_pred,
        //  9 load at the touch, 10 tilt at the touch, 11 tilt at the close, 12 ticks open, 13 planted ticks, 14 contact ticks,
        //  15 loaded ticks, 16 mean load share, 17 mean held height, 18 creep at the close, 19 the leg that went next (-1 none),
        //  20 the search skipped the wave's turn, 21 close reason, 22-25 the outcomes (-1 = not given), 26-29 the predictions, 30 the
        //  go was seen, 31 spare]
        std::vector<double> floats() const;
    };

    AfterTouch() { reset(); }
    bool on() const { return learn_ > 0.0; }
    void reset();

    // ---- the model, standing alone (the unit tests drive it directly) ----
    static Feat features(double gap_m, double tilt_deg, double load);
    Out predict(int leg, const Feat& phi) const;
    void update(int leg, const Feat& phi, const Out& y, const Has& has);
    double skill(int k) const { return b2_[size_t(k)] > 0.0 ? 1.0 - r2_[size_t(k)] / b2_[size_t(k)] : 0.0; }   // 1 - the model's residual over the base rate's
    double nis(int k) const { return nis_den_[size_t(k)] > 0.0 ? nis_num_[size_t(k)] / nis_den_[size_t(k)] : 0.0; }

    // ---- the seam: once a tick, after the stance and the IK (PoseSequencer::tick) ----
    void observe(const StepExecutor& ex, const ScaffoldPlanner& pl, const StepperInputs& in, const LoadBook& book);

    nlohmann::json diag() const;
    nlohmann::json snapshot() const;
    void restore(const nlohmann::json& s);

    // config
    double learn_ = 0.0, p0_ = 1.0;
    int min_ticks_ = 10;    // a toe planted for fewer ticks than this before its record closes has no bearing to read
    int cap_ticks_ = 250;   // a record still open after this many ticks is closed (a long stand)

    // the model: per leg and per outcome a posterior over the features and a weight row
    struct Leg { std::array<Eigen::Matrix<double, kF, kF>, kY> P; Eigen::Matrix<double, kY, kF> W; std::array<int64_t, kY> n{}; std::array<double, kY> mean{}; };
    std::array<Leg, 4> legs_;
    // the grades, per outcome over all legs: the count, the outcome's running mean, the model's prequential residual mean square,
    // the base rate's (each leg's own running mean, predicted before the update), and the NIS as LandingOffset reads it
    std::array<int64_t, kY> n_{}; std::array<double, kY> mean_{}, r2_{}, b2_{}, v2_{}, nis_num_{}, nis_den_{};

    Record rec_;         // the record open now
    Record last_;        // the last one closed
    int64_t seen_seq_ = 0;
    Phase prev_phase_ = Phase::Idle;
    int go_leg_ = -1; double go_y_pred_ = 0.0; bool go_fresh_ = false;   // the latest go: its leg and the touch height the executor expected
    // instruments: [records opened, closed, dropped at a disengage, closed by the cap, with a bearing read, not bearing, with a go,
    // re-steps, searches that skipped the wave's turn, sunk (the lowering ended without a touch)]
    std::array<int64_t, 10> count_{};

private:
    void close(int reason, const StepExecutor& ex, const StepperInputs& in);
    static double tilt_deg(const StepperInputs& in);
};

} // namespace ogma::stepper
