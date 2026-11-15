#pragma once
// ScaffoldPlanner -- the FROZEN pose search, emitting step intents: the leg, its landing and the body's shift chosen together one
// step ahead (plan 1), the unloaded-toe shift, the relief step, and the fixed order / greedy choice when the search is off or finds
// nothing.  It reads the stance from the executor (const) and decides; it moves nothing.  The learned StepPolicy replaces it
// through the arbiter (plan Phases 5-7); until then it is the experience generator, the fallback and the baseline.
// Split out of PoseSequencer byte-identical (learned stepper plan Phase 2, 2026-11-15): every function body is the
// monolith's, moved, with its members qualified by owner; the golden traces (scripts_tools/stepper_battery.sh +
// stepper_ident.py) are the gate.  The member comments are the monolith's, kept with the member they document.
#include <array>
#include <cstdint>
#include <vector>
#include "ogma/body/LegSolver.hpp"
#include "ogma/modules/stepper/StepExecutor.hpp"
#include "ogma/modules/stepper/StepperCommon.hpp"
#include "ogma/modules/stepper/SupportFloor.hpp"

namespace ogma::stepper {

class ScaffoldPlanner {
public:
    ScaffoldPlanner(const StepperInputs& in, const SupportFloor& floor, LoadBook& book, const body::LegSolver& kin, uint32_t& rng, const StepExecutor& ex)
        : in_(in), floor_(floor), book_(book), kin_(kin), rng_(rng), ex_(ex) {}
    ScaffoldPlanner(const ScaffoldPlanner&) = delete; ScaffoldPlanner& operator=(const ScaffoldPlanner&) = delete;

    StepIntent decide();   // the next step: the unloaded toe first, then the pose search, then the order / greedy choice
    bool plan_step(int& leg_out, Vec3f& land_out, double& sx_out, double& sz_out, double& score_out);   // the pose search (lever 2a)
    bool plan_relief(int stuck, double cx, double cz, int bearing, int& leg_out, Vec3f& land_out, double& margin_out) const;
    int  choose_leg();
    bool shift_feasible(int lifting, const std::array<Vec3f, 4>& toes, double sx, double sz) const;
    int  shift_refusal(int lifting, const std::array<Vec3f, 4>& toes, double sx, double sz, int& toe) const;
    double best_margin_over_shifts(int leg, const std::array<Vec3f, 4>& toes, double cx, double cz, double& sx, double& sz, bool forward_first) const;
    void check_drift(double cx, double cz, double need);
    void diagnose_block(int leg, double cx, double cz, double need);

    const StepperInputs& in_; const SupportFloor& floor_; LoadBook& book_; const body::LegSolver& kin_; uint32_t& rng_; const StepExecutor& ex_;

    // THE ORDER (2026-11-08, after six batteries of the greedy choice starving the rear legs 5:1): a FIXED crawl order,
    // rear-left, front-left, rear-right, front-right (a wave gait: each side's rear then its front), with the model as
    // the gate -- the body sways / advances until the leg whose turn it is has its margin, and if the shift is spent
    // without one the turn passes (counted).  A toe the creep could not bring to ground is still placed first.
    // order empty = the greedy choice (the first form).
    std::vector<double> order_ = {2, 0, 3, 1};
    int order_pos_ = -1;
    mutable std::array<double, 4> score_{};
    double relief_step_m_ = 0.01;
    int relief_cells_ = 4;
    double face_gap_m_ = 0.0; double face_penalty_m_ = 0.02;
    int64_t face_penalties_ = 0;
    int64_t unload_shifts_ = 0;
    int last_plan_skips_ = 0;   // the time rule fires only when the last search had to skip this many legs (the unloaded toe blocking the others; a toe lightly loaded on a free crawl is left alone)   // consecutive landings of a leg that never bore weight; the rule fires at the second (one is common and benign)   // the most weight a toe has carried since its landing; the leg that just landed
    // THE LANDING RADIUS (2026-11-13; land_r_min_m 0 = off, byte-identical): a planned landing is kept at least this far from its
    // hip -- the search rewards advance and not base width, and a stance collapsed inward (both right toes at the inner edge of
    // their reach) left every body shift infeasible and one leg landing 134 times at that edge while the body stood.
    double land_r_min_m_ = 0.0;
    int64_t land_r_rejects_ = 0;
    // THE EXTENSION MARGIN (2026-11-14; ext_margin_m 0 = off, byte-identical): a body shift is infeasible when it leaves a planted
    // toe less than ext_margin_m short of its leg's full extension (and further out than it was) -- the distance from hip2 to the
    // toe in the leg's plane at the toe's own depth, against the longest hip2-to-toe the knee's range allows.  The horizontal
    // reach_max_m (0.14) IS a straight leg at the stance's depth: a trailing rear leg the wave had not yet reached was walked to
    // a straight knee (commanded knee u < 0.15 on 50 % of rows after six other steps, 0 % on the front legs; operator 2026-11-14:
    // "the rear legs extend backward too far ... it gets caught on steps and makes the kinematics solve harder").
    double ext_margin_m_ = 0.0;
    mutable int64_t ext_rejects_ = 0;
    // THE LIFTING-LEG FORM (2026-11-14; ext_lifting 0 = off, byte-identical): the leg about to lift keeps ext_lifting x ext_margin_m
    // of slack -- the shift that buys its lift margin runs while its toe is planted and, for a rear leg, points along that leg's
    // heading away from it (a symmetric stance gives a rear leg zero static margin), so with the first form the stretched leg still
    // lifted from near-straight (nearly every crossing into the margin was the lifting leg's own); held to the FULL margin a rear
    // toe at the margin could never lift, so the planted legs' margin reserves the difference for the leg's own pre-lift shift
    double ext_lifting_ = 0.0;
    mutable int64_t ext_lift_rejects_ = 0;
    // THE OVERDUE LEG (2026-11-14; overdue 0 and ext_urgent_m 0 = off, byte-identical; operator: "the leg that is starting to become
    // too straight, especially in the back, needs to be prioritized in the sequence of new lifts"): a rear leg goes straight almost
    // only when its turn was SKIPPED (no feasible step when the wave reached it) and later legs stepped in its place with forward
    // shifts -- near-straight in 5 % of on-schedule stance periods, 54-75 % after six or more other steps.  overdue 1: while the
    // wave's leg (or an urgent leg) has no feasible step, the substitute steps serve it -- no shift may reduce its slack to full
    // extension, and a substitute's landing is ranked first by whether it leaves that leg liftable, then as before.  ext_urgent_m:
    // the planted leg with the least slack below this is tried before the wave's leg (a jump does not consume the wave's turn).
    double overdue_ = 0.0; double ext_urgent_m_ = 0.0;
    mutable int protect_ = -1;
    mutable int64_t overdue_rejects_ = 0;
    int64_t overdue_decisions_ = 0; int64_t urgent_jumps_ = 0; int64_t urgent_fails_ = 0;
    bool jumped_ = false;   // one jump per wave turn (a jump every decision starved the wave's own leg: 43 steps of one leg, 0 of another)
    std::array<int64_t, 4> skip_nomargin_{}; std::array<int64_t, 4> skip_noland_{}; std::array<int64_t, 4> skip_miss_{};   // why each leg's turn was skipped (an instrument, always on)
    // THE BLOCKER (2026-11-14, an instrument, always on, decides nothing): when a leg's turn is skipped because no feasible shift gives
    // it a lift margin, the shift grid is walked again; of the shifts that WOULD give the margin, the smallest one's refusal is
    // counted -- which toe, which check -- or "grid" when no shift in the grid gives the margin at all.  block_[leg][toe * 7 + reason],
    // reason 1 outer reach, 2 inner reach, 3 protection, 4 sector, 5 extension margin, 6 the lifting leg's floor (0 = feasible: a bug);
    // index 28 = grid.  block_d_ sums the smallest margin-giving shift's length, block_short_ the grid case's shortfall of margin.
    std::array<std::array<int64_t, 29>, 4> block_{};
    std::array<double, 4> block_d_{}; std::array<double, 4> block_short_{};
    // NEXT LEG LIFTABLE FIRST (2026-11-14; plan_next_first 0 = off, byte-identical): the landing lookahead rated the next leg's lift
    // margin only as a capped term the advance could outweigh, so a landing that left the next leg unliftable won whenever it went
    // further forward -- and the next leg's turn was then skipped (the blocker: the opposite toe at its inner reach).  On: a landing
    // that leaves the next leg liftable ranks above every landing that does not; the advance chooses among those.  next_lift_ counts
    // the chosen landings [left the next leg liftable, did not] (an instrument, always on).
    double plan_next_first_ = 0.0;
    std::array<int64_t, 2> next_lift_{};
    // THE PLAN DRIFT (2026-11-14, an instrument, always on, decides nothing): each decision's prediction for the next leg -- its lift
    // margin over the toes the plan expects (the others moved by the planned shift, the stepping toe at its planned landing) and the
    // centre of mass -- is checked at the next decision against the margin found.  drift_ = [predictions checked, predicted
    // liftable, of those found unliftable, predicted unliftable]; for a predicted-liftable leg found unliftable, drift_cause_ =
    // [the landed toe alone breaks it, the other toes alone, the centre of mass alone, only together] by swapping one factor at a
    // time from predicted to actual; drift_mag_ = summed [landed-toe offset, mean other-toe offset, centre-of-mass move] (m) over
    // those cases, drift_mag_all_ the same over every check.
    int pred_leg_ = -1; int pred_step_ = -1;
    double pred_m_ = 0.0; double pred_cx_ = 0.0; double pred_cz_ = 0.0;
    std::array<body::Vec3f, 4> pred_toes_{};
    std::array<int64_t, 4> drift_{}; std::array<int64_t, 4> drift_cause_{};
    std::array<double, 3> drift_mag_{}; std::array<double, 3> drift_mag_all_{};
    int64_t miss_skips_ = 0;
    double plan_ = 0.0; double plan_shift_m_ = 0.03; double plan_shift_fwd_m_ = 0.04; double plan_land_m_ = 0.04; double plan_cell_m_ = 0.01; double plan_adv_w_ = 0.3;
    int64_t plans_ = 0; int64_t plan_skips_ = 0; int64_t plan_none_ = 0;
};

} // namespace ogma::stepper
