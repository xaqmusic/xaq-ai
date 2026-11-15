#pragma once
// LegSolver -- the stepper's leg model: forward and inverse kinematics on the body's own FK (LegKinematics.hpp), the
// joint ranges in the body's action units, the workspace sector, and the extension slack.  Split out of PoseSequencer
// (learned stepper plan Phase 2, 2026-11-15) byte-identical: every expression is the monolith's, moved.  It is a plain
// component, not a bus module -- the step executor, the scaffold planner and (later) the step generator and policy all
// read the same leg model, and none of them may own it.
//
// The model arrives as the body's reality.proprio.leg_model token (kModelFloats floats: per leg 27 = the eight LegAnchors
// vectors and the toe offset, then the eight joint-range floats).  ready() is false until it has.
//
// The ROOM measure (Phase 2's instrument, the plan's "one quantity replacing the five proxies") is room(): per leg, how
// far the commanded pose is from each of its limits -- the extension slack to a straight leg (m, by the true FK), and the
// distance of each joint from its nearest stop in action units (1 = at the rest pose's far side, 0 = at the stop).
#include <array>
#include <cstddef>
#include <cstdint>
#include "ogma/body/LegKinematics.hpp"

namespace ogma::body {

struct LegSolverParams {
    int    ik_iters = 4;              // damped steps per tick
    double ik_damping_m = 0.01, ik_step_max_rad = 0.12, ik_tol_m = 0.004;
    double ik_switch_m = 0.005;       // the multistart takes another branch only when it is this much closer
    double reach_min_m = 0.06, reach_max_m = 0.14, sector_half_rad = 0.9;   // the workspace: a sector about the leg's heading from hip1
};

class LegSolver {
public:
    static constexpr int kModelFloats = 4 * 27 + 8;
    LegSolverParams p;

    // the body's leg model; false (and nothing changed) when v is short.  The extension reference is taken once.
    bool set_model(const float* v, std::size_t n);
    bool ready() const { return have_model_; }

    Vec3f toe_of(int leg, const std::array<double, 3>& t) const;                 // body-frame toe for joint angles
    std::array<double, 3> angles_of_u(int leg, const std::array<double, 3>& u) const;
    std::array<double, 3> u_of_angles(int leg, const std::array<double, 3>& t) const;
    bool ik_step(int leg, const Vec3f& target, std::array<double, 3>& t) const;   // one damped step; true when within tol
    // the solve: ik_iters damped steps, then (allow_multistart) a restart from seeds covering the branches when the single seed
    // is left further than 2 x ik_tol from its target; counts multistarts / branch_switches
    bool ik_solve(int leg, const Vec3f& target, std::array<double, 3>& t, bool allow_multistart);

    Vec3f hip1(int leg) const { return anchors_[std::size_t(leg)].hip1_world; }
    Vec3f hip2(int leg) const { return anchors_[std::size_t(leg)].hip2_world; }
    // clamp a body-frame point into the leg's sector (half angle half_rad about its heading, radius in [reach_min, reach_max]);
    // true when it moved by more than a tenth of a millimetre
    bool sector_clamp(int leg, Vec3f& p, double half_rad) const;
    bool sector_clamp(int leg, Vec3f& pt) const { return sector_clamp(leg, pt, p.sector_half_rad); }
    // the extension slack: full extension minus the hip2-to-toe distance for a toe at pt (the leg's plane through the hip1
    // axis); and the same at joint angles t by the true FK.  1.0 before the model.
    bool ext_ready() const { return ext_lmax_ok_; }
    double ext_slack(int leg, const Vec3f& pt) const;
    double ext_slack_fk(int leg, const std::array<double, 3>& t) const;

    struct Room { double ext_m = 1.0, knee_fold = 1.0, knee_straight = 1.0, hip1 = 1.0, hip2 = 1.0; };
    Room room(int leg, const std::array<double, 3>& t) const;

    int64_t multistarts = 0, branch_switches = 0;

private:
    std::array<LegAnchors, 4> anchors_{}; std::array<Vec3f, 4> toe_off_{};
    double hip1_rest_ = 0, hip1_range_ = 1.4, hip2_rest_ = 0, hip2_range_ = 1.4, knee_rest_ = -1.6, knee_fold_ = 3.2, knee_hyper_ = 0.85; bool knee_widening_ = true;
    bool have_model_ = false;
    std::array<double, 4> ext_lmax_{}; bool ext_lmax_ok_ = false;
};

} // namespace ogma::body
