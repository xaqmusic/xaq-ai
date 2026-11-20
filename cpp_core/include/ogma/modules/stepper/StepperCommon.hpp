#pragma once
// The stepper's shared vocabulary: its sensor inputs, the unloaded-toe book, the step intent, the support triangle's margin.
// Split out of PoseSequencer byte-identical (learned stepper plan Phase 2, 2026-11-15): every function body is the
// monolith's, moved, with its members qualified by owner; the golden traces (scripts_tools/stepper_battery.sh +
// stepper_ident.py) are the gate.  The member comments are the monolith's, kept with the member they document.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
#include "ogma/body/LegKinematics.hpp"

namespace ogma::stepper {
using body::Vec3f;
inline Vec3f sub(const Vec3f& a, const Vec3f& b) { return Vec3f(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3f add(const Vec3f& a, const Vec3f& b) { return Vec3f(a.x + b.x, a.y + b.y, a.z + b.z); }
inline double len(const Vec3f& a) { return std::sqrt(double(a.x) * a.x + double(a.y) * a.y + double(a.z) * a.z); }

enum class Phase { Idle = 0, Shift, Lift, Swing, Lower, Settle, Return };   // Return (2026-11-19): the hand-back's way to a stand, see StepExecutor::return_now

// the signed distance of (cx, cz) inside the triangle p (+ inside) and the inward unit normal of the edge nearest it
double tri_margin(const std::array<std::array<double, 2>, 3>& p, double cx, double cz, double& nx, double& nz);

// The sensor caches the module's subscriptions fill (PoseSequencer::handle), read by every component.
struct StepperInputs {
    std::array<double, 3> odom_in_{0.0, 0.0, 0.0};
    bool have_odom_in_ = false;
    float belly_ = 0.0f;
    bool have_belly_ = false;
    std::array<float, 6> imu_{};
    bool have_imu_ = false;
    // inputs
    std::array<float, 26> support_{};
    bool have_support_ = false;
    std::vector<float> patch_;
    int N_ = 0;
    double cell_ = 0.03;
    bool have_patch_ = false;
    std::array<float, 8> xz_{};
    std::array<float, 4> y_{}; std::array<float, 4> contact_{}; std::array<float, 4> load_{};
    std::array<double, 3> up_{0.0, 1.0, 0.0};
    bool have_xz_ = false; bool have_y_ = false; bool have_contact_ = false; bool have_load_ = false; bool have_up_ = false;
    std::array<float, 12> gait_u_{};
    bool have_gait_ = false;
    bool patch_h(double fwd, double lat, double& h) const;                  // remembered ground height (gravity frame) under a body-frame point
    bool patch_h(double fwd, double lat, double& h, double& sigma) const;   // ... with its uncertainty (m) from the cells' precisions
    bool patch_edge(double fwd, double lat, double& relief) const;          // the height relief (max - min) of the remembered cells around a point: a riser's edge
    double gravity_h(const Vec3f& p) const { return p.x * up_[0] + p.y * up_[1] + p.z * up_[2]; }
    Vec3f at_gravity_h(double x, double z, double h) const { return Vec3f(float(x), float((h - x * up_[0] - z * up_[2]) / std::max(0.2, up_[1])), float(z)); }
};

// The unloaded-toe rule's memory: the executor's stance books what each toe carries, the planner decides on it.
struct LoadBook {
    // LEVER 3 (operator, 2026-11-12: "on the high steps the chassis tilting backwards moves the centre of gravity back and the rear
    // legs cannot be lifted; the front planted legs should lower the chassis in the front"): the FRONT DROP -- the front toes are
    // held front_drop_gain x the nose-up pitch beyond front_drop_deadband shallower (not zero-mean: the front of the chassis
    // comes down), bounded by front_drop_max_m; and the DEPTH CAP -- no toe is asked more than depth_max_m below the body (the
    // freezes at the 38 and 64 mm risers had a rear toe asked 13-14 cm down on a nose-up body: the leg straightened to its pole,
    // the real knee hyperextended against its stop and the servos could not bring it back; the brain has no joint feedback).
    // CLIP AND RE-ACQUIRE (operator, 2026-11-12: "why can't we assume the commanded joint angles are the real joint angles and
    // clip (a partial miss of the target) or attempt re-acquisition of the target?"): clip_ticks -- a lift or swing whose commanded
    // toe has been more than unreach_tol_m from its target for this many ticks takes the part of the arc it reached and moves on
    // (the body froze while a leg waited out its timeouts: only SHIFT moves the body); miss_reacquire -- a missed landing walks the
    // leg to its neutral pose (the re-seed's walk), a lowering gives up after miss_ticks or six retreats instead of 4.8 s, and the
    // leg's next attempt is DIFFERENT: its landing grid perturbed by up to 2 cm, its lift 1 cm lower per miss, the leg penalised
    // in the search by 1 cm of margin per miss so the others step first; the count clears at a landing that touched.
    // THE UNLOADED TOE (2026-11-13; unloaded_shift_m 0 = off, byte-identical): a planted toe that touches but has carried less
    // than unloaded_share of the weight for unloaded_ticks (the body resting on the other diagonal, the front toes tucked at
    // their inner reach: one leg "landed" 134 times in place at 0.8 s each, the only leg the search found feasible) -- before
    // any leg lifts, the body shifts unloaded_shift_m toward that toe (a planned shift, then a fresh decision).
    double unloaded_shift_m_ = 0.0; double unloaded_share_ = 0.08;
    int unloaded_ticks_ = 50;
    std::array<int, 4> unloaded_run_{};
    std::array<int, 4> unload_cool_{};
    std::array<double, 4> land_max_load_{};
    int last_landed_ = -1;
    std::array<int, 4> unloaded_landings_{};
    int unloaded_landings_min_ = 2;
    int unloaded_skips_min_ = 2;
};

// A step intent: what the planner (later the policy, through the arbiter) asks the executor to do next.
//   Unload  the body shifts (sx, sz) toward leg's unloaded toe, then a fresh decision
//   Plan    leg steps to land, the body shifted (sx, sz) first (the pose search's choice; score its rank)
//   Choice  leg's turn by the order or the greedy choice, the shift controller steering
struct StepIntent {
    enum Kind { Unload, Plan, Choice } kind = Choice;
    int leg = -1; Vec3f land{}; double sx = 0.0, sz = 0.0, score = 0.0;
};

// A step's outcome (Phase 2's instrument, the learned step generator's training signal from Phase 4): what the executor did with
// an intent, recorded at the touch (or the sunk lowering) and published on step.outcome.  kind: -1 untracked (no go recorded),
// 0 unplanned, 1 planned, 2 planned but the leg switched; from = where the step set out to land (the plan's landing for a planned
// step), land = where the toe stopped; h_pred = the ground the executor expected there, h_touch = where the toe met it (gravity
// heights, m); ticks from the go to the touch; hang = a hung-toe reflex fired on the way; moves = the corrections, per rule
// (reach clamp, sector clamp, edge move, obstacle reflex, lowering retreat, the IK's shortfall, clip), as xz vectors (m).
struct StepOutcome {
    int64_t seq = 0; int leg = -1, kind = -1; bool touched = false, hang = false; int ticks = 0;
    Vec3f from{}, land{}; double h_pred = 0.0, h_touch = 0.0;
    std::array<std::array<double, 2>, 7> moves{};
    static constexpr int kFloats = 12 + 14;   // [seq, leg, kind, touched, from x, from z, land x, land z, h_pred, h_touch, ticks, hang, moves x 7 (x, z)]
    std::vector<float> floats() const {
        std::vector<float> v{float(seq), float(leg), float(kind), touched ? 1.0f : 0.0f, from.x, from.z, land.x, land.z, float(h_pred), float(h_touch), float(ticks), hang ? 1.0f : 0.0f};
        for (auto const& m : moves) { v.push_back(float(m[0])); v.push_back(float(m[1])); }
        return v;
    }
};

} // namespace ogma::stepper
