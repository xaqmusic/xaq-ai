#pragma once
// =============================================================================
// LegKinematics.hpp -- per-leg forward kinematics, shared by sim and host
// =============================================================================
//
// Port doc Phase 4, Order step (a), second half.  The GDScript original is
// picrawler_body.gd::_fk_leg(), and the acceptance bar is the same one ImuAttitude
// already cleared: the sim must stay BYTE-IDENTICAL across the swap.
//
// WHY IT RETURNS FULL TRANSFORMS AND NOT TOE POSITIONS.  The obvious simplification
// -- "callers only want where the foot is" -- is wrong here, and cheaply checked:
// picrawler_body.gd:6675 does `lower_cmd * _toe_off_c[i]`, which needs the BASIS, and
// :7923 assigns all three transforms to node global_transforms in calibrate mode.
// :6722 feeds `feet_y_gravity_cmd_imu`, a promoted input. So all three segments come
// back whole.
//
// THE STATE IT NEEDS is fixed at body construction and does not change with pose:
// three world anchors (hip1/hip2/knee), three rest-transform ORIGINS (only the origin
// is read -- the rest bases never enter), and the two joint axes. Eight Vec3f per leg,
// in LegAnchors. `suspend_lift_y` is the only per-call piece of body state.
//
// ⚠ ARITHMETIC IS TRANSCRIBED FROM godot-cpp, NOT DERIVED -- see GodotFloat.hpp. The
// traps that actually bite, all of them load-bearing here:
//
//   * Transform3D::operator*= sets `origin = xform(p.origin)` using the OLD basis and
//     only then multiplies the basis. Doing it the other way is a different answer,
//     not just different rounding.
//   * The chains are LEFT-associative exactly as GDScript evaluates them:
//     `h3 * h2 * h1 * T` is `((h3*h2)*h1)*T`, and `rot2 * rot1 * axis` is
//     `(rot2*rot1)*axis`. Re-bracketing changes the bits.
//   * Joint angles and suspend_lift_y arrive as DOUBLE because GDScript's `float` is
//     64-bit; they narrow exactly where Godot's API narrows and nowhere earlier.

#include "ogma/body/GodotFloat.hpp"

namespace ogma::body {

// Construction-time anchors for one leg, in the body's world frame.
struct LegAnchors {
    Vec3f hip1_world;          // _hip1_world_c[i]
    Vec3f hip2_world;          // _hip2_world_c[i]
    Vec3f knee_world;          // _knee_world_c[i]
    Vec3f coxa_rest_origin;    // _coxa_rest_xform[i].origin
    Vec3f upper_rest_origin;   // _upper_rest_xform[i].origin
    Vec3f lower_rest_origin;   // _lower_rest_xform[i].origin
    Vec3f hip2_axis;           // _hip2_axes[i]  -- leg-local lateral
    Vec3f knee_axis;           // _knee_axes[i]  -- same lateral as hip2
};

struct LegPose {
    Xform3f coxa;    // [0]
    Xform3f upper;   // [1]
    Xform3f lower;   // [2] -- the one the hot paths read
};

// _fk_leg(i, t1, t2, t3).  t1/t2/t3 are joint angles in radians; suspend_lift_y is
// the body's current suspension lift (_suspend_lift_y).
inline LegPose fk_leg(const LegAnchors& a, double t1, double t2, double t3,
                      double suspend_lift_y) {
    const Vec3f lift(0.0f, float(suspend_lift_y), 0.0f);
    const Vec3f hip1_w  = a.hip1_world + lift;
    const Vec3f hip2_w  = a.hip2_world + lift;
    const Vec3f knee_w  = a.knee_world + lift;
    const Vec3f coxa_c  = a.coxa_rest_origin  + lift;
    const Vec3f upper_c = a.upper_rest_origin + lift;
    const Vec3f lower_c = a.lower_rest_origin + lift;

    // Hip1 rotation around world UP at the hip1 anchor.
    const Basis3f rot1{ Quatf(Vec3f(0.0f, 1.0f, 0.0f), float(t1)) };
    const Xform3f h1(rot1, hip1_w - rot1 * hip1_w);
    const Xform3f t_coxa = h1 * Xform3f(Basis3f(), coxa_c);

    // Hip2 rotation around the leg-local lateral -- which hip1 has rotated -- at the
    // hip2 anchor, which has also moved with hip1.
    const Vec3f   hip2_w_now    = h1 * hip2_w;
    const Vec3f   hip2_axis_now = rot1 * a.hip2_axis;
    const Basis3f rot2{ Quatf(hip2_axis_now, float(t2)) };
    const Xform3f h2(rot2, hip2_w_now - rot2 * hip2_w_now);
    const Xform3f t_upper = h2 * h1 * Xform3f(Basis3f(), upper_c);

    // Knee rotation -- the knee axis is carried by BOTH prior rotations.
    const Vec3f   knee_w_now    = h2 * h1 * knee_w;
    const Vec3f   knee_axis_now = rot2 * rot1 * a.knee_axis;
    const Basis3f rot3{ Quatf(knee_axis_now, float(t3)) };
    const Xform3f h3(rot3, knee_w_now - rot3 * knee_w_now);
    const Xform3f t_lower = h3 * h2 * h1 * Xform3f(Basis3f(), lower_c);

    return LegPose{ t_coxa, t_upper, t_lower };
}

}  // namespace ogma::body
