#pragma once
// =============================================================================
// StrideOdometry.hpp  --  GDScript binding for ogma::body::StrideOdometry
// =============================================================================
//
// Port doc Phase 4, Order step (b): the stance-FK velocity primitive, the servo
// forward model and the stride_v ⊕ slip fusion move out of picrawler_body.gd so the
// sim and ogma_host share one implementation.  Gate: the sim stays BYTE-IDENTICAL.
//
// A thin shim, and it must stay one -- all arithmetic lives in
// cpp_core/include/ogma/body/StrideOdometry.hpp, bit-verified against the GDScript
// original (cpp_core/tests/body/stride_odometry_parity_check.cpp, 900 steps, every
// field, all five stance branches).  Logic added HERE would sit outside that proof.
//
// ⚠ WHY THREE CLASSES AND NOT ONE.  The sim needs them at different granularities:
// the fusion is one object with carried state, but planted_foot_velocity is called
// for FOUR variants (three of which are sim-only diagnostics -- see the cpp_core
// header for the cut), so it is exposed as a stateless call the diagnostics can share
// rather than being buried inside the estimator.  Fusing them would force the sim to
// keep its own copy of the formula, which is the exact drift this port removes.
//
// ⚠ THESE CARRY STATE ACROSS TICKS, so unlike fk_leg they must be reset when the body
// is -- _do_hard_reset teleports the chassis and the previous toe positions become
// meaningless.  StrideVNode::reset() and ServoLag::reset() are what that hooks to, and
// picrawler_body.gd calls both from _do_hard_reset; skipping them integrates a teleport
// as velocity.  The sim keeps mirror copies of est/bias/slip for its trace and HUD --
// those are written FROM the filter, so resetting only the mirrors would show a clean
// zero while the estimator carried the discontinuity forward.

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "ogma/body/StrideOdometry.hpp"
#include "ogma/body/Distress.hpp"

namespace godot {

// The servo forward model: a first-order lag on the 12 effective joint targets.
class ServoLag : public RefCounted {
    GDCLASS(ServoLag, RefCounted)

public:
    bool   seeded() const;
    // ⚠ seed() and advance() are SEPARATE because the original's two halves sit under
    // different guards: it seeds whenever the block is reached, but only advances once
    // the toe offsets exist.  Collapsing them into one "seed on first use, step after"
    // call is the same thing ONLY IF those guards always agree -- which is an assumption
    // about construction order, not a fact the code states.  Kept apart so the port does
    // not depend on it.
    void   seed(const PackedFloat64Array& eff);
    // eff: 12 doubles, index = leg*3 + joint.
    void   advance(const PackedFloat64Array& eff, double alpha);
    double get(int k) const;
    void   reset();

protected:
    static void _bind_methods();

private:
    ogma::body::ServoForwardModel m_;
};

// The stride_v ⊕ slip PI complementary filter.
class StrideVNode : public RefCounted {
    GDCLASS(StrideVNode, RefCounted)

public:
    // accel is gravity-INCLUSIVE body frame; up is the FUSED estimate, never exact.
    Vector3 linear_accel(Vector3 accel, Vector3 up) const;
    // stance_sum is the SUM over planted feet (not a mean); stance_n = 0 coasts.
    void    step(Vector3 a_lin, Vector3 stance_sum, int stance_n, double tau);
    Vector2 est() const;
    Vector2 bias() const;
    double  slip() const;
    void    reset();
    void    configure(double fuse_beta, double bias_ki, double slip_alpha,
                      double coast_leak, double gravity);

protected:
    static void _bind_methods();

private:
    ogma::body::StrideV f_;
};

// The PERCH × STALL distress accumulator in the robot-computable form (odometry
// displacement × fused tilt).  Shared with ogma_host; see ogma/body/Distress.hpp.
class DistressNode : public RefCounted {
    GDCLASS(DistressNode, RefCounted)

public:
    double step(double v_right, double v_fwd, double heading, double tilt, double dt,
                int64_t tick);
    double value() const;
    double stuck_deficit() const;
    void   reset();

protected:
    static void _bind_methods();

private:
    ogma::body::DistressAccumulator d_;
};

// Stateless helpers, shared by the ported channel and the sim-only diagnostics.
class StrideMath : public RefCounted {
    GDCLASS(StrideMath, RefCounted)

public:
    // v_body = -(ṗ_body + ω × p̄_body) for a foot planted in the world.
    Vector3 planted_foot_velocity(Vector3 toe_now, Vector3 toe_prev,
                                  Vector3 gyro_mean, double tau) const;
    // foot·up - L3/2.  Both halves are contract: see the cpp_core header.
    double  feet_y_gravity(Vector3 foot_body, Vector3 up, double l3) const;
    // clamp(raw_m / stand_m, 0, 1) — the belly-ToF normalizer the height homeostat
    // rides.  Shared so the robot cannot divide by a different standing height.
    double  ground_clearance(double raw_m, double stand_m) const;
    // The BOOM-mounted belly ToF (BOM §9.1/§9.10).  `up` is the FUSED gravity estimate in
    // the body frame; `sensor_above_belly` is the level-pose sensor-to-belly height, which
    // on the robot IS the fitted tof.mount_offset_mm.
    //
    // ⚠ TWO PARAMETERS WHERE THE GDSCRIPT ORIGINAL USED THREE, and that is the fix, not a
    // simplification.  Taking the offset from the BELLY PLANE instead of the body origin
    // cancels the "how far is the belly below the origin" term exactly.  The original added
    // that term UN-projected (`+ _chassis_bottom_local` where the geometry needs
    // `+ bottom * up.y`), so the two agree at zero tilt -- where it was checked -- and
    // diverge by bottom*(1 - cos θ): ~-0.3 mm at 10°, ~-2.8 mm at 30°.
    double  ground_clearance_boom(double along_ray_m, Vector3 up,
                                  double sensor_above_belly_m, double boom_z_m) const;
    // What a driver ignoring attitude would publish.  Kept so the correction can be
    // MEASURED against it rather than assumed.
    double  ground_clearance_boom_uncomp(double along_ray_m, double sensor_above_belly_m) const;

protected:
    static void _bind_methods();
};

}  // namespace godot
