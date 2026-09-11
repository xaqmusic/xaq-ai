#pragma once
// =============================================================================
// ImuAttitude.hpp  --  GDScript binding for ogma::body::ImuAttitude
// =============================================================================
//
// Port doc Phase 4, Order step (a): the complementary gravity-up filter moves out of
// picrawler_body.gd and into cpp_core, and BOTH the sim and ogma_host call the same
// code.  The acceptance bar is that the sim stays BYTE-IDENTICAL across this swap.
//
// This is a thin shim and must stay one.  All of the arithmetic lives in
// cpp_core/include/ogma/body/ImuAttitude.hpp, which is bit-verified against the
// GDScript original (cpp_core/tests/body/imu_attitude_parity_check.cpp).  Adding logic
// here would put it outside that verification.
//
// ⚠ Vector3 crosses the binding as three float32s and dt as a double -- which is
// exactly the width GDScript already uses (its `float` is 64-bit, its Vector3
// components are 32-bit), so the transfer is lossless in both directions.  That is a
// requirement, not an accident: the whole point is that the caller cannot tell the
// maths moved.

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "ogma/body/ImuAttitude.hpp"

namespace godot {

class ImuAttitude : public RefCounted {
    GDCLASS(ImuAttitude, RefCounted)

public:
    // accel: m/s^2 body frame, gravity-inclusive.  gyro: rad/s body frame.
    void    step(Vector3 accel, Vector3 gyro, double dt);
    Vector3 up_fused() const;
    Vector3 up_accel() const;
    double  trust() const;
    double  acc_mag() const;
    double  disagree_deg() const;
    void    reset();
    // Defaults already mirror IMU_ACC_TRUST / IMU_ACC_GATE_FRAC / 9.81.  ⚠ These are
    // DOUBLE on purpose: GDScript's literals are, and a float32 9.81f is a different
    // number (9.8100004196166992), which costs 1 ULP per step.
    void    configure(double acc_trust, double acc_gate_frac, double gravity);

protected:
    static void _bind_methods();

private:
    ogma::body::ImuAttitude f_;
};

}  // namespace godot
