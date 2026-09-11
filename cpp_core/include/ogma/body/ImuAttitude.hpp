#pragma once
// =============================================================================
// ImuAttitude.hpp -- the complementary gravity-up filter, shared by sim and host
// =============================================================================
//
// Port doc Phase 4, Order step (a).  The GDScript original is picrawler_body.gd's
// _imu_substep(); this is the half of it that BOTH bodies need, and the acceptance
// bar the plan sets is that the sim stays BYTE-IDENTICAL across the swap.
//
// WHAT IS HERE, AND WHAT DELIBERATELY IS NOT.  _imu_substep does two separable jobs:
//
//   1. It MODELS an IMU -- differentiating chassis world velocity, adding gravity,
//      rotating into the body frame, applying a DLPF and clipping at +-4 g.
//   2. It FILTERS the result into a gravity-up estimate.
//
// Only (2) is shared.  On hardware the chip does (1) in silicon: the ICM-20948 has
// its own analog anti-alias filter and its own full-scale clip, and there is no
// world velocity to differentiate.  Porting the model would mean shipping a
// simulation of a part we physically own.  So the model stays in GDScript, where it
// belongs, and this class takes the measured accel and gyro as INPUTS.
//
// ⚠ BYTE-IDENTITY IS A REAL CONSTRAINT AND IT DICTATES THE CODE BELOW.  Every
// operation replicates godot-cpp's exactly, because mathematically-equal is not
// bit-equal in float32:
//
//   * real_t is FLOAT (math_defs.hpp:77), so everything here is float, not double.
//   * Vector3::normalize() divides each component by the length (v.x /= len).  It
//     does NOT multiply by a reciprocal -- that rounds differently.
//   * Vector3::dot() sums strictly left to right: x*b.x + y*b.y + z*b.z.
//   * Basis(axis, angle) BUILDS A MATRIX via set_axis_angle and then xform()s,
//     three dots against its rows.  ⚠ A direct Rodrigues rotation is the same
//     rotation and a different number -- pi_host/Icm20948.cpp uses Rodrigues and is
//     therefore NOT a byte-identical substitute for this.  That driver predates this
//     header and should be moved onto it (its own header says so); until then the
//     two are knowingly separate and only this one carries the identity guarantee.
//   * The trust blend is written as explicit scalar multiply-and-add, NOT as lerp,
//     because the original is -- Vector3::lerp routes through Math::lerp and rounds
//     differently.
//
// No Eigen, no godot-cpp, no ogma_core: header-only and dependency-free on purpose,
// so pi_host can use it without taking on the brain (pi_host/CMakeLists.txt keeps
// ogma_hw independent of ogma_core so the driver builds in seconds).

#include <cmath>

#include "ogma/body/GodotFloat.hpp"

namespace ogma::body {

struct ImuAttitudeParams {
    // Mirrors IMU_ACC_TRUST / IMU_ACC_GATE_FRAC / the literal 9.81 in _imu_substep.
    //
    // ⚠ DOUBLE, because the GDScript constants are.  A `const float` of 9.81 is
    // 9.8100004196166992 once widened; GDScript's 9.81 is 9.8099999999999996, and
    // 0.02f vs 0.02 differ likewise.  Declaring these float made the filter diverge
    // by 1 ULP per step even though every operation was already bit-exact -- the
    // arithmetic was right and the CONSTANTS were a different number.
    double acc_trust     = 0.02;
    double acc_gate_frac = 0.5;
    double gravity       = 9.81;
};

// The filter.  One call per IMU substep; the caller owns the rate.
//
// The accelerometer only indicates "down" while the body is quasi-static -- during a
// footfall it is measuring the impact.  So its correction is WEIGHTED by how close
// |a| is to g rather than accepted or rejected outright: a hard gate starved this
// filter in sim, which is why the adaptive form is the one that shipped.
class ImuAttitude {
public:
    explicit ImuAttitude(ImuAttitudeParams p = {}) : p_(p) {}

    // accel_meas: m/s^2, body frame, gravity-inclusive (what an accelerometer reads).
    // gyro:       rad/s, body frame.  dt is DOUBLE -- see the note on scalars below.
    //
    // ⚠ SCALARS ARE DOUBLE HERE AND THAT IS NOT A STYLE CHOICE.  GDScript's `float`
    // is 64-bit.  Vector3 components are 32-bit, so a value like gyro.length() comes
    // back as float32 but is then held and combined in DOUBLE, and only narrows again
    // where it is handed to a Vector3/Basis call.  Doing these scalars in float32
    // instead diverges by 1 ULP per step: measured 735/800 steps mismatched, while
    // every pure-Vector3 result (up_accel: length, normalize, sqrt) stayed bit-exact.
    // So each scalar below is computed wide and narrowed exactly where Godot narrows.
    void step(const Vec3f& accel_meas, const Vec3f& gyro, double dt) {
        acc_mag_ = accel_meas.length();
        if (acc_mag_ > 1e-4f) up_acc_ = accel_meas.normalized();

        if (up_est_.length() < 0.5f) up_est_ = up_acc_;

        // Gyro propagation.  A WORLD-fixed direction seen from the body rotates by
        // -w*dt.  EXACT rotation, not the first-order v -= (w x v)*dt form, which
        // leaves O((w*dt)^2) per step and integrates to radians over a run.
        const double w_mag = double(gyro.length());       // float32 value, double storage
        if (w_mag > 1e-6) {
            // `gyro / w_mag`: Vector3::operator/ takes real_t, so the double narrows
            // first -- exactly, since w_mag holds a float32 value.  The ANGLE is the
            // one that matters: -w_mag * dt is a double product, narrowed on the call.
            const float angle = float(-w_mag * dt);
            up_est_ = basis_axis_angle_xform(gyro / float(w_mag), angle, up_est_).normalized();
        }

        const double g = p_.gravity;
        const double acc_dev = std::fabs(double(acc_mag_) - g) / g;
        double c = 1.0 - acc_dev / p_.acc_gate_frac;
        if (c < 0.0) c = 0.0;
        if (c > 1.0) c = 1.0;                   // clampf(x, 0, 1)
        const double trust = p_.acc_trust * c;
        trust_ = trust;
        if (trust > 0.0) {
            // Explicit mul-add, NOT lerp -- see the header note.  Vector3 * real_t
            // narrows each scalar on the call, so narrow here and nowhere earlier.
            up_est_ = (up_est_ * float(1.0 - trust) + up_acc_ * float(trust)).normalized();
        }
    }

    const Vec3f& up_fused() const { return up_est_; }
    const Vec3f& up_accel() const { return up_acc_; }
    double trust()  const { return trust_; }
    float acc_mag() const { return acc_mag_; }

    // THE health signal on hardware: there is no ground truth, so accel-vs-fused
    // disagreement is what says whether the filter is working.  Degrees.
    float disagree_deg() const {
        float d = up_est_.dot(up_acc_);
        if (d > 1.0f) d = 1.0f;
        if (d < -1.0f) d = -1.0f;
        return std::acos(d) * 57.29577951308232f;
    }

    void reset() { up_est_ = Vec3f(); up_acc_ = Vec3f(); trust_ = 0.0; acc_mag_ = 0.0f; }

private:
    ImuAttitudeParams p_;
    Vec3f up_est_{};
    Vec3f up_acc_{};
    double trust_ = 0.0;
    float acc_mag_ = 0.0f;
};

}  // namespace ogma::body
