#pragma once
// =============================================================================
// StrideOdometry.hpp -- stance-FK velocity + the stride_v ⊕ slip fusion,
//                       shared by sim and host
// =============================================================================
//
// Port doc Phase 4, Order step (b).  The GDScript original is the stride block in
// picrawler_body.gd::_step_one (~:6755-6850) plus the servo forward model at ~:6697.
// Same acceptance bar as steps (a): the sim stays BYTE-IDENTICAL across the swap.
//
// WHY THIS ONE MATTERS MORE THAN THE FIRST TWO.  fk_leg and ImuAttitude are
// stateless-per-call and near-stateless respectively.  This module ACCUMULATES --
// `est`, `bias` and `slip` carry forward every tick, and `stride_v` is consumed by
// MotorEPMv2 and GainEvolver.  A 1-ULP divergence therefore does not stay 1 ULP; it
// integrates.  That makes the byte-identity gate unusually sensitive here, which is
// a feature: if the port is wrong at all, the gate says so loudly.
//
// ---------------------------------------------------------------------------
// WHAT IS SHARED, AND WHAT DELIBERATELY IS NOT
// ---------------------------------------------------------------------------
// Same cut as ImuAttitude: share the ESTIMATOR, never the simulation of a part we
// physically own.  The sim computes four stride variants; only one of them is a
// thing the robot can build.
//
//   cmdlp  ✅ SHARED -- commanded angles through the servo forward model, stance
//             gated on published foot_load.  The only input set an encoder-less
//             robot has, and therefore the one `stride_v` is built from.
//   cmd    ❌ sim-only diagnostic -- raw commanded FK.  Kept in GDScript because its
//             VALUE is the comparison (r_tick ≈ 0.03), not the estimate.
//   meas   ❌ sim-only -- measured joint angles.  Hobby servos report nothing; there
//             is no hardware analogue to port.
//   tc     ❌ sim-only -- stance from TRUE physics contact.  God's-eye.
//   true   ❌ sim-only -- god's-eye chassis velocity.  The measurement TARGET, and
//             per the instrument's own contract it must never become an input.
//
// ⚠ But all four share ONE formula, so `planted_foot_velocity` below is public and
// the sim's diagnostics call it too.  The alternative -- a C++ copy for the ported
// variant and a GDScript copy for the diagnostics -- would leave two implementations
// of the expression this whole port exists to unify, and they would drift silently
// because only one of them is gated.
//
// ---------------------------------------------------------------------------
// ⚠ SCALAR WIDTH IS THE WHOLE DIFFICULTY (see GodotFloat.hpp, and ImuAttitude.hpp's
// account of losing a day to a `const float` 9.81).  The original mixes widths in a
// way that looks arbitrary and is not -- it follows from what each GDScript
// container stores:
//
//   Vector2 / Vector3 components -> float32 STORAGE, double ARITHMETIC between
//       stores.  So `Vector2(est.x + (a.x - bias.x)*tau, ...)` computes both
//       components in double and narrows at the constructor -- and nowhere earlier.
//   `Array[float]` elements      -> DOUBLE.  _strido_lp, _eff_target_* and
//       _foot_load_ema are all Array[float], so the servo forward model and the
//       stance test run entirely in double.
//   a bare `var x: float`        -> DOUBLE.  _stridev_slip is one, which is why slip
//       accumulates in double while est/bias accumulate in float32 -- beside each
//       other, in the same `if` block, at different widths.  That asymmetry is real.
// ---------------------------------------------------------------------------

#include <cmath>

#include "ogma/body/GodotFloat.hpp"

namespace ogma::body {

// --- the stance-FK primitive -------------------------------------------------
//
// For a foot PLANTED in the world, d/dt p_world = 0 gives, in the body frame:
//
//     v_body = -( ṗ_body + ω_body × p̄_body )
//
// The gyro term removes what body ROTATION does to a planted foot, leaving pure
// translation -- without it a tight circle reads as travel, which was the
// operator-agreed fork when this instrument was specified.
//
// `tau` is the BRAIN tick (50 Hz), not the physics step: ṗ is a per-tick difference.
// `gyro_mean` is averaged across the tick's physics substeps, because the last
// 240 Hz sample alone misrepresents a 20 ms displacement window mid-swing.
inline Vec3f planted_foot_velocity(const Vec3f& toe_now, const Vec3f& toe_prev,
                                   const Vec3f& gyro_mean, double tau) {
    // Transcribed left to right from:
    //   -((now - prev) / TAU + gyro_mean.cross((now + prev) * 0.5))
    const Vec3f d   = (toe_now - toe_prev) / float(tau);
    const Vec3f mid = (toe_now + toe_prev) * 0.5f;
    const Vec3f s   = d + gyro_mean.cross(mid);
    return Vec3f(-s.x, -s.y, -s.z);       // Vector3::operator-() is componentwise
}

// --- the servo forward model -------------------------------------------------
//
// Hobby servos report nothing, so the joint angle must be PREDICTED from the command.
// A first-order lag on the slew-limited effective target (α ≈ 0.2 ≙ ~80-100 ms) was
// measured to match the achieved angle at r 0.93-0.99 per joint.  Raw commanded
// angles instead give an FK velocity near-uncorrelated with truth (r_tick ≈ 0.03),
// so this lag is not polish -- it is what makes the channel work at all.
//
// ⚠ ALL DOUBLE.  _strido_lp is Array[float], and so are the effective targets.
//
// ⚠ THE LAG CONSTANT IS DISCOVERABLE INSIDE THE BLANKET (fit it to the FK/IMU
// disagreement), which is what keeps it on the right side of prohibition 5.  It is
// not a constant tuned to a signal's scale.
class ServoForwardModel {
public:
    // index = leg*3 + joint, matching _strido_lp.
    static constexpr int N = 12;

    bool seeded() const { return seeded_; }

    // First use seeds from the current effective target rather than from zero: a
    // lag starting at 0 would spend its first ~100 ms reporting a leg folded flat,
    // and those ticks feed a velocity estimate.
    void seed(const double eff[N]) {
        for (int k = 0; k < N; ++k) lp_[k] = eff[k];
        seeded_ = true;
    }

    void step(const double eff[N], double alpha) {
        for (int k = 0; k < N; ++k) lp_[k] += alpha * (eff[k] - lp_[k]);
    }

    double operator[](int k) const { return lp_[k]; }

    // Hard reset drops the model -- the next use re-seeds from the post-reset target.
    // The sim's teleport does exactly this, and must: a lag carried across it predicts
    // a pose from before the discontinuity, and the FK built on it still looks like a
    // plausible robot.
    void reset() { seeded_ = false; for (int k = 0; k < N; ++k) lp_[k] = 0.0; }

private:
    double lp_[N] = { 0.0 };
    bool   seeded_ = false;
};

// --- feet_y_gravity ----------------------------------------------------------
//
// Foot height along the gravity estimate, relative to the chassis rest frame.  Two
// lines in GDScript, and shared anyway because BOTH halves are contract rather than
// detail: which `up` is used decides whether the channel is legal (`_up_est_body`,
// the IMU's own fused estimate -- never the exact basis), and the `- L3*0.5` offset
// is what puts the toe rather than the shin's midpoint at the origin.  A hardware
// publisher that dropped the offset would emit a plausible, wrong, silently-accepted
// number into a PROMOTED MotorEPMv2 input.
//
// Returns double: the topic is a PackedFloat64Array.
inline double feet_y_gravity(const Vec3f& foot_body, const Vec3f& up, double l3) {
    return double(foot_body.dot(up)) - l3 * 0.5;
}

// --- stride_v ⊕ slip ---------------------------------------------------------

struct StrideVParams {
    // ⚠ DOUBLE, because the GDScript constants are.  ImuAttitude.hpp records what
    // declaring these `float` costs: 1 ULP per step, from correct arithmetic over
    // constants that were a different number.
    double fuse_beta  = 1.0;    // STRIDE_V_FUSE_BETA
    double bias_ki    = 0.1;    // STRIDE_V_BIAS_KI
    double slip_alpha = 0.05;   // STRIDE_V_SLIP_ALPHA
    double coast_leak = 0.005;  // STRIDE_V_COAST_LEAK
    double gravity    = 9.81;
};

// PI complementary filter over stance-FK and the accelerometer.
//
// Predict with the accelerometer's linear part (gravity removed via the HONEST fused
// attitude -- the IMU's own, never the exact basis) minus a LEARNED bias; correct
// toward stance-FK when stance feet exist, and let the innovation do double duty:
// it teaches the bias, and it IS `slip`.
//
// The three roles the IMU plays here were each measured, not assumed:
//   * bridging swing/flight ticks, where no stance estimate exists at all;
//   * the PI bias estimator, learning the accelerometer's attitude-leak -- without
//     it the mean is crushed (integrated travel 0.65 m of 3.57; with it ~0.85 of it);
//   * the innovation itself -- a foot sliding back reads as body-forward to FK but
//     not to the IMU, so the mismatch is re-afference, published as `slip`.
//
// β = 1.0 was chosen AGAINST lower values on measurement: β 0.3 buys per-tick r
// (0.75-0.81 vs 0.57) and costs the criterion band (r_w50 0.62-0.72 vs 0.71-0.79),
// and the criterion consumes ~1 s EMAs.  Re-use context for β≈0.3 + ki≈0.05: a
// consumer that needs the FAST band (reflexes, footfall-scale prediction).
//
// ⚠ NO CALIBRATION AGAINST TRUE SCALE IS APPLIED.  FK reads a stable ~75 % of truth
// and consumers adapt -- correcting it against a god's-eye ratio is exactly what
// hard prohibition 5 forbids.
class StrideV {
public:
    explicit StrideV(StrideVParams p = {}) : p_(p) {}

    // accel_meas: gravity-INCLUSIVE body-frame accelerometer reading, m/s².
    // up_est:     the fused gravity-up estimate (ImuAttitude::up_fused()).
    Vec3f linear_accel(const Vec3f& accel_meas, const Vec3f& up_est) const {
        // `_accel_body_last - 9.81 * _up_est_body`.  The scalar narrows at the
        // operator (godot-cpp has no Vector3::operator*(double)), so the multiply is
        // componentwise float32 -- NOT a double multiply narrowed afterwards.
        return accel_meas - up_est * float(p_.gravity);
    }

    // One brain tick.
    //
    //   a_lin        from linear_accel(), body frame
    //   stance_sum   the SUM of planted_foot_velocity over stance feet (not a mean);
    //                only .x and .z are read -- .y is vertical and unused
    //   stance_n     how many feet were planted at BOTH ends of the tick.  0 takes
    //                the coast branch.
    //   tau          the brain tick, 0.02
    //
    // ⚠ THE COAST BRANCH IS NOT AN EDGE CASE.  With no planted feet the FK anchor is
    // gone, and unleaked integration turns attitude error into phantom velocity
    // within seconds -- the shape a hardware audit fails on.  It fires on every
    // full-swing or airborne tick, so a parity oracle that never reaches stance_n = 0
    // has not tested this module (memory: the gate cannot see what it does not run).
    void step(const Vec3f& a_lin, const Vec3f& stance_sum, int stance_n, double tau) {
        // ⚠ Both components in DOUBLE, narrowed at the Vector2 constructor.
        //   Vector2(est.x + (a_lin.x - bias.x)*TAU,  est.y + (a_lin.z - bias.y)*TAU)
        // Note the axis mapping: est.y is FORWARD and pairs with a_lin.Z.
        const Vec2f v_pred(float(double(est_.x) + (double(a_lin.x) - double(bias_.x)) * tau),
                           float(double(est_.y) + (double(a_lin.z) - double(bias_.y)) * tau));

        if (stance_n > 0) {
            const Vec2f v_fk(float(double(stance_sum.x) / double(stance_n)),
                             float(double(stance_sum.z) / double(stance_n)));
            const Vec2f innov = v_fk - v_pred;                       // float32
            est_  = v_pred + innov * float(p_.fuse_beta);            // float32
            bias_ = bias_ + innov * float(-p_.bias_ki);              // float32
            // ⚠ DOUBLE -- _stridev_slip is a bare `var x: float`.  Right beside two
            // float32 accumulators, in the same branch.  See the header note.
            slip_ += p_.slip_alpha * (double(innov.length()) - slip_);
        } else {
            est_ = v_pred * float(1.0 - p_.coast_leak);
        }
    }

    const Vec2f& est()  const { return est_; }   // [x = right, y = forward] m/s
    const Vec2f& bias() const { return bias_; }  // learned accel bias [x, z] m/s²
    double       slip() const { return slip_; }

    void reset() { est_ = Vec2f(); bias_ = Vec2f(); slip_ = 0.0; }

private:
    StrideVParams p_;
    Vec2f  est_{};
    Vec2f  bias_{};
    double slip_ = 0.0;
};

}  // namespace ogma::body
