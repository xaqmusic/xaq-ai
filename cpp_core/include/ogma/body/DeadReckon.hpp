#pragma once
// =============================================================================
// DeadReckon.hpp -- egocentric odometry from the body's own stride and heading,
//                   shared by the sim and the robot host
// =============================================================================
//
// S1 of the MicroDuck port plan (docs/plans-and-designs/picrawler_microduck_port_plan.md).
// The duck's loops (PlayLoop, BearingSeekLoop, the place map) want a pose the body
// dead-reckons for itself: `reality.proprio.odom` = [x m, y m, unwrapped yaw rad].  The
// PiCrawler has the two legal ingredients already -- `stride_v` (stance-leg kinematics
// fused with the IMU, StrideOdometry.hpp) and `ego_heading` (the gyro's yaw, integrated)
// -- and nothing that integrates them.  This does, and nothing else.
//
// THE FRAME.  The odom frame is the body frame at the last reset: x_o is the body's
// forward axis then (the sim's world +Z at spawn), y_o is the body's +X axis then, and
// yaw is the heading measured from x_o toward y_o.  With that choice the integrator
// needs no world axis at all:
//     dx_o = (v_fwd cos ψ − v_x sin ψ) dt
//     dy_o = (v_fwd sin ψ + v_x cos ψ) dt
// where v_fwd = stride_v[1] (body forward) and v_x = stride_v[0] (body +X).  The sim
// names stride_v[0] "v_right" with the leg-naming mirror recorded beside it (the
// picrawler's body +X is anatomically LEFT); here it is only ever "body +X", so the
// mirror never enters.  ψ is handed in already unwrapped (ego_heading accumulates
// increments); the integrator keeps it as given.
//
// Egocentric by construction: every input is a quantity the robot measures about
// itself, and the frame is its own past pose.  It drifts, exactly as dead reckoning
// does, and that drift is a fact a consumer must live with -- the god's-eye comparison
// lives in the sim's JSONL as an instrument and never on the bus.
//
// Width: double throughout.  There is no GDScript original to be byte-identical with,
// and the robot will run this header, not a port of it.

#include <cmath>

namespace ogma::body {

class DeadReckon {
public:
    // One tick.  v_x, v_fwd in m/s (stride_v[0], stride_v[1]); yaw in rad, unwrapped;
    // dt in s.  Integrates with the heading at the START of the tick, which is the
    // heading the stride estimate was formed under.
    void step(double v_x, double v_fwd, double yaw, double dt) {
        const double c = std::cos(yaw_), s = std::sin(yaw_);
        x_ += (v_fwd * c - v_x * s) * dt;
        y_ += (v_fwd * s + v_x * c) * dt;
        yaw_ = yaw;
        dist_ += std::sqrt(v_x * v_x + v_fwd * v_fwd) * dt;
    }
    void reset() { x_ = 0.0; y_ = 0.0; yaw_ = 0.0; dist_ = 0.0; }
    double x()    const { return x_; }
    double y()    const { return y_; }
    double yaw()  const { return yaw_; }
    double dist() const { return dist_; }   // path length walked, m (an instrument)

private:
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, dist_ = 0.0;
};

}  // namespace ogma::body
