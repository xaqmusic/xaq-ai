#pragma once
// =============================================================================
// DeadReckon.hpp  --  GDScript binding for ogma::body::DeadReckon
// =============================================================================
//
// S1 of the MicroDuck port plan: the odometry the duck's loops want, dead-reckoned from
// stride_v and ego_heading.  A thin shim, like ImuAttitude: the arithmetic and the
// frame convention live in cpp_core/include/ogma/body/DeadReckon.hpp, which the robot
// host will run unchanged.

#include <godot_cpp/classes/ref_counted.hpp>

#include "ogma/body/DeadReckon.hpp"

namespace godot {

class DeadReckonNode : public RefCounted {
    GDCLASS(DeadReckonNode, RefCounted)

public:
    void   step(double v_x, double v_fwd, double yaw, double dt);
    void   reset();
    double x() const;
    double y() const;
    double yaw() const;
    double dist() const;

protected:
    static void _bind_methods();

private:
    ogma::body::DeadReckon d_;
};

}  // namespace godot
