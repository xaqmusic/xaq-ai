#include "DeadReckon.hpp"

#include <godot_cpp/core/class_db.hpp>

namespace godot {

void DeadReckonNode::_bind_methods() {
    ClassDB::bind_method(D_METHOD("step", "v_x", "v_fwd", "yaw", "dt"), &DeadReckonNode::step);
    ClassDB::bind_method(D_METHOD("reset"), &DeadReckonNode::reset);
    ClassDB::bind_method(D_METHOD("x"), &DeadReckonNode::x);
    ClassDB::bind_method(D_METHOD("y"), &DeadReckonNode::y);
    ClassDB::bind_method(D_METHOD("yaw"), &DeadReckonNode::yaw);
    ClassDB::bind_method(D_METHOD("dist"), &DeadReckonNode::dist);
}

void   DeadReckonNode::step(double v_x, double v_fwd, double yaw, double dt) { d_.step(v_x, v_fwd, yaw, dt); }
void   DeadReckonNode::reset()      { d_.reset(); }
double DeadReckonNode::x()    const { return d_.x(); }
double DeadReckonNode::y()    const { return d_.y(); }
double DeadReckonNode::yaw()  const { return d_.yaw(); }
double DeadReckonNode::dist() const { return d_.dist(); }

}  // namespace godot
