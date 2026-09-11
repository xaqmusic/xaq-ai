#include "ImuAttitude.hpp"

#include <godot_cpp/core/class_db.hpp>

namespace godot {

void ImuAttitude::_bind_methods() {
    ClassDB::bind_method(D_METHOD("step", "accel", "gyro", "dt"), &ImuAttitude::step);
    ClassDB::bind_method(D_METHOD("up_fused"), &ImuAttitude::up_fused);
    ClassDB::bind_method(D_METHOD("up_accel"), &ImuAttitude::up_accel);
    ClassDB::bind_method(D_METHOD("trust"), &ImuAttitude::trust);
    ClassDB::bind_method(D_METHOD("acc_mag"), &ImuAttitude::acc_mag);
    ClassDB::bind_method(D_METHOD("disagree_deg"), &ImuAttitude::disagree_deg);
    ClassDB::bind_method(D_METHOD("reset"), &ImuAttitude::reset);
    ClassDB::bind_method(D_METHOD("configure", "acc_trust", "acc_gate_frac", "gravity"),
                         &ImuAttitude::configure);
}

void ImuAttitude::step(Vector3 accel, Vector3 gyro, double dt) {
    f_.step(ogma::body::Vec3f(accel.x, accel.y, accel.z),
            ogma::body::Vec3f(gyro.x, gyro.y, gyro.z), dt);
}

Vector3 ImuAttitude::up_fused() const {
    const auto& v = f_.up_fused();
    return Vector3(v.x, v.y, v.z);
}

Vector3 ImuAttitude::up_accel() const {
    const auto& v = f_.up_accel();
    return Vector3(v.x, v.y, v.z);
}

double ImuAttitude::trust()        const { return f_.trust(); }
double ImuAttitude::acc_mag()      const { return f_.acc_mag(); }
double ImuAttitude::disagree_deg() const { return f_.disagree_deg(); }
void   ImuAttitude::reset()              { f_.reset(); }

void ImuAttitude::configure(double acc_trust, double acc_gate_frac, double gravity) {
    ogma::body::ImuAttitudeParams p;
    p.acc_trust = acc_trust;
    p.acc_gate_frac = acc_gate_frac;
    p.gravity = gravity;
    f_ = ogma::body::ImuAttitude(p);
}

}  // namespace godot
