#include "StrideOdometry.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace godot {

namespace {
inline ogma::body::Vec3f to_v(const Vector3& v) {
    return ogma::body::Vec3f(float(v.x), float(v.y), float(v.z));
}
}  // namespace

// --- ServoLag ----------------------------------------------------------------

void ServoLag::_bind_methods() {
    ClassDB::bind_method(D_METHOD("seeded"), &ServoLag::seeded);
    ClassDB::bind_method(D_METHOD("seed", "eff"), &ServoLag::seed);
    ClassDB::bind_method(D_METHOD("advance", "eff", "alpha"), &ServoLag::advance);
    ClassDB::bind_method(D_METHOD("get", "k"), &ServoLag::get);
    ClassDB::bind_method(D_METHOD("reset"), &ServoLag::reset);
}

bool ServoLag::seeded() const { return m_.seeded(); }

namespace {
// Loud, not silent: a short array would leave stale joints lagging on a pose the body
// has left, and the FK built from them still looks like a robot.
bool unpack12(const PackedFloat64Array& eff, double* buf, const char* who) {
    if (eff.size() != ogma::body::ServoForwardModel::N) {
        UtilityFunctions::push_error(
            who, ": expected ", int(ogma::body::ServoForwardModel::N),
            " effective targets, got ", int(eff.size()));
        return false;
    }
    for (int k = 0; k < ogma::body::ServoForwardModel::N; ++k) buf[k] = eff[k];
    return true;
}
}  // namespace

void ServoLag::seed(const PackedFloat64Array& eff) {
    double buf[ogma::body::ServoForwardModel::N];
    if (unpack12(eff, buf, "ServoLag.seed")) m_.seed(buf);
}

void ServoLag::advance(const PackedFloat64Array& eff, double alpha) {
    double buf[ogma::body::ServoForwardModel::N];
    if (unpack12(eff, buf, "ServoLag.advance")) m_.step(buf, alpha);
}

double ServoLag::get(int k) const {
    if (k < 0 || k >= ogma::body::ServoForwardModel::N) {
        UtilityFunctions::push_error("ServoLag.get: index out of range: ", k);
        return 0.0;
    }
    return m_[k];
}

void ServoLag::reset() { m_.reset(); }

// --- StrideVNode -------------------------------------------------------------

void StrideVNode::_bind_methods() {
    ClassDB::bind_method(D_METHOD("linear_accel", "accel", "up"), &StrideVNode::linear_accel);
    ClassDB::bind_method(D_METHOD("step", "a_lin", "stance_sum", "stance_n", "tau"),
                         &StrideVNode::step);
    ClassDB::bind_method(D_METHOD("est"), &StrideVNode::est);
    ClassDB::bind_method(D_METHOD("bias"), &StrideVNode::bias);
    ClassDB::bind_method(D_METHOD("slip"), &StrideVNode::slip);
    ClassDB::bind_method(D_METHOD("reset"), &StrideVNode::reset);
    ClassDB::bind_method(D_METHOD("configure", "fuse_beta", "bias_ki", "slip_alpha",
                                  "coast_leak", "gravity"),
                         &StrideVNode::configure);
}

Vector3 StrideVNode::linear_accel(Vector3 accel, Vector3 up) const {
    const ogma::body::Vec3f a = f_.linear_accel(to_v(accel), to_v(up));
    return Vector3(a.x, a.y, a.z);
}

void StrideVNode::step(Vector3 a_lin, Vector3 stance_sum, int stance_n, double tau) {
    f_.step(to_v(a_lin), to_v(stance_sum), stance_n, tau);
}

Vector2 StrideVNode::est()  const { return Vector2(f_.est().x,  f_.est().y); }
Vector2 StrideVNode::bias() const { return Vector2(f_.bias().x, f_.bias().y); }
double  StrideVNode::slip() const { return f_.slip(); }
void    StrideVNode::reset()      { f_.reset(); }

void StrideVNode::configure(double fuse_beta, double bias_ki, double slip_alpha,
                            double coast_leak, double gravity) {
    ogma::body::StrideVParams p;
    p.fuse_beta  = fuse_beta;
    p.bias_ki    = bias_ki;
    p.slip_alpha = slip_alpha;
    p.coast_leak = coast_leak;
    p.gravity    = gravity;
    // ⚠ Rebuilding the filter DROPS est/bias/slip.  That is correct for a parameter
    // change (the learned bias belongs to the old gains) and is why configure() is
    // called at construction, not mid-run.
    f_ = ogma::body::StrideV(p);
}

// --- StrideMath --------------------------------------------------------------

void StrideMath::_bind_methods() {
    ClassDB::bind_method(D_METHOD("planted_foot_velocity", "toe_now", "toe_prev",
                                  "gyro_mean", "tau"),
                         &StrideMath::planted_foot_velocity);
    ClassDB::bind_method(D_METHOD("feet_y_gravity", "foot_body", "up", "l3"),
                         &StrideMath::feet_y_gravity);
}

Vector3 StrideMath::planted_foot_velocity(Vector3 toe_now, Vector3 toe_prev,
                                          Vector3 gyro_mean, double tau) const {
    const ogma::body::Vec3f v = ogma::body::planted_foot_velocity(
        to_v(toe_now), to_v(toe_prev), to_v(gyro_mean), tau);
    return Vector3(v.x, v.y, v.z);
}

double StrideMath::feet_y_gravity(Vector3 foot_body, Vector3 up, double l3) const {
    return ogma::body::feet_y_gravity(to_v(foot_body), to_v(up), l3);
}

}  // namespace godot
