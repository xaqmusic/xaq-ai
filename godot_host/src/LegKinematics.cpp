#include "LegKinematics.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace godot {

namespace {
inline ogma::body::Vec3f to_v(const Vector3& v) {
    return ogma::body::Vec3f(float(v.x), float(v.y), float(v.z));
}
inline Transform3D to_xform(const ogma::body::Xform3f& x) {
    Basis b;
    for (int r = 0; r < 3; ++r) {
        b.rows[r] = Vector3(x.basis.rows[r].x, x.basis.rows[r].y, x.basis.rows[r].z);
    }
    return Transform3D(b, Vector3(x.origin.x, x.origin.y, x.origin.z));
}
}  // namespace

void LegKinematics::_bind_methods() {
    ClassDB::bind_method(D_METHOD("clear"), &LegKinematics::clear);
    ClassDB::bind_method(D_METHOD("set_leg", "i", "hip1_world", "hip2_world", "knee_world",
                                  "coxa_rest_origin", "upper_rest_origin",
                                  "lower_rest_origin", "hip2_axis", "knee_axis"),
                         &LegKinematics::set_leg);
    ClassDB::bind_method(D_METHOD("fk", "i", "t1", "t2", "t3", "suspend_lift_y"),
                         &LegKinematics::fk);
    ClassDB::bind_method(D_METHOD("legs_set"), &LegKinematics::legs_set);
}

void LegKinematics::clear() {
    for (int i = 0; i < 4; ++i) have_[i] = false;
}

void LegKinematics::set_leg(int i, Vector3 hip1_world, Vector3 hip2_world, Vector3 knee_world,
                            Vector3 coxa_rest_origin, Vector3 upper_rest_origin,
                            Vector3 lower_rest_origin, Vector3 hip2_axis, Vector3 knee_axis) {
    if (i < 0 || i > 3) {
        UtilityFunctions::push_error("LegKinematics.set_leg: leg index out of range: ", i);
        return;
    }
    ogma::body::LegAnchors& a = a_[i];
    a.hip1_world        = to_v(hip1_world);
    a.hip2_world        = to_v(hip2_world);
    a.knee_world        = to_v(knee_world);
    a.coxa_rest_origin  = to_v(coxa_rest_origin);
    a.upper_rest_origin = to_v(upper_rest_origin);
    a.lower_rest_origin = to_v(lower_rest_origin);
    a.hip2_axis         = to_v(hip2_axis);
    a.knee_axis         = to_v(knee_axis);
    have_[i] = true;
}

int LegKinematics::legs_set() const {
    int n = 0;
    for (int i = 0; i < 4; ++i) if (have_[i]) ++n;
    return n;
}

Array LegKinematics::fk(int i, double t1, double t2, double t3, double suspend_lift_y) const {
    Array out;
    if (i < 0 || i > 3 || !have_[i]) {
        // Loud, not silent. A default pose here would be a body at the origin, which
        // reads downstream as a plausible robot rather than as a missing one.
        UtilityFunctions::push_error(
            "LegKinematics.fk: leg ", i, " has no anchors — _build_body() did not push them. "
            "A stale or empty cache returns poses for geometry that does not exist.");
        return out;
    }
    const ogma::body::LegPose p = ogma::body::fk_leg(a_[i], t1, t2, t3, suspend_lift_y);
    out.resize(3);
    out[0] = to_xform(p.coxa);
    out[1] = to_xform(p.upper);
    out[2] = to_xform(p.lower);
    return out;
}

}  // namespace godot
