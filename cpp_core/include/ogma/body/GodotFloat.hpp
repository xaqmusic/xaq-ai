#pragma once
// =============================================================================
// GodotFloat.hpp -- float32 Vector3 / Quaternion / Basis / Transform3D whose
//                   arithmetic matches godot-cpp's BIT FOR BIT
// =============================================================================
//
// Shared substrate for cpp_core/include/ogma/body/*.  The point of these types is
// not convenience -- Eigen would be more convenient -- it is that a port of GDScript
// body maths must produce the SAME NUMBERS, not merely the same rotations, or the
// sim moves when we swap it in (port doc Phase 4: byte-identity is the gain-0 gate).
//
// ⚠ EVERY FUNCTION HERE IS TRANSCRIBED, NOT DERIVED.  The source and line are cited
// at each one.  Do not "simplify" any of them: a mathematically-equal rewrite is a
// different float, and that is not a hypothetical -- an earlier Rodrigues rotation
// here was exactly equal in maths and wrong in bits (see ImuAttitude.hpp).  Three
// rules the transcription depends on:
//
//   * real_t is FLOAT (math_defs.hpp:77).  Everything is float, never double.
//   * Vector3::normalize() DIVIDES each component by the length; it does not
//     multiply by a reciprocal.
//   * Dots sum strictly left to right, and Transform3D::operator*= computes the new
//     origin with the OLD basis before multiplying the basis.  Order is semantics.
//
// ⚠ BUILD WITH -ffp-contract=off, OR THE ROBOT AND THE SIM DISAGREE.  On aarch64 GCC
// fuses `a*b+c` into a single FMA by default; baseline x86-64 has no FMA and cannot, so
// the same source produces different floats on the two machines.  Measured 2026-09-12,
// the Pi against x86-generated oracles: by default ALL THREE helpers mismatch
// (leg_kinematics 400/400, worst 3.1e-07; imu_attitude 796/800, worst 6.3e-07;
// stride_odometry 900/900).  With -ffp-contract=off every one is bit-exact, worst
// |delta| 0.000e+00.
//
// ⚠ This CORRECTS the port doc's earlier reading that "no compiler flag buys cross-arch
// bit-parity".  That was measured on RunTumbleNavV2 — 4 000 stochastic steps of DOUBLE
// precision leaning on libm — where per-architecture libm does remain after contraction
// is disabled.  These helpers are float32 and their trig agreed across the pair as soon
// as FMA was off, so for ogma::body contraction was the WHOLE difference.  Parity by
// construction is therefore actually available here; it just has to be asked for.
// The flag is set in cpp_core/CMakeLists.txt and pi_host/CMakeLists.txt.
//
// ⚠ SCALARS THAT COME FROM GDScript ARE DOUBLE.  GDScript's `float` is 64-bit, so a
// value like an angle or a constant is held and combined in double and only narrows
// where it is handed to one of these calls.  Callers must narrow at the boundary and
// not before; getting this wrong costs 1 ULP per step (measured).

#include <cmath>

namespace ogma::body {

// --- Vector3 -----------------------------------------------------------------
struct Vec3f {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3f() = default;
    Vec3f(float ax, float ay, float az) : x(ax), y(ay), z(az) {}

    float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }

    // vector3.hpp:208
    float dot(const Vec3f& b) const { return x * b.x + y * b.y + z * b.z; }
    // vector3.hpp:199 -- each component is one multiply MINUS one multiply, in that
    // order.  Not a determinant expansion, not FMA-able: see the note on operator/.
    Vec3f cross(const Vec3f& b) const {
        return Vec3f((y * b.z) - (z * b.y),
                     (z * b.x) - (x * b.z),
                     (x * b.y) - (y * b.x));
    }
    // vector3.hpp:488 -- squares first, then sums, then sqrt
    float length() const {
        const float x2 = x * x, y2 = y * y, z2 = z * z;
        return std::sqrt(x2 + y2 + z2);
    }
    float length_squared() const { return x * x + y * y + z * z; }
    // vector3.hpp:504 -- per-component DIVISION, and zero stays zero
    Vec3f normalized() const {
        Vec3f v = *this;
        const float lsq = v.length_squared();
        if (lsq == 0.0f) { v.x = v.y = v.z = 0.0f; }
        else {
            const float len = std::sqrt(lsq);
            v.x /= len; v.y /= len; v.z /= len;
        }
        return v;
    }
    Vec3f operator*(float s) const { return Vec3f(x * s, y * s, z * s); }
    Vec3f operator/(float s) const { return Vec3f(x / s, y / s, z / s); }
    Vec3f operator+(const Vec3f& b) const { return Vec3f(x + b.x, y + b.y, z + b.z); }
    Vec3f operator-(const Vec3f& b) const { return Vec3f(x - b.x, y - b.y, z - b.z); }
};

// --- Vector2 -----------------------------------------------------------------
// The stride_v fusion state (_stridev_est / _stridev_bias) is a GDScript Vector2, so
// it is float32 storage with double-width arithmetic BETWEEN the stores.  That split
// is the whole reason this type exists rather than a std::pair<double,double>:
// narrowing at the constructor is what the original does, and doing it anywhere else
// is a different number.
struct Vec2f {
    float x = 0.0f, y = 0.0f;

    Vec2f() = default;
    Vec2f(float ax, float ay) : x(ax), y(ay) {}

    // vector2.cpp:46 -- ⚠ sqrt(x*x + y*y) DIRECTLY, unlike Vector3::length() which
    // squares into temporaries first.  The two are not interchangeable in bits.
    float length() const { return std::sqrt(x * x + y * y); }

    Vec2f operator+(const Vec2f& b) const { return Vec2f(x + b.x, y + b.y); }
    Vec2f operator-(const Vec2f& b) const { return Vec2f(x - b.x, y - b.y); }
    Vec2f operator*(float s) const { return Vec2f(x * s, y * s); }
};

// --- Quaternion --------------------------------------------------------------
struct Quatf {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

    Quatf() = default;
    Quatf(float ax, float ay, float az, float aw) : x(ax), y(ay), z(az), w(aw) {}

    // quaternion.cpp Quaternion(axis, angle).  ⚠ sin/cos of angle*0.5f, then
    // s = sin_angle / d where d is the axis LENGTH -- it does not assume unit axis.
    Quatf(const Vec3f& axis, float angle) {
        const float d = axis.length();
        if (d == 0.0f) { x = 0.0f; y = 0.0f; z = 0.0f; w = 0.0f; }
        else {
            const float sin_angle = std::sin(angle * 0.5f);
            const float cos_angle = std::cos(angle * 0.5f);
            const float s = sin_angle / d;
            x = axis.x * s; y = axis.y * s; z = axis.z * s; w = cos_angle;
        }
    }
    // quaternion.hpp:174 / :178
    float dot(const Quatf& q) const { return x * q.x + y * q.y + z * q.z + w * q.w; }
    float length_squared() const { return dot(*this); }
};

// --- Basis -------------------------------------------------------------------
struct Basis3f {
    Vec3f rows[3];

    // Identity, as Godot's default constructor gives.
    Basis3f() : rows{ Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1) } {}
    Basis3f(float xx, float xy, float xz,
            float yx, float yy, float yz,
            float zx, float zy, float zz)
        : rows{ Vec3f(xx, xy, xz), Vec3f(yx, yy, yz), Vec3f(zx, zy, zz) } {}

    // basis.cpp set_quaternion
    explicit Basis3f(const Quatf& q) {
        const float d = q.length_squared();
        const float s = 2.0f / d;
        const float xs = q.x * s,  ys = q.y * s,  zs = q.z * s;
        const float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
        const float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
        const float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
        rows[0] = Vec3f(1.0f - (yy + zz), xy - wz,          xz + wy);
        rows[1] = Vec3f(xy + wz,          1.0f - (xx + zz), yz - wx);
        rows[2] = Vec3f(xz - wy,          yz + wx,          1.0f - (xx + yy));
    }

    // basis.hpp:115-123 -- COLUMN dots
    float tdotx(const Vec3f& v) const { return rows[0][0] * v[0] + rows[1][0] * v[1] + rows[2][0] * v[2]; }
    float tdoty(const Vec3f& v) const { return rows[0][1] * v[0] + rows[1][1] * v[1] + rows[2][1] * v[2]; }
    float tdotz(const Vec3f& v) const { return rows[0][2] * v[0] + rows[1][2] * v[1] + rows[2][2] * v[2]; }

    // basis.hpp:308 -- three ROW dots
    Vec3f xform(const Vec3f& v) const {
        return Vec3f(rows[0].dot(v), rows[1].dot(v), rows[2].dot(v));
    }
    Vec3f operator*(const Vec3f& v) const { return xform(v); }

    // basis.hpp:253
    Basis3f operator*(const Basis3f& m) const {
        return Basis3f(m.tdotx(rows[0]), m.tdoty(rows[0]), m.tdotz(rows[0]),
                       m.tdotx(rows[1]), m.tdoty(rows[1]), m.tdotz(rows[1]),
                       m.tdotx(rows[2]), m.tdoty(rows[2]), m.tdotz(rows[2]));
    }
};

// --- Transform3D -------------------------------------------------------------
struct Xform3f {
    Basis3f basis;
    Vec3f   origin;

    Xform3f() = default;
    Xform3f(const Basis3f& b, const Vec3f& o) : basis(b), origin(o) {}

    // transform3d.hpp:135
    Vec3f xform(const Vec3f& v) const {
        return Vec3f(basis.rows[0].dot(v) + origin.x,
                     basis.rows[1].dot(v) + origin.y,
                     basis.rows[2].dot(v) + origin.z);
    }
    Vec3f operator*(const Vec3f& v) const { return xform(v); }

    // transform3d.cpp operator*= : ⚠ ORIGIN FIRST, using the OLD basis.
    Xform3f operator*(const Xform3f& t) const {
        Xform3f r = *this;
        r.origin = r.xform(t.origin);
        r.basis  = r.basis * t.basis;
        return r;
    }
};

// Basis(axis, angle).xform(v) -- basis.cpp set_axis_angle then basis.hpp:308.
// ⚠ NOT Rodrigues.  Same rotation, different float; see the header note.
inline Vec3f basis_axis_angle_xform(const Vec3f& axis, float angle, const Vec3f& v) {
    const Vec3f axis_sq(axis.x * axis.x, axis.y * axis.y, axis.z * axis.z);
    const float cosine = std::cos(angle);
    const float r00 = axis_sq.x + cosine * (1.0f - axis_sq.x);
    const float r11 = axis_sq.y + cosine * (1.0f - axis_sq.y);
    const float r22 = axis_sq.z + cosine * (1.0f - axis_sq.z);
    const float sine = std::sin(angle);
    const float t = 1 - cosine;
    float xyzt = axis.x * axis.y * t;
    float zyxs = axis.z * sine;
    const float r01 = xyzt - zyxs, r10 = xyzt + zyxs;
    xyzt = axis.x * axis.z * t;
    zyxs = axis.y * sine;
    const float r02 = xyzt + zyxs, r20 = xyzt - zyxs;
    xyzt = axis.y * axis.z * t;
    zyxs = axis.x * sine;
    const float r12 = xyzt - zyxs, r21 = xyzt + zyxs;
    return Vec3f(Vec3f(r00, r01, r02).dot(v),
                 Vec3f(r10, r11, r12).dot(v),
                 Vec3f(r20, r21, r22).dot(v));
}

}  // namespace ogma::body
