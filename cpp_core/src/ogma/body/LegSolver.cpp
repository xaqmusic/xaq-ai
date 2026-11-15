// LegSolver -- see the header.  Every expression below is PoseSequencer's (2026-11-14), moved: the golden traces are the gate.
#include "ogma/body/LegSolver.hpp"
#include <algorithm>
#include <cmath>

namespace ogma::body {

namespace {   // PoseSequencer.cpp's helpers, the same arithmetic
Vec3f sub(const Vec3f& a, const Vec3f& b) { return Vec3f(a.x - b.x, a.y - b.y, a.z - b.z); }
double len(const Vec3f& a) { return std::sqrt(double(a.x) * a.x + double(a.y) * a.y + double(a.z) * a.z); }
}

bool LegSolver::set_model(const float* v, std::size_t n) {
    if (n < std::size_t(kModelFloats)) return false;
    auto vec = [&](int k) { return Vec3f(v[k], v[k + 1], v[k + 2]); };
    for (int i = 0; i < 4; ++i) {
        const int b = 27 * i;
        auto& a = anchors_[std::size_t(i)];
        a.hip1_world = vec(b); a.hip2_world = vec(b + 3); a.knee_world = vec(b + 6);
        a.coxa_rest_origin = vec(b + 9); a.upper_rest_origin = vec(b + 12); a.lower_rest_origin = vec(b + 15);
        a.hip2_axis = vec(b + 18); a.knee_axis = vec(b + 21); toe_off_[std::size_t(i)] = vec(b + 24);
    }
    const int m = 108;
    hip1_rest_ = v[m]; hip1_range_ = v[m + 1]; hip2_rest_ = v[m + 2]; hip2_range_ = v[m + 3];
    knee_rest_ = v[m + 4]; knee_fold_ = v[m + 5]; knee_hyper_ = v[m + 6]; knee_widening_ = v[m + 7] > 0.5f;
    have_model_ = true;
    if (!ext_lmax_ok_) {   // the extension margin's reference, once: each leg's longest hip2-to-toe over the knee's range (rigid under hip1 and hip2)
        const double klo = knee_rest_ - (knee_widening_ ? knee_hyper_ : hip2_range_), khi = knee_rest_ + (knee_widening_ ? knee_fold_ : hip2_range_);
        for (int i = 0; i < 4; ++i) {
            double best = 0.0;
            for (int j = 0; j <= 256; ++j) { const double k = klo + (khi - klo) * j / 256.0; best = std::max(best, double(len(sub(toe_of(i, {0.0, 0.0, k}), anchors_[std::size_t(i)].hip2_world)))); }
            ext_lmax_[std::size_t(i)] = best;
        }
        ext_lmax_ok_ = true;
    }
    return true;
}

Vec3f LegSolver::toe_of(int leg, const std::array<double, 3>& t) const {
    const auto pose = fk_leg(anchors_[std::size_t(leg)], t[0], t[1], t[2], 0.0);
    return pose.lower.xform(toe_off_[std::size_t(leg)]);
}
std::array<double, 3> LegSolver::angles_of_u(int leg, const std::array<double, 3>& u) const {
    (void)leg;
    const double kr = knee_widening_ ? (u[2] >= 0.0 ? knee_fold_ : knee_hyper_) : hip2_range_;
    return {u[0] * hip1_range_ + hip1_rest_, u[1] * hip2_range_ + hip2_rest_, u[2] * kr + knee_rest_};
}
std::array<double, 3> LegSolver::u_of_angles(int leg, const std::array<double, 3>& t) const {
    (void)leg;
    const double dk = t[2] - knee_rest_;
    const double kr = knee_widening_ ? (dk >= 0.0 ? knee_fold_ : knee_hyper_) : hip2_range_;
    return {std::clamp((t[0] - hip1_rest_) / hip1_range_, -1.0, 1.0), std::clamp((t[1] - hip2_rest_) / hip2_range_, -1.0, 1.0), std::clamp(dk / kr, -1.0, 1.0)};
}
// one damped least-squares step toward `target`: dt = J^T (J J^T + lambda^2 I)^-1 e, J by central differences, the step bounded
bool LegSolver::ik_step(int leg, const Vec3f& target, std::array<double, 3>& t) const {
    const Vec3f cur = toe_of(leg, t);
    const Vec3f e = sub(target, cur);
    const double en = len(e);
    if (en < p.ik_tol_m) return true;
    double J[3][3]; const double h = 1e-3;
    for (int j = 0; j < 3; ++j) {
        auto tp = t, tm = t; tp[std::size_t(j)] += h; tm[std::size_t(j)] -= h;
        const Vec3f d = sub(toe_of(leg, tp), toe_of(leg, tm));
        J[0][j] = d.x / (2 * h); J[1][j] = d.y / (2 * h); J[2][j] = d.z / (2 * h);
    }
    // A = J J^T + lambda^2 I (3x3), solve A y = e, dt = J^T y
    const double l2 = p.ik_damping_m * p.ik_damping_m;
    double A[9];
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) { double s = 0; for (int k = 0; k < 3; ++k) s += J[r][k] * J[c][k]; A[r * 3 + c] = s + (r == c ? l2 : 0.0); }
    const double det = A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) + A[2] * (A[3] * A[7] - A[4] * A[6]);
    if (std::fabs(det) < 1e-18) return false;
    const double id = 1.0 / det;
    const double Ai[9] = { (A[4] * A[8] - A[5] * A[7]) * id, -(A[1] * A[8] - A[2] * A[7]) * id,  (A[1] * A[5] - A[2] * A[4]) * id,
                          -(A[3] * A[8] - A[5] * A[6]) * id,  (A[0] * A[8] - A[2] * A[6]) * id, -(A[0] * A[5] - A[2] * A[3]) * id,
                           (A[3] * A[7] - A[4] * A[6]) * id, -(A[0] * A[7] - A[1] * A[6]) * id,  (A[0] * A[4] - A[1] * A[3]) * id };
    const double ev[3] = {e.x, e.y, e.z};
    double y[3]; for (int r = 0; r < 3; ++r) y[r] = Ai[r * 3] * ev[0] + Ai[r * 3 + 1] * ev[1] + Ai[r * 3 + 2] * ev[2];
    double dt[3]; double mx = 0.0;
    for (int j = 0; j < 3; ++j) { dt[j] = J[0][j] * y[0] + J[1][j] * y[1] + J[2][j] * y[2]; mx = std::max(mx, std::fabs(dt[j])); }
    const double sc = mx > p.ik_step_max_rad ? p.ik_step_max_rad / mx : 1.0;
    for (int j = 0; j < 3; ++j) t[std::size_t(j)] += dt[j] * sc;
    // joint limits, in action units
    const auto u = u_of_angles(leg, t); t = angles_of_u(leg, u);
    return false;
}

bool LegSolver::ik_solve(int leg, const Vec3f& target, std::array<double, 3>& t, bool allow_multistart) {
    bool done = false;
    for (int k = 0; k < p.ik_iters && !done; ++k) done = ik_step(leg, target, t);
    if (done || !allow_multistart) return done;
    const double e_cur = len(sub(toe_of(leg, t), target));
    if (e_cur <= 2.0 * p.ik_tol_m) return false;
    ++multistarts;
    // the seeds: the branches of a three-joint leg, in action units
    static const std::array<std::array<double, 3>, 6> seeds = {{{0.0, 0.0, 0.3}, {0.0, -0.6, 0.3}, {0.0, 0.6, 0.3}, {0.0, -0.3, -0.6}, {0.0, 0.0, 0.8}, {0.0, -0.6, -0.3}}};
    std::array<double, 3> best = t; double e_best = e_cur;
    const auto u_cur = u_of_angles(leg, t);
    for (auto const& su : seeds) {
        std::array<double, 3> a = angles_of_u(leg, {u_cur[0], su[1], su[2]});   // the yaw stays: the branches are in the leg's plane
        bool ok = false; for (int k = 0; k < 12 && !ok; ++k) ok = ik_step(leg, target, a);
        const double e = len(sub(toe_of(leg, a), target));
        if (e < e_best - 1e-9) { e_best = e; best = a; }
    }
    if (e_best < e_cur - p.ik_switch_m) { t = best; ++branch_switches; return e_best < p.ik_tol_m; }
    return false;
}

// the leg's workspace: a sector about its neutral heading (hip1 -> hip2) from its hip, radius in [reach_min, reach_max]
bool LegSolver::sector_clamp(int leg, Vec3f& pt, double half_rad) const {
    const Vec3f hip = hip1(leg); const Vec3f h2 = hip2(leg);
    const double hx = h2.x - hip.x, hz = h2.z - hip.z, hn = std::hypot(hx, hz);
    if (hn < 1e-6) return false;
    const double ux = hx / hn, uz = hz / hn;                      // the neutral heading
    double dx = pt.x - hip.x, dz = pt.z - hip.z; const double r = std::hypot(dx, dz);
    if (r < 1e-9) { pt.x = float(hip.x + ux * p.reach_min_m); pt.z = float(hip.z + uz * p.reach_min_m); return true; }
    const double ang = std::atan2(dx * uz - dz * ux, dx * ux + dz * uz);   // signed angle from the heading
    const double a = std::clamp(ang, -half_rad, half_rad), rr = std::clamp(r, p.reach_min_m, p.reach_max_m);
    const bool clamped = std::fabs(a - ang) > 1e-9 || std::fabs(rr - r) > 1e-9;
    if (!clamped) return false;
    const double c = std::cos(a), sn = std::sin(a);               // the heading rotated by a, in the x-z plane
    const double vx = ux * c + uz * sn, vz = uz * c - ux * sn;
    const double nx_ = hip.x + vx * rr, nz_ = hip.z + vz * rr;
    const bool moved = std::hypot(nx_ - pt.x, nz_ - pt.z) > 1e-4;    // blocked means the clamp took more than a tenth of a millimetre back
    pt.x = float(nx_); pt.z = float(nz_);
    return moved;
}

// the extension margin: the leg's full extension minus the hip2-to-toe distance for a toe at pt, the leg's plane taken through the
// hip1 axis (hip2 sits L1h out from hip1 along the toe's bearing, at its fixed height)
double LegSolver::ext_slack(int leg, const Vec3f& pt) const {
    if (!ext_lmax_ok_) return 1.0;
    const Vec3f hip = hip1(leg), h2 = hip2(leg);
    const double l1h = std::hypot(double(h2.x) - hip.x, double(h2.z) - hip.z), r = std::hypot(double(pt.x) - hip.x, double(pt.z) - hip.z);
    return ext_lmax_[std::size_t(leg)] - std::hypot(r - l1h, double(pt.y) - h2.y);
}
// the same at the joint angles by the true FK (an instrument: checks the plane approximation against the solved leg)
double LegSolver::ext_slack_fk(int leg, const std::array<double, 3>& t) const {
    if (!ext_lmax_ok_) return 1.0;
    const Vec3f hip = hip1(leg), h2 = hip2(leg);
    const double c = std::cos(t[0]), sn = std::sin(t[0]), dx = double(h2.x) - hip.x, dz = double(h2.z) - hip.z;   // hip2 about the vertical hip1 axis (Godot's rotation about +Y)
    const Vec3f h2n(float(hip.x + c * dx + sn * dz), h2.y, float(hip.z - sn * dx + c * dz));
    return ext_lmax_[std::size_t(leg)] - double(len(sub(toe_of(leg, t), h2n)));
}

// the room: how far the pose is from each limit (an instrument, Phase 2)
LegSolver::Room LegSolver::room(int leg, const std::array<double, 3>& t) const {
    Room r; const auto u = u_of_angles(leg, t);
    r.ext_m = ext_slack_fk(leg, t);
    r.knee_straight = u[2];            // 0 = straight, negative = past straight (hyperextended toward its stop at -1)
    r.knee_fold = 1.0 - u[2];          // to the fold stop
    r.hip1 = 1.0 - std::fabs(u[0]); r.hip2 = 1.0 - std::fabs(u[1]);
    return r;
}

} // namespace ogma::body
