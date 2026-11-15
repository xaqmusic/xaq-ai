// The stepper's shared helpers: the terrain patch readers and the support triangle's margin.  See StepperCommon.hpp.
#include "ogma/modules/stepper/StepperCommon.hpp"
#include <algorithm>
#include <cmath>

namespace ogma::stepper {

// the relief of the remembered 3 x 3 cells around a body-frame point (max - min height); false with fewer than two cells known
bool StepperInputs::patch_edge(double fwd, double lat, double& relief) const {
    relief = 0.0;
    if (!have_patch_) return false;
    const int ci = int(std::floor(fwd / cell_)) + N_ / 2, cj = int(std::floor(lat / cell_)) + N_ / 2;
    double lo = 1e9, hi = -1e9; int n = 0;
    for (int di = -1; di <= 1; ++di) for (int dj = -1; dj <= 1; ++dj) {
        const int i = ci + di, j = cj + dj;
        if (i < 0 || j < 0 || i >= N_ || j >= N_) continue;
        const float hv = patch_[size_t(4 + i * N_ + j)]; const double w = patch_[size_t(4 + N_ * N_ + i * N_ + j)];
        if (hv <= -98.0f || w <= 0.0) continue;
        lo = std::min(lo, double(hv)); hi = std::max(hi, double(hv)); ++n;
    }
    if (n < 2) return false;
    relief = hi - lo; return true;
}
bool StepperInputs::patch_h(double fwd, double lat, double& h) const { double sg; return patch_h(fwd, lat, h, sg); }
bool StepperInputs::patch_h(double fwd, double lat, double& h, double& sigma) const {
    h = 0.0; sigma = 0.0;
    if (!have_patch_) return false;
    const int ci = int(std::floor(fwd / cell_)) + N_ / 2, cj = int(std::floor(lat / cell_)) + N_ / 2;
    double sw = 0.0, swh = 0.0;
    for (int di = -1; di <= 1; ++di) for (int dj = -1; dj <= 1; ++dj) {
        const int i = ci + di, j = cj + dj;
        if (i < 0 || j < 0 || i >= N_ || j >= N_) continue;
        const float hv = patch_[size_t(4 + i * N_ + j)]; const double w = patch_[size_t(4 + N_ * N_ + i * N_ + j)];
        if (hv <= -98.0f || w <= 0.0) continue;
        sw += w; swh += w * hv;
    }
    if (sw <= 0.0) return false;
    h = swh / sw; sigma = std::sqrt(1.0 / sw); return true;   // the precision-weighted mean's own uncertainty
}

// ---- the sequence ----
// the signed distance of (cx, cz) inside the triangle p (+ inside) and the inward unit normal of the edge nearest it
double tri_margin(const std::array<std::array<double, 2>, 3>& p, double cx, double cz, double& nx, double& nz) {
    // orient counter-clockwise in the (x, z) plane
    std::array<std::array<double, 2>, 3> q = p;
    const double area = (q[1][0] - q[0][0]) * (q[2][1] - q[0][1]) - (q[1][1] - q[0][1]) * (q[2][0] - q[0][0]);
    if (area < 0) std::swap(q[1], q[2]);
    double best = 1e9; nx = 0; nz = 0;
    for (int i = 0; i < 3; ++i) {
        const auto& a = q[size_t(i)]; const auto& b = q[size_t((i + 1) % 3)];
        const double ex = b[0] - a[0], ez = b[1] - a[1], L = std::hypot(ex, ez); if (L < 1e-9) continue;
        const double d = (ex * (cz - a[1]) - ez * (cx - a[0])) / L;   // + on the interior side of a CCW edge
        if (d < best) { best = d; nx = -ez / L; nz = ex / L; }         // the inward normal of that edge
    }
    return best;
}

} // namespace ogma::stepper
