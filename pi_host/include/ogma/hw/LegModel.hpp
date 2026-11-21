// LegModel.hpp -- the body's leg model as the token the stepper reads (reality.proprio.leg_model).
//
// The sim publishes 116 floats every tick (picrawler_body.gd, OGMA_PICRAWLER_LEG_MODEL=1): per leg
// the six FK anchor points in the chassis rest frame, the two joint axes and the toe offset
// (27 floats), then the action-to-angle map (8 floats).  ogma::body::LegSolver::set_model reads
// that layout; PoseSequencer, StanceFeatures and SupportModel build their kinematics from it.
//
// On the robot every one of those numbers is already loaded: the anchors and toe offsets are the
// BodyCalib the sense channels use (feet_xz, toe_y), the map is the ActionMap the actuation uses.
// This packs them, so the stepper's FK and IK run on the same body the host's own FK does.
//
// Two things the 116-float layout cannot say, refused here rather than published wrong:
//   * a per-leg hip1 sign (the brain path's splay_out_sign): LegSolver's map is u * range + rest
//     for every leg, so any sign other than +1 would turn that leg's hip the wrong way;
//   * a narrow knee whose range differs from hip2's (LegSolver uses hip2_range when the widening
//     is off).
#pragma once

#include "ogma/hw/Actuation.hpp"
#include "ogma/hw/BrainInputs.hpp"

#include <array>
#include <cmath>
#include <string>

namespace ogma::hw::brain {

constexpr int kLegModelFloats = 4 * 27 + 8;   // == ogma::body::LegSolver::kModelFloats

struct LegModel {
    std::array<float, kLegModelFloats> v{};
    bool ok = false;
    std::string why;
};

inline LegModel leg_model(const BodyCalib& b, const ActionMap& m) {
    LegModel out;
    if (!b.ok) { out.why = "body calib not loaded: " + b.why; return out; }
    if (!m.ok) { out.why = "action map not loaded: " + m.why; return out; }
    for (int l = 0; l < 4; ++l)
        if (std::fabs(m.splay_out_sign[size_t(l)] - 1.0) > 1e-9) {
            out.why = "splay_out_sign is not +1 on every leg: the leg model has no per-leg hip1 sign";
            return out;
        }
    if (!m.knee_widening && std::fabs(m.knee_symmetric - m.hip2_range) > 1e-9) {
        out.why = "narrow knee range differs from hip2_range: the leg model cannot carry it";
        return out;
    }
    auto put = [&](int k, const ogma::body::Vec3f& p) {
        out.v[size_t(k)] = p.x; out.v[size_t(k + 1)] = p.y; out.v[size_t(k + 2)] = p.z;
    };
    for (int l = 0; l < 4; ++l) {
        const auto& a = b.anchors[size_t(l)];
        const int k = 27 * l;
        // points: rest_inv (a translation; BodyCalib refuses a rest basis that is not identity)
        put(k,      b.rest_inv(a.hip1_world));
        put(k + 3,  b.rest_inv(a.hip2_world));
        put(k + 6,  b.rest_inv(a.knee_world));
        put(k + 9,  b.rest_inv(a.coxa_rest_origin));
        put(k + 12, b.rest_inv(a.upper_rest_origin));
        put(k + 15, b.rest_inv(a.lower_rest_origin));
        // directions: the rest basis is identity, so they are published as loaded
        put(k + 18, a.hip2_axis);
        put(k + 21, a.knee_axis);
        put(k + 24, b.toe_off[size_t(l)]);
    }
    const int n = 108;
    out.v[size_t(n)]     = float(m.hip1_rest);
    out.v[size_t(n + 1)] = float(m.hip1_range);
    out.v[size_t(n + 2)] = float(m.hip2_rest);
    out.v[size_t(n + 3)] = float(m.hip2_range);
    out.v[size_t(n + 4)] = float(m.knee_rest);
    out.v[size_t(n + 5)] = float(m.knee_fold);
    out.v[size_t(n + 6)] = float(m.knee_hyperext);
    out.v[size_t(n + 7)] = m.knee_widening ? 1.0f : 0.0f;
    out.ok = true;
    return out;
}

}  // namespace ogma::hw::brain
