#pragma once
// Distress — the PERCH × STALL accumulator behind reality.proprio.distress, in a form the
// physical robot can compute.  Shared by the sim's honest path and ogma_host.
//
// The sim's original (picrawler_body.gd, "Panic pathway") multiplies two scores:
//   stall  = 1 − (net displacement over a 120-tick window) / max_disp
//   perch  = smoothed |tilt| mapped from [perch_lo, perch_hi] onto [0, 1]
// and accumulates their product slowly (rise × score, decay × (1 − score)), after a
// 600-tick warmup.  It reads WORLD XZ position and the EXACT basis, neither of which a
// robot has.  This class keeps the arithmetic and replaces only the two inputs:
//   * displacement: stride odometry (body-frame velocity, x = right, y = forward)
//     integrated along the dead-reckoned heading.  Odometry reads ~75 % of true
//     travel (StrideV note), so the same walk reads as slightly MORE stalled.
//   * tilt: acos(up.y) from the fused attitude estimate.
// Heading convention matches the sim's: forward = (sin h, cos h), right = (cos h, −sin h)
// in the XZ plane, so only the magnitude of the net displacement enters.
//
// ⚠ max_disp keeps the sim's quirk on purpose.  The sim computes it as
// ref_speed · window / physics_hz with physics_hz = 240, i.e. 0.08 · 120 / 240 = 0.04 m —
// a normaliser in brain ticks divided by the PHYSICS rate.  A port is the same numbers,
// so the default is 0.04, not the 0.192 m the comment's "2 s at 0.08 m/s" would give.
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>

namespace ogma::body {

struct DistressParams {
    int     window_ticks   = 120;
    double  max_disp_m     = 0.04;    // = 0.08 · 120 / 240 — the sim's normaliser, quirk kept
    int64_t warmup_ticks   = 600;
    double  tilt_ema_alpha = 0.02;
    double  perch_lo       = 0.15;    // rad
    double  perch_hi       = 0.30;    // rad
    double  rise           = 0.006;
    double  decay          = 0.004;
};

class DistressAccumulator {
public:
    explicit DistressAccumulator(DistressParams p = {}) : p_(p) {}

    // One brain tick.  v_right / v_fwd in m/s (body frame), heading in rad, tilt in rad
    // (>= 0), dt in s, tick = the caller's tick counter (the warmup is judged on it, as
    // the sim judges it on tick_counter).  Returns the published distress in [0, 1].
    double step(double v_right, double v_fwd, double heading, double tilt, double dt,
                int64_t tick) {
        const double sh = std::sin(heading), ch = std::cos(heading);
        px_ += (v_fwd * sh + v_right * ch) * dt;
        pz_ += (v_fwd * ch - v_right * sh) * dt;
        hist_.emplace_back(px_, pz_);
        if (int(hist_.size()) > p_.window_ticks) hist_.pop_front();
        if (int(hist_.size()) == p_.window_ticks) {
            const double dx = hist_.back().first  - hist_.front().first;
            const double dz = hist_.back().second - hist_.front().second;
            const double disp = std::sqrt(dx * dx + dz * dz);
            stuck_deficit_ = p_.max_disp_m > 0.0 ? clamp01(1.0 - disp / p_.max_disp_m) : 0.0;
        }
        tilt_ema_ = (1.0 - p_.tilt_ema_alpha) * tilt_ema_ + p_.tilt_ema_alpha * std::fabs(tilt);
        const double perch = clamp01((tilt_ema_ - p_.perch_lo) / (p_.perch_hi - p_.perch_lo));
        const double score = perch * stuck_deficit_;
        if (tick < p_.warmup_ticks) distress_ = 0.0;
        else distress_ = clamp01(distress_ + p_.rise * score - p_.decay * (1.0 - score));
        return distress_;
    }

    double value()         const { return distress_; }
    double stuck_deficit() const { return stuck_deficit_; }
    double tilt_ema()      const { return tilt_ema_; }
    void reset() { hist_.clear(); px_ = pz_ = 0.0; tilt_ema_ = stuck_deficit_ = distress_ = 0.0; }

private:
    static double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
    DistressParams p_;
    std::deque<std::pair<double, double>> hist_;
    double px_ = 0.0, pz_ = 0.0;
    double tilt_ema_ = 0.0, stuck_deficit_ = 0.0, distress_ = 0.0;
};

}  // namespace ogma::body
