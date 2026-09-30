#pragma once
// The duck's eyes: the VL53L8CX's 8x8 depth matrix on the head, simulated by casting
// the sensor's 64 beams from the ToF site with MuJoCo's ray query against WORLD geometry
// only (group 0) — the robot is invisible to its own sensor, as in reality — and
// classified by a port of Pollen's kinematics::tof::Reprojector: Empty (no return),
// TooClose (a return within 10 cm horizontally), Floor (a downward beam whose return
// reaches 85 % of the way to the ground), Hit (everything else, with its horizontal
// range and its point in the trunk frame).  Head pose from forward kinematics, the
// level from gravity: egocentric throughout.
#include <array>

namespace mjhost {

class DuckBody;

struct TofZone {
    enum Class { Empty = 0, TooClose = 1, Floor = 2, Hit = 3 };
    Class cls = Empty;
    double range = -1.0;               // slant range of the return, m (-1 = none)
    double horizontal = 0.0;           // horizontal range (Hit)
    std::array<double, 3> point{};     // the return, in the trunk frame (Floor / Hit)
    // The same return in the GRAVITY-LEVELLED trunk frame: origin at the trunk, +z up by measured
    // gravity, +x/+y turning with the body.  This is the frame a point CLOUD wants -- the head may
    // sweep and the trunk may tilt a few degrees between casts and the points still compose, which
    // the raw trunk frame does not (2 m x sin 3 deg = 10 cm of apparent height, against a 4 cm block).
    std::array<double, 3> point_level{};
};

class Tof {
public:
    static constexpr int kRows = 8, kCols = 8, kZones = 64;
    static constexpr double kFovDeg = 45.0, kMaxRangeM = 4.0;
    static constexpr double kFloorSafety = 0.85, kMinRangeM = 0.10;

    Tof();
    // Cast and classify.  trunk_height_m = the trunk's height above the ground as the
    // robot itself estimates it (contact odometry's z).
    void sense(const DuckBody& body, double trunk_height_m);

    const std::array<TofZone, kZones>& zones() const { return zones_; }
    // The sensor's own position in the GRAVITY-LEVELLED trunk frame at the last cast (the origin every return's
    // ray starts from): what a cloud needs to know which voxels a ray passed THROUGH (CloudMap's vacated voxels).
    const std::array<double, 3>& origin_level() const { return origin_level_; }
    // The nearest Hit per column (m; kMaxRangeM if none), left to right as seen.
    std::array<double, kCols> column_hit() const;
    int too_close() const;
    // A four-slot summary in unit form for a brain: proximity ahead-left, ahead, ahead-right
    // (1 − range / 1 m over columns 0-2, 3-4, 5-7; 0 = nothing within a metre) and the
    // TooClose fraction.
    std::array<float, 4> summary() const;
    // THE REAL SENSOR'S TIMING (2026-10-01, --tof-real SPREAD LAG; 0 0 = off, byte-identical).  The simulated cast
    // reads the head pose at the instant of the cast; the VL53L8CX builds an 8x8 frame from FOUR integrations in
    // sequence (datasheet DS14161; 5 ms each by default, the VCSEL on for the whole period in continuous mode at up to
    // 15 Hz), and the robot composes the frame with the head pose it reads when the frame arrives.  On: sub-frame k
    // (zones by the 2x2 interleave, k = (row % 2) * 2 + col % 2 -- an assumption about the SPAD groups) is cast from the
    // sensor's pose LAG + SPREAD * (3 - k) / 4 seconds ago (interpolated between recorded ticks), and every return is
    // reprojected with the CURRENT pose.  A still head loses nothing; a moving head misplaces its points by the angle
    // it turned in between -- the cost of head motion the instantaneous cast gives away.  record() every tick.
    void set_realism(double spread_s, double lag_s) { spread_s_ = spread_s; lag_s_ = lag_s; }
    bool realism() const { return spread_s_ > 0.0 || lag_s_ > 0.0; }
    void record(const DuckBody& body);
    // The instrument: the last cast's mean and max distance (m) between each Hit/Floor return as composed (the current
    // pose) and where it truly was (its sub-frame's pose) -- the registration error the timing costs.  0 when off.
    double reg_error_mean() const { return reg_mean_; }
    double reg_error_max() const { return reg_max_; }
    // Beam directions in the site frame (forward = +x, left = +y, up = +z).
    const std::array<std::array<double, 3>, kZones>& beams() const { return beams_; }

private:
    std::array<std::array<double, 3>, kZones> beams_{};
    std::array<TofZone, kZones> zones_{};
    std::array<double, 3> origin_level_{};
    double spread_s_ = 0.0, lag_s_ = 0.0, reg_mean_ = 0.0, reg_max_ = 0.0;
    static constexpr int kHist = 16;                   // 0.32 s of 50 Hz ticks
    std::array<std::array<double, 3>, kHist> hist_pos_{};
    std::array<std::array<double, 4>, kHist> hist_quat_{};
    int hist_n_ = 0, hist_head_ = -1;                  // hist_head_ = the newest entry
};

}  // namespace mjhost
