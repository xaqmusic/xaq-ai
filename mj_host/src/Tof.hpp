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
    // Beam directions in the site frame (forward = +x, left = +y, up = +z).
    const std::array<std::array<double, 3>, kZones>& beams() const { return beams_; }

private:
    std::array<std::array<double, 3>, kZones> beams_{};
    std::array<TofZone, kZones> zones_{};
    std::array<double, 3> origin_level_{};
};

}  // namespace mjhost
