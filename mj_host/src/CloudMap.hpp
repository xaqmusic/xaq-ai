#pragma once
// =============================================================================
// CloudMap — the duck's own point cloud, accumulated while it stands and looks
// =============================================================================
//
// Measured motivation (design doc §17.28, `microduck_tof_studies.md`): a small object is
// SUB-PIXEL in one ToF cast and multi-point in a sweep.  A 4 cm block at a metre subtends
// 2.3° against 5.625° zone spacing, so one cast lands 0.058 points on it and a 60 s gaze
// babble lands 43.  An EPM over single frames therefore cannot hold a node that means
// "block" — the block is not in its input.  The cloud is the level at which it exists.
//
// Three things make the accumulation honest, all of them the duck's own:
//
//   1. GRAVITY LEVELLING.  Points arrive as TofZone::point_level — the return in the
//      trunk frame rotated so +z is up by measured gravity.
//   2. YAW DE-ROTATION.  The levelled trunk frame turns with the body, and the trunk's
//      heading drifts 9.7° over a 56 s stop — 17 cm of smear at a metre, against a 4 cm
//      block.  Each cast is turned back by (yaw − anchor_yaw) from the contact odometry,
//      which agrees with truth to 0.1° (§16.3).  Measured worth: +7 % distinct voxels and
//      +4 points of change detection.
//   3. POSITION IGNORED, ON PURPOSE.  Over the same stop the trunk holds its position to
//      0.9 cm, so translation is below the voxel size and is not corrected.  This is why
//      the cloud is a STOP's cloud and not a room's: on the walk the anchor is meaningless
//      and the caller closes it.
//
// What it is NOT: a clusterer.  The break profile below is per-sector arithmetic over the
// voxel set — a frozen geometric reduction in the sense of `CLAUDE.md` §0, the same kind of
// thing Tof::column_hit already is.  Any vocabulary over it stays the EPM's to earn (§0
// rule 1), and no consumer here decides that two returns are "the same object".
#include <array>
#include <cstdint>
#include <unordered_map>

#include "Tof.hpp"

namespace mjhost {

class CloudMap {
public:
    static constexpr int kSectors = 8;     // azimuth sectors across the swept cone, body frame
    static constexpr int kProfile = kSectors * 4 + 4;   // 8 × (range, height, extent, mass) + 4 globals

    struct Params {
        double voxel_m   = 0.04;   // one voxel ≈ one block
        double break_lo  = 0.02;   // a return this far above the floor is not the floor
        double break_hi  = 0.20;   // ...and this low is something standing ON the floor
        double half_fov  = 40.0;   // degrees either side that the gaze babble actually reaches
        double max_range = 2.5;    // beyond this the cloud is the room's walls, not its contents
        int    max_voxels = 400000;
    };

    CloudMap() = default;
    explicit CloudMap(const Params& p) : p_(p) {}

    // Start a fresh cloud anchored on this heading.  Called when the body comes to rest.
    void open(double anchor_yaw, uint64_t tick);
    void close();
    bool is_open() const { return open_; }
    uint64_t opened_at() const { return opened_tick_; }

    // Accumulate one cast.  yaw = the odometry's heading now; trunk_z = the trunk's height
    // above the floor as the robot itself estimates it.
    void add(const std::array<TofZone, Tof::kZones>& zones, double yaw, double trunk_z, uint64_t tick);

    int  voxels() const { return int(vox_.size()); }
    uint64_t points() const { return points_; }
    int  break_voxels() const { return break_vox_; }

    // THE CHANGE SIGNAL, as measured offline: of the voxels touched in the last `window`
    // ticks, the fraction first seen inside that window.  A mover sweeps into voxels the
    // sweep had never occupied; the gaze revisiting known geometry does not.  0 when the
    // window holds nothing.
    double new_fraction(uint64_t tick, int window_ticks) const;

    // The cloud's FLOOR-BREAK PROFILE: per azimuth sector, the nearest break's range, its
    // height, its vertical extent and its voxel mass; then four globals (break mass, the
    // azimuth span it covers, its mean height, and the cloud's own size).  All in [0,1].
    std::array<float, kProfile> break_profile() const;

private:
    struct Vox { uint32_t hits = 0; uint64_t first = 0; uint64_t last = 0; };
    static int64_t key(int x, int y, int z) {
        return (int64_t(x + 524288) << 42) | (int64_t(y + 524288) << 21) | int64_t(z + 524288);
    }

    Params   p_;
    bool     open_ = false;
    double   anchor_yaw_ = 0.0;
    uint64_t opened_tick_ = 0, points_ = 0;
    int      break_vox_ = 0;
    std::unordered_map<int64_t, Vox> vox_;
};

}  // namespace mjhost
