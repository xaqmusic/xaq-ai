#pragma once

// =============================================================================
// CloudMap.hpp  --  the sweep's point cloud, and a cache of them keyed by place
// =============================================================================
//
// Measured motivation (microduck design doc §17.28–17.29, `microduck_tof_studies.md`):
// a small object is SUB-PIXEL in one ToF cast and multi-point in a sweep.  A 4 cm block at
// a metre subtends 2.3° against 5.625° zone spacing, so one cast lands 0.058 points on it
// and a 60 s gaze babble lands 43.  An EPM over single frames therefore cannot hold a node
// that means "block" — the block is not in its input.  The cloud is the level at which a
// small object exists, and this module is that level.
//
// Measured again, once an EPM was put over it (§17.29): the cloud's reduction gives the
// steadiest vocabulary this body has had — 16 nodes, every one baked, changing seven times
// a minute against a place map's 60–130 — and it captures 98 % of the object information
// that survives knowing the body's pose, where the raw 8×8 frame captures 55 %.
//
// WHAT MAKES THE ACCUMULATION HONEST, all of it the body's own:
//
//   1. The points arrive GRAVITY-LEVELLED (the host rotates each return so +z is up by
//      measured gravity) with z already relative to the floor.
//   2. YAW DE-ROTATION.  The levelled body frame turns with the body, and a trunk's heading
//      drifts ~10° over a minute of standing — 17 cm of smear at a metre, against a 4 cm
//      block.  Each cast is turned back by R(+(yaw − anchor_yaw)): a body that yaws +d sees a
//      world-fixed point rotated by −d, so +d undoes it.  THE SIGN WAS WRONG until 2026-09-13 —
//      R(−d) doubled the smear, and its "+7 % distinct voxels" was read as a benefit when for a
//      static scene more distinct voxels is more smear.  Measured over 11 real stops: R(−d) was
//      sharpest on none and worse than no rotation (14 156 voxels vs 13 134); R(+d) sharpest in
//      total (12 795).  test_cloud_map's DerotationCancelsTheBodysOwnYaw now pins it.
//   3. TRANSLATION IS IGNORED, on purpose: a standing trunk holds position to ~1 cm, below
//      one voxel.  This is why a cloud belongs to a STOP and not to a room.
//   4. STILLNESS OPENS AND CLOSES IT, not a schedule.  The module watches the stillness flag
//      its input carries and arms after `still_ticks` of it, so the cloud survives the stop
//      becoming a decision rather than a timer (`CLAUDE.md` §5 rule 7).
//
// THE CACHE, keyed by PLACE.  When a cloud closes it is stored under the map EPM's modal
// winner while it was open — the place the body was at, in the map's own vocabulary, not a
// coordinate.  A revisit then has something to compare against, which is the capability the
// cache exists for: `revisit_change` is the fraction of the new cloud's voxels that the
// stored cloud of this same place had never occupied.  "This place is not as I left it" is a
// quantity no single cloud can express.  Least-recently-used eviction at `cache_size`.
//
// WHAT THIS IS NOT: a clusterer.  The break profile is per-sector arithmetic over the voxel
// set — a frozen geometric reduction in the sense of `CLAUDE.md` §0, the same kind of thing
// a nearest-hit-per-column already is.  Any vocabulary over it stays an EPM's to earn (§0
// rule 1), and nothing here decides that two returns are "the same object".
//
// Default-off: with no `input_topic` the module never accumulates and never publishes, so a
// graph that does not declare it is byte-identical.

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma {

class CloudMap : public Module {
public:
    static constexpr int kSectors = 8;                   // azimuth sectors across the swept cone
    static constexpr int kProfile = kSectors * 4 + 4;    // 8 × (range, height, extent, mass) + 4 globals
    static constexpr int kZones   = 64;                  // the sensor's returns per cast

    CloudMap() = default;
    ~CloudMap() override = default;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void                   on_param_change(std::string_view key, ParamValue const& value) override;

    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;

    nlohmann::json diag_snapshot() const override;
    nlohmann::json diag_lite()     const override;

    // ---- accessors (tests, telemetry, the viewer's dump) ----
    bool     is_open()       const { return open_; }
    bool     just_closed()   const { return just_closed_; }   // true on the tick a cloud was filed
    int      voxels()        const { return int(vox_.size()); }
    uint64_t points()        const { return points_; }
    int      break_voxels()  const { return break_vox_; }
    int      cached()        const { return int(cache_.size()); }
    int      last_key()      const { return last_key_; }      // the place the last filed cloud belongs to
    double   new_fraction()  const { return new_frac_; }       // change WITHIN this sweep
    double   revisit_change() const { return revisit_change_; } // change against this place's stored cloud; -1 = no match
    // How far apart the two visits' anchors were, by DEAD RECKONING, and how far the two clouds
    // had to be moved to line up.  Read revisit_change WITH this: the alignment is only as good as
    // the odometry, and the odometry drifts 4-6 % of distance travelled (§16.3), so two visits
    // minutes apart are aligned by a pose estimate that may be metres out.  Measured 2026-09-13 on
    // the one genuine revisit in a 1500 s run: aligned by the TRUE poses 0.639 of the cloud read as
    // new, by the odometry's 0.824, unaligned 0.864 -- the transform does real work and the pose it
    // is given is the weak link.  Registering the clouds by their own CONTENT is the open fix.
    double   revisit_anchor_dist() const { return revisit_dist_; }
    std::vector<float> profile() const;

    // The voxel set of the cloud last FILED, as flat [ix, iy, iz, hits, mean_height_mm] 5-tuples —
    // what a viewer draws.  Empty until a cloud closes.  Voxel indices, not metres: multiply by
    // voxel_m and add half a voxel for the centre; colour by the mean height, not the centre.
    const std::vector<int32_t>& last_filed_voxels() const { return filed_vox_; }
    double   last_anchor_yaw() const { return filed_anchor_yaw_; }

private:
    // zsum: the running sum of the points' own heights.  A voxel is classified by the MEAN height of
    // what landed in it, never by its centre: the ground layer spans 0-4 cm, its centre is exactly
    // break_lo, and a centre test put every floor voxel in the floor-break band (found 2026-09-13 in
    // the replay, where the whole floor drew in the break colour).
    struct Vox { uint32_t hits = 0; uint64_t first = 0; uint64_t last = 0; float zsum = 0.0f; };
    // A cached cloud carries the POSE it was anchored on, not just the heading: two visits to one
    // place stand up to a place-cell's width apart (~0.25 m = six voxels), so comparing them needs
    // the rigid transform between the two anchors, which the odometry gives for free.  Without it
    // the comparison is meaningless — measured 2026-09-13, before this existed: 86 % of a revisited
    // cloud read as new, which is what two frames that do not line up always say.
    struct Cached { std::unordered_map<int64_t, Vox> vox; double ax = 0.0, ay = 0.0, ayaw = 0.0; uint64_t filed = 0; };

    static int64_t key_of(int x, int y, int z) {
        return (int64_t(x + 524288) << 42) | (int64_t(y + 524288) << 21) | int64_t(z + 524288);
    }
    static void unkey(int64_t k, int& x, int& y, int& z) {
        z = int((k & ((int64_t(1) << 21) - 1)) - 524288);
        y = int(((k >> 21) & ((int64_t(1) << 21) - 1)) - 524288);
        x = int(((k >> 42) & ((int64_t(1) << 21) - 1)) - 524288);
    }
    void open_cloud(double anchor_yaw, double ax, double ay, uint64_t tick);
    void file_cloud(uint64_t tick);
    void add_cast(const Eigen::VectorXf& v, double yaw, double trunk_z, uint64_t tick);

    Bus* bus_ = nullptr;
    std::string input_topic_, place_topic_, output_topic_, change_topic_;
    double voxel_m_ = 0.04, break_lo_ = 0.02, break_hi_ = 0.20;
    double half_fov_ = 40.0, max_range_ = 2.5;
    int    max_voxels_ = 400000, still_ticks_ = 25, cache_size_ = 8, new_window_ = 50;
    int    move_ticks_ = 25;

    bool     open_ = false, just_closed_ = false;
    int      still_run_ = 0, move_run_ = 0;
    double   anchor_yaw_ = 0.0, anchor_x_ = 0.0, anchor_y_ = 0.0;
    uint64_t opened_tick_ = 0, points_ = 0;
    int      break_vox_ = 0;
    double   new_frac_ = 0.0, revisit_change_ = -1.0;
    std::unordered_map<int64_t, Vox> vox_;
    std::unordered_map<int, int>     winner_hist_;        // winner → ticks, for the modal key
    std::unordered_map<int, Cached>  cache_;
    std::deque<int>                  lru_;                // front = least recently filed
    int      last_key_ = -1, filed_count_ = 0;
    std::vector<int32_t> filed_vox_;
    double   filed_anchor_yaw_ = 0.0;
    int      revisit_overlap_ = 0;   // voxels the comparison actually had to work with
    double   revisit_dist_ = -1.0;   // dead-reckoned separation of the two anchors, metres
};

}  // namespace ogma
