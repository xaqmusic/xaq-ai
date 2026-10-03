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

#include <array>
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
    static constexpr int kThing   = 8;                   // the attended thing's FULL descriptor (thing_descriptor)
    static constexpr int kThingShape = 5;                // ...and its SHAPE-ONLY form (things_shape true)

    // THINGS (the things phase, 2026-09-15, `microduck_things_phase.md` T1).  The stack rule of design doc
    // §17.31, run on the OPEN cloud: break-band voxels grouped into 8-connected columns; a cluster's stack top
    // is the contiguous chain of heights over its dilated footprint with a gap of max(gap_min, gap_k x range),
    // because the sensor's rows are 5.6 deg apart and the vertical spacing of returns grows with range.  A
    // cluster whose stack tops out under small_top and spans at most small_ext is SMALL: a thing the duck
    // could interact with (about five voxels high).  One that keeps rising is an obstacle.  Everything here
    // is in the cloud's own de-rotated frame; nothing decides that two returns are the same object.
    //
    // A thing does not grow on approach: a voxel is world-sized.  Its SAMPLING grows -- hits per column, and
    // whether top/ext/chain read the same from one sweep to the next -- so the descriptor carries density
    // and range, and the error an approach reduces is the descriptor's own instability.
    struct Thing {
        double cx = 0.0, cy = 0.0;     // footprint centroid, cloud frame (x forward, y left), metres
        double rng = 0.0;              // horizontal range of the centroid
        double ext = 0.0, ext_min = 0.0;   // footprint's larger and smaller span (+ one voxel)
        double lo = 0.0, top = 0.0;    // lowest break-band height, and the stack top the chain reaches
        int    ncols = 0;              // footprint columns
        double hits = 0.0;             // returns over the footprint (all heights)
        int    chain = 0;              // voxel levels from lo to top
        bool   small = false;          // the rule's verdict
        // MOVERS (cluster_recent): of the cluster's voxels inside the window, the share FIRST seen inside it, and
        // their mean age in ticks (the last tick seen minus the first).  A thing that moves keeps entering voxels
        // the cloud has never held, so its voxels stay fresh for as long as it moves; a static thing's voxels are
        // re-hit and age, however much the subset the window sees flickers.  0 / 0 from cluster_things().
        double fresh = 0.0, age = 0.0;
        double age_w = 0.0;            // the same age weighted by each voxel's hits: a re-hit voxel counts for its returns
        // VACATED (2026-09-27, T6's first half): off-floor voxels within vacate_radius of the centroid that a ray has
        // passed THROUGH within vacate_window_ticks after being occupied -- a thing that moved away leaves them; a
        // static thing newly in view, or a fragment sliding into view, leaves none.  0 with vacate_window_ticks 0.
        int    vacated = 0;
        // ISOLATION (2026-09-28, the operator: "predict a blob of voxels is part of a larger object by the proximity of
        // other voxels in its area, especially those higher than our small target objects; smaller objects will be
        // isolated into low blobs"): the voxels of the whole open cloud higher than iso_height within iso_radius of the
        // centroid.  A wall base has the wall above it, a chair leg its seat; a ball, a block or the train has none.
        int    tall_near = 0;
        // TOP SEEN (2026-10-02, the ten-minutes phase S1): the highest point above the floor at which a ray of this cloud
        // passed through the cluster's own columns without returning there (m; -1 = none, or free_rays off).  With the
        // head pitched down, the stack rule sees no top and calls a wall's foot or a chair leg small; a ball's top is
        // seen when a ray has passed over it.  small_needs_top requires seen_above >= top + one voxel.
        double seen_above = -1.0;
        // THE LINE (2026-10-02, the ten-minutes phase S1b): the share of the cluster's columns lying ON the closed tall
        // footprint -- the open cloud's columns with a voxel at or above small_top, closed (dilated then eroded) with a
        // radius of the ToF's zone spacing at their range, so a wall the sampling left dotted is a line again.  -1 =
        // not computed (line_tol_k 0, or the cluster was not small by shape).
        double on_line = -1.0;
        // S3's context: exp(-d / (line_close_k x range)), d the distance from the cluster's columns to the nearest tall
        // column -- 1 at the foot of something tall, ~0 in the open.  -1 = not computed.
        double near_tall = -1.0;
    };

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
    bool     is_walking_cloud() const { return open_ && walking_cloud_; }
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
    // The cloud as a gaze-invariant VIEW, for a place map (2026-09-13, the operator: "feed the map the cloud").
    // Per azimuth sector across +-view_half_fov of the cloud's own de-rotated frame (sector 0 on the right, as in
    // the profile), the horizontal range of the nearest voxel whose mean height clears the floor (break_lo),
    // divided by view_range; 1 where a sector holds none.  The reduction a nearest-hit-per-column is, taken over
    // the whole swept cloud instead of one frame: a sweeping gaze does not move it, and a cloud that has stopped
    // growing has stopped changing it.  All 1 while no cloud is open (filing clears the voxels).
    std::vector<float> view() const;

    // The clusters of the open cloud, as last computed (every things_every ticks while open; empty when closed),
    // and which one is attended: the nearest SMALL thing within things_range, or -1.
    const std::vector<Thing>& things() const { return things_; }
    int attended() const { return attended_; }
    // The world-sized descriptor a thing EPM earns its vocabulary over, in [0,1].  FULL (kThing):
    // [top / break_hi, ext / small_ext, ext_min / ext (aspect), ncols / 25, hits per column / 20,
    //  chain / 5, rng / max_range, lo / break_hi].  No bearing: a vocabulary of things, not of poses.
    // SHAPE-ONLY (kThingShape, things_shape true): [top / break_hi, ext / small_ext, aspect, lo / break_hi,
    //  chain / 5] -- the sampling dims (columns, hits, range) left out.  Measured on R57 (n = 6, 4 903 attended
    //  ticks): with them in, the EPM's 35 nodes followed the sweep's fill-in state (majority-label purity 0.56,
    //  each object under 4+ winners with a modal share of 0.10-0.17); shape-only bins of the same ticks reach
    //  0.76.  Sampling belongs to the attention gate and the pull, not to the vocabulary (`CLAUDE.md` §0 rule 2).
    std::vector<float> thing_descriptor(const Thing& t) const;
    int thing_dims() const { return (things_shape_ ? kThingShape : kThing) + (things_age_dim_ ? 1 : 0); }
    // The attended thing's bearing in the BODY frame this tick, [vx = +right, vy = +forward, proximity]:
    // the shape VisualBearing emits, so VisualHomingNav consumes it unchanged.  All 0 when nothing is attended.
    std::array<float, 3> thing_bearing() const { return bearing_; }
    // MOVERS (the chase phase, stage 1, 2026-09-27): the cluster of the open cloud, through the recency window, whose
    // voxels are YOUNG against the cloud's own -- measured at stage 0 (design doc §17.53): a thing that moves keeps
    // entering voxels the cloud has never held, so its voxels are as old as one voxel crossing takes (0.26 s for a
    // train at 0.2 m/s) while a static thing's are as old as the watching (7 s).  A candidate: hit-weighted mean voxel
    // age under mover_age_k x the OLDEST cluster's age in the window (the cloud's own proof of how long it has been
    // watching; a fresh cloud vouches for nothing and yields no candidate), within mover_range, any size.  The nearest
    // candidate is published on mover_topic as [vx=+right, vy=+forward, proximity, age_s, oldest_s]; zeros when none.
    // Nothing here decides that two sightings are one mover: that is the loop's (BearingSeekLoop's chase).
    bool mover_seen() const { return mover_ >= 0; }
    std::array<float, 3> mover_bearing() const { return mover_bearing_; }
    const std::vector<Thing>& mover_clusters() const { return recent_; }
    int mover_index() const { return mover_; }
    int mover_candidates() const { return mover_cands_; }
    std::vector<Thing> cluster_things() const;   // recompute from the current voxels (tests, the host's filing record)
    // MOVERS (chasing moving things, 2026-09-27, stage 0's instrument): the same stack rule over only the voxels
    // seen in the last `window_ticks` -- a cloud accumulates, so a thing that moves leaves a smear the whole-cloud
    // rule reads as one long obstacle; the recency window is what lets a cluster have a position at a TIME.
    // Every voxel already carries its last-seen tick; nothing else is added.  All clusters, small or not: a
    // mover is defined by its motion, not its size.
    std::vector<Thing> cluster_recent(uint64_t window_ticks) const {
        return cluster_things(last_tick_ > window_ticks ? last_tick_ - window_ticks : 0);
    }
    uint64_t last_tick() const { return last_tick_; }
    const std::vector<Thing>& last_filed_things() const { return filed_things_; }   // the clusters of the cloud last filed

    // The voxel set of the cloud last FILED, as flat [ix, iy, iz, hits, mean_height_mm] 5-tuples —
    // what a viewer draws.  Empty until a cloud closes.  Voxel indices, not metres: multiply by
    // voxel_m and add half a voxel for the centre; colour by the mean height, not the centre.
    const std::vector<int32_t>& last_filed_voxels() const { return filed_vox_; }
    double   last_anchor_yaw() const { return filed_anchor_yaw_; }
    // THE LIVE VIEW (2026-09-27, the operator: "make the voxel view more similar to what the robot is perceiving"):
    // the voxels this cast touched, as [ix, iy, iz, mean_height_mm] 4-tuples, so a viewer can grow the OPEN cloud
    // as the module does and age it as the module does (every voxel's last-seen tick); whether the cloud last filed
    // was a WALKING one (forgotten on filing, never cached) or a stop's (cached by place, remembered); and the place
    // the cache evicted this tick, or -1.  Instrumentation: nothing here changes the module.
    const std::vector<int32_t>& last_cast_voxels() const { return cast_vox_; }
    bool     last_filed_walking() const { return filed_walking_; }
    int      last_evicted()       const { return evicted_; }

private:
    // zsum: the running sum of the points' own heights.  A voxel is classified by the MEAN height of
    // what landed in it, never by its centre: the ground layer spans 0-4 cm, its centre is exactly
    // break_lo, and a centre test put every floor voxel in the floor-break band (found 2026-09-13 in
    // the replay, where the whole floor drew in the break colour).
    struct Vox { uint32_t hits = 0; uint64_t first = 0; uint64_t last = 0; float zsum = 0.0f; uint64_t vacated = 0; };
    struct Vacated { uint64_t tick; double x, y; };   // a recently vacated off-floor voxel's centre, cloud frame
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
    std::vector<Thing> cluster_things(uint64_t since_tick) const;   // ...over voxels last seen at or after since_tick
    void update_things(double yaw);
    void update_bearing(double yaw);
    void publish_bearing(uint64_t tick_id);
    void update_movers(double yaw, uint64_t tick_id);
    void publish_mover(uint64_t tick_id);
    std::array<float, 3> bearing_of(const Thing& t, double yaw) const;
    // A cluster's position relative to the BODY, in the body's frame, and its range from the body.  The cloud's
    // frame is the anchor's; a stop's body is the anchor (translation ignored, header point 3), but a WALKING body
    // has moved since -- up to walk_reset_m -- so the anchor-frame position must have the body's displacement
    // taken off before it is turned into the body's frame.  Until 2026-09-27 the bearing and the range were taken
    // from the ANCHOR on a walking cloud (R74's walk re-fix, O61; the chase's first arm, §17.54): every static
    // cluster then read as a thing moving at the body's own velocity.
    void body_rel(const Thing& t, double yaw, double& bx, double& by, double& rng) const;

    Bus* bus_ = nullptr;
    std::string input_topic_, place_topic_, output_topic_, change_topic_;
    std::string things_topic_, thing_bearing_topic_;
    double small_top_ = 0.16, small_ext_ = 0.20, small_ext_min_ = 0.0, gap_min_ = 0.10, gap_k_ = 0.12, things_range_ = 0.0;
    bool   things_shape_ = false;
    int    things_every_ = 4;
    bool   things_on_ = false;
    // THE WALKING CLOUD (2026-09-19, the operator: "are we using any ToF data while the robot is walking?").
    // With walk_cloud on, a cloud is also open while the body MOVES: each cast is translated by the odometry's
    // displacement from the anchor (the cast token carries x, y) and de-rotated as at a stop, and the cloud is
    // filed and reopened every walk_reset_m of travel so the odometry's drift (4-6 % of distance) stays under a
    // voxel.  A walking cloud is never cached as a place (it belongs to no stop); the things reduction runs on it,
    // so the seek loop gets LIVE bearings on the walk instead of a remembered position.  Off = byte-identical.
    bool   walk_cloud_ = false; double walk_reset_m_ = 1.0; bool walking_cloud_ = false;
    // walk_things (2026-09-27, O65): whether the THINGS reduction (the attended thing, its descriptor and bearing)
    // runs on a walking cloud.  R84 (walk_cloud on, things on the walk) turned seed 1 into a wall walk (wall
    // episodes 256 -> 944 against R83) though the seek loop took no more targets; false = things only at stops, as
    // in R83, while the mover candidate (mover_topic) still reads the walking cloud.  true = R84's behaviour.
    bool   walk_things_ = true;
    std::string mover_topic_;
    int    mover_window_ = 25; double mover_age_k_ = 0.06, mover_range_ = 1.5; bool mover_weighted_ = true;
    double mover_ext_max_ = 0.0;            // candidates no wider than this (m); 0 = any size
    // mover_range_hold (2026-09-29): a mover already being followed stays a candidate out to this range -- a young,
    // isolated cluster within mover_hold_gate of where the last published mover would now be (its last position plus
    // its displacement per update) is published beyond mover_range.  The start of a chase is gated close; its
    // continuation is not.  0 = off.
    double mover_range_hold_ = 0.0, mover_hold_gate_ = 0.4;
    // the chase push (2026-10-02): mover_hold_any_age -- a cluster within mover_hold_gate of the followed mover's predicted
    // position is the mover WHATEVER its age (starting a chase needs surprise, young voxels; continuing it rests on the
    // prediction: in a walking cloud re-filed every metre the oldest cluster is seconds old and the age gate rejects the
    // train itself).  mover_hold_ticks -- the hold survives this many ticks of recomputes without a published mover, its
    // point carried forward by the last displacement (a miss no longer ends the follow).  Both off = byte-identical.
    bool   mover_hold_any_age_ = false; int mover_hold_ticks_ = 0, hold_left_ = 0, hold_miss_ = 0;
    // mover_hold_min_v (the chase push, 2026-10-02): the any-age exemption holds only while the followed mover MOVES -- a
    // held step under this speed (m/s) for half a second returns it to the youth gate (T6's hold latched static clusters the
    // prediction swept over: 6 chases at structure in the first minute, §17.100).  0 = off.
    double mover_hold_min_v_ = 0.0; int hold_still_ = 0;
    // mover_not_target_m (the chase push, 2026-10-02): a NEW mover candidate within this of the seek loop's held target is
    // refused -- the thing being walked to reads young as it comes into view, and T6's first-minute chases at the green block
    // were exactly that (§17.103); the followed mover (the hold) is exempt.  Needs target_topic.  0 = off.
    double mover_not_target_m_ = 0.0;
    bool   target_in_cloud(double yaw, double& gx, double& gy) const;
    bool   mover_prev_ = false; double mover_px_ = 0.0, mover_py_ = 0.0, mover_dx_ = 0.0, mover_dy_ = 0.0;
    // THE TARGET'S SURROUNDINGS (2026-09-29, the pursuit that yields near tall structure): the seek loop's target
    // (its bearing and range, body frame) placed in the cloud's frame, and the open cloud's voxels at or above
    // iso_height within target_iso_radius of it counted; published on target_tall_topic as [count, range].  A chase
    // whose target stands at the foot of a wall is about to meet the wall the thing turned away from.  Empty = off.
    std::string target_topic_, target_range_topic_, target_tall_topic_;
    double target_iso_radius_ = 0.35;
    int    target_tall_ = 0, target_small_ = 0;
    void   publish_target_tall(double yaw, uint64_t tick_id);
public:
    int target_tall() const { return target_tall_; }
    // the small things within half target_iso_radius (at least 12 cm) of the seek target (2026-10-02, the impeded look:
    // a wall is tall structure at the target with NO small thing there; a ball by a wall has both).  Not published.
    int target_small() const { return target_small_; }
private:
    // things_skip_movers (2026-09-28): the things reduction does not ATTEND a cluster whose voxels are young by the
    // mover rule (a passing thing's smear at a stop reads as a small thing, and the seek loop then fixes a place
    // the thing has left -- the operator watched the duck peck at one).  A thing has a place while it is still.
    bool   things_skip_movers_ = false;
    // isolation (see Thing::tall_near): the mover candidate and / or the attended thing must be isolated from tall
    // structure; the descriptor may carry the voxel age (things_age_dim) so the kind vocabulary can EARN a moving kind
    double iso_height_ = 0.25, iso_radius_ = 0.25;
    bool   mover_isolated_ = false, things_isolated_ = false, things_age_dim_ = false;
    double things_oldest_ = 0.0;            // the oldest cluster's age at the last things update (the age dim's scale)
    std::vector<Thing> recent_;             // the window's clusters, as last computed
    int    mover_ = -1, mover_cands_ = 0;   // the attended mover (index into recent_) and candidates seen in all
    double mover_age_s_ = 0.0, mover_oldest_s_ = 0.0;
    uint64_t mover_tick_ = 0;               // the tick the candidate was last recomputed (the token's sixth value)
    std::array<float, 3> mover_bearing_{0.0f, 0.0f, 0.0f};
    std::vector<Thing>   things_, filed_things_;
    int                  attended_ = -1;
    std::array<float, 3> bearing_{0.0f, 0.0f, 0.0f};
    double voxel_m_ = 0.04, break_lo_ = 0.02, break_hi_ = 0.20;
    double half_fov_ = 40.0, max_range_ = 2.5;
    double view_half_fov_ = 64.0, view_range_ = 4.0;
    int    max_voxels_ = 400000, still_ticks_ = 25, cache_size_ = 8, new_window_ = 50;
    int    move_ticks_ = 25;

    bool     open_ = false, just_closed_ = false;
    int      still_run_ = 0, move_run_ = 0;
    double   anchor_yaw_ = 0.0, anchor_x_ = 0.0, anchor_y_ = 0.0;
    double   cur_x_ = 0.0, cur_y_ = 0.0;   // the body's odometry position this tick (the cast token's)
    double   cur_yaw_ = 0.0;               // ...and its yaw (the inspector's body marker; read by nothing else)
    int      vacate_window_ = 0;           // vacate_window_ticks: 0 = no ray traversal (byte-identical)
    double   vacate_radius_ = 0.25;
    double   vacate_beyond_ = 0.20;        // the ray must reach at least this far beyond the voxel it passes through
    int      mover_vacated_ = 0;           // the candidate needs at least this many vacated voxels (0 = not required)
    std::deque<Vacated> vacated_;          // the recently vacated voxels, oldest first
    // free_rays (S1): every ray of the cast, returning or not (the empty zones' rays from the cast's appended block),
    // walked from the origin; per column the highest free sample above break_lo.  small_needs_top gates `small` on it.
    bool     free_rays_ = false, small_needs_top_ = false;
    // line_tol_k (S1b): > 0 = a small cluster with half or more of its columns within max(1 voxel, line_tol_k x range) of the
    // closed tall footprint is a fragment of it, not a thing.  line_close_k: the closing radius per metre of range (the
    // ToF's zone spacing, 45 deg / 8 = 0.098 rad).  0 = off, byte-identical.
    double   line_tol_k_ = 0.0, line_close_k_ = 0.0982;
    // context_topic (S3): the attended thing's surroundings, ProprioToken [on_line, near_tall] in [0, 1], for a context
    // EPM whose winner the outcome loop keys its table by.  Empty = off.
    std::string context_topic_;
    std::unordered_map<int64_t, float> free_col_;   // keyed by (ix, iy, 0)
    uint64_t vacated_total_ = 0;
public:
    uint64_t vacated_total() const { return vacated_total_; }
private:
    uint64_t opened_tick_ = 0, points_ = 0, last_tick_ = 0;
    int      break_vox_ = 0;
    double   new_frac_ = 0.0, revisit_change_ = -1.0;
    std::unordered_map<int64_t, Vox> vox_;
    std::unordered_map<int, int>     winner_hist_;        // winner → ticks, for the modal key
    std::unordered_map<int, Cached>  cache_;
    std::deque<int>                  lru_;                // front = least recently filed
    int      last_key_ = -1, filed_count_ = 0;
    std::vector<int32_t> filed_vox_;
    double   filed_anchor_yaw_ = 0.0;
    std::vector<int32_t> cast_vox_;
    bool     filed_walking_ = false;
    int      evicted_ = -1;
    int      revisit_overlap_ = 0;   // voxels the comparison actually had to work with
    double   revisit_dist_ = -1.0;   // dead-reckoned separation of the two anchors, metres
};

}  // namespace ogma
