#pragma once

// =============================================================================
// MotionField.hpp  --  the always-on motion loop's sensor: returns where the world was just seen to be empty
// =============================================================================
//
// The ten-minutes phase, the chase push (2026-10-02).  The operator: "we need some type of motion-sensitive peripheral
// vision that is always active, as an ongoing loop separate from the others; it should be looking for voxels that are
// moving relative to the world frame at all times; moving voxels should always capture the robot's attention."
//
// Before this module the duck's only motion detector was a side-effect of CloudMap: a cluster whose voxels are YOUNG
// against the open cloud's oldest.  That cloud is filed and reopened for reasons that have nothing to do with motion (every
// metre walked, every stop), so its notion of "new" is only as long as the cloud is old -- on the chase seed at 150 s the
// train, 0.6 m away and 11 deg off the head's axis, was rejected because a 1.7 s-old walking cloud made 6 % of its
// oldest age 0.06 s (design doc §17.100).
//
// MotionField keeps its OWN memory: the last memory_s of the sensor's rays, every one, in the odometry frame -- never filed,
// no stop or walk states.  A new off-floor return is MOTION EVIDENCE when, a moment ago, a ray passed within near_m of the
// same point and went on at least beyond_m past it: something now stands where the world was just seen to be empty.  A
// static surface cannot do that (a ray that reached past it went through it); a grazing ray past a thin leg can, so a
// BLOB needs min_points evidence points within cluster_m in one cast, and a TRACK -- blobs associated cast to cast within
// gate_m -- is published only after persist_casts consecutive casts.  The track carries its world velocity.  The test is
// per ray, not per voxel: the voxel form of the other half (the vacated trail, R87) failed at 4 cm because a surface fills
// a voxel partly.
//
// Input: the cast token CloudMap reads (reality.proprio.tof_points: [still, yaw, trunk_z, odom_x, odom_y, 64 x (x, y, z)
// levelled body frame with z above the floor, origin (x, y, z), 64 x the empty zones' ray ends]).  A cast is processed
// when its points change (the sensor updates at 12.5 Hz; the token repeats between).  Output (output_topic, every tick):
// ProprioToken [vx = +right, vy = +forward (unit bearing, body frame), proximity = 1 - range / proximity_range, salience,
// track casts, the cast's tick (the sighting's stamp, where the cloud's mover token carries it), world vx, world vy] for the
// most salient published track; proximity 0 = none.
// Passive by itself: nothing reads it until a consumer is wired.  Absent = byte-identical.
#include <array>
#include <deque>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class MotionField : public Module {
public:
    static constexpr int kZones = 64;

    struct Ray { double ox, oy, oz, ex, ey, ez; };          // odometry frame; the end is the return or the empty ray's end
    struct Cast { uint64_t tick; std::vector<Ray> rays; };
    struct Track {
        int id = 0;
        double x = 0.0, y = 0.0, vx = 0.0, vy = 0.0;          // odometry frame
        int casts = 0, points = 0, misses = 0;
        double x0 = 0.0, y0 = 0.0;                            // where it began (the travel test)
        uint64_t first = 0, last = 0;
        double salience = 0.0;
    };

    MotionField();
    ~MotionField() override;

    std::string_view       type_name()      const override;
    std::vector<TopicSpec> input_topics()   const override;
    std::vector<TopicSpec> output_topics()  const override;
    ParamSchema            params_schema()  const override;
    ParamMap               current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;
    nlohmann::json diag_lite()     const override;

    // read-backs (the host's --log-motion, the tests)
    int  evidence_points() const { return last_evidence_; }
    int  blobs()           const { return last_blobs_; }
    const std::vector<Track>& tracks() const { return tracks_; }
    int  published() const { return published_; }               // index into tracks(), -1 = none
    bool cast_now()  const { return cast_now_; }                 // a new cast was processed this tick
    std::vector<std::array<double, 3>> const& last_evidence() const { return evidence_xyz_; }
    // for each evidence point, the remembered ray that vouched for it and its age in ticks (instrumentation)
    std::vector<std::array<double, 7>> const& last_justification() const { return why_; }

private:
    Bus* bus_ = nullptr;
    std::string input_topic_ = "reality.proprio.tof_points";
    std::string output_topic_;
    // cloud_mover_topic (step 3): the cloud's own mover token; with no published track of its own, MotionField passes the
    // cloud's sighting through on output_topic, so a consumer (the seek loop's chase) gets the UNION of the two detectors on
    // one topic.  Empty = its own tracks only.
    std::string cloud_mover_topic_;
    int passed_through_ = 0;
    // THE VOUCH (2026-10-02, M7): with vouch_m > 0 the cloud's sighting is passed through only if this module's motion
    // EVIDENCE lies within vouch_m of it in the last vouch_s (a static thing newly in view reads young to the cloud but puts no
    // return where a ray just passed), or if it continues a vouched mover (within gate_m of the last one passed, inside
    // drop_s).  own_tracks false: the module's own tracks are not published (the cloud does the following).
    double vouch_m_ = 0.0, vouch_s_ = 0.6; bool own_tracks_ = true;
    std::deque<std::array<double, 4>> recent_ev_;      // x, y, z, tick
    bool vouched_ = false; double vx_last_ = 0.0, vy_last_ = 0.0; uint64_t v_tick_ = 0;
    int vouched_n_ = 0, refused_n_ = 0;
    double memory_s_ = 1.5, near_m_ = 0.025, beyond_m_ = 0.08, min_height_ = 0.03, max_range_ = 2.5;
    double cluster_m_ = 0.12, gate_m_ = 0.25, proximity_range_ = 2.5, drop_s_ = 0.5;
    int    min_points_ = 2, persist_casts_ = 2;
    // THE BACKGROUND (2026-10-02, measured: the free-ray test alone fires on silhouette EDGES -- a table's top edge, a wall's
    // top, a ball's crown -- where earlier rays skimmed past within near_m; 244 false tracks a minute on seed 5): a return
    // with a REMEMBERED return within bg_m of it, between bg_min_s and bg_s old, is the static world (an edge already hit),
    // not motion; a mover's own trail is younger than bg_min_s.  0 = off.
    double bg_m_ = 0.0, bg_min_s_ = 1.0, bg_s_ = 10.0;
    // a published track must have travelled this far (m) in the world since it began: edges do not travel.  0 = off.
    double min_travel_m_ = 0.0;
    struct BgPt { double x, y, z; uint64_t tick; };
    std::deque<BgPt> bg_;                                  // the remembered returns, oldest first

    std::deque<Cast> casts_;
    std::vector<float> prev_pts_;
    std::vector<Track> tracks_;
    int next_id_ = 1, published_ = -1, last_evidence_ = 0, last_blobs_ = 0;
    bool cast_now_ = false;
    double px_ = 0.0, py_ = 0.0, pyaw_ = 0.0;
    std::vector<std::array<double, 3>> evidence_xyz_;
    std::vector<std::array<double, 7>> why_;
    uint64_t casts_seen_ = 0, published_ticks_ = 0;

    void process_cast(const Eigen::VectorXf& v, uint64_t tick_id);
    void publish(uint64_t tick_id);
};

}  // namespace ogma
