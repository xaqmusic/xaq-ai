// =============================================================================
// TouchMemory.hpp  --  the cloud of touches: where the toes have found something
// =============================================================================
//
// An INSTRUMENT (2026-11-01, operator: "use FK to build a cloud of touches ... multiple
// sensors increase confidence ... see the drift").  Every time a foot's contact flag rises,
// the toe's commanded-FK position is one claim about where a surface is.  Hobby servos
// report nothing back, so the claim carries the kinematic error the sag study measured
// (~32 mm standing, ~50 mm walking, ledger 2026-10-24) and the odometry's drift (2-3x
// over-read on slip).  One touch is a 3 cm guess; agreement across touches, feet and time
// is what makes a voxel believable.  So the memory is EVIDENTIAL: each voxel keeps its hit
// count, which feet touched it, the load it bore and when, and the drift shows itself as
// new touches landing beside old ones on the same surface.
//
// INPUTS, all egocentric and the same on both bodies:
//   contact_topic  [4]  per leg 0/1 (the sim's physics flag, the robot's FSR >= 200 counts)
//   feet_xz_topic  [8]  per leg [x, z] of the toe in the chassis rest frame at the COMMANDED
//                       pose: x = body +X (anatomically left), z = forward
//   feet_y_topic   [4]  per leg toe height along fused gravity, body origin 0, planted ~ -0.09 m
//   odom_topic     [3]  [x, y, yaw] in DeadReckon's frame (x_o = forward at reset, y_o = body +X)
//   load_topic     [4]  optional, per leg load as a fraction of body weight
//
// HEIGHT is relative to the SUPPORT PLANE, not the body: the running median height of the
// planted toes.  A floor touch is h ~ 0 whatever the body's stance height; a toe landing on
// a box reads h > 0; nothing here needs the belly ToF.  The plane is updated from the feet
// that were already planted, never from the foot that just landed.
//
// STORAGE is in the odom frame (a voxel grid, voxel_m), so that a body that walks on leaves
// its footfalls behind and a consumer re-expressing them in the body frame sees them slide
// as the odometry drifts.  `odom_translation` is ProxMemory's: 1 = carried by the full
// odometry; 0 = rotation only.  diag_snapshot() re-expresses the nearest `diag_max` voxels in
// the current body frame: [fwd, lat, h, hits, foot_mask, age_ticks, load_max] plus the plane
// height, so a dashboard can place them against the body it draws.
//
// Publishes NOTHING on the bus.  Absent from a config = byte-identical.  Owns no partition
// (CLAUDE.md §0 rule 1): a voxel grid is a frozen geometric reduction, not a vocabulary.
#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class TouchMemory : public Module {
public:
    struct Voxel {
        double sx = 0.0, sy = 0.0, sh = 0.0;   // sums of the touch points (odom frame x, y; height over the plane)
        int    hits = 0;
        uint64_t first_t = 0, last_t = 0;
        int    foot_mask = 0;                 // bit i = leg i touched it
        float  load_max = 0.0f;
    };
    struct BodyVoxel { double fwd, lat, h; int hits; int foot_mask; uint64_t age; float load_max; };

    TouchMemory();
    ~TouchMemory() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json snapshot_state() const override;
    nlohmann::json diag_snapshot() const override;
    void           restore_state(nlohmann::json const& s) override;

    // ---- accessors for tests and dashboards ----
    int      voxels()  const { return int(vox_.size()); }
    int64_t  touches() const { return touches_; }
    double   plane_y() const { return plane_y_; }
    bool     have_plane() const { return have_plane_; }
    // The voxels in the CURRENT body frame, nearest first, at most n (0 = all).
    std::vector<BodyVoxel> body_voxels(int n = 0) const;

private:
    struct Key { int64_t ix, iy, iz; bool operator==(Key const& o) const { return ix == o.ix && iy == o.iy && iz == o.iz; } };
    struct KeyHash { size_t operator()(Key const& k) const {
        return size_t(k.ix * 73856093LL) ^ size_t(k.iy * 19349663LL) ^ size_t(k.iz * 83492791LL); } };

    void handle_contact(MessagePtr p);
    void handle_feet_xz(MessagePtr p);
    void handle_feet_y(MessagePtr p);
    void handle_odom(MessagePtr p);
    void handle_load(MessagePtr p);
    void add_touch(int leg, double x, double y, double h, float load, uint64_t t);
    void evict_if_full();
    void odom_pose(double& px, double& py, double& c, double& s) const;

    std::string contact_topic_ = "reality.proprio.foot_contact";
    std::string feet_xz_topic_ = "reality.proprio.feet_xz";
    std::string feet_y_topic_  = "reality.proprio.feet_y_gravity_cmd_imu";
    std::string odom_topic_    = "reality.proprio.odom";
    std::string load_topic_;                 // "" = none
    double voxel_m_          = 0.02;
    double odom_translation_ = 1.0;
    int    max_voxels_       = 4000;
    int    diag_max_         = 400;
    double plane_tau_ticks_  = 50.0;        // the support plane's EMA time constant

    // latest inputs
    float contact_[4] = {0, 0, 0, 0}, prev_contact_[4] = {0, 0, 0, 0};
    float feet_xz_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float feet_y_[4]  = {0, 0, 0, 0};
    float load_[4]    = {0, 0, 0, 0};
    bool  have_contact_ = false, have_xz_ = false, have_y_ = false, have_odom_ = false, have_load_ = false;
    double ox_ = 0.0, oy_ = 0.0, oyaw_ = 0.0;

    // the support plane
    double plane_y_ = 0.0; bool have_plane_ = false;

    std::unordered_map<Key, Voxel, KeyHash> vox_;
    int64_t  touches_ = 0;
    uint64_t last_tick_ = 0;
    Bus* bus_ = nullptr;
    std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
