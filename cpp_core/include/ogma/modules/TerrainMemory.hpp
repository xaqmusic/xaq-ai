#pragma once
// TerrainMemory -- the terrain ahead and underfoot as one memory of heights, from the forward 8x8 field and the toes,
// and the foothold PREDICTION it affords (2026-11-07, the careful stepper's step 1).
//
// Every return of the field within max_range_m, and every touch of a toe, is a claim about the ground's height at a
// place: the field's at a couple of centimetres, a touch's at half a centimetre.  Places are voxels in the odom frame
// (voxel_m); each keeps a height estimate and its variance, updated like a one-dimensional Kalman filter whose variance
// is floored at a touch's (a voxel is never surer than a toe), its class from the field (floor / thing / drop), its
// counts and its age (max_age_ticks: the odometry's drift makes an old claim unlocatable).
//
// THE DATUM.  A height must mean the same thing an hour and a flight of stairs later, so it is not "over the floor the
// body sees now" (on a staircase that floor jumps a tread ahead of the body).  Heights are measured in the body's own
// gravity-levelled frame (up_body, the fused up vector; the field's points and the toes alike) and stored plus the
// body's HEIGHT ABOVE THE DATUM `b`, which the body tracks from its own touches: a toe meeting ground the toes already
// measured reads the datum (the ground did not move, the body did -- it climbed), a toe meeting ground only the field
// saw corrects the ground (the field's height is the less certain claim).  `b` is thus the body's climb since the
// memory began, from touches alone, and on the staircase it is checked against the tread level (log only).  The
// field's own floor fit (its bottom rows) serves the CLASS and a sanity gate, not the height.
//
// The prediction: for each leg, where it will land next is its running mean of landing positions in the body frame
// (the stride is periodic); the memory under that point, now, is what the toe will meet -- published every tick on
// predict.foothold as [h_pred, precision, has, fwd, lat] x 4.  At the toe's contact the ERROR h_touch - h_pred is
// published on predict.foothold_error ([e, valid] x 4), kept as a running RMS per leg, and the touch then corrects the
// voxel.  Nothing acts on either yet: the error's fall along a run is the instrument's own admission, before the gait
// takes per-leg swing targets from the prediction (step 2).  Module absent = byte-identical.
#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"
#include "ogma/body/Tof8Geometry.hpp"

namespace ogma {

class TerrainMemory : public Module {
public:
    TerrainMemory();
    ~TerrainMemory() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    struct Voxel { double h = 0.0, var = 1.0; int n_field = 0, n_touch = 0; uint64_t first_t = 0, last_t = 0; int cls = 0; };
    struct BodyVoxel { double fwd, lat, h, var; int n_field, n_touch, cls; uint64_t age; };
    struct Prediction { double h = 0.0, prec = 0.0, fwd = 0.0, lat = 0.0; bool has = false; };
    // ---- for tests ----
    std::vector<BodyVoxel> body_voxels(int n = 0) const;
    Prediction const& prediction(int leg) const { return pred_[size_t(leg)]; }
    double last_error(int leg) const { return err_last_[size_t(leg)]; }
    bool   last_error_valid(int leg) const { return err_valid_[size_t(leg)]; }
    double rms_error(int leg) const { return std::sqrt(err_ms_[size_t(leg)]); }
    double datum() const { return datum_; }
    int    datum_edge_skips() const { return datum_edge_skips_; }
    double registration_applied() const { return reg_applied_; }
    int    registrations() const { return reg_count_; }
    void   frame(double& x, double& y, double& yaw) const { x = ox_; y = oy_; yaw = oyaw_; }
    int    n_errors() const { return n_err_; }
    int    count() const { return int(vox_.size()); }
    bool   floor_ok() const { return plane_.ok; }

private:
    struct Key { int64_t ix, iy; bool operator==(Key const& o) const { return ix == o.ix && iy == o.iy; } };
    struct KeyHash { size_t operator()(Key const& k) const { return std::hash<int64_t>()(k.ix * 73856093LL ^ k.iy * 19349663LL); } };
    void handle(std::string const& which, MessagePtr p);
    void ingest_field(uint64_t tick_id);
    void ingest_touch(int leg, double fwd, double lat, double h, uint64_t tick_id);
    void note_load(int leg, uint64_t tick_id);   // the toe's load after its touch, for the datum's gate
    void datum_move(double innov);
    void observe(double X, double Y, double h, double sigma, int cls, bool touch, uint64_t tick_id);
    void odom_pose(double& px, double& py, double& c, double& s) const;
    Prediction predict_at(double fwd, double lat) const;
    void evict_if_full();

    // config
    std::string field_topic_ = "sense.tof8", contact_topic_ = "reality.proprio.foot_contact", feet_xz_topic_ = "reality.proprio.feet_xz",
                feet_y_topic_ = "reality.proprio.toe_y", odom_topic_ = "reality.proprio.odom", up_topic_ = "reality.proprio.up_body",
                load_topic_ = "reality.proprio.foot_load", stall_topic_ = "",
                predict_topic_ = "predict.foothold", error_topic_ = "predict.foothold_error";
    double pitch_deg_ = 30.0, sensor_y_m_ = 0.049, sensor_z_m_ = 0.049, sensor_x_m_ = 0.0;
    double max_range_m_ = 2.0, thing_min_m_ = 0.02, fit_slope_max_ = 0.55; int fit_rows_ = 3;
    double voxel_m_ = 0.03; int max_voxels_ = 12000, max_age_ticks_ = 1500;
    double sigma_field_m_ = 0.02, sigma_touch_m_ = 0.005;
    double predict_tau_touches_ = 20.0, predict_radius_m_ = 0.045, plane_tau_ticks_ = 50.0, err_alpha_ = 0.05;
    double datum_gain_ = 1.0;   // how much of a touched-ground innovation the datum takes
    double datum_min_load_ = 0.15, datum_step_max_m_ = 0.03;   // a datum move needs a BEARING toe, and is bounded per touch
    // REGISTRATION BY RE-OBSERVATION (2026-11-07): before a field frame is stored, it is matched against what the memory
    // already holds along the body's forward axis -- the shift that best explains the remembered heights is the
    // odometry's error since the ground was last seen (a held body's legs claim stride) -- and register_gain of it moves
    // the frame.  Only when a step is in view (the frame's heights spread) and enough of the frame lands on remembered
    // ground.  0 = off (byte-identical).
    double register_gain_ = 0.0, register_range_m_ = 0.15, register_min_spread_m_ = 0.015; int register_min_points_ = 16;
    // A datum move is allowed only on known ground that is locally FLAT: a voxel on a riser's edge is touched at two levels
    // and disagrees by the riser (2026-11-07 pm: the datum's jitter against the truth was 3-4 cm, most of the re-contact
    // error).  The field was tried as the datum's second source and cannot be one: the only ground it re-sees is ground it
    // stored itself at the current datum, and ground the toes measured falls behind the body before the field looks there.
    double datum_edge_max_m_ = 0.02;
    double reg_last_shift_ = 0.0, reg_applied_ = 0.0; int reg_count_ = 0, reg_last_matched_ = 0; double reg_last_gain_ratio_ = 1.0;
    double stall_gate_ = 1.0;   // the memory's frame advances by the odometry's increment x (1 - stall_gate x stall): a body that is
                                // held does not move, whatever its legs claim (the admitted stall sense, 2026-11-07)
    int    diag_max_ = 300, recent_max_ = 32;

    // inputs
    std::array<float, body::tof8::kZones> field_{}; bool have_field_ = false; uint64_t field_tick_ = 0, field_done_ = ~uint64_t(0);
    std::array<float, 4> contact_{}, prev_contact_{}, feet_y_{}, load_{}; std::array<float, 8> feet_xz_{}; bool have_load_ = false;
    bool have_contact_ = false, have_xz_ = false, have_y_ = false, have_odom_ = false;
    double ox_ = 0, oy_ = 0, oyaw_ = 0;          // the memory's own frame: the odometry's yaw, its translation gated by the stall
    double raw_ox_ = 0, raw_oy_ = 0; bool have_raw_odom_ = false; double stall_ = 0.0;
    std::array<double, 3> up_{0.0, 1.0, 0.0}; bool have_up_ = false;
    double datum_ = 0.0; int datum_updates_ = 0, datum_edge_skips_ = 0;   // b: the body's height above the datum
    std::deque<double> datum_innov_;               // the last datum_window bearing innovations: the datum moves toward their MEDIAN
    int datum_window_ = 5;                         // (a toe on a voxel straddling a riser's edge disagrees by the riser; one such touch must not move the body)
    // the floor
    body::tof8::Plane plane_; double planted_plane_y_ = 0.0; bool have_planted_plane_ = false;
    // the memory
    std::unordered_map<Key, Voxel, KeyHash> vox_;
    // the prediction and its error
    std::array<std::array<double, 2>, 4> land_mean_{}; std::array<int, 4> land_n_{};
    std::array<Prediction, 4> pred_{};
    std::array<double, 4> err_last_{}, err_ms_{}; std::array<bool, 4> err_valid_{}; int n_err_ = 0;
    std::deque<std::array<float, 9>> recent_;    // [tick, leg, h_pred, prec, h_touch, e, n_touch_before, n_field_before, landing offset (m)]
    std::array<int, 4> pending_datum_{};          // a touch on known ground waiting for its toe to bear (ticks left), per leg
    std::array<double, 4> pending_innov_{};
    int64_t frames_ = 0, touches_ = 0; uint64_t last_tick_ = 0;
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
