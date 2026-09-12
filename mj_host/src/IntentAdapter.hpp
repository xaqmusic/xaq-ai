#pragma once
// The brain one level up (the intent boundary, phase 1e): an OgmaInstance whose
// "motors" are the three twist commands to Pollen's walking policy and whose
// "senses" are the body's own velocity — from the contact odometry and the gyro —
// normalised by the walker's trained command ranges.  The same module code as the
// joint-level brain; only the topics and the body differ.  Egocentric throughout:
// nothing here reads the simulator's world pose.
#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <mutex>

#include <nlohmann/json_fwd.hpp>

#include "InspectorSurface.hpp"

namespace ogma { class OgmaInstance; }

namespace mjhost {

// The walker's trained command ranges (microduck_rl velocity task): the unit the
// level-2 brain commands and senses in.
constexpr double kTwistRangeVx = 0.4, kTwistRangeVy = 0.3, kTwistRangeVyaw = 1.0;

// What the host measures for the place vector each tick: the dead-reckoned pose and the ToF in
// both reductions.  Which of them the brain is given is the adapter's decision (PlaceForm).
struct PlaceInputs {
    std::array<float, 4>  pose{};    // x/2, y/2, cos yaw, sin yaw
    float                 head_yaw = 0.0f;   // the head-yaw joint from HOME / its range (W3: a view is pose + gaze)
    std::array<float, 8>  cols{};    // the nearest Hit per column / 4 m (R24's reduction)
    std::array<float, 64> zones{};   // every zone's slant range / 4 m, Empty = 1 (the sensor as it is)
};

class IntentAdapter {
public:
    IntentAdapter(const std::string& graph_path, uint64_t seed);
    ~IntentAdapter();

    // One brain tick.  vel_body = the body's own velocity estimate (vx, vy, yaw rate),
    // gravity + gyro from the IMU.  Returns the commanded twist for the walker.
    // odom_yaw = the odometry's own heading (boot frame, radians).  Sense slot 10 carries the
    // unwrapped heading's DEVIATION from its own slow running average (τ ≈ 60 s), as a fraction
    // of π: a continuous, bounded, linear heading memory a prior can hold at zero ("keep the
    // heading I have been keeping").  Slot 11 the cosine of the raw yaw.
    // tof = the ToF summary (proximity ahead-left / ahead / ahead-right, TooClose fraction)
    // as slots 12-15; a level-2 bridge with load_slots 12 sees the first twelve only.
    std::array<double, 3> tick(const std::array<double, 3>& vel_body,
                               const std::array<double, 3>& gravity,
                               const std::array<double, 3>& gyro,
                               const std::array<double, 3>& accel,
                               double odom_yaw = 0.0,
                               const std::array<float, 4>& tof = {0.0f, 0.0f, 0.0f, 0.0f},
                               const PlaceInputs* place = nullptr);
    // The map: a slow EPM over the place vector (dead-reckoned x, y, heading, then the ToF)
    // publishes reality.proprio.place; its surprise is the novelty the brain senses (slot 11)
    // and, with a prior, seeks.  The vector's FORM follows the graph (2026-09-11, the
    // exploration line's control arm), read at construction and echoed at start:
    //   Columns  the map EPM on reality.proprio.place_in declares 12: pose + the 8 column ranges (R24 on)
    //   Zones    it declares 68: pose + the 64 zone ranges (R35 -- the RBF flattens it; kept for the record)
    //   Stacked  the graph has an EPM on reality.proprio.depth_in: the host publishes the 64 zone
    //            ranges with the frame's mean taken out; that EPM's latent (its previous tick) is
    //            appended to the pose -> place_in = [pose ; depth latent].  The map EPM must
    //            declare 4 + that EPM's projection_dim.  The plan's O10 form with the sensor swapped.
    enum class PlaceForm { Columns, Zones, Stacked, ColumnsGaze };
    PlaceForm   place_form() const { return place_form_; }
    int         place_dims() const { return place_dims_; }
    std::string place_form_desc() const;
    double map_tle() const { return map_tle_; }
    bool   map_novel() const { return map_novel_; }
    bool   map_baked_now() const { return map_baked_now_; }
    int    map_node_count() const { return map_node_count_; }     // from the token, every tick
    double map_quant_error() const { return map_qe_; }            // the view's distance to its winner
    double map_transition() const { return map_trans_; }          // the distance between consecutive winners (the TLE's second term)
    double map_expected_error() const { return map_expected_; }   // the channel's running expected TLE (Kalman-lessons Stage 2)
    int    map_baked_count() const { return map_baked_count_; }
    const std::vector<int>& map_pruned_ids() const { return map_pruned_ids_; }   // this tick's prunes
    // W3 (playroom plan §12.2, insert-on-stop): the map EPM's insertion, prototype adaptation and
    // stale pruning off while the body walks and on while it stands and looks; the token keeps
    // publishing (the play loop's node positions need a live winner).  Through hot-mutable params.
    void set_map_learning(bool on);
    int    map_winner() const { return map_winner_; }
    int    map_nodes() const;
    // Wander (phase 2b, R25): when the map has been unsurprised — its surprise below a
    // fraction of its own long average — for bored_s seconds, the heading the brain keeps
    // (the slow reference behind sense slot 10) jumps by turn_deg with a random sign, and
    // the heading prior turns the body.  Novelty holds the heading.  0 = off.
    void set_wander(double bored_s, double turn_deg, uint64_t seed);
    int wander_turns() const { return wander_turns_; }
    // R27 (2026-09-06, §17.3 fork (a)): a loop in the graph that publishes an egocentric bearing
    // on percept.play_bearing ([cx = +right, cy = +forward], the Cell's PlayLoop) sets the heading
    // reference behind sense slot 10 each tick -- novelty becomes a direction.  By absence: a
    // graph without such a loop is byte-identical (the reference stays the slow running average).
    double heading() const { return heading_; }          // the unwrapped own-yaw (rad) and the reference the
    double heading_ref() const { return heading_ref_; }  // twist brain is held to: the JSONL's hdg field
    int play_steers() const { return play_steers_; }
    int last_steer() const { return last_steer_; }   // 0 none, 1 play, 2 avoidance (this tick)
    // A constant command in place of the brain's (an open-loop baseline); NaN = off.
    void set_override(const std::array<double, 3>& twist) { override_ = twist; has_override_ = true; }
    // --no-backing (2026-09-10, operator's observation: the duck backs into a wall and stays):
    // the forward command is clamped at zero. The body has no rear sensor — the ToF looks
    // forward, on the robot as in the host — so a step backward is a step into the unseen.
    // Off by default: byte-identical without the flag.
    void set_no_backing(bool on) { no_backing_ = on; }
    int backing_clamped() const { return backing_clamped_; }

    void on_reset();
    void set_learning(bool on);
    std::array<double, 3> last_twist() const { return last_twist_; }
    std::array<float, 3> last_sensed() const { return last_sensed_; }
    nlohmann::json brain_state() const;
    // The twist brain's OWN forward-model surprise (diag_lite motor_tle).  The only body-error
    // channel live while the walker drives the legs -- the joint brain is not even ticked then
    // (main.cpp, the W-line) -- so a stumble can only show up here.  -1 = no such module.
    double motor_tle() const;
    std::vector<std::string> diagnostics() const;
    uint64_t ticks() const { return tick_id_; }

private:
    std::unique_ptr<ogma::OgmaInstance> instance_;
    std::recursive_mutex instance_mtx_;
    std::unique_ptr<InspectorSurface> inspector_;
    uint64_t tick_id_ = 0;
    std::array<double, 3> last_twist_{};
    std::array<float, 3> last_sensed_{};
    std::map<std::string, double> frozen_rates_;
    bool frozen_ = false;
    std::array<double, 3> override_{};
    bool has_override_ = false;
    double heading_ = 0.0, prev_yaw_ = 0.0;   // the odometry yaw, unwrapped: a continuous heading
    double heading_ref_ = 0.0;                // its slow running average — the heading "I have been keeping"
    double map_tle_ = 0.0; bool map_novel_ = false; int map_winner_ = -1;
    bool map_baked_now_ = false; bool map_frozen_ = false; std::string map_module_id_;
    int map_node_count_ = 0, map_baked_count_ = 0; std::vector<int> map_pruned_ids_;
    double map_qe_ = 0.0, map_expected_ = 0.0, map_trans_ = 0.0;
    std::map<std::string, double> map_saved_;          // the map's configured rates (schema defaults if absent; stale_prune as 0/1)
    double wander_bored_s_ = 0.0, wander_turn_deg_ = 90.0;
    double map_tle_long_ = 0.0; int bored_ticks_ = 0; int wander_turns_ = 0;
    PlaceForm   place_form_ = PlaceForm::Columns;
    int         place_dims_ = 12, depth_dims_ = 0;
    std::string depth_topic_;
    uint64_t wander_rng_ = 0x9E3779B97F4A7C15ull;
    bool have_yaw_ = false;
    int play_steers_ = 0;                     // ticks on which a loop's bearing set the heading reference
    int avoid_steers_ = 0;                    // of those, ticks the avoidance loop won
    int last_steer_ = 0;
    bool no_backing_ = false; int backing_clamped_ = 0;
};

}  // namespace mjhost
