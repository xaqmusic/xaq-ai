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
#include "ogma/modules/CloudMap.hpp"   // CloudMap::Thing, the things phase (T1)

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
    // One ToF cast for the CloudMap MODULE to accumulate: [still, yaw, trunk_z, odom_x, odom_y, 64 x
    // (x, y, z)] in the gravity-levelled body frame with z already height above the floor, NaN
    // marking a zone that returned nothing.  Published on reality.proprio.tof_points.  The host
    // no longer accumulates anything: a small object exists in the SWEPT cloud (§17.28) and the
    // sweep, its cache and its reduction all live in ogma::CloudMap, where the inspector and the
    // brain builder can see them.  Absent: no publication, so a graph without the module is
    // byte-identical.
    // ...then, appended (2026-09-27, the chase phase): the sensor's own origin [x, y, z] in the same frame, z above the
    // floor, so the cloud can tell which voxels a ray passed through (vacated voxels).  Old consumers read the first
    // 5 + 3 * 64 values as before.
    std::array<float, 5 + 3 * 64 + 3> tof_points{};
    bool tof_points_valid = false;
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
    int    thing_winner_ = -1;   // the thing EPM's token this tick (things phase T1)
    double thing_tle_ = 0.0;
    int    thing_nodes_ = 0;
    bool   thing_seen_ = false;
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
    int last_steer() const { return last_steer_; }   // 0 none, 1 play, 2 avoidance, 3 seek (this tick)
    // THE SEEK GATE (things phase T2, `--seek-gate`): while the seek loop holds the heading reference, the
    // ToF sense slot of the sector its target lies in reads 0, so the twist brain's proximity prior does
    // not push the body off the thing it is walking to.  Gated by the state it exploits (a held seek
    // target, its sector); off = byte-identical.
    void set_seek_gate(bool on) { seek_gate_ = on; }
    // THE HEADING REFLEX (2026-09-17, `--heading-reflex TAU DAMP GATE`).  The twist brain's yaw column
    // does not hold a heading (design doc §17.17: with a quiet reference the error sits at two radians and
    // the command saturates at the gait frequency), so every loop's bearing -- play's, seek's -- goes into a
    // channel that circles.  The picrawler's answer (`CLAUDE.md` §1): a proportional hold on the body's own
    // dead-reckoned yaw through the authoritative channel.  Here: while a loop holds the reference, the
    // yaw command is the one that closes the heading error in TAU seconds (in the walker's own units, so
    // no constant is tuned to the signal), damped by the sensed yaw rate; it is MIXED with the brain's own
    // yaw command by proximity -- nothing within a metre: the reflex owns yaw; a wall at hand: the brain's
    // avoidance owns it.  Gated by the state it exploits (a held reference, a clear field).  Off = byte-identical.
    void set_heading_reflex(double tau_s, double damp, double tof_gate) { hr_tau_ = tau_s; hr_damp_ = damp; hr_gate_ = tof_gate; }
    // A CONTINUOUS REFERENCE (2026-09-19, `--ref-unwrap`).  The reference is rebuilt from the winning loop's
    // bearing every tick as heading - atan2(cx, cy); a bearing that flickers across +-pi (the thing behind the
    // body) flips it by 2 pi, the error flips between +3.0 and -3.1, the yaw command flips sign every few ticks,
    // and the body jitters in place instead of turning round (measured on R67 seed 1 at 1030 s; under play the
    // same flip at each pass behind reverses the turn and keeps the orbit alive: the "reference that will not
    // stand still" of §17.26).  With this on, each new reference is taken modulo 2 pi nearest the previous one,
    // so the turn direction persists through the back; the sense slot and the reflex clamp instead of wrapping.
    void set_ref_unwrap(bool on) { ref_unwrap_ = on; }
    // THE FREE-SPACE GATE ON THE REFERENCE (2026-09-19, `--ref-free P`): a loop's bearing is held as the
    // reference only if the ToF sector it points into is freer than P (proximity 1 - range / 1 m); a bearing
    // into a wall releases the reference (the reference = the heading, R29's release form) so the twist brain's
    // avoidance acts unopposed and the reflex stands down.  Measured need: with the reflex the body follows
    // the loops (64 %), and on two seeds of six a reference held into a wall costs 45-58 contacts a minute.
    void set_ref_free(double p) { ref_free_ = p; }
    // THE ESCAPE (2026-09-19, `--stuck-escape SECS`): after a stuck stop the reference is HELD for a while at a
    // heading the host chose from the cloud's view (the freest sector), the loops' bearings ignored meanwhile,
    // so the reflex turns the body out of the surface before play or seek can aim it back in (O35: the bad
    // seeds are long bursts at a surface).  Steer code 4 while it holds.
    void set_ref_hold(double bearing_rel, int ticks) { ref_hold_ = heading_ - bearing_rel; ref_hold_left_ = ticks; }
    int  ref_hold_left() const { return ref_hold_left_; }
    int  ref_released() const { return ref_released_; }
    double heading_reflex_share() const { return hr_share_; }   // the reflex's share of the yaw command this tick
    // STUCK (things phase, 2026-09-17, `--stop-on-stuck K`): the body's own forward-model error as a DURATION.
    // A stall is a run of ticks on which the brain commands forward (> 0.75 of range) and the body's sensed
    // forward velocity stays under 0.25 of range.  Walking is full of short stalls (the gait: median 0.22 s,
    // p99 1.2 s, measured n = 6 on R64); a push against a leg or a wall is a stall of 2-8 s, and every stall
    // of 2 s or more in those runs was one.  So the signal is the stall's length against the body's own
    // running median stall length: stuck = longer than K medians.  No level is tuned; the scale is the body's.
    void set_stuck(double k) { stuck_k_ = k; }
    // --stuck-cmd F (2026-09-29): the forward command that counts as pushing, as a fraction of range (0.75 = the original;
    // in the eye's arm's long wall bursts the command sits at half range with the body not moving at all)
    void set_stuck_cmd(double f) { stuck_cmd_frac_ = f; }
    // PROGRESS, not speed (2026-09-19, `--stuck-progress`): a stall is then "commanded forward and making no
    // progress toward the reference" -- the body's velocity along the reference's direction, in the body
    // frame, under 0.25 of range -- so a body sliding along a surface at walking speed with its reference into
    // the surface is as stuck as one pushing.  The reference's direction in the body frame is (cos e, -sin e)
    // with e = heading - reference and +y left.
    void set_stuck_progress(bool on) { stuck_progress_ = on; }
    bool stuck_now() const { return stuck_now_; }       // this tick: a stall crossed K x the running median
    double stall_s() const { return stall_run_ / 50.0; }
    double stall_median_s() const { return stall_med_ / 50.0; }
    int  seek_gated() const { return seek_gated_; }   // ticks the gate zeroed a slot
    // The seek loop's token (reality.cognitive.seek_value / seek_range), if a graph has one.
    bool   seek_present() const { return seek_present_; }
    bool   seek_arrived() const { return seek_arrived_; }   // this tick: a held target's need went to 0 with the range under 0.3 m
    double seek_ego()     const { return seek_ego_; }       // the seek bearing last set, body frame (rad, + = right)
    double seek_value()   const { return seek_value_; }
    // THE CHASE (chase phase, stage 1): the seek loop's chase state, read from the module for the record
    double seek_target_x() const;   // the seek loop's held target, odometry frame (0 when none)
    double seek_target_y() const;
    bool   chase_present() const;
    bool   chase_active()  const;
    int    chase_n()       const;
    int    chases()        const;
    double chase_vx()      const;
    double chase_vy()      const;
    bool   mover_seen()    const;
    int    mover_cands()   const;
    double chase_gaze_ego() const;
    std::array<int, 4> chase_cand_fates() const;   // replaced, too fast, still, timed out
    int    walk_takes() const;
    int    chases_yielded() const;
    int    yield_drops() const;                   // sightings dropped at a yielded place
    int    static_yielded() const;                // static targets dropped at the foot of tall structure (static_yield_tall)
    int    static_yield_drops() const;
    int    progress_forgets() const;              // static targets forgotten because the walk did not close on them (progress_walk_m)                 // chases that yielded near tall structure (chase_yield_tall)                     // targets started from a walking sighting (walk_take_range)
    std::array<double, 3> chase_last_judgement() const;   // miss (m), implied speed (m/s), decision code    // the bearing the head should turn to (rad, + = right), NaN when nothing moving is in mind
    bool   chase_coasting() const;    // the target is the lost thing's prediction, coasting (chase_permanence_ticks)
    int    chases_reacquired() const;
    void   restore_brain_state(nlohmann::json const& s);
    bool   chase_lost_now() const;    // this tick a chase was dropped with the thing still moving...
    double chase_lost_ego() const;    // ...and where it was last predicted: bearing (rad, + = right) and range from the body
    double chase_lost_range() const;
    bool   chase_have_cand() const;
    double chase_cand_x()  const;
    double chase_cand_y()  const;
    // THE LIVE VIEW: the voxels the cloud touched this cast, whether the cloud last filed was a walking one, the
    // place the cache evicted this tick (-1 none)
    std::vector<int32_t> cloud_cast_voxels() const;
    bool   cloud_filed_walking() const;
    int    cloud_evicted() const;
    double seek_range()   const { return seek_range_; }
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
    // The CloudMap module, if the graph declares one.  The host reads these for the JSONL and for
    // the replay dump; it never writes the cloud.
    bool   cloud_present()   const;
    bool   cloud_open()      const;
    bool   cloud_just_closed() const;
    bool   cloud_walking()   const;      // the open cloud is a WALKING cloud (CloudMap.walk_cloud): not the map's view
    int    cloud_voxels()    const;
    int    cloud_break()     const;
    int    cloud_place()     const;
    double cloud_newfrac()   const;
    double cloud_revisit()   const;
    double cloud_revisit_dist() const;
    int    cloud_cached()    const;
    // [ix, iy, iz, hits, mean_height_mm] 5-tuples of the cloud last filed, and the voxel edge they scale by.
    std::vector<int32_t> cloud_filed_voxels() const;
    double cloud_voxel_m()   const;
    std::vector<float> cloud_profile() const;   // the break profile the module publishes, this tick
    std::vector<float> cloud_view() const;      // the cloud as a gaze-invariant view (--map-view cloud), this tick
    // THINGS (the things phase, T1): the module's clusters of the open cloud, which is attended, its body-frame
    // bearing, and the clusters of the cloud last filed.  Present only when the graph gives CloudMap a things
    // topic; the host logs them and reads nothing back into the body.
    bool cloud_things_on() const;
    std::vector<ogma::CloudMap::Thing> cloud_things() const;
    // MOVERS (stage 0's instrument): the clusters of the open cloud through a recency window of window_ticks
    std::vector<ogma::CloudMap::Thing> cloud_things_recent(uint64_t window_ticks) const;
    int  cloud_attended() const;
    std::array<float, 3> cloud_thing_bearing() const;
    std::vector<ogma::CloudMap::Thing> cloud_filed_things() const;
    // The thing EPM's token (reality.cognitive.thing), if a graph has one: winner, tle, node count; winner -1 = none.
    int    thing_winner() const { return thing_winner_; }
    double thing_tle()    const { return thing_tle_; }
    int    thing_nodes()  const { return thing_nodes_; }
    // the KIND (O62): a second, coarser thing vocabulary on reality.cognitive.thing_kind, when a graph has one
    bool   kind_seen()    const { return kind_seen_; }
    int    kind_winner()  const { return kind_winner_; }
    double kind_tle()     const { return kind_tle_; }
    int    kind_nodes()   const { return kind_nodes_; }
    bool   thing_seen()   const { return thing_seen_; }
    std::vector<std::string> diagnostics() const;
    std::vector<std::string> take_inspector_events();   // live changes a client made since the last call
    // SKILLS AT THE INTENT BOUNDARY: a module publishes `intent.skill` (ProprioToken [id, request]) with
    // request 1 on the tick it wants one; ids: 0 kick_left, 1 kick_right, 2 roulade.  -1 = none this tick.
    int skill_request() const { return skill_request_; }
    // the outcome loop's token this tick, [node, predicted, observed, surprise, samples]; empty when none was observed
    std::vector<float> outcome_now() const { return outcome_; }
    // the last attended thing's position from the outcome loop's need topic (reality.cognitive.outcome_need
    // [need, x, y], odometry frame), as a bearing in the body frame (+ = right, like seek_ego) and a range:
    // what the host aims the unwind and the look stop at (--skill-unwind-aim).  Absent topic = not present.
    bool   thing_pos_present() const { return thing_present_; }
    double thing_ego()   const { return thing_ego_; }
    double thing_range() const { return thing_rng_; }
    // the play loop's bearing this tick and its state (climbing / wandering / next node), for the record
    std::array<float, 2> play_bearing() const { return play_bearing_; }
    nlohmann::json play_state() const;
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
    bool seek_gate_ = false; int seek_gated_ = 0;
    bool ref_unwrap_ = false; double ref_free_ = 0.0; int ref_released_ = 0;
    double ref_hold_ = 0.0; int ref_hold_left_ = 0;
    double hr_tau_ = 0.0, hr_damp_ = 0.0, hr_gate_ = 1.0, hr_share_ = 0.0;
    double stuck_k_ = 0.0; double stuck_cmd_frac_ = 0.75; int stall_run_ = 0; double stall_med_ = 12.5; bool stuck_now_ = false, stuck_fired_ = false; bool stuck_progress_ = false;
    bool seek_present_ = false; double seek_value_ = 0.0, seek_range_ = 0.0;
    bool seek_arrived_ = false; double seek_value_prev_ = 0.0;
    int skill_request_ = -1; uint64_t skill_request_tick_ = 0;
    bool thing_present_ = false; double thing_ego_ = 0.0, thing_rng_ = 0.0;
    bool kind_seen_ = false; int kind_winner_ = -1; double kind_tle_ = 0.0; int kind_nodes_ = 0;
    std::vector<float> outcome_;
    std::array<float, 2> play_bearing_{0.0f, 0.0f};
    int  seek_steers_ = 0;
    double seek_ego_ = 0.0;                   // the seek bearing this tick, body frame (rad, + = right)
    bool no_backing_ = false; int backing_clamped_ = 0;
};

}  // namespace mjhost
