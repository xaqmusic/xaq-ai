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
                               const std::array<float, 12>* place = nullptr);
    // The map: a slow EPM over the place vector (dead-reckoned x, y, heading, the eight
    // column ranges) publishes reality.proprio.place; its surprise is the novelty the
    // brain senses (slot 11) and, with a prior, seeks.
    double map_tle() const { return map_tle_; }
    bool   map_novel() const { return map_novel_; }
    int    map_winner() const { return map_winner_; }
    int    map_nodes() const;
    // Wander (phase 2b, R25): when the map has been unsurprised — its surprise below a
    // fraction of its own long average — for bored_s seconds, the heading the brain keeps
    // (the slow reference behind sense slot 10) jumps by turn_deg with a random sign, and
    // the heading prior turns the body.  Novelty holds the heading.  0 = off.
    void set_wander(double bored_s, double turn_deg, uint64_t seed);
    int wander_turns() const { return wander_turns_; }
    // A constant command in place of the brain's (an open-loop baseline); NaN = off.
    void set_override(const std::array<double, 3>& twist) { override_ = twist; has_override_ = true; }

    void on_reset();
    void set_learning(bool on);
    std::array<double, 3> last_twist() const { return last_twist_; }
    std::array<float, 3> last_sensed() const { return last_sensed_; }
    nlohmann::json brain_state() const;
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
    double wander_bored_s_ = 0.0, wander_turn_deg_ = 90.0;
    double map_tle_long_ = 0.0; int bored_ticks_ = 0; int wander_turns_ = 0;
    uint64_t wander_rng_ = 0x9E3779B97F4A7C15ull;
    bool have_yaw_ = false;
};

}  // namespace mjhost
