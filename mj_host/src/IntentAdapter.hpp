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
    std::array<double, 3> tick(const std::array<double, 3>& vel_body,
                               const std::array<double, 3>& gravity,
                               const std::array<double, 3>& gyro,
                               const std::array<double, 3>& accel,
                               double odom_yaw = 0.0);
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
    bool have_yaw_ = false;
};

}  // namespace mjhost
