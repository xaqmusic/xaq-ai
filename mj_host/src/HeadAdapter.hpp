#pragma once
// The head loop (playroom plan, the H line; 2026-09-10): a brain whose "motors" are the
// four head commands in Pollen's walker command vector — neck_pitch, head_pitch, head_yaw,
// head_roll, deltas from HOME, the same four `robot.head` carries on the robot — and whose
// "senses" are the head IMU: gravity in the head frame and the head gyro, plus the trunk's.
// The same module code as the twist brain (IntentAdapter), with four command dimensions
// and the head's ranges.  It needs no joint access: the walker tracks the commands.
//
// The error it exists to reduce (H2): head-frame motion the brain did not command —
// gravity off vertical, angular rate not zero.  H1 is the identification babble that
// tells it which command moves the head which way.  Egocentric throughout.
#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace ogma { class OgmaInstance; }

namespace mjhost {

// The walker's trained head-command ranges (microduck_rl velocity task, final curriculum
// stage): the unit the head brain commands in.
constexpr std::array<double, 4> kHeadRange = {1.10, 1.10, 1.40, 0.31};

class HeadAdapter {
public:
    HeadAdapter(const std::string& graph_path, uint64_t seed);
    ~HeadAdapter();

    // One brain tick.  head_q = the four head joints' positions relative to HOME (what the
    // command asks for, sensed); head_g / head_w = the head IMU; g / w = the trunk IMU.
    // Returns the four head commands, radians from HOME.
    std::array<double, 4> tick(const std::array<double, 4>& head_q,
                               const std::array<double, 3>& head_g,
                               const std::array<double, 3>& head_w,
                               const std::array<double, 3>& g,
                               const std::array<double, 3>& w);

    void on_reset();
    void set_learning(bool on);
    // The vestibulo-ocular reflex on the yaw axis (--head-vor TAU LEAD; 0 = off, byte-identical).
    // The head yaw command is minus the trunk's yaw increment integrated from its gyro, leaking
    // back to centre with time constant tau (s): the gait's yaw wobble is cancelled, a slow turn
    // passes through. lead (s) adds minus the rate itself, a phase advance against the walker's
    // head lag. Feed-forward from the trunk gyro — predictive, not reactive — as the operator
    // asked (2026-09-10); yaw is the axis that carries most of the camera's motion. Its gain is
    // the joint ratio (a yaw command of theta turns the head theta against the trunk), so the
    // only free numbers are the two time constants.
    void set_vor(double tau_s, double lead_s) { vor_tau_ = tau_s; vor_lead_ = lead_s; }
    // The rate loop against the head's OWN gyro (--head-rate K TAU; 0 = off, byte-identical):
    // the yaw command integrates minus K times the head's measured yaw rate — whatever moves the
    // head, the trunk's turn or the walking policy's own jitter of the joint — leaking back to
    // centre in tau s (the position anchor a rate target alone lacked, Y1). The head IMU's x
    // axis points down, so its yaw rate is gyro x (measured 2026-09-10: +0.34 with the trunk's
    // yaw rate, the same sign). With an ideal actuator the head's excursion shrinks by 1/(1+K).
    void set_rate_loop(double k, double tau_s) { rate_k_ = k; rate_tau_ = tau_s; }
    std::array<double, 4> last_command() const { return last_cmd_; }
    nlohmann::json brain_state() const;
    // Restore a saved head brain (every module's working state) into this instance — the
    // H2 protocol: identify standing (H1, saved), act walking (loaded here, the prior on).
    void restore_brain_state(const nlohmann::json& s);
    std::vector<std::string> diagnostics() const;
    // The identified A's head-attitude rows against the four commands, for the H1 gate.
    std::vector<std::string> readback() const;
    uint64_t ticks() const { return tick_id_; }

private:
    std::unique_ptr<ogma::OgmaInstance> instance_;
    std::recursive_mutex instance_mtx_;
    uint64_t tick_id_ = 0;
    std::array<double, 4> last_cmd_{};
    std::map<std::string, double> frozen_rates_;
    bool frozen_ = false;
    uint64_t babble_ticks_ = 0;                        // from the graph: the yaw command is masked after it
    bool mask_yaw_ = true;                             // false when the graph owns action.head_yaw
    double vor_tau_ = 0.0, vor_lead_ = 0.0, vor_state_ = 0.0;
    double rate_k_ = 0.0, rate_tau_ = 0.0, rate_state_ = 0.0;
};

}  // namespace mjhost
