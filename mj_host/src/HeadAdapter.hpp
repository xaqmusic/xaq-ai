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
    // gait = one periodic proprioceptive signal of the gait (the left hip pitch relative to HOME),
    // for the phase feed-forward below; ignored unless it is on.
    std::array<double, 4> tick(const std::array<double, 4>& head_q,
                               const std::array<double, 3>& head_g,
                               const std::array<double, 3>& head_w,
                               const std::array<double, 3>& g,
                               const std::array<double, 3>& w,
                               double gait = 0.0);

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
    // The gait-PHASE feed-forward on yaw (--head-phase LEAD_TICKS LEARN_S; 0 = off, byte-identical):
    // the head's yaw jitter is periodic with the gait (2.2 Hz, §17.13), so it can be countered a
    // quarter period AHEAD instead of a quarter period behind. Phase comes from the hip pitch's
    // own upward crossings of its running mean (the stride clock; the doctrine's "feed it
    // phase"); a 16-bin table over phase learns the head's yaw rate per bin for LEARN_S seconds
    // after the babble with the command at zero, is integrated into the periodic yaw ANGLE the
    // head will make, and is then frozen and commanded with the opposite sign LEAD_TICKS early —
    // identify, then act, as the rest of the head loop. The honest signal is the residual: the
    // measured rate minus the table's, printed at the end against the rate the table learned.
    void set_phase(double lead_ticks, double learn_s) { phase_lead_ = lead_ticks; phase_learn_s_ = learn_s; }
    std::vector<std::string> phase_report() const;
    std::array<double, 4> last_command() const { return last_cmd_; }
    // W2: the yaw command is the given target while on (the stance-gated saccade channel: at a stop,
    // the head's yaw is the gaze axis; on the walk it follows the trunk as before). The loop does not
    // own yaw (its model is pitch and roll), so nothing it learns pairs with this command.
    void set_yaw_override(bool on, double target) { yaw_override_ = on; yaw_target_ = target; }
    // W3b: the gaze babble also moves head_pitch (looking down puts the floor's objects in more rows);
    // the level prior's pitch command is replaced while on, its learning frozen through the stop.
    void set_pitch_override(bool on, double target) { pitch_override_ = on; pitch_target_ = target; }
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
    bool yaw_override_ = false; double yaw_target_ = 0.0;
    bool pitch_override_ = false; double pitch_target_ = 0.0;
    double vor_tau_ = 0.0, vor_lead_ = 0.0, vor_state_ = 0.0;
    double rate_k_ = 0.0, rate_tau_ = 0.0, rate_state_ = 0.0;
    // the phase feed-forward
    static constexpr int kPhaseBins = 16;
    double phase_lead_ = 0.0, phase_learn_s_ = 0.0;
    double gait_mean_ = 0.0; bool gait_mean_init_ = false; double gait_prev_ = 0.0;
    uint64_t last_cross_ = 0; double period_ticks_ = 0.0; int crossings_ = 0;
    std::array<double, kPhaseBins> rate_bin_{}; std::array<int, kPhaseBins> rate_n_{};
    std::array<double, kPhaseBins> angle_bin_{}; bool phase_frozen_ = false;
    double learn_ms_ = 0.0, act_ms_ = 0.0, act_res_ms_ = 0.0; uint64_t learn_n_ = 0, act_n_ = 0;
};

}  // namespace mjhost
