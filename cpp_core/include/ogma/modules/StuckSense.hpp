// =============================================================================
// StuckSense.hpp  --  "I am commanding forward and the world is not getting closer"
// =============================================================================
//
// S3 lever 2 of the MicroDuck port.  Pressed against a wall this body keeps stepping, so its
// stride odometry keeps reading progress, its honest distress stays at zero and its slip
// channel does not move (measured, ledger 2026-10-11): the stuck state was a hidden state
// with no observation.  The observation that carries it is the forward range beam read as
// RE-AFFERENCE against the stride's EFFERENCE: over a window, the range to whatever is ahead
// should have closed by the distance the stride says the body walked.  When it has not, the
// body is pushing on something.
//
// TWO CHANNELS, the score is their max.
//
// 1. THE STALL (the duck's definition, body-scaled): the body commands forward and its own
//    stride says it is not going anywhere.  Pressed on a wall this body's stride forward
//    estimate OSCILLATES (−0.35 … +0.46 m/s, measured) around a mean of ~0.007 m/s, against
//    ~0.03 m/s walking free — so the per-tick stride is useless and its 1 s mean is the
//    signal.  v_ema = EMA(stride_v.y); v_ref = a slowly decaying peak-hold of v_ema (the
//    body's own recent best forward speed, so no constant names a speed);
//    stall = clamp(1 − v_ema / (stall_frac · v_ref), 0, 1) while commanding ≥ cmd_min.
//    This channel needs no beam, so a glancing wall is not hidden from it.
//
// 2. THE RANGE DEFICIT: over a window the range to what is ahead should have closed by what
//    the stride says the body walked; expected = ∫ stride_v.y dt, observed = r(t−W) − r(t),
//    deficit = clamp(1 − observed / expected, 0, 1) when expected ≥ min_expected_m and both
//    ends of the window had a valid echo.  It speaks only when the stride claims walking —
//    the slip case (feet sliding, stride over-reading) — and abstains otherwise.
//
// Everything here is egocentric; nothing is learned.  Module absent = byte-identical.
#pragma once
#include <deque>
#include <string>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class StuckSense : public Module {
public:
    StuckSense();
    ~StuckSense() override;
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

    float score() const { return score_; }
    float stall() const { return stall_; }
    float last_deficit() const { return deficit_; }
    bool  measured() const { return measured_; }

private:
    void handle_range(MessagePtr p);
    void handle_stride(MessagePtr p);
    void handle_speed(MessagePtr p);
    void handle_odom(MessagePtr p);

    std::string range_topic_  = "sense.range";
    std::string stride_topic_ = "reality.proprio.stride_v";
    std::string speed_topic_  = "reality.proprio.speed_cmd";   // the efference; empty = assume forward
    std::string output_topic_ = "reality.proprio.stuck";
    // Optional [x, y, yaw]: the body's own heading.  The sense abstains while the body IS
    // turning (|yaw rate| > yaw_rate_max): a turn zeroes forward stride legitimately.  Gating
    // on the COMMANDED bearing instead was measured wrong (S3b): a pinned body commanded to
    // turn away cannot, its bearing error stays large, and the sense abstained for good.
    // The turning test averages over WHOLE STRIDES: a quadruped yaws every stride by
    // construction (±0.2 rad/s rocking at the 70-tick gait period, measured), and a short EMA
    // reads the rocking as turning and abstains on a pinned body (S3b, escape-only arm v4: zero
    // escapes).  The net yaw over yaw_window_ticks is what a turn is.
    std::string odom_topic_   = "";
    double yaw_rate_max_      = 0.15;   // rad/s, net over the window
    int    yaw_window_ticks_  = 150;    // 3 s: two strides and change
    int    window_ticks_   = 50;      // 1 s at 50 Hz
    double min_expected_m_ = 0.05;    // below this the window says too little to judge
    double cmd_min_        = 0.3;     // the body must be commanding at least this forward
    double range_max_m_    = 1.4;     // a range beyond this is the beam's far end, not a thing
    double ema_alpha_      = 0.05;    // the range channel's EMA on measured ticks
    double decay_          = 0.01;    // per tick toward 0 while the range channel abstains
    double hz_             = 50.0;
    double v_ema_alpha_    = 0.01;    // the stride forward's EMA (~2 s)
    double v_ref_tau_s_    = 20.0;    // the peak-hold's decay
    double v_floor_        = 0.01;    // m/s; v_ref never falls below this (a body that never walked has no stall)
    double stall_frac_     = 0.25;    // v_ema below this fraction of v_ref is a full stall
    // 2026-10-18: the range channel's expectation from the COMMANDED speed instead of the
    // stride estimate.  expected = sum(speed_cmd * expect_from_cmd / hz) over the window, so the
    // channel speaks whenever the brain commands forward with a surface ahead -- at the fatal
    // stall on the 18th the stride read -0.3 m/s (carpet noise) and the channel stayed silent.
    // 0 = legacy (the stride's own claim), byte-identical.
    double expect_from_cmd_ = 0.0;    // m/s at full speed command

    struct Sample { double r; bool valid; double vfwd; double cmd; };
    std::deque<Sample> ring_;
    double d_ = 1.5; bool valid_ = false; double vfwd_ = 0.0; double speed_cmd_ = 1.0; bool have_speed_ = false;
    double yaw_ = 0.0, yaw_rate_ = 0.0; bool have_odom_ = false;
    std::deque<double> yaw_ring_;
    float score_ = 0.0f, deficit_ = 0.0f; bool measured_ = false;
    double v_ema_ = 0.0, v_ref_ = 0.0; float stall_ = 0.0f, range_score_ = 0.0f;
    double expected_ = 0.0, observed_ = 0.0;
    int64_t measurements_ = 0;
};

} // namespace ogma
