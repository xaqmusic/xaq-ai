#pragma once
// ServoDriver — the safety envelope, below the brain, where it cannot be routed
// around (port doc Phase 2 / SPEC §4).  The brain and the bench both talk to
// servos ONLY through this.
//
//   clamp        per-channel [min_us, max_us]; calibration narrows these to the
//                measured linkage hard stops + margin (Phase 2)
//   slew         at most `slew_us_per_tick` change per tick(): legs are 42 % of
//                body mass and a 50 Hz step command slams the gear train
//   watchdog     no command() for `watchdog_ticks` -> limp_all(), which writes pulse 0
//                and disarms.  ⚠ ON THIS HAT PULSE 0 IS A NO-OP: the V4 cannot
//                de-energise a servo from software at all (measured 2026-08-29 —
//                pulse 0/1/ARR ignored, stopped timer ignored, MCU held in reset
//                30 s and the servo still powered).  So this watchdog does NOT
//                make anything go slack; it stops the driver refreshing, and the
//                servo simply holds its last pulse.  THE ACTUAL SAFE ACTION LIVES
//                ONE LAYER UP: benchd's deadman (DEADMAN_MS = 1000) commands the
//                saved `rescue` POSE, and benchd::limp_all() is literally
//                `rescue(why)`.  A bench operator sees the robot move TO RESCUE,
//                not go limp — and if that move is small it can pass for nothing
//                happening, which is how it corrupts a measurement quietly.
//                wire action and is never the safe one (SPEC §4.1)
//   time-at-limit  per-channel seconds spent commanded AT a clamp bound — a
//                sustained stall against carpet is invisible without current
//                sensing and will cook a servo quietly
//
// Not a thread: the owner calls tick() at the servo frame rate.
#include "ogma/hw/RobotHat.hpp"
#include <algorithm>
#include <array>
#include <cstdint>

namespace ogma::hw {

struct ServoLimits {
    int min_us = 500;
    int max_us = 2500;
};

struct ServoDriverConfig {
    int slew_us_per_tick = 40;     // 2000 us/s: full travel in ~1 s
    int watchdog_ticks   = 25;     // 0.5 s at 50 Hz; 0 disables
    double tick_hz       = 50.0;
};

class ServoDriver {
public:
    static constexpr int N = RobotHat::N_SERVO;

    ServoDriver(RobotHat& hat, ServoDriverConfig cfg = {});

    void set_limits(int ch, ServoLimits lim);
    // Slew rate for every armed channel, changeable at runtime (pose moves run gentler).
    void set_slew_us_per_tick(int v) { cfg_.slew_us_per_tick = std::max(1, v); }
    int  slew_us_per_tick() const { return cfg_.slew_us_per_tick; }
    // True when every armed channel has reached its target.
    bool settled() const { for (int c = 0; c < N; ++c) if (armed_[c] && current_[c] != target_[c]) return false; return true; }
    ServoLimits limits(int ch) const { return lim_[ch]; }

    // Request a pulse width; it is clamped now and slewed by tick().
    // Also feeds the watchdog.
    void command(int ch, int us);
    // Advance one frame: slew every armed channel toward its target and write
    // it; run the watchdog; accumulate time-at-limit.
    void tick();
    // Immediate: pulse 0 on every channel, targets cleared, watchdog idle.
    // (On the Robot HAT V4 the MCU ignores pulse 0 — the owner must ALSO reset the MCU
    // and then call forget_timers(); see McuReset.)
    void limp_all();
    // After an MCU reset every timer is unprogrammed: the next command() re-programs it.
    // The reset also stops the PWM, so the last-sent pulses no longer describe the servos.
    void forget_timers() { timer_ready_.fill(false); known_.fill(0); }

    // ⚠ THE FIRST COMMAND ON A CHANNEL MUST SLEW FROM WHERE THE SERVO IS, NOT JUMP.
    // An unarmed channel has no slew history, and command() used to start it AT the target —
    // a step change the servo then crosses at its own full speed.  On the robot (2026-10-03)
    // every fresh benchd's first pose ran all 12 servos at full speed at once; the inrush
    // pulled the bench supply to 6.21 V and tripped the low-voltage rescue.  But the servo is
    // not "nowhere": the HAT keeps holding the last pulse written to it, by this process or a
    // previous one.  known_ is that pulse (0 = unknown).  It is updated on every write, kept
    // across limp_all() (this HAT ignores pulse 0, so the last pulse stays on the line), cleared
    // by forget_timers() (an MCU reset stops the PWM), and can be SEEDED by the owner from a
    // previous process's record.  With it, a first command ramps from there at the slew rate.
    void seed_known_pulse(int ch, int us) { if (ch >= 0 && ch < N && us > 0) known_[ch] = us; }
    int  last_sent_us(int ch) const { return known_[ch]; }

    bool   armed(int ch) const { return armed_[ch]; }
    int    target_us(int ch) const { return target_[ch]; }
    int    current_us(int ch) const { return current_[ch]; }
    double time_at_limit_s(int ch) const { return at_limit_ticks_[ch] / cfg_.tick_hz; }
    bool   watchdog_tripped() const { return tripped_; }
    uint64_t ticks() const { return tick_count_; }

private:
    RobotHat& hat_;
    ServoDriverConfig cfg_;
    std::array<ServoLimits, N> lim_{};
    std::array<int, N>  target_{};
    std::array<int, N>  current_{};
    std::array<bool, N> armed_{};
    std::array<bool, N> timer_ready_{};
    std::array<int, N>  known_{};            // last pulse written to the HAT, 0 = unknown
    std::array<uint64_t, N> at_limit_ticks_{};
    uint64_t tick_count_ = 0;
    uint64_t last_cmd_tick_ = 0;
    bool any_armed_ = false;
    bool tripped_ = false;
};

} // namespace ogma::hw
