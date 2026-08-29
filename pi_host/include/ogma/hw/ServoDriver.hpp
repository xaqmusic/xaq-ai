#pragma once
// ServoDriver — the safety envelope, below the brain, where it cannot be routed
// around (port doc Phase 2 / SPEC §4).  The brain and the bench both talk to
// servos ONLY through this.
//
//   clamp        per-channel [min_us, max_us]; calibration narrows these to the
//                measured linkage hard stops + margin (Phase 2)
//   slew         at most `slew_us_per_tick` change per tick(): legs are 42 % of
//                body mass and a 50 Hz step command slams the gear train
//   watchdog     no command() for `watchdog_ticks` -> limp_all().  pulse 0, so
//                pulses STOP; "pause" (holding the last pulse) is a different
//                wire action and is never the safe one (SPEC §4.1)
//   time-at-limit  per-channel seconds spent commanded AT a clamp bound — a
//                sustained stall against carpet is invisible without current
//                sensing and will cook a servo quietly
//
// Not a thread: the owner calls tick() at the servo frame rate.
#include "ogma/hw/RobotHat.hpp"
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
    void forget_timers() { timer_ready_.fill(false); }

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
    std::array<uint64_t, N> at_limit_ticks_{};
    uint64_t tick_count_ = 0;
    uint64_t last_cmd_tick_ = 0;
    bool any_armed_ = false;
    bool tripped_ = false;
};

} // namespace ogma::hw
