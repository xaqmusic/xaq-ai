#include "ogma/hw/ServoDriver.hpp"

#include <algorithm>
#include <stdexcept>

namespace ogma::hw {

ServoDriver::ServoDriver(RobotHat& hat, ServoDriverConfig cfg) : hat_(hat), cfg_(cfg) {}

void ServoDriver::set_limits(int ch, ServoLimits lim) {
    if (ch < 0 || ch >= N) throw std::out_of_range("ServoDriver channel");
    if (lim.min_us > lim.max_us) std::swap(lim.min_us, lim.max_us);
    lim_[ch] = lim;
}

void ServoDriver::command(int ch, int us) {
    if (ch < 0 || ch >= N) throw std::out_of_range("ServoDriver channel");
    const int clamped = std::clamp(us, lim_[ch].min_us, lim_[ch].max_us);
    if (!armed_[ch]) {
        // First command: no slew history — start from the clamped target so
        // the servo does not sweep in from an arbitrary current_ value.
        current_[ch] = clamped;
        if (!timer_ready_[ch]) {
            hat_.setup_servo_timer(ch);
            for (int c = (ch / 4) * 4; c < (ch / 4) * 4 + 4; ++c) timer_ready_[c] = true;
        }
    }
    target_[ch] = clamped;
    armed_[ch] = true;
    any_armed_ = true;
    tripped_ = false;
    last_cmd_tick_ = tick_count_;
}

void ServoDriver::tick() {
    ++tick_count_;
    if (!any_armed_) return;
    if (cfg_.watchdog_ticks > 0 && tick_count_ - last_cmd_tick_ > static_cast<uint64_t>(cfg_.watchdog_ticks)) {
        limp_all();
        tripped_ = true;
        return;
    }
    for (int ch = 0; ch < N; ++ch) {
        if (!armed_[ch]) continue;
        const int delta = target_[ch] - current_[ch];
        const int step  = std::clamp(delta, -cfg_.slew_us_per_tick, cfg_.slew_us_per_tick);
        current_[ch] += step;
        hat_.set_pulse_us(ch, current_[ch]);
        if (current_[ch] <= lim_[ch].min_us || current_[ch] >= lim_[ch].max_us) ++at_limit_ticks_[ch];
    }
}

void ServoDriver::limp_all() {
    hat_.limp_all();
    armed_.fill(false);
    any_armed_ = false;
}

} // namespace ogma::hw
