#pragma once
// McuReset — the Robot HAT V4's MCU reset line (MCURST = GPIO5), via the kernel GPIO
// character-device uAPI v2 (no libgpiod dependency).
//
// Bench finding 2026-08-29: the HAT's MCU IGNORES out-of-window pulse values (0, 1,
// ARR) and keeps pulsing the last valid one — there is no per-channel release, and
// SunFounder's robots never go limp.  Resetting the MCU stops every PWM output until
// the timers are re-programmed: that is the only "limp" this HAT has, and it is also
// SunFounder's own recovery for an MCU stuck mid-I2C ("Remote I/O error").
//
// The line is requested once and HELD HIGH for the object's lifetime.  Releasing a
// line right after driving it low left it low on the RP1 (MCU held in reset, bus
// dead) — never release after a low.
#include <string>

namespace ogma::hw {

class McuReset {
public:
    explicit McuReset(const std::string& chip = "/dev/gpiochip0", unsigned line = 5);
    ~McuReset();
    // Low 10 ms, high, then settle: the MCU is back on I2C after ~50 ms; its ADC
    // returns garbage for a little longer (an 11.2 V "battery" was read once).
    void reset(int settle_ms = 100);
    bool ok() const { return fd_ >= 0; }
private:
    void set(int value);
    int fd_ = -1;
};

} // namespace ogma::hw
