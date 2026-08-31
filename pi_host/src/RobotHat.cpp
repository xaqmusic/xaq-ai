#include "ogma/hw/RobotHat.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ogma::hw {

int RobotHat::prescaler_value() {
    // robot_hat: prescaler = round(CLOCK / FREQ / PERIOD); register gets prescaler - 1
    return static_cast<int>(std::lround(CLOCK_HZ / SERVO_HZ / SERVO_ARR)) - 1;
}

double RobotHat::actual_frame_hz() {
    return CLOCK_HZ / (prescaler_value() + 1) / SERVO_ARR;
}

int RobotHat::pulse_count(int us) {
    if (us <= 0) return 0;
    // robot_hat servo.py: int(pulse_width_time / 20000 * PERIOD) — truncation, kept exactly
    return static_cast<int>(static_cast<double>(us) / 20000.0 * SERVO_ARR);
}

void RobotHat::write_reg(uint8_t reg, int value) {
    if (value < 0 || value > 0xFFFF) throw std::out_of_range("RobotHat::write_reg value");
    bus_.write(ADDR, {reg, static_cast<uint8_t>((value >> 8) & 0xFF), static_cast<uint8_t>(value & 0xFF)});
}

void RobotHat::setup_servo_timer(int channel) {
    if (channel < 0 || channel >= N_SERVO) throw std::out_of_range("RobotHat channel");
    const uint8_t t = static_cast<uint8_t>(channel / 4);
    write_reg(REG_PSC + t, prescaler_value());
    write_reg(REG_ARR + t, SERVO_ARR);
}

void RobotHat::set_pulse_us(int channel, int us) {
    if (channel < 0 || channel >= N_SERVO) throw std::out_of_range("RobotHat channel");
    write_reg(REG_CHN + static_cast<uint8_t>(channel), pulse_count(us));
}

void RobotHat::set_pulse_raw(int channel, int count) {
    if (channel < 0 || channel >= N_SERVO) throw std::out_of_range("RobotHat channel");
    write_reg(REG_CHN + static_cast<uint8_t>(channel), std::max(0, std::min(count, 0xFFFF)));
}

void RobotHat::limp_all() {
    for (int c = 0; c < N_SERVO; ++c) limp(c);
}

int RobotHat::adc_raw(int channel) {
    if (channel < 0 || channel >= 8) throw std::out_of_range("RobotHat adc channel");
    const uint8_t sel = static_cast<uint8_t>((7 - channel) | 0x10);
    bus_.write(ADDR, {sel, 0, 0});
    const int msb = bus_.read_byte(ADDR);
    const int lsb = bus_.read_byte(ADDR);
    return (msb << 8) | lsb;
}

double RobotHat::adc_volts(int channel) {
    return adc_raw(channel) * ADC_VREF / ADC_MAX;
}

} // namespace ogma::hw
