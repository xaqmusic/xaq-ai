#pragma once
// RobotHat — SunFounder Robot HAT V4 wire protocol, taken from their library
// (robot_hat 2.5.5: pwm.py / adc.py / servo.py / i2c.py), NOT the library.
// Rationale in the port doc (Phase 4): the picrawler library is scripted gaits;
// the protocol is the only part that transfers.
//
//   MCU I2C address 0x14.  Every register write is one 3-byte transaction
//   [reg, hi, lo]  (robot_hat sends it as an SMBus word whose low byte is hi).
//   PWM: 12 channels P0..P11 on 3 timers (timer = channel / 4).
//        REG_PSC + t  = prescaler - 1        REG_ARR + t = period (ARR)
//        REG_CHN + ch = on-count  in [0, ARR]
//        CLOCK 72 MHz.  Servo setup: ARR 4095, prescaler round(72e6/50/4095) = 352
//        -> written as 351 -> actual frame rate 72e6/352/4095 = 49.95 Hz.
//        pulse count = trunc(us / 20000 * 4095)   (0 = no pulses = servo LIMP)
//   ADC: 12-bit, 3.3 V ref.  Select channel by writing [ (7-ch)|0x10, 0, 0 ],
//        then two 1-byte reads: msb, lsb.  A4 = battery via 20K/10K divider (x3).
#include "ogma/hw/I2cBus.hpp"

namespace ogma::hw {

class RobotHat {
public:
    static constexpr uint8_t  ADDR      = 0x14;
    static constexpr uint8_t  REG_CHN   = 0x20;
    static constexpr uint8_t  REG_PSC   = 0x40;
    static constexpr uint8_t  REG_ARR   = 0x44;
    static constexpr double   CLOCK_HZ  = 72.0e6;
    static constexpr int      SERVO_HZ  = 50;
    static constexpr int      SERVO_ARR = 4095;
    static constexpr int      N_SERVO   = 12;
    static constexpr int      N_ADC     = 5;      // A0..A3 user, A4 battery
    static constexpr double   ADC_VREF  = 3.3;
    static constexpr int      ADC_MAX   = 4095;
    static constexpr double   VBAT_DIV  = 3.0;    // 20K/10K divider

    explicit RobotHat(I2cBus& bus) : bus_(bus) {}

    // Program the timer that carries `channel` for 50 Hz servo frames.
    // Must be called once per timer before pulses mean anything.
    void setup_servo_timer(int channel);
    // Pulse width in microseconds.  0 -> pulses stop -> the servo goes limp.
    void set_pulse_us(int channel, int us);
    void limp(int channel) { set_pulse_us(channel, 0); }
    // Raw on-count for the channel register (bench experiments only: 0..ARR).
    void set_pulse_raw(int channel, int count);
    // Any register, any 16-bit value (bench experiments only).
    void write_raw(uint8_t reg, int value) { write_reg(reg, value); }
    void limp_all();

    int    adc_raw(int channel);                 // 0..4095
    double adc_volts(int channel);               // 0..3.3
    double battery_volts() { return adc_volts(4) * VBAT_DIV; }

    // Pure helpers, exposed for the tests.
    static int pulse_count(int us);
    static int prescaler_value();                // the value WRITTEN (psc - 1)
    static double actual_frame_hz();

private:
    void write_reg(uint8_t reg, int value);
    I2cBus& bus_;
};

} // namespace ogma::hw
