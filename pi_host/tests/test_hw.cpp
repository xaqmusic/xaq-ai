// Protocol + envelope tests against a fake bus.  The byte sequences here are the
// ones SunFounder's robot_hat 2.5.5 puts on the wire, cross-checked on the bench
// 2026-08-28 (ADC read of A4 = 7.65 V pack; P0 moved at 1300/1500/1700 us).
#include "ogma/hw/ServoDriver.hpp"
#include <gtest/gtest.h>

using namespace ogma::hw;

struct FakeI2cBus : I2cBus {
    std::vector<std::vector<uint8_t>> writes;
    std::vector<uint8_t> read_queue;
    void write(uint8_t addr, const std::vector<uint8_t>& b) override {
        EXPECT_EQ(addr, RobotHat::ADDR);
        writes.push_back(b);
    }
    uint8_t read_byte(uint8_t) override {
        EXPECT_FALSE(read_queue.empty());
        uint8_t v = read_queue.front(); read_queue.erase(read_queue.begin()); return v;
    }
    std::vector<uint8_t> last() const { return writes.empty() ? std::vector<uint8_t>{} : writes.back(); }
};

TEST(RobotHatProtocol, TimerSetupWritesPrescalerAndPeriod) {
    FakeI2cBus bus; RobotHat hat(bus);
    hat.setup_servo_timer(5);                       // channel 5 -> timer 1
    ASSERT_EQ(bus.writes.size(), 2u);
    EXPECT_EQ(RobotHat::prescaler_value(), 351);    // round(72e6/50/4095)=352, minus 1
    EXPECT_EQ(bus.writes[0], (std::vector<uint8_t>{0x41, 0x01, 0x5F}));   // PSC+1 = 351 = 0x015F
    EXPECT_EQ(bus.writes[1], (std::vector<uint8_t>{0x45, 0x0F, 0xFF}));   // ARR+1 = 4095
    EXPECT_NEAR(RobotHat::actual_frame_hz(), 49.95, 0.01);
}

TEST(RobotHatProtocol, PulseCountMatchesLibraryTruncation) {
    EXPECT_EQ(RobotHat::pulse_count(1500), 307);    // int(1500/20000*4095) = 307.125 -> 307
    EXPECT_EQ(RobotHat::pulse_count(500),  102);
    EXPECT_EQ(RobotHat::pulse_count(2500), 511);
    EXPECT_EQ(RobotHat::pulse_count(0),    0);
    FakeI2cBus bus; RobotHat hat(bus);
    hat.set_pulse_us(0, 1500);
    EXPECT_EQ(bus.last(), (std::vector<uint8_t>{0x20, 0x01, 0x33}));       // CHN+0 = 307 = 0x0133
    hat.set_pulse_us(11, 0);
    EXPECT_EQ(bus.last(), (std::vector<uint8_t>{0x2B, 0x00, 0x00}));       // limp = pulse 0
}

TEST(RobotHatProtocol, AdcSelectAndReadAndBatteryScale) {
    FakeI2cBus bus; RobotHat hat(bus);
    bus.read_queue = {0x0C, 0x5B};                  // 3163 = the bench reading
    EXPECT_EQ(hat.adc_raw(4), 3163);
    EXPECT_EQ(bus.last(), (std::vector<uint8_t>{0x13, 0x00, 0x00}));       // (7-4)|0x10
    bus.read_queue = {0x0C, 0x5B};
    EXPECT_NEAR(hat.battery_volts(), 3163 * 3.3 / 4095 * 3.0, 1e-9);       // 7.65 V
    bus.read_queue = {0, 0};
    hat.adc_raw(0);
    EXPECT_EQ(bus.last()[0], 0x17);                                         // (7-0)|0x10
}

TEST(ServoDriver, ClampsToLimitsAndAccumulatesTimeAtLimit) {
    FakeI2cBus bus; RobotHat hat(bus); ServoDriver d(hat, {40, 0, 50.0});
    d.set_limits(3, {1200, 1800});
    d.command(3, 2500);
    EXPECT_EQ(d.target_us(3), 1800);
    for (int i = 0; i < 50; ++i) d.tick();
    EXPECT_EQ(d.current_us(3), 1800);
    EXPECT_NEAR(d.time_at_limit_s(3), 1.0, 1e-9);
}

TEST(ServoDriver, SlewLimitsRateAndFirstCommandDoesNotSweepIn) {
    FakeI2cBus bus; RobotHat hat(bus); ServoDriver d(hat, {40, 0, 50.0});
    d.command(0, 1500);
    EXPECT_EQ(d.current_us(0), 1500);              // no sweep from 0 on first arm
    d.tick();
    EXPECT_EQ(d.current_us(0), 1500);
    d.command(0, 1700);
    d.tick(); EXPECT_EQ(d.current_us(0), 1540);
    d.tick(); EXPECT_EQ(d.current_us(0), 1580);
    for (int i = 0; i < 10; ++i) d.tick();
    EXPECT_EQ(d.current_us(0), 1700);
    EXPECT_EQ(bus.last(), (std::vector<uint8_t>{0x20, 0x01, 0x5C}));       // 1700us -> 348 = 0x15C
}

TEST(ServoDriver, WatchdogLimpsWhenCommandsStop) {
    FakeI2cBus bus; RobotHat hat(bus); ServoDriver d(hat, {40, 5, 50.0});
    d.command(2, 1500);
    for (int i = 0; i < 5; ++i) d.tick();
    EXPECT_FALSE(d.watchdog_tripped());
    d.tick();                                       // 6th silent tick > 5
    EXPECT_TRUE(d.watchdog_tripped());
    EXPECT_FALSE(d.armed(2));
    // last 12 writes are pulse 0 on every channel
    ASSERT_GE(bus.writes.size(), 12u);
    for (int c = 0; c < 12; ++c) {
        auto w = bus.writes[bus.writes.size() - 12 + c];
        EXPECT_EQ(w, (std::vector<uint8_t>{static_cast<uint8_t>(0x20 + c), 0, 0}));
    }
    d.command(2, 1500);                             // re-arming clears the trip
    EXPECT_FALSE(d.watchdog_tripped());
}

TEST(ServoDriver, TimerIsProgrammedOncePerTimerOnFirstArm) {
    FakeI2cBus bus; RobotHat hat(bus); ServoDriver d(hat, {40, 0, 50.0});
    d.command(0, 1500); d.command(1, 1500); d.command(4, 1500);
    int psc_writes = 0;
    for (auto& w : bus.writes) if (w[0] >= 0x40 && w[0] < 0x44) ++psc_writes;
    EXPECT_EQ(psc_writes, 2);                       // timers 0 and 1 only
}
