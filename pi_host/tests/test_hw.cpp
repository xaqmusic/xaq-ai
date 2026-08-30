// Protocol + envelope tests against a fake bus.  The byte sequences here are the
// ones SunFounder's robot_hat 2.5.5 puts on the wire, cross-checked on the bench
// 2026-08-28 (ADC read of A4 = 7.65 V pack; P0 moved at 1300/1500/1700 us).
#include "ogma/hw/ServoDriver.hpp"
#include "ogma/hw/Ina219.hpp"
#include <gtest/gtest.h>

using namespace ogma::hw;

struct FakeI2cBus : I2cBus {
    uint8_t expect_addr = RobotHat::ADDR;
    std::vector<std::vector<uint8_t>> writes;
    std::vector<uint8_t> read_queue;
    int byte_reads = 0, block_reads = 0;
    void write(uint8_t addr, const std::vector<uint8_t>& b) override {
        EXPECT_EQ(addr, expect_addr);
        writes.push_back(b);
    }
    uint8_t read_byte(uint8_t) override {
        ++byte_reads;
        EXPECT_FALSE(read_queue.empty());
        uint8_t v = read_queue.front(); read_queue.erase(read_queue.begin()); return v;
    }
    std::vector<uint8_t> read_bytes(uint8_t, std::size_t n) override {
        ++block_reads;
        EXPECT_GE(read_queue.size(), n);
        std::vector<uint8_t> out(read_queue.begin(), read_queue.begin() + n);
        read_queue.erase(read_queue.begin(), read_queue.begin() + n);
        return out;
    }
    std::vector<uint8_t> last() const { return writes.empty() ? std::vector<uint8_t>{} : writes.back(); }
    // Queue a 16-bit register value the way the part returns it: MSB first.
    void queue_reg(uint16_t v) { read_queue.push_back(v >> 8); read_queue.push_back(v & 0xFF); }
};

// ---------------------------------------------------------------------------
// INA219 — battery-inline current monitor (BOM §3).  Everything here is byte- or
// datasheet-level; nothing needs the part present.
// ---------------------------------------------------------------------------

// A fake wired for 0x40 and the R010 shunt we run inline on the pack.
struct InaFixture {
    FakeI2cBus bus;
    Ina219     ina{bus, 0.01, Ina219::ADDR_DEFAULT};
    InaFixture() { bus.expect_addr = Ina219::ADDR_DEFAULT; }
};

TEST(Ina219Protocol, ConfigWordReproducesTheDatasheetPowerOnDefault) {
    // The strongest available check on the bit layout: 0x399F is published, and
    // it is exactly BRNG=32V, PGA/8, 12-bit both, shunt+bus continuous.
    Ina219Config d;
    d.brng = BusRange::V32; d.pga = Pga::Div8;
    d.badc = Adc::Bits12;   d.sadc = Adc::Bits12;
    d.mode = Mode::ShuntBusContinuous;
    EXPECT_EQ(Ina219::config_word(d), 0x399F);
}

TEST(Ina219Protocol, OurDefaultsAre16VRangeAndWidestPga) {
    // 16 V range for a 2S pack (6.0-8.4 V); PGA/8 so an inrush peak cannot clip.
    EXPECT_EQ(Ina219::config_word(Ina219Config{}), 0x199F);
    EXPECT_EQ(Ina219::pga_full_scale_v(Pga::Div8), 0.320);
    EXPECT_EQ(Ina219::pga_clip_counts(Pga::Div8), 32000);   // 0.320 V / 10 uV
    // Widening the PGA costs no resolution: the shunt LSB is 10 uV on every range.
    EXPECT_EQ(Ina219::SHUNT_LSB_V, 10e-6);
}

TEST(Ina219Protocol, CaptureConfigIsShuntOnlyAtFullSpeed) {
    // The inrush measurement: shunt only, 532 us -> ~1.9 kHz.  Pack voltage is
    // not what browns out, so the bus channel is dropped to halve the period.
    EXPECT_EQ(Ina219::config_word(ina219_capture_config()), 0x181D);
    EXPECT_EQ(Ina219::conversion_time_us(Adc::Bits12), 532);
    EXPECT_EQ(Ina219::conversion_time_us(Adc::Avg128), 68100);
}

TEST(Ina219Protocol, CalibrationWordForBothShunts) {
    // R100 (stock) and R010 (fitted for the brownout measurement) both land on
    // cal = 4096; only the current LSB and the full scale move.
    EXPECT_DOUBLE_EQ(Ina219::choose_current_lsb(0.1,  Pga::Div8), 100e-6);
    EXPECT_DOUBLE_EQ(Ina219::choose_current_lsb(0.01, Pga::Div8), 1e-3);
    EXPECT_EQ(Ina219::calibration_word(0.1,  100e-6), 4096);
    EXPECT_EQ(Ina219::calibration_word(0.01, 1e-3),   4096);
    // Bit 0 of the calibration register is void and always reads back 0.
    EXPECT_EQ(Ina219::calibration_word(0.0091, 2e-3) & 1u, 0u);
}

TEST(Ina219Protocol, ConfigureWritesConfigThenCalibration) {
    InaFixture f;
    f.ina.configure(ina219_capture_config());
    ASSERT_EQ(f.bus.writes.size(), 2u);
    EXPECT_EQ(f.bus.writes[0], (std::vector<uint8_t>{0x00, 0x18, 0x1D}));   // CONFIG
    EXPECT_EQ(f.bus.writes[1], (std::vector<uint8_t>{0x05, 0x10, 0x00}));   // CALIB = 4096
    EXPECT_EQ(f.ina.calibration_word(), 4096);
}

TEST(Ina219Protocol, RegisterReadIsPointerWriteThenOneTwoByteTransaction) {
    // The trap this guards: two read_byte() calls are two START..STOPs, and the
    // INA219 restarts at the MSB each time -- it would return the high byte twice.
    InaFixture f;
    f.bus.queue_reg(0x0BB8);
    EXPECT_EQ(f.ina.read_shunt_raw(), 3000);
    EXPECT_EQ(f.bus.last(), (std::vector<uint8_t>{0x01}));   // pointer = SHUNT_V
    EXPECT_EQ(f.bus.block_reads, 1);
    EXPECT_EQ(f.bus.byte_reads,  0);
}

TEST(Ina219Protocol, BusVoltageDropsTheThreeFlagBitsAndSurfacesThem) {
    InaFixture f;
    f.bus.queue_reg(0);                                       // shunt = 0
    f.bus.queue_reg((1850u << 3) | 0x2);                      // 7.400 V, CNVR set
    auto s = f.ina.read();
    EXPECT_NEAR(s.bus_v, 7.400, 1e-9);
    EXPECT_TRUE(s.conversion_ready);
    EXPECT_FALSE(s.overflow);
    EXPECT_EQ(s.bus_raw, (1850u << 3) | 0x2);                 // the record keeps it raw
    f.bus.queue_reg(0);
    f.bus.queue_reg((1500u << 3) | 0x1);                      // OVF: chip math invalid
    EXPECT_TRUE(f.ina.read().overflow);
}

TEST(Ina219Protocol, ShuntIsSignedSoAReversedInstallReadsNegativeNotGarbage) {
    // Vin+/Vin- swapped is a realistic wiring mistake.  It must be VISIBLE.
    InaFixture f;
    f.bus.queue_reg(0xFA24);                                  // -1500 counts
    f.bus.queue_reg(1850u << 3);
    auto s = f.ina.read();
    EXPECT_EQ(s.shunt_raw, -1500);
    EXPECT_NEAR(s.shunt_v, -0.015, 1e-12);
    EXPECT_NEAR(s.current_a, -1.5, 1e-9);                     // -0.015 V / 0.01 ohm
}

TEST(Ina219Protocol, AClippedPeakIsFlaggedNotReportedAsMeasured) {
    // At full scale the reading is a FLOOR on the truth.  Under-measuring the
    // inrush silently is the one failure this sensor exists to avoid.
    InaFixture f;
    f.bus.queue_reg(32000); f.bus.queue_reg(1850u << 3);
    EXPECT_TRUE(f.ina.read().pga_clipped);
    f.bus.queue_reg(31999); f.bus.queue_reg(1850u << 3);
    EXPECT_FALSE(f.ina.read().pga_clipped);
}

TEST(Ina219Protocol, CurrentIsHostDerivedSoARefitRescalesTheRecord) {
    // r_shunt is calibration data, not a constant: a bench re-fit must re-derive
    // already-recorded raw samples rather than strand them behind a stale value.
    InaFixture f;
    EXPECT_NEAR(f.ina.shunt_to_amps(1000), 1.0, 1e-12);       // 10 mV / 0.01 ohm
    f.ina.set_r_shunt(0.0091);                                // R010 || stock R100
    EXPECT_NEAR(f.ina.shunt_to_amps(1000), 1.0989010989, 1e-9);
    EXPECT_EQ(f.bus.last()[0], Ina219::REG_CALIB);            // and the chip follows
}

TEST(Ina219Protocol, RejectsAShuntThatCannotBeCalibrated) {
    FakeI2cBus bus;
    EXPECT_THROW(Ina219(bus, 0.0), std::invalid_argument);
    EXPECT_THROW(Ina219::calibration_word(1e-9, 1e-9), std::invalid_argument);
}

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
