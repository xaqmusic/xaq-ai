// Protocol + envelope tests against a fake bus.  The byte sequences here are the
// ones SunFounder's robot_hat 2.5.5 puts on the wire, cross-checked on the bench
// 2026-08-28 (ADC read of A4 = 7.65 V pack; P0 moved at 1300/1500/1700 us).
#include "ogma/hw/ServoDriver.hpp"
#include "ogma/hw/Ina219.hpp"
#include "ogma/hw/ResourceMonitor.hpp"
#include <cstdlib>
#include <fstream>
#include <filesystem>
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


// ---------------------------------------------------------------------------
// ResourceMonitor — what the loop costs, in tick-budget percent.
// ---------------------------------------------------------------------------

TEST(TickBudget, ReportsPercentOfTheTickPeriodNotOfAWallClockSecond) {
    TickBudget b(50.0, 4);                       // 50 Hz -> a 20 ms budget
    EXPECT_FALSE(b.sample(2000000, 1000000));    // 2 ms wall / 1 ms cpu = 10 % / 5 %
    EXPECT_FALSE(b.sample(2000000, 1000000));
    EXPECT_FALSE(b.sample(2000000, 1000000));
    EXPECT_TRUE (b.sample(2000000, 1000000));    // 4th closes the window
    EXPECT_EQ(b.last().n, 4);
    EXPECT_NEAR(b.last().budget_ms, 20.0, 1e-9);
    EXPECT_NEAR(b.last().wall_p50, 10.0, 1e-9);
    EXPECT_NEAR(b.last().cpu_p50,   5.0, 1e-9);
}

TEST(TickBudget, TheMaxCatchesTheSpikeThatAMeanHides) {
    // The blind-metric case this instrument exists for: 39 quiet ticks and one
    // that blows the budget.  The mean says 17 %; the loop still missed a deadline.
    TickBudget b(50.0, 40);
    for (int i = 0; i < 39; ++i) b.sample(3000000, 3000000);   // 15 %
    ASSERT_TRUE(b.sample(22000000, 22000000));                 // 110 % -- an overrun
    EXPECT_NEAR(b.last().wall_p50, 15.0, 1e-9);                // middle looks fine
    EXPECT_NEAR(b.last().wall_max, 110.0, 1e-9);               // the tail does not
    EXPECT_GT(b.last().wall_max, 100.0);
}

TEST(TickBudget, WallAndCpuAreSeparateSoBlockedIsDistinguishableFromBusy) {
    // 15 ms of wall for 1 ms of compute = blocked on the I2C bus, not out of CPU.
    // Same wall time with 14 ms of compute would be the opposite diagnosis and the
    // opposite response, so one number cannot serve.
    TickBudget b(50.0, 2);
    b.sample(15000000, 1000000);
    ASSERT_TRUE(b.sample(15000000, 1000000));
    EXPECT_NEAR(b.last().wall_max, 75.0, 1e-9);
    EXPECT_NEAR(b.last().cpu_max,   5.0, 1e-9);
}

TEST(TickBudget, PercentileIsNearestRankSoEveryNumberIsARealTick) {
    std::vector<int64_t> v{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    EXPECT_EQ(TickBudget::percentile(v, 0.50), 6.0);
    EXPECT_EQ(TickBudget::percentile(v, 0.95), 10.0);
    EXPECT_EQ(TickBudget::percentile(v, 0.00), 1.0);
    EXPECT_EQ(TickBudget::percentile({}, 0.5), 0.0);           // empty window
}

TEST(HostStats, ParsesKbFieldsAndIsNotFooledByAKeyThatIsASuffixOfAnother) {
    const std::string status =
        "Name:\tognma\nVmRSS:\t   41216 kB\nVmSwap:\t       0 kB\nRssAnon:\t 9999 kB\n";
    EXPECT_NEAR(HostStats::parse_kb_field(status, "VmRSS"),  41216.0 / 1024.0, 1e-9);
    EXPECT_NEAR(HostStats::parse_kb_field(status, "VmSwap"), 0.0, 1e-9);
    // "Rss" must not match inside "VmRSS"/"RssAnon" at a non-line-start.
    EXPECT_LT(HostStats::parse_kb_field(status, "Rss"), 0.0);
    EXPECT_LT(HostStats::parse_kb_field(status, "Nope"), 0.0);
}

TEST(HostStats, StatParseSurvivesACommContainingSpacesAndParens) {
    // The classic /proc/<pid>/stat trap: field 2 is the executable name in
    // parentheses and may contain both spaces and ')'.  Splitting on whitespace
    // from the left silently shifts every field after it.
    const std::string stat =
        "1234 (ogma benchd (x)) S 1 1234 1234 0 -1 4194304 5000 0 "
        "7 0 111 222 0 0 20 0 4 0 99 0 0";
    long ut = 0, st = 0, mf = 0;
    ASSERT_TRUE(HostStats::parse_stat_cpu(stat, ut, st, mf));
    EXPECT_EQ(mf, 7);          // field 12
    EXPECT_EQ(ut, 111);        // field 14
    EXPECT_EQ(st, 222);        // field 15
    EXPECT_FALSE(HostStats::parse_stat_cpu("no parens here", ut, st, mf));
}

TEST(HostStats, ReadsAFixtureTreeAndReportsUnreadableRootsAsNotOk) {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / "ogma_hoststats_fixture";
    fs::remove_all(root);
    fs::create_directories(root / "proc" / "self");
    fs::create_directories(root / "sys" / "class" / "thermal" / "thermal_zone0");
    std::ofstream(root / "proc" / "self" / "status")
        << "VmRSS:\t  102400 kB\nVmSwap:\t   2048 kB\n";
    std::ofstream(root / "proc" / "self" / "stat")
        << "1 (x) S 1 1 1 0 -1 0 0 0 3 0 10 20 0 0 20 0 4 0 0 0 0\n";
    std::ofstream(root / "proc" / "meminfo") << "MemTotal:\t2000000 kB\nMemAvailable:\t1433600 kB\n";
    std::ofstream(root / "sys" / "class" / "thermal" / "thermal_zone0" / "temp") << "52104\n";

    HostStats hs((root / "proc").string(), (root / "sys").string());
    const auto s = hs.sample();
    EXPECT_TRUE(s.ok);
    EXPECT_NEAR(s.rss_mb,       100.0, 1e-9);
    EXPECT_NEAR(s.swap_mb,        2.0, 1e-9);
    EXPECT_NEAR(s.mem_avail_mb, 1400.0, 1e-9);
    EXPECT_NEAR(s.cpu_temp_c,   52.104, 1e-6);
    EXPECT_EQ(s.majflt, 3);
    EXPECT_EQ(s.proc_cpu_pct, 0.0);        // no previous sample to difference against

    HostStats missing((root / "nope").string(), (root / "nope").string());
    EXPECT_FALSE(missing.sample().ok);
    fs::remove_all(root);
}


#ifdef PI_HOST_HAVE_SENSORS
#include "ogma/hw/Ultrasonic.hpp"
#include "ogma/hw/CameraCapture.hpp"

// ---------------------------------------------------------------------------
// Sensor helpers.  Pure arithmetic and pure resampling — no device, no thread.
// ---------------------------------------------------------------------------

TEST(Ultrasonic, SpeedOfSoundTracksTemperatureRatherThanAssuming343) {
    // Hardcoding 343 m/s costs ~7 % across 0-40 C, and 7 % of every distance this
    // sensor reports is not a rounding error.
    EXPECT_NEAR(Ultrasonic::speed_of_sound_mps(0.0),  331.3, 1e-9);
    EXPECT_NEAR(Ultrasonic::speed_of_sound_mps(20.0), 343.42, 1e-6);
    EXPECT_GT(Ultrasonic::speed_of_sound_mps(40.0), Ultrasonic::speed_of_sound_mps(0.0) * 1.06);
}

TEST(Ultrasonic, PulseWidthToMetresHalvesForTheReturnTrip) {
    // The measured bench ping: 866 us of echo, with a wall about 15 cm away.
    const double d = Ultrasonic::pulse_to_metres(866000, 20.0);
    EXPECT_NEAR(d, 0.1487, 0.001);
    // Out AND back: forgetting the factor of two doubles every reading.
    EXPECT_NEAR(Ultrasonic::pulse_to_metres(2000000, 20.0), 0.34342, 1e-5);
    EXPECT_EQ(Ultrasonic::pulse_to_metres(0, 20.0), 0.0);
}

TEST(CameraCapture, ReduceAreaAveragesRatherThanDecimating) {
    // A 4x4 checkerboard reduced to 2x2 must give the MEAN (127), not whichever
    // corner pixel a nearest-neighbour decimation happened to land on (0 or 255).
    std::vector<uint8_t> src(16);
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x) src[size_t(y*4+x)] = ((x + y) % 2) ? 255 : 0;
    std::vector<uint8_t> dst;
    CameraCapture::reduce(src.data(), 4, 4, 4, 2, dst);
    ASSERT_EQ(dst.size(), 4u);
    for (uint8_t v : dst) EXPECT_NEAR(int(v), 127, 1);
}

TEST(CameraCapture, ReduceCentreCropsSoTheSceneIsNotSquashed) {
    // 4 wide x 2 high: only the centre 2x2 square may contribute, so the outer
    // columns (marked 255) must not reach the output.
    std::vector<uint8_t> src(8, 0);
    src[0] = src[3] = src[4] = src[7] = 255;          // the cropped-away columns
    std::vector<uint8_t> dst;
    CameraCapture::reduce(src.data(), 4, 2, 4, 2, dst);
    ASSERT_EQ(dst.size(), 4u);
    for (uint8_t v : dst) EXPECT_EQ(int(v), 0);
}

TEST(CameraCapture, ReduceIsTotalOnDegenerateInput) {
    std::vector<uint8_t> dst;
    CameraCapture::reduce(nullptr, 0, 0, 0, 4, dst);
    EXPECT_EQ(dst.size(), 16u);                        // sized, zeroed, never a crash
    std::vector<uint8_t> one{200};
    CameraCapture::reduce(one.data(), 1, 1, 1, 2, dst);   // upscale: every cell the pixel
    ASSERT_EQ(dst.size(), 4u);
    for (uint8_t v : dst) EXPECT_EQ(int(v), 200);
}
TEST(Ultrasonic, MaxRangeIsTheUsableRangeBecauseItIsWhatNoEchoReportsAs) {
    // Regression guard for a real bug (2026-09-01): a no-echo ping used to publish
    // distance 0, which maps "nothing within range" onto "something against the
    // sensor" -- the opposite extreme -- and it poisoned the commissioned range, whose
    // lower bound came back as exactly 0.0 against a true band of 0.03-1.2 m.
    // The default must stay the module's USABLE range, not the datasheet's 4 m,
    // because an over-wide value re-inflates the axis commissioning just calibrated.
    Ultrasonic::Config cfg;
    EXPECT_LT(cfg.max_range_m, 2.0) << "an over-wide no-echo value re-inflates the axis";
    EXPECT_GT(cfg.max_range_m, 1.2) << "must cover the observed 1.43 m maximum";
    EXPECT_DOUBLE_EQ(cfg.hz, 20.0);   // measured ceiling; 30 Hz degrades, 40 Hz ghosts
}

TEST(CameraCapture, StrideIsTheRowPitchAndPaddingNeverEntersTheAverage) {
    // The bug this guards, measured 2026-09-01: rpicam-vid emits a Y plane whose row
    // pitch is the width rounded UP to a multiple of 128, so a 160-wide frame is really
    // 256 bytes per row.  Reading width-as-stride consumes 28800 of a 46080-byte frame,
    // slides across frame boundaries forever, and yields a repeating pattern that ignores
    // the scene while mean brightness still looks plausible.
    EXPECT_EQ(CameraCapture::stride_for_width(160), 256);
    EXPECT_EQ(CameraCapture::stride_for_width(192), 256);
    EXPECT_EQ(CameraCapture::stride_for_width(320), 384);
    EXPECT_EQ(CameraCapture::stride_for_width(128), 128);   // already aligned
    EXPECT_EQ(CameraCapture::stride_for_width(256), 256);
    EXPECT_EQ(CameraCapture::stride_for_width(640), 640);
    EXPECT_EQ(CameraCapture::frame_bytes_for(256, 120), 46080u);   // the measured value
    EXPECT_EQ(CameraCapture::frame_bytes_for(256, 192), 73728u);   // the new default

    // 4 wide but padded to a pitch of 8: the right-hand 4 bytes are garbage padding and
    // must not reach the output, which is 0 from the real columns.
    std::vector<uint8_t> padded(8 * 4, 255);
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x) padded[size_t(y * 8 + x)] = 0;   // real pixels are 0
    std::vector<uint8_t> dst;
    CameraCapture::reduce(padded.data(), 4, 4, 8, 2, dst);
    ASSERT_EQ(dst.size(), 4u);
    for (uint8_t v : dst) EXPECT_EQ(int(v), 0) << "ISP padding leaked into the image";
}
#endif  // PI_HOST_HAVE_SENSORS
