#pragma once
// Ina219 — TI INA219 high-side current/power monitor, wire protocol (not a library).
// Sits inline on the 2S battery input (BOM §3): it measures the WHOLE robot —
// Pi 5 + the HAT's 5 V regulator + all 12 servos.  Its first job is the one
// measurement the bench cannot currently make: the inrush of a pose recall, which
// browns the Pi out on the shared 5 V/3 A rail (ledger 2026-08-29).  A4 is blind
// to that transient and vcgencmd takes ~0.7 s per read; this part resolves 532 us.
//
//   I2C 0x40 (A0/A1 straps).  Six 16-bit registers, big-endian, behind a pointer:
//     0x00 CONFIG   0x01 SHUNT_V   0x02 BUS_V   0x03 POWER   0x04 CURRENT   0x05 CALIBRATION
//   A register read is write([reg]) then a 2-byte read transaction — the pointer
//   persists across the STOP, so I2cBus::read_bytes() is required (two read_byte()
//   calls would return the high byte twice).
//
//   SHUNT_V : int16, LSB 10 uV, on EVERY PGA range.  The PGA sets full scale only
//             (+/-40/80/160/320 mV), so a wider range costs no resolution.  We
//             default to /8 because a clipped inrush reading is an under-measurement
//             that looks like a real number.
//   BUS_V   : bits 15:3 = value, LSB 4 mV; bit 1 = CNVR (ready); bit 0 = OVF.
//
// ⚠ r_shunt is CALIBRATION DATA, not a constant.  At 10 mOhm the trace and solder
// resistance are a large fraction of the part, and a multimeter cannot measure it
// (probe leads are ~200 mOhm).  It is fitted on the bench against a known current
// and stored in calib JSON.  Consequently the AUTHORITATIVE current here is derived
// on the host from the raw shunt microvolts — so a later re-fit of r_shunt re-derives
// every recorded sample, instead of stranding the record behind a stale constant.
// The chip's own CURRENT register is programmed too, but only as a cross-check.
#include "ogma/hw/I2cBus.hpp"

namespace ogma::hw {

enum class BusRange { V16, V32 };                    // 2S pack (6.0-8.4 V) -> V16
enum class Pga      { Div1, Div2, Div4, Div8 };      // +/-40, 80, 160, 320 mV

// BADC/SADC field values: 9..12-bit single conversions, then 2..128-sample averages.
enum class Adc : uint8_t {
    Bits9 = 0x0, Bits10 = 0x1, Bits11 = 0x2, Bits12 = 0x3,
    Avg2  = 0x9, Avg4   = 0xA, Avg8   = 0xB, Avg16 = 0xC,
    Avg32 = 0xD, Avg64  = 0xE, Avg128 = 0xF,
};

enum class Mode : uint8_t {
    PowerDown = 0, ShuntTriggered = 1, BusTriggered = 2, ShuntBusTriggered = 3,
    AdcOff    = 4, ShuntContinuous = 5, BusContinuous = 6, ShuntBusContinuous = 7,
};

struct Ina219Config {
    BusRange brng = BusRange::V16;
    Pga      pga  = Pga::Div8;
    Adc      badc = Adc::Bits12;
    Adc      sadc = Adc::Bits12;
    Mode     mode = Mode::ShuntBusContinuous;
};

// Steady telemetry: 128-sample averages, ~68 ms per channel.  Smooths servo PWM
// ripple out of a number a human reads on the dashboard.
Ina219Config ina219_telemetry_config();
// Inrush capture: shunt only, 12-bit single conversions -> 532 us, ~1.9 kHz.
// The bus channel is dropped because pack voltage is not what browns out.
Ina219Config ina219_capture_config();

class Ina219 {
public:
    static constexpr uint8_t ADDR_DEFAULT = 0x40;
    static constexpr uint8_t REG_CONFIG   = 0x00;
    static constexpr uint8_t REG_SHUNT    = 0x01;
    static constexpr uint8_t REG_BUS      = 0x02;
    static constexpr uint8_t REG_POWER    = 0x03;
    static constexpr uint8_t REG_CURRENT  = 0x04;
    static constexpr uint8_t REG_CALIB    = 0x05;

    static constexpr double SHUNT_LSB_V = 10e-6;   // fixed on all PGA ranges
    static constexpr double BUS_LSB_V   = 4e-3;
    static constexpr double CAL_CONST   = 0.04096; // TI's fixed calibration numerator

    Ina219(I2cBus& bus, double r_shunt_ohm, uint8_t addr = ADDR_DEFAULT);

    void reset();                                  // CONFIG bit 15; leaves chip defaults
    void configure(const Ina219Config& cfg);       // CONFIG then CALIBRATION

    struct Sample {
        int16_t  shunt_raw    = 0;                 // as read, for the record
        uint16_t bus_raw      = 0;                 // as read, unshifted
        double   shunt_v      = 0.0;
        double   bus_v        = 0.0;
        double   current_a    = 0.0;               // host-derived: shunt_v / r_shunt
        bool     conversion_ready = false;
        bool     overflow     = false;             // CNVR/OVF from the bus register
        bool     pga_clipped  = false;             // |shunt_raw| at PGA full scale
    };

    Sample  read();                                // shunt + bus (two transactions)
    int16_t read_shunt_raw();                      // capture fast path (one transaction)
    double  shunt_to_amps(int16_t raw) const;      // re-derive a recorded raw sample

    double chip_current_a();                       // CURRENT reg — cross-check only
    double chip_power_w();                         // POWER reg   — cross-check only

    void     set_r_shunt(double ohms);             // re-fit, then CALIBRATION is rewritten
    double   r_shunt()      const { return r_shunt_; }
    double   current_lsb_a() const { return current_lsb_; }
    uint16_t calibration_word() const { return cal_word_; }
    Pga      pga()          const { return cfg_.pga; }

    // Pure helpers, exposed for the tests.
    static uint16_t config_word(const Ina219Config& cfg);
    static double   pga_full_scale_v(Pga pga);
    static int16_t  pga_clip_counts(Pga pga);
    static int      conversion_time_us(Adc adc);
    // Full-scale current the PGA range allows, and the LSB/calibration pair for it.
    static double   choose_current_lsb(double r_shunt_ohm, Pga pga);
    static uint16_t calibration_word(double r_shunt_ohm, double current_lsb_a);

private:
    uint16_t read_reg(uint8_t reg);
    void     write_reg(uint8_t reg, uint16_t value);
    void     write_calibration();

    I2cBus&      bus_;
    uint8_t      addr_;
    double       r_shunt_;
    double       current_lsb_ = 0.0;
    uint16_t     cal_word_    = 0;
    Ina219Config cfg_{};
};

} // namespace ogma::hw
