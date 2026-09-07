#pragma once
// Vl53l0x — ST VL53L0X time-of-flight rangefinder, wire protocol (not ST's API).
//
// ⚠ DOWNWARD, AT THE BELLY.  This is the `gc_raw` channel — the belly-clearance
// sensor the promoted height homeostat rides (BOM §7, port doc §7.7).  The
// Ultrasonic class is the FORWARD obstacle channel and is a different sensor with a
// different job; crossing the two would silently corrupt a promoted lever, so
// neither class knows about the other's topic.
//
//   I2C 0x29, 8-bit register addresses, big-endian multi-byte values.  A register
//   read is write([reg]) then a read transaction — the pointer persists across the
//   STOP, so I2cBus::read_bytes() is required for anything wider than a byte.
//
// WHY THIS IS NOT A THIN DRIVER.  The part ships unconfigured and cannot range until
// a long boot sequence has run: reference-SPAD selection (which SPADs are even wired
// on THIS die, read out of NVM), ~80 undocumented tuning register writes ST publishes
// only as a blob, and two reference calibrations (VHV and phase).  None of it is
// optional and none of it can be derived — it is transcribed from ST's API, and the
// blob is deliberately kept as a literal table rather than paraphrased into code,
// because a "tidier" version of an opaque list is just a version you cannot diff
// against the source.
//
// WHAT IT PUBLISHES, AND WHY IT IS MORE THAN MILLIMETRES.  Every measurement carries
// its own range status plus the signal and ambient return rates.  A ToF reading that
// failed its internal sigma/signal checks is not a small number or a large one — it
// is an arbitrary one, and it looks exactly like a good reading at the consumer.  The
// status IS the confound channel and the part computes it for free; throwing it away
// and publishing the millimetres alone would be the expensive mistake.
//
// ⚠ mount_offset_mm is CALIBRATION DATA, not a constant — the same contract as the
// INA219's r_shunt.  The sensor is recessed up inside the chassis so that the belly's
// 0-56 mm working range (geometry §G2: 56.3 mm standing, 9.5 mm at the crouch gate)
// sits above the part's unreliable short end, which means every reading carries a
// fixed additive offset that CAD does not know and a tape measure does.  So the
// AUTHORITATIVE record is raw_mm as the chip reported it, and the belly clearance is
// derived from it here — a later re-fit of the offset re-derives every recorded
// sample instead of stranding the record behind a stale constant.
#include "ogma/hw/I2cBus.hpp"

#include <chrono>
#include <cstdint>

namespace ogma::hw {

struct Vl53l0xConfig {
    // Distance from the chip's optical face down to the belly plane, in mm.  The
    // recess that keeps the working range off the part's weak short end (§ header).
    // 0 = flush-mounted, which is the pre-bench default and NOT the fitted value.
    double   mount_offset_mm = 0.0;
    // What an invalid reading is REPORTED AS, per Reading::distance_m.  Not a taste
    // setting: an over-wide value re-inflates the very axis commissioning calibrated.
    // 1.2 m is the part's honest indoor reach in default mode, not the 2 m headline.
    double   max_range_m     = 1.2;
    // ST's default is 33 ms.  Longer budget = lower sigma, at proportionally fewer
    // measurements per second; this must stay >= the 50 Hz control loop's need, so
    // the default keeps ~30 Hz and the knob is here for a bench accuracy sweep.
    uint32_t timing_budget_us = 33000;
    // Return-signal floor in Mcps below which a measurement fails SignalFail.
    // ST's default; raising it trades range for confidence.
    float    signal_rate_limit_mcps = 0.25f;
};

class Vl53l0x {
public:
    static constexpr uint8_t ADDR_DEFAULT = 0x29;
    // 0xC0 reads 0xEE on every VL53L0X.  Confirmed on the bench 2026-09-07 alongside
    // 0xC1 = 0xAA and 0xC2 = 0x10 (revision).  An address that ACKs proves a device is
    // wired; only the model ID proves it is THIS device and is answering coherently.
    static constexpr uint8_t REG_MODEL_ID = 0xC0;
    static constexpr uint8_t MODEL_ID     = 0xEE;

    // ST's PAL range status, which is what "is this reading usable" actually means.
    // Derived here from the device's own 4-bit status; see decode_status().
    enum class Status : uint8_t {
        Valid        = 0,
        SigmaFail    = 1,   // return spread too wide — the distance is a guess
        SignalFail   = 2,   // too little light came back (dark/absorbent/far)
        MinRangeFail = 3,   // target closer than the part can resolve — READ THE COMMENT
        PhaseFail    = 4,   // return phase out of bounds — wrapped or multi-path
        HardwareFail = 5,   // VCSEL/VHV fault; the part, not the scene
        NoUpdate     = 255, // ST's "None": no measurement was produced at all
    };
    static const char* status_name(Status s);

    struct Reading {
        // RAW, as the chip reported it, before the mount offset.  This is what gets
        // recorded; distance_m is derived (see the mount_offset_mm note above).
        uint16_t raw_mm       = 0;
        // Belly clearance in metres: (raw_mm - mount_offset_mm) / 1000, floored at 0.
        // ⚠ On an INVALID reading this is max_range_m, NOT zero — the same encoding
        // Ultrasonic uses and for the same reason.  Zero maps "saw nothing" onto
        // "something against the sensor", which is the opposite extreme and the worst
        // available answer.  Nothing returned means nothing within range, so the far
        // limit is the honest floor and `valid` carries the caveat.
        double   distance_m   = 0.0;
        Status   status       = Status::NoUpdate;
        uint8_t  device_status = 0;      // the raw 4-bit device code, for the record
        // The confound channel.  signal is what came back off the target; ambient is
        // what the room contributed.  A collapsing signal-to-ambient ratio is the tell
        // for a reading about to go bad, and it moves BEFORE the status flips.
        double   signal_mcps  = 0.0;
        double   ambient_mcps = 0.0;
        // Effective SPADs in the return, already scaled out of 8.8 fixed point.  A
        // drop here with the distance unchanged is the tell for an obstructed or
        // fouled aperture — the part is seeing the target through fewer detectors.
        double   spads        = 0.0;
        bool     valid        = false;
        uint64_t seq          = 0;
    };


    using Config = Vl53l0xConfig;

    Vl53l0x(I2cBus& bus, Vl53l0xConfig cfg = {}, uint8_t addr = ADDR_DEFAULT);

    // Hardware soft reset (ST's VL53L0X_ResetDevice), then wait for the part to boot.
    // init() calls this first; it is exposed because "put it back how it powered up" is
    // a useful thing to be able to ask for on its own.
    void reset_device();

    // Full boot: soft reset, model-ID check, SPAD map, tuning blob, ref cal, budget.
    // Throws std::runtime_error with a stage name if any of it fails — a half-configured
    // ToF returns plausible numbers, so there is no partial success worth reporting.
    void init();
    bool model_id_ok();                    // the cheap liveness check, on its own

    // Continuous back-to-back ranging.  period_ms 0 = as fast as the timing budget
    // allows; non-zero = inter-measurement timed mode.
    void start_continuous(uint32_t period_ms = 0);
    void stop_continuous();

    bool data_ready();                     // one register read, never blocks
    // Non-blocking: false (and out untouched) when no measurement is ready.  This is
    // the shape benchd needs — its telemetry thread holds the bus mutex that the 50 Hz
    // servo tick also wants, so it can poll but must never wait on a conversion.
    bool read_ready(Reading& out);
    // Blocking, for the bench tools.  Throws on timeout.
    Reading read_blocking(std::chrono::milliseconds timeout = std::chrono::milliseconds(500));

    uint32_t timing_budget_us();           // as the part currently has it, re-derived
    void     set_timing_budget_us(uint32_t us);
    void     set_signal_rate_limit(float mcps);
    void     set_mount_offset_mm(double mm) { cfg_.mount_offset_mm = mm; }

    const Vl53l0xConfig& config() const { return cfg_; }
    uint64_t reads()   const { return reads_; }
    uint64_t invalid() const { return invalid_; }

    // Pure helpers, exposed for the tests.  Everything below is arithmetic on the
    // datasheet's encodings and needs no part present.
    static uint16_t decode_timeout(uint16_t reg);
    static uint16_t encode_timeout(uint32_t mclks);
    static uint32_t calc_macro_period_ns(uint8_t vcsel_period_pclks);
    static uint32_t timeout_mclks_to_us(uint16_t mclks, uint8_t vcsel_period_pclks);
    static uint32_t timeout_us_to_mclks(uint32_t us, uint8_t vcsel_period_pclks);
    static uint8_t  decode_vcsel_period(uint8_t reg) { return uint8_t((reg + 1) << 1); }
    // 9.7 fixed point -> Mcps, the encoding of the signal/ambient rate fields.
    static double   fixpoint97_to_mcps(uint16_t v) { return v / 128.0; }
    // 8.8 fixed point -> a count, the encoding of the effective SPAD field.  A
    // DIFFERENT fixed-point format in the same 12 bytes as the rates above, which is
    // exactly how it gets published raw and read as tens of thousands of SPADs.
    static double   fixpoint88_to_count(uint16_t v) { return v / 256.0; }
    // The device's 4-bit range status -> ST's PAL status.  ⚠ This implements the part
    // of ST's mapping that depends on the device code alone.  The full API also raises
    // SigmaFail from a sigma estimate and MinRangeFail from a signal-ref clip flag,
    // both of which need state this driver does not carry — so a Valid here means "the
    // device did not itself report a failure", which is weaker than the full API's
    // Valid.  The rates in Reading are published so a consumer can be stricter.
    static Status   decode_status(uint8_t device_status);
    // (raw_mm - offset) clamped at zero, in metres.  The belly cannot be behind itself.
    static double   raw_to_clearance_m(uint16_t raw_mm, double mount_offset_mm);

private:
    uint8_t  read_reg(uint8_t reg);
    uint16_t read_reg16(uint8_t reg);
    void     write_reg(uint8_t reg, uint8_t v);
    void     write_reg16(uint8_t reg, uint16_t v);
    void     write_reg32(uint8_t reg, uint32_t v);
    void     read_block(uint8_t reg, uint8_t* out, std::size_t n);

    uint8_t  read_reg_tolerant(uint8_t reg, uint8_t on_nack);
    void     get_spad_info(uint8_t& count, bool& type_is_aperture);
    void     load_tuning_settings();
    void     single_ref_calibration(uint8_t vhv_init_byte);
    Reading  decode_result(const uint8_t b[12]);

    I2cBus&  bus_;
    uint8_t  addr_;
    Vl53l0xConfig cfg_;
    uint8_t  stop_variable_ = 0;      // read at init, replayed on every start_continuous
    uint64_t seq_     = 0;
    uint64_t reads_   = 0;
    uint64_t invalid_ = 0;
};

} // namespace ogma::hw
