#include "ogma/hw/Vl53l0x.hpp"

#include <cstring>
#include <stdexcept>
#include <thread>

namespace ogma::hw {
namespace {

// Register map — the subset the boot sequence and a ranging read actually touch.
constexpr uint8_t SYSRANGE_START                    = 0x00;
constexpr uint8_t SYSTEM_SEQUENCE_CONFIG            = 0x01;
constexpr uint8_t SYSTEM_INTERMEASUREMENT_PERIOD    = 0x04;
constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO      = 0x0A;
constexpr uint8_t SYSTEM_INTERRUPT_CLEAR            = 0x0B;
constexpr uint8_t RESULT_INTERRUPT_STATUS           = 0x13;
constexpr uint8_t RESULT_RANGE_STATUS               = 0x14;
constexpr uint8_t MSRC_CONFIG_TIMEOUT_MACROP        = 0x46;
constexpr uint8_t FINAL_RANGE_CONFIG_MIN_COUNT_RATE = 0x44;
constexpr uint8_t PRE_RANGE_CONFIG_VCSEL_PERIOD     = 0x50;
constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI= 0x51;
constexpr uint8_t MSRC_CONFIG_CONTROL               = 0x60;
constexpr uint8_t FINAL_RANGE_CONFIG_VCSEL_PERIOD   = 0x70;
constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
constexpr uint8_t GLOBAL_CONFIG_SPAD_ENABLES_REF_0  = 0xB0;
constexpr uint8_t GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
constexpr uint8_t DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
constexpr uint8_t DYNAMIC_SPAD_REF_EN_START_OFFSET  = 0x4F;
constexpr uint8_t OSC_CALIBRATE_VAL                 = 0xF8;
constexpr uint8_t SOFT_RESET_GO2_SOFT_RESET_N       = 0xBF;

// ST's default tuning settings, verbatim as (register, value) pairs.  This blob is
// published only as a list — there is no documented meaning for most of these
// registers, so it is transcribed rather than expressed, and kept in source order so
// it can be diffed against ST's API line for line.  Do not "simplify" it.
constexpr uint8_t TUNING[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00},
};

// setMeasurementTimingBudget's fixed per-step overheads, in us (ST's API).
constexpr uint32_t START_OVERHEAD_US       = 1910;
constexpr uint32_t END_OVERHEAD_US         = 960;
constexpr uint32_t MSRC_OVERHEAD_US        = 660;
constexpr uint32_t TCC_OVERHEAD_US         = 590;
constexpr uint32_t DSS_OVERHEAD_US         = 690;
constexpr uint32_t PRE_RANGE_OVERHEAD_US   = 660;
constexpr uint32_t FINAL_RANGE_OVERHEAD_US = 550;
constexpr uint32_t MIN_TIMING_BUDGET_US    = 20000;

struct StepEnables { bool tcc, msrc, dss, pre_range, final_range; };
struct StepTimeouts {
    uint8_t  pre_range_vcsel_pclks = 0, final_range_vcsel_pclks = 0;
    uint16_t msrc_dss_tcc_mclks = 0, pre_range_mclks = 0, final_range_mclks = 0;
    uint32_t msrc_dss_tcc_us = 0, pre_range_us = 0, final_range_us = 0;
};

} // namespace

// --------------------------------------------------------------------------- pure

uint32_t Vl53l0x::calc_macro_period_ns(uint8_t vcsel_period_pclks) {
    return ((2304u * vcsel_period_pclks * 1655u) + 500u) / 1000u;
}

uint16_t Vl53l0x::decode_timeout(uint16_t reg) {
    // (LSByte * 2^MSByte) + 1 — a byte mantissa with a byte exponent.
    return uint16_t((uint16_t(reg & 0x00FF) << uint16_t((reg & 0xFF00) >> 8)) + 1);
}

uint16_t Vl53l0x::encode_timeout(uint32_t mclks) {
    if (mclks == 0) return 0;
    uint32_t ls = mclks - 1;
    uint16_t ms = 0;
    while ((ls & 0xFFFFFF00u) > 0) { ls >>= 1; ++ms; }
    return uint16_t((ms << 8) | (ls & 0xFF));
}

uint32_t Vl53l0x::timeout_mclks_to_us(uint16_t mclks, uint8_t vcsel_period_pclks) {
    const uint32_t macro_ns = calc_macro_period_ns(vcsel_period_pclks);
    return ((uint32_t(mclks) * macro_ns) + 500u) / 1000u;
}

uint32_t Vl53l0x::timeout_us_to_mclks(uint32_t us, uint8_t vcsel_period_pclks) {
    const uint32_t macro_ns = calc_macro_period_ns(vcsel_period_pclks);
    return ((us * 1000u) + (macro_ns / 2u)) / macro_ns;
}

// ST's device-code -> PAL-status mapping (VL53L0X_get_pal_range_status), restricted to
// the branches that depend on the device code alone.  See the header for what that
// restriction costs.  Codes 0/5/7/12/13/14/15 are the API's NoneFlag set -- no
// measurement was produced at all, which is a different fact from one that was
// produced and failed a check, so it maps to NoUpdate and not to a failure reason.
Vl53l0x::Status Vl53l0x::decode_status(uint8_t device_status) {
    switch (device_status) {
        case 0: case 5: case 7: case 12: case 13: case 14: case 15:
                                  return Status::NoUpdate;
        case 1: case 2: case 3:   return Status::HardwareFail;
        case 6: case 9:           return Status::PhaseFail;
        case 8: case 10:          return Status::MinRangeFail;
        case 4:                   return Status::SignalFail;
        default:                  return Status::Valid;   // 11 and any unlisted code
    }
}

const char* Vl53l0x::status_name(Status s) {
    switch (s) {
        case Status::Valid:        return "valid";
        case Status::SigmaFail:    return "sigma";
        case Status::SignalFail:   return "signal";
        case Status::MinRangeFail: return "minrange";
        case Status::PhaseFail:    return "phase";
        case Status::HardwareFail: return "hardware";
        case Status::NoUpdate:     return "noupdate";
    }
    return "?";
}

double Vl53l0x::raw_to_clearance_m(uint16_t raw_mm, double mount_offset_mm) {
    const double mm = double(raw_mm) - mount_offset_mm;
    return mm > 0.0 ? mm / 1000.0 : 0.0;
}

// --------------------------------------------------------------------------- bus

Vl53l0x::Vl53l0x(I2cBus& bus, Vl53l0xConfig cfg, uint8_t addr)
    : bus_(bus), addr_(addr), cfg_(cfg) {
    if (cfg_.max_range_m <= 0.0) throw std::invalid_argument("Vl53l0x: max_range_m must be > 0");
}

void Vl53l0x::write_reg(uint8_t reg, uint8_t v) { bus_.write(addr_, {reg, v}); }

void Vl53l0x::write_reg16(uint8_t reg, uint16_t v) {
    bus_.write(addr_, {reg, uint8_t(v >> 8), uint8_t(v & 0xFF)});
}

void Vl53l0x::write_reg32(uint8_t reg, uint32_t v) {
    bus_.write(addr_, {reg, uint8_t(v >> 24), uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)});
}

uint8_t Vl53l0x::read_reg(uint8_t reg) {
    bus_.write(addr_, {reg});
    return bus_.read_byte(addr_);
}

uint16_t Vl53l0x::read_reg16(uint8_t reg) {
    bus_.write(addr_, {reg});
    const auto b = bus_.read_bytes(addr_, 2);
    return uint16_t((uint16_t(b[0]) << 8) | b[1]);
}

void Vl53l0x::read_block(uint8_t reg, uint8_t* out, std::size_t n) {
    bus_.write(addr_, {reg});
    const auto b = bus_.read_bytes(addr_, n);
    std::memcpy(out, b.data(), n);
}

bool Vl53l0x::model_id_ok() { return read_reg(REG_MODEL_ID) == MODEL_ID; }

// --------------------------------------------------------------------------- boot

// Which reference SPADs this particular die has, out of its NVM.  Every part is
// different, which is why this cannot be a constant: enabling a SPAD the die does not
// carry gives a sensor that ranges and is wrong.
void Vl53l0x::get_spad_info(uint8_t& count, bool& type_is_aperture) {
    write_reg(0x80, 0x01); write_reg(0xFF, 0x01); write_reg(0x00, 0x00);
    write_reg(0xFF, 0x06);
    write_reg(0x83, uint8_t(read_reg(0x83) | 0x04));
    write_reg(0xFF, 0x07); write_reg(0x81, 0x01);
    write_reg(0x80, 0x01); write_reg(0x94, 0x6B); write_reg(0x83, 0x00);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (read_reg(0x83) == 0x00) {
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("Vl53l0x: SPAD info readout timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    write_reg(0x83, 0x01);
    const uint8_t tmp = read_reg(0x92);
    count            = uint8_t(tmp & 0x7F);
    type_is_aperture = ((tmp >> 7) & 0x01) != 0;

    write_reg(0x81, 0x00); write_reg(0xFF, 0x06);
    write_reg(0x83, uint8_t(read_reg(0x83) & ~0x04));
    write_reg(0xFF, 0x01); write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00); write_reg(0x80, 0x00);
}

void Vl53l0x::load_tuning_settings() {
    for (const auto& rv : TUNING) write_reg(rv[0], rv[1]);
}

// VHV (vhv_init_byte 0x40) and phase (0x00).  Both must run once after the tuning
// blob, in that order, or the part ranges with an uncalibrated reference.
void Vl53l0x::single_ref_calibration(uint8_t vhv_init_byte) {
    write_reg(SYSRANGE_START, uint8_t(0x01 | vhv_init_byte));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while ((read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("Vl53l0x: reference calibration timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    write_reg(SYSRANGE_START, 0x00);
}

// A register read that survives the part not answering.  During a soft reset the
// device NACKs, which LinuxI2cBus quite correctly turns into an exception; here that
// is the expected answer, not a fault.
uint8_t Vl53l0x::read_reg_tolerant(uint8_t reg, uint8_t on_nack) {
    try { return read_reg(reg); } catch (const std::exception&) { return on_nack; }
}

// ⚠ WHY init() MUST RESET FIRST, measured 2026-09-07.  The "stop variable" at 0x91 is
// a per-die value that is only valid as read after a fresh boot -- and stop_continuous()
// WRITES ZERO to it, because that is what stopping means to this part.  So a second
// init() on a part this process (or a previous one) had already used reads a stop
// variable of 0x00 instead of the die's real value, replays that zero on every
// start_continuous(), and ranges wrongly: on the bench the part returned a correct
// 124 mm on its first-ever init and then nothing but out-of-range on every init after,
// while pointed at the same desk the whole time.  Metered on this die: 0x3c after a
// reset, 0x00 after a stop.  A reset makes init() genuinely idempotent, which is what
// a daemon that can be restarted requires.
void Vl53l0x::reset_device() {
    write_reg(SOFT_RESET_GO2_SOFT_RESET_N, 0x00);
    // Held in reset, the model ID reads back as 0 (or the part NACKs entirely).
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (read_reg_tolerant(REG_MODEL_ID, 0x00) != 0x00) {
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("Vl53l0x: part would not enter soft reset");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    write_reg(SOFT_RESET_GO2_SOFT_RESET_N, 0x01);
    deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (read_reg_tolerant(REG_MODEL_ID, 0x00) == 0x00) {
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("Vl53l0x: part did not come back from soft reset");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Vl53l0x::init() {
    if (!model_id_ok())
        throw std::runtime_error("Vl53l0x: model ID is not 0xEE — wrong part, or a dead bus");

    // Before anything else, and for the reason above: the stop variable read below is
    // only meaningful on a freshly booted die.
    reset_device();

    // 2v8 mode: the breakout regulates its own supply, and the HAT's I2C is 3.3 V.
    write_reg(0x89, uint8_t(read_reg(0x89) | 0x01));

    // The magic sequence that unlocks 0x91, where the "stop variable" lives.  It has
    // to be replayed on every start_continuous(), so it is captured once here.
    write_reg(0x88, 0x00);
    write_reg(0x80, 0x01); write_reg(0xFF, 0x01); write_reg(0x00, 0x00);
    stop_variable_ = read_reg(0x91);
    write_reg(0x00, 0x01); write_reg(0xFF, 0x00); write_reg(0x80, 0x00);

    // Disable the SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks: they are
    // redundant with the final-range check we set below, and they reject returns the
    // final check would have accepted.
    write_reg(MSRC_CONFIG_CONTROL, uint8_t(read_reg(MSRC_CONFIG_CONTROL) | 0x12));
    set_signal_rate_limit(cfg_.signal_rate_limit_mcps);
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    uint8_t spad_count = 0; bool aperture = false;
    get_spad_info(spad_count, aperture);

    uint8_t spad_map[6] = {0};
    read_block(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6);

    write_reg(0xFF, 0x01);
    write_reg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_reg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_reg(0xFF, 0x00);
    write_reg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    // Keep exactly spad_count enabled, starting at 12 for an aperture-type die and 0
    // otherwise; clear every other bit the NVM had set.
    const uint8_t first = aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; ++i) {
        if (i < first || enabled == spad_count) spad_map[i / 8] &= uint8_t(~(1u << (i % 8)));
        else if ((spad_map[i / 8] >> (i % 8)) & 0x01) ++enabled;
    }
    bus_.write(addr_, {GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map[0], spad_map[1],
                       spad_map[2], spad_map[3], spad_map[4], spad_map[5]});

    load_tuning_settings();

    // Interrupt on "new sample ready", active low, and clear whatever is latched.  We
    // never wire GPIO1; this is what makes RESULT_INTERRUPT_STATUS meaningful as a
    // polled data-ready flag.
    write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write_reg(0x84, uint8_t(read_reg(0x84) & ~0x10));
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    set_timing_budget_us(cfg_.timing_budget_us);

    write_reg(SYSTEM_SEQUENCE_CONFIG, 0x01);
    single_ref_calibration(0x40);          // VHV
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    single_ref_calibration(0x00);          // phase
    write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
}

void Vl53l0x::set_signal_rate_limit(float mcps) {
    if (mcps < 0.0f || mcps > 511.99f)
        throw std::invalid_argument("Vl53l0x: signal rate limit out of 9.7 fixed-point range");
    write_reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE, uint16_t(mcps * (1 << 7)));
    cfg_.signal_rate_limit_mcps = mcps;
}

// --------------------------------------------------------------------- timing budget

namespace {

StepEnables read_step_enables(uint8_t seq_cfg) {
    StepEnables e{};
    e.tcc         = (seq_cfg >> 4) & 0x1;
    e.dss         = (seq_cfg >> 3) & 0x1;
    e.msrc        = (seq_cfg >> 2) & 0x1;
    e.pre_range   = (seq_cfg >> 6) & 0x1;
    e.final_range = (seq_cfg >> 7) & 0x1;
    return e;
}

} // namespace

uint32_t Vl53l0x::timing_budget_us() {
    const StepEnables e = read_step_enables(read_reg(SYSTEM_SEQUENCE_CONFIG));
    StepTimeouts t{};
    t.pre_range_vcsel_pclks = decode_vcsel_period(read_reg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    t.msrc_dss_tcc_mclks    = uint16_t(read_reg(MSRC_CONFIG_TIMEOUT_MACROP) + 1);
    t.msrc_dss_tcc_us       = timeout_mclks_to_us(t.msrc_dss_tcc_mclks, t.pre_range_vcsel_pclks);
    t.pre_range_mclks       = decode_timeout(read_reg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    t.pre_range_us          = timeout_mclks_to_us(t.pre_range_mclks, t.pre_range_vcsel_pclks);
    t.final_range_vcsel_pclks = decode_vcsel_period(read_reg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    t.final_range_mclks     = decode_timeout(read_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    // The final-range timeout register INCLUDES the pre-range period when pre-range is
    // enabled, so it has to come back out before the arithmetic means anything.
    if (e.pre_range) t.final_range_mclks = uint16_t(t.final_range_mclks - t.pre_range_mclks);
    t.final_range_us = timeout_mclks_to_us(t.final_range_mclks, t.final_range_vcsel_pclks);

    uint32_t budget = START_OVERHEAD_US + END_OVERHEAD_US;
    if (e.tcc)             budget += t.msrc_dss_tcc_us + TCC_OVERHEAD_US;
    if (e.dss)             budget += 2 * (t.msrc_dss_tcc_us + DSS_OVERHEAD_US);
    else if (e.msrc)       budget += t.msrc_dss_tcc_us + MSRC_OVERHEAD_US;
    if (e.pre_range)       budget += t.pre_range_us + PRE_RANGE_OVERHEAD_US;
    if (e.final_range)     budget += t.final_range_us + FINAL_RANGE_OVERHEAD_US;
    return budget;
}

void Vl53l0x::set_timing_budget_us(uint32_t us) {
    if (us < MIN_TIMING_BUDGET_US)
        throw std::invalid_argument("Vl53l0x: timing budget below the part's 20 ms minimum");

    const StepEnables e = read_step_enables(read_reg(SYSTEM_SEQUENCE_CONFIG));
    StepTimeouts t{};
    t.pre_range_vcsel_pclks = decode_vcsel_period(read_reg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    t.msrc_dss_tcc_mclks    = uint16_t(read_reg(MSRC_CONFIG_TIMEOUT_MACROP) + 1);
    t.msrc_dss_tcc_us       = timeout_mclks_to_us(t.msrc_dss_tcc_mclks, t.pre_range_vcsel_pclks);
    t.pre_range_mclks       = decode_timeout(read_reg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    t.pre_range_us          = timeout_mclks_to_us(t.pre_range_mclks, t.pre_range_vcsel_pclks);
    t.final_range_vcsel_pclks = decode_vcsel_period(read_reg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));

    // Everything except the final range is fixed by the sequence config; the budget is
    // spent on what is left over, which is why a budget below the overheads is refused
    // rather than silently truncated.
    uint32_t used = START_OVERHEAD_US + END_OVERHEAD_US;
    if (e.tcc)       used += t.msrc_dss_tcc_us + TCC_OVERHEAD_US;
    if (e.dss)       used += 2 * (t.msrc_dss_tcc_us + DSS_OVERHEAD_US);
    else if (e.msrc) used += t.msrc_dss_tcc_us + MSRC_OVERHEAD_US;
    if (e.pre_range) used += t.pre_range_us + PRE_RANGE_OVERHEAD_US;
    if (!e.final_range) return;                 // nothing to spend it on
    used += FINAL_RANGE_OVERHEAD_US;
    if (used > us)
        throw std::invalid_argument("Vl53l0x: timing budget too small for the enabled steps");

    uint32_t final_mclks = timeout_us_to_mclks(us - used, t.final_range_vcsel_pclks);
    if (e.pre_range) final_mclks += t.pre_range_mclks;
    write_reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(final_mclks));
    cfg_.timing_budget_us = us;
}

// ------------------------------------------------------------------------- ranging

void Vl53l0x::start_continuous(uint32_t period_ms) {
    write_reg(0x80, 0x01); write_reg(0xFF, 0x01); write_reg(0x00, 0x00);
    write_reg(0x91, stop_variable_);
    write_reg(0x00, 0x01); write_reg(0xFF, 0x00); write_reg(0x80, 0x00);

    if (period_ms != 0) {
        // The inter-measurement period is counted in the part's own oscillator ticks,
        // and that oscillator is trimmed per die — so the millisecond figure has to be
        // scaled by the die's calibration value or the actual rate is off by percent.
        const uint16_t osc = read_reg16(OSC_CALIBRATE_VAL);
        write_reg32(SYSTEM_INTERMEASUREMENT_PERIOD, osc != 0 ? period_ms * osc : period_ms);
        write_reg(SYSRANGE_START, 0x04);        // timed
    } else {
        write_reg(SYSRANGE_START, 0x02);        // back-to-back
    }
}

void Vl53l0x::stop_continuous() {
    write_reg(SYSRANGE_START, 0x01);            // single-shot mode = stop continuous
    write_reg(0xFF, 0x01); write_reg(0x00, 0x00);
    write_reg(0x91, 0x00);
    write_reg(0x00, 0x01); write_reg(0xFF, 0x00);
}

bool Vl53l0x::data_ready() { return (read_reg(RESULT_INTERRUPT_STATUS) & 0x07) != 0; }

// The 12 bytes at 0x14 are one measurement: status, effective SPAD count, the signal
// and ambient rates, and the range.  Read as ONE block, because reading the range
// separately from the status that qualifies it invites pairing a good status with the
// next measurement's distance.
Vl53l0x::Reading Vl53l0x::decode_result(const uint8_t b[12]) {
    Reading r;
    r.device_status = uint8_t((b[0] & 0x78) >> 3);
    r.status        = decode_status(r.device_status);
    r.spads         = fixpoint88_to_count(uint16_t((uint16_t(b[2]) << 8) | b[3]));
    r.signal_mcps   = fixpoint97_to_mcps(uint16_t((uint16_t(b[6]) << 8) | b[7]));
    r.ambient_mcps  = fixpoint97_to_mcps(uint16_t((uint16_t(b[8]) << 8) | b[9]));
    r.raw_mm        = uint16_t((uint16_t(b[10]) << 8) | b[11]);
    r.valid         = r.status == Status::Valid;
    r.distance_m    = r.valid ? raw_to_clearance_m(r.raw_mm, cfg_.mount_offset_mm)
                              : cfg_.max_range_m;
    r.seq           = ++seq_;
    ++reads_;
    if (!r.valid) ++invalid_;
    return r;
}

bool Vl53l0x::read_ready(Reading& out) {
    if (!data_ready()) return false;
    uint8_t b[12];
    read_block(RESULT_RANGE_STATUS, b, 12);
    write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    out = decode_result(b);
    return true;
}

Vl53l0x::Reading Vl53l0x::read_blocking(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    Reading r;
    while (!read_ready(r)) {
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("Vl53l0x: timed out waiting for a measurement");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return r;
}

} // namespace ogma::hw
