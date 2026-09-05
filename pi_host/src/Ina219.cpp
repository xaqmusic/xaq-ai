#include "ogma/hw/Ina219.hpp"

#include <cmath>
#include <stdexcept>

namespace ogma::hw {

Ina219Config ina219_telemetry_config() {
    Ina219Config c;
    c.badc = Adc::Avg128;    // ~68 ms: averages the servo PWM ripple away
    c.sadc = Adc::Avg128;
    c.mode = Mode::ShuntBusContinuous;
    return c;
}

Ina219Config ina219_capture_config() {
    Ina219Config c;
    c.badc = Adc::Bits9;     // unused in shunt-only mode; smallest legal value
    c.sadc = Adc::Bits12;    // 532 us
    c.mode = Mode::ShuntContinuous;
    return c;
}

double Ina219::pga_full_scale_v(Pga pga) {
    switch (pga) {
        case Pga::Div1: return 0.040;
        case Pga::Div2: return 0.080;
        case Pga::Div4: return 0.160;
        case Pga::Div8: return 0.320;
    }
    return 0.320;
}

int16_t Ina219::pga_clip_counts(Pga pga) {
    // lround, not a cast: 0.320/10e-6 is 31999.999... in binary floating point.
    return static_cast<int16_t>(std::lround(pga_full_scale_v(pga) / SHUNT_LSB_V));
}

int Ina219::conversion_time_us(Adc adc) {
    switch (adc) {
        case Adc::Bits9:  return 84;
        case Adc::Bits10: return 148;
        case Adc::Bits11: return 276;
        case Adc::Bits12: return 532;
        case Adc::Avg2:   return 1060;
        case Adc::Avg4:   return 2130;
        case Adc::Avg8:   return 4260;
        case Adc::Avg16:  return 8510;
        case Adc::Avg32:  return 17020;
        case Adc::Avg64:  return 34050;
        case Adc::Avg128: return 68100;
    }
    return 532;
}

uint16_t Ina219::config_word(const Ina219Config& cfg) {
    uint16_t w = 0;
    if (cfg.brng == BusRange::V32) w |= 1u << 13;
    w |= static_cast<uint16_t>(static_cast<uint8_t>(cfg.pga)) << 11;
    w |= static_cast<uint16_t>(static_cast<uint8_t>(cfg.badc) & 0x0F) << 7;
    w |= static_cast<uint16_t>(static_cast<uint8_t>(cfg.sadc) & 0x0F) << 3;
    w |= static_cast<uint16_t>(static_cast<uint8_t>(cfg.mode) & 0x07);
    return w;
}

// The LSB that spans the PGA's full scale across the 15 magnitude bits, rounded
// UP onto a 1-2-5 step so the number a human reads is a round one.  Rounding up
// (never down) keeps full scale reachable, so the CURRENT register cannot
// saturate before the shunt ADC does.
double Ina219::choose_current_lsb(double r_shunt_ohm, Pga pga) {
    if (r_shunt_ohm <= 0.0) throw std::invalid_argument("Ina219: r_shunt must be > 0");
    const double ideal = (pga_full_scale_v(pga) / r_shunt_ohm) / 32767.0;
    const double decade = std::pow(10.0, std::floor(std::log10(ideal)));
    for (double step : {1.0, 2.0, 5.0})
        if (ideal <= step * decade) return step * decade;
    return 10.0 * decade;
}

uint16_t Ina219::calibration_word(double r_shunt_ohm, double current_lsb_a) {
    if (r_shunt_ohm <= 0.0 || current_lsb_a <= 0.0)
        throw std::invalid_argument("Ina219: calibration needs positive r_shunt and LSB");
    const double cal = std::trunc(CAL_CONST / (current_lsb_a * r_shunt_ohm));
    if (cal > 65535.0) throw std::invalid_argument("Ina219: calibration overflows 16 bits");
    // Bit 0 of the calibration register is a void bit and always reads 0.
    return static_cast<uint16_t>(static_cast<uint16_t>(cal) & 0xFFFEu);
}

Ina219::Ina219(I2cBus& bus, double r_shunt_ohm, uint8_t addr)
    : bus_(bus), addr_(addr), r_shunt_(r_shunt_ohm) {
    if (r_shunt_ohm <= 0.0) throw std::invalid_argument("Ina219: r_shunt must be > 0");
    current_lsb_ = choose_current_lsb(r_shunt_, cfg_.pga);
    cal_word_    = calibration_word(r_shunt_, current_lsb_);
}

void Ina219::write_reg(uint8_t reg, uint16_t value) {
    bus_.write(addr_, {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)});
}

uint16_t Ina219::read_reg(uint8_t reg) {
    bus_.write(addr_, {reg});
    const auto b = bus_.read_bytes(addr_, 2);
    return static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
}

void Ina219::reset() { write_reg(REG_CONFIG, 1u << 15); }

void Ina219::configure(const Ina219Config& cfg) {
    cfg_ = cfg;
    current_lsb_ = choose_current_lsb(r_shunt_, cfg_.pga);
    cal_word_    = calibration_word(r_shunt_, current_lsb_);
    write_reg(REG_CONFIG, config_word(cfg_));
    write_calibration();
}

void Ina219::write_calibration() { write_reg(REG_CALIB, cal_word_); }

void Ina219::set_r_shunt(double ohms) {
    if (ohms <= 0.0) throw std::invalid_argument("Ina219: r_shunt must be > 0");
    r_shunt_     = ohms;
    current_lsb_ = choose_current_lsb(r_shunt_, cfg_.pga);
    cal_word_    = calibration_word(r_shunt_, current_lsb_);
    write_calibration();
}

double Ina219::shunt_to_amps(int16_t raw) const { return raw * SHUNT_LSB_V / r_shunt_; }

int16_t Ina219::read_shunt_raw() { return static_cast<int16_t>(read_reg(REG_SHUNT)); }

Ina219::Sample Ina219::read() {
    Sample s;
    s.shunt_raw = read_shunt_raw();
    s.bus_raw   = read_reg(REG_BUS);
    s.shunt_v   = s.shunt_raw * SHUNT_LSB_V;
    s.bus_v     = (s.bus_raw >> 3) * BUS_LSB_V;
    s.current_a = shunt_to_amps(s.shunt_raw);
    s.conversion_ready = (s.bus_raw & 0x0002) != 0;
    s.overflow         = (s.bus_raw & 0x0001) != 0;
    // At full scale the reading is a floor on the truth, not the truth.  Say so
    // rather than reporting a clipped peak as a measured one.
    s.pga_clipped = std::abs(static_cast<int>(s.shunt_raw)) >= pga_clip_counts(cfg_.pga);
    return s;
}

double Ina219::chip_current_a() {
    return static_cast<int16_t>(read_reg(REG_CURRENT)) * current_lsb_;
}

double Ina219::chip_power_w() {
    return read_reg(REG_POWER) * 20.0 * current_lsb_;   // power LSB = 20 x current LSB
}

} // namespace ogma::hw
