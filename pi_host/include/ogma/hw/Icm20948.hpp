#pragma once
// Icm20948 -- InvenSense ICM-20948 6-axis IMU on SPI, plus the attitude filter that
// rides on it.  Fitted and calibrated 2026-09-10; the measurements behind every
// constant here are in BOM sec 4 (docs/operational/picrawler_sensor_wiring_and_bom.md).
//
// SPI, NOT I2C, and the reason is jitter.  The HAT shares its I2C bus with all twelve
// servo writes, and host-side jitter integrates directly into dead-reckoned yaw.  SPI
// removes that at the source instead of filtering it afterwards.  CE0, <= 7 MHz (the
// datasheet register limit -- measured clean at 10 MHz on the bench, which is not a
// licence to exceed it).  The register map is BANKED, four banks via REG_BANK_SEL
// (0x7F), unlike the flat MPU-6050/9150 map.
//
// THE MAGNETOMETER IS DARK BY DECISION.  The internal I2C master is never enabled, so
// the AK09916 never speaks.  Accepted trade: heading is dead-reckoned from the gyro.
// The upside is that this part has NO magnetic-hygiene constraint -- the usual dominant
// IMU placement rule is void here, which is why it can sit next to the servo leads.
//
// WHAT IT PUBLISHES, AND WHY IT IS MORE THAN SIX NUMBERS.  On a real robot there is no
// ground-truth attitude to check a filter against, so the honest health signal is the
// DISAGREEMENT between the accelerometer-only gravity estimate and the fused one --
// both computable on-robot.  That scalar is the point of this class; raw accel and gyro
// alone would throw away the one reading that says whether the filter is working.
//
// ⚠ THE AXIS MAP IS MEASURED, NOT ASSUMED (BOM sec 4.1).  Lifting the body and watching
// which axis moves gave: chip +X = body LEFT, +Y = body AFT, +Z = body UP.  The remap to
// the sim body frame (+X left, +Y up, +Z forward) is therefore
//
//     v_sim = ( +v_x , +v_z , -v_y )
//
// and it applies to BOTH accel and gyro -- but only because its determinant is +1.
// Angular velocity is a pseudovector: had the frames differed by a reflection the gyro
// would need an extra global sign flip the accelerometer does not, and that failure is
// silent.  It is a proper rotation here.  Do not "simplify" the remap without redoing
// that check.
//
// ⚠ level_ref IS CALIBRATION DATA, on the same contract as the ToF's mount_offset_mm
// and the INA219's r_shunt.  It is the chip-frame direction of body-up measured with
// the belly flat on a level floor (BOM sec 4.2: five placements over four verified 180
// deg rotations, 0.05 deg repeatability, floor slope 0.49 deg subtracted out).  It
// lumps mount tilt together with accelerometer bias -- no rotation about vertical can
// separate them, and inversion is impossible on this robot because the HAT and its
// wiring are on top.  Stable to 0.07 deg across seven resets and a reboot, which is why
// it is stored rather than re-estimated.  ⚠ Untested cold: every measurement behind it
// was taken at 34 C.
//
// ⚠ THE GYRO BIAS IS NOT STORED, AND MUST NOT BE.  It is re-estimated at run time from
// quasi-static windows (prohibition sec 5: adapt from the system's own dynamics, never
// tune a constant).  Measured turn-on/thermal behaviour: 0.02-0.036 dps of drift within
// a session at constant temperature, but 0.15 dps across 5 C -- so temperature moves it
// far more than a reset does, and a stored constant would be wrong by more than the
// signal within one warm-up.
//
// WHERE THIS CODE BELONGS LATER.  The complementary filter is duplicated from the sim's
// _imu_substep (picrawler_body.gd:5801) and is bit-comparable to it by design.  The port
// doc's Order step (a) puts the shared version at cpp_core/include/ogma/body/
// ImuAttitude.hpp, used by BOTH sim and host.  It lives here for now because ogma_hw is
// deliberately independent of ogma_core (pi_host/CMakeLists.txt) so the driver builds in
// seconds.  When step (a) happens, move it -- do not fork it.

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace ogma::hw {

struct Icm20948Config {
    std::string device   = "/dev/spidev0.0";
    // <= 7 MHz: the datasheet's register limit.  See the header note.
    uint32_t    speed_hz = 4000000;

    // --- calibration (BOM sec 4.2) -------------------------------------------------
    // Chip-frame body-up with the belly flat on a level floor.  Mount tilt + accel bias,
    // unsplit.  Defaults to the 2026-09-10 fit; override from calib JSON when that lands.
    std::array<float, 3> level_ref = {-0.03493f, -0.00558f, +0.99937f};

    // --- complementary filter (mirrors the sim's IMU_ACC_* constants) ---------------
    // The accelerometer only indicates "down" when the body is quasi-static; during a
    // footfall it is measuring the impact.  Weight its trust by how close |a| is to g
    // rather than accepting or rejecting outright -- a hard gate starved the filter.
    float acc_trust     = 0.02f;
    float acc_gate_frac = 0.5f;

    // --- gyro bias estimator ---------------------------------------------------------
    // A window counts as quasi-static when every axis is below this, in dps.  Set from
    // the measured noise floor (~0.15 dps sd) with margin, not from taste.
    float bias_still_dps = 1.5f;
    float bias_alpha     = 0.002f;   // EMA per accepted sample (~8 s at 225 Hz)
};

// One conditioned reading.  Everything here is computable ON-ROBOT -- there is
// deliberately no ground-truth field, because the hardware has none.
struct ImuSample {
    bool  ok           = false;
    // Raw-ish, chip frame, after bias correction on the gyro.
    std::array<float, 3> accel_g   = {0, 0, 0};
    std::array<float, 3> gyro_dps  = {0, 0, 0};
    // Body frame (sim axes: +X left, +Y up, +Z forward), after the sec 4.1 remap.
    std::array<float, 3> accel_body = {0, 0, 0};
    std::array<float, 3> gyro_body  = {0, 0, 0};
    std::array<float, 3> up_accel   = {0, 1, 0};   // accelerometer-only gravity-up
    std::array<float, 3> up_fused   = {0, 1, 0};   // complementary-filter gravity-up
    float a_norm_g     = 0.0f;      // |a| in g -- 1.0004 measured at rest
    float trust        = 0.0f;      // the correction gain actually applied this step
    float disagree_deg = 0.0f;      // THE health signal (see header)
    float temp_c       = 0.0f;
    // Bias estimator state, published so a consumer can see it converge rather than
    // trusting that it did.
    std::array<float, 3> gyro_bias_dps = {0, 0, 0};
    bool  bias_valid   = false;
    int   bias_samples = 0;
    float dt_s         = 0.0f;
};

class Icm20948 {
public:
    explicit Icm20948(Icm20948Config cfg = {});
    ~Icm20948();
    Icm20948(const Icm20948&) = delete;
    Icm20948& operator=(const Icm20948&) = delete;

    // Opens spidev, resets, wakes and configures the part.  false + *err on failure;
    // the WHO_AM_I check (0xEA) is part of this, so a miswired bus fails here, loudly,
    // rather than producing plausible numbers later.
    bool begin(std::string* err);

    // One burst read + one filter step.  dt comes from the steady clock, so calling it
    // irregularly degrades the integration rather than corrupting it.
    bool sample(ImuSample& out);

    bool     healthy() const { return fd_ >= 0 && ok_; }
    int      errors()  const { return errors_; }
    uint8_t  who_am_i() const { return who_; }

private:
    bool     xfer(const uint8_t* tx, uint8_t* rx, size_t n);
    bool     wr(uint8_t reg, uint8_t val);
    bool     rd(uint8_t reg, uint8_t* buf, size_t n);
    bool     bank(uint8_t b);

    Icm20948Config cfg_;
    int      fd_     = -1;
    bool     ok_     = false;
    uint8_t  who_    = 0;
    int      errors_ = 0;
    int8_t   cur_bank_ = -1;

    std::array<float, 3> up_    = {0, 1, 0};
    std::array<float, 3> bias_  = {0, 0, 0};
    int      bias_n_ = 0;
    std::chrono::steady_clock::time_point last_{};
    bool     have_last_ = false;
};

}  // namespace ogma::hw
