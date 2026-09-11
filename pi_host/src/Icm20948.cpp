#include "ogma/hw/Icm20948.hpp"

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <thread>

namespace ogma::hw {
namespace {

// Bank 0
constexpr uint8_t REG_WHO_AM_I   = 0x00;
constexpr uint8_t REG_USER_CTRL  = 0x03;
constexpr uint8_t REG_PWR_MGMT_1 = 0x06;
constexpr uint8_t REG_PWR_MGMT_2 = 0x07;
constexpr uint8_t REG_ACCEL_XOUT = 0x2D;   // 14 bytes: accel[3], gyro[3], temp
// Bank 2
constexpr uint8_t REG_GYRO_SMPLRT_DIV  = 0x00;
constexpr uint8_t REG_GYRO_CONFIG_1    = 0x01;
constexpr uint8_t REG_ACCEL_SMPLRT_1   = 0x10;
constexpr uint8_t REG_ACCEL_SMPLRT_2   = 0x11;
constexpr uint8_t REG_ACCEL_CONFIG     = 0x14;
// Any bank
constexpr uint8_t REG_BANK_SEL   = 0x7F;

constexpr uint8_t WHO_EXPECT     = 0xEA;
// +-4 g and +-500 dps, DLPF cfg 1 on both, ODR 225 Hz.  These set the LSB scales below;
// changing one without the other silently rescales every reading.
constexpr float   ACCEL_LSB_PER_G   = 8192.0f;
constexpr float   GYRO_LSB_PER_DPS  = 65.5f;

inline void  vnorm(std::array<float,3>& v) {
    float n = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n > 1e-9f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}
inline float vdot(const std::array<float,3>& a, const std::array<float,3>& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// Rotate v about a unit axis k by angle rad (Rodrigues).  Used for the EXACT gyro
// propagation below -- the first-order `v -= w x v dt` form leaves O((w dt)^2) error per
// step, which integrates to radians over a run.
std::array<float,3> rotate(const std::array<float,3>& v,
                           const std::array<float,3>& k, float rad) {
    const float c = std::cos(rad), s = std::sin(rad);
    const std::array<float,3> kxv = {k[1]*v[2] - k[2]*v[1],
                                     k[2]*v[0] - k[0]*v[2],
                                     k[0]*v[1] - k[1]*v[0]};
    const float kd = vdot(k, v) * (1.0f - c);
    return {v[0]*c + kxv[0]*s + k[0]*kd,
            v[1]*c + kxv[1]*s + k[1]*kd,
            v[2]*c + kxv[2]*s + k[2]*kd};
}

// The measured level reference (chip frame) rotated onto chip +Z, i.e. the correction
// that makes a level body read (0,0,1).
//
// ⚠ APPLIED TO THE ACCELEROMETER ONLY, ON PURPOSE.  level_ref lumps mount tilt together
// with accelerometer bias and no yaw rotation can separate them (BOM sec 4.3).  A mount
// tilt would misalign the gyro too; an accel bias would not.  Since the constant is
// FITTED FROM ACCEL DATA, it is applied to accel data.  If the split is ever measured and
// the tilt turns out to dominate, the gyro wants the same rotation -- that is a
// deliberate later change, not an oversight here.  At 2.03 deg the difference is small
// either way, which is exactly why it must be written down rather than guessed at.
std::array<float,3> level_correct(const std::array<float,3>& a,
                                  const std::array<float,3>& ref) {
    std::array<float,3> u = ref; vnorm(u);
    const std::array<float,3> z = {0.0f, 0.0f, 1.0f};
    std::array<float,3> axis = {u[1]*z[2] - u[2]*z[1],
                                u[2]*z[0] - u[0]*z[2],
                                u[0]*z[1] - u[1]*z[0]};
    const float s = std::sqrt(vdot(axis, axis));
    if (s < 1e-7f) return a;                    // already aligned (or antiparallel)
    axis[0] /= s; axis[1] /= s; axis[2] /= s;
    return rotate(a, axis, std::asin(s < 1.0f ? s : 1.0f));
}

// Chip frame -> sim body frame (+X left, +Y up, +Z forward).  MEASURED, BOM sec 4.1.
// Determinant +1, so the SAME map is valid for the gyro pseudovector; see the header.
inline std::array<float,3> to_body(const std::array<float,3>& v) {
    return { +v[0], +v[2], -v[1] };
}

}  // namespace

Icm20948::Icm20948(Icm20948Config cfg) : cfg_(std::move(cfg)) {}
Icm20948::~Icm20948() { if (fd_ >= 0) ::close(fd_); }

bool Icm20948::xfer(const uint8_t* tx, uint8_t* rx, size_t n) {
    if (fd_ < 0) return false;
    spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<uint64_t>(tx);
    tr.rx_buf = reinterpret_cast<uint64_t>(rx);
    tr.len    = static_cast<uint32_t>(n);
    tr.speed_hz = cfg_.speed_hz;
    tr.bits_per_word = 8;
    if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 1) { ++errors_; return false; }
    return true;
}

bool Icm20948::wr(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val}, rx[2] = {0, 0};
    return xfer(tx, rx, 2);
}

bool Icm20948::rd(uint8_t reg, uint8_t* buf, size_t n) {
    uint8_t tx[16] = {0}, rx[16] = {0};
    if (n + 1 > sizeof(tx)) return false;
    tx[0] = static_cast<uint8_t>(reg | 0x80);
    if (!xfer(tx, rx, n + 1)) return false;
    std::memcpy(buf, rx + 1, n);
    return true;
}

bool Icm20948::bank(uint8_t b) {
    if (cur_bank_ == static_cast<int8_t>(b)) return true;
    if (!wr(REG_BANK_SEL, static_cast<uint8_t>((b & 3) << 4))) return false;
    cur_bank_ = static_cast<int8_t>(b);
    return true;
}

bool Icm20948::begin(std::string* err) {
    auto fail = [&](const char* m) { if (err) *err = m; ok_ = false; return false; };
    fd_ = ::open(cfg_.device.c_str(), O_RDWR);
    if (fd_ < 0) return fail("open spidev failed (is dtparam=spi=on set, and are we in group spi?)");
    uint8_t mode = SPI_MODE_0, bits = 8;
    uint32_t hz = cfg_.speed_hz;
    if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0)          return fail("SPI_IOC_WR_MODE failed");
    if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return fail("SPI_IOC_WR_BITS_PER_WORD failed");
    if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0)    return fail("SPI_IOC_WR_MAX_SPEED_HZ failed");

    using namespace std::chrono_literals;
    cur_bank_ = -1;
    if (!bank(0)) return fail("bank select failed (no response on CE0)");
    wr(REG_PWR_MGMT_1, 0x80);                    // DEVICE_RESET
    std::this_thread::sleep_for(200ms);
    cur_bank_ = -1;
    if (!bank(0)) return fail("bank select failed after reset");
    wr(REG_PWR_MGMT_1, 0x01);                    // wake, CLKSEL = auto
    std::this_thread::sleep_for(50ms);
    wr(REG_PWR_MGMT_2, 0x00);                    // accel + gyro, all axes on
    wr(REG_USER_CTRL,  0x10);                    // I2C_IF_DIS: lock SPI, keep the mag dark
    std::this_thread::sleep_for(20ms);

    if (!bank(2)) return fail("bank 2 select failed");
    wr(REG_GYRO_SMPLRT_DIV, 0x04);               // 1125/(1+4) = 225 Hz
    wr(REG_GYRO_CONFIG_1,   0x0B);               // DLPF cfg 1, +-500 dps, FCHOICE = 1
    wr(REG_ACCEL_SMPLRT_1,  0x00);
    wr(REG_ACCEL_SMPLRT_2,  0x04);               // 225 Hz
    wr(REG_ACCEL_CONFIG,    0x0B);               // DLPF cfg 1, +-4 g, FCHOICE = 1
    if (!bank(0)) return fail("bank 0 select failed after config");
    std::this_thread::sleep_for(100ms);

    if (!rd(REG_WHO_AM_I, &who_, 1)) return fail("WHO_AM_I read failed");
    if (who_ != WHO_EXPECT) {
        if (err) *err = "WHO_AM_I = 0x" + std::to_string(int(who_)) + ", expected 0xEA";
        ok_ = false; return false;
    }
    ok_ = true; have_last_ = false; bias_n_ = 0;
    bias_ = {0, 0, 0}; up_ = {0, 1, 0};
    return true;
}

bool Icm20948::sample(ImuSample& out) {
    out = ImuSample{};
    if (!ok_ || fd_ < 0) return false;
    uint8_t b[14] = {0};
    if (!bank(0) || !rd(REG_ACCEL_XOUT, b, 14)) { out.ok = false; return false; }

    auto s16 = [&](int i) { return static_cast<int16_t>((b[i] << 8) | b[i + 1]); };
    const std::array<float,3> araw = {s16(0)  / ACCEL_LSB_PER_G,
                                      s16(2)  / ACCEL_LSB_PER_G,
                                      s16(4)  / ACCEL_LSB_PER_G};
    const std::array<float,3> graw = {s16(6)  / GYRO_LSB_PER_DPS,
                                      s16(8)  / GYRO_LSB_PER_DPS,
                                      s16(10) / GYRO_LSB_PER_DPS};
    out.temp_c = s16(12) / 333.87f + 21.0f;

    // ---- dt from the steady clock ---------------------------------------------------
    const auto now = std::chrono::steady_clock::now();
    float dt = 1.0f / 225.0f;
    if (have_last_) dt = std::chrono::duration<float>(now - last_).count();
    last_ = now; have_last_ = true;
    if (dt <= 0.0f || dt > 0.5f) dt = 1.0f / 225.0f;   // a scheduling hiccup must not integrate
    out.dt_s = dt;

    const float amag = std::sqrt(araw[0]*araw[0] + araw[1]*araw[1] + araw[2]*araw[2]);
    out.a_norm_g = amag;

    // ---- gyro bias: quasi-static windows only ---------------------------------------
    // ⚠ STARTUP ASSUMES THE ROBOT IS STILL.  The first seed window is accepted without a
    // motion gate, because the gate needs a bias estimate to be meaningful and the raw
    // bias (up to ~2.4 dps on one axis, BOM sec 4.2) exceeds any sane stillness
    // threshold.  Seeding while the robot moves poisons heading for the whole run, so
    // begin() is a stationary operation -- same contract as any gyro on any robot.
    constexpr int SEED_N = 200;
    const bool accel_still = std::fabs(amag - 1.0f) < 0.08f;
    if (bias_n_ < SEED_N) {
        if (accel_still) {
            const float w = 1.0f / float(bias_n_ + 1);
            for (int i = 0; i < 3; ++i) bias_[i] += (graw[i] - bias_[i]) * w;
            ++bias_n_;
        }
    } else {
        bool still = accel_still;
        for (int i = 0; i < 3 && still; ++i)
            still = std::fabs(graw[i] - bias_[i]) < cfg_.bias_still_dps;
        if (still) {
            for (int i = 0; i < 3; ++i) bias_[i] += (graw[i] - bias_[i]) * cfg_.bias_alpha;
            ++bias_n_;
        }
    }
    out.gyro_bias_dps = bias_;
    out.bias_valid    = bias_n_ >= SEED_N;
    out.bias_samples  = bias_n_;

    const std::array<float,3> gcor = {graw[0] - bias_[0], graw[1] - bias_[1], graw[2] - bias_[2]};
    out.accel_g  = araw;
    out.gyro_dps = gcor;

    // ---- level correction (accel only -- see level_correct) + body-frame remap -------
    const std::array<float,3> alvl = level_correct(araw, cfg_.level_ref);
    out.accel_body = to_body(alvl);
    out.gyro_body  = to_body(gcor);

    std::array<float,3> up_acc = out.accel_body;
    vnorm(up_acc);
    if (amag > 1e-4f) out.up_accel = up_acc;

    // ---- complementary filter (mirrors picrawler_body.gd:_imu_substep) --------------
    if (std::sqrt(vdot(up_, up_)) < 0.5f) up_ = up_acc;
    // Gyro propagation: a WORLD-fixed direction seen from the body rotates by -w dt.
    const float wx = out.gyro_body[0] * float(M_PI) / 180.0f;
    const float wy = out.gyro_body[1] * float(M_PI) / 180.0f;
    const float wz = out.gyro_body[2] * float(M_PI) / 180.0f;
    const float wmag = std::sqrt(wx*wx + wy*wy + wz*wz);
    if (wmag > 1e-6f) {
        const std::array<float,3> k = {wx / wmag, wy / wmag, wz / wmag};
        up_ = rotate(up_, k, -wmag * dt);
        vnorm(up_);
    }
    // Adaptive-gain correction: trust the accelerometer in proportion to how close |a|
    // is to g.  A hard accept/reject gate starved this filter in sim.
    const float acc_dev = std::fabs(amag - 1.0f);
    float trust = cfg_.acc_trust * (1.0f - acc_dev / cfg_.acc_gate_frac);
    if (trust < 0.0f) trust = 0.0f;
    if (trust > cfg_.acc_trust) trust = cfg_.acc_trust;
    if (trust > 0.0f && amag > 1e-4f) {
        for (int i = 0; i < 3; ++i) up_[i] = up_[i] * (1.0f - trust) + up_acc[i] * trust;
        vnorm(up_);
    }
    out.trust    = trust;
    out.up_fused = up_;

    // THE health signal: with no ground truth on hardware, accel-vs-fused disagreement is
    // what says whether the filter is working.
    float d = vdot(up_, up_acc);
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    out.disagree_deg = (amag > 1e-4f) ? std::acos(d) * 180.0f / float(M_PI) : 0.0f;

    out.ok = true;
    return true;
}

}  // namespace ogma::hw
