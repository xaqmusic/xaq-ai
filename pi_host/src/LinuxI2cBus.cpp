#include "ogma/hw/I2cBus.hpp"

#include <stdexcept>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/file.h>
#include <chrono>
#include <thread>

// The HAT's MCU occasionally NACKs a transaction (servo noise, a momentary sag):
// retry a few times before it becomes the caller's problem.
static constexpr int kRetries = 3;

namespace ogma::hw {

LinuxI2cBus::LinuxI2cBus(const std::string& dev) {
    fd_ = ::open(dev.c_str(), O_RDWR);
    if (fd_ < 0)
        throw std::runtime_error("LinuxI2cBus: open " + dev + ": " + std::strerror(errno));

    // ⚠ EXCLUSIVE CLAIM ON THE BUS, and it is here rather than in systemd on purpose.
    //
    // Several programs want /dev/i2c-1 — ogma_benchd (servos + bench sensors), hat_tool,
    // and now ogma_host when it is given --tof.  The kernel will happily let all of them
    // open it, and then their transactions INTERLEAVE.  That does not fail: a register
    // read is a write(pointer) followed by a read, and another process's write landing
    // between the two returns a different register's contents as a perfectly plausible
    // number.  The INA219 driver already carries a test pinning exactly this shape.
    //
    // The README's rule ("stop benchd before hat_tool") is that hazard managed by
    // convention.  A `Conflicts=` between the two units was the first attempt and was
    // WRONG: ogma_host only touches this bus with --tof, its unit does not pass it, and
    // mutual exclusion would have broken the operator's working arrangement of running
    // the brain and the bench together.  The contention is on the DEVICE, so the claim
    // belongs on the device — and this way it covers hat_tool and any future tool for
    // free, however it was started.
    //
    // LOCK_EX|LOCK_NB: whoever has it keeps it, and the loser fails LOUDLY at startup
    // naming the holder, instead of silently reading a plausible wrong number forever.
    // The lock is released by close() and by process death, so a crash cannot wedge it.
    if (::flock(fd_, LOCK_EX | LOCK_NB) < 0) {
        const int e = errno;
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error(
            "LinuxI2cBus: " + dev + " is already claimed by another process (" +
            std::strerror(e) + "). Only one owner at a time — the transactions of two "
            "would interleave and return plausible wrong values rather than failing. "
            "Stop the other holder (usually: sudo systemctl stop ogma-benchd).");
    }
}

LinuxI2cBus::~LinuxI2cBus() {
    if (fd_ >= 0) ::close(fd_);
}

void LinuxI2cBus::select(uint8_t addr) {
    if (selected_ == addr) return;
    if (::ioctl(fd_, I2C_SLAVE, addr) < 0)
        throw std::runtime_error("LinuxI2cBus: I2C_SLAVE " + std::to_string(addr) + ": " + std::strerror(errno));
    selected_ = addr;
}

void LinuxI2cBus::write(uint8_t addr, const std::vector<uint8_t>& bytes) {
    select(addr);
    for (int attempt = 0;; ++attempt) {
        ssize_t n = ::write(fd_, bytes.data(), bytes.size());
        if (n == static_cast<ssize_t>(bytes.size())) return;
        if (attempt >= kRetries)
            throw std::runtime_error(std::string("LinuxI2cBus: write: ") + std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

uint8_t LinuxI2cBus::read_byte(uint8_t addr) {
    select(addr);
    uint8_t b = 0;
    for (int attempt = 0;; ++attempt) {
        if (::read(fd_, &b, 1) == 1) return b;
        if (attempt >= kRetries)
            throw std::runtime_error(std::string("LinuxI2cBus: read: ") + std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

std::vector<uint8_t> LinuxI2cBus::read_bytes(uint8_t addr, std::size_t n) {
    select(addr);
    std::vector<uint8_t> out(n);
    for (int attempt = 0;; ++attempt) {
        // One transaction: a short read is a bus fault, not a partial result to
        // stitch together — a second read() would restart the device's pointer.
        if (::read(fd_, out.data(), n) == static_cast<ssize_t>(n)) return out;
        if (attempt >= kRetries)
            throw std::runtime_error(std::string("LinuxI2cBus: read_bytes: ") + std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

} // namespace ogma::hw
