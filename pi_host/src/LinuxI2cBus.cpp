#include "ogma/hw/I2cBus.hpp"

#include <stdexcept>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
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
