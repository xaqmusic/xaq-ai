#include "ogma/hw/I2cBus.hpp"

#include <stdexcept>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

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
    ssize_t n = ::write(fd_, bytes.data(), bytes.size());
    if (n != static_cast<ssize_t>(bytes.size()))
        throw std::runtime_error(std::string("LinuxI2cBus: write: ") + std::strerror(errno));
}

uint8_t LinuxI2cBus::read_byte(uint8_t addr) {
    select(addr);
    uint8_t b = 0;
    if (::read(fd_, &b, 1) != 1)
        throw std::runtime_error(std::string("LinuxI2cBus: read: ") + std::strerror(errno));
    return b;
}

} // namespace ogma::hw
