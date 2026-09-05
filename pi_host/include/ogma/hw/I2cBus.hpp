#pragma once
// I2cBus — the one seam between the HAT protocol and the kernel.
// RobotHat speaks to this interface only, so the protocol is unit-testable
// against a FakeI2cBus (tests/test_hw.cpp) and the byte formation is proven
// before a real servo ever sees it.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ogma::hw {

class I2cBus {
public:
    virtual ~I2cBus() = default;
    // One write transaction to the selected device: bytes exactly as given
    // (register first, if any).  Throws std::runtime_error on bus failure.
    virtual void write(uint8_t addr, const std::vector<uint8_t>& bytes) = 0;
    // One 1-byte read transaction (no register phase).
    virtual uint8_t read_byte(uint8_t addr) = 0;
    // One n-byte read transaction.  NOT the same as n calls to read_byte():
    // each read_byte() is its own START..STOP, and a device with a pointer
    // register (the INA219) restarts at the MSB every time, so two 1-byte reads
    // return the high byte twice.  Multi-byte registers need this.
    virtual std::vector<uint8_t> read_bytes(uint8_t addr, std::size_t n) = 0;
};

// Linux userspace I2C via /dev/i2c-N + ioctl(I2C_SLAVE).  Plain read()/write()
// on the fd — the HAT's MCU has no SMBus PEC or repeated-start needs, and this
// matches byte-for-byte what SunFounder's smbus2 calls put on the wire.
class LinuxI2cBus : public I2cBus {
public:
    explicit LinuxI2cBus(const std::string& dev = "/dev/i2c-1");
    ~LinuxI2cBus() override;
    void write(uint8_t addr, const std::vector<uint8_t>& bytes) override;
    uint8_t read_byte(uint8_t addr) override;
    std::vector<uint8_t> read_bytes(uint8_t addr, std::size_t n) override;
private:
    void select(uint8_t addr);
    int fd_ = -1;
    int selected_ = -1;
};

} // namespace ogma::hw
