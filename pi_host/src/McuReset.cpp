#include "ogma/hw/McuReset.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

namespace ogma::hw {

McuReset::McuReset(const std::string& chip, unsigned line) {
    int cfd = ::open(chip.c_str(), O_RDONLY | O_CLOEXEC);
    if (cfd < 0) throw std::runtime_error("McuReset: open " + chip + ": " + std::strerror(errno));
    gpio_v2_line_request req{};
    req.offsets[0] = line; req.num_lines = 1;
    std::strncpy(req.consumer, "ogma_benchd MCURST", sizeof(req.consumer) - 1);
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    req.config.num_attrs = 1;                                   // initial value: HIGH (run)
    req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    req.config.attrs[0].attr.values = 1;
    req.config.attrs[0].mask = 1;
    if (::ioctl(cfd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) { int e = errno; ::close(cfd); throw std::runtime_error(std::string("McuReset: request line: ") + std::strerror(e)); }
    ::close(cfd);
    fd_ = req.fd;
}

McuReset::~McuReset() {
    if (fd_ >= 0) { set(1); ::close(fd_); }
}

void McuReset::set(int value) {
    gpio_v2_line_values v{}; v.mask = 1; v.bits = value ? 1 : 0;
    if (::ioctl(fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) < 0)
        throw std::runtime_error(std::string("McuReset: set: ") + std::strerror(errno));
}

void McuReset::reset(int settle_ms) {
    if (fd_ < 0) return;
    set(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
}

} // namespace ogma::hw
