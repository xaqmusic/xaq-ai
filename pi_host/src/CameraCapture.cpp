#include "ogma/hw/CameraCapture.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace ogma::hw {

CameraCapture::CameraCapture(Config cfg) : cfg_(std::move(cfg)) {}
CameraCapture::~CameraCapture() { stop(); }

void CameraCapture::reduce(const uint8_t* y, int w, int h, int out, std::vector<uint8_t>& dst) {
    dst.assign(size_t(out) * size_t(out), 0);
    if (!y || w <= 0 || h <= 0 || out <= 0) return;
    // Centre-crop to square first so the scene is not anamorphically squashed; the
    // JL projection is frozen and random, but a geometry the operator cannot reason
    // about is a bad thing to hand a vocabulary.
    const int side = std::min(w, h);
    const int x0   = (w - side) / 2;
    const int y0   = (h - side) / 2;
    for (int oy = 0; oy < out; ++oy) {
        const int sy0 = y0 + (oy * side) / out;
        const int sy1 = std::max(sy0 + 1, y0 + ((oy + 1) * side) / out);
        for (int ox = 0; ox < out; ++ox) {
            const int sx0 = x0 + (ox * side) / out;
            const int sx1 = std::max(sx0 + 1, x0 + ((ox + 1) * side) / out);
            uint32_t sum = 0, n = 0;
            for (int sy = sy0; sy < sy1 && sy < h; ++sy)
                for (int sx = sx0; sx < sx1 && sx < w; ++sx) { sum += y[size_t(sy) * size_t(w) + size_t(sx)]; ++n; }
            dst[size_t(oy) * size_t(out) + size_t(ox)] = uint8_t(n ? sum / n : 0);
        }
    }
}

bool CameraCapture::start() {
    if (running_) return true;
    int fds[2];
    if (::pipe(fds) != 0) { err_ = std::string("pipe: ") + std::strerror(errno); return false; }

    const pid_t pid = ::fork();
    if (pid < 0) { err_ = std::string("fork: ") + std::strerror(errno); ::close(fds[0]); ::close(fds[1]); return false; }
    if (pid == 0) {                                   // child: rpicam-vid -> stdout -> pipe
        ::close(fds[0]);
        ::dup2(fds[1], STDOUT_FILENO);
        ::close(fds[1]);
        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) { ::dup2(devnull, STDERR_FILENO); ::close(devnull); }
        // Close everything above stdio before exec.  fork() hands the child every open
        // descriptor, and the brain's are LISTENING SOCKETS (control 7400, diag 7401):
        // an inherited copy keeps those ports bound for as long as rpicam-vid lives, so
        // a host that died would leave a camera subprocess squatting on the inspector's
        // ports.  Observed in `ss -ltnp` as rpicam-vid holding fd 10 on 7400.
        const long maxfd = ::sysconf(_SC_OPEN_MAX);
        for (int fd = STDERR_FILENO + 1; fd < int(maxfd > 0 ? maxfd : 4096); ++fd) ::close(fd);
        char sw[16], sh[16], sf[16];
        std::snprintf(sw, sizeof sw, "%d", cfg_.src_width);
        std::snprintf(sh, sizeof sh, "%d", cfg_.src_height);
        std::snprintf(sf, sizeof sf, "%d", cfg_.fps);
        ::execlp(cfg_.binary.c_str(), cfg_.binary.c_str(),
                 "-t", "0", "--codec", "yuv420", "--nopreview",
                 "--width", sw, "--height", sh, "--framerate", sf,
                 "-o", "-", (char*)nullptr);
        ::_exit(127);
    }
    ::close(fds[1]);
    pipe_fd_ = fds[0];
    child_   = pid;
    running_ = true;
    err_.clear();
    th_ = std::thread(&CameraCapture::run, this);
    return true;
}

void CameraCapture::stop() {
    const bool was = running_.exchange(false);
    if (!was && !th_.joinable() && child_ < 0) return;
    if (child_ > 0) ::kill(child_, SIGTERM);          // unblocks the reader with EOF
    if (th_.joinable()) th_.join();
    if (pipe_fd_ >= 0) { ::close(pipe_fd_); pipe_fd_ = -1; }
    if (child_ > 0) { int st = 0; ::waitpid(child_, &st, 0); child_ = -1; }
}

void CameraCapture::run() {
    const size_t ysize     = size_t(cfg_.src_width) * size_t(cfg_.src_height);
    const size_t framesize = ysize * 3 / 2;           // YUV420: Y plane then quarter-size U, V
    std::vector<uint8_t> raw(framesize);
    std::vector<uint8_t> small;

    while (running_) {
        size_t got = 0;
        while (got < framesize && running_) {
            const ssize_t n = ::read(pipe_fd_, raw.data() + got, framesize - got);
            if (n > 0) { got += size_t(n); continue; }
            // EOF during our own stop() is the SIGTERM we sent, not a fault.  A false
            // "camera died" on every clean exit teaches the operator to ignore the real one.
            if (n == 0) {
                if (running_) err_ = "rpicam-vid closed the pipe (camera stack gone?)";
                running_ = false;
                return;
            }
            if (errno == EINTR) continue;
            if (running_) err_ = std::string("read: ") + std::strerror(errno);
            running_ = false;
            return;
        }
        if (got < framesize) return;

        reduce(raw.data(), cfg_.src_width, cfg_.src_height, cfg_.out_size, small);
        uint32_t sum = 0;
        for (uint8_t v : small) sum += v;
        {
            std::lock_guard<std::mutex> lk(m_);
            frame_ = small;
            fresh_ = true;
            mean_  = small.empty() ? 0.0f : float(sum) / float(small.size());
            ++frames_;
        }
    }
}

bool CameraCapture::latest(std::vector<uint8_t>& out) {
    std::lock_guard<std::mutex> lk(m_);
    if (!fresh_ || frame_.empty()) return false;
    out = frame_;
    fresh_ = false;
    return true;
}

} // namespace ogma::hw
