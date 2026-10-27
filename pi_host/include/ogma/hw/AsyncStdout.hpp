#pragma once
// AsyncStdout -- stdout through a pipe and a writer thread, so the tick loop never waits on the SD card (2026-10-26).
//
// ⚠ WHY.  The dash runs ogma_host with stdout on a file in pi_host/log, on the SD card, and the host prints a JSON line
// every tick (--dump-actions) plus the input dumps.  A card write can block for 0.1 s to seconds (journal commits,
// writeback); benchd froze the servos on exactly that until its writes left the lock.  Robot runs 2026-10-26 (robot_p1_
// presskin_lz, two runs): benchd's feed had no gap over 100 ms, yet ogma_host stalled 0.7-3.6 s eight times — the brain
// held, the servos paused.
//
// HOW.  fd 1 becomes the write end of a pipe (enlarged to 1 MB, ~30 s of the dump at 50 Hz); a thread copies the pipe to
// the real file.  printf() returns as soon as the pipe has room, so a card stall costs the writer, not the tick.  A write
// to the file over SLOW_MS is noted in-band as {"kind":"stdout_slow",...}, so the record says when the card stalled.
// A terminal is left alone (no file behind it).  Only stdout: stderr stays direct and rare.
#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace ogma::hw {

class AsyncStdout {
public:
    static constexpr double SLOW_MS = 80.0;

    // Returns false (and changes nothing) when stdout is a terminal or the pipe cannot be made.
    bool start() {
        if (isatty(STDOUT_FILENO)) return false;
        std::fflush(stdout);
        int p[2];
        if (::pipe2(p, O_CLOEXEC) != 0) return false;
        ::fcntl(p[1], F_SETPIPE_SZ, 1 << 20);   // best effort; the default is 64 KB
        real_ = ::dup(STDOUT_FILENO);
        if (real_ < 0) { ::close(p[0]); ::close(p[1]); return false; }
        if (::dup2(p[1], STDOUT_FILENO) < 0) { ::close(p[0]); ::close(p[1]); ::close(real_); real_ = -1; return false; }
        ::close(p[1]);
        rd_ = p[0];
        thr_ = std::thread([this] { run(); });
        return true;
    }
    // Flushes, restores fd 1 to the file, and waits for the writer to drain the pipe.
    void stop() {
        if (rd_ < 0) return;
        std::fflush(stdout);
        ::dup2(real_, STDOUT_FILENO);            // closes the pipe's last write end: the writer sees EOF after the backlog
        if (thr_.joinable()) thr_.join();
        ::close(rd_); rd_ = -1;
        ::close(real_); real_ = -1;
    }
    ~AsyncStdout() { stop(); }

    long   slow_writes() const { return slow_.load(); }
    double max_write_ms() const { return max_ms_.load(); }
    long   max_backlog() const { return backlog_.load(); }   // bytes waiting in the pipe, worst seen

private:
    void run() {
        static char buf[1 << 16];
        for (;;) {
            int queued = 0;
            if (::ioctl(rd_, FIONREAD, &queued) == 0 && queued > backlog_.load()) backlog_ = queued;
            const ssize_t n = ::read(rd_, buf, sizeof buf);
            if (n <= 0) { flush_note(); return; }
            const auto t0 = std::chrono::steady_clock::now();
            // A pending note goes in at the first line break of this chunk, never inside a line (a note written at a
            // chunk edge split a JSON line in two, robot run 2026-10-27).
            ssize_t off = 0;
            if (!note_.empty()) {
                const void* nl = std::memchr(buf, '\n', size_t(n));
                if (nl) {
                    const ssize_t k = static_cast<const char*>(nl) - buf + 1;
                    write_all(buf, k); off = k;
                    flush_note();
                }
            }
            write_all(buf + off, n - off);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            if (ms > max_ms_.load()) max_ms_ = ms;
            if (ms > SLOW_MS) {
                ++slow_;
                note_ += "{\"kind\":\"stdout_slow\",\"ms\":" + std::to_string(ms) + ",\"bytes\":" + std::to_string(n) + "}\n";
                if (buf[n - 1] == '\n') flush_note();     // this chunk ended on a line break: safe now
            }
        }
    }
    void write_all(const char* p, ssize_t len) {
        for (ssize_t off = 0; off < len;) {
            const ssize_t w = ::write(real_, p + off, size_t(len - off));
            if (w <= 0) break;
            off += w;
        }
    }
    void flush_note() { if (!note_.empty()) { write_all(note_.data(), ssize_t(note_.size())); note_.clear(); } }
    std::string note_;
    int rd_ = -1, real_ = -1;
    std::thread thr_;
    std::atomic<long> slow_{0}, backlog_{0};
    std::atomic<double> max_ms_{0.0};
};

}  // namespace ogma::hw
