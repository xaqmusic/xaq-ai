#pragma once
// ResourceMonitor — what the host is costing us, in the units a control loop cares about.
//
// Two instruments, deliberately separate because they are sampled from different
// threads at different rates:
//
//   TickBudget  — per tick, on the tick thread.  How much of the tick PERIOD each
//                 tick consumed.  This is the real-time number: at 50 Hz the budget
//                 is 20 ms, and a tick that takes 10 ms is at 50 %.
//   HostStats   — ~1 Hz, OFF the tick thread.  Process and machine state from /proc.
//                 Reading /proc costs ~100 us; doing it under the tick mutex would
//                 make the instrument a cause of the thing it measures.
//
// Two design points that are the whole value of this file:
//
// 1. A MEAN IS A BLIND METRIC.  A loop averaging 15 % of budget while spiking to
//    110 % every 40th tick reads healthy and still misses deadlines.  The window
//    therefore reports p50, p95 and MAX; max is the one that predicts an overrun.
//
// 2. WALL TIME AND CPU TIME ARE DIFFERENT QUESTIONS.  The same span is measured on
//    CLOCK_MONOTONIC (how much budget went) and CLOCK_THREAD_CPUTIME_ID (how much of
//    it was computing).  wall >> cpu means the tick is BLOCKED -- the 12 I2C servo
//    writes, lock contention, preemption -- and more compute is nearly free.
//    wall ~= cpu means compute-bound, and every new module costs budget directly.
//    A single percentage cannot tell those apart, and they have opposite responses.
#include <cstdint>
#include <string>
#include <vector>

namespace ogma::hw {

// Percentages are of the tick period, so 100.0 means "this tick used its whole budget".
struct TickStats {
    double wall_p50 = 0.0, wall_p95 = 0.0, wall_max = 0.0;
    double cpu_p50  = 0.0, cpu_p95  = 0.0, cpu_max  = 0.0;
    double budget_ms = 0.0;      // the period these percentages are against
    int    n = 0;                // samples in the window (0 = none closed yet)
};

class TickBudget {
public:
    // tick_hz sets the budget the percentages are against; window is in ticks.
    explicit TickBudget(double tick_hz, int window = 100);

    // Feed one tick.  Returns true on the tick that closed a window, i.e. when
    // last() has just been replaced with fresh numbers.
    bool sample(int64_t wall_ns, int64_t cpu_ns);
    const TickStats& last() const { return last_; }

    // Pure helper, exposed for the tests: nearest-rank percentile of a SORTED span.
    static double percentile(const std::vector<int64_t>& sorted, double q);

private:
    void close_window();
    double budget_ns_;
    int    window_;
    std::vector<int64_t> wall_, cpu_;
    TickStats last_{};
};

struct HostSample {
    double rss_mb        = 0.0;
    double swap_mb       = 0.0;   // non-zero on a 2 GB board = latency is about to die
    double mem_avail_mb  = 0.0;
    double rss_growth_mb_per_min = 0.0;   // mean slope since the first sample
    double proc_cpu_pct  = 0.0;   // all threads, % of ONE core, since the last sample
    double cpu_temp_c    = 0.0;   // thermal throttling silently shrinks the tick budget
    long   majflt        = 0;     // major faults: pages fetched from disk/zram, not free
    bool   ok            = false; // false if /proc could not be read at all
};

class HostStats {
public:
    // Roots are injectable so the parsers can be tested against fixture files
    // rather than against whatever the build machine happens to be doing.
    explicit HostStats(std::string proc_root = "/proc",
                       std::string sys_root  = "/sys");
    // Call ~1 Hz, never while holding a lock the tick thread wants.
    HostSample sample();

    // Pure helpers, exposed for the tests.
    static double parse_kb_field(const std::string& status_text, const std::string& key);
    static bool   parse_stat_cpu(const std::string& stat_text, long& utime, long& stime, long& majflt);

private:
    std::string proc_root_, sys_root_;
    long   page_kb_;
    long   clk_tck_;
    bool   have_prev_ = false;
    long   prev_cpu_ticks_ = 0;
    int64_t prev_ms_ = 0;
    double first_rss_mb_ = 0.0;
    int64_t first_ms_ = 0;
};

} // namespace ogma::hw
