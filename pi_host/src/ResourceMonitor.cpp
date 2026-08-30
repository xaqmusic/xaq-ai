#include "ogma/hw/ResourceMonitor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace ogma::hw {

// ---------------------------------------------------------------- TickBudget

TickBudget::TickBudget(double tick_hz, int window)
    : budget_ns_(tick_hz > 0.0 ? 1e9 / tick_hz : 0.0), window_(window > 0 ? window : 1) {
    wall_.reserve(window_);
    cpu_.reserve(window_);
}

double TickBudget::percentile(const std::vector<int64_t>& sorted, double q) {
    if (sorted.empty()) return 0.0;
    // Nearest-rank: the smallest value at or above q of the way through.  No
    // interpolation, so every reported number is a tick that actually happened.
    const std::size_t n = sorted.size();
    std::size_t idx = static_cast<std::size_t>(q * static_cast<double>(n));
    if (idx >= n) idx = n - 1;
    return static_cast<double>(sorted[idx]);
}

void TickBudget::close_window() {
    auto w = wall_, c = cpu_;
    std::sort(w.begin(), w.end());
    std::sort(c.begin(), c.end());
    const double to_pct = budget_ns_ > 0.0 ? 100.0 / budget_ns_ : 0.0;
    last_.wall_p50 = percentile(w, 0.50) * to_pct;
    last_.wall_p95 = percentile(w, 0.95) * to_pct;
    last_.wall_max = (w.empty() ? 0.0 : double(w.back())) * to_pct;
    last_.cpu_p50  = percentile(c, 0.50) * to_pct;
    last_.cpu_p95  = percentile(c, 0.95) * to_pct;
    last_.cpu_max  = (c.empty() ? 0.0 : double(c.back())) * to_pct;
    last_.budget_ms = budget_ns_ / 1e6;
    last_.n = static_cast<int>(w.size());
    wall_.clear();
    cpu_.clear();
}

bool TickBudget::sample(int64_t wall_ns, int64_t cpu_ns) {
    wall_.push_back(wall_ns < 0 ? 0 : wall_ns);
    cpu_.push_back(cpu_ns < 0 ? 0 : cpu_ns);
    if (static_cast<int>(wall_.size()) < window_) return false;
    close_window();
    return true;
}

// ----------------------------------------------------------------- HostStats

namespace {
std::string slurp(const std::string& path) {
    std::ifstream f(path);
    if (!f) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace

HostStats::HostStats(std::string proc_root, std::string sys_root)
    : proc_root_(std::move(proc_root)), sys_root_(std::move(sys_root)),
      page_kb_(::sysconf(_SC_PAGESIZE) / 1024), clk_tck_(::sysconf(_SC_CLK_TCK)) {
    if (page_kb_ <= 0) page_kb_ = 4;
    if (clk_tck_ <= 0) clk_tck_ = 100;
}

// /proc/*/status and /proc/meminfo are "Key:   <n> kB" lines.
double HostStats::parse_kb_field(const std::string& text, const std::string& key) {
    const std::string needle = key + ":";
    std::size_t p = text.find(needle);
    // Must be at a line start, or "VmRSS" would also match inside another key.
    while (p != std::string::npos && p != 0 && text[p - 1] != '\n')
        p = text.find(needle, p + 1);
    if (p == std::string::npos) return -1.0;
    return std::strtod(text.c_str() + p + needle.size(), nullptr) / 1024.0;   // kB -> MB
}

// /proc/*/stat.  ⚠ field 2 is the comm in parentheses and MAY CONTAIN SPACES AND
// PARENS, so the only safe parse starts after the LAST ')'.  Fields (1-indexed
// from there): 12 majflt, 14 utime, 15 stime.
bool HostStats::parse_stat_cpu(const std::string& text, long& utime, long& stime, long& majflt) {
    const std::size_t close = text.rfind(')');
    if (close == std::string::npos) return false;
    std::istringstream ss(text.substr(close + 1));
    std::string tok;
    std::vector<std::string> f;
    while (ss >> tok) f.push_back(tok);
    // f[0] is field 3 (state), so field N is f[N-3].
    if (f.size() < 13) return false;
    majflt = std::strtol(f[12 - 3].c_str(), nullptr, 10);
    utime  = std::strtol(f[14 - 3].c_str(), nullptr, 10);
    stime  = std::strtol(f[15 - 3].c_str(), nullptr, 10);
    return true;
}

HostSample HostStats::sample() {
    HostSample s;
    const std::string status = slurp(proc_root_ + "/self/status");
    const std::string stat   = slurp(proc_root_ + "/self/stat");
    if (status.empty() && stat.empty()) return s;                 // ok stays false

    const double rss  = parse_kb_field(status, "VmRSS");
    const double swap = parse_kb_field(status, "VmSwap");
    s.rss_mb  = rss  > 0 ? rss  : 0.0;
    s.swap_mb = swap > 0 ? swap : 0.0;

    const double avail = parse_kb_field(slurp(proc_root_ + "/meminfo"), "MemAvailable");
    s.mem_avail_mb = avail > 0 ? avail : 0.0;

    long ut = 0, st = 0, mf = 0;
    const int64_t ms = now_ms();
    if (parse_stat_cpu(stat, ut, st, mf)) {
        s.majflt = mf;
        const long cpu_ticks = ut + st;
        if (have_prev_ && ms > prev_ms_)
            s.proc_cpu_pct = 100.0 * double(cpu_ticks - prev_cpu_ticks_) / double(clk_tck_)
                             / (double(ms - prev_ms_) / 1000.0);
        prev_cpu_ticks_ = cpu_ticks;
        prev_ms_ = ms;
        have_prev_ = true;
    }

    // Growth since the first sample.  A mean slope, not a fit: over a long run it
    // converges, and it is the leak signal that matters as the brain grows.
    if (first_ms_ == 0) { first_ms_ = ms; first_rss_mb_ = s.rss_mb; }
    const double minutes = double(ms - first_ms_) / 60000.0;
    if (minutes > 0.5) s.rss_growth_mb_per_min = (s.rss_mb - first_rss_mb_) / minutes;

    const std::string t = slurp(sys_root_ + "/class/thermal/thermal_zone0/temp");
    if (!t.empty()) s.cpu_temp_c = std::strtod(t.c_str(), nullptr) / 1000.0;

    s.ok = true;
    return s;
}

} // namespace ogma::hw
