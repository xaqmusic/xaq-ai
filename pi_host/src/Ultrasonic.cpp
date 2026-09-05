#include "ogma/hw/Ultrasonic.hpp"

#include <gpiod.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <glob.h>

namespace ogma::hw {

double Ultrasonic::speed_of_sound_mps(double air_temp_c) {
    // 331.3 + 0.606*T -- the standard linearisation.  Worth doing rather than
    // hardcoding 343: over 0-40 C the speed moves ~7 %, which is 7 % of every
    // distance this sensor will ever report.
    return 331.3 + 0.606 * air_temp_c;
}

double Ultrasonic::pulse_to_metres(uint64_t width_ns, double air_temp_c) {
    // Out and back, so halve it.
    return (double(width_ns) * 1e-9) * speed_of_sound_mps(air_temp_c) * 0.5;
}

std::string Ultrasonic::find_header_chip() {
    glob_t g{};
    if (::glob("/dev/gpiochip*", 0, nullptr, &g) != 0) return {};
    std::string found;
    for (size_t i = 0; i < g.gl_pathc && found.empty(); ++i) {
        gpiod_chip* c = gpiod_chip_open(g.gl_pathv[i]);
        if (!c) continue;
        if (gpiod_chip_info* info = gpiod_chip_get_info(c)) {
            const char* label = gpiod_chip_info_get_label(info);
            // Pi 5 brings the 40-pin header out on RP1; older boards on bcm2835/2711.
            if (label && (std::strstr(label, "rp1") || std::strstr(label, "bcm2835") ||
                          std::strstr(label, "bcm2711") || std::strstr(label, "pinctrl")))
                found = g.gl_pathv[i];
            gpiod_chip_info_free(info);
        }
        gpiod_chip_close(c);
    }
    ::globfree(&g);
    return found;
}

namespace {
// Small RAII wrappers: the v2 C API has six free functions to forget.
struct Settings {
    gpiod_line_settings* p = gpiod_line_settings_new();
    ~Settings() { if (p) gpiod_line_settings_free(p); }
};
struct LineCfg {
    gpiod_line_config* p = gpiod_line_config_new();
    ~LineCfg() { if (p) gpiod_line_config_free(p); }
};
struct ReqCfg {
    gpiod_request_config* p = gpiod_request_config_new();
    ~ReqCfg() { if (p) gpiod_request_config_free(p); }
};
} // namespace

Ultrasonic::Ultrasonic(Config cfg) : cfg_(std::move(cfg)) {}
Ultrasonic::~Ultrasonic() { stop(); }

bool Ultrasonic::start() {
    if (running_) return true;
    if (cfg_.chip.empty()) cfg_.chip = find_header_chip();
    if (cfg_.chip.empty()) { err_ = "no GPIO chip carrying the 40-pin header found"; return false; }
    running_ = true;
    err_.clear();
    th_ = std::thread(&Ultrasonic::run, this);
    return true;
}

void Ultrasonic::stop() {
    running_ = false;
    if (th_.joinable()) th_.join();
}

void Ultrasonic::run() {
    gpiod_chip* chip = gpiod_chip_open(cfg_.chip.c_str());
    if (!chip) { err_ = "gpiod_chip_open(" + cfg_.chip + ") failed"; running_ = false; return; }

    gpiod_line_request* trig_req = nullptr;
    gpiod_line_request* echo_req = nullptr;
    {   // trigger: output, starts low
        Settings s; LineCfg lc; ReqCfg rc;
        gpiod_line_settings_set_direction(s.p, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(s.p, GPIOD_LINE_VALUE_INACTIVE);
        unsigned off = cfg_.trig_offset;
        gpiod_line_config_add_line_settings(lc.p, &off, 1, s.p);
        gpiod_request_config_set_consumer(rc.p, "ogma_host:ultrasonic_trig");
        trig_req = gpiod_chip_request_lines(chip, rc.p, lc.p);
    }
    {   // echo: input with BOTH edges -- the timestamps come from the interrupt path
        Settings s; LineCfg lc; ReqCfg rc;
        gpiod_line_settings_set_direction(s.p, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_edge_detection(s.p, GPIOD_LINE_EDGE_BOTH);
        unsigned off = cfg_.echo_offset;
        gpiod_line_config_add_line_settings(lc.p, &off, 1, s.p);
        gpiod_request_config_set_consumer(rc.p, "ogma_host:ultrasonic_echo");
        echo_req = gpiod_chip_request_lines(chip, rc.p, lc.p);
    }
    if (!trig_req || !echo_req) {
        err_ = "gpiod_chip_request_lines failed (line busy? another process holding D0/D1?)";
        if (trig_req) gpiod_line_request_release(trig_req);
        if (echo_req) gpiod_line_request_release(echo_req);
        gpiod_chip_close(chip);
        running_ = false;
        return;
    }

    gpiod_edge_event_buffer* buf = gpiod_edge_event_buffer_new(8);
    const auto period  = std::chrono::microseconds(int64_t(1e6 / cfg_.hz));
    // Flight time bounds the wait: 4 m out and back at ~343 m/s is ~23 ms.
    const int64_t timeout_ns = int64_t((cfg_.max_range_m * 2.0 / 300.0) * 1e9) + 5000000;
    auto next = std::chrono::steady_clock::now();
    double prev_m = 0.0; bool have_prev = false;
    auto prev_t = std::chrono::steady_clock::now();

    while (running_) {
        next += period;
        std::this_thread::sleep_until(next);

        while (gpiod_line_request_wait_edge_events(echo_req, 0) == 1)   // drain stale edges
            gpiod_line_request_read_edge_events(echo_req, buf, 8);

        gpiod_line_request_set_value(trig_req, cfg_.trig_offset, GPIOD_LINE_VALUE_ACTIVE);
        std::this_thread::sleep_for(std::chrono::microseconds(12));      // datasheet wants >=10 us
        gpiod_line_request_set_value(trig_req, cfg_.trig_offset, GPIOD_LINE_VALUE_INACTIVE);

        uint64_t t_rise = 0, t_fall = 0;
        bool rose = false, fell = false;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::nanoseconds(timeout_ns);
        while (!fell && std::chrono::steady_clock::now() < deadline && running_) {
            if (gpiod_line_request_wait_edge_events(echo_req, timeout_ns) != 1) break;
            const int n = gpiod_line_request_read_edge_events(echo_req, buf, 8);
            for (int i = 0; i < n && !fell; ++i) {
                gpiod_edge_event* ev = gpiod_edge_event_buffer_get_event(buf, size_t(i));
                const uint64_t ts = gpiod_edge_event_get_timestamp_ns(ev);
                if (gpiod_edge_event_get_event_type(ev) == GPIOD_EDGE_EVENT_RISING_EDGE) {
                    t_rise = ts; rose = true;
                } else if (rose) {
                    t_fall = ts; fell = true;
                }
            }
        }

        Reading r;
        const auto now_t = std::chrono::steady_clock::now();
        if (fell && t_fall > t_rise) {
            r.distance_m = pulse_to_metres(t_fall - t_rise, cfg_.air_temp_c);
            // Beyond range is "no useful reading", not a large number: an out-of-range
            // ping and a 4 m wall are different facts and must not encode the same.
            r.valid = r.distance_m > 0.01 && r.distance_m <= cfg_.max_range_m;
            ++pings_;
        } else {
            // No echo: report the far limit, not zero.  See Reading::distance_m.
            r.distance_m = cfg_.max_range_m;
            r.valid      = false;
            ++timeouts_;
        }
        if (r.valid && have_prev) {
            const double dt = std::chrono::duration<double>(now_t - prev_t).count();
            if (dt > 1e-4) r.rate_mps = (r.distance_m - prev_m) / dt;
        }
        if (r.valid) { prev_m = r.distance_m; prev_t = now_t; have_prev = true; }

        std::lock_guard<std::mutex> lk(m_);
        if (fresh_) ++dropped_;              // the previous reading was never read
        r.seq  = ++last_.seq;
        last_  = r;
        fresh_ = true;
    }

    if (buf) gpiod_edge_event_buffer_free(buf);
    gpiod_line_request_release(trig_req);
    gpiod_line_request_release(echo_req);
    gpiod_chip_close(chip);
}

bool Ultrasonic::latest(Reading& out) {
    std::lock_guard<std::mutex> lk(m_);
    if (!fresh_) return false;
    out = last_;
    fresh_ = false;
    return true;
}

} // namespace ogma::hw
