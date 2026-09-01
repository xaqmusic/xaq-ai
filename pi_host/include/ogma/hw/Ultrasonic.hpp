#pragma once
// Ultrasonic — the forward rangefinder the robot already carries.
//
// ⚠ FORWARD-FACING, FOR OBSTACLES.  It is NOT the belly channel.  `gc_raw` is a
// DOWNWARD belly sensor that a promoted height homeostat rides, and the VL53L0X is
// what will feed it.  Routing a forward reading into `gc_raw` would silently corrupt
// a promoted lever (BOM §7, port doc §7.7).  This class publishes its own topic and
// knows nothing about that one.
//
// ⚠ PINS ARE D2/D3, NOT D0/D1.  BOM §1 and §7 both assumed trigger D0 (GPIO17) /
// echo D1 (GPIO4) and recorded "D2/D3 stay free".  MEASURED 2026-08-30: with every
// D pin held as an input, GPIO22 (D3) ignores an internal pull and reads low both
// ways — a device output idling low — while GPIO4, GPIO17 and GPIO27 all follow the
// pull, i.e. nothing is connected.  Triggering GPIO27 (D2) then produces a clean
// 866 us echo on GPIO22 every time.  So: trigger D2 = GPIO27, echo D3 = GPIO22,
// which is also what SunFounder's own PiCrawler examples pair.
//
// TIMING.  The BOM's worry was that echo is "a userspace pulse width at ~58 us/cm, so
// 100 us of scheduling jitter is 1.7 cm".  That is true of a poll loop, so this does
// not use one: libgpiod v2 edge events carry a KERNEL timestamp taken in the interrupt
// path, and the width is the difference of two of those.  Userspace scheduling then
// affects when we learn the answer, not what the answer is.
//
// Rate is ~15 Hz and bounded by flight time (~23 ms for 4 m, there and back). It
// cannot be a 50 Hz channel, so it runs on its own thread and the brain reads the
// newest value.
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace ogma::hw {

class Ultrasonic {
public:
    struct Config {
        std::string chip;                 // empty = auto-detect the RP1 pinctrl chip
        unsigned trig_offset = 27;        // D2 — measured, not assumed
        unsigned echo_offset = 22;        // D3 — measured, not assumed
        // 20 Hz measured 2026-09-01 as the reliable ceiling on a static target:
        // sd 0.56 cm at 20, 1.34 at 30, and 12.7 at 40 where the previous ping's echo
        // ghosts into the next window (the MEAN drifts too, 37 -> 48 cm, which is the
        // tell that it is trailing echoes and not just noise).
        double   hz           = 20.0;
        double   max_range_m  = 4.0;
        double   air_temp_c   = 20.0;     // speed of sound is temperature-dependent
    };

    struct Reading {
        double   distance_m = 0.0;
        double   rate_mps   = 0.0;        // closing speed, from consecutive readings
        bool     valid      = false;      // false = no echo, or beyond max range
        uint64_t seq        = 0;
    };

    explicit Ultrasonic(Config cfg);
    ~Ultrasonic();
    Ultrasonic(const Ultrasonic&) = delete;
    Ultrasonic& operator=(const Ultrasonic&) = delete;

    bool start();
    void stop();
    bool running() const { return running_.load(); }
    bool latest(Reading& out);            // false when nothing new since last call

    const std::string& last_error() const { return err_; }
    uint64_t pings()   const { return pings_; }
    uint64_t timeouts() const { return timeouts_; }

    // Pure helpers, exposed for the tests.
    static double speed_of_sound_mps(double air_temp_c);
    static double pulse_to_metres(uint64_t width_ns, double air_temp_c);
    // Auto-detect the chip carrying the 40-pin header (RP1 on a Pi 5, bcm2835 before).
    static std::string find_header_chip();

private:
    void run();

    Config      cfg_;
    std::thread th_;
    mutable std::mutex m_;
    Reading     last_{};
    bool        fresh_    = false;
    std::atomic<bool> running_{false};
    uint64_t    pings_    = 0;
    uint64_t    timeouts_ = 0;
    std::string err_;
};

} // namespace ogma::hw
