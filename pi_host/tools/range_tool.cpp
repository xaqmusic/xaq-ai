// range_tool — what the ultrasonic actually reports, before any encoding.
//   range_tool watch [seconds] [hz]
// Prints raw distance so it can be held against a tape measure.  The point is to
// settle "is the SENSOR right" separately from "is the ENCODER right": those are
// different questions and conflating them is how a conditioning problem gets
// diagnosed as a broken rangefinder.
#include "ogma/hw/Ultrasonic.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace ogma::hw;

int main(int argc, char** argv) {
    const double secs = argc > 2 ? std::atof(argv[2]) : 10.0;
    Ultrasonic::Config cfg;
    if (argc > 3) cfg.hz = std::atof(argv[3]);
    if (argc < 2 || std::string(argv[1]) != "watch") {
        std::fprintf(stderr, "usage: range_tool watch [seconds] [hz]\n");
        return 2;
    }
    Ultrasonic us(cfg);
    if (!us.start()) { std::fprintf(stderr, "ultrasonic: %s\n", us.last_error().c_str()); return 1; }
    std::printf("trig GPIO%u  echo GPIO%u  %.0f Hz  max %.1f m  (speed of sound %.1f m/s at %.0f C)\n",
                cfg.trig_offset, cfg.echo_offset, cfg.hz, cfg.max_range_m,
                Ultrasonic::speed_of_sound_mps(cfg.air_temp_c), cfg.air_temp_c);

    std::vector<double> good;
    Ultrasonic::Reading r;
    const auto t0 = std::chrono::steady_clock::now();
    int n = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs) {
        if (!us.latest(r)) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        ++n;
        if (r.valid) good.push_back(r.distance_m);
        // A crude bar so a hand moving toward the sensor is visible as motion.
        const int cells = r.valid ? int(r.distance_m / 2.0 * 50.0) : 0;
        std::string bar(size_t(cells > 50 ? 50 : cells < 0 ? 0 : cells), '#');
        std::printf("  %6.1f cm  rate %+6.2f m/s  %-7s |%-50s|\n",
                    r.valid ? r.distance_m * 100.0 : 0.0, r.rate_mps,
                    r.valid ? "ok" : "NO ECHO", bar.c_str());
        std::fflush(stdout);
    }
    us.stop();

    std::printf("\n  readings %d   valid %zu   pings %llu   timeouts %llu\n",
                n, good.size(), (unsigned long long)us.pings(), (unsigned long long)us.timeouts());
    if (!good.empty()) {
        double mean = 0;
        for (double d : good) mean += d;
        mean /= double(good.size());
        double sd = 0;
        for (double d : good) sd += (d - mean) * (d - mean);
        sd = std::sqrt(sd / double(good.size()));
        double lo = good[0], hi = good[0];
        for (double d : good) { lo = std::fmin(lo, d); hi = std::fmax(hi, d); }
        std::printf("  distance  mean %.1f cm   sd %.2f cm   min %.1f   max %.1f\n",
                    mean * 100, sd * 100, lo * 100, hi * 100);
    }
    return 0;
}
