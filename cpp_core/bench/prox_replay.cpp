// prox_replay — a robot run's logged range pings and odometry through the real ProxMemory, offline (2026-10-27).
//
// Why: the robot's ultrasonic returns single false close pings (a front foot in the beam).  ProxMemory's corroboration
// (support_n, too_close_pings) is meant to drop them without delaying a real wall; this replays one run under several
// settings so the setting is chosen from the robot's own data before it drives the robot.
//
// Input: an ogma_host run log (the dash's), which carries {"kind":"range",...} per ping and the brain_inputs dumps
// (every 50 ticks) with "odom" [x, y, yaw].  Odometry is interpolated linearly between dumps (the body moves ~6 cm and
// turns a few degrees per second, so the error is millimetres).  The "trend" is the median of the pings over the last
// 2 s: a tick whose ahead slot says nearer than 0.25 m while the trend is beyond 0.40 m is counted FALSE.
//
// Usage: prox_replay <run.log> [support_n too_close_pings]...   (default: 1 1  2 2  3 2  3 3  5 3)
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ProxMemory.hpp"

namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)];
    p->producer_id = "host";
    return p;
}
struct Ping { long tick; float m; bool valid; };
}

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: prox_replay <run.log> [support_n too_close_pings]...\n"; return 2; }
    std::vector<Ping> pings;
    std::map<long, std::array<double, 3>> odom;
    std::map<long, double> gyro;               // per-tick yaw rate (deg/s, body up) from the imu rows, when logged
    std::ifstream f(argv[1]);
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("{\"a_g\"", 0) == 0) {
            nlohmann::json j;
            try { j = nlohmann::json::parse(line); } catch (...) { continue; }
            if (j.value("kind", "") == "imu" && j.contains("g_dps")) gyro[j["tick"].get<long>()] = j["g_dps"][1].get<double>();
            continue;
        }
        if (line.rfind("{\"kind\":\"range\"", 0) == 0 || line.rfind("{\"distress\"", 0) == 0) {
            nlohmann::json j;
            try { j = nlohmann::json::parse(line); } catch (...) { continue; }
            if (j.value("kind", "") == "range")
                pings.push_back({j["tick"].get<long>(), j["m"].get<float>(), j["valid"].get<int>() != 0 && j["dz"].get<int>() == 0});
            else if (j.value("kind", "") == "brain_inputs" && j.contains("odom") && j["odom"].size() == 3)
                odom[j["tick"].get<long>()] = {j["odom"][0].get<double>(), j["odom"][1].get<double>(), j["odom"][2].get<double>()};
        }
    }
    if (pings.empty() || odom.size() < 2) { std::cerr << "need range pings and odom dumps (" << pings.size() << ", " << odom.size() << ")\n"; return 1; }
    auto odom_at = [&](long t) -> std::array<double, 3> {
        auto hi = odom.lower_bound(t);
        if (hi == odom.begin()) return hi->second;
        if (hi == odom.end()) return std::prev(hi)->second;
        auto lo = std::prev(hi);
        const double a = double(t - lo->first) / double(hi->first - lo->first);
        double dyaw = hi->second[2] - lo->second[2];
        while (dyaw > M_PI) dyaw -= 2 * M_PI;
        while (dyaw < -M_PI) dyaw += 2 * M_PI;
        return {lo->second[0] + a * (hi->second[0] - lo->second[0]), lo->second[1] + a * (hi->second[1] - lo->second[1]),
                lo->second[2] + a * dyaw};
    };
    // Arms: support_n too_close_pings [odom_translation]  (the third is optional, default 1 = the full odometry).
    // Arms: support_n too_close_pings [odom_translation] [f]   ("f" = pinged_range_floor on).
    struct Arm { int k, tc; double tr; int pf; };
    std::vector<Arm> arms;
    for (int i = 2; i + 1 < argc; ) {
        Arm a{std::atoi(argv[i]), std::atoi(argv[i + 1]), 1.0, 0}; i += 2;
        if (i < argc && std::strchr(argv[i], '.')) { a.tr = std::atof(argv[i]); ++i; }
        if (i < argc && std::strcmp(argv[i], "f") == 0) { a.pf = 1; ++i; }
        arms.push_back(a);
    }
    if (arms.empty()) arms = {{1, 1, 1.0, 0}, {2, 2, 1.0, 0}, {3, 2, 1.0, 0}, {3, 3, 1.0, 0}, {5, 3, 1.0, 0}};
    const long t0 = pings.front().tick, t1 = pings.back().tick;
    // Yaw at 50 Hz: the 1 Hz odom yaw is too coarse for a body turning 5 deg/s, so when the per-tick imu rows
    // are present the yaw between odom samples is the gyro integrated from the last sample (the same gyro the
    // host integrates); the translation stays interpolated.
    const bool have_gyro = !gyro.empty();
    std::printf("%zu pings over ticks %ld..%ld (%.0f s); %zu odom samples; yaw %s\n", pings.size(), t0, t1, (t1 - t0) / 50.0, odom.size(),
                have_gyro ? "gyro-integrated between odom samples" : "interpolated (no imu rows)");
    std::printf("support too_close transl floor | ahead<0.25m ticks: all / FALSE (trend > 0.40 m) | too_close: all / FALSE | first real ahead<0.25m | ACTIVE ticks (max slot >= 0.10) | STRONG (>= 0.30) | side L / R\n");
    for (auto [k, tc, tr, pf] : arms) {
        ogma::InProcessBus bus; ogma::ProxMemory m;
        m.set_id("prox");
        m.on_setup(&bus, {{"support_n", int64_t(k)}, {"too_close_pings", int64_t(tc)}, {"range_m", 0.5}, {"too_close_m", 0.15},
                          {"odom_translation", tr}, {"pinged_range_floor", double(pf)}});
        long active = 0, strong = 0, sideL = 0, sideR = 0;
        double yaw_g = 0.0; long last_anchor = -1;
        size_t pi = 0;
        std::deque<float> recent;          // valid ping metres over the last 100 ticks
        std::deque<long>  recent_t;
        float held_m = 1.5f; bool held_v = false;
        long ahead_all = 0, ahead_false = 0, tc_all = 0, tc_false = 0, first_real = -1;
        for (long t = t0; t <= t1; ++t) {
            while (pi < pings.size() && pings[pi].tick <= t) {
                held_m = pings[pi].m; held_v = pings[pi].valid;
                if (held_v) { recent.push_back(held_m); recent_t.push_back(t); }
                ++pi;
            }
            while (!recent_t.empty() && t - recent_t.front() > 100) { recent.pop_front(); recent_t.pop_front(); }
            std::vector<float> s(recent.begin(), recent.end());
            float trend = 9.0f;
            if (!s.empty()) { std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end()); trend = s[s.size() / 2]; }
            auto o = odom_at(t);
            if (have_gyro) {
                auto oa = odom.find(t);
                if (oa != odom.end()) { yaw_g = oa->second[2]; last_anchor = t; }
                else if (last_anchor >= 0) { auto g = gyro.find(t); if (g != gyro.end()) yaw_g += g->second * (M_PI / 180.0) / 50.0; }
                else yaw_g = o[2];
                o[2] = yaw_g;
            }
            bus.begin_tick(uint64_t(t));
            bus.publish("reality.proprio.odom", pv({float(o[0]), float(o[1]), float(o[2])}));
            bus.publish("sense.range", pv({held_m, held_v ? 1.0f : 0.0f}));
            m.tick(uint64_t(t));
            bus.end_tick();
            const bool ahead_near = m.slot(1) > 0.5f;           // nearer than 0.25 m with range_m 0.5
            if (ahead_near) { ++ahead_all; if (trend > 0.40f) ++ahead_false; else if (first_real < 0) first_real = t; }
            if (m.slot(3) > 0.0f) { ++tc_all; if (trend > 0.40f) ++tc_false; }
            const float mx = std::max(m.slot(0), std::max(m.slot(1), m.slot(2)));
            if (mx >= 0.10f) { ++active; if (m.slot(0) > m.slot(2)) ++sideL; else if (m.slot(2) > m.slot(0)) ++sideR; }
            if (mx >= 0.30f) ++strong;
        }
        std::printf("   %d       %d     %4.2f   %d   | %6ld / %6ld                            | %5ld / %5ld     | %6ld            | %6ld                      | %6ld           | %5ld / %5ld\n",
                    k, tc, tr, pf, ahead_all, ahead_false, tc_all, tc_false, first_real, active, strong, sideL, sideR);
    }
    return 0;
}
