// sag_from_log — servo sag on the real robot, from a dash run's 50 Hz feed (2026-10-24).
//
// The servos report no position, so sag cannot be read directly.  But the feed records every commanded pulse and the
// belly rangefinder.  From the commanded pulses the robot host's own kinematics (BodyCalib + ServoMapping +
// feet_y_gravity_cmd_fk, the code ogma_host runs) give each foot's height below the body along gravity IF the servos
// tracked their commands exactly.  On level ground the planted feet (FSR contact) are on the floor, so that predicts the
// body's height; the belly ToF measures it.  Their difference over time, and how it moves with load, is the sag.
//
// Assumes level ground and an upright body (up = +y): a bench walk on flat floor.  Prints one JSON line per frame with
// the commanded height (mean over planted feet, and over all feet), the ToF, contact count and servo current.
//
// Build (x86 or Pi):  g++ -std=c++20 -O2 -I cpp_core/include -I pi_host/include -I <nlohmann include>
//                     pi_host/tools/sag_from_log.cpp -o sag_from_log
// Run:                sag_from_log <feed.jsonl> [servo_map.json] [body_measured_fsr.json] > sag.jsonl
#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
#include "ogma/hw/BrainInputs.hpp"

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: sag_from_log feed.jsonl [servo_map.json] [body.json]\n"; return 2; }
    const std::string map_path  = argc > 2 ? argv[2] : "pi_host/calib/servo_map.json";
    const std::string body_path = argc > 3 ? argv[3] : "pi_host/calib/body_measured_fsr.json";
    const auto map  = ogma::hw::brain::ServoMapping::load(map_path);
    const auto body = ogma::hw::brain::BodyCalib::load(body_path);
    if (!map.complete) { std::cerr << "servo map: " << map.why << "\n"; return 1; }
    if (!body.ok)      { std::cerr << "body: " << body.why << "\n"; return 1; }
    const double us_per_rad = 545.2;
    const ogma::body::Vec3f up(0.0f, 1.0f, 0.0f);
    std::ifstream f(argv[1]);
    std::string line;
    while (std::getline(f, line)) {
        nlohmann::json j;
        try { j = nlohmann::json::parse(line); } catch (...) { continue; }
        if (!j.contains("us") || !j.contains("fsr")) continue;
        std::array<int, 12> us{};
        bool live = true;
        for (int c = 0; c < 12; ++c) { us[size_t(c)] = j["us"][size_t(c)].get<int>(); if (us[size_t(c)] <= 0) live = false; }
        if (!live) continue;
        const auto ang  = ogma::hw::brain::hinge_angles_from_us(us, map, us_per_rad);
        const auto fy   = ogma::hw::brain::feet_y_gravity_cmd_fk(body, ang, up);   // foot height rel. body, m (negative = below)
        std::array<int, 4> fsr{};
        for (int i = 0; i < 4; ++i) fsr[size_t(i)] = j["fsr"][size_t(i)].get<int>();
        double sum_p = 0.0, sum_a = 0.0; int np = 0;
        nlohmann::json legs = nlohmann::json::array();
        for (int i = 0; i < 4; ++i) {
            const bool planted = fsr[size_t(ogma::hw::brain::kFsrChannelForSimLeg[size_t(i)])] >= 200;
            sum_a += -fy[size_t(i)];
            if (planted) { sum_p += -fy[size_t(i)]; ++np; }
            legs.push_back(-fy[size_t(i)]);
        }
        nlohmann::json o{{"t", j.value("t", 0)}, {"mode", j.value("mode", std::string())}, {"planted", np},
                         {"cmd_h_all", sum_a / 4.0}, {"cmd_h_planted", np ? sum_p / np : -1.0}, {"leg_h", legs},
                         {"tof_m", j.value("tof_m", -1.0)}, {"tof_valid", j.value("tof_valid", false)},
                         {"i_a", j.contains("i_a") && j["i_a"].is_number() ? j["i_a"].get<double>() : -1.0}};
        std::cout << o.dump() << "\n";
    }
    return 0;
}
