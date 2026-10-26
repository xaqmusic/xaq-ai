// leg_jac_from_log — how much each leg's femur and knee can lift its corner, from a dash run's 50 Hz feed (2026-10-26).
//
// For every frame the commanded pulses give the joint angles (as sag_from_log); the robot host's own kinematics
// (fk_leg + BodyCalib, the code ogma_host runs) then give each foot's depth below the body along gravity and, by finite
// difference, its GAIN: depth change per radian of hip2 and per radian of knee.  The learned stance press adds the same
// command to both (press_knee 1), so its lift is the sum; where that sum falls to zero or below, pressing harder folds
// the leg under the body instead of raising the corner (operator 2026-10-26: the right rear leg tucking under).
// Prints one JSON line per frame: per leg [depth m, d depth/d hip2, d depth/d knee, foot reach from the hip m], FSR
// planted flags, the belly ToF.  Level ground assumed (up = +y), as sag_from_log.
//
// Build: g++ -std=c++20 -O2 -I cpp_core/include -I pi_host/include -I <nlohmann include> pi_host/tools/leg_jac_from_log.cpp
// Run:   leg_jac_from_log <feed.jsonl> [servo_map.json] [body_measured_fsr.json] > jac.jsonl
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
#include "ogma/hw/BrainInputs.hpp"

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: leg_jac_from_log feed.jsonl [servo_map.json] [body.json]\n"; return 2; }
    const auto map  = ogma::hw::brain::ServoMapping::load(argc > 2 ? argv[2] : "pi_host/calib/servo_map.json");
    const auto body = ogma::hw::brain::BodyCalib::load(argc > 3 ? argv[3] : "pi_host/calib/body_measured_fsr.json");
    if (!map.complete) { std::cerr << "servo map: " << map.why << "\n"; return 1; }
    if (!body.ok)      { std::cerr << "body: " << body.why << "\n"; return 1; }
    const ogma::body::Vec3f up(0.0f, 1.0f, 0.0f);
    const double eps = 0.01;
    auto depth = [&](int i, double t1, double t2, double t3) {
        const auto p = ogma::body::fk_leg(body.anchors[size_t(i)], t1, t2, t3, 0.0);
        return -double(ogma::body::feet_y_gravity(body.rest_inv(p.lower.origin), up, body.l3));
    };
    std::ifstream f(argv[1]);
    std::string line;
    while (std::getline(f, line)) {
        nlohmann::json j;
        try { j = nlohmann::json::parse(line); } catch (...) { continue; }
        if (!j.contains("us") || !j.contains("fsr") || j.value("mode", std::string()) != "autonomous") continue;
        std::array<int, 12> us{};
        bool live = true;
        for (int c = 0; c < 12; ++c) { us[size_t(c)] = j["us"][size_t(c)].get<int>(); if (us[size_t(c)] <= 0) live = false; }
        if (!live) continue;
        const auto a = ogma::hw::brain::hinge_angles_from_us(us, map, 545.2);
        nlohmann::json legs = nlohmann::json::array(), planted = nlohmann::json::array();
        for (int i = 0; i < 4; ++i) {
            const double t1 = a[size_t(i * 3)], t2 = a[size_t(i * 3 + 1)], t3 = a[size_t(i * 3 + 2)];
            const double d0 = depth(i, t1, t2, t3);
            const double g2 = (depth(i, t1, t2 + eps, t3) - d0) / eps;
            const double g3 = (depth(i, t1, t2, t3 + eps) - d0) / eps;
            const auto p = ogma::body::fk_leg(body.anchors[size_t(i)], t1, t2, t3, 0.0);
            const auto hip = body.anchors[size_t(i)].hip1_world;
            const double reach = std::hypot(double(p.lower.origin.x - hip.x), double(p.lower.origin.z - hip.z));
            legs.push_back({d0, g2, g3, reach, t2, t3});
            planted.push_back(j["fsr"][size_t(ogma::hw::brain::kFsrChannelForSimLeg[size_t(i)])].get<int>() >= 200 ? 1 : 0);
        }
        std::cout << nlohmann::json{{"t", j.value("t", 0)}, {"legs", legs}, {"planted", planted},
                                    {"tof_m", j.value("tof_m", -1.0)}, {"tof_valid", j.value("tof_valid", false)}}.dump() << "\n";
    }
    return 0;
}
