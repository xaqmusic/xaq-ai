// leg_jac_from_log — how much each leg's femur and knee can lift its corner, from a dash run's 50 Hz feed (2026-10-26).
//
// For every frame the commanded pulses give the joint angles (as sag_from_log); the robot host's own kinematics
// (fk_leg + BodyCalib, the code ogma_host runs) then give each foot's depth below the body along gravity and, by finite
// difference, its GAIN: depth change per radian of hip2 and per radian of knee.  The learned stance press adds the same
// command to both (press_knee 1), so its lift is the sum; where that sum falls to zero or below, pressing harder folds
// the leg under the body instead of raising the corner (operator 2026-10-26: the right rear leg tucking under).
// Prints one JSON line per frame: per leg [depth m, d depth/d hip2, d depth/d knee, shin-midpoint reach from the hip m,
// hip2 rad, knee rad, femur tilt deg, shin tilt deg, toe x m, toe z m (chassis rest frame)], FSR
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
    const ogma::hw::brain::FsrModel fsr;   // predicted counts -> grams curve (unmeasured below 30 g)
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
        nlohmann::json legs = nlohmann::json::array(), planted = nlohmann::json::array(), grams = nlohmann::json::array();
        for (int i = 0; i < 4; ++i) {
            const double t1 = a[size_t(i * 3)], t2 = a[size_t(i * 3 + 1)], t3 = a[size_t(i * 3 + 2)];
            const double d0 = depth(i, t1, t2, t3);
            const double g2 = (depth(i, t1, t2 + eps, t3) - d0) / eps;
            const double g3 = (depth(i, t1, t2, t3 + eps) - d0) / eps;
            const auto p = ogma::body::fk_leg(body.anchors[size_t(i)], t1, t2, t3, 0.0);
            const auto hip = body.anchors[size_t(i)].hip1_world;
            const double reach = std::hypot(double(p.lower.origin.x - hip.x), double(p.lower.origin.z - hip.z));
            // Segment tilts from the body vertical (level ground assumed): the femur hip2 -> knee and the shin
            // knee -> shin midpoint, in degrees; 0 = the segment points straight down.  The depth model above treats
            // the shin as vertical (feet_y_gravity subtracts l3/2), so a tilted shin is also its error.
            const ogma::body::Basis3f rot1{ ogma::body::Quatf(ogma::body::Vec3f(0.0f, 1.0f, 0.0f), float(t1)) };
            const ogma::body::Xform3f h1(rot1, body.anchors[size_t(i)].hip1_world - rot1 * body.anchors[size_t(i)].hip1_world);
            const ogma::body::Vec3f hip2_now = h1 * body.anchors[size_t(i)].hip2_world;
            const ogma::body::Basis3f rot2{ ogma::body::Quatf(rot1 * body.anchors[size_t(i)].hip2_axis, float(t2)) };
            const ogma::body::Xform3f h2(rot2, hip2_now - rot2 * hip2_now);
            const ogma::body::Vec3f knee_now = h2 * h1 * body.anchors[size_t(i)].knee_world;
            auto tilt = [](ogma::body::Vec3f a, ogma::body::Vec3f b) {
                const double dx = double(b.x - a.x), dy = double(b.y - a.y), dz = double(b.z - a.z);
                return std::atan2(std::hypot(dx, dz), -dy) * 180.0 / M_PI;
            };
            // The toe in the chassis rest frame (x, z horizontal): the shin midpoint plus half the shin along the
            // knee -> midpoint direction.  For the support polygon (the CoG question, 2026-10-26).
            const auto mid_r  = body.rest_inv(p.lower.origin), knee_r = body.rest_inv(knee_now);
            const ogma::body::Vec3f sh = mid_r - knee_r;
            const double shl = std::sqrt(double(sh.x * sh.x + sh.y * sh.y + sh.z * sh.z));
            const double hk = shl > 1e-6 ? 0.5 * body.l3 / shl : 0.0;
            legs.push_back({d0, g2, g3, reach, t2, t3, tilt(hip2_now, knee_now), tilt(knee_now, p.lower.origin),
                            double(mid_r.x) + hk * double(sh.x), double(mid_r.z) + hk * double(sh.z)});
            const int cnt = j["fsr"][size_t(ogma::hw::brain::kFsrChannelForSimLeg[size_t(i)])].get<int>();
            planted.push_back(cnt >= 200 ? 1 : 0);
            grams.push_back(fsr.grams(cnt));
        }
        std::cout << nlohmann::json{{"t", j.value("t", 0)}, {"legs", legs}, {"planted", planted}, {"grams", grams},
                                    {"tof_m", j.value("tof_m", -1.0)}, {"tof_valid", j.value("tof_valid", false)}}.dump() << "\n";
    }
    return 0;
}
