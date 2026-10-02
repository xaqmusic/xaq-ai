#pragma once
// BrainInputs — the robot side of the P-e brain input contract (port doc, "Brain input
// contract for P-e"), as pure functions over calibration data.  No bus, no threads, no
// ogma_core: ogma_host calls these, and test_hw checks them on any machine.
//
// What each one answers, and which sim formula it must reproduce:
//   * ServoMapping + hinge_angles_from_us   benchd's commanded pulse -> the sim's hinge angle
//                                           (the servo map's sign/origin, at the MEASURED
//                                           µs/rad).  Feeds the shared ServoForwardModel, whose
//                                           output is the sim's honest_joints `joints`.
//   * joints_topic                          the sim's `joints` normalisation, joint-major.
//   * FsrModel                              counts -> foot_contact / foot_load, in the SIM's leg
//                                           order (the legs are mirrored: sim fl = physical FR).
//   * BodyCalib (+ toe_body, feet_y_zero)   the FK anchors the sim exported
//                                           (scripts_tools/export_body_calib.gd), so
//                                           ogma::body::fk_leg is the same maths on both sides.
//
// ⚠ ORDER CONVENTIONS, because both are easy to get backwards:
//   * per-leg arrays are SIM leg order: 0 fl, 1 fr, 2 rl, 3 rr (sim names; mirrored);
//   * angle arrays are leg*3 + joint, joint 0 hip1, 1 hip2, 2 knee (the forward model's
//     index); the `joints` TOPIC is joint-major (hip1 x4, hip2 x4, knee x4).
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "ogma/body/GodotFloat.hpp"
#include "ogma/body/LegKinematics.hpp"
#include "ogma/body/StrideOdometry.hpp"

namespace ogma::hw::brain {

inline int sim_leg_index(const std::string& n) {
    if (n == "fl") return 0;
    if (n == "fr") return 1;
    if (n == "rl") return 2;
    if (n == "rr") return 3;
    return -1;
}
inline int joint_index(const std::string& n) {
    if (n == "hip1") return 0;
    if (n == "hip2") return 1;
    if (n == "knee") return 2;
    return -1;
}

// ---- servo map: which HAT channel drives which sim (leg, joint), and its calibration ----
struct ServoChannel {
    int    ch = -1;
    int    sign = 1;
    double origin_us = 1500.0;
};

struct ServoMapping {
    std::array<ServoChannel, 12> by_lj{};   // index leg*3 + joint
    bool complete = false;                  // all 12 (leg, joint) slots found exactly once
    std::string why;                        // what is missing, when not complete

    static ServoMapping from_json(const nlohmann::json& j) {
        ServoMapping m;
        std::array<int, 12> seen{};
        if (!j.contains("servos") || !j["servos"].is_array()) { m.why = "no servos array"; return m; }
        for (const auto& s : j["servos"]) {
            const int leg = sim_leg_index(s.value("sim_leg", std::string()));
            const int jt  = joint_index(s.value("joint", std::string()));
            if (leg < 0 || jt < 0) continue;
            const int k = leg * 3 + jt;
            m.by_lj[size_t(k)] = ServoChannel{s.value("ch", -1), s.value("sign", 1),
                                              s.value("origin_us", 1500.0)};
            ++seen[size_t(k)];
        }
        m.complete = true;
        for (int k = 0; k < 12; ++k)
            if (seen[size_t(k)] != 1 || m.by_lj[size_t(k)].ch < 0 || m.by_lj[size_t(k)].ch > 11) {
                m.complete = false;
                m.why += "slot " + std::to_string(k) + " seen " + std::to_string(seen[size_t(k)]) + "x; ";
            }
        return m;
    }
    static ServoMapping load(const std::string& path) {
        std::ifstream f(path);
        if (!f) { ServoMapping m; m.why = "cannot read " + path; return m; }
        return from_json(nlohmann::json::parse(f));
    }
};

// The commanded hinge angle, in the sim's convention, from each channel's commanded pulse.
// angle = sign · (us − origin_us) / us_per_rad — the dashboard's mirror formula, at the
// MEASURED scale (545.2 µs/rad, sensors.json), not the 636.6 hobby-servo standard.
inline std::array<double, 12> hinge_angles_from_us(const std::array<int, 12>& us_by_channel,
                                                   const ServoMapping& m, double us_per_rad) {
    std::array<double, 12> a{};
    for (int k = 0; k < 12; ++k) {
        const ServoChannel& c = m.by_lj[size_t(k)];
        a[size_t(k)] = double(c.sign) * (double(us_by_channel[size_t(c.ch)]) - c.origin_us) / us_per_rad;
    }
    return a;
}

// ---- body calibration exported by the sim --------------------------------------------
struct BodyCalib {
    std::string geometry;
    double l3 = 0.0, knee_rest = 0.0, hip1_limit = 1.4, hip2_limit = 1.4, total_mass_kg = 0.0;
    ogma::body::Vec3f rest_origin;                     // rest_inv(v) = v - rest_origin
    std::array<ogma::body::LegAnchors, 4> anchors{};
    std::array<ogma::body::Vec3f, 4> toe_off{};
    std::array<ogma::body::Vec3f, 4> foot_b_zero{};    // the sim's own FK at zero angles
    bool ok = false;
    std::string why;

    static ogma::body::Vec3f v3(const nlohmann::json& a) {
        return ogma::body::Vec3f(a.at(0).get<float>(), a.at(1).get<float>(), a.at(2).get<float>());
    }
    static BodyCalib from_json(const nlohmann::json& j) {
        BodyCalib b;
        try {
            b.geometry      = j.value("geometry", std::string());
            b.l3            = j.at("l3").get<double>();
            b.knee_rest     = j.at("knee_rest").get<double>();
            b.hip1_limit    = j.at("hip1_limit").get<double>();
            b.hip2_limit    = j.at("hip2_limit").get<double>();
            b.total_mass_kg = j.at("total_mass_kg").get<double>();
            b.rest_origin   = v3(j.at("chassis_rest_origin"));
            // rest_inv is a pure translation ONLY if the rest basis is identity.  The sim
            // spawns level, so it is; refuse anything else rather than apply wrong maths.
            const auto& B = j.at("chassis_rest_basis");
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    if (std::fabs(B.at(r).at(c).get<double>() - (r == c ? 1.0 : 0.0)) > 1e-6) {
                        b.why = "chassis_rest_basis is not identity"; return b;
                    }
            const auto& legs = j.at("legs");
            if (!legs.is_array() || legs.size() != 4) { b.why = "need 4 legs"; return b; }
            for (size_t i = 0; i < 4; ++i) {
                const auto& L = legs[i];
                if (sim_leg_index(L.value("name", std::string())) != int(i)) {
                    b.why = "legs out of sim order (fl, fr, rl, rr)"; return b;
                }
                auto& a = b.anchors[i];
                a.hip1_world        = v3(L.at("hip1_world"));
                a.hip2_world        = v3(L.at("hip2_world"));
                a.knee_world        = v3(L.at("knee_world"));
                a.coxa_rest_origin  = v3(L.at("coxa_rest_origin"));
                a.upper_rest_origin = v3(L.at("upper_rest_origin"));
                a.lower_rest_origin = v3(L.at("lower_rest_origin"));
                a.hip2_axis         = v3(L.at("hip2_axis"));
                a.knee_axis         = v3(L.at("knee_axis"));
                b.toe_off[i]        = v3(L.at("toe_off_c"));
                b.foot_b_zero[i]    = v3(L.at("foot_b_zero"));
            }
            b.ok = true;
        } catch (const std::exception& e) {
            b.why = e.what();
        }
        return b;
    }
    static BodyCalib load(const std::string& path) {
        std::ifstream f(path);
        if (!f) { BodyCalib b; b.why = "cannot read " + path; return b; }
        return from_json(nlohmann::json::parse(f));
    }

    ogma::body::Vec3f rest_inv(const ogma::body::Vec3f& v) const { return v - rest_origin; }

    // Body-frame toe position from hinge angles — the sim's toe_cmdlp_b:
    // rest_inv * (fk_leg(...).lower * toe_off_c).  Feeds StrideV's stance velocity.
    ogma::body::Vec3f toe_body(int leg, double t1, double t2, double t3) const {
        const ogma::body::LegPose p = ogma::body::fk_leg(anchors[size_t(leg)], t1, t2, t3, 0.0);
        return rest_inv(p.lower * toe_off[size_t(leg)]);
    }
};

// The sim's `joints` normalisation (picrawler_body.gd, the honest_joints branch), from
// angles indexed leg*3 + joint to the topic's joint-major order.  Clamped to ±1, as the
// sim clamps.  Note the knee saturates above KNEE_REST + 1 rad, exactly as in the sim.
inline std::array<float, 12> joints_topic(const std::array<double, 12>& a, const BodyCalib& b) {
    auto cl = [](double v) { return float(v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v)); };
    std::array<float, 12> out{};
    for (int leg = 0; leg < 4; ++leg) {
        out[size_t(0 * 4 + leg)] = cl(a[size_t(leg * 3 + 0)] / b.hip1_limit);
        out[size_t(1 * 4 + leg)] = cl(a[size_t(leg * 3 + 1)] / b.hip2_limit);
        out[size_t(2 * 4 + leg)] = cl(a[size_t(leg * 3 + 2)] - b.knee_rest);
    }
    return out;
}

// `feet_y_gravity_cmd_imu` exactly as P-e computes it: FK of the ZERO pose (servo_targets
// stays 0 in brain mode — ledger 2026-10-02) along the fused up vector.  Parity with a known
// quirk, on purpose: the robot must feed the brain what the validated sim fed it.  The
// constant is the sim's own export (foot_b_zero), not a re-derivation.
inline std::array<float, 4> feet_y_gravity_zero_pose(const BodyCalib& b, const ogma::body::Vec3f& up) {
    std::array<float, 4> out{};
    for (int i = 0; i < 4; ++i)
        out[size_t(i)] = float(ogma::body::feet_y_gravity(b.foot_b_zero[size_t(i)], up, b.l3));
    return out;
}

// ---- foot sensors ----------------------------------------------------------------------
// The FSRs sit on HAT ADC A0-A3 = PHYSICAL FL, FR, RL, RR (foot_cal_sweep.py / foot_tap_test.py).
// The sim's legs are mirrored, so in SIM order: fl <- A1, fr <- A0, rl <- A3, rr <- A2.
constexpr std::array<int, 4> kFsrChannelForSimLeg = {1, 0, 3, 2};

struct FsrModel {
    // Counts -> grams, piecewise linear through the as-built divider's predicted curve
    // (wiring doc §5.7, R_g 15 k).  ⚠ Below the 30 g point the curve is UNMEASURED; it is
    // interpolated from the in-air floor.  No consumer reads magnitude (§5.7.12): the
    // decisions are foot_load >= 0.05 of body weight (GainEvolver) and >= 0.2 (stride stance).
    std::vector<std::pair<double, double>> curve = {
        {60, 0}, {1536, 30}, {1920, 50}, {2118, 100}, {2362, 175}, {2603, 300}, {2925, 500}};
    // Counts at or above which the foot counts as touching (foot_contact).  The sim's
    // contact is ANY physics touch, so this sits just above in-air noise (0-60 counts) and
    // below a light touch (~1000, §5.7.6).  ⚠ Not yet validated on the bench.
    int    contact_counts = 200;
    double body_mass_g    = 598.0;

    double grams(int counts) const {
        if (counts <= curve.front().first) return 0.0;
        for (size_t i = 1; i < curve.size(); ++i)
            if (counts <= curve[i].first) {
                const auto& p = curve[i - 1]; const auto& q = curve[i];
                return p.second + (q.second - p.second) * (counts - p.first) / (q.first - p.first);
            }
        const auto& p = curve[curve.size() - 2]; const auto& q = curve.back();   // extrapolate
        return q.second + (q.second - p.second) * (counts - q.first) / (q.first - p.first);
    }
    // foot_load as the sim defines it: the fraction of body weight on that foot, ±2.
    float load(int counts) const {
        const double v = grams(counts) / body_mass_g;
        return float(v < -2.0 ? -2.0 : (v > 2.0 ? 2.0 : v));
    }
    float contact(int counts) const { return counts >= contact_counts ? 1.0f : 0.0f; }

    // fsr[4] in PHYSICAL channel order A0-A3 -> SIM leg order.
    std::array<float, 4> loads_sim(const std::array<int, 4>& fsr) const {
        std::array<float, 4> o{};
        for (int i = 0; i < 4; ++i) o[size_t(i)] = load(fsr[size_t(kFsrChannelForSimLeg[size_t(i)])]);
        return o;
    }
    std::array<float, 4> contacts_sim(const std::array<int, 4>& fsr) const {
        std::array<float, 4> o{};
        for (int i = 0; i < 4; ++i) o[size_t(i)] = contact(fsr[size_t(kFsrChannelForSimLeg[size_t(i)])]);
        return o;
    }
};

}  // namespace ogma::hw::brain
