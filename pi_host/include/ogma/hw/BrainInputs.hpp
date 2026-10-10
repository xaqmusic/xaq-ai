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
#include "ogma/body/Distress.hpp"
#include "ogma/body/DeadReckon.hpp"
#include <algorithm>

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

// A pulse of 0 µs is benchd's "never commanded / limp": the servo is unpowered and its
// angle is UNKNOWN.  hinge_angles_from_us() would turn it into ±2.7 rad, clamped to the
// joints topic's rails — a confident, plausible, wrong posture.  First seen on the robot
// 2026-10-03 (fresh benchd, all channels unarmed).  So joints exist only when every channel
// the map uses carries a real pulse; otherwise the tick is withheld, never guessed.
inline bool all_servos_commanded(const std::array<int, 12>& us_by_channel, const ServoMapping& m) {
    for (int k = 0; k < 12; ++k)
        if (us_by_channel[size_t(m.by_lj[size_t(k)].ch)] <= 0) return false;
    return true;
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

// ---- the per-tick builder ---------------------------------------------------------------
// Everything P-e·h0 reads from outside the graph, computed from what the robot has, in the
// order the sim computes it within a tick (port doc contract; picrawler_body.gd, honest
// branches).  Inputs arrive in the ROBOT's units and are converted here, once:
//   accel_g   — ImuSample::accel_body, in g (×9.81 -> m/s², as Icm20948 itself does);
//   gyro_dps  — ImuSample::gyro_body, in deg/s (-> rad/s);
//   up        — ImuSample::up_fused (body frame, +X left, +Y up, +Z forward = sim axes).
// ⚠ ORDER MATTERS and mirrors the sim:
//   * `imu`[2] and `distress` read the PREVIOUS tick's stride estimate (the sim publishes
//     them before it steps StrideV);
//   * the forward model steps BEFORE `joints` is published, and the toe FK for stride_v
//     uses the same stepped angles.
struct TickInputs {
    std::array<int, 12> us{};          // benchd state feed: commanded pulse per HAT channel
    std::array<int, 4>  fsr{};         // benchd state feed: A0-A3 counts (physical order)
    bool   fsr_ok   = true;
    std::array<float, 3> accel_g{};    // ImuSample::accel_body
    std::array<float, 3> gyro_dps{};   // ImuSample::gyro_body
    std::array<float, 3> up{0, 1, 0};  // ImuSample::up_fused
    double  dt   = 0.02;
    int64_t tick = 0;
};

struct BrainTopics {
    std::array<float, 12> joints{};
    std::array<float, 4>  imu{};
    std::array<float, 3>  gyro{};
    std::array<float, 2>  stride_v{};
    std::array<float, 4>  foot_contact{};
    std::array<float, 4>  foot_load{};
    std::array<float, 12> joint_torque{};   // zeros: hobby servos report no torque
    std::array<float, 4>  feet_y_gravity_cmd_imu{};
    float distress = 0.0f;
    float upright  = 1.0f;
    bool  fsr_stale = false;                // fsr_ok was false: foot values are the last good ones
    // S1 of the MicroDuck port (opt-in at the host: --odom / --vel-ego).  Same layouts as
    // the sim publishes them (picrawler_body.gd: ego_heading, odom, place_in, vel_ego).
    float                ego_heading = 0.0f;  // the gyro's yaw, integrated, UNWRAPPED (rad)
    std::array<float, 3> odom{};              // [x m, y m, yaw rad] in the body frame at the last odom reset
    std::array<float, 4> place_in{};          // [x/L, y/L, cos yaw, sin yaw], the place map's input
    std::array<float, 2> vel_ego{};           // [stride_v.x, stride_v.y]: the honest form
};

class BrainInputBuilder {
public:
    static constexpr double kServoLagAlpha   = 0.2;    // STRIDO_LP_ALPHA (picrawler_body.gd)
    static constexpr double kStanceLoadFrac  = 0.2;    // STRIDE_V_LOAD_THRESH
    static constexpr double kG               = 9.81;
    static constexpr double kDeg2Rad         = 3.14159265358979323846 / 180.0;

    BrainInputBuilder(BodyCalib body, ServoMapping map, FsrModel fsr, double us_per_rad)
        : body_(std::move(body)), map_(std::move(map)), fsr_(std::move(fsr)),
          us_per_rad_(us_per_rad),
          sv_(ogma::body::StrideVParams{1.0, 0.1, 0.05, 0.005, kG}) {}

    BrainTopics step(const TickInputs& in) {
        using ogma::body::Vec3f;
        BrainTopics t;
        const Vec3f up(in.up[0], in.up[1], in.up[2]);
        const Vec3f gyro(float(in.gyro_dps[0] * kDeg2Rad), float(in.gyro_dps[1] * kDeg2Rad),
                         float(in.gyro_dps[2] * kDeg2Rad));
        const Vec3f accel(float(in.accel_g[0] * kG), float(in.accel_g[1] * kG),
                          float(in.accel_g[2] * kG));

        // 1. Ego heading: dead-reckoned yaw about the body's up axis (sim: _ego_heading).
        ego_heading_ = wrap_pi(ego_heading_ + double(gyro.y) * in.dt);
        yaw_unwrapped_ += double(gyro.y) * in.dt;     // the sim's _ego_heading never wraps; nor does this

        // 2. Joints: commanded pulse -> hinge angle -> servo forward model -> sim normalisation.
        const std::array<double, 12> cmd = hinge_angles_from_us(in.us, map_, us_per_rad_);
        if (!lag_.seeded()) lag_.seed(cmd.data());
        lag_.step(cmd.data(), kServoLagAlpha);
        std::array<double, 12> ang{};
        for (int k = 0; k < 12; ++k) ang[size_t(k)] = lag_[k];
        t.joints = joints_topic(ang, body_);

        // 3. imu (honest): ego heading, the PREVIOUS stride estimate, body-up yaw rate.
        auto cl1 = [](double v) { return float(v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v)); };
        t.imu = {float(std::sin(ego_heading_)), float(std::cos(ego_heading_)),
                 cl1(sv_prev_y_), cl1(double(gyro.y) / 3.14159265358979323846)};
        t.gyro = {cl1(double(gyro.x) / 3.14159265358979323846),
                  cl1(double(gyro.y) / 3.14159265358979323846),
                  cl1(double(gyro.z) / 3.14159265358979323846)};

        // 4. distress (honest): odometry along the ego heading × fused tilt; previous estimate.
        const double upy = std::max(-1.0, std::min(1.0, double(up.y)));
        t.distress = float(dist_.step(sv_prev_x_, sv_prev_y_, ego_heading_, std::acos(upy),
                                      in.dt, in.tick));

        // 5. The zero-pose swing-detector input and the fused upright.
        t.feet_y_gravity_cmd_imu = feet_y_gravity_zero_pose(body_, up);
        t.upright = up.y;

        // 6. Foot sensors.  A failed read holds the last good values and says so.
        if (in.fsr_ok) last_fsr_ = in.fsr;
        t.fsr_stale = !in.fsr_ok;
        t.foot_load    = fsr_.loads_sim(last_fsr_);
        t.foot_contact = fsr_.contacts_sim(last_fsr_);

        // 7. stride_v: stance FK on the forward-model angles, feet loaded >= 0.2 of body
        //    weight at BOTH ends of the tick, through the shared StrideV.
        std::array<Vec3f, 4> toe{};
        std::array<bool, 4> loaded{};
        Vec3f stance_sum(0.0f, 0.0f, 0.0f);
        int stance_n = 0;
        for (int i = 0; i < 4; ++i) {
            toe[size_t(i)] = body_.toe_body(i, ang[size_t(i * 3 + 0)], ang[size_t(i * 3 + 1)],
                                            ang[size_t(i * 3 + 2)]);
            loaded[size_t(i)] = double(t.foot_load[size_t(i)]) >= kStanceLoadFrac;
            if (prev_valid_ && loaded[size_t(i)] && prev_loaded_[size_t(i)]) {
                stance_sum = stance_sum + ogma::body::planted_foot_velocity(
                                              toe[size_t(i)], prev_toe_[size_t(i)], gyro, in.dt);
                ++stance_n;
            }
        }
        prev_toe_ = toe; prev_loaded_ = loaded; prev_valid_ = true;
        sv_.step(sv_.linear_accel(accel, up), stance_sum, stance_n, in.dt);
        t.stride_v = {sv_.est().x, sv_.est().y};
        sv_prev_x_ = sv_.est().x;
        sv_prev_y_ = sv_.est().y;

        // 8. Odometry (ogma::body::DeadReckon, the sim's integrator): stride_v under the
        //    unwrapped heading, anchored at the last odom reset.  And vel_ego's honest form.
        odom_.step(double(sv_.est().x), double(sv_.est().y), yaw_unwrapped_ - yaw_anchor_, in.dt);
        t.ego_heading = float(yaw_unwrapped_);
        t.odom = {float(odom_.x()), float(odom_.y()), float(odom_.yaw())};
        auto cl11 = [](double v) { return float(v < -1.1 ? -1.1 : (v > 1.1 ? 1.1 : v)); };
        t.place_in = {cl11(odom_.x() / odom_scale_), cl11(odom_.y() / odom_scale_),
                      float(std::cos(odom_.yaw())), float(std::sin(odom_.yaw()))};
        t.vel_ego = {sv_.est().x, sv_.est().y};
        return t;
    }

    // place_in's x/y normaliser (the room's half-size plus a margin; the sim's odom_scale_m).
    void set_odom_scale(double s) { odom_scale_ = s > 0.1 ? s : 0.1; }
    // Re-anchor the odom frame at the current pose (the sim does this on a hard reset).
    void reset_odom() { odom_.reset(); yaw_anchor_ = yaw_unwrapped_; }

    double ego_heading() const { return ego_heading_; }

private:
    static double wrap_pi(double a) {
        const double tp = 2.0 * 3.14159265358979323846;
        a = std::fmod(a + 3.14159265358979323846, tp);
        if (a < 0.0) a += tp;
        return a - 3.14159265358979323846;
    }
    BodyCalib    body_;
    ServoMapping map_;
    FsrModel     fsr_;
    double       us_per_rad_;
    ogma::body::ServoForwardModel   lag_;
    ogma::body::StrideV             sv_;
    ogma::body::DistressAccumulator dist_;
    double ego_heading_ = 0.0, sv_prev_x_ = 0.0, sv_prev_y_ = 0.0;
    double yaw_unwrapped_ = 0.0, yaw_anchor_ = 0.0, odom_scale_ = 2.0;
    ogma::body::DeadReckon odom_;
    std::array<int, 4> last_fsr_{};
    std::array<ogma::body::Vec3f, 4> prev_toe_{};
    std::array<bool, 4> prev_loaded_{};
    bool prev_valid_ = false;
};

}  // namespace ogma::hw::brain
