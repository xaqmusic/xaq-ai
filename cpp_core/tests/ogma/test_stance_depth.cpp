// MotorEPMv2's stance depth handle (2026-11-08): the socket wired with the direct gain at 0 (and no learned press) is
// byte-identical; with the gain on, a PLANTED leg's hip2 and knee move in the requested sense, bounded, and an
// airborne leg is left alone.
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/MotorEPMv2.hpp"
#include <nlohmann/json.hpp>
namespace {
using ogma::ParamMap; using ogma::ParamValue;
constexpr int kLegs = 1, kMotors = 3, kStateN = 9;
ParamMap base_params() {
    ParamMap p;
    p["n_legs"] = int64_t{kLegs}; p["motor_dim"] = int64_t{kMotors}; p["seed"] = int64_t{1234}; p["babble_ticks"] = int64_t{12};
    p["explore_noise"] = 0.0; p["proprio_topics"] = std::vector<std::string>{"sp.p0"}; p["action_topics"] = std::vector<std::string>{"sp.a0", "sp.a1", "sp.a2"};
    p["imu_topic"] = std::string("sp.imu"); p["contact_topic"] = std::string("sp.contact"); p["jac_topic"] = std::string("sp.jac");
    for (auto k : {"coupling_gain", "coord_reward_drive", "amp_seek_rate", "stroke_gain", "balance_gain", "heading_gain", "nav_gain", "postural_gain", "height_homeo_gain", "panic_strength"}) p[k] = 0.0;
    return p;
}
std::shared_ptr<ogma::ProprioToken> tok(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values = Eigen::VectorXf(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
struct Fixture {
    ogma::InProcessBus bus; ogma::MotorEPMv2 m;
    explicit Fixture(ParamMap const& p) { m.set_id("motor_epm_v2"); m.on_setup(&bus, p); }
    void run_tick(uint64_t t, float contact, std::vector<float> const* req) {
        bus.begin_tick(t);
        auto pt = std::make_shared<ogma::ProprioToken>(); pt->values = Eigen::VectorXf::Zero(kStateN); pt->sensor = "proprio";
        const double ph = 0.15 * double(t);
        for (int j = 0; j < kMotors; ++j) { pt->values[3 * j] = float(0.30 * std::sin(ph + j)); pt->values[3 * j + 1] = float(0.20 * std::cos(ph + j)); pt->values[3 * j + 2] = float(0.045 * std::cos(ph + j)); }
        bus.publish("sp.p0", pt);
        bus.publish("sp.contact", tok({contact}));
        bus.publish("sp.jac", tok({0.10f, 0.05f, 0.10f, 0.05f, 0.10f, 0.05f, 0.10f, 0.05f}));   // metres of depth per radian, hip2 and knee
        if (req) bus.publish("sp.stance", tok(*req));
        m.tick(t); bus.end_tick();
    }
    float accel(int j) const { auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(bus.last_value("sp.a" + std::to_string(j))); return a ? a->accel : std::numeric_limits<float>::quiet_NaN(); }
};
}

TEST(StanceDepth, WiredWithGainZeroIsByteIdentical_AndGainOnMovesThePlantedLegTheWayAsked) {
    auto base = base_params(); auto wired = base; wired["stance_depth_topic"] = std::string("sp.stance");
    auto on = wired; on["stance_depth_gain"] = 1.0;
    Fixture a(base), b(wired), c(on), d(on);
    const std::vector<float> down = {-0.02f, 1.0f}, up = {0.02f, 1.0f};
    double maxdiff_ab = 0.0, diff_c = 0.0; float hip2_c = 0.0f, hip2_d = 0.0f;
    for (uint64_t t = 0; t < 150; ++t) {
        a.run_tick(t, 1.0f, nullptr); b.run_tick(t, 1.0f, &down); c.run_tick(t, 1.0f, &down); d.run_tick(t, 1.0f, &up);
        if (t < 30) continue;
        for (int j = 0; j < kMotors; ++j) { maxdiff_ab = std::max(maxdiff_ab, double(std::abs(a.accel(j) - b.accel(j)))); diff_c = std::max(diff_c, double(std::abs(c.accel(j) - a.accel(j)))); }
        hip2_c += c.accel(1) - a.accel(1); hip2_d += d.accel(1) - a.accel(1);
    }
    EXPECT_LT(maxdiff_ab, 1e-12) << "the socket with the gain at 0 and no learned press must change nothing";
    EXPECT_GT(diff_c, 1e-4) << "the handle must move the planted leg";
    EXPECT_LT(hip2_c, 0.0f) << "a shallower request lowers hip2 (the corner comes down)";
    EXPECT_GT(hip2_d, 0.0f) << "a deeper request raises it";
    auto dj = c.m.diag_snapshot();
    ASSERT_TRUE(dj.contains("stance_depth_u")); EXPECT_LT(dj["stance_depth_u"][0].get<double>(), 0.0); EXPECT_GE(dj["stance_depth_u"][0].get<double>(), -0.3);
}

TEST(StanceDepth, AnAirborneLegIsLeftAlone_AndTheBoundHolds) {
    auto base = base_params(); auto on = base; on["stance_depth_topic"] = std::string("sp.stance"); on["stance_depth_gain"] = 1.0; on["stance_depth_max_u"] = 0.05;
    Fixture a(base), c(on);
    const std::vector<float> huge = {-1.0f, 1.0f};
    double diff_air = 0.0, diff_planted = 0.0;
    for (uint64_t t = 0; t < 150; ++t) {
        const float contact = t < 90 ? 0.0f : 1.0f;   // airborne first, then planted
        a.run_tick(t, contact, nullptr); c.run_tick(t, contact, &huge);
        if (t < 30) continue;
        for (int j = 0; j < kMotors; ++j) { const double d = std::abs(double(c.accel(j) - a.accel(j))); if (t < 90) diff_air = std::max(diff_air, d); else diff_planted = std::max(diff_planted, d); }
    }
    EXPECT_LT(diff_air, 1e-12) << "an airborne leg takes no stance request";
    EXPECT_GT(diff_planted, 1e-4);
    auto dj = c.m.diag_snapshot();
    EXPECT_NEAR(dj["stance_depth_u"][0].get<double>(), -0.05, 1e-6) << "a metre of request is bounded to max_u";
}
