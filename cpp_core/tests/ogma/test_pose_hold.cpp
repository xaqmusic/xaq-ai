// MotorEPMv2's pose hold for the amplitude homeostat (homeo_pose_hold, 2026-11-18): while a planner owns the legs through the
// pose socket the gait's own oscillation is not the body's motion, and the regulator winds its gain to the ceiling against a
// target it cannot reach (measured in the simulator: 0.1 -> 5.0 over a four-minute sequencer bout, the body flipped at the
// handback in 2 of 4 seeds).  With the hold on the gain is held where it stood; with the gait owning the legs the hold changes
// nothing.
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/MotorEPMv2.hpp"
#include <nlohmann/json.hpp>
namespace {
using ogma::ParamMap;
constexpr int kLegs = 1, kMotors = 3, kStateN = 9;
ParamMap base_params() {
    ParamMap p;
    p["n_legs"] = int64_t{kLegs}; p["motor_dim"] = int64_t{kMotors}; p["seed"] = int64_t{1234}; p["babble_ticks"] = int64_t{12};
    p["explore_noise"] = 0.0; p["proprio_topics"] = std::vector<std::string>{"sp.p0"}; p["action_topics"] = std::vector<std::string>{"sp.a0", "sp.a1", "sp.a2"};
    p["imu_topic"] = std::string("sp.imu"); p["contact_topic"] = std::string("sp.contact"); p["pose_topic"] = std::string("sp.pose");
    for (auto k : {"coupling_gain", "coord_reward_drive", "amp_seek_rate", "stroke_gain", "balance_gain", "heading_gain", "nav_gain", "postural_gain", "height_homeo_gain", "panic_strength"}) p[k] = 0.0;
    p["amp_homeo_gain"] = 0.01; p["amp_target"] = 0.15;
    return p;
}
std::shared_ptr<ogma::ProprioToken> tok(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values = Eigen::VectorXf(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
struct Fixture {
    ogma::InProcessBus bus; ogma::MotorEPMv2 m;
    explicit Fixture(ParamMap const& p) { m.set_id("motor_epm_v2"); m.on_setup(&bus, p); }
    // moving: the joints oscillate (the gait's own motion reaches the body); still: they stand (a planner holds a pose)
    void run_tick(uint64_t t, bool moving, float engage) {
        bus.begin_tick(t);
        auto pt = std::make_shared<ogma::ProprioToken>(); pt->values = Eigen::VectorXf::Zero(kStateN); pt->sensor = "proprio";
        const double ph = 0.15 * double(t);
        if (moving) for (int j = 0; j < kMotors; ++j) { pt->values[3 * j] = float(0.30 * std::sin(ph + j)); pt->values[3 * j + 1] = float(0.20 * std::cos(ph + j)); pt->values[3 * j + 2] = float(0.045 * std::cos(ph + j)); }
        bus.publish("sp.p0", pt);
        bus.publish("sp.contact", tok({1.0f}));
        bus.publish("sp.pose", tok({engage, 0.0f, 0.0f, 0.0f}));
        m.tick(t); bus.end_tick();
    }
    float accel(int j) const { auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(bus.last_value("sp.a" + std::to_string(j))); return a ? a->accel : std::numeric_limits<float>::quiet_NaN(); }
    float gain() const { const auto d = m.snapshot_state(); return d.at("module").at("amp_gain_mean").get<float>(); }
};
}

TEST(PoseHold, WhileAPlannerOwnsTheLegsTheGainIsHeld_WithoutTheHoldItWinds) {
    auto off = base_params(); auto on = off; on["homeo_pose_hold"] = 1.0;
    Fixture a(off), b(on);
    for (uint64_t t = 0; t < 300; ++t) { a.run_tick(t, true, 0.0f); b.run_tick(t, true, 0.0f); }
    const float g0a = a.gain(), g0b = b.gain();
    EXPECT_FLOAT_EQ(g0a, g0b) << "the gait owning the legs, the hold changes nothing";
    for (uint64_t t = 300; t < 1500; ++t) { a.run_tick(t, false, 1.0f); b.run_tick(t, false, 1.0f); }
    EXPECT_GT(a.gain(), g0a + 0.5f) << "without the hold a standing body winds the regulator up";
    EXPECT_NEAR(b.gain(), g0b, 5e-3f) << "with it the gain is where it stood when the planner took the legs (the tick or two before the first pose token arrives aside)";
    // handed back: the regulator works again
    for (uint64_t t = 1500; t < 1800; ++t) b.run_tick(t, false, 0.0f);
    EXPECT_GT(b.gain(), g0b + 0.1f) << "the gait owning the legs again, the regulator integrates again";
}

TEST(PoseHold, WithTheGaitOwningTheLegsTheCommandsAreTheSameToTheBit) {
    auto off = base_params(); auto on = off; on["homeo_pose_hold"] = 1.0;
    Fixture a(off), b(on);
    double maxdiff = 0.0;
    for (uint64_t t = 0; t < 400; ++t) {
        a.run_tick(t, true, 0.0f); b.run_tick(t, true, 0.0f);
        for (int j = 0; j < kMotors; ++j) maxdiff = std::max(maxdiff, double(std::abs(a.accel(j) - b.accel(j))));
    }
    EXPECT_EQ(maxdiff, 0.0);
    EXPECT_EQ(a.gain(), b.gain());
}

TEST(PoseHold, AHalfShareLearnsAtHalfTheRate) {
    auto off = base_params(); auto on = off; on["homeo_pose_hold"] = 1.0;
    Fixture a(off), b(on);
    for (uint64_t t = 0; t < 300; ++t) { a.run_tick(t, true, 0.0f); b.run_tick(t, true, 0.0f); }
    for (uint64_t t = 300; t < 700; ++t) { a.run_tick(t, false, 0.5f); b.run_tick(t, false, 0.5f); }   // the amplitude estimate settles on the standing body
    const float ga = a.gain(), gb = b.gain();
    for (uint64_t t = 700; t < 1000; ++t) { a.run_tick(t, false, 0.5f); b.run_tick(t, false, 0.5f); }
    const float da = a.gain() - ga, db = b.gain() - gb;
    ASSERT_GT(da, 0.05f) << "the full-rate regulator is winding";
    EXPECT_GT(db, 0.35f * da); EXPECT_LT(db, 0.65f * da);
}
