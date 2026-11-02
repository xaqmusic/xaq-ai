// EncounterFeatures: one vector per touch, from the half second after a toe meets something.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/EncounterFeatures.hpp"
namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::EncounterFeatures m; int published = 0; std::vector<float> last;
    Fix(ogma::ParamMap p = {}) {
        m.set_id("enc"); m.on_setup(&bus, p);
        bus.subscribe("sense.touch_encounter", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){
            auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); ++published; last.clear();
            for (int i = 0; i < t->values.size(); ++i) last.push_back(t->values[i]); });
    }
    void run(uint64_t t, std::vector<float> contact, std::vector<float> xz, std::vector<float> y, float cur, float belly, float upright, std::vector<float> load = {0, 0, 0, 0}) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.foot_contact", pv(contact)); bus.publish("reality.proprio.feet_xz", pv(xz));
        bus.publish("reality.proprio.feet_y_gravity_cmd_imu", pv(y)); bus.publish("reality.proprio.foot_load", pv(load));
        bus.publish("sense.servo_current", pv({cur})); bus.publish("reality.proprio.belly_m", pv({belly})); bus.publish("reality.proprio.upright", pv({upright}));
        m.tick(t); bus.end_tick();
    }
};
const std::vector<float> kXZ = {0.1f, 0.1f, -0.1f, 0.1f, 0.1f, -0.1f, -0.1f, -0.1f};
}

TEST(EncounterFeatures, ABoxTopPressReadsHeightEffortAndLift) {
    Fix f({{"post_ticks", ogma::ParamValue{int64_t{5}}}, {"pre_ticks", ogma::ParamValue{int64_t{5}}}});
    // three planted on the floor at -0.09, leg 3 descending vertically toward a 0.05 box top over 10 ticks
    uint64_t t = 0;
    for (int k = 0; k < 10; ++k, ++t) f.run(t, {1, 1, 1, 0}, kXZ, {-0.09f, -0.09f, -0.09f, -0.09f + 0.05f + 0.03f - 0.003f * k}, 0.10f, 0.000f, 1.0f);
    // onset on the box top (0.05 over the plane); then the press: current climbs 0.4 A, belly lifts 8 mm, the foot loads
    for (int k = 0; k < 6; ++k, ++t)
        f.run(t, {1, 1, 1, 1}, kXZ, {-0.09f, -0.09f, -0.09f, -0.04f}, 0.10f + 0.08f * k, 0.0016f * k, std::cos(0.02f * k), {0, 0, 0, 0.1f * k});
    ASSERT_EQ(f.published, 1); ASSERT_EQ(f.last.size(), 10u);
    EXPECT_NEAR(f.last[0], -0.04 + 0.029, 1e-5) << "height over the ToF floor (belly 0 -> floor at -0.029): the toe at -0.04";
    EXPECT_NEAR(f.last[1], -0.1, 1e-6); EXPECT_NEAR(f.last[2], -0.1, 1e-6);
    EXPECT_GT(f.last[3], 0.3f) << "load peak";
    EXPECT_NEAR(f.last[4], 0.0, 1e-6) << "no current rise before onset: a top, not a face";
    EXPECT_GT(f.last[5], 0.3f) << "the press's current";
    EXPECT_GT(f.last[6], 0.005f) << "belly lift";
    EXPECT_GT(f.last[7], 0.05f) << "lean rise";
    EXPECT_FLOAT_EQ(f.last[8], -1.0f) << "no range published";
    EXPECT_GT(f.last[9], 0.95f) << "a vertical approach";
    EXPECT_EQ(f.m.last_leg(), 3);
}

TEST(EncounterFeatures, AHorizontalAdvanceReadsAsSuch_AndAHeldContactIsOneEvent) {
    Fix f({{"post_ticks", ogma::ParamValue{int64_t{3}}}, {"pre_ticks", ogma::ParamValue{int64_t{5}}}});
    uint64_t t = 0;
    std::vector<float> xz = kXZ;
    for (int k = 0; k < 10; ++k, ++t) { xz[7] = -0.1f + 0.01f * k; f.run(t, {1, 1, 1, 0}, xz, {-0.09f, -0.09f, -0.09f, -0.06f}, 0.1f, 0.0f, 1.0f); }
    for (int k = 0; k < 8; ++k, ++t) f.run(t, {1, 1, 1, 1}, xz, {-0.09f, -0.09f, -0.09f, -0.06f}, 0.1f, 0.0f, 1.0f);
    EXPECT_EQ(f.published, 1) << "the held contact is one event";
    EXPECT_LT(f.last[9], 0.05f) << "a sideways approach";
    EXPECT_NEAR(f.last[0], -0.06 + 0.029, 1e-5);
}

TEST(EncounterFeatures, NoInputsNoEventsNoCrash) {
    Fix f; f.bus.begin_tick(0); f.m.tick(0); f.bus.end_tick();
    EXPECT_EQ(f.m.events(), 0); EXPECT_EQ(f.published, 0);
}
