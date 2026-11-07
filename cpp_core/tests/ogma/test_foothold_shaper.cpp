// FootholdShaper: per-leg swing targets from the foothold prediction.
#include <gtest/gtest.h>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/FootholdShaper.hpp"
namespace {
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::FootholdShaper m; std::vector<float> out;
    Fix(ogma::ParamMap p = {}) {
        m.set_id("foothold"); m.on_setup(&bus, p);
        bus.subscribe("swing.targets", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); out.assign(t->values.data(), t->values.data() + t->values.size()); });
    }
    // predictions per leg: {h, prec, has, fwd, lat, spread}
    void run(uint64_t t, std::vector<std::vector<float>> pred, std::vector<float> contact, std::vector<float> y) {
        std::vector<float> pv24; for (auto const& p : pred) pv24.insert(pv24.end(), p.begin(), p.end());
        bus.begin_tick(t); bus.publish("predict.foothold", pv(pv24)); bus.publish("reality.proprio.foot_contact", pv(contact)); bus.publish("reality.proprio.toe_y", pv(y)); m.tick(t); bus.end_tick();
    }
};
const std::vector<float> kFlat = {-0.09f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.0f};
}

TEST(FootholdShaper, GainsZeroPublishUnitLiftAndNoReach) {
    Fix f; f.run(1, {{-0.04f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.06f}, kFlat, kFlat, kFlat}, {0, 1, 1, 1}, {-0.09f, -0.09f, -0.09f, -0.09f});
    ASSERT_EQ(f.out.size(), 8u);
    for (int i = 0; i < 4; ++i) { EXPECT_FLOAT_EQ(f.out[size_t(2 * i)], 1.0f); EXPECT_FLOAT_EQ(f.out[size_t(2 * i + 1)], 0.0f); }
}

TEST(FootholdShaper, APredictedRiseLiftsThatLegOnly_InProportionAndCapped) {
    Fix f({{"lift_gain", 20.0}, {"lift_max", 2.0}});
    // every toe last stood on the floor (-0.09); leg 0 will meet ground 5 cm above where it stood: lift 1 + 20 x 0.05 = 2.0 (the cap)
    f.run(0, {kFlat, kFlat, kFlat, kFlat}, {1, 1, 1, 1}, {-0.09f, -0.09f, -0.09f, -0.09f});
    f.run(1, {{-0.04f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.0f}, kFlat, kFlat, kFlat}, {0, 1, 1, 1}, {-0.05f, -0.09f, -0.09f, -0.09f});
    EXPECT_FLOAT_EQ(f.m.lift_mult(0), 2.0f); EXPECT_FLOAT_EQ(f.m.lift_mult(1), 1.0f); EXPECT_FLOAT_EQ(f.m.lift_mult(2), 1.0f);
    // a 2 cm rise: 1.4; a thin memory (precision a quarter of prec_full) asks for a quarter of it
    f.run(2, {{-0.07f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.0f}, kFlat, kFlat, kFlat}, {0, 1, 1, 1}, {-0.05f, -0.09f, -0.09f, -0.09f});
    EXPECT_NEAR(f.m.lift_mult(0), 1.4f, 1e-5);
    f.run(3, {{-0.07f, 500.0f, 1.0f, 0.1f, 0.1f, 0.0f}, kFlat, kFlat, kFlat}, {0, 1, 1, 1}, {-0.05f, -0.09f, -0.09f, -0.09f});
    EXPECT_NEAR(f.m.lift_mult(0), 1.1f, 1e-5);
    // a predicted drop lifts nothing
    f.run(4, {{-0.14f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.0f}, kFlat, kFlat, kFlat}, {0, 1, 1, 1}, {-0.05f, -0.09f, -0.09f, -0.09f});
    EXPECT_FLOAT_EQ(f.m.lift_mult(0), 1.0f);
}

TEST(FootholdShaper, AnEdgeLandingIsMovedForwardOntoTheTread) {
    Fix f({{"edge_shift_m", 0.04}});
    f.run(0, {kFlat, kFlat, kFlat, kFlat}, {1, 1, 1, 1}, {-0.09f, -0.09f, -0.09f, -0.09f});
    f.run(1, {{-0.065f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.05f}, kFlat, kFlat, kFlat}, {0, 1, 1, 1}, {-0.05f, -0.09f, -0.09f, -0.09f});
    EXPECT_FLOAT_EQ(f.m.reach_m(0), 0.04f) << "its prediction spans a riser and rises from where it stood: an up-edge";
    // the same spread with no rise (a down-edge or noise): no shift
    f.run(2, {{-0.11f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.05f}, kFlat, kFlat, kFlat}, {0, 1, 1, 1}, {-0.05f, -0.09f, -0.09f, -0.09f});
    EXPECT_FLOAT_EQ(f.m.reach_m(0), 0.0f);
    f.run(3, {{-0.065f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.05f}, kFlat, kFlat, kFlat}, {0, 1, 1, 1}, {-0.05f, -0.09f, -0.09f, -0.09f});
    EXPECT_FLOAT_EQ(f.m.reach_m(1), 0.0f);
    EXPECT_FLOAT_EQ(f.m.lift_mult(0), 1.0f) << "lift shaping off";
}

TEST(FootholdShaper, AHungRearLegIsLiftedByItsOwnRiseNotTheBodysMedian) {
    // the front pair stands on a tread at -0.04, the rear pair a step below at -0.09; every leg will land on the tread
    // (-0.04).  The rear legs must lift (a 5 cm rise from where they stood); the front legs, already there, must not.
    Fix f({{"lift_gain", 20.0}, {"lift_max", 2.0}});
    f.run(0, {kFlat, kFlat, kFlat, kFlat}, {1, 1, 1, 1}, {-0.04f, -0.04f, -0.09f, -0.09f});
    std::vector<float> tread = {-0.04f, 5000.0f, 1.0f, 0.1f, 0.1f, 0.0f};
    f.run(1, {tread, tread, tread, tread}, {1, 1, 1, 1}, {-0.04f, -0.04f, -0.09f, -0.09f});
    EXPECT_FLOAT_EQ(f.m.lift_mult(0), 1.0f); EXPECT_FLOAT_EQ(f.m.lift_mult(1), 1.0f);
    EXPECT_FLOAT_EQ(f.m.lift_mult(2), 2.0f); EXPECT_FLOAT_EQ(f.m.lift_mult(3), 2.0f);
}
