// Tof8Range: the 8x8 field reduced to the nearest feature ahead, by the field's own floor.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/Tof8Range.hpp"
namespace {
// A synthetic field: the sensor H above a flat floor, pitched `pitch` down; an optional box (top at height hb, from z0
// to z1 ahead, |x| <= w) and an optional drop (the floor hb lower beyond zd).  Slant ranges by ray intersection, -1 none.
std::vector<float> synth(double H, double pitch_deg, double hb = 0, double z0 = 0, double z1 = 0, double w = 0, double drop_z = 0, double drop_h = 0) {
    std::vector<float> f(64, -1.0f);
    ogma::Tof8Range m; ogma::InProcessBus bus; m.on_setup(&bus, {{"pitch_deg", ogma::ParamValue{pitch_deg}}});
    for (int z = 0; z < 64; ++z) {
        const auto d = m.beam_pitched(z);
        double best = 1e9;
        auto floor_hit = [&](double floor_y, double zmin, double zmax) {
            if (d[1] >= -1e-9) return;
            const double t = floor_y / d[1]; const double zz = t * d[2];
            if (t > 0 && zz >= zmin && zz <= zmax) best = std::min(best, t);
        };
        if (drop_h > 0) { floor_hit(-H, -1e9, drop_z); floor_hit(-H - drop_h, drop_z, 1e9); }
        else floor_hit(-H, -1e9, 1e9);
        if (hb > 0 && z1 > z0) {
            // the box top
            if (d[1] < -1e-9) { const double t = (-H + hb) / d[1]; const double zz = t * d[2], xx = t * d[0];
                if (t > 0 && zz >= z0 && zz <= z1 && std::fabs(xx) <= w) best = std::min(best, t); }
            // the front face
            if (d[2] > 1e-9) { const double t = z0 / d[2]; const double yy = t * d[1], xx = t * d[0];
                if (t > 0 && yy >= -H && yy <= -H + hb && std::fabs(xx) <= w) best = std::min(best, t); }
        }
        if (best < 4.0) f[size_t(z)] = float(best);
    }
    return f;
}
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::Tof8Range m; std::vector<float> last; int n = 0;
    Fix(double pitch = 30.0) {
        m.set_id("tof8_range"); m.on_setup(&bus, {{"pitch_deg", ogma::ParamValue{pitch}}});
        bus.subscribe("sense.range_tof", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){
            auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); ++n; last = {t->values[0], t->values[1]}; });
    }
    void feed(std::vector<float> f, uint64_t t = 1) { bus.begin_tick(t); auto p = pv(f); p->tick_id = t; bus.publish("sense.tof8", p); m.tick(t); bus.end_tick(); }
};
}

TEST(Tof8Range, AFlatFloorIsNoFeature) {
    Fix f; f.feed(synth(0.13, 30.0));
    ASSERT_EQ(f.n, 1); EXPECT_FLOAT_EQ(f.last[1], 0.0f) << "nothing ahead";
    EXPECT_TRUE(f.m.floor_ok()); EXPECT_EQ(f.m.n_thing_up(), 0); EXPECT_EQ(f.m.n_thing_down(), 0);
}

TEST(Tof8Range, ABoxAheadIsTheNearestFeatureAtItsFacesRange) {
    Fix f; f.feed(synth(0.13, 30.0, 0.05, 0.60, 0.80, 0.30));
    ASSERT_EQ(f.n, 1); EXPECT_FLOAT_EQ(f.last[1], 1.0f);
    // the face at z = 0.60 is seen from 0.13 - 0.05 .. 0.13 above it: slant ~ sqrt(0.6^2 + 0.1^2) = 0.61
    EXPECT_NEAR(f.last[0], 0.61, 0.03) << "the box's face";
    EXPECT_GT(f.m.n_thing_up(), 3); EXPECT_TRUE(f.m.floor_ok());
    // the columns: the box spans |x| <= 0.3 at 0.6 m, wider than the field there: every column sees it
    for (int c = 0; c < 8; ++c) EXPECT_LT(f.m.cols()[size_t(c)], 1.0f) << "column " << c;
}

TEST(Tof8Range, ThePitchedBodyStillReadsItsFloor_NoImuNeeded) {
    // the body nose-down 6 deg: the floor arrives 6 deg steeper than the mount says; the fit absorbs it
    Fix f; f.feed(synth(0.13, 36.0));
    EXPECT_FLOAT_EQ(f.last[1], 0.0f); EXPECT_TRUE(f.m.floor_ok()); EXPECT_EQ(f.m.n_thing_up(), 0);
    // the same box nearer (at 36 deg the top row reaches only 0.44 m: a box at 0.6 m is out of the field, rightly)
    Fix g; g.feed(synth(0.13, 36.0, 0.05, 0.30, 0.45, 0.30));
    EXPECT_FLOAT_EQ(g.last[1], 1.0f); EXPECT_NEAR(g.last[0], 0.31, 0.03) << "the face at 0.30 m";
}

TEST(Tof8Range, ADropBeyondIsAFeatureToo_AndAWallFillingTheFieldIsAThing) {
    Fix f; f.feed(synth(0.13, 30.0, 0, 0, 0, 0, 0.5, 0.04));
    EXPECT_GT(f.m.n_thing_down(), 2) << "the floor 4 cm lower past 0.5 m";
    EXPECT_FLOAT_EQ(f.last[1], 1.0f);
    // a wall at 0.25 m, taller than the field: the bottom rows still see the floor (they land at 0.11-0.20 m), so the
    // floor fits and the wall is a thing at its foot's range
    Fix w; w.feed(synth(0.13, 30.0, 1.0, 0.25, 0.26, 2.0));
    EXPECT_TRUE(w.m.floor_ok()); EXPECT_FLOAT_EQ(w.last[1], 1.0f); EXPECT_NEAR(w.last[0], 0.26, 0.04);
    // a wall at 0.08 m, inside the bottom rows' floor distance: no floor fits; whatever returns is a thing
    Fix n; n.feed(synth(0.13, 30.0, 1.0, 0.08, 0.09, 2.0));
    EXPECT_FALSE(n.m.floor_ok()); EXPECT_FLOAT_EQ(n.last[1], 1.0f); EXPECT_NEAR(n.last[0], 0.085, 0.03);
}

TEST(Tof8Range, AHeldFieldIsReducedOnceAndRepublishedEveryTick) {
    Fix f; auto field = synth(0.13, 30.0, 0.05, 0.60, 0.80, 0.30);
    f.feed(field, 1);
    f.bus.begin_tick(2); f.m.tick(2); f.bus.end_tick();
    f.bus.begin_tick(3); f.m.tick(3); f.bus.end_tick();
    EXPECT_EQ(f.n, 3); EXPECT_EQ(f.m.diag_snapshot()["frames"].get<int>(), 1);
}
