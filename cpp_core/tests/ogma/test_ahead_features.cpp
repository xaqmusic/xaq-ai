// The ground ahead: TerrainMemory's ahead grid and AheadFeatures' conditioning of it (learned stepper plan 9.7, 2026-11-20).
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/AheadFeatures.hpp"
#include "ogma/modules/AheadField.hpp"
#include "ogma/modules/TerrainMemory.hpp"
namespace {
using ogma::body::tof8::beam_pitched;
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v, uint64_t t = 0) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); p->tick_id = t;
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
// an ahead-grid token: R x C heights (kNoHeight = nothing remembered), precision 1 where there is one
std::vector<float> grid(int R, int C, std::vector<float> const& h) {
    std::vector<float> g{float(R), float(C), 0.04f, 0.08f, 0.10f, 0.0f};
    for (float x : h) g.push_back(x);
    for (float x : h) g.push_back(x <= ogma::TerrainMemory::kNoHeight + 1.0f ? 0.0f : 1.0f);
    return g;
}
// a floor at `floor` with a step of `rise` from row `from` on, R x C
std::vector<float> step(int R, int C, float floor, float rise, int from) {
    std::vector<float> h; for (int i = 0; i < R; ++i) for (int j = 0; j < C; ++j) h.push_back(floor + (i >= from ? rise : 0.0f)); return h;
}
struct Fix {
    ogma::InProcessBus bus; ogma::AheadFeatures m; std::vector<float> out; int n_out = 0;
    Fix(ogma::ParamMap p = {}) {
        m.set_id("ahead_features"); m.on_setup(&bus, p);
        bus.subscribe("reality.proprio.ahead_features", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){
            auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); out.assign(t->values.data(), t->values.data() + t->values.size()); ++n_out; });
    }
    void run(uint64_t t, std::vector<float> const& g, float up_y = 1.0f) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.up_body", pv({0, up_y, 0}, t)); bus.publish("terrain.ahead", pv(g, t));
        m.tick(t); bus.end_tick();
    }
};
double cosine(std::vector<float> const& a, std::vector<float> const& b) {
    double ab = 0, aa = 0, bb = 0; for (size_t i = 0; i < a.size(); ++i) { ab += double(a[i]) * b[i]; aa += double(a[i]) * a[i]; bb += double(b[i]) * b[i]; }
    return ab / std::sqrt(aa * bb);
}
}

TEST(AheadFeatures, FlatGroundIsTheConstantAlone_WhateverTheBodysHeight) {
    Fix f; f.run(1, grid(9, 3, step(9, 3, -0.09f, 0.0f, 99)));
    ASSERT_EQ(f.n_out, 1); ASSERT_EQ(int(f.out.size()), 8 * 3 + 3 + 1);
    for (int k = 0; k < 27; ++k) EXPECT_NEAR(f.out[size_t(k)], 0.0f, 1e-6) << k;
    EXPECT_NEAR(f.out[27], 0.5 * std::sqrt(24.0), 1e-5) << "the constant: bias_ratio x sqrt(cells)";
    const auto flat = f.out;
    f.run(2, grid(9, 3, step(9, 3, -0.05f, 0.0f, 99)));   // the body crouched 4 cm: the same ground
    EXPECT_NEAR(cosine(flat, f.out), 1.0, 1e-9) << "the common mode (the body's height over its own ground) is out";
}

TEST(AheadFeatures, ARiserIsItsHeightInBodyUnits_FromTheRowItStandsAt) {
    Fix f; f.run(1, grid(9, 3, step(9, 3, -0.09f, 0.064f, 4)));
    ASSERT_EQ(f.n_out, 1);
    for (int i = 1; i < 9; ++i) for (int j = 0; j < 3; ++j)
        EXPECT_NEAR(f.out[size_t((i - 1) * 3 + j)], i >= 4 ? 0.064 / 0.085 : 0.0, 1e-5) << "row " << i << " col " << j;
    EXPECT_NEAR(f.m.ref(), -0.09, 1e-6) << "the reference is row 0: the ground under the front toes";
}

TEST(AheadFeatures, TheHeightOfARiserIsADirection_NotOnlyAMagnitude) {
    // the encoder keeps a direction only; without the constant a 38 mm and a 64 mm riser at the same place are one vector
    Fix f; f.run(1, grid(9, 3, step(9, 3, -0.09f, 0.038f, 3))); const auto low = f.out;
    f.run(2, grid(9, 3, step(9, 3, -0.09f, 0.064f, 3))); const auto high = f.out;
    f.run(3, grid(9, 3, step(9, 3, -0.09f, 0.0f, 99)));   const auto flat = f.out;
    const double a_low = std::acos(cosine(low, flat)), a_high = std::acos(cosine(high, flat)), a_between = std::acos(cosine(low, high));
    EXPECT_GT(a_low, 0.5) << "a 38 mm riser is well away from flat";
    EXPECT_GT(a_high - a_low, 0.2) << "and a 64 mm one further still";
    EXPECT_GT(a_between, 0.2);
    Fix g(ogma::ParamMap{{"bias_ratio", ogma::ParamValue{0.0}}});
    g.run(1, grid(9, 3, step(9, 3, -0.09f, 0.038f, 3))); const auto l0 = g.out;
    g.run(2, grid(9, 3, step(9, 3, -0.09f, 0.064f, 3)));
    EXPECT_NEAR(cosine(l0, g.out), 1.0, 1e-6) << "the control: no constant, one direction";
}

TEST(AheadFeatures, WhatIsNotRememberedIsUnknown_NotFlat) {
    auto h = step(9, 3, -0.09f, 0.0f, 99);
    for (int i = 1; i < 9; ++i) h[size_t(i * 3 + 2)] = ogma::TerrainMemory::kNoHeight;   // column 2 unseen past the toes' line
    Fix f; f.run(1, grid(9, 3, h));
    ASSERT_EQ(f.n_out, 1);
    EXPECT_NEAR(f.out[24], 0.0f, 1e-6); EXPECT_NEAR(f.out[25], 0.0f, 1e-6); EXPECT_NEAR(f.out[26], 1.0f, 1e-6) << "the column's unknown share";
    for (int i = 1; i < 9; ++i) EXPECT_NEAR(f.out[size_t((i - 1) * 3 + 2)], 0.0f, 1e-6);
    Fix g; g.run(1, grid(9, 3, step(9, 3, -0.09f, 0.0f, 99)));
    EXPECT_LT(cosine(f.out, g.out), 0.99) << "an unseen column is not the flat vector";
}

TEST(AheadFeatures, AnEmptyFirstRowTakesTheNearestRememberedRow_AndAnEmptyGridSaysNothing) {
    auto h = step(9, 3, -0.09f, 0.05f, 5);
    for (int j = 0; j < 3; ++j) h[size_t(j)] = ogma::TerrainMemory::kNoHeight;
    Fix f; f.run(1, grid(9, 3, h));
    ASSERT_EQ(f.n_out, 1); EXPECT_NEAR(f.m.ref(), -0.09, 1e-6);
    EXPECT_NEAR(f.out[size_t((5 - 1) * 3)], 0.05 / 0.085, 1e-5);
    f.run(2, grid(9, 3, std::vector<float>(27, ogma::TerrainMemory::kNoHeight)));
    EXPECT_EQ(f.n_out, 1) << "nothing remembered ahead: no claim";
}

TEST(AheadFeatures, ABodyThatIsNotUprightHoldsItsFeatures) {
    Fix f; f.run(1, grid(9, 3, step(9, 3, -0.09f, 0.0f, 99)), 0.5f);
    EXPECT_EQ(f.n_out, 0); EXPECT_EQ(f.m.held(), 1);
    f.run(2, grid(9, 3, step(9, 3, -0.09f, 0.0f, 99)), 0.95f);
    EXPECT_EQ(f.n_out, 1);
}

// ---- the grid itself, from the memory ----
namespace {
std::vector<float> synth(double H, double pitch, double hb, double z0, double z1, double w) {
    std::vector<float> f(64, -1.0f);
    for (int z = 0; z < 64; ++z) {
        const auto d = beam_pitched(z, pitch); double best = 1e9;
        if (d.y < -1e-9) { const double t = -H / d.y; if (t > 0) best = std::min(best, t); }
        if (hb > 0 && z1 > z0) {
            if (d.y < -1e-9) { const double t = (-H + hb) / d.y; const double zz = t * d.z, xx = t * d.x; if (t > 0 && zz >= z0 && zz <= z1 && std::fabs(xx) <= w) best = std::min(best, t); }
            if (d.z > 1e-9) { const double t = z0 / d.z; const double yy = t * d.y, xx = t * d.x; if (t > 0 && yy >= -H && yy <= -H + hb && std::fabs(xx) <= w) best = std::min(best, t); }
        }
        if (best < 4.0) f[size_t(z)] = float(best);
    }
    return f;
}
}

TEST(TerrainMemoryAhead, TheGridReadsTheFloorAndTheBoxWhereTheyAre_AndFeedsTheFeatures) {
    const double kSensorY = 0.049, kFloorY = -0.09, kH = kSensorY - kFloorY;
    ogma::InProcessBus bus; ogma::TerrainMemory tm; ogma::AheadFeatures af; std::vector<float> g, feat;
    tm.set_id("terrain_memory");
    tm.on_setup(&bus, ogma::ParamMap{{"sensor_y_m", ogma::ParamValue{kSensorY}}, {"sensor_z_m", ogma::ParamValue{0.0}}, {"ahead_topic", ogma::ParamValue{std::string("terrain.ahead")}}});
    af.set_id("ahead_features"); af.on_setup(&bus, {});
    bus.subscribe("terrain.ahead", ogma::SubscriptionKind::Direct, [&](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); g.assign(t->values.data(), t->values.data() + t->values.size()); });
    bus.subscribe("reality.proprio.ahead_features", ogma::SubscriptionKind::Direct, [&](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); feat.assign(t->values.data(), t->values.data() + t->values.size()); });
    // the body walks 0.30 m toward a 5 cm step (wider than the grid) and ends 0.30 m from it: one still frame leaves gaps between
    // the field's rays at distance, the approach fills them (the memory is what fuses the frames)
    for (uint64_t t = 1; t <= 31; ++t) {
        const double x = 0.01 * double(t - 1);
        const auto field = synth(kH, 30.0, 0.05, 0.60 - x, 1.50 - x, 0.40);
        bus.begin_tick(t);
        bus.publish("reality.proprio.odom", pv({float(x), 0, 0}, t)); bus.publish("reality.proprio.up_body", pv({0, 1, 0}, t));
        bus.publish("sense.tof8", pv(field, t));
        bus.publish("reality.proprio.foot_contact", pv({1, 1, 1, 1}, t)); bus.publish("reality.proprio.feet_xz", pv({0.1f, 0.1f, -0.1f, 0.1f, 0.1f, -0.1f, -0.1f, -0.1f}, t));
        bus.publish("reality.proprio.toe_y", pv({float(kFloorY), float(kFloorY), float(kFloorY), float(kFloorY)}, t)); bus.publish("reality.proprio.foot_load", pv({0.3f, 0.3f, 0.3f, 0.3f}, t));
        tm.tick(t); af.tick(t); bus.end_tick();
    }
    ASSERT_EQ(int(g.size()), ogma::TerrainMemory::kAheadHeader + 2 * 9 * 3);
    EXPECT_EQ(int(g[0]), 9); EXPECT_EQ(int(g[1]), 3);
    const int H = ogma::TerrainMemory::kAheadHeader;
    // rows are 4 cm deep, row i centred at 0.10 + 0.04 i: rows 1-3 (0.12-0.24) are floor, rows 6-8 (0.32-0.44) are the box's top
    int floor_rows = 0, box_rows = 0, box_row = -1;
    for (int i : {1, 2, 3}) if (g[size_t(H + 27 + i * 3 + 1)] > 0.0f) { ++floor_rows; EXPECT_NEAR(g[size_t(H + i * 3 + 1)], kFloorY, 0.012) << "row " << i; }
    for (int i : {6, 7, 8}) if (g[size_t(H + 27 + i * 3 + 1)] > 0.0f) { ++box_rows; box_row = i; EXPECT_NEAR(g[size_t(H + i * 3 + 1)], kFloorY + 0.05, 0.02) << "row " << i; }   // the voxel on the edge also holds the riser's face
    ASSERT_GE(floor_rows, 2); ASSERT_GE(box_rows, 2);
    ASSERT_EQ(int(feat.size()), 28);
    EXPECT_NEAR(feat[size_t((box_row - 1) * 3 + 1)], 0.05 / 0.085, 0.15) << "the features carry the step in body units";
    EXPECT_NEAR(feat[size_t((2 - 1) * 3 + 1)], 0.0, 0.15);
}

TEST(TerrainMemoryAhead, NoTopicNoGrid) {
    ogma::InProcessBus bus; ogma::TerrainMemory tm; tm.set_id("terrain_memory"); tm.on_setup(&bus, {});
    for (auto const& t : tm.output_topics()) EXPECT_NE(t.name, std::string("terrain.ahead"));
    EXPECT_EQ(int(tm.output_topics().size()), 2);
}

// ---- the grid straight from the field, against the front toes ----
namespace {
struct FF {
    ogma::InProcessBus bus; ogma::AheadField m; std::vector<float> g;
    FF(ogma::ParamMap p = {}) {
        p.emplace("sensor_z_m", ogma::ParamValue{0.0});
        m.set_id("ahead_field"); m.on_setup(&bus, p);
        bus.subscribe("terrain.ahead", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); g.assign(t->values.data(), t->values.data() + t->values.size()); });
    }
    // one tick: a field from a sensor `sensor_over_floor` above the floor, the front toes at toe_y (body origin 0), up = +y
    void run(uint64_t t, const std::vector<float>* field, float toe_y, std::vector<float> contact = {1, 1, 1, 1}, float up_y = 1.0f) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.up_body", pv({0, up_y, 0}, t)); bus.publish("reality.proprio.toe_y", pv({toe_y, toe_y, toe_y, toe_y}, t));
        bus.publish("reality.proprio.foot_contact", pv(contact, t));
        if (field) bus.publish("sense.tof8", pv(*field, t));
        m.tick(t); bus.end_tick();
    }
    int known_in_row(int i) const { int n = 0; for (int j = 0; j < 3; ++j) n += g[size_t(ogma::TerrainMemory::kAheadHeader + 27 + i * 3 + j)] > 0.0f ? 1 : 0; return n; }
    float h(int i, int j) const { return g[size_t(ogma::TerrainMemory::kAheadHeader + i * 3 + j)]; }
};
}

TEST(AheadField, FlatGroundReadsZeroAgainstTheToes_AtAnyBodyHeight) {
    for (double floor_y : {-0.09, -0.06}) {   // standing, then crouched 3 cm: the sensor is 0.049 above the origin
        FF f; const auto field = synth(0.049 - floor_y, 30.0, 0, 0, 0, 0);
        for (uint64_t t = 1; t <= 5; ++t) f.run(t, &field, float(floor_y));
        int known = 0;
        for (int i = 0; i < 9; ++i) for (int j = 0; j < 3; ++j)
            if (f.g[size_t(ogma::TerrainMemory::kAheadHeader + 27 + i * 3 + j)] > 0.0f) { ++known; EXPECT_NEAR(f.h(i, j), 0.0f, 0.004) << "row " << i << " col " << j << " floor " << floor_y; }
        EXPECT_GE(known, 8) << "the field reaches most of the grid from one still frame";
        EXPECT_NEAR(f.m.ref(), floor_y, 1e-6);
    }
}

TEST(AheadField, AStepReadsItsHeightWhereItStands_WithNoOdometryAndNoDatum) {
    // The body walks 0.34 m toward a 5 cm step at the rhythm's pace (10 cm/s), stops 0.26 m from it and stands 2 s; the module is
    // never told it moved.  A level body's rays meet a level surface at fixed distances, so only some rows are reached: on the
    // floor rows 0-3, on the step's top row 5 (0.31 m).  What a ray last left (the riser's face, as it passed) is forgotten after
    // hold_s; what is reached now reads true.
    FF f;
    for (uint64_t t = 1; t <= 271; ++t) {
        const double x = std::min(0.34, 0.002 * double(t - 1)); const auto field = synth(0.049 + 0.09, 30.0, 0.05, 0.60 - x, 1.50 - x, 0.40);
        f.run(t, &field, -0.09f);
    }
    int top = 0, floor = 0, unknown = 0;
    for (int j = 0; j < 3; ++j) { bool k; const double v = f.m.cell(5, j, k); if (k) { ++top; EXPECT_NEAR(v, 0.05, 0.008) << "row 5 col " << j; } }
    for (int i : {0, 1, 2, 3}) for (int j = 0; j < 3; ++j) { bool k; const double v = f.m.cell(i, j, k); if (k) { ++floor; EXPECT_NEAR(v, 0.0, 0.006) << "row " << i << " col " << j; } }
    for (int i = 0; i < 9; ++i) for (int j = 0; j < 3; ++j) { bool k; f.m.cell(i, j, k); unknown += k ? 0 : 1; }
    EXPECT_GE(top, 1); EXPECT_GE(floor, 6); EXPECT_GT(unknown, 0) << "rows no ray reaches are unknown, not stale";
}

TEST(AheadField, NoFrontToeOnTheGroundNoMeasurement_AndATiltedBodyDropsTheFrame) {
    FF f; const auto field = synth(0.049 + 0.09, 30.0, 0, 0, 0, 0);
    f.run(1, &field, -0.09f, {0, 0, 1, 1});
    EXPECT_EQ(f.m.frames(), 0); EXPECT_EQ(f.m.no_ref(), 1);
    for (int i = 0; i < 9; ++i) EXPECT_EQ(f.known_in_row(i), 0);
    f.run(2, &field, -0.09f, {1, 1, 1, 1}, 0.5f);
    EXPECT_EQ(f.m.frames(), 0);
    f.run(3, &field, -0.09f, {0, 1, 1, 1});   // one front toe is enough
    EXPECT_EQ(f.m.frames(), 1);
}

TEST(AheadField, ACellNoReturnReachesGoesUnknownAfterTheHold) {
    FF f(ogma::ParamMap{{"hold_s", ogma::ParamValue{0.2}}}); const auto field = synth(0.049 + 0.09, 30.0, 0, 0, 0, 0);
    f.run(1, &field, -0.09f);
    int known = 0; for (int i = 0; i < 9; ++i) known += f.known_in_row(i);
    ASSERT_GT(known, 0);
    for (uint64_t t = 2; t <= 14; ++t) f.run(t, nullptr, -0.09f);   // 0.26 s with no frame
    known = 0; for (int i = 0; i < 9; ++i) known += f.known_in_row(i);
    EXPECT_EQ(known, 0);
}

TEST(AheadField, ItsGridFeedsTheFeaturesWithNothingSubtracted) {
    ogma::InProcessBus bus; ogma::AheadField af; ogma::AheadFeatures ft; std::vector<float> feat;
    af.set_id("ahead_field"); af.on_setup(&bus, ogma::ParamMap{{"sensor_z_m", ogma::ParamValue{0.0}}});
    ft.set_id("ahead_features"); ft.on_setup(&bus, ogma::ParamMap{{"ref_row0", ogma::ParamValue{false}}});
    bus.subscribe("reality.proprio.ahead_features", ogma::SubscriptionKind::Direct, [&](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); feat.assign(t->values.data(), t->values.data() + t->values.size()); });
    for (uint64_t t = 1; t <= 271; ++t) {
        const double x = std::min(0.34, 0.002 * double(t - 1)); const auto field = synth(0.049 + 0.09, 30.0, 0.05, 0.60 - x, 1.50 - x, 0.40);
        bus.begin_tick(t);
        bus.publish("reality.proprio.up_body", pv({0, 1, 0}, t)); bus.publish("reality.proprio.toe_y", pv({-0.09f, -0.09f, -0.09f, -0.09f}, t));
        bus.publish("reality.proprio.foot_contact", pv({1, 1, 1, 1}, t)); bus.publish("sense.tof8", pv(field, t));
        af.tick(t); ft.tick(t); bus.end_tick();
    }
    ASSERT_EQ(int(feat.size()), 28);
    EXPECT_NEAR(feat[size_t((5 - 1) * 3 + 1)], 0.05 / 0.085, 0.1) << "row 5, the step's top, in body units";
    EXPECT_NEAR(feat[size_t((2 - 1) * 3 + 1)], 0.0, 0.07);
    EXPECT_NEAR(ft.ref(), 0.0, 1e-9) << "the toes' ground is already out of the grid";
    EXPECT_GT(feat[24] + feat[25] + feat[26], 0.0f) << "the rows no ray reaches are in the unknown shares";
}

TEST(AheadField, TheStepTheBodyStandsAcrossIsTheFrontToesOverTheRearToes) {
    FF f; const auto field = synth(0.049 + 0.03, 30.0, 0, 0, 0, 0);
    for (uint64_t t = 1; t <= 100; ++t) {   // the front toes 6 cm above the rear toes: the body is across a riser
        f.bus.begin_tick(t);
        f.bus.publish("reality.proprio.up_body", pv({0, 1, 0}, t)); f.bus.publish("reality.proprio.toe_y", pv({-0.03f, -0.03f, -0.09f, -0.09f}, t));
        f.bus.publish("reality.proprio.foot_contact", pv({1, t % 2 ? 1.0f : 0.0f, 1, 1}, t)); f.bus.publish("sense.tof8", pv(field, t));
        f.m.tick(t); f.bus.end_tick();
    }
    EXPECT_NEAR(f.m.astride(), 0.06, 1e-4);
    EXPECT_NEAR(f.g[5], 0.06f, 1e-4) << "the token's sixth header value";
}

TEST(AheadFeatures, TheGroundUnderTheBodyIsAFeatureWithTheWeightOfTheRowsItCovers) {
    auto g = grid(9, 3, step(9, 3, 0.0f, 0.0f, 99)); g[5] = 0.064f;   // flat ahead, a 64 mm step astride
    Fix f(ogma::ParamMap{{"ref_row0", ogma::ParamValue{false}}, {"under_rows", ogma::ParamValue{int64_t{5}}}});
    f.run(1, g);
    ASSERT_EQ(int(f.out.size()), 24 + 3 + 1 + 1);
    EXPECT_NEAR(f.out[27], -0.064 / 0.085 * std::sqrt(15.0), 1e-4) << "the rear ground is below the front ground, five rows of three cells' worth";
    const auto astride = f.out;
    g[5] = 0.0f; f.run(2, g);
    EXPECT_LT(cosine(astride, f.out), 0.8) << "astride a riser is not the flat vector, though the ground ahead is flat in both";
    EXPECT_THROW({ ogma::InProcessBus b; ogma::AheadFeatures m; m.on_setup(&b, ogma::ParamMap{{"under_rows", ogma::ParamValue{int64_t{5}}}}); }, std::invalid_argument);
}
