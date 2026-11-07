// TerrainMemory: heights from the field and the toes in one memory, and the foothold prediction's error.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/TerrainMemory.hpp"
namespace {
using ogma::body::tof8::beam_pitched;
// the field from a sensor H above a flat floor pitched `pitch` down, with an optional box (top hb, z0..z1 ahead of the
// SENSOR, |x| <= w): slant ranges by ray intersection, -1 none
std::vector<float> synth(double H, double pitch, double hb = 0, double z0 = 0, double z1 = 0, double w = 0) {
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
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v, uint64_t t = 0) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size())); p->tick_id = t;
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
// the sim's mount: 0.049 above the body origin, the floor 0.09 below the origin -> the sensor 0.139 above the floor
constexpr double kSensorY = 0.049, kFloorY = -0.09, kH = kSensorY - kFloorY;
struct Fix {
    ogma::InProcessBus bus; ogma::TerrainMemory m; std::vector<float> pred, err;
    Fix(ogma::ParamMap p = {}) {
        p.emplace("sensor_y_m", ogma::ParamValue{kSensorY}); p.emplace("sensor_z_m", ogma::ParamValue{0.0});
        m.set_id("terrain_memory"); m.on_setup(&bus, p);
        bus.subscribe("predict.foothold", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); pred.assign(t->values.data(), t->values.data() + t->values.size()); });
        bus.subscribe("predict.foothold_error", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); err.assign(t->values.data(), t->values.data() + t->values.size()); });
    }
    // one tick: the field (or none), the contacts, the toes' [x, z] and gravity heights (body origin 0), odom at rest
    void run(uint64_t t, std::vector<float> const* field, std::vector<float> contact, std::vector<float> xz, std::vector<float> y) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.odom", pv({0, 0, 0}, t)); bus.publish("reality.proprio.up_body", pv({0, 1, 0}, t));
        if (field) bus.publish("sense.tof8", pv(*field, t));
        bus.publish("reality.proprio.foot_contact", pv(contact, t)); bus.publish("reality.proprio.feet_xz", pv(xz, t)); bus.publish("reality.proprio.toe_y", pv(y, t));
        std::vector<float> load(4); for (int i = 0; i < 4; ++i) load[size_t(i)] = contact[size_t(i)] > 0.5f ? 0.3f : 0.0f;
        bus.publish("reality.proprio.foot_load", pv(load, t));
        m.tick(t); bus.end_tick();
    }
};
const std::vector<float> kXZ = {0.1f, 0.1f, -0.1f, 0.1f, 0.1f, -0.1f, -0.1f, -0.1f};
const std::vector<float> kY = {float(kFloorY), float(kFloorY), float(kFloorY), float(kFloorY)};
}

TEST(TerrainMemory, TheFieldPlacesABoxAtItsHeightAndDistance) {
    Fix f; auto field = synth(kH, 30.0, 0.05, 0.40, 0.60, 0.30);
    f.run(1, &field, {1, 1, 1, 1}, kXZ, kY);
    ASSERT_TRUE(f.m.floor_ok());
    int on_box = 0, floor = 0; double hsum = 0;
    for (auto const& v : f.m.body_voxels()) {
        if (v.fwd > 0.42 && v.fwd < 0.58 && std::fabs(v.lat) < 0.1) { ++on_box; hsum += v.h; }
        else if (v.fwd < 0.35 && v.cls == 1) ++floor;
    }
    ASSERT_GT(on_box, 3); EXPECT_NEAR(hsum / on_box, kFloorY + 0.05, 0.012) << "the box's top, in the body's gravity frame";
    EXPECT_GT(floor, 5) << "the near floor is floor";
}

TEST(TerrainMemory, ALegThatLandsOnTheBoxIsPredictedToMeetItAndTheTouchHasNoError) {
    Fix f; auto field = synth(kH, 30.0, 0.05, 0.40, 0.60, 0.30);
    // leg 0 lands at fwd 0.45 (on the box's top) each stride; two floor touches elsewhere first define nothing there
    std::vector<float> xz = kXZ; xz[1] = 0.45f;   // leg 0: [x, z]
    uint64_t t = 1;
    for (int k = 0; k < 5; ++k, ++t) f.run(t, &field, {0, 1, 1, 1}, xz, kY);        // leg 0 in the air, the field seen
    // leg 0 touches the box top (its toe 0.05 above the floor): the first touch sets its landing mean
    std::vector<float> y = kY; y[0] = float(kFloorY + 0.05);
    f.run(t++, &field, {1, 1, 1, 1}, xz, y);
    EXPECT_FLOAT_EQ(f.err[1], 0.0f) << "no prediction before the first landing is known";
    f.run(t++, &field, {0, 1, 1, 1}, xz, kY);        // the toe lifts: now the prediction stands at its landing spot
    ASSERT_FLOAT_EQ(f.pred[2], 1.0f) << "has a prediction";
    EXPECT_NEAR(f.pred[0], kFloorY + 0.05, 0.012) << "it will meet the box";
    f.run(t++, &field, {1, 1, 1, 1}, xz, y);         // it lands on the box: the error is small
    EXPECT_FLOAT_EQ(f.err[1], 1.0f); EXPECT_NEAR(f.err[0], 0.0, 0.015);
    EXPECT_EQ(f.m.n_errors(), 1);
}

TEST(TerrainMemory, ATouchLowerThanPredictedIsAnErrorAndCorrectsTheMemoryStrongly) {
    Fix f; auto field = synth(kH, 30.0, 0.05, 0.40, 0.60, 0.30);
    std::vector<float> xz = kXZ; xz[1] = 0.45f;
    uint64_t t = 1;
    for (int k = 0; k < 20; ++k, ++t) f.run(t, &field, {0, 1, 1, 1}, xz, kY);     // many frames: the field is sure of the box
    std::vector<float> yb = kY; yb[0] = float(kFloorY + 0.05);
    f.run(t++, &field, {1, 1, 1, 1}, xz, yb); f.run(t++, &field, {0, 1, 1, 1}, xz, kY);
    ASSERT_NEAR(f.pred[0], kFloorY + 0.05, 0.012);
    // the toe finds the surface 2 cm lower than remembered (a compliant top)
    std::vector<float> yl = kY; yl[0] = float(kFloorY + 0.03);
    f.run(t++, nullptr, {1, 1, 1, 1}, xz, yl);   // no field this tick: the touch alone speaks
    EXPECT_LT(f.err[0], -0.005) << "the error says lower"; EXPECT_GT(f.err[0], -0.035);
    f.run(t++, nullptr, {0, 1, 1, 1}, xz, kY);
    EXPECT_LT(f.pred[0], kFloorY + 0.042) << "one touch moved the memory at least half way (its variance is floored at a touch's)";
    EXPECT_GT(f.m.rms_error(0), 0.001);
}

TEST(TerrainMemory, NoFieldNoTouchesNoCrash_AndAVoxelAgesOut) {
    Fix f({{"max_age_ticks", ogma::ParamValue{int64_t{50}}}});
    f.run(1, nullptr, {0, 0, 0, 0}, kXZ, kY);
    EXPECT_EQ(f.m.count(), 0); EXPECT_EQ(f.pred.size(), 20u);
    auto field = synth(kH, 30.0);
    f.run(2, &field, {0, 0, 0, 0}, kXZ, kY);
    EXPECT_GT(f.m.count(), 30);
    for (uint64_t t = 3; t <= 80; ++t) f.run(t, nullptr, {0, 0, 0, 0}, kXZ, kY);
    EXPECT_EQ(f.m.count(), 0) << "unseen for 50 ticks, every voxel left";
}

TEST(TerrainMemory, ClimbingATreadMovesTheDatumNotTheGround) {
    // the floor under all four feet is touched and known; the field sees a 5 cm tread ahead; the front toe (leg 0)
    // lands on it; then the body climbs onto it (every toe back at -0.09 in the body frame) and the rear toes land on
    // ground the front toe already touched: the datum must rise by 0.05 and the old floor must read -0.14 from up here
    Fix f; auto field = synth(kH, 30.0, 0.05, 0.25, 0.90, 0.40);
    std::vector<float> xz = kXZ; uint64_t t = 1;
    // standing: four floor touches (one per leg, rising edges)
    f.run(t++, &field, {0, 0, 0, 0}, xz, kY);
    f.run(t++, &field, {1, 1, 1, 1}, xz, kY);
    for (int k = 0; k < 10; ++k, ++t) f.run(t, &field, {1, 1, 1, 1}, xz, kY);
    EXPECT_NEAR(f.m.datum(), 0.0, 1e-6);
    // the front toe reaches forward onto the tread (0.30 ahead): the tread was seen by the field only -> the voxel is corrected, the datum stays
    std::vector<float> xz2 = xz; xz2[1] = 0.30f; std::vector<float> y2 = kY; y2[0] = float(kFloorY + 0.05);
    f.run(t++, &field, {0, 1, 1, 1}, xz2, kY); f.run(t++, &field, {1, 1, 1, 1}, xz2, y2);
    EXPECT_NEAR(f.m.datum(), 0.0, 1e-6) << "seen-only ground: the ground is corrected, not the body";
    // the body climbs: the tread is now the floor under the body (every toe at -0.09); the field sees the next floor;
    // the rear toes (legs 2, 3) land on the place the front toe touched: the datum takes the 0.05
    std::vector<float> xz3 = xz; xz3[5] = 0.30f; xz3[7] = 0.30f;   // legs 2, 3 at z = 0.30 (where leg 0 touched)
    auto field_up = synth(kH, 30.0);                              // from up here the ground ahead is flat again
    f.run(t++, &field_up, {1, 1, 0, 0}, xz3, kY); f.run(t++, &field_up, {1, 1, 1, 1}, xz3, kY);
    f.run(t++, &field_up, {1, 1, 1, 1}, xz3, kY);   // leg 2 bears on the place leg 0 touched: the datum moves, bounded to 3 cm
    EXPECT_NEAR(f.m.datum(), 0.03, 1e-6) << "one bearing touch on known ground: one bounded step";
    // leg 3's place was field-only (the other side of the tread): the ground was corrected, not the body.  Another
    // stride of leg 2 on the same spot completes the climb
    f.run(t++, &field_up, {1, 1, 0, 1}, xz3, kY); f.run(t++, &field_up, {1, 1, 1, 1}, xz3, kY); f.run(t++, &field_up, {1, 1, 1, 1}, xz3, kY);
    EXPECT_NEAR(f.m.datum(), 0.05, 0.012) << "the body knows it climbed 5 cm from its toes alone";
    // and a prediction for a leg landing back on the old floor voxel reads 5 cm lower than the body's floor
    std::vector<float> xz4 = xz; xz4[1] = -0.30f;   // leg 0 would land 0.30 behind (never touched there) -- no prediction expected
    f.run(t++, &field_up, {0, 1, 1, 1}, xz4, kY);
    EXPECT_GT(f.m.n_errors(), 0);
}

TEST(TerrainMemory, AScrapeThatNeverBearsDoesNotMoveTheDatum) {
    Fix f; auto field = synth(kH, 30.0);
    std::vector<float> xz = kXZ; uint64_t t = 1;
    f.run(t++, &field, {0, 0, 0, 0}, xz, kY); f.run(t++, &field, {1, 1, 1, 1}, xz, kY);
    for (int k = 0; k < 5; ++k, ++t) f.run(t, &field, {1, 1, 1, 1}, xz, kY);
    // leg 0 lifts, grazes its own known spot 4 cm higher (a riser face) for 3 ticks with no load, and leaves
    std::vector<float> yg = kY; yg[0] = float(kFloorY + 0.04);
    f.run(t++, &field, {0, 1, 1, 1}, xz, kY);
    for (int k = 0; k < 3; ++k, ++t) { f.bus.begin_tick(t); f.bus.publish("reality.proprio.odom", pv({0, 0, 0}, t)); f.bus.publish("reality.proprio.up_body", pv({0, 1, 0}, t));
        f.bus.publish("reality.proprio.foot_contact", pv({1, 1, 1, 1}, t)); f.bus.publish("reality.proprio.feet_xz", pv(xz, t)); f.bus.publish("reality.proprio.toe_y", pv(yg, t));
        f.bus.publish("reality.proprio.foot_load", pv({0.02f, 0.3f, 0.3f, 0.3f}, t)); f.m.tick(t); f.bus.end_tick(); }
    f.run(t++, &field, {0, 1, 1, 1}, xz, kY);
    EXPECT_NEAR(f.m.datum(), 0.0, 1e-6) << "no weight came onto it: not a climb";
}

TEST(TerrainMemory, AHeldBodysMemoryDoesNotSlideUnderIt) {
    // the field sees a tread 0.45 ahead; the legs then claim 0.30 m of odometry while the stall says the body is held:
    // with the stall gate the tread stays 0.45 ahead in the memory; without it the memory thinks the tread is under the body
    for (bool gated : {true, false}) {
        ogma::ParamMap p; if (gated) p["stall_topic"] = std::string("reality.proprio.stuck");
        Fix f(p); auto field = synth(kH, 30.0, 0.05, 0.40, 0.60, 0.30);
        std::vector<float> xz = kXZ; xz[1] = 0.45f;   // leg 0 lands 0.45 ahead
        uint64_t t = 1;
        auto step = [&](double ox, float stall, std::vector<float> contact, std::vector<float> y, bool with_field) {
            f.bus.begin_tick(t); f.bus.publish("reality.proprio.odom", pv({float(ox), 0, 0}, t)); f.bus.publish("reality.proprio.up_body", pv({0, 1, 0}, t));
            f.bus.publish("reality.proprio.stuck", pv({stall, 0, 0, 0, 0}, t));
            if (with_field) f.bus.publish("sense.tof8", pv(field, t));
            f.bus.publish("reality.proprio.foot_contact", pv(contact, t)); f.bus.publish("reality.proprio.feet_xz", pv(xz, t)); f.bus.publish("reality.proprio.toe_y", pv(y, t));
            std::vector<float> load(4); for (int i = 0; i < 4; ++i) load[size_t(i)] = contact[size_t(i)] > 0.5f ? 0.3f : 0.0f; f.bus.publish("reality.proprio.foot_load", pv(load, t));
            f.m.tick(t); f.bus.end_tick(); ++t;
        };
        std::vector<float> y_box = kY; y_box[0] = float(kFloorY + 0.05);
        step(0.0, 0.0f, {0, 1, 1, 1}, kY, true); step(0.0, 0.0f, {1, 1, 1, 1}, y_box, true); step(0.0, 0.0f, {0, 1, 1, 1}, kY, true);   // leg 0 touched the tread once
        ASSERT_NEAR(f.pred[0], kFloorY + 0.05, 0.012);
        // held for 30 ticks while the legs claim 1 cm per tick, no new field frames (the sensor sees the same thing; keep it simple)
        for (int k = 1; k <= 30; ++k) step(0.01 * k, 1.0f, {0, 1, 1, 1}, kY, false);
        if (gated) EXPECT_NEAR(f.pred[0], kFloorY + 0.05, 0.012) << "gated: the tread is still where the toe will land";
        else EXPECT_GT(std::fabs(f.pred[0] - (kFloorY + 0.05)) + (f.pred[2] < 0.5f ? 1.0 : 0.0), 0.012) << "ungated: the memory slid 0.30 m and the landing spot reads something else";
    }
}

TEST(TerrainMemory, RegistrationPullsASlidFrameBackOntoTheRiserItRemembers) {
    // the field sees a 5 cm tread from 0.40 m; the memory holds it; then the odometry claims 0.12 m forward while the
    // body (and the field's view) stays put.  Registered, the frame is pulled back and the tread stays 0.40 ahead; the
    // same without registration leaves the memory believing the tread is 0.28 ahead
    for (double gain : {0.5, 0.0}) {
        ogma::ParamMap p; p["register_gain"] = gain;
        Fix f(p); auto field = synth(kH, 30.0, 0.05, 0.40, 0.90, 0.40);
        uint64_t t = 1; double ox = 0.0;
        auto step = [&](double o, bool with_field) {
            f.bus.begin_tick(t); f.bus.publish("reality.proprio.odom", pv({float(o), 0, 0}, t)); f.bus.publish("reality.proprio.up_body", pv({0, 1, 0}, t));
            if (with_field) f.bus.publish("sense.tof8", pv(field, t));
            f.bus.publish("reality.proprio.foot_contact", pv({1, 1, 1, 1}, t)); f.bus.publish("reality.proprio.feet_xz", pv(kXZ, t)); f.bus.publish("reality.proprio.toe_y", pv(kY, t));
            f.bus.publish("reality.proprio.foot_load", pv({0.3f, 0.3f, 0.3f, 0.3f}, t)); f.m.tick(t); f.bus.end_tick(); ++t;
        };
        for (int k = 0; k < 5; ++k) step(0.0, true);
        for (int k = 1; k <= 12; ++k) { ox = 0.01 * k; step(ox, k % 3 == 0); }   // the legs claim 12 cm; the field keeps seeing the same tread
        for (int k = 0; k < 10; ++k) step(ox, true);
        double fx, fy, fyaw; f.m.frame(fx, fy, fyaw);
        // where does the memory put the tread's face now, in the body frame?  the nearest voxel ahead with h > floor + 3 cm
        double edge = 9.0;
        for (auto const& v : f.m.body_voxels()) if (std::fabs(v.lat) < 0.06 && v.h > kFloorY + 0.03 && v.fwd < edge) edge = v.fwd;
        if (gain > 0.0) { EXPECT_LT(std::fabs(fx), 0.05) << "registered: the frame came back to within 5 cm of where the body really is"; EXPECT_NEAR(edge, 0.40, 0.10) << "the remembered face, blurred by the frames stored while the pull-back converged"; EXPECT_GT(f.m.registrations(), 0); }
        else { EXPECT_NEAR(fx, 0.12, 1e-4) << "unregistered: the frame is the odometry's claim"; EXPECT_EQ(f.m.registrations(), 0); }
    }
}
