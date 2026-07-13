// =============================================================================
// test_column_builder.cpp
//   ColumnBuilder — passive place-recorder (column = view-feature + heading + IMU).
//
//   1. ColumnSize           — default n_strips=6 → dim 22.
//   2. StripMeansNormalized — left 3 strips red, right 3 strips blue → strip RGB
//                             means correct + normalized to [0,1].
//   3. PoseTail             — [18]=sin, [19]=cos (copied), [20]=clamp(vel/4),
//                             [21]=clamp(ang/2).
//   4. RecordEveryGating    — with record_every=10, output present only on ticks
//                             0,10,20... (recorded_last_tick + bus freshness).
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/ColumnBuilder.hpp"
#include "ogma/Topics.hpp"

namespace {

// Image with the left half (cols < W/2) one colour and the right half another.
// With W = 2*n_strips/... we use W=6, n_strips=6 → 1 column per strip; left 3
// strips = colour L, right 3 strips = colour R.
std::shared_ptr<ogma::RawImageFrame>
half_frame(int W, int H, int lr, int lg, int lb, int rr, int rg, int rb) {
    auto img = std::make_shared<ogma::RawImageFrame>();
    img->width = W; img->height = H; img->channels = 3;
    img->pixels.assign(size_t(W) * H * 3, 0);
    for (int row = 0; row < H; ++row) {
        for (int col = 0; col < W; ++col) {
            long idx = (long(row) * W + col) * 3;
            bool left = (col < W / 2);
            img->pixels[idx + 0] = uint8_t(left ? lr : rr);
            img->pixels[idx + 1] = uint8_t(left ? lg : rg);
            img->pixels[idx + 2] = uint8_t(left ? lb : rb);
        }
    }
    return img;
}

std::shared_ptr<ogma::ProprioToken> vec2(float a, float b) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->values.resize(2); p->values[0] = a; p->values[1] = b;
    return p;
}
std::shared_ptr<ogma::ProprioToken> scal(float v) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->values.resize(1); p->values[0] = v;
    return p;
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::ColumnBuilder cb;
    explicit Fixture(ogma::ParamMap const& p = {}) {
        cb.set_id("column");
        cb.on_setup(&bus, p);
    }
    void run_tick(uint64_t t, std::shared_ptr<ogma::RawImageFrame> fr,
                  float hsin, float hcos, float vel, float ang) {
        bus.begin_tick(t);
        if (fr) bus.publish("host.video.color", fr);
        bus.publish("reality.proprio.heading_vec", vec2(hsin, hcos));
        bus.publish("reality.proprio.vel_ego", scal(vel));
        bus.publish("reality.proprio.ang_vel", scal(ang));
        cb.tick(t);
        bus.end_tick();
    }
};

}  // namespace

TEST(ColumnBuilder, ColumnSize) {
    Fixture f;  // n_strips=6 default
    f.run_tick(0, half_frame(6, 4, 255,0,0, 0,0,255), 0.0f, 1.0f, 0.0f, 0.0f);
    EXPECT_EQ(f.cb.dims(), 22);
    EXPECT_EQ(int(f.cb.last_column().size()), 22);
}

TEST(ColumnBuilder, StripMeansNormalized) {
    Fixture f;  // n_strips=6, W=6 → 1 col per strip
    // left 3 strips pure red, right 3 strips pure blue
    f.run_tick(0, half_frame(6, 4, 255,0,0, 0,0,255), 0.0f, 1.0f, 0.0f, 0.0f);
    auto const& c = f.cb.last_column();
    ASSERT_EQ(int(c.size()), 22);
    for (int s = 0; s < 3; ++s) {  // left strips: R=1, G=0, B=0
        EXPECT_NEAR(c[s*3 + 0], 1.0f, 1e-3f) << "left strip " << s << " R";
        EXPECT_NEAR(c[s*3 + 1], 0.0f, 1e-3f) << "left strip " << s << " G";
        EXPECT_NEAR(c[s*3 + 2], 0.0f, 1e-3f) << "left strip " << s << " B";
    }
    for (int s = 3; s < 6; ++s) {  // right strips: R=0, G=0, B=1
        EXPECT_NEAR(c[s*3 + 0], 0.0f, 1e-3f) << "right strip " << s << " R";
        EXPECT_NEAR(c[s*3 + 1], 0.0f, 1e-3f) << "right strip " << s << " G";
        EXPECT_NEAR(c[s*3 + 2], 1.0f, 1e-3f) << "right strip " << s << " B";
    }
}

TEST(ColumnBuilder, PoseTail) {
    Fixture f;
    const float hsin = 0.5f, hcos = 0.8660254f;   // copied straight through
    const float vel = 6.0f;                        // /4 = 1.5 → clamp to 1.0
    const float ang = -3.0f;                        // /2 = -1.5 → clamp to -1.0
    f.run_tick(0, half_frame(6, 4, 255,0,0, 0,0,255), hsin, hcos, vel, ang);
    auto const& c = f.cb.last_column();
    ASSERT_EQ(int(c.size()), 22);
    EXPECT_NEAR(c[18], hsin, 1e-5f);          // sin(heading) copied
    EXPECT_NEAR(c[19], hcos, 1e-5f);          // cos(heading) copied
    EXPECT_NEAR(c[20], 1.0f, 1e-5f);          // clamp(6/4,-1,1) = 1.0
    EXPECT_NEAR(c[21], -1.0f, 1e-5f);         // clamp(-3/2,-1,1) = -1.0

    // Unclamped (in-range) values pass straight through the normalization.
    Fixture g;
    g.run_tick(0, half_frame(6, 4, 255,0,0, 0,0,255), 0.0f, 1.0f, 2.0f, 1.0f);
    auto const& d = g.cb.last_column();
    EXPECT_NEAR(d[20], 0.5f, 1e-5f);          // 2/4
    EXPECT_NEAR(d[21], 0.5f, 1e-5f);          // 1/2
}

TEST(ColumnBuilder, RecordEveryGating) {
    Fixture f({{"record_every", int64_t{10}}});
    auto img = half_frame(6, 4, 255,0,0, 0,0,255);
    for (uint64_t t = 0; t <= 25; ++t) {
        f.run_tick(t, img, 0.0f, 1.0f, 0.0f, 0.0f);
        bool expect_emit = (t % 10 == 0);   // ticks 0,10,20
        EXPECT_EQ(f.cb.recorded_last_tick(), expect_emit)
            << "recorded_last_tick at tick " << t;
        // Bus freshness: a token published THIS tick carries tick_id == t.
        auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(
            f.bus.last_value("percept.column"));
        if (expect_emit) {
            ASSERT_NE(o, nullptr) << "expected a column at tick " << t;
            EXPECT_EQ(o->tick_id, t) << "fresh column at emit tick " << t;
            EXPECT_EQ(int(o->values.size()), 22);
        } else if (o != nullptr) {
            // No fresh publish this tick → held value is stale (older tick_id).
            EXPECT_LT(o->tick_id, t) << "no fresh column on non-emit tick " << t;
        }
    }
}
