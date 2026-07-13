// =============================================================================
// test_visual_bearing.cpp
//   VisualBearing — vision food-direction perception (host.video.color → bearing).
//
//   1. CentroidLeft / CentroidRight / CentroidCenter — column centroid → bearing sign.
//   2. OccludedZeroConfidence — no green pixel → [0,0], mag 0.
//   3. ConfidenceScalesWithGreenFrac — more food pixels → larger green_frac.
//   4. LesionEmitsZero — force_lesion and lesion_after_ticks → [0,0].
//   5. NonFoodPixelsRejected — wall/sky pixels never counted.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/VisualBearing.hpp"
#include "ogma/Topics.hpp"

namespace {

// Build a RawImageFrame: sky background, food (green [26,230,51]) in `food_cols`
// across every row (a vertical stripe — rows are irrelevant to the lateral bearing).
std::shared_ptr<ogma::RawImageFrame> make_frame(int W, int H,
                                                std::vector<int> const& food_cols,
                                                uint64_t tick = 0) {
    auto img = std::make_shared<ogma::RawImageFrame>();
    img->tick_id  = tick;
    img->width    = W;
    img->height   = H;
    img->channels = 3;
    img->pixels.assign(size_t(W) * H * 3, 0);
    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            int idx = (j * W + i) * 3;
            img->pixels[idx]     = 20;   // sky [20,20,41]
            img->pixels[idx + 1] = 20;
            img->pixels[idx + 2] = 41;
        }
    }
    for (int j = 0; j < H; ++j) {
        for (int c : food_cols) {
            int idx = (j * W + c) * 3;
            img->pixels[idx]     = 26;   // food [26,230,51]
            img->pixels[idx + 1] = 230;
            img->pixels[idx + 2] = 51;
        }
    }
    return img;
}

struct Fixture {
    ogma::InProcessBus  bus;
    ogma::VisualBearing vb;
    explicit Fixture(ogma::ParamMap const& p = {}) {
        vb.set_id("visual_bearing");
        vb.on_setup(&bus, p);
    }
    void run_tick(uint64_t t, std::shared_ptr<ogma::RawImageFrame> frame) {
        bus.begin_tick(t);
        if (frame) bus.publish("host.video.color", frame);
        vb.tick(t);
        bus.end_tick();
    }
    std::shared_ptr<const ogma::ProprioToken> out() const {
        return std::dynamic_pointer_cast<const ogma::ProprioToken>(
            bus.last_value("percept.visual_bearing"));
    }
};

}  // namespace

// Build a frame with food of an ARBITRARY colour in `food_cols` (all rows).
std::shared_ptr<ogma::RawImageFrame> make_frame_rgb(int W, int H, std::vector<int> const& food_cols,
                                                    int fr, int fg, int fb, uint64_t tick = 0) {
    auto img = std::make_shared<ogma::RawImageFrame>();
    img->tick_id = tick; img->width = W; img->height = H; img->channels = 3;
    img->pixels.assign(size_t(W) * H * 3, 0);
    for (int j = 0; j < H; ++j) for (int i = 0; i < W; ++i) {
        int idx = (j * W + i) * 3; img->pixels[idx]=20; img->pixels[idx+1]=20; img->pixels[idx+2]=41; }
    for (int j = 0; j < H; ++j) for (int c : food_cols) {
        int idx = (j * W + c) * 3; img->pixels[idx]=fr; img->pixels[idx+1]=fg; img->pixels[idx+2]=fb; }
    return img;
}

// Non-scaffolded: the bug learns food's colour from EATING — here RED food, which the
// hard-coded green test would never detect. After learning it bears toward red food.
TEST(VisualBearing, LearnsFoodAppearanceFromEatingThenBears) {
    ogma::InProcessBus bus;
    ogma::VisualBearing vb; vb.set_id("vb");
    vb.on_setup(&bus, {{"learn_appearance", true}, {"hit_topic", std::string("events.hit")},
                       {"appearance_alpha", 1.0}, {"color_match_dist", 60.0}, {"central_ema_rate", 1.0}});
    std::vector<int> center; for (int c = 8; c <= 15; ++c) center.push_back(c);  // central cols

    // (1) Before any eat: red food in view but NOT yet learned → no bearing.
    bus.begin_tick(0); bus.publish("host.video.color", make_frame_rgb(24,24,{0,1,2,3},230,26,51,0)); vb.tick(0); bus.end_tick();
    EXPECT_FALSE(vb.have_proto());
    EXPECT_FLOAT_EQ(vb.last_mag(), 0.0f) << "can't recognize food before being taught";

    // (2) Approach + EAT: red food fills the centre, then a hit → learn red.
    bus.begin_tick(1); bus.publish("host.video.color", make_frame_rgb(24,24,center,230,26,51,1));
    bus.publish("events.hit", std::make_shared<ogma::EnvEvent>()); vb.tick(1); bus.end_tick();
    EXPECT_TRUE(vb.have_proto());
    EXPECT_GT(vb.proto_r(), vb.proto_g()) << "learned a RED-dominant prototype";
    EXPECT_GT(vb.proto_r(), vb.proto_b());

    // (3) Now red food on the LEFT → bears left (vx < 0), having LEARNED red = food.
    bus.begin_tick(2); bus.publish("host.video.color", make_frame_rgb(24,24,{0,1,2,3},230,26,51,2)); vb.tick(2); bus.end_tick();
    EXPECT_LT(vb.last_vx(), 0.0f) << "bears toward the learned-red food on the left";
    EXPECT_GT(vb.last_mag(), 0.0f);
}

TEST(VisualBearing, CentroidLeftIsNegativeRight) {
    Fixture f;
    f.run_tick(0, make_frame(24, 24, {0, 1, 2, 3}));   // far-left columns
    EXPECT_LT(f.vb.last_vx(), 0.0f) << "food on the left → bearing +right negative";
    EXPECT_GT(f.vb.last_vy(), 0.0f) << "forward component always positive (food ahead)";
    EXPECT_NEAR(f.vb.last_mag(), 1.0f, 1e-3f) << "unit bearing when visible";
}

TEST(VisualBearing, CentroidRightIsPositiveRight) {
    Fixture f;
    f.run_tick(0, make_frame(24, 24, {20, 21, 22, 23}));   // far-right columns
    EXPECT_GT(f.vb.last_vx(), 0.0f) << "food on the right → bearing +right positive";
    EXPECT_GT(f.vb.last_vy(), 0.0f);
}

TEST(VisualBearing, CentroidCenterIsForward) {
    Fixture f;
    // Symmetric columns straddling the centre → centroid u ≈ 0 → bearing ≈ forward.
    f.run_tick(0, make_frame(24, 24, {10, 11, 12, 13}));
    EXPECT_NEAR(f.vb.last_vx(), 0.0f, 1e-2f);
    EXPECT_NEAR(f.vb.last_vy(), 1.0f, 1e-2f);
}

TEST(VisualBearing, OccludedZeroConfidence) {
    Fixture f;
    f.run_tick(0, make_frame(24, 24, {}));   // no food pixels
    EXPECT_FLOAT_EQ(f.vb.last_vx(), 0.0f);
    EXPECT_FLOAT_EQ(f.vb.last_vy(), 0.0f);
    EXPECT_FLOAT_EQ(f.vb.last_mag(), 0.0f);
    EXPECT_FLOAT_EQ(f.vb.last_green_frac(), 0.0f);
    auto o = f.out();
    ASSERT_NE(o, nullptr);
    EXPECT_FLOAT_EQ(o->values[0], 0.0f);
    EXPECT_FLOAT_EQ(o->values[1], 0.0f);
}

TEST(VisualBearing, ConfidenceScalesWithGreenFrac) {
    Fixture f1, f2;
    f1.run_tick(0, make_frame(24, 24, {12}));                       // 1 column
    f2.run_tick(0, make_frame(24, 24, {8, 9, 10, 11, 12}));         // 5 columns
    EXPECT_GT(f2.vb.last_green_frac(), f1.vb.last_green_frac());
    EXPECT_NEAR(f1.vb.last_green_frac(), 1.0f / 24.0f, 1e-3f);
}

TEST(VisualBearing, ForceLesionEmitsZero) {
    ogma::ParamMap p{{"force_lesion", true}};
    Fixture f(p);
    f.run_tick(0, make_frame(24, 24, {20, 21, 22, 23}));   // food present, but lesioned
    EXPECT_TRUE(f.vb.lesioned());
    EXPECT_FLOAT_EQ(f.vb.last_mag(), 0.0f);
    EXPECT_FLOAT_EQ(f.vb.last_vx(), 0.0f);
}

TEST(VisualBearing, LesionAfterTicksFlips) {
    ogma::ParamMap p{{"lesion_after_ticks", int64_t{3}}};
    Fixture f(p);
    auto frame = make_frame(24, 24, {20, 21, 22, 23});
    for (uint64_t t = 0; t < 3; ++t) {
        f.run_tick(t, frame);
        EXPECT_FALSE(f.vb.lesioned()) << "not lesioned before tick 3";
        EXPECT_GT(f.vb.last_mag(), 0.0f);
    }
    f.run_tick(3, frame);
    EXPECT_TRUE(f.vb.lesioned());
    EXPECT_FLOAT_EQ(f.vb.last_mag(), 0.0f);
}

TEST(VisualBearing, NonFoodPixelsRejected) {
    Fixture f;
    // A frame full of wall [191,184,173] + sky — no pixel passes R<128 & B<128 & G>128.
    auto img = std::make_shared<ogma::RawImageFrame>();
    img->width = 24; img->height = 24; img->channels = 3;
    img->pixels.assign(24 * 24 * 3, 0);
    for (int k = 0; k < 24 * 24; ++k) {
        int idx = k * 3;
        img->pixels[idx]     = 191;   // wall
        img->pixels[idx + 1] = 184;
        img->pixels[idx + 2] = 173;
    }
    f.run_tick(0, img);
    EXPECT_FLOAT_EQ(f.vb.last_green_frac(), 0.0f);
    EXPECT_FLOAT_EQ(f.vb.last_mag(), 0.0f);
}
