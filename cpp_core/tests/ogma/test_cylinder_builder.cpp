// =============================================================================
// test_cylinder_builder.cpp
//   CylinderBuilder — heading-indexed panorama place-code (Pathway C2).
//
//   1. CapturesPerHeadingColour — bin b gets the colour shown while facing b.
//   2. SameSweepSameCylinder — identical sweep → identical panorama.
//   3. DifferentSweepsDistinguishable — different colour layout → distinguishable
//      (the anti-aliasing gate).
//   4. OnlyAccumulatesWhenActive — frames outside a saccade are ignored.
//   5. CarryOverUnfilledBins — a bin not visited this sweep keeps its prior value.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/modules/CylinderBuilder.hpp"
#include "ogma/Topics.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846f;

std::shared_ptr<ogma::RawImageFrame> frame(int W, int H, int r, int g, int b) {
    auto img = std::make_shared<ogma::RawImageFrame>();
    img->width = W; img->height = H; img->channels = 3;
    img->pixels.assign(size_t(W) * H * 3, 0);
    for (int k = 0; k < W * H; ++k) { img->pixels[k*3]=r; img->pixels[k*3+1]=g; img->pixels[k*3+2]=b; }
    return img;
}
std::shared_ptr<ogma::ProprioToken> scal(float v) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->values.resize(1); p->values[0] = v;
    return p;
}

struct Fixture {
    ogma::InProcessBus bus;
    ogma::CylinderBuilder cb;
    explicit Fixture(ogma::ParamMap const& p = {}) {
        cb.set_id("cylinder");
        cb.on_setup(&bus, p);
    }
    void run_tick(uint64_t t, std::shared_ptr<ogma::RawImageFrame> fr, float heading, float active) {
        bus.begin_tick(t);
        if (fr) bus.publish("host.video.color", fr);
        bus.publish("reality.proprio.heading", scal(heading));
        bus.publish("saccade.active", scal(active));
        cb.tick(t);
        bus.end_tick();
    }
};

float l2(std::vector<float> const& a, std::vector<float> const& b) {
    float s = 0; for (size_t i = 0; i < a.size(); ++i) { float d = a[i]-b[i]; s += d*d; }
    return std::sqrt(s);
}

// heading for bin b with n_bins=8: b*(2π/8) + small offset into the bin
float head_for_bin(int b) { return float(b) * (2.0f * kPi / 8.0f) + 0.05f; }

}  // namespace

TEST(CylinderBuilder, CapturesPerHeadingColour) {
    Fixture f;  // n_bins=8 default
    // Sweep: face bin 0 showing red, face bin 4 showing blue, then end the saccade.
    f.run_tick(0, frame(4,4,255,0,0), head_for_bin(0), 1.0f);
    f.run_tick(1, frame(4,4,0,0,255), head_for_bin(4), 1.0f);
    f.run_tick(2, nullptr,            head_for_bin(4), 0.0f);   // saccade end → finalize
    auto& pan = f.cb.panorama();
    ASSERT_EQ(int(pan.size()), 8*3);
    EXPECT_NEAR(pan[0*3+0], 1.0f, 1e-3f);  // bin0 red
    EXPECT_NEAR(pan[0*3+2], 0.0f, 1e-3f);
    EXPECT_NEAR(pan[4*3+2], 1.0f, 1e-3f);  // bin4 blue
    EXPECT_NEAR(pan[4*3+0], 0.0f, 1e-3f);
    EXPECT_EQ(f.cb.cylinders_built(), 1);
    EXPECT_EQ(f.cb.bins_filled(), 2);
}

TEST(CylinderBuilder, SameSweepSameCylinder) {
    Fixture a, b;
    for (Fixture* f : {&a, &b}) {
        f->run_tick(0, frame(4,4,200,40,40), head_for_bin(0), 1.0f);
        f->run_tick(1, frame(4,4,40,40,200), head_for_bin(4), 1.0f);
        f->run_tick(2, nullptr, head_for_bin(4), 0.0f);
    }
    EXPECT_NEAR(l2(a.cb.panorama(), b.cb.panorama()), 0.0f, 1e-5f)
        << "same spot scanned twice → same cylinder";
}

TEST(CylinderBuilder, DifferentSweepsDistinguishable) {
    Fixture a, b;
    // place A: bin0 red, bin4 blue
    a.run_tick(0, frame(4,4,230,30,30), head_for_bin(0), 1.0f);
    a.run_tick(1, frame(4,4,30,30,230), head_for_bin(4), 1.0f);
    a.run_tick(2, nullptr, head_for_bin(4), 0.0f);
    // place B: bin0 blue, bin4 red (swapped)
    b.run_tick(0, frame(4,4,30,30,230), head_for_bin(0), 1.0f);
    b.run_tick(1, frame(4,4,230,30,30), head_for_bin(4), 1.0f);
    b.run_tick(2, nullptr, head_for_bin(4), 0.0f);
    EXPECT_GT(l2(a.cb.panorama(), b.cb.panorama()), 0.5f)
        << "different surrounds → distinguishable cylinders (anti-aliasing)";
}

TEST(CylinderBuilder, OnlyAccumulatesWhenActive) {
    Fixture f;
    // frames with active=0 → ignored, no cylinder built
    for (uint64_t t = 0; t < 5; ++t) f.run_tick(t, frame(4,4,255,0,0), head_for_bin(t%8), 0.0f);
    EXPECT_EQ(f.cb.cylinders_built(), 0);
}

TEST(CylinderBuilder, CarryOverUnfilledBins) {
    Fixture f;
    // Sweep 1: fill bin0 (red) + bin4 (blue).
    f.run_tick(0, frame(4,4,255,0,0), head_for_bin(0), 1.0f);
    f.run_tick(1, frame(4,4,0,0,255), head_for_bin(4), 1.0f);
    f.run_tick(2, nullptr, head_for_bin(4), 0.0f);
    float bin4_blue = f.cb.panorama()[4*3+2];
    ASSERT_NEAR(bin4_blue, 1.0f, 1e-3f);
    // Sweep 2: only visit bin0 (green) — bin4 must carry over from sweep 1.
    f.run_tick(3, frame(4,4,0,255,0), head_for_bin(0), 1.0f);
    f.run_tick(4, nullptr, head_for_bin(0), 0.0f);
    EXPECT_NEAR(f.cb.panorama()[0*3+1], 1.0f, 1e-3f) << "bin0 now green";
    EXPECT_NEAR(f.cb.panorama()[4*3+2], 1.0f, 1e-3f) << "bin4 carried over (blue)";
}
