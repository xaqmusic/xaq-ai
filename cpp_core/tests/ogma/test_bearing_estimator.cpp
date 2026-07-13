// BearingEstimator (A′) — de-scaffold the analytic ScentCompass: VQ the nostril ring
// into learned prototypes, distil the compass bearing into each, then LESION the teacher
// so the percept runs learned-alone.
#include <gtest/gtest.h>
#include "ogma/modules/BearingEstimator.hpp"
#include "ogma/InProcessBus.hpp"

#include <memory>
#include <vector>

namespace {
std::shared_ptr<ogma::ProprioToken> ring(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->values.resize(int(v.size()));
    for (size_t i = 0; i < v.size(); ++i) p->values[i] = v[i];
    return p;
}
std::shared_ptr<ogma::ProprioToken> teach(float cx, float cy) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->values.resize(3); p->values[0] = cx; p->values[1] = cy; p->values[2] = 0.1f;
    return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::BearingEstimator be;
    Fix(ogma::ParamMap p = {}) { be.set_id("be"); be.on_setup(&bus, p); }
    void run(uint64_t t, std::vector<float> const& r, float cx, float cy) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent", ring(r));
        bus.publish("percept.scent_compass", teach(cx, cy));
        be.tick(t); bus.end_tick();
    }
};
const std::vector<float> R = {0.1f, 0.2f, 0.3f, 0.2f, 0.1f, 0.05f, 0.05f, 0.1f};
}  // namespace

TEST(BearingEstimator, DistillsTeacherBearing) {
    Fix f({{"bearing_lr", 0.3}});
    for (uint64_t t = 0; t < 40; ++t) f.run(t, R, 0.6f, 0.8f);
    EXPECT_GE(f.be.n_prototypes(), 1);
    EXPECT_NEAR(f.be.inferred_cx(), 0.6f, 0.05f) << "learned readout distilled toward the teacher";
    EXPECT_NEAR(f.be.inferred_cy(), 0.8f, 0.05f);
}

TEST(BearingEstimator, LesionFreezesReadoutTeacherIgnored) {
    Fix f({{"bearing_lr", 0.3}, {"lesion_after_ticks", int64_t{40}}});
    for (uint64_t t = 0; t < 40; ++t) f.run(t, R, 0.6f, 0.8f);   // distil [0.6,0.8]
    EXPECT_FALSE(f.be.lesioned());
    EXPECT_NEAR(f.be.inferred_cx(), 0.6f, 0.05f);
    // After the lesion the teacher flips, but the frozen readout must IGNORE it —
    // foraging now runs on the LEARNED percept alone (the scaffold is gone at runtime).
    for (uint64_t t = 40; t < 90; ++t) f.run(t, R, -0.6f, 0.8f);
    EXPECT_TRUE(f.be.lesioned());
    EXPECT_NEAR(f.be.inferred_cx(), 0.6f, 0.1f) << "readout frozen at pre-lesion teacher";
}

TEST(BearingEstimator, DistinctRingsGrowDistinctPrototypes) {
    Fix f({{"bearing_lr", 0.3}, {"novelty_thresh", 0.1}});
    for (uint64_t t = 0; t < 20; ++t) f.run(t, R, 0.6f, 0.8f);                    // ring A → bearing right
    std::vector<float> R2 = {0.05f,0.05f,0.1f,0.2f,0.3f,0.2f,0.1f,0.05f};         // ring B (different)
    for (uint64_t t = 20; t < 40; ++t) f.run(t, R2, -0.6f, 0.8f);                 // ring B → bearing left
    EXPECT_GE(f.be.n_prototypes(), 2) << "distinct ring patterns → distinct learned percepts";
}
