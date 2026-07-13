// Klinotaxis — honest scalar-gradient follower by epistemic foraging. The defining property:
// it can recover a gradient's DIRECTION (a hidden external state) purely by WEAVING and
// lock-in detecting — steering its weave centre toward the rising direction.
#include <gtest/gtest.h>
#include "ogma/modules/Klinotaxis.hpp"
#include "ogma/InProcessBus.hpp"

#include <cmath>
#include <memory>

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float wrap(float a) { return std::atan2(std::sin(a), std::cos(a)); }
std::shared_ptr<ogma::ProprioToken> p1(float v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(1); p->values[0] = v; return p;
}
struct Fix {
    ogma::InProcessBus bus; ogma::Klinotaxis k; float prev_heading = 0.0f;
    Fix(ogma::ParamMap p = {}) { k.set_id("k"); k.on_setup(&bus, p); }
    // one closed-loop step: scalar = f(heading), IMU = actual yaw rate, bug follows perfectly
    float step(uint64_t t, float heading, float target) {
        float scalar = 2.0f + std::cos(heading - target);   // higher when facing the gradient
        float omega  = wrap(heading - prev_heading);        // reafferent yaw rate this step
        prev_heading = heading;
        bus.begin_tick(t);
        bus.publish("reality.proprio.scent_max", p1(scalar));
        bus.publish("reality.proprio.heading", p1(heading));
        bus.publish("reality.proprio.ang_vel", p1(omega));
        k.tick(t); bus.end_tick();
        auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(bus.last_value("percept.klino_heading"));
        float delta = std::atan2(float(o->values[0]), float(o->values[1]));
        return wrap(heading + delta);   // perfect-follow → next heading = weave heading
    }
};
}  // namespace

TEST(Klinotaxis, PublishesUnitHeading) {
    Fix f;
    f.step(0, 0.0f, 0.8f);
    auto o = std::dynamic_pointer_cast<const ogma::ProprioToken>(f.bus.last_value("percept.klino_heading"));
    ASSERT_NE(o, nullptr); ASSERT_EQ(o->values.size(), 2u);
    EXPECT_NEAR(std::hypot(float(o->values[0]), float(o->values[1])), 1.0f, 1e-3f);
}

TEST(Klinotaxis, SteersTowardGradient) {
    Fix f({{"period_ticks", 20.0}, {"weave_amp", 0.4}, {"steer_gain", 0.1}, {"lockin_lr", 0.03},
           {"scale_lr", 0.05}, {"adapt_period", false}});
    float target = 0.9f, heading = 0.0f;
    for (uint64_t t = 0; t < 3000; ++t) heading = f.step(t, heading, target);
    EXPECT_LT(std::fabs(wrap(f.k.base_heading() - target)), 0.5f)
        << "weave centre should steer toward the gradient direction (" << f.k.base_heading() << " vs " << target << ")";
    EXPECT_GT(f.k.lockin_mag(), 0.0f);
}

TEST(Klinotaxis, ShuffleOmegaBreaksLockin) {
    // (c) control: replace ω with noise → corr(dscalar/dt, ω) → 0 → no gradient lock →
    // the weave centre does NOT converge to the gradient (proves the lock-in does the work).
    Fix f({{"period_ticks", 20.0}, {"weave_amp", 0.4}, {"steer_gain", 0.1}, {"lockin_lr", 0.03},
           {"adapt_period", false}, {"shuffle_omega", true}});
    float target = 0.9f, heading = 0.0f;
    for (uint64_t t = 0; t < 3000; ++t) heading = f.step(t, heading, target);
    EXPECT_LT(f.k.lockin_mag(), 0.5f) << "shuffled ω should keep the lock-in weak (no real correlation)";
}

TEST(Klinotaxis, FleeSteersAway) {
    Fix f({{"period_ticks", 20.0}, {"weave_amp", 0.4}, {"steer_gain", 0.1}, {"lockin_lr", 0.03},
           {"scale_lr", 0.05}, {"adapt_period", false}, {"mode", int64_t{-1}}});
    float target = 0.0f, heading = 0.0f;
    for (uint64_t t = 0; t < 3000; ++t) heading = f.step(t, heading, target);
    // flee: weave centre should end up roughly opposite the gradient (|Δ| > π/2)
    EXPECT_GT(std::fabs(wrap(f.k.base_heading() - target)), kPi / 2.0f)
        << "flee should steer away from the gradient (" << f.k.base_heading() << ")";
}
