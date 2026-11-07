// SupportModel: the pose evaluator -- bearing toes, static load shares, the support margin and the lift margins.
#include <gtest/gtest.h>
#include <cmath>
#include "ogma/InProcessBus.hpp"
#include "ogma/modules/SupportModel.hpp"
namespace {
using ogma::SupportModel; using P = SupportModel::Point;
std::shared_ptr<ogma::ProprioToken> pv(std::vector<float> v) {
    auto p = std::make_shared<ogma::ProprioToken>(); p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)]; return p;
}
// a flat patch at height h (16 x 3 cm), full precision; cells in `holes` (fwd, lat) unknown
std::vector<float> flat_patch(float h, std::vector<P> const& risers = {}, float h2 = 0.0f) {
    const int N = 16; std::vector<float> v(4 + 2 * N * N, -99.0f);
    v[0] = N; v[1] = 0.03f; v[2] = 0; v[3] = 0;
    for (int i = 0; i < N; ++i) for (int j = 0; j < N; ++j) {
        const double fwd = (i - N / 2 + 0.5) * 0.03, lat = (j - N / 2 + 0.5) * 0.03;
        float hh = h;
        for (auto const& r : risers) if (fwd >= r.fwd) hh = h2;   // a riser: everything at or beyond r.fwd is at h2
        v[size_t(4 + i * N + j)] = hh; v[size_t(4 + N * N + i * N + j)] = 40000.0f;
    }
    return v;
}
struct Fix {
    ogma::InProcessBus bus; SupportModel m; std::vector<float> out;
    Fix(ogma::ParamMap p = {}) {
        m.set_id("support"); m.on_setup(&bus, p);
        bus.subscribe("support.state", ogma::SubscriptionKind::Direct, [this](std::string_view, ogma::MessagePtr msg){ auto t = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg); out.assign(t->values.data(), t->values.data() + t->values.size()); });
    }
    // toes [x, z] x 4 (FL, FR, RL, RR), their gravity heights, the body's up, the contacts, the loads, the patch
    void run(uint64_t t, std::vector<float> xz, std::vector<float> y, std::vector<float> up, std::vector<float> contact, std::vector<float> load, std::vector<float> const& patch) {
        bus.begin_tick(t);
        bus.publish("reality.proprio.feet_xz", pv(xz)); bus.publish("reality.proprio.toe_y", pv(y)); bus.publish("reality.proprio.up_body", pv(up));
        bus.publish("reality.proprio.foot_contact", pv(contact)); bus.publish("reality.proprio.foot_load", pv(load)); bus.publish("terrain.patch", pv(patch));
        m.tick(t); bus.end_tick();
    }
    float margin() const { return out[0]; } int n() const { return int(out[1]); }
    float gap(int i) const { return out[size_t(6 + 5 * i)]; } bool contact(int i) const { return out[size_t(8 + 5 * i)] > 0.5f; }
    float load(int i) const { return out[size_t(9 + 5 * i)]; } float lift(int i) const { return out[size_t(10 + 5 * i)]; }
};
// a symmetric stance: toes 7 cm out and 9 cm fore/aft, 9 cm below the body
const std::vector<float> kXZ = {0.07f, 0.09f, -0.07f, 0.09f, 0.07f, -0.09f, -0.07f, -0.09f};
const std::vector<float> kY = {-0.09f, -0.09f, -0.09f, -0.09f};
const std::vector<float> kUp = {0, 1, 0};
}

TEST(SupportModel, MarginAndLoadsOfTheBarePolygons) {
    std::vector<P> sq = {{0.09, 0.07}, {0.09, -0.07}, {-0.09, 0.07}, {-0.09, -0.07}};
    EXPECT_NEAR(SupportModel::margin_of(sq, P{0, 0}), 0.07, 1e-9);                 // the nearest edge is the side, 7 cm away
    EXPECT_NEAR(SupportModel::margin_of(sq, P{0.1, 0}), -0.01, 1e-9);              // 1 cm past the front edge
    std::vector<P> tri = {{0.09, 0.07}, {0.09, -0.07}, {-0.09, 0.07}};              // the right rear toe gone
    EXPECT_GT(SupportModel::margin_of(tri, P{0.02, 0.02}), 0.0);
    EXPECT_LT(SupportModel::margin_of(tri, P{-0.02, -0.02}), 0.0);                 // the weight is behind the diagonal
    EXPECT_LT(SupportModel::margin_of({sq[0], sq[1]}, P{0, 0}), 0.0);               // two toes hold nothing
    auto w = SupportModel::loads_of(sq, P{0, 0});
    for (auto x : w) EXPECT_NEAR(x, 0.25, 1e-9);                                    // centred: a quarter each
    w = SupportModel::loads_of(sq, P{0.045, 0});
    EXPECT_NEAR(w[0] + w[1], 0.75, 1e-9); EXPECT_NEAR(w[2] + w[3], 0.25, 1e-9);     // the weight forward: the fronts carry three quarters
    w = SupportModel::loads_of(tri, P{0.02, 0.02});
    double s = 0; for (auto x : w) { EXPECT_GE(x, 0.0); s += x; } EXPECT_NEAR(s, 1.0, 1e-9);
    w = SupportModel::loads_of(sq, P{0.12, 0});                                      // beyond the front edge: the rears unload to zero, not below
    EXPECT_NEAR(w[2], 0.0, 1e-9); EXPECT_NEAR(w[3], 0.0, 1e-9); EXPECT_NEAR(w[0] + w[1], 1.0, 1e-9);
}

TEST(SupportModel, LevelStanceOnFlatGround_AllBearQuarterEach) {
    Fix f; f.run(1, kXZ, kY, kUp, {1, 1, 1, 1}, {0.25f, 0.25f, 0.25f, 0.25f}, flat_patch(-0.09f));
    ASSERT_EQ(f.out.size(), 26u);
    EXPECT_EQ(f.n(), 4);
    for (int i = 0; i < 4; ++i) { EXPECT_TRUE(f.contact(i)); EXPECT_NEAR(f.gap(i), 0.0, 1e-4); EXPECT_NEAR(f.load(i), 0.25, 0.02); }
    EXPECT_NEAR(f.margin(), 0.07, 0.005);
    // a symmetric stance has NO static margin for any single lift: the remaining triangle's diagonal passes through the weight
    for (int i = 0; i < 4; ++i) EXPECT_NEAR(f.lift(i), 0.0, 1e-6);
    EXPECT_NEAR(f.m.contact_agreement(), 1.0, 1e-9);
    // the chassis's weight 6 cm forward (the net centre 3 cm forward, the legs being half the mass): the rear toes may leave, the fronts may not
    Fix g({{"com_z_m", ogma::ParamValue{0.06}}}); g.run(1, kXZ, kY, kUp, {1, 1, 1, 1}, {0.25f, 0.25f, 0.25f, 0.25f}, flat_patch(-0.09f));
    EXPECT_NEAR(g.out[2], 0.03, 1e-6);
    EXPECT_GT(g.lift(2), 0.01); EXPECT_GT(g.lift(3), 0.01); EXPECT_LT(g.lift(0), -0.01); EXPECT_LT(g.lift(1), -0.01);
    EXPECT_GT(g.load(0) + g.load(1), 0.6);   // and the fronts carry the shift
}

TEST(SupportModel, AToeHeldAboveTheGroundDoesNotBear_AndTheOthersCarryIt) {
    // the flag says the fourth toe is off; under support_source gap the memory says the same from its 4 cm gap
    for (auto const& src : {std::string("flag"), std::string("gap")}) {
    Fix f({{"support_source", ogma::ParamValue{src}}}); f.run(1, kXZ, {-0.09f, -0.09f, -0.09f, -0.05f}, kUp, {1, 1, 1, 0}, {0.3f, 0.3f, 0.3f, 0.0f}, flat_patch(-0.09f));
    EXPECT_FALSE(f.contact(3)); EXPECT_NEAR(f.gap(3), 0.04, 1e-4);
    EXPECT_EQ(f.n(), 3); EXPECT_NEAR(f.load(3), 0.0, 1e-9);
    double s = 0; for (int i = 0; i < 3; ++i) s += f.load(i); EXPECT_NEAR(s, 1.0, 1e-6);
    EXPECT_LT(f.lift(2), 0.0);   // the other rear toe may not leave now: two front toes cannot hold the body
    }
}

TEST(SupportModel, TheFlagDefinesTheBearingSet_AndAToeLeavingWithoutMarginIsAnUnsafeLift) {
    // the flag says the right rear toe is off although the memory puts it on the ground: the flag wins (the FSR is the sensor)
    Fix f; f.run(1, kXZ, kY, kUp, {1, 1, 1, 0}, {0.3f, 0.3f, 0.4f, 0.0f}, flat_patch(-0.09f));
    EXPECT_EQ(f.n(), 3); EXPECT_FALSE(f.contact(3)); EXPECT_NEAR(f.load(3), 0.0, 1e-9);
    EXPECT_LT(f.m.contact_agreement(), 1.0);   // and the gap prediction is graded as wrong on that toe
    // the left rear toe now leaves too, while its lift margin was negative: one unsafe lift counted, on leg 2
    f.run(2, kXZ, kY, kUp, {1, 1, 0, 0}, {0.5f, 0.5f, 0.0f, 0.0f}, flat_patch(-0.09f));
    auto d = f.m.diag_snapshot();
    EXPECT_EQ(d["lifts"][2].get<int>(), 1); EXPECT_EQ(d["unsafe_lifts"][2].get<int>(), 1); EXPECT_EQ(d["unsafe_lifts"][3].get<int>(), 0);
    EXPECT_EQ(f.n(), 2); EXPECT_LT(f.margin(), 0.0); EXPECT_GT(d["unsup_streak"].get<int>(), 0);
}

TEST(SupportModel, NoseUpOnARiser_TheWeightSitsBehindTheRearToes_SoLiftingARearToeIsUnsafe) {
    // the body 20 deg nose-up (up has a +z component), the front toes up on a riser 6 cm above the rear ground
    const float th = 20.0f * float(M_PI) / 180.0f; const std::vector<float> up = {0.0f, std::cos(th), std::sin(th)};
    // toes in the body frame: the fronts shortened (hung on the riser), the rears stretched back under the body
    const std::vector<float> xz = {0.07f, 0.06f, -0.07f, 0.06f, 0.07f, -0.05f, -0.07f, -0.05f};
    const std::vector<float> y = {-0.04f, -0.04f, -0.10f, -0.10f};   // gravity heights: the fronts 6 cm higher
    auto patch = flat_patch(-0.10f, {P{0.0, 0.0}}, -0.04f);           // the riser at the body's fore-aft centre
    Fix f; f.run(1, xz, y, up, {1, 1, 1, 1}, {0.2f, 0.2f, 0.3f, 0.3f}, patch);
    EXPECT_EQ(f.n(), 4);
    // levelled, the toes sit forward of the body's weight: the rears carry more than the fronts
    EXPECT_GT(f.load(2) + f.load(3), f.load(0) + f.load(1));
    // the body still stands on four, but the lift margins of the rear toes are small or negative: lifting one leaves the
    // weight on or behind the remaining diagonal
    EXPECT_LT(f.lift(2), 0.02); EXPECT_LT(f.lift(3), 0.02);
    EXPECT_GT(f.lift(0), f.lift(2)); EXPECT_GT(f.lift(1), f.lift(3));   // a front toe may leave more safely than a rear one
    // the same stance level: the rears lose that disadvantage
    Fix g; g.run(1, xz, y, kUp, {1, 1, 1, 1}, {0.25f, 0.25f, 0.25f, 0.25f}, patch);
    EXPECT_GT(g.lift(2), f.lift(2)); EXPECT_GT(g.lift(3), f.lift(3));
}

TEST(SupportModel, UnknownGroundFallsBackToTheContactFlag_AndOnItsSideNothingBears) {
    const int N = 16; std::vector<float> none(4 + 2 * N * N, -99.0f); none[0] = N; none[1] = 0.03f;
    for (int k = 0; k < N * N; ++k) none[size_t(4 + N * N + k)] = 0.0f;
    Fix f; f.run(1, kXZ, kY, kUp, {1, 1, 0, 1}, {0.3f, 0.3f, 0.0f, 0.4f}, none);
    EXPECT_EQ(f.n(), 3); EXPECT_FALSE(f.contact(2)); EXPECT_TRUE(f.contact(3));
    Fix g; g.run(1, kXZ, kY, {1, 0, 0}, {1, 1, 1, 1}, {0.25f, 0.25f, 0.25f, 0.25f}, flat_patch(-0.09f));
    EXPECT_EQ(g.n(), 0); EXPECT_NEAR(g.margin(), -0.10, 1e-6);
}
