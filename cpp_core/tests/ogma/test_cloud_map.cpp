// =============================================================================
// test_cloud_map.cpp
//   ogma::CloudMap — the sweep's point cloud, cached by place (microduck, 2026-09-13).
//
//   What is locked down, and the failure each guard answers for:
//     1. InertWithoutAnInputTopic — the default-off contract: no input_topic, no cloud, no publish.
//     2. StillnessOpensItAndATwitchDoesNotCloseIt — the hysteresis.  Closing on the first non-still
//        tick chopped one stop's sweep into fragments of 50, 70 and 943 voxels (measured).
//     3. DerotationCancelsTheBodysOwnYaw — the SIGN.  A body that yaws +d sees a world-fixed point
//        rotated by -d; the cloud must turn it back by +d.  A wrong sign doubles the smear instead
//        of cancelling it, and on real data that reads as "more distinct voxels", which is easy to
//        misread as more detail.  Tested on a synthetic body so the sign cannot hide.
//     4. CachesByPlaceAndAnIdenticalRevisitReadsUnchanged — revisit_change is 0 for the same scene
//        from the same pose, and rises when something new stands there.
//     5. RevisitAlignsThroughTheOdometryPose — a revisit from a different position and heading is
//        lined up through the two anchors before comparing; unaligned it would read as all-new.
//     6. ProfileIsBoundedAndSeesAFloorBreak — the published reduction has kProfile values in [0,1]
//        and registers an object standing on the floor.
//     7. AFlatFloorIsNotAFloorBreak — a voxel is classified by the mean height of its points.  The
//        ground layer's CENTRE is exactly break_lo, and a centre test counted the whole floor as
//        things standing on it — in the break profile the object EPM learns from.
//     8. ViewIsTheNearestOffFloorReturnPerSector — the cloud as a place map's view: empty with no cloud,
//        unmoved by a bare floor, and the nearest thing standing up in the sectors it stands in.
//     9. ThingsAreOffWithoutATopic — the things reduction is not computed and nothing is published unless
//        a graph asks for it (the things phase's gain-0 guard).
//    10. ASmallCubeIsAThingAndARisingPostIsNot — the stack rule in the module: a 12 cm cube is small and
//        attended; a narrow post whose heights chain up past small_top is an obstacle even though its
//        footprint is small; the descriptor is kThing values in [0,1] and the bearing points at the cube.
//    11. BearingTurnsWithTheBodysYawDrift — the bearing is in the BODY frame: the cloud is anchored on the
//        opening yaw, and a body that has since yawed left sees the same thing to its right.
//    12. FilingKeepsTheThingsAndClearsTheAttention — the filed cloud's clusters survive for the record;
//        the live list, the attention and the bearing are cleared, and the bearing topic reads 0.
//    13. AFootprintFloorLeavesFragmentsUnattended — small_ext_min: a one-column fragment is not attended,
//        the cube behind it is; and things_shape publishes the 5-dim descriptor without the sampling dims.
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/CloudMap.hpp"

namespace {

using ogma::ParamMap;
using ogma::ParamValue;
using Pt = std::array<double, 3>;

constexpr int kCast = 5 + 3 * ogma::CloudMap::kZones;

ParamMap params(bool with_input = true) {
    ParamMap p;
    if (with_input) p["input_topic"] = std::string("in.points");
    p["place_topic"]  = std::string("in.place");
    p["output_topic"] = std::string("out.cloud");
    p["change_topic"] = std::string("out.change");
    p["still_ticks"]  = int64_t{1};
    p["move_ticks"]   = int64_t{5};
    p["cache_size"]   = int64_t{4};
    return p;
}

// A dense patch standing on the floor about a metre ahead: 8 x 8 points 5 cm apart, 5 cm up.
// Coordinates sit a quarter-voxel off every 4 cm boundary so voxel membership is unambiguous.
std::vector<Pt> patch() {
    std::vector<Pt> w;
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
            w.push_back({0.805 + 0.05 * i, -0.195 + 0.05 * j, 0.05});
    return w;
}

// What a body at odometry pose (ox, oy, yaw) sees of world-fixed points: the points moved into its
// own levelled frame, p_body = R(-yaw) * (p_world - o).
std::vector<Pt> seen_from(const std::vector<Pt>& world, double ox, double oy, double yaw) {
    std::vector<Pt> b;
    const double c = std::cos(-yaw), s = std::sin(-yaw);
    for (auto const& p : world) {
        const double dx = p[0] - ox, dy = p[1] - oy;
        b.push_back({c * dx - s * dy, s * dx + c * dy, p[2]});
    }
    return b;
}

struct Rig {
    ogma::InProcessBus bus;
    ogma::CloudMap m;
    uint64_t t = 0;

    explicit Rig(ParamMap const& p) {
        m.set_id("cloud");
        m.on_setup(&bus, p);
    }

    // the sensor origin appended to the cast (NaN = not carried); vacated voxels need it
    Pt origin{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
    // the empty zones' ray ends appended after the origin (--tof-free-rays); empty = the block is not carried
    std::vector<Pt> far_pts;
    void cast(bool still, double yaw, double ox, double oy, int winner, std::vector<Pt> const& pts) {
        bus.begin_tick(t);
        auto tok = std::make_shared<ogma::ProprioToken>();
        tok->values = Eigen::VectorXf::Constant(kCast + 3 + (far_pts.empty() ? 0 : 3 * ogma::CloudMap::kZones),
                                                std::numeric_limits<float>::quiet_NaN());
        for (int k = 0; k < 3; ++k) tok->values[kCast + k] = float(origin[size_t(k)]);
        for (size_t i = 0; i < far_pts.size() && i < size_t(ogma::CloudMap::kZones); ++i)
            for (int k = 0; k < 3; ++k) tok->values[int(kCast + 3 + 3 * i + k)] = float(far_pts[i][size_t(k)]);
        tok->values[0] = still ? 1.0f : 0.0f;
        tok->values[1] = float(yaw);
        tok->values[2] = 0.12f;
        tok->values[3] = float(ox);
        tok->values[4] = float(oy);
        for (size_t i = 0; i < pts.size() && i < size_t(ogma::CloudMap::kZones); ++i)
            for (int k = 0; k < 3; ++k) tok->values[int(5 + 3 * i + k)] = float(pts[i][size_t(k)]);
        tok->sensor = "tof_points";
        bus.publish("in.points", tok);
        if (winner >= 0) {
            auto rt = std::make_shared<ogma::RealityToken>();
            rt->winner_id = winner;
            bus.publish("in.place", rt);
        }
        m.tick(t);
        bus.end_tick();
        ++t;
    }

    // stand still at a pose for n casts of the same view, then walk away until the cloud is filed
    void visit(int place, double ox, double oy, double yaw, std::vector<Pt> const& world, int n = 6) {
        const auto view = seen_from(world, ox, oy, yaw);
        for (int i = 0; i < n; ++i) cast(true, yaw, ox, oy, place, view);
        for (int i = 0; i < 6 && m.is_open(); ++i) cast(false, yaw, ox, oy, place, {});
    }
};

}  // namespace

TEST(CloudMap, InertWithoutAnInputTopic) {
    Rig r(params(/*with_input=*/false));
    for (int i = 0; i < 20; ++i) r.cast(true, 0.0, 0.0, 0.0, 3, patch());
    EXPECT_FALSE(r.m.is_open());
    EXPECT_EQ(r.m.voxels(), 0);
    EXPECT_EQ(r.bus.last_value("out.cloud"), nullptr) << "no input topic must mean no publication at all";
}

TEST(CloudMap, StillnessOpensItAndATwitchDoesNotCloseIt) {
    Rig r(params());
    const auto view = patch();
    r.cast(true, 0.0, 0.0, 0.0, 3, view);
    ASSERT_TRUE(r.m.is_open()) << "still_ticks 1: one still tick opens the cloud";
    const int v0 = r.m.voxels();
    ASSERT_GT(v0, 0);
    for (int i = 0; i < 4; ++i) r.cast(false, 0.0, 0.0, 0.0, 3, view);   // four twitches, move_ticks 5
    EXPECT_TRUE(r.m.is_open()) << "fewer than move_ticks non-still ticks must not file the cloud";
    EXPECT_EQ(r.m.voxels(), v0) << "a moving tick must not contribute points";
    r.cast(true, 0.0, 0.0, 0.0, 3, view);                                  // settles again
    for (int i = 0; i < 4; ++i) r.cast(false, 0.0, 0.0, 0.0, 3, {});
    EXPECT_TRUE(r.m.is_open()) << "the still tick in between must reset the move run";
    r.cast(false, 0.0, 0.0, 0.0, 3, {});
    EXPECT_TRUE(r.m.just_closed()) << "move_ticks consecutive non-still ticks file it";
    EXPECT_FALSE(r.m.is_open());
    EXPECT_EQ(int(r.m.last_filed_voxels().size() / 5), v0) << "the dump is [ix, iy, iz, hits, mean height mm]";
    EXPECT_EQ(r.m.last_key(), 3);
}

TEST(CloudMap, DerotationCancelsTheBodysOwnYaw) {
    Rig r(params());
    const auto world = patch();
    r.cast(true, 0.0, 0.0, 0.0, 3, seen_from(world, 0.0, 0.0, 0.0));      // anchored on yaw 0
    const int v0 = r.m.voxels();
    ASSERT_GT(v0, 40);
    // the trunk's heading drifts half a radian while standing: the same world, seen from yaw 0.5
    r.cast(true, 0.5, 0.0, 0.0, 3, seen_from(world, 0.0, 0.0, 0.5));
    const int v1 = r.m.voxels();
    EXPECT_LE(v1 - v0, v0 / 10)
        << "the rotated cast should land on the voxels the first cast made; " << (v1 - v0)
        << " new voxels of " << v0 << " means the de-rotation is not cancelling the body's yaw "
        << "(a wrong sign rotates by -2d instead of 0)";
}

TEST(CloudMap, CachesByPlaceAndAnIdenticalRevisitReadsUnchanged) {
    Rig r(params());
    const auto world = patch();
    r.visit(7, 0.0, 0.0, 0.0, world);
    EXPECT_EQ(r.m.cached(), 1);
    EXPECT_LT(r.m.revisit_change(), 0.0) << "the first visit to a place has nothing to compare against";

    r.visit(7, 0.0, 0.0, 0.0, world);
    EXPECT_EQ(r.m.cached(), 1) << "the same place overwrites its own entry";
    EXPECT_NEAR(r.m.revisit_change(), 0.0, 1e-9) << "the same scene from the same pose is unchanged";

    auto moved = world;                                                    // a thing now stands beside it
    for (int i = 0; i < 16; ++i) moved[size_t(i)] = {0.805 + 0.05 * (i % 4), 0.31 + 0.05 * (i / 4), 0.09};
    r.visit(7, 0.0, 0.0, 0.0, moved);
    EXPECT_GT(r.m.revisit_change(), 0.15) << "new geometry at a known place must read as change";

    r.visit(9, 0.0, 0.0, 0.0, world);
    EXPECT_EQ(r.m.cached(), 2);
    EXPECT_LT(r.m.revisit_change(), 0.0) << "a new place has no stored cloud";
}

TEST(CloudMap, RevisitAlignsThroughTheOdometryPose) {
    const auto world = patch();
    {   // a translation of exactly eight voxels: alignment must be exact
        Rig r(params());
        r.visit(5, 0.0, 0.0, 0.0, world);
        r.visit(5, -0.32, 0.0, 0.0, world);
        EXPECT_NEAR(r.m.revisit_change(), 0.0, 1e-9)
            << "the same world seen from 32 cm further back must line up through the anchors";
    }
    {   // a turned body: allow the one-voxel remaps a rotation of voxel centres introduces
        Rig r(params());
        r.visit(5, 0.0, 0.0, 0.0, world);
        r.visit(5, 0.0, 0.0, 0.4, world);
        EXPECT_LT(r.m.revisit_change(), 0.5)
            << "the same world from a heading 0.4 rad away must mostly line up; ~1.0 means the "
            << "revisit transform is not undoing the turn";
        EXPECT_NEAR(r.m.revisit_anchor_dist(), 0.0, 1e-6);
    }
}

TEST(CloudMap, ProfileIsBoundedAndSeesAFloorBreak) {
    Rig r(params());
    r.cast(true, 0.0, 0.0, 0.0, 3, patch());
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.cloud"));
    ASSERT_TRUE(tok);
    ASSERT_EQ(int(tok->values.size()), ogma::CloudMap::kProfile);
    for (int i = 0; i < tok->values.size(); ++i) {
        EXPECT_GE(tok->values[i], 0.0f) << "profile[" << i << "]";
        EXPECT_LE(tok->values[i], 1.0f) << "profile[" << i << "]";
    }
    float mass = 0.0f;
    for (int s = 0; s < ogma::CloudMap::kSectors; ++s) mass += tok->values[3 * ogma::CloudMap::kSectors + s];
    EXPECT_GT(mass, 0.0f) << "a patch 5 cm off the floor, a metre ahead, is a floor break";
    auto ch = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.change"));
    ASSERT_TRUE(ch);
    EXPECT_EQ(int(ch->values.size()), 2);
}

TEST(CloudMap, AFlatFloorIsNotAFloorBreak) {
    Rig r(params());
    std::vector<Pt> floor;
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
            floor.push_back({0.605 + 0.1 * i, -0.395 + 0.1 * j, 0.01});      // 1 cm: in the 0-4 cm layer
    r.cast(true, 0.0, 0.0, 0.0, 3, floor);
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.cloud"));
    ASSERT_TRUE(tok);
    ASSERT_GT(r.m.voxels(), 30);
    for (int s = 0; s < ogma::CloudMap::kSectors; ++s)
        EXPECT_EQ(tok->values[3 * ogma::CloudMap::kSectors + s], 0.0f)
            << "sector " << s << ": a floor 1 cm down counted as something standing on it";
    EXPECT_EQ(tok->values[4 * ogma::CloudMap::kSectors + 0], 0.0f) << "break mass must be zero on a bare floor";
    const auto& dump = r.m.last_filed_voxels();
    EXPECT_TRUE(dump.empty()) << "nothing filed yet";
}

TEST(CloudMap, ViewIsTheNearestOffFloorReturnPerSector) {
    Rig r(params());
    for (float v : r.m.view()) EXPECT_EQ(v, 1.0f) << "no cloud open: an empty view";
    std::vector<Pt> floor;
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
            floor.push_back({0.605 + 0.1 * i, -0.395 + 0.1 * j, 0.01});
    r.cast(true, 0.0, 0.0, 0.0, 3, floor);
    ASSERT_TRUE(r.m.is_open());
    for (float v : r.m.view()) EXPECT_EQ(v, 1.0f) << "a bare floor is not something to see";
    r.cast(true, 0.0, 0.0, 0.0, 3, patch());
    const auto v = r.m.view();
    ASSERT_EQ(int(v.size()), ogma::CloudMap::kSectors);
    // the patch starts 0.82 m out (its nearest voxel centre) and spans bearings within about +-12 deg
    const float nearest = *std::min_element(v.begin(), v.end());
    EXPECT_NEAR(nearest, 0.82f / 4.0f, 0.01f) << "the nearest off-floor return, over 4 m";
    EXPECT_LT(v[3], 1.0f);
    EXPECT_LT(v[4], 1.0f);
    EXPECT_EQ(v[0], 1.0f) << "nothing out at -60 deg";
    EXPECT_EQ(v[size_t(ogma::CloudMap::kSectors - 1)], 1.0f) << "nothing out at +60 deg";
}

// ---------------------------------------------------------------------------------------------- things

namespace {

ParamMap things_params() {
    ParamMap p = params();
    p["things_topic"] = std::string("out.thing");
    p["thing_bearing_topic"] = std::string("out.thing_bearing");
    p["things_every"] = int64_t{1};
    return p;
}

// A 12 cm cube standing a little under a metre ahead: a 3 x 3 x 3 lattice of returns 4 cm apart, each a
// quarter-voxel off the boundaries, heights 3 / 7 / 11 cm.  Stack top 11 cm, footprint 12 cm: SMALL.
std::vector<Pt> cube() {
    std::vector<Pt> w;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) w.push_back({0.81 + 0.04 * i, -0.03 + 0.04 * j, 0.03 + 0.04 * k});
    return w;
}

// A narrow post 1.5 m ahead and to the left: 8 cm wide, returns every 4 cm from 3 cm up to 35 cm.  Its
// footprint would pass as small; its chain climbs past small_top and it is an obstacle.
std::vector<Pt> post() {
    std::vector<Pt> w;
    for (int j = 0; j < 2; ++j)
        for (int k = 0; k < 9; ++k) w.push_back({1.51, 0.31 + 0.04 * j, 0.03 + 0.04 * k});
    return w;
}

// Feed a world of any size as as many still casts as it takes (a cast carries at most kZones points).
void cast_world(Rig& r, double yaw, const std::vector<Pt>& body_pts, int winner = 3) {
    for (size_t at = 0; at < body_pts.size(); at += size_t(ogma::CloudMap::kZones)) {
        std::vector<Pt> chunk(body_pts.begin() + long(at),
                              body_pts.begin() + long(std::min(body_pts.size(), at + size_t(ogma::CloudMap::kZones))));
        r.cast(true, yaw, 0.0, 0.0, winner, chunk);
    }
}

std::vector<Pt> scene() {
    auto w = cube();
    const auto p = post();
    w.insert(w.end(), p.begin(), p.end());
    return w;
}

}  // namespace

TEST(CloudMap, ThingsAreOffWithoutATopic) {
    Rig r(params());
    cast_world(r, 0.0, scene());
    EXPECT_TRUE(r.m.is_open());
    EXPECT_TRUE(r.m.things().empty()) << "no things topic: the reduction is not run";
    EXPECT_EQ(r.m.attended(), -1);
    EXPECT_EQ(r.bus.last_value("out.thing"), nullptr);
    EXPECT_EQ(r.bus.last_value("out.thing_bearing"), nullptr);
    EXPECT_EQ(r.m.cluster_things().size(), 2u) << "the reduction itself still works on demand";
}

TEST(CloudMap, ASmallCubeIsAThingAndARisingPostIsNot) {
    Rig r(things_params());
    cast_world(r, 0.0, scene());
    const auto& th = r.m.things();
    ASSERT_EQ(th.size(), 2u) << "two 8-connected break-band clusters";
    // sorted by range: the cube first
    EXPECT_NEAR(th[0].rng, std::hypot(0.85, 0.01), 0.03);
    EXPECT_TRUE(th[0].small);
    EXPECT_NEAR(th[0].top, 0.11, 0.02);
    EXPECT_NEAR(th[0].ext, 0.12, 1e-6);
    EXPECT_EQ(th[0].ncols, 9);
    EXPECT_EQ(th[0].chain, 3);
    EXPECT_FALSE(th[1].small) << "the post's footprint is small but its chain climbs to 35 cm";
    EXPECT_NEAR(th[1].ext, 0.08, 1e-6);
    EXPECT_GT(th[1].top, 0.3);
    EXPECT_EQ(r.m.attended(), 0);

    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing"));
    ASSERT_NE(tok, nullptr);
    ASSERT_EQ(tok->values.size(), ogma::CloudMap::kThing);
    for (int i = 0; i < tok->values.size(); ++i) {
        EXPECT_GE(tok->values[i], 0.0f) << "dim " << i;
        EXPECT_LE(tok->values[i], 1.0f) << "dim " << i;
    }
    EXPECT_NEAR(tok->values[0], 0.11 / 0.20, 0.1) << "top / break_hi";
    EXPECT_NEAR(tok->values[1], 0.12 / 0.20, 1e-5) << "footprint / small_ext";
    EXPECT_NEAR(tok->values[2], 1.0, 1e-5) << "a cube is round in plan";

    auto b = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing_bearing"));
    ASSERT_NE(b, nullptr);
    ASSERT_EQ(b->values.size(), 4);
    EXPECT_FLOAT_EQ(b->values[3], 0.0f) << "seen from a stop, not a walking cloud";
    EXPECT_NEAR(b->values[0], 0.0, 0.05) << "straight ahead: no rightward component";
    EXPECT_NEAR(b->values[1], 1.0, 0.01) << "forward";
    EXPECT_NEAR(b->values[2], 1.0 - th[0].rng / 2.5, 1e-4) << "proximity = 1 - range / max_range (things_range 0)";
}

TEST(CloudMap, BearingTurnsWithTheBodysYawDrift) {
    Rig r(things_params());
    const auto world = cube();
    cast_world(r, 0.0, seen_from(world, 0.0, 0.0, 0.0));             // anchored on yaw 0, the cube dead ahead
    ASSERT_EQ(r.m.attended(), 0);
    // the trunk drifts 0.3 rad to the LEFT while standing; the same world, seen from yaw 0.3
    cast_world(r, 0.3, seen_from(world, 0.0, 0.0, 0.3));
    ASSERT_EQ(r.m.things().size(), 1u) << "the de-rotation keeps one cluster";
    const auto b = r.m.thing_bearing();
    EXPECT_NEAR(b[0], std::sin(0.3), 0.03) << "the cube is now to the body's RIGHT";
    EXPECT_NEAR(b[1], std::cos(0.3), 0.03);
}

TEST(CloudMap, FilingKeepsTheThingsAndClearsTheAttention) {
    Rig r(things_params());
    cast_world(r, 0.0, scene());
    ASSERT_EQ(r.m.attended(), 0);
    for (int i = 0; i < 6 && r.m.is_open(); ++i) r.cast(false, 0.0, 0.0, 0.0, 3, {});
    ASSERT_FALSE(r.m.is_open());
    EXPECT_EQ(r.m.last_filed_things().size(), 2u) << "the filed cloud's clusters are kept for the record";
    EXPECT_TRUE(r.m.last_filed_things()[0].small);
    EXPECT_TRUE(r.m.things().empty());
    EXPECT_EQ(r.m.attended(), -1);
    const auto b = r.m.thing_bearing();
    EXPECT_EQ(b[0], 0.0f); EXPECT_EQ(b[1], 0.0f); EXPECT_EQ(b[2], 0.0f);
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing_bearing"));
    ASSERT_NE(tok, nullptr);
    EXPECT_EQ(tok->values[2], 0.0f) << "nothing attended reads as proximity 0";
}

TEST(CloudMap, AFootprintFloorLeavesFragmentsUnattended) {
    ParamMap p = things_params();
    p["small_ext_min"] = 0.08;
    p["things_shape"] = true;
    Rig r(p);
    // a single-column fragment 0.5 m ahead (one voxel footprint, 5 cm up), and the cube behind it
    auto world = cube();
    world.push_back({0.51, 0.01, 0.05});
    cast_world(r, 0.0, world);
    const auto& th = r.m.things();
    ASSERT_EQ(th.size(), 2u);
    EXPECT_NEAR(th[0].ext, 0.04, 1e-6) << "the fragment is nearest";
    EXPECT_FALSE(th[0].small) << "a 4 cm footprint is below the floor";
    EXPECT_TRUE(th[1].small);
    EXPECT_EQ(r.m.attended(), 1) << "attention passes over the fragment to the cube";
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing"));
    ASSERT_NE(tok, nullptr);
    EXPECT_EQ(tok->values.size(), ogma::CloudMap::kThingShape);
    EXPECT_EQ(r.m.thing_dims(), ogma::CloudMap::kThingShape);
    EXPECT_NEAR(tok->values[1], 0.12 / 0.20, 1e-5) << "footprint / small_ext";
    EXPECT_NEAR(tok->values[4], 3.0 / 5.0, 1e-5) << "chain / 5";
}


// The walking cloud (2026-09-19): with walk_cloud on, casts from a MOVING body are translated by the odometry's
// displacement from the anchor, so a world-fixed patch seen from 0.4 m further on lands on the voxels the first
// cast made; without it a moving tick contributes nothing.  A walking cloud is never cached as a place.
TEST(CloudMap, AWalkingCloudTranslatesByTheOdometry) {
    ParamMap p = params();
    p["walk_cloud"] = true; p["walk_reset_m"] = 2.0;
    Rig r(p);
    const auto world = patch();
    r.cast(false, 0.0, 0.0, 0.0, 3, seen_from(world, 0.0, 0.0, 0.0));     // moving from the first tick: a walking cloud opens
    ASSERT_TRUE(r.m.is_walking_cloud());
    const int v0 = r.m.voxels();
    ASSERT_GT(v0, 40);
    r.cast(false, 0.0, 0.4, 0.0, 3, seen_from(world, 0.4, 0.0, 0.0));     // 0.4 m further along +x (ten voxels)
    EXPECT_LE(r.m.voxels() - v0, v0 / 10) << "translated by the odometry, the second cast lands on the first's voxels";
    r.cast(false, 0.0, 2.5, 0.0, 3, seen_from(world, 2.5, 0.0, 0.0));     // past walk_reset_m: filed and reopened
    EXPECT_EQ(r.m.cached(), 0) << "a walking cloud is never cached as a place";
    EXPECT_TRUE(r.m.is_walking_cloud());
    if (auto b = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing_bearing")))
        { ASSERT_EQ(int(b->values.size()), 4); EXPECT_FLOAT_EQ(b->values[3], 1.0f) << "the bearing token says: seen from a walking cloud"; }
}

TEST(CloudMap, WithoutWalkCloudAMovingTickContributesNothing) {
    Rig r(params());
    r.cast(false, 0.0, 0.0, 0.0, 3, patch());
    EXPECT_FALSE(r.m.is_open());
    EXPECT_EQ(r.m.voxels(), 0);
}

// MOVERS (chasing moving things, 2026-09-27, stage 0): the recency window.  A cloud accumulates, so a thing that
// moves smears; cluster_recent runs the same rule over only the voxels seen in the last N ticks, so a cluster has a
// position at a time.  A cube cast at 0.85 m and again, ten casts later, at 1.25 m is in the whole cloud twice and
// is one small cube at its LATEST position through a short window.
TEST(CloudMap, ARecencyWindowSeesAMovedCubeWhereItIsNow) {
    Rig r(things_params());
    cast_world(r, 0.0, cube());                       // the cube at 0.81-0.89 m
    for (int i = 0; i < 10; ++i) r.cast(true, 0.0, 0.0, 0.0, 3, {});   // ten empty casts pass
    std::vector<Pt> moved;
    for (auto p : cube()) { p[0] += 0.40; moved.push_back(p); }
    cast_world(r, 0.0, moved);                        // the same cube 40 cm further out
    const auto all = r.m.cluster_things();
    ASSERT_EQ(all.size(), 2u) << "the whole cloud keeps both positions: the cube is in it twice";
    const auto recent = r.m.cluster_recent(3);
    ASSERT_EQ(recent.size(), 1u) << "through a 3-tick window only the latest cast's voxels remain";
    EXPECT_TRUE(recent[0].small);
    EXPECT_NEAR(recent[0].cx, 0.85 + 0.40, 0.03);
    EXPECT_EQ(r.m.last_tick(), r.t - 1);
    EXPECT_EQ(r.m.cluster_recent(1000).size(), 2u) << "a window wider than the run sees both positions";
    // freshness: the cube's voxels at the new position were first seen inside the window
    EXPECT_NEAR(recent[0].fresh, 1.0, 1e-9) << "every voxel of a thing that just moved is new to the cloud";
    EXPECT_NEAR(recent[0].age, 0.0, 1e-9);
    // the same cube cast again where it is, three ticks later: through a one-tick window its voxels are all
    // re-hits, first seen before the window -- not fresh.  This is what tells a mover from a static thing.
    for (int i = 0; i < 2; ++i) r.cast(true, 0.0, 0.0, 0.0, 3, {});
    cast_world(r, 0.0, moved);
    const auto again = r.m.cluster_recent(1);
    ASSERT_EQ(again.size(), 1u);
    EXPECT_NEAR(again[0].fresh, 0.0, 1e-9) << "a thing seen where it was is not fresh";
    EXPECT_NEAR(again[0].age, 3.0, 1e-9) << "its voxels are three ticks old";
    EXPECT_NEAR(r.m.cluster_things()[0].fresh, 0.0, 1e-9) << "the whole-cloud rule reports no freshness";
}

// MOVERS (the chase phase, stage 1): the candidate.  A cube that has stood for 60 casts and a second cube that
// moves 2 cm a cast: through a 25-tick window the mover's voxels are young against the oldest cluster's, the
// standing cube's are not, and mover_topic names the mover.  Without the topic the bus carries nothing.
TEST(CloudMap, AYoungClusterAgainstTheCloudsOwnAgeIsTheMover) {
    ParamMap p = things_params();
    p["mover_topic"] = std::string("out.mover");
    p["mover_window_ticks"] = int64_t{25};
    p["mover_age_k"] = 0.3;
    Rig r(p);
    for (int i = 0; i < 60; ++i) cast_world(r, 0.0, cube());               // the standing cube, 60 casts
    EXPECT_EQ(r.m.mover_index(), -1) << "one cluster: nothing to be young against";
    std::vector<Pt> mover;
    for (int k = 0; k < 8; ++k) {
        mover.clear();
        for (auto q : cube()) { q[0] += 0.4 + 0.02 * k; q[1] += 0.5; mover.push_back(q); }   // a second cube, left and ahead, moving
        cast_world(r, 0.0, cube());
        cast_world(r, 0.0, mover);
    }
    ASSERT_GE(r.m.mover_index(), 0) << "the moving cube is the candidate";
    const auto& cl = r.m.mover_clusters();
    EXPECT_NEAR(cl[size_t(r.m.mover_index())].cy, 0.5, 0.1) << "the mover is the cube to the left";
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.mover"));
    ASSERT_NE(tok, nullptr);
    ASSERT_EQ(tok->values.size(), 6);
    EXPECT_LT(tok->values[0], -0.2f) << "to the left: a negative +right component";
    EXPECT_GT(tok->values[1], 0.8f);
    EXPECT_GT(tok->values[2], 0.0f);
    EXPECT_LT(tok->values[3], tok->values[4] * 0.3f) << "young against the oldest";
    Rig plain(things_params());
    for (int i = 0; i < 5; ++i) cast_world(plain, 0.0, cube());
    EXPECT_EQ(plain.bus.last_value("out.mover"), nullptr) << "no topic, nothing published";
}

// walk_things false (O65): a walking cloud attends nothing and the bearing reads proximity 0, while a stop's cloud
// attends as before; the mover candidate still reads the walking cloud.
TEST(CloudMap, WalkThingsFalseAttendsNothingOnTheWalk) {
    ParamMap p = things_params();
    p["walk_cloud"] = true; p["walk_reset_m"] = 5.0; p["walk_things"] = false;
    p["mover_topic"] = std::string("out.mover"); p["mover_window_ticks"] = int64_t{25}; p["mover_age_k"] = 0.3;
    Rig r(p);
    for (int i = 0; i < 60; ++i) r.cast(false, 0.0, 0.0, 0.0, 3, cube());   // moving from the first tick: a walking cloud
    for (int k = 0; k < 8; ++k) {
        std::vector<Pt> mover;
        for (auto q : cube()) { q[0] += 0.4 + 0.02 * k; q[1] += 0.5; mover.push_back(q); }
        for (size_t at = 0; at < cube().size(); at += size_t(ogma::CloudMap::kZones)) r.cast(false, 0.0, 0.0, 0.0, 3, cube());
        r.cast(false, 0.0, 0.0, 0.0, 3, mover);
    }
    EXPECT_TRUE(r.m.is_walking_cloud());
    EXPECT_EQ(r.m.attended(), -1) << "things at stops only";
    auto b = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing_bearing"));
    ASSERT_NE(b, nullptr);
    EXPECT_FLOAT_EQ(b->values[2], 0.0f) << "proximity 0: nothing attended on the walk";
    EXPECT_GE(r.m.mover_index(), 0) << "the mover candidate still reads the walking cloud";
}

// The walking cloud's bearing is relative to the BODY, not the anchor (2026-09-27, §17.54): a cube 0.85 m ahead at
// the anchor reads 0.45 m ahead once the body has walked 0.4 m toward it, and ahead-right once the body has stepped
// left.  (Before the fix every static thing on a walk read as moving at the body's speed.)
TEST(CloudMap, AWalkingCloudsBearingIsFromTheBodyNotTheAnchor) {
    ParamMap p = things_params();
    p["walk_cloud"] = true; p["walk_reset_m"] = 5.0;
    Rig r(p);
    const auto world = cube();                                          // at x 0.81-0.89, y -0.03..0.05
    r.cast(false, 0.0, 0.0, 0.0, 3, seen_from(world, 0.0, 0.0, 0.0));  // a walking cloud opens at the origin
    auto b0 = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing_bearing"));
    ASSERT_NE(b0, nullptr);
    EXPECT_NEAR(b0->values[2], 1.0 - 0.85 / 2.5, 0.02);
    r.cast(false, 0.0, 0.4, 0.0, 3, seen_from(world, 0.4, 0.0, 0.0));  // the body 0.4 m along +x
    auto b1 = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing_bearing"));
    ASSERT_NE(b1, nullptr);
    EXPECT_NEAR(b1->values[2], 1.0 - 0.45 / 2.5, 0.03) << "0.45 m ahead of the body now, not 0.85";
    EXPECT_NEAR(b1->values[1], 1.0, 0.02) << "still straight ahead";
    r.cast(false, 0.0, 0.4, 0.3, 3, seen_from(world, 0.4, 0.3, 0.0));  // and 0.3 m to the LEFT of the line
    auto b2 = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing_bearing"));
    ASSERT_NE(b2, nullptr);
    EXPECT_GT(b2->values[0], 0.4f) << "the cube is now ahead-RIGHT";
    EXPECT_NEAR(std::hypot(b2->values[0], b2->values[1]), 1.0, 1e-4);
    EXPECT_NEAR(b2->values[2], 1.0 - std::hypot(0.45, 0.3) / 2.5, 0.03);
}

// VACATED voxels (T6's first half, 2026-09-27): a cube seen for ten casts from an origin 10 cm up, then the same
// rays reaching a wall 2 m out (the cube gone): the rays pass through the cube's old voxels, which are marked
// vacated; a cluster there counts them.  Without the origin, or with the window off, nothing is marked.
TEST(CloudMap, RaysThroughWhereAThingWasMarkItVacated) {
    ParamMap p = things_params();
    p["vacate_window_ticks"] = int64_t{50};
    Rig r(p);
    r.origin = {0.0, 0.0, 0.10};
    for (int i = 0; i < 10; ++i) cast_world(r, 0.0, cube());               // the cube at 0.81-0.89 m, heights 3-11 cm
    EXPECT_EQ(r.m.vacated_total(), 0u) << "rays that END on the cube pass through nothing of it";
    // the cube is gone: the rays that hit it now reach a wall at 2.0 m, at heights that carry them through its voxels
    std::vector<Pt> wall;
    for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k) wall.push_back({2.0, -0.03 + 0.04 * j, 0.10 + (0.03 + 0.04 * k - 0.10) * (2.0 / 0.85)});
    cast_world(r, 0.0, wall);
    EXPECT_GT(r.m.vacated_total(), 0u) << "the cube's voxels lie on the way to the wall";
    const auto cl = r.m.cluster_recent(60);
    ASSERT_GE(cl.size(), 1u);
    const auto cube_it = std::find_if(cl.begin(), cl.end(), [](const ogma::CloudMap::Thing& t) { return t.cx < 1.5; });
    ASSERT_NE(cube_it, cl.end());
    EXPECT_GE(cube_it->vacated, 1) << "the cube's cluster has a trail";
    // without the window the same casts mark nothing
    Rig q(things_params());
    q.origin = {0.0, 0.0, 0.10};
    for (int i = 0; i < 10; ++i) cast_world(q, 0.0, cube());
    cast_world(q, 0.0, wall);
    EXPECT_EQ(q.m.vacated_total(), 0u);
    EXPECT_EQ(q.m.cluster_recent(60)[0].vacated, 0);
}

// things_skip_movers (2026-09-28): a cube that has stood for 60 casts and a NEARER cube that moves 2 cm a cast:
// the plain reduction attends the nearer (moving) one; with the switch the moving cube is young against the
// standing one's age and the standing cube is attended.  A thing has a place while it is still.
TEST(CloudMap, ThingsSkipMoversAttendsTheStandingCubeNotTheNearerMovingOne) {
    for (bool skip : {false, true}) {
        ParamMap p = things_params();
        p["mover_topic"] = std::string("out.mover"); p["mover_window_ticks"] = int64_t{25}; p["mover_age_k"] = 0.3;
        p["things_skip_movers"] = skip;
        Rig r(p);
        for (int i = 0; i < 60; ++i) cast_world(r, 0.0, cube());               // the standing cube at 0.85 m
        for (int k = 0; k < 8; ++k) {
            std::vector<Pt> mover;
            for (auto q : cube()) { q[0] -= 0.30 - 0.005 * k; q[1] += 0.45; mover.push_back(q); }   // nearer, to the left, creeping (its smear stays SMALL)
            cast_world(r, 0.0, cube());
            cast_world(r, 0.0, mover);
        }
        ASSERT_GE(r.m.attended(), 0);
        const auto& a = r.m.things()[size_t(r.m.attended())];
        if (skip) { EXPECT_NEAR(a.cy, 0.0, 0.05) << "the standing cube"; }
        else      { EXPECT_NEAR(a.cy, 0.45, 0.10) << "the nearer, moving cube"; }
    }
}

// ISOLATION (2026-09-28, the operator): a cube at the foot of a post has tall voxels near it; a cube on open floor
// has none.  With mover_isolated the young cube by the post is not the candidate; with things_isolated it is not
// attended; and things_age_dim appends the age to the descriptor.
TEST(CloudMap, ACubeAtTheFootOfAPostIsPartOfIt) {
    ParamMap p = things_params();
    p["mover_topic"] = std::string("out.mover"); p["mover_window_ticks"] = int64_t{25}; p["mover_age_k"] = 0.3;
    p["things_isolated"] = true; p["mover_isolated"] = true; p["things_age_dim"] = true;
    Rig r(p);
    // a post at 0.85 m (8 cm wide, up to 35 cm) with a cube against its foot, and a lone cube 1.4 m ahead-right
    std::vector<Pt> world;
    for (int j = 0; j < 2; ++j) for (int k = 0; k < 9; ++k) world.push_back({0.85, 0.19 + 0.04 * j, 0.03 + 0.04 * k});
    for (auto q : cube()) world.push_back(q);                                        // the cube at 0.81-0.89, y -0.03..0.05: two empty columns from the post, 18 cm from its centroid
    for (auto q : cube()) { q[0] += 0.56; q[1] -= 0.6; world.push_back(q); }          // the lone cube (14 voxels along: off the boundaries)
    for (int i = 0; i < 40; ++i) cast_world(r, 0.0, world);
    const auto& th = r.m.things();
    ASSERT_GE(th.size(), 2u);
    int lone = -1, footed = -1;
    for (size_t i = 0; i < th.size(); ++i) { if (th[i].small && th[i].cy < -0.3) lone = int(i); if (th[i].small && std::fabs(th[i].cy) < 0.1) footed = int(i); }
    ASSERT_GE(lone, 0); ASSERT_GE(footed, 0);
    EXPECT_GT(th[size_t(footed)].tall_near, 0) << "the post stands within 0.25 m of the cube at its foot";
    EXPECT_EQ(th[size_t(lone)].tall_near, 0) << "nothing tall near the lone cube";
    std::string dump;
    for (size_t i = 0; i < th.size(); ++i)
        dump += "[" + std::to_string(i) + "] cx " + std::to_string(th[i].cx) + " cy " + std::to_string(th[i].cy) + " top " + std::to_string(th[i].top) +
                " ext " + std::to_string(th[i].ext) + " small " + std::to_string(th[i].small) + " tall " + std::to_string(th[i].tall_near) + "\n";
    EXPECT_EQ(r.m.attended(), lone) << "the nearer cube by the post is part of the post; the lone cube is the thing\n" << dump;
    auto tok = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.thing"));
    ASSERT_NE(tok, nullptr);
    EXPECT_EQ(tok->values.size(), ogma::CloudMap::kThing + 1) << "the age dim appended";
    EXPECT_NEAR(tok->values[ogma::CloudMap::kThing], 1.0, 0.05) << "as old as anything here: still";
}

// mover_range_hold (2026-09-29): a creeping cube followed from 1.0 m stays the candidate as it passes 1.2 m, out to the hold range.
TEST(CloudMap, AMoverBeingFollowedStaysACandidateBeyondTheStartRange) {
    ParamMap p = things_params();
    p["mover_topic"] = std::string("out.mover"); p["mover_window_ticks"] = int64_t{25}; p["mover_age_k"] = 0.3;
    p["mover_range"] = 1.2; p["mover_range_hold"] = 2.5;
    Rig r(p);
    for (int i = 0; i < 60; ++i) cast_world(r, 0.0, cube());               // the standing cube at 0.85 m (the oldest)
    int published_far = 0;
    for (int k = 0; k < 40; ++k) {
        std::vector<Pt> mover;
        for (auto q : cube()) { q[0] += 0.10 + 0.02 * k; q[1] += 0.5; mover.push_back(q); }   // from 0.95 m out to 1.75 m, 2 cm a cast
        cast_world(r, 0.0, cube());
        cast_world(r, 0.0, mover);
        if (r.m.mover_index() >= 0 && r.m.mover_clusters()[size_t(r.m.mover_index())].rng > 1.2) ++published_far;
    }
    EXPECT_GT(published_far, 10) << "followed out past 1.2 m";
    Rig q(things_params());
    ParamMap p2 = things_params(); p2["mover_topic"] = std::string("out.mover"); p2["mover_window_ticks"] = int64_t{25}; p2["mover_age_k"] = 0.3; p2["mover_range"] = 1.2;
    Rig r2(p2);
    for (int i = 0; i < 60; ++i) cast_world(r2, 0.0, cube());
    int far2 = 0;
    for (int k = 0; k < 40; ++k) {
        std::vector<Pt> mover;
        for (auto qq : cube()) { qq[0] += 0.10 + 0.02 * k; qq[1] += 0.5; mover.push_back(qq); }
        cast_world(r2, 0.0, cube()); cast_world(r2, 0.0, mover);
        if (r2.m.mover_index() >= 0 && r2.m.mover_clusters()[size_t(r2.m.mover_index())].rng > 1.2) ++far2;
    }
    EXPECT_EQ(far2, 0) << "without the hold, nothing past 1.2 m";
}

// THE TARGET'S SURROUNDINGS (2026-09-29): the seek loop's target placed in the cloud and the tall voxels around it counted.
TEST(CloudMap, TallStructureAroundTheHeldTargetIsCounted) {
    ParamMap p = things_params();
    p["target_topic"] = std::string("in.seek"); p["target_range_topic"] = std::string("in.seek_range");
    p["target_tall_topic"] = std::string("out.target_tall");
    Rig r(p);
    std::vector<Pt> world;
    for (int j = 0; j < 2; ++j) for (int k = 0; k < 9; ++k) world.push_back({0.85, 0.19 + 0.04 * j, 0.03 + 0.04 * k});   // a post ahead-left
    for (int i = 0; i < 5; ++i) cast_world(r, 0.0, world);
    auto aim = [&](double cx, double cy, double range) {
        r.bus.begin_tick(r.t);
        auto b = std::make_shared<ogma::ProprioToken>(); b->values = Eigen::VectorXf(3); b->values << float(cx), float(cy), 1.0f;
        r.bus.publish("in.seek", b);
        auto g = std::make_shared<ogma::ProprioToken>(); g->values = Eigen::VectorXf::Constant(1, float(range));
        r.bus.publish("in.seek_range", g);
        r.bus.end_tick();
        cast_world(r, 0.0, world);
        auto out = std::dynamic_pointer_cast<const ogma::ProprioToken>(r.bus.last_value("out.target_tall"));
        return out ? int(out->values[0]) : -1;
    };
    // a target at the post's foot (0.85 ahead, 0.21 left: bearing +left = negative cx): tall voxels around it
    const double n1 = std::hypot(0.85, 0.21);
    EXPECT_GT(aim(-0.21 / n1, 0.85 / n1, n1), 0) << "the post stands within a body length of the target";
    // a target 1.6 m ahead-right: nothing tall near it
    const double n2 = std::hypot(1.4, 0.8);
    EXPECT_EQ(aim(0.8 / n2, 1.4 / n2, n2), 0);
}

// TOP SEEN (2026-10-02, the ten-minutes phase S1): with the head pitched down the stack rule sees no top and calls a
// wall's foot small.  free_rays records the free space each ray passed through; small_needs_top asks that a ray went
// at least a voxel above a cluster's top in its own columns.  The cube's own rays end on it (no seen top): not small;
// an empty ray that passes level over it at the sensor's height (20 cm) sees its 11 cm top: small again.
TEST(CloudMap, ACubeIsSmallOnlyOnceARayHasPassedOverItsTop) {
    ParamMap p = things_params();
    p["free_rays"] = true;
    p["small_needs_top"] = true;
    Rig r(p);
    r.origin = {0.0, 0.0, 0.20};
    for (int i = 0; i < 4; ++i) cast_world(r, 0.0, scene());
    auto th = r.m.things();
    ASSERT_EQ(th.size(), 2u);
    EXPECT_LT(th[0].seen_above, th[0].top + 0.04) << "rays that end on the cube pass only just over its far columns";
    EXPECT_FALSE(th[0].small) << "its top was never seen";
    EXPECT_EQ(r.m.attended(), -1);
    // empty rays, level at the sensor's height, straight over the cube to 4 m
    // (zones 61-63: a zone either returns or is empty, and the scene's returns fill zones 0-44)
    const double nan = std::numeric_limits<double>::quiet_NaN();
    r.far_pts.assign(size_t(ogma::CloudMap::kZones), Pt{nan, nan, nan});
    for (int j = 0; j < 3; ++j) r.far_pts[size_t(61 + j)] = {4.0, 4.0 * (-0.03 + 0.04 * j) / 0.85, 0.20};
    for (int i = 0; i < 2; ++i) cast_world(r, 0.0, scene());
    th = r.m.things();
    ASSERT_EQ(th.size(), 2u);
    EXPECT_NEAR(th[0].seen_above, 0.20, 0.01);
    EXPECT_TRUE(th[0].small) << "a ray passed over it: its top is seen";
    EXPECT_FALSE(th[1].small) << "the post still climbs past small_top";
    EXPECT_EQ(r.m.attended(), 0);
    // passive: free_rays alone records seen_above and changes no verdict
    ParamMap q = things_params();
    q["free_rays"] = true;
    Rig s(q);
    s.origin = {0.0, 0.0, 0.20};
    for (int i = 0; i < 4; ++i) cast_world(s, 0.0, scene());
    EXPECT_TRUE(s.m.things()[0].small);
    EXPECT_GE(s.m.things()[0].seen_above, 0.0);
    // off: no traversal, seen_above unset
    Rig o(things_params());
    o.origin = {0.0, 0.0, 0.20};
    for (int i = 0; i < 4; ++i) cast_world(o, 0.0, scene());
    EXPECT_TRUE(o.m.things()[0].small);
    EXPECT_DOUBLE_EQ(o.m.things()[0].seen_above, -1.0);
}
