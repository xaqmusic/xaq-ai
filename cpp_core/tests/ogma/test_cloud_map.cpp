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

    void cast(bool still, double yaw, double ox, double oy, int winner, std::vector<Pt> const& pts) {
        bus.begin_tick(t);
        auto tok = std::make_shared<ogma::ProprioToken>();
        tok->values = Eigen::VectorXf::Constant(kCast, std::numeric_limits<float>::quiet_NaN());
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
    ASSERT_EQ(b->values.size(), 3);
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
}

TEST(CloudMap, WithoutWalkCloudAMovingTickContributesNothing) {
    Rig r(params());
    r.cast(false, 0.0, 0.0, 0.0, 3, patch());
    EXPECT_FALSE(r.m.is_open());
    EXPECT_EQ(r.m.voxels(), 0);
}
