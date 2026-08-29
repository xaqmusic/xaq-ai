// =============================================================================
// test_diag_lite.cpp — the high-rate diagnostic payload contract.
//
//   Module::diag_lite() is what DiagPublisher serves when a subscription's topic
//   is "lite" (DiagPublisher.cpp).  It exists because diag_snapshot() for an EPM
//   is the whole GNG — measured at ~50 KB serialised ON THE TICK THREAD, per
//   frame, per subscription — and a sonifier wants the error signal at 60 Hz, not
//   the state.  So the contract is narrow and worth pinning:
//
//     * a payload is FLAT SCALARS (numbers and bools), optionally one level of
//       nesting for a per-source map whose keys the module cannot know in advance
//       (LateralVoter's trust weights, HomeostaticDrive's channels).  Never an
//       array, never a nested container — those are what grow with run time.
//     * it must NOT delegate to snapshot_state()/diag_snapshot() when those carry
//       the unbounded payload.  The regression that matters is someone "fixing" a
//       missing key by forwarding the full snapshot; the size assert below is the
//       cheap guard, the absence of the container keys is the real one.
//     * a module that opts out returns {} — and an EMPTY payload is a real signal
//       to the subscriber, not an error.  xaq_voice selects voices by type name,
//       so a type that matches its filter while publishing {} opens an oscillator
//       that can never make a sound.  Every such type is covered here.
//
//   The values are not asserted — these are default-constructed modules and the
//   numbers are zeros.  Shape, boundedness and coverage are the contract.
// =============================================================================

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ogma/Module.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/modules/GainEvolver.hpp"
#include "ogma/modules/GradientEPM.hpp"
#include "ogma/modules/HomeostaticDrive.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/modules/MotorEPM.hpp"
#include "ogma/modules/MotorEPMv2.hpp"
#include "ogma/modules/NeurochemState.hpp"
#include "ogma/modules/SequenceGNG.hpp"

namespace {

// A "lite" payload may be scalars, or one level of {name -> scalar} for a map the
// module cannot enumerate at compile time.  Anything deeper is a container that
// will grow with run time, which is the whole thing this payload exists to avoid.
void expect_lite_shape(const nlohmann::json& j, const std::string& who) {
    ASSERT_TRUE(j.is_object()) << who << ": diag_lite must be a JSON object";
    for (auto const& [key, v] : j.items()) {
        if (v.is_object()) {
            for (auto const& [k2, v2] : v.items())
                EXPECT_TRUE(v2.is_number() || v2.is_boolean())
                    << who << ": " << key << "." << k2 << " must be a number or bool";
        } else {
            EXPECT_TRUE(v.is_number() || v.is_boolean())
                << who << ": " << key << " must be a number or bool (no arrays, no strings)";
        }
    }
    // Generous: the point is to catch a full snapshot forwarded by mistake, which is
    // three orders of magnitude over this, not to police a handful of extra keys.
    EXPECT_LT(j.dump().size(), 4096u) << who << ": diag_lite is not small any more";
}

template <typename M>
nlohmann::json lite_of() { M m; return m.diag_lite(); }

// Every type that must publish something.  xaq_voice's type filter matches any type
// name containing "EPM" plus SequenceGNG, so all of those are here whether or not
// they are interesting sources — a silent voice with no diagnostic is the bug.
TEST(DiagLite, CoveredModulesPublishSomething) {
    EXPECT_FALSE(lite_of<ogma::EPM>().empty())              << "EPM";
    EXPECT_FALSE(lite_of<ogma::MotorEPM>().empty())         << "MotorEPM (v1)";
    EXPECT_FALSE(lite_of<ogma::MotorEPMv2>().empty())       << "MotorEPMv2";
    EXPECT_FALSE(lite_of<ogma::GradientEPM>().empty())      << "GradientEPM";
    EXPECT_FALSE(lite_of<ogma::SequenceGNG>().empty())      << "SequenceGNG";
    EXPECT_FALSE(lite_of<ogma::NeurochemState>().empty())   << "NeurochemState";
    EXPECT_FALSE(lite_of<ogma::LateralVoter>().empty())     << "LateralVoter";
    EXPECT_FALSE(lite_of<ogma::HomeostaticDrive>().empty()) << "HomeostaticDrive";
    EXPECT_FALSE(lite_of<ogma::GainEvolver>().empty())      << "GainEvolver";
}

TEST(DiagLite, PayloadsAreFlatScalarsAndSmall) {
    expect_lite_shape(lite_of<ogma::EPM>(),              "EPM");
    expect_lite_shape(lite_of<ogma::MotorEPM>(),         "MotorEPM");
    expect_lite_shape(lite_of<ogma::MotorEPMv2>(),       "MotorEPMv2");
    expect_lite_shape(lite_of<ogma::GradientEPM>(),      "GradientEPM");
    expect_lite_shape(lite_of<ogma::SequenceGNG>(),      "SequenceGNG");
    expect_lite_shape(lite_of<ogma::NeurochemState>(),   "NeurochemState");
    expect_lite_shape(lite_of<ogma::LateralVoter>(),     "LateralVoter");
    expect_lite_shape(lite_of<ogma::HomeostaticDrive>(), "HomeostaticDrive");
    expect_lite_shape(lite_of<ogma::GainEvolver>(),      "GainEvolver");
}

// The regression this file exists for: the unbounded containers must never appear.
// Naming them individually beats a size bound, because a module early in its life
// is small no matter what it forwards.
TEST(DiagLite, NeverCarriesTheUnboundedContainers) {
    const auto epm = lite_of<ogma::EPM>();
    EXPECT_FALSE(epm.contains("gng"))               << "EPM lite must not ship the GNG";
    EXPECT_FALSE(epm.contains("transition_counts")) << "EPM lite must not ship the transition map";
    EXPECT_FALSE(epm.contains("prev_winner_prototype"));

    const auto m2 = lite_of<ogma::MotorEPMv2>();
    EXPECT_FALSE(m2.contains("A"))    << "MotorEPMv2 lite must not ship the self-model matrix";
    EXPECT_FALSE(m2.contains("legs")) << "MotorEPMv2 lite must not ship per-leg arrays";
    EXPECT_FALSE(m2.contains("gng"));

    const auto sg = lite_of<ogma::SequenceGNG>();
    EXPECT_FALSE(sg.contains("gng"))              << "SequenceGNG lite must not ship the GNG";
    EXPECT_FALSE(sg.contains("successor_counts")) << "SequenceGNG lite must not ship the successor map";

    const auto lv = lite_of<ogma::LateralVoter>();
    EXPECT_FALSE(lv.contains("embedding_cache")) << "LateralVoter lite must not ship the embedding cache";
    EXPECT_FALSE(lv.contains("assoc"));

    const auto ge = lite_of<ogma::GainEvolver>();
    EXPECT_FALSE(ge.contains("accept_log")) << "GainEvolver lite must not ship the accept log";
}

// The two maps a module cannot enumerate at compile time are published as nested
// objects, so a subscriber walking dotted paths gets trust.<modality> without this
// side having to fix a source order.  Empty is correct before the first tick; the
// contract is that the KEY exists and is an object.
TEST(DiagLite, PerSourceMapsAreNestedObjects) {
    const auto lv = lite_of<ogma::LateralVoter>();
    ASSERT_TRUE(lv.contains("trust"));
    EXPECT_TRUE(lv["trust"].is_object());
    ASSERT_TRUE(lv.contains("surprise"));
    EXPECT_TRUE(lv["surprise"].is_object());
    EXPECT_FALSE(lv["has_token"].get<bool>()) << "no token before the first tick";

    const auto hd = lite_of<ogma::HomeostaticDrive>();
    ASSERT_TRUE(hd.contains("channels"));
    EXPECT_TRUE(hd["channels"].is_object());
}

// An EPM with no GNG yet publishes only the four unconditional error scalars.  This
// pins the conditional branch: the four GNG life-signals appear only once there is a
// GNG to ask, and a subscriber must tolerate their absence rather than assume them.
TEST(DiagLite, EpmWithoutGngPublishesOnlyTheErrorScalars) {
    const auto j = lite_of<ogma::EPM>();
    EXPECT_TRUE(j.contains("last_tle"));
    EXPECT_TRUE(j.contains("ema_tle"));
    EXPECT_TRUE(j.contains("last_quant_error"));
    EXPECT_TRUE(j.contains("novelty_threshold_now"));
    EXPECT_FALSE(j.contains("nodes"))     << "no GNG constructed, so no node count";
    EXPECT_FALSE(j.contains("baked_now"));
}

// Opting out stays possible and stays cheap: the base returns {} rather than falling
// back to snapshot_state() the way diag_snapshot() does.  A subscriber reads {} as
// "this module has nothing to say at rate", which is a supported answer.
TEST(DiagLite, BaseClassOptsOutWithAnEmptyObject) {
    struct Bare : ogma::Module {
        std::string_view              type_name()     const override { return "Bare"; }
        std::vector<ogma::TopicSpec>  input_topics()  const override { return {}; }
        std::vector<ogma::TopicSpec>  output_topics() const override { return {}; }
        ogma::ParamSchema             params_schema() const override { return {}; }
        void on_setup(ogma::Bus*, ogma::ParamMap const&) override {}
        void tick(uint64_t) override {}
    } bare;
    EXPECT_TRUE(bare.diag_lite().is_object());
    EXPECT_TRUE(bare.diag_lite().empty());
}

}  // namespace
