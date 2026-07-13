// =============================================================================
// test_clone_shipping_configs.cpp  --  W3.1 determinism contract on real configs
// =============================================================================
//
// The base `test_clone.cpp::ClonedBrainIsByteEquivalentForN_Ticks` proves the
// contract on a hand-built minimal MC-like graph.  This file extends coverage
// to the actual JSON configs shipped under
// `godot_host/project/addons/ami_ogma/configs/`, so that adding a new module
// type (or breaking snapshot_state on an existing one) is caught against the
// same surface that frozen-eval benchmarks and the future Save/Load UI will
// use.
//
// Strategy
// --------
//
// Each test loads one shipping config via `GraphConfig::load_from_file`,
// boots an `OgmaInstance`, drives it for 50 warmup ticks with deterministic
// synthetic inputs (proprioceptive frames + alive-pulse events bridged at the
// host:* boundary), clones, then runs source AND clone for 100 more ticks
// and asserts byte-equivalence on `action.out` and `neuro.state.dopamine`.
//
// Configs that contain a module currently missing `snapshot_state()` are
// gated with GTEST_SKIP and a message naming the blocking module type.  As
// each Tier A / Tier B implementation lands (see
// `docs/v4_ui_dev_module_state_audit.md`), the SKIP for affected configs
// flips to a real assertion.
//
// Synthetic input plumbing
// ------------------------
//
// We walk every module's `input_topics()` and, for every topic in
// `reality.proprio.*` or `events.*`, publish a deterministic payload per
// tick.  Proprio dimensionality is read from each EPM's `proprio_state_dims`
// param when the input topic matches; otherwise we default to 4-D (the
// CartPole shape).  This is a config-aware harness, not a synthesizer of
// arbitrary host inputs (e.g. RawImageFrame for visual EPMs), so configs
// with vision modalities are skipped explicitly.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"
#include "ogma/Module.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

namespace {

namespace fs = std::filesystem;

// Resolve config dir relative to the repo root.  CMake test invocation cwd
// is typically `cpp_core/build*`, so we walk up to find the configs folder.
fs::path locate_configs_dir() {
    auto candidates = {
        fs::path("../godot_host/project/addons/ami_ogma/configs"),
        fs::path("../../godot_host/project/addons/ami_ogma/configs"),
        fs::path("../../../godot_host/project/addons/ami_ogma/configs"),
        fs::path("godot_host/project/addons/ami_ogma/configs"),
    };
    for (auto const& c : candidates) {
        if (fs::exists(c) && fs::is_directory(c)) return fs::canonical(c);
    }
    return {};
}

// Modules that fall through to `Module::snapshot_state` no-op.  Empty as of
// ui-dev Tier B landing.  Source of truth: docs/v4_ui_dev_module_state_audit.md.
std::set<std::string> const kModulesMissingSnapshot = {};

// Visual encoders that consume RawImageFrame — the harness doesn't synthesize
// pixel input today.  Configs containing any such EPM are skipped.
std::set<std::string> const kVisualEncoderKinds = {
    "jl",          // FrozenJLEncoder consumes RawImageFrame
    "stft",        // FrozenSTFTEncoder consumes RawAudioFrame
};

struct SkipReason {
    bool        skip = false;
    std::string why;
};

// Inspect a parsed config and report any blockers preventing the harness
// from running it (missing snapshot modules, visual encoders, etc.).
SkipReason classify_config(ogma::GraphConfig const& g) {
    std::set<std::string> blockers;
    bool has_visual = false;
    for (auto const& m : g.modules) {
        if (kModulesMissingSnapshot.count(m.type)) {
            blockers.insert(m.type);
        }
        if (m.type == "EPM") {
            auto it = m.params.find("encoder_kind");
            if (it != m.params.end()) {
                if (auto* s = std::get_if<std::string>(&it->second);
                    s && kVisualEncoderKinds.count(*s)) {
                    has_visual = true;
                }
            }
        }
    }
    if (has_visual) {
        return {true, "config contains JL/STFT encoder; harness "
                      "doesn't synthesize RawImageFrame/RawAudioFrame yet"};
    }
    if (!blockers.empty()) {
        std::string b;
        for (auto const& s : blockers) { if (!b.empty()) b += ","; b += s; }
        return {true, "config contains module(s) without snapshot_state: " + b};
    }
    return {};
}

// Collect host-bridged input topics the harness must drive.  We look at
// each module's declared input_topics() and pick out exact matches in
// `reality.proprio.*` and `events.*`.  Prefix subscriptions (trailing dot)
// are skipped — they catch whatever modules publish on their own.
struct HostInputs {
    // proprio topic -> dim (read from a matching EPM's `proprio_state_dims`,
    // or 4 if the topic is ActionDecoder's `proprio_topic`-style bridge).
    std::unordered_map<std::string, int> proprio_dims;
    std::unordered_set<std::string>      event_topics;
};

HostInputs collect_host_inputs(ogma::OgmaInstance& inst,
                               ogma::GraphConfig const& g) {
    HostInputs h;
    // Pull dims from EPM params first.
    for (auto const& m : g.modules) {
        if (m.type != "EPM") continue;
        auto topic_it = m.params.find("input_topic");
        auto dim_it   = m.params.find("proprio_state_dims");
        if (topic_it == m.params.end()) continue;
        auto const* topic = std::get_if<std::string>(&topic_it->second);
        if (!topic || topic->rfind("reality.proprio.", 0) != 0) continue;
        int dim = 4;
        if (dim_it != m.params.end()) {
            if (auto* i = std::get_if<int64_t>(&dim_it->second)) dim = int(*i);
        }
        h.proprio_dims[*topic] = dim;
    }
    // Walk every module's declared inputs.
    for (auto* mod : inst.modules()) {
        for (auto const& t : mod->input_topics()) {
            if (t.kind != ogma::SubscriptionKind::Direct &&
                t.kind != ogma::SubscriptionKind::Feedback) continue;
            // Skip prefix subs (trailing dot).
            if (!t.name.empty() && t.name.back() == '.') continue;
            if (t.name.rfind("reality.proprio.", 0) == 0) {
                h.proprio_dims.try_emplace(t.name, 4);
            } else if (t.name.rfind("events.", 0) == 0) {
                h.event_topics.insert(t.name);
            }
        }
    }
    return h;
}

void publish_synthetic(ogma::Bus* bus, HostInputs const& h, uint64_t tick) {
    for (auto const& [topic, dim] : h.proprio_dims) {
        auto p = std::make_shared<ogma::ProprioToken>();
        p->tick_id = tick;
        // Sensor name is the trailing component after "reality.proprio.".
        p->sensor  = topic.substr(std::string("reality.proprio.").size());
        p->values.resize(dim);
        // Deterministic, bounded, non-stationary: per-dim phase shift.
        for (int i = 0; i < dim; ++i) {
            double phase = double(tick) * (0.07 + 0.013 * double(i));
            p->values(i) = float(std::sin(phase));
        }
        bus->publish(topic, p);
    }
    // Fire each event topic on a fixed 30-tick cadence so HomeostaticDrive's
    // alive-pulse channel stays replenished without saturating.
    if (tick % 30 == 0) {
        for (auto const& ev_topic : h.event_topics) {
            auto ev = std::make_shared<ogma::EnvEvent>();
            ev->tick_id   = tick;
            // Trailing component after "events." is the event name.
            ev->name      = ev_topic.substr(std::string("events.").size());
            ev->intensity = 1.0f;
            bus->publish(ev_topic, ev);
        }
    }
}

double action_digest(ogma::Bus const* bus) {
    auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus->last_value(ogma::topics::kActionOut));
    return a ? double(a->accel) : 0.0;
}

double dopamine_digest(ogma::Bus const* bus) {
    auto n = std::dynamic_pointer_cast<const ogma::NeuroState>(
        bus->last_value(ogma::topics::kNeuroState));
    return n ? double(n->dopamine) : 0.0;
}

void run_byte_equiv_check(std::string const& config_path,
                          int warmup_ticks = 50,
                          int compare_ticks = 100) {
    auto cfg = ogma::GraphConfig::load_from_file(config_path);
    auto skip = classify_config(cfg);
    if (skip.skip) {
        GTEST_SKIP() << config_path << " — " << skip.why;
    }
    // Pin to 1 thread.  The byte-equivalence contract is about
    // snapshot/restore correctness, not about whether the parallel
    // scheduler has bit-stable reduction order — a separate concern.
    cfg.runtime.num_threads = 1;

    auto src = std::make_unique<ogma::OgmaInstance>(
        cfg, std::make_unique<ogma::InProcessBus>());
    auto inputs = collect_host_inputs(*src, cfg);

    // Warmup.
    for (uint64_t t = 0; t < uint64_t(warmup_ticks); ++t) {
        publish_synthetic(src->bus(), inputs, t);
        src->tick();
    }

    auto clone = src->clone();

    // Compare phase.
    for (uint64_t t = uint64_t(warmup_ticks);
         t < uint64_t(warmup_ticks + compare_ticks); ++t) {
        publish_synthetic(src->bus(),   inputs, t);
        publish_synthetic(clone->bus(), inputs, t);
        src->tick();
        clone->tick();

        ASSERT_DOUBLE_EQ(action_digest(src->bus()),
                         action_digest(clone->bus()))
            << config_path << " — action divergence at tick " << t;
        ASSERT_DOUBLE_EQ(dopamine_digest(src->bus()),
                         dopamine_digest(clone->bus()))
            << config_path << " — dopamine divergence at tick " << t;
    }
}

}  // namespace

class ShippingConfigDeterminism
    : public ::testing::TestWithParam<std::string> {
};

TEST_P(ShippingConfigDeterminism, ClonedBrainIsByteEquivalent) {
    auto dir = locate_configs_dir();
    if (dir.empty()) {
        GTEST_SKIP() << "configs/ directory not findable from cwd "
                     << fs::current_path();
    }
    auto path = (dir / GetParam()).string();
    if (!fs::exists(path)) {
        GTEST_SKIP() << "config not present: " << path;
    }
    run_byte_equiv_check(path);
}

// One row per shipping config.  As snapshot implementations land, the
// associated rows flip from SKIP (with a "missing snapshot_state" message)
// to PASS automatically — no test edit required.
INSTANTIATE_TEST_SUITE_P(
    AllShippingConfigs,
    ShippingConfigDeterminism,
    ::testing::Values(
        // 2026-05-13 (v6.0 launcher prune): list pared to the three
        // demo-highlight Cell configs the launcher still surfaces, plus
        // the CartPole/MountainCar headline pair.  All other shipping
        // configs moved to configs/archive/ — still loadable by full
        // path but not in the regression scorecard.  Restore individual
        // entries here if a regression A/B against an archived config
        // becomes load-bearing.
        "the_cartpole_minimal.json",
        "the_cartpole_premotor_aac.json",
        "the_mountain_car.json",
        "the_mountain_car_premotor_efe.json",
        "the_cell.json",
        "the_cell_crossfade.json",
        "the_cell_chunks_lifecycle_v9.json",
        "the_quadruped_minimal.json"
    ),
    [](::testing::TestParamInfo<std::string> const& info) {
        std::string s = info.param;
        // Sanitize for gtest test name: strip extension, replace dots.
        if (auto dot = s.rfind('.'); dot != std::string::npos) s.resize(dot);
        for (auto& c : s) if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
        return s;
    });
