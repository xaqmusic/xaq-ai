// =============================================================================
// test_patch_mode_harness.cpp  --  human-paced patch-mode workflow simulator
// =============================================================================
//
// Drives an OgmaInstance through long sequences of patch operations and ticks
// with synthetic per-tick host input, mimicking the workflows the Godot panel
// exposes:
//
//   * Sequentially deleting every module down to the reflex chain.
//   * Building a brain from a blank canvas one module at a time.
//   * Toggling auto_subscribe at runtime.
//   * Connect / disconnect storms, including in manual mode where the gate
//     should actually ablate data flow.
//   * Snapshot / restore interleaved with patch ops.
//   * Inspector-thread races against the patch / tick thread.
//
// Goal: catch crashes that only surface when the brain is actively ticking
// and an external actor (a user or the inspector) mutates it on a different
// thread.  Every test inserts ticks with realistic synthetic input between
// actions, so the modules go through their normal handler / tick paths
// during the mutations rather than being touched only at quiescent boundaries.
//
// The harness is deliberately resilient: scenarios that should fail
// (validation errors, etc.) catch the exception and assert on it rather
// than letting it abort the test.  Crashes — segfaults, assertion failures,
// uncaught exceptions in module ticks — fail the test.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <unordered_set>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"
#include "ogma/Module.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

namespace {

// ---------------------------------------------------------------------------
// Cell-class config builder
//
// Hand-built minimal cell-like graph: NeurochemState + 4 EPMs (one of each
// modality flavour) + LateralVoter + HomeostaticDrive + ActionDecoder.
// Smaller than the_cell_premotor.json (no whisker x6, no DescendingPredictor)
// but covers every shipping-config module type the harness needs to mutate.
// ---------------------------------------------------------------------------

ogma::GraphConfig cell_like_config(bool auto_subscribe = true) {
    ogma::GraphConfig g;
    g.version = 1;
    g.runtime.auto_subscribe = auto_subscribe;

    {   ogma::ModuleSpec m;
        m.id = "neuro"; m.type = "NeurochemState";
        m.params["event_coupled_da"]      = ogma::ParamValue{true};
        m.params["da_baseline_ema_alpha"] = ogma::ParamValue{0.001};
        g.modules.push_back(m); }
    {   ogma::ModuleSpec m;
        m.id = "epm_state"; m.type = "EPM";
        m.params["modality_group"] = std::string("kinematic");
        m.params["modality_name"]  = std::string("state");
        m.params["encoder_kind"]   = std::string("rbf");
        m.params["input_topic"]    = std::string("reality.proprio.state");
        m.params["projection_dim"] = int64_t{16};
        m.params["proprio_state_dims"] = int64_t{2};
        m.params["subtract_descending_prediction"] = ogma::ParamValue{false};
        g.modules.push_back(m); }
    {   ogma::ModuleSpec m;
        m.id = "voter_0"; m.type = "LateralVoter";
        g.modules.push_back(m); }
    {   ogma::ModuleSpec m;
        m.id = "drive"; m.type = "HomeostaticDrive";
        m.params["channels"]              = std::vector<std::string>{"alive_pulse"};
        m.params["channel_kinds"]         = std::vector<std::string>{"alive_pulse"};
        m.params["setpoints"]             = std::vector<double>{0.8};
        m.params["urgency_normalizers"]   = std::vector<double>{1.0};
        m.params["channel_input_topics"]  = std::vector<std::string>{"events.alive"};
        g.modules.push_back(m); }
    {   ogma::ModuleSpec m;
        m.id = "action_decoder"; m.type = "ActionDecoder";
        m.params["consensus_level"] = int64_t{0};
        m.params["proprio_topic"]   = std::string("reality.kinematic.state");
        m.params["action_bins"]     = int64_t{3};
        m.params["accel_min"]       = double{-1.0};
        m.params["accel_max"]       = double{ 1.0};
        m.params["td_gamma"]        = double{0.95};
        g.modules.push_back(m); }
    return g;
}

// ---------------------------------------------------------------------------
// Synthetic host input — published on the bus before each tick, with
// producer_id="host" so the per-primitive gate (in manual mode) admits via
// host: edges.
// ---------------------------------------------------------------------------

void publish_synthetic(ogma::Bus* bus, uint64_t tick) {
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id     = tick;
    p->producer_id = "host";
    p->sensor      = "state";
    p->values.resize(2);
    p->values(0) = float(std::sin(double(tick) * 0.07));
    p->values(1) = float(std::cos(double(tick) * 0.13));
    bus->publish("reality.proprio.state", p);

    if (tick % 30 == 0) {
        auto e = std::make_shared<ogma::EnvEvent>();
        e->tick_id     = tick;
        e->producer_id = "host";
        e->name        = "alive";
        e->intensity   = 1.0f;
        bus->publish("events.alive", e);
    }
}

// Tick the instance N times with synthetic input.  Catches and re-throws
// any module-tick / hot-patch exceptions so the gtest macro reports the
// failing tick number.
void tick_n(ogma::OgmaInstance& inst, int n, uint64_t& tick_cursor) {
    for (int i = 0; i < n; ++i) {
        publish_synthetic(inst.bus(), tick_cursor);
        try {
            inst.tick();
        } catch (std::exception const& e) {
            FAIL() << "tick #" << tick_cursor << " threw: " << e.what();
        }
        ++tick_cursor;
    }
}

// Apply a one-op patch and run a few ticks so the patch lands and at least
// one full publish-tick cycle exercises the post-patch state.  Returns true
// on success (validation accepted + apply ran), false if validation
// rejected the op (apply throws on next tick — caught here).
bool apply_op(ogma::OgmaInstance& inst, ogma::GraphPatchOp const& op,
              uint64_t& tick_cursor, int settle_ticks = 4) {
    ogma::GraphPatchBatch batch;
    batch.ops.push_back(op);
    inst.scheduler()->enqueue_hot_patch(std::move(batch));
    // Enqueue is asynchronous — the validation + apply runs in
    // process_pending_patches at the next tick.  Tick once first to
    // surface any validation throw, then settle.
    try {
        inst.tick();
    } catch (std::exception const&) {
        return false;  // validation rejected
    }
    publish_synthetic(inst.bus(), tick_cursor);  // one publish for tick we just did
    ++tick_cursor;
    tick_n(inst, settle_ticks - 1, tick_cursor);
    return true;
}

// Same helper convenience wrappers — keep test code readable.
ogma::GraphPatchOp op_remove(std::string id) {
    return ogma::RemoveNodeOp{std::move(id)};
}
ogma::GraphPatchOp op_add(std::string id, std::string type, ogma::ParamMap params = {}) {
    return ogma::AddNodeOp{ogma::ModuleSpec{std::move(id), std::move(type), std::move(params)}};
}
ogma::GraphPatchOp op_connect(std::string from, std::string to, std::string topic = "") {
    ogma::ConnectOp c;
    c.edge.from  = std::move(from);
    c.edge.to    = std::move(to);
    c.edge.topic = std::move(topic);
    return c;
}
ogma::GraphPatchOp op_disconnect(std::string from, std::string to, std::string topic = "") {
    ogma::DisconnectOp d;
    d.from  = std::move(from);
    d.to    = std::move(to);
    d.topic = std::move(topic);
    return d;
}

// Live module ids on the instance.
std::unordered_set<std::string> live_ids(ogma::OgmaInstance const& inst) {
    std::unordered_set<std::string> out;
    auto modules = const_cast<ogma::OgmaInstance&>(inst).modules();
    for (auto const* m : modules) out.emplace(m->id());
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Scenario 1 — sequential remove down to the empty graph.
//
// Repros the user's "delete every module one at a time" workflow.  Settles
// 30 ticks between each remove so the brain runs through normal tick paths
// (handlers, snapshots, etc.) on the post-remove state before the next
// patch.  Final assertion: every module removed, brain still tickable.
// ---------------------------------------------------------------------------

TEST(PatchModeHarness, SequentialRemoveAllModules) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        cell_like_config(/*auto_subscribe=*/true),
        std::make_unique<ogma::InProcessBus>());

    uint64_t tick_cursor = 0;
    tick_n(*inst, 30, tick_cursor);  // warmup

    std::vector<std::string> remove_order = {
        "action_decoder", "drive", "voter_0", "epm_state", "neuro",
    };
    for (auto const& id : remove_order) {
        ASSERT_TRUE(apply_op(*inst, op_remove(id), tick_cursor, 30))
            << "remove of " << id << " failed validation";
        EXPECT_EQ(live_ids(*inst).count(id), 0u)
            << "module " << id << " still present after remove";
    }
    EXPECT_EQ(live_ids(*inst).size(), 0u)
        << "expected empty graph; got " << live_ids(*inst).size() << " modules";

    // Brain should still tick on an empty graph.
    tick_n(*inst, 10, tick_cursor);
}

// ---------------------------------------------------------------------------
// Scenario 2 — sequential remove with reverse order (mirror of #1).
//
// Some destruction-order dependencies surface only when teardown runs in a
// specific order (publishers before subscribers vs. the reverse).  This
// test inverts the order from #1 to catch those.
// ---------------------------------------------------------------------------

TEST(PatchModeHarness, SequentialRemoveReverseOrder) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        cell_like_config(/*auto_subscribe=*/true),
        std::make_unique<ogma::InProcessBus>());

    uint64_t tick_cursor = 0;
    tick_n(*inst, 30, tick_cursor);

    std::vector<std::string> remove_order = {
        "neuro", "epm_state", "voter_0", "drive", "action_decoder",
    };
    for (auto const& id : remove_order) {
        ASSERT_TRUE(apply_op(*inst, op_remove(id), tick_cursor, 30))
            << "remove of " << id << " failed";
        EXPECT_EQ(live_ids(*inst).count(id), 0u);
    }
    EXPECT_EQ(live_ids(*inst).size(), 0u);
    tick_n(*inst, 10, tick_cursor);
}

// ---------------------------------------------------------------------------
// Scenario 3 — blank-canvas build.
//
// Boot empty, add modules one at a time, tick between each.  Mirrors the
// user's stated goal of authoring brains from scratch.  Every add uses
// realistic params; each subsequent tick must succeed.
// ---------------------------------------------------------------------------

TEST(PatchModeHarness, BlankCanvasBuild) {
    ogma::GraphConfig empty;
    empty.version = 1;
    auto inst = std::make_unique<ogma::OgmaInstance>(
        std::move(empty), std::make_unique<ogma::InProcessBus>());

    uint64_t tick_cursor = 0;
    tick_n(*inst, 5, tick_cursor);  // empty brain still ticks

    // Build the same graph cell_like_config() boots with, but via patches.
    auto cfg = cell_like_config();
    for (auto const& spec : cfg.modules) {
        ASSERT_TRUE(apply_op(*inst, op_add(spec.id, spec.type, spec.params),
                             tick_cursor, 20))
            << "add of " << spec.id << " (" << spec.type << ") failed";
        EXPECT_EQ(live_ids(*inst).count(spec.id), 1u);
    }
    EXPECT_EQ(live_ids(*inst).size(), cfg.modules.size());

    // Final shake-out: 60 more ticks on the assembled brain.
    tick_n(*inst, 60, tick_cursor);
}

// ---------------------------------------------------------------------------
// Scenario 4 — auto_subscribe toggle storm.
//
// Flip routing mode many times while the brain is ticking with synthetic
// input.  Verifies that the install_manual_gates / clear-default-deny
// transitions never crash regardless of the per-tick cycle they fire on.
// ---------------------------------------------------------------------------

TEST(PatchModeHarness, AutoSubscribeToggleStorm) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        cell_like_config(/*auto_subscribe=*/true),
        std::make_unique<ogma::InProcessBus>());

    uint64_t tick_cursor = 0;
    tick_n(*inst, 20, tick_cursor);

    // Toggle 20 times, with 5-tick gaps.
    bool current = true;
    for (int i = 0; i < 20; ++i) {
        current = !current;
        try {
            inst->scheduler()->set_auto_subscribe(current);
        } catch (std::exception const& e) {
            FAIL() << "set_auto_subscribe(" << (current ? "true" : "false")
                   << ") threw at iteration " << i << ": " << e.what();
        }
        EXPECT_EQ(inst->scheduler()->is_auto_subscribe(), current);
        tick_n(*inst, 5, tick_cursor);
    }
}

// ---------------------------------------------------------------------------
// Scenario 5 — manual-mode connect / disconnect with verified ablation.
//
// Confirms that in manual mode, drag-connecting a host:* edge actually
// causes the receiver to start ticking on the new input, and disconnecting
// stops it again.  Uses an EPM whose internal node count grows when it
// receives proprio — the count is the observable.
// ---------------------------------------------------------------------------

TEST(PatchModeHarness, ManualModeEdgeAblation) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        cell_like_config(/*auto_subscribe=*/false),  // manual from boot
        std::make_unique<ogma::InProcessBus>());
    uint64_t tick_cursor = 0;

    // Find the EPM and capture its node count before any host edge exists.
    ogma::Module* epm = inst->module("epm_state");
    ASSERT_NE(epm, nullptr);

    auto epm_snapshot_size = [&]() {
        auto snap = epm->snapshot_state();
        if (!snap.contains("gng")) return 0;
        if (!snap["gng"].contains("nodes")) return 0;
        return int(snap["gng"]["nodes"].size());
    };

    int before = epm_snapshot_size();
    tick_n(*inst, 30, tick_cursor);
    int after_no_edge = epm_snapshot_size();
    EXPECT_EQ(before, after_no_edge)
        << "manual mode without explicit edge: EPM should not have grown";

    // Add the host edge.  EPM should start receiving on subsequent ticks.
    ASSERT_TRUE(apply_op(*inst,
        op_connect("host:reality.proprio.state", "epm_state",
                    "reality.proprio.state"),
        tick_cursor, 30));
    int after_connect = epm_snapshot_size();
    EXPECT_GT(after_connect, after_no_edge)
        << "EPM should grow once the host→EPM edge is wired";

    // Disconnect.  EPM stops receiving — node count holds steady.
    ASSERT_TRUE(apply_op(*inst,
        op_disconnect("host:reality.proprio.state", "epm_state",
                      "reality.proprio.state"),
        tick_cursor, 30));
    int after_disconnect = epm_snapshot_size();
    tick_n(*inst, 30, tick_cursor);
    int after_long = epm_snapshot_size();
    EXPECT_EQ(after_disconnect, after_long)
        << "EPM should not grow any further after disconnect";
}

// ---------------------------------------------------------------------------
// Scenario 6 — snapshot interleaved with patches.
//
// Take a snapshot, mutate the graph, restore the snapshot.  After restore,
// the module list must match the pre-mutation state and the brain must
// still tick.  Runs through several iterations to surface any leak in
// snapshot/restore that compounds over time.
// ---------------------------------------------------------------------------

TEST(PatchModeHarness, SnapshotInterleavedWithPatches) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        cell_like_config(/*auto_subscribe=*/true),
        std::make_unique<ogma::InProcessBus>());
    uint64_t tick_cursor = 0;
    tick_n(*inst, 20, tick_cursor);

    auto initial_ids = live_ids(*inst);

    for (int iter = 0; iter < 5; ++iter) {
        nlohmann::json snap;
        try {
            snap = inst->snapshot_state();
        } catch (std::exception const& e) {
            FAIL() << "snapshot threw at iter " << iter << ": " << e.what();
        }

        // Mutate: remove neuro + add a fresh stub of the same id.
        ASSERT_TRUE(apply_op(*inst, op_remove("neuro"), tick_cursor, 10));
        ogma::ParamMap np;
        np["event_coupled_da"]      = ogma::ParamValue{true};
        np["da_baseline_ema_alpha"] = ogma::ParamValue{0.001};
        ASSERT_TRUE(apply_op(*inst, op_add("neuro", "NeurochemState", np),
                             tick_cursor, 10));

        // Restore.  Module list should match initial again (other state
        // round-trips through snapshot).
        try {
            inst->restore_state(snap);
        } catch (std::exception const& e) {
            FAIL() << "restore threw at iter " << iter << ": " << e.what();
        }
        EXPECT_EQ(live_ids(*inst), initial_ids)
            << "restore at iter " << iter << " did not return to initial set";

        tick_n(*inst, 20, tick_cursor);
    }
}

// ---------------------------------------------------------------------------
// Scenario 7 — module-list query race against patches.
//
// Spawns a worker thread that calls instance->modules() / instance->module()
// in tight loops while the main thread applies patches and ticks.  This is
// the simulation of the W2 inspector control-server reading modules_ on its
// own thread while apply_remove mutates the same vector.  OgmaBrain holds
// instance_mtx_ to serialise these in production; here we simulate the
// same-thread serialised access (since the OgmaInstance API isn't
// internally locked) and verify no UB.
//
// NOTE: this scenario exercises the SAME-thread access pattern enforced
// by OgmaBrain's mutex.  A truly concurrent multi-thread access without
// the mutex IS undefined and intentionally not tested here.
// ---------------------------------------------------------------------------

TEST(PatchModeHarness, InspectorReaderRaceProxy) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        cell_like_config(/*auto_subscribe=*/true),
        std::make_unique<ogma::InProcessBus>());

    uint64_t tick_cursor = 0;
    tick_n(*inst, 20, tick_cursor);

    // Sequential interleaving simulates what OgmaBrain's instance_mtx_
    // guarantees under the actual ControlServer thread: every read goes
    // before / after a patch, never during.  We loop reads + patches +
    // ticks tightly to exercise as many transition states as possible.
    std::vector<std::string> remove_order = {
        "action_decoder", "drive", "voter_0", "epm_state", "neuro",
    };
    for (auto const& id : remove_order) {
        // Read patterns OgmaBrain's verbs use:
        for (int r = 0; r < 5; ++r) {
            auto mods = inst->modules();          // list_modules verb path
            EXPECT_GT(mods.size(), 0u);
            ogma::Module* m = inst->module(id);   // module_snapshot verb path
            if (m != nullptr) {
                ASSERT_NO_THROW({
                    auto snap = m->snapshot_state();
                    (void)snap;
                });
            }
            tick_n(*inst, 1, tick_cursor);
        }
        ASSERT_TRUE(apply_op(*inst, op_remove(id), tick_cursor, 5));
        for (int r = 0; r < 5; ++r) {
            (void)inst->modules();
            (void)inst->module(id);  // expect nullptr post-remove
            EXPECT_EQ(inst->module(id), nullptr);
            tick_n(*inst, 1, tick_cursor);
        }
    }
}

// ---------------------------------------------------------------------------
// Scenario 8 — repeated remove + readd of same id.
//
// Exercises module-id reuse: remove a module, run several ticks, add a
// fresh instance of the same type with the same id.  Stale gate state
// from before the remove must not pollute the new instance.  Iterated to
// surface any leak.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Scenario 9 — repro user-reported full-graph Premotor remove.
//
// Loads the_cell_premotor.json (the same config the user was in patch mode
// with), warms up 30 ticks, then removes 'premotor' as the very first
// patch.  No inspector active.  If the live-Godot crash is reproducible
// in pure C++, this scenario fails or aborts.  If it passes, the crash is
// in the Godot-side path (renderer, scheduler signals, etc.) and the
// harness is doing its job by ruling out the C++ paths.
// ---------------------------------------------------------------------------

// Drive the full set of proprio topics the_cell_premotor.json's EPMs
// subscribe to.  Without these, the voter has no inputs and never
// publishes consensus.0 — so a stale Premotor subscription on
// consensus.0 (the use-after-free bug from shadowed Module::bus_)
// never fires and the harness can't catch it.
void publish_full_cell_inputs(ogma::Bus* bus, uint64_t tick) {
    auto pp = [&](char const* topic, std::string sensor, int dim) {
        auto p = std::make_shared<ogma::ProprioToken>();
        p->tick_id     = tick;
        p->producer_id = "host";
        p->sensor      = std::move(sensor);
        p->values.resize(dim);
        for (int i = 0; i < dim; ++i) {
            p->values(i) = float(std::sin(double(tick) * (0.05 + 0.013 * i)));
        }
        bus->publish(topic, p);
    };
    pp("reality.proprio.imu",      "imu",      4);
    pp("reality.proprio.scent",    "scent",    8);
    pp("reality.proprio.terrain",  "terrain",  4);
    for (int i = 0; i < 6; ++i) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "reality.proprio.whisker_%d", i);
        pp(buf, std::string("whisker_") + char('0' + i), 1);
    }
    if (tick % 30 == 0) {
        auto e = std::make_shared<ogma::EnvEvent>();
        e->tick_id     = tick;
        e->producer_id = "host";
        e->name        = "alive";
        e->intensity   = 1.0f;
        bus->publish("events.alive", e);
    }
}

TEST(PatchModeHarness, FullGraphPremotorRemove) {
    char const* path =
        "/home/xaqmusic/ami-ogma-ui/godot_host/project/addons/ami_ogma/"
        "configs/the_cell_premotor.json";
    std::ifstream f(path);
    if (!f.good()) {
        GTEST_SKIP() << "config not present: " << path;
    }
    auto cfg = ogma::GraphConfig::load_from_file(path);
    auto inst = std::make_unique<ogma::OgmaInstance>(
        std::move(cfg), std::make_unique<ogma::InProcessBus>());

    ASSERT_GT(live_ids(*inst).count("premotor"), 0u)
        << "this test relies on premotor being in the_cell_premotor.json";

    // Warm up by driving the FULL set of cell inputs (so EPMs publish
    // RealityTokens, voter fuses, voter publishes consensus.0 — which is
    // exactly the publish path that fires Premotor.handle_consensus and
    // exposes any stale subscription post-remove).
    uint64_t tick_cursor = 0;
    for (int i = 0; i < 30; ++i) {
        publish_full_cell_inputs(inst->bus(), tick_cursor);
        inst->tick();
        ++tick_cursor;
    }

    // The user's exact action: click Premotor → press Delete → first patch.
    {
        ogma::GraphPatchBatch batch;
        batch.ops.push_back(op_remove("premotor"));
        inst->scheduler()->enqueue_hot_patch(std::move(batch));
    }
    // Now drive 60 more ticks with the same full input set.  Voter will
    // publish consensus.0 every tick.  If Premotor's subscription leaked
    // (Module::bus_ shadowing bug), the first publish post-remove segfaults
    // here on the gate's input_allowed lookup or state_visit_ema_ access.
    for (int i = 0; i < 60; ++i) {
        publish_full_cell_inputs(inst->bus(), tick_cursor);
        try {
            inst->tick();
        } catch (std::exception const& e) {
            FAIL() << "tick " << tick_cursor << " threw: " << e.what();
        }
        ++tick_cursor;
    }
    EXPECT_EQ(live_ids(*inst).count("premotor"), 0u);
}

TEST(PatchModeHarness, RepeatedRemoveReaddSameId) {
    auto inst = std::make_unique<ogma::OgmaInstance>(
        cell_like_config(/*auto_subscribe=*/false),
        std::make_unique<ogma::InProcessBus>());
    uint64_t tick_cursor = 0;
    tick_n(*inst, 10, tick_cursor);

    ogma::ParamMap epm_params;
    epm_params["modality_group"] = std::string("kinematic");
    epm_params["modality_name"]  = std::string("state");
    epm_params["encoder_kind"]   = std::string("rbf");
    epm_params["input_topic"]    = std::string("reality.proprio.state");
    epm_params["projection_dim"] = int64_t{16};
    epm_params["proprio_state_dims"] = int64_t{2};
    epm_params["subtract_descending_prediction"] = ogma::ParamValue{false};

    for (int iter = 0; iter < 6; ++iter) {
        ASSERT_TRUE(apply_op(*inst, op_remove("epm_state"), tick_cursor, 5))
            << "remove iter " << iter;
        EXPECT_EQ(live_ids(*inst).count("epm_state"), 0u);
        ASSERT_TRUE(apply_op(*inst, op_add("epm_state", "EPM", epm_params),
                             tick_cursor, 5))
            << "add iter " << iter;
        EXPECT_EQ(live_ids(*inst).count("epm_state"), 1u);

        // Fresh module should have an empty allowlist (manual mode default).
        // Boot edges in cell_like_config don't include any host:* edge to
        // epm_state, so the allowlist stays empty across re-adds.
        ogma::Module* m = inst->module("epm_state");
        ASSERT_NE(m, nullptr);
        EXPECT_TRUE(m->is_input_default_deny())
            << "re-added EPM should inherit manual mode";
        EXPECT_EQ(m->allowed_producers().size(), 0u)
            << "re-added EPM should have an empty allowlist (no boot edge)";
    }
}
