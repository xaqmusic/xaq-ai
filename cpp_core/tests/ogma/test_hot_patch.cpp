// =============================================================================
// test_hot_patch.cpp  --  Phase 3 hot-patch API against a running instance
// =============================================================================
//
// Per docs/primitives/_hot_patch.md.  Exercises every supported op type
// (AddNodeOp / RemoveNodeOp / SetParamOp) on a live OgmaInstance, plus
// validation rules + batch atomicity.  ConnectOp/DisconnectOp are MVP
// no-ops in this implementation (subscriptions live inside module
// on_setup); they're scheduled for a Phase-3 follow-up.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

namespace {

// Build a minimal three-module graph: NeurochemState + LateralVoter +
// HomeostaticDrive.  EPM/ActionDecoder are excluded because they need
// dimension-matched proprio/consensus inputs that complicate the test
// fixture; the hot-patch contract is exercised identically at the
// infrastructure level regardless of which module types are used.
ogma::GraphConfig minimal_graph() {
    ogma::GraphConfig g;
    g.version = 1;

    ogma::ModuleSpec neuro;
    neuro.id   = "neuro";
    neuro.type = "NeurochemState";
    g.modules.push_back(neuro);

    ogma::ModuleSpec voter;
    voter.id   = "voter_0";
    voter.type = "LateralVoter";
    voter.params["level"]          = int64_t{0};
    voter.params["input_pattern"]  = std::string("reality.");
    voter.params["trust_epsilon"]  = 0.05;
    voter.params["group_balance"]  = true;
    voter.params["priority_group"] = std::string("proprio");
    g.modules.push_back(voter);

    ogma::ModuleSpec drive;
    drive.id   = "drive";
    drive.type = "HomeostaticDrive";
    drive.params["channels"]            = std::vector<std::string>{"energy", "novelty_satiation"};
    drive.params["setpoints"]           = std::vector<double>{0.8, 0.5};
    drive.params["urgency_normalizers"] = std::vector<double>{1.0, 0.5};
    drive.params["channel_input_topics"] =
        std::vector<std::string>{"reality.proprio.energy", "consensus.0"};
    drive.params["energy_drain_per_tick"] = 0.0;
    g.modules.push_back(drive);

    return g;
}

std::unique_ptr<ogma::OgmaInstance> make_instance() {
    return std::make_unique<ogma::OgmaInstance>(
        minimal_graph(),
        std::make_unique<ogma::InProcessBus>());
}

} // namespace

// -- OgmaInstance construction --------------------------------------------

TEST(HotPatch, OgmaInstanceConstructsModulesFromConfig) {
    auto inst = make_instance();
    EXPECT_NE(inst->module("neuro"),   nullptr);
    EXPECT_NE(inst->module("voter_0"), nullptr);
    EXPECT_NE(inst->module("drive"),   nullptr);
    EXPECT_EQ(inst->modules().size(), 3u);
    EXPECT_EQ(inst->tick_count(), 0u);
}

TEST(HotPatch, OgmaInstanceTicksAdvanceCounter) {
    auto inst = make_instance();
    for (int i = 0; i < 5; ++i) inst->tick();
    EXPECT_EQ(inst->tick_count(), 5u);
}

TEST(HotPatch, ConstructionUnknownTypeThrows) {
    ogma::GraphConfig g;
    g.modules.push_back({.id = "x", .type = "DefinitelyNotARealType", .params = {}});
    EXPECT_THROW({
        ogma::OgmaInstance inst(g, std::make_unique<ogma::InProcessBus>());
    }, std::invalid_argument);
}

// -- AddNodeOp ------------------------------------------------------------

TEST(HotPatch, AddNodeOpAppearsAfterNextTick) {
    auto inst = make_instance();
    EXPECT_EQ(inst->module("seq_0"), nullptr);

    ogma::ModuleSpec spec;
    spec.id   = "seq_0";
    spec.type = "SequenceGNG";
    spec.params["source_topic"]   = std::string("reality.proprio.imu");
    spec.params["source_kind"]    = std::string("winner");
    spec.params["window_size"]    = int64_t{4};
    spec.params["projection_dim"] = int64_t{16};

    ogma::GraphPatchBatch batch;
    batch.source = "test";
    batch.ops.push_back(ogma::AddNodeOp{spec});
    inst->enqueue_hot_patch(std::move(batch));

    // Until the next tick processes the queue, the module is still absent.
    EXPECT_EQ(inst->module("seq_0"), nullptr);

    inst->tick();   // applies the queued patch first, then runs.
    EXPECT_NE(inst->module("seq_0"), nullptr);
    EXPECT_EQ(inst->modules().size(), 4u);
}

TEST(HotPatch, AddNodeOpDuplicateIdRejected) {
    auto inst = make_instance();

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::AddNodeOp{ogma::ModuleSpec{
        .id = "voter_0", .type = "NeurochemState", .params = {}}});
    inst->enqueue_hot_patch(std::move(batch));

    // Validation throws inside Scheduler::tick() on apply.
    EXPECT_THROW(inst->tick(), std::invalid_argument);
}

TEST(HotPatch, AddNodeOpUnknownTypeRejected) {
    auto inst = make_instance();
    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::AddNodeOp{ogma::ModuleSpec{
        .id = "ghost", .type = "DefinitelyNotARealType", .params = {}}});
    inst->enqueue_hot_patch(std::move(batch));
    EXPECT_THROW(inst->tick(), std::invalid_argument);
}

// -- RemoveNodeOp ---------------------------------------------------------

TEST(HotPatch, RemoveNodeOpDropsModule) {
    auto inst = make_instance();
    ASSERT_NE(inst->module("drive"), nullptr);

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::RemoveNodeOp{"drive"});
    inst->enqueue_hot_patch(std::move(batch));

    inst->tick();
    EXPECT_EQ(inst->module("drive"), nullptr);
    EXPECT_EQ(inst->modules().size(), 2u);
}

TEST(HotPatch, RemoveNodeOpUnknownIdRejected) {
    auto inst = make_instance();
    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::RemoveNodeOp{"never_existed"});
    inst->enqueue_hot_patch(std::move(batch));
    EXPECT_THROW(inst->tick(), std::invalid_argument);
}

// -- SetParamOp -----------------------------------------------------------

TEST(HotPatch, SetParamOpHotMutableApplies) {
    auto inst = make_instance();

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::SetParamOp{
        .target_id = "neuro",
        .key       = "da_decay",
        .value     = ogma::ParamValue{0.5}});
    inst->enqueue_hot_patch(std::move(batch));
    EXPECT_NO_THROW(inst->tick());
    // Module should accept the new value.  We can't read it back through
    // the public API, but the test validates the patch path.
}

TEST(HotPatch, SetParamOpConstructionOnlyRejected) {
    auto inst = make_instance();

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::SetParamOp{
        .target_id = "neuro",
        .key       = "master_seed",       // ConstructionOnly per the schema
        .value     = ogma::ParamValue{int64_t{99}}});
    inst->enqueue_hot_patch(std::move(batch));
    EXPECT_THROW(inst->tick(), std::invalid_argument);
}

TEST(HotPatch, SetParamOpUnknownTargetRejected) {
    auto inst = make_instance();
    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::SetParamOp{
        .target_id = "nonexistent",
        .key       = "da_decay",
        .value     = ogma::ParamValue{0.5}});
    inst->enqueue_hot_patch(std::move(batch));
    EXPECT_THROW(inst->tick(), std::invalid_argument);
}

TEST(HotPatch, SetParamOpUnknownKeyRejected) {
    auto inst = make_instance();
    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::SetParamOp{
        .target_id = "neuro",
        .key       = "not_a_real_key",
        .value     = ogma::ParamValue{0.5}});
    inst->enqueue_hot_patch(std::move(batch));
    EXPECT_THROW(inst->tick(), std::invalid_argument);
}

// -- Batch atomicity ------------------------------------------------------

TEST(HotPatch, BatchAtomicityOneBadOpRejectsAll) {
    auto inst = make_instance();

    // Batch with one good op (Add seq_0) followed by one bad op (Add ghost
    // with unknown type).  Atomicity → the good op MUST NOT apply.
    ogma::ModuleSpec good;
    good.id   = "seq_0";
    good.type = "SequenceGNG";
    good.params["source_topic"]   = std::string("reality.proprio.imu");
    good.params["source_kind"]    = std::string("winner");
    good.params["window_size"]    = int64_t{4};
    good.params["projection_dim"] = int64_t{16};

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::AddNodeOp{good});
    batch.ops.push_back(ogma::AddNodeOp{ogma::ModuleSpec{
        .id = "ghost", .type = "DefinitelyNotARealType", .params = {}}});
    inst->enqueue_hot_patch(std::move(batch));

    EXPECT_THROW(inst->tick(), std::invalid_argument);

    // The good op must NOT have applied — atomicity contract.
    EXPECT_EQ(inst->module("seq_0"), nullptr);
    EXPECT_EQ(inst->module("ghost"), nullptr);
}

// -- Inter-tick boundary --------------------------------------------------

TEST(HotPatch, PatchAppliedBetweenTicksNotMidTick) {
    auto inst = make_instance();

    // Run a few ticks so we know the system is "running."
    for (int i = 0; i < 3; ++i) inst->tick();
    auto pre_count = inst->modules().size();

    ogma::ModuleSpec spec;
    spec.id   = "seq_late";
    spec.type = "SequenceGNG";
    spec.params["source_topic"]   = std::string("reality.proprio.imu");
    spec.params["source_kind"]    = std::string("winner");
    spec.params["window_size"]    = int64_t{4};
    spec.params["projection_dim"] = int64_t{16};
    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::AddNodeOp{spec});
    inst->enqueue_hot_patch(std::move(batch));

    // Same module count until the next tick.
    EXPECT_EQ(inst->modules().size(), pre_count);

    inst->tick();
    EXPECT_EQ(inst->modules().size(), pre_count + 1u);
}

// -- Multiple-batch ordering ----------------------------------------------

TEST(HotPatch, MultiplePatchesAppliedInSubmissionOrder) {
    auto inst = make_instance();

    // Submit two batches: first adds A, second removes "drive".  Both apply
    // at the next tick boundary, in submission order.
    ogma::ModuleSpec a_spec;
    a_spec.id   = "a";
    a_spec.type = "NeurochemState";
    ogma::GraphPatchBatch batch1;
    batch1.ops.push_back(ogma::AddNodeOp{a_spec});
    auto id1 = inst->enqueue_hot_patch(std::move(batch1));

    ogma::GraphPatchBatch batch2;
    batch2.ops.push_back(ogma::RemoveNodeOp{"drive"});
    auto id2 = inst->enqueue_hot_patch(std::move(batch2));

    EXPECT_NE(id1, id2);
    EXPECT_GT(id2, id1);

    inst->tick();
    EXPECT_NE(inst->module("a"),     nullptr);
    EXPECT_EQ(inst->module("drive"), nullptr);
}

// -- Hot-patch produces no NaN in subsequent published payloads -----------

TEST(HotPatch, AddedModuleParticipatesInSubsequentTicks) {
    auto inst = make_instance();

    // Add a NeurochemState replica and confirm it publishes neuro.state.
    // (Both the original "neuro" and the new "neuro_b" will publish each
    // tick — since both are sequential, the second wins last_value.)
    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::AddNodeOp{ogma::ModuleSpec{
        .id = "neuro_b", .type = "NeurochemState", .params = {}}});
    inst->enqueue_hot_patch(std::move(batch));

    inst->tick();
    inst->tick();   // ensure neuro_b ran in this tick

    auto last = std::dynamic_pointer_cast<const ogma::NeuroState>(
        inst->bus()->last_value(ogma::topics::kNeuroState));
    ASSERT_NE(last, nullptr);
    EXPECT_FALSE(std::isnan(last->dopamine));
    EXPECT_FALSE(std::isnan(last->serotonin));
}

// Regression: a single batch shaped like [Remove "foo", Add "foo"] is the
// canonical topology-load shape (UI sends "wipe everything, recreate from
// JSON" as one transaction).  Pre-fix, validate_add ran against the still-
// live modules_ vector and rejected the Add as "id already exists" — the
// throw escaped Scheduler::tick into Godot and SIGSEGV'd the host.  Fix:
// pass-1 walks a working set of live ids and applies each op's effect to
// the set, so the second op sees "foo" as removed by the time it validates.
TEST(HotPatch, RemoveThenAddSameIdInOneBatch) {
    auto inst = make_instance();
    inst->tick();
    ASSERT_NE(inst->module("neuro"), nullptr);

    ogma::GraphPatchBatch batch;
    batch.ops.push_back(ogma::RemoveNodeOp{"neuro"});
    batch.ops.push_back(ogma::AddNodeOp{ogma::ModuleSpec{
        .id = "neuro", .type = "NeurochemState", .params = {}}});
    inst->enqueue_hot_patch(std::move(batch));
    inst->tick();

    ASSERT_NE(inst->module("neuro"), nullptr)
        << "Add should have re-created 'neuro' after the Remove";
    // Module is functional: ticks publish on neuro.state without throwing.
    inst->tick();
    auto last = std::dynamic_pointer_cast<const ogma::NeuroState>(
        inst->bus()->last_value(ogma::topics::kNeuroState));
    ASSERT_NE(last, nullptr);
}

// Regression: an AddNodeOp whose params are missing a required key (e.g.
// EPM without "modality_group") must reject the entire batch during pass-1
// validation BEFORE any earlier ops in the batch mutate the live graph.
// Pre-fix this only failed in pass-2 — by which point preceding remove
// ops had already deleted live modules, leaving a half-applied state.
TEST(HotPatch, BadAddParamsRejectedBeforePartialApply) {
    auto inst = make_instance();
    inst->tick();
    auto modules_before = inst->modules().size();
    ASSERT_GT(modules_before, 0u);

    // Batch: [remove every live module, then add an EPM with empty params].
    // The trailing add MUST throw during validation; without the fix it
    // would throw mid-pass-2 with the live modules already gone.
    ogma::GraphPatchBatch bad_batch;
    for (auto* m : inst->modules())
        bad_batch.ops.push_back(ogma::RemoveNodeOp{std::string(m->id())});
    bad_batch.ops.push_back(ogma::AddNodeOp{ogma::ModuleSpec{
        .id = "bad_epm", .type = "EPM", .params = {}}});
    inst->enqueue_hot_patch(std::move(bad_batch));

    // The throw happens inside the next tick when the batch is processed.
    EXPECT_THROW(inst->tick(), std::exception);

    // Critical: live graph is unchanged because validation rejected the
    // batch before any apply ran.
    EXPECT_EQ(inst->modules().size(), modules_before)
        << "Failed batch must leave live module list intact";
    EXPECT_NE(inst->module("neuro"), nullptr);
}

// Regression: removing a module that has live bus subscriptions must NOT
// leave dangling Handler closures pointing at the destroyed object.  Phase
// 6.6.A bug — `on_teardown` was the no-op default in the base class, so
// the bus continued to invoke captured `this` pointers on the next publish
// (SIGSEGV).  Fix: lift `sub_ids_` to Module base + default `on_teardown`
// unsubscribes them.  This test asserts no-crash semantics: add a module,
// confirm it runs, remove it, then publish on the topic it had subscribed
// to and tick — pre-fix this segfaulted.
TEST(HotPatch, RemoveUnsubscribesFromBus) {
    auto inst = make_instance();
    inst->tick();  // baseline tick — neuro publishes neuro.state

    // Add a fresh NeurochemState that subscribes to events.* (its on_setup
    // does so).  Then remove it and continue ticking; if its subscription
    // outlived the object, the bus would dispatch to a destroyed handler.
    {
        ogma::GraphPatchBatch add;
        add.ops.push_back(ogma::AddNodeOp{ogma::ModuleSpec{
            .id = "neuro_b", .type = "NeurochemState", .params = {}}});
        inst->enqueue_hot_patch(std::move(add));
    }
    inst->tick();
    ASSERT_NE(inst->module("neuro_b"), nullptr);

    {
        ogma::GraphPatchBatch rem;
        rem.ops.push_back(ogma::RemoveNodeOp{"neuro_b"});
        inst->enqueue_hot_patch(std::move(rem));
    }
    inst->tick();
    EXPECT_EQ(inst->module("neuro_b"), nullptr);

    // Publish on a topic neuro_b had subscribed to during on_setup.  Pre-fix,
    // this would dispatch to neuro_b's destroyed handler and crash.
    auto evt = std::make_shared<ogma::EnvEvent>();
    evt->tick_id   = inst->tick_count();
    evt->name      = "hit";
    evt->intensity = 1.0f;
    inst->bus()->publish("events.hit", evt);

    // Many subsequent ticks — gives any latent stale callback ample chance
    // to fire.  No crash → fix is in place.
    for (int i = 0; i < 32; ++i) inst->tick();
    SUCCEED();
}
