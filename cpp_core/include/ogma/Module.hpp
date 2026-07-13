#pragma once

// =============================================================================
// Module.hpp  --  Base class for every Ogma Core cognitive module
// =============================================================================
//
// A Module is the unit of computation the Scheduler orchestrates.  Every
// cognitive primitive (NeurochemState, EPM, LateralVoter, HomeostaticDrive,
// ActionDecoder, DescendingPredictor, SequenceGNG, GNGRollout,
// MotorRepertoire) inherits from this class.
//
// Pillar 1 Critical Rules that this interface enforces:
//
//   1. Modules communicate ONLY through the Bus pointer they receive at setup.
//      No static globals, no direct neighbour pointers, no shared buffers.
//   2. Modules are thread-safe with respect to their OWN state — their tick()
//      may run in parallel with sibling modules at the same DAG level.
//   3. Modules declare their topic surface up front.  The Scheduler uses
//      input_topics() and output_topics() to build the DAG; runtime topic
//      changes go through Module::set_param() or the hot-patch API, never
//      through magic Bus-side dispatch tricks.
//   4. Modules expose a parameter schema so the diff-patch API and the v3
//      launcher-style UI controls can mutate hot-mutable params at runtime.
//
// Lifecycle
// ---------
//
//      construction (default ctor + params from JSON)
//         |
//      on_setup(Bus*, ParamMap)        <-- subscribe to topics here
//         |
//   ┌─── tick(t)  -----  tick(t+1)  -----  tick(t+2)  ...    (per global tick)
//   │     │
//   │     └- module reads Bus::last_value() / Direct dispatched
//   │       payloads, writes its own state, calls Bus::publish().
//   │
//   └─── on_param_change(...)  <- hot-mutable param updates between ticks
//
//      on_teardown()                    <-- called once before destruction

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <typeindex>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "ogma/Bus.hpp"

namespace ogma {

// -----------------------------------------------------------------------------
// Topic spec — what a module publishes / subscribes
// -----------------------------------------------------------------------------
//
// `name` is either an exact topic name ("neuro.state") or a trailing-dot
// prefix ("reality.video.").  `kind` mirrors Bus::SubscriptionKind for inputs;
// outputs always declare exact topic names.

struct TopicSpec {
    std::string         name;
    std::type_index     payload_type;       // typeid(ConcreteMessage)
    SubscriptionKind    kind        = SubscriptionKind::Direct;
    bool                required    = true; // false → topic is optional input

    TopicSpec(std::string n, std::type_index t,
              SubscriptionKind k = SubscriptionKind::Direct,
              bool req = true)
        : name(std::move(n)), payload_type(t), kind(k), required(req) {}
};

// -----------------------------------------------------------------------------
// Parameter schema
// -----------------------------------------------------------------------------
//
// `ParamValue` covers the JSON-loadable scalar/list types we need for graph
// configs.  More complex parameters (e.g. an Eigen matrix) are loaded inside
// the module's on_setup() from a side-channel file referenced by name.

using ParamValue = std::variant<bool,
                                int64_t,
                                double,
                                std::string,
                                std::vector<double>,
                                std::vector<std::string>>;

using ParamMap = std::unordered_map<std::string, ParamValue>;

enum class ParamMutability {
    ConstructionOnly,   // only valid when first creating the module
    HotMutable,         // can be changed at runtime via set_param()
};

struct ParamSpec {
    std::string         key;
    ParamMutability     mutability    = ParamMutability::ConstructionOnly;
    std::string         description;
    std::optional<ParamValue> default_value;     // nullopt = required
    std::optional<ParamValue> min_value;         // for numeric ranges
    std::optional<ParamValue> max_value;
};

using ParamSchema = std::vector<ParamSpec>;

// -----------------------------------------------------------------------------
// Module base class
// -----------------------------------------------------------------------------

class Module {
public:
    virtual ~Module() = default;

    // -------------------------------------------------------------------------
    // Identity and topic surface  (pure virtual; declared by every module)
    // -------------------------------------------------------------------------

    // Module ID assigned by the graph config (e.g. "epm_retinal").  Unique
    // within the OgmaInstance.  Set by the Runtime at construction; modules
    // should not override.
    std::string_view id() const { return id_; }

    // The module class name, for telemetry and contract docs (e.g. "EPM").
    virtual std::string_view type_name() const = 0;

    virtual std::vector<TopicSpec> input_topics()  const = 0;
    virtual std::vector<TopicSpec> output_topics() const = 0;
    virtual ParamSchema             params_schema() const = 0;

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    // Called once after construction.  Module records `bus_`, validates
    // params against its schema, and registers Bus subscriptions for every
    // topic returned by input_topics().  Throws on invalid params.
    virtual void on_setup(Bus* bus, ParamMap const& params) = 0;

    // Called once per global tick by the Scheduler, after Bus::begin_tick() and
    // before Bus::end_tick(), at the level the Scheduler assigned.  All
    // current-tick reads of upstream topics are guaranteed to be available.
    // Modules in the same level run in parallel; modules in different levels
    // are sequential within the tick.
    virtual void tick(uint64_t tick_id) = 0;

    // Hot-mutable param update.  Called between ticks (never during).  Module
    // re-validates the value and applies it to its working state.  Throws on
    // construction-only or unknown keys.
    virtual void on_param_change(std::string_view /*key*/,
                                 ParamValue const& /*value*/) {}

    // Snapshot of the module's CURRENT parameter values for any subset of its
    // schema keys.  Surfaces to the inspector / Patch Mode param editor so a
    // human can see what's actually set without having to grep the config.
    // Default returns empty — each module SHOULD override and populate the
    // params it tracks.  Falling back to empty causes the editor to display
    // schema defaults instead, which is informative but not authoritative.
    virtual ParamMap current_params() const { return {}; }

    // Called once before destruction.  Default impl unsubscribes every
    // bus subscription the module recorded in `sub_ids_` during on_setup.
    // This is the cleanup that prevents the Bus from invoking dangling
    // Handler closures (whose captured `this` was just destroyed) on the
    // next publish — a hard segfault that bit Phase 6.6.A when the user
    // removed an EPM at runtime.  Modules with extra teardown work should
    // override but call Module::on_teardown() to keep this cleanup intact.
    virtual void on_teardown() {
        if (bus_ == nullptr) return;
        for (auto id : sub_ids_) bus_->unsubscribe(id);
        sub_ids_.clear();
    }

    // -------------------------------------------------------------------------
    // Snapshot / restore  (Phase 6.5.4 — instance cloning)
    // -------------------------------------------------------------------------
    //
    // `snapshot_state()` serializes ALL working state that affects future
    // behavior: member fields, accumulated maps/graphs, pending event counters,
    // RNG state, cached references to recent bus messages.  Static parameters
    // (those in params_schema()) are NOT included — they are part of the
    // GraphConfig snapshot taken separately.
    //
    // `restore_state(json)` repopulates the module from a previously-taken
    // snapshot.  After restore, the module must produce IDENTICAL outputs to
    // the source module if both are ticked from the same point onward.  This
    // is the determinism contract that makes evolutionary fork experiments
    // and rewind-style debugging meaningful.
    //
    // Default impls: return empty / no-op.  Modules with persistent state
    // override both.  Modules with no state (pure functions) may opt out.
    //
    // The JSON includes a `"version"` field so future schema migrations can
    // reject incompatible snapshots cleanly rather than silently misbehaving.
    virtual nlohmann::json snapshot_state() const;
    virtual void           restore_state(nlohmann::json const&);

    // -------------------------------------------------------------------------
    // Diagnostic snapshot — lightweight payload for the diag stream
    // -------------------------------------------------------------------------
    //
    // The diag pub stream calls this at the subscriber-chosen rate (typically
    // 30 Hz) to ship module state to inspector widgets.  Default impl delegates
    // to snapshot_state() so any module that hasn't been tuned for live
    // observation still works — at the cost of shipping the full clone-ready
    // payload over ZMQ each tick.
    //
    // Modules whose snapshot_state() grows unboundedly with run-time (e.g.
    // GNG-based modules whose serialised nodes include high-D weight vectors,
    // or maps with O(N^2) edges) should override this to return only the
    // fields actually consumed by the corresponding inspector widget.  Keep
    // the JSON shape stable so the widget code doesn't have to branch.
    virtual nlohmann::json diag_snapshot() const;

    // -------------------------------------------------------------------------
    // Per-primitive input gate (aux-send routing)
    // -------------------------------------------------------------------------
    //
    // Default-deny allowlist of producer ids whose messages this module
    // admits into its handlers.  Empty + default_deny=true means nothing
    // gets through; in default_deny=false (auto-subscribe) mode the gate
    // bypasses the check entirely so back-compat behaviour is preserved.
    //
    // The gate is consulted at the top of every `handle_*` method via
    // `input_allowed(payload->producer_id)`.  Mutated only by the
    // Scheduler between ticks (when ConnectOp/DisconnectOp fire or when
    // auto_subscribe is toggled).  Safe to read concurrently from
    // handlers running inside a tick.
    //
    // See docs/.../adaptive-twirling-babbage plan for the full design.

    bool input_allowed(std::string_view producer_id) const {
        if (!input_default_deny_) return true;
        return allowed_producers_.count(std::string(producer_id)) > 0;
    }

    // Scheduler calls these between ticks when edges_ changes affect this
    // module (its `to`).  Default impls just update the allowlist;
    // modules can override if they need to react to wiring changes
    // (e.g. drop a cached value from a now-banned producer).
    virtual void on_producer_allowed(std::string const& producer_id) {
        allowed_producers_.insert(producer_id);
    }
    virtual void on_producer_denied(std::string const& producer_id) {
        allowed_producers_.erase(producer_id);
    }
    void set_input_default_deny(bool deny) { input_default_deny_ = deny; }
    bool is_input_default_deny() const     { return input_default_deny_; }

    // Read-only accessors used by OgmaInstance::snapshot_state to
    // round-trip gate state alongside per-module snapshots.
    std::unordered_set<std::string> const& allowed_producers() const {
        return allowed_producers_;
    }
    void clear_allowed_producers() { allowed_producers_.clear(); }

    // -------------------------------------------------------------------------
    // Internal helpers (set by the Runtime; not for module overrides)
    // -------------------------------------------------------------------------

    void set_id(std::string id) { id_ = std::move(id); }

protected:
    Bus*        bus_ = nullptr;   // populated by on_setup() implementations
    std::string id_;              // populated by Runtime via set_id()
    // Bus subscription IDs.  Modules push every `bus_->subscribe(...)` return
    // value into this vector during on_setup so the default on_teardown can
    // tear them down cleanly when the module is hot-removed.  Centralised in
    // the base to enforce the invariant (Phase 6.6.A: per-module sub_ids_
    // declarations leaked subscriptions on hot-remove → bus delivered to
    // dangling Handler captures → SIGSEGV).
    std::vector<SubscriptionId> sub_ids_;

    // See "Per-primitive input gate" above for semantics.
    std::unordered_set<std::string> allowed_producers_;
    bool                            input_default_deny_ = false;
};

using ModulePtr = std::unique_ptr<Module>;

// -----------------------------------------------------------------------------
// Module factory registration
// -----------------------------------------------------------------------------
//
// Each module type registers a factory function keyed by its type_name() so
// the GraphConfig loader can instantiate by string.  The macro `OGMA_REGISTER_MODULE`
// in implementation files invokes this.

class ModuleRegistry {
public:
    using Factory = std::function<ModulePtr()>;

    static ModuleRegistry& instance();

    void register_type(std::string_view type_name, Factory factory);

    ModulePtr create(std::string_view type_name) const;

    std::vector<std::string> registered_types() const;

private:
    ModuleRegistry() = default;
    std::unordered_map<std::string, Factory> factories_;
};

} // namespace ogma
