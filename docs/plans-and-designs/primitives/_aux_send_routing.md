# Aux-Send Routing — Per-Primitive Subscription Filter

**Status:** shipped (commits `f4ef2c4` C++ foundation, `6333dbb` panel UI).
**Cross-cutting topic:** affects every Module, Scheduler, OgmaInstance, and
the Godot panel.  Read this before adding a new module type or changing
how the bus dispatches.

---

## 1. Why this exists

Before this feature, the bus delivered every published message to every
matching subscriber unconditionally.  `Scheduler::ConnectOp` and
`DisconnectOp` only mutated edge metadata — they had no effect on actual
data flow because each Module's `bus_->subscribe(topic, ...)` call from
`on_setup` stayed alive regardless.  The original Scheduler comment was
honest about it:

> Phase 3 MVP: these update the scheduler's edge list.  Full runtime
> Bus subscription re-binding (teaching modules to dynamically subscribe
> to new topics) requires either a module-level reconnect() hook or a
> topic-delegation mechanism in the Bus itself, both deferred to the
> ZmqBus work.

Two user-facing problems:

1. **Disconnect was a no-op for ablation** — the panel's "remove edge"
   action removed the visual line but data kept flowing.  Confusing and
   unsuitable for experimental brain authoring.
2. **No blank-canvas authoring** — every shipping config relied on
   implicit topic-match delivery.  There was no way to say "start with
   sources/sinks only and build the graph manually".

Aux-send routing solves both without changing the bus, by giving every
module its own input gate.

---

## 2. The model — audio aux-send routing

The mental model comes straight from analog audio mixing consoles:

- **The bus** is the patchbay — a shared signal carrier.  Every channel
  publishes to the bus; the bus is dumb pipes.
- **Each input** (sink) is an aux return on a primitive.  The primitive
  decides which producer ids it wants to listen to.
- **An edge** is an aux send wired from a producer's bus output to a
  primitive's aux return.  Adding an edge enables the route; removing
  it disables it.
- **Default-deny** is the manual-mode equivalent of an aux send with
  the level fader pulled all the way down — no signal passes until the
  user explicitly raises it.

This decentralises routing decisions to the primitives that consume
signals, which aligns with v4's "modules are autonomous primitives"
architecture.  The bus stays simple, snapshot/restore byte-equivalence
across clones is preserved, and richer per-edge processing (gain, delay,
transformations) becomes a non-breaking module-level extension later.

---

## 3. The contract

### 3.1 Modes

`GraphConfig.runtime.auto_subscribe` is the per-graph toggle.  Default
`true`.

| Mode                   | Gate behaviour                                                    |
|------------------------|-------------------------------------------------------------------|
| `true`  (auto)         | `input_allowed()` short-circuits to `true`.  Bus delivery applies as before. |
| `false` (manual)       | `input_allowed()` consults the per-module `allowed_producers_` set.  Empty set ⇒ nothing admitted. |

### 3.2 Producer ids

Every `Message` carries a `producer_id` field.  Convention:

- **Modules** stamp `producer_id = id_` (the module's GraphConfig id) on
  every published message.  This is what the gate matches on.
- **Hosts** (Godot, HAL Host, Debug Host) stamp `producer_id = "host"`
  on bridged sensor / event / video publishes.  Host-bridged edges in
  the scheduler's edge list have the form `from = "host:<topic>"`; the
  scheduler resolves these to producer_id `"host"` when populating
  receiver allowlists.

The resolver lives at the top of `Scheduler.cpp`:
```cpp
std::string resolve_producer_id(std::string const& edge_from);
```

### 3.3 Edge → allowlist propagation

The Scheduler is the single source of truth for routing topology.  It
fires three callbacks on the receiver Module whenever the edge list
changes:

| Trigger                              | Scheduler action                                          | Module callback         |
|--------------------------------------|-----------------------------------------------------------|-------------------------|
| Boot in manual mode                  | `install_manual_gates()` — set default-deny + seed allowlists from boot edges | `on_producer_allowed(p)` per edge |
| `apply_connect`                      | push edge → `edges_`                                      | `on_producer_allowed(p)` |
| `apply_disconnect`                   | erase edge → `edges_`; check if any other edge from same producer survives | `on_producer_denied(p)` only when no surviving edge |
| `set_auto_subscribe(false)`          | same as boot in manual mode                               | `on_producer_allowed(p)` per edge |
| `set_auto_subscribe(true)`           | clear `input_default_deny_` on every module               | (none)                  |

`on_producer_allowed/denied` are virtual; default impls update the set,
modules can override to react (e.g. drop a cached value from a
now-banned producer).

### 3.4 Toggle non-destructiveness

Toggling `auto → manual → auto` does not lose state.  When toggling to
auto, allowlists are left in place — only `input_default_deny_` flips.
That means a subsequent toggle back to manual restores the same
admit-set without re-seeding from `edges_`.

### 3.5 Snapshot / restore

Per-module gate state (`allowed_producers_`, `input_default_deny_`)
round-trips through `OgmaInstance::snapshot_state` at the **instance
level**, in a sibling `gates` dict keyed by module id.  Per-module
`snapshot_state` overrides do not need to know about the gate.  Older
snapshots without `gates` restore with default-allow gates (no
behaviour change, fully back-compat).

Cloned brains keep their wiring: `test_clone_shipping_configs` (17/6/0)
and `test_snapshot_disk_round_trip` continue to pass because auto mode
is the default and the gate is a no-op there.

---

## 4. API surface

### 4.1 Module base — `cpp_core/include/ogma/Module.hpp`

```cpp
// Read at the top of every handle_* method.
bool input_allowed(std::string_view producer_id) const;

// Default impls update allowed_producers_; override to react to wiring.
virtual void on_producer_allowed(std::string const& producer_id);
virtual void on_producer_denied (std::string const& producer_id);

// Scheduler-only — flips the gate's mode.
void set_input_default_deny(bool deny);

// Snapshot path uses these.
std::unordered_set<std::string> const& allowed_producers() const;
void clear_allowed_producers();
bool is_input_default_deny() const;
```

### 4.2 Scheduler — `cpp_core/include/ogma/Scheduler.hpp`

```cpp
// Per-graph toggle.  Mirrors RuntimeSpec.auto_subscribe at construction.
virtual void set_auto_subscribe(bool enabled) = 0;
virtual bool is_auto_subscribe() const        = 0;
```

Mutation hooks (private, in `MinimalScheduler`):
- `install_manual_gates()` — walk modules, set default-deny, seed allowlists from `edges_`.
- `apply_connect`/`apply_disconnect` — also call `on_producer_allowed/denied` on the receiver.

### 4.3 GraphConfig — `cpp_core/include/ogma/GraphConfig.hpp`

```cpp
struct RuntimeSpec {
    ThreadPoolPolicy thread_pool   = ThreadPoolPolicy::PerInstance;
    int              num_threads    = 0;
    bool             auto_subscribe = true;   // back-compat default
};
```

JSON I/O round-trip via `parse_runtime` / `runtime_to_json`.

### 4.4 OgmaBrain (Godot host) — `godot_host/src/OgmaBrain.cpp`

GD-callable methods:
```gdscript
brain.set_auto_subscribe(true|false)
brain.is_auto_subscribe() -> bool
```

Both protected by `instance_mtx_` (recursive_mutex shared with
`tick()` and the inspector control-server).

`get_graph_edges()` adds an `is_implicit: bool` field per edge dict —
`false` for boot config + scheduler `current_edges()` entries,
`true` for topic-derived synthesis.  The panel uses this for the
edge-hover tooltip.

Host stamping (every bridged publish):
```cpp
p->producer_id = "host";
```

### 4.5 Panel — `godot_host/project/scripts/ogma_graph_panel.gd`

- Auto-subscribe `CheckButton` in the patch toolbar, enabled only when
  patch mode is on.  Mirrors `brain.is_auto_subscribe()` on patch-mode
  entry; toggles call `brain.set_auto_subscribe(...)`.
- Edge-hover tooltip via mouse-motion on `graph.gui_input`.  Uses the
  existing bezier hit-test from the connection-delete feature.  Status
  bar reads `from → to (topic: X) — implicit` or `... — explicit`.

---

## 5. Adding a new module — checklist

Three things every new module must do correctly.  See
`docs/primitives/_module_lifecycle.md` for the full lifecycle contract
(on_setup / on_teardown semantics, base-class member list, snapshot
versioning rules).

### 5.0 Don't redeclare inherited members

`Module` already owns `bus_`, `id_`, `sub_ids_`, `allowed_producers_`,
and `input_default_deny_`.  Subclasses must use the inherited ones —
never redeclare them.  C++ will compile a shadowing redeclaration
silently, but `on_setup` will write the derived field while
`on_teardown` reads the base field, leaking subscription closures past
module destruction.  This is exactly the bug that caused the Premotor
remove crash on 2026-05-07; full post-mortem in
`_module_lifecycle.md` § 6.

### 5.1 Stamp `producer_id` on every published Message

```cpp
auto p = std::make_shared<MyToken>();
p->tick_id     = tick_id;
p->producer_id = id_;          // <-- this is the gate-matching key
// ... rest of the payload
bus_->publish(my_output_topic_, p);
```

Forgetting this means the message arrives at every receiver with an
empty `producer_id`, which **never matches** any `allowed_producers_`
entry in manual mode.  The receiver silently drops it.  You'll notice
because `test_manual_routing` patterns will fail when wired up.

### 5.2 Gate every input handler

```cpp
void MyModule::handle_input(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;   // <-- one-liner
    // ... existing handler logic
}
```

Every method bound to `bus_->subscribe()` needs this at the top.  Place
it before any cast or dereference of `payload` so default-deny is the
first decision, not a partial parse.

In auto mode (the default), `input_allowed` short-circuits to true so
you pay nothing.  In manual mode it's an `unordered_set::count` —
single hash lookup, negligible.

### 5.3 (Optional) Override `on_producer_allowed/denied` for richer routing

Default impls just maintain the allowlist.  Override when the module
wants to react to wiring changes:

```cpp
void MyModule::on_producer_allowed(std::string const& producer_id) override {
    Module::on_producer_allowed(producer_id);
    // e.g. allocate per-producer state, reset a per-source EMA, ...
}

void MyModule::on_producer_denied(std::string const& producer_id) override {
    Module::on_producer_denied(producer_id);
    // e.g. drop the cached last-value from this producer so it can't
    // be read on the next tick after disconnect
    cached_per_producer_.erase(producer_id);
}
```

### 5.4 Snapshot/restore is automatic

Gate state is captured at the instance level (`OgmaInstance::snapshot_state`
emits a sibling `gates` field) and restored before per-module
`restore_state` runs.  Your module's per-module snapshot doesn't need
to know about it — but if you override `on_producer_allowed/denied` to
maintain per-producer module state, *that* state DOES need to live in
your own `snapshot_state` override.

---

## 6. Hook points — what to update when changing behaviour

When making a change in any of these areas, check the listed hook points
and update them in lockstep:

### 6.1 Adding a new payload type

In addition to `Topics.hpp`:
- `cpp_core/include/ogma/PayloadTypeName.hpp` / `.cpp` — add the new
  type to `payload_type_name()`.  The W1 graph panel renders ports
  coloured by this name.
- Make sure every publisher of the new type stamps `producer_id`.

### 6.2 Adding a new host-bridge call (Godot or HAL)

In `OgmaBrain.cpp` (or HAL host equivalent):
- Stamp `producer_id = "host"` on the published Message (same line that
  sets `tick_id`).
- The boundary-edge form `from = "host:<topic>"` automatically resolves
  to producer_id `"host"` via `resolve_producer_id` — no scheduler
  changes required.

### 6.3 Adding a new patch-op variant (e.g. ReroutOp)

In `Scheduler.cpp`:
- The op's `apply_*` method must call `on_producer_allowed` /
  `on_producer_denied` on the affected receiver(s) so manual-mode
  routing stays in sync.
- Validation pass (`validate_*_against`) must accept `host:` prefixes
  for endpoints that may resolve to host bridges.

### 6.4 Adding a new module type

See § 5.  The mechanical bits are: subscribe in `on_setup`, gate every
handler with `input_allowed`, stamp `producer_id` on every publish.

### 6.5 Changing the bus implementation

The aux-send filter is **module-side** by design.  Bus implementations
(`InProcessBus`, future `ZmqBus`, `HybridBus`) do not need to know
about `producer_id` matching or `auto_subscribe`.  The contract is just
"deliver every Message to every matching subscriber" — modules filter
on receipt.  Snapshot/restore byte-equivalence across `clone()` depends
on this — don't push routing logic into the bus without a strong reason.

---

## 7. Future concepts

The producer-id-keyed `allowed_producers_` set is the minimum viable
gate.  Everything below is a non-breaking extension built on top of
the same hook points.

### 7.1 Per-source gain

Replace `unordered_set<string>` with `unordered_map<string, float>`
where the value is a multiplicative gain on incoming payloads.
`on_producer_allowed(p)` defaults gain to 1.0; a future panel
"send-level slider" mutates it.  Each module's handler reads the gain
and applies it at the appropriate place (e.g. EPM scales the incoming
proprio vector, NeurochemState scales the event intensity).

Snapshot field becomes `{"allowed": {"src_a": 1.0, "src_b": 0.5}}` —
keep `is_input_default_deny` as the boolean it is.

### 7.2 Per-source delay

`unordered_map<string, int>` of tick-delay values.  The module
buffers the last N values per producer and reads from `now − delay`.
Useful for synchronising fast and slow modalities at consensus
fusion, or for testing predictive-coding latency hypotheses.

### 7.3 Per-source transform

Each producer entry in the gate carries a small DAG of preprocessing
ops (clip, scale, lowpass, normalise).  Implemented as a chain of
small functors, configured per-edge.  Heavier than the gain/delay
extensions but maps directly onto audio EQ-on-aux-send conventions.

### 7.4 Mute groups

Named groups of edges that toggle together — "scent system",
"whisker reflexes", "all visual" — addressable from a panel sidebar.
Implementation: edges gain a `group: string` field, scheduler tracks
group → set<edge> map, mute toggle calls `on_producer_denied/allowed`
across the group.

### 7.5 Per-edge throughput stats

The bus already routes every payload through the per-module gate —
that's the natural meter point.  Each module increments a per-producer
counter when `input_allowed` returns true.  Counter snapshots every
tick into the inspector's diag stream as `producer_id → msgs/tick`
gives the panel an "edge flux" overlay (line thickness or colour
intensity proportional to live throughput).

### 7.6 Hierarchical routing modes

Sub-graphs (e.g. a reflex chain inside a larger brain) running in
manual while the parent graph is auto.  Implementation: `RuntimeSpec`
gains a per-module `auto_subscribe` override, scheduler walks
modules and applies the most-specific setting.

### 7.7 Edge-driven dynamic subscription

Instead of every module calling `bus_->subscribe()` at `on_setup` for
its declared `input_topics()`, the scheduler could drive subscriptions
from the edge list — a module subscribes only to topics that have an
incoming edge to it.  This eliminates the gate entirely (no defaulted
delivery to filter) but requires every module to expose a
`bind_input(topic, kind, handler)` method the scheduler can call.
Bigger refactor; defer until a use case demands it.

### 7.8 Auto-derived → materialised conversion

When toggling auto → manual, optionally copy every implicit edge
from `get_graph_edges()` into `edges_` so the manual-mode allowlist
captures the current data flow exactly.  Useful for "I want to start
manual but with the current wiring as my baseline".  Currently the
toggle is non-destructive in the other direction (rebuild from edges
on toggle to manual); this would make it symmetric.

---

## 8. Open questions / known limitations

### 8.1 Hot-patch tick-boundary lag

`enqueue_hot_patch` queues; `process_pending_patches` runs at the
start of the next tick.  Bus publishes are synchronous and fire
handlers immediately.  So between drag-connecting in the panel and
the first tick that applies the patch, one publish-cycle's worth of
data has already been gated by the old allowlist.  At 60 Hz this is
~16 ms — typically below human perception, but it can show up as a
brief "delay" between the click and the data starting to flow.

`test_manual_routing` documents the workaround: tick once after
enqueue with no publish so the patch lands before the measurement
loop.

### 8.2 Producer-id-only granularity

Today the gate matches purely on `producer_id`.  A primitive can't
say "I want consensus.0 from voter_0 but not drive.errors from
voter_0" — but in practice each v4 module owns one main output topic,
so the constraint isn't binding.  When it does bind, extend the gate
to `unordered_set<pair<producer_id, topic>>` (see § 7.3).

### 8.3 ConnectOp doesn't validate the receiver actually subscribes

Drawing an edge `host:reality.proprio.imu → my_module` allows
`producer_id="host"` through `my_module`'s gate — but if `my_module`
doesn't subscribe to `reality.proprio.imu` in its `on_setup`, the
edge is dead.  The panel shows the line; nothing flows.  Today this
is silent.  Future enhancement: `validate_connect_against` could
check that the receiver's declared `input_topics()` actually contains
the edge's topic and warn or reject otherwise.

### 8.4 Per-module gate state overhead

Every module pays for an `unordered_set<string>` even in auto mode
(empty, never read).  Empty hash table is ~56 bytes — 23 modules =
~1.3 KB.  Negligible.  Mentioned for completeness.

---

## 9. Verification recipes

### 9.1 Unit / integration

```bash
cd cpp_core && cmake --build build && ctest --output-on-failure
```

Expected:
- `test_clone` 3/3 PASS — clone byte-equivalence holds (auto mode default).
- `test_clone_shipping_configs` 17 PASS / 6 SKIP / 0 FAIL — every shipping
  config still passes byte-equiv determinism (gate is a no-op in auto mode).
- `test_snapshot_disk_round_trip` 1/1 PASS — disk-mediated snapshot
  round-trip (including `gates` field) survives.
- `test_manual_routing` 3/3 PASS — pins the contract.

### 9.2 Live brain via Godot host

1. Build the host: `cmake --build godot_host/build`
2. Launch a shipping config (e.g. `the_cell.json`) — auto mode default;
   behaviour identical to today.  Inspect a few modules in the inspector
   dashboard, confirm metrics still update.
3. In Patch Mode, toggle Auto-subscribe **OFF**.  All implicit edges
   stop carrying data; the visible signal chain (drives, consensus,
   action) goes silent — verify via NeurochemState dashboard (DA stops
   responding to events) and HomeostaticDrive dashboard (urgency
   frozen).
4. Drag-connect a single explicit edge from a body event source to
   NeurochemState.  Verify DA starts responding again on that event but
   not others.
5. Hover an edge: status bar reads `from → to (topic) — implicit` for
   derived edges and `... — explicit` for ones the user just drew.
6. Click an explicit edge, press Delete: edge disappears and the
   corresponding signal stops flowing.
7. Toggle Auto-subscribe back **ON**: all signals resume immediately.

### 9.3 Save / load round-trip

1. Save Topology while in manual mode with a few explicit edges.
   Reload from disk on a fresh boot.  Verify `auto_subscribe` is
   preserved in the saved JSON and the loaded brain has the same
   edges and same routing behaviour.
2. Save Snapshot in manual mode, restore — confirm
   `allowed_producers_` per module survives the round-trip (clone
   byte-equivalence test_clone-style).

---

## 10. Glossary

| Term                  | Meaning                                                                                                       |
|-----------------------|---------------------------------------------------------------------------------------------------------------|
| Aux send              | Audio-console concept of routing a tap of a channel signal to an external destination via a per-channel send. |
| Default-allow         | Gate behaviour in auto mode — `input_allowed` returns `true` regardless of the allowlist.                     |
| Default-deny          | Gate behaviour in manual mode — `input_allowed` returns `true` only for producer ids in the allowlist.        |
| Allowlist             | Per-module `unordered_set<std::string>` of producer ids whose messages are admitted.                          |
| Producer id           | The publisher's identity, stamped on every Message in `producer_id`.  Module id for modules; `"host"` for hosts. |
| Implicit edge         | Edge synthesized from publisher↔subscriber topic-match in `get_graph_edges`; not in the scheduler's `edges_`. |
| Explicit edge         | Edge present in either the boot `GraphConfig.edges` or the scheduler's `current_edges()` (post hot-patch).    |
| Auto mode             | `RuntimeSpec.auto_subscribe = true`.  Implicit edges deliver; gate is a no-op.                                |
| Manual mode           | `RuntimeSpec.auto_subscribe = false`.  Modules go default-deny; only explicit edges admit signal.             |
| Resolver              | `Scheduler.cpp::resolve_producer_id` — maps `host:<topic>` edge endpoints to the producer id `"host"`.        |

---

## 11. Related documents

- `docs/primitives/_module_lifecycle.md` — on_setup/on_teardown contract, base-class member list, post-mortem on the bus_ shadowing bug, full new-module checklist.
- `docs/primitives/_first_tick.md` — feedback subscription semantics, last-value cache.
- `docs/primitives/_hot_patch.md` — diff-patch API, ConnectOp/DisconnectOp.
- `docs/v4_phase6_5_4_clone_system.md` — snapshot / restore byte-equivalence.
- `docs/v4_ui_dev_module_state_audit.md` — per-module `snapshot_state` coverage.
- The plan that drove this work: `~/.claude/plans/adaptive-twirling-babbage.md`.
