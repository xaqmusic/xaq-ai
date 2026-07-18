# Module Lifecycle — Authoring Contract

Cross-cutting reference for every Ogma Core module subclass.  Read this
before adding a new module type or modifying an existing one's
`on_setup` / `on_teardown` / snapshot path.

Companion to:
- `docs/primitives/_first_tick.md` — first-tick / feedback semantics.
- `docs/primitives/_hot_patch.md`   — diff-patch API, ConnectOp / DisconnectOp.
- `docs/primitives/_aux_send_routing.md` — per-primitive subscription gate.

---

## 1. Lifecycle phases

```
construction (default ctor) → set_id() → on_setup(bus, params)
                                              │
                            ┌───── tick(t) ───┴── tick(t+1) ─── tick(t+2) ─── ...
                            │
                            └─── on_param_change(...)   (between ticks)
                            └─── on_producer_allowed/denied(producer_id)
                                                          (between ticks)

                                              ↓
                                       on_teardown()
                                              ↓
                                       destruction
```

Order guarantees:

| Phase | Caller | Invariants |
|---|---|---|
| Construction | `ModuleRegistry::create` | Default-constructed.  `bus_ == nullptr`, `id_ == ""`.  Member defaults from header active. |
| `set_id()` | `OgmaInstance` ctor or `apply_add` | `id_` populated.  Still no bus / params. |
| `on_setup(bus, params)` | Same as `set_id` | Module records `bus_`, validates params, registers Bus subscriptions, allocates working state. |
| `tick(tick_id)` | Scheduler module loop | All current-tick reads of upstream topics are guaranteed available.  Modules in different DAG levels are sequential within the tick. |
| `on_param_change` | Scheduler `apply_set_param` | Hot-mutable param updates between ticks.  Module re-validates and applies. |
| `on_producer_allowed/denied` | Scheduler `apply_connect/disconnect` / `set_auto_subscribe` | Allowlist mutations between ticks.  Default impls update `allowed_producers_`. |
| `on_teardown()` | Scheduler `apply_remove` (just before module destruction) | Module unsubscribes every entry it registered in `on_setup`. |
| Destruction | `unique_ptr<Module>::reset()` (during `apply_remove`) | Members destruct in reverse declaration order. |

---

## 2. Module base class — what's already on it

Members declared on the `Module` base class.  **Subclasses must not
redeclare any of these.**  See §6 for the post-mortem on what happens
when they do.

```cpp
protected:
    Bus*        bus_ = nullptr;                        // wrote in on_setup, read in on_teardown
    std::string id_;                                   // set by Runtime via set_id()
    std::vector<SubscriptionId> sub_ids_;              // every subscribe pushes here

    std::unordered_set<std::string> allowed_producers_; // aux-send filter (default-deny)
    bool                            input_default_deny_ = false;
```

Public helpers (do not override unless you have a specific reason):

```cpp
bool input_allowed(std::string_view producer_id) const;
virtual void on_producer_allowed(std::string const& producer_id);
virtual void on_producer_denied (std::string const& producer_id);
void set_input_default_deny(bool deny);

virtual void on_teardown();   // base impl unsubscribes every sub_ids_ entry
```

Public methods every subclass MUST override:

```cpp
virtual std::string_view             type_name()      const = 0;
virtual std::vector<TopicSpec>       input_topics()   const = 0;
virtual std::vector<TopicSpec>       output_topics()  const = 0;
virtual ParamSchema                  params_schema()  const = 0;
virtual void                         on_setup(Bus*, ParamMap const&) = 0;
virtual void                         tick(uint64_t tick_id) = 0;
```

Optional overrides (override when the module has hot-mutable params or
non-trivial working state):

```cpp
virtual void           on_param_change(std::string_view, ParamValue const&);
virtual ParamMap       current_params() const;
virtual nlohmann::json snapshot_state() const;
virtual void           restore_state(nlohmann::json const&);
virtual void           on_teardown();   // override if you need extra
                                         // teardown logic; ALWAYS call
                                         // Module::on_teardown() to keep
                                         // the unsubscribe loop intact.
```

---

## 3. on_setup contract

```cpp
void MyModule::on_setup(Bus* bus, ParamMap const& params) override {
    bus_ = bus;                                      // (a) record bus
    if (!bus_) throw std::invalid_argument("...");

    apply_param(params, "key", [&](auto const& v){ /* ... */ });

    sub_ids_.push_back(bus_->subscribe(             // (b) push every sub
        topic, kind,
        [this](auto t, auto p){ handle_xxx(t, p); }));

    // ... allocate working state, seed RNG, etc.
}
```

Three rules:

1. **(a)** Always assign `bus_ = bus` first.  Use the inherited member —
   never redeclare `Bus* bus_` on the subclass (see §6).
2. **(b)** Every `bus_->subscribe(...)` return value must land in
   `sub_ids_` so the base `on_teardown` can clean up.
3. **(c)** Throw on validation failure.  The Scheduler trial-constructs
   AddNodeOps against a throwaway bus first; throwing here surfaces as
   a clean error to the caller (`OgmaBrain::apply_patch` returns the
   error string back to GDScript).

---

## 4. on_teardown contract

The base class handles the common case for you:

```cpp
virtual void Module::on_teardown() {
    if (bus_ == nullptr) return;
    for (auto id : sub_ids_) bus_->unsubscribe(id);
    sub_ids_.clear();
}
```

**If you don't override `on_teardown`, you don't need to write any
unsubscribe logic** — pushing into `sub_ids_` during `on_setup` is
enough.  Override only when you have additional cleanup (rare):

```cpp
void MyModule::on_teardown() override {
    // ... extra cleanup (close files, drain queues, ...)
    Module::on_teardown();   // <- ALWAYS call.  Never skip.
}
```

The `if (bus_ == nullptr) return;` at the top of the base impl exists
because `on_teardown` is also called during destruction of the trial
module the validation pass constructs against a throwaway bus, plus
defensively in case a module is destroyed before `on_setup` ran (e.g.
a registry-level construction failure).  Read §6 for the trap that
guard caused before the bus shadowing fix.

---

## 5. snapshot_state / restore_state contract

Default impls return empty / no-op.  Override when the module has
working state that affects future behaviour.

Every snapshot has a `"version"` field; future schema migrations reject
incompatible snapshots cleanly:

```cpp
nlohmann::json MyModule::snapshot_state() const override {
    return nlohmann::json{
        {"version", 1},
        {"my_field", my_field_},
        // ...
    };
}

void MyModule::restore_state(nlohmann::json const& s) override {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error(
            "MyModule::restore_state: unknown version " + std::to_string(version));
    }
    my_field_ = s.value("my_field", my_field_);
}
```

Per-module gate state (`allowed_producers_`, `input_default_deny_`) is
captured at the **OgmaInstance level** in a sibling `gates` dict — the
per-module `snapshot_state` does not need to round-trip it.  See
`_aux_send_routing.md` § 3.5.

What goes in:

- Member fields, accumulated maps / graphs, pending event counters.
- RNG state (round-trip via `std::ostringstream << rng` →
  `std::istringstream >> rng`).
- Cached references to recent bus messages.

What stays out:

- Static parameters (those in `params_schema()`).  They live in
  `GraphConfig` and are restored at construction.
- Intra-tick caches cleared at `tick()`'s end (e.g. `pending_brain_`
  on MotorFader).  Document the omission in a comment.
- The aforementioned per-module gate state.

Coverage list: `docs/v4_ui_dev_module_state_audit.md`.  Determinism
contract: `cpp_core/tests/ogma/test_clone.cpp` and friends; every
shipping module type either implements `snapshot_state` or is asserted
stateless.

---

## 6. Post-mortem — Premotor / ActionGate shadowed `Module::bus_`

**Symptom.** A user opens patch mode in `the_cell_premotor.json`,
clicks Premotor, presses Delete.  Godot exits silently a moment later
— no error in the editor's debug pane, no GDScript runtime error.
Running from a terminal under gdb captures `SIGSEGV` inside
`std::__detail::_Hashtable_base<...>::_S_equals`.  The hashtable type
is exactly `std::unordered_map<std::string, std::unordered_map<int, float>>`
— the type of `Premotor::state_visit_ema_`.

**Backtrace, abridged.**

```
#6  ogma::Premotor::handle_consensus(...)
#11 std::function<...>::operator()(...)
#12 ogma::InProcessBus::publish(...)
#13 ogma::LateralVoter::tick(...)
#15 ogma::OgmaInstance::tick()
#16 godot::OgmaBrain::tick(double)
```

A `LateralVoter::tick` publishes a `ConsensusToken` on `consensus.0`;
the bus dispatches; the dispatch invokes a lambda whose captured
`this` points at a Premotor that was destroyed three lines earlier in
`apply_remove`.  The lambda calls `state_visit_ema_[topic][wid]` and
walks bucket node pointers in freed memory.

**Why the lambda was still in the bus.**  Both `Premotor.hpp` and
`ActionGate.hpp` had:

```cpp
private:
    Bus*  bus_ = nullptr;     // <-- shadows Module::bus_
```

The derived declaration **shadows** the inherited `Module::bus_` —
a subtle C++ trap that compiles cleanly with no warnings.

- `on_setup` (defined on the subclass) wrote the **derived** `bus_`.
- `Module::on_teardown` (not overridden) read the **base** `bus_`,
  which never moved off its default `nullptr`.
- The early-return `if (bus_ == nullptr) return;` at the top of the
  base impl fired unconditionally for these two modules → never
  unsubscribed anything → bus retained the stale lambdas → first
  matching publish post-destruction segfaulted.

**Fix.**  Remove the shadowing declarations.  Subclasses use the
inherited `Module::bus_` directly.  Both modules' `on_setup` already
just did `bus_ = bus;` — that line now writes the base member, and
`on_teardown` sees it non-null and unsubscribes properly.  Comments
left in both headers warn future authors not to redeclare bus_.

**Why every existing test passed before the fix.**  `test_clone`,
`test_clone_shipping_configs`, and the disk round-trip all exercise
`snapshot_state` / `restore_state` paths — they don't call
`apply_remove` and so never invoke `on_teardown`.  The patch-mode
harness DID call `apply_remove` but used a synthetic input set that
didn't drive any topic the EPMs in the cell config subscribed to — so
the voter never published `consensus.0`, the stale Premotor sub never
fired, and the test passed despite the bug.  Updated the harness to
publish the full cell input set; now `FullGraphPremotorRemove`
exercises the crashing path and would catch a regression.

**Lessons.**

- **Never redeclare an inherited member.**  C++ permits it, GCC
  doesn't warn, the runtime behaviour is silent corruption.  When
  you need access to a base-class field, use the inherited one or
  expose a setter on the base.
- **Test `apply_remove` with realistic publish traffic.**  A test
  that constructs and destroys a module without driving the topics
  its handlers subscribe to misses every UAF that lives in stale
  subscription closures.
- **gdb backtraces are worth more than speculation.**  This was
  diagnosed in <10 min once the user provided the trace; it was
  unsolvable without it.

---

## 7. Adding a new module — full checklist

1. **Class skeleton.**
   - Derive `public ogma::Module`.
   - Declare `type_name`, `input_topics`, `output_topics`, `params_schema`,
     `on_setup`, `tick` overrides.
   - **Do NOT redeclare `bus_`, `id_`, `sub_ids_`, `allowed_producers_`,
     or `input_default_deny_`.**  Use the inherited members.
2. **Register in `cpp_core/src/ogma/ModuleRegistry.cpp`** with the
   `OGMA_REGISTER_MODULE` pattern.
3. **`on_setup`**:
   - `bus_ = bus;` first (writes the inherited member).
   - Validate params via the `apply_param` / `get_*` helpers used by
     existing modules.
   - Push every `bus_->subscribe(...)` return into `sub_ids_`.
4. **Every published `Message` must stamp `producer_id = id_`.**
   Without this, manual-mode (default-deny) routing silently drops
   the producer.  See `_aux_send_routing.md` § 5.1.
5. **Every input handler starts with the gate one-liner**:
   ```cpp
   if (!input_allowed(payload->producer_id)) return;
   ```
   No-op in auto mode, ablation gate in manual mode.
6. **Snapshot / restore** (only if the module carries working state):
   - Implement `snapshot_state` / `restore_state` with the version-tagged
     idiom from §5.
   - Add an entry to `docs/v4_ui_dev_module_state_audit.md` so the
     coverage list stays a source of truth.
   - Drop the module from `kModulesMissingSnapshot` in
     `cpp_core/tests/ogma/test_clone_shipping_configs.cpp` so the
     determinism harness exercises it.
7. **OgmaBrain metrics** (only if the inspector / HUD wants per-module
   metrics): add a `dynamic_cast` branch in
   `godot_host/src/OgmaBrain.cpp::get_module_metrics`.
8. **Inspector dashboard** (only if a useful dashboard exists for this
   module type): add a row to `tools/v4_inspector/widgets/__init__.py
   ::WIDGET_REGISTRY` mapping the type name to a widget class.  See
   `tools/v4_inspector/README.md`.

---

## 8. Verification

When making any change to `Module` base or to a subclass's lifecycle
methods, run the C++ test suite:

```bash
cd /home/xaqmusic/ami-ogma-ui/cpp_core
cmake --build build && ctest --output-on-failure
```

Five suites must stay green:

| Suite | Coverage |
|---|---|
| `test_clone` | Determinism contract on a hand-built MC-like graph. |
| `test_clone_shipping_configs` | The same contract on every shipping JSON config that fits the harness's input synthesizer. |
| `test_snapshot_disk_round_trip` | Snapshot → JSON file → restore round-trip. |
| `test_manual_routing` | Per-primitive aux-send filter contract. |
| `test_patch_mode_harness` | Human-paced patch-mode workflows: sequential remove, blank-canvas build, mode toggle, edge ablation, snapshot interleaving, inspector race proxy, remove + readd, full-graph Premotor remove. |

Total: 33 PASS / 6 SKIP / 0 FAIL.  The 6 SKIPs in shipping_configs are
visual-encoder configs awaiting a `RawImageFrame` / `RawAudioFrame`
synthesizer in the harness; they are not regressions.

For Godot-side changes, headless boot a scene to surface GDScript
parse errors:

```bash
cd godot_host/project
timeout 6 godot4 --headless res://scenes/the_cell.tscn 2>&1 | grep -iE "error|parse|fail" | head
```

Empty output = clean.

---

## 9. Related code

- `cpp_core/include/ogma/Module.hpp` — the base class.
- `cpp_core/src/ogma/Module.cpp`     — base default impls.
- `cpp_core/src/ogma/Scheduler.cpp::apply_add / apply_remove` — the
  paths that call `on_setup` / `on_teardown` and route gate updates.
- `cpp_core/src/ogma/OgmaInstance.cpp::snapshot_state / restore_state` —
  the instance-level wrapper that round-trips per-module gate state.
- `cpp_core/tests/ogma/test_patch_mode_harness.cpp` — the regression
  gate for module lifecycle correctness.
