// =============================================================================
// Scheduler.cpp  --  Phase 3 minimal Scheduler implementation
// =============================================================================
//
// Single-threaded sequential ticker.  Modules execute in their registration
// order — which mirrors the DAG order produced by Phase 0's intended
// topological-sort levelization for the tests we exercise here, since
// graph configs are hand-built in registration order anyway.  Real DAG
// build + parallel level execution is a subsequent Phase 3 expansion.
//
// Hot-patch queue: enqueue_hot_patch() appends.  Applied at the start of
// the next tick (before bus.begin_tick) so module topology is stable
// throughout each tick — the contract from docs/primitives/_hot_patch.md.

#include "ogma/Scheduler.hpp"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <stdexcept>
#include <unordered_set>
#include <variant>

#include "ogma/GraphConfig.hpp"
#include "ogma/InProcessBus.hpp"

namespace ogma {

namespace {

// Resolve an EdgeSpec.from string into a producer id for matching against
// Message::producer_id in the per-primitive input gate.  `host:*` boundary
// edges all map to "host" since the Godot host stamps producer_id="host"
// on its bridged publishes.  Module ids pass through unchanged.
std::string resolve_producer_id(std::string const& edge_from) {
    constexpr std::string_view kHost = "host:";
    if (edge_from.size() >= kHost.size()
        && edge_from.compare(0, kHost.size(), kHost) == 0)
        return "host";
    return edge_from;
}

class MinimalScheduler : public Scheduler {
public:
    using ModuleList = std::vector<ModulePtr>;

    MinimalScheduler(Bus* bus, ModuleList* modules,
                     std::vector<EdgeSpec> boot_edges, bool auto_subscribe)
        : bus_(bus), modules_(modules),
          edges_(std::move(boot_edges)),
          auto_subscribe_(auto_subscribe) {
        // Apply initial gate state.  When manual mode is requested
        // (auto_subscribe=false), every module starts default-deny with
        // an allowlist seeded from boot_edges.
        if (!auto_subscribe_) install_manual_gates();
    }

    void tick() override {
        process_pending_patches();

        bus_->begin_tick(current_tick_);
        for (auto& m : *modules_) m->tick(current_tick_);
        bus_->end_level();
        bus_->end_tick();
        ++current_tick_;
    }

    uint64_t current_tick() const override { return current_tick_; }

    BatchId enqueue_hot_patch(GraphPatchBatch batch) override {
        std::lock_guard<std::mutex> lock(patch_mutex_);
        BatchId id = ++next_batch_id_;
        pending_.emplace_back(id, std::move(batch));
        return id;
    }

    std::vector<std::vector<std::string>> levels() const override {
        // MVP: every module sits at level 0 (no DAG yet).
        std::vector<std::string> only_level;
        for (auto const& m : *modules_) only_level.emplace_back(m->id());
        return { std::move(only_level) };
    }

    std::vector<EdgeSpec> feedback_edges() const override {
        std::vector<EdgeSpec> out;
        for (auto const& e : edges_) if (e.feedback) out.push_back(e);
        return out;
    }

    std::vector<EdgeSpec> edges() const override {
        return edges_;
    }

    // -------------------------------------------------------------------------
    // Per-graph routing-mode toggle (UI-dev manual-routing feature)
    // -------------------------------------------------------------------------

    void set_auto_subscribe(bool enabled) override {
        if (enabled == auto_subscribe_) return;
        auto_subscribe_ = enabled;
        if (enabled) {
            // Toggling back to auto-mode: turn off default-deny on every
            // live module.  Allowlists are left in place (cheap) so a
            // subsequent toggle-back-to-manual restores prior wiring
            // without rebuilding from edges_.
            for (auto& m : *modules_) m->set_input_default_deny(false);
        } else {
            install_manual_gates();
        }
    }

    bool is_auto_subscribe() const override { return auto_subscribe_; }

private:
    void process_pending_patches() {
        std::vector<std::pair<BatchId, GraphPatchBatch>> to_apply;
        {
            std::lock_guard<std::mutex> lock(patch_mutex_);
            to_apply.swap(pending_);
        }
        for (auto& [batch_id, batch] : to_apply) apply_batch(batch_id, batch);
    }

    // Validate the entire batch first (to honor atomicity), then apply.
    // Throws on failure — the caller (test, agent, etc.) treats the throw
    // as "batch rejected".  Future Phase 3 enhancement: report failure via a
    // status callback rather than exception so concurrent submitters don't
    // crash the whole instance.
    void apply_batch(BatchId batch_id, GraphPatchBatch const& batch) {
        // Pass 1 — validation, tracking intra-batch effects against a
        // working set of live ids so that [Remove "foo", Add "foo"] (the
        // shape produced by the topology-load UI) validates correctly.
        // We also trial-construct every AddNodeOp against a throwaway Bus
        // so an on_setup throw (e.g. missing required params) is surfaced
        // here rather than partway through pass-2 — at which point earlier
        // remove ops would have already mutated the live graph.
        std::unordered_set<std::string> live_ids;
        for (auto const& m : *modules_) live_ids.insert(std::string(m->id()));
        for (auto const& op : batch.ops) {
            std::visit([&](auto const& concrete) {
                using T = std::decay_t<decltype(concrete)>;
                if constexpr (std::is_same_v<T, AddNodeOp>) {
                    validate_add_against(concrete, live_ids);
                    live_ids.insert(concrete.spec.id);
                } else if constexpr (std::is_same_v<T, RemoveNodeOp>) {
                    validate_remove_against(concrete, live_ids);
                    live_ids.erase(concrete.id);
                } else if constexpr (std::is_same_v<T, SetParamOp>) {
                    validate_set_param(concrete);
                } else if constexpr (std::is_same_v<T, ConnectOp>) {
                    validate_connect_against(concrete, live_ids);
                } else if constexpr (std::is_same_v<T, DisconnectOp>) {
                    validate_disconnect_against(concrete, live_ids);
                }
            }, op);
        }

        // Pass 2 — apply.  After validation passes, individual op failures
        // here would indicate a programmer error, so we don't roll back.
        for (auto const& op : batch.ops) {
            std::visit([&](auto const& concrete) {
                using T = std::decay_t<decltype(concrete)>;
                if constexpr (std::is_same_v<T, AddNodeOp>) {
                    apply_add(concrete);
                } else if constexpr (std::is_same_v<T, RemoveNodeOp>) {
                    apply_remove(concrete);
                } else if constexpr (std::is_same_v<T, SetParamOp>) {
                    apply_set_param(concrete);
                } else if constexpr (std::is_same_v<T, ConnectOp>) {
                    apply_connect(concrete);
                } else if constexpr (std::is_same_v<T, DisconnectOp>) {
                    apply_disconnect(concrete);
                }
            }, op);
        }
        (void)batch_id;
    }

    // ---- ConnectOp / DisconnectOp ----
    // Phase 3 MVP: these update the scheduler's edge list.  Full runtime
    // Bus subscription re-binding (teaching modules to dynamically subscribe
    // to new topics) requires either a module-level `reconnect()` hook or
    // a topic-delegation mechanism in the Bus itself, both deferred to the
    // ZmqBus work.  For now we record the topology change so that:
    //   a) ConnectOp/DisconnectOp are valid schedulable ops (not rejected).
    //   b) The edge list is accurate for any tooling that inspects it
    //      (e.g. visual graph editor, graph export).
    //   c) When a module is added (AddNodeOp), its on_setup() wires its own
    //      subscriptions from its params, so the topology implied by the
    //      edges is realised at that point.

    void validate_connect_against(ConnectOp const& op,
                                   std::unordered_set<std::string> const& live_ids) const {
        auto resolve = [&](std::string const& endpoint) {
            constexpr std::string_view kHost = "host:";
            if (endpoint.compare(0, kHost.size(), kHost) == 0) return;  // host endpoint OK
            if (live_ids.count(endpoint) == 0)
                throw std::invalid_argument("hot_patch ConnectOp: endpoint '" + endpoint + "' not found");
        };
        if (op.edge.from.empty())
            throw std::invalid_argument("hot_patch ConnectOp: empty 'from'");
        if (op.edge.to.empty())
            throw std::invalid_argument("hot_patch ConnectOp: empty 'to'");
        resolve(op.edge.from);
        resolve(op.edge.to);
    }

    void validate_disconnect_against(DisconnectOp const& op,
                                      std::unordered_set<std::string> const& live_ids) const {
        // Only check that the endpoints exist; not finding the edge is benign.
        auto resolve = [&](std::string const& e) {
            constexpr std::string_view kHost = "host:";
            if (e.compare(0, kHost.size(), kHost) == 0) return;
            if (live_ids.count(e) == 0)
                throw std::invalid_argument("hot_patch DisconnectOp: endpoint '" + e + "' not found");
        };
        resolve(op.from);
        resolve(op.to);
    }

    void apply_connect(ConnectOp const& op) {
        // Record edge; Bus subscription wiring is the responsibility of the
        // downstream module (happens inside its on_setup / reconnect path).
        edges_.push_back(op.edge);
        // Per-primitive aux-send: when manual routing is on, allow this
        // producer through the receiver module's input gate.  No-op in
        // auto mode (all gates are wide open) but the bookkeeping costs
        // nothing and keeps allowlists ready if the user toggles back
        // into manual without rebuilding from edges_.
        Module* recv = find_mutable_module(op.edge.to);
        if (recv != nullptr) {
            recv->on_producer_allowed(resolve_producer_id(op.edge.from));
        }
    }

    void apply_disconnect(DisconnectOp const& op) {
        edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
            [&](EdgeSpec const& e) {
                return e.from == op.from && e.to == op.to &&
                       (op.topic.empty() || e.topic == op.topic);
            }), edges_.end());
        // After removal, only revoke the producer's allow entry on the
        // receiver if NO surviving edge from the same producer remains
        // — a single (from, to) pair may carry multiple distinct topics.
        Module* recv = find_mutable_module(op.to);
        if (recv == nullptr) return;
        std::string producer = resolve_producer_id(op.from);
        bool still_allowed = false;
        for (auto const& e : edges_) {
            if (e.to == op.to && resolve_producer_id(e.from) == producer) {
                still_allowed = true;
                break;
            }
        }
        if (!still_allowed) recv->on_producer_denied(producer);
    }

    // Walk the live module list and, for each module, set default-deny +
    // populate its allowlist from the producers of every edge whose `to`
    // matches.  Called at construction in manual mode and on
    // set_auto_subscribe(false).
    void install_manual_gates() {
        for (auto& m : *modules_) {
            m->clear_allowed_producers();
            m->set_input_default_deny(true);
        }
        for (auto const& e : edges_) {
            Module* recv = find_mutable_module(e.to);
            if (recv == nullptr) continue;
            recv->on_producer_allowed(resolve_producer_id(e.from));
        }
    }

    void validate_add_against(AddNodeOp const& op,
                               std::unordered_set<std::string> const& live_ids) const {
        if (op.spec.id.empty())
            throw std::invalid_argument("hot_patch: AddNodeOp.spec.id empty");
        if (live_ids.count(op.spec.id) > 0)
            throw std::invalid_argument("hot_patch: id '" + op.spec.id + "' already exists");
        // type known to registry
        auto known = ModuleRegistry::instance().registered_types();
        if (std::find(known.begin(), known.end(), op.spec.type) == known.end())
            throw std::invalid_argument("hot_patch: unknown type '" + op.spec.type + "'");
        // Trial-construct against a throwaway Bus so on_setup throws (e.g.
        // missing required params, malformed values) surface here BEFORE
        // pass-2 starts mutating the live graph.  Without this, a failing
        // add midway through a load batch leaves the brain with the
        // earlier remove ops applied but the new modules never created.
        try {
            ModulePtr trial = ModuleRegistry::instance().create(op.spec.type);
            trial->set_id(op.spec.id);
            InProcessBus dummy_bus;
            trial->on_setup(&dummy_bus, op.spec.params);
            // trial's destructor + base on_teardown unsubscribes from
            // dummy_bus, which is also about to be destroyed — both safe.
        } catch (std::exception const& e) {
            throw std::invalid_argument(
                "hot_patch: AddNodeOp '" + op.spec.id + "' (type "
                + op.spec.type + ") failed trial setup: " + e.what());
        }
    }

    void validate_remove_against(RemoveNodeOp const& op,
                                  std::unordered_set<std::string> const& live_ids) const {
        if (live_ids.count(op.id) == 0)
            throw std::invalid_argument("hot_patch: RemoveNodeOp.id '" + op.id + "' not found");
    }

    void validate_set_param(SetParamOp const& op) const {
        Module const* m = find_module(op.target_id);
        if (m == nullptr)
            throw std::invalid_argument("hot_patch: SetParamOp.target_id '" + op.target_id + "' not found");
        // Check the param key is in the schema and is HotMutable.
        auto schema = m->params_schema();
        auto it = std::find_if(schema.begin(), schema.end(),
            [&](ParamSpec const& p) { return p.key == op.key; });
        if (it == schema.end())
            throw std::invalid_argument("hot_patch: '" + op.target_id + "' has no param '" + op.key + "'");
        if (it->mutability != ParamMutability::HotMutable)
            throw std::invalid_argument("hot_patch: '" + op.key + "' is ConstructionOnly");
    }

    void apply_add(AddNodeOp const& op) {
        ModulePtr m = ModuleRegistry::instance().create(op.spec.type);
        m->set_id(op.spec.id);
        m->on_setup(bus_, op.spec.params);
        // Newly-added modules must inherit the live routing mode.  Without
        // this, a module added while auto_subscribe is false starts with
        // input_default_deny_=false (the Module base default), defeating
        // the manual-routing contract — the new module would receive every
        // matching topic regardless of explicit edges.  Caught by the
        // patch-mode harness's RepeatedRemoveReaddSameId scenario.
        if (!auto_subscribe_) {
            m->set_input_default_deny(true);
            // Allowlist remains empty until subsequent ConnectOp edges
            // populate it.  No boot edges seed it because boot edges are
            // already in edges_ and would have been associated with the
            // previously-removed module of the same id (see apply_remove
            // which revokes those entries).
        }
        modules_->push_back(std::move(m));
    }

    void apply_remove(RemoveNodeOp const& op) {
        auto it = std::find_if(modules_->begin(), modules_->end(),
            [&](ModulePtr const& m) { return m->id() == op.id; });
        if (it == modules_->end()) return;

        // Pre-remove cleanup: any edge whose `to` is the removed module
        // becomes meaningless (the receiver is gone); any edge whose
        // `from` is the removed module would leak a stale allow entry
        // on its receivers.  Drop both sets and revoke the producer
        // from receivers that had it allowed — keeps gate state coherent
        // through the remove without waiting for an explicit DisconnectOp.
        std::vector<EdgeSpec> kept;
        kept.reserve(edges_.size());
        for (auto const& e : edges_) {
            bool from_removed = (resolve_producer_id(e.from) == op.id);
            bool to_removed   = (e.to == op.id);
            if (from_removed) {
                // Inform any surviving receiver of this edge that the
                // producer is gone.  Without this, the receiver's
                // allowlist keeps a dangling entry that would silently
                // allow a future module reusing the same id to receive
                // through this stale edge.
                if (Module* recv = find_mutable_module(e.to)) {
                    // Only revoke when no OTHER edge from the same
                    // producer survives — match the apply_disconnect
                    // discipline so we don't drop an entry the
                    // receiver still needs from a parallel topic.
                    bool other_edge = false;
                    for (auto const& f : edges_) {
                        if (&f == &e) continue;
                        if (resolve_producer_id(f.from) != op.id) continue;
                        if (f.to != e.to) continue;
                        other_edge = true; break;
                    }
                    if (!other_edge) recv->on_producer_denied(op.id);
                }
            }
            if (!from_removed && !to_removed) kept.push_back(e);
        }
        edges_.swap(kept);

        (*it)->on_teardown();
        modules_->erase(it);
    }

    void apply_set_param(SetParamOp const& op) {
        Module* m = find_mutable_module(op.target_id);
        if (!m) return;
        m->on_param_change(op.key, op.value);
    }

    Module const* find_module(std::string const& id) const {
        for (auto const& m : *modules_)
            if (std::string(m->id()) == id) return m.get();
        return nullptr;
    }
    Module* find_mutable_module(std::string const& id) {
        for (auto& m : *modules_)
            if (std::string(m->id()) == id) return m.get();
        return nullptr;
    }

    Bus*                                                bus_;
    ModuleList*                                         modules_;
    std::vector<EdgeSpec>                               edges_;
    bool                                                auto_subscribe_   = true;
    uint64_t                                            current_tick_     = 0;
    std::mutex                                          patch_mutex_;
    std::vector<std::pair<BatchId, GraphPatchBatch>>    pending_;
    std::atomic<BatchId>                                next_batch_id_    {0};
};

} // namespace

namespace detail {

std::unique_ptr<Scheduler> make_minimal_scheduler(
    Bus* bus, std::vector<ModulePtr>* modules,
    std::vector<EdgeSpec> boot_edges, bool auto_subscribe) {
    return std::make_unique<MinimalScheduler>(
        bus, modules, std::move(boot_edges), auto_subscribe);
}

} // namespace detail

} // namespace ogma
