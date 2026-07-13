// =============================================================================
// OgmaInstance.cpp  --  Phase 3 minimal implementation
// =============================================================================
//
// One Bus, one Scheduler, a vector of modules.  Constructed from a
// GraphConfig; modules are instantiated via the global ModuleRegistry,
// each receives set_id() + on_setup() before the first tick().

#include "ogma/OgmaInstance.hpp"

#include <stdexcept>

#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"

namespace ogma {

namespace detail {
std::unique_ptr<Scheduler> make_minimal_scheduler(
    Bus* bus, std::vector<ModulePtr>* modules,
    std::vector<EdgeSpec> boot_edges, bool auto_subscribe);
}

struct OgmaInstance::Impl {
    GraphConfig                                config;
    std::unique_ptr<Bus>                       bus;
    std::vector<ModulePtr>                     modules;
    std::unique_ptr<Scheduler>                 scheduler;
    std::string                                label;
};

OgmaInstance::OgmaInstance(GraphConfig config, std::unique_ptr<Bus> bus)
    : impl_(std::make_unique<Impl>()) {
    if (!bus) throw std::invalid_argument("OgmaInstance requires a non-null Bus");

    impl_->config = std::move(config);
    impl_->bus    = std::move(bus);

    // Instantiate every module declared in the config.
    auto& reg = ModuleRegistry::instance();
    for (auto const& spec : impl_->config.modules) {
        ModulePtr m = reg.create(spec.type);
        m->set_id(spec.id);
        m->on_setup(impl_->bus.get(), spec.params);
        impl_->modules.push_back(std::move(m));
    }

    impl_->scheduler = detail::make_minimal_scheduler(
        impl_->bus.get(), &impl_->modules,
        impl_->config.edges,
        impl_->config.runtime.auto_subscribe);
}

OgmaInstance::~OgmaInstance() {
    // Modules teardown in reverse order so downstream modules unsub first.
    for (auto it = impl_->modules.rbegin(); it != impl_->modules.rend(); ++it)
        (*it)->on_teardown();
}

void OgmaInstance::tick() { impl_->scheduler->tick(); }

uint64_t OgmaInstance::tick_count() const { return impl_->scheduler->current_tick(); }

Bus*       OgmaInstance::bus()             { return impl_->bus.get(); }
Bus const* OgmaInstance::bus() const       { return impl_->bus.get(); }
Scheduler* OgmaInstance::scheduler()       { return impl_->scheduler.get(); }

Module* OgmaInstance::module(std::string_view id) {
    for (auto& m : impl_->modules)
        if (m->id() == id) return m.get();
    return nullptr;
}
Module const* OgmaInstance::module(std::string_view id) const {
    for (auto const& m : impl_->modules)
        if (m->id() == id) return m.get();
    return nullptr;
}

std::vector<Module*> OgmaInstance::modules() {
    std::vector<Module*> out;
    out.reserve(impl_->modules.size());
    for (auto& m : impl_->modules) out.push_back(m.get());
    return out;
}

GraphConfig const& OgmaInstance::config() const { return impl_->config; }

Scheduler::BatchId OgmaInstance::enqueue_hot_patch(GraphPatchBatch batch) {
    return impl_->scheduler->enqueue_hot_patch(std::move(batch));
}

std::vector<EdgeSpec> OgmaInstance::current_edges() const {
    return impl_->scheduler->edges();
}

void OgmaInstance::set_label(std::string l) { impl_->label = std::move(l); }
std::string const& OgmaInstance::label() const { return impl_->label; }

// ---------------------------------------------------------------------------
// Snapshot / clone (Phase 6.5.4)
// ---------------------------------------------------------------------------

nlohmann::json OgmaInstance::snapshot_state() const {
    nlohmann::json modules = nlohmann::json::object();
    nlohmann::json gates   = nlohmann::json::object();
    for (auto const& m : impl_->modules) {
        std::string const id(m->id());
        modules[id] = m->snapshot_state();
        // Per-primitive input gate (aux-send routing).  Sibling field at
        // the OgmaInstance level so per-module snapshot_state overrides
        // don't need to know about gate state — keeps the existing 9
        // HAS-snapshot modules unchanged.  Older snapshots without
        // `gates` restore with default-allow gates (back-compat).
        nlohmann::json allowed = nlohmann::json::array();
        for (auto const& p : m->allowed_producers()) allowed.push_back(p);
        gates[id] = {
            {"allowed",      std::move(allowed)},
            {"default_deny", m->is_input_default_deny()},
        };
    }
    return nlohmann::json{
        {"version", 1},
        {"label",   impl_->label},
        {"modules", modules},
        {"gates",   gates},
    };
}

void OgmaInstance::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("OgmaInstance::restore_state: unknown version " +
                                 std::to_string(version));
    }
    if (s.contains("label") && s["label"].is_string())
        impl_->label = s["label"].get<std::string>();
    if (s.contains("modules") && s["modules"].is_object()) {
        for (auto& m : impl_->modules) {
            std::string id(m->id());
            auto it = s["modules"].find(id);
            if (it != s["modules"].end() && !it->is_null()) {
                m->restore_state(*it);
            }
        }
    }
    // Restore per-module gate state.  Missing field = clear/disable
    // (older snapshots predate the gate feature, default-allow stays).
    if (s.contains("gates") && s["gates"].is_object()) {
        for (auto& m : impl_->modules) {
            std::string const id(m->id());
            auto it = s["gates"].find(id);
            if (it == s["gates"].end() || !it->is_object()) continue;
            m->clear_allowed_producers();
            m->set_input_default_deny(
                it->value("default_deny", false));
            if (it->contains("allowed") && (*it)["allowed"].is_array())
                for (auto const& p : (*it)["allowed"])
                    m->on_producer_allowed(p.get<std::string>());
        }
    } else {
        // No gates field → reset to default-allow so we don't carry
        // stale per-module allowlists into a pre-gate-era snapshot.
        for (auto& m : impl_->modules) {
            m->clear_allowed_producers();
            m->set_input_default_deny(false);
        }
    }
}

std::unique_ptr<OgmaInstance> OgmaInstance::clone() const {
    // Construct a fresh InProcessBus.  If the source uses a non-InProcess
    // bus type, we can extend this later — for now, the Godot host always
    // uses InProcessBus.
    auto* src_bus = dynamic_cast<InProcessBus const*>(impl_->bus.get());
    if (!src_bus) {
        throw std::runtime_error(
            "OgmaInstance::clone: source bus is not InProcessBus; clone unsupported.");
    }

    auto new_bus = std::make_unique<InProcessBus>();
    auto* new_bus_raw = new_bus.get();

    // Construct the new instance from the SAME config — this re-instantiates
    // every module and runs on_setup so subscriptions are live before any
    // tick.  Modules start with their own (default) state; we restore right
    // after.
    auto cloned = std::make_unique<OgmaInstance>(impl_->config, std::move(new_bus));

    // Shallow-copy the bus topic cache — Messages are immutable so sharing
    // pointers between source and clone is safe.  Consumers in the clone
    // will see the same last-known values until they're overwritten by
    // fresh publishes on the clone's own bus.
    new_bus_raw->restore_topic_cache(src_bus->snapshot_topic_cache());
    new_bus_raw->set_tick_id(src_bus->current_tick_id());

    // Apply per-module state snapshots.
    cloned->restore_state(snapshot_state());

    // Tag the clone for telemetry.
    cloned->set_label(impl_->label.empty() ? std::string("clone")
                                           : impl_->label + "_clone");
    return cloned;
}

// GraphConfig methods are implemented in GraphConfig.cpp.

} // namespace ogma
