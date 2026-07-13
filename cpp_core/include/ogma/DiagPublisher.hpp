#pragma once

// =============================================================================
// DiagPublisher.hpp  --  ZMQ PUB fan-out for live module diagnostics (W2)
// =============================================================================
//
// Companion to ControlServer (which handles JSON-RPC commands).  ControlServer
// is request/reply on a TCP socket — fine for low-rate verbs (list_modules,
// module_snapshot, subscribe_diag).  For high-rate streaming telemetry
// (catching temporal patterns at 30-60 Hz across many modules) the cost of
// JSON-RPC framing dominates; ZMQ PUB/SUB on a separate port amortises it.
//
// Lifecycle
// ---------
//
//   DiagPublisher pub(7301);
//   pub.start();
//   ...per tick:
//   pub.publish_tick(tick_id, instance);    // walks active subs, throttles by hz
//   ...on shutdown:
//   pub.stop();
//
// Subscription model
// ------------------
//
// Each subscription has an id, a module_id, an optional topic filter, and a
// target Hz.  publish_tick() decides per subscription whether to fire this
// tick (based on elapsed ticks since last publish vs target interval).  When
// it fires, it builds a JSON object:
//
//   {
//     "sub_id":    int,
//     "module_id": string,
//     "tick_id":   uint64,
//     "snapshot":  <module->snapshot_state()>,
//     "topic":     string (optional, the topic filter that was subscribed)
//   }
//
// and PUBs it on a ZMQ topic of the form `diag.<sub_id>.` so subscribers
// filter cleanly without sharing the broadcast firehose.
//
// Thread model
// ------------
//
// All methods are called from the host's tick thread (the same thread that
// drives OgmaInstance::tick).  No internal threads — keeps snapshot_state()
// access serial relative to the module that owns the state.  ControlServer's
// command handler thread enqueues subscribe / unsubscribe ops that are
// applied at the next publish_tick (mutex-guarded).

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ogma {

class OgmaInstance;

class DiagPublisher {
public:
    explicit DiagPublisher(uint16_t port);
    ~DiagPublisher();

    DiagPublisher(DiagPublisher const&) = delete;
    DiagPublisher& operator=(DiagPublisher const&) = delete;

    // Open the PUB socket.  Idempotent.
    bool start();

    // Close the PUB socket.  All subscriptions are dropped.
    void stop();

    // Add a subscription.  Returns the new subscription id.  hz <= 0 falls
    // back to the per-instance default (30 Hz).  topic is informational —
    // currently unused for state filtering (we always publish snapshot_state)
    // but echoed in payloads so subscribers can route on it.
    int subscribe(std::string module_id, std::string topic, double hz);

    // Remove a subscription.  No-op for unknown ids.
    void unsubscribe(int sub_id);

    // List active subscriptions for the inspector to render.
    struct Subscription {
        int          id        = 0;
        std::string  module_id;
        std::string  topic;
        double       hz        = 30.0;
        uint64_t     last_pub_tick = 0;
        bool         have_pubbed = false;
    };
    std::vector<Subscription> active_subscriptions() const;

    // Called by the host every tick after OgmaInstance::tick.  Iterates
    // active subscriptions, throttles each by its target Hz against the
    // host's nominal tick rate (default 60 Hz), serialises and PUBs.
    void publish_tick(uint64_t tick_id, OgmaInstance& instance);

    // Host tick rate in Hz (default 60).  Used to convert sub.hz into
    // an integer "every N ticks" interval.  Set once at startup.
    void set_host_tick_hz(double hz) { host_tick_hz_ = hz; }

    uint16_t port() const { return port_; }
    bool     running() const { return running_.load(); }

private:
    uint16_t                  port_;
    std::atomic<bool>         running_{false};
    void*                     ctx_       = nullptr;
    void*                     sock_      = nullptr;
    double                    host_tick_hz_ = 60.0;

    mutable std::mutex        subs_mtx_;
    std::unordered_map<int, Subscription> subs_;
    int                       next_id_   = 1;
};

} // namespace ogma
