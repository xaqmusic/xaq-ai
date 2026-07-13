#pragma once

// =============================================================================
// Bus.hpp  --  Abstract pub/sub interface for the cellular bath
// =============================================================================
//
// Every Ogma Core module communicates through this interface and only through
// this interface.  Direct neighbour pointers, shared globals, and static state
// are forbidden by Pillar 1's Critical Rules.  The Bus implementation is
// chosen by the host:
//
//   InProcessBus  -- direct topic-keyed dispatch on the Runtime thread pool.
//                    Default for HAL Host and Godot Host.  Function-call cost.
//   ZmqBus        -- PUB/SUB for data, REQ/REP for control, PUSH/PULL for
//                    registry handshake.  ~1-10 us localhost.  For distributed
//                    or external-debug deployments.
//   HybridBus     -- InProcess dispatch + ZMQ mirror for external observers.
//                    Production on robot when live telemetry is desired.
//
// A module compiled against `Bus*` links identically into all three.
//
// -----------------------------------------------------------------------------
// Topic-name semantics
// -----------------------------------------------------------------------------
//
// Topics are dot-separated hierarchical strings (see Topics.hpp).  Subscriptions
// match patterns:
//
//   Exact match     "neuro.state"     matches only "neuro.state".
//   Prefix match    "reality."        matches every topic starting with
//                                     "reality." (the trailing dot is the
//                                     prefix indicator and must be present).
//
// `subscribe("reality.video.")` catches `reality.video.retinal`,
// `reality.video.saliency`, etc., but not `reality.audio.stft`.
//
// There is no `*` wildcard, no glob, no regex.  Trailing-dot prefix matching is
// the only form supported.  This keeps dispatch cheap and topic intent obvious
// at the call site.
//
// -----------------------------------------------------------------------------
// Last-known-value cache and first-tick semantics
// -----------------------------------------------------------------------------
//
// On every successful `publish`, the Bus records the payload as the topic's
// "last value".  `last_value(topic)` returns the most recent payload (or
// nullptr if the topic has never been published to in this OgmaInstance).
//
// This addresses two contracts:
//
// 1. **First-tick reads.**  At tick 0, a downstream module's producer may not
//    have run yet.  The downstream module calls `last_value(topic)` and gets
//    nullptr; it MUST handle that explicitly (typically by using its
//    construction-time defaults).  See docs/primitives/_first_tick.md.
//
// 2. **Feedback reads.**  When a module subscribes via a feedback edge (see
//    SubscriptionKind::Feedback below), the handler is invoked with the
//    *previous* tick's payload — i.e. the value that was current at the start
//    of this tick, before the producer ran in the current level.  This is how
//    the Scheduler breaks DAG cycles (e.g. NeurochemState <-> EPM,
//    DescendingPredictor <-> EPM) without a chicken-and-egg deadlock.
//
// -----------------------------------------------------------------------------
// Payload sharing semantics
// -----------------------------------------------------------------------------
//
// Bus dispatches `MessagePtr` (== `std::shared_ptr<const Message>`).  Every
// subscriber receives the SAME pointer; payload mutation is forbidden by the
// const-qualification.  InProcessBus does not copy; ZmqBus serializes per
// recipient.
//
// Modules MUST NOT cache MessagePtrs across ticks unless they intend to use
// the value as historical context.  Caching imports liveness assumptions the
// Bus does not guarantee.
//
// -----------------------------------------------------------------------------

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "ogma/Topics.hpp"

namespace ogma {

// Opaque token returned by subscribe(); pass to unsubscribe().
using SubscriptionId = uint64_t;
constexpr SubscriptionId kInvalidSubscriptionId = 0;

enum class SubscriptionKind {
    // Handler is invoked synchronously inside the producer's tick(), with the
    // value just published (current tick).  This is the default.
    Direct,

    // Handler is invoked once per global tick, with the value the producer
    // published in the previous tick.  Used to break DAG cycles.  The Bus
    // delivers the previous-tick value during the consumer's level execution,
    // BEFORE the producer's current-tick publish (which the consumer ignores).
    Feedback,
};

class Bus {
public:
    virtual ~Bus() = default;

    // -------------------------------------------------------------------------
    // Producer side
    // -------------------------------------------------------------------------

    // Publish `payload` on `topic`.  The Bus dispatches synchronously to every
    // matching Direct subscription before returning; Feedback subscriptions
    // see this value on the NEXT tick.  `payload->tick_id` and `producer_id`
    // are expected to be set by the publisher.
    virtual void publish(std::string_view topic, MessagePtr payload) = 0;

    // -------------------------------------------------------------------------
    // Consumer side
    // -------------------------------------------------------------------------

    // Subscribe to a topic or a trailing-dot prefix.  The handler signature is
    // (topic_name, payload).  The topic_name argument is the actual published
    // topic, not the pattern, so prefix subscribers can route by suffix.
    using Handler = std::function<void(std::string_view topic, MessagePtr payload)>;

    virtual SubscriptionId subscribe(std::string_view  pattern,
                                     SubscriptionKind  kind,
                                     Handler           handler) = 0;

    virtual void unsubscribe(SubscriptionId id) = 0;

    // Last-known-value cache: returns the most recent payload published on
    // exactly `topic` (no wildcard expansion), or nullptr if the topic has
    // never been written to.  Cheap O(1) lookup.  See "first-tick semantics"
    // in the file header.
    virtual MessagePtr last_value(std::string_view topic) const = 0;

    // -------------------------------------------------------------------------
    // Tick lifecycle (driven by Scheduler)
    // -------------------------------------------------------------------------

    // Called by the Scheduler at the start of every global tick BEFORE any
    // module runs.  Rotates current-tick state into "previous-tick" so that
    // Feedback subscribers see consistent t-1 values throughout the tick.
    virtual void begin_tick(uint64_t tick_id) = 0;

    // Called by the Scheduler after every level barrier completes — i.e.
    // between DAG levels within a tick.  Implementations may use this to
    // flush any deferred dispatch (e.g. ZmqBus mirrors).
    virtual void end_level() = 0;

    // Called by the Scheduler at the end of every global tick AFTER all
    // levels have completed.  Implementations may use this for telemetry,
    // ZMQ wire flushes, or hot-patch op queue processing.
    virtual void end_tick() = 0;

    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------

    // Returns the list of topic names that currently have at least one Direct
    // or Feedback subscriber.  Used by Phase 2 trace assertions to verify
    // that every published topic is consumed by something.
    virtual std::vector<std::string> subscribed_topics() const = 0;
};

} // namespace ogma
