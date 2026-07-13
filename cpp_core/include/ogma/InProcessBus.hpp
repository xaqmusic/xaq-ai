#pragma once

// =============================================================================
// InProcessBus.hpp  --  Default Bus implementation for production hosts
// =============================================================================
//
// Direct topic-keyed dispatch on the calling thread.  Per Pillar 1 of
// docs/v4_refactor.md, this is the default Bus for HAL Host (Pi5) and Godot
// Host — zero IPC, function-call cost.
//
// Semantics (per docs/primitives/_first_tick.md and Bus.hpp):
//
//   - publish(topic, payload):
//       * stores payload as the topic's "current-tick" last value
//       * synchronously dispatches to every Direct subscription whose pattern
//         matches `topic`, in subscription-ID order (deterministic)
//   - subscribe(pattern, kind, handler):
//       * Direct: handler fires synchronously on each matching publish
//       * Feedback: handler fires once per global tick at begin_tick(), with
//         the previous-tick value(s) for matching topics
//   - last_value(topic): O(1) lookup of the current-tick payload, or nullptr
//   - begin_tick(t): rotates current-tick → previous-tick caches; fires every
//     Feedback subscription's handler with the prior-tick payload(s) it covers
//   - end_tick(): no-op (the rotation happens at begin_tick of tick t+1)
//   - end_level(): no-op for InProcess (all dispatch is synchronous)
//
// Pattern matching: trailing-dot prefix-match only.  "reality." matches any
// topic starting with "reality.".  Otherwise exact match.
//
// Thread safety: NOT thread-safe by itself.  Each OgmaInstance owns its own
// InProcessBus; the Scheduler serializes module-level publishes via the level
// barrier.  Modules within a level can race only on their own state, which is
// the v4 contract — no cross-module state outside the Bus, no shared mutable
// data through the Bus.

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "ogma/Bus.hpp"

namespace ogma {

class InProcessBus : public Bus {
public:
    InProcessBus();
    ~InProcessBus() override;

    // Bus interface
    void           publish(std::string_view topic, MessagePtr payload) override;
    SubscriptionId subscribe(std::string_view  pattern,
                             SubscriptionKind  kind,
                             Handler           handler) override;
    void           unsubscribe(SubscriptionId id) override;
    MessagePtr     last_value(std::string_view topic) const override;

    void           begin_tick(uint64_t tick_id) override;
    void           end_level() override;
    void           end_tick() override;

    std::vector<std::string> subscribed_topics() const override;

    // -------------------------------------------------------------------------
    // Phase 6.5.4 — clone support
    // -------------------------------------------------------------------------
    //
    // The last-value cache holds shared_ptr<const Message> entries.  Messages
    // are immutable so sharing pointers between buses is safe.  These two
    // methods let OgmaInstance::clone() shallow-copy the cache from a source
    // bus into a freshly-constructed one, so the clone's modules see the
    // same most-recent-value answers via last_value() that the source did.
    //
    // Subscriptions are NOT copied — the cloned modules re-subscribe via
    // their own on_setup() when the new OgmaInstance is constructed.
    using TopicCache = std::unordered_map<std::string, MessagePtr>;
    TopicCache snapshot_topic_cache() const { return current_values_; }
    void       restore_topic_cache(TopicCache cache) {
        current_values_  = std::move(cache);
        previous_values_ = current_values_;
    }
    uint64_t   current_tick_id() const { return tick_id_; }
    void       set_tick_id(uint64_t t) { tick_id_ = t; }

private:
    struct Subscription {
        std::string         pattern;
        bool                is_prefix;   // pattern ends with '.'
        SubscriptionKind    kind;
        Handler             handler;
    };

    static bool pattern_matches(std::string_view pattern,
                                bool             is_prefix,
                                std::string_view topic);

    // Subscriptions are keyed by ID and iterated in ID-sort order so dispatch
    // is deterministic across runs.
    SubscriptionId                          next_id_   = 0;
    std::map<SubscriptionId, Subscription>  subs_;

    // Last-known-value cache: topic name -> most recent payload published.
    std::unordered_map<std::string, MessagePtr> current_values_;
    std::unordered_map<std::string, MessagePtr> previous_values_;

    uint64_t tick_id_ = 0;
};

} // namespace ogma
