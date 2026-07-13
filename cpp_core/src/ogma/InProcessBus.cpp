#include "ogma/InProcessBus.hpp"

#include <algorithm>
#include <utility>

namespace ogma {

InProcessBus::InProcessBus()  = default;
InProcessBus::~InProcessBus() = default;

bool InProcessBus::pattern_matches(std::string_view pattern,
                                   bool             is_prefix,
                                   std::string_view topic) {
    if (is_prefix) {
        return topic.size() >= pattern.size() &&
               topic.compare(0, pattern.size(), pattern) == 0;
    }
    return pattern == topic;
}

void InProcessBus::publish(std::string_view topic, MessagePtr payload) {
    // Store as current-tick last value.
    current_values_[std::string(topic)] = payload;

    // Synchronously dispatch to every matching Direct subscription.
    // Iteration order is sorted by SubscriptionId, which is the
    // monotonically-increasing ID assigned at subscribe() time, so this is
    // registration-order deterministic.
    for (auto const& [id, sub] : subs_) {
        if (sub.kind != SubscriptionKind::Direct) continue;
        if (pattern_matches(sub.pattern, sub.is_prefix, topic)) {
            sub.handler(topic, payload);
        }
    }
}

SubscriptionId InProcessBus::subscribe(std::string_view  pattern,
                                       SubscriptionKind  kind,
                                       Handler           handler) {
    Subscription sub;
    sub.pattern   = std::string(pattern);
    sub.is_prefix = !sub.pattern.empty() && sub.pattern.back() == '.';
    sub.kind      = kind;
    sub.handler   = std::move(handler);

    SubscriptionId id = ++next_id_;
    subs_.emplace(id, std::move(sub));
    return id;
}

void InProcessBus::unsubscribe(SubscriptionId id) {
    subs_.erase(id);
}

MessagePtr InProcessBus::last_value(std::string_view topic) const {
    auto it = current_values_.find(std::string(topic));
    if (it == current_values_.end()) return nullptr;
    return it->second;
}

void InProcessBus::begin_tick(uint64_t tick_id) {
    tick_id_ = tick_id;

    // Rotate current → previous so Feedback subscribers see t-1 values
    // throughout this tick, even after current-tick publishes update
    // current_values_.
    previous_values_ = current_values_;

    // Fire every Feedback subscription's handler with the matching
    // previous-tick payloads.  This is the cycle-break: a consumer that reads
    // EPM(t-1) sees it before its own tick() runs in the current level.
    for (auto const& [id, sub] : subs_) {
        if (sub.kind != SubscriptionKind::Feedback) continue;
        for (auto const& [topic, payload] : previous_values_) {
            if (!payload) continue;
            if (pattern_matches(sub.pattern, sub.is_prefix, topic)) {
                sub.handler(topic, payload);
            }
        }
    }
}

void InProcessBus::end_level() {
    // No-op for InProcessBus.  ZmqBus and HybridBus would use this to flush
    // wire frames between DAG levels.
}

void InProcessBus::end_tick() {
    // No-op.  Rotation happens at begin_tick(t+1) so Feedback subs see a
    // stable t-1 view through tick t.
}

std::vector<std::string> InProcessBus::subscribed_topics() const {
    std::vector<std::string> out;
    out.reserve(subs_.size());
    for (auto const& [id, sub] : subs_) {
        out.push_back(sub.pattern);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

} // namespace ogma
