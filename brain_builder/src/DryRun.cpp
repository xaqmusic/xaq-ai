#include "DryRun.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <set>

#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/Topics.hpp"

namespace bb {

namespace {

struct Feed {
    std::string topic, payload;
    int dims = 4;
};

bool starts_with(std::string const& s, std::string const& p) { return s.rfind(p, 0) == 0; }

} // namespace

DryRunReport dry_run(Graph const& g, Wiring const& w, Body const* body, int ticks) {
    DryRunReport r;
    using clock = std::chrono::steady_clock;

    // What to feed: every body source or event a module consumes (exact or
    // through a prefix subscription).
    std::map<std::string, Feed> feeds;
    std::set<std::string> events, foreign;   // foreign: fed although no manifest source provides it
    std::vector<Pin const*> consumers;
    for (auto const& n : w.nodes)
        if (n.kind == NodeKind::Module)
            for (auto const& p : n.inputs) if (!p.topic.empty()) consumers.push_back(&p);
    auto wanted = [&](std::string const& topic) {
        for (auto const* c : consumers)
            if (c->prefix ? starts_with(topic, c->topic) : c->topic == topic) return true;
        return false;
    };
    if (body) {
        for (auto const& s : body->sources) {
            if (!s.prefix.empty()) {   // a prefix source stands for several topics: feed six
                for (int i = 0; i < 6; ++i) {
                    std::string t = s.prefix + std::to_string(i);
                    if (wanted(t)) feeds[t] = {t, s.payload, s.dims > 0 ? s.dims : 1};
                }
                continue;
            }
            if (wanted(s.topic)) feeds[s.topic] = {s.topic, s.payload, s.dims > 0 ? s.dims : 4};
        }
        for (auto const& e : body->events) if (wanted(e.topic)) events.insert(e.topic);
    }
    // Consumers of reality.proprio.* topics no manifest source provides: feed
    // them anyway (4 dims) so a graph built for another body still runs.
    for (auto const* c : consumers) {
        if (c->prefix) continue;
        if (starts_with(c->topic, "reality.proprio.") && !feeds.count(c->topic)) {
            bool produced = false;
            for (auto const& n : w.nodes) if (n.kind == NodeKind::Module) for (auto const& o : n.outputs) if (o.topic == c->topic) produced = true;
            if (!produced) { feeds[c->topic] = {c->topic, "ProprioToken", 4}; foreign.insert(c->topic); }
        }
        if (starts_with(c->topic, "events.") && c->topic.size() > 7) events.insert(c->topic);
    }
    for (auto const& [t, f] : feeds)
        r.fed.push_back(t + " [" + std::to_string(f.dims) + "]" + (foreign.count(t) ? "  (not provided by this body)" : ""));
    for (auto const& e : events) r.fed.push_back(e + " (every 30 ticks)");

    std::unique_ptr<ogma::OgmaInstance> inst;
    auto t0 = clock::now();
    try {
        inst = std::make_unique<ogma::OgmaInstance>(g.to_graph_config(), std::make_unique<ogma::InProcessBus>());
    } catch (std::exception const& e) {
        r.error = std::string("construction: ") + e.what();
        return r;
    }
    r.constructed = true;
    r.construct_ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    ogma::Bus* bus = inst->bus();

    auto t1 = clock::now();
    for (int tick = 1; tick <= ticks; ++tick) {
        for (auto const& [topic, f] : feeds) {
            if (f.payload == "RawImageFrame") {
                auto img = std::make_shared<ogma::RawImageFrame>();
                img->tick_id = uint64_t(tick);
                img->width = 32; img->height = 32; img->channels = 3;
                img->pixels.assign(size_t(32 * 32 * 3), uint8_t(tick % 255));
                bus->publish(topic, img);
                continue;
            }
            auto p = std::make_shared<ogma::ProprioToken>();
            p->tick_id = uint64_t(tick);
            p->sensor  = starts_with(topic, "reality.proprio.") ? topic.substr(16) : topic;
            p->values.resize(f.dims);
            for (int i = 0; i < f.dims; ++i) {
                double phase = double(tick) * (0.07 + 0.013 * double(i));
                p->values(i) = float(std::sin(phase));
            }
            bus->publish(topic, p);
        }
        if (tick % 30 == 0)
            for (auto const& e : events) {
                auto ev = std::make_shared<ogma::EnvEvent>();
                ev->tick_id = uint64_t(tick);
                ev->name = e.substr(7);
                ev->intensity = 1.0f;
                bus->publish(e, ev);
            }
        try {
            inst->tick();
        } catch (std::exception const& e) {
            r.error = "tick " + std::to_string(tick) + ": " + e.what();
            break;
        }
        r.ticks_done = tick;
    }
    r.tick_ms = std::chrono::duration<double, std::milli>(clock::now() - t1).count();

    for (auto const& n : w.nodes) {
        if (n.kind != NodeKind::Module) continue;
        for (auto const& o : n.outputs) {
            if (o.topic.empty() || o.prefix) continue;
            if (bus->last_value(o.topic)) r.published.push_back(o.topic); else r.silent.push_back(o.topic);
        }
    }
    if (body)
        for (auto const& s : body->sinks)
            (bus->last_value(s.topic) ? r.actions_seen : r.actions_missing).push_back(s.topic);
    return r;
}

} // namespace bb
