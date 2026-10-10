#pragma once
// The wiring model the canvas draws: nodes (modules + the body's fixed
// nodes), pins (live topics from trial setup, placeholders from the palette
// sockets), links (topic matches), diagnostics, and the two edits a drag
// performs — connect and disconnect — which are param edits on the Graph.
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "Body.hpp"
#include "Catalogue.hpp"
#include "Graph.hpp"
#include "TrialSetup.hpp"

namespace bb {

uint64_t stable_id(std::string const& key);   // fnv1a64, never 0

enum class NodeKind { Module, Sources, Sinks, Events };

struct Pin {
    uint64_t    id = 0;
    std::string node;              // owner node name
    bool        output = false;
    std::string topic;             // "" for a placeholder
    std::string label;
    std::string payload = "Unknown";
    bool feedback = false, required = true, prefix = false, placeholder = false, list = false;
    bool polled = false;           // named by a param but not a declared port (read by last_value)
    bool plus   = false;           // the "+" pin: drop a producer here to pick an optional input socket
    std::string param;             // the socket param (placeholder)
    std::string description;       // body pins: the manifest text
    int         dims = 0;
};

struct Node {
    uint64_t    id = 0;
    NodeKind    kind = NodeKind::Module;
    std::string name;              // module id, or "@sources" / "@sinks" / "@events"
    std::string type;              // module type, or the body title
    std::string category;
    int         index = -1;        // module array index
    bool        setup_ok = true;
    std::string setup_error;
    std::vector<Pin> inputs, outputs;
};

struct Link {
    uint64_t    id = 0, from_pin = 0, to_pin = 0;
    std::string from_node, to_node, topic;
    bool        feedback = false, type_ok = true;
};

struct Diagnostic {
    enum Severity { Info, Warning, Error } severity = Info;
    std::string node, message;
};

// Where a pin's topic lives in its module's params.
struct Slot {
    enum Kind { Placeholder, Scalar, ListEntry, Composed, Fixed } kind = Fixed;
    std::string param;
    int         index = -1;
    bool        list = false;
    std::vector<std::string> params;   // Composed
    std::string pattern;               // Composed
};

class TrialCache {
public:
    TrialResult const& get(std::string const& type, std::string const& id, ojson const& params);
    void clear() { cache_.clear(); }
private:
    std::unordered_map<std::string, TrialResult> cache_;
};

struct Wiring {
    std::vector<Node>       nodes;
    std::vector<Link>       links;
    std::vector<Diagnostic> diagnostics;
    int errors = 0, warnings = 0;

    Node const* node(uint64_t id) const;
    Node*       node(uint64_t id);
    Node const* node_named(std::string const& name) const;
    Pin const*  pin(uint64_t id) const;
    Node const* owner(uint64_t pin_id) const;
    std::vector<std::string> known_topics() const;
    std::vector<Link const*> links_of(uint64_t pin_id) const;

    static Wiring build(Graph const& g, Catalogue const& cat, Body const* body, TrialCache& cache);

    Slot        resolve(Graph const& g, Catalogue const& cat, Node const& n, Pin const& p) const;
    // Optional input sockets of a module that are still unset (for the "+" pin).
    std::vector<SocketInfo> optional_unset_inputs(Graph const& g, Catalogue const& cat, Node const& n) const;
    // "" on success, otherwise the reason (shown to the operator).
    std::string connect(Graph& g, Catalogue const& cat, uint64_t from_pin, uint64_t to_pin) const;
    std::string disconnect(Graph& g, Catalogue const& cat, Link const& link) const;
};

} // namespace bb
