#include "Wiring.hpp"

#include <algorithm>
#include <regex>
#include <set>

#include "ogma/PayloadTypeName.hpp"

namespace bb {

uint64_t stable_id(std::string const& key) {
    uint64_t h = 1469598103934665603ull;
    for (unsigned char c : key) { h ^= c; h *= 1099511628211ull; }
    return h | 1ull;
}

namespace {

bool starts_with(std::string const& s, std::string const& p) { return s.rfind(p, 0) == 0; }

std::string json_scalar_string(ojson const& v) {
    if (v.is_string()) return v.get<std::string>();
    if (v.is_number_integer()) return std::to_string(v.get<long long>());
    if (v.is_boolean()) return v.get<bool>() ? "true" : "false";
    return "";
}

// Substitute {param} in a socket pattern from the module's params; "" if any
// composing param is unset.
std::string compose(SocketInfo const& s, ojson const& params) {
    std::string out = s.pattern;
    for (auto const& p : s.params) {
        std::string ph = "{" + p + "}";
        if (!params.contains(p)) return "";
        std::string v = json_scalar_string(params[p]);
        if (v.empty()) return "";
        size_t at = out.find(ph);
        if (at == std::string::npos) return "";
        out.replace(at, ph.size(), v);
    }
    return out;
}

// A socket param counts as set when the document or the schema default gives
// it a non-empty value (MotorEPMv2's optional inputs default to real topics).
bool socket_unset(SocketInfo const& s, ojson const& params, TypeInfo const* ti) {
    for (auto const& p : s.params) {
        nlohmann::json v;
        if (params.contains(p)) v = params[p];
        else if (ti) if (auto const* pi = ti->param(p)) v = pi->def;
        if (v.is_null()) return true;
        if (v.is_string() && v.get<std::string>().empty()) return true;
        if (v.is_array() && v.empty()) return true;
    }
    return false;
}

// The socket's current topics from the document (or the schema default).
std::vector<std::string> socket_topics(SocketInfo const& s, ojson const& params, TypeInfo const* ti) {
    std::vector<std::string> out;
    ojson eff = params;
    if (ti) for (auto const& p : s.params)
        if (!eff.contains(p)) if (auto const* pi = ti->param(p)) if (!pi->def.is_null()) eff[p] = pi->def;
    if (s.list && s.params.size() == 1) {
        if (!eff.contains(s.params[0]) || !eff[s.params[0]].is_array()) return out;
        std::string ph = "{" + s.params[0] + "}";
        for (auto const& e : eff[s.params[0]]) {
            if (!e.is_string() || e.get<std::string>().empty()) continue;
            std::string t = s.pattern;
            size_t at = t.find(ph);
            if (at != std::string::npos) t.replace(at, ph.size(), e.get<std::string>());
            out.push_back(t);
        }
        return out;
    }
    std::string t = compose(s, eff);
    if (!t.empty()) out.push_back(t);
    return out;
}

Pin make_pin(std::string const& node, bool output, std::string const& key, std::string const& topic) {
    Pin p;
    p.node   = node;
    p.output = output;
    p.topic  = topic;
    p.label  = topic;
    p.id     = stable_id("pin:" + node + ":" + (output ? "out:" : "in:") + key);
    return p;
}

} // namespace

// ---------------------------------------------------------------------------
TrialResult const& TrialCache::get(std::string const& type, std::string const& id, ojson const& params) {
    std::string key = type + '\x1f' + id + '\x1f' + params.dump();
    auto it = cache_.find(key);
    if (it != cache_.end()) return it->second;
    ogma::ParamMap pm;
    try { pm = param_map_from_json(params); }
    catch (std::exception const& e) {
        TrialResult r; r.ok = false; r.error = std::string("params: ") + e.what();
        return cache_.emplace(key, std::move(r)).first->second;
    }
    return cache_.emplace(key, trial_setup(type, id, pm)).first->second;
}

// ---------------------------------------------------------------------------
Node const* Wiring::node(uint64_t id) const { for (auto const& n : nodes) if (n.id == id) return &n; return nullptr; }
Node*       Wiring::node(uint64_t id)       { for (auto& n : nodes) if (n.id == id) return &n; return nullptr; }
Node const* Wiring::node_named(std::string const& name) const { for (auto const& n : nodes) if (n.name == name) return &n; return nullptr; }

Pin const* Wiring::pin(uint64_t id) const {
    for (auto const& n : nodes) {
        for (auto const& p : n.inputs)  if (p.id == id) return &p;
        for (auto const& p : n.outputs) if (p.id == id) return &p;
    }
    return nullptr;
}

Node const* Wiring::owner(uint64_t pin_id) const {
    for (auto const& n : nodes) {
        for (auto const& p : n.inputs)  if (p.id == pin_id) return &n;
        for (auto const& p : n.outputs) if (p.id == pin_id) return &n;
    }
    return nullptr;
}

std::vector<std::string> Wiring::known_topics() const {
    std::set<std::string> s;
    for (auto const& n : nodes)
        for (auto const& p : n.outputs) if (!p.topic.empty()) s.insert(p.topic);
    return {s.begin(), s.end()};
}

std::vector<Link const*> Wiring::links_of(uint64_t pin_id) const {
    std::vector<Link const*> out;
    for (auto const& l : links) if (l.from_pin == pin_id || l.to_pin == pin_id) out.push_back(&l);
    return out;
}

// ---------------------------------------------------------------------------
Wiring Wiring::build(Graph const& g, Catalogue const& cat, Body const* body, TrialCache& cache) {
    Wiring w;
    std::set<std::string> seen_ids;

    for (size_t i = 0; i < g.size(); ++i) {
        Node n;
        n.kind  = NodeKind::Module;
        n.name  = g.id_of(i);
        n.type  = g.type_of(i);
        n.index = int(i);
        n.id    = stable_id("node:" + n.name);
        if (n.name.empty()) { n.name = "#" + std::to_string(i); n.id = stable_id("node:" + n.name); }
        if (!seen_ids.insert(n.name).second)
            w.diagnostics.push_back({Diagnostic::Error, n.name, "duplicate module id '" + n.name + "'"});
        TypeInfo const* ti = cat.find(n.type);
        if (ti) n.category = ti->category;
        ojson const& params = g.params(i);

        if (!ti) {
            n.setup_ok = false;
            n.setup_error = "unknown module type '" + n.type + "'";
        } else {
            TrialResult const& tr = cache.get(n.type, n.name, params);
            n.setup_ok = tr.ok;
            n.setup_error = tr.error;
            if (tr.ok) {
                for (auto const& spec : tr.ports.inputs) {
                    Pin p = make_pin(n.name, false, spec.name, spec.name);
                    p.payload  = ogma::payload_type_name(spec.payload_type);
                    p.feedback = spec.kind == ogma::SubscriptionKind::Feedback;
                    p.required = spec.required;
                    p.prefix   = !spec.name.empty() && spec.name.back() == '.';
                    n.inputs.push_back(std::move(p));
                }
                for (auto const& spec : tr.ports.outputs) {
                    Pin p = make_pin(n.name, true, spec.name, spec.name);
                    p.payload = ogma::payload_type_name(spec.payload_type);
                    p.prefix  = !spec.name.empty() && spec.name.back() == '.';
                    n.outputs.push_back(std::move(p));
                }
            }
            // Placeholders: sockets whose params are unset (or every socket
            // and fixed topic when setup failed, so the node keeps its shape).
            for (auto const& s : ti->sockets) {
                bool unset = socket_unset(s, params, ti);
                if (!unset) {
                    // Set, but absent from the declared ports (setup failed, or
                    // the module polls the topic with last_value): show it anyway.
                    for (auto const& topic : socket_topics(s, params, ti)) {
                        auto& side = s.output ? n.outputs : n.inputs;
                        bool declared = false;
                        for (auto const& q : side) if (q.topic == topic) declared = true;
                        if (declared) continue;
                        Pin p = make_pin(n.name, s.output, topic, topic);
                        p.payload = s.payload; p.feedback = s.feedback; p.required = s.required; p.prefix = s.prefix;
                        p.polled = n.setup_ok;
                        side.push_back(std::move(p));
                    }
                    continue;
                }
                if (!s.required && !s.output && n.setup_ok) continue;   // reachable through the "+" pin
                Pin p = make_pin(n.name, s.output, "param:" + s.pattern, "");
                p.label       = s.pattern;
                p.payload     = s.payload;
                p.feedback    = s.feedback;
                p.required    = s.required;
                p.prefix      = s.prefix;
                p.list        = s.list;
                p.placeholder = true;
                p.param       = s.params.empty() ? "" : s.params.front();
                (s.output ? n.outputs : n.inputs).push_back(std::move(p));
            }
            {
                bool any_optional = false;
                for (auto const& s : ti->sockets)
                    if (!s.required && !s.output && socket_unset(s, params, ti)) any_optional = true;
                if (any_optional && n.setup_ok) {
                    Pin p = make_pin(n.name, false, "+", "");
                    p.label = "+"; p.plus = true; p.required = false; p.placeholder = true;
                    n.inputs.push_back(std::move(p));
                }
            }
            if (!n.setup_ok)
                for (auto const& f : ti->fixed) {
                    Pin p = make_pin(n.name, f.output, f.topic, f.topic);
                    p.payload = f.payload; p.feedback = f.feedback; p.required = f.required;
                    p.prefix = !f.topic.empty() && f.topic.back() == '.';
                    (f.output ? n.outputs : n.inputs).push_back(std::move(p));
                }
        }
        if (!n.setup_ok)
            w.diagnostics.push_back({Diagnostic::Error, n.name, n.setup_error});
        w.nodes.push_back(std::move(n));
    }

    if (body) {
        Node src; src.kind = NodeKind::Sources; src.name = "@sources"; src.type = body->title; src.category = "body";
        src.id = stable_id("node:@sources");
        for (auto const& s : body->sources) {
            std::string topic = s.topic.empty() ? s.prefix : s.topic;
            Pin p = make_pin(src.name, true, topic, topic);
            p.label = s.name; p.payload = s.payload; p.description = s.description; p.dims = s.dims;
            p.prefix = !s.prefix.empty(); p.required = !s.optional;
            src.outputs.push_back(std::move(p));
        }
        Node sink; sink.kind = NodeKind::Sinks; sink.name = "@sinks"; sink.type = body->title; sink.category = "body";
        sink.id = stable_id("node:@sinks");
        for (auto const& s : body->sinks) {
            Pin p = make_pin(sink.name, false, s.topic, s.topic);
            p.label = s.name; p.payload = s.payload; p.description = s.description;
            sink.inputs.push_back(std::move(p));
        }
        for (auto const& r : body->reads) {
            Pin p = make_pin(sink.name, false, r.topic, r.topic);
            p.label = r.name + " (read)"; p.payload = r.payload; p.description = r.description; p.required = false;
            sink.inputs.push_back(std::move(p));
        }
        Node ev; ev.kind = NodeKind::Events; ev.name = "@events"; ev.type = body->title; ev.category = "body";
        ev.id = stable_id("node:@events");
        for (auto const& e : body->events) {
            Pin p = make_pin(ev.name, true, e.topic, e.topic);
            p.label = e.name; p.payload = "EnvEvent"; p.description = e.description;
            ev.outputs.push_back(std::move(p));
        }
        w.nodes.push_back(std::move(src));
        w.nodes.push_back(std::move(sink));
        w.nodes.push_back(std::move(ev));
    }

    // Links: every consumer against every producer.
    struct Ref { Node const* n; Pin const* p; };
    std::vector<Ref> producers, consumers;
    for (auto const& n : w.nodes) {
        for (auto const& p : n.outputs) if (!p.topic.empty()) producers.push_back({&n, &p});
        for (auto const& p : n.inputs)  if (!p.topic.empty()) consumers.push_back({&n, &p});
    }
    for (auto const& c : consumers) {
        for (auto const& p : producers) {
            if (c.n == p.n) continue;
            bool match = c.p->prefix ? starts_with(p.p->topic, c.p->topic)
                       : p.p->prefix ? starts_with(c.p->topic, p.p->topic)
                                     : p.p->topic == c.p->topic;
            if (!match) continue;
            Link l;
            l.from_pin  = p.p->id;
            l.to_pin    = c.p->id;
            l.from_node = p.n->name;
            l.to_node   = c.n->name;
            l.topic     = c.p->prefix ? p.p->topic : c.p->topic;
            l.feedback  = c.p->feedback;
            l.type_ok   = p.p->payload == c.p->payload || p.p->payload == "Unknown" || c.p->payload == "Unknown";
            l.id        = stable_id("link:" + std::to_string(l.from_pin) + ":" + std::to_string(l.to_pin));
            w.links.push_back(std::move(l));
        }
    }

    // Diagnostics.
    std::set<uint64_t> linked;
    for (auto const& l : w.links) { linked.insert(l.from_pin); linked.insert(l.to_pin); }
    for (auto const& n : w.nodes) {
        if (n.kind != NodeKind::Module) continue;
        for (auto const& p : n.inputs) {
            if (p.plus) continue;
            if (p.placeholder) {
                if (p.required && n.setup_ok)
                    w.diagnostics.push_back({Diagnostic::Error, n.name, "input " + p.label + " is not set"});
                continue;
            }
            if (linked.count(p.id)) continue;
            if (p.feedback) continue;
            if (p.required) w.diagnostics.push_back({Diagnostic::Error, n.name, "no producer for required input " + p.topic});
            else            w.diagnostics.push_back({Diagnostic::Info,  n.name, "optional input " + p.topic + " has no producer"});
        }
        for (auto const& p : n.outputs) {
            if (p.placeholder) continue;   // an unset output param is a choice, not a fault
            if (linked.count(p.id)) continue;
            if (starts_with(p.topic, "action.") && body)
                w.diagnostics.push_back({Diagnostic::Warning, n.name, "the body will not act on " + p.topic});
            else
                w.diagnostics.push_back({Diagnostic::Warning, n.name, "nothing reads " + p.topic});
        }
    }
    for (auto const& l : w.links)
        if (!l.type_ok) {
            Pin const* a = w.pin(l.from_pin); Pin const* b = w.pin(l.to_pin);
            // A prefix subscription catches every payload under it by design.
            auto sev = (b && b->prefix) ? Diagnostic::Info : Diagnostic::Warning;
            w.diagnostics.push_back({sev, l.to_node, l.topic + ": " + (a ? a->payload : "?") +
                                     " from " + l.from_node + " into a " + (b ? b->payload : "?") + " input"});
        }
    for (auto const& d : w.diagnostics) {
        if (d.severity == Diagnostic::Error)   ++w.errors;
        if (d.severity == Diagnostic::Warning) ++w.warnings;
    }
    return w;
}

// ---------------------------------------------------------------------------
std::vector<SocketInfo> Wiring::optional_unset_inputs(Graph const& g, Catalogue const& cat, Node const& n) const {
    std::vector<SocketInfo> out;
    if (n.kind != NodeKind::Module || n.index < 0 || size_t(n.index) >= g.size()) return out;
    TypeInfo const* ti = cat.find(n.type);
    if (!ti) return out;
    for (auto const& s : ti->sockets)
        if (!s.required && !s.output && socket_unset(s, g.params(size_t(n.index)), ti)) out.push_back(s);
    return out;
}

Slot Wiring::resolve(Graph const& g, Catalogue const& cat, Node const& n, Pin const& p) const {
    Slot s;
    if (n.kind != NodeKind::Module || n.index < 0 || size_t(n.index) >= g.size()) { s.kind = Slot::Fixed; return s; }
    if (p.placeholder) { s.kind = Slot::Placeholder; s.param = p.param; s.list = p.list; return s; }
    ojson const& params = g.params(size_t(n.index));
    for (auto const& [k, v] : params.items())
        if (v.is_string() && v.get<std::string>() == p.topic) { s.kind = Slot::Scalar; s.param = k; return s; }
    for (auto const& [k, v] : params.items())
        if (v.is_array())
            for (size_t i = 0; i < v.size(); ++i)
                if (v[i].is_string() && v[i].get<std::string>() == p.topic) {
                    s.kind = Slot::ListEntry; s.param = k; s.index = int(i); return s;
                }
    if (TypeInfo const* ti = cat.find(n.type))
        for (auto const& sock : ti->sockets)
            if (sock.params.size() >= 2 && sock.output == p.output && compose(sock, params) == p.topic) {
                s.kind = Slot::Composed; s.params = sock.params; s.pattern = sock.pattern; return s;
            }
    s.kind = Slot::Fixed;
    return s;
}

namespace {

// Set a socket param from a topic: scalar, list append, or composed split.
std::string assign_socket(Graph& g, Catalogue const& cat, Node const& n, Pin const& pin, Slot const& slot, std::string const& topic) {
    std::string id = n.name;
    switch (slot.kind) {
        case Slot::Placeholder: {
            if (slot.param.empty()) return "that pin has no param to set";
            TypeInfo const* ti = cat.find(n.type);
            SocketInfo const* sock = nullptr;
            if (ti) for (auto const& s : ti->sockets) if (s.pattern == pin.label && s.output == pin.output) sock = &s;
            if (sock && sock->params.size() >= 2) {
                // Split the topic by the pattern: {a} → ([^.]+)
                std::string re_src = "^";
                std::vector<std::string> order;
                std::string pat = sock->pattern;
                size_t pos = 0;
                while (pos < pat.size()) {
                    size_t open = pat.find('{', pos);
                    if (open == std::string::npos) { for (size_t i = pos; i < pat.size(); ++i) { if (std::string(".+*?()[]\\^$|").find(pat[i]) != std::string::npos) re_src += '\\'; re_src += pat[i]; } break; }
                    for (size_t i = pos; i < open; ++i) { if (std::string(".+*?()[]\\^$|").find(pat[i]) != std::string::npos) re_src += '\\'; re_src += pat[i]; }
                    size_t close = pat.find('}', open);
                    if (close == std::string::npos) return "bad socket pattern " + pat;
                    order.push_back(pat.substr(open + 1, close - open - 1));
                    re_src += "([^.]+)";
                    pos = close + 1;
                }
                re_src += "$";
                std::smatch m;
                if (!std::regex_match(topic, m, std::regex(re_src)))
                    return "'" + topic + "' does not fit " + sock->pattern;
                g.checkpoint();
                for (size_t i = 0; i < order.size(); ++i) g.set_param(id, order[i], std::string(m[i + 1]), false);
                return "";
            }
            if (slot.list) {
                ojson lst = ojson::array();
                ojson const* cur = g.find(id);
                if (cur && (*cur)["params"].contains(slot.param) && (*cur)["params"][slot.param].is_array())
                    lst = (*cur)["params"][slot.param];
                lst.push_back(topic);
                g.set_param(id, slot.param, lst);
            } else {
                g.set_param(id, slot.param, topic);
            }
            return "";
        }
        case Slot::Scalar:
            g.set_param(id, slot.param, topic);
            return "";
        case Slot::ListEntry: {
            ojson const* cur = g.find(id);
            if (!cur) return "module vanished";
            ojson lst = (*cur)["params"][slot.param];
            if (!lst.is_array() || slot.index < 0 || size_t(slot.index) >= lst.size()) return "list entry vanished";
            lst[size_t(slot.index)] = topic;
            g.set_param(id, slot.param, lst);
            return "";
        }
        case Slot::Composed: {
            std::string joined;
            for (auto const& p : slot.params) joined += (joined.empty() ? "" : ", ") + p;
            return "that topic is composed from " + joined + " — edit those in Properties";
        }
        case Slot::Fixed:
            return "'" + pin.topic + "' is hard-wired in " + n.type + "; wire the other side to it instead";
    }
    return "unreachable";
}

} // namespace

std::string Wiring::connect(Graph& g, Catalogue const& cat, uint64_t from_pin, uint64_t to_pin) const {
    Pin const* a = pin(from_pin); Pin const* b = pin(to_pin);
    if (!a || !b) return "unknown pin";
    if (a->output == b->output) return "connect an output to an input";
    Pin const* src = a->output ? a : b;
    Pin const* dst = a->output ? b : a;
    Node const* ns = owner(src->id); Node const* nd = owner(dst->id);
    if (!ns || !nd) return "unknown node";
    if (ns == nd) return "a module cannot feed itself";
    if (ns->kind != NodeKind::Module && nd->kind != NodeKind::Module) return "wire a module, not the body to itself";
    if (dst->plus) return src->placeholder ? "one side must already have a topic" : "choose";
    if (src->placeholder && dst->placeholder) return "one side must already have a topic";

    if (!src->placeholder && nd->kind == NodeKind::Module) {
        // The producer's name wins: set the consumer's param to it.
        Slot s = resolve(g, cat, *nd, *dst);
        if (s.kind == Slot::Fixed || s.kind == Slot::Composed) {
            // Perhaps the producer is the editable side.
            if (ns->kind == NodeKind::Module && !dst->topic.empty()) {
                Slot ss = resolve(g, cat, *ns, *src);
                if (ss.kind != Slot::Fixed && ss.kind != Slot::Composed)
                    return assign_socket(g, cat, *ns, *src, ss, dst->topic);
            }
        }
        return assign_socket(g, cat, *nd, *dst, s, src->topic);
    }
    if (src->placeholder) {
        if (dst->topic.empty()) return "the input has no topic yet";
        Slot ss = resolve(g, cat, *ns, *src);
        return assign_socket(g, cat, *ns, *src, ss, dst->topic);
    }
    // src live, dst is a body sink/read with a fixed topic.
    if (src->topic == dst->topic) return "";
    Slot ss = resolve(g, cat, *ns, *src);
    return assign_socket(g, cat, *ns, *src, ss, dst->topic);
}

std::string Wiring::disconnect(Graph& g, Catalogue const& cat, Link const& link) const {
    Pin const* dst = pin(link.to_pin); Pin const* src = pin(link.from_pin);
    Node const* nd = owner(link.to_pin); Node const* ns = owner(link.from_pin);
    if (!dst || !src || !nd || !ns) return "unknown link";
    auto clear = [&](Node const& n, Pin const& p) -> std::string {
        Slot s = resolve(g, cat, n, p);
        switch (s.kind) {
            case Slot::Scalar:    g.erase_param(n.name, s.param); return "";
            case Slot::ListEntry: {
                ojson const* cur = g.find(n.name);
                if (!cur) return "module vanished";
                ojson lst = (*cur)["params"][s.param];
                if (!lst.is_array() || size_t(s.index) >= lst.size()) return "list entry vanished";
                lst.erase(lst.begin() + s.index);
                g.set_param(n.name, s.param, lst);
                return "";
            }
            case Slot::Composed:  return "that topic is composed from params; edit them in Properties";
            case Slot::Fixed:     return "'" + p.topic + "' is hard-wired in " + n.type;
            case Slot::Placeholder: return "";
        }
        return "";
    };
    if (nd->kind == NodeKind::Module) {
        std::string e = clear(*nd, *dst);
        if (e.empty() || ns->kind != NodeKind::Module) return e;
        return clear(*ns, *src);
    }
    if (ns->kind == NodeKind::Module) return clear(*ns, *src);
    return "nothing to edit";
}

} // namespace bb
