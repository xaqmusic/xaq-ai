#include "ogma/LiveGraph.hpp"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

#include "ogma/InProcessBus.hpp"
#include "ogma/Module.hpp"
#include "ogma/OgmaInstance.hpp"

namespace ogma {

namespace {

std::string need_string(nlohmann::json const& j, char const* key, char const* op) {
    if (!j.contains(key) || !j[key].is_string() || j[key].get<std::string>().empty())
        throw std::invalid_argument(std::string(op) + ": '" + key + "' (non-empty string) is required");
    return j[key].get<std::string>();
}

nlohmann::json edge_to_json(EdgeSpec const& e) {
    return {{"from", e.from}, {"to", e.to}, {"topic", e.topic}, {"feedback", e.feedback}};
}

} // namespace

LiveGraph::LiveGraph(OgmaInstance& inst, std::string source_path)
    : inst_(inst), source_path_(std::move(source_path)), specs_(inst.config().modules) {}

GraphPatchBatch LiveGraph::batch_from_json(nlohmann::json const& ops, std::string const& source) {
    if (!ops.is_array()) throw std::invalid_argument("'ops' must be an array");
    GraphPatchBatch batch;
    batch.source = source;
    for (size_t i = 0; i < ops.size(); ++i) {
        nlohmann::json const& o = ops[i];
        if (!o.is_object()) throw std::invalid_argument("ops[" + std::to_string(i) + "] must be an object");
        std::string op = o.value("op", "");
        if (op == "add_node") {
            AddNodeOp a;
            a.spec.id   = need_string(o, "id", "add_node");
            a.spec.type = need_string(o, "type", "add_node");
            if (o.contains("params")) {
                if (!o["params"].is_object()) throw std::invalid_argument("add_node: 'params' must be an object");
                for (auto it = o["params"].begin(); it != o["params"].end(); ++it)
                    a.spec.params[it.key()] = GraphConfig::param_from_json(it.value());
            }
            batch.ops.emplace_back(std::move(a));
        } else if (op == "remove_node") {
            batch.ops.emplace_back(RemoveNodeOp{need_string(o, "id", "remove_node")});
        } else if (op == "connect") {
            ConnectOp c;
            c.edge.from     = need_string(o, "from", "connect");
            c.edge.to       = need_string(o, "to", "connect");
            c.edge.topic    = o.value("topic", "");
            c.edge.feedback = o.value("feedback", false);
            batch.ops.emplace_back(std::move(c));
        } else if (op == "disconnect") {
            DisconnectOp d;
            d.from  = need_string(o, "from", "disconnect");
            d.to    = need_string(o, "to", "disconnect");
            d.topic = o.value("topic", "");
            batch.ops.emplace_back(std::move(d));
        } else if (op == "set_param") {
            SetParamOp s;
            s.target_id = need_string(o, "id", "set_param");
            s.key       = need_string(o, "key", "set_param");
            if (!o.contains("value")) throw std::invalid_argument("set_param: 'value' is required");
            s.value = GraphConfig::param_from_json(o["value"]);
            batch.ops.emplace_back(std::move(s));
        } else {
            throw std::invalid_argument("ops[" + std::to_string(i) + "]: unknown op '" + op + "'");
        }
    }
    if (batch.ops.empty()) throw std::invalid_argument("empty patch");
    return batch;
}

std::vector<std::string> LiveGraph::validate_offline(GraphPatchBatch const& batch) {
    std::vector<std::string> errors;
    for (auto const& op : batch.ops) {
        auto const* add = std::get_if<AddNodeOp>(&op);
        if (!add) continue;
        try {
            InProcessBus bus;
            ModulePtr trial = ModuleRegistry::instance().create(add->spec.type);
            if (!trial) { errors.push_back("add_node '" + add->spec.id + "': unknown type '" + add->spec.type + "'"); continue; }
            trial->set_id(add->spec.id);
            trial->on_setup(&bus, add->spec.params);
            trial->on_teardown();
        } catch (std::exception const& e) {
            errors.push_back("add_node '" + add->spec.id + "' (" + add->spec.type + "): " + e.what());
        }
    }
    return errors;
}

void LiveGraph::reconcile() {
    std::unordered_set<std::string> live;
    for (auto* m : inst_.modules()) live.insert(std::string(m->id()));
    specs_.erase(std::remove_if(specs_.begin(), specs_.end(),
                                [&](ModuleSpec const& s) { return !live.count(s.id); }),
                 specs_.end());
    std::unordered_set<std::string> known;
    for (auto const& s : specs_) known.insert(s.id);
    for (auto* m : inst_.modules())
        if (!known.count(std::string(m->id()))) {
            ModuleSpec s;
            s.id     = std::string(m->id());
            s.type   = std::string(m->type_name());
            s.params = m->current_params();
            specs_.push_back(std::move(s));
        }
}

nlohmann::json LiveGraph::apply(GraphPatchBatch batch) {
    reconcile();
    std::unordered_set<std::string> live;
    for (auto const& s : specs_) live.insert(s.id);
    auto known_or_host = [&](std::string const& id) { return id.rfind("host:", 0) == 0 || live.count(id) > 0; };

    for (auto const& op : batch.ops) {
        if (auto const* a = std::get_if<AddNodeOp>(&op)) {
            if (live.count(a->spec.id)) throw std::invalid_argument("add_node: id '" + a->spec.id + "' already exists");
            live.insert(a->spec.id);
        } else if (auto const* r = std::get_if<RemoveNodeOp>(&op)) {
            if (!live.count(r->id)) throw std::invalid_argument("remove_node: no module '" + r->id + "'");
            live.erase(r->id);
        } else if (auto const* s = std::get_if<SetParamOp>(&op)) {
            Module* m = inst_.module(s->target_id);
            if (!m) throw std::invalid_argument("set_param: no module '" + s->target_id + "'");
            bool hot = false, found = false;
            for (auto const& p : m->params_schema())
                if (p.key == s->key) { found = true; hot = p.mutability == ParamMutability::HotMutable; }
            if (!found) throw std::invalid_argument("set_param: '" + s->target_id + "' has no param '" + s->key + "'");
            if (!hot) throw std::invalid_argument("set_param: '" + s->key + "' on '" + s->target_id + "' is construction-only; recreate the module to change it");
        } else if (auto const* c = std::get_if<ConnectOp>(&op)) {
            if (!known_or_host(c->edge.from) || !known_or_host(c->edge.to))
                throw std::invalid_argument("connect: unknown endpoint " + c->edge.from + " -> " + c->edge.to);
        } else if (auto const* d = std::get_if<DisconnectOp>(&op)) {
            if (!known_or_host(d->from) || !known_or_host(d->to))
                throw std::invalid_argument("disconnect: unknown endpoint " + d->from + " -> " + d->to);
        }
    }

    // Ledger first (the scheduler applies between ticks; a rejection there
    // is printed by the scheduler and corrected by the next reconcile).
    for (auto const& op : batch.ops) {
        if (auto const* a = std::get_if<AddNodeOp>(&op)) specs_.push_back(a->spec);
        else if (auto const* r = std::get_if<RemoveNodeOp>(&op))
            specs_.erase(std::remove_if(specs_.begin(), specs_.end(), [&](ModuleSpec const& s) { return s.id == r->id; }), specs_.end());
        else if (auto const* s = std::get_if<SetParamOp>(&op))
            for (auto& spec : specs_) if (spec.id == s->target_id) spec.params[s->key] = s->value;
    }
    auto batch_id = inst_.enqueue_hot_patch(std::move(batch));
    ++version_;
    return {{"status", "ok"}, {"batch_id", int64_t(batch_id)}, {"graph_version", int64_t(version_)}};
}

void LiveGraph::record_set_param(std::string const& id, std::string const& key, ParamValue const& value) {
    for (auto& spec : specs_) if (spec.id == id) { spec.params[key] = value; ++version_; return; }
}

nlohmann::json LiveGraph::get_graph() {
    reconcile();
    GraphConfig const& boot = inst_.config();
    nlohmann::json runtime = {
        {"thread_pool", boot.runtime.thread_pool == ThreadPoolPolicy::Shared ? "shared" : "per_instance"},
        {"num_threads", boot.runtime.num_threads},
        {"auto_subscribe", boot.runtime.auto_subscribe}};
    nlohmann::json modules = nlohmann::json::array();
    for (auto const& s : specs_) {
        nlohmann::json params = nlohmann::json::object();
        for (auto const& [k, v] : s.params) params[k] = GraphConfig::param_to_json(v);
        modules.push_back({{"id", s.id}, {"type", s.type}, {"params", params}});
    }
    nlohmann::json edges = nlohmann::json::array();
    for (auto const& e : inst_.current_edges()) edges.push_back(edge_to_json(e));
    nlohmann::json config = {{"version", boot.version}, {"runtime", runtime}, {"modules", modules}, {"edges", edges}};
    return {{"status", "ok"}, {"graph_version", int64_t(version_)}, {"source_path", source_path_},
            {"config", config}, {"edges", edges}, {"module_count", specs_.size()}};
}

} // namespace ogma
