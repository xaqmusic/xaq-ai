#pragma once
// The document: the config file itself as ordered JSON.  metadata,
// description, _comment and stray keys survive untouched; the builder writes
// only modules[i].{id,type,params}, the modules order, and metadata.builder.
// GraphConfig::to_json() drops those keys, so the document is never a
// GraphConfig; one is derived on demand (to_graph_config).
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace ogma { struct GraphConfig; }

namespace bb {

using ojson = nlohmann::ordered_json;

struct Graph {
    ojson       doc;
    std::string path;
    bool        dirty         = false;
    bool        ascii_escapes = false;   // the source used \uXXXX (newtest.py does)

    static Graph empty(std::string const& env_target = "");
    static Graph load(std::string const& path);        // throws std::runtime_error
    void        save(std::string const& to_path);       // throws std::runtime_error
    std::string dump() const;

    // --- read
    size_t       size() const;
    ojson&       module(size_t i);
    ojson const& module(size_t i) const;
    int          index_of(std::string const& id) const;   // -1 if absent
    ojson*       find(std::string const& id);
    ojson const* find(std::string const& id) const;
    std::string  id_of(size_t i) const;
    std::string  type_of(size_t i) const;
    ojson&       params(size_t i);
    ojson const& params(size_t i) const;
    ojson&       metadata();                              // created if absent
    ojson const* metadata() const;
    std::string  env_target() const;
    std::string  body_manifest() const;                   // metadata.body_manifest or ""
    std::string  unique_id(std::string const& prefix) const;
    std::vector<std::string> ids() const;

    // --- undo.  Call checkpoint() before a mutation you want undoable; the
    // mutation helpers below do it themselves unless told not to.
    void checkpoint();
    bool can_undo() const { return !undo_.empty(); }
    bool can_redo() const { return !redo_.empty(); }
    bool undo();
    bool redo();

    // --- mutate
    std::string add_module(std::string const& type, std::string const& id, ojson params, int at = -1);
    bool        remove_module(std::string const& id);
    bool        move_module(int from, int to);
    bool        reorder(std::vector<int> const& order);   // new position → old index
    void        set_param(std::string const& id, std::string const& key, ojson value, bool with_checkpoint = true);
    void        erase_param(std::string const& id, std::string const& key);
    bool        rename_module(std::string const& old_id, std::string const& new_id);
    void        set_body_manifest(std::string const& body);

    // --- derive
    ogma::GraphConfig to_graph_config() const;            // throws

private:
    std::vector<ojson> undo_, redo_;
    void ensure_shape();
};

} // namespace bb
