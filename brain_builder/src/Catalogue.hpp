#pragma once
// The catalogue: every registered module type with its param schema (from the
// core) and its palette entry (from palette.json: category, layer, purpose,
// and the socket templates that say which params name topics).
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace bb {

enum class ParamKind { Bool, Int, Float, String, ListFloat, ListString };
const char* kind_name(ParamKind k);
ParamKind   kind_from_name(std::string const& s, ParamKind fallback);
ParamKind   kind_of_json(nlohmann::json const& v, ParamKind fallback);

struct ParamInfo {
    std::string    key;
    bool           hot = false;        // HotMutable (else ConstructionOnly)
    std::string    description;
    bool           required = false;   // no default in the schema
    nlohmann::json def;                // null when required
    nlohmann::json min, max;           // null when absent
    ParamKind      kind = ParamKind::String;
    std::vector<std::string> enum_values;   // parsed from "a|b|c" descriptions
};

// A pin template: a topic this type reads or writes whose name is set by one
// or more params.  `pattern` is the topic with each composing param in braces:
// "{input_topic}", "reality.{modality_group}.{modality_name}".
struct SocketInfo {
    std::string pattern;
    std::vector<std::string> params;
    bool output   = false;
    bool feedback = false;
    bool required = true;
    bool list     = false;   // the param is a list of topics (one pin per entry)
    bool prefix   = false;   // trailing-dot prefix subscription
    bool dynamic  = false;   // the topic depends on something the builder cannot see
    std::string payload = "Unknown";
};

// A topic literal in the module's code.
struct FixedTopic {
    std::string topic;
    bool output   = false;
    bool feedback = false;
    bool required = true;
    std::string payload = "Unknown";
};

struct TypeInfo {
    std::string type;
    std::string category = "other";
    int         layer = 3;
    std::string purpose;
    bool        deprecated = false;
    std::string deprecated_note;
    std::string id_prefix;
    bool        no_trial_setup = false;
    std::string baseline;              // config the sockets were probed from
    nlohmann::json baseline_params;    // hand-authored params for types no config uses
    std::vector<ParamInfo>  params;
    std::vector<SocketInfo> sockets;
    std::vector<FixedTopic> fixed;

    ParamInfo const* param(std::string const& key) const;
};

extern const std::vector<std::string> kCategoryOrder;
extern const std::vector<std::string> kPayloadTypes;

struct Catalogue {
    std::vector<TypeInfo>    types;      // category order, then name
    std::vector<std::string> warnings;
    std::string              palette_path;

    TypeInfo const* find(std::string const& type) const;
    std::vector<std::string> categories() const;   // those present, in kCategoryOrder
    nlohmann::ordered_json to_json() const;

    static Catalogue build(std::string const& palette_path);
};

nlohmann::ordered_json socket_to_json(SocketInfo const& s);
nlohmann::ordered_json fixed_to_json(FixedTopic const& f);
SocketInfo socket_from_json(nlohmann::json const& j);
FixedTopic fixed_from_json(nlohmann::json const& j);

} // namespace bb
