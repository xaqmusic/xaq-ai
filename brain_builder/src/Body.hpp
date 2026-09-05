#pragma once
// Body manifests: the host's pre-configured sources, sinks, reads and events
// for one body, hand-authored in bodies/<body>.json from the host's own
// publish/poll code.  The builder renders them as fixed nodes.
#include <string>
#include <vector>

namespace bb {

struct BodySource { std::string name, topic, prefix, payload = "ProprioToken", description; int dims = 0; bool optional = false; };
struct BodySink   { std::string name, topic, payload = "ActionOut", description; };
struct BodyRead   { std::string name, topic, payload = "RealityToken", description; };
struct BodyEvent  { std::string name, topic, event_type, description; };

struct Body {
    std::string id;            // file stem: "microduck_joints"
    std::string body;          // "microduck"
    std::string host;          // "mj_host" | "godot"
    std::string env_target;    // metadata.env_target the launchers key on
    std::string title;
    std::string launch_hint;
    std::string description;
    std::vector<BodySource> sources;
    std::vector<BodySink>   sinks;
    std::vector<BodyRead>   reads;
    std::vector<BodyEvent>  events;

    static Body load(std::string const& path);   // throws std::runtime_error
};

struct BodyRegistry {
    std::vector<Body>        bodies;
    std::vector<std::string> warnings;
    static BodyRegistry load_dir(std::string const& dir);
    Body const* find(std::string const& id) const;
    // metadata.body_manifest, else env_target + a look at the action topics.
    Body const* guess(std::string const& body_manifest, std::string const& env_target,
                      std::vector<std::string> const& action_topics) const;
};

} // namespace bb
