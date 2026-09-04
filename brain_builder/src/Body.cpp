#include "Body.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace bb {

Body Body::load(std::string const& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j = nlohmann::json::parse(f, nullptr, false);
    if (j.is_discarded()) throw std::runtime_error(path + ": not valid JSON");
    Body b;
    b.id          = fs::path(path).stem().string();
    b.body        = j.value("body", b.id);
    b.host        = j.value("host", "");
    b.env_target  = j.value("env_target", b.body);
    b.title       = j.value("title", b.id);
    b.launch_hint = j.value("launch_hint", "");
    b.description = j.value("description", "");
    for (auto const& s : j.value("sources", nlohmann::json::array())) {
        BodySource x;
        x.name = s.value("name", ""); x.topic = s.value("topic", ""); x.prefix = s.value("prefix", "");
        x.payload = s.value("payload", "ProprioToken"); x.description = s.value("description", "");
        x.dims = s.value("dims", 0); x.optional = s.value("optional", false);
        if (x.topic.empty() && !x.prefix.empty()) x.topic = x.prefix;
        b.sources.push_back(std::move(x));
    }
    for (auto const& s : j.value("sinks", nlohmann::json::array())) {
        BodySink x;
        x.name = s.value("name", ""); x.topic = s.value("topic", "");
        x.payload = s.value("payload", "ActionOut"); x.description = s.value("description", "");
        b.sinks.push_back(std::move(x));
    }
    for (auto const& s : j.value("reads", nlohmann::json::array())) {
        BodyRead x;
        x.name = s.value("name", ""); x.topic = s.value("topic", "");
        x.payload = s.value("payload", "RealityToken"); x.description = s.value("description", "");
        b.reads.push_back(std::move(x));
    }
    for (auto const& s : j.value("events", nlohmann::json::array())) {
        BodyEvent x;
        x.name = s.value("name", ""); x.topic = s.value("topic", "");
        x.event_type = s.value("event_type", ""); x.description = s.value("description", "");
        b.events.push_back(std::move(x));
    }
    return b;
}

BodyRegistry BodyRegistry::load_dir(std::string const& dir) {
    BodyRegistry r;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) { r.warnings.push_back("bodies dir missing: " + dir); return r; }
    std::vector<fs::path> files;
    for (auto const& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    for (auto const& f : files) {
        try { r.bodies.push_back(Body::load(f.string())); }
        catch (std::exception const& e) { r.warnings.push_back(e.what()); }
    }
    return r;
}

Body const* BodyRegistry::find(std::string const& id) const {
    for (auto const& b : bodies) if (b.id == id) return &b;
    return nullptr;
}

Body const* BodyRegistry::guess(std::string const& body_manifest, std::string const& env_target,
                                std::vector<std::string> const& action_topics) const {
    if (!body_manifest.empty()) if (auto const* b = find(body_manifest)) return b;
    std::vector<Body const*> cands;
    for (auto const& b : bodies) if (b.env_target == env_target) cands.push_back(&b);
    if (cands.empty()) return nullptr;
    if (cands.size() == 1) return cands.front();
    // Several manifests for one env: pick the one whose sinks the graph drives.
    Body const* best = cands.front();
    size_t best_hits = 0;
    for (auto const* b : cands) {
        size_t hits = 0;
        for (auto const& s : b->sinks)
            if (std::find(action_topics.begin(), action_topics.end(), s.topic) != action_topics.end()) ++hits;
        if (hits > best_hits) { best = b; best_hits = hits; }
    }
    return best;
}

} // namespace bb
