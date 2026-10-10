#pragma once
// Publish: write the graph where a launcher will find it, named and numbered
// the way that launcher expects.  mj_host: tools/duck_launcher/newtest.py's
// rule (a1v2_r<nn>_<slug>.json, "R<nn> · <title>", rank 1000+nn).  Godot:
// <slug>.json with the metadata launcher.gd's _read_metadata requires, plus
// the reminder that its per-body allowlist is a one-line GDScript edit.
#include <string>
#include <vector>

#include "Body.hpp"
#include "Graph.hpp"

namespace bb {

struct PublishPlan {
    std::string target;        // "mj_host" | "godot"
    std::string dir, filename, path;
    std::string name;          // metadata.name
    std::string description;   // top-level description
    std::string env_target, phase_tag;
    int         number = 0;    // R<nn> (mj_host)
    int         rank   = 0;    // metadata.launcher_rank (mj_host)
    std::vector<std::string> notes;
};

int next_series_number(std::string const& mj_config_dir);

PublishPlan plan_publish(Graph const& g, Body const* body, std::string const& target,
                         std::string const& slug, std::string const& title,
                         std::string const& why, std::string const& phase_tag);

// Writes the plan's metadata into the document (one undo step) and saves it
// to plan.path.  Throws std::runtime_error on I/O failure.
void apply_publish(Graph& g, PublishPlan const& plan);

} // namespace bb
