#include "Publish.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <regex>

#include "Layout.hpp"

namespace fs = std::filesystem;

namespace bb {

namespace {

std::string slugify(std::string const& s) {
    std::string out;
    for (unsigned char c : s) {
        if (std::isalnum(c)) out += char(std::tolower(c));
        else if (c == '_' || c == '-' || c == ' ' || c == '.') { if (!out.empty() && out.back() != '_') out += '_'; }
    }
    while (!out.empty() && out.back() == '_') out.pop_back();
    return out.empty() ? "graph" : out;
}

} // namespace

int next_series_number(std::string const& dir) {
    int n = 0;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return 1;
    static const std::regex re(R"(^a1v2_r(\d+))");
    for (auto const& e : fs::directory_iterator(dir)) {
        std::string name = e.path().filename().string();
        std::smatch m;
        if (std::regex_search(name, m, re)) n = std::max(n, std::stoi(m[1]));
    }
    return n + 1;
}

PublishPlan plan_publish(Graph const& g, Body const* body, std::string const& target,
                         std::string const& slug_in, std::string const& title,
                         std::string const& why, std::string const& phase_tag) {
    PublishPlan p;
    p.target     = target;
    p.phase_tag  = phase_tag;
    p.env_target = body ? body->env_target : g.env_target();
    std::string slug = slugify(slug_in.empty() ? title : slug_in);
    if (target == "mj_host") {
        p.dir      = BB_MJ_CONFIG_DIR;
        p.number   = next_series_number(p.dir);
        p.rank     = 1000 + p.number;
        p.filename = "a1v2_r" + std::to_string(p.number) + "_" + slug + ".json";
        p.name     = "R" + std::to_string(p.number) + " \xC2\xB7 " + title;
        p.description = "R" + std::to_string(p.number) + " (" + title + ")" + (why.empty() ? "" : ": " + why);
        if (p.env_target.empty()) p.env_target = "microduck";
        p.notes.push_back("the duck launcher lists it by launcher_rank; a preset with controls is still tools/duck_launcher/newtest.py's job");
    } else {
        p.dir      = BB_GODOT_CONFIG_DIR;
        p.filename = slug + ".json";
        p.name     = title;
        p.description = why;
        if (p.env_target.empty()) p.notes.push_back("no env_target: the Godot launcher skips configs without metadata.env_target");
        else if (p.env_target != "cell")
            p.notes.push_back("add \"" + p.filename + "\" to the " + p.env_target + " allowlist in godot_host/project/scripts/launcher.gd to see it in the launcher");
        else
            p.notes.push_back("the cell allowlist in launcher.gd, if non-empty, must name \"" + p.filename + "\"");
    }
    p.path = (fs::path(p.dir) / p.filename).string();
    std::error_code ec;
    if (fs::exists(p.path, ec)) p.notes.push_back("WILL OVERWRITE " + p.path);
    return p;
}

void apply_publish(Graph& g, PublishPlan const& plan) {
    g.checkpoint();
    auto& md = g.metadata();
    md["name"]       = plan.name;
    md["env_target"] = plan.env_target;
    if (!plan.phase_tag.empty()) md["phase_tag"] = plan.phase_tag;
    if (plan.target == "mj_host") md["launcher_rank"] = plan.rank;
    if (!plan.description.empty()) {
        g.doc["description"] = plan.description;
        if (plan.target == "godot") md["description"] = plan.description;
    }
    if (plan.target == "mj_host") g.ascii_escapes = true;   // newtest.py's style
    g.dirty = true;
    g.save(plan.path);
}

} // namespace bb
