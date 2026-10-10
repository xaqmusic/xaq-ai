// =============================================================================
// test_shipping_config_schema.cpp -- every config key must be in its module's schema
// =============================================================================
//
// Cell system audit, 2026-09-06.  GraphConfig copies a module's "params" block
// verbatim and on_setup reads only the keys its schema declares, so a key the
// schema does not know is DROPPED SILENTLY.  Seven such keys (hunger_gate,
// progress_gate, route_eps, short_alpha, long_alpha, scent_max_topic,
// scent_bearing_topic) sat on the Cell planner's block in three study configs
// and read as live mechanism.  This test walks every shipping config, reports
// the unknown keys per module, and ASSERTS zero unknown keys for the configs in
// kAssertClean.  Add a config to that set once its block is clean; never remove
// one.  Keys starting with '_' (e.g. _comment) are documentation and ignored.
//
// Run from cpp_core/build (the config dir is located relative to the cwd).

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ogma/GraphConfig.hpp"
#include "ogma/Module.hpp"

namespace fs = std::filesystem;

namespace {

fs::path locate_configs_dir() {
    auto candidates = {
        fs::path("../godot_host/project/addons/ami_ogma/configs"),
        fs::path("../../godot_host/project/addons/ami_ogma/configs"),
        fs::path("../../../godot_host/project/addons/ami_ogma/configs"),
        fs::path("godot_host/project/addons/ami_ogma/configs"),
    };
    for (auto const& c : candidates)
        if (fs::exists(c) && fs::is_directory(c)) return fs::canonical(c);
    return {};
}

// Configs asserted clean.  Start with the ones the audit's config pass has
// cleaned; the report below shows which others still carry dropped keys.
const std::set<std::string> kAssertClean = {
    "the_cell_chemotaxis_baseline.json",
    "the_cell_maze_fusion__dims3.json",
};

struct Unknown { std::string config, module, type, key; };

std::vector<Unknown> unknown_keys(fs::path const& file) {
    std::vector<Unknown> out;
    auto cfg = ogma::GraphConfig::load_from_file(file.string());
    auto& reg = ogma::ModuleRegistry::instance();
    for (auto const& spec : cfg.modules) {
        ogma::ModulePtr m;
        try { m = reg.create(spec.type); }
        catch (std::exception const&) {
            out.push_back({file.filename().string(), spec.id, spec.type, "<unknown module type>"});
            continue;
        }
        auto schema = m->params_schema();
        for (auto const& [k, v] : spec.params) {
            if (!k.empty() && k[0] == '_') continue;
            bool known = false;
            for (auto const& ps : schema) if (ps.key == k) { known = true; break; }
            if (!known) out.push_back({file.filename().string(), spec.id, spec.type, k});
        }
    }
    return out;
}

} // namespace

TEST(ShippingConfigSchema, UnknownParamsAreReportedAndAssertedOnCleanSet) {
    auto dir = locate_configs_dir();
    ASSERT_FALSE(dir.empty()) << "configs dir not found; run from cpp_core/build";

    std::map<std::string, std::vector<Unknown>> by_config;
    int n_configs = 0;
    for (auto const& e : fs::directory_iterator(dir)) {
        if (!e.is_regular_file() || e.path().extension() != ".json") continue;
        ++n_configs;
        std::vector<Unknown> u;
        try { u = unknown_keys(e.path()); }
        catch (std::exception const& ex) {
            u.push_back({e.path().filename().string(), "", "", std::string("<load failed: ") + ex.what() + ">"});
        }
        if (!u.empty()) by_config[e.path().filename().string()] = std::move(u);
    }
    int n_keys = 0;
    for (auto const& [cfg, us] : by_config) {
        for (auto const& u : us) {
            ++n_keys;
            std::printf("  DROPPED  %-60s %-24s %-22s %s\n", cfg.c_str(), u.module.c_str(), u.type.c_str(), u.key.c_str());
        }
    }
    std::printf("shipping configs: %d scanned, %zu with dropped keys, %d dropped keys in all\n",
                n_configs, by_config.size(), n_keys);

    for (auto const& name : kAssertClean) {
        auto it = by_config.find(name);
        EXPECT_TRUE(it == by_config.end())
            << name << " is in kAssertClean but carries " << (it == by_config.end() ? 0 : it->second.size())
            << " dropped key(s)";
    }
}
