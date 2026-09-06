// =============================================================================
// test_schema_defaults_effective.cpp -- a schema default must be the effective default
// =============================================================================
//
// Cell system audit, 2026-09-06.  A module's params_schema() advertises a
// default for each key; the value the module actually runs with when a config
// OMITS the key is whatever its constructor / on_setup chose.  The two can
// drift: EPM advertised baking_threshold 50 while the GNG's own default of 100
// ran in the 105 EPM instances whose configs omitted the key (the "bake
// threshold trap", Kalman-lessons Stage 0), and SequenceGNG carries the same
// pair.  This test constructs every registered module with EMPTY params, reads
// back current_params(), and compares each reported key with its schema
// default.  Known traps are PINNED (they must still mismatch until the merge
// lever lands); any NEW mismatch fails.  Modules that report no current_params,
// or that require params, are listed as unverifiable rather than skipped silently.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <variant>
#include <vector>

#include "ogma/InProcessBus.hpp"
#include "ogma/Module.hpp"

namespace {

// "<Type>.<key>" pairs whose mismatch is a documented DEFAULT_TRAP awaiting its
// lever (aligning an effective default with its schema changes every config that
// omits the key, so it is never done silently).  Register: the cell system audit
// appendix.  A pinned pair that stops mismatching is reported so the pin can go.
//
// Note on EPM / SequenceGNG: their current_params() report the STORED param
// (schema 50), not the GNG's effective 100 when the key is omitted -- the trap is
// one level down and this test cannot see it; the bench pins it instead
// (epm_kalman_lessons_plan.md, Stage 0).  Listed here so the limitation is explicit.
const std::set<std::string> kKnownTraps = {
    "EPM.baking_threshold",            // not observable here (see note)
    "SequenceGNG.baking_threshold",    // not observable here (see note)
    "WhiskerSteerReflex.steer_gain",   // schema 8, effective 10      (audit 2026-09-06)
    "PlaceNav.arrival_window",         // schema 30, effective 60     (audit 2026-09-06)
    "SaccadeReflex.scent_gate",        // schema 0.05, effective 1e9  (audit 2026-09-06)
};

bool numeric(ogma::ParamValue const& v) {
    return std::holds_alternative<bool>(v) || std::holds_alternative<int64_t>(v) || std::holds_alternative<double>(v);
}
double as_double(ogma::ParamValue const& v) {
    if (auto p = std::get_if<bool>(&v))    return *p ? 1.0 : 0.0;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    if (auto p = std::get_if<double>(&v))  return *p;
    return std::nan("");
}
std::string show(ogma::ParamValue const& v) {
    if (auto p = std::get_if<bool>(&v))        return *p ? "true" : "false";
    if (auto p = std::get_if<int64_t>(&v))     return std::to_string(*p);
    if (auto p = std::get_if<double>(&v))      return std::to_string(*p);
    if (auto p = std::get_if<std::string>(&v)) return "\"" + *p + "\"";
    if (auto p = std::get_if<std::vector<double>>(&v)) return "[" + std::to_string(p->size()) + " doubles]";
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return "[" + std::to_string(p->size()) + " strings]";
    return "?";
}
bool same(ogma::ParamValue const& a, ogma::ParamValue const& b) {
    if (numeric(a) && numeric(b)) {
        const double x = as_double(a), y = as_double(b);
        return std::fabs(x - y) <= 2e-6 * std::max(1.0, std::fabs(x));   // float32 round-trip tolerance
    }
    return a == b;
}

} // namespace

TEST(SchemaDefaultsEffective, EveryReportedDefaultMatchesItsSchema) {
    auto& reg = ogma::ModuleRegistry::instance();
    int n_checked = 0, n_unverifiable = 0, n_required = 0;
    std::vector<std::string> new_mismatches, pinned_seen;

    for (auto const& type : reg.registered_types()) {
        ogma::ModulePtr m;
        try { m = reg.create(type); } catch (std::exception const&) { continue; }
        auto schema = m->params_schema();
        bool has_required = false;
        for (auto const& ps : schema) if (!ps.default_value) { has_required = true; break; }
        if (has_required) { ++n_required; std::printf("  required-params  %s\n", type.c_str()); continue; }

        ogma::InProcessBus bus;
        m->set_id("t_" + type);
        try { m->on_setup(&bus, {}); }
        catch (std::exception const& ex) { ++n_unverifiable; std::printf("  setup-threw      %s: %s\n", type.c_str(), ex.what()); continue; }

        auto cur = m->current_params();
        if (cur.empty()) { ++n_unverifiable; std::printf("  no-current       %s\n", type.c_str()); continue; }

        for (auto const& ps : schema) {
            auto it = cur.find(ps.key);
            if (it == cur.end()) continue;
            ++n_checked;
            if (same(it->second, *ps.default_value)) continue;
            const std::string tag = type + "." + ps.key;
            std::printf("  MISMATCH %-8s %-40s schema %s  effective %s\n",
                        kKnownTraps.count(tag) ? "(pinned)" : "(NEW)", tag.c_str(),
                        show(*ps.default_value).c_str(), show(it->second).c_str());
            (kKnownTraps.count(tag) ? pinned_seen : new_mismatches).push_back(tag);
        }
        try { m->on_teardown(); } catch (...) {}
    }
    std::printf("schema defaults: %d keys checked, %d modules unverifiable, %d with required params\n",
                n_checked, n_unverifiable, n_required);

    EXPECT_TRUE(new_mismatches.empty()) << "new schema/effective default mismatches: "
        << [&]{ std::string s; for (auto& t : new_mismatches) s += t + " "; return s; }();
    for (auto const& t : kKnownTraps)
        if (std::find(pinned_seen.begin(), pinned_seen.end(), t) == pinned_seen.end())
            std::printf("  pinned-not-seen  %s (fixed, or not observable through current_params)\n", t.c_str());
}
