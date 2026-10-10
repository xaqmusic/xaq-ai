#pragma once
// --gen-palette: derive each type's socket templates by probing.  For every
// string / list-of-string param, set a probe value, re-run trial setup, and
// diff the ports: a topic that appears containing the probe is a socket of
// that param.  Runs offline; the reviewed result lives in palette.json.
#include <iosfwd>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "Catalogue.hpp"

namespace bb {

// Returns {"types": {<type>: {"baseline": ..., "sockets": [...], "fixed": [...]}}}.
nlohmann::ordered_json gen_palette(Catalogue const& cat,
                                   std::vector<std::string> const& config_dirs,
                                   std::ostream& log);

// Write the generated sockets/fixed/baseline into palette.json, keeping every
// hand-authored key (category, layer, purpose, ...).
void merge_palette(std::string const& palette_path, nlohmann::ordered_json const& gen);

} // namespace bb
