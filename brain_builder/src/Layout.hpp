#pragma once
// Node positions: computed by depth from the body's sources when a config
// has none, and stored in the config under metadata.builder.layout so a
// layout travels with the file.  Launchers and the core ignore the key.
#include <string>
#include <unordered_map>

#include "Catalogue.hpp"
#include "Graph.hpp"
#include "Wiring.hpp"

namespace bb {

struct Pos { float x = 0, y = 0; };
using LayoutMap = std::unordered_map<std::string, Pos>;
using SizeMap   = std::unordered_map<std::string, Pos>;   // measured node sizes (w, h)

LayoutMap auto_layout(Wiring const& w, Catalogue const& cat, float scale = 1.0f, SizeMap const* sizes = nullptr);
LayoutMap read_layout(Graph const& g);
bool      write_layout(Graph& g, LayoutMap const& pos);   // true if the document changed

} // namespace bb
