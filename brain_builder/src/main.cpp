// brain_builder — a Dear ImGui app for wiring a brain from scratch.
//
//   brain_builder [config.json]                  open the builder
//   brain_builder --dump-catalogue               module catalogue as JSON, then exit
//   brain_builder --list-types                   registered module types, then exit
//   brain_builder --gen-palette [--merge] [--out F] [--config-dir D]...
//                                                probe sockets; print (or merge into palette.json)
//   --palette P                                  use another palette.json
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Catalogue.hpp"
#include "PaletteGen.hpp"
#include "TrialSetup.hpp"
#include "ogma/Module.hpp"
#include "ui/App.hpp"

namespace {

void usage() {
    std::cerr <<
        "usage: brain_builder [config.json] [--palette P]\n"
        "       brain_builder --dump-catalogue [--palette P]\n"
        "       brain_builder --list-types\n"
        "       brain_builder --gen-palette [--merge] [--out F] [--config-dir D]... [--palette P]\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string palette = BB_PALETTE;
    std::string config_path, out_path;
    std::vector<std::string> config_dirs;
    bool dump = false, list = false, gen = false, merge = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](std::string& dst) {
            if (i + 1 >= argc) { usage(); std::exit(2); }
            dst = argv[++i];
        };
        if      (a == "--dump-catalogue") dump = true;
        else if (a == "--list-types")     list = true;
        else if (a == "--gen-palette")    gen = true;
        else if (a == "--merge")          merge = true;
        else if (a == "--palette")        next(palette);
        else if (a == "--out")            next(out_path);
        else if (a == "--config-dir")     { std::string d; next(d); config_dirs.push_back(d); }
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else if (!a.empty() && a[0] == '-') { std::cerr << "unknown option " << a << "\n"; usage(); return 2; }
        else config_path = a;
    }

    if (list) {
        bb::warm_registry();
        auto names = ogma::ModuleRegistry::instance().registered_types();
        std::sort(names.begin(), names.end());
        for (auto const& n : names) std::cout << n << "\n";
        return 0;
    }

    bb::Catalogue cat = bb::Catalogue::build(palette);

    if (dump) {
        std::cout << cat.to_json().dump(2) << "\n";
        return 0;
    }

    if (gen) {
        if (config_dirs.empty()) config_dirs = {BB_MJ_CONFIG_DIR, BB_GODOT_CONFIG_DIR};
        nlohmann::ordered_json g = bb::gen_palette(cat, config_dirs, std::cerr);
        if (merge) {
            bb::merge_palette(palette, g);
            std::cerr << "merged into " << palette << "\n";
        } else if (!out_path.empty()) {
            std::ofstream f(out_path);
            f << g.dump(2) << "\n";
            std::cerr << "wrote " << out_path << "\n";
        } else {
            std::cout << g.dump(2) << "\n";
        }
        return 0;
    }

    bb::AppState st;
    st.catalogue   = std::move(cat);
    st.config_path = config_path;
    for (auto const& w : st.catalogue.warnings) st.logf("catalogue: " + w);
    return bb::run_app(st);
}
