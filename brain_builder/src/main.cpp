// brain_builder — a Dear ImGui app for wiring a brain from scratch.
//
//   brain_builder [config.json]                  open the builder
//   brain_builder --dump-catalogue               module catalogue as JSON, then exit
//   brain_builder --list-types                   registered module types, then exit
//   brain_builder --gen-palette [--merge] [--out F] [--config-dir D]...
//                                                probe sockets; print (or merge into palette.json)
//   --palette P                                  use another palette.json
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Body.hpp"
#include "Catalogue.hpp"
#include "DryRun.hpp"
#include "Graph.hpp"
#include "LiveClient.hpp"
#include "Order.hpp"
#include "Publish.hpp"
#include "PaletteGen.hpp"
#include "TrialSetup.hpp"
#include "Wiring.hpp"
#include "ogma/Module.hpp"
#include "ui/App.hpp"

namespace {

void usage() {
    std::cerr <<
        "usage: brain_builder [config.json] [--palette P]\n"
        "       brain_builder --dump-catalogue [--palette P]\n"
        "       brain_builder --list-types\n"
        "       brain_builder --gen-palette [--merge] [--out F] [--config-dir D]... [--palette P]\n"
        "       brain_builder --ports config.json [--body B]      every module's trial-setup ports\n"
        "       brain_builder --validate config.json [--body B]   wiring diagnostics; exit 1 on errors\n"
        "       brain_builder --dry-run config.json [--ticks N] [--body B]   construct, feed synthetic input, tick\n"
        "       brain_builder --roundtrip in.json out.json            load and save through the document model\n"
        "       brain_builder --publish-dry config.json --title T [--slug S] [--target mj_host|godot]   show the publish plan\n"
        "       brain_builder --connect host:port [--pull out.json] [--patch ops.json]   talk to a running brain\n"
        "       brain_builder --live [host:port]                       open the builder connected to a running brain\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string palette = BB_PALETTE;
    std::string config_path, out_path;
    std::vector<std::string> config_dirs;
    std::string body_id, select_id;
    bool dump = false, list = false, gen = false, merge = false, ports = false, validate = false, dryrun = false, roundtrip = false, pubdry = false;
    std::string pub_title, pub_slug, pub_target = "mj_host";
    std::string connect_to, pull_to, patch_file, live_to;
    bool live = false;
    std::vector<std::string> positional;
    int ticks = 50;

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
        else if (a == "--ports")          ports = true;
        else if (a == "--validate")       validate = true;
        else if (a == "--body")           next(body_id);
        else if (a == "--select")         next(select_id);
        else if (a == "--dry-run")        dryrun = true;
        else if (a == "--roundtrip")      roundtrip = true;
        else if (a == "--publish-dry")    pubdry = true;
        else if (a == "--title")          next(pub_title);
        else if (a == "--slug")           next(pub_slug);
        else if (a == "--target")         next(pub_target);
        else if (a == "--connect")        next(connect_to);
        else if (a == "--live")           { live = true; if (i + 1 < argc && argv[i + 1][0] != '-') live_to = argv[++i]; }
        else if (a == "--pull")           next(pull_to);
        else if (a == "--patch")          next(patch_file);
        else if (a == "--ticks")          { std::string t; next(t); ticks = std::stoi(t); }
        else if (a == "--palette")        next(palette);
        else if (a == "--out")            next(out_path);
        else if (a == "--config-dir")     { std::string d; next(d); config_dirs.push_back(d); }
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else if (!a.empty() && a[0] == '-') { std::cerr << "unknown option " << a << "\n"; usage(); return 2; }
        else { positional.push_back(a); config_path = positional.front(); }
    }

    if (!connect_to.empty()) {
        std::string host; uint16_t port = 7400;
        if (!bb::parse_endpoint(connect_to, host, port)) { std::cerr << "bad endpoint\n"; return 2; }
        bb::LiveClient c;
        std::string err;
        if (!c.connect(host, port, &err)) { std::cerr << err << "\n"; return 1; }
        nlohmann::json g = c.call({{"verb", "get_graph"}});
        if (g.value("status", "") != "ok") { std::cerr << "get_graph: " << g.value("message", std::string("?")) << "\n"; return 1; }
        std::cout << "connected " << c.endpoint() << ": graph v" << g.value("graph_version", int64_t(0)) << ", "
                  << g.value("module_count", size_t(0)) << " modules, source " << g.value("source_path", std::string("?")) << "\n";
        for (auto const& m : g["config"]["modules"]) std::cout << "  " << m.value("id", "") << " (" << m.value("type", "") << ")\n";
        if (!pull_to.empty()) {
            std::ofstream f(pull_to);
            f << g["config"].dump(2) << "\n";
            std::cout << "wrote " << pull_to << "\n";
        }
        if (!patch_file.empty()) {
            std::ifstream f(patch_file);
            nlohmann::json ops = nlohmann::json::parse(f, nullptr, false);
            if (ops.is_discarded()) { std::cerr << "bad ops file\n"; return 2; }
            nlohmann::json r = c.call({{"verb", "apply_patch"}, {"ops", ops}, {"source", "builder-cli"}});
            std::cout << "apply_patch: " << r.dump() << "\n";
            if (r.value("status", "") != "ok") return 1;
        }
        return 0;
    }

    if (roundtrip) {
        if (positional.size() != 2) { usage(); return 2; }
        try {
            bb::Graph g = bb::Graph::load(positional[0]);
            g.save(positional[1]);
            std::cout << "wrote " << positional[1] << " (" << g.size() << " modules" << (g.ascii_escapes ? ", ascii escapes" : "") << ")\n";
        } catch (std::exception const& e) { std::cerr << e.what() << "\n"; return 1; }
        return 0;
    }

    // A config named relative to the repo root works from any directory.
    auto resolve = [](std::string& p) {
        if (p.empty() || std::filesystem::exists(p)) return;
        std::filesystem::path alt = std::filesystem::path(BB_ROOT) / p;
        if (std::filesystem::exists(alt)) p = alt.string();
    };
    resolve(config_path);
    for (auto& p : positional) resolve(p);

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

    bb::BodyRegistry bodies = bb::BodyRegistry::load_dir(BB_BODIES_DIR);

    if (pubdry) {
        if (config_path.empty() || pub_title.empty()) { usage(); return 2; }
        bb::Graph g;
        try { g = bb::Graph::load(config_path); } catch (std::exception const& e) { std::cerr << e.what() << "\n"; return 2; }
        bb::Body const* body = bodies.guess(g.body_manifest(), g.env_target(), {});
        bb::PublishPlan p = bb::plan_publish(g, body, pub_target, pub_slug, pub_title, "", "");
        std::cout << "file  " << p.path << "\nname  " << p.name << "\nrank  " << p.rank << "\nenv   " << p.env_target << "\n";
        for (auto const& n : p.notes) std::cout << "note  " << n << "\n";
        return 0;
    }

    if (ports || validate || dryrun) {
        if (config_path.empty()) { usage(); return 2; }
        bb::Graph g;
        try { g = bb::Graph::load(config_path); }
        catch (std::exception const& e) { std::cerr << e.what() << "\n"; return 2; }
        std::vector<std::string> actions;
        for (size_t i = 0; i < g.size(); ++i)
            if (g.params(i).contains("action_topics") && g.params(i)["action_topics"].is_array())
                for (auto const& t : g.params(i)["action_topics"]) if (t.is_string()) actions.push_back(t.get<std::string>());
        bb::Body const* body = body_id.empty() ? bodies.guess(g.body_manifest(), g.env_target(), actions) : bodies.find(body_id);
        if (!body_id.empty() && !body) { std::cerr << "unknown body '" << body_id << "'\n"; return 2; }
        bb::TrialCache cache;
        bb::Wiring w;
        { bb::StdoutSilencer quiet; w = bb::Wiring::build(g, cat, body, cache); }
        std::cout << config_path << ": " << g.size() << " modules, body " << (body ? body->id : "none") << "\n";
        if (ports) {
            for (auto const& n : w.nodes) {
                std::cout << (n.kind == bb::NodeKind::Module ? "  " : "  BODY ") << n.name << " (" << n.type << ")"
                          << (n.setup_ok ? "" : "  SETUP FAILED: " + n.setup_error) << "\n";
                for (auto const& p : n.inputs)  std::cout << "     in  " << (p.placeholder ? "[unset] " + p.label : p.topic) << "  " << p.payload << (p.feedback ? " feedback" : "") << (p.required ? "" : " optional") << "\n";
                for (auto const& p : n.outputs) std::cout << "     out " << (p.placeholder ? "[unset] " + p.label : p.topic) << "  " << p.payload << "\n";
            }
            std::cout << "  links: " << w.links.size() << "\n";
            for (auto const& l : w.links) std::cout << "     " << l.from_node << " -> " << l.to_node << "  " << l.topic << (l.feedback ? "  (feedback)" : "") << (l.type_ok ? "" : "  PAYLOAD MISMATCH") << "\n";
        }
        for (auto const& d : w.diagnostics)
            std::cout << "  " << (d.severity == bb::Diagnostic::Error ? "ERROR  " : d.severity == bb::Diagnostic::Warning ? "warning" : "info   ")
                      << "  " << d.node << ": " << d.message << "\n";
        std::cout << "  " << w.errors << " errors, " << w.warnings << " warnings\n";
        if (dryrun) {
            bb::DryRunReport r;
            { bb::StdoutSilencer quiet; r = bb::dry_run(g, w, body, ticks); }
            for (auto const& f : r.fed) std::cout << "  fed       " << f << "\n";
            if (!r.constructed) { std::cout << "  FAILED: " << r.error << "\n"; return 1; }
            std::cout << "  constructed in " << r.construct_ms << " ms; " << r.ticks_done << " ticks in " << r.tick_ms << " ms\n";
            if (!r.error.empty()) std::cout << "  FAILED: " << r.error << "\n";
            for (auto const& t : r.published)       std::cout << "  published " << t << "\n";
            for (auto const& t : r.silent)          std::cout << "  SILENT    " << t << "\n";
            for (auto const& t : r.actions_seen)    std::cout << "  driven    " << t << "\n";
            for (auto const& t : r.actions_missing) std::cout << "  NO VALUE  " << t << " (body sink)\n";
            return r.error.empty() ? 0 : 1;
        }
        return (validate && w.errors) ? 1 : 0;
    }

    bb::AppState st;
    st.catalogue   = std::move(cat);
    st.bodies      = std::move(bodies);
    st.selected    = select_id;
    if (live) { st.live_on_start = true; st.live.dialog_endpoint = live_to.empty() ? "127.0.0.1:7400" : live_to; }
    st.config_path = config_path;
    for (auto const& w : st.catalogue.warnings) st.logf("catalogue: " + w);
    return bb::run_app(st);
}
