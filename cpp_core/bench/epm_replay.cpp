// epm_replay — run recorded input vectors through a REAL EPM, offline (2026-10-24).
//
// Feeds one vector per line (whitespace-separated floats) as a ProprioToken into an EPM built from a JSON params object
// (exactly as a config would give it), one tick per vector, and prints one JSON line per vector: winner, node and baked
// counts, TLE, and the encoder latent.  For conditioning studies (CLAUDE.md §0 rule 2): is the separation in the input
// still there in what the GNG sees?
//
// Usage: epm_replay <vectors.txt> '<params json>'   e.g. '{"encoder_kind":"rbf","proprio_state_dims":26,...}'
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/EPM.hpp"

namespace {
ogma::ParamValue to_param(nlohmann::json const& v) {
    if (v.is_boolean()) return v.get<bool>();
    if (v.is_number_integer()) return int64_t(v.get<int64_t>());
    if (v.is_number()) return v.get<double>();
    if (v.is_string()) return v.get<std::string>();
    if (v.is_array() && !v.empty() && v[0].is_string()) return v.get<std::vector<std::string>>();
    if (v.is_array()) { std::vector<double> d; for (auto const& x : v) d.push_back(x.get<double>()); return d; }
    throw std::invalid_argument("unsupported param value: " + v.dump());
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) { std::cerr << "usage: epm_replay vectors.txt 'params-json'\n"; return 2; }
    auto pj = nlohmann::json::parse(argv[2]);
    ogma::ParamMap p;
    for (auto& [k, v] : pj.items()) p[k] = to_param(v);
    if (!p.count("input_topic")) p["input_topic"] = std::string("obs.replay");
    if (!p.count("modality_group")) p["modality_group"] = std::string("replay");
    if (!p.count("modality_name")) p["modality_name"] = std::string("x");
    const std::string in_topic = std::get<std::string>(p["input_topic"]);
    const std::string out_topic = "reality." + std::get<std::string>(p["modality_group"]) + "." + std::get<std::string>(p["modality_name"]);
    ogma::InProcessBus bus;
    ogma::EPM epm; epm.set_id("replay");
    epm.on_setup(&bus, p);
    std::ifstream f(argv[1]);
    std::string line;
    uint64_t t = 0;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::vector<float> v; float x;
        while (ss >> x) v.push_back(x);
        if (v.empty()) continue;
        ++t;
        auto tok = std::make_shared<ogma::ProprioToken>();
        tok->tick_id = t; tok->producer_id = "host"; tok->sensor = "replay";
        tok->values = Eigen::Map<Eigen::VectorXf>(v.data(), Eigen::Index(v.size()));
        bus.begin_tick(t);
        bus.publish(in_topic, tok);
        epm.tick(t);
        auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(bus.last_value(out_topic));
        bus.end_tick();
        if (!rt) { std::cout << "{\"t\":" << t << ",\"winner\":-1}\n"; continue; }
        nlohmann::json o{{"t", t}, {"winner", rt->winner_id}, {"nodes", rt->node_count}, {"baked", rt->baked_count},
                         {"tle", rt->tle}, {"latent", std::vector<double>(rt->latent.data(), rt->latent.data() + rt->latent.size())}};
        std::cout << o.dump() << "\n";
    }
    return 0;
}
