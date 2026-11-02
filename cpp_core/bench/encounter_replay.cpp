// encounter_replay -- a bench palpation session through the REAL encounter stack, offline (palpation stage 1, 2026-11-02).
//
// Reads benchd's record (adc_fast rows at 50 Hz with the FSR counts and the twelve pulses; telemetry at 10 Hz with the fused
// up vector, the bus current and the belly ToF; the palpate marks), rebuilds the brain's inputs with the robot host's own
// kinematics (ServoMapping + BodyCalib: the same FK ogma_host runs), and drives EncounterFeatures -> EPM -> BeliefFilter
// tick by tick on an InProcessBus.  One JSON line per encounter: the feature vector, the EPM's winner / tle / novelty, the
// belief's MAP and entropy, and the mark's labels (condition, probed leg) -- the sensor-admission gate: does the vocabulary
// the GNG earns separate what the operator placed, measured against the labels it never saw?
//
//   encounter_replay <servo_map.json> <body_calib.json> <toe_height_calib.json|-> <benchd.jsonl>... > events.jsonl
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include "ogma/InProcessBus.hpp"
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"
#include "ogma/hw/BrainInputs.hpp"

using json = nlohmann::json;
using namespace ogma;

static std::shared_ptr<ProprioToken> tok(std::vector<float> v, uint64_t t) {
    auto p = std::make_shared<ProprioToken>(); p->tick_id = t; p->producer_id = "host"; p->values.resize(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) p->values[i] = v[size_t(i)];
    return p;
}
static const std::array<int, 4> kFsrChForSim = {1, 0, 3, 2};          // sim leg i reads ADC channel (physical FL FR RL RR = A0..A3)
static const char* kPhysOfSim[4] = {"FR", "FL", "RR", "RL"};
static double grams(double c) {
    static const double cv[][2] = {{60, 0}, {1536, 30}, {1920, 50}, {2118, 100}, {2362, 175}, {2603, 300}, {2925, 500}};
    if (c <= cv[0][0]) return 0.0;
    for (int i = 0; i + 1 < 7; ++i) if (c <= cv[i + 1][0]) return cv[i][1] + (cv[i + 1][1] - cv[i][1]) * (c - cv[i][0]) / (cv[i + 1][0] - cv[i][0]);
    return cv[6][1] + (cv[6][1] - cv[5][1]) * (c - cv[6][0]) / (cv[6][0] - cv[5][0]);
}

int main(int argc, char** argv) {
    if (argc < 5) { std::cerr << "usage: encounter_replay servo_map.json body_calib.json toe_height_calib.json|- benchd.jsonl...\n"; return 2; }
    const auto map  = ogma::hw::brain::ServoMapping::load(argv[1]);
    const auto body = ogma::hw::brain::BodyCalib::load(argv[2]);
    if (!map.complete) { std::cerr << "servo map: " << map.why << "\n"; return 1; }
    if (!body.ok)      { std::cerr << "body: " << body.why << "\n"; return 1; }
    // the floor calibration, applied where the host applies it: on the toe tip's height (reality.proprio.toe_y)
    ogma::hw::brain::ToeCalib tcal;
    if (std::string(argv[3]) != "-") {
        tcal = ogma::hw::brain::ToeCalib::load(argv[3]);
        if (!tcal.loaded) { std::cerr << "toe calib: " << tcal.why << "\n"; return 1; }
    }
    auto& reg = ModuleRegistry::instance();
    InProcessBus bus;
    ParamMap enc_p = {{"feet_y_topic", ParamValue{std::string("reality.proprio.toe_y")}}};
    // optional overrides anywhere after the calibration: enc.<param>=<value> / epm.<param>=<value> (number, true/false,
    // or a comma list of numbers); the remaining arguments are the records
    std::vector<std::string> records; std::vector<std::pair<std::string, std::string>> epm_over;
    auto parse_value = [](std::string const& v) -> ParamValue {
        if (v == "true") return ParamValue{true}; if (v == "false") return ParamValue{false};
        if (v.find(',') != std::string::npos) { std::vector<double> xs; size_t p = 0; while (p <= v.size()) { size_t q = v.find(',', p); if (q == std::string::npos) q = v.size(); xs.push_back(std::atof(v.substr(p, q - p).c_str())); p = q + 1; } return ParamValue{xs}; }
        if (v.find_first_not_of("-0123456789") == std::string::npos) return ParamValue{int64_t(std::atoll(v.c_str()))};
        return ParamValue{std::atof(v.c_str())};
    };
    for (int ai = 4; ai < argc; ++ai) {
        const std::string a = argv[ai]; const size_t eq = a.find('=');
        if (eq != std::string::npos && (a.rfind("enc.", 0) == 0 || a.rfind("epm.", 0) == 0)) {
            const std::string key = a.substr(4, eq - 4), val = a.substr(eq + 1);
            if (a[1] == 'n') enc_p[key] = parse_value(val); else epm_over.emplace_back(key, val);
        } else records.push_back(a);
    }
    ParamMap epm_p = {
        {"modality_group", ParamValue{std::string("touch")}}, {"modality_name", ParamValue{std::string("encounter")}},
        {"encoder_kind", ParamValue{std::string("rbf")}}, {"input_topic", ParamValue{std::string("sense.touch_encounter")}},
        {"projection_dim", ParamValue{int64_t{48}}}, {"proprio_state_dims", ParamValue{int64_t{10}}}, {"max_nodes", ParamValue{int64_t{32}}},
        {"baking_threshold", ParamValue{int64_t{3}}}, {"min_insertion_error", ParamValue{0.12}}, {"subtract_descending_prediction", ParamValue{false}},
        {"stale_prune_enabled", ParamValue{false}},      // an event-rate EPM: minutes pass between touches, and the prune counts ticks
        {"dim_min", ParamValue{std::vector<double>{-0.03, -0.10, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0}}},
        {"dim_max", ParamValue{std::vector<double>{ 0.09,  0.25,  0.25, 0.6, 0.8, 0.8, 0.03, 0.4, 1.5, 1.0}}}};
    for (auto const& [k, v] : epm_over) epm_p[k] = parse_value(v);
    ParamMap bel_p = {{"reality_topic", ParamValue{std::string("reality.touch.encounter")}}, {"obs_topic", ParamValue{std::string("sense.touch_encounter")}},
                      {"min_count", ParamValue{int64_t{2}}}};
    auto enc = reg.create("EncounterFeatures"); enc->set_id("encounter"); enc->on_setup(&bus, enc_p);
    auto epm = reg.create("EPM"); epm->set_id("epm_encounter"); epm->on_setup(&bus, epm_p);
    auto bel = reg.create("BeliefFilter"); bel->set_id("belief_encounter"); bel->on_setup(&bus, bel_p);

    const double us_per_rad = 545.2;
    uint64_t tick = 0; long events = 0;
    for (size_t ri = 0; ri < records.size(); ++ri) {
        const char* rec = records[ri].c_str();
        // pass 1: telemetry and marks in time order; pass 2: the fast rows drive the ticks
        struct Tele { long t; ogma::body::Vec3f up; double i_a; double tof; bool tof_ok; };
        std::vector<Tele> tele; std::vector<std::pair<long, json>> marks;
        {
            std::ifstream f(rec); std::string line;
            while (std::getline(f, line)) {
                if (line.find("\"kind\":\"telemetry\"") != std::string::npos) {
                    json j; try { j = json::parse(line); } catch (...) { continue; }
                    auto const& d = j["data"]; Tele te{j.value("t_mono_ms", 0L), ogma::body::Vec3f(0, 1, 0), 0.0, 0.0, false};
                    if (d.contains("imu") && d["imu"].contains("up_fused") && d["imu"]["up_fused"].is_array() && d["imu"]["up_fused"].size() >= 3)
                        te.up = ogma::body::Vec3f(d["imu"]["up_fused"][0].get<float>(), d["imu"]["up_fused"][1].get<float>(), d["imu"]["up_fused"][2].get<float>());
                    if (d.contains("ina") && d["ina"].contains("i_a")) te.i_a = d["ina"]["i_a"].get<double>();
                    if (d.contains("tof") && d["tof"].value("valid", false)) { te.tof = d["tof"].value("m", 0.0); te.tof_ok = true; }
                    tele.push_back(te);
                } else if (line.find("\"kind\":\"mark\"") != std::string::npos) {
                    json j; try { j = json::parse(line); } catch (...) { continue; }
                    json kv; std::string text = j["data"].value("text", ""); size_t pos = 0;
                    while (pos < text.size()) {
                        size_t sp = text.find(' ', pos); if (sp == std::string::npos) sp = text.size();
                        std::string w = text.substr(pos, sp - pos); size_t eq = w.find('=');
                        if (eq != std::string::npos) kv[w.substr(0, eq)] = w.substr(eq + 1);
                        pos = sp + 1;
                    }
                    marks.emplace_back(j.value("t_mono_ms", 0L), kv);
                }
            }
        }
        size_t ti = 0, mi = 0; json cur_mark = json::object(); long seen_events = enc->diag_snapshot().value("events", 0L);
        std::ifstream f(rec); std::string line;
        while (std::getline(f, line)) {
            if (line.find("\"kind\":\"adc_fast\"") == std::string::npos) continue;
            json j; try { j = json::parse(line); } catch (...) { continue; }
            auto const& d = j["data"]; if (!d.contains("a") || !d.contains("pulse")) continue;
            const long t = j.value("t_mono_ms", 0L);
            while (ti + 1 < tele.size() && tele[ti + 1].t <= t) ++ti;
            while (mi < marks.size() && marks[mi].first <= t) { cur_mark = marks[mi].second; ++mi; }
            if (tele.empty()) continue;
            const Tele& te = tele[ti];
            std::array<int, 12> us{}; for (int c = 0; c < 12; ++c) us[size_t(c)] = d["pulse"][size_t(c)].get<int>();
            if (!ogma::hw::brain::all_servos_commanded(us, map)) continue;
            const auto ang = ogma::hw::brain::hinge_angles_from_us(us, map, us_per_rad);
            std::vector<float> contact(4), load(4), xz(8), y(4);
            for (int i = 0; i < 4; ++i) {
                const double c = d["a"][size_t(kFsrChForSim[size_t(i)])].get<double>();
                contact[size_t(i)] = c >= 200 ? 1.0f : 0.0f; load[size_t(i)] = float(grams(c) / 598.0);
                const auto toe = body.toe_body(i, ang[size_t(i * 3)], ang[size_t(i * 3 + 1)], ang[size_t(i * 3 + 2)]);
                xz[size_t(2 * i)] = toe.x; xz[size_t(2 * i + 1)] = toe.z;
                y[size_t(i)] = float(double(toe.dot(te.up)) + tcal.apply(i, ang[size_t(i * 3)], ang[size_t(i * 3 + 2)]));
            }
            bus.begin_tick(tick);
            bus.publish("reality.proprio.foot_contact", tok(contact, tick)); bus.publish("reality.proprio.feet_xz", tok(xz, tick));
            bus.publish("reality.proprio.toe_y", tok(y, tick)); bus.publish("reality.proprio.foot_load", tok(load, tick));
            bus.publish("sense.servo_current", tok({float(te.i_a)}, tick));
            if (te.tof_ok) bus.publish("reality.proprio.belly_m", tok({float(te.tof)}, tick));
            bus.publish("reality.proprio.upright", tok({te.up.y}, tick));
            enc->tick(tick); epm->tick(tick); bel->tick(tick);
            const long now_events = enc->diag_snapshot().value("events", 0L);
            auto e = std::dynamic_pointer_cast<const ProprioToken>(bus.last_value("sense.touch_encounter"));
            if (now_events != seen_events && e) {
                seen_events = now_events;
                json out; out["t_mono_ms"] = t; out["file"] = rec;
                auto dg = enc->diag_snapshot(); out["leg"] = kPhysOfSim[dg.value("last_leg", 0)];
                json feats = json::array(); for (int k = 0; k < e->values.size(); ++k) feats.push_back(e->values[k]);
                out["features"] = feats;
                if (auto r = std::dynamic_pointer_cast<const RealityToken>(bus.last_value("reality.touch.encounter"))) {
                    out["winner"] = r->winner_id; out["tle"] = r->tle; out["novel"] = r->is_novel;
                }
                auto bd = bel->diag_snapshot(); out["belief_map"] = bd.value("map_node", -1); out["belief_entropy"] = bd.value("entropy", 0.0);
                auto ed = epm->diag_lite(); out["nodes"] = ed.value("nodes", 0); out["baked"] = ed.value("baked", 0);
                out["toe_y"] = y[size_t(dg.value("last_leg", 0))];
                out["cond"] = cur_mark.value("cond", "-"); out["mark_leg"] = cur_mark.value("leg", "-"); out["knee"] = cur_mark.value("knee", "-");
                out["height"] = cur_mark.value("height", "-"); out["hip1"] = cur_mark.value("hip1", "-");
                std::cout << out.dump() << "\n"; ++events;
            }
            bus.end_tick(); ++tick;
        }
    }
    std::cerr << "encounter_replay: " << tick << " ticks, " << events << " encounters, EPM nodes " << epm->diag_lite().value("nodes", 0)
              << " baked " << epm->diag_lite().value("baked", 0) << "\n";
    return 0;
}
