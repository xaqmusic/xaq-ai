// epm_tof_study — drive the REAL EPM over recorded duck ToF frames.
//
// The ToF studies (2026-09-12, the duck): the operator asked for an EPM on the sensor's full
// output with its PCA visible, for several EPMs in different roles off the same sensor, and for
// the head-babble point cloud.  CLAUDE.md §0 rule 1 forbids hand-rolling a clusterer for any of
// it, so the study runs the shipped module: frames come from a host log
// (mj_host/tools/tof_prep.py's source JSONL), each configured EPM gets its own topic and its own
// view of the same frame, and the per-tick token, the encoder latent and the final node
// prototypes are written out for offline PCA.
//
//   epm_tof_study --frames LOG.jsonl --out OUT.jsonl
//                 --epm name:view[:kind][,key=json]*   (repeatable)
//                 [--from-tick N] [--latent-every N]
//
//   view   full64  the 64 zone ranges / 4 m, Empty -> 1.0        (the sensor as it is)
//          cols8   the nearest Hit per column / 4 m              (today's map input)
//          geom    the geometric reduction: per column, the nearest return's height band and
//                  range, plus the frame's floor-break count and its angular extent
//   kind   jl_state (default for full64/geom) | rbf | identity
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"
#include "ogma/Topics.hpp"
#include "ogma/modules/EPM.hpp"

using json = nlohmann::json;

namespace {

[[noreturn]] void die(std::string const& m) { std::cerr << "epm_tof_study: " << m << "\n"; std::exit(2); }

constexpr int kRows = 8, kCols = 8, kZones = 64;
constexpr double kMaxRange = 4.0;
constexpr double kHalfDeg = 45.0 / 2.0 - 45.0 / kCols / 2.0;
constexpr double kStepDeg = 2.0 * kHalfDeg / (kCols - 1);

struct Frame {
    double t = 0.0;
    std::vector<double> r;                    // 64 slant ranges, -1 = Empty
    std::vector<int>    cls;                  // 64 classes
    std::vector<std::array<double, 3>> pts;   // 64 levelled points, NaN = none
    bool  has_pts = false;
    double trunk_z = 0.0;
};

// ---- the three views -------------------------------------------------------
std::vector<float> view_full64(Frame const& f) {
    std::vector<float> v(kZones);
    for (int i = 0; i < kZones; ++i)
        v[size_t(i)] = float((f.r[size_t(i)] < 0.0 ? kMaxRange : std::min(f.r[size_t(i)], kMaxRange)) / kMaxRange);
    return v;
}

std::vector<float> view_cols8(Frame const& f) {
    std::vector<float> v(kCols, float(kMaxRange));
    for (int i = 0; i < kZones; ++i)
        if (f.cls[size_t(i)] == 3) v[size_t(i % kCols)] = std::min(v[size_t(i % kCols)], float(f.r[size_t(i)]));
    for (auto& c : v) c /= float(kMaxRange);
    return v;
}

// The geometric reduction: what the 8x8 knows about SHAPE that the column minima throw away.
// Per column (8): the height above the floor of its nearest non-floor return, and that return's
// range.  Then four frame-level terms: the fraction of zones whose return breaks the floor plane
// (something is standing on the floor), the angular extent of that break, the number of range
// edges across columns (an object has two, a wall none), and the mean height of everything seen.
std::vector<float> view_geom(Frame const& f) {
    std::vector<float> h(kCols, 0.0f), rg(kCols, 1.0f);
    int  brk = 0, brk_lo = kCols, brk_hi = -1, seen = 0;
    double hsum = 0.0;
    for (int i = 0; i < kZones; ++i) {
        if (!f.has_pts || std::isnan(f.pts[size_t(i)][2])) continue;
        const double hz = f.pts[size_t(i)][2] + f.trunk_z;      // height above the floor
        ++seen; hsum += hz;
        const int c = i % kCols;
        const bool floorish = hz < 0.02;
        if (!floorish && f.r[size_t(i)] > 0.0) {
            const float rr = float(std::min(f.r[size_t(i)], kMaxRange) / kMaxRange);
            if (rr < rg[size_t(c)]) { rg[size_t(c)] = rr; h[size_t(c)] = float(std::clamp(hz / 0.6, 0.0, 1.0)); }
        }
        // a floor break: a return between 2 and 20 cm, i.e. an object sitting on the ground
        if (hz >= 0.02 && hz < 0.20) { ++brk; brk_lo = std::min(brk_lo, c); brk_hi = std::max(brk_hi, c); }
    }
    int edges = 0;
    for (int c = 1; c < kCols; ++c) if (std::fabs(rg[size_t(c)] - rg[size_t(c - 1)]) > 0.10) ++edges;
    std::vector<float> v;
    v.insert(v.end(), h.begin(), h.end());
    v.insert(v.end(), rg.begin(), rg.end());
    v.push_back(float(brk) / float(kZones));
    v.push_back(brk_hi >= brk_lo ? float(brk_hi - brk_lo + 1) / float(kCols) : 0.0f);
    v.push_back(float(edges) / float(kCols));
    v.push_back(seen ? float(std::clamp(hsum / seen / 0.6, 0.0, 1.0)) : 0.0f);
    return v;                                                   // 20 dims
}

// Rule 2 of CLAUDE.md §0, applied: the raw 64 are dominated by ONE common mode -- how far away
// whatever is ahead happens to be (measured: PC1 holds 85 % of the variance, and a nearest-centroid
// readout of object class from four raw PCs scores 0.45 against a 0.36 majority).  Object identity
// lives in the residual, so these views take the common mode out.
std::vector<float> view_full64_dm(Frame const& f) {          // the frame's own mean removed
    auto v = view_full64(f);
    float m = 0.0f; for (float z : v) m += z; m /= float(v.size());
    for (auto& z : v) z = 0.5f + 0.5f * std::clamp((z - m) / 0.25f, -1.0f, 1.0f);
    return v;
}

std::vector<float> view_full64_norm(Frame const& f) {        // scale out: each frame on its own max
    auto v = view_full64(f);
    float mx = 1e-6f; for (float z : v) mx = std::max(mx, z);
    for (auto& z : v) z /= mx;
    return v;
}

// Per column, the HEIGHT above the floor of its nearest non-floor return -- and nothing about
// range at all.  M1 measured height as the one feature that told the kinds apart (a block returns
// 1-2 zones at 2-10 cm, a chair 4-6 at 5-50 cm), and height is the one thing that does not change
// when the duck walks closer.  8 dims.
std::vector<float> view_heights8(Frame const& f) {
    std::vector<float> h(kCols, 0.0f), best(kCols, 9.0f);
    for (int i = 0; i < kZones; ++i) {
        if (!f.has_pts || std::isnan(f.pts[size_t(i)][2]) || f.r[size_t(i)] <= 0.0) continue;
        const double hz = f.pts[size_t(i)][2] + f.trunk_z;
        if (hz < 0.02) continue;                              // the floor is not a thing
        const int c = i % kCols;
        if (f.r[size_t(i)] < best[size_t(c)]) { best[size_t(c)] = float(f.r[size_t(i)]); h[size_t(c)] = float(std::clamp(hz / 0.6, 0.0, 1.0)); }
    }
    return h;
}

// The shape terms of `geom` with every absolute range dropped: heights per column, the floor-break
// fraction, its angular extent, the edge count, the mean height.  12 dims, distance-free.
std::vector<float> view_geom_shape(Frame const& f) {
    auto g = view_geom(f);
    std::vector<float> v(g.begin(), g.begin() + kCols);       // the 8 heights
    v.insert(v.end(), g.end() - 4, g.end());                  // break, extent, edges, mean height
    return v;
}

std::vector<float> make_view(std::string const& v, Frame const& f) {
    if (v == "full64") return view_full64(f);
    if (v == "cols8")  return view_cols8(f);
    if (v == "geom")   return view_geom(f);
    if (v == "full64_dm")   return view_full64_dm(f);
    if (v == "full64_norm") return view_full64_norm(f);
    if (v == "heights8")    return view_heights8(f);
    if (v == "geom_shape")  return view_geom_shape(f);
    die("unknown view '" + v + "' (full64|cols8|geom|full64_dm|full64_norm|heights8|geom_shape)");
}

ogma::ParamValue to_param(json const& v) {
    if (v.is_boolean())        return ogma::ParamValue{v.get<bool>()};
    if (v.is_number_integer()) return ogma::ParamValue{v.get<int64_t>()};
    if (v.is_number_float())   return ogma::ParamValue{v.get<double>()};
    if (v.is_string())         return ogma::ParamValue{v.get<std::string>()};
    if (v.is_array())          return ogma::ParamValue{v.get<std::vector<double>>()};
    die("unsupported --epm value: " + v.dump());
}

struct Arm {
    std::string name, view, kind;
    std::map<std::string, json> sets;
    std::unique_ptr<ogma::EPM> epm;
    int dims = 0;
    std::string topic;
};

json token_json(std::shared_ptr<const ogma::RealityToken> const& t) {
    if (!t) return nullptr;
    return json{{"winner", t->winner_id}, {"qe", t->quant_error}, {"eerr", t->expected_error},
                {"ts", t->transition_surp}, {"tle", t->tle}, {"nodes", t->node_count},
                {"baked", t->baked_count}, {"novel", t->is_novel}, {"just_baked", t->just_baked}};
}

json nodes_json(ogma::EPM const& e) {
    json snap = e.snapshot_state(), out = json::array();
    if (!snap.contains("gng") || snap["gng"].is_null()) return out;
    for (auto const& n : snap["gng"]["nodes"])
        out.push_back(json{{"id", n.value("id", -1)}, {"visits", n.value("visits", 0)},
                           {"ema_error", n.value("ema_error", 0.0)}, {"proto", n["prototype"]}});
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string frames_path, out_path;
    std::vector<Arm> arms;
    int latent_every = 16;
    double from_t = 0.0;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto need = [&]() { if (i + 1 >= argc) die(k + " needs a value"); return std::string(argv[++i]); };
        if      (k == "--frames")       frames_path = need();
        else if (k == "--out")          out_path = need();
        else if (k == "--from-t")       from_t = std::stod(need());
        else if (k == "--latent-every") latent_every = std::stoi(need());
        else if (k == "--epm") {
            std::string spec = need();
            Arm a;
            auto colon = spec.find(':');
            if (colon == std::string::npos) die("--epm needs name:view[:kind][,key=json]*");
            a.name = spec.substr(0, colon);
            std::string rest = spec.substr(colon + 1);
            auto comma = rest.find(',');
            std::string head = comma == std::string::npos ? rest : rest.substr(0, comma);
            auto c2 = head.find(':');
            a.view = c2 == std::string::npos ? head : head.substr(0, c2);
            a.kind = c2 == std::string::npos ? "" : head.substr(c2 + 1);
            if (comma != std::string::npos) {
                std::string kv = rest.substr(comma + 1);
                size_t p = 0;
                while (p < kv.size()) {
                    auto e = kv.find(',', p);
                    std::string one = kv.substr(p, e == std::string::npos ? std::string::npos : e - p);
                    auto eq = one.find('=');
                    if (eq == std::string::npos) die("--epm set needs key=json: " + one);
                    a.sets[one.substr(0, eq)] = json::parse(one.substr(eq + 1));
                    if (e == std::string::npos) break;
                    p = e + 1;
                }
            }
            arms.push_back(std::move(a));
        } else die("unknown flag " + k);
    }
    if (frames_path.empty() || out_path.empty() || arms.empty())
        die("need --frames, --out and at least one --epm");

    // ---- load the frames ----
    std::vector<Frame> frames;
    {
        std::ifstream in(frames_path);
        if (!in) die("cannot open " + frames_path);
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] != '{') continue;
            json j = json::parse(line);
            if (!j.contains("tofr")) continue;
            Frame f;
            f.t = j.value("t", 0.0);
            f.trunk_z = j.value("z", 0.0);
            f.r = j["tofr"].get<std::vector<double>>();
            const std::string z = j.value("tofz", std::string());
            f.cls.assign(size_t(kZones), 0);
            for (int i = 0; i < kZones && i < int(z.size()); ++i) f.cls[size_t(i)] = z[size_t(i)] - '0';
            f.pts.assign(size_t(kZones), {std::nan(""), std::nan(""), std::nan("")});
            if (j.contains("tofp")) {
                f.has_pts = true;
                for (auto const& e : j["tofp"]) {
                    const int zi = e[0].get<int>();
                    if (zi >= 0 && zi < kZones) f.pts[size_t(zi)] = {e[1].get<double>(), e[2].get<double>(), e[3].get<double>()};
                }
            }
            // The host prints the current ToF state every tick but CASTS every fourth, so a raw
            // read would feed every frame four times and quadruple the GNG's visit counts (and
            // its baking).  Feed the distinct frames only: the sensor's own 12.5 Hz.
            if (!frames.empty() && f.r == frames.back().r) continue;
            frames.push_back(std::move(f));
        }
    }
    if (frames.empty()) die("no frames with tofr in " + frames_path);

    // ---- set the EPMs up ----
    ogma::InProcessBus bus;
    for (auto& a : arms) {
        a.dims = int(make_view(a.view, frames.front()).size());
        if (a.kind.empty()) a.kind = (a.view == "cols8") ? "rbf" : "jl_state";
        ogma::ParamMap p{
            {"modality_group", std::string("tof")},
            {"modality_name",  a.name},
            {"encoder_kind",   a.kind},
            {"input_topic",    std::string("obs." + a.name)},
            {"baking_threshold", int64_t{20}},          // the duck's live value (R43)
            {"min_insertion_error", 0.06},              // ditto
            {"max_nodes", int64_t{256}},
        };
        if (a.kind == "identity") p["projection_dim"] = int64_t{a.dims};
        else {
            p["proprio_state_dims"] = int64_t{a.dims};
            if (a.kind == "rbf") {
                p["dim_min"] = std::vector<double>(size_t(a.dims), 0.0);
                p["dim_max"] = std::vector<double>(size_t(a.dims), 1.0);
            } else {
                p["projection_dim"] = int64_t{104};
            }
        }
        for (auto const& [k, v] : a.sets) p[k] = to_param(v);
        a.epm = std::make_unique<ogma::EPM>();
        a.epm->set_id(a.name);
        a.epm->on_setup(&bus, p);
        a.topic = "reality.tof." + a.name;
    }

    std::ofstream out(out_path);
    if (!out) die("cannot open " + out_path);
    {
        json h{{"event", "header"}, {"frames", frames.size()}, {"from_t", from_t},
               {"latent_every", latent_every}};
        for (auto const& a : arms) h["arms"][a.name] = {{"view", a.view}, {"kind", a.kind}, {"dims", a.dims}};
        out << h.dump() << '\n';
    }

    for (size_t fi = 0; fi < frames.size(); ++fi) {
        if (frames[fi].t < from_t) continue;
        auto const& f = frames[fi];
        bus.begin_tick(uint64_t(fi));
        for (auto& a : arms) {
            const auto v = make_view(a.view, f);
            if (a.kind == "identity") {
                auto c = std::make_shared<ogma::ConsensusToken>();
                c->tick_id = fi; c->producer_id = "study";
                c->fused_embedding = Eigen::VectorXf::Map(v.data(), long(v.size()));
                bus.publish("obs." + a.name, c);
            } else {
                auto pt = std::make_shared<ogma::ProprioToken>();
                pt->tick_id = fi; pt->producer_id = "study"; pt->sensor = a.name;
                pt->values = Eigen::VectorXf::Map(v.data(), long(v.size()));
                bus.publish("obs." + a.name, pt);
            }
        }
        for (auto& a : arms) a.epm->tick(uint64_t(fi));
        bus.end_tick();

        json line{{"event", "tick"}, {"i", fi}, {"t", f.t}};
        const bool want_latent = (int(fi) % std::max(1, latent_every)) == 0;
        for (auto const& a : arms) {
            auto tok = std::dynamic_pointer_cast<const ogma::RealityToken>(bus.last_value(a.topic));
            line[a.name] = token_json(tok);
            if (want_latent && tok) {
                json l = json::array();
                for (int k = 0; k < tok->latent.size(); ++k) l.push_back(tok->latent[k]);
                line[a.name + "_lat"] = l;
            }
        }
        out << line.dump() << '\n';
    }
    for (auto const& a : arms)
        out << json{{"event", "nodes"}, {"arm", a.name}, {"nodes", nodes_json(*a.epm)}}.dump() << '\n';
    std::cerr << "epm_tof_study: " << frames.size() << " frames, " << arms.size() << " arms -> " << out_path << "\n";
    return 0;
}
