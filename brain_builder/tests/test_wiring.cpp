#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

#include <set>

#include "Body.hpp"
#include "Catalogue.hpp"
#include "DryRun.hpp"
#include "LiveSync.hpp"
#include "Order.hpp"
#include "Graph.hpp"
#include "TrialSetup.hpp"
#include "Wiring.hpp"

namespace {

bb::Catalogue const& catalogue() { static bb::Catalogue c = bb::Catalogue::build(BB_PALETTE); return c; }
bb::BodyRegistry const& bodies()  { static bb::BodyRegistry b = bb::BodyRegistry::load_dir(BB_BODIES_DIR); return b; }

bool has_link(bb::Wiring const& w, std::string const& from, std::string const& to, std::string const& topic) {
    for (auto const& l : w.links) if (l.from_node == from && l.to_node == to && l.topic == topic) return true;
    return false;
}

} // namespace

TEST(Wiring, R19DuckConfigWiresBodyToBrainAndBack) {
    bb::StdoutSilencer quiet;
    bb::Graph g = bb::Graph::load(std::string(BB_MJ_CONFIG_DIR) + "/a1v2_r19_settle_each.json");
    bb::Body const* body = bodies().find("microduck_joints");
    ASSERT_NE(body, nullptr);
    bb::TrialCache cache;
    bb::Wiring w = bb::Wiring::build(g, catalogue(), body, cache);
    ASSERT_EQ(w.nodes.size(), g.size() + 3);
    for (auto const& n : w.nodes) EXPECT_TRUE(n.setup_ok) << n.name << ": " << n.setup_error;
    EXPECT_TRUE(has_link(w, "@sources", "legs_bridge", "reality.proprio.joints"));
    EXPECT_TRUE(has_link(w, "@sources", "regime_epm", "reality.proprio.sense1"));
    EXPECT_TRUE(has_link(w, "legs_bridge", "motor_epm_legs", "reality.motor_limb.left"));
    EXPECT_TRUE(has_link(w, "motor_epm_legs", "legs_bridge", "action.left_knee"));
    EXPECT_TRUE(has_link(w, "motor_epm_legs", "@sinks", "action.left_knee"));
    EXPECT_TRUE(has_link(w, "regime_epm", "@sinks", "reality.proprio.regime"));
    EXPECT_EQ(w.errors, 0);
    // every pin id unique and non-zero
    std::set<uint64_t> ids;
    for (auto const& n : w.nodes) {
        EXPECT_NE(n.id, 0u);
        EXPECT_TRUE(ids.insert(n.id).second) << n.name;
        for (auto const& p : n.inputs)  { EXPECT_NE(p.id, 0u); EXPECT_TRUE(ids.insert(p.id).second) << n.name << " in " << p.label; }
        for (auto const& p : n.outputs) { EXPECT_NE(p.id, 0u); EXPECT_TRUE(ids.insert(p.id).second) << n.name << " out " << p.label; }
    }
    for (auto const& l : w.links) EXPECT_TRUE(ids.insert(l.id).second);
}

TEST(Wiring, ConnectSetsParamsAndComposedSplitsTopic) {
    bb::StdoutSilencer quiet;
    bb::Body const* body = bodies().find("microduck_joints");
    ASSERT_NE(body, nullptr);
    bb::Graph g = bb::Graph::empty(body->env_target);
    std::string epm = g.add_module("EPM", "epm", bb::ojson::object());
    bb::TrialCache cache;
    bb::Wiring w = bb::Wiring::build(g, catalogue(), body, cache);
    bb::Node const* n = w.node_named(epm);
    ASSERT_NE(n, nullptr);
    EXPECT_FALSE(n->setup_ok);   // required params missing → placeholders shown
    bb::Pin const* in_ph = nullptr; bb::Pin const* out_ph = nullptr;
    for (auto const& p : n->inputs)  if (p.placeholder && p.label == "{input_topic}") in_ph = &p;
    for (auto const& p : n->outputs) if (p.placeholder && p.label.find("{modality_group}") != std::string::npos) out_ph = &p;
    ASSERT_NE(in_ph, nullptr);
    ASSERT_NE(out_ph, nullptr);
    bb::Pin const* sense1 = nullptr; bb::Pin const* regime_read = nullptr;
    for (auto const& p : w.node_named("@sources")->outputs) if (p.topic == "reality.proprio.sense1") sense1 = &p;
    for (auto const& p : w.node_named("@sinks")->inputs)    if (p.topic == "reality.proprio.regime") regime_read = &p;
    ASSERT_NE(sense1, nullptr);
    ASSERT_NE(regime_read, nullptr);

    EXPECT_EQ(w.connect(g, catalogue(), sense1->id, in_ph->id), "");
    EXPECT_EQ(g.params(0)["input_topic"], "reality.proprio.sense1");
    EXPECT_EQ(w.connect(g, catalogue(), out_ph->id, regime_read->id), "");
    EXPECT_EQ(g.params(0)["modality_group"], "proprio");
    EXPECT_EQ(g.params(0)["modality_name"], "regime");

    bb::Wiring w2 = bb::Wiring::build(g, catalogue(), body, cache);
    EXPECT_TRUE(has_link(w2, "@sources", epm, "reality.proprio.sense1"));
    EXPECT_TRUE(has_link(w2, epm, "@sinks", "reality.proprio.regime"));

    // disconnect the input: the key goes away, undo brings it back
    bb::Link const* l = nullptr;
    for (auto const& x : w2.links) if (x.to_node == epm) l = &x;
    ASSERT_NE(l, nullptr);
    EXPECT_EQ(w2.disconnect(g, catalogue(), *l), "");
    EXPECT_FALSE(g.params(0).contains("input_topic"));
    EXPECT_TRUE(g.undo());
    EXPECT_EQ(g.params(0)["input_topic"], "reality.proprio.sense1");
}

TEST(Graph, RoundTripKeepsEveryKeyAndOrder) {
    std::string path = std::string(BB_MJ_CONFIG_DIR) + "/a1v2_r19_settle_each.json";
    bb::Graph g = bb::Graph::load(path);
    std::ifstream f(path);
    std::stringstream ss; ss << f.rdbuf();
    nlohmann::json a = nlohmann::json::parse(ss.str());
    nlohmann::json b = nlohmann::json::parse(g.dump());
    EXPECT_EQ(a, b);
    std::vector<std::string> ids_a, ids_b;
    for (auto const& m : a["modules"]) ids_a.push_back(m["id"]);
    for (auto const& m : b["modules"]) ids_b.push_back(m["id"]);
    EXPECT_EQ(ids_a, ids_b);
    EXPECT_TRUE(g.ascii_escapes);
    EXPECT_EQ(g.dump(), ss.str()) << "bytes differ (semantic identity still holds)";
}

TEST(Order, R19KeepsBridgesBeforeMotorEpmsAndNamesTheCycle) {
    bb::StdoutSilencer quiet;
    bb::Graph g = bb::Graph::load(std::string(BB_MJ_CONFIG_DIR) + "/a1v2_r19_settle_each.json");
    bb::TrialCache cache;
    bb::Wiring w = bb::Wiring::build(g, catalogue(), bodies().find("microduck_joints"), cache);
    bb::OrderSuggestion s = bb::topological_order(g, w);
    ASSERT_EQ(s.order.size(), g.size());
    int bridge = -1, motor = -1;
    for (size_t i = 0; i < s.order.size(); ++i) {
        std::string id = g.id_of(size_t(s.order[i]));
        if (id == "legs_bridge") bridge = int(i);
        if (id == "motor_epm_legs") motor = int(i);
    }
    EXPECT_LT(bridge, motor);
    bool cycle_note = false;
    for (auto const& n : s.notes) if (n.find("cycle") != std::string::npos) cycle_note = true;
    EXPECT_TRUE(cycle_note);
    // reorder round-trips through the graph
    std::vector<int> rev(g.size());
    for (size_t i = 0; i < rev.size(); ++i) rev[i] = int(rev.size() - 1 - i);
    std::string first = g.id_of(0);
    EXPECT_TRUE(g.reorder(rev));
    EXPECT_EQ(g.id_of(g.size() - 1), first);
    EXPECT_TRUE(g.undo());
    EXPECT_EQ(g.id_of(0), first);
}

TEST(DryRun, R19ConstructsAndDrivesTheJoints) {
    bb::StdoutSilencer quiet;
    bb::Graph g = bb::Graph::load(std::string(BB_MJ_CONFIG_DIR) + "/a1v2_r19_settle_each.json");
    bb::Body const* body = bodies().find("microduck_joints");
    bb::TrialCache cache;
    bb::Wiring w = bb::Wiring::build(g, catalogue(), body, cache);
    bb::DryRunReport r = bb::dry_run(g, w, body, 50);
    EXPECT_TRUE(r.constructed) << r.error;
    EXPECT_EQ(r.error, "");
    EXPECT_EQ(r.ticks_done, 50);
    bool knee = false;
    for (auto const& a : r.actions_seen) if (a == "action.left_knee") knee = true;
    EXPECT_TRUE(knee) << "action.left_knee not driven";
    bool regime = false;
    for (auto const& p : r.published) if (p == "reality.proprio.regime") regime = true;
    EXPECT_TRUE(regime);
}

TEST(LiveSync, DiffEmitsPatchesAndFlagsConstructionOnlyEdits) {
    bb::StdoutSilencer quiet;
    bb::Graph g = bb::Graph::load(std::string(BB_MJ_CONFIG_DIR) + "/a1v2_r19_settle_each.json");
    nlohmann::json synced = nlohmann::json::parse(g.doc["modules"].dump());
    // nothing changed → nothing to send
    bb::LiveOps none = bb::diff_for_live(g, catalogue(), synced);
    EXPECT_TRUE(none.ops.empty()); EXPECT_TRUE(none.recreate.empty());
    // a hot-mutable param on the regime EPM → set_param
    bb::TypeInfo const* epm = catalogue().find("EPM");
    ASSERT_NE(epm, nullptr);
    std::string hot;
    for (auto const& p : epm->params) if (p.hot && p.kind == bb::ParamKind::Float) { hot = p.key; break; }
    ASSERT_FALSE(hot.empty());
    g.set_param("regime_epm", hot, 0.123);
    // a construction-only change on a bridge → recreate
    g.set_param("head_bridge", "load_topic", "reality.proprio.sense1");
    // a new module → add_node; a removed one → remove_node
    g.add_module("EPM", "epm_new", bb::ojson{{"modality_group", "proprio"}, {"modality_name", "x"}, {"input_topic", "reality.proprio.imu"}});
    g.remove_module("motor_epm_head");
    bb::LiveOps d = bb::diff_for_live(g, catalogue(), synced);
    int adds = 0, removes = 0, sets = 0;
    for (auto const& o : d.ops) {
        std::string op = o.value("op", "");
        if (op == "add_node") { ++adds; EXPECT_EQ(o["id"], "epm_new"); }
        if (op == "remove_node") { ++removes; EXPECT_EQ(o["id"], "motor_epm_head"); }
        if (op == "set_param") { ++sets; EXPECT_EQ(o["id"], "regime_epm"); EXPECT_EQ(o["key"], hot); }
    }
    EXPECT_EQ(adds, 1); EXPECT_EQ(removes, 1); EXPECT_EQ(sets, 1);
    ASSERT_EQ(d.recreate.size(), 1u);
    EXPECT_EQ(d.recreate[0], "head_bridge");
    nlohmann::json rc = bb::recreate_ops(g, "head_bridge");
    ASSERT_EQ(rc.size(), 2u);
    EXPECT_EQ(rc[0]["op"], "remove_node");
    EXPECT_EQ(rc[1]["op"], "add_node");
    EXPECT_EQ(rc[1]["params"]["load_topic"], "reality.proprio.sense1");
}

TEST(LiveSync, AdoptReplacesModulesAndKeepsMetadata) {
    bb::Graph g = bb::Graph::load(std::string(BB_MJ_CONFIG_DIR) + "/a1v2_r19_settle_each.json");
    std::string name = g.metadata().value("name", "");
    nlohmann::json cfg = {{"modules", nlohmann::json::array({{{"id", "a"}, {"type", "EPM"}, {"params", {{"k", 1}}}}})},
                          {"runtime", {{"thread_pool", "per_instance"}, {"num_threads", 0}, {"auto_subscribe", true}}}};
    bb::adopt_live_modules(g, cfg);
    EXPECT_EQ(g.size(), 1u);
    EXPECT_EQ(g.id_of(0), "a");
    EXPECT_EQ(g.metadata().value("name", ""), name);
}
