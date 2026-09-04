#include <gtest/gtest.h>

#include <fstream>

#include "Catalogue.hpp"
#include "TrialSetup.hpp"
#include "ogma/Module.hpp"

namespace {

bb::Catalogue const& catalogue() {
    static bb::Catalogue cat = bb::Catalogue::build(BB_PALETTE);
    return cat;
}

} // namespace

TEST(Catalogue, EnumeratesEveryRegisteredType) {
    bb::warm_registry();
    auto reg = ogma::ModuleRegistry::instance().registered_types();
    EXPECT_GE(reg.size(), 70u);
    EXPECT_EQ(catalogue().types.size(), reg.size());
    for (auto const& n : reg) EXPECT_NE(catalogue().find(n), nullptr) << n;
}

TEST(Catalogue, PaletteCoversEveryType) {
    for (auto const& t : catalogue().types) {
        EXPECT_NE(t.category, "other") << t.type;
        EXPECT_FALSE(t.purpose.empty()) << t.type;
    }
}

TEST(Catalogue, EpmSchemaAndSockets) {
    auto const* epm = catalogue().find("EPM");
    ASSERT_NE(epm, nullptr);
    EXPECT_NE(epm->param("modality_name"), nullptr);
    EXPECT_NE(epm->param("encoder_kind"), nullptr);
    bool composed_out = false;
    for (auto const& s : epm->sockets)
        if (s.output && s.params.size() >= 2) composed_out = true;
    EXPECT_TRUE(composed_out) << "EPM output should compose modality_group + modality_name";
}

TEST(TrialSetup, ShippedDuckConfigConstructs) {
    std::ifstream f(std::string(BB_MJ_CONFIG_DIR) + "/a1v2_r19_settle_each.json");
    ASSERT_TRUE(f.good());
    nlohmann::json doc = nlohmann::json::parse(f);
    bb::StdoutSilencer quiet;
    bool regime_out = false;
    for (auto const& m : doc["modules"]) {
        auto pm = bb::param_map_from_json(m.value("params", nlohmann::json::object()));
        auto r = bb::trial_setup(m["type"], m["id"], pm);
        EXPECT_TRUE(r.ok) << m["id"] << ": " << r.error;
        for (auto const& o : r.ports.outputs) if (o.name == "reality.proprio.regime") regime_out = true;
    }
    EXPECT_TRUE(regime_out);
}
