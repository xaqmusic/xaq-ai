// =============================================================================
// LoopCompetence.cpp  --  see the header (Cell round 4, register O21)
// =============================================================================
#include "ogma/modules/LoopCompetence.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

namespace ogma {

namespace {
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("LoopCompetence: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("LoopCompetence: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("LoopCompetence: param '" + k + "' must be a string");
}
} // namespace

LoopCompetence::LoopCompetence()  = default;
LoopCompetence::~LoopCompetence() = default;

std::string_view LoopCompetence::type_name() const { return "LoopCompetence"; }

std::vector<TopicSpec> LoopCompetence::input_topics() const {
    std::vector<TopicSpec> v;
    if (!objective_topic_.empty())
        v.push_back(TopicSpec{objective_topic_, objective_field_ == "tle" ? std::type_index(typeid(RealityToken))
                                                                          : std::type_index(typeid(ProprioToken)),
                              SubscriptionKind::Direct, false});
    if (!gain_topic_.empty())
        v.push_back(TopicSpec{gain_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false});
    return v;
}

std::vector<TopicSpec> LoopCompetence::output_topics() const {
    return { TopicSpec{output_topic_, std::type_index(typeid(RealityToken))} };
}

ParamSchema LoopCompetence::params_schema() const {
    return {
        {"objective_topic", ParamMutability::ConstructionOnly,
            "The scalar this loop's policy claims to improve while it drives (klino: reality.proprio.scent_max; planner: reality.cognitive.plan_value; vision: reality.cognitive.vision_value; play: the place EPM's token with objective_field tle).",
            std::nullopt},
        {"objective_field", ParamMutability::ConstructionOnly,
            "'value' = ProprioToken values[objective_index]; 'tle' = RealityToken.tle (novelty, for play).",
            ParamValue{std::string("value")}},
        {"objective_index", ParamMutability::ConstructionOnly, "Index into the ProprioToken values.", ParamValue{int64_t{0}}},
        {"gain_topic", ParamMutability::ConstructionOnly,
            "arbiter.gain.<loop> (ProprioToken scalar): the loop drives when > 0.5. Competence is checked only while it drives; otherwise it relaxes toward the prior.",
            std::nullopt},
        {"modality_group", ParamMutability::ConstructionOnly, "Output topic group: reality.<group>.<name>.", ParamValue{std::string("loop")}},
        {"modality_name",  ParamMutability::ConstructionOnly, "Output topic name (the loop's name).", std::nullopt},
        {"sign", ParamMutability::HotMutable, "+1: the loop predicts its objective RISES while it drives; -1: falls.", ParamValue{1.0}},
        {"horizon_ticks", ParamMutability::HotMutable,
            "The loop's prediction horizon: one check per window of this many consecutive driving ticks (did sign·Δobjective exceed 0 over the window).",
            ParamValue{int64_t{30}}},
        {"alpha", ParamMutability::HotMutable, "EMA rate of the competence over checks (a fraction in [0,1]).", ParamValue{0.05}},
        {"forget", ParamMutability::HotMutable, "Per-tick relaxation of the competence toward 0.5 while NOT driving (uncertainty grows without observation).", ParamValue{0.001}},
    };
}

ParamMap LoopCompetence::current_params() const {
    ParamMap m;
    m["objective_topic"] = ParamValue{objective_topic_}; m["objective_field"] = ParamValue{objective_field_};
    m["objective_index"] = ParamValue{int64_t(objective_index_)}; m["gain_topic"] = ParamValue{gain_topic_};
    m["modality_group"] = ParamValue{modality_group_}; m["modality_name"] = ParamValue{modality_name_};
    m["sign"] = ParamValue{double(sign_)}; m["horizon_ticks"] = ParamValue{int64_t(horizon_ticks_)};
    m["alpha"] = ParamValue{double(alpha_)}; m["forget"] = ParamValue{double(forget_)};
    return m;
}

void LoopCompetence::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("LoopCompetence requires a non-null Bus");
    apply_param(params, "objective_topic", [&](auto const& v){ objective_topic_ = get_string(v,"objective_topic"); });
    apply_param(params, "objective_field", [&](auto const& v){ objective_field_ = get_string(v,"objective_field"); });
    apply_param(params, "objective_index", [&](auto const& v){ objective_index_ = int(get_int(v,"objective_index")); });
    apply_param(params, "gain_topic",      [&](auto const& v){ gain_topic_      = get_string(v,"gain_topic"); });
    apply_param(params, "modality_group",  [&](auto const& v){ modality_group_  = get_string(v,"modality_group"); });
    apply_param(params, "modality_name",   [&](auto const& v){ modality_name_   = get_string(v,"modality_name"); });
    apply_param(params, "sign",            [&](auto const& v){ sign_            = float(get_double(v,"sign")); });
    apply_param(params, "horizon_ticks",   [&](auto const& v){ horizon_ticks_   = int(get_int(v,"horizon_ticks")); });
    apply_param(params, "alpha",           [&](auto const& v){ alpha_           = float(get_double(v,"alpha")); });
    apply_param(params, "forget",          [&](auto const& v){ forget_          = float(get_double(v,"forget")); });
    if (objective_topic_.empty()) throw std::invalid_argument("LoopCompetence: objective_topic is required");
    if (gain_topic_.empty())      throw std::invalid_argument("LoopCompetence: gain_topic is required");
    if (modality_name_.empty())   throw std::invalid_argument("LoopCompetence: modality_name is required");
    if (horizon_ticks_ < 1) horizon_ticks_ = 1;
    output_topic_ = "reality." + modality_group_ + "." + modality_name_;
    c_ = prior_;

    sub_ids_.push_back(bus_->subscribe(objective_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_objective(p); }));
    sub_ids_.push_back(bus_->subscribe(gain_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ handle_gain(p); }));
}

void LoopCompetence::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "sign")          sign_ = float(get_double(value, k));
    else if (k == "horizon_ticks") horizon_ticks_ = std::max(1, int(get_int(value, k)));
    else if (k == "alpha")         alpha_ = float(get_double(value, k));
    else if (k == "forget")        forget_ = float(get_double(value, k));
    else throw std::invalid_argument("LoopCompetence: param '" + k + "' is construction-only / unknown");
}

void LoopCompetence::handle_objective(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    if (objective_field_ == "tle") {
        if (auto rt = std::dynamic_pointer_cast<const RealityToken>(p)) { obj_ = rt->tle; have_obj_ = true; }
        return;
    }
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(p))
        if (objective_index_ >= 0 && objective_index_ < pt->values.size()) { obj_ = float(pt->values[objective_index_]); have_obj_ = true; }
}
void LoopCompetence::handle_gain(MessagePtr p) {
    if (!input_allowed(p->producer_id)) return;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(p))
        if (pt->values.size() > 0) gain_ = float(pt->values[0]);
}

void LoopCompetence::tick(uint64_t tick_id) {
    const bool drive_now = gain_ > 0.5f && have_obj_;
    if (drive_now) {
        if (!driving_) { run_ticks_ = 0; o_start_ = obj_; }
        ++run_ticks_;
        if (run_ticks_ >= horizon_ticks_) {
            // one check per window of continuous driving: did the objective move as predicted?
            const bool improved = sign_ * (obj_ - o_start_) > 0.0f;
            ++checks_; improvements_ += improved;
            c_ += alpha_ * ((improved ? 1.0f : 0.0f) - c_);
            run_ticks_ = 0; o_start_ = obj_;
        }
    } else {
        c_ += forget_ * (prior_ - c_);          // uncertainty grows without observation
    }
    driving_ = drive_now;

    auto tok = std::make_shared<RealityToken>();
    tok->tick_id     = tick_id;
    tok->producer_id = id_.empty() ? output_topic_ : id_;
    tok->winner_id   = 0;
    tok->latent      = Eigen::VectorXf::Constant(1, c_);
    tok->winner_prototype = tok->latent;
    const float err  = std::clamp(1.0f - c_, 0.0f, 1.0f);
    tok->quant_error = err; tok->expected_error = err; tok->tle = err; tok->transition_surp = 0.0f;
    tok->node_count = 1; tok->baked_count = 3;   // informative by construction (one concept, graded)
    bus_->publish(output_topic_, tok);
}

nlohmann::json LoopCompetence::snapshot_state() const {
    return nlohmann::json{{"version", 1}, {"c", c_}, {"obj", obj_}, {"have_obj", have_obj_}, {"gain", gain_},
                          {"driving", driving_}, {"run_ticks", run_ticks_}, {"o_start", o_start_},
                          {"checks", checks_}, {"improvements", improvements_}};
}
void LoopCompetence::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty() || s.value("version", 0) != 1) return;
    c_ = s.value("c", 0.5f); obj_ = s.value("obj", 0.0f); have_obj_ = s.value("have_obj", false); gain_ = s.value("gain", 0.0f);
    driving_ = s.value("driving", false); run_ticks_ = s.value("run_ticks", 0); o_start_ = s.value("o_start", 0.0f);
    checks_ = s.value("checks", uint64_t{0}); improvements_ = s.value("improvements", uint64_t{0});
}
nlohmann::json LoopCompetence::diag_snapshot() const {
    return nlohmann::json{{"competence", c_}, {"driving", driving_}, {"checks", checks_}, {"improvements", improvements_},
                          {"objective", obj_}, {"gain", gain_}};
}

} // namespace ogma
