#include "ogma/modules/PosturalPrior.hpp"

#include <algorithm>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("PosturalPrior param '" + key + "' must be string");
}
std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("PosturalPrior param '" + key + "' must be a string array");
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("PosturalPrior param '" + key + "' must be numeric");
}
template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
} // namespace

PosturalPrior::PosturalPrior()  = default;
PosturalPrior::~PosturalPrior() = default;

std::string_view PosturalPrior::type_name() const { return "PosturalPrior"; }

std::vector<TopicSpec> PosturalPrior::input_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(proprio_topics_.size());
    for (auto const& t : proprio_topics_)
        v.emplace_back(t, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    return v;
}

std::vector<TopicSpec> PosturalPrior::output_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(objective_output_topics_.size());
    for (auto const& t : objective_output_topics_)
        v.emplace_back(t, std::type_index(typeid(PredictionToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    return v;
}

ParamSchema PosturalPrior::params_schema() const {
    return {
        {"proprio_topics", ParamMutability::ConstructionOnly,
         "Per-leg ProprioToken input topics ([pos,act,delta]×motor_dim). The rest pose is captured from the first frame of each.",
         std::nullopt, std::nullopt, std::nullopt},
        {"objective_output_topics", ParamMutability::ConstructionOnly,
         "Per-leg PredictionToken output topics (e.g. objective.posture.<leg>) carrying the soft rest-pose target. Length must equal proprio_topics.",
         std::nullopt, std::nullopt, std::nullopt},
        {"motor_dim", ParamMutability::ConstructionOnly, "joints per leg",
         ParamValue{int64_t(3)}, ParamValue{int64_t(1)}, ParamValue{int64_t(12)}},
        {"postural_gain", ParamMutability::HotMutable,
         "Objective weight w published as PredictionToken.confidence (clamped [0,1]). 0 = the prior proposes nothing.",
         ParamValue{1.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"knee_tuck_target", ParamMutability::ConstructionOnly,
         "Override the LAST joint's captured rest with this tuck target (spider stance). < -90 disables the override (keep the measured rest).",
         ParamValue{-100.0}, std::nullopt, std::nullopt},
    };
}

ParamMap PosturalPrior::current_params() const {
    ParamMap m;
    m["proprio_topics"]          = std::vector<std::string>(proprio_topics_);
    m["objective_output_topics"] = std::vector<std::string>(objective_output_topics_);
    m["motor_dim"]        = int64_t(motor_dim_);
    m["postural_gain"]    = postural_gain_;
    m["knee_tuck_target"] = knee_tuck_target_;
    return m;
}

void PosturalPrior::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("PosturalPrior requires a non-null Bus");

    apply_param(params, "motor_dim", [&](auto const& v){
        if (auto p = std::get_if<int64_t>(&v)) motor_dim_ = int(*p);
        else throw std::invalid_argument("PosturalPrior: motor_dim must be integer"); });
    apply_param(params, "postural_gain",    [&](auto const& v){ postural_gain_    = get_double(v, "postural_gain"); });
    apply_param(params, "knee_tuck_target", [&](auto const& v){ knee_tuck_target_ = get_double(v, "knee_tuck_target"); });
    apply_param(params, "proprio_topics",   [&](auto const& v){ proprio_topics_   = get_string_vec(v, "proprio_topics"); });
    apply_param(params, "objective_output_topics",
                [&](auto const& v){ objective_output_topics_ = get_string_vec(v, "objective_output_topics"); });

    if (proprio_topics_.empty())
        throw std::invalid_argument("PosturalPrior: proprio_topics must be non-empty");
    if (objective_output_topics_.size() != proprio_topics_.size())
        throw std::invalid_argument("PosturalPrior: objective_output_topics length must equal proprio_topics");
    (void)&get_string;   // reserved for future scalar topic params

    int nl = int(proprio_topics_.size());
    rest_pos_.assign(nl, Eigen::VectorXf());
    captured_.assign(nl, 0);

    sub_ids_.clear();
    for (int leg = 0; leg < nl; ++leg) {
        sub_ids_.push_back(bus_->subscribe(proprio_topics_[leg], SubscriptionKind::Direct,
            [this, leg](std::string_view, MessagePtr p){ handle_proprio(leg, p); }));
    }
}

void PosturalPrior::on_param_change(std::string_view key, ParamValue const& value) {
    if (key == "postural_gain") { postural_gain_ = get_double(value, "postural_gain"); return; }
    throw std::invalid_argument("PosturalPrior: param '" + std::string(key) + "' is not HotMutable");
}

void PosturalPrior::handle_proprio(int leg, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p || leg < 0 || leg >= int(rest_pos_.size())) return;
    // Capture the standing rest pose from the FIRST frame (mirrors MotorEPM's capture):
    // position component is at index 3j of the [pos,act,delta] layout; the last joint
    // (knee) rest is overridden by the tuck target for the spider stance.
    if (!captured_[leg] && p->values.size() >= 3 * motor_dim_) {
        rest_pos_[leg] = Eigen::VectorXf::Zero(motor_dim_);
        for (int j = 0; j < motor_dim_; ++j)
            rest_pos_[leg][j] = p->values[3 * j];
        if (knee_tuck_target_ > -90.0)
            rest_pos_[leg][motor_dim_ - 1] = float(knee_tuck_target_);
        captured_[leg] = 1;
    }
}

void PosturalPrior::tick(uint64_t tick_id) {
    int nl = int(proprio_topics_.size());
    float w = std::clamp(float(postural_gain_), 0.0f, 1.0f);
    for (int leg = 0; leg < nl; ++leg) {
        if (!captured_[leg]) continue;                 // wait until the rest pose is known
        auto out = std::make_shared<PredictionToken>();
        out->tick_id          = tick_id;
        out->producer_id      = id_.empty() ? std::string("postural_prior") : id_;
        out->target_modality  = "posture." + std::to_string(leg);
        out->predicted_latent = rest_pos_[leg];        // the soft rest-pose target
        out->confidence       = w;                      // objective weight
        bus_->publish(objective_output_topics_[leg], out);
    }
}

int PosturalPrior::legs_captured() const {
    int c = 0; for (char b : captured_) if (b) ++c; return c;
}

// ---- snapshot / restore (persist rest_pos so a checkpoint never re-captures a fallen pose) ----
nlohmann::json PosturalPrior::snapshot_state() const {
    nlohmann::json legs = nlohmann::json::array();
    for (int leg = 0; leg < int(rest_pos_.size()); ++leg) {
        nlohmann::json lj;
        lj["captured"] = bool(captured_[leg]);
        lj["rest_pos"] = std::vector<float>(rest_pos_[leg].data(),
                                            rest_pos_[leg].data() + rest_pos_[leg].size());
        legs.push_back(std::move(lj));
    }
    return nlohmann::json{{"version", 1}, {"legs", legs}};
}

void PosturalPrior::restore_state(nlohmann::json const& s) {
    if (!s.contains("legs")) return;
    auto const& legs = s.at("legs");
    for (int leg = 0; leg < int(rest_pos_.size()) && leg < int(legs.size()); ++leg) {
        auto const& lj = legs[leg];
        captured_[leg] = lj.value("captured", false) ? 1 : 0;
        if (lj.contains("rest_pos")) {
            auto v = lj["rest_pos"].get<std::vector<float>>();
            rest_pos_[leg] = Eigen::Map<const Eigen::VectorXf>(v.data(), int(v.size()));
        }
    }
}

} // namespace ogma
