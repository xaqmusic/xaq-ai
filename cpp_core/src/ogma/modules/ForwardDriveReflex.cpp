#include "ogma/modules/ForwardDriveReflex.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))   return *p;
    if (auto p = std::get_if<int64_t>(&v))  return double(*p);
    throw std::invalid_argument("ForwardDriveReflex: param '" + key + "' must be numeric");
}

int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v))  return *p;
    if (auto p = std::get_if<double>(&v))   return int64_t(*p);
    throw std::invalid_argument("ForwardDriveReflex: param '" + key + "' must be integer");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("ForwardDriveReflex: param '" + key + "' must be string");
}

} // namespace

ForwardDriveReflex::ForwardDriveReflex()  = default;
ForwardDriveReflex::~ForwardDriveReflex() = default;

std::string_view ForwardDriveReflex::type_name() const { return "ForwardDriveReflex"; }

std::vector<TopicSpec> ForwardDriveReflex::input_topics() const {
    return {};   // no inputs — purely a constant-thrust pump
}

std::vector<TopicSpec> ForwardDriveReflex::output_topics() const {
    if (!output_topic_.empty()) {
        return { TopicSpec{output_topic_, std::type_index(typeid(ActionOut))} };
    }
    return {
        TopicSpec{output_topic_left_,  std::type_index(typeid(ActionOut))},
        TopicSpec{output_topic_right_, std::type_index(typeid(ActionOut))},
    };
}

ParamSchema ForwardDriveReflex::params_schema() const {
    return {
        {"thrust", ParamMutability::HotMutable,
            "Symmetric thrust applied to both action.left and action.right "
            "each tick (accel ∈ [-4, 4] body convention; default 2.0).",
            ParamValue{2.0}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "Phase 6.6.F: when non-empty, publish a single ActionOut to this topic instead of bilateral action.left/right (lets this serve as the reflex side of a single-channel MotorFader). Empty default preserves bilateral pathway.",
            ParamValue{std::string("")}},
        {"output_topic_left",  ParamMutability::ConstructionOnly,
            "Phase 6.6.G: left-side ActionOut topic when in bilateral mode (default action.left; set to action.reflex.left to route through bilateral MotorFader)",
            ParamValue{std::string(topics::kActionLeft)}},
        {"output_topic_right", ParamMutability::ConstructionOnly,
            "Phase 6.6.G: right-side ActionOut topic when in bilateral mode (default action.right; set to action.reflex.right to route through bilateral MotorFader)",
            ParamValue{std::string(topics::kActionRight)}},
        {"noise_amplitude", ParamMutability::HotMutable,
            "Phase 6.6.G: per-tick uniform(-amp, +amp) added independently to each side's thrust before publish.  Restores exploration drift the body's pre-modular flagellum_base_rate used to provide.  Default 0 = legacy.",
            ParamValue{0.0}},
        {"master_seed", ParamMutability::ConstructionOnly,
            "PRNG seed for the noise stream (0 = built-in default)",
            ParamValue{int64_t{0}}},
    };
}

ParamMap ForwardDriveReflex::current_params() const {
    ParamMap m;
    m["thrust"]              = ParamValue{double(thrust_)};
    m["output_topic"]        = ParamValue{output_topic_};
    m["output_topic_left"]   = ParamValue{output_topic_left_};
    m["output_topic_right"]  = ParamValue{output_topic_right_};
    m["noise_amplitude"]     = ParamValue{double(noise_amplitude_)};
    m["master_seed"]         = ParamValue{int64_t(master_seed_)};
    return m;
}

void ForwardDriveReflex::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("ForwardDriveReflex requires a non-null Bus");
    apply_param(params, "thrust",
        [&](auto const& v){ thrust_ = float(get_double(v, "thrust")); });
    apply_param(params, "output_topic",
        [&](auto const& v){ output_topic_ = get_string(v, "output_topic"); });
    apply_param(params, "output_topic_left",
        [&](auto const& v){ output_topic_left_  = get_string(v, "output_topic_left"); });
    apply_param(params, "output_topic_right",
        [&](auto const& v){ output_topic_right_ = get_string(v, "output_topic_right"); });
    apply_param(params, "noise_amplitude",
        [&](auto const& v){ noise_amplitude_ = float(get_double(v, "noise_amplitude")); });
    apply_param(params, "master_seed",
        [&](auto const& v){ master_seed_ = uint64_t(get_int(v, "master_seed")); });
    rng_.seed(master_seed_ ? master_seed_ : 0xF00DBABEu);
}

void ForwardDriveReflex::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "thrust")             thrust_          = float(get_double(value, k));
    else if (k == "noise_amplitude") noise_amplitude_ = float(get_double(value, k));
    else throw std::invalid_argument("ForwardDriveReflex: unknown/non-mutable param '" + k + "'");
}

void ForwardDriveReflex::tick(uint64_t tick_id) {
    auto sample_noise = [&]() {
        if (noise_amplitude_ <= 0.0f) return 0.0f;
        std::uniform_real_distribution<float> u(-noise_amplitude_, noise_amplitude_);
        return u(rng_);
    };

    if (!output_topic_.empty()) {
        // Single-channel mode (Phase 6.6.F): symmetric thrust → single accel.
        // Noise on the single channel still helps the body avoid locking
        // into a constant-rate steer; one independent draw per tick.
        auto act = std::make_shared<ActionOut>();
        act->tick_id     = tick_id;
        act->producer_id = id_.empty() ? std::string("forward_drive") : id_;
        act->accel       = thrust_ + sample_noise();
        act->source      = "forward_drive";
        bus_->publish(output_topic_, act);
    } else {
        // Two independent draws — left and right side noise are independent
        // so a fresh asymmetric bias is published every tick.  At
        // noise_amplitude=0.5 the per-tick |L−R| ranges over [0, 1] giving
        // continuous turning drift.
        auto act_l = std::make_shared<ActionOut>();
        act_l->tick_id     = tick_id;
        act_l->producer_id = id_.empty() ? std::string("forward_drive") : id_;
        act_l->accel       = thrust_ + sample_noise();
        act_l->source      = "forward_drive";
        act_l->probe       = false;
        act_l->action_tle  = 0.0f;
        bus_->publish(output_topic_left_, act_l);

        auto act_r = std::make_shared<ActionOut>();
        act_r->tick_id     = tick_id;
        act_r->producer_id = act_l->producer_id;
        act_r->accel       = thrust_ + sample_noise();
        act_r->source      = "forward_drive";
        act_r->probe       = false;
        act_r->action_tle  = 0.0f;
        bus_->publish(output_topic_right_, act_r);
    }

    ++tick_count_;
}

nlohmann::json ForwardDriveReflex::snapshot_state() const {
    std::ostringstream os; os << rng_;
    return nlohmann::json{
        {"version",    1},
        {"tick_count", tick_count_},
        {"rng",        os.str()},
    };
}

void ForwardDriveReflex::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("ForwardDriveReflex::restore_state: unknown version " +
                                 std::to_string(version));
    }
    tick_count_ = s.value("tick_count", tick_count_);
    if (s.contains("rng") && s["rng"].is_string()) {
        std::istringstream is(s["rng"].get<std::string>()); is >> rng_;
    }
}

} // namespace ogma
