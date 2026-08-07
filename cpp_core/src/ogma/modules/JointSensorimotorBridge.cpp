#include "ogma/modules/JointSensorimotorBridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <typeindex>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("JointSensorimotorBridge param '" + key + "' must be string");
}

std::vector<std::string> get_string_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("JointSensorimotorBridge param '" + key + "' must be a string array");
}

std::vector<double> get_double_vec(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("JointSensorimotorBridge param '" + key + "' must be a numeric array");
}

} // namespace

JointSensorimotorBridge::JointSensorimotorBridge()  = default;
JointSensorimotorBridge::~JointSensorimotorBridge() = default;

std::string_view JointSensorimotorBridge::type_name() const { return "JointSensorimotorBridge"; }

std::vector<TopicSpec> JointSensorimotorBridge::input_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(1 + action_topics_.size());
    if (!load_topic_.empty())
        v.emplace_back(load_topic_, std::type_index(typeid(ProprioToken)),
                       SubscriptionKind::Direct, /*required=*/false);
    v.emplace_back(proprio_input_topic_, std::type_index(typeid(ProprioToken)),
                   SubscriptionKind::Direct, /*required=*/false);
    for (auto const& t : action_topics_) {
        v.emplace_back(t, std::type_index(typeid(ActionOut)),
                       SubscriptionKind::Direct, /*required=*/false);
    }
    return v;
}

std::vector<TopicSpec> JointSensorimotorBridge::output_topics() const {
    std::vector<TopicSpec> v;
    v.reserve(output_topics_.size());
    for (auto const& t : output_topics_) {
        v.emplace_back(t, std::type_index(typeid(ProprioToken)));
    }
    return v;
}

ParamSchema JointSensorimotorBridge::params_schema() const {
    return {
        {"proprio_input_topic", ParamMutability::ConstructionOnly,
            "Bundled per-joint ProprioToken topic (e.g. reality.proprio.joints).  values[proprio_indices[i]] is read for joint i.",
            ParamValue{std::string("reality.proprio.joints")}},
        {"action_topics",       ParamMutability::ConstructionOnly,
            "Array of N ActionOut topics, one per joint (e.g. [\"action.fl_hip1\", ...]).",
            std::nullopt},
        {"output_topics",       ParamMutability::ConstructionOnly,
            "Array of N output ProprioToken topics (e.g. [\"reality.joint_fl.hip1\", ...]).  Same length as action_topics.",
            std::nullopt},
        {"proprio_indices",     ParamMutability::ConstructionOnly,
            "Array of N integer indices into ProprioToken.values.  Same length as action_topics.",
            std::nullopt},
        {"pos_noise_sigma",     ParamMutability::ConstructionOnly,
            "IMPORT I4b: colored noise amplitude on the POSITION channel only (delta stays "
            "computed from clean positions, so the dose reaches one channel not two).  PM wires "
            "every legged controller through ColorUniformNoise(0.1); measured optimum on this "
            "body is lower (posture peaks near 0.03).  0 = off, byte-identical.",
            ParamValue{0.0}},
        {"vel_noise_sigma",     ParamMutability::ConstructionOnly,
            "IMPORT I4c: colored noise on the VELOCITY (delta) channel only, position left clean "
            "-- the exact complement of pos_noise_sigma.  Confirms whether the measured posture "
            "rise from both-channel noise comes from the velocity component (the channel the HK "
            "gradient weights most, 44% of |C| mass).  SCALE: delta is a per-tick difference, so "
            "the body-side sigma=0.03 equivalent here is ~0.015, not 0.03.  0 = off.",
            ParamValue{0.0}},
        {"pos_noise_tau",       ParamMutability::ConstructionOnly,
            "Correlation length of that noise in ticks (1 = white).  Colored noise excites the "
            "low-frequency modes a leg can actually follow; white is filtered out by servo dynamics.",
            ParamValue{8.0}},
        {"seed",      ParamMutability::ConstructionOnly,
            "RNG seed for the position-noise trace.  Deliberately named `seed` so "
            "OgmaBrain::set_master_seed rewrites it per run; otherwise every seed shares one "
            "noise trace and the seed-average understates the spread.",
            ParamValue{int64_t(0)}},
        {"sensor_label_prefix", ParamMutability::ConstructionOnly,
            "Optional prefix for the published ProprioToken.sensor field (final form = <prefix>.<derived joint suffix>).  Empty = use output topic suffix only.",
            ParamValue{std::string("joint")}},
        {"group_size",          ParamMutability::ConstructionOnly,
            "Phase 7.2-EPM: when > 1, groups every N consecutive action_topics/proprio_indices into ONE output of length 3*N (concatenated [pos,action,delta] triples).  output_topics length must equal action_topics.size()/group_size.  Default 1 = per-joint outputs.",
            ParamValue{int64_t{1}}},
        {"load_topic",          ParamMutability::ConstructionOnly,
            "Optional per-leg LOAD channel (e.g. reality.proprio.foot_load) appended as the trailing element of every output vector, so a downstream forward model can learn HOW ITS OWN ACTIONS REDISTRIBUTE WEIGHT.  MotorEPMv2 sizes its model from the arriving vector and guards on `>= 3*motor_dim`, so the extra element is learned automatically and existing indices are unchanged.  ⚠ The emitted width is fixed by whether this is CONFIGURED, not by whether a value has arrived — MotorEPMv2 latches its dimensionality on the first frame and drops any frame of a different width, so a late-arriving topic would otherwise silently kill the consumer.  Empty = off, byte-identical.",
            ParamValue{std::string("")}},
        {"range_probe_ticks",   ParamMutability::HotMutable,
            "INSTRUMENT, not a lever.  When > 0, accumulate per-output per-dim min/max/mean/std of the published [pos,action,delta] channels and print one `BRIDGE_RANGE` JSON line to stdout every N ticks (cumulative, so the LAST line is the whole-run answer).  Exists to set a downstream RBF EPM's `dim_min`/`dim_max` from MEASUREMENT rather than assumption: pos/action are ~[-1,1] but delta is a per-tick difference an order of magnitude smaller, and the EPM's default [-1,1] range would crush the velocity channels (CLAUDE.md §0 rule 2).  0 = off: nothing accumulated, nothing printed, byte-identical.",
            ParamValue{int64_t{0}}},
    };
}

// --- Range probe (instrument) ------------------------------------------------
//
// Cumulative Welford-free accumulation: min/max plus sum/sumsq per (output, dim).
// Cheap enough to leave in the tick path when enabled, absent entirely when not.

void JointSensorimotorBridge::range_probe_accum(int out_idx, int dim, float v) {
    const int dim_out = 3 * group_size_;
    const size_t k    = size_t(out_idx) * size_t(dim_out) + size_t(dim);
    if (k >= rp_min_.size()) return;
    rp_min_[k]    = std::min(rp_min_[k], v);
    rp_max_[k]    = std::max(rp_max_[k], v);
    rp_sum_[k]   += double(v);
    rp_sumsq_[k] += double(v) * double(v);
}

void JointSensorimotorBridge::range_probe_report(uint64_t tick_id) const {
    if (rp_count_ == 0) return;
    const int dim_out   = 3 * group_size_;
    const int n_outputs = int(output_topics_.size());
    const double n      = double(rp_count_);
    // One line, all outputs, so the reader greps a single record.  Channel order
    // within each output is [pos,action,delta] repeated group_size_ times.
    std::printf("{\"BRIDGE_RANGE\":{\"tick\":%llu,\"n\":%llu,\"dim_out\":%d,\"legs\":{",
                (unsigned long long)tick_id, (unsigned long long)rp_count_, dim_out);
    for (int o = 0; o < n_outputs; ++o) {
        std::printf("%s\"%s\":{\"min\":[", o ? "," : "", output_topics_[o].c_str());
        for (int d = 0; d < dim_out; ++d)
            std::printf("%s%.5f", d ? "," : "", double(rp_min_[size_t(o) * size_t(dim_out) + size_t(d)]));
        std::printf("],\"max\":[");
        for (int d = 0; d < dim_out; ++d)
            std::printf("%s%.5f", d ? "," : "", double(rp_max_[size_t(o) * size_t(dim_out) + size_t(d)]));
        std::printf("],\"mean\":[");
        for (int d = 0; d < dim_out; ++d)
            std::printf("%s%.5f", d ? "," : "", rp_sum_[size_t(o) * size_t(dim_out) + size_t(d)] / n);
        std::printf("],\"std\":[");
        for (int d = 0; d < dim_out; ++d) {
            const size_t k  = size_t(o) * size_t(dim_out) + size_t(d);
            const double mu = rp_sum_[k] / n;
            const double var = std::max(0.0, rp_sumsq_[k] / n - mu * mu);
            std::printf("%s%.5f", d ? "," : "", std::sqrt(var));
        }
        std::printf("]}");
    }
    std::printf("}}}\n");
    std::fflush(stdout);
}

ParamMap JointSensorimotorBridge::current_params() const {
    ParamMap p;
    p["proprio_input_topic"] = ParamValue{proprio_input_topic_};
    p["action_topics"]       = ParamValue{action_topics_};
    p["output_topics"]       = ParamValue{output_topics_};
    std::vector<double> idx;
    idx.reserve(proprio_indices_.size());
    for (int i : proprio_indices_) idx.push_back(double(i));
    p["proprio_indices"]     = ParamValue{idx};
    p["sensor_label_prefix"] = ParamValue{sensor_label_prefix_};
    return p;
}

void JointSensorimotorBridge::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("JointSensorimotorBridge requires a non-null Bus");

    apply_param(params, "proprio_input_topic", [&](auto const& v){
        proprio_input_topic_ = get_string(v, "proprio_input_topic"); });
    apply_param(params, "sensor_label_prefix", [&](auto const& v){
        sensor_label_prefix_ = get_string(v, "sensor_label_prefix"); });
    apply_param(params, "pos_noise_sigma", [&](auto const& v){
        if (auto p = std::get_if<double>(&v)) pos_noise_sigma_ = std::max(0.0, *p);
    });
    apply_param(params, "vel_noise_sigma", [&](auto const& v){
        if (auto p = std::get_if<double>(&v)) vel_noise_sigma_ = std::max(0.0, *p);
    });
    apply_param(params, "pos_noise_tau", [&](auto const& v){
        if (auto p = std::get_if<double>(&v)) pos_noise_tau_ = std::max(1.0, *p);
    });
    apply_param(params, "seed", [&](auto const& v){
        if (auto p = std::get_if<int64_t>(&v)) pos_noise_seed_ = uint64_t(*p);
    });
    pos_noise_rng_.seed(static_cast<uint32_t>(pos_noise_seed_ ^ 0x5E4501u));
    apply_param(params, "group_size", [&](auto const& v){
        if (auto p = std::get_if<int64_t>(&v)) group_size_ = std::max(1, int(*p));
        else throw std::invalid_argument("JointSensorimotorBridge: group_size must be integer");
    });
    apply_param(params, "load_topic", [&](auto const& v){ load_topic_ = get_string(v, "load_topic"); });
    apply_param(params, "range_probe_ticks", [&](auto const& v){
        if (auto p = std::get_if<int64_t>(&v)) range_probe_ticks_ = std::max(0, int(*p));
    });

    auto it_a = params.find("action_topics");
    auto it_o = params.find("output_topics");
    auto it_i = params.find("proprio_indices");
    if (it_a == params.end() || it_o == params.end() || it_i == params.end()) {
        throw std::invalid_argument(
            "JointSensorimotorBridge: action_topics, output_topics, and proprio_indices are required");
    }
    action_topics_   = get_string_vec(it_a->second, "action_topics");
    output_topics_   = get_string_vec(it_o->second, "output_topics");
    auto idx_dbl     = get_double_vec(it_i->second, "proprio_indices");
    proprio_indices_.clear();
    proprio_indices_.reserve(idx_dbl.size());
    for (double d : idx_dbl) proprio_indices_.push_back(int(d));

    if (action_topics_.empty()) {
        throw std::invalid_argument("JointSensorimotorBridge: action_topics must be non-empty");
    }
    if (proprio_indices_.size() != action_topics_.size()) {
        throw std::invalid_argument(
            "JointSensorimotorBridge: action_topics and proprio_indices must have the same length");
    }
    if (group_size_ <= 1) {
        if (output_topics_.size() != action_topics_.size()) {
            throw std::invalid_argument(
                "JointSensorimotorBridge: output_topics length must equal action_topics length when group_size=1");
        }
    } else {
        if (action_topics_.size() % size_t(group_size_) != 0
            || output_topics_.size() != action_topics_.size() / size_t(group_size_)) {
            throw std::invalid_argument(
                "JointSensorimotorBridge: when group_size>1, action_topics.size() must be a multiple of group_size and output_topics.size() must equal action_topics.size()/group_size");
        }
    }

    int n = int(action_topics_.size());
    last_position_.assign(n, 0.0f);
    prev_position_.assign(n, 0.0f);
    last_action_.assign(n, 0.0f);

    if (range_probe_ticks_ > 0) {
        const size_t sz = output_topics_.size() * size_t(3 * group_size_);
        rp_min_.assign(sz,  std::numeric_limits<float>::max());
        rp_max_.assign(sz, -std::numeric_limits<float>::max());
        rp_sum_.assign(sz, 0.0);
        rp_sumsq_.assign(sz, 0.0);
        rp_count_ = 0;
    }

    sub_ids_.clear();
    sub_ids_.push_back(bus_->subscribe(proprio_input_topic_, SubscriptionKind::Direct,
        [this](std::string_view, MessagePtr p){ this->handle_proprio(p); }));
    if (!load_topic_.empty()) {
        last_load_.assign(output_topics_.size(), 0.0f);
        sub_ids_.push_back(bus_->subscribe(load_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){
                if (!input_allowed(p->producer_id)) return;
                auto t = std::dynamic_pointer_cast<const ProprioToken>(p);
                if (!t) return;
                for (size_t k = 0; k < last_load_.size() && int(k) < t->values.size(); ++k)
                    last_load_[k] = t->values(int(k));
                have_load_ = true;
            }));
    }
    for (int i = 0; i < n; ++i) {
        sub_ids_.push_back(bus_->subscribe(action_topics_[i], SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){ this->handle_action(i, p); }));
    }
}

void JointSensorimotorBridge::on_param_change(std::string_view key, ParamValue const& value) {
    if (key == "range_probe_ticks") {
        if (auto p = std::get_if<int64_t>(&value)) {
            const bool was_off = (range_probe_ticks_ == 0);
            range_probe_ticks_ = std::max(0, int(*p));
            if (was_off && range_probe_ticks_ > 0) {
                const size_t sz = output_topics_.size() * size_t(3 * group_size_);
                rp_min_.assign(sz,  std::numeric_limits<float>::max());
                rp_max_.assign(sz, -std::numeric_limits<float>::max());
                rp_sum_.assign(sz, 0.0);
                rp_sumsq_.assign(sz, 0.0);
                rp_count_ = 0;
            }
            return;
        }
    }
    throw std::invalid_argument(
        "JointSensorimotorBridge: param '" + std::string(key) + "' is ConstructionOnly");
}

void JointSensorimotorBridge::handle_proprio(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto p = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!p) return;
    int n = int(action_topics_.size());
    for (int i = 0; i < n; ++i) {
        int idx = proprio_indices_[i];
        if (idx >= 0 && idx < p->values.size()) {
            last_position_[i] = p->values(idx);
        }
    }
    have_proprio_ = true;
    ++total_proprio_in_;
}

void JointSensorimotorBridge::handle_action(int joint_idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto a = std::dynamic_pointer_cast<const ActionOut>(payload);
    if (!a) return;
    if (joint_idx >= 0 && joint_idx < int(last_action_.size())) {
        last_action_[joint_idx] = a->accel;
    }
    ++total_action_in_;
}

void JointSensorimotorBridge::tick(uint64_t tick_id) {
    if (!have_proprio_) return;   // wait until first proprio arrives

    int n_inputs  = int(action_topics_.size());
    int n_outputs = int(output_topics_.size());

    for (int o = 0; o < n_outputs; ++o) {
        int base    = o * group_size_;
        int dim_per = 3;
        const int load_slots = load_topic_.empty() ? 0 : 1;
        int dim_out = dim_per * group_size_ + load_slots;

        auto out = std::make_shared<ProprioToken>();
        out->tick_id     = tick_id;
        out->producer_id = id_.empty() ? std::string("joint_bridge") : id_;
        out->sensor      = sensor_label_prefix_.empty()
                            ? output_topics_[o]
                            : (sensor_label_prefix_ + "." + output_topics_[o]);
        out->values.resize(dim_out);
        for (int g = 0; g < group_size_; ++g) {
            int i = base + g;
            if (i >= n_inputs) break;
            float pos   = last_position_[i];
            float act   = last_action_[i];
            float delta = pos - prev_position_[i];   // from CLEAN positions — noise below
            // Import I4b: colored noise on the POSITION channel only.  delta is already
            // computed above from the un-noised positions, so the dose lands on one
            // channel instead of re-entering the velocity channel as a difference.
            if (pos_noise_sigma_ > 0.0) {
                if (int(pos_noise_.size()) != n_inputs) pos_noise_.assign(n_inputs, 0.0f);
                const float a = 1.0f / float(std::max(1.0, pos_noise_tau_));
                std::uniform_real_distribution<float> ud(-float(pos_noise_sigma_),
                                                          float(pos_noise_sigma_));
                pos_noise_[i] = (1.0f - a) * pos_noise_[i] + a * ud(pos_noise_rng_);
                pos = std::clamp(pos + pos_noise_[i], -1.0f, 1.0f);
            }
            // I4c: velocity-channel-only noise.  NOTE THE SCALE -- delta is a per-tick
            // difference (~0.01-0.05), not a position (~1.0), so the same nominal sigma
            // is a far larger RELATIVE perturbation here.  The body-side sigma=0.03
            // arm that produced the posture rise delivers an effective delta-noise std
            // of only ~0.0022 (an OU increment with a=1/8), so the equivalent dose here
            // is sigma ~= 0.015, not 0.03.
            if (vel_noise_sigma_ > 0.0) {
                if (int(vel_noise_.size()) != n_inputs) vel_noise_.assign(n_inputs, 0.0f);
                const float a = 1.0f / float(std::max(1.0, pos_noise_tau_));
                std::uniform_real_distribution<float> ud(-float(vel_noise_sigma_),
                                                          float(vel_noise_sigma_));
                vel_noise_[i] = (1.0f - a) * vel_noise_[i] + a * ud(pos_noise_rng_);
                delta += vel_noise_[i];
            }
            out->values(g * dim_per + 0) = pos;
            out->values(g * dim_per + 1) = act;
            out->values(g * dim_per + 2) = delta;
            if (range_probe_ticks_ > 0) {
                range_probe_accum(o, g * dim_per + 0, pos);
                range_probe_accum(o, g * dim_per + 1, act);
                range_probe_accum(o, g * dim_per + 2, delta);
            }
        }
        if (load_slots > 0)
            out->values(dim_per * group_size_) =
                (o < int(last_load_.size())) ? last_load_[o] : 0.0f;
        bus_->publish(output_topics_[o], out);
        ++total_publishes_;
    }

    if (range_probe_ticks_ > 0) {
        ++rp_count_;
        if (rp_count_ % uint64_t(range_probe_ticks_) == 0) range_probe_report(tick_id);
    }

    prev_position_ = last_position_;
}

} // namespace ogma
