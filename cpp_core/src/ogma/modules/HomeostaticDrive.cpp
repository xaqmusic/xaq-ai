#include "ogma/modules/HomeostaticDrive.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>
#include <variant>

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
    throw std::invalid_argument("HomeostaticDrive param '" + key + "' must be numeric");
}
int64_t get_int(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    throw std::invalid_argument("HomeostaticDrive param '" + key + "' must be integer");
}
std::vector<double> get_doubles(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("HomeostaticDrive param '" + key + "' must be vector<double>");
}
std::vector<std::string> get_strings(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("HomeostaticDrive param '" + key + "' must be vector<string>");
}

} // namespace

HomeostaticDrive::HomeostaticDrive()  = default;
HomeostaticDrive::~HomeostaticDrive() = default;

std::string_view HomeostaticDrive::type_name() const { return "HomeostaticDrive"; }

HomeostaticDrive::ChannelKind HomeostaticDrive::parse_kind(std::string const& s) {
    if (s == "energy")          return ChannelKind::Energy;
    if (s == "integrity")       return ChannelKind::Integrity;
    if (s == "novelty_ema")     return ChannelKind::NoveltyEma;
    if (s == "proprio_passive") return ChannelKind::ProprioPassive;
    if (s == "alive_pulse")     return ChannelKind::AlivePulse;
    throw std::invalid_argument("HomeostaticDrive: unknown channel kind '" + s + "'");
}

HomeostaticDrive::ChannelKind HomeostaticDrive::default_kind_for_name(std::string const& name) {
    if (name == "energy")             return ChannelKind::Energy;
    if (name == "integrity")          return ChannelKind::Integrity;
    if (name == "novelty_satiation")  return ChannelKind::NoveltyEma;
    if (name == "alive")              return ChannelKind::AlivePulse;
    return ChannelKind::ProprioPassive;
}

std::vector<TopicSpec> HomeostaticDrive::input_topics() const {
    std::vector<TopicSpec> out;
    bool wants_proprio_prefix = false;
    bool wants_consensus      = false;
    bool wants_events         = false;

    for (auto const& ch : channels_) {
        switch (ch.kind) {
            case ChannelKind::Energy:
                wants_proprio_prefix = true;
                wants_events         = true;
                break;
            case ChannelKind::Integrity:
                wants_proprio_prefix = true;
                wants_events         = true;
                break;
            case ChannelKind::NoveltyEma:
                wants_consensus      = true;
                break;
            case ChannelKind::ProprioPassive:
                wants_proprio_prefix = true;
                break;
            case ChannelKind::AlivePulse:
                wants_events         = true;
                break;
        }
    }

    if (wants_proprio_prefix)
        out.push_back(TopicSpec{topics::kRealityProprioPrefix,
                                std::type_index(typeid(ProprioToken)),
                                SubscriptionKind::Direct, /*required=*/true});
    if (wants_consensus)
        out.push_back(TopicSpec{subscribed_consensus_topic_,
                                std::type_index(typeid(ConsensusToken)),
                                SubscriptionKind::Direct, /*required=*/true});
    if (wants_events)
        out.push_back(TopicSpec{topics::kEventsPrefix,
                                std::type_index(typeid(EnvEvent)),
                                SubscriptionKind::Direct, /*required=*/false});
    return out;
}

std::vector<TopicSpec> HomeostaticDrive::output_topics() const {
    return { TopicSpec{topics::kDriveErrors, std::type_index(typeid(DriveErrors))} };
}

ParamSchema HomeostaticDrive::params_schema() const {
    return {
        {"channels",                   ParamMutability::ConstructionOnly, "Per-channel name list",          std::nullopt},
        {"setpoints",                  ParamMutability::HotMutable,       "Per-channel setpoint (same length as channels)", std::nullopt},
        {"urgency_normalizers",        ParamMutability::HotMutable,       "Per-channel |error| normalizer", std::nullopt},
        {"channel_input_topics",       ParamMutability::ConstructionOnly, "Per-channel source topic",       std::nullopt},
        {"channel_kinds",              ParamMutability::ConstructionOnly, "Per-channel kind (energy/integrity/novelty_ema/proprio_passive); default per-name", std::nullopt},
        {"energy_replenish_per_hit",   ParamMutability::HotMutable,       "Energy refill fraction per hit", ParamValue{0.4}},
        {"energy_drain_per_tick",      ParamMutability::HotMutable,       "Per-tick energy drain",          ParamValue{0.0005}},
        {"integrity_drain_per_miss",   ParamMutability::HotMutable,       "Integrity drain per miss",       ParamValue{0.05}},
        {"alive_pulse_decay_per_tick", ParamMutability::HotMutable,       "Per-tick decay for alive_pulse channels", ParamValue{0.005}},
        {"alive_pulse_replenish_per_event", ParamMutability::HotMutable,  "Deficit-fraction closed by each events.alive", ParamValue{0.5}},
        {"novelty_satiation_alpha",    ParamMutability::HotMutable,       "EMA factor for novelty_ema",     ParamValue{0.001}},
        {"urgency_clamp_lo",           ParamMutability::HotMutable,       "Lower urgency clamp",            ParamValue{0.0}},
        {"urgency_clamp_hi",           ParamMutability::HotMutable,       "Upper urgency clamp",            ParamValue{1.0}},
        {"ema_source_level",           ParamMutability::ConstructionOnly, "Which consensus.<n> drives novelty_ema", ParamValue{int64_t{0}}},
    };
}

void HomeostaticDrive::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("HomeostaticDrive requires a non-null Bus");

    // Channel declaration is required.
    auto find_required = [&](std::string const& key) -> ParamValue const& {
        auto it = params.find(key);
        if (it == params.end())
            throw std::invalid_argument("HomeostaticDrive: required param '" + key + "' missing");
        return it->second;
    };
    auto names      = get_strings(find_required("channels"),             "channels");
    auto setpts     = get_doubles(find_required("setpoints"),            "setpoints");
    auto norms      = get_doubles(find_required("urgency_normalizers"),  "urgency_normalizers");
    auto topics_v   = get_strings(find_required("channel_input_topics"), "channel_input_topics");

    if (names.size() != setpts.size() || names.size() != norms.size() || names.size() != topics_v.size())
        throw std::invalid_argument("HomeostaticDrive: channel arrays must all have the same length");

    std::vector<std::string> kinds_v;
    auto kit = params.find("channel_kinds");
    if (kit != params.end()) {
        kinds_v = get_strings(kit->second, "channel_kinds");
        if (kinds_v.size() != names.size())
            throw std::invalid_argument("HomeostaticDrive: channel_kinds length mismatch");
    }

    apply_param(params, "energy_replenish_per_hit",  [&](auto const& v){ energy_replenish_per_hit_  = get_double(v, "energy_replenish_per_hit"); });
    apply_param(params, "energy_drain_per_tick",     [&](auto const& v){ energy_drain_per_tick_     = get_double(v, "energy_drain_per_tick"); });
    apply_param(params, "integrity_drain_per_miss",  [&](auto const& v){ integrity_drain_per_miss_  = get_double(v, "integrity_drain_per_miss"); });
    apply_param(params, "alive_pulse_decay_per_tick",      [&](auto const& v){ alive_pulse_decay_per_tick_      = get_double(v, "alive_pulse_decay_per_tick"); });
    apply_param(params, "alive_pulse_replenish_per_event", [&](auto const& v){ alive_pulse_replenish_per_event_ = get_double(v, "alive_pulse_replenish_per_event"); });
    apply_param(params, "novelty_satiation_alpha",   [&](auto const& v){ novelty_satiation_alpha_   = get_double(v, "novelty_satiation_alpha"); });
    apply_param(params, "urgency_clamp_lo",          [&](auto const& v){ urgency_clamp_lo_          = get_double(v, "urgency_clamp_lo"); });
    apply_param(params, "urgency_clamp_hi",          [&](auto const& v){ urgency_clamp_hi_          = get_double(v, "urgency_clamp_hi"); });
    apply_param(params, "ema_source_level",          [&](auto const& v){ ema_source_level_          = int(get_int(v, "ema_source_level")); });

    subscribed_consensus_topic_ = std::string("consensus.") + std::to_string(ema_source_level_);

    // Build channel list.
    channels_.clear();
    channel_idx_by_name_.clear();
    for (size_t i = 0; i < names.size(); ++i) {
        Channel ch;
        ch.name               = names[i];
        ch.setpoint           = float(setpts[i]);
        ch.urgency_normalizer = float(norms[i]);
        ch.input_topic        = topics_v[i];
        ch.kind               = kinds_v.empty() ? default_kind_for_name(ch.name)
                                                : parse_kind(kinds_v[i]);
        ch.current            = ch.setpoint;   // initial value at setpoint → zero error
        channel_idx_by_name_[ch.name] = i;
        channels_.push_back(std::move(ch));
    }

    // Subscribe.  One subscription per channel for proprio sources; one
    // shared consensus subscription if any channel uses NoveltyEma; one
    // shared events.* subscription if any channel needs hit/miss.
    sub_ids_.clear();
    for (size_t i = 0; i < channels_.size(); ++i) {
        auto& ch = channels_[i];
        // NoveltyEma reads the consensus topic; AlivePulse reads only events;
        // neither needs a proprio subscription.
        if (ch.kind == ChannelKind::NoveltyEma) continue;
        if (ch.kind == ChannelKind::AlivePulse) continue;
        size_t cap_idx = i;
        sub_ids_.push_back(bus_->subscribe(ch.input_topic,
                                            SubscriptionKind::Direct,
            [this, cap_idx](std::string_view t, MessagePtr p) {
                this->handle_proprio(t, p, cap_idx);
            }));
    }
    bool wants_consensus = false;
    bool wants_events    = false;
    for (auto const& ch : channels_) {
        wants_consensus = wants_consensus || (ch.kind == ChannelKind::NoveltyEma);
        wants_events    = wants_events    ||
            (ch.kind == ChannelKind::Energy
             || ch.kind == ChannelKind::Integrity
             || ch.kind == ChannelKind::AlivePulse);
    }
    if (wants_consensus) {
        sub_ids_.push_back(bus_->subscribe(subscribed_consensus_topic_,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_consensus(t, p); }));
    }
    if (wants_events) {
        sub_ids_.push_back(bus_->subscribe(topics::kEventsPrefix,
                                            SubscriptionKind::Direct,
            [this](auto t, auto p){ this->handle_event(t, p); }));
    }
}

void HomeostaticDrive::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if (k == "energy_replenish_per_hit")  energy_replenish_per_hit_  = get_double(value, k);
    else if (k == "energy_drain_per_tick")  energy_drain_per_tick_  = get_double(value, k);
    else if (k == "integrity_drain_per_miss") integrity_drain_per_miss_ = get_double(value, k);
    else if (k == "alive_pulse_decay_per_tick")      alive_pulse_decay_per_tick_      = get_double(value, k);
    else if (k == "alive_pulse_replenish_per_event") alive_pulse_replenish_per_event_ = get_double(value, k);
    else if (k == "novelty_satiation_alpha") novelty_satiation_alpha_ = get_double(value, k);
    else if (k == "urgency_clamp_lo")       urgency_clamp_lo_      = get_double(value, k);
    else if (k == "urgency_clamp_hi")       urgency_clamp_hi_      = get_double(value, k);
    else if (k == "setpoints") {
        auto sp = get_doubles(value, k);
        if (sp.size() != channels_.size())
            throw std::invalid_argument("setpoints length mismatch");
        for (size_t i = 0; i < channels_.size(); ++i) channels_[i].setpoint = float(sp[i]);
    }
    else if (k == "urgency_normalizers") {
        auto un = get_doubles(value, k);
        if (un.size() != channels_.size())
            throw std::invalid_argument("urgency_normalizers length mismatch");
        for (size_t i = 0; i < channels_.size(); ++i) channels_[i].urgency_normalizer = float(un[i]);
    }
    else if (k == "channels" || k == "channel_input_topics" || k == "channel_kinds"
             || k == "ema_source_level")
        throw std::invalid_argument("HomeostaticDrive param '" + k + "' is ConstructionOnly");
    else
        throw std::invalid_argument("HomeostaticDrive: unknown param '" + k + "'");
}

void HomeostaticDrive::handle_proprio(std::string_view /*topic*/, MessagePtr payload, size_t channel_idx) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() == 0) return;
    if (channel_idx >= channels_.size()) return;
    channels_[channel_idx].latest_proprio_value = pt->values[0];
    channels_[channel_idx].proprio_seen         = true;
}

void HomeostaticDrive::handle_consensus(std::string_view /*topic*/, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ct = std::dynamic_pointer_cast<const ConsensusToken>(payload);
    if (!ct) return;
    for (auto& ch : channels_) {
        if (ch.kind != ChannelKind::NoveltyEma) continue;
        ch.latest_consensus_tle = ct->fused_tle;
        ch.consensus_seen       = true;
    }
}

void HomeostaticDrive::handle_event(std::string_view topic, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ev = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!ev) return;
    auto const& name = !ev->name.empty()
                        ? ev->name
                        : std::string(topic.substr(std::min(topic.size(),
                            std::string_view("events.").size())));
    if      (name == "hit")    ++pending_hits_;
    else if (name == "miss")   ++pending_misses_;
    else if (name == "alive")  ++pending_alive_;
    else if (name == "failed") ++pending_failed_;
    // Other events are ignored at this layer.
}

void HomeostaticDrive::tick(uint64_t tick_id) {
    auto out = std::make_shared<DriveErrors>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("drive") : id_;

    float max_normalized = 0.0f;
    for (auto& ch : channels_) {
        switch (ch.kind) {
            case ChannelKind::Energy: {
                if (ch.proprio_seen) ch.current = ch.latest_proprio_value;
                ch.current -= float(energy_drain_per_tick_);
                if (ch.current < 0.0f) ch.current = 0.0f;
                if (pending_hits_ > 0) {
                    float deficit = ch.setpoint - ch.current;
                    if (deficit > 0.0f) {
                        ch.current += float(pending_hits_) * float(energy_replenish_per_hit_) * deficit;
                        if (ch.current > ch.setpoint) ch.current = ch.setpoint;
                    }
                }
                break;
            }
            case ChannelKind::Integrity: {
                if (ch.proprio_seen) ch.current = ch.latest_proprio_value;
                if (pending_misses_ > 0)
                    ch.current -= float(pending_misses_) * float(integrity_drain_per_miss_);
                if (ch.current < 0.0f) ch.current = 0.0f;
                break;
            }
            case ChannelKind::NoveltyEma: {
                // EMA never resets, even in absence of new consensus deliveries.
                if (ch.consensus_seen) {
                    float a = float(novelty_satiation_alpha_);
                    ch.current = (1.0f - a) * ch.current + a * ch.latest_consensus_tle;
                }
                break;
            }
            case ChannelKind::ProprioPassive: {
                if (ch.proprio_seen) ch.current = ch.latest_proprio_value;
                break;
            }
            case ChannelKind::AlivePulse: {
                // Hard-zero on failure (single-tick spike of |error|=setpoint).
                if (pending_failed_ > 0) {
                    ch.current = 0.0f;
                    break;
                }
                ch.current -= float(alive_pulse_decay_per_tick_);
                if (ch.current < 0.0f) ch.current = 0.0f;
                if (pending_alive_ > 0) {
                    float deficit = ch.setpoint - ch.current;
                    if (deficit > 0.0f) {
                        ch.current += float(pending_alive_)
                                    * float(alive_pulse_replenish_per_event_)
                                    * deficit;
                        if (ch.current > ch.setpoint) ch.current = ch.setpoint;
                    }
                }
                break;
            }
        }

        float err = ch.current - ch.setpoint;
        out->errors[ch.name] = err;
        float norm = std::max(1e-6f, ch.urgency_normalizer);
        max_normalized = std::max(max_normalized, std::abs(err) / norm);

        // NaN guard.
        if (std::isnan(ch.current)) ch.current = ch.setpoint;

        ch.proprio_seen   = false;
        ch.consensus_seen = false;
    }

    urgency_ = std::clamp(max_normalized,
                          float(urgency_clamp_lo_),
                          float(urgency_clamp_hi_));
    out->urgency = urgency_;
    bus_->publish(topics::kDriveErrors, out);

    pending_hits_   = 0;
    pending_misses_ = 0;
    pending_alive_  = 0;
    pending_failed_ = 0;
}

float HomeostaticDrive::current_value(std::string const& channel) const {
    auto it = channel_idx_by_name_.find(channel);
    if (it == channel_idx_by_name_.end()) return 0.0f;
    return channels_[it->second].current;
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------

nlohmann::json HomeostaticDrive::snapshot_state() const {
    nlohmann::json chans = nlohmann::json::array();
    for (auto const& c : channels_) {
        chans.push_back({
            {"name",                 c.name},
            {"current",              c.current},
            {"proprio_seen",         c.proprio_seen},
            {"consensus_seen",       c.consensus_seen},
            {"latest_proprio_value", c.latest_proprio_value},
            {"latest_consensus_tle", c.latest_consensus_tle},
        });
    }
    return nlohmann::json{
        {"version",         1},
        {"urgency",         urgency_},
        {"pending_hits",    pending_hits_},
        {"pending_misses",  pending_misses_},
        {"pending_alive",   pending_alive_},
        {"pending_failed",  pending_failed_},
        {"channels",        chans},
    };
}

void HomeostaticDrive::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1) {
        throw std::runtime_error("HomeostaticDrive::restore_state: unknown snapshot version " +
                                 std::to_string(version));
    }
    urgency_         = s.value("urgency",        urgency_);
    pending_hits_    = s.value("pending_hits",   pending_hits_);
    pending_misses_  = s.value("pending_misses", pending_misses_);
    pending_alive_   = s.value("pending_alive",  pending_alive_);
    pending_failed_  = s.value("pending_failed", pending_failed_);
    if (s.contains("channels") && s["channels"].is_array()) {
        for (auto const& jc : s["channels"]) {
            std::string name = jc.value("name", std::string{});
            auto it = channel_idx_by_name_.find(name);
            if (it == channel_idx_by_name_.end()) continue;
            auto& c = channels_[it->second];
            c.current              = jc.value("current",              c.current);
            c.proprio_seen         = jc.value("proprio_seen",         c.proprio_seen);
            c.consensus_seen       = jc.value("consensus_seen",       c.consensus_seen);
            c.latest_proprio_value = jc.value("latest_proprio_value", c.latest_proprio_value);
            c.latest_consensus_tle = jc.value("latest_consensus_tle", c.latest_consensus_tle);
        }
    }
}

} // namespace ogma
