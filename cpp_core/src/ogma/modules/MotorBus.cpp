#include "ogma/modules/MotorBus.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeindex>

#include <nlohmann/json.hpp>

namespace ogma {

namespace {

template <typename Fn>
void apply_param(ParamMap const& params, std::string const& key, Fn&& fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}

std::vector<std::string> get_strings(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<std::string>>(&v)) return *p;
    throw std::invalid_argument("MotorBus param '" + key + "' must be a string list");
}
std::vector<double> get_doubles(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::vector<double>>(&v)) return *p;
    throw std::invalid_argument("MotorBus param '" + key + "' must be a number list");
}
double get_double(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("MotorBus param '" + key + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("MotorBus param '" + key + "' must be a string");
}

bool get_bool(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<bool>(&v)) return *p;
    throw std::invalid_argument("MotorBus param '" + key + "' must be bool");
}

} // namespace

MotorBus::MotorBus()  = default;
MotorBus::~MotorBus() = default;

std::string_view MotorBus::type_name() const { return "MotorBus"; }

std::vector<TopicSpec> MotorBus::input_topics() const {
    std::vector<TopicSpec> v;
    for (auto const& c : chans_) {
        v.push_back(TopicSpec{c.topic_a, std::type_index(typeid(ActionOut)),
                              SubscriptionKind::Direct, /*required=*/false});
        v.push_back(TopicSpec{c.topic_b, std::type_index(typeid(ActionOut)),
                              SubscriptionKind::Direct, /*required=*/false});
        // Optional L2-arbiter gain (so the Scheduler builds the arbiter→bus DAG edge).
        if (!gain_mod_prefix_.empty())
            v.push_back(TopicSpec{gain_mod_prefix_ + c.name, std::type_index(typeid(ProprioToken)),
                                  SubscriptionKind::Direct, /*required=*/false});
    }
    return v;
}

std::vector<TopicSpec> MotorBus::output_topics() const {
    return {
        TopicSpec{output_left_,  std::type_index(typeid(ActionOut))},
        TopicSpec{output_right_, std::type_index(typeid(ActionOut))},
    };
}

ParamSchema MotorBus::params_schema() const {
    return {
        {"influencer_names", ParamMutability::ConstructionOnly,
            "Per-influencer labels (HUD).  Parallel with kinds/topics_a/topics_b/gains.",
            ParamValue{std::vector<std::string>{}}},
        {"kinds", ParamMutability::ConstructionOnly,
            "Per-influencer mapping to motor channels: 'lr' (topic_a=left, "
            "topic_b=right) or 'steer_thrust' (topic_a=steer/differential, "
            "topic_b=thrust/common-mode → L=thrust+steer, R=thrust−steer).",
            ParamValue{std::vector<std::string>{}}},
        {"topics_a", ParamMutability::ConstructionOnly,
            "Per-influencer ActionOut input topic A (left, or steer).",
            ParamValue{std::vector<std::string>{}}},
        {"topics_b", ParamMutability::ConstructionOnly,
            "Per-influencer ActionOut input topic B (right, or thrust).",
            ParamValue{std::vector<std::string>{}}},
        {"input_scale", ParamMutability::HotMutable,
            "Per-influencer INPUT GAIN-STAGE: native full-scale that maps to unity "
            "(±1).  Each channel's raw contribution is divided by this and clamped "
            "to ±1 BEFORE its fader, so every effector reads on a common 0..1 level "
            "(default 4 = the ±4 accel convention).  Trim a quiet channel DOWN "
            "(smaller scale) to lift it to unity, or a hot one UP.",
            ParamValue{std::vector<double>{}}},
        {"gains", ParamMutability::HotMutable,
            "Per-influencer FADER gain (>=0), applied to the ±1-normalized signal.  "
            "Whisker pegged high → dominates the bus when it fires; others at unity "
            "or below.  HUD slider-driven.",
            ParamValue{std::vector<double>{}}},
        {"limit", ParamMutability::HotMutable,
            "Output full-scale (body accel range): out = limit·tanh(normalized_sum). "
            "The compressor knee is at unity (1.0); a single normalized channel at "
            "unity passes ~linearly, a loud sum saturates → masking.  Default 4.",
            ParamValue{4.0}},
        {"active_window", ParamMutability::HotMutable,
            "Ticks of silence before an influencer's contribution drops to 0 "
            "(release).  Event-driven sources (contact whisker) drop out when "
            "idle.  Default 2.",
            ParamValue{int64_t(2)}},
        {"proportional_mix", ParamMutability::HotMutable,
            "true: skip the per-channel ±1 clamp and replace the tanh compressor with a "
            "differential-PRESERVING normalize (scale L/R by their own max) so steer keeps "
            "authority at full thrust — forward yields headroom, no brake-to-turn. "
            "false (default): legacy clamp+tanh (byte-identical).",
            ParamValue{false}},
        {"turn_brake", ParamMutability::HotMutable,
            "Turn-priority brake gain (proportional_mix only): head = max(0, 1 − turn_brake·|diff|). "
            "1 = forward yields only at full steer; >1 = forward yields faster → pivot on a moderate "
            "turn → converge instead of orbiting the food.", ParamValue{1.0}},
        {"output_left", ParamMutability::ConstructionOnly,
            "Compressed left-channel ActionOut output topic.",
            ParamValue{std::string("action.left")}},
        {"output_right", ParamMutability::ConstructionOnly,
            "Compressed right-channel ActionOut output topic.",
            ParamValue{std::string("action.right")}},
        {"mod_topic", ParamMutability::ConstructionOnly,
            "SIDECHAIN modulation input (a scalar in [0,1] read from a ReflexGate, "
            "e.g. cognition.boredom).  Drives each channel's boredom_response. "
            "Empty = no sidechain (static gains).",
            ParamValue{std::string("")}},
        {"boredom_response", ParamMutability::HotMutable,
            "Per-influencer sidechain response to mod_topic: effective_gain = "
            "gain·max(0, 1 + boredom_response·m).  −1 DUCKS to 0 at m=1 (cog yields "
            "when stuck); +k BOOSTS by (1+k) (hk/escape gets authority); 0 = no "
            "response (contact reflex).  Empty/all-0 = static gains (bit-identical).",
            ParamValue{std::vector<double>{}}},
        {"authority_prefix", ParamMutability::ConstructionOnly,
            "When set, publish each channel's AUTHORITY (its EMA share of the "
            "realized bus drive ∈[0,1]) on <prefix><influencer_name> as a "
            "ProprioToken scalar.  A learner on that channel reads it to scale its "
            "learning rate by how much it actually drove the body (credit-by-"
            "authority).  Empty = no publish.",
            ParamValue{std::string("")}},
        {"gain_mod_prefix", ParamMutability::ConstructionOnly,
            "When set, subscribe each channel to <prefix><influencer_name> (a "
            "ProprioToken scalar) = an L2 arbiter's winner-take-all gain (default 1.0 = "
            "pass).  effective_gain = base_gain · sidechain · arbiter_gain, used for BOTH "
            "the mix contribution AND the authority share — so a muted channel (arbiter "
            "gain 0) gets 0 authority and its advance learning pauses.  Fresh-windowed "
            "like the action inputs (stale → 1.0).  Empty = no arbiter coupling "
            "(default-off, bit-identical).",
            ParamValue{std::string("")}},
    };
}

ParamMap MotorBus::current_params() const {
    ParamMap m;
    std::vector<std::string> names, kinds, ta, tb;
    std::vector<double> gains, scales, responses;
    for (auto const& c : chans_) {
        names.push_back(c.name); kinds.push_back(c.kind);
        ta.push_back(c.topic_a); tb.push_back(c.topic_b);
        gains.push_back(double(c.gain));
        scales.push_back(double(c.scale));
        responses.push_back(double(c.boredom_response));
    }
    m["influencer_names"] = ParamValue{names};
    m["kinds"]            = ParamValue{kinds};
    m["topics_a"]         = ParamValue{ta};
    m["topics_b"]         = ParamValue{tb};
    m["input_scale"]      = ParamValue{scales};
    m["gains"]            = ParamValue{gains};
    m["boredom_response"] = ParamValue{responses};
    m["mod_topic"]        = ParamValue{mod_topic_};
    m["authority_prefix"] = ParamValue{authority_prefix_};
    m["gain_mod_prefix"]  = ParamValue{gain_mod_prefix_};
    m["limit"]            = ParamValue{double(limit_)};
    m["active_window"]    = ParamValue{int64_t(active_window_)};
    m["proportional_mix"] = ParamValue{proportional_mix_};
    m["turn_brake"]       = ParamValue{double(turn_brake_)};
    m["output_left"]      = ParamValue{output_left_};
    m["output_right"]     = ParamValue{output_right_};
    return m;
}

void MotorBus::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("MotorBus requires a non-null Bus");

    std::vector<std::string> names, kinds, ta, tb;
    std::vector<double> gains, scales, responses;
    apply_param(params, "influencer_names", [&](auto const& v){ names = get_strings(v, "influencer_names"); });
    apply_param(params, "kinds",            [&](auto const& v){ kinds = get_strings(v, "kinds"); });
    apply_param(params, "topics_a",         [&](auto const& v){ ta    = get_strings(v, "topics_a"); });
    apply_param(params, "topics_b",         [&](auto const& v){ tb    = get_strings(v, "topics_b"); });
    apply_param(params, "input_scale",      [&](auto const& v){ scales = get_doubles(v, "input_scale"); });
    apply_param(params, "gains",            [&](auto const& v){ gains = get_doubles(v, "gains"); });
    apply_param(params, "boredom_response", [&](auto const& v){ responses = get_doubles(v, "boredom_response"); });
    apply_param(params, "mod_topic",        [&](auto const& v){ mod_topic_ = get_string(v, "mod_topic"); });
    apply_param(params, "authority_prefix", [&](auto const& v){ authority_prefix_ = get_string(v, "authority_prefix"); });
    apply_param(params, "gain_mod_prefix",  [&](auto const& v){ gain_mod_prefix_  = get_string(v, "gain_mod_prefix"); });
    apply_param(params, "limit",            [&](auto const& v){ limit_ = float(get_double(v, "limit")); });
    apply_param(params, "active_window",    [&](auto const& v){ active_window_ = int64_t(get_double(v, "active_window")); });
    apply_param(params, "proportional_mix", [&](auto const& v){ proportional_mix_ = get_bool(v, "proportional_mix"); });
    apply_param(params, "turn_brake",       [&](auto const& v){ turn_brake_ = float(get_double(v, "turn_brake")); });
    apply_param(params, "output_left",      [&](auto const& v){ output_left_  = get_string(v, "output_left"); });
    apply_param(params, "output_right",     [&](auto const& v){ output_right_ = get_string(v, "output_right"); });

    size_t n = names.size();
    if (n == 0) throw std::invalid_argument("MotorBus: influencer_names is empty");
    auto check = [&](std::vector<std::string> const& a, const char* nm){
        if (a.size() != n) throw std::invalid_argument(std::string("MotorBus: '") + nm + "' length != influencer_names"); };
    check(kinds, "kinds"); check(ta, "topics_a"); check(tb, "topics_b");
    if (gains.size() != n) throw std::invalid_argument("MotorBus: 'gains' length != influencer_names");
    if (!scales.empty() && scales.size() != n)
        throw std::invalid_argument("MotorBus: 'input_scale' length != influencer_names");
    if (!responses.empty() && responses.size() != n)
        throw std::invalid_argument("MotorBus: 'boredom_response' length != influencer_names");
    if (limit_ <= 0.0f) throw std::invalid_argument("MotorBus: limit must be > 0");

    chans_.clear();
    chans_.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (kinds[i] != "lr" && kinds[i] != "steer_thrust")
            throw std::invalid_argument("MotorBus: kind must be 'lr' or 'steer_thrust' (got '" + kinds[i] + "')");
        Channel c;
        c.name = names[i]; c.kind = kinds[i];
        c.topic_a = ta[i]; c.topic_b = tb[i];
        c.gain = float(gains[i]);
        c.scale = scales.empty() ? 4.0f : std::max(1e-3f, float(scales[i]));   // default ±4 accel → ±1
        c.boredom_response = responses.empty() ? 0.0f : float(responses[i]);
        c.eff_gain = c.gain;
        chans_.push_back(std::move(c));
    }

    // Subscribe each channel's A and B topics (capture the index).
    for (int i = 0; i < int(chans_.size()); ++i) {
        sub_ids_.push_back(bus_->subscribe(chans_[i].topic_a, SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){ this->handle_a(i, p); }));
        sub_ids_.push_back(bus_->subscribe(chans_[i].topic_b, SubscriptionKind::Direct,
            [this, i](std::string_view, MessagePtr p){ this->handle_b(i, p); }));
        // Optional L2-arbiter gain on <gain_mod_prefix><name> (default 1.0 = pass).
        if (!gain_mod_prefix_.empty())
            sub_ids_.push_back(bus_->subscribe(gain_mod_prefix_ + chans_[i].name, SubscriptionKind::Direct,
                [this, i](std::string_view, MessagePtr p){ this->handle_arb(i, p); }));
    }
    // Subscribe the sidechain modulation topic (a ReflexGate value, e.g. boredom).
    if (!mod_topic_.empty()) {
        sub_ids_.push_back(bus_->subscribe(mod_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ this->handle_mod(p); }));
    }
}

void MotorBus::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if (k == "input_scale") {
        auto s = get_doubles(value, "input_scale");
        if (s.size() != chans_.size())
            throw std::invalid_argument("MotorBus: 'input_scale' length must equal influencer count");
        for (size_t i = 0; i < chans_.size(); ++i) chans_[i].scale = std::max(1e-3f, float(s[i]));
    } else if (k == "gains") {
        auto g = get_doubles(value, "gains");
        if (g.size() != chans_.size())
            throw std::invalid_argument("MotorBus: 'gains' length must equal influencer count");
        for (size_t i = 0; i < chans_.size(); ++i) chans_[i].gain = float(g[i]);
    } else if (k == "limit") {
        float v = float(get_double(value, "limit"));
        if (v <= 0.0f) throw std::invalid_argument("MotorBus: limit must be > 0");
        limit_ = v;
    } else if (k == "proportional_mix") {
        proportional_mix_ = get_bool(value, "proportional_mix");
    } else if (k == "turn_brake") {
        turn_brake_ = float(get_double(value, "turn_brake"));
    } else if (k == "active_window") {
        active_window_ = int64_t(get_double(value, "active_window"));
    } else if (k == "boredom_response") {
        auto rsp = get_doubles(value, "boredom_response");
        if (rsp.size() != chans_.size())
            throw std::invalid_argument("MotorBus: 'boredom_response' length must equal influencer count");
        for (size_t i = 0; i < chans_.size(); ++i) chans_[i].boredom_response = float(rsp[i]);
    } else {
        throw std::invalid_argument("MotorBus param '" + k + "' is ConstructionOnly / unknown");
    }
}

void MotorBus::handle_mod(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    // ReflexGate carries the [0,1] modulation scalar (DistressDrive boredom).
    if (auto g = std::dynamic_pointer_cast<const ReflexGate>(payload))
        mod_value_ = std::clamp(g->value, 0.0f, 1.0f);
}

void MotorBus::handle_a(int idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    if (auto a = std::dynamic_pointer_cast<const ActionOut>(payload)) {
        chans_[idx].a_val  = a->accel;
        chans_[idx].a_tick = int64_t(payload->tick_id);
    }
}
void MotorBus::handle_b(int idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    if (auto a = std::dynamic_pointer_cast<const ActionOut>(payload)) {
        chans_[idx].b_val  = a->accel;
        chans_[idx].b_tick = int64_t(payload->tick_id);
    }
}
void MotorBus::handle_arb(int idx, MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    if (auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload)) {
        if (pt->values.size() > 0) {
            chans_[idx].arb_gain = pt->values[0];
            chans_[idx].arb_tick = int64_t(payload->tick_id);
        }
    }
}

void MotorBus::tick(uint64_t tick_id) {
    last_tick_ = tick_id;
    float sum_l = 0.0f, sum_r = 0.0f;
    for (auto& c : chans_) {
        bool fresh_a = (int64_t(tick_id) - c.a_tick) < active_window_;
        bool fresh_b = (int64_t(tick_id) - c.b_tick) < active_window_;
        c.last_active = fresh_a || fresh_b;
        // INPUT GAIN-STAGE: normalize each side to ±1 by the channel's scale and
        // clamp, so every effector reads on a common unity reference (no channel
        // runs past unity — incl. the steer+thrust decode which sums two axes).
        float a = (fresh_a ? c.a_val : 0.0f) / c.scale;   // stale side → 0 (release)
        float b = (fresh_b ? c.b_val : 0.0f) / c.scale;
        float cl, cr;
        if (c.kind == "steer_thrust") {
            // steer = differential (a), thrust = common-mode (b).  Matches the
            // MotorEPM cell convention: L = thrust + steer, R = thrust − steer.
            cl = b + a;
            cr = b - a;
        } else {                              // "lr"
            cl = a;
            cr = b;
        }
        if (!proportional_mix_) {
            // legacy: independent clamp — DISCARDS the steer overflow at high thrust
            // (cl=thrust+steer>1 → clipped → differential collapses → no turn authority).
            cl = std::clamp(cl, -1.0f, 1.0f);
            cr = std::clamp(cr, -1.0f, 1.0f);
        }   // proportional: leave the pair intact; the final normalize preserves the differential.
        // SIDECHAIN: boredom modulates the fader (duck cog / boost escape when stuck).
        // ARBITER GAIN: an L2 winner-take-all gain (fresh-windowed; stale → 1.0 = pass)
        // folded into the SAME effective fader → it scales BOTH the mix contribution and
        // the authority share below, so muting a channel zeroes its authority.
        bool fresh_arb = gain_mod_prefix_.empty() ? false
                                                   : (int64_t(tick_id) - c.arb_tick) < active_window_;
        float arb = fresh_arb ? std::max(0.0f, c.arb_gain) : 1.0f;
        c.eff_gain = c.gain * std::max(0.0f, 1.0f + c.boredom_response * mod_value_) * arb;
        c.last_l = c.eff_gain * cl;   // post-fader, normalized (±eff_gain) — what the meter shows
        c.last_r = c.eff_gain * cr;
        sum_l += c.last_l;
        sum_r += c.last_r;
    }
    sum_l_ = sum_l; sum_r_ = sum_r;
    if (proportional_mix_) {
        // TURN-PRIORITY mixing.  Decompose the bus into common-mode (forward) + differential
        // (turn).  The TURN keeps full authority — up to ±1, which can drive the opposite
        // paddle NEGATIVE (a pivot, like turning-in-place) — and the FORWARD yields the
        // remaining headroom.  So a tight turn while moving pivots instead of arcing wide
        // (equal-normalize shrank the turn along with the forward → the small L−R you saw).
        // Subsumption still holds: a loud reflex dominates the mix + the authority share.
        float common = 0.5f * (sum_l + sum_r);
        float diff   = 0.5f * (sum_l - sum_r);
        diff   = std::clamp(diff, -1.0f, 1.0f);          // turn gets the range first
        float head = std::max(0.0f, 1.0f - turn_brake_ * std::fabs(diff));  // forward yields (harder if >1)
        common = std::clamp(common, -head, head);
        out_norm_l_ = common + diff;                     // ∈ [-1, 1]
        out_norm_r_ = common - diff;
    } else {
        // Soft bus compressor in NORMALIZED space: unity (1.0) is the knee.  A single
        // channel at unity passes ~linearly; a loud sum saturates → masking.
        out_norm_l_ = std::tanh(sum_l);
        out_norm_r_ = std::tanh(sum_r);
    }
    out_l_ = limit_ * out_norm_l_;
    out_r_ = limit_ * out_norm_r_;
    float rawmag = std::max(std::fabs(sum_l), std::fabs(sum_r));
    float outmag = (std::fabs(sum_l) >= std::fabs(sum_r)) ? std::fabs(out_norm_l_) : std::fabs(out_norm_r_);
    gr_ = (rawmag > 1e-4f) ? std::clamp(1.0f - outmag / rawmag, 0.0f, 1.0f) : 0.0f;

    auto pub = [&](std::string const& topic, float v){
        auto out = std::make_shared<ActionOut>();
        out->tick_id     = tick_id;
        out->producer_id = id_.empty() ? std::string("motor_bus") : id_;
        out->accel       = v;
        out->source      = "bus";
        bus_->publish(topic, out);
    };
    pub(output_left_,  out_l_);
    pub(output_right_, out_r_);

    // AUTHORITY: each channel's EMA share of the realized bus drive ∈[0,1].  A
    // learner on this channel scales its LEARNING RATE by it → a muted/ducked/
    // masked channel doesn't learn from motion it didn't cause.
    // Share is by FADER (effective gain among ACTIVE channels), NOT by
    // contribution magnitude — otherwise an untrained channel issuing small
    // commands gets low authority → gated out → never bootstraps (self-
    // defeating).  The operator's "suppressed channel" = a low FADER, so a
    // channel at full gain has full authority regardless of what it commands;
    // it only loses authority when ANOTHER channel's fader actually fires
    // (e.g. the contact whisker), which correctly gates out those moments.
    float total = 0.0f;
    for (auto const& c : chans_) if (c.last_active) total += c.eff_gain;
    for (auto& c : chans_) {
        float share = (c.last_active && total > 1e-6f) ? (c.eff_gain / total) : 0.0f;
        c.authority = (1.0f - authority_alpha_) * c.authority + authority_alpha_ * share;
        if (!authority_prefix_.empty()) {
            auto a = std::make_shared<ProprioToken>();
            a->tick_id     = tick_id;
            a->producer_id = id_.empty() ? std::string("motor_bus") : id_;
            a->sensor      = "authority";
            a->values.resize(1);
            a->values[0]   = c.authority;
            bus_->publish(authority_prefix_ + c.name, a);
        }
    }
}

// Live viz (xaq_inspector meter-bridge widget): per-influencer POST-FADER level
// on each motor channel (L,R), the bus gain reduction, and the compressed output
// level — the console's meter bridge.  Two channels per meter (L/R) so the audio
// stereo analogy holds; a multi-motor body would aggregate its motors into L/R
// averages upstream.
nlohmann::json MotorBus::diag_snapshot() const {
    nlohmann::json names = nlohmann::json::array();
    nlohmann::json cl    = nlohmann::json::array();
    nlohmann::json cr    = nlohmann::json::array();
    nlohmann::json gains = nlohmann::json::array();
    nlohmann::json kinds = nlohmann::json::array();
    nlohmann::json active = nlohmann::json::array();
    for (auto const& c : chans_) {
        names.push_back(c.name);
        cl.push_back(c.last_l);            // post-fader contribution to L
        cr.push_back(c.last_r);            // post-fader contribution to R
        gains.push_back(c.gain);
        kinds.push_back(c.kind);
        active.push_back(c.last_active);
    }
    nlohmann::json scales = nlohmann::json::array();
    nlohmann::json eff = nlohmann::json::array();
    nlohmann::json arb = nlohmann::json::array();
    for (auto const& c : chans_) { scales.push_back(c.scale); eff.push_back(c.eff_gain); arb.push_back(c.arb_gain); }
    return nlohmann::json{
        {"names",     names},
        {"contrib_l", cl},          // post-fader, NORMALIZED (±eff_gain); meter 0 dB = unity 1.0
        {"contrib_r", cr},
        {"gains",     gains},
        {"eff_gain",  eff},         // sidechain + arbiter modulated gain (boredom ducks/boosts, arbiter mutes)
        {"arb_gain",  arb},         // L2-arbiter winner-take-all gain per channel (1=pass, 0=muted)
        {"mod",       mod_value_},  // sidechain modulation value (boredom)
        {"input_scale", scales},
        {"kinds",     kinds},
        {"active",    active},
        {"sum_l",     sum_l_},      // normalized sum (±1 units)
        {"sum_r",     sum_r_},
        {"out_l",     out_norm_l_}, // compressor output, NORMALIZED ±1 (meter)
        {"out_r",     out_norm_r_},
        {"out_accel_l", out_l_},    // published body accel (±limit)
        {"out_accel_r", out_r_},
        {"gr",        gr_},
        {"unity",     1.0f},        // meter 0 dB reference (normalized)
        {"limit",     limit_},      // output full-scale
    };
}

nlohmann::json MotorBus::snapshot_state() const {
    std::vector<float> gains;
    for (auto const& c : chans_) gains.push_back(c.gain);
    return nlohmann::json{
        {"version", 1},
        {"gains",   gains},        // HotMutable → may have been changed live
        {"limit",   limit_},
        {"active_window", active_window_},
    };
}

void MotorBus::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1)
        throw std::runtime_error("MotorBus::restore_state: unknown version " + std::to_string(version));
    if (s.contains("gains") && s["gains"].is_array()) {
        auto g = s["gains"].get<std::vector<float>>();
        for (size_t i = 0; i < chans_.size() && i < g.size(); ++i) chans_[i].gain = g[i];
    }
    limit_         = s.value("limit", limit_);
    active_window_ = s.value("active_window", active_window_);
}

} // namespace ogma
