#include "ogma/modules/DistressDrive.hpp"

#include <algorithm>
#include <cmath>
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
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("DistressDrive: param '" + key + "' must be numeric");
}

std::string get_string(ParamValue const& v, std::string const& key) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("DistressDrive: param '" + key + "' must be a string");
}

float clamp01(float x) { return std::clamp(x, 0.0f, 1.0f); }

} // namespace

DistressDrive::DistressDrive()  = default;
DistressDrive::~DistressDrive() = default;

std::string_view DistressDrive::type_name() const { return "DistressDrive"; }

std::vector<TopicSpec> DistressDrive::input_topics() const {
    return {
        TopicSpec{meta_topic_,      std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{staleness_topic_, std::type_index(typeid(ReflexGate)),   SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{pool_topic_,      std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{neuro_topic_,     std::type_index(typeid(NeuroState)),   SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{imu_topic_,       std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{efference_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{scent_topic_,     std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{clearance_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, /*required=*/false},
        TopicSpec{green_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, /*required=*/false},
    };
}

std::vector<TopicSpec> DistressDrive::output_topics() const {
    return {
        TopicSpec{output_topic_,   std::type_index(typeid(ReflexGate))},
        TopicSpec{interest_topic_, std::type_index(typeid(ReflexGate))},
    };
}

ParamSchema DistressDrive::params_schema() const {
    return {
        {"meta_topic", ParamMutability::ConstructionOnly,
            "Slow meta-EPM RealityToken topic (its .tle is the onset alarm).",
            ParamValue{std::string("meta.distress")}},
        {"staleness_topic", ParamMutability::ConstructionOnly,
            "StaleConfidenceDecay(meta-EPM) ReflexGate topic (sustained boredom).",
            ParamValue{std::string("cognition.meta_staleness")}},
        {"pool_topic", ParamMutability::ConstructionOnly,
            "Pooled distress-aggregate ProprioToken topic (model-free freeze signal).",
            ParamValue{std::string("distress.pool")}},
        {"neuro_topic", ParamMutability::ConstructionOnly,
            "NeurochemState topic; rising dopamine SUPPRESSES boredom (homing, don't thrash).",
            ParamValue{std::string("neuro.state")}},
        {"imu_topic", ParamMutability::ConstructionOnly,
            "Afferent IMU ProprioToken; dims 2,3 = ACTUAL world velocity (result).",
            ParamValue{std::string("reality.proprio.imu")}},
        {"efference_topic", ParamMutability::ConstructionOnly,
            "Efference-copy ProprioToken = COMMANDED velocity (intent).",
            ParamValue{std::string("reality.proprio.motor_efference")}},
        {"scent_topic", ParamMutability::ConstructionOnly,
            "Scent EPM RealityToken; its .tle = scent novelty (curiosity interest).",
            ParamValue{std::string("reality.olfactory.scent")}},
        {"clearance_topic", ParamMutability::ConstructionOnly,
            "Forward-clearance ProprioToken (1=open ahead, 0=wall ahead).",
            ParamValue{std::string("reality.proprio.clearance")}},
        {"green_topic", ParamMutability::ConstructionOnly,
            "Green-food saliency ProprioToken (green_fraction in view).",
            ParamValue{std::string("reality.proprio.green_fraction")}},
        {"w_green_int", ParamMutability::HotMutable,
            "Interest weight on green-food saliency — innate attraction to explore "
            "toward food-color (bee→flower sensory prior). Not a green-taxis: the "
            "klinokinesis run-while-rising climbs the green gradient.", ParamValue{0.5}},
        {"output_topic", ParamMutability::ConstructionOnly,
            "ReflexGate topic carrying the 0..1 boredom signal MotorEPM consumes.",
            ParamValue{std::string("cognition.boredom")}},
        {"interest_topic", ParamMutability::ConstructionOnly,
            "ReflexGate topic carrying the 0..1 curiosity interest (③ escape direction).",
            ParamValue{std::string("cognition.interest")}},
        {"w_scent_int", ParamMutability::HotMutable,
            "Interest weight on (short-term) scent novelty.", ParamValue{0.5}},
        {"w_clear_int", ParamMutability::HotMutable,
            "Interest weight on forward clearance (open ahead).", ParamValue{0.5}},
        {"scent_nov_scale", ParamMutability::HotMutable,
            "Scent-TLE above its slow baseline that counts as fully novel.", ParamValue{0.03}},
        {"scent_nov_alpha", ParamMutability::HotMutable,
            "Slow baseline EMA on scent TLE → SHORT-TERM novelty (different from the "
            "current stuck/wall state), per operator.", ParamValue{0.01}},
        {"w_mismatch", ParamMutability::HotMutable,
            "Weight of the PRIMARY reafference mismatch (commanded vs actual motion): "
            "the fraction of intended motion not realised. 1 when wedged (trying, not "
            "moving), 0 when free or at rest. Scale-free homeokinetic distress signal.",
            ParamValue{1.0}},
        {"min_effort", ParamMutability::HotMutable,
            "Commanded-speed floor below which the bug counts as 'not trying' → no "
            "mismatch (so a genuine rest is not flagged as distress).", ParamValue{0.05}},
        {"w_progress", ParamMutability::HotMutable,
            "Weight of the sustained-no-progress term (catches PARK/idle-stuck, which "
            "the reafference mismatch misses because the bug isn't trying). 1 when the "
            "net-displacement EMA has decayed (parked), 0 when getting somewhere.",
            ParamValue{1.0}},
        {"progress_alpha", ParamMutability::HotMutable,
            "EMA rate on the afferent velocity VECTOR (sets the no-progress window). "
            "Slow → brief pauses don't trip it; only sustained no-net-displacement does.",
            ParamValue{0.01}},
        {"progress_scale", ParamMutability::HotMutable,
            "Net-progress speed counted as 'getting somewhere' (normalised). |EMA(vel)| "
            "below this ramps no_progress toward 1.", ParamValue{0.2}},
        {"w_tle", ParamMutability::HotMutable,
            "Weight of the meta-EPM TLE onset spike (corroborating).", ParamValue{0.3}},
        {"w_staleness", ParamMutability::HotMutable,
            "Weight of the sustained winner-staleness term (corroborating).", ParamValue{0.3}},
        {"w_motion", ParamMutability::HotMutable,
            "Weight of the model-free pooled-state freeze term (noisy on this body; "
            "0 = telemetry only).", ParamValue{0.0}},
        {"tle_ema_alpha", ParamMutability::HotMutable,
            "EMA rate for the meta-TLE baseline; the spike is (tle - baseline). "
            "Slow so a sustained freeze keeps reading as an onset until it learns.",
            ParamValue{0.02}},
        {"tle_spike_scale", ParamMutability::HotMutable,
            "Normaliser turning (tle - baseline) into [0,1].", ParamValue{0.5}},
        {"motion_alpha", ParamMutability::HotMutable,
            "Low-pass smoother rate on the raw per-tick pooled-state motion "
            "(denoises tick-to-tick fluctuation before thresholding).",
            ParamValue{0.1}},
        {"motion_scale", ParamMutability::HotMutable,
            "FIXED free-swim motion reference. motion_inv = 1 - smoothed_motion/"
            "motion_scale (0 at typical swim speed, ramps to 1 when frozen). Set "
            "from the body's observed free-swim pooled-state motion (its natural scale).",
            ParamValue{0.001}},
        {"suppress_gain", ParamMutability::HotMutable,
            "Strength of rising-dopamine boredom suppression.", ParamValue{5.0}},
    };
}

ParamMap DistressDrive::current_params() const {
    ParamMap m;
    m["meta_topic"]      = ParamValue{meta_topic_};
    m["staleness_topic"] = ParamValue{staleness_topic_};
    m["pool_topic"]      = ParamValue{pool_topic_};
    m["neuro_topic"]     = ParamValue{neuro_topic_};
    m["imu_topic"]       = ParamValue{imu_topic_};
    m["efference_topic"] = ParamValue{efference_topic_};
    m["scent_topic"]     = ParamValue{scent_topic_};
    m["clearance_topic"] = ParamValue{clearance_topic_};
    m["output_topic"]    = ParamValue{output_topic_};
    m["interest_topic"]  = ParamValue{interest_topic_};
    m["green_topic"]     = ParamValue{green_topic_};
    m["w_scent_int"]     = ParamValue{w_scent_int_};
    m["w_clear_int"]     = ParamValue{w_clear_int_};
    m["w_green_int"]     = ParamValue{w_green_int_};
    m["scent_nov_scale"] = ParamValue{scent_nov_scale_};
    m["scent_nov_alpha"] = ParamValue{scent_nov_alpha_};
    m["w_mismatch"]      = ParamValue{w_mismatch_};
    m["min_effort"]      = ParamValue{min_effort_};
    m["w_progress"]      = ParamValue{w_progress_};
    m["progress_alpha"]  = ParamValue{progress_alpha_};
    m["progress_scale"]  = ParamValue{progress_scale_};
    m["w_tle"]           = ParamValue{w_tle_};
    m["w_staleness"]     = ParamValue{w_staleness_};
    m["w_motion"]        = ParamValue{w_motion_};
    m["tle_ema_alpha"]   = ParamValue{tle_ema_alpha_};
    m["tle_spike_scale"] = ParamValue{tle_spike_scale_};
    m["motion_alpha"]    = ParamValue{motion_alpha_};
    m["motion_scale"]    = ParamValue{motion_scale_};
    m["suppress_gain"]   = ParamValue{suppress_gain_};
    return m;
}

void DistressDrive::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    if (!bus_) throw std::invalid_argument("DistressDrive requires a non-null Bus");

    apply_param(params, "meta_topic",      [&](auto const& v){ meta_topic_      = get_string(v, "meta_topic"); });
    apply_param(params, "staleness_topic", [&](auto const& v){ staleness_topic_ = get_string(v, "staleness_topic"); });
    apply_param(params, "pool_topic",      [&](auto const& v){ pool_topic_      = get_string(v, "pool_topic"); });
    apply_param(params, "neuro_topic",     [&](auto const& v){ neuro_topic_     = get_string(v, "neuro_topic"); });
    apply_param(params, "imu_topic",       [&](auto const& v){ imu_topic_       = get_string(v, "imu_topic"); });
    apply_param(params, "efference_topic", [&](auto const& v){ efference_topic_ = get_string(v, "efference_topic"); });
    apply_param(params, "scent_topic",     [&](auto const& v){ scent_topic_     = get_string(v, "scent_topic"); });
    apply_param(params, "clearance_topic", [&](auto const& v){ clearance_topic_ = get_string(v, "clearance_topic"); });
    apply_param(params, "output_topic",    [&](auto const& v){ output_topic_    = get_string(v, "output_topic"); });
    apply_param(params, "interest_topic",  [&](auto const& v){ interest_topic_  = get_string(v, "interest_topic"); });
    apply_param(params, "green_topic",     [&](auto const& v){ green_topic_     = get_string(v, "green_topic"); });
    apply_param(params, "w_scent_int",     [&](auto const& v){ w_scent_int_     = get_double(v, "w_scent_int"); });
    apply_param(params, "w_clear_int",     [&](auto const& v){ w_clear_int_     = get_double(v, "w_clear_int"); });
    apply_param(params, "w_green_int",     [&](auto const& v){ w_green_int_     = get_double(v, "w_green_int"); });
    apply_param(params, "scent_nov_scale", [&](auto const& v){ scent_nov_scale_ = get_double(v, "scent_nov_scale"); if (scent_nov_scale_ <= 0.0) scent_nov_scale_ = 1e-6; });
    apply_param(params, "scent_nov_alpha", [&](auto const& v){ scent_nov_alpha_ = get_double(v, "scent_nov_alpha"); });
    apply_param(params, "w_mismatch",      [&](auto const& v){ w_mismatch_      = get_double(v, "w_mismatch"); });
    apply_param(params, "min_effort",      [&](auto const& v){ min_effort_      = get_double(v, "min_effort"); });
    apply_param(params, "w_progress",      [&](auto const& v){ w_progress_      = get_double(v, "w_progress"); });
    apply_param(params, "progress_alpha",  [&](auto const& v){ progress_alpha_  = get_double(v, "progress_alpha"); });
    apply_param(params, "progress_scale",  [&](auto const& v){ progress_scale_  = get_double(v, "progress_scale"); if (progress_scale_ <= 0.0) progress_scale_ = 1e-6; });
    apply_param(params, "w_tle",           [&](auto const& v){ w_tle_           = get_double(v, "w_tle"); });
    apply_param(params, "w_staleness",     [&](auto const& v){ w_staleness_     = get_double(v, "w_staleness"); });
    apply_param(params, "w_motion",        [&](auto const& v){ w_motion_        = get_double(v, "w_motion"); });
    apply_param(params, "tle_ema_alpha",   [&](auto const& v){ tle_ema_alpha_   = get_double(v, "tle_ema_alpha"); });
    apply_param(params, "tle_spike_scale", [&](auto const& v){ tle_spike_scale_ = get_double(v, "tle_spike_scale"); });
    apply_param(params, "motion_alpha",    [&](auto const& v){ motion_alpha_    = get_double(v, "motion_alpha"); });
    apply_param(params, "motion_scale",    [&](auto const& v){ motion_scale_    = get_double(v, "motion_scale"); });
    apply_param(params, "suppress_gain",   [&](auto const& v){ suppress_gain_   = get_double(v, "suppress_gain"); });

    if (tle_spike_scale_ <= 0.0) tle_spike_scale_ = 1e-6;
    if (motion_scale_    <= 0.0) motion_scale_    = 1e-6;

    if (!meta_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(meta_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_meta(p); }));
    if (!staleness_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(staleness_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_staleness(p); }));
    if (!pool_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(pool_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_pool(p); }));
    if (!neuro_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(neuro_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_neuro(p); }));
    if (!imu_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(imu_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_imu(p); }));
    if (!efference_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(efference_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_efference(p); }));
    if (!scent_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(scent_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_scent(p); }));
    if (!clearance_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(clearance_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_clearance(p); }));
    if (!green_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(green_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_green(p); }));
}

void DistressDrive::on_param_change(std::string_view key, ParamValue const& value) {
    std::string k(key);
    if      (k == "w_mismatch")      w_mismatch_      = get_double(value, k);
    else if (k == "min_effort")      min_effort_      = get_double(value, k);
    else if (k == "w_progress")      w_progress_      = get_double(value, k);
    else if (k == "progress_alpha")  progress_alpha_  = get_double(value, k);
    else if (k == "progress_scale")  { progress_scale_ = get_double(value, k); if (progress_scale_ <= 0.0) progress_scale_ = 1e-6; }
    else if (k == "w_scent_int")     w_scent_int_     = get_double(value, k);
    else if (k == "w_clear_int")     w_clear_int_     = get_double(value, k);
    else if (k == "w_green_int")     w_green_int_     = get_double(value, k);
    else if (k == "scent_nov_scale") { scent_nov_scale_ = get_double(value, k); if (scent_nov_scale_ <= 0.0) scent_nov_scale_ = 1e-6; }
    else if (k == "scent_nov_alpha") scent_nov_alpha_ = get_double(value, k);
    else if (k == "w_tle")           w_tle_           = get_double(value, k);
    else if (k == "w_staleness")     w_staleness_     = get_double(value, k);
    else if (k == "w_motion")        w_motion_        = get_double(value, k);
    else if (k == "tle_ema_alpha")   tle_ema_alpha_   = get_double(value, k);
    else if (k == "tle_spike_scale") { tle_spike_scale_ = get_double(value, k); if (tle_spike_scale_ <= 0.0) tle_spike_scale_ = 1e-6; }
    else if (k == "motion_alpha")    motion_alpha_    = get_double(value, k);
    else if (k == "motion_scale")    { motion_scale_  = get_double(value, k); if (motion_scale_ <= 0.0) motion_scale_ = 1e-6; }
    else if (k == "suppress_gain")   suppress_gain_   = get_double(value, k);
    else throw std::invalid_argument("DistressDrive: param '" + k + "' is construction-only / unknown");
}

void DistressDrive::handle_meta(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt) return;
    meta_tle_ = rt->tle;
}

void DistressDrive::handle_staleness(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto g = std::dynamic_pointer_cast<const ReflexGate>(payload);
    if (!g) return;
    staleness_ = g->active ? g->value : 0.0f;
}

void DistressDrive::handle_pool(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt) return;
    pool_cur_.assign(pt->values.begin(), pt->values.end());
    pool_seen_ = true;
}

void DistressDrive::handle_imu(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 4) return;
    // imu = [sin h, cos h, ACTUAL vx, ACTUAL vz] (afferent — real world motion)
    float vx = pt->values[2], vz = pt->values[3];
    afferent_vx_ = vx; afferent_vz_ = vz;
    afferent_speed_ = std::sqrt(vx * vx + vz * vz);
}

void DistressDrive::handle_efference(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() < 2) return;
    float vx = pt->values[0], vz = pt->values[1];   // commanded velocity (intent)
    efferent_speed_ = std::sqrt(vx * vx + vz * vz);
}

void DistressDrive::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto rt = std::dynamic_pointer_cast<const RealityToken>(payload);
    if (!rt) return;
    scent_tle_ = rt->tle;
}

void DistressDrive::handle_clearance(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() == 0) return;
    clearance_ = clamp01(pt->values[0]);
}

void DistressDrive::handle_green(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (!pt || pt->values.size() == 0) return;
    green_ = clamp01(pt->values[0]);
}

void DistressDrive::handle_neuro(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ns = std::dynamic_pointer_cast<const NeuroState>(payload);
    if (!ns) return;
    // Rising dopamine = approaching reward (potential-based) -> suppress.
    float da = ns->dopamine;
    if (da_init_) {
        float rising = std::max(0.0f, da - da_prev_);
        suppress_ = clamp01(float(suppress_gain_) * rising);
    } else {
        da_init_ = true;
    }
    da_prev_ = da;
}

void DistressDrive::tick(uint64_t tick_id) {
    // (1) Onset spike: how far the meta-EPM's TLE sits above its slow baseline.
    if (!tle_ema_init_) { tle_ema_ = meta_tle_; tle_ema_init_ = true; }
    tle_spike_ = clamp01((meta_tle_ - tle_ema_) / float(tle_spike_scale_));
    tle_ema_   = (1.0f - float(tle_ema_alpha_)) * tle_ema_ + float(tle_ema_alpha_) * meta_tle_;

    // (2) Sustained staleness is refreshed in handle_staleness (latest gate value).

    // (3) Model-free freeze: mean per-dim per-tick motion of the pooled state,
    //     LOW-PASS smoothed (denoise tick-to-tick jitter) then compared to a
    //     FIXED free-swim reference.  0 at typical swim speed, ramps to 1 as the
    //     pooled state stops moving.  Fixed scale (not a self-calibrating EMA,
    //     which would normalise the freeze away) so a sustained pin stays high.
    if (pool_seen_) {
        if (pool_prev_init_ && pool_prev_.size() == pool_cur_.size() && !pool_cur_.empty()) {
            float acc = 0.0f;
            for (size_t i = 0; i < pool_cur_.size(); ++i)
                acc += std::fabs(pool_cur_[i] - pool_prev_[i]);
            motion_raw_ = acc / float(pool_cur_.size());
            if (!motion_ema_init_) { motion_ema_ = motion_raw_; motion_ema_init_ = true; }
            else motion_ema_ = (1.0f - float(motion_alpha_)) * motion_ema_ + float(motion_alpha_) * motion_raw_;
            motion_inv_ = clamp01(1.0f - motion_ema_ / float(motion_scale_));
        }
        pool_prev_ = pool_cur_;
        pool_prev_init_ = true;
        pool_seen_ = false;
    }

    // (4) PRIMARY signal — reafference mismatch: the fraction of intended motion
    //     not actually realised.  Scale-free.  Trying hard but not moving (wedged
    //     at a wall) → ~1; trying and moving (free swim) → ~0; not trying (rest)
    //     → gated to 0 by min_effort.  This is the homeokinetic broken-contingency
    //     signal the efferent/afferent split exists to compute.
    if (efferent_speed_ > float(min_effort_))
        mismatch_ = clamp01((efferent_speed_ - afferent_speed_) / efferent_speed_);
    else
        mismatch_ = 0.0f;

    // (5) Sustained-no-progress: slow EMA of the afferent velocity VECTOR.
    //     Consistent travel keeps |EMA| up; jitter/pause cancels it toward 0.
    //     Catches the PARK/idle-stuck mode the mismatch misses (bug not trying,
    //     just not getting anywhere).  Slow EMA → brief pauses (valid, energy-
    //     conserving locomotion) don't trip it; only a sustained park does.
    if (!vel_ema_init_) { vel_ema_x_ = afferent_vx_; vel_ema_z_ = afferent_vz_; vel_ema_init_ = true; }
    else {
        vel_ema_x_ = (1.0f - float(progress_alpha_)) * vel_ema_x_ + float(progress_alpha_) * afferent_vx_;
        vel_ema_z_ = (1.0f - float(progress_alpha_)) * vel_ema_z_ + float(progress_alpha_) * afferent_vz_;
    }
    float progress = std::sqrt(vel_ema_x_ * vel_ema_x_ + vel_ema_z_ * vel_ema_z_);
    no_progress_ = clamp01(1.0f - progress / float(progress_scale_));

    // Combine: mismatch (RAM) + no_progress (PARK) are the two primary stuck
    // detectors; meta-EPM TLE/staleness corroborate; pooled motion off by
    // default (noisy).  Non-normalised so either primary alone drives boredom
    // high.  Suppressed when homing (rising DA).
    float raw = float(w_mismatch_)  * mismatch_
              + float(w_progress_)  * no_progress_
              + float(w_tle_)       * tle_spike_
              + float(w_staleness_) * staleness_
              + float(w_motion_)    * motion_inv_;
    boredom_ = clamp01(raw * (1.0f - suppress_));

    auto gate = std::make_shared<ReflexGate>();
    gate->tick_id     = tick_id;
    gate->producer_id = id_.empty() ? std::string("distress_drive") : id_;
    gate->value       = boredom_;
    gate->active      = true;   // publish every tick so MotorEPM always has a fresh value
    bus_->publish(output_topic_, gate);

    // ③ Curiosity INTEREST — where to run when escaping.  scent novelty is
    // SHORT-TERM (scent TLE above its slow baseline = different from the current
    // stuck state) per operator; clearance = open ahead.  interest high → run
    // (the escape stops tumbling and drives forward into the open/scent-rich
    // heading); interest low (flat scent AND blocked) → tumble.
    if (!scent_ema_init_) { scent_tle_ema_ = scent_tle_; scent_ema_init_ = true; }
    else scent_tle_ema_ = (1.0f - float(scent_nov_alpha_)) * scent_tle_ema_ + float(scent_nov_alpha_) * scent_tle_;
    scent_novelty_ = clamp01((scent_tle_ - scent_tle_ema_) / float(scent_nov_scale_));
    interest_ = clamp01(float(w_scent_int_) * scent_novelty_
                      + float(w_clear_int_) * clearance_
                      + float(w_green_int_) * green_);

    auto ig = std::make_shared<ReflexGate>();
    ig->tick_id     = tick_id;
    ig->producer_id = gate->producer_id;
    ig->value       = interest_;
    ig->active      = true;
    bus_->publish(interest_topic_, ig);
}

nlohmann::json DistressDrive::snapshot_state() const {
    return nlohmann::json{
        {"version",        1},
        {"tle_ema",        tle_ema_},
        {"tle_ema_init",   tle_ema_init_},
        {"motion_ema",     motion_ema_},
        {"motion_ema_init",motion_ema_init_},
        {"da_prev",        da_prev_},
        {"da_init",        da_init_},
    };
}

void DistressDrive::restore_state(nlohmann::json const& s) {
    if (s.is_null() || s.empty()) return;
    int version = s.value("version", 0);
    if (version != 1)
        throw std::runtime_error("DistressDrive::restore_state: unknown version " + std::to_string(version));
    tle_ema_         = s.value("tle_ema", tle_ema_);
    tle_ema_init_    = s.value("tle_ema_init", tle_ema_init_);
    motion_ema_      = s.value("motion_ema", motion_ema_);
    motion_ema_init_ = s.value("motion_ema_init", motion_ema_init_);
    da_prev_         = s.value("da_prev", da_prev_);
    da_init_         = s.value("da_init", da_init_);
}

} // namespace ogma
