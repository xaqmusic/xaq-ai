#include "ogma/modules/RunTumbleNav.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ogma {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float wrap_pi(float a) { return std::atan2(std::sin(a), std::cos(a)); }
// Interpolate on the circle from angle a (w=0) to angle b (w=1) — a lerp of the two unit
// vectors, re-angled. Used to bias the tumble centre from the current heading toward the
// believed up-gradient heading μ by the belief precision w=R (KF6). w=0 returns a exactly.
inline float circ_interp(float a, float b, float w) {
    float x = (1.0f - w) * std::cos(a) + w * std::cos(b);
    float y = (1.0f - w) * std::sin(a) + w * std::sin(b);
    if (x == 0.0f && y == 0.0f) return a;   // antipodal degenerate → keep a
    return std::atan2(y, x);
}

template <class Fn>
void apply_param(ParamMap const& params, char const* key, Fn fn) {
    auto it = params.find(key);
    if (it != params.end()) fn(it->second);
}
int64_t get_int(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<int64_t>(&v)) return *p;
    if (auto p = std::get_if<double>(&v))  return int64_t(*p);
    throw std::invalid_argument("RunTumbleNav: param '" + k + "' must be integer");
}
double get_double(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<double>(&v))  return *p;
    if (auto p = std::get_if<int64_t>(&v)) return double(*p);
    throw std::invalid_argument("RunTumbleNav: param '" + k + "' must be numeric");
}
std::string get_string(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<std::string>(&v)) return *p;
    throw std::invalid_argument("RunTumbleNav: param '" + k + "' must be a string");
}
bool get_bool(ParamValue const& v, std::string const& k) {
    if (auto p = std::get_if<bool>(&v))    return *p;
    if (auto p = std::get_if<int64_t>(&v)) return *p != 0;
    throw std::invalid_argument("RunTumbleNav: param '" + k + "' must be bool");
}
}  // namespace

std::string_view RunTumbleNav::type_name() const { return "RunTumbleNav"; }

std::vector<TopicSpec> RunTumbleNav::input_topics() const {
    // eat_topic (a GROUND-TRUTH consummatory event, NOT the scent-progress reward on events.hit)
    // calibrates eat_scent_ = the self-reported CONFIDENCE only; the run/tumble POLICY is reward-free
    // (baseline_/p_tumble/output never read the eat).
    return {
        TopicSpec{scent_topic_,   std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{heading_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{vel_topic_,     std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false},
        TopicSpec{eat_topic_,     std::type_index(typeid(EnvEvent)),     SubscriptionKind::Direct, false},
    };
}

std::vector<TopicSpec> RunTumbleNav::output_topics() const {
    return {
        TopicSpec{output_topic_,     std::type_index(typeid(ProprioToken))},
        TopicSpec{confidence_topic_, std::type_index(typeid(ProprioToken))},  // self-reported capability ∈[0,1]
    };
}

ParamSchema RunTumbleNav::params_schema() const {
    return {
        {"scent_topic",   ParamMutability::ConstructionOnly, "SCALAR scent concentration (no ring).", ParamValue{std::string("reality.proprio.scent_max")}},
        {"heading_topic", ParamMutability::ConstructionOnly, "Egomotion heading (run-direction frame).", ParamValue{std::string("reality.proprio.heading")}},
        {"vel_topic",     ParamMutability::ConstructionOnly, "Egomotion velocity (forward = stuck check).", ParamValue{std::string("reality.proprio.vel_ego")}},
        {"eat_topic",     ParamMutability::ConstructionOnly, "GROUND-TRUTH consummatory event (a REAL eat, e.g. events.eat) — calibrates eat_scent_ (self-reported confidence ONLY; the run/tumble policy stays reward-free). Distinct from events.hit, which in the Cell is overloaded with the scent-progress inferential reward and would mis-calibrate the scale.", ParamValue{std::string("events.eat")}},
        {"output_topic",  ParamMutability::ConstructionOnly, "Chosen heading [vx,vy] → HeadingController.", ParamValue{std::string("percept.runtumble_heading")}},
        {"confidence_topic", ParamMutability::ConstructionOnly, "SCALAR self-reported capability ∈[0,1] → EFEArbiter (klino's expected pragmatic gain; →0 when blind).", ParamValue{std::string("percept.klino_confidence")}},
        {"baseline_alpha", ParamMutability::HotMutable, "Methylation adaptation rate (EMA of scent = the prediction baseline).", ParamValue{0.05}},
        {"scale_alpha",   ParamMutability::HotMutable, "EMA rate of the running |error| normaliser.", ParamValue{0.02}},
        {"tumble_base",   ParamMutability::HotMutable, "Per-tick tumble probability at flat gradient (mean run ~1/this).", ParamValue{0.1}},
        {"tumble_gain",   ParamMutability::HotMutable, "How strongly the normalised CHANGE (temporal gradient) modulates tumble probability.", ParamValue{0.1}},
        {"tumble_level_gain", ParamMutability::HotMutable, "ORTHOKINESIS ON THE TUMBLE RATE (coarse→fine homing): how strongly the scent LEVEL (proximity cap∈[0,1], self-calibrated from the bug's own eats) raises the tumble rate → runs SHORTEN near food for fine sampling at the close. p_tumble = clamp(tumble_base·(1+tumble_level_gain·cap) − tumble_gain·error_n, min, max). Keyed on the LEVEL not the change, so the long approach runs are preserved (cap≈0 far → unchanged). Complements the HeadingController speed_gate (orthokinesis on the advance). 0 (default) = off.", ParamValue{0.0}},
        {"tumble_min",    ParamMutability::HotMutable, "Tumble-probability clamp (low end).", ParamValue{0.0}},
        {"tumble_max",    ParamMutability::HotMutable, "Tumble-probability clamp (high end).", ParamValue{0.5}},
        {"peak_decay",    ParamMutability::HotMutable, "STRUCTURAL: decay of the slow scent-magnitude memory (the PRE-EAT bootstrap capability denominator). Halflife ~1400 ticks so a wilderness stretch doesn't erase the remembered food-scent scale → the bootstrap capability stays an honest current/typical-smell ratio, not 'recent max'.", ParamValue{0.0005}},
        {"eat_scent_alpha", ParamMutability::HotMutable, "Per-EAT EMA rate of eat_scent_ (the scent at which klino actually eats = the calibrated capability denominator). Fast per-event (hits are rare) → converges over ~5 eats. The eat ONLY calibrates confidence; the run/tumble policy stays reward-free.", ParamValue{0.2}},
        {"tumble_range",  ParamMutability::HotMutable, "Max reorient per tumble (rad); ±π/2 keeps the turn forward (no reverse).", ParamValue{1.5708}},
        {"stuck_vel_thresh", ParamMutability::HotMutable, "Forward |vel| below this during a run = blocked.", ParamValue{0.5}},
        {"stuck_ticks",   ParamMutability::HotMutable, "Sustained blocked ticks → forced tumble.", ParamValue{int64_t{10}}},
        {"shuffle",       ParamMutability::HotMutable, "ABLATION: gradient-blind tumbling at the flat tumble_base rate (same mean rate, no gradient modulation) — isolates gradient-awareness.", ParamValue{false}},
        {"run_commit",    ParamMutability::HotMutable, "KF1 RUN INTEGRITY (opt-in). The per-tick tumble draw re-tumbles during the multi-tick turn transient after a tumble (thrust≈0 while reorienting → no travel → p_tumble≈base → most runs die before they start = the random walk). When true, a tumble decision belongs to a RUN: while reorienting toward run_dir suppress the tumble draw AND the stuck counter; resume once the body faces it. The 'clock' is the executed reorientation, not a fixed cycle. Also makes the stuck check efference-matched (fires only when EXECUTING a run but not moving → no mid-turn false tumbles). Default false = per-tick draw (byte-identical).", ParamValue{false}},
        {"dir_belief",    ParamMutability::HotMutable, "KF6 DIRECTIONAL BELIEF (opt-in). Morph the tumble from a uniform draw to a belief-BIASED choice (kinesis → reactive taxis). The gradient direction is INFERRED from the agent's own run outcomes (a run that raised scent makes a similar direction a better bet), never read from a percept. Circular accumulator g=EMA(outcome·unit(run_dir)); R=|g|/EMA(|outcome|)∈[0,1] = directional consistency = belief precision (self-scaled, no tuned constant); μ=atan2(g). The tumble centre blends from the current heading toward μ by w=R (R≈0 tabula-rasa → uniform draw = byte-identical; R→1 → centre on μ). Adaptive precision → free perturbation recovery (bad runs collapse R → re-broaden → re-infer). Default false. Most meaningful with run_commit (needs real runs to credit a direction).", ParamValue{false}},
        {"dir_lr",        ParamMutability::HotMutable, "KF6 EMA rate of the directional belief (the short timescale over which the tumble morphs random→predicted; ~1/dir_lr runs). Only active when dir_belief.", ParamValue{0.1}},
        {"dir_decay_on_loss", ParamMutability::HotMutable, "KF6 belief COLLAPSE-on-loss (default on): each tick the scent is FALLING (methylation error<0), bleed the directional accumulator toward 0 ∝|error| so the belief precision drops WITHIN a run and re-infers — prevents a stale heading locking after a source relocation / at a sharp bend (the L-maze regression). Asymmetric with the slow per-run build. off = build-only (regresses on moving food).", ParamValue{true}},
        {"authority_topic", ParamMutability::ConstructionOnly, "KF3 (needed for run_commit under the L2 arbiter): MotorBus authority share for the klino channel (ProprioToken scalar ∈[0,1], e.g. motor.bus.authority.klino). While MUTED (below authority_floor) klino is not driving the body → COAST: track the actual heading (no stuck reorient) and suppress the tumble decision + belief update (a run it didn't drive must not be credited to it, §5). Empty (default) = no authority read (byte-identical).", ParamValue{std::string("")}},
        {"authority_floor", ParamMutability::HotMutable, "KF3: authority below this = muted (another loop or a reflex has the bus).", ParamValue{0.5}},
        {"master_seed",   ParamMutability::ConstructionOnly, "RNG seed.", ParamValue{int64_t{11}}},
    };
}

ParamMap RunTumbleNav::current_params() const {
    ParamMap m;
    m["scent_topic"] = ParamValue{scent_topic_};
    m["heading_topic"] = ParamValue{heading_topic_};
    m["vel_topic"] = ParamValue{vel_topic_};
    m["eat_topic"] = ParamValue{eat_topic_};
    m["output_topic"] = ParamValue{output_topic_};
    m["confidence_topic"] = ParamValue{confidence_topic_};
    m["baseline_alpha"] = ParamValue{double(baseline_alpha_)};
    m["scale_alpha"] = ParamValue{double(scale_alpha_)};
    m["tumble_base"] = ParamValue{double(tumble_base_)};
    m["tumble_gain"] = ParamValue{double(tumble_gain_)};
    m["tumble_level_gain"] = ParamValue{double(tumble_level_gain_)};
    m["tumble_min"] = ParamValue{double(tumble_min_)};
    m["tumble_max"] = ParamValue{double(tumble_max_)};
    m["peak_decay"] = ParamValue{double(peak_decay_)};
    m["eat_scent_alpha"] = ParamValue{double(eat_scent_alpha_)};
    m["tumble_range"] = ParamValue{double(tumble_range_)};
    m["stuck_vel_thresh"] = ParamValue{double(stuck_vel_thresh_)};
    m["stuck_ticks"] = ParamValue{int64_t(stuck_ticks_)};
    m["shuffle"] = ParamValue{shuffle_};
    m["run_commit"] = ParamValue{run_commit_};
    m["dir_belief"] = ParamValue{dir_belief_};
    m["dir_lr"] = ParamValue{double(dir_lr_)};
    m["dir_decay_on_loss"] = ParamValue{dir_decay_on_loss_};
    m["authority_topic"] = ParamValue{authority_topic_};
    m["authority_floor"] = ParamValue{double(authority_floor_)};
    m["master_seed"] = ParamValue{int64_t(master_seed_)};
    return m;
}

void RunTumbleNav::on_param_change(std::string_view key, ParamValue const& value) {
    auto k = std::string(key);
    if      (k == "baseline_alpha") baseline_alpha_ = float(get_double(value, k));
    else if (k == "scale_alpha")    scale_alpha_    = float(get_double(value, k));
    else if (k == "tumble_base")    tumble_base_    = float(get_double(value, k));
    else if (k == "tumble_gain")    tumble_gain_    = float(get_double(value, k));
    else if (k == "tumble_level_gain") tumble_level_gain_ = float(get_double(value, k));
    else if (k == "tumble_min")     tumble_min_     = float(get_double(value, k));
    else if (k == "tumble_max")     tumble_max_     = float(get_double(value, k));
    else if (k == "peak_decay")     peak_decay_     = float(get_double(value, k));
    else if (k == "eat_scent_alpha") eat_scent_alpha_ = float(get_double(value, k));
    else if (k == "tumble_range")   tumble_range_   = float(get_double(value, k));
    else if (k == "stuck_vel_thresh") stuck_vel_thresh_ = float(get_double(value, k));
    else if (k == "stuck_ticks")    stuck_ticks_    = std::max(1, int(get_int(value, k)));
    else if (k == "shuffle")        shuffle_        = get_bool(value, k);
    else if (k == "run_commit")     run_commit_     = get_bool(value, k);
    else if (k == "dir_belief")     dir_belief_     = get_bool(value, k);
    else if (k == "dir_lr")         dir_lr_         = float(get_double(value, k));
    else if (k == "dir_decay_on_loss") dir_decay_on_loss_ = get_bool(value, k);
    else if (k == "authority_floor") authority_floor_ = float(get_double(value, k));
    else throw std::invalid_argument("RunTumbleNav: param '" + k + "' is construction-only / unknown");
}

void RunTumbleNav::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    apply_param(params, "scent_topic",   [&](auto const& v){ scent_topic_   = get_string(v,"scent_topic"); });
    apply_param(params, "heading_topic", [&](auto const& v){ heading_topic_ = get_string(v,"heading_topic"); });
    apply_param(params, "vel_topic",     [&](auto const& v){ vel_topic_     = get_string(v,"vel_topic"); });
    apply_param(params, "eat_topic",     [&](auto const& v){ eat_topic_     = get_string(v,"eat_topic"); });
    apply_param(params, "output_topic",  [&](auto const& v){ output_topic_  = get_string(v,"output_topic"); });
    apply_param(params, "confidence_topic", [&](auto const& v){ confidence_topic_ = get_string(v,"confidence_topic"); });
    apply_param(params, "baseline_alpha",[&](auto const& v){ baseline_alpha_= float(get_double(v,"baseline_alpha")); });
    apply_param(params, "scale_alpha",   [&](auto const& v){ scale_alpha_   = float(get_double(v,"scale_alpha")); });
    apply_param(params, "tumble_base",   [&](auto const& v){ tumble_base_   = float(get_double(v,"tumble_base")); });
    apply_param(params, "tumble_gain",   [&](auto const& v){ tumble_gain_   = float(get_double(v,"tumble_gain")); });
    apply_param(params, "tumble_level_gain", [&](auto const& v){ tumble_level_gain_ = float(get_double(v,"tumble_level_gain")); });
    apply_param(params, "tumble_min",    [&](auto const& v){ tumble_min_    = float(get_double(v,"tumble_min")); });
    apply_param(params, "tumble_max",    [&](auto const& v){ tumble_max_    = float(get_double(v,"tumble_max")); });
    apply_param(params, "peak_decay",    [&](auto const& v){ peak_decay_    = float(get_double(v,"peak_decay")); });
    apply_param(params, "eat_scent_alpha",[&](auto const& v){ eat_scent_alpha_ = float(get_double(v,"eat_scent_alpha")); });
    apply_param(params, "tumble_range",  [&](auto const& v){ tumble_range_  = float(get_double(v,"tumble_range")); });
    apply_param(params, "stuck_vel_thresh",[&](auto const& v){ stuck_vel_thresh_ = float(get_double(v,"stuck_vel_thresh")); });
    apply_param(params, "stuck_ticks",   [&](auto const& v){ stuck_ticks_   = std::max(1, int(get_int(v,"stuck_ticks"))); });
    apply_param(params, "shuffle",       [&](auto const& v){ shuffle_       = get_bool(v,"shuffle"); });
    apply_param(params, "run_commit",    [&](auto const& v){ run_commit_    = get_bool(v,"run_commit"); });
    apply_param(params, "dir_belief",    [&](auto const& v){ dir_belief_    = get_bool(v,"dir_belief"); });
    apply_param(params, "dir_lr",        [&](auto const& v){ dir_lr_        = float(get_double(v,"dir_lr")); });
    apply_param(params, "dir_decay_on_loss", [&](auto const& v){ dir_decay_on_loss_ = get_bool(v,"dir_decay_on_loss"); });
    apply_param(params, "authority_topic", [&](auto const& v){ authority_topic_ = get_string(v,"authority_topic"); });
    apply_param(params, "authority_floor", [&](auto const& v){ authority_floor_ = float(get_double(v,"authority_floor")); });
    apply_param(params, "master_seed",   [&](auto const& v){ master_seed_   = uint64_t(get_int(v,"master_seed")); });

    rng_.seed(master_seed_);

    if (!scent_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(scent_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_scent(p); }));
    if (!heading_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(heading_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_heading(p); }));
    if (!vel_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(vel_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_vel(p); }));
    // eat → calibrate eat_scent_ (self-reported CONFIDENCE only; the run/tumble policy is reward-free).
    if (!eat_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(eat_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_eat(p); }));
    // KF3 — MotorBus authority for the klino channel (only when wired; byte-identical otherwise).
    if (!authority_topic_.empty())
        sub_ids_.push_back(bus_->subscribe(authority_topic_, SubscriptionKind::Direct,
            [this](std::string_view, MessagePtr p){ handle_authority(p); }));
}

void RunTumbleNav::handle_scent(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { smax_ = float(pt->values[0]); have_scent_ = true; }
}
void RunTumbleNav::handle_heading(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) heading_ = float(pt->values[0]);
}
void RunTumbleNav::handle_vel(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() >= 2) vel_fwd_ = float(pt->values[1]);  // [vx_ego, vy_ego], +fwd
}
void RunTumbleNav::handle_authority(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(payload);
    if (pt && pt->values.size() > 0) { authority_ = float(pt->values[0]); have_authority_ = true; }
}
void RunTumbleNav::handle_eat(MessagePtr payload) {
    if (!input_allowed(payload->producer_id)) return;
    auto ev = std::dynamic_pointer_cast<const EnvEvent>(payload);
    if (!ev) return;
    // Capture the scent AT the eat moment. The food moves the instant the bug eats (food_alternate),
    // so by the next tick smax_ has already collapsed to the post-eat low value — sampling here (when
    // the eat event arrives, bug still on the food) records the true at-eat scent scale.
    eat_scent_sample_ = smax_;
    eat_pending_ = true;
}

void RunTumbleNav::tick(uint64_t tick_id) {
    if (!have_run_dir_) { run_dir_abs_ = heading_; have_run_dir_ = true; baseline_ = smax_; run_start_scent_ = smax_; }

    // --- E. coli methylation: per-tick gradient-modulated tumble probability ---
    // baseline_ = methylation level = a leaky PREDICTION of the scent the bug expects.
    // error = scent − baseline_ = the prediction error (rising > 0 means "getting better").
    float error = smax_ - baseline_;                              // prediction error (rising > 0)
    baseline_ += baseline_alpha_ * (smax_ - baseline_);           // methylation adapts toward current scent (continuous, no reset)
    error_scale_ += scale_alpha_ * (std::fabs(error) - error_scale_);
    float error_n = error / (error_scale_ + 1e-6f);               // dimensionless
    last_error_ = error_n;

    // KF6 belief COLLAPSE-ON-LOSS: the scent is FALLING (error_n < 0 = the committed direction is
    // not working) → bleed the directional accumulator toward 0 at a rate ∝ |error_n|, so the belief
    // precision R drops WITHIN the run and the next tumble re-infers instead of re-committing to a
    // stale heading (the L-maze / relocating-source fix). Asymmetric with the slow per-run build:
    // quick to abandon, deliberate to form. Uses the module's own prediction error; no new constant.
    if (dir_belief_ && dir_decay_on_loss_ && error_n < 0.0f) {
        float keep = 1.0f - dir_lr_ * std::min(-error_n, 1.0f);
        belief_x_ *= keep; belief_y_ *= keep;   // |g| shrinks faster than belief_absw_ → R collapses
        float bmag = std::sqrt(belief_x_ * belief_x_ + belief_y_ * belief_y_);
        last_R_  = std::clamp(bmag / (belief_absw_ + 1e-6f), 0.0f, 1.0f);
        last_mu_ = std::atan2(belief_y_, belief_x_);
    }

    // ORTHOKINESIS on the tumble rate: the scent LEVEL (proximity cap∈[0,1], from the PREVIOUS tick —
    // a slow signal, the 1-tick lag is negligible) raises the base tumble rate so runs SHORTEN near
    // food (fine sampling at the close). Keyed on the level, not the change → long approach runs
    // preserved (cap≈0 far). tumble_level_gain=0 → tumble_base·1 = unchanged (byte-identical).
    float base_rate = tumble_base_ * (1.0f + tumble_level_gain_ * capability_);
    float p_tumble = std::clamp(base_rate - tumble_gain_ * error_n, tumble_min_, tumble_max_);
    last_p_tumble_ = p_tumble;

    // --- SELF-REPORTED CAPABILITY (the nested blanket §2.1): klino assesses its own state ---
    // klino is a chemotaxer: with no scent gradient its expected PRAGMATIC gain is ~0. It tells the
    // arbiter so — honest capability, not suppression. capability = current-smell / typical-EAT-smell
    // ∈[0,1]: →0 when blind (numerator smax_ ≈ 0), →~1 in its own eating range.
    //
    // The denominator is EAT-CALIBRATED: klino learns eat_scent_ = EMA(scent at the moment it eats)
    // — the scent scale at which it ACTUALLY scores food (a GROUND-TRUTH consummatory event on
    // eat_topic, NOT the scent-progress reward on events.hit, which fires all through the approach and
    // would peg the scale far too low). So capability reaches ~1 the instant the bug is in its own
    // eating range, which is exactly what the L2 arbiter needs to let klino's value reach the planner's
    // scale and OWN the close (the field is source-normalised but the bug samples a sub-1 scent at its
    // closest approach; eat_scent_ pins that closest-approach scale to 1). This is the ONLY use of the
    // eat: the run/tumble policy above is untouched (reward-free). Before the first eat there is no eat
    // scale, so a slow-decaying running scent peak bootstraps the ratio.
    if (eat_pending_) {
        float s = eat_scent_sample_;   // scent captured AT the eat moment (see handle_eat)
        if (!have_eat_scent_) { eat_scent_ = s; have_eat_scent_ = true; }
        else                  eat_scent_ += eat_scent_alpha_ * (s - eat_scent_);
        eat_pending_ = false;
    }
    // CAPABILITY = HONEST proximity ∈[0,1]: current scent as a fraction of the food-scent scale.
    // POST-EAT: eat_scent_ (the scent the bug actually eats at) is the scale → cap→1 in the eating range
    // (klino owns the close). PRE-EAT: the maze diffusion field pins each source to 1.0 and decays along
    // free cells, so smax_ IS the fraction-of-source-strength the nose reads → use 1.0 as the scale (the
    // field's own normalisation, not a hand-set constant). The old pre-eat bootstrap (smax_/scent_peak_,
    // running max) SATURATED cap→1 on any AWASH field (baseline scent a large fraction of the peak, e.g.
    // the quad ~0.33 vs ~0.55) because the bug is ~always at its own recent max → the arbiter thought
    // klino was fully capable everywhere (g_prag_klino=hunger·cap ⇒ forages blind) AND the orthokinesis
    // crank pegged the tumble rate → runs collapsed to ~1.4 ticks. Referencing to the field's source
    // scale (1.0) makes cap≈0.33 at the baseline (klino honestly modest, runs stay long) and ≈0.55 near
    // food, rising to ~1 once eat_scent_ calibrates the true closest-approach scale post-eat.
    scent_peak_  = std::max(smax_, scent_peak_ * (1.0f - peak_decay_));   // (telemetry: speak)
    float cap_denom = have_eat_scent_ ? eat_scent_ : 1.0f;
    capability_  = std::clamp(smax_ / (cap_denom + 1e-4f), 0.0f, 1.0f);

    // --- KF0/KF1: reorientation state — are we EXECUTING a run, or still turning to face run_dir? ---
    // (geometric, always computed for telemetry; only GATES behaviour when run_commit_.)
    constexpr float kTurnExit = 0.6283f;   // ~36°: the codebase turn-commit exit (planner/playloop) = "aligned enough to be running"
    // KF3: while MUTED (another loop / a reflex drives the body), klino is not in control — COAST:
    // adopt the body's actual heading as the committed run_dir (so the run_commit reorientation latch
    // cannot freeze on a stale direction), and start a fresh run (no outcome is credited to klino for
    // motion it did not cause, §5). On regaining authority it resumes aligned, executing a fresh run.
    // Only active when a KF feature that needs it is on → authority_topic can be wired in the shipped
    // config (so the sliders work when toggled) while the flags-off default stays byte-identical.
    bool muted = (run_commit_ || dir_belief_) && have_authority_ && (authority_ < authority_floor_);
    if (muted) { run_dir_abs_ = heading_; reorienting_ = false; run_ticks_ = 0; run_start_scent_ = smax_; }
    float pre_delta = wrap_pi(run_dir_abs_ - heading_);
    bool  in_turn   = std::fabs(pre_delta) > kTurnExit;          // still reorienting toward the committed run_dir
    if (run_commit_ && reorienting_ && !in_turn) reorienting_ = false;   // the run is now underway
    bool  executing = !(run_commit_ && reorienting_);            // run_commit_ off → always executing (byte-identical)
    turn_frac_ += 0.02f * ((in_turn ? 1.0f : 0.0f) - turn_frac_);       // KF0 telemetry EMA

    // tumble-on-stuck (honest egomotion action-consequence — KEEP): commanded a run but didn't move → blocked.
    // KF2 (efference-matched, when run_commit_): accumulate ONLY while EXECUTING a run (intending to advance);
    // hold the counter while reorienting (intending to TURN, low speed is expected) → no mid-turn false tumbles.
    bool forced = false;
    if (muted) {
        stuck_counter_ = 0;   // KF3: not driving → the body's low speed isn't klino being blocked
    } else if (!run_commit_ || executing) {
        if (std::fabs(vel_fwd_) < stuck_vel_thresh_) ++stuck_counter_;
        else stuck_counter_ = 0;
    }   // else (run_commit_ && reorienting): hold the counter (neither accumulate nor reset)
    if (stuck_counter_ >= stuck_ticks_) { forced = true; stuck_counter_ = 0; ++forced_tumbles_; if (in_turn) ++forced_in_turn_; }

    // --- decide whether to tumble ---
    // KF1: while reorienting toward run_dir (not yet executing), COMMIT to reaching it — no tumble
    // decision, no rng draw. Otherwise the per-tick methylation/shuffle draw as before.
    bool do_tumble;
    if (muted) {
        do_tumble = false;   // KF3: coasting (not driving the body) → no tumble decision, no belief credit
    } else if (run_commit_ && !executing) {
        do_tumble = false;   // reorienting: hold the committed run_dir until the body faces it
    } else {
        std::uniform_real_distribution<float> u01(0.0f, 1.0f);
        // ABLATION: gradient-BLIND tumbling at the flat base rate (same mean rate, no gradient
        // modulation) — isolates whether the gradient-awareness helps.
        do_tumble = shuffle_ ? (u01(rng_) < tumble_base_)
                             : (forced || u01(rng_) < p_tumble);
    }

    if (do_tumble) {
        // --- KF0: classify the run that just ended by whether it RAISED or LOWERED scent ---
        float run_delta = smax_ - run_start_scent_;
        float len = float(run_ticks_);
        if      (run_delta > 0.0f) run_len_up_   += 0.05f * (len - run_len_up_);
        else if (run_delta < 0.0f) run_len_down_ += 0.05f * (len - run_len_down_);

        // --- KF6: infer the up-gradient direction from THIS run's own outcome (before choosing anew) ---
        // The direction we just ran, weighted by whether scent rose, EMAs into a circular accumulator.
        // R = |g|/EMA(|outcome|) ∈[0,1] is the mean resultant length = directional consistency = the
        // belief precision, SELF-SCALED from the agent's own consequences (no tuned crossover, §2.3/§6).
        if (dir_belief_) {
            float outcome_n = std::clamp(run_delta / (error_scale_ + 1e-6f), -4.0f, 4.0f);
            float ux = std::cos(run_dir_abs_), uy = std::sin(run_dir_abs_);   // circular embedding of the run direction
            belief_x_    += dir_lr_ * (outcome_n * ux - belief_x_);
            belief_y_    += dir_lr_ * (outcome_n * uy - belief_y_);
            belief_absw_ += dir_lr_ * (std::fabs(outcome_n) - belief_absw_);
            float bmag = std::sqrt(belief_x_ * belief_x_ + belief_y_ * belief_y_);
            last_R_  = std::clamp(bmag / (belief_absw_ + 1e-6f), 0.0f, 1.0f);
            last_mu_ = std::atan2(belief_y_, belief_x_);
        }

        // --- choose the new run direction: uniform off the current heading, CENTRE biased toward μ and
        // cone TIGHTENED by the belief precision R (a confident belief makes a predicted choice, not a
        // random one). R≈0 → the full uniform cone off the current heading (byte-identical floor); R→1 →
        // a narrow cone centred on μ. The (1−½R) keeps ≥half the cone so exploration never fully
        // collapses (the epistemic term stays alive — play/klino never abstains). ---
        std::uniform_real_distribution<float> ut(-tumble_range_, tumble_range_);
        float u = ut(rng_);
        float centre = heading_;                                  // reorient off CURRENT heading (cooperates with reflexes)
        if (dir_belief_ && last_R_ > 0.0f) {
            centre = circ_interp(heading_, last_mu_, last_R_);    // R=0 → heading_ (byte-identical)
            u *= (1.0f - 0.5f * last_R_);                         // tighten the tumble cone as confidence rises
        }
        run_dir_abs_ = wrap_pi(centre + u);
        ++tumble_count_; committed_action_ = 1;

        // start a fresh run; commit to reaching the new direction before the next tumble decision (KF1)
        run_ticks_ = 0; run_start_scent_ = smax_;
        if (run_commit_) reorienting_ = true;
    } else {
        ++run_count_; committed_action_ = 0;
        ++run_ticks_;
    }

    // egocentric bearing to the committed run direction (turn toward it + advance)
    float delta = wrap_pi(run_dir_abs_ - heading_);
    if (run_commit_) {
        // TURN-COMMIT LATCH (mirrors planner/playloop): while run_commit holds a steady run_dir, a
        // behind-target (|delta|→π) makes sin(delta)→0 → the HeadingController's steer direction
        // dithers → the body oscillates at the antipode and NEVER turns through (the run_commit
        // freeze). Latch a turn direction and cap the commanded turn under π so the bug commits to
        // the reorientation. Off when run_commit is off → byte-identical.
        constexpr float kPiF = 3.14159265f;
        if (!turning_ && std::fabs(delta) > 1.5708f)     { turning_ = true; turn_dir_ = (delta >= 0.0f) ? 1.0f : -1.0f; }
        else if (turning_ && std::fabs(delta) < 0.6283f) { turning_ = false; }
        if (turning_) delta = turn_dir_ * std::min(std::fabs(delta), 0.92f * kPiF);
    }
    out_vx_ = std::sin(delta);
    out_vy_ = std::cos(delta);

    auto out = std::make_shared<ProprioToken>();
    out->tick_id     = tick_id;
    out->producer_id = id_.empty() ? std::string("runtumble") : id_;
    out->sensor      = "runtumble_heading";
    out->values.resize(2);
    out->values[0] = out_vx_;
    out->values[1] = out_vy_;
    bus_->publish(output_topic_, out);

    // publish the self-reported capability EVERY tick → the arbiter gates klino's z-score by it
    // (capability×excitement: silent when blind, full excitement when smelling).
    if (!confidence_topic_.empty()) {
        auto cap = std::make_shared<ProprioToken>();
        cap->tick_id     = tick_id;
        cap->producer_id = id_.empty() ? std::string("runtumble") : id_;
        cap->sensor      = "klino_confidence";
        cap->values.resize(1);
        cap->values[0]   = capability_;
        bus_->publish(confidence_topic_, cap);
    }
}

nlohmann::json RunTumbleNav::diag_snapshot() const {
    return nlohmann::json{
        {"baseline", baseline_},          // methylation level (EMA of scent = the prediction)
        {"error", last_error_},           // normalised prediction error (scent − baseline)
        {"p_tumble", last_p_tumble_},     // per-tick tumble probability
        {"action", committed_action_},    // 0 run / 1 tumble
        {"runs", run_count_}, {"tumbles", tumble_count_}, {"forced", forced_tumbles_},
        {"vx", out_vx_}, {"vy", out_vy_},
        {"smax", smax_},
        {"cap", capability_},             // self-reported capability ∈[0,1] (→0 blind, →~1 in its eating range)
        {"speak", scent_peak_},           // slow-decaying memory of typical food-scent magnitude (pre-eat bootstrap)
        {"eat_scent", eat_scent_},        // EMA of the scent at which it actually EATS (calibrated denom)
        {"have_eat_scent", have_eat_scent_},
        // KF0 run-length-asymmetry (a healthy KINESIS: up-runs ≫ down-runs even when the needle looks random)
        {"run_len_up", run_len_up_},      // EMA run length for runs that RAISED scent
        {"run_len_down", run_len_down_},  // EMA run length for runs that LOWERED scent
        {"turn_frac", turn_frac_},        // EMA fraction of ticks reorienting (K1 turn-transient burn)
        {"forced_in_turn", forced_in_turn_},  // forced tumbles fired mid-turn (K2 livelock signature; → 0 under run_commit)
        // KF1/KF6 state
        {"reorienting", reorienting_},    // committing to reach run_dir before the next tumble decision
        {"run_commit", run_commit_},
        {"dir_belief", dir_belief_},
        {"dir_R", last_R_},               // directional consistency ∈[0,1] = belief precision (κ proxy); 0 = kinesis floor
        {"dir_mu", last_mu_},             // believed up-gradient absolute heading (rad)
        {"authority", authority_},        // KF3 MotorBus authority share for the klino channel ∈[0,1]
        {"muted", muted()},               // KF3 another loop / a reflex has the bus → klino coasts
    };
}

}  // namespace ogma
