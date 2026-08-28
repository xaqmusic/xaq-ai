#include "OgmaBrain.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

#include <cstring>
#include <cstdlib>
#include <nlohmann/json.hpp>

#include "ogma/InProcessBus.hpp"
#include "ogma/OgmaInstance.hpp"
#include "ogma/GraphConfig.hpp"
#include "ogma/Module.hpp"
#include "ogma/Rng.hpp"
#include "ogma/PayloadTypeName.hpp"
#include "ogma/Topics.hpp"
#include "ogma/DiagPublisher.hpp"
#include "control_server.hpp"

// Module headers — only needed in the .cpp so we can dynamic_cast for metrics.
#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/modules/Premotor.hpp"
#include "ogma/modules/PremotorAI.hpp"
#include "ogma/modules/FaderController.hpp"
#include "ogma/modules/NeurochemState.hpp"
#include "ogma/modules/DescendingPredictor.hpp"
#include "ogma/modules/GNGRollout.hpp"
#include "ogma/modules/HomeokineticExploration.hpp"
#include "ogma/modules/GainEvolver.hpp"
#include "ogma/modules/MotorEPM.hpp"
#include "ogma/modules/KeyframeAverager.hpp"
#include "ogma/modules/DualEMADetector.hpp"
#include "ogma/modules/EventConjunction.hpp"
#include "ogma/modules/ChunkAbortGate.hpp"
#include "ogma/modules/ChunkOutcomeGate.hpp"
#include "ogma/modules/EpisodicCapture.hpp"
#include "ogma/modules/MotorFader.hpp"
#include "ogma/modules/MotorBus.hpp"
#include "ogma/modules/DistressDrive.hpp"
#include "ogma/modules/MotorRepertoire.hpp"
#include "ogma/modules/SequenceGNG.hpp"
#include "ogma/modules/CPGOscillator.hpp"
#include "ogma/modules/ScentCompass.hpp"
#include "ogma/modules/HeadingController.hpp"
#include "ogma/modules/GoalBelief.hpp"
#include "ogma/modules/HeadingPlanner.hpp"
#include "ogma/modules/HeadingProbe.hpp"
#include "ogma/modules/MotivationGate.hpp"
#include "ogma/modules/BearingEstimator.hpp"
#include "ogma/modules/VisualBearing.hpp"
#include "ogma/modules/BearingFusion.hpp"
#include "ogma/modules/ScentHomingLearner.hpp"
#include "ogma/modules/SaccadeReflex.hpp"
#include "ogma/modules/CylinderBuilder.hpp"
#include "ogma/modules/ColumnBuilder.hpp"
#include "ogma/modules/PlaceGraphPlanner.hpp"
#include "ogma/modules/PlayLoop.hpp"
#include "ogma/modules/VisualHomingNav.hpp"
#include "ogma/modules/RunTumbleNav.hpp"
#include "ogma/modules/RunTumbleNavV2.hpp"
#include "ogma/modules/PlaceNav.hpp"
#include "ogma/modules/EFEArbiter.hpp"
#include "ogma/modules/GradientEPM.hpp"
#include "ogma/modules/Klinotaxis.hpp"

namespace godot {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static Dictionary make_dict() { return Dictionary(); }

// Safely reads a float from a payload and adds it to a Dictionary.
template <typename T>
static void set_if(Dictionary& d, String const& key, T const& v) {
    d[key] = v;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

OgmaBrain::OgmaBrain()  = default;
OgmaBrain::~OgmaBrain() {
    if (control_server_) control_server_->stop();
    if (diag_publisher_) diag_publisher_->stop();
}

void OgmaBrain::_bind_methods() {
    ClassDB::bind_method(D_METHOD("setup",            "config_path"),           &OgmaBrain::setup);
    ClassDB::bind_method(D_METHOD("set_master_seed",  "seed"),                  &OgmaBrain::set_master_seed);
    ClassDB::bind_method(D_METHOD("tick",             "delta"),                 &OgmaBrain::tick);
    ClassDB::bind_method(D_METHOD("publish_proprio",  "values", "sensor"),      &OgmaBrain::publish_proprio);
    ClassDB::bind_method(D_METHOD("publish_event",    "name",   "intensity"),   &OgmaBrain::publish_event);
    ClassDB::bind_method(D_METHOD("publish_video",    "pixels", "height", "width", "channels", "modality"),
                                                                                 &OgmaBrain::publish_video);
    ClassDB::bind_method(D_METHOD("get_action"),                                &OgmaBrain::get_action);
    ClassDB::bind_method(D_METHOD("get_urgency"),                               &OgmaBrain::get_urgency);
    ClassDB::bind_method(D_METHOD("get_dopamine"),                              &OgmaBrain::get_dopamine);
    ClassDB::bind_method(D_METHOD("get_serotonin"),                             &OgmaBrain::get_serotonin);
    ClassDB::bind_method(D_METHOD("get_node_count"),                            &OgmaBrain::get_node_count);
    ClassDB::bind_method(D_METHOD("get_active_chunk_id"),                       &OgmaBrain::get_active_chunk_id);
    ClassDB::bind_method(D_METHOD("is_brain_ready"),                            &OgmaBrain::is_brain_ready);
    ClassDB::bind_method(D_METHOD("get_action_left"),                           &OgmaBrain::get_action_left);
    ClassDB::bind_method(D_METHOD("get_action_right"),                          &OgmaBrain::get_action_right);
    ClassDB::bind_method(D_METHOD("is_action_bilateral"),                       &OgmaBrain::is_action_bilateral);
    ClassDB::bind_method(D_METHOD("register_action_channel", "name", "topic"),  &OgmaBrain::register_action_channel);
    ClassDB::bind_method(D_METHOD("get_action_channel",      "index"),          &OgmaBrain::get_action_channel);
    ClassDB::bind_method(D_METHOD("action_channel_count"),                      &OgmaBrain::action_channel_count);
    ClassDB::bind_method(D_METHOD("get_cpg_pure_bias"),                          &OgmaBrain::get_cpg_pure_bias);
    // Metrics API
    ClassDB::bind_method(D_METHOD("get_module_metrics"),                        &OgmaBrain::get_module_metrics);
    ClassDB::bind_method(D_METHOD("get_module_list"),                           &OgmaBrain::get_module_list);
    ClassDB::bind_method(D_METHOD("get_graph_edges"),                           &OgmaBrain::get_graph_edges);
    ClassDB::bind_method(D_METHOD("set_auto_subscribe", "enabled"),             &OgmaBrain::set_auto_subscribe);
    ClassDB::bind_method(D_METHOD("is_auto_subscribe"),                         &OgmaBrain::is_auto_subscribe);
    // Sensor registry
    ClassDB::bind_method(D_METHOD("register_source", "name","topic","description","active"), &OgmaBrain::register_source);
    ClassDB::bind_method(D_METHOD("register_sink",   "name","topic","description"),          &OgmaBrain::register_sink);
    ClassDB::bind_method(D_METHOD("register_event",  "name","topic","event_type"),           &OgmaBrain::register_event);
    ClassDB::bind_method(D_METHOD("get_sensor_registry"),                       &OgmaBrain::get_sensor_registry);

    ClassDB::bind_method(D_METHOD("snapshot_state"),                            &OgmaBrain::snapshot_state);
    ClassDB::bind_method(D_METHOD("restore_state",   "snapshot"),               &OgmaBrain::restore_state);
    ClassDB::bind_method(D_METHOD("get_module_snapshot",  "module_id"),         &OgmaBrain::get_module_snapshot);
    ClassDB::bind_method(D_METHOD("set_module_snapshot",  "module_id", "snapshot"), &OgmaBrain::set_module_snapshot);

    // Hot-patch API
    ClassDB::bind_method(D_METHOD("get_motor_fader_state"),                     &OgmaBrain::get_motor_fader_state);
    ClassDB::bind_method(D_METHOD("get_motor_bus_state"),                       &OgmaBrain::get_motor_bus_state);
    ClassDB::bind_method(D_METHOD("get_module_output_topics", "id"),            &OgmaBrain::get_module_output_topics);
    ClassDB::bind_method(D_METHOD("get_module_input_specs",   "id"),            &OgmaBrain::get_module_input_specs);
    ClassDB::bind_method(D_METHOD("get_module_output_specs",  "id"),            &OgmaBrain::get_module_output_specs);
    ClassDB::bind_method(D_METHOD("apply_patch",        "patch"),               &OgmaBrain::apply_patch);
    ClassDB::bind_method(D_METHOD("list_module_types"),                         &OgmaBrain::list_module_types);
    ClassDB::bind_method(D_METHOD("get_module_param_schema", "id"),             &OgmaBrain::get_module_param_schema);
    ClassDB::bind_method(D_METHOD("get_module_specs"),                          &OgmaBrain::get_module_specs);
}

// ---------------------------------------------------------------------------
// Core lifecycle
// ---------------------------------------------------------------------------

void OgmaBrain::set_master_seed(int64_t seed) {
    if (initialized_) {
        UtilityFunctions::push_warning("OgmaBrain::set_master_seed called after setup() — ignored.");
        return;
    }
    master_seed_override_ = uint64_t(seed);
}

bool OgmaBrain::setup(String const& config_path) {
    if (initialized_) {
        UtilityFunctions::push_warning("OgmaBrain::setup called twice — ignoring.");
        return true;
    }
    String fs_path = ProjectSettings::get_singleton()->globalize_path(config_path);
    try {
        auto cfg = ogma::GraphConfig::load_from_file(
            std::string(fs_path.utf8().get_data()));
        // v6.0 — propagate runtime master seed into every module's
        // master_seed param.  Each module's effective seed becomes
        // namespace_seed(master_seed_override_, module_id) so identical
        // configs produce different stochastic streams across runs
        // (OGMA_SEED actually does something at the brain level).
        // Override 0 (the default) leaves config values untouched —
        // preserves legacy "config is the only seed source" behaviour
        // for golden-replay paths.
        if (master_seed_override_ != 0) {
            int rewritten = 0;
            // Modules name their RNG seed param differently — "master_seed"
            // (HeadingPlanner, GaitSelector, …) OR "seed" (MotorEPM base_seed_,
            // KeyframeGait).  The old loop only rewrote "master_seed", so the
            // picrawler's MotorEPM (which owns the gait's explore_noise RNG via
            // "seed") NEVER got reseeded → every OGMA_SEED gave byte-identical runs
            // and seed-averaging was impossible.  Rewrite BOTH.
            static const char* kSeedParams[] = {"master_seed", "seed"};
            for (auto& m : cfg.modules) {
                for (const char* pname : kSeedParams) {
                    auto it = m.params.find(pname);
                    if (it == m.params.end()) continue;
                    uint64_t derived = ogma::namespace_seed(master_seed_override_, m.id);
                    it->second = ogma::ParamValue{int64_t(derived)};
                    ++rewritten;
                }
            }
            UtilityFunctions::print(
                "OgmaBrain: applied master_seed override = ", int64_t(master_seed_override_),
                " (", rewritten, " seed params rewritten)");
        }
        instance_ = std::make_unique<ogma::OgmaInstance>(
            std::move(cfg),
            std::make_unique<ogma::InProcessBus>());
        initialized_ = true;
        UtilityFunctions::print("OgmaBrain: instance ready (", config_path, ")");

        // UI-dev W2 — start the inspector surfaces.  Best-effort: a port bind
        // failure logs a warning and the brain runs without an inspector.
        // Ports: control = OGMA_INSPECTOR_PORT (default 7400), diag = control+1.
        // Headless test runs export a non-default base (e.g. 7500) so they NEVER collide
        // with a live UI's 7400/7401 — otherwise the UI's bind fails (best-effort) and the
        // inspector gets "connection refused" while a headless run holds the ports.
        uint16_t control_port = 7400;
        if (const char* env = std::getenv("OGMA_INSPECTOR_PORT")) {
            int p = std::atoi(env);
            if (p > 1024 && p < 65534) control_port = uint16_t(p);
        }
        uint16_t diag_port = uint16_t(control_port + 1);
        diag_publisher_ = std::make_unique<ogma::DiagPublisher>(diag_port);
        diag_publisher_->start();
        control_server_ = std::make_unique<ami_ogma::control::ControlServer>(control_port);
        UtilityFunctions::print("OgmaBrain: inspector control=", control_port, " diag=", diag_port);
        control_server_->set_command_handler(
            [this](nlohmann::json const& req) -> nlohmann::json {
                // Serialise against tick-thread mutation of the modules
                // vector (apply_remove etc).  Without this lock, a verb
                // like list_modules / module_snapshot can iterate or
                // dereference modules_ at the exact moment the scheduler
                // is erasing one — segfault.  Held for the entire verb
                // implementation.
                std::lock_guard<std::recursive_mutex> lk(instance_mtx_);
                std::string verb = req.value("verb", std::string());
                try {
                    if (!instance_)
                        return {{"status","error"},{"message","brain not initialised"}};
                    if (verb == "list_modules") {
                        nlohmann::json mods = nlohmann::json::array();
                        for (auto* m : instance_->modules()) {
                            mods.push_back({
                                {"id",   std::string(m->id())},
                                {"type", std::string(m->type_name())},
                            });
                        }
                        return {{"status","ok"}, {"modules", mods}};
                    }
                    if (verb == "module_snapshot") {
                        std::string id = req.value("id", std::string());
                        auto* m = instance_->module(id);
                        if (m == nullptr)
                            return {{"status","error"},{"message","unknown module: "+id}};
                        return {{"status","ok"},
                                {"module_id", id},
                                {"snapshot",  m->snapshot_state()}};
                    }
                    if (verb == "module_subscribe_diag") {
                        std::string id    = req.value("id", std::string());
                        std::string topic = req.value("topic", std::string());
                        double      hz    = req.value("hz", 30.0);
                        if (instance_->module(id) == nullptr)
                            return {{"status","error"},{"message","unknown module: "+id}};
                        int sub_id = diag_publisher_->subscribe(id, topic, hz);
                        return {{"status","ok"},
                                {"sub_id",      sub_id},
                                {"diag_port",   diag_publisher_->port()},
                                {"topic_prefix","diag." + std::to_string(sub_id) + "."}};
                    }
                    if (verb == "unsubscribe") {
                        int sub_id = req.value("sub_id", 0);
                        diag_publisher_->unsubscribe(sub_id);
                        return {{"status","ok"}};
                    }
                    if (verb == "set_param") {
                        // Live hot-mutation of a module param (experiments: coupling wean, gain
                        // sweeps, ...).  Scalar values only; reuses the SetParamOp hot-patch path.
                        std::string id  = req.value("id", std::string());
                        std::string key = req.value("key", std::string());
                        if (instance_->module(id) == nullptr)
                            return {{"status","error"},{"message","unknown module: "+id}};
                        if (!req.contains("value"))
                            return {{"status","error"},{"message","set_param requires 'value'"}};
                        ogma::SetParamOp s;
                        s.target_id = id;
                        s.key       = key;
                        auto const& jv = req["value"];
                        if      (jv.is_boolean())        s.value = jv.get<bool>();
                        else if (jv.is_number_integer()) s.value = int64_t(jv.get<int64_t>());
                        else if (jv.is_number())         s.value = jv.get<double>();
                        else if (jv.is_string())         s.value = jv.get<std::string>();
                        else return {{"status","error"},{"message","set_param: unsupported value type"}};
                        ogma::GraphPatchBatch batch;
                        batch.source = "tcp";
                        batch.ops.emplace_back(std::move(s));
                        auto batch_id = instance_->enqueue_hot_patch(std::move(batch));
                        return {{"status","ok"},{"batch_id", int64_t(batch_id)},{"id",id},{"key",key}};
                    }
                    return {{"status","error"},{"message","unknown verb: "+verb}};
                } catch (std::exception const& e) {
                    return {{"status","error"},{"message", e.what()}};
                }
            });
        control_server_->start();
        return true;
    } catch (std::exception const& e) {
        UtilityFunctions::push_error("OgmaBrain::setup failed: ", e.what());
        return false;
    }
}

void OgmaBrain::tick(double /*delta*/) {
    if (!initialized_) return;
    // Hold instance_mtx_ for the duration of the tick + diag publish so
    // ControlServer command handlers (running on a separate thread)
    // cannot iterate modules_ while apply_remove / apply_add is mid-
    // mutation.  See OgmaBrain.hpp for the full race story.
    std::lock_guard<std::recursive_mutex> lk(instance_mtx_);
    // Phase 6.6.A guard: hot-patches that throw during apply (e.g. an added
    // module whose on_setup rejects missing required params) propagate out of
    // Scheduler::tick.  Without this catch the exception escapes into Godot's
    // process loop and the host segfaults.  Surface the error and skip the
    // tick — the bad batch is dropped, but live modules already removed
    // earlier in the batch are gone (apply_batch is not transactional once
    // pass-2 starts).  Caller is responsible for noticing the mismatch via
    // get_module_list and re-issuing a corrected patch.
    try {
        instance_->tick();
    } catch (std::exception const& e) {
        UtilityFunctions::push_error(
            "OgmaBrain::tick: hot-patch or module-tick threw: ", e.what());
        return;
    }
    // UI-dev W2 — fan out diag streams for any active inspector subscriptions.
    if (diag_publisher_ && diag_publisher_->running())
        diag_publisher_->publish_tick(tick_id_, *instance_);
    auto* bus = instance_->bus();

    // Modules that ran inside instance_->tick() above stamped their published
    // tokens with the Scheduler's current tick id, which is in sync with our
    // pre-increment tick_id_.  Do all freshness-sensitive polls BEFORE
    // bumping tick_id_; bumping first would shift the comparison frame and
    // make every fresh token look stale by one (Phase 6.6.D.7 bug — bilateral
    // never activated even when reflex modules were publishing every tick).
    if (auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus->last_value(ogma::topics::kActionOut))) {
        last_action_   = double(a->accel);
        last_chunk_id_ = a->chunk_id;
    }

    // Phase 6.6.D.6 — bilateral motor poll.  When both channels were
    // published this tick, mark the tick bilateral so the body switches
    // to differential motor handling.
    auto al = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus->last_value(ogma::topics::kActionLeft));
    auto ar = std::dynamic_pointer_cast<const ogma::ActionOut>(
        bus->last_value(ogma::topics::kActionRight));
    bool fresh_left  = al && al->tick_id == tick_id_;
    bool fresh_right = ar && ar->tick_id == tick_id_;
    if (fresh_left)  last_action_left_  = double(al->accel);
    if (fresh_right) last_action_right_ = double(ar->accel);
    last_was_bilateral_ = fresh_left && fresh_right;

    // v6.0 — N-channel action poll for multi-actuator bodies.  Independent
    // of the bilateral path; bodies that registered explicit channels
    // (quadruped fl/fr/rl/rr, drone t_fl/t_fr/t_rl/t_rr) read each topic
    // here.  Stale tokens (tick_id != current) leave the cached value
    // unchanged — same freshness semantics as bilateral.
    for (size_t i = 0; i < action_channel_topics_.size(); ++i) {
        auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
            bus->last_value(action_channel_topics_[i]));
        if (a && a->tick_id == tick_id_) {
            last_action_channels_[i] = double(a->accel);
        }
    }
    ++tick_id_;

    if (auto d = std::dynamic_pointer_cast<const ogma::DriveErrors>(
            bus->last_value(ogma::topics::kDriveErrors)))
        last_urgency_ = double(d->urgency);

    if (auto n = std::dynamic_pointer_cast<const ogma::NeuroState>(
            bus->last_value(ogma::topics::kNeuroState))) {
        last_dopamine_  = double(n->dopamine);
        last_serotonin_ = double(n->serotonin);
    }

    // Phase 6.6.F — MotorFader telemetry for the on-screen meter.  Stays
    // unset (fader_seen_=false) when neither a FaderController nor a
    // MotorFader is in the graph; that's how the HUD widget
    // distinguishes "no fader" from "α=0".
    //
    // Phase 6.6.G — α fields come from the FaderController's FaderState
    // publish; per-channel raw accels come from the first MotorFader's
    // white-box state (FaderController publishes those fields as 0
    // because it isn't channel-aware).  This preserves the unified
    // get_motor_fader_state() dictionary the meter already consumes.
    if (auto fs = std::dynamic_pointer_cast<const ogma::FaderState>(
            bus->last_value(ogma::topics::kMotorFaderAlpha))) {
        fader_seen_         = true;
        fader_alpha_        = double(fs->alpha);
        fader_alpha_target_ = double(fs->alpha_target);
        fader_surprise_     = double(fs->surprise_scalar);
        fader_brain_seen_   = fs->brain_seen;
        fader_reflex_seen_  = fs->reflex_seen;
        fader_brain_accel_  = double(fs->brain_accel);
        fader_reflex_accel_ = double(fs->reflex_accel);
        fader_output_accel_ = double(fs->output_accel);
        fader_source_       = fs->source;
    }
    // Override channel telemetry from the first MotorFader instance, by
    // convention "fader_left".  The design doc notes per-channel display
    // (showing both sides simultaneously) is a polish item; v1 follows
    // the first MotorFader and matches the legacy single-channel
    // semantics for the HUD.
    for (auto const* m : instance_->modules()) {
        if (auto const* mf = dynamic_cast<ogma::MotorFader const*>(m)) {
            fader_seen_         = true;   // MotorFader alone is enough to show the meter
            fader_brain_seen_   = mf->brain_seen();
            fader_reflex_seen_  = mf->reflex_seen();
            fader_brain_accel_  = double(mf->brain_accel());
            fader_reflex_accel_ = double(mf->reflex_accel());
            fader_output_accel_ = double(mf->output_accel());
            fader_clash_        = double(mf->last_clash());
            fader_clash_ema_    = double(mf->clash_ema());
            // If FaderController hasn't published yet but MotorFader has
            // a fallback α from its own alpha_fixed param, surface that.
            if (!mf->alpha_from_bus()) {
                fader_alpha_        = double(mf->alpha());
                fader_alpha_target_ = fader_alpha_;
                fader_source_       = "fixed";
            }
            break;
        }
    }

    for (auto const& t : bus->subscribed_topics()) {
        if (t.rfind("reality.proprio.", 0) == 0) {
            if (auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(
                    bus->last_value(t))) {
                last_node_count_ = rt->node_count;
                break;
            }
        }
    }
}

void OgmaBrain::publish_proprio(PackedFloat64Array const& values, String const& sensor) {
    if (!initialized_) return;
    auto p = std::make_shared<ogma::ProprioToken>();
    p->tick_id     = tick_id_;
    p->producer_id = "host";  // for the per-primitive input gate (manual routing)
    p->sensor      = std::string(sensor.utf8().get_data());
    p->values.resize(values.size());
    for (int i = 0; i < values.size(); ++i) p->values[i] = float(values[i]);
    instance_->bus()->publish("reality.proprio." + std::string(sensor.utf8().get_data()), p);
}

void OgmaBrain::publish_event(String const& name, double intensity) {
    if (!initialized_) return;
    auto e = std::make_shared<ogma::EnvEvent>();
    e->tick_id     = tick_id_;
    e->producer_id = "host";  // see publish_proprio for the routing-gate rationale
    e->name        = std::string(name.utf8().get_data());
    e->intensity   = float(intensity);
    instance_->bus()->publish("events." + std::string(name.utf8().get_data()), e);
}

void OgmaBrain::publish_video(PackedByteArray const& pixels,
                              int height, int width, int channels,
                              String const& modality) {
    if (!initialized_) return;
    int64_t expected = int64_t(height) * width * channels;
    if (pixels.size() != expected) {
        UtilityFunctions::push_error("OgmaBrain::publish_video: pixel count ", pixels.size(),
                                     " != H*W*C ", expected, " (", height, "x", width, "x", channels, ")");
        return;
    }
    auto f = std::make_shared<ogma::RawImageFrame>();
    f->tick_id     = tick_id_;
    f->producer_id = "host";  // see publish_proprio for the routing-gate rationale
    f->height      = height;
    f->width       = width;
    f->channels    = channels;
    f->pixels.resize(size_t(expected));
    auto const* src = pixels.ptr();
    std::memcpy(f->pixels.data(), src, size_t(expected));
    // Publish on host.video.* — distinct from reality.video.* (the EPM's
    // RealityToken output topic).  Sharing the same topic name would cause
    // EPM self-feedback (the EPM's own RealityToken publish would re-fire
    // its handle_input, fail the cast, and null out pending_image_).
    instance_->bus()->publish("host.video." + std::string(modality.utf8().get_data()), f);
}

// ---------------------------------------------------------------------------
// Metrics API
// ---------------------------------------------------------------------------

PackedStringArray OgmaBrain::get_module_output_topics(String const& id) const {
    PackedStringArray out;
    if (!initialized_ || !instance_) return out;
    auto* m = instance_->module(std::string(id.utf8().get_data()));
    if (m == nullptr) return out;
    for (auto const& spec : m->output_topics()) {
        out.push_back(String(spec.name.c_str()));
    }
    return out;
}

Array OgmaBrain::get_module_input_specs(String const& id) const {
    Array out;
    if (!initialized_ || !instance_) return out;
    auto* m = instance_->module(std::string(id.utf8().get_data()));
    if (m == nullptr) return out;
    for (auto const& spec : m->input_topics()) {
        Dictionary d;
        d["name"]         = String(spec.name.c_str());
        d["payload_type"] = String(ogma::payload_type_name(spec.payload_type).c_str());
        d["kind"]         = String(spec.kind == ogma::SubscriptionKind::Feedback
                                   ? "feedback" : "direct");
        d["required"]     = spec.required;
        out.push_back(d);
    }
    return out;
}

Array OgmaBrain::get_module_output_specs(String const& id) const {
    Array out;
    if (!initialized_ || !instance_) return out;
    auto* m = instance_->module(std::string(id.utf8().get_data()));
    if (m == nullptr) return out;
    for (auto const& spec : m->output_topics()) {
        Dictionary d;
        d["name"]         = String(spec.name.c_str());
        d["payload_type"] = String(ogma::payload_type_name(spec.payload_type).c_str());
        d["kind"]         = "direct";
        d["required"]     = true;
        out.push_back(d);
    }
    return out;
}

Dictionary OgmaBrain::get_motor_fader_state() const {
    Dictionary d;
    if (!fader_seen_) return d;   // empty → meter widget hides itself
    d["alpha"]            = fader_alpha_;
    d["alpha_target"]     = fader_alpha_target_;
    d["surprise_scalar"]  = fader_surprise_;
    d["brain_seen"]       = fader_brain_seen_;
    d["reflex_seen"]      = fader_reflex_seen_;
    d["brain_accel"]      = fader_brain_accel_;
    d["reflex_accel"]     = fader_reflex_accel_;
    d["output_accel"]     = fader_output_accel_;
    d["clash"]            = fader_clash_;
    d["clash_ema"]        = fader_clash_ema_;
    d["source"]           = String(fader_source_.c_str());
    return d;
}

Dictionary OgmaBrain::get_motor_bus_state() const {
    Dictionary d;
    if (!initialized_) return d;
    for (auto const* m : instance_->modules()) {
        auto const* mb = dynamic_cast<ogma::MotorBus const*>(m);
        if (mb == nullptr) continue;
        Array names, gains, cl, cr, active;
        int n = mb->n_influencers();
        for (int i = 0; i < n; ++i) {
            names.push_back(String(mb->influencer_name(i).c_str()));
            gains.push_back(double(mb->gain(i)));
            cl.push_back(double(mb->contrib_l(i)));
            cr.push_back(double(mb->contrib_r(i)));
            active.push_back(mb->active(i));
        }
        d["names"]     = names;
        d["gains"]     = gains;
        d["contrib_l"] = cl;
        d["contrib_r"] = cr;
        d["active"]    = active;
        d["out_l"]     = double(mb->out_l());
        d["out_r"]     = double(mb->out_r());
        d["sum_l"]     = double(mb->sum_l());
        d["sum_r"]     = double(mb->sum_r());
        d["limit"]     = double(mb->limit());
        d["gr"]        = double(mb->gain_reduction());
        d["id"]        = String(std::string(mb->id()).c_str());
        break;   // first MotorBus
    }
    return d;
}

Dictionary OgmaBrain::get_module_metrics() const {
    Dictionary result;
    if (!initialized_) return result;
    auto* bus = instance_->bus();

    for (auto const* m : instance_->modules()) {
        Dictionary d;
        std::string_view type = m->type_name();
        d["type"] = String(std::string(type).c_str());

        // --- NeurochemState ---
        if (type == "NeurochemState") {
            if (auto n = std::dynamic_pointer_cast<const ogma::NeuroState>(
                    bus->last_value(ogma::topics::kNeuroState))) {
                d["dopamine"]        = double(n->dopamine);
                d["serotonin"]       = double(n->serotonin);
                d["reward_signal"]   = double(n->reward_signal);
                d["epsilon_b_scale"] = double(n->epsilon_b_scale);
                d["novelty_scale"]   = double(n->novelty_threshold_scale);
            }
            if (auto const* nc = dynamic_cast<const ogma::NeurochemState*>(m)) {
                d["total_hits"]    = nc->total_hits();
                d["total_misses"]  = nc->total_misses();
                d["total_bricks"]  = nc->total_bricks();
            }
        }
        // --- EPM ---
        else if (type == "EPM") {
            auto tops = m->output_topics();
            if (!tops.empty()) {
                if (auto rt = std::dynamic_pointer_cast<const ogma::RealityToken>(
                        bus->last_value(tops[0].name))) {
                    d["node_count"]   = rt->node_count;
                    d["baked_count"]  = rt->baked_count;
                    d["tle"]          = double(rt->tle);
                    d["is_novel"]     = rt->is_novel;
                    d["winner_id"]    = rt->winner_id;
                    d["mitosis_count"]= rt->mitosis_count;
                    // v5.4.L Phase 2a — emit the EPM's latest published
                    // latent as a flattened double array.  Enables
                    // scripts/encoding_variance.py to measure pairwise
                    // cosine distribution of the slow consensus latent
                    // across simulation time (key signal for whether
                    // the entry-match gate has any chance of working).
                    // Emitted for ALL EPMs since the same diagnostic is
                    // useful for any modality investigation.
                    Array latent;
                    latent.resize(rt->latent.size());
                    for (int i = 0; i < rt->latent.size(); ++i)
                        latent[i] = double(rt->latent(i));
                    d["last_latent"] = latent;
                }
            }
            // v5.4.L Diagnostic B — top-8 winner histogram + total samples.
            // Surfaces GNG saturation: if the top-2 winner_ids account
            // for >90% of total winner counts, the encoder is severely
            // under-discriminating.  Pruned-node entries are kept until
            // GNG explicitly notifies us of a prune (winner_counts isn't
            // cleared on prune; the diagnostic just shows the long-run
            // utilisation pattern across this run).
            if (auto const* epm = dynamic_cast<const ogma::EPM*>(m)) {
                auto const& wc = epm->winner_counts();
                int total = 0;
                for (auto const& [_, n] : wc) total += n;
                d["winner_total_samples"] = total;
                // Sort by count desc, take top 8.
                std::vector<std::pair<int, int>> sorted_wc(wc.begin(), wc.end());
                std::sort(sorted_wc.begin(), sorted_wc.end(),
                          [](auto const& a, auto const& b){ return a.second > b.second; });
                Array hist;
                int n_emit = std::min(int(sorted_wc.size()), 8);
                for (int i = 0; i < n_emit; ++i) {
                    Dictionary entry;
                    entry["id"]    = sorted_wc[i].first;
                    entry["count"] = sorted_wc[i].second;
                    hist.push_back(entry);
                }
                d["winner_top8"] = hist;
            }
        }
        // --- SequenceGNG (2026-08-11, twin-gate S0) ---
        // Cheap per-tick scalars for the body-log sg_* mirror: the last
        // published SequenceMotif + the module's own counters.  Deliberately
        // NOT snapshot_state() — that ships the full GNG and is too heavy
        // for a per-tick read (the diag_snapshot ZMQ lesson).
        else if (type == "SequenceGNG") {
            auto tops = m->output_topics();
            if (!tops.empty()) {
                if (auto sm = std::dynamic_pointer_cast<const ogma::SequenceMotif>(
                        bus->last_value(tops[0].name))) {
                    d["motif_id"]          = sm->motif_id;
                    d["match_confidence"]  = double(sm->match_confidence);
                    d["predicted_next_id"] = sm->predicted_next_id;
                    d["is_baked"]          = sm->is_baked;
                }
            }
            if (auto const* sg = dynamic_cast<const ogma::SequenceGNG*>(m)) {
                d["node_count"]  = sg->node_count();
                d["baked_count"] = sg->baked_count();
                d["n_events"]    = int64_t(sg->n_events());
            }
        }
        // --- GainEvolver (PART IV, 2026-08-17) ---
        // Cheap per-tick scalars for the body-log ge_* mirror: generation /
        // accepts / reverts / σ / phase / window scores + criterion term
        // breakdown + the active vector.  Deliberately metrics(), NOT
        // snapshot_state() (the diag_snapshot ZMQ lesson above).
        else if (type == "GainEvolver") {
            if (auto const* ge = dynamic_cast<const ogma::GainEvolver*>(m)) {
                nlohmann::json mj = ge->metrics();
                for (auto it = mj.begin(); it != mj.end(); ++it) {
                    if (it.key() == "vec") continue;
                    auto const& v = it.value();
                    String k(it.key().c_str());
                    if (v.is_boolean())             d[k] = v.get<bool>();
                    else if (v.is_number_integer()) d[k] = v.get<int64_t>();
                    else if (v.is_number())         d[k] = v.get<double>();
                    else if (v.is_string())         d[k] = String(v.get<std::string>().c_str());
                }
                Array vec;
                for (double g : mj["vec"].get<std::vector<double>>()) vec.push_back(g);
                d["vec"] = vec;
            }
        }
        // --- LateralVoter ---
        else if (type == "LateralVoter") {
            auto tops = m->output_topics();
            if (!tops.empty()) {
                if (auto ct = std::dynamic_pointer_cast<const ogma::ConsensusToken>(
                        bus->last_value(tops[0].name))) {
                    d["fused_tle"]        = double(ct->fused_tle);
                    d["active_modality"]  = String(ct->active_modality.c_str());
                    d["active_winner_id"] = ct->active_winner_id;
                    d["level"]            = ct->level;
                    Dictionary trust;
                    for (auto const& [k, v] : ct->trust_weights)
                        trust[String(k.c_str())] = double(v);
                    d["trust_weights"]    = trust;
                    // Audit: latent dim + norm + sparsity-ish (count of |v|<0.01).
                    int dim = int(ct->fused_embedding.size());
                    d["latent_dim"]  = dim;
                    if (dim > 0) {
                        double n2 = 0.0;
                        int near_zero = 0;
                        for (int i = 0; i < dim; ++i) {
                            float v = ct->fused_embedding(i);
                            n2 += double(v) * double(v);
                            if (std::fabs(v) < 0.01f) ++near_zero;
                        }
                        d["latent_norm"]      = std::sqrt(n2);
                        d["latent_near_zero"] = near_zero;
                    }
                }
            }
            // B1 observability: Hebbian association_matrix counters.
            // Both 0 when association_enabled=false (mechanism inert).  A/B
            // runs that flip association_enabled=true read these to confirm
            // the matrix is actually populating.
            if (auto const* lv = dynamic_cast<const ogma::LateralVoter*>(m)) {
                d["assoc_matrix_nnz"] = int64_t(lv->assoc_matrix_nnz());
                d["assoc_matrix_sum"] = double(lv->assoc_matrix_sum());
            }
        }
        // --- HomeostaticDrive ---
        else if (type == "HomeostaticDrive") {
            if (auto dr = std::dynamic_pointer_cast<const ogma::DriveErrors>(
                    bus->last_value(ogma::topics::kDriveErrors))) {
                d["urgency"] = double(dr->urgency);
                Dictionary errors;
                for (auto const& [k, v] : dr->errors)
                    errors[String(k.c_str())] = double(v);
                d["errors"] = errors;
            }
        }
        // --- ActionDecoder ---
        else if (type == "ActionDecoder") {
            if (auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(
                    bus->last_value(ogma::topics::kActionOut))) {
                d["accel"]    = double(a->accel);
                d["probe"]    = a->probe;
                d["chunk_id"] = a->chunk_id;
            }
            if (auto const* adc = dynamic_cast<const ogma::ActionDecoder*>(m)) {
                d["valence_size"]         = int(adc->valence_size());
                // 2026-06-21 corridor-flat diagnosis: is the pragmatic landscape flat?
                d["pref_obs"]             = double(adc->latest_pref_obs_diag());
                d["obs_states_known"]     = int(adc->obs_states_known());
                d["score_spread"]         = double(adc->last_score_spread_diag());
                d["nodeval_spread"]       = double(adc->nodeval_spread_diag());
                d["greedy_accel"]         = double(adc->greedy_accel_diag());
                d["authority"]            = double(adc->latest_authority_diag());
                d["plan_entropy"]         = double(adc->plan_entropy_diag());
                d["plan_confidence"]      = double(adc->plan_confidence_diag());
                d["commit_idx"]           = adc->commit_action_idx_diag();
                d["commit_thrust"]        = double(adc->commit_thrust_diag());
                d["state_node"]           = adc->state_node_diag();
                d["fwd_model_size"]       = int(adc->forward_model_size());
                d["hebbian_size"]         = int(adc->hebbian_size());
                d["chunk_playing"]        = adc->chunk_playing();
                d["active_chunk_id"]      = adc->active_chunk_id();
                d["chunk_remaining"]      = adc->chunk_remaining();
                d["chunk_dispatch_count"] = adc->chunk_dispatch_count();
                d["entry_history_seen"]   = adc->entry_history_seen();
                d["entry_history_size"]   = adc->entry_history_size();
                d["entry_match_dispatches"] = adc->entry_match_dispatches();
                d["manual_dispatches"]      = adc->manual_dispatches();
                d["manual_dispatch_misses"] = adc->manual_dispatch_misses();
                d["dispatches_gated_score"] = adc->dispatches_gated_score();
                d["dispatches_gated_match"] = adc->dispatches_gated_match();
                // v5.4.J — chunk armed-state diag.
                d["chunks_armed_count"]        = adc->chunks_armed_count();
                d["dispatches_blocked_unarmed"]= adc->dispatches_blocked_unarmed();
                d["chunk_rearm_threshold"]     = double(adc->chunk_rearm_threshold());
                // v5.4.L — chunk dispatch age gate diag.
                d["chunk_dispatch_min_age_ticks"]     = adc->chunk_dispatch_min_age_ticks();
                d["dispatches_blocked_too_young"]     = adc->dispatches_blocked_too_young();
            }
        }
        // --- HeadingController (learned heading-following locomotor) ---
        else if (type == "HeadingController") {
            if (auto const* hc = dynamic_cast<const ogma::HeadingController*>(m)) {
                d["bearing"]      = double(hc->last_bearing());   // egocentric, 0=facing
                d["learned_gain"] = double(hc->learned_gain());   // effective turn gain this tick
                d["k_body"]       = double(hc->body_turn_gain()); // learned |ω|/|steer| (converges)
                d["hc_steer"]     = double(hc->last_steer());
                d["hc_thrust"]    = double(hc->last_thrust());
                d["nav_on"]       = hc->last_nav_on();
                if (hc->learn_advance()) {
                    d["learn_advance"] = true;
                    d["err_bin"]       = hc->last_err_bin();      // heading-error bin acted in
                    d["thrust_act"]    = hc->last_thrust_act();   // chosen thrust level
                    d["adv_reward"]    = double(hc->last_adv_reward());  // fwd progress along heading
                    d["adv_spread"]    = double(hc->adv_value_spread()); // rising = learning
                    d["adv_cov"]       = double(hc->adv_coverage());
                }
            }
        }
        // --- HeadingPlanner (learned heading selection: tabular V[state][heading]) ---
        else if (type == "HeadingPlanner") {
            if (auto const* hp = dynamic_cast<const ogma::HeadingPlanner*>(m)) {
                d["bsec"]      = hp->last_belief_sector();
                d["asec"]      = hp->last_action_sector();
                d["vspread"]   = double(hp->last_value_spread());   // rising = learning
                d["vmax"]      = double(hp->last_value_max());
                d["win_dprog"] = double(hp->last_win_progress());
                d["eps_pick"]  = hp->last_explore_pick();
                d["coverage"]  = double(hp->coverage());
                d["abearing"]  = double(hp->action_bearing());      // chosen heading θ/π (corr vs fbear)
            }
        }
        // --- HeadingProbe (isolation harness: random commanded heading, no food) ---
        else if (type == "HeadingProbe") {
            if (auto const* pr = dynamic_cast<const ogma::HeadingProbe*>(m)) {
                d["target_deg"] = double(pr->target_world() * 180.0 / 3.14159265358979);
                d["pbearing"]   = double(pr->ego_bearing());   // egocentric bearing to commanded heading
                d["held"]       = pr->ticks_held();
            }
        }
        // --- BearingEstimator (inferred bearing: VQ of the ring + distilled readout) ---
        else if (type == "BearingEstimator") {
            if (auto const* be = dynamic_cast<const ogma::BearingEstimator*>(m)) {
                d["n_proto"]  = be->n_prototypes();
                d["winner"]   = be->last_winner();
                d["tle"]      = double(be->last_tle());      // ring reconstruction error
                d["cx"]       = double(be->inferred_cx());
                d["cy"]       = double(be->inferred_cy());
                d["lesioned"] = be->lesioned();
            }
        }
        // --- MotivationGate (homeostatic gate on foraging: pursue when hungry) ---
        else if (type == "MotivationGate") {
            if (auto const* mg = dynamic_cast<const ogma::MotivationGate*>(m)) {
                d["gain"]   = double(mg->last_gain());     // pursuit gain ∝ hunger (0=sated→idle)
                d["energy"] = double(mg->last_energy());
            }
        }
        // --- GoalBelief (persistent goal-direction belief / path integration) ---
        else if (type == "GoalBelief") {
            if (auto const* gb = dynamic_cast<const ogma::GoalBelief*>(m)) {
                d["belief_x"]   = double(gb->belief_x());
                d["belief_y"]   = double(gb->belief_y());
                d["confidence"] = double(gb->confidence());
                d["perceiving"] = gb->perceiving();
            }
        }
        // --- ScentCompass (perception: bearing direction + gradient strength) ---
        else if (type == "ScentCompass") {
            if (auto const* sc = dynamic_cast<const ogma::ScentCompass*>(m)) {
                d["cx"]  = double(sc->last_cx());   // +right (which-way), post-normalize if on
                d["cy"]  = double(sc->last_cy());   // +forward (facing)
                d["mag"] = double(sc->last_mag());  // raw gradient strength (confidence)
                d["lesioned"] = sc->lesioned();
            }
        }
        // --- VisualBearing (vision perception: food-pixel centroid bearing) ---
        else if (type == "VisualBearing") {
            if (auto const* vb = dynamic_cast<const ogma::VisualBearing*>(m)) {
                d["cx"]         = double(vb->last_vx());   // +right
                d["cy"]         = double(vb->last_vy());   // +forward
                d["mag"]        = double(vb->last_mag());  // 0 = occluded, ~1 = food in view
                d["green_frac"] = double(vb->last_green_frac());
                d["lesioned"]   = vb->lesioned();
                d["have_proto"] = vb->have_proto();   // learned a food appearance yet?
            }
        }
        // --- BearingFusion (trust-weighted vision+scent blend → fused heading) ---
        else if (type == "BearingFusion") {
            if (auto const* bf = dynamic_cast<const ogma::BearingFusion*>(m)) {
                d["fx"]       = double(bf->last_fx());
                d["fy"]       = double(bf->last_fy());
                d["w_scent"]  = double(bf->last_w_scent());
                d["w_vision"] = double(bf->last_w_vision());
            }
        }
        // --- ScentHomingLearner (Pathway A: learned scent-homing) ---
        else if (type == "ScentHomingLearner") {
            if (auto const* sh = dynamic_cast<const ogma::ScentHomingLearner*>(m)) {
                d["n_proto"]  = sh->n_prototypes();
                d["proto"]    = sh->last_proto();
                d["action"]   = sh->last_action();
                d["abearing"] = double(sh->action_bearing());   // chosen θ/π (corr vs food dir)
                d["vspread"]  = double(sh->last_value_spread()); // rising = learning
                d["vmax"]     = double(sh->last_value_max());
                d["dprog"]    = double(sh->last_win_progress());
                d["eps"]      = sh->last_explore_pick();
            }
        }
        // --- SaccadeReflex (Pathway C1: learning-walk pivot) ---
        else if (type == "SaccadeReflex") {
            if (auto const* sr = dynamic_cast<const ogma::SaccadeReflex*>(m)) {
                d["state"]    = sr->state();          // 0 idle / 1 pivot / 2 refractory
                d["pivoting"] = sr->is_pivoting();
                d["dist"]     = double(sr->dist_accum());
                d["count"]    = sr->saccade_count();
                d["novelty"]  = double(sr->last_novelty());   // vision surprise (epistemic trigger)
                d["progress"] = double(sr->last_progress());  // foraging progress (>0 approaching)
                d["hunger"]   = double(sr->last_hunger());    // 1−energy (explore-when-hungry)
            }
        }
        // --- CylinderBuilder (Pathway C2: heading-indexed panorama place-code) ---
        else if (type == "CylinderBuilder") {
            if (auto const* cyl = dynamic_cast<const ogma::CylinderBuilder*>(m)) {
                d["built"]       = cyl->cylinders_built();
                d["bins_filled"] = cyl->bins_filled();
                Array pano;
                auto const& p = cyl->panorama();
                pano.resize(int(p.size()));
                for (int i = 0; i < int(p.size()); ++i) pano[i] = double(p[i]);
                d["panorama"] = pano;   // [n_bins*3] RGB, 0..1 — the place code
            }
        }
        // --- ColumnBuilder (passive place-recorder: view-feature + heading + IMU) ---
        else if (type == "ColumnBuilder") {
            if (auto const* col = dynamic_cast<const ogma::ColumnBuilder*>(m)) {
                d["dims"]     = col->dims();                 // 3*n_strips + 4
                d["recorded"] = col->recorded_last_tick();   // emitted this tick?
                d["n_strips"] = col->n_strips();
                auto const& c = col->last_column();
                d["s0r"] = c.size() > 0 ? double(c[0]) : 0.0;   // first strip RGB (HUD sanity)
                d["s0g"] = c.size() > 1 ? double(c[1]) : 0.0;
                d["s0b"] = c.size() > 2 ? double(c[2]) : 0.0;
                Array column;
                column.resize(int(c.size()));
                for (int i = 0; i < int(c.size()); ++i) column[i] = double(c[i]);
                d["column"] = column;   // [3*n_strips+4] full column
            }
        }
        // --- PlaceGraphPlanner (Pathway D: map → heading) ---
        else if (type == "PlaceGraphPlanner") {
            if (auto const* pp = dynamic_cast<const ogma::PlaceGraphPlanner*>(m)) {
                d["planning"]  = pp->planning();
                d["wandering"] = pp->wandering();
                d["homing_vision"] = pp->homing_vision();
                d["cur_node"]  = pp->cur_node();
                d["next_node"] = pp->next_node();
                d["n_nodes"]   = pp->n_nodes();
                d["fx"]        = double(pp->last_fx());
                d["fy"]        = double(pp->last_fy());
                d["hab_cur"]     = double(pp->hab_cur());
                d["n_nodes_hab"] = pp->n_nodes_hab();
                d["max_hab"]     = double(pp->max_hab());
                d["desperation"] = double(pp->desperation());   // = hunger (accelerates cache disconfirmation)
                d["steer_bias"]  = double(pp->steer_bias());     // confinement-steer strength (toward fresh ground)
                d["escaping"]    = pp->escaping();               // the steer is meaningfully pulling the route toward fresh ground
            }
        }
        // --- PlaceNav (planner reframed: NAVIGATE the map to a loose food-region tag) ---
        else if (type == "PlaceNav") {
            if (auto const* pn = dynamic_cast<const ogma::PlaceNav*>(m)) {
                d["planning"]   = pn->planning();                  // routing to a fresh reachable tag
                d["wandering"]  = pn->wandering();                 // explore run-and-tumble (no food route)
                d["cur_node"]   = pn->cur_node();
                d["next_node"]  = pn->next_node();
                d["n_nodes"]    = pn->n_nodes();
                d["value"]      = double(pn->value(pn->cur_node())); // V at the current place
                d["food_tag"]   = double(pn->food_tag(pn->cur_node()));
                d["n_tags"]     = pn->n_food_tags();                // # of nodes holding a live food tag (map-wide)
                d["eats_rx"]    = pn->eats_received();              // lifetime eat events that reached PlaceNav
                d["hab_cur"]    = double(pn->hab_cur());
                d["route_stall"] = pn->route_stall();              // ticks on a committed hop with no transition
                d["route_ceded"] = pn->route_ceded();              // the committed hop was ruled unreachable (blocked)
                d["block_cost"] = double(pn->block_cost(pn->cur_node(), pn->next_node()));
                d["plan_value"]   = double(pn->last_plan_value());   // honest reach-to-region → arbiter reach_planner
                d["plan_novelty"] = double(pn->last_plan_novelty()); // coverage need when no food route → arbiter epistemic
            }
        }
        // --- PlayLoop (Cell task #33 — the third policy: GROW the map, epistemic explore) ---
        else if (type == "PlayLoop") {
            if (auto const* pl = dynamic_cast<const ogma::PlayLoop*>(m)) {
                d["climbing"]    = pl->climbing();               // routing UP the novelty gradient toward the frontier
                d["wandering"]   = pl->wandering();              // run-and-tumble BEYOND the frontier (unmapped ground)
                d["forced_wander"] = pl->forced_wander();        // stall-wander overriding the climb (pushing past the frontier)
                d["have_frontier"] = pl->have_frontier();        // frontier-directed wander engaged (steering away from the visited centroid)
                d["frontier_bearing"] = double(pl->frontier_bearing());
                d["stale_explore"] = pl->stale_explore();        // ticks since the map last grew
                d["cur_node"]    = pl->cur_node();
                d["next_node"]   = pl->next_node();
                d["n_nodes"]     = pl->n_nodes();
                d["fx"]          = double(pl->last_fx());
                d["fy"]          = double(pl->last_fy());
                d["play_value"]  = double(pl->last_play_value()); // frontier value ∈[0,1] → arbiter (epistemic map-growth potential)
                d["novelty_cur"] = double(pl->novelty_cur());     // novelty (place-EPM TLE EMA) at the current node
                d["value_peak"]  = double(pl->value_peak());      // slow-decaying V_play peak (the play_value normaliser)
                d["hab_cur"]     = double(pl->hab_cur());
                d["max_hab"]     = double(pl->max_hab());
                d["eat_credit"]  = double(pl->eat_credit());      // EMA of "exploration led to a real eat"
            }
        }
        // --- VisualHomingNav (loop #4 — CLOSE on a SEEN source) ---
        else if (type == "VisualHomingNav") {
            if (auto const* vh = dynamic_cast<const ogma::VisualHomingNav*>(m)) {
                d["have_food"]       = vh->have_food();            // food in view this tick (occlusion gate)
                d["have_target"]     = vh->have_target();          // a remembered food target is held (persistence)
                d["persisting"]      = vh->persisting();           // homing to the remembered target (food occluded)
                d["tgt_conf"]        = double(vh->tgt_conf());     // decaying confidence in the remembered target
                d["value"]           = double(vh->value());        // vision_value ∈[0,1] → arbiter
                d["cap_vision"]      = double(vh->cap_vision());   // eat-calibrated reach confidence
                d["eat_green"]       = double(vh->eat_green());    // learned green_frac at the eat (reach scale)
                d["informativeness"] = double(vh->informativeness()); // EPM food-structure trust ∈[0,1]
                d["epm_tle"]         = double(vh->epm_tle());      // vision-food EPM TLE (§1 predictive error)
                d["node_count"]      = vh->node_count();
                d["fx"]              = double(vh->last_vx());
                d["fy"]              = double(vh->last_vy());
            }
        }
        // --- Klinotaxis (epistemic-foraging gradient follower) ---
        else if (type == "Klinotaxis") {
            if (auto const* kt = dynamic_cast<const ogma::Klinotaxis*>(m)) {
                d["base"]   = double(kt->base_heading());
                d["lockin"] = double(kt->lockin_mag());
                d["period"] = double(kt->weave_period());
                d["trend"]  = double(kt->trend());
                d["cap"]    = double(kt->cap());        // self-calibrated proximity ∈[0,1]
                d["weff"]   = double(kt->weave_eff());  // the applied (proximity-shrunk) weave amplitude
                d["align"]  = double(kt->align());      // heading-vs-travel alignment gate ∈[0,1]
            }
        }
        // --- GradientEPM (meta-EPM scalar-gradient follower) ---
        else if (type == "GradientEPM") {
            if (auto const* ge = dynamic_cast<const ogma::GradientEPM*>(m)) {
                d["nodes"]   = ge->node_count();
                d["baked"]   = ge->baked_count();
                d["pred"]    = double(ge->last_pred());
                d["dscalar"] = double(ge->last_dscalar());
                d["ghead"]   = double(ge->chosen_heading());
            }
        }
        // --- RunTumbleNav (honest temporal chemotaxis — E. coli methylation reflex) ---
        else if (type == "RunTumbleNav") {
            if (auto const* rt = dynamic_cast<const ogma::RunTumbleNav*>(m)) {
                d["baseline"] = rt->baseline();             // methylation level (EMA of scent = prediction)
                d["error"]    = double(rt->last_error());   // normalised prediction error
                d["p_tumble"] = double(rt->last_p_tumble());// per-tick tumble probability
                d["action"]   = rt->last_action();          // 0 run / 1 tumble
                d["runs"]     = rt->run_count();
                d["tumbles"]  = rt->tumble_count();
                d["forced"]   = rt->forced_tumbles();
                d["cap"]       = double(rt->capability());   // self-reported capability ∈[0,1] (→0 blind, →~1 in its eating range)
                d["speak"]     = double(rt->scent_peak());   // slow-decaying scent-magnitude memory (pre-eat bootstrap denom)
                d["eat_scent"] = double(rt->eat_scent());    // EMA of the scent at which it actually EATS (calibrated cap denom)
                // KF0/KF1/KF6 kt-loop health telemetry (run-length asymmetry, run integrity, directional belief)
                d["run_len_up"]    = double(rt->run_len_up());     // EMA run length for runs that RAISED scent
                d["run_len_down"]  = double(rt->run_len_down());   // EMA run length for runs that LOWERED scent
                d["turn_frac"]     = double(rt->turn_frac());      // EMA fraction of ticks reorienting (K1 turn burn)
                d["forced_in_turn"] = rt->forced_in_turn();        // forced tumbles fired mid-turn (K2 signature; →0 under run_commit)
                d["reorienting"]   = rt->reorienting();            // committing to reach run_dir before the next tumble
                d["dir_R"]         = double(rt->dir_consistency());// directional consistency ∈[0,1] = belief precision (0 = kinesis)
                d["dir_mu"]        = double(rt->dir_mu());         // believed up-gradient absolute heading (rad)
                d["muted"]         = rt->muted();                 // KF3: another loop / a reflex has the bus → klino coasts
            }
        }
        // --- RunTumbleNavV2 (clean-room taxis: methylation+KF4 floor, KF1 integrity, KF2 stuck, KF6 belief) ---
        else if (type == "RunTumbleNavV2") {
            if (auto const* rt = dynamic_cast<const ogma::RunTumbleNavV2*>(m)) {
                d["baseline"] = rt->baseline();
                d["error"]    = double(rt->last_error());
                d["p_tumble"] = double(rt->last_p_tumble());
                d["action"]   = rt->last_action();
                d["runs"]     = rt->run_count();
                d["tumbles"]  = rt->tumble_count();
                d["forced"]   = rt->forced_tumbles();
                d["cap"]       = double(rt->capability());
                d["eat_scent"] = double(rt->eat_scent());
                d["nfloor"]    = double(rt->noise_floor());       // KF4 stationary-noise floor
                d["vscale"]    = double(rt->vel_scale());         // KF2/KF4 learned speed scale
                d["run_len_up"]    = double(rt->run_len_up());
                d["run_len_down"]  = double(rt->run_len_down());
                d["turn_frac"]     = double(rt->turn_frac());
                d["forced_in_turn"] = rt->forced_in_turn();
                d["reorienting"]   = rt->reorienting();
                d["dir_R"]         = double(rt->dir_consistency());
                d["dir_mu"]        = double(rt->dir_mu());
                d["muted"]         = rt->muted();
            }
        }
        // --- EFEArbiter (Cell L2 — active-inference policy selection: the value race) ---
        else if (type == "EFEArbiter") {
            if (auto const* ar = dynamic_cast<const ogma::EFEArbiter*>(m)) {
                d["scoring_mode"] = String(ar->scoring_mode().c_str()); // "value_race" | "efe"
                d["raw_klino"]    = double(ar->raw_klino());     // hunger × scent
                d["raw_planner"]  = double(ar->raw_planner());   // food-route value (0 while exploring)
                d["v_klino"]      = double(ar->v_klino());       // selected klino score (value-race MAX, or G_klino in efe)
                d["v_planner"]    = double(ar->v_planner());     // selected planner score (value-race LEVEL, or G_planner in efe)
                d["cap_klino"]    = double(ar->cap_klino());     // klino's self-reported capability ∈[0,1] (telemetry only)
                d["mean_klino"]   = double(ar->mean_klino());    // klino's running raw baseline
                d["plan_peak"]    = double(ar->plan_peak());     // planner's slow-decaying peak food-route value (value-race level denominator)
                // explicit-EFE decomposition (efe mode; 0 in value_race) — the four-term race for the inspector
                d["g_prag_klino"]    = double(ar->g_prag_klino());    // hunger · reach-prob(klino)   — pragmatic, sensory precision
                d["g_prag_planner"]  = double(ar->g_prag_planner());  // hunger · reach-prob(planner) — pragmatic, model precision
                d["g_epist_klino"]   = double(ar->g_epist_klino());   // (1−hunger) · normalised z-spike — klino approach/epistemic
                d["g_epist_planner"] = double(ar->g_epist_planner()); // (1−hunger) · planner frontier novelty (Stage 3)
                d["G_klino"]         = double(ar->G_klino());         // g_prag_klino + g_epist_klino
                d["G_planner"]       = double(ar->G_planner());       // g_prag_planner + g_epist_planner
                d["plan_novelty"]    = double(ar->plan_novelty());    // planner frontier novelty ∈[0,1] (efe epistemic input)
                d["plan_precision"]  = double(ar->plan_precision());  // planner model precision ∈[0,1] (§2.3 controlled precision)
                // play policy (task #33) — the epistemic GROW loop, energy-surplus weighted
                d["play_active"]  = ar->play_active();           // play participates in the race (weight>0 && wired && efe)
                d["play_value"]   = double(ar->play_value());    // PlayLoop frontier value ∈[0,1] (epistemic map-growth potential)
                d["g_epist_play"] = double(ar->g_epist_play());  // play_weight · (1−hunger=energy surplus) · play_value
                d["v_play"]       = double(ar->v_play());        // selected play score (= G_play)
                d["G_play"]       = double(ar->G_play());
                d["gain_play"]    = double(ar->gain_play());
                // vision policy (loop #4) — pragmatic CLOSE on a SEEN source
                d["vision_active"] = ar->vision_active();        // vision participates in the race (weight>0 && wired && efe)
                d["vision_value"]  = double(ar->vision_value()); // VisualHomingNav sight-confidence ∈[0,1]
                d["g_prag_vision"] = double(ar->g_prag_vision());// vision_weight · hunger · vision_value
                d["v_vision"]      = double(ar->v_vision());     // selected vision score (= G_vision)
                d["G_vision"]      = double(ar->G_vision());
                d["gain_vision"]   = double(ar->gain_vision());
                d["winner"]       = ar->winner();                // 0 = klino, 1 = planner, 2 = play, 3 = vision
                d["gain_klino"]   = double(ar->gain_klino());    // hard 1/0 gain → MotorBus
                d["gain_planner"] = double(ar->gain_planner());
                d["margin"]       = double(ar->margin());        // adaptive hysteresis band
                d["hunger"]       = double(ar->hunger());
                d["scent"]        = double(ar->scent());
            }
        }
        // --- MotorBus (per-influencer effective gains + authority — see the arbiter mute) ---
        else if (type == "MotorBus") {
            if (auto const* mb = dynamic_cast<const ogma::MotorBus*>(m)) {
                Array names, gains, eff, arb, auth, act;
                int n = mb->n_influencers();
                for (int i = 0; i < n; ++i) {
                    names.push_back(String(mb->influencer_name(i).c_str()));
                    gains.push_back(double(mb->gain(i)));        // base fader
                    eff.push_back(double(mb->eff_gain(i)));      // base · sidechain · arbiter
                    arb.push_back(double(mb->arb_gain(i)));      // L2-arbiter gain (1=pass, 0=muted)
                    auth.push_back(double(mb->authority(i)));    // realized share of drive
                    act.push_back(mb->active(i));
                }
                d["names"]     = names;
                d["gains"]     = gains;
                d["eff_gain"]  = eff;
                d["arb_gain"]  = arb;
                d["authority"] = auth;
                d["active"]    = act;
                d["gr"]        = double(mb->gain_reduction());
            }
        }
        // --- FaderController (audit — α + drivers per tick) ---
        else if (type == "MotorFader") {
            if (auto const* mf = dynamic_cast<const ogma::MotorFader*>(m)) {
                d["alpha"]        = double(mf->alpha());
                d["brain_seen"]   = mf->brain_seen();
                d["reflex_seen"]  = mf->reflex_seen();
                d["brain_accel"]  = double(mf->brain_accel());
                d["reflex_accel"] = double(mf->reflex_accel());
                d["output_accel"] = double(mf->output_accel());
                d["clash"]        = double(mf->last_clash());
                d["clash_ema"]    = double(mf->clash_ema());
                d["publish_count"]= int(mf->publish_count());
            }
        }
        else if (type == "FaderController") {
            if (auto const* fc = dynamic_cast<const ogma::FaderController*>(m)) {
                d["alpha"]              = double(fc->alpha());
                d["alpha_target"]       = double(fc->alpha_target());
                d["surprise_scalar"]    = double(fc->surprise_scalar());
                d["familiarity_scalar"] = double(fc->familiarity_scalar());
                d["action_msgs_received"] = int(fc->action_msgs_received());
                d["policy_msgs_received"] = int(fc->policy_msgs_received());
                d["last_premotor_entropy"] = double(fc->last_premotor_entropy());
                d["last_premotor_n_intents"] = int(fc->last_premotor_n_intents());
                d["chunk_active_ticks"]   = int(fc->chunk_active_ticks());
                d["current_chunk_id"]     = int(fc->current_chunk_id_diag());
                d["boredom_term"]       = double(fc->boredom_term());
                d["learned_setpoint"]   = double(fc->learned_setpoint());
            }
        }
        // --- DistressDrive (Phase 6.9.A — boredom/distress combiner) ---
        else if (type == "DistressDrive") {
            if (auto const* dd = dynamic_cast<const ogma::DistressDrive*>(m)) {
                d["boredom"]    = double(dd->boredom());
                d["mismatch"]   = double(dd->mismatch());
                d["no_progress"]= double(dd->no_progress());
                d["interest"]      = double(dd->interest());
                d["scent_novelty"] = double(dd->scent_novelty());
                d["clearance"]     = double(dd->clearance());
                d["green"]         = double(dd->green());
                d["tle_spike"]  = double(dd->tle_spike());
                d["staleness"]  = double(dd->staleness());
                d["motion_inv"] = double(dd->motion_inv());
                d["motion_raw"] = double(dd->motion_raw());
                d["motion_ema"] = double(dd->motion_ema());
                d["suppress"]   = double(dd->suppress());
            }
        }
        // --- Premotor (Phase 6.6.O diag — BC + chosen-intent histograms) ---
        else if (type == "Premotor") {
            if (auto const* p = dynamic_cast<const ogma::Premotor*>(m)) {
                Array bc;
                for (int v : p->bc_intent_counts()) bc.append(v);
                Array chosen;
                for (int v : p->chosen_intent_counts()) chosen.append(v);
                d["n_intents"]         = p->n_intents();
                d["bc_intent_counts"]  = bc;
                d["chosen_intent_counts"] = chosen;
                d["bc_total_updates"]  = p->bc_total_updates();
                d["total_overrides_used"] = p->total_overrides_used();   // v5.3 Phase B
                d["total_explore_overrides_used"] = p->total_explore_overrides_used(); // Phase 6.7
                d["last_explore_active"]  = p->last_explore_active();    // Phase 6.7
                d["aligned_rewards_seen"] = p->aligned_rewards_seen();   // v5.3 Phase C
                d["last_chosen"]       = p->last_chosen();
                d["raw_chosen"]        = p->raw_chosen();
                d["held_intent"]       = p->held_intent();
                d["held_intent_ticks_left"] = p->held_intent_ticks_left();
                d["dwell_holds"]       = p->dwell_holds();
                d["dwell_breaks"]      = p->dwell_breaks();
                d["phase_bin"]         = p->phase_bin();
                d["phase_switch_penalties"] = p->phase_switch_penalties();
                d["phase_boundary_holds"] = p->phase_boundary_holds();
                d["phase_bin_changes"] = p->phase_bin_changes();
                d["last_bc_intent"]    = p->last_bc_intent();
                d["last_entropy"]      = double(p->last_entropy());
                // v5.4.M Diagnostic B — windowed Shannon entropy of
                // CHOSEN intents (rut signal).
                d["chosen_window_entropy"] = double(p->chosen_window_entropy());
                d["chosen_window_size"]    = p->chosen_window_size();
                // Audit: per-row Frobenius norm of W; non-uniform = brain
                // is differentiating intents.  Identical rows = brain has
                // not yet learned anything specific to action choice.
                Eigen::MatrixXf const& W = p->weights();
                Array row_norms;
                for (int i = 0; i < W.rows(); ++i)
                    row_norms.append(double(W.row(i).norm()));
                d["W_row_norms"] = row_norms;
                d["W_total_norm"] = double(W.norm());
                // 2026-05-29 gait-bucket bet — per-bucket bias row norms.
                // Non-uniform across buckets = brain has learned per-bucket
                // specialization (the per-leg-role gradient that the
                // standing-reward landscape doesn't otherwise expose).
                Eigen::MatrixXf const& BB = p->bucket_bias();
                Array bucket_row_norms;
                for (int i = 0; i < BB.rows(); ++i)
                    bucket_row_norms.append(double(BB.row(i).norm()));
                d["bucket_bias_row_norms"] = bucket_row_norms;
                d["n_buckets"]             = p->n_buckets();
                d["current_bucket"]        = p->current_bucket();
                // Phase v5.1 MC actor-critic diag.
                d["mc_episodes_seen"]    = p->mc_episodes_seen();
                d["mc_trajectory_size"]  = p->mc_trajectory_size();
                d["mc_last_return"]      = double(p->mc_last_return());
                d["mc_return_mean"]      = double(p->mc_return_mean());
                d["mc_return_std"]       = double(p->mc_return_std());
            }
        }
        // --- PremotorAI (2026-06-04 Phase A2 — same diag surface as Premotor
        // plus new W_leg per-row norms + last_leg_phase_contribution).
        // Separate dynamic_cast branch because PremotorAI does not inherit
        // from Premotor.
        else if (type == "PremotorAI") {
            if (auto const* p = dynamic_cast<const ogma::PremotorAI*>(m)) {
                Array bc;
                for (int v : p->bc_intent_counts()) bc.append(v);
                Array chosen;
                for (int v : p->chosen_intent_counts()) chosen.append(v);
                d["n_intents"]         = p->n_intents();
                d["bc_intent_counts"]  = bc;
                d["chosen_intent_counts"] = chosen;
                d["bc_total_updates"]  = p->bc_total_updates();
                d["total_overrides_used"] = p->total_overrides_used();
                d["total_explore_overrides_used"] = p->total_explore_overrides_used();
                d["last_explore_active"]  = p->last_explore_active();
                d["aligned_rewards_seen"] = p->aligned_rewards_seen();
                d["last_accel"]        = double(p->last_accel());  // H1 V3 — continuous steering output
                d["last_chosen"]       = p->last_chosen();
                d["raw_chosen"]        = p->raw_chosen();
                d["held_intent"]       = p->held_intent();
                d["held_intent_ticks_left"] = p->held_intent_ticks_left();
                d["dwell_holds"]       = p->dwell_holds();
                d["dwell_breaks"]      = p->dwell_breaks();
                d["phase_bin"]         = p->phase_bin();
                d["phase_switch_penalties"] = p->phase_switch_penalties();
                d["phase_boundary_holds"] = p->phase_boundary_holds();
                d["phase_bin_changes"] = p->phase_bin_changes();
                d["last_bc_intent"]    = p->last_bc_intent();
                d["last_entropy"]      = double(p->last_entropy());
                d["chosen_window_entropy"] = double(p->chosen_window_entropy());
                d["chosen_window_size"]    = p->chosen_window_size();
                Eigen::MatrixXf const& W = p->weights();
                Array row_norms;
                for (int i = 0; i < W.rows(); ++i)
                    row_norms.append(double(W.row(i).norm()));
                d["W_row_norms"] = row_norms;
                d["W_total_norm"] = double(W.norm());
                Eigen::MatrixXf const& BB = p->bucket_bias();
                Array bucket_row_norms;
                for (int i = 0; i < BB.rows(); ++i)
                    bucket_row_norms.append(double(BB.row(i).norm()));
                d["bucket_bias_row_norms"] = bucket_row_norms;
                d["n_buckets"]             = p->n_buckets();
                d["current_bucket"]        = p->current_bucket();
                d["mc_episodes_seen"]    = p->mc_episodes_seen();
                d["mc_trajectory_size"]  = p->mc_trajectory_size();
                d["mc_last_return"]      = double(p->mc_last_return());
                d["mc_return_mean"]      = double(p->mc_return_mean());
                d["mc_return_std"]       = double(p->mc_return_std());
            }
        }
        // --- DescendingPredictor ---
        else if (type == "DescendingPredictor") {
            if (auto const* dp = dynamic_cast<const ogma::DescendingPredictor*>(m)) {
                d["target_count"] = dp->target_count();
                Dictionary per_target;
                for (int i = 0; i < dp->target_count(); ++i) {
                    Dictionary tt;
                    tt["topic"]   = String(dp->target_topic(i).c_str());
                    tt["label"]   = String(dp->target_label(i).c_str());
                    tt["err"]     = double(dp->target_err_ema(i));
                    tt["norm"]    = double(dp->target_norm_ema(i));
                    tt["W_rows"]  = dp->target_W_rows(i);
                    tt["W_cols"]  = dp->target_W_cols(i);
                    tt["cached"]  = dp->target_cached_valid(i);
                    per_target[String(dp->target_label(i).c_str())] = tt;
                }
                d["per_target"] = per_target;
            }
            auto tops = m->output_topics();
            if (!tops.empty()) {
                if (auto pt = std::dynamic_pointer_cast<const ogma::PredictionToken>(
                        bus->last_value(tops[0].name))) {
                    d["confidence"] = double(pt->confidence);
                }
            }
        }
        // --- CPGOscillator ---
        else if (type == "CPGOscillator") {
            if (auto const* cpg = dynamic_cast<const ogma::CPGOscillator*>(m)) {
                d["competence_gate"]         = double(cpg->competence_gate());
                d["ema_reward_signal"]       = double(cpg->ema_reward_signal());
                d["ema_fused_tle"]           = double(cpg->ema_fused_tle());
                d["max_fused_tle_seen"]      = double(cpg->max_fused_tle_seen());
                d["latest_fused_tle"]        = double(cpg->latest_fused_tle());
                d["phase"]                   = double(cpg->phase());
                d["last_walking_amp"]        = double(cpg->last_walking_amp());
                d["last_standing_factor"]    = double(cpg->last_standing_factor());
                d["base_amplitude"]          = double(cpg->base_amplitude_p());
                d["amplitude_floor"]         = double(cpg->amplitude_floor_p());
                d["standing_bias_amplitude"] = double(cpg->standing_bias_amplitude_p());
                d["gate_ema_alpha"]          = double(cpg->gate_ema_alpha_p());
                d["gate_scale"]              = double(cpg->gate_scale_p());
                d["period_ticks"]            = int(cpg->period_ticks());
                d["n_joints"]                = int(cpg->n_joints());
                d["total_outputs"]           = int(cpg->total_outputs());
                // Per-joint vectors as Arrays so the inspector can plot them.
                Array w;  for (auto v : cpg->last_bias_walking())   w.append(double(v));
                Array s;  for (auto v : cpg->last_bias_standing())  s.append(double(v));
                Array b;  for (auto v : cpg->last_blended())        b.append(double(v));
                Array ss; for (auto v : cpg->standing_signs())      ss.append(double(v));
                Array lp; for (auto v : cpg->leg_phase_offsets())   lp.append(double(v));
                Array jp; for (auto v : cpg->joint_phase_offsets()) jp.append(double(v));
                Array ot; for (auto const& t : cpg->output_topics_list()) ot.append(String(t.c_str()));
                d["last_bias_walking"]   = w;
                d["last_bias_standing"]  = s;
                d["last_blended"]        = b;
                d["standing_signs"]      = ss;
                d["leg_phase_offsets"]   = lp;
                d["joint_phase_offsets"] = jp;
                d["output_topics"]       = ot;
            }
        }
        // --- SequenceGNG ---
        else if (type == "SequenceGNG") {
            if (auto const* sg = dynamic_cast<const ogma::SequenceGNG*>(m)) {
                d["node_count"]    = sg->node_count();
                d["baked_count"]   = sg->baked_count();
                d["current_motif"] = sg->current_motif();
            }
            auto tops = m->output_topics();
            if (!tops.empty()) {
                if (auto sm = std::dynamic_pointer_cast<const ogma::SequenceMotif>(
                        bus->last_value(tops[0].name))) {
                    d["match_confidence"] = double(sm->match_confidence);
                    d["phase"]            = sm->phase;
                }
            }
        }
        // --- GNGRollout ---
        else if (type == "GNGRollout") {
            if (auto const* gr = dynamic_cast<const ogma::GNGRollout*>(m)) {
                d["known_sources"] = int(gr->known_sources());
            }
        }
        // --- MotorRepertoire ---
        else if (type == "MotorRepertoire") {
            if (auto const* mr = dynamic_cast<const ogma::MotorRepertoire*>(m)) {
                d["chunk_count"]          = int(mr->chunk_count());
                d["active_chunk_count"]   = int(mr->active_chunk_count());
                d["total_dispatch_count"] = int(mr->total_dispatch_count());
                d["failed_dispatch_count"]= int(mr->failed_dispatch_count());
                // v5.3 Phase B
                d["intent_history_size"]    = int(mr->intent_history_size());
                d["intents_received_total"] = int(mr->intents_received_total());
                // v5.4 Phase A
                d["episodic_proposals_ingested"] = int(mr->episodic_proposals_ingested());
                // v5.4 Phase G
                d["position_hits_credited"] = int(mr->position_hits_credited());
                // v5.4 Phase H — chunk lifecycle diag.
                d["chunks_pruned_total"]       = int(mr->chunks_pruned_total());
                d["eligibility_credits_total"] = int(mr->eligibility_credits_total());
                d["chunk_dispatch_trace_size"] = int(mr->chunk_dispatch_trace_size());
            }
            // Surface the live chunk library by snooping motor.chunks (the
            // last published library snapshot).  Per-chunk fields support
            // verifying that captured action_sequences are reasonable
            // pivot/approach patterns rather than noise bursts.
            if (auto mc = std::dynamic_pointer_cast<const ogma::MotorChunks>(
                    bus->last_value(ogma::topics::kMotorChunks))) {
                Array chunks_arr;
                for (auto const& c : mc->chunks) {
                    Dictionary cd;
                    cd["id"]               = c.id;
                    cd["length"]           = int(std::max(c.action_sequence.size(),
                                                             c.intent_sequence.size()));
                    cd["intent_length"]    = int(c.intent_sequence.size());   // v5.3 Phase B
                    cd["use_count"]        = c.use_count;
                    cd["drive_delta"]      = double(c.outcome_drive_delta);
                    cd["trigger_motif"]    = c.trigger_consensus_motif_id;
                    cd["trigger_urgency"]  = double(c.trigger_urgency);
                    cd["hits_during"]      = c.hits_during;
                    // v5.4 Phase H — float-valued post-eligibility-trace.
                    cd["replay_hits"]      = double(c.replay_hits);
                    cd["replay_misses"]    = double(c.replay_misses);
                    Array seq;
                    for (auto v : c.action_sequence)
                        seq.push_back(double(v));
                    cd["accel_seq"]        = seq;
                    Array iseq;
                    for (auto v : c.intent_sequence)
                        iseq.push_back(int(v));
                    cd["intent_seq"]       = iseq;   // v5.3 Phase B
                    // v5.4.K — entry_embeddings flattened as nested arrays
                    // [[d0,d1,...],[d0,d1,...]] so an external probe can
                    // compute pairwise cosines without separate plumbing.
                    // Empty for non-episodic chunks (seeded / motif-baked).
                    Array entry_emb_arr;
                    for (auto const& vec : c.entry_embeddings) {
                        Array one;
                        for (int i = 0; i < vec.size(); ++i)
                            one.push_back(double(vec(i)));
                        entry_emb_arr.push_back(one);
                    }
                    cd["entry_embeddings"] = entry_emb_arr;
                    // Stddev across the whole sequence — flat chunks (motor
                    // smoothing) have stddev≈0, varying chunks (genuine
                    // pivots/maneuvers) have stddev>0.  Most informative
                    // single number for verifying chunk shape.
                    double mean = 0.0, var = 0.0;
                    if (!c.action_sequence.empty()) {
                        for (auto v : c.action_sequence) mean += v;
                        mean /= double(c.action_sequence.size());
                        for (auto v : c.action_sequence) var += (v - mean) * (v - mean);
                        var /= double(c.action_sequence.size());
                    }
                    cd["accel_mean"]       = mean;
                    cd["accel_stddev"]     = std::sqrt(var);
                    chunks_arr.push_back(cd);
                }
                d["chunks"] = chunks_arr;
            }
        }
        // --- HomeokineticExploration ---
        else if (type == "DualEMADetector") {
            if (auto const* de = dynamic_cast<const ogma::DualEMADetector*>(m)) {
                d["fire_count"] = int(de->fire_count());
                d["short_ema"]  = double(de->short_ema());
                d["long_ema"]   = double(de->long_ema());
            }
        }
        else if (type == "EventConjunction") {
            if (auto const* ec = dynamic_cast<const ogma::EventConjunction*>(m)) {
                d["fire_count"]  = int(ec->fire_count());
                d["inputs_seen"] = int(ec->inputs_seen());
                Array fires;
                for (auto t : ec->last_fire_ticks()) fires.push_back(int64_t(t));
                d["last_fire_ticks"] = fires;
            }
        }
        else if (type == "ChunkAbortGate") {
            if (auto const* cg = dynamic_cast<const ogma::ChunkAbortGate*>(m)) {
                d["aborts_total"]    = int(cg->aborts_total());
                d["baseline_mean"]   = double(cg->baseline_mean());
                d["baseline_var"]    = double(cg->baseline_var());
                d["last_surprise"]   = double(cg->last_surprise_value());
                d["last_abort_tick"] = int64_t(cg->last_abort_tick());
            }
        }
        else if (type == "EpisodicCapture") {
            if (auto const* ec = dynamic_cast<const ogma::EpisodicCapture*>(m)) {
                d["keyframes_seen"]    = int(ec->keyframes_seen());
                d["intents_seen"]      = int(ec->intents_seen());
                d["proposals_emitted"] = int(ec->proposals_emitted());
                d["rewards_seen"]      = int(ec->rewards_seen());
                d["buffer_fill"]       = int(ec->buffer_fill());
                d["last_intent_index"] = int(ec->last_intent_index());
            }
        }
        else if (type == "ChunkOutcomeGate") {
            if (auto const* og = dynamic_cast<const ogma::ChunkOutcomeGate*>(m)) {
                d["aborts_total"]    = int(og->aborts_total());
                d["action_msgs"]     = int(og->action_msgs_seen());
                d["active_chunk_id"] = int(og->active_chunk_id());
                d["ticks_in_chunk"]  = int(og->ticks_in_chunk());
                d["signal_at_start"] = double(og->signal_at_start());
                d["current_signal"]  = double(og->current_signal());
            }
        }
        else if (type == "KeyframeAverager") {
            if (auto const* k = dynamic_cast<const ogma::KeyframeAverager*>(m)) {
                d["window_fill"]        = k->window_fill();
                d["window_size"]        = k->window_size();
                d["payload_dim"]        = k->payload_dim();
                d["total_inputs_seen"]  = k->total_inputs_seen();
                d["total_publishes"]    = k->total_publishes();
                auto const& mean = k->last_mean();
                if (!mean.empty()) {
                    PackedFloat64Array arr;
                    arr.resize(int(mean.size()));
                    for (size_t i = 0; i < mean.size(); ++i)
                        arr[int(i)] = double(mean[i]);
                    d["last_mean"] = arr;
                }
            }
        }
        else if (type == "HomeokineticExploration") {
            if (auto const* k = dynamic_cast<const ogma::HomeokineticExploration*>(m)) {
                d["episodes_armed"]      = int(k->episodes_armed());
                d["active"]              = k->active();
                d["episode_id"]          = int(k->current_episode_id());
                d["window_fill"]         = k->window_fill();
                // Phase 6.5.7 — gate-debug surface so we can see WHY
                // kinesis isn't firing during long MC lulls.  All four
                // fields are populated by HomeokineticExploration's
                // existing accessors.
                d["chunk_blocks"]        = k->gate_chunk_block();
                d["long_change_ema"]     = double(k->long_change_ema());
                d["urgency_buffer_fill"] = k->urgency_buffer_fill();
                d["ratio_buffer_fill"]   = k->ratio_buffer_fill();
                d["sample_count"]        = int(k->sample_count());
                d["success_rate"]        = double(k->success_rate());
                d["saturation_streak"]   = int(k->saturation_streak());
                d["entropy_collapse_fires"] = int(k->entropy_collapse_fires());
                d["entropy_tracked_count"]  = k->entropy_tracked_count();
            }
        }
        // --- MotorEPM (homeokinetic motor-TLE + loop gain) ---
        else if (type == "MotorEPM") {
            if (auto const* me = dynamic_cast<const ogma::MotorEPM*>(m)) {
                d["motor_tle"] = double(me->motor_tle_mean());
                d["loop_gain"] = double(me->loop_gain_mean());
                d["cog_steer"] = double(me->cog_steer_diag());
                d["cog_steer_msgs"] = me->cog_steer_msgs();
                d["boredom"]   = double(me->boredom_diag());
                d["interest"]  = double(me->interest_diag());
                d["hunger"]    = double(me->hunger_diag());
                d["boredom_streak"] = me->boredom_streak_diag();
                d["tc_x"]      = double(me->tc_x_diag());
                d["tc_y"]      = double(me->tc_y_diag());
            }
        }

        result[String(std::string(m->id()).c_str())] = d;
    }
    return result;
}

Array OgmaBrain::get_module_list() const {
    Array out;
    if (!initialized_) return out;
    for (auto const* m : instance_->modules()) {
        Dictionary d;
        d["id"]   = String(std::string(m->id()).c_str());
        d["type"] = String(std::string(m->type_name()).c_str());
        out.push_back(d);
    }
    return out;
}

Array OgmaBrain::get_graph_edges() const {
    Array out;
    if (!initialized_) return out;

    // Each emitted edge dict has an `is_implicit` boolean: false for edges
    // explicitly declared in the boot config or added via ConnectOp,
    // true for edges synthesised here by walking publisher/subscriber
    // topic topology.  The panel uses this for the edge-hover tooltip
    // ("explicit" vs "implicit") and to disable manual-routing-mode
    // disconnects on implicit edges.

    // 1a. Explicit edges from the boot-time GraphConfig.
    for (auto const& e : instance_->config().edges) {
        Dictionary d;
        d["from"]        = String(e.from.c_str());
        d["to"]          = String(e.to.c_str());
        d["topic"]       = String(e.topic.c_str());
        d["feedback"]    = e.feedback;
        d["is_implicit"] = false;
        out.push_back(d);
    }

    // 1b. Live scheduler edges — cumulative ConnectOp/DisconnectOp from
    // Patch Mode and external agents (Phase 6.6.D.6+ visualization fix).
    // Without this, drag-connects in the live graph panel vanish on the
    // periodic repopulate because the panel reads only get_graph_edges().
    for (auto const& e : instance_->current_edges()) {
        Dictionary d;
        d["from"]        = String(e.from.c_str());
        d["to"]          = String(e.to.c_str());
        d["topic"]       = String(e.topic.c_str());
        d["feedback"]    = e.feedback;
        d["is_implicit"] = false;
        out.push_back(d);
    }

    // 2. Implicit edges derived from publisher/subscriber topic topology.
    //    Build topic→publisher map, then for each subscriber's input_topic
    //    find matching publishers (exact or prefix-pattern).
    auto mods = instance_->modules();
    struct Pub { std::string topic; std::string module_id; };
    std::vector<Pub> pubs;
    for (auto const* m : mods) {
        std::string mid(m->id());
        for (auto const& t : m->output_topics()) {
            pubs.push_back({t.name, mid});
        }
    }
    for (auto const* m : mods) {
        std::string sub_id(m->id());
        for (auto const& t : m->input_topics()) {
            bool is_prefix = !t.name.empty() && t.name.back() == '.';
            for (auto const& p : pubs) {
                if (p.module_id == sub_id) continue;  // self-loop noise
                bool match = is_prefix
                    ? (p.topic.rfind(t.name, 0) == 0)  // starts with prefix
                    : (p.topic == t.name);
                if (!match) continue;
                Dictionary d;
                d["from"]        = String(p.module_id.c_str());
                d["to"]          = String(sub_id.c_str());
                d["topic"]       = String(p.topic.c_str());
                d["feedback"]    = (t.kind == ogma::SubscriptionKind::Feedback);
                d["is_implicit"] = true;
                out.push_back(d);
            }
        }
    }
    return out;
}

void OgmaBrain::set_auto_subscribe(bool enabled) {
    if (!initialized_ || !instance_) return;
    std::lock_guard<std::recursive_mutex> lk(instance_mtx_);
    instance_->scheduler()->set_auto_subscribe(enabled);
}

bool OgmaBrain::is_auto_subscribe() const {
    if (!initialized_ || !instance_) return true;
    std::lock_guard<std::recursive_mutex> lk(instance_mtx_);
    return instance_->scheduler()->is_auto_subscribe();
}

// ---------------------------------------------------------------------------
// Sensor registry
// ---------------------------------------------------------------------------

void OgmaBrain::register_source(String const& name, String const& topic,
                                  String const& description, bool active) {
    sensors_.push_back({name.utf8().get_data(), topic.utf8().get_data(),
                         description.utf8().get_data(), "source", "", active});
}

void OgmaBrain::register_sink(String const& name, String const& topic,
                               String const& description) {
    sensors_.push_back({name.utf8().get_data(), topic.utf8().get_data(),
                         description.utf8().get_data(), "sink", "", true});
}

void OgmaBrain::register_event(String const& name, String const& topic,
                                 String const& event_type) {
    sensors_.push_back({name.utf8().get_data(), topic.utf8().get_data(),
                         "", "event", event_type.utf8().get_data(), true});
}

int OgmaBrain::register_action_channel(String const& name, String const& topic) {
    std::string n = name.utf8().get_data();
    std::string t = topic.utf8().get_data();
    for (auto const& existing : action_channel_names_) {
        if (existing == n) return -1;
    }
    int idx = int(action_channel_topics_.size());
    action_channel_topics_.push_back(t);
    action_channel_names_.push_back(n);
    last_action_channels_.push_back(0.0);
    register_sink(name, topic, "v6.0 N-channel actuator (auto-polled each tick)");
    return idx;
}

double OgmaBrain::get_action_channel(int index) const {
    if (index < 0 || index >= int(last_action_channels_.size())) return 0.0;
    return last_action_channels_[index];
}

Array OgmaBrain::get_cpg_pure_bias() const {
    // Phase 7.x — return the first CPGOscillator's per-joint pure bias
    // (last_bias_walking[i] + last_bias_standing[i]).  This is the
    // CPG's contribution to action.<joint> BEFORE the brain command
    // is added; lets the C-mode visualisation show the pure
    // sine-generator output instead of brain + bias blended.
    Array out;
    if (!initialized_ || !instance_) return out;
    for (auto* mod : instance_->modules()) {
        if (auto const* cpg = dynamic_cast<const ogma::CPGOscillator*>(mod)) {
            auto const& bw = cpg->last_bias_walking();
            auto const& bs = cpg->last_bias_standing();
            int N = int(std::min(bw.size(), bs.size()));
            for (int i = 0; i < N; ++i)
                out.append(double(bw[i] + bs[i]));
            return out;
        }
    }
    return out;
}

Array OgmaBrain::get_sensor_registry() const {
    Array out;
    for (auto const& s : sensors_) {
        Dictionary d;
        d["name"]        = String(s.name.c_str());
        d["topic"]       = String(s.topic.c_str());
        d["description"] = String(s.description.c_str());
        d["kind"]        = String(s.kind.c_str());
        d["event_type"]  = String(s.event_type.c_str());
        d["active"]      = s.active;
        out.push_back(d);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Snapshot / restore (Phase 6.5.4)
// ---------------------------------------------------------------------------

String OgmaBrain::snapshot_state() {
    if (!instance_) return String();
    nlohmann::json snap = instance_->snapshot_state();
    return String(snap.dump().c_str());
}

void OgmaBrain::restore_state(String const& snapshot) {
    if (!instance_) return;
    std::string s = snapshot.utf8().get_data();
    if (s.empty()) return;
    try {
        nlohmann::json j = nlohmann::json::parse(s);
        instance_->restore_state(j);
    } catch (std::exception const& e) {
        UtilityFunctions::printerr(
            String("OgmaBrain::restore_state failed: ") + String(e.what()));
    }
}

String OgmaBrain::get_module_snapshot(String const& module_id) {
    if (!instance_) return String();
    std::string target = module_id.utf8().get_data();
    for (auto* m : instance_->modules()) {
        if (std::string(m->id()) == target) {
            nlohmann::json snap = m->snapshot_state();
            return String(snap.dump().c_str());
        }
    }
    return String();
}

bool OgmaBrain::set_module_snapshot(String const& module_id, String const& snapshot) {
    if (!instance_) return false;
    std::string target = module_id.utf8().get_data();
    std::string s      = snapshot.utf8().get_data();
    if (s.empty()) return false;
    for (auto* m : instance_->modules()) {
        if (std::string(m->id()) == target) {
            try {
                nlohmann::json j = nlohmann::json::parse(s);
                m->restore_state(j);
                return true;
            } catch (std::exception const& e) {
                UtilityFunctions::printerr(
                    String("OgmaBrain::set_module_snapshot(") + module_id +
                    String(") failed: ") + String(e.what()));
                return false;
            }
        }
    }
    UtilityFunctions::printerr(
        String("OgmaBrain::set_module_snapshot: module '") + module_id +
        String("' not found"));
    return false;
}

// ---------------------------------------------------------------------------
// Hot-patch API  (Phase 6.6.A)
// ---------------------------------------------------------------------------

namespace {

// Translate a Godot Variant into an ogma::ParamValue.  Throws on unsupported
// types so the caller can surface a precise error in the UI.
ogma::ParamValue variant_to_param_value(Variant const& v) {
    switch (v.get_type()) {
        case Variant::BOOL:
            return ogma::ParamValue{static_cast<bool>(v)};
        case Variant::INT:
            return ogma::ParamValue{static_cast<int64_t>(static_cast<int64_t>(v))};
        case Variant::FLOAT:
            return ogma::ParamValue{static_cast<double>(v)};
        case Variant::STRING:
        case Variant::STRING_NAME: {
            String s = v;
            return ogma::ParamValue{std::string(s.utf8().get_data())};
        }
        case Variant::PACKED_FLOAT64_ARRAY: {
            PackedFloat64Array arr = v;
            std::vector<double> out;
            out.reserve(arr.size());
            for (int i = 0; i < arr.size(); ++i) out.push_back(arr[i]);
            return ogma::ParamValue{std::move(out)};
        }
        case Variant::PACKED_FLOAT32_ARRAY: {
            PackedFloat32Array arr = v;
            std::vector<double> out;
            out.reserve(arr.size());
            for (int i = 0; i < arr.size(); ++i) out.push_back(double(arr[i]));
            return ogma::ParamValue{std::move(out)};
        }
        case Variant::PACKED_STRING_ARRAY: {
            PackedStringArray arr = v;
            std::vector<std::string> out;
            out.reserve(arr.size());
            for (int i = 0; i < arr.size(); ++i) {
                String s = arr[i];
                out.emplace_back(s.utf8().get_data());
            }
            return ogma::ParamValue{std::move(out)};
        }
        case Variant::ARRAY: {
            // Heterogeneous array — inspect first element to disambiguate
            // doubles vs strings.  Empty arrays default to vector<double>.
            Array arr = v;
            if (arr.size() == 0) return ogma::ParamValue{std::vector<double>{}};
            Variant::Type t = arr[0].get_type();
            if (t == Variant::STRING || t == Variant::STRING_NAME) {
                std::vector<std::string> out;
                out.reserve(arr.size());
                for (int i = 0; i < arr.size(); ++i) {
                    String s = arr[i];
                    out.emplace_back(s.utf8().get_data());
                }
                return ogma::ParamValue{std::move(out)};
            }
            std::vector<double> out;
            out.reserve(arr.size());
            for (int i = 0; i < arr.size(); ++i) {
                Variant e = arr[i];
                out.push_back(double(e));  // BOOL/INT/FLOAT all coerce
            }
            return ogma::ParamValue{std::move(out)};
        }
        default:
            throw std::runtime_error(
                "unsupported Variant type for ParamValue (got Variant::Type "
                + std::to_string(int(v.get_type())) + ")");
    }
}

// Convert a Godot Dictionary of <String, Variant> into an ogma::ParamMap.
ogma::ParamMap dict_to_param_map(Dictionary const& d) {
    ogma::ParamMap out;
    Array keys = d.keys();
    for (int i = 0; i < keys.size(); ++i) {
        String k = keys[i];
        std::string key(k.utf8().get_data());
        out.emplace(key, variant_to_param_value(d[keys[i]]));
    }
    return out;
}

// Read a required string field, throw on missing / wrong type.
std::string require_string(Dictionary const& d, char const* key) {
    if (!d.has(key))
        throw std::runtime_error(std::string("missing required field: ") + key);
    Variant v = d[key];
    if (v.get_type() != Variant::STRING && v.get_type() != Variant::STRING_NAME)
        throw std::runtime_error(std::string("field '") + key + "' must be a String");
    String s = v;
    return std::string(s.utf8().get_data());
}

// Optional string with default.
std::string opt_string(Dictionary const& d, char const* key, std::string const& def = "") {
    if (!d.has(key)) return def;
    Variant v = d[key];
    if (v.get_type() == Variant::NIL) return def;
    String s = v;
    return std::string(s.utf8().get_data());
}

ogma::GraphPatchOp parse_op(Dictionary const& op_dict) {
    std::string op = require_string(op_dict, "op");

    if (op == "add_node") {
        ogma::ModuleSpec spec;
        spec.id   = require_string(op_dict, "id");
        spec.type = require_string(op_dict, "type");
        if (op_dict.has("params")) {
            Variant pv = op_dict["params"];
            if (pv.get_type() == Variant::DICTIONARY) {
                Dictionary pd = pv;
                spec.params = dict_to_param_map(pd);
            } else if (pv.get_type() != Variant::NIL) {
                throw std::runtime_error("add_node 'params' must be a Dictionary");
            }
        }
        return ogma::AddNodeOp{std::move(spec)};
    }

    if (op == "remove_node") {
        return ogma::RemoveNodeOp{require_string(op_dict, "id")};
    }

    if (op == "connect") {
        ogma::EdgeSpec edge;
        edge.from  = require_string(op_dict, "from");
        edge.to    = require_string(op_dict, "to");
        edge.topic = opt_string(op_dict, "topic");
        if (op_dict.has("feedback")) {
            Variant fv = op_dict["feedback"];
            edge.feedback = bool(fv);
        }
        return ogma::ConnectOp{std::move(edge)};
    }

    if (op == "disconnect") {
        ogma::DisconnectOp d;
        d.from  = require_string(op_dict, "from");
        d.to    = require_string(op_dict, "to");
        d.topic = opt_string(op_dict, "topic");
        return d;
    }

    if (op == "set_param") {
        ogma::SetParamOp s;
        s.target_id = require_string(op_dict, "id");
        s.key       = require_string(op_dict, "key");
        if (!op_dict.has("value"))
            throw std::runtime_error("set_param requires 'value' field");
        s.value = variant_to_param_value(op_dict["value"]);
        return s;
    }

    throw std::runtime_error("unknown op: '" + op + "'");
}

} // anonymous namespace

Dictionary OgmaBrain::apply_patch(Dictionary const& patch) {
    Dictionary result;
    result["success"]  = false;
    result["error"]    = String();
    result["batch_id"] = int64_t(0);

    if (!initialized_ || !instance_) {
        result["error"] = String("brain not initialized");
        return result;
    }

    try {
        ogma::GraphPatchBatch batch;
        batch.source = "ui";

        if (patch.has("ops")) {
            // Batch form
            Variant ov = patch["ops"];
            if (ov.get_type() != Variant::ARRAY)
                throw std::runtime_error("'ops' must be an Array");
            Array ops = ov;
            batch.ops.reserve(ops.size());
            for (int i = 0; i < ops.size(); ++i) {
                Variant ev = ops[i];
                if (ev.get_type() != Variant::DICTIONARY)
                    throw std::runtime_error(
                        "ops[" + std::to_string(i) + "] must be a Dictionary");
                Dictionary od = ev;
                batch.ops.push_back(parse_op(od));
            }
            batch.source = opt_string(patch, "source", "ui");
        } else if (patch.has("op")) {
            // Single-op form — wrap in a 1-op batch
            batch.ops.push_back(parse_op(patch));
        } else {
            throw std::runtime_error("patch must contain either 'op' or 'ops'");
        }

        // Synchronous trial-validate every AddNodeOp before the patch is
        // enqueued.  The Scheduler's tick-time validation does the same
        // trial-construct against a dummy bus, but a failure there
        // throws into OgmaBrain::tick which surfaces only as a Godot
        // console error — the user clicks "Add" and watches the panel
        // do nothing.  Doing the trial here returns the real reason
        // synchronously so apply_patch's caller can show it in the
        // status bar (e.g. "missing required param 'targets'").
        for (auto const& op : batch.ops) {
            if (auto* add = std::get_if<ogma::AddNodeOp>(&op)) {
                try {
                    auto trial = ogma::ModuleRegistry::instance()
                                    .create(add->spec.type);
                    if (!trial) {
                        throw std::invalid_argument(
                            "unknown module type '" + add->spec.type + "'");
                    }
                    trial->set_id(add->spec.id);
                    ogma::InProcessBus dummy_bus;
                    trial->on_setup(&dummy_bus, add->spec.params);
                } catch (std::exception const& e) {
                    result["error"] = String(
                        ("AddNodeOp '" + add->spec.id + "' (" + add->spec.type
                         + ") failed validation: " + e.what()).c_str());
                    return result;
                }
            }
        }

        auto batch_id = instance_->enqueue_hot_patch(std::move(batch));
        result["success"]  = true;
        result["batch_id"] = int64_t(batch_id);
    } catch (std::exception const& e) {
        result["error"] = String(e.what());
    }
    return result;
}

PackedStringArray OgmaBrain::list_module_types() const {
    PackedStringArray out;
    auto types = ogma::ModuleRegistry::instance().registered_types();
    for (auto const& t : types) out.push_back(String(t.c_str()));
    return out;
}

namespace {

// ParamValue -> Variant for surfacing into Godot.
Variant param_value_to_variant(ogma::ParamValue const& v) {
    return std::visit([](auto const& x) -> Variant {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, bool>) {
            return Variant(x);
        } else if constexpr (std::is_same_v<T, int64_t>) {
            return Variant(int64_t(x));
        } else if constexpr (std::is_same_v<T, double>) {
            return Variant(double(x));
        } else if constexpr (std::is_same_v<T, std::string>) {
            return Variant(String(x.c_str()));
        } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            PackedFloat64Array a;
            for (auto d : x) a.push_back(d);
            return Variant(a);
        } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
            PackedStringArray a;
            for (auto const& s : x) a.push_back(String(s.c_str()));
            return Variant(a);
        }
    }, v);
}

// Type tag mirroring ParamValue's variant alternatives — used by the GDScript
// param editor to choose the right widget.
char const* param_value_type_tag(ogma::ParamValue const& v) {
    return std::visit([](auto const& x) -> char const* {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, bool>)                     return "bool";
        else if constexpr (std::is_same_v<T, int64_t>)             return "int";
        else if constexpr (std::is_same_v<T, double>)              return "float";
        else if constexpr (std::is_same_v<T, std::string>)         return "string";
        else if constexpr (std::is_same_v<T, std::vector<double>>) return "list_float";
        else                                                       return "list_string";
    }, v);
}

} // anonymous namespace

Array OgmaBrain::get_module_specs() const {
    Array out;
    if (!initialized_ || !instance_) return out;
    auto const& cfg = instance_->config();
    // id → boot-time ModuleSpec (params source-of-truth for unmodified mods).
    std::unordered_map<std::string, ogma::ModuleSpec const*> by_id;
    for (auto const& spec : cfg.modules) by_id[spec.id] = &spec;

    for (auto const* m : instance_->modules()) {
        Dictionary d;
        std::string id(m->id());
        d["id"]   = String(id.c_str());
        d["type"] = String(std::string(m->type_name()).c_str());

        Dictionary params_dict;
        // Prefer the module's own snapshot if it overrides current_params().
        ogma::ParamMap current = m->current_params();
        if (!current.empty()) {
            for (auto const& [k, v] : current)
                params_dict[String(k.c_str())] = param_value_to_variant(v);
        } else {
            auto it = by_id.find(id);
            if (it != by_id.end()) {
                for (auto const& [k, v] : it->second->params)
                    params_dict[String(k.c_str())] = param_value_to_variant(v);
            }
            // No fallback for hot-added modules whose type doesn't override
            // current_params() — they save with empty params and require
            // re-tuning on load.  Documented limitation.
        }
        d["params"] = params_dict;
        out.push_back(d);
    }
    return out;
}

Array OgmaBrain::get_module_param_schema(String const& id) const {
    Array out;
    if (!initialized_ || !instance_) return out;
    std::string sid(id.utf8().get_data());
    ogma::Module const* m = instance_->module(sid);
    if (m == nullptr) return out;

    ogma::ParamSchema schema = m->params_schema();
    ogma::ParamMap   current  = m->current_params();

    for (auto const& spec : schema) {
        Dictionary d;
        d["key"]         = String(spec.key.c_str());
        d["description"] = String(spec.description.c_str());
        d["mutability"]  = String(
            spec.mutability == ogma::ParamMutability::HotMutable
                ? "hot_mutable"
                : "construction_only");

        char const* type_tag = "string";
        Variant default_v;
        if (spec.default_value.has_value()) {
            default_v = param_value_to_variant(*spec.default_value);
            type_tag  = param_value_type_tag(*spec.default_value);
        }
        d["default_value"] = default_v;

        // Current value: prefer the module's own snapshot if it provided one
        // for this key; else fall back to schema default.
        Variant current_v = default_v;
        auto it = current.find(spec.key);
        if (it != current.end()) {
            current_v = param_value_to_variant(it->second);
            type_tag  = param_value_type_tag(it->second);
        }
        d["current_value"] = current_v;
        d["type"]          = String(type_tag);

        if (spec.min_value.has_value())
            d["min_value"] = param_value_to_variant(*spec.min_value);
        if (spec.max_value.has_value())
            d["max_value"] = param_value_to_variant(*spec.max_value);

        out.push_back(d);
    }
    return out;
}

} // namespace godot
