#pragma once

// =============================================================================
// OgmaBrain.hpp  --  GDObject wrapping a live OgmaInstance
// =============================================================================

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ogma {
class OgmaInstance;
class DiagPublisher;
}
namespace ami_ogma { namespace control { class ControlServer; } }

namespace godot {

class OgmaBrain : public Node {
    GDCLASS(OgmaBrain, Node)

public:
    OgmaBrain();
    ~OgmaBrain() override;

    // -----------------------------------------------------------------------
    // Core lifecycle
    // -----------------------------------------------------------------------
    bool setup(String const& config_path);
    void tick(double delta);

    // v6.0 — propagate the body's resolved seed (from
    // ExperimentConfig.resolve_seed() / OGMA_SEED) into every brain
    // module's master_seed param.  Must be called BEFORE setup().
    // For each ModuleSpec in the loaded graph whose params contain a
    // `master_seed` key, the value is replaced with
    // `namespace_seed(master_seed, module_id)` so every stochastic
    // module gets a distinct but deterministically-derived stream.
    // Pass 0 (default) to keep the hardcoded config values — useful
    // for golden replay where seed must come from the config alone.
    void set_master_seed(int64_t seed);
    void publish_proprio(PackedFloat64Array const& values, String const& sensor);
    void publish_event(String const& name, double intensity);
    // Publish a raw video frame on reality.video.<modality>.  pixels is row-major
    // H × W × C uint8 (channels = 1 grayscale or 3 RGB).  Wraps RawImageFrame.
    void publish_video(PackedByteArray const& pixels,
                       int height, int width, int channels,
                       String const& modality);

    // -----------------------------------------------------------------------
    // Fast-poll output accessors (updated every tick)
    // -----------------------------------------------------------------------
    double get_action()     const { return last_action_; }
    double get_urgency()    const { return last_urgency_; }
    double get_dopamine()   const { return last_dopamine_; }
    double get_serotonin()  const { return last_serotonin_; }
    int    get_node_count() const { return last_node_count_; }
    int    get_active_chunk_id() const { return last_chunk_id_; }
    bool   is_brain_ready() const { return initialized_; }

    // Phase 6.6.D.6 — bilateral motor accessors.  When both left and right
    // were published in the most recent tick, body code should drive each
    // actuator from its own channel (differential L/R produces steering
    // mechanically).  When only action.out is being published, fall back to
    // the legacy single-channel pathway.
    double get_action_left()       const { return last_action_left_; }
    double get_action_right()      const { return last_action_right_; }
    bool   is_action_bilateral()   const { return last_was_bilateral_; }

    // v6.0 — N-channel action polling for multi-actuator bodies (quadruped,
    // drone).  Body registers a named channel via register_action_channel
    // BEFORE setup(); each tick the brain polls every registered topic and
    // caches the latest ActionOut.accel value.  get_action_channel reads
    // the cached value by integer index (returned from register_action_channel)
    // — string-keyed map kept out of the hot path.  Channels are
    // INDEPENDENT of the bilateral fast-poll path; bodies that use both
    // (e.g. front-bilateral + rear-bilateral) can register channels for
    // the rear pair only.  Returns -1 if registration fails (called
    // after setup or duplicate name).
    int    register_action_channel(String const& name, String const& topic);
    double get_action_channel(int index) const;
    int    action_channel_count() const { return int(last_action_channels_.size()); }

    // Phase 7.x — exposes the first CPGOscillator's per-joint pure
    // bias (last_bias_walking[i] + last_bias_standing[i], BEFORE the
    // brain command is added).  Used by C-mode's CPG-drive visualisation
    // to show the pure sine-generator output without Premotor babbling
    // mixed in.  Returns an empty Array if no CPGOscillator is in the
    // current config.
    Array get_cpg_pure_bias() const;

    // -----------------------------------------------------------------------
    // Metrics API — used by both JSONL logger and GraphEdit panel
    // -----------------------------------------------------------------------

    // Per-module metrics keyed by module_id.
    // Values are Dictionaries with type-specific fields (see implementation).
    Dictionary get_module_metrics() const;

    // Phase 6.6.F — current MotorFader state (α, surprise scalar, source-
    // seen flags) for the on-screen meter widget.  Returns an empty
    // Dictionary when no MotorFader has ever published on
    // motor.fader.alpha (configs without a fader → meter hides itself).
    Dictionary get_motor_fader_state() const;
    // Multichannel motor bus-compressor state for the HUD fader panel: per-
    // influencer {names, gains, contrib_l/r, active} + out_l/r, sum_l/r, limit,
    // gr (gain reduction).  Empty Dictionary when no MotorBus is in the graph.
    Dictionary get_motor_bus_state() const;

    // Phase 6.6.F — live output topics for module `id`.  Replaces the
    // graph panel's hardcoded type→topics map (which couldn't see
    // per-instance `output_topic` params) by querying the live module's
    // `output_topics()` directly.  Returns an empty array when `id` is
    // unknown.  Each entry is a topic string.
    PackedStringArray get_module_output_topics(String const& id) const;

    // UI-dev W1 — typed channel ports.  Returns one Dictionary per declared
    // input / output topic on module `id`, with fields:
    //   name         : String — the topic name (e.g. "consensus.0").  Trailing-dot
    //                  prefixes indicate a wildcard subscription.
    //   payload_type : String — canonical short name from PayloadTypeName.hpp
    //                  ("RealityToken", "ActionOut", ...).  "Unknown" for an
    //                  unregistered type_index.
    //   kind         : String — "direct" | "feedback" (input only; outputs always "direct")
    //   required     : bool   — input only; mirrors TopicSpec.required.  Outputs default true.
    // Returns an empty Array when `id` is unknown or the brain isn't initialized.
    Array get_module_input_specs(String const& id) const;
    Array get_module_output_specs(String const& id) const;

    // Graph topology — module list and edges from the live OgmaInstance config.
    Array      get_module_list()  const;  // [{id, type}]
    Array      get_graph_edges()  const;  // [{from, to, topic, feedback, is_implicit}]

    // -----------------------------------------------------------------------
    // Per-graph routing mode (UI-dev manual-routing feature)
    // -----------------------------------------------------------------------
    //
    // When auto_subscribe=true (default), the bus dispatches every matching
    // delivery and modules' input gates pass everything through.
    // When false, modules default-deny: only producer ids reachable through
    // an explicit edge to the receiver are admitted.
    //
    // The Godot panel's Auto-subscribe CheckButton calls set_auto_subscribe;
    // is_auto_subscribe lets the panel mirror the live state.
    void  set_auto_subscribe(bool enabled);
    bool  is_auto_subscribe() const;

    // -----------------------------------------------------------------------
    // Hot-patch API  (Phase 6.6.A — Patch Mode UI)
    // -----------------------------------------------------------------------
    //
    // apply_patch translates a Godot Dictionary into a GraphPatchBatch and
    // enqueues it on the live OgmaInstance.  Patches are applied between
    // ticks by the Scheduler; validation against the live graph happens at
    // apply time, NOT at enqueue time, so success here means "parsed and
    // enqueued" — runtime application failure surfaces on the next tick via
    // the graph state diverging from the optimistic UI.
    //
    // Accepted shapes:
    //   - Single op:  { "op": "add_node", "id": ..., "type": ..., "params": {...} }
    //   - Batch:      { "ops": [ {op-dict}, {op-dict}, ... ], "source": "ui" }
    //
    // Op vocabulary (mirrors GraphConfig.hpp:136-164):
    //   add_node     { id, type, params? }
    //   remove_node  { id }
    //   connect      { from, to, topic?, feedback? }
    //   disconnect   { from, to, topic? }
    //   set_param    { id, key, value }
    //
    // Returns:
    //   { success: bool, error: String, batch_id: int }
    Dictionary       apply_patch(Dictionary const& patch);

    // Names of every module type registered with ModuleRegistry — drives the
    // "Add Module" popup in Patch Mode.
    PackedStringArray list_module_types() const;

    // Live module list with their original (boot-time) params, suitable for
    // serialising the topology back to JSON.  Entries:
    //   {id, type, params: Dictionary<String,Variant>}
    // Params are sourced first from the module's current_params() override
    // when it provides one, else from the GraphConfig spec the brain was
    // booted with — accurate for unmodified modules but loses subsequent
    // SetParamOp mutations (the Scheduler does not yet expose post-mutation
    // values).  Hot-added modules have empty params unless their type
    // overrides current_params().
    Array get_module_specs() const;

    // Param schema for the live module `id` with current values folded in.
    // Returned as Array of Dictionaries, one per param:
    //   {key, type, mutability, description, default_value, current_value,
    //    min_value (optional), max_value (optional)}
    // - `type`        : "bool" | "int" | "float" | "string" | "list_float" | "list_string"
    // - `mutability`  : "construction_only" | "hot_mutable"
    // - `current_value`: from Module::current_params() if the module overrides
    //                    that hook, otherwise falls back to default_value.
    // Returns an empty Array if `id` is not a live module.
    Array get_module_param_schema(String const& id) const;

    // -----------------------------------------------------------------------
    // Sensor registry — host declares every source/sink/event so the graph
    // panel can display the full environment↔brain interface for auditability.
    // Called from body_controller.gd _ready().
    // -----------------------------------------------------------------------
    void register_source(String const& name, String const& topic,
                          String const& description, bool active);
    void register_sink(String const& name, String const& topic,
                        String const& description);
    void register_event(String const& name, String const& topic,
                         String const& event_type);  // "reward" | "aversive"

    // Returns the full registry as an Array of Dictionaries for the panel.
    Array get_sensor_registry() const;

    // -----------------------------------------------------------------------
    // Snapshot / restore  (Phase 6.5.4 — benchmark-without-amnesia)
    // -----------------------------------------------------------------------
    //
    // Returns the full live brain state (every module's working state plus
    // bus last-value cache) as a JSON string.  `restore_state(s)` applies a
    // previously-taken snapshot in-place — both calls require the brain to
    // be initialized and use byte-equivalent GraphConfig.
    //
    // Use case: "snapshot, run a 100-episode benchmark with plasticity ON,
    // restore" — the live brain is rolled back after the benchmark, so any
    // amnesia or drift introduced by the artificial 200-tick episodic
    // protocol is undone.  Mirrors the principle that biological organisms
    // perform benchmarks with fixed policies, while the substrate must
    // continue learning continuously when not under test.
    //
    // Snapshot strings are typically a few KB to a few MB depending on the
    // GNG node count and Q-table size.  Safe to hold in GDScript memory.
    String snapshot_state();
    void   restore_state(String const& snapshot);

    // Per-module snapshot accessors.  Same JSON format as the whole-brain
    // snapshot but scoped to a single module identified by its
    // configured id (e.g. "premotor_fl_hip1").  Returns "" if the module
    // is not found.  Used by the picrawler leg-symmetry hook to read
    // and average Premotor weight matrices across mirrored leg pairs at
    // mc_episode_period boundaries.
    String get_module_snapshot(String const& module_id);
    bool   set_module_snapshot(String const& module_id, String const& snapshot);

    static void _bind_methods();

private:
    std::unique_ptr<ogma::OgmaInstance> instance_;

    // UI-dev W2 — inspector control surface.
    // ControlServer hosts the JSON-RPC command verbs (list_modules,
    // module_snapshot, module_subscribe_diag, unsubscribe).  DiagPublisher
    // owns a ZMQ PUB socket on a separate port for high-rate streaming
    // telemetry — sidecar SUBs subscribe to a per-subscription topic
    // prefix and receive serialized snapshot_state at their requested Hz.
    // Both default-off; setup() starts them on configured ports.
    std::unique_ptr<ami_ogma::control::ControlServer> control_server_;
    std::unique_ptr<ogma::DiagPublisher>              diag_publisher_;

    // Serialises tick-thread mutation of the OgmaInstance against
    // ControlServer command-handler reads from the inspector.  The
    // control server runs handler callbacks on its own detached
    // thread; without this guard, a verb like `module_snapshot` or
    // `list_modules` can iterate / dereference modules_ while the
    // tick thread is in the middle of apply_remove (which erases
    // unique_ptrs from the same vector and runs each module's
    // on_teardown).  Crashes when removing a module the inspector
    // happened to be reading at the time.
    //
    // Held by:
    //   * tick(): for the duration of instance_->tick() AND
    //     diag_publisher_->publish_tick(), so an inspector verb
    //     cannot race with a hot-patch apply.
    //   * control-server command handler: around any call into
    //     instance_ (module(), modules(), snapshot_state, ...).
    //
    // recursive_mutex so e.g. snapshot_state -> instance_->snapshot
    // doesn't self-deadlock if called from the same thread.
    mutable std::recursive_mutex instance_mtx_;

    // Fast-poll cache
    double   last_action_      = 0.0;
    double   last_urgency_     = 0.0;
    double   last_dopamine_    = 0.20;
    double   last_serotonin_   = 0.65;
    int      last_node_count_  = 0;
    int      last_chunk_id_    = -1;
    bool     initialized_      = false;
    uint64_t tick_id_          = 0;
    // v6.0 — runtime master seed (set via set_master_seed before setup()).
    // 0 = no override; the config's hardcoded module master_seed values
    // are used as-is.  Non-zero = each module's master_seed param is
    // overridden by namespace_seed(master_seed_override_, module_id) so
    // OGMA_SEED actually shapes the brain's stochastic streams.
    uint64_t master_seed_override_ = 0;
    // Phase 6.6.D.6 — bilateral cache.  Both scalars updated whenever each
    // channel publishes a fresh ActionOut; last_was_bilateral_ flips true
    // for the duration of the next frame whenever BOTH channels published
    // in the just-completed scheduler tick.  Body uses is_action_bilateral()
    // to choose its pathway.
    double   last_action_left_       = 0.0;
    double   last_action_right_      = 0.0;
    bool     last_was_bilateral_     = false;

    // v6.0 — N-channel action cache for multi-actuator bodies.  Parallel
    // arrays: action_channel_topics_[i] is the bus topic, names_[i] is
    // the developer-friendly identifier (echoed back from
    // register_action_channel), last_action_channels_[i] is the cached
    // float updated each tick from bus->last_value(topic).  Names must
    // be unique; duplicate register returns -1.  Cleared on every
    // setup() so re-launching with a new config starts fresh.
    std::vector<std::string> action_channel_topics_;
    std::vector<std::string> action_channel_names_;
    std::vector<double>      last_action_channels_;

    // Phase 6.6.F — last MotorFader state cache.  fader_seen_ stays false
    // until the first FaderState arrives, which is how the HUD widget
    // distinguishes "MotorFader not in graph" from "MotorFader at α=0".
    bool     fader_seen_         = false;
    double   fader_alpha_        = 0.0;
    double   fader_alpha_target_ = 0.0;
    double   fader_surprise_     = 0.0;
    bool     fader_brain_seen_   = false;
    bool     fader_reflex_seen_  = false;
    double   fader_brain_accel_  = 0.0;
    double   fader_reflex_accel_ = 0.0;
    double   fader_output_accel_ = 0.0;
    // v5.4 — clash = intent erased when brain & reflex contributions have
    // opposite signs.  Surfaced for the brain-state HUD widget.
    double   fader_clash_        = 0.0;
    double   fader_clash_ema_    = 0.0;
    std::string fader_source_;

    // Sensor registry
    struct SensorEntry {
        std::string name;
        std::string topic;
        std::string description;
        std::string kind;        // "source" | "sink" | "event"
        std::string event_type;  // "reward" | "aversive" (events only)
        bool        active;
    };
    std::vector<SensorEntry> sensors_;
};

} // namespace godot
