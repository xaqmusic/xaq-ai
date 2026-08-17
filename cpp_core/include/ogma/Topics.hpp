#pragma once

// =============================================================================
// Topics.hpp  --  Strongly-typed payloads for the Ogma Core cellular bath
// =============================================================================
//
// Every Bus message carries a payload that derives from `Message`.  Subscribers
// receive `std::shared_ptr<const Message>` and `dynamic_cast` to the concrete
// type they expect.  The contract for each topic name -> payload type lives in
// this header; downstream modules `#include <ogma/Topics.hpp>` and refer to
// these structs by name.
//
// Topic-name conventions (the "logical address" string passed to Bus::publish):
//
//   reality.<group>.<modality>     EPM output for one modality.
//                                    e.g.  reality.video.retinal
//                                          reality.video.saliency
//                                          reality.video.flow
//                                          reality.audio.stft
//                                          reality.proprio.imu
//                                          reality.proprio.whisker
//                                          reality.proprio.scent
//   consensus.<level>              LateralVoter output at a given level.
//                                    e.g.  consensus.0  (over reality.*)
//                                          consensus.1  (over consensus.0 via L1 EPMs)
//   neuro.state                    NeurochemState broadcast.  Singleton topic.
//   drive.errors                   HomeostaticDrive output.  Singleton topic.
//   action.out                     ActionDecoder output.  Singleton topic.
//   prediction.<modality>          DescendingPredictor's expected next latent
//                                  for one downstream EPM.
//   sequence.motif.<source>        SequenceGNG output for one input stream.
//                                    e.g.  sequence.motif.consensus.0
//                                          sequence.motif.action.out
//   motor.chunks                   MotorRepertoire library snapshot.
//   motor.play.cmd                 REQ→MotorRepertoire: "play this chunk".
//   motor.play.stream              ←REP from MotorRepertoire: action stream.
//   rollout.query                  REQ→GNGRollout.
//   rollout.result                 ←REP from GNGRollout.
//   fitness.score                  Per-OgmaInstance fitness scalar (Phase 6).
//   events.<name>                  Discrete environment events bridged in by
//                                  the host (events.hit, events.miss,
//                                  events.brick, events.wall_stuck,
//                                  events.whisker_bump).
//
// Hierarchical namespace with `.` as the separator is fixed.  Wildcards are
// prefix-matched (see Bus.hpp): `subscribe("reality.")` catches every
// `reality.<group>.<modality>` topic.  This is the design rule that lets
// LateralVoter parse the modality group from the topic name without any
// per-modality config plumbing.
//
// Payload immutability.  Every Message is const-by-contract once published.
// A subscriber may inspect but never mutate.  InProcessBus dispatches
// `std::shared_ptr<const Message>` directly; ZmqBus serializes via Message's
// `serialize()` / a registered factory and produces a fresh instance per
// recipient.  Modules must not pass these pointers between threads except via
// the Bus.

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

namespace ogma {

// -----------------------------------------------------------------------------
// Base class for all bus payloads.
// -----------------------------------------------------------------------------
//
// `tick_id` is the Scheduler's monotonically-increasing global tick counter at
// the moment the payload was produced; consumers reading via the Bus's
// last-known-value cache use this to detect stale data and to distinguish
// current-tick (t) from feedback (t-1) reads.
//
// `producer_id` is the publishing module's `Module::name()`.  Useful for trace
// assertions in Phase 2 (verifying DAG-level dispatch order).

struct Message {
    uint64_t    tick_id     = 0;
    std::string producer_id;
    virtual ~Message() = default;
};

using MessagePtr = std::shared_ptr<const Message>;

// -----------------------------------------------------------------------------
// reality.<group>.<modality>  --  EPM output
// -----------------------------------------------------------------------------
//
// Carries everything a downstream LateralVoter, NeurochemState, or
// DescendingPredictor needs to make a decision about this modality's
// contribution to the current tick.  `latent` is the encoder output (typically
// 128-D); `winner_prototype` is the GNG winner's prototype vector (same dim).

struct RealityToken : Message {
    int     winner_id        = -1;     // stable GNG node ID
    float   quant_error      = 0.0f;   // distance from latent to winner prototype
    float   transition_surp  = 0.0f;   // distance between consecutive winners
    float   tle              = 0.0f;   // dual TLE = α·QE + β·transition_surp
    float   novelty_threshold = 0.0f;  // adaptive threshold this tick
    bool    is_novel         = false;
    bool    just_baked       = false;
    bool    just_pruned      = false;
    bool    just_mitosis     = false;
    int     node_count       = 0;
    int     baked_count       = 0;
    int     mitosis_count    = 0;
    std::vector<int> pruned_ids;
    std::vector<int> history_trace;    // last N winner IDs for sequence learners
    // Phase 6.6.E: forward-pointing analogue of history_trace.  Empty unless
    // the EPM is configured with predicted_pathway_steps > 0.  Sequence of
    // node IDs the EPM expects to win in the next M ticks, produced by a
    // greedy walk over its own per-node transition counts.  Receivers
    // (LateralVoter, future Premotor consumers) look up the embedding via
    // EmbeddingRegistry rather than carrying it on the wire.
    std::vector<int> predicted_pathway;

    Eigen::VectorXf latent;             // encoder output, dim = projection_dim
    Eigen::VectorXf winner_prototype;   // GNG node prototype (same dim)
};

// -----------------------------------------------------------------------------
// consensus.<level>  --  LateralVoter output
// -----------------------------------------------------------------------------
//
// Fused embedding aggregates RealityTokens from one level's EPMs with
// modality-group balancing applied.  `active_modality` reports which group's
// most-active member won the position-encoding priority used by ActionDecoder.
// Trust weights are exposed for tracing/debugging.

struct ConsensusToken : Message {
    Eigen::VectorXf fused_embedding;   // typically 128-D
    float           fused_tle = 0.0f;
    int             level     = 0;     // 0 = first voter, 1+ = higher-level

    std::string     active_modality;   // e.g. "video.retinal"
    int             active_winner_id = -1;

    // Per-input-modality bookkeeping (modality -> trust weight in [0,1]).
    std::unordered_map<std::string, float> trust_weights;

    // Phase 6.6.F — per-modality predicted_pathway surprise EMA, copied
    // from the LateralVoter's internal surprise_ema_ map at fusion time.
    // Empty when the voter is configured with surprise_gain=0 (no
    // predictions). MotorFader aggregates this into a single scalar to
    // drive α (see docs/v4_phase6_6f_motor_fader_plan.md).
    std::unordered_map<std::string, float> surprise_ema;

    // Phase 6.6.I — per-modality forward rollouts and current winners,
    // mirrored through from the input RealityTokens at fusion time so
    // downstream consumers (Premotor's rollout-aware exploration) can
    // reason about predicted future states without re-subscribing to
    // every reality.* topic.  predicted_pathways is empty when no input
    // EPM is configured with predicted_pathway_steps > 0.
    std::unordered_map<std::string, std::vector<int>> predicted_pathways;
    std::unordered_map<std::string, int>              winner_ids_by_modality;
};

// -----------------------------------------------------------------------------
// neuro.state  --  NeurochemState broadcast (singleton topic)
// -----------------------------------------------------------------------------
//
// Bundles the four scaling factors EPM/GNG consume each tick plus the reward
// signal ActionDecoder credits against the Hebbian table.  The `*_scale`
// fields are dimensionless multiplicands: e.g. an EPM scales its working
// `epsilon_b` by `epsilon_b_scale` before stepping the GNG.

struct NeuroState : Message {
    float dopamine                    = 0.0f;  // [0, 1]
    float serotonin                   = 0.0f;  // [0, 1]
    float reward_signal               = 0.0f;  // dopamine - baseline, ~[-0.45, +0.8]

    float epsilon_b_scale             = 1.0f;  // [~0.3, ~2.5]
    float min_insertion_error_scale   = 1.0f;  // [~0.5, ~1.8]
    float mitosis_threshold_scale     = 1.0f;  // [~0.6, ~1.8]
    float novelty_threshold_scale     = 1.0f;  // [~0.5, ~1.5]
};

// -----------------------------------------------------------------------------
// drive.errors  --  HomeostaticDrive output (singleton topic)
// -----------------------------------------------------------------------------
//
// Per-setpoint deviation vector + urgency scalar.  ActionDecoder reads this
// directly; `urgency` modulates exploitation-vs-exploration in the EFE policy.
// Setpoint identity is by string key so the body schema can declare e.g.
// {"energy", "integrity", "novelty_satiation"} and add new channels without
// schema migration.

struct DriveErrors : Message {
    std::unordered_map<std::string, float> errors;  // setpoint name -> deviation
    float urgency = 0.0f;                            // max normalized deviation
};

// -----------------------------------------------------------------------------
// action.out  --  ActionDecoder output (singleton topic)
// -----------------------------------------------------------------------------
//
// A single tick's motor command.  When `chunk_id` is set, MotorRepertoire owns
// playback for the next `chunk_remaining_ticks`; otherwise `accel` is the
// scalar force.  Hosts (Godot, HAL) translate `accel` to actuator units.

struct ActionOut : Message {
    float   accel                 = 0.0f;   // [-4, +4] in v3 units
    int     chunk_id              = -1;     // -1 = no chunk active
    int     chunk_remaining_ticks = 0;
    bool    probe                 = false;  // true if action came from probe RNG
    // Phase 6.5.3.1 — action TLE: prediction error of ActionDecoder's
    // forward model on the most recent (prev_state, prev_bin) → s'
    // transition.  1.0 = surprised (unseen outcome), 0.0 = perfectly
    // predicted.  Mirror of EPM TLE on the perception side; closes the
    // substrate's TLE-asymmetry per v4_phase6_5_3_action_layer_plan.md.
    float   action_tle            = 0.0f;
    // Phase 6.5.25 — telemetry: which premotor stream produced this
    // action.  "decoder" = legacy EFE ActionDecoder; "premotor" = graded
    // policy module; "chunk" = MotorRepertoire replay; "explore" =
    // HomeokineticExploration directive.  Empty = unspecified.
    std::string source            = "";
    // v5.4 Phase F (Proposal B) — when ActionDecoder dispatches a chunk,
    // it stamps the chunk's score (Beta-prior × entry-match-product)
    // here so FaderController.alpha_source="chunk_quality_sigmoid" can
    // read it and modulate α continuously by chunk quality.  0 when no
    // chunk is replaying.
    float chunk_quality           = 0.0f;
    // v5.4 Phase G (Proposal C) — body-keyframe position currently being
    // replayed (0..body_keyframes-1).  Lets MotorRepertoire credit the
    // exact position when a reward fires DURING chunk replay, so
    // chunks decompose into per-position Beta-priors rather than a
    // single chunk-wide statistic.  -1 when no chunk active.
    int   chunk_position          = -1;
};

// -----------------------------------------------------------------------------
// motor.fader.alpha  --  MotorFader telemetry (Phase 6.6.F)
// -----------------------------------------------------------------------------
//
// Singleton topic.  Tiny payload published by MotorFader every tick it
// produces an action.  α is the smoothed brain↔reflex blend coefficient
// (0 = pure reflex, 1 = pure brain).  Premotor's Hebbian update gates on
// α via update_alpha_threshold (off-policy contamination guard).  HUDs
// render the on-screen meter from these fields.

struct FaderState : Message {
    float       alpha            = 0.0f;   // smoothed α applied this tick
    float       alpha_target     = 0.0f;   // pre-smoothing target
    float       surprise_scalar  = 0.0f;   // aggregated surprise (mean/max)
    bool        brain_seen       = false;  // brain side published this tick
    bool        reflex_seen      = false;  // reflex side published this tick
    std::string source           = "";     // "fixed" | "surprise"
    // Raw component accels + final blend, exposed so the HUD meter (or any
    // other observer) can directly verify the math instead of inferring it
    // from agent motion.  brain_accel/reflex_accel are 0 when the
    // corresponding side did not publish this tick.
    float       brain_accel      = 0.0f;
    float       reflex_accel     = 0.0f;
    float       output_accel     = 0.0f;
};

// -----------------------------------------------------------------------------
// policy.intent  --  Premotor module output (Phase 6.5.25)
// -----------------------------------------------------------------------------
//
// A *graded* policy distribution over named motor intents.  Decouples
// "what should I do" (continuous, learned) from "what action right now"
// (discrete, sampled) — the missing biomimetic layer between consensus
// fusion and motor commands.  See docs/v4_phase6_5_25_premotor.md (TBD).
//
// intent_distribution sums to 1.0 across intent_names.size() bins.
// intent_accels gives the accel value each intent maps to in body units
// ([-accel_max, +accel_max]).  weighted_accel is the trust-weighted
// mean (sum_i intent_distribution[i] * intent_accels[i]) — a smooth
// graded action ready for body consumption without further sampling.

struct PolicyToken : Message {
    Eigen::VectorXf            intent_distribution;  // softmax over named intents
    std::vector<std::string>   intent_names;         // e.g., {"hard_left","slow_left","neutral","slow_right","hard_right"}
    std::vector<float>         intent_accels;        // accel value per intent (same length as names)
    float                      weighted_accel = 0.0f;// distribution-weighted mean accel
    float                      entropy        = 0.0f;// shannon entropy of distribution; high = uncertain
    float                      temperature    = 1.0f;// softmax temperature actually used (post-DA modulation)
    int                        chosen_intent  = -1;  // -1 = no sample; ≥0 = sampled intent index
};

// -----------------------------------------------------------------------------
// intent.override  --  ActionDecoder → Premotor chunk-replay override (v5.3 Phase B)
// -----------------------------------------------------------------------------
//
// During chunk replay, ActionDecoder publishes one IntentToken per tick on
// intent.override carrying the intent index from the chunk's
// intent_sequence.  Premotor (when configured with intent_override_topic)
// substitutes this index for its own softmax sample for that tick — both
// motor emission AND REINFORCE crediting happen on the override-supplied
// index, so chunk dispatches naturally feed the policy gradient.
//
// Bilateral semantics live entirely in Premotor's intent table; the chunk
// pipeline never sees (L,R) floats.  A chunk is just a sequence of intent
// indices that resolve to bilateral motor commands via the same table
// Premotor uses for its own samples.  Cognitive plan unit composed over
// the existing intent vocabulary, not a parallel motor primitive.

struct IntentToken : Message {
    int index = -1;   // single-channel intent index this tick (-1 = no override).
                      // Legacy single-channel chunks populate only this field.
    // Phase 7.2-EPM multi-channel chunks: when a chunk crystallises across
    // multiple Premotors (e.g., 3 joints in one leg), ActionDecoder publishes
    // the per-tick vector here.  Subscribers (Premotors) read `indices[intent_channel]`
    // when their intent_channel param is >= 0; otherwise they fall back to
    // `index` (legacy behaviour).  Single-channel chunks populate ONLY index;
    // multi-channel chunks populate indices AND set index = indices[0] for
    // back-compat with subscribers that don't yet know about the vector.
    std::vector<int> indices;
};

// -----------------------------------------------------------------------------
// prediction.<modality>  --  DescendingPredictor output
// -----------------------------------------------------------------------------
//
// One predicted next-tick latent per downstream EPM.  The EPM subtracts this
// from its raw encoder output before feeding the GNG, so the GNG topologizes
// surprise rather than raw input.

struct PredictionToken : Message {
    std::string     target_modality;   // which `reality.<group>.<modality>` this targets
    Eigen::VectorXf predicted_latent;  // dim = target's projection_dim
    float           confidence = 0.0f; // [0, 1] — predictor's self-rated accuracy
};

// -----------------------------------------------------------------------------
// sequence.motif.<source>  --  SequenceGNG output
// -----------------------------------------------------------------------------
//
// Reports the active motif (a baked n-gram of winner transitions) at the
// SequenceGNG's current position, plus the predicted next transition for
// downstream rollout / chunk-boundary detection.

struct SequenceMotif : Message {
    int     motif_id          = -1;
    float   match_confidence  = 0.0f;
    int     phase             = 0;       // current position within the motif (0..N-1)
    int     motif_length      = 0;       // total length N
    int     predicted_next_id = -1;      // expected next winner ID
    bool    just_baked        = false;   // true ONLY on the tick the bake fires
    bool    is_baked          = false;   // persistent: motif's underlying SeqGNG node is crystallised
};

// -----------------------------------------------------------------------------
// exploration.directive  --  HomeokineticExploration output (singleton topic)
// -----------------------------------------------------------------------------
//
// Stuck-detector directive.  When the windowed gate (urgency rising AND
// drive-delta flat AND no chunk applicable) trips, the primitive samples a
// held accel value from a deterministic PRNG and arms the directive for
// `episode_ticks` ticks.  ActionDecoder subscribes; while `active==true` it
// overrides its EFE-selected action with `accel`.  Outside an episode the
// directive is published with `active=false` so subscribers never need to
// handle a missing message.
//
// Replaces the v3 / Phase-1 Probe machine inside ActionDecoder, which gated
// on a satiety proxy (serotonin > X AND dopamine < Y) rather than on
// architectural failure of the drive/chunk loop.

struct ExplorationDirective : Message {
    bool     active          = false;
    int      ticks_remaining = 0;
    uint64_t episode_id      = 0;
    float    accel           = 0.0f;   // held sample; ActionDecoder clamps to its own [accel_min, accel_max]
};

// -----------------------------------------------------------------------------
// motor.chunks  --  MotorRepertoire library snapshot (broadcast on change)
// -----------------------------------------------------------------------------

struct MotorChunk {
    int                 id = -1;
    std::vector<float>  action_sequence;        // tick-by-tick motor commands (legacy / pre-v5.3)
    // v5.3 Phase B — when non-empty, the chunk is an "intent chunk":
    // tick-by-tick intent indices replayed by ActionDecoder via
    // intent.override (the chunk pipeline never sees (L,R) floats; the
    // bilateral motor mapping happens in Premotor's intent table).  Both
    // formats coexist for backward compatibility with legacy chunk
    // configs; only one is non-empty per chunk.  When intent_sequence is
    // populated, action_sequence is empty (and vice versa).
    std::vector<int>    intent_sequence;
    // v5.4 Phase A — episodic chunk entry context.  Paired embeddings of
    // the K slow-consensus keyframes immediately preceding the chunk's
    // motor body.  Dispatch fires when ALL K embeddings cosine-match
    // the brain's last K consumed keyframes above min_entry_similarity.
    // Empty = legacy chunk (Beta-prior dispatch only).
    std::vector<Eigen::VectorXf> entry_embeddings;
    // v5.4 Phase G (Proposal C) — per-body-position credit.  Both vectors
    // have body_keyframes entries (= intent_sequence.size() /
    // playback_ticks_per_intent for episodic chunks).  Updated each time
    // events.hit or events.miss fires during chunk replay against the
    // chunk's current chunk_position.  When non-empty, ActionDecoder
    // computes chunk_score as MEAN of per-position Beta(1,1) priors
    // instead of the chunk-wide aggregate.  Empty = legacy chunk-wide.
    // v5.4 Phase H — type widened to float to accept fractional eligibility-
    // trace credit (decay^position weights) and per-tick freshness decay.
    std::vector<float>  position_hits;
    std::vector<float>  position_misses;
    int                 body_keyframes      = 0;     // derived from chunk shape
    int                 playback_ticks_per_position = 1;
    Eigen::VectorXf     entry_state_prototype;  // when does this chunk apply?  (Phase-4 stretch — superseded by trigger_consensus_motif_id)
    float               outcome_drive_delta = 0.0f;  // expected drive-error reduction
    int                 use_count           = 0;
    // Consensus (perceptual) motif id that was the baked winner at hit time.
    // ActionDecoder dispatches a chunk only when the current consensus
    // motif matches, so chunks become (perceptual context → motor sequence)
    // mappings rather than action self-loops.  -1 if no consensus context
    // was available at crystallisation (chunk fires unconditionally then).
    int                 trigger_consensus_motif_id = -1;
    // Phase 6.5.12 — hybrid trigger: drive urgency at the hit moment.
    // ActionDecoder ALSO dispatches when current_urgency >= this × tolerance,
    // letting chunks fire in environments where consensus motifs at goal-
    // approach are too perceptually variable to recur (MC, future quadruped).
    // -1.0f = no drive trigger captured (back-compat for older chunks).
    float               trigger_urgency            = -1.0f;
    // Post-crystallization outcomes.  hits_during is the pre-crystallization
    // count from the gate; replay_{hits,misses} accumulate from events.hit
    // / events.miss arriving while ActionDecoder is replaying this chunk.
    // Combined success rate = (hits_during + replay_hits + 1) /
    //                          (hits_during + replay_hits + replay_misses + 2)
    // (Beta(1,1) prior — starts 0.5 for new chunks, refines with use).
    // v5.4 Phase H — replay_{hits,misses} widened to float to accept
    // fractional credit from the chunk-dispatch eligibility trace
    // (recent N dispatches share hit credit with decay^position weights)
    // and from per-tick freshness decay (slow drift toward 0).
    // hits_during stays int — pre-crystallisation gate count, not subject
    // to ongoing decay.
    int                 hits_during         = 0;
    float               replay_hits         = 0.0f;
    float               replay_misses       = 0.0f;
    // v5.4.L — tick when this chunk was first crystallised.  ActionDecoder
    // uses (current_tick - created_tick_id) ≥ chunk_dispatch_min_age_ticks
    // as a pre-dispatch gate so freshly-captured chunks can't fire on the
    // very next tick after the reward event that created them (the
    // canonical eat→replay loop the entry-match gate alone can't prevent
    // when the slow consensus encoding is degenerate).
    uint64_t            created_tick_id     = 0;
};

struct MotorChunks : Message {
    std::vector<MotorChunk> chunks;
};

// -----------------------------------------------------------------------------
// rollout.query / rollout.result  --  GNGRollout REQ/REP
// -----------------------------------------------------------------------------

struct RolloutQuery : Message {
    std::string source_modality;
    int     winner_id   = -1;
    float   action      = 0.0f;        // candidate action being evaluated
    int     M_steps     = 5;           // forward horizon
    int     K_samples   = 32;          // trajectories
    uint64_t request_id = 0;           // matches result back to caller
};

struct RolloutResult : Message {
    uint64_t                              request_id = 0;
    std::vector<std::vector<int>>         trajectories;     // [K][M] node IDs
    std::vector<float>                    terminal_values;  // [K]
    float                                 entropy = 0.0f;   // policy uncertainty
};

// -----------------------------------------------------------------------------
// motor.play.cmd / motor.play.stream  --  MotorRepertoire REQ/REP
// -----------------------------------------------------------------------------

struct MotorPlayCmd : Message {
    int     chunk_id   = -1;
    uint64_t request_id = 0;
    // v5.4 Phase C — manual probe.  When true, MotorRepertoire ignores
    // the is_active gate (Wilson-CI demoted chunks still play) AND skips
    // evaluate_dispatch_outcome_ for the previous dispatch (so probing a
    // chunk doesn't mark it as a miss against whatever was playing before).
    // Default false preserves all existing dispatch semantics.
    bool    force      = false;
};

// -----------------------------------------------------------------------------
// motor.episodic_proposal  --  EpisodicCapture → MotorRepertoire (v5.4 Phase A)
// -----------------------------------------------------------------------------
//
// Reward-triggered episodic chunk proposal.  EpisodicCapture maintains a
// rolling buffer of last N (slow_consensus_embedding, intent_index) tuples
// at the slow consensus EPM's keyframe rate.  When a configured reward
// event fires, snapshots the buffer, splits into entry context
// (first entry_keyframes embeddings) + intent body (remaining intent
// indices), and publishes this proposal to MotorRepertoire which inserts
// it as a new chunk.

struct EpisodicChunkProposal : Message {
    std::vector<Eigen::VectorXf> entry_embeddings;   // K vectors (default K=2)
    std::vector<int>             intent_sequence;    // total = body_keyframes × playback_ticks_per_position
    std::string                  trigger_event;      // "hit", "scent_aligned_with_green", etc.
    float                        intensity = 1.0f;   // event intensity at trigger
    // v5.4 Phase G (Proposal C) — segmentation hint so MotorRepertoire
    // sets up per-position credit buckets correctly.
    int                          body_keyframes              = 0;
    int                          playback_ticks_per_position = 1;
};

struct MotorPlayStream : Message {
    uint64_t          request_id = 0;
    int               chunk_id   = -1;
    std::vector<float> actions;          // tick-by-tick motor commands (legacy / pre-v5.3)
    // v5.3 Phase B — intent-sequence chunks: when non-empty, ActionDecoder
    // publishes IntentToken on intent.override per tick instead of ActionOut.
    // Exactly one of `actions` / `intents` is populated per stream.
    std::vector<int>  intents;
};

// -----------------------------------------------------------------------------
// Host-bridged raw sensor frames (input to JL/STFT encoders inside an EPM)
// -----------------------------------------------------------------------------
//
// The host (Godot Host, HAL Host, Debug/Replay Host) reads from physical
// sensors or recorded streams and bridges raw bytes onto the Bus.  The EPM
// configured with `encoder_kind = jl` consumes RawImageFrame; configured
// with `encoder_kind = stft` consumes RawAudioFrame.  The host chooses the
// topic name (typically `reality.video.<modality>` or `reality.audio.<modality>`).
//
// These are NOT produced by Ogma Core modules — they cross the boundary
// from physical I/O.  The host edges in the graph config are wired with
// `from = "host:<topic>"`.

struct RawImageFrame : Message {
    std::vector<uint8_t> pixels;        // row-major H × W × C, uint8 per channel
    int                  height   = 0;
    int                  width    = 0;
    int                  channels = 0;  // 1 (grayscale) or 3 (RGB)
};

struct RawAudioFrame : Message {
    std::vector<float>   samples;       // float PCM in [-1, 1]
    int                  n_samples = 0; // per channel
    int                  channels  = 0; // 1 (mono) or 2 (stereo)
};

// -----------------------------------------------------------------------------
// reality.proprio.<sensor>  --  Bundled proprioception (host-published)
// -----------------------------------------------------------------------------
//
// Hosts (Godot, HAL, Debug) bridge the body's interoceptive + exteroceptive
// proprioceptive channels onto these topics.  v3's MazeAdapter `body_state`
// (22-D vector with [pos, heading, whiskers, scent, hunger, pheromone]) maps
// to per-sensor topics here so NeurochemState and HomeostaticDrive can
// subscribe to exactly what they need.

struct ProprioToken : Message {
    Eigen::VectorXf  values;             // sensor-defined dimensionality
    std::string      sensor;             // e.g. "imu", "whisker", "scent"
};

// -----------------------------------------------------------------------------
// events.<name>  --  Discrete environment events (host-published)
// -----------------------------------------------------------------------------
//
// Sparse boolean / scalar events bridged in by the host: hit, miss, brick,
// wall_stuck, whisker_bump.  NeurochemState, HomeostaticDrive, and
// MotorRepertoire (for chunk-outcome tagging) subscribe.  Payload is intentionally
// minimal — events are timestamps with optional intensity.

struct EnvEvent : Message {
    std::string name;        // matches the trailing component of `events.<name>`
    float       intensity = 1.0f;
};

// -----------------------------------------------------------------------------
// reflex.gate.<name>  --  scalar gate / suppression signal between reflex modules
// -----------------------------------------------------------------------------
//
// Phase 6.6.D.8.  A producer reflex (e.g. ScentGateReflex when scent EMA is
// climbing) publishes a 0..1 suppression value on a configured topic; downstream
// reflexes (e.g. WhiskerAversionReflex) optionally subscribe and scale their
// output by (1 - value).  Replaces the body-side `_scent_suppress` block.
//
// `active` lets a producer indicate "I'm subscribed but my upstream signal
// hasn't warmed up yet — ignore me this tick" without flooding consumers
// with stale zeros.

struct ReflexGate : Message {
    float value  = 0.0f;
    bool  active = false;
};

// -----------------------------------------------------------------------------
// hormone.state  --  Slow-timescale hormonal broadcast (Phase 3 placeholder)
// -----------------------------------------------------------------------------
//
// Cortisol, estradiol, testosterone — multi-day setpoint modifiers that shift
// HomeostaticDrive setpoints or decay rates over timescales longer than a tick.
// Per v4_algorithmic_gaps.md §Primitive 4 (HomeostaticDrive): "future hormones
// live here as additional drive channels."  Phase 3 adds the topic schema and
// the topic-name constant; implementation (a new HomeostaticDrive channel kind
// "hormone_ema" or a dedicated HormoneState module) is Phase-3.5+.
//
// Any module that needs slow-timescale modulation subscribes to "hormone.state".

struct HormoneState : Message {
    std::unordered_map<std::string, float> levels;   // hormone name → level [0, 1]
};

// -----------------------------------------------------------------------------
// fitness.score  --  Per-OgmaInstance fitness scalar (Phase 6)
// -----------------------------------------------------------------------------

struct FitnessScore : Message {
    float cumulative_drive_reduction = 0.0f;
    float window_drive_reduction     = 0.0f;   // last N ticks
    int   ticks_alive                = 0;
};

// -----------------------------------------------------------------------------
// gain.<consumer_id>  --  PART IV GainEvolver → consumer evolved-gain vector
// -----------------------------------------------------------------------------
//
// keys/values are PARALLEL and travel together so the key→value mapping is
// explicit in every message — the consumer verifies each key against its own
// param schema by read-back, never by config-side agreement.  `generation` and
// `is_candidate` exist so the consumer's telemetry can attribute which window
// (incumbent or candidate) a landed vector belongs to.

struct GainVector : Message {
    std::vector<std::string> keys;
    std::vector<double>      values;
    uint64_t generation   = 0;
    bool     is_candidate = false;   // false = incumbent window
};

// -----------------------------------------------------------------------------
// rhythm.bias.<premotor_id>  --  Phase 7.9 SynergyTimer pre-softmax bias
// -----------------------------------------------------------------------------
//
// Per-Premotor logit bias keyed by per-leg gait phase + reward-gated
// learned synergy table.  Published each tick by SynergyTimer for every
// Premotor that has a leg assignment.  Premotor subscribes via the
// rhythm_bias_topic ConstructionOnly param and adds the bias vector
// to its softmax scores pre-decision.  Composes additively with
// epistemic_gain and value_head_gain.
//
// `bias` length must equal Premotor's n_intents.  Already pre-multiplied
// by rhythm_bias_gain × rhythm_confidence[leg] — Premotor doesn't need
// to scale.  Confidence-zero means an all-zero vector → byte-identical
// legacy behaviour.

struct RhythmBiasToken : Message {
    Eigen::VectorXf bias;          // pre-softmax logit bias per intent
    float           confidence = 0.0f;     // [0, 1] — for telemetry only
    int             phase_bin  = -1;       // for telemetry only
};

// -----------------------------------------------------------------------------
// Topic name constants  --  the canonical strings used in JSON config and code.
// -----------------------------------------------------------------------------
//
// Wildcard subscriptions use the trailing-dot prefix-match form:
//   "reality."        matches reality.*
//   "reality.video."  matches reality.video.*
// (See Bus.hpp for matching rules.)  Singleton topics have no wildcards.

namespace topics {
constexpr const char* kRealityPrefix       = "reality.";
constexpr const char* kRealityVideoPrefix  = "reality.video.";
constexpr const char* kRealityAudioPrefix  = "reality.audio.";
constexpr const char* kRealityProprioPrefix = "reality.proprio.";
constexpr const char* kRhythmBiasPrefix     = "rhythm.bias.";
constexpr const char* kConsensusPrefix     = "consensus.";
constexpr const char* kPredictionPrefix    = "prediction.";
constexpr const char* kSequenceMotifPrefix = "sequence.motif.";
constexpr const char* kEventsPrefix        = "events.";

constexpr const char* kNeuroState     = "neuro.state";
constexpr const char* kDriveErrors    = "drive.errors";
constexpr const char* kActionOut      = "action.out";
// Bilateral motor channels (Phase 6.6.D.6).  Same ActionOut payload; bodies
// with a pair of actuators (paddler L+R, future quadruped legs, octopus arms)
// subscribe to both and apply differentially.  When ONLY action.out is
// published, bodies fall back to symmetric application — preserves the
// pre-bilateral configs and the legacy ActionDecoder/ActionGate pathway.
constexpr const char* kActionLeft     = "action.left";
constexpr const char* kActionRight    = "action.right";
constexpr const char* kPolicyIntent   = "policy.intent";
constexpr const char* kIntentOverride = "intent.override";   // v5.3 Phase B
constexpr const char* kMotorChunks    = "motor.chunks";
constexpr const char* kMotorPlayCmd   = "motor.play.cmd";
constexpr const char* kMotorEpisodicProposal = "motor.episodic_proposal";   // v5.4 Phase A
constexpr const char* kMotorPlayResp  = "motor.play.stream";
constexpr const char* kRolloutQuery   = "rollout.query";
constexpr const char* kRolloutResult  = "rollout.result";
constexpr const char* kFitnessScore   = "fitness.score";
constexpr const char* kHormoneState   = "hormone.state";
constexpr const char* kExplorationDirective = "exploration.directive";
constexpr const char* kMotorFaderAlpha       = "motor.fader.alpha";
} // namespace topics

} // namespace ogma
