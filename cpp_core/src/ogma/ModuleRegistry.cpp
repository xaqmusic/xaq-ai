#include "ogma/Module.hpp"

#include <stdexcept>

#include "ogma/modules/ActionDecoder.hpp"
#include "ogma/modules/ActionGate.hpp"
#include "ogma/modules/AdaptiveThresholdTracker.hpp"
#include "ogma/modules/CellReflex.hpp"
#include "ogma/modules/ChemotaxisAI.hpp"
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
#include "ogma/modules/RunTumbleNav.hpp"
#include "ogma/modules/RunTumbleNavV2.hpp"
#include "ogma/modules/VisualHomingNav.hpp"
#include "ogma/modules/PlaceNav.hpp"
#include "ogma/modules/EFEArbiter.hpp"
#include "ogma/modules/GradientEPM.hpp"
#include "ogma/modules/Klinotaxis.hpp"
#include "ogma/modules/DescendingPredictor.hpp"
#include "ogma/modules/DualEMADetector.hpp"
#include "ogma/modules/EmbeddingRegistry.hpp"
#include "ogma/modules/EventConjunction.hpp"
#include "ogma/modules/ChunkAbortGate.hpp"
#include "ogma/modules/ChunkOutcomeGate.hpp"
#include "ogma/modules/EpisodicCapture.hpp"
#include "ogma/modules/EPM.hpp"
#include "ogma/modules/FaderController.hpp"
#include "ogma/modules/ForwardDriveReflex.hpp"
#include "ogma/modules/GainEvolver.hpp"
#include "ogma/modules/GaitSelector.hpp"
#include "ogma/modules/GNGRollout.hpp"
#include "ogma/modules/HomeokineticExploration.hpp"
#include "ogma/modules/HomeostaticDrive.hpp"
#include "ogma/modules/JointSensorimotorBridge.hpp"
#include "ogma/modules/PolicyChannelAggregator.hpp"
#include "ogma/modules/CPGOscillator.hpp"
#include "ogma/modules/MotorPlanner.hpp"
#include "ogma/modules/MotorEPM.hpp"
#include "ogma/modules/MotorEPMv2.hpp"
#include "ogma/modules/PosturalPrior.hpp"
#include "ogma/modules/KeyframeGait.hpp"
#include "ogma/modules/BodyRhythmTracker.hpp"
#include "ogma/modules/KeyframeAverager.hpp"
#include "ogma/modules/KeyframePeakDetector.hpp"
#include "ogma/modules/LateralVoter.hpp"
#include "ogma/modules/MotorBus.hpp"
#include "ogma/modules/MotorFader.hpp"
#include "ogma/modules/MotorRepertoire.hpp"
#include "ogma/modules/NeurochemState.hpp"
#include "ogma/modules/Premotor.hpp"
#include "ogma/modules/PremotorAI.hpp"
#include "ogma/modules/ScentCompass.hpp"
#include "ogma/modules/ScentGateReflex.hpp"
#include "ogma/modules/SensorBundle.hpp"
#include "ogma/modules/SequenceGNG.hpp"
#include "ogma/modules/CruseCoordinator.hpp"
#include "ogma/modules/DistressDrive.hpp"
#include "ogma/modules/StaleConfidenceDecay.hpp"
#include "ogma/modules/StuckEscapeReflex.hpp"
#include "ogma/modules/SynergyTimer.hpp"
#include "ogma/modules/WhiskerAversionReflex.hpp"
#include "ogma/modules/WhiskerSteerReflex.hpp"

namespace ogma {

ModuleRegistry& ModuleRegistry::instance() {
    static ModuleRegistry reg;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        // Register every Phase-1 primitive's factory keyed by its type_name().
        // Order is irrelevant — the registry is by string.
        reg.register_type("NeurochemState",      [](){ return std::make_unique<NeurochemState>(); });
        reg.register_type("EPM",                 [](){ return std::make_unique<EPM>(); });
        reg.register_type("MotorPlanner",        [](){ return std::make_unique<MotorPlanner>(); });
        reg.register_type("LateralVoter",        [](){ return std::make_unique<LateralVoter>(); });
        reg.register_type("HomeostaticDrive",    [](){ return std::make_unique<HomeostaticDrive>(); });
        reg.register_type("ActionDecoder",       [](){ return std::make_unique<ActionDecoder>(); });
        reg.register_type("DescendingPredictor", [](){ return std::make_unique<DescendingPredictor>(); });
        reg.register_type("SequenceGNG",         [](){ return std::make_unique<SequenceGNG>(); });
        reg.register_type("GNGRollout",          [](){ return std::make_unique<GNGRollout>(); });
        reg.register_type("MotorRepertoire",     [](){ return std::make_unique<MotorRepertoire>(); });
        reg.register_type("HomeokineticExploration",
                                                 [](){ return std::make_unique<HomeokineticExploration>(); });
        reg.register_type("Premotor",            [](){ return std::make_unique<Premotor>(); });
        // Premotor active-inference upgrade — coexists with Premotor under
        // backward-compat directive (no edits to existing modules).  All
        // EFE gains default 0 → bit-identical to Premotor on identity-check
        // configs; new behaviour only fires when the new params are set.
        reg.register_type("PremotorAI",          [](){ return std::make_unique<PremotorAI>(); });
        reg.register_type("ActionGate",          [](){ return std::make_unique<ActionGate>(); });
        // Phase 6.6.D — reflex-as-module primitives
        reg.register_type("WhiskerAversionReflex",
                                                 [](){ return std::make_unique<WhiskerAversionReflex>(); });
        reg.register_type("DualEMADetector",     [](){ return std::make_unique<DualEMADetector>(); });
        reg.register_type("AdaptiveThresholdTracker",
                                                 [](){ return std::make_unique<AdaptiveThresholdTracker>(); });
        reg.register_type("StuckEscapeReflex",   [](){ return std::make_unique<StuckEscapeReflex>(); });
        reg.register_type("WhiskerSteerReflex",  [](){ return std::make_unique<WhiskerSteerReflex>(); });
        reg.register_type("ScentGateReflex",     [](){ return std::make_unique<ScentGateReflex>(); });
        reg.register_type("ForwardDriveReflex",  [](){ return std::make_unique<ForwardDriveReflex>(); });
        // Phase 6.6.E — shared embedding cache for predicted_pathway lookups.
        reg.register_type("EmbeddingRegistry",   [](){ return std::make_unique<EmbeddingRegistry>(); });
        // Phase 6.6.F — brain↔reflex crossfade primitive.
        reg.register_type("MotorFader",          [](){ return std::make_unique<MotorFader>(); });
        // 2026-06-19 — multichannel mixer → bus-compressor for the motor
        // influencer population (per-source faders, tanh masking/release).
        reg.register_type("MotorBus",            [](){ return std::make_unique<MotorBus>(); });
        // Phase 6.6.G — α-computation primitive (split out of MotorFader).
        reg.register_type("FaderController",     [](){ return std::make_unique<FaderController>(); });
        // Phase 6.6.G — consolidated Cell-environment reflex state machine.
        reg.register_type("CellReflex",          [](){ return std::make_unique<CellReflex>(); });
        reg.register_type("ChemotaxisAI",        [](){ return std::make_unique<ChemotaxisAI>(); });
        reg.register_type("HeadingController",    [](){ return std::make_unique<HeadingController>(); });
        reg.register_type("GoalBelief",           [](){ return std::make_unique<GoalBelief>(); });
        reg.register_type("HeadingPlanner",       [](){ return std::make_unique<HeadingPlanner>(); });
        reg.register_type("HeadingProbe",         [](){ return std::make_unique<HeadingProbe>(); });
        reg.register_type("MotivationGate",       [](){ return std::make_unique<MotivationGate>(); });
        reg.register_type("BearingEstimator",     [](){ return std::make_unique<BearingEstimator>(); });
        reg.register_type("VisualBearing",        [](){ return std::make_unique<VisualBearing>(); });
        reg.register_type("BearingFusion",        [](){ return std::make_unique<BearingFusion>(); });
        reg.register_type("ScentHomingLearner",   [](){ return std::make_unique<ScentHomingLearner>(); });
        reg.register_type("SaccadeReflex",         [](){ return std::make_unique<SaccadeReflex>(); });
        reg.register_type("CylinderBuilder",       [](){ return std::make_unique<CylinderBuilder>(); });
        // 2026-06 — passive place-recorder (replaces saccade+cylinder mapping):
        // every record_every ticks publishes a column = view-feature + heading + IMU.
        reg.register_type("ColumnBuilder",         [](){ return std::make_unique<ColumnBuilder>(); });
        reg.register_type("PlaceGraphPlanner",     [](){ return std::make_unique<PlaceGraphPlanner>(); });
        // 2026-07-03 — Cell task #33: the third policy. PlayLoop = "PlaceGraphPlanner
        // minus traverse" — GROWS the shared place-map by ascending novelty→frontier
        // (run-and-tumble beyond the mapped graph) instead of routing to remembered food.
        reg.register_type("PlayLoop",              [](){ return std::make_unique<PlayLoop>(); });
        reg.register_type("RunTumbleNav",          [](){ return std::make_unique<RunTumbleNav>(); });
        reg.register_type("RunTumbleNavV2",        [](){ return std::make_unique<RunTumbleNavV2>(); });
        reg.register_type("VisualHomingNav",       [](){ return std::make_unique<VisualHomingNav>(); });
        reg.register_type("PlaceNav",              [](){ return std::make_unique<PlaceNav>(); });
        // 2026-06-29 — Cell L2 EFE arbiter: active-inference policy selection over the
        // two competent nav loops (klino CLOSER vs planner SEARCHER). Whitened EFE-value
        // race + winner-take-all gain (1/0) into the MotorBus → mutes the loser's channel
        // (mix AND authority → its advance learning pauses).
        reg.register_type("EFEArbiter",            [](){ return std::make_unique<EFEArbiter>(); });
        reg.register_type("GradientEPM",           [](){ return std::make_unique<GradientEPM>(); });
        reg.register_type("Klinotaxis",            [](){ return std::make_unique<Klinotaxis>(); });
        reg.register_type("ScentCompass",        [](){ return std::make_unique<ScentCompass>(); });
        reg.register_type("SensorBundle",        [](){ return std::make_unique<SensorBundle>(); });
        // Phase v5.2 — sensor / motor downsampler for multi-rate brain.
        reg.register_type("KeyframeAverager",    [](){ return std::make_unique<KeyframeAverager>(); });
        // 2026-06-05 Phase H1 V7 — peak-deviation companion to KeyframeAverager.
        // Same window+input contract but publishes (max - mean) per dim, surfacing
        // transients the averager smooths away (Joseph's "purple-mountain-swings-past"
        // bursts). LGMD ON-cell / predictive-error neuron analog.
        reg.register_type("KeyframePeakDetector",[](){ return std::make_unique<KeyframePeakDetector>(); });
        // Phase 6.9 Stage A — "boredom of being stuck" distress combiner.
        // Reads the slow meta-EPM stack (TLE onset + staleness + model-free
        // pooled-state freeze), suppressed by rising dopamine, → cognition.boredom.
        reg.register_type("DistressDrive",       [](){ return std::make_unique<DistressDrive>(); });
        // Phase v5.3.C — N-event AND primitive for compound reward gating.
        reg.register_type("EventConjunction",    [](){ return std::make_unique<EventConjunction>(); });
        // Phase v5.3.D — surprise-spike chunk-abort gate.
        reg.register_type("ChunkAbortGate",      [](){ return std::make_unique<ChunkAbortGate>(); });
        // Phase v5.3.F — outcome-driven chunk-abort gate (counterpart to D).
        reg.register_type("ChunkOutcomeGate",    [](){ return std::make_unique<ChunkOutcomeGate>(); });
        // Phase v5.4.A — reward-triggered episodic chunk crystallisation.
        reg.register_type("EpisodicCapture",     [](){ return std::make_unique<EpisodicCapture>(); });
        // v6.0.e — Playful Machine principle #4: predictive model degradation.
        reg.register_type("StaleConfidenceDecay",[](){ return std::make_unique<StaleConfidenceDecay>(); });
        // Phase 7.2-EPM — joins per-joint ActionOut with bundled proprio
        // for sensorimotor per-joint EPM consumption.
        reg.register_type("JointSensorimotorBridge",
                                                 [](){ return std::make_unique<JointSensorimotorBridge>(); });
        // Phase 7.2-EPM — N-channel PolicyToken packer for multi-joint chunks.
        reg.register_type("PolicyChannelAggregator",
                                                 [](){ return std::make_unique<PolicyChannelAggregator>(); });
        // Phase 7.x — biology-inspired spinal CPG: rhythmic accel bias
        // summed onto Premotor brain commands per joint.
        reg.register_type("CPGOscillator",       [](){ return std::make_unique<CPGOscillator>(); });
        // 2026-06-12 — Motor-EPM: homeokinetic per-leg sensorimotor controller
        // (forward self-model + controller, learns by descending the motor TLE;
        // no reward).  docs/plans-and-designs/motor_epm_homeokinetic_plan.md
        reg.register_type("MotorEPM",            [](){ return std::make_unique<MotorEPM>(); });
        // STAGE 0: a mechanical copy of MotorEPM.  Must stay byte-identical to it until
        // moduledif.py passes; see docs/plans-and-designs/motor_epm_v2_plan.md.
        reg.register_type("MotorEPMv2",          [](){ return std::make_unique<MotorEPMv2>(); });
        reg.register_type("PosturalPrior",       [](){ return std::make_unique<PosturalPrior>(); });
        reg.register_type("KeyframeGait",        [](){ return std::make_unique<KeyframeGait>(); });
        reg.register_type("BodyRhythmTracker",   [](){ return std::make_unique<BodyRhythmTracker>(); });
        // Phase 7.9 — closed-loop adaptive timer.  Per-leg touchdown
        // detection + period EMA + prediction.  Reward-gated Hebbian on
        // per-(premotor, phase_bin, intent) bias table.  Output is
        // pre-softmax logit bias for the Premotors of each leg.
        reg.register_type("SynergyTimer",        [](){ return std::make_unique<SynergyTimer>(); });
        // Phase 7.11 — Cruse-rules inter-leg coordinator.  Rule 1: rear-leg
        // Premotors get pre-softmax bias toward stance when their anatomical
        // anterior is in swing.  Adaptive magnitude via per-Premotor
        // violation_ema — fades silently when brain is naturally compliant.
        reg.register_type("CruseCoordinator",    [](){ return std::make_unique<CruseCoordinator>(); });
        // Phase 8 A2 — temporal gait-option selector.  Graded REINFORCE over a
        // library of multi-tick posture sequences (gaits); credits each gait by
        // the return accrued over its execution window.  The action-vocabulary
        // fix for A1's flat-landscape failure-to-converge.
        reg.register_type("GaitSelector",        [](){ return std::make_unique<GaitSelector>(); });
        // PART IV (2026-08-17) — lifetime (1+1)-ES over a declared consumer gain
        // vector (MotorEPMv2's high-value sliders), intrinsic viability+flow
        // criterion, interleaved incumbent re-evaluation.  mutation_sigma 0 =
        // silent observer = byte-identical.  Charter:
        // docs/plans-and-designs/adaptive_gains_substrate_plan.md
        reg.register_type("GainEvolver",         [](){ return std::make_unique<GainEvolver>(); });
    }
    return reg;
}

void ModuleRegistry::register_type(std::string_view type_name, Factory factory) {
    factories_[std::string(type_name)] = std::move(factory);
}

ModulePtr ModuleRegistry::create(std::string_view type_name) const {
    auto it = factories_.find(std::string(type_name));
    if (it == factories_.end())
        throw std::invalid_argument("ModuleRegistry: unknown type '"
                                     + std::string(type_name) + "'");
    return it->second();
}

std::vector<std::string> ModuleRegistry::registered_types() const {
    std::vector<std::string> out;
    out.reserve(factories_.size());
    for (auto const& [name, _] : factories_) out.push_back(name);
    return out;
}

} // namespace ogma
