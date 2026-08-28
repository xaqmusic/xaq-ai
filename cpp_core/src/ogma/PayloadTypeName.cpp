#include "ogma/PayloadTypeName.hpp"

#include "ogma/Topics.hpp"
#include "ogma/modules/AdaptiveThresholdTracker.hpp"

namespace ogma {

std::string payload_type_name(std::type_index t) {
    if (t == std::type_index(typeid(RealityToken)))         return "RealityToken";
    if (t == std::type_index(typeid(ConsensusToken)))       return "ConsensusToken";
    if (t == std::type_index(typeid(NeuroState)))           return "NeuroState";
    if (t == std::type_index(typeid(DriveErrors)))          return "DriveErrors";
    if (t == std::type_index(typeid(ActionOut)))            return "ActionOut";
    if (t == std::type_index(typeid(FaderState)))           return "FaderState";
    if (t == std::type_index(typeid(PolicyToken)))          return "PolicyToken";
    if (t == std::type_index(typeid(IntentToken)))          return "IntentToken";
    if (t == std::type_index(typeid(EpisodicChunkProposal))) return "EpisodicChunkProposal";
    if (t == std::type_index(typeid(PredictionToken)))      return "PredictionToken";
    if (t == std::type_index(typeid(SequenceMotif)))        return "SequenceMotif";
    if (t == std::type_index(typeid(ExplorationDirective))) return "ExplorationDirective";
    if (t == std::type_index(typeid(MotorChunk)))           return "MotorChunk";
    if (t == std::type_index(typeid(MotorChunks)))          return "MotorChunks";
    if (t == std::type_index(typeid(MotorPlayCmd)))         return "MotorPlayCmd";
    if (t == std::type_index(typeid(MotorPlayStream)))      return "MotorPlayStream";
    if (t == std::type_index(typeid(RolloutQuery)))         return "RolloutQuery";
    if (t == std::type_index(typeid(RolloutResult)))        return "RolloutResult";
    if (t == std::type_index(typeid(RawImageFrame)))        return "RawImageFrame";
    if (t == std::type_index(typeid(RawAudioFrame)))        return "RawAudioFrame";
    if (t == std::type_index(typeid(ProprioToken)))         return "ProprioToken";
    if (t == std::type_index(typeid(EnvEvent)))             return "EnvEvent";
    if (t == std::type_index(typeid(ReflexGate)))           return "ReflexGate";
    if (t == std::type_index(typeid(HormoneState)))         return "HormoneState";
    if (t == std::type_index(typeid(FitnessScore)))         return "FitnessScore";
    if (t == std::type_index(typeid(GainVector)))           return "GainVector";
    if (t == std::type_index(typeid(AdaptiveThreshold)))    return "AdaptiveThreshold";
    return "Unknown";
}

} // namespace ogma
