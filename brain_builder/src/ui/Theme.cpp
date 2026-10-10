#include "ui/Theme.hpp"

#include <unordered_map>

namespace bb {

namespace {
ImU32 rgb(float r, float g, float b) { return IM_COL32(int(r * 255), int(g * 255), int(b * 255), 255); }
}

ImU32 payload_color(std::string const& payload) {
    static const std::unordered_map<std::string, ImU32> table = {
        {"RealityToken",          rgb(0.55f, 0.20f, 0.70f)},
        {"ConsensusToken",        rgb(0.20f, 0.65f, 0.60f)},
        {"NeuroState",            rgb(0.22f, 0.45f, 0.90f)},
        {"DriveErrors",           rgb(0.90f, 0.55f, 0.10f)},
        {"ActionOut",             rgb(0.85f, 0.20f, 0.45f)},
        {"FaderState",            rgb(0.95f, 0.75f, 0.20f)},
        {"PolicyToken",           rgb(0.85f, 0.40f, 0.55f)},
        {"IntentToken",           rgb(0.80f, 0.45f, 0.35f)},
        {"EpisodicChunkProposal", rgb(0.50f, 0.35f, 0.70f)},
        {"PredictionToken",       rgb(0.50f, 0.55f, 0.20f)},
        {"SequenceMotif",         rgb(0.30f, 0.55f, 0.30f)},
        {"ExplorationDirective",  rgb(0.55f, 0.55f, 0.60f)},
        {"MotorChunk",            rgb(0.45f, 0.30f, 0.65f)},
        {"MotorChunks",           rgb(0.45f, 0.30f, 0.65f)},
        {"MotorPlayCmd",          rgb(0.55f, 0.35f, 0.55f)},
        {"MotorPlayStream",       rgb(0.65f, 0.40f, 0.55f)},
        {"RolloutQuery",          rgb(0.40f, 0.50f, 0.60f)},
        {"RolloutResult",         rgb(0.50f, 0.60f, 0.65f)},
        {"RawImageFrame",         rgb(0.30f, 0.30f, 0.55f)},
        {"RawAudioFrame",         rgb(0.55f, 0.30f, 0.30f)},
        {"ProprioToken",          rgb(0.15f, 0.55f, 0.65f)},
        {"EnvEvent",              rgb(0.15f, 0.70f, 0.30f)},
        {"ReflexGate",            rgb(0.65f, 0.50f, 0.20f)},
        {"HormoneState",          rgb(0.40f, 0.30f, 0.55f)},
        {"FitnessScore",          rgb(0.70f, 0.65f, 0.35f)},
        {"GainVector",            rgb(0.75f, 0.60f, 0.25f)},
        {"AdaptiveThreshold",     rgb(0.55f, 0.60f, 0.55f)},
    };
    auto it = table.find(payload);
    return it == table.end() ? rgb(0.50f, 0.50f, 0.50f) : it->second;
}

ImU32 category_color(std::string const& category) {
    static const std::unordered_map<std::string, ImU32> table = {
        {"sensory",    rgb(0.40f, 0.18f, 0.52f)},
        {"fusion",     rgb(0.14f, 0.45f, 0.42f)},
        {"neurochem",  rgb(0.16f, 0.32f, 0.62f)},
        {"drive",      rgb(0.62f, 0.38f, 0.08f)},
        {"predictor",  rgb(0.36f, 0.40f, 0.14f)},
        {"nav",        rgb(0.20f, 0.42f, 0.55f)},
        {"reflex",     rgb(0.50f, 0.36f, 0.14f)},
        {"motor",      rgb(0.58f, 0.14f, 0.32f)},
        {"meta",       rgb(0.38f, 0.38f, 0.42f)},
        {"instrument", rgb(0.30f, 0.44f, 0.30f)},
    };
    auto it = table.find(category);
    return it == table.end() ? rgb(0.35f, 0.35f, 0.35f) : it->second;
}

ImU32 topic_color(std::string const& t) {
    auto starts = [&](char const* p) { return t.rfind(p, 0) == 0; };
    if (starts("prediction.")) return rgb(0.50f, 0.55f, 0.20f);
    if (starts("consensus."))  return rgb(0.20f, 0.65f, 0.60f);
    if (starts("neuro.") || starts("hormone.")) return rgb(0.22f, 0.45f, 0.90f);
    if (starts("action.") || starts("motor."))  return rgb(0.85f, 0.20f, 0.45f);
    if (starts("events."))     return rgb(0.15f, 0.70f, 0.30f);
    if (starts("reality."))    return rgb(0.55f, 0.20f, 0.70f);
    return rgb(0.60f, 0.60f, 0.60f);
}

void apply_theme(float scale) {
    ImGui::StyleColorsDark();
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 4.0f;
    s.FrameRounding  = 3.0f;
    s.GrabRounding   = 3.0f;
    s.TabRounding    = 3.0f;
    s.ScaleAllSizes(scale);
}

} // namespace bb
