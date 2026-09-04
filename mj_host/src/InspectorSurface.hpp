#pragma once
// The inspector's two surfaces, served by the MuJoCo host exactly as the Godot host
// serves them, so tools/xaq_inspector (and xaq_voice) attach to a duck brain unchanged:
//   control  TCP  OGMA_INSPECTOR_PORT (default 7400), newline-delimited JSON verbs —
//            list_modules, module_snapshot, module_subscribe_diag, unsubscribe, set_param,
//            and the brain builder's get_graph / apply_patch / graph_version (ogma::LiveGraph)
//   diag     ZMQ PUB on control + 1, per-subscription topic prefix diag.<sub_id>.
// Best-effort: a port that is already taken (a battery of hosts in parallel) logs one
// line and the brain runs without an inspector.  OGMA_INSPECTOR_PORT=0 disables.
// Nothing here touches the brain's computation: the JSONL a run writes is identical
// with the surfaces on or off.
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace ogma { class OgmaInstance; class DiagPublisher; class LiveGraph; }
namespace ami_ogma { namespace control { class ControlServer; } }

namespace mjhost {

class InspectorSurface {
public:
    // `mtx` is the lock the owner takes around every tick of `instance`; verbs take it too.
    // source_path: the config file the host loaded, reported to clients that
    // pull the live graph (the brain builder opens it for metadata and layout).
    InspectorSurface(ogma::OgmaInstance& instance, std::recursive_mutex& mtx,
                     std::string source_path = std::string());
    ~InspectorSurface();
    void publish_tick(uint64_t tick_id);
    bool active() const { return active_; }

private:
    ogma::OgmaInstance& instance_;
    std::recursive_mutex& mtx_;
    std::unique_ptr<ogma::DiagPublisher> diag_;
    std::unique_ptr<ogma::LiveGraph> live_;
    std::unique_ptr<ami_ogma::control::ControlServer> control_;
    bool active_ = false;
};

}  // namespace mjhost
