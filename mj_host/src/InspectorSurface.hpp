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
#include <vector>

namespace ogma { class OgmaInstance; class DiagPublisher; class LiveGraph; }
namespace ami_ogma { namespace control { class ControlServer; } }

namespace mjhost {

class InspectorSurface {
public:
    // `mtx` is the lock the owner takes around every tick of `instance`; verbs take it too.
    // source_path: the config file the host loaded, reported to clients that
    // pull the live graph (the brain builder opens it for metadata and layout).
    // port_offset / role (2026-10-02): the duck runs up to three brains -- the walker / intent brain (offset 0, the
    // historical 7400/7401), the head brain (+2: 7402/7403) and the stand brain (+4: 7404/7405) -- and each serves its
    // own surface, so the inspector reaches any of them by pointing its host field at that port.  Before this every
    // brain asked for 7400 and the first one constructed won.
    InspectorSurface(ogma::OgmaInstance& instance, std::recursive_mutex& mtx,
                     std::string source_path = std::string(), int port_offset = 0, std::string role = std::string());
    ~InspectorSurface();
    void publish_tick(uint64_t tick_id);
    bool active() const { return active_; }
    // Every live change a client made to the brain since the last call (set_param, apply_patch), as
    // "patch:<module>.<key>" / "patch:graph".  The host prints them into the run's JSONL, so a watched
    // run's record shows what was changed under it (2026-09-17: a watched run and a measured one can
    // only be compared if every difference is on the record).
    std::vector<std::string> take_events();

private:
    ogma::OgmaInstance& instance_;
    std::recursive_mutex& mtx_;
    std::unique_ptr<ogma::DiagPublisher> diag_;
    std::unique_ptr<ogma::LiveGraph> live_;
    std::unique_ptr<ami_ogma::control::ControlServer> control_;
    bool active_ = false;
    std::vector<std::string> events_;      // guarded by mtx_
};

}  // namespace mjhost
