// engine.hpp — the running instrument: what the brain said, what it sounds like.
//
// Two threads meet here and the split is deliberate.  The NETWORK thread calls
// on_frame() once per diag frame (tens of Hz): it files the numbers, evaluates every
// route, and leaves a small set of target values behind.  The AUDIO thread calls
// render() (thousands of Hz): it reads those targets, glides toward them, and makes
// sound.  Nothing expensive happens between a sample and the next.
//
// The registry is the other half of the idea.  Sources are discovered from FRAMES, never
// from a compiled-in list — the tool cannot know what a given brain config runs, and a
// hardcoded key list would quietly mis-describe any brain but the one it was written for.
#pragma once

#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "dsp.hpp"
#include "patch.hpp"

namespace xv {

// ---------------------------------------------------------------------------- stats
// Median + MAD over a sliding window.  Median rather than mean, MAD rather than standard
// deviation, because these signals are spiky by nature: one mitosis event would drag a
// mean for a full window and flatten everything after it.
struct RunningStats {
    std::deque<float> win;
    size_t            cap;
    explicit RunningStats(size_t cap_ = 500) : cap(cap_) {}
    void  push(float v) { win.push_back(v); while (win.size() > cap) win.pop_front(); }
    bool  ready() const { return win.size() >= 25; }
    float median() const;
    float mad() const;
};

struct SourceState {
    bool     is_bool  = false;
    double   last     = 0.0;
    double   prev     = 0.0;
    bool     have_prev = false;
    double   delta    = 0.0;         // per second
    double   lo       = 0.0, hi = 0.0;   // running window extremes
    bool     have_ext = false;
    uint64_t seen     = 0;
    double   mean     = 0.0, m2 = 0.0;   // Welford, for auto_patch's "what moves most"
    RunningStats stats{500};             // over the value
    RunningStats dstats{500};            // over the per-second rate of change

    double variance() const { return seen > 1 ? m2 / double(seen - 1) : 0.0; }
};

// ---------------------------------------------------------------------------- registry
class SourceRegistry {
  public:
    // Flattens a snapshot to dotted paths of numeric/bool leaves and files each one.
    // One level of nesting is expected (a module's per-source map); deeper is ignored
    // rather than exploded, because that would be an unbounded container and diag_lite()
    // promises not to send one.
    void observe(const std::string& module, const std::string& type,
                 const nlohmann::json& snapshot, double dt_s);

    void note_module(const std::string& module, const std::string& type);   // even if empty

    const SourceState* find(const SourceRef& s) const;
    SourceState*       find(const SourceRef& s);

    std::vector<ObservedModule> observed() const;
    nlohmann::json              to_json() const;      // the studio's source tree
    size_t                      module_count() const { return mods_.size(); }

  private:
    struct Mod {
        std::string                        type;
        uint64_t                           frames = 0;
        std::map<std::string, SourceState> keys;
    };
    std::map<std::string, Mod> mods_;
};

// ---------------------------------------------------------------------------- runtime
struct RouteRT {
    double smoothed  = 0.0;
    bool   have      = false;
    double last_norm = 0.0;      // what the studio's per-route meter shows
    double last_out  = 0.0;
};

struct EventRT {
    double prev    = 0.0;
    bool   have    = false;
};

struct VoiceRT {
    Oscillator oscil;
    FilterUnit filt;
    double     hz_now = 0.0, amp_now = 0.0;
    float      note_semis = 0.f;      // quantised note, with hysteresis
    double     chirp_t = -1.0;
    int        chirp_kind = 0;        // index into the event-sound table

    // Targets written once per diag frame, consumed per sample.
    double t_hz = 261.63, t_amp = 0.0, t_level = 1.0;
    double t_cutoff = 4000.0, t_q = 0.7, t_pw = 0.5, t_noise = 0.0, t_morph = 0.0, t_pan = 0.0;

    std::vector<RouteRT> routes;
    std::vector<EventRT> events;
};

// ---------------------------------------------------------------------------- engine
class Engine {
  public:
    explicit Engine(int sample_rate) : sr_(sample_rate) {}

    // -- patch --
    void  set_patch(const Patch& p);
    Patch patch() const;
    bool  apply_ops_locked(const nlohmann::json& ops, std::string& err);

    // -- brain frames (network thread) --
    void on_frame(const std::string& module, const std::string& type,
                  const nlohmann::json& snapshot, double dt_s);

    // Register a module before any frame arrives, so a module that never publishes a
    // source is still visible as a module that said nothing — which is the report the
    // operator needs, rather than an absence they have to notice.
    void note_module_seen(const std::string& module, const std::string& type) {
        std::lock_guard<std::mutex> lk(mtx_);
        reg_.note_module(module, type);
    }

    // -- audio (audio thread).  Mono int16 into `out`. --
    void render(int16_t* out, int n);

    // -- studio --
    nlohmann::json sources_json() const;
    nlohmann::json state_json() const;              // meters
    std::vector<ObservedModule> observed() const;

    void set_master_volume(double v) { std::lock_guard<std::mutex> lk(mtx_); patch_.master.volume = v; }
    void set_mute(bool m)            { mute_ = m; }
    bool muted() const               { return mute_; }
    void set_tone_enabled(bool e)    { tone_ = e; }
    bool tone_enabled() const        { return tone_; }

    mutable std::mutex mtx_;      // guards patch_, voices_, registry_

  private:
    void resize_runtime_locked();
    // Evaluates one route against the registry and returns its contribution in the
    // destination's own units.
    double eval_route_locked(const Route& r, RouteRT& rt, double dt_s);
    void   eval_events_locked(const VoiceCfg& cfg, VoiceRT& rt);
    double normalise_locked(const Route& r, const SourceState& s, const SourceRef& ref);

    int     sr_;
    Patch   patch_;
    std::vector<VoiceRT>     voices_;
    std::vector<std::string> prev_ids_;      // voice ids matching voices_, for re-matching
    std::vector<RouteRT>     master_routes_;
    SourceRegistry           reg_;

    // master bus runtime
    FilterUnit master_filt_, master_filt_r_;   // one per channel: filters carry state
    double     m_cutoff = 4000.0, m_q = 0.7, m_morph = 0.0, m_level = 1.0;
    double     master_peak_ = 0.0;      // for the studio's level meter
    std::vector<float> mixL_, mixR_;

    std::atomic<bool> mute_{false};
    std::atomic<bool> tone_{true};
};

}  // namespace xv
