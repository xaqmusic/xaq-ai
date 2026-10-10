// =============================================================================
// BrainTape.hpp  --  a recording of everything the host does to the brains, for an open-loop replay (2026-10-04)
// =============================================================================
//
// The resource push (design doc §17.110-17.111): a cost A/B of a brain variant needs the SAME inputs (a whole-run profile
// follows the run's own path), and the cost on ARM (the Pi 5, Pollen's Radxa Zero 3W) needs the brains without MuJoCo.
// The host touches a brain in four ways only -- it publishes sensor tokens, publishes reset events, changes module params
// (the learning switches), and restores a checkpoint -- and ticks it.  The tape is those calls, in call order, for every
// brain (0 intent, 1 head, 2 stop), opened by each brain's FINAL config (after the adapter's seeding), so a replay builds
// identical instances and drives them identically.  After each tick the brain's action outputs are recorded too, so the
// replay can say whether it reproduced the live run (fidelity) before its timings are trusted.
//
// STREAMING (the operator: big replay files have caused OOM before): the writer appends through a 1 MB stdio buffer and
// never holds the run; the reader returns one record at a time, so a replay's memory is the brains' plus one record (the
// largest is a checkpoint restore, a few MB).  ~2 kB a tick (the ToF cast is 392 floats): ~65 MB for 600 s.
//
// Format (little-endian): "OGMATAPE2\n", then records [u8 kind][u8 brain][payload]:
//   1 CONFIG  str(json)                              the GraphConfig the brain was built from
//   2 RESTORE str(json)                              instance->restore_state(json)
//   3 TOKEN   u64 tick, str topic, str sensor, str producer, u32 n, f32[n]     a ProprioToken published by the host
//   4 EVENT   u64 tick, str topic, str name, str producer, f32 intensity        an EnvEvent published by the host
//   5 PARAM   str module, str key, str(json value)   module->on_param_change(key, value)
//   6 TICK    u64 host tick                          instance->tick()
//   7 ACTIONS u32 n, n x (str topic, f32 accel), u64 digest    the action.* outputs after that tick, and a digest of EVERY
//             output the brain published (ProprioToken values, ActionOut accel, RealityToken winner + tle): the fidelity check.
//             The actions alone are blind in open loop -- the walker's inputs are all recorded host data, so a variant of
//             the cloud cannot change them; the digest sees the bearings, gains, skill requests and every EPM's token.
//   8 CALL    str module, str method, f64 arg        a host call that mutates a module directly (BearingSeekLoop::forget_target,
//             MotorEPMv2::rebabble): the replay dispatches it by name
// str = u32 length + bytes.  Absent (no --record-brains) = no hooks fire: byte-identical.
#pragma once

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "ogma/Module.hpp"
#include "ogma/OgmaInstance.hpp"

namespace mjhost {

enum class TapeKind : uint8_t { Config = 1, Restore = 2, Token = 3, Event = 4, Param = 5, Tick = 6, Actions = 7, Call = 8 };

class BrainTapeWriter {
public:
    explicit BrainTapeWriter(const std::string& path);
    ~BrainTapeWriter();
    bool ok() const { return f_ != nullptr; }
    void config(int brain, const std::string& json);
    void restore(int brain, const std::string& json);
    void token(int brain, uint64_t tick, const std::string& topic, const std::string& sensor, const std::string& producer,
               const float* v, uint32_t n);
    void event(int brain, uint64_t tick, const std::string& topic, const std::string& name, const std::string& producer, float intensity);
    void param(int brain, const std::string& module, const std::string& key, const ogma::ParamValue& value);
    void tick(int brain, uint64_t host_tick);
    void call(int brain, const std::string& module, const std::string& method, double arg);
    void actions(int brain, ogma::OgmaInstance& inst);   // every action.* topic's last ActionOut.accel
    uint64_t bytes() const { return bytes_; }
private:
    void head(TapeKind k, int brain);
    void u8(uint8_t v);
    void u32(uint32_t v);
    void u64(uint64_t v);
    void f32(float v);
    void str(const std::string& s);
    std::FILE* f_ = nullptr;
    std::vector<char> buf_;
    uint64_t bytes_ = 0;
};

// The process's tape, or nullptr (no recording).  The adapters call these; each is a no-op without a tape.
extern BrainTapeWriter* g_brain_tape;
void tape_param(int brain, ogma::Module* m, const std::string& key, const ogma::ParamValue& v);   // applies AND records

// One record at a time (the replay).
struct TapeRecord {
    TapeKind kind{};
    int brain = 0;
    uint64_t tick = 0;
    std::string a, b, c, json;          // CONFIG/RESTORE: json; TOKEN: a topic, b sensor, c producer; EVENT: a topic, b name,
                                        // c producer; PARAM: a module, b key, json value
    std::vector<float> values;          // TOKEN values; EVENT: [intensity]
    std::vector<std::pair<std::string, float>> actions;
    uint64_t digest = 0;
    double arg = 0.0;                   // CALL: a module, b method
};

// A digest of every output a brain's modules published (their last values): the replay's fidelity check.
uint64_t brain_digest(ogma::OgmaInstance& inst);

class BrainTapeReader {
public:
    explicit BrainTapeReader(const std::string& path);
    ~BrainTapeReader();
    bool ok() const { return f_ != nullptr && good_header_; }
    bool next(TapeRecord& r);           // false at the end (or a truncated tail, reported once)
    uint64_t records() const { return n_; }
private:
    bool rd(void* p, size_t n);
    bool u8(uint8_t& v);
    bool u32(uint32_t& v);
    bool u64(uint64_t& v);
    bool f32(float& v);
    bool str(std::string& s);
    std::FILE* f_ = nullptr;
    bool good_header_ = false;
    uint64_t n_ = 0;
};

}  // namespace mjhost
