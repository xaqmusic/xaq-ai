// BrainTape.cpp  --  see BrainTape.hpp
#include "BrainTape.hpp"

#include <cstring>

#include <nlohmann/json.hpp>

#include "ogma/Bus.hpp"
#include "ogma/GraphConfig.hpp"
#include "ogma/Topics.hpp"

namespace mjhost {

namespace {
constexpr char kMagic[] = "OGMATAPE2\n";
constexpr uint32_t kMaxStr = 256u << 20;     // a record larger than 256 MB is corruption, not a checkpoint
constexpr uint32_t kMaxFloats = 1u << 24;
}  // namespace

BrainTapeWriter* g_brain_tape = nullptr;

BrainTapeWriter::BrainTapeWriter(const std::string& path) {
    f_ = std::fopen(path.c_str(), "wb");
    if (!f_) return;
    buf_.resize(1u << 20);
    std::setvbuf(f_, buf_.data(), _IOFBF, buf_.size());
    std::fwrite(kMagic, 1, sizeof(kMagic) - 1, f_);
    bytes_ += sizeof(kMagic) - 1;
}
BrainTapeWriter::~BrainTapeWriter() { if (f_) std::fclose(f_); }

void BrainTapeWriter::u8(uint8_t v)   { std::fwrite(&v, 1, 1, f_); bytes_ += 1; }
void BrainTapeWriter::u32(uint32_t v) { std::fwrite(&v, 4, 1, f_); bytes_ += 4; }
void BrainTapeWriter::u64(uint64_t v) { std::fwrite(&v, 8, 1, f_); bytes_ += 8; }
void BrainTapeWriter::f32(float v)    { std::fwrite(&v, 4, 1, f_); bytes_ += 4; }
void BrainTapeWriter::str(const std::string& s) { u32(uint32_t(s.size())); std::fwrite(s.data(), 1, s.size(), f_); bytes_ += s.size(); }
void BrainTapeWriter::head(TapeKind k, int brain) { u8(uint8_t(k)); u8(uint8_t(brain)); }

void BrainTapeWriter::config(int brain, const std::string& json)  { if (!f_) return; head(TapeKind::Config, brain); str(json); }
void BrainTapeWriter::restore(int brain, const std::string& json) { if (!f_) return; head(TapeKind::Restore, brain); str(json); }
void BrainTapeWriter::token(int brain, uint64_t tick, const std::string& topic, const std::string& sensor,
                            const std::string& producer, const float* v, uint32_t n) {
    if (!f_) return;
    head(TapeKind::Token, brain); u64(tick); str(topic); str(sensor); str(producer); u32(n);
    std::fwrite(v, 4, n, f_); bytes_ += 4ull * n;
}
void BrainTapeWriter::event(int brain, uint64_t tick, const std::string& topic, const std::string& name,
                            const std::string& producer, float intensity) {
    if (!f_) return;
    head(TapeKind::Event, brain); u64(tick); str(topic); str(name); str(producer); f32(intensity);
}
void BrainTapeWriter::param(int brain, const std::string& module, const std::string& key, const ogma::ParamValue& value) {
    if (!f_) return;
    head(TapeKind::Param, brain); str(module); str(key); str(ogma::GraphConfig::param_to_json(value).dump());
}
void BrainTapeWriter::tick(int brain, uint64_t host_tick) { if (!f_) return; head(TapeKind::Tick, brain); u64(host_tick); }
void BrainTapeWriter::call(int brain, const std::string& module, const std::string& method, double arg) {
    if (!f_) return;
    head(TapeKind::Call, brain); str(module); str(method); std::fwrite(&arg, 8, 1, f_); bytes_ += 8;
}
void BrainTapeWriter::actions(int brain, ogma::OgmaInstance& inst) {
    if (!f_) return;
    std::vector<std::pair<std::string, float>> out;
    for (auto* m : inst.modules())
        for (auto const& spec : m->output_topics())
            if (spec.name.rfind("action.", 0) == 0)
                if (auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(inst.bus()->last_value(spec.name)))
                    out.emplace_back(spec.name, a->accel);
    head(TapeKind::Actions, brain); u32(uint32_t(out.size()));
    for (auto const& [t, v] : out) { str(t); f32(v); }
    u64(brain_digest(inst));
}

uint64_t brain_digest(ogma::OgmaInstance& inst) {
    uint64_t h = 1469598103934665603ull;                 // FNV-1a over the bytes of every published output
    const auto mix = [&](const void* p, size_t n) {
        const auto* b = static_cast<const unsigned char*>(p);
        for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
    };
    for (auto* m : inst.modules())
        for (auto const& spec : m->output_topics()) {
            auto msg = inst.bus()->last_value(spec.name);
            if (!msg) continue;
            mix(spec.name.data(), spec.name.size());
            if (auto p = std::dynamic_pointer_cast<const ogma::ProprioToken>(msg)) mix(p->values.data(), sizeof(float) * size_t(p->values.size()));
            else if (auto a = std::dynamic_pointer_cast<const ogma::ActionOut>(msg)) mix(&a->accel, sizeof(a->accel));
            else if (auto r = std::dynamic_pointer_cast<const ogma::RealityToken>(msg)) { mix(&r->winner_id, sizeof(r->winner_id)); mix(&r->tle, sizeof(r->tle)); }
        }
    return h;
}

void tape_param(int brain, ogma::Module* m, const std::string& key, const ogma::ParamValue& v) {
    if (g_brain_tape) g_brain_tape->param(brain, std::string(m->id()), key, v);
    m->on_param_change(key, v);
}

// ---------------------------------------------------------------------------------------------------------------------

BrainTapeReader::BrainTapeReader(const std::string& path) {
    f_ = std::fopen(path.c_str(), "rb");
    if (!f_) return;
    char magic[sizeof(kMagic) - 1];
    good_header_ = std::fread(magic, 1, sizeof(magic), f_) == sizeof(magic) && std::memcmp(magic, kMagic, sizeof(magic)) == 0;
}
BrainTapeReader::~BrainTapeReader() { if (f_) std::fclose(f_); }

bool BrainTapeReader::rd(void* p, size_t n) { return std::fread(p, 1, n, f_) == n; }
bool BrainTapeReader::u8(uint8_t& v)   { return rd(&v, 1); }
bool BrainTapeReader::u32(uint32_t& v) { return rd(&v, 4); }
bool BrainTapeReader::u64(uint64_t& v) { return rd(&v, 8); }
bool BrainTapeReader::f32(float& v)    { return rd(&v, 4); }
bool BrainTapeReader::str(std::string& s) {
    uint32_t n = 0;
    if (!u32(n) || n > kMaxStr) return false;
    s.resize(n);
    return n == 0 || rd(s.data(), n);
}

bool BrainTapeReader::next(TapeRecord& r) {
    if (!ok()) return false;
    uint8_t k = 0, b = 0;
    if (!u8(k)) return false;                    // clean end
    if (!u8(b)) return false;
    r.kind = TapeKind(k); r.brain = b;
    r.values.clear(); r.actions.clear();
    bool good = true;
    switch (r.kind) {
        case TapeKind::Config:
        case TapeKind::Restore: good = str(r.json); break;
        case TapeKind::Token: {
            uint32_t n = 0;
            good = u64(r.tick) && str(r.a) && str(r.b) && str(r.c) && u32(n) && n <= kMaxFloats;
            if (good) { r.values.resize(n); good = n == 0 || rd(r.values.data(), 4ull * n); }
            break;
        }
        case TapeKind::Event: {
            float in = 0.0f;
            good = u64(r.tick) && str(r.a) && str(r.b) && str(r.c) && f32(in);
            r.values.assign(1, in);
            break;
        }
        case TapeKind::Param: good = str(r.a) && str(r.b) && str(r.json); break;
        case TapeKind::Tick: good = u64(r.tick); break;
        case TapeKind::Call: good = str(r.a) && str(r.b) && rd(&r.arg, 8); break;
        case TapeKind::Actions: {
            uint32_t n = 0;
            good = u32(n) && n <= 4096;
            for (uint32_t i = 0; good && i < n; ++i) {
                std::string t; float v = 0.0f;
                good = str(t) && f32(v);
                r.actions.emplace_back(std::move(t), v);
            }
            good = good && u64(r.digest);
            break;
        }
        default: good = false;
    }
    if (!good) { std::fprintf(stderr, "tape: truncated or corrupt record after %llu records\n", (unsigned long long)n_); return false; }
    ++n_;
    return true;
}

}  // namespace mjhost
