#include "ogma/hw/AudioCapture.hpp"

#include <alsa/asoundlib.h>
#include <algorithm>
#include <cmath>

namespace ogma::hw {

AudioCapture::AudioCapture(Config cfg) : cfg_(std::move(cfg)) {}

AudioCapture::~AudioCapture() { stop(); }

bool AudioCapture::start() {
    if (running_) return true;
    snd_pcm_t* pcm = nullptr;
    int rc = snd_pcm_open(&pcm, cfg_.device.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) { err_ = "snd_pcm_open(" + cfg_.device + "): " + snd_strerror(rc); return false; }

    // S16_LE through the "plug" plugin: the most universally supported capture format,
    // with ALSA converting whatever the card actually does.  The conversion to float
    // is ours, below, because the EPM's contract is float PCM in [-1, 1].
    unsigned rate = cfg_.rate;
    rc = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                            cfg_.channels, rate, /*soft_resample=*/1,
                            /*latency_us=*/100000);
    if (rc < 0) {
        err_ = std::string("snd_pcm_set_params: ") + snd_strerror(rc);
        snd_pcm_close(pcm);
        return false;
    }
    // ⚠ MEASURED 2026-08-30: a hw:/plughw: capture stream must be explicitly prepared
    // AND started, or the very first snd_pcm_readi returns -EIO, every time, on every
    // latency and resample setting.  Only the "default" (dmix/plug chain) device
    // auto-starts -- which is why arecord works and a direct open does not.
    if ((rc = snd_pcm_prepare(pcm)) < 0) {
        err_ = std::string("snd_pcm_prepare: ") + snd_strerror(rc);
        snd_pcm_close(pcm);
        return false;
    }
    if ((rc = snd_pcm_start(pcm)) < 0) {
        err_ = std::string("snd_pcm_start: ") + snd_strerror(rc);
        snd_pcm_close(pcm);
        return false;
    }
    ring_.assign(size_t(cfg_.window_samples) * size_t(cfg_.ring_windows > 0 ? cfg_.ring_windows : 1),
                 0.0f);
    wpos_ = 0; written_ = 0; read_at_ = 0;
    pcm_ = pcm;
    running_ = true;
    err_.clear();
    th_ = std::thread(&AudioCapture::run, this);
    return true;
}

void AudioCapture::stop() {
    // Unconditional join.  The capture thread clears running_ itself when ALSA hands
    // it an unrecoverable error, so gating the join on running_ leaves a joinable
    // std::thread to be destroyed -- which is std::terminate, not a leak.
    running_ = false;
    if (th_.joinable()) th_.join();
    if (pcm_) { snd_pcm_close(static_cast<snd_pcm_t*>(pcm_)); pcm_ = nullptr; }
}

void AudioCapture::run() {
    auto* pcm = static_cast<snd_pcm_t*>(pcm_);
    const unsigned n = cfg_.window_samples;
    std::vector<int16_t> raw(size_t(n) * cfg_.channels);
    std::vector<float>   buf(n);

    while (running_) {
        snd_pcm_sframes_t got = snd_pcm_readi(pcm, raw.data(), n);
        if (got < 0) {
            // An overrun is normal under load and recoverable; count it rather than
            // dying, so a busy tick cannot silently end the sensory channel.
            ++xruns_;
            if (snd_pcm_recover(pcm, int(got), /*silent=*/1) < 0) {
                // Say what ALSA said.  A dead sensory channel that reports only
                // "0 windows" is indistinguishable from a silent room.
                err_ = std::string("snd_pcm_readi: ") + snd_strerror(int(got));
                running_ = false;
                return;
            }
            continue;
        }
        const unsigned have = unsigned(got);
        float pk = 0.0f;
        if (cfg_.channels == 1) {
            for (unsigned i = 0; i < have; ++i) {
                buf[i] = float(raw[i]) / 32768.0f;
                pk = std::max(pk, std::fabs(buf[i]));
            }
        } else {
            // Downmix to mono: the cochlear encoder can take stereo, but one honest
            // channel beats two of an unknown mic's channels being the same signal.
            for (unsigned i = 0; i < have; ++i) {
                float s = 0.0f;
                for (unsigned c = 0; c < cfg_.channels; ++c)
                    s += float(raw[size_t(i) * cfg_.channels + c]) / 32768.0f;
                buf[i] = s / float(cfg_.channels);
                pk = std::max(pk, std::fabs(buf[i]));
            }
        }
        {
            std::lock_guard<std::mutex> lk(m_);
            // Unread samples about to be overwritten are a real loss; with a ring deeper
            // than ALSA's buffer this stays zero, and a non-zero value means the tick
            // stopped polling rather than that audio arrived in a burst.
            const uint64_t unread = written_ - read_at_;
            if (unread + have > ring_.size())
                dropped_ += (unread + have) - ring_.size();
            for (unsigned i = 0; i < have; ++i) {
                ring_[wpos_] = buf[i];
                wpos_ = (wpos_ + 1) % ring_.size();
            }
            written_ += have;
            peak_ = pk;
            ++windows_;
        }
    }
}

bool AudioCapture::latest(std::vector<float>& out) {
    std::lock_guard<std::mutex> lk(m_);
    const size_t n = cfg_.window_samples;
    if (ring_.empty() || written_ < n) return false;   // still filling
    if (written_ == read_at_) return false;            // no new audio since last call
    out.resize(n);
    // Walk back n samples from the write head: the newest CONTIGUOUS window, regardless
    // of how raggedly ALSA delivered it.
    size_t idx = (wpos_ + ring_.size() - n) % ring_.size();
    for (size_t i = 0; i < n; ++i) {
        out[i] = ring_[idx];
        idx = (idx + 1) % ring_.size();
    }
    read_at_ = written_;
    return true;
}

} // namespace ogma::hw
