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
            // The writer OWNS this accounting, because it is the one destroying data:
            // it counts the overflow and advances read_at_ past the samples it is about
            // to overwrite.  latest() therefore never has to subtract a loss of its own
            // (it would double-count the same samples if it did), and its backlog is
            // guaranteed to be <= ring_.size().
            const uint64_t unread = written_ - read_at_;
            if (unread + have > ring_.size()) {
                const uint64_t lost = (unread + have) - ring_.size();
                dropped_ += lost;
                read_at_ += lost;
            }
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

    // ⚠ DRAIN, DO NOT SAMPLE.  read_at_ is a real read CURSOR, not a high-water mark.
    // The previous version returned the newest window and then set read_at_ = written_,
    // which declared the entire backlog consumed -- so when ALSA delivered two reads
    // between two polls (its 1200-frame period divides into no read size we use, so it
    // bursts), the OLDER window was discarded unread.  Measured on the robot: 798 of 935
    // windows delivered, 14.7 % of captured audio never reaching the brain, while
    // dropped_ read 0 throughout because that assignment kept `unread` at zero.
    // Advancing by exactly one window instead spreads a burst over consecutive ticks;
    // the 50 Hz consumer outruns the 46.9 Hz producer, so the backlog always drains.
    uint64_t backlog = written_ - read_at_;
    if (backlog < n) return false;                     // no complete new window yet

    // Defensive only: the writer already advanced read_at_ past anything it overwrote,
    // so this cannot fire.  It is NOT a second drop counter -- counting here would
    // double-count samples the writer has already charged to dropped_.
    if (backlog > ring_.size()) {
        read_at_ = written_ - ring_.size();
        backlog  = ring_.size();
    }

    out.resize(n);
    // Walk forward n samples from the cursor: the OLDEST contiguous window not yet
    // handed over.  `backlog` samples sit behind the write head, so the cursor is that
    // far back in the ring.
    size_t idx = (wpos_ + ring_.size() - size_t(backlog)) % ring_.size();
    for (size_t i = 0; i < n; ++i) {
        out[i] = ring_[idx];
        idx = (idx + 1) % ring_.size();
    }
    read_at_ += n;
    // ⚠ ZERO THE DROP COUNT ON THE FIRST DELIVERY.  Between start() and the host's first
    // successful poll the capture thread is already filling the ring, and the host is
    // still loading its config and constructing modules -- so the writer legitimately
    // laps the ring and charges tens of thousands of samples to dropped_ before the
    // consumer has asked for anything.  Measured after one restart: 40836 samples
    // (~0.85 s), and 2.2 M on a long-running host.  That is startup, not steady-state
    // loss, and leaving it in makes the counter a constant nobody can read a regression
    // out of.  dropped_ is therefore defined as loss AFTER the channel is live.
    if (delivered_ == 0) dropped_ = 0;
    ++delivered_;
    return true;
}

} // namespace ogma::hw
