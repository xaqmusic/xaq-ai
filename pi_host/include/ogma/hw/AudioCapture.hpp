#pragma once
// AudioCapture — the USB microphone, as a sensory channel.
//
// Feeds an EPM configured `encoder_kind = stft` (the cochlear path of CLAUDE.md §0):
// the host bridges raw PCM onto the Bus as a RawAudioFrame and the encoder does the
// filterbank itself.  The host's whole job is to deliver honest samples on time.
//
// Capture runs on its OWN thread with a ring buffer, and the brain tick only ever
// copies the newest window.  ALSA's read is blocking and period-quantised; calling it
// from the 50 Hz loop would couple the brain's deadline to the sound card's clock,
// which is a different clock (the same lesson as the HAT's 49.95 Hz servo frames).
//
// `latest()` reports whether the window is NEW.  A host that republishes a stale
// window every tick would feed the GNG duplicate observations and inflate its visit
// counts — baking a vocabulary out of the sensor standing still.
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ogma::hw {

class AudioCapture {
public:
    struct Config {
        // ⚠ BY CARD ID, NOT BY INDEX.  Card numbers are enumeration order and they
        // MOVE: adding the hifiberry-dac overlay pushed this mic from card 0 to card 1
        // (measured 2026-08-30), which would have silently pointed capture at an HDMI
        // output with no capture device.  "CARD=Device" is the USB codec's stable id
        // from /proc/asound/cards.  "plug" so ALSA converts rate/format for us.
        std::string device = "plughw:CARD=Device,DEV=0";
        unsigned    rate          = 48000;  // must match the EPM's sample_rate param
        unsigned    channels      = 1;
        unsigned    window_samples = 1024;  // ~21 ms at 48 kHz, about one 50 Hz tick
    };

    explicit AudioCapture(Config cfg);
    ~AudioCapture();
    AudioCapture(const AudioCapture&) = delete;
    AudioCapture& operator=(const AudioCapture&) = delete;

    bool start();          // false + last_error() on failure; never throws
    void stop();
    bool running() const { return running_.load(); }

    // Copies the newest window into `out`.  Returns false when nothing new has
    // arrived since the previous call, so the caller can skip publishing.
    bool latest(std::vector<float>& out);

    const std::string& last_error() const { return err_; }
    unsigned  rate()            const { return cfg_.rate; }
    unsigned  window_samples()  const { return cfg_.window_samples; }
    uint64_t  windows()         const { return windows_; }
    uint64_t  xruns()           const { return xruns_; }
    // Peak absolute sample of the last window — the level meter, and the cheapest
    // possible answer to "is the microphone actually hearing anything?"
    float     peak()            const { return peak_; }

private:
    void run();

    Config      cfg_;
    void*       pcm_ = nullptr;          // snd_pcm_t*, kept opaque to spare the header
    std::thread th_;
    mutable std::mutex m_;
    std::vector<float> window_;          // newest complete window
    bool        fresh_    = false;
    std::atomic<bool> running_{false};
    uint64_t    windows_  = 0;
    uint64_t    xruns_    = 0;
    float       peak_     = 0.0f;
    std::string err_;
};

} // namespace ogma::hw
