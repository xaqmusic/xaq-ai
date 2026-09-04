#pragma once
// AudioCapture — the USB microphone, as a sensory channel.
//
// Feeds an EPM configured `encoder_kind = stft` (the cochlear path of CLAUDE.md §0):
// the host bridges raw PCM onto the Bus as a RawAudioFrame and the encoder does the
// filterbank itself.  The host's whole job is to deliver honest samples on time.
//
// Capture runs on its OWN thread into a CONTINUOUS RING, and the tick copies the most
// recent window_samples out of it whenever it asks.  ALSA's read is blocking and
// period-quantised; calling it from the 50 Hz loop would couple the brain's deadline to
// the sound card's clock, which is a different clock (the same lesson as the HAT's
// 49.95 Hz servo frames).
//
// ⚠ WHY A RING AND NOT A HANDOFF OF DISCRETE WINDOWS.  The first version published the
// newest completed window and counted the rest as dropped -- 14 % of them, measured.
// The cause is not that the window is longer than the tick; it is that ALSA delivers in
// PERIODS (1200 frames / 25 ms here, measured) that divide into no read size we use, so
// windows arrive in bursts: two back to back, then a wait.  A 50 Hz consumer sees the
// newest of each burst and the rest are gone.
//
// A ring removes the whole class: burstiness is absorbed, producer and consumer rates
// stop having to match, and the consumer always gets the newest CONTIGUOUS window.
//
// ⚠ AND THE WINDOW MUST NOT SHRINK BELOW ONE TICK.  One 50 Hz tick is 960 samples at
// 48 kHz; a window shorter than that leaves GAPS between polls -- audio the brain never
// sees.  1024 sits just above the floor and overlaps consecutive polls by 64 samples,
// which is the property to preserve.  Shortening it for FFT reasons would trade a
// resolution the STFT already needs (f_min 80 Hz) for holes in the channel.
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
        unsigned    window_samples = 1024;  // >= one tick (960 @ 48 kHz) or coverage gaps
        unsigned    ring_windows    = 8;    // ring depth; must exceed ALSA's buffer (100 ms)
    };

    explicit AudioCapture(Config cfg);
    ~AudioCapture();
    AudioCapture(const AudioCapture&) = delete;
    AudioCapture& operator=(const AudioCapture&) = delete;

    bool start();          // false + last_error() on failure; never throws
    void stop();
    bool running() const { return running_.load(); }

    // Copies the most recent window_samples out of the ring.  False only when no NEW
    // audio arrived since the previous call (ALSA stalled) or the ring has not filled
    // yet — so unlike the camera, this normally succeeds every tick, and should: audio
    // is continuous, and each tick's window covers new time rather than repeating old.
    bool latest(std::vector<float>& out);

    const std::string& last_error() const { return err_; }
    unsigned  rate()            const { return cfg_.rate; }
    unsigned  window_samples()  const { return cfg_.window_samples; }
    uint64_t  windows()         const { return windows_; }
    uint64_t  xruns()           const { return xruns_; }   // ALSA over/underruns
    // Samples the ring overwrote before the consumer read them — a true overrun on our
    // side rather than ALSA's, and with a ring deep enough for the ALSA buffer it should
    // stay at zero.  Not the same fault as an xrun, so it is counted separately.
    uint64_t  dropped()         const { return dropped_; }
    // Peak absolute sample of the last window — the level meter, and the cheapest
    // possible answer to "is the microphone actually hearing anything?"
    float     peak()            const { return peak_; }

private:
    void run();

    Config      cfg_;
    void*       pcm_ = nullptr;          // snd_pcm_t*, kept opaque to spare the header
    std::thread th_;
    mutable std::mutex m_;
    std::vector<float> ring_;            // continuous, power-of-nothing-in-particular
    size_t      wpos_     = 0;           // next write index
    uint64_t    written_  = 0;           // total samples ever written
    uint64_t    read_at_  = 0;           // `written_` at the previous latest()
    std::atomic<bool> running_{false};
    uint64_t    windows_  = 0;
    uint64_t    xruns_    = 0;
    uint64_t    dropped_  = 0;
    float       peak_     = 0.0f;
    std::string err_;
};

} // namespace ogma::hw
