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
// A ring absorbs the burst, but storage alone is NOT enough -- the consumer must also
// DRAIN it.  Handing back the newest window and marking the whole backlog read discards
// the older window of every burst: measured on the robot at 14.7 % of captured audio
// (6.85 windows/s of 46.75), with `dropped` reading 0 the entire time (2026-09-05).
// latest() therefore advances a read CURSOR by exactly one window per call, so a burst
// is delivered across consecutive ticks instead of being thrown away.
//
// ⚠ AND THE WINDOW MUST NOT SHRINK BELOW ONE TICK.  One 50 Hz tick is 960 samples at
// 48 kHz; a window shorter than that makes the producer faster than the consumer, and
// the cursor then falls behind until the ring genuinely wraps.  1024 sits just above
// that floor (46.9 windows/s against a 50 Hz drain).  It does NOT "overlap consecutive
// polls by 64 samples" -- this header claimed that until 2026-09-05, but ALSA is read in
// whole 1024-sample blocks, so delivered windows are ADJACENT and tile the stream with
// no overlap.  Shortening it for FFT reasons would trade a resolution the STFT already
// needs (f_min 80 Hz) for a channel the consumer cannot keep up with.
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

    // Copies the OLDEST not-yet-delivered window out of the ring and advances the read
    // cursor by exactly one window.  False only when less than a full new window has
    // arrived since the previous call (ALSA stalled) or the ring has not filled yet — so
    // unlike the camera, this normally succeeds every tick, and should: audio is
    // continuous, and each tick's window covers new time rather than repeating old.
    // Delivering OLDEST-first is what lets a burst survive; see the ring note above.
    bool latest(std::vector<float>& out);

    const std::string& last_error() const { return err_; }
    unsigned  rate()            const { return cfg_.rate; }
    unsigned  window_samples()  const { return cfg_.window_samples; }
    uint64_t  windows()         const { return windows_; }
    uint64_t  xruns()           const { return xruns_; }   // ALSA over/underruns
    // Samples the ring overwrote before the cursor reached them — a true overrun on our
    // side rather than ALSA's, and with a ring deep enough for the ALSA buffer it should
    // stay at zero.  Not the same fault as an xrun, so it is counted separately.
    // ⚠ RESET ON THE FIRST DELIVERY, so this counts loss AFTER the channel is live.
    // Between start() and the host's first poll the capture thread is already filling
    // while the host still loads its config, so the writer laps the ring and charges
    // startup to this counter: 40836 samples measured after one restart, 2.2 M on a
    // long-running host.  A large constant here is not a fault and would drown a real
    // regression, which is the whole job of the number.
    uint64_t  dropped()         const { return dropped_; }
    // Windows actually handed to the consumer.  windows() - delivered() IS the audio the
    // brain never saw; the two must track.  ⚠ While latest() marked the whole backlog
    // consumed, dropped() was structurally incapable of firing and reported 0 through a
    // measured 14.7 % loss -- so this is published BESIDE it, never inferred from it.
    uint64_t  delivered()       const { return delivered_; }
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
    uint64_t    delivered_ = 0;
    float       peak_     = 0.0f;
    std::string err_;
};

} // namespace ogma::hw
