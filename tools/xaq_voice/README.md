# xaq_voice — the brain's surprise, audible

An **instrument**, not a behaviour: it subscribes to the same diag stream the inspector
reads (control socket on `OGMA_INSPECTOR_PORT`, diag on port+1), so the brain neither
controls it nor pays for it. It runs unchanged against the sim (laptop speakers) and
against `ogma_host` on the Pi (HAT speaker: `dtoverlay=hifiberry-dac`, enable pin GPIO20
high).

```sh
cmake -S tools/xaq_voice -B tools/xaq_voice/build && cmake --build tools/xaq_voice/build -j
# with a sim running in the Godot UI (inspector port 7400 by default):
tools/xaq_voice/build/xaq_voice                        # discover, build a patch, play
tools/xaq_voice/build/xaq_voice --config patches/mine.json
tools/xaq_voice/build/xaq_voice --no-audio --verbose   # the mapping only, no sound
```

With no `--config` it subscribes to every module, watches for a second, and builds a patch
from **what actually arrived** — so it works on any brain config, including ones that did
not exist when it was written. Then [the studio](../xaq_voice_studio/README.md) is how you
make it sound like something:

```sh
tools/run_voice_studio.sh          # sliders, meters, and Save As…
```

## The patch

A patch is the whole instrument as data — sources, oscillators, routes, filters:

```json
{ "master": { "volume": 0.5, "quantize": true, "scale": "major_pentatonic",
              "filter": { "enabled": true, "mode": "vowel",
                          "vowel_a": "O", "vowel_b": "E", "morph": 0.0 },
              "routes": [ { "source": {"module":"voter","key":"fused_tle"},
                            "dest": "vowel_morph", "depth": 1.0,
                            "norm": {"mode":"median_mad","z_lo":0,"z_hi":4} } ] },
  "voices": [ { "id": "body_pose", "module": "body_pose",
                "osc": { "waveform": "saw", "base_hz": 261.63, "glide_ms": 30 },
                "filter": { "enabled": true, "mode": "bandpass", "cutoff_hz": 1200 },
                "routes": [
                  { "source": {"module":"body_pose","key":"last_tle"}, "dest": "pitch",
                    "norm": {"mode":"median_mad","z_lo":0,"z_hi":4,"smooth_ms":60},
                    "depth": 24.0 },
                  { "source": {"module":"body_pose","key":"last_tle"}, "dest": "amp",
                    "norm": {"mode":"threshold_ratio","ref_key":"novelty_threshold_now",
                             "gate":1.4,"full":2.0},
                    "depth": 1.0, "curve": 0.5 } ],
                "events": [ { "source": {"module":"body_pose","key":"baked_now"},
                              "trigger": "true", "sound": "chirp_up" } ] } ] }
```

**Any source can reach any destination** — `pitch`, `amp`, `level`, `cutoff`, `resonance`,
`pulse_width`, `noise_mix`, `vowel_morph`, `pan`, `detune` — and the master bus has its own
rack, which is where `vowel` earns its keep: the whole mix speaking one vowel that morphs
with a signal belonging to no single voice reads as the brain's mood rather than as any one
module's opinion.

**Normalisation is per route, and it is the part that makes this work for any brain.**
Every signal has a different scale and most have never been looked at: `last_tle` lives
near 0.1, `nodes` counts to hundreds, `upright` sits at 1.0 and dips. A depth slider means
nothing until the source is mapped onto 0..1, so the mode is part of the route —
`median_mad`, `threshold_ratio`, `minmax`, `delta` (a counter's value is a ramp; its *rate*
is where the event is), `raw`. The studio's README walks through choosing between them.

Oscillators: `sine`, `triangle`, `saw`, `square`, `pulse`, `noise_white`, `noise_pink`,
plus a `noise_mix` that blends noise over any of them. Filters, per voice and on the
master: `lowpass`, `highpass`, `bandpass`, `notch`, and `vowel` — three formant resonators
with a continuous morph between any two of A E I O U.

## The default mapping

Launch with no `--config` and you get what this tool has always done, voice for voice:

| signal | sound |
|---|---|
| a module's **TLE**, normalised by its own running median/MAD | **pitch**: 0–2 octaves above the voice's base as z goes 0→4, on a log scale; **quantised** to a major pentatonic (voices an octave apart form chords) or **raw** |
| TLE **relative to the novelty threshold** (`novelty_threshold_now` where the module publishes one; median + 1 MAD where it does not) | **volume**: silent until `tle > gate·threshold` (**1.4×**), full at `full·threshold` (**2.0×**), `x^γ` between (**γ 0.5**) — set by ear on the corridor sim, 2026-08-28. Silence = "I know this" |
| a GNG node **baking** / **mitosis** / **prune** | a rising chirp / two notes / a low blip |
| each module | its own voice; bases C3, C4, C5, … in the brain's own module order, so `motor_epm` is the LOWEST voice |

`--gate 1.4 --full 2.0 --gamma 0.5 --span 24 --base motor_epm=A2,body_pose=E4` still work,
applied as overrides on top of whatever patch was built or loaded. `--vary` gives each
voice a different waveform instead of all square.

Keys: `q` quantise · `t` tone · `+`/`-` volume · `m` mute · `1`-`9` select voice ·
`<` `>` semitone · `{` `}` octave · `w` save (needs `--save`) · `v` list voices · `x` quit.

## What the brain publishes

Sources come from the `lite` diag topic, which every module may override
(`Module::diag_lite()`). Nine modules do: `EPM`, `MotorEPM`, `MotorEPMv2`, `GradientEPM`,
`SequenceGNG`, `NeurochemState`, `LateralVoter`, `HomeostaticDrive`, `GainEvolver` — about
60 signals on the picrawler's corridor config.

A module that is subscribed and publishes nothing is **named on startup and shown greyed in
the studio**, because a silent oscillator with no diagnostic is worse than an error.
Wanting a new signal is a one-line addition to that module's `diag_lite()`; that is the one
place this tool may touch brain code, and it must stay O(1) and additive. `cpp_core/tests/
ogma/test_diag_lite.cpp` pins the contract.

---

## Working on this tool (hand-off for a separate session)

**What it is, and is not.** A diagnostic instrument — the audible counterpart of the
inspector. The brain does not control it and cannot hear it, so nothing here touches the
rewrite rule (`CLAUDE.md` §1). The genuinely different project — the speaker as an *action*
the brain hears back through a cochlear EPM, so vocalisation emerges from a prediction to
fulfil — is not this tool and must not be smuggled in as a scripted mapping (prohibition
§7). Keep the two apart.

**Contract with the brain — the only coupling.**
- Control: TCP, newline-delimited JSON on `OGMA_INSPECTOR_PORT` (default 7400):
  `list_modules` → `{status, modules:[{id,type}]}`; `module_subscribe_diag {id, topic:"lite", hz}`
  → `{sub_id, diag_port, topic_prefix}`; `unsubscribe {sub_id}`.
- Diag: ZMQ PUB on port+1, **two-part** frames `["diag.<sub_id>.", json]` with
  `{sub_id, module_id, tick_id, topic, snapshot}`.
- **Always subscribe `lite`**: the full EPM snapshot is ~50 KB serialised on the tick
  thread per frame per subscription — that was measured to matter.
- **Always unsubscribe** on exit (signals are handled). A leaked subscription costs the sim
  on every tick, forever, and they stack across restarts.
- The requested rate is decimated to whole ticks — `60/round(60/hz)` — so `--hz 50` yields
  60, not 50. The startup line reports what will actually arrive.

**Layout.**

| file | holds |
|---|---|
| `src/dsp.hpp` | oscillators, the state-variable filter, the vowel bank, notes and scales |
| `src/patch.hpp/.cpp` | the schema, its JSON round-trip, pointer ops, `auto_patch()` |
| `src/engine.hpp/.cpp` | the source registry, route evaluation, the audio path |
| `src/control.hpp/.cpp` | the studio's REP commands and 15 Hz meter PUB |
| `src/main.cpp` | command line, the brain connection, discovery, thread wiring |

Two algorithm choices are load-bearing and both come from the same constraint — **the brain
modulates this at audio rate**. Oscillators are polyBLEP because a naive square aliases the
moment pitch moves; the filter is a TPT/Zavalishin state variable because a direct-form
biquad blows up when its coefficients jump. Deps are libzmq, ALSA, nlohmann/json
(FetchContent); no brain headers are included.

```sh
cmake -S tools/xaq_voice -B tools/xaq_voice/build && cmake --build tools/xaq_voice/build -j
./tools/xaq_voice/build/test_xaq_voice        # ~408k checks, no sim, no sound card
```

The tests cover the patch round-trip, every normalisation mode, filter stability under a
full cutoff sweep at max Q, DC offset, and that a disabled voice is *digitally* silent.
They earned their place immediately, catching a JSON-pointer op that silently grew an array
rather than rejecting a bad path, and a pulse-width compensation that introduced a DC
offset — several voices each carrying DC sum into a thump the master soft clipper then
bakes in.

**Testing without ears.** `--no-audio --verbose` prints the mapping once a second. Run the
sim headless on its own inspector port so it cannot collide with a Godot window the
operator has open (any scene with an `OgmaBrain` binds 7400/7401 — the bench dashboard
included):

```sh
cd godot_host/project && OGMA_PICRAWLER_BODY=measured OGMA_PICRAWLER_GYM=corridor \
OGMA_PICRAWLER_CONFIG=res://addons/ami_ogma/configs/the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__native_measured.json \
OGMA_RESET_MODE=continuous OGMA_PICRAWLER_MAX_STEPS=6000 OGMA_INSPECTOR_PORT=7500 \
godot4 --headless --fixed-fps 60 --quit-after 4000000 --path . res://scenes/the_picrawler.tscn &
tools/xaq_voice/build/xaq_voice --no-audio --verbose --port 7500
```

⚠ Do **not** run headless sims on the laptop while the operator is listening to the UI sim:
the two starve each other (tick rate varied 4× with load) and it reads as "the voice slows
the sim". Kill your own sim by pid; the process is named `Godot_v4.6.2-st…`, not `godot4`,
and the operator's windows are the same binary. A second sim that cannot bind 7500 keeps
running anyway and logs only `Control socket bind failed` — check for strays before
concluding the tool is broken.

**State of tuning.** Gate 1.4× / full 2.0× / γ 0.5 and the octave ladder were set by ear on
the corridor sim (2026-08-28) and remain the defaults. Everything past that is now a patch,
and **no tuned patch ships** — `patches/` explains why. That is the open work: the
mechanism is there, the listening is not done.

**Roadmap, one dimension at a time.**
1. Neurochem → timbre now has its source (`NeurochemState::diag_lite()`); it wants a patch
   that uses it, and a slow pulse tempo the current oscillator cannot do.
2. On the Pi: the HAT speaker is an I²S DAC + amp (`dtoverlay=hifiberry-dac`, enable pin
   **GPIO20** high, play 0.5 s of silence after enabling as SunFounder does). Same tool,
   pointed at `ogma_host`'s publisher; audio off the tick thread by construction. One bench
   check: the speaker sits near the ultrasonic module's 40 kHz receiver.
3. A safety sound that is NOT the brain: the bench deadman / limp gets a descending tone.
4. Stereo is live but the field is unused — per-leg panning would put the body in space.

**Coordination.** This tool lives on `picrawler-dev` alongside the hardware work. Work it
on its own branch (`git switch -c xaq-voice picrawler-dev`) or a worktree; the only shared
files are the `diag_lite()` overrides in `cpp_core`, which are additive.
