# xaq_voice — the brain's surprise, audible

An **instrument**, not a behaviour: it subscribes to the same diag stream the inspector reads
(control socket on `OGMA_INSPECTOR_PORT`, diag on port+1), so the brain neither controls it nor
pays for it. It runs unchanged against the sim (laptop speakers) and against `ogma_host` on the
Pi (HAT speaker: `dtoverlay=hifiberry-dac`, enable pin GPIO20 high).

| signal | sound |
|---|---|
| a module's **TLE**, normalised by its own running median/MAD | **pitch**: 0–2 octaves above the voice's base as z goes 0→4, on a log scale; **quantised** to a major pentatonic (voices an octave apart form chords) or **raw** continuous — toggle live with `q` |
| TLE **relative to the novelty threshold** (`novelty_threshold_now` for GNG EPMs; median + 1 MAD for `MotorEPMv2`, which has none) | **volume**: silent until `tle > gate·threshold` (default **1.4×**), full at `full·threshold` (**2.0×**), `x^γ` between (**γ 0.5**) — defaults set by ear on the corridor sim, 2026-08-28. Silence = "I know this" |
| a GNG node **baking** / **mitosis** | a rising chirp / two notes |
| each subscribed module | its own square-wave voice; default bases C3, C4, C5, … — **tune per voice** with `--base motor_epm=A2,body_pose=E4,body_pose_t=C6` (note names or Hz) or live: `1`–`9` selects a voice, `<`/`>` moves it a semitone, `{`/`}` an octave; `(`/`)` change the pitch span (default 24 semitones over z 0→4). Every change echoes the full `--base …` line to paste into the next launch |

```sh
cmake -S tools/xaq_voice -B tools/xaq_voice/build && cmake --build tools/xaq_voice/build -j
# with a sim running in the Godot UI (inspector port 7400 by default):
tools/xaq_voice/build/xaq_voice                 # all TLE-carrying modules, quantised
tools/xaq_voice/build/xaq_voice --raw --modules motor_epm --volume 0.6
tools/xaq_voice/build/xaq_voice --verbose       # + per-second numbers and event lines
tools/xaq_voice/build/xaq_voice --no-audio --verbose   # the mapping only, no sound
```
Keys: `q` quantised/raw · `t` TLE tone · `b` bake chirps · `n` mitosis chirps · `p` prune blips ·
`,`/`.` gate ×0.1 · `;`/`'` full ×0.5 · `[`/`]` γ ±0.25 · `+`/`-` master volume · `m` mute · `x` quit.
Flags `--gate 1.4 --full 2.0 --gamma 0.5` set the same. Pitch follows a ~60 ms-smoothed TLE and
quantised notes carry hysteresis, so tick jitter no longer warbles. Quiet by default (one header
line + the key hint); `--verbose` prints per-voice tle / z / threshold / margin / Hz / amp once a
second plus each bake / mitosis / prune event.

Not yet: neurochem → timbre, LateralVoter precision → relative loudness, the deadman tone on
the Pi. Those are one added dimension at a time, each named in the ledger.

---

## Working on this tool (hand-off for a separate session)

**What it is, and is not.** A diagnostic instrument — the audible counterpart of the inspector.
The brain does not control it and cannot hear it, so nothing here touches the rewrite rule
(`CLAUDE.md` §1). The genuinely different project — the speaker as an *action* the brain hears
back through a cochlear EPM, so vocalisation emerges from a prediction to fulfil — is not this
tool and must not be smuggled in as a scripted mapping (prohibition §7). Keep the two apart.

**Contract with the brain — the only coupling.**
- Control: TCP, newline-delimited JSON on `OGMA_INSPECTOR_PORT` (default 7400):
  `list_modules` → `{status, modules:[{id,type}]}`; `module_subscribe_diag {id, topic:"lite", hz}`
  → `{sub_id, diag_port, topic_prefix}`; `unsubscribe {sub_id}`.
- Diag: ZMQ PUB on port+1, **two-part** frames `["diag.<sub_id>.", json]` with
  `{sub_id, module_id, tick_id, topic, snapshot}`. With topic `"lite"` the snapshot is
  `Module::diag_lite()` (`cpp_core/include/ogma/Module.hpp`; overrides in `EPM.cpp`,
  `MotorEPMv2.cpp`). **Always subscribe `lite`**: the full EPM snapshot is ~50 KB serialised on
  the tick thread per frame per subscription — that was measured to matter.
- **Always unsubscribe** on exit (signals are handled). A leaked subscription costs the sim on
  every tick, forever, and they stack across restarts.
- Wanting a new scalar from a module = a one-line addition to that module's `diag_lite()`.
  That is the one place this tool may touch brain code; keep it O(1) and additive.

**Layout.** One file, `src/main.cpp`: running stats → `map_voice()` (network thread, one call
per frame) → atomics → `audio_thread()` (256-sample blocks, polyBLEP square, per-voice glide +
attack/release, chirp overlay, soft clip) → ALSA. `keyboard_thread()` owns the live keys.
`Voice` holds everything per module. No brain headers are included; deps are libzmq, ALSA,
nlohmann/json (FetchContent). Build: `cmake -S tools/xaq_voice -B tools/xaq_voice/build`.

**Testing without ears.** `--no-audio --verbose` prints the mapping once a second. Run the sim
headless on its own inspector port so it cannot collide with a Godot window the operator has
open (any scene with an `OgmaBrain` binds 7400/7401 — the bench dashboard included):

```sh
cd godot_host/project && OGMA_PICRAWLER_BODY=measured OGMA_PICRAWLER_GYM=corridor \
OGMA_PICRAWLER_CONFIG=res://addons/ami_ogma/configs/the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__native_measured.json \
OGMA_RESET_MODE=continuous OGMA_PICRAWLER_MAX_STEPS=6000 OGMA_INSPECTOR_PORT=7500 \
godot4 --headless --fixed-fps 60 --quit-after 4000000 --path . res://scenes/the_picrawler.tscn &
tools/xaq_voice/build/xaq_voice --no-audio --verbose --port 7500
```
⚠ Do **not** run headless sims on the laptop while the operator is listening to the UI sim: the
two starve each other (tick rate varied 4× with load) and it reads as "the voice slows the sim".
Kill your own sim by pid; the process is named `Godot_v4.6.2-st…`, not `godot4`, and the
operator's windows are the same binary.

**State of tuning (2026-08-28).** Gate 1.4× / full 2.0× / γ 0.5 were set by ear on the corridor
sim and are the defaults. Per-voice base pitch is live-tunable and **not yet dialled in** — the
operator will hand back a `--base …` line to make default. Note the octave order: `motor_epm`
is the LOWEST voice; `body_pose_t` (C5, the spikiest TLE) is what reaches 2 kHz.

**Roadmap, one dimension at a time (each named in the ledger when it lands).**
1. LateralVoter precision weights (`1/(tle+ε)`) → relative loudness: hear *which* module the
   consensus trusts. Needs a `diag_lite()` on `LateralVoter`.
2. Neurochem → timbre (pulse width) and a slow pulse tempo. Needs `NeurochemState::diag_lite()`.
3. On the Pi: the HAT speaker is an I²S DAC + amp (`dtoverlay=hifiberry-dac`, enable pin
   **GPIO20** high, play 0.5 s of silence after enabling as SunFounder does). Same tool, pointed
   at `ogma_host`'s publisher; audio off the tick thread by construction. One bench check: the
   speaker sits near the ultrasonic module's 40 kHz receiver.
4. A safety sound that is NOT the brain: the bench deadman / limp gets a descending tone.
5. Persisting tunings (`--base`, gate/full/γ, span) to a small JSON alongside the tool.

**Coordination.** This tool lives on `picrawler-dev` alongside the hardware work. Work it on
its own branch (`git switch -c xaq-voice picrawler-dev`) or a worktree; the only shared files
are the `diag_lite()` overrides in `cpp_core`, which are additive.
