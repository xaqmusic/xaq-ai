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
