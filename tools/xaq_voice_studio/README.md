# xaq_voice studio — tuning the brain's voice while it runs

A Qt front end for [`tools/xaq_voice`](../xaq_voice/README.md). It shows every signal the
running brain publishes, lets any of them drive any part of the synth, and writes the
result as a patch the engine loads at launch.

```sh
# 1. a brain, on its own inspector port
cd godot_host/project && OGMA_INSPECTOR_PORT=7500 godot4 --headless … the_picrawler.tscn &

# 2. the engine — the only thing that talks to the brain
tools/xaq_voice/build/xaq_voice --port 7500

# 3. the studio — talks only to the engine
tools/run_voice_studio.sh
```

Three panes:

| pane | what it is for |
|---|---|
| **Sources** | every numeric signal any module publishes, live, with a sparkline. Double-click one to route it into the selected voice. Modules that publish nothing are greyed, with the file to fix in the tooltip |
| **Voice** | the oscillator (waveform, base pitch, width, noise, glide/attack/release), its own filter, and its **modulation rack** — one row per route |
| **Master** | output volume, quantise/scale, the output filter, and a **master mod rack** so the filter is modulatable by a signal belonging to no single voice |

`Auto-assign` rebuilds the whole patch from what the brain is currently publishing.
`Save As…` writes the patch; `xaq_voice --config <file>` loads it.

## Reading a route

```
  ▌ body_pose.last_tle  →  cutoff              ✓  ✕
    norm   threshold_ratio
    ref    novelty_threshold_now
    gate   1.40×      full  2.00×     smooth  60 ms
    depth  −18.00 st  curve 0.50      [ ] inv
    out    ▓▓▓▓▓▓░░░░░░░░░░░░         −7.42
```

* **source** — any signal, addressed `module.key`.
* **dest** — what it moves. The depth unit follows it: semitones for pitch and cutoff, Q
  for resonance, 0..1 for the rest.
* **norm** — how the source's own range maps onto 0..1 *before* depth applies. This is the
  control that makes one slider mean the same thing for `last_tle` near 0.1 and `nodes` in
  the hundreds, and it is the first thing to check when a route seems to do nothing:
  * `median_mad` — z-score against a running median and MAD. The default for error
    signals, and what survives a GNG baking and rescaling itself.
  * `threshold_ratio` — the value against another key on the same module, with a gate/full
    window. This is what makes silence mean "I know this" rather than "the number is
    small".
  * `minmax` — running window extremes. For a bounded quantity.
  * `delta` — the per-second **rate**. A counter's value is a ramp and makes a dull
    modulator; its rate is where the event is.
  * `raw` — an explicit input range, when you know it.
* **curve** — `x^curve`. Below 1 opens up the quiet end, which is usually what a spiky
  error signal needs to be expressive rather than binary.
* **out** — the route's live contribution. A route doing nothing is visibly doing nothing.

Colour groups destinations into three families — blue for pitch, orange for level, green
for shape — and the chip is only ever a secondary cue, because the destination's name is
beside it at all times. Three rather than ten is measured, not stylistic: see the note in
[`theme.py`](theme.py), and re-run
`tools/xaq_inspector/validate_palette.py` before changing any of them.

## How it connects

```
  brain ──diag "lite"──▶  xaq_voice  ──ALSA──▶ speakers
   :7500 / :7501             │  ▲
                REP :7460 ───┘  └─── PUB :7461  (meters, 15 Hz)
                          │             │
                        xaq_voice_studio
```

**The studio never connects to the brain.** The engine is already subscribed to every
module, so a second subscriber would double the sim's per-tick serialisation cost for
nothing — and a leaked subscription costs the sim on every tick for the life of the
process, and they stack across restarts. Routing everything through the engine makes that
leak structurally impossible from here.

The two hops are the same shape as the brain's own inspector interface, which is why
[`xaq_inspector/transport.py`](../xaq_inspector/transport.py)'s `DiagSubscriber` is reused
verbatim for the meter stream. The command half cannot be: `ControlClient` speaks
newline-JSON over raw TCP and the engine speaks ZMQ REQ/REP, so `engine_client.py` has a
small lazy-pirate REQ instead — a REQ socket is strictly alternating, and one lost reply
wedges it until the socket is discarded.

Engine calls run on a worker thread. The inspector calls its own control client inline on
the GUI thread despite the docstring saying not to, so a dead brain freezes it for the
full timeout; that is fixed here rather than copied.

## Working on it

```sh
# layout, with no engine, no sim and no sound card — synthetic data, offscreen Qt
PYTHONPATH=tools .venv/bin/python3 tools/xaq_voice_studio/render_studio_preview.py /tmp/s.png

# the palette gate, before changing any colour
.venv/bin/python3 tools/xaq_inspector/validate_palette.py \
    "#3987e5,#d95926,#199e70" --mode dark --surface "#1a1a19" --pairs all
```

Patterns worth keeping, all inherited from the inspector: the ZMQ thread never touches a
widget (it crosses on a one-signal `QObject` bridge); payloads are buffered and redrawn on
a timer rather than per frame; custom-painted widgets subclass `SurfaceWidget` or Qt's
light default background makes every ink token invisible; and the window is sized relative
to the screen, because a hardcoded 1280 put the inspector's title bar out of reach on a
150%-scaled display.

Adding a waveform, filter mode, destination or normalisation mode needs **no change
here** — the studio asks the engine what it supports via `hello` and builds its combo
boxes from the reply.

`requirements.txt` pins the same four packages as the inspector. PyQt6 is GPL-3.0 or
commercial; this is one of the import sites enumerated in
[`THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md).
