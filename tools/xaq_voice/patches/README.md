# Patches

A patch is one instrument: which brain signals are sources, which oscillator each drives,
through what normalisation, into which synthesis destination. `xaq_voice --config <file>`
loads one; the studio's **Save As…** writes one.

`xaq_voice` with no `--config` discovers what the running brain publishes and builds a
patch from it, so nothing here is needed to make sound. What is worth keeping is a patch
somebody **tuned by ear** — that is a listening decision, not something that can be
derived. A patch is small, diffable JSON; treat it the way the arena configs are treated,
as an artifact of a session worth the commit.

| patch | tuned against | what it is |
|---|---|---|
| `picrawler1.json` | corridor, `native_measured` | `motor_epm` as a **pink-noise** bass at 50.65 Hz through a resonant bandpass that `couple_R` opens and closes; `body_pose` a saw whose `ema_tle` drives pitch, amplitude and vowel morph at once, with the GNG life events chirping over it. Master runs a U→I vowel with a steep (curve 3.76) morph, so the mouth only opens near the top of the range. `body_pose_t` and `gain_evolver` are kept but switched off. |
| `picrawler2.json` | corridor, `native_measured` | The same tuning with `motor_epm` as a **saw** at 57.88 Hz instead — the one voice changed, so the two files diff to a single oscillator. |

Both quantise **off** and lean on `minmax` for the `body_pose` routes rather than the
`median_mad` default, which is worth knowing: `minmax` tracks the running extremes, so it
keeps a signal expressive once its spread has settled, where a z-score keeps re-centring
on whatever the last ten seconds looked like.
