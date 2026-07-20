# Picrawler active-inference gait loop — findings (L-1b: steps 2–3 + Gate 2)

Status as of 2026-07-19. Companion to `docs/plans-and-designs/picrawler_active_inference_plan.md`.
All results are headless, reset-masked, reward-free. Verification scripts live in the session
scratchpad (`gate1*/brt/gait_metrics/gate2*` collectors).

## The architecture under test

A closed gait loop decomposed as **substrate / objective / arbitration**:

- **Substrate** — `MotorEPM` (homeokinetic self-model + control), behaviour-agnostic.
- **Objective** — `KeyframeGait` learns a **phase-indexed keyframe map** (whole-body posture vs
  gait phase) and feeds it back as a soft target on the `objective.posture.*` socket
  (a `PredictionToken {target, confidence}`; the controller descends a retargeted error, never
  an additive injection).
- **Coordinating clock** — `CPGOscillator`, a smooth phase everyone keys off.
- **Afferent estimator** — `BodyRhythmTracker` (new), turns proprioception into a body-gait-phase
  estimate the CPG entrains to.

## Step 2 — the washout (falsifiable branch, as designed)

`KeyframeGait` binned whole-body posture against a **free-running** CPG (fixed period 60). The map
**washed out**: `keyframe_tle` pinned ~0.87, `mean_consistency` fell 0.99→0.52. Root cause was
structural, not a bug — **two uncoupled clocks**: the CPG ran an autonomous 60-tick clock while
`MotorEPM`'s gait rhythm is self-generated with no CPG input. Binning the body against a clock it
doesn't follow smears every bin. → the CPG↔body PLL (step 3) is required, cleanly diagnosed.

## Step 3 — CPG↔body PLL, entrainment, and the confidence gate

- **`BodyRhythmTracker`** — a morphology-specific trot collective coordinate
  `F(t) = Σ_leg sign·pos[swing_joint]` feeds a **measurement-seeded PLL** (frequency from the
  unbiased hysteresis up-crossing interval; phase = smooth integrator locked to the crossings).
  A Righetti–Ijspeert adaptive Hopf oscillator was tried first and **abandoned** — at dt=1 it
  showed a strong-forcing frequency bias (railed to the wrong frequency even when started exactly
  on the true one), while the zero-crossing period read exact every time.
- **CPG entrainment** — the CPG optionally tracks `rhythm.body.gait`: slow period pull toward the
  body's measured frequency + a phase nudge, clamped to an **aliasing-safe** band `[48,120]`
  (≥3 ticks/bin at 16 bins). Additive, default-off, byte-identical for existing configs.
- **Measurement (measure-first)** — the body walks at **~70 ticks (0.85 Hz), not 60**; the free CPG
  was ~17% too fast. `hip1` is the dominant carrier. Notably **the knee cycles at a different
  period (~42) than the hip (~70)** — the body is *not* a clean single-frequency limit cycle.
- **Confidence gate** — the published confidence is `gain · self_precision(bin)`, where
  `self_precision = warmup_ramp · exp(−bin_dev/scale)`: a smeared or unproven bin drives weakly or
  not at all ("drive on consistency"). This is the objective's **bottom-up self-precision** — the
  exact quantity the future EFE arbiter multiplies its **top-down allocation** onto
  (`w_final = gain · self_precision · allocation`; gate and arbiter compose, they don't compete).

**Gate-1 outcome (entrained + gated):** `keyframe_tle` falls (0.92→0.82) *while* the gait stays
coherent — the premature-drive failure (ungated: `keyframe_tle` rises, coherence collapses 0.20→0.06)
is fixed. But crystallization **plateaus** (`bin_dev`≈0.8 ⇒ precision ~0.26). Raising drive gain
made it worse, so the plateau is **coherence-limited, not drive-limited** — the ceiling is the body's
own multi-frequency incoherence.

## Gait-quality metrics (beyond `keyframe_tle`)

To judge "what better looks like" we measure locomotion (`fwd_v`), stability (falls/streak),
`gait_coherence`, **whole-body frequency lock** (per-joint period spread — one period = clean limit
cycle), rhythm regularity (period CV), and effort (`motor_tle`, `out_mag`).

**Baseline (coupling ON), 3-min:** `fwd_v` +0.041, coherence ~0.41, **joint-period spread ~32**
(hip1≈61 / hip2≈29 / knee≈42; hip2 amplitude near-dead 0.095), CV ~0.20, stable after settling. The
gated keyframe drive is **not fighting** the coupling gait — marginally better (faster, steadier,
knee period nudged toward hip). The whole-body incoherence is inherent to the imposed coupling
(present drive on *or* off).

## Gate 2 — retire the imposed coupling (Kuramoto contrast)

Because the map can only learn from an existing gait, a fresh-start coupling-off run has no
coordinator to bootstrap from. So we test **crystallize-then-wean** via a deterministic
tick-scheduled `coupling_fade` in `MotorEPM` (a TCP `set_param` verb was added but the control
server is single-connection + idle-times-out, so live mid-run control is fragile; the baked fade is
deterministic and collected purely over the diag stream).

**Result (fade 70→90s, observe to ~150s):**

| phase | fwd_v | gait_coherence | joint-period spread | falls |
|---|---|---|---|---|
| COUPLED (1.0–1.55) | **+0.062** | 0.45 | **21.5** (h1 52 / h2 31 / kn 43) | 1 |
| WEANED (→0) | **+0.001** | 0.46 | **75.1** (h1 89 / h2 15 / kn 14) | 0 |

**Coordination is still imposed, not learned.** Removing coupling stops locomotion (`fwd_v`→0) and
the joints scatter to unrelated frequencies (spread 21→75); the body stays upright (posture + the
Gate-0 prior) but doesn't walk. The learned map (precision ~0.25) is too weak to carry the gait —
the §2.5 chicken-and-egg. A slower wean won't rescue it (the plateau is coherence-limited).

**Metric caveat:** `gait_coherence` held ~0.46 *through* the collapse — it survives a near-static
body, so it is a **misleading** Gate-2 metric. `fwd_v` and joint-period spread are the honest signals.

## The crux, and an open question

The plateau, the premature-drive fragility, and the Gate-2 collapse are one problem: **the body is
not a clean single-frequency whole-body limit cycle** (hip2/knee ≠ the hip1 fundamental; hip2 nearly
dead). Everything downstream is capped by that.

Open, before pushing further on wean/adapt mechanisms: **is a trot the right target gait for this
stiff, hobby-servo quadruped at all?** A trot is dynamically unstable (only two legs down), which
shows up as belly-up flips and a body that fights the imposed coordination. A statically-stable
four-beat walk may be far easier to synchronise and keep upright — possibly dissolving the
incoherence at its root rather than fighting it downstream.

## Reusable pieces added

- `BodyRhythmTracker` (module) — proprioception → body-gait-phase reference.
- `CPGOscillator` afferent entrainment (`entrain_topic`, aliasing clamp).
- `KeyframeGait` self-precision drive gate (`warmup_visits`, `precision_scale`).
- `MotorEPM` `coupling_fade_start/end` — deterministic scaffold-retirement schedule.
- `OgmaBrain` TCP `set_param` verb — live param mutation for experiments (single-connection server).
- `gait_metrics.py` — the quantitative gait-quality metric block.
