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

## RESOLUTION (2026-07-20): the incoherence was INTRA-leg, and a coherent CPG-phase drive fixes it

A raw per-(leg,joint) diagnostic (added to `BodyRhythmTracker`) showed the incoherence is **within
each leg**, not between legs: a single leg's three joints run at *different* frequencies
(e.g. hip1≈61, hip2≈28, knee≈40; intra-leg period spread ~28). The knees actually sync well across
legs (inter-leg std ~3.6) — the coupling works — but the design sourced each leg's oscillator phase
from the **knee** (`L.phase`, MotorEPM.cpp:883) while the actual locomotor rhythm is **hip1** (the
fore-aft stride), and only hip1 was rhythmically driven (via the stroke) — so the joints drifted to
independent frequencies and the keyframe could never crystallize.

**The fix — drive every joint from ONE clean phase.** New MotorEPM knobs (all default-off,
byte-identical): `phase_joint` (which proprio joint sources `L.phase`), a per-joint **coherent
rhythmic drive** `rhythm_gains`/`rhythm_offsets` (`y[j] += g_j·sin(base + off_j)`), and a
`cpg_phase_topic` so `base` is the **clean, entrained global CPG phase** (`+ gait_phase[leg]`)
rather than the noisy proprio-derived `L.phase`. With coupling+stroke off and all joints driven from
the CPG phase:

| | trot (before) | **CPG-phase coherent drive** |
|---|---|---|
| intra-leg period spread | ~28 | **0.0** (knee/hip = 1.00) |
| inter-leg std | ~6 | **0.1** |
| mean fwd_v | +0.041 | **+0.084** (2×) |
| `keyframe_tle` | 0.82 (plateau) | **0.25** |
| `mean_precision` | 0.26 (plateau) | **0.53 → 0.73** |
| `gait_coherence` | 0.41 | **0.62–0.76** |
| belly-up flips | 3 | 0–4 |

The whole body became a **clean single-frequency limit cycle**, which broke the precision plateau and
made the keyframe crystallize (`keyframe_tle` 0.82→0.25, precision 0.26→0.73). This was the root fix.

**Gate 2 on the coherent gait (rhythm-scaffold wean).** Fading the rhythm scaffold to 0, the learned
keyframe **holds the coordination** — the body stays a clean limit cycle (spread 1.1) and matches the
map almost perfectly (`keyframe_tle` 0.08, precision **0.918**) — but **loses propulsion**
(fwd_v→0). The keyframe is a phase-indexed **posture** template: it reproduces the gait's *shape* but
not its propulsive *push*. So coordination is genuinely learnable/holdable; propulsion needs a
rhythmic pump. Practical upshot: keep a residual CPG-phase pump (a permanent rhythm generator, as in
animal locomotion) and let the keyframe learn/refine the coordination — or extend the objective to
carry a phase-indexed feed-forward command, not just posture. (`rhythm_fade_start/end` provides the
deterministic wean schedule for this study.)

## Final tuned gait (`the_picrawler_motor_epm_cpgwalk.json`)

Combining the coherence fix with statically-stable walk phasing and clean propulsion signing yields
a gait that is **3× faster, coherent, and stable** vs the original trot:

- CPG-phase coherent drive (coupling + stroke off; all joints from `rhythm.cpg.body`).
- **Walk phasing** `gait_phase = [π/2, 3π/2, 0, π]` (lateral-sequence LH→LF→RH→RF, more legs down).
- `rhythm_gains = [1.8, 1.0, 1.6]`, `rhythm_offsets = [0, π/2, π/2]` (knee/hip2 lift 90° into swing).
- `stroke_signs = [-1, 1, -1, 1]` — the fore-aft joint (0) carries the per-leg propulsion sign so
  opposite sides push the body the same way → **straight, forward** walking (a MotorEPM change: the
  rhythm drive applies `stroke_signs[leg]` to joint 0).

| | original trot | **final coherent walk** |
|---|---|---|
| mean fwd_v | +0.041 | **+0.116 (≈3×)** |
| joint-period spread | ~32 | **0.5** |
| `mean_precision` | 0.26 (plateau) | **0.52** (crystallizes) |
| `keyframe_tle` | 0.82 | 0.37 |
| belly-up flips (90 s) | 3 | **0–2** |
| Gate 0 (reset_rate) | 0 | 0 |

Remaining polish: CPG clamp sits at 48 while the body runs ~38 (a residual ~1.25× mismatch); lowering
the entrainment clamp toward the body's cadence should tighten crystallization further. Propulsion
after a full scaffold wean still needs a residual pump or a feed-forward-command objective (above).

## CORRECTION (2026-07-20, UI observation): the coherent CPG-walk is REJECTED

The "coherent walk" above scores well headless but is an **open-loop servo sequencer**: driving every
joint from a CPG clock ignores ground contact, so the chassis **slams its corners into the ground every
step** (a flopping fish). This was invisible to fwd_v + joint-coherence and caught only after a
`chassis_h` collision metric was added (fraction of time the chassis dips below 0.35 of target height:
**15.6% for the CPG-walk vs 3.3% for the keyframe**). Coherent ≠ emergent; fast fwd_v ≠ walking. Lesson:
**no gait is good if the chassis collides — always measure chassis height.**

## The emergent path: CPG as an EMBEDDING (not a driver) + firm stance

Back on the `keyframe` config (which showed emergent steps with some "fighting"), the audit found the
keyframer is a phase-indexed POSTURE map fed as a soft position target — a template that *averages* the
gait to a damped smear, so driving toward it fights the very motion it should reproduce. Tweening the
template (smooth C0 vs 16-step staircase) only made it **stiffer** (it lowpasses out the adaptive slack
HK/balance need) — rejected.

The fix demotes the CPG phase from a COMMAND to a CONTEXT (as in the earlier RL pipeline, where a CPG
token was LateralVoter-fused with proprio/IMU into the EPM's input):
- **`cpg_embed` (MotorEPM, default-off)**: the HK controller learns a phase-conditioned bias
  `y = tanh(C·x + Cphi·[cosφ,sinφ] + h)`. `Cphi` is trained NOT on HK surprise (which damps motion — the
  least-surprising thing is a still body) but to reduce the **keyframe error** (x*−x) at the command
  phase — a self-limiting phase-indexed feed-forward toward the learned posture. Phase modulates the
  learned control law; it never commands a joint, so HK exploration + balance reflexes stay live.
- **firm stance** (`postural_gain` 0.3→0.7): the weak postural reflex let the lively controller
  over-extend the legs and sag the chassis; firming it holds the stance → 0 falls, min chassis height
  0.19→0.28 (no collision), sharper crystallization. (`hip2_tuck_target` added as a knob but did not
  merit on its own.)

Isolated A/B vs the keyframe baseline (promote-on-merit): precision 0.255→0.30+ (crystallizes),
keyframe_tle 0.82→0.72, falls halved then zeroed, motor_tle smoother — at a modest fwd_v cost (the
feed-forward still pushes toward the damped *posture*, not a propulsive *trajectory*).

**UI validation (the real proof):** the robot reads as ALIVE and EVOLVING (not a template), traverses
~5 m, and — stuck on a pyramid with its legs off the ground — **discovered a NOVEL limb movement to
dislodge itself and continued walking.** That self-rescue is the homeokinetic exploration drive working
as designed; an open-loop sequencer cannot do it. This is the emergent, feedback-driven direction.
Config: `the_picrawler_motor_epm_embed.json`.

**Open (not blockers):** hip2 is still passive (no rhythmic role, raw amp ~0.035); a sustained/efficient
cycle likely wants the objective to carry phase-indexed VELOCITY (a propulsive trajectory), not just
posture — the next lever.

## Reusable pieces added

- `BodyRhythmTracker` (module) — proprioception → body-gait-phase reference.
- `CPGOscillator` afferent entrainment (`entrain_topic`, aliasing clamp).
- `KeyframeGait` self-precision drive gate (`warmup_visits`, `precision_scale`).
- `MotorEPM` `coupling_fade_start/end` + `rhythm_fade_start/end` — deterministic scaffold-retirement schedules.
- `MotorEPM` `phase_joint` + `rhythm_gains`/`rhythm_offsets` + `cpg_phase_topic` — the coherent
  per-joint rhythmic drive (the REJECTED open-loop sequencer).
- `MotorEPM` `cpg_embed` (+ `embed_lr`/`embed_decay`) — CPG-as-embedding: HK controller learns a
  phase-conditioned feed-forward from the keyframe error (the emergent, validated path).
- `MotorEPM` `chassis_h`/`chassis_h_ema` diag + `hip2_tuck_target` — collision metric + femur-crouch knob.
- `BodyRhythmTracker` `raw_period`/`raw_amp` diag — per-(leg,joint) frequency diagnostic.
- `OgmaBrain` TCP `set_param` verb — live param mutation for experiments (single-connection server).
- `gait_metrics.py` + `raw_diag.py` — quantitative gait-quality + per-joint frequency diagnostics.
