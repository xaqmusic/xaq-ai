# Rung 2 — the regime vocabulary at the motor layer

> **Design doc, 2026-09-01, branch `microduck-lean-prior`.** The v2 plan's rung 2 ("a real
> EPM at the motor layer"), designed against everything the A1-v2 campaign measured — see
> the port plan §"A1-v2 — the state-prior campaign" for the evidence chain this stands on.
> Stage R0 is measured below; R1+ are module changes awaiting the operator's eye.

## 1. Why now — the three walls, and that they are one wall

The campaign ended at three independently measured walls:

1. **Mixture-poisoned identification.** One linear self-model fit to falling, fallen,
   flailing and standing data holds sign-scrambled authority; identification only succeeded
   when the harness *carved out a regime by hand* (near-upright gate + identification
   episodes).
2. **The crouch as an out-of-regime minimum.** The prior's descent stalls wherever the
   model's authority estimates stop applying — and the model cannot know its estimates are
   regime-local, because it has no regimes.
3. **Unbounded storm regrowth.** Every parametric containment of homeokinesis was consumed,
   or crushed the prior's content too (squelch / shared damping / five adaptive keys /
   split controllers — all measured). The deepest failure was the *calm key*: every
   continuous error signal is storm-coupled, so "am I standing?" could never be answered
   from inside the storm.

These are one wall: **the substrate has no discrete, predictive answer to "which dynamical
situation am I in?"** The scaffold policy converges on still balance because standing is,
for it, a state with its own quiet dynamics. CLAUDE.md §0 names the machinery this project
already owns for exactly this: *wherever a continuous stream must become a discrete,
addressable, predictive vocabulary, the EPM is the answer until proven otherwise.*

## 2. The shape

```
reality.proprio.sense1  (12-dim body/attitude bundle, calibrated origin, [-1,1])
        │
        ▼
   EPM (existing module, config only)          ── R0
   RBF encoder · GNG · dual TLE
        │  RealityToken: winner_id = REGIME, tle, transition_surp, is_novel
        ▼
   MotorEPMv2 regime socket                    ── R1..R3 (module changes, gain-0-guarded)
     R1  per-regime self-models  (A, Bx, b banks keyed by winner_id)
     R2  per-regime exploration precision      (the calm key, finally discrete)
     R3  per-regime controllers  (C banks — only if R1/R2 demand)
```

No new clusterer, no bespoke confidence scalar (§0 rules 1 and 8): the regime layer **is**
the shipped EPM, its `tle` is the confidence, its `winner_id` is the key. This is also the
duck brain's first hierarchy level — the same stacking the slow-loop notes describe, arriving
at the motor layer first because that is where the measured need is.

**What each stage answers, in the campaign's own terms:**

- **R1** makes the identification-episode carve-out *learned instead of harness-imposed*:
  the standing regime's model never eats falling data, so A(idx,·) stops de-identifying —
  the confound was always cross-regime. The harness regime gate (a hand rule at 25°) can
  then be de-scaffolded: its job moves into the vocabulary.
- **R2** is the calm study's conclusion made honest: the five failed keys were all
  continuous and storm-coupled; `winner_id` is discrete and lives on slow attitude
  states. Per-node exploration precision, learned from each node's own prior-error
  statistics — nodes that satisfy the prior anneal toward quiet, nodes that do not keep
  exploring. Nothing is hand-labeled "standing"; quiet is *earned per node*.
- **R3** exists only if measurement demands it: per-regime C banks, so the standing
  controller stops being rewritten by walking/falling learning. Not built until R1+R2
  numbers ask for it.

**Biology, briefly and honestly:** this is the classical postural-set picture — discrete
postural synergies selected by brainstem gating, with context-switched internal models
(the cerebellar story) rather than one monolithic controller re-tuned continuously. The
regime EPM plays the state-estimating gate; the MotorEPM banks play the switched models.
The analogy motivates the *shape*; the measurements above are the reason.

## 3. Conditioning (§0 rule 2 — where EPM use actually goes wrong)

- The input is `sense1`: the 12-slot attitude bundle, already centred on the **calibrated
  stand** (the origin is the scaffold's own equilibrium) and scaled to [−1,1] per channel.
  This is the §0 rule-2 work already done in the sensory-completion lever.
- RBF encoder, `proprio_state_dims = 12`, default [−1,1] ranges (honest — the channels are
  conditioned to exactly that), `projection_dim` auto (→ 96).
- **The gate before any behavioural number** (the v2 plan's own rule): node count vs the
  scatter. If the GNG says "one node" while the tilt distribution says several regimes —
  or says hundreds while the body has a handful of situations — the conditioning is wrong,
  not the idea. R0 exists to answer exactly this, instrument-only.
- Chatter levers if dwell times come out too short for regime semantics: `process_every_n_ticks`
  (regimes are slow; 50 Hz classification is not required), or an EMA on the `sense1`
  publish. Neither used until measured necessary.

## 4. Stages and gates

| stage | change surface | guard | gate (promote-or-kill) |
|---|---|---|---|
| **R0** — the vocabulary, instrument-only | adapter publishes `sense1`; graph adds the EPM; host emits `rg`/`rtle` per tick. **Nothing consumes the token.** | absent from old configs | nodes bake; count sane vs the situation count; dwell times ≥ regime timescales; node↔tilt-band purity high; transition surprise spikes at falls |
| **R1** — per-regime self-models | MotorEPMv2 `regime_topic` + (A, Bx, b) banks switched by `winner_id`; learning writes the active slot | empty topic = byte-identical | standing-regime A matches probe-J signs *without* the harness regime gate; TLE per regime < mixed TLE; then falls/upright A/B |
| **R2** — per-regime exploration precision | per-node calm learned from per-node prior-error stats | gain param 0 = off | quiet-band \|u\| falls **and stays** over an hour soak (the metric the whole gain-gap study established); no upright loss |
| **R3** — per-regime controllers | C banks | gain 0 | only if R1/R2 leave a measured residual demanding it |

Every stage: one lever, seed-averaged (n≥3 triage → n≥6), the amplitude instruments
(|u|-by-tilt, quiet-band trend), the anti-blind pair (falls **and** upright15), §3.2
read-backs (`rg` visible per tick; bank switching verified from the module's own counters).
The (d) test at the end: push a standing duck — the token must transition, drive must
re-arm, recovery must re-quiet.

**Named degenerates per gate:** a 1-node vocabulary makes R1 the status quo ante (gate:
count ≥ 3 with distinct tilt profiles); a hundred-node vocabulary starves every bank (gate:
dwell and per-node sample counts); purity measured against tilt bands the vocabulary never
saw (purity is a *check* on the vocabulary, tilt is instrumentation-only, never an input).

## 5. Out of scope

Rung 3 (expected-free-energy action selection over the regime graph) stays out until the
vocabulary and banks stand on their own numbers. The picrawler is untouched throughout
(§Coordination; G5 at both ends).

## 6. R0 measured (2026-09-01) — **PASS WITH NOTES**

Config `a1v2_r0_regime.json`: the standard EPM (RBF, 12 dims, projection 96 auto,
`max_nodes` 64, classification at 10 Hz via `process_every_n_ticks 5`) over `sense1`, on the
best standing stack, instrument-only. 15 min × 2 seeds, scored in a 2-D label space
(tilt band × motion, thresholds derived from the run's own statistics):

| gate | result |
|---|---|
| vocabulary lives | ✅ 27–39 live nodes, 4–12 baked, pruning active |
| **anchor regimes** | ✅ **a pure STANDING node self-organizes in both seeds** (share 0.23–0.25, purity 0.95–1.00), and a pure FALLEN-STILL node (0.99) |
| dwell at regime scale | ✅ mean ~0.6 s after the 10 Hz conditioning lever (was 40–80 ms at 50 Hz — the doc's predicted chatter, fixed by its predicted lever) |
| transition surprise at falls | ⚠ elevated pre-fall (ratio 1.1–1.4) but weak — sharpen later |
| mixed transitional nodes | ⚠ the largest node in each seed mixes STAND-STILL with fall-moving (~0.5/0.3); weighted top-7 purity 0.63–0.64 |
| mitosis | ⚠ never fires even at threshold 0.10 — the designed cure for mixed nodes is inert here; open conditioning item |

**R1 measured (2026-09-01) — GATES MET, PROMOTED AS SIGNAL.**  Build: `regime_topic` +
per-regime (A, Bx, b, TLE) banks in MotorEPMv2, L.A/Bx/b as the active working copy swapped
on winner change, warm-start from the incumbent, boundary sample dropped at switches, banks
engaged **after** the identification phase (engaged-during-babble scattered pair-writes to
init noise — measured), plus `babble_owns_a` (one-owner-per-estimand extended past warmup:
closed-loop LMS erodes the babble-identified authority — measured at 900 s — so the babble's
paired-difference estimator owns A permanently; LMS keeps b and Bx).  Unit test
`RegimeBanksUnmixOpposedAuthorities`: banks provably learn sign-opposed authorities no
shared model can hold (9/9 suite green).

| R1 gate | result |
|---|---|
| §3.2 consumer | ✅ ~900 switches/600 s; samples across all 6 banks |
| identification WITHOUT the harness tilt gate | ✅ standing-bank A holds **4/5 probe-J signs at 900 s** (bankless ungated control: 2/5 scrambled) — the hand rule's job, learned |
| per-regime TLE < mixed | ✅ standing bank 0.45 vs 2.0–2.7 (transitional) and ~1.0 (mixed) |
| behaviour (n=6 × 600 s) | ✅ upright **0.66±0.12 vs 0.58±0.10** (campaign best), tilt 21.4 vs 23.9, brain 62.5 % vs 56.5 %, falls tie |

**R2 measured (2026-09-01) — TIE at 600 s, GATE FAILED at the hour; default-off.**
Build: `state_prior_calm_mode 1` — the calm target becomes the ACTIVE BANK's prior-error EMA
against the worst across banks (discrete, regime-local, storm-proof by construction), through
the existing ratchet; per-bank `sp_err` statistics; unit test
`RegimeKeyedCalmQuietsTheSatisfiedRegime` (the satisfied regime anneals, the violated one
keeps drive, nothing hand-labeled — 10/10 suite green).  On the duck: behaviourally a TIE at
n=6 × 600 s (11.5±2.9 vs 12.9±3.5 falls, upright 0.58±0.19 vs 0.66±0.12); at the hour-soak
gate the squelch ENGAGED once — quiet-band |u| 0.40–0.47 for 25 minutes, the campaign's
first sustained quiet — then re-inflated, and did not reproduce (1 of 6 soak runs).  Two
mechanisms recorded: the prior's attitude columns in shared C grow without bound under the
squelch (to 103; the split's honest-G fix, ported to shared mode, did NOT tame it — the
growth also happens unsquelched), and the key's engagement needs the standing bank's error
to genuinely separate, which needs sustained quiet standing first — the chicken-egg one
level up.  **Re-use contexts:** (a) an attitude-column governor (the split architecture's
unfinished business); (b) retry once identified standing stretches lengthen.

**The vocabulary-stability audit (2026-09-01, the operator's prune-cascade observation).**
The operator asked whether the picrawler's watched pattern — long pruning sequences after a
perturbation, baked nodes included, then relearning the same nodes on recovery — could be
eating our regimes.  **Mechanism confirmed in code**: the GNG's health-death sweep iterates
ALL nodes, baked included (its docstring says it "replaces the binary baked/unbaked utility
system" — the header's "baked = never pruned" contract was silently void), so a long
absence (fall, rescue, crouch) starves an earned node to death in minutes.  **Fingerprint
measured on the duck**: 41 regime ids over an hour with 25 dead; the standing crown itself
migrating 1→16→29 — orphaning the bank map, sending the crown's data to the overflow bank,
and resetting R2's statistics.  Both of R2's fragilities in one stroke.

**Fix, guarded**: `health_death_spares_baked` (GNG config + EPM param, default false —
picrawler byte-identical until the operator opts in) + `min_insertion_error` 0.06 on the
regime EPM (near-duplicate crown-stealing).  Measured: ids 41 → 10, deaths 25 → 2 (seed 1;
seeds 2–3 stabilise less — 34–38 ids — noted as seed-dependent conditioning).

**And the stack entered the probe band.**  R1-stable, n=6 × 600 s: **falls 7.5 ± 5.1/min
(the hand-gain probe's own 7–8 band), upright 0.73 ± 0.24 (campaign best), tilt 15.7°,
brain share 77 %** — versus the churn-vocabulary incumbent at equal falls and upright 0.56.
Re-running that incumbent also surfaced a silent cross-arm gift: the shared-mode honest-G
fix (committed during R2) had halved the fixed-squelch stack's falls (12.9 → 7.6) — the
control re-run is what caught it.  Known residuals: strong seed bimodality (crouch-prone
seeds drive the ±0.24), the R2 adaptive key still never engages (fixed squelch does the
work), vocabulary stability varies by seed.

**Why the notes do not block R1:** R1's critical consumer is the *standing* bank, and its
key (the pure standing node) exists in both seeds at high purity and high share. Mixed
nodes blur only the transitional banks — which today do not exist at all, so their floor is
the status quo. The two ⚠ items are recorded as R1-era conditioning work (mitosis
triggering, transition-surprise sharpening), not gate failures.

---

## 7. Earned consolidation (2026-09-01) — **STANDING, PERMANENT, 3/6 SEEDS**

The seed-robustness pass found the last disease and its cure in one day.

**The disease: standing found, then destroyed.**  Per-seed 30-min diagnosis on the
R1-stable stack: seeds find the standing basin in minutes (seed 4: up15 0.99 in bucket 0)
and *continued learning erodes it* (0.99 → 0.63 by bucket 6).  `--freeze-after 300`
(a diagnostic hard freeze, added for this test) proved the mechanism: the same seed frozen
at 5 min holds 0.93 for the rest of the run.  The destroyer is the learning itself — the
homeokinetic terms keep sharpening sensitivity out of a solved posture.  A hand-picked
freeze time is a scaffold, so the shipped form is the GNG's baking principle at the
controller level:

**`consolidate_gain`** — every learning rate (model, Bx, dC, h, prior lw) is scaled by
`1 − gain·c`, where `c` ramps up (τ ≈ 10 s) only while the state prior is satisfied
**and** no fall for 30 s, and re-arms fast (τ ≈ 2 s) when either breaks.  The
satisfaction reference took four designs: short/long EMA ratio (never engages when
standing is found *first*), smoothed peak (dilutes), decaying peak (collapses during the
quiet it should protect).  What survived is a **fixed 0.15 threshold on the prior-error
EMA** — a fraction of a unit-scaled channel, not a constant tuned to a signal (§5.5).

**Measured** (`a1v2_r3_consol.json` = r1_stable + consolidate_gain 1.0, 30 min/seed):

| seeds | outcome |
|---|---|
| 4, 5, 6 | **0.0 falls/min, up15 1.00, five straight 5-min buckets, cons=1.00** — permanent standing |
| 1, 3 | never found the basin; cons=0.00, behaviour unchanged (the gate cannot consolidate a non-solution) |
| 2 | crouch attractor; cons=0.00, unchanged |

Gain-0 verified live: the r3 and r1-stable runs are byte-identical until the first
engagement (~t=277 s, exactly 30 s + ramp after the last rescue), then diverge.  Late
quiet-band posture: joint sd ~0.001 rad, tilt sd 0.06° — the near-motionless stance the
operator asked the amplitude study to converge to.  Committed as 4f7b0c2; validation
10/10 unit tests, all gates, v1↔v2 identity IDENTICAL.

**The remaining gap** is basin *finding*, not basin *keeping*: seeds 1/3 never enter
standing post-identification, seed 2 settles into a crouch.  Candidate levers: longer or
repeated identification, basin-entry assistance from the standing bank's own model, or
accepting per-seed convergence variance and measuring the finding rate at n≥20.

---

## 8. The stand-tall drive (R4 family, 2026-09-01) — five iterations, REFUTED AS A PARTIAL-POSE PULL

The operator's read of the standing video: the stance is slump-shaped, and the drive
should be toward standing tall.  First measurement split that in two: the stance is
**dynamically balanced, not a passive slump** (servos held at the achieved pose with the
brain removed topple in 30 s, same as the STAND keyframe) — but the *pose* is a crane-head
crouch, because nothing in the attitude-only prior says tall.

Lever: state-prior targets on the stand-calibrated head-CoM slots (sense 10/11).  Five
arms, each killing one mechanism (full chain in the 729b0cc commit message): gate
poisoning → the race (pull costs pre-consolidation robustness; the 30 s calm window
misses) → listing side-channels (calm exemption hands HK a full-gain channel) → dormancy
itself (a waking pull pushes a consolidation-frozen loop out of a basin it never learned
to widen, 3/3 seeds, 3–28 s) → h windup during pre-standing chaos (hmax 2.67).

**What stands after the smoke clears:**

1. **R4b seed 4 is the existence proof**: permanent 1.00 upright, cons=1.00, with the
   full head-CoM C-pull live — standing and a reach objective can coexist when learning
   *co-adapts around the pull from tick 0*.
2. **The reach machinery works** — seeds reached the origin (6.5 mm) — and the roles are
   confirmed a third time: C balances, h reaches.
3. **The verdict's real content is about the TARGET, not the mechanism**: head-at-origin
   over slump legs is not a balanceable configuration (seed 4 reached it twice and fell
   20/min there).  The scaffold balances that head position with *extended legs under
   it*.  A reach toward a partial pose deforms the body into unproven territory; the
   coherent target is the whole-body stand pose the probe already proved balanceable.

**Re-use context:** whole-body reach (legs + head toward the calibrated stand pose, both
modules) with the arm-shape the five verdicts select — C-pull live from tick 0, h gated
by consolidation while the spared C-pull keeps the reach direction plastic (mode-3
semantics, unbuilt).  Alternative shape: tall standing as its own regime with its own
bank and consolidation, entered from the scaffold's settle pose rather than deformed into
from the slump.

---

## 9. THE DUCK STANDS TALL (R7→R8, 2026-09-01) — capability proven, 1/6 permanent

The whole-body-reach directive resolved through a chain of eight more measured arms after
§8 (controller banks R6a–c: interesting phenomena — a new stander, tall-hovering — but
refuted for tall; the full verdicts live in commits e7c64a2 and c647d02).  The three that
mattered:

1. **The calm window was the binding constraint** (R7): with `consolidate_calm_ticks` 500
   instead of the picked-not-derived 1500, the whole-body C-pull arm holds a TALL stance —
   0.92–0.96 upright, pose-distance 0.072, tilt 4° — for 90 straight minutes.  The 4-hour
   soak then showed why consolidation must catch during the good window: the destroyer
   reverses the trend after hour one (falls 3.2 → 10.7/min).
2. **The ratchet's losses were bookkeeping, not physics**: legacy c-decay wipes e⁻⁷ per
   fall (topple + frozen rescue + the whole calm window all decay).  Symmetric slow decay
   (R7b) anneals adaptation during real chaos — falls worse.  The EMA-keyed three-state
   (R7c v1) misses 1 s topples — c held 0.94 through 21 falls/min.  The working form
   (v2): **ramp when satisfied-and-calm, decay only while the INSTANT gate error exceeds
   2× the satisfaction fraction (a topple crosses within ~0.3 s), hold otherwise.**
3. **R8** = C-pull only (mode-3's hr wound at high c) + calm 500 + three-state v2.
   Seed 2 — the control stack's crouch degenerate, which never stood at all — falls
   7.7 → 2.7 → 0.8 → 0.1 → **0.0 for the final eighty minutes**, upright 1.00, cons 1.00,
   pose 0.065–0.071, tilt ~1°, |u| converging 0.88 → 0.66.  Video sent.

**Scale of claim** (§3.3/§3.7): the capability is LOUD — tall, permanent, earned, with
amplitude convergence — on one seed, exactly reproduced in the battery.  Seeds 1/3/4/5/6
hover TALL when upright (tilt 3–4.4°, pose 0.10–0.15: the pull shapes every seed) but
fall-cycle at 15–22/min and never string the gaps.  **The frontier is now singular: the
same basin-finding/race variance that leaves the slump stack at 3/6.**  Also killed
cheaply on the way: the exploration-noise-tipper hypothesis (falls UP without dither —
it excites the ongoing identification).

---

## 10. CONTROLLED STANDING (R10→R12b, 2026-09-01) — the oscillator found, named, and rested

The operator watched the tall stander and saw it: dynamically stable but oscillating
constantly.  Quantified: a coherent **~5.5 Hz whole-body limit cycle**, 12.5 mrad/tick of
joint motion — 134× the consolidated slump's stillness — with the head (neck_pitch,
head_roll) carrying most of it.

**What it was NOT** (every probe from the same brain snapshot — the checkpoint machinery
built this session made each one a controlled 30-minute experiment): not the spared
prior's churn (null), not the exploration dither (removing it made things worse, twice —
it is load-bearing), not raw gain (the ctrl_damping 2e-5 hunt shed C from 40–60 to
3.6–10.6 with standing intact, and the cycle barely moved), not the calm exemption's
scope (null), and not the missing robotd servo filter (three arms: the lag denies the
basin entirely at learning time; re-use context — lag-aware learning).

**What it was**: command-self-feedback through C's ACT (efference-copy) columns — the
controller re-exciting itself through its own last action at one-tick lag.  The lesion
proved it in ten minutes: zero those columns and the cycle collapses **160×** (11.1 →
0.069 mrad, quieter than the slump) with the tall stance intact.  And a lesion cannot
stick: the spared prior regrows the columns within minutes.

**The mechanism**: `consolidate_rests_act` — the command's act input scales by
(1 − gain·c).  Full efference while learning and during chaos; cut at earned stillness;
restored the moment c collapses.  Plus the completing config move (R12 measured the trap):
the prior must rest too (`spares` 0), or its writes inflate act columns the rested command
cannot see and the first c-dip detonates them.  At earned stillness **nothing writes and
nothing self-excites; the stance freezes whole** — and stays fully reversible through the
v3 gate.

**Measured, R12b, one hour**: 0.12–0.43 mrad/tick, upright 1.00, pose 0.062–0.070, tilt
2–5.6°, |u| 0.30 (halved), one wobble self-caught and re-quieted.  Video sent (before/
after).  Checkpoint `duck_controlled_brain_s2.json`.  Scale of claim: one seed, one hour
(3 h soak in flight); the from-scratch pipeline (find → consolidate → hunt → rest) and
seed-robustness are the open items, same as ever.

**§10 addendum — the whole freeze (R12c).**  The 3 h soak of R12b caught the last leak:
the gain hunt's unscaled decay kept eroding a frozen C that was now too stable to ever
re-arm learning (90 quiet minutes, then collapse — the R9c disease in slow motion).
R12c adds `ctrl_damping_lr_scaled` 1: at earned stillness nothing writes AND nothing
decays.  **Measured, three hours: 0.083–0.090 mrad/tick (the slump's own stillness,
still improving), ZERO falls, upright 1.00, pose 0.068, tilt 1.4→1.0°, |u| 0.34.**
`a1v2_r12c_whole.json` is the promoted controlled-standing stack;
`duck_controlled_brain_s2.json` is its checkpoint.

## 11. THE (d) PUSH TEST (2026-09-02) — the loop re-infers; the stance does not catch

**Setup.** `--push` wired into `run_with_brain` (the scaffold's `--hold` schedule, with
shoves delivered only on brain-driven ticks and each one reported as *caught by the brain*
or *rescued by the scaffold*; the JSONL now carries the active force in `push` and each
MotorEPM's consolidation in `cons`; `mj_host/tools/push_report.py` reads it).  A world-frame
force on the trunk, rotating +x, +y, −x, −y, every 60 s from 30 s — six shoves per 420 s
run, each judged in a 4 s window (recovered = tilt < 5° held 0.4 s).  Two families: an
impulse (0.1 s) and a sustained lean (1.0 s).  Three stances, same body, same seed 2:
the **controlled** checkpoint (R12c, `duck_controlled_brain_s2`), the **tall** checkpoint
(R8, `duck_tall_brain_s2`: C norms 40–60, the 12 mrad oscillation still on), and the
**scaffold** (`--hold`, the STAND keyframe).  14 runs, 78 shoves.  One seed — the only
tall stander — so a signal, not a finding.

### 11.1 The envelope

| shove | N·s | controlled: caught / rescued | tall: caught / rescued | scaffold |
|---|---|---|---|---|
| 0.5 N × 0.1 s | 0.05 | 5/5 (peaks 1–2°, one 26°) | — | — |
| 1 N × 0.1 s | 0.1 | 5/6 (peaks 1–4°) | 5/6 | — |
| 1 N × 0.2 s | 0.2 | 3/6 | — | — |
| 1.5 N × 0.1 s | 0.15 | 3/6 | — | — |
| 2 N × 0.1 s | 0.2 | 3/5 | 1/6 | peaks 0.4–7.9°, 6/6 |
| 3 N × 0.1 s | 0.3 | 0/6 | 0/6 | peaks 0.5–4.8°, 6/6 |
| 5 N × 0.1 s | 0.5 | 0/6 | 0/6 | peaks 3–11°, 6/6 |
| 7 N × 0.1 s | 0.7 | — | — | 3/6 knocked over |
| 10 N × 0.1 s | 1.0 | — | — | 6/6 knocked over, 6/6 stood back up |
| 0.5 N × 1.0 s | 0.5 | 3/6 | — | — |
| 1 N × 1.0 s | 1.0 | 2/6 | — | — |
| 2 N × 1.0 s | 2.0 | 0/6 | — | — |

The learned stance is knocked over at **0.15–0.2 N·s** (direction-dependent: fore-aft goes
first, one lateral side is caught at 1.5 N); the scaffold at **~0.7 N·s**.  The tall and
the controlled checkpoints have the **same envelope** at 5–10× the controller gain, so the
gain hunt and the rest did not shed a catch — there was never one to shed.

### 11.2 What a topple looks like (controlled, 2 N, forward)

| t after push | 40 ms | 200 | 360 | 440 | 520 | 600 | 680 | 840 |
|---|---|---|---|---|---|---|---|---|
| tilt | 1.8° | 6.9° | 12.4° | 16.1° | 21.3° | 29.2° | 42.4° | 79.6° |
| \|u\| | 0.36 | 0.36 | 0.37 | 0.38 | 0.41 | 0.48 | 0.60 | 0.35 |

Eight hundred milliseconds from shove to flat — forty brain ticks — and the command does not
move through the first twenty of them.  It rises only past 21°, where the learnable-regime
gate is about to close.  At 1 N the body leans to 3.7° and **stays within 0.2° of it for
over a second**: the "catch" is the support polygon, not the controller.  The stance is
statically stable and nothing else.  The four gated attitude elements — gravity x/y and the
pitch/roll rates — are in the state prior and in the blanket; the consolidated C simply
carries no gain on them at the scale a catch needs.

### 11.3 The loop re-infers — and this is the (d) result

Every rescue ran the same cycle, **20 of 20** clean cycles across both checkpoints: handoff
→ c ×0.37 (the v3 gate's reset event) → 10 s calm → ramp → **c ≥ 0.95 at +38 s** (37.3–39.1)
→ stillness back at 0.06–0.08 mrad/tick, the tall pose (0.065–0.075) intact.  The 5 N run
is six of these in a row with nothing else happening: six shoves, six falls, six re-earned
stances.  And when the stance was *destroyed* — a wake that cascaded into the pre-
consolidation chaos (c → 0, 10–17 falls a minute, |dq| 10–17 mrad) — the brain re-found and
re-earned it inside 57–100 s in **4 of 6** cascades (the 1.5 N run came back at 0.034
mrad, quieter than it left, and caught its next three shoves); 2 were still cascading at
420 s (3 N; 1 N sustained).

The cascade itself: **6 of 10** controlled runs, **0 of 4** tall runs.  Two hypotheses,
both cheap on the same checkpoint and both untested: (i) the controlled wake is a regime
change — the rested efference returns at 64 % into a stance frozen without it (R8 never
rests, so its wake changes nothing); (ii) slow topples write poison — a 2–3 N shove spends
~500 ms in the 12–25° band with learning re-armed, a 5 N knock-down 200 ms, and the 5 N run
had no cascade.  Also seen and not diagnosed: **late falls** 8–27 s after sub-2° shoves
(0.5 N at 300 s → fall at 327 s; 1 N at 120 s → fall at 128 s), in a stance that stood
three unperturbed hours; the natural reading is a shifted contact state the frozen
controller cannot re-centre.

### 11.4 Verdicts (one seed, one checkpoint per stance — signal, not finding)

- **(d) at the consolidation loop: WORKING.**  Perturb → c collapses → plasticity wakes →
  stance re-found → re-earned → stillness returns.  Re-consolidation is metronomic (38 s)
  and full re-inference after destruction is 4/6.
- **(d) at balance: NULL, in context.**  No active catch; the envelope is the support
  polygon's.  Re-use context: the catch's *gradient* — the brain learned to stand in a world
  that never leaned it (dither is millirad; real leans happened only in falls, past the
  learning gate).  Not a gain question (11.1) and not a sensor question (11.2).
- **The wake cascade: PARTIAL.**  Re-use: test (i) and (ii) above — `consolidate_rests_act
  0` on the controlled checkpoint at 2 N, and a learning hold-off during the first ticks of
  a chaos decay.
- **Harness: `--seed` is a no-op on `--load-brain`** — the RNG state is restored with the
  brain; runs at seeds 3 and 4 are byte-identical to seed 2.  Variance on a checkpoint can
  only come from the perturbation schedule.  (A §3.2 catch: the "seed variance" arm was a
  tautology.)

### 11.5 The fork — how does a catch get learned?

The rewrite rule's three questions, answered by 11.1–11.2: the error exists (the attitude
prior), the sensor exists (gravity + rates, in the blanket), the module owns both.  What is
missing is **data in the regime where the catch acts**: leans of a few degrees while the
controller is still plastic.  Two directions, the operator's call:

- **A world with wind (no code).**  Sub-topple shoves (0.5–1 N, 0.1 s) throughout the
  from-scratch pipeline — `--push 0.7 --push-every 8 --push-from 0` on
  `a1v2_r8_tall.json` — so the prior's descent sees lean excursions at the scale a catch
  needs while C still writes.  One lever, A/B against no wind, judged on the 11.1 envelope
  of the resulting stance.  Risk: the from-scratch pipeline is 1/6 seed-robust, so the A/B
  is noisy; and shoves are calm-gate neutral only below the 0.30 instant threshold (check
  with the `cons` trace before trusting a null).
- **A lean-aware wake.**  Let a *lean* re-arm plasticity the way a fall does, at a gentler
  cost — so the consolidated stander learns from the shoves it survives instead of only from
  the ones that flatten it.  A design discussion first: it touches the v3 gate.

## 12. THE WIND A/B AND THE RATCHET TAX (2026-09-02) — the lever is null; the pipeline was broken and is repaired

The operator chose §11.5's option A: sub-topple shoves throughout learning, on the
from-scratch pipeline, A/B against no wind.  Their caveat going in: the brain cannot step,
so the only catch available is in place.  Three results came out, in this order.

### 12.1 Commands (every run here is deterministic; a live window of the same command is the same run)

```sh
# from scratch, the R8 tall stack (the "base" arm); the wind arm adds the three --push flags
mj_host/build/ogma_mjhost --brain --graph mj_host/configs/a1v2_r8_tall.json --secs 5400 --seed S \
    --ident-every 12 --ident-until 3000 --save-brain base_sS.brain.json \
    [--push 0.7 --push-every 8 --push-hold 0.1 --push-from 240]      # ident ends at ~200 s
# the repaired pipeline (12.3) is the same with a1v2_r13_tax001.json and --secs 7200
# the envelope of a saved brain: --load-brain X.brain.json --secs 420 --push N --push-every 60 --push-from 30
# live:  ./mj_host/run.sh brain <the same host args>      replay:  tools/duck_viewer/view.py replay run.jsonl --fast
```

### 12.2 First battery — NULL against a broken baseline

Two arms × six seeds × 90 min under the current code.  **Zero of six baseline seeds
consolidate.**  Seed 2 — the recorded tall stander of §9 — reaches c 0.39–0.44 at 20–40
min with 3 falls/min and then erodes (falls 7.6 → 3.6 → 3.1 → 3.8 → 4.4 → 5.6 → 6.2 → 6.9
→ 8.0/min; c → 0.04).  Seeds 1/4/5/6 fall-cycle at 12–22/min, seed 3 sits in a
lying-down attractor (upright 0.25) — all as the §9 battery recorded for them.  The wind
arm on seed 2 was worse (c peak 0.26): a 0.7 N × 0.1 s shove topples the *unconsolidated*
stance 59 % of the time (287 of 483 delivered), which is the operator's caveat measured.
A §3.2 case 4 (baseline validity): nothing about the lever can be read from this battery.

### 12.3 The ratchet tax — the from-scratch pipeline was silently broken since 6fac760

The R8 record was made under ratchet v2.  v3 (6fac760, `consolidate_hold 1` changed in
place) added the harness reset event as a chaos trigger: **every rescued fall decays c for
100 ticks at `consolidate_down_rate`** — ×0.37 at the 0.01 default — on top of v2's
instant-error decay.  The ramp is +0.002·(1−c) per tick after a 10 s calm window, so at
one fall per 20 s the tax-then-ramp fixed point is c ≈ 0.4: **exactly seed 2's plateau.**
v3 made the *resumed* stander chaos-aware (R9c) and killed the *race* — the from-scratch
pipeline has not been run since, and both checkpoints predate the change.

**The repair is config-only** (`a1v2_r13_tax001.json` / `a1v2_r13_tax003.json` = R8 +
`consolidate_down_rate` 0.001 / 0.003; per-fall tax ×0.90 / ×0.74; chaos decay τ 20 s /
6.7 s):

| seed 2, falls/min per 10 min (c) | |
|---|---|
| tax 0.001 | 7.9 → 2.8 (0.84) → 1.2 (0.95) → 0.2 (0.99) → 0.1 → **0.0 → 0.3 → 0.0 → 0.0 → 0.1 → 0.0 → 0.2, c 1.00 — permanent from 30 min**, 128 rescues in 2 h, all early |
| tax 0.003 | 8.0 → 4.5 (0.51) → 2.7 (0.72) → 2.2 (0.78) → 1.5 (0.88) → 3.2 (0.66) → 4.8 → 6.5 → 6.3 (0.26) — climbs, then loses it |
| tax 0.01 (v3 default) | the 12.2 plateau at c 0.4 |

The §9 trajectory (7.7 → 2.7 → 0.8 → 0.1 → 0.0) is back at 0.001.  Seeds 1/3/4/5/6 are
**byte-identical across all three rates** — they never earn any c, so the tax never acts
— and seed-robustness stays 1/6, the same race gap §9 recorded.  R13 is the from-scratch
find-and-consolidate stage; the resumed R12c stack keeps the v3 default (its rest trio
relies on a strong wake).  Whether the two stages hand over cleanly is 12.6.

### 12.4 The wind on the repaired pipeline — it stands, it never rests, it does not catch

Seed 2, R13, 2 h, wind at 0.3 / 0.5 / 0.7 N × 0.1 s every 8 s from 240 s.

| | delivered | toppled | c by 30 min | c, hour 2 | falls/min, hour 2 |
|---|---|---|---|---|---|
| base | — | — | 0.99 | 1.00 | 0.0–0.3 |
| wind 0.3 N | 801 | 16 % | 0.95 | 0.96 → 0.80 | 0.1 → 4.0 |
| wind 0.5 N | 745 | 39 % | 0.89 | 0.55–0.75 | 4.6–6.6 |
| wind 0.7 N | 809 | 19 % | 0.83 | 0.92 → 0.75 | 1.0 → 3.9 |

The wind does not stop the race — all three arms consolidate a few minutes behind the
base — but a stance under wind never fully rests: even 0.3 N topples the plastic stance
one shove in six, each topple taxes c and re-arms learning at 4–20 %, and the stance
erodes through the second hour (the destroyer, slowly).  **In-run catch fraction per 20
min**, 0.3 N: 41 → 94 → 97 → 97 → 90 → 71 %; 0.5 N: 36 → 77 → 73 → 57 → 55 → 58 %;
0.7 N: 53 → 85 → 89 → 86 → 87 → 73 %.  The rise is the stance being found (a found stance
absorbs 0.3 N); the mean peak tilt of caught shoves never tightens (3–5° throughout); the
late decline is the erosion.  **Envelope of the saved brains** (caught / delivered):

| shove | base | wind 0.3 | wind 0.5 | wind 0.7 |
|---|---|---|---|---|
| 1 N | 6/6 | 4/5 | 2/3 | 0/4 |
| 1.5 N | 3/6 | 1/6 | 1/4 | 3/6 |
| 2 N | 1/6 | 0/4 | 2/5 | 1/6 |
| 3 N | 0/6 | 0/5 | 0/5 | 1/6 |

Unchanged: the threshold is 0.15–0.2 N·s for all four, the §11.1 envelope.

### 12.5 Verdicts and the diagnosis

- **A world with wind: NULL** (from-scratch R13, seed 2, 2 h, 0.3–0.7 N × 0.1 s every 8 s;
  baseline healthy).  Leans while plastic do not produce a catch; they keep the brain
  awake at low plasticity, and a brain kept awake erodes.  Re-use context below.
- **The ratchet tax: a §3.2 harness finding, repaired.**  R13 restores the from-scratch
  race on the one seed that ever won it.  The R8 configs are unchanged; the record in §9
  stands for the code it was measured on.
- **Option B (a lean-aware wake) is disfavoured by the same data**: the wind runs *were* a
  low-plasticity wake with continuous leans, and nothing was learned from it.

*Why the leans taught nothing* — the diagnosis to test next, not a verdict: the pull that
shapes C is one linear descent on **ten pose elements and four attitude elements at equal
weight**.  A catch is a movement *away* from the pose target in the service of the
attitude target (hips, ankles, and a head that is 38 % of the mass), and the pose term is
under direct control while attitude is reached only through it; locally the pose term
wins, so a lean is answered with stiffness, which is what §11.2 saw.  Two cheap
discriminators: (i) **precision** — keep the four attitude elements' descent live at c = 1
while the pose elements rest (the rest trio rests the whole prior today), or weight the
attitude subset above the pose subset; (ii) **sensitivity** — read the identified model's
action → attitude columns: if babble on a stiff stance identified "actions barely move
attitude", the catch has no gradient however often the world leans it.  Both are
gain-0-guarded knobs on the same module; both are the operator's fork.

### 12.6 The from-scratch pipeline, end to end — CLOSED

The open item since §9.  From the R13 seed-2 brain at 2 h (c 1.00, C norms un-hunted, the
12 mrad oscillation on), two hand-overs to the rest stack, 30 min each stage:

| hand-over | falls | c | \|dq\| mrad/tick |
|---|---|---|---|
| R13 → R12c directly (1 h) | 209 rescues | 0.13–0.52 | 14.6–16.5 — the stance is lost (the R12 trap: the efference cut lands on an un-hunted C) |
| R13 → R11 hunt (30 min) → R12c (30 min) | **0 and 0** | 1.00 throughout | 12.0 → 10.9 through the hunt, then **0.1** under R12c |

**find (R13, 2 h) → hunt (R11, 30 min) → rest (R12c): zero falls after the find, and the
controlled checkpoint's stillness (0.085) reproduced from nothing.**  Three sim-hours,
about three minutes of wall clock.  The hunt is not optional: it is what makes the rest
safe.  The resulting brain is `mj_host/checkpoints/duck_pipeline_s2.json`.

```sh
H=mj_host/build/ogma_mjhost; C=mj_host/configs
$H --brain --graph $C/a1v2_r13_tax001.json --secs 7200 --seed 2 --ident-every 12 --ident-until 3000 --save-brain find.json
$H --brain --graph $C/a1v2_r11_hunt.json    --secs 1800 --seed 2 --load-brain find.json --save-brain hunt.json
$H --brain --graph $C/a1v2_r12c_whole.json  --secs 1800 --seed 2 --load-brain hunt.json --save-brain rest.json
```


## 13. THE ATTITUDE PRECISION TEST (2026-09-02) — REGRESSION, and the model says why

The operator chose §12.5's discriminator (i).  One gain-0-guarded knob,
`state_prior_gate_weight` (MotorEPMv2; 1 = byte-identical, verified on the r3 reference;
21/21 unit tests, the new one asserting the guard, the effect, and inertness without a
gate subset): the four attitude elements — gravity x/y and the pitch/roll rates, the
first `consolidate_n` prior indices — descend at W× the pose elements' rate, C and h
alike.  R14 = R13 + W ∈ {3, 10} (`a1v2_r14_attw3.json`, `a1v2_r14_attw10.json`).

### 13.1 From scratch, six seeds, two hours each

| weight | seeds standing at 2 h | seed 2 | chaos |
|---|---|---|---|
| 1 (R13) | **1/6** (seed 2, permanent from 30 min) | falls 7.9 → 2.8 → 1.2 → 0.2 → 0/min, c 1.00 | 13 mrad/tick |
| 3 | **0/6** | 11 → 17 → 16 → 16 … 16/min, c 0.00 throughout | 16–22 mrad/tick |
| 10 | **0/6** | 12 → 15 → 14 → 16 → 19 … 13/min, c 0.00 throughout | 14–22 mrad/tick |

No seed at either weight earns any consolidation in two hours, seed 2 included, and the
chaos is more energetic than at weight 1.  The R4b/R5 wall (§8): a stronger pull before
the basin is found loses the race.

### 13.2 After the basin is found — resume the weight-1 stander under the weight

Seed 2's R13 brain (c 1.00, standing 90 min), one hour, R13 keeps the prior live at c = 1
(`consolidate_spares_prior` 1), so the weight acts at once:

| arm | first 10 min | hour |
|---|---|---|
| weight 1 + wind 0.5 N (control) | 0.2 falls/min, c 0.99 | holds 40 min, erodes to c 0.20 (§12.4 again) |
| weight 3 | 10.6 falls/min, c 0.25 → 0 | 11–15/min, c 0, \|dq\| 16–20 |
| weight 3 + wind | 8.7/min, c 0.35 → 0 | 10–16/min, c 0 |
| weight 10 | 17.2/min, c 0.09 → 0 | 12–19/min, c 0, \|dq\| 18–22 |
| weight 10 + wind | 18.0/min, c 0.09 → 0 | 13–21/min, c 0 |

The stance is destroyed within minutes at either weight.  The wind runs delivered 4/177
and 37/274 caught shoves against the control's 339/420.

### 13.3 Discriminator (ii), read from the saved brain — CORRECTED 2026-09-03

**The reading first written here was wrong.**  The brain snapshot flattens matrices in
Eigen's column-major order and the analysis reshaped them row-major, so the "row norms"
grouped the wrong elements.  The corrected reading (the sanity check: the first entries of
the flat A are motor 0's authority over state 0, its own position):

| A row norms, seed 2's R13 brain | gravity x | gravity y | pitch rate | roll rate | joint positions |
|---|---|---|---|---|---|
| legs (5 motors) | 0.024 | 0.011 | 0.089 | 0.044 | 0.05–0.09 |
| head (4 motors) | 0.019 | 0.021 | 0.011 | 0.019 | 0.02–0.03 |

**The lean channel is identified** — at about a quarter of the pose rows' authority, with
the rates at a comparable size.  The claim below it in the first version ("the model
knows a joint move barely moves the gravity vector", rows 20× below pose) is retracted,
and with it the diagnosis that the catch had no gradient at the model.  What the
corrected layout shows instead (§14.3): in every standing brain the controller's
attitude columns are *restoring* — the one-step loop gain Σⱼ A(lean, j)·C(j, lean) is
negative on all four attitude elements, the columns aligned with −A at cosine 0.6–0.9 —
and the wind brains grew that gain three to seven times larger without a catch.  The
direction was never missing.  What is missing is measured in §14.4: the identified lean
rows are *right for the left leg only*.

The R14 regression (13.1, 13.2) stands as measured; its reading is now the phase one —
more attitude gain into a loop already at its limit cycle (§10) — not the model one.

### 13.4 Verdicts

- **Attitude precision as a fixed weight: REGRESSION** (W 3 and 10; from scratch, 6 seeds
  × 2 h, 0/6 vs 1/6; and on a found stance, collapse within 10 min at both weights).
- ~~**Discriminator (ii) answered: the catch has no gradient at the MODEL.**~~  **RETRACTED
  2026-09-03** — a matrix-layout error in the reading (13.3, corrected).  The lean channel
  is identified and the attitude feedback is restoring; the defect is one-sided
  identification (§14.4).
- **Re-use context** (what would justify retrying attitude precision): an identification
  that is right for the whole body (§14.4–14.5), then the weight is the same experiment on
  a controller whose attitude gradient is not one-sided.  (The first version of this bullet
  proposed temporal depth on the attitude rows via `model_trace`; §14 measured that
  direction — the horizon was not the defect.)

## 14. THE IDENTIFICATION SCHEDULE (2026-09-03) — the duck stands on every seed, and catches

The operator chose "the model_trace test" from §13.4.  In the R13 configuration the state
model A is owned by the paired-difference babble for the whole run (`babble_owns_a` 1), and
`model_trace` only filters the model's *prediction* input, so the bare arm would have been
a §3.2 dead-code tautology; the faithful form of "temporal depth on the attitude rows" is
the identification pulse itself.  Four arms, then the reading that found the defect, then
the arm that fixed it.  All from scratch, R13 as the base, 2 h, six seeds unless stated.

### 14.1 The pulse-horizon arms — all REGRESSION

| arm | change | identification | race |
|---|---|---|---|
| R15 | `babble_hold` 25 (500 ms), ident 50/12500 | ends 823 s; **250 falls in 250 pairs** | **0/6**, 18–26 falls/min, c 0 |
| R16 | R15 + `model_trace` 0.04 | A byte-identical to R15 (the trace never touches a babble-owned A) | **0/6** |
| R17 | R15 at `babble_scale` 0.1 | **250 falls in 250 pairs — at any amplitude** | not run |
| R18 | `babble_hold` 12 (240 ms), ident 24/6000 | ends 483 s; 19 falls; TLE 0.85 vs R13's 0.95 | **0/6**, 15–18 falls/min, c 0 |
| R13 | hold 6, ident 12/3000 | ends 202 s; 0 falls | 1/6 (seed 2) |

R15's falls equal its pairs because the body has no passive equilibrium (§A1-v2): an idle
one-second pair topples on its own, so the long window measured the fall, not the action.
The identifiable horizon is bounded by the body's topple time; R18 sits inside it,
identified cleanly, predicted better — and still lost the race.  The horizon was not the
defect.

### 14.2 A layout error in §13.3, and the corrected readings

The snapshot flattens matrices column-major; the §13.3 reading reshaped row-major and
grouped the wrong elements.  Corrected (§13.3 now carries the table): the lean rows of
seed 2's R13 brain are 0.024 / 0.011 against pose rows of 0.05–0.09 — **identified, at a
quarter of the pose authority** — and the rates 0.089 / 0.044.  The "no gradient at the
model" diagnosis is retracted.

### 14.3 The attitude feedback is restoring in every standing brain

One-step attitude loop gain S = Σⱼ A(idx, j)·C(j, idx), negative = restoring, with the
cosine between C(:, idx) and −A(idx, :):

| brain | gravity x | gravity y | pitch rate | roll rate |
|---|---|---|---|---|
| R13 stand, un-hunted | −0.42 / 0.85 | −0.13 / 0.72 | −0.45 / 0.66 | −1.05 / 0.71 |
| pipeline checkpoint (hunted, rested) | −0.08 / 0.80 | −0.02 / 0.64 | −0.18 / 0.49 | −0.11 / 0.56 |
| controlled checkpoint (old era) | −0.03 / 0.88 | −0.00 / 0.06 | −0.20 / 0.97 | −0.04 / 0.78 |
| wind 0.5 N brain (§12.4) | −1.47 / 0.89 | −0.32 / 0.78 | −3.27 / 0.90 | −2.87 / 0.78 |

The direction was never missing, and the wind grew the gain three to seven times without a
catch.  The host's `--probe` adds the reference point: a hand-gained PD on the *true*
Jacobian, best of twenty gain pairs, still needs 7 rescues/min — a one-tick linear
attitude reflex does not catch this body by gain alone.

### 14.4 The defect: the lean rows are right for the left leg only

The probe prints the true Jacobian (∂gravity/∂rad per joint).  Against it, per module:

| identified A, seed 2 | left leg, pitch / roll | right leg, pitch / roll | head, pitch / roll |
|---|---|---|---|
| R13 (settle every 2 windows) | **+0.86** / +1.00 | **−0.22** / +0.98 | **−0.23** / +0.02 |
| R19 (settle every window) | +0.84 / +0.99 | **+0.54** / +0.93 | **+0.51** / +0.42 |

(cosines; the head's neck and head pitch carry the largest pitch authority of any joint,
+0.41 and −0.20).  The babble alternates legs inside one settle period, so the right leg
always pulsed on a body still moving from the left leg's pulse, and the head's 7-tick
windows straddled the 12-tick settles.  So the prior's forward-lean descent built
restoring feedback in one leg — the one-sided response §11.2's topples showed (left
ankle +128 mrad, right +20) — and a one-legged push twists instead of catching.

### 14.5 R19 — a settle before every pulse window

`a1v2_r19_settle_each.json` = R13 with the head's `babble_hold` 6 (the legs' value) and the
harness at `--ident-every 6 --ident-until 3000`: every window starts from a settled body,
and a completed first half survives the settle between windows (a settle *inside* a window
drops it).  Identification ends at 170 s with **0 falls**, upright TLE 0.18 (R13: 0.95).

**The race, six seeds, two hours each:**

| | R13 | **R19** |
|---|---|---|
| seeds consolidated | 1/6 (seed 2, at 30 min) | **6/6, inside 15 min** |
| rescues in 2 h | 128 (seed 2); 1300–2300 elsewhere | **2, 3, 2, 2, 2, 2** |
| tilt · pose · trunk z | 1.0° · 0.069 · 0.116 m | **0.35° · 0.036 · 0.1163 m** |
| \|u\| · joint motion | 0.34–0.8 · 11–13 mrad/tick un-rested | **0.18 · at the noise floor**, without the rest trio |
| upright TLE | 0.85 | **0.06** |

**The envelope, caught / delivered, six seeds × six shoves per force:**

| shove | old stance (§11.1) | **R19** | RL scaffold |
|---|---|---|---|
| 1 N | 5/6 | **36/36** | — |
| 1.5 N | 3/6 | **36/36** | — |
| 2 N | 1/6 | **35/36** | 6/6 at 0.4–7.9° |
| 3 N | 0/6 | **13/36** (lateral caught, fore-aft goes over) | 6/6 at 0.5–4.8° |
| 5 N | 0/6 | — | 6/6 at 3–11° |

A 2 N shove peaks at 2.3–3.0° and is back under 1° within 600 ms on every heading; the
neck pitches +20–40 mrad, the hips ±30, the ankles ±15 — a whole-body catch.  The old
stance leaned 3.7° and stayed at 1 N and went over at 2 N.  After a 3 N rescue c drops
only to 0.90 (the R13 tax) and is re-earned in 19 s.  Checkpoint `duck_r19_s2.json`.

### 14.6 Verdicts and scale

- **Identification schedule (R19): WORKING, LOUD.**  6/6 seeds stand inside 15 minutes and
  catch 2 N — the §3.3 shape: minutes, seed-robust, no averaging needed to see it.  From
  scratch, no rest trio, no checkpoint lineage.
- **The pulse horizon (R15–R18): REGRESSION**, re-use context: the identifiable window is
  bounded by the body's own topple time; a longer pulse needs a body that stays up through
  it (a settle-held pair, or a lighter body).
- **What the catch needed**, in the rewrite rule's terms: not a sensor, not a weight, not a
  longer model — an identification in which every actuator's authority over the lean was
  read from a still body.  The wind (§12) and the weight (§13) were amplifying a gradient
  that was right on one side and wrong on the other.
- **Not yet done**: the (d) push test proper on R19 at 3 N (rescue → re-earn is measured
  once above); the R12c rest on top of R19 (it may not be needed — the stance is already
  at the noise floor); the fore-aft 3 N limit and whether the head's identified pitch
  authority (cosine 0.51, still the weakest) is what bounds it.  Promotion to `★` is the
  operator's eye (a preset shoves the checkpoint live).
