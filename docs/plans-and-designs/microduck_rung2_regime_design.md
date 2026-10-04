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
| mitosis | ⚠ never fires even at threshold 0.10 — *not conditioning: the v4 EPM never called the gatekeeper (found 2026-09-05; `mitosis_gatekeeper`, default off, restores the call; register O4)* |

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
the status quo. The two ⚠ items are recorded as R1-era work (mitosis: the gate was never
called, not a conditioning issue — see above; transition-surprise sharpening), not gate failures.

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

### 14.7 Addendum, the operator's eye (2026-09-03) — the deployed servo filter, and sustained pushes

The operator watched R19 from scratch and then shoved the resumed stand with long, gentle
pushes, and saw the body adjust *during* the push: "balancing has been maximized up to the
point of the robot needing to take a step".  Two measurements to put numbers on that, both
on `duck_r19_s2`:

- **The deployed servo filter (`--servo-filter`, robotd's lag) no longer matters.**  Ten
  minutes resumed under it: 0 falls, 0.021 mrad/tick, pose 0.036.  Envelope under it: 6/6
  at 1 N, 6/6 at 2 N, 1/6 at 3 N — the same as without.  The §10 result ("the consolidated
  loop cannot run through phase lag it never learned", 80 falls in 5 min) was about the old
  stance; R19's catch is slow enough to be indifferent to the lag.  Sim-to-real relevant.
- **Sustained pushes, 2 s holds every 6 s, 44–48 per run:** 0.8 N caught 46/48 with the
  filter and 43/48 without (peak lean 1.6–2.8°, held against the push); 1.5 N caught 5/44;
  2.5 N 0/44.  The limit is the angle the feet can hold, not the impulse — 0.8 N × 2 s is
  1.6 N·s and is caught, where 3 N × 0.1 s (0.3 N·s) goes over half the time.  Past that
  angle the only recovery is a step, which this brain does not have.

R19 is promoted: `★ STACK` in the launcher, the old R13 → R11 → R12c chain relabelled
`PIPELINE (old lineage)`.

## 15. PHASE 0 OF THE INTENT BOUNDARY (2026-09-03) — the step hand-off

The first capability of "the middle" ([intent-boundary design](microduck_intent_boundary_design.md)
§4, phase 0): when the reflex's in-place catch is about to fail, hand the joints to the walker,
let it recover, hand back.  Host-only; no brain code changed.

### 15.1 The trigger, measured before it was written

Across six R19 brains and 72 shoves, a caught shove never leans past 5.4° (projected-gravity
lean), and every 3 N topple passes 5.8° by 300 ms with the lean still rising, 8° by 400 ms,
10° by 500 ms — about 700 ms before the fall detector fires.  `--step-lean 6.5` with three
consecutive rising ticks separates the two classes with margin, on the same egocentric
signal the fall detector reads.  The hand-off reuses the identification path's machinery:
learning off, pairing invalidated at both edges, the walker (`alpha_stand`, the standing
network given a twist) drives for `--step-twist-secs` then until still or `--step-settle-secs`,
handback resets the brain's action memory.  A rescue that starts mid-step ends the step and is
counted.  0 disables; byte-identical on the r3 reference.

### 15.2 Results (seed 2 checkpoint unless stated; 0.1 s shoves every 60 s, six per run)

| shove | hand-off off | hand-off on (twist 0.2) | steps / handed back / rescued mid-step |
|---|---|---|---|
| 1 N | 6/6 caught | 6/6 caught | 0 — never fires |
| 2 N | 6/6 | 6/6 | 0 — never fires |
| 3 N, **six brains** | **13/36** upright | **36/36** upright | 20 / 20 / 0 |
| 4 N | 1/6 | 5/6 | 4 / 3 / 1 |
| 5 N | 1/6 | 6/6 | 5 / 5 / 0 |

**The twist is not load-bearing.**  At 3 N every variant — 0, +0.2, −0.2, +0.35 m/s — went 3
for 3; at 5 N twist 0 went 6/6 (4 steps), +0.2 6/6, and +0.35 and −0.2 each 5/6.  The walker's
own trained stagger is the step, and it does not need to be told where to go.  Default set to 0.

**A 20-minute soak** (3 N every 30 s, 39 shoves): 25 hand-offs, 25 handed back, 0 rescues,
c 0.98 at the end (each hand-off costs the reset tax ×0.9, re-earned in ~17 s).  **The reflex
afterwards** (2 N, hand-off off): 6/6 caught, 0 steps, 0.065 mrad/tick, c 1.00 — gate (iii)
holds: nothing the walker did was learned into the brain.

### 15.3 Verdict and what it is

**WORKING, LOUD** — the three gates of phase 0 pass (byte-identical off; 3 N front-to-back
36/36 vs 13/36; the reflex's envelope unchanged).  What it is, said plainly: a *hand-off*, not a
learned step.  The brain contributes the trigger — the measured edge of its own catch — and the
walker contributes the recovery it was trained for.  The walker's stagger recovers a 5 N shove
from a 6.5° lean, which is the crutch meter's next rung: rescues per hour at 3 N went from ~14
to 0 with the hand-off, and a "step" is now a counted, logged event with its own colour in the
viewer (yellow).  What remains for the learned step is route 1's question (foot load and
contact in the blanket); what remains for phase 1 is to move the trigger into the brain as a
published saturation scalar rather than a harness rule.

## 16. PHASE 1 OF THE INTENT BOUNDARY (2026-09-03) — the trigger becomes the brain's, then the surface

### 16.1 Phase 1a — the hand-off fires on the brain's own signal

The attitude prior's instant error (`gate_err_inst`, the same scalar the consolidation ratchet
reads at 0.30 for "chaos") is now published per MotorEPM (`att` in the JSONL) and can be the
step trigger: `--step-att E`.  Measured on the same 72 shoves as the lean rule: a caught shove
never exceeds 0.049 (3 N) / 0.059 (2 N); topples pass 0.052 by 240 ms and 0.060–0.070 by 400 ms
with the error rising.  The signal includes the rates, and a catch itself produces rate, so the
margin is thinner than the lean's and the trigger fires ~70 ms later.  A/B across six brains,
threshold 0.06, three rising ticks:

| shove | brain-error trigger | lean rule 6.5° |
|---|---|---|
| 1 N (seed 2) | 6/6, 0 steps | 6/6, 0 steps |
| 2 N | 36/36, **2 false hand-offs** | 36/36, 0 |
| 3 N | 36/36, 20 steps | 36/36, 20 steps |
| 5 N | 30/36, 36 steps (6 rescued mid-step) | 29/36, 32 steps (7 rescued) |

**Verdict: WORKING, ties the lean rule on real topples, with a 6 % false-trigger rate at the
reflex's edge** (a false hand-off costs the ×0.9 consolidation tax and ~2 s of walker, not a
fall).  Adopted as the default trigger: the brain asks for help when its own prediction error
exceeds what its catch handles, which is the level-2 reading the design wants, and on hardware
it is a number the brain already publishes rather than a side computation in the harness.  The
lean rule stays available.  Re-use note for the adaptive form (§5 rule 5): the threshold could
be learned as a margin over the largest error the brain has survived without help.

### 16.2 Phase 1b — a walk on request: the behaviour hand-off

The same excursion as a step, triggered by an intent (a scripted one for now: `--walk-from S
--walk-secs S --walk-vx --walk-vy --walk-vyaw`), and driven by the right network.  The stander
(`alpha_stand`) ignores a twist — the first walks moved the body 0.00 m — so Pollen's walking
policy is now vendored (`alpha_walking.onnx`, "velstand": walking on velocity commands and fall
recovery in one network, the same 61-wide observation; provenance in the scaffolds README and
THIRD_PARTY_NOTICES) and driven as their runtime drives it: actions × 0.9, joint targets
low-passed 0.7 (legs) / 0.5 (head), the values it was trained with.  It runs on the body it was
trained on: `scene.xml` includes the all-collisions model the velstand task used.  JSONL drive
`walk`, events `walk:start` / `walk:end`, a blue ball in the viewer.

**The walker's response, 4 s commands from the R19 stand (world displacement is
instrumentation; the brain never reads it):**

| command | forward | lateral | yaw | peak tilt | handed back |
|---|---|---|---|---|---|
| vx 0.1, 0.2 | 0.00, 0.01 m | — | −2°, −3° | ~1° | yes |
| vx 0.3 / 0.4 / 0.5 | 0.46 / 0.60 / 0.86 m | −0.09 / −0.21 / −0.21 m | −21° / −30° / −25° | 4.5–5.0° | yes |
| vx 0.6 (outside its ±0.4) | 1.04 m | −0.39 m | +316° (a spin) | 4.5° | yes |
| vx −0.3 | 0.00 m | — | −2° | 2.8° | yes |
| vy 0.2 / 0.3 | 0.00 / 0.05 m | 0.00 / +0.19 m | −4° / −21° | 1.2° / 5.6° | yes |
| vyaw 1.0 / 1.5 / −1.0 | 0 | 0 | +9° / +174° / −13° | 1.8° / 4.4° / 0.7° | yes |

Every walk handed back upright with no rescue, and the reflex was back at 0.06–0.09 mrad/tick
within five seconds of handback.  Two properties of the policy to design against: **a
standing regime** — commands below about 0.25 m/s (0.3 sideways, ~1.2 rad/s turning) are
"stand still" to it, and it does not walk backward at −0.3 — and **a start-up curve**: the
first 4 s of a forward walk turn the body 20–30°, after which it walks straight — measured
over 900 s at vx 0.3 (§16.5): heading drift −0.02°/s, 110 m net of 155 m walked at 0.128 m/s.
(The first version of this paragraph read the 4 s curve as a 5–8°/s drift; it is a transient.)
The first bounds the intent vocabulary.  Trained ranges: vx ±0.4, vy ±0.3, vyaw ±1.0.

**Verdict: WORKING** as a hand-off; a scaffold in the exact sense (a scripted intent driving a
trained walker), named as such, and the machinery every level-2 intent will use.

### 16.3 Phase 1c — the robot's own "where am I": Pollen's contact odometry, ported

`mj_host/src/Odometry.{hpp,cpp}` is a line-for-line port of their `odometry` crate (itself
from the prototype runtime and Rhoban's model_service): one sole corner is the anchor, at
world Z = 0 and the X/Y it had when it became the anchor; the trunk is oriented by the IMU;
the trunk's position follows by forward kinematics; when another corner drops below the
anchor for two ticks, the anchor moves there at that corner's current X/Y, so the estimate
never jumps.  Heading is the IMU's yaw.  The inputs are egocentric by construction: the two
foot sites' poses in the trunk frame (forward kinematics, evaluated by the simulator from the
joint angles and the body's geometry — `DuckBody::site_pose_trunk`) and the IMU quaternion
(`imu_quat`, the framequat sensor on the imu site, the analogue of the robot's IMU fusion).
Nothing reads the world pose.  Logged per tick as `odom: [x, y, yaw]`; the world pose the
host already logs is the instrument it is judged against.

| walk from the R19 stand | true world Δ | dead-reckoned Δ | error | of distance |
|---|---|---|---|---|
| vx 0.3, 4 s | −0.397, −0.244 m | −0.376, −0.240 m | 0.022 m | 5 % |
| vx 0.4, 4 s | −0.582, −0.258 m | −0.553, −0.235 m | 0.036 m | 6 % |
| vy 0.3, 4 s | +0.099, −0.174 m | +0.086, −0.175 m | 0.012 m | 6 % |
| vyaw 1.5, 4 s (a turn in place) | 0.002 m | 0.021 m | 0.022 m | — |
| vx 0.3, 20 s | −2.379, −0.308 m | −2.288, −0.313 m | 0.091 m | 4 % |
| standing, 10 s | 0.1 mm | 0.2 mm | | |

Yaw agrees to 0.1° throughout (the simulated IMU quaternion is absolute; the real one drifts,
which is the one difference between this estimate and the robot's).  Per-tick velocity error
0.014–0.027 m/s.  **Verdict: WORKING** — the level-2 blanket's position channel exists and
is the same computation the robot publishes.  Also on the way: the vendored policies are
no longer in git (`*.onnx` is ignored) — `mj_host/scripts/fetch_scaffolds.sh` fetches both by
pinned commit and hash, and `run.sh` calls it when one is missing.

### 16.4 Phase 1e — the brain one level up: identification at the intent boundary

`--level2 --graph a1v2_r20_l2_ident.json` (R20).  The same module code as the joint-level brain
with the blanket drawn one layer out: a sensorimotor bridge and one MotorEPMv2 whose three
"motors" are the twist commands to Pollen's walker (vx, vy, vyaw, in the walker's trained
units 0.4 / 0.3 / 1.0) and whose "senses" are the body's own velocity — contact odometry
differenced and rotated into the body frame by the odometry's own yaw, the yaw rate from
the gyro, both smoothed over ten ticks — plus the 12-slot attitude/rate/accel sense as load
slots.  The walker drives the joints throughout; the rescue harness stands the body up after
a fall with the level-2 learning frozen and its pairing invalidated at both edges, exactly as
at the joints (`IntentAdapter`).  Identification by the same structured babble: held twist
pulses, one axis at a time, alternating sign — 75 ticks (1.5 s) at 0.8 of the range.  No
prior, no controller learning: the gate is the identified A.

**The gate — the identified A, sensed-velocity row against command column (positive,
dominant diagonal wanted), seed 2, 700 s, 0 rescues in every run:**

| babble | vx row | vy row | vyaw row | verdict |
|---|---|---|---|---|
| hold 75, scale 0.8 (R20) | **+0.0083** +0.0034 +0.0000 | −0.0011 **+0.0040** −0.0002 | +0.0039 −0.0013 **+0.0090** | pass |
| hold 50, scale 0.8 | **+0.0126** +0.0051 +0.0000 | +0.0009 **+0.0050** +0.0018 | +0.0062 −0.0007 **+0.0137** | pass |
| hold 100, scale 0.8 | **+0.0063** +0.0026 +0.0001 | +0.0026 **+0.0047** +0.0017 | +0.0035 −0.0009 **+0.0084** | pass |
| hold 75, scale 1.0 | **+0.0083** +0.0033 +0.0000 | −0.0002 **+0.0037** +0.0017 | +0.0043 −0.0015 **+0.0091** | pass |
| hold 75, scale 0.6 | **+0.0078** +0.0033 −0.0000 | −0.0086 **−0.0055** −0.0072 | +0.0098 +0.0029 **+0.0123** | **vy fails** |

The one failure is the rule from §14 one level up: at 0.6 of range the lateral pulse is
0.18 m/s, inside the walker's standing regime (§16.2), so the sideways pulses move nothing
and the row is identified from drift.  Identify where the actuator answers.  The off-diagonal
terms that survive are physics, not noise: a forward command yaws the body (+0.0039, the
heading drift of §16.2) and a sideways command moves it forward a little.

**The senses are honest**: sensed versus true body velocity differ by 0.012 m/s on average over
35 000 ticks; during forward pulses true vx +0.110, sensed +0.106; backward pulses walk at
−0.099 (the walker does walk backward at −0.32 m/s, which the 4 s command of §16.2 at −0.30
did not show); sideways +0.036 / −0.029; turning ±0.33 rad/s; at zero command all zero.

**Verdict: WORKING** — the level-2 blanket is closed, egocentric, and identifies.  The
design doc's phase 1 gate ("the identified move → odometry rows agree with dead-reckoned
ground truth in sign, per axis") is met.  What follows is the first level-2 *control*: a
prior on this surface — forward speed and zero yaw rate, "I predict I am walking straight" —
learned through the identified A, judged against the walker's own 5–8°/s drift.

### 16.5 R21 — the first level-2 control: walking straight, and the homeokinetic spin

R21 = R20 + a state prior on the sensed forward velocity (slot 0 → 0.75 of range = 0.3 m/s)
and on the yaw rate (slot 6 → 0), learned through the identified A.  Babble 600 s, then the
prior drives; judged over 700–1500 s against the walker under a constant forward command.

| arm | forward speed | heading drift | straightness (net / path) | rescues |
|---|---|---|---|---|
| open loop, vx 0.3 constant, 900 s | 0.128 m/s | −0.02°/s | 0.71 (110 of 155 m) | 0 |
| R21 with `ctrl_lr` 0.1 (HK on) | 0.163 m/s | **+6.5°/s — a spin** | 0.02 | 0 |
| R21, HK on, no dither | 0.165 m/s | +7.1°/s | 0.01 | 0 |
| **R21 prior-only (`ctrl_lr` 0)** | **0.214 m/s** | **0.00°/s** | **0.90 (178 of 197 m)** | 0 |
| prior-only, target 0.5 | 0.145 m/s | −1.4°/s | 0.05 | 0 |

The spin is §10's efference limit cycle one level up: with HK on, the learned yaw row is
dominated by the copy of its own yaw command (C(vyaw, yaw act) = −57) and the body circles
at 7°/s regardless of dither.  With the prior alone the controller walks straighter than the
walker does by itself, at the walker's top speed (the 0.75 target sits above what it can do;
the command rails at 0.4).  At target 0.5 the corrections the yaw-rate prior asks for fall
inside the walker's turning dead zone and the heading wanders.  **Verdict: prior-only
WORKING; HK at the intent level REGRESSION** (re-use: the rest trio's efference rest, one
level up).  R21's config is the prior-only form.  Also measured here: the walker holds its
heading over 15 min by itself (§16.2 corrected), so "walk straight" needed a perturbation to
mean anything — which is §16.6.

### 16.6 R22 — heading regulation: the (d) test at the intent level, and what a heading is

R22 adds a heading prior.  Getting the *sense* right was the whole work, and each wrong form
was measured before the next (the ledger's process record; the result is the last row):

| heading sense (slot 10) | identified row (vs vyaw) | what the body did under 2 N shoves |
|---|---|---|
| odometry yaw / π, wrapped | −0.0014 (wrong sign: a pulse across ±π hands the estimator a jump of two) | held **±176°** tightly — the wrong fixed point |
| sin(yaw) | −0.0049 (the body faced ~180° through the babble, where the sine's slope is negative) | held ±177° |
| unwrapped yaw / π, clamped | 0.0000 (the body winds past a half turn in the babble; the clamp saturates) | wandered like the open loop |
| **deviation from a slow running average (τ 60 s) / π** | **+0.0034** | **returned to its pre-shove heading** |

A wrapped angle is not one linear row (its slope changes sign with where the body faces), an
unwrapped one saturates, and a heading *relative to the heading recently kept* is bounded,
linear, and a memory that forgets over a minute — long enough to answer a shove, short
enough never to saturate.  With it, twenty 2 N shoves every 40 s during the walk:

| arm | peak excursion | \|offset\| 20 s after a shove | rescues | speed |
|---|---|---|---|---|
| open loop (constant vx 0.4) | 39° | **109°** — never returns | 0 | 0.204 m/s |
| heading prior only (R22b) | 17° | **3°** | 0 | 0.240 m/s |
| heading prior only, 3 N | 36° | 5° | 2 | 0.245 m/s |
| heading + yaw-rate priors (R22) | 20° | 4° | 0 | 0.247 m/s |

Per-shove, the heading prior brings the body back within 5–10 s (+2 s: −12°, +5 s: −2°,
+10 s: −2°, +20 s: +3°).  Learned: C(vyaw, heading) = −0.91 (restoring), C(vyaw, yaw rate)
= −1.24 (damping), through an A the brain identified itself.  **Verdict: WORKING, LOUD** —
the doctrine's "heading regulation: re-corrects when noise turns the body", on the intent
boundary, with every input egocentric (contact odometry, gyro, IMU) and the walker untouched.
Scale: one seed (the identification is deterministic), 20 shoves per arm, two forces; the (d)
test is the result itself.

## 17. PHASES 1d AND 2 (2026-09-03) — the eyes, avoidance, and the drive that avoidance needs

### 17.1 Phase 1d — the ToF in simulation

`mj_host/src/Tof.{hpp,cpp}`: the VL53L8CX's 8×8 depth matrix on the head, its 64 beams cast
from the `tof` site with MuJoCo's ray query against world geometry only (group 0 — the robot
is invisible to its own sensor, as in reality), classified by a port of Pollen's
`kinematics::tof::Reprojector` (Empty / TooClose within 10 cm / Floor when a downward beam
reaches 85 % of the way to the ground / Hit with its horizontal range), with the head pose
from forward kinematics, the level from gravity, and the trunk height from the contact
odometry.  Cast every 4 ticks (12.5 Hz, the sensor's rate).  The beam frame was measured,
not assumed: at the standing keyframe the site's axes coincide with the world's (+x the
optical axis, +y left, +z up — a ray along +x meets a wall 0.5 m ahead at 0.41 m from the
site).  Gate: a wall 0.5 m ahead reads Hit at 0.40–0.47 m on seven rows and Floor on the
eighth; in the 2 m arena (`scene_arena.xml`, walls 0.3 m high) the top row is Empty above
the walls, three rows Hit the wall a metre away, four rows Floor at 0.46–0.97 m.  The
JSONL carries the 64 classes and ranges (`tofz`, `tofr`) and the four-slot summary the brain
sees (proximity ahead-left / ahead / ahead-right as 1 − range/1 m, TooClose fraction, `tofs`);
the viewer draws the beams from the head (Hit orange, Floor grey, TooClose red).  A wall
contact instrument (`wall`) counts collisions for the reader.

### 17.2 Phase 2a — avoidance (R23), and what avoidance alone does

R23 = the walk-forward prior (vx → 0.75) plus priors of zero on the three proximities (sense
slots 12–14, `load_slots` 16), identified by the same twist babble in the arena.  1500 s,
seed 2, judged over the control phase (700–1500 s):

| arm, in the arena | wall-contact episodes | ticks in contact | TooClose fraction | path | cells visited (0.25 m) | falls |
|---|---|---|---|---|---|---|
| open loop, vx 0.4 | 45.8/min | 5.7 % | 0.23 | 150 m | 33 | 0 |
| R21, walk straight | **194/min** — pinned against a wall by its own forward prior | 42.8 % | 0.50 | 92 m | 31 | 2 |
| **R23, avoidance** | **0.0/min** | **0.0 %** | **0.000** | 167 m | **9** | 0 |

Learned: C(vyaw, proximity left) = −0.25, C(vyaw, proximity right) = −0.22 — a turn away
whenever anything is near, on either side — and a forward bias.  The proximity rows of A
are weak (0.0009 against the command; the babble rarely reached a wall) and the result is
what those gains produce: **an orbit in the middle of the arena, 167 m inside nine cells.**
No collisions, no exploration.  The blind metric caught by its complement: collisions = 0
is also satisfied by never approaching anything.  **Verdict: avoidance WORKING, coverage
DEGENERATE** — and the reason is the one the design named in advance: avoidance is a
constraint, not a drive; the drive is novelty (phase 2b).  Also learned: the walk-straight
brain pins itself against a wall at 43 % of ticks, which on hardware is a stalled servo —
the proximity priors are a safety result before they are a behaviour.

### 17.3 Phase 2b, first probe (R24) — a map that learns the arena, and a surprise prior with no gradient

A slow EPM over the robot's own place vector — dead-reckoned x/2, y/2, cos and sin of the
heading, the eight ToF column ranges / 4 m, RBF-encoded, every 5 ticks (`map_epm`) — is the
learned novelty grid; its surprise (TLE) enters the level-2 sense as slot 11 and R24 puts a
prior of 0.6 on it: "I expect to be surprised", curiosity as a prediction to fulfil, with
R23's avoidance priors kept.  1500 s in the arena, and 1800 s with the far wall moved from
x = 1.0 to 0.5 at 1100 s (`--arena-shift`, the (d) test):

| arm | cells (0.25 m), control phase | span | map nodes | map TLE / novel | walls |
|---|---|---|---|---|---|
| R23 avoidance | 9 | 0.5 × 0.6 m | — | — | 0 |
| **R24 + surprise prior** | **9** | 0.6 × 0.6 m | 8 | 0.13 / 33 % | 0 |
| R24, wall moved at 1100 s | 9 before, 9 after | unchanged | 8 → 11 | 0.13 → 0.10 | 0 |
| open-loop walker, map only | 33 | the whole arena | 30 at 5 min, **34** plateau | 0.21 → 0.14 / 30 → 18 % | 45/min |

**The map works**: under a walker that tours the arena it settles at 34 nodes for 33 visited
cells, and its surprise falls as the arena becomes familiar — a learned familiarity signal,
egocentric by construction.  **The prior on it does nothing**: the identified A row for the
surprise slot is [4e-4, −7e-5, 3e-4] — zero to the precision that matters — because surprise
depends on where the body has been, not linearly on the command it gives now.  With no
gradient, the orbit R23 learned (a turn away from anything near, on either side) stays the
attractor, the map sees eight places, and a moved wall adds three nodes without drawing the
body toward it.  **Verdict: NULL** — with its cause: novelty is sensed but not yet a
*direction*.  Two ways to make it one, the operator's fork: (a) a level-3 inference on the
map — the bearing to the least-visited region as a learned "where to go" belief, the
doctrine's own requirement that selection be learned; (b) the map's familiarity gating a
change of the heading the brain keeps — cheap, testable now, and the first form of their
Wander/LookAround.  (b) is measured next as R25.

### 17.4 Phase 2b, second probe (R25) — wander by boredom, and the arbitration question

R25 = R24's map + R22's heading prior + R23's avoidance priors + the host's wander rule
(`--wander-bored 8 --wander-turn 90`: when the map's surprise has sat below 0.8 of its own
long average for 8 s, the heading reference behind sense slot 10 jumps by 90° with a random
sign and the heading prior turns the body; novelty holds the heading — self-scaled, no
constant tuned to the signal).  1500 s in the arena, seed 2:

| arm | wall-contact episodes | cells | span | map nodes / TLE | heading changes |
|---|---|---|---|---|---|
| R25 without the wander rule (heading + avoidance priors) | **297/min** | 28 | the whole arena | 46 / 0.21 | — |
| R25, turn 90° | 286–294/min | 28–32 | the whole arena | 44–50 / 0.21 | 32 |
| R25, turn 135° | 280/min | 28 | the whole arena | 36 / 0.22 | 35 |
| R25, wall moved at 1100 s | 294 → 272 → 269/min | 32 → 8 → **1** | pinned in a corner by the moved wall | 44 → 31 / 0.22 → 0.06 | 23 |
| R23 (avoidance only, §17.2) | 0/min | 9 | an orbit | — | — |
| open loop | 46/min | 33 | the whole arena | 34 / 0.14 | — |

**Verdict: REGRESSION**, with two causes that are the same cause.  The heading prior wins
against the avoidance priors because its identified row (0.0034) is four times the
proximity rows' (0.0009), so the body rides the walls: full coverage by contact, worse than
the open loop's 46 contacts a minute.  The proximity rows are weak because the babble
rarely met a wall and the state model is babble-owned (`babble_owns_a` 1), so nothing the
body learned about walls afterwards ever entered A.  The wander rule changed the heading
32 times and the coverage not at all, and after the wall moved the body ended in a corner
with the map's surprise at 0.06 — nothing new to see, nowhere to go.

**What phase 2 has established, and the fork it leaves.**  The eyes work (17.1); avoidance
as a constraint works and pins the body in an orbit without a drive (17.2); the map works as
a learned familiarity signal (17.3); and a single linear pull cannot hold three behaviours
at once — forward, avoid, keep heading — because it resolves their conflicts by the size of
the identified rows, not by need (17.4).  That is the arbitration question the design named
in §3 ("presence, mood and the beat are inputs to one brain"; the Cell report's "curiosity
must yield to need"), now measured.  Two things to decide, the operator's:

1. **Identification of the world channels.** Let the state model keep learning after the
   babble at level 2 (`babble_owns_a` 0 — the one-owner rule was about the babble *phase*),
   so wall encounters identify the proximity rows; or babble deliberately near walls.
2. **Arbitration.** Behaviours as separate loops with the LateralVoter's precision
   weighting (the substrate this brain already has one level down), so avoidance carries
   the precision when something is near and heading or novelty otherwise — rather than one
   pull on one C.  With that in place the drive question (a level-3 "where to go" inference
   on the map versus a boredom-gated heading) can be answered on a controller that can
   hold more than one thing true.

### 17.5 The harness, and the fork's first item at n = 6 (2026-09-06)

Every table above was read at seed 2. `mj_host/tools/l2_sweep.py` now runs a level-2
config from scratch over N seeds in parallel (the host is unpaced headless: 1500 s of sim in
about twenty seconds of wall time) and reports the §17 metrics over the control phase, paired
by seed, with the identified A rows read back. Two harness facts came out of its first use
and are fixed: the level-2 command's scene defaults to the **open floor**, not the arena (a
sweep that omits the scene reports zero wall contacts because there are no walls; the tables
above did pass the arena), and the reset-noise flag never reached the level-2 command, so a
"seed" varied only the babble. With the arena and reset noise 0.05:

| arm (n = 6, 1500 s, control phase 700–1500 s) | walls/min | contact | TooClose | path | cells | span | map nodes |
|---|---|---|---|---|---|---|---|
| R23 avoidance | 0.0 ± 0.0 | 0 % | 0.00 | 167 m | **8.8 ± 1.9** | 0.29 m² | — |
| R23 + `babble_owns_a 0` | 4.0 ± 9.0 (two seeds) | 1.2 % | 0.00 | 168 m | **29 ± 21** (6 of 6 up, t 2.3) | 1.5 m² | — |
| R25 heading + avoidance + map | **295 ± 8** | 43 % | 0.79 | 87 m | 28 ± 0 | 3.5 m² | 89 ± 13 |
| **R26 = R25 + `babble_owns_a 0`** | **26 ± 35** (Δ −269, t −17, 0+/6−) | 4.9 % | 0.06 | 157 m (+71, t 15) | 30 ± 19 | 1.9 m² | 35 ± 18 |

**R23's orbit and R25's wall-riding are findings, not seed-2 readings: six of six each.** And
the fork's item 1 alone (§17.4: let the state model keep learning after the babble, so the
proximity rows are identified from real wall encounters) removes the regression on every
seed — a ten-fold drop in wall contacts, a body that walks 157 m instead of scraping 87, a
ToF that sees a wall 6 % of the time instead of 79 %, and a map that stops tiling wall
texture (89 → 35 nodes). One flag, no new module; the identified yaw row goes from
`[+0.004 −0.001 +0.009]` to a row that carries the proximities. Verdict: **`WORKING`, loud**
(CLAUDE.md §3.3). R26 is the line's new base (`a1v2_r26_l2_learn_after_babble.json`,
launcher rank 1026).

Two things the seeds say that seed 2 could not: coverage is **bimodal** under the fix (four
seeds hold 14–25 cells, two tour 49–60), so the orbit and the tour are both attractors and
which one a seed finds is part of the next question; and the wall contacts that remain (two
seeds at 52 and 85 per minute) are the touring seeds — the avoidance is weakest exactly where
the coverage is best, which is the arbitration question in its next form.

**The moved-wall (d) test at n = 6** (`--arena-shift 1100`, 1800 s, the control phase split
at the shift):

| arm | cells before \| after | walls/min before \| after | map nodes before \| after |
|---|---|---|---|
| R25 | 28 \| 128 ± 277 (one seed leaves the arena through the moved wall) | 296 \| 351 ± 204 | 52 \| 47 |
| **R26** | 29 ± 20 \| 27 ± 19 | 47 ± 69 \| **29 ± 32** | 31 ± 16 \| 28 ± 10 |

R26 is robust to the change (its contacts fall after the wall moves where R25's rise) and it
does **not** re-explore: coverage holds and the map does not grow after the shift. The (d)
bar's re-inference half is `NULL` for R26 as it stands — the body keeps its habits in the
new room rather than mapping it — which is the coverage question again: what would make a
moved wall a *direction*. Scene note: after the shift one R25 seed escaped the arena (a
522 m² span), so the shifted scene has a gap the (d) reading must exclude or the metric must
clip to the arena.

**The wander rule on R26 at n = 6** (`--wander-bored 8 --wander-turn 90`, paired against R26
without it): identical runs on five of six seeds, one seed changed (walls +80/min, nodes +20).
The boredom trigger — the map's surprise below 0.8 of its own long average for 8 s — almost
never fires under R26, whose map stays 27 % novel because the body keeps touring; the rule
is inert here (`NULL`), and where it fired it bought nodes with wall contacts. What §17.4
called the arbitration question now has its measured shape: R26 already has coverage and
avoidance in one body without a wander rule; what it lacks is a *reason to go somewhere*
(a drive with reach), which is the Cell recipe's pragmatic loop, not a heading jump.

### 17.6 R27 — the fork's (a): novelty as a direction, the Cell's play loop over the map (2026-09-06)

The operator's choice of the two §17.3 forks: a "where to go" belief on the map before any
arbitration. Built as the Cell recipe's own loop: `PlayLoop` in the level-2 graph reads the
map EPM's winner as its place (`pi_cell_size 0`), climbs the place-TLE novelty field
(`wander_stall_ticks 0`, the Cell's A2, so the climb engages), and publishes an egocentric
bearing; the `IntentAdapter` now publishes the unwrapped heading and the body velocity as
`reality.proprio.heading` / `vel_ego`, and sets the heading reference behind sense slot 10
from that bearing every tick when the topic exists (by presence: a graph without the loop is
byte-identical). No new module; the loop the Cell dropped because it cost eats is the loop
the duck wants, because coverage is the duck's goal. Config `a1v2_r27_l2_play_heading.json`,
rank 1027.

| n = 6, paired | walls/min | cells | span | map nodes | (d) moved wall: cells before \| after | walls before \| after | nodes before \| after |
|---|---|---|---|---|---|---|---|
| R26 | 26 ± 35 | 30 ± 19 | 1.9 m² | 35 ± 18 | 29 \| 27 | 47 \| 29 | 31 \| 28 |
| **R27** | 9 ± 9 (Δ −16, t −1.1) | **42 ± 18** (Δ +12, t 1.7, 4+/1−) | 2.3 m² | 29 ± 13 | 37 \| 64 ± 69 (one seed leaves through the gap) | 13 \| **253 ± 143** (Δ +130, t 2.8) | 22 \| **49** (Δ +14, t 2.2, 5+/1−) |

The loop steers on every tick (the host's own count). In the steady arena it is a `PARTIAL`
at six seeds in the right direction on both blind-metric complements: more coverage with
fewer contacts, five of six seeds touring, one still orbiting. Under the moved wall the
(d) bar's re-inference half is now present — the map grows by half after the change,
which R26's never did — and safety collapses: the novel region is where the wall now
stands, the play bearing drives the body at it, and contacts go from 13 to 253 per minute
while R26's fall. **A learned direction beats the proximity priors when the two conflict.**
That is §17.4's item 2 in measured form, with a learned direction in place of a boredom
jump: avoidance and novelty each have a bearing now, and the body has no arbitration
between them but a linear pull on one C matrix. The Cell recipe's arbitration (need ×
competence over loops with bearings; `LoopCompetence` → `LateralVoter` → `EFEArbiter`
precision mode, all gain-0) is the next lever, with avoidance made a loop that emits a
bearing rather than a prior in the matrix. Scene note again: the shifted wall leaves a gap
one seed escapes through; the (d) metric must clip to the arena or the scene must close.

### 17.7 R28 / R29 — the Cell recipe's arbitration on the duck: avoidance as a loop (2026-09-06)

The measured form of §17.4's item 2: `TofAvoidLoop` (new, generic) turns the ToF summary
into a bearing away from the nearest obstacle with that proximity as its need; the Cell's
`LoopCompetence` grades each loop (avoidance: proximity falls while it drives; play: novelty
rises; Beta + optimism), a `LateralVoter` turns competence into trust, `EFEArbiter`'s
precision mode selects by need × trust with the nearest proximity as avoidance's "hunger"
and its complement as play's surplus, and the `IntentAdapter` takes the winner's bearing as
the heading reference (by presence of the arbiter; R27 and R26 stay byte-identical). The
harness now records the winner per tick (`steer`), the steer shares, and clips samples that
escape the arena.

| n = 6, steady arena | walls/min | cells | avoid share | (d) moved wall: walls before \| after | nodes before \| after |
|---|---|---|---|---|---|
| R26 (no loops) | 26 ± 35 | 30 ± 19 | — | 47 \| 29 | 31 \| 28 |
| R27 (play alone) | **9 ± 9** | 42 ± 18 | 0 | 13 \| **253 ± 143** | 22 \| **49** |
| R28 (avoid's bearing wins) | 71 ± 80 | 54 ± 21 | 0.30 | 71 \| 114 ± 132 | 42 \| 33 |
| R28 wrong-sign | 28 ± 23 | 52 ± 20 | 0.00 | — | — |
| **R29 = release form** (avoid wins → the reference is released) | 44 ± 62 | 49 ± 19 | 0.23 | 45 \| 149 ± 151 | 31 \| 41 |

**Why the bearing form fails, measured.** Past the babble, when avoidance won and set the
heading reference to its away-bearing, the body barely turned: 0.14 rad in 0.5 s with no side
preference at 0.5, 2 or 4 s. The heading reference is a slow regulator (§16.5 closed 3° in
20 s) and the duck reaches a wall in two seconds. Avoidance's fast path is the proximity
priors that R26 made work; only a slow direction belongs on the reference. So the
arbitration is not one bearing against another but **"hold the novelty direction" against
"yield to the reflex"**: R29 publishes avoidance's need with no bearing, and the adapter
treats a winning loop with no bearing as *release* (the reference set to the current
heading, so the heading prior stops fighting the proximity priors).

**Verdict `PARTIAL`.** The trade-off is real and neither pole wins both regimes. In a known
room play alone is the best avoider — walls stop being novel within minutes and the novelty
climb turns away from them nine times in ten (P(turn left | wall on the right) 0.90 while
play steers) — and every form that takes the motor from play near a wall costs contacts:
R28 +62/min, R29 +34/min (4 of 4 seeds worse). After a wall moves, the new wall *is* novel and
play drives at it; R29 cuts that burst by 40 % (253 → 149/min) while keeping half of R27's
re-exploration (map +10 vs +22 nodes); R28 stops the burst harder (114) and the
re-exploration with it. The wrong-sign control never lets avoidance win (its share 0.00), so
here it is a play-alone arm with a different hysteresis, not a control of the ordering.

What this hands the recipe: on a body whose only goal is coverage, the novelty loop is the
pragmatic loop too, and a need-gated reflex that overrides it should fire only when the
world has *changed* — the moved wall is novel and dangerous at once. The next form is not
a better ranking but a competence signal that distinguishes "novel because unvisited" from
"novel because it moved": the map's node *persistence* (a node whose prototype the world
contradicts) rather than its TLE. That is the Cell's disconfirmation problem (register O9)
arriving on the duck, and it is where the arbitration line stops for this phase. The
escape counts (5 000–17 000 samples per run beyond the arena after the shift) say the
shifted scene must be closed before the (d) reading is trusted at power.

### 17.8 A1 — the playroom: the room is the lever, R27 unchanged (2026-09-10)

The first lever of the [playroom plan](microduck_playroom_plan.md) §9: an arena that can show
the behaviour set. `mj_host/tools/playroom_gen.py` generates `scene_playroom.xml` from a seed
— a 4 m × 4 m room, walls 0.3 m, a rug, a table the duck walks under, two chairs, a shelf of
coloured books, a wall clock whose hand the host turns once per 20 s (`gravcomp` on the hand:
without it the hand hung at the bottom on the first run), two balls and two blocks as free
bodies — and writes a manifest beside it (seed, hash, every object's class and position, and
the qpos address of every non-robot entry). The host echoes the manifest at start, the sweep
prints it, and the sweep now refuses to print a run that produced no JSONL as a row of zeros
(the first sweep did exactly that: the host runs in `mj_host/` and a relative scene path
missed). Textures are MuJoCo builtins with fixed seeds.

**Host changes, gain-0.** Reset noise now perturbs the robot's own joints only (a ball's
quaternion is not "a slightly wrong pose"); the `wall` instrument counts contact with any
static world geom that is not floor or rug, so furniture counts; an `obj` field (contact with
a movable, named `obj_*`) is printed only when the scene has movables; `--move NAME X Y S`
relocates a movable, a furniture body or a world geom at S s (the (d) test; `--arena-shift`
unchanged). **R27 seed 6 in the arena, 400 s, is byte-identical before and after** (md5
`784d4acc…`, `mj_host/log/playroom/ref_r27_arena_s6_{before,after}.jsonl`); the gates pass.

**R27's loop in the playroom, unchanged (R30 = R27's config, the scene swapped), n = 6,
1500 s, noise 0.05, control phase 700–1500 s, room seed 1:**

| | walls/min | cells (0.25 m) | map nodes | map TLE | objs/min | objects moved (m) | rescues/min | escaped |
|---|---|---|---|---|---|---|---|---|
| playroom (R30) | 33 ± 39 (5.1 / 6.5 / 17 / 23 / 39 / 108) | 140 ± 15 | 98 ± 9 | 0.20 | 0.07 | 3.7 ± 2.5 | 0.05 | 0 |
| arena (R27, §17.6) | 9 ± 9 | 42 | ~30–40 | 0.21 → 0.14 | — | — | — | (gap) |

Read with the room's size: the arena has 64 cells, the playroom 256, so 140 cells is 55 %
of the room against the arena's 66 %, over a 4× area with a 1.5 m longer crossing. The map
grows with the room (98 nodes). Wall contact attributes mostly to the walls, not the
furniture: seed 2 (108/min) rides the walls (87 % of its contact ticks at a wall, 9 % at a
chair), seed 5 splits wall and chair1, seed 6 (5/min) touches everything a little (54 % wall,
16 % table, 12 % chair, 11 % shelf). The duck almost never touches a movable (0.07/min) —
nothing seeks them yet, which is what E1/E2 are for — and still moves 3.7 m of ball per run
by walking into them. No escapes: the room has no gap, so the moved-object (d) can be read
without the arena's clip.

**Verdict.** A1 `WORKING` as an instrument: generated, manifested, byte-identical elsewhere,
every object class present, the (d) flag exercised (a ball, a chair and a wall moved at
100/120/140 s in a test run). R27 on it is a *signal*, not a degenerate baseline: 5 of 6
seeds tour (115–157 cells), one rides the walls. The operator's eye next, through the
launcher's R30 preset (seed 6) and its moved-ball twin. **Found for C1, and fixed the same night (operator):** Pollen's
`head_camera` sits 8.5 mm inside the lens looking along the head's +z — backward — with its
up vector sideways, so a render from it shows the inside of the head rotated 90°. The
vendored file is never edited, so `playroom_gen.py` writes `robot_overlay_playroom.xml`, a
copy with one line changed: the camera at the lens's foremost vertex on the lens axis (body
frame `0.0155 −9e−05 −0.0818`), turned to look along the ToF site's measured forward with
the site's up as up (`quat 0.707107 0 0 −0.707107`), `fovy` 49° (the IMX219's ~62°
horizontal field at 4:3). The playroom scene includes the overlay; the run is byte-identical
with and without it (a camera has no physics), the gates pass. Rendered mid-run the frame is
level and upright: rug, walls, a block — and a lot of sky, because the walls are 0.3 m and
the camera is 12 cm off the floor. Whether the walls rise for the camera's sake is the
operator's call before C2. **Decided the same night: walls 1 m, and the light moved off
noon.** The camera-mounted headlight, which flattens every texture, is nearly off; one angled
sun (elevation ~45°) casts shadows and a weak fill from the opposite side keeps the shadowed
sides readable; shadow map 4096. Rendered: no saturated pixels in either view, textures
crisp. **The taller walls change the runs from tick 3** — the ToF's upper rows used to pass
over 0.3 m walls and now hit — so R30 was re-measured on the 1 m room:

| 1 m walls, n = 6 | walls/min | cells | map nodes | map TLE | objs/min | objects moved (m) | rescues/min | escaped |
|---|---|---|---|---|---|---|---|---|
| playroom (R30) | 7.1 ± 6.0 (2.8 / 3.9 / 4.6 / 4.7 / 7.8 / 18.8) | 151 ± 34 | 100 ± 20 | 0.20 | 0.07 | 5.1 ± 2.3 | 0.05 | 0 |

The wall-rider is gone (seed 2: 108 → 7.8/min) and the room's contact rate is now below the
2 m arena's 9/min over four times the area: the 0.3 m walls were partly invisible to the
sensor, which is a measurement about the *old room*, not the loop. The §17.8 table above
stays as the record of the 0.3 m room.

**The instruments, corrected (2026-09-10, later).** The `obj` flag was true on every tick of
every playroom run: `touching_object` counted *any* contact involving a movable, and a ball
resting on the floor is one. `touching_wall` had the same hole (a block leaning on a chair
leg would have counted). Both now count only contacts the robot is in; the arena is
byte-identical, the playroom's physics is byte-identical (only the two fields change). The
sweep gained `down%` (ticks with the trunk past 60° of tilt) and the level-2 summary prints
rescues the harness gave up on. Re-measured, same runs:

| 1 m walls, n = 6, corrected | walls/min | objs/min | objects moved (m) | down % |
|---|---|---|---|---|
| playroom (R30) | 6.6 ± 5.8 | 22.5 ± 23.7 | 5.1 ± 2.3 | 0.14 ± 0.18 |

So the duck *does* run into the movables — twenty-odd contact episodes a minute, a ball
dribbled along — without anything seeking them; 0.07/min was the broken flag. Everything
else in the table above stands.

**The table-leg trap (operator's observation, the same day).** In a watched run on a random
seed (1069061822; log `mj_host/log/launcher/20260910-103450_*`) the duck walked into a table
leg at 947.7 s, fell to 141° of tilt with its left foot on the leg and the trunk against it,
and stayed there for the remaining 690 s: 83 rescues, every one given up after the harness's
8 s (`give_up_s`), hand-back, 0.2 s debounce, hand-off again. Three things the log settles:

1. **The rescue is the standing policy, not a get-up.** `Driver::Scaffold` runs
   `alpha_stand`, which holds the standing pose. On a flat floor that happens to right the
   body; wedged against a pillar it cannot, and nothing in the loop knows the difference.
   The harness has no "wedged" state — it just cycles. The operator's reading is exact: the
   sequence is not aware of its situation.
2. **The sensor saw the leg and the loop drove on.** Two seconds before contact the ToF's
   TooClose slot read 0.47 then 0.81 while `steer` stayed at play's bearing; a 3 cm cylinder
   is R27's known failure at its smallest — a learned direction beats the proximity priors
   (§17.6) — and the room now supplies that stimulus without an operator moving a wall.
3. **At level 2 our brain is not at the joints.** Pollen's walker drives; the joint-level
   brain that stood and caught (R19) is not in this loop at all. "It would have babbled its
   way out" is therefore a claim about B2 of the playroom plan (the fallen regime opened to
   learning, the rescue held back), and this trap is its first (d) scenario: dropped poses
   *and* wedged on a table leg. The six harness seeds never wedged (`down%` 0.14), which is
   why the number must be read per seed.

### 17.9 H0 / H1 — the head loop: the head IMU, and the identification babble on the four head commands (2026-09-10)

The playroom plan's H line (§4b there; agreed head-first the same day). The walker's command
vector carries four head targets — neck_pitch, head_pitch, head_yaw, head_roll, deltas from
HOME (`Observation.hpp` slots 51–55; Pollen's `robot.head`) — so a brain that owns the head
needs no joint access: `mj_host/src/HeadAdapter.*` is the twist adapter's code with four
command dimensions and the trained ranges (±1.10, ±1.10, ±1.40, ±0.31 rad), commanding the
head through the walker and sensing the head IMU.

**H0.** The overlay adds a gyro and an orientation on Pollen's `head_imu` site (their ToF board
carries the IMU; tofd reads it at 100 Hz); `DuckBody::head_gyro()` reads it, zeros without it.
Sensors touch no physics: the arena run and the playroom R30 run are byte-identical.

**H1.** `configs/head_h1_babble.json`: `motor_epm_head` over the four commands, 12 load slots
(head gravity x, y | head gyro x, y, z | trunk gravity x, y | trunk gyro x, y, z | 2 spare),
`babble_ticks 30000`, `babble_hold 25` (0.5 s; the walker's head low-pass answers in ~10 ticks),
`babble_scale 0.3` (±0.33 rad on the pitches), the prior off. Run with the twist brain overridden
to zero (`--l2-twist 0 0 0`: the walker stands) in the playroom, 700 s, seeds 1–5. The
identified A, rows vs the four commands, seed 2 (the other four agree to ±5 % on every entry):

| row \ command | neck_pitch | head_pitch | head_yaw | head_roll |
|---|---|---|---|---|
| pos neck_pitch | **+0.0237** | +0.0226 | +0.0052 | +0.0015 |
| pos head_pitch | +0.0040 | **+0.0555** | +0.0205 | +0.0014 |
| pos head_yaw | −0.0009 | +0.0004 | **+0.0497** | +0.0147 |
| pos head_roll | +0.0224 | +0.0074 | +0.0112 | **+0.0601** |
| head gravity x | **−0.0041** | **+0.0083** | +0.0019 | +0.0004 |
| head gravity y | +0.0056 | +0.0029 | −0.0102 | **+0.0132** |

Read: every position diagonal positive; head_pitch, head_yaw and head_roll dominant; **the
two pitches move head-gravity x with opposite signs** — at HOME both joints sit at +20° and
the head is level, so the joints oppose each other, and a positive delta on the neck tilts the
head one way while a positive delta on the head tilts it the other (the read-back agrees with
the geometry, not a guess); roll moves head-gravity y; the yaw row on gravity y (−0.010) is a
pitched head's yaw axis not being vertical. One coupling to know: a head_pitch command moves
the neck joint almost as much as a neck_pitch command does (+0.0226 vs +0.0237) — the walker's
policy treats the two pitches as one head pose. The gyro rows are small, as 0.5 s holds
average a rate to nothing; the transients are there for a faster model. **0 rescues on every
seed**: a 0.3-scale head babble does not fall the standing walker.

**Verdict.** H0 `WORKING` (byte-identical elsewhere). H1 `WORKING`: the head identifies from a
still body in one babble, seed-consistently, with the rows the H2 prior needs. Next, H2: the
prior on slots 12, 13 (head gravity x, y → 0) while walking, A/B against the walker's own
head, judged on head gyro RMS, gravity deviation, rescues/min and the brain-frame difference.
Launcher: R31 (watch the head babble; the twist brain held at zero).

### 17.10 H2 — the head prior while walking: level `WORKING`, still `NULL`, the picture `REGRESSION` (2026-09-10)

**Protocol.** Identify standing, act walking: the H1 head brain is saved at the end of its
standing babble (`--save-head`) and loaded into R30's tour (`--load-head`) with the prior on
and no babble. The control is the same load with `motor_gain 0`, which is R30 exactly (the
walker's own head; verified byte-identical on the tour's metrics). n = 6, 1500 s, control phase
700–1500 s, the head metrics from the head IMU while upright: gyro RMS over x, y (the head's
world motion, whatever the joints do), gravity deviation over roll and pitch (0 = level), and
H3 — the mean frame-to-frame difference of the 64 × 48 brain frame at 12.5 Hz, rendered from
the run's qpos (seed 6).

**Four things had to be found before the reflex could be judged**, each a measurement, none a
tuning:

1. **The head IMU's frame.** Its x axis points down when the camera is level (head joints at
   zero: camera forward = world +x, head gravity = (−1, 0, 0)). The first prior targeted x, y
   → 0 and pitched the camera 90°, collapsing the map (the ToF is on the head). Roll and pitch
   are the y and z components; the sense now carries the deviations, so level reads zero.
2. **The idle head brain holds its last pose.** The controller starts as an identity on the
   position slots (y = the sensed position), so with nothing driving it the head stays where
   the babble left it — 0.3 rad off, yawed 40°, the ToF sideways, wall contacts ×8. The honest
   gain-0 control is `motor_gain 0`.
3. **Yaw winds to its rail.** An unpriored axis that holds its position while the trunk turns
   under it reaches the rail; the adapter masks yaw after the babble (the plan's choice: yaw
   follows the trunk).
4. **The two pitch joints are a redundant pair.** Their null space (neck up, head down) is
   invisible to the sensor, and the prior's bias drifted into it until both pitches sat at the
   rails with `ctrl_damping` making no difference. The loop now owns two motors — head_pitch
   and head_roll — with the neck held at zero (H1/2: head_pitch → pitch −0.043, roll → roll
   +0.018, clean).
5. And one from the module's own note: the closed-loop model after a babble "bore no
   resemblance" to the babbled one; with the model learning while walking the prior acted
   through a drifting A and the head went to 38° of deviation. **Frozen at the identified
   values (`model_lr 0`)**, the prior does what it says.

| arm (all n = 6) | head gyro RMS (rad/s) | roll dev | pitch dev | walls/min | rescues/min | frame diff (s6) |
|---|---|---|---|---|---|---|
| control = R30, the walker's head | 1.85 ± 0.13 | 0.090 | 0.197 | 6.6 ± 5.8 | 0.05 | 12.3 |
| 4 motors, level (wrong frame) | 3.26 ± 0.72 | — | — | 46 | 0.11 | 18.6 |
| 4 motors, level, yaw masked | 2.40 ± 0.12 | 0.27 | 0.16 | 9.7 ± 8.3 | 0.03 | — |
| 4 motors, level, + ctrl_damping | 2.41 ± 0.21 | 0.23 | 0.19 | 62 ± 125 | 0.03 | — |
| 2 motors, level, model learning | 2.86 ± 0.72 | 0.62 (roll + pitch) | | 13.5 ± 7.4 | 0.05 | — |
| **2 motors, level, model frozen** | 2.11 ± 0.07 | **0.041 ± 0.002** | **0.044 ± 0.004** | 9.3 ± 8.9 | 0.02 | **16.4** |

**Verdict, three parts.** *Level*: `WORKING`, loud — pitch deviation 0.20 → 0.044 and roll
0.09 → 0.04 on every seed, the head visibly held level while the body walks (launcher R32
against R30). *Still*: `NULL` — the head gyro is 14 % worse; the pitch command sits at its
rail on half the ticks, chasing the 2 Hz gait through a one-step model and a lagged position
channel. *The picture*: `REGRESSION` — frame difference 12.3 → 16.4; the operator's criterion
is worse, because a level head that bangs the rail shakes the camera more than a walker's head
that droops 11° and stays put. **Not promoted.** Re-use context: a slower prior (the 50 Hz
Gauss–Newton step at `state_prior_lr 0.1` is a 200 ms time constant against a ~200 ms
actuator lag — the wind-up condition); a rate target on the frozen model; or feed-forward
from the trunk gyro (the controller's rows over the trunk slots are what a vestibulo-collic
reflex actually is). One lever at a time, from R32.

### 17.11 The operator's eye on H2, the slow prior, and the no-backing clamp (2026-09-10)

**The operator watched R32 and called it working:** the head stays level fore-aft "very much
like a chicken or other bird that walks"; the side-to-side tilt appears when the robot turns;
and once the robot backed into a wall and stayed there.

**Roll in turns, read from the logs.** Over the six frozen-model seeds, the head's world-frame
roll error is the same turning (|head yaw rate| > 1 rad/s) as straight (p95 0.076 vs 0.079; the
walker's own 0.12–0.18), and the roll *rate* is the walker's. What changes in a turn is the
roll *command*: its mean rises from +0.014 to +0.04–0.07 rad because the walker banks and the
head counter-rolls to stay level, and it sits at its ±0.31 rad rail on a third to a half of all
ticks, as the pitch command does at ±1.10. So the tilt seen from outside is the roll joint
working relative to a banked body, plus the same rail-chasing that shakes the camera. In the
simulator gravity comes from the orientation, so no precession reaches the sense; on hardware
the IMU's gravity estimate will carry the centripetal term and that reading will need
revisiting there.

**The slow prior** (`head2_h2_level_slow.json`, `state_prior_lr 0.02` — a fifth of the
model-implied correction per tick, a ~1 s time constant, instead of a tenth at 50 Hz against a
~200 ms head lag), n = 6: pitch deviation 0.054 ± 0.009 (the walker 0.20; lr 0.1 gave 0.044),
roll 0.048 (0.09; 0.041), head gyro 2.10 ± 0.07 (unchanged), rail-hitting pitch 39 % (from
50 %) and roll 26 % (from 45 %), frame difference on seed 6 **14.4** (control 12.3, lr 0.1
16.4). Direction right, the picture still worse than the walker's. `PARTIAL`; R33 to watch.

**The no-backing clamp** (`--no-backing`, the twist brain's forward command clamped at zero
— the body has no rear sensor on either side of the boundary, so a step backward is a step
into the unseen). The operator's R32 watch had backward commands on 35 % of ticks in the
tour, during turns, in 66 short runs (median 0.23 s, longest 3.5 s) and no wall stretch over
4 s; no harness seed of any arm has one over 4 s either. Clamped, n = 6 on R30: **walls/min
6.6 → 36.8 ± 28, path 156 → 133 m** — `REGRESSION`. The short backward commands are the loop
backing *off* a wall; forbidden, it rides walls. Kept as a flag, off by default. Re-use: a
rear sensor, or a clamp gated on "nothing ahead", neither of which this body has.

### 17.12 Yaw is the axis; the model must stay frozen; three levers refuted (2026-09-10)

**The operator saw the improvement of R33 and named yaw as the largest remaining error.**
Measured on seed 6 by splitting the brain-frame difference by the head's rotation between
consecutive frames (from the logged orientations, at the camera's 12.5 Hz): the walker's head
turns at **91°/s RMS about its vertical axis, 39 pitch, 50 roll**, and yaw carries the largest
fitted share of the frame difference. R33 brings roll to 37 and leaves pitch and yaw where
they were; yaw was masked to follow the trunk, and the gait's yaw wobble goes straight into
the picture.

Three levers on top of R33, each n = 6 with the seed-6 picture, each `REGRESSION`:

| lever | level (roll+pitch dev) | frame diff (s6) | what happened |
|---|---|---|---|
| R33 (2 motors, slow prior, model frozen) | 0.07 | 14.4 | the reference |
| F1 the model learning while walking (`model_lr 0.02`, `state_model_lr 0.05`) | 0.80 ± 0.19 | 15.7 | the identified pitch authority drifted −0.043 → −0.011 and the head left level; pitch and roll rotation 85 and 91°/s |
| F2 F1 + `lookahead_gain 1` (act on the predicted state) | 0.41 ± 0.29 | 15.9 | no recovery; walls 66 ± 96 |
| Y1 yaw as a third motor with a head-yaw-rate prior → 0 (model learning) | 0.77 ± 0.14 | 20.7 | the yaw joint at its ±1.4 rad rail 88–100 % of ticks on four seeds: a rate target with no position anchor winds up in every sustained turn |

**Two conclusions.** The frozen model is not a shortcut: the closed-loop model drifts while
walking, exactly as the module's own note warns, and with it the level goes; "be predictive
rather than frozen" has to be done *outside* the model's learning — feed-forward from the
trunk gyro that leaves the identified authority alone. And a yaw reflex cannot be a target on
the module's state prior: it needs a position anchor with a leak — the vestibulo-ocular
reflex's own form — which is the next lever (`--head-vor TAU LEAD` in the head adapter: minus
the trunk's integrated yaw rate, leaking to centre in TAU s, plus LEAD s of the rate as a
phase advance against the walker's head lag; 0 = off, byte-identical).

**The yaw reflex, measured (V1 `--head-vor 2 0`, V2 `--head-vor 2 0.1`, on R33), n = 6:**
level kept (roll + pitch dev 0.10, 0.11), head gyro 2.25 / 2.12, walls **65 ± 85 / 49 ± 28**
(R33 20), frame difference **16.6 / 19.2** (R33 14.4), head yaw rotation 95 / 94°/s (R33 85).
`REGRESSION`, both. The diagnosis from the seed-6 log: the reflex is correct and fast — the
head-yaw joint's rate is anti-correlated with the trunk's yaw rate, peaking at a 4-tick (80 ms)
lag, and the joint follows the command at 0.98 — and it still adds motion, because **the yaw
wobble is not the trunk's**. With the head command held at exactly zero (R30, seed 6), the walker's
own policy moves the head-yaw joint at **1.21 rad/s RMS (69°/s)**, the neck pitch at 0.67, the
head pitch at 0.48 and the roll at 0.45, against a trunk yaw rate of 0.79: the walking policy jitters the head joints at
the gait rate on its own, and a reflex integrating the trunk gyro cancels the wrong thing and
piles its own counter-rotation on top. What the yaw axis needs is a loop against the *head's
own* rate with a position anchor — the head gyro is the sensor, and the actuator is a policy
that wiggles what it is told to hold — or, on the real duck, a question for Pollen: their
walking policy's head-pose tracking leaves ~1 rad/s of yaw jitter at a fixed command, which a
client cannot remove through `robot.head`. That is a measurement on their MJCF with their
policy, and belongs in the outreach as a finding, not a complaint. Any head yaw also swings
the ToF off the direction of travel, which is where the wall contacts come from; the same
coupling exists on the hardware.

**Where the head line stands (end of 2026-09-10).** R33 is the head loop as it works: two
motors, identified standing, frozen, a slow level prior — level fore-aft and in roll like a
walking bird, the operator's eye confirmed, and the camera still 17 % less steady than the
walker's drooping head because the gait's yaw and the policy's own head jitter are untouched.
Refuted in this context: the fast prior, the learning model, the lookahead, a yaw-rate prior,
and the trunk-gyro reflex. The next lever is a rate loop against the head's own gyro with a
position anchor, or a step down the ladder: the walker's head jitter is the actuator's noise,
and Track A (the joints) is where it would be removed.

### 17.13 The rate loop on the head's own gyro — refuted by the actuator's lag at the gait frequency (2026-09-10)

`--head-rate K TAU` (off by default): the yaw command integrates minus K times the head's own
yaw rate (the head IMU's x component, its down axis; measured +0.34 with the trunk's yaw rate,
the same sign) and leaks to centre in TAU s — the position anchor Y1 lacked. On R33, K = 1
and 2, TAU = 2 s, n = 6:

| arm | head yaw rate RMS (rad/s) | 3-axis head rate | walls/min | frame diff (s6) | yaw rotation (s6, °/s) |
|---|---|---|---|---|---|
| R33 | 1.72 | 2.30 | 20 | 14.4 | 85 |
| G1 K = 1 | 2.03 | 2.95 ± 0.22 | 44 ± 24 | 16.3 | 94 |
| G2 K = 2 | 2.20 | 3.17 ± 0.17 | 42 ± 37 | 20.3 | 132 |

`REGRESSION`, worse with gain — the signature of a lag-limited loop. The loop does what it
says: the command's rate is −0.86 correlated with the measured head yaw rate at lag 0. But the
joint's counter-motion arrives **6–8 ticks (120–160 ms) later** — the walker's head tracking,
its policy and its low-pass — and the jitter it is countering sits at the **gait frequency**:
the head yaw rate's spectrum peaks at 2.23 Hz with 44 % of its power in 2–3 Hz, where the hip
pitch has 46 % of its own. A correction 120–160 ms late on a 450 ms period is 100–130° late:
it adds energy. (The pitch axis carries an 8.3 Hz neck resonance, 82 % of its power — a
different problem, not the gait's.)

**What this closes, and what it opens.** Through Pollen's walker no feedback loop on the head
can remove the gait's yaw jitter: the actuator answers a quarter period late, and the walker's
own policy is the source. Two routes remain, both predictive in the sense the operator asked
for. (1) **Phase, not feedback**: the jitter is periodic with the gait, so a feed-forward
locked to the gait phase can command the counter-motion a quarter period *ahead* — the
doctrine's "feed it phase" (CPG → EPM), the picrawler's stride model; the module carries a
`step_phase`/CPG apparatus already. Its honest signal is the head gyro's residual at the gait
frequency. (2) **The joints** (Track A): the jitter is the actuator's noise, removable only
where the actuator is ours. And a third for Pollen: their policy's head-pose tracking leaves
1.2 rad/s of yaw jitter at a fixed command, a measured finding on their model for the
outreach. R33 remains the head loop; the H line pauses here for the operator's decision.

### 17.14 Both routes tried: Track A at the head beats the walker's own head; the phase feed-forward finds no waveform (2026-09-10)

The operator asked for both routes measured, even if marginal, before a plan goes to Pollen.

**Track A at the head** (`--head-joints`): the head brain's two commands become the head
joint targets (HOME + command) written over the walker's head outputs; the walker keeps the
legs and is told nothing about the head. The actuator is the servo. Identified standing at the
joints (`head2j_h1_s2`: pitch authority −0.075, roll +0.018 — nearly double the walker
route's), then R33's level prior while walking, n = 6:

| arm | 3-axis head rate (rad/s) | yaw rate | roll / pitch dev | walls/min | cells | rescues/min | frame diff (s6) | yaw / pitch / roll rotation (s6, °/s) |
|---|---|---|---|---|---|---|---|---|
| walker's own head (R30) | ~2.3 | 1.72 | 0.09 / 0.20 | 6.6 | 151 | 0.05 | 12.3 | 91 / 39 / 50 |
| R33 (walker route, level) | 2.30 | 1.72 | 0.05 / 0.05 | 20 | 127 | 0.07 | 14.4 | 85 / 38 / 37 |
| **J1 joints, level** | **1.18 ± 0.04** | **1.03** | **0.025 / 0.038** | 10.9 ± 9.3 | 144 | 0.19 | **9.9** | **58 / 40 / 28** |
| J2 joints + rate loop K 1 | 1.57 | — | 0.08 | 44 | 145 | 0.27 | 11.1 | 52 / 29 / 36 |
| J3 joints + rate loop K 2 | 2.36 | — | 0.16 | 24 | 163 | 1.05 | 14.9 | 68 / 89 / 96 |

**J1 is the first arm steadier than the walker's own head on the operator's criterion**:
the picture 12.3 → 9.9, the head's world rate halved, yaw 91 → 58°/s (the policy no longer
jitters the joint; what remains is the trunk's own yaw), roll halved, level held at 0.03–0.05
on every seed, the tour intact. The cost: rescues 0.05 → 0.19/min — the walker's balance
apparently used the head it no longer moves (the head is 38 % of the mass), a number to carry
into the ask. The rate loop on top of it fails again, now for a different reason: at the
joints the lag is the servo's, but the remaining yaw is the trunk's, which the loop fights
through a policy that then falls (J3 1.05 rescues/min). `WORKING`, loud, seed-consistent.

**The gait-phase feed-forward** (`--head-phase LEAD LEARN_S`): a stride clock from the hip
pitch's upward crossings of its running mean (found: 2.3–2.7 Hz, thousands of crossings), a
16-bin table of the head's yaw rate learned for 100 s after the babble with the command at
zero, integrated into the periodic yaw angle and commanded with the opposite sign LEAD ticks
early (7 through the walker, 2 at the joints). Measured, n = 6: **the table explains nothing**
— the residual against it equals the measured rate (1.58 of 1.59 through the walker, 0.84 of
0.85 at the joints) and the learned angle waveform is ±0.014–0.017 rad, one degree. The yaw
jitter is at the stride rate without being phase-locked to the stride. Through the walker
(P1): head rate 2.39 (R33 2.30), picture 18.2, `REGRESSION`. At the joints (P2): head rate
1.09 ± 0.23 vs J1's 1.18 — a tie within seeds — while the tour collapses (cells 82 vs 144,
seed 3 riding a wall at 125/min with 36 cells); its seed-6 picture of 8.7 beats J1's 9.9 for
the wrong reason: **the picture is blind to a duck that stops touring**, and must always be
read with cells and path. `NULL` on the head, `REGRESSION` on the tour. Re-use: a phase
estimate from the head's own rate rather than the hip; or the EPM form over (phase, rate) if
the jitter turns out to be locked to something other than the hip.

**The comparison the operator asked for.** Track A at the head wins, and not marginally:
on the picture it is the only arm better than the walker, and it is better on every head
number with the tour intact. Everything through Pollen's walker command — the level prior
(R33), the trunk reflex, the head-gyro rate loop, the phase feed-forward — is bounded by two
facts of their walker: it answers a head command 120–160 ms late, and its own policy jitters
the head-yaw joint at 1.2 rad/s. That is the case to take to Pollen (outreach plan §8).

**PROMOTED (the operator's eye, 2026-09-11):** "a significant improvement in stability compared
to R30. While the head is not perfectly still the wobble is tolerable when viewing through the
head camera, and the overall look of the duck walking is more interesting and birdlike. It is
a win." R34 is the head loop: `★ HEAD` in the launcher; R32 retired from the list (its file
kept); R33 kept as the walker-route comparison for the Pollen case. Before PR-2 the operator
wants the behaviour set validated in the simulator with what exists — the exploration line
resumes (playroom plan ▶ Resume here).

### 17.15 The exploration line's control arm: the ToF's own 8×8 in the place map (2026-09-11)

**The question** (operator, 2026-09-11: lean on the ToF; the camera stays a demonstration for
the ask to Pollen). Today's place vector is `[x/2, y/2, cos, sin, the 8 column ranges / 4 m]`
(§17.3): eight ranges over a 45° cone look nearly the same from most spots in a 4 m room, so what
tells one map node from another is mostly the odometry — a grid over a path integral wearing an
EPM (the Cell audit's V1, on the duck). Before the camera (playroom plan C2) the cheapest richer
geometry is the sensor's own 8×8 depth matrix, on the wire on the real robot. This is the arm the
camera arm has to beat, and the pipeline it will share with the sensor swapped.

**The host.** `IntentAdapter` now owns the place vector's *form*, read from the graph and echoed
at start (`place vector: …`, captured by `l2_sweep` as a read-back): `Columns` when the map EPM on
`reality.proprio.place_in` declares 12 (every existing config; byte-identical — R34 seed 6, 400 s,
md5 `cc87df84…` before and after), `Zones` at 68, `Stacked` when the graph has an EPM on
`reality.proprio.depth_in`. The host measures the pose and the ToF in both reductions every tick
(`PlaceInputs`); the adapter publishes what the graph asked for.

**R35 — the 64 zone ranges straight into the RBF place EPM: the encoder flattens it.** Seed 6,
1500 s: 10 map nodes against R34's 67; the map's TLE 0.04–0.08 through the babble against
0.1–0.2. Not the input: the logged 64-zone input needs 7 principal dimensions for 90 % of its
variance, the 8 columns 6, and its raw pairwise spread is twice the columns' (0.258 vs 0.124).
Re-encoding the logged control phase offline through the encoder's own rule (Halton centres,
σ = 0.8 × mean nearest-centre distance, L2-normalised activations):

| form | latent pairwise spread | 1 s apart |
|---|---|---|
| 12-D RBF as configured (96 centres, σ 0.61) — R34's map | 0.092 | 0.025 |
| 68-D RBF as configured (544 centres, σ 1.32) — R35 | 0.048 | 0.016 |
| 68-D RBF, per-dim ranges commissioned from the data | 0.050 | 0.017 |
| 68-D RBF, depth centred and commissioned | 0.049 | 0.017 |
| 36-D RBF over [pose ; a 32-D JL of the depth] | 0.047 | 0.018 |
| `jl_state` 128 over [pose ; frame-centred depth] | 0.109 | 0.035 |
| [pose ; 64-D JL of the frame-centred depth], Euclidean | 0.173 | 0.051 |

The RBF grid's bandwidth in 68 dimensions makes every input the same activation profile, and
neither commissioned ranges (`dim_autocal`'s form) nor centring recovers it; a JL projection, which
preserves distances at any width, keeps the spread. `CLAUDE.md` §0 rule 2 in its other direction:
the PCA said "several", the node count said "one", and the encoder was the reason. R35 is a
measurement of the encoder, not of the idea (§3.2 rule 6, a weakened slice); its config stays for
the record, no preset.

**R36 — the stacked form, the plan's O10 with the sensor swapped.** A new EPM encoder kind,
`jl_state` (`cpp_core`: the frozen JL projection over a `ProprioToken`, `FrozenJLEncoder::
make_state_encoder`; takes no per-dim ranges; three unit tests; every existing config
byte-identical). `depth_epm`: `jl_state` over the 64 zone slant ranges / 4 m with the frame's mean
taken out (the host; the common mode is mostly the floor in the lower rows), 64-D latent, the same
insertion gate and node budget as the map. `map_epm`: `jl_state` over `[pose ; depth latent]`
(68 → 64), the RBF and its ranges gone, everything else R34's. Playroom, ★ HEAD stack, n = 6,
1500 s, control 700–1500 s, paired with R34:

| n = 6, paired | walls/min | cells | path m | distinct map winners | map TLE | objs/min | down % | resc/min | headW rms |
|---|---|---|---|---|---|---|---|---|---|
| R34 ★ HEAD | 10.9 ± 9.3 | 144 ± 20 | 146.6 | 92 ± 13 | 0.18 | 16.8 | 0.38 | 0.19 | 1.18 |
| **R36** depth stacked | **17.6 ± 12.2** (Δ +6.6, t +2.3, **6+/0−**) | 136 ± 27 (Δ −8, t −0.8) | 144.7 | **162 ± 42** (Δ +70, t +4.3, 6+/0−) | **0.24** (Δ +0.06, t +4.9, 6+/0−) | 17.8 | 0.40 | 0.18 | 1.17 |

Per seed the map's live vocabulary ends at 57–110 nodes (R34: 55–78) with 105–197 *distinct
winners* over the control phase — more distinct winners than the 128-node budget on four seeds,
so the vocabulary churns (health-death pruning and re-insertion), and the map's running TLE ends
at 0.17–0.40 (R34: 0.13–0.19). The depth EPM itself ends at 70–115 nodes, 41–50 baked, TLE
0.20–0.52: the room has that many distinct views at this sensor's resolution, and it does not
settle in 1500 s either.

**Verdict, R36 as built: `REGRESSION` on wall contact (six of six seeds), `NULL` on coverage,
ties on objects, falls and the head.** The mechanism reads straight off the numbers: the play loop
climbs the map's TLE (R27), the depth-stacked map's TLE is higher and churning, and what is
novel in it is what is *close* — a wall or a chair leg fills the frame with a pattern the
vocabulary has not settled — so the bearing to novelty is the bearing to the nearest surface.
§17.6's finding ("a learned direction beats the proximity priors") with a richer novelty field
behind it. Context of the verdict: the play loop over the raw place TLE; R34's insertion gate
(`min_insertion_error 0.06`, an absolute threshold set for the RBF latent's error scale) carried
onto a latent whose errors run a third higher; the node budget unchanged; no (d) moved-object run.

**The gate in its adaptive form does not rescue it.** The first suspect was the insertion gate:
R34's `min_insertion_error 0.06` is an absolute threshold set for the RBF latent's error scale,
carried onto a latent whose errors run a third higher. `insertion_autotune true` on both EPMs
(the rank-based gate the EPM already has, `--arm` from R36, same seeds):

| n = 6, paired | walls/min | cells | path m | distinct map winners | map TLE | objs/min | down % |
|---|---|---|---|---|---|---|---|
| R36 + adaptive gate | **45.8 ± 39.2** (vs R34: Δ +35, t +2.6, 5+/1−; two seeds ride the walls at 80 and 102/min) | 135 ± 24 (ties) | 133 (Δ −13, **0+/6−**) | 150 ± 28 | 0.23 | 13.6 | 0.34 |

Worse, not better: the vocabulary still churns (94–170 distinct winners against 57–106 live
nodes), and the path shortens on every seed because the body spends it against walls. The gate's
scale was not the mechanism.

**Re-use context.** What would justify the next try, in order: the novelty the play loop climbs
taken from *baked* nodes or the transition term rather than the raw TLE of a churning vocabulary
(so a place is interesting once the vocabulary has stopped moving under it, not while it is being
tiled — the churn is the finding: the map now has enough resolution that it never stops
re-tiling in 1500 s, and play chases the tiling); the avoidance loop's bearing arbitrated against
play's (R28/R29's machinery, gain-0 on this stack); a node budget and a prune policy sized to the
room's actual view count (the depth EPM says ~100 at this resolution). The moved-object (d) has
not been read on any arm. **For C2 the pipeline is built:** the camera arm is R36's graph with
`depth_epm`'s input swapped for the rendered frame — and the same trap waits for it, so E1 should
be designed around the churn before the camera is rendered, not after.

### 17.16 The operator's eye on R36, the dither measured, and the hold refuted (2026-09-11)

**The observation.** Watching R36 for 1000 s the operator saw the duck circling in the centre of
the room with many similar map nodes there, and read it as: it needs to hold a trajectory for
longer to find novel places. The harness agrees about the circling and it is not R36's alone —
R34 does it too. Seed 6, control phase, both arms: **286° turned per metre travelled**, mean yaw
rate 50°/s, the yaw command at |0.95| of its range with **48–50 sign flips a minute** (same-sign
runs of 0.3 s), straightness over 20 s windows 0.24 (R34) and 0.14 (R36), where 1 is a line and 0
is back where it started.

**The mechanism, from the logs.** The map's current node changes **130–155 times a minute**
(dwell 0.2 s): its nodes' pose centroids are 8–13 cm apart and each node's visits spread over
23–47 cm, so several nodes claim the same spot and the winner flickers between them. The play
loop drops its committed sub-goal whenever the current node is not adjacent to it, so the target
is re-chosen several times a second; the bearing to a fresh target — usually inside the body's
measured **turning radius of 0.18 m** (v/ω while moving and turning) — saturates the yaw command;
the body swings past it and the next flicker picks another. New nodes are minted while walking,
not while turning in place (0 % of mintings at low speed and high yaw rate), so the depth map
does not mint novelty by spinning; R36 only makes the flicker denser (195 winners against 97).

**R37 — `commit_hold` on the play loop, the operator's hypothesis in the loop's terms.** New
`PlayLoop` param (off by default, byte-identical — R34 seed 6 md5 `cc87df84…` unchanged; a unit
test): the committed node is held until *reached* (it becomes the current node) or until it is no
longer uphill in the value field, adjacency no longer required, and the bearing is taken from the
loop's own odometry to the target's position rather than from the flickering node's centroid. No
timescale is set. The sweep gained two columns for this: `switch/min` (winner switches) and
`straight` (the 20 s straightness, the complement cells and path are blind to). Playroom, ★ HEAD
stack, n = 6, paired with R34:

| n = 6, paired | walls/min | cells | straight | switch/min | distinct winners | resc/min | down % |
|---|---|---|---|---|---|---|---|
| R34 ★ HEAD | 10.9 ± 9.3 | 144 ± 20 | 0.19 ± 0.07 | 133 ± 16 | 92 | 0.19 | 0.38 |
| **R37** hold | 33.1 ± 37.4 (3+/3−) | **73 ± 50** (Δ −70, t −5.5, **0+/6−**) | **0.10 ± 0.08** (Δ −0.10, t −4.4, **0+/6−**) | 114 ± 19 (Δ −19, t −2.2) | 58 ± 30 (Δ −34, t −4.5) | 0.05 (Δ −0.14, 0+/6−) | 0.12 |

Per seed it splits into two failures: seeds 3, 4 and 6 **orbit** (straightness 0.07–0.10, 23–63
cells on 155 m of path, the y-range down to 1.2 m, zero walls) and seeds 1, 2 and 5 **ride the
walls** (41–91/min, straightness 0.03). Holding the target does exactly what a target inside the
turning radius predicts: the body circles it forever, "reached" never fires because the winner
never settles on that node while the body orbits its centroid, and the value criterion never
releases it because the orbit learns nothing. Fewer falls (0.05/min) because it walks less into
things it cannot see. **`REGRESSION`**, six of six on the two metrics that matter, preset removed,
config kept.

**What the pair R36/R37 settles.** The dither and the orbit are two faces of one geometry: the
play loop's targets are one hop away on a map whose hops are shorter than the body can turn.
Dropping the target on every flicker gives a dither that at least drifts; holding it gives an
orbit. Neither "how long to hold" nor "which sensor feeds the map" is the lever. The re-use
context, in error terms, is a **fork for the operator**, because each option changes something
the plan or the recipe has stated:

1. **A target beyond the turning radius** — follow the value gradient several hops and bear on the
   first node whose position is at least the body's turning radius away; arrival geometric
   (within the target node's own pose spread, which the loop already accumulates). Derivable, no
   timescale; but the recipe states the horizon as one step (§"Horizon", register O2), and this
   is a longer horizon for the *bearing*, not the arbiter.
2. **Places at the body's scale** — a place node that is a cell of the loop's own path integral
   (`pi_cell_size` > 0, the grid the Cell audit demoted) or a map EPM whose insertion gate is set
   so that nodes are farther apart than the turning radius. The second is a scale constant
   (prohibition 5) unless derived from the body's motion.
3. **The heading regulator** — the yaw command is bang-bang (|0.95| on average); a law whose gain
   comes from the identified yaw row of A (the read-back prints it) would turn a saturated swing
   into a proportional one and shrink the orbit radius the dither produces. This changes the
   level-2 twist brain, not the play loop.

The instruments for any of them are now in the sweep (`straight`, `switch/min`).

### 17.17 Fork item 1 tried (R38, the lookahead target) — refuted, and the heading regulator found not to regulate (2026-09-11)

**R38 — `lookahead` on the play loop** (operator: try item 1). New `PlayLoop` params, off by
default (R34 byte-identical, a unit test): the sub-goal is the first node along the greedy uphill
walk of the value field whose position is at least the body's turning radius from the loop's own
odometry — held until the body is inside that radius or the node is no longer uphill, the bearing
from the live odometry; the radius is the loop's own running estimate of forward speed / heading
rate on turning ticks (no constant; `lookahead_reach` > 0 overrides it for tests). The read-back
prints it (`play {… "reach": …}`, now a `diag_lite`): 14–22 odometry units on five seeds ≈
0.11–0.17 m against the 0.18 m measured from the walk; seed 5 estimated 3.3 (its turning ticks
were slow ones). Playroom, ★ HEAD stack, n = 6, paired with R34:

| n = 6, paired | walls/min | cells | straight | switch/min | path m | resc/min | objs/min |
|---|---|---|---|---|---|---|---|
| R34 ★ HEAD | 10.9 | 144 ± 20 | 0.19 ± 0.07 | 133 | 147 | 0.19 | 16.8 |
| **R38** lookahead | 8.4 (ties, 3+/3−) | **110 ± 52** (Δ −34, t −2.1, 0+/5−) | **0.13 ± 0.06** (Δ −0.06, t −4.9, **0+/6−**) | 132 (ties) | 158 (Δ +11, 5+/1−) | 0.06 | 6.0 |

Straighter it is not: worse on every seed. Seed 4 reproduces R37's orbit **to the metre** (23 cells,
159.1 m, x-range 1.2) — the fallback clause ("the last uphill node if none is that far") re-chooses
the same near node every tick at a local peak of the value field, which is R37's held target by
another route. **`REGRESSION`**, preset removed, config kept. The lever's own premise held (the
target was far and the reference quiet: 3 reference jumps a minute against R34's 102) and the body
still did not go straight — which is the finding.

**The heading regulator does not regulate.** The host now logs the heading and its reference
per tick (`hdg`, a printed field; the physics byte-identical with it stripped), so the twist brain's
yaw channel can be read directly. Seed 6, control phase:

| | R34 | R38 |
|---|---|---|
| heading error \|e\| (rad), mean | 1.76 | 1.44 |
| ticks with \|e\| > 0.8 rad | 80 % | 72 % |
| \|vyaw\| command (of ±1) | 0.95 | 0.92 |
| yaw-command sign flips / min | 50 | 140 |
| reference jumps > 0.5 rad / min | 102 | 3 |
| in windows where the reference is quiet: \|e\|, flips/min | 2.08 rad, 62 | 1.91 rad, 192 |
| vyaw spectral peak | broadband | **2.39 Hz — the gait** (sensed yaw rate 2.41 Hz) |
| linear fit of vyaw(t) on the senses(t−1): R² | 0.63 | 0.37 |
| dominant term | heading error (coef −0.93, corr −0.76) | ToF right column (+1.97); heading error −0.12 |
| own-yaw-rate copy: command has the same sign when \|rate\| > 0.3 | **83 %** | 65 % |

In R34 the yaw command *does* answer the heading error — but the reference it is held to moves at
4 rad/s because the play target flickers, so the error never closes, and a positive-feedback term on
the body's own yaw-rate copy (R21's finding, §17.2: "dominated by the copy of its own yaw command")
sustains the spin. In R38 the reference stands still and the learned yaw row lets go of the error
altogether: learning is live through the control phase, the state prior's descent (`state_prior_lr
0.1`, through the identified A) is one term among the C matrix's sixteen columns, and with nothing
to chase the row settles on whatever else moves at the gait frequency — the right ToF column, the
velocity copies — a saturated 2.4 Hz oscillation. Either way the body is not steered: with the
reference still, the error sits at two radians.

**Verdict on the fork.** Items 1 and 2 are moot while item 3 stands: nothing above the twist brain
can hold a trajectory if the twist brain does not hold a heading. The heading loop's earlier
verdicts (R22 `PARTIAL`, R27's "steers on every tick") measured that the reference was *set*, never
that it was *followed*; this is the first measurement of the following, and it is the §3.2 catch
of the line. **The lever is the yaw channel of the level-2 twist brain**, in the doctrine's terms:
the prior's own error on slot 10 through the model's yaw authority (A's vyaw row, printed in every
read-back) as the yaw command's *objective*, with the learned C's other columns unable to swamp it.
Two honest forms, the operator's call: (a) a **lesion** first — hold C's yaw row to the heading
column alone and read straightness (if it jumps, the swamping is proven and the mechanism is the
prior's weight, not a new law); (b) the **model-implied step** — the yaw command that closes the
prior's error in one identified step, `u = −e / A(idx, vyaw)`, clamped, which is what the prior's
half 2 is meant to converge to and here does not. The positive feedback on the own-rate copy is a
second, older defect (R21) that a lesion would also expose. `straight` and the `hdg` field are the
instruments; the bar is `straight` well above 0.2 on every seed *with* the play reference live.

### 17.18 W1 — the hand-back at the joints: scheduled stops, and the R19 stander takes the legs (R39, 2026-09-11)

**The line.** The walk-stop-look line (playroom plan §12) needs one transition before anything
else: a walking duck stops, and the joint brain that stands and catches (R19) takes the legs from
whatever pose the walker leaves them in. W1 measures that transition with a scaffold stimulus — a
stop schedule — before §12.2's stop, which is the map's, exists.

**Built.** `--stop-every S --stop-secs S --stop-from S` on the level-2 host: the twist is zeroed and
the walker stands; once still (the identification settle's own criterion, at most 2 s) the legs —
and the head, its validated regime — go to the joint brain named by `--stop-brain CFG --stop-load
CKPT`, calibrated and restored exactly as `--brain` does it; the walker takes them back past 6.5° of
rising lean (the step hand-off's threshold, `--stop-handoff-lean`, or the brain's attitude error,
`--stop-handoff-att`) and at the stop's end. `--stop-att X` gates the hand-back on the joint brain's
own attitude error (0 = ungated); `--stop-brain walker` is the control arm — the same stops, the
walker holding them; `--stop-keep-head` leaves the head brain on the head (unmeasured). Both brains
that do not drive are frozen and reset at the edges (the H2 lesson, §17.10). Absent, the tick
stream is byte-identical (10 000 ticks of R34 compared). The JSONL gains `drive: stand`, the events
`stop:start / handback / refused / handoff / end / rescued / walker`, and `stop` (the phase) with
`satt` (the joint brain's attitude error) when the schedule is on. **W0**, the ten-minute
instrument, is in `l2_sweep.py`: the behaviour histogram (`walk% stopW% stand% resc%`) and the stop
counters, plus `--host-arm NAME:'ARGS'` for a lever that lives in the host. The viewer draws the
stand green. R39 = R34's graph with the stops in the host args; preset on the ★ HEAD controls.

**Measured.** Playroom, ★ HEAD stack, n = 6, 1500 s, stops every 60 s for 20 s from 600 s (15 per
run, 14 in the 700–1500 s control phase), paired by seed:

| n = 6 | walk% | stopW% | stand% | walls/min | cells | path m | straight | switch/min | resc/min | hand-backs | survived | handed off | rescued |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R34 ★ HEAD | 99.1 | 0 | 0 | 10.9 ± 9.3 | 144 ± 20 | 147 ± 4 | 0.19 | 133 ± 16 | 0.19 | — | — | — | — |
| stops, the walker holds | 67.2 | 32.0 | 0 | 29.9 ± 42.5 (3+/3−) | 70 ± 36 | 86 ± 32 | 0.15 | 76 ± 16 | 0.08 | — | — | — | — |
| **stops, the joint brain stands** | 66.9 | 2.5 | **29.5** | 18.1 ± 23.6 (3+/3−) | 106 ± 26 | 97 ± 6 | 0.16 | 103 ± 13 | 0.17 | **89 / 90** | **87** | 2 | 1 |

- **The hand-back is loud.** 89 of 90 stops handed the legs over (one stop was cut by a rescue
  before stillness), 87 of 89 stands held to the stop's end, 2 went back to the walker on lean, 1
  fell — all three on seed 1. Over the 77 control-phase hand-backs the tilt goes 2.23° at the
  hand-back → 0.49° one second later → 0.39° at five; the joint brain stands at **0.49° mean
  tilt** where the walker's own stand sits at 3.0–3.5° (walker-held stops, both arms). The
  transition lurch is 0.02 rad/tick for ten ticks; the walker's resumption is the larger jump.
- **The gate had nothing to gate.** `--stop-att` was 0 (ungated), and what a gate would have seen —
  `satt` at the hand-back — is 0.0000, max 0.0020, on every hand-back: the joint brain's attitude
  error at the walker's settled stance is not a discriminating competence signal. The
  competence-gated hand-back of §12.3 is therefore **unmeasured**, not confirmed (a §3.2 rule 1
  catch made before the verdict, not after). Its re-use context: a hand-back from a walker that has
  not settled (`--stop-settle-secs 0`), or from a shoved body, where the error is not ~0.
- **The stops cost a third of the tour, as they must.** Cells 144 → 106 and path 147 → 97 m are the
  stimulus (30 % of the phase standing), not a regression; `switch/min` falls 133 → 103 because the
  map stops re-tiling while the body is still — §12.2's premise in miniature.
- **The §3.2 catch: a stop exposes §17.17's saturation as a deadlock.** In the walker-holds arm,
  seed 1 resumes from a still body with the wall behind it; the twist command saturates at full
  reverse (`[−0.40, −0.30, −1.0]`) and the body is pinned to the wall for the remaining 900 s — 14
  of 14 windows pinned, 98 walls/min, 3 cells. The ToF faces away from the wall, and because the
  body does not move nothing perturbs the loop; in R34 the same defect flips at 2.4 Hz because the
  body is always moving. Under the joint brain the same seed is pinned for one window (1080 s) and
  leaves through a lean → hand-off → rescue. That deadlock is the whole of the walker arm's walls/min
  and the reason both stop arms tie R34 on walls (3+/3−). It is the resume-from-still form of the
  twist brain's yaw defect and raises W5's priority: §12.2's turn-in-place under a homing target must
  begin from exactly this state.
- **W0 reads the stack as the operator saw it:** R34 is 99.1 % walk and 0.9 % rescue — the Roomba,
  in one row. The stop arms are 67 % walk / 32 % stop; the joint-brain arm 29.5 % stand.

**Verdict.** W1 `WORKING` at n = 6, loud on the transition: the stand survives 87 of 89 hand-backs
and is stiller than the walker's at every one. Not a promotion — the stop is a schedule and the
operator's eye is pending (preset R39, seed 6, fast-forward through the babble). The competence gate
`DEFERRED` with its re-use context above; `--stop-keep-head` unmeasured; the resume-from-still
deadlock recorded against §17.17 (W5). Next: W2, the stance-gated head yaw at the stop.

### 17.19 W2 — the saccade channel: head yaw at the stop, and the stand under a swinging head (R40, 2026-09-11)

**Built.** The head brain's yaw command, masked to zero since §4b (yaw follows the trunk), takes an
override at a stop: `HeadAdapter::set_yaw_override`, driven by `--stop-scan AMP HOLD` — the yaw
steps through 0, +AMP, 0, −AMP, each held HOLD s, from the hand-back (or the walker's hold) to the
stop's end, then the mask returns for the walk. The scan is a **scaffold for the channel**: §12.2's
target is the map's residual (W3); what W2 measures is whether the stand survives a head that moves
(38 % of the mass), the yaw excursion at stops against zero on the walk, and the walk untouched.
Two more flags fell out: `--stop-keep-head` (the head brain keeps the head through the stop, the
joint brain's `motor_epm_head` frozen for the run by a new `OgmaBrainAdapter::freeze_module`, since
its commands are not applied) and `--stop-freeze-head` (the head brain's learning off through the
stop even when it keeps the head). With none of them, the W1 path is byte-identical (10 000 ticks).
The sweep reports `yawStop sd` / `yawWalk sd` (the head-yaw joint's spread in each phase).

**Measured.** Playroom, ★ HEAD stack, the R39 stops (every 60 s for 20 s from 600 s), n = 6, 1500 s,
two sweeps paired by seed (the reference arm of the second is the first's `keepHead`, and it
reproduces it to the digit — the host is deterministic per seed):

| n = 6 | head at the stop | walls/min | cells | resc/min | stands held | handed off | rescued | yawStop sd | yawWalk sd | headG dev |
|---|---|---|---|---|---|---|---|---|---|---|
| R39 (W1) | the joint brain's | 18.1 ± 23.6 | 106 | 0.17 | 87 / 89 | 2 | 1 | 0.01 | 0.01 | 0.12 |
| keepHead | the head brain's, learning live | 35.1 ± 9.5 (4+/2−) | 118 | 0.12 | **89 / 89** | 0 | 0 | 0.00 | 0.01 | 0.04 |
| keepHeadScan | the head brain's + the scan | 29.7 ± 18.4 (5+/1−) | 99 | 0.10 | **90 / 90** | 0 | 0 | **0.40** | 0.02 | 0.04 |
| walkerScan | the walker holds + the scan | 14.3 ± 18.0 | 112 | 0.01 | — | — | — | 0.40 | 0.02 | 0.02 |
| frozenHead | the head brain's, frozen through the stop | 18.6 ± 19.1 (1+/5− vs keepHead) | 118 | 0.11 | 89 / 89 | 0 | 0 | 0.00 | 0.01 | 0.04 |
| **frozenHeadScan** | frozen + the scan (**R40**) | **13.3 ± 4.8** (0+/6− vs keepHead, t −5.0) | 102 | 0.13 | **88 / 88** | 0 | 0 | 0.40 | 0.02 | 0.04 |

- **The channel works and the stand does not care.** Head yaw spreads 0.40 rad at stops (±0.61, the
  joint tracks the step inside a few ticks) and 0.02 rad on the walk; every hand-back with the head
  brain on the head held to the stop's end — 267 of 267 across the three keep-head arms, zero
  handed off, zero rescued at a stop — where the joint brain's own head (W1) gave 87 of 89. The head
  is also more level at the stop (gravity deviation 0.12 → 0.04): the R19 brain's head objective is
  its own stance, not a level camera.
- **The §3.2 catch, pinned in the same session.** The two keep-head arms with the head brain's
  learning live raised wall contact through the *whole walk* (26.6 → 44.8 episodes per minute of
  walking; not clustered at the resumes), on every seed. Hypothesis: the level prior's descent
  (`state_prior_lr 0.02`, the model frozen) kept integrating through 20 s stands on a body it was not
  identified on — the H2 drifting-model lesson (§17.10) one more time. Test: `--stop-freeze-head`.
  Result: 35 → 19 walls/min and, with the scan, 13.3 ± 4.8 (0+/6−), below R39 and level with R34's
  10.9 ± 9.3. The rule generalises: **a brain that keeps its actuator through a regime it was not
  identified in is frozen through it, whether or not its commands are applied.**
- **The stops themselves raise walking contact** (R34 10.9/min → R39's walk phase 26.6/min): a
  resume from stillness is §17.17's defect in its mild form, and the R39 seed-1 deadlock its severe
  one. W5's ground.

**Verdict.** W2 `WORKING` at n = 6, loud: the saccade channel is live at the stops and silent on the
walk, the stand is indifferent to the head moving, and R40 (frozen head brain + scan) is the
cleanest stop arm measured. Not a promotion — the scan is a schedule, and the operator's eye is
pending (preset R40, seed 6). `--stop-keep-head` without `--stop-freeze-head` is `REGRESSION` on
wall contact (re-use: a head brain whose prior is identified standing as well as walking).
Next: W3 — the stop inserts into the map and the saccade's target is the map's own residual by
bearing, which replaces the scan.

### 17.20 W3 — a place is a stop: the map learns only while standing, a view is pose + gaze, the look is the map's (R41, 2026-09-12)

**Built.** Three pieces, each a flag on the level-2 host. (1) `--map-on-stop`: the map EPM's
insertion, prototype adaptation and stale pruning are off while the body walks and on while it
stands and looks (`IntentAdapter::set_map_learning`, through the EPM's hot-mutable
`min_insertion_error`, `epsilon_b`, `epsilon_n`, `stale_prune_enabled`); the token keeps
publishing, so the play loop's node positions stay live. (2) A 13-dim place form: pose, **the
head-yaw joint / its range**, the 8 ToF column ranges — a *view*, so each bearing at a stop is its
own node (`map_epm.proprio_state_dims 13`, config R41). (3) `--stop-look AMP HOLD MAX`: at a stop
the head steps 0, +AMP, −AMP; each bearing is held at least HOLD s and, **while the view's winner
is not a baked node**, up to MAX s; a full round with nothing unbaked ends the stop early
(`stop:bored`). The baked set is a host-side lookup keyed by the EPM's own ids from the token's
`just_baked`. Read-backs: map nodes grown on walks vs at stops, saccades, holds extended, stops
ended by a quiet round, mean stop length. R34 and W1 paths byte-identical.

**Two §3.2 catches before the numbers counted.** The first sweep's `look` and `lookLive` arms
were byte-identical: the gate had read the EPM's live parameters through `Module::current_params`,
which the EPM does not override (it is empty; MotorEPMv2's is not — why the joint-level freeze
works), so nothing was changed. The gate now restores the graph's configured values and prints
its growth read-back (2 nodes on walks — the GNG's two seeds — against 33–48 at stops; 13 on walks
with the gate off). The second: the look's first novelty signal was the token's adaptive
`is_novel`, which fires on ~25 % of ticks in every arm — a percentile, not "this view is unbaked"
— and ended 1 stop in 15 early. §12.2 says *hold until the view bakes*; the baked-set test is that.

**Measured.** Playroom, ★ HEAD stack, the R39 stops (every 60 s for 20 s from 600 s), the head
brain on the head and frozen through the stop (R40's form), n = 6, 1500 s, paired by seed; the
reference arm is the R41 graph with no look:

| n = 6 | walls/min | cells | path m | nodes | switch/min | mapTLE | stands held | saccades | holds extended | stops ended early | stop s |
|---|---|---|---|---|---|---|---|---|---|---|---|
| R41 graph, no look | 33.1 ± 54.8 | 91 ± 21 | 89 | 60 ± 23 | 91 | 0.14 | 90 / 90 | — | — | — | 20 |
| scan13 (W2's scan on the 13-dim map) | 21.8 ± 37.6 | 97 | 97 | 64 | 103 | 0.16 | 90 / 90 | — | — | — | 20 |
| lookLive (the look, map learning everywhere) | 16.5 ± 15.3 | 88 | 112 | 70 ± 27 | 119 | 0.16 | 88 / 90 | 79 | 38 | **41 %** | **15.3** |
| **look + map-on-stop (R41)** | **5.4 ± 4.4** (1+/5−) | **102 ± 25** | 106 | 44 ± 5 | 94 | 0.23 | **89 / 90** | 84 | 52 | 22 % | 18.4 |
| for scale: R34 ★ HEAD (no stops); R40 | 10.9 ± 9.3; 13.3 ± 4.8 | 144; 102 | 147; 101 | 92; — | 133; — | 0.18; — | —; 88/88 | | | | |

- **The map is the stop's.** Growth 2 on walks vs 33–48 at stops on every seed; 4.8 distinct
  winners per stop (the three bearings and their transitions), 44 nodes for 15 stops. Places land
  at the scale of stops, as §12.2 predicted, and the walk between them is the cleanest measured:
  **5.4 walls/min**, below R34's 10.9 and R40's 13.3, with cells up (91 → 102) and path up
  (89 → 106). The play loop's targets are now stop views rather than a tiling of the walk, and
  `switch/min` stays at 94 where the live map's rises to 119.
- **The stand is unchanged** (89 / 90, zero rescues at stops) with the head looking 250–280 s a run.
- **Contingency, half present.** Stops end early on a quiet round (22 % under map-on-stop, 41 %
  with the live map) and the look extends 52 of 84 holds on an unbaked view. But under
  map-on-stop the stop length does **not** fall over the run (stops 1–5: 18.1 s; 11–15: 17.9 s),
  while with the live map it does (18.2 → 13.2 s). The reason is the bake rule: a view bakes after
  50 processed ticks as winner (5 s at `process_every_n_ticks 5`), which a 4 s hold cannot supply
  in one visit, and 15 stops across a 4 m room rarely revisit a pose and bearing; the live map
  bakes its views on the walks instead. So the stop length tracks *what the map has baked*, and
  this run's map-on-stop has baked too little for it to shorten. `mapTLE` 0.23 says the same:
  the walk is measured against stop views only.
- **`is_novel` is not a novelty gate.** Recorded for every consumer of the token: it fires on a
  quarter of ticks by construction (an adaptive threshold on the TLE's spread); "unbaked" is
  the question §12.2 asks.

**The (d) test, three forms (2026-09-12).** A moved *ball* (R30's form, `--move obj_ball0 -1.2 0.8
1100`): stops within 1 m of its new place did not lengthen (14.0 s, n = 9, none touching it) — the
ball is 0.06 m, one ToF column at a metre, and the compact logs cannot say whether a stop faced
it: inconclusive, a stimulus below the sensor. A moved *chair* (`--move furn_chair0 0.3 0.3
1100`, four legs at the room's centre), stops still capped at 20 s: the map re-infers (nodes after
the move 42.7 vs 34.5 unmoved; cells 78 vs 54; stops near the new chair see 7.1 distinct views
against 4.5; 2 of 16 end on a quiet round against 6 of 23) but the length cannot show — 14 of 16
run to the cap. **The chair with the stop uncapped** (`--stop-every 80 --stop-secs 60`: the stop
ends on a quiet round or at 60 s, §12.2's stop, the map's own), n = 6, paired:

| uncapped stops, n = 6 | all stops before 1100 s | all stops after | at the chair's new place, before | **after** | elsewhere after | stops at that place after |
|---|---|---|---|---|---|---|
| unmoved | 37.4 s (n = 42) | **34.5 s** (n = 23) | 39.4 s (n = 13) | 24.5 s (n = 3, all quiet) | 36.0 s | 3 |
| chair moved at 1100 s | 37.2 s (n = 42) | **42.8 s** (n = 24) | 38.6 s (n = 13) | **45.7 s** (n = 15) | 38.0 s | **15** |

Both halves of the contingency are now present at signal strength: with nothing moved the stops
shorten as the map bakes (37.4 → 34.5 s; 10 of 11 end on a quiet round by the run's end), and a
moved chair lengthens the stops at its place (38.6 → 45.7 s) and draws the duck there (3 → 15
stops within 1.2 m; the play loop's novelty climbs to the change, R27's half). Wall contact after
the move: 5.4/min moved vs 37.9 unmoved — the time spent standing near the change is time not
spent riding walls; n = 6, read as a signal. The stand: 11 of 11 hand-backs held on every seed,
the joint brain standing 300–400 s a run.

**Verdict.** W3 `WORKING` at n = 6 on the map and the walk (a place is a stop; the cleanest walk
measured; the stand untouched) and, with the stop uncapped, **`WORKING` on contingency as a signal** — stops shorten as the map bakes and lengthen at a moved chair, which is the claim a Roomba cannot make; capped at 20 s it read `PARTIAL` (stops end on a quiet round, and
their length would fall with revisits the run does not contain). Not promoted; the operator's
eye is pending (preset R41, seed 6). The ball (d) is inconclusive (a stimulus below the sensor's resolution); the chair (d) is the
evidence. To a finding: n ≥ 20 varied rooms and the uncapped stop as the default form. Next: W4
(the command mux) and W5 (leaving a surface, §12.7b).

### 17.21 W3b — the gaze babbles: random steps, error-driven dwell, and the map's baking watched (R42, 2026-09-12)

**The operator's direction.** The three-bearing round of §17.20 is a schedule wearing a saccade's
clothes; let the head babble while standing so the error of the scene it takes in is what it
reduces — it will look more alive and the place gets more detail. And watch the baking: an EPM
that lets nodes decay suits locomotion and escape, not learning a place.

**Built.** `--stop-gaze YAW_SD PITCH_SD HOLD MAX QUIET`: at a stop the gaze takes random steps
(a normal step per move, yaw clamped to ±0.7 rad, pitch to [−2·sd, +0.7·sd]), each held HOLD s and,
while the view's winner is not a baked node, up to MAX s — long enough to bake it in one dwell
(50 processed ticks at `process_every_n_ticks 5` = 5 s) — and QUIET known gazes in a row end the
stop. The move is exploration, the dwell is the map's error. A pitch override joins the yaw
override in the head adapter. **The baking read-back**, from the token every tick: views inserted
and baked by phase, prunes, and whether a pruned id was ever baked. In the GNG every prune path —
isolation, stale prune, the health sweep with `health_death_spares_baked` (on in the map config) —
spares baked nodes. W1 path byte-identical.

**Measured.** R41's graph, the uncapped stops (every 80 s, up to 60 s), map only at stops, head
brain frozen through the stop, n = 6, 1500 s:

| n = 6 | stands held | handed off | rescued at stops | stop s | quiet-round ends | saccades | inserted at stops | baked at stops | pruned (baked) | nodes / baked at the end | walls/min | cells |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| the round (§17.20) | 66 / 66 | 0 | 0 | 36.5 | 71 % | 143 | 42 | 21 | 18 (0) | 31 / 28.5 | 23 ± 19 | 97 |
| gaze, yaw only (sd 0.35) | 66 / 66 | 0 | 0 | 29.5 | 91 % | 168 | 45 | 20 | 34 (0) | 30 / 26.5 | 47 ± 43 | 96 |
| gaze, yaw + pitch sd 0.20 | **24 / 66** | 42 | 34 | 26.6 | 92 % | 115 | 37 | 12 | 25 (0) | 26 / 21 | 16 ± 17 | 104 |
| **gaze, yaw + pitch sd 0.08 (R42)** | **65 / 66** | 1 | 0 | 31.3 | 92 % | 181 | 41 | 21 | 18 (0) | 33 / 29 | 14 ± 12 | 111 |

- **The babble stands, at a small pitch.** Yaw babble alone is as harmless as the round (66 of 66);
  pitch steps of sd 0.20 rad tip the stand at 42 of 66 hand-backs and fall 34 times — the smoke's
  two-of-three, at scale: a pitched head is a stance the R19 brain was never identified in, and
  with its head module frozen the neck cannot join the catch (register O36, the operator's to-do:
  work on the stand under pitch change; rate-limit the babble if pitch *speed* is the constraint).
  At sd 0.08 the stand holds 65 of 66 with one hand-off and no fall, and it is the best arm on
  cells (111) and wall contact (14 ± 12, 4 of 6 seeds under 17).
- **Wall contact is seed noise across the arms** (two wall-riding seeds in the yaw-only arm at 92
  and 113/min, different seeds elsewhere): the walk between stops is W5's problem, and the gaze
  moves it only by moving where the map's nodes, hence the play loop's targets, fall.
- **The baking, watched.** No baked view was ever pruned in any arm (`prunedBaked` 0 of 18–34
  prunes per run): a baked view is permanent here, as the map config's `health_death_spares_baked`
  intends. What decays is the *unbaked*: of 41–45 views inserted at stops per run, 20–21 bake
  and 18–34 die before a revisit. The dwell can bake a view in 6 s only if one node stays the
  winner through the hold; two views a gaze step apart share the hold and neither reaches 50
  visits. So the place is learned by half. The levers: the map's `baking_threshold` (hot-mutable,
  50 by default — 20 would bake in a 2 s dwell), or a dwell that counts the winner's visits
  rather than seconds. Recorded, not built.
- **The stop is the map's.** 92 % of stops end on a quiet round (71 % for the round), mean 31 s,
  181 gaze steps a run — the head is never still and never on a schedule, which is what was asked.

**Verdict.** W3b `WORKING` at n = 6 in the small-pitch form (R42): the babble replaces the round
with no cost to the stand and a fuller look, the stop ends on the map's own word. Full pitch is
`REGRESSION` on the stand (O36). The baking read-back is the instrument the operator asked for and
its first reading is the next lever: half the views a stop inserts die unbaked. Not promoted
(operator's eye pending, preset R42). Next: the baking threshold on the map, then W5.

### 17.22 O37 tried: the map's baking threshold (R43, 2026-09-12)

**The lever.** `map_epm.baking_threshold` 50 → 20 and → 10 (a config arm on R42's form; the gaze
babble's dwell bakes a view in 2 s / 1 s of one winner instead of 5 s). n = 6, paired:

| n = 6 | inserted at stops | baked at stops | pruned (baked) | nodes / baked at the end | stop s | quiet-round ends | saccades | stands held | cells | path m | mapTLE | walls/min |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R42 (threshold 50) | 41 | 21 | 17.5 (0) | 33 / 29 | 31.3 | 92 % | 181 | 65 / 66 | 111 | 94 | 0.22 | 14 ± 12 |
| threshold 20 | 33 | 24.5 | **5.8** (0) | 33.5 / **33** | **11.7** | 100 % | 150 | 66 / 66 | 131 | 127 | 0.31 | 23 ± 28 |
| threshold 10 | 30 | 24.5 | **3.3** (0) | 31 / 30 | **10.1** | 100 % | 160 | 64 / 66 | 138 | 133 | 0.32 | 19 ± 16 |

- **The decay is gone.** At 20 the map keeps what it sees: 5.8 prunes a run against 17.5, and
  every node standing at the end is baked (33 of 33.5). O37's aim, met by the constant.
- **And the dwell loses its signal.** A view now bakes inside one gaze, so "the winner is
  unbaked" is true for two seconds and then never; every stop ends on a quiet round at 10–12 s
  instead of 31, with fewer views inserted (33 vs 41) and a map that fits the walk worse
  (`mapTLE` 0.22 → 0.31, 6+/0−). The stop's contingency — long where the place is new — was
  carried by the bake flag, and the bake flag was carrying it only because 50 visits was slow.
  This is `CLAUDE.md` §5 rule 5 in the flesh: a constant tuned to a scale, and either setting
  of it is wrong for one of the two consumers (the map's memory wants fast baking; the dwell's
  novelty wants slow).
- The stand is indifferent (66 / 66, 64 / 66); cells and path rise because the stops are
  short; wall contact ties at this power.

**Verdict.** `PARTIAL`: threshold 20 fixes the map's forgetting and breaks the dwell's novelty.
Not promoted. The lever that separates the two: a dwell on the token's **residual against its
own expectation** (`quant_error` vs `expected_error`, the channel's running TLE — the
Kalman-faithful quantity the token already carries) instead of the bake flag; the view is
worth holding while it surprises the map more than the map expects to be surprised, and the
bake threshold can then be the map's own choice (20). Re-use context for bake 20: with that
dwell. Register O37 stays open with this as its next step.

### 17.23 The dwell's signal, three forms (R44–R45 sweeps; R43 config, 2026-09-12)

**The question left by §17.22:** the bake flag carried the dwell's novelty only because baking was
slow; what should the gaze hold on, so that the map can bake fast and keep its views (O37) while
the stop still ends when the place is known? Three forms, each a guarded flag on `--stop-gaze`,
all on R42's babble (yaw sd 0.35, pitch sd 0.08), uncapped stops, n = 6:

| n = 6 | dwell signal | bake | stop s | stand % | quiet-round ends | inserted / baked at stops | pruned | nodes / baked at the end | walls/min | cells | stands held |
|---|---|---|---|---|---|---|---|---|---|---|---|
| R42 | the winner is unbaked | 50 | 31 | 35 | 92 % | 41 / 21 | 17.5 | 33 / 29 | 14 ± 12 | 111 | 65 / 66 |
| R43 arm | the winner is unbaked | 20 | 12 | 12 | 100 % | 33 / 24.5 | 5.8 | 33.5 / 33 | 23 ± 28 | 131 | 66 / 66 |
| residual K 1.0 | `quant_error` > K × `expected_error` | 50 | 48 | — | 24 % | 54 / 16.5 | 39 | 32 / 27 | 8.5 ± 9 | 85 | 66 / 66 |
| residual K 1.0 | the same | 20 | 54 | 68 | 17 % | 55.5 / 32 | 21 | 42.7 / 40.5 | 3.1 ± 4 | 81 | 65 / 66 |
| residual K 1.5 | the same | 50 / 20 | 10 | — | 98 % | 29 / 1.3 ; 23.5 / 11 | 27 ; 4 | 20 / 18 ; 24 / 23 | 32 ; 22 | 92 ; 121 | 65 ; 66 / 66 |
| learning progress | surprised on arrival (K 1.0), held while `quant_error` > 0.5 × its arrival value | 50 | 50 | 58 | 26 % | 72 / 16 | 61 | 34 / 28 | 9.8 ± 16 | 71 | 66 / 66 |
| **learning progress (R43)** | the same | **20** | 46 | 53 | 36 % | 53 / 30 | 17.5 | **43 / 42** | **10.2 ± 5.5** | 89 | **66 / 66** |

- **The plain residual is a knife edge.** Against the channel's global expectation a view is
  "surprising" about half the time by construction, so K 1.0 holds two thirds of the gazes and the
  duck stands 68 % of the run with stops that never shorten (55 → 45 s; flat at 58 s with bake 20);
  K 1.5 is the token's own `is_novel` percentile and ends every stop at 10 s with almost nothing
  baked. No K serves both the map and the stop.
- **Learning progress does what the doctrine says the dwell is for.** A view that surprised the
  map on arrival is held while the prototype is still moving to it — its error above half its
  arrival value — and released when learned; a known view ends at the minimum hold. With bake 20
  the map is the fullest measured, **43 nodes with 42 baked at the end** and no baked view ever
  pruned, on the tightest wall contact of the line (10.2 ± 5.5), with the stand untouched (66/66).
  O37's aim is met in this form.
- **What it costs, and what that cost is.** The stops run 46 s and the duck stands 53 % of the
  run; the stop length does not fall over eleven stops (49 → 47 s). The reason is not the map: the
  random babble keeps finding views the map has not seen — 53 insertions a run — so "six known
  gazes in a row" rarely comes. Whether a stop should end when *this stop's* scene is learned or
  when *random gazes* stop finding novelty is a design choice, and it is the balance of the
  ten-minute story (walking against looking). Recorded as the open knob rather than tuned.

**Verdict.** The learning-progress dwell with bake 20 `WORKING` for the map (O37 `RESOLVED` in this
form: the place is learned whole and kept), `PARTIAL` for the stop's contingency (its ending is
the babble's, not the place's). Config R43 carries it (preset). The stop's ending rule — the
place's own learned-ness (e.g. the fraction of this stop's views baked) against the babble's
quiet count — is the next design decision, the operator's.

### 17.24 W3d — the orienting reflex: a change at a still gaze ends the stop and the duck goes to look (R44 config; R46 sweeps, 2026-09-12)

**Agreed with the operator.** Something changing during a stationary view — a ball rolling by, a
person walking past — should trigger interest in that direction: the stop ends and the duck walks
toward it. The signal is the map's own: while the head holds a bearing the view's winner should not
change and its error should not jump; if they do with the gaze still, the world moved.

**Built.** `--stop-orient K TURN_VX WALK_VX SECS`. While the gaze is still (arrived ~10 ticks, the
token caught up ~10 more, three clean samples at the map's rate): the view's quant error more than
K spreads above the hold's own running mean (the spread floored at 5 % of the mean), on two
consecutive samples, **touching at least two different ToF columns** — the change detector. A
first hit freezes the gaze on the surprise so the confirmation can come. Then: the stop ends
(`stop:orient`), the body **pivots** to the gaze's world bearing (odometry yaw + head yaw; the
walker does not turn on a yaw command alone, 0.03 rad/s, but at 0.2 m/s with full yaw it turns
0.5–0.8 rad/s nearly in place), walks with a P on the dead-reckoned yaw until it has covered ~1 m
(or reached a surface after 0.4 m), and stops there to look; SECS is the cap. The twist brain is
frozen through it. Two stimuli, harness actions like a shove: `--roll-past DELAY SPEED` places
`obj_ball0` inside the free space the gaze sees and rolls it across the view (stopped 1.5 s later);
`--walk-past DELAY SPEED` carries the chair across — a person-sized mover. Both start once the
hold's baseline is armed and just inside the view's edge, and retry at the next gaze when the gaze
faces a surface. Read-backs: rolls and skips, changes prompted (within 3 s of a stimulus) and
unprompted, orientations, arrivals, timeouts, reach to where the mover was.

**Nine catches on the way, each measured before the rule changed** (the ledger's, kept here
because each is a property of the sensor or the map a later consumer will meet again):
a winner switch to an existing node is the map's own flicker, not a change (dropped);
a per-view expectation kept across stops over-fires (the same node reached from another pose has
another error level; dropped for the hold's own); the hold's expectation needs the token to have
caught up (armed from tick 21, not 11); the pure-yaw pivot does not turn the walker; the stimulus
landed exactly as a 6 s learning hold ended, and later exactly five ticks before a minimum hold
ended (alignment artifacts of a tick-scheduled stimulus — it now starts when the detector is armed);
a stimulus placed 1.2 m along a gaze that faces a wall lands inside the wall (placed at 0.7 of the
range the ToF reports, skipped under 0.6 m); the head-pitch joint is positive DOWN (measured by the
ToF's floor fraction; the babble's clamp had looked up); the ball keeps rolling and re-enters later
(stopped after 1.5 s); and **the unprompted triggers are the sensor's edge flicker** — at every one
a few ToF zones flipped between empty and a hit as the body swayed a thousandth of a radian, and
the nearest-hit-per-column reduction carries one grazing ray into a whole column. A mover crosses
columns; flicker stays in one — hence the two-column rule.

**Measured.** R43's form (gaze babble, learning-progress dwell, bake 20, map at stops), the
uncapped stops, n = 6, 1500 s:

| n = 6 | stimulus per stop | stimuli / run | detected | unprompted / run | orientations | arrived | reach | stands held | rescues at stops | stop s | cells |
|---|---|---|---|---|---|---|---|---|---|---|---|
| R43 (no reflex) | — | — | — | — | — | — | — | 65 / 65 | 1 | 54.5 | 70 |
| reflex, nothing moved | — | — | — | 1.7 ± 1.6 | 1.7 | 1.5 | — | 66 / 66 | 1 | 50.5 | 66 |
| reflex, a ball rolls past | the ball | 12.3 | **42 ± 14 %** | 1.0 ± 0.9 | 6.3 | 6.0 | **0.36 m** | 87 / 87 | 0 | 39.2 | 97 |
| reflex, a chair carried past | the chair | 15.8 | **57 ± 12 %** | 1.8 ± 0.8 | 10.8 | 10.3 | — | 106 / 106 | 0 | 28.7 | 87 |

- **It orients and it arrives.** 95 % of orientations reach the place (the rest time out); with the
  ball, 0.36 m from where it was rolled. The stand is untouched: 259 of 259 hand-backs held across
  the three reflex arms, no rescue at a stop. A change ends the stop early (54 → 29–39 s), which is
  the walking/looking balance moving for a reason.
- **Detection is the sensor's.** A person-sized mover is caught 57 % of the time and a 6 cm ball
  42 %; the misses are the ToF's resolution (one column for the ball) and the stimulus crossing a
  view the map was still learning. The false rate is about one orientation per fifteen minutes in
  every arm, the edge-flicker residue the two-column rule leaves.
- **The head's pitch now looks down** (the sign catch): the floor's objects are in the frame.

**Verdict.** W3d `WORKING` at n = 6, loud on the behaviour (the duck sees a thing move, turns, walks
to it, and looks), signal-strength on the detection rate. Not promoted; the operator's eye pending
(preset R44: a chair carried past at every stop). The change detector is a host-side scaffold over
the map's error and the sensor's columns; its substrate form is the approach loop of the plan's §3
(E2) with the view-level transition surprise as its trigger, and the place cloud (§12, the operator's
second point) is the level that should own "the scene changed". Next: the place cloud.

### 17.25 The speed question: the map's error is a signal, the yaw rail is why it cannot be spent yet (R46 sweeps re-read, 2026-09-12)

**The operator's question.** The duck walks at one slow pace; a speed that varied would be more
engaging. What does the walking surface actually offer, and what should a speed be a function of?

**The surface, as built.** Pollen's walker takes a 13-slot command (`Observation.hpp`): the twist
(vx, vy, vyaw in trained ranges 0.4 / 0.3 / 1.0), the four head joints, and a **body pose block —
`body_z`, `body_roll`, `body_pitch` — wired into the observation and set by nothing, on any run this
project has made.** The forward speed itself is not scripted anywhere: it is a state prior,
`state_prior_indices [0, …]` → `state_prior_targets [0.75, …]`, "I predict I am moving at 0.75 of
the walker's vx range", descended through the identified A (§16.5). Two constants sit beside it: the
orienting reflex's `TURN_VX` / `WALK_VX` (§17.24), a named scaffold, which is a literal fixed 0.25
m/s for **40 % of the walking ticks** of an R44 run. The rest is the prior's own command. Below
about 0.25 m/s the walker stands still (§16.2), so the usable forward band is ~0.25–0.5 — about 2×.

**What was measured.** No new run: the R46 sweeps re-read (`reflexWalk`, n = 6, control phase
700–1500 s, 78 526 brain-driven walking ticks), asking whether the map's error on the walk could be
the thing a speed is a function of. With `--map-on-stop` the map does not learn while walking, so
its TLE there is a clean read — *do I recognise where I am* — with nothing written back.

| mapTLE, by phase | mean ± sd | p5 | p95 |
|---|---|---|---|
| at stops | 0.139 ± 0.093 | 0.022 | 0.324 |
| the orient approach | 0.215 ± 0.101 | 0.088 | 0.404 |
| **the brain-driven walk** | **0.299 ± 0.148** | 0.107 | 0.596 |

Ordered the way the mechanism predicts — the map knows the places it learned at best — with ~5× of
range and no saturation at either end. It is **not a wall proxy**: corr(ToF proximity ahead, mapTLE)
= +0.17. And it carries **place that replicates**: a 0.25 m cell's value agrees across *separate*
visits at split-half r = **+0.44** over 192 cells, between-cell sd 0.11.

**The EMA, and the artifact it nearly produced.** The signal dithers — the map's winner switches
143/min on the walk against 23/min at a stop, and TLE on a switch tick is 0.422 against 0.293 on a
held one, so the transition term is most of the chatter. Smoothing looked like it bought a great
deal, and did not:

| | variance share between cells (ICC) | split-half r **across separate visits** |
|---|---|---|
| raw | 0.386 | **+0.438** |
| EMA τ 0.5 s | 0.479 | +0.446 |
| EMA τ 2 s | 0.588 | +0.405 |
| EMA τ 5 s | 0.606 | +0.431 |
| EMA τ 10 s | 0.609 | — |

**The ICC column is an artifact and the right-hand column is the measurement.** A cell is visited in
contiguous runs of ticks, so an EMA shrinks within-cell variance mechanically, whatever it does to
the content; tested across separate visits the gain is flat. An EMA adds **no place information** to
this signal. Its real job is narrower and still worth doing: at τ ≈ 0.5–1 s it removes the
transition spikes (raw autocorrelation 0.64 at 0.1 s, 0.22 at 1 s, 0.04 at 2 s) without touching
what replicates, and past ~1 s it buys only lag. A second term to subtract: mapTLE drifts
**0.383 → 0.273** across the control phase as the map bakes — the same size as the between-cell sd —
so the signal a drive should read is TLE against its own slow running average (`map_tle_long_`,
τ 3000 ticks, already in `IntentAdapter` for the wander rule), not TLE.

**Why the lever cannot be built on it yet.** The forward command does not reach the body:

| on the walk, n = 6 | |
|---|---|
| \|vyaw\| > 0.9 | **94.3 % of walking ticks** (mean 0.968; sign flips 43/min) |
| \|vyaw\| < 0.4 | 1.2 % — **no 2 s window in six seeds** |
| corr(commanded vx, achieved speed) | +0.18 (1–5 s windows) |
| corr(duty above the 0.25 dead zone, speed) | +0.19 |
| corr(wall contact, speed) | **−0.81** |
| speed across the full range of commanded duty, 0.24 → 1.00 | 0.167 → 0.201 m/s |

The duck commands maximum yaw rate essentially all the time and flips its sign every 1.4 s; its
speed is then set by whether it is jammed against a surface, not by what it asks for. This is
§17.16's dither read at the command level, and it means a speed drive built now would fail §3.2
rule 5 before it started — the consumer cannot fire. One confound to carry forward when it can:
corr(mapTLE, instantaneous speed) is already −0.17 without anything asking for it, because wall
contact produces both a low speed and an unusual view, so a "slow where surprised" arm must be
judged off-wall or it will score on a mechanism it did not add.

**Verdict.** The signal is `WORKING` **as a signal** — measured, place-bearing, non-degenerate, and
its conditioning known (slow-average-relative, EMA τ ≈ 0.5–1 s, judged off-wall). The speed lever is
`DEFERRED` behind **W5, the twist brain's yaw channel** (O31, §17.17): until \|vyaw\| comes off the
rail there is no forward-speed channel to modulate, and the 2× band the walker offers cannot be seen
through a body that is always turning. The body pose block is recorded as a second, untouched
control surface (O39). Next: W5.

### 17.26 W5, fork item (a) — the lesion refutes the swamping, and the reference is what will not stand still (R47, 2026-09-12)

**The hypothesis under test**, §17.17's own words: the prior's Gauss-Newton step writes the full
outer product `C(j,:) += g·prev_xᵀ`, so an error on one index deposits content in every column of
every motor's row; hold the yaw row to the heading column and, *if straightness jumps, the swamping
is proven and the mechanism is the prior's weight rather than a new law.*

**Built.** `state_prior_isolate` (MotorEPMv2, HotMutable, default 0): after every update, C(:, i) —
and Cp's, in split mode — is zeroed for every state column that is not a resolved
`state_prior_indices` entry; h is spared (h reaches, C balances). Applied broader than §17.17's
literal form, on purpose: confining the yaw row to the heading column alone would also delete the
ToF columns, and avoidance through them is `WORKING` (§17.2) — the lesion should remove what has no
objective behind it, not a measured mechanism. Read back as `spIso` in `diag_lite`.

**Guard.** With the param absent the host reproduces the pre-change R46 run byte-for-byte —
75 000 ticks, md5 `e4b5c3fa…` on both — which checks the gain-0 guard and the harness's
reproducibility in one go. Arm: `--arm iso:motor_epm_intent.state_prior_isolate=1` on the R43 W3c
stack (**not** the R44 one: the orienting scaffold owns 40 % of walking ticks there and would mask
the lever). n = 6, 1500 s, control phase 700–1500 s. `spIso 5` on the twist brain, −1 on the head
and stander brains — the lever landed, on the module it was aimed at and no other.

| n = 6, paired | straight | hdgErr (rad) | \|vyaw\| | yawFlip/min | walls/min | path m |
|---|---|---|---|---|---|---|
| base (R43 W3c) | 0.09 ± 0.02 | 1.66 ± 0.09 | 0.95 ± 0.01 | 69 ± 17 | 14.8 ± 11.8 | 49.6 ± 9.0 |
| **iso** (the lesion) | **0.08 ± 0.02** (Δ −0.01, t −3.5, **1+/5−**) | 1.63 ± 0.13 (t −0.8) | 0.89 ± 0.05 (t −3.7) | 57 ± 23 (t −1.4) | 22.0 ± 16.9 (5+/1−) | 40.2 ± 5.7 (0+/6−) |

Straightness did not jump; it fell, on five of six seeds, and the body walked less and hit more.
**`REGRESSION`, and the hypothesis is refuted** — with a mechanism, which is the useful part.

**Why, in three measurements.**

1. **The command was already aimed, and the lesion is what broke the aim.** Counting ticks where
   the yaw command opposes the heading error: base **68.2 %** (62–79 % on every one of six seeds,
   n = 84 k), lesion **43.9 %** — below a coin flip, and wild across seeds (6, 22, 37, 60, 67,
   75 %). The reason is in the step's own algebra: `C(j,:) += g·prev_xᵀ` spreads the aim across the
   *whole* state vector, so the command contribution is ≈ g·(prev_x·x) rather than one column's
   product. **The cross-talk columns are where the heading feedback lives; the "swamping" is the
   mechanism, not the defect.** This also re-motivates fork item (b) — the model-implied step
   computes the aim explicitly instead of accumulating it — and it is untouched by this result.
2. **The body does turn.** Achieved yaw rate at full command, from the unwrapped own-heading over
   1 s windows: **0.36 rad/s at vx < 0.05** and **0.47 rad/s at vx > 0.20**, monotone and correctly
   signed across the whole vx × vyaw grid, in both arms. (§17.24's "the walker does not turn on a
   yaw command alone, 0.03 rad/s" is a *standing* hand-off number and does not carry to a walking
   body — an inference worth correcting here, since it nearly became the diagnosis.) Also, for
   O39: the duck turns half again as fast while moving, so **yaw authority and forward speed are
   one channel, not two.**
3. **The reference will not stand still, and it moves the way the body moves.** It jumps > 0.5 rad
   **90 times a minute**, and — the number this section adds — it **follows the body at +0.50 rad
   per radian turned** (0.5 s steps, jumps excluded, n = 48 k). Half of every correction the duck
   makes is absorbed by its target turning with it. Inside quiet windows (≥ 2 s, no step above
   0.1 rad) the error is 1.65 rad at the start and 1.68 rad two seconds later, closing on **44 %**
   of windows, with \|vyaw\| at 0.95. A saturated, correctly-aimed command against a target that
   retreats at half the rate it is chased.

A fourth, smaller correction: the efference term is real but weaker than §17.17 read. Comparing the
command with the *simultaneous* yaw rate scores 72 %, but the command causes that rate — the
comparison is partly a tautology. Against the rate of 0.4 s **earlier** it is 65–68 %.

**The instrument.** `refFollow` is now a sweep column (`l2_sweep.py`): radians of reference motion
per radian of body turn, over 0.5 s steps with jumps excluded. ~0 is a target in the world, which
turning closes; ~1 is a target that turns with the body, which turning cannot. **Any fix to this
line is judged on moving it toward 0**, and neither §17.16's fork items 1 and 2 (`commit_hold`,
`lookahead` — both `REGRESSION`) nor R38's quiet reference were ever measured against it.

**Verdict.** Fork item (a) `REGRESSION`; re-use context: a config whose prior has *one* index, where
the distributed aim has nowhere else to go. The yaw channel's controller is exonerated — it aims,
and the body answers it. **The defect is upstream, in what sets the reference**, which is the play
loop's target on a map whose winner switches 60 times a minute. Next is the operator's call between
fork item (b) (the model-implied step, now the better-motivated of the two) and a reference-side
lever judged on `refFollow`.

### 17.27 W5, fork item (b) — the model-implied step: avoidance becomes real, the heading does not (R48, 2026-09-12)

**The lever.** §17.17's other form: *the yaw command that closes the prior's error in one
identified step, `u = −e / A(idx, vyaw)`, clamped — what the prior's half 2 is meant to converge
to and here does not.* Built as `state_prior_step_gain` (MotorEPMv2, HotMutable, default 0), in the
general form rather than the yaw-only one: the ridge least-squares command over **all** the prior's
rows, `y* = argmin ||A_p·y − e||² + reg_eps·||y||²`, clamped to ±1 per motor and added to the
pre-tanh operating point. The ridge is the module's own `reg_eps` — no new constant, and the gain
is still the model's own authority, which is the same justification part 2's descent has. The step
the command carried is stored per leg and added back when the update reconstructs its operating
point, or G would report the slope at a different point on the tanh than the body actually ran.

**What it turns out to be, numerically.** With `reg_eps` 0.01 against `A_pᵀA_p` ~ 10⁻³, the ridge
dominates and the solve reduces to `≈ A_pᵀe / reg_eps` — the same *direction* part 2 descends, applied
straight to the command instead of accumulated into C. The read-back says it then rails:
`spStep` 1.00. So what ships is a **model-signed saturating command**, not a deadbeat solve. That is
the honest description and it is what the clamp in §17.17's own formulation implies for a channel
whose authority is ~0.01 per tick.

**Guard.** Param absent → the pre-change R46 run reproduced byte-for-byte, md5 `e4b5c3fa…`.
`spStep` reads −1 off and its own size on. Unit test covers gain-0, that it acts, the read-back, and
the **sign control with part 2 switched off** (`state_prior_lr` 0, `ctrl_lr` 0), so the pull toward
the target is demonstrably the step's own doing. Same base and host args as R47; n = 6, 1500 s.

| n = 6, paired | walls/min | contact% | eps / metre | cells | span m² | straight | hdgErr | refFollow | mapTLE | switch/min |
|---|---|---|---|---|---|---|---|---|---|---|
| base (R43 W3c) | 14.76 ± 11.82 | 3.31 | 4.58 | 69.7 ± 11.7 | 10.17 | 0.09 | 1.66 | 0.48 | 0.19 | 59.9 |
| **step, gain 1.0** | **1.82 ± 1.64** (0+/6−, t −2.5) | **0.14** | **0.65** (1+/5−) | 73.8 ± 11.8 (ties) | 10.23 (ties) | 0.07 (2+/4−) | 1.57 (1+/5−, t −1.6) | 0.58 (worse) | 0.16 (0+/6−) | 42.1 (0+/6−) |
| step, gain 0.3 | 3.25 ± 2.97 (0+/6−) | 0.48 | — | 69.8 (ties) | 7.48 | **0.07** (0+/6−, t −2.8) | 1.60 | 0.45 | 0.16 | 45.1 |

- **Avoidance becomes real, and it is not the degenerate orbit.** Wall contact falls eight-fold on
  every seed, and the blind metric's complement holds: cells and span **tie**. Per seed the loudest
  case is seed 3 — the base's worst wall-rider, 34.1/min on 56 cells, becomes **0.0/min on 89
  cells** over the same path length. Seed 5 likewise goes to zero. Normalised for walking time
  (below), episodes per metre go 4.58 → 0.65, better on five of six seeds.
- **The heading does not move.** `hdgErr` 1.66 → 1.57 is inside the noise (1+/5−, t −1.6),
  `refFollow` gets *worse* (0.48 → 0.58), `straight` does not improve at gain 1.0 and is worse on
  every seed at gain 0.3, and `yawFlip/min` rises 69 → 106 (6+/0−). **The lever was proposed as the
  yaw channel's fix and it is not one.** §17.26's diagnosis survives item (b) as it survived item
  (a): what holds the heading error open is the reference, not the law that chases it.
- **Why avoidance and not heading**, by inference rather than measurement: the proximity rows of A
  are weak (§17.2 measured 0.0009 against the command — "the babble rarely reached a wall"), so the
  descent through them built almost nothing, while the computed step divides by `reg_eps` instead
  and turns a weak-but-correctly-signed authority into a real command. The heading row is not
  authority-starved in the same way; its problem is the target. The arm that would settle this —
  the step restricted to the proximity indices — is one lever away and has not been run.
- **The side effect must be named.** Stops now run to the 60 s cap on every seed (`bored%` 12.1 →
  0.0, `stop s` 54.5 → 60.0, stand% 62.6 → 70.0), so walking time falls 285 → 217 s. That is O37's
  open ending-rule knob moving, not a new pathology — a run of six consecutive known gazes is a
  rare event either way (≈ 8 of 66 stops in the base) — but it is why the wall result is reported
  per metre as well as per minute.

**Verdict. `PARTIAL`** — `WORKING` and loud on avoidance, `NULL` on the heading channel it was
aimed at. Not promoted: preset R48 for the operator's eye, and the stop-length interaction is the
thing to watch while watching it. Re-use context for the null half: a reference that does not follow
the body (`refFollow` → 0), after which the same step would be worth re-reading on the heading row.
Follow-ups: the proximity-only arm above; and for O39, wall contact was the term that dominated
achieved speed (corr −0.81), so a walk that stops hitting things is the first thing the speed
question needed.

### 17.28 The ToF studies — the sensor can carry objects, the frame cannot, and the walk is the ceiling (2026-09-12)

**Asked for** (the operator, after the W5 verdicts): an EPM on the sensor's full output with its PCA
visible; several EPMs in different roles off the same sensor; and the point cloud a head babble
builds, watched for change. Method, tooling and re-run commands are in
[`microduck_tof_studies.md`](microduck_tof_studies.md); figures and the written report are the
[study page](https://claude.ai/code/artifact/468b1fff-ed27-483b-86a6-ca87d7c5459a). Everything runs
the **shipped EPM** over recorded frames (`cpp_core/bench/epm_tof_study`), 4 runs × 1500 s, seeds 6
and 3, 16 790 distinct casts. World-derived labels judge the vocabularies and reach no brain.

**M1 — acuity. A block is one pixel.** Class mix 9 % Empty / 2 % TooClose / 27 % Floor / 62 % Hit;
8.7 % of returns sit in the 2–10 cm height band. Per cast, zones landing in the object's own height
band: **block 1.2, ball 2.0, chair 6.3, shelf 6.7** (seed 3 agrees: 1.8 / 1.9 / 3.8). A 4 cm block at
1 m subtends 2.3° against 5.625° zone spacing — it falls between beams more often than on one. Found
on the way and fixed: `TofZone::point` was in the *raw* trunk frame; `point_level` is the
gravity-levelled one, and the correction moved ~5 % of returns out of the low-object band.

**M2 — the stumble channel is blind, and the event barely happens.** `stander->act` is called only
inside a stop, so **the joint brain is asleep for the whole walk** and nothing predicts the body
then. The one live channel, the twist brain's `motor_tle`, reads 0.360 at object contact against
0.300 before (**−0.11 sd**, nothing above its own p99); wall contact manages +0.28 sd over 102
onsets. The body does register it physically — tilt 3.5° → 5.6° — so the signature exists and no
predictor watches it. And there were **9 object contacts on seed 6, 0 on seed 3, and no falls**:
trip-and-investigate needs the trip arranged, like a shove. Caveat that bounds this: `motor_tle` is
an EMA (τ ≈ 20 ticks) and cannot show a 100 ms event even in principle, so this measures the
*available* channel as blind, not the body error as absent.

**S1 — the full output is two-dimensional.** PCA of the raw 64: **PC1 holds 85 %** and two components
hold 90 % (0.89 on seed 3). A nearest-centroid readout of object class from four raw PCs scores
**0.450 against a 0.361 majority** on seed 6, and **0.284 against 0.646** on seed 3 — worse than
naming the commonest class. The one common mode is *how far away whatever is ahead happens to be*;
object identity is in the residual, exactly §0 rule 2's failure. On encoders, in this new context:
`rbf` collapses the 64 to a latent needing **2** dims for 90 % of its variance where `jl_state` keeps
**7** — R35's 2026-09-11 finding reproduced, and the encoder question settled for anything this wide.

**S2 — every view's vocabulary is a pose code.** Six EPMs on the same stream (cols8, full64,
full64_dm, full64_norm, heights8, geom_shape). Raw mutual information with object class reaches
0.770 and a best-node "block" detector reaches F1 0.90 against a 0.36 base rate. Condition on the
duck's pose — 1 m cell × heading octant — and **every arm loses 85–90 %** of it:

| arm | I(W;O) | given place | given pose |
|---|---|---|---|
| cols8 | 0.770 | 0.648 | **0.113** |
| full64_jl | 0.705 | 0.639 | **0.099** |
| full64_dm | 0.614 | 0.564 | 0.078 |
| geom_shape | 0.473 | 0.423 | 0.068 |
| heights8 | 0.469 | 0.436 | 0.107 |

Seed 3 agrees (0.050–0.096 residual). Removing the common mode does bend the ratio the right way —
with the frame mean out, location information falls further than object information (0.396 → 0.299
against 0.394 → 0.357) — and by far too little to carry a behaviour. **And there is a ceiling no
encoder can lift: each object is viewed from 4–13 distinct poses in thirteen minutes** (a ball from
*one* location cell and two poses on seed 3). The duck stands 63–70 % of the run and nets ~30 cm per
walking bout, so it never sees the same thing from two places. **The sensor study is capped by the
behaviour problem** — the standing-still the operator saw is what starves the object vocabulary.

**S3 — the cloud is the level at which a small object exists.** At a stop the gaze already babbles
and the levelled returns compose with no extra geometry. One cast returns 52 points; the sweep
returns **37 635**, filling 2.7× the solid angle and **36× the distinct 4 cm voxels**. Points landing
on an object, per cast → per sweep: shelf 25.95 → 19 465; chair 2.35 → 1 765; **block 0.058 → 43.2**;
ball 0.002 → 1.2. A block goes from one point every seventeen casts to forty-three per sweep — the
difference between absent from the representation and present in it — but only in **3 sweeps of 11**,
because the babble (yaw sd 0.35 rad, pitch sd 0.08) rarely dwells on the floor.

Two body properties decide it: over a 56 s stop the trunk holds *position* to **0.9 cm** (the R19
stander is that still) but its *heading* drifts **9.7°**, which smears the cloud 17 cm at a metre.
De-rotating each cast by the duck's own odometry yaw — accurate to 0.1°, so its own to make —
was reported here to recover 7 % more distinct voxels. **Corrected 2026-09-13 (§17.30):** the rotation
was applied with the wrong sign and the 7 % was the doubled smear; with the correct sign the sweep
changes by −1 % and +4 % on two runs. Change detection on a rolling ball, 605 windows of which 22
carried motion, at a matched 5 % false-positive rate: **single frame 18 % (AUC 0.852) → cloud 41 %
(0.866)**, a 2.3× gain on the best single-frame statistic; the de-rotated cloud also reads 41 % (0.865)
with the correct sign — the 45 % first reported here came from the wrong one. For scale, R44's live
detector catches a ball 42 %.

**Verdicts.** M1 `WORKING` as a characterisation — the sensor resolves furniture and is at its limit
on floor objects. M2 the stumble channel `DEAD_CODE` in the measurement sense (the predictor is not
ticked), the event `DEFERRED` until arranged. S1 the full frame `NULL` as an object input and the
encoder question `RESOLVED` (`jl_state`). S2 **`NULL` for every view tried, with the cause located in
the data rather than the encoder** — re-use context: a run in which the duck travels. S3 `WORKING`
and the constructive result of the day: the cloud, de-rotated, with the gaze aimed at the floor.
Scale: two seeds for M1/M2/S1/S2, which replicate; one seed and 22 positive windows for the
detection number, which is a signal.

**What this changes.** An EPM on single frames cannot hold a node meaning "block", because a block is
not in its input — object work belongs downstream of the sweep. De-rotation belongs in the host (it is there now, and matters little — §17.30). The
gaze babble's pitch decides whether floor objects are found at all, which puts **O36 on the critical
path** rather than beside it. And the viewpoint ceiling says the sensor line and the behaviour line
are one line: novelty-toward-things needs a duck that travels.

### 17.29 The four steps out of §17.28 — the cloud in the host, the gaze null, the cloud vocabulary, and a body that notices (R49–R51, 2026-09-12)

The operator's go on the order §17.28 proposed. Each is guarded; each landed; two worked, one is a
null with a bonus finding, and one cannot be settled on data this duck can produce.

**Step 1a — the cloud moves into the host. `WORKING` as substrate.** `mj_host/src/CloudMap.{hpp,cpp}`:
a voxel-hashed occupancy cloud at 4 cm, opened when the body comes to rest, closed when the stop
ends, accumulating `TofZone::point_level` turned back by (yaw − anchor_yaw) from the contact
odometry. Position is deliberately not corrected (0.9 cm of drift is below a voxel). It exposes its
size, its floor-break mass, the `new_fraction` change signal, and a 36-dim **break profile** — 8
azimuth sectors × (nearest break range, its height, its vertical extent, its mass) + 4 globals —
published on `reality.proprio.cloud_in` for any graph EPM that declares it. `--cloud` absent
reproduces the pre-change R46 run byte-for-byte (md5 `e4b5c3fa…`). Read-back: **11 stops per run,
mean 883 voxels of which 124 break the floor.** The profile is per-sector arithmetic, not a
clusterer — the vocabulary over it stays the EPM's (§0 rule 1).

**Step 1b — aiming the gaze at the floor. `NULL`, and O36 does not reproduce.** Two levers, both
built (`--stop-gaze-slew` on the head override, `--stop-gaze-down` as the pitch babble's centre),
both landed (head-pitch command reaching +0.400 rad against the base's +0.229), neither moves the
floor-object signal:

| n = 6 | stands held | cloud voxels | break voxels / stop |
|---|---|---|---|
| pitch sd 0.08 (base) | **65 / 65** | 890 | 127.0 ± 27.1 |
| pitch sd 0.20 | **65 / 65** | 1057 | 137.5 ± 10.8 |
| pitch sd 0.20 + slew 0.6 rad/s | **65 / 65** | 1085 | 132.8 ± 39.8 |
| pitch sd 0.05, centred 0.13 rad down | **65 / 65** | 913 | 113.3 ± 17.5 |
| pitch sd 0.05, centred 0.22 rad down | **65 / 65** | 822 | 144.0 ± 23.6 |

Break voxels move less than their own spread in every arm. Measured properly — points per sweep on
a labelled block — the downward bias gives 43.2 → 58.7 (+36 %) on three stops, which is a direction
and not a result. The geometry says why: at pitch sd 0.2 the gaze already reaches 23° down, and
*that looks at the floor 0.2 m from the duck's feet*, where nothing is; a 4 cm block at a metre sits
7° below level, and the band that finds it is narrow and already inside the sensor's cone. **The
gaze was never the binding constraint.** Balls stay invisible at every gaze tried (0.5–1.2 points
per sweep, found in 1 sweep of 16).

The bonus finding is worth more than the lever: **O36's regression does not reproduce.** R42 measured
24 of 66 stands held at pitch sd 0.2; on the R43 stack the same amplitude holds **65 of 65**, with
and without a slew limit. The fix was not a slew — it was R43's dwell rule. O36 is closed by
measurement, not by a lever, and the slew's own contribution is unproven.

**Step 2 — an EPM over the cloud. The best vocabulary yet; the pose question still unanswerable.**
The `cloudp` view (the host's own 36-dim profile) against the two frame views, one EPM each over the
same stop frames. On a single seed it looked like mastery — I(W;O) 0.945, block F1 0.97, ball F1 1.00
— and that reading is **wrong**: the cloud exists only while the duck stands, so the frames carry
exactly *one pose per object*, `I(W;O | pose)` is 0.000, and the vocabulary is a stop-identity code.
Pooling six runs into one EPM raises the ceiling to 3–8 poses per object and gives the honest table:

| pooled, 6 seeds | nodes | baked | switch/min | I(W;O) | I(W;O \| pose) | F1 block / ball / chair / shelf |
|---|---|---|---|---|---|---|
| **cloudp** (36) | 16 | **16** | **7** | **1.546** | **0.128** | **0.91 / 0.90 / 0.94 / 0.96** |
| cols8 (8) | 21 | 18 | 20 | 1.216 | 0.093 | 0.81 / 0.70 / 0.78 / 0.76 |
| full64 (64) | 16 | 15 | 15 | 0.768 | 0.071 | 0.58 / 0.46 / 0.62 / 0.40 |

The ordering is consistent and the cloud wins every column.

**Correction to this section's first reading (computed 2026-09-12, after the operator asked what the
conditional meant).** It was reported here and in §17.28 that the conditional "collapses 85–92 %" as
though that were a deficiency of the vocabulary. It is not: it is a property of the room and the
itinerary. The quantity the conditional can reach is **H(object | pose) = 0.130 nats** against
H(object) = 1.731 — *pose alone fixes 93 % of what is in view*, and no vocabulary can explain
information that is not there. Against the ceiling that actually remains, over 45 pose cells at ~634
frames each:

| view | I(W;O \| pose) | of H(O \| pose) | shuffle control |
|---|---|---|---|
| **cloudp** | **0.128** | **98 %** | 0.001 |
| cols8 | 0.093 | 72 % | 0.001 |
| full64 | 0.071 | 55 % | 0.000 |

Shuffling the winner within each pose cell gives 0.001, so the estimator bias is negligible and the
numbers are real. **The cloud vocabulary extracts essentially all of the object information that
survives knowing the duck's pose**; the frame views get 72 % and 55 % of it. The earlier "the test is
not yet runnable" reading was wrong — the test ran, it simply had no denominator attached.

What stays true, and is a different claim: this is a *within-distribution* result. It shows the
vocabulary uses the object information present in the poses the duck actually visited. It does **not**
show pose-*invariance* — that a node would fire for a block from a viewpoint the duck has never
occupied — and only travel can test that. The absolute amount at stake is also small (0.13 nats of
1.73), for the same reason. What *is* clean is the vocabulary's shape: **16 symbols, every one baked, changing
seven times a minute** against the place map's 60–130 on the walk. That is the first thing in this
duck stable enough to be a symbol, whatever it turns out to denote. (Pooling is a probe, not a
trajectory: the duck teleports between runs, which is why 125 winner ids appear across a run that
ends with 16 nodes.)

**Step 3 — a body that notices. `WORKING` as a channel, `PARTIAL` as a detector.** `--body-predicts`
ticks the joint brain on every tick with its learning off, so it has an honest forward-model residual
while the walker drives. Predicting is not identifying: §17.10's drifting model was a model
*identified* under another driver's closed loop, which this is not, and the command is never applied
outside the stop it already owns. Across 6 seeds and **600 contact onsets** while the walker drives:

| channel | peak at contact | before contact | above its own p99 |
|---|---|---|---|
| joint brain `btle` (new) | **+1.05 sd** | +0.10 sd | **15 %** |
| twist brain `mtle` (all §17.28 had) | +0.39 sd | — | 3 % |

A channel 2.7× stronger where there was effectively none, and behaviourally free: at n = 6 nothing
moves (walls 3+/3−, rescues tie, stands 11/11 on every seed, `straight` +0.03 incidentally). It is
not yet a clean detector — 15 % of contacts clear p99 — and the reason is visible in the numbers:
the frozen model sits at a baseline of 1.55 with a spread of 0.13, so its dynamic range is
compressed. The instantaneous residual rather than the EMA remains the open one-line fix.

**Step 4 — travel. Not started, and now indicted three times over.** §17.28's viewpoint ceiling, step
1b's finding that the gaze is not the constraint, and step 2's unanswerable conditional all reduce to
the same sentence: *the duck does not go anywhere, so it never sees the same thing twice from a
different place.* Every remaining question about an object vocabulary is downstream of that.

**Verdicts.** 1a `WORKING` (substrate, guarded, read-back live). 1b `NULL` on both levers, with
**O36 `RESOLVED` by measurement**. 2 `WORKING` on vocabulary quality *and* on the within-pose object
content (98 % of the available 0.130 nats, against 72 % and 55 % for the frame views — measured on a break profile later found to be 82 % bare floor; on the fixed profile it is 95 % against 75 % and 64 %, §17.30); `DEFERRED` on
pose-INVARIANCE, which is a generalisation claim this data cannot test; re-use context: any run with
tens of poses per object. 3 `WORKING`
as a channel, `PARTIAL` as a detector; follow-up is the instantaneous residual. Nothing promoted;
scale is n = 6 for every A/B and 600 onsets for step 3.

### 17.30 The cloud becomes a module — and three bugs its first replay exposed (R46, 2026-09-13)

**The operator's decisions.** `CloudMap` becomes an ogma module; its cache is keyed by the map's own
place (the map EPM's winner); the duck viewer gets replay first; the inspector follows later as a
standalone static voxel viewer.

**Built.** `ogma::CloudMap` (`cpp_core/src/ogma/modules/CloudMap.{hpp,cpp}`, registered). The host no
longer accumulates anything — `mj_host/src/CloudMap.*` is deleted — and instead publishes one cast per
sense tick on `reality.proprio.tof_points`: `[still, yaw, trunk_z, odom_x, odom_y, 64 × (x, y, z)]`,
gravity-levelled, z above the floor, NaN for a zone with no return. All of it the body's own. The
module opens a cloud after `still_ticks` (25) of stillness and files it after `move_ticks` (25) of
motion — hysteresis that is load-bearing: closing on the first non-still tick chopped one stop's sweep
into fragments of 50, 70 and 943 voxels, because the gaze babble jogs the trunk's gyro past any
instantaneous stillness test. A filed cloud is cached under the MODAL map winner while it was open
(LRU, `cache_size` 8), and a revisit is judged once, at file time, against the cloud filed under the
same key, aligned through the two odometry anchors. It publishes the 36-dim break profile on
`reality.proprio.cloud` and `[new_fraction, revisit_change]` on `percept.cloud_change`. Config
`a1v2_r46_cloud.json` = R43 + `CloudMap` + `object_epm` (jl_state over the profile). The JSONL gains
`cld` per tick while a cloud is open, a `cloudv` record per filed cloud (voxels as
`[ix, iy, iz, hits, mean_height_mm]`, the world pose it was anchored on — instrumentation for the
viewer — plus `revisit` and `revisit_dist`), and `cldp` under `--log-cloud-profile`. The duck viewer
draws filed clouds at their world anchor, coloured by mean point height (`P` toggles, `N` solos a
place), in live, replay and record.

**Guards.** On the final binary `--cloud` absent reproduces the R46 reference byte-for-byte (md5
`e4b5c3fa…`). `test_cloud_map` 7/7 (inert without an input, the hysteresis, the de-rotation sign,
cache and identical revisit, alignment through the pose, profile bounds, a flat floor is not a break);
schema-defaults 1/1, state prior 23/23, motor EPM 36/36. Read-back, seed 6, 1500 s: 11 clouds, one per
stop, ~840 voxels, 8 cached.

**What the cache does and does not do yet.** The map mints new nodes at nearly every stop — the places
filed were `[5, 8, 6, 15, 12, 17, 22, 8, 37, 39, 44]`, one genuine revisit in 1500 s. On that revisit
the two anchors were **55.5 cm and 28.8° apart**. Aligned by the true poses, 0.639 of the new cloud was
absent from the old; aligned by the odometry, 0.824; unaligned, 0.864. The transform does real work;
the pose it is given is the weak link — dead reckoning drifts 4–6 % of distance travelled (§16.3),
minutes separate the visits, and one map node spans more ground than a voxel comparison tolerates.
`revisit_dist` is logged beside every judgement so the number is never read alone. The open fix is
registering the two clouds by their own content. (A first version judged revisits mid-accumulation
against whichever winner led early — another place's cloud — and was moved to file time.)

**Three bugs, and the claims they carried.**

1. **The OOM.** `view.py record` kept every rendered frame in a list and wrote the video at the end —
   fine for the 8 s clips it was built for. Pointed at a 1500 s playroom run (75 000 frames at
   2.07 MB), the kernel killed it at **23.9 GB RSS** (08:35:33). It now streams to the encoder, peaks
   at 711 MB for 4 300 frames, and takes `--from / --to / --every`.

2. **The de-rotation had the wrong sign** — in the module and in the offline analysis behind §17.28.
   A body that yaws +d sees a world-fixed point rotated by −d, so undoing it takes R(+d); R(−d) was
   applied, which doubles the heading smear. Writing the unit test caught it: a synthetic body turned
   0.5 rad put **64 of 64** voxels in new cells. On 11 real stops (seed 6) the as-built sign was
   sharpest on none and worse than no rotation at all (14 156 distinct voxels against 13 134); the
   correct sign was sharpest in total (12 795). **Withdrawn from §17.28:** the "+7 % distinct voxels"
   (the smear, read as detail — for a static scene more distinct voxels is worse) and the de-rotated
   cloud's "45 %" detection. **Re-measured with the correct sign:** sweep voxels −1 % (seed 6) and
   +4 % (the rolling-ball run) — de-rotation barely matters at ~10° of drift — and ball detection at a
   matched 5 % false-positive rate is **41 % for the cloud with or without de-rotation** (AUC 0.866 /
   0.865). The cloud's 41 % against a single frame's 18 % stands; it never depended on de-rotation.

3. **The floor was being counted as things standing on it.** Voxels were classified by their centre;
   the ground layer spans 0–4 cm, its centre is exactly `break_lo` (0.02 m), and `0.02 < 0.02` is
   false — so every ground voxel landed in the floor-break band. In the R46 dumps **4 326 of the 5 277
   voxels (82 %) the break profile counted were bare ground**, 93–96 % on some stops. That profile is
   the object EPM's input, and the host-side profile §17.29 measured on had the same test. Voxels are
   now classified by the mean height of the points in them: the same stop's cone reads **56 % ground,
   13 % floor break**. The replay shows it — the floor draws grey, and the break band hugs the ball, the
   block and the base of the walls. (A wall's bottom 20 cm is a floor break by definition, so it is the
   profile's range and extent terms, not the band, that must tell a wall base from an object.)

**And a verification trap worth recording.** The first round of verification ran on a stale binary:
a parallel host build, its output filtered to lowercase `error`, never ran, so the gain-0 check, the
R46 run and a replay all came from code built before any fix. The unfiltered rebuild then surfaced a
`printf` with one more argument than specifiers — adding `revisit_dist` had shifted the logged anchor
by one field and dropped the world yaw. Both are fixed and everything above is from the final binary.
Never filter a build to "error": the warning was the bug.

**The object EPM, re-measured on the fixed profile.**
On the module's own profile, with the floor classified by mean point height, six seeds pooled into
one EPM per arm (33 474 frames, 45 pose cells; H(object) 1.705 nats and H(object | pose) 0.136, so
pose alone fixes 92 % of what is in view):

| view | nodes | baked | switch/min | I(W;O \| pose) | of ceiling | shuffle floor | F1 block / ball / chair / shelf |
|---|---|---|---|---|---|---|---|
| **cloud profile** | 16 | **16** | **6** | **0.129** | **95 %** | 0.001 | **0.95 / 0.96 / 0.91 / 0.93** |
| 8 column minima | 21 | 20 | 21 | 0.102 | 75 % | 0.001 | 0.85 / 0.70 / 0.73 / 0.73 |
| 64 raw zones | 16 | 14 | 13 | 0.087 | 64 % | 0.001 | 0.61 / 0.41 / 0.67 / 0.41 |

The ordering survives the fix and so does the gap: with the floor out of the profile, the cloud
vocabulary captures 95 % of the object information pose leaves, twenty points above the column minima,
and it is steadier than before (six winner switches a minute). §17.29's 98 / 72 / 55 cannot be compared
number for number, and the reason is worth recording because it looked like a confound and is not one.
The frame arms moved too (72 → 75 %, 55 → 64 %), which would mean the duck had walked differently — so
passivity was checked directly: on seed 1 the R43 graph with the points published, the R46 graph with
its modules idle, and the R46 graph with them running all reproduce the reference byte-for-byte (md5
`e4b5c3fa…`). The walk is unchanged. What changed is which frames the bench sees: it feeds only frames
that carry a profile, the old host cloud was open exactly during scheduled stops, and the module opens
on the body's own stillness. Its windows differ, reach more poses (a block from 12 pose cells rather
than 8), and hand all three arms the same different set. The honest comparison is within a run, and
within this one the cloud leads every column.

An observation on the stillness rule: per seed the module filed 11, 12, 11, 14, 11 and 17 clouds against
11 scheduled stops, and two runs averaged 491 voxels a cloud against ~840 for the rest — stillness
opening a cloud outside a stop, or a stop split where the body moved for longer than `move_ticks`. The
cache keys on place, so this is not wrong, but a cloud of a few hundred voxels is a thinner comparison.

**Verdicts.** `CloudMap` as a module `WORKING` (substrate, guarded, tested, and passive — the walk is
byte-identical with it running). The object vocabulary over it `WORKING` on within-pose object content
(95 % of the ceiling on the fixed profile, against 75 % and 64 % for the frame views); pose invariance
still untested, and still gated on travel. Cache-by-place `WORKING` as a store, `PARTIAL` as a revisit
judge — dead-reckoned alignment and a coarse place key; re-use context: content-based registration, or
revisits close enough in time that odometry has not drifted. Replay `WORKING`. The three bugs fixed, with
the claims they carried withdrawn or re-measured above. Nothing promoted.

**The static voxel viewer (built after the above).** `tools/run_voxel_viewer.sh RUN.jsonl` opens every
filed cloud of a run in an interactive 3D view (`tools/xaq_inspector/voxel_viewer.py`, PyQt6 +
pyqtgraph's GL view, no brain connection; usage in the inspector README). World lays the clouds out at
their anchors, which are instrumentation. Body shows one cloud in its own frame, which is what the duck
has. Colour is by the mean-height bands or by hits. One thing the first render taught: a thin streak
in the body view is not a fault. Seed 6's second place-8 cloud was anchored 11 cm from a wall and
facing along it (yaw 169°), and a wall seen that way is a line whose far end reads tall, because the
sensor's vertical fan widens with range.

### 17.31 Small things on the floor: the stack rule, and a gaze that never holds (R52, 2026-09-13)

**The operator's direction.** From the voxel viewer: the ToF is myopic, but objects on the floor are clear; a
duck that walks around finding objects smaller than itself and trying to pick them up would make an
interesting ten minutes, and voxels that stack taller are obstacles. Then, watching R46's stops: the gaze
"moves to an angle, pauses, moves to another angle, pauses" for half a minute and never covers the angles the
head can traverse, while the cloud could be accumulating the whole time the head moves.

**The stack rule, measured before anything is built on it.** In the cloud's own frame: break-band voxels
(2–20 cm mean height) grouped into 8-connected columns; each cluster's stack top is the contiguous chain of
voxel heights over its footprint (dilated by one voxel) with a gap of max(10 cm, 0.12 × range), because the
sensor's rows are 5.625° apart and the vertical spacing of its returns grows with distance. A cluster that
tops out below 16 cm and spans at most 20 cm is a small thing; one that keeps rising is an obstacle. Scored
against the scene manifest and the free bodies' simulated positions at filing (instrumentation, never an
input) on R46 runs, seeds 1–5, which are out of sample: the range-scaled gap was chosen after seed 6 showed
distant wall bases breaking a fixed gap's chain.

| what counts as a thing | flagged | real objects among them | real objects caught |
|---|---|---|---|
| break band alone (the object EPM's input today) | 342 | 11 % | 100 % |
| stack top < 16 cm, fixed 10 cm gap | 145 | 26 % | 100 % |
| stack top < 16 cm, gap max(10 cm, 0.12 × range) | 70 | 53 % | 100 % |
| stack top < 12 cm, same gap | 61 | 59 % | 97 % |

What still passes: thin chair legs seen from far off (13; a leg's footprint is 8 cm against 12–16 cm for the
objects), wall fragments (12), and six others. The break profile the object EPM reads cannot draw this line at
all — every one of its terms lives inside 2–20 cm — which is §17.30's wall-base caveat. Three more facts from
the same logs bear on seeking small things: they are seen at 0.6–2.2 m (median 1.3 m); the nearest floor
return during a stop sits 0.42–0.67 m out (seed 6), so the last half-metre of an approach is blind at the
stop's gaze; and a stop holds a detectable small object 0.15 times a minute with two balls and two blocks in
the room. "Pick up" has no simulated counterpart yet: no MJCF variant has the mouth hinge, Pollen's
`ground_pick` is a phase-scripted 4 s cycle (`robotd/src/control.rs`), and the skill runner (X2) is not in the
host.

**Built: `--stop-gaze-sweep SPEED YAW_MAX`.** Two measurements say why the babble cannot fill a cloud. During
an R46 stop the head's joints move on **6 %** of the ticks and hold for the other 94, inside a yaw clamp of
±0.7 rad against a joint range of ±2.97. And the simulated ToF casts one ray per zone, 5.6° apart, so a
7–12 cm ball at 1.3 m (3–5°) can sit between rays for as long as the gaze holds. With the flag the gaze never
holds: it moves at SPEED toward a cell of a yaw × pitch grid over its range (0.1 rad cells, the babble's pitch
band), drawn at random among the cells this stop has looked at least. The error it descends is the stop's own
coverage deficit, so it neither replays a fixed scan nor babbles back over what it has already seen. The
hold's novelty rule runs unchanged on HOLD-long windows of the moving view, and the dwell becomes speed: a
quarter of SPEED while the view is novel, up to MAX. At 12.5 Hz of ToF and 0.3 rad/s the beams advance 1.4° a
frame. Guard: without the flag the seed-1 run's JSON reproduces the reference byte-for-byte (md5
`28fc5942…`; the files differ only by the two banner lines the host prints when the inspector port binds).
`l2_sweep.py` gains `--full-logs` so a harness run keeps the cloud records; `mj_host/tools/cloud_objects.py`
scores both the rule (`rules`) and the arms (`arms`).

**The A/B** (6 seeds × 1500 s, playroom, the R46 host arguments). A moving view keeps the place map
surprised, so the babble's quiet rule rarely ends a sweep's stop early and the sweep stands longer. Hence two
pairs: the arms as deployed, and an equal-length pair (QUIET 999: every stop the full 60 s, so only the head's
motion differs).

| n = 6 | head moving | voxels / cloud | objects ≤ 2 m in the babble's reach found | balls ≤ 2 m found | rule precision | real small things flagged / min | stands held |
|---|---|---|---|---|---|---|---|
| R46 babble | 6 % | 751 ± 165 | 42 / 63 (67 %) | 19 / 90 (21 %) | 0.62 | 0.34 ± 0.22 | 65 / 66 |
| **R52 sweep, ±0.7 rad** | 97 % | **2 271 ± 417** | 75 / 100 (75 %) | 40 / 122 (33 %) | **0.92** | 0.53 ± 0.19 | 65 / 66 |
| sweep, ±1.2 rad | 98 % | 2 399 ± 640 | 61 / 75 (81 %), and 11 / 116 beyond it | 41 / 110 (37 %) | 0.80 | 0.59 ± 0.19 | 63 / 66 |
| babble, full-length stops | 5 % | 760 ± 167 | 41 / 61 (67 %) | 21 / 86 (24 %) | 0.68 | 0.33 ± 0.25 | 65 / 66 |
| **sweep, full-length stops** | 97 % | 2 309 ± 391 | **94 / 109 (86 %)** | **50 / 124 (40 %)** | **0.98** | **0.67 ± 0.36** | 66 / 66 |

Voxels per cloud rise on all six seeds in both pairs; objects in reach found rise on five of six in each (the
sixth: seed 6 falls 87 → 64 % deployed and ties 87 / 86 % at equal length); balls rise on five of six in each.
The voxel viewer shows it at a glance: both arms are identical until the first stop at 600 s, and at seed 6's
first stop the two small objects the babble left as a few scattered voxels are solid clusters under the sweep,
and its walls are continuous surfaces.

The side effects, from the harness (sweep − babble, paired, n = 6): the place map calls **34 % of ticks novel
against 18 %**, holds 5.7 more nodes (4+/0−) and switches winner 17 more times a minute (5+/1−) — its view is
head yaw plus eight ToF column ranges, so a gaze that never holds is a view that never repeats. Stops rarely
end early, so the duck stands 5 % more of the control phase and walks 7.4 m less (0+/6−), and meets walls less
(14.8 → 4.6 per minute), which is walking less rather than avoiding better and is not claimed. The ±1.2 rad
sweep's map holds 19.5 more nodes.

**A sim-to-real caveat.** Part of the gain on balls comes from the simulated sensor: one ray per zone leaves
gaps between zone centres that a moving gaze fills, while the real VL53L8CX integrates each zone's whole 5.6°
cone, so a ball between centres still shifts that zone's return. The coverage gain (a head moving through 97 %
of the stop, the whole reachable field swept) should transfer; the gain on balls probably overstates the
hardware's. On the robot the head angle must also be matched to each ToF frame's timestamp; at 0.3 rad/s,
30 ms of latency is 0.5°, a tenth of a zone.

**Verdicts.** The stack rule `WORKING` as a sensor reduction: precision 0.11 → 0.53 out of sample at full recall
(n = 6, a signal; not yet in `CloudMap`). R52, the gaze sweep at ±0.7 rad, `WORKING` on what it is for: clouds
three times denser on every seed, the rule's precision 0.62 → 0.92, objects in reach found 67 → 75 % (86 % at
equal stop length), balls 21 → 33 % (24 → 40 %), stands unchanged. Its effect on the place map is real and is
the next design question rather than a reason to hold the gaze again. The ±1.2 rad sweep `PARTIAL`: the first
sightings beyond the babble's reach (11 of 116 objects), at the cost of 63 / 66 stands, precision 0.80 and 19.5
extra map nodes; re-use context: a stand that absorbs the head's swing, or a sweep that slows toward the ends
of its range. Nothing promoted; presets R46 and R52 (seed 3) put the pair in front of the operator's eye. Open
(O42): the place map should read the stop's cloud instead of a moving frame, and a stop should end when its
cloud stops growing rather than when the map stops being surprised.

### 17.32 The map reads the cloud, stops end on it, and how fast the head can look (R53–R55, 2026-09-13)

**The operator's direction**, on §17.31's open item: feed the map the cloud and end stops on cloud growth; and,
if possible, let the head traverse faster, as long as the cloud still renders well enough to find small
objects and walls.

**Built, each behind its own flag, all three off by default.**
- **R53 `--map-view cloud`.** `ogma::CloudMap::view()` is the cloud as a view: across ±64° of its own de-rotated
  frame (a ±0.7 rad sweep plus half the sensor's field), 8 sectors, each the nearest voxel whose mean height
  clears the floor, divided by 4 m (the frame columns' own scale; empty = 1). With the flag, the place map's
  view slots carry it instead of the frame in front of a moving head, and head yaw reads 0. The map's 13-dim
  input and its config are unchanged. While no cloud is open (the walk), the slots hold the last cloud's view,
  so a walk is matched by pose against the places the stops learned. `test_cloud_map`
  `ViewIsTheNearestOffFloorReturnPerSector`: empty with no cloud, a bare floor is not a view, the nearest
  standing thing lands in its sectors.
- **R54 `--stop-cloud-end F`.** A stop ends once the open cloud's growth (new voxels over the last 2 s) has
  stayed below F × the highest growth this stop has shown, for 2 s more. It is scale-free, since each stop is
  judged against its own peak, and it replaces the gaze's quiet rule. F = 0.1 came from R52's full-length
  stops: it would have ended them at a median 16 s holding 62 % of their 60 s voxels (F 0.05: 24 s, 72 %;
  0.2: 12 s, 54 %).
- **R55 `--stop-gaze-sweep-slow F`.** The sweep's speed while the map finds the view novel, as a fraction of
  SPEED (0.25 is R52). Measured first: R52's head sat at the quarter speed on **69 %** of stop ticks, so the
  slow-down, not SPEED, set how fast it looked around. R55 is the sweep at one constant speed (F = 1) at 0.3,
  0.6, 1.0 and 1.5 rad/s.

Guards: CloudMap 8/8, schema defaults 1/1; the flags absent, seed 1 of R46 and of R52 reproduce their A/B logs'
JSON lines (md5 `28fc5942…`, `4b182be6…`).

**The ladder** (l2_sweep, 6 seeds × 1500 s, playroom, R46 host arguments; R52 as the control; each arm adds to
the one above it, the speed arms to R54).

| n = 6 | stop (s) | stands held | map switch / min | map nodes | map TLE | straight | path (m) | cells | walls / min |
|---|---|---|---|---|---|---|---|---|---|
| R52 sweep (control) | 58.5 | 65 / 66 | 77.4 | 46.5 | 0.17 | 0.08 | 42 | 62 | 4.6 |
| R53 + the map reads the cloud | 35.8 | 64 / 66 | **24.9** | 39.0 | 0.25 | 0.18 | 85 | 112 | 8.2 |
| R54 + stops end on growth | **23.6** | 66 / 66 | 23.5 | 31.8 | 0.29 | 0.16 | 108 | 105 | 5.0 |
| R55 constant 0.3 rad/s | 26.6 | 66 / 66 | 23.2 | 31.8 | 0.26 | 0.17 | 104 | 107 | 8.0 |
| constant 0.6 rad/s | 24.0 | 64 / 66 | 24.1 | 26.3 | 0.27 | 0.26 | 105 | 147 | 23.7 |
| constant 1.0 rad/s | 25.5 | 63 / 63 | 20.5 | 23.7 | 0.27 | 0.17 | 103 | 113 | 22.7 |
| constant 1.5 rad/s | 24.3 | 66 / 66 | 20.0 | 22.3 | 0.25 | 0.18 | 109 | 81 | 4.6 |

| n = 6 | head speed p50 (rad/s) | voxels / cloud | objects ≤ 2 m in reach found | balls found | rule precision | walls and furniture read as small |
|---|---|---|---|---|---|---|
| R52 | 0.08 | 2 271 | 75 / 100 (75 %) | 40 / 122 (33 %) | 0.92 | 3 / 221 (1.4 %) |
| R53 | 0.08 | 1 806 | 47 / 58 (81 %) | 30 / 85 (35 %) | 0.72 | 14 / 297 (4.7 %) |
| R54 | 0.11 | 1 463 | 52 / 73 (71 %) | 31 / 83 (37 %) | 0.84 | 8 / 302 (2.6 %) |
| **R55 0.3** | 0.30 | 2 026 | **58 / 69 (84 %)** | 29 / 81 (36 %) | **0.86** | 9 / 345 (2.6 %) |
| **R55 0.6** | 0.60 | 1 691 | 39 / 46 (85 %) | 19 / 82 (23 %) | 0.82 | 9 / 393 (2.3 %) |
| R55 1.0 | 1.00 | 1 669 | 46 / 62 (74 %) | 26 / 93 (28 %) | 0.73 | 14 / 369 (3.8 %) |
| R55 1.5 | 1.49 | 2 281 | 52 / 60 (87 %) | 22 / 65 (34 %) | 0.71 | 11 / 312 (3.5 %) |

**What it says.**
1. **The map reading the cloud is loud.** Winner switches fall from 77 to 25 a minute on every seed; walks
   straighten on every seed (+0.10); the map settles on fewer places (46 → 39, nearly all baked); and stops
   shorten to 36 s even under the quiet rule, because a sweeping gaze no longer surprises it. The cost is the
   map's own error, up 0.08–0.13 on every seed: on the walk the held view is the last stop's, so the node the
   pose finds fits less closely. The per-stop cloud also thins, as R53's stops are shorter while the slow-down
   still holds the head at 0.08 rad/s (precision 0.92 → 0.72).
2. **Ending on growth sets the tempo.** Stops fall from 58.5 to 23.6 s, 62 of 66 end on growth, and every
   stand holds. The duck walks 66 m more and reaches 43 more cells (both 6+/0−). Per-stop quality is uneven:
   two seeds find only 2 of 5 objects in reach.
3. **Turning the slow-down off recovers the cloud at the same tempo.** At a constant 0.3 rad/s: 2 026 voxels a
   cloud in 27 s against R52's 2 271 in 58 s, objects in reach 84 % (75–93 % on every seed), precision 0.86.
   The dwell had cost the cloud more than it gave it.
4. **Faster: small things hold, obstacles fray, and stops do not shorten.**
   - Objects in reach and balls show no reliable trend with speed (84 / 85 / 74 / 87 %, 36 / 23 / 28 / 34 %).
     The ToF still interleaves at 1.5 rad/s, 6.9° a frame, because a 24 s stop makes some 25 passes.
   - The obstacle side degrades above 0.6 rad/s: precision 0.86 / 0.82 / 0.73 / 0.71, walls and furniture
     read as small 2.6 / 2.3 / 3.8 / 3.5 %, and per-seed precision floors of 0.57 at 1.0 and 0.22 at 1.5.
   - Stop length is 24–27 s at every speed. The growth rule is relative to the stop's own peak, and a faster
     head raises the peak along with everything else, so how long a stop lasts is F's to set, not the head's.
5. **Walking twice as much** gives the walk's own wall problem twice the room to show. Walls tie at 0.3
   (8.0 against 4.6, t 0.6), while 0.6 and 1.0 read 23.7 and 22.7 a minute with single seeds at 45–62. The
   rate does not rise with speed (1.5 reads 4.6) and R48's step is not in this stack, so none of it is
   attributed to the head. The 1.0 arm's 63 stops are one seed's 13 walking rescues pushing its stops past
   the schedule; every stop that started held.

**A sim-to-real caveat on speed.** The simulated ToF reads the head's pose at the instant of the cast. On the
robot a 30 ms mismatch between a frame and its head angle is 0.9° at 0.6 rad/s but 1.7–2.6° at 1.0–1.5, a
third to a half of a zone, so the hardware's ceiling is probably lower than the simulation's.

**Verdicts.** R53, the map reading the cloud: `WORKING`, loud on the map's steadiness (switches −52 a minute,
6+/0−), with its error up as the cost. R54, stops ending on cloud growth: `WORKING` (stops 2.5× shorter,
stands 66 / 66), per-stop quality uneven. R55, a constant sweep: `WORKING` at 0.3–0.6 rad/s, and `PARTIAL`
at 1.0–1.5, where small objects hold but walls start breaking into false small things, and speed buys no
shorter stops. The stack for the operator's eye is R53 + R54 + a constant 0.6 rad/s: the fastest speed whose
object and obstacle numbers both hold, with balls (19 of 82) the thing to watch, and 0.3 as the quality
reference. Presets R55 (0.6) and R55b (1.5), seed 5. Nothing promoted. Re-use context for the fast sweep: a
head-angle timestamp per frame on the robot, or a wall-continuity term that does not break at speed. Open:
the map's error on the walk (a held view is a stale one), and F as the tempo knob.

**PROMOTED (the operator's eye, 2026-09-15):** R55 at 0.6 rad/s is "the best choice to promote. Faster head
movement looks unnatural and does not seem to speed up stops." R53 + R54 + a constant 0.6 rad/s sweep is the
stop's look: `★ CLOUD` in the launcher. R55b (1.5 rad/s) stays in the list as the measured edge. The operator's
next step, and why: the shorter the stop, the more interesting the robot. It glances around, has an idea of
where the walls are even if not a perfect one, and still finds the smaller objects that matter most. The growth
threshold is the knob for that (§17.33).

### 17.33 How short a stop can be: the growth threshold (R56, 2026-09-15)

**The question.** The operator, promoting R55: the shorter the stop, the more interesting the duck. It glances
around, has an idea of where the walls are even if not a perfect one, and still finds the smaller things that
matter most. On R55 the stop's length belongs to `--stop-cloud-end F`; §17.32 showed head speed does not move
it. The study is F: 0.1 (R55 itself), 0.2, 0.3, 0.45 and 0.6, at 6 seeds × 1500 s each, with the rest of the
R55 stack untouched. The base arm reproduces §17.32's 0.6 rad/s arm exactly (stands 64 / 66, mean stop
24.0 s), a free determinism check.

**A wall measure for "an idea of where the walls are"** (`cloud_objects.py arms`, added for this). Take every
2° across the cloud view's ±64°. Among those bearings whose room wall lies within 2.4 m of the stop (the cloud
keeps returns to 2.5 m), it counts the share where the cloud holds an off-floor voxel within 2° and 12 cm of
that wall's true range. Furniture standing in front of a wall hides it equally in every arm. As a check on the
measure itself, R52's 58 s stops locate 70 % and R55's 24 s stops 66 %.

| F (n = 6) | stop p10 / p50 / p90 (s) | cloud open | voxels / cloud | walls located | objects ≤ 2 m in reach found | balls found | rule precision | walls and furniture read as small | real small things flagged / min | stands held |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.1 (R55) | 11.8 / 20.7 / 35.9 | 93 % | 1 691 | 66 % | 39 / 46 (85 %) | 19 / 82 (23 %) | 0.82 | 2.3 % | 0.33 | 64 / 66 |
| 0.2 | 9.0 / 18.1 / 26.0 | 80 % | 1 473 | 60 % | 51 / 69 (74 %) | 20 / 82 (24 %) | 0.89 | 1.0 % | 0.43 | 61 / 63 |
| 0.3 | 10.1 / 13.6 / 25.8 | 63 % | 1 528 | 70 % | 71 / 89 (80 %) | 35 / 115 (30 %) | 0.82 | 3.7 % | 0.59 | 66 / 66 |
| **0.45** | 7.3 / **10.5** / 15.3 | 54 % | 1 296 | **70 %** | 56 / 78 (72 %) | 30 / 87 (34 %) | 0.76 | 4.3 % | 0.45 | 66 / 66 |
| 0.6 | 6.6 / 8.4 / 12.1 | 58 % | 1 004 | 62 % | 47 / 66 (71 %) | 21 / 93 (23 %) | 0.72 | 3.8 % | 0.35 | 64 / 64 |

"Cloud open" is the share of a stop's ticks with the cloud open.

**What it says.**
1. **Stops halve and tighten.** From F 0.1 to 0.45 the median stop falls from 20.7 to 10.5 s and the 90th
   percentile from 35.9 to 15.3 s, and every stand holds.
2. **The walls hold.** 60–70 % of the in-reach wall is located at every F up to 0.45 (70 ± 4 % per seed at
   0.45). At 0.6 it slips to 62 %, and the worst quarter of clouds locate 29 % or less.
3. **Small things hold, within seed noise.**
   - Objects in reach: 85 % at 0.1, then 71–80 % from 0.2 to 0.6.
   - Balls: 23–34 %, with no trend.
   - Real small things flagged per minute of run, which counts the walking the shorter stops buy: highest at
     0.3 (0.59), 0.45 at F 0.45, against R55's 0.33.
4. **The obstacle side is what shortening costs.** Precision is 0.82–0.89 up to F 0.3, then 0.76 and 0.72,
   and walls and furniture read as small rise from 1–2 % to about 4 %. A shorter cloud has fewer returns up
   a wall's face to chain its stack.
5. **There is a floor.** A stop cannot end before its cloud has opened (half a second of stillness after the
   settle) and been judged over two 2 s windows. So at F 0.45–0.6 the cloud is open for only 54–58 % of the
   stop, and F 0.6's 10th-percentile stop (6.6 s) sits on that floor. Stops much under 8 s would need the
   window shortened, not F.
6. **Behaviour.** The duck stands 5–12 % less of the control phase and walks 9–25 m more a run (5+/1− to
   6+/0−), wall contacts a minute tie at every F (sd 9–29), and the map's error rises a little further
   (+0.02–0.05). One line is unexplained: at F 0.3 cells fall by 50 and straightness by 0.13 on every seed,
   and neither neighbouring F shows it.

**Verdicts.** F 0.1 → 0.45 `WORKING` for the operator's aim: the median stop halves (20.7 → 10.5 s) while the
walls are still located (70 %) and small things still found (objects in reach 72 %, balls 34 %), at the cost of
the obstacle side (precision 0.82 → 0.76, misreads 2.3 → 4.3 %). F 0.6 `PARTIAL`: its stops are the shortest
(8.4 s) and sit on the floor, but the cloud is 41 % thinner, the walls slip (62 %, a quarter of clouds at 29 %
or less) and precision is 0.72. Recommended for the operator's eye: F 0.45 (preset R56, seed 3), with F 0.6 as
the edge (R56b) and F 0.3 as the middle between quality and tempo. Nothing promoted; n = 6 is a signal. Open:
the stop's schedule is still a timer (every 80 s from 600 s), so a shorter stop lengthens the walk rather than
adding glances. Whether the duck should stop more often, and on what, is the next question this raises.

**PROMOTED (the operator's eye, 2026-09-15):** R56 at threshold 0.45: "the robot is able to make a very solid
map in a very short amount of time." R56 is `★ CLOUD` in the launcher, superseding R55. This closes the cloud
phase. Its summary is [`microduck_cloud_phase.md`](microduck_cloud_phase.md): the promoted run, what the phase
learned, the tools, the traps, and the open questions for the next one (register O43, when to stop; O44,
seeking small things).

### 17.34 The things phase opens: the stack rule in the module, the attended thing, and a vocabulary of things (R57–R59, 2026-09-15)

**The direction.** The operator, on `★ CLOUD`: map-making is not the interesting thing for the duck to be doing;
it should seek what is smaller than itself, interact with it, and be surprised when it answers. The plan is
[`microduck_things_phase.md`](microduck_things_phase.md) (register O45–O50); this section is its T1, the
sensor side, built passive before any loop reads it. Two corrections from the discussion shaped it: a thing does
not grow on approach (a voxel is world-sized; its sampling grows, so the error an approach reduces is the
descriptor's precision), and the head should look where the thing is (a gaze error, T3), not tilt on a script.

**Built.** `ogma::CloudMap` runs §17.31's stack rule on the OPEN cloud every `things_every` ticks (4, the
sensor's cadence): break-band voxels into 8-connected columns, each cluster's stack top the contiguous chain of
heights over its dilated footprint with a gap of max(`gap_min`, `gap_k` × range), SMALL when the top is under
`small_top` (0.16) and the footprint within [`small_ext_min`, `small_ext`] (0, 0.20). The nearest small cluster
is ATTENDED. Two new topics, both empty by default so a graph without them is byte-identical: `things_topic`
carries the attended thing's descriptor, world-sized and in [0,1] and without a bearing (top, footprint, aspect,
columns, hits per column, chain, range, lowest height), published only while a thing is attended; and
`thing_bearing_topic` carries `[vx = +right, vy = +forward, proximity]` in the BODY frame, the cloud's frame
turned back by the yaw drift since the anchor, the shape `VisualBearing` emits so `VisualHomingNav` consumes it
unchanged, and it is published every tick the topic exists, reading proximity 0 while the body walks (a
consumer must not home on the last stop's stale bearing). The host logs `thg` (the attended thing per compute
tick, in the cloud's frame), `tepm` (the thing EPM's token) and `things` (the filed cloud's clusters), and
`cloud_objects.py things` scores them against the manifest. R57 (`a1v2_r57_things.json`) is R46 plus the
topics and a thing EPM (`rbf`, 8 dims, bake 20). Tests 9–13 of `test_cloud_map` pin the default-off contract,
the rule on a cube and a rising post, the bearing's frame, filing, and the floor.

**Guards.** R46 on the pre-change and post-change binaries: the same md5 over every JSON line (760 s, seed 3).
R57 against R46 on the post-change binary: 38 000 lines identical once `thg`/`tepm`/`things` are stripped, and
at n = 6 × 1500 s R57's `arms` scores equal R56's F 0.45 arm to the last digit (13.0 clouds a run, 1 296
voxels a cloud, precision 0.76, walls 70 %). The module's clusters against the offline rule on the same 78
filed clouds: 583 of 583 matched by centroid agree on the verdict, top and footprint deviating by 0.0 cm.

**Measured, n = 6 × 1500 s, the playroom, `★ CLOUD`'s arguments** (4 903 attended ticks on R57):

| | R57 (no floor, full descriptor) | R58 (floor 0.08, shape-only descriptor) | R59 (floor 0.08, full descriptor) |
|---|---|---|---|
| a small cluster in view, % of thing ticks | 68 | 55 | 55 |
| the attended thing is a real object (precision) | 0.73 | **0.82** (5+/1−) | **0.82** (5+/1−) |
| balls attended, ticks | 1 452 | 1 395 | 1 395 |
| thing EPM nodes seen per run | 23.7 | 14.7 | 21.2 |
| per-run majority-label purity (chance) | **0.87 ± 0.05** (0.43) | 0.83 ± 0.07 (0.52) | **0.91 ± 0.07** (0.52) |
| purity − chance | **+0.44 ± 0.06** | +0.30 ± 0.15 | +0.38 ± 0.16 |

What the attended thing was on R57: block 2 112, ball 1 452, chair 605, wall 557, table 87, other 75, shelf 15.
The misses are one-column clusters (columns p50 = 1, hits per column 4–8 against the objects' 4–5 columns
and 8–18 hits): a wall base or a chair leg that the sweep has landed a few rays on. Offline on the same ticks:
requiring the attended cluster to be UNCHANGED for K recomputes changes nothing (K 0 → 12: 0.73 → 0.75, at
57 % of ticks kept), because a fragment that gets no more rays is as stable as a ball; requiring two columns
gives 0.86 at 73 % kept and three 0.93 at 61 %. Hence `small_ext_min`, a floor on the footprint in the rule's
own units (0.08 m = two columns): a real ball is one column early in a sweep too, so the floor delays attention
until the thing is sampled, which is the point.

**Two catches on the way (§3.2).** First, the thing EPM's purity was read at 0.56 when the scorer pooled node
ids across runs; every seed grows its own EPM, so node 9 in one run is unrelated to node 9 in another. Scored
per run it is 0.87. Second, R58 bundled two changes, and they disagreed: the floor raised precision on five of
six seeds (seed 3: 0.56 → 0.49), while the shape-only descriptor LOWERED the vocabulary's purity (+0.44 →
+0.30). Offline bins over (top, footprint) had read 0.76 and suggested the sampling dims were noise; in the
EPM they are shape. On R57 a block's flat face returns 18 hits per column and a ball's curved face 8, and the
first run's nodes divide the two cleanly (node 11: 106 block, 6 ball; node 10: 69 ball). R59 is the floor
alone.

**Pose invariance, still untestable.** Each real object is attended from 1.3–2.6 poses a run, and its modal
winner holds 0.40–0.66 of its ticks. O40's limitation stands: the duck does not travel enough for the test.

R59 keeps R58's attention exactly (the floor decides attention, the descriptor does not) and R57's vocabulary:
per-run purity 0.91 ± 0.07, 21 nodes a run. Purity minus chance is the weaker comparison here, since the floor
also raises chance (fewer fragment labels among the attended ticks: 0.43 → 0.52).

**Verdicts.** T1's reduction `WORKING` as a sensor (exact against the offline rule; attention on a real object
on 82 % of attended ticks with the floor, up on five of six seeds). The thing EPM's vocabulary `WORKING` on kind (per-run purity 0.87 against chance 0.43, blocks
and balls under separate nodes) with the FULL descriptor; the shape-only descriptor `REGRESSION` on it
(re-use context: a sensor whose returns per column do not depend on the surface, or a descriptor that carries
curvature explicitly). The stability gate on attention `NULL` offline, not built (re-use: a moving scene, where
stability would mean something). R59 is the T1 config the seek loop (T2) builds on. Passive on every seed:
the walk is byte-identical to `★ CLOUD`, so there is nothing for the operator's eye yet; presets R57–R59
(seed 3) show the cloud panel's counts at each stop.

### 17.35 The seek loop: the duck walks to a thing it saw, remembering where it was (R60, things phase T2, 2026-09-15)

**Built.** `ogma::BearingSeekLoop` (new, generic, registered): the duck sees a small thing only while it stands
(the cloud exists at a stop and the thing bearing reads proximity 0 the moment the body walks), so a loop that
walked toward things would be silent for the whole walk unless it remembered where the thing was. While the
bearing is live the loop fixes the thing's POSITION by dead reckoning (the body's odometry pose, published by
the adapter as `reality.proprio.odom`, plus the bearing and the range the proximity encodes); while the bearing
is silent it homes to that position, re-aiming as the body moves and turns, until the remaining range falls
under `arrive_m` (0.25) or its confidence decays to the floor (`forget_ticks` 3000, about 60 s to 0.37). The
Cell's `VisualHomingNav` remembered an allocentric bearing through an occlusion; the duck has range, so the
belief is a position and arrival is the loop's own. Its need is the confidence in the held target; its honest
signal, for `LoopCompetence` (sign −1), the range left. R60 (`a1v2_r60_seek.json`) is R59 plus the loop and
R28's arbitration with seek on the arbiter's vision channel: G_seek = need × trust, G_play = (1 − need) ×
trust; the adapter takes the winner's bearing (steer 3). A host flag `--seek-gate` (R60g) makes the ToF sense
slot of the target's sector read free while seek holds the reference, so the twist brain's proximity prior does
not push the body off the thing it walks to. Four unit tests pin the loop (`test_bearing_seek_loop`); the
unchanged R46 config is byte-identical on the new binary (md5 `cb24520c…`); `l2_sweep.py` gains `seek%`,
`seekHeld%` and `seekEnds`; `cloud_objects.py seek` scores every episode against the manifest.

**The smoke run first** (seed 3, 900 s): at the first stop the loop fixed a target at 1.52 m, the body homed
to it on the walk (range 1.52 → 0.30 m over 46 s, seek winning every tick), and once the range stopped falling
the arbiter handed the reference back to play. The target was a wall base the rule had read as small (the
18 % false-positive class of T1), 17 cm from the wall; the frames are right (the attended cluster's world
position from the anchor matches the direction the body took) and the dead-reckoned endpoint sat 0.5 m from it.

**Measured, n = 6 × 1500 s, the playroom, `★ CLOUD`'s arguments, against R59:**

| | R59 (base) | **R60 seek** | R60g seek + gate |
|---|---|---|---|
| seek wins the reference, % of ticks | — | 28 ± 8 | 24 ± 10 |
| a target held, % of ticks | — | 55 ± 16 | 49 ± 20 |
| seek episodes (all seeds) | — | 21 | 23 |
| targets that were real objects | — | 17 / 21 (11 blocks, 6 balls) | 20 / 23 |
| blocks: closest approach p50 · within 0.4 m · an object touched | — | **0.27 m · 7 / 11 · 3** | 0.24 m · 11 / 14 · 3 |
| balls: closest approach p50 · within 0.4 m · touched | — | 0.52 m · 1 / 6 · 0 | 0.45 m · 3 / 6 · 2 |
| episodes ending by arrival / forgetting | — | 20 / 1 | 21 / 2 |
| walls / min (per seed, down vs base) | 21.7 ± 23.2 | **12.4 ± 14.4** (4 / 6) | 28.9 ± 26.9 (2 / 6; seed 4: 13.9 → 80.9) |
| cells | 145 ± 33 | 124 ± 29 (3 / 6 down) | 131 ± 34 |
| object contacts / min · objects moved, m | 14.1 ± 18.8 · 3.5 ± 2.0 | 19.2 ± 26.8 · 3.4 ± 2.2 | 5.9 ± 7.8 · 3.0 ± 2.1 |
| stands held · stops · stop length | 66 / 66 · 11 · 15.0 s | 63 / 65 · 10.8 · 14.5 s | 66 / 66 · 11 · 15.4 s |

**Reading it.** The mechanism is loud at the episode level: 21 episodes, 17 of them at a real object, and for
blocks the body comes within 0.4 m of the thing's true position on 7 of 11 by dead reckoning alone, three times
into contact (two episodes of 15–19 s pushing a block). Balls are harder (median 0.52 m): the ball is the
thing most likely to have moved between the stop that fixed it and the walk (the base already displaces the
room's four movables by 3.5 m a run through the duck's ordinary stumbling), and its cluster centroid is the
least stable. Arrival by dead reckoning ends 20 of 21 episodes. On the aggregate metrics the lever ties or
helps: wall contacts fall on four seeds of six (the seek heading is a steady direction, like play's), coverage
ties, stands hold. Object contacts and displacement are blind here: the base moves objects 3.5 m a run
without seeking anything, so "interaction" cannot be read from displacement until the duck stops at the
thing (T4) and something distinguishes a sought contact from a stumbled one.

**The gate.** Blinding the target's ToF sector while seek holds the reference brings the body closer (blocks
11 / 14 within 0.4 m, balls 3 / 6) at the cost of the walls: 28.9 / min against 12.4, up on four seeds of six,
seed 4 to 80.9. A held target 17 cm from a wall, or a wall behind the thing, is exactly what the sector gate
hides. `REGRESSION` in this form; re-use context: a gate limited to the last half metre of the approach, or one
that opens only for a target whose vocabulary node is a thing's rather than a wall's (the thing EPM separates
them at purity 0.9).

**Verdicts.** T2's seek loop `WORKING` as a mechanism and `PARTIAL` as a behaviour at n = 6: it takes the duck
to the things it saw, walls do not rise, and the interaction the phase is for needs the stop at arrival (T4)
before it can be read. Not promoted; preset R60 (seed 3, fast-forward through 600 s) puts the first walk-to-a-
thing in front of the operator's eye; R60g the gated form. Next: T4's arrival stop, then T3's gaze at the
thing, so that a thing reached is a thing looked at.

### 17.36 A stop that starts on arrival (R60a, things phase T4, 2026-09-15)

**Built.** `--stop-on-arrive`: a stop starts when the seek loop drops a target it has reached (its need goes to
0 with the range under 0.3 m), with the same settle, hand-back, sweep and cloud as a timer stop, and the same
ending on the cloud's growth. The 80 s timer stays as the floor, so a duck that has seen nothing still
glances (the plan's "retire the timer" is deferred until the map's walk-time error is the second trigger).
`cloud_objects.py stops` scores every stop by how it started, how near the nearest object was when it began,
and what its cloud attended; `l2_sweep.py` gains `stopsArrive`. R46 stays byte-identical (md5 `cb24520c…`).

**Measured, n = 6 × 1500 s, against R60** (the same seeds, the same arguments plus the flag):

| | R60 seek | **R60a seek + arrival stops** |
|---|---|---|
| stops a run · of which on arrival | 10.8 · — | 15.3 ± 2.4 · **5.3 ± 3.3** (32 of 92) |
| nearest object when an arrival stop begins, p10 / p50 / p90 | — | **0.13 / 0.21 / 0.29 m** (29 of 31 within 0.5 m) |
| nearest object when a timer stop begins, p50 | 1.14 m (seed 5) | 1.00 m (13 of 47 within 0.5 m) |
| the arrival stop's cloud attends a real object · one within 0.8 m · nothing | — | 18 / 31 · 8 / 31 · **12 / 31** |
| stop length p50 · mean (arrival / timer) | 14.5 s | 10.9 / 12.8 s · 20.9 / 28.7 s (25 of 92 reach the 60 s cap) |
| walk / stand, % of the control phase | 82 / 16 | 54 / 44 |
| path · cells | 124 m · 124 | 81 m (6 / 6 down) · 89 (5 / 6 down) |
| walls / min | 12.4 ± 14.4 | 20.4 ± 20.7 (3 / 6 up; seed 4: 8.2 → 56.1) |
| object contacts / min · objects moved | 19.2 · 3.4 m | 3.2 · 2.5 m |
| map TLE on the walk · nodes | 0.31 · 23 | 0.21 · 27 |
| stands held | 63 / 65 | 92 / 92 |

**Reading it.** The mechanism is loud: an arrival stop begins a median 0.21 m from a real object, by dead
reckoning from a bearing fixed at the previous stop. The duck walks to a thing it saw and stops beside it.
What it then does is the phase's next finding, and it is what T3 predicted. At 12 of 31 arrival stops the
cloud attends nothing, and at most of the rest it attends a *different* thing 0.8–1.3 m away: the thing
reached sits at 0.2 m, below a level gaze, which first sees the floor at about 0.5 m. The thing reached is the
thing not looked at. Where the reached thing sat at 0.3 m or more it was seen (seed 1 at 772–800 s: ball at
0.30, 0.24, 0.27, 0.26 m, attended at 0.33, 0.30, 0.29 m), and the duck then did the degenerate thing:
attend it, fix it as a target at 0.3 m, walk, arrive at once, stop, four times in 30 s. That is lingering
without habituation; the thing EPM's error at an attended thing (the plan's pull) is what should let it go.

The costs are those of standing: walk time falls from 82 % to 54 %, path and coverage with it (path down on
every seed), wall contacts return to the base's level on three seeds (seed 4's 56 / min is a duck stopping and
starting beside furniture), and a quarter of the stops now run to the 60 s cap, which the cloud's growth rule
had ended in 10 s before (O37's ending question, re-opened by a cloud that keeps growing next to a thing).
Object contacts fall from 19 to 3 a minute because the duck now stops at the thing instead of stumbling
through it; displacement falls with it. Read together: the duck reaches things and stands by them, and cannot
yet see or push what it reached.

**Verdicts.** T4 `WORKING` as a mechanism (arrival stops at 0.21 m, 29 / 31 within 0.5 m), `PARTIAL` as a
behaviour: the reached thing is invisible to the stop, and without habituation the duck loops at a visible one.
Not promoted; preset R60a (seed 1, fast-forward through 600 s, where four arrival stops at a ball follow one
another from 772 s) for the operator's eye. The two levers this hands the plan, in order: **T3's gaze at the
reached thing** (the sweep centred on the target's bearing and expected elevation, about 35° down at 0.3 m,
which O36 showed the stand can take), and **habituation** (the seek need weighted by the thing EPM's error at
the attended thing, so a thing looked at four times lets the duck go). Then the (d) tests with a moved ball.

### 17.37 The circling: a heading reflex, and the play loop's mirrored bearing (R60a → R64, 2026-09-17)

**The operator's eye on R60a (seed 1).** "The duck walks in a small clockwise circle for much of the run
between stops; it isn't until 930 s that it looks across to the other side of the room and scans the balls
and blocks; it does end up in the vicinity of the small objects and continues its circling there. A more
random walk would be more interesting, which may occur if the robot is trying to reduce its yaw error
relative to a small-object target."

**What the circle is made of** (`cloud_objects.py heading`, new: on walking ticks, by which loop holds the
reference). Two things, both in the ledger already, and one of them wrong.

- *Under play* (74 % of walking ticks on R60a): the reference runs ahead of the heading at the body's own
  turn rate (median 0.70 rad/s; faster than 0.5 rad/s on 56 % of ticks), the error sits at 1.56 rad and
  never closes, and the yaw command is on its rail (|vyaw| > 0.9 on 93 %). This is §17.16–17.17's orbit.
- *Under seek* (26 %): the reference is quiet (0.10 rad/s), and the body still circles: the command holds
  +1 while the error crosses zero and grows to 1.4 rad before it flips, with the ToF clear. At forward
  speed a saturated yaw is a circle of about half a metre. This is §17.17's "the regulator does not
  regulate".

**Lever 1, the heading reflex** (`--heading-reflex TAU DAMP GATE`, off by default, R46 byte-identical): the
picrawler's answer (`CLAUDE.md` §1) on the duck's authoritative yaw channel. While a loop holds the
reference, `action.vyaw` becomes the rate that closes the heading error in TAU seconds (1.0), damped by the
sensed yaw rate (0.3), in the walker's own units; it is mixed with the twist brain's own yaw by proximity
(nothing within a metre: the reflex owns the yaw; a wall at hand: the brain's avoidance owns it). Gated by
the state it exploits. On R60a, n = 6: |vyaw| on the rail 96 → 64 %, cells 89 → 129 (5 / 6 up), span 9.2 →
12.7 m², straightness 0.15 → 0.21, walls tie (20.4 → 21.0, sd 21 → 6), stands hold. Under seek the error
closes: median 1.36 → 0.43 rad (under 0.3 rad on 40 % of ticks against 11 %). Under play it does not (1.56
→ 1.80), because the reference still turns at 0.6 rad/s. `WORKING` on the channel.

**Lever 2, the target inside the turning radius: two built answers retried on the reflex.** R38's
`lookahead` (its re-use context was a regulator that regulates): the reference still moves at 0.60 rad/s
under play, walls 64 / min, `REGRESSION` again. The play loop's run-and-tumble wander forced on walks
(`wander_stall_ticks` 100, `explore_cycle` 250, R62, the operator's random walk): the reference moves at
1.50 rad/s, faster than before, `REGRESSION` — and the trace of why is the finding of the day.

**The finding: the play loop's bearing has been mirrored since R27.** On an R62 walk the reference runs
away from the heading at about twice the body's turn rate: `ref = 2·heading − target`. The play loop is
the Cell's, and its frame has forward(h) = (−sin h, −cos h): a positive heading step is a clockwise turn.
The duck's heading is a right-handed odometry yaw, counter-clockwise positive. Seen from the duck, the
loop's frame is a reflection (x, y ↦ −y, −x), and a reflection reverses the turn sense: the loop's "turn
right" (fx > 0) is the duck's left. The adapter turns the body clockwise for fx > 0 (R27, the documented
contract), the body's response reads in the loop's frame as a turn the other way, the bearing error grows,
and the loop keeps saying "right": a perpetual circle. The seek loop (§17.35) was written in the duck's
frame, which is why its reference is quiet and the reflex closes on it.

Fix: `PlayLoop.heading_sign` (default +1, byte-identical; a unit test pins the frames: a body walking
forward at +π/2 goes to −x in the Cell's frame and to +x with −1). −1 multiplies the incoming heading,
which turns the reflection into a rotation, so the loop's position integration and its bearing are both
in the duck's handedness. R63 = R60 + `heading_sign −1`.

**What the un-mirrored play loop does, n = 6:**

| | R60a | + reflex | R63 (sign) | R63 + reflex |
|---|---|---|---|---|
| the reference's motion under play, median · > 0.5 rad/s | 0.70 · 56 % | 0.60 · 53 % | **0.00 · 16 %** | **0.00 · 16 %** |
| refFollow | 0.30 | 0.44 | **0.11** | **0.16** |
| heading error under play · under seek, median | 1.56 · 1.36 | 1.80 · 0.43 | 1.34 · 1.46 | **1.12 · 0.26** |
| \|vyaw\| > 0.9, play · seek | 93 · 90 % | 50 · 10 % | 96 · 94 % | **21 · 4 %** |
| straightness per walk p50 | 0.13 | 0.22 | 0.14 | 0.23 |
| cells · span m² | 89 · 9.2 | 129 · 12.7 | 78 · 9.8 | 106 · 12.5 |
| walls / min · contact % | 20 · 2.3 | 21 · 3.4 | **47 · 14** | **49 · 12** |
| object contacts / min · moved m | 3.2 · 2.5 | 8.9 · 2.9 | 43 · 2.9 | 5.1 · 2.1 |
| stands held · arrival stops | 92/92 · 5.3 | 94/95 · 6.0 | 85/85 · 4.0 | 85/89 · 5.0 |

The premise holds at once: with the sign right the reference under play stands still (refFollow 0.11, the
lowest measured on this body), and with the reflex the seek error closes to 0.26 rad and the yaw command
leaves its rail on both loops. The cost is as loud: wall contacts double. Novelty in a known room is at its
edges, and the mirrored bearing had been an accidental avoider, turning the duck away from what it aimed
at — which is what "play alone is the best avoider in a known room" (§17.7) was measuring. Nothing above
the twist brain's proximity prior now stops the body at a wall, and that prior alone was never enough
(R48's model-implied step was the loud avoider, 14.8 → 1.8 / min, and is not in the `★ CLOUD` stack).

**R64 = R63 + R48's model-implied step** (`state_prior_step_gain 1.0`), with and without the reflex, and with the
reflex releasing at half a metre instead of one (`--heading-reflex 1.0 0.3 0.5`):

| n = 6 | R60a | **R64, no reflex** | R64 + reflex (gate 1.0) | R64 + reflex (gate 0.5) |
|---|---|---|---|---|
| the reference's motion under play, median · > 0.5 rad/s | 0.70 · 56 % | **0.00 · 15 %** | 0.00 · 13 % | 0.00 · 18 % |
| heading error under play · seek, median | 1.56 · 1.36 | 1.05 · 1.61 | **0.89 · 0.91** | 1.13 · 0.61 |
| \|vyaw\| > 0.9 (all walking ticks) | 96 % | 93 % | **47 %** | 69 % |
| straightness (harness) · per walk p50 | 0.15 · 0.13 | 0.22 · 0.15 | **0.29 · 0.24** | 0.29 · 0.21 |
| cells · span m² · path m | 89 · 9.2 · 81 | 126 · 12.4 · **96** | 139 · **14.2** · 90 | **144** · 13.0 · 86 |
| walls / min (per seed vs R60a) · contact % | 20.4 ± 20.7 · 2.3 | **14.7 ± 7.1** (3 / 6 down; none above 24) · **1.4** | 43.1 ± 10.3 · 6.9 | 32.1 ± 20.7 · 4.3 |
| object contacts / min · objects moved m | 3.2 · 2.5 | 11.3 · **5.1** | 10.7 · 3.7 | 6.2 · 4.3 |
| seek episodes: blocks within 0.4 m · touched | 7 / 11 · 3 (R60) | — | **7 / 8 · 6**; balls 5 / 10 · 4 | — |
| rescues / min · walk % · arrival stops | 0.04 · 54 · 5.3 | 0.05 · 65 · 3.8 | 0.07 · 68 · 4.2 | 0.19 · 59 · 5.2 |

Without the reflex, R64 is the cleanest walk this body has had: the reference stands still, coverage rises
on five seeds of six, the path on four, objects moved double, and wall contacts fall from 20 to 15 a minute
with the seed spread gone (R60a ran 0–56, R64 3–24) — while the yaw command is still on its rail 93 % of
the time and the error still sits near a radian. The body goes where the reference points, in wide arcs.
With the reflex the walks are straighter (0.29) and the error closes (0.89 rad under play, 0.26 under seek
on R63), and the seek episodes are the strongest measured (blocks reached within 0.4 m on 7 of 8, 6 into
contact), but the walls come back (43 a minute; 32 with the gate at half a metre, at the cost of rescues
0.19 a minute). Half a second before 78 % of R64+reflex's wall contacts the ToF had the wall within 0.7 m
and the brain already owned most of the yaw; the error to the reference was a radian: the reference itself
lies at or beyond the wall (a novel node at the room's edge, a wall base attended as a thing), and a
heading held to it wins against the avoidance in the last half metre.

**Verdicts, and what they re-open.** `heading_sign −1` with R48's step (R64) `WORKING`: the reference stands
still and the walk goes where it points, with coverage up and walls down against R60a; **preset R64 (seed 1)
is the candidate for the operator's eye**, and R64r (the reflex at half a metre) the straighter version to
judge beside it. The heading reflex `WORKING` on the channel (the first time a loop's bearing has been
followed on this body: the error closes, the yaw leaves its rail) and `PARTIAL` as a behaviour in this
stack, since a heading held to a reference at a wall costs contacts; re-use context: a reference that the
cloud has checked for free space, or a seek target whose vocabulary node is a thing's and not a wall base's.
`heading_sign −1` alone (R63) is a `REGRESSION` on walls, and a confound on the record: every verdict in which the play loop set
the reference — R27–R29 (§17.6–17.7), the playroom's R27 (§17.8), R34–R38 (§17.16–17.17), R47–R48
(§17.26–17.27) — measured a mirrored bearing. Their behavioural verdicts are `ABLATED` by this finding
(the mechanism operated, on the wrong sign); their measurement lessons stand (the reference's motion, the
rail, refFollow, the instruments). `lookahead` and `commit_hold` were refuted on the mirror and are open
again. §3.2's rule 7: the arm that ran was not the arm that was thought to run.

### 17.38 The body's errors as triggers, the watched walk, and the run's record (2026-09-17, after the operator's eye on R64)

**The operator's eye on R64** (seed 1, the launcher): "the directional fix is a big improvement; the robot is
traversing much faster." Three events: at ~880 s it runs into a table leg; at ~945 and ~1200 s it trips over a
block. "Those must be triggering big TLE spikes; great opportunities for the robot to stop and look, or
trigger the pre-built skills — a loop that uses the emotes and skills as an output to reduce error on its
body or the environment." Agreed, with the boundary settled: a skill fires by NAME at the intent boundary,
the simulator stands in for the daemon, and what the brain learns is what each intent does.

**What the events are in the log.** The watched run's record has them: 18 s pushing against the table leg
with the forward command at full and the body not moving (`wall` flag on 146 ticks: the playroom's
furniture counts as a wall), and two falls over blocks (tilt 120° and 131°, `obj` on 78 and 96 ticks; the
rescue stood it up). The harness's seed-1 run does not have them, because the watched run was a different
arm (below).

**Do the brain's error channels see them?** R64 with `--log-motor-tle --body-predicts`, n = 6, each channel
scored against its own walking distribution (median and MAD):

| event (on walks, from 700 s) | count | twist brain `mtle` > 3 robust sd | joint brain `btle` | forward residual |
|---|---|---|---|---|
| a fall (tilt > 60°) | 30 | **100 %** | **100 %** | 57 % |
| a contact onset (wall, furniture, object) | 2 327 | 14 % | 17 % | 15 % |
| a push of 2 s (commanded forward, not moving) | 11 | 9 % | 18 % | 36 % |

Falls are loud in every channel (and in tilt). Contacts mostly are not: a leaning or glancing contact
changes nothing the body predicts. A push is not a spike at all — the walker achieves about half of its
commanded speed at the best of times (the 2 s mean forward residual is 0.51 ± 0.27 while walking and 0.63
during a push), so no level separates it. What separates it is DURATION: walking stalls (commanded forward
above 0.75 of range, sensed under 0.25) last 0.22 s at the median and 1.2 s at the 99th percentile, while
every stall of 2 s or more in the six runs (11 of them, 2–8 s) was a push against a wall or furniture.

**Lever: the stuck stop** (`--stop-on-stuck K`, off by default; R46 byte-identical): a stop starts when a
stall has lasted longer than K times the body's own running median stall length (K = 8, about 1.8 s), with
the same settle, sweep and cloud as any stop. The body's forward-model error as a duration against its own
scale. Measured on R64w (below), n = 6: 12 stuck stops over six runs, 8 of them at a wall or furniture, seed 5
(no stall past the threshold) byte-identical to its control; long pushes 8 → 5. Walk time 79 → 71 %, path
115 → 100 m, straightness 0.28 → 0.21, walls tie (22 → 23 / min, per seed both ways), rescues tie.
`WORKING` as a mechanism, `NULL` as a behaviour: the duck stops, looks, and resumes toward the same
reference, and pushes again. The stop has to change what the walk is for — drop the seek target it could not
reach, and let the stop's cloud bake the node play was climbing to — before it is a behaviour. Preset R64s.

**The run's record: three catches (§3.2 rule 7).** Chasing why the watched run and the harness's run of
the same seed diverged at 613.58 s:

1. **The host is deterministic.** Five headless replays of seed 1 are identical to each other and to the
   harness's run over all 75 000 ticks, with the inspector bound, blocked, or carrying a read-only client
   that subscribes every module's diag. Two replays of 700 s diverged from the harness at 680 s only because
   a stop is not started with under 60 s of run left; a short replay is not a replay.
2. **The watched run was a different arm.** `newtest.py` copies the controls of the first preset naming the
   base config; R60 has two (R60 and R60a, which differ only by `--stop-on-arrive`), so R61–R64 were minted
   without the arrival-stop flag every measured arm carried. The launcher ran R64 without arrival stops;
   the harness measured it with them. Fixed: `newtest.py` copies the LAST preset of the base config and
   prints the copied host args; presets R61–R64 and R64r corrected; the walk the operator watched is its
   own preset, R64w, measured below. The launcher's start-up scan of configs was the earlier half of the same
   trap (the R64 launch that "closed at once" ran `head2_h1_babble.json`); fixed the same day.
3. **A live parameter change under a watched run was not on its record.** `set_param` and `apply_patch`
   through the inspector now print into the JSONL as `patch:<module>.<key>` / `patch:graph`, so a watched run
   can always be compared with a measured one.

Also noted for O41: R64 with `--log-motor-tle --body-predicts` is not the same run as R64 without them
(stops 15.5 against 13.7, cells 159 against 126 at n = 6); the earlier "behaviourally free" reading needs a
byte-identity check before the channel is used as an input.

**The walk the operator watched (R64w = R64, timer stops only), n = 6, beside the measured R64:**

| | R60 | R64 (arrival stops, §17.37) | **R64w (watched)** |
|---|---|---|---|
| walls / min · contact % | 12.4 · 1.7 | 14.7 · 1.4 | 22.1 ± 18.5 · 2.6 |
| path m · cells · straightness | 124 · 124 · 0.17 | 96 · 126 · 0.22 | 115 · 140 ± 58 · 0.28 |
| walk % · stand % · stops | 82 · 16 · 10.8 | 65 · 33 · 13.7 | 79 · 19 · 11 |
| objects moved m · rescues / min | 3.4 · 0.13 | 5.1 · 0.05 | 2.6 · 0.07 |
| refFollow · heading error | 0.27 · 1.50 | 0.12 · 1.28 | 0.08 · 1.26 |

The walk the operator liked is the un-mirrored play with the step and no arrival stops: the fastest and
straightest cover of the room this body has made, at the cost of a third more wall contact than with the
arrival stops. Both are on the record; the choice is the operator's eye.

**The skill runner, opened.** Pollen's one-shot networks are the walker's architecture (61 observations, 14
actions; the files are 793 685 bytes to the walker's 793 705): `ball_kick_left.onnx`, `ball_kick_right.onnx`
and `roulade.onnx` fetched at the walker's pinned commit `3954496` (SHA-256 `d6928284…`, `147a32c3…`,
`3d60da08…`; the current release seeds them from the `microduck-policies` Hub repository instead). Their
daemon runs a kick as a 0.5 s window and a roulade as 1 s: the network sees an all-zero command, runs at
standing tuning, then unwinds and hands back to the gait, in the priority roulade > kick > ground pick >
sit > stand > walk. The build that follows: the host runs a requested skill exactly so, a loop requests it
by name through the bus (`intent.skill`), and the brain's side is a model of what the intent does — the
cloud before and after a kick, the gravity vector after a roll — whose error is the thing reduced.

### 17.39 Skills at the intent boundary: the kick, from the walk and from standing (2026-09-17)

**Built.** The host runs one of Pollen's one-shot networks as their daemon runs it (`robotd/src/control.rs`):
a window of the skill's duration (a kick 0.5 s, a roulade 1 s) in which the network sees an all-zero command
and drives every joint at standing tuning, then the gait resumes from where the body was left; the stander,
if it was standing, resumes from there too. Requested by NAME: from the graph through `intent.skill`
(a ProprioToken `[id, request]`, the adapter's `skill_request()`), or for the first measurement by a host
flag, `--skill-on-arrive NAME` (the seek loop's arrival) and `--skill-at SECS NAME` (a scripted check).
`kick` picks the side from the thing's bearing. The networks `ball_kick_left.onnx`, `ball_kick_right.onnx`
and `roulade.onnx` are fetched at the walker's pinned commit (`fetch_scaffolds.sh`, hashes in the scaffolds
README) and load with the walker's shape (61 observations, 14 actions). Off = byte-identical (md5 `cb24520c…`).
On the robot the same request is `robot.do{skill}`; nothing here is a trajectory of ours.

**Measured, n = 6 × 1500 s on R64 with arrival stops.** Per arrival, the nearest object's displacement over
the following seconds, and the body:

| | R64 (arrival stop, no kick) | kick fired MID-WALK at arrival, then the stop | **kick fired FROM STANDING, after the stop's hand-back** |
|---|---|---|---|
| arrivals · nearest object at | 23 · 0.22 m | 25 · 0.28 m | 27 · 0.21 m |
| the object moved > 5 cm within 3–4 s | 4 % (balls 8 %, blocks 0 %) | 16 % (balls 20 %, blocks 10 %) | **19 % (blocks 25 %, balls 9 %)** |
| max tilt in the window p90 · falls | 9° · 0 % | 101° · **28 %** | 6° · **0 %** |
| walls / min · rescues / min | 14.7 · 0.05 | 22.8 · 0.15 | 16.5 · 0.07 |
| objects moved (whole run) · walk % | 5.1 m · 65 | 3.0 m · 72 | 3.8 m · 75 |

A kick fired into a walk at full command topples the body a quarter of the time: their daemon runs the
kick at standing tuning, and the window's zero command from a walking state is a stumble, not a kick.
Fired from standing (the arrival stop settles, the stander takes the legs, the kick runs, the stander
resumes) it is safe on every kick, and the thing answers one time in five, blocks more than balls. The
remaining four in five are the foot missing: the thing sits at 0.21 m at a bearing the network does not
read, and a ball that is touched rolls out of the window's reach. Which foot, and how far the thing is from
it, is what a loop that learns the kick's outcome would have to learn.

**Verdicts.** The skill runner `WORKING` as the intent boundary's stand-in; the standing kick `WORKING` as
a mechanism (safe, and it answers); the walking kick `REGRESSION` (falls). Preset R64k (the standing kick
at arrival). What it hands the plan: the outcome loop needs to SEE the outcome, and a thing at 0.21 m sits
below a level gaze (§17.36, 12 of 31 arrival stops attended nothing); so T3, the gaze at the reached thing,
comes before the loop that learns what a kick does.

### 17.40 The gaze at the reached thing, and the loop that learns what a kick does (R65–R66, 2026-09-17)

**T3, the gaze at the reached thing** (`--stop-gaze-at-thing`, off by default; R46 byte-identical). At an
arrival stop the sweep's pitch band is centred on the reached thing's elevation (atan2 of the sensor's
height over its range, clamped to 0.2–0.55 rad down, ±0.12) and its yaw on the thing's bearing. n = 6 on R64
with arrival stops: arrival stops whose cloud attended a real object within 0.8 m, 5 of 28 against 1 of 23;
"nothing attended" 13 of 28 against 8 of 23; rescues 0.05 → 0.20 a minute, walk 65 → 57 %. A thing at 0.2 m
lies 45° under the beak, past the 31° the band reaches and past the 23° the stand was shown to take (O36),
so the head goes down, the stand pays, and the thing is still at the field's edge. `PARTIAL` on what it is
for and a `REGRESSION` on the stand; not adopted. The outcome loop below observes from a step back instead.

**The outcome loop** (`ogma::SkillOutcomeLoop`, new, generic; four unit tests): the operator's framing —
our brain learning what the robot's pre-built intents do is an error we can reduce — as one loop, for the
kick, on one question: does the thing answer? While the attended thing's bearing is live the loop fixes its
position in the odometry frame; at the seek loop's arrival it requests a kick by name on `intent.skill`
(the side by the thing's bearing) when the thing's node has an uncertain answer — fewer than two recorded
outcomes, or a spread above the mean spread over nodes; the next live bearing within 0.6 m of the fixed
position is the same thing, moved, and its distance is the outcome, folded into the node's running mean and
variance; a thing not seen again within the window is UNKNOWN, and nothing is learned from it. The surprise,
|observed − predicted| in the node's spread, goes out on `reality.cognitive.outcome`. The host fires a bus
request from standing at the next hand-back (never into a walk), and logs each observed outcome as `outc`.
R65 = R64 + the loop, run with arrival stops and no host flag: the graph asks.

| n = 6 × 1500 s | R64 (no kick) | **R65: the loop asks** | R66: a 100 s window | **R65 + unwind (back off 1.5 s, then look)** |
|---|---|---|---|---|
| kicks requested · fired · falls | — | 26 · 23 · 0 | 13 · 13 · 0 | **27 · 27 · 0** |
| outcomes observed · unknown · nodes known | — | 5 · 19 · 0 | 0 · 11 · 0 | **12 · 15 · 1** |
| the observed answer, m (p10–p90) | — | — | — | 0.03–0.10 |
| walls / min · rescues / min | 14.7 · 0.05 | 15.2 · 0.07 | — | 12.0 · 0.11 |
| stops · arrival + look stops · walk % | 14.3 · 3.8 · 65 | 14.7 · 4.5 · 74 | — | 18.8 · 9.3 · 61 |
| objects moved, m · cells | 5.1 · 126 | 3.6 · 141 | — | 2.9 · 138 |

**Reading it.** The boundary works from the graph's side: the loop asks, the host runs Pollen's network from
standing, nothing falls. What starves the learning is SEEING the answer: a kicked thing sits under the beak,
and 19 of 24 outcomes were unknown; a longer window (R66) made it worse, since a pending outcome blocks the
next request and the thing was never re-found within the radius from a later, farther stop. The daemon's own
`unwind` is the answer to that (`--skill-unwind VX SECS`): after the kick the stop ends, the body backs off
for 1.5 s, and a fresh stop looks at the thing from half a metre. Observed outcomes 5 → 12 of 27, one node
reached two samples, walls fall to 12 a minute, at the cost of the extra stops (walk 61 %) and a few more
rescues from backing. The answers themselves are 3–10 cm: a kick that connects moves a ball or a block by a
few centimetres, and the fixed position's own error (odometry, the cluster's centroid) is of that order, so
the per-node statistics need several samples before "answers" and "does not" separate. At one kick every
three minutes they do not, in fifteen. The mechanism is whole; its loudness waits on more arrivals.

**Verdicts.** T3's gaze `PARTIAL` / `REGRESSION` on the stand, not adopted. The outcome loop `WORKING` as a
mechanism (the intent requested by name, the answer learned per node when seen), `PARTIAL` as a behaviour
(too few answers to habituate on in a run); the unwind `WORKING` for what it is for (observed outcomes ×2.4)
and is the form to carry. R66 `NULL`. Presets R65 and R65u. Next: more arrivals per run (the seek loop's
share, and the timer stop's floor), and the outcome's precision (the thing re-fixed from the look stop's
cloud rather than the arrival's), before the habituation can be read; then the get-up (the roulade after a
fall, O27) on the same runner.

### 17.41 The peck: a second intent, and the choice between them (R67, 2026-09-18)

**The operator's eye on R65u:** "the kick and look cycle works." And two asks: the reach-down pick as a
second way to explore ("very similar to a kick, like a peck; I don't expect anything to be picked up"),
and a creative way to choose when to kick and when to peck.

**Built.** Pollen's ground pick is not a window but a PHASE their daemon drives: the network sees
`[cos 2πφ, sin 2πφ, 0]` in the twist slots while φ runs from 0 to 0.7 over a 4 s period, so a 2.8 s
reach-down at standing tuning (`robotd/src/control.rs`, `DEFAULT_GROUND_PICK_END_PHASE`). The host runs it
so, as the skill `peck` (`alpha_ground_pick.onnx`, from the `microduck-policies` Hub set at `v1`, the set a
fresh board is seeded with; `fetch_scaffolds.sh` fetches it by hash). Fired alone from seed 1's walk: the
body drops from 12 to 8.5 cm, the knees and ankles fold, it leans to 24° and stands again at the end.
Nothing is grasped (no MJCF has the mouth hinge), and nothing was meant to be.

**The choice.** The outcome loop keeps its per-thing statistics per INTENT, and at an arrival asks for the
one whose answer for this thing it knows least: fewer recorded outcomes first, then the larger spread, then
the one it did not try last (`peck_id`; −1 keeps R65 byte-identical; a unit test pins the alternation). A
creature that has kicked a block twice and never pecked it pecks; one that knows both leaves it alone. The
outcome measure is the same for both, the thing's displacement, which is what "does it answer" means here.

| n = 6 × 1500 s, arrival stops, the unwind | R65u (kick only) | **R67 (kick and peck)** |
|---|---|---|
| requests · windows fired (kick / peck) · falls | 27 · 27 (27 / —) · 0 | 31 · 29 (16 / 13) · **0** |
| outcomes observed · unknown | 12 · 15 | 13 (kick 9, peck 4) · 18 |
| the observed answer, m (p10–p90): kick · peck | 0.03–0.10 · — | 0.06–0.17 · 0.05–0.22 |
| walls / min · rescues / min · walk % | 12.0 · 0.11 · 61 | 17.9 · 0.11 · 66 |
| stops · stands held · objects moved m | 18.8 · 18.5 · 2.9 | 17.8 · 15.7 · 3.2 |

**Reading it.** The choice does what it says: the loop alternates, the peck runs from standing without a
fall, and a peck moves a thing about as often and as far as a kick does, which is the peck being a
forward lean onto the thing at 0.2 m. The learning is where R65u left it: too few answers per run for a node
to reach two samples of either intent (none did), so the habituation cannot yet be read, and the answers'
scale (5–20 cm) is within the fixed position's own error. Walls rise from 12 to 18 a minute, within the
spread. `WORKING` as a mechanism; the behaviour's loudness still waits on more arrivals per run. Preset R67.

**The get-up question** (the operator: how would a roulade get-up differ from the current one). The current
get-up is the host's `Recovery`: when projected gravity says the body is down (past 60°, held 200 ms, the
daemon's own late detector), the host hands the joints to the standing scaffold until the body is upright,
then hands them back; the brain observes it frozen and never decides it. A roulade get-up would change who
decides and what is fired: the brain, from its own gravity error, would request an intent by name through
the boundary, and the intent's outcome (gravity back to upright, or not) would be learned like the kick's.
Whether Pollen's roulade rises from an arbitrary fallen pose is unmeasured — it is a forward roll trained
from standing that ends on the floor and rises — and their walker already carries fall recovery in one
network; so the roll may be the wrong intent for a fall and the right one for a trick. The measurement is
cheap once wanted: fire `roulade` when the body is down instead of the scaffold, and count the rises.

### 17.42 The orbit's second cause (a reference that flips by 2π), the ToF on the walk, and the roll as a get-up (R68–R70, 2026-09-19)

**The operator's eye on R67** (seed 1): "around 1060 s the robot started circling again and ignoring small
objects"; and: "are we using any ToF data while the robot is walking?"; and: proceed with the roulade
experiments.

**What the circle was, this time.** R67 carries the un-mirrored play, so the mirror is not it. From the
run's record: from 1041 s play holds the reference and the reference turns at the body's rate (−15 rad per
20 s); the seek loop holds a target the whole time, at 2–3.4 m, and its need decays from 0.86 to 0.18 while
its range grows, since the body circles away from it. R38's lookahead and R37's committed sub-goal, both
built for the orbit and both refuted on the mirror, were retried on R67 (R68, R69): neither quiets the
reference (turning faster than 0.5 rad/s on 23 % and 29 % of play's ticks against 19 %), coverage falls
(142 → 123 and 116 cells), `NULL`. So the orbit is not the target's distance.

The record gained play's bearing and state per tick (`pb`, `pl`) and seed 1 was replayed. Two things
appear. Under seek at 1030 s the seek target is BEHIND the body: the reference alternates by 2π from one
tick to the next (−8.78, −2.49, −8.77 …), the error between +3.0 and −3.1, and the yaw command flips
sign every few ticks; the body jitters in place and never turns round. The reference is rebuilt from the
winning loop's bearing every tick as heading − atan2(cx, cy), and a bearing that flickers across ±π flips
it by a full turn. Under play from 1041 s the bearing sweeps through the body frame once every seven
seconds while the body turns a circle every eleven — the orbit — and at each pass behind the same flip
reverses the turn, which is what keeps the orbit alive. This is the "reference that will not stand still"
of §17.26, its jumps of more than 0.5 rad ninety times a minute; it survived the mirror's fix because it
is a second defect on the same line.

**Lever: a continuous reference** (`--ref-unwrap`, off by default; the physics byte-identical with it off,
md5 `cb24520c…` once the two new record fields are stripped). Each new reference is taken modulo 2π
nearest the reference held so far, so the turn direction persists through the back; the sense slot and
the heading reflex clamp the error at ±π instead of re-wrapping it. Measured: it does what it says and it makes the circle worse. Seed 1 replayed, 1000–1200 s: reference jumps
of more than 3 rad 60 → 2, but the heading turns −79 rad against −51, the body travels 22.6 m of path for
0.4 m of net displacement, and n = 6 gives walls 18 → 6 a minute with coverage 142 → 60 cells and the yaw
on its rail 94 % of the time: a duck spinning in place. So the flip was not the orbit's cause but its
brake: play's bearing genuinely rotates with the body, and with nothing reversing the turn the spin never
ends. `REGRESSION`, off. What rotates play's bearing with the body is in the loop's own geometry — its
position is integrated in command-unit ticks from a lateral and a forward velocity with different scales,
and its target is a node's mean position in that frame — and that is the open question (O56), to be
answered by logging the loop's own odometry beside the body's before any further lever.

**The ToF on the walk, as it stood.** Every 4 ticks the ToF's four proximity slots feed the twist brain's
sense (the avoidance prior) and the place map's view holds the last stop's cloud; the cloud, and with it
the things and the seek bearing, existed only at stops. Between stops the seek loop homed to a remembered
position. The cast is taken in the gravity-levelled trunk frame with the head's pose folded in by forward
kinematics, so head motion is not the obstacle in simulation (on the robot each frame must be timestamped
against its head angle, §7 of the cloud phase). What was missing was translation.

**Lever: the walking cloud** (`CloudMap.walk_cloud`, off by default, two unit tests): between stops a
cloud stays open, each cast translated by the odometry's displacement from the anchor (the cast token
already carries x and y) and de-rotated as at a stop, filed and re-anchored every metre of travel so the
odometry's drift stays under a voxel; never cached as a place; the things reduction runs on it. R70 = R67
+ the walking cloud, n = 6: things attended on walking ticks 18 601 against 178 — the sensor works. The
behaviour above it does not, yet: with a live bearing on every walk the seek loop holds the reference
80 % of the time (28 % before), walls 18 → 55 a minute, the stands 4 of 17 stops, coverage 142 → 178
cells. The loops were built for a thing seen at a stop and remembered on the walk; fed a live bearing
they chase, and a fifth of what they chase is a wall base. `WORKING` as a sensor, `REGRESSION` as a
behaviour in this stack; re-use context: seek's need gated by the thing's vocabulary node and by the
free space ahead, and the arrival stop reading the walking cloud.

**The roll as a get-up** (`--skill-when-down NAME`): when the recovery declares the body down, the named
skill drives the joints first, and the scaffold's rescue continues if the body is not upright when the
window ends. R67 + roulade, n = 6: 11 falls, the roulade fired at each, the body upright within 2 s of the
window on **none**; walls 18 → 28 ± 38 (one seed's flailing). `NULL` as a get-up, as the priority table of
their daemon suggested: the roll is a trick from standing, and their walker already carries the fall
recovery. The difference a brain-requested get-up would make — who decides and a learnable outcome —
stands, but the intent for it is not the roll.

### 17.43 The orbit, resolved to its parts: the yaw column turns the wrong way, and the loop's committed turn holds it there (2026-09-19, later)

**Measured before any lever.** On R67 seed 1 the map's nodes are views, smeared over 0.5–1 m of position
(median spread 0.53 m over nodes with more than 50 ticks), and the play target's centroid sits 0.8–1.2 m from
the body through the circle — outside the turning radius, so the orbit is not a near target (the premise
R37/R38 were built on). With the sign fix in place the loop's arithmetic makes the reference the target's
own direction, a constant for a fixed target; a reference that turns with the body means the body is not
doing what the loop asks. The record answers: play asks a turn of more than 0.6 rad on 57 % of its ticks, and
the body turns that way over the next second on **41 %** of them — worse than chance. The twist brain's
learned yaw column has the wrong sign under play's error (§17.17's "does not regulate", now with a number),
and the loop's own committed turn does the rest: past 90° off it holds the turn's sign and clamps the
bearing at 0.92π until the target is within 36°; a body turning the wrong way never gets there, so the clamp
holds the wrong command indefinitely (the emitted bearing sits at exactly −2.89 through the circle).

**The reflex, on the stack as it stands.** R64r's heading reflex (a hold on own yaw through `action.vyaw`,
mixed with the brain's yaw by proximity) was set aside for its wall cost on R64. On R67 seed 1: agreement
41 → 62 %, circling windows 4 → 1 of 25, wall ticks 460 → 293 with the one-metre gate; the half-metre gate
49 %, walls 1 567 — near a wall the brain's yaw owns the channel and turns wrong there too. 

| n = 6 × 1500 s, the R67 stack | R67 | **R67 + reflex (gate 1.0)** | R67 + reflex + free-space gate |
|---|---|---|---|
| the body turns the way play asks (> 0.6 rad asked) | 41 % | **64 %** | 61 % |
| heading error, median · under play · under seek | 1.44 · 1.27 · 1.50 | **0.99 · 0.81 · 0.61** | 0.87 · 0.74 · 0.42 |
| the reference turning > 0.5 rad/s under play · \|vyaw\| on the rail | 19 % · 83 % | 11 % · **22 %** | 19 % · 15 % |
| straightness (harness · per walk p50) | 0.25 · 0.17 | 0.27 · 0.23 | 0.25 · 0.23 |
| walls / min (per seed) | 17.9 ± 6.4 (12, 20, 20, 28, 12, 16) | 25.8 ± 20.9 (**6, 16, 12, 17**, 45, 58) | 34.7 ± 28.8 (14, 6, 9, 58, 74, 47) |
| cells · rescues / min · stands | 142 · 0.11 · 15.7 / 17.8 | 137 · **0.04** · 15.5 / 17.8 | 127 · 0.10 · 15.2 / 17.7 |
| kicks requested · outcomes observed | 31 · 13 | 32 · 13 | 32 · 10 |

The reflex does what the diagnosis says: the body follows the loops, the error closes by a third, the yaw
leaves its rail, and rescues fall by two thirds. Walls fall on four seeds and blow up on two, the failure of
R64r again: a reference held into a wall, and a reflex that holds it there while the brain's avoidance, with
its share of the yaw shrinking as the wall nears, cannot turn the body away. The lever for that is the one
named as the reflex's re-use context in §17.37: **a reference the sensor has checked for free space**
(`--ref-free P`): a loop's bearing into a ToF sector nearer than P is not held; the reference is released to
the heading (R29's release form), the reflex stands down, and the avoidance acts unopposed.

Measured (proximity above 0.6, a hit within 0.4 m of the bearing's sector; the reference released on 4 408
ticks in one run): the errors close further, and the walls do not. Released, the yaw belongs to the twist
brain, whose avoidance is what fails at those surfaces. The bad seeds are not many collisions but a few long
bursts: on the reflex arm seeds 4–6 spend 108, 217 and 318 s of the 800 in 10–19 bursts, the longest 34–80 s;
with the gate 289, 374 and 180 s, the longest 120 s. That is O35's "cannot leave a surface", now with a
reference held into it. The stuck stop of §17.38 was built for that and was `NULL` because the stop changed
nothing about the reference; a stuck stop that releases the reference and turns toward the free space the
cloud shows is the lever this hands the next session.

**Verdicts.** The orbit is resolved to its parts: the mirrored bearing (§17.37, fixed), the twist brain's
yaw column turning the wrong way for a large error (measured: 41 % agreement), and the play loop's
committed-turn clamp holding the wrong command (7 134 clamped ticks a run). The heading reflex on the R67
stack `WORKING` on the channel (agreement 64 %, error 1.44 → 0.99, the rail 83 → 22 %, rescues 0.11 → 0.04)
and `PARTIAL` as a behaviour (walls down on four seeds, two seeds stuck at surfaces). The free-space gate
`NULL` (the errors close, the walls do not). The continuous reference `REGRESSION` (§17.42). Preset R67r (the
reflex on R67) for the operator's eye beside R67; nothing promoted.

### 17.44 The escape: a stuck stop that turns toward free space (2026-09-19, last)

**Built** (`--stuck-escape SECS`, off; guard byte-identical with the record fields stripped): when a stuck stop
ends, the host takes the stop's cloud view (eight sectors across ±64°, each the nearest off-floor return over
4 m) and holds the reference at the freest sector's bearing for SECS, the loops' bearings ignored meanwhile
(steer code 4), so the reflex turns the body out of the surface before play or seek can aim it back in.
`cloud_objects.py heading` and the sweep count the escapes.

**Measured, n = 6, on R67 + the reflex + `--stop-on-stuck 8 --stuck-escape 4`:** 8 stuck stops over six runs,
8 escapes. Where it fired it worked: seeds 5 and 6, the reflex arm's two stuck seeds, fall from 216 and 318 s
in wall bursts to 85 and 78 (walls 45 → 15 and 58 → 16 a minute). Seed 1 diverged into a 111 s burst the
detector never saw (walls 6 → 90); seconds in bursts over the six seeds 802 → 717, a tie. The stall detector
reads a forward speed under 0.1 m/s, and most bursts are the body SLIDING along a surface at walking speed,
commanded forward, making no progress toward its reference. `PARTIAL`: the escape is the right act and the
detector misses most of what it is for. Next form of the detector: progress toward the reference — the
body's velocity along the reference's direction, against the body's own distribution while walking — which
would see a slide as it sees a push.

**Where the walk stands at the end of 2026-09-19.** The circle the operator watched is understood in full
and its parts are on the record with numbers: the mirrored bearing (fixed), the yaw column turning the wrong
way for a large error (the reflex fixes it: 41 → 64 % agreement, the rail 83 → 22 %), the loop's committed
turn holding the wrong command, and a reference that flips at ±π (its brake, left alone). What remains is
O35 in a new form: a reflex that follows its loops faithfully into a surface on some seeds, and a detector
that does not yet see a slide. R67 and R67r are the arms for the operator's eye; nothing is promoted.

**Addendum, the progress-based stall** (`--stuck-progress`, off): the stall is then no progress toward the
reference (the body's velocity along the reference's direction under 0.25 of range while commanded forward),
so a slide along a surface counts. n = 6 on R67 + reflex + stuck 8 + escape 4: 26 stuck stops over six runs
(8 before), and walls 28 → 37 a minute, seconds in bursts 717 → 934, the longest bursts shorter (33–80 s
against 47–111) and more numerous. The escape works each time and the loops aim the body back at the same
surface four seconds later: the surfaces are where play's novel nodes and seek's wall-base targets lie, and
an exit reflex cannot change what the targets want. `REGRESSION` in this form; the question it leaves is the
loops', not the reflex's: a play value field in which a node at a wall stops being novel once the body has
stood at it, and a seek need that does not hold a wall base (its vocabulary node knows the difference at
purity 0.9). Both are the next levers on the walk, and both are the operator's call on design before a build.

### 17.45 The operator's eye on R67r, R69 and R70: the interesting scale, the walk promoted, and the cloud that was drawn at one pose (2026-09-22)

**What the operator saw.** R67r (the heading reflex): "the robot seems to be spending a lot of time staring at
the walls"; earlier configs interacted with the small objects more, so a regression on the *interesting* scale
if the numbers agree; the circling is gone. R69: good early interactions with the balls, some circling in a
corner, wandering, then repeated interactions with the purple block from 1060 s. R70: the wall voxels in the
viewer "rotated 45 degrees relative to the real walls". The rule they gave for the phase from here: any time
the robot interacts with an object is interesting; wandering and looking get boring quickly; a wider
vocabulary of behaviours is encouraged; the circling is mostly solved, so promote that and move on to object
interactions and lingering in areas of interest with play that involves the whole body, not only reducing
the map's error.

**The instrument the eye asked for** (`cloud_objects.py where`): every stop by WHERE the body stands when it
starts, at a small thing (within 0.6 m of a movable object's edge), at a wall (within 0.35 m of one), or on
open floor, and the seconds of stop spent in each. Per seed, n = 6, 1500 s:

| arm | stops at a thing | at a wall | open floor | stop-seconds at things / walls | thing/wall stops per seed |
|---|---|---|---|---|---|
| R65u (kick only) | 12.2 | 2.2 | 4.3 | 217 / 23 | 16/1 8/3 3/3 17/4 15/1 14/1 |
| R67 (kick + peck) | 9.5 | 3.5 | 4.8 | 133 / 46 | 12/3 8/3 3/3 7/6 17/2 10/4 |
| R67r (+ reflex) | 9.2 | 4.0 | 4.7 | 193 / 52 | **4/8** 12/2 22/1 11/0 **1/4** **5/9** |
| R69 (commit_hold) | 9.3 | 3.8 | 4.7 | 125 / 79 | 16/3 5/2 1/8 7/7 15/1 12/2 |

The means agree with nothing; the per-seed column agrees with the eye exactly. The reflex makes the walk
*bimodal*: three seeds stand at things (22/1, 12/2, 11/0) and three at walls (4/8, 5/9, 1/4), and seed 1, the
preset's seed, is a wall seed. The reflex follows its loops faithfully, and the loops' targets are, on half
the seeds, play's novel nodes at walls (§17.44). The sweep's own columns say the same in aggregate: walls
17.9 → 25.8 a minute (± 21), contact 1.9 → 4.6 %, objects moved 0.34 → 0.05 m, seek's share 28 → 22 %; the
heading error 1.44 → 0.99 rad and the same 29 skills fired in each of the three arms. **The reflex:
`PARTIAL`; not promoted.** Re-use context: once the loops' targets are things (the linger below), a reflex
that follows them is what the eye wants.

The peck cost something too: R65u stood at things 12.2 times a run for 217 s, R67 9.5 times for 133 s. A
peck is 2.8 s of skill and the same unwind and look as a kick, so it is not the time; it is the second
intent's second arrival at a thing that the walk does not make (§17.41: 31 requests a run against R65u's 27,
but 13 pecks spread over 6 runs). Within the spread at n = 6, and the linger is the lever aimed at it.

**Promoted: the walk with the mirror fixed** (`PlayLoop.heading_sign −1`, in every config since R63, with
R48's step, the seek loop, the arrival stop, the skill runner from standing, the outcome loop with two
intents and the unwind) — the R67 stack, as **`★ THINGS`**, the operator's call on the circling ("mostly
solved"). What is NOT in it: the reflex, the reference unwrap, the free reference, the stuck stops, the escape,
the progress stall, the walking cloud, the roll when down — all flags, all off. R69's corner circling is the
residue §17.43 named (the yaw column, play's committed turn), unchanged by commit_hold (`NULL`, §17.42).

**The rotated walls (R70) were the viewer's, not the odometry's.** The replay payload carries the world pose
the cloud was anchored on, latched by the host on the module's *open edge* (cloud closed → open). With
`walk_cloud` a cloud files and the next opens on the same tick, so there is no edge: the R70 record has 48
clouds filed under ONE anchor, and the viewer drew every cloud after the first at the first one's pose, turned
by whatever the body had turned since. The odometry itself was measured against the ground truth on the same
logs: over 10 s windows of walking its displacement is 1.3° off in direction (IQR −3.4..+0.3°) at 0.92–0.94
of the true length, its heading error is zero modulo a turn, and its position carries a fixed offset of about
1 m acquired before the first stop (the babble). The fix latches on the file-and-reopen tick as well and emits
the FILED cloud's anchor: R70 seed 1 rebuilt, 13 clouds, 13 anchors, each within 4 mm and 0.7° (max 7 mm,
3.0°) of the body's pose on its open tick. The R46 guard is byte-identical (`cb24520c`, the pb/pl fields
stripped): a config without the walking cloud never has a file-and-reopen tick. O55's sensor verdict stands;
the operator's 45° was §2's "instrumentation" drawn wrong.

**The next lever, written by the rewrite rule: the LINGER (R71).** The behaviour asked for is "stay at a thing
and do things to it; leave when it is boring." The error it minimises already exists: the outcome loop's
uncertainty about what each intent does to *this* thing (`SkillOutcomeLoop`, §17.40–17.41: per node × intent,
unknown under `min_samples` outcomes or a spread above the known mean). Today that error is consulted once,
at an arrival, and then the seek target is dropped and play's novel nodes carry the body to a wall. The
lever hands it to the loop that owns the body's target:

- `SkillOutcomeLoop.need_topic` publishes `[need, x, y]`: the share of intents whose answer for the last
  attended thing is still unknown (1 before any answer, 1/2 once the kick's is known, 0 once both are), 0
  while an outcome is in flight (the loop is looking, not asking), and the thing's fixed position.
- `BearingSeekLoop.renew_topic` reads it: after an arrival has dropped the target (its zero for one tick IS
  the arrival the outcome loop sees), a need above `renew_min` (0.25) at a position between 1.5 × `arrive_m`
  and `renew_range` (2.0 m) re-arms the target there with confidence = need. The duck that backed off 0.45 m
  after a kick and saw the thing again from its look stop walks back for the peck; the duck that knows both
  answers is renewed by nothing, and play takes it away. Habituation and lingering are one rule.

Both default off (empty topics): byte-identical. Tests: the need's ladder 1 → 0 → 1/2 → 0 (`test_skill_outcome_loop`,
6 tests) and the renewal's four guards (`test_bearing_seek_loop`, 5 tests). Config `a1v2_r71_linger.json` =
R67 + the two topics; preset R71 (seed 1). The prediction, at n = 6 against R67: more requests and more
observed answers per run, more nodes known, stops at things up and stop-seconds at walls down, cells and
walls within the spread; the failure mode to watch is a duck that shuttles between a thing and its look stop
without the outcome ever being observed (need stuck at 1 with `unknown` climbing).

**R71 measured (n = 6, 1500 s, against R67).** Requests 29 → 31, answers observed 13 → 11 (unknown 16 → 20),
renewals 20 over six runs, arrival stops 48 of 107 → 54 of 110; seek held the reference 63 → 81 % of the walk;
walls 17.9 → 29.0 a minute (± 19: one seed at 37), contact 1.9 → 3.3 %; stops at things 9.5 → 8.5 a run, at
walls 3.5 → 5.3, stop-seconds at walls 46 → 95; cells 142 → 139. Nodes known: 0 in every run, the need 1.0 at
every run's end — the ladder the test climbs (1 → ½ → 0) is never climbed on the duck, because a node × intent
needs two observed answers and the look stop sees the thing again about one time in three. **`NULL` as a
behaviour in this form, `REGRESSION` on walls on one seed.** The renewal re-arms what the outcome loop last
fixed, and on the seeds where that was a wall base (the attention's 18 % misses) it re-arms the wall.

**Why the answer is not seen — the diagnosis that sets the next lever.** Every skill in R67 and R71 (60) was
matched to the nearest movable thing at its start and that thing's true position 8 s later: the unknown
outcomes are NOT balls that rolled out of the 0.6 m match radius (0 of 36); in 33 of 36 the thing had not
moved at all, and the median true displacement after a kick or a peck is 0.00 m (the thing answers one time
in five, §17.39). The thing was simply not seen again. At the look stop after the unwind, the cases the loop
observed had the thing 0.33 m ahead (IQR 0.29–0.36) and 17° off the nose; the cases it did not had it at
0.75 m (IQR 0.24–2.33) and 55° off, ten of fourteen beyond the sweep's ±34°. Two sources: a skill fired
where no movable thing was within 0.5 m (5 of 29 in R67, 11 of 31 in R71 — wall bases and furniture legs the
cloud attends as things, and arrivals by dead reckoning at a place the thing is not), and a thing that is to
the SIDE after the skill (the kick's foot, the peck's crouch, a turn during the window) while the unwind backs
straight off and the look stop sweeps ±0.6 rad about the nose. Nothing in the loops can learn across an answer
they never see; the look is the bottleneck, and it is a gaze error on a remembered bearing — the operator's
own framing of the head (T1's correction).

**R71a: the unwind AIMED** (`--skill-unwind-aim GAIN`, off = byte-identical): the outcome loop's need topic
carries the thing's fixed position; the adapter turns it into a bearing in the body frame; during the unwind
the twist's yaw keeps the nose on it (−GAIN × bearing, clamped), and the look stop's sweep is centred on that
bearing in yaw (the pitch band stays — T3's pitch part cost the stand, §17.40). The sweep's yaw centre already
existed for `--stop-gaze-at-thing`; this is that half of it, on the look stop, from the kicked thing's position
rather than the last seek bearing. Prediction: answers observed up from a third toward two thirds, nodes
known > 0, renewals that lead to a second intent at the same thing; walls back to R67's.

**R71a measured (n = 6, against R71 and R67).** Skills 30 (17 unwinds, all 17 look stops aimed); answers
observed 10 of 30 (R71 11 of 31, R67 13 of 29); renewals 20; nodes known 0, need 1.0 at every end. On the
interesting scale the aim undid the linger's cost and a little more: stops at things 9.5 (R67) → 8.5 (R71) →
10.5 a run, stop-seconds at things 133 → 138 → 173, at walls 46 → 95 → 55; walls 17.9 → 29.0 → 18.5 a minute;
cells 142 → 139 → 131, stands 29 → 32 → 36 %, rescues 0.8 → 1.2 → 0.5 %. The look's geometry moved as
predicted: at the look stop the observed answers had the thing 0.38 m ahead and 12° off the nose, and the
UNKNOWN ones no longer had it to the side (34° median, 3 of 7 beyond 35°, against 55° and 10 of 14) — they had
it 2.17 m away: the skill had been fired where no movable thing was within 0.5 m (9 of 30). And of 30 skills,
2 moved their thing by more than 5 cm. **`PARTIAL` on the interesting scale (a signal at n = 6, all within
the spread), `NULL` on learning: the ladder is not climbed because the boundary's intents mostly do not
touch anything.** The linger and the aim are kept (both off by flag; R71a is the arm for the operator's eye)
and the phase's next lever is no longer the look — it is the intent vocabulary's reach: a kick from standing
at 0.19 m reaches the thing one time in five, and a third of the requests go to a wall base or a furniture
leg the cloud attends as a thing, or to a dead-reckoned place the thing is not. A PUSH (a short walk into the
thing from standing, the walker's own intent at the boundary) is the one thing in the runtime that moves a
thing every time it is pointed at one — the base walk moves objects 3.5 m a run by stumbling — and a thing
that rolls beyond the match radius must count as an answer, not an unknown.

### 17.46 The push: a third intent, and the vocabulary that cycles at a thing (R72, 2026-09-22)

**The lever, by the rewrite rule.** The operator asked for a wider vocabulary of behaviours and for
interaction over wandering. §17.45's diagnosis: the boundary's two intents mostly do not touch the thing (a
kick from standing at 0.19 m moves it one time in five; 2 of 30 skills in R71a moved their thing by more than
5 cm), so the outcome loop has nothing to learn from and the linger has nothing to hold. The one intent in the
runtime that moves a thing every time it is pointed at one is the walk itself: the base run moves objects
3.5 m a run by stumbling into them (§17.35's blind metric, read the other way). So the third intent is the
robot's own `move`, from standing, into the thing: **the push**, a window of 1.2 s in which the walker is
driven at 0.25 m/s forward (the walk's own scale and low-pass, the head as the walk holds it), then the same
unwind and look as a kick. On the robot it is `move` for 1.2 s; on the host it is a skill with no network
(`kSkills` entry `push`, `file = nullptr`, `push_vx`), id 4 on the boundary.

The loop's rule generalises without a new rule: `SkillOutcomeLoop.push_id` (−1 = absent) adds a third row
per thing node; the least-known choice is the fewest recorded outcomes, then the largest spread, then the one
after the last asked in the cycle — kick, peck, push — so a duck at a new thing tries all three before it has
an opinion, and the need (§17.45) counts the three. `key_of` is now `node × 4 + intent` (a saved outcome
state from before this change does not restore meaningfully; none is promoted). Tests: 7 (the cycle at one
thing, the need's fall to 0 after each is known twice, the habituation that follows). Config
`a1v2_r72_push.json` = R71 + `push_id 4`, run with R71a's flags (the aim); preset R72, seed 1.

Two things to read in the measurement: whether the push's answers are SEEN (a pushed ball may roll beyond
the 0.6 m match radius — the `skills` instrument counts "rolled beyond the radius" separately from "not
seen"), and the falls: a walker started from the R19 stand into a thing and stopped after 1.2 s is the step
hand-off of R48 (`★ STACK · the step`) plus a contact.

**R72 measured (n = 6, 1500 s, against R71a and R67).** Requests 46, skills fired 39 (kick 15, peck 13, push
11; R71a 30), answers observed 16 of 39 (R71a 10 of 30, R67 13 of 29), renewals 26, nodes known 0. On the
interesting scale the best arm so far: stops at things 12.0 a run (R67 9.5, R71a 10.5), stop-seconds at
things 191 (133, 173), at walls 62 (46, 55); walls 15.7 a minute (17.9, 18.5); stands 39 % (29, 36); arrival
stops 11.7 of 21 (8.0 of 17.8); cells 135 (142, 131); rescues 0.5 % (0.8, 0.5), nine over six runs. But the
push did not reach either: 1 of 11 pushes moved its thing by more than 5 cm (the kicks 0 of 15, the pecks
1 of 13), and 13 of the 39 skills fired with no movable thing within 0.5 m. **`PARTIAL` on the interesting
scale (a consistent signal at n = 6 across R71a and R72: more stops at things, fewer at walls, more stands,
within the spread), `NULL` on the intents' reach.** The `skills` instrument (`cloud_objects.py skills`)
carries the split.
Why the push does not reach: in its 1.2 s window the body travels 0.06 m (median over the 11; the walker
started from the stand is still getting under way — the low-pass and the gait's first step), and only 5 of
the 11 had the thing within 0.3 m and 30° at the start (three had no thing within a metre, two had it 51–61°
to the side). The one push that answered (0.35 m, a ball at 0.07 m) is what the intent is for. Re-use
context: a window long enough to walk the thing's distance (3 s ≈ 0.5 m) aimed at its bearing, fired only at
a thing seen now within reach (O59).

### 17.47 The spin as frustration, the roll as its answer, and the approach by sight (R73, R74, 2026-09-23)

**What the operator saw in R72 (seed 1).** Between 750 and 850 s the duck circled near the blocks and balls;
they read it as a turning-radius failure — a heading or place it could not reach — and asked (a) for a metric
of that frustration, (b) for the forward roll as the behaviour that breaks the cycle with a novel orientation,
(c) whether the walking cloud could find things on the approach and fine-tune the position before the kick or
peck, which lands beside the thing rather than in front of it.

**The window, read from the record.** From 740 to 826 s play holds the reference and the reference itself
turns at the body's rate (−2673° → −5841° in 86 s: O56, the reference that rotates with the body), the
heading error sits beyond 90° on most ticks, the body walks at 0.3 m/s in a circle of 0.3 m: 17.6 m of path
for 0.5 m of net displacement and 3165° of turn in 120 s, the block 0.3–0.6 m away the whole time. Not a
target inside the turning radius, then, but the orbit of §17.43 in its play form — and the operator's word
for it is the right one: the loop asks for a heading it never reaches.

**The frustration metric.** Three already exist in parts: the sweep's `straight` and `hdgErr`, the host's
progress stall (§17.44: no progress toward the reference while commanded forward), and the loop's own
competence (the range left, sign −1, which the arbiter's trust reads). None names the spin. The one that
does, on the body's own odometry only: *in 20 s of walking the heading turned more than a full turn while
the dead-reckoned position moved less than half a metre.* Offline (`cloud_objects.py spins`), spin windows
per seed at n = 6: R67 [6, 1, 14, 0, 7, 0], R71a [6, 3, 8, 2, 2, 2], R72 [6, 0, 1, 2, 2, 0] — seed 1's six
are the operator's 750–850 s. It is a property of the walk (R67's seed 3 spends 140 s in it), not of the
things levers.

**The roll, measured alone first.** Pollen's roulade fired mid-walk at 650 s on two seeds: a 1 s roll to 82°
and 49° of tilt, the host's recovery scaffold (Pollen's late fall detector stands in) up in about 4 s, the
walker back by 656 s, the heading 50–110° from where it was, nothing broken (3 rescues in each 720 s run,
the roll's among them). On the robot the daemon's own recovery would do the rising; in simulation the
scaffold is named as such.

**R73: the roll on a spin** (`--skill-on-spin NAME TURNS NET SECS`, off = byte-identical). The host keeps a
ring of the odometry pose over SECS of walking ticks (a stop or a skill resets the run) and, when the rule
above fires, asks for the skill by name at the boundary, at most once per 30 s; `spins` and the rolls fired
are counted and the sweep carries them. This is the stall detector's sibling (§17.44) with a different
answer: not an exit reflex toward free space, which the loops undid four seconds later, but a fall and a
rise that leave the body facing elsewhere, with play's committed turn and the twist brain's yaw history
reset by the recovery's hand-back. Prediction: spin windows down on the seeds that have them, walls and cells
within the spread, a few rescues more, and — the operator's scale — something to watch.

**R74: the approach by sight.** At R72's 43 arrival stops the thing was 0.18 m away (median) but 34° off the
nose, within 0.35 m AND 30° on 16 of 43, and more than 0.5 m away on 13 — the arrival is by dead reckoning to
a position fixed from a stop a metre back, and the last 0.3 m of the approach are blind. The walking cloud
(O55) sees the thing on the approach; what made it a `REGRESSION` as a behaviour in R70 was that every
walking sighting became a seek target (seek 80 % of the walk, walls 55 a minute) — and, found now, that
the map's place vector read the walking cloud as its view (`--map-view cloud`), so the map vocabulary
collapsed to 6 nodes. Two changes: the bearing token carries a fourth value, *seen from a walking cloud*,
and `BearingSeekLoop.walk_refix_m` (0 = off) lets such a bearing only REFINE a target already held, when its
fix lies within that distance of it — never set one; and the host's map view holds the stop's cloud through
a walking cloud. Config `a1v2_r74_walkrefix.json` = R72 + `cloud.walk_cloud` + `seek.walk_refix_m 0.5`.
Prediction: at the arrival stop the thing within 0.35 m and 30° on most arrivals, skills fired with no thing
within 0.5 m down from 13 of 39, and the map's node count back at R72's; the number to fear is walls.

**R73 measured (n = 6, against R72).** Spins detected 16, rolls fired 13 (one per 30 s at most); spin
windows 11 → 10 over six runs — seed 1's six windows moved from 700–850 s to 1380–1500 s: the roll at 709 s
broke the operator's spin, and the two at 1405 and 1459 s did not break the later one. Rescues 9 → 55: three
per roll (at 709, 711, 713 s — the recovery scaffold's rise takes three hand-offs, about 5 s; on the robot the
daemon's own recovery), not new falls. Stops at things 12.0 → 11.7 a run, stop-seconds at things 191 → 211,
walls 15.7 → 18.6 a minute, cells 135 → 136, stands 39 → 38 %, hdgErr 1.31 → 1.36. **`WORKING` as a
mechanism — the detector names the operator's frustration and the roll leaves the duck facing elsewhere
— `NULL` on the spin total at n = 6 (10 against 11 windows), a tie on everything else, and a new behaviour
for the eye.** Kept as a flag (preset R73). The brain-side form is the re-use: the roll as an intent the
arbiter prefers when the holding loop's competence has fallen (O60).

**R74 measured (n = 6, against R72), after one host fix.** The first run ended every stop at 1.4 s: the
stop-end rule read the walking cloud's file-and-reopen as "the cloud stopped growing" — the true cause of
R70's collapse (stands 4 of 17, 6 map nodes), now gated to the stop's own cloud. Rerun: stops 16 s, nodes 31,
the sensor live (4 000–12 000 re-fix ticks a run). And a `REGRESSION` on its own prediction: at the arrival
stop the thing within 0.35 m and 30° on 4 of 32 (R72: 16 of 43), more than 0.5 m away on 18 (13 of 43), the
median 0.56 m and 86° off the nose; stops at things 12.0 → 5.7 a run, stop-seconds at things 191 → 73; seek
held the reference 77 → 93 % of the walk, walls 15.7 → 20 a minute; answers observed 11 of 30 (tie); the
skills that reached rose to 5 of 30 (2 of 39). The re-fix drifts: within 0.5 m of a held target the walking
cloud offers wall bases, chair legs and the same thing seen from a moving, less exact anchor (the operator's
own caveat), and the target follows each. Re-use context: a re-fix that must be the NEAREST small thing and
within the sweep's ±0.6 rad of the nose, at a radius under 0.25 m, in the last metre only — or the honest
form, a live bearing that competes with the remembered one by precision, as the voter fuses. Kept off; the
stop-end fix stays (it is what any walking cloud needs).

### 17.48 O59: fire at what you see, and the intent that reaches (R75, R76, 2026-09-23)

**The two levers, on the operator's "proceed".** (a) `SkillOutcomeLoop.reach_m`: an arrival only ARMS the
loop (for `armed_ticks`); the request goes out on the first tick the thing's bearing is live within reach and
ahead (forward cosine ≥ `reach_cos`), and a window that lapses is a *miss*, counted. The host honours a
request that arrives mid-stop from standing at once (the same `skill:stand` as the deferred path). (b) The
push that reaches: `--push-reach VX MAX_S` sizes the push's window to the seen distance, (range + 0.15 m) / VX
between 0.6 s and MAX_S, from the need topic's position, and keeps the nose on the thing with the unwind aim's
gain; and `reach_short_m` lets a kick or a peck be asked for only within that range, the push alone beyond it.
A/B on R72 (the promoted stack + push), R71a's flags.

**R75 (reach 0.5 m, n = 6): the gate starves.** Two skills in six runs (R72: 39), 29 misses; both fired were
seen and answered (0.29 m, moved 0). The reason, from the record: at the 49 arrival stops the reached thing
was within 0.5 m on 40 (0.19 m median), and the cloud attended something within 0.5 m on 3 — the reached
thing sits below the level gaze (T3, §17.40) and what the sweep attends is 0.93 m away. Stops at things 12.0
→ 9.3 a run but stop-seconds at things 191 → 238 (the armed stops run their 30 s), walls 15.7 → 14.2, cells
135 → 112. **`NULL` as a behaviour in this form: the rule is right and the gaze cannot feed it at 0.2 m.**
The re-use is exactly R76: open the sighting gate to what the gaze CAN see (1 m) and let only the intent that
walks the distance answer beyond a kick's reach.

**R76 measured (reach 1 m, the push that walks the distance; n = 6, against R72).** Nine skills in six runs,
all pushes (R72: 39); answers observed 7 of 9 — the best rate of the phase (R72 16 of 39) — and the first
node ever KNOWN (seed 1: two pushes on one block, both seen); misses 15 of 24 armings. On the interesting
scale a cost: stops at things 12.0 → 7.0 a run, stop-seconds at things 191 → 158; walls 15.7 → 12.3 a minute,
stands 39 → 43 %, cells and rescues tie. The pushes: six of nine had a 1.5 s window (the need position 0.25 m
off) and travelled 0.10–0.13 m — the walker started from the stand moves at 0.07 m/s over its first two
seconds — and the two 5 s windows travelled 0.36–0.38 m and ENDED ON the thing (0.01 and 0.06 m) without
moving it (0.05, 0.00 m): the walker stops at the contact rather than pushing through. Two of nine moved
their thing (a ball at 0.26 m, 0.19 m; the block, 0.05 m). And at the arrival stops themselves the cloud
attended something within 1 m and ahead on 2 of 33 — the pushes fired from later sightings, not from the
arrival. **`PARTIAL` on learning (the answers are honest when the rule fires), `REGRESSION` on the interesting
scale, `NULL` on reach.** Not promoted; preset R76 for the eye.

**What O59 taught, for the phase.** Every rule that fires at what the duck sees starves on the same fact: at
a stop the level gaze does not see the thing the duck has walked to (0.19 m away, 45° under the beak; T3,
§17.40), and what it does see is 0.9 m off. The kick and the peck cannot reach what the gaze can see, and the
push that could is a walker that needs two seconds to get going and stops at contact. The lever under all of
this is not another intent rule but the gaze at the reached thing — the pitch that T3 tried and the stand
refused (rescues ×4) — which is O36: a stand that tolerates the head's pitch. That is the design decision for
the operator before more intent levers: (a) O36 first (the stander with the head pitched down at arrival
stops), then O59's rules as built; or (b) a push that pushes — a longer window at the walk's own speed and a
contact rule that keeps driving through the thing — measured on its own with the roll and the other spice
already in hand.

### 17.49 O36: the stand with the head down — measured, and it holds (2026-09-27)

**The operator's direction.** R75 and R76 fired almost nothing to the eye (2 and 9 skills in six runs; R76
walked backward at 700–740 s of seed 1 — the twist brain's forward command flipping sign with its saturated
yaw, the orbit of §17.43 in yet another dress, not the unwind); R73 remains the most interesting arm. So O36:
"the robot should be able to stand with the head angled down as long as its hips shift back to compensate for
the change in CoG. Let's investigate."

**What the record already said.** The standard stop's gaze sweep never pitches the head below +0.16 rad (its
band is `gaze_pitch_sd` × (−0.7 .. +2.0) about a centre that was 0 and had no flag). T3's `--stop-gaze-at-thing`
centred the band at 0.2–0.55 rad at arrival stops and the rescues rose ×4 (§17.40); that was read as "the
stand refuses the head's pitch", and O59's gates starved on a gaze that could not see the reached thing.

**The instrument.** `--stop-gaze-down RAD` sets the sweep's pitch centre at every stop, and `--log-com`
writes the whole body's centre of mass over the midpoint of the two soles, in the heading frame, to the
record (MuJoCo's subtree CoM; the sole geoms; truth for the harness). Both off = byte-identical (guard
`cb24520c`).

**Measured (R67 stack, 3 seeds × 1100 s, stops from 600 s), head pitched at four centres:**

| pitch centre (rad) | head_pitch joint | CoM forward of the soles' midpoint | trunk grav_x | hip pitch | stops · survived · rescued at stops |
|---|---|---|---|---|---|
| 0 (the standard) | +0.45 | −0.5 cm | −0.001 | −0.415 | 8 · 95 % · 0 |
| 0.3 | +0.82 | +0.2 cm | +0.007 | −0.415 | 16 · 83 % · 0.3 |
| 0.5 | +1.04 | +0.5 cm | +0.009 | −0.415 | 15 · 81 % · 0 |
| 0.7 | +1.25 (72°) | +0.6 cm | +0.010 | −0.415 | 8 · 95 % · 0 |

The head is a quarter of the duck (189 g of 740) but it pivots close to the trunk: pitched to 72° it moves
the whole body's centre of mass **1.1 cm** forward, the trunk leans 0.6°, and the hips do not move at all —
the R19 stand holds its pose and the ankles carry the offset, well inside a 6 cm sole. No stop was rescued
at 0.5 or 0.7 rad; the hand-offs to the walker past 6.5° of lean rose a little at 0.3–0.5 (survival 95 → 83 %)
and not at 0.7. **The stand tolerates the head down. The hips need not shift; the CoG the operator asked
about moves a centimetre.** T3's rescues came from something other than the pitch — the next measurement
(R77 below) puts the gaze at the thing back on the R72 stack with the CoM on the record to find it.

Two side effects worth the eye: with the head down the stops DOUBLE (8 → 16 at 0.3 rad, 15 at 0.5) because a
gaze that sees the floor sees the things on it, and the seek loop's arrivals follow — exactly the sightings
O59's gate starved for — and the sweep's pitch band at 0.7 rad covers +0.64 .. +0.86, which is where a
reached thing at 0.2 m sits (45° = 0.79 rad under the beak, §17.40).

**R77: the gaze at the reached thing, on the promoted stack (R72 + `--stop-gaze-at-thing`, n = 6, 1500 s).**
The arm the phase has been looking for. Skills 39 → 55 (kick 20, peck 19, push 16), answers observed 16 → 31,
things moved by more than 5 cm 2 → 10 (the pushes 5 of 16, the right kick 3 of 9), stops at things 12.0 →
14.3 a run (stop-seconds 191 → 182, at walls 62 → 100), arrival stops 11.7 → 16.5 of 24.8, walls 15.7 → 13.5
a minute, contact 2.2 → 1.2 %, stands 39 → 43 %, rescues 0.49 → 0.26 % — and ONE stop rescued in six runs
(stop survival 90 → 87 %). At the look stop the thing sits 0.28 m ahead and 18° off the nose whether the
answer is seen or not; the unknowns are now the thing that did not move (14 of 24) and 7 fired at nothing.
Nodes known still 0 (55 requests over 30 nodes × 3 intents; the ladder needs two answers per cell). **T3's
verdict is reversed on this stack: the gaze at the thing is `WORKING`, no cost to the stand.** Its 2026-09-17
rescues ×4 belonged to R64's walk (the reflex-less orbit arriving at things at speed) and the R60a-era
stop, not to the head's pitch. Preset R77 for the eye; not promoted before it. R78 = R75's gate (fire at what
you see within 0.5 m) on R77, running.

**R78: O59's gate on R77 (R75's `reach_m 0.5` + the gaze at the thing, n = 6).** The gate is fed now and it
is perfectly honest: 12 skills fired, 12 answers observed (the first arm with no unknown), at the look stop the
thing 0.22 m ahead and 13° off the nose; stop-seconds at things 182 → 260 (the most of the phase), walls 13.5 →
10.6 a minute, stop survival 87 → 99 %, stands 43 → 51 %. And it fires a fifth as often as R77 (12 against
55): 21 requests were made, 9 of them dropped by the host — a request that landed while the stop was still
settling, or during a skill's own window or its unwind, was read only in the standing phase — and 20 armings
lapsed as misses. One thing moved in twelve. **`PARTIAL`: honest answers and a calmer walk, `REGRESSION` on
the interaction count against R77.** The dropped requests are fixed (kept as pending, fired at the next
hand-back) and R78 is rerun as R78b below.

**Why nothing is ever "known" (every arm since R65).** The outcome table is keyed by the thing EPM's node ×
intent — about 30 nodes × 3 intents = 90 cells — and a run makes 12–55 requests. Two answers per cell is a
bar the phase's request rate cannot reach; the ladder that habituation and the linger need never climbs.
The vocabulary that decides the cell is finer than the question ("what does a kick do to a ball") by a factor
of ten: the thing EPM separates balls from blocks at purity 0.9 but with ~7 nodes per kind. The fix is not a
lower bar but a coarser key — the node's kind, as the EPM's own baked-node clusters give it, or the descriptor's
first two dimensions — and it is the next design item after the operator's eye (O62).

**R78b (the dropped requests kept as pending, n = 6) — and a double fire.** Skills 46, the loop's own
requests 25, every one of them answered (25 of 25 seen; the `skills` instrument reads 37 of 46 because the
duplicates below are counted too); the first nodes ever KNOWN (seed 5: two); stops at things 16.0 a run,
stop-seconds at things 262, both the phase's best; walls 14.4, rescues 0.26 → 0.73 %. The record shows why
46 and not 25: each request fired TWICE — started at once from standing (`skill:stand`) and, on the same
tick, read again by the new deferral as "landed during a skill's window" and kept as pending, so the same
skill ran again 3 s later at the look stop's hand-back with the thing 0.45 m back. The duplicate is what
raised the rescues. Fixed (a request started this tick is not deferred); rerun as R78c.

**R78c (the double fire fixed; n = 6, against R77 and R72).** Requests 36, skills 35, answers observed 36 of
36 by the loop's own count (31 of 35 by the instrument's stricter matching; the 4 unknown had the thing 0.38 m
ahead and 12° off) — the first arm in which every ask is answered — and nodes KNOWN 3 (two seeds). On the
interesting scale it is the linger the operator asked for in §17.45: stops at things 18.2 a run (R77 14.3,
R72 12.0), stop-seconds at things 358 (182, 191), at walls 45 (100, 62), walls 7.5 a minute (13.5, 15.7),
stands 53 % (43, 39), rescues 0.49 % (0.26, 0.49), stop survival 93 %. Its costs: cells 104 (131, 135) — a
duck that stays with things covers less room — and things moved 1 of 35 (R77 10 of 55): the gate fires at a
thing seen within 0.5 m from where the duck already stands (0.26 m at the look), and a kick or a peck from
there does not reach it, while R77's ten came from arrivals at 0.18 m. **`WORKING` as the linger-and-learn
arm (every answer seen, the first known nodes, the fewest walls of the phase), `PARTIAL` on the interesting
scale against R77 (more standing at things, fewer things moved).** Both are for the operator's eye; the
recommendation is R77 as the `★` candidate on their scale and R78c as the learning form to carry once the
outcome table's key is coarse enough to be climbed (O62) and the reach is closed (O59).

**Promoted (2026-09-27): `★ THINGS` = R77.** The operator's eye: "the robot looking down is stable and
interesting"; R78 "ambiguous and not an improvement over R77". The promoted run is R72's stack with
`--stop-gaze-at-thing` (preset `★ THINGS · R77`, seed 1; harness argv = the standard stops + `--stop-on-arrive
--skill-unwind 0.3 1.5 --skill-unwind-aim 1.0 --log-com --stop-gaze-at-thing` on `a1v2_r72_push.json`). The
reach gate (R75's `reach_m`), the linger's renewal, the roll on a spin, the walking cloud and its re-fix stay
as options, off. Next, on R77: O62 (a coarser outcome key) and the reach (the approach ending with the thing
at the foot).

### 17.50 On `★ THINGS` R77: the kind as the outcome's key (O62) and the arrival at the foot (O59) — R79, R80 (2026-09-27)

**R79, the kind.** The outcome table's key has been the thing EPM's winner × intent: ~30 nodes a run (the
vocabulary separates balls from blocks at purity 0.9, with some seven nodes per kind) × 3 intents, and a run
asks 12–55 times, so no cell reaches `min_samples` = 2 and nothing is known (§17.49). The coarser key is not a
hand-rolled clusterer (CLAUDE.md §0 rule 1) but a second EPM over the same descriptor with `max_nodes 4` and a
coarser insertion floor (0.25 against 0.06), `thing_kind_epm` on `reality.cognitive.thing_kind`; the outcome
loop's `thing_topic` reads it. Nothing else changes: the fine vocabulary still drives attention and the seek.
The record carries the kind token as `tkind`; `cloud_objects.py things --field tkind` scores its purity by
object kind. Prediction: nodes known > 0 on most seeds, the linger's need able to fall, requests per known
cell ≥ 2; the risk is a kind that lumps a wall base with a block (purity), in which case the table learns
that "blocks" sometimes do not move — which is true of wall bases.

**R80, the arrival at the foot.** R77's ten moved things came from arrivals where the thing was 0.18 m off;
R78c's one from a kick at 0.26 m. The seek loop arrives by dead reckoning at `arrive_m` 0.25 and the stop
starts there; `arrive_m 0.15` (with the outcome loop's `arrive_range` 0.2) ends the approach with the thing
where a kick from standing reaches. Prediction: things moved up, the thing's range at the look stop down from
0.28 m, more contacts on arrival (the base moves objects by walking into them — interesting on the operator's
scale, and a fall risk to watch: rescues).

**R79 measured (n = 6, against ★ R77).** The ladder climbs: nodes KNOWN 14 over six runs (5, 0, 0, 3, 3, 3; R77
0), and on two seeds the need at the last attended thing fell to 0.33 and 0 — the first time habituation
has had anything to act on. Skills 55 → 60 (kick 22, peck 19, push 19), answers observed 31 → 44, things
moved 10 → 9, stops at things 14.3 → 17.0 a run, stop-seconds at things 182 → 231, walls 13.5 → 9.6 a minute,
cells 131 → 117, stands 43 → 46 %, rescues tie. The kind itself is coarse in the way feared: purity 0.78
against a chance of 0.66 (+0.12 ± 0.10; the fine vocabulary was +0.30–0.44), four nodes with blocks and balls
mixed under some — so what the table learns is "this kind of thing, roughly". **`WORKING` as the mechanism
O62 asked for; `PARTIAL` on the kind's purity.** The re-use: a kind that is the fine EPM's own baked-node
clusters (topology, not a second GNG) would carry the fine vocabulary's purity into the coarse key.

**R80 measured (n = 6, against ★ R77): `REGRESSION`.** Arriving at 0.15 m by dead reckoning is arriving
rarely: arrival stops 16.5 → 5.8 a run, skills 55 → 21, stops at things 14.3 → 7.0, stop-seconds at things
182 → 99, walls 13.5 → 28.3 a minute, stands 43 → 24 %, rescues 0.26 → 0.84 %. The seek loop's dead-reckoned
range rarely falls under 0.15 m before the target is forgotten or the body bumps the thing, so the walk
becomes R64's again. Things moved 3 of 21 (the right kick 2 of 5). Not kept. The reach stays with O59 in a
different form: the closing step must be by SIGHT at the stop (the pitched gaze sees the thing at 0.26 m; a
short step onto it before the kick), not by a tighter dead-reckoned arrival.

### 17.51 On `★ THINGS` R79: the closing step, and why the stops lengthen (2026-09-27)

**Promoted: `★ THINGS` = R79.** The operator's eye: "the robot successfully kicked the block several times and
lingered to play"; the peck "slightly off — just slightly too far back from the object to make contact with
the beak"; and "the robot should make contact with the head/beak even if it results in a fall — we want the
robot to perturb its environment as much as possible." Also: the looking grows longer as a run goes on.

**Reach, measured on R79's 41 kicks and pecks.** Every thing that moved had its edge 0.05–0.08 m from the
body when the skill began (the left kick 4 of 14 at 0.05–0.07 m; the peck 1 of 19, at 0.08 m); seven pecks
within 0.15 m and six left kicks within 0.15 m moved nothing. The reach of both intents is a hand's width;
the stop, by dead reckoning, leaves the thing 0.12–0.20 m off (median). A tighter arrival did not close it
(R80). **The closing step** (`--skill-approach REACH VX`, off = as before): when a kick or a peck is asked for
and the seen thing's centre is beyond REACH, the duck first steps onto it — the walker at VX for
(range − REACH) / VX seconds, at most 3 s, the nose kept on the thing — and the skill follows at once from
wherever the step left the body; the unwind and the look follow the skill as before. The step is `move` and
the skill is `do`: two of the daemon's verbs in sequence, from the stop. R81 = R79 + `--skill-approach 0.10
0.25` (preset R81). Prediction: pecks and kicks that move their thing up from 5 of 41; the price the operator
has accepted, more falls (a peck from a body still settling from a step), to be counted.

**Why the stops lengthen.** In R79 the stops from 600 to 900 s are ended by the gaze's "six known gazes in a
row" rule 60 times, by a hand-off 15 times, by the 60 s cap once; from 900 to 1200 s the cap ends 18 of 49,
and the stops' median length climbs from 8 s to 60 s in the 1050–1200 s window. In a capped stop the map's
token sits on one winner while its novelty flag flickers on every few gazes — the map is still inserting
nodes late in the run (28 → 29 during that stop), so six *known* gazes in a row never come, and the cloud
rule ends only 17 of 33 stops. It is the map growing, not the duck getting slower; the 60 s cap is what the
eye sees. The cheap knob is the cap (`--stop-secs 30`) or letting the cloud's growth alone end a stop that
the gaze cannot; both are a preset's argv, to be measured when the operator wants it — "not a big issue".

**R81 measured (n = 6) — a loop, then a fix.** Steps onto a thing: 295, 7, 204, 333, 301, 24 a run; skills
fired 11; walls 9.6 → 99 a minute, arrival stops 17.8 → 3.7. The step that ended beyond reach started the skill,
whose start began another step: with the walker making 0.06 m over a 1.2 s window from the stand and the
window sized on 0.25 m/s, the closing step rarely closed, and the pair looped for minutes, chasing a
remembered centre into the wall. Two fixes: one step per request (the skill that follows a step never steps
again), and the step ends by the odometry — the remembered centre within REACH — under a ceiling sized on the
walker's real start-up speed (0.1 m/s, at most 3 s). Rerun as R81b.

**R81b measured (one step per request, the step ended by the odometry; n = 6).** The loop is gone: 29 steps
for 40 skills (kick 17, peck 12, push 11). But the step closes little — the thing's edge 0.21 → 0.17 m over a
1.4 s window (6 of 29 to within 0.08 m), because the walker from the stand barely moves in a second — and the
thing is 37° off the nose when the step begins and 49° when the peck does: a peck straight ahead misses a
thing beside it. Pecks moved 0 of 12 (two of them at 0.04 and 0.07 m), kicks 2 of 17, pushes 3 of 11; no fall
followed a skill (the rescues 6 → 17 are the walk's). On the interesting scale a `REGRESSION` against R79:
stops at things 17.0 → 11.8 a run, walls 9.6 → 14.6, stands 46 → 34 %, nodes known 14 → 6. The reach is
angular as much as radial: the closing step must FACE the thing before it walks. R81c: the step turns in
place while the thing is more than 0.2 rad off the nose, then walks, and ends when the centre is within reach
AND on the nose, under a ceiling sized on both (at most 4 s).

**R81c measured (the step faces the thing first; n = 6).** The most skills of any arm — 64 (kick 26, peck 21,
push 17) after 47 steps — stops at things 17.2 a run and nodes known 12, a tie with R79 on the interesting
scale; and **things moved 0 of 111.** The step does not step: over its 1.9 s window the thing's edge goes
0.17 → 0.15 m and its bearing 28° → 27°, and 4 of 47 steps end within reach and on the nose. The walker from
the stand neither turns in place nor walks in under two seconds (it needs about two to get under way, §17.46),
and a kick or a peck fired the moment it is asked to stop connects with nothing. **`REGRESSION` on reach in
all three forms of the closing step (R81, R81b, R81c); the walker is not a stepping tool from the stand.**
Kept as a flag, off. What has moved things in this phase: a kick from a SETTLED stand with the thing 0.05–0.08 m
off (R79, 4 of 14), and the push — walking into the thing (R77, 5 of 16; R81b, 3 of 11).

**R82: the skill from the walk.** The operator's rule — "contact with the head/beak even if it results in a
fall; perturb the environment as much as possible" — lifts the constraint the runner has carried since
§17.39, where the mid-walk kick fell 28 % of the time and was moved to the arrival stop's hand-back.
`--skill-now`: a request at the arrival tick fires at once, from the walk, with the body still closing on the
thing at 0.25 m by dead reckoning and moving at 0.3 m/s. Prediction: things moved up (the foot arrives at the
thing at speed), falls up (rescues), the answers seen down (a fall is not a look). Preset R82. The eye decides
what a fall is worth.

**R82 measured (n = 6, against ★ R79).** Contact: the PECK from the walk moved its thing 5 of 11 (R79: 1 of
19; every other arm 0–2 of ~20), the push 5 of 12, the kicks 0 of 16 (fired at 0.29–0.46 m, the foot never
arrives); things moved 10 of 39 against R79's 9 of 60 — and the falls the operator priced in did not come:
3 of 39 skills were followed by a fall within 8 s, rescues 0.32 → 0.63 %. The beak reaches from the walk
because the body is still closing on the thing when the reach-down begins. The cost is the rest of the cycle:
a skill fired at the arrival tick pre-empts the arrival stop (arrival stops 17.8 → 0; stops 26 → 11, the timer's
only), and the unwind and look ran only from a stop, so 2 of 39 answers were seen, nodes known 0, stops at
things 17.0 → 4.0 a run, walls 9.6 → 28 a minute. **Contact `WORKING`, the cycle `REGRESSION`.** Fix: the
unwind and the look follow a skill fired from the walk as they follow one from standing; rerun as R82b.

**R82b measured (the unwind and look after a skill from the walk; n = 6, against ★ R79).** The cycle is
back: skills 49 (kick 18, peck 17, push 14), answers observed 26 (R82: 2), nodes known 7, arrival stops 7.8
(R82: 0; R79: 17.8), stop-seconds at things 274 (R79 231), stops at things 11.5 a run (17.0), walls 12.0 a
minute (9.6), cells 110 (117), stands 48 % (46). The peck from the walk keeps its reach — 3 of 17 moved their
thing here, 5 of 11 in R82: 8 of 28 against 1 of 19 from the stand — and now pays: 8 of the 49 skills were
followed by a fall within 8 s (4 of the 17 pecks, 4 of the 18 kicks), rescues 0.32 → 0.70 %. The kicks from
the walk do not reach (1 of 18: fired at 0.15–0.22 m, the foot swings before the body arrives). **`PARTIAL`:
the peck's contact `WORKING` from the walk (the operator's ask), the cycle a tie, the interesting scale down
(stops at things 17 → 11.5) and the falls doubled — the price the operator named.** The eye decides; R79 stays
`★`. If the contact is worth the price, the form to carry is R82b's flag with the kicks left to the stand
(the peck alone from the walk) — one line in the host, measured next if asked.

### 17.52 The phase closes: R83, the closer arrival on the kind, promoted on the operator's eye (2026-09-27)

**The operator's call.** After the R81/R82 report: "let's be sure we are using R80 as our promoted latest config —
the closer distance for interaction is a win." R80 was minted on R77 and carries no kind (§17.50), so the stack the
eye asked for is R79 with R80's two numbers, `seek.arrive_m 0.15` and `outcome.arrive_range 0.2`: **R83**,
`a1v2_r83_kindfoot.json`, on R79's harness flags. Measured at n = 6 before the star moved.

**R83 measured (n = 6, against ★ R79).** Every aggregate is R80's to the last decimal: walls 9.6 → 28.3 a minute,
arrival stops 17.8 → 5.8 a run, stops 26 → 16, stops at things 17.0 → 7.0 (stop-seconds at things 231 → 99),
skills 60 → 21 (kick 9, peck 8, push 4), things moved 9 → 3, answers observed 44 → 8, nodes known 14 → 0, stands
46 → 24 %, walk 48 → 71 %, rescues 0.32 → 0.84 %, cells 117 → 147, path 73 → 102 m. The identity with R80 is not a
confound (§3.2 rule 7 checked): the kind's records are on R83's log (2266 `tkind` lines on seed 1, none on R80's),
and the kind changes a choice only once a cell holds two answers, which no cell does in 21 skills over six runs —
the least-known cycle asked the same intents at the same moments and the physics followed. Per seed, the preset's
seed 1 is the phase's exception again (§17.45): R79 s1 26 stops at things / 327 s / 15 skills / 3 moved / 13 answers;
R83 s1 16 / 297 s / 8 / 1 / 4; seeds 3, 4 and 6 are wall seeds (59, 31, 42 walls a minute). Where an arrival does
complete the thing is closer — the eye's win — but the dead-reckoned range falls under 0.15 m a third as often
before the target is forgotten or bumped, and the walk between arrivals is R64's.

**Verdict and promotion.** On the harness `REGRESSION` on the interesting scale, as R80 (§17.50); on the operator's
eye the closer interaction is worth it, and the eye is the gate (CLAUDE.md §3 rule 5): **`★ THINGS` = R83**, preset
"★ THINGS · R83" (seed 1), with R79 kept one line below as the comparison arm. Re-use: the closer arrival wants a
target that is not forgotten on the way — a seek target renewed from the pitched gaze in the last half metre (O61's
re-use) would give R83's closeness with R79's arrival count; and the peck from the walk (R82b) reaches without
arriving at all. Both belong to the next phase's design, not to a lever tonight.

**The phase closes here.** Entry point for a cold start: the phase page
[`microduck_things_phase.md`](microduck_things_phase.md) §10–13 — the promoted run, the phase in one table, the
findings, and what to carry into the next push, chasing moving things. The launcher's presets were pruned 82 → 13
(the rest in `tools/duck_launcher/presets_archive.json`, argv intact) and 46 refuted configs lost their rank (files
kept, names keep their verdicts).

### 17.53 The chase phase opens: a toy train on a track, and what the walking cloud says about a thing that moves (stage 0, 2026-09-27)

**The operator's direction** (the phase page [`microduck_chase_phase.md`](microduck_chase_phase.md) §1): chase moving
objects, using the ToF while walking to tell whether part of the cloud is changing relative to the rest; a
predictable mover for the playroom — "a toy train or car on a track that stops and starts at regular intervals";
home in on *any* cluster moving relative to the world frame. The design discussion (§2 there) put the error as the
seek loop's own residual — a thing not where the assumption "things do not move" predicted it — and ranked a moving
fix inside the seek loop first, a separate chase loop second, T6's free space third, with a stage 0 that measures
the signal before any loop rides it.

**Built, all off by default and byte-identical off** (the R83 argv on the plain room, seed 3, 300 s: the JSON stream's
md5 `6b9a0b3a…` before and after, twice): `playroom_gen.py --train` (the train, its track in the scene's `<custom>`
block, the plain room regenerating unchanged), `--train SPEED RUN STOP` (kinematic, seed-phased), `--log-movers W`
(`CloudMap::cluster_recent`: the stack rule over the voxels seen in the last W seconds, each cluster with the share
of its voxels first seen in the window, their mean age and their hit-weighted age; two unit tests),
`mover_tracks.py` (labels by truth, tracks, velocities, gates), the R84 preset.

**R84 = R83 + `walk_cloud`, the train room, `--train 0.2 8 8 --log-movers 0.5`, n = 6 × 1500 s.** The walk is R74's,
as expected with the walking cloud ungated: walls 31.1 ± 24.3 a minute (seeds 3.1 to 68.5), seek 37.6 % of the
walk, play 62.4 %, stands 34 %, stops 16.8 with 7.2 arrivals, rescues 0.12 a minute, path 84 m, cells 134. Not a
lever, not compared.

**The train is seen.** 104 847 walking casts with a cloud open (139.8 min), 4.4 clusters a cast. With the train
within 2 m and 0.6 rad of the head's look, a cluster lies within 0.22 m of it on 35 / 76 / 65 / 29 % of casts by
half-metre bin while it moves and 41 / 80 / 74 / 34 % while it stands.

**The centroid's velocity: `NULL` as a separator.** Estimated over the last 0.5 s of a track (nearest-neighbour
association under 0.10 m between casts of one cloud): compact static clusters 0.053 / 0.188 / **0.378** m/s (p50 /
p90 / p99), wide ones (over 0.25 m) 0.140 / 0.376 / 0.626, balls and blocks 0.048 / 0.148 / 0.349, the stopped
train 0.050 / 0.130 / 0.250, the **moving train 0.092 / 0.208 / 0.332** against a true 0.20; velocity error median
0.157 m/s; the common mode (the median over compact static tracks) 0.030 m/s p50, 0.113 p90, and subtracting it
moves the static p99 from 0.378 to 0.350. Recall on the moving train at 1 / 1.5 / 2 × the static p99: 0 / 0 / 0,
with 19 / 2.3 / 0.34 static clusters over the threshold a minute. A window's smear lags the centroid and the
sampling's flicker moves a static centroid faster than the train moves.

**The voxels' age: the separator.** Over clusters tracked for at least 0.5 s in clouds older than 1 s (271 759
samples): mean voxel age p10 / p25 / p50 / p90 — static 0.26 / 1.62 / **7.07** / 41.7 s, wide 0.72 / 2.37 / 7.38 /
26.6, balls and blocks 0.44 / 1.55 / 6.36 / 28.0, the stopped train 0.12 / 0.40 / **0.96** / 6.21, the **moving
train 0.06 / 0.13 / 0.26 / 10.7**. Freshness (the share first seen in the window): static 0.09 p50 and 0.71 p90,
the moving train 0.50 p50. Gates, per minute of walking cloud:

| gate | recall, moving train | stopped train passing | everything else / min | of which within 1.5 m | direction error p50 |
|---|---|---|---|---|---|
| age < 0.3 s | 0.55 | 0.20 | 167 | 51 | 0.31 rad |
| age < 0.5 s | 0.69 | 0.32 | 237 | 88 | 0.31 |
| age < 1.0 s | 0.77 | 0.51 | 358 | 158 | 0.33 |
| fresh ≥ 0.7 | 0.29 | 0.17 | 158 | — | 0.32 |
| age / watched < 0.15, watched ≥ 1 s | 0.45 | 0.11 | 68 | — | 0.31 |

At age < 0.3 s the false alarms by range: 1.7 / 13.4 / 36.3 / 59.9 a minute by half-metre bin; by class 141
compact static, 16 wide, 10 balls and blocks; by sampling they are not the sparse clusters (0–4 returns in the
window: 23 a minute) but the well-sampled ones (16 and over: 77) — big things sliding into view. The moving train's
old-voxel tail (p90 10.7 s) is the train re-entering its own trail on a 3.17 m closed track within one cloud's
life; a rolling ball does not do this. The **hit-weighted age** (`age_w`, a re-hit voxel counting for its returns; the
third sweep, the same seeds reproducing the same ages to the digit) buys a little: at 0.3 s recall 0.40 with 128 a
minute (34 within 1.5 m) against the plain age's 0.55 / 167 / 51; at matched recall about a sixth fewer false
alarms — the weighting cannot help where the false alarm is a thing whose voxels are all new.

**Verdict.** Stage 0 `WORKING`: the signal exists in the walking cloud at the reach's range, and it is the voxels'
own age, not the centroid's velocity (that form `NULL`, re-use: a longer window or a tracker with a motion model
might read the speed, but the age already answers the question the speed was asked). The confounds are named and
each has its remover in the loop: a thing newly in view ages out within a second (persistence under the loop's own
prediction), the far fragments fall to the last-metre gate. Stage 1 builds the gate and the moving fix (the phase
page §5). Register: O63 updated, O64 opened.

### 17.54 Stage 1 of the chase: the mover candidate, the moving fix, and the bearing that was taken from the anchor (R85, R86; 2026-09-27 evening)

**Built** (the phase page §5's design, off by default): `CloudMap.mover_topic` — the cluster of the open cloud, through
the recency window, whose hit-weighted mean voxel age is under `mover_age_k` (0.06) × the OLDEST cluster's age in the
window (the cloud's own proof of how long it has been watching; a fresh cloud vouches for nothing), within
`mover_range` (1.5 m), any size, the nearest published as [vx, vy, proximity, age, oldest]; and the CHASE in
`BearingSeekLoop.mover_topic` — a sighting held as a candidate with a position in the odometry frame, confirmed by a
later sighting within `chase_gate_m` (0.35) of where the candidate should now be, chased after `chase_confirm` (2)
sightings spread over `chase_confirm_ticks` (25): the target its predicted position `chase_lead_s` (0.3) ahead, the
need 1, no arrival, forgotten after `chase_forget_ticks` (50) unseen into an ordinary remembered target. Four unit
tests on the loop, two on the candidate. R85 = R84 + both topics; the record carries `"chase"`; `mover_tracks.py
--chases` labels every chase by what its target really was, through the body's own pose.

**R85, the first arm (n = 6, R84's control, the same room and argv): the chase chased the room.** 338 chases in six
runs (56 a run, 1.2 s each, 82 s of chasing a run): 301 at static clusters, 33 at balls and blocks, 3 at the stopped
train, **1 at the moving train**; the closest approach to a moving train 1.4–2.4 m by seed. The walk changed with it:
seek held the reference 38 → 78 % of the walk (play 62 → 22 %), walls 31 → 19 a minute, rescues 0.12 → 0.09,
stands and stops the same. Not a gate that fails on the room's flicker: the seed-1 smoke run showed 9 chases in 240 s
with the chased "movers" reading 0.2–0.3 m/s — the body's own speed.

**The bug.** `CloudMap::update_bearing` (and the mover's copy) turned the cluster's ANCHOR-frame position into the
body frame by the yaw drift alone. On a stop's cloud the body is the anchor (translation is ignored on purpose,
the header's point 3). On a WALKING cloud the body has moved up to `walk_reset_m` from the anchor, so the bearing
and the range were those from the anchor: a fix in the odometry frame then equals the thing's true position plus
the body's displacement since the anchor — every static cluster reads as a thing moving at the body's velocity,
and the chase, asked to confirm a thing that follows its own prediction, confirmed the room. Fixed (`body_rel`:
the body's displacement, in the anchor's frame, taken off before the turn; the attended thing and the mover chosen
by the range from the BODY; a unit test with the body 0.4 m along and 0.3 m beside the line). Stop clouds are
untouched by construction (R83's guard: md5 `6b9a0b3a…` before and after). **The same bearing fed R74's walk re-fix
(§17.47, O61): "the re-fix drifts to wall bases and legs within 0.5 m" is what a fix that carries the body's
displacement does. O61's verdict stands as measured; its re-use context is now this fix.**

**O65, the walk under the walking cloud** (the operator's eye on R84 seed 1: the corner from 893 s, several falls):
wall episodes 256 → 944 on seed 1 against R83, most between 800 and 1400 s with play steering; seek's targets from
sightings 11 → 10, renewals 0 in both, arrivals 9 → 5 — not seek's targets. `CloudMap.walk_things false` (the
things reduction at stops only, the mover still on the walk) is the first lever: R86 = R85 + it. The bearing bug
touched this walk too (the attended thing on the walk fed the thing and kind EPMs and the outcome loop at the
wrong range), so R84, R85 and R86 are re-measured together on the fixed bearing.

**Measured on the fixed bearing (n = 6 × 1500 s, the three arms in one sweep, the same seeds).**

| arm | walls / min | cells | seek % of the walk | rescues / min | stops / arrivals | objects touched / min | chases a run (at the moving train) |
|---|---|---|---|---|---|---|---|
| R84 (control, the fixed bearing) | **12.4 ± 9.5** | 119 | 43 | 0.06 | 18.2 / 8.3 | 22.8 | — |
| R85 = R84 + the chase | 13.0 ± 8.6 | 140 | 88 | 0.13 | 18.3 / 9.0 | 10.2 | 74.5 (3.0) |
| R86 = R85 + `walk_things false` | 20.3 ± 12.3 | 132 | 86 | 0.18 | 18.2 / 8.2 | 5.9 | 82.5 (1.7) |

- **The bearing fix is the loud result of the evening, and it is O65's answer.** R84 on the fixed bearing walks
  at 12.4 walls a minute against 31.1 on the anchor-relative bearing (the same config, the same seeds, the same
  room): seed 1, the operator's, 68.5 → 6.1 a minute, its 900–1100 s windows 150 → 25–30 wall episodes per 100 s
  with one rescue. That is below R83's 28.3 (things phase §17.52): the walking cloud, given a bearing from where
  the body is, helps the walk rather than wrecking it. `WORKING` as a fix; the walk's verdict on the walking cloud
  (O55) turns from `REGRESSION` to *better than the stack without it*, at n = 6, awaiting the eye.
- **`walk_things false` (R86) is a `REGRESSION` on the fixed bearing** (walls 13 → 20, rescues up): the things
  reduction on the walk was never the cause; the wrong range was. O65's lever is withdrawn; the switch stays,
  default true (R84's behaviour), as a lesion for tests.
- **The chase (R85), on the age gate with persistence, `NULL` for the train and a `REGRESSION` for the walk's
  ownership.** 447 chases in six runs, 73 % at static clusters, 21 % at balls and blocks, 4 % (18) at the moving
  train; the closest approach to a moving train 1.5–2.3 m by seed; seek holds the reference 88 % of the walk
  (play 12 %) because each false chase leaves a remembered target with confidence 1. Walls tie the control,
  rescues double, objects touched halve. The candidate gate does what stage 0 said it would: a young cluster that
  persists half a second is, most of the time, a static thing whose voxels are being entered for the first time as
  the body turns — and the persistence test, asked only "is a young cluster still near where I predicted", is
  satisfied by a thing that stays put. **The confirmation must ask for the other half of a change: the voxels the
  thing LEFT.** Built next (§17.55): the cast carries the sensor's origin, every returning ray marks the occupied
  off-floor voxels it passes through the core of as vacated, a cluster counts its trail within 0.25 m over the
  last second, and the candidate needs one (R87 the instrument, R88 the chase with a trail).

### 17.55 The trail: vacated voxels as the other half of a change (R87, R88; 2026-09-27 night)

**Built.** The cast token carries the sensor's own origin (three values appended; old consumers unchanged). With
`CloudMap.vacate_window_ticks` on, every returning ray is walked from the origin toward its return and an occupied
off-floor voxel it passes through the core of (within 0.3 voxel of the centre on every axis), not hit this cast, is
marked vacated; a cluster counts the vacated voxels within `vacate_radius` (0.25 m) of its centroid over the window
(`Thing::vacated`, the `mvc` record's eleventh field); `mover_vacated` makes the candidate need one. A unit test:
rays through where a cube stood, on their way to a wall, mark its voxels. Byte-identical off (R83's guard; and the
R84 instrument arm reproduces the fixed-bearing R84 to the decimal, the record differing only in the field).

**The first form marked the room.** With the ray allowed to stop 1.5 voxels short of its return, 85 % of static
clusters, 93 % of wide ones, 92 % of the balls and blocks and 85 % of the moving train's carried a "trail" — no
information at all. An oblique static surface fills its voxels partly, and a ray through the empty part reaches
its own return a voxel or two further along the surface. R88 (the chase needing a trail, on that form) is a
`REGRESSION` on every column: walls 12 → 22 a minute, rescues 0.06 → 0.45, down 0.2 → 8.6 %, 405 chases with 5
at the moving train. Withdrawn with the form.

**The second form** (`vacate_beyond_m`, 0.20): a traversed voxel is vacated only when the ray's return lies at least
0.2 m beyond it — a thing that left exposes the floor or the wall behind it, much further than the next voxel of
its own surface. Measured next on the instrument arm (R87 re-run).

**The second form, measured (R87 re-run, n = 6):** a trail on 68 % of static clusters, 66 % of wide ones, 82 % of
the balls and blocks, 81 % of the stopped train's and 80 % of the moving train's. The gate "hit-weighted age under
0.3 s and a trail": recall 0.43 on the moving train with 97 false alarms a minute (25 within 1.5 m), against 0.46 and
128 (34) for the age alone — a quarter fewer, not a discriminator. **Verdict: the trail at this resolution is
`NULL`.** A 4 cm voxel is partly filled by any surface that crosses it, and a ray through its empty part reaches
the floor or the wall behind a chair leg 0.2 m further as readily as it reaches the floor behind a train that has
gone. Re-use: finer voxels near the body, or a trail counted at the CLUSTER level (most of a cluster's former
voxels traversed, not any one), or the real sensor's multi-target returns. The code stays, default off.

**The next arm, R89 (the chase asked to see motion):** `BearingSeekLoop.chase_min_v` 0.1 — a candidate is chased
only if its velocity and its displacement per second watched are both at least 0.1 m/s (a young thing that stays
put is a thing newly in view); `chase_v_max` 0.6; `CloudMap.mover_ext_max` 0.35 (a measured retreat from "any
size": the wide clusters are wall bases whose visible part slides with the view) and `mover_range` 1.2.

**R89 measured (n = 6, against R84 on the fixed bearing and R85).** Chases 447 → **72** in six runs (12 a run, 1.3 s
each, 18 s of chasing a run): 65 at static clusters, 3 at balls and blocks, 1 at the stopped train, **3 at the
moving train**; the closest approach to a moving train 0.9–1.9 m by seed (3.2 on seed 3). The walk's ownership is
back: seek 88 → 58 % (control 43 %), play 12 → 42 %; walls 18.0 ± 17.0 against 12.4 ± 9.5 (one wall seed each
way; a tie at this power), rescues 0.12, stands and stops the control's. Mover candidates seen by the cloud
1 100–1 800 a run (R85: 1 900–3 600).

**Stage 1's verdict.** The chase as a loop is `WORKING` mechanically (a moving thing that follows its own prediction
is chased, the unit tests and the seed-1 smoke), and `NULL` for the train at this power in this room: three chases
of the moving train in six runs, on every gate tried. Two causes, separable:
1. **Opportunity.** The train is a cluster of the walking cloud on 65–80 % of casts within 1.5 m (§17.53), but the
   walk brings it within 1.2 m of a moving train seldom: 681 moving-train cluster samples in six runs against
   200 000 static ones. The track sits in a corner of the room the walk visits little, and the chase's range gate
   is the last metre. A stimulus that comes to the duck — a track through the middle, or the ball rolled across
   the WALK as `--roll-past` rolls it across a stop — is the (d) test this needs.
2. **The room's own young clusters.** Every gate measured leaves a stream of static candidates: the age (167 a
   minute), the age with persistence (74 chases a run), the trail (`NULL` at 4 cm voxels), motion (12 chases a
   run, 90 % static). What remains after "young, persistent, compact, near and moving" is a fragment of the room
   sliding into view as the body turns, which moves in the odometry frame by construction. Its remover is
   geometric — the sliding is along the surface it belongs to and correlated with the body's own yaw rate — or
   T6 proper at a resolution the sensor supports (the real VL53L8CX reports several targets a zone, which is a
   thing behind a thing, and would give the trail directly).

Re-use for the next design discussion: keep R89's chase (the motion requirement is what returned the walk); bring
the stimulus to the walk before touching the gate again; and treat the sliding fragment as the one confound left.

### 17.56 The big track: the stimulus brought to the walk (R84, R89 on the redesigned room; 2026-09-27, late)

**The operator's eye on R89:** "the wall stuck issue is mostly resolved. However, the robot is not really interacting
with the train at all." Their redesign: the track larger, wider and longer, stretching almost the whole way between
the green wall and the wall across from it; the train larger, like the purple block; the train over the ball for its
regular schedule, known velocity and control by the seed. Built (`playroom_gen.py --train`, the plain room
byte-identical): a 1.52 × 0.80 m oval centred on the room, its long axis along y, laid first with the furniture and
the things placed clear of it; an 18 × 10 × 10 cm train; perimeter 7.48 m, so a run of 8 s at 0.2 m/s covers a
fifth of a lap.

**Measured (n = 6, R84 and R89 together on the new room).** The opportunity tripled: the train is in the ToF's cone
within 1.5 m while moving on 4 745 walking casts (the corner track: about 1 600), on 31–49 % of which a cluster lies
within 0.22 m of its centre (the corner track's 65–80 %: the longer train and its smear put the centroid further
from the centre, partly the label's radius). R84 walks at 14.7 ± 10.1 walls a minute (12.4 on the corner track: a
tie), cells 139, objects touched 4.6 a minute. **R89: 71 chases in six runs, 8 at the moving train (11 %, from 4 %),
5 at the stopped train, 54 at static clusters (76 %)**; the closest approach to a moving train 0.8–2.6 m by seed;
walls 18.1 ± 20.0 (seed 2 at 54.9), rescues 0.25 against 0.13, arrival stops 12.5 against 8.5 (the false chases'
remembered targets), seek 58 % of the walk. **Verdict: `PARTIAL` on the train, `REGRESSION` on the walk at this
power** — bringing the stimulus to the walk doubled the train's chases without touching the false ones, so the
false ones are the design's own, not the room's. The sliding fragment (O64) is the mechanism to remove before the
chase can be judged; the room is now right for judging it.

### 17.57 A position belongs to a thing while it is still (R90; 2026-09-28)

**The operator's eye on R89, with the new cloud view:** "I observed the robot measure the train passing while
standing. Then it proceeded to the location where the train was during the measurement (smear of voxels) and
pecked in that location. The train was no longer there. Our brain should be learning the difference between
stationary and moving objects." The gap was the chase's own: a chase that lost its sightings kept the last predicted
position as a remembered target ("where it stopped is where to go and look"), and the thing had not stopped, it had
left the cone; one level down, a passing thing's short smear at a stop reads as a small thing and the seek loop
fixes its place.

**Built, one principle in three parts (R90 = R89 + these; each guarded, R83's md5 unchanged):**
`BearingSeekLoop.chase_stop_v` 0.05 — a chase that ends with the thing still moving is dropped, only a thing that had
slowed is remembered where it stopped; `CloudMap.things_skip_movers` — the things reduction never attends a cluster
whose voxels are young by the mover rule; `--stop-on-chase` — a stop ends when a chase is confirmed, so the walker
follows (the orienting reflex's substrate form). Two unit tests. The viewer's chase marker is now a yellow arrow.

**Measured (n = 6, the big track, R84 the control in the same sweep).**

| | R84 | R89 (motion, the corner-track gate) | **R90** |
|---|---|---|---|
| walls / min | 14.7 ± 10.1 | 18.1 ± 20.0 | **6.5 ± 3.8** |
| rescues / min · down % · stop rescues | 0.13 · 0.36 · 0.33 | 0.25 · 0.52 · 0.67 | **0.06 · 0.10 · 0.17** |
| stops / at a thing / open floor (a run) | 17.8 / 8.7 / 6.5 | 21.8 / 10.0 / 8.5 | 22.3 / **12.7** / 6.8 |
| arrival stops | 8.5 | 12.5 | 13.0 |
| chases (at the moving train) in six runs | — | 71 (8) | 67 (8) |
| stops ended on a chase, six runs | — | — | 12 |
| skills · with no thing within 0.5 m | 29 · 8 | 44 · 12 | 45 · 15 |
| seek % of the walk | 39 | 58 | 52 |

- **The walk: `WORKING`, loud.** Walls halve against the control and fall to a third of R89's, with the spread
  collapsing (3.8 against 20); rescues halve, falls a third, stop rescues halve. Dropping a lost chase instead of
  remembering it removed the false targets that had sent the duck into walls, and the stops at things rose 8.7 → 12.7
  a run. Every seed under 12 walls a minute.
- **The gap, in part.** Open-floor stops are the control's (6.8 against 6.5, R89's 8.5); but skills fired with
  nothing within half a metre are 15 of 45 — the seek loop still reaches places a thing has left. The remaining
  cases are not the chase's: a thing seen standing at a stop (the train in its eight seconds still, a ball it
  kicked) and walked to after it moved. That is the (d) test's own answer, and the remedy is the operator's
  sentence: the brain has to *learn* that a kind of thing moves.
- **The chase itself** is unchanged by this lever: 67 chases, 8 at the moving train (12 %), the closest approach to
  a moving train 0.6–1.6 m by seed; 12 stops ended on a confirmed chase in six runs.

**Verdict.** R90 `WORKING` on the walk and `PARTIAL` on the gap; not promoted without the eye. **Re-use / the next
lever:** the age of a cluster's voxels into the thing descriptor, so the kind EPM earns a *moving kind* and the
outcome loop learns that its intents get no answer from one — habituation then ends the pecks at places, by the
brain's own account rather than a rule.

### 17.58 Isolation, and the age in the descriptor (R91, R92, R93; 2026-09-28)

**The operator's two asks, with the new cloud view.** "Go ahead with the voxel age in the descriptor. I am still
seeing the target ring skipping around and often choosing walls, especially later in the run. I wonder if we can
predict a blob of voxels is part of a larger object by the proximity of other voxels in its area, especially those
groups that are higher than our small target objects? Smaller objects will be isolated into low blobs. A ball under a
table gets lost, but it could keep the focus off the walls, which is the bigger issue." Two levers, two arms on R90.

**Built.** `Thing::tall_near` — the open cloud's voxels with mean height at or above `iso_height` (0.25 m) within
`iso_radius` (0.25 m) of the cluster's centroid; `mover_isolated` and `things_isolated` require it to be zero for
the candidate and for the attended thing; `things_age_dim` appends the cluster's hit-weighted voxel age against
the oldest cluster's to the descriptor (the thing and kind EPMs read nine values). A unit test: a cube at the foot
of a post has tall voxels near it and is passed over for a lone cube further away. The `mvc` record carries
`tall_near`, so the instrument can score isolation offline.

**Isolation as a measurement (R91's logs, the stage-0 scorer).** The share of clusters with nothing tall within
0.25 m: static 0.06, wide 0.01, balls and blocks 0.64, the stopped train 0.92, **the moving train 0.95**. The gate
"hit-weighted age under 0.3 s AND isolated": recall 0.44 on the moving train with **13 false candidates a minute**
(5.6 within 1.5 m) — against 128 (34) for the age alone (§17.53). **This is the discriminator stage 0 lacked, and it
is the operator's.** The sliding fragment (O64) is a fragment of something tall.

**Measured (n = 6, the big track, R84 and R90 as the controls, the same seeds).**

| | R84 | R90 | R91 = R90 + isolation (mover and thing) | R92 = R90 + the age dim |
|---|---|---|---|---|
| chases (at the moving train), six runs | — | 67 (8) | **19 (8, 42 %)** | 69 (9) |
| walls / min | 14.7 ± 10.1 | 6.5 ± 3.8 | 18.9 ± 13.0 | 11.5 ± 4.5 |
| rescues / min · down % | 0.13 · 0.36 | 0.06 · 0.10 | 0.15 · 0.51 | 0.09 · 0.22 |
| seek % of the walk | 39 | 52 | 37 | 52 |
| stops / at a thing / arrivals | 17.8 / 8.7 / 8.5 | 22.3 / 12.7 / 13.0 | 17.8 / 8.7 / 7.5 | 22.2 / 11.7 / 12.8 |
| skills · at nothing | 29 · 8 | 45 · 15 | 28 · **5** | 44 · 15 |
| the train's kind, modal share | — | 0.90 | — | 0.70 |

- **R91: the chase came clean and the walk went back.** Chases 67 → 19 in six runs, the moving train 12 → 42 % of
  them, static targets 49 → 7; skills at nothing 15 → 5. But `things_isolated` also refused the stops' targets
  near furniture and walls (the room's balls and blocks stand within a few voxels of them), seek fell to 37 % of
  the walk, and the walls returned to R84's, bimodal (seeds 3 and 4 at 38 and 32 a minute). `WORKING` on the
  candidate, `REGRESSION` on the thing: the two uses of one measure pull apart, so **R93 = R90 + `mover_isolated`
  alone** (in flight).
- **R92: the age in the descriptor is `NULL` at this power.** Chases, stops, skills at nothing all R90's; walls
  11.5 against 6.5 (within one spread); and the kind vocabulary did not sharpen — the train's modal share fell
  0.90 → 0.70 while the blocks stayed at 0.58–0.66. One value in nine under a random projection is the small
  signal on a large common mode of CLAUDE.md §0 rule 2: the shape dims own the vocabulary. Re-use: a vocabulary
  of its own over [age ratio, fresh] (two dims, two or three nodes: still, moving, just stopped) feeding the
  outcome table as a second key, rather than one more column in the shape's.

**R93 = R90 + `mover_isolated` alone, measured (n = 6).** Chases 67 → **25** in six runs: 8 at the moving train
(32 %), 6 at the stopped train, 4 at balls and blocks, 7 at static clusters (28 %, from 73 %); the closest approach
to a moving train 0.46–0.95 m on the four seeds that chased it; 6 stops ended on a chase. Skills 24, at nothing 6.
**The chase is the cleanest of the phase: `WORKING` on the candidate, and the operator's ask — the ring off the
walls — is met.** The walk, though, is a `REGRESSION` at this power: walls 24.0 ± 14.2 (R84 14.7 ± 10.1, R90 6.5 ±
3.8), three wall seeds at 22, 37 and 45 a minute; stands 24 % (R90 38), stops 16.5 (22.3), arrivals 6.7 (13.0).

**What that says about R90.** R90's halved walls came with 13 arrival stops a run and 38 % standing; R93 differs
from it only in refusing the candidates at the foot of tall things, and it loses six arrival stops a run with them.
So a good part of R90's walk was **false chases turning into stops**: a young fragment at a wall base chased for a
second, "stopped" by the loop's own account (its velocity under `chase_stop_v`), remembered, walked to, stood at.
Standing is not wall contact, so the walls fell. An interesting artifact (CLAUDE.md §6): R90's number is real and
its cause is not the chase working. The clean chase (R93) walks like the control does, bimodally, and at n = 6 the
control's own spread (10) covers most of the difference.

**Verdict.** R93 `WORKING` on the chase, `PARTIAL` on the walk (a tie with the control within the spread, a
regression against R90's artifact); not promoted without the eye. O64 (the sliding fragment) is answered by
isolation: it is a fragment of something tall. **Next:** the walk itself — the stops are what keep the duck off
the walls, and a chase that ends at a moving thing gives none; the stop's trigger, not the chase's gate, is the
lever (O43's question returns: what should START a stop, when the thing chased is gone).

### 17.59 A lost chase starts a look, not a walk (R94 = R93 + `--stop-on-lost`; 2026-09-29)

**Built.** The seek loop reports the tick a chase is dropped with the thing still moving, with the bearing and range
of where the thing was last predicted (`chase_lost_now / _ego / _range`); the host's `--stop-on-lost` starts a
stop on it, the sweep's yaw centred on that bearing and its pitch band on that range, as the arrival stop centres
on the reached thing; with `--stop-on-chase` already on, a chase confirmed during the look ends it — chase, lose,
look where it went, chase again. A unit test. The record's event reads `stop:lost`. Paired against R93 in one
sweep (`--host-arm`), the same seeds.

**Measured (n = 6, the big track).** The lever fired 8 times in six runs (1.3 a run: seeds 1, 2, 4, 6; none on 3
and 5, whose records are byte-identical between the arms — the guard in the data). Chases 25 → 23, at the moving
train 8 → **10 (43 %)**, static 7 → 8; the closest approach to a moving train under a metre on three seeds. The walk,
paired by seed (walls a minute, R93 → R94): 22.4 → **7.0**, 9.4 → 8.5, 9.6 = 9.6, 44.6 → **172**, 21.4 = 21.4, 36.6 →
**2.7**; the mean 24.0 → 36.9 ± 66.6 is one seed, the median 22 → 9. Rescues 0.17 → 0.09, falls 0.34 → 0.17 %, stands
24 → 30 %, stops 16.5 → 17.8. Seed 4 — already R93's worst — rides the walls from 700 s and sits in the −x −y corner
from 1000 s to the end (300–550 wall episodes per 100 s), two of the run's lost looks taken there: the corner trap
of O65's kind, which the stuck stop and escape (R64s, §17.44) exist for and this argv does not carry.

**Verdict.** The mechanism `WORKING` (a look at where the thing went, from the loop's own loss); on the chase a
`PARTIAL` (43 % of chases at the moving train, the best of the phase, at n = 6); on the walk a *signal*, not a
finding: three seeds loud in its favour, one corner trap, and eight events a run's worth of divergence — CLAUDE.md
§3.3's own warning about a lever that fires rarely. Not promoted without the eye (preset R94, seed 1: three lost
looks). **Re-use:** the corner trap is the stuck stop's to answer (`--stop-on-stuck`, `--stuck-escape`), not this
lever's; and a look that finds nothing should count as an answer to the outcome table one day.

### 17.60 The campaign, sweep 1: the loaded brain, the stuck stop, permanence, the pull, and two leans (2026-09-29)

**The harness** (chase phase §8): `--load-brain` for level 2 from `checkpoints/duck_r94_s1.brain.json`, the body at its
reset, stops from the first second, 600 s a run, the judged window from 30 s; six arms on six seeds in one sweep.
The loaded brain walks from the first second (vx 0.22 m/s mean in the first two minutes, against the babble's 0.00).

| arm (600 s, n = 6) | walls / min | rescues / min · down % | stops · at a thing · open | chases (train) · lost looks | skills · at nothing | walk m/s |
|---|---|---|---|---|---|---|
| R94, the whole brain restored | 41.7 ± 46.2 | 0.20 · 0.29 | 11.3 · 5.2 · 4.5 | 9 (2) · 2 | 17 · 5 | 0.165 |
| + `--stop-on-stuck 8 --stuck-escape 6` | 30.8 ± 18.5 | 0.18 · 0.53 | 13.8 · 6.3 · 4.7 | 10 (3) · 4 | 21 · 6 | 0.174 |
| R95 permanence (3 s) | 37.2 ± 49.0 | 0.18 · 0.23 | 10.7 · 4.3 · 5.3 | 10 (3), 1 re-acquired · 2 | 13 · 6 | 0.166 |
| R96 the pull's decay | = R94 | = | = | = | = | = |
| `--head-forward 0.15` | 34.5 ± 28.0 | **0.60 · 1.25** | 11.7 · 5.0 · 5.8 | 17 (5) · 7 | 15 · 7 | 0.162 |
| `--body-pitch 0.1` | 22.9 ± 10.8 | **0.87 · 7.36** | 14.2 · **9.8** · 3.8 | 9 (2) · 3 | **29** · 6 | 0.174 |

- **The whole restored brain is the wrong harness.** Walls 42 ± 46: a saved run's map, play field, cloud cache and
  outcome table are keyed to *its* odometry frame and *its* habituation; put into a body at the origin they place
  the duck in a room it believes it knows from somewhere else. The babble is the intent EPM's alone, so
  `--load-brain-modules` now restores the identification only by default (sweep 2 re-measures every arm on it).
- **The stuck stop and escape**: walls 42 → 31 with the spread halved, stops up, the chases the same — a signal in
  the right direction, to be re-read on the corrected harness.
- **The leans buy falls, not speed.** Head forward 0.15 rad: rescues ×3, speed unchanged; body pitch 0.1: falls
  7.4 % of ticks (one seed past 15 %), speed +5 %, and — the interesting artifact — the most stops at things and
  skills of the sweep (the pitched trunk arrives at things). Sweep 2 tries both at half the angle, and spends the
  speed where the operator wants it: `--chase-vx 0.35` raises the forward command only while a mover is chased.
- **Permanence and the pull are unmeasurable at 1.5 chases a run**: two losses in six runs, one re-acquisition;
  the pull's arm is the base's run to the decimal (a need of 0.6 still wins the arbiter). Sweep 2 adds the
  mover's reach at 1.5 m (R97) for more starts, and both ride the corrected harness.

### 17.61 The campaign, sweep 2: every arm on the identification-only brain (2026-09-29)

The harness corrected (`--load-brain-modules motor_epm_intent` by default): the intent EPM's identification
restored, the map, play field, cloud cache and outcome table fresh; the first two minutes a walk at 0.17 m/s with
three stops. Eight arms, six seeds, 600 s, the same seeds.

| arm | walls / min | rescues / min · down % | stops · at a thing | chases (moving train) · lost looks | skills | walk m/s |
|---|---|---|---|---|---|---|
| R94 base | 27.2 ± 23.6 | 0.62 · 3.71 | 13.5 · 8.3 | 17 (5) · 10 | 21 | 0.166 |
| **+ `--stop-on-stuck 8 --stuck-escape 6`** | **18.7 ± 18.5** | **0.23 · 0.34** | 16.0 · 9.2 | 12 (1) · 5 | 27 | 0.177 |
| **+ `--chase-vx 0.35`** | 24.3 ± 14.1 | 0.45 · 1.26 | 13.7 · 7.3 | 16 (**6, 38 %**) · — | 23 | 0.176 |
| + `--body-pitch 0.05` | 25.6 ± 15.8 | 0.20 · 0.60 | 14.0 · 6.5 | 12 (3) · 5 | 27 | 0.173 |
| + `--head-forward 0.08` | 28.8 ± 19.9 | 0.35 · 0.41 | 10.5 · 4.0 | 9 (3) · 3 | 11 | 0.168 |
| R95 permanence (coasting 3 s) | 44.5 ± 52.0 | 0.42 · 1.39 | 11.3 · 5.5 | 11 (5), 0 re-acquired · 7 | 13 | 0.168 |
| R96 the pull's decay | = base | = | = | = | = | = |
| R97 the reach at 1.5 m | 23.2 ± 23.4 | 0.68 · 3.91 | 12.0 · 7.0 | 22 (4; 13 static) · 8 | 15 | 0.166 |

- **The stuck stop and escape: `WORKING`.** Rescues a third of the base's and falls a tenth, walls 27 → 19,
  stops and stops at things up, more standing (30 % of the run). The cost: fewer chases and one at the moving
  train — the stuck stop fires where the chase would have started, or the escape holds the reference. Kept.
- **The pursuit at speed (`--chase-vx 0.35`): a signal.** The moving train 29 → 38 % of chases, objects
  touched up, walls a little down; falls between. Kept, and sweep 3 tries 0.4, the walker's trained top.
- **The leans, at half the angle: `NULL`.** Body pitch 0.05 no longer falls (0.60 %) and gains 4 % of speed;
  head forward 0.08 changes nothing. The operator's centre-of-gravity idea does not turn into speed on this walker:
  the trained policy holds its own pitch, and a commanded lean is either absorbed or, past 0.1 rad, paid in falls.
  Body pitch 0.05 rides along in sweep 3's three-lever stack for one more look.
- **Permanence by coasting: `REGRESSION` in this context.** One seed at 150 walls a minute: after the train
  leaves the cone the coasting target leads across the track toward the wall it went behind, and no re-sighting
  ever came (0 of 11). A moving thing's predicted path in a room with walls needs the map to say where a thing
  can go; without it the look (R94) is the safer permanence. Re-use: coasting bounded by the cloud's free space.
- **The pull's decay: `NULL`** — the base's run to the decimal on every seed; a need of 0.6 still wins the
  arbiter, so no decision changed. Re-use: a decay that reaches the arbiter's margin, or the outcome table.
- **The reach at 1.5 m: `NULL`** — chases 17 → 22, all of the gain static (8 → 13), walls the same. The stage-0
  finding holds: past 1.2 m the false candidates grow faster than the true ones.

### 17.62 The campaign, sweep 3: the stack (2026-09-29)

The two keepers together on the base's seeds, plus the pursuit at the walker's top and the three-lever form.

| arm (600 s, n = 6) | walls / min | rescues / min · down % | stops · at a thing · arrivals | chases (moving train) | skills · answers | walk m/s |
|---|---|---|---|---|---|---|
| R94 base | 27.2 ± 23.6 | 0.62 · 3.71 | 13.5 · 8.3 · 5.3 | 17 (5, 29 %) | 21 · 8 | 0.166 |
| **the stack**: stuck stop and escape + chase vx 0.35 | **18.7 ± 19.5** | **0.20 · 0.34** | 15.0 · 8.2 · 6.7 | 13 (5, 38 %) | 24 · 7 | 0.176 |
| the stack, chase vx 0.4 | 20.1 ± 19.4 | 0.18 · 0.32 | 14.2 · 7.5 · 5.2 | 9 (4, 44 %) | 18 · 6 | 0.173 |
| the stack + body pitch 0.05 | 21.3 ± 19.2 | 0.22 · 0.28 | **17.5 · 10.5 · 9.8** | 13 (6, 46 %) | **35 · 18** | 0.176 |

- **The stack holds both keepers' gains**: walls 27 → 19, rescues a third, falls a tenth, the moving train 29 → 38 %
  of chases, objects touched 8 → 13 a minute, the walk 6 % faster. The corner trap is not gone — one seed in each arm
  still rides a wall for minutes (the base's seed 4 at 72, the stack's at 56, another seed in each variant) — the
  stuck stop shortens it and moves it, it does not prevent it.
- **The pursuit at 0.4** buys a purer chase (44 %) with fewer chases; a tie with 0.35 on everything else. 0.35 stays.
- **Body pitch 0.05 on the stack** is the interaction-rich variant: the most stops at things, arrivals, skills (35)
  and answers observed (18) of the campaign — the pitched trunk arrives at things and its pecks land — at the same
  falls. `NULL` alone, a `PARTIAL` on the stack; for the eye, not for the stack by numbers.

**The stack goes to a confirmation on twelve seeds the signal never used (7–18), 600 s, against the base:
n = 18 paired in all (§17.63).**

### 17.63 The campaign, sweep 4: the stack confirmed on twelve seeds it never saw (2026-09-29)

The stack (`--stop-on-stuck 8 --stuck-escape 6 --chase-vx 0.35` on R94, the identification-only brain, 600 s)
against the base on seeds 7–18, paired; then pooled with sweeps 2–3's seeds 1–6: **n = 18**.

| | base, seeds 7–18 | stack, seeds 7–18 | base, n = 18 | stack, n = 18 |
|---|---|---|---|---|
| walls / min, mean ± sd | 45.0 ± 46.7 | 23.1 ± 16.2 | 39.0 ± 39.4 | **21.6 ± 16.4** |
| walls / min, median · worst seed | 18.4 · 127 | 21.1 · 50 | 22.5 · 127 | 18.2 · **56** |
| seeds over 60 walls a minute (a wall ridden for minutes) | 5 | 0 | **6** | **0** |
| rescues / min | 0.26 | 0.27 | 0.38 | 0.24 |
| stops · at a thing · arrivals (a run) | 15.0 · 7.1 · 8.3 | 17.0 · 9.2 · 9.5 | | |
| chases (moving train), twelve runs | 26 (6, 23 %) | 23 (8, 35 %) | | |
| skills · answers observed, twelve runs | 54 · 27 | 64 · 30 | | |
| stand % · walk m/s | 20 · 0.165 | 28 · 0.177 | | |

**Verdict: the stack is `WORKING` on what the operator asked for first — not being stuck against walls — and a
signal on the rest.** The tail is the finding: six of eighteen base runs ride a wall for minutes (60–127 episodes a
minute); no stack run does, the worst at 56 and the spread halved. The median ties (a run that never traps is the
same run either way; the stack is better on 12 of 18). Rescues 0.38 → 0.24 pooled, a tie on seeds 7–18 where the
base fell little. The pursuit: the moving train a third of chases on both seed sets against a quarter and a fifth,
objects touched, stops at things, skills and answers all up, the walk 7 % faster, more standing (the stuck stop's
looks). Not promoted without the eye — preset "R94 · loaded + the STACK". What the stack does not do: the trap is
shortened and moved, not prevented (a run at 50 walls a minute remains), and the chase count is unchanged at two a
run in ten minutes — the pursuit is purer, not more frequent.

### 17.64 The campaign, sweep 5: permanence in recognition on the stack (2026-09-29), and the campaign's close

R98 = R94 + `chase_memory_ticks 250`: a lost mover kept in mind for 5 s (its last predicted position and velocity,
extrapolated) without driving the walk; one mover sighting within the chase gate of where it should now be
re-acquires the chase at once. On the stack, six seeds, 600 s, against the stack.

| | the stack | + recognition permanence (R98) |
|---|---|---|
| walls / min · rescues / min · down % | 18.7 · 0.20 · 0.34 | 18.4 · 0.22 · 0.36 |
| stops · at a thing · arrivals | 15.0 · 8.2 · 6.7 | 15.0 · 7.8 · 6.5 |
| chases (moving train) | 13 (5, 38 %) | 13 (**7, 54 %**), 1 re-acquired |
| answers observed · skills at nothing | 7 · 6 | 9 · 6 |

**Verdict: a signal, safe.** The walk is the stack's to the decimal; the chases are the same in number and purer
(the moving train 38 → 54 %), one re-acquisition in six runs, two more answers observed. The mechanism fires
rarely because the chase fires rarely (two a run), which is the campaign's standing limit. Not in the stack
without the eye; preset R98.

**The campaign, closed (2026-09-29).** The operator's four items, as they stand:
1. **The babble is gone from the observation runs.** A saved brain's identification alone is restored
   (`--load-brain`, `--load-brain-modules motor_epm_intent`), the map and the loops fresh; the first ten minutes are
   a walk from the first second. The whole restored brain was measured and refused (§17.60).
2. **The head over the centre of gravity does not buy speed on this walker.** A commanded lean is absorbed
   (0.05 rad: +4 % of speed, no falls) or paid in falls (0.1 rad: 7 % of ticks down); the speed that helps is the
   pursuit's own, `--chase-vx 0.35` (§17.61–17.62).
3. **Object permanence:** coasting after a lost mover is a `REGRESSION` (it leads into walls, §17.61); permanence
   in recognition is a safe signal (this section) beside the look (§17.59). The real form the operator named — a
   slow loop that seeks a target that has left the field of view — wants a prediction of where a thing *can* go
   (the map's free space) and, for the train, its schedule; that is the design discussion for the camera boundary.
4. **The two levers proposed at the last hand-off:** the stuck stop and escape `WORKING` (no run rides a wall for
   minutes, n = 18); a look that finds nothing as an answer `NULL` in the pull's form (no decision changed) — its
   re-use is the outcome table.
The stack for the eye: **R94 + `--stop-on-stuck 8 --stuck-escape 6 --chase-vx 0.35` on the identification-only
brain** (preset "R94 · loaded + the STACK"); the interaction-rich variant with body pitch 0.05 and the recognition
permanence (R98) are the two arms worth a look beside it.

### 17.65 The campaign, sweep 6: the mover's priority, the follow beyond the start range, the gaze (2026-09-29)

**The operator's eye on the campaign's presets:** "I did see the robot start to pursue the train in some occurrences,
but then would get distracted by the block that was lying nearby. We should definitely be prioritizing the moving
objects. … plenty of occasions where the moving train could move across the duck's visual path, but because the
duck is performing some other tasks, such as walking, it doesn't notice … the robot should be able to turn its head
while it's walking in order to try to reacquire the moving target."

**Measured first, on the stack's logs.** While a chase is active, seek holds the heading on every tick (1 056 of
1 056): the arbiter is not where the mover loses. In the five seconds after a chase ends a static target is held on
87 % of ticks — the block takes the train's place the moment it is lost. And the train crosses the cone (within
1.5 m and 0.4 rad of the head's look, moving, for a second or more) 22 times in six runs; a candidate follows on
17–22 of them, a chase on 5. The crossings are noticed; the confirmation is where three in four die.

**Built.** `BearingSeekLoop.chase_memory_holds`: while a lost mover is in mind (R98's five seconds) no new static
target is taken (R99 = R98 + it). `CloudMap.mover_range_hold` 2.5 m: a mover already published stays a candidate
beyond the 1.2 m start range when a young, isolated cluster lies within 0.4 m of where it would now be (R100 = R99
+ it). `--chase-gaze GAIN`: on the walk the head yaw target is offset toward the chased target or the lost mover's
memory (±0.7 rad). Two unit tests; the scorer now reports the post-chase static share and the crossings, candidate
and chased.

| arm (the stack, loaded brain, 600 s, n = 6) | walls / min | rescues · down % | chases (moving train) | crossings: candidate · chased | after-chase static share |
|---|---|---|---|---|---|
| R98 the stack + recognition permanence | 18.4 ± 19.7 | 0.22 · 0.36 | 13 (7) | 21: 17 · 5 | 0.85 |
| **R99 + the mover's priority** | **10.3 ± 6.8** | **0.12 · 0.22** | 14 (6) | 22: 19 · 6 | 0.86 |
| R100 + the follow to 2.5 m | = R99 | = | = | = | = |
| R100 + `--chase-gaze 1.0` | 16.1 ± 12.8 | 0.18 · 0.36 | 10 (3) | 14: 13 · 4 | 0.96 |

- **The mover's priority is the campaign's best walk**: walls 18 → 10 with every seed under 24 (5.6, 6.0, 7.5, 7.4,
  11.9, 23.3), rescues and falls halved, stops and arrivals up. The swerve to the block was costing walls. The
  post-chase static share does not fall because most chases end with the train *stopped* (its eight still seconds),
  and a stopped thing's place is a legitimate target; the priority only bars a new static target after a loss.
- **The follow beyond 1.2 m changed no decision** (R100 = R99 to the decimal): the train receding past the start
  range is not how chases end at this speed.
- **The gaze at gain 1: `NULL` here** — fewer chases, fewer crossings counted (the head follows the train, so each
  crossing is one long one), objects touched down. A moving cone on the walk may unsettle the cloud the candidate is
  read from; and the head brain's own loop is written over. Re-use: a smaller gain, or the gaze only while a memory
  lives, measured with the head-window replay.
- **The confirmation is the loss.** Sweep 7 turns each of its three knobs alone on R99 — the gate 0.35 → 0.5 m,
  the motion 0.1 → 0.05 m/s, the wait 0.5 → 0.3 s — with the loop now counting why candidates fail (replaced,
  too fast, still, timed out).

### 17.66 The campaign, sweep 7: the confirmation's three knobs, and why candidates fail (2026-09-29)

The loop now counts why a candidate was not chased. On R99, six runs: **replaced 0, too fast 0, still 240, timed out
161.** No candidate ever missed the gate or implied too high a speed — so the gate at 0.5 m (R101) is R99's run to
the decimal. Candidates fail the motion test (their displacement per second watched under 0.1 m/s at confirmation)
or get no second sighting within a second.

| arm (R99 + one knob, 600 s, n = 6) | walls / min | rescues · down % | chases (moving train) | still · timed out |
|---|---|---|---|---|
| R99 | 10.3 ± 6.8 | 0.12 · 0.22 | 14 (6, 43 %) | 240 · 161 |
| R101 gate 0.5 m | = R99 | = | = | = |
| R102 motion 0.05 m/s | 12.8 ± 5.0 | 0.23 · 0.60 | 18 (6, 33 %) | 176 · 169 |
| R103 wait 0.3 s | 21.7 ± 18.0 | 0.15 · 0.27 | 12 (4, 33 %) | 586 · 159 |

- **The motion test is doing its work**: relaxing it (R102) admits four more chases, none of them the train, and
  costs walls and rescues; shortening the wait (R103) makes it fail more (586) and the walls double. The still
  candidates are mostly the static room, rightly refused.
- **Where the train is lost:** its displacement over the 0.5 s watch is 0.1 m — the threshold itself — and the
  window centroid jitters by that much, so the train passes the test about half the time. The remedy is not less
  motion but a longer watch: over a second the train travels 0.2 m and the jitter does not grow. Sweep 8: R104
  `chase_confirm_ticks 50`, and R105 the same with the motion at 0.08 m/s.

### 17.67 The campaign, sweep 8: the longer watch (2026-09-29)

| arm (R99 + the watch, 600 s, n = 6) | walls / min | rescues · down % | chases (moving train) | still · timed out |
|---|---|---|---|---|
| R99, the watch at 0.5 s | 10.3 ± 6.8 | 0.12 · 0.22 | 14 (6) | 240 · 161 |
| R104, the watch at 1 s | 14.5 ± 9.7 | 0.15 · 0.35 | **3 (0)** | 38 · 159 |
| R105, at 1 s with the motion at 0.08 m/s | 14.5 ± 9.6 | 0.18 · 0.42 | 2 (0) | 32 · 141 |

**`REGRESSION`.** A candidate that must be sighted for a full second before it is chased is almost never chased: the
train's sightings on a crossing come and go with the candidate gate's own flicker (isolation, the 1.2 m start
range, the window's clusters splitting and merging), and the confirmation is only evaluated when a sighting
arrives. The 0.5 s watch with the motion at 0.1 m/s stands. **The standing limit of the pursuit is now measured
from both sides:** a crossing becomes a candidate nine times in ten and a chase one in four; the loss is the motion
test on a displacement (0.1 m in 0.5 s) that sits at the centroid's own jitter, and neither a looser test (R102)
nor a longer look (R104) buys the train without buying the room or losing the candidate. **Re-use, the substrate
form:** the smear itself carries the velocity — the window's voxels are fresh at the leading edge and older at
the trail, so the vector from the oldest voxels' centroid to the freshest voxels' centroid over their age
difference is a single-cast velocity with no tracker and no wait; a static cluster's age gradient is random. That
is the next lever for the chase's start, and it is the cloud's to compute.

### 17.68 The campaign, sweep 9: the gaze at smaller gains, and the campaign's second close (2026-09-29)

| arm (R99 + the gaze, 600 s, n = 6) | walls / min | rescues · down % | chases (moving train) | crossings: candidate · chased |
|---|---|---|---|---|
| R99 | 10.3 ± 6.8 | 0.12 · 0.22 | 14 (6) | 22: 19 · 6 |
| `--chase-gaze 0.5` | 12.5 ± 6.3 | 0.20 · 0.62 | 11 (4) | 18: 17 · 6 |
| `--chase-gaze 0.3` | 15.8 ± 5.9 | 0.20 · 0.48 | 12 (5) | — |

**`NULL` at every gain** (1.0 in §17.65, 0.5, 0.3): the head turned toward the chase on the walk buys no chases and
costs a little of the walk. The form is wrong, not the idea: a yaw offset written over the head brain's own targets
fights the loop that keeps the head level and steady, and a cone that turns while the body turns unsettles the
walking cloud the candidate is read from. Re-use: the gaze as an intent the head brain fulfils (its own prediction
to keep the mover centred), and the head's replay window to see what it did — not a host offset.

**The campaign's second close (2026-09-29).** The stack for the eye is **R99 on the identification-only brain**:
R94 + `--stop-on-stuck 8 --stuck-escape 6 --chase-vx 0.35` + `chase_memory_ticks 250` + `chase_memory_holds` —
walls 10.3 ± 6.8 a minute with every seed under 24 (the campaign's base: 27 ± 24; the original R94 from scratch:
24 ± 14 on the corner track's numbers), rescues 0.12, the moving train 43 % of chases, stops at things and arrivals
the campaign's highest (preset "R99 · the MOVER's PRIORITY"). What the operator asked for and what stands:
- **prioritise moving objects** — done in the loop (the priority) and measured (the swerve to the block is gone
  from the walls, though a *stopped* train's place is still a legitimate target, and should be);
- **turn the head while walking to re-acquire** — `NULL` as a host offset at three gains; the re-use above;
- **pursue rapidly** — the pursuit at 0.35 m/s, kept; the chase's START is the standing limit: a crossing becomes a
  chase one time in four, on a displacement that sits at the window centroid's jitter, and neither knob of the
  confirmation moves it without buying the room (§17.66–17.67). **The next lever is the substrate's: the age
  gradient across the window's voxels as a single-cast velocity** (§17.67), which would let a crossing be chased
  on its first sighting.

### 17.69 The chase's start had two defects; fixed, the pursuit triples (sweep 10; 2026-09-29, late)

**The operator's eye on R99 seed 6** ("306 s: the robot gains attention of the green block, the attention ring is
visible, ignores it completely; 522 s: the train in view, again ignores it. Is our attention mechanism regressing?")
and a per-sighting trace in the record (the chase record's fields 11–13: the miss from the prediction, the implied
speed, the decision). Two defects, both in the chase's start, both mine:

1. **Duplicate sightings.** `CloudMap` recomputes its clusters every four ticks but publishes the mover bearing every
   tick (re-aimed for yaw drift). The loop took every token as a sighting: four confirmations per real one, each
   pulling the velocity estimate half-way to zero, until a crossing train read as *still*. Fixed: the token carries
   the recompute tick as a sixth value and a sighting is new only when it changes.
2. **A per-step speed test.** The "too fast" test compared consecutive casts 80 ms apart: a centroid jitter of 8 cm
   read as 1 m/s, over the 0.6 m/s cap, and the candidate was replaced — the 518 s crossing at half a metre was
   replaced every half second. Fixed: the speed and the velocity are taken over a ring of the last 0.8 s of
   sightings (long enough to average the jitter, short enough to see a thing that just stopped).

The block at 306 s is not a defect: the seek loop takes no target from a *walking* sighting (R74's rule, `walk_refix_m`
0), made for the anchor-relative bearing that §17.54 fixed. R106 measures its removal (sweep 11).

**Seed 6 replayed on the fix:** 1 chase → 9 (3 re-acquired), four at the moving train within 0.5–1.1 m, six lost looks.

**Sweep 10 (n = 6, the base and R99 on the fixed loop, the same seeds as sweeps 6–9):**

| | R94 base, before → fixed | R99, before (§17.65) → fixed |
|---|---|---|
| chases in six runs (at the moving train) | 17 (5) → 21 (6) | 14 (6) → **47 (22)** |
| re-acquired after a loss · lost looks | 0 · 10 → 0 · 16 | 1 · 3 → **8 · 32** |
| train crossings chased | 5 of 22 → 5 of 17 | 6 of 22 → **14 of 27 (52 %)** |
| walls / min · rescues / min | 27.2 · 0.62 → 12.0 · 0.15 | 10.3 · 0.12 → 11.3 · 0.17 |
| stops · at a thing | 13.5 · 8.3 → 17.3 · 8.3 | 15.8 · 8.2 → 17.0 · 7.5 |
| seconds chasing a run | 4 → 5 | 4 → **14** |

Per seed, R99's chases: 4, 7, 7, 12, 8, 9 (re-acquired 1, 2, 0, 1, 1, 3) — every seed. **`WORKING`, loud**: the pursuit tripled and its purity held
(47 % of chases at the moving train, the train stopped another 19 %), the memory re-acquires it eight times, the
loop's own losses turn into 32 looks, and the walk is the walk of §17.65 (walls 11 ± 8, every seed under 23). The
cost the eye should weigh: seconds chasing a run 4 → 14 and stops ended on a chase 5 → 23 — the duck now breaks
off a look to follow the train, which is what was asked. The base also gains (the fix is in the loop both share),
and the priority's lead over it widens from 14/17 to 47/21.

### 17.70 Sweep 11: small things taken from the walk (R106; 2026-09-29, late)

`BearingSeekLoop.walk_take_range` 1.0: a bearing seen from a walking cloud starts a target when its fix is within a
metre, no target is held and no lost mover is in mind — R74's refusal of walking sightings removed, now that the
bearing it was made for is fixed. On R99, the fixed chase, six seeds.

| | R99 | R106 = R99 + the take from the walk |
|---|---|---|
| targets taken from the walk, six runs | 0 | 187 |
| walls / min · rescues / min | 11.3 ± 8.1 · 0.17 | 12.3 ± 7.7 · 0.22 |
| stops · at a thing · arrivals | 17.0 · 9.3 · 5.5 | 17.8 · 9.5 · 6.3 |
| skills · answers observed | 19 · 8 | 23 · 10 |
| chases (moving train) · re-acquired | 47 (22, 47 %) · 8 | 44 (15, 34 %) · 7 |

**`PARTIAL`.** The mechanism is live — thirty-one targets a run taken from the walk — and it does what the eye asked
(the walk turns to a block it passes), but the outcome barely moves: arrivals +0.8 a run, skills +4, walls and
rescues a tie within the spread, and the chase's purity falls from 47 to 34 % because a taken block competes with a
mover that has not yet confirmed. Not in the recommended stack by the numbers; an arm for the eye (preset R106),
and on the confirmation's twelve seeds (sweep 12) beside R99.

### 17.71 Sweep 12: the fixed chase on twelve seeds it never saw, and the priority's reversal (2026-09-29, late)

The stack (R94 + stuck stop and escape + chase vx 0.35, the campaign base), R99 (+ the mover's memory and priority)
and R106 (+ the take from the walk) on seeds 7–18, 600 s, the fixed chase; then pooled with seeds 1–6 (sweeps 10–11).

| | the stack, 7–18 | R99, 7–18 | R106, 7–18 | the stack, n = 18 | R99, n = 18 |
|---|---|---|---|---|---|
| walls / min, mean ± sd · median · max | 22.4 ± 23.5 · 13.8 · 88 | 35.2 ± 32.2 · 22.1 · 98 | 32.3 ± 32.0 · 22.4 · 98 | **18.9** | 27.2 |
| seeds over 60 walls a minute | 1 | 2 | 2 | 1 of 18 | 2 of 18 |
| rescues / min · down % | 0.26 · 0.43 | 0.28 · 0.30 | 0.36 · 0.41 | 0.22 | 0.24 |
| chases, twelve runs (moving train) | 58 (17, 29 %) | 70 (13, 19 %) | 71 (22, 31 %) | 79 (23) | **117 (35)** |
| re-acquired · lost looks | 0 · 47 | 10 · 54 | 8 · 52 | 0 · 63 | 18 · 86 |
| stops · at a thing · skills | 18.2 · — · 53 | 18.0 · — · 46 | 17.1 · — · 42 | | |
| walk m/s | 0.179 | 0.159 | 0.163 | | |

- **The fix confirms**: the chase is alive on every seed set — the stack 4.8 chases a run on the fresh seeds, R99
  5.8, crossings chased 38 % on the stack (27 % before the fix on seeds 1–6).
- **The priority's walk does not confirm.** On seeds 1–6 R99 walked at 10 walls a minute against the stack's 12;
  on seeds 7–18 at 35 against 22, with two runs trapped past 60. Pooled over eighteen seeds R99 is *worse* on walls
  (27 against 19), a tie on rescues and falls, and it pursues more: 117 chases against 79, 35 at the moving train
  against 23, 18 re-acquisitions against none, a fifth slower on the walk (the time spent chasing and looking). The
  purity is a tie (30 % against 29 %). The walls come with the pursuit: the track's long ends run 0.45 m from the
  walls, and a duck chasing the train toward an end at 0.35 m/s meets the wall the train turns away from; more
  chases, more of those. And five seconds without a static target after each of 86 losses is five seconds under
  play, which walks the walls.
- **The take from the walk (R106)** is R99 with a better chase purity (31 %) and the same walls: still a `PARTIAL`.

**Verdicts, n = 18 on the fixed chase.** The stack: `WORKING` on the walk (walls 19, one trap in eighteen, rescues
0.22) and now a live pursuit (4.4 chases a run). R99: `PARTIAL` — the pursuit-heavy form (6.5 chases a run, the
train re-acquired after a loss, the operator's stated first priority) at a cost in walls (27) that the seeds 1–6
signal had hidden. **The eye decides the trade**: preset "R94 · loaded + the STACK" for the walk, "R99 · the
MOVER's PRIORITY" for the pursuit; both on the fixed chase. What would reconcile them is the chase knowing the
walls — a pursuit that yields when its predicted target runs to within a body length of tall structure, the
isolation measure the candidate already uses, applied to the target (unbuilt).

### 17.72 Sweep 13–14: the pursuit that yields near tall structure (R107), built, measured, and found churning (2026-09-29, night)

**Built.** The cloud places the seek loop's held target back into its own frame (the inverse of `body_rel`: the
bearing token on `percept.seek_bearing` and the range on `reality.cognitive.seek_range`) and counts the voxels at
or above `iso_height` (0.25 m) within `target_iso_radius` (0.35 m, a body length) of it, published on
`percept.target_tall` as `[count, range]`. The seek loop (`yield_topic`, `chase_yield_tall 1`) yields a chase or a
coast whose target has a tall voxel that close. R107 = R99 + the yield; the gain-0 guard byte-identical (R83 plain,
md5 `6b9a0b3a…`). As built, the yield went through `lose()`: the memory of the mover kept, the lost look started.

R99 and R107 on seeds 1–6 (sweep 13) and 7–18 (sweep 14), 600 s, the campaign base (the loaded brain, the stuck
stop and escape, chase vx 0.35):

| | R99, 1–6 | R107, 1–6 | R99, 7–18 | R107, 7–18 | R99, n = 18 | R107, n = 18 |
|---|---|---|---|---|---|---|
| walls / min, mean ± sd | 11.3 ± 8.1 | 14.1 ± 18.7 | 35.2 ± 32.2 | 18.1 ± 15.3 | 27.2 ± 28.7 · median 19.0 | **16.8 ± 16.1 · median 8.5** |
| seeds over 60 walls a minute | 0 | 0 (seed 6: 52) | 2 (94, 98) | 0 (max 48) | 2 of 18 | 0 of 18 |
| R107 lower on the same seed | | 4 of 6 | | 8 of 12 | | 12 of 18; paired −10.4 ± 6.8 (sem) |
| rescues / min · down % | 0.17 · 0.28 | 0.17 · 0.33 | 0.28 · 0.30 | 0.33 · 0.43 | 0.24 | 0.28 |
| stops a run · stand % | 17.0 · 32 | 29.5 · 35 | 18.0 · 33 | 27.3 · 35 | | |
| chases (moving train, share) | 47 (22, 47 %) | 115 (32, 28 %) | 70 (13, 19 %) | 250 (76, 30 %) | 117 (35, 30 %) | 365 (108, 30 %) |
| yielded · re-acquired · lost looks | 0 · 8 · 32 | 93 · 78 · 94 | 0 · 10 · 54 | 195 · 178 · 175 | 0 · 18 · 86 | **288 · 256 · 269** |
| median chase length · seconds chasing a run | 1.6 s · 14 | **0.0 s** · 7 | 1.2 s | 0.0 s | | |
| walk m/s | 0.179 | 0.178 | 0.159 | 0.178 | | |

**Where the yields fire** (`yield_where.py`: each chase's start mapped into the world through the body's pose):
every one of seed 1's twenty and 59 of seeds 2–6's 63 within 0.4 m of a wall or a piece of furniture — the median
yield 0.28–0.31 m from furniture and 0.8–1.3 m from a wall. It is chair1 beside the track's western leg (its centre
0.45 m from the leg, its back 0.3 m from the passing train), not the track's ends: those run 0.45 m from the walls,
beyond the 0.35 m radius, so the wall case the lever was built for is out of its reach as configured, and the chair
case, which it was not built for, is in it. By label the yields are the train moving 13, the train stopped 25,
static 12, small things 13 (seeds 2–6).

**The churn.** Seed 1: 41 chases started, 36 re-acquired while coasting, 39 yielded, 39 lost looks; the yields at
110.6, 110.7, 110.7, 110.8, 110.9, 111.0, 111.1 s at the same place. The yield's `lose()` left the memory of a mover
at the yielded place; the next fresh sighting there matched the memory's prediction, re-acquired it (`chases_reacquired`),
and yielded again a tick later — a chase, a stop and a look per cast for as long as the thing stayed in view.
288 of 365 chases ended on their first tick; a run has twelve more stops than R99 and stands 3 points more.

**What the numbers mean.** The walls fell — pooled n = 18 from 27 to 17 a minute, the median 19 → 8.5, both of R99's
trapped runs gone, lower on twelve of eighteen seeds — to the stack's level (19) with R99's priority still in the
config. But the pursuit was destroyed in the same stroke (median chase 0 s, the seconds chasing halved, a stop per
yield), and the fall in walls cannot be credited to the yield's geometry: it came with twelve extra standing looks a
run, and a duck standing looks at fewer walls. R107 as built is a weakened slice of the mechanism (§3.2 item 6):
`PARTIAL` on the walk (an interesting artifact, the walls bought with stops), `REGRESSION` on the pursuit.

**The fix (R108, `yield_to_structure()`).** A yield is not a loss: the thing at the foot of tall structure is part
of that structure, so there is nothing to look for and no mover to keep in mind — no look, no memory. The PLACE
is remembered as not-a-mover for `chase_memory_ticks` (5 s): a sighting within `chase_gate_m` (0.35 m) of it is
dropped (`yield_drops`, in the host's summary); one further off is a fresh candidate again. Same parameters as
R107; the guard byte-identical (md5 `6b9a0b3a…`); the seek loop's test extended (the yield leaves no look and no
memory, the same sighting is dropped, a thing elsewhere is chased). Measured in §17.73 (sweeps 15–16).

### 17.73 Sweeps 15–16: the yield that forgets (R108) on eighteen seeds (2026-09-29, night)

R99 and R108 (R107's parameters on the fixed yield: no look, no memory, the yielded place not-a-mover for 5 s) on
seeds 1–6 (sweep 15) and 7–18 (sweep 16), 600 s, the campaign base. R99's rows are the same runs as §17.72.

| | R99, 1–6 | R108, 1–6 | R99, 7–18 | R108, 7–18 | R99, n = 18 | R108, n = 18 |
|---|---|---|---|---|---|---|
| walls / min, mean ± sd | 11.3 ± 8.1 | 11.3 ± 6.7 | 35.2 ± 32.2 | 23.1 ± 32.8 | 27.2 ± 28.7 · median 19.0 | **19.2 ± 27.2 · median 11.8** |
| seeds over 60 walls a minute | 0 | 0 | 2 (94, 98) | 1 (seed 7: 124) | 2 of 18 | 1 of 18 (13.0 without it) |
| R108 lower on the same seed | | 2 of 6 | | 8 of 12 | | 10 of 18; paired −8.1 ± 7.9 (sem) |
| rescues / min · down % | 0.17 · 0.28 | 0.15 · 0.35 | 0.28 · 0.30 | 0.28 · 0.48 | 0.24 · 0.29 | 0.24 · 0.44 |
| stops a run · stand % | 17.0 · 32 | 16.3 · 28 | 18.0 · 33 | 19.1 · 30 | | |
| chases (moving train, share) | 47 (22, 47 %) | 29 (7, 24 %) | 70 (13, 19 %) | 64 (20, 31 %) | 117 (35, 30 %) | 93 (27, 29 %) |
| yielded · dropped sightings | 0 · 0 | 13 · 10 | 0 · 0 | 24 · 64 | | 37 · 74 |
| re-acquired · lost looks · stops ended on a chase | 8 · 32 · 23 | 3 · 11 · 13 | 10 · 54 · 37 | 9 · 34 · 31 | 18 · 86 · 60 | 12 · 45 · 44 |
| median chase length · seconds chasing a run | 1.6 s · 14 | 1.1 s · 5 | 1.2 s · 8 | 1.0 s · 5 | | |
| path m · cells · walk m/s | 65 · 109 · 0.179 | 69 · 115 · 0.176 | 57 · 103 · 0.159 | 65 · 119 · 0.178 | | |

- **The churn is gone.** 37 yields in eighteen runs (R107: 288), 74 sightings dropped at a yielded place, the median
  chase a second long again, re-acquisitions 12 (R107's 256 were the churn). The yields fire where the lever was
  built to fire: `yield_where.py` on seeds 1–6 puts all six first-tick yields within 0.4 m of a wall or furniture
  (two at a wall face, the target beyond it; the rest at the table and chair1), none at the moving train.
- **The walls: R99's reversal is undone, to the stack's level.** Pooled 27 → 19 a minute (the stack, §17.71: 19),
  the median 19 → 12, lower on ten of eighteen seeds and on eight of the twelve where R99 reversed; R99's two
  trapped runs (94, 98) are 18 and 19 on R108. One R108 run is trapped instead — seed 7, 124 a minute, 3 stuck
  escapes that did not free it, 5 chases and one yield: the walk's own corner failure (the stuck stop's `PARTIAL`
  from §17.60), not the yield's. Rescues tie; down-time 0.29 → 0.44 % (a few more falls, within the sd).
- **The pursuit pays, less than R107 did.** 93 chases against R99's 117 (5.2 a run against 6.5), the moving train
  27 against 35 at the same share (29 %), 12 re-acquisitions against 18, the seconds chasing a run about half. The
  loss is not the yields themselves (37) but what R99 did after a chase ran to the chair or the wall: it lost the
  thing and LOOKED (86 lost looks against 45), and the look — a stop centred on where the train went — was where
  the train was found again (60 of R99's chases started from a stop). A silent yield walks on: the duck covers more
  room (65 m and 119 cells against 57 and 103 on seeds 7–18, at the walk's full speed) and sees the train less.

**Verdict, n = 18.** R108 `PARTIAL`, and the better trade so far: the stack's walk (19 walls a minute, one trap in
eighteen) with four fifths of R99's pursuit — the first arm that holds both. The reconciling mechanism works as a
mechanism; what it gives away is the look after the yield. **R109 (`chase_yield_look`)**: the yield starts the
lost look at the target's bearing (no memory, so no churn; the place still not-a-mover) — the duck stops short of
the chair or the wall and watches the train go by, and a train that has moved a body length on is a fresh candidate
from the stop. Measured in §17.74 (sweep 17, seeds 1–6, against R99's runs).

### 17.74 Sweep 17: the yield that looks (R109) — a regression on the walk at n = 6 (2026-09-29, night)

R108 + `chase_yield_look` (the yield raises the lost look at the target's bearing; no memory; the place still
not-a-mover) against R99's runs on seeds 1–6.

| seeds 1–6 | R99 | R108 (§17.73) | R109 |
|---|---|---|---|
| walls / min | 11.3 ± 8.1 | 11.3 ± 6.7 | **30.7 ± 23.7** (seeds 1 and 6: 53, 66) |
| rescues / min · down % | 0.17 · 0.28 | 0.15 · 0.35 | 0.27 · 0.44 |
| chases (moving train, share) | 47 (22, 47 %) | 29 (7, 24 %) | 32 (19, **59 %**) |
| yielded · dropped · lost looks · stops ended on a chase | 0 · 0 · 32 · 23 | 13 · 10 · 11 · 13 | 20 · 70 · 29 · 20 |
| median chase · seconds chasing a run | 1.6 s · 14 | 1.1 s · 5 | 0.9 s · 5 |

- **The pursuit's purity is the best of the campaign** — 19 of 32 chases at the moving train, against R99's 22 of
  47 — and 20 of its chases started from a stop: the look after the yield does find the train again, as §17.73
  predicted. But the pursuit is not larger (32 chases against 47; the seconds chasing a run 5).
- **The walk regresses**, and not through the looks: on the two bad seeds the wall episodes come from minute 3–4
  on (seed 1 per minute 0, 0, 0, 19, 70, 64, 99, 140, 78, 36; seed 6 peaks at 287 in its sixth minute), with two
  stuck stops and two escapes each that did not free the duck, and only 33 of 506 and 88 of 627 episodes within
  15 s of a lost look (3 and 5 a run). The same seeds walk at 3 and 22 on R99 and 13 and 23 on R108. It is the walk's
  corner trap (§17.60's `PARTIAL` on the stuck escape) landing on two runs of six, where R108 met it on one of
  eighteen and R99 on two; whether the extra standing beside walls and furniture (stand 35 % against 32) raises the
  odds of it is what n = 6 cannot say.

**Verdict.** R109 `REGRESSION` on the walk at n = 6 (a signal, enough to kill: §3 rule 7), the pursuit's purity a
signal the other way. Re-use context: once the walk's corner trap is solved (the stuck escape that frees, or a
free-space map, O49), the yield that looks is the form to retry — it is the only arm that finds the train again
without a memory to churn on. Not confirmed on seeds 7–18; R108 stays the arm.

### 17.75 Sweep 18: the yield's reach (R110, radius 0.5 m) — no better, killed at n = 6 (2026-09-29, night)

R108 against R108 + `cloud.target_iso_radius 0.5` on seeds 1–6: with the track's ends 0.45 m from the walls, a
wider radius was the knob that would put the wall case within the yield's reach.

| seeds 1–6 | R108 | R110 |
|---|---|---|
| walls / min | 11.3 ± 6.7 | 18.1 ± 12.7 (seeds 1, 3, 4: 27, 34, 26; seed 6: 2.3) |
| rescues / min · down % | 0.15 · 0.35 | 0.18 · 0.33 |
| chases (moving train, share) | 29 (7, 24 %) | 27 (6, 22 %) |
| yielded · dropped · lost looks | 13 · 10 · 11 | 17 · 39 · 10 |
| median chase · seconds chasing a run | 1.1 s · 5 | 0.3 s · 3 |
| yields at the moving train (`yield_where`) | 0 of 6 first-tick | 1 of 11 |

The wider radius yields more (17), drops four times the sightings (39), cuts the chase to a third of a second, and
neither fires at the train's ends (one of eleven first-tick yields at the moving train) nor improves the walk
(11 → 18 walls a minute, worse on three seeds of six). The pursuit is the same size. `NULL` on the pursuit,
`REGRESSION` on the walk at n = 6; killed. The track's ends are not where the yield's help was needed after all:
R108's walls at the stack's level came from the chair and the wall faces the chase ran at, and the run to an end
that loses the train there is the walk's ordinary business.

**Where the campaign stands (2026-09-29, night).** The arm is **R108** — R99's priority on the fixed chase with the
yield that forgets: the stack's walk (19 walls a minute, one trap in eighteen), four fifths of R99's pursuit (5.2
chases a run, 29 % at the moving train, 12 re-acquisitions), a preset for the eye ("R108 · the yield that
FORGETS"). The floor under every arm is the walk's corner trap, which lands on one or two runs in eighteen of each
(R99 seeds 11 and 14, R108 seed 7, R109 seeds 1 and 6) and which the stuck escape does not free; the next walk lever
is there (O49's free space, or an escape that turns rather than backs). The pursuit's next lever is the look after
a yield (R109's purity) once that floor is raised.

### 17.76 Sweeps 19–20: the heading reflex on the campaign's arm — the channel closes, the walk does not (2026-09-29, night)

**The operator's eye on R108 seed 1** (walks past the train it attends at 0.59 m, circles the green block, ignores
the red ball after a stop) traced through the log to one mechanism: the seek loop held a target 92 % of the run,
and the body did not walk to it. `walk_closing.py` (the new instrument) on R108's eighteen runs: a target held on
the walk 354 s a run; the range closing on 32 % of one-second windows, tangential 40 %, opening 27 %; with seek
holding the heading (54 %) closing 38 %; the heading error's median 1.26 rad; the yaw command's sign agreeing with
the reference on 49–67 % of firm half-seconds at every error size — a coin flip. The walker's yaw column does not
hold a heading (§17.17, §17.37), and the campaign's arguments never carried the heading reflex, set aside on
2026-09-22 (O57: the loops' targets were play's wall nodes then). R108 + `--heading-reflex 1.0 0.3 1.0` on seeds
1–6 (sweep 19) and 7–18 (sweep 20), against R108's own runs (seed-deterministic):

| n = 18 | R108 | R108 + reflex |
|---|---|---|
| walls / min, mean ± sd · median | 19.2 ± 27.2 · 11.8 | **28.5 ± 34.0 · 17.1** (higher on 14 of 18; seed 7: 153, seed 1: 38) |
| stuck stops · escapes | 36 · 34 | 28 · 27 |
| yaw command toward the reference, \|err\| 0.3–1.5 rad | 57–67 % | **79–80 %** |
| heading error while seek steers, median | 1.26 rad (1–6) · 1.12 (7–18) | **0.74 · 0.70** |
| closing · tangential · opening (target held, walking) | 32 · 40 · 28 % (1–6); 33 · 40 · 27 (7–18) | 31 · 48 · 22 %; 33 · 47 · 21 |
| closing with seek steering, target 1.0–1.5 m | 41 % | **61 %** |
| closing with seek steering, target under 0.5 m · tangential | 46 · 41 % | 34 · **60 %** (twice the windows) |
| attended on the walk nearer than the held target, not taken | 8.5 (1–6) · 6.2 (7–18) a run | **4.0 · 4.7** |
| arrival stops · near-misses a run | 5.0 · 2.2 (1–6); 5.6 · 3.3 (7–18) | 6.3 · 4.2; 6.3 · 3.1 |
| chases (1–6) · re-acquired · yields · dropped | 29 · 3 · 13 · 10 | 23 · 2 · 9 · 54 |

- **On the channel the reflex does what §17.37 measured**: agreement 57–67 → 79–80 %, the heading error halved,
  and where the field is clear the walk goes where seek points (closing 61 % at 1–1.5 m).
- **Within a metre of the thing it is released.** The reflex's share is `1 − near/gate` on the raw ToF field, and
  the thing walked to is a ToF hit: inside the gate the walker's own avoidance owns yaw again and orbits the
  thing — under 0.5 m the tangential share goes 41 → 60 % over twice the windows, near-misses 2.2 → 4.2 a run on
  seeds 1–6. The closing share over the whole walk does not move (32 → 31, 33 → 33).
- **The walls come back** (O57's cost): play holds the heading 37–41 % of the time and the reflex follows it
  faithfully; walls 19 → 28 a minute, higher on 14 of 18 seeds, seed 7's trap deeper (124 → 153) — while the stuck
  stops fall 36 → 28: the reflex's walls are brushed in passing, not the corner trap.

**Verdict, n = 18.** The reflex alone: `WORKING` on the channel, `NULL` on the closing, `REGRESSION` on the walls.
The mechanism was a weakened slice again (§3.2 item 6): a hold on the heading that lets go exactly where the
arrival happens. **The pair** the things phase built for the last metre is the seek gate (`--seek-gate`, T2: while
seek holds the reference its target's ToF sector reads free to the walker's sense) — extended now to the reflex's
release (the same sector is free for `near`; off = byte-identical). Measured in §17.77 (sweeps 21–22).

### 17.77 Sweeps 21–22: the reflex with the seek gate — the walk goes where the loops point, and brushes the walls (2026-09-29, night)

R108 + `--heading-reflex 1.0 0.3 1.0 --seek-gate` (the seek target's ToF sector free for the walker's sense AND for
the reflex's release) on seeds 1–6 (sweep 21) and 7–18 (sweep 22), against R108's runs and the reflex alone (§17.76).

| n = 18 | R108 | + reflex | **+ reflex + seek gate** |
|---|---|---|---|
| walls / min, mean ± sd · median · max | 19.2 ± 27.2 · 11.8 · 124 | 28.5 ± 34.0 · 17.1 · 153 | 29.8 ± 14.0 · 29.6 · **62** (higher on 15 of 18) |
| seconds a run touching a wall (1–6) | 7 | 16 | 15 |
| stuck stops · rescues / min | 36 · 0.24 | 28 · — | **26** · 0.19 |
| closing · tangential · opening (target held, walking) | 32 · 40 · 27 % | 32 · 47 · 21 % | **40 · 42 · 19 %** |
| seek holds the heading · closing under it | 54 % · 38 % | 60 % · 39 % | **65 % · 46 %** |
| heading error while seek steers, median | 1.2 rad | 0.7 | **0.5** |
| arrival stops a run | 5.4 | 6.3 | **8.3** |
| attended on the walk nearer than the held target, not taken | 7.0 a run | 4.5 | 5.6 |
| near-misses a run | 2.9 | 3.5 | 3.2 |
| chases (moving train, share) | 93 (27, 29 %) | — | 81 (27, 33 %) |

- **The walk closes.** With the target's sector free the reflex is no longer released at the thing: closing 32 → 40 %
  over the whole walk and 38 → 46 % under seek, the heading error a third of the control's, arrivals half again as
  many, the corner trap gone from the tail (the worst run 62 a minute against 124 and 153; stuck stops 36 → 26).
  The chase keeps its size and gains purity (33 % at the moving train).
- **The walls are brushed, not ridden.** The mean doubles but the spread halves: every seed between 7 and 62, the
  median 12 → 30, fifteen seconds a run in contact. Where: 73 % of the walking wall episodes come with seek holding
  the heading (the control: 44 %), the seek target a median 0.39 m from the nearest wall and 0.45 m away — the
  walk now reaches the things that stand by the walls (the ball in the corner) and the body, arriving within
  0.15 m of them, touches the wall beside them; and only 20 % of the episodes have a small thing within 0.6 m of
  the body (the control 47 %), so most are en route — a body that follows its reference straight, through the
  brushes the control's wandering avoided.

**Verdict, n = 18.** `PARTIAL`, and the arm for the eye on the operator's own criterion (a thing of interest is
walked to, not past): preset **"R108 · + the heading reflex + the SEEK GATE"**. The wall metric now counts
something different from the campaign's traps — brushing beside things and en route — and whether that is a cost
is the eye's call. Two levers follow from the where: (1) a seek target within a body length of tall structure
yields as the chase does (`percept.target_tall` is already published for the held target; a static-target yield
is one parameter away), which drops the things by the walls and the wall brushes with them; (2) the arrival
radius measured from the thing's near edge, so the body stops short of the thing and of what stands behind it.
The nearer attended thing replacing a held target (5.6 episodes a run) is the third.

### 17.78 Sweeps 23 and 25: the static yield near tall structure (R111) — killed at n = 6 (2026-09-29, night)

The operator's choice after §17.77 ("bound up near walls"): the chase's yield rule on static targets. Where the
eye's arm's static targets come from (c21–c22, 436 targets): sightings at stops 73 % (the target a median 0.33 m
from a wall, a quarter within 3 cm — tiny fragments of a wall's base band, 8 cm and two columns, the ToF's zone
spacing at a metre splitting the band into "small isolated" pieces), sightings on the walk 14 %, renewals 13 %,
movers that stopped 1 %. `static_yield_tall 1` (`seek`): a held static target with a tall voxel within a body
length of it (the cloud's `target_tall` count, a tick old) is dropped and the place is not-a-thing for
`forget_ticks`; a target set there by any path is refused. The first build (sweep 23) churned — the renewal
re-armed the yielded place and it yielded again two ticks later, 1 700 times a run, the arrival stops 12.7 →
1.5 — and was fixed at the target. The fixed build on the eye's arm (reflex + seek gate), seeds 1–6, sweep 25:

| seeds 1–6 | eye's arm (§17.77) | + the static yield |
|---|---|---|
| walls / min | 20.4 ± 7.8 (max 34) | 27.0 ± 22.1 (seed 4: 67; 2 of 6 better) |
| rescues / min · down % | 0.12 · 0.39 | 0.35 · 0.64 |
| static targets yielded · refused | — | 139 · 54 434 (the renewal, every tick) |
| a target held on the walk | 298 s a run | 176 s |
| arrivals (the loop's count) | 58 | 41 |
| closing · under seek | 40 · 47 % | 41 · 50 % |
| attended on the walk, not taken | 7.3 a run | 10.3 |
| chases (moving train) | 30 (13) | 33 (12) |

The yield fires (139 in six runs, at the wall fragments), the walk closes as before on what it keeps, and the
walls do not fall: they rise, with the rescues. Dropping the things by the walls leaves the loop without a target
a third more of the walk, and the walk without a target is play's, which brushes as many walls. The wall contact
was never the wall-adjacent targets. `REGRESSION` on the walk at n = 6, `NULL` on the walls; killed. Re-use: with a
row rule that tells a wall's base from a thing at the sighting (the operator's "long continuous rows", which the
neighbour count alone cannot — a fragment has 13 cloud columns within 0.3 m, a thing 18, the floor and other
things filling the ring either way), the same yield stops the *approach* rather than the target.

### 17.79 Sweep 27: the free-space gate on the reference, retried on the eye's arm — a tie, killed (2026-09-29, night)

Where the eye's arm's wall contact is (c21–c22, `wall_bursts`): 16 contact bursts a run, 24 s a run in contact
(R108: 11 and 14), the median burst half a second, and about five bursts of 5 s or more a run that hold two fifths
of the contact time. In those long bursts seek holds the heading on 60 of 95, a target is held on 84, the stuck
stop fired inside 5; the body pushes forward at half range (command 0.21 of 0.40) and does not move (sensed
forward speed 0.00), a stall the stuck detector does not count because its trigger wants the command above three
quarters of range. The free-space gate (`--ref-free 0.6`, §17.44: `NULL` on R67 without the stuck escape) on the
eye's arm, the seek target's own sector exempt (the seek gate), seeds 1–6:

| seeds 1–6 | eye's arm (§17.77) | + `--ref-free 0.6` |
|---|---|---|
| walls / min | 20.4 ± 7.8 | 19.5 ± 16.6 (seed 1: 34 → 15; seed 5: 24 → 52) |
| rescues / min | 0.12 | 0.22 |
| references released | — | 1 358 a run |
| closing · under seek | 40 · 47 % | 33 · 42 % |
| arrivals (the loop's count) | 58 | 49 |

A tie on the walls with a trap moved from one seed to another, and a cost on the walk: a released reference is
the heading itself, and the walk closes less. `NULL`, killed at n = 6; the 2026-09-19 verdict stands in the new
context. Next: the stall the bursts actually show — the stuck stop's forward-command bar lowered (`--stuck-cmd 0.4`,
byte-identical at 0.75), so pushing at a wall without moving starts the stop and the escape (§17.80).

### 17.80 Sweep 29: the stuck stop's bar lowered (`--stuck-cmd 0.4`) — the detector still does not fire; killed (2026-09-29, night)

| seeds 1–6 | eye's arm | + `--stuck-cmd 0.4` |
|---|---|---|
| walls / min | 20.4 ± 7.8 | 25.8 ± 16.2 (worse on 4, 5, 6) |
| stuck stops · escapes | 6 · 6 | 8 · 8 |
| rescues / min · down % | 0.12 · 0.39 | 0.20 · 0.21 |
| closing · arrivals (the loop's count) | 40 % · 58 | 40 % · 50 |
| bursts ≥ 5 s a run · share of contact time | 0.3 · 22 % | 0.8 · 38 % |

Two more stuck stops in six runs: the trigger's other half — a stall run longer than eight times the body's own
running median of stall lengths — moves with the bar, since the shorter stalls now counted raise the median. The
long bursts are not shortened. `NULL`, killed at n = 6. Three levers on the eye's arm's walls (§17.78–17.80) are
null; the bursts' anatomy says why: in 34 of 60 seek-held long bursts the target is half a metre or more away —
beyond the wall the body pushes on, a dead-reckoned position the world refutes — and no escape rule addresses a
target that should not be held. The seek loop's honest signal is the range left; a range that does not shrink
while the body walks is the loop's own error to act on (§17.81).

### 17.81 Sweeps 31–32: the progress forget (R112) — the walk closes, and two defects hold the corner (2026-09-29, night)

The seek loop acting on its own honest signal: with a static target held, every `progress_walk_m` (0.5 m) of
walking must shrink the range left by `progress_m` (0.05 m), or the target is forgotten — a dead-reckoned position
the world refutes (a wall between, an orbit). R112 = R108 + it, on the eye's arm (reflex + seek gate), seeds 1–6
(sweep 31) and 7–18 (sweep 32), against the eye's arm's own runs (§17.77):

| | eye's arm, 1–6 | R112, 1–6 | eye's arm, 7–18 | R112, 7–18 | eye's arm, n = 18 | R112, n = 18 |
|---|---|---|---|---|---|---|
| walls / min, mean ± sd | 20.4 ± 7.8 | **17.8 ± 9.1** | 34.6 ± 14.2 | 38.4 ± 25.5 | 29.8 ± 14.0 · median 30 · max 62 | 31.5 ± 23.4 · median 27 · **max 90** |
| lower than the eye's arm on the seed | | 4 of 6 | | 7 of 12 | | 11 of 18 |
| closing · tangential · opening | 40 · 41 · 20 % | **55 · 37 · 8 %** | 39 · 43 · 19 % | **48 · 44 · 8 %** | | |
| seek holds the heading · play | 69 · 29 % | **98 · 0 %** | 62 · 35 % | **95 · 2 %** | | |
| heading error while seek steers, median | 0.59 rad | **0.41** | 0.45 | **0.38** | | |
| arrival stops a run (host) | 12.7 | **14.5** | 13.3 | **14.9** | | |
| attended on the walk, not taken | 7.3 a run | 3.7 | 4.8 | 3.2 | | |
| targets forgotten for no progress | — | 25 a run | — | 32 a run | | 520 |
| stuck stops | 6 | 10 | 20 | 26 | 26 | 36 |
| rescues / min · down % | 0.12 · 0.39 | 0.20 · 0.20 | 0.23 · 0.39 | 0.18 · 0.32 | | |
| wall bursts ≥ 5 s a run · longest | 0.3 · 15 s | 0.5 · 8 s | 1.3 · 25 s | 0.8 · **77 s** | | |

- **The walk goes where it looks, at last.** Closing 40 → 55 % on seeds 1–6 and 39 → 48 % on 7–18, the opening
  share 20 → 8, the heading error 0.4 rad, arrivals up on both sets, and play never holds the heading while a
  target is held — a target going nowhere is dropped before its need decays to play's level.
- **The walls tie on average and trap on two seeds** (14: 90 a minute, 16: 77; the eye's arm had 7 and 62 there),
  one burst of 77 s. Seed 14's burst, traced tick by tick (510–600 s): the body sits in the corner at (−1.9, −1.9)
  and never moves; the odometry drifts 1.1 m on the gait's sway, so the seek range "grows" 0.35 → 1.37 m; the
  progress forget fires — and the outcome loop's renewal (`renew_topic`) re-arms the same position the next tick,
  resetting the window, 25 times a second; the stuck stop fires once at 513 s, its escape (6 s) fails, and it never
  fires again, because the detector re-arms only on a tick that is not stalled. Two defects, neither a lever:
  **a forgotten place must refuse the renewal** (the static yield's refusal, §17.78, now shared: the place is
  not-a-thing for `forget_ticks`), and **the stuck detector re-arms when its escape ends**. Both built (guard
  byte-identical); measured in §17.82 (sweeps 33–34).

**Verdict as built, n = 18.** `WORKING` on the walk's closing (the loudest move of the campaign on the operator's
criterion), `PARTIAL` on the walls pending the two fixes.

### 17.82 Sweeps 33–35: what a forgotten place may refuse — the renewal is load-bearing (2026-09-29, night)

Three forms of the progress forget's aftermath, each on the eye's arm, R112, seeds 1–6 (and 7–18 for the first):

| seeds 1–6 | as built (§17.81): the renewal re-arms at once | refuse everything there for 60 s | refuse the renewal, take a sighting |
|---|---|---|---|
| walls / min | **17.8 ± 9.1** (max 31) | 39.4 ± 17.8 (max 63) | 39.7 ± 26.4 (seed 1: 84) |
| a target held on the walk | 227 s a run | 167 s | 160 s |
| closing · opening | 55 · 8 % | **72 · 4 %** | **71 · 4 %** |
| heading error under seek | 0.41 rad | 0.31 | 0.31 |
| arrival stops (host) · the loop's arrivals | 14.5 · 50 | 13.8 · 54 | 12.3 · 45 |
| forgets · renewals | 147 · — | 50 · 22 179 refused | 62 · 40 523 refused |
| stuck stops | 10 | 6 | 3 |
| chases (moving train) | 25 (9) | 12 (6) | 22 (13) |
| wall bursts ≥ 5 s a run · longest | 0.5 · 8 s | 0.5 · 21 s | 1.7 · 23 s |
| seeds 7–18: walls · max · closing | 38.4 · 90 · 48 % | 34.2 · 120 · 69 % | — |

- **Refusing the renewal empties the loop.** With the forgotten place barred to the outcome loop's need, the walk
  holds a target a quarter less of the time; what it keeps it closes on beautifully (72 %), and the rest of the
  walk is play's, which doubles the walls. Admitting a fresh sighting changes nothing: the sightings at stops are
  too few to refill what the renewal did. The renewal — "a need still open at the thing re-arms it" — is the
  loop's engagement, and the progress forget's churn with it (a forget, a renewal, a restarted window) is cheap
  everywhere but a corner the body cannot leave, which is the stuck detector's business, not the target's.
- **The stuck detector's re-arm** (fires again when its escape ends, §17.81) did not multiply escapes (26 on seeds
  7–18 both ways) and shortened seed 14 (90 → 42) and 16 (77 → 11) while seed 9 found a new trap (120): under play,
  no target, pushing at the east wall with the ToF reading nothing at contact — the walker's blind zone against a
  surface (O35), a sensor's gap, not a loop's.

**Verdict.** The refusal `REGRESSION` in both forms; the as-built progress forget (the renewal admitted) is the
form, with the stuck re-arm kept — measured as the arm in §17.83 (sweeps 37–38). Lesson for the ledger: on this
duck the outcome loop's renewal is load-bearing for engagement; a forget or yield that bars it hands the walk to
play, and play walks the walls.

### 17.83 Sweeps 37–38: the stuck detector's re-arm is null; the arm, and where the night ends (2026-09-29, night)

| n = 18 | R108 | eye's arm (reflex + seek gate) | **R112, the progress forget, on it** | R112 + the re-arm |
|---|---|---|---|---|
| walls / min, mean ± sd · median · max | 19.2 ± 27.2 · 12 · 124 | 29.8 ± 14.0 · 30 · 62 | 31.5 ± 23.4 · 27 · 90 | 32.9 ± 26.6 · 23 · 90 |
| runs over 60 walls a minute | 1 | 1 | 2 | 3 |
| closing on the held target (1–6 · 7–18) | 32 · 33 % | 40 · 39 % | **55 · 48 %** | 55 · 51 % |
| opening | 27 % | 20 · 19 % | **8 · 8 %** | 9 · 8 % |
| play holds the heading with a target held | 44 % | 29–35 % | **0–2 %** | 1–2 % |
| heading error under seek, median | 1.2 rad | 0.5–0.6 | **0.4** | 0.4 |
| arrival stops a run (host) | 5.4 | 12.7 · 13.3 | **14.5 · 14.9** | 15.0 · 15.6 |
| stuck stops | 36 | 26 | 36 | 35 |
| rescues / min | 0.24 | 0.19 | 0.19 | 0.21 |

The re-arm (§17.81's second fix) fires nowhere it was meant to: seeds 14 and 16 are tick-identical to the as-built
runs (the stall there never re-crosses the detector's adaptive bar), ten of twelve seeds unchanged, seed 6 trapped
worse. `NULL`; it stays in the code as the detector's correct behaviour and changes nothing measured.

**Where the night ends.** On the operator's criterion — a thing of interest walked to, not past — the arm is
**R112 on the reflex and seek gate** (preset "R112 · the PROGRESS forget"): the walk closes on its target on half
its seconds where R108 closed on a third, opens on a twelfth where R108 opened on a quarter, play never holds the
heading while a target is held, arrivals nearly triple R108's, the heading error a third. The walls tie the eye's
arm at 30 a minute against R108's 19: the brushes beside the things now reached and en route, and two runs in
eighteen in a trap the walk's floor sets — the body pushing at a surface its ToF reads as empty at contact
(seed 9 under play, seed 14 under seek; O35's blind zone). Every escape-side lever on that floor was null
(§17.78–17.80, 17.83) and the target-side one that moved the walk (the progress forget) cannot see a body that
does not move. The floor is a sensor's: a return at contact that the walker and the stuck detector can act on —
the rewrite rule's second step, not a smarter policy. That is the next lever, and the one the camera boundary
should carry.

### 17.84 The contact consumers (2026-09-29, night): the ToF's near field is the contact sense

**The finding behind it.** In both of §17.83's traps the ToF summary's fourth slot — the share of zones classed
too-close, a horizontal range under `kMinRangeM` (0.10 m) — reads 1.0 for the whole burst, every one of the 64
zones, while the three range slots read 0.0: a too-close zone is not a hit with a range, so every consumer that
asks "is something ahead" of the range slots is blind exactly at contact. The sensor sees the wall. The walker's
sense carries the share as slot 15 (the restored identification never learned it: the babble did not push into
walls); the reflex's release, the free-space gate and the stuck detector take the maximum of the three range slots
and see nothing. The real part reports the same thing (the VL53L5CX's per-zone status under its minimum range), so
the channel exists on Pollen's hardware unchanged. Three consumers, one guarded lever each, on the arm R112 (the
progress forget on the reflex and seek gate), in the operator's order:

1. `--stuck-contact T` — a forward push with the too-close share above T is a stall, no adaptive bar;
2. `--contact-release` — the reflex's release and the free-space gate take the share as proximity;
3. `--contact-cloud` — too-close zones are filed in the cloud as occupied at the body's edge.

**Lever 1, first form (sweeps 39–40: push + contact for a second).** Seeds 1–6: walls 17.8 → 26.9 (seed 6: 70),
arrivals 14.5 → 9.5, chases 25 → 16, closing 55 → 49 %; seeds 7–18: walls 38.4 → 40.0, the traps gone — the
longest burst 77 → 11 s, bursts of 5 s or more 0.8 → 0.4 a run, contact time 33 → 22 s a run, closing 48 → 55 %,
arrivals 14.9 → 12.3, chases 59 → 36. It fired 387 times in eighteen runs (21 a run), 20 of seed set 1–6's 22
stuck stops at a wall and none at a thing — but on brushes the body was still sliding past, with the escape's
six seconds paid each time: two minutes a run of escaping, and the pursuit and the arrivals paid for it. The rule
lacked the stall half of its own definition. **Refined**: push, contact, and no forward motion (the sensed forward
speed under a quarter of range, the stall test's own term). Measured as sweeps 45–46, then levers 2 and 3 on top.

**Levers 1–3 measured (sweeps 45–50, the refined lever 1 as the base; R112 the control; n = 18 each).**

| | R112 | L1 · contact stall | L1 + L2 · release | L1 + L2 + L3 · cloud |
|---|---|---|---|---|
| walls / min, n = 18, mean ± sd · median · max | 31.5 ± 23.4 · 27 · 90 | 32.8 ± 20.6 · 27 · 90 | 29.6 ± 20.1 · 24 · 93 | 30.9 ± 23.5 · 26 · 108 |
| runs over 60 · over 40 | 2 · 4 | 1 · 6 | 1 · 3 | 1 · 5 |
| seeds 1–6: walls · contact s a run · longest burst | 17.8 · 15 · 8 s | 35.6 · 24 · 11 s | 43.8 · 32 · 21 s | 27.6 · 19 · 9 s |
| seeds 7–18: walls · contact s a run · longest burst | 38.4 · 33 · 77 s | 31.4 · 24 · 32 s | **22.5 · 13 · 8 s** | 32.5 · 22 · 29 s |
| rescues / min · down % (7–18) | 0.18 · 0.32 | 0.12 · **0.08** | 0.12 · 0.16 | 0.24 · 0.42 |
| rescues / min · down % (1–6) | 0.20 · 0.20 | 0.15 · 0.15 | 0.35 · 0.82 | 0.15 · 0.45 |
| closing (1–6 · 7–18) | 55 · 48 % | 52 · 53 % | 48 · 58 % | 57 · 54 % |
| arrival stops (1–6 · 7–18) | 14.5 · 14.9 | 11.2 · 13.3 | 11.2 · 15.0 | 12.3 · 13.8 |
| chases (1–6 · 7–18) | 25 · 59 | 15 · 33 | 13 · 48 | 20 · 57 |
| contact stalls fired, 18 runs | — | 285 | 331 | 382 |

- **The sense is right and every consumer of it is loud on one seed set and quiet or wrong on the other.** The
  contact stall fires at walls (20 of 22 of a set's stuck stops at a wall, none at a thing) and shortens the traps
  on seeds 7–18 (77 → 32 s, falls to a quarter) while worsening seeds 1–6 (18 → 36), since each firing is a
  six-second escape and 57 % of escapes walk straight back under a seek target held beyond the wall. The release
  gives seeds 7–18 the campaign's best wall tail (max 38, 13 s a run in contact, one long burst in twelve runs,
  arrivals and chases up) and seeds 1–6 their worst (seed 1: 93, falls ×4): yaw handed at contact to a walker
  whose avoidance never learned contact. The cloud filing undoes the release's damage on 1–6 (44 → 28, the
  longest burst 21 → 9 s) and undoes its gain on 7–18 (22 → 33).
- **Pooled over eighteen seeds every arm ties R112 within a run's noise** (30 ± 20 a minute, one run past 60).
  The wall count on this arm is set by which seeds fall into the walk's chaotic traps, and a lever that moves the
  tail on one set moves it back on the other. §3.3's rule applies: a lever that only shows after averaging is not
  the capability. `NULL` on the pooled walls for all three, with named signals — L1 on falls, L1+L2 on the tail of
  seeds 7–18 — and the sense itself confirmed (`WORKING` as an observation: 1.0 at every trap).
- **The consequence is the missing piece, not the sense.** The escape backs off and the reference walks the body
  back; the target the surface refutes is still held. Lever 1b: a stall fired by contact drops the seek loop's
  target (`--contact-forget`), measured in sweeps 51–52.

**Lever 1b (sweeps 51–52, `--contact-forget`).** A stall fired by contact drops the seek target (177 of 228
firings dropped one). Seeds 1–6: walls 35.6 → 33.8 (a tie; the chases back to 24 from 15, falls up); seeds 7–18:
31.4 → 39.2; pooled 37.4 ± 20.8, three runs past 60 (R112: 31.5, two). `REGRESSION`. The walk without the
target is play's, and play walks back to the wall.

**Verdict on the contact consumers, n = 18 each (2026-09-29, late night).** The sense is `WORKING` as an
observation: the ToF's too-close share reads 1.0 at every trap while the range slots read empty, on the sensor
the duck has and on the part Pollen ships. The four consequences imposed on it — the stop and escape, the
reflex's release, the cloud's filing, the target's drop — are `NULL` or `REGRESSION` on the pooled walls, each
loud on one seed set and reversed on the other, because each returns the body to the wall by a different road:
the escape's six seconds end and the reference walks back; the released yaw goes to a walker whose avoidance
never learned contact; the filed wall changes the escape's direction and not the walk's; the dropped target
hands the walk to play, which has the same wall in its node. The doctrine's through-line holds (§1: learned
cooperates, imposed fights): the walker's own sense already carries the share as slot 15, and the identification
restored for these runs never saw a push — the babble that produced it was a walk in the open. **The lever that
follows from tonight is a learned one: a babble that includes contact** (the walker pushed into walls and
things while its identification learns), so the avoidance that owns the yaw at contact has seen contact. That is
the re-use context for all four consumers, and the cheap one — the contact stall as a stop trigger — is the
instrument to keep on for it (it fires where it should and nowhere else).

**The arm stands: R112 on the reflex and seek gate** (preset "R112 · the PROGRESS forget"), with the contact
presets beside it for the eye. R108 on the wall metric alone.

### 17.85 The contact regime: a babble that includes contact (2026-09-30)

**What the walker's avoidance is** (`MotorEPMv2`, the intent module on the twist channel). The identification is
600 s of random twists, each held 1.5 s (`babble_ticks 30000`, `babble_hold 75`, `babble_scale 0.8`), learning a
forward model `A` (motor → sensor) whose row for each state element is the model's estimate of the motors'
authority over it. The avoidance is a **state prior**: `state_prior_indices [0, −6, −4, −3, −2]` with targets
`[0.75, 0, 0, 0, 0]` — the three ToF range slots (left, ahead, right, counted from the end of the state) held at
zero, the prior's error descended through `A`'s own authority row. The too-close share is the last element (−1)
and is **not in the prior**; and even with the objective, a model that never saw a push has a zero authority row
there and nothing to descend. That is why §17.84's four imposed consumers fought: nothing in the walker's model
knows what a push does.

**The regime, in the operator's shape.** (1) A contact room: `playroom_gen.py --babble-room` — a 2 m square
(`--half 1.0`), walls, a post (a 4 cm cylinder to 0.5 m), a box (24 cm, to 0.28 m), one chair; nothing else.
(2) The second babble: the current identification (`duck_r94_s1`) loaded, its babble window reopened
(`--rebabble 600`: the per-leg `steps_seen` and the whole-body counter set back), 600 s of random twists in the
room with the model learning, saved (`--save-brain checkpoints/duck_contact_s1.brain.json`). (3) The objective,
separately: R113 = R112 + the too-close share in the prior (`state_prior_indices [..., −1]`, target 0), with the
seek gate covering the share while the target is within reach (`--seek-gate-contact 0.3`: the thing walked onto
fills the near field at the arrival). Instrument: the host prints the walker's authority over the four ToF slots
after the restore and at the end (the abs-sum of `A`'s row).

**The first run did not babble.** The whole-body counter was set back and the per-leg path, which the duck
runs, keeps its own (`steps_seen`): the run walked under control for 620 s — and still met contact 42 s in its
second minute, since the model learns under control too. Fixed; the authority read from the per-leg `A`.

**Making the room teach (2026-09-30, the liveness check doing its job).** Three babbles before one that met a
wall face-on, each judged by the authority row and the contact seconds before any arm ran:
- 2 m room, the arm's babble (1.5 s pulses): the pulses come in antisymmetric pairs that cancel their own drift
  (`babble_isolate 1`: one motor at a time, a held pulse then its twin), so the duck babbled in place — 0 s of
  contact in ten minutes, the contact authority 0.016 → 0.004 (the model relearned without a push).
- 1.6 m room (a post and a box, the start keep-out cut to a body length), 3 s pulses: a forward pulse at the
  walk's real 0.18 m/s covers 0.54 m and turns back 0.16 m short of the wall — 3 s of contact.
- 1.6 m room, 6 s pulses at full scale: the walls touched 479 times, but backwards and sideways where the head's
  sensor looks away; face-on for 3 s; the too-close share above a half for 17 s; the authority unmoved.
- **1 m room, walls only, 6 s pulses**: every forward pulse a face-on push within two seconds — measured below.

**The seek gate's contact cover alone** (`--seek-gate-contact 0.3` on R112, seeds 1–6, sweep 56): walls 17.8 →
29.6, worse on every seed; zeroing a slot of the walker's sense changes the state its model sees. `REGRESSION`;
it does not ride in the arms, and the arrival risk under the contact prior is measured bare.

**The 1 m room's babble (the checkpoint the arms load, `duck_contact_s1`):** 52 s of contact in ten minutes
(9, 11, 6, 2, 4, 2, 5, 4, 2, 6 by minute), 16 s of it face-on pushing, 615 wall touches, no falls; the walker's
authority row over the contact slot 0.016 → 0.043, the left slot 0.022 → 0.031, and the ahead and right rows
0.022 → 0.008 and 0.020 → 0.012 — the model relearned its near field in a room where the ahead range is always
short. Liveness passed on contact; the walk itself is what the arms must also measure.

**The three arms, seeds 1–6 (sweeps 54, 57, 58; R112 on the current brain the control, sweep 31).**

| seeds 1–6 | R112, current brain | R113 prior, current brain | R112, contact brain | **R113 prior, contact brain** |
|---|---|---|---|---|
| walls / min | 17.8 ± 9.1 | 37.4 ± 18.8 (seed 6: 75) | 28.4 ± 12.9 | 21.5 ± 10.8 |
| rescues / min · down % | 0.20 · 0.20 | 0.48 · 0.64 | 0.23 · 0.34 | 0.15 · 0.47 |
| too-close share > ½ while walking, s a run · pushing | 17 · 6 | 36 · 9 | 22 · 8 | 21 · 6 |
| wall contact s a run · bursts ≥ 5 s · longest | 15 · 0.5 · 8 s | 39 · 1.5 · 41 s | 23 · 1.0 · 22 s | 17 · 0.5 · 17 s |
| arrival stops · closing | 14.5 · 55 % | 16.2 · 51 % | 18.8 · 54 % | 17.7 · 50 % |
| contact authority at the end (seed 1) | — | 0.085 | 0.034 | 0.054 |

- **The prior on a model that never saw a push is a regression** (walls doubled, rescues ×2.4, contact ×2): it
  descends an authority row the model invents under control, and the walk pays. The operator's premise, measured.
- **The contact brain alone is worse than the current one** (walls 18 → 28, contact 17 → 22 s): the second babble
  in a 1 m room relearned the near field where the ahead range is always short (its ahead row 0.022 → 0.008), and
  the walk carried that into the playroom.
- **The prior on the contact brain recovers both** — 37 → 21 against the prior alone, 28 → 21 against the brain
  alone — the objective and the learned authority need each other, which is the regime's whole claim. Against
  R112 it is a tie on walls and contact (21 vs 18 a minute, 21 vs 17 s), with more arrivals and the lowest rescues.

**Verdict at n = 6.** The mechanism `WORKING` (the interaction is loud); the pair `NULL` against R112 on the
outcome. What it wants is a richer room: 16 s of face-on pushing in ten minutes gave a contact row of 0.043
against range rows of 0.02–0.03, and eroded the ahead row. Next: the pair and the contact brain on seeds 7–18,
and a 1200 s babble's checkpoint on the pair (sweeps 59–61).

**Eighteen seeds (sweeps 57–60 with 31–32 the control), and the 1200 s babble (sweep 61).**

| n = 18 | R112, current brain | R112, contact brain | **R113 prior, contact brain** |
|---|---|---|---|
| walls / min, mean ± sd · median · max | 31.5 ± 23.4 · 27.0 · 90 | 24.4 ± 15.4 · 23.8 · 67 | **23.7 ± 17.9 · 19.7** · 71 |
| runs over 60 | 2 | 1 | 1 |
| the too-close share above ½ while walking, s a run | 30 | 17 | **16** |
| wall contact, s a run · bursts ≥ 5 s · longest | 27 · 0.7 · 77 s | 18 · 1.1 · 24 s | **16 · 0.6 · 17 s** |
| seeds 7–18: walls · contact s · longest burst | 38.4 · 36 · 77 s | 22.4 · 14 · 24 s | 24.8 · 14 · 10 s |
| seeds 1–6: walls · contact s · longest burst | 17.8 · 17 · 8 s | 28.4 · 22 · 22 s | 21.5 · 21 · 17 s |
| closing (1–6 · 7–18) · arrival stops | 55 · 48 % · 14.7 | 54 · 53 % · 19.0 | 50 · 52 % · 17.5 |
| rescues / min (1–6 · 7–18) | 0.20 · 0.18 | 0.23 · 0.18 | 0.15 · 0.19 |

- **The learned avoidance transfers.** A model that pushed walls for 16 s in a 1 m room, loaded into the playroom
  with the train and the toys, halves the time the walker spends with its near field full (30 → 16 s a run),
  cuts wall contact 27 → 16 s, takes the longest burst from 77 to 17 s and walls from 31.5 to 24 a minute, with
  closing held and arrivals up. On the twelve seeds the babble never saw, the contact brain alone does most of it
  (36 → 14 s of contact); the prior's share shows on seeds 1–6 (28 → 21 walls) and in the tail (the longest
  burst 24 → 17 s, bursts of 5 s or more 1.1 → 0.6 a run).
- **More room is not better.** The 1200 s babble (223 s of contact, 116 s of pushing) left the contact row at
  0.021 and blew the range rows up (left 0.022 → 0.198); the pair on it keeps the near field clearest of all (11 s)
  by standing sideways at walls (wall contact 39 s a run, the longest burst 64 s) and loses the walk (closing 55 →
  40 %, arrivals 14.5 → 8.3). The room teaches a linear model what it can hold; ten minutes is the dose here.

**Verdict, n = 18 (2026-09-30).** The contact regime `WORKING`: the sense the four imposed consumers could not
use is now used by a model that learned it, and the outcome moved on every wall measure while the walk held —
the first wall lever of the campaign that is not a tie. `PARTIAL` on the prior's share (a signal on six seeds and
in the tail, a tie on the fresh twelve). **The arm is R113 on the contact brain** (preset "R113 · the CONTACT
prior on the CONTACT brain", checkpoint `duck_contact_s1`); the regime is the room, `--rebabble 600` from the
current identification, saved. Re-use: the room dose (the babble's duration and the room's size) is the knob to
sweep next, and the prior's share to confirm at n ≥ 20 varied worlds.

### 17.86 The seven-motor identification: the head as the walker's own motor (2026-09-30)

**The operator's question**: can the brain babble neck and head positions for manoeuvrability, since a head
offset changes the walk and the walk's turning radius circles? **Two facts.** Pollen's walking policy takes a
command vector that carries the four head commands (neck pitch, head pitch, head yaw, head roll, as offsets from
home) and was trained across their ranges (±1.10, ±1.10, ±1.40, ±0.31 rad, `kHeadRange`), so the gait
compensates for the head because it learned to; and with `--head-joints` the head brain writes the joint targets
directly while the policy's head command is zeroed — the walk never sees the head it carries. The walker's
identification (`motor_epm_intent`, `MotorEPMv2` on the twist) has three motors and no authority over anything
the head does. The earlier `--head-forward` lever (§17.60, null on speed, falls at 0.1 rad) was a constant
imposed on a policy that treats the head as an input.

**Built.** `--intent-head F`: the intent's action topics extended to the seven commands (`action.neck_pitch`,
`action.head_pitch`, `action.head_yaw`, `action.head_roll` beside the twist; `motor_dim 7`); on the walk the four
head actions are the policy's head command at F of the trained ranges, and the head brain's joint ownership
waits for a stop (the gaze at stops unchanged). 0 = off, byte-identical. The host prints the model's authority
table (rows: sensed vx, vy, wz, the heading error, the four ToF slots; columns: the motors) after the restore
and at the end. The yaw cap (`kTwistRangeVyaw 1.0`) against the policy's trained yaw range is unverified
(the scaffold notes do not state it) — the model's own column for `vyaw` is the measurement that matters.

**The room identification (from scratch, since a three-motor checkpoint cannot restore into seven motors):**
600 s of the structured babble cycling all seven motors in the 1 m room, the head at 0.4 of its ranges; 33 s of
contact, 13 s of pushing, one fall. The authority table at the end, the rows that matter:

| row \ motor | vx | vy | vyaw | neck_p | head_p | head_y | head_r |
|---|---|---|---|---|---|---|---|
| sensed wz | +0.001 | 0.000 | −0.009 | −0.003 | +0.003 | −0.004 | +0.001 |
| heading error | −0.005 | −0.024 | **+0.025** | +0.005 | **+0.029** | **−0.027** | −0.002 |
| tof left | −0.008 | −0.003 | +0.032 | +0.010 | +0.027 | −0.007 | −0.014 |
| tof right | +0.034 | +0.033 | −0.021 | +0.014 | −0.029 | +0.009 | +0.036 |
| contact | −0.015 | +0.007 | +0.008 | +0.001 | +0.002 | +0.002 | −0.021 |

Head pitch and head yaw carry authority over the heading error on a par with the yaw command's own: the lean
the operator saw is in the model. Head pitch also moves the ToF slots (it tilts the sensor), which is the cheat
the mask guards against if the arms show it. The playroom arms: R113 on seven motors loading this brain, against
R113 on three motors loading a three-motor identification from the same room from scratch (the fair control:
continued against from-scratch is a confound the current-brain checkpoint would carry).

**The playroom arms, seeds 1–6 (sweeps 62–63; R113 on the continued contact brain, sweep 58, beside them).**

| seeds 1–6 | R113, three motors, continued brain (58) | R113, three motors, from-scratch room brain (63) | **R113, seven motors, seven-motor room brain (62)** |
|---|---|---|---|
| yaw rate toward the reference with \|err\| > 1 rad, median · p75 | 0.06 · 0.32 rad/s | 0.04 · 0.31 | **0.16 · 0.53** |
| the error closed by 0.5 rad within 3 s | 38 % | 33 % | **46 %** |
| closing · tangential · opening | 50 · 39 · 11 % | 46 · 43 · 11 % | **58 · 33 · 10 %** |
| walk m/s · path m · arrival stops | — · — · 17.7 | 0.175 · 55 · 15.5 | **0.211 · 73 · 23.3** |
| walls / min | 21.5 | 17.2 | 30.4 |
| too-close share > ½, s a run · wall contact s | 21 · 17 | 22 · 16 | 43 · 32 |
| rescues / min · falls a run | 0.15 · 1.5 | 0.30 · 3.0 | **1.33 · 13.3** |

- **The head is a steering motor**, as the operator saw: the seven-motor walker turns toward a large heading
  error four times as fast as the three-motor one, closes on its targets on 58 % of its seconds (the campaign's
  best on eighteen or six seeds), walks a fifth faster and arrives half again as often. The turning radius that
  circled is the three-motor command's, not the body's.
- **It spends the head with no objective on balance**: thirteen falls a run against three, rescues ×4, twice the
  contact. The prior descends the heading error through the head's authority as far as it will go; nothing in
  the prior says stay upright. Two one-knob answers, each an arm: the head at a fifth of its ranges
  (`--intent-head 0.2`), and the trunk's tilt (the sense's gravity x, y: state elements 3 and 4) held at zero in
  the prior — the error a fall minimises, given to the module that owns the head (sweeps 64–65).

**The balance arms, seeds 1–6 (sweeps 64–65).** The head at a fifth of its ranges: falls 13.3 → 11.5, turn 0.13 rad/s,
closing 56 %, contact 45 s — the amplitude is not the cause. The tilt prior (state 3, 4 → 0): falls 10.5, contact
43 → 21 s a run (halved), the turn 0.22 rad/s (the best), closing 58 %, walls 31. **The cause of the falls is in
the head's motion, not its reach**: with the intent driving it the head joints move 0.05 rad a tick (2.5 rad/s,
three times the head brain's 0.018) because the controller emits a fresh head command every 20 ms, which the
policy was not trained to track and no servo could follow. The fix is the walker's own kind: a low-pass on the
intent's head command (`--intent-head F TAU`, 0.3 s), measured with and without the tilt prior (sweeps 66–67).

**The filtered head (sweeps 66–67) and where the falls come from.** A 0.3 s low-pass on the intent's head
command: with the tilt prior, walls 48, closing 48 %, falls 9.7; without it, closing **62 %** (the best of the
campaign), arrivals 21.5, walk 0.20 m/s, falls 9.0. The head joints still step 0.038 rad a tick (the control's
0.018), so the joints' motion is the policy's own tracking, not the command's jitter, and the falls did not
move. **The falls' anatomy** (the two seconds before each fall from a walk): the control falls in fast forward
turns (yaw rate 0.52 rad/s, forward 0.23 m/s, the yaw command 0.63); the seven-motor arms fall **walking
backwards** (forward −0.05 m/s, the yaw command 0.16, the head yaw 0.18 rad). The seven-motor controller backs up
because its model barely knows what forward does: identified from scratch in a 1 m room, where a forward pulse
meets a wall within two seconds, its authority of `vx` over the sensed forward speed is 0.009 (the room table),
so the speed prior (state 0 → 0.75) has nothing to descend and the controller drifts backward, where the policy
falls. The regime the operator proposed had the order right: **identify in the open, then the contact room as
the second babble.** Chained: the seven motors babbled 600 s in the plain playroom, rebabbled 600 s in the 1 m
room, then the two arms (sweeps 68–69).

**The open-then-room brain (sweeps 68–69) and the head that stays turned.** Identifying in the open first
(vx's authority over forward speed 0.029, three times the room's) and rebabbling in the room (33 s of contact,
one fall; the room erodes the forward row to 0.011 and makes head yaw the strongest heading motor, −0.045, and
head pitch the strongest contact motor, +0.070 — the sensor-pointing channel) did not move the falls: with the
tilt prior 10.5 a run (turn 0.06 rad/s, contact 49 s), without it 15.3 (turn 0.15, closing 56 %). The backing
before a fall is not the difference — the control commands backward on 23 % of walking ticks too. **The
difference is the head**: yawed past 0.3 rad on 73–80 % of walking ticks in every seven-motor arm, never in the
control. The controller found heading authority in head yaw and holds the head turned, so the ToF points sideways
(every range prior then steers against a view that is not the walk's), the pursuit's sightings come from the
side, and the policy falls backing up with a turned head. A permanently turned head is the degenerate use of a
steering motor, the sensor-pointing cheat in another form. Two masks follow: the ToF and contact priors descend
through the twist motors only (`state_prior_motors`, sweeps 70–71 on the first ordering), and, re-identified
with head yaw as the last motor, the heading and tilt priors through everything but head yaw — the head leans,
it does not steer by yaw (sweep 72).

**The ToF mask alone (sweeps 70–71).** With the ToF and contact priors held to the twist motors and the heading
prior still free to use head yaw: the head yawed past 0.3 rad on 82 % of the walk (from 76), contact 60 s,
falls 10.0 with the tilt prior; falls 17.0 and one run down for six minutes without it. `NULL` on its own — the
turned head is the heading prior's, and the mask that matters is the one on the heading (sweep 72).

**Head yaw out of every prior (sweep 72, the second identification with head yaw last).** The room table gives
head roll the largest authority over the heading error (+0.055) and the contact share (+0.068), head yaw −0.030
on the heading. With the heading and tilt priors kept off head yaw and the ToF priors on the twist: the head
yawed past 0.3 rad on **93 %** of the walk, falls 12.7, contact 27 s, walls 26, turn 0.07 rad/s, closing 51 %.
The turned head is not the prior's descent: it is the controller's own learning on that motor, and no mask
on the prior reaches it. **Head yaw as a walker motor is unusable as built** — the gaze stays the head brain's.
The clean form of the operator's hypothesis, the lean, is six motors: the twist with neck pitch, head pitch and
head roll (sweep 73).

**The lean (sweep 73, six motors: the twist with neck pitch, head pitch, head roll; head yaw the head brain's).**

| seeds 1–6 | control: 3 motors, room brain (63) | 7 motors (62) | 7, tilt prior (65) | 7, filtered head (67) | 7 v2, yaw masked (72) | **6, the lean (73)** |
|---|---|---|---|---|---|---|
| yaw rate toward the reference, \|err\| > 1 rad, median · p75 | 0.04 · 0.31 | 0.16 · 0.53 | 0.22 · 0.54 | 0.12 · 0.42 | 0.07 · 0.38 | **0.13 · 0.53** |
| the error closed by 0.5 rad within 3 s | 33 % | 46 % | 46 % | 41 % | 40 % | **46 %** |
| closing on the target | 46 % | 58 % | 58 % | **62 %** | 51 % | 52 % |
| walk m/s · arrival stops | 0.175 · 15.5 | 0.211 · 23.3 | 0.199 · 19.5 | 0.202 · 21.5 | 0.198 · 15.8 | 0.206 · 21.2 |
| head yawed past 0.3 rad, share of the walk | 0 % | 73 % | — | — | 93 % | **1 %** |
| walls / min · wall contact s · longest burst | 17 · 16 · 12 s | 30 · 32 · 22 s | 31 · 22 · 23 s | 28 · — · — | 26 · 27 · 15 s | 31 · 24 · **9 s** |
| falls a run | **3.0** | 13.3 | 10.5 | 9.0 | 12.7 | 11.7 |

**Verdict, §17.86 (n = 6 each, 2026-09-30).** The head is a steering motor and the walker learns to use it: in
every form the seven- and six-motor walkers turn toward a large heading error three to five times faster than
the three-motor one, close on their targets more, walk a fifth faster and arrive half again as often — the
operator's hypothesis, measured, and the circling's cause named (the three-motor command's turning radius, not
the body's). With head yaw among the motors the controller holds the head turned on three quarters of the walk
whatever the prior is allowed to descend through, so head yaw stays the head brain's; the lean (neck pitch, head
pitch, head roll) keeps the gaze straight and keeps the manoeuvrability. **What every form pays is falls: four
times the control's**, and neither the head's amplitude (0.2 of range), nor its smoothing (0.3 s), nor a tilt
objective, nor an open-first identification moved that below nine a run. The falls come backing up with the
head pitched or turned, states the policy meets only under this controller. `PARTIAL`: the manoeuvrability
`WORKING`, the balance unsolved. Re-use: the falls are the next lever, and the candidates are the policy's own
envelope (a rate limit on the head command in the walker's own units; a prior on the sensed backward speed) —
measured before any of this rides in the arm. The arm stays R113 on the three-motor contact brain. The contact
instrument on the ToF's too-close share is confounded under a pitching head (looking down puts the floor in the
near field); the wall-contact and burst measures are the ones to read for the head arms.

**The operator's eye on the seven-motor walk (2026-09-30):** more expressive and more alive, the seeking more
accurate — and the head rings on the pitch axis at the moment of standing, and the duck sometimes falls backwards
with the head pitched up as it stops; the previous embodiment's still head was good for the gaze and the ToF.
Measured on the six-motor arm: 9.5 of its 11.7 falls a run come within 3 s of a stop's start, the head pitched
back 0.43 rad half a second before; the head's pitch speed in a stop's first two seconds 1.64 rad/s (the control
0.33). **It is the hand-off**: at a stop the head brain takes the joints from wherever the intent's steering
left them and its level loop starts with a large error; when the walk resumes the intent's command jumps back.
`--head-slew R`: both targets slew from the head's current position at a servo's rate across an ownership
change (1.0 rad/s), a property of the embodiment, not a policy (sweep 74).

**The slew (sweep 75, `--head-slew 1.0`, the six-motor arm):** the head's pitch speed in a stop's first two
seconds 1.64 → 1.01 rad/s (the control 0.33), falls 11.7 → 8.5 a run — and 8.0 of them still within 3 s of a
stop's start, the head pitched back 0.39 rad half a second before. The slew bounds the ringing; it does not
change where the head is when the stand begins. The second thing hidden in the hand-off: with the head brain
owning the joints, the policy's head command is zeroed ("the walker is told nothing about the head"), a choice
from the level-head regime; the standing policy balances for a head at home while carrying one pitched back.
`--tell-head`: the policy's head command is the head's own targets as offsets from home (sweep 76).

**Telling the policy about the head (sweep 76, `--tell-head`):** falls 8.5 → 12.8, 11.2 of them at stops.
`REGRESSION`, killed. **The anatomy at the stop's start**: the head arrives from the walk pitched down (p90
+0.37 rad), the head brain's level loop overshoots to −0.42 (pitched up) as it rings, and the duck goes over
backwards. The slew bounds the speed of that, not the overshoot. The previous embodiment's still head at stops
is the protocol to keep: `--head-home S` — for S seconds after a stop begins the head's targets are HOME
(slewed), and the head brain's gaze takes over from level rather than from where the walk left the head
(sweep 77, 1.5 s).

**The head home at stops (sweep 77, `--head-slew 1.0 --head-home 1.5`, the six-motor arm, seeds 1–6):**

| seeds 1–6 | control: 3 motors, room brain (63) | 6 motors, the lean (73) | + slew (75) | **+ slew + head home (77)** |
|---|---|---|---|---|
| falls a run · at stops | 3.0 · 0.3 | 11.7 · 9.5 | 8.5 · 8.0 | **2.7 · 2.2** |
| rescues / min · down % | 0.30 · 0.82 | 1.17 · 2.75 | 0.85 · 1.97 | **0.27 · 0.47** |
| head pitch speed in a stop's first 2 s | 0.33 rad/s | 1.64 | 1.01 | **0.79** |
| yaw rate toward the reference, \|err\| > 1 rad · closed in 3 s | 0.04 · 33 % | 0.13 · 46 % | 0.12 · 48 % | **0.14 · 42 %** |
| closing · walk m/s · arrivals a run | 46 % · 0.175 · 15.5 | 52 % · 0.206 · 21.2 | 53 % · 0.198 · 21.7 | **53 % · 0.197 · 22.5** |
| walls / min | 17.2 | 31.0 | 29.9 | 27.1 |

The falls return to the three-motor walker's rate with the lean's turning, closing and arrivals kept; the ringing
is halved and the rare fall left is a different one (the head pitched down, +0.32). `WORKING` at n = 6 on the
operator's request — the resonance damped, the feet kept — with the walls (27 against 17) the remaining cost and
the confirmation on seeds 7–18 in flight (sweep 78). Preset "★ R113 · SIX motors, the LEAN, the head home at
stops"; the seven-motor preset carries the same hand-off.

**Eighteen seeds (sweeps 77–79; the three-motor room brain on all eighteen the control).**

| n = 18 | 3 motors, room brain | **6 motors, the lean, the head home at stops** |
|---|---|---|
| falls a run · within 3 s of a stop's start | 2.0 · 0.2 | 2.7 · 1.9 |
| head pitch speed in a stop's first 2 s (7–18) | 0.31 rad/s | 0.64 |
| yaw rate toward the reference, \|err\| > 1 rad, median | 0.06 rad/s | **0.10** |
| the error closed by 0.5 rad within 3 s | 37 % | **42 %** |
| closing on the target | 50 % | **53 %** |
| arrival stops a run (the loop's arrivals) | 10.4 | **14.4** |
| walk m/s (7–18) | 0.171 | 0.190 |
| walls / min, mean ± sd · median · max | 20.7 ± 12.5 · 17.5 · 48 | 28.3 ± 14.5 · 29.3 · 52 |

**Verdict, n = 18 (2026-09-30, evening).** On the operator's request the hand-off protocol is `WORKING`: the
resonance is damped (the head's pitch speed at a stop's start a third of the unprotected lean's 1.64 rad/s)
and the falls are back at the three-motor walker's rate, with the lean's gains kept and confirmed on the fresh
seeds — more modestly than six seeds said (the control walks well there): the turn toward a large error +70 %,
the error closed within three seconds +5 points, closing +3, arrivals +38 %, a tenth faster. The cost is the
walls (28 against 21 a minute, brushes rather than traps: the worst run 52). `PARTIAL` overall — the expressive
walker keeps its feet, and the eye decides the walls against the aliveness. Preset "★ R113 · SIX motors, the
LEAN, the head home at stops" (checkpoint `duck_contact6_s1`, `--intent-head 0.4 --head-slew 1.0 --head-home
1.5`). Ledger: `--tell-head` `REGRESSION`; the slew alone halves the ringing and not the falls; the room's
second babble erodes the forward row (identify in the open first); head yaw as a motor is held sideways.

**The operator's eye on the lean (2026-09-30, evening):** the head tilted most of the time and moving a lot,
a worry for the ToF's accuracy on the walk; the roll the strangest and the least stable; looking around while
walking is what is wanted, so yaw only, with the stability of before. Measured on the lean: the head pitched
more than 0.2 rad from its median on 28 % of walking ticks and rolled on 15 %, the head joints moving at 2.4 rad/s
against the still head's 1.1. **Yaw as the only head motor (four motors, sweep 80, open then room, the slew and
the head home):** the head yawed past 0.2 rad on **91 %** of the walk with pitch and roll level (3 %), the turn
toward a large error 0.02 rad/s (below the three-motor control's 0.04), closing 51 %, walls **42** a minute
(the control 17), falls 2.0. `REGRESSION`. The controller parks a yaw motor sideways whatever it is allowed to
descend through (the fourth time measured), and a head looking sideways points the walker's own ToF slots —
left, ahead, right, in the head's frame — away from the walk, so the avoidance steers against the wrong view
and the walls double. **Head yaw is not a walker motor.**

**What the eye's two wishes need, separately.** (1) The ToF's accuracy and the stable head: the three-motor
walker, whose head the head brain holds level — the manoeuvrability the lean bought (turn +70 %, arrivals +38 %)
costs exactly the head motion the operator distrusts, and the choice between them is the eye's. (2) Looking
around while walking is a gaze, not a steering motor: the head brain's yaw sweeping on the walk (the stop's gaze
sweep exists, a walk form does not), and it needs the walker's ToF slots rotated from the head's frame into the
body's first, or the avoidance reads a sideways view — the enabling lever before any gaze on the walk (unbuilt;
the cloud already casts in the head's true pose, so the map is unaffected). Presets: "★ R113 · the CONTACT
prior on the CONTACT brain" (three motors, the still head) is the arm; "R113 · SIX motors, the LEAN, the head
home at stops" the expressive alternative for the eye.

### 17.87 The lean that settles: the head sensed, a level prior, grow on restore (2026-10-01)

**The operator's direction** (chase phase §10): the six-motor lean's head motion and its nod at stops are wanted
(personality, and a nod at a small thing on the floor fits the duck's interest in it) as long as nothing falls, and
the brain must be able to hold the head still when it needs a stable cloud. First the lean's own habit: the head
down going forward, up going backward, held as long as the pace is. An inverted pendulum leans into an acceleration
and comes back at a steady pace. Worked in the small room first, then the playroom; the new sense grown on restore.

**Stage A (the saved sweeps; `mj_host/tools/lean_readout.py`).** The head's attitude pitch (−hg[2], + = nose down)
against forward speed and acceleration, each through a 0.5 s box. The still three-motor walker: level (sd 0.014 rad).
The lean (sweeps 73, 77): 0.31 / 0.52 rad down at a forward cruise, 0.79 (45°) at 0.2–0.3 m/s, a head-up tail on a
fifth of the backing; pitch = … + 1.4–1.7·v + 0.0·a. The lean follows SPEED. **The cause, in the saved brain:** the
speed prior (sensed vx → 0.75 of range) descends through every motor with authority over forward speed, head pitch
and neck pitch among them (+0.010, −0.013 against vx's +0.015); its tonic half is an integrator and the head
motors' tonics sit at the rails (neck `h` −2.15, head pitch +1.14). The walker has no head in its state, so no error
reads "head down", and the speed target is never met: **the vx command sits at its rail (0.40 m/s) on 47–68 % of
walking ticks** while the walk reaches ~0.2 m/s. Mid-ranging needs the slow actuator able to carry the steady load;
here only the head can still add speed.

**Three §3.2 catches, found reading the state's layout** (`[pos, act, delta]` per twist joint, then the sense from
index 9): the "tilt prior" on state 3, 4 of the six- and seven-motor configs is sensed lateral speed and the vy
command, not the trunk's gravity (9, 10) — §17.86's "the tilt prior halves contact" is a lateral-speed prior's
result; the host's authority table printed rows 1, 2 as "sensed vy / wz" (they are vx's action echo and delta; fixed,
and the columns now named from the graph's action topics, the lean's sixth column having printed as head_y); and the
prior's model-implied step ignores `state_prior_motors`, so §17.86's mask arms (sweeps 70–72) masked the slow descent
only — a weakened slice of the mask; sweep 80 (yaw the only head motor) is unaffected.

**Built** (off by default; guards byte-identical: R83 plain `6b9a0b3a…`, the starred six-motor preset old build
against new `915e1391…`): `MotorEPMv2.state_prior_weights` (per-index precision, in the descent and as √w rows of the
step's weighted least squares), `MotorEPMv2.state_grow_at` (grow on restore: a wider state's new elements inserted
unidentified — zero model rows, zero controller and state-model columns), host `--intent-head-sense` (head-frame
gravity y, z at the front of the walker's sense, load_slots 18), `scene_babble_room2m.xml` (the 2 m room: a post, a
box, a chair), host `--tof-real SPREAD LAG` (below).

**Stage B, the rebabble.** `duck_contact6_s1` into `a1v2_contact_room6h` with the head sensed: 25 → 27 elements, 600 s
of the structured babble in the 2 m room (2 rescues), saved `duck_contact6h_s1`. The new rows identified (head pitch:
vx −0.021, vyaw +0.021, neck +0.028, head pitch −0.016, head roll −0.024), the old ones re-identified beside them.

**Stage C, the level prior (state 9, 10 → 0) at a swept precision against the speed prior's 1.** The room, n = 3 (300 s):

| 2 m room, seeds 1–3 | old lean brain | sensed, no prior | level 0.3 | **level 1** | level 3 |
|---|---|---|---|---|---|
| pitch per 0.1 m/s · per 0.2 m/s² | +0.26 · −0.04 | +0.10 · +0.03 | +0.01 · +0.08 | **−0.02 · +0.14** | −0.02 · +0.12 |
| acceleration's share of the pitch's variance | 0 % | 0 % | 3 % | **27 %** | 16 % |
| cruise · accelerating · decelerating (rad down) | +0.44 · +0.31 · +0.10 | +0.36 · +0.52 · +0.36 | +0.24 · +0.44 · +0.27 | **+0.18 · +0.36 · +0.12** | +0.12 · +0.20 · +0.01 |
| walk m/s · arrival stops · down % | — | 0.166 · 6.7 · 3.4 | 0.156 · 2.7 · 0.8 | **0.144 · 6.0 · 0.05** | 0.134 · 1.3 · 0.3 |

The playroom, seeds 1–6, 600 s (sweep h3; the old lean brain is sweep 77 on the same seeds and harness):

| playroom, seeds 1–6 | ★ lean, old brain (77) | sensed, no prior | level 0.3 | **level 1** |
|---|---|---|---|---|
| pitch per 0.1 m/s · per 0.2 m/s² | +0.15 · +0.03 | +0.22 · −0.01 | +0.16 · +0.02 | **+0.04 · +0.09** |
| cruise · accelerating · decelerating (rad down) | 0.52 · 0.55 · 0.35 | 0.57 · 0.52 · 0.29 | 0.45 · 0.50 · 0.30 | **0.24 · 0.39 · 0.16** |
| sd of the head's pitch on the walk | 0.33 | 0.37 | 0.33 | **0.20** |
| after an acceleration, pitch at 0 / 1 / 2 s | 0.55 / 0.55 / 0.56 | 0.67 / 0.63 / 0.62 | — | 0.33 / 0.33 / 0.34 |
| falls a run · within 3 s of a stop's start | 2.7 · 2.2 | 2.8 · 2.3 | 2.3 · 1.7 | **1.0 · 0.3** |
| the stop's ringing, p90 (rad/s) · rescues / min | 1.92 · 0.27 | 1.91 · 0.28 | 1.67 · 0.23 | **0.85 · 0.10** |
| the error closed by 0.5 rad within 3 s · closing | 42 % · 53 % | 44 % · 54 % | 45 % · 58 % | **60 %** · 49 % |
| walk m/s · walls / min · arrival stops | 0.197 · 27 · 22.5 | 0.212 · 17 · 19.0 | 0.198 · 24 · 20.7 | 0.175 · 17 · 18.3 |

- **Sensing the head changes nothing by itself** (the no-prior arm leans as before): the error must be given.
- **The level prior at 1 decouples the lean from speed** (its speed coefficient down four fifths in the playroom and
  to zero in the room), halves the head's motion, and **cuts the falls to a third** (2.8 → 1.0 a run, at stops 2.3 →
  0.3; the stop's ringing halved) — the head arrives at a stop nearer level, so the hand-off has less to undo. Costs:
  a sixth of the walk's speed, five points of closing.
- **What it does not do is settle**: after an acceleration the head stays where it is for the next two seconds in
  every arm; the prior moves the head's equilibrium (0.55 → 0.33 rad down) rather than making the lean a transient.
  The rail explains why: the speed error never closes, so the speed prior pushes through the head at every pace.
- **The reachable-target probe (room, n = 3, sweep h4):** the speed target at 0.5 of range without the level prior
  gives a slow partial settle (0.52 → 0.43 rad over 2 s) and the lean still on speed; with the level prior the walk
  crawls (0.117 m/s, 2.3 arrivals); at 0.6 with the level prior it looks like the level prior alone. `NULL` as a lever;
  the re-use is a target adapted from the body's own achieved speed rather than a set one.

**Verdict (n = 6, a signal):** the level prior at 1 is `PARTIAL` on the operator's picture — the lean off speed, half
the motion, a third of the falls — and `NULL` on the settle within a steady pace. The confirmation on seeds 7–18 and
the ToF's real timing are measured next (below).

**The ToF's real timing (`--tof-real SPREAD LAG`, Tof::set_realism; 0 0 = off, byte-identical).** The VL53L8CX builds
an 8×8 frame from four integrations in sequence (datasheet DS14161; 5 ms each by default in autonomous mode, the
VCSEL on for the whole period in continuous mode, at most 15 Hz), and the robot composes a frame with the head pose
it reads on arrival. On: sub-frame k (the 2×2 zone interleave, an assumption about the SPAD groups) is cast from the
sensor's pose LAG + SPREAD·(3 − k)/4 s ago (interpolated between recorded ticks) and reprojected with the current pose;
the JSON stream carries the cast's registration error (`tre`: mean, max, m). The arm measured: continuous mode at
15 Hz (SPREAD 0.066) and a 30 ms readback (LAG 0.03) — both assumptions to confirm against Pollen's driver settings.

**The confirmation, seeds 7–18 (sweep h5, n = 12):**

| seeds 7–18 | head sensed, no prior | **level prior at 1** |
|---|---|---|
| falls a run · within 3 s of a stop's start | 3.5 · 2.6 | **1.2 · 1.0** |
| rescues / min · down % | 0.35 · 0.70 | **0.12 · 0.10** |
| head pitch accelerating · cruise · decelerating (rad down) | 0.57 · 0.59 · 0.39 | **0.39 · 0.17 · 0.18** |
| pitch per 0.1 m/s · per 0.2 m/s² | +0.26 · −0.03 | **−0.07 · +0.15** |
| the error closed by 0.5 rad within 3 s · closing · tangential | 42 % · 52 % · 35 % | 48 % · **41 % · 53 %** |
| walk m/s · walls / min · arrival stops | 0.193 · 31 · 18.2 | 0.162 · 33 · 16.7 |

Pooled over eighteen seeds the falls go 3.3 → 1.1 a run. On the fresh seeds the lean has the operator's shape at the
phase level: 0.39 rad down while the body accelerates, 0.17 at a cruise; the return is complete by the time the
acceleration ends (the settle window, which starts there, reads flat at 0.21). A 0.17–0.2 rad offset at a cruise
remains. The cost is the steering: closing 52 → 41 %, the turn toward a large error slower (median 0.14 → 0.08
rad/s), tangential walking 35 → 53 % — head pitch was a steering motor (§17.86), and a level head gives it back.

**The ToF's real timing (sweeps h6, h7; seeds 1–6; `--tof-real 0.066 0.03`; `mj_host/tools/tre_readout.py`):**

| mean registration error (p90) | on the walk | at stops | casts with the head turning > 2 rad/s |
|---|---|---|---|
| three motors, the still head (`duck_contact3_s1`) | 4.7 cm (9.1) | 2.8 cm | 4 % |
| the lean, head sensed, no prior | 5.4 cm (11.7) | 2.7 cm | 45 % |
| the lean + the level prior at 1 | **10.7 cm (19.6)** | 3.8 cm | 51 % |

By the head's angular speed on the walk (all arms alike): about 2 cm below 0.3 rad/s, 3 cm at 0.3–0.6, 4–5 cm at
0.6–1, 5–9 cm at 1–2, 7–13 cm above 2. **The registration error follows the head's angular SPEED, not its attitude**,
and the level prior, which holds the attitude by moving the head, doubles it. Even the three-motor still head turns at
0.6–2 rad/s on three quarters of its walking casts: the trunk's gait sway carries it. The behaviour under the timing,
n = 6 against the same seeds without it (sweep h3): the sensed lean's rescues 0.28 → 0.63 and arrivals 19.0 → 15.7; the
level lean's walls 16.6 → 10.6 and arrivals 18.3 → 20.2 — mixed, not read further at this power. The numbers scale with
the timing assumed (continuous mode at 15 Hz, a 30 ms readback); the datasheet's autonomous default (four 5 ms
integrations) would shrink the spread, not the readback lag.

**Verdicts (2026-10-01).** Stage A: the lean follows speed (measured; the cause in the saved brain). Grow on restore:
`WORKING` (unit-tested; the rebabble identified the new rows beside the old). The head sense alone: `NULL` (an
observation without an error changes nothing). **The level prior at 1: `WORKING` on falls** (a third, n = 18 pooled,
confirmed on fresh seeds) **and on the lean's shape** (on acceleration, off speed, 0.17 rad at a cruise), `REGRESSION`
on the steering (closing −11 points on the fresh seeds) and on the cloud under the real timing (registration error
×2); `PARTIAL` overall — the eye decides. The reachable speed target: `NULL` (probe). **The design consequence:** the
stability the cloud needs is a head still IN SPACE — its angular rate, which the head IMU senses — gated by when a loop
needs the cloud; "level" is a different error (the view, the balance) and the two conflict on a walking body. Re-use:
the level prior's precision gated by the loop's state (off while turning toward a target, on at a cruise and at stops)
would keep the falls and give the steering back; the gaze-in-space prior (the head gyro to zero, the head brain's H2
"still" slots, the VOR feed-forward of the H line) is the stability stage's error.

### 17.88 The bird's neck: the head slides fore-aft, the view stays level (2026-10-01)

**The operator's eye on the level lean (§17.87):** the head nods fore and aft a lot and it seems to unsettle the walk;
the neck and head pitch joints can move the head forward and back relative to the body's centre of gravity instead of
pitching it, as any bird does while it walks.

**The geometry (forward kinematics of `scene.xml`).** Neck pitch and head pitch tilt the view by equal and opposite
amounts (neck +0.3 rad: the ToF's axis +17°; head pitch +0.3: −17°), so moving both joints TOGETHER keeps the view's
attitude exactly and slides the head fore-aft: at 0.44 rad the head's centre of mass moves 2.0 cm and the robot's 0.76
cm, with 0° of tilt; head pitch alone moves the robot's 0.48 cm and tilts the view 25°. The head is 38 % of the mass.
**A catch:** `--head-forward` (§17.60) adds the same positive offset to both joints, which moves the head BACK (the ToF
−1.0 cm at +0.2 rad) — §17.60's "head forward" was a head-back lever.

**What the nod was (the saved sweeps, the joints split into the two modes).** Translation T = (neck + head)/2, pitch
P = (neck − head)/2. The lean before the level prior used both (sd 0.22 each; only P followed speed, r −0.57). The
level prior shrank the slow pitch (0.22 → 0.13) and GREW the stride-band pitch 40 % (0.061 → 0.086 rad), faster
(p90 0.93 → 1.04 rad/s): the prior corrects the tilt every tick against the stride's bob through a lagging actuator,
the H2 head brain's own lesson (its `state_prior_lr` 0.02). That is the nod the operator saw, and the doubled
registration error of §17.87.

**Built** (off by default; guards byte-identical: R83 `6b9a0b3a…`, six-motor `915e1391…`): host
`--intent-head-translate F [RATE]` — the walker's fourth motor `action.head_fore` slides the head (both pitch joints
by −T, T ≤ F·1.10 rad, rate-limited at RATE rad/s like a servo, centred at stops), written on top of the head brain's
joints (the head brain keeps the tilt: `--head-joints` required), and the walking policy is told it in its head
command; host `--intent-fore-sense` — where the head sits fore-aft leads the walker's sense (load_slots 17).
Identification by the recipe (600 s on the open playroom, 600 s in the 2 m room, seed 1) for both arms:
`duck_fore_s1` (twist + head_fore) and the control `duck_ctrl3_s1` (the twist). The new motor's authority: the trunk's
forward tilt +0.022 (the centre of mass moving), forward speed −0.006 open / −0.001 after the room — the walker
credits a forward head with slowing, not speeding (the policy, told the head, compensates). The neck servo sags
~0.14 rad under the extended head's weight; the head brain's level loop takes it out of the view.

**The playroom, seeds 1–6 (sweeps f3, f4; configs `a1v2_r113_fore`, `…_fore_c03`, `…_fore_c10` = + a centring
prior on the fore-aft slot at precision 0.3 / 1):**

| seeds 1–6 | control: three motors | the bird's neck | + centring 0.3 | **+ centring 1** |
|---|---|---|---|---|
| the view's pitch at a cruise (sd on the walk) | 0.000 (0.029) | 0.000 (0.031) | 0.000 (0.031) | 0.000 (0.032) |
| head fore-aft: accelerating · cruise · decelerating · backing (+ = forward) | −0.02 · −0.02 · −0.02 · −0.05 | **+0.11** · −0.46 · −0.05 · −0.06 | −0.10 · −0.05 · −0.16 · −0.24 | −0.03 · **−0.06** · −0.08 · −0.08 |
| the error closed by 0.5 rad within 3 s · turn toward it (median) | 30 % · 0.03 rad/s | 46 % · 0.13 | 33 % · 0.07 | **47 % · 0.14** |
| closing on the target | 53 % | **65 %** | 56 % | 58 % |
| falls a run · walls / min · arrivals · walk m/s | 2.5 · 26 · 17.5 · 0.175 | 1.5 · 41 ± 40 · 18.2 · 0.186 | 2.8 · 41 · 17.7 · 0.177 | 2.2 · **27** · 17.2 · 0.179 |

In the room (n = 3) the same shape: the view level everywhere (sd 0.015–0.020), the stride-band pitch at the still
head's level (0.018–0.024, against the level lean's 0.086). **The nod is gone and the view is the still head's**;
the translation is a steering motor as the pitch was (§17.86), without tilting the ToF. Without a centring prior the
head reaches forward as the body accelerates and parks back at a cruise (the speed prior's unmet tonic, now in the
translation); at centring 1 it is centred at a cruise and reaches back as the body slows and backs.

**The confirmation, seeds 7–18 (sweeps f5, f6), and pooled over eighteen:**

| | control: three motors | the bird's neck | + centring 1 |
|---|---|---|---|
| seeds 7–18: falls · walls · arrivals · walk m/s | 1.7 · 31 · 17.7 · 0.172 | 3.2 · 28 · 18.6 · 0.188 | 1.6 · 40 · 16.6 · 0.170 |
| seeds 7–18: closing · closed within 3 s · turn median | 48 % · 35 % · 0.07 | 58 % · 39 % · 0.10 | 49 % · 33 % · 0.05 |
| seeds 7–18: head fore-aft accelerating · cruise | −0.01 · −0.02 | **+0.12 · −0.38** | −0.05 · −0.06 |
| **n = 18: falls · walls · closing · closed within 3 s · walk m/s** | **2.0 · 29 · 50 % · 33 % · 0.173** | **2.6 · 32 · 60 % · 41 % · 0.187** | **1.8 · 36 · 52 % · 38 % · 0.173** |

**The ToF's real timing (sweeps f7, f8, seeds 1–6, `--tof-real 0.066 0.03`):** the registration error on the walk
4.7 cm for the still head, **4.8 for the bird's neck, 4.9 with centring** (the level lean of §17.87: 10.7); at stops
2.3 / 2.5 / 2.7. The translation leaves the view where the still head keeps it.

**Verdicts (2026-10-01).** **The nod: solved** — the view as level as the still head's (pitch sd 0.03, stride band
0.02) and the cloud's registration error the still head's under the real timing; `WORKING` on the operator's
complaint. **The bird's neck without a centring prior: `PARTIAL`** — closing +10 points and the walk +8 % over
eighteen seeds (the translation steers without tilting the ToF), the head reaching FORWARD as the body accelerates
(+0.12 rad) and then drifting BACK at a cruise (−0.38: the speed prior's unmet tonic, and the walker credits a forward
head with slowing); falls 2.0 → 2.6 (seeds 7–18: 1.7 → 3.2). **Centring at 1: `NULL`** against the control — the head
centred at a cruise and back as it slows, falls, closing and turning a tie, walls a little worse; the seeds 1–6
steering gain did not replicate. The operator's lean shape (forward into the acceleration, home at a steady pace) is
the no-prior arm's acceleration phase without its cruise; between the two, the centring at 0.3 (seeds 1–6 only) sits
in between. Re-use: the drift is the speed prior's unmet target (§17.87's rail); a centring prior gated by the pace
(on at a cruise, off while the speed error is changing) is the form that would keep the reach and drop the drift —
the same gate §17.87 names for the level prior. Presets "BIRD · …" for the eye.

**The pace gate (2026-10-01, the operator: "proceed with the pace-gated centring").** Built
`MotorEPMv2.state_prior_gated_by` (parallel to the prior's indices: the state element whose steadiness gates each
index; the element's 0.5 s EMA minus its 2 s EMA against that difference's own 30 s RMS, gate = 1 − |d|/rms clamped;
it multiplies the index's descent and its row in the step; empty = byte-identical; unit-tested, 26/26). Guards: R83
`6b9a0b3a…`, six-motor `915e1391…`, and the bird's neck's config identical to the previous build's sweep for its
first 4 847 ticks (the divergence after is the host declining a stop a 150 s run has no time for). Configs
`a1v2_r113_fore_g10 / g30`: the centring prior at 1 / 3 gated by the sensed forward speed (state 0).

| playroom, n = 18 | control | bird, no prior | centring 1 | **centring 1, pace-gated** | gated at 3 |
|---|---|---|---|---|---|
| falls a run | 2.0 | 2.6 | 1.8 | **1.9** | 3.2 |
| closing · the error closed within 3 s | 50 % · 33 % | 60 % · 41 % | 52 % · 38 % | **63 % · 41 %** | 56 % · 40 % |
| walls / min · walk m/s · arrivals | 29 · 0.173 · 17.6 | 32 · 0.187 · 18.5 | 36 · 0.173 · 16.8 | **34 · 0.183 · 18.1** | 33 · 0.179 · 17.1 |
| head fore-aft: accelerating · cruise · decelerating · backing | ≈ 0 | +0.12 · −0.40 · −0.08 · −0.15 | −0.05 · −0.06 · −0.10 · −0.10 | −0.05 · **−0.10** · −0.13 · −0.22 | +0.01 · −0.04 · −0.06 · −0.09 |

The gate reconstructed offline from the logged speed (six runs): open 0.46 on average at a cruise, 0.31 accelerating,
0.20 decelerating — it discriminates, softly (closed on a third of cruise ticks), so the centring at a steady pace
runs at about 0.4 of its weight. **Verdict: the pace-gated centring at 1 is the best bird's-neck arm on the full set
— the steering gain of the no-prior arm (closing +13 points, the error closed within 3 s +8) at the still head's fall
rate, the cruise drift a quarter of the no-prior arm's, the walk +6 %; walls +5 a minute against the control.
`PARTIAL` leaning `WORKING`; the eye decides.** The forward reach into an acceleration did not survive the gate: the
walker identified a forward head as slowing this policy, so nothing asks for it; what the gated arm does is slide the
head BACK as the body brakes and backs — the pendulum's move for a negative acceleration. Re-use: a sharper gate
(the threshold at 2 RMS, or the gate on the speed error's change rather than the speed's) would centre harder at a
cruise; the forward reach needs a body that credits it, which this policy does not.

**Promoted (2026-10-01, the operator's eye):** "this method is promoted. The robot's use of its neck is an overall win
on multiple fronts, including object seeking, voxel cloud clarity and escapes." ★ BIRD = R113 + the bird's neck + the
pace-gated centring (`a1v2_r113_fore_g10`, `duck_fore_s1`, `--intent-head-translate 0.6 --intent-fore-sense`); the
still-head R113 stays as the reference. The cold start is chase phase §11.

### 17.89 Looking toward where the walk is going: the body-frame ToF slots and the gaze (2026-10-01)

**The next move after ★ BIRD (chase phase §11's first open lever, the operator: "start the next move").** A walking
bird holds its head still in space and glances. With ★ BIRD the walker owns the head's fore-aft translation and the
head brain the tilt and yaw; the obstacle to letting the head look around was §17.86's sweep 80 — the walker's
left/ahead/right ToF slots are the sensor's columns, so a turned head turns the walker's "ahead" (walls doubled).

**Built** (host, off by default; the ★ BIRD sweep reproduces byte for byte over 600 s, R83 `6b9a0b3a…`):
`--tof-body MEM` — the three proximity slots from every Hit return of the last MEM s, carried by the odometry into the
current body frame and binned by BODY azimuth from the sensor into the sensor's own sector widths (left 5.6–22.5°,
ahead ±5.6°, right); with the head straight they track the column slots (r 0.95–0.97, means within 0.015).
`--seek-gaze K [RATE [MAX]]` — on the walk the head yaw turns toward the seek loop's target while seek holds the
reference (rate-limited at RATE rad/s, at most MAX rad, home at stops; a first build held a stale bearing for 20 s
when seek let go — fixed before any arm). Instrument `mj_host/tools/gaze_readout.py` (the target inside the ToF's
±22.5° by the body and by the head).

**Seeds 1–6 (sweep z1):** the gaze with the body slots kept the target in view 88 % of seeking time (49 %), turned
toward it twice as fast, closed the error within 3 s 51 % (43), walls tied (26 / 24); the same gaze on the
head-frame slots failed as sweep 80 did (walls 45, falls 3.5, arrivals 13.5) — **the body-frame slots are what make a
turned head possible.**

**Eighteen seeds (sweeps z1–z3, pooled; the ★ BIRD control reproduces byte for byte):**

| n = 18 | ★ BIRD | body slots 0.5 s | body slots + gaze | body slots 0 s | body slots 0 s + gaze |
|---|---|---|---|---|---|
| target in the ToF's field while seeking (head) | 48 % | 40 % | **82 %** | 48 % | **83 %** |
| walls / min | 34 | 28 | 43 | 32 | 54 |
| closing · error closed within 3 s | 63 % · 41 % | 57 % · 38 % | 52 % · 41 % | 57 % · 39 % | 53 % · 39 % |
| falls a run · arrivals | 1.9 · 18.1 | 2.3 · 16.9 | 2.0 · 16.2 | 3.1 · 15.3 | 2.1 · 16.7 |

The seeds 1–6 signal did not hold: the gaze looks (the target in view 82 %) and the walk does not profit (walls up,
closing down). Two causes, each a measurement before a verdict: (1) **the walker was identified on the column slots**
and is being run on body-frame slots — with the head straight and no memory the body slots alone cost falls (1.9 →
3.1) and closing (63 → 57), a model mismatch, not a property of the sense; (2) **geometry**: the gaze turns a median
0.46 rad, and past 22.5° − 5.6° = 0.29 rad the body's own ahead sector leaves the ToF's field, so the avoidance runs on
memory. In flight: ★ BIRD's identification redone with the body slots (`duck_forebody_s1`), then on it the body slots
alone, the gaze bounded at 0.29 rad (`--seek-gaze 1.0 1.0 0.29`) and unbounded, n = 18 (sweep z4).

**The fair test (sweep z4, n = 18):** ★ BIRD's identification redone with the body slots (`duck_forebody_s1`: 600 s
open + 600 s in the 2 m room with `--tof-body 0.5`; the walker's ToF rows identified on the sense it acts on), then:

| n = 18, the body-slot brain | slots alone | gaze bounded at 0.29 rad | **gaze unbounded** |
|---|---|---|---|
| target in the ToF's field while seeking | 43 % | 58 % | **86 %** |
| turn toward it (median) · error closed within 3 s | 0.04 rad/s · 34 % | 0.04 · 33 % | **0.20 · 46 %** |
| closing on the target | 53 % | 49 % | **57 %** |
| walls / min · falls a run · arrivals · walk m/s | 24 · 2.2 · 18.6 · 0.175 | 30 · 2.1 · 15.7 · 0.174 | **22 · 1.7 · 17.8 · 0.181** |

**Verdicts.** The gaze on the column-slot brain: `NULL`/`REGRESSION` — a model mismatch (the walker identified on one
sense and run on another), not a verdict on the gaze. **The gaze on the brain identified with the body slots:
`WORKING`** — better than the same brain without it on every row (the turn toward the target ×5, the error closed
within 3 s +12 points, closing +4, falls −23 %, walls a tie or better), the target in view twice as often. The bound
at 0.29 rad: `REGRESSION` (arrivals −15 %) — the geometric worry was wrong, the 0.5 s memory carries the ahead sector
while the head looks. Against ★ BIRD (another identification, pooled n = 18: walls 34, falls 1.9, closing 63 %, closed
within 3 s 41 %) the gaze arm walls fewer and falls less, closes 6 points less and turns faster; brains from two
identifications differ by their babble as well as their sense, so the eye judges it. Re-use for the learned form:
the gaze's error (the target's bearing in the head's frame) as the head brain's sense with a prior to zero, through
the yaw it now owns on the walk. Presets "GAZE · the body-slot brain + …" (the candidate, its control, the bound).

### 17.90 The gaze, learned in the head brain (2026-10-01)

**The operator: "I like it. Make the gaze learned in the head brain."** The reflex (§17.89) pointed the head yaw at the
seek target's bearing. The learned form gives the head brain the error and lets it find the motion:

- **The sense** (host `--head-gaze-sense`): the head brain's spare 12th sense slot carries the gaze error — where the
  walk is going (the seek target's body bearing while seek steers the reference, straight ahead otherwise) minus the
  head's yaw, in units of the yaw range. Off = the slot stays 0 (★ BIRD byte-identical over 600 s, R83 `6b9a0b3a…`).
- **The identification**: H1/3 (head pitch, roll, yaw) standing at the joints with the slot fed, 700 s, 0 rescues: the
  gaze row is minus the yaw position row (yaw authority −0.063).
- **The error**: an H2 prior driving the gaze error to zero beside the level prior, the model frozen as identified.

**Four failures before it behaved, each diagnosed (n = 18 unless said; the body-slot walker `duck_forebody_s1`):**
1. **Feedback growth.** With the prior's feedback (C) and tonic (h) halves both on, the yaw thrashed (p90 2.7 rad/s;
   falls 4.9 / 7.1 at precision 1 / 3). The saved head brain after 300 s: a +2.41 feedback gain on the gaze error
   (the row's size 1.15 → 7.28) — the descent writes C in proportion to a large, step-like error through a lagging
   servo. A lower precision (0.3: falls 6.5), a head-still prior beside it (10.3) and a servo-rate slew on the head
   joints (14.2, the slew worsened the stops) did not cure it.
2. **A pure reach runs away.** `MotorEPMv2.state_prior_c_weights` (the feedback half's per-index weight; empty =
   byte-identical; unit-tested) at 0 makes the gaze a pure reach through h, as the module's own role note prescribes
   (C balances, h reaches). The yaw ran to its rail (1.3–1.4 rad): under the controller's identity hold on its own
   position (an initialisation, never learned), a tonic is a VELOCITY.
3. **The hold released, still at the rail** (`head3j_gaze_h1_s2_nohold`): the level priors' tonic leaked into yaw
   through H1's cross-couplings (pitch row −0.024 on yaw). A two-seed probe with the level priors off: the target in
   view 91 %, the yaw p90 0.41 rad/s.
4. **Yaw last** (`head3o_gaze_h1_s2`, 0 rescues): the level priors masked to pitch and roll (`state_prior_motors 2`),
   the gaze a pure reach, the yaw's hold released (`head3o_gaze_h1_s2_nohold`, config `head3o_h2_gaze_w10`).

| n = 18 | the same head brain, no gaze (control) | **the learned gaze** | the reflex (§17.89) | the reflex's control |
|---|---|---|---|---|
| target in the ToF's field while seeking | 45 % | **90 %** | 86 % | 43 % |
| yaw speed on the walk, p50 · p90 | 0.09 · 0.29 rad/s | 0.14 · 0.61 | 0.16 · 0.66 | — |
| turn toward the target · the error closed within 3 s | 0.07 rad/s · 41 % | 0.16 · 40 % | 0.20 · 46 % | 0.04 · 34 % |
| closing on the target | 56 % | 57 % | 57 % | 53 % |
| falls a run · walls / min · arrivals | 1.8 · 24 · 17.6 | 2.6 · 33 · 17.4 | 1.7 · 22 · 17.8 | 2.2 · 24 · 18.6 |

**Verdict (2026-10-01).** The learned gaze **looks**: the target in view twice as often as its control, the yaw as
smooth as the reflex's, the head home when nothing is sought — the behaviour is the head brain's own, found through
its identified authority. It does **not yet help the walk**: against its own control closing and the error closed
within 3 s tie, walls +9 a minute, falls +0.8 (at stops 0.6 against 0.3). `PARTIAL`. The reflex remains the better
walker (its control: the error closed +12 points, falls −0.5). What differs between them is the shape of the
command: the reflex is a proportional position at a fixed rate; the learned reach is an integral one, which lags when
the target switches and must unwind when seek lets go. Re-use: a proportional term the head brain can earn (the
feedback half at a small weight, now that the level tonic no longer drives yaw), and the stop's take-over from a
turned head (slewed from where the gaze left it). Presets "GAZE · LEARNED …" and its control.

**The two follow-ups (the operator: "add the proportional term and the stop slew"; sweeps z12, z13, n = 18).**
`--head-stop-slew RATE` (`HeadAdapter.set_release_slew`): during a stop, until the look's override takes the yaw, the
yaw slews home at RATE; when an override lets go, the yaw slews back to the head brain's command instead of stepping
(off = byte-identical; ★ BIRD and the learned gaze reproduce over 600 s). The proportional term: the gaze prior's
feedback half at weight 0.03 / 0.1 (`head3o_h2_gaze_p03 / _p1`).

| n = 18 | learned gaze | + stop slew | + P 0.03 | + P 0.1 | + P 0.1, slew | + P 0.03, slew | control (no gaze) |
|---|---|---|---|---|---|---|---|
| falls a run · within 3 s of a stop | 2.6 · 0.6 | 3.1 · **0.2** | 2.4 · 0.8 | 7.4 · 1.5 | 4.9 · 0.6 | 2.8 · 0.7 | 1.8 · 0.3 |
| walls / min · arrivals | 33 · 17.4 | 34 · **19.0** | **26** · 16.7 | 37 · 15.9 | 37 · 16.0 | 34 · 16.7 | 24 · 17.6 |
| closing · the error closed within 3 s | 57 · 40 % | 55 · 44 % | 52 · 41 % | 50 · 47 % | 49 · 36 % | 51 · 40 % | 56 · 41 % |
| yaw speed p50 · p90 (rad/s) | 0.14 · 0.61 | 0.16 · 0.84 | 0.27 · 0.95 | 0.74 · 2.88 | 0.75 · 2.50 | 0.36 · 1.17 | 0.09 · 0.29 |

**Verdicts.** The stop slew: `WORKING` on its own target (falls at a stop's start 0.6 → 0.2, under the control's 0.3;
arrivals the best of any arm) and `NULL` on the walk (walk falls up). The proportional term at 0.1: `REGRESSION` — the
gain grows again and the yaw thrashes, as in §17.90's first form. At 0.03: walls back to the control's level (26 against
24), the rest a tie. The two together: `NULL` — neither keeps its gain (at n = 18 the stop-fall counts between 0.2 and
0.8 are near the noise). **The learned gaze stays `PARTIAL`**: it looks as the reflex does and does not yet help the
walk against its own control. What the series shows about the module, for the next form: the feedback half grows
without a bound on a large error (0.1 thrashes, 0.03 is mild), so a learned proportional gain wants a bound the module
can earn — the model's own one-step correction (`state_prior_step_gain`, the per-tick least-squares command) is that
proportional form, and it is the lever not yet tried here.

**The per-tick step as the gaze's proportional term (the operator: "the learned gaze is working well … try the
per-tick step"; sweep z14, n = 18):** the model-implied step (`state_prior_step_gain` 1 / 0.3: each tick the
least-squares command over the prior's rows of the identified model, added pre-tanh) on the learned gaze: falls 3.0 /
3.2 (the learned gaze 2.6), walls 33 / 38 (33), closing 50 / 52 % (57), the error closed within 3 s 42 / 40 % (40),
the target in view 93 / 91 %, the yaw p90 1.70 / 0.87 rad/s (0.61). `NULL`. The learned gaze stays as §17.90 left it
(the pure reach; the stop slew the one follow-up that worked, at the stop).

### 17.91 The turning radius: the body can turn; the walker does not aim its turn (2026-10-01)

**The operator:** the bigger problem is the very wide turning radius of the walking intention — there is still a lot
of orbiting while the duck homes in on an area of interest; it may need to walk backwards and turn, or something
else; gains in wall avoidance and tracking may wait on it.

**The walk's turning (`mj_host/tools/turning_readout.py`, the body-slot walker, n = 18).** With the reference more than
1 rad off the nose the walker commands its forward speed at the rail (0.39 m/s on 64–83 % of ticks) and a yaw of
0.73–0.78 rad/s; the body yaws at 0.19–0.22 rad/s, forward 0.04–0.06 m/s — a 0.27 m radius, a half turn in ~15 s.

**The body's own turning, open loop** (the walker's command fixed by `--l2-twist`, 40 s each, the true yaw rate from the
simulator; the odometry and the sensed rate agree with it to 0.01 rad/s):

| forward \ yaw command (rad/s) | 0.5 | 1.0 | 1.25 | 1.5 | 2.0 | 2.5 | 3.0 | 4.0 |
|---|---|---|---|---|---|---|---|---|
| in place | 0.00 | **0.00** | 0.33 | 0.48 | 0.86 | 1.26 | 1.68 | 2.17 |
| 0.1 m/s | 0.00 | 0.29 | 0.28 | 0.38 | 0.68 | 1.05 | 1.44 | 2.19 |
| 0.2 m/s | 0.15 | 0.47 | 0.59 | 0.63 | 0.66 | 0.88 | 1.22 | 1.93 |
| 0.4 m/s | 0.26 | 0.58 | 0.69 | 0.81 | 1.04 | 1.29 | 1.31 | 2.27 |
| backing 0.2 m/s | 0.00 | 0.55 | 0.67 | 0.84 | 1.30 | 1.69 | 1.89 | 2.03 |

No falls in any. **The walking policy does not turn in place below a yaw command of ~1.25 rad/s (a standing deadband)
and turns near-linearly above it to 2.2 rad/s; the walker's yaw range of ±1.0 (`kTwistRangeVyaw`, §17.86's
"unverified" cap) lies entirely inside the in-place deadband.** Backing, it turns at 1.0.

**The range opened (`--twist-yaw-range R`; the command, the sensed yaw's unit and the heading reflex's clamp; 1.0 =
byte-identical; ★ BIRD, the learned gaze and R83 reproduce):** ★ BIRD's body-slot walker re-identified at ±3.0 by the
same recipe and seed (`duck_yaw3_s1`), n = 18 against the matched range-1 walker (`duck_forebody_s1`): the body yawed
LESS (0.08–0.09 rad/s at large errors against 0.19–0.22), falls 3.9 against 2.2, walls 22 against 24, arrivals 15.4
against 18.6. `REGRESSION` — and the reason is not jitter (the command flips sign on 4–5 % of ticks at either range):
**with the reference more than 1 rad off, the yaw command points toward it only 52–58 % of the time.**

**Where the aim is lost** (the matched range-1 walker): on those ticks the heading reflex (`--heading-reflex 1.0 0.3
1.0`) has its full share on 20 % — and then the command points toward the reference 98 % of the time; on the other 80 %
something is within the reflex's 1 m gate and the yaw is handed back (in part or whole) to the walker's brain, whose
command then points toward the reference 40–50 % of the time — part of it legitimate avoidance (a wall between the
duck and its target), the rest the orbit. **The orbit is the walker's heading authority under clutter, not the body's
turning speed.** In flight: the heading prior's precision (`state_prior_weights` on index −6 at 3 and 10, range 1,
sweep y2).

**The heading prior's precision (sweeps y2, y3, n = 18).** At 10 on the range-1 walker: with the target 1–2 rad off the
forward command falls to 0.13 m/s and at 2–4 rad to −0.17 m/s — **the walker sheds speed and backs when the target is
behind**, the operator's "walk backwards and turn" from the heading error alone — but its yaw command (~0.8 rad/s) sits
in the policy's in-place deadband, so the body turns at 0.25–0.29 rad/s; the aim 55 → 62 %, the error closed within 3 s
34 → 45 %, walls 24 → 38, falls 2.2 → 2.5. At 3: the aim 60 %, falls 3.8. On the range-3 walker both precisions: the
body yaws 0.07–0.10 rad/s, the aim 50 %, falls 3.7–4.9, arrivals 11–15. `PARTIAL` (the backing) / `REGRESSION`.

**Why the yaw does not aim (the walker saved after 300 s, seed 1).** The yaw command flips sign every ~0.4 s (5 % of
ticks at either range): a limit cycle. The walker's yaw row of C doubles in 300 s (|C| 9.7 → 22.4): −4.05 on its own
sensed yaw rate, +2.85 on its own previous command, ±2.2 on the side ToF slots, −2.15 on the heading — the feedback half
growing on large, step-like errors through the policy's lag, §17.90's disease in the walker. Its yaw tonic winds to
the rail (h −0.93 → −2.88, tanh −0.99): a small yaw command turns nothing (the deadband), the heading error persists,
the integrator pushes on. And at range ±3 the identification found the yaw's authority over the heading error with the
WRONG sign (A −0.12; the range-1 identification +0.03): full-scale yaw pulses spin the body further than the heading
sense (a deviation from a 60 s mean, clamped at ±π) can follow. **The turning radius is the walker's yaw loop — a
deadband its linear model cannot see, a tonic that winds in it, a feedback half that grows — not the body.**

**★ GAZE promoted (2026-10-01, the operator's eye):** "the learned gaze is actually working well; it is an interesting
behaviour, and we should continue with it." ★ GAZE = ★ BIRD's body-slot walker (`duck_forebody_s1`, `--tof-body 0.5`)
+ the learned gaze (`--head-graph head3o_h2_gaze_w10 --load-head head3o_gaze_h1_s2_nohold --head-gaze-sense`), §17.90's
form (no stop slew). Preset "★ GAZE · the LEARNED gaze". The turning work of §17.91 builds on it.

### 17.92 The three steps on the turning radius: the yaw motor calibrated, the heading as a reach, the walker re-identified (2026-10-01)

**The operator:** "consider the learned gaze promoted, then proceed with the next three steps; I'll review after you
have results on all three."

1. **The yaw motor's calibration** (host `--yaw-linearize`): the walker's yaw output is a DESIRED yaw rate, mapped
   through the inverse of the walking policy's measured open-loop response (the §17.91 table, made monotone, interpolated
   in the commanded forward speed) to the policy's yaw command — the in-place deadband compensated, an actuator
   calibration taken from the body, not a behaviour. Open loop it delivers what is asked, in place 0.33 / 0.59 / 1.00
   rad/s for 0.3 / 0.6 / 1.0, walking 0.31 / 0.61 / 0.99, backing 0.48 / 0.63 / 0.96, no falls. Off = byte-identical
   (★ BIRD, ★ GAZE over 600 s, R83 `6b9a0b3a…`). The logged `twist` carries the policy's command (calibrated).
2. **The heading as a pure reach** (`state_prior_c_weights` 0 on index −6), with a precision-10 variant.
3. **The re-identification**: ★ BIRD's walker on the calibrated yaw, 3 s babble pulses (`a1v2_contact_room4t_h150`, a full
   pulse turns ≤ ~3 rad, inside the heading sense's ±π), open then the 2 m room, 0 rescues (`duck_lin_s1`). The yaw's
   authority over the sensed yaw rate +0.016 (the earlier walkers 0.002–0.003) and over the heading error +0.032 (the
   right sign, its row's largest).

**Results (sweep t1, n = 18, ★ GAZE's stack; the control ★ GAZE, sweep z10, the same seeds):**

| n = 18 | ★ GAZE | **linear yaw** | + heading reach | + reach at precision 10 |
|---|---|---|---|---|
| seconds a run with the target > 1 rad off | 69 | **34** | 36 | 38 |
| the error closed by 0.5 rad within 3 s | 40 % | **55 %** | 54 % | 54 % |
| bearing error, median · the target in the body's ToF field | 28° · 39 % | **19° · 60 %** | 20° · 56 % | 20° · 56 % |
| body yaw rate at 1–4 rad of error · radius | 0.24 rad/s · 0.30 m | **0.31–0.33 · 0.22 m** | 0.33–0.36 · 0.22 m | 0.32–0.41 · 0.22 m |
| walls / min | 33 | **23** | 25 | 39 |
| falls a run · arrivals · closing | 2.6 · 17.4 · 57 % | 2.9 · 17.3 · 53 % | 2.9 · 17.6 · 56 % | 4.3 · 16.3 · 47 % |

**Verdicts.** The calibrated yaw motor with its re-identification: **`WORKING`** — the time spent with the target well
off the nose halved, the error closed within 3 s +15 points, the bearing error a third smaller, the target in the
body's own ToF field half again as often, walls −30 %; the cost a third of a fall a run and four points of closing.
The heading as a pure reach: `NULL` on top of it (a tie: once the motor is linear the heading's feedback no longer
runs away). Precision 10: `REGRESSION` (walls 39, falls 4.3). The body still yaws at ~0.3 rad/s at large errors
against the 1 rad/s it can deliver: the walker asks for more (policy commands 1.4–1.7) but its forward command stays at
the rail on 62–82 % of those ticks, and walking forward at the rail the calibrated map asks the policy for a turn the
policy delivers more slowly than in place. Re-use: the speed prior's precision gated by the heading error (on when
the target is ahead) would let the walker stop and turn in place — the backing §17.91's precision 10 found, without
its falls. Presets "TURN · …".

### 17.93 The walk that faces its thing: the speed target gated by the heading error (2026-10-01)

**The operator, watching the linear yaw (§17.92):** it approaches well and is much less accurate with objects — in the
previously promoted configuration it consistently kicked the ball or block, and a peck had a very good chance of the head
touching the object, which is the goal. **Measured** (`mj_host/tools/skill_align.py`, the true bearing at each skill's
start): the linear yaw arrived still turning (the body's yaw rate in the half second before an arrival stop 0.37 rad/s
against 0.24) and the thing sat 61° off the nose at a kick or a peck (★ GAZE 41° / 46°); kicks started within 0.15 m and
30° fell 38 → 13 %. The kick and the peck fire straight ahead; the arrival fired on range alone.

**Built:** `MotorEPMv2.state_prior_target_gated_by` (a prior's TARGET scaled by `1 − |x_j| / rms_j`, rms_j the element's
own running RMS; empty = byte-identical; unit-tested, 28/28). Config `a1v2_r113_fore_g10_facing`: the forward-speed
prior's target gated by the heading error — the speed the walker asks for falls as the target leaves the nose (gating
the precision instead would leave the speed tonic holding the command at the rail). The arrival rule is unchanged.
Guards: ★ BIRD, the linear-yaw candidate and R83 reproduce.

| n = 18 (sweep f9) | ★ GAZE | linear yaw | **+ facing** |
|---|---|---|---|
| the thing at a kick's · a peck's start, \|bearing\| median | 41° · 46° | 61° · 61° | **27° · 22°** |
| kicks · pecks started within 0.15 m and 30° | 38 % · 28 % | 13 % · 24 % | **47 % · 48 %** |
| the head (or body) touching the thing during a peck | 32 % | 38 % | **57 %** |
| kicks · pushes that moved their thing > 5 cm | 31 % · 31 % | 26 % · 26 % | 33 % · 35 % |
| walls / min · falls a run · rescues / min | 33 · 2.6 · 0.26 | 23 · 2.9 · 0.29 | **14 · 1.5 · 0.15** |
| the error closed within 3 s · seconds with the target > 1 rad off | 40 % · 69 | 55 % · 34 | **70 % · 17** |
| the target in the body's ToF field · bearing error median | 39 % · 28° | 60 % · 19° | **75 % · 15°** |
| arrivals · walk m/s · closing | 17.4 · 0.176 · 57 % | 17.3 · 0.176 · 53 % | 15.2 · 0.163 · 36 % |

With the target 1–4 rad off, the walker's forward command is −0.25 to −0.30 m/s: **it backs while it turns** (the
operator's "walk backwards and turn", from the error alone), and the arrival stop catches a body that has stopped turning
(0.24 rad/s, ★ GAZE's level). **Verdict: `WORKING`** — the alignment at the skill restored and beyond ★ GAZE's, the peck's
touch 32 → 57 %, walls and falls the campaign's best on this stack; the cost a sixth fewer arrivals and a slower walk
(more of it is turning in place: closing reads 36 % with 60 % tangential). The pecks still rarely MOVE their thing
(16 %) — a peck reaches the thing, and the reach is by contact. Preset "TURN · the linear yaw + the walk that FACES its
thing (the candidate)".

### 17.94 The walk's dynamic range: turn before you arrive (2026-10-01)

**The operator, watching §17.93's facing walk:** accurate, but it shuffles in place a lot — not as good as the promoted
configuration; split the difference: keep the speed and the motion while closing in — "increase the dynamic range of its
walking behaviour".

**The shuffle's cause:** the RMS form of the target gate tightened itself — as the walk improved, the heading error's
RMS shrank (65 → 35°) and the speed was halved at 18° and stopped at 35°; the body moved at 0.04 m/s with the target
dead ahead (★ GAZE 0.11). **Geometric gates** (`state_prior_target_gate_cos` / `_pow`: the speed target × max(0, cos e)^p,
the speed that closes on the target): cos restored the walk (0.175 m/s, arrivals 20.3, the campaign's most) and lost the
alignment (the thing 48° off at a peck, touch 31 %); cos² sat between (touch 47 %, arrivals 17.6). Neither depends on how
far the thing is, and a 30° error matters only close in.

**Built:** the walker senses the distance to its target (host `--intent-range-sense`: the seek target's range / 2 m, 1 with
no target, after the fore-aft slot; the linear-yaw brain grown on restore 26 → 27); `MotorEPMv2.state_prior_target_gate_reach`
/ `_k`: the target's gate is min(cos e, clamp(k·range / |e|, 0, 1)) — the time to turn the error (|e| / ω) must not
exceed the time to arrive (r / v), k = 2 m · ω / (π · 0.3 m/s). Unit-tested (30/30); ★ BIRD, the cos arm, R83 reproduce.

| n = 18 (sweep f11) | ★ GAZE | facing (§17.93) | cos | **reach, k 0.64** | reach, k 2.1 |
|---|---|---|---|---|---|
| walk m/s · arrivals | 0.176 · 17.4 | 0.163 · 15.2 | 0.175 · 20.3 | **0.174 · 17.8** | 0.175 · 19.5 |
| the head (or body) touching the thing during a peck | 32 % | 57 % | 31 % | **56 %** | 41 % |
| the thing at a peck's start, \|bearing\| median | 46° | 22° | 48° | 37° | 38° |
| kicks touching · moving their thing | 37 · 31 % | 25 · 33 % | 34 · 29 % | **36 · 30 %** | 24 · 29 % |
| walls / min · falls a run | 33 · 2.6 | 14 · 1.5 | 24 · 2.1 | **18 · 1.8** | 24 · 1.9 |
| the error closed within 3 s | 40 % | 70 % | 56 % | 56 % | 58 % |

**Verdict: `WORKING`** at k 0.64 (ω = 0.3 rad/s, the turn the walker achieves at large errors): ★ GAZE's speed and
arrivals with the facing walk's peck (the touch 32 → 56 %), walls about half, falls 2.6 → 1.8. k 2.1 (ω = 1 rad/s, the
calibrated yaw's open-loop capacity) is too lenient: the walker does not turn as fast as the motor can on the walk. The
kicks' alignment is unchanged (the kick side follows the thing's bearing; touch and displacement tie ★ GAZE). Preset
"TURN · the walk's DYNAMIC RANGE: turn before you arrive (the candidate)".

### 17.95 The first minute: the green block, and the dynamic range on ★ GAZE's own walker (2026-10-02)

**The operator:** "a lot of my evaluation is based on the first minute" — with ★ GAZE the duck walks forward confidently,
engages the green block, comes very close to kicking it and pecks most of the time; with the dynamic range it misses
most of the time. Study the first minute and the first interaction with the green block ahead at the start.

**The instrument** `mj_host/tools/first_minute.py` (the first S seconds, one object by name — `obj_block0`, the green
block 1.4 m ahead; the room is fixed, the seeds vary the start pose and the brain's randomness): the first stop at it, every
skill begun with it the nearest movable within 0.35 m — its edge distance and true bearing, the robot's contact during
the skill, its displacement.

**The finding** (n = 18, the first 60 s; the bearing of the green block at the first skill begun at it): ★ BIRD 2°, ★ GAZE
26° (the block touched in 13/18 runs), and **every arm on the linear-yaw walker 55–86°, always to the right** (the
calibrated walker, the heading reach, cos, cos², the dynamic range at k 0.64 and 2.1); the facing walk reaches it in only
3/18. Traced on seed 1 (identical until the walk begins at 14.7 s): the calibrated yaw makes the first correction a real
turn twice as strong (+0.60 against +0.32), the error swings +18° → −24° and the recovery comes inside half a metre —
the duck passes the block on its left. An underdamped heading loop: before the calibration the deadband swallowed the
small commands. Damping the heading reflex (rate damping 1.0) took the first-minute bearing 69 → 43°; a 2 s time
constant 59°.

**The fix that kept ★ GAZE's approach:** the deadband exists only in place (walking, the raw yaw turns the body), so
the calibration is applied only while the forward command is low — host `--yaw-linearize-below 0.15` — on ★ GAZE's own
walker (`duck_forebody_s1`, grown by the range sense) with the dynamic-range gate (`a1v2_r113_fore_g10_reach_gaze`):
★ GAZE's walking yaw is untouched; when the gate has slowed the duck to square up, the yaw turns it in place.

| n = 18 (sweep f12, 600 s; first minute from the same runs) | ★ GAZE | the dynamic range, linear-yaw walker | **the dynamic range on ★ GAZE's walker** |
|---|---|---|---|
| first minute: runs touching the green block · moving it | 13 · 4 | 11 · 3 | **14 · 5** |
| first minute: the first skill at it, bearing · touching | 26° · 7/14 | 69° · 6/14 | **26° · 10/14** |
| the thing touched during pecks · kicks | 32 % · 37 % | 56 % · 36 % | **51 % · 49 %** |
| kicks · pushes that moved their thing | 31 % · 31 % | 30 % · 36 % | 33 % · 43 % |
| arrivals · walk m/s | 17.4 · 0.176 | 17.8 · 0.174 | **20.2 · 0.177** |
| walls / min · falls a run | 33 · 2.6 | 18 · 1.8 | 33 · 3.2 |

**Verdict: `WORKING`** — ★ GAZE's first minute kept and bettered (the first skill touching the block 7/14 → 10/14), the
touches up across the run, the most arrivals of any arm, the speed kept; the cost half a fall a run (0.9 at stops). The
linear-yaw walker: `REGRESSION` on the first approach (its gains elsewhere — walls 18 — stand, re-use: a heading loop
damped for the calibrated motor). Preset "TURN · the dynamic range on ★ GAZE's walker (the candidate)".

**★ TURN promoted (2026-10-02, the operator's eye):** "we can promote this approach. We still need to address underlying
issues with navigation, but the current body mechanics seem to be the best so far." ★ TURN = ★ GAZE (★ BIRD's body-slot
walker `duck_forebody_s1` with the learned gaze) + the walker's range sense (`--intent-range-sense`) + the dynamic-range
speed target (`a1v2_r113_fore_g10_reach_gaze`: min(cos e, 0.64·range/|e|)) + the in-place deadband compensated only while
slowed (`--yaw-linearize-below 0.15`). The cold start for the next push is chase phase §12.

### 17.96 The ten-minutes phase opens: where ★ TURN's ten minutes go (phase 0, 2026-10-02)

**The operator's direction** (`microduck_ten_minutes_phase.md` §1): ten minutes of interesting behaviour for the plan
to Pollen; the run is boring when the duck gets stuck in corners and stares at the wall. Two pushes — tall structure
consolidated as one immovable object, and the phantom (a lost mover held by a slow loop). Before any lever, the
instrument: `mj_host/tools/ten_minutes.py` sorts every tick into one category (truth labels, scoring only) and traces
every boring episode ≥ 3 s to what began it. On ★ TURN's own sweep (f12, n = 18 × 600 s, the train room, judged from
30 s):

| category | s a run (± sd) | share |
|---|---|---|
| **stare@structure** (standing, wall or furniture within 0.5 m along the view, nothing movable in the cone) | **104 ± 87** | 18 % |
| **seek→structure** (walking to a held target that is a wall or furniture face) | **80 ± 33** | 14 % |
| pinned (wall contact, or stalled by structure) | 49 ± 24 | 9 % |
| seek→nothing | 13 ± 16 | 2 % |
| stand@thing · seek→thing · skill · chase | 139 · 44 · 14 · **1.6** | 24 · 8 · 2.5 · 0.3 % |
| stand-open · wander · down | 62 · 53 · 11 | 11 · 9 · 2 % |

**Boring 43 % of the run (20–72 % by seed), interesting 35 % (15–56 %).** Contingency: skills touching a thing 57/161;
stops begun at a thing 229/395.

- **The seek loop's targets: 94 of 248 arrivals (38 %) are at a wall or furniture face**, 136 at a thing, 18 at
  nothing (the believed target put into the world through the odometry's own pose; checked: the body-to-target
  distance equals the logged seek range at every arrival). The walk to them is the 80 s of seek→structure, and the
  stand at them 37 s of the stares (episodes begun by `stop:arrive`). The operator's diagnosis, measured: the
  fragments of tall structure are the duck's targets four times in ten.
- **The long stares are stops that run to the 60 s cap.** 30 stops of 395 reach the cap; stare episodes of 30 s or more
  are 47 s a run, concentrated on five seeds (5, 11, 12, 13, 14: boring 55–72 %); the look after a skill (`stop:look`,
  29 s a run of stares) and the timer stop (`stop:start`, 17 s) end facing structure as often as the arrival does. A
  stop facing a wall does not end on the cloud's growth rule.
- **The chase is nearly absent on ★ TURN: 1.6 s a run.** The phantom (phase 2) starts from a confirmed chase; its entry
  must be re-measured before the phantom is built (the chase's start, §17.68, and whatever ★ TURN's walk changed).

**Verdict: instrument `WORKING`; the baseline for the phase.** Phase 1's consumers, in the order these numbers rank them:
the approach to a fragment (seek→structure + the arrival stares, ~80–120 s a run), then the stop that does not end facing
structure (~47 s a run in long stares). Re-use: `ten_minutes.py --json` for every arm of the phase, beside the first
minute.

### 17.97 Structure, S0–S1: the fragments are clusters whose top was never seen; a small thing needs a seen top (2026-10-02)

**S0, offline (`mj_host/tools/structure_rule.py`, the f12 logs, 942 small clusters on 1 018 filed clouds).** The operator's
consolidation as built geometrically — the tall footprint closed with the ToF's zone spacing (0.098 rad × range) as the
radius, a small cluster ON the closed line a fragment — keeps every open-floor thing and refuses only 25–47 % of the
structure fragments (12 % of things by walls at its widest tolerance); R91's isolation on the same clusters refuses 55 %
and half the things by walls. The missed fragments sit 2–3 voxels in FRONT of the tall line on walking clouds (the
registration smear grows with range) — and on stop clouds **there is no tall line to find**: in 16 of 28 sampled stop
clouds holding a fragment, nothing above 14 cm was ever seen (the stop's gaze pitched down at a thing). The stack rule
calls a cluster small because its chain stops — and it stops where the field of view does. **A missing observation, not
a missing consolidation** (CLAUDE.md §1 step 2): small should mean *a ray passed over it*. The offline proxy for that
(farther voxels on the same bearing) cannot see the rays that returned nothing, so the rule went into the module.

**S1, built (off by default, both guards byte-identical: the old and new binaries on ★ TURN's argv with every flag off —
tick records identical; the passive arm reproduces f12 seed 5 tick for tick):** host `--tof-free-rays` (each EMPTY zone's
ray end at 4 m appended to the cast, `TofZone::far_level`); `CloudMap.free_rays` (every ray walked from the origin — a
returning ray to 1.5 voxels short of its return, an empty ray to `max_range` — and per column the highest free sample;
each cluster carries `seen_above`; the filed `things` record its 10th value) and `CloudMap.small_needs_top` (small only
if `seen_above ≥ top + one voxel`). Unit test `ACubeIsSmallOnlyOnceARayHasPassedOverItsTop` (25/25).

**The module's own verdicts (passive arm, n = 18):** refused for an unseen top — structure fragments **65 % at stops**,
32 % on the walk; open-floor things **0 % at stops**, 1.4 % on the walk; things within 0.3 m of structure 8 %.

**The lever (sweep `log/ten/s1`, n = 18 × 600 s, ★ TURN's harness, against the passive arm = ★ TURN):**

| | ★ TURN | **+ small needs a seen top** |
|---|---|---|
| walls / min (paired) | 32.7 ± 21.1 | **15.4 ± 16.9** (Δ −17.4, t −3.4, 12 of 18 seeds better) |
| boring · interesting share of the run (`ten_minutes.py`) | 43 % · 35 % | **25 % · 47 %** |
| boring by seed (min–max) | 20–72 % | 5–55 % |
| stare@structure · seek→structure · pinned, s a run | 104 · 80 · 49 | **45 · 42 · 34** |
| stand@thing · seek→thing, s a run | 139 · 44 | **183 · 64** |
| arrivals at a thing · at structure · at nothing | 136 · 94 · 18 | **167 · 32** · 34 |
| skills touching a thing · stops begun at a thing | 57/161 · 229/395 | 65/161 · 291/394 |
| falls a run · chase s a run | 3.2 · 1.6 | 2.1 · 4.1 |
| first minute: touched the green block · first skill touching | 14/18 · 10/14 | 14/18 · 9/14 (a tie) |

**Verdict: `WORKING`, loud** — the operator's boring time nearly halved, arrivals at structure cut by two thirds with more
at things, walls halved, falls down, the first minute kept. Blind metric read: arrivals at nothing rose 18 → 34 (a seen
top on a fragment that is no longer there, or a dead-reckoned place); stand-open +20 s. Not promoted without the eye;
preset "T1 · TOP SEEN". Next (one at a time): the walking cloud's 32 % (the smear in front of the line — the closing with a
range-scaled tolerance, S0's numbers), then the stop that does not end facing structure (§17.96's long stares, 45 s a run
left), then S3 (the outcome loop learns structure does not answer).

### 17.98 The stop that stares at a wall: its own cloud never opened (S2c, `--stop-is-still`; 2026-10-02)

**The operator's eye on T1 (§17.97):** seed 1 stuck against a wall, staring, from ~291 s. **Why a stop facing a wall runs
to the cap:** all 41 stops of 40 s or more on the T1 sweep (34 of them the full 60 s) held a WALKING cloud for 99 % of
their casts, against 59 % for the stops that ended. The growth rule (`--stop-cloud-end`) judges only a stop's own cloud
(R74's lesson), and a stop's cloud opens only after 25 consecutive casts the host calls still: gravity level AND every
trunk-gyro component under 0.15 rad/s. Replayed with the new instrument `--log-still` (seed 1, 288–350 s, reproducing
the sweep tick for tick): the gaze sweep rocks the stand at a median 0.168 rad/s, 44 % of stop ticks read still, the
longest still run 39 ticks — every run of 25 "moving" ticks files the stop's cloud for a walking one. The stop never
gets a cloud whose growth can be judged.

**The lever (host `--stop-is-still`, off by default; guard byte-identical to the T1 sweep's seed 5, 600 s):** during a
stop's standing phases (the joint brain stands, or the walker holds) the cast is still whatever the gyro says — the
body commanded the stop (efference, not an oracle).

| T1 (n = 18 × 600 s, sweep `log/ten/s2`, the same seeds) | T1 · TOP SEEN | + the stop is still |
|---|---|---|
| stops a run · median · p90 length | 23.0 · 8.1 s · 37.8 s | 31.1 · 7.3 s · **11.2 s** |
| stops at the 60 s cap (all runs) · stop-seconds a run | 35 · 304 | **0** · 208 |
| stare@structure · seek→structure · pinned, s a run | 44 · 42 · 34 | **24** · 69 · 48 |
| stand@thing · seek→thing · skill · wander, s a run | 183 · 64 · 15 · 75 | 119 · 92 · 20 · 111 |
| boring · interesting (`ten_minutes.py`) | 25 % · 47 % | 31 % · 41 % (6 of 18 seeds less boring) |
| arrivals at a thing · structure · nothing | 167 · 32 · 34 | 221 · **71** · 44 |
| walls / min · falls a run | 15.4 · 2.1 | 17.4 · 2.2 |
| first minute: touched · MOVED the green block · first skill bearing | 14/18 · 4/18 · 30° | 16/18 · **10/18** · 15° |

**Verdict: `WORKING` as a mechanism (no stop reaches the cap; stares halved), `REGRESSION` on the boring share at this
power, not promoted.** The time the stares gave back goes to walking — and much of it to structure: the stop's own cloud
now opens on every stop, a stop's cloud is where 73 % of static targets come from (§17.78), and `small_needs_top` refuses
65 % of a stop cloud's fragments, not all; the shorter stops let the remaining fragments become the next target. Seed 1
(the preset's) is the loop in full: stares 93 → 152 s with no capped stop at all, seek→structure 22 → 120 s — stop briefly
at a fragment, be handed the next. The first minute improves (the block moved in 10 of 18 runs). **Re-use:** once the stop
cloud's remaining fragments are gone (the line on a seen wall, S0's tolerance; or S3, the outcome loop learning that
structure does not answer), this lever's freed time should go to things — retry it on top of either.

**The operator's test seed 2060249272 (one run per arm, a signal):** no stare at all (18 → 0 s), skills 8 → 17 (touching
2 → 7), arrivals at things 11 → 23 with none at structure in either arm, interesting 56 → 55 %, boring 12 → 20 % (the walk
to nothing and pinned time); chases 10 → 8.

**The operator's eye (2026-10-02):** "the stop-is-still config is not a regression, but there is still long dwell time
near walls" — the verdict on the eye is `PARTIAL`, and **T1 + `--stop-is-still` is the base for the next levers**. The
operator's next three: the outcome loop learns that structure does not answer (S3); investigate the line rule; and the
robot should learn to LOOK UP to see whether an obstacle impeding its movement is a wall, so it is backed away from.

### 17.99 The line on a seen wall (S1b), the outcome loop that learns structure (S3), and the base T1 + the stop is still (2026-10-02)

**The base** is T1 · TOP SEEN + `--stop-is-still` (§17.98, the operator's eye: not a regression), sweep `log/ten/s2`; every
arm below is one lever on it, the same 18 seeds, 600 s.

**S1b, the line — investigated offline on what `small_needs_top` leaves** (`structure_rule.py --line-after-top`, the base's
filed clouds): the module's small clusters that are really structure are fragments whose top WAS seen — the wall is in the
cloud, smeared two or three voxels in front of its tall line by registration (§17.97). The closed tall footprint with a
tolerance of max(1 voxel, k × range) refuses them: k = 0.05 → 45 % at stops and 56 % on the walk, at a cost of 3.7 / 2.7 %
of the balls and blocks by walls and 0 % of the open-floor things; k = 0.08 → 52 / 66 % at 8 / 6 %. Built:
`CloudMap.line_tol_k` (0 = off) at half the ToF's zone spacing (0.049, from the sensor's geometry), `line_close_k` 0.098, a
cluster's `on_line` share; unit test `AFragmentOnAClosedWallLineIsNotAThingButACubeBeforeItIs` (at 1.5 m the tolerance is 1.8
voxels: the line catches a two-voxel smear from ~1.65 m out). Config `a1v2_t2_line`, sweep `log/ten/s3`:

| | base | + the line |
|---|---|---|
| boring · interesting | 30.5 % · 41.1 % | **23.9 % · 48.1 %** (12 of 18 seeds less boring; paired t −1.7) |
| worst run's boring share | 66 % | 41 % |
| seed 1 (the preset's) boring · interesting | 66 % · 21 % | **21 % · 60 %** |
| stare · seek→structure · pinned, s a run | 24 · 69 · 48 | 11 · 44 · 42 |
| arrivals at a thing · structure · nothing | 221 · 71 · 44 | **274 · 38** · 46 |
| skills touching a thing · walls / min | 78 / 234 · 17.4 | 99 / 259 · 15.8 |
| falls a run · down s a run | 2.2 · 5.8 | 2.9 · 9.5 |
| first minute: block moved · first skill touching | 10/18 · 7/16 | 8/18 · 5/16 |

**Verdict: `WORKING` (a signal at n = 18), the cost three quarters of a fall a run**; preset for the eye.

**S3, the outcome loop learns that structure does not answer.** The kind vocabulary does not separate fragments: on the base,
the attended wall and furniture fragments fall into the same four kind nodes as the balls and blocks on every seed — so a
peck at a fragment would teach "this kind does not move" and silence the blocks. Built (all off by default; unit tests
`StructureThatNeverAnswersLosesItsPullAndLendsItToItsContext`, `AThingThatMovesKeepsItsPull`, `ASightedThingsNeedIsTheOutcomesPull`):
`CloudMap.context_topic` (the attended thing's [on_line, near_tall], near_tall = exp(−d / (0.098 × range))), a context EPM
(3 nodes) over it, `SkillOutcomeLoop.context_topic` (the table keyed kind × context), `answer_m` 0.08 (an answer is a
displacement above two voxels; each cell counts its answers), `pull_topic` (the expected answer at the attended thing's cell:
1 while uncertain, else (answers + 1) / (n + 2); an uncertain cell borrows its context's pooled share once the context holds
min_samples × intents outcomes), and `BearingSeekLoop.pull_topic` (a sighted thing's need is the pull, not 1). Config
`a1v2_t3_habit`, sweep `log/ten/s4`: boring 30.7 % (base 30.5), arrivals at structure 77 (71) — **`NULL`, starved**: a run
asks ~15 times and sees ~10 answers, the cells known per run are 0–4 of twelve, and a context reaches its pooled bar of six
answers late if at all. Not a verdict on the idea: the question it asks ("does anything standing here move?") does not depend
on the intent, so the pooled bar should be min_samples, not min_samples × intents (S3b).

**The impeded look (host `--look-up-when-impeded 3 -0.3 1.5 0.6`; the operator: "the robot should learn to look up to see if
an obstacle that is impeding its movement is a wall, so it should be backed away from").** Measured first, on the line arm:
58 s a run of dwelling at structure (episodes ≥ 5 s, under 10 cm net in 3 s, median 8 s), seek holding the reference on 68 %
of those ticks, 4 of 112 episodes caught by the stall detector — whose stall needs a forward command above 75 % of range,
which the dynamic-range walk's slowed approach never gives — and the progress forget (half a metre walked) never firing on
a body that walks nowhere. Every stop's sweep looks DOWN (head pitch 0.29–0.51, positive = down): the tops are never in
view. Built: IMPEDED = the seek loop's own prediction failing (it steers and its range has not closed 5 cm in 3 s); the duck
backs off 0.6 s (the ToF's near field is too-close inside 10 cm), stops with the head held up (−0.3; it reaches about level
in 1.5 s under the override's slew), and at the stop's end reads the cloud: tall voxels around the target and NO small thing
within 12 cm of it (`CloudMap::target_small`, new) = a WALL → the target is forgotten and the escape turns the walk to the
freest sector; else a THING → kept. Guard byte-identical (off). Sweep `log/ten/s5` against the base:

| | base | + the impeded look |
|---|---|---|
| boring · interesting | 30.5 % · 41.1 % | 28.3 % · 44.3 % |
| arrivals at a thing · structure · nothing | 221 · 71 · 44 | 215 · **31** · 42 |
| seek→structure · pinned · stare, s a run | 69 · 48 · 24 | 54 · 43 · 28 |
| walls / min · falls a run | 17.4 · 2.2 | 19.1 · 3.2 (11 of 58 falls within 5 s of a look) |
| looks a run · judged wall · thing | — | 6.4 · 2.1 · 3.7 |

Verdicts against truth (the target's true label at the look): structure → wall 28, thing 18; a real thing → thing 38, wall 8;
nothing → wall 2, thing 10. **Verdict: `PARTIAL`** — arrivals at structure more than halved and the operator's sequence
visible (impeded, back off, look up, turn away), the boring share a small move, a fall a run its cost (the back-off and the
stand with the head moving, O36's old risk). The wrong "thing" verdicts at structure are mostly a fragment still reading small
near the target — what the line removes: the two belong together. Preset for the eye.

**S3b, the context pooled at two answers (`context_pool_min 2`, config `a1v2_t3b_pool`, sweep `log/ten/s6`; unit test
`ALowerPoolBarLendsAfterTwoAnswers`):** a context lends its answered share once it holds two outcomes, whatever the intent.
Against the base: arrivals at structure 71 → **40** (things 221 → 231), boring 30.5 → **27.2 %** (worst run 66 → 49 %),
stares 24 → 16 s, seek→structure 69 → 65 s, walls 17.4 → 19.7 (a tie), observed answers 6–15 a run. **`PARTIAL`, the learned
form works** — fewer walks to walls because the duck has learned that things standing there do not move. **Falls:** every arm
since the base reads 2.9–3.2 a run (★ TURN 3.2, T1 2.1, the base 2.2); the base looks like the low draw, not the levers' cost.
The three levers attack one failure from three sides — the sensor (the line), learning (S3b), an epistemic action (the impeded
look) — and are stacked next (T4).

**T4, the stack (config `a1v2_t4_stack` = T1 + the line + S3b, host `--stop-is-still --look-up-when-impeded 3 -0.3 1.5 0.6`;
sweep `log/ten/s7`):**

| n = 18 × 600 s | ★ TURN | base | the line | **T4 stack** |
|---|---|---|---|---|
| boring · interesting | 43 % · 35 % | 30.5 · 41.1 | 23.9 · 48.1 | **22.3 · 48.4** |
| worst three runs' boring share | 61–72 % | 48 · 57 · 66 | 37 · 38 · 41 | **30 · 31 · 41** |
| arrivals at a thing · structure · nothing | 136 · 94 · 18 | 221 · 71 · 44 | 274 · 38 · 46 | 222 · 34 · 30 |
| walls / min | 32.7 | 17.4 | 15.8 | **14.5 ± 10.7** |
| falls a run | 3.2 | 2.2 | 2.9 | 2.7 |
| seed 1 boring · interesting | 36 · 30 | 66 · 21 | 21 · 60 | 29 · 46 |

Paired against the base: boring −8.2 points (t −2.5, 13 of 18 seeds), interesting +7.4; against the line alone a tie (−1.6,
t −0.5). The impeded looks fire 5.8 times a run and judge 1.1 walls, 4.2 things (the line has removed most fragments before
the look). **Verdict: `WORKING` against the base; the line carries most of it, S3b and the look tighten the tail and the walls
(ties at this power).** Fewer skills than the line alone (191 against 241 by the instrument's count) — the learned pull is
also quieter on things it has tried. Preset "T4 · the STACK" for the eye; the line alone stands beside it.

**★ T4 promoted (2026-10-02, the operator's eye):** "I agree. The stack is promoted." ★ T4 = ★ TURN + host `--tof-free-rays
--stop-is-still --look-up-when-impeded 3 -0.3 1.5 0.6` + config `a1v2_t4_stack` (CloudMap `free_rays`, `small_needs_top`,
`line_tol_k 0.049`, `context_topic`; the context EPM; SkillOutcomeLoop `context_topic`, `context_n 3`, `pull_topic`,
`context_pool_min 2`; BearingSeekLoop `pull_topic`). The base for the phantom (phase 2).

### 17.100 The chase push: the pursuit replaces the look, and the follow rests on the prediction (2026-10-02)

**The operator, on ★ T4:** "the chase loop, once arbitrated to enabled, needs to keep the robot moving. I am seeing the robot
stop and look after doing a short chase, which breaks off the chasing behaviour." The tunnel can wait: the duck loses sight of
the train often enough without one. **Instrument:** `mj_host/tools/chase_readout.py` (chases and their length, at the train,
seconds on the moving train and the share closing on it, following, contacts, lost→stop, standing after a chase,
re-acquisitions, pursuit seconds).

**★ T4's chase (n = 18):** 4.5 chases a run, median 1.0 s, p90 1.8 s; 3.1 s a run on the moving train; at the end of a chase
the train was still moving in 55 of 81 and in view in 34; 1.6 stops a run started by the loss (`--stop-on-lost`, R94) — the
break-off the operator sees.

**Lever 1 — the pursuit replaces the look** (`chase_permanence_ticks` 250 / 500, host without `--stop-on-lost`; configs
`a1v2_t5_pursuit5` / `_pursuit10`; sweep `log/ten/c1`). No code: coasting (R95) already walks to the lost mover's predicted
position with its need decaying linearly — the interest spends itself — and was a `REGRESSION` in R95 because the prediction
led into walls; ★ T4's line, tall-structure yield and impeded look are its re-use context.

| | ★ T4 | pursuit 5 s | pursuit 10 s |
|---|---|---|---|
| lost → stop a run · standing within 4 s after a chase | 1.6 · 4.6 s | 0 · 0.9 s | **0 · 1.2 s** |
| pursuit s · re-acquired a run | — · 0.4 | 7.2 · 0.3 | 12.2 · **0.9** |
| on the moving train s · closing share · contacts a run | 3.1 · 52 % · 1.7 | 3.0 · 53 % · 2.2 | 3.3 · 59 % · 2.2 |
| boring · walls / min · falls | 22.3 % · 14.5 · 2.7 | 22.4 · 18.6 · 2.8 | 22.5 · 15.6 · 2.4 |

`WORKING` on the operator's ask (no break-off, re-acquisitions doubled at 10 s, nothing paid). The chase itself still lasts
a second.

**Why a confirmed chase lasts a second:** in the last second before a chase drops into the pursuit, the cloud publishes no
mover on 98 % of ticks (the tracker's 26 sightings all missed its gate). The train's own cluster in those seconds (52 drops):
no cluster in the recent window on 25 (out of the sensor's view), **not young against the cloud's oldest on 21** — the age
gate that starts a chase is also the only thing that continues one, and in a walking cloud re-filed every metre the oldest
cluster is seconds old, so 6 % of it falls below the train's own voxel age (~0.26 s). Cloud re-anchors coincide with 22 % of
drops.

**Lever 2 — the follow rests on the prediction** (`CloudMap.mover_hold_any_age`, `mover_hold_ticks`, with the existing
`mover_range_hold`; config `a1v2_t6_follow` = pursuit 10 s + range hold 2.5 m, any age, 50-tick hold; the hold point carried
across the walking cloud's re-anchor and accepted during a new cloud's vouching window; unit test
`AFollowedMoverStaysTheMoverWhenItsVoxelsAgeWithTheHold`; guard byte-identical; sweep `log/ten/c2`):

| | pursuit 10 s | **+ the follow on the prediction** |
|---|---|---|
| chases a run (at the train) · median · p90 length | 4.9 (43/89) · 1.0 · 2.1 s | 11.2 (90/201) · 1.3 · **7.7 s** |
| on the moving train s a run · closing share | 3.3 · 59 % | **15.2** · 48 % |
| contacts with the train a run · pursuit s | 2.2 · 12.2 | **3.8** · 22.1 |
| boring · interesting · walls / min | 22.5 % · 46 % · 15.6 | **18.2 % · 51 % · 13.3** |
| arrivals at a thing · structure · nothing | 199 · 35 · 30 | 196 · 16 · 45 |
| falls a run (within 3 s of a chase, all runs) | 2.4 (6) | 3.6 (15) |
| first minute: block touched · moved · first skill touching | 16/18 · 8–10/18 · 5–7/16 | **11/18 · 5/18 · 3/13** |

**Verdict: `WORKING` on the chase (the moving train chased 4.6× longer, contacts +70 %, the lowest boring share and walls of
the phase), `REGRESSION` on the first minute and a fall a run.** The first minute's cost: 3.7 s of chasing in it (0.4 s
before), 21 chases of which 7 at the train, 6 at structure, 6 at things — part wanted (the train passes early on many seeds),
part the hold accepting a static cluster the prediction sweeps over. Next: a held cluster must still be moving (its own
displacement), and the out-of-view drops (the gaze on the walk).

### 17.101 The always-on motion loop, step 1: a motion sensor with its own memory (MotionField, passive; 2026-10-02)

**The operator, watching T6 on the chase seed:** "at 150 seconds the robot loses track of where the train is … we need some
type of motion-sensitive peripheral vision that is always active, as an ongoing loop separate from the others, looking for
voxels moving relative to the world frame at all times; moving voxels should always capture the robot's attention."
**At 150 s:** the train 0.6 m away, 11° off the head's axis, moving — its cluster in the walking cloud 7 cm from the truth,
and rejected: the cloud had re-opened 1.7 s earlier, its oldest cluster was 1.04 s old, 6 % of that is 0.06 s, the train's
voxels 0.9 s old. The cloud's notion of "new" is only as long as the cloud is old.

**Built: `MotionField`** (cpp_core, off unless in the graph; passive without `output_topic`; host `--log-motion` logs evidence,
tracks, the published track and each evidence point's vouching ray; instrument `mj_host/tools/motion_readout.py`; unit tests
`test_motion_field` 3/3). Its own memory of every ray of the last 1.5 s in the odometry frame, never filed. A new off-floor
return is evidence when a remembered ray passed within 2.5 cm of it and went on `beyond_m` past it; blobs of evidence within
12 cm, tracked cast to cast, published after two casts. A cast is processed when its points' x and y change (the host's z
carries the trunk's height of every tick — the first build processed 29 685 "casts" in 600 s instead of 7 435). Guards: the
graph with MotionField passive equals T6 tick for tick (seeds 1, 5, 7, 13).

**What it calls motion, and the layers that removed it (seed 5, 600 s):** the free-ray test alone — 244 false tracks a
minute, the vouching rays showing why: SILHOUETTE EDGES (a table's or seat's top edge at 0.37 m, a wall's top, a ball's
crown) where earlier rays skimmed within 2.5 cm and hit the wall a metre behind. A BACKGROUND (`bg_m` 0.04: a return with a
remembered return within 4 cm, 1–10 s old, is the static world) → 54 a minute; a track must TRAVEL 10 cm (`min_travel_m`) →
22; the vouching ray must go on 30 cm → 4.7. Standing or walking, head still or turning: the same false rate (not registration).

**n = 18 (sweep `log/ten/m2`), the moving train in the ToF's cone within 2 m (5 % of casts):**

| | background only (`bg_m` 0.04) | strict (+ travel 10 cm, beyond 30 cm) | the cloud's detector* |
|---|---|---|---|
| a track on the train, share of in-view casts | 35 % | 22 % | 47 % |
| entries into view caught · median latency | 126/456 · 0.32 s | 68/456 · 1.12 s | 194/456 · 0.00 s |
| false tracks a minute | 57 (structure 47, things 11) | 7.7 | — |

*any mover the cloud published while the train was in view (an upper bound). Complementarity (background-only): both 27 %
of in-view casts, the motion sensor only 6 %, the cloud only 19 %, **neither 45 %** (235 of 456 entries: brief glimpses at
the cone's edge, too short for a two-cast confirmation). On the chase seed at 150 s it catches the train 0.4 s after it
starts moving where the cloud sees nothing; from 151 s the train passes beside the duck at half a metre, outside the 45°
cone. **Verdict: `PARTIAL` as a sensor — complementary to the cloud's detector (+27 entries, +6 % of in-view casts), not a
replacement at this precision; the larger loss is COVERAGE (the train in view briefly, or not at all while it passes close),
which is the gaze's business (step 2).**

### 17.102 The motion loop, step 2: motion captures the gaze — `NULL` on the chase, `REGRESSION` on the walk (2026-10-02)

**Built:** host `--gaze-to-motion` — the head's attention target is a mover whenever there is one (the chase's target, else
MotionField's published track, else the cloud's unconfirmed candidate), otherwise where the walk is going; the learned gaze
turns the head to it on the walk (the gaze-error sense), a stop gives it the yaw override; the record's `gm`. `--gaze-no-candidate`
drops the cloud's unconfirmed candidate. Guards byte-identical. Sweeps `log/ten/g1`, `log/ten/g2`, n = 18, against T6:

| | T6 | gaze: chase + candidate | + the motion sensor | gaze: chase + motion only |
|---|---|---|---|---|
| on the moving train s a run · contacts | 15.2 · 3.8 | 15.1 · 2.2 | 14.1 · 2.3 | 15.4 · 2.9 |
| boring · walls / min · falls | 18.2 % · 13.3 · 3.6 | 24.7 · 19.9 · 2.4 | 23.6 · 16.5 · 2.5 | 23.0 · 22.2 · 3.5 |
| first minute: the block touched | 11/18 | 12/18 | 13/18 | 12/18 |

**The head does reach its target** (|want − head yaw| after a second of held attention: median 0.06–0.08 rad walking), but the
attention target was mostly the cloud's unconfirmed candidate (15 % of ticks, mostly young static clusters), and the moving
train within 1.5 m stayed in the 45° cone 16 → 18 % of the time. Without the candidate (chase + motion tracks only) the chase
ties and the walk loses its forward view: walls 13 → 22 a minute. **Verdict: `NULL` on the chase, `REGRESSION` on the walk —
not kept.** Re-use: a gaze that leaves the walk's direction needs the walk to keep its own forward view (a body-frame memory of
what lies ahead long enough to cover the glance), or a glance only while standing.

### 17.103 The motion loop, step 3: motion feeds the chase; and why T6's first minute is worse (2026-10-02)

**T6's first-minute cost, traced** (the green block touched 11/18 against the pursuit's 16/18): in the seeds that lost it, a
chase starts at ~10–12 s on a THING — the green block itself, young in the walking cloud as it comes into view — and the
follow's hold keeps it at the chase's raised speed: the arrival comes with falls and skills at wild bearings (s15, s18, s5, s9);
on s4 and s10 the early chase is the train (wanted). **T6b, the hold keeps only what moves** (`CloudMap.mover_hold_min_v` 0.05:
the any-age exemption lapses when the followed mover's step stays under 5 cm/s for half a second; unit test
`AFollowedMoverThatStopsLosesTheAnyAgeHold`; config `a1v2_t6b_follow_moving`, sweep `log/ten/c3`): the moving train 15.2 →
11.1 s a run, contacts 3.8 → 2.8, walls 13 → 17, falls 3.6 → 2.9, the first minute unchanged (11/18) — `REGRESSION` on the
chase, not the first minute's cause (the false START is). Not kept.

**Step 3 — MotionField feeds the chase** (`cloud_mover_topic`: with no track of its own MotionField passes the cloud's sighting
through, so the seek loop's `mover_topic` gets the union; unit test `WithoutItsOwnTrackTheCloudsMoverPassesThrough`). **Trap
found:** the scheduler runs modules in the config's order — MotionField appended after the seek loop gave a token stamped a tick
late, which the seek loop ignores: zero chases. MotionField must sit between CloudMap and the seek loop.

| n = 18 | T6 | M3: the union (strict sensor + the cloud) | M4: the strict sensor only |
|---|---|---|---|
| chases a run · on the moving train s · contacts | 11.2 · 15.2 · 3.8 | 13.6 · **16.5** · 2.4 | 1.3 · 1.3 · 2.2 |
| re-acquired · pursuit s | 0.4 · 22.1 | **0.8** · 21.9 | 0.1 · 1.8 |
| boring · walls / min · falls | **18.2 %** · 13.3 · 3.6 | 21.7 · **11.7** · 3.6 | 21.2 · 14.8 · 2.8 |
| first minute: the block touched | 11/18 | 12/18 | **15/18** |

M3 `NULL` (the chase a little longer, the walk a little better, contacts and boring worse). M4 confirms the first minute's
false starts come from the cloud's detector (a static block newly in view reads young; the motion sensor needs free space where
a ray just passed and does not fire on it) — and that the strict sensor is too sparse to chase from (`REGRESSION` on the chase).
Next: the background-only sensor (recall 35 %) as the chase's source, alone (M5) and in union with the cloud (M6), the chase's
own prediction and speed tests as the second filter.

**M5 / M6 — the background-only sensor (`bg_m` 0.04, no travel, beyond 8 cm; recall 35 %) as the chase's source** (configs
`a1v2_t9b_motion_bg_only`, `a1v2_t8b_motion_bg_union`; sweep `log/ten/m5`): alone, the first minute is kept (15/18) but its
chases are mostly false and drop at once (195 chases, 47 at the train, median 0.0 s; the moving train 5.2 s a run); in union
with the cloud, everything but contacts is worse than T6 (boring 23.2 %, walls 19.8). Both `REGRESSION`.

**M7 — the cloud follows, the motion sensor VOUCHES** (`MotionField.vouch_m` 0.3, `vouch_s` 0.6, `own_tracks` false: a cloud
sighting passes to the chase only with motion evidence within 0.3 m in the last 0.6 s, or continuing a vouched mover; unit test
`TheCloudsSightingPassesOnlyWhereMotionWasSeen`; config `a1v2_t10_vouch`, sweep `log/ten/m7`): chases 6.8 a run, half at the
train (T6 45 %), the moving train 10.7 s a run (T6 15.2), contacts 2.8, boring 21.4 %, walls 15.0, falls 3.3, **the first minute
14/18** (T6 11). `PARTIAL`: the precision and the first minute the division of labour was built for, at a third of T6's
chasing; the continuation may starve (a vouched mover must re-appear within 0.25 m in 0.5 s). M7b loosens it (0.4 m, 1 s).

**M7b, the vouch loosened** (0.4 m, 1 s of evidence, 1 s of continuation; `a1v2_t10b_vouch_loose`, sweep `log/ten/m8`): the moving
train 9.4 s a run, falls 4.4 — no better; the vouch suppresses chase STARTS (the evidence lands on the train on half its
casts), the continuation was not the limit. `NULL`.

**T11 — the thing walked to is not a mover** (`CloudMap.mover_not_target_m` 0.25: a NEW mover candidate within 25 cm of the
seek loop's held STATIC target is refused, the followed mover exempt; `BearingSeekLoop.publish_chase_flag` gives the seek
token a 4th value, 1 while chasing or coasting, so a pursuit's predicted position is not gated; unit test
`ANewMoverAtTheHeldStaticTargetIsRefused`; config `a1v2_t11_not_target`, sweep `log/ten/c4`):

| n = 18 | T6 | T11 |
|---|---|---|
| the moving train s a run (sd) · contacts | 15.2 (11.8) · 3.8 | 11.1 (7.8) · 2.6 |
| boring · walls / min · falls | 18.2 % · 13.3 · 3.6 | 21.4 % · 19.0 · **2.3** |
| first minute: the block touched · the first skill touching | 11/18 · 3/13 | **15/18** · 3/15 |

Paired on the moving train −4.2 s (t −1.7, a signal; the per-seed sd is 8–12 s, so every follow arm — T6, M3, M7, T11 — lands in
10–17 s against the pursuit's 3.3). **`PARTIAL`, the balanced arm:** the first minute recovered and the fewest falls of the
chasing arms, the chase at three times the pursuit's, boring and walls the cost. Preset for the eye beside T6.

**T12 — T11 + the union with the strict motion sensor** (`a1v2_t12_gate_union`, sweep `log/ten/c5`): the moving train 10.0 s a
run, boring 23.4 %, falls 3.6, the first minute 15/18 — `NULL` against T11. **On the chase seed 2060249272** (one run each): T6
chases the moving train 7.1 s with no contact, T11 14.1 s with one.

**Where the motion loop stands (2026-10-02):** the always-on sensor exists (MotionField, its own memory of rays, background and
travel tests) and its best use found is as a precision instrument — it showed the first minute's false chases come from the
cloud's detector, and its vouch recovers the first minute — but at 22–35 % recall on the 8 × 8, 45° sensor it cannot drive the
chase by itself, and pointing the head at motion costs the walk its forward view. Every follow arm (T6, M3, M7, T11, T12) lands
at 10–17 s a run on the moving train (the per-seed sd 8–12 s) against the pursuit's 3.3 and ★ T4's 3.1; the residual limit is
COVERAGE: the moving train within 1.5 m is inside the cone 16–18 % of the time. The arms for the eye: **T6** (the most chasing,
the lowest boring share and walls; the first minute and a fall a run its cost) and **T11** (the first minute recovered, the
fewest falls, a third less chasing). Next leads, in order: a walk that keeps its forward view while the head glances (a
body-frame memory long enough to cover a glance — the re-use context of the gaze's regression), then the gaze to motion
retried; the head's free time at stops spent sweeping wider for motion.

### 17.104 The train room: practice with something that moves (2026-10-03)

**The operator:** "we need to keep iterating; perhaps a smaller room with only the train in it that runs continuously would
help. You need to give the robot a lot of contact with moving objects in order for it to learn how to chase them."
**Built:** `playroom_gen.py --train-room` (walls and the train on its oval, nothing else; half 1.25 m, the oval 0.875 × 0.8 m to
0.35 m of the walls, the duck starting inside it; `scene_train_room.xml`; the plain and train playrooms regenerate
byte-identical); the host's `--train 0.2 60 0` runs the train without stops.

**The session (T11, learning on as on every walk, 3 seeds × 1200 s, brains saved; `log/ten/tr1`):** no learning curve —
chase seconds per 200 s window 2–21 with no trend on any seed; the train inside the sensor's cone only 20–48 s of every 200
even circling the duck; ~5 contacts per window. **What learns from the exposure:** the cloud's detector, MotionField and the
chase are fixed rules; the walker's MotorEPMv2 is the one learner that senses the target (bearing, range). **Transfer** (the
saved brain loaded into the train playroom, T11, n = 18):

| | T11 (★ GAZE's walker) | 1200 s train-room walker | 300 s train-room walker |
|---|---|---|---|
| chases at the train · on the moving train s · closing | 34 % · 11.1 · 39 % | **51 %** · 12.0 · 39 % | **51 %** · 10.0 · **50 %** |
| contacts a run | 2.6 | 3.3 | 3.3 |
| first minute: the block touched | 15/18 | **0/18** | 10/18 |
| arrivals at things · skills touching | 198 · 73/215 | 136 · 30/129 | 196 · 67/197 |
| boring · walls / min · falls | 21.4 % · 19.0 · 2.3 | 25.3 % · 23.5 · 1.8 | 23.6 % · 16.0 · 2.7 |

**Verdict: `PARTIAL` on the chase's precision and contacts, `REGRESSION` on the approach to still things** — the contact regime's
lesson again (§17.85: a regime the brain never sees the others in overwrites them; 1200 s lost the walk there too): a walker
identified only on a target that moves forgets how to arrive at one that does not, and the dose only scales the cost. The
walker has nothing chase-specific to identify — it senses where the target is, not how it moves. Next: the mover sense (the
chased mover's velocity in the walker's sense, `--intent-mover-sense`), so practice has a chase-specific thing to learn.

**The mover sense** (host `--intent-mover-sense`: two walker sense slots after the range, the chased mover's velocity in the body
frame / 0.6 m/s, zero when nothing is chased; `twist_bridge.load_slots` 20; ★ GAZE's walker grown 26 → 29 at 10; config
`a1v2_t13_mover_sense`; guard byte-identical): from ★ GAZE's walker, its slots identified online (sweep `log/ten/ms1`) — chases at
the train 49 %, the moving train 12.2 s, contacts 2.8, the first minute 14/18, boring 22.9 %, walls 16, falls 3.3: a tie with T11
(`NULL`). Practised 600 s in the train room first (`duck_trainroom_mover_s1`, `log/ten/ms2`): the moving train 9.7 s, closing 35 %,
**the first minute 1/18** — the train-only regime's loss again (`REGRESSION`). The mover sense did not make train-only practice
safe: the walker still learns a world without still targets. A MIXED practice (1200 s in the train playroom with the train
running continuously, `duck_mixedpractice_mover_s1`) shows no learning curve inside the session either (chase per 200 s: 12,
11, 4, 9, 0, 10 s); its transfer in `log/ten/ms3`.

### 17.105 The learned chase: where to aim, learned at the intent level (2026-10-03)

**The operator:** "the body model is not at the layer where chasing would occur. Chasing is a planning loop and exists at the
intent level. Strategic." — and "build it with the walker frozen during practice." **Built:** `BearingSeekLoop` THE LEARNED LEAD
— `lead_options` (aim points 0 / 0.5 / 1 / 2 s ahead along the mover's velocity); the chase's SITUATION published on
`situation_out_topic` ([range / 2.5, |bearing| / π, the bearing's drift outward, the range rate], in [0, 1]) for an EPM
(`chase_situation_epm`, ≤ 6 nodes) whose winner keys the table; every `lead_eval_ticks` (1 s) of chasing, the closing speed of the
range to the mover's estimated position is recorded for (situation, option), then the least-tried option (under
`lead_min_samples` 2) or the best mean is chosen; the table snapshots and restores, `restore_lead_only` keeping a practice's held
target out of the next room. Host `--freeze-walker` (the walker's MotorEPM learning off for the run; nothing re-enables it) and
the record's `cl` (option, situation). Unit tests `TheChaseLearnsWhereToAimFromItsOwnOutcomes`,
`ARestoredLeadTableAimsWithItsBestOptionAndBringsNoTarget` (26/26); guard byte-identical. Config `a1v2_t14_learned_lead`.

**Practice (the train room, walker frozen, 3 seeds × 1200 s, `log/ten/ll1`):** the chase runs 1–12 s per 300 s window even with
the train circling the duck; the tables hold **11–21 outcomes** over up to six situations and four options, and disagree across
seeds (situation 0's best: 1 s ahead on s1 and s3, 2 s or none on s2). **The learner works and starves:** the experience the
operator asked for is gated by the same thing the chase is — the duck sees the train in its 45° cone 10–17 % of the time and
confirms a chase on a fraction of that. The playroom arm learning online (no practice) in `log/ten/ll2`.

**The learned lead online** (T14 in the train playroom from ★ GAZE's walker, no practice, n = 18, `log/ten/ll2`): chases at the
train 42 %, on the moving train 8.1 s (closing 49 %), the first minute 16/18, boring **19.3 %**, falls 2.6. The chase seconds
fall (8.1 vs T11's 11.1) and the boring share falls (19.3 vs 21.4): the table spends its first outcomes on options that lose the
mover. Within noise on both; `NULL` until the table has experience to act on, which the coverage below gates.

### 17.106 Coverage: can the duck keep the train in view? (2026-10-03)

**The operator** chose to fix coverage before feeding the learned chase. The measure: the share of ticks the moving train is
within 1.5 m and inside the ToF's 45° cone (body yaw + head yaw). Two arms on T11, n = 18, the train playroom:

| | T11 | C1: the walker's ToF memory 2 s (`--tof-body 2.0`) | C2: C1 + MotionField + the gaze to motion |
|---|---|---|---|
| the moving train near and in the cone | 15 % | 14 % | 14 % |
| walking with the head ahead (\|head yaw\| < 0.4) | 67 % | 61 % | 62 % |
| on the moving train s · closing · contacts | 11.1 · 39 % · 2.6 | 10.5 · 41 % · 3.0 | 8.6 · 41 % · 2.0 |
| first minute · boring · falls | 15/18 · 21.4 % · 2.3 | 12/18 · 23.0 % · **3.7** | 12/18 · 23.3 % · **4.1** |

**Verdict: `NULL` on coverage, `REGRESSION` on falls (both arms).** The diagnosis is structural, not a tuning miss: every
mechanism tried for the chase (the cloud's mover, MotionField, the gaze to motion, the memory that lets the head look away)
works on what is ALREADY in the 45° cone. Turning toward motion needs the motion seen first, so none of them can bring into
view a train that is outside it — the share stays at the geometry's ~15 %. Rewrite rule step 2: the signal the error needs is
not in any observation the duck has. **What would carry it:** a wide field, low resolution, motion-only sense — which is what
peripheral vision is — i.e. the head camera (playroom plan C1, not yet rendered in the host; frame differencing over a wide
lens); or an active one, a gaze that visits where the map is oldest (the cone made wide by time, at the forward view's cost,
which the body-frame memory now protects). Re-use context for C1/C2: revisit once a wide sensor exists to point the gaze.

### 17.107 Looking around while walking (2026-10-03)

**The operator:** "let's try the feature of looking around while walking (it will make the robot more interesting to watch);
we will then report the limitations and outline our case for camera access" (the camera buffer is not exposed in Pollen's
API). **Built:** host `--look-around MAX_YAW STALE_S` — a TARGET for the head brain's learned gaze error, not a sweep: 24
sectors of 15° in the odometry frame, each aged since it was last inside the 45° cone; on the walk, with no chase, no seek and
no mover holding the gaze, the target is the stalest sector within MAX_YAW of the body's axis if older than STALE_S, else
straight ahead. A sector seen is fresh, so the look moves on of itself. Logged as `la` [on, bearing]. Guard byte-identical.
On T11, n = 18, the train playroom:

| | T11 | LA1 (0.9 rad, 2 s) | LA2 (1.2 rad, 1 s) |
|---|---|---|---|
| look target active (share of walking) · \|head yaw\| median walking | — · 0.21 | 14 % · 0.33 | 25 % · 0.41 |
| the moving train near and in the cone | 15.3 % | 14.2 % | 14.9 % |
| on the moving train s · closing · contacts | 11.1 · 39 % · 2.6 | 11.0 · 44 % · 2.4 | 10.9 · 42 % · 2.5 |
| first minute · boring · walls/min · falls | 15/18 · 21.4 % · 19.0 · 2.3 | 13/18 · 22.8 % · 15.1 · 2.4 | 13/18 · 21.7 % · 14.6 · **5.9** |

**Verdict: LA1 ties T11 on every number (`NULL` on coverage, no cost) with a visibly livelier head — the candidate for the
operator's eye, since the claim is "more interesting to watch"; LA2 `REGRESSION` (falls 2.5×).** Why coverage cannot move: a
head that looks around changes WHICH 45° the duck sees, not HOW MUCH; at any moment a train at an unpredictable bearing is in
view with the same ~15 % probability. Only a wider field changes that.

**The field of view the case for the camera rests on** — the same T11 logs re-scored as if the sensor's cone were wider
(the moving train within 1.5 m, ~107 s a run): 45° 15 % · 60° 19 % · 90° 27 % · **120° 36 %** · 160° 47 %. A 120° camera
used only as a motion detector would put the train in view 2.4× as often as the ToF does, before any gaze helps.
The case for camera access, in Pollen's terms: outreach plan §9.

**★ LA1 PROMOTED on the operator's eye (2026-10-03):** "the robot actually looks quite interesting when it's looking around at
areas that it hasn't seen before; it's novelty to the behaviour, and it is definitely moving in a direction that we want." The
stack is now ★ LA1 = T11 (config `a1v2_t11_not_target`) + host `--look-around 0.9 2.0` (launcher preset, second row).

**Correction, the same day (checked against Pollen's tree at `ded2f7c`):** the duck's camera is an IMX219 at ~62° horizontal,
not a 120° lens, and `media.frame` (one raw frame per call on `mediad`'s socket) already exists. Re-scored over ALL the train's
moving time (not only near it), the ToF has it 8 % (45°, under 2 m) and a 62° camera at any range 19 % (T11, n = 18; LA1 7 vs
16 %): still 2.4×, from range more than width. The ask is reshaped to their own "features, not pixels" direction: outreach plan §9.

### 17.108 Defects found while documenting ★ LA1 (2026-10-04, open)

A file:line fact sheet of the three ★ LA1 brains (for the companion document to PR-2) turned up these; each fix is a lever
(guarded, n = 18, the eye), none is applied yet.
- **The escapes are mirrored (confirmed).** `CloudMap::view()` numbers sector 0 on the RIGHT (az + = left, `CloudMap.cpp:1188-1205`);
  `main.cpp:2474` and `:2488` map sector k to `-64 + (k + 0.5)·16` deg and call it "+ = right", so `set_ref_hold` heads for the
  mirror image of the freest sector — the stuck escape and the impeded look's wall escape (★ T4) alike.
- **The stop sweep's centre is mirrored (confirmed).** `sweep_yc` takes `seek_ego()`, `lost_ego` and `thing_ego()` (all + = right)
  as a head-yaw offset (+ = left) at `main.cpp:2228`, `:2232`, `:2236`; the gaze code elsewhere negates (`want = -seek_ego`).
- **`--map-on-stop` bakes on walks.** The walk freeze sets `min_insertion_error` 1e9, and the bake gate compares against the same
  value (`gng.cpp:291`), so any place node reaching 20 visits on a walk bakes without the consistency check (probe: 4 of 7).
- **The place map's pose clamps at ±1.2 m** (`dim_min/max` ±0.6 on x/2, y/2) in a 4 m room: 61 % of ticks at the clamp in a 120 s probe.
- **Suspected, unconfirmed:** each ToF cast re-added to the cloud ~4× (republished every tick between casts); `comp_seek` counting
  stop-time windows as failures; `regime_epm`'s restore resetting `max_nodes` 2000 / `health_death_spares_baked` false.
- **Inert:** `object_epm` and `thing_epm` outputs unread by any loop; PlayLoop's value not in the precision score.
Fact sheet (2,980 lines, every claim anchored): [`microduck_la1_fact_sheet.md`](microduck_la1_fact_sheet.md); companion document
"Inside the MicroDuck Brain" (Claude Docs).

### 17.109 The §17.108 fixes, measured (2026-10-04)

**The operator:** "run the powered studies on the fixes you proposed; let's find out if they have meaningful impact on
behaviour." Four levers on ★ LA1, each alone, guard byte-identical (all off = ★ LA1 seed 5): **F1** `--fix-escape-sign`;
**F2** `--fix-sweep-sign`; **F3** `--map-bake-honest` (new GNG/EPM `bake_gate`; unit test
`BakeGateKeepsTheConsistencyCheckWhenInsertionIsFrozen`); **F4** config `a1v2_la1_f4_map_wide` (the map's pose slots ±2.2 m).
n = 24 seeds (1–24) per arm, 600 s, the train playroom, a fresh ★ LA1 base (`log/ten/fx0`–`fx4`). Instrument:
`mj_host/tools/fx_study.py` (paired by seed; 95 % CI; the detectable difference at 80 % power). Seed pairing barely cuts the
variance (runs diverge), so the detectable differences are wide: boring ±7–9 points, walls ±9–14 /min.

| | ★ LA1 | F1 escape | F2 sweep | F3 bake | F4 map wide |
|---|---|---|---|---|---|
| boring · interesting % | 22.1 · 46.9 | 22.2 · 45.6 | 21.5 · 48.0 | 23.1 · 45.5 | 20.9 · 50.0 |
| walls/min (sd) | 16.2 (14.6) | 18.8 (28.4) | 15.9 (12.5) | 13.9 (9.5) | 10.7 (7.5) |
| falls | 2.58 | 2.38 | 3.42 | 2.88 | 1.96 |
| block touched in the first minute | 17/24 | 18/24 | 16/24 | 16/24 | 18/24 |
| skills touching % | 30.0 | 30.9 | 32.1 | **37.7** (Δ +7.6, CI [0.0, 15.2]) | 32.9 |
| arrival stop: target in the ToF's view | 32.8 % | 35.1 % | **51.4 %** (Δ +18.6, CI [+9.4, +27.9]) | 34.5 % | 35.0 % |
| outcomes observed % | 63.6 | 68.7 | 70.0 (Δ +6.4, CI [−0.7, +13.6]) | 64.3 | 65.7 |
| map nodes · baked on walks | 41.9 · 15.6 | 40.9 · 14.4 | 42.6 · 15.3 | **35.6 · 6.1** | 41.0 · 14.0 |
| after an escape: displacement 10 s · wall % | 0.47 m · 3.2 | 0.47 m · 3.6 | — | — | — |

**Verdicts (n = 24, one scene).**
- **F1 `NULL`.** The escape now heads for the freest sector, and nothing downstream moves. Escapes are rare (~2.3 a run) and
  last 6 s; the walker's own avoidance and the loops take over within seconds either way. Correct the sign on correctness
  grounds; it is not a behaviour lever at this rate.
- **F2 `WORKING` on its mechanism, `NULL` on behaviour.** The arrival stop's sweep now covers the target: in view 33 → 51 %, the
  one loud effect of the five arms. The outcome loop sees more of its kicks (+6.4 points, CI just crosses 0); boring, skills
  and the first minute do not move. Falls 3.42 vs 2.58 is inside the noise (CI [−0.5, +2.2]); watch it.
- **F3 `PARTIAL`.** The map now bakes on walks only what passes the check: 15.6 → 6.1 walk bakes, 6 fewer nodes (the walk
  bakes were noise). Skills touching +7.6 points at the edge of significance; nothing regresses.
- **F4 `NULL`, a lean.** Walls/min 16.2 → 10.7 and falls 2.58 → 1.96 with half the spread; neither difference clears the
  noise at this n. The map does not grow (41 nodes either way).
**Reading:** the four defects were real and fixing them costs nothing measurable, but none of them is what ★ LA1's behaviour
rests on; the stack of all four is measured next (`log/ten/fx5`).

**The four together** (F5: F4's config + F1–F3's flags; `log/ten/fx5`, n = 24, against the same ★ LA1 base): boring 20.9 %
(Δ −1.2), interesting 48.2 %, walls 11.5 /min (Δ −4.7, CI [−11.0, +1.5]), falls 2.75 (Δ +0.17), the block touched in the
first minute 19/24 (★ LA1 17/24), skills 11.9 a run (Δ +1.7, CI [−0.2, +3.6]), skills touching 35.0 %, train contacts 3.7,
target in view at arrival 50.7 % (Δ +17.9, CI [+9.5, +26.3]), map 36 nodes and 6.4 walk bakes. **Verdict: `PARTIAL`, no
regression.** Every mechanism lands and every behaviour number leans the right way, none beyond the noise at n = 24: a
correctness stack, not a capability. Preset "F5 · the four fixes" beside ★ LA1 for the operator's eye.

**★ F5 PROMOTED on the operator's eye (2026-10-04):** "the subtle effects of the fixes are notable: the robot seems smarter
during short-term decisions (not turning the wrong way into a wall), and seems more accurate during object interactions." The
eye sees what n = 24 could not resolve (the walls and skill-contact leans, the sweep on the target's side). ★ F5 is the demo
configuration for Pollen: config `a1v2_la1_f4_map_wide` + ★ LA1's host args + `--fix-escape-sign --fix-sweep-sign
--map-bake-honest` (launcher preset, second row).
