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
