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

**Why the notes do not block R1:** R1's critical consumer is the *standing* bank, and its
key (the pure standing node) exists in both seeds at high purity and high share. Mixed
nodes blur only the transitional banks — which today do not exist at all, so their floor is
the status quo. The two ⚠ items are recorded as R1-era conditioning work (mitosis
triggering, transition-surprise sharpening), not gate failures.
