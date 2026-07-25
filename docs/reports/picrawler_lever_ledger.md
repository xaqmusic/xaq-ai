# Picrawler lever ledger — what is promoted, what is refuted

*The per-lever verdict record for the picrawler active-inference gait (branch
`picrawler-dev`). **Read this before proposing a lever.** Most plausible ideas here have
already been built and falsified; re-proposing a dead one is the single most common way to
waste a session.*

*Companions: [`picrawler_gait_loop_findings.md`](picrawler_gait_loop_findings.md) (the
narrative + architecture), [`../plans-and-designs/picrawler_active_inference_plan.md`](../plans-and-designs/picrawler_active_inference_plan.md)
(the plan), [`../brain_building_doctrine.md`](../brain_building_doctrine.md) (the method),
[`../../CLAUDE.md`](../../CLAUDE.md) (the A/B protocol this ledger's verdicts were produced under).*

**Last updated: 2026-07-25.** Status as of `picrawler-dev` ~`92a2b47`.

---

## How to read this ledger

> **Nothing in this ledger is dead.** Every negative entry means *refuted in the context, at
> the power, and against the baseline stated* — never "this idea does not work." A refuted
> lever whose re-use context arrives is a lever to try again, and several entries below exist
> precisely because a lever was re-tested in a second regime. See `CLAUDE.md` §3.1–3.2 for
> the verdict vocabulary and the checks that must pass before any negative verdict is trusted.

- **A verdict is scoped to its SCENARIO.** "Refuted" means *refuted in the regime tested*.
  A lever refuted on flat ground has **not** been refuted on an incline — re-test it there
  before treating it as settled (and record the second verdict here). Conversely, do not cite
  a flat-ground refutation of a lever whose whole premise is terrain.
- **A verdict is also scoped to its BASELINE.** A null measured against a degenerate control
  is a fact about the control. This project has already had to reopen an entire era of nulls
  for exactly this reason (§6). Before believing a negative entry — including one you wrote —
  run the seven checks in `CLAUDE.md` §3.2.
- **Results before 2026-07-23 are single-seed.** The seed override was broken (it rewrote
  0 modules, so `OGMA_SEED` did nothing and every run was byte-identical). Pre-fix results
  are **byte-perfect isolations but not seed-averaged** — generality untested. Re-measure
  before building on them.
- **Every lever below shipped gain-0-guarded** — at gain 0 the build is byte-identical.
  Refuted levers whose infrastructure is harmless are **kept, default-off**, not ripped out.

### Verdict vocabulary

Every entry carries one of these **plus the scenario, the power, and the baseline** it was
decided against, and — if negative — a re-use context (§6).

| Verdict | Meaning |
|---|---|
| `BASELINE` | in the deployed stack |
| `WORKING` | real positive effect, kept but not in the default stack |
| `PARTIAL` | effect on secondary metrics, null/regression on the primary |
| `NULL` | no measurable effect **at the power and against the baseline stated** |
| `REGRESSION` | measurable negative effect |
| `TAUTOLOGY` | the variant was byte-identical — the mechanism was already on |
| `DEAD_CODE` | no effect because the code path wasn't live in that config |
| `ABLATED` | actively removed for negative consequences |
| `DEFERRED` | built but never tested at adequate power |
| `IN_FLIGHT` | under test now |

**`TAUTOLOGY`, `DEAD_CODE`, and a `NULL` against a broken baseline are measurement outcomes,
not verdicts on the idea.** Before recording any negative entry, run the seven checks in
`CLAUDE.md` §3.2.

---

## 1. The deployed stack (promoted)

Base config: `the_picrawler_motor_epm_embed_corridor_bearinghold.json`, built on the
milestone gait `the_picrawler_motor_epm_embed.json` (emergent CPG-embedding + firm stance;
walks, and improvises a novel limb movement to free itself when stuck).

| Lever | Setting | Verdict & evidence |
|---|---|---|
| **CPG-as-embedding** (`cpg_embed`) — controller learns a phase-conditioned feed-forward from the keyframe error | on | **★ The milestone.** Phase as *context*, not as drive → emergent, self-rescuing gait |
| **Firm stance** (`postural_gain` 0.3→0.7) | 0.7 | Defends the stance → 0 falls, no collision, sharper phase map |
| **KeyframeGait** phase-indexed map + self-precision gate | on | Learns coordination; drives on consistency |
| **BodyRhythmTracker** PLL + CPG entrainment | on | CPG tracks the body — fixes the washout |
| **Heading bearing-hold (P)** — PD on dead-reckoned own-yaw through the *authoritative* skid-steer channel | `heading_bearing_hold_gain=7.0` | **Heading SOLVED.** Straightness 0.05→0.53 **and variance collapses** (net_z std 0.92→0.07). P-sweep {5,7,10,14}: 7 is the sweet spot (10 ties but re-adds a turn outlier; 14 overdrives→0.44). **POSITIVE = go straight; negative = catastrophic spin** |
| **Yaw-rate damping (D)** | `heading_hold_gain=0.3` | Pairs with the P term above |
| **Progress→commit** — sustained forward progress damps exploration σ + adds thrust | `progress_commit_gain=1.0` | **Marginal keeper.** net_z +2.55→+2.62; variance holds; falls unchanged. Seizes a found push, kills the circle-then-go dither. (0.5 too weak, 2.0 over-commits) |
| **Belly ToF rangefinder as the height observation** | `height_topic=reality.proprio.ground_clearance`, `height_k=0.30` | **Hump SOLVED** + Markov-compliant, *the same move*. Replaces the god's-eye `chassis_y_norm` (see §3) |
| **Height-homeostat windup fix** — defense fades with forward progress (`height_rest_frac`) | on | Height is a **standing** reflex: full defense at rest (Gate 0 preserved), →0 while walking. A hip2 lift bias on an incline hoists the legs off the slope = traction loss. Flat net_z +2.62→+3.48, straight 0.67, **falls 0**; teleport-to-hump final_z **+4.11, all seeds clear** (was ~2.6 stall) |
| **Stance-lift knee tuck** — constant knee-tuck bias on **planted legs only** (Cruse foot-height gate; swing legs untouched) | `stance_lift_gain=0.5` | **Belly-up SOLVED.** Clearance 0.015→0.030 (min 0.003→0.008 — off the ground), net_z +1.46→+1.75 (**~20 % faster**), 0 falls, **still clears the hump belly-up** = unified. The stance-gating is the whole trick; a blind DC knee bias kills the gait. Currently in `..._stancelift.json`, pending bake into the base config |

---

## 2. Refuted levers — **do not re-propose without a new scenario**

| Lever | Scenario tested | Why it failed |
|---|---|---|
| **CPG-phase drive** (open-loop per-joint rhythm) | flat | Servo sequencer; chassis slams the ground ("flopping fish"). *Injected a rhythm instead of a prediction* |
| **Keyframe tween** (smooth the staircase) | flat | Low-passes out the adaptive slack → stiff |
| **hip2 stroke** (imposed femur lift) | flat | Energetic but chaotic; no stable gait, frequent falls |
| **hip2 tuck** (femur rest override, alone) | flat | Didn't crouch (weak reflex) + destabilized |
| **Learned hip2** (per-joint postural *profile* loosening the femur spring) | flat, long 60k A/B | No gait/traversal/disengagement gain, **+instability** (firm ~3× faster net traversal, 2× less stuck, 0 tipovers vs loose's 4). The self-rescue is an HK+embedding property on a *stable* base — hip2 was never its source |
| **Phase-indexed velocity objective** (`Cvel` propulsive pump on v*−ẋ) | flat | Marginal steady gain (Cvel self-limits: v* *is* the body's own velocity → error ~0 in steady state), then **amplified a rear-leg asymmetry into circling** (embed 6.1 m straight / 0.47 turns vs velobj ~1.89 turns). Not wrong in principle — incompatible with an *asymmetric* base gait |
| **LR velocity-symmetry prior** (the fix for the above) | flat | **Backfired** — circles worse (~3.19 turns). Magnitude-equalization over-drove the weak leg; yaw is signed/phased, not per-joint RMS. Fixed the wrong layer |
| **Gait symmetry** — amp-homeostat, exploration/coord-adapt, per-leg controller coupling (`ctrl_symmetry_gain`), walk-phasing, balance, heading-hold knobs | flat, **~35 isolated A/Bs** | **Every symmetry-forcing lever → circling.** The RR-under-plant is a stable emergent **tripod-skid** and that asymmetry is **load-bearing for straightness**. One variant hit 3 % knee-amp imbalance *while spinning −19 turns* ⇒ **amplitude symmetry ≠ functional symmetry** |
| **Cruse / Walknet contact-load reflex** (`cruse_gain`, `cruse_rule5_gain`) | flat **and** re-tested on the incline | A second coordination controller firing **out of phase** — its own foot-height detector ≠ the emergent gait phase. Flat: worse everywhere, shatters the variance collapse, +falls. Incline (its supposed home): correct-signed `cruse_gain` final_z 2.54, Rule 5 2.05, both 1.87 — **all worse than the plain 4.13**. **Grip/lift is the wrong instinct: the belly must ride LOW to climb** (or use stance-lift). *This is the model refutation — killed in the regime where its premise applied* |
| **Forward-flow homeostat** (`forward_flow_gain`, amplify stroke ∝ magnitude·predictability) | flat | No distance; falls climb 0.75→1.38→2.75. The predictability term is oscillation-dominated → just raw destabilizing thrust |
| **Proprioceptive balance / `propbal`** | corridor, seed-avg | Refuted as a straightener |
| **stuck→explore** | flat | Refuted — but note this was arguably **the wrong scenario for it**; a stuck-detector has no content on open flat ground. Its inverse twin (progress→commit) was promoted |
| **Lateral-sequence walk phasing** (`gait_phase=[π/2,3π/2,0,π]`) | flat — **but only ever measured inside the rejected open-loop `cpgwalk` config** | **Its +65 % distance is the SEQUENCER's speed, not a gait win — see the correction below.** The only config carrying this phasing is `the_picrawler_motor_epm_cpgwalk.json`, which also carries `rhythm_gains=[1.8,1.0,1.6]` and **no `cpg_embed`** — i.e. the open-loop CPG servo sequencer that was **REJECTED on UI observation** (chassis collision 15.6 % vs 3.3 %; "flopping fish"). It also relocated the asymmetry (RL skids), cost 2 falls, and was fragile to every knob |

> ### ⚠️ Correction (2026-07-25) — the "+65 % walk phasing" number does not mean what it looks like
>
> Earlier versions of this ledger, and the closing line of
> [`picrawler_gait_loop_findings.md`](picrawler_gait_loop_findings.md) §"Gait symmetry", framed
> lateral-sequence walk phasing as a **banked speed lever awaiting stability**. That is wrong, and the
> operator caught it from memory: the gait it produced *looked sequenced and paddle-like, not alive.*
>
> **What actually happened.** In that report, "walk" means `the_picrawler_motor_epm_cpgwalk.json`
> throughout (§"Final tuned gait" names it). That config drives **every joint open-loop from the CPG
> clock**, ignoring ground contact. It was **rejected on UI observation** as a servo sequencer that slams
> the chassis into the ground every step. The +65 % distance was measured on *that* body. The symmetry
> sweep, written a day after the rejection, carried the number forward without re-flagging its source.
>
> **This is a textbook instance of two doctrine failures at once:** a **blind metric** (distance certified
> a degenerate paddling behavior — exactly what
> [`../operational/aliveness_metric_protocol.md`](../operational/aliveness_metric_protocol.md) warns
> distance metrics do), and **fast ≠ walking** (doctrine §8; the collision was invisible until
> `chassis_h` was added). The speed was real. It was the speed of the wrong thing.
>
> **What is genuinely open.** `gait_phase=[π/2,3π/2,0,π]` has **never been tested on the emergent
> `embed` base** — no config combines the two, so phasing has never been isolated from the open-loop
> drive it shipped with. That is a clean, cheap experiment and a legitimate open question. It is *not*
> a +65 % result waiting to be unlocked; any retry starts from zero and must be judged on chassis
> height, belly clearance, and adaptation — not distance.

---

## 3. Substrate decisions (retired, with reasons)

- **God's-eye `chassis_y_norm` → RETIRED** as the height observation. Absolute world-Y reads
  ~1.0 on a raised hump, i.e. **blind to belly grounding**, so it cannot represent terrain
  at all (fixed god's-eye incline final_z 2.84, stalls). It is fine on flat and *only* on
  flat. Replaced by the belly ToF rangefinder — **Markov-blanket compliance and the hump fix
  turned out to be the same move.**
- **Active-balance reflex** — was **inert by default headless** (`publish_tilt` @export
  defaults false and the brain config doesn't set the body export → tilt=0 at MotorEPM), and
  *destabilizing* when enabled (`OGMA_PICRAWLER_PUBLISH_TILT=1` maps tilt→hip2 = pitch/roll
  leveling, not yaw; both signs circle). **Known measurement gap: headless ≠ UI here.**
- **`heading_gain` anti-yaw** — erratic (circled `embed` even at 0.4). Superseded by the
  bearing-hold P+D through the authoritative channel.

---

## 4. Bugs found (fixed — keep in mind as failure shapes)

| Bug | Shape |
|---|---|
| **Cruse `cruse_gain` sign** | Backward at +, forward at − (the emergent gait's foot-height swing is **anti-phase** with the canonical Cruse assumption). Seed-avg: +0.5 → net_z +0.26 (backs up); −0.5 → +2.57 clean forward. Fixed for hygiene; gains remain off |
| **Execution guards `> 0.0`** | Silently disabled **negative** gains — widening a parameter's bound is not enough if the guard rejects the sign. Changed to `!= 0.0` (3 sites) |
| **`postural_gain_joints` as absolute override** | Made the global UI scalar a **silent no-op** — and the robot's behavior improved right afterward by natural evolution, which *looked* causal. **Hand-tuning manufactures false causation; isolate before promoting.** Fixed to a multiplier + `postural_eff` diag so a knob-turn can never silently do nothing |
| **`master_seed` override rewrote 0 modules** | `OGMA_SEED` did nothing; every run byte-identical → **no A/B could be seed-averaged**. Fixed 2026-07-23 (the override now rewrites `seed` too) |
| **Height setpoint slammed the integrator** | `height_k=0.65` (a flat-ground memory) at the tucked-spawn low clearance → 3 startup flips. Lowered to 0.30 |
| **Reset artifact** | Auto-reset teleports fired no bus event, and MotorEPM's leg-phase/EMA survived fall+respawn → **any coherence/TLE trend across a reset was fake**. Fixed by publishing `events.reset` + reset-masking (Gate 0) |

---

## 5. Open frontier

- **Fast flat traversal, belly-up** — the active thread. `stance_lift=0.5` is the current
  best belly-up base; bake it (+ `feet_topic`) into `..._bearinghold.json` as the default.
  An alternative framing offered but not chosen: express stance-lift as a **postural-target
  shift on stance legs** rather than a separate additive bias.
- **Genuine step-over obstacle negotiation** — deferred. Current hump clearance works by
  letting the belly ride low, which **may be a sim exploit** (frictionless belly drag); a
  real chassis could not do it. Obstacle nav is "good enough" for now.
- **A working closed-loop attitude controller** — the balance reflex is a redesign, not a
  knob. Prerequisite for revisiting lateral-sequence walk phasing (+65 % distance, blocked
  on stability).
- **Deferred nav layers** — L1 nav loops, L2 EFE arbiter, L3 keystone, per the plan's build
  order. The nav layer is still the plan's declared *disqualifier* (`target_compass` is an
  oracle); nothing in this ledger addresses it yet.

---

## 6. Re-use contexts — when a refuted lever should be tried again

*A complete verdict names the conditions that would justify a retry. These are live
proposals, not dead entries.*

| Lever | Refuted in | Try again when |
|---|---|---|
| **Lateral-sequence walk phasing** | only ever measured *inside the rejected open-loop sequencer* — its +65 % is that sequencer's speed (see §2 correction) | **As a fresh, cheap experiment — not as a banked win.** Phasing has never been isolated from the open-loop drive: no config combines `gait_phase=[π/2,3π/2,0,π]` with `cpg_embed`. Worth one A/B on the `embed` base now that heading-hold exists — but judged on chassis height / belly clearance / adaptation, and expected to start from zero |
| **stuck→explore** | flat ground, where a stuck-detector has no content — *the wrong scenario for it* | A regime where the body genuinely gets stuck: terrain, corridor corners, obstacle contact. Its inverse twin (progress→commit) was promoted, so the family is not dead |
| **Phase-indexed velocity (`Cvel`)** | on the *asymmetric* tripod-skid gait, which it amplified into circling | The base gait becomes symmetric, or the pump is gated by heading error so it cannot amplify yaw asymmetry. Explicitly "not wrong in principle" |
| **Cruse / Walknet contact-load reflex** | flat **and** incline; out-of-phase with the emergent gait | Its foot-height detector is replaced by the *emergent gait's own phase* (the failure was phase-misalignment, not the coordination idea). Note the historical warning: an earlier Walknet null rested on a 1-of-6-rule slice, so scope any future claim carefully |
| **Learned hip2** | flat, against a stable base | A regime where the femur must do real work — steep terrain, step-over. It was refuted as a *gait* lever, not as a terrain lever |
| **Gait symmetry (all forms)** | flat, ~35 A/Bs; the asymmetry is load-bearing for straightness | A different base gait exists whose straightness does not depend on the tripod-skid. Amplitude symmetry ≠ functional symmetry — any retry must target functional symmetry |
| **Active-balance reflex** | headless (inert — `publish_tilt` off) and UI (destabilizing — maps tilt→hip2, i.e. pitch/roll, not yaw) | It is redesigned as a real closed-loop attitude controller. The existing verdict is mostly a **DEAD_CODE + wrong-target** finding, not a verdict on balance |

---

## 7. Inherited failure patterns (from the pre-split RL era)

*These come from `ami-ogma/docs/findings/mechanism_registry.md`, this ledger's ancestor. That
registry covers the **reward-shaped RL era** the doctrine now disowns, so its individual
mechanism verdicts do **not** transfer. The **failure shapes** and **measurement lessons**
do — they were paid for at enormous cost and several have already recurred on the current
reward-free stack.*

| Pattern | Signature | Recognize it by |
|---|---|---|
| **A — constrains without unlocking** | Variance tightens, mean does not lift | The mechanism *restricts* the policy rather than opening new policy space. Most nulls in this project's history were variance-constrainers |
| **B — stability bought with exploration** | Stability up, translation/discovery down | Damping motor noise or over-committing to stance suppresses the very exploration that finds the behavior |
| **C — dead code** | Byte-identical to control | The affected path isn't live under this config. **Not a verdict on the idea** |
| **D — tautology** | Byte-identical to control | The knob was already at that value. **Not a verdict on the idea** |
| **E — activity without aliveness** | Motion/energy metrics climb while goal metrics collapse | A degenerate attractor found by the agent: jitter, lurch, circling, lying down. Often arises from *channel interactions* that are each individually correct |
| **F — open-loop primitive with no differential outcome** | The selector learns mechanically but nothing translates | If no primitive changes the world usefully, the selector has no advantage signal. An open-loop primitive library is not a substitute for closed-loop control |
| **G — trades stability for reach** | Proposed, then **withdrawn** | The apparent reach gain was a single-seed outlier; only the stability cost was real. A cautionary tale about naming a pattern before powering it |

**The measurement lessons that came with them** — all now in `CLAUDE.md` §3.2 as
pre-verdict checks: baseline validity (a null against a degenerate control is not a null);
silent confounds (a harness flag that never got set invalidated an entire A/B family);
faithfulness (a 1-of-6-rule slice is not the mechanism); and consumer verification (a
published topic nobody acts on reads exactly like a null).

**Two inherited *open* questions**, neither ever settled: whether a metric should score
**aliveness** (closed-loop adaptive reorganization) rather than distance — the old repo
concluded distance metrics "select against aliveness"; and the **unambiguous-emergence
bar** (real capability should be *loud in a single run*, the way standing was) versus
seed-averaging small deltas. See `CLAUDE.md` §3.3 for how these two reconcile.

---

*Append every new verdict here — **with its scenario, its power, and its baseline** — as soon
as it is decided, plus the re-use context that would justify a retry (§6). When a verdict
generalizes into a reusable principle, fold that principle into
[`../brain_building_doctrine.md`](../brain_building_doctrine.md) and leave the specific
result here.*
