# Picrawler lever ledger — what is promoted, what is refuted

*The per-lever verdict record for the picrawler active-inference gait (branch
`picrawler-dev`). **Read this before proposing a lever.** Most plausible ideas here have
already been built and falsified; re-proposing a dead one is the single most common way to
waste a session.*

*Companions: [`picrawler_gait_loop_findings.md`](picrawler_gait_loop_findings.md) (the
narrative + architecture), [`../plans-and-designs/picrawler_active_inference_plan.md`](../plans-and-designs/picrawler_active_inference_plan.md)
(the plan), [`../brain_building_doctrine.md`](../brain_building_doctrine.md) (the method),
[`../../CLAUDE.md`](../../CLAUDE.md) (the A/B protocol this ledger's verdicts were produced under).*

**Last updated: 2026-07-27.** Status as of `picrawler-dev` ~`ea9265b` + the stroke-to-step
lock work below.

> ⚠️ **Every corridor number recorded before 2026-07-27 is from a DIFFERENT GYM.** The
> corridor's back wall was a vertical seal a robot could park against, and its far end
> dropped off the world; both are now 30° self-centering ramps. Re-measured deployed
> baseline (corridor, n=4, 6000 ticks): net_z **4.58 ± 0.27**, straight 0.73, flat_v 0.05,
> step_bal 0.48, tilt_sd 0.065, planted 3.69, 0 falls. See the harness-defects box in §2.

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
| **Stance-lift knee tuck** — constant knee-tuck bias on **planted legs only** (Cruse foot-height gate; swing legs untouched) | `stance_lift_gain=0.5` | **Belly-up SOLVED.** Clearance 0.015→0.030 (min 0.003→0.008 — off the ground), net_z +1.46→+1.75 (**~20 % faster**), 0 falls, **still clears the hump belly-up** = unified. The stance-gating is the whole trick; a blind DC knee bias kills the gait. Currently in `..._stancelift.json`, pending bake into the base config. **2026-07-25 — gain sweep 0/0.5/0.8 (n=4): 0.5 confirmed as the optimum, and its cost is now understood.** Belly clearance rises monotonically with gain (0.016→0.026→0.033) but so does damage: 0.8 gives net_z 1.87, straight 0.28, tilt_sd 0.178, 0.75 falls. The cause was **not** the bias — it was the swing detector it gates on (see §4), and the reason "the stance-gating is the whole trick" is subtler than recorded: what makes it rhythm-safe is that the bias is *modulated*, but it was being modulated by a self-referential signal beating against the gait clock |
| **★ Reward-free coordination fitness** — the (1+1) phase search keeps running, but ranks probes by `coherence · activity / (1 + tle)` instead of by forward velocity | `coord_fitness_mode=1` (0 = legacy fwd_v, byte-identical) | **Resolves the §4 reward problem at no cost.** n=4 vs the fwd_v-reward arm: net_z 3.52→**3.76**, straight 0.67→**0.70**, tilt_sd 0.069→**0.067**, 0 falls; hump gate **identical** (final_z 5.21, gain_z 2.59); recovery gate **better** — coordination returns 0.30→**0.48** after perturbation, which is the structural point: *a thrash can no longer become the incumbent, because a thrash has high `tle` and low coherence.* For contrast, deleting the search outright costs net_z 3.52→2.36 and step_bal 0.26→**0.03** ⇒ **the capability was the SEARCH, not the reward.** **Why all three factors are needed** (each was measured, not assumed): coherence alone is *maximal on a frozen body* at all-equal offsets — R = \|mean e^{−iP_j}\| = 1 — and the search can reach those offsets; `1/(1+tle)` alone also favours freezing (a still body is trivially predictable — the same reason Cphi was deliberately not trained on HK surprise); **the activity term is the homeokinetic normalisation that kills both**, since it → 0 when the body stops. Predicted failure mode was coherent marching-in-place; it did not occur. **Honest caveat: post-perturbation progress is LOWER (1.35→0.78)** — the fwd_v reward was directly buying that, and removing it costs some of it |
| **Adaptive coordination** — a leaky first-order tracker pulling the Kuramoto target offsets toward the body's OWN measured per-leg phase pattern | `coord_adapt_rate=0.001` (0.005 over-drives: net_z 2.84, +falls) | n=4: **ties the baseline on every progress metric** (net_z 3.52 vs 3.62, flat speed 0.04, straight 0.67 exactly) while reducing wobble on **4/4 seeds** (tilt_sd 0.078→0.069). **Passes the obstacle gate and improves it** — teleport-to-hump final_z 4.57→**5.21**, better on 4/4 seeds, 0 falls. ⚠️ **The RATIONALE first recorded here was wrong** — it was written as "replaces an imposed topology with a learned one," but `coord_reward_drive=0.3` was already reward-searching that topology (see §4), and it **overwrites `gait_phase` wholesale every 240 ticks**, discarding this tracker's nudges unless a probe wins. The measured A/B stands (the ratchet was equally on in both arms); the *explanation* does not. **Safety property that does hold, and it is the important one:** a leaky tracker has **no stored winner and no fitness**, so unlike the ratchet it has nothing to lock in — it always tracks what the body is doing *now* (τ ≈ 17 s). **Pending UI observation** |

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
| **Cruse Rule 3 (contralateral load), isolated with Cruse LIVE** (`cruse_gain=0.5`, `cruse_rule3_weight` 0.5 → 0.0) | corridor + coordadapt base, n=3 seed-avg — **and confirmed by operator UI observation the same day** | **Rule 3 is the harmful part.** Turning it off while leaving Rule 1/2 live: net_z 1.95→**2.30**, straight 0.28→0.40, tilt_sd 0.217→**0.085**, falls 0.67→0.33, parasitic steps 167→129. Operator independently: "a few trot-like steps emerge, but it fights the natural motion; setting it to 0 traverses obstacles without regression." Consistent with the standing Cruse verdict — a second coordination controller fighting the emergent gait — and now localized to the *contralateral* rule specifically. **Caveat: this is MotorEPM's Rule 3, not CruseCoordinator's — see §4** |
| **Cruse / Walknet contact-load reflex** (`cruse_gain`, `cruse_rule5_gain`) | flat **and** re-tested on the incline | A second coordination controller firing **out of phase** — its own foot-height detector ≠ the emergent gait phase. Flat: worse everywhere, shatters the variance collapse, +falls. Incline (its supposed home): correct-signed `cruse_gain` final_z 2.54, Rule 5 2.05, both 1.87 — **all worse than the plain 4.13**. **Grip/lift is the wrong instinct: the belly must ride LOW to climb** (or use stance-lift). *This is the model refutation — killed in the regime where its premise applied* |
| **Forward-flow homeostat** (`forward_flow_gain`, amplify stroke ∝ magnitude·predictability) | flat | No distance; falls climb 0.75→1.38→2.75. The predictability term is oscillation-dominated → just raw destabilizing thrust |
| **Proprioceptive balance / `propbal`** | corridor, seed-avg | Refuted as a straightener |
| **★ TRUE foot-contact sensor as the swing gate** (`contact_topic=reality.proprio.foot_contact` — the physics touch flag, wired in place of the foot-height inference) | corridor + reward-free base, n=4, **plus a compensating `stance_lift` sweep {0.3, 0.35, 0.4, 0.5}** | **REFUTED — and it is the most counter-intuitive result in this ledger: the ACCURATE sensor produces a WORSE gait.** net_z 3.76→**2.37**, straight 0.70→**0.46**, tilt_sd 0.067→0.088 (belly clearance did improve, 0.027→0.030). First hypothesis was a confound rather than a defeat — true contact reports stance 77 % of the time vs the proxy's 59 %, so every consumer's *effective gain* rises — but the compensating sweep **kills that explanation**: 0.3→2.53, 0.35→2.10, 0.4→1.95, none anywhere near 3.76. **Diagnosis: the height-EMA "detector" was never measuring contact — it was measuring the foot's PHASE within its own cycle (above/below its moving average), and a phase-locked gate is what `stance_lift` actually needs.** "Push down on this foot" wants *is this foot in the propulsive part of its cycle*, not *is this foot touching*. The inaccurate proxy was accidentally the right signal. This is the same lesson the Cruse re-use context already predicted from the other direction — that its foot-height detector should be replaced by *the emergent gait's own phase*. **Kept default-off; its real value is as an INSTRUMENT** (ground truth for duty factor — it is what showed the world-height proxy under-reports swing by 16×) **and as the input a genuine load/Cruse consumer would need.** Re-use: for a consumer that truly needs contact (load distribution, step-over foot placement), not for anything that wants gait phase |
| **Phase readout moved to the stride joint** (`phase_joint=0`, hip1 instead of the legacy knee) | corridor + stance-lift base, n=4 × a **4-point `stroke_phase` sweep** covering the full circle {−2.85, −1.28, +0.29, +1.86} | **`REGRESSION`, and structurally so — not a tuning miss.** Locomotion collapses at *every* phase offset: net_z 3.62 → {0.83, 0.25, −0.14, 0.55}, flat speed → ~0.00, falls 0 → {2.5, 0.75, 2.0, 1.0}, tilt_sd 0.078 → {0.34, 0.18, 0.23, 0.13}. **Cause: the stroke and the phase readout cannot share a joint.** `L.phase = atan2(hip1 velocity, hip1 deviation)` while the stroke drives `y[0] += amp·sin(L.phase + stroke_phase)` on that same hip1 → a self-excited oscillator. Sweeping the full circle proves the feedback is fatal rather than mis-offset. **Note what it DID do: step-balance rose 0.30 → 0.41–0.58 and feet-planted rose** — locking coordination to the stride joint really does even out the legs, so the *premise* is intact; only this wiring of it is refuted. See §6 for the re-use context |
| **stuck→explore** | flat | Refuted — but note this was arguably **the wrong scenario for it**; a stuck-detector has no content on open flat ground. Its inverse twin (progress→commit) was promoted |
| **Lateral-sequence walk phasing** (`gait_phase=[π/2,3π/2,0,π]`) | flat — **but only ever measured inside the rejected open-loop `cpgwalk` config** | **Its +65 % distance is the SEQUENCER's speed, not a gait win — see the correction below.** The only config carrying this phasing is `the_picrawler_motor_epm_cpgwalk.json`, which also carries `rhythm_gains=[1.8,1.0,1.6]` and **no `cpg_embed`** — i.e. the open-loop CPG servo sequencer that was **REJECTED on UI observation** (chassis collision 15.6 % vs 3.3 %; "flopping fish"). It also relocated the asymmetry (RL skids), cost 2 falls, and was fragile to every knob |
| **Lateral-sequence phasing, now isolated on the `embed` base** (`gait_phase=[0,π,3π/2,π/2]` on the `[FL,FR,RL,RR]` order — the vector above, rotated to make leg 0 the reference) | corridor + stance-lift base, n=4 seed-avg — **the clean isolation the correction below asked for** | `NULL`/slight `REGRESSION`. The *mechanical* prediction held exactly — de-clustering the swing targets raised feet-planted **3.30 → 3.63** — but it bought nothing behavioral: net_z 3.62→**3.16** (and variance doubled, std 0.37→0.94), straight 0.67→**0.55**, belly 0.026→0.024, 0→**0.25** falls, flat speed **unchanged**. **Diagnosis: the static-stability argument was right and irrelevant.** Trot is dynamically stable and *should* be wobbly at this body's crawl speed — but the body was already averaging 3.3 of 4 feet planted, so there was almost no support-polygon headroom to win back. The phasing question is now **closed on this base**, not deferred |

| **★ LOAD-GATED POWER STROKE** (`stroke_load_gain` — scale each leg's propulsion by its share of measured hip1 load, so a leg pushes in proportion to the ground it actually has) | corridor + the deployed `..._imufused` base, n=4 seed-avg × a **5-point gain sweep {0.5, 1, 2, 4, 6}** at the standard 6 000 ticks | **`NULL` on progress, `PARTIAL` on leg participation. Not promoted.** No gain beats the baseline on distance or straightness: net_z 4.75 → {4.74, 4.33, 4.52, 4.24, 4.47}, straight 0.74 → {0.74, 0.71, 0.72, 0.69, 0.70}, and **`flat_v` stays pinned at 0.04–0.05 — the ninth lever to leave it there** (§5). 0 falls in every arm. The one real movement is **`step_bal` 0.44 ± 0.16 → 0.54 ± 0.08 at gain 2** (the legs do share the gait more evenly, and the variance halves) — but it costs `steps` 53 → 43.5 and buys no progress. **The consumer demonstrably fired**: gate spread scales monotonically 0.28 → 1.87 across the 6.6× gain range while `stroke_gate_mean` holds at 1.00 (the gate is mean-normalized, so it redistributes thrust rather than attenuating it). **Why the null is believable rather than a measurement failure: it is what the mechanism predicts.** A purchase gate removes thrust spent in the air, but *during stance* it scales the push and drag halves of the stroke equally — and the boxed finding above measured that balance at 50/50. A magnitude gate cannot fix a timing problem. Built on the honest signal the ledger asked for (§6 named `joint_torque` as the load observation Walknet never had) and on the joint that measurement — not assumption — picked |

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

### ✓ 2026-07-25 — THE GOD'S-EYE SWING SIGNAL IS GONE, replaced by something the real robot can build

**RESOLVED.** `feet_y` (absolute world-Y) is no longer the swing input. The replacement is
**`feet_y_gravity_cmd` = forward kinematics from the COMMANDED servo angles, projected onto
the accelerometer's gravity-up axis** — IK ⊕ IMU, and buildable on the physical PiCrawler,
whose hobby servos report no position at all.

| swing input | legal? | on hardware? | net_z | straight | hump final_z |
|---|---|---|---|---|---|
| `feet_y` — world-Y **oracle** | ✗ | ✗ | 3.76 ± 0.40 | 0.70 | 5.21 ± 0.67 |
| `feet_y_body` — chassis-frame FK | ✓ | ✓ | 2.52 ± 1.34 | 0.50 | — |
| `feet_y_ground` — FK + belly ToF | ✓ | ✓ | 2.26 ± 1.65 | 0.46 | — |
| `foot_contact` — true touch flag | ✓ | **✗ (no foot switches)** | 2.37 ± 0.86 | 0.46 | — |
| `feet_y_gravity` — FK from **achieved** pose | ✓ | ✗ (needs encoders) | 3.95 ± 0.56 | 0.62 | 5.35 ± 1.15 |
| **`feet_y_gravity_cmd`** — FK from **commanded** angles | ✓ | **✓** | **4.36 ± 0.28** | **0.74 ± 0.01** | **6.10 ± 0.46** |

**The property that mattered was a GRAVITY REFERENCE.** Three legal candidates lost badly and
two phase hypotheses died at chance agreement (0.49–0.56); all of them lacked it, and an
accelerometer supplies it directly. **And the hardware's poorer information proved BETTER**:
servo deflection (measured 22 mm mean / 38 mm max at the foot; FK chain validated to 1.1 mm)
is *noise* to the gate, so the commanded angles — the clean intended trajectory — give tighter
straightness (± 0.01 across seeds) and halved distance variance. Full analysis:
[`sensor_legitimacy_and_the_feet_y_oracle.md`](../plans-and-designs/sensor_legitimacy_and_the_feet_y_oracle.md).

**2026-07-26 — the attitude gap is closed too, and the result improved again.**
`feet_y_gravity_cmd_imu` uses a MODELLED accelerometer + gyro complementary filter (no exact
simulator attitude anywhere), sampled and filtered at the **240 Hz physics rate** rather than
the 50 Hz brain tick — real IMUs run 1–8 kHz, and the physics was already oversampled to stop
foot tunnelling, so that bandwidth was free. Measured attitude error **3.17° mean / 9.2° max**
(accel-only: 21.3°). n=4 vs the oracle: net_z 3.76→**4.75**, flat_v 0.04→0.05, straight
0.70→0.74, **tilt_sd 0.067→0.068 (wobble gap CLOSED)**, hump 5.21→**6.09**, 0 falls. It beats
even the exact-attitude arm (4.36) — the third time a hardware-honest model beat its idealized
twin, each of which happens to include smoothing.

**ROBUSTNESS MATRIX (`robustavg.py`, 8 sensors × 2 actuation backends): every
gravity-referenced signal emerges in BOTH backends, 8/8 — and the god's-eye oracle does NOT
(it falls on g6dof).** Overall emergence 10/16, net_z spread 2.4×. **A gravity reference is
the invariant; the substrate is robust in kind, sensitive in degree.** Full matrix in the
design note §6.

<details><summary>Historical: the entry as written when this was still open</summary>

#### ⚠️ THE DEPLOYED GAIT DEPENDS ON A GOD'S-EYE SIGNAL, and no legal substitute recovers it

**`reality.proprio.feet_y` is absolute WORLD-Y** (`_lowers[i].global_transform.origin.y −
L3*0.5`, picrawler_body.gd) — a quantity no physical picrawler can sense, and **the same
violation that retired `chassis_y_norm`** (§3). It is the input to MotorEPM's swing detector,
which gates `stance_lift` and every Cruse rule. It has been there the whole time and was not
caught until the operator asked where the brain thinks its feet are.

The detector partially launders it — `foot_y > foot_y_ema` is a *difference* of world-Y
values ~50 ticks apart, which on flat ground with a steady chassis ≈ the foot's motion
relative to the body (legitimate proprioception). But the difference still contains the
body's own vertical drift, so it is contaminated exactly where terrain matters. And
`feet_y < stance_y_threshold` (the body's "planted" test) is **doubly** god's-eye: absolute
world height against a fixed constant.

**Three Markov-compliant replacements were built and seed-averaged (n=4). All lose, by a
similar margin, with 2–4× the variance:**

| swing-detector input | legal? | net_z | straight | tilt_sd |
|---|---|---|---|---|
| `feet_y` — absolute world-Y | **✗** | **3.76 ± 0.40** | **0.70** | **0.067** |
| `feet_y_body` — foot pose vs chassis (encoder FK) | ✓ | 2.52 ± 1.34 | 0.50 | 0.095 |
| `feet_y_ground` — FK + belly ToF (terrain-relative) | ✓ | 2.26 ± 1.65 | 0.46 | 0.101 |
| `foot_contact` — true physics touch flag | ✓ | 2.37 ± 0.86 | 0.46 | 0.088 |

`feet_y_body` is the cleanest experiment: **identical formula, identical toe approximation,
only the reference frame changed** — so the delta isolates the god's-eye component and
nothing else. **It is load-bearing.**

**Best available reading (NOT verified):** world-Y and body-relative differ by exactly the
chassis's own vertical motion, so what the illegal signal carries is the **body's bounce** —
a whole-body vertical phase reference — and gating a knee push on that syncs it with when
the legs are actually being loaded. The `feet_y_ground` arm was built to reconstruct that
legally (belly ToF supplies body height) and **did not recover it**, which either falsifies
the reading or reflects the ToF being a noisier, single-point, short-range substitute for
true chassis height. Untried and the most promising legal source of body-bounce phase: the
**IMU's vertical acceleration**, which is a real sensor and needs no differencing of a range
finder.

</details>

**Status when written: an OPEN LEGITIMACY PROBLEM (now resolved above).** Options, all requiring an operator
call: (a) accept the dependency and state it in every claim; (b) keep hunting a legal
reconstruction (IMU vertical phase is next); (c) re-tune the consumers *on* a legal signal —
note every arm above ran `stance_lift=0.5`, tuned for the god's-eye signal, and the high
variance suggests instability rather than a simple offset. **Until resolved, any "the gait
works" claim should carry the caveat that its swing gate is not physically realizable.**

---

### ★ 2026-07-26 — CATASTROPHIC FORGETTING AFTER INVERSION WAS AN INTEGRATOR-WINDUP BUG

**Operator observation:** placed on angled/complex surfaces the robot flips onto its back,
lies tucked, then homeokinesis breaks the lock and it **self-rights** (~27 s measured, ~1 min
observed) — a large emergent win. But afterwards the walk never recovers: exaggerated
movements, wrong phasing, no forward traversal.

**Measured** (`forgetavg.py`, deterministic flip via the pre-existing
`OGMA_PICRAWLER_TELEPORT_FLIP=1`, `AUTO_RESET_INVERSION=0`): after self-righting, forward
progress stayed **NEGATIVE for 7200+ ticks** against a `+0.892` pre-flip baseline, with **zero
recovery trend** across six 1200-tick bins.

**Per-variable attribution — and it overturned the expected diagnosis:**

| variable | pre-flip | after righting | verdict |
|---|---|---|---|
| `chassis_h_max` | 0.999 | 0.999→1.000 | **not the mechanism** — the height signal is clamped to [0,1] so the "monotonic max" is already saturated and cannot ratchet |
| **`height_bias`** | −0.191 | **−0.500, all six bins** | **LATCHED at its clamp** |
| **`amp_gain`** | 0.100 | **2.9–4.3, never returns** | **LATCHED 30–40× high** ⇒ "movements are exaggerated" |
| `coord_best_fitness` | 0.336 | decays normally | not implicated |
| `motor_tle` | 0.278 | returns to band | **HK self-model re-adapts fine** |

**The learned structures recover; only the two homeostat INTEGRATORS latch.** So this is not
the "monolithic weights get overwritten, we need capacity allocation" problem it was first
framed as — it is two unbounded integrators accumulating in a regime where their setpoint is
meaningless, then being unable to unwind. Far smaller and entirely local.

**The general shape, worth keeping:** *an integrator must not accumulate where its error
signal is invalid, and it must be able to unwind anywhere it can wind.* `height_bias` violated
both — it integrates while inverted, and `height_rest_frac` (the incline windup fix) then
freezes it while the robot walks, so **the more it tried to walk the longer it stayed broken.**

> #### ⚠️ CORRECTION (same day) — the gate is NOT promoted: it may disable the escape
>
> Follow-up at n=8 on an angled surface: **the gated arm self-righted 0/8, the ungated arm
> 1/8.** 1-vs-0 is not significant alone, but the mechanism is specific and strong:
> **`amp_gain` winding 0.100 → 4.0 is a 40× amplitude escalation, and that violent flailing
> is WHAT RIGHTS THE ROBOT.** Freezing the integrator removes the escape drive. The poison
> and the capability are the same mechanism — so the gate protects the walk by lesioning a
> working loop, exactly what CLAUDE.md §5.4 forbids.
>
> **Reverted:** the launcher CURRENT is back to `..._imufused.json` (the forgetting flaw
> documented and unfixed) and `..._uprightgate.json` is parked as an UNRESOLVED TRADE.
> `homeo_upright_gate` stays in the code default-off.
>
> **The redesign the evidence points to — intervene on the TRANSITION, not the state:**
> *snapshot the homeostat integrators when uprightness is lost and restore them when it is
> regained*, making the inverted excursion scratch space. Full wind allowed (escape
> untouched), posture restored exactly rather than slowly unwound. That is "plastic" in the
> operator's sense; a freeze is the opposite.
>
> **Also — a power fix for any retest.** Self-righting runs ~10–30 %, so n=8 buys one event.
> `_teleport_to` applies a RELATIVE 180° rotation, so with `TELEPORT_EVERY` a second flip
> turns an inverted robot upright again — making recovery deterministic and giving full n on
> the post-recovery walk. That separates "does the fix restore the walk" (fully powered) from
> "does the fix preserve self-righting" (stochastic, needs its own larger run).
>
> **What the ungated n=8 curve added:** the latch does eventually release — `height_bias` sat
> at −0.500 for six bins then crept to −0.256 by 12 000 ticks — and progress partially
> recovers (+31 % of baseline at ~7 000 ticks) before collapsing to −63 % and drifting back
> to +14 %. So it is a ~200 s erratic degradation, not a permanent flatline.

**Mechanism fix (measured, but see the correction above) — `homeo_upright_gate=0.5` on `reality.proprio.upright`:** Normal walking
is byte-identical (net_z 4.75, flat_v 0.05, straight 0.74, tilt_sd 0.068, 0 falls) because
`upright` stays ~1.0 while walking so the gate never engages; inverted, `height_bias` holds
(−0.093 vs +1.497) and `amp_gain` **does not move at all** (0.100 → 0.100).

**Two process notes.** (1) The gate was initially wired to `tilt_topic` and was **silent dead
code** — the body's `publish_tilt` defaults FALSE so tilt never arrives headless and `upright_`
sat at its 1.0 init. Caught only by checking that the consumer fired (`amp_gain` wound
*identically* gate-on and gate-off). §3.2 rule 5, again. (2) `height_unwind_free` (asymmetric
windup fade, so a railed bias can unwind while moving) was also built and is **NOT promoted**:
it addresses recovery from a wind the gate prevents, and it cost ~8 % net_z and ~12 % hump.
Kept default-off as infrastructure.

**Also learned:** inverted on **flat** ground the robot never self-rights (0/3 seeds, 16 000
ticks) and `motor_tle` **falls** 0.24→0.08 — inverted-on-flat is a *low-surprise attractor*, so
homeokinesis has no pressure to escape. The self-righting win depends on the terrain's ongoing
disturbance keeping TLE high. **Still unmeasured at power:** the end-to-end post-righting
recovery curve, since self-righting is stochastic (1 of 3 seeds even on an angled surface).

---

### 2026-07-26 — the PLASTICITY / FORGETTING family (nothing promoted; `homeo_leak_cycles=0`)

The forgetting-after-inversion failure is an **integrator-windup** bug, not a learned-weights
one (see the entry above). Four mechanisms were built and measured. **None is promoted** — at
`homeo_leak_cycles=0` the stack performs as well as it ever has, and the operator's read is
that recovery does happen after a stuck period, just slower than wanted.

| Lever | Verdict |
|---|---|
| **`homeo_upright_gate`** — freeze the integrators while not upright | **`REGRESSION`.** Stops the latch (amp_gain 2.53→0.100, height_bias +1.50→−0.09) and normal walking is byte-identical — but self-righted **0/8 vs 1/8** ungated. **`amp_gain` winding 40× IS the flail amplitude that rights the robot**, so freezing it lesions the escape (§5.4). Do not use |
| **`height_unwind_free`** — asymmetric windup fade so a railed bias can unwind while moving | **`NULL`/`REGRESSION`.** Addresses recovery from a wind the leak/gate prevents anyway, and cost ~8 % net_z and ~12 % hump. Default-off infra |
| **`homeo_leak_cycles`** — leaky homeostats, rate in stride cycles off the body's own ω | **`PARTIAL`.** At 5 cycles FREE on flat (net_z 4.78 ± 0.16 vs 4.75; tilt_sd 0.068 identical) and bounds the amp_gain excursion by half (2.53→1.18). **But a CONSTANT leak blocks escape entirely: 0/4 off the operator's wall vs 2/4 with the leak off.** 2 cycles is too aggressive. Target matters — leaking toward *unity* (semantically neutral for a gain) cost net_z 4.75→3.6 because normal walking pins the integrator at its FLOOR; retargeted to minimum authority. **Semantically neutral ≠ behaviourally neutral** |
| **`homeo_leak_upright_only`** — posture-gate the leak: forget while upright, accumulate while not | **Best variant, still not promoted.** Escape restored (**2/4 @ 72 s** vs baseline 2/4 @ 96 s), free on flat (4.78, tilt_sd 0.068, 0 falls), hump 5.68 vs 6.09 (−7 %). The **inverse** of `homeo_upright_gate` — which is exactly why it works: accumulate while inverted (escape), forget once upright (fast recovery) |
| **`homeo_leak_progress_gate`** — also stop forgetting while stalled | **`REGRESSION`, two attempts.** Second operator observation: upright but BLOCKED at a slanted wall, the robot "no longer learns how to climb, it stays in the same gait it used on the flat" — a posture gate cannot catch that. Gating *both* integrators on stall cost hump 6.09→4.84 (stalling on a hill readmits the height windup). Gating *only* amp_gain: net_z 4.33, **falls 0.25**, hump 4.90, escape 1/3 @ 228 s. **Cause: `height_rest_frac` is INSTANTANEOUS progress, so it fires on ordinary gait pauses.** The signal wanted is a SUSTAINED stall — `stuck_ticks_`/`stuck_boost_` (~5 s) — which must first be decoupled from `stuck_explore_gain` |

**Two principles earned here.** (1) *Forgetting is a luxury of success; escalation is the
response to failure* — the wind-up is not only damage, it is how the body gets out of trouble,
so any mechanism that bounds it must not bound it while the body is failing. (2) **The leak had
to become per-integrator**, because `amp_gain` is EFFORT (may escalate when failing) and
`height_bias` is POSTURE BIAS (must never accumulate on a slope — that is the refuted windup).
*Which state may escalate depends on what that state means.*

**Open:** the upright-but-blocked case. Two failed attempts; needs the sustained-stall signal.

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
| **★ `coord_reward_drive=0.3` was live in the ENTIRE config lineage — including the `embed` milestone** (found 2026-07-25 by `mkarm.py`'s tautology guard refusing to write a "no-op" arm) | It is a **(1+1) hill-climb that KEEPS the highest-fitness probe and reverts to it** (`coord_best_phase_`), fitness = `fwd_v` minus wobble/lateral penalties, incumbent score decaying only `0.99` per 240-tick window (**τ ≈ 400 s**). Three consequences. **(1) It is a reward.** Probe → score by forward velocity → keep the winner. That sits in direct tension with §5.1 ("no reward shaping, ever — intrinsic homeostatic valence only"), and it has been on since the milestone, so *every* result in this ledger was measured with it running. **(2) It is structurally the destructive-crystallization shape** the operator identified from experience: a violent escape thrash that momentarily scores high `fwd_v` becomes the incumbent, and subsequent normal probes score lower and are reverted *back* to the thrash. **(3) It overwrites `gait_phase` wholesale every 240 ticks** from `coord_best_phase_` + noise — so `coord_adapt_rate`'s slow nudges are discarded each window unless a probe happens to win. **This invalidates the stated RATIONALE for the coord_adapt promotion** (it was never replacing an "imposed" topology — the topology was already being reward-searched); the A/B itself stands, since the ratchet was equally on in every arm. **Ratchet-OFF A/B (n=4):** it is *load-bearing for distance* — stancelift net_z 3.62→**2.90**, step_bal 0.30→**0.11**; coordadapt net_z 3.52→**2.36**, step_bal 0.25→**0.03** (a leg stops participating) — while *improving* wobble (tilt_sd 0.078→0.066). So a reward-driven search is producing a real share of the forward progress and nearly all the leg participation in a stack that is supposed to be reward-free. **Unresolved: this needs an operator decision, not a unilateral fix** |
| **TWO different Rule-3 parameters, in two different modules** (2026-07-25) | `MotorEPM::cruse_rule3_weight` is a **sub-weight inside the `cruse_gain` block** (single use site, MotorEPM.cpp:1503) — **inert whenever `cruse_gain == 0`, which is its default** — yet its own default is **0.5**, so the MOTOR-EPM panel *displays it as enabled while it does nothing*. Separately, `CruseCoordinator` has its own `rule3_weight`, gated by `cruse_bias_gain` whose default is **1.0 = ON**. The MOTOR-EPM panel slider writes the FORMER (`motor_epm_panel.gd:208` → `set_param` on `motor_epm`), so **zeroing MotorEPM's `cruse_gain` says nothing whatsoever about CruseCoordinator's Rule 3.** Verified by measurement, not inference: with `cruse_rule3_weight` at its **max (2.0)** and `cruse_gain=0`, MotorEPM's contribution is **exactly 0.00000** and `swing_frac` is bit-identical to the default arm; at `cruse_gain=0.3` it is 0.358. Instrumented with a `cruse_bias` diag (mean \|MotorEPM's own Cruse contribution\|, in diag + snapshot + the body's stdout JSON) so this is answerable by a number. **Shape: the same rule name in two modules with opposite default gating is the `postural_gain_joints` silent-no-op trap wearing a second module.** Consequence for the record: **any historical Cruse verdict must name WHICH module it tested** |
| **Height setpoint slammed the integrator** | `height_k=0.65` (a flat-ground memory) at the tucked-spawn low clearance → 3 startup flips. Lowered to 0.30 |
| **Reset artifact** | Auto-reset teleports fired no bus event, and MotorEPM's leg-phase/EMA survived fall+respawn → **any coherence/TLE trend across a reset was fake**. Fixed by publishing `events.reset` + reset-masking (Gate 0) |
| **★ THE CORRIDOR GYM RUNS OUT AT ~9.5 m, AND A LONG RUN CHARGES THE FASTEST ARM A `fall` FOR IT** (2026-07-27) | `_build_corridor()` lays a **9.5 m** curriculum on a **20×20** floor (`picrawler_body.gd:2945`, `:2665`), so the walkable strip ends near z=9.5 and the world ends at z=10. At **12 000** ticks a fast arm reaches it: in the load-stroke gain-2 sweep, seed 1 posted **the best distance of the entire campaign (net_z 10.04, max_z 10.06) while its mean `chassis_y` was −39.29** — it walked off the floor and kept falling, and `falls` and `chassis_y` both recorded that as a gait failure. **The bias runs the wrong way: it hits the FASTEST arm first, i.e. exactly the arm a propulsion lever exists to demonstrate, so a long run systematically penalizes levers that work.** Re-run clean at 6 000 ticks, the same arm shows 0 falls and no distance gain at all — so the 12 000-tick reading would have recorded BOTH a false positive (net_z +10 %) and a false negative (0.5 falls) on the same lever. This is §3.2 rule 7 (silent confound) and also explains why the standard protocol is 6 000 ticks — that is the run length that fits the gym, not an arbitrary choice. **Fixed in the harness:** `seedavg.py` now prints a loud `GYM-BOUNDARY WARNING` naming every seed whose `max_z` passes `SEEDAVG_SAFE_Z` (default 8.5) and states that `falls`/`chassis_y` are untrustworthy for those seeds |
| **Swing detector inferred contact from height instead of using the contact sensor that already existed** (2026-07-25; magnitude CORRECTED below) | `bool sw = foot_y_[i] > foot_y_ema_[i]` — no deadband. **Measured against the TRUE physics foot-contact sensor on identical runs: the detector reads 0.408 swing vs a true 0.229 — over-reporting by ~1.8×.** ⚠️ This row first claimed "40.3 % vs 0.7 %, wrong by ~50×"; that 0.7 % came from a **world-height** proxy which itself under-reports swing by ~16×, so the 50× figure was an artifact of comparing one bad proxy against another. **The real fix was not a deadband at all: `reality.proprio.foot_contact` (a physics touch flag, and the sensor a real picrawler has) was already published every tick and simply never wired into MotorEPM** — now available as `contact_topic`. Historical detail retained: | It then closes a feedback loop with any consumer that moves the foot (`stance_lift`, Cruse): bias lifts the foot above its EMA → declared swing → bias removed → foot drops → declared stance → bias returns. Cost scales with the consumer's gain, which is the measured `stance_lift` sweep: steps 50→84→**147** across gain 0→0.5→0.8 with **no** speed gain, tilt_sd 0.066→0.078→**0.178**, falls 0→0→**0.75**. Mitigated behind `swing_hyst_frac` (MAD-scaled deadband, default 0 = legacy; guard verified byte-identical to the per-seed digit). At gain 0.8 a 0.5 band halves the parasitic lifts (147→73), halves the wobble (0.178→0.089) and removes the falls. **⚠️ CORRECTED after unit testing — this is CONDITIONAL, and the condition is the absence of stepping.** Given a *real* duty cycle (80 planted + 20 lift) the legacy detector is essentially correct (0.18 measured vs 0.20 true): the lift excursions pull the EMA up, so the stance phase sits decisively below it and there is nothing to chatter on. And a band of 1.0·MAD in that regime is too wide — the stance deviation never clears −band, so the detector **latches** (0.58), which is the failure mode behind the live `frac=2.0` arm degrading after 0.5 helped. **So the detector is an AMPLIFIER of the no-stepping problem (§5), not an independent root cause.** Two shapes to remember: **a self-referential threshold is not a sensor, and any bias that moves what it measures will ring it** — and **a scale-invariant deadband cannot separate jitter from a step by amplitude** (a sinusoid has peak/MAD ≈ 1.57 at any size), so answering "is this foot loaded" needs a load observation, which the bus does not have |

---

### ★ 2026-07-26 — THE POWER STROKE IS NOT PHASE-LOCKED TO GROUND CONTACT

**This is the measured cause of the pinned-flat-speed entry in §5, and it re-frames the
whole inter-leg coordination family.** It overturns no prior verdict; it supplies the *why*
that eight nulls were missing.

**Instrument.** `gait_align_diag` in MotorEPM (diagnostic only — the block is skipped
entirely at its default 0). Config `..._embed_corridor_alignprobe.json` subscribes the true
`foot_contact` flag with the new `contact_instrument_only=1`, so ground truth is READ
without being wired to the stance gate (that swap is separately refuted, §2), plus
`joint_torque`. Collector `scripts_tools/gaitalign.py`.

**Gain-0 guard verified by measurement, not argument:** the probe arm and the deployed
`..._imufused.json` produce identical `seedavg` output on all 17 metrics, per seed, at both
6 000 and 12 000 ticks.

#### The result (n=4, corridor, diff 0.3)

| | 12 000 ticks | 6 000 ticks |
|---|---|---|
| `pos_stance` — frac of STANCE in the stroke's positive half | **0.509 ± 0.010** | **0.512 ± 0.014** |
| `pos_swing` — same over SWING | **0.505 ± 0.010** | **0.513 ± 0.010** |
| `td_plv` — stroke phase-lock at true touchdown | 0.228 ± 0.022 | 0.200 ± 0.003 |
| `contact_duty` — true stance duty | 0.752 ± 0.027 | 0.800 ± 0.010 |

**The push direction is statistically independent of whether the foot is on the ground.**
Stance and swing split the stroke waveform identically, to ±0.01 on every seed. Half the
power stroke is spent in the air; half the return swing scrubs while planted.

#### Why: three clocks, none locked to each other

| clock | period (ticks) | what it drives |
|---|---|---|
| hip1 — the stride | 32.3 ± 1.4 / 30.7 ± 3.1 | — |
| **knee — `L.phase`** | **23.6 ± 2.7 / 22.2 ± 2.2** | **the power stroke**, `y[0] += amp·sin(L.phase + stroke_phase)` |
| **contact — the real step** | **29.9 ± 3.3 / 26.0 ± 0.6** | what the leg actually does |
| foot-height — the incumbent detector | 15.3 ± 4.5 / 12.6 ± 1.2 | `stance_lift` + every Cruse rule |

`phase_joint` defaults to −1 = the knee, so the stroke is timed by a ~22–24 tick clock while
the leg steps every ~26–30. **They beat with a period of ~2–2.5 s**, which is the operator's
report — *"occasionally the three planted legs are in a good position and the fourth steps
forward and moves the body, but this synchronization is often lost"* — as a number.

Separately: the foot-height detector runs at roughly **half** the true contact period, i.e.
it fires about twice per real step. That is the relaxation oscillator documented at
`MotorEPM.hpp:314-328`, measured directly for the first time. It is chatter, not stepping.

#### Consequence for the coordination family

All eight refuted timing levers adjusted phase **between** legs while the thrust↔support
relation **within** a leg was uncorrelated. Re-phasing legs whose own thrust is random with
respect to their own footfall cannot buy anything — and did not. **Re-use context for the
whole family: do not retry an inter-leg phasing lever until thrust and support are locked
within a leg.**

#### Two follow-on questions, answered by the same run

- **Load is real, and hip1 is the signal — which overturns the obvious guess.** Stance/swing
  torque ratio: **hip1 1.368 ± 0.053**, hip2 1.124 ± 0.018, **knee 1.011 ± 0.031 (nothing)**.
  hip2 and the knee hold a near-static posture in *both* phases; hip1's torque is the ground
  reaction to the sweep itself, measured on the very joint the stroke acts on.
- **A precision-gated coordination probe is NOT a tautology.** `explore_mult` = 0.87 (12 k) /
  0.73 (6 k), so progress→commit leaves the probe σ near full most of the time.

#### Also established

- **`gait_phase` has left the imposed trot entirely**, to a different place per seed —
  `[0,−2.13,2.43,−0.70]`, `[0,−2.66,1.87,−2.12]`, `[0,2.15,−3.07,−1.02]`,
  `[0,2.58,2.15,−2.35]` vs the configured `[0,π,π,0]`. The coordination target is a random
  walk, not an imposed topology.
- **The §4 "which Rule 3 is live" ambiguity resolves to NEITHER** for the deployed stack:
  `cruse_bias` is exactly 0.0000 *and* `CruseCoordinator` is not instantiated (the graph is 5
  modules: bridge, MotorEPM, CPG, KeyframeGait, BodyRhythmTracker).
- **`planted` = 3.69–3.79**, higher than the 3.30 previously recorded. The support constraint
  is over-satisfied, independently re-confirming that the phasing question is closed (§2).
- **Baseline figures are run-length-dependent and must be quoted with their tick count.**
  `..._imufused.json` scores net_z **4.75 ± 0.29 / straight 0.74** at **6 000** ticks (the
  standard protocol, reproduced exactly) and net_z **6.55 ± 0.56 / straight 0.61** at 12 000.
  §8's "a number outlives the body it was measured on" applies to run length too.

**New instruments, all default-off:** `gait_align_diag`, `contact_instrument_only`,
`torque_topic`, `stroke_load_gain`, `scripts_tools/gaitalign.py`, and the body-stdout fields
`td_plv`/`sd_plv`/`pos_stance`/`pos_swing`/`contact_duty`/`tq_agree`/`tq_sep_j`/
`per_hip1|knee|foot|con`/`explore_mult`/`sgate`/`sgate_spr`.

---

### ★ 2026-07-27 — THE ROBOT WALKS STRAIGHT-LEGGED, AND THE CORRIDOR WAS HIDING IT

Operator UI observation (arena gym): *"hip1 is appropriately doing the work to swing the leg
forward, but hip2 and knee stay fairly horizontal to the chassis."* Measured, and it is worse
than it looks.

#### The sprawl, quantified (arena, n=3, 1032 leg-frames, post-warmup)

| | measured | design rest pose |
|---|---|---|
| `hip2` angle | **−3.6° ± 4.4** (range −23…+6) | 0° = femur PARALLEL TO GROUND |
| `knee` angle | −49.3° ± 15.8 (range −98…+15) | −80° |
| **tibia off vertical** | **37.5° ± 15.3** (range 0…**101°**) | **10°** |
| **planted foot radius** | **170 mm** (range 136…179) | total leg reach **166 mm** |

**The femur never leaves neutral for an entire run**, so hip2 contributes nothing but a fixed
53.6 mm horizontal offset; the knee carries the whole gait alone and sweeps the shank to
**~4× the design rest angle**; and the feet plant **at the limb's full reach** — straight-legged,
maximum moment arm, minimum mechanical advantage. Corroborating waste: **`scrub` 0.100 against
`fwd_v` 0.050 — the body slides sideways twice as fast as it advances**, which is what sprawled
legs do (the lateral components largely cancel left-to-right and that work is thrown away).

#### ⚠️ THE CORRIDOR MASKS THIS FAMILY — measure heading effects in the ARENA

`_build_corridor()` places the curriculum *"inside self-centering 30° walls"*
(`picrawler_body.gd:2684`). That geometry **actively re-centers the body**, so a yaw excursion
is corrected by the wall before it reaches `straight` or `turns`. Every tool in
`scripts_tools/` hardcoded `OGMA_PICRAWLER_GYM="corridor"`, so this whole family was being
scored in the one gym where its target failure is suppressed. New `scripts_tools/arenaavg.py`
(open floor; `net_disp`/`straight` since `net_z` is meaningless off-axis; boundary guard;
`tib_off` and `foot_r` posture columns).

**The two gyms disagree materially, so a corridor verdict does not transfer.** Baseline
`steps` 53 → **25** and `step_bal` 0.44 → **0.07** in the arena: the tripod-skid is far more
severe on open ground. The load-gated stroke likewise reads differently (corridor: `step_bal`
0.44→0.54; arena: `tilt_sd` 0.088→**0.063** and `planted` 3.87→**3.99**, but `steps` 25→14).

#### The proposed MECHANISM was refuted while the proposed ACTION worked

The operator's hypothesis was that sweeping an extended limb dumps yaw into the chassis. A new
instrument (`|Δyaw rate|` split by support state, per limb) says **no**: `yawd_swing_excess` is
**negative in every arm** (−0.035 baseline) and grows *more* negative as lifting rises. The
chassis takes **less** yaw impulse while a foot is airborne than during full support — the
larger source appears to be the skid-steer heading controller pushing through four planted legs
that fight each other. *(A first version of this instrument used mean |yaw rate| and was blind:
intentional steering acts through planted feet and swamped the reaction torque. A reaction
torque is an impulse; differencing separates them.)*

**The likelier reading is foot CLEARANCE, not angular momentum:** the arena baseline takes 25
steps in 85 s with `step_bal` 0.07 — a limb that is not clearing the ground to complete a step.

#### ★ KINEMATIC CONFLICT — tibia-vertical and belly-up cannot both be won by femur angle

`hip2_tuck_target` sweep (arena, n=3) splits the two goals exactly as the CAD predicts, because
the femur angle sets ride height and shank angle in *opposite* directions:

| | base | −0.2 (femur UP) | +0.2 (femur DOWN) |
|---|---|---|---|
| tibia off vertical | 37.5° | **32.9°** | 43.7° |
| belly clearance | 0.0221 | **0.0134** ⚠️ | **0.0335** |
| foot radius (mm) | 170.3 | 170.8 | **167.9** |
| net_disp | 4.85 | **6.16 (+27 %)** | 4.56 |
| straight | 0.71 | **0.80** | 0.65 |
| falls | 0 | 0 | **0.33** ⚠️ |

Femur up plumbs the shank and buys **+27 % distance and +13 % straightness** — while dropping
belly clearance **39 %**, a regression on a promoted invariant. Femur down raises the belly
**52 %** and shortens the moment arm, while making the shank *more* oblique, slower, and adding
falls. **Neither is promotable, and the trade is kinematic, not a tuning miss.** Note also that
the +27 % arm did NOT shorten the moment arm (`foot_r` 170.3→170.8), so its gain is not the
gear ratio — it is shank verticality plus a lower CoG.

**Re-use context stands and is now sharper:** the original refutation was *"didn't crouch (weak
reflex)"* at `postural_gain=0.3`; at the promoted 0.7 the parameter now bites hard in both
directions. The failure is no longer authority, it is the conflict above.

#### ★★ `tibia_plumb_gain` — the operator's INVERSE-KINEMATICS framing, as an error

hip2 and the knee are a planar 2-link arm. With hip2 pinned at its horizontal rest the KNEE
ALONE must set both the foot's height and its fore-aft position, so the foot is forced along a
circular arc about the knee axis and the shank has to sweep through a large angle to translate
the foot at all. The reflex gives hip2 an objective — null the shank's deviation from vertical —
so the knee's gait drive TRANSLATES the foot instead of arcing it. Nothing about timing is
specified, so the rhythm stays emergent (§5.7).

`θ_tibia = 1.40·x[hip2] + x[knee] − 0.0292` rad, from the body's own kinematic constants
(`HIP2_LIMIT=1.40`, `KNEE_REST=−1.6`), validated against the CAD rest pose and cross-checked
controller-side against the raw joint angles (51.5° vs 52.5°).

**Distinct from the refuted "learned hip2"**, which merely LOOSENED hip2's postural spring and
hoped the HK controller would discover the coordination. An unconstrained joint is a wobble
dimension, not an IK solver; this one is given an objective.

| gain | net_disp | straight | tilt_sd | tib_off | bellyc | falls |
|---|---|---|---|---|---|---|
| base | 4.85 ± 0.50 | 0.71 ± 0.02 | 0.0877 | 37.5° | 0.0221 | 0 |
| −0.3 | 5.17 | 0.65 | 0.1228 | 50.8° | 0.0480 | 0.33 |
| −0.15 | 5.48 | 0.70 | 0.0834 | 43.6° | 0.0319 | 0.33 |
| **+0.15** | **6.38 ± 0.61 (+32 %)** | **0.82 ± 0.00** | **0.0689** | 34.3° | 0.0156 ⚠️ | **0** |
| +0.3 | 5.44 ± 1.09 | 0.75 | 0.266 ⚠️ | 33.9° | 0.0103 ⚠️ | 0 |

**+0.15 is the largest single effect measured this session: +32 % distance and `straight` 0.82
with a standard deviation of 0.00 across three seeds** — the variance-collapse signature the
promoted heading-hold produced. It beats the static `hip2_tuck_target` shift (6.16 / 0.80) while
moving the tibia *less* (34.3° vs 32.9°), i.e. the reflex buys more per degree than a mean
posture shift — consistent with correcting the shank *through the stride* rather than on
average. **Not promoted: belly clearance falls 0.0221 → 0.0156 (−29 %),** and belly-up is a
promoted invariant. `IN_FLIGHT` — needs the corridor, hump, recovery and inversion gates.

#### ★★ THE KINEMATIC DEAD END — no joint angle buys both a vertical shank and a high chassis

The escape route was to plumb with hip2 and pay the ride height back with the knee (the joint
`stance_lift` proved pushes the body up off PLANTED feet). It **closes, and informatively**:

| | base | hip2 −0.2 | hip2 −0.2 + knee_tuck 0.85 |
|---|---|---|---|
| belly clearance | 0.0221 | 0.0134 ⚠️ | **0.0215** ✓ restored |
| tibia off vertical | 37.5° | **32.9°** | **44.1°** ✗ worse than baseline |

**The knee buys height by FOLDING the shank, which is by definition the opposite of plumbing
it.** hip2 plumbs and drops the body; the knee raises the body and un-plumbs. Those are the same
2-link constraint seen twice, and the reason is geometric: the feet plant at a **170 mm** radius
against a **166 mm** total leg reach, so the shank *must* angle out simply to reach the ground.
At that foot placement the leg is not long enough for both.

**⇒ And the last escape — "shorter steps bring the feet closer in" — is REFUTED on kinematics.**
`stroke_gain` swept 1.65 → 1.4 → 1.2 → 0.9 (a 1.8× range) leaves **`foot_r` invariant at
170.3–171.2 mm and `tib_off` invariant at 37.4–38.2°.** The reason is structural:
**`HIP1_AXIS` is world +Z** (`picrawler_geometry.md`) — hip1 is a *yaw* joint that sweeps the
foot fore-aft along an arc at **CONSTANT RADIUS**. Stride length and foot radius are
kinematically independent on this body; only hip2 + knee set the radius, and every way they
reduce it increases shank obliquity (the `knee_tuck` column above: 170→164→162→145 mm bought
`tib_off` 37.5→44.1→48.8→**64.5°** and collapsed the gait to net_disp 1.33).

**So the "get the tibia vertical" family is closed on this geometry.** Verticality trades against
ride height (hip2), against feet-in (knee), and is untouchable by stride. It is not a tuning
failure — it is that the leg is not long enough for the foot placement the gait uses. Re-use
context: a body with a longer tibia or a shorter femur, or a lever that moves the foot placement
by some route other than these three.

#### ★★ BUT THE SWEEP FOUND A REAL LEVER: the body is OVER-STRIDING

The same runs answer a different question — the one §5 named and nobody had tried
(*"propulsion amplitude / stroke operating point (never swept)"*):

| `stroke_gain` | net_disp | straight | tilt_sd | bellyc | falls |
|---|---|---|---|---|---|
| **1.65 (deployed)** | 4.85 ± 0.50 | 0.71 ± 0.02 | 0.0877 | 0.0221 | 0 |
| 1.4 | 5.17 ± 0.43 | 0.75 ± 0.03 | **0.0703** | 0.0232 | 0 |
| **1.2** | **5.45 ± 0.51 (+12 %)** | **0.78 ± 0.03** | 0.0778 | 0.0228 | 0 |
| 0.9 | 4.56 ± 0.11 | **0.79 ± 0.01** | 0.0739 | 0.0223 | 0 |

**A SHORTER stroke walks FURTHER**, peaking near 1.2: +12 % distance, +10 % straightness, less
wobble, **belly clearance preserved**, 0 falls throughout. The deployed 1.65 is past the
optimum. This validates the operator's *conclusion* ("shorter steps move the robot faster")
while refuting the *mechanism* they proposed for it (feet closer in — see above): the gain is
that a long stroke wastes effort, not that it changes where the foot lands.

**GATES (all run).** Corridor cross-check n=4 vs the deployed baseline at `stroke_gain=1.2`:
net_z 4.75→4.70, straight 0.74→0.75, flat_v 0.05→0.05, belly 0.023→0.024, 0 falls — **a tie on
progress** — but `steps` 53→**64** and `step_bal` 0.44→**0.52**. (1.4 is worse in the corridor:
net_z 4.24.) Hump gate n=4: final_z 4.74→**4.67**, gain_z 2.13→2.03, 0 falls — **holds, with the
variance halved** (std 0.51→0.25).

**Verdict `PARTIAL`, not promoted.** No regression anywhere, belly preserved, hump intact, 0
falls in every arm — but the +12 % is arena-only and the corridor is a tie, so by §3.3 (*a real
capability is LOUD*) this is not loud enough to promote on metrics alone. **What appears in BOTH
gyms is more stepping and better leg participation.** Next step is operator UI observation (§3
rule 5) before any bake-in; the mechanism (over-striding) is worth pursuing further because it
is the first time this campaign has moved distance at all without paying in belly or falls.

#### Swing-phase leg fold (`swing_tuck_hip2` / `swing_tuck_knee`) — the KNEE half carries it

The mirror of the promoted `stance_lift` (which biases the knee of PLANTED legs), gated on
**true contact** rather than the foot-height detector — that detector fires ~2× per real step,
and a tuck on a chattering gate would retract the limb mid-stance, i.e. lift a loaded foot.
Wiring true contact as a swing gate is refuted (§2) but for a consumer that wanted gait *phase*;
this one wants "is the foot off the ground", which the re-use context names ("step-over foot
placement"). This is also `hip2_tuck_target` **with the gate it was missing** — doctrine §5:
*ask what state should have gated a failed bias before calling the idea dead.*

Sign probe (n=3, arena): **−hip2 LIFTS** the limb (airborne fraction 0.30–0.47 vs a 0.20
baseline), **+hip2 SUPPRESSES** lifting (0.11–0.17, `steps` collapse 25→4). Magnitude sweep:

| arm | net_disp | straight | tilt_sd | step_bal | steps | bellyc |
|---|---|---|---|---|---|---|
| base | 4.85 | 0.71 | 0.0877 | **0.07** | 25 | 0.0221 |
| h−0.1 / k+0.1 | **5.26** | **0.75** | 0.0713 | 0.28 | 26 | 0.0226 |
| h−0.2 / k+0.2 | 4.96 | 0.68 | 0.0710 | 0.47 | 35 | — |
| h−0.4 / k+0.4 | 4.20 | 0.65 | 0.0804 | 0.18 | 91 | 0.0237 |
| h−0.2 only | 4.68 | 0.71 | 0.0868 | 0.20 | 47 | 0.0221 |
| **k+0.2 only** | **5.08** | 0.72 | **0.0731** | **0.49** | 27 | **0.0229** |

**`step_bal` 0.07 → 0.49 (7×) on the KNEE HALF ALONE**, with distance +5 %, wobble −17 %, belly
preserved, 0 falls. Folding the shank during swing carries the effect; lifting the femur does
not.

**CORRIDOR CROSS-CHECK (n=4, 6000 ticks) — the headline does NOT transfer.** Against a matched
control (identical instruments, gain 0): net_z 4.75→4.80, straight 0.74→0.73, **step_bal
0.44→0.43**, belly 0.023→0.023, 0 falls, `steps` 53→35, and the one real move is **tilt_sd
0.068→0.059 (−13 %)**. A tie plus a modest wobble improvement. **Diagnosis: the arena's 7× gain
was HEADROOM, not transferable mechanism** — the corridor baseline already sits at `step_bal`
0.44 while the arena sits at 0.07. Verdict `PARTIAL`, scenario-scoped to open ground; the wobble
reduction is the part that appears in both gyms. Not promoted.

---

### ★★ 2026-08-02 — PLAYFUL MACHINE IMPORTS: `c_init` `WORKING` on activity, gravity scaffold `NULL`

Source analysis: [`playful_machine_source_analysis.md`](playful_machine_source_analysis.md).
All arms n=4, corridor, 6 000 ticks, diff 0.3. **Measured on the `pure_hk` tier**, per the
context finding below — the deployed base cannot express a homeokinetic lever.

**`c_init` (import I1)** — PM's Sox `cInit`: adds a positive own-joint position feedback to
`C(j,3j)` so the loop starts self-exciting instead of at a dead fixed point. **Added to**, not
replacing, the per-leg random init (that randomness is this module's inter-leg symmetry breaker).
Effective loop gain is `motor_gain · c_init`, so PM's cInit 0.7–1.2 ⇒ `c_init` ≈ **0.23–0.40**
at our `motor_gain=3.0`. 0 = off, byte-identical (verified by measurement + unit test).

| `c_init` on `pure_hk` | net_z | chassis_y | belly | steps | step_bal | tilt_sd | falls |
|---|---|---|---|---|---|---|---|
| 0 (control) | −0.03 | 0.026 | 0.005 | 9.75 | **0.00** | 0.064 | 0.25 |
| 0.25 | −0.07 | 0.038 | 0.036 | 20.0 | **0.32** | 0.567 | 2.00 |
| 0.5 | +0.10 | 0.033 | 0.017 | 18.0 | 0.19 | 0.270 | 0.25 |
| 1.0 | +0.11 | 0.038 | 0.030 | **32.8** | 0.22 | 0.395 | 1.00 |

**`WORKING` on activity, `NULL` on locomotion, `REGRESSION` on stability.** Every non-zero value
converts a folded, inert body into an active one — chassis up, belly clearance ×3–7, steps ×2–3,
leg participation from literally zero to 0.19–0.32 — and none produces forward progress
(net_z ≤ 0.11). Activity rises with `c_init`; the stability cost is non-monotonic (0.25 wobbliest,
0.5 calmest — higher `C` pushes `z` further into `tanh` where `g'` shrinks and gain self-limits).
**The mechanism is confirmed; activity is not locomotion.**

**`scaffold_gravity_scale` (import I6)** — PM runs *every* legged experiment at gravity −6 vs
−9.81 (snake −4), on rubber, with compliant passive distal joints. New body knob (export +
`OGMA_PICRAWLER_GRAVITY_SCALE`), 1.0 = off, byte-identical (deployed baseline reproduces
4.58 ± 0.27 seed-for-seed). PM equivalent 0.61.

**`NULL`.** At 0.61 g the un-excited `pure_hk` body collapses to **exactly the same chassis height
(0.026) and belly clearance (0.005)** as at 1.0 g; `turns` gets *worse* (±1.1–5.6). With
`c_init=0.25` it buys mildly less wobble (tilt_sd 0.567→0.387, falls 2.0→1.5) and no progress.
**Diagnosis: gravity was never the binding constraint on standing** — the servos have ~4×
headroom over the static hip2 load already. PM's gravity reduction buys *dynamic* margin, not
the ability to hold a pose. **Re-use context: retry as a dynamics scaffold (fast/unstable gaits),
not as an uprightness scaffold; and the honest test it points to instead is `pure_hk` + the
stance reflex** (`postural_gain`), since that is the one parameter separating a `pure_hk` body
that collapses from an all-learning-ablated deployed body that stands fine (chassis 0.058).

**`cmd_squash` (import I3)** — see the saturation entry below. `NULL` on the deployed base, and
that null is a *passed prediction*, not a failed lever.

**★ What the pure-HK campaign converged on: HK produces MOTION but not DIRECTED motion.** Across
every pure-HK arm (1.0 g and 0.61 g, with and without the stance reflex, `c_init` 0 → 1.0) the
legs move and the body does not travel — best net_z **+0.23** against a deployed 4.58. The thing
that converts leg motion into translation in the deployed config is the hip1 stroke with its
hand-specified per-leg `stroke_signs` `{+1,−1,+1,−1}`. **This is structural, not a tuning gap:
MotorEPM is four independent 3×3 per-leg controllers, so `C` has no cross-leg terms and cannot
represent an inter-leg phase relationship at all** — four uncoupled oscillators at arbitrary
relative phase sum to no thrust. PM's dog, hexapod and humanoid all use **one Sox across every
joint**, where inter-leg coordination lives in `C`'s off-diagonal blocks and is *learned*.
⇒ **Whole-body `C` is promoted to the leading candidate** (the only untested import that could
supply direction as a learned quantity rather than as `stroke_signs`); **colored sensor noise** is
second (cheap, unbuilt, and PM's loop amplifies correlated sensory noise into motion while ours
has none).

**Also established, and it is a BODY fact:** `pure_hk` + `postural_gain=0.7` + `knee_tuck_target=0.7`
— identical to the deployed values — holds the robot at chassis **0.029** taking **0.5 steps per
6 000 ticks**, where the deployed config stands at **0.058**. The difference is entirely
`height_homeo_gain` + `stance_lift_gain`. **The picrawler's standing posture is actively
constructed by the height homeostat, not held by postural tone** — which connects to the
kinematic dead-end entry (feet plant at 170 mm against 166 mm total leg reach). PM's robots have
no analogue: their servos' neutral pose *is* a standing pose. That is a large part of why this
project needed a scaffold stack and theirs did not.

---

### ★★★ 2026-08-02 — THE DEPLOYED GAIT SURVIVES A TOTAL ABLATION OF THE LEARNED CONTROLLER

**Context:** a deep dive on the Playful Machine sources (Der/Martius, `~/Documents/PlayfulMachine`)
prompted three cheap Phase-0 measurements on our own stack. Full analysis, source diff and import
list: [`playful_machine_source_analysis.md`](playful_machine_source_analysis.md).
**Protocol:** corridor, n=4 fixed seeds, 6 000 ticks, diff 0.3; every arm one `mkarm.py` config
off the deployed `..._imufused__steplock_off.json` base with the diff printed.

| arm | net_z | straight | steps | step_bal | tilt_sd | planted | falls | verdict |
|---|---|---|---|---|---|---|---|---|
| **baseline** (all learning on) | 4.58 ± 0.27 | 0.73 | 50.0 | **0.48** | 0.065 | 3.69 | 0 | `BASELINE` |
| `ctrl_lr=0` (HK gradient off) | 3.78 ± 0.92 | 0.68 | 57.3 | 0.26 | 0.073 | 3.63 | 0 | see caveat |
| `embed_lr=0` (CPG-embedding off) | 4.23 ± 0.50 | 0.71 | 41.8 | 0.29 | 0.062 | 3.60 | 0 | — |
| **ALL controller learning off** (`ctrl_lr`+`embed_lr`+`sat_lr`+`bias_lr` = 0) | **4.75 ± 0.48** | 0.72 | 44.0 | 0.40 | 0.067 | 3.66 | 0 | **`NULL` — the ablation costs nothing** |

**Turning off every learned component in MotorEPM does not reduce distance, straightness,
stability, belly clearance or falls.** The learned layer buys `steps` 44→50 and `step_bal`
0.40→0.48 and nothing else. ⇒ **In the deployed config, locomotion is produced by the hand-built
scaffold stack** (stroke · Kuramoto-toward-a-specified-trot · postural · height · heading PD ·
stance-lift), not by the homeokinetic controller or the CPG-embedding.

**Verified, not assumed** (`CLAUDE.md` §3.2, all seven): `hk_share` 0.111 → **0.006** and `|C|`
mass reverts to the flat random-init profile (consumer fired); `motor_tle` **0.2495 in both arms**
— the forward MODEL still learns, only the controller was ablated (not dead code); baseline
reproduces 4.58 ± 0.27 exactly; `mkarm` printed four real diffs (no tautology, no silent
confound); this IS the faithful version — the 3-param attempt was the weakened slice (below).

**Both partial ablations are worse than either extreme** (3.78 and 4.23 vs 4.58 on and 4.75 off).
The learned components mutually compensate and the scaffold gains were hand-tuned with them
running — the signature of a learned layer the surrounding controller was fitted **around**,
rather than one the gait is built **on**.

**Power (§3.3):** n=4 is a **signal, and this is a null**, which is the claim type that most needs
powering. **Not yet a finding — needs n≥20 with varied world seeds before it is settled.** It is
recorded now because it is congruent with three independent things already in this ledger:
`hk_share`=0.11, `step_cv` identical (0.94–1.06) in *every* arm ever measured, and the nine-lever
flat-`flat_v` record.

**What it does NOT say.** It is not a verdict on the Motor-EPM. The same run shows the HK gradient
learns something real and correct: `|C|` mass on the **velocity** columns is **0.445 with HK on
and 0.042 with it off** — homeokinesis learning a velocity-feedback law, exactly as it should.
It is a verdict on the **context**: a base whose scaffolds are loud enough that no homeokinetic
lever can be read. **⇒ Every mechanism lever from here is measured on the `pure_hk` tier first;
the deployed base stays the reference for "does this help the robot we have."**

> #### ⚠️ The first attempt at this ablation was INVALID, and the new instruments caught it
> `ctrl_lr=0`+`embed_lr=0`+`sat_lr=0` (without `bias_lr=0`) gave net_z **0.01**, `steps` **0**,
> `tilt_sd` **0.000** — a frozen statue. Not an ablation: an **integrator windup**. `h += bias_lr·μ`
> (`MotorEPM.cpp:2432`) is **not gated by `ctrl_lr`**, and `h -= sat_lr·z·tanh²(z)` is its only
> restoring term. Instruments named it on sight: clip duty **0.99 on every joint**, mean pre-clamp
> command **12.68** (baseline 1.40), `motor_tle` **0.0000** (a body pinned against its stops is
> trivially predictable). §3.2 rule 7 — the arm you think you ran is not the arm that ran.

---

### ★★ 2026-08-02 — `sat_lr` IS THE ONLY BRAKE ON THE BIAS INTEGRATOR (`REGRESSION`, mechanism proven)

Single parameter, `sat_lr` 0.02 → 0, deployed base, n=4 corridor:

| | baseline | `sat_lr=0` |
|---|---|---|
| net_z | 4.58 ± 0.27 | **0.12 ± 0.41** |
| steps | 50.0 | **20.5 ± 35.5** — `[82, 0, 0, 0]`, three seeds **zero** |
| tilt_sd | 0.065 | **0.380 ± 0.426** (one seed 1.117 — convulsed) |
| mean pre-clamp \|cmd\| hip1 | 1.40 | **14.27 ± 11.16** — seeds 4.2 / 9.3 / 13.6 / **30.0** |

Unbounded, still-growing, seed-divergent magnitude = an integrator with no brake. **The mechanism
is proven by the pairing with the entry above: with `bias_lr` ALSO zero, `sat_lr=0` is completely
harmless** (net_z 4.75, the best arm measured). So the failure is **bias windup**, not saturation.

**`sat_lr` is documented in-source as "anti-saturation — surrogate for the dropped ∂G term."
Nothing recorded that it is load-bearing for stability.** ⚠️ **Live trap:** the obvious
improvement — replacing `sat_lr` with the principled confinement term from PM's `sos_avggrad.cpp`
— removes this brake. Any such change **must** supply an explicit bound on `h`; PM's Sox carries a
separate `damping` parameter (dog 0.0001, humanoid 0.0001–0.0003) that we have no analogue of,
and that is very likely what it is for. **Third unbounded-integrator failure in this campaign**
(cf. 2026-07-26) — it is this codebase's characteristic failure shape.

---

### ★★ 2026-08-02 — THE STRIDE JOINT IS SATURATED 56 % OF THE TIME (new instrument)

New Phase-0 instruments, report-only, in **both** `snapshot_state()`'s `mod` dict and
`diag_snapshot()`: `clip_duty` / `clip_duty_j` (fraction of post-warmup leg-ticks the assembled
command exceeded ±1), `pre_mag_j` / `pre_max_j`, `hk_share`, `echo_a_gain`, `c_mass_{pos,act,del}`.

| deployed baseline | clip duty | mean pre-clamp \|cmd\| |
|---|---|---|
| **hip1 (stride)** | **0.559 ± 0.044** | **1.404** |
| knee | 0.179 | 0.592 |
| hip2 | 0.010 | 0.091 |

`MotorEPM.cpp` computes `y = motor_gain·tanh(z)` with `motor_gain=3.0`, then adds ~7 further
terms, then applies the **only** clamp 400 lines later. **The nonlinearity the body applies is a
hard discontinuity; the HK loop-Jacobian `G = diag(1−tanh²)` assumes a smooth one**, so `L`
overstates loop gain wherever the command is railed. `mean|1.65·sin|` = 1.05, so the power stroke
alone is past the rail 42 % of every cycle: **`stroke_gain` above ~1.0 is a duty-cycle control,
not an amplitude control**, and the heading PD (gain 7.0) has full authority on one side of the
stroke and none on the other.

**Retrodiction, tested the same day — directionally right, magnitude insufficient.**
`stroke_gain` 1.65→1.2 (the ledger's one uncosted win) drops hip1 mean request 1.404 → **1.038**
and clip duty 0.559 → **0.476**, and buys **`steps` 50 → 63.8 (+27 %)** with `step_bal` 0.48 →
0.52 — but net_z is a tie (4.70 ± 0.58) and the joint is *still* railed 48 % of the time. **1.2 is
a less-saturated operating point, not a de-saturated one**, so "shorter stride" is not simply a
saturation result in disguise; the part it moved is step count.

**`cmd_squash`** (new lever: tanh squash of the assembled command instead of the hard clamp,
default 0 = byte-identical) on the deployed base: `NULL`/slight `REGRESSION` — net_z 4.58 → 4.17 ±
0.50, `planted` 3.69 → 3.54, `steps` 50 → 58.5 ± 16.0, tilt_sd tied. **Exactly what the ablation
entry predicts:** on a base where the learned controller contributes nothing, making the actuator
honest *for the learning rule* buys nothing and costs ~11 % of the scripted stroke's peak. **Re-use
context: it is a prerequisite for reading the HK Jacobian honestly and must be re-measured on the
`pure_hk` tier, where it is a mechanism question rather than a stroke-amplitude question.**

**Also measured — the echo channel (`[pos, action, delta]` state layout).** `echo_a_gain` =
**0.945**: the forward model HAS latched the channel that is a copy of its own command, spending
one of three output directions on it. But `|C|` mass on those columns is **0.159 against a
uniform 0.333** — the controller is *not* exploiting the free-win subspace. Real, wasteful of
model rank, **not the dominant failure**; fix when the state vector is next touched.

⚠️ **`hk_share` > 1.0 is not a bug.** It is `Σ|HK branch| / Σ|assembled command|`, and the additive
terms partially *cancel* the HK branch (the postural reflex pulls against it), so the denominator
can be the smaller number. Values above 1 mean **HK is being opposed**, not "HK is more than
everything". Seen at 1.07 on the windup arm and 1.20 on pure-HK + stance.

⚠️ **Instrument caveat, learned the hard way: `hk_share` is a BLIND metric for importance.**
It measures how loud a term is, not how much it matters — the scaffolds carry ~89 % of the
magnitude and (per the windup arm) produce zero locomotion without the right modulation. **Read
`hk_share` only alongside an ablation.**

---

### ★★★ 2026-07-27 — THE BODY HAS NO STEP PERIOD, AND THAT INVALIDATES THE PREMISE OF THE WHOLE TIMING FAMILY

**The single most consequential measurement of the session, and it was never taken before.**
New instrument `step_cv` — the cycle-to-cycle coefficient of variation of the TRUE
inter-touchdown interval, computed in the diagnostic block so it reports on the **deployed
baseline**, not only on lever arms.

| deployed baseline, corridor, 6000 ticks | |
|---|---|
| `step_cv` — all touchdowns | **0.984** |
| `step_cv_real` — micro-lifts (<4-tick swings) filtered out | **0.882** |
| real-step period | **61.9 ticks** (~1 s per leg) |
| `stance_bout` / `swing_bout` | 21.3 / **5.58** ticks |
| `short_bouts` — bouts under 4 ticks | **0.399** |
| `contact_duty` | 0.793 |

**A CV near 1.0 means the interval's standard deviation equals its mean — an essentially
memoryless process.** Filtering out the micro-lifts barely helps (0.98 → 0.88), so this is
not a sensor artifact and there is no rhythm hiding underneath. The picture: feet planted
~79 % of the time, ~80 % of lifts lasting only 1–3 ticks, and a genuine step roughly once
per second per leg at irregular intervals.

**⇒ There is no step phase to lock anything to.** Every timing lever in this ledger — nine of
them now — presupposes a rhythm the contact signal does not have. That is a far better
explanation for their uniform failure than any of the per-lever diagnoses recorded against
them, and it should be checked before another one is proposed.

**It also reframes the 2026-07-26 "three clocks beating at ~2.5 s" finding.** There are not
three clocks. There is one oscillator (`L.phase`, the knee) and an **aperiodic contact
process**; `td_plv ≈ 0.2` was reported as "the stroke is mistimed relative to the step" when
it also reads as "there is no step timing to be mistimed against."

**And it is the operator's stated goal, as a number.** The target has been "a good,
repetitive, efficient stepping gait" — `step_cv` measures exactly the *repetitive* part, and
it currently says the gait is not repetitive at all. **Drive `step_cv` down first; only then
does locking a stroke to it mean anything.** That is a concrete, cheap, and previously
unmeasured objective.

---

### ★★ 2026-07-27 — LOCKING THE STROKE TO THE STEP: the CLOCK NEVER ENTRAINED, so the lever is `DEFERRED`, not refuted

> #### ⚠️ READ THIS FIRST — the verdict below was written as `REGRESSION` and is CORRECTED
>
> The behavioural numbers are real and reproducible: driving the stroke from this step clock
> costs net_z **4.58 → 0.20** at the deployed offset, one parameter, n=4. **But the mechanism
> instruments — once their own plumbing was fixed — say the clock was never phase-locked to
> anything.** `td_plv` sits at **0.04–0.10** (near-uniform: touchdown lands at a random stroke
> phase) at every loop gain from 0.1 to 0.7, and the true entrainment error is ≈**1.55 rad**
> throughout. The clock RAN (`step_lock` 1.0, all four legs, only 4–15 lock/unlock flips per
> 3 000 ticks) — it just free-ran near the right frequency while a weak pull failed to close a
> persistent phase error.
>
> **So the stroke rode a phase that was neither the leg's own state nor locked to the step —
> worst of both worlds.** That is `CLAUDE.md` §3.2 rule 6 (*faithfulness: did you build the
> mechanism or a weakened slice of it?*), and it means **the lever as specified has not been
> tested.** `DEFERRED`, with the strong caveat that what WAS tested is decisively bad.
>
> **The tell was in the sweep and I misread it.** The src=1 row is FLAT across all 8 offsets
> (0.20–0.69) while the src=0 control row varies enormously (−0.13 → 4.62). **A sweep that is
> flat where the control's sweep is structured means the swept parameter has no effect** — an
> offset does nothing when the phase it offsets is not locked. I read that flatness as "no good
> region exists"; it was the signature of a mechanism that never engaged.
>
> **Two instrument failures made this hard to see, both mine, both now fixed:**
> 1. `update_gait_align_diag` hard-coded `L.phase`, so on a step-clock arm the whole alignment
>    diagnostic reported the LEGACY phase's alignment. *Not "did the consumer fire?" but "is my
>    verification instrument watching the thing I changed?"*
> 2. `step_lock`/`mv_stance`/`torque_agree_hip1` were added to `diag_snapshot()` only. The body's
>    stdout reads `get_module_metrics()`, fed from the `mod` dict in `snapshot_state()` — so they
>    read **0.0 in every headless run**, which is indistinguishable from "the mechanism never
>    fired." **Add a new instrument in BOTH places.**
> 3. `step_td_err` is sampled AFTER the phase pull, so it understates the true error by
>    (1 − gain) and *looked* like it improved monotonically with loop gain (1.39 → 0.47) when
>    dividing the corrector out shows a flat ≈1.55 rad. **An instrument that measures your own
>    corrector is not a measure of lock.** `td_plv` is the honest read.
>
> **What to do next, concretely:** make the clock actually entrain before judging the idea —
> the frequency estimate, not the phase pull, is the suspect (`step_period` reads 23.6 ticks
> against a true contact period of 26–28). Then re-run this sweep. Everything below stands as
> a record of what was measured; only the *verdict* changes.

**§5's headline lever, built and refuted.** The 2026-07-26 finding measured thrust and support
unlocked *within* a leg (`pos_stance` 0.512 vs `pos_swing` 0.513, three clocks beating at
~2.5 s) and named the fix: give the stroke a phase it does not itself drive. Built as
`stroke_phase_src` — a per-leg touchdown-referenced step clock, φ=0 AT touchdown, consumed at
the stroke site ONLY (coupling, amplitude homeostat and prop-credit keep the legacy
`L.phase`, so this is one lever on one consumer — the isolation `phase_joint=0` lacked).

#### The clean result: ONE parameter, 96 % of locomotion

| corridor, n=4, 6000 ticks, ψ = −2.85 (the deployed offset) | net_z | straight | falls | planted |
|---|---|---|---|---|
| **src=0** — legacy `L.phase` (= the deployed config) | **4.58 ± 0.27** | 0.73 | 0.00 | 3.69 |
| **src=1** — contact-referenced step clock | **0.20 ± 0.33** | 0.05 | 1.00 | 3.90 |

Identical in every other respect. The consumer demonstrably fired — it destroyed the gait.

#### It is not an offset miss: the FULL CIRCLE was swept, with a matched control row

The first sweep was **uninterpretable and is retracted**: each arm changed *two* parameters
(`stroke_phase_src` **and** `stroke_phase`, because −2.85 was tuned against the knee-derived
phase and does not transfer). Adding the src=0 control row at the same offsets is what made
the result readable, and it changed the reading completely.

| `stroke_phase` | src=0 (legacy `L.phase`) | src=1 (step clock) |
|---|---|---|
| 0 | 0.28 | 0.38 |
| π/2 | −0.13 | 0.33 |
| **π** | **4.37 ± 0.30** | 0.40 |
| **−π/2** | **4.62 ± 1.96** | 0.69 |
| **−2.85 (deployed)** | **4.58 ± 0.27** | **0.20** |

**Legacy has a BROAD good window** (π → −π/2, ~90° wide, containing −2.85); **the step clock
never exceeds 0.69 at any offset tested.** The control row is load-bearing: at ψ=0 and ψ=π/2
*both* sources collapse, so the cardinal-offset arms alone would have "refuted" a lever that
had not been tested.

#### Why: a hidden virtue in the thing being "fixed"

`planted` rises 3.58–3.69 → **3.85–3.91** in every step-clock arm while progress collapses:
the legs keep stepping (`steps` ≈ 48–54, as the baseline) and push, and the body does not
move. Best reading: **the unlocked stroke's virtue was COHERENCE ACROSS LEGS.** All four
strokes previously shared one reference — each leg's knee oscillation, mutually coupled by
Kuramoto on `L.phase`. Locking each leg's stroke to *its own* footfall makes the four thrusts
independent, and independent thrusts on a body whose `gait_phase` is a measured random walk
cancel. The within-leg misalignment was real; removing it cost a between-leg alignment nobody
had measured.

**`L.phase` is not a clock — it is a STATE OBSERVATION.** `atan2(joint velocity, joint
deviation)` is where the leg *is* in its own oscillation, re-derived every tick, so the stroke
riding it is a resonant drive reinforcing what the leg already does. Any external timebase,
however well locked to footfall, trades feedback for feedforward — *LEARNED cooperates,
IMPOSED fights* (CLAUDE.md §1).

#### A separate, generalizable lesson: HOW the phase is corrected, not just where it comes from

The first implementation SNAPPED `step_phase = 0` at touchdown. It did not merely
underperform — it convulsed the body (repeated inversion, `tilt_sd` 0.065 → 0.34). Two
compounding causes: **(1)** the stroke can *cause* touchdowns, so resetting phase *on*
touchdown is positive feedback — a push bounces the foot, the bounce re-triggers the reset,
the period estimate runs to its rail. Closing a loop "through the world" is only safe when the
stroke cannot trigger the phase-setting **event**. **(2)** `sin(φ + stroke_phase)` is a
*continuous motor command*, so snapping φ steps the command at every off-schedule footfall.

The repo already held the fix, in the module §6's re-use context had named: BodyRhythmTracker
does `φ += ω` with a **0.10 proportional pull** at each crossing. SynergyTimer (which this was
ported from) *does* snap — correctly, because its phase only INDEXES a discrete bin.

> **A phase that DRIVES a continuous command needs a soft pull; a phase that is only READ to
> index a discrete bin can take a reset.**

`step_phase_lock` = 0.10 default, **1.0 reproduces the snap** so that refutation stays
reproducible rather than becoming folklore. All numbers above are the *corrected* PLL form —
the snap form is worse everywhere.

#### Instruments added (all default-off, gain-0 guard verified byte-identical by measurement)

`stroke_phase_src` · `step_phase_lock` · `step_phase_debounce` · `step_period_alpha/_min/_max`
· `gait_raster_diag` (512-tick footfall ring → live Hildebrand plot in `xaq_inspector`, the
picture that makes stroke-vs-step visible while the robot walks) · `mv_stance`/`mv_swing`
(the stance/swing split on **achieved** hip1 motion — the non-tautological read, since a
touchdown-referenced phase satisfies `td_plv`/`pos_stance` *by construction*) ·
`step_td_err` · `torque_agree_hip1`. Unit tests 16/16 → **24/24**.

**Re-use context.** Retry when the phase is contact-referenced **AND shared across legs** — a
body-level step phase with per-leg offsets, i.e. `BodyRhythmTracker`'s `rhythm.body.gait`,
which §6 already named as *"built for exactly this"* and which this campaign ranked last. The
objection to it (its collective coordinate is built from hip1, the joint the stroke drives)
now looks much weaker than the inter-leg coherence it would preserve. Also still open: leave
the stroke on `L.phase` and instead **entrain `L.phase` itself** toward the contact rhythm
with a weak pull, so the leg *and every consumer of its phase* lock to footfall together —
giving the oscillator a prediction to fulfil rather than imposing a clock on one consumer
(§5.7).

---

### ⚠️ 2026-07-27 — THREE HARNESS/GYM DEFECTS THAT WERE SILENTLY CORRUPTING MEASUREMENTS

Found while running the above. Each is fixed, and each invalidated real numbers.

| Defect | What it did | Fix |
|---|---|---|
| **Corridor back wall was a VERTICAL seal** | a robot that turned round parked against it with no escape, *while still accumulating `fwd_v`* — a trapped body reading as "walking" (the blind-metric shape, §3 rule 4) | 30° self-centering ramp, as the side walls; containment verified by teleport at both ends |
| **Corridor +Z end simply DROPPED OFF** | fast arms walked off the world and were charged a `fall` (already recorded in §4) | 30° ramp; the far end is now contained rather than open |
| **★ `auto_reset` zeroes `step_in_episode`, so continuous-mode runs never reached `max_steps`** | an arm that keeps flipping **restarts its own countdown forever**. Measured on the snap-form p0 arm: seeds ended at ticks **13 407 and 72 043** against a 6 000-tick protocol, while the healthy baseline ended at exactly 6 000. Every COUNT (`falls`, `steps`) becomes counts-per-unequal-duration, so arms are comparable neither to the baseline nor to each other | terminate on the monotonic `tick_counter`; **byte-identical for any arm that never auto-resets**, verified on the baseline (all 17 metrics, `step_in_episode == tick == 6000`) |

**The third one's bias is the dangerous kind**: it costs wall-clock *in proportion to how badly
an arm fails*, so the worse a lever is the longer it takes to find out — which quietly
discourages running the sweeps that would refute things. It was caught only because it
corrupted a result of ours that we then checked.

**Also fixed: `mkarm.py` parsed its own `--allow-noop` FLAG as a `key=value`** and wrote a bogus
`"--allow-noop"` param into the arm config. `MotorEPM::apply_param` ignores unknown keys, so
that arm would have run silently — *a silent-confound generator inside the one tool whose job
is preventing silent confounds*. Caught only because `mkarm` prints its diff, which is what
that printing is for.

**⚠️ Every corridor number in this ledger predates the wall fix and is from a different gym.**
The deployed baseline re-measured in the fixed corridor (n=4, 6000): net_z **4.58 ± 0.27**
(was 4.75 ± 0.29), straight 0.73, flat_v 0.05, step_bal 0.48, tilt_sd 0.065, planted 3.69,
0 falls. The change is within ~0.6 σ because a 4.6 m walker never reaches either end wall —
the fix protects **fast** arms, i.e. exactly the ones a propulsion lever exists to produce.

---

## 5. Open frontier

- **★★★ THE MEASUREMENT CONTEXT IS THE BLOCKER (2026-08-02).** Ablating *every* learned component
  of MotorEPM costs the deployed gait nothing (4.75 ± 0.48 vs 4.58 ± 0.27, n=4). Locomotion here
  is the scaffold stack. **Consequence for everything below: a lever tested on the deployed base
  is being measured as a perturbation of a script.** That is the §3.1 lesson arriving from the
  other side — we have been generating verdicts in a context that cannot express the mechanism.
  **New protocol: every mechanism lever is measured on the `pure_hk` tier FIRST**
  (`motor_epm_pure_hk__inst.json`, instrumented); the deployed base remains the reference for
  "does this help the robot we have", not the place mechanisms are judged.
  **First powering job: take the ablation null to n≥20 with varied world seeds** — it is a null
  at n=4, which is the claim type that most needs power.
- **★★ NOTHING HAS EVER MOVED `step_cv`.** Across every arm measured to date — deployed baseline,
  HK-off, embed-off, full ablation, shorter stride, squash — `step_cv` sits between **0.94 and
  1.06**. Neither the learned controller nor the hand-built scaffolds affect footfall regularity.
  The operator's goal is untouched by the entire lever space tried so far, which argues the cause
  is structural (actuation, body, or the absence of any mechanism that *predicts* touchdown)
  rather than a gain anywhere in the current stack.
- **★ The Playful Machine import list** —
  [`playful_machine_source_analysis.md`](playful_machine_source_analysis.md) §5. Built and
  gain-0-guarded so far: `c_init` (self-exciting controller init, PM's Sox `cInit`; PM-equivalent
  ≈ 0.23–0.40 at our `motor_gain=3.0`) and `cmd_squash` (actuator honesty). Not yet built:
  colored **sensor** noise (PM puts `ColorUniformNoise(0.1)` on every sensor; our proprio is
  noiseless and our only noise is white and motor-side), the principled ∂G/`sense` confinement
  term (**must carry an `h` bound — see the `sat_lr` entry**), `Logarithmic` scale-free error,
  and `SERVO_KI` (PM's `ForceBoostWiring` booster 0.05 is on in every legged experiment; ours
  is 0).
- **Fast flat traversal, belly-up** — still the active thread. ~~bake `stance_lift=0.5` into
  the base~~ **DONE — `stance_lift_gain=0.5` + `feet_topic` are in the deployed
  `..._imufused.json`.** An alternative framing offered but never tested: express stance-lift
  as a **postural-target shift on stance legs** rather than a separate additive bias.
  **2026-07-27 status: distance CAN now be moved without paying in belly or falls** — a
  shorter stroke gives +12 % on open ground (below) — but `flat_v` specifically is still
  pinned at 0.05 in the corridor.
- **Genuine step-over obstacle negotiation** — deferred. Current hump clearance works by
  letting the belly ride low, which **may be a sim exploit** (frictionless belly drag); a
  real chassis could not do it. Obstacle nav is "good enough" for now.
- **★ FLAT SPEED IS PINNED ACROSS EVERY TIMING LEVER TRIED — ~~cause still unknown~~
  CAUSE MEASURED 2026-07-26, see the boxed finding below**
  (2026-07-25). Eight isolated levers were seed-averaged against the stance-lift base —
  stance-lift gain (3 values), swing-detector deadband (3), lateral-sequence phasing,
  stride-joint phase readout (4 offsets), adaptive coordination (2). **`flat_v` was
  0.03–0.04 in every arm that stayed upright and 0.00 in the ones that did not.** Nothing in
  the coordination, phasing, or phase-bookkeeping layer moved it. *That observation stands*
  — and a ninth lever (the load-gated stroke, §2) has since joined it at 0.04–0.05.

  > ### ⚠️ CORRECTION (same day) — the explanation first recorded here was WRONG
  >
  > This entry originally read "**the binding constraint is that the body does not step**",
  > citing "the feet are on the ground 99.3 % of the time". That number came from
  > `feet_y < stance_y_threshold` — a **world-height** test, which is neither
  > terrain-relative nor chassis-relative. Measured against the **true physics
  > foot-contact sensor** on identical runs, that proxy **under-reports swing by ~16×**:
  > true swing is **0.229**, the proxy says 0.014. **The robot IS stepping**, at roughly a
  > 77 % stance duty — an ordinary walking duty factor. The "shuffling / nothing swings"
  > story is retracted, along with the claim that this explains the old gait-symmetry era.
  >
  > The same correction shrinks the swing-detector bug in §4: the legacy detector reads
  > 0.408 against a true 0.229 — **over-reporting by ~1.8×, not the "50×" recorded.**
  > Still wrong, still worth replacing, but a different order of magnitude.
  >
  > **The methodological lesson is the sharper one, and it is on me:** I used an absolute
  > world-height threshold as *ground truth* while, in the very same change, arguing
  > correctly that an absolute world-height threshold is unsuitable for exactly this job.
  > A proxy cannot be the control for the proxy it is replacing. **Three different measures
  > of "is the foot down" disagreed by 16×, and the campaign ran without ever checking them
  > against each other.** The body publishes the real sensor (`reality.proprio.foot_contact`,
  > a physics touch flag) every tick and always did.

  **What is genuinely open:** why flat speed does not move. It is *not* explained by a
  missing duty cycle. Candidates now: propulsion amplitude/stroke operating point (never
  swept), ground friction / foot geometry, or the stroke and stance phases being
  mistimed relative to actual contact — the last of which is newly testable now that true
  contact is wired. **And note the sensor surface is far richer than anything yet consumed:
  `joint_torque` (a real per-joint load signal) is published and unused, and the IMU token
  carries no vertical acceleration and no pitch/roll rates at all.** See
  [`sensor_legitimacy_and_the_feet_y_oracle.md`](../plans-and-designs/sensor_legitimacy_and_the_feet_y_oracle.md).
- **~~OPERATOR DECISION PENDING — is `coord_reward_drive` allowed to stay?~~ RESOLVED
  2026-07-25 via option (c): the fitness was replaced, not the search.** `coord_fitness_mode=1`
  ranks probes by `coherence · activity / (1 + tle)` — no position, distance or velocity term
  — and ties-or-beats the reward it replaced on every metric while recovering better from
  perturbation (§1). **The generalizable result: the capability lived in the SEARCH, not in
  the reward.** Deleting the search cost net_z 3.52→2.36 and step_bal 0.26→0.03; deleting
  only the *reward* cost nothing. Worth trying the same substitution wherever a fitness has
  crept in. Remaining thread: `coord_reward_drive` still stores a winner, so the ratchet
  *shape* remains — it is now much harder to poison (a thrash scores badly on this fitness),
  but "stores a winner" is still the risky shape, and the inverted-stuck case below is
  still untested.
- **Does the ratchet lock in a destructive pattern after the robot gets stuck?** Partly
  answered. **Recovery gate (new tool `recoveravg.py`, n=3, 4 hump perturbations per run):
  no lock-in in either arm** — every seed kept advancing afterwards, 0 falls, and
  ratchet-ON actually recovered *more* (offsets returned 0.30 vs 0.11) and progressed more
  after the hits (+1.35 vs +0.76). This matches the operator's UI observation that the
  current config relearns to walk after getting stuck. **BUT the scenario tested is not the
  one that originally burned us:** the operator's failure was the robot stuck **on its
  back**, and in these runs the body never inverts (0 falls). So the inverted-stuck case is
  **UNTESTED**, not cleared. Note the likely reason it no longer bites:
  `auto_reset_on_inversion` defaults **true** (`picrawler_body.gd:490`), so an inverted robot
  is reset rather than left to thrash — meaning **the protection may be the auto-reset, not
  the fitness penalties.** If so it is a fragile dependency: disabling auto-reset could bring
  the old failure straight back. Worth testing directly before trusting the ratchet.
- **★★ GAIT REGULARITY IS THE BLOCKING PREREQUISITE — `step_cv` = 0.98 on the deployed
  baseline (0.88 with micro-lifts filtered out).** The body has NO step period: real steps
  arrive about once per second per leg at near-memoryless intervals. Every timing lever here
  presupposes a rhythm the contact signal does not have, which is a better explanation for
  their uniform failure than any per-lever diagnosis recorded against them. **This is also
  the operator's goal as a number** ("repetitive stepping" = low `step_cv`) and it had never
  been measured. Drive it down before proposing another phase lever. See the boxed entry in §2.
- **★ LOCK THE STROKE TO THE STEP — BUILT 2026-07-27; the build is `DEFERRED`, the IDEA is
  still open.** Driving the stroke from a per-leg touchdown-referenced clock costs net_z
  **4.58 → 0.20** (one parameter, full circle vs a matched control row) — **but the clock was
  measured never to entrain** (`td_plv` 0.04–0.10 at every loop gain; true phase error ≈1.55
  rad), so that number refutes *this build*, not the lever. See the boxed entry in §2 for the
  full record and the three instrument failures that hid it.
  **Concrete next steps, in order:** (1) fix the FREQUENCY estimate so the clock actually
  entrains (`step_period` reads 23.6 against a true contact period of 26–28) and re-run the
  sweep — only then is the lever tested; (2) if it still fails, try a phase that is
  contact-referenced *and shared across legs* (`rhythm.body.gait` + `gait_phase` offsets),
  since a per-leg clock also removes the inter-leg coherence the four strokes had via
  Kuramoto on `L.phase` (`planted` rose 3.69 → 3.90 while progress collapsed — effort up,
  output down); (3) or leave the stroke on `L.phase` and **entrain `L.phase` itself** toward
  the contact rhythm, so every consumer locks together. The original entry follows.
- **★ LOCK THE STROKE TO THE STEP — the sharpest lever the record now points at.** The
  boxed finding above measured the stroke riding a ~23-tick knee-derived clock while the leg
  steps every ~29 ticks, beating at ~2.5 s. The fix is to derive the phase the stroke rides
  from a signal the stroke does **not** drive — which is exactly the re-use context already
  recorded in §6 for `phase_joint=0`, whose refutation was a *wiring* refutation (a
  self-excited oscillator, because the stroke drove the same hip1 it read its phase from)
  and whose premise measured **positive**: step-balance rose 0.30 → 0.41–0.58 and
  feet-planted rose. Candidate phase sources, none of them hip1: the per-leg **load cycle**
  from `joint_torque` (now subscribed and measured to carry the signal on hip1 at ratio
  1.368), or `BodyRhythmTracker`'s `rhythm.body.gait`, already live in the config and built
  for exactly this. Budget a full `stroke_phase` re-sweep — the previous attempt swept the
  whole circle and still collapsed, but from self-excitation, which these sources do not
  have. **Judge it on `td_plv` / `pos_stance` vs `pos_swing` directly** (the instruments now
  exist), not only on the behavioural metrics — a lever that raises phase-lock without
  raising speed is still the informative result.
- **A working closed-loop attitude controller** — the balance reflex is a redesign, not a
  knob. ~~Prerequisite for revisiting lateral-sequence walk phasing~~ — that phasing question
  is now closed on this base (§2), so this is no longer blocking it.
- **Deferred nav layers** — L1 nav loops, L2 EFE arbiter, L3 keystone, per the plan's build
  order. The nav layer is still the plan's declared *disqualifier* (`target_compass` is an
  oracle); nothing in this ledger addresses it yet.

---

## 6. Re-use contexts — when a refuted lever should be tried again

*A complete verdict names the conditions that would justify a retry. These are live
proposals, not dead entries.*

| Lever | Refuted in | Try again when |
|---|---|---|
| **Lateral-sequence walk phasing** | ~~only ever measured inside the rejected sequencer~~ — **DONE 2026-07-25: isolated on the `embed`+stance-lift base, n=4. `NULL`/slight regression** (see §2). The support-polygon prediction held (planted 3.30→3.63) and bought nothing | **Only if the body starts genuinely stepping.** The isolation showed the win was already spent: the gait sits at 3.3/4 feet planted, so there is no support-polygon headroom for a phasing change to recover. Retry if a future lever produces real swing phases (a duty factor well under 1), which is when a swing *schedule* starts to matter at all |
| **Phase readout on the stride joint** (`phase_joint=0`) | corridor, n=4 × full-circle `stroke_phase` sweep — locomotion collapsed at every offset because the stroke drives the same joint the phase is read from (self-excited oscillator) | ~~When the stroke is moved off hip1, or the phase is derived from a joint the stroke does not drive~~ — **DONE 2026-07-27 and it also failed** (`stroke_phase_src`, a per-leg contact-referenced step clock: net_z 4.58 → 0.20 at the deployed offset, refuted across the full circle against a matched control row). Both refutations now point the same way: **the problem is not which signal the phase comes from, it is that any per-leg external timebase destroys the coherence the four strokes shared.** Retry only with a phase that is contact-referenced **and shared across legs** (`rhythm.body.gait` + per-leg offsets), or by entraining `L.phase` itself rather than replacing it |
| **★ Stroke-to-step lock, per-leg** (`stroke_phase_src=1`, touchdown-referenced step clock driving the stroke) | corridor, n=4, 6000 ticks, deployed instrumented base — **full circle swept (8 offsets) with a matched src=0 control row** | **`DEFERRED`, not refuted — THE CLOCK NEVER ENTRAINED** (`td_plv` 0.04–0.10 at every loop gain 0.1–0.7; true phase error ≈1.55 rad throughout). What was measured is decisively bad (net_z 4.58 → 0.20) but it is a *weakened slice*: the stroke rode a phase that was neither the leg's own state nor locked to the step. Fix the FREQUENCY estimate (`step_period` 23.6 vs a true contact period of 26–28) so the clock entrains, then re-run. Retry when the phase is **shared across legs**, not per-leg: `rhythm.body.gait` + `gait_phase` offsets, which preserves the inter-leg coherence this lever destroyed. The *within-leg* premise (thrust ⊥ support, measured) remains true and unaddressed — it just cannot be fixed by retargeting one consumer's phase. **Also carries a lesson independent of the idea:** the first build SNAPPED the phase at touchdown and convulsed the body; a phase that DRIVES a continuous command needs a soft proportional pull (`step_phase_lock=0.10`, BodyRhythmTracker's form), while a phase that is only READ to index a discrete bin can take a reset (SynergyTimer's form). `step_phase_lock=1.0` reproduces the snap |
| **stuck→explore** | flat ground, where a stuck-detector has no content — *the wrong scenario for it* | A regime where the body genuinely gets stuck: terrain, corridor corners, obstacle contact. Its inverse twin (progress→commit) was promoted, so the family is not dead |
| **Phase-indexed velocity (`Cvel`)** | on the *asymmetric* tripod-skid gait, which it amplified into circling | The base gait becomes symmetric, or the pump is gated by heading error so it cannot amplify yaw asymmetry. Explicitly "not wrong in principle" |
| **Cruse / Walknet contact-load reflex** | flat **and** incline; out-of-phase with the emergent gait — **and now known to have been gating on a GOD'S-EYE foot-height signal, never on load** | **This verdict is weak and should be re-opened.** Every Cruse rule gates on `in_swing_`, derived from `feet_y` = **absolute world-Y** (see the §2 oracle box), via a detector that over-reports swing ~1.8× vs true contact. So "out of phase with the emergent gait" was substantially a measurement of the *detector*, not of Walknet — a §7 *weakened-slice* shape. **The deeper point: Walknet's rules are LOAD rules, and they have never once had a load signal.** ⚠️ Correction to an earlier version of this row, which claimed there is "no load observation on the bus" — **there is: `reality.proprio.joint_torque`** (servo current sensing, 12 floats, hip1/hip2/knee × 4) is published every tick and **nothing in MotorEPM has ever consumed it.** That is the honest retry: give the load rules a real load observation. Historical warning still applies — an earlier Walknet null rested on a 1-of-6-rule slice, so scope any future claim carefully |
| **Load-gated power stroke** (`stroke_load_gain`) | corridor, n=4 × a 5-point gain sweep, on the deployed base — `NULL` on progress; the gate fired monotonically and the gait did not care | **When the stroke's TIMING is fixed first.** A magnitude gate cannot repair a push/drag balance measured at 50/50 (see the boxed finding) — it removes thrust spent in the air but scales push and drag equally during stance. Retry it *after* a lever locks thrust to support, where a purchase gate becomes a refinement of a correct stroke rather than a patch on a random one. Keep the `step_bal` 0.44→0.54 result: the load share genuinely does even out which legs take steps, so it is a candidate ingredient rather than a dead idea. The infra (`torque_topic`, `leg_load()`) stays default-off and is the load observation any Walknet retry needs |
| **`swing_tuck` (swing-phase leg fold)** | arena n=3 = a real 7× `step_bal` gain on the KNEE half alone; **corridor n=4 = a TIE** (0.44→0.43) | **It is not refuted, it is SCENARIO-SCOPED.** The arena gain was HEADROOM — the corridor baseline already sits at `step_bal` 0.44 while open ground sits at 0.07. Retry it wherever leg participation is genuinely poor (open ground, terrain), and expect nothing where it is already good. The −13 % wobble is the part that appeared in BOTH gyms and is the piece worth chasing |
| **The whole "get the tibia vertical" family** (`hip2_tuck_target`, `tibia_plumb_gain`, knee compensation, shorter stride) | arena n=3, all four routes, each closing differently | **Closed on LIMB GEOMETRY, not control** — the feet plant at 170 mm against a 166 mm total leg reach, so the shank must angle out just to touch the ground. Retry on **a body with a longer tibia or a shorter femur**, or behind a lever that moves foot placement by some route other than hip2 / knee / stride. Note `tibia_plumb_gain=0.15` is the largest single effect ever measured here (+32 % distance, `straight` 0.82 ± 0.00) and is held back ONLY by belly clearance — if belly-up is ever solved by another mechanism, this is the first thing to re-run |
| **Shorter stride** (`stroke_gain` 1.2 vs the deployed 1.65) | arena +12 % / corridor tie, n=3–4; hump gate holds; belly preserved; 0 falls | **`PARTIAL` awaiting operator UI observation, not refuted.** It is the first lever in this campaign to move distance without paying in belly clearance or falls. If the UI reads it as purposeful rather than merely busier, sweep 1.1–1.3 at n≥6 and re-run the recovery + inversion gates before baking |
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
