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
| **Swing detector inferred contact from height instead of using the contact sensor that already existed** (2026-07-25; magnitude CORRECTED below) | `bool sw = foot_y_[i] > foot_y_ema_[i]` — no deadband. **Measured against the TRUE physics foot-contact sensor on identical runs: the detector reads 0.408 swing vs a true 0.229 — over-reporting by ~1.8×.** ⚠️ This row first claimed "40.3 % vs 0.7 %, wrong by ~50×"; that 0.7 % came from a **world-height** proxy which itself under-reports swing by ~16×, so the 50× figure was an artifact of comparing one bad proxy against another. **The real fix was not a deadband at all: `reality.proprio.foot_contact` (a physics touch flag, and the sensor a real picrawler has) was already published every tick and simply never wired into MotorEPM** — now available as `contact_topic`. Historical detail retained: | It then closes a feedback loop with any consumer that moves the foot (`stance_lift`, Cruse): bias lifts the foot above its EMA → declared swing → bias removed → foot drops → declared stance → bias returns. Cost scales with the consumer's gain, which is the measured `stance_lift` sweep: steps 50→84→**147** across gain 0→0.5→0.8 with **no** speed gain, tilt_sd 0.066→0.078→**0.178**, falls 0→0→**0.75**. Mitigated behind `swing_hyst_frac` (MAD-scaled deadband, default 0 = legacy; guard verified byte-identical to the per-seed digit). At gain 0.8 a 0.5 band halves the parasitic lifts (147→73), halves the wobble (0.178→0.089) and removes the falls. **⚠️ CORRECTED after unit testing — this is CONDITIONAL, and the condition is the absence of stepping.** Given a *real* duty cycle (80 planted + 20 lift) the legacy detector is essentially correct (0.18 measured vs 0.20 true): the lift excursions pull the EMA up, so the stance phase sits decisively below it and there is nothing to chatter on. And a band of 1.0·MAD in that regime is too wide — the stance deviation never clears −band, so the detector **latches** (0.58), which is the failure mode behind the live `frac=2.0` arm degrading after 0.5 helped. **So the detector is an AMPLIFIER of the no-stepping problem (§5), not an independent root cause.** Two shapes to remember: **a self-referential threshold is not a sensor, and any bias that moves what it measures will ring it** — and **a scale-invariant deadband cannot separate jitter from a step by amplitude** (a sinusoid has peak/MAD ≈ 1.57 at any size), so answering "is this foot loaded" needs a load observation, which the bus does not have |

---

## 5. Open frontier

- **Fast flat traversal, belly-up** — the active thread. `stance_lift=0.5` is the current
  best belly-up base; bake it (+ `feet_topic`) into `..._bearinghold.json` as the default.
  An alternative framing offered but not chosen: express stance-lift as a **postural-target
  shift on stance legs** rather than a separate additive bias.
- **Genuine step-over obstacle negotiation** — deferred. Current hump clearance works by
  letting the belly ride low, which **may be a sim exploit** (frictionless belly drag); a
  real chassis could not do it. Obstacle nav is "good enough" for now.
- **★ FLAT SPEED IS PINNED ACROSS EVERY TIMING LEVER TRIED — cause still unknown**
  (2026-07-25). Eight isolated levers were seed-averaged against the stance-lift base —
  stance-lift gain (3 values), swing-detector deadband (3), lateral-sequence phasing,
  stride-joint phase readout (4 offsets), adaptive coordination (2). **`flat_v` was
  0.03–0.04 in every arm that stayed upright and 0.00 in the ones that did not.** Nothing in
  the coordination, phasing, or phase-bookkeeping layer moved it. *That observation stands.*

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
| **Phase readout on the stride joint** (`phase_joint=0`) | corridor, n=4 × full-circle `stroke_phase` sweep — locomotion collapsed at every offset because the stroke drives the same joint the phase is read from (self-excited oscillator) | **When the stroke is moved off hip1, or the phase is derived from a joint the stroke does not drive** (e.g. a multi-joint collective coordinate, or `BodyRhythmTracker`'s already-working `rhythm.body.gait` phase — which is *built* for exactly this and is already live in the config). The premise survived its own refutation: step-balance rose 0.30→0.41–0.58, so locking coordination to the stride rhythm does even out the legs. **This is a wiring refutation, not an idea refutation** |
| **stuck→explore** | flat ground, where a stuck-detector has no content — *the wrong scenario for it* | A regime where the body genuinely gets stuck: terrain, corridor corners, obstacle contact. Its inverse twin (progress→commit) was promoted, so the family is not dead |
| **Phase-indexed velocity (`Cvel`)** | on the *asymmetric* tripod-skid gait, which it amplified into circling | The base gait becomes symmetric, or the pump is gated by heading error so it cannot amplify yaw asymmetry. Explicitly "not wrong in principle" |
| **Cruse / Walknet contact-load reflex** | flat **and** incline; out-of-phase with the emergent gait — **and now known to have been gating on a GOD'S-EYE foot-height signal, never on load** | **This verdict is weak and should be re-opened.** Every Cruse rule gates on `in_swing_`, derived from `feet_y` = **absolute world-Y** (see the §2 oracle box), via a detector that over-reports swing ~1.8× vs true contact. So "out of phase with the emergent gait" was substantially a measurement of the *detector*, not of Walknet — a §7 *weakened-slice* shape. **The deeper point: Walknet's rules are LOAD rules, and they have never once had a load signal.** ⚠️ Correction to an earlier version of this row, which claimed there is "no load observation on the bus" — **there is: `reality.proprio.joint_torque`** (servo current sensing, 12 floats, hip1/hip2/knee × 4) is published every tick and **nothing in MotorEPM has ever consumed it.** That is the honest retry: give the load rules a real load observation. Historical warning still applies — an earlier Walknet null rested on a 1-of-6-rule slice, so scope any future claim carefully |
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
