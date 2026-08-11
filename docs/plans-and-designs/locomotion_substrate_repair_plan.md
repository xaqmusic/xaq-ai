# Locomotion substrate repair campaign — formalized plan

## Context

The 2026-08-09 motor-system audit (`docs/reports/motor_system_audit_2026-08-09.md`,
operator-concurred) found: 11 live mechanisms out of 148 params; three configured-but-dead
params; five observer EPMs consumed by nothing; a per-leg phase (`L.phase`) that runs
retrograde 2/3 of ticks; three disagreeing stance/swing definitions; and a stance press
that fights the first 7–9 ticks of every lift. First-principles conclusion: **stepping is
an error-driven response, not a predicted cycle** — rough terrain supplies the error
stream, level ground starves it, and the shuffle is a rewarded fixed point.

Follow-up exploration pinned four facts that shape the ordering:
1. **The contact process is aperiodic, not a third clock** (`step_cv` 0.88–0.98,
   memoryless; "there is no step phase to lock anything to"). Entrainment work must
   therefore FOLLOW stepping-density restoration, not precede it.
2. **No filter can fix `L.phase`** — group delay drives the leg backward (the
   phase_vel/sym_smooth retraction, net_disp −57%). The clean design already exists in
   the repo: BodyRhythmTracker's integrator + soft-pull-at-crossings PLL
   (`BodyRhythmTracker.cpp:205–284`), which has no high-pass arm and no group delay.
3. **The step-clock's blocker may already be fixed in code**: its frequency bias was
   traced to the v1 confirm-N debounce; the current v3 CANDIDATE→CONFIRM→BACK-DATE
   debounce (`MotorEPMv2.cpp:1345–1510`) removes it and was never re-tested.
4. **Contact vs phase jobs must be separated, not swapped** (the contact-as-gate
   refutation: "the consumer wanted gait PHASE"); `contact_instrument_only=1` is the
   existing tool that splits subscription from gate.

Goal: a confident, continuous gait on level ground — measured as **walk fraction at
n≥20** (baseline 2–5/20) with `step_cv` falling from ~1.0 — before any higher-level
active-inference loop is built.

Operator decisions taken: stance release enters early, gated on UI review (dose per the
curve, likely 0.5). Observer EPMs are REMOVED from the canonical config until Phase 5
re-introduces what its predictor consumes.

## Campaign-wide protocol

- One lever at a time; every change gain-0-guarded and verified byte-identical at 0
  (same-seed net_z reproduction, as done for `stance_release_frac`).
- Primary metric: walkers/n at n=20 (walker = steps>30), plus the full seedavg metric
  set; `CHASSIS_COLLIDE=1` always.
- **GYM PROTOCOL (operator correction, 2026-08-10): gait-QUALITY levers are judged in
  the OPEN ARENA (`arenaavg.py`, diff 0) — the corridor's self-centering walls suppress
  exactly the heading/circling failures a gait lever can cause, so corridor
  `straight` is wall-assisted and not a gait property. The corridor (diff 0.3) is the
  OBSTACLE gate, already established, and keeps that role.** P0–P4 numbers before this
  date are corridor-scoped; arena confirmation is required before any promotion.
  (nav oracle: zeroed in v2base-derived configs, so the old arena nav confound is gone.)
- Operator UI observation gates every promotion (§3 rule 5).
- Ledger entry per verdict; the audit's dispositions table is the checklist.
- Existing tools, reused not rebuilt: `seedavg.py`, `mkarm.py`, `actsweep.py`,
  `flbrake.py`, `gaitalign.py`, the attribution trace.

## Phase 0 — Hygiene and instruments (no behavior change intended)

New canonical config descending from `..._stancehip2__supportepm.json`
(name suggestion: `the_picrawler_motor_epm_embed_corridor_v2base.json`):
1. Remove dead params: `balance_gain`, `tilt_topic`, `coord_stab_penalty`; set
   `nav_gain=0` (corridor-latent oracle; annotate). Remove "balance" from
   `scaffolds_active` metadata.
2. Surface the panic family explicitly (`distress_topic`, `panic_on/off/noise/push_*` at
   current header defaults) so the config states what runs.
3. Remove the 5 observer EPM modules (operator decision). Keep their configs available
   as instrument arms.
4. Wire `contact_topic=reality.proprio.foot_contact` + `contact_instrument_only=1`
   (ground truth for every later gate/instrument; stance gate untouched — the recorded
   re-use pattern).
5. Instruments: add `mod`-dict export to `BodyRhythmTracker::snapshot_state()` (its
   diagnostics currently read 0.0 in every headless run — the documented
   instrument-invisibility trap) and add a lock-quality instrument (residual phase error
   at crossings / PLV analogue). Add a `walkers` count line to `seedavg.py`.
6. Verify: n=20 on the new canonical vs the old config — expect statistical tie on all
   metrics (removals are dead code; EPM removal must be checked, not assumed — RNG
   stream alignment can shift trajectories; the tie is the verdict that matters).

## Phase 1 — Honest phase substrate (shadow-first; no consumer switched blind)

1. Build 2–3 candidate phases as **shadow instruments** (diag-only, zero authority):
   a. Per-leg PLL of the BRT form: integrator + frequency state + soft pull at the
      leg's own knee up-crossings with amplitude-proportional hysteresis (reuses the
      proven design; keeps per-leg autonomy — the step-clock lesson says the stroke must
      remain a state observation of its own leg).
   b. Shared body phase + per-leg offsets: `phi_body` (rhythm.body.gait) +
      `gait_phase[i]` (ledger's recorded option 2). Self-excitation risk (hip1 carrier)
      measured, not assumed.
   c. (cheap control) delay-compensated filtered readout: filtered phase advanced by
      `phase_freq·τ` (the retraction's own recorded re-use context).
2. Score shadows offline on existing + new traces: retro fraction (target ≪0.5 vs 0.666),
   monotonicity, and stroke-timing alignment — replay `sin(shadow − 2.85)` against
   flbrake's propulsive windows (per-seed traces already banked by actsweep).
3. Switch ONE consumer — the stroke — to the winning shadow behind a new param
   (`phase_src_v2`-style enum, 0 = legacy byte-identical). A/B n=20. Then, only on a win,
   migrate coupling / amp homeostat / coordination fitness one at a time.
   Gate per switch: walk fraction not reduced; retro ≪ 0.5; td-alignment improved.

## Phase 2 — Stance/swing repair and separation

1. Sweep `swing_hyst_frac` (built for exactly the detector's chatter/self-feedback,
   never swept; deadband in units of the foot's own MAD — self-scaling, doctrine-legal).
   Gate: short bouts (<4 ticks) fall from ~40%, chatter ~2×/step → ~1×, walk fraction
   not reduced.
2. Keep the detector as the PHASE-wanting gate (honoring the refutation); true contact
   (already wired instrument-only) becomes the input for contact-wanting consumers:
   the release's bout reset, later load rules, and Phase 5's touchdown ground truth.

## Phase 3 — Stepping density (the suppressor levers)

1. **Operator UI review of the stance release** (0.5 and 1.0 arms) — the standing gate.
   If the 1.0 wobble is deck-riding, build the knee-only shaped variant (release the
   knee press, keep the hip2 press) as the alternative arm.
2. Bake the approved dose into the canonical config; re-run the n=20 reference set.
3. `explore_floor` sweep (e.g. 0.05/0.1/0.2): commit may damp but never abolish
   exploration. Gate: byte-identical at 0; walk fraction not reduced; watch falls.
4. Phase gate (prerequisite for Phase 4): steps ≥ ~3× baseline and `step_cv`
   (micro-lift-filtered, via true contact) measurably below ~0.9 — i.e., contact is no
   longer memoryless. If density rises but CV does not fall, stop and diagnose before
   entrainment.

## Phase 4 — Close the stroke↔contact loop (only after the Phase 3 gate)

1. Re-test the step clock as built: `stroke_phase_src=1` with the v3 debounce (the
   frequency-bias fix was never re-run). Judge on `td_plv` (the honest instrument — not
   `step_td_err`, which measures its own corrector) and the full metric set.
2. If it still fails: the ledger's option 3 — entrain `L.phase` (or the Phase 1 winner)
   itself toward the contact rhythm with a weak soft pull, so every consumer locks
   together and inter-leg coherence is preserved (the diagnosed reason the per-leg lock
   failed: independent thrusts cancel).
   Gate: `td_plv` decisively off its 0.04–0.10 floor with walk fraction ≥ baseline.

## Phase 5 — The body-pose predictor (instrument-first)

1. One body-level EPM over conditioned whole-body state: 12-D joints + 4 contact,
   common-mode centred (§0 rule 2 — the crouch is a huge common mode), or stacked
   per-leg EPM latents (re-introducing exactly the modules Phase 0 removed, now with a
   consumer). Instrument-only: zero authority.
2. Promote-or-kill gate: its transition surprise must ANTICIPATE touchdowns better than
   the contact base rate (the support EPM's TLE was measured reactive — that is the
   bar). Measured against true contact, offline, before any consumer exists.
3. Only on a pass, choose ONE consumer by measurement: anticipatory unload (release
   before commanded reversal), phase entrainment toward predicted contact, or
   TLE-precision-weighted reflex gains. Each is its own lever, own A/B.

## Files to modify (by phase)

- P0: new canonical config in `godot_host/project/addons/ami_ogma/configs/`;
  `cpp_core/src/ogma/modules/BodyRhythmTracker.cpp` (+`snapshot_state` mod dict,
  lock instrument); `godot_host/project/scripts_tools/seedavg.py` (walkers line).
- P1: `cpp_core/src/ogma/modules/MotorEPMv2.cpp/.hpp` (shadow phases + `phase_src`
  switch; shadow diag exports mirrored into the body log via
  `godot_host/project/scripts/picrawler_body.gd` — remember the mod-dict/body-log
  double-export trap).
- P2–P4: config arms via `mkarm.py`; MotorEPMv2 only if the shaped release variant or
  the weak-pull entrainment is built.
- P5: new EPM config module(s); offline analysis scripts in `scripts_tools/`.

## Verification

Every phase ends with: gain-0 byte-identity check → n=20 corridor 0.3 + flat →
walkers/full-metric report → ledger entry → operator UI look for anything promoted.
Campaign success criteria: walk fraction ≥ ~15/20 on level ground with `step_cv` < 0.7
and no belly/falls regression — then, and only then, the higher-level active-inference
work reopens.

## Campaign log

**2026-08-09 — P0 COMPLETE.** The v2base canonical config ties the old config
**bit-exactly** at n=20 (every seed, every metric identical) — the five observer EPMs,
the dead params, nav zeroing, panic surfacing, and instrument-only contact wiring have
zero behavioral footprint, verified at the trajectory level, not assumed.
First full read of the new BRT lock instrument (n=20): **brt_plv 0.10 ± 0.05,
residual 1.48 ± 0.14 rad, period_est 42.8 ± 5.3 ticks** — the body-rhythm PLL is
UNLOCKED everywhere, including the two walker seeds (0.08 / 0.03), and its period sits
at the knee-harmonic timescale, not the ~70-tick stride. Consequences for P1: the
per-leg PLL (candidate a) is promoted to front-runner; the shared-phase option (b) must
first explain why the shared reference fails to lock even during walking; BRT's own
crossing detection (hysteresis on the hip1 diagonal coordinate) joins the suspect list.

**2026-08-09 — P1 in flight; two findings from the first scoring pass.**
(1) **Preliminary scorecard (truncated runs): monotonicity and body-coupling trade off
across every candidate.** The smooth oscillators (A retro 0.005 / B 0.001) know LESS
about contact and propulsion (td_plv 0.15/0.12, prop_R 0.09/0.08) than the retrograde
incumbent (0.26/0.33); the filtered readout C is the reverse (prop_R 0.50 — best; retro
0.82 — worst). "L.phase is a state observation, not a clock," now quantified. A fourth
design follows: PLL with a CONTINUOUS confidence-weighted phase detector (integrator for
monotonicity + per-tick pull toward the readout for coupling).
(2) **A latent process-abort found and fixed:** the coordination probe constructs
`normal_distribution(0, σ)` with σ = drive × explore_mult, and full commit drives
explore_mult to EXACTLY 0 (floor 0) → glibc++ assert → the whole process dies. Fired
stochastically whenever a probe boundary landed inside full commit. Fix: σ→0 proposes
the incumbent unchanged. ⚠ Corollary discovered en route: the brain trajectory DIFFERS
across logging cadences (same seed crashed at DIAG_INTERVAL=1, ran clean at 60) —
instrument env settings are part of a run's context; A/B arms must share identical
instrument settings, always.

**2026-08-09 — P1 VERDICT: the phase-substrate swap is REGIME-BLOCKED; deferred behind
P3.** Full clean scorecard (6/6 seeds, 12 000 ticks): incumbent retro 0.652 / td_plv
0.213 / prop_R 0.357; per-leg PLL 0.005 / **0.043** / **0.038**; shared+offsets 0.001 /
0.046 / 0.027; filtered readout 0.819 / 0.144 / **0.517**. The smooth oscillators are
essentially uncorrelated with the body; the incumbent's jitter carries ~5–7× more
contact information. The hybrid scan (integrator at A's rate + continuous pull toward
C, k ∈ [0.02, 0.5], reconstructed offline at zero sim cost) traces a clean frontier —
and **no k beats the incumbent on all three axes; td_plv is strictly worse everywhere**
(0.02–0.11): the raw readout's jitter is exactly what snaps it to contact events.
⇒ In an aperiodic-contact regime there is no better phase than the state observation —
"an honest phase substrate" requires a stepping body first. **P1's consumer switch is
DEFERRED; the shadows stay as zero-cost instruments; re-score after P3's density gate.
The campaign order becomes P2 → P3 → (re-run P1 scoring) → P4.** Verdict class:
NULL-in-context with a crisp re-use gate, not a refutation of the designs.

**2026-08-09 — THE CRASH FINDING (supersedes much of the above; ledger §4 ★★★★).**
Every pre-fix run today was killed by a hardened-libstdc++ assert at the moment
progress-commit saturated (σ=0 in the probe's `normal_distribution`), and `seedavg`
scored the corpses. The "shuffle attractor", the walk-fraction crisis, the release
conversion story, the criterion multi-axis refutation, and the "logging-context"
claim are all retracted (details in the ledger entry). **The stack walks: post-fix
v2base 6/6 screened seeds, net_z ~6.5, ~110 steps — the Aug-7 baselines reproduce.**
P0's tie holds (both arms crashed identically). P1's scoring ran post-fix on full
walking runs and STANDS — measured on walkers, its verdict is stronger than first
framed: the incumbent phase wins on body-coupling in the *walking* gait, so the
substrate question closes and **P4's entrainment question (td_plv still ≤0.21 while
walking) is the campaign's live thread, in direct service of the operator's step-rhythm
goal.** Tooling hardened: seedavg completion guard; rebuild-after-detour rule; and the
campaign's honest baseline (n=20, fixed build) is being measured now.

**2026-08-09 — CLEAN BASELINE SET + release verdict; the campaign's honest state.**
n=20 per arm, fixed build, zero crashes: **v2base walks 20/20** (net_z 6.43 ± 1.58,
steps 113, tilt_sd 0.099, falls 0.30) — the canonical baseline, protocol fully
recorded. **stance_release: NULL-to-REGRESSION** (steps flat, tilt 2.4×, falls +83 %);
stays 0; re-use context recorded (knee-only shape, or fix the LIFT timing instead).
**P2 deadband: NULL on its gate** (short_bouts 0.39 → 0.38 at every dose — the
detector's chatter is real crossings, not hysteresis noise); nothing promoted.
Campaign state: P0 ✓, P1 closed (incumbent phase stands), P2 NULL, P3's premise
dissolved with the crash artifact (stepping density was never the problem), **P4 is
the live thread — the body walks arrhythmically (`td_plv` ≤ 0.21, `step_cv_real`
unmeasured-by-default) and the operator's continuous-gait goal is now cleanly a
RHYTHM problem on a healthy, crash-free, fully-instrumented baseline.**

**2026-08-09 — P4 arm 1 (`stroke_phase_src=1` re-test, clean build, n=6): REGRESSION
again — and the two most useful facts of the campaign ride along.** Transport collapses
exactly as 2026-07-27 (net_z 6.49 → 0.23, straight 0.03, circling) with both old
excuses eliminated: the v3 debounce FIXED the frequency estimate (step_per_real 54.6
vs true ~55–59) and the clock runs locked — yet step_td_err holds at ~1.4 rad: the
stroke drives the foot it locks to, a self-referential loop with a stable lag
equilibrium no weak pull can close. **And `step_cv_real` 0.97 → 0.82 — the FIRST
lever ever to move step regularity** (the ledger's "nothing has ever moved step_cv"
falls), at the cost of transport. Direction right, topology wrong: per-leg-independent
locking destroys the inter-leg coherence that carries thrust. ⇒ Arm 2 (in build):
touchdown-consistency pull on a SHARED per-leg phase offset — all consumers rotate
together, Kuramoto coherence preserved, no imposed target (each leg pulls toward its
own running touchdown phase — self-consistency, not a reference).

**2026-08-09 — P4 arm 2 (`phase_td_pull` 0.1, n=6): REGRESSION — and the principle that
closes the phase-manipulation family.** net_z 6.49 → 0.91, straight 0.12, falls 1.67,
plv 0.14 → **0.07** (the coherence it was designed to preserve), step_cv_real 1.01 (no
rhythm gain). Per-leg offsets drift independently and rotate each stroke away from its
limb's true state — the same lesson as P1 and arm 1, now from the third direction:
**the phase is a STATE OBSERVATION and every consumer needs it raw; smoothing it (P1),
locking it (arm 1), or offsetting it (arm 2) all break stroke–limb alignment and cost
transport. Timing can only change by changing when the limb PHYSICALLY moves.**
Arm 1 remains the proof that rhythm CAN move (step_cv 0.82) when the stroke genuinely
reorganizes. ⇒ **Arm 3 (specced, not yet built): select rhythm instead of forcing it —
add a touchdown-consistency term to the mode-1 coordination fitness
(`coord_td_weight`: per-probe-window resultant of L.phase at raw touchdowns), so the
existing (1+1) search DISCOVERS gait_phase offsets that make contact regular. No phase
is touched; the body's own search does the entraining. Doctrine-native ("give it a
prediction to fulfill"), one param, existing machinery.**

**2026-08-10 — P4 arm 3 (`coord_td_weight` 0.1/0.3, n=6 each): NULL on rhythm, PARTIAL
(signal) on robustness — and the miss is diagnostic.** step_cv_real 0.97/0.98/0.98
across 0/0.1/0.3: the rhythm target is not reachable through touchdown-PHASE
consistency, because td_R measures consistency against the knee's own oscillation —
a gait can land every touchdown at one phase of an IRREGULAR clock. The fitness
selected exactly what it measured: at 0.1, net_z variance collapses (6.49 ± 1.70 →
6.93 ± **0.36**), straight 0.59 ± 0.03 (the circler seed gone), tilt 0.078, falls
0.17, with FEWER steps (110 → 92) at equal distance; 0.3 erodes it (non-monotonic —
n=6 caution). **Two forks from here: (a) confirm the 0.1 robustness at n=20 — variance
collapse across every transport metric is "confident gait" in seed terms and may merit
promotion on its own; (b) the direct form, arm 3b (`coord_cv_weight`): put the
INTERVAL CV itself in the fitness — egocentric, intrinsic, the operator's number
selected literally — or escalate rhythm to P5's anticipatory predictor.**

**2026-08-10 — P4 CLOSES.** Arm 3 n=20 confirmation: robustness holds, softened —
net_z 6.43 ± 1.58 → 6.64 ± **0.96**, straight ± 0.15 → ± 0.07, sub-5 seeds 3 → 1,
falls 0.25; step_cv/steps/plv flat. `WORKING` (signal) as a variance lever; bake
decision = operator's. Arm 3b (interval-CV selection, n=6): **NULL — step_cv_real
0.97 even when the fitness selects the operator's number literally.**
**P4's structural conclusion, four arms deep: forcing rhythm breaks stroke–limb
alignment (1, 2); selecting rhythm through the coordination search fails because
RHYTHM IS NOT IN THE SEARCH SPACE (3, 3b) — inter-leg offsets do not control interval
regularity. The irregularity lives below the coordination layer, in the physical
cycle itself. Step regularity requires a mechanism that changes WHEN the limb moves:
P5's anticipatory predictor is next, with arm 1's step_cv 0.82 standing as the
existence proof and this campaign's instruments (true-contact step_cv_real, td_plv,
completion guard) as its measurement frame.**

**2026-08-10 — THE ARENA PROTOCOL'S FIRST CATCH: ctd10 demoted.** Open floor, n=20:
net_disp 10.17 ± 1.15 (control) vs 9.51 ± 1.43 (ctd10), straight 0.70 ± 0.04 vs
0.67 ± 0.07 — the corridor variance collapse was WALL-ASSISTED; the td-consistency
selection found wall-exploiting coordinations. NULL-to-negative in the honest gym;
NOT baked. Two more firsts from the same runs: (1) the operator's right-circling UI
observation is EPISODIC, not systematic (turns ≈ 0 ± 0.23 both arms; single-seed
circlers in both directions — a UI run draws one seed); (2) `yawd_swing_excess`
fired for the first time and is **negative** in both arms (≈ −0.037): the body turns
MORE with all four feet planted than during swing — yaw is stance-injected
(skid-steer differentials on planted legs), overturning the swing-reaction-torque
hypothesis and redirecting any future anti-circling lever away from swing-phase
ideas. Arena baseline on record: **net_disp 10.2 ± 1.2, straight 0.70, falls 0.20 —
the healthiest n=20 this stack has ever posted.** P4 stays closed; P5 unchanged as
the next build, now to be judged arena-first.

**2026-08-10 — P5 GATE: FAIL (clean kill at the instrument stage).** Arena n=6, two
zero-authority body-pose EPMs (contract 0.7/0.3 + transition-heavy 0.3/0.7): antic
index **+0.022σ / +0.005σ**, predictive lift **1.06 / 1.02**, reactive ≈ 0. The
excuses are ruled out by the same run: vocabularies healthy (60–70 nodes, ~65 %
baked, no collapse — conditioning is NOT the diagnosis) and both tle mixes agree.
Pose-transition surprise does not lead touchdowns; the support-EPM's reactive
verdict generalizes. §3.3: sub-σ excavation is not a capability — killed, no
consumer built. **Re-use contexts, recorded: (a) re-audition the predictor if
stepping ever becomes regular (anticipating an aperiodic process is the hard case);
(b) the untried input framings — per-leg EPM latents stacked, joint-delta features,
or PER-LEG-conditioned surprise (a body-global scalar dilutes four legs' events);
(c) the anticipation target may want the distal joints' fine pre-touchdown motion,
below RBF-grid granularity.**
**CAMPAIGN POSITION after P0–P5: the substrate is healthy, honest, and fully
instrumented; every cheap rhythm lever is now measured and killed (force, select,
anticipate). The evidence points below the coordination layer — the HK controller's
own limit-cycle dynamics (the Playful-Machine import list: SERVO_KI/ForceBoost,
colored sensor noise, ctrl_lr scale) as the place step rhythm must come FROM, rather
than a mechanism added around the controller. That is a design conversation with
the operator, not another autonomous lever.**

## Explicitly out of scope

L1 nav / EFE arbiter layers; new reflex levers on the unrepaired substrate; ratchet-leak
redesign (documented, deferred — revisit before hardware); promotion of anything without
its UI review.


---

# PART II — Below the coordination layer (approved 2026-08-10)

# Below the coordination layer — P6 (ctrl_lr) → P7 (SERVO_KI) → P8 (mechanical advantage / hardware readiness)

## Context

The substrate-repair campaign (P0–P5, complete; full log in
`docs/plans-and-designs/locomotion_substrate_repair_plan.md`) measured and killed every
cheap rhythm lever: forcing phase breaks stroke–limb alignment, selecting rhythm fails
because it is not in the coordination search space, and pose-transition surprise does
not anticipate touchdowns. The stack walks (arena n=20: net_disp 10.2 ± 1.2, straight
0.70) but arrhythmically, and the evidence points below the coordination layer — the HK
controller's own limit-cycle dynamics and the actuation model.

Operator decisions (2026-08-10): **sequence = measure ctrl_lr first, then build
SERVO_KI**; SERVO_KI is framed as a **body-fidelity correction** (real servos have
integral behavior; `docs/servo_dynamics.md:82` specifies Ki 0.01–0.05, currently
omitted) — a healthy value becomes the default body, like `chassis_collides`.
**Added scope (operator): P8 — leg mechanical advantage.** The sprawl is wide and a
REAL picrawler integration is imminent: chassis height, tuck kinematics, and energy
efficiency are in scope, with hardware-readiness (no deck contact, honest effort
budgets) as the framing.

Key priors that shape both steps:
- ctrl_lr ladder (2026-08-02, pure-HK base, n=4): steps ×4–5 and step_bal ×2.6 at
  0.05–0.10, but activity **peaks at 14–20k ticks and decays by 40k**; raising it on
  the DEPLOYED base destabilizes; it trades inter-leg coordination for per-leg power.
  **No rhythm metric was ever recorded** (step_cv read a structural 0.000 that era).
- Authority is not the binding constraint (tq_sat 0.5%→3.8% at high drive — direction
  of force, not magnitude). If SERVO_KI matters it is via **load-dependent timing**
  (stance leg under load → error integrates → force ramps → later-but-harder release),
  i.e. a within-leg thrust↔support coupling — the ledger's standing prerequisite.
- Windup precedents: three no-leak ratchets + the freeplay drift-twitch limit cycle.
  Any integral must be leaky, clamped, and frozen inside the deadband.
- Pooled step_cv ≈ 1 can hide tight in-cluster rhythm (2026-08-05) — read windowed
  regularity, not the pooled number alone.

## P6 — ctrl_lr ladder re-run with rhythm instruments (measurement only)

1. Arms: mkarm `gait_align_diag=1` variants of the existing pure-HK ladder configs
   (`motor_epm_pure_hk__inst__stance__c025.json` = 0.01, `__lr005` = 0.05,
   `__lr10` = 0.10, `__lr020` = 0.20; ignore the lr05/lr20 name-duplicates). These
   configs already carry `contact_topic` + `contact_instrument_only=1`.
2. Tooling: extend `arenaavg.py` with the STEP-CLOCK / DETECTOR / BODY-RHYTHM parsing
   groups (mirror the seedavg additions) and the completion guard — the new gym
   protocol judges gait quality in the arena, and the pure-HK arms wander (turns ±12),
   which the corridor walls would mask.
3. Protocol: arena diff 0, `CHASSIS_COLLIDE=1`, **24k ticks** (≥20k stripped-controller
   horizon; brackets the 14–20k peak), n=6 per dose. Read early/mid/late windows
   (the peak-and-decay is part of the result), `step_cv_real`, td_plv, bout structure,
   plus the full metric set.
4. Decision: does any dose move windowed step regularity vs 0.01? Either answer sets
   P7's judgment frame; a positive also nominates the pure-HK dose P7 tests on.

## P7 — SERVO_KI (the body-fidelity build)

1. Implementation in `godot_host/project/scripts/picrawler_body.gd`:
   - Per-joint leaky integral of servo tracking error against `_eff_target_*` (the
     rate-limited target, `:950`), **frozen when |error| ≤ deadband** (drift-twitch
     precedent), leak τ ≈ 0.3–0.5 s, contribution clamped to ≤ ~0.5 × MAX_SERVO_TORQUE.
   - Attach on the **torque/impulse-cap path** (`_powered_torque()`, `:8594` — raises
     authority under sustained load), NOT the velocity command (speed-clipped, windup-prone).
   - Expose `@export var servo_ki = 0.0` + env `OGMA_PICRAWLER_SERVO_KI` (the
     chassis_collides pattern). Diag: per-joint integral magnitude + boost duty
     mirrored into the body log (consumer-fired check).
2. A/B: default-0 byte-identity first (body-side change touches every config — the
   guard run is mandatory); then doses {0.01, 0.03, 0.05} (the servo_dynamics band,
   = PM's 0.05 at the top), n=20 arena diff 0 primary + corridor 0.3 obstacle-gate
   spot-check, on BOTH tiers (v2base__ga and the best P6 pure-HK dose). Judged on
   windowed step regularity / td_plv first, full metric set always, belly + falls
   as the cost axes.
3. Fidelity promotion (operator-decided framing): if a dose is healthy, it becomes the
   body DEFAULT, recorded as a body correction in `docs/servo_dynamics.md` and the
   ledger; configs stay untouched (the body carries it).

## P8 — Leg mechanical advantage & hardware readiness (operator-added)

**The measured anchor.** Arena control posture: feet planted at ~170 mm against a
166 mm total leg reach (fully stretched sprawl), tibia 37.5° off vertical vs the 10°
design rest, scrub 0.10 vs fwd_v 0.05 (slides sideways 2× faster than it advances).
The largest single effect ever measured (`ik_plumb`, tibia_plumb 0.15: net_disp +32 %,
straight 0.82) is blocked ONLY by its belly cost — and the recorded plumb-vs-height
conflict ("hip2 plumbs and drops; the knee raises and un-plumbs") holds at the CURRENT
stance radius. `foot_r` is set by hip2+knee alone (hip1 is a yaw sweep at constant
radius), so **narrowing the sprawl is the un-refuted combination: feet inboard buys
height AND plumb from the same leg reach**, and plumber shanks attack the measured
recovery-shear brake (fl −0.48 shear ratio from the most-tilted tibia). Hardware
stake: `bellyc_min` runs ~1–2 mm even in healthy runs — deck-kissing grinds a real
chassis; sustained clearance is a hardware GATE, not a preference.

1. **Posture operating-point sweep (one lever: the stance radius).** Move the rest
   posture inboard via the existing gated carriers — `knee_tuck_target` (deployed 0.7)
   and a hip2 rest target — as a 2-D coarse grid (3×3, n=6 arena screens), with
   `tibia_plumb_gain` off first. Judged on: tib_off (the `ga_tib` instrument), chassis_y
   + bellyc/bellyc_min, scrub, net_disp/straight, falls, and per-leg shear via flbrake
   traces. The ungated `hip2_tuck_target` refutation stands — any hip2 rest change
   rides the stance-phase machinery or the postural-profile path, never a blind
   rest override.
2. **Re-audition tibia_plumb AT the narrowed radius** (its recorded re-use context):
   if the inboard posture holds height, the +32 % transport effect may return without
   the belly cost that blocked promotion. Dose from the ik_plumb arm (0.15), n=20
   arena on a win.
3. **Energy: activate the built-but-never-deployed CoT search** (`amp_seek_rate` —
   (1+1) hill-climb on amp_target maximizing fwd_v per oscillation amplitude), and
   promote effort metrics to first-class in every P8 report: tq_mag, tq_sat, scrub,
   and net_disp per unit amplitude (the CoT proxy). Hardware framing: the effort
   budget of a real servo is the binding resource.
4. **Hardware-readiness gate for the phase**: bellyc_min sustained > ~8 mm at n=20
   with falls ≤ baseline — the "would not grind" bar — alongside the standard set.

## Files

- P6: 4 mkarm'd `__ga` ladder arms; `godot_host/project/scripts_tools/arenaavg.py`
  (parsing groups + completion guard, mirroring seedavg).
- P7: `godot_host/project/scripts/picrawler_body.gd` (integral, env, diag mirror);
  dose arms via env (body param — no config changes needed); `docs/servo_dynamics.md`
  + ledger/campaign-log entries.
- P8: config arms via `mkarm.py` (knee_tuck_target / hip2 rest grid, tibia_plumb
  re-audition, amp_seek activation — all existing MotorEPMv2 params, no new C++);
  `flbrake.py` + attribution traces for the shear reads; ledger §2 re-use-context
  updates for `hip2_tuck_target` and `tibia_plumb`.

## PART II campaign log

**2026-08-10 — P6 COMPLETE: ctrl_lr 0.05 (PM dog) dominates the pure-HK tier.**
Arena diff 0, 24k, n=6, full instruments (first rhythm-instrumented ladder ever):
0.05 → steps ×2.7 (72→191), step_bal ×2 (0.21→0.43), net_disp ×1.8, short_bouts
0.49→0.41, swing bouts LENGTHEN 7.0→8.7 ticks, stride 58→42 ticks, falls FLAT (0.33)
— and `step_cv_real` 0.94→**0.89** (4/6 seeds ≤0.90): the second rhythm move in
campaign history, signal-grade (sub-σ pooled; the structural companions are the
stronger evidence). 0.10/0.20 buy falls ×3.5 and tilt ×1.8 for nothing extra —
**the 2026-08-02 "0.10 for power / 0.01 for coordination, no optimum" verdict is
REVISED: on full instruments 0.05 dominates both.** Windowed activity still decays
across quarters at every dose (the peak-and-decay stands) but stays alive at Q4 on
0.05. ⇒ P7 tests SERVO_KI on BOTH tiers: v2base__ga and pure-HK @ lr 0.05.

**2026-08-10 — P7 VERDICT: SERVO_KI is NULL on both tiers; the omission is
measured-benign.** With the boost on the verified force path (imp_* → _set_motor_vf;
the first build attached to telemetry-only `_powered_torque` and was bit-identical —
both lessons now inline in the body script), doses {10, 30, 60}%-at-0.1-rad, n=6 per
tier: stance/swing bouts UNMOVED (the load-dependent-timing prediction refuted),
step_cv sub-σ drift, transport tie at best (v2) and −30/−40 % at mid/high doses
(pure-HK tier). Confirms tq_sat's "authority was never the binding constraint" from
the actuation side with a live lever. **servo_ki stays 0; no fidelity promotion**
(the healthy-dose condition is not met — inert-to-mildly-negative); the real-servo
I-term omission is now a MEASURED non-issue for behavioral fidelity at this scale.
Re-use context: a body with weaker servos (hardware's real effort budget) or a
load-conditioned rather than error-integral form. ⇒ **P8 (mechanical advantage) is
the live phase.**

**2026-08-10 — P8 GRID SCREEN (n=6): THE SPRAWL WAS THE RHYTHM BLOCKER.** The
knee_tuck × stance_lift_hip2 grid's kt=0.85 band delivers **step_cv_real 0.95 →
0.80–0.83 — the largest rhythm move ever recorded** (beating arm 1's destructive
0.82) — WITH steps ×1.7 (69→117), transport HELD (10.47/10.06 vs 10.17), falls
better (0.17), chassis +7–16 %, bellyc_min ×3–5, feet inboard 170→165 mm. kt=1.0
overreaches (transport 6.3–7.7, tilt ×2). No clock, no fitness term, no phase
machinery — rhythm emerged from the GEOMETRY the limb moves through, exactly the
"change when the limb physically moves" requirement P4's five levers could not meet.
The operator's mechanical-advantage thread found it. n=20 confirmation of 0.85/0.25
and 0.85/0.40 in flight; then the corridor obstacle gate, the tibia_plumb
re-audition at the narrowed radius, and operator UI review.

**n=20 CONFIRMED (same day): step_cv_real 0.81 ± 0.07 both cells, falls QUARTERED
(0.20 → 0.05), steps ×1.8, transport statistically tied (9.79/9.42 vs 10.17, ns),
clearance up.** The 0.85/0.25 cell is a ONE-PARAMETER lever (knee_tuck_target
0.7 → 0.85 alone) — the promotion candidate. ⚠ Surprise that reshapes step 2:
`tib_off` ROSE (37.5° → 48°) — mechanical advantage came through moment arm and
chassis height, not shank plumbing, and the tuck un-plumbs further. The plumb
re-audition now runs against its own premise (screen for the record per the re-use
context). Pending gates: corridor obstacle spot-check, operator UI review.

**GATES (same day): corridor PASSES WITH A BONUS — net_z 7.37 ± 1.22 vs 6.43 ± 1.58
(+15 %), 6/6 walkers, falls better; the tucked posture climbs the hump BETTER than
the sprawl. Plumb re-audition: NULL at the narrowed radius — pulls the shank
vertical by DROPPING the body (chassis 0.048 → 0.045), costs rhythm (cv 0.81 →
0.85); the plumb-vs-height trade is inherent at both radii and its re-use context
is closed. ⇒ `knee_tuck_target 0.85` is fully gated pending ONLY operator UI review
(launcher: "★ P8 TUCK"). Remaining P8 item: the CoT/amp_seek energy pass.**

**2026-08-10 — SWING-ECONOMY THREAD (operator UI observation → measured → counter-lever
killed honestly).** swingreport.py confirmed all three observed signatures on the tuck
base: swing knee-throw AMPLIFIED by the tuck (−0.17 → −0.27 rad), return whip ×2–4,
front-leg reaction coherent in SIGNED yaw (fr r ≈ −0.15..−0.26 — invisible to the
magnitude-only yaw instrument), and the "lost power stroke" localized to EARLY-STANCE
loading (rear-leg first-5-tick forward impulse 0.032 → 0.003), not to missed plants
(6–13 %, HALVED by the tuck). The purpose-built counter (`swing_tuck_knee` 0.2 on the
tuck base): n=6 screen looked excellent (disp 10.23 ± 0.35, scrub −12 %) — **n=20
REFUTES it** (disp 8.81 ± 3.03, tilt 0.117, falls 0.20, cv 0.90 ± 0.15) **while the
traced mechanism check shows the fold DOES shrink throw/whip as designed. Mechanism
delivered, outcome worse: swing economy is not the binding constraint, and the ±0.35
screen was a favorable draw — n=6 variance collapse can be luck (§3.3, again).**
0.4 is catastrophic (disp 1.9, falls 1.3). swing_tuck stays 0; re-use: smaller dose
or hip2-half only with a NEW hypothesis for the seed-dependent destabilization.
**The tuck alone (knee_tuck_target 0.85) remains the promotion candidate at the
operator UI gate.** P8 remaining: the CoT/amp_seek energy pass.

**2026-08-10 — THE DIALED-IN TUCK (operator-prescribed): rhythm record 0.75, a real
transport trade.** swing_tuck_hip2 −0.2 + swing_tuck_knee 0.1 on the tuck base — the
operator's "hip2 should lift, knee should fold" — MECHANISM CONFIRMED in traces (lift
share inverted: hip2 −0.05 → −0.16..−0.22, rr femur out-lifts the knee; front-leg
momentum reaction collapsed fr −0.14 → −0.10, fl/rr ≈ 0; fl early braking 8× better).
n=20: **step_cv_real 0.75 ± 0.11 (campaign record, success bar is <0.7)**, steps 196,
falls 0.10 — but net_disp 8.34 ± 2.36 vs the tuck's 9.79 ± 1.48: **the −15 % transport
cost is real at power** (shorter, denser strides). OPERATOR DECISION: (a) bake tuck
alone (transport-first), (b) bake both (rhythm/safety-first — arguably the hardware
profile: higher cadence, less per-step torque, fewer falls), or (c) dose-search the
middle (h1k1 held transport at n=6 with one wobble seed — n=20 unmeasured).

**2026-08-10 — THE DESCENT PRESS: diagnosis confirmed, trade conserved, frontier
mapped.** swing_descend_gain (hip2 flips lift→press past half the leg's own swing
duration): transport recovers DOSE-MONOTONICALLY (8.34 → 9.23 → 9.81 at 0/0.2/0.4 —
the operator's late-plant diagnosis is causal), but rhythm pays back proportionally
(cv 0.75 → 0.85 → 0.89) and step density halves. **The posture family now spans a
mapped frontier — stance tuck / swing lift+fold / descent press are three orthogonal
knobs trading rhythm-density against stride length — and the plain TUCK still
Pareto-holds its point (9.79 / 0.81 / falls 0.05).** Operator picks the operating
point; any chosen point gets n=20 + traces + UI before promotion.

**2026-08-10 — PROMOTED: V3 BASE (TUCK+PAIR), all gates green.** Operator UI verdict
"confident and fast"; corridor gate n=20: **20/20 walkers, ZERO falls** (best safety
result of the campaign), cv 0.77 on rough terrain, net_z 5.90 (the known −8 % trade).
Baked as `the_picrawler_motor_epm_embed_corridor_v3base.json` (+ __ga instrument twin
— the new measurement control): knee_tuck_target 0.85, swing_tuck_hip2 −0.2,
swing_tuck_knee 0.1 on the v2 canonical. Remaining P8 study: the CoT/amp_seek energy
pass, screening on the new base.

**2026-08-10 — P8 CLOSES: the CoT screen, and the campaign's end state.** amp_seek
0.05 on V3 BASE: transport tie with variance nearly halved (8.32 ± 1.41 vs
8.34 ± 2.36), falls 0/6, rhythm held — signal-grade, parked (not powered: the energy
half of the CoT claim needs an effort column — tq_mag — that the arena harness does
not yet surface; recorded as an instrument gap beside the signed-yaw upgrade).
0.02 mildly regresses. **END STATE: V3 BASE promoted and canonical; the posture
frontier mapped; every lever verdicted with re-use contexts; the harness completion-
guarded, gym-honest, and context-aware. Open for the hardware road: the ~8 mm
clearance bar, effort instrumentation, ratchet leaks — then the higher-level
active-inference loops.**

## Verification

Every step: completion-guarded runs only; gain-0/default-0 byte-identity (same-seed
reproduction) before any dose; consumer-fired checks from the new diag mirrors; arena
primary per the gym protocol with a corridor obstacle-gate spot-check before any
promotion; ledger + campaign-log entry per verdict; launcher naming convention for any
exposed arm. Known failure shapes to watch: integral windup (leak/clamp/freeze guards),
the 14–20k peak-and-decay masquerading as a win at short horizons, and pooled-step_cv
hiding cluster rhythm (windowed reads mandatory).


---

# PART III — The Motor Planner: rolling masked action prediction (approved 2026-08-10)

## Context

Operator direction: achieve MPC/RL-class competence while keeping runtime plasticity —
the project's actual goal. Diagnosis: the input side (EPM coarse-graining/prediction)
is solid; **the output side lacks real prediction and planning**. Proposal (operator,
from E/I pathways + speech-denoising masking): **roll out sequences of future motor
messages and continuously mask/refine them as they approach the present.**

Field coordinates, recorded for orientation: this is a receding horizon (MPC's actual
power source) executed as continuous refinement rather than re-optimization; it is the
anatomy of a diffusion policy (coarse far-future, committed near-future); and the E/I
masking is basal-ganglia action selection — GO/NO-GO as selective disinhibition of
prepared motor predictions. Doctrine-native reading: **a motor rollout IS a prediction
of future proprioception; acting is fulfilling it; masking IS precision** (inhibition =
precision withdrawal). The bumblebee argument operationalized: not a bigger brain, a
structured one — the missing module is central-complex-shaped: a small structured
future buffer on the motor side.

What this answers on our books: the ablation null (the learned layer finally gets a job
reflexes cannot do — THE FUTURE; reflexes own now), and the P4 impasse (a plan can
carry contact-CONTINGENT structure — a stroke element masked until its predicted-contact
element precedes it — which phase machinery could never express).

## Non-negotiables (from the campaign's own scars)

- The plan NEVER becomes an imposed trajectory. Entry is exclusively through the
  objective/confidence socket (KeyframeGait's error-retarget pattern), with confidence
  EARNED from the planner's own predictive accuracy 1/(tle+ε). The plan proposes;
  reflexes dispose. ("Flopping fish" + P4 arms 1–2 are the graves this fence guards.)
- Instrument-first at every stage; shadow before authority; mechanism in traces before
  n=20; n=20 + UI before promotion; all harness rules (completion guard, instrument
  context, byte-identity, consumer-fired after the FIRST arm).
- Shadow stages publish on shadow topics — NOT `prediction.*` — because the EPMs'
  descending-subtraction sockets are live-by-default and feeding them is itself a
  behavioral change to be made deliberately (stage M3+, gated).

## M0 — Raw material: the vocabulary and transition model on the RHYTHMIC gait

All prior EPM measurements predate V3 BASE. A rhythmic body should produce a far
better-conditioned vocabulary and transition graph than the shuffle era's.
1. `v3base__ga__bodypose`: re-introduce the body-pose EPM (+ later the 4 leg EPMs —
   exactly the P0 disposition: "re-introduced by the predictor WITH a consumer").
2. Collect per-tick winner streams (the bp_/bpt_ mirrors, DIAG_INTERVAL=1, n=4).
3. Offline: empirical transition matrix from each run's first half; **k-step rollout
   accuracy on the second half vs persistence** (k = 1..10). GATE: the transition
   model must beat persistence decisively at k ≥ 3 — the planner's raw material check.
   If it fails: conditioning work on the vocabulary BEFORE any planner code.

## M1 — The shadow planner (zero authority)

`MotorPlanner` module: horizon buffer H ≈ one stride, rolled from the transition
graph; per-element confidence; per-tick shift-and-refine; branch masking by predicted
TLE along the rollout (high-expected-surprise branches pruned — EFE arriving
bottom-up). Publishes on `shadow.plan.*`. Scored offline against traces:
- GATE: near-horizon (1–5 tick) predictions of proprio/contact beat BOTH persistence
  and the unrefined rollout — refinement must add anticipation, measurably.

## M2 — One consumer: the objective socket with earned confidence

Plan head enters as confidence-weighted error retarget (the KeyframeGait pattern),
w = f(1/(planner_tle+ε)), gain-0-guarded. Judged on anticipation metrics (td timing,
obstacle pre-adjustment), the full set, arena + corridor, n=20, operator UI.

## M3 — The inhibitory pathway + the thesis experiment

Contact-contingent masking (the P4 answer: strokes wait for predicted contact), then
the (d)-tests on the V3 body: perturb / lesion / relocate mid-episode — does the PLAN
re-roll (visible re-inference in the buffer) and behavior recover? This is the
project's central claim, run on an honest substrate for the first time.

## Files

- M0: config script (v3base__ga + EPM modules); offline scorer (planscore.py).
- M1: cpp_core/src/ogma/modules/MotorPlanner.{cpp,hpp} (new module, bus-native);
  registry entry; config arm; body-log mirrors for plan diagnostics.
- M2/M3: MotorEPMv2 objective-socket wiring only (existing pattern); no new paths.

## PART III campaign log

**2026-08-10 — M0: FAIL (the gate worked; no planner code gets written on this
material).** Per-tick chain: self-transition mass 0.72 → the argmax model IS
persistence (lift 1.00 at every k). Event space (dwell collapsed): real short-range
structure — next-posture 3.9× chance — decaying below baseline by ~5 events. Beam
(width 8, top-3 branching, the fairest read for a MASKED planner): truth-in-beam ≈
chance by k=5; top-3 next-event coverage only 0.30. **Verdict: the 51-token
unconditioned first-order material cannot support a stride-scale horizon.**
DIAGNOSIS, two levels: (1) these observer EPMs were built WITHOUT phase context —
§0's "feed it phase" rule, measured load-bearing for the controller, was never
applied to the vocabulary; on a rhythmic body transitions are phase-dependent by
construction. (2) Vocabulary conditioning never done: common mode (the tuck rest)
uncentred (§0 rule 2), 51/200 nodes in use. ⇒ **M0.b (next): phase-conditioned
transition scoring on the SAME banked streams (offline — P(next | token, CPG-phase
bin) needs the cpg field joined per tick), then a conditioned vocabulary arm
(centred input, phase context) if the offline read demands it. The gate re-runs
before any MotorPlanner code.**

**2026-08-10 — M0.b: the planner AS the instrument (operator pivot), and a §3.2
catch.** The operator redirected M0.b from offline scoring to building the
MotorPlanner itself as a zero-authority shadow — "reflexes write the t0 row, EPMs
write the tN rows; analyze the probability cone this produces." Built:
`MotorPlanner` (cpp_core), no bus outputs, learns P(next | token, phase_bin) online,
rolls the cone each tick from the t0 row with phase advance φ+n·ω, verifies itself
at probe depths {1,3,5,8,13,21,34} against the arriving future; `plan` mirror in the
body log; `conescore.py` reads it with a per-depth PERSISTENCE baseline from the
same run's `bp_win` stream. Protocol = M0 (arena, 12k, seeds 1–4, per-tick diag),
conditioning on `rhythm.body.gait`, mask off. **Full-run table (n=4): top1 beats
persistence at k=5–21 (lift 1.2–1.6, peak k=13) and marginal 2–2.7× throughout —
but this is a FIRST-HALF ARTIFACT.** Warm-model cut (second half only, per-seed
lift vs second-half persistence): k=5 **0.96±0.09**, k=13 1.21±0.51, k=21
0.78±0.15 — the conditioned cone equals persistence once warm. §3.2 check found
the cause before the verdict: **the conditioning variable was noise** —
`brt_plv` on these very runs is 0.02–0.10 (the BodyRhythmTracker never locks on
V3 BASE), so the 8 phase bins fragment the transition counts without carrying
signal, and the cone's no-data fallback (hold state) degrades it to persistence.
**Verdict: NULL against a broken reference — a measurement outcome, not a verdict
on phase conditioning.** The masking arm was NOT run on this reference (masking on
noise affinity would measure nothing). ⇒ **M0.c: same protocol, conditioning
swapped to `rhythm.cpg.body` — the CPG clock, the phase context measured
load-bearing for the controller itself (§0 rule 3). Planner now accepts 2-D
[cos,sin] clocks and self-estimates ω from the clock's own advance.**

**2026-08-10 — M0.c + masking experiment #1: the vocabulary is the limit,
confirmed three ways on a perfectly controlled A/B.** The planner is shadow (zero
authority), so same-seed arms have BIT-IDENTICAL token streams (verified: `bp_win`
sequences equal across all arms) — every cone delta is the planner-side variable
alone. Three results, n=4 each, arena 12k:
(1) **Conditioning reference is irrelevant.** CPG-clock conditioning (M0.c)
reproduces the brt-conditioned table to the third decimal (top1@13 0.108 both;
@34 0.084 vs 0.082). Whatever phase either reference carries, the transition
model can't use it.
(2) **The beam holds no structure past dwell + frequency.** Cone reality-in-beam
at k=21–34 is 0.34–0.38 — statistically the STATIC most-frequent-12 beam (marginal
top-12 mass 0.396). The dynamic cone buys nothing over a frequency table at
stride scale (body period ≈ 70 ticks; k=34 ≈ half a stride).
(3) **Phase-affinity masking (mode 1, floor 0.02) DAMAGES the cone.** Consumer
fired hard (masked_out 13k–39k/run), yet top1 falls at every depth (0.211 vs
0.234 @5; 0.069 vs 0.084 @34), assigned mass falls, and ~12% of probe rows empty
out entirely. Token↔clock-bin affinity is too weak to gate on — the mask removes
reality, not noise. Same root cause as (1).
**Verdict: the MASKING INTERFACE is WORKING (built, consumer-verified, measures
controlled deltas — the operator's E/I method now exists as an instrument); the
phase-affinity mask ON THIS VOCABULARY is a REGRESSION; the tick-level cone on
the current body-pose tokens is refuted as planning material — persistence-
equivalent argmax (warm-model lift ≈ 1.0), frequency-equivalent beam.
Re-use context: any mask/conditioning retry needs tokens that actually carve the
stride.** Two candidate fixes, in tension order: **(b) event-space cone** —
planner rows advance per token-CHANGE event instead of per tick (M0 measured the
only real structure anywhere in this stream: next-event 3.9× chance), planner-side
change only; **(a) conditioned vocabulary** — centre the tuck common-mode out of
the 12-D input and give the observer EPM phase context (§0 rules 2+3; needs a
conditioning bridge or an EPM input-conditioning extension — a substrate design
decision). (b) is cheap and sharpens (a)'s requirement; (a) is the deeper fix.

**2026-08-11 — the MOTOR PIANO ROLL + the AUTHORITY HORIZON (operator-directed
instrument; "the authority horizon line is essential").** The roll is the plan
buffer made visible: executed past immutable left of the playhead, t0 owned by
reflexes, future columns the mutable rows loops will write into — slow loops far,
fast loops near — with reflex suppression allowed only where confidence is EARNED.
Built: (1) planner-side per-token 12-D pose readout (Welford; instrument, not
percept) decoding every cone row to joint space via the law of total variance —
the ±1σ FAN; (2) an online PERSISTENCE baseline scored under the identical
pending protocol (Pending carries tok0), giving `cone_persist` per depth;
(3) **`authority_depth` = deepest probe with n≥200 verdicts where verified cone
top1 > 1.05 × persistence** — the gold line; (4) diag payload shipping the whole
roll (40 × 12 mean/sd) + a 128-tick past ring (server-side, the gait-raster
anti-aliasing lesson) at ~48 KB/payload; (5) `PianoRollInspector` (12 tracks =
4 legs × 3 joints, or per-leg 3-track zoom): past curve, decoded future with
saturated-vs-washed authority segments, fan, playhead, stride ruler from the
rhythm reference, adaptive per-track ranging; registered for MotorPlanner with a
teaching doc. Verified end-to-end on a LIVE run (ControlClient →
module_subscribe_diag → ZMQ payloads validated → offscreen render). First live
reading, honest: **authority_depth = 0 — under the identical-protocol baseline,
persistence ≥ cone at every depth (k1: 0.74 vs 0.77) — reflexes own the roll.**
The line is drawn; the campaign's job is to move it right. Body-log `plan`
mirror now carries `pr` (persist) + `auth` per line, so seed runs record the
authority trajectory.

**2026-08-11 — PER-JOINT VERIFICATION: the first verified positive planner
result, found by the operator's eye.** Operator observation on the live roll:
"some joints are predicted a few ticks out while others are not" — on a SINGLE
whole-body predictor, so the difference had to live in the marginals. Built:
per-joint continuous-space verification under the identical pending protocol
(Pending freezes the decoded joint prediction + the pose at prediction time;
verify accumulates per-depth per-joint |err| for cone-decode vs hold-pose).
**Result (live, seed 2, n≈9k verdicts/depth): the error-ratio table splits the
body cleanly.** At k=1–3 every joint LOSES ~3× (the decode jumps to token mean
against a barely-moving body). At k=8–34, **all eight hip1/knee marginals WIN
12–23%** (ratios 0.77–0.92); the four h2 marginals never meaningfully win
(~0.91–1.07 — the slow postural joints, where hold-pose is near-optimal).
Native authority bands: FL/FR/RL/RR h1 and knee all **[8–34]**; h2 ·/·/21-21/
13-13. TWO consequences: (1) **the whole-body cone DOES carry verified
stride-scale structure in its joint marginals** even though the token-argmax
authority is 0 — the global gate was measuring the argmax, not the material;
(2) **authority is a BAND, not a line from the present** — the ladder-from-t0
definition hid the win entirely (contiguity from k=1 demanded beating
persistence exactly where it is unbeatable). This matches the operator's
architecture directly: reflexes own the near field; a planner earns an
INTERVAL of the future. Implemented: `joint_band` [lo,hi] per joint (longest
winning run) in the payload + body mirror (`jband`); the roll now draws each
track's amber authority band with the bright segment inside it; global dashed
line unchanged. **Re-use context for M1: refinement/masking experiments should
be scored per-joint in the band, where verified material exists — h2 tracks
and k<5 are reflex territory on this vocabulary.**
