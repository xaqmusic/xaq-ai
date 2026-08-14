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

**2026-08-11 — THE TWIN GATES (S0 + M0.d): formalized, with the full module
audit behind them.**

*The audit (all 69 modules; operator asked "pieces we can put together, or
hand-roll?").* Answer: **the pieces exist; hand-roll nothing but scorers.**
The chunk pipeline the sequencing conversation asked for was designed and
built in Phase 1: `SequenceGNG` (n-grams of winners → JL → GNG motifs +
successor counts, with a complete registered inspector widget — the
"abandoned chunk UI" is `seqgng_inspector.py`, alive, just starved of a live
module) → `GNGRollout` (K-sample rollouts with MOTIF TELEPORT — planning-via-
chunks, built) → `MotorRepertoire` (chunk library, drive-tagged
crystallization, playback with policy suppression) + `EpisodicCapture` +
`ChunkAbortGate`/`ChunkOutcomeGate` (the closed-loop abort machinery) +
`GaitSelector` (sequence REINFORCE) + `KeyframeGait` (phase-indexed posture+
velocity map with per-bin self-precision — PROMOTED, live in V3 BASE).

*The recorded verdicts, and why re-entry is legal now (§3.1 discipline).*
The archive is uniformly negative on chunking — `seqgng_body.baked_count=0`
on the tabula-rasa picrawler; GaitSelector NULL ("no primitive in the
library translates"); the 11-module cartpole chunk arm lost 18% to minimal;
EpisodicCapture "currently silent"; ChunkAbortGate "wired but never fires."
But every one of those verdicts carries the SAME re-use condition, recorded
identically in three places: *"only after a known-good translating primitive
exists inside a closed-loop, abortable controller."* **That condition is met
as of V3 BASE** (cv 0.75, 20/20 walkers): the RL-era chunker starved because
the body never produced recurring behavior to chunk; today's body does, and
M0 measured its event-space structure directly (next-posture 3.9× chance).
The abort machinery the condition demands is already built. Re-audition is
not re-proposing a refuted lever — it is exercising the recorded re-use
context.

*GATE S0 — is there chunkable sequence structure? (the long-loop gate).*
`SequenceGNG` over the body-pose winner stream — with one required
extension: **`event_mode` (new param, default 0 = byte-identical legacy)**
that pushes the window only on winner CHANGE. Without it the 0.72
self-transition stream makes every motif a dwell run; the Cell-era per-tick
windows are one plausible reason it never baked ("food-approach takes
seconds but SequenceGNG encodes 83 ms" — the KeyframeAverager doc recorded
this exact diagnosis in 2026-05). Shadow instance `seq_bodypose` on
`reality.bodypose.pose`, window 4, proj 64, bounded nodes; nothing consumes
the motifs. Instruments: the existing `seqgng_inspector` + a body-log
mirror (`sg_*`: nodes/baked/motif/match_conf) + `seqscore.py` (new).
**Pass:** motifs BAKE (≥3 by ~3k events, the contract acceptance), the
vocabulary self-limits (support-EPM signature), motifs recur across seeds,
and the active motif's successor argmax beats the flat first-order event
chain on next-event prediction. **Fail:** no baking or no lift ⇒ chunking
waits for M0.d's vocabulary.

*GATE M0.d — phase-space vocabulary (the refinement-mechanics gate).* The
structural diagnosis stands: position-only tokens self-intersect on a limit
cycle ("knee at 0.3 going up" ≡ "going down"), which is WHY the chain is
persistence and the near-field decode jumps to the token mean. Fix at the
input (§0 rule 2): body publishes `reality.proprio.joints_dyn` (24-D: q +
per-tick Δq; a transparent sensor reduction — Δq dims are naturally
zero-mean, dissolving most of the common-mode problem), a new observer EPM
`body_pose_dyn` over it (RBF 24-D, dim ranges MEASURED via range probe per
the support-EPM precedent, never assumed), and a second shadow MotorPlanner
`motor_planner_dyn` keyed to it. **Pass:** per-joint authority bands extend
below k=8 (the jump-to-mean dies) and/or past k=34; h2 tracks earn first
bands; self-transition mass drops materially from 0.72; dwell shortens
(tokens carve the stride). **Fail:** conditioning goes deeper (explicit
common-mode centring arm) before any M1.

*Protocol.* ONE shadow config carrying both gates on the same streams:
v3base__ga__bodypose + joints_dyn + body_pose_dyn + seq_bodypose + both
planners — every addition zero-authority, so the behavioral base is
untouched (the M0.b↔M0.c bit-identical-stream check is the precedent; a
same-seed stream-identity check against a prior M0.b log is step 1 of the
run). n=4 seeds × 12k arena, DIAG_INTERVAL=1, M0-matched. Two gates, two
independent instruments, one set of runs — one-lever discipline applies to
behavioral levers, and there are none here.

*Decision matrix for M1 (the first authority-bearing loop).*
- S0 loud + M0.d loud → chunk PROPOSER over the dyn vocabulary writing far
  rows as sequence templates, lattice refinement inward (the operator's
  long-loop architecture, both halves earned).
- S0 loud only → chunk proposer on the current vocabulary, event-space rows.
- M0.d loud only → backward-pass lattice refinement (masking propagates
  future→present survival) on the dyn vocabulary.
- Neither → vocabulary conditioning arm (centred common-mode) before any
  authority. In ALL cases: reflexes keep t0; per-chunk/per-row precision is
  EARNED from verification (the authority-band machinery, already live);
  ChunkAbortGate/OutcomeGate are the recorded execution guards when
  authority eventually flows.

*Build list.* (1) `SequenceGNG.event_mode` (C++, default-0 guarded);
(2) body-side `joints_dyn` publish (new topic, no subscribers in existing
configs — behaviorally null); (3) `sg_*` body-log mirror; (4) the twin-gate
config + launcher entry per convention; (5) `seqscore.py`; (6) range-probe
pass for the dyn EPM's dim_min/max. KeyframeGait, GNGRollout, MotorRepertoire,
ChunkAbort/OutcomeGate: untouched now, named as the M1/M2 assembly kit.

**2026-08-11 — TWIN GATES RUN: both FAIL as built, both failures diagnosed as
CONDITIONING — and a §3.2 harness catch on the way.** Build: SequenceGNG
`event_mode` (default-0), `joints_dyn` [q, Δq] body stream, `body_pose_dyn`
EPM (measured dim ranges), second planner, sg_*/bd_*/pland mirrors,
`seqscore.py`; smoke-verified consumers + BIT-IDENTICAL bp_win stream vs
M0.b (behavioral null confirmed). **The catch:** first collection showed the
bodypose CONTROL cone's top1@1 collapsed 0.71→0.24 on an identical stream —
the planner's learn context was `static thread_local`, shared by the twin
config's TWO instances, cross-writing each other's transition tables. Fixed
(members), rerun, control reproduced M0.c exactly (0.713). The control arm
caught the bug; this is why the twin config keeps one.

**GATE S0 — FAIL on this vocabulary (n=4, ~2.7k events/run).** Zero motifs
baked (again — the RL-era signature, now WITH event mode), vocabulary runs
to the 96-node cap in 4/4 seeds, match_conf 0.00, and the motif successor
argmax scores 0.118 vs the flat first-order event chain's 0.175 — **lift
0.67: the chunker is WORSE than the chain it must beat.** Diagnosis: (1)
exact/near 4-gram recurrence is rare over a ~60–100-token vocabulary with
GNG boundary jitter — the noisy tokens shred motif identity; (2) hash-window
clustering treats all 4 positions equally, so windows differing in the LAST
token cluster together — the successor is conditioned on a state blurrier
than "last token" alone, structurally losing to the chain on noisy streams.
**Verdict: NULL — sequence memory beyond first order is not extractable from
THIS vocabulary by window clustering. Re-use context: retry after a
vocabulary that passes M0.d-style dynamics checks, and/or with
last-position-weighted or shorter windows.**

**GATE M0.d — PARTIAL: the stream improves exactly as designed; the chain
does not.** Stream-level (real, n=4): self-transition mass 0.72 → 0.55,
dwell ~halved, vocab 51 → 109, and h2 joints begin earning authority bands
in the dyn arm. But the cone over dyn tokens is no better than the bodypose
control (top1/persistence ×0.90–1.08 shallow, ×0.56 at k=34), and the
decisive planscore check — the UNCONDITIONED first-order chain, no phase-bin
sparsity — shows dyn lift 0.94 (k=1) falling to 0.64 (k=10): **worse than
persistence at every depth.** Diagnosis (§3.2 faithfulness): a weakened
slice of the mechanism — single-tick Δq of a noisy servo stream is a poor
velocity estimator, and the dq dims' RBF ranges (set from p1/p99 envelopes)
leave the typical |Δq| crowded near zero. The phase-space idea is measured
to do its structural job (self-intersection reduced); the VELOCITY ESTIMATE
is the broken part. **Verdict: PARTIAL — retry with a smoothed velocity
(EMA/boxcar over ~3–5 ticks, matching servo dynamics) and scale-adapted dq
ranges before judging phase-space vocabularies. ⇒ M0.d.2.** Secondary notes:
seed 3's dyn EPM hit the 200-node cap (raise for v2); the phase conditioning
(bins=8) costs 8× count sparsity for a measured-zero gain — drop to the
minimum in v2 arms.

**2026-08-11 — M0.d.2 (smoothed velocity): the anti-signal fixed, the gate
still not passed — and the campaign's token-chain chapter closes.** v2
changed exactly the diagnosed conditioning: q̇ = EMA(Δq, α=0.3) body-side,
dq ranges re-measured on the smoothed signal (p2/p98+20%, ~half the raw
envelopes), max_nodes 320, planner_dyn phase_bins 2. Stream identity
verified again. **Result (n=4): the unconditioned dyn chain went from
LOSING to persistence (v1 lift 0.94→0.64 with depth) to TYING it flat
(0.94/0.97/1.00/1.02/1.03/1.00 at k=1..10)** — the smoothing removed the
noise-injection, confirming the estimator diagnosis — **but surfaced no
positive structure.** The cone shows a small mid-horizon bump (×1.14–1.20
at k=5–8, n=4 — a signal at best) yet decays worse at stride scale (×0.52
at k=34); bands did not widen (38/48, width 17.7); authority 0 everywhere.
The vocabulary meanwhile kept growing with every cap raise (163 in-use;
238–278 total, near the new 320 cap) at unchanged self-mass 0.55 — finer
tiling without added predictability, at per-token sample cost (k=1 model
0.47 on 163 tokens vs 0.67 on 51).

**The pattern, now complete across every arm tried:** per-tick FIRST-ORDER
TOKEN CHAINS on this body's proprioceptive stream do not beat persistence
under ANY tested vocabulary (position, position+raw-q̇, position+smoothed-q̇)
or conditioning (none, brt phase, CPG clock, 2 or 8 bins); window-clustered
chunks lose to the chain itself; event space holds only ~1–4 events of
structure. **The one verified positive in the entire program is CONTINUOUS:
the per-joint marginal decode beats hold-pose by 12–23% in the k∈[8,34]
band.** The predictable object at stride scale is the continuous trajectory
distribution, not the symbolic successor. **Verdict: M0.d.2 PARTIAL
(estimator fixed, gate unpassed); the token-argmax planning route is
REFUTED on this substrate at per-tick granularity — re-use context: a
vocabulary whose tokens are EARNED FROM predictive success (e.g., a
descending-predictor-conditioned EPM per §0's predictive-coding path)
rather than from spatial quantization.** Recommendation for M1 recorded:
the piano-roll architecture survives unchanged — its rows already carry
continuous per-joint distributions (the decode/fan); make THAT the
prediction substrate the refinement/masking loops operate on, with the
authority bands as the earned-confidence gate, and drop the requirement
that a symbolic chain must first beat persistence.

**2026-08-11 — M1 SUBSTRATE PIVOT (operator decision) + CONTINUOUS MASKING
with the three-layer debugging view.** Operator, after the token-chain
chapter closed: "the approach needs to be entirely fresh, and the new joint
piano roll is going to be the substrate on which we can excite and inhibit
future motions" — with a hard debugging requirement: the widget must show
the ORIGINAL motion, the MASK applied, and the FINAL motion. Built:
`mask_mode=2` — continuous region inhibition ON THE ROLL: token mass whose
READOUT pose falls in [val_lo,val_hi] on the masked joint (or all) at cone
depths [depth_lo,depth_hi] is suppressed by mask_strength, and because the
mask reroutes the ROW itself, suppression propagates into every deeper row.
Dual decode (raw pre-mask + final) and dual verification (per-depth
masked-vs-raw |err| = the inhibition damage/benefit meter); raw roll ships
only while suppressing. Widget: dashed ghost + faint fan (original),
magenta region rect (the mask), bright layers (final). All params inert by
default (gain-0). **Demo arm `__mask2` verified live** (inhibit FR-knee
rest [0.7,1.05] at depths 8–21 — "don't be at rest a quarter-stride out"):
255k suppressions, divergence localized to the mask window, damage meter
honest (+14% err at k=8 — the body DOES rest there), and the reroute
visible ACROSS joints — masking one joint moves its correlated partners
through the whole-body mixture, which is precisely the debugging visibility
the contract demanded. The E/I loop now has its full instrument: excite =
the decoded roll, inhibit = region masks, verify = the banded per-joint
error meters.**

**2026-08-11 — the LEG-NAMING MIRROR (operator-diagnosed on the roll).** With
per-tick motor traces beside the 3-D view for the first time, the operator saw
the red leg lift while the widget said FL. Geometry confirms: true forward is
+Z (eyes / corridor / fwd_v), so body-left = +X — yet leg 0 "fl" (red) is
built at x<0, the anatomical FRONT-RIGHT. The names were assigned in the
default-camera screen frame (the mirror illusion of labelling a body that
faces you). The frame is used CONSISTENTLY end-to-end — every action topic,
config, event, instrument, and historical per-leg finding ("fl brakes 2.5×" =
the red = anatomical front-RIGHT leg) — so the record is coherent and the
mirror is behaviorally null in sim. Fixes: the roll now labels tracks by
ANATOMY tinted the leg's sim color (cfg name kept for cross-reference); loud
warnings at LEG_NAMES in the body script (do NOT rename piecemeal — the blast
radius is every config + topic + ledger history) and in the sim2real port doc,
where the servo map MUST be written by anatomy, not by name. Instrument
payoff: this is exactly the class of defect the piano roll exists to catch —
invisible to every aggregate metric, obvious the moment motor output sits
beside the body.

**2026-08-11 — M1 MASK AUTHOR v1: the authoring slow loop BUILT and WORKING;
per-joint inhibitory keep-rights are EARNABLE but seed-specific — plus a
dual-cone fidelity fix and the demo-mask/band interaction explained.**

*The operator's observation, diagnosed first.* On the live `__mask2` demo the
confidence (authority-band) windows grow everywhere EXCEPT the front-right
leg. Mechanism confirmed in code: the per-joint bands score the FINAL
(post-mask) decode, the demo mask fights reality on that leg ("don't rest"
where the body does rest — the recorded +14% @ k=8), and mode-2 rerouting
moves the whole-body mixture, so all three FR tracks carry damaged final
decodes and their bands stay pinned while unmasked legs grow. Expected;
the meter is honest. (Design note kept: bands describe what the planner
currently OUTPUTS — during inhibition experiments they will show the mask's
cost, which is precisely their job as the earned-confidence gate.)

*Fidelity fix (build prerequisite).* The old "raw" layer was decoded from the
SAME cone pre-this-row's-mask, so past the first masked depth it inherited
upstream rerouting — understating the mask's true effect (the demo's recorded
+14% @ k=8 stands: k=8 WAS its first masked depth). Now, while a region mask
is live, a TRUE second unmasked cone propagates beside the final one:
raw-vs-final is a genuine per-tick counterfactual at every depth, the ghost
layer is honestly "the original motion" (operator contract), and — the point —
the dual verification becomes a perfectly controlled within-run A/B: both
decodes frozen at prediction time, scored against the same arriving reality.

*The build (author_mode=1, all params inert by default — gain-0).* The M1
slow loop that AUTHORS masks: per (probe, joint) it tracks the RAW decode's
signed residual (Welford); each trial it proposes the cell with the strongest
standardized residual t≥3 not yet tried (where the un-inhibited excitation
systematically hallucinates), builds a one-sided slab from the past-ring
envelope (μ + sign·[0.5σ, 3σ] — placed from the body's own running stats,
§5), depth windows probe-to-probe within [5,34] (the ledger's re-use
context: k<5/h2-near-field is reflex territory), with a random h1/knee-biased
arm as fallback (never fired: the residual gradient always had a target).
Trial = 800 ticks masked + 48 drain; judge on all probes ≥ the mask's first
depth. Instruments: `author` block in snapshot/diag (phase/trial/cand/kept +
`res_top` hallucination cells), body-log `plan.au` mirror, piano-roll AUTHOR
readout (the magenta rect hops per trial — the search made visible),
`authorscore.py`, config `__m1auth` + launcher entry. Consumer checks all
fired (masked_out 136k–162k per run; trials 10/10/10/10 at n=4 × 12k arena,
seeds 11–14).

*v1.0 result — whole-body keep-gate (ratio_all < 0.95): NEAR-NULL, and the
null is DIAGNOSTIC.* 40 trials judged, 1 kept (s11 j7 [-0.28,-0.10] d[6,8],
r_all 0.943), no recurrence. Reconstructing all 40 trials: ratio_all sits
0.96–1.14 (masks neutral-to-damaging at body altitude) while ratio_tgt shows
loud repeated wins — s11 rt 0.86/0.86/0.88 (j6/j7), s13 rt 0.75/0.82/0.87
(j1), s14 rt 0.82/0.84 (j5) — the 11 untargeted joints DILUTE the pooled
ratio to ~1. The wrong altitude, exactly as the per-joint-verification entry
predicted ("score per-joint in the band").

*v1.1 — per-joint keep-gate (ratio_tgt < 0.95 AND ratio_all < 1.0, the
no-damage guard): the author EARNS keep-rights.* Same seeds → bit-identical
body streams and trials (masked_out equal to v1.0 per seed — the scoring
change is the only variable; a free controlled comparison and a determinism
check in one). Keeps: s11 4 masks (j6/j7 h2, d5–13, rt 0.855–0.933), s13
4 masks (j1 h1, d5–34, rt 0.749–0.912), s12/s14 0 — s14's two loud target
wins REJECTED by the guard (r_all 1.02–1.03: target wins, body pays; the
demo-mask failure mode, now caught automatically). Class-level regularity:
8/8 kept masks are guided, below-envelope slabs on slow joints — the raw
decode systematically UNDER-predicts them (jump-to-mean), and inhibiting the
low-side mass corrects it, verified out-of-sample. No cross-seed recurrence
of specific regions: the residual field is seed-specific (each seed's gait
settles a different attractor with different decode biases).

**Verdict: the AUTHOR MECHANISM is WORKING (built, consumer-verified,
deterministic, honest keeps/rejects including the guard) — the operator's
E/I loop now has its authoring half as an instrument. The SEARCH RESULT is
PARTIAL: per-joint inhibitory keep-rights are earnable on this vocabulary
(2/4 seeds, rt 0.75–0.93 at n=800–4000 verdicts/keep), but what is earned is
SEED-SPECIFIC MODEL-BIAS CORRECTION (below-envelope, slow-joint, guided —
the class recurs; the regions do not), not yet a body-invariant inhibitory
vocabulary. Re-use context / next rungs: (a) CLOSE THE LOOP — re-apply the
kept set continuously after keeping and verify the final roll's per-joint
bands WIDEN vs the no-author control (authority earned through the meters,
the M1 contract); (b) class-level recurrence needs n≥20 before any "the
decode under-predicts slow joints" finding; (c) the deeper fix remains
vocabulary conditioning — a predictive-coding-conditioned EPM whose tokens
are earned from predictive success would shrink the residual field the
author is currently mopping up.** §3.2 notes: no tautology (all params new),
consumers verified per seed, control = the same-seed v1.0/v1.1 identity,
faithfulness = the true-counterfactual fix landed BEFORE any keep was
recorded.

**2026-08-12 — M1 RUNG (a): THE PREDICTION LOOP CLOSES — earned inhibition
compounds into a measurably better operating roll (operator-directed:
"close the loop and fully verify the instrumentation before lever (b)").**

*Design.* `author_apply=1`: kept masks apply to the roll CONTINUOUSLY from
the moment they are earned, in CHRONOLOGICAL keep order — each keep is
judged marginally against its predecessors' composite, so keep order is the
order in which validity was established (and the kept cap now STOPS new
keeps rather than evicting: eviction would silently invalidate every later
keep's judgment). Up to THREE cones per tick: FINAL (kept+candidate), BASE
(kept only — the operating roll), RAW (unmasked). Candidates are judged
MARGINALLY (final vs base) so they cannot inherit the kept set's credit;
with the author on, the main verification and the authority bands score
BASE, so bands measure the kept set, not trial churn (manual-mask configs
keep their final-scored semantics). `jerr`/`jpers` tables now mirror into
the body log (`je`/`jp`) — jband alone is thresholded and hides sub-0.95
movement. Widget: faint steady rects = the earned set operating; bright
hopping rect = the live trial. Control arm = same build, `author_apply=0`
(`__m1auth_ctrl`, harness-only). Scorer: `authorab.py`.

*Smoke regression (seed 11).* Pre-keep trials bit-match the prospector run
(trial-7 keep identical: rt 0.8574); after the first keep the same j7
candidate that scored rt 0.855 standalone scores rt 0.938 MARGINALLY — its
correlated partner j6 was already kept and operating, so the candidate is
credited only for what it adds. The credit-inheritance protection observed
working on live data.

*A/B (n=4 matched seeds × 12k, apply vs prospector, same build).*
- s13 (kept at trials 1–2 → operating ~60% of the run): target-joint
  operating-error ratio apply/control **0.881 @ k=21, 0.914 @ k=34**;
  untargeted joints 0.987–1.000; band width 269 → 290 (**grew**).
- s11 (kept at trials 7–8 → operating ~25% of the run): target ratios
  0.973–0.983 from depth 5 outward (the d[5,5] masks propagate deeper);
  untargeted 0.989–1.004; band width 198 → 185 (**dipped** — see caveat).
- s12/s14 (no keeps): max |Δje| = 0.0000 — apply ≡ control exactly; the
  three-cone plumbing is inert when idle (built-in null, clean).
Benefit magnitude tracks keep-time coverage (cumulative accounting dilutes
late keeps), matching the trial-time instantaneous ratios (0.75–0.94).

**Verdict: WORKING — the M1 contract's rung (a) is met: masks EARN
keep-rights through the meters and the earned set VERIFIABLY improves the
operating roll where it acts, with clean nulls and no collateral damage.
Caveats recorded: (1) s11's total band width dipped 6.5% — threshold
crossings at diluted-cumulative altitude, not a measured harm (its je
ratios improve); re-check with earlier warmup or longer runs before calling
it real; (2) cumulative meters understate late keeps — a windowed
(since-first-keep) accumulator is a v2 nicety; (3) global token authority
stays 0 everywhere, as expected — the symbolic argmax is refuted material,
the continuous marginals are the substrate. INSTRUMENTATION STATUS FOR (b):
proposal → trial → marginal keep → apply → compound verified benefit is now
a fully closed, self-auditing chain with built-in nulls. Lever (b) — first
behavioral authority: the BASE roll's decode published as a weak, band-gated
objective (reflexes keep t0, per-joint gate = the earned bands, gain-0
guarded, ChunkAbort/OutcomeGate as recorded execution guards) — is cleared
to build NEXT, with the operator watching the UI before any promotion.**

**2026-08-12 — LEVER (b): FIRST BEHAVIORAL AUTHORITY — the plan pull.
Built, guarded, measured at n=6: SAFE, mildly positive, not loud. NOT
PROMOTED (UI review is the operator's gate).**

*The mechanism (rewrite-rule form).* The error the behavior minimizes: the
discrepancy between the body's trajectory and its OWN verified prediction —
proprioceptive active inference, plan-as-prediction (§5 rule 7 honored: no
rhythm injected, no trajectory scripted). MotorPlanner publishes its BASE
(operating, kept-mask-shaped) roll decode at plan_depth=8 as per-leg
PredictionTokens `[3 targets | 3 weights]` on `objective.plan.<leg>`; a
joint's weight is 1 ONLY where its verified authority holds at that depth
(the slot-win test the bands are built from — earned, never assigned).
MotorEPMv2 gains a second posture-objective socket (`plan_topics` +
`plan_gain`) fused with the keyframe objective PER JOINT, precision-weighted
(the LateralVoter pattern): `w_eff = wk + plan_gain·wp`, so an ungated
joint's keyframe pull is untouched (never weaken a working loop). Distress
above `plan_distress_cut` zeroes all plan weights — reflexes own
emergencies, and t0 always. Because the published decode is the BASE roll,
EARNED inhibition (rung a) now has a behavioral path — and trial candidates
do NOT leak into behavior (base excludes the live candidate by
construction).

*Guards verified.* (1) gain-0: `__m1auth__planpull0` (publisher ON, consumer
gain 0) is behaviorally IDENTICAL to `__m1auth` tick-for-tick on a matched
seed (66/66 mirror lines). (2) Publisher gate matches the verified authority
map exactly on live data: h1 (j0–3) and knees (j8,9,11) gate in; h2 (j4–7)
never do — reflex territory stays reflex. (3) Consumer fired across all arm
seeds (pl_pull ≈ 0.015, ~10–11.7k gated publish-ticks / 12k). (4) Config
diff between arms is `plan_gain` alone.

*A/B (seedavg, n=6 × 12k corridor 0.3, control = gain 0).* Transport:
net_z 4.87±1.76 → 5.49±1.78 (+13%, sub-σ), straight 0.44 → 0.47, flat_v
0.03 → 0.04. Stability: falls 0.67 → 0.17 (4 total → 1), unstable 0.12 →
0.09, planted 3.37 → 3.49, plv_wn 0.91 → 0.98. Rhythm: step_cv_real 0.88 →
0.81 with the step-period spread HALVED (σ 6.3 → 3.2 ticks) — consistent
with prediction-fulfilment adding coherence. Cost, honestly: arm seed 2
regressed (tilt_sd 0.93, net_z 2.35, the only nonzero panic_duty 0.08 in
either arm) — the pull can entrench a bad episode; belly and scrub flat.

**Verdict: PARTIAL (safe + mildly positive SIGNAL at n=6) — behavioral
authority flows through earned bands without destabilizing the stack; no
metric shows systematic damage; transport/stability/rhythm all lean
positive; the effect is NOT loud (§3.3 — a real capability announces
itself; +13% sub-σ is not an announcement). NOT PROMOTED: the operator's UI
review is the promotion gate. What to WATCH live: does the gait visibly
gain step-rhythm coherence; does arm-seed-2-style entrenchment appear (a
stumble the pull then commits to); and the demo-with-teeth — hand-apply the
FR-knee rest mask on the [O] bench with the pull live: does inhibiting the predicted
rest actually recruit an earlier FR step? Re-use context: plan_gain dose
(0.05/0.2), plan_depth 13, and gating on the kept-mask-shaped vs raw roll
are the untried axes; a (d)-style perturbation (drop the pull mid-run,
watch re-coordination) is the sharpest next evidence.**

**2026-08-14 — OPERATOR BENCH SESSION: the levers verified by hand, and the
M1 chapter's closing diagnosis.** With the full bench in place ([O]: mix,
gate override, group masks, reflex↔plan fade, tug vectors), the operator
drove the crossfade and the tier masks live. Two observations, both
load-bearing: (1) **at fade 1 (reflexes silenced, pure plan playback) the
motors settle into a stable OSCILLATORY state** — the closed loop
pose→tokens→cone→decode→servo→pose converges to the predictor's
self-consistent orbit: the substrate carries the body's RHYTHM but the
PROPULSION (stroke, stance press, load corrections) survives only as a
blurred mixture-mean, and embodied, that residue is an oscillation that
goes nowhere; (2) **no region mask on any joint set (incl. all-h2 with
override + fade) visibly improves the gait** — inhibition reroutes mass to
other tokens the model already believes, and the vocabulary, learned from
the body's own history, contains only its habits. **Verdict (three
independent demonstrations now: token-chain refutation M0–M0.d.2, the
author's search finding only seed-specific bias masks, and the operator's
embodied playback): the E/I instrument is WORKING and BOUNDED BY THE
VOCABULARY'S CONTENT, which is a mirror of the reflex stack's habits.
Selection over habitual futures can stabilize and commit (the lever-(b)
coherence signal) but cannot innovate. Re-use context: E/I selection
becomes interesting again the moment the vocabulary contains futures that
DIFFER from habit — via a predictive-coding-conditioned vocabulary
(doctrine §0's descending-predictor path, the recorded (c) fix) and/or a
generative excitation source (M2: episodic capture of the body's own best
segments written into far rows).**

**2026-08-14 — OPTION A LAUNCHED (operator-approved: consolidate (b), then
B) + the SETPARAM_AT hook + B's design queued.** Running: n=20 powering
(planpull vs planpull0, 12k), dose arms (plan_gain 0.05/0.2, n=6), depth
arm (plan_depth 13, n=6), and the (d)-test (16k, plan_gain 0.1→0 at
t=10000 vs no-flip baseline, n=6; scorer `ddropscore.py` — windowed
disp/steps/planted/tilt + the pl_pull flip-fired check).  New generic
perturbation hook: `OGMA_PICRAWLER_SETPARAM_AT="tick:module:key:value[;…]"`
(the lesion-AT idiom generalized to any HotMutable brain param; announced
loudly per §3.2 rule 7; tick 1 doubles as arm differentiation without
config proliferation).

**2026-08-14 — OPTION A COMPLETE: lever (b) consolidation — the 0.1 dose
REFUTED, the 0.05 dose WORKING at n=20, and two harness bugs caught by the
§3.2 discipline.**

*The §3.2 catch that paid for itself.* The dose/depth arms came back
BIT-IDENTICAL to the 0.1 arm — the tick-1 SETPARAM_AT flips reported
enqueue-OK but never took effect.  Root cause found in the SCHEDULER:
`process_pending_patches` drained all queued batches then applied them in
a loop where one batch's validation throw (the pre-existing init-time
cruse patch on configs without that module) aborted the loop — EVERY BATCH
QUEUED BEHIND THE BAD ONE was silently discarded after its enqueue had
returned success.  Fixed: per-batch isolation (each batch keeps its own
validate-then-apply atomicity; rejections surface loudly; neighbors still
apply).  Historically only cruse-family patches sat in the init queue, so
no prior measurement was contaminated.  Second bug: the pl_pull/pl_w
telemetry EMAs froze at their last value when the pull went inactive — a
mid-run gain drop read as "flip never fired."  Fixed: inactive pull decays
the meter.  Bonus: the three accidentally-identical arms proved the whole
pipeline bit-deterministic per seed.

*The verdicts (all corridor 0.3, 12k unless noted).*
- **n=20 @ gain 0.1: TIE** — net_z 5.40±1.85 vs 5.44±2.24, straight
  0.47/0.46, step_cv_real 0.80/0.80.  The n=6 coherence signal was
  substantially seed luck (control's n=6 subset happened to contain its
  bad seeds).  Three arm seeds circle (straight 0.11–0.16).  The original
  lever-(b) dose is REFUTED as a behavioral improvement.
- **Dose sweep (n=6, post-fix): INVERTED-U** — net_z 4.87 (0) / 7.24
  (0.05) / 5.49 (0.1) / 6.72 (0.2); depth 13 no better than depth 8 with
  worse tails.  The house pattern again: a whisper cooperates, a shout
  fights the keyframe loop it fuses with.
- **n=20 @ gain 0.05: WORKING** — net_z 5.40→6.41 (+19%), straight
  0.47→0.52 (σ 0.15→0.11), falls 8→2, tilt_sd 0.164→0.098 with the
  outlier tail GONE (max 0.16 vs control's 0.49), bellyc equal, 20/20
  walkers, one weak seed (s19).  No metric worse.  Stability-dominant.
- **(d)-test @ 0.1 (16k, flip at 10k, n=6): FLAT** — no transient, no
  recovery signature; the pull was not load-bearing at that dose.  Late
  windows: the keep-it-on baseline shows HIGHER tilt variance.
- **(d)-test @ 0.05 (same protocol, flip verified by the fixed meter,
  n=6): a WEAK dip-then-recover** — disp 0.70→0.61 relative dip in the
  first two post-flip windows, recovery to ABOVE baseline by 14–16k
  (0.54 vs 0.40), steps recover likewise; base again shows the late tilt
  blowup (0.48±0.51 vs flip 0.18).  Signal-grade only (n=6): a hint that
  0.05 carries load and the body re-coordinates without it.

**Verdict: lever (b) at plan_gain 0.05 is a WORKING n=20 signal —
stability-dominant (falls, tilt), transport-positive, nothing worse —
and the config/launcher now carry 0.05 as the operating point.
PROMOTION AWAITS THE OPERATOR'S UI REVIEW (the standing gate); what to
watch: overall gait quality at 0.05, the rare weak seed (s19-type
circling), and whether the late-run tilt cost of KEEPING the pull on
(both (d)-tests hint at it) is visible by eye.  Re-use context: a finding
(vs signal) needs the (d) at 0.05 powered to n≥20 and varied worlds; the
late-tilt hint deserves its own windowed look before any long-run
deployment.**

**2026-08-14 — LEVER (b) PROMOTED (operator UI review, the standing gate).**
The operator's eye at plan_gain 0.05: "better stability during walking,"
slight rear-leg improvement, and — the load-bearing observation — **the
robot climbs the corridor's 30° walls markedly better than before.**  The
plan pull at the whisper dose joins the stack: the promoted operating
point is v3base + bodypose EPMs + M1 author (apply) + plan pull @ 0.05,
band-gated, distress-cut.  NAMED TARGET for the next behavioral lever
(operator-set): **REAR-LEG PLANTING** — the rear legs swing forward then
back without planting correctly, usually because the KNEE does not flex
downward to make ground contact (a swing-termination/touchdown failure;
anatomically the rear pair = cfg 'rl'/'rr', rear knees = planner joints
10/11).  This is the evaluation lens B inherits: touchdown corrections are
precisely the aperiodic, load-linked events the pose vocabulary cannot
see — if the surprise vocabulary is doing its job, rear-knee touchdown
error should become one of its DENSEST, most predictable token regions,
and its planner's bands on j10/j11 should say so.

**2026-08-14 — B GATE RUN (both pre-registered arms): the predictive-coding
pair WORKS MECHANICALLY; the surprise vocabulary FAILS its planner gates as
built — and the failure diagnosis is §6 CONDITIONING, again, in a new
place.**

*Build integrity (§3.2 catches before first run, all three latent bugs
found at design time or first smoke):* (1) the predictor's source port
silently dropped ProprioToken context (ConsensusToken-only cast — fixed
with a typed fallback; RealityToken added for arm 2); (2) the closed pair
converged to HALF-subtraction (the EPM publishes its residual as latent;
the legacy update subtracted the cached prediction from it again — fixed
with `target_is_residual`, which integrates the residual directly);
(3) the err/norm health ratio is TAUTOLOGICALLY 1 in residual mode — the
honest bite-meter is ‖prediction‖ (dp_pn) vs ‖residual‖ (dp_err).

*Arm 1 — CPG-clock context (2-D [cosφ,sinφ]), n=4 × 12k arena.* The pair
closes and bites: dp_err falls 0.89 → 0.18–0.22 with dp_pn ≈ 0.99 — the
first stride harmonic (all a 2-D linear context can express) absorbs ~80%
of the encoding.  But the residual vocabulary yields NOTHING for the
planner: self-mass 0.69 → 0.65 (gate wanted < 0.55), chain lift 0.92–0.99
(≤ persistence), pc bands equal-or-NARROWER than control everywhere, zero
new h2 bands, rear-knee ratios unchanged, h2 decodes a few points WORSE.

*Arm 2 — latent-autoregression context (the plain EPM's raw latent), the
pre-registered fail branch.* Absorbs more (dp_err 0.10–0.12, ~90%), and
the residual vocabulary COLLAPSES 41 → 25 tokens; chain lift 1.00 flat;
h2 bands still absent; h2 decodes still worse.  ONE genuine positive:
**the rear-knee marginals (j10/j11 — the operator's named planting target)
improve 3–8 points at k∈[13,21] in ALL FOUR seeds** (e.g. s13: 0.76/0.77
vs control 0.83/0.85) — the latent-AR residual carries real rear-knee
correction structure; the near field (k 1–3) degrades and the bands'
0.95-contiguity threshold hides the gain.

**Verdict: B v1 is a NULL against its pre-registered gates — but a §3.2
review says the measured object was partly the HARNESS AGAIN: the
subtraction happens in latent space, so the GNG receives a residual of
norm ~0.1–0.2 against insertion/error scales sized for encoder outputs of
norm ~1.  The vocabulary collapse under the STRONGER predictor (arm 2,
41→25) is §6's insertion-gate collapse in a new costume — the residual's
SHAPE is never tiled because its SIZE is below the gate's resolution.
Doctrine §5 rule 5 prescribes the fix: adapt, don't tune — a running-RMS
normalization of the post-subtraction residual before the GNG (an EPM
option, off by default), so the vocabulary tiles residual DIRECTION at
unit scale.  Re-use context: (i) B v2 = residual normalization + re-run
both context arms (the rear-knee k∈[13,21] signal is the thing to watch
grow); (ii) if v2 still nulls, the linear predictor family is exhausted —
phase-binned piecewise-linear or the planner-as-predictor are the next
rungs; (iii) the rear-knee planting target does NOT wait on B: it can get
its own behavioral lever (stance-gated knee-flexion-at-touchdown through
the promoted plan-pull carrier) regardless of vocabulary work.**

**2026-08-14 — REAR-KNEE PLANTING lever, arm 1 (constant descent extension):
REGRESSION — and the §3.2-rule-6 catch on my own build.** The operator's
named target built as the knee half of swing-descent (`swing_descend_knee`:
past half the leg's own swing, the shank flips from fold to extend).  n=6
corridor vs the promoted stack: net_z 7.24→5.15/4.73 (0.3/0.6), cv
0.75→0.87/0.89, falls 0→3/1, planted DROPS 3.54→3.36/3.46 — the constant
form pays the hip2-press's rhythm cost AND loses transport: extending
through EVERY descent stabs healthy swings; the 2026-08-10 "knee keeps its
fold" was load-bearing.  **Faithfulness check on myself: the constant form
is a WEAKENED SLICE of the stated design** ("contact expected by now, none
arrived").  Built the error-form (`swing_overdue_knee`): extension fires
ONLY when a swing outlives the leg's own running-average duration, grows
with lateness, releases on contact — healthy swings untouched BY
CONSTRUCTION.  Verdict on arm 1: REGRESSION (recorded, kept as the arm);
arm 2 (overdue form, 0.4) in flight.  Re-use context for arm 1: none
foreseen — the mechanism class is superseded by the error-form.

**2026-08-14 — PLANTING arm 2 (the overdue error-form) + B v2 (residual
normalization): one PARTIAL with a named ratchet, one HISTORIC structural
pass with a starved planner.**

*swing_overdue_knee 0.4 (n=6): PARTIAL.* Five of six seeds hold transport
(7.0–8.8 vs control 7.24); swing bouts shorten 8.29→7.68 exactly as
designed (overdue swings get terminated by the reach); but cv 0.75→0.85,
falls 0→3 (two seeds), and the consumer counter exposes the flaw: 4,575
overdue leg-ticks — **the expectation RATCHETS.** The reach shortens
swings; completed-swing durations feed the running average; the average
falls; more swings read as overdue.  The intervention chases its own
reference.  Re-use context (the fix, one gate): learn the expected
duration ONLY from unassisted swings (skip the EMA update when the reach
fired that swing).  Operator eye pending on the planting quality itself.

*B v2 (normalize_residual, both context arms, n=4 × 12k arena):* **The
structural gates PASS for the first time in the campaign's history** —
vocab 25 → 235/246 in-use, entropy 5.1 nats, and self-transition mass
falls to **0.51 (clock ctx) / 0.32 (latent-AR ctx)** against the pose
vocabulary's 0.69–0.72 and the gate's < 0.55: the token stream finally
CARVES THE STRIDE, the target M0 set on 2026-08-09.  The collapse fix
did exactly what the §6 diagnosis said it would.  **But the planner
gates still FAIL:** the vocabulary slams into the 200-node cap with ZERO
baked (≈50 visits/token at 12k — below the baking threshold's
statistics), the chain scores BELOW the now-weaker persistence baseline
(0.77–0.94), h2 bands appear only scattered (2/4 seeds, narrow), and the
v1 rear-knee gain is GONE (pc ties-to-worse vs control) — the M0.d.2
lesson verbatim: finer tiling without transition statistics is per-token
sample starvation.  **Verdict: B v2 PARTIAL — the substrate condition is
finally met; the planner layer is data-starved, not refuted.  Re-use
context / v2.1: raise max_nodes (cap-hit is §0 rule 4's diagnosis),
lengthen runs (24k+) so tokens earn visits and bakes, and/or coarsen the
residual tiling; re-judge the planner gates only when baked > 0 and the
per-token count matches the bodypose reference's.**

**2026-08-14 — RATCHET-FIXED OVERDUE KNEE (arm 3) + B v2.1 (fed vocabulary):
the planting family reaches the operator's-eye gate; the residual planner
nulls again with a NEW mechanism named.**

*swing_overdue_knee 0.4 with the unassisted-reference fix (n=6): the
GAIT'S CHARACTER CHANGES.* Robustness is the headline: net_z σ COLLAPSES
1.33→0.63 (every seed 5.4–7.4, zero falls), **planted 3.54→3.67 — the
planting metric's first move in the family**, straight holds.  The costs:
steps 270→167 (cadence −38%), cv 0.89.  And the consumer count ROSE to
9,305: excluding assisted swings from training exposes the REVERSE
entanglement — the reference cannot track a legitimate gait slow-down, so
lengthening swings read as perpetually overdue.  The result is fewer,
longer, more-planted strides.  **Verdict: PARTIAL, ESCALATED TO THE
OPERATOR'S EYE — aggregate metrics cannot distinguish 'deliberate planted
gait' from 'sluggish gait' (a blind-metric situation by construction).
`swing_overdue_knee` added to the [M] motor-panel sliders for live
judgment.  Re-use context: if the eye says 'planted', the cadence cost is
the trade to tune; if 'sluggish', the reference needs a two-timescale form
(slow tracking of ALL swings + fast exclusion of assisted ones).**

*B v2.1 (latent-AR + norm, cap 800, 24k, n=4): the vocabulary is healthy
and the planner nulls AGAIN — with the cause visible.* Nodes 274–285
(below cap, pruning live), in-use 453, self-mass **0.27** (the deepest
carve yet) — but **baked = 0 even at 24k with ~85 visits/node**, and the
answer is structural: the predictor NEVER STOPS LEARNING (lr 0.01, no
freeze), so the residual distribution drifts under the GNG forever — a
substrate chasing itself cannot stabilize enough to bake.  Planner gates:
rear-knee ratios TIE control, rk bands narrower, h2 absent, chain ≈
persistence, authority 0.  **Verdict: NULL for the planner layer at every
tested configuration (v1/v2/v2.1); the pre-registered next knob costs
zero code — `freeze_after_ticks` on pc_predictor (converge, then freeze →
stationary residual space → the GNG can finally bake).  The B program's
structural achievement stands: self-mass 0.72 → 0.27 across the campaign;
the planner value question stays open pending the freeze arm.**

*B DESIGN (IN_FLIGHT, starts after A's verdicts): the SURPRISE VOCABULARY.*
The audit answer again — the pieces exist, hand-roll only scorers: EPM
already implements descending-prediction subtraction (`gng_input =
encode(obs) − predicted_latent`, unit-tested), and `DescendingPredictor`
(AR(1): W·context + b per target, online SGD) has existed since Phase 1
with no motor-path consumer.  Wiring: `body_pose_pc` EPM over
`reality.proprio.joints` (subtract_descending_prediction=true) + a
DescendingPredictor whose CONTEXT is the CPG clock [cosφ,sinφ] (§0 rule 3
— phase is the load-bearing context), so the subtracted term is the
stride's phase-expected pose and the GNG tiles DEVIATION-FROM-RHYTHM — the
corrections the operator sees as h2 noise become the densest, most
predictable token regions instead of invisible residue.  Shadow
MotorPlanner over the pc vocabulary beside the bodypose control (the twin
protocol).  Gates, M0.d-style: vocabulary self-limits; self-transition
mass falls materially below 0.55; per-joint bands extend below k=8 or past
k=34; h2 tracks earn FIRST bands; decode sharpens (fade-1 embodied
playback is the operator's blur test).  Fail ⇒ context arm swap (own-latent
AR(1) vs clock) before any verdict on the predictive-coding path itself.
