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
  set; `CHASSIS_COLLIDE=1` always. Report corridor diff 0.3 and flat (diff 0);
  arena spot-checks for gym-transfer claims (nav oracle engages ONLY in the arena —
  account for it).
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

## Explicitly out of scope

L1 nav / EFE arbiter layers; new reflex levers on the unrepaired substrate; ratchet-leak
redesign (documented, deferred — revisit before hardware); promotion of anything without
its UI review.
