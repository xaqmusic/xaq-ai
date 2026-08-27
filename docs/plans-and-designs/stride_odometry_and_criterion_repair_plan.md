# PART V — Stride Odometry: making the criterion point at locomotion

**Drafted 2026-08-25, at the close of PART IV.** Operator-agreed direction (design
discussion 2026-08-25); the fork decisions in §6 remain theirs. Companion documents:
`picrawler_part4_status.md` (where the criterion stands), `picrawler_part4_audit.md`
(the audit + the pre-registered decision experiment), and the ledger's closing
2026-08-24 entries (the evidence this plan is built on).

---

## 1. The position

PART IV ended on a sentence this plan exists to falsify: **the search follows its
criterion; the criterion does not yet point at locomotion.**

The evidence, all measured:

- **Nothing in J measures translation.** The flow term — included as the counterweight
  that stops the search minimizing everything by standing still — is a gait-amplitude
  proxy (r = +0.97 with amplitude, +0.26 with `|fwd_v|`): its magnitude factor saturates
  above 0.05 m/s, so its variance is all velocity-volatility, which tracks amplitude.
- **The heaviest term is a "move less" pressure.** Energy (weight 4, more than everything
  else combined) reads `joint_torque`, which is Kd-damping dominated (corr(τ, dθ)
  −0.46..−0.56; Kd·ω ≈ 24 vs Kp·err ≈ 3.6) — largely mean joint speed.
- **The consequence is a wrong-way basin.** The decision experiment (pre-registered,
  n=6/arm, true displaced control): a coupling displaced to 0.30 descends to coupling≈0
  in 7 of 12 searching runs, energy improving while flow pays. The two runs that climbed
  to the band instead posted the experiment's two best ΔJ with *both* terms improving.
- **The one speed-flavored input dies at the port anyway.** `fwd_v` is a soft oracle
  (`_chassis.linear_velocity`), and only 1 of 7 criterion weight units is buildable on
  the physical robot today.

The rewrite rule applies to criteria as much as to behaviors: the missing piece is an
**observation**, not a cleverer evaluator. No re-weighting of the current terms can make
J prefer travel, because no current term carries travel.

## 2. The design — efference-copy odometry, vestibular-checked

The biology states the design. Animals have no speedometer; they estimate self-motion
from **efference copy plus limb re-afference** — how the stance limbs' geometry actually
swept — checked against the **vestibular** channel, and the *mismatch* between the two is
itself a first-class percept (a cat stepping on ice knows instantly, not from vision).
Both halves are already wired on this robot:

| half | signal | error profile |
|---|---|---|
| **stance-leg FK** | commanded joint angles of *planted* legs (planted = `foot_load` over threshold — the FSR-legal contact) imply body translation over the stance polygon | bias-free over short horizons; **over-reports when feet slip** (a foot sliding back in body frame is indistinguishable from the body moving forward) |
| **IMU** | accelerometer + gyro | catches slip, but integration drift is *bias*, not noise — a 1° attitude error leaks false velocity exceeding the true signal within ~1 s, gait-synchronously (hardware-audit entry, 2026-08-24) |

The profiles are exactly complementary, and **the disagreement between them IS the
post-plant-slip term** deferred from GainEvolver v1 ("no egocentric slip signal exists in
the codebase"). One fusion therefore delivers three things at once: a legal `vel_ego`,
the flow term's missing travel magnitude, and the deferred slip percept.

**Architectural check (CLAUDE.md §0 — EPM first, always argued):** this is a
*transparent sensor reduction* — a derivation of a physical quantity from existing
channels — not a coarse-graining. There is no vocabulary to earn, no novelty to
topologize, no per-token error to carry: the output is a continuous egocentric scalar
(later, if a *map* over travel states is wanted, an EPM sits naturally downstream of it).
Publishing it as `reality.proprio.*` is the same move as `foot_load` and `joint_load`
— a named bootstrap sensor, not a scaffold to remove.

**Why the cheap alternatives lose, recorded so they stay lost:**
- *Stride/touchdown counting*: rewards stepping in place — a blind metric. FK odometry
  over stance feet reads ~zero for stepping in place, which is exactly the property wanted.
- *Leaning harder on `fwd_v`*: deepens the oracle, still saturates, still dies at the port.
- *God's-eye displacement*: prohibited outright (§5 hard prohibitions). God's-eye truth
  appears in this plan only as a **measurement instrument** in stage A, never as a brain
  input.

## 3. The stages — measurement first, one lever at a time

### Stage A — measure the sensor before building it (~a day)

The horizontal commanded-vs-achieved FK error has **never been measured** — only foot
*height* (22 mm mean). Instrument the body to log FK-implied horizontal velocity beside
true velocity (god's-eye as instrument only) and run seed-averaged sweeps across gaits
and difficulties.

**Gate A (promote-or-kill):** stance-phase FK velocity correlates with true forward
velocity at r ≥ ~0.8, and the slip residual is *structured* (gait-phase-locked), not
white. Fail ⇒ FK odometry on this body is dead cheaply, before any build — and that is
worth knowing before hardware FSRs are ordered. Both outcomes go to the ledger.

> **RUN 2026-08-25 — `PARTIAL`, promoted to stage B** (ledger entry of the same date has
> the full numbers). r = 0.74–0.79 at 1 s windows across gaits and difficulties, phase
> lock 6–10× null. Two design facts the gate paid for: the efference copy works **only
> through a first-order servo forward model** (raw commanded FK is dead, r ≈ 0), and the
> stance rule is `foot_load ≥ ~0.2` (a plateau; G2's 0.05 costs ~0.15 of r). Median-of-
> legs consensus refuted in this context. Scale: FK travel = a stable 73–78% of truth.
> Stepping-in-place immunity demonstrated (7× separation at equal step amplitude).

### Stage B — build the sensor (small, body-level)

Publish `reality.proprio.stride_v` (stance-FK velocity estimate, IMU-fused per the
complementary-filter shape gate A justifies) and `reality.proprio.slip` (the FK/IMU
disagreement). Registration strings carry the legality argument, per house convention.
Default-no-consumer: with nothing subscribed the build is behavior-identical.

**Gate B:** the two-sided consumer check once anything subscribes (publisher meter +
consumer meter in the body log), and `stride_v` ≈ 0 while stepping in place — the
blind-metric immunity demonstrated, not assumed.

> **RUN 2026-08-25 — `WORKING`, shipped** (ledger entry of the same date). Published
> `stride_v` r_w50 0.71/0.79 vs truth; PI bias estimator required (P-only fusion crushed
> the mean to 0.18 of truth); β=1.0 chosen by the criterion's own timescale, β≈0.3's
> fast-band win recorded as re-use context. No-consumer guard measured: 7900/7900 ticks
> byte-identical vs the pre-sensor build. Consumer-side meter arms at stage C.

### Stage C — re-point the criterion (mostly configuration; the machinery exists)

PART IV already built the repaired flow term and shipped it OFF, each with a recorded
re-use context that is *exactly this stage*:

1. **Lever C1 — the term swap.** New ConstructionOnly `travel_topic` on the GainEvolver,
   defaulting to the current input (byte-identical, gain-0 shape). Point it at
   `stride_v`; enable `flow_min_form = 1` (its ledgered context: *"becomes live the
   moment a LEGAL travel signal replaces fwd_v"* — magnitude can no longer be traded for
   stillness-bought predictability) and `flow_turn_k > 0` (a tight circle reads as travel
   to any body-frame magnitude; the gyro factor is built, unit-pinned, never live).
2. **Lever C2 — energy's fate, decided by measurement, not taste.** Re-measure the term
   budget on real windows with the gate-analyze method (noise from revert pairs;
   **variance share, never magnitude**; the method that overturned intuition three times
   out of three in gate 2). Candidates: de-weight from 4.0, or re-point `torque_topic` at
   `joint_load` (the honest load proxy — measured NOT to discriminate stance from swing,
   0.383 vs 0.257, so it is not a drop-in and must earn its weight on the measured
   budget). A slip term entering J is also a candidate here — measured first, weighted
   after, or shipped inert like dwell was.

One lever per A/B. C1 and C2 are separate arms, never combined in one comparison.

### Stage D — re-validate on the harness PART IV built

1. **Landscape re-sweep at j1s4** under the new criterion (`gainevo_landscape.py`).
   **The concrete success test: the coupling→0 basin flattens or the band basin
   dominates.** Also read what the new criterion says about amp — under a real travel
   magnitude it should stop preferring the quiet end of amp's range for free.

> **D1 PRE-REGISTRATION (2026-08-25, committed before the runs).** Protocol: the
> coupling-authority shape exactly — σ=0 observer, SETPARAM_AT steps
> `motor_epm.coupling_gain` through [0.0, 0.4, 0.8, 1.2, 1.6, 2.0], 3 scored 12k
> windows per level, warmup 10000; seeds 1–6, odd ascending, even descending. Two
> arms, same seeds: `j1s4` (old criterion, the control) and `j1s4_c1` (travel_topic =
> stride_v, flow_min_form 1, flow_turn_k 4). Bodies are behavior-identical across
> arms (measured, C1 commit), so any J difference is the criterion alone.
> **Predictions:** (P1) the control replicates PART IV's wrong-way basin — J improves
> toward coupling 0 (energy-led). (P2) under C1 the coupling→0 basin flattens or
> inverts: mean J at level 0.0 no longer beats the band (1.2–1.6), because fmag now
> pays for the travel collapse (the c0 config measured stride_v 0.006 vs 0.021 m/s).
> **Decision rule:** C1 passes D1 if P1 holds AND under C1 J(0.0) − J(1.2..1.6-best)
> ≥ 0, judged on seed means with the asc/desc hysteresis check from the analyzer.
> If P1 fails, the control is invalid and NOTHING about C1 is concluded (§3.2 #4).
> If P1 holds and P2 fails, the wrong-way basin survives a real travel term and the
> next suspect is the term BALANCE on the stage-C2 budget, as chartered.
>
> **RESULT 2026-08-25 — P1 HELD, P2 FAILED; the fail-branch fired** (ledger D1 entry).
> The flow repair works (its zero-vs-band penalty gap widened ~50%) but energy at w=4
> outvotes it. The C2 budget on the same corpus: energy 55% of decision variance at
> s/n 0.76; repaired flow the cleanest term (s/n 1.11) at 8.7%. Offline recomputation
> over the 108 windows: `w_energy 4→1` + `w_flow 1→2` flips the basin (5/6 seeds,
> +0.206 margin; coupling 0 becomes the WORST point). Slip measured term-grade
> (s/n 1.10) but travel-linked (rises 0.124→0.144 with coupling) — stays OUT of J.

> **D2 PRE-REGISTRATION (2026-08-25, committed before the runs).** Protocol: tsw2
> exactly (`gainevo_make_arms.py d2` — the tsweep arms with the C2 criterion applied:
> travel_topic = stride_v, flow_min_form 1, flow_turn_k 4, w_flow 2, w_energy 1).
> Arena, difficulty 0.3, 600k ticks, arms sigma0 (displaced control, start written
> into BOTH evolver seed and MotorEPM params) / fix008 / fix020, OGMA_SEED 1–6.
> **Predictions:** (P1) sigma0 controls stay near the displaced coupling 0.30 with
> flat J. (P2, the phase's headline gate as chartered) a searching arm re-enters
> coupling 1.2–2.0 in the MAJORITY of its runs with ΔJ below the control's.
> **Secondary registered read:** under the C2 criterion the landscape argmin measured
> 0.8, so runs that climb out of the zero basin to ≥ 0.8 without crossing 1.2 are
> reported as *directional* success, distinctly — they would satisfy the criterion
> while missing the chartered band. **Decision rule:** gate D passes on P2 for either
> searching arm. If controls also climb, the protocol is invalid (consumer-displacement
> confound). If no arm climbs while D1's offline flip predicted the pressure exists,
> the suspect becomes search dynamics (step size / margin), NOT the criterion — σ=0.20's
> recorded re-use context applies.
>
> **RESULT 2026-08-26 — `PARTIAL`, one run short of the endpoint majority, and the
> zero basin is ELIMINATED** (ledger gate-D entry). Control valid (0.30 in 6/6).
> fix020: ΔJ −0.117 (t −2.9, the J half passes decisively), endpoint re-entry 3/6;
> by trajectory 6/6 entered the band, and the two runs that visited coupling ≈ 0
> mid-run both escaped and ended in-band — under the old criterion 7/12 runs ended
> there. The binding residual is the flat band (criterion resolution, §6 out of
> scope), not direction. The C2 criterion is promoted as the reference criterion for
> successor-phase work; the deployed j1s4 stack is untouched.

2. **Re-arm the displacement protocol** — the tsw2 shape exactly: displaced start written
   into evolver seed *and* consumer params, σ=0 control, n=6, pre-registered predictions
   and decision rule committed before the data (doctrine §8: that pre-registration is
   what kept PART IV's wrong prediction from bending its verdict).

**Gate D (the phase's headline):** a displaced `coupling_gain` re-enters 1.2–2.0 in the
majority of runs with ΔJ below the control's. Pass ⇒ *the criterion points at locomotion
and the search climbs it* — PART IV's machinery vindicated end to end. Fail ⇒ the basin
survives a real travel term, and the next suspect is the term *balance*, measured on the
stage-C budget.

### Stage E (added 2026-08-26, operator-directed) — the TWO-BODY study

The embodiment capstone: same criterion (C2), same seed streams, same coupling
displacement, on `cad` (the body as designed — every prior ledger result) and
`measured` (the body as built: hip_z_span +15%, l1 −14%, l2 −10%, chassis mass +14%,
real CoM). If the discovered gains are properties of the body, each body's search must
land in a basin matching its OWN landscape. Tool: `strido_twobody.py`; PART IV's basin
sweep ran 2 bodies but pre-dated a locomotion-tied criterion, which is why it returns
now.

> **E1 PRE-REGISTRATION (2026-08-26, before the runs).** Landscape per body: the D1
> protocol in the ARENA (shared with the search phase), σ=0 on `j1s4_c2`, coupling
> stepped 0→2, seeds 1–6 odd-asc/even-desc. These are conditional landscapes at the
> shared j1s4 operating point (tuned on cad) — exactly E2's start. **Gate to E2:**
> some level's between-body |ΔJ| exceeds 2× the pooled same-level window noise, or
> the argmins sit ≥ 2 levels apart. Fail ⇒ the search comparison is unpowered by
> construction: stop, report, consider an amplified third body.
> **E2 (registered in outline; exact cross-predictions filled from E1 numbers BEFORE
> E2 launches):** sigma0 + fix020 × seeds 1–6 per body (fix008 dropped — too slow to
> climb, twice). The claim under test: paired by seed, each body's endpoints match
> its OWN E1 landscape better than the other body's — falsifiable in both directions,
> robust to flat-band endpoint wander.
>
> **E1 RESULT (2026-08-26): GATE PASS, 4/6 levels beyond 2× noise.** cad: a sculpted
> basin, monotone descent to argmin 1.6 (J 1.759 vs 2.167 at zero). measured: nearly
> flat across 0.4–1.2 (argmin 1.2 at 2.135, tied within noise with 0.4/0.8), and
> RISING where cad's optimum sits (1.6: +0.44 vs cad, the largest between-body gap).
> The bodies prefer different regions. Logs `~/xaq_runs/twobodyE1_20260826/`.
>
> **E2 EXACT PRE-REGISTRATION (2026-08-26, committed before the measured-body runs).**
> The cad arm is the D2 corpus REUSED (identical configs/seeds/gym/body/criterion;
> the sim is deterministic — a re-run would reproduce the logs byte-for-byte), so
> cad's numbers are post-diction and are cited as-is: fix020 endpoints
> [0.41, 0.76, 1.17, 1.22, 1.81, 2.00], mean 1.23, 3/6 ≥ 1.2, ΔJ −0.117 (t −2.9).
> Fresh runs: measured body only, sigma0 + fix020 × seeds 1–6.
> **Predictions:** (P1) measured sigma0 controls hold coupling 0.30, J flat. (P1b)
> measured fix020's ΔJ beats its own control (the search descends on a body nothing
> was tuned for — the mechanism-transfers claim). (P2) measured fix020 endpoints
> concentrate in its own flat preferred region: mean endpoint BELOW cad's 1.23, and
> fewer runs ending ≥ 1.6 than cad's 2/6. (P3, the headline contrast) cad mean
> endpoint > measured mean endpoint, one-sided. (P4, secondary) own-landscape excess
> J at endpoint (J_own(end) − min J_own) averaged over a body's runs is smaller than
> its cross-landscape excess — the cross-prediction as one number; noted weak on the
> measured side by its own flatness. Falsification of P2/P3 = the endpoints do NOT
> track the body's own landscape — the settling is criterion- or search-generic, and
> the embodiment claim fails at this power.

> **E2 RESULT (2026-08-26, ledger stage-E entry):** P1 ✓, P1b ✗ (no ΔJ descent on
> measured — its flat landscape gives the search nothing), P2 partial, P3 ✓
> directionally (underpowered), P4 ✗ with the metric lesson recorded. The strong
> settling claim is not demonstrated at this power; E1 + the climb/diffuse contrast
> carry the embodiment story. Unregistered loud finding: falls medians 2/17/42/210
> (cad-σ0/cad-search/meas-σ0/meas-search) — viability guards do NOT transfer across
> bodies; re-earn guards on the physical body before the search may move (port-plan
> requirement).

### Stage E3 (added 2026-08-26, operator-directed) — native operating points, then displace

The re-use context of the E2 verdict, run: give each body its OWN operating point
first, then displace both from their own points — the settling demo becomes
well-posed because both bodies then have a basin to return to. The settled points
also ship as **permanent UI benchmark configs** (operator-requested, for demos),
carried body-and-gains-together via new `metadata.body` support in the launcher chain.

> **E3a PRE-REGISTRATION (2026-08-26, before the runs).** Settle protocol: free
> search at σ 0.2 pinned (the step size that climbs), C2 criterion, from the NATIVE
> j1s4 3-gain point [amp 0.385, coupling 1.655, postural 1.092] — generation 0 is
> the deployed stack's body on cad. Arena 0.3, 600k ticks, both bodies × seeds 1–6.
> **Selection rule (registered before any run):** per body, the benchmark vector is
> the final incumbent of the run with the best last-third mean J among runs whose
> falls are ≤ that arm's median (viability filter first, quality second); all six
> endpoints and falls go to the ledger beside the selected one. E3a is exploratory
> (settling, not a hypothesis test); E3b's predictions are registered from its
> numbers before E3b launches.
>
> **E3a RESULT (2026-08-26):** cad native = [amp 0.1857, coupling 0.8781, postural
> 1.2623] (settle_s6: J3 1.765, 2 falls — the arm's cleanest run); measured native =
> [0.3075, 0.9197, 0.8516] (settle_s3: J3 1.993, 383 falls while searching; the
> measured arm's search stayed fall-heavy, median 405 — the stage-E viability finding
> again). The natives separate in AMP and POSTURAL, barely in coupling — and cad's
> joint optimum (coupling 0.88, amp 0.19) does not match its E1 conditional slice
> (argmin 1.6 at j1s4 amp/postural): non-composition, measured on the joint optimum.
> cad shows a second good mode at coupling ~2.0 (s3/s5). Both natives ship as
> permanent launcher benchmarks (`native_cad` / `native_measured`, metadata.body).
>
> **E3b PRE-REGISTRATION (2026-08-26, committed before the runs).** Per body, three
> arms, arena 0.3, 600k: `native-σ0` n=3 (the benchmark config frozen at the native
> point — characterizes the benchmark and anchors the recovery target),
> `displaced-σ0` n=3 (native with coupling → 0.30 in both evolver seed and MotorEPM
> params), `displaced-fix020` n=6 (same start, σ 0.2 pinned). **Validity condition:**
> displaced-σ0's last-third J must be WORSE than native-σ0's (the displacement must
> cost something, or there is nothing to recover — else report and stop).
> **Predictions:** (P1) displaced-σ0 holds coupling 0.30. (P2, per body) fix020
> re-climbs: ≥ 4/6 runs end coupling ≥ 0.6 AND arm ΔJ beats displaced-σ0's.
> (P3, recovery) fix020's last-third J recovers the majority of the displacement
> cost: (J3_fix020 − J3_native) < 0.5 × (J3_displaced − J3_native), per body on arm
> means. The two bodies' recovered (amp, postural) endpoints separating toward their
> own natives is reported DESCRIPTIVELY, not as a registered pass (the start is
> body-specific, so a cross-body preference test would be biased by construction —
> the common-start version of that test is the already-run D2/E2 pair).

## 4. The sim2real dividend

This phase is also the port plan. With `stride_v` (needs no new hardware — commanded
angles + the already-wired IMU), the INA219 on the servo rail (energy as *real* current),
and 4 foot FSRs (`foot_load` + the G2 guard + stance truth for odometry bring-up), the
criterion goes from **1 of 7** hardware-buildable weight units to **~6 of 7**, and both
guards become real. Every criterion input this plan adds is a sensor the physical
picrawler can carry for ~$20.

## 5. Compliance (the standing prohibitions, checked)

- **No reward shaping** — J stays error-form: flow measures *failure of intended
  translation to materialize*, slip measures re-afference mismatch. Nothing rewards raw
  speed.
- **Egocentric only** — every new input derives from commanded angles, foot load, IMU.
  God's-eye appears only as a stage-A measurement instrument, named as such.
- **No teacher distillation** — `stride_v` is a derivation, not a learned copy; nothing
  is trained against the oracle.
- **EPM-first argued, not skipped** — §2: sensor reduction, no vocabulary to earn.
- **Gain-0 guards** — `travel_topic` defaults to the current input; `flow_min_form`,
  `flow_turn_k`, and any slip weight ship at their current OFF values until their arm runs.
- **Nothing disabled** — the criterion keeps every current term until stage-C measurement
  says otherwise; the search loop is untouched (§6).

## 6. Out of scope, with re-use contexts (the operator's forks)

- **The accept/anneal loop.** Untouched this phase. `sigma_min` stays 0.08 per the
  decision experiment's registered rule; σ=0.20's faster descent and sole band re-entry
  is its recorded re-use context — *retry larger steps after the criterion is repaired*.
  Anneal-on-realized-ΔJ likewise waits until there is a landscape worth annealing on.
- **The 8-D vector.** The rear trio, height_homeo and plan_gain stay hand knobs. The
  audit found two of them consumer-gated (height_homeo multiplied by ~0 while walking;
  rear_knee_plant dead when rear_land ≈ 0) — re-admitting them is a consumer-side
  question, not a criterion one.
- **The distress sensor repair** (world-frame stall half + the 50-vs-240 Hz bug) —
  separate lever; `w_distress` keeps its parking space at 0.
- **A slow EPM over travel states** (the map): natural *after* `stride_v` exists;
  explicitly not this phase.

## 7. Sequencing and cost

A (measure, ~1 day) → B (small publisher) → C1 → C2 (each a seed-averaged A/B on the
existing harness) → D (landscape + displacement, the tools as committed). Total compute
is dominated by stage D, which is the tsw2 protocol PART IV already paid to debug. The
main risk is gate A failing on unstructured slip — which would itself be a finding worth
a ledger star, and it is the cheapest gate in the plan by an order of magnitude.
