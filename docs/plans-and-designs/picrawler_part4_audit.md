# PART IV — GainEvolver: the pre-report audit

**Working document, not a report** — same standing as `picrawler_part4_status.md`, whose §5
audit brief this answers. Audited in fresh context on 2026-08-24: the module
(`cpp_core/src/ogma/modules/GainEvolver.cpp`), its consumer path (`MotorEPMv2.cpp`), the
body's sensor publish sites (`picrawler_body.gd`), all 33 unit tests, the five analyzers,
the arm generator, and the three shipped configs, against the ledger's campaign record.

**The audit's verdict: the status doc's headline survives, sharpened.** "The criterion
carries signal; the search cannot climb it" becomes *"the search cannot climb it **at the
step size its own anneal selects** — and the campaign's own measured constants predict
exactly that."* The failure is arithmetic, not mystery, and it names a cheap decisive
experiment (§4.1) that should run before the report is written, because its outcome decides
which of two very different stories the report tells.

---

## 1. Why the search cannot climb — derived from the campaign's own numbers

Four measured constants, all already in the ledger:

- per-window noise `σ̂` = **0.168** (4000-tick window; 0.125 at 12000) — winverify
- coupling slope `|dJ/dg|` = **0.182 /unit** — step-0 authority check
- accept margin = `accept_k·σ̂` = 0.25 × 0.168 = **0.042**
- the anneal settles σ at **0.08 = σ_min** in every AUTO arm — tsweep

A single comparison is `J_cand − J_inc`, sd = √2·σ̂ = 0.238. A coupling mutation at σ = 0.08
moves the gain by ~0.16 units (σ × range 2.0), so |ΔJ| ≈ 0.029. Then:

| | P(accept) |
|---|---|
| beneficial step | Φ((0.029 − 0.042)/0.238) ≈ **0.48** |
| harmful step | Φ((−0.029 − 0.042)/0.238) ≈ **0.38** |

**The selection differential per generation is ~0.09.** Expected drift toward the optimum
≈ 0.16 × 0.09/2 ≈ **0.008 units/generation**; the displacement test demands 0.9 units in
28–35 generations. Predicted travel ≈ 0.2 units against a diffusion sd of ≈ 0.55. (Order-of
-magnitude — the slope is region-dependent — but the margin is 4×.)

**So C5, C6 and C10 were arithmetically inevitable at σ = 0.08.** The basin diffusion, the
3-D null, and "searching is indistinguishable from not searching" are all the same fact: at
this step size the landscape's slope is invisible to a single-window comparison, and no
window length fixes it (C7 measured that; the margin shrinks with σ̂, but so does nothing
else — per tick, longer windows buy √-noise while costing generations linearly).

The same arithmetic at **σ = 0.20**: |ΔJ| ≈ 0.073, accept 0.55 vs 0.31, drift ≈ 0.047
units/gen → **1.3 units in 28 generations — enough to recover the band.** And the record
agrees: `fix020` is the only arm in the whole campaign that ever re-entered coupling's good
band (n=1, ΔJ +0.122 — it paid for recovery by moving `amp_target`). **The n=6 rerun that
produced C10 did not include fix020** — it compared sigma0/auto/fix008, i.e. only σ = 0.08.

Two module-level facts complete the causal chain:

1. **The anneal parks σ at σ_min.** With `target_accept` AUTO the setpoint is the pure-noise
   floor (0.430 at k=0.25), but G1/G2 rejections plus landscape flatness hold realized
   acceptance at 0.29–0.40 — permanently *below* the setpoint — so `rate > target` almost
   never fires and σ decays to `sigma_min` and stays (`GainEvolver.cpp:641-643`). The anneal
   is not merely contributing nothing (the C-series read); it actively selects the least
   powerful step size the config allows.
2. **σ_min violates the module's own recorded bound.** The `accept_k` schema note derives
   the step-matching inequality — σ ≥ accept_k·σ̂/(range·|slope|) ≈ **0.115** at the 4000
   window (0.086 at 12000) — and that is only *parity* (margin = signal, 50% power). The
   same fix series set `sigma_min = 0.08`, below its own bound, and nothing folded the
   inequality back into the floor or the configs.

---

## 2. New findings (not in the status doc)

### 2.1 ⚠ C10's σ=0 control ran a different body than intended

At σ = 0 the module is a silent observer and **publishes nothing**
(`GainEvolver.cpp:650` — `start_window` publishes only when `sigma_ > 0`). The consumer
therefore runs its **own config values**, and the tsweep base config's MotorEPMv2 carries
the j1s4 point — `coupling_gain` **1.655**, not the displaced 0.30. `gainevo_make_arms.py`
writes `TSWEEP_START` only into the GainEvolver's `gain_seed`, never into the MotorEPM
params, so the "no-search control" was the *good* operating point while every searching arm
started displaced.

The ledger's line "`sigma0` holds coupling at 0.30 in all six runs" read `ge_vec` — the
module's internal, never-published incumbent. That is the instrument reporting its intent,
not the body's state.

**What survives:** the searching arms' own null stands on its own — ΔJ ≈ −0.03 where
recovery predicts ≈ −0.16, and band re-entry 1/12. **What does not:** the specific claim
"J does not drift on its own after a displaced start." Drift was measured at the wrong
point. Fix in §4.2; the control must be re-run before the report cites it.

### 2.2 C12 is measured, not documentary — upgrade it

The status doc lists the energy-term diagnosis as "documentary; not yet measured here." The
measurement exists: **2026-08-06, per-tick, corr(τ, dθ) = −0.46..−0.56 on all three
joints**, with the decomposition Kd·ω ≈ 24 vs Kp·err ≈ 3.6
(`picrawler_body.gd:1056-1070`). The publish site confirms the wiring: `joint_torque`
publishes `_prev_torque_*` — the PD/damping value — at `picrawler_body.gd:6754-6757`, while
the honest load proxy (`joint_load`, velocity-tracking deficit) is published on the next
lines and is unconsumed. The report can cite C12 as measured. The only unmeasured remnant
is the window-level corr of mean|τ| against mean joint speed — a confirmation, not a gap.

### 2.3 Two "invisible" gains are consumer-gated, not criterion-blind

- `height_homeo_gain`'s expression is multiplied by `height_rest_frac_` ≈ 0 **whenever the
  body is walking** — on both the integrator and the output term
  (`MotorEPMv2.cpp:2535, 2619-2626, 3696-3700`). Cruising windows cannot see this gene by
  construction.
- `rear_knee_plant` is only read inside the `rear_land_gain > 0` branch
  (`MotorEPMv2.cpp:3849`) — an epistatic dead zone. A random-start basin run that wanders
  `rear_land_gain` near 0 disconnects the gene entirely, and mutations to it still count as
  "applied."

This *strengthens* the authority cross-check (flat gains have a mechanism for being flat)
and *redirects* any plan to "extend the criterion to see them" — the body does not express
them in the measured regime, so the criterion is not the place to look.

### 2.4 The verification tooling has three defects as committed

1. **`gainevo_basin.py` crashes** — `GOOD_BAND` and `AUTHORITY` (lines 117/130/143) were
   deleted along with the old bounds loader in f35db7e. `NameError` on any data. **C4, the
   campaign's best positive, is currently non-reproducible**, and the run logs were
   ephemeral, so the recorded numbers cannot be re-derived at all until this is fixed and
   the study re-run.
2. **`gainevo_tsweep.py` never analyzes the control** — `ARMS` (line 26) omits `sigma0`,
   though the docstring and make_arms both call it the control. The committed analyzer
   cannot produce C10's control column.
3. **`gainevo_make_arms.py` builds the confounded control** (§2.1) — the `sigma0` arm must
   also write `TSWEEP_START` into the MotorEPMv2 params.

### 2.5 The shipped configs still carry the refuted window — and one weight drift

All three ship candidates (`__gainevo`, `__gainevo_live`, `__gainevo_factory`) still run
`eval_window_ticks: 4000` against a module default of 12000 whose schema text records *why*
4000 was refuted. The session's own harness-trap entry names this exact drift, and it is
still live in the artifacts about to ship. Separately: the module default `w_energy = 8.0`
disagrees with every config's 4.0 (the status doc's 1/1/1/4 describes what ran); align one
to the other before a fresh config inherits the default silently.

### 2.6 The unit tests never close the loop

In all 33 tests, J is **causally independent of the published gain vector** — the feeds are
functions of tick index only. The (1+1)-ES has never been shown to climb even a synthetic
bowl; `mutate_candidate` could ignore `gain_sigma_scale_` or center on the wrong point and
no test would fail. Specific defects:

- `AcceptMarginBlocksSubNoiseImprovement` is **vacuous**: distress 1.0 vs 0.99 both clear
  `distress_thresh` 0.05, so both windows score duty 1.0 and `J_cand == J_inc` exactly —
  the strict `<` rejects with or without the margin feature.
- `AcceptMarginStillAcceptsClearWin` runs with σ̂ = 0 (identical incumbent windows make the
  first noise sample 0², which `sigma_est()` short-circuits on), so it proves a win is
  accepted with **no margin in play**.
- The fall alarm's actual contract — tripped alarm forces `tol = −1`, a strict falls
  *reduction* (`GainEvolver.cpp:586`) — is never exercised; only the accumulator is.
- The anneal's behavior at exactly-chance acceptance is unpinned (the one test asserts
  σ ≤ 0.1, compatible with flat, drifting, or collapsing), and `sigma_min`'s shipped 0.08
  is pinned only at ≥ 0.01.

### 2.7 Minor design notes (recorded, none the failure cause)

- The revert-pair noise estimator samples pairs conditional on a revert, which regression-
  to-the-mean inflates slightly — conservative direction; and with `noise_alpha 0.25`,
  `noise_min_n 3`, the margin is jittery for the first ~dozen pairs.
- Incumbent→candidate always run in that order, so any within-generation drift (the
  MotorEPM keeps learning under the evaluator) biases every comparison with the same sign.
  The measured whole-run drift is small, so this is hygiene, not diagnosis — an order flip
  or ABBA scheme per generation is cheap if the loop is ever rebuilt.
- The consumer path is clean: no clamps narrower than the declared bounds on any of the 8
  keys, read-back equality is exact, restore replays applied gains in the right order. The
  one guarded hazard — amp_seek's 0.60 ceiling vs the evolver's 0.8 — is blocked at setup
  but re-openable by hot-mutating `amp_seek_rate` (no re-check in `on_param_change`).
- The gyro feeding `flow_turn` is genuinely body-frame (`ω⃗·body_up`,
  `picrawler_body.gd:6044-6063`) — the lever's input is legal; the lever itself has never
  been live (§3.7).

---

## 3. The §5 audit brief, item by item

1. **C4 regrouping (authority cross-check).** Cannot be recomputed: analyzer broken (§2.4)
   and the logs were ephemeral. On structure: group means over 2–3 gains at n=12 carry
   roughly ±0.2 sampling error, so STRONG 0.95 vs weak 1.11 is **inside noise**; the
   defensible claim is binary — *the three seen gains resist scattering, the two flat ones
   scatter* — plus the caveat that the "two bodies" shared start vectors and criterion.
   Whether FLAT is carried by `rear_land_gain` alone is unanswerable without a re-run.
   §2.3 gives the grouping a mechanism, which is worth more than the third significant
   figure.
2. **C3 independence.** The two operating points genuinely differ where it matters —
   postural 0.7 → 1.09, rear_land 0.5 → 0.19, rear_push 0.5 → 0.03 — but share
   `amp_target` (0.400 vs 0.385), the body, the environment and the criterion. *"Replicated
   at a second operating point"* is accurate; *"independent"* overstates. Keep the claim,
   soften the adjective.
3. **C2 band logic.** Sound. Windows-by-order is robust at the 60-tick emit cadence; the
   hysteresis check caught `rear_land_gain` and `plan_gain` (both non-STRONG, so no verdict
   rests on them). One framing caveat for the report: authority is measured against
   **single-window** noise because that is what the accept rule compares. "The criterion
   sees 3 of 8" is a statement *at single-window resolution* — K-window averaging raises
   every span/noise by ~√K and could promote the weak gains to visible. The criterion may
   be sharper than the search that reads it.
4. **C1 attribution.** The A/B stands and the variance-collapse F-tests are the loud part.
   Report the mechanism as *run-and-select* (search proposes, operator selects), exactly as
   the ledger already phrases it.
5. **C10 estimator.** Last-third-minus-first-third is fine (the slope estimator agrees and
   is already printed). The real problems are §2.1 (the control) and §1 (power): at σ = 0.08
   the expected climb signal over 28 generations is ~0.04 J against a per-run sd of ~0.2 —
   the experiment could not have shown success. **C10's honest form: "at the step size the
   anneal selects, searching is indistinguishable from not searching — as the accept
   arithmetic predicts."** "The (1+1)-ES cannot climb this criterion" is *not established*;
   σ = 0.20 is untested at n ≥ 6 and is the arithmetic's own predicted fix.
6. **C12.** Measured — upgrade, cite the 2026-08-06 numbers (§2.2).
7. **`flow_turn`.** Confirmed dead code in the shipped sense: exactly one config sets
   `flow_turn_k > 0` (`__j1s4_flowturn.json`) and nothing references it; every run that
   ever executed had `turn = 1.0` identically (`ge_fturn` 1.0000 in every log). Unit
   coverage is good (the signed-slow-EMA test is real). Mention only as built-unevaluated.

---

## 4. Before the report — ranked, with costs

1. **Re-run the decision experiment (one overnight, ~12 runs).** Arms: `sigma0` with the
   displaced point written into the MotorEPM params (the fixed control), `fix008`, and
   **`fix020`**, n=6 each, varied `OGMA_SEED`, window 12000. This is the experiment §1 says
   can change the headline. If fix020 recovers the band at n≥4/6, the report's story
   becomes *"the search climbs once the step matches the noise; the anneal was pinning it
   below that"* — a working mechanism with one mis-set servo. If it fails, C10 becomes
   defensible at the right σ. Either outcome is a better report than the current one.
2. **Fix the tooling (small diffs):** restore `GOOD_BAND`/`AUTHORITY` in `gainevo_basin.py`
   (or move them into the spec.json make_arms emits); add `sigma0` to `gainevo_tsweep.py`'s
   `ARMS` plus the vs-control t; make `gainevo_make_arms.py`'s sigma0 arm displace the
   consumer's params too.
3. **Resolve the config drift:** either move the three ship configs to
   `eval_window_ticks: 12000` or write down why 4000 stays; align `w_energy` default vs
   config (§2.5).
4. **Fold the step-matching bound into the module.** Cheapest honest form: raise `sigma_min`
   toward 0.2 and neutralize the anneal (C9 showed `target_accept` is only a σ selector, so
   pinning σ loses nothing measured). The principled form is the status doc's own §6: anneal
   on realized ΔJ over a block of generations — acceptance rate can never be its own
   feedback while it is servoed to chance.
5. **Add the missing test** — a closed-loop synthetic bowl: a fake consumer feeds
   J ∝ Σ(gₖ−gₖ*)² + noise back through the sensor topics; assert the incumbent's distance
   to g* falls over N generations and beats a σ=0 control. Repair the two vacuous margin
   tests with a continuous term (torque), not thresholded duty; exercise the alarm→G1
   tightening.
6. **Report framing for the criterion:** tilt_sd + unloaded carry the structure; energy
   (weight 4, dominant) is a measured damping copy ≈ joint-speed penalty; flow is a measured
   amplitude proxy; 1 of 7 weight units ports. Present as the measured map it is — the
   sim2real additions (INA219, FSRs, FK+IMU travel) are what make each proxy honest, which
   is the argument that carries here.

Deeper levers, post-report: paired multi-window acceptance (K interleaved pairs, ABBA
order), SPRT-style racing of candidate vs incumbent, `joint_load`/current-analog energy, a
legal travel term from FK+IMU fusion (which is also the deferred slip signal).

---

## 5. What this audit did not do

No new runs were executed; every quantitative claim above is either read from committed
code or recomputed from numbers already in the ledger. The §4.1 experiment is the only
place where an audit conclusion depends on data that does not yet exist.
