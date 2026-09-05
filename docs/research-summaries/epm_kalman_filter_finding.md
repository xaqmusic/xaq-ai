# EPM as a Kalman filter: a trust-sign finding, and a docs TODO

*Design note. Connects the EPM/LateralVoter trust mechanism to Kalman filter theory,
reports a concrete gap the comparison exposes in the current trust formula (verified
against `python/xaq/xaq/lateral_voter_v3.py`), and leaves a TODO for a future
documentation pass. Nothing here has been implemented or measured yet — treat §3 as a
proposal, not a result.*

---

## 1. The isomorphism, briefly

An EPM's dual Time-Loop Error (TLE) cycle — predict next state, observe, compute error,
correct — is structurally a Kalman filter cycle: TLE plays the role of the innovation
(observed-minus-predicted residual), and trust = 1/(TLE+ε) plays the role of the Kalman
gain. One divergence is already a strength: trust is estimated from the EPM's own
realized error rather than an assumed fixed noise covariance, which is closer to adaptive
Kalman filtering (self-estimating Q/R from the innovation sequence) than to the textbook
fixed-noise version, and matches the doctrine's own law that precision should be earned
from the system's dynamics, never a tuned constant.

## 2. The finding: placeholder tokens get the trust sign backwards

`docs/plans-and-designs/primitives/EPM.md`'s failure-mode table documents at least three
cases where an EPM publishes `tle = 0.0` as a safe placeholder rather than a real
measurement:

- First tick, GNG bootstrap not complete (`winner_id = -1`).
- `params.input_topic` missing throughout the run (bootstrap-placeholder token).
- GNG step throws (NaN propagation) — one-tick "warming" mode.

In `lateral_voter_v3.py`, trust is computed as `inv_tle = 1.0 / (tles + _EPS)`, fed
directly into group-proportional trust weighting — no other gate on this value was found
in that file. A `tle = 0.0` placeholder therefore produces the single **largest** possible
trust weight in the fusion, for a channel that has published no real information at all.

This is exactly the failure a Kalman filter's design would prevent: a real filter's
uncertainty *grows* when no genuine observation has updated the state. Here, the absence
of a genuine observation instead produces the most-trusted channel in the room — the sign
is inverted.

**Scope, stated honestly.** This is confirmed in the Python v3 reference module
specifically. The doctrine (`brain_building_doctrine.md:368`) references a separate
"baked-node informativeness" strip for degenerate channels; it isn't present in
`lateral_voter_v3.py`, so either it lives only in the C++ core or it's a documented
mechanism the code doesn't actually have — a pattern this project has been burned by
before (the insertion-gate self-tuning mismatch in `EPM.md`). Worth checking the live
path before assuming either way (§4).

## 3. Proposed fix — not yet implemented or tested

Two options, not mutually exclusive:

1. **Exclude placeholder tokens from the trust computation entirely** — gate on
   `winner_id == -1` or an explicit placeholder flag before the token ever reaches
   `inv_tle`. Cheapest fix; reuses a signal (`winner_id = -1`) the codebase already
   treats as "nothing real happened" (`EPM.md` invariant 3).
2. **Gate trust by baked-node fraction**, reviving whatever the doctrine's
   "baked-node informativeness" strip was meant to be. This would also let a
   fast-EPM's confidence into the fast → slow interface already logged in
   `fusion_notes.md` — that doc leans on baked-node informativeness for a related but
   distinct purpose (stripping degenerate channels before concatenation), so a single
   informativeness signal could plausibly serve both.

## 4. Open questions

- Does the C++ core already implement the informativeness strip this Python reference
  lacks? Check before assuming the gap reaches production.
- Does the same inversion reach the slow loop? An event-gated keyframe sampler
  (`slow_loop_design_notes.md` §1) that happens to fire during a placeholder-token tick
  would inherit the same backwards trust signal one level up.

## 5. Gate

| Mechanism | Gate |
|---|---|
| Placeholder-token trust exclusion | Force a `winner_id = -1` tick mid-run on one channel; that channel's trust should drop to (near) zero for that tick, not spike to the run's maximum |

---

## TODO — future docs pass

- [ ] Add a section to `docs/plans-and-designs/primitives/EPM.md` (or a new
      `docs/research-summaries/` entry) framing the EPM's dual-TLE cycle explicitly as a
      Kalman filter analogue, for readers arriving from the active inference / control
      theory literature:
      - predict step = forward TLE prediction
      - observation = new sensory input
      - innovation = TLE
      - gain = trust = 1/(TLE+ε)
      - update = GNG prototype move
      - note the divergences as features, not gaps: self-estimated (adaptive-Kalman-style)
        precision instead of a fixed Q/R; the GNG's discrete multi-node topology as a
        particle-filter / Gaussian-mixture-filter analogue rather than a single-Gaussian
        belief.
      - cite Rao & Ballard (1999) and Friston's generalized filtering as the reason this
        framing will be recognizable to that audience.
- [ ] Cross-reference from `docs/glossary.md`'s existing "bake" entry.
- [ ] Once §1 lands, fold this document's §2–3 finding into `EPM.md`'s failure-mode table
      directly (or supersede this note) rather than leaving it as a standalone file.
