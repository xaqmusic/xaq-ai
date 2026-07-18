# Motor-EPM — Vision Integration + Roadmap (2026-06-13)

Picks up after reward-free nav landed (perceive target bearing → steer → contact;
4/4 seeds visit, indefinite chaining). The oracle `target_compass` and the
facing-gate are SCAFFOLDS. This plan replaces them with real perception, and
lists the parallel improvements the nav work surfaced.

**Guiding principle (operator, 2026-06-13):** a hand-binned vision signal wired
to a steering lever works for a steerable robot but is System-2 hand-coding — it
presumes we already know what in the visual field matters and forecloses richer
behaviour (obstacles, multiple targets, terrain). Prefer the EPM /
active-inference path: vision goes into a *learned* encoder+predictor on the
perceptual bus; the robot learns what matters. Keep oracle signals as MEASUREMENT
baselines. Stage every step behind a falsification gate (fast-fail protocol).

---

## THREAD 1 — Vision on the slow nav path (Cell raycast→EPM)

The fast loop stays as-is (MotorEPM on local proprioception — spinalized walking).
Vision belongs ONE LEVEL UP as slow gait modulation: perception decides
where/whether, homeokinesis decides how. The `steer` input of MotorEPM is the
socket the slow path plugs into (already there, already drives the body).

### V0 — Recon (no code, ~½ day)
Inventory before building:
- The Cell's raycast→EPM vision pipeline: confirm it exists as a reusable graph
  module, its raycast sensor shape (rays × channels), encoder type, EPM config.
- The picrawler's existing panoramic raycast (`_compute_target_loom`, 180° FOV
  sweep) — can it feed the Cell vision EPM, or do we publish a fuller raycast
  vector (distances/hit-IDs per ray) as a new `reality.vision.*` proprio channel?
- Decide the sensor→EPM adapter. Deliverable: a short interface note.

**Two sensor modalities from one raycast (operator insight 2026-06-13).** A
forward raycast grid yields BOTH per ray, free: hit COLOUR/class = a **camera
(RGB)**, hit DISTANCE = a **LiDAR/sonar range field**. The loom was already the
range sensor, collapsed to a scalar. Both are now captured
(`_last_vision_pixels` / `_last_vision_depth`) and rendered as stacked HUD panels
(top-down view). **Keep both as alternative V1 sensors:** depth/range is the
*sim2real-friendly* one (real quadrupeds carry cheap ToF / ultrasonic / single-
plane LiDAR arrays far more often than a calibrated, lit, distortion-corrected
camera). Camera RGB is the richer signal in sim. V1 should run the
predictive-bearing-decode GATE on EACH modality independently — depth alone may
already carry bearing (a target pyramid is a coherent range blob), which would be
the strongest sim2real story: reward-free nav from a $5 sensor.
CAVEAT: our "depth" is geometric ground-truth range; real ToF/sonar add noise,
dropouts, multipath, and a wide beam cone — model those before claiming sim2real
transfer (a V-phase sub-task, not assumed).

### V1 — Predictive check (perception only, NO motor wiring) — THE GATE
Wire panoramic raycast → vision EPM; let it tick and form a latent. Verify the
latent **carries target bearing across an approach** (incl. the far regime).
- Metric: linear-decode bearing from the latent (or correlation of latent PCs
  with ground-truth `target_compass`). Reuse the oracle here purely as the label.
- **GATE / falsification:** if the representation is bearing-blind (decode ≈
  chance), STOP — raycast resolution or the EPM isn't capturing it; fix the
  sensor before anything downstream. Nothing can steer on a signal that isn't
  there. This is the predictive-vs-reactive check the discipline memo demands.

### V2 — Slow-path steer modulation (vision drives the body)
Let the vision signal set MotorEPM's `steer` target. First pass may use a SIMPLE
decode (even a fixed linear readout) just to prove the end-to-end loop closes
with vision — not the oracle — driving steering.
- A/B vs oracle nav (same seeds/arena). Keep `target_compass` for measurement.
- **GATE:** vision-driven nav reaches targets comparably to the oracle (visits
  within a factor). Honest flag: the decode is still a semi-hand-coded readout —
  a stepping stone, not the endpoint.

### V3 — Reward-free active-inference readout (the prize)
Replace the decode with a homeokinetic controller on the slow vision loop: **steer
to minimize visual prediction error** (the vision EPM's TLE). "Approach the
target" emerges from "reduce visual surprise / keep the looming thing predictable
and centred." No reward, no bin→wheel map — the slow-path mirror of what MotorEPM
does on proprioception.
- A/B vs V2.
- **Honest risk:** reward-free action-extraction from a perceptual EPM is the
  unsolved-in-this-project problem (cognitive-premotor REINFORCE collapse). The
  MotorEPM win is the existence proof the template can work, but this can null.
  If it does, V2's decode is the grounded fallback — we've still replaced the
  oracle with a real EPM. Don't rip out working nav for V3 on faith.

### Cross-cutting enabler — telemetry
Surface MotorEPM + vision-EPM internals (motor TLE, loop gain, `gait_coherence`,
vision latent/TLE) in the diag stream (accessors+snapshot already exist; needs a
`get_module_metrics` branch / panel readout). Makes V1's check and V3's loop
MEASURABLE instead of eyeballed. Flagged "not yet built" in memory; do it
alongside V1.

---

## THREAD 2 — Improvements the nav work surfaced (prioritised)

1. **g6dof gait re-validation (HIGH).** The substrate shifted to g6dof+freeplay;
   most open-loop walking metrics (straightness, speed, amplitude spread,
   reproducibility) are hinge/legacy-era. Re-run them on g6dof so the standing +
   walking claims rest on the substrate we actually ship.
2. **height_k n=10 sweep.** The height↔stability tradeoff (k=0.65 default) is n=1.
   Lock it with the powered run; confirm no tip-overs across seeds.
3. **Nav reproducibility n=10 (Phase A5).** The nav result is n=1-per-cell quick
   A/Bs. The powered, seed-paired run is what the publication needs.
4. **Obstacle avoidance via vision-loom (unlocks capability).** The across-router
   is a stopgap for occlusion. Once vision is on the slow path, looming = an
   obstacle detector too (LGMD veer) — retire the stopgap, enable dense arenas +
   pyramid-climbing. Natural extension of Thread 1, not a separate build.
5. **Contact reliability / facing-gate.** 4/4 seeds but not every traverse. DO NOT
   over-invest in the hand-coded gate — it's exactly what vision should subsume.
   Expose the gate floor as a tunable at most; let Thread 1 replace it.
6. **Two-stage walk→nav curriculum (minor).** Stage 0 walk-only → stage 1 target,
   so the "watch the gait, then engage the target" flow is curriculum-defined too
   (currently that flow leans on the launcher dropdown for the backend).

---

## THREAD 3 — Publication track (resume, paused for gait refinement)
A3 ablation (needs honest reframe — HK-not-strictly-necessary finding), A4 reward
head-to-head, A5 reproducibility, A6 developmental figure, A7 docs/claims sheet.
The reward-free NAV + the hip2 STANDING wins are new headline results to fold into
the evidence package. See the approved plan in
`/home/xaqmusic/.claude/plans/i-don-t-want-to-typed-ocean.md`.

---

## Suggested sequencing
1. V0 recon + telemetry enabler (low-risk, unblocks measurement).
2. V1 predictive check — the gate that decides if Thread 1 is viable at all.
3. In parallel (independent, no substrate risk): #1 g6dof re-validation, #3 nav
   n=10 — these also feed the publication track.
4. V2 → V3 (vision drives, then reward-free), gated.
5. #4 obstacle avoidance once V2 proves vision-on-slow-path.
