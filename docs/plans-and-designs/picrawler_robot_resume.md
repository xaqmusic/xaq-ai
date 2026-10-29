# PiCrawler on the robot — ▶ Resume here (2026-10-27)

The working state of the real robot's brain, for a session starting fresh. The evidence for every claim
is in [`picrawler_lever_ledger.md`](../reports/picrawler_lever_ledger.md), entries dated 2026-10-26 and
2026-10-27. This page says where things stand and what to do next.

## The goal now

**Converge on a slow, steady pace** (operator, 2026-10-27). The robot walks, every leg pushes, and nothing
saturates. But the pace is not steady: the body's path speed is roughly constant, while its net progress
swings with heading wander and loose leg coordination. Per CLAUDE.md §1, "steady pace" has to become an error
the brain minimizes, not a schedule it follows.

## The deployed stack

Config: `…__lpgn_r05_e0d0_ref300__robot_p1_presskin_lz_sgk_hk0.json` (launcher and picrawler-dash). Each
suffix is one lever on the one before it:

| suffix | lever | what it fixed | verdict |
|---|---|---|---|
| `robot_p1` | the promoted robot stack (LoadPhaseGait, running-attitude gate, corner integrator) | — | `BASELINE` |
| `presskin` | MotorEPMv2 `press_kin`: the learned stance press pushes along each leg's lift direction (`feet_jac`) | a fixed hip2+knee press folded a leg under | `WORKING` |
| `_lz` | `level_int_zero_mean`: the corner integrator's offsets held zero-mean | it ran all legs to −0.30 against the press at +0.30; belly 48 → 62 mm | `WORKING` (2 runs) |
| `_sg` | LoadPhaseGait `support_gate`: a leg waits at the end of stance until the CoG is within a learned allowance of the feet that stay down (toe positions `feet_xz` from the commanded FK) | sim dips 29 → 18 % of lift-offs | `WORKING` sim / robot unclear (holds ~65 % of lift-offs) |
| `k` | MotorEPMv2 `kin_route`: the stance reach and the integrator also push along the lift direction | a rear leg folded under, knee pinned at the keep-out cap | `WORKING` (no fold since) |
| `_hk0` | `ctrl_lr` 0, `bias_lr` 0: the HK controller frozen at its start (its output remains, ~14 % of the command) | one leg at a time slid into saturated chatter after minutes (FR 82 %, stroke reversed, body moved backward) | `WORKING`; operator: HK off is right for the robot now |
| `_cs` | LoadPhaseGait `load_topic` empty: the Tegotae rule's load share from the contact flags (1/n touching), not the FSR force through the shared curve | the legs never locked (diagonal phase lock ~0.3 vs the sim's 0.88; per-leg rates 30–45 % apart); with contact shares 0.51 and 0.89, touchdown CV 0.38, the support gate's dips at target, no "swim" marks | `WORKING` (2 runs, 2026-10-28) — **the operator's best stepping so far**; circles right (below) |
| `_cs_ra` | **lever A**, BearingToIntent `ref_attack_s` 2 s: the heading reference is earned by a loop's persistence, not seized by a one-ping sonar blip | `_cs` circled right at 4–6°/s: the avoid loop's short activations set the reference to the right and the hold finished each turn; the left-side push was the hold's own steer differential | `IN_FLIGHT` (sim A/B 2026-10-28; robot run pending) |

On the shelf, not deployed: `_sgkc` (ProxMemory corroboration, below), `_sg0` (the gate watching only).

## Read before the next session (2026-10-28)

- **Leg names in brain_diag are the sim's order and the sim is mirrored:** sim `fl` = physical FR (ADC A1), `fr` = FL (A0), `rl` = RR (A3), `rr` = RL (A2).
- **The stride odometry over-reads 2–3× on this floor** (sonar closing rate vs the stride estimate); the brain's progress sense cannot see slip. `progress_commit_gain` is fed by it (`_pc0` = off, `NULL` on coordination, n = 1).
- **A severed FSR wire (physical FR) broke the gait** on two runs of the 28th morning; check the four channels' duty in the feed before trusting a run.
- **The operator's mark key** (`g` / `w` on the dash) and the per-tick IMU rows in the host log exist since 2026-10-27.
- **Next:** one robot run of `_cs_ra` (lever A); then video the carpet → vinyl transition on the fixed stack; then `_pc0` on top; in the sim, the robot-measured disturbance layer and the contact belief for the gait's shares (`generator_and_belief_filter.md` G2).

## What the last run says about pace (run 185039, 336 s)

- **Path speed is steady** (144–227 cm per 30 s, dead-reckoned); **net progress is not** (−25 to +182 cm per 30 s).
  The difference is heading wander: summed |Δyaw| 60–306° per 30 s.
- **The per-sample stride velocity is a blind metric.** Its sd is ~11 cm/s around a mean of −2 to +3 cm/s, so pace
  must be read per stride or per window, not per sample.
- **Coordination is loose**: touchdown-phase PLV 0.15–0.35, inter-leg PLV 0.25–0.45.
- **The support gate holds ~65 % of lift-offs**, and 26–51 % of lift-offs are still followed by a belly dip.
  The gate may be fighting the rhythm on the robot; it has never had a robot A/B.
- The LPG period is fixed at 50 ticks, and the level gate shortens the stroke to 0.4–0.65 of its nominal value.
- **Speed is set by period and stroke together** (sim speed map, 2026-10-26): with the stroke at 1.0, the period alone
  sweeps 4.9 → 7.0 cm/s without a fall. Short strides slip.

**Directions to consider** (none built). Check the ledger before building any:
1. **A pace error.** A per-stride speed estimate (odometry over each stride, not per sample) against an intended
   slow pace. The descent direction should go to ONE handle (the LPG period, with the stroke at 1.0) as a meta-control,
   rather than to several knobs.
2. **Heading.** Net progress is lost to yaw wander. Check what the heading hold sees on the robot (intent `err`
   runs ±0.5 rad) and whether its correction rides the stroke differential.
3. **The support gate on the robot.** One A/B run, `_sgk_hk0` with `support_gate` 0, to see whether the 65 % hold rate
   costs regularity.
4. **Instruments first.** Stride-period CV per leg, per-stride displacement, and per-stride heading change, so a pace
   lever has a full metric set (CLAUDE.md §3 rule 4).

## Open items

- **Range false readings.** About half are a front toe in the beam (44 % vs 10.9 % base rate). ProxMemory
  `support_n` / `too_close_pings` are built and unit-tested. Replay picked 6 / 6 in-sample (0 false). Out of sample
  it left 56 false ticks and may have delayed a real approach by 13 s. Needs a better truth criterion (the camera)
  or the toe-in-beam gate (the toes' FK plus the sensor's mount and beam width) before it drives the robot. The
  234 s cluster in run 164309 was a real reflex turn.
- **FSR scale.** Rear-left (sim RL) reads ~680 g while planted, others 30–56 g. LoadPhaseGait self-normalises, so the
  gait is fine, but any load-magnitude use (the CoG at stand, `support_cog`) needs a per-foot calibration first.
- **Not deployed yet:** the dash's second sync after "SAFE TO POWER OFF" (that line was lost twice), and
  `hk_cnorm` / `hk_hnorm` in the brain_diag snapshot.
- **Sim fidelity:** the sim does not sink on low friction (the slip sheet `OGMA_PICRAWLER_TERRAIN_SLIP` loses traction
  only), never tucks a leg, and its ultrasonic does not see the legs. The chassis friction and the per-foot slick
  ablation were inert (Godot's rough-material rule; a separate task is open for the chassis).
- **Sim start-up flip** with the support gate (3 flips at tick ~300 across 12 gate runs). Operator: not an issue on
  the robot.

## Tools

- `pi_host/tools/leg_jac_from_log.cpp` — per frame, per leg: depth, lift gains (hip2, knee), reach, hip2, knee,
  femur and shin tilt, toe x/z (chassis frame), plus grams per foot. Build: `g++ -std=c++20 -O2 -I cpp_core/include
  -I pi_host/include -I pi_host/build_local/_deps/json-src/single_include pi_host/tools/leg_jac_from_log.cpp`.
  Run from the repo root.
- `cpp_core/build/prox_replay <run.log> [k tc]...` — a run's pings and odometry through the real ProxMemory.
- **Propulsion:** the commanded toe's fore-aft speed while planted (negative = pushing forward), from the leg tool.
  It found the reversed FR stroke.
- **Saturation:** the run log's `{"kind":"actions"}` rows (u leg-major: leg × 3 + [hip1, hip2, knee]); % of ticks
  at |u| ≥ 0.99 per leg per window.
- **Pace:** `brain_inputs` rows every 50 ticks carry `odom` [x, y, yaw] and `stride_v`; brain_diag carries
  `load_phase`, `motor_epm`, `intent`, `avoid` at ~5 Hz. The run log carries every range ping
  (`{"kind":"range"}`).
- **Long sims:** run past the slowest integrator. The one-leg saturation only appears after ~9000 ticks; use 24 000.

## Procedures

- **Deploy:** `ssh picrawler 'cd ~/xaq-ai && git fetch -q origin && git merge --ff-only -q origin/picrawler-dev'`,
  then `cmake --build pi_host/build --target ogma_host -j3` on the Pi (~1 min). No service restart: the dash
  starts the new binary on the next run. Deploys need the operator's go-ahead.
- **After a boot** the Pi answers on `10.0.0.114` before `picrawler.local` resolves.
- **Runs land in** `~/xaq-ai/pi_host/log/dashrun_<stamp>/` (feed, brain_diag, events, meta) plus the host log
  `dashrun_<stamp>_…__<config>.log`. Copy them to `~/xaq_runs/robot_<date>/`.
- **The operator powers off after every run** (bench supply). The dash flushes to the SD card and says SAFE TO
  POWER OFF. A red light on power-up was cleared by reseating the power (2026-10-27).
