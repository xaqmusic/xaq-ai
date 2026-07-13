# The Fractal JEPA Workspace: An Autopoietic Architecture for Open-Ended Continuous Learning

**Abstract**
Current machine learning architectures struggle with open-ended continuous learning, often falling victim to catastrophic forgetting or prototype runaway when exposed to novel, high-dimensional environments. [cite_start]We propose a novel cognitive architecture that integrates Joint Embedding Predictive Architectures (VL-JEPA) [cite: 422][cite_start], homeokinetic intrinsic motivation [cite: 841, 843][cite_start], and Assembly Theory-inspired complexity metrics [cite: 114] within a Global Workspace framework. [cite_start]By decoupling continuous perceptual processing from localized Episodic Predictive Memory (EPM) columns, and utilizing a "Mitosis Gatekeeper" based on high-order entropy[cite: 114], the system can autonomously balance between adapting existing knowledge and spawning new structural modules. This allows for fractal, unsupervised expansion of intelligence that supports both rapid, reactive System 1 exploration and slow, deliberative System 2 planning.

---

### 1. Introduction
[cite_start]The pursuit of Artificial General Intelligence requires systems capable of "self-actualization"—the self-determined unfolding of sensorimotor contingencies without explicit external goals[cite: 837]. [cite_start]Standard Vision-Language Models (VLMs) operate in discrete token spaces, which requires expensive autoregressive generation and struggles with real-time, continuous state updates[cite: 436, 440]. Furthermore, scaling these systems to accommodate new paradigms often necessitates retraining massive parameter sets. 

We propose a "Publish-Subscribe Cognitive Bus" architecture. [cite_start]By utilizing non-generative representation learning (VL-JEPA) [cite: 422] [cite_start]to publish continuous abstract embeddings [cite: 423][cite_start], and governing behavior via the Homeokinetic Principle[cite: 841], we establish a robust System 1 substrate. We then introduce an epistemic memory layer (EPMs acting as cortical columns) governed by a Lateral Voter and Global Workspace (System 2). [cite_start]To manage unsupervised structural growth, we define a "Mitosis Gatekeeper" utilizing high-order entropy [cite: 114] to detect emergent complexity and trigger the localized crystallization of new neural modules.

### 2. The Perceptual Kernel & System 1 Substrate
[cite_start]The foundation of the architecture relies on continuous, real-time perception and intrinsic motivation, bypassing the latency of traditional token-prediction[cite: 453].

**2.1. The Continuous Latent Bus (VL-JEPA)**
[cite_start]The system processes environmental states using a Joint Embedding Predictive Architecture[cite: 422]. [cite_start]Foundational encoders (e.g., Retinal, Dorsal, Ventral) map visual inputs $X_V$ into continuous visual embeddings $S_V$[cite: 444, 479]. [cite_start]Concurrently, a forward model predicts future target embeddings $\hat{S}_Y$[cite: 444]. These encoders operate at a fixed clock speed, acting as "always-on" publishers of both the current state of the world and the expected future.

**2.2. Homeokinetic Exploration**
[cite_start]To drive exploration without explicit reward functions, the system utilizes Homeokinesis[cite: 841, 843]. [cite_start]Rather than minimizing standard forward prediction error (which can lead to a "lazy robot" effect where the agent does nothing to maintain perfect predictability) [cite: 841, 899][cite_start], the agent seeks to minimize the Time-Loop Error (TLE)[cite: 911, 913]. [cite_start]TLE measures the reconstruction error of the past state from the predicted future state[cite: 913, 914]. [cite_start]Minimizing this error naturally drives the agent to destabilize static states while finding functional, predictable physical resonances in its environment[cite: 912, 916].

### 3. The Epistemic Engine & The Mitosis Gatekeeper
As System 1 babbles and explores the continuous latent space, the system requires a mechanism to capture and modularize useful discoveries without suffering from node explosion. 

**3.1. Episodic Predictive Memory (EPM)**
EPMs act as specialized, limited-capacity experts (analogous to cortical columns) that subscribe to the VL-JEPA latent bus. [cite_start]Instead of retraining the foundational visual encoders[cite: 444], learning occurs within these lightweight relational nodes.

**3.2. Complexity Detection via High-Order Entropy**
To evaluate the structural value of a latent trajectory, the system employs an Assembly Theory proxy. [cite_start]Drawing on methodologies from *Computational Life*, we monitor the "high-order entropy" of the latent stream[cite: 114]. [cite_start]This metric factors out information that comes from sampling independent and identically distributed variables[cite: 116]. 

[cite_start]By vector-quantizing the latent space, we can compute the Shannon entropy $H(X)$ to measure vocabulary diversity, and approximate the normalized Kolmogorov complexity $K(X)$ using Lempel-Ziv-style text compressors[cite: 114, 128]. [cite_start]The resulting High-Order Entropy metric detects when the system transitions from random noise into highly structured, repeating behaviors[cite: 121, 143].

**3.3. The Spawning Trigger**
The Global Workspace relies on a strict conditional matrix to manage neural resources:
1.  [cite_start]**Resonance:** A sudden drop in the homeokinetic Time-Loop Error indicates the agent has found a predictable physical mode[cite: 916].
2.  [cite_start]**Complexity:** A simultaneous spike in High-Order Entropy indicates the sequence is structured and non-random[cite: 142, 143].
When both conditions are met, the system crystallizes the transient sensorimotor loop into a new, permanent EPM module.

### 4. The Deliberative Supervisor (System 2)
The society of EPM experts is managed by a routing hierarchy that facilitates both reactive control and deliberative planning.

**4.1. Lateral Voting**
EPMs constantly compare the JEPA encoder's broadcasted reality against their own internal predictions. The Lateral Voter monitors this Prediction Error (PE). EPMs with a low PE achieve "Predictive Resonance" and are granted access to the Global Workspace. EPMs experiencing massive PE (e.g., during a drastic environment shift) lose resonance and are safely archived, protecting their weights from catastrophic forgetting.

**4.2. Global Workspace (GW) & Top-Down Guidance**
The GW operates over extended time horizons, maintaining a stable abstract context vector. It facilitates System 2 planning by performing explicit rollout predictions in the latent space. The GW exerts top-down control by generating sustained bias adjustments to the lower-level homeokinetic controllers, channeling playful exploration into abstract, goal-directed behaviors.

### 5. Mechanism of Autopoiesis: Fractal Expansion
The proposed architecture natively scales to handle paradigm shifts, such as moving from a low-dimensional deterministic task (e.g., Pong) to a highly complex, hostile 3D environment (e.g., DOOM). 

Upon experiencing a massive context shift, existing EPMs experience a spike in Prediction Error and are archived by the Lateral Voter. [cite_start]The system reverts to System 1 Homeokinetic babbling[cite: 916]. [cite_start]Once the agent discovers a new physical resonance (e.g., navigating a 3D corridor), the High-Order Entropy metric spikes[cite: 142]. The Global Workspace crystallizes this new behavior into a localized EPM. As the library of EPMs grows, the Global Workspace can spawn localized Sub-Voters to manage specific environmental domains, achieving true fractal abstraction.

### 6. Conclusion
By decoupling foundational representation learning from specialized episodic memory, and by utilizing high-order entropy as a gatekeeper for neurogenesis, the Fractal JEPA Workspace presents a computationally tractable path to continuous learning. The integration of homeokinetic drive ensures the system remains intrinsically motivated to explore, while the publish-subscribe cognitive bus guarantees that knowledge can be infinitely expanded without destructive interference.

---

# Addendum (2026-06-06): Plasticity Asymmetry and the Homeokinetic Cognitive Population

**Status:** Empirically anchored in the picrawler v6 substrate by the F0–F6
frozen-brain battery executed 2026-06-06. See
`docs/plans-and-designs/hierarchical_plasticity_battery.md` for full
experimental design + dose-response data.

This addendum extends §§ 2, 3, 5, and 6 of the original position paper. It
does not replace any prior claim; it makes explicit a plasticity-theoretic
distinction the original implicitly assumed, and prescribes the specific
architectural move that makes the system's reward dynamics self-stabilizing
in the same sense the EPMs already are.

## A.1 The empirical foundation: plasticity quadrants × navigation

A 2×2 cross-product of {Premotors plastic, Premotors frozen} × {EPMs plastic,
EPMs frozen} was run on the H1v6 picrawler substrate with a standing-snapshot
brain warmup. All four arms loaded the same snapshot, used the same target
curriculum where applicable, and ran for 20 sim-min × n=5 stratified seeds.

| Arm                                | Premotors | EPMs       | Speed   | max_d  | Falls | %up   | walk_visits |
|------------------------------------|-----------|------------|---------|--------|-------|-------|-------------|
| F1 (stand-only baseline)           | frozen    | plastic    | 0.112   | 1.94 m | 240   | 94.5  | 0/5         |
| F2 (target curriculum)             | frozen    | plastic    | 0.127   | 2.48 m | 104   | 96.8  | 0/5         |
| F0' (truly frozen)                 | frozen    | frozen     | 0.113   | 1.44 m | 260   | 93.2  | 0/2         |
| F5 (hierarchical: substrate frz, slow plastic) | frozen | mixed | 0.130 | 2.64 m | 100   | 96.7  | 0/5         |
| F6 (stable features + plastic policy) | plastic | frozen    | **0.143** | **3.45 m** | **49**  | 96.7  | 0/5         |

Two empirical claims fall out cleanly:

**Claim 1 (Plasticity asymmetry).** EPM plasticity does not measurably affect
Premotor motor output. F0 vs F0' showed bit-identical Premotor speeds (0.113
vs 0.114 m/s) and bit-identical per-leg gait frequencies. GNG node churn
produces stable embeddings as long as the input distribution is stationary —
the consolidated state of an EPM is *emergent from sufficient exposure*, not
externally triggered.

**Claim 2 (Reward-supervised collapse is plasticity-quadrant-invariant).**
Cognitive PremotorAI's REINFORCE policy collapsed to one extreme intent in
4/5 seeds across BOTH F5 (hierarchical plasticity) AND F6 (stable features +
plastic policy). The collapse direction varied between arms (F5: 4 seeds
collapsed to intent 0; F6: 4 seeds collapsed to intents 1 and 3), but the
*phenomenon* was invariant. No plasticity quadrant prevented collapse.
Navigation (`walk_visit_count`) was 0/5 in every arm — the substrate has
locomotion competence, but no plasticity gating alone redirects it toward
a target.

The 2×2 matrix is exhaustive evidence that **the navigation bottleneck is at
the cognitive policy level, not at any plasticity-scheduling level.**

## A.2 The asymmetry the original paper implicitly assumed

The original paper (§ 2.2) describes homeokinesis as the System 1 substrate's
intrinsic drive, and (§ 3.1) describes EPMs as "limited-capacity experts
that subscribe to the VL-JEPA latent bus." It does not distinguish between:

- **Self-supervised plasticity** (EPMs, foundational encoders, lateral voter
  Hebbian trust). Optimized to match the world's statistical structure. The
  optimization target is *fixed by the world*, so gradients shrink as the
  module's representation converges. Convergence is the consolidation trigger
  — no external gating mechanism required.

- **Reward-supervised plasticity** (Premotors, value heads, eventually
  goal-directed planners). Optimized to maximize a reward signal that the
  policy's own actions influence. Feedback loop: policy → action → state
  distribution → reward distribution → policy gradient → policy. The natural
  dynamic is *mode collapse* (one intent's probability mass grows, gradient
  magnitude grows with it via `(𝟙{chosen} − p)`, exploration over). Without
  external counter-pressure, the policy commits to one extreme regardless
  of the underlying reward landscape.

This is a structural asymmetry, not a parameter difference. The F0 vs F0'
result demonstrates the first half (EPM plasticity is benign). The F5/F6
results demonstrate the second half (Premotor REINFORCE collapses
intrinsically). Treating them with a single uniform plasticity policy — as
all prior picrawler experiments did — produces the observed catastrophic
failure mode.

## A.3 The Mitosis Gatekeeper's dual: Consolidation by Convergence

§ 3.3 names the **Spawning Trigger** that creates new EPMs when novelty is
high. The dual is **Consolidation by Convergence** — and crucially, it is
*not symmetric across module classes*:

- For self-supervised modules: consolidation requires no explicit trigger.
  The Lateral Voter's trust-share mechanism (§ 4.1) already operates as the
  system-level gating signal. A module whose predictions match observations
  gains trust, gains influence, and naturally dampens its own learning
  signal as its TLE drops. The architecture as originally specified is
  *already correct for the self-supervised side*. EPMs in the picrawler
  substrate confirm this empirically: after several minutes of exposure to
  stationary stand-only proprio input, EPM outputs are indistinguishable
  with `epsilon_b=0.05` (plastic) vs `epsilon_b=0` (frozen). The
  consolidation is emergent.

- For reward-supervised modules: explicit consolidation IS needed, but the
  *form* of consolidation should not be a uniform external freeze trigger.
  Instead, the addendum prescribes an architectural move — a Cognitive
  Premotor Population — that makes reward dynamics *self-stabilizing*
  in the same fixed-point sense that EPMs are.

## A.4 The architectural move: Cognitive Premotor Population with Predictive Trust

The picrawler v6 cognitive layer is currently a *single* PremotorAI at level=10
fed by a slow voter aggregating slow EPMs. It learns via REINFORCE on the
task reward signal. This is the unstable configuration the F5/F6 results
exposed.

The addendum prescribes the *direct architectural twin* of the EPM →
Lateral Voter → Consensus pattern, applied recursively at the cognitive
level:

**A.4.1 Multiple cognitive Premotors as a population.** Replace the single
cognitive PremotorAI with a population of cognitive Premotors, each
specializing in a coarse strategy (e.g., per-direction steering biases;
per-task affordances; per-modality emphasis). Local mode collapse is *expected
and allowed* inside each Premotor — the population's diversity is what
matters, not any individual Premotor's exploration.

**A.4.2 A cognitive-level Lateral Voter.** Aggregate the cognitive Premotor
population through the same Lateral Voter mechanism § 4.1 specifies for
perceptual EPMs. Each cognitive Premotor publishes both its action proposal
and its *prediction* (next leg event timing, next chassis velocity, next
target_compass change — whatever is locally observable and bounded).

**A.4.3 Trust gated by predictive accuracy, not by reward.** The Lateral
Voter's trust-share is updated by the same Hebbian-on-prediction-error
rule it uses at the perceptual level. A cognitive Premotor whose predictions
match outcomes gains influence; one whose predictions are systematically
wrong (e.g., because it has collapsed to an intent inappropriate for the
current state) loses influence. This converts the cognitive-level
optimization from "maximize reward" (feedback-loop unstable) to "maintain
accurate predictions while the underlying REINFORCE provides bias toward
behaviors that work." The fixed-point dynamic that stabilizes EPMs is
*structurally identical*: optimize against something that doesn't move in
response to the optimizer.

**A.4.4 Homeokinesis as the cognitive-level intrinsic drive.** Per § 2.2,
the substrate is already driven by Time-Loop Error minimization. The
addendum prescribes that the *same principle* drives the cognitive layer,
applied to slow-stream predictions rather than per-tick perception. The
cognitive Premotor population's effective `mc_lr` scales with the
cognitive-level TLE — when slow-stream predictions are accurate (low TLE),
the cognitive policies settle into stable specializations; when the
environment changes (TLE rises), learning re-engages. This is the same
self-modulating drive § 2.2 names for System 1, extended into the System 2
deliberative supervisor.

**A.4.5 Reward as bias, not target.** External task reward (events.hit,
proximity-to-target, leg_event quality) still updates each cognitive
Premotor's W via REINFORCE — but now as a *secondary modulator on top of
the predictive trust mechanism*. The Lateral Voter selects whose policy
gets influence based on predictive accuracy; the REINFORCE updates merely
shape the policy distribution inside each Premotor. The catastrophic
feedback loop is broken at the system level: even if individual policies
collapse, the population's effective consensus tracks the policies whose
*predictions* are currently most accurate, which is necessarily *not* a
collapsed argmax (a collapsed policy predicts the same thing regardless of
state, and is wrong as soon as the state changes).

## A.5 Why this is the same principle, applied recursively

The original paper's elegance is that EPMs + Lateral Voter + Mitosis
Gatekeeper form a *self-organizing autopoietic system* for perception. The
addendum's claim is that this is not just a perceptual mechanism — it is the
*general architectural pattern for stable continual learning* in any domain
where the optimization target has feedback-loop instability.

The picrawler v6 F0–F6 battery is the empirical case that closes the
argument. The paper's Mitosis Gatekeeper (Spawn Trigger) handles autopoietic
expansion; the Lateral Voter's trust-share (already specified) handles
self-supervised consolidation; and the cognitive Premotor population + its
own Lateral Voter (NEW) handles reward-supervised stabilization. All three
mechanisms use the same Hebbian-on-prediction-error update rule, applied at
different timescales and to different signal types.

The system at full architectural completeness has three temporal/abstraction
regimes, all governed by the same principle:

- **Per-tick perceptual EPMs**: self-supervised via input distribution
  matching, consolidated by GNG convergence + Lateral Voter trust-share
- **Slow cognitive Premotor population**: reward-modulated locally,
  trust-aggregated globally via cognitive-level Lateral Voter and
  cognitive-level homeokinetic drive
- **Mitosis events**: spawn new EPMs or new cognitive Premotors when
  high-order entropy signals a paradigm shift, allowing the architecture
  to grow into new task spaces without overwriting existing competencies

## A.6 Implication for the picrawler v6 substrate (concrete next move)

The picrawler v6 architecture should not attempt to "fix" the cognitive
collapse with anti-collapse mechanisms inside a single Premotor (the N1–N8
knob exhaustion phase tested and falsified that approach). Instead:

1. Build 3–4 cognitive PremotorAI modules, each subscribed to the same
   slow consensus but specializing via initial bucket-bias or per-direction
   intent_accel masking
2. Add a cognitive-level LateralVoter aggregating their action proposals
3. Wire each cognitive Premotor's slow-stream predictions into a TLE
   computation; use TLE to modulate Hebbian trust-share updates
4. Per § A.4.3, the body publishes the predicted slow-stream observables
   one window later, closing the prediction-error loop

The architectural cost is one new LateralVoter instance + 2–3 additional
PremotorAI modules + per-Premotor prediction hooks. The benefit is that
single-policy collapse becomes structurally impossible to express — there
is no longer a single policy to collapse, only a population governed by
the same predictive-trust mechanism that already stabilizes the perceptual
side of the architecture.

## A.7 What this addendum does NOT prescribe

It is worth being explicit about what is intentionally left open:

1. The number and shape of cognitive Premotor specializations. We do not
   commit to a fixed count (3? 5? task-dependent?). The Mitosis Gatekeeper
   should grow the population based on high-order entropy signals at the
   slow stream, analogous to how it grows EPMs at the perceptual level.

2. The specific slow-stream observable each cognitive Premotor predicts
   against. The principle is "predict something measurable at the slow
   timescale that isn't controlled by the policy itself," but the exact
   observable is a design choice (next-window mean foot_y? next-window
   target_compass shift? next-window leg_event count?). Empirically informed
   by future battery arms.

3. The relative weight of REINFORCE-internal updates vs predictive-trust
   gating. Both are present; the balance is a hot-mutable parameter we
   tune adaptively (per the no-tuning directive, ideally adaptive via
   the cognitive TLE band itself).

These will be settled by the next experimental phase, which begins where
this addendum's prescription begins: cognitive Premotor population
construction.
