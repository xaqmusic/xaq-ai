> **Ported from the pre-split `ami-ogma` repo, 2026-07-25.** A plain-language glossary of the
> conceptual stack, written for a general-but-technical audience. Useful as onboarding for anyone
> — human or agent — new to active inference as a design pattern. Terminology note: it predates
> the rename, so `AMI-Ogma`/`ogma` == **xaq** (see [`../AGENTS.md`](../AGENTS.md)); and it predates
> the Cell rebuild, so its architectural examples may not match current module names. The
> *concepts* are current; verify any specific claim about the code against the source.

# AMI-Ogma Glossary
### Terms, Abbreviations, and Jargon for the "Climbing the Active Inference Mountain" Series

**Audience:** Someone who listens to AI podcasts and understands the landscape but is not a mathematician or ML researcher. Definitions favor intuition and analogy over precision.

---

## Part 1 — The Big Ideas

### Active Inference
The theory that intelligent behavior is a single process: an agent constantly predicts what its senses will report next, and acts to make those predictions come true. Perception and action are two sides of the same coin — both are in service of minimizing the gap between what the brain expects and what actually happens.

The theory was developed by **Karl Friston** at University College London and is grounded in Bayesian probability and thermodynamics. It is the most mathematically unified theory of brain function that currently exists — it claims to explain perception, learning, attention, emotion, and motor control as a single optimization.

In practice, building a system that actually does this is very hard. Most of this series is about why.

**Learn more:** Karl Friston's introductory paper "The free-energy principle: a unified brain theory?" (Nature Reviews Neuroscience, 2010). Lex Fridman's interviews with Karl Friston. The *Active Inference* book by Parr, Pezzulo & Friston (MIT Press, 2022).

---

### Free Energy
The quantity that active inference systems minimize. Technically: an upper bound on the "surprise" of sensory observations given the agent's internal model of the world.

In plain English: how wrong is the agent's model of the world right now? Low free energy = the model fits well. High free energy = something unexpected is happening.

The term comes from physics (thermodynamics) and information theory, not engineering. This causes endless confusion. When someone in this series says "minimizing free energy," read it as "reducing the gap between prediction and reality."

---

### Predictive Coding
A specific implementation of active inference ideas for perception. The brain (or a neural network) maintains a prediction of what the next sensory input will be. It subtracts that prediction from the actual input. Only the *error* — the difference — gets passed upward.

**The audio analogy:** Noise cancellation headphones work this way. The headphone predicts the incoming ambient noise and generates an inverted copy to cancel it. What you hear is only the part the prediction got wrong — the music. Predictive coding does the same thing with perception: what bubbles up to conscious awareness is the surprising part, not the predicted part.

**Learn more:** Rao & Ballard, "Predictive coding in the visual cortex" (Nature Neuroscience, 1999). This is the paper that brought predictive coding into computational neuroscience.

---

### JEPA — Joint Embedding Predictive Architecture
Yann LeCun's framework for self-supervised learning. Instead of predicting the raw next frame of a video (which is computationally expensive and requires predicting every pixel), JEPA predicts in *latent space* — the compressed, abstract representation of the world.

**The key insight:** You don't need to predict exactly what the next video frame looks like. You need to predict the abstract structure — "the ball will be in roughly this region, moving in roughly this direction." JEPA learns to predict abstractions, not specifics.

This is why AMI-Ogma does not reconstruct audio waveforms or video pixels. All learning happens in latent space.

**Learn more:** LeCun's 2022 paper "A Path Towards Autonomous Machine Intelligence." His presentation "Objective-Driven AI" at various venues — search for it on YouTube. The term is also discussed at length on the Machine Learning Street Talk podcast.

---

### Homeokinesis
A theory of intrinsic motivation for robots, developed by **Ralf Der and Georg Martius** (Max Planck Institute). The core idea: an agent should be driven to maximize the *sensitivity* of its own sensorimotor loop — how much a small motor action changes the sensory outcome.

A robot sitting still has zero sensitivity: nothing it does matters to its sensors. A robot on the edge of doing something interesting has high sensitivity. Homeokinesis pushes the robot to stay at that edge — the "edge of chaos" where small actions have big, interesting consequences.

The companion concept is **homeostasis**: the drive to keep internal state within acceptable bounds (temperature, energy, balance). Homeokinesis + homeostasis = explore, but don't break yourself.

**Learn more:** Der & Martius, *The Playful Machine* (Springer, 2012). Georg Martius's talks on YouTube. The book is the direct inspiration for the AMI-Ogma exploration architecture.

---

### Morphological Computation
The idea that the body itself is part of the cognitive system — not just a passive vessel for the brain's instructions, but an active participant that solves problems the brain never has to touch.

A classic example: passive dynamic walkers. These are simple mechanical toys — no motor, no sensor, no controller — that walk down a gentle slope purely through the geometry and compliance of their limbs. The "computation" of taking a step is performed by the shape of the leg, the springiness of the knee, and gravity. No brain required.

In biological organisms this scales up dramatically. A walking insect does not have a brain large enough to coordinate all the moment-to-moment details of six-legged locomotion. Instead:
- Compliant tendons absorb foot impacts without neural involvement
- Load sensors in each leg automatically extend the stance phase when weight increases
- The mechanical coupling through the shared body — when one leg pushes, every other leg's load shifts — transmits coordination information without a single neuron firing

Holk Cruse's six rules of coordination (see the Cruse entry in this glossary) demonstrate this precisely: the gait pattern of a stick insect emerges from local rules plus body physics, not from a central program.

**The relevance to AMI-Ogma:** A perception-heavy AI architecture trying to solve gait coordination through richer sensory representation is, from a morphological computation perspective, solving the wrong problem. The coordination that biology achieves through body structure and local reflexes cannot be replicated by adding more EPMs. It requires architectural changes on the action side — inter-joint coupling rules, physically-grounded reflexes, and mechanical priors that mirror what the body would provide on real hardware.

Put plainly: some of what looks like a hard cognition problem is actually a hard embodiment problem. And some of what looks like a hard embodiment problem is solved for free by the right body design.

**Learn more:** Pfeifer & Bongard, *How the Body Shapes the Way We Think* (MIT Press, 2007) — the definitive readable introduction. Hauser et al., "What is Morphological Computation?" (*Artificial Life*, 2011).

---

### The Dark Room Problem (also: The Lazy Robot Problem)
The central paradox of active inference. If an agent's goal is to minimize prediction error, the mathematically optimal strategy is to find the most predictable environment possible and stay there. A dark, silent room is perfectly predictable. A robot that doesn't move is perfectly predictable.

This is not a theoretical curiosity. Every active inference implementation has to solve it. AMI-Ogma's solution is homeokinesis: make *low* prediction error aversive (boredom) rather than rewarding.

---

### Hebbian Learning
"Neurons that fire together, wire together." The rule, proposed by Donald Hebb in 1949, is the basis for the Lateral Voter's association matrix. If two signals are consistently active at the same time, strengthen the connection between them. If they are consistently not co-occurring, weaken it.

No gradient descent. No external reward. Just: do these things happen together? 

**The audio analogy:** When you mix a record, you learn that the kick drum and bass guitar need to occupy the same frequency space. Your brain wires together the habit of sidechain compression — notch the bass when the kick hits. Hebbian learning is that same "these things belong together" intuition, formalized.

**Learn more:** Hebb, *The Organization of Behavior* (1949). Peter Dayan & Larry Abbott, *Theoretical Neuroscience* (MIT Press, 2001) — Chapter 8 covers Hebbian plasticity in detail.

---

### Tabula Rasa
Latin for "blank slate." A system that starts with no prior knowledge — no pre-trained weights, no pre-loaded patterns — and must learn everything from its own experience. AMI-Ogma always runs tabula rasa. Every experiment starts from scratch.

This is deliberately hard. It is also the honest test of whether the architecture works: can the system learn to stand, walk, or navigate from zero, using only its own sensory experience?

---

## Part 2 — The AMI-Ogma Architecture

### AMI — Audio Machine Intelligence
The machine learning division at Bongiovi Media and Technology, founded and led by Joseph Butera III. Originally a supervised learning framework for audio processing in medical and industrial applications. AMI-Ogma is its evolution — AMI without supervised learning, built on active inference instead.

---

### Ogma
The Celtic god of eloquence and knowledge, credited with inventing the Ogham alphabet — the earliest writing system used in Ireland. The name was chosen because the system builds its own symbolic vocabulary (the knowledge graph) from raw experience, without being told what anything means.

---

### EPM — Episodic Predictive Module
The core perceptual unit of AMI-Ogma. Each EPM is a self-contained module that:
1. Encodes raw sensory input into a compressed representation (the encoder)
2. Predicts what that representation will look like next tick (the predictor)
3. Accumulates patterns into a growing knowledge graph
4. Crystallizes reliable patterns into permanent episodic memory

The system can have multiple EPMs running in parallel, each specialized for a different sense: audio (Cochlear EPM), body tilt (IMU EPM), joint angles (Joints EPM), visual orientation (Compass EPM), etc.

**The audio analogy:** Each EPM is a channel strip on a mixing console. It processes one input, has its own gain and EQ (encoder), and feeds a shared mix bus (the voter). You can add a channel without rewiring anything else.

---

### TLE — Time-Loop Error
The EPM's primary learning signal. How wrong was the predictor's prediction? The difference between "what I expected to sense next" and "what I actually sensed."

Low TLE = the EPM has a good model of this sensory territory.  
High TLE = something unexpected happened — pay attention.

The *Playful Machines* principle says TLE should also drive exploration: low TLE means the territory is boring, so go find somewhere with higher TLE.

---

### Lateral Voter
The multimodal fusion layer. When multiple EPMs are running simultaneously, each one produces a latent vector representing its current best understanding of the world. The Lateral Voter fuses these into a single shared "Reality Token" — a consensus state of what the whole system currently believes.

The fusion is not a simple average. The voter maintains a Hebbian association matrix that learns which EPMs tend to agree with each other and up-weights their combination. EPMs that make consistent predictions earn higher trust.

**The audio analogy:** A summing bus with per-channel VCA faders, where the fader levels are set automatically based on how reliably each channel has been tracking the mix. A channel that's been consistently useful gets more gain. A channel that's been noisy gets attenuated.

---

### Reality Token
The shared multimodal state that passes through the system at every tick. It contains the fused latent vector from the Lateral Voter, plus neurochemical signals (arousal, valence) and metadata. Every cognitive module reads from and writes to the Reality Token each tick.

**The audio analogy:** The main stereo bus. Everything feeds into it; everything reads from it.

---

### Premotor
The policy learning module. At each tick, the Premotor reads the current Reality Token (what the system believes about its state), selects an intent (which of N possible action directions to pursue), and emits an action signal to the body.

The Premotor learns via Hebbian REINFORCE: actions that coincide with positive reward strengthen their connection to the current state. Actions that coincide with negative reward weaken it. Over time, the Premotor develops a policy: a habit of choosing intents that produce good outcomes in familiar states.

---

### ActionDecoder
An earlier policy module (used before the Premotor) that attempted to estimate the value of actions using a lookup table approach. Exposed a critical sign-bias flaw and was superseded by the Premotor.

---

### GNG — Growing Neural Gas
An unsupervised clustering algorithm that grows its own structure as data arrives. Start with two nodes. Each new data point moves the nearest nodes toward it. When a region gets too much traffic, a new node is inserted. When a region goes quiet, nodes are pruned.

The result is a topology map: a network of nodes that reflects the actual structure of the data, with no fixed architecture decided in advance.

AMI-Ogma uses GNG as the knowledge graph that grows inside each EPM. It is also used in SeqGNG for temporal sequences.

**Learn more:** Fritzke, "A growing neural gas network learns topologies" (NIPS, 1995). Conceptually similar to Kohonen Self-Organizing Maps, but the network size is not fixed.

---

### SeqGNG — Sequential Growing Neural Gas
GNG extended to learn *sequences* rather than static clusters. Instead of asking "what patterns exist in this data?" it asks "what patterns tend to follow what other patterns?" Used for learning motor chunks — recurring sequences of joint movements.

---

### CPG — Central Pattern Generator
A neural circuit (biological or artificial) that produces rhythmic output without requiring rhythmic input. Walking animals have CPGs in their spinal cords that generate the gait pattern; the brain just says "walk faster" or "walk slower" without specifying each leg movement.

AMI-Ogma experimented with an artificial CPG to provide a rhythmic scaffold for the quadruped's walking. It produced a net-negative result — the handcrafted rhythm interfered with the brain's own emerging patterns.

---

### IMU — Inertial Measurement Unit
A sensor that measures acceleration and rotation — the body's sense of which way is down and whether it's moving. In the PiCrawler, the IMU tells the brain how tilted the chassis is and whether it's falling.

The IMU EPM turned out to be the most critical perceptual input for standing balance: 91% upright with IMU, 28% with joint angles alone.

---

### Neurochemical State
A vector of scalar signals representing the system's internal drive state. Includes valence (positive/negative, roughly: is this going well?) and arousal (roughly: how much should I care right now?). These signals modulate the behavior of other modules — high arousal increases exploration noise, positive valence reinforces the current policy.

---

### Crystallization / Baking
The process by which a pattern in the GNG knowledge graph becomes a permanent episodic memory. A node that has been visited frequently, consistently, and with low prediction error "bakes" — it is promoted from temporary cluster to long-term memory. Unreliable nodes are pruned.

**The audio analogy:** A sample that has been triggered reliably enough in the same musical context gets saved to the sample library. One-off accidents get deleted.

---

### Chunk
A crystallized motor sequence — a recurring multi-tick pattern of joint movements that has been observed often enough and reliably enough to be worth storing. The intended mechanism for building a vocabulary of locomotion primitives (lift, press, stride). Getting chunks to actually crystallize tabula-rasa on the quadruped is an open problem as of Phase 7.

---

### Modality
A single sensory channel — one type of input to the system. Audio is a modality. IMU (body tilt) is a modality. Joint angles are a modality. Visual orientation (compass direction) is a modality. Each modality gets its own EPM.

---

### Consensus
The fused output of the Lateral Voter — the shared belief state that emerges from combining multiple EPMs. "Building consensus" means the voter is integrating signals from multiple modalities into a single coherent representation of what the world is doing right now.

---

### Latent Space
The compressed, abstract representation inside a neural network, between the input (raw sensors) and the output (actions or predictions). AMI-Ogma never tries to reconstruct raw sensor data — all learning happens in latent space. The latent vector represents the *meaning* of an experience, not its raw form.

**The audio analogy:** A compressor's gain-reduction signal is a latent representation of the audio's dynamics — it captures the essential information (how loud is it?) without storing the actual waveform.

---

### Knowledge Graph
The network of nodes and edges that the GNG builds over time. Each node represents a cluster of similar experiences. Edges connect experiences that tend to follow each other. The graph is the system's accumulated model of its world — not trained from a dataset, but grown from its own experience.

---

## Part 3 — The Learning Algorithms

### REINFORCE
A classic reinforcement learning algorithm. At the end of a sequence of actions, look back at what happened, and adjust the probability of each action up or down based on whether the outcome was good or bad. Simple, general, and has a well-known weakness: it credits all actions in a sequence equally, even if only one of them was actually responsible for the outcome.

In AMI-Ogma's quadruped, REINFORCE credits all 12 Premotors equally for every reward event. This is the credit assignment bottleneck identified in Phase 7.

**Named after:** The algorithm's formal name is the "REINFORCE" policy gradient method. Williams, "Simple statistical gradient-following algorithms for connectionist reinforcement learning" (Machine Learning, 1992).

---

### Eligibility Traces (λ — lambda)
An extension to REINFORCE that addresses the credit assignment problem. Instead of assigning credit only at the end, eligibility traces maintain a fading memory of recent actions. Actions that happened recently get more credit for the current reward; actions that happened a long time ago get less.

The λ (lambda) parameter controls the decay rate:
- λ = 0: only the most recent action gets credit (reactive, good for fast-feedback tasks)
- λ = 0.95: credit fades slowly over ~20 ticks (good for tasks where the cause and effect are separated in time)
- λ = 1.0: all past actions get equal credit (Monte Carlo returns)

For the quadruped, where a walking burst takes 2+ seconds, λ closer to 0.95 is theoretically correct — the cause (a good leg movement pattern) and the effect (outward displacement reward) are separated by hundreds of milliseconds.

---

### MC — Monte Carlo (Returns) / MC Mode
A reinforcement learning update rule that uses the *actual observed total reward across an entire episode* as the credit signal — no intermediate estimate, no bootstrap. The agent acts, the episode plays out, and only at the very end does the policy learn: "this action sequence produced this total return; reinforce it accordingly."

Contrast with TD (next entry), which updates after every tick using a moving estimate. MC waits for ground truth. TD pays for speed with bias.

In AMI-Ogma's quadruped, the Premotor runs in **MC mode**: rewards accumulate across an episode (e.g., 25 seconds), and the policy update fires once at episode-end. This is well-matched to the reward structure — standing reward is dense per tick, but walking and gait-cycle rewards are sparse and delayed (you can't tell whether a gait pattern is producing forward progress until several seconds of motion play out). Letting the full episodic return arrive before crediting actions removes the bias that a per-tick TD update would introduce.

**The practical implication — Pattern C in the registry:** The Premotor has *two* reward paths in its codebase — `apply_reward` (TD / per-tick) and `finalize_mc_episode` (MC / end-of-episode). PiCrawler uses MC only. Any mechanism wired to the `apply_reward` path is dead code on picrawler. Several experimental mechanisms — `eligibility_lambda`, `baseline_lr` — silently did nothing for entire phases because they were attached to the wrong path. The fix is not a code change but a diagnostic discipline: every new mechanism must verify which reward path is live under the current config.

**The audio analogy:** MC is like mixing a song — you don't decide whether a take was good after each bar; you let it play through and judge the whole performance. TD is like a live engineer pulling faders moment-to-moment. Both are valid; they serve different production styles.

**Learn more:** Sutton & Barto, *Reinforcement Learning: An Introduction* (MIT Press, 2nd ed. 2018), Chapter 5 ("Monte Carlo Methods") vs Chapter 6 ("Temporal-Difference Learning") — the canonical side-by-side comparison.

---

### TD — Temporal Difference Learning
A family of algorithms that update value estimates tick-by-tick, without waiting for the end of an episode. "How much better or worse is my situation now compared to what I predicted?" The difference between the expected value and the actual value is the TD error, which drives learning.

TD(λ) combines temporal difference learning with eligibility traces. TD sits at the opposite end of the time-horizon spectrum from MC (above): MC waits for full ground truth; TD updates immediately from a bootstrap estimate. Most modern RL sits somewhere in between — TD(λ) with `λ` between 0 and 1 blends the two.

---

### EFE — Expected Free Energy
The active inference version of "value." Instead of asking "how much reward do I expect from this action?", EFE asks "how much will this action reduce my uncertainty about the world while also satisfying my preferences?" It combines epistemic value (learn something) with pragmatic value (get something).

---

### Actor-Critic
An architecture where two modules work together: the *actor* selects actions, and the *critic* evaluates how good the current situation is. The critic provides a baseline — "this is what a typical outcome looks like" — so the actor knows whether a particular outcome was actually above or below average. Without a critic, the actor may reinforce actions that got positive reward even when that reward was below average for the situation.

AMI-Ogma's Premotor is heading toward an actor-critic architecture. The missing piece as of Phase 7 is a reliable baseline (critic) signal.

---

### BC — Behavioral Cloning
A supervised learning technique applied to reinforcement learning: copy the behavior of a good policy by treating it as labeled training data. In AMI-Ogma, behavioral cloning attempted to imprint the body reflex's decisions onto the Premotor — "do what the reflex does, but learn to do it yourself." Multiple implementation bugs made it silent for several experimental phases.

---

### EMA — Exponential Moving Average
A running average that weights recent values more than old ones. Used throughout AMI-Ogma to smooth noisy signals: prediction error EMA, speed EMA, reward attribution EMA. The decay rate controls how much history is remembered.

---

### Pre_w_growth
A diagnostic metric unique to AMI-Ogma. Measures how much the Premotor's weight matrix changed during a run — a direct readout of whether the policy learner is actually making updates. 

pre_w_growth ≈ 1.0: weights barely changed (Premotor is idle or stuck)  
pre_w_growth = 33–48: real learning happening  

This metric was crucial for proving that the brain was learning in the Phase 6 and Phase 7 experiments, even when behavioral improvements were not visible.

---

### Paired-Seed A/B
AMI-Ogma's experiment methodology. Two configurations (A and B) are run with identical random seeds — the same sequence of random numbers controlling all stochastic behavior. Any difference in outcome is due to the architecture, not luck. Running n=20 paired seeds gives enough statistical power to detect moderate effect sizes.

This discipline was adopted after discovering that single-seed results were frequently misleading.

---

### Bernoulli-Impulse Actuation
A method for converting continuous motor commands into a stream of discrete impulses — the body becomes a physical integrator instead of a position tracker. The brain emits a continuous scalar (e.g. "accelerate forward at 0.7"). The body, on each physics tick, converts this to a *spike rate* — a probability that the actuator fires this tick. A coin is flipped against that probability (the Bernoulli sample). On fire: apply a fixed impulse to the actuator's velocity. On no-fire: nothing. Friction or natural damping decays the integrated velocity each tick.

The result: the body's smooth motion *emerges from the integration of many discrete impulses*, not from the smoothness of the brain's commands. This matches how biological motor units actually work — your motor neurons don't send smooth analog signals; they send rate-coded discrete spikes that your muscles integrate into force.

**The audio analogy:** This is exactly how Class-D amplifiers work. Instead of producing a smooth output voltage to match the audio signal, they generate a high-frequency stream of full-power pulses (+V or -V), where the *duty cycle* of pulses encodes the signal level. A low-pass filter (the speaker coil + driver inertia) integrates the pulses back into smooth audio. Bernoulli-impulse actuation does the same for motor control: discrete pulses, integrated by physics into smooth motion.

**The body-as-integrator hypothesis:** AMI-Ogma's v4 Phase 6.0.a demonstrated that converting smooth motor commands into Bernoulli-impulse spikes *improved* learning on hard tasks (gradient-desert chemotaxis: 20% → 35% find rate). The intuition is that momentum carries the body through low-signal regions where a perfectly-tracking controller would freeze waiting for clearer input. Stage 3.D extends this to picrawler's leg actuators.

Contrast with LIF (Leaky Integrate-and-Fire) neurons in computational neuroscience, which use a similar spike-emission idea but with an internal membrane potential that integrates input and fires when threshold is crossed. Bernoulli-impulse is the simpler cousin — fire probability is a direct function of commanded rate, no membrane state.

**Learn more:** "Spiking neural networks for embodied robotics" literature — search Wolfgang Maass or Eugene Izhikevich. The body-as-integrator framing is also discussed in Rolf Pfeifer's *How the Body Shapes the Way We Think* (already referenced under Morphological Computation).

---

## Part 4 — The Math Symbols

### λ (lambda)
The eligibility trace decay parameter. Controls how far back in time an agent assigns credit for a reward. See *Eligibility Traces* above. Also used as the general symbol for "decay rate" in several other AMI-Ogma contexts.

---

### α (alpha)
Learning rate — how big a step the system takes when updating its weights after each new piece of evidence. High α = learns fast but unstably. Low α = learns slowly but stably. Also used in AMI-Ogma for the bilateral fader blending parameter (how much brain vs. reflex controls the output).

---

### σ (sigma)
Standard deviation — the spread of a set of measurements. When experiment results say "σ=8", it means the noise level is high enough that detecting a real signal requires more samples. Also used as the symbol for experiment difficulty variants (e.g., the σ=8 Cell configuration with higher noise).

---

### τ (tau)
Time constant — how quickly a system responds to changes. In audio, the attack and release of a compressor are time constants. In AMI-Ogma, τ controls how quickly running averages and motor targets update. (Caution: in Godot/GDScript, `TAU` is also the mathematical constant 2π — the codebase has both, and they once collided in a bug.)

---

### β (beta)
Used in Bayesian statistics as the parameter for Beta distributions — the tool for representing confidence in a binary outcome (like "does this chunk work?"). A chunk with Beta(α=10, β=2) is highly confident it works. A chunk with Beta(α=1, β=10) expects failure.

---

### d (Cohen's d)
Effect size — how big is the difference between two groups in units of standard deviation? 
- d = 0.2: small effect (you need many samples to detect it)
- d = 0.5: medium effect (noticeable with reasonable sample sizes)
- d = 0.8+: large effect (obvious)

Used throughout the experimental results to characterize whether an architectural change made a meaningful difference.

---

### n
Sample size — how many independent trials were run. AMI-Ogma's methodology settled on n=20 paired seeds as the minimum for publishable results after a series of n=10 results that did not replicate.

---

## Part 5 — The People

### Karl Friston
Neuroscientist at University College London, creator of the Free Energy Principle and Active Inference framework. One of the most cited scientists alive. His work claims to unify perception, learning, attention, and action under a single mathematical framework. Interviewed extensively by Lex Fridman and others.

---

### Yann LeCun
Chief AI Scientist at Meta, one of the founders of deep learning (Turing Award 2018 with Hinton and Bengio). Proposed JEPA as a path beyond current deep learning toward more structured, prediction-based learning. Frequently argues that current LLMs are missing fundamental components of intelligence. Inspirational for this project's "latent space prediction" architecture.

---

### Ralf Der & Georg Martius
Researchers at the Max Planck Institute for Mathematics in the Sciences. Authors of *The Playful Machines* (2012), which provides the mathematical and experimental foundation for homeokinesis — intrinsic motivation through sensorimotor sensitivity. Directly inspirational for AMI-Ogma's exploration architecture.

---

### François Chollet
AI researcher at Google, creator of Keras. Most relevant here for his work on the ARC benchmark (Abstraction and Reasoning Corpus) and his writing on the limits of current deep learning. Argues that genuine intelligence requires systematic generalization that current architectures do not have. Frequently interviewed on Lex Fridman and Dwarkesh.

---

### Sarah Walker
Astrobiologist and physicist at Arizona State University. Develops Assembly Theory — a framework for measuring the complexity of objects based on how many steps are required to construct them. Relevant to AMI-Ogma through the question of how biological-like complexity emerges from simple rules. Interviewed on many podcasts including Lex Fridman.

---

### Blaise Agüera y Arcas
VP and CTO of Technology & Society at Google, leading the Paradigms of Intelligence team. Physicist by training (Princeton), inventor of federated learning, and one of the more philosophically interesting voices in AI.

Two bodies of work are directly relevant to this project:

**On intelligence and consciousness:** His 2022 essay "Do Large Language Models Understand Us?" (published in *Daedalus*, the journal of the American Academy of Arts and Sciences) argued — controversially at the time — that LLMs may exhibit genuine, if alien, forms of understanding. His broader body of writing explores what intelligence actually *is*, separate from the behavioral benchmarks we typically use to measure it. His 2025 book "What is Intelligence? Lessons from AI about Evolution, Computing, and Minds" extends this into evolutionary and thermodynamic territory.

**On BFF and emergence:** In his Nautilus essay "In the Beginning, There Was Computation," Agüera y Arcas writes about BFF — a cellular automaton system whose name is a riff on "Brainfuck," a minimalist programming language. BFF is a computational substrate so simple it can be described in a few lines, yet when you run it, self-replicating structures emerge spontaneously. Nobody designed them. Nobody told the system to produce life-like behavior. It arose from the rules.

The entropy connection: BFF is a demonstration of how *complexity* and *order* can emerge from *entropy* — from random starting states and simple local rules. The insight is that the arrow from chaos to structure does not require a designer; it requires only the right substrate and enough time. This is deeply relevant to AMI-Ogma's ambition: a system where gait, exploration, and knowledge emerge from the interaction of simple modules, not from explicit programming of those behaviors.

Agüera y Arcas sits at the intersection of AI capability, biological plausibility, and the philosophy of mind — exactly the territory this project occupies. His work is a useful reminder that the "what does it mean to be alive?" question is not poetic decoration; it is the precise scientific question this architecture is trying to make progress on.

**Learn more:** "Do Large Language Models Understand Us?" — *Daedalus*, 2022. "In the Beginning, There Was Computation" — *Nautilus*. His five TED talks, searchable by name.

---

### Holk Cruse
Neurobiologist at the University of Bielefeld, Germany. Best known for decades of meticulous experimental work on how stick insects walk — and what that reveals about the fundamental nature of coordination, cognition, and control in biological systems.

**The Six Rules of Coordination**

Cruse identified six local rules that govern how adjacent legs in a walking insect negotiate their timing. No central controller. No global gait pattern pre-computed by the brain. Each leg communicates only with its immediate neighbors through simple excitatory and inhibitory signals:

1. **Swing inhibits swing** — A leg currently in its swing phase (lifted, moving forward) prevents the leg next to it from starting its own swing. Two adjacent legs cannot lift simultaneously or the animal falls.
2. **Stance-start excites swing-start** — When a leg transitions into its stance phase (foot touching down, pushing), this signals the adjacent forward leg that it is now safe to begin its swing.
3. **Caudal position excites swing-start** — When a leg has been pushed far back (caudal = toward the tail), it triggers the signal to start its next swing. The leg itself knows when it is time to step, based on its own position.
4. **Targeting** — The current position of a leg influences where the leg in front of it aims to place its foot. Positional information flows forward along the body.
5. **Coactivation** — Increased load on a leg prolongs its stance phase and increases its force output. The leg adapts to what it is carrying without being told to.
6. **TOT reflex (treading-on-tarsus)** — A tactile reflex: if one leg steps onto another leg's foot, the lower leg immediately lifts to avoid the obstruction.

That is the complete coordination architecture. Six rules, all local, all operating simultaneously in parallel. From these rules — and crucially, from the physics of the body itself — a continuous range of gaits emerges: slow wave gait, fast tripod gait, curves, slopes, uneven terrain, recovery from leg loss. The brain's only job is to say "move forward" or "move backward." The gait patterns write themselves.

**WalkNet**

WalkNet is Cruse's computational implementation of these principles — a decentralized neural network where each leg is controlled by its own subnet, and the subnets communicate through the six rules plus feedback from the physical environment. WalkNet has been validated in simulation and on physical six-legged robots. It produces insect-like walking without any central pattern generator, without any explicit gait formula, and without a hierarchical controller.

**Why this matters for AMI-Ogma**

The Phase 7 arc finding was: *walking does not emerge from perception complexity alone.* Cruse's biology explains precisely why. The coordination problem in legged locomotion is not primarily a perception problem. It is not even primarily a brain problem. It is a *body* problem — solved by local inter-leg rules and by the mechanical properties of the limbs themselves.

A huge fraction of what we intuitively attribute to "the nervous system" in a walking animal is actually being performed by:
- The passive dynamics of compliant limb joints (spring-like tendons absorb impacts without neural involvement)
- Local proprioceptive reflexes (each leg's load sensor extends its stance automatically)
- Mechanical coupling between legs through the shared chassis (when one leg pushes, the body shifts, which changes the load on every other leg)

This is **morphological computation**: the body computes, not just the brain. The physical structure of the organism is part of the cognitive architecture.

For the quadruped: twelve independent Premotors, each sampling its own policy independently, with no mechanical coupling rules between them, is an architecture that asks the brain to solve a problem that biology solved by distributing it into the body and the inter-leg connections. Cruse's work is the biological ground truth against which the next phase of AMI-Ogma's action-side development should be measured.

**Learn more:** Cruse et al., "WalkNet — a Decentralized Architecture for the Control of Walking Behaviour Based on Insect Studies," *Biological Cybernetics* (2013). Cruse, "The Function of the Legs in the Free Walking Stick Insect" (1976). The EUCognition project's summary of Cruse's coordination rules is freely available online.

---

### Donald Hebb
Psychologist at McGill University, author of *The Organization of Behavior* (1949). Proposed the synaptic learning rule that bears his name: connections between neurons strengthen when they fire together. Foundational to the Lateral Voter's association matrix, the Premotor's policy update, and most biologically-inspired learning in the system.

---

## Part 6 — The Jargon

### DPS — Digital Power Station
Bongiovi Media and Technology's flagship audio processing technology. A suite of 50+ patents covering audio enhancement, intelligibility, hearing protection, and immersive audio across automotive, aviation, medical, and consumer applications. The professional context in which AMI was originally developed.

---

### LLM — Large Language Model
A neural network (typically transformer-based) trained on large text corpora to predict the next word. GPT-4, Claude, Gemini. Extremely capable at language tasks; not designed for real-time embodied learning. AMI-Ogma is explicitly not an LLM and is not attempting to do what LLMs do. The two architectures address fundamentally different regimes of intelligence.

---

### RNG — Random Number Generator
A software routine that produces a stream of numbers that look random but are actually deterministic — given the same starting value (the *seed*), the same stream is generated every time. Modern computers cannot generate true randomness; everything is pseudo-random. For research, this is a feature, not a bug.

**Why it matters in AMI-Ogma:** every stochastic decision the system makes — which intent the Premotor samples, which way the Bernoulli-impulse actuator fires, where the random whisker noise comes from — passes through an RNG. Seed the RNG with the same value, replay the same physics, and you get the same trajectory tick-for-tick. This is the foundation of *paired-seed A/B* methodology (see above): comparing two configurations that share an RNG seed means every observed difference is due to architecture, not luck.

The hidden discipline: every new stochastic mechanism that gets added to the codebase needs its own RNG, seeded as a deterministic function of the global seed. If a mechanism shares an RNG with something else, adding it perturbs the shared stream, and pre-mechanism A/B comparisons silently lose their pairing. The convention in AMI-Ogma is to derive sub-RNGs by hashing the global seed with a mechanism-name tag: `_my_rng.seed = OGMA_SEED.hash() ^ "my_mech".hash()`. This way, adding a new mechanism doesn't change the stream of any existing mechanism.

**The audio analogy:** Same as audio dithering. A good dither generator uses a pseudo-random stream to inject controlled noise that prevents quantisation artifacts — and if you re-dither the same source with the same seed, you get the same output bit-for-bit. The randomness is reproducible.

---

### ONNX — Open Neural Network Exchange
A standard file format for saving trained neural network models so they can be run in different programming languages and environments. AMI-Ogma trains in Python (PyTorch), exports to ONNX, and deploys in C++. The format is the bridge between the training environment and the real-time deployment environment.

---

### MLP — Multi-Layer Perceptron
A standard feedforward neural network: input → hidden layers → output, with no recurrence or attention. AMI-Ogma's predictors (the part that predicts next latent state from current latent state) are MLPs. Fast, bounded compute, no memory footprint beyond the weights.

---

### GRU — Gated Recurrent Unit
A type of recurrent neural network with internal memory — each step can read from and write to a memory cell. Used in earlier versions of AMI-Ogma's meta-EPM before being replaced by the Lateral Voter + GNG architecture.

---

### HDC — Hyperdimensional Computing
A computing paradigm that represents information as very high-dimensional binary vectors (thousands of dimensions). The key property: combining two HDC vectors produces a third vector that is distinct from both parents but can be decomposed back into them. Used in AMI-Ogma's consensus fusion layer for encoding structural relationships between EPMs.

**The audio analogy:** A stereo mid-side matrix. The mid (M = L+R) and side (S = L-R) vectors encode the full stereo image, and you can recover L and R exactly. HDC does the same thing in high dimensions with many more signals.

---

### SIGReg — Sketched Isotropic Gaussian Regularization
An AMI-Ogma-specific technique to prevent latent collapse — the failure mode where all the system's latent representations collapse to the same point (the encoder learns to output the same vector regardless of input). SIGReg checks, via random 1D projections, that the latent distribution stays spread out. O(D) complexity — fast enough to run every tick.

---

### EMA — Exponential Moving Average
See Part 3. Listed again here because it appears constantly in the logs and diagnostics.

---

### DAG — Directed Acyclic Graph
A network of nodes with directed edges and no cycles (no loops back to the start). AMI-Ogma's cognitive module graph is a DAG: the scheduler builds it from declared input/output topics and runs modules in dependency order each tick.

---

### AGC — Automatic Gain Control
An audio engineering term for a circuit that automatically adjusts signal level to stay within a target range. Used in the series as an analogy for AMI-Ogma's homeostatic drive — the module that keeps internal signals (arousal, valence) within useful operating ranges.

---

### Affordance
Borrowed from psychology (J.J. Gibson): an affordance is an action possibility offered by an environment. A door handle affords pulling. A flat surface affords walking. In AMI-Ogma, the "affordance ladder" is the experimental curriculum — environments and body configurations arranged in order of increasing difficulty, where each rung tests whether the system can exploit the opportunities the environment offers.

---

### Edge of Chaos
A concept from complexity theory: the regime between order (everything predictable, nothing interesting) and chaos (everything random, nothing learnable). Complex systems — biological brains, ecosystems, the best music — operate near this edge. Homeokinesis is designed to keep the sensorimotor system near its edge of chaos.

---

### Lyapunov Exponent
A measure from dynamical systems theory of how quickly two similar starting states diverge over time. Positive Lyapunov exponent = the system is chaotic (small differences grow exponentially). Zero = stable. The homeokinesis framework explicitly targets high Lyapunov exponent — high sensitivity to initial conditions — as the definition of "interesting" behavior.

---

### Egocentric / Allocentric
Two frames of reference for space. *Egocentric* is relative to the agent — "the wall is to my left." *Allocentric* is relative to the environment — "the wall is north." The compass EPM in Phase 7.5 provides an egocentric radial orientation signal: which direction is "away from the starting point" relative to the robot's current heading.

---

### Tabula Rasa
See Part 1. Listed again because it appears frequently in experiment descriptions.

---

### Curriculum
In machine learning, a curriculum is a sequence of increasingly difficult training tasks designed to scaffold learning. AMI-Ogma's PiCrawler curriculum: first learn to stand (no movement required), then learn to walk (distance reward introduced after 3 minutes of standing). The transition timing matters — too long in stage 1 and the brain attractor is too deep to escape.

---

---

## Part 7 — HUD and Diagnostic Metrics

*These are the abbreviations and field names that appear on screen during live runs, in the JSONL log files, and in the experiment analysis scripts. They are the instruments on the dashboard.*

---

### The Brain Line: `da= ht= H=`

The most-watched line on the HUD. Three real-time readouts of the brain's current state, updated every physics tick.

**DA — Dopamine**  
The neurochemical signal for reward. When the body achieves something the reward function values (staying upright, moving outward), the brain's NeurochemState module releases a dopamine pulse. The HUD shows the current value on a 0–1 scale. A flat DA line means no reward is arriving; a bouncing line means the reward channel is active.

In analysis scripts: `da_mean` (average across the run), `da_var` (how much it fluctuated), `da_max`, `da_min`.

The name is deliberately biological — dopamine in the human brain is the neurotransmitter most associated with reward prediction. The mapping is not exact but the intuition is right: more DA = things are going well from the brain's perspective.

---

**HT — Serotonin** *(displayed as `ht=`)*  
The neurochemical signal for stability and homeostasis. Serotonin rises when the body is in a calm, sustainable state — upright, not moving too fast, energy expenditure within bounds. It falls when things are chaotic.

The `ht` abbreviation comes from an older internal naming convention (homeostatic tone). The underlying variable in the code is `serotonin`. Like dopamine, the biological analogy is approximate: serotonin in humans is associated with mood stability and the sense that basic needs are met.

---

**H — Premotor Entropy** *(displayed as `H=`)*  
How spread out is the Premotor's intent distribution right now? Entropy (the H comes from information theory, specifically Shannon entropy) measures how uncertain the policy is.

- **H near ln(N)** (maximum): the Premotor is choosing randomly among all N intents — no strong preference. Cold-start or stuck state.
- **H near 0**: the Premotor is almost always choosing the same intent — a committed, deterministic policy.

Watching H drop over a run is watching the brain make up its mind. The HUD shows both the raw entropy and `ln(N)` — the theoretical maximum — so you can read commitment as a fraction.

In analysis scripts: `pre_H` (per-Premotor entropy, one value per joint).

---

### The Premotor Line: `W_norm= H= last_chosen=`

**W_norm — Weight Matrix Norm**  
The total magnitude of the Premotor's learned weight matrix. Starts near zero (tabula rasa, all weights small). Grows as the Premotor accumulates REINFORCE updates. A Premotor with large `W_norm` has made many learning updates and has strong policy preferences.

In analysis scripts: `pre_w_mean` (average W_norm across all Premotors), `pre_w_growth` (ratio of final to initial W_norm — the primary diagnostic for "is the brain actually learning?").

**last_chosen** — The index of the intent the Premotor selected on the most recent tick. Useful for spotting lock-in: if `last_chosen` never changes, the policy has collapsed to a single action.

---

### The Chassis Line: `y= tilt= leg×=`

**y** — Chassis height in meters. The quadruped's center-of-mass height above the floor. The FAIL_HEIGHT threshold is 0.025 m — below this and the body is considered fallen.

**tilt** — Chassis tilt angle. How far the body has rotated away from vertical, in radians and degrees. TIPOVER threshold: π/2 radians (90°) — at this point the chassis is on its side.

**leg×** — `leg_strength` multiplier. A scalar applied to all servo motor forces. Used during experiments to calibrate how much torque the simulated servos generate.

---

### The Speed Line: `now= ema= m/s  best=  sustained=  PRs=`

**ema** (speed EMA) — The exponential moving average of speed. Smooths out the raw per-tick noise so the trend is visible. If `now` is bouncing but `ema` is climbing, the robot is genuinely accelerating on average.

**sustained** — How many consecutive ticks the robot has held speed above a threshold (typically 0.5 m/s). The walking curriculum's stage-advance gate is defined in terms of sustained speed ticks — the robot must prove it can hold speed, not just spike it.

**PR / PR!** — Personal Record. Flashes on the HUD when the robot achieves a new best speed for the session. The `PRs=` count is how many personal records have been set. A rising PR count means the robot is still improving; a flat count means it has plateaued.

---

### The Mechanism Indicators: `HK  SWAP  ESC`

Three one-line status indicators for active intervention mechanisms. Each shows `[off]` (mechanism not loaded), `[idle]` (loaded and monitoring), `[FIRED]` (triggered recently), or `[ACTIVE]` (currently overriding normal behavior).

**HK — Homeokinetic Exploration**  
The entropy-collapse detector. Fires when the Premotor's intent distribution has been stuck at near-zero entropy for too long — the brain has locked into a single behavior and won't explore. When HK fires, it injects noise to break the lock-in. `arms=N` shows how many times it has triggered this session.

**SWAP — EPM Memory Swap**  
An experimental mechanism that replaces the current EPM's knowledge graph with a previously-saved state when the brain is stuck. The theory: maybe a different perceptual history would unstick the policy. `n=N` shows how many swaps have occurred.

**ESC — Escape Detector**  
A body-side detector that fires when the chassis has been in the same spatial region for too long, triggering a positional reset. Prevents the robot from wedging itself against a wall and learning nothing. `n=N` shows fire count.

---

### Stability Metrics: `pct_upright  pct_tipover  fall_events`

Aggregated over the run, reported in the end-of-run summary and in JSONL output.

**pct_upright** — Percentage of physics ticks where the chassis was above FAIL_HEIGHT and below FAIL_TILT. The primary standing performance metric. B0 achieved 91% upright; B1 achieved 28%.

**pct_below_FAIL_HEIGHT** — Percentage of ticks where the chassis y-position was below 0.025 m. The robot is on the floor (fallen or collapsed).

**pct_tipover** — Percentage of ticks where chassis tilt exceeded π/2 (90°). The robot is fully on its side or beyond.

**pct_high_tilt** — Percentage of ticks where tilt exceeded a "wobbling" threshold (less extreme than tipover). High pct_high_tilt with low pct_tipover = a robot that wobbles but recovers. The hierarchical EPM experiment showed lower pct_high_tilt but higher pct_below_FAIL_HEIGHT — "less wobbly, more collapsed."

**fall_events** — Number of times the chassis crossed from above FAIL_HEIGHT to below it. A robot that falls 5 times and recovers has 5 fall_events. Distinct from pct_below — frequency vs. duration.

**tipover_events** — Number of times the chassis crossed the π/2 tilt threshold. Catastrophic falls (fully inverted).

**longest_upright_physics_ticks** — The longest single continuous period the chassis remained upright. Converted to seconds for reporting (× TAU = × 0.02 s).

---

### Navigation Metrics: `max_distance  path_length  burst_efficiency`

**max_distance_from_origin** — The furthest the chassis ever got from its starting position during the run, in meters. The primary locomotion performance metric in Phase 7. Persistent across resets within a session.

**end_distance_from_origin** — Where the chassis ended up when the run stopped. Usually less than max_distance (the robot wanders back).

**total_path_length** — Total distance traveled by the chassis across the entire run, in meters. A robot that walks in circles can have high path_length and low max_distance. In the Phase 7.5 1-hour run: 163 m path length, 2.6 m max distance — the robot was very active but not going anywhere.

**burst_efficiency** — During a motion burst (a period of sustained speed), what fraction of the velocity was directed *away from the origin* vs. sideways or inward? A value of 1.0 means perfectly outward. A value of −1.0 means moving directly back to the start. The Phase 7.5 analysis found burst efficiency oscillating around 0.2 with no upward trend — the robot was generating motion but not consolidating outward direction.

---

### Reward Attribution: `standing=  walking=  gated=`

Three channels of reward, tracked separately so the experimenter can see which channel is driving the brain's dopamine pulses.

**standing** — Reward earned for being upright (chassis y above threshold, tilt below threshold). Pure stability reward.

**walking** — Reward earned for forward speed (velocity reward component). Encourages movement.

**gated** — Multiplicative bonus earned when both standing and walking criteria are met simultaneously. The Phase 7.5 design intent: make the gated bonus the dominant reward channel so the brain is driven to *walk while standing*, not just stand or just move. In the 1-hour run, gated reward accounted for 47% of total reward — larger than either standing or walking alone.

---

### Module Types (Brain Summary)

When the end-of-run summary lists the brain modules, each appears with its type name and key metrics:

**NeurochemState** — The neurochemical module. Reports `da=` (dopamine), `ht=` (serotonin), `r_sig=` (raw reward signal before neurochemical processing).

**EPM** — Each Episodic Predictive Module. Reports `nodes=` (current GNG node count), `baked=` (how many have crystallized), `tle=` (current prediction error).

**LateralVoter** — The fusion module. Reports `fused_tle=` (combined prediction error across all EPMs), `active_modality=` (which EPM is currently most active), `trust:` (per-EPM trust weights, e.g. `imu=0.31 joints=0.19`).

**DescendingPredictor** — The predictive arc module. Reports `loss=` (how well it's predicting future consensus states from the current state).

**HomeostaticDrive** — The drive module. Reports `urg=` (urgency — how far the system's internal state is from its setpoint) and per-channel errors.

**Premotor** — Each per-joint policy module. Reports `W_norm=` (weight magnitude), `H=` (current intent entropy), `last_chosen=` (last intent index).

---

### r_sig — Reward Signal

The raw reward value computed by the body each tick before it is processed into dopamine. This is the number the body decides to emit based on its reward function (stability + speed + gated bonus). The NeurochemState module converts this into a DA pulse, applying smoothing and scaling. `r_sig` in the brain summary tells you what the body sent; `da` tells you what the brain heard after processing.

---

### urg — Urgency

The HomeostaticDrive module's output. How far is the system's current state from its homeostatic setpoint? High urgency = the body is significantly off-balance (tilt, speed, energy) and the drive is pushing hard to correct it. Low urgency = homeostasis is satisfied, the drive is quiet.

Urgency is the active inference version of "how uncomfortable am I right now?" It feeds into the Premotor's action selection — high urgency biases toward corrective actions.

---

*This glossary will be updated as the project evolves. Last updated: 2026-06-01.*
