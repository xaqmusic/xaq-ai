# The Fractal JEPA Workspace: An Autopoietic Architecture for Open-Ended Continuous Learning

> **Status: aspirational / deferred.** This is a forward-looking position paper. The Fractal JEPA Workspace — and in particular the Mitosis Gatekeeper and the autopoietic node-spawning it describes — is a proposed research direction, not shipped Zanshin behaviour. It sketches where the architecture could grow, not what it does today.

**Abstract**
Current machine learning architectures struggle with open-ended continuous learning, often falling victim to catastrophic forgetting or prototype runaway when exposed to novel, high-dimensional environments. We propose a novel cognitive architecture that integrates Joint Embedding Predictive Architectures (VL-JEPA), homeokinetic intrinsic motivation, and Assembly Theory-inspired complexity metrics within a Global Workspace framework. By decoupling continuous perceptual processing from localized Episodic Predictive Memory (EPM) columns, and utilizing a "Mitosis Gatekeeper" based on high-order entropy, the system can autonomously balance between adapting existing knowledge and spawning new structural modules. This allows for fractal, unsupervised expansion of intelligence that supports both rapid, reactive System 1 exploration and slow, deliberative System 2 planning.

---

### 1. Introduction
The pursuit of Artificial General Intelligence requires systems capable of "self-actualization"—the self-determined unfolding of sensorimotor contingencies without explicit external goals. Standard Vision-Language Models (VLMs) operate in discrete token spaces, which requires expensive autoregressive generation and struggles with real-time, continuous state updates. Furthermore, scaling these systems to accommodate new paradigms often necessitates retraining massive parameter sets. 

We propose a "Publish-Subscribe Cognitive Bus" architecture. By utilizing non-generative representation learning (VL-JEPA) to publish continuous abstract embeddings, and governing behavior via the Homeokinetic Principle, we establish a robust System 1 substrate. We then introduce an epistemic memory layer (EPMs acting as cortical columns) governed by a Lateral Voter and Global Workspace (System 2). To manage unsupervised structural growth, we define a "Mitosis Gatekeeper" utilizing high-order entropy to detect emergent complexity and trigger the localized crystallization of new neural modules.

### 2. The Perceptual Kernel & System 1 Substrate
The foundation of the architecture relies on continuous, real-time perception and intrinsic motivation, bypassing the latency of traditional token-prediction.

**2.1. The Continuous Latent Bus (VL-JEPA)**
The system processes environmental states using a Joint Embedding Predictive Architecture. Foundational encoders (e.g., Retinal, Dorsal, Ventral) map visual inputs $X_V$ into continuous visual embeddings $S_V$. Concurrently, a forward model predicts future target embeddings $\hat{S}_Y$. These encoders operate at a fixed clock speed, acting as "always-on" publishers of both the current state of the world and the expected future.

**2.2. Homeokinetic Exploration**
To drive exploration without explicit reward functions, the system utilizes Homeokinesis. Rather than minimizing standard forward prediction error (which can lead to a "lazy robot" effect where the agent does nothing to maintain perfect predictability), the agent seeks to minimize the Time-Loop Error (TLE). TLE measures the reconstruction error of the past state from the predicted future state. Minimizing this error naturally drives the agent to destabilize static states while finding functional, predictable physical resonances in its environment.

### 3. The Epistemic Engine & The Mitosis Gatekeeper
As System 1 babbles and explores the continuous latent space, the system requires a mechanism to capture and modularize useful discoveries without suffering from node explosion. 

**3.1. Episodic Predictive Memory (EPM)**
EPMs act as specialized, limited-capacity experts (analogous to cortical columns) that subscribe to the VL-JEPA latent bus. Instead of retraining the foundational visual encoders, learning occurs within these lightweight relational nodes.

**3.2. Complexity Detection via High-Order Entropy**
To evaluate the structural value of a latent trajectory, the system employs an Assembly Theory proxy. Drawing on methodologies from *Computational Life*, we monitor the "high-order entropy" of the latent stream. This metric factors out information that comes from sampling independent and identically distributed variables. 

By vector-quantizing the latent space, we can compute the Shannon entropy $H(X)$ to measure vocabulary diversity, and approximate the normalized Kolmogorov complexity $K(X)$ using Lempel-Ziv-style text compressors. The resulting High-Order Entropy metric detects when the system transitions from random noise into highly structured, repeating behaviors.

**3.3. The Spawning Trigger**
The Global Workspace relies on a strict conditional matrix to manage neural resources:
1. **Resonance:** A sudden drop in the homeokinetic Time-Loop Error indicates the agent has found a predictable physical mode.
2. **Complexity:** A simultaneous spike in High-Order Entropy indicates the sequence is structured and non-random.
When both conditions are met, the system crystallizes the transient sensorimotor loop into a new, permanent EPM module.

### 4. The Deliberative Supervisor (System 2)
The society of EPM experts is managed by a routing hierarchy that facilitates both reactive control and deliberative planning.

**4.1. Lateral Voting**
EPMs constantly compare the JEPA encoder's broadcasted reality against their own internal predictions. The Lateral Voter monitors this Prediction Error (PE). EPMs with a low PE achieve "Predictive Resonance" and are granted access to the Global Workspace. EPMs experiencing massive PE (e.g., during a drastic environment shift) lose resonance and are safely archived, protecting their weights from catastrophic forgetting.

**4.2. Global Workspace (GW) & Top-Down Guidance**
The GW operates over extended time horizons, maintaining a stable abstract context vector. It facilitates System 2 planning by performing explicit rollout predictions in the latent space. The GW exerts top-down control by generating sustained bias adjustments to the lower-level homeokinetic controllers, channeling playful exploration into abstract, goal-directed behaviors.

### 5. Mechanism of Autopoiesis: Fractal Expansion
The proposed architecture natively scales to handle paradigm shifts, such as moving from a low-dimensional deterministic task (e.g., Pong) to a highly complex, hostile 3D environment (e.g., DOOM). 

Upon experiencing a massive context shift, existing EPMs experience a spike in Prediction Error and are archived by the Lateral Voter. The system reverts to System 1 Homeokinetic babbling. Once the agent discovers a new physical resonance (e.g., navigating a 3D corridor), the High-Order Entropy metric spikes. The Global Workspace crystallizes this new behavior into a localized EPM. As the library of EPMs grows, the Global Workspace can spawn localized Sub-Voters to manage specific environmental domains, achieving true fractal abstraction.

### 6. Conclusion
By decoupling foundational representation learning from specialized episodic memory, and by utilizing high-order entropy as a gatekeeper for neurogenesis, the Fractal JEPA Workspace presents a computationally tractable path to continuous learning. The integration of homeokinetic drive ensures the system remains intrinsically motivated to explore, while the publish-subscribe cognitive bus guarantees that knowledge can be infinitely expanded without destructive interference.
