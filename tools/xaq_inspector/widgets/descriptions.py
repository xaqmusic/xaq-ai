"""Educational per-module documentation shown in the inspector.

The xaq inspector doubles as a teaching surface: one of AMI-Ogma's stated
purposes is to explain *active inference* as a design pattern for robotics.
So every module widget gets a description panel keyed by the C++ module
``type_name`` (the same keys as ``WIDGET_REGISTRY`` in ``__init__.py``).

Each entry is a :class:`ModuleDoc`:

  * ``title``    — a friendly one-line name for the module.
  * ``summary``  — a LAYMAN, non-expert explanation (always visible).
  * ``formulas`` — the exact math used to derive the module's outputs,
    with every abbreviation expanded at least once (collapsed by default).

Both ``summary`` and ``formulas`` accept the small HTML subset that Qt's
``QLabel`` understands (``<b> <i> <br> <code> <sub> <sup> &nbsp;`` and named
entities). Formula fragments are wrapped in ``<code>`` (rendered monospace);
literal ``<`` / ``>`` inside a formula must be written ``&lt;`` / ``&gt;``.
Greek and math glyphs (α β γ λ μ ω π θ σ ε · − × √ Σ ≤ ≥ ≠ → ∈ ½) are written
as UTF-8 literals. Keep formulas faithful to the C++ source in
``cpp_core/src/ogma/modules/`` (a handful — SIGReg, JEPA, HDC — are
training-time or conceptual and are described, not read live).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ModuleDoc:
    title: str
    summary: str      # HTML — layman summary, shown by default
    formulas: str      # HTML — exact formulas + glossary, collapsed by default


# Keyed by C++ module type_name (see WIDGET_REGISTRY). Each value pairs a
# plain-language summary with the exact formulas the module uses.
DOCS: dict[str, ModuleDoc] = {

    # ---------------------------------------------------------------- substrate
    "EPM": ModuleDoc(
        title="EPM — Episodic Predictive Module (one “sense”)",
        summary=(
            "Each EPM is a single sense. It squeezes its raw input into a small "
            "code, matches that code to the closest memory in a bank of "
            "prototypes it grows over time (new prototypes sprout when something "
            "genuinely novel appears), and reports how <i>surprised</i> it was — "
            "the gap between what it expected and what it got. That surprise, the "
            "Time-Loop Error, is this module's learning signal and the currency "
            "the rest of the brain trades in."
        ),
        formulas=(
            "<b>How it works.</b> The encoder makes a latent vector <code>z</code>; "
            "a Growing Neural Gas (GNG) picks the nearest prototype and returns the "
            "distance to it. Two errors are combined into one surprise value.<br><br>"
            "<code>quant_error = ‖z − p<sub>winner</sub>‖</code> — distance to nearest prototype<br>"
            "<code>transition_surp = ‖p<sub>winner</sub>(t) − p<sub>winner</sub>(t−1)‖</code> — jump between winners<br>"
            "<code>TLE = α·quant_error + β·transition_surp</code><br>"
            "<code>ema_tle ← (1−λ)·ema_tle + λ·TLE</code><br>"
            "<code>novelty_threshold = max(floor, ema_tle·mult·scale)</code><br>"
            "<code>z ← z − ẑ</code> (optional: subtract a descending prediction before quantizing)<br><br>"
            "<b>SIGReg</b> (training only, Python): project the latent buffer onto P "
            "random unit directions and push each 1-D projection toward a unit "
            "Gaussian — <code>loss = mean(μ²) + mean((σ²−1)²)</code> — which stops the "
            "latent from collapsing.<br><br>"
            "<b>Terms.</b> EPM = Episodic Predictive Module; latent = compressed "
            "feature vector; GNG = Growing Neural Gas (prototype bank that grows "
            "nodes); TLE = Time-Loop Error (prediction-error surprise); EMA = "
            "Exponential Moving Average; SIGReg = Sketched Isotropic Gaussian "
            "Regularization; JEPA = Joint-Embedding Predictive Architecture (learn "
            "in latent space, never rebuild pixels)."
        ),
    ),

    "NeurochemState": ModuleDoc(
        title="NeurochemState — dopamine & serotonin (the brain's mood)",
        summary=(
            "Two chemical dials set the brain's mood: dopamine (reward / curiosity) "
            "and serotonin (comfort / patience). Each tick they drift back toward a "
            "resting level and get nudged by events — eating and rising smell raise "
            "dopamine; bumping walls, getting stuck, or going hungry lower "
            "serotonin. Crucially, downstream learners don't credit raw dopamine; "
            "they credit how much it beat its own recent average, so only "
            "<i>surprising</i> good news teaches."
        ),
        formulas=(
            "<code>DA ← DA<sub>base</sub> + (DA − DA<sub>base</sub>)·decay</code> — decay to baseline (5-HT analogous)<br>"
            "<code>Δ = TLE(t−1) − TLE(t); if Δ&gt;0: DA ← clamp01(DA + g·Δ)</code> — reward for predicting better<br>"
            "<code>DA / 5-HT ← clamp01( ± rate·count )</code> — per event (hit, miss, wall, whisker, hunger, scent-rise…)<br>"
            "<code>DA_baseline_ema ← (1−a)·DA_baseline_ema + a·DA</code> — slow reference<br>"
            "<code>reward_signal = DA − DA_baseline_ema</code> — the reward-prediction error broadcast to learners<br><br>"
            "<b>Terms.</b> DA = dopamine; 5-HT / HT = serotonin "
            "(5-hydroxytryptamine); RPE = reward-prediction error; TLE = Time-Loop "
            "Error; EMA = Exponential Moving Average; clamp01 = clip to [0, 1]. The "
            "baseline-subtraction is the classic Schultz dopamine model."
        ),
    ),

    "HomeostaticDrive": ModuleDoc(
        title="HomeostaticDrive — internal needs & urgency",
        summary=(
            "The body's “how am I doing?” gauge. Each internal variable (chiefly "
            "energy) has a target level (setpoint); the module reports the signed "
            "gap to that target and boils all the gaps down to one number — "
            "<i>urgency</i> — equal to the worst normalized deviation. Eating "
            "refills energy toward its setpoint. This is the interoception that "
            "makes the agent hungry, and hunger is what drives foraging."
        ),
        formulas=(
            "<code>err<sub>c</sub> = current<sub>c</sub> − setpoint<sub>c</sub></code> — signed gap per channel c<br>"
            "<code>urgency = clamp( max<sub>c</sub> |err<sub>c</sub>| / normalizer<sub>c</sub>, lo, hi )</code><br>"
            "<code>current ← min(setpoint, current + hits·replenish·(setpoint − current))</code> — refill on eat<br><br>"
            "<b>Terms.</b> setpoint = target value of an internal variable; urgency "
            "= worst normalized deficit (0 = content, 1 = critical); interoception "
            "= sensing the body's own state; proprio = proprioceptive (body-motion) "
            "input."
        ),
    ),

    "FaderController": ModuleDoc(
        title="FaderController — how much the “brain” drives vs. reflexes",
        summary=(
            "A single fader that decides how much authority the deliberative brain "
            "gets versus the fast reflexes. When the world is surprising or the "
            "plan is uncertain it hands control to reflexes (low α); when things "
            "are familiar and the policy is confident it lets the brain steer "
            "(high α). It also nudges α upward when bored, so a settled agent "
            "gradually starts trusting itself."
        ),
        formulas=(
            "<code>α ← (1−λ<sub>s</sub>)·α + λ<sub>s</sub>·α_target</code> — smooth toward the target<br><br>"
            "α_target has several modes:<br>"
            "<code>surprise: clamp( (1 − max(s, familiarity·coupling)) + boredom_gain·(1 − α_long_ema) )</code><br>"
            "<code>learned:  clamp( setpoint·(1 − s) )</code><br>"
            "<code>premotor: α_min + (α_max−α_min)·(1 − H/ln N)</code> — confident policy → brain<br>"
            "<code>chunk:    α_min + (α_max−α_min)·σ(k·(q − midpoint))</code><br><br>"
            "<b>Terms.</b> α = blend weight (0 = reflex-only, 1 = brain-only); s = "
            "surprise scalar; H = Shannon entropy of the policy; N = number of "
            "intents; σ = logistic sigmoid; EMA = Exponential Moving Average."
        ),
    ),

    "LateralVoter": ModuleDoc(
        title="LateralVoter — cross-modal consensus (the senses vote)",
        summary=(
            "The senses vote on one shared reality. Each modality's opinion is "
            "trusted in proportion to how precise and informative it is right now — "
            "a crisp, well-learned channel outvotes a confused one. The trusted "
            "opinions are averaged into a single fused code, and channels that fire "
            "together wire together in a Hebbian association matrix — the system's "
            "cross-modal memory."
        ),
        formulas=(
            "<code>precision<sub>i</sub> = 1 / (|quant_error<sub>i</sub>| + ε)</code><br>"
            "<code>info<sub>i</sub> = floor + (1−floor)·ci<sub>i</sub></code> — informativeness from baked-node count<br>"
            "<code>raw<sub>i</sub> = precision<sub>i</sub> · info<sub>i</sub><sup>gain</sup></code> &nbsp;(legacy: <code>1/(|TLE<sub>i</sub>|+ε)</code>)<br>"
            "<code>trust<sub>i</sub> = raw<sub>i</sub> / Σ<sub>j</sub> raw<sub>j</sub></code> — normalize<br>"
            "<code>T = max(0.01, T₀·(1 + 0.5·DA)); trust<sub>i</sub> ← trust<sub>i</sub><sup>1/T</sup></code> — dopamine sharpens the vote<br>"
            "<code>fused = Σ<sub>i</sub> trust<sub>i</sub>·latent<sub>i</sub></code>, &nbsp;<code>fused_TLE = Σ<sub>i</sub> trust<sub>i</sub>·TLE<sub>i</sub></code><br>"
            "<code>A ← decay·A; &nbsp; A[win<sub>i</sub>][win<sub>j</sub>] += trust<sub>i</sub>·trust<sub>j</sub></code> (i≠j) — Hebbian update<br><br>"
            "<b>Terms.</b> trust = normalized reliability weight; precision = "
            "inverse error; ci = crystallized informativeness; DA = dopamine; TLE = "
            "Time-Loop Error; Hebbian = “fire together, wire together.” HDC "
            "(Hyperdimensional Computing) is the design vision; this path is plain "
            "trust-weighted linear fusion."
        ),
    ),

    "Premotor": ModuleDoc(
        title="Premotor — graded policy actor (perception → action)",
        summary=(
            "A one-layer policy that turns the fused perception into a probability "
            "distribution over action “intents,” then blends those intents into a "
            "motor push. It learns online — intents that preceded reward get their "
            "weights strengthened — and dopamine sets a temperature that makes it "
            "commit more or less decisively. It can also imitate the reflexes "
            "(behavioral cloning) to bootstrap."
        ),
        formulas=(
            "<code>scores = (W·z + b)·gain</code><br>"
            "<code>T = t_base·(1 + g_path·familiarity) / (1 + DA·g_DA)</code>, floored at 0.05 — temperature<br>"
            "<code>p = softmax(scores / T)</code><br>"
            "<code>accel = Σ<sub>i</sub> p<sub>i</sub>·a<sub>i</sub></code>, &nbsp;<code>H = −Σ<sub>i</sub> p<sub>i</sub>·ln p<sub>i</sub></code><br>"
            "<code>Hebbian:   ΔW<sub>i</sub> = lr·r·p<sub>i</sub>·zᵀ</code><br>"
            "<code>REINFORCE: ΔW<sub>i</sub> = lr·advantage·(𝟙{i=chosen} − p<sub>i</sub>)·zᵀ</code><br>"
            "<code>BC:        ΔW = lr_bc·gate·zᵀ</code> — imitate the reflex demonstration<br><br>"
            "<b>Terms.</b> W = policy weights; z = latent input; b = bias; T = "
            "softmax temperature; H = entropy; N = #intents; DA = dopamine; lr = "
            "learning rate; BC = behavioral cloning; REINFORCE = score-function "
            "policy gradient; advantage = return − value baseline V(s); CPG = "
            "Central Pattern Generator (phase context)."
        ),
    ),

    "SequenceGNG": ModuleDoc(
        title="SequenceGNG — temporal chunker (learns recurring motifs)",
        summary=(
            "Watches the stream of “which node won” over a sliding window and "
            "learns to name recurring little sequences — motifs — the way you hear "
            "a repeated drum pattern. It grows a new motif whenever the window "
            "doesn't match anything it knows, reports how well the current window "
            "fits a known motif, and predicts which motif tends to follow."
        ),
        formulas=(
            "<code>encode: window of W winner-ids → hash each → concatenate</code><br>"
            "<code>qe = ‖encoded − nearest prototype‖</code><br>"
            "<code>match_conf = clamp(1 − qe / (5·min_ins_error), 0, 1)</code><br>"
            "<code>predicted_next = argmax<sub>next</sub> successor_count[motif][next]</code><br><br>"
            "Underlying GNG (shared engine):<br>"
            "<code>node.error += d₁²; &nbsp; ema_error ← 0.9·ema_error + 0.1·d₁²</code><br>"
            "<code>stability = max(visits/bake_thr, min(1, 0.02·health))</code><br>"
            "<code>Δp = ε·(1 − 0.9·stability)·(x − p)</code> — move winner (ε_b) & neighbours (ε_n)<br>"
            "<code>grow: q = argmax error; insert if q.ema_error ≥ min_ins_error; new p = ½(p_q + p_f)</code><br><br>"
            "<b>Terms.</b> GNG = Growing Neural Gas (Fritzke); qe = quantization "
            "error; motif = a crystallized recurring sub-sequence; baking = "
            "freezing a well-visited node; ε_b / ε_n = winner / neighbour learn "
            "rates; EMA = Exponential Moving Average."
        ),
    ),

    "CPGOscillator": ModuleDoc(
        title="CPGOscillator — spinal rhythm generator",
        summary=(
            "A built-in rhythm generator: it produces the leg's basic gait wave and "
            "standing bias — the “spinal cord” beat that moves the body before the "
            "brain has learned anything. As the brain proves it can predict and "
            "reward its own motion, a competence gate fades the oscillator out so "
            "the learned controller takes over."
        ),
        formulas=(
            "<code>φ ← φ + 2π/period</code> (wrapped) — fixed-frequency phase clock (not a Hopf oscillator)<br>"
            "<code>A_eff = floor + (amplitude − floor)·sin(π·gate)</code> — walking amplitude bell<br>"
            "<code>standing = standing_amp·(1 − gate)²</code><br>"
            "<code>bias = A_eff·sin(φ + leg_off + joint_off)·sign + standing·sign</code>; &nbsp;<code>out = clamp(brain + bias)</code><br>"
            "<code>gate = ratchet( √(g_tle·g_reward) )</code>, &nbsp;<code>g_tle = clamp(1 − ema_TLE/max_TLE)</code>, &nbsp;<code>g_reward = clamp(ema_reward/scale)</code><br><br>"
            "<b>Terms.</b> CPG = Central Pattern Generator; φ = phase; gate = "
            "competence fade (0 = CPG drives, 1 = brain drives); TLE = Time-Loop "
            "Error; EMA = Exponential Moving Average; ratchet = climbs fast, decays "
            "slowly."
        ),
    ),

    "CruseCoordinator": ModuleDoc(
        title="CruseCoordinator — inter-leg gait coordination (Walknet)",
        summary=(
            "Coordinates the legs so they don't all lift at once, using Holk "
            "Cruse's insect “Walknet” rules. It reads each foot's height to tell "
            "stance from swing, then nudges each leg's rhythm: hold your stance if "
            "a neighbour is swinging, release to swing when the leg ahead just "
            "landed, and firm up when the opposite leg is in the air. The result is "
            "an organized gait instead of a scramble."
        ),
        formulas=(
            "<code>span = high − low; plant when foot_y ≤ low + low_frac·span; lift when ≥ low + high_frac·span</code> (hysteresis)<br>"
            "<code>Rule 1</code> neighbour swinging → +1 to stance (×boost if illegally co-swinging)<br>"
            "<code>Rule 2</code> anterior just landed within window → −1 to swing<br>"
            "<code>Rule 3</code> contralateral swinging → += rule3_weight to stance<br>"
            "<code>Rule 6</code> swing overran → +1 to stance; stance overran → −1 to swing<br>"
            "<code>bias<sub>i</sub> = magnitude·Σ(rule factors)·stance_sign·(accel<sub>i</sub> − mean)/max_abs</code><br>"
            "<code>φ ← φ + ω; φ += K·(target − φ) at touchdown/liftoff; ω = 2π/(swing_ema + stance_ema)</code><br><br>"
            "<b>Terms.</b> Cruse / Walknet = insect inter-leg coordination rules; "
            "anterior = leg ahead in the chain; contralateral = opposite-side leg; "
            "ω = angular velocity; Kuramoto = phase entrainment; EMA = Exponential "
            "Moving Average."
        ),
    ),

    # ---------------------------------------------------- Cell perception / nav
    "ScentCompass": ModuleDoc(
        title="ScentCompass — the chemical-gradient sense",
        summary=(
            "Turns the ring of “nostrils” around the body into one arrow pointing "
            "up the food-smell gradient — which way food is. The arrow's length is "
            "how confident that direction is (the gradient strength); a separate "
            "proximity number says how much food is nearby regardless of "
            "direction. This is the perception half of the chemotaxis loop."
        ),
        formulas=(
            "<code>θ<sub>i</sub> = 2π·i/N</code> — angle of nostril i on the body ring<br>"
            "<code>gx = Σ<sub>i</sub> cos(θ<sub>i</sub>)·s<sub>i</sub></code>, &nbsp;<code>gz = Σ<sub>i</sub> sin(θ<sub>i</sub>)·s<sub>i</sub></code> — vector-sum the concentrations<br>"
            "<code>cx = gx</code> (right), &nbsp;<code>cy = −gz</code> (forward)<br>"
            "<code>mag = √(cx² + cy²)</code> — gradient strength = confidence<br>"
            "<code>if normalize & mag&gt;min: [cx,cy] ← [cx,cy]/mag</code><br>"
            "<code>prox = proximity_gain·(1/N)·Σ<sub>i</sub> s<sub>i</sub></code> — mean concentration<br><br>"
            "<b>Terms.</b> cx / cy = bearing components (+right / +forward); mag = "
            "gradient magnitude (confidence); prox = proximity (mean scalar "
            "concentration); N = nostril_count; s<sub>i</sub> = nostril i "
            "concentration; chemotaxis = movement along a chemical gradient."
        ),
    ),

    "MotorEPM": ModuleDoc(
        title="MotorEPM — homeokinetic sensorimotor self-model",
        summary=(
            "The one always-on motor loop. Per limb (for the Cell, two flagella) it "
            "learns a little forward model — “if I send this command, this is the "
            "sensor reading I'll get next” — and then drives itself to stay "
            "maximally responsive to its own actions. That homeokinetic rule "
            "produces self-sustaining swimming with no reward at all; the cognitive "
            "brain just adds steering and thrust on top."
        ),
        formulas=(
            "<code>x̂(t+1) = A·y(t) + b</code> — forward self-model; &nbsp;<code>ξ = x − x̂</code> — motor TLE<br>"
            "<code>A ← A + η_M·ξ·yᵀ, &nbsp; b ← b + η_M·ξ</code> — learn the model<br>"
            "<code>y = tanh(C·x + h)</code> — controller output (× gains)<br>"
            "<code>G = diag(1 − tanh²(z)), &nbsp; L = A·G·C</code> — loop Jacobian<br>"
            "<code>P = (LLᵀ + εI)⁻¹, &nbsp; q = P·ξ</code><br>"
            "<code>ΔC = 2η_K·(AG)ᵀq·(qᵀL); &nbsp; h ← h + η_h·G·(Aᵀq)</code> — homeokinetic descent<br>"
            "<code>cell: (y₀,y₁) += (+1,∓1)·gain·cmd·(1 − boredom)</code> — steer/thrust injection<br><br>"
            "<b>Terms.</b> EPM = Episodic Predictive Module; TLE = Time-Loop Error "
            "(self-model surprise); HK / homeokinesis = Der–Martius stability-"
            "seeking control; A / C = forward-model / controller matrices; L = loop "
            "Jacobian; η = learning rate; ε = regularizer."
        ),
    ),

    "ActionDecoder": ModuleDoc(
        title="ActionDecoder — the coxswain (deliberate actor)",
        summary=(
            "The deliberate decision-maker. It works out which discrete “state” it "
            "believes it's in, then scores possible motor actions by how much each "
            "would bring about its preferred observation — more food-smell, more "
            "food-in-view — using a learned model of what actions lead to. It can "
            "score by expected free energy, by learned value, or by rolling a short "
            "plan forward, and outputs a turn plus a thrust."
        ),
        formulas=(
            "<code>a_b = accel_min + b·(accel_max − accel_min)/(bins−1)</code> — bin → accel (P-control)<br>"
            "<code>EFE: S(b) = gain·Σ<sub>s'</sub> P(s'|s,b)·V(s') + epi_scale·(−Σ P·ln P)</code><br>"
            "<code>value-RL: S(b) = gain·Q(s,p,b) + epi_gain/(1 + visits)</code><br>"
            "<code>plan: V_H(s) = max<sub>a</sub> [ node_value(next) + γ·V_{H−1}(next) ]</code><br>"
            "<code>node_value = w_scent·scent + w_green·green</code> — the preferred-observation prior<br>"
            "<code>ε = clamp(1 − serotonin, 0.05, 0.30)</code> — exploration rate<br>"
            "<code>turn_bin = idx / thrust_bins, &nbsp; thrust_bin = idx mod thrust_bins</code><br><br>"
            "<b>Terms.</b> EFE = Expected Free Energy (pragmatic value + epistemic "
            "info-gain); TD / Q = temporal-difference state-action value; TLE / "
            "action_tle = forward-model surprise; P-control = proportional bin→accel "
            "map; γ = discount; ε-greedy = explore with probability ε; “C prior” = "
            "active-inference preference over observations."
        ),
    ),

    "MotorBus": ModuleDoc(
        title="MotorBus — the motor mixing console",
        summary=(
            "A mixing desk for the body's muscles. Every channel that wants to move "
            "the agent — reflexes, klino, planner, vision — submits a left/right or "
            "steer/thrust vote; the bus applies each channel's fader, sums them, "
            "and compresses the result into a final drive. A turn-priority mix "
            "gives turning the road first (so the body pivots to face rather than "
            "orbiting), and each channel's “authority share” shows who's really "
            "steering."
        ),
        formulas=(
            "<code>steer_thrust → L = thrust + steer, R = thrust − steer; &nbsp; lr → L=a, R=b</code> (÷scale)<br>"
            "<code>eff_gain = base·max(0, 1 + boredom_resp·mod)·arb_gain</code><br>"
            "<code>sum_L = Σ eff_gain·cl, &nbsp; sum_R = Σ eff_gain·cr</code><br>"
            "<code>common = ½(sum_L+sum_R); &nbsp; diff = clamp(½(sum_L−sum_R), ±1)</code><br>"
            "<code>head = max(0, 1 − turn_brake·|diff|); &nbsp; common ← clamp(common, ±head)</code><br>"
            "<code>out = common ± diff</code> &nbsp;(legacy compressor: <code>out = limit·tanh(sum)</code>)<br>"
            "<code>authority ← EMA( eff_gain / Σ eff_gain )</code><br><br>"
            "<b>Terms.</b> fader / gain = per-channel volume; turn_brake = "
            "forward-yields-to-turn damping (the pivot-vs-orbit knob); GR = gain "
            "reduction (compression); arb_gain = the L2 arbiter's winner-take-all "
            "gate; EMA = Exponential Moving Average."
        ),
    ),

    "HeadingController": ModuleDoc(
        title="HeadingController — learned heading follower",
        summary=(
            "Takes a desired direction (say, the scent bearing) and actually turns "
            "the body toward it — but instead of a hand-tuned steering gain it "
            "learns how sharply <i>this</i> body rotates per unit of steering "
            "command and inverts that, so the same controller transfers across "
            "bodies and friction. It also learns when to thrust: charge when facing "
            "the target, brake or turn-in-place when it's off to the side."
        ),
        formulas=(
            "<code>β = atan2(cx, cy)/π ∈ [−1,1]</code> — heading error (gated on |compass| &gt; min_signal)<br>"
            "<code>k_body ← k_body + lr·(|ω|/|steer_prev| − k_body)</code> — learn body turn-response<br>"
            "<code>g = clamp(turn_fraction / k_body, g_min, g_max)</code>; &nbsp;<code>steer = clamp(g·β, ±max)</code><br>"
            "<code>thrust = max_thrust·clamp( (cos(πβ) − cos(align)) / (1 − cos(align)) )</code> — advance gate<br>"
            "<code>reward = max(0, v_fwd)·cos(β) − effort·|v_fwd|</code>; &nbsp;<code>UCB = c·√(ln(N+1)/(n_a+1))</code><br><br>"
            "<b>Terms.</b> β = normalized heading error (0 = dead-ahead); k_body = "
            "learned turn-response (rad/tick per steer); ω = yaw rate; P-control = "
            "proportional control; UCB = Upper Confidence Bound (exploration bonus); "
            "orthokinesis = slowing near the source; EMA = Exponential Moving Average."
        ),
    ),

    "PlaceGraphPlanner": ModuleDoc(
        title="PlaceGraphPlanner — cognitive map & route planner",
        summary=(
            "The original mental map. As the agent forages it crystallizes discrete "
            "“places,” learns the compass heading between adjacent places, and tags "
            "where it ate. It value-iterates a “which way to food” gradient over "
            "that graph (with curiosity and boredom mixed in) and steers along the "
            "learned edges toward a remembered cache — falling back to raw "
            "scent-following when it has no route, and letting vision take over for "
            "the final approach."
        ),
        formulas=(
            "<code>local = hunger·food[n] + tle_gain·TLE[n] (+coverage in patrol)</code><br>"
            "<code>V[n] = local·(1 − hab[n]) + γ·max<sub>m</sub> V[m]</code> — value iteration<br>"
            "<code>on hit: food[cur] += reward</code> (accumulates); &nbsp;<code>decay: food ×= 0.9995</code><br>"
            "<code>disconfirm: food[cur] ×= (1 − clamp(disc·hunger·hab))</code><br>"
            "<code>steered(n) = V[n] − escape_gain·hunger·stale·hab[n]</code> — confinement escape<br>"
            "<code>precision = 1 − H/ln(nf), &nbsp; H = −Σ p<sub>i</sub>·ln p<sub>i</sub>, &nbsp; p<sub>i</sub> = food<sub>i</sub>/Σfood</code><br>"
            "<code>novelty = clamp(frontier_TLE / tle_peak)</code>; &nbsp;<code>if |vis|&gt;floor: home to [vis_x, vis_y]</code><br><br>"
            "<b>Terms.</b> V = route value; hab = habituation (boredom); γ = "
            "discount; TLE = Time-Loop Error (novelty); H = Shannon entropy; nf = "
            "number of food caches. plan_value / plan_precision / plan_novelty feed "
            "the L2 EFE arbiter."
        ),
    ),

    "PlaceNav": ModuleDoc(
        title="PlaceNav — honest region navigator (the planner, reframed)",
        summary=(
            "The cleaned-up successor to the planner. It uses the same map but "
            "treats food memory honestly: a place gets a bounded food tag set to 1 "
            "when you actually eat there (never an ever-growing super-magnet), and "
            "that tag is disconfirmed if you arrive hungry and find nothing. It "
            "routes toward a remembered food region, marks edges that turn out to "
            "be walls as costly, and only advertises a food route to the arbiter "
            "when that route is real and executable."
        ),
        formulas=(
            "<code>local = hunger·food_tag[n] (+coverage if no tag anywhere)</code><br>"
            "<code>V[n] = local·(1 − hab[n]) + γ·max<sub>m</sub> (V[m] − block[n][m])</code> — wall-aware value iteration<br>"
            "<code>on eat: food_tag[cur] := 1</code> (SET, capped); &nbsp;<code>fade: ×= 0.999</code><br>"
            "<code>honest forget: hungry(&gt;0.3) departure without eating → food_tag ×= 0.1</code><br>"
            "<code>cede a stalled hop when route_stall &gt; stall_factor·max(hop_ema, 8), then block += 0.5</code><br>"
            "<code>plan_value = planning ? clamp(V[next]) : 0</code>; &nbsp;<code>plan_novelty = clamp(1 − hab[next])</code> when exploring<br><br>"
            "<b>Terms.</b> food_tag = bounded honest food memory (vs the planner's "
            "accumulating cache); block = learned per-edge wall cost; hab = "
            "habituation; γ = discount; hop_ema = EMA of successful hop durations; "
            "ceded = route ruled unreachable."
        ),
    ),

    "PlayLoop": ModuleDoc(
        title="PlayLoop — epistemic play (grow the map)",
        summary=(
            "Curiosity as a policy. Where the planner descends a value field toward "
            "known food, PlayLoop <i>ascends novelty</i> toward the least-modelled "
            "frontier — and when the mapped graph stops growing it runs-and-tumbles "
            "beyond it into unexplored ground. It is “planner minus exploit”: both "
            "are thin overlays on the same place-EPM, but this one is driven by how "
            "badly the world-model still fails (novelty), not by food."
        ),
        formulas=(
            "<code>V_play[n] = novelty[n]·(1 − hab[n]) + γ·max<sub>m</sub> V_play[m]</code> — novelty value iteration<br>"
            "<code>novelty[n] ← novelty[n] + α·(TLE_place − novelty[n])</code> — place-EPM surprise<br>"
            "<code>hab_cur ← hab_cur + rise·(1 − hab_cur); &nbsp; all hab ×= (1 − decay)</code><br>"
            "<code>climb ⟺ a strictly-more-novel neighbour exists AND not forced_wander</code><br>"
            "<code>forced_wander ⟺ ticks-since-map-grew ≥ wander_stall_ticks</code><br>"
            "<code>play_value = max(climb_v, wander_v)</code>, &nbsp;<code>climb_v = clamp(V_play[next]/ref)</code>, &nbsp;<code>wander_v = clamp(1 − hab_cur)</code><br>"
            "<code>frontier: steer outward from the habituation-weighted visited centroid, blended by clamp(frontier_bias·max_hab)</code><br><br>"
            "<b>Terms.</b> TLE = Time-Loop Error (model degradation = novelty); hab "
            "= habituation (boredom); γ = discount; frontier = highest-novelty "
            "(least-modelled) node. play_value feeds the L2 EFE arbiter as the "
            "epistemic (explore) term."
        ),
    ),

    "Klinotaxis": ModuleDoc(
        title="Klinotaxis — weaving gradient follower",
        summary=(
            "A gradient-climber that can't read the gradient's direction directly, "
            "so it does what a maggot does: weave side to side and correlate “is "
            "the smell rising?” against its own turning. When rising smell lines up "
            "with turning one way, that way is uphill — it steers the weave's "
            "centreline toward it. The dial also shows a known failure: if the "
            "commanded target lands behind the nose, the body can pin (turn without "
            "advancing)."
        ),
        formulas=(
            "<code>phase ← wrap(phase + 2π/period)</code>; &nbsp;<code>heading = base + weave_eff·sin(phase)</code><br>"
            "<code>g = cov / √(var_ddt·var_om)</code> — lock-in (Pearson r of scent-change ddt vs yaw ω)<br>"
            "<code>base_heading ← wrap(base_heading + mode·steer_gain·g·align)</code><br>"
            "<code>align = clamp(v_forward/|v|)</code>; &nbsp;<code>cap = clamp((s − s_min)/(s_peak − s_min))</code><br>"
            "<code>weave_eff = weave_amp·(1 − cap)</code>; &nbsp;<code>[vx,vy] = [sin δ, cos δ], δ = wrap(base + weave − heading)</code><br><br>"
            "<b>Terms.</b> ddt = scent change per tick; ω = yaw rate; g = lock-in "
            "correlation (−1..1, the recovered gradient sign+strength); cap = "
            "self-calibrated proximity; align = forward-motion gate; mode = +1 seek "
            "/ −1 flee; klinotaxis = weave-and-correlate gradient ascent."
        ),
    ),

    "RunTumbleNav": ModuleDoc(
        title="RunTumbleNav — E. coli run-and-tumble chemotaxis",
        summary=(
            "Bacterial foraging. It runs in a committed direction and, like "
            "E. coli, only re-orients (tumbles) when the smell stops climbing above "
            "a slowly-adapting memory of recent smell (methylation). Rising smell → "
            "keep going; flat or falling → randomly pick a new heading. An "
            "orthokinesis crank makes it tumble more often when it's already near "
            "food, and an optional directional belief biases new headings toward "
            "the way that has been paying off."
        ),
        formulas=(
            "<code>error = s − baseline; &nbsp; baseline ← baseline + α·(s − baseline); &nbsp; error_n = error/error_scale</code><br>"
            "<code>p_tumble = clamp(base·(1 + level_gain·cap) − gain·error_n, min, max)</code><br>"
            "<code>tumble if U(0,1) &lt; p_tumble</code> (or forced when stuck: |v_fwd| &lt; thresh for stuck_ticks)<br>"
            "<code>cap = clamp(s / eat_scent)</code> — eat-calibrated proximity<br>"
            "<code>KF6: outcome_n = clamp(run_delta/error_scale, ±4); &nbsp; g = EMA(outcome_n·unit(run_dir))</code><br>"
            "<code>R = |g|/EMA(|outcome_n|); &nbsp; μ = atan2(g)</code> — belief precision / mean heading<br>"
            "<code>new heading: centre = circ_lerp(heading → μ, R); &nbsp; u ~ U(±range)·(1 − ½R)</code><br><br>"
            "<b>Terms.</b> methylation baseline = adaptive smell memory (the "
            "prediction); cap = proximity (0 far → 1 at source); orthokinesis = "
            "tumble-rate rises with proximity; KF = Keyframe-feature ladder (opt-in "
            "mechanisms, not Kalman filter); R / μ = directional-belief precision / "
            "mean heading; EFE = the L2 arbiter it feeds."
        ),
    ),

    "RunTumbleNavV2": ModuleDoc(
        title="RunTumbleNavV2 — clean-room run-and-tumble (KF ladder)",
        summary=(
            "A rebuilt run-and-tumble with the doctrine “ladder” of mechanisms on "
            "by default and validated by a single ablation switch — and, "
            "deliberately, no orthokinesis crank. Its new trick is a learned noise "
            "floor: while sitting still it measures its own sensor jitter, so the "
            "tumble threshold comes from the body's own dynamics rather than a "
            "hand-set constant."
            "<br><br><b>Loop, or baseline?</b> Inside the full agent this is the "
            "<i>klino</i> loop — one of four Markov-blanket loops the EFE arbiter "
            "selects among. Run <i>alone</i> (the chemotaxis-baseline config: no "
            "arbiter, no map, no play, no vision, no cross-loop model), the very same "
            "controller is the report's <b>reactive external baseline</b> — the "
            "hand-designed <i>E. coli</i> yardstick the composition must beat (§6), "
            "not a component of the AI system. What makes it a baseline is the "
            "<i>wiring</i> (one reflex steering the body directly), not the module."
        ),
        formulas=(
            "<code>vel_scale = max(|v_fwd|, vel_scale·(1 − decay))</code>; &nbsp;<code>stationary if |v_fwd| &lt; 0.15·vel_scale</code><br>"
            "<code>when stationary: noise_floor ← + α·(|error| − noise_floor)</code><br>"
            "<code>eff_scale = max(error_scale, noise_floor); &nbsp; error_n = error/eff_scale</code><br>"
            "<code>p_tumble = clamp(base − gain·error_n, min, max)</code> — no orthokinesis term<br>"
            "<code>KF2 stuck: blocked if |v_fwd| &lt; stuck_frac·vel_scale for stuck_ticks</code><br>"
            "<code>KF6 directional belief identical to V1; ablation ∈ {None, Shuffle, Kinesis, WrongSign, ShuffleDir}</code><br><br>"
            "<b>Terms.</b> KF = Keyframe-feature ladder (KF4 = the new noise floor + "
            "speed scale); nfloor = learned stationary noise floor; vscale = learned "
            "forward-speed scale; eff_scale = max(error_scale, noise floor); taxis "
            "vs kinesis = directed vs undirected movement."
        ),
    ),

    "CylinderBuilder": ModuleDoc(
        title="CylinderBuilder — place-code panorama",
        summary=(
            "As the agent spins, this bins the first-person camera colour around "
            "the heading circle, storing the average colour seen while facing each "
            "direction. The finished “panorama” is a colour signature of where the "
            "agent is — view-invariant because it's indexed by absolute heading, "
            "not by whatever is momentarily ahead. That signature feeds a "
            "place-EPM, whose surprise becomes the explorer's novelty drive."
        ),
        formulas=(
            "<code>bin = floor( wrap(heading, 0..2π)/2π · n_bins )</code><br>"
            "<code>each tick: frame mean (R,G,B) over all pixels → add to current bin's sums, count++</code><br>"
            "<code>finalize: panorama[bin] = (Σ frame-means / count)/255</code> per channel; unfilled bins carry over<br>"
            "<code>output length = n_bins·3</code><br><br>"
            "<b>Terms.</b> FPV = First-Person View; panorama / place-code = "
            "heading-indexed colour signature; EPM = Episodic Predictive Module; "
            "view-invariant = keyed by absolute heading; the downstream place-EPM's "
            "TLE is the novelty the explorer chases."
        ),
    ),

    "EFEArbiter": ModuleDoc(
        title="EFEArbiter — active-inference policy selector (L2)",
        summary=(
            "The referee that decides which navigation loop drives the body each "
            "moment, by scoring each as an Expected Free Energy: a pragmatic term "
            "(how likely this loop is to reach food, weighted by hunger) plus an "
            "epistemic term (how much uncertainty it would resolve, weighted by "
            "not-hunger). Hungry → exploit the loop most likely to feed you; sated "
            "→ explore the one that would teach you most. The winner takes the whole "
            "motor bus (gain 1), losers get 0, with an adaptive margin so channels "
            "don't chatter."
        ),
        formulas=(
            "<code>raw_klino = hunger·scent; &nbsp; raw_planner = plan_value</code><br>"
            "<code>g_prag_klino = hunger·cap_klino; &nbsp; g_prag_planner = hunger·clamp(plan_value)</code><br>"
            "<code>g_prag_vision = vision_w·hunger·vision_value</code><br>"
            "<code>gate = clamp(1 − max(g_prag_klino, g_prag_planner, g_prag_vision))</code> — reach-gated precision (R1)<br>"
            "<code>g_epist_klino = gate·z_spike_norm; &nbsp; g_epist_planner = gate·plan_novelty; &nbsp; g_epist_play = play_w·gate·play_value</code><br>"
            "<code>G<sub>x</sub> = g_prag<sub>x</sub> + g_epist<sub>x</sub></code> — total score per policy x<br>"
            "<code>keep incumbent unless v_challenger − v_incumbent &gt; margin; &nbsp; margin = k·std(gap)</code><br>"
            "<code>winner gain = 1.0, all others = 0.0</code><br><br>"
            "<b>Terms.</b> EFE = Expected Free Energy (here scored so higher value "
            "wins); L2 = second-level arbiter; g_prag / g_epist = pragmatic "
            "(exploit) / epistemic (explore) terms; reach = P(reach food | policy); "
            "z-spike = standardized scent surprise; WTA = winner-take-all; hunger = "
            "the preference precision that trades off exploit vs explore."
        ),
    ),

    "VisualBearing": ModuleDoc(
        title="VisualBearing — the visual food-bearing sense",
        summary=(
            "The eyes' version of the scent compass. It scans the first-person "
            "frame for food-coloured pixels (a green scaffold, or a learned food "
            "colour), finds their average screen column, and turns that into an "
            "arrow pointing toward the food — plus a green-fraction number for how "
            "much food fills the view (a looming / proximity cue). When no food is "
            "in view it reads zero: the “searching…” state."
        ),
        formulas=(
            "<code>food pixel: R &lt; r_max AND B &lt; b_max AND G &gt; g_min</code> &nbsp;(or <code>‖RGB − proto‖² &lt; dist²</code>)<br>"
            "<code>u = −1 + 2·(i+0.5)/width</code>; &nbsp;<code>u_c = (Σ u over food px)/green_count</code><br>"
            "<code>green_frac = green_count/(w·h)</code>; proceed only if <code>green_frac &gt; min_conf</code><br>"
            "<code>offset = u_c·tan(fov/2)</code><br>"
            "<code>[vx,vy] = [offset, 1]/√(offset²+1) = [sin θ, cos θ]</code>; &nbsp;<code>mag = √(vx²+vy²)</code><br>"
            "<code>lesioned window → output [0,0]</code><br><br>"
            "<b>Terms.</b> FOV = camera Field Of View; vx / vy = bearing (+right / "
            "+forward); u = normalized screen column ∈ [−1,1]; green_frac = fraction "
            "of the view that is food (graded confidence); RGB = red/green/blue; "
            "lesioned = vision knocked out (an ablation control)."
        ),
    ),

    "VisualHomingNav": ModuleDoc(
        title="VisualHomingNav — close on a SEEN source (loop #4)",
        summary=(
            "The sight-driven forager. It takes the visual food-bearing, checks the "
            "food isn't occluded and that its vision food-memory is trustworthy, "
            "and steers toward it — advertising a distance-independent “I can see "
            "food this way” value to the arbiter. Optionally it remembers the "
            "food's world-direction so it keeps homing for a moment while the food "
            "is briefly hidden (object permanence), though that is off by default."
        ),
        formulas=(
            "<code>have_food ⟺ green_frac &gt; min_conf</code> — occlusion gate<br>"
            "<code>cap_vision = clamp(green_frac / eat_green)</code> — eat-calibrated reach<br>"
            "<code>informativeness = clamp(node_count / node_ref)</code><br>"
            "<code>value = have_food ? informativeness : 0</code> — detection·direction, NOT distance<br>"
            "<code>persistence: world_bearing = wrap(atan2(vx,vy) + heading)</code><br>"
            "<code>while occluded: tgt_conf ×= (1 − decay); ego = wrap(world_bearing − heading); value = tgt_conf</code><br><br>"
            "<b>Terms.</b> EPM = Episodic Predictive Module (the vision-food "
            "memory); informativeness = food-structure trust; cap_vision = "
            "eat-calibrated reach (telemetry only); allocentric / egocentric = "
            "world-frame / body-frame. Its value feeds the arbiter's pragmatic "
            "vision term hunger·vision_weight·value."
        ),
    ),

    # ------------------------------------------------------- reflexes / detectors
    "CellReflex": ModuleDoc(
        title="CellReflex — bilateral swim + stuck-escape reflex",
        summary=(
            "The Cell's built-in swim. It pushes both flagella forward with a "
            "little noise (so it wanders), adds a scent-gated turn pulse to twist "
            "free when it's stuck, and fires “miss” and “wall-stuck” events. When "
            "smell is climbing it suppresses the aversive turning — don't flee the "
            "thing you're trying to reach."
        ),
        formulas=(
            "<code>EMA ← (1−α)·EMA + α·s</code> — short & long scent averages<br>"
            "<code>scent_factor = clamp((EMA_short − EMA_long)/EMA_long, 0, cap)</code><br>"
            "<code>deficit = clamp(1 − mean_speed/ref, 0, 1)</code><br>"
            "<code>t = tanh(|diff_sum|)·(1 − scent_factor); &nbsp; magnitude = |rand|·(1 − t) + t; &nbsp; pulse = clamp(dir·magnitude)</code><br>"
            "<code>steer = clamp(diff_max·gain·(1 − 2·scent_factor) + deficit·pulse, ±1)</code><br>"
            "<code>al = clamp(thrust + noise + steer_amp·steer); &nbsp; ar = clamp(thrust + noise − steer_amp·steer)</code><br><br>"
            "<b>Terms.</b> EMA = Exponential Moving Average; al / ar = accel left / "
            "right; deficit = fraction of reference speed missing; diff_max / "
            "diff_sum = whisker left−right; scent_factor = suppress aversion when "
            "smell is rising."
        ),
    ),

    "WhiskerSteerReflex": ModuleDoc(
        title="WhiskerSteerReflex — contact turn-away",
        summary=(
            "On a whisker touch it kicks a held turn away from the contact — a "
            "single-side push if one side is blocked, or a common-mode reverse "
            "“back out” if both sides are wedged — then falls silent for a "
            "refractory period so the brain has to learn the manoeuvre itself."
        ),
        formulas=(
            "<code>per side: sum = Σ(v·w) over whiskers with v &gt; threshold; activate if max(left,right) &gt; threshold</code><br>"
            "<code>single-side: al = clamp(gain·left_sum), ar = clamp(gain·right_sum)</code> → curve away<br>"
            "<code>wedge (both): al = min(−gain·left_sum, −reverse), ar = min(−gain·right_sum, −reverse)</code><br>"
            "<code>single-channel body: accel = clamp(al − ar)</code><br><br>"
            "<b>Terms.</b> al / ar = accel left / right; max_w = max contact; w = "
            "per-whisker position weight; refractory = enforced silence after a "
            "kick; wedge / head-on = both sides in contact. (A documented scent "
            "gate is cached but not applied to al/ar in the current code.)"
        ),
    ),

    "WhiskerAversionReflex": ModuleDoc(
        title="WhiskerAversionReflex — contact event detector",
        summary=(
            "A detection-only whisker sensor. It fires a “wall-stuck” event every "
            "tick a whisker is pressed hard, and a “miss” event when contact "
            "crosses a threshold (then stays quiet for a refractory window). A "
            "scent gate can soften the miss when smell is rising."
        ),
        formulas=(
            "<code>max_w = max<sub>i</sub> v<sub>i</sub></code><br>"
            "<code>wall_stuck: fire each tick max_w &gt; wall_thresh, intensity = max_w</code><br>"
            "<code>miss: if not refractory AND max_w &gt; thresh → intensity = max_w·(1 − gate), then refractory = refractory_ticks</code><br><br>"
            "<b>Terms.</b> max_w = maximum whisker contact; gate = scent-gate "
            "suppression ∈ [0,1]; refractory = quiet period after firing."
        ),
    ),

    "ScentGateReflex": ModuleDoc(
        title="ScentGateReflex — “smell is rising” gate",
        summary=(
            "Publishes a single “smell is climbing” signal used to suppress "
            "wall-aversion when the agent is closing on food. It tracks a fast and "
            "a slow average of the smell and outputs the clamped positive gap "
            "between them — zero unless smell is genuinely rising over its baseline."
        ),
        formulas=(
            "<code>EMA ← (1−α)·EMA + α·x</code> — short & long<br>"
            "<code>gate = clamp((short_ema − long_ema)/long_ema, 0, cap)</code>, and 0 unless <code>long_ema &gt; long_pos_min</code><br><br>"
            "<b>Terms.</b> EMA = Exponential Moving Average; ratio = fractional "
            "scent gradient (rise of short over long baseline); cap = maximum gate "
            "value; α = smoothing coefficient."
        ),
    ),

    "StuckEscapeReflex": ModuleDoc(
        title="StuckEscapeReflex — stuck detector + escape pulse",
        summary=(
            "Notices when the body is commanded to move but isn't (low actual speed "
            "versus intended), and after a sustained window fires a wall-stuck "
            "event and, optionally, a held rotation pulse to twist free. It can be "
            "gated on hunger so a resting agent doesn't thrash."
        ),
        formulas=(
            "<code>|intended| = √(Σ command²); &nbsp; |actual| = √(vx²+vz²)</code><br>"
            "<code>stuckness = clamp(1 − |actual|/|intended|, 0, 1)</code> — efference mode<br>"
            "<code>severity = mean(stuckness over window)</code><br>"
            "<code>hunger = clamp((sated − energy)/sated)</code><br>"
            "<code>fire if not refractory AND window full AND severity &gt; thresh AND hungry</code><br>"
            "<code>pulse: al = dir·rotation + noise, ar = −dir·rotation + noise</code> (dir = ±1 random)<br><br>"
            "<b>Terms.</b> efference (copy) = the internally commanded velocity; "
            "afferent = the actually-sensed velocity; ω = angular speed; severity = "
            "window-mean stuckness; refractory = quiet period after firing."
        ),
    ),

    "ForwardDriveReflex": ModuleDoc(
        title="ForwardDriveReflex — baseline forward pump",
        summary=(
            "The simplest possible drive: every tick, push both sides forward by a "
            "fixed thrust plus a little independent noise, giving a baseline swim "
            "that other reflexes and the brain can override. The independent "
            "left/right noise draws produce gentle turning drift."
        ),
        formulas=(
            "<code>noise ~ Uniform(−amp, +amp)</code><br>"
            "<code>al = thrust + noise_L; &nbsp; ar = thrust + noise_R</code> (independent draws)<br><br>"
            "<b>Terms.</b> thrust = symmetric forward accel command; amp = noise "
            "amplitude; al / ar = accel left / right."
        ),
    ),

    "DualEMADetector": ModuleDoc(
        title="DualEMADetector — trend-acceleration detector",
        summary=(
            "A general-purpose “something is ramping up” detector. It keeps a fast "
            "and a slow average of a scalar and fires an event when the fast one "
            "pulls far enough above the slow one — optionally only while the body "
            "is actually moving, with a refractory so it doesn't machine-gun."
        ),
        formulas=(
            "<code>EMA ← (1−α)·EMA + α·x</code> — short & long<br>"
            "<code>fire if initialized AND (optional long_ema&gt;0) AND not refractory AND motion ≥ floor AND short_ema &gt; long_ema·ratio_thresh</code><br>"
            "<code>intensity = short_ema</code><br><br>"
            "<b>Terms.</b> EMA = Exponential Moving Average; ratio_threshold = the "
            "multiplicative fire ratio (short must exceed long by this factor); "
            "motion floor = optional “must be moving” gate; refractory = quiet "
            "period."
        ),
    ),

    "AdaptiveThresholdTracker": ModuleDoc(
        title="AdaptiveThresholdTracker — self-calibrating threshold",
        summary=(
            "Instead of a hand-set alarm level, this watches a scalar's own running "
            "mean and spread and publishes a threshold that floats at “mean plus N "
            "standard deviations.” Detectors downstream compare against it, so the "
            "alarm level self-tunes to whatever the signal's normal range turns out "
            "to be."
        ),
        formulas=(
            "<code>mean ← (1−α)·mean + α·x</code><br>"
            "<code>var ← (1−α)·var + α·(x − mean_prev)²</code><br>"
            "<code>stddev = max(√max(0,var), min_stddev)</code><br>"
            "<code>threshold = mean + n_stddev·stddev</code><br>"
            "<code>warm = (samples_seen ≥ warmup_ticks)</code><br><br>"
            "<b>Terms.</b> EMA = Exponential Moving Average; var / stddev = variance "
            "/ standard deviation; n_stddev = standard deviations above the mean; "
            "warm = warmup-complete flag."
        ),
    ),

    "MotorFader": ModuleDoc(
        title="MotorFader — brain↔reflex blend + clash meter",
        summary=(
            "The mixer that actually blends the brain's motor command with the "
            "reflex command using the fader's α (α·brain + (1−α)·reflex). It can "
            "inject exploration noise scaled by how unsurprised / uncertain the "
            "policy is, and it measures “clash” — how much intent gets cancelled "
            "when brain and reflex pull opposite ways."
        ),
        formulas=(
            "<code>blended = α_eff·brain + (1 − α_eff)·reflex</code> &nbsp;(α_eff = 1 if idle-passthrough & reflex silent)<br>"
            "<code>noise_gain = amp·(1 − surprise)·(1 + entropy_gain·(1 − norm_entropy))</code>; &nbsp;<code>noise = noise_gain·N(0,1)</code><br>"
            "<code>norm_entropy = clamp(H/ln N)</code><br>"
            "<code>clash = max(0, |α·brain| + |(1−α)·reflex| − |α·brain + (1−α)·reflex|)</code><br><br>"
            "<b>Terms.</b> α = brain-vs-reflex blend (0 = reflex, 1 = brain); H = "
            "policy entropy; N = number of intents; N(0,1) = standard-normal noise; "
            "surprise = predictor surprise scalar; clash = intent erased when the "
            "two disagree in sign."
        ),
    ),

    "ActionGate": ModuleDoc(
        title="ActionGate — basal-ganglia priority arbiter",
        summary=(
            "The final gate on what motor command actually leaves the brain. It's a "
            "strict priority: an active exploration directive wins; otherwise the "
            "policy's (premotor's) command passes; if neither has produced yet, "
            "output zero. Think of the basal ganglia deciding which single action "
            "gets released."
        ),
        formulas=(
            "<code>if explore.active: accel = clamp(explore.accel), source = \"explore\"</code><br>"
            "<code>elif policy: accel = clamp(policy.weighted_accel), source = \"premotor\"</code><br>"
            "<code>else: accel = 0</code><br><br>"
            "<b>Terms.</b> explore = the ExplorationDirective pathway; premotor / "
            "policy = the PolicyToken pathway; weighted_accel = the policy's "
            "aggregated action command; clamp bounds the output to "
            "[accel_min, accel_max]."
        ),
    ),
}


# Shown for any module type without a bespoke entry (e.g. types that fall
# through to RawPayloadView). Keeps the panel present everywhere so the
# teaching surface is uniform.
_GENERIC = ModuleDoc(
    title="Live module snapshot",
    summary=(
        "This panel streams the module's raw internal state straight from the "
        "running brain, one update per tick. No bespoke visualisation or formula "
        "sheet has been written for this module type yet, so the fields below are "
        "shown as-is."
    ),
    formulas=(
        "<i>No formula sheet has been authored for this module type.</i><br>"
        "Each row is a scalar the module publishes in its diagnostic snapshot; "
        "consult the module's source in "
        "<code>cpp_core/src/ogma/modules/</code> for the exact update rules."
    ),
)


def doc_for(module_type: str) -> ModuleDoc:
    """Return the ModuleDoc for a C++ module type_name (never None)."""
    return DOCS.get(module_type, _GENERIC)


# MotorEPMv2 is the same module under the differ gate (a verified-identical copy that
# then grew levers), so it documents identically.  Aliased rather than duplicated so the
# two cannot drift apart.
DOCS["MotorEPMv2"] = DOCS["MotorEPM"]
