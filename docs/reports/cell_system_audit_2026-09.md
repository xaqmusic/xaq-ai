# The Cell Navigator, audited — what the study's brain actually ran

> **Scope.** A peer-review-stance audit of the Cell navigator, the shared brain substrate it
> runs on, and the level of the microduck that will inherit its loops. It checks each claim
> the project's documents and configuration files make against the code, the configuration
> that ran, and a measurement, and it records where they disagree. The row-by-row register,
> with the file anchors a checker verifies, is the
> [appendix](cell_system_audit_2026-09_appendix.md); the process record is the
> [Cell lever ledger](cell_lever_ledger.md); the corrections to the study itself are the
> errata appendix of [the Cell report](cell_markov_blanket_loops_report.md).
>
> **Why now.** The study was run before the tooling could inspect the substrate closely, and
> a later campaign on the substrate found four mechanisms that the documents described and
> the code never ran. This audit looks for the rest before the next experiment is designed,
> so that the experiment measures the design rather than the bugs.
>
> **Status, 2026-09-06.** The code reading is complete; the reach-gate ablation and the
> seeding re-baseline have both run at n = 20 and are reported below.

## Executive summary

Does the Cell forage the way its report says it does? The report's behavioural numbers stand:
the four-loop composition forages no better than chance in its scent-poor room, removing its
exploration loop lifts it to the level of a hand-written reactive forager, and lesioning
vision degrades then restores food approach. What this audit changes is the account of *why*,
and of how much the numbers can be trusted to generalise.

Three findings change how the report reads. First, the study's "twenty randomised worlds"
shared one pillar layout, one body random stream and one arbitration random stream: the
harness passed a fixed seed to every run and the world never read the layout seed the harness
wrote for it. Only the two food sites and the starting position varied. The pairing design is
sound; the sampled population is far narrower than stated. Re-run over twenty genuinely
distinct worlds, every direction the report states survives at about two-thirds of the
effect and twice the spread, and the arm that tied the hand-written forager now trends
below it. Second, the exploration loop that
crowded out the planner never explored. Its climb toward novelty engaged on none of the
report's own recorded runs, because a stall latch forces it back to a memoryless wander
thirty ticks after the map stops growing. What won eighty percent of decisions was a random
walk with a recency scalar for a value. Third, the arbiter already contains the need-gated
weighting the report names as the missing next step, switched on by default, and switching
it to the form the report asks for changes nothing: play still takes four decisions in five
and eats do not move. The crowding is not a gating problem. It is a units problem. Every
loop's pragmatic score is hunger times a reach measured in scent, a few hundredths in a
scent-poor room, while the exploration loop's value is normalised to its own running peak
and sits at one. No gate on the exploration side can rescue a foraging side that never
exceeds five hundredths, and the same arithmetic makes the planner lose in the far-food
arena where it does score routes.

Two more findings change design decisions. The place map that the planner routes over is not
the learned vocabulary the architecture is built on. It is a grid over a path integral of the
body's own velocity and a heading that the body sets equal to its true world orientation,
drift-free and noise-free. That compass is a scaffold no document names, and the grid is the
kind of hand-rolled discretiser the project's own rules forbid in exactly this slot. And the
planner's epistemic term, described in every configuration comment, is multiplied by zero in
every configuration that runs.

The rest is hygiene with teeth. Six hundred and eighty-one configuration keys across the
shipping configs are silently dropped at load because no module declares them, seven of them
on the planner block where they read as a scent-gated policy. Three modules run with defaults
their schemas do not advertise. A voter parameter that the design notes call the trigger for
hierarchical growth is parsed and never read. The "hierarchy is configuration" claim is
instantiated by no live configuration. A far-food configuration places its food outside its
own arena.

The audit leaves behind instruments, not just findings: a warning at load for any dropped
key, a test that walks every shipping config, a test that compares every schema default with
the value that actually runs, a liveness gate that a configuration must pass before a battery
counts, a seed manifest the harness prints per job, and a register whose anchors a script
re-checks. It also leaves an inventory of mechanisms that are effective on their own terms
even where they fail this project's bar: the drift-free grid map, the value-race arbiter, the
reactive chemotaxis specialist, and the wander. A game character would be well served by any
of them.

## 1. Method

The unit of audit is a claim. Each row of the register pairs a claim, as a document or
configuration states it, with what the code does, the anchors that prove it, the measurement
that settles it where one is cheap, a status, a severity, a re-use context, and a utility
note. A row enters only with a code anchor or a measurement. Every status is scoped to a
configuration and a harness, never global: the project's own rule that nothing is refuted
except in the context it was tried applies to claims as it does to levers.

Ten statuses were added to the lever vocabulary for claims and harnesses: unsupported,
dead parameter, dropped parameter, default trap, unnamed scaffold, invalid harness, broken
configuration, misattributed, rule violation, and open. The order of work put the items that
change the report's reading first, then module liveness in the configurations that actually
run, then sensor legality, then the doctrine's own rules, then the documents' disagreements
with each other, then test coverage, then the duck.

Utility is recorded because a failed bar is a statement about this project's discipline, not
about whether a mechanism is any good. Section 5 collects the ones worth keeping.

## 2. What the study's brain actually runs

The study configuration wires four loops into one arbiter. A scent-following loop climbs a
scalar scent sample taken at the body's centre; a planner routes over a place graph to
remembered food; an exploration loop grows that graph; a vision loop closes on food it can
see. Each loop emits a heading and a confidence, and each has its own heading controller
feeding one channel of a motor bus. The arbiter scores each loop every tick as a pragmatic
term, the body's hunger times the loop's self-reported reach, plus an epistemic term, and
gives the motor to the winner with an adaptive hysteresis.

Three things in that description are not what the documents say. The epistemic terms are
gated by one minus the largest pragmatic reach, so a hungry body that can reach nothing keeps
exploring at full weight; the documents describe a fixed weight. The planner's own epistemic
term, its frontier novelty, is computed, published, subscribed, and then multiplied by zero
because a flag is off in every configuration. And the place graph the planner and the
explorer share is not shared: each keeps its own grid over its own path integral, at
different cell sizes, and the perceptual module meant to supply the map supplies only a
novelty scalar.

The seed flows from the harness through one environment variable into the world's layout,
the body's random streams and, through the host, into every module that names a seed. The
harness fixed that variable at one value for every job, so all of those were identical across
the twenty "worlds". The exploration loop's own seed, patched per job, was the only stream
that varied, and the food layout was drawn from that same integer.

## 3. Findings that change the report's reading

- **One pillar world.** The report's methods sentence, and the raw file's own header, say food
  positions and pillar layout were drawn per seed. The harness passed a literal fixed seed to
  every run and the world took its layout from that seed; the layout seed the harness wrote
  into the configuration's metadata was never read. The confidence intervals in the report
  are over twenty food layouts in one room with one set of random streams. The repaired
  harness draws a distinct world and a distinct exploration seed per job and prints both; a
  legacy mode reproduces the old behaviour exactly, so the re-baseline is paired against a
  faithful control. Over twenty distinct worlds: the specialist 2.30 eats, the composition
  0.75, the composition without play 1.65, the gate ablation 0.60. Against the specialist the
  composition loses by 1.55 (t −3.5) and the minus-play arm by 0.65 (t −1.6, not
  significant); minus-play beats the full composition by 0.90 with a spread of 1.48 (t −2.7)
  where the single fixed world gave 1.40 with a spread of 0.82 (t −7.6). Every direction
  stands; every spread roughly doubles; the tie becomes a trend below.
- **The explorer did not explore.** The report's raw file records the exploration loop's
  climb fraction as zero on every run. The loop climbs only while two conditions hold: a
  strictly more novel neighbour exists, and the map has grown within the last thirty ticks.
  The second fails permanently once the room is tiled, and the loop is then a run-and-tumble
  wander whose value is how recently it was somewhere else. The mechanism section's
  explorer-that-crowds is a random walker. The diagnostic stream now carries both conditions,
  and a sixty-second liveness run shows the first holding on a handful of samples.
- **The gate the report asks for is already on, and the other form does not help.** The
  arbiter's need gate, on by default, suppresses exploration only when some pragmatic loop can
  reach something. The report's proposed next work, need-gated epistemic weighting, is that
  shipped default; the hunger-only gate it describes is the ablation. Run in the report's own
  world over twenty paired layouts, the ablation is a null: eats 0.6 against 0.5, a paired
  difference of a tenth of an eat with a standard deviation ten times that, and exploration's
  share of decisions falls only from ninety-one to eighty-two percent. The mechanism section's
  diagnosis is therefore wrong in both directions, and the cause is in the arbiter's units,
  below.
- **The perturbation test ran in a different world.** The sensor-dropout figure comes from a
  pillar-free room at one fixed seed with five exploration seeds, on a configuration with no
  fusion machinery at all. Its result stands as a within-subject signal in that room. A
  correction elsewhere in this project that attributed the recovery to the fusion module's
  confidence floor was wrong; the fusion module was not present.
- **The leave-one-out lever for the scent loop does not deaden it.** Clearing its input zeroes
  its epistemic spike, but its pragmatic reach comes from a self-reported capability that
  survives, so the loop can still win. The minus-scent arm's arbiter histogram will say how
  often it did; pending.
- **Names and numbers.** The planner module the appendix names does not exist; the room is
  forty metres, not sixteen; the scent field is a screened-Poisson relaxation, not an
  exponential. The study configuration ships with its vision loop weighted to zero; the raw
  file shows the report's runs patched it on.

## 4. Findings that change a design decision

- **The map is a grid over a perfect compass.** Both the planner and the explorer integrate
  egocentric velocity rotated by the published heading into world coordinates and bin them at
  a fixed cell size given in ticks-per-speed units and documented as metres. The heading is
  world yaw by construction: the body integrates its commanded rotation and then sets its
  orientation to the result. Nothing drifts, so a fixed grid works. This is legal as an
  integrated own-yaw signal, and it is a scaffold, and no document names it. The doctrine's
  first rule, that anything discrete comes from the learned vocabulary and never from a
  hand-rolled binner, is violated in the one place the architecture most needs it to hold.
  Round 2's single build lever replaces the grid with the vocabulary and keeps the grid as the
  oracle it is measured against; a heading-drift perturbation names the scaffold by breaking
  it.
- **The scent loop's confidence is an average on eat events.** Its capability is the current
  scent over a running average of scent-at-eat, updated on the world's ground-truth eat event,
  and that scalar is the loop's pragmatic term in the arbiter. The doctrine warns against
  exactly this construction. Its replacement is the loop's own approach error graded by its
  own success.
- **Food memory is a value function on a ground-truth event.** The planner adds a fixed
  amount at the current node on every eat, propagates it, and never decays it; only camping
  disconfirms it. Whether a homeostatic hit memory is reward shaping is a question the
  operator decides; the register holds it open with both readings.
- **The panorama is a staircase.** The place signature is a mean colour per absolute heading
  bin, refreshed every ninety ticks, about four and a half metres of travel, and held between
  refreshes. The place vocabulary sees long runs of identical input. This is the conditioning
  failure the doctrine describes; the scatter that would confirm it is queued.
- **The arbiter's shared units are not shared.** The doctrine forbids a value race between
  quantities on different scales and prescribes precision weighting; the shipped arbiter
  races on what its comments call shared units. They are not. The scent loop's epistemic
  spike and the exploration loop's value are each divided by their own running peak, so both
  saturate at one; the two pragmatic scores are hunger times a raw, scent-scaled reach. In
  the study room the pragmatic scores peak between three and five hundredths while
  exploration sits at one. In the far-food arena with pillars, the planner scores a route on a
  third of the samples, its score peaks at fifteen hundredths, and it never wins a decision.
  This is the mis-scaled proxy the doctrine names, measured. It moves the arbitration lever up
  the round-2 order, since no map lever can show anything while the planner cannot take the
  motor. The recipe states the horizon as one step, greedy, and holds the precision question
  open until that lever runs.
- **Hierarchy is configuration, in theory.** The claim that a higher-level vocabulary is the
  same module on the consensus topic with an identity encoder is instantiated by no live
  configuration; every such configuration is archived. The code path may work. Nothing has
  measured it. The voter flag the growth design consumes is parsed and never read.

## 5. Interesting artifacts

Each of these works well on its own terms. Each fails the bar here for a stated reason. Each
is worth having somewhere.

| artifact | what it does well | why it fails the bar here | worth having for |
|---|---|---|---|
| the grid place map over a path integral | a zero-drift, constant-time spatial memory that made routing to remembered food work | a hand-rolled discretiser in the vocabulary's slot; depends on a perfect compass | any engine that has true pose: a non-player character's memory; the oracle the learned map is measured against |
| the value-race arbiter with adaptive hysteresis | a small, legible policy selector with no dwell constant | one-step and greedy; its normalisation may hide the scale mismatch the doctrine forbids | behaviour selection for game agents and simple robots where a designer names the policies |
| the reactive chemotaxis specialist | out-forages the whole composition in a monotonic-gradient world at almost no cost | its confidence is an average on reward events | the baseline every regime carries; a gradient follower for anything with a scalar field |
| the wander with habituation | cheap, robust room coverage without a map | its value collapses to recency; its climb never engages | exploration for game characters; a coverage baseline |
| food-memory value iteration | remembers sites and routes to them; disconfirms when camping | a reward-like term on a ground-truth event | resource memory for game agents; the pragmatic half of any planner once the event is inferred |
| the absolute-heading panorama | a place signature from one camera with no features | a whole-frame mean per bin, refreshed every ninety ticks, world-anchored | cheap place recognition wherever a compass exists |
| the Cell world | a fast headless foraging testbed with dense proxies | its harness seeding was invalid; two configurations were broken | a general foraging benchmark now that seeding is repaired |
| the paired harness | paired seed-averaged comparison and a three-phase perturbation protocol in one script each | the seeding bug | the template for the duck's level-two harness |

## 6. Cruft, with dispositions

| class | count | disposition |
|---|---|---|
| dropped configuration keys (present in a config, absent from the module's schema) | 681 across the shipping configs; the largest classes are two premotor learning rates, a whisker reflex strength, motor-bus labels, and the seven planner keys | warned at load and in the builder's validation; a census test asserts clean configurations and grows its set as the config pass cleans them |
| schema defaults that are not the effective defaults | five: the bake threshold in two modules, a whisker steer gain, a place navigator's arrival window, a saccade reflex's scent gate | pinned in a test; each realignment is a lever with a byte-identity cost |
| parsed and never read | the voter's novelty threshold | the growth design's first gate has no localizer until it is wired |
| dead by configuration | the planner's epistemic term; the planner's precision channel | one is round 2's lever; the other is marked in the recipe |
| never instantiated | the level-N vocabulary over consensus; mitosis in any live configuration | the documents now say so |
| broken configurations | the two far-food configurations | repaired in round 2's first stage |

## 7. Test coverage

The arbiter, the explorer, the planner, the place navigator, the two bearing modules and the
two chemotaxis modules have unit tests. The heading controller, which is the action layer of
every loop and carries a learned advance policy, has none; nor do the scent compass, the
motivation gate, or the goal belief. On the planner, the cell size, the escape gain and the
disconfirmation are untested, and the arbiter's planner-epistemic test exercises the path
every configuration disables. Two tests were added by this audit: the configuration census
and the schema-default check.

## 8. Claims we are not making

- We do not claim the report's behavioural results are wrong. They stand as measured; what
  changes is the population they were measured over and the mechanism offered for them.
- We do not claim the explorer cannot explore. Its climb is one flag from engaging; whether
  it then helps is round 2's second lever.
- We do not claim either form of the need gate matters. Both were measured; neither does.
- We do not claim the grid map is bad. It is an effective mechanism in the wrong slot for
  this project's purpose.
- We do not claim food memory is reward shaping. It is a question the operator decides.
- We do not claim the level-N path is broken. It is unmeasured.

## 9. Next work

Round 2 of the Cell ran on the repaired far-food arena with pillars, calibrated so that a
reactive forager's return to relocated food is nearly chance. Its result, recorded in the
round-2 charter and the Cell ledger: the composition's memory works once the exploration
loop is out of the race (the two-loop brain ties the reactive specialist and beats the
gradient-blind floor over twenty worlds), and no repair of the exploration loop, the
arbiter's units, the map, or the arbitration's currency rescues the four-loop brain while
that loop holds the motor. The exploration loop is dropped from the Cell's instance of the
recipe. What follows is the perturbation battery on the two-loop brain, the L-bend as its
transfer room, the duck's level-two harness, and a competence-graded arbitration in which
each loop is weighed by its own prediction error about the world. Its endpoint is the loop and
arbitration recipe the microduck will inherit, with a level-two harness so that the port's
first verdict is not a single seed.
