# Outreach to Pollen Robotics — the PRs as communication (2026-09-03)

**Status: DRAFT for discussion. Nothing is pushed, opened, or sent; all of that is the
operator's call (REPORTS.md §9.6).**

## 1. The problem with a "big statement" PR, and its solution

REPORTS.md §9 says one thing per PR and that a change which needs our framework to look
worthwhile is the wrong change to send. A pull request that carries our intent and our high-level
plans in its description would violate both, and it would land on a reviewer who did not ask for a
manifesto. The statement needs its own vehicle.

Pollen's repository supplies one. Their culture is design-doc-first: eight documents in
`docs/design/`, each owning one subsystem, and their own `docs/ideas/autonomous_behavior.md`
opens with *"The brain is the biggest untracked gap in the parity audit… no design doc owns it
yet."* **The statement is a design document in their repository, in their format and voice,
offered as a pull request that adds one file.** It is one thing. It is the thing they said was
missing. And it can carry the whole plan, because a design doc is where plans belong.

## 2. The sequence

| # | what | why this order |
|---|---|---|
| 0 | **Preparation** (operator): confirm how they take proposals (Discussions on? issues? a PR to `docs/design/`?), read their last twenty PRs and the review tone, decide whether the standing report can be linked from outside (our repo's visibility) or needs a public copy | a stranger's first message sets the whole relationship |
| 1 | **PR-1: joint velocities and currents on `robot.state`** (prepared: `state-velocities-currents`, 199 lines, tests 205 → 208, fmt and clippy clean, cost measured at +235 B/frame) | a gift with no philosophy attached; the smallest possible way to show how we write and how we validate; earns the review relationship before the ask |
| 2 | **PR-2: `docs/design/autonomous-brain.md`** — the statement | opened after PR-1 has been reviewed (merged or not): they have seen our care before they read our plan |
| 3 | **PR-3: a simulated robot for client authors** — only if PR-2's discussion wants it | it helps every client, not us alone; shape decided with them, not for them |
| 4 | `ogma_duckd` in our repository, with a README written for their users | their repo receives only the hooks; the daemon is ours to run and theirs to ignore |

The report is evidence, not a deliverable to them: PR-2 links it once, for the numbers.

## 3. What PR-2 says, in their terms

Structure per §9.3, narrowing: title, their situation, the change, cost, validation, out of scope.

- **Title, in their vocabulary.** "A design for the autonomous behaviour stack: one brain that
  the runtime's sixteen states become inputs to."
- **Open with their situation.** Their words: no design doc owns the brain; the sixteen-state
  machine is to be ported; their shape note says presence, mood and the beat are inputs to one
  brain, not modes beside it. That note *is* the design; we are proposing how to build it.
- **The change.** A document that specifies the boundary (intents in, state out, `safety` keeps
  the only write handle, a client that forks nothing and is off by default), the arbitration
  rule for who drives the joints when, the parity ladder, and the metrics. It names our brain as
  one implementation and leaves room for theirs; it does not name the framework.
- **What we will not do.** Touch locomotion. Ask anyone to run our daemon. Fork. Claim hardware.
- **Cost.** A document; a review hour. Measured elsewhere: nothing in their tree changes.
- **Validation, scaled to evidence.** One sentence with three numbers from the report (six of six
  starts stand in fifteen minutes; 2 N caught 35 of 36; unchanged under their servo filter), each
  with "in simulation, on your MJCF, six seeds", and what it does not show (no hardware, no
  walking, half their policy's envelope).
- **The ask.** Feedback on the boundary: the two fields (PR-1), and whether a simulated robot for
  client authors is something they want.
- **Attribution.** `Co-Authored-By` for the machine-written parts, as PR-1 carries; no session
  links, no paths into our repo.

Vocabulary that must not appear, even defined: EPM, TLE, Markov blanket, homeokinetic,
precision-weighted, active inference. What replaces them: "a self-model learned by babbling",
"the error between what it expected to feel and what it felt", "the boundary between the brain
and the rest of the robot", "which behaviour wins is decided by how badly each is surprised".

## 4. Risks, and the sentence that answers each

| risk | the answer, said early |
|---|---|
| we look like we are competing with the RL stack | "Your locomotion works and transfers; this sits above it and asks it to walk. A user who keeps everything as it is loses nothing." |
| overclaiming from simulation | every number carries "in simulation" and its seed count; the report's "claims we are not making" travels with it |
| a manifesto in a PR | the plan lives in the design doc, the PRs stay one thing each |
| our jargon leaks | the §9.7 checklist, run by a second reader before the operator sees it |
| timing against their release cadence (0.10.0 just shipped) | PR-1 is small enough for any window; PR-2 waits for PR-1's review |
| the reviewer is a stranger doing us a favour | every paragraph either reduces their review work or is cut |

## 5. A draft opening for PR-2, for the operator's ear (not to be posted)

> `docs/ideas/autonomous_behavior.md` says the brain is the biggest untracked gap in the parity
> audit and that no design doc owns it. This adds one. It proposes the shape your own shape
> notes ask for: presence, mood and the beat as inputs to a single brain rather than modes beside
> it, with the sixteen runtime states falling out of what that brain is trying to keep true about
> itself and its surroundings.
>
> The boundary is the one `robotd` already draws. The brain is a client like `padd`: it
> subscribes to state, sends intents as notifications at your rate, and `safety` keeps the only
> write handle. It is off unless started, and a duck without it behaves exactly as today. It
> does not touch the trained policies; it asks them to walk, look and pick, and it hands the
> joints back the moment it is done.
>
> We have one implementation of the idea running in simulation on your MJCF, and its numbers are
> in a report linked at the end: from a cold start it stands on six of six seeds inside fifteen
> minutes, catches a 2 N shove 35 times in 36, and is unchanged under the servo filter your
> firmware applies. It does not walk, it has never run on hardware, and its push envelope is about
> half of `alpha_stand`'s. Those limits are the reason the design leaves locomotion where it is.

## 6. Decisions the operator owns

- Whether the first contact is PR-1 (credibility first) or PR-2 (statement first). This plan
  recommends PR-1 first.
- Whether the standing report is made reachable from outside, and how.
- Whether to open a Discussion before either PR, if they use Discussions.
- Every push, every open, every reply.
