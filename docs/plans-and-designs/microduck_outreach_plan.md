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

## 6. Code sharing and packaging — how the brain reaches a robot

**No submodule in either direction.** Their tree is a Rust workspace of daemons with its own
cross-compile, release, signing and health-gate pipeline; a submodule of ours inside it would
put a C++ core into their build under reviewers who did not write it. A submodule of theirs
inside ours buys nothing: we never need their source at build time. We speak their socket
protocol, which `duck-ipc-proto` defines, and we vendor their body model for the simulation.

**The shape is the one their architecture already draws**: a platform repository owns the
protocol and the daemons; third-party clients live in their own repositories and speak the
socket. Intents in, state out, `safety` keeps the only write handle; `padd` and the perception
workers are the in-tree examples. Their `Hello` call carries an API version, so a client refuses
an incompatible robot the way their own clients do.

### 6.1 What crosses the boundary, and in which direction

| direction | what | how |
|---|---|---|
| upstream, as PRs | the two state fields; the autonomous design doc; later, if wanted, a way to test clients against a simulated robot; later still, if wanted, a documented community-component stanza | one thing each, §9 |
| downstream, vendored | their MJCF and scene files, their policy ONNX files, their protocol version | pinned by version and hash in a manifest in our repo, refreshed from their tags by a script; reviewable, no live coupling |
| never | the brain into their tree; their daemons into ours | |

### 6.2 The daemon as an updater component

Their update engine is config-driven: *each robot declares its components*, and a component is
a thing with its own version line, a pluggable source, a signed artifact and manifest, a
systemd unit, a post-install hook, a health gate and a rollback. Models are components too.
That is the delivery vehicle for the brain, and it makes "don't run our daemon" mechanical:
absent from the robot's updater config, absent from the robot.

| element | ours |
|---|---|
| repository | `ogma_duckd` lives in this repo: a **Rust shell** on their `duck-ipc-proto` crate around the **C++ core** behind a C ABI (design doc §4.1); cross-compiled to aarch64 in our CI (their `cargo board` never sees it) |
| release | GitHub Releases on our repo, tag `ogma-duckd-v0.1.0`; assets = `ogma-duckd-0.1.0.tar.zst` + `.minisig` + manifest, in their §5.2–5.3 layout: `bin/ogma_duckd`, `systemd/ogma-duckd.service`, `version.toml` (semver, minimum `robotd` API version), `hooks/postinstall` |
| signing | our own minisign key pair; the private key in our CI secrets, the public key published with the releases |
| the robot side | one stanza in `/etc/robot/updater.toml`, e.g. `[component.ogma-duckd]` with `source = github-releases`, our repo and tag pattern; `on_apply` restarts our unit only |
| health gate | our post-install hook and the unit's readiness: the daemon comes up, says `Hello` with its API version, subscribes to `robot.state`, and reports healthy on its own socket; a failure rolls back under their engine, not ours |
| off by default | the component is not in any robot's shipped config; a user adds it, and removing the stanza removes the daemon |
| compatibility | `version.toml` carries the minimum `robotd` API version; the daemon refuses to run above or below its window, in their own idiom |

### 6.3 What this needs from Pollen — the one architectural ask

A robot verifies a component only against the public keys baked into `/etc/robot/trusted_keys`.
A community component therefore needs one of:

1. its public key admitted to the trusted set — their key-custody decision, and the durable
   answer;
2. the developer sideload path their config already exposes (`allow_dev_keys` on a developer
   board) — enough for enthusiasts and for us, today, with no change on their side;
3. a documented "community component" affordance: a second trusted-keys directory, or a per-
   component key, so a user can opt one third party in without trusting it for firmware.

The design-doc PR raises this once, as a question about their trust model, and proposes nothing
until they answer. Path 2 is how the first hardware runs happen regardless.

### 6.4 Licences and attribution

Both repositories are Apache-2.0. Vendored files (MJCF, policies) carry their copyright in our
NOTICE. Machine-written code upstream carries `Co-Authored-By`, as PR-1 does.

## 7. Decisions the operator owns

- Whether the first contact is PR-1 (credibility first) or PR-2 (statement first). This plan
  recommends PR-1 first.
- Whether the standing report is made reachable from outside, and how.
- Whether to open a Discussion before either PR, if they use Discussions.
- Every push, every open, every reply.
- The signing key custody for our component releases (who holds the private minisign key).
