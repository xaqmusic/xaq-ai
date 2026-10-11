# REPORTS.md — how to write for a reader outside this repo

**This governs writing that leaves the project.** Two kinds, sharing one standard:

| | Governed by | Reader |
|---|---|---|
| **Formal reports** — `docs/reports/` | §1–§8 | curious, outside active inference and machine learning |
| **Outward-facing communication** — pull requests, issues, and commit messages in someone else's repository | §3 and §9 | an expert in *their* codebase who knows nothing about ours |

It does not govern the ledger, plan docs, commit messages in **this** repo, or conversational
replies. Those have their own jobs and their own registers.

The reference instance for a report is
[`cell_markov_blanket_loops_report.md`](docs/reports/cell_markov_blanket_loops_report.md).
Read it before writing a new one.

---

## 1. The audience, and what the report is for

**Write for an intelligent reader who is outside active inference and outside machine
learning.** Assume curiosity, assume no vocabulary. A robotics-adjacent engineer, a
research funder, a colleague from another field, a future contributor on their first day.

A report has four jobs, in this order:

1. **Invite.** The reader should feel welcomed into the ideas, not audited by them.
2. **Educate.** They should leave understanding something they did not understand before.
3. **Persuade.** These are good ideas and worth someone's attention; say so by showing them.
4. **Record.** The evidence, stated so a skeptic can check it.

A report is a **stepping stone, never a final chapter.** Every one of them ends with more
work to do, and says what that work is.

## 2. Structure: broadest first, narrowing monotonically

**The reader must be able to stop at any point and have missed nothing that matters at
their chosen depth.** This is the single structural rule everything else serves.

| Layer | Contains | A reader who stops here leaves with |
|---|---|---|
| **Executive summary** | The whole story in plain language | What we built, why it matters, what we learned |
| **Introduction** | The system, the concepts, the vocabulary | Enough to follow any later section |
| **The object under test** | What specifically was built and why | The design and its rationale |
| **Results** | What happened, with numbers | The evidence |
| **Mechanism / discussion** | Why it happened | The understanding |
| **Limits and next steps** | What is not shown; what comes next | An accurate sense of maturity |

Never introduce a concept below the layer where a reader first needs it. Never require a
later section to make an earlier one honest.

### The executive summary

Treat it as an abstract written for someone outside the field. It is the most important
part of the document and usually the only part most readers finish.

- **Open with the question or the capability, not the apparatus.** "Can a robot find its
  own reflexes?" lands; "This report evaluates the GainEvolver module" does not.
- **Use a concrete analogy** to anchor each unfamiliar concept, and let it do real work
  rather than decorate. A good analogy survives being pushed on; drop any that has to be
  apologized for.
- **Define every term you will use later**, including the ones that feel obvious from
  inside the project.
- **State the outcome.** Do not withhold the result to create suspense.
- **No numbers without their meaning.** A figure appears here only if the sentence around
  it tells the reader why that figure is good or bad.

## 3. Language

### Do not pre-qualify your own honesty

Phrases like *"the honest take"*, *"to be direct"*, *"stated plainly"*, *"in truth"*,
*"what really happened"* **imply that the surrounding text is less honest.** They achieve
the opposite of their intent: a reader who meets "the honest answer is X" begins wondering
which of the other answers was not. Every sentence in the report is the honest one. Write
it and move on.

The same applies to *"to be fair"*, *"I should note"*, *"it must be said"*, and
*"transparently"*.

The rule is symmetric: **do not certify your own candour after the fact either.**
*"That is reported as a result, not buried"*, *"we state this openly"*, *"this is not
hidden"* fail exactly as the pre-qualifiers do, by inviting the reader to ask what else
might have been buried. Report the result; the reporting of it is the candour.

### Banned constructions

These are the tells of machine-written prose. They are not wrong so much as they are
noise, and a reader who spots the pattern stops trusting the content.

| Avoid | Because | Instead |
|---|---|---|
| "It's worth noting that…", "Importantly,", "Notably," | Throat-clearing; if it were not worth noting it would not be there | Say the thing |
| "This isn't just X — it's Y" | A rhetorical template, instantly recognizable | State Y |
| Triads everywhere ("faster, cleaner, and more robust") | Rhythm substituting for content | Use the number of items you actually have |
| Bolding many phrases per paragraph | Emphasis inflation flattens to noise | At most one emphasis per paragraph, and only where a reader skimming must not miss it |
| "significantly", "dramatically", "substantially" without a number | Intensifier standing in for evidence | Give the figure |
| "Let me…", "Let's…", "We'll now…" | Conversational scaffolding; a document is not a chat | Delete; start the sentence |
| "In this section we will…" | Meta-commentary about the document | Delete; the heading did that |
| Closing paragraphs that restate the opening | Adds length, not information | End on what comes next |
| Stacked hedges ("might perhaps somewhat suggest") | Reads as evasion | One hedge, or a stated confidence level |
| Em-dashes as a default connector | A recognizable rhythm when overused | Vary: commas, colons, full stops, parentheses |

### Register

Plain, warm, declarative. Contractions are fine. Humor is fine if it is dry and rare.
Address the reader directly when it helps ("if you have ever watched a foal stand…").
Avoid grandiosity: no "revolutionary", "paradigm", "unprecedented". The work is
interesting enough to describe accurately.

## 4. Tone: positive, and genuinely so

**The tone is confident and generous, including — especially — when a hypothesis was
falsified.** A prediction that failed sharpened the picture, and that is the ordinary
mechanism of progress rather than a consolation.

- Frame a falsified hypothesis as **what it taught**, not as what was lost. "The result
  showed that X was not the constraint, which redirected the work to Y" is a finding.
- Never apologize for a null. State it, state its power, and state what would resolve it.
- Never oversell either. Enthusiasm is carried by clear description of real results, not
  by adjectives attached to weak ones.
- There is always more work. Say what it is, concretely.

## 5. What belongs elsewhere

**Do not narrate the investigation.** Wrong turns, false starts, analysis bugs, tools that
misreported, protocol errors caught mid-flight — none of it belongs in a report. It is
valuable, and it is what
[`picrawler_lever_ledger.md`](docs/reports/picrawler_lever_ledger.md) is for.

The distinction worth holding:

- **A falsified hypothesis is a RESULT.** It belongs in the report. "We predicted the
  criterion was blind to coordination; measurement showed it was not."
- **A process mistake is PROCESS.** It belongs in the ledger. "The first version of the
  analysis used the wrong test."

Also excluded from reports: self-correction of earlier drafts, apologies, and any sentence
whose subject is the author's own reasoning.

This extends to **the document's own history**. A report describes the system, not its
own drafting: *"the external baseline this report had been missing"*, *"a section we
previously lacked"*, *"now finally measured"*. The reader did not read the earlier draft
and cannot be told what it was missing. Present what is here, not the gap it closed.

## 6. Jargon, abbreviations, and symbols

- **No project-internal jargon** unless it is defined in the executive summary and earns
  its place by being used repeatedly afterward. Prefer plain description over a defined
  term used twice.
- **No code identifiers, parameter names, or variable symbols in prose.** Write "the
  strength of the coupling between legs", not `coupling_gain`. Configuration names,
  source filenames, and parameters belong in an appendix or a methods note where a
  replicator can find them.
- **Companion documents are citations, not identifiers, and belong inline.** This rule
  bans code, not references. A reader who meets "the doctrine already required this"
  with no antecedent cannot follow it up; name the document where they first need it and
  link it. Burying a cross-reference in an appendix to satisfy the line above is a
  misreading of it.
- **Expand every abbreviation on first use**, in the standard form: "Episodic Predictive
  Module (EPM)". Then use the short form consistently.
- **Greek letters and mathematical symbols** get a plain-language gloss the first time:
  "the step size (σ, sigma)". If a symbol appears only once, replace it with words.
- Equations are welcome where they are the clearest statement, and each is followed by a
  sentence saying what it means in words.

## 7. Evidence

- Numbers carry their **uncertainty and their sample size**: "0.25 ± 0.55 across 20 seeds".
- Distinguish **signal from finding** as the doctrine does: a handful of fixed seeds
  promotes or kills a direction; a finding needs power. Say which one a number is.
- When a result is not significant, **say so and give the minimum effect the test could
  have detected.** A null without its power is not information.
- Every claim a skeptic might challenge names the measurement behind it.
- A **"Claims we are not making"** section near the end is expected. It is the fastest way
  to earn a careful reader's trust.

## 8. Checklist before shipping a report

- [ ] Executive summary readable by someone outside the field, with the outcome stated
- [ ] Every acronym expanded at first use; no bare code identifiers in prose
- [ ] Scope narrows monotonically; stopping anywhere leaves nothing important behind
- [ ] No "honest"/"to be fair"/"plainly" pre-qualifiers
- [ ] No process narration, no self-correction, no apologies
- [ ] No sentences about the document's own drafting or what an earlier draft lacked
- [ ] No claims that the report is being candid, before or after the fact
- [ ] Companion documents named and linked where first needed
- [ ] Every number has its sample size; every null has its power
- [ ] Falsified hypotheses framed as findings
- [ ] Ends with concrete next work
- [ ] "Claims we are not making" present

---

## 9. Outward-facing communication: pull requests, issues, upstream commits

Everything in §3 (language, the ban on pre-qualifying your own honesty, the banned
constructions) applies here word for word. This section is what changes on top of it.

### 9.1 The reader is a stranger doing you a favour

A report's reader is curious and chose to be there. A pull request's reader is an expert in a
codebase we are visiting, knows nothing about ours, and did not ask for any of this. Reviewing
costs them time they had allocated to something else.

**Their attention is the scarce resource, and every line either reduces their review work or is
noise.** That single test settles most questions of what to include.

We are guests. A contribution is a gift only if it does not arrive as a bill.

### 9.2 Our vocabulary does not travel

No "EPM", "TLE", "Markov blanket", "homeokinetic", "precision-weighted", or "active inference"
in an upstream pull request. Not defined-on-first-use, as §6 would allow in a report. Absent.

The rule behind it is sharper than a style preference: **if a change only makes sense once the
reader accepts our framework, it is the wrong change to send.** Split it until each piece
stands on its own merits, stated in their terms, against their problem. A contribution that
cannot survive that split is one we should keep and run ourselves.

It is also the test that separates augmenting from competing. Anything that needs our
philosophy to look worthwhile is competing.

### 9.3 The shape of a pull request

Narrowing monotonically, on §2's rule, at a much smaller scale:

| Layer | Contains | A reviewer who stops here knows |
|---|---|---|
| **Title** | the situation or the change, one line, in their vocabulary | whether to open it |
| **Opening** | what today's code does and what that costs, in their terms | whether they care |
| **The change** | what it does, minimally described | what they are approving |
| **Cost** | measured, with numbers | what it takes from them |
| **Validation** | what was run, and what that does *not* prove | how much to trust it |
| **Out of scope** | what you deliberately did not do | that the diff is the whole story |

Lead with **their** problem, never with our need. "The servos report velocity and load every
tick, and the wire dropped both" opens a review. "Our brain needs load data" opens an argument.

### 9.4 Rules specific to a pull request

- **One thing per PR.** A second change bundled in is not efficiency, it is a debate attached
  to a merge.
- **Measured cost, in numbers you took yourself.** "A modest increase" asks the reviewer to do
  your measuring. Give them the bytes, the milliseconds, the allocation.
- **Say what you did not do.** Scope discipline is invisible unless stated, and stating it is
  the cheapest trust available.
- **Never describe their design as a mistake.** A reasonable design that does not yet do
  something is not a bug, and a contributor who arrives correcting people is not read twice.
- **Match their conventions, including the ones we would not have picked** — commit subject
  style, test idiom, doc-comment voice, error wording, whether they sign off at all. Read ten
  of their commits before writing one. Our house style has no standing in their repository.
- **Scale claims to evidence** (§7). "Validated against their own fake IO, which exercises the
  control loop but not a servo" is worth more than "fully tested", and it is what a maintainer
  needs in order to know what still needs a bench.
- **Out-of-scope findings get mentioned once and left alone.** Fixing something you noticed on
  the way turns one reviewable change into two unreviewable ones. Note it; move on.
- **No links a stranger cannot open**, and none they can open but should not have to: our
  internal docs, our branch names, our file paths, our ledger.

### 9.5 Attribution, and what never leaves

- **The `Co-Authored-By` line stays.** The work is machine-written and a maintainer merging it
  is entitled to know that without having to ask.
- **Private session URLs never travel.** Our own commits carry one; an upstream commit must
  not.
- **Our DCO sign-off is our convention, not theirs.** If their history has no `Signed-off-by`,
  adding one is noise. If they require it, follow their instructions exactly.
- **Real network identifiers never travel — and this repo is public, so that includes our
  own docs.** A wifi SSID, a BSSID or MAC, a public IP: use the placeholders below, and keep
  the real values in `docs/operational/local_env.md`, which is gitignored.

  | real thing | write this |
  |---|---|
  | wifi SSID | `<your-ssid>` |
  | BSSID / MAC | `AA:BB:CC:DD:EE:F0` (`:F1` for a second radio) |
  | public / static IP | `<your-public-ip>` |

  RFC1918 addresses (`10.0.0.114`, `192.168.x.x`) are **fine** and appear throughout the
  port doc — they mean nothing outside the LAN and redacting them would cost readability
  for no gain.

  ⚠ **This is a measurement rule, not just a privacy one.** The 2026-09-05 leak happened
  because a wifi-roaming session was written up *honestly* and the BSSIDs were the
  evidence for which radio the robot had attached to. That instinct is right — the fix is
  not to write up less, it is to put the identifier in `local_env.md` and keep the
  measurement in the report. A BSSID lock is reproducible from "lock to the 2.4 GHz radio
  of your own AP"; nobody else can use yours. `.githooks/pre-push` enforces this, but it
  is opt-in (`git config core.hooksPath .githooks`) and bypassable, so it is a backstop
  for this rule rather than a substitute for it.

### 9.6 Opening it is the operator's call, always

Writing a pull request is reversible and is ordinary work. **Opening one is not**: it is a
public act in someone else's project, under the operator's name, that others will respond to.

Prepare it on a local branch, run their gates, and show the operator what it would say. Pushing
a fork, opening the PR, and replying in its thread all wait for an explicit go-ahead. The same
holds for issues, discussions, and any reply to a maintainer.

### 9.7 Checklist before showing a pull request to the operator

- [ ] Title states the situation in their vocabulary, not ours
- [ ] Opens with what their code does today and what it costs
- [ ] No project-internal jargon anywhere, defined or otherwise
- [ ] The change would stand up without any reference to our framework
- [ ] One thing only; a second improvement was noticed and left alone
- [ ] Cost measured, with the numbers in the description
- [ ] Validation says what it proves *and* what it leaves for a bench
- [ ] An explicit out-of-scope note
- [ ] Their formatter, linter, and test suite run clean, with before/after counts
- [ ] Commit subject and body match the style of their last ten commits
- [ ] `Co-Authored-By` present; no session URL, no internal links, no our-repo paths
- [ ] Nothing pushed anywhere, and nothing opened
