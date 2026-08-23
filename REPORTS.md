# REPORTS.md — how to write a formal report in this repo

**This governs formal reports only** — the documents in `docs/reports/` that an outside
reader might be handed. It does not govern the ledger, plan docs, commit messages, or
conversational replies, which have their own jobs and their own registers.

The reference instance is
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

## 6. Jargon, abbreviations, and symbols

- **No project-internal jargon** unless it is defined in the executive summary and earns
  its place by being used repeatedly afterward. Prefer plain description over a defined
  term used twice.
- **No code identifiers, parameter names, or variable symbols in prose.** Write "the
  strength of the coupling between legs", not `coupling_gain`. Configuration names,
  filenames, and parameters belong in an appendix or a methods note where a replicator
  can find them.
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
- [ ] Every number has its sample size; every null has its power
- [ ] Falsified hypotheses framed as findings
- [ ] Ends with concrete next work
- [ ] "Claims we are not making" present
