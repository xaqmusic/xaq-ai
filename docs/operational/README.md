# Operational docs — protocols, ground truth, and open metric questions

Ported from the pre-split `ami-ogma` repo on 2026-07-25 (the split was incidental — these were
simply not reached for). Naming: `ami_ogma` / `ogma` / `AMI-Ogma` == **xaq**.

**For how to actually run the current work — build, headless, the seed-avg harness, the A/B
protocol — see [`../../CLAUDE.md`](../../CLAUDE.md), not these.** These are the deeper protocol
and reference material behind it.

| Doc | Era | Why it's here |
|---|---|---|
| [`picrawler_geometry.md`](picrawler_geometry.md) | **current** | CAD/STEP-derived body geometry — chassis envelope, hip-attach pattern, segment lengths. Physical ground truth; doesn't go stale with the software. The reference for sim-to-real fidelity |
| [`../servo_dynamics.md`](../servo_dynamics.md) | **current** | The servo model, cited directly from `picrawler_body.gd`. Lives one level up because the code references that path |
| [`aliveness_metric_protocol.md`](aliveness_metric_protocol.md) | RL-era doc, **OPEN question** | Distance metrics can reward dead drift and select *against* closed-loop adaptation. The current metric set is distance-flavoured in exactly the way this warns about — tracked as open in [`../reports/picrawler_lever_ledger.md`](../reports/picrawler_lever_ledger.md) §7 |
| [`picrawler_curriculum_protocol.md`](picrawler_curriculum_protocol.md) | RL-era machinery, **live discipline** | The curriculum/trainer machinery is disowned, but its **contamination discipline** is permanent: how one careless launch silently invalidates weeks of paired-seed results |
| [`v4_methodology.md`](v4_methodology.md) | historical (2026-05-04) | The most complete single statement of the measurement protocols — what a claim required and how promote-or-kill was actually staged. Superseded on method by the doctrine, on procedure by `CLAUDE.md` |

## The recurring theme

Three of these five exist because a **measurement** failed, not a mechanism: a metric that
rewarded the wrong thing, a launch that silently contaminated an arm, a protocol that let an
underpowered claim get promoted. That is the same lesson as
[`../reports/archive/`](../reports/archive/README.md), and it is why
[`../../CLAUDE.md`](../../CLAUDE.md) §3.2 makes you validate the harness before recording any
negative verdict.
