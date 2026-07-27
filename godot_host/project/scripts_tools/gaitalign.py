#!/usr/bin/env python3
"""Phase-0 gait-alignment diagnostic — is the power stroke phase-locked to ground contact?

THE QUESTION.  The picrawler's gait runs on two per-leg clocks that nothing ever forced to
agree:

  * THRUST  — the power stroke on hip1, `y[0] += amp * sin(L.phase + stroke_phase)`, where
              L.phase is derived from the KNEE (MotorEPM `phase_joint` defaults to -1).
  * SUPPORT — the stance/swing gate, `foot_y > foot_y_ema`: the FOOT-HEIGHT cycle, driven
              by hip2+knee.

MotorEPM's own `legphase_agree` instrument already reads ~0.5 between them, which is chance.
If they are genuinely unlocked then each leg pushes backward without regard to whether its
foot is on the ground — roughly half the power stroke spent in the air and half the return
swing spent scrubbing the body backward.  That would explain BOTH the operator's "it is
always stumbling / it loses synchronization" AND the ledger's standing unknown that flat
speed is pinned at 0.03-0.05 across all eight timing levers ever tried: every one of those
adjusted the phase relationship BETWEEN legs while the relationship between thrust and
support WITHIN a leg stayed random.

THE HEADLINE is `td_plv` — the phase-locking value of the stroke waveform at TRUE touchdown.
Each contact onset contributes e^(i*theta) where theta is the stroke's phase; if touchdown
lands at a uniformly random point in the stroke the vectors cancel and PLV -> 0.

  td_plv ~ 0.0-0.15  => UNLOCKED. The clocks are independent; the diagnosis is confirmed.
  td_plv ~ 0.3-0.6   => partially locked (a beat, not independence).
  td_plv > 0.7       => locked, and `td_phase` says whether stroke_phase is merely offset.

It also answers the two questions that gate what comes next:

  tq_agree / tq_sep  — does `joint_torque` separate stance from swing?  At chance, a
                       load-gated stroke has nothing to gate on and dies here, for the cost
                       of one run instead of a build plus a seed-averaged A/B.
  explore_mult       — progress->commit already damps the coordination probe sigma.  If this
                       already sits at 0 on flat ground, a precision gate on the same sigma
                       would be a TAUTOLOGY (CLAUDE.md 3.2 rule 1).

Usage:  python3 gaitalign.py [config.json] [n_seeds] [max_steps] [difficulty] [K=V ...]
        default config = the_picrawler_motor_epm_embed_corridor_alignprobe.json

Logs go to $SEEDAVG_OUT or /tmp/xaq_seedavg (shared with seedavg.py).
"""
import json, math, os, subprocess, sys, statistics, concurrent.futures as cf

PROJ = "/home/xaqmusic/xaq-ai/godot_host/project"
SP = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_seedavg")
os.makedirs(SP, exist_ok=True)

DEFAULT_CFG = "the_picrawler_motor_epm_embed_corridor_alignprobe.json"

# The alignment numbers are cumulative RUN MEANS maintained inside MotorEPM, so the honest
# read is the LAST line of the run, not an average over lines.  The spawn transient is
# already excluded from the per-tick metrics below by WARMUP.
WARMUP_TICKS = int(os.environ.get("SEEDAVG_WARMUP", 900))

# Fields carried straight through from the last diag line (they are already run means).
LAST = ("td_plv", "sd_plv", "pos_stance", "pos_swing", "contact_duty",
        "tq_agree", "tq_stance", "tq_swing", "legphase_agree", "phase_agree",
        "swing_frac", "cruse_bias", "motor_tle")


def run_one(cfg, seed, max_steps, difficulty, extra):
    out = f"{SP}/ga_{os.path.splitext(cfg)[0]}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7500 + seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(max_steps))
    for kv in extra:
        k, _, v = kv.partition("=")
        env[k] = v
    with open(out, "w") as f:
        subprocess.run(["godot4", "--headless", "--fixed-fps", "60", "--quit-after", "4000000",
                        "--path", ".", "res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out)


def parse(path):
    last, n = None, 0
    em = []                      # explore_mult, sampled over the steady state
    zs = []
    for line in open(path):
        line = line.strip()
        if '"x":' not in line or not line.startswith("{"):
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        n += 1
        zs.append(d.get("z", 0.0))
        if d.get("t", 0) >= WARMUP_TICKS and "explore_mult" in d:
            em.append(d["explore_mult"])
        if "td_plv" in d:
            last = d
    if last is None or n < 5:
        return None
    r = {k: float(last.get(k, 0.0)) for k in LAST}
    r["explore_mult"] = statistics.mean(em) if em else float("nan")
    r["explore_mult_min"] = min(em) if em else float("nan")
    # tq_sep is threshold-free: a ratio of mean loads, so it does not inherit the
    # self-referential-threshold problem that tq_agree shares with the foot-height detector.
    r["tq_sep"] = r["tq_stance"] / (r["tq_swing"] + 1e-6) if r["tq_swing"] or r["tq_stance"] else 0.0
    r["net_z"] = zs[-1] - zs[0]
    for key, out in (("per_hip1", "p_hip1"), ("per_knee", "p_knee"),
                     ("per_foot", "p_foot"), ("per_con", "p_con")):
        v = [x for x in last.get(key, []) if x and x > 0]
        r[out] = statistics.mean(v) if v else 0.0
    gp = last.get("gait_phase", [])
    r["gait_phase"] = [round(float(x), 2) for x in gp] if gp else []
    return r


def ms(vals):
    vals = [v for v in vals if v == v]                       # drop NaN
    if not vals:
        return (float("nan"), float("nan"))
    return (statistics.mean(vals), statistics.pstdev(vals))


def row(label, vals, fmt="{:6.3f}", note=""):
    m, s = ms(vals)
    print(f"  {label:<16}" + fmt.format(m) + " ± " + fmt.format(s) + ("   " + note if note else ""))


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CFG
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    steps = int(sys.argv[3]) if len(sys.argv) > 3 else 12000
    diff = sys.argv[4] if len(sys.argv) > 4 else "0.3"
    extra = list(sys.argv[5:])
    seeds = list(range(1, n + 1))
    with cf.ThreadPoolExecutor(max_workers=min(8, n)) as ex:
        res = list(ex.map(lambda s: run_one(cfg, s, steps, diff, extra), seeds))
    ok = [r for r in res if r]
    print(f"\n{cfg}  (n={len(ok)}/{n} seeds, {steps} ticks, diff {diff})")
    if not ok:
        print("  no valid runs — is gait_align_diag set, and did the run reach the diag block?")
        sys.exit(1)

    print("\n  ── THRUST vs SUPPORT: are they the same clock? ───────────────────")
    row("td_plv", [r["td_plv"] for r in ok], note="stroke phase at TRUE touchdown; ~0 = UNLOCKED")
    row("sd_plv", [r["sd_plv"] for r in ok], note="same vs the incumbent swing detector")
    row("pos_stance", [r["pos_stance"] for r in ok], note="frac of STANCE in the stroke's + half")
    row("pos_swing", [r["pos_swing"] for r in ok], note="same over SWING; both ~0.5 = no relation")
    row("legphase_agree", [r["legphase_agree"] for r in ok], note="leg phase vs the stance gate")
    row("phase_agree", [r["phase_agree"] for r in ok], note="global body phase vs the stance gate")

    print("\n  ── THE THREE CLOCKS (period, ticks) ──────────────────────────────")
    row("hip1 (stride)", [r["p_hip1"] for r in ok], "{:6.1f}")
    row("knee (stroke)", [r["p_knee"] for r in ok], "{:6.1f}", "the phase the stroke rides")
    row("foot (detector)", [r["p_foot"] for r in ok], "{:6.1f}", "the phase the stance gate rides")
    row("contact (REAL)", [r["p_con"] for r in ok], "{:6.1f}", "true step period; << foot => detector chatter")

    print("\n  ── LOAD: can joint_torque separate stance from swing? ────────────")
    row("tq_agree", [r["tq_agree"] for r in ok], note="0.5 = chance = no load lever is possible")
    row("tq_sep", [r["tq_sep"] for r in ok], note="mean stance load / mean swing load (>1 wanted)")
    row("tq_stance", [r["tq_stance"] for r in ok], "{:6.4f}")
    row("tq_swing", [r["tq_swing"] for r in ok], "{:6.4f}")

    print("\n  ── CONTEXT / TAUTOLOGY CHECKS ────────────────────────────────────")
    row("explore_mult", [r["explore_mult"] for r in ok], note="probe damping; 0 => precision gate is a no-op")
    row("explore_mult_min", [r["explore_mult_min"] for r in ok])
    row("contact_duty", [r["contact_duty"] for r in ok], note="TRUE stance duty (ledger: ~0.77)")
    row("swing_frac", [r["swing_frac"] for r in ok], note="what the incumbent detector reports")
    row("cruse_bias", [r["cruse_bias"] for r in ok], "{:6.4f}", "EXACTLY 0 => MotorEPM's Cruse never ran")
    row("motor_tle", [r["motor_tle"] for r in ok], "{:6.4f}")
    row("net_z", [r["net_z"] for r in ok], "{:6.2f}", "sanity: the body still walked")
    for r, s in zip(ok, seeds):
        print(f"    seed {s}: gait_phase = {r['gait_phase']}   (imposed trot = [0.0, 3.14, 3.14, 0.0])")
