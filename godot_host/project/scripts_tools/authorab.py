#!/usr/bin/env python3
"""M1 rung (a) — did closing the loop improve the operating roll?

Compares two arms of the same build, matched by seed:
  APPLY   (author_apply=1): kept masks operate on the roll continuously.
  CONTROL (author_apply=0): prospector — same trials, keeps recorded only.

Both arms' body-log `plan` mirrors carry `je`/`jp` (per-depth × per-joint
cumulative mean |err| of the OPERATING decode vs hold-pose; 7 probes × 12
joints, row-major by depth).  The last mirrored line is the run's verdict.

Reported per matched seed:
  kept masks (apply arm) and their targets
  per-depth OPERATING-error ratio apply/control on the KEPT-TARGET joints and
  on all others (the null row: untargeted joints must sit ≈1.0), plus bands.
Seeds with an empty kept set are the built-in null: apply ≡ control there.

Usage: authorab.py '/tmp/xaq_m1a3/*.log' '/tmp/xaq_m1a3c/*.log'
"""
import glob, json, re, sys

PROBES = [1, 3, 5, 8, 13, 21, 34]
NJ = 12


def last_plan(path):
    plan = None
    for line in open(path):
        line = line.lstrip()
        if not line.startswith("{") or '"plan"' not in line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(d.get("plan"), dict) and "je" in d["plan"]:
            plan = d["plan"]
    return plan


def seed_of(path):
    m = re.search(r"_s(\d+)\.log$", path)
    return m.group(1) if m else path


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    apply_runs = {seed_of(f): last_plan(f) for f in sorted(glob.glob(argv[0]))}
    ctrl_runs = {seed_of(f): last_plan(f) for f in sorted(glob.glob(argv[1]))}
    seeds = sorted(set(apply_runs) & set(ctrl_runs))
    if not seeds:
        print("no matched seeds with je/jp mirrors")
        return 1
    print(f"authorab: {len(seeds)} matched seeds  (probes {PROBES})")
    for s in seeds:
        a, c = apply_runs[s], ctrl_runs[s]
        if not a or not c:
            print(f"  s{s}: missing je/jp mirror in one arm — skipped")
            continue
        au = a.get("au", {})
        kept = au.get("kept", [])
        targets = sorted({int(k["j"]) for k in kept})
        kmk = int(au.get("kmk", 0))
        print(f"\n  s{s}: kept={len(kept)} targets={targets} kept_suppressions={kmk}"
              f"  auth {int(c.get('auth', 0))}->{int(a.get('auth', 0))}")
        for k in kept:
            print(f"      j{k['j']} [{k['lo']:+.2f},{k['hi']:+.2f}] d[{k['dlo']},{k['dhi']}]"
                  f" r={k['r']:.3f} rt={k['rt']:.3f} tr={k['tr']}")
        if not kept:
            # built-in null: with nothing kept, apply must equal control
            drift = max(abs(float(x) - float(y)) for x, y in zip(a["je"], c["je"]))
            print(f"      no keeps — null check: max |je(apply)-je(ctrl)| = {drift:.4f}"
                  f"  ({'OK ≈ 0' if drift < 5e-4 else 'DRIFT — investigate'})")
            continue
        hdr = "      depth:" + "".join(f"{p:>8}" for p in PROBES)
        print(hdr)
        for label, joints in (("tgt", targets),
                              ("oth", [j for j in range(NJ) if j not in targets])):
            row = f"      {label} :"
            for i in range(len(PROBES)):
                na = sum(float(a["je"][i * NJ + j]) for j in joints)
                nc = sum(float(c["je"][i * NJ + j]) for j in joints)
                row += f"{(na / nc if nc > 1e-9 else float('nan')):>8.3f}"
            print(row)
        jb_a, jb_c = a.get("jband", []), c.get("jband", [])
        if len(jb_a) >= 2 * NJ and len(jb_c) >= 2 * NJ:
            wa = sum(hi - lo for lo, hi in zip(jb_a[::2], jb_a[1::2]) if hi > 0)
            wc = sum(hi - lo for lo, hi in zip(jb_c[::2], jb_c[1::2]) if hi > 0)
            print(f"      band width total (ticks): ctrl {wc:.0f} -> apply {wa:.0f}")
    print("\nGATE (rung a): on kept-target joints the apply/control operating-error ratio")
    print("must fall below 1 at the kept depths while 'oth' stays ≈1 and bands do not")
    print("shrink.  Untouched seeds must show apply ≡ control (the built-in null).")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
