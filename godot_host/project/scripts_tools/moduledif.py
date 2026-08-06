#!/usr/bin/env python3
"""Are two module builds BYTE-IDENTICAL? — the gate MotorEPMv2 must pass before it means anything.

WHY THIS EXISTS.  MotorEPMv2 is built as a separate module so the deployed MotorEPM stays a
frozen benchmark (docs/plans-and-designs/motor_epm_v2_plan.md).  That only buys anything if
v2 with every new feature at 0 reproduces MotorEPM EXACTLY.  Not "close" — exactly.  Any
divergence at that point is a v2 BUG, never a finding, and a v2 that has silently drifted
would make every subsequent A/B compare two unknowns instead of one lever.

⚠ THIS TOOL SHIPPED BEFORE ANY v2 FEATURE.  That ordering is the point.  A copy verified
after the fact is a copy nobody checked.

WHAT IS COMPARED.  Not the motor command alone — the WHOLE BODY TRAJECTORY, tick by tick,
from the JSONL diagnostic stream: chassis x/y/z, fwd_v, every knee and foot, the heading,
the learning telemetry.  This is strictly stronger than diffing y[]: identical commands into
identical physics must yield identical bodies, so any drift anywhere surfaces here, and the
FIRST tick of divergence localises it in time.

Floating-point note: the comparison is EXACT (==) by default because both arms run the same
binary on the same seed through the same fixed-timestep physics — there is no legitimate
source of difference.  --tol exists only to characterise a failure that has already
happened, never to declare success.

Usage:
  moduledif.py <logA> <logB>            exact compare, report first divergence
  moduledif.py <logA> <logB> --tol 1e-6 characterise an existing divergence
"""
import json, sys

SKIP = {"t"}          # the tick index itself is the x-axis, not a payload field


def rows(path):
    out = []
    for line in open(path):
        if line.startswith("{") and '"fwd_v"' in line:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def flat(d, prefix=""):
    """Flatten one diagnostic row so list fields (knee[], feet_y[]) compare elementwise."""
    out = {}
    for k, v in d.items():
        key = prefix + k
        if key in SKIP:
            continue
        if isinstance(v, list):
            for i, e in enumerate(v):
                if isinstance(e, (int, float)):
                    out[f"{key}[{i}]"] = e
        elif isinstance(v, (int, float)):
            out[key] = v
    return out


def main():
    a_path, b_path = sys.argv[1], sys.argv[2]
    tol = 0.0
    if "--tol" in sys.argv:
        tol = float(sys.argv[sys.argv.index("--tol") + 1])

    A, B = rows(a_path), rows(b_path)
    if not A or not B:
        print(f"  FAIL: no diagnostic rows ({len(A)} vs {len(B)}) — did a run crash?")
        return 2
    if len(A) != len(B):
        print(f"  ⚠ row-count mismatch: {len(A)} vs {len(B)} (comparing the common prefix)")

    n = min(len(A), len(B))
    first_bad, worst, worst_field = None, 0.0, None
    bad_fields = {}

    for i in range(n):
        fa, fb = flat(A[i]), flat(B[i])
        keys = set(fa) & set(fb)
        if i == 0:
            only_a, only_b = set(fa) - set(fb), set(fb) - set(fa)
            if only_a or only_b:
                print(f"  ⚠ field sets differ — only in A: {sorted(only_a)[:6]}"
                      f"  only in B: {sorted(only_b)[:6]}")
        for k in keys:
            d = abs(fa[k] - fb[k])
            if d > tol:
                if first_bad is None:
                    first_bad = (A[i].get("t"), k, fa[k], fb[k])
                bad_fields[k] = max(bad_fields.get(k, 0.0), d)
                if d > worst:
                    worst, worst_field = d, k

    print(f"\n  A: {a_path}\n  B: {b_path}")
    print(f"  compared {n} diagnostic rows x {len(flat(A[0]))} fields, tolerance {tol:g}")
    if first_bad is None:
        print("\n  ★ BYTE-IDENTICAL — every field matches on every tick.")
        print("    The v2 copy is verified.  Any future divergence is attributable to a")
        print("    single added lever, which is the whole point of the separate module.")
        return 0

    t, k, va, vb = first_bad
    print(f"\n  ✗ DIVERGENT — this is a v2 BUG, not a finding.")
    print(f"    first divergence: tick {t}, field '{k}'   A={va!r}  B={vb!r}")
    print(f"    worst: '{worst_field}' delta {worst:g}")
    print(f"    {len(bad_fields)} field(s) differ; top offenders:")
    for k2, d in sorted(bad_fields.items(), key=lambda kv: -kv[1])[:8]:
        print(f"      {k2:<24} max delta {d:g}")
    print("\n    Fix v2 until this prints BYTE-IDENTICAL before running any A/B.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
