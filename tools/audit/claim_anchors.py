#!/usr/bin/env python3
"""claim_anchors.py — keep the audit's claim register honest about its own anchors.

The register (docs/reports/cell_system_audit_2026-09_appendix.md) carries, per row, one or
more anchors of the form `path:pattern` — a repo-relative file and a literal substring (or
`/regex/`) that must occur in it.  This script greps every anchor; a missing file or an
absent pattern fails, so a row whose evidence has moved or been fixed rots LOUDLY instead
of quietly.  Run it from the repo root:

    python3 tools/audit/claim_anchors.py            # exit 1 on any broken anchor
    python3 tools/audit/claim_anchors.py --list     # print every anchor it found

Anchor syntax inside a markdown table cell (any number, separated by `;`):
    `cpp_core/src/ogma/modules/EFEArbiter.cpp:epistemic_reach_gated_`
    `godot_host/project/scripts_tools/cell_coverage.py:/run_one\\(.*42/`
Anchors are only read from cells wrapped in backticks so prose is never parsed.
"""
import argparse, pathlib, re, sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
REGISTER = ROOT / "docs/reports/cell_system_audit_2026-09_appendix.md"
ANCHOR = re.compile(r"`([A-Za-z0-9_./\-]+\.[A-Za-z0-9]+):((?:/[^`]+/)|[^`;]+)`")

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--register", default=str(REGISTER))
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    text = pathlib.Path(a.register).read_text()
    anchors = ANCHOR.findall(text)
    if not anchors:
        print(f"no anchors found in {a.register}"); return 1
    broken = 0
    cache: dict[str, str | None] = {}
    for path, pat in anchors:
        f = ROOT / path
        if path not in cache:
            cache[path] = f.read_text(errors="replace") if f.is_file() else None
        body = cache[path]
        if body is None:
            print(f"MISSING FILE  {path}   (pattern {pat!r})"); broken += 1; continue
        ok = re.search(pat[1:-1], body) if (pat.startswith("/") and pat.endswith("/")) else (pat in body)
        if a.list or not ok:
            print(f"{'ok      ' if ok else 'BROKEN  '} {path}:{pat}")
        broken += (not ok)
    print(f"{len(anchors)} anchors, {broken} broken")
    return 1 if broken else 0

if __name__ == "__main__":
    sys.exit(main())
