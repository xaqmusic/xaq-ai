#!/usr/bin/env bash
# scan-identifiers.sh — refuse to publish real network identifiers.
#
# WHY THIS EXISTS.  On 2026-09-05 a real SSID and two router BSSIDs reached a public
# repo through `docs/plans-and-designs/picrawler_sim2real_port.md` and had to be removed
# by rewriting history.  They were never in code or config -- nothing here has ever
# needed them -- so no amount of config hygiene would have caught it.  They arrived in
# PROSE, as the evidence in a debugging narrative, which is exactly where a writer will
# put them again.  This scans content, so it does not care whether the value came from a
# source file, a doc, or a commit message.
#
# TWO KINDS OF SECRET, TWO MECHANISMS:
#   - Shaped ones (MAC/BSSID) match a regex and need no local state, so this protects
#     every contributor from a fresh clone.
#   - Unshaped ones (an SSID is an arbitrary string) cannot be pattern-matched, so they
#     live in .secrets-local -- which is GITIGNORED, because a denylist committed to the
#     repo publishes the very strings it exists to protect.
#
# Usage: scan-identifiers.sh <<< "$text"      (reads the text to scan on stdin)
# Exit 0 = clean, 1 = hits found (printed to stderr).
set -uo pipefail
repo_root="$(git rev-parse --show-toplevel)"
text="$(cat)"
hits=0

emit() { printf '  %s\n' "$1" >&2; hits=1; }

# ---- 1. MAC / BSSID shaped strings -----------------------------------------------
# Documented placeholders are the ALLOWED form and must not trip the hook; anything
# else with this shape is assumed real, because a made-up MAC in prose is a placeholder
# that should have used the documented one.
mac_hits="$(printf '%s\n' "$text" \
  | grep -nIE '\b([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}\b' \
  | grep -viE 'AA:BB:CC:DD:EE:[0-9A-F]{2}|00:00:00:00:00:00|FF:FF:FF:FF:FF:FF|DE:AD:BE:EF:[0-9A-F]{2}:[0-9A-F]{2}' \
  || true)"
if [ -n "$mac_hits" ]; then
    echo "MAC/BSSID-shaped strings (use AA:BB:CC:DD:EE:F0, and see REPORTS.md):" >&2
    while IFS= read -r l; do emit "$l"; done <<< "$mac_hits"
fi

# ---- 2. Local denylist -------------------------------------------------------------
deny="$repo_root/.secrets-local"
if [ -f "$deny" ]; then
    while IFS= read -r term; do
        [ -z "$term" ] && continue
        case "$term" in \#*) continue ;; esac
        found="$(printf '%s\n' "$text" | grep -nIiF -- "$term" || true)"
        if [ -n "$found" ]; then
            echo "matches .secrets-local entry (replace with a <placeholder>):" >&2
            while IFS= read -r l; do emit "$l"; done <<< "$found"
        fi
    done < "$deny"
fi

exit $hits
