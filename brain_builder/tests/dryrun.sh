#!/usr/bin/env bash
# Dry-run acceptance: the reference duck configs construct, tick 50 times and
# drive the body's sinks; the publish plan for r19 names the next config.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
BIN="$HERE/../build/brain_builder"
fail=0
for f in a1v2_r19_settle_each.json a1v2_r25_l2_lookaround.json; do
  out="$("$BIN" --dry-run "$REPO/mj_host/configs/$f" --ticks 50 2>/dev/null)" || { echo "FAIL  $f: dry run failed"; echo "$out" | tail -3; fail=1; continue; }
  if echo "$out" | grep -q "NO VALUE"; then echo "FAIL  $f: a body sink received no value"; echo "$out" | grep "NO VALUE"; fail=1; else echo "ok    $f: $(echo "$out" | grep -c 'driven ') sinks driven"; fi
done
"$BIN" --publish-dry "$REPO/mj_host/configs/a1v2_r19_settle_each.json" --slug smoke --title "smoke test" 2>/dev/null || { echo "FAIL  publish-dry"; fail=1; }
exit $fail
