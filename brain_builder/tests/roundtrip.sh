#!/usr/bin/env bash
# Round-trip acceptance: load → save through the document model must keep
# every key and value (semantic identity, metadata.builder excluded) and the
# module order; byte identity is reported, not required.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
BIN="$HERE/../build/brain_builder"
OUT="${TMPDIR:-/tmp}/bb_roundtrip.$$"
mkdir -p "$OUT"
G="$REPO/godot_host/project/addons/ami_ogma/configs"
FILES=(
  "$REPO/mj_host/configs/a1v2_r19_settle_each.json"
  "$REPO/mj_host/configs/a1v2_r25_l2_lookaround.json"
  "$G/the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__native_cad.json"
  "$G/the_picrawler_motor_epm_rung0.json"
)
fail=0
for f in "${FILES[@]}"; do
  o="$OUT/$(basename "$f")"
  "$BIN" --roundtrip "$f" "$o" >/dev/null || { echo "FAIL  $(basename "$f"): roundtrip errored"; fail=1; continue; }
  python3 - "$f" "$o" <<'PY' || fail=1
import json, sys
a, b = [json.load(open(p)) for p in sys.argv[1:3]]
for d in (a, b):
    d.get("metadata", {}).pop("builder", None)
name = sys.argv[1].rsplit("/", 1)[-1]
same = json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)
order = [m["id"] for m in a["modules"]] == [m["id"] for m in b["modules"]]
bytes_same = open(sys.argv[1], "rb").read() == open(sys.argv[2], "rb").read()
print(("ok    " if same and order else "FAIL  ") + name + (" (byte-identical)" if bytes_same else " (semantic identity; bytes differ)"))
sys.exit(0 if same and order else 1)
PY
done
rm -rf "$OUT"
exit $fail
