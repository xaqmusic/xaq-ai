#!/usr/bin/env bash
# The trained Pollen policies mj_host uses as scaffolds (a prop, named as one — see
# models/microduck/scaffolds/README.md).  Not tracked in git (*.onnx is ignored): fetched
# by pinned upstream commit and verified by SHA-256.  A local clone (MICRODUCK_CLONE, default
# ~/microduck) is used when present, GitHub otherwise.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dst="$here/../models/microduck/scaffolds"
fetch() {
  local name="$1" commit="$2" sha="$3" out="$dst/$1"
  if [[ -f "$out" ]] && echo "$sha  $out" | sha256sum -c --quiet 2>/dev/null; then echo "  ok       $name"; return; fi
  local src="${MICRODUCK_CLONE:-$HOME/microduck}/policies/$name"
  if [[ -f "$src" ]]; then cp "$src" "$out"; else
    curl -fsSL -o "$out" "https://raw.githubusercontent.com/pollen-robotics/microduck/$commit/policies/$name"; fi
  echo "$sha  $out" | sha256sum -c --quiet && echo "  fetched  $name ($commit)"
}
fetch alpha_stand.onnx   590b986 1569268713e40deea795dd2922dba50d3621e15a872855408b6b1b125b1c094b
fetch alpha_walking.onnx 3954496 e36332d383997d51401897734cd3e79cf5038406feddb18b4d57ecfb141daa6c
