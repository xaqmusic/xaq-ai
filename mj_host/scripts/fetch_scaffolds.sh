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
fetch ball_kick_left.onnx 3954496 d6928284dccd3dd61e08bf2f760effa74309fbefd97b2b31afb2a60f526d196a
fetch ball_kick_right.onnx 3954496 147a32c388c6b19111b3ac3b550a9a6dc8b8bf267118af4d8c3712522eedb5af
fetch roulade.onnx 3954496 3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b
