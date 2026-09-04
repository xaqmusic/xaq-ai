#!/usr/bin/env bash
# =============================================================================
# run.sh — the brain builder, from one place
# =============================================================================
#
#   ./brain_builder/run.sh build                 configure + build (fetches ImGui, GLFW, node-editor on first run)
#   ./brain_builder/run.sh open [config.json]    open the builder (empty, or on a config)
#   ./brain_builder/run.sh catalogue             dump the module catalogue as JSON (70 types, params, sockets)
#   ./brain_builder/run.sh types                 list the registered module types
#   ./brain_builder/run.sh gen-palette           re-derive sockets by probing and merge them into palette.json
#   ./brain_builder/run.sh tests                 unit tests
#
# The builder never runs a brain against a body: it edits the GraphConfig JSON
# that mj_host and the Godot host run unchanged.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$HERE/build/brain_builder"

build() {
  cmake -S "$HERE" -B "$HERE/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "$HERE/build" -j"$(nproc)"
}

need_bin() { [[ -x "$BIN" ]] || { echo "builder not built — running build first" >&2; build; }; }

case "${1:-}" in
  build)       build ;;
  open)        shift; need_bin; exec "$BIN" "$@" ;;
  catalogue)   shift; need_bin; exec "$BIN" --dump-catalogue "$@" ;;
  types)       need_bin; exec "$BIN" --list-types ;;
  gen-palette) shift; need_bin; exec "$BIN" --gen-palette --merge "$@" ;;
  tests)       need_bin; (cd "$HERE/build" && ctest --output-on-failure) ;;
  *)
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
    exit 1 ;;
esac
