#!/usr/bin/env bash
# =============================================================================
# run.sh — the Microduck host, from one place
# =============================================================================
#
# Everything the operator needs, in the order it is usually wanted:
#
#   ./mj_host/run.sh build              build (and fetch MuJoCo / ONNX on first run)
#   ./mj_host/run.sh gates              every gate that can be checked, exit non-zero on any failure
#   ./mj_host/run.sh watch [args]       WATCH IT LIVE — spawns the host, opens a window
#   ./mj_host/run.sh stub  [args]       watch the recovery harness: a brain that falls
#                                       over, and the scaffold that picks it up
#   ./mj_host/run.sh brain [args]       watch phase A1: MotorEPM driving the joints
#   ./mj_host/run.sh record out.mp4 [args]   render a run to video (no window needed)
#   ./mj_host/run.sh hold [args]        headless, JSONL to log/
#
#
# `record` takes --stub as its first host arg to drive the harness instead of --hold.
#
# `args` are the host's: --secs N --noise R --seed N, plus a scene path.
# Examples:
#   ./mj_host/run.sh watch --secs 30
#   ./mj_host/run.sh watch --secs 10 --noise 0.05 --seed 2
#   ./mj_host/run.sh record /tmp/duck.mp4 --secs 8 --noise 0.05
#   ./mj_host/run.sh stub --secs 60 --stub-amp 0
#   ./mj_host/run.sh record /tmp/stub.mp4 --stub --secs 20
#
# The viewer never simulates: it draws the qpos the host wrote. What is on screen
# is the run, not a re-derivation of it.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
HOST="$HERE/build/ogma_mjhost"
VIEW="$REPO/tools/duck_viewer/view.py"
VENV="$REPO/tools/duck_viewer/.venv/bin/python"
LOGDIR="$HERE/log"

build() {
  cmake -S "$HERE" -B "$HERE/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "$HERE/build" -j"$(nproc)"
}

need_host() {
  [[ -x "$HOST" ]] || { echo "host not built — running build first"; build; }
}

need_viewer() {
  [[ -x "$VENV" ]] || { echo "viewer venv missing — creating it"; "$REPO/tools/duck_viewer/setup.sh"; }
}

case "${1:-}" in
  build)
    build
    ;;

  gates)
    need_host
    echo "=== G1 / G3 / G4 · scene.xml ==="
    "$HOST" --load-only
    echo
    echo "=== G1 / G3 / G4 · scene_walk.xml ==="
    "$HOST" --load-only "$HERE/models/microduck/scene_walk.xml" | tail -3
    echo
    echo "=== G2 · the settle sweep ==="
    "$HOST" --gate-g2
    echo
    echo "ALL GATES PASS"
    ;;

  watch)
    shift
    need_host; need_viewer
    mkdir -p "$LOGDIR"
    run="$LOGDIR/watch-$(date +%Y%m%d-%H%M%S).jsonl"
    "$VENV" "$VIEW" live --save "$run" -- "$@"
    ;;

  brain)
    shift
    need_host; need_viewer
    mkdir -p "$LOGDIR"
    run="$LOGDIR/brain-$(date +%Y%m%d-%H%M%S).jsonl"
    "$VENV" "$VIEW" live --save "$run" --host-mode=--brain -- "$@"
    ;;

  stub)
    shift
    need_host; need_viewer
    mkdir -p "$LOGDIR"
    run="$LOGDIR/stub-$(date +%Y%m%d-%H%M%S).jsonl"
    "$VENV" "$VIEW" live --save "$run" --host-mode=--stub -- "$@"
    ;;

  record)
    shift
    out="${1:?usage: run.sh record OUT.mp4 [host args]}"; shift || true
    mode=--hold
    if [[ "${1:-}" == "--stub" || "${1:-}" == "--brain" ]]; then mode="$1"; shift; fi
    need_host; need_viewer
    "$HOST" "$mode" "$@" | "$VENV" "$VIEW" record - "$out"
    ;;

  hold)
    shift
    need_host
    mkdir -p "$LOGDIR"
    run="$LOGDIR/hold-$(date +%Y%m%d-%H%M%S).jsonl"
    "$HOST" --hold "$@" > "$run"
    echo "  kept $run  ($(wc -l < "$run") ticks)"
    ;;

  *)
    sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's|^# \?||'
    exit 2
    ;;
esac
