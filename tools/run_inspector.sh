#!/usr/bin/env bash
# run_inspector.sh — launch the xaq inspector (sidecar UI for the live C++ brain).
#
# Connects to a running Godot host's ControlServer (TCP OGMA_INSPECTOR_PORT,
# default 7400) and the ZMQ DiagPublisher (control+1, default 7401).  Start the
# Godot project FIRST, then run this.
#
# Usage:
#   tools/run_inspector.sh                          # defaults to 127.0.0.1:7400/7401
#   tools/run_inspector.sh --control-port 7400 \
#                          --diag-port    7401 \
#                          --diag-host    127.0.0.1
#
# Requires an environment with PyQt6 + pyqtgraph + pyzmq + numpy
# (see tools/xaq_inspector/requirements.txt; pyzmq is also an xaq_core dep).
# By default that's the repo-root .venv/ (see AGENTS.md) — this script uses it
# directly below so it works regardless of what's active in your shell (e.g. a
# conda `base` env auto-activated in a fresh terminal), not just when you
# remember to `source .venv/bin/activate` first.
set -euo pipefail

# This script lives in tools/, which is the directory that CONTAINS the
# xaq_inspector package — put it on sys.path so `python -m xaq_inspector`
# resolves regardless of the caller's CWD.
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$here/.." && pwd)"
export PYTHONPATH="$here${PYTHONPATH:+:$PYTHONPATH}"

py="python3"
if [ -x "$repo_root/.venv/bin/python3" ]; then
    py="$repo_root/.venv/bin/python3"
fi

exec "$py" -m xaq_inspector "$@"
