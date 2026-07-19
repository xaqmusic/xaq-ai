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
set -euo pipefail

# This script lives in tools/, which is the directory that CONTAINS the
# xaq_inspector package — put it on sys.path so `python -m xaq_inspector`
# resolves regardless of the caller's CWD.
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$here${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m xaq_inspector "$@"
