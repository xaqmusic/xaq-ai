#!/usr/bin/env bash
# xaq inspector launcher — sidecar UI for the live brain.
#
# Connects to a running Godot host's ControlServer (TCP 7400) and ZMQ
# DiagPublisher (7401).  Run this AFTER the Godot project is up.
#
# Usage:
#   ./run.sh                                  # defaults to localhost:7400/7401
#   ./run.sh --control-port 7400 \
#            --diag-port    7401 \
#            --diag-host    127.0.0.1
#
# Requires an environment with PyQt6 + pyqtgraph + pyzmq + numpy
# (declared in requirements.txt; pyzmq is also an xaq_core dependency).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Put tools/ (the directory containing the xaq_inspector package) on
# sys.path so `python -m xaq_inspector` resolves regardless of CWD.
export PYTHONPATH="$(dirname "$here")${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m xaq_inspector "$@"
