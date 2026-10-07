#!/usr/bin/env bash
# Launch the static voxel viewer — the duck's sweep clouds in an interactive 3D view.
#
# Mirrors tools/run_inspector.sh: puts tools/ on PYTHONPATH so `xaq_inspector.voxel_viewer` resolves
# as a plain directory package, and pins the repo venv (PyQt6 + pyqtgraph + PyOpenGL, see
# tools/xaq_inspector/requirements.txt) rather than whatever environment happens to be active.
#
# Usage:
#   tools/run_voxel_viewer.sh RUN.jsonl                      # every filed cloud, laid out in the room
#   tools/run_voxel_viewer.sh RUN.jsonl --place 8            # one place, alone
#   tools/run_voxel_viewer.sh RUN.jsonl --frame body --colour hits
#   tools/run_voxel_viewer.sh RUN.jsonl --screenshot out.png # render once and exit (needs a display)
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(dirname "$here")"

export PYTHONPATH="$here${PYTHONPATH:+:$PYTHONPATH}"

py="python3"
if [[ -x "$repo_root/.venv/bin/python3" ]]; then
  py="$repo_root/.venv/bin/python3"
fi

exec "$py" -m xaq_inspector.voxel_viewer "$@"
