#!/usr/bin/env bash
# Launch the xaq_voice studio.
#
# Mirrors tools/run_inspector.sh: puts tools/ on PYTHONPATH so both `xaq_voice_studio`
# and the `xaq_inspector.transport` module it reuses resolve as plain directory packages,
# and pins the repo venv rather than whatever environment happens to be active (a conda
# `base` auto-activating is the usual way this breaks).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(dirname "$here")"

export PYTHONPATH="$here${PYTHONPATH:+:$PYTHONPATH}"

py="python3"
if [[ -x "$repo_root/.venv/bin/python3" ]]; then
  py="$repo_root/.venv/bin/python3"
fi

exec "$py" -m xaq_voice_studio "$@"
