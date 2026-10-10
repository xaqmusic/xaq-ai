#!/usr/bin/env bash
# Local venv for the viewer.  Deliberately its own, not the repo's .venv: this
# needs mujoco and imageio, which nothing else in the repo does, and a viewer
# should never be able to break a build.
set -euo pipefail
cd "$(dirname "$0")"
python3 -m venv .venv
./.venv/bin/pip -q install --upgrade pip
./.venv/bin/pip -q install mujoco==3.12.0 "imageio[ffmpeg]"
echo "ready:  tools/duck_viewer/.venv"
./.venv/bin/python -c "import mujoco; print('  mujoco', mujoco.__version__)"
