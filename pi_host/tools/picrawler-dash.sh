#!/bin/sh
# picrawler-dash — the dashboard's numbers on a terminal, for a monitor plugged into
# the robot or over ssh.  Defaults to localhost because it is meant to run ON the Pi.
#   picrawler-dash            live, refreshing        q quit   r refresh
#   picrawler-dash --once     one frame, for scripts
#   picrawler-dash --host X   watch a different robot
# Read-only: it polls stateless verbs and cannot move a servo or change a parameter.
#
# ⚠ Resolve $0 through symlinks.  install.sh puts a link in /usr/local/bin so a local
# login can just type the name, and a bare dirname "$0" would then look for the Python
# beside the LINK instead of beside the script.
SELF=$(readlink -f "$0" 2>/dev/null || echo "$0")
exec python3 "$(dirname "$SELF")/picrawler_dash.py" "$@"
