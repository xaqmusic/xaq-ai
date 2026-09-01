#!/bin/sh
# Install/refresh the pi_host systemd units.  Run ON THE PI, from the repo root.
#   sudo pi_host/systemd/install.sh [unit ...]      (default: all of them)
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
UNITS=${*:-"ogma-benchd.service ogma-host.service"}
for u in $UNITS; do
    install -m 644 "$DIR/$u" "/etc/systemd/system/$u"
    echo "installed $u"
done
mkdir -p /home/xaqmusic/xaq-ai/pi_host/log
systemctl daemon-reload
for u in $UNITS; do systemctl enable "$u"; done
echo "Now: systemctl start <unit>   (enable only arms it for the next boot)"
