#!/bin/sh
# Commission the range EPM's input ranges on the bench.  Run ON THE PI from the repo root.
#
#   pi_host/tools/commission_range.sh [config]
#
# The EPM runs normally for the first `dim_autocal_ticks` INPUT FRAMES while accumulating
# per-dim statistics, then derives ranges, installs them, and RESETS the GNG so the
# vocabulary is re-earned in calibrated space.
#
# ⚠ SWEEP DURING THE WINDOW.  The ranges become whatever the sensor saw, so a run where
# nothing moved installs the width of the room's noise and clamps everything after.  The
# contract is explicit: "a full sweep for a scanning sensor... a range set by a startup
# transient is worse than the default."  Move a target smoothly through the NEAR and FAR
# limits you actually care about, twice, while the countdown runs.
set -e
CFG=${1:-godot_host/project/addons/ami_ogma/configs/picrawler_senses.json}
LOG=/tmp/commission_range.log
sudo systemctl stop ogma-host 2>/dev/null || true
sleep 1
setsid -f nohup ./pi_host/build/ogma_host --config "$CFG" --hz 50 --range --listen 0.0.0.0 \
    < /dev/null > "$LOG" 2>&1
echo "commissioning: SWEEP THE TARGET NOW — near to far and back, twice."
i=0
while [ $i -lt 30 ]; do
    D=$(python3 - <<'PY' 2>/dev/null || echo "?"
import socket, json
s = socket.create_connection(("127.0.0.1", 7400), 2)
s.sendall((json.dumps({"verb": "host_sensors"}) + "\n").encode())
b = b""
while not b.endswith(b"\n"):
    c = s.recv(65536)
    if not c:
        break
    b += c
print("%.1f" % json.loads(b.decode().strip().splitlines()[0])["range"]["cm"])
PY
)
    printf "\r  %2ds   range %8s cm   " "$i" "$D"
    if grep -q EPM_AUTOCAL "$LOG" 2>/dev/null; then break; fi
    i=$((i + 1))
    sleep 1
done
echo
echo
grep EPM_AUTOCAL "$LOG" | tail -1 || echo "  no EPM_AUTOCAL line — is dim_autocal_ticks set on epm_range?"
echo
echo "If those ranges cover the band you care about, write them into the config as"
echo "dim_min/dim_max and DELETE dim_autocal_ticks — otherwise every restart re-commissions"
echo "(and a service restarting unattended would commission on whatever it happened to see)."
pkill -x ogma_host 2>/dev/null || true
