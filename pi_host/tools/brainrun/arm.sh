#!/bin/bash
# arm.sh <tag> <lag_alpha> [seconds=30] [start_pose=rescue] -- ONE brain-driven run on the robot,
# run ON THE PI from anywhere (it cd's to ~/xaq-ai).  Stop ogma-host/ogma-benchd first
# (sudo systemctl stop ogma-host ogma-benchd) and restart them after.  Launch it DETACHED
# (setsid nohup bash pi_host/tools/brainrun/arm.sh ... &) so a dropped ssh cannot kill the
# tilt guard.  Records land in /tmp/ab/<tag>/: feed.jsonl (50 Hz state feed), run.jsonl (5 Hz
# telemetry + tilt guard), host.log (ogma_host + input dumps), benchd_record.jsonl.
# Battery only: on the bench supply the Pi reset the moment the brain took the servos.
# ⚠ Known protocol flaw: ogma_host ticks ~12 s in bench mode (commands not applied) before dev.
TAG=$1; ALPHA=$2; DUR=${3:-30}; POSE=${4:-rescue}
cd ~/xaq-ai
CFG=godot_host/project/addons/ami_ogma/configs/the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__native_measured__tofboom__fsrleg__honest__nohomeo.json
BV="python3 pi_host/tools/brainrun/bv.py"; ST="python3 pi_host/tools/brainrun/st.py"; CTL="python3 pi_host/tools/ogma_ctl.py"
O=/tmp/ab/$TAG; mkdir -p $O
echo "== ARM $TAG  lag alpha $ALPHA  ${DUR}s  start pose $POSE"
for p in $(pgrep -x ogma_host); do kill -TERM "$p"; done; sleep 2
for try in 1 2 3; do
  for p in $(pgrep -x ogma_benchd); do kill -TERM "$p"; done; sleep 2
  setsid nohup ./pi_host/build/ogma_benchd --body measured --state-pub 5592 --cmd-port 5594 --ctl-port 5593 --servo-lag-alpha $ALPHA > $O/benchd.log 2>&1 < /dev/null &
  sleep 5
  grep -aq "ICM-20948 SPI" $O/benchd.log && break
  echo "  benchd IMU probe failed (try $try): $(grep -a 'no ICM' $O/benchd.log)"
done
grep -aq "ICM-20948 SPI" $O/benchd.log || { echo "ABORT: benchd has no IMU"; exit 1; }
grep -aE "servo start|output lag|MODE" $O/benchd.log
( for i in $(seq 1 200); do $BV 5590 '{"verb":"ping"}' >/dev/null 2>&1; sleep 0.2; done ) &
PINGER=$!
US=$($BV 5590 '{"verb":"pose.get","name":"'$POSE'"}' | python3 -c "import json,sys; print(json.dumps(json.loads(sys.stdin.read())['us']))")
$BV 5590 "{\"verb\":\"pose.set\",\"us\":$US}" >/dev/null
sleep 3; for i in $(seq 1 40); do $ST | grep -q "moving=False" && break; sleep 0.5; done
setsid nohup ./pi_host/build/ogma_host --config $CFG --imu --brain-inputs --actuate tcp://127.0.0.1:5594 --dump-inputs 10 --listen 0.0.0.0 --rt > $O/host.log 2>&1 < /dev/null &
sleep 12
$CTL mode dev; kill $PINGER 2>/dev/null; sleep 1
python3 pi_host/tools/brainrun/feedrec.py $((DUR+4)) $O/feed.jsonl > $O/feedrec.out 2>&1 &
$CTL resume
python3 pi_host/tools/brainrun/runlog.py $DUR $O/run.jsonl
$CTL stop; sleep 2
$CTL mode bench; sleep 12
for p in $(pgrep -x ogma_host); do kill -TERM "$p"; done; sleep 3
cat $O/feedrec.out
grep -aE "IMU —|IMU sampler|brain inputs —|STOP —|actuation —" $O/host.log
L=$(ls -t pi_host/log/benchd_*.jsonl | head -1); cp "$L" $O/benchd_record.jsonl
grep -aE '"kind":"(resume|stop|rescue|deadman|vbat_dip|low_battery|brain_stream_lost|rail_undervolt)"' "$L" | cut -c1-160
echo "== ARM $TAG DONE"
