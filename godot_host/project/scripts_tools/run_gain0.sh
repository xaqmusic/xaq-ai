#!/bin/bash
# run_gain0.sh — one headless picrawler run of a config, with the config's metadata.body_env applied (a var already set in the
# environment wins, as in seedavg.py).  usage: run_gain0.sh <config basename> <inspector port> <out.jsonl>
# env: STEPS (default 1500), OGMA_SEED (7), OGMA_PICRAWLER_GYM (the config's), OGMA_PICRAWLER_LOAD_LOG=1 OGMA_PICRAWLER_DIAG_INTERVAL=5 for
# the body log every 5 ticks, OGMA_PICRAWLER_STAIRS_H_MM / _N / _START_M for a bench (e.g. 38 3 0.3).  The sequencer work of 2026-11
# (ledger) was measured with it; score the outputs with bench_seq.py, or run the fixed battery with stepper_battery.sh.  Lived in a scratchpad until 2026-11-14 (a reboot wiped it).
cd /home/xaqmusic/xaq-ai/godot_host/project
CFG=addons/ami_ogma/configs/$1.json
eval "$(python3 -c "
import json,shlex; d=json.load(open('$CFG')); be=d['metadata'].get('body_env',{})
import os
print(' '.join('export %s=%s'%(k,shlex.quote(str(v))) for k,v in be.items() if k not in os.environ))   # a var already set wins, as in seedavg
print('export GYM=%s DIFF=%s'%(d['metadata'].get('gym_mode','corridor'), d['metadata'].get('gym_difficulty',0.3)))")"
GYM=${OGMA_PICRAWLER_GYM:-$GYM}
OGMA_SEED=${OGMA_SEED:-7} OGMA_INSPECTOR_PORT=$2 OGMA_PICRAWLER_GYM=$GYM OGMA_PICRAWLER_CONFIG=res://$CFG OGMA_RESET_MODE=continuous \
OGMA_PICRAWLER_MAX_STEPS=${STEPS:-1500} OGMA_PICRAWLER_GYM_DIFFICULTY=$DIFF \
godot4 --headless --fixed-fps 60 ${OGMA_TURBO:+--disable-render-loop} --quit-after 4000000 --path . res://scenes/the_picrawler.tscn 2>&1 \
  | if [[ $3 == *.gz ]]; then gzip -c > $3; else cat > $3; fi   # gzip on write when the name ends .gz (a full /tmp truncated a battery, 2026-11-14)
echo "done $1 $(zcat -f $3 | grep -c '"fwd_v"') diag lines"
