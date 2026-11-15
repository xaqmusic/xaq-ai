#!/bin/bash
# stepper_resume_ab.sh — a paired A/B from carried learned state (learned stepper plan Phase 3+): for each chain's saved state, one run of
# config A and one of config B, resumed from the SAME state, on the same bench and world seed. The fair comparison of a learned module's
# behaviour under a change (Phase 3: the stance belief with and without injected FSR flicker; Phase 4: a frozen-model evaluation).
#   usage: stepper_resume_ab.sh <chain dir> <run k> <config A> <config B> <arm label> [bench (b38|b64|cor)] [steps (9000)]
#   reads <chain dir>/state_c<c>_r<k>.json for every chain c present; writes <out>/A/<arm>A_<bench>_c<c>.jsonl.gz and B likewise, out =
#   <chain dir>/ab_<arm>. Gates per run: the seed echoed, the row count, the resume restored every module in the state file.
#   env: PORT_BASE (8300), EXPECT_B ("module.key=value ..." asserted on the B runs through stepper_expect.py; OGMA_DUMP_PARAMS is on)
set -u
CH=$1; K=$2; CA=$3; CB=$4; ARM=$5; BENCH=${6:-b38}; STEPS=${7:-9000}
TOOLS=/home/xaqmusic/xaq-ai/godot_host/project/scripts_tools; OUT=$CH/ab_$ARM; mkdir -p $OUT/A $OUT/B
one() {   # chain, side, config
  c=$1; side=$2; cfg=$3; st=$CH/state_c${c}_r$K.json; seed=$(( 900 + c )); f=$OUT/$side/${ARM}${side}_${BENCH}_c$c.jsonl.gz
  off=0; [ $side = B ] && off=1; port=$(( PORT_BASE + 2 * c + off ))
  ev=(env -u OGMA_PICRAWLER_STAIRS_H_MM -u OGMA_PICRAWLER_STAIRS_N -u OGMA_PICRAWLER_STAIRS_START_M -u OGMA_PICRAWLER_SAVE_STATE_AT_EXIT
      OGMA_PICRAWLER_LOAD_LOG=1 OGMA_PICRAWLER_DIAG_INTERVAL=5 OGMA_DUMP_PARAMS=1 OGMA_SEED=$seed STEPS=$STEPS OGMA_PICRAWLER_RESUME_STATE=$st)
  if [ $BENCH = cor ]; then ev+=(OGMA_PICRAWLER_GYM=corridor); else ev+=(OGMA_PICRAWLER_GYM=stairs OGMA_PICRAWLER_STAIRS_H_MM=${BENCH#b} OGMA_PICRAWLER_STAIRS_N=3 OGMA_PICRAWLER_STAIRS_START_M=0.3); fi
  "${ev[@]}" $TOOLS/run_gain0.sh $cfg $port $f > /dev/null
  got=$(zcat $f | grep -o '"event":"WORLD_READY"[^}]*"seed":"[0-9]*"' | grep -o '[0-9]*"$' | tr -d '"'); rows=$(zcat $f | grep -c '"fwd_v"'); ok=OK
  [ "$got" = "$seed" ] || ok="SEED MISMATCH"
  [ $rows -ge $(( STEPS / 5 - 2 )) ] || ok="$ok SHORT ($rows)"
  n=$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))['snapshots']))" $st)
  zcat $f | grep -q "brain state load — restored $n modules" || ok="$ok RESUME FAILED"
  if [ $side = B ] && [ -n "${EXPECT_B:-}" ]; then ex=$(python3 $TOOLS/stepper_expect.py $f $EXPECT_B) || ok="$ok $ex"; fi
  echo "c$c $side $cfg seed $seed rows $rows $ok"
}
export -f one; export CH K ARM BENCH STEPS TOOLS OUT EXPECT_B="${EXPECT_B:-}" PORT_BASE=${PORT_BASE:-8300}
for st in $CH/state_c*_r$K.json; do c=$(basename $st | sed 's/state_c\([0-9]*\)_.*/\1/'); echo "$c A $CA"; echo "$c B $CB"; done \
  | xargs -P 16 -L1 bash -c 'one "$0" "$1" "$2"' | sort -V | tee $OUT/runs.txt
bad=$(grep -vc ' OK$' $OUT/runs.txt); echo "resume A/B $ARM: $(wc -l < $OUT/runs.txt) runs, $bad failed a gate -> $OUT"
[ "$bad" = 0 ]
