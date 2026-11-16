#!/bin/bash
# stepper_chain.sh — learned modules carried across runs (learned stepper plan Phase 3: persistence). C chains in parallel, R runs each;
# after every run the host's saved brain state (OGMA_PICRAWLER_SAVE_STATE_AT_EXIT) is FILTERED to the modules named in KEEP and the next
# run resumes from it (OGMA_PICRAWLER_RESUME_STATE). Everything else — TerrainMemory's map of another world, the stepper, the other EPMs —
# starts fresh each run, exactly as in stepper_battery.sh.
#   usage: stepper_chain.sh <config basename> <arm label> [out dir]
#   env: CHAINS (6), RUNS (8), STEPS (9000), KEEP ("stance_features epm_stance belief_stance"), PORT_BASE (8200), NOBUILD=1,
#        EXPECT ("module.key=value ..." asserted on every run through stepper_expect.py; OGMA_DUMP_PARAMS is then on),
#        SCHEDULE ("cor arena cor ... b38 b64 ...": the bench of each run in order, RUNS = its length; a CURRICULUM -- build a
#        vocabulary in known worlds, then meet the stairs as a novel one. Unset = the cycle below),
#        FROM (a chain dir) + FROM_RUN (k): every chain c STARTS from FROM/state_c<c>_r<k>.json (copied in as state_c<c>_r0.json) and
#        its seeds continue FROM's numbering (10 c + FROM_RUN + k), so the same bench after the same state reproduces FROM's run exactly
#   run k of chain c: bench (b38, b64, cor)[(c + k) % 3] (or SCHEDULE's k-th; benches b<mm> = one riser, cor, arena), world seed 10 c + k; logs in <out>/r<k>/<arm>_<bench>_c<c>.jsonl.gz, the carried
#   state in <out>/state_c<c>_r<k>.json. Gates per run: the seed echoed, the row count, and (k > 1) the resume restored exactly the
#   KEEP modules — a silent cold start would read as "no learning".
#   Evaluate: stepper_belief.py <out>/r1 (cold) against stepper_belief.py <out>/r<R> (after R - 1 carried runs).
set -u
CFG=$1; ARM=$2; OUT=${3:-$HOME/xaq_runs/stepper/$ARM}
ROOT=/home/xaqmusic/xaq-ai; TOOLS=$ROOT/godot_host/project/scripts_tools; SAVED=$HOME/.local/share/godot/app_userdata/xaq-ai/saved_states
KEEP=${KEEP:-stance_features epm_stance belief_stance}; CHAINS=${CHAINS:-6}; RUNS=${RUNS:-8}; STEPS=${STEPS:-9000}; SCHEDULE=${SCHEDULE:-}
[ -n "$SCHEDULE" ] && RUNS=$(echo $SCHEDULE | wc -w)
FROM=${FROM:-}; FROM_RUN=${FROM_RUN:-0}
if [ -n "$FROM" ]; then for c in $(seq 1 $CHAINS); do [ -f $FROM/state_c${c}_r$FROM_RUN.json ] || { echo "FROM GATE: no $FROM/state_c${c}_r$FROM_RUN.json"; exit 4; }; done; fi
mkdir -p $OUT
if [ -z "${NOBUILD:-}" ]; then
  cmake --build $ROOT/godot_host/build --target ami_ogma_host -j8 > $OUT/build.log 2>&1
  ne=$(grep -c ' error' $OUT/build.log); [ "$ne" = 0 ] || { echo "BUILD GATE: $ne errors (see $OUT/build.log)"; exit 3; }
fi
echo "config $CFG arm $ARM commit $(git -C $ROOT rev-parse --short HEAD) chains $CHAINS runs $RUNS steps $STEPS keep [$KEEP] expect [${EXPECT:-}] schedule [${SCHEDULE:-cycle}] from [${FROM:-none} r$FROM_RUN]" > $OUT/manifest.txt
chain() {
  c=$1; port=$(( PORT_BASE + c )); prev=""
  if [ -n "$FROM" ]; then cp $FROM/state_c${c}_r$FROM_RUN.json $OUT/state_c${c}_r0.json; prev=$OUT/state_c${c}_r0.json; fi
  for k in $(seq 1 $RUNS); do
    if [ -n "$SCHEDULE" ]; then b=$(echo $SCHEDULE | cut -d' ' -f$k); else b=$(echo b38 b64 cor | cut -d' ' -f$(( (c + k) % 3 + 1 ))); fi
    seed=$(( 10 * c + FROM_RUN + k )); d=$OUT/r$k; mkdir -p $d
    f=$d/${ARM}_${b}_c$c.jsonl.gz; name=${ARM}_c${c}_r$k
    ev=(env -u OGMA_PICRAWLER_STAIRS_H_MM -u OGMA_PICRAWLER_STAIRS_N -u OGMA_PICRAWLER_STAIRS_START_M -u OGMA_PICRAWLER_RESUME_STATE
        OGMA_PICRAWLER_LOAD_LOG=1 OGMA_PICRAWLER_DIAG_INTERVAL=5 OGMA_SEED=$seed STEPS=$STEPS OGMA_PICRAWLER_SAVE_STATE_AT_EXIT=$name)
    [ -n "${EXPECT:-}" ] && ev+=(OGMA_DUMP_PARAMS=1)
    if [ $b = cor ]; then ev+=(OGMA_PICRAWLER_GYM=corridor); elif [ $b = arena ]; then ev+=(OGMA_PICRAWLER_GYM=arena); else ev+=(OGMA_PICRAWLER_GYM=stairs OGMA_PICRAWLER_STAIRS_H_MM=${b#b} OGMA_PICRAWLER_STAIRS_N=3 OGMA_PICRAWLER_STAIRS_START_M=0.3); fi
    [ -n "$prev" ] && ev+=(OGMA_PICRAWLER_RESUME_STATE=$prev)
    "${ev[@]}" $TOOLS/run_gain0.sh $CFG $port $f > /dev/null
    got=$(zcat $f | grep -o '"event":"WORLD_READY"[^}]*"seed":"[0-9]*"' | grep -o '[0-9]*"$' | tr -d '"')
    rows=$(zcat $f | grep -c '"fwd_v"'); ok=OK
    [ "$got" = "$seed" ] || ok="SEED MISMATCH (set $seed, world $got)"
    [ $rows -ge $(( STEPS / 5 - 2 )) ] || ok="$ok SHORT ($rows rows)"
    if [ -n "${EXPECT:-}" ]; then ex=$(python3 $TOOLS/stepper_expect.py $f $EXPECT) || ok="$ok $ex"; fi
    if [ -n "$prev" ]; then
      res=$(zcat $f | grep -o 'brain state load — restored [0-9]* modules[^;]*' | head -1)
      echo "$res" | grep -q "restored $(echo $KEEP | wc -w) modules" || ok="$ok RESUME FAILED ($res)"
    fi
    full=$SAVED/$name.brain_state.json; st=$OUT/state_c${c}_r$k.json
    if [ -f $full ] && python3 - "$full" "$st" $KEEP <<'EOF'
import json, sys
src, dst, keep = sys.argv[1], sys.argv[2], sys.argv[3:]
p = json.load(open(src)); snaps = p.get('snapshots', {})
missing = [k for k in keep if k not in snaps]
if missing: sys.exit(f'missing {missing}')
p['snapshots'] = {k: snaps[k] for k in keep}
json.dump(p, open(dst, 'w')); json.load(open(dst))   # written and read back before the full state is removed
EOF
    then rm -f $full; prev=$st; else ok="$ok STATE NOT SAVED"; prev=""; fi
    echo "c$c r$k $b seed $seed world ${got:-none} rows $rows $ok"
  done
}
export -f chain; export OUT ARM CFG TOOLS SAVED KEEP RUNS STEPS SCHEDULE FROM FROM_RUN EXPECT="${EXPECT:-}" PORT_BASE=${PORT_BASE:-8200}
seq 1 $CHAINS | xargs -P $CHAINS -I{} bash -c 'chain {}' | sort -V | tee $OUT/runs.txt
bad=$(grep -vc ' OK$' $OUT/runs.txt); echo "chain $ARM: $(wc -l < $OUT/runs.txt) runs, $bad failed a gate -> $OUT"
[ "$bad" = 0 ]
