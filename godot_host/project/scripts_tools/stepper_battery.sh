#!/bin/bash
# stepper_battery.sh — the learned stepper plan's FIXED battery (docs/plans-and-designs/learned_stepper_plan.md §4 Phase 0), one arm.
#   usage: stepper_battery.sh <config basename> <arm label> [out dir]
#   15 runs, logs gzipped on write as <arm>_<bench>.jsonl.gz:
#     L42        the launcher layout: gym stairs, graded risers from 0.5 m, seed 42, 13 000 ticks
#     b38_1..6   one 38 mm riser x 3 treads from 0.3 m, seeds 1-6, 9 000 ticks
#     b64_1..6   the same at 64 mm
#     cor_1..2   the flat corridor (the config's difficulty), seeds 1-2, 9 000 ticks
#   env: OUT_ROOT (default ~/xaq_runs/stepper; NOT /tmp, a tmpfs that filled and truncated a battery on 2026-11-14), JOBS (15),
#        PORT_BASE (8100), NOBUILD=1 to skip the build gate, ONLY="L42 b38_1" to run a subset,
#        EXPECT="pose_sequencer.hang_ticks=0 ..." to assert in every run that the arm's levers landed (OGMA_DUMP_PARAMS).
#   Gates built in (each cost a battery when skipped): the host build must report 0 errors (a failed build leaves the OLD .so in
#   place and the arm runs it silently); every run's WORLD_READY seed must equal the seed set and its diag row count the ticks / 5.
#   Score with stepper_score.py <dir>; check identity with stepper_ident.py <golden dir> <dir>.
set -u
CFG=$1; ARM=$2; OUT=${3:-${OUT_ROOT:-$HOME/xaq_runs/stepper}/$ARM}
ROOT=/home/xaqmusic/xaq-ai; TOOLS=$ROOT/godot_host/project/scripts_tools
[ -f $ROOT/godot_host/project/addons/ami_ogma/configs/$CFG.json ] || { echo "no config $CFG"; exit 2; }
mkdir -p $OUT
if [ -z "${NOBUILD:-}" ]; then
  cmake --build $ROOT/godot_host/build --target ami_ogma_host -j8 > $OUT/build.log 2>&1
  ne=$(grep -c ' error' $OUT/build.log); [ "$ne" = 0 ] || { echo "BUILD GATE: $ne errors (see $OUT/build.log)"; exit 3; }
  echo "build gate: 0 errors"
fi
# the battery: name gym h_mm n start seed steps
BAT="L42 stairs graded - - 42 13000
$(for s in 1 2 3 4 5 6; do echo "b38_$s stairs 38 3 0.3 $s 9000"; done)
$(for s in 1 2 3 4 5 6; do echo "b64_$s stairs 64 3 0.3 $s 9000"; done)
$(for s in 1 2; do echo "cor_$s corridor - - - $s 9000"; done)"
[ -n "${ONLY:-}" ] && BAT=$(echo "$BAT" | grep -wE "^($(echo $ONLY | tr ' ' '|'))")
{ echo "config $CFG  arm $ARM  commit $(git -C $ROOT rev-parse --short HEAD)$(git -C $ROOT diff --quiet HEAD -- cpp_core godot_host/src godot_host/project/scripts || echo '+dirty')"
  echo "config sha256 $(sha256sum $ROOT/godot_host/project/addons/ami_ogma/configs/$CFG.json | cut -c1-16)  so sha256 $(sha256sum $ROOT/godot_host/project/addons/ami_ogma/ami_ogma_host.so | cut -c1-16)"
  [ -n "${EXPECT:-}" ] && echo "expect $EXPECT"
  echo "$BAT"; } > $OUT/manifest.txt
cat $OUT/manifest.txt
one() {   # one battery line, with every bench variable set or unset explicitly so the caller's shell cannot leak into it
  read -r name gym h n st seed steps <<< "$1"; port=$(( PORT_BASE + $2 ))
  ev=(env -u OGMA_PICRAWLER_STAIRS_H_MM -u OGMA_PICRAWLER_STAIRS_N -u OGMA_PICRAWLER_STAIRS_START_M OGMA_PICRAWLER_LOAD_LOG=1 OGMA_PICRAWLER_DIAG_INTERVAL=5 OGMA_DUMP_PARAMS=1
      OGMA_PICRAWLER_GYM=$gym OGMA_SEED=$seed STEPS=$steps)
  [ "$h" != - ] && ev+=(OGMA_PICRAWLER_STAIRS_H_MM=$h)
  [ "$n" != - ] && ev+=(OGMA_PICRAWLER_STAIRS_N=$n OGMA_PICRAWLER_STAIRS_START_M=$st)
  f=$OUT/${ARM}_$name.jsonl.gz
  "${ev[@]}" $TOOLS/run_gain0.sh $CFG $port $f > /dev/null
  got=$(zcat $f | grep -o '"event":"WORLD_READY"[^}]*"seed":"[0-9]*"' | grep -o '[0-9]*"$' | tr -d '"')
  rows=$(zcat $f | grep -c '"fwd_v"'); want=$(( steps / 5 ))
  ok=OK; [ "$got" = "$seed" ] || ok="SEED MISMATCH (set $seed, world $got)"
  [ $rows -ge $(( want - 2 )) ] || ok="$ok SHORT ($rows of $want rows)"
  if [ -n "${EXPECT:-}" ]; then ex=$(python3 $TOOLS/stepper_expect.py $f $EXPECT) || ok="$ok $ex"; fi
  echo "$name seed $seed world_seed ${got:-none} rows $rows/$want $ok"
}
export -f one; export OUT ARM CFG TOOLS EXPECT="${EXPECT:-}" PORT_BASE=${PORT_BASE:-8100}
echo "$BAT" | awk '{print NR-1 "\t" $0}' | xargs -P ${JOBS:-15} -I{} bash -c 'l="{}"; one "${l#*	}" "${l%%	*}"' | sort | tee $OUT/runs.txt
bad=$(grep -vc ' OK$' $OUT/runs.txt); echo "battery $ARM: $(wc -l < $OUT/runs.txt) runs, $bad failed a harness gate -> $OUT"
[ "$bad" = 0 ]
