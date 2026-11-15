#!/bin/bash
# stepper_loo.sh — the learned stepper plan's Phase 1 leave-one-out (docs/plans-and-designs/learned_stepper_plan.md §4): each of the
# scaffold's reflexes and priors OFF ALONE on the fixed battery, against a control on the same build.
#   usage: stepper_loo.sh [base config (sim60_stairs_seq2)] [arm tag ...]     (no tags = every arm below, the control first)
#   writes <base>__p1<tag>.json with mkarm.py (exactly one PoseSequencer param changed, the diff printed), then runs
#   stepper_battery.sh with EXPECT set so every run asserts the lever landed; logs in ~/xaq_runs/stepper/p1/<tag>/.
#   Score all arms together:  stepper_score.py ~/xaq_runs/stepper/p1/*   (arm = the file prefix p1<tag>)
set -u
BASE=${1:-sim60_stairs_seq2}; shift || true
TOOLS=/home/xaqmusic/xaq-ai/godot_host/project/scripts_tools; ROOT=${OUT_ROOT:-$HOME/xaq_runs/stepper}/p1
declare -A KV=(
  [settle]="settle=0"                 # the slide to the neutral places at engage
  [level]="level_gain=0"              # the stance's levelling from the IMU
  [head]="heading_hold=0"             # the heading hold (the planted toes rotated during SHIFT)
  [centre]="centre_gain=0"            # the shift's pull toward the toes' centroid (the margin controller's; under a plan only after a hang or an empty search)
  [edge]="land_edge_m=0"              # the landing moved off a remembered riser edge
  [retreat]="retreat_m=0"             # the lowering's retreat toward the hip when the toe cannot reach down
  [fastlow]="lower_fast_v_mps=0"      # the fast lowering to a standoff above the memory's ground
  [hang]="hang_ticks=0"               # the hung-toe reflex (form 3)
  [reseed]="ik_reseed_ticks=0"        # the IK re-seed from a pinned wrong branch
  [multi]="ik_multistart=0"           # the multistart IK
  [fdrop]="front_drop_gain=0"         # the front drop on a nose-up body
  [landr]="land_r_min_m=0"            # the landing radius floor (7 cm; 0 = reach_min_m, 6 cm)
  [tmo]="phase_max_ticks=100000"      # the phase timeouts (LIFT, SWING, and the lowering's 4x)
  [clamp]="land_clamp=0"              # the go-time landing clamps (reach, landing sector)
  [obst]="obstacle_ticks=0"           # the obstacle reflex (a toe stopped mid-swing is lowered where it is)
  [creep]="creep_v_mps=0"             # the load-seeking creep of an unbearing planted toe
)
ORDER="settle level head centre edge retreat fastlow hang reseed multi fdrop landr tmo clamp obst creep"
TAGS=${*:-ctl $ORDER}
cmake --build /home/xaqmusic/xaq-ai/godot_host/build --target ami_ogma_host -j8 > /tmp/stepper_loo_build.log 2>&1
ne=$(grep -c ' error' /tmp/stepper_loo_build.log); [ "$ne" = 0 ] || { echo "BUILD GATE: $ne errors"; exit 3; }
for t in $TAGS; do
  if [ $t = ctl ]; then cfg=$BASE; ex=""
  else
    kv=${KV[$t]:-}; [ -n "$kv" ] || { echo "unknown arm $t"; continue; }
    python3 $TOOLS/mkarm.py $BASE.json p1$t $kv --module=pose_sequencer | tail -3
    cfg=${BASE}__p1$t; k=${kv%%=*}; ex="pose_sequencer.$k=${kv#*=}"
  fi
  echo "=== arm p1$t ($cfg) $(date +%H:%M)"
  EXPECT="$ex" NOBUILD=1 $TOOLS/stepper_battery.sh $cfg p1$t $ROOT/$t | tail -1
done
