#!/usr/bin/env bash
# run.sh — sweep epm_kalman_bench over seeds and summarise.
#
# Charter: docs/plans-and-designs/epm_kalman_lessons_plan.md (Stage 0.2).
#
#   tools/epm_bench/run.sh <scenario> [--seeds N] [--tag name] [--arm module.key=json]* [bench args...]
#
# Output: $EPM_BENCH_OUT (default /tmp/epm_bench)/<scenario>_<tag>/seed_<i>.jsonl, then the
# analyze.py table for that directory.  Compare two tags with
#   python3 tools/epm_bench/analyze.py --compare <dir_base> <dir_arm>
#
# Examples
#   tools/epm_bench/run.sh S1 --seeds 20                      # baseline
#   tools/epm_bench/run.sh S1 --seeds 20 --tag kalman --arm epm.gain_kind=kalman
#   tools/epm_bench/run.sh S5 --seeds 20 --tag dead --dead_at 2000 --arm voter.informativeness_gain=1.0
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
BIN="$ROOT/cpp_core/build/epm_kalman_bench"
OUT_ROOT="${EPM_BENCH_OUT:-/tmp/epm_bench}"

if [[ $# -lt 1 ]]; then sed -n 2,20p "$0"; exit 2; fi
scenario="$1"; shift
seeds=20; tag=base; rest=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --seeds) seeds="$2"; shift 2 ;;
    --tag)   tag="$2";   shift 2 ;;
    --arm)   rest+=(--set "$2"); shift 2 ;;
    *)       rest+=("$1"); shift ;;
  esac
done

[[ -x "$BIN" ]] || { echo "bench binary missing: build with  cmake --build cpp_core/build --target epm_kalman_bench -j8"; exit 1; }
dir="$OUT_ROOT/${scenario}_${tag}"
mkdir -p "$dir"
rm -f "$dir"/seed_*.jsonl

jobs="$(nproc 2>/dev/null || echo 4)"
seq 0 $((seeds - 1)) | xargs -P "$jobs" -I{} \
  "$BIN" --scenario "$scenario" --seed {} --out "$dir/seed_{}.jsonl" "${rest[@]}"

echo "wrote $seeds seeds to $dir"
python3 "$HERE/analyze.py" "$dir"/seed_*.jsonl
