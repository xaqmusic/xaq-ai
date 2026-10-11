#!/usr/bin/env bash
# =============================================================================
# remote_run.sh -- runs ON the ARM board, in the bench directory arm_bench.py staged (README.md beside this file)
# =============================================================================
#
#   remote_run.sh BENCH_DIR TAPE OUT_DIR [--build DIR] [--core N] [--repeat N] [--vmem-kb N] [--no-contention] [--sudo-counters]
#
# Unprivileged, and touches nothing outside BENCH_DIR and OUT_DIR: no governor, no service, no package.  What it cannot
# control it records instead, because a confounded number looks exactly like a good one: the board's facts, and a
# sampler over the whole run (every thermal zone, every core's clock, every core's busy share, the Pi's throttle flags).
#
#   1 timing      the replay pinned to one core, --repeat N, nothing else of ours running        -> timing.json
#   2 counters    the same, once, with --counters and the per-tick CSV                          -> counters.json, ticks.csv
#   3 profile     the same, once, OGMA_PROFILE=1: thread CPU per module and per cloud stage      -> profile.txt
#   4 contention  one replay per core at once (the shared L3 and DRAM under load), timing only  -> contention_<core>.json
#                 (Pollen's runtime and its policy share that L3 with the brain on the real duck)
set -euo pipefail

BENCH=$1 TAPE=$2 OUT=$3; shift 3
BUILD=build CORE=2 REPEAT=3 VMEM=1048576 CONTENTION=1 SUDO_CTR=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) BUILD=$2; shift 2 ;;
    --core) CORE=$2; shift 2 ;;
    --repeat) REPEAT=$2; shift 2 ;;
    --vmem-kb) VMEM=$2; shift 2 ;;
    --no-contention) CONTENTION=0; shift ;;
    --sudo-counters) SUDO_CTR=1; shift ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
done
BIN="$BENCH/$BUILD/ogma_brain_replay"
[[ -x "$BIN" ]] || { echo "not built: $BIN" >&2; exit 1; }
[[ -f "$TAPE" ]] || { echo "no tape: $TAPE" >&2; exit 1; }
mkdir -p "$OUT"
NCPU=$(nproc)

# ---- the board's facts -------------------------------------------------------------------------------------------
{
  echo "date: $(date -Is)"
  echo "model: $(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo '?')"
  echo "uname: $(uname -a)"
  echo "page_size: $(getconf PAGESIZE)"
  echo "nproc: $NCPU"
  echo "core: $CORE"
  echo "cpu_parts: $(grep -E '^CPU part' /proc/cpuinfo | awk '{print $NF}' | sort | uniq -c | tr -s ' ' | tr '\n' ';')"
  for f in scaling_governor scaling_min_freq scaling_max_freq cpuinfo_max_freq scaling_available_frequencies; do
    echo "cpu${CORE}_$f: $(cat /sys/devices/system/cpu/cpu$CORE/cpufreq/$f 2>/dev/null || echo '?')"
  done
  echo "perf_event_paranoid: $(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo '?')"
  echo "mem_total_kb: $(awk '/MemTotal/{print $2}' /proc/meminfo)"
  echo "mem_available_kb: $(awk '/MemAvailable/{print $2}' /proc/meminfo)"
  echo "loadavg: $(cat /proc/loadavg)"
  echo "build_dir: $BUILD"
  echo "compiler: $(head -1 "$BENCH/$BUILD/compiler.txt" 2>/dev/null || echo '?')"
  echo "build_flags: $(cat "$BENCH/$BUILD/flags.txt" 2>/dev/null || echo '?')"
  echo "throttled_before: $(vcgencmd get_throttled 2>/dev/null || echo 'n/a')"
  echo "lscpu:"; lscpu 2>/dev/null | sed 's/^/  /'
  echo "busiest processes (before):"; ps -eo pid,psr,pcpu,rtprio,ni,rss,comm --sort=-pcpu | head -8 | sed 's/^/  /'
} > "$OUT/facts.txt"

# ---- the sampler: one ';'-separated line every 0.5 s -------------------------------------------------------------
# t ; temps (m°C, every thermal zone) ; per-core cur freq (kHz) ; throttled ; the per-core lines of /proc/stat
sampler() {
  while :; do
    local temps freqs thr stat
    temps=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | tr '\n' ',')
    freqs=$(cat /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq 2>/dev/null | tr '\n' ',')
    thr=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2 || true)
    stat=$(grep -E '^cpu[0-9]+ ' /proc/stat | tr '\n' '|')
    echo "$(date +%s.%N);$temps;$freqs;$thr;$stat"
    sleep 0.5
  done
}
phase() { echo "$(date +%s.%N) $1" >> "$OUT/phases.txt"; echo "== $1"; }
sampler > "$OUT/samples.txt" &
SAMPLER=$!
trap 'kill $SAMPLER 2>/dev/null || true' EXIT

phase idle_start; sleep 5; phase idle_end

# ---- 1 timing ----------------------------------------------------------------------------------------------------
phase timing_start
( ulimit -v "$VMEM"; taskset -c "$CORE" "$BIN" "$TAPE" --repeat "$REPEAT" --json "$OUT/timing.json" ) > "$OUT/timing.txt" 2> "$OUT/timing.err"
phase timing_end
grep -E "ticks|fidelity|process|heap with" "$OUT/timing.txt" || true

# ---- 2 counters --------------------------------------------------------------------------------------------------
phase counters_start
CTR=(taskset -c "$CORE" "$BIN" "$TAPE" --counters --json "$OUT/counters.json" --ticks-csv "$OUT/ticks.csv")
if [[ $SUDO_CTR == 1 ]]; then
  ( ulimit -v "$VMEM"; sudo -n "${CTR[@]}" ) > "$OUT/counters.txt" 2> "$OUT/counters.err"
  sudo -n chown "$(id -u):$(id -g)" "$OUT/counters.json" "$OUT/ticks.csv"
else
  ( ulimit -v "$VMEM"; "${CTR[@]}" ) > "$OUT/counters.txt" 2> "$OUT/counters.err"
fi
phase counters_end
grep -E "counters|ticks|per 1000" "$OUT/counters.txt" || true

# ---- 3 profile ---------------------------------------------------------------------------------------------------
phase profile_start
( ulimit -v "$VMEM"; OGMA_PROFILE=1 taskset -c "$CORE" "$BIN" "$TAPE" ) 2>&1 | grep '^OGMA_PROFILE' > "$OUT/profile.txt" || true
phase profile_end

# ---- 4 contention ------------------------------------------------------------------------------------------------
if [[ $CONTENTION == 1 ]]; then
  phase contention_start
  pids=()
  for ((c = 0; c < NCPU; c++)); do
    ( ulimit -v "$VMEM"; taskset -c "$c" "$BIN" "$TAPE" --json "$OUT/contention_$c.json" ) > "$OUT/contention_$c.txt" 2>/dev/null &
    pids+=($!)
  done
  for p in "${pids[@]}"; do wait "$p"; done
  phase contention_end
fi

phase cooldown_start; sleep 5; phase cooldown_end
{
  echo "throttled_after: $(vcgencmd get_throttled 2>/dev/null || echo 'n/a')"
  echo "busiest processes (after):"; ps -eo pid,psr,pcpu,rtprio,ni,rss,comm --sort=-pcpu | head -8 | sed 's/^/  /'
} >> "$OUT/facts.txt"
echo "done: $OUT"
