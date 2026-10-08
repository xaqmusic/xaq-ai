# arm_bench — the MicroDuck brain's cost on an ARM board

The question it answers: **how much CPU and memory do the MicroDuck's three brains (intent, head, stop) need on
Pollen's Radxa Zero 3W** (Rockchip RK3566, 4× Cortex-A55)? We do not have one yet. We do have a Raspberry Pi 5
over ssh (`picrawler`). So the harness measures on the Pi, records everything a projection needs, and runs unchanged
on a Radxa when one turns up. At that point the projection becomes a measurement.

It is built on the replay benchmark (design doc §17.111): the MuJoCo host records a tape of everything it did to the
brains (`--record-brains`); `ogma_brain_replay` rebuilds the brains from the tape alone (no MuJoCo, no ONNX), drives them
with the same inputs, checks tick by tick that every output matches the live run, and times each tick.

```sh
mj_host/tools/arm_bench/arm_bench.py all picrawler           # record (if needed) → stage → build → run → report
mj_host/tools/arm_bench/arm_bench.py report mj_host/log/arm_bench/<run>/   # re-read a run
```

| step | where | what |
|---|---|---|
| `record` | here | ★ F5, seed 1, 600 s, with the argv taken from the launcher's own `host_args()`, so it cannot drift from the preset. Writes `mj_host/log/arm_bench/f5_s1.tape` (95 MB) with `.argv` / `.preset` beside it |
| `stage HOST` | board | rsyncs `cpp_core/`, the two replay sources, this directory, the pinned Eigen 3.4.0 / json v3.11.2 sources (taken from a local build, so the board builds offline) and the tape to `/tmp/ogma_arm_bench`. That is tmpfs on the Pi, so nothing is written to its SD card, which is full. 139 MB |
| `build HOST` | board | the replay-only CMake project here (`mj_host/CMakeLists.txt` would fetch x86 MuJoCo); `-mcpu=cortex-a55`, otherwise the compiler's defaults (as a deployment would build), `-j2` |
| `run HOST` | board | `remote_run.sh`: timing, counters, contention (below), with a sampler over the whole run. Results are copied back to `mj_host/log/arm_bench/<host>_<stamp>/` |
| `report DIR` | here | `report.md` + `projection.json` |

Requirements on the board: `cmake`, `g++`, `libzmq3-dev` (cpp_core links it), `rsync`, `taskset`. Nothing runs as root.
On a kernel with `perf_event_paranoid > 2` (some Debian kernels), the counters pass needs `--sudo-counters`.

## What the run measures

1. **timing**: the replay pinned to one core (`--core`, default 2), `--repeat 3`, under `ulimit -v 1 GB`. Thread CPU
   time per tick for each brain, its quantiles, ticks over the 20 ms budget, and fidelity. If "reproduces the live run"
   is not *yes*, the board computed different numbers from the desktop. The timings are still the cost of driving these
   brains with these inputs, but no longer the cost of *that* run.
2. **counters**: one pass with `--counters`: per-tick user-space hardware counts around `tick()` only. These are
   instructions, cycles, L1D/L2D/L3D refills, backend stalls and branch misses, using the ARMv8 common event numbers,
   which are the same on the A76 and the A55. The cost of reading the counters is measured on an empty window and
   subtracted. The report compares this pass's mean with the timing pass's, so the instrument's own perturbation is
   visible.
3. **contention**: one replay on every core at once. This loads the shared L3 and DRAM, as Pollen's runtime and the
   walking policy will on the real duck.

A sampler logs every thermal zone, every core's clock, every core's busy share and (on a Pi) the throttle flags every
0.5 s. `facts.txt` records the board, kernel, page size, governor, compiler, flags and the busiest processes before and
after. **The harness does not change the governor or stop any service.** It records them instead. On the picrawler,
`ogma-host.service` (the PiCrawler brain, SCHED_FIFO, ~5–9 % of a core) is normally running; the Conditions table shows
how busy the other cores were.

## Fidelity: why "reproduces the live run" says NO on ARM, and what replaces it

On x86 the replay reproduces every output of all three brains for all 600 s. On aarch64 it cannot. The cause was isolated
one operation at a time on 2026-10-08, using `ogma_brain_replay --dump FILE --dump-ticks N` (every module's outputs as
hex floats) on both machines and probes of single operations:

- **The first difference** is in all three brains: a MotorEPMv2 action at tick 1, 1–4 ulp. Every EPM token before it
  (winner, TLE) matches.
- **libm is not the cause.** Twenty-three float and double functions (`tanh`, `exp`, `log`, `sin`, `atan2`, `pow`, …)
  give the same bits over a million inputs on glibc 2.43 (x86) and 2.41 (Pi).
- **FMA is only part of it.** `-ffp-contract=off -U__ARM_FEATURE_FMA` makes Eigen's element-wise ops, dot products,
  norms and sums bit-identical to x86, and the replay still diverges at the same tick with the same counts.
- **Eigen's matrix products are the rest.** Matrix–vector, matrix–matrix and the inverse differ under every flag
  combination: Eigen's aarch64 product kernels block registers differently, so they sum in another order. MotorEPMv2's
  command is `tanh(C·x + …)`.

No build flag reaches bit-identity, short of turning off Eigen's vectorization, which would change the summation order
on x86 too and would misstate the cost. So the build uses the compiler's defaults, as a deployment would, and fidelity
is checked a different way: **the same work at the same ticks.** `arm_bench.py reference` replays the tape twice on x86,
where it does reproduce the run. The report correlates the board's per-tick cost with that reference, using the two x86
copies as the noise ceiling, and counts how many of the most expensive 1 % of ticks coincide. On the Pi 5 the intent
brain, 90 % of the cost, reaches r = 0.994 against a ceiling of 0.999, and 94 % of its top-1 % ticks are the same ticks.
The tiny numerical differences change the brain's choices slightly but not its work. Head and stop cost nearly the same
every tick, so even two x86 runs barely correlate there; the check is uninformative for them, not failed.

## The projection to the RK3566 — and why it is needed

The Pi 5's Cortex-A76 is out-of-order, 2.4 GHz, with a 64 kB L1D, a 512 kB private L2 and a 2 MB L3. The RK3566's
Cortex-A55 is in-order (dual-issue), 1.6 GHz on the Zero 3W, with a 32 kB L1D, **no L2**, and **one 512 kB L3 shared by all
four cores**. Microseconds measured on the Pi do not transfer. Two things do:

- **The instructions.** The binary is built with `-mcpu=cortex-a55`. Both cores implement ARMv8.2-A, so the A55 build
  runs on the A76 and retires the same instruction stream it would on the A55.
- **The working set's cache behaviour, roughly.** The A76's private L2 is 512 kB, the size of the RK3566's whole L3, so
  a tick's A76 L2 refills stand in for its DRAM accesses on the Radxa. This is pessimistic where the A76 prefetcher
  refills lines the tick never uses, and optimistic where the other cores on the RK3566 claim part of its L3.

The report gives two independent estimates, each a low–high bracket (low = the kinder board):

**A, whole-CPU ratio.** Time on the Pi × 4.2–4.4. Geekbench 6 single-core is ~764–800 for the Pi 5 at 2.4 GHz
([Raspberry Pi](https://www.raspberrypi.com/news/benchmarking-raspberry-pi-5/)) against ~203 for the RK3566
([Notebookcheck](https://www.notebookcheck.com/Rockchip-RK3566-Prozessor-Benchmarks-und-Specs.741610.0.html), boards
typically at 1.8 GHz), rescaled to the Zero 3W's 1.6 GHz
([CNX Software](https://www.cnx-software.com/2023/10/30/radxa-zero-3w-sbc-features-rockchip-rk3566-soc-up-to-8gb-ram-in-raspberry-pi-zero-2-w-form-factor/)).
It is valid only if the Pi ran at 2.4 GHz during the timing pass; the Conditions table shows the clock. A benchmark
suite's mix is not this workload's, which is why B exists.

**B, the work on an in-order core**, tick by tick:

```
t_A55 = ( instructions × CPI_core                        CPI_core 1.3–2.0: the A55 with its data in L1
        + L1D refills × miss_scale × L3_hit_cycles       miss_scale 1–2 (32 kB L1D vs 64 kB), L3 hit 25–45 cycles
        + L2D refills × DRAM_ns × f ) / f                DRAM 110–170 ns (LPDDR4), f = 1.6 GHz
```

Computing it per tick keeps the worst tick's own instructions and misses together, so its p99 and max are the model's
estimate of the slowest ticks, not a mean scaled up. The parameters sit in `TARGET` in `arm_bench.py`. They are
assumptions from the cores' published structure, **not measurements**. When A and B agree, each supports the other;
when they disagree, the disagreement tells you which assumption to doubt.

**Memory transfers directly.** It is the same 64-bit code, the same libstdc++ and the same allocator. The one difference:
the Pi 5 kernel uses 16 kB pages, and the Radxa's Debian/Ubuntu images use 4 kB, so resident set size on the Pi is a
slight overestimate. The brains' own heap (arena + mmapped blocks, with every brain alive at the end of the tape), and
its trace once a second, do not depend on page size.

## What would turn the projection into a measurement

Run `arm_bench.py all <radxa-host>`. The same harness, on the real core, measures cycles directly, and projection B can
then be checked against it. Two further gaps, both named here so they are not forgotten:

- **The policy beside it.** On the duck, the brain shares the A55s and the L3 with Pollen's 50 Hz Rust loop and its ONNX
  walking policy. The contention pass is a proxy (copies of the brain, not the policy).
- **Longer than 600 s.** The heap grows over the tape. Whether it levels off needs a longer tape
  (`arm_bench.py record --secs 3600 --tape …`, ~570 MB, which is too large for the Pi's tmpfs alongside the build; stage it to disk).
