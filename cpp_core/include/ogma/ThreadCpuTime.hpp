#pragma once

// =============================================================================
// ThreadCpuTime.hpp  --  CPU time used by the calling thread, for OGMA_PROFILE
// =============================================================================
//
// POSIX: clock_gettime(CLOCK_THREAD_CPUTIME_ID).  Windows: GetThreadTimes
// (kernel + user, 100 ns units; the OS updates it at its tick, ~15.6 ms by
// default, so short spans read as 0 and only means over many ticks are useful).

namespace ogma {

// Microseconds of CPU time the calling thread has used.
double thread_cpu_us();

}  // namespace ogma
