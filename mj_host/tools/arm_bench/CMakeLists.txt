cmake_minimum_required(VERSION 3.22)
project(ogma_arm_bench LANGUAGES CXX)

# The replay benchmark alone, for an ARM board (2026-10-08; README.md beside this file).
#
# mj_host/CMakeLists.txt fetches the x86-64 MuJoCo and ONNX Runtime tarballs unconditionally, and the replay needs
# neither -- so the board builds this instead: ogma_core plus brain_replay.cpp and BrainTape.cpp, nothing else.
# The tree is laid out as in the repo (arm_bench.py copies cpp_core/ and the two mj_host files to the same paths).
#
# The target CPU is a flag, not a guess: OGMA_ARM_CPU (default cortex-a55, Pollen's Radxa Zero 3W) becomes -mcpu=, so
# the binary measured on the Pi 5's A76 is the one an A55 would run (both are ARMv8.2-A), and it retires the same
# instructions on both.
# Otherwise the compiler's defaults, as a deployment would build.  The aarch64 brain does NOT compute the x86 recording's
# numbers bit for bit, and no flag makes it (2026-10-08, README.md "Fidelity"): libm agrees across the two machines, and
# -ffp-contract=off -U__ARM_FEATURE_FMA makes element-wise ops and reductions agree, but Eigen's aarch64 matrix-product
# kernels sum in a different order -- the first difference is MotorEPMv2's action, tanh(C*x + ...), at tick 1, 1-4 ulp.
# The replay's own check is therefore replaced by arm_bench.py's: the same work at the same ticks.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(OGMA_ARM_CPU "cortex-a55" CACHE STRING "-mcpu= for every target ('' for the compiler's default)")
if(OGMA_ARM_CPU)
  add_compile_options(-mcpu=${OGMA_ARM_CPU})
endif()

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(OGMA_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${REPO}/cpp_core cpp_core_build)

add_executable(ogma_brain_replay ${REPO}/mj_host/tools/brain_replay.cpp ${REPO}/mj_host/src/BrainTape.cpp)
target_link_libraries(ogma_brain_replay PRIVATE ogma_core ogma_infra)
target_include_directories(ogma_brain_replay PRIVATE ${REPO}/cpp_core/include ${REPO}/mj_host/src)
target_compile_options(ogma_brain_replay PRIVATE -Wall -Wextra)
