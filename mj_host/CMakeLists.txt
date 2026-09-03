cmake_minimum_required(VERSION 3.22)
project(ogma_mj_host LANGUAGES CXX)

# mj_host — the Microduck MuJoCo host.
#
# A peer of godot_host/ and pi_host/, not a layer on either.  OgmaInstance.hpp names
# "the host (Godot Host, HAL Host, Debug Host)" as the owner of the Bus, so a second
# host is the anticipated shape rather than a new one: it holds an OgmaInstance in
# plain C++ and never touches OgmaBrain's Godot Variant marshalling.
#
# Plan: docs/plans-and-designs/microduck_port_plan.md  (phase S0).
# Body:  models/microduck/  — vendored from pollen-robotics/microduck_rl, unmodified.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Linking ogma_core costs a few minutes on a cold build and buys nothing until S1.
# Off by default so `--load-only` (gate G1) stays a seconds-long edit/build loop;
# S1 turns it on and keeps it on.
option(MJ_HOST_WITH_BRAIN "Link ogma_core into the host (needed from phase S1)" OFF)

include(FetchContent)

# ---------------------------------------------------------------------------
# 1. MuJoCo — the official prebuilt release, pinned by version AND by hash.
#
#    Prebuilt rather than built from source: the source build pulls its own
#    dependency tree for no benefit here, and the release binary is what
#    microduck_rl's own CPU path runs against.
#
#    The version is part of the body of record, not an implementation detail.
#    MJCF parsing is version-sensitive, and 3.12.0 is the version every number in
#    models/microduck/README.md was measured under.  Re-pin deliberately, and
#    re-run `ogma_mjhost --load-only` when you do: a model that loads differently
#    is a different robot.
# ---------------------------------------------------------------------------

set(MUJOCO_VERSION 3.12.0)

FetchContent_Declare(mujoco
  URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz
  URL_HASH SHA256=a9367911e6d5eaeade17c2197304687421c1fc932cdf7bcd4cb8cfaf0374dcb2
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(mujoco)

# The tarball is a plain SDK layout (include/ + lib/), not a CMake project, so the
# target is declared here rather than imported.
add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
  IMPORTED_LOCATION             "${mujoco_SOURCE_DIR}/lib/libmujoco.so.${MUJOCO_VERSION}"
  INTERFACE_INCLUDE_DIRECTORIES "${mujoco_SOURCE_DIR}/include"
)

# ---------------------------------------------------------------------------
# 2. ONNX Runtime — for the standing scaffold (models/microduck/scaffolds/).
#
#    S1 needs a controller in the loop because this body has no passive standing
#    equilibrium: holding the STAND keyframe's own ctrl topples it to 81 degrees.
#    So the no-brain baseline is an actively balanced one, and that costs a second
#    prebuilt dependency.  Pinned the same way MuJoCo is, and to the same version
#    the Python cross-check ran against.
# ---------------------------------------------------------------------------

set(ORT_VERSION 1.29.0)

FetchContent_Declare(onnxruntime
  URL https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz
  URL_HASH SHA256=c3fddc4f139a045b0c4902c57410f0694f1c2fdf9b6939fbe38b1aeae7cd14ba
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(onnxruntime)

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
  IMPORTED_LOCATION             "${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so"
  INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_SOURCE_DIR}/include"
)

# ---------------------------------------------------------------------------
# 3. Ogma Core — the brain, when asked for.
# ---------------------------------------------------------------------------

if(MJ_HOST_WITH_BRAIN)
  set(OGMA_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  add_subdirectory(../cpp_core cpp_core_build)
endif()

# ---------------------------------------------------------------------------
# 4. The host
# ---------------------------------------------------------------------------

add_executable(ogma_mjhost
  src/main.cpp
  src/DuckBody.cpp
  src/Odometry.cpp
  src/IntentAdapter.cpp
  src/InspectorSurface.cpp
  src/Observation.cpp
  src/Policy.cpp
  src/Recovery.cpp
  src/OgmaBrainAdapter.cpp
)
target_link_libraries(ogma_mjhost PRIVATE mujoco onnxruntime)
target_compile_options(ogma_mjhost PRIVATE -Wall -Wextra)

if(MJ_HOST_WITH_BRAIN)
  target_link_libraries(ogma_mjhost PRIVATE ogma_core ogma_infra)
  target_include_directories(ogma_mjhost PRIVATE ../cpp_core/include)
  target_compile_definitions(ogma_mjhost PRIVATE MJ_HOST_WITH_BRAIN=1)
endif()

# Where the vendored body lives, so `--load-only` needs no argument and no cwd.
target_compile_definitions(ogma_mjhost PRIVATE
  MJ_HOST_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models/microduck"
  MJ_HOST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/configs")

# libmujoco.so is not on the loader path; carry it on the binary rather than
# asking every caller to set LD_LIBRARY_PATH.
set_target_properties(ogma_mjhost PROPERTIES
  BUILD_RPATH   "${mujoco_SOURCE_DIR}/lib;${onnxruntime_SOURCE_DIR}/lib"
  INSTALL_RPATH "${mujoco_SOURCE_DIR}/lib;${onnxruntime_SOURCE_DIR}/lib")
