cmake_minimum_required(VERSION 3.22)
project(ogma_pi_host LANGUAGES CXX)
# pi_host — the PiCrawler hardware host (Raspberry Pi 5 + SunFounder Robot HAT V4).
# Day-1 scope: the HAT wire protocol, the servo safety envelope, a bench CLI, and
# tests against a fake bus.  ogma_core (the brain) is linked in by ogma_host later;
# it is deliberately NOT a dependency of this layer so the driver builds in seconds.
# Design: docs/plans-and-designs/picrawler_sim2real_port.md  (Phase 2, Phase 4, SPEC).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
option(PI_HOST_BUILD_TESTS "Build pi_host tests (fetches googletest)" ON)

add_library(ogma_hw
  src/RobotHat.cpp
  src/ServoDriver.cpp
  src/LinuxI2cBus.cpp
  src/McuReset.cpp
  src/Ina219.cpp
  src/ResourceMonitor.cpp
)
target_include_directories(ogma_hw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(ogma_hw PRIVATE -Wall -Wextra)

add_executable(hat_tool tools/hat_tool.cpp)
target_link_libraries(hat_tool PRIVATE ogma_hw)

# ogma_benchd — the bench daemon (PROTOCOL.md): ZMQ REP verbs + PUB telemetry over the driver.
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
include(FetchContent)
FetchContent_Declare(json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3)
FetchContent_MakeAvailable(json)
add_executable(ogma_benchd src/benchd.cpp)
target_include_directories(ogma_benchd PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(ogma_benchd PRIVATE ogma_hw nlohmann_json::nlohmann_json ${ZMQ_LIBRARIES} pthread)
target_compile_options(ogma_benchd PRIVATE -Wall -Wextra)

# ogma_sensors — the three physical input channels.  Kept OUT of ogma_hw so the
# servo/driver layer keeps its seconds-long build and gains no ALSA/gpiod deps, but
# OUTSIDE the brain guard so its pure helpers stay unit-testable without ogma_core.
pkg_check_modules(ALSA alsa)
pkg_check_modules(GPIOD libgpiod)
if(ALSA_FOUND AND GPIOD_FOUND)
  set(PI_HOST_HAVE_SENSORS ON)
  add_library(ogma_sensors
    src/AudioCapture.cpp
    src/CameraCapture.cpp
    src/Ultrasonic.cpp
  )
  target_include_directories(ogma_sensors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_include_directories(ogma_sensors PRIVATE ${ALSA_INCLUDE_DIRS} ${GPIOD_INCLUDE_DIRS})
  target_link_libraries(ogma_sensors PUBLIC ${ALSA_LIBRARIES} ${GPIOD_LIBRARIES} pthread)
  target_compile_options(ogma_sensors PRIVATE -Wall -Wextra)

  # cam_tool — dumps what the retinal EPM sees, as an image.  A claim about pixels is
  # settled by looking at pixels, not by reading the pipeline again.
  add_executable(cam_tool tools/cam_tool.cpp)
  target_link_libraries(cam_tool PRIVATE ogma_sensors)
  target_compile_options(cam_tool PRIVATE -Wall -Wextra)
else()
  message(STATUS "pi_host: ALSA/libgpiod not found — ogma_sensors and ogma_host disabled")
endif()

if(PI_HOST_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  add_executable(test_hw tests/test_hw.cpp)
  target_link_libraries(test_hw PRIVATE ogma_hw gtest_main)
  if(PI_HOST_HAVE_SENSORS)
    target_link_libraries(test_hw PRIVATE ogma_sensors)
    target_compile_definitions(test_hw PRIVATE PI_HOST_HAVE_SENSORS=1)
  endif()
  add_test(NAME test_hw COMMAND test_hw)
endif()

# ---------------------------------------------------------------------------
# ogma_host — the brain on the Pi (port doc SPEC §1).  OFF by default: it pulls in
# ogma_core, which is a ~10 minute build on this board, and the servo/driver work
# above must stay a seconds-long edit-build-test loop.
#   cmake -S pi_host -B pi_host/build -DPI_HOST_BUILD_BRAIN=ON
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# ogma_host — the brain on the Pi (port doc SPEC §1).  OFF by default: it pulls in
# ogma_core, a ~10 minute build on this board, and the servo/driver work above must
# stay a seconds-long edit-build-test loop.
#   cmake -S pi_host -B pi_host/build -DPI_HOST_BUILD_BRAIN=ON
# ---------------------------------------------------------------------------
option(PI_HOST_BUILD_BRAIN "Build ogma_host (links ogma_core)" OFF)
if(PI_HOST_BUILD_BRAIN AND PI_HOST_HAVE_SENSORS)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp_core cpp_core_build EXCLUDE_FROM_ALL)
  add_executable(ogma_host src/ogma_host.cpp)
  target_link_libraries(ogma_host PRIVATE ogma_hw ogma_sensors ogma_core ogma_infra)
  target_compile_options(ogma_host PRIVATE -Wall -Wextra)
endif()
