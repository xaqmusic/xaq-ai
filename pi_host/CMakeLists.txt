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
)
target_include_directories(ogma_hw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(ogma_hw PRIVATE -Wall -Wextra)

add_executable(hat_tool tools/hat_tool.cpp)
target_link_libraries(hat_tool PRIVATE ogma_hw)

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
  add_test(NAME test_hw COMMAND test_hw)
endif()
