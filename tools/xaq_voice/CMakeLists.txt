cmake_minimum_required(VERSION 3.22)
project(xaq_voice LANGUAGES CXX)
# xaq_voice — the brain's surprise, audible.  A subscriber to the diag stream (the same
# one the inspector reads), never on the control loop.  See README.md.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(ALSA REQUIRED alsa)
include(FetchContent)
FetchContent_Declare(json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.3)
FetchContent_MakeAvailable(json)
add_library(xaq_voice_core STATIC src/patch.cpp src/engine.cpp src/control.cpp)
target_include_directories(xaq_voice_core PUBLIC src PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(xaq_voice_core PUBLIC nlohmann_json::nlohmann_json PRIVATE ${ZMQ_LIBRARIES})
target_compile_options(xaq_voice_core PRIVATE -Wall -Wextra -O2)

add_executable(xaq_voice src/main.cpp)
target_include_directories(xaq_voice PRIVATE ${ZMQ_INCLUDE_DIRS} ${ALSA_INCLUDE_DIRS})
target_link_libraries(xaq_voice PRIVATE xaq_voice_core nlohmann_json::nlohmann_json ${ZMQ_LIBRARIES} ${ALSA_LIBRARIES} pthread)
target_compile_options(xaq_voice PRIVATE -Wall -Wextra -O2)

# The DSP and the patch format are pure arithmetic and pure data, so they are testable
# with no sim, no socket and no sound card.  That is the whole reason they are not in
# main.cpp: an oscillator that aliases or a filter that blows up under modulation is a
# thing to catch here, not by ear at 2 a.m. with a robot walking.
option(XAQ_VOICE_TESTS "Build the xaq_voice unit tests" ON)
if(XAQ_VOICE_TESTS)
  enable_testing()
  add_executable(test_xaq_voice tests/test_xaq_voice.cpp)
  target_link_libraries(test_xaq_voice PRIVATE xaq_voice_core nlohmann_json::nlohmann_json ${ZMQ_LIBRARIES} pthread)
  target_compile_options(test_xaq_voice PRIVATE -Wall -Wextra -O2)
  add_test(NAME xaq_voice COMMAND test_xaq_voice)
endif()
