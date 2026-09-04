cmake_minimum_required(VERSION 3.22)
project(brain_builder LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# The brain builder: a Dear ImGui desktop app that links the core directly,
# enumerates every registered module, and edits GraphConfig JSON that mj_host
# and the Godot host run unchanged.  Nothing in cpp_core is touched by this
# project; it is a consumer of the registry exactly like the hosts are.
#
# Windowing: GLFW built here.  This machine has X11 headers but no Wayland
# development headers, so the Wayland backend is opt-in; under a Wayland
# session the X11 build runs through XWayland and the compositor decorates
# the window like any other X11 client.
# ---------------------------------------------------------------------------
option(BRAIN_BUILDER_WAYLAND
  "Build GLFW's Wayland backend (needs libwayland-dev, wayland-protocols, libxkbcommon-dev)" OFF)
option(BRAIN_BUILDER_TESTS "Build the builder's unit tests" ON)

include(FetchContent)

# 1. GLFW 3.4 (zlib licence)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_X11      ON  CACHE BOOL "" FORCE)
set(GLFW_BUILD_WAYLAND  ${BRAIN_BUILDER_WAYLAND} CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG        3.4
  GIT_SHALLOW    TRUE)

# 2. Dear ImGui, docking branch (MIT).  No CMake upstream: SOURCE_SUBDIR names
#    a directory that does not exist so FetchContent only populates it, and
#    the static library is declared below over the exact files we use.
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.91.9b-docking
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  .nocmake)

# 3. imgui-node-editor (MIT).  Its CMake is for its examples; same trick.
#    The last tag (v0.9.3, 2023) predates ImGui 1.91; the develop branch tracks
#    current ImGui, so it is pinned by commit.
FetchContent_Declare(imgui_node_editor
  GIT_REPOSITORY https://github.com/thedmd/imgui-node-editor.git
  GIT_TAG        b302971455b3719ec9b5fb94b2f92d27c62b9ff0
  SOURCE_SUBDIR  .nocmake)

FetchContent_MakeAvailable(glfw imgui imgui_node_editor)
find_package(OpenGL REQUIRED)

add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${imgui_SOURCE_DIR}/misc/cpp)
target_compile_definitions(imgui PUBLIC IMGUI_DEFINE_MATH_OPERATORS)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

file(GLOB NODE_EDITOR_SOURCES ${imgui_node_editor_SOURCE_DIR}/*.cpp)
add_library(imgui_node_editor STATIC ${NODE_EDITOR_SOURCES})
target_include_directories(imgui_node_editor PUBLIC ${imgui_node_editor_SOURCE_DIR})
target_link_libraries(imgui_node_editor PUBLIC imgui)

# 4. The core, exactly as mj_host pulls it in.
set(OGMA_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(../cpp_core cpp_core_build)

get_filename_component(BB_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

# 5. bb_core: everything that is not a window.  The CLI modes and the tests
#    link this alone, so the catalogue and the wiring model stay headless.
add_library(bb_core STATIC
  src/TrialSetup.cpp
  src/Catalogue.cpp
  src/PaletteGen.cpp
  src/Graph.cpp
  src/Body.cpp
  src/Wiring.cpp
  src/Layout.cpp
  src/DryRun.cpp
  src/Order.cpp
  src/Publish.cpp)
target_include_directories(bb_core PUBLIC src ${BB_REPO_ROOT}/cpp_core/include)
target_link_libraries(bb_core PUBLIC ogma_core ogma_infra)
target_compile_definitions(bb_core PUBLIC
  BB_ROOT="${BB_REPO_ROOT}"
  BB_MJ_CONFIG_DIR="${BB_REPO_ROOT}/mj_host/configs"
  BB_GODOT_CONFIG_DIR="${BB_REPO_ROOT}/godot_host/project/addons/ami_ogma/configs"
  BB_BODIES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/bodies"
  BB_PALETTE="${CMAKE_CURRENT_SOURCE_DIR}/palette.json")
target_compile_options(bb_core PRIVATE -Wall -Wextra)

# 6. The app.
add_executable(brain_builder
  src/main.cpp
  src/ui/App.cpp
  src/ui/Theme.cpp
  src/ui/PalettePanel.cpp
  src/ui/CanvasPanel.cpp
  src/ui/PropertiesPanel.cpp
  src/ui/PublishDialog.cpp)
target_link_libraries(brain_builder PRIVATE bb_core imgui imgui_node_editor)
target_compile_options(brain_builder PRIVATE -Wall -Wextra)

# 7. Tests (GoogleTest, the same pin cpp_core uses).
if(BRAIN_BUILDER_TESTS)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  add_executable(bb_test_catalogue tests/test_catalogue.cpp)
  target_link_libraries(bb_test_catalogue PRIVATE bb_core GTest::gtest_main pthread)
  add_test(NAME bb_test_catalogue COMMAND bb_test_catalogue)
  add_executable(bb_test_wiring tests/test_wiring.cpp)
  target_link_libraries(bb_test_wiring PRIVATE bb_core GTest::gtest_main pthread)
  add_test(NAME bb_test_wiring COMMAND bb_test_wiring)
endif()
