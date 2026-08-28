cmake_minimum_required(VERSION 3.14)
project(AmiOgmaGodotHost CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

# ---------------------------------------------------------------------------
# 1. Ogma Core (library targets only — tests are off when used as subdir)
# ---------------------------------------------------------------------------

set(OGMA_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(../cpp_core cpp_core_build)

# ---------------------------------------------------------------------------
# 2. godot-cpp  —  4.5-stable base, but using the actual extension_api.json
#    and gdextension_interface.h dumped from the installed Godot 4.6.2 binary.
#
#    Why: godot-4.6.x-stable tags don't exist in godot-cpp yet.  The GDExtension
#    binary interface is stable across minor versions; we use 4.5 template code
#    but override the API descriptor so godot-cpp generates 4.6.2-correct
#    bindings.  The dump files are committed to godot_host/ and reproduced by:
#        godot4 --headless --dump-extension-api
#        godot4 --headless --dump-gdextension-interface
#
#    GIT_TAG is pinned to a commit on `master` (not a floating branch name) for
#    build reproducibility — this is the exact commit last verified to build
#    cleanly and pass the picrawler headless smoke test against the 4.6.2 API
#    files above. Re-pin deliberately (and re-verify) rather than letting this
#    silently drift with upstream.
# ---------------------------------------------------------------------------

FetchContent_Declare(
    godot_cpp
    GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
    GIT_TAG        7e18e40d7591429f915035a7de7cf79457d555cc
    # GIT_SHALLOW must stay FALSE while GIT_TAG is a pinned COMMIT.  A shallow
    # clone fetches only the branch tip, so the pin becomes unreachable the
    # moment upstream master moves past it — `fatal: reference is not a tree`,
    # and the build stops being reproducible on any machine that has not
    # already cloned it (found 2026-08-26 on a from-scratch reconfigure).
    GIT_SHALLOW    FALSE
)

# Populate without immediately adding the subdirectory so we can inject
# the 4.6.2 API files before godot-cpp's CMakeLists runs.
FetchContent_GetProperties(godot_cpp)
if(NOT godot_cpp_POPULATED)
    FetchContent_Populate(godot_cpp)

    # Override the bundled 4.5 API with the files from the actual Godot 4.6.2.
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/extension_api.json
         DESTINATION ${godot_cpp_SOURCE_DIR}/gdextension/)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/gdextension_interface.h
         DESTINATION ${godot_cpp_SOURCE_DIR}/gdextension/)
endif()

set(GODOT_CPP_SYSTEM_HEADERS   ON  CACHE BOOL "" FORCE)
set(GODOT_CPP_STATIC           ON  CACHE BOOL "" FORCE)
set(GODOT_CPP_WARNING_AS_ERROR OFF CACHE BOOL "" FORCE)

# Now add the subdirectory — godot-cpp will use our patched API files.
add_subdirectory(${godot_cpp_SOURCE_DIR} ${godot_cpp_BINARY_DIR})

# ---------------------------------------------------------------------------
# 3. GDExtension shared library
#
#   ami_ogma_host.so — loaded by Godot at runtime.
#   Embeds: godot-cpp (static) + ogma_core (static) + ogma_v3 (static).
#   One file to deploy; no rpath magic needed.
# ---------------------------------------------------------------------------

add_library(ami_ogma_host SHARED
    src/AmiOgmaPlugin.cpp
    src/OgmaBrain.cpp
)

# No "lib" prefix — we reference it as "ami_ogma_host.so" in the .gdextension.
set_target_properties(ami_ogma_host PROPERTIES PREFIX "")

target_include_directories(ami_ogma_host PRIVATE
    ../cpp_core/include
    ${eigen_SOURCE_DIR}
    ${json_SOURCE_DIR}/include
)

target_link_libraries(ami_ogma_host PRIVATE
    godot-cpp   # provides all Godot binding headers + implementations
    ogma_core   # statically embedded into the .so
    ogma_infra  # ControlServer (W2 inspector control socket)
)

# Copy the built .so into the Godot addons folder so the editor finds it.
set(ADDON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/project/addons/ami_ogma)

add_custom_command(TARGET ami_ogma_host POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:ami_ogma_host>
        "${ADDON_DIR}/ami_ogma_host.so"
    COMMENT "Copying ami_ogma_host.so → project/addons/ami_ogma/"
)
